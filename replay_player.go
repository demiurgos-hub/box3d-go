// Port of src/replay.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"unsafe"
)

// PORT: third part of replay.c (see replay.go): the public entry point and the b3RecPlayer
// implementation, with the public player types of box3d.h (b3RecPlayerInfo, b3RecQueryType,
// b3RecQueryInfo, b3RecQueryHit). The b3RecPlayer_* functions are methods on *RecPlayer; like the C
// functions they accept a nil player. size_t is uint64.

// RecPlayerInfo is b3RecPlayerInfo.
// Summary of a recording, read once at open so a viewer can frame and label it.
type RecPlayerInfo struct {
	FrameCount   int32   // total recorded steps
	WorkerCount  int32   // worker count requested for the replay world
	TimeStep     float32 // dt of the recorded steps
	SubStepCount int32   // recorded sub-steps
	LengthScale  float32 // length units per meter in effect when recorded
	Bounds       AABB    // accumulated world bounds over the recording, zero-extent if unavailable
}

// RecQueryType is b3RecQueryType.
// The kind of a recorded spatial query, matching the public query and cast functions.
type RecQueryType int32

const (
	RecQueryOverlapAABB    RecQueryType = iota // b3_recQueryOverlapAABB
	RecQueryOverlapShape                       // b3_recQueryOverlapShape
	RecQueryCastRay                            // b3_recQueryCastRay
	RecQueryCastShape                          // b3_recQueryCastShape
	RecQueryCastRayClosest                     // b3_recQueryCastRayClosest
	RecQueryCastMover                          // b3_recQueryCastMover
	RecQueryCollideMover                       // b3_recQueryCollideMover
)

// RecQueryInfo is b3RecQueryInfo.
// A spatial query recorded during a replayed frame, exposed for inspection.
// PORT: const char* name is a string; empty stands for NULL.
type RecQueryInfo struct {
	Type        RecQueryType
	Filter      QueryFilter
	AABB        AABB   // world-space bounds of the query, swept for casts
	Origin      Pos    // query origin (zero for overlap AABB)
	Translation Vec3   // ray and cast translation
	HitCount    int32  // number of recorded results
	Key         uint64 // identity key, the hash of (id, name), 0 if untagged
	Id          uint64 // query id, 0 if none
	Name        string // query label, NULL if none
}

// RecQueryHit is b3RecQueryHit.
// One result of a recorded spatial query.
type RecQueryHit struct {
	Shape    ShapeID
	Point    Pos
	Normal   Vec3
	Fraction float32
}

// Public entry point

// ValidateReplay is b3ValidateReplay.
// Replay a recording from memory and verify it reproduces the same world-state hashes.
// Stands up a fresh world, restores the seed snapshot, replays every op, and checks each embedded
// StateHash record. Returns true if replay completed without id mismatches or hash divergences.
// PORT: const void* data + int size is a slice.
func ValidateReplay(data []byte, workerCount int) bool {
	player := CreatePlayer(data, workerCount)
	if player == nil {
		return false
	}

	for player.StepFrame() {
		if player.rdr.diverged {
			break
		}
	}

	ok := player.rdr.ok && player.rdr.diverged == false
	player.Destroy()
	return ok
}

// b3RecPlayer implementation

const (
	recKeyframeIntervalDefault = 16                        // B3_REC_KEYFRAME_INTERVAL_DEFAULT
	recKeyframeBudgetDefault   = uint64(512 * 1024 * 1024) // B3_REC_KEYFRAME_BUDGET_DEFAULT
)

// recGrow is b3RecGrow.
// Overflow-safe growth for the player's accumulating arrays. Counts come from the replay itself,
// not the file, so this only guards the byte-size multiply. Preserves keep elements.
// PORT: generic over the slice; its length is the capacity. Go sizes cannot overflow here.
func recGrow[T any](data *[]T, need int, keep int) {
	capacity := len(*data)
	if need <= capacity {
		return
	}
	newCap := 2 * capacity
	if capacity == 0 {
		newCap = 8
	}
	if newCap < need {
		newCap = need
	}
	grown := make([]T, newCap)
	if *data != nil {
		if keep > 0 {
			copy(grown, (*data)[:keep])
		}
	}
	*data = grown
}

// Block B: per-frame query store helpers. Forward declared above the query dispatchers, which run
// before the player struct is defined, so the player-dereferencing code lives here.

// recGrowFrameQueries is b3RecGrowFrameQueries.
func recGrowFrameQueries(player *RecPlayer) {
	recGrow(&player.frameQueries, player.frameQueryCount+1, player.frameQueryCount)
}

// recGrowFrameHits is b3RecGrowFrameHits.
func recGrowFrameHits(player *RecPlayer, need int) {
	recGrow(&player.frameHits, player.frameHitCount+need, player.frameHitCount)
}

// recStashQueryBegin is b3RecStashQueryBegin.
// Push a draw record for one query and copy its hits into the per-frame store. Ids in hits[] are
// already remapped to the replay world by the dispatcher.
func recStashQueryBegin(player *RecPlayer, kind int32, hits []recRecordedHit, hitCount int) *recDrawQuery {
	recGrowFrameQueries(player)
	q := &player.frameQueries[player.frameQueryCount]
	*q = recDrawQuery{}
	q.kind = kind
	// Pair the query with the key from its preceding QueryTag op, if any, then clear it so the next
	// untagged query reads 0.
	q.key = player.rdr.pendingQueryKey
	player.rdr.pendingQueryKey = 0
	q.hitStart = int32(player.frameHitCount)
	q.hitCount = int32(hitCount)
	recGrowFrameHits(player, hitCount)
	for i := 0; i < hitCount; i++ {
		player.frameHits[player.frameHitCount+i] = hits[i]
	}
	player.frameHitCount += hitCount
	player.frameQueryCount++
	return q
}

// recTrackBodyCreate is b3RecTrackBodyCreate.
// Append a created body to the outliner list. Ordinals are creation order and never reused.
func recTrackBodyCreate(player *RecPlayer, id BodyID) {
	recGrow(&player.bodyIds, player.bodyIdCount+1, player.bodyIdCount)
	player.bodyIds[player.bodyIdCount] = id
	player.bodyIdCount += 1
}

// recTrackBodyDestroy is b3RecTrackBodyDestroy.
// Leave a hole so later ordinals do not shift, keeping a stored selection stable.
func recTrackBodyDestroy(player *RecPlayer, id BodyID) {
	for i := 0; i < player.bodyIdCount; i++ {
		if player.bodyIds[i] == id {
			player.bodyIds[i] = NullBodyID
			return
		}
	}
}

// recSeedBodyIds is b3RecSeedBodyIds.
// Snapshot bodies are restored as a struct image and never hit the CreateBody hook the tracker keys
// on, so the seed world must be walked once to populate the outliner list. Slot order is stable.
func recSeedBodyIds(player *RecPlayer) {
	world := getWorldFromId(player.rdr.replayWorldId)
	player.bodyIdCount = 0
	count := len(world.bodies)
	for i := 0; i < count; i++ {
		if int(world.bodies[i].id) != i {
			continue // free slot
		}
		recTrackBodyCreate(player, makeBodyId(world, i))
	}
}

// recSeedFrame0BodyIds is b3RecSeedFrame0BodyIds.
// Seed the outliner list from the current world and save it as the frame-0 restore copy.
func recSeedFrame0BodyIds(player *RecPlayer) {
	recSeedBodyIds(player)
	if player.frame0BodyIds != nil {
		player.frame0BodyIds = nil
	}
	player.frame0BodyIdCount = player.bodyIdCount
	if player.bodyIdCount > 0 {
		player.frame0BodyIds = make([]BodyID, player.bodyIdCount)
		copy(player.frame0BodyIds, player.bodyIds[:player.bodyIdCount])
	}
}

// Tag key to tag index, so the viewer resolves a query's caller id and label in O(1) instead of a
// linear scan over the tag table.
// PORT: the verstable b3RecTagLookup (uint64 -> uint32) is a Go map; it is only looked up, never
// iterated.

// recLoadTags is b3RecLoadTags.
// Read the optional query-tag table trailing the geometry entries: u32 tagCount then per tag
// { u64 key, u64 id, u16 len, name bytes }. A recording written before the tag table leaves rp at
// dataEnd, so nothing loads. Bounds-checked; tagCount reflects only the tags that fully fit, so a
// truncated tail loads what it can and reports the real count.
// PORT: rp and dataEnd are byte offsets into data.
func recLoadTags(rdr *recReader, data []byte, rp int, dataEnd int) {
	var sub recReader
	sub.data = data[rp:]
	sub.size = dataEnd - rp
	sub.ok = true

	count := recRU32(&sub)
	if sub.ok == false || count == 0 {
		return
	}

	// Each tag is at least 18 bytes (8 key + 8 id + 2 length). Reject a count that cannot fit the
	// remaining bytes so a corrupt table cannot request a wild allocation.
	if uint64(count) > uint64(sub.size-sub.cursor)/18 {
		return
	}

	tags := make([]recTag, count)

	tagMap := make(map[uint64]uint32)

	loaded := uint32(0)
	for i := uint32(0); i < count; i++ {
		key := recRU64(&sub)
		id := recRU64(&sub)
		length := recRU16(&sub)
		if sub.ok == false {
			break
		}
		if length == 0xFFFF {
			length = 0 // a null name is written as 0xFFFF
		}
		if int64(sub.cursor)+int64(length) > int64(sub.size) {
			break
		}
		n := int(length)
		if length > maxQueryNameLength {
			n = maxQueryNameLength
		}
		tags[loaded].key = key
		tags[loaded].id = id
		if n > 0 {
			copy(tags[loaded].queryName[:n], sub.data[sub.cursor:sub.cursor+n])
		}
		tags[loaded].queryName[n] = 0
		sub.cursor += int(length)
		if _, exists := tagMap[key]; !exists {
			// PORT: verstable insert keeps the first value for a duplicate key.
			tagMap[key] = loaded
		}
		loaded += 1
	}

	rdr.tags = tags
	rdr.tagCount = int(loaded)
	rdr.tagCapacity = int(count)
	rdr.tagMap = tagMap
}

// recFreeSlots is b3RecFreeSlots.
// Free slots loaded by b3RecLoadSlots.
// PORT: the slot blobs are garbage collected; this only drops the references.
func recFreeSlots(slots []registrySlot, slotCount int) {
	if slots == nil {
		return
	}
	for i := 0; i < slotCount; i++ {
		slots[i].bytes = nil
	}
}

// recLoadSlots is b3RecLoadSlots.
// Load the trailing registry block and fill rdr->slots/slotCount, then the optional tag table.
// Returns true on success. On failure sets rdr->ok = false and returns false.
// PORT: const void* data + int size is a slice with size kept; rp and dataEnd are byte offsets. The
// (int) narrowing of the 64-bit locator is int32 truncation as in C. Each slot's bytes are an
// 8-byte aligned blob (blobAlloc) so the geometry headers can be read in place.
func recLoadSlots(rdr *recReader, data []byte, size int, registryOffset uint64, registryByteCount uint64) bool {
	if registryOffset == 0 || registryByteCount == 0 {
		rdr.slots = nil
		rdr.slotCount = 0
		return true
	}

	regStart := int(int32(registryOffset))
	regEnd := regStart + int(int32(registryByteCount))
	if regEnd > size {
		fmt.Printf("b3ReplayFile: registry block out of bounds\n")
		return false
	}
	if regStart+4 > size {
		fmt.Printf("b3ReplayFile: registry too small\n")
		return false
	}

	dataEnd := regEnd
	rp := regStart
	count := uint32(data[rp]) | (uint32(data[rp+1]) << 8) | (uint32(data[rp+2]) << 16) | (uint32(data[rp+3]) << 24)
	rp += 4

	if count == 0 {
		rdr.slots = nil
		rdr.slotCount = 0
		recLoadTags(rdr, data, rp, dataEnd)
		return true
	}

	// Each entry is at least 5 bytes (kind + 4-byte length). A count that cannot fit the remaining
	// registry bytes is a corrupt header, so reject it before allocating.
	if rp > dataEnd || uint64(count) > uint64(dataEnd-rp)/5 {
		fmt.Printf("b3ReplayFile: registry count out of range\n")
		return false
	}

	slots := make([]registrySlot, count)

	for i := uint32(0); i < count; i++ {
		if rp+5 > dataEnd {
			fmt.Printf("b3ReplayFile: registry truncated at entry %d\n", i)
			recFreeSlots(slots, int(count))
			return false
		}
		kind := data[rp]
		byteCount := uint32(data[rp+1]) | (uint32(data[rp+2]) << 8) | (uint32(data[rp+3]) << 16) | (uint32(data[rp+4]) << 24)
		rp += 5
		errorText := ""
		if byteCount == 0 || byteCount > math.MaxInt32 {
			errorText = "has an invalid byte count"
		} else if uint64(byteCount) > uint64(dataEnd-rp) {
			errorText = "bytes out of bounds"
		} else {
			bytes := blobBytes(blobAlloc(int(byteCount)), int32(byteCount))
			copy(bytes, data[rp:rp+int(byteCount)])
			slots[i].kind = geometryKind(kind)
			slots[i].byteCount = int32(byteCount)
			slots[i].bytes = bytes

			// Validate the compound.
			// todo validate other types?
			if geometryKind(kind) == geometryCompound && ValidateCompound(bytes, int(byteCount)) == nil {
				errorText = "is not a valid compound"
			}
		}
		if errorText != "" {
			fmt.Printf("b3ReplayFile: registry entry %d %s\n", i, errorText)
			recFreeSlots(slots, int(count))
			return false
		}
		rp += int(byteCount)
	}

	rdr.slots = slots
	rdr.slotCount = int(count)
	recLoadTags(rdr, data, rp, dataEnd)
	return true
}

// recScanFile is b3RecScanFile.
// Walk the op stream once without dispatching: count Step ops and grab the first step's tuning.
func recScanFile(player *RecPlayer) {
	data := player.data
	size := player.registryEnd
	cursor := player.headerEnd
	frameCount := 0
	gotStep := false

	u32 := func(i int) uint32 {
		return uint32(data[i]) | (uint32(data[i+1]) << 8) | (uint32(data[i+2]) << 16) | (uint32(data[i+3]) << 24)
	}

	for cursor+4 <= size {
		opcode := data[cursor]
		payloadSize := uint32(data[cursor+1]) | (uint32(data[cursor+2]) << 8) | (uint32(data[cursor+3]) << 16)
		payloadStart := cursor + 4
		if payloadStart+int(payloadSize) > size {
			break
		}
		if opcode == recOpStep {
			frameCount += 1
			if !gotStep && payloadSize >= 12 {
				dtBits := u32(payloadStart + 4)
				player.recordedDt = math.Float32frombits(dtBits)
				player.recordedSubStepCount = int(int32(u32(payloadStart + 8)))
				gotStep = true
			}
		} else if opcode == 0xF2 && payloadSize >= 24 { // RecordingBounds
			// Payload is a single b3AABB (lower xyz, upper xyz as f32), written at stop so the
			// viewer can frame the whole recorded motion without playing to the end.
			f := func(k int) float32 { return math.Float32frombits(u32(payloadStart + 4*k)) }
			player.bounds = AABB{Vec3{f(0), f(1), f(2)}, Vec3{f(3), f(4), f(5)}}
		}
		cursor = payloadStart + int(payloadSize)
	}
	player.frameCount = frameCount
}

// freeKeyframe is b3FreeKeyframe.
// Free one keyframe's heap.
// PORT: garbage collected; this only drops the references.
func freeKeyframe(kf *recKeyframe) {
	kf.image = nil
	kf.bodyIds = nil
}

// recSeedKeyframeRegistry is b3RecSeedKeyframeRegistry.
// Pre-populate keyframeRec's registry to mirror rdr.slots so geometry ids stay stable during
// b3SerializeWorld. Each slot becomes one entry with id == slot index, even byte-identical slots that
// a hash collision left undeduplicated in an already-recorded file. b3SerializeWorld then resolves a
// live blob back to a valid slot index via the registry's exact dedup, so capture never grows it.
func recSeedKeyframeRegistry(player *RecPlayer) {
	reg := &player.keyframeRec.registry
	for i := 0; i < player.rdr.slotCount; i++ {
		slot := &player.rdr.slots[i]
		h := hash64NonZero(slot.bytes[:slot.byteCount])
		id := appendGeometry(reg, slot.kind, h, slot.bytes[:slot.byteCount])
		// Seeding in order without dedup keeps id == slot index.
		assert(id == uint32(i), "id == (uint32_t)i")
		_ = id
	}
}

// recCaptureKeyframe is b3RecCaptureKeyframe.
// Capture a restore-point keyframe for the just-completed frame. rdr.cursor already points to
// the next frame's Step op.
func recCaptureKeyframe(player *RecPlayer) {
	world := getWorldFromId(player.rdr.replayWorldId)
	var buf recBuffer

	regCountBefore := len(player.keyframeRec.registry.entries)

	serializeWorld(world, &buf, player.keyframeRec)
	// Registry must not grow: all geometry was pre-seeded and the registry dedups exactly.
	assert(len(player.keyframeRec.registry.entries) == regCountBefore,
		"player->keyframeRec->registry.entries.count == regCountBefore")

	bodyBytes := uint64(player.bodyIdCount) * uint64(unsafe.Sizeof(BodyID{}))
	newBytes := uint64(buf.capacity) + bodyBytes

	// Make room under the budget by doubling the spacing and evicting off-grid keyframes.
	for player.keyframeCount > 0 && player.keyframeBytes+newBytes > player.keyframeBudget {
		player.keyframeInterval *= 2
		kept := 0
		keptBytes := uint64(0)
		for i := 0; i < player.keyframeCount; i++ {
			kf := &player.keyframes[i]
			if kf.frame%player.keyframeInterval == 0 {
				player.keyframes[kept] = *kf
				keptBytes += uint64(kf.imageCapacity) + uint64(kf.bodyIdCount)*uint64(unsafe.Sizeof(BodyID{}))
				kept += 1
			} else {
				freeKeyframe(kf)
			}
		}
		progress := kept < player.keyframeCount
		player.keyframeCount = kept
		player.keyframeBytes = keptBytes
		if !progress {
			break
		}
	}

	// Grow the keyframe ring if needed.
	if player.keyframeCount >= len(player.keyframes) {
		newCap := len(player.keyframes) * 2
		if len(player.keyframes) < 8 {
			newCap = 8
		}
		grown := make([]recKeyframe, newCap)
		copy(grown, player.keyframes)
		player.keyframes = grown
	}

	kf := &player.keyframes[player.keyframeCount]
	kf.image = buf.data
	kf.imageSize = int(buf.size)
	kf.imageCapacity = int(buf.capacity)
	kf.frame = player.frame
	kf.cursor = player.rdr.cursor
	kf.divergeFrame = player.divergeFrame
	kf.diverged = player.rdr.diverged
	kf.bodyIdCount = player.bodyIdCount
	kf.bodyIds = nil
	if bodyBytes > 0 {
		kf.bodyIds = make([]BodyID, player.bodyIdCount)
		copy(kf.bodyIds, player.bodyIds[:player.bodyIdCount])
	}

	player.keyframeBytes += newBytes
	player.keyframeCount += 1
	player.lastKeyframeFrame = player.frame
}

// recPlayerRestoreKeyframe is b3RecPlayerRestoreKeyframe.
// Restore the world in-place from a keyframe image.
func recPlayerRestoreKeyframe(player *RecPlayer, kf *recKeyframe) {
	world := getWorldFromId(player.rdr.replayWorldId)
	if deserializeIntoShell(kf.image[:kf.imageSize], world, &player.rdr) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = kf.cursor
	player.rdr.ok = true
	player.rdr.diverged = kf.diverged
	player.frame = kf.frame
	player.divergeFrame = kf.divergeFrame
	player.atEnd = false
	player.atPreStep = false

	// Restore the outliner list verbatim so ordinals match this frame.
	recGrow(&player.bodyIds, kf.bodyIdCount, 0)
	player.bodyIdCount = kf.bodyIdCount
	if kf.bodyIdCount > 0 {
		copy(player.bodyIds, kf.bodyIds[:kf.bodyIdCount])
	}
}

// recPlayerCreateWorld is b3RecPlayerCreateWorld.
// Create a replay world carrying the host debug-shape callbacks. Every world the player
// stands up funnels through here so the sample renderer can draw replayed shapes.
func recPlayerCreateWorld(player *RecPlayer) WorldID {
	worldDef := DefaultWorldDef()
	worldDef.CreateDebugShape = player.createDebugShape
	worldDef.DestroyDebugShape = player.destroyDebugShape
	worldDef.UserDebugShapeContext = player.debugShapeContext
	// Carry the requested worker count so a rebuild on Restart or backward seek keeps the same
	// graph partitioning. Replaying at a different count than recorded is a determinism check.
	worldDef.WorkerCount = uint32(MaxInt(1, player.recordedWorkerCount))
	return CreateWorld(&worldDef)
}

// recForeignSeed holds the world scalars of a seed snapshot written by another build (see
// recDeserializeSeed).
// PORT: Go addition, not upstream.
type recForeignSeed struct {
	gravity                      Vec3
	hitEventThreshold            float32
	restitutionThreshold         float32
	restitutionIterations        int32
	maxLinearSpeed               float32
	isotropicInertiaTolerance    float32
	contactSpeed                 float32
	contactHertz                 float32
	contactDampingRatio          float32
	contactRecycleDistance       float32
	stepIndex                    uint64
	splitIslandId                int32
	inv_h                        float32
	inv_dt                       float32
	endEventArrayIndex           int32
	compoundShapeCount           int32
	maxCapacity                  Capacity
	enableSleep                  bool
	enableWarmStarting           bool
	enableContinuous             bool
	enableSpeculative            bool
	enableRestitutionPropagation bool
}

// Snapshot image header constants of world_snapshot.c (B3_SNAP_MAGIC, B3_SNAP_VERSION,
// B3_SNAP_FLAG_DOUBLE_PRECISION), read by recReadForeignEmptySeed.
// PORT: Go addition; world_snapshot.go owns the snapshot codec.
const (
	recSeedSnapMagic           uint32 = 0x33534E42
	recSeedSnapVersion         uint32 = snapVersion
	recSeedSnapFlagDoublePrecs uint32 = 0x2
)

// recReadForeignEmptySeed decodes the seed snapshot of a recording made by another build (the C
// library) when that seed is an empty world: a world that never held a body, shape, contact, joint
// or island. Such a seed carries only the world scalars, which are written field by field ahead of
// any layout-dependent struct image, so they decode without the other build's struct layout.
// Returns false for anything else (bad magic or version, double precision, a non-empty world).
// PORT: Go addition, not upstream. The design keeps snapshots Go-native (a C image fails the Go
// layout hash), yet a recording made by the C library must replay in Go; every recording is
// snapshot-seeded, so a C recording started on an empty world restores through this path.
func recReadForeignEmptySeed(data []byte) (recForeignSeed, bool) {
	var seed recForeignSeed
	var r recReader
	r.data = data
	r.size = len(data)
	r.ok = true

	magic := recRU32(&r)
	version := recRU32(&r)
	_ = recRU32(&r) // layout hash of the writing build
	flags := recRU32(&r)
	if !r.ok || magic != recSeedSnapMagic || version != recSeedSnapVersion || (flags&recSeedSnapFlagDoublePrecs) != 0 {
		return seed, false
	}

	// b3SerWorldConfig
	seed.gravity = recRVEC3(&r)
	seed.hitEventThreshold = recRF32(&r)
	seed.restitutionThreshold = recRF32(&r)
	seed.restitutionIterations = recRI32(&r)
	seed.maxLinearSpeed = recRF32(&r)
	seed.isotropicInertiaTolerance = recRF32(&r)
	seed.contactSpeed = recRF32(&r)
	seed.contactHertz = recRF32(&r)
	seed.contactDampingRatio = recRF32(&r)
	seed.contactRecycleDistance = recRF32(&r)
	seed.stepIndex = recRU64(&r)
	seed.splitIslandId = recRI32(&r)
	seed.inv_h = recRF32(&r)
	seed.inv_dt = recRF32(&r)
	seed.endEventArrayIndex = recRI32(&r)
	seed.compoundShapeCount = recRI32(&r)
	seed.maxCapacity.StaticShapeCount = recRI32(&r)
	seed.maxCapacity.DynamicShapeCount = recRI32(&r)
	seed.maxCapacity.StaticBodyCount = recRI32(&r)
	seed.maxCapacity.DynamicBodyCount = recRI32(&r)
	seed.maxCapacity.ContactCount = recRI32(&r)
	worldFlags := recRU8(&r)
	seed.enableSleep = (worldFlags & 0x01) != 0
	seed.enableWarmStarting = (worldFlags & 0x02) != 0
	seed.enableContinuous = (worldFlags & 0x04) != 0
	seed.enableSpeculative = (worldFlags & 0x08) != 0
	seed.enableRestitutionPropagation = (worldFlags & 0x10) != 0

	// b3SerIdPool for the body, shape, contact, joint and island pools (the solver set pool follows
	// and is not needed): all must be untouched.
	for i := 0; i < 5; i++ {
		nextIndex := recRI32(&r)
		freeCount := recRI32(&r)
		if !r.ok || nextIndex != 0 || freeCount != 0 {
			return seed, false
		}
	}

	return seed, r.ok
}

// recWorldIsEmpty reports whether a world never held a body, shape, contact, joint or island.
// PORT: Go addition, not upstream (see recDeserializeSeed).
func recWorldIsEmpty(world *world) bool {
	return world.bodyIdPool.nextIndex == 0 && world.shapeIdPool.nextIndex == 0 && world.contactIdPool.nextIndex == 0 &&
		world.jointIdPool.nextIndex == 0 && world.islandIdPool.nextIndex == 0
}

// recApplyForeignSeed applies the world scalars of a foreign empty-world seed to an empty world.
// PORT: Go addition, not upstream (see recDeserializeSeed); the fields are those of
// b3DesWorldConfig.
func recApplyForeignSeed(world *world, seed *recForeignSeed) {
	world.gravity = seed.gravity
	world.hitEventThreshold = seed.hitEventThreshold
	world.restitutionThreshold = seed.restitutionThreshold
	world.restitutionIterations = seed.restitutionIterations
	world.maxLinearSpeed = seed.maxLinearSpeed
	world.isotropicInertiaTolerance = seed.isotropicInertiaTolerance
	world.contactSpeed = seed.contactSpeed
	world.contactHertz = seed.contactHertz
	world.contactDampingRatio = seed.contactDampingRatio
	world.contactRecycleDistance = seed.contactRecycleDistance
	world.stepIndex = seed.stepIndex
	world.splitIslandId = seed.splitIslandId
	world.inv_h = seed.inv_h
	world.inv_dt = seed.inv_dt
	world.endEventArrayIndex = seed.endEventArrayIndex
	world.compoundShapeCount = seed.compoundShapeCount
	world.maxCapacity = seed.maxCapacity
	world.enableSleep = seed.enableSleep
	world.enableWarmStarting = seed.enableWarmStarting
	world.enableContinuous = seed.enableContinuous
	world.enableSpeculative = seed.enableSpeculative
	world.enableRestitutionPropagation = seed.enableRestitutionPropagation
}

// recDeserializeSeed restores the seed snapshot (the frame-0 image) into the replay world. A Go
// image goes through b3DeserializeIntoShell in place. A foreign empty-world seed (a C recording,
// see recReadForeignEmptySeed) is restored by applying its world scalars to an empty world; when
// the replay world is no longer empty (Restart, a backward seek to frame 0, a debug-shape rebuild)
// it is replaced by a fresh one first, so the replay world id changes for such recordings.
// PORT: Go addition, not upstream; called where upstream calls b3DeserializeIntoShell on the
// seed image (b3CreatePlayer, b3RecPlayer_Restart, b3RecPlayer_SetDebugShapeCallbacks).
func recDeserializeSeed(player *RecPlayer, data []byte) bool {
	world := getWorldFromId(player.rdr.replayWorldId)
	if deserializeIntoShell(data, world, &player.rdr) {
		return true
	}

	seed, ok := recReadForeignEmptySeed(data)
	if !ok {
		return false
	}

	if recWorldIsEmpty(world) == false {
		player.rdr.replayWorldId.Destroy()
		player.rdr.replayWorldId = recPlayerCreateWorld(player)
		world = getWorldFromId(player.rdr.replayWorldId)
	}

	recApplyForeignSeed(world, &seed)
	return true
}

// recHeaderSize is sizeof( b3RecHeader ).
const recHeaderSize = 48

// recDecodeHeader decodes the 48 byte little-endian b3RecHeader image (the memcpy of the header in
// b3CreatePlayer).
// PORT: helper; Go cannot memcpy into a struct.
func recDecodeHeader(data []byte) recHeader {
	var r recReader
	r.data = data
	r.size = recHeaderSize
	r.ok = true
	var hdr recHeader
	hdr.magic = recRU32(&r)
	hdr.versionMajor = recRU16(&r)
	hdr.versionMinor = recRU16(&r)
	hdr.pointerWidth = recRU8(&r)
	hdr.bigEndian = recRU8(&r)
	hdr.validationEnabled = recRU8(&r)
	hdr.reserved = recRU8(&r)
	hdr.lengthScale = recRF32(&r)
	hdr.reserved2 = recRU32(&r)
	hdr.reserved3 = recRU32(&r)
	hdr.snapshotSize = recRU64(&r)
	hdr.registryOffset = recRU64(&r)
	hdr.registryByteCount = recRU64(&r)
	return hdr
}

// recPointerWidth is the sizeof( void* ) the op stream's POD def layout is pinned to.
// PORT: the design fixes the op stream at pointer width 8 (C struct layout with zeroed 8 byte
// pointer slots) on every Go platform.
const recPointerWidth = 8

// CreatePlayer is b3CreatePlayer.
// Create a player over a recording. Owns a private copy of the bytes.
// Returns a new player, or nil on bad header or deserialization failure.
// PORT: const void* data + int size is a slice. The seed snapshot is restored with
// recDeserializeSeed, which also accepts a C recording started on an empty world.
func CreatePlayer(data []byte, workerCount int) *RecPlayer {
	size := len(data)
	if data == nil || size < recHeaderSize {
		fmt.Printf("b3RecPlayer_Create: recording too small\n")
		return nil
	}

	hdr := recDecodeHeader(data)

	if hdr.magic != recMagic {
		fmt.Printf("b3RecPlayer_Create: bad magic 0x%08X\n", hdr.magic)
		return nil
	}
	// Only the major version is breaking. Minor bumps are additive op-stream changes that keep the
	// header shape, and the dispatcher skips opcodes it doesn't know, so a minor mismatch still loads.
	if hdr.versionMajor != recVersionMajor {
		fmt.Printf("b3RecPlayer_Create: version mismatch %d.%d vs %d.%d\n", hdr.versionMajor, hdr.versionMinor, recVersionMajor,
			recVersionMinor)
		return nil
	}
	if hdr.pointerWidth != recPointerWidth {
		fmt.Printf("b3RecPlayer_Create: pointer width mismatch %d vs %d\n", hdr.pointerWidth, recPointerWidth)
		return nil
	}
	if hdr.bigEndian != 0 {
		fmt.Printf("b3RecPlayer_Create: big-endian recording not supported\n")
		return nil
	}

	// Every recording is snapshot-seeded: the seed blob sits between the header and the op stream.
	if hdr.snapshotSize == 0 {
		fmt.Printf("b3RecPlayer_Create: missing snapshot seed\n")
		return nil
	}

	// snapshotSize and registryOffset are 64-bit and come from the file. Validate in 64-bit so a
	// hostile value can't wrap when narrowed to int, then narrow once the bounds are known good.
	headerEnd64 := uint64(recHeaderSize) + hdr.snapshotSize
	registryEnd64 := uint64(size)
	if hdr.registryOffset != 0 {
		registryEnd64 = hdr.registryOffset
	}

	if headerEnd64 < recHeaderSize || headerEnd64 > registryEnd64 || registryEnd64 > uint64(size) {
		fmt.Printf("b3RecPlayer_Create: corrupt offsets\n")
		return nil
	}

	headerEnd := int(headerEnd64)
	registryEnd := int(registryEnd64)

	// Own a private copy so the caller can free their buffer right away.
	dataCopy := make([]byte, size)
	copy(dataCopy, data)

	player := &RecPlayer{}

	player.data = dataCopy
	player.size = size
	player.headerEnd = headerEnd
	player.registryEnd = registryEnd
	player.lengthScale = hdr.lengthScale
	player.previousLengthScale = GetLengthUnitsPerMeter()
	player.frame = 0
	player.frameCount = 0
	player.recordedDt = 0.0
	player.recordedSubStepCount = 0
	player.recordedWorkerCount = workerCount
	player.atEnd = false
	player.atPreStep = false
	player.divergeFrame = -1
	player.keyframeMinInterval = recKeyframeIntervalDefault
	player.keyframeInterval = recKeyframeIntervalDefault
	player.keyframeBudget = recKeyframeBudgetDefault
	player.lastKeyframeFrame = 0

	// Set length scale so replay reproduces the same tuning constants.
	if hdr.lengthScale > 0.0 {
		SetLengthUnitsPerMeter(hdr.lengthScale)
	}

	// Count frames and read first step's dt so the viewer can show hz up front.
	recScanFile(player)

	// Create the replay world. Debug-shape callbacks are NULL here; the sample wires
	// them right after Create via b3RecPlayer_SetDebugShapeCallbacks, which rebuilds.
	worldId := recPlayerCreateWorld(player)

	// Initialize the reader.
	player.rdr.data = dataCopy
	player.rdr.size = size
	player.rdr.cursor = headerEnd
	player.rdr.replayWorldId = worldId
	player.rdr.ok = true
	player.rdr.diverged = false
	player.rdr.owner = player

	// Load the trailing geometry registry.
	if !recLoadSlots(&player.rdr, dataCopy, size, hdr.registryOffset, hdr.registryByteCount) {
		worldId.Destroy()
		// PORT: upstream frees the copy and the player; garbage collected here. The global length
		// scale stays as set above, as upstream.
		return nil
	}

	// Restore the seed snapshot to stand up the replay world. The blob doubles as the frame-0
	// restore image, owned by the copy held above.
	{
		snapStart := recHeaderSize
		snapSize := int(hdr.snapshotSize)
		if recDeserializeSeed(player, dataCopy[snapStart:snapStart+snapSize]) == false {
			fmt.Printf("b3RecPlayer_Create: snapshot deserialization failed\n")
			player.rdr.replayWorldId.Destroy()
			recFreeSlots(player.rdr.slots, player.rdr.slotCount)
			player.rdr.tags = nil
			player.rdr.tagMap = nil
			return nil
		}
		player.rdr.cursor = headerEnd
		player.frame0Image = dataCopy[snapStart : snapStart+snapSize]
		player.frame0Size = snapSize
	}

	// Seed the outliner from the restored world (snapshot bodies bypass the create hook) and save
	// the frame-0 restore copy.
	recSeedFrame0BodyIds(player)

	// Build the keyframe recording with a pre-seeded registry that mirrors rdr.slots,
	// so b3SerializeWorld geometry ids stay stable across captures.
	player.keyframeRec = CreateRecording(0)
	recSeedKeyframeRegistry(player)

	return player
}

// Destroy is b3DestroyPlayer.
// Destroy the player and free all memory. Restores the previous global length scale.
// PORT: memory is garbage collected; the world is destroyed and the length scale restored as
// upstream.
func (player *RecPlayer) Destroy() {
	if player == nil {
		return
	}

	if player.rdr.replayWorldId.IsValid() {
		player.rdr.replayWorldId.Destroy()
	}

	// Free shape data.
	recFreeSlots(player.rdr.slots, player.rdr.slotCount)

	// Free reader scratch.
	player.rdr.matScratch = nil
	player.rdr.proxyScratch = nil
	player.rdr.hits = nil
	player.rdr.tags = nil
	player.rdr.tagMap = nil

	// Free the per-frame query store.
	player.frameQueries = nil
	player.frameHits = nil

	// Free keyframe ring.
	for i := 0; i < player.keyframeCount; i++ {
		freeKeyframe(&player.keyframes[i])
	}
	player.keyframes = nil

	// The keyframe recording owns only its buffer and registry; b3DestroyRecording frees both.
	if player.keyframeRec != nil {
		player.keyframeRec.Destroy()
	}

	// Free the outliner body lists.
	player.bodyIds = nil
	player.frame0BodyIds = nil

	// frame0Image points into the owned data copy, not separately allocated.

	player.data = nil

	// Restore the global length scale.
	SetLengthUnitsPerMeter(player.previousLengthScale)
}

// StepFrame is b3RecPlayer_StepFrame.
// Advance one frame. dispatch ops until the next Step completes.
// Returns true when a frame was stepped, false at end-of-recording.
func (player *RecPlayer) StepFrame() bool {
	// This is never true when full stepping
	player.atPreStep = false

	if player.atEnd {
		return false
	}

	// Reset the per-frame query store before this frame's records are dispatched.
	player.frameQueryCount = 0
	player.frameHitCount = 0

	// A frame is its leading inputs (queries and between-step mutators), one Step, and the Step's
	// trailing StateHash. Queries are recorded before the Step they belong to, so they stash here
	// against the world state they were computed for.
	stepped := false
	for {
		if player.rdr.cursor >= player.registryEnd || !player.rdr.ok {
			player.atEnd = true
			return stepped
		}

		// Once stepped, the StateHash is the only record still belonging to this frame. Anything else
		// begins the next frame, so stop and let the next StepFrame consume it. Capture a keyframe at
		// the boundary.
		if stepped && player.rdr.data[player.rdr.cursor] != recOpStateHash {
			if player.frame > player.lastKeyframeFrame && player.frame%player.keyframeInterval == 0 {
				recCaptureKeyframe(player)
			}
			return true
		}

		op := recDispatchOne(&player.rdr)
		if op < 0 {
			player.atEnd = true
			return stepped
		}
		if op == recOpDestroyWorld { // end of recording
			player.atEnd = true
			return stepped
		}
		if op == recOpStep {
			player.frame += 1
			stepped = true
		} else if op == recOpStateHash { // trailing record of the frame just stepped
			// Latch the first frame whose state hash diverged. The hash belongs to the frame Step just
			// advanced, so latch against the current frame, not the next Step which would be one late.
			if player.divergeFrame < 0 && player.rdr.diverged {
				player.divergeFrame = player.frame
			}
		}
	}
}

// SubStepFrame is b3RecPlayer_SubStepFrame.
// Sub-step one frame. This will sub-step and return immediately after body creation.
// The next call will execute the time step. This allows bodies to be rendered
// at the creation pose.
func (player *RecPlayer) SubStepFrame() {
	if player.atEnd {
		return
	}

	// Reset the per-frame query store before this frame's records are dispatched.
	if player.atPreStep == false {
		player.frameQueryCount = 0
		player.frameHitCount = 0
	}

	// A frame is its leading inputs (queries and between-step mutators), one Step, and the Step's
	// trailing StateHash. Queries are recorded before the Step they belong to, so they stash here
	// against the world state they were computed for.
	stepped := false
	haveCreateBodyOp := false
	for {
		if player.rdr.cursor >= player.registryEnd || !player.rdr.ok {
			player.atEnd = true
			player.atPreStep = false
			return
		}

		// Once stepped, the StateHash is the only record still belonging to this frame. Anything else
		// begins the next frame, so stop and let the next StepFrame consume it. Capture a keyframe at
		// the boundary.
		currentOpCode := player.rdr.data[player.rdr.cursor]
		if stepped && currentOpCode != recOpStateHash {
			if player.frame > player.lastKeyframeFrame && player.frame%player.keyframeInterval == 0 {
				recCaptureKeyframe(player)
			}
			return
		}

		if player.atPreStep == false && haveCreateBodyOp == true && currentOpCode == recOpStep {
			player.atPreStep = true
			return
		}

		op := recDispatchOne(&player.rdr)
		if op < 0 {
			player.atEnd = true
			player.atPreStep = false
			return
		}
		if op == recOpDestroyWorld { // end of recording
			player.atEnd = true
			player.atPreStep = false
			return
		}

		if op == recOpCreateBody {
			assert(player.atPreStep == false, "player->atPreStep == false")
			haveCreateBodyOp = true
		}

		if op == recOpStep {
			player.atPreStep = false
			player.frame += 1
			stepped = true
		} else if op == recOpStateHash { // trailing record of the frame just stepped
			// Latch the first frame whose state hash diverged. The hash belongs to the frame Step just
			// advanced, so latch against the current frame, not the next Step which would be one late.
			if player.divergeFrame < 0 && player.rdr.diverged {
				player.divergeFrame = player.frame
			}
		}
	}
}

// Restart is b3RecPlayer_Restart.
// Rewind to frame 0 (in-place restore so the world id stays stable).
// PORT: a C recording's seed (see recDeserializeSeed) rebuilds the world, so its id changes.
func (player *RecPlayer) Restart() {
	// Restore the frame-0 image in place so the replay world id stays stable across a restart or
	// backward scrub. Stepping resumes at the first Step, which rebuilds the body list deterministically.
	if recDeserializeSeed(player, player.frame0Image[:player.frame0Size]) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = player.headerEnd
	player.rdr.ok = true
	player.rdr.diverged = false
	player.frame = 0
	player.divergeFrame = -1
	player.atEnd = false
	player.atPreStep = false

	// Frame 0 is the pre-step snapshot with no recorded queries, so clear the per-frame store. This
	// keeps the last stepped frame's queries from lingering on a restart or a backward scrub to 0.
	player.frameQueryCount = 0
	player.frameHitCount = 0

	// Roll the outliner body list back to its frame-0 contents.
	recGrow(&player.bodyIds, player.frame0BodyIdCount, 0)
	player.bodyIdCount = player.frame0BodyIdCount
	if player.frame0BodyIdCount > 0 {
		copy(player.bodyIds, player.frame0BodyIds[:player.frame0BodyIdCount])
	}
}

// SeekFrame is b3RecPlayer_SeekFrame.
// Seek to a specific frame. Forward seek steps op-by-op; backward seek restores
// the nearest keyframe then re-steps the remaining gap.
func (player *RecPlayer) SeekFrame(targetFrame int) {
	if player == nil {
		return
	}

	player.atPreStep = false

	if targetFrame < 0 {
		targetFrame = 0
	}

	// Find the best keyframe strictly before the target.
	var best *recKeyframe
	for i := 0; i < player.keyframeCount; i++ {
		kf := &player.keyframes[i]
		if kf.frame < targetFrame && (best == nil || kf.frame > best.frame) {
			best = kf
		}
	}

	if targetFrame < player.frame {
		// Backward seek: restore keyframe or restart from frame 0.
		if best != nil {
			recPlayerRestoreKeyframe(player, best)
		} else {
			player.Restart()
		}
	} else if best != nil && best.frame > player.frame {
		// Forward seek that can skip ahead via a keyframe.
		recPlayerRestoreKeyframe(player, best)
	}

	for player.frame < targetFrame && player.StepFrame() {
	}
}

// GetWorldId is b3RecPlayer_GetWorldId: the world currently driven by this player.
func (player *RecPlayer) GetWorldId() WorldID {
	if player != nil {
		return player.rdr.replayWorldId
	}
	return NullWorldID
}

// GetFrame is b3RecPlayer_GetFrame: the last fully-stepped frame index (0 before any step).
func (player *RecPlayer) GetFrame() int {
	if player != nil {
		return player.frame
	}
	return 0
}

// GetFrameCount is b3RecPlayer_GetFrameCount: total number of recorded frames.
func (player *RecPlayer) GetFrameCount() int {
	if player != nil {
		return player.frameCount
	}
	return 0
}

// IsAtEnd is b3RecPlayer_IsAtEnd: true when the op stream is exhausted.
func (player *RecPlayer) IsAtEnd() bool {
	if player != nil {
		return player.atEnd
	}
	return true
}

// IsAtPreStep is b3RecPlayer_IsAtPreStep: true when the op stream is paused between body creation
// and world step.
func (player *RecPlayer) IsAtPreStep() bool {
	if player != nil {
		return player.atPreStep
	}
	return false
}

// HasDiverged is b3RecPlayer_HasDiverged: true when any StateHash mismatch has been detected.
func (player *RecPlayer) HasDiverged() bool {
	if player != nil {
		return player.rdr.diverged
	}
	return false
}

// GetInfo is b3RecPlayer_GetInfo: a summary of the recording read at open: frame count, recorded
// tuning, and bounds.
func (player *RecPlayer) GetInfo() RecPlayerInfo {
	var info RecPlayerInfo
	if player != nil {
		info.FrameCount = int32(player.frameCount)
		info.WorkerCount = int32(player.recordedWorkerCount)
		info.TimeStep = player.recordedDt
		info.SubStepCount = int32(player.recordedSubStepCount)
		info.LengthScale = player.lengthScale
		info.Bounds = player.bounds
	}
	return info
}

// GetDivergeFrame is b3RecPlayer_GetDivergeFrame: the first frame at which replay diverged, or -1
// if it has not diverged.
func (player *RecPlayer) GetDivergeFrame() int {
	if player != nil {
		return player.divergeFrame
	}
	return -1
}

// SetWorkerCount is b3RecPlayer_SetWorkerCount.
// Set the worker count of the replay world. Clamped to [1, B3_MAX_WORKERS]. Applied to the live
// world at once and reused whenever the player rebuilds its world on Restart or a backward seek.
// Replaying at a different count than recorded re-partitions the constraint graph, so the StateHash
// check becomes a cross-thread determinism test.
func (player *RecPlayer) SetWorkerCount(count int) {
	if player == nil {
		return
	}

	player.recordedWorkerCount = ClampInt(count, 1, MaxWorkers)

	// Apply to the live world now so the next steps re-partition without a rebuild. Worker count is
	// host state, not part of a keyframe image, so it survives an in-place restore. A rebuild on
	// Restart or deep backward seek picks the count back up through b3RecPlayerCreateWorld.
	if player.rdr.replayWorldId.IsValid() {
		player.rdr.replayWorldId.SetWorkerCount(player.recordedWorkerCount)
	}
}

// SetKeyframePolicy is b3RecPlayer_SetKeyframePolicy.
// Tune the keyframe ring used to speed up backward seeking. A keyframe is a periodic snapshot the
// player restores from instead of replaying from the start, trading memory for seek speed.
// budgetBytes is the memory cap for the kept snapshots; the spacing widens to stay under it.
// minIntervalFrames is the finest spacing between keyframes, in frames.
// A zero budget or a non-positive interval keeps that value. Clears the existing ring, so call
// Restart afterward to repopulate it under the new policy.
func (player *RecPlayer) SetKeyframePolicy(budgetBytes uint64, minIntervalFrames int) {
	if player == nil {
		return
	}
	if budgetBytes > 0 {
		player.keyframeBudget = budgetBytes
	}
	if minIntervalFrames > 0 {
		player.keyframeMinInterval = minIntervalFrames
	}

	// Drop the ring so it repopulates under the new policy on the next replay.
	for i := 0; i < player.keyframeCount; i++ {
		freeKeyframe(&player.keyframes[i])
	}
	player.keyframeCount = 0
	player.keyframeBytes = 0
	player.keyframeInterval = player.keyframeMinInterval
	player.lastKeyframeFrame = 0
}

// GetKeyframeBudget is b3RecPlayer_GetKeyframeBudget: the keyframe memory budget in bytes.
func (player *RecPlayer) GetKeyframeBudget() uint64 {
	if player != nil {
		return player.keyframeBudget
	}
	return 0
}

// GetKeyframeMinInterval is b3RecPlayer_GetKeyframeMinInterval: the finest keyframe spacing in frames.
func (player *RecPlayer) GetKeyframeMinInterval() int {
	if player != nil {
		return player.keyframeMinInterval
	}
	return 0
}

// GetKeyframeInterval is b3RecPlayer_GetKeyframeInterval: the current keyframe spacing in frames;
// starts at the min interval and doubles as the ring evicts to stay under budget, so it reflects the
// effective backward-seek granularity now.
func (player *RecPlayer) GetKeyframeInterval() int {
	if player != nil {
		return player.keyframeInterval
	}
	return 0
}

// GetKeyframeBytes is b3RecPlayer_GetKeyframeBytes: the memory currently held by keyframe
// snapshots, in bytes.
func (player *RecPlayer) GetKeyframeBytes() uint64 {
	if player != nil {
		return player.keyframeBytes
	}
	return 0
}

// GetBodyCount is b3RecPlayer_GetBodyCount: the number of bodies tracked in creation order
// (including holes for destroyed bodies).
func (player *RecPlayer) GetBodyCount() int {
	if player != nil {
		return player.bodyIdCount
	}
	return 0
}

// GetBodyId is b3RecPlayer_GetBodyId.
// Resolve a creation ordinal to the live body id at the current frame.
// Returns the body id, or a null id if that ordinal is out of range or its body is destroyed.
func (player *RecPlayer) GetBodyId(index int) BodyID {
	if player == nil || index < 0 || index >= player.bodyIdCount {
		return NullBodyID
	}
	return player.bodyIds[index]
}

// recQuerySelColor is b3RecQuerySelColor.
// A selected query draws in one reserved color so it stands out when every query is drawn at once.
func recQuerySelColor(selected bool, base HexColor) HexColor {
	if selected {
		return ColorPlum
	}
	return base
}

// recDrawHitBounds is b3RecDrawHitBounds.
// Highlight each reported overlap shape by its AABB. Skip any destroyed since the query, per the
// b3Shape_GetAABB contract that overlap results may contain stale shapes.
func recDrawHitBounds(player *RecPlayer, q *recDrawQuery, draw *DebugDraw, color HexColor) {
	if draw.DrawBoundsFcn == nil {
		return
	}
	for hi := q.hitStart; hi < q.hitStart+q.hitCount; hi++ {
		id := player.frameHits[hi].id
		if id.IsValid() == false {
			continue
		}
		draw.DrawBoundsFcn(id.GetAABB(), color, draw.Context)
	}
}

// recDrawProxy is b3RecDrawProxy.
// Draw a recorded shape proxy at basePos. A lone point with no radius draws a fat point, a lone point
// with a radius draws a translucent sphere, and a multi-point cloud draws its points. Capsule and hull
// proxies are rare and fall through to the point cloud for now.
func recDrawProxy(draw *DebugDraw, basePos Pos, q *recDrawQuery, color HexColor) {
	if q.proxyCount == 1 {
		p := OffsetPos(basePos, q.proxyPoints[0])
		if q.proxyRadius > 0.0 {
			if draw.DrawSphereFcn != nil {
				draw.DrawSphereFcn(p, q.proxyRadius, color, 0.5, draw.Context)
			}
		} else if draw.DrawPointFcn != nil {
			draw.DrawPointFcn(p, 10.0, color, draw.Context)
		}
	} else if q.proxyCount == 2 && q.proxyRadius > 0.0 {
		if draw.DrawCapsuleFcn != nil {
			p1 := OffsetPos(basePos, q.proxyPoints[0])
			p2 := OffsetPos(basePos, q.proxyPoints[1])
			draw.DrawCapsuleFcn(p1, p2, q.proxyRadius, color, 0.5, draw.Context)
		}
	} else if q.proxyCount >= 2 && draw.DrawPointFcn != nil {
		for i := int32(0); i < q.proxyCount; i++ {
			draw.DrawPointFcn(OffsetPos(basePos, q.proxyPoints[i]), 6.0, color, draw.Context)
		}
	}
}

// recCString returns the bytes of a zero-terminated name buffer up to the terminator.
// PORT: helper for the char arrays of b3RecTag.
func recCString(b []byte) string {
	for i, c := range b {
		if c == 0 {
			return string(b[:i])
		}
	}
	return string(b)
}

// DrawFrameQueries is b3RecPlayer_DrawFrameQueries.
// Draw the spatial queries recorded during the most recently replayed frame, layered on top of the
// world. Call after World.Draw. Nil draw functions are skipped.
// queryIndex is the index of the frame query to draw, or -1 to draw all of them.
// selectedIndex is the index of the query to emphasize (reserved color plus a label), or -1 for none.
func (player *RecPlayer) DrawFrameQueries(draw *DebugDraw, queryIndex int, selectedIndex int) {
	if player == nil || draw == nil {
		return
	}

	// queryIndex < 0 draws every query, otherwise just the one selected in the viewer. The query at
	// selectedIndex draws in one reserved color and is labeled, so it stands out among the rest.
	for qi := 0; qi < player.frameQueryCount; qi++ {
		if queryIndex >= 0 && qi != queryIndex {
			continue
		}

		q := &player.frameQueries[qi]
		selected := qi == selectedIndex

		switch q.kind {
		case recqCastRay, recqCastRayClosest:
			origin := q.origin
			end := OffsetPos(origin, q.translation)
			if draw.DrawSegmentFcn != nil {
				draw.DrawSegmentFcn(origin, end, recQuerySelColor(selected, ColorYellow), draw.Context)
			}
			for hi := q.hitStart; hi < q.hitStart+q.hitCount; hi++ {
				h := &player.frameHits[hi]
				if draw.DrawPointFcn != nil {
					draw.DrawPointFcn(h.point, 4.0, recQuerySelColor(selected, ColorYellow), draw.Context)
				}
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(h.point, OffsetPos(h.point, MulSV(0.2, h.normal)),
						recQuerySelColor(selected, ColorYellowGreen), draw.Context)
				}
			}

		case recqCastShape:
			// Draw the cast line and the proxy at its start, then each hit point and normal.
			if draw.DrawSegmentFcn != nil {
				draw.DrawSegmentFcn(q.origin, OffsetPos(q.origin, q.translation),
					recQuerySelColor(selected, ColorSkyBlue), draw.Context)
			}
			recDrawProxy(draw, q.origin, q, recQuerySelColor(selected, ColorLightGreen))
			for hi := q.hitStart; hi < q.hitStart+q.hitCount; hi++ {
				h := &player.frameHits[hi]
				if draw.DrawPointFcn != nil {
					draw.DrawPointFcn(h.point, 4.0, recQuerySelColor(selected, ColorSkyBlue), draw.Context)
				}
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(h.point, OffsetPos(h.point, MulSV(0.2, h.normal)),
						recQuerySelColor(selected, ColorLightSkyBlue), draw.Context)
				}
				if draw.DrawSphereFcn != nil {
					p := OffsetPos(q.origin, MulSV(h.fraction, q.translation))
					recDrawProxy(draw, p, q, recQuerySelColor(selected, ColorSkyBlue))
				}
			}

		case recqCastMover:
			c1 := OffsetPos(q.origin, q.mover.Center1)
			c2 := OffsetPos(q.origin, q.mover.Center2)
			c := ColorLightSkyBlue
			if draw.DrawCapsuleFcn != nil {
				draw.DrawCapsuleFcn(c1, c2, q.mover.Radius, recQuerySelColor(selected, c), 0.6, draw.Context)

				if q.castFraction > 0.01 {
					d := MulSV(q.castFraction, q.translation)
					c1 = OffsetPos(c1, d)
					c2 = OffsetPos(c2, d)
					draw.DrawCapsuleFcn(c1, c2, q.mover.Radius, c, 0.3, draw.Context)
				}
			}

		case recqCollideMover:
			c1 := OffsetPos(q.origin, q.mover.Center1)
			c2 := OffsetPos(q.origin, q.mover.Center2)
			c := ColorTan
			if draw.DrawCapsuleFcn != nil {
				draw.DrawCapsuleFcn(c1, c2, q.mover.Radius, recQuerySelColor(selected, c), 0.6, draw.Context)
			}

			for hi := q.hitStart; hi < q.hitStart+q.hitCount; hi++ {
				h := &player.frameHits[hi]
				point := OffsetPos(q.origin, h.plane.Point)
				if draw.DrawSegmentFcn != nil {
					draw.DrawSegmentFcn(point, OffsetPos(point, MulSV(0.2, h.plane.Plane.Normal)),
						recQuerySelColor(selected, ColorOrange), draw.Context)
				}
			}

		case recqOverlapAABB:
			if draw.DrawBoundsFcn != nil {
				draw.DrawBoundsFcn(q.aabb, recQuerySelColor(selected, ColorLimeGreen), draw.Context)
			}
			recDrawHitBounds(player, q, draw, recQuerySelColor(selected, ColorMagenta))

		case recqOverlapShape:
			// The overlap proxy sits at the origin; draw it, then the overlapping shape bounds.
			recDrawProxy(draw, q.origin, q, recQuerySelColor(selected, ColorLimeGreen))
			recDrawHitBounds(player, q, draw, recQuerySelColor(selected, ColorMagenta))

		default:
		}

		// Label the selected query at its origin so it reads by name among the others. The overlap AABB
		// has no origin, so anchor at the box center. Untagged queries (no key) rely on the color alone.
		if selected && q.key != 0 && draw.DrawStringFcn != nil {
			name := ""
			hasName := false
			id := uint64(0)
			if player.rdr.tagMap != nil {
				if val, found := player.rdr.tagMap[q.key]; found {
					tag := &player.rdr.tags[val]
					name = recCString(tag.queryName[:])
					hasName = true
					id = tag.id
				}
			}
			// PORT: %.40s truncates to 40 bytes as in C.
			short := name
			if len(short) > 40 {
				short = short[:40]
			}
			var label string
			if hasName && name != "" && id != 0 {
				label = fmt.Sprintf("%s (%d)", short, id)
			} else if hasName && name != "" {
				label = short
			} else {
				label = fmt.Sprintf("#%d", id)
			}
			labelPos := q.origin
			if q.kind == recqOverlapAABB {
				labelPos = ToPos(q.aabb.Center())
			} else if q.kind == recqCastMover || q.kind == recqCollideMover {
				// Sit the label just past the center2 end cap, which for an upright mover is above it.
				c1 := OffsetPos(q.origin, q.mover.Center1)
				c2 := OffsetPos(q.origin, q.mover.Center2)
				dir := Normalize(SubPos(c2, c1))
				labelPos = OffsetPos(c2, MulSV(1.25*q.mover.Radius, dir))
			}
			draw.DrawStringFcn(labelPos, label, ColorWhite, draw.Context)
		}
	}
}

// The internal b3RecQueryKind values match the public b3RecQueryType, so the kind copies across as a
// plain cast. Pin the first and last kinds to catch enum drift.
// PORT: _Static_assert as constant index expressions.
var _ = [1]struct{}{}[RecQueryOverlapAABB+recqOverlapAABB]         // query type enum drift
var _ = [1]struct{}{}[RecQueryCollideMover-6+(recqCollideMover-6)] // query type enum drift

// GetFrameQueryCount is b3RecPlayer_GetFrameQueryCount: the number of spatial queries recorded for
// the most recently replayed frame.
func (player *RecPlayer) GetFrameQueryCount() int {
	if player != nil {
		return player.frameQueryCount
	}
	return 0
}

// GetFrameQuery is b3RecPlayer_GetFrameQuery.
// Get a recorded query from the most recently replayed frame by index.
func (player *RecPlayer) GetFrameQuery(index int) RecQueryInfo {
	var info RecQueryInfo
	if player == nil || index < 0 || index >= player.frameQueryCount {
		return info
	}

	q := &player.frameQueries[index]
	info.Type = RecQueryType(q.kind)
	info.Filter = q.filter
	info.AABB = q.aabb
	info.Origin = q.origin
	info.Translation = q.translation
	info.HitCount = q.hitCount
	info.Key = q.key
	info.Id = 0
	info.Name = ""
	if q.key != 0 && player.rdr.tagMap != nil {
		if val, found := player.rdr.tagMap[q.key]; found {
			tag := &player.rdr.tags[val]
			info.Id = tag.id
			// An id-only tag interns an empty name; report it as none so the viewer shows the id alone.
			info.Name = recCString(tag.queryName[:])
		}
	}
	return info
}

// GetFrameQueryHit is b3RecPlayer_GetFrameQueryHit.
// Get one result of a recorded query from the most recently replayed frame.
func (player *RecPlayer) GetFrameQueryHit(queryIndex int, hitIndex int) RecQueryHit {
	var hit RecQueryHit
	if player == nil || queryIndex < 0 || queryIndex >= player.frameQueryCount {
		return hit
	}

	q := &player.frameQueries[queryIndex]
	if hitIndex < 0 || hitIndex >= int(q.hitCount) {
		return hit
	}

	h := &player.frameHits[int(q.hitStart)+hitIndex]
	hit.Shape = h.id
	hit.Point = h.point
	hit.Normal = h.normal
	hit.Fraction = h.fraction
	return hit
}

// SetDebugShapeCallbacks is b3RecPlayer_SetDebugShapeCallbacks.
// Wire host debug-shape callbacks into the player's replay world so a renderer can build
// per-shape draw resources (the 3D sample needs this or the replay world draws nothing).
// Rebuilds the current world under the new callbacks and rewinds to frame 0, so call it
// once right after CreatePlayer and re-read the world id afterward. The callbacks
// persist across Restart and backward seeks, which recreate the world internally.
func (player *RecPlayer) SetDebugShapeCallbacks(createDebugShape CreateDebugShapeCallback,
	destroyDebugShape DestroyDebugShapeCallback, context any) {
	if player == nil {
		return
	}

	player.createDebugShape = createDebugShape
	player.destroyDebugShape = destroyDebugShape
	player.debugShapeContext = context

	// A world fixes its debug-shape callbacks at creation, so rebuild frame 0 under the new
	// wiring. The old world held no adapter shapes (its callbacks were NULL), so the tear-down
	// is balanced. Geometry slots are byte blobs reused as-is, the same path Restart relies on.
	if player.rdr.replayWorldId.IsValid() {
		player.rdr.replayWorldId.Destroy()
	}
	player.rdr.replayWorldId = recPlayerCreateWorld(player)
	player.rdr.cursor = player.headerEnd
	player.rdr.ok = true
	player.rdr.diverged = false
	player.frame = 0
	player.divergeFrame = -1
	player.atEnd = false

	// Re-seed the world so its shapes are recreated through the new callbacks.
	if recDeserializeSeed(player, player.frame0Image[:player.frame0Size]) == false {
		player.rdr.ok = false
		return
	}
	player.rdr.cursor = player.headerEnd

	// Rebuild the outliner from the frame-0 world that was just stood up under the new callbacks.
	recSeedFrame0BodyIds(player)
}
