// Port of src/body.h and src/body.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// ---- body.h ----

// b3BodyFlags. PORT: untyped constants so they mix with the uint32 flag fields.
const (
	// This body has fixed translation along the x-axis
	lockLinearX = 0x00000001

	// This body has fixed translation along the y-axis
	lockLinearY = 0x00000002

	// This body has fixed translation along the z-axis
	lockLinearZ = 0x00000004

	// This body has fixed rotation around the x-axis
	lockAngularX = 0x00000008

	// This body has fixed rotation around the y-axis
	lockAngularY = 0x00000010

	// This body has fixed rotation around the z-axis
	lockAngularZ = 0x00000020

	// This flag is used for debug draw and contact recycling. Only lives on b3BodySim.
	isFast = 0x00000040

	// This dynamic body does a final CCD pass against all body types, but not other bullets
	isBullet = 0x00000080

	// This body was speed capped in the current time step
	isSpeedCapped = 0x00000100

	// This body had a time of impact event in the current time step (body sim only)
	hadTimeOfImpact = 0x00000200

	// This body has no limit on angular velocity
	allowFastRotation = 0x00000400

	// This bullet body needs to have its AABB increased. Needed because bullets don't follow
	// the standard broad-phase update.
	enlargeBulletBounds = 0x00000800

	// This body is dynamic so the solver should write to it.
	// This prevents writing to kinematic bodies that causes a multithreaded sharing
	// cache coherence problem even when the values are not changing.
	// Used for b3BodyState flags.
	dynamicFlag = 0x00001000

	enableSleep = 0x00002000

	enableContactRecycling = 0x00004000

	// The user deferred mass computation via the updateBodyMass shape option and mass
	// data still hasn't been set.
	dirtyMass = 0x00008000

	enableGyroscopicTorque = 0x00010000

	// All lock flags
	allLocks = lockLinearX | lockLinearY | lockLinearZ | lockAngularX | lockAngularY | lockAngularZ

	// If all these flags are set then the body has fixed rotation
	fixedRotation = lockAngularX | lockAngularY | lockAngularZ

	// These flags are transient per time step. These may be different across b3Body, b3BodySim, and b3BodyState.
	bodyTransientFlags = isSpeedCapped | hadTimeOfImpact
)

// body is b3Body: body organizational details that are not used in the solver.
type body struct {
	userData any

	// index of solver set stored in b3World
	// may be B3_NULL_INDEX
	setIndex int32

	// body sim and state index within set
	// may be B3_NULL_INDEX
	localIndex int32

	// [31 : contactId | 1 : edgeIndex]
	headContactKey int32
	contactCount   int32

	// todo maybe move this to the body sim
	headShapeId int32
	shapeCount  int32

	headChainId int32

	// [31 : jointId | 1 : edgeIndex]
	headJointKey int32
	jointCount   int32

	// All enabled dynamic and kinematic bodies are in an island.
	islandId int32

	// Index into the island's bodies array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	sleepThreshold float32
	sleepTime      float32
	sleepVelocity  float32
	safetyFactor   float32
	mass           float32

	// this is used to adjust the fellAsleep flag in the body move array
	bodyMoveIndex int32

	id int32

	// b3BodyFlags
	flags  uint32
	nameId uint32

	typ BodyType // PORT: type is a Go keyword

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3BodyId
	generation uint16
}

// bodyState is b3BodyState.
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.
//
// 64 bytes
type bodyState struct {
	linearVelocity Vec3 // 12
	padding1       float32

	angularVelocity Vec3 // 12
	padding2        float32

	// Using delta position reduces round-off error far from the origin
	deltaPosition Vec3 // 12

	// b3BodyFlags
	// Important flags: locking, dynamic
	flags uint32 // 4

	// Using delta rotation because I cannot access the full rotation on static bodies in
	// the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
	deltaRotation Quat // 16
}

// identityBodyState is b3_identityBodyState: identity body state, notice the deltaRotation is identity
var identityBodyState = bodyState{
	Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 0.0, 0.0}, 0, Quat{Vec3{0.0, 0.0, 0.0}, 1.0},
}

// bodySim is b3BodySim.
// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
type bodySim struct {
	// transform for body origin, double translation in large world mode
	transform WorldTransform

	// center of mass position in world space
	center Pos

	// b3BodyFlags
	flags uint32

	minExtent float32
	maxExtent Vec3

	// previous rotation and COM for TOI
	rotation0 Quat
	center0   Pos

	// location of center of mass relative to the body origin
	localCenter Vec3

	force  Vec3
	torque Vec3

	invMass float32

	// Local space inertia
	inertia Matrix3

	// Rotational inertia about the center of mass. The world space inverse inertia tensor
	// must be updated whenever the body rotation is modified.
	invInertiaLocal Matrix3
	invInertiaWorld Matrix3

	linearDamping  float32
	angularDamping float32
	gravityScale   float32

	// Index of b3Body
	bodyId int32
}

// encodeBodySimIndex is b3EncodeBodySimIndex.
// Encode the body sim index for storage in the contact.
func encodeBodySimIndex(body *body) int {
	if body.setIndex == awakeSet {
		return int(body.localIndex)
	}

	if body.setIndex == staticSet {
		return -(int(body.localIndex) + 2)
	}

	return NullIndex
}

// isStaticSimIndex is b3IsStaticSimIndex.
func isStaticSimIndex(encodedBodySimIndex int) bool {
	return encodedBodySimIndex < NullIndex
}

// decodeAwakeIndex is b3DecodeAwakeIndex.
func decodeAwakeIndex(encodedBodySimIndex int) int {
	if encodedBodySimIndex >= 0 {
		return encodedBodySimIndex
	}
	return NullIndex
}

// sleepBodySimIndex is b3SleepBodySimIndex.
func sleepBodySimIndex(encodedBodySimIndex int) int {
	if encodedBodySimIndex >= 0 {
		return NullIndex
	}
	return encodedBodySimIndex
}

// makeRelativeSweep is b3MakeRelativeSweep.
// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
func makeRelativeSweep(bodySim *bodySim, base Pos) Sweep {
	var s Sweep
	s.C1 = SubPos(bodySim.center0, base)
	s.C2 = SubPos(bodySim.center, base)
	s.Q1 = bodySim.rotation0
	s.Q2 = bodySim.transform.Q
	s.LocalCenter = bodySim.localCenter
	return s
}

// ---- body.c ----

// PORT: the b3Body_* functions of box3d.h are methods on BodyID and b3CreateBody is a method on
// WorldID. B3_REC / B3_REC_CREATE are expanded at the call site (see recording.go).

// getBodyFullId is b3GetBodyFullId.
// Get a validated body from a world using an id.
func getBodyFullId(world *world, bodyId BodyID) *body {
	assert(bodyId.IsValid(), "b3Body_IsValid( bodyId )")

	// id index starts at one so that zero can represent null
	// id index starts at one so that zero can represent null
	return arrayGet(world.bodies, int(bodyId.Index1-1))
}

// getBodyTransformQuick is b3GetBodyTransformQuick.
func getBodyTransformQuick(world *world, body *body) WorldTransform {
	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))
	return bodySim.transform
}

// getBodyTransform is b3GetBodyTransform.
func getBodyTransform(world *world, bodyId int) WorldTransform {
	body := arrayGet(world.bodies, bodyId)
	return getBodyTransformQuick(world, body)
}

// makeBodyId is b3MakeBodyId.
// Create a b3BodyId from a raw id.
func makeBodyId(world *world, bodyId int) BodyID {
	body := arrayGet(world.bodies, bodyId)
	return BodyID{int32(bodyId + 1), world.worldId, body.generation}
}

// getBodySim is b3GetBodySim.
func getBodySim(world *world, body *body) *bodySim {
	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))
	return bodySim
}

// getBodyState is b3GetBodyState.
func getBodyState(world *world, body *body) *bodyState {
	if body.setIndex == awakeSet {
		set := arrayGet(world.solverSets, awakeSet)
		return arrayGet(set.bodyStates, int(body.localIndex))
	}

	return nil
}

// refreshBodyContactIndices is b3RefreshBodyContactIndices.
func refreshBodyContactIndices(world *world, body *body) {
	encodedBodySimIndex := int32(encodeBodySimIndex(body))

	contactKey := body.headContactKey
	for contactKey != NullIndex {
		edgeIndex := contactKey & 1
		contactId := contactKey >> 1

		contact := arrayGet(world.contacts, int(contactId))
		contactKey = contact.edges[edgeIndex].nextKey

		if edgeIndex == 0 {
			contact.encodedBodySimA = encodedBodySimIndex
		} else {
			contact.encodedBodySimB = encodedBodySimIndex
		}
	}
}

// syncBodyFlags is b3SyncBodyFlags.
func syncBodyFlags(world *world, body *body) {
	bodySim := getBodySim(world, body)

	// Preserve the sim only flags: fast for contact recycling, time of impact for debug draw.
	bodySim.flags = (bodySim.flags & (isFast | hadTimeOfImpact)) | (body.flags &^ bodyTransientFlags)

	bodyState := getBodyState(world, body)
	if bodyState != nil {
		bodyState.flags = body.flags &^ bodyTransientFlags
	}
}

// createIslandForBody is b3CreateIslandForBody.
func createIslandForBody(world *world, setIndex int, body *body) {
	assert(body.islandId == NullIndex, "body->islandId == B3_NULL_INDEX")
	assert(setIndex != disabledSet, "setIndex != b3_disabledSet")

	island := createIsland(world, setIndex)
	arrayPush(&island.bodies, body.id)
	body.islandId = island.islandId
	body.islandIndex = 0

	validateIsland(world, int(island.islandId))
}

// removeBodyFromIsland is b3RemoveBodyFromIsland.
func removeBodyFromIsland(world *world, body *body) {
	if body.islandId == NullIndex {
		assert(body.islandIndex == NullIndex, "body->islandIndex == B3_NULL_INDEX")
		return
	}

	islandId := int(body.islandId)
	island := arrayGet(world.islands, islandId)
	{
		localIndex := body.islandIndex
		movedBodyId := island.bodies[len(island.bodies)-1]
		island.bodies[localIndex] = movedBodyId
		validate(world.bodies[movedBodyId].islandIndex == int32(len(island.bodies)-1),
			"world->bodies.data[movedBodyId].islandIndex == island->bodies.count - 1")
		world.bodies[movedBodyId].islandIndex = localIndex
		island.bodies = island.bodies[:len(island.bodies)-1]
	}

	if len(island.bodies) == 0 {
		// Destroy empty island
		assert(len(island.contacts) == 0, "island->contacts.count == 0")
		assert(len(island.joints) == 0, "island->joints.count == 0")

		// Free the island
		destroyIsland(world, int(island.islandId))
	} else {
		validateIsland(world, islandId)
	}

	body.islandId = NullIndex
	body.islandIndex = NullIndex
}

// destroyBodyContacts is b3DestroyBodyContacts.
func destroyBodyContacts(world *world, body *body, wakeBodies bool) {
	// Destroy the attached contacts
	edgeKey := body.headContactKey
	for edgeKey != NullIndex {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		edgeKey = contact.edges[edgeIndex].nextKey
		destroyContact(world, contact, wakeBodies)
	}

	validateSolverSets(world)
}

// CreateBody is b3CreateBody.
// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
// on the stack and pass it as a pointer.
//
//	bodyDef := box3d.DefaultBodyDef()
//	myBodyId := myWorldId.CreateBody(&bodyDef)
//
// Warning: This function is locked during callbacks.
func (worldId WorldID) CreateBody(def *BodyDef) BodyID {
	checkDef(def.internalValue)
	assert(IsValidPosition(def.Position), "b3IsValidPosition( def->position )")
	assert(IsValidQuat(def.Rotation), "b3IsValidQuat( def->rotation )")
	assert(IsValidVec3(def.LinearVelocity), "b3IsValidVec3( def->linearVelocity )")
	assert(IsValidVec3(def.AngularVelocity), "b3IsValidVec3( def->angularVelocity )")
	assert(IsValidFloat(def.LinearDamping) && def.LinearDamping >= 0.0, "b3IsValidFloat( def->linearDamping ) && def->linearDamping >= 0.0f")
	assert(IsValidFloat(def.AngularDamping) && def.AngularDamping >= 0.0, "b3IsValidFloat( def->angularDamping ) && def->angularDamping >= 0.0f")
	assert(IsValidFloat(def.SleepThreshold) && def.SleepThreshold >= 0.0, "b3IsValidFloat( def->sleepThreshold ) && def->sleepThreshold >= 0.0f")
	assert(IsValidFloat(def.SafetyFactor) && def.SafetyFactor >= 0.0, "b3IsValidFloat( def->safetyFactor ) && def->safetyFactor >= 0.0f")
	assert(IsValidFloat(def.GravityScale), "b3IsValidFloat( def->gravityScale )")

	world := getUnlockedWorldFromId(worldId)

	if world == nil {
		return NullBodyID
	}

	world.locked = true

	isAwake := (def.IsAwake || def.EnableSleep == false) && def.IsEnabled

	// determine the solver set
	var setId int
	if def.IsEnabled == false {
		// any body type can be disabled
		setId = disabledSet
	} else if def.Type == StaticBody {
		setId = staticSet
	} else if isAwake == true {
		setId = awakeSet
	} else {
		// new set for a sleeping body in its own island
		setId = allocId(&world.solverSetIdPool)
		if setId == len(world.solverSets) {
			// Create a zero initialized solver set. All sub-arrays are also zero initialized.
			arrayPush(&world.solverSets, solverSet{})
		} else {
			assert(world.solverSets[setId].setIndex == NullIndex, "world->solverSets.data[setId].setIndex == B3_NULL_INDEX")
		}

		world.solverSets[setId].setIndex = int32(setId)
	}

	assert(0 <= setId && setId < len(world.solverSets), "0 <= setId && setId < world->solverSets.count")

	bodyId := allocId(&world.bodyIdPool)

	var lockFlags uint32 = 0
	if def.MotionLocks.LinearX {
		lockFlags |= lockLinearX
	}
	if def.MotionLocks.LinearY {
		lockFlags |= lockLinearY
	}
	if def.MotionLocks.LinearZ {
		lockFlags |= lockLinearZ
	}
	if def.MotionLocks.AngularX {
		lockFlags |= lockAngularX
	}
	if def.MotionLocks.AngularY {
		lockFlags |= lockAngularY
	}
	if def.MotionLocks.AngularZ {
		lockFlags |= lockAngularZ
	}

	set := arrayGet(world.solverSets, setId)
	bodySim := arrayEmplace(&set.bodySims)
	// PORT: *bodySim = (b3BodySim){ 0 } is implied: arrayEmplace returns a zeroed element.
	bodySim.transform.P = def.Position
	bodySim.transform.Q = def.Rotation
	bodySim.center = def.Position
	bodySim.rotation0 = bodySim.transform.Q
	bodySim.center0 = bodySim.center
	bodySim.localCenter = Vec3Zero
	bodySim.force = Vec3Zero
	bodySim.torque = Vec3Zero
	bodySim.invMass = 0.0
	bodySim.inertia = Mat3Zero
	bodySim.invInertiaLocal = Mat3Zero
	bodySim.minExtent = Huge()
	bodySim.maxExtent = Vec3Zero
	bodySim.linearDamping = def.LinearDamping
	bodySim.angularDamping = def.AngularDamping
	bodySim.gravityScale = def.GravityScale
	bodySim.bodyId = int32(bodyId)
	bodySim.flags = lockFlags
	if def.IsBullet {
		bodySim.flags |= isBullet
	}
	if def.AllowFastRotation {
		bodySim.flags |= allowFastRotation
	}
	if def.Type == DynamicBody {
		bodySim.flags |= dynamicFlag
	}
	if def.EnableSleep {
		bodySim.flags |= enableSleep
	}
	if def.EnableContactRecycling {
		bodySim.flags |= enableContactRecycling
	}

	if setId == awakeSet {
		bodyState := arrayEmplace(&set.bodyStates)

		// PORT: *bodyState = (b3BodyState){ 0 } is implied: arrayEmplace returns a zeroed element.
		bodyState.linearVelocity = def.LinearVelocity
		bodyState.angularVelocity = def.AngularVelocity
		bodyState.deltaRotation = QuatIdentity
		bodyState.flags = bodySim.flags
	}

	if bodyId == len(world.bodies) {
		arrayPush(&world.bodies, body{})
	} else {
		assert(world.bodies[bodyId].id == NullIndex, "world->bodies.data[bodyId].id == B3_NULL_INDEX")
	}

	body := arrayGet(world.bodies, bodyId)
	body.userData = def.UserData
	body.setIndex = int32(setId)
	body.localIndex = int32(len(set.bodySims) - 1)
	body.generation += 1
	body.headShapeId = NullIndex
	body.shapeCount = 0
	body.headChainId = NullIndex
	body.headContactKey = NullIndex
	body.contactCount = 0
	body.headJointKey = NullIndex
	body.jointCount = 0
	body.islandId = NullIndex
	body.islandIndex = NullIndex
	body.bodyMoveIndex = NullIndex
	body.id = int32(bodyId)
	body.sleepThreshold = def.SleepThreshold
	body.sleepTime = 0.0
	body.sleepVelocity = 0.0
	body.safetyFactor = def.SafetyFactor
	body.mass = 0.0
	body.nameId = addName(&world.names, def.Name)
	body.typ = def.Type
	body.flags = bodySim.flags

	// dynamic and kinematic bodies that are enabled need a island
	if setId >= awakeSet {
		createIslandForBody(world, setId, body)
	}

	validateSolverSets(world)

	id := BodyID{int32(bodyId + 1), world.worldId, body.generation}

	world.locked = false

	if world.recording != nil {
		recCreateArgs := recArgsCreateBody{worldId, *def}
		recWriteRetCreateBody(world.recording, &recCreateArgs, id)
	}

	return id
}

// isBodyAwake is b3IsBodyAwake.
func isBodyAwake(world *world, body *body) bool {
	return body.setIndex == awakeSet
}

// wakeBody is b3WakeBody.
// careful calling this because it can invalidate body, state, joint, and contact pointers
func wakeBody(world *world, body *body) bool {
	if body.setIndex >= firstSleepingSet {
		wakeSolverSet(world, int(body.setIndex))
		validateSolverSets(world)
		return true
	}

	return false
}

// wakeBodyWithLock is b3WakeBodyWithLock.
func wakeBodyWithLock(world *world, body *body) bool {
	assert(world.locked == false, "world->locked == false")
	world.locked = true
	woke := wakeBody(world, body)
	world.locked = false
	return woke
}

// Destroy is b3DestroyBody.
// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
// Do not keep references to the associated shapes and joints.
func (bodyId BodyID) Destroy() {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsDestroyBody{bodyId}
		recWriteDestroyBody(world.recording, &recArgs)
	}

	world.locked = true

	body := getBodyFullId(world, bodyId)

	// Wake bodies attached to this body, even if this body is static.
	wakeBodies := true

	// Destroy the attached joints
	edgeKey := body.headJointKey
	for edgeKey != NullIndex {
		jointId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		joint := arrayGet(world.joints, int(jointId))
		edgeKey = joint.edges[edgeIndex].nextKey

		// Careful because this modifies the list being traversed
		destroyJointInternal(world, joint, wakeBodies)
	}

	// Destroy all contacts attached to this body.
	destroyBodyContacts(world, body, wakeBodies)

	// Destroy the attached shapes and their broad-phase proxies.
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))

		if shape.typ == CompoundShape {
			assert(world.compoundShapeCount > 0, "world->compoundShapeCount > 0")
			world.compoundShapeCount -= 1
		}

		if shape.sensorIndex != NullIndex {
			destroySensor(world, shape)
		}

		destroyShapeProxy(shape, &world.broadPhase)

		destroyShapeAllocations(world, shape)

		// Return shape to free list.
		freeId(&world.shapeIdPool, int(shapeId))
		shape.id = NullIndex

		shapeId = shape.nextShapeId
	}

	removeBodyFromIsland(world, body)

	// Remove body sim from solver set that owns it
	set := arrayGet(world.solverSets, int(body.setIndex))
	movedIndex := arrayRemoveSwap(&set.bodySims, int(body.localIndex))
	if movedIndex != NullIndex {
		// Fix moved body index
		movedSim := &set.bodySims[body.localIndex]
		movedId := movedSim.bodyId
		movedBody := arrayGet(world.bodies, int(movedId))
		assert(int(movedBody.localIndex) == movedIndex, "movedBody->localIndex == movedIndex")
		movedBody.localIndex = body.localIndex
		refreshBodyContactIndices(world, movedBody)
	}

	// Remove body state from awake set
	if body.setIndex == awakeSet {
		result := arrayRemoveSwap(&set.bodyStates, int(body.localIndex))
		assert(result == movedIndex, "result == movedIndex")
	} else if set.setIndex >= firstSleepingSet && len(set.bodySims) == 0 {
		// Remove solver set if it's now an orphan.
		destroySolverSet(world, int(set.setIndex))
	}

	// Free body and id (preserve body revision)
	freeId(&world.bodyIdPool, int(body.id))

	body.setIndex = NullIndex
	body.localIndex = NullIndex
	body.id = NullIndex

	validateSolverSets(world)

	world.locked = false
}

// GetContactCapacity is b3Body_GetContactCapacity.
// Get the maximum capacity required for retrieving all the touching contacts on a body
func (bodyId BodyID) GetContactCapacity() int {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return 0
	}

	body := getBodyFullId(world, bodyId)

	// Conservative and fast
	return int(body.contactCount)
}

// GetContactData is b3Body_GetContactData.
// Get the touching contact data for a body
// PORT: contactData + capacity become one slice; the capacity is len(contactData).
func (bodyId BodyID) GetContactData(contactData []ContactData) int {
	capacity := len(contactData)

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return 0
	}

	body := getBodyFullId(world, bodyId)

	contactKey := body.headContactKey
	index := 0
	for contactKey != NullIndex && index < capacity {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := arrayGet(world.contacts, int(contactId))

		// Is contact touching?
		if contact.flags&contactTouchingFlag != 0 {
			shapeA := arrayGet(world.shapes, int(contact.shapeIdA))
			shapeB := arrayGet(world.shapes, int(contact.shapeIdB))

			contactData[index].ContactId = ContactID{contact.contactId + 1, bodyId.World0, 0, contact.generation}
			contactData[index].ShapeIdA = ShapeID{shapeA.id + 1, bodyId.World0, shapeA.generation}
			contactData[index].ShapeIdB = ShapeID{shapeB.id + 1, bodyId.World0, shapeB.generation}
			contactData[index].Manifolds = contact.manifolds
			contactData[index].ManifoldCount = contact.manifoldCount
			index += 1
		}

		contactKey = contact.edges[edgeIndex].nextKey
	}

	assert(index <= capacity, "index <= capacity")

	return index
}

// ComputeAABB is b3Body_ComputeAABB.
// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
// If there are no shapes attached then the returned AABB is empty and centered on the body origin.
func (bodyId BodyID) ComputeAABB() AABB {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return AABB{}
	}

	body := getBodyFullId(world, bodyId)
	if body.headShapeId == NullIndex {
		transform := getBodyTransform(world, int(body.id))
		p := ToVec3(transform.P)
		return AABB{p, p}
	}

	shape := arrayGet(world.shapes, int(body.headShapeId))
	aabb := shape.aabb
	for shape.nextShapeId != NullIndex {
		shape = arrayGet(world.shapes, int(shape.nextShapeId))
		aabb = aabb.Union(shape.aabb)
	}

	return aabb
}

// GetClosestPoint is b3Body_GetClosestPoint.
// Get the closest point on a body to a world target.
// PORT: returns the distance and the closest point (upstream's result out-parameter).
func (bodyId BodyID) GetClosestPoint(target Vec3) (float32, Vec3) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return 0.0, Vec3{}
	}

	body := getBodyFullId(world, bodyId)
	worldTransform := getBodyTransform(world, int(body.id))
	transform := ToRelativeTransform(worldTransform, PosZero)

	closestDistance := fltMax
	closestPoint := transform.P

	var input DistanceInput
	// PORT: the proxy points at a one element slice holding target (upstream points at target).
	input.ProxyA = ShapeProxy{[]Vec3{target}, 1, 0.0}

	// Target rides in frame A at the origin, so the relative pose of the shape in A is the body transform
	input.Transform = transform
	input.UseRadii = false

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		typ := shape.typ
		if typ != SphereShape && typ != CapsuleShape && typ != HullShape {
			continue
		}

		input.ProxyB = makeShapeProxy(shape)

		var cache SimplexCache
		output := ShapeDistance(&input, &cache, nil)
		if output.Distance < closestDistance {
			closestDistance = output.Distance
			closestPoint = output.PointB
		}
	}

	return closestDistance, closestPoint
}

// CastRay is b3Body_CastRay.
// Cast a ray at a specific body using a specified body transform.
// Ignores back-side collision on meshes and height-fields.
func (bodyId BodyID) CastRay(origin Pos, translation Vec3, filter QueryFilter, maxFraction float32,
	bodyTransform WorldTransform) BodyCastResult {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return BodyCastResult{}
	}

	var result BodyCastResult
	body := getBodyFullId(world, bodyId)

	// The consistent framing is to center on the ray origin.
	var shapeInput RayCastInput
	shapeInput.Origin = Vec3Zero
	shapeInput.Translation = translation
	shapeInput.MaxFraction = maxFraction

	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		if shouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		shapeOutput := rayCastShapeWithBackFaces(shape, transform, &shapeInput, filter.BackFaces)

		if shapeOutput.Hit == false {
			continue
		}

		if shapeOutput.Fraction > shapeInput.MaxFraction {
			continue
		}

		// Careful with id, shapeId is the next shape.
		id := ShapeID{shape.id + 1, bodyId.World0, shape.generation}

		materialIndex := ClampInt(int(shapeOutput.MaterialIndex), 0, int(shape.materialCount)-1)
		userMaterialId := getShapeMaterials(shape)[materialIndex].UserMaterialId

		result = BodyCastResult{
			ShapeId:        id,
			Point:          OffsetPos(origin, shapeOutput.Point),
			Normal:         shapeOutput.Normal,
			Fraction:       shapeOutput.Fraction,
			TriangleIndex:  shapeOutput.TriangleIndex,
			UserMaterialId: userMaterialId,
			Iterations:     shapeOutput.Iterations,
			Hit:            true,
		}

		shapeInput.MaxFraction = shapeOutput.Fraction
	}

	return result
}

// CastShape is b3Body_CastShape.
// Cast a shape at a specific body using a specified body transform.
// Ignores back-side collision on meshes and height-fields.
func (bodyId BodyID) CastShape(origin Pos, proxy *ShapeProxy, translation Vec3, filter QueryFilter, maxFraction float32,
	canEncroach bool, bodyTransform WorldTransform) BodyCastResult {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return BodyCastResult{}
	}

	var result BodyCastResult
	body := getBodyFullId(world, bodyId)

	transform := ToRelativeTransform(bodyTransform, origin)

	var shapeInput ShapeCastInput
	shapeInput.Proxy = *proxy
	shapeInput.Translation = translation
	shapeInput.MaxFraction = maxFraction
	shapeInput.CanEncroach = canEncroach

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		if shouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		shapeOutput := shapeCastShapeWithBackFaces(shape, transform, &shapeInput, filter.BackFaces)

		if shapeOutput.Hit == false {
			continue
		}

		if shapeOutput.Fraction > shapeInput.MaxFraction {
			continue
		}

		// Careful with id, shapeId is the next shape.
		id := ShapeID{shape.id + 1, bodyId.World0, shape.generation}
		materialIndex := ClampInt(int(shapeOutput.MaterialIndex), 0, int(shape.materialCount)-1)
		userMaterialId := getShapeMaterials(shape)[materialIndex].UserMaterialId

		result = BodyCastResult{
			ShapeId:        id,
			Point:          OffsetPos(origin, shapeOutput.Point),
			Normal:         shapeOutput.Normal,
			Fraction:       shapeOutput.Fraction,
			TriangleIndex:  shapeOutput.TriangleIndex,
			UserMaterialId: userMaterialId,
			Iterations:     shapeOutput.Iterations,
			Hit:            true,
		}

		shapeInput.MaxFraction = shapeOutput.Fraction
	}

	return result
}

// OverlapShape is b3Body_OverlapShape.
// Overlap a shape with a specific body using a specified body transform.
func (bodyId BodyID) OverlapShape(origin Pos, proxy *ShapeProxy, filter QueryFilter, bodyTransform WorldTransform) bool {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return false
	}

	body := getBodyFullId(world, bodyId)
	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		if shouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		overlaps := overlapShape(shape, transform, proxy)
		if overlaps {
			return true
		}
	}

	return false
}

// CollideMover is b3Body_CollideMover.
// Collide a character mover with a specific body using a specified body transform.
// Only considers convex shapes on the body.
// PORT: bodyPlanes + planeCapacity become one slice; the capacity is len(bodyPlanes).
func (bodyId BodyID) CollideMover(bodyPlanes []BodyPlaneResult, origin Pos, mover *Capsule, filter QueryFilter,
	bodyTransform WorldTransform) int {
	planeCapacity := len(bodyPlanes)

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return 0
	}

	if planeCapacity == 0 {
		return 0
	}

	resultCount := 0
	body := getBodyFullId(world, bodyId)

	transform := ToRelativeTransform(bodyTransform, origin)

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		if shouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		typ := shape.typ
		if typ != SphereShape && typ != CapsuleShape && typ != HullShape {
			continue
		}

		// PORT: a one element array stands in for the single b3PlaneResult passed by pointer.
		var plane [1]PlaneResult
		count := collideMover(plane[:], shape, transform, mover)

		if count > 0 {
			id := ShapeID{shape.id + 1, bodyId.World0, shape.generation}
			bodyPlanes[resultCount] = BodyPlaneResult{ShapeId: id, Result: plane[0]}
			resultCount += 1
			if resultCount == planeCapacity {
				return resultCount
			}
		}
	}

	return resultCount
}

// TimeOfImpactMover is b3Body_TimeOfImpactMover.
// Perform a time of impact between a character mover and a body using specified sweep transforms.
// Initial overlap of any shape on the body is ignored. A non-overlapped shape can still be hit.
// Only considers convex shapes on the body.
func (bodyId BodyID) TimeOfImpactMover(origin Pos, mover *Capsule, moverTranslation Vec3, filter QueryFilter,
	bodyTransform1, bodyTransform2 WorldTransform) BodyTOIResult {
	var result BodyTOIResult
	result.Fraction = 1.0

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return result
	}

	xf1 := ToRelativeTransform(bodyTransform1, origin)
	xf2 := ToRelativeTransform(bodyTransform2, origin)

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	localCenter := bodySim.localCenter

	capsulePoints := [2]Vec3{mover.Center1, mover.Center2}
	var input TOIInput
	input.ProxyB = ShapeProxy{
		Points: capsulePoints[:],
		Count:  2,
		Radius: mover.Radius,
	}
	input.SweepA.C1 = TransformPoint(xf1, localCenter)
	input.SweepA.C2 = TransformPoint(xf2, localCenter)
	input.SweepA.Q1 = bodyTransform1.Q
	input.SweepA.Q2 = bodyTransform2.Q
	input.SweepA.LocalCenter = localCenter

	input.SweepB.C1 = Vec3Zero
	input.SweepB.C2 = moverTranslation
	input.SweepB.Q1 = QuatIdentity
	input.SweepB.Q2 = QuatIdentity
	input.SweepB.LocalCenter = Vec3Zero

	input.MaxFraction = 1.0

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		if shouldQueryCollide(&shape.filter, &filter) == false {
			continue
		}

		typ := shape.typ
		if typ != SphereShape && typ != CapsuleShape && typ != HullShape {
			continue
		}

		input.ProxyA = makeShapeProxy(shape)

		output := TimeOfImpact(&input)
		validate(output.State != TOIStateUnknown, "output.state != b3_toiStateUnknown")

		// Mimic behavior in b3ContinuousQueryCallback. Ignore shapes that initially overlap.
		if 0.0 < output.Fraction && output.Fraction < result.Fraction {
			input.MaxFraction = output.Fraction

			result.Point = OffsetPos(origin, output.Point)
			result.Normal = output.Normal
			result.Fraction = output.Fraction
			result.ShapeId = ShapeID{
				Index1:     shape.id + 1,
				World0:     world.worldId,
				Generation: shape.generation,
			}
		}
	}

	return result
}

// updateIsotropicInertia is b3UpdateIsotropicInertia.
// https://en.wikipedia.org/wiki/Cauchy_stress_tensor#Stress_deviator_tensor
func updateIsotropicInertia(world *world, body *body, bodySim *bodySim) {
	inertia := bodySim.inertia
	mean := (inertia.Cx.X + inertia.Cy.Y + inertia.Cz.Z) / 3.0
	dx := inertia.Cx.X - mean
	dy := inertia.Cy.Y - mean
	dz := inertia.Cz.Z - mean
	offDiagonal := float32(inertia.Cy.X*inertia.Cy.X) + float32(inertia.Cz.X*inertia.Cz.X) + float32(inertia.Cz.Y*inertia.Cz.Y)

	// This is the square of the Frobenius matrix norm.
	// https://en.wikipedia.org/wiki/Matrix_norm
	deviation := float32(dx*dx) + float32(dy*dy) + float32(dz*dz) + float32(2.0*offDiagonal)

	tolerance := world.isotropicInertiaTolerance

	if 2.0*deviation > 3.0*tolerance*tolerance*mean*mean {
		body.flags |= enableGyroscopicTorque
	} else {
		body.flags &^= enableGyroscopicTorque
	}

	syncBodyFlags(world, body)
}

// updateBodyMassData is b3UpdateBodyMassData.
func updateBodyMassData(world *world, body *body) {
	bodySim := getBodySim(world, body)

	// Mass is no longer dirty
	body.flags &^= dirtyMass
	body.flags &^= enableGyroscopicTorque
	syncBodyFlags(world, body)

	// Compute mass data from shapes. Each shape has its own density.
	body.mass = 0.0
	bodySim.inertia = Mat3Zero
	bodySim.invMass = 0.0
	bodySim.invInertiaLocal = Mat3Zero
	bodySim.invInertiaWorld = Mat3Zero
	bodySim.localCenter = Vec3Zero
	bodySim.minExtent = Huge()
	bodySim.maxExtent = Vec3Zero

	if body.headShapeId == NullIndex {
		return
	}

	// Static and kinematic sims have zero mass.
	if body.typ != DynamicBody {
		bodySim.center = bodySim.transform.P
		bodySim.center0 = bodySim.center

		// Need extents for kinematic bodies for sleeping to work correctly.
		if body.typ == KinematicBody {
			shapeId := body.headShapeId
			for shapeId != NullIndex {
				s := arrayGet(world.shapes, int(shapeId))

				extent := computeShapeExtent(s, Vec3Zero)
				bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
				bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)

				shapeId = s.nextShapeId
			}
		}

		return
	}

	shapeCount := int(body.shapeCount)
	masses := stackAlloc[MassData](&world.stack, shapeCount, "mass data")

	// Accumulate mass over all shapes.
	localCenter := Vec3Zero
	shapeId := body.headShapeId
	shapeIndex := 0
	for shapeId != NullIndex {
		s := arrayGet(world.shapes, int(shapeId))
		shapeId = s.nextShapeId

		if s.density == 0.0 {
			masses[shapeIndex] = MassData{}
			shapeIndex += 1
			continue
		}

		massData := computeShapeMass(s)
		body.mass += massData.Mass
		localCenter = MulAdd(localCenter, massData.Mass, massData.Center)

		masses[shapeIndex] = massData
		shapeIndex += 1
	}

	// Compute center of mass.
	if body.mass > 0.0 {
		bodySim.invMass = 1.0 / body.mass
		localCenter = MulSV(bodySim.invMass, localCenter)
	}

	// Second loop to accumulate the rotational inertia about the center of mass
	for shapeIndex = 0; shapeIndex < shapeCount; shapeIndex++ {
		massData := masses[shapeIndex]
		if massData.Mass == 0.0 {
			continue
		}

		// Shift to center of mass. This is safe because it can only increase.
		offset := Sub(localCenter, massData.Center)
		inertia := AddMM(massData.Inertia, Steiner(massData.Mass, offset))
		bodySim.inertia = AddMM(bodySim.inertia, inertia)
	}

	stackFree(&world.stack, masses)
	masses = nil

	det := Det(bodySim.inertia)
	assert(det >= 0.0, "det >= 0.0f")

	if det > 0.0 {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = InvertT(bodySim.inertia)

		rotationMatrix := MakeMatrixFromQuat(bodySim.transform.Q)
		bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))
	}

	// Move center of mass.
	oldCenter := bodySim.center
	bodySim.localCenter = localCenter
	bodySim.center = TransformWorldPoint(bodySim.transform, bodySim.localCenter)
	bodySim.center0 = bodySim.center

	// Update center of mass velocity
	state := getBodyState(world, body)
	if state != nil {
		deltaLinear := Cross(state.angularVelocity, SubPos(bodySim.center, oldCenter))
		state.linearVelocity = Add(state.linearVelocity, deltaLinear)
	}

	// Compute body extents relative to center of mass
	shapeId = body.headShapeId
	for shapeId != NullIndex {
		s := arrayGet(world.shapes, int(shapeId))

		extent := computeShapeExtent(s, localCenter)
		bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
		bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)

		shapeId = s.nextShapeId
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	edgeKey := body.headContactKey
	for edgeKey != NullIndex {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		contact.flags &^= relativeTransformValid

		edgeKey = contact.edges[edgeIndex].nextKey
	}

	// Apply fixed rotation
	if (bodySim.flags & fixedRotation) == fixedRotation {
		bodySim.inertia = Mat3Zero
		bodySim.invInertiaLocal = Mat3Zero
		bodySim.invInertiaWorld = Mat3Zero
	}

	updateIsotropicInertia(world, body, bodySim)
}

// GetPosition is b3Body_GetPosition.
// Get the world position of a body. This is the location of the body origin.
func (bodyId BodyID) GetPosition() Pos {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return transform.P
}

// GetRotation is b3Body_GetRotation.
// Get the world rotation of a body as a quaternion
func (bodyId BodyID) GetRotation() Quat {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return transform.Q
}

// GetTransform is b3Body_GetTransform.
// Get the world transform of a body.
func (bodyId BodyID) GetTransform() WorldTransform {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return getBodyTransformQuick(world, body)
}

// GetLocalPoint is b3Body_GetLocalPoint.
// Get a local point on a body given a world point.
func (bodyId BodyID) GetLocalPoint(worldPoint Pos) Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return InvTransformWorldPoint(transform, worldPoint)
}

// GetWorldPoint is b3Body_GetWorldPoint.
// Get a world point on a body given a local point.
func (bodyId BodyID) GetWorldPoint(localPoint Vec3) Pos {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return TransformWorldPoint(transform, localPoint)
}

// GetLocalVector is b3Body_GetLocalVector.
// Get a local vector on a body given a world vector.
func (bodyId BodyID) GetLocalVector(worldVector Vec3) Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return InvRotateVector(transform.Q, worldVector)
}

// GetWorldVector is b3Body_GetWorldVector.
// Get a world vector on a body given a local vector.
func (bodyId BodyID) GetWorldVector(localVector Vec3) Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	transform := getBodyTransformQuick(world, body)
	return RotateVector(transform.Q, localVector)
}

// SetTransform is b3Body_SetTransform.
// Set the world transform of a body. This acts as a teleport and is fairly expensive.
// Note: Generally you should create a body with the intended transform.
// See BodyDef.Position and BodyDef.Rotation.
func (bodyId BodyID) SetTransform(position Pos, rotation Quat) {
	assert(IsValidPosition(position), "b3IsValidPosition( position )")
	assert(IsValidQuat(rotation), "b3IsValidQuat( rotation )")
	assert(bodyId.IsValid(), "b3Body_IsValid( bodyId )")
	world := getWorld(int(bodyId.World0))
	assert(world.locked == false, "world->locked == false")

	if world.recording != nil {
		recArgs := recArgsBodySetTransform{bodyId, position, rotation}
		recWriteBodySetTransform(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)

	bodySim.transform.P = position
	bodySim.transform.Q = rotation
	bodySim.center = TransformWorldPoint(bodySim.transform, bodySim.localCenter)

	rotationMatrix := MakeMatrixFromQuat(bodySim.transform.Q)
	bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))

	bodySim.rotation0 = bodySim.transform.Q
	bodySim.center0 = bodySim.center

	broadPhase := &world.broadPhase

	transform := bodySim.transform
	speculativeDistance := SpeculativeDistance()

	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		aabb := computeFatShapeAABB(shape, transform, speculativeDistance)
		shape.aabb = aabb

		shapeFatAABB := &world.fatAABBs[shapeId]
		if shapeFatAABB.Contains(aabb) == false {
			fatAABB := aabb.Inflate(shape.aabbMargin)
			*shapeFatAABB = fatAABB

			// The body could be disabled
			if shape.proxyKey != NullIndex {
				broadPhase.moveProxy(int(shape.proxyKey), fatAABB)
			}
		}

		shapeId = shape.nextShapeId
	}
}

// GetLinearVelocity is b3Body_GetLinearVelocity.
// Get the linear velocity of a body's center of mass. Usually in meters per second.
func (bodyId BodyID) GetLinearVelocity() Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	state := getBodyState(world, body)
	if state != nil {
		return state.linearVelocity
	}
	return Vec3Zero
}

// GetAngularVelocity is b3Body_GetAngularVelocity.
// Get the angular velocity of a body in radians per second.
func (bodyId BodyID) GetAngularVelocity() Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	state := getBodyState(world, body)
	if state != nil {
		return state.angularVelocity
	}
	return Vec3Zero
}

// SetLinearVelocity is b3Body_SetLinearVelocity.
// Set the linear velocity of a body at the center of mass. Usually in meters per second.
func (bodyId BodyID) SetLinearVelocity(linearVelocity Vec3) {
	assert(IsValidVec3(linearVelocity), "b3IsValidVec3( linearVelocity )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodySetLinearVelocity{bodyId, linearVelocity}
		recWriteBodySetLinearVelocity(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if body.typ == StaticBody {
		return
	}

	if LengthSquared(linearVelocity) > 0.0 {
		wakeBodyWithLock(world, body)
	}

	state := getBodyState(world, body)
	if state == nil {
		return
	}

	state.linearVelocity = linearVelocity
}

// SetAngularVelocity is b3Body_SetAngularVelocity.
// Set the angular velocity of a body in radians per second.
func (bodyId BodyID) SetAngularVelocity(angularVelocity Vec3) {
	assert(IsValidVec3(angularVelocity), "b3IsValidVec3( angularVelocity )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodySetAngularVelocity{bodyId, angularVelocity}
		recWriteBodySetAngularVelocity(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if body.typ == StaticBody {
		return
	}

	// Apply locks to avoid waking
	var w Vec3
	if body.flags&lockAngularX != 0 {
		w.X = 0.0
	} else {
		w.X = angularVelocity.X
	}
	if body.flags&lockAngularY != 0 {
		w.Y = 0.0
	} else {
		w.Y = angularVelocity.Y
	}
	if body.flags&lockAngularZ != 0 {
		w.Z = 0.0
	} else {
		w.Z = angularVelocity.Z
	}

	if LengthSquared(w) != 0.0 {
		wakeBodyWithLock(world, body)
	}

	state := getBodyState(world, body)
	if state == nil {
		return
	}

	state.angularVelocity = w
}

// SetTargetTransform is b3Body_SetTargetTransform.
// Set the velocity to reach the given transform after a given time step.
// The result will be close but maybe not exact. This is meant for kinematic bodies.
// The target is not applied if the velocity would be below the sleep threshold.
// This will optionally wake the body if asleep, but only if the movement is significant.
func (bodyId BodyID) SetTargetTransform(target WorldTransform, timeStep float32, wake bool) {
	assert(IsValidWorldTransform(target), "b3IsValidWorldTransform( target )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodySetTargetTransform{bodyId, target, timeStep, wake}
		recWriteBodySetTargetTransform(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if body.setIndex == disabledSet {
		return
	}

	if body.typ == StaticBody || timeStep <= 0.0 {
		return
	}

	if body.setIndex != awakeSet && wake == false {
		return
	}

	sim := getBodySim(world, body)

	// Compute linear velocity. The center difference is taken in world precision then demoted.
	center1 := sim.center
	center2 := TransformWorldPoint(target, sim.localCenter)
	invTimeStep := 1.0 / timeStep
	linearVelocity := MulSV(invTimeStep, SubPos(center2, center1))

	// Compute angular velocity:
	// q' = 0.5 * w * q
	// <~> ( q2 - q1 ) / dt =  0.5 * w * q1
	// <=> w = 2 * ( q2 - q1 ) * Conjugate( q1 ) / dt
	q1 := sim.transform.Q
	q2 := target.Q

	// Use the shortest arc quaternion
	if DotQuat(q1, q2) < 0.0 {
		q2 = NegateQuat(q2)
	}

	dq := Quat{Sub(q2.V, q1.V), q2.S - q1.S}
	omega := MulQuat(dq, Conjugate(q1))
	angularVelocity := MulSV(2.0*invTimeStep, omega.V)

	// Early out if the body is asleep already and the desired movement is small
	if body.setIndex != awakeSet {
		maxVelocity := Length(linearVelocity) + Length(Mul(angularVelocity, sim.maxExtent))

		// Return if velocity would be sleepy
		if maxVelocity < body.sleepThreshold {
			return
		}

		// Must wake for state to exist
		wakeBodyWithLock(world, body)
	}

	assert(body.setIndex == awakeSet, "body->setIndex == b3_awakeSet")

	state := getBodyState(world, body)
	state.linearVelocity = linearVelocity
	state.angularVelocity = angularVelocity
}

// GetLocalPointVelocity is b3Body_GetLocalPointVelocity.
// Get the linear velocity of a local point attached to a body. Usually in meters per second.
func (bodyId BodyID) GetLocalPointVelocity(localPoint Vec3) Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	state := getBodyState(world, body)
	if state == nil {
		return Vec3Zero
	}

	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))

	r := RotateVector(bodySim.transform.Q, Sub(localPoint, bodySim.localCenter))
	v := Add(state.linearVelocity, Cross(state.angularVelocity, r))
	return v
}

// GetWorldPointVelocity is b3Body_GetWorldPointVelocity.
// Get the linear velocity of a world point attached to a body. Usually in meters per second.
func (bodyId BodyID) GetWorldPointVelocity(worldPoint Pos) Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	state := getBodyState(world, body)
	if state == nil {
		return Vec3Zero
	}

	set := arrayGet(world.solverSets, int(body.setIndex))
	bodySim := arrayGet(set.bodySims, int(body.localIndex))

	r := SubPos(worldPoint, bodySim.center)
	v := Add(state.linearVelocity, Cross(state.angularVelocity, r))
	return v
}

// ApplyForce is b3Body_ApplyForce.
// Apply a force at a world point. If the force is not applied at the center of mass,
// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
// The force is ignored if the body is not awake.
//   - force: the world force vector, usually in newtons (N)
//   - point: the world position of the point of application
//   - wake: option to wake up the body
func (bodyId BodyID) ApplyForce(force Vec3, point Pos, wake bool) {
	assert(IsValidVec3(force), "b3IsValidVec3( force )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyForce{bodyId, force, point, wake}
		recWriteBodyApplyForce(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if wake && body.setIndex >= firstSleepingSet {
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		bodySim := getBodySim(world, body)
		bodySim.force = Add(bodySim.force, force)
		bodySim.torque = Add(bodySim.torque, Cross(SubPos(point, bodySim.center), force))
	}
}

// ApplyForceToCenter is b3Body_ApplyForceToCenter.
// Apply a force to the center of mass. This optionally wakes up the body.
// The force is ignored if the body is not awake.
//   - force: the world force vector, usually in newtons (N).
//   - wake: also wake up the body
func (bodyId BodyID) ApplyForceToCenter(force Vec3, wake bool) {
	assert(IsValidVec3(force), "b3IsValidVec3( force )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyForceToCenter{bodyId, force, wake}
		recWriteBodyApplyForceToCenter(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if wake && body.setIndex >= firstSleepingSet {
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		bodySim := getBodySim(world, body)
		bodySim.force = Add(bodySim.force, force)
	}
}

// ApplyTorque is b3Body_ApplyTorque.
// Apply a torque. This affects the angular velocity without affecting the linear velocity.
// This optionally wakes the body. The torque is ignored if the body is not awake.
//   - torque: the world torque vector, usually in N*m.
//   - wake: also wake up the body
func (bodyId BodyID) ApplyTorque(torque Vec3, wake bool) {
	assert(IsValidVec3(torque), "b3IsValidVec3( torque )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyTorque{bodyId, torque, wake}
		recWriteBodyApplyTorque(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if wake && body.setIndex >= firstSleepingSet {
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		bodySim := getBodySim(world, body)
		bodySim.torque = Add(bodySim.torque, torque)
	}
}

// ApplyLinearImpulse is b3Body_ApplyLinearImpulse.
// Apply an impulse at a point. This immediately modifies the velocity.
// It also modifies the angular velocity if the point of application
// is not at the center of mass. This optionally wakes the body.
// The impulse is ignored if the body is not awake.
//   - impulse: the world impulse vector, usually in N*s or kg*m/s.
//   - point: the world position of the point of application.
//   - wake: also wake up the body
//
// Warning: This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func (bodyId BodyID) ApplyLinearImpulse(impulse Vec3, point Pos, wake bool) {
	assert(IsValidVec3(impulse), "b3IsValidVec3( impulse )")
	assert(IsValidPosition(point), "b3IsValidPosition( point )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyLinearImpulse{bodyId, impulse, point, wake}
		recWriteBodyApplyLinearImpulse(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if wake && body.setIndex >= firstSleepingSet {
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		localIndex := int(body.localIndex)
		set := arrayGet(world.solverSets, awakeSet)
		state := arrayGet(set.bodyStates, localIndex)
		bodySim := arrayGet(set.bodySims, localIndex)

		state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

		maxLinearSpeed := world.maxLinearSpeed
		if LengthSquared(state.linearVelocity) > maxLinearSpeed*maxLinearSpeed {
			state.linearVelocity = MulSV(maxLinearSpeed, Normalize(state.linearVelocity))
		}

		delta := MulMV(bodySim.invInertiaWorld, Cross(SubPos(point, bodySim.center), impulse))
		state.angularVelocity = Add(state.angularVelocity, delta)
	}
}

// ApplyLinearImpulseToCenter is b3Body_ApplyLinearImpulseToCenter.
// Apply an impulse to the center of mass. This immediately modifies the velocity.
// The impulse is ignored if the body is not awake. This optionally wakes the body.
//   - impulse: the world impulse vector, usually in N*s or kg*m/s.
//   - wake: also wake up the body
//
// Warning: This should be used for one-shot impulses. If you need a steady force,
// use a force instead, which will work better with the sub-stepping solver.
func (bodyId BodyID) ApplyLinearImpulseToCenter(impulse Vec3, wake bool) {
	assert(IsValidVec3(impulse), "b3IsValidVec3( impulse )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyLinearImpulseToCenter{bodyId, impulse, wake}
		recWriteBodyApplyLinearImpulseToCenter(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	if wake && body.setIndex >= firstSleepingSet {
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		localIndex := int(body.localIndex)
		set := arrayGet(world.solverSets, awakeSet)
		state := arrayGet(set.bodyStates, localIndex)
		bodySim := arrayGet(set.bodySims, localIndex)
		state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

		maxLinearSpeed := world.maxLinearSpeed
		if LengthSquared(state.linearVelocity) > maxLinearSpeed*maxLinearSpeed {
			state.linearVelocity = MulSV(maxLinearSpeed, Normalize(state.linearVelocity))
		}
	}
}

// ApplyAngularImpulse is b3Body_ApplyAngularImpulse.
// Apply an angular impulse in world space. The impulse is ignored if the body is not awake.
// This optionally wakes the body.
//   - impulse: the world angular impulse vector, usually in units of kg*m*m/s
//   - wake: also wake up the body
//
// Warning: This should be used for one-shot impulses. If you need a steady torque,
// use a torque instead, which will work better with the sub-stepping solver.
func (bodyId BodyID) ApplyAngularImpulse(impulse Vec3, wake bool) {
	assert(IsValidVec3(impulse), "b3IsValidVec3( impulse )")
	assert(bodyId.IsValid(), "b3Body_IsValid( bodyId )")

	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyApplyAngularImpulse{bodyId, impulse, wake}
		recWriteBodyApplyAngularImpulse(world.recording, &recArgs)
	}

	id := int(bodyId.Index1 - 1)
	body := arrayGet(world.bodies, id)
	assert(body.generation == bodyId.Generation, "body->generation == bodyId.generation")

	if wake && body.setIndex >= firstSleepingSet {
		// this will not invalidate body pointer
		wakeBodyWithLock(world, body)
	}

	if body.setIndex == awakeSet {
		localIndex := int(body.localIndex)
		set := arrayGet(world.solverSets, awakeSet)
		state := arrayGet(set.bodyStates, localIndex)
		bodySim := arrayGet(set.bodySims, localIndex)

		localImpulse := InvRotateVector(bodySim.transform.Q, impulse)
		localAngularVelocityDelta := MulMV(bodySim.invInertiaLocal, localImpulse)
		state.angularVelocity =
			Add(state.angularVelocity, RotateVector(bodySim.transform.Q, localAngularVelocityDelta))
	}
}

// GetType is b3Body_GetType.
// Get the body type: static, kinematic, or dynamic
func (bodyId BodyID) GetType() BodyType {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.typ
}

// SetType is b3Body_SetType.
// Change the body type. This is an expensive operation. This automatically updates the mass
// properties regardless of the automatic mass setting.
//
// This should follow similar steps as you would get destroying and recreating the body, shapes, and joints.
// Contacts are difficult to preserve because the broad-phase pairs change, so I just destroy them.
// todo with a bit more effort I could support an option to let the body sleep
//
// Revised steps:
// 1 Skip disabled bodies
// 2 Destroy all contacts on the body
// 3 Wake the body
// 4 For all joints attached to the body
//   - wake attached bodies
//   - remove from island
//   - move to static set temporarily
//
// 5 Change the body type and transfer the body
// 6 If the body was static
//   - create an island for the body
//     Else if the body is becoming static
//   - remove it from the island
//
// 7 For all joints
//   - if either body is non-static
//   - link into island
//   - transfer to constraint graph
//
// 8 For all shapes
//   - Destroy proxy in old tree
//   - Create proxy in new tree
//
// Notes:
//   - the implementation below tries to minimize the number of predicates, so some
//     operations may have no effect, such as transferring a joint to the same set
//
// PORT: the parameter type is typ (Go keyword); the local solver set pointers staticSet and
// awakeSet are staticSolverSet and awakeSolverSet so the set index constants stay visible.
func (bodyId BodyID) SetType(typ BodyType) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetType{bodyId, int32(typ)}
		recWriteBodySetType(world.recording, &recArgs)
	}

	world.locked = true
	body := getBodyFullId(world, bodyId)

	originalType := body.typ
	if originalType == typ {
		world.locked = false
		return
	}

	if typ != StaticBody {
		shapeId := body.headShapeId
		for shapeId != NullIndex {
			shape := arrayGet(world.shapes, int(shapeId))
			if shape.typ == CompoundShape || shape.typ == HeightShape {
				// Setting the body type is not supported for bodies with compound shapes
				// PORT: upstream returns with world->locked still true; ported as is.
				return
			}

			shapeId = shape.nextShapeId
		}
	}

	// Stage 1: skip disabled bodies
	if body.setIndex == disabledSet {
		// Disabled bodies don't change solver sets or islands when they change type.
		body.typ = typ

		if typ == DynamicBody {
			body.flags |= dynamicFlag
		} else {
			body.flags &^= dynamicFlag
		}

		syncBodyFlags(world, body)

		// Body type affects the mass properties
		updateBodyMassData(world, body)
		world.locked = false
		return
	}

	// Stage 2: destroy all contacts but don't wake bodies (because we don't need to)
	wakeBodies := false
	destroyBodyContacts(world, body, wakeBodies)

	// Stage 3: wake this body (does nothing if body is static), otherwise it will also wake
	// all bodies in the same sleeping solver set.
	wakeBody(world, body)

	// Stage 4: move joints to temporary storage
	staticSolverSet := arrayGet(world.solverSets, staticSet)

	jointKey := body.headJointKey
	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))
		jointKey = joint.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if joint.setIndex == disabledSet {
			continue
		}

		// Wake attached bodies. The b3WakeBody call above does not wake bodies
		// attached to a static body. But it is necessary because the body may have
		// no joints.
		bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
		bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))
		wakeBody(world, bodyA)
		wakeBody(world, bodyB)

		// Remove joint from island
		unlinkJoint(world, joint)

		// It is necessary to transfer all joints to the static set
		// so they can be added to the constraint graph below and acquire consistent colors.
		jointSourceSet := arrayGet(world.solverSets, int(joint.setIndex))
		transferJoint(world, staticSolverSet, jointSourceSet, joint)
	}

	// Stage 5: change the body type and transfer body
	body.typ = typ

	if typ == DynamicBody {
		body.flags |= dynamicFlag
	} else {
		body.flags &^= dynamicFlag
	}

	awakeSolverSet := arrayGet(world.solverSets, awakeSet)
	sourceSet := arrayGet(world.solverSets, int(body.setIndex))
	targetSet := awakeSolverSet
	if typ == StaticBody {
		targetSet = staticSolverSet
	}

	// Transfer body
	transferBody(world, targetSet, sourceSet, body)

	// Stage 6: update island participation for the body
	if originalType == StaticBody {
		// Create island for body
		createIslandForBody(world, awakeSet, body)
	} else if typ == StaticBody {
		// Remove body from island.
		removeBodyFromIsland(world, body)
	}

	// Stage 7: Transfer joints to the target set
	jointKey = body.headJointKey
	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))

		jointKey = joint.edges[edgeIndex].nextKey

		// Joint may be disabled by other body
		if joint.setIndex == disabledSet {
			continue
		}

		// All joints were transferred to the static set in an earlier stage
		assert(joint.setIndex == staticSet, "joint->setIndex == b3_staticSet")

		bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
		bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))
		assert(bodyA.setIndex == staticSet || bodyA.setIndex == awakeSet, "bodyA->setIndex == b3_staticSet || bodyA->setIndex == b3_awakeSet")
		assert(bodyB.setIndex == staticSet || bodyB.setIndex == awakeSet, "bodyB->setIndex == b3_staticSet || bodyB->setIndex == b3_awakeSet")

		if bodyA.typ == DynamicBody || bodyB.typ == DynamicBody {
			transferJoint(world, awakeSolverSet, staticSolverSet, joint)
		}
	}

	// Recreate shape proxies in broadphase
	transform := getBodyTransformQuick(world, body)
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))

		// Setting the body type is not supported for bodies with compound shapes
		assert(shape.typ != CompoundShape, "shape->type != b3_compoundShape")

		shapeId = shape.nextShapeId
		destroyShapeProxy(shape, &world.broadPhase)
		forcePairCreation := true
		createShapeProxy(world, shape, typ, transform, forcePairCreation)
	}

	// Relink all joints
	jointKey = body.headJointKey
	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))
		jointKey = joint.edges[edgeIndex].nextKey

		otherEdgeIndex := edgeIndex ^ 1
		otherBodyId := joint.edges[otherEdgeIndex].bodyId
		otherBody := arrayGet(world.bodies, int(otherBodyId))

		if otherBody.setIndex == disabledSet {
			continue
		}

		if body.typ != DynamicBody && otherBody.typ != DynamicBody {
			continue
		}

		linkJoint(world, joint)
	}

	syncBodyFlags(world, body)

	// Body type affects the mass
	updateBodyMassData(world, body)

	validateSolverSets(world)
	validateIsland(world, int(body.islandId))

	world.locked = false
}

// SetName is b3Body_SetName.
// Set the body name.
func (bodyId BodyID) SetName(name string) {
	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodySetName{bodyId, name}
		recWriteBodySetName(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	body.nameId = addName(&world.names, name)
}

// GetName is b3Body_GetName.
// Get the body name. Returns an empty string if the name isn't set.
func (bodyId BodyID) GetName() string {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return findNameWithDefault(&world.names, body.nameId, "")
}

// SetUserData is b3Body_SetUserData.
// Set the user data for a body
func (bodyId BodyID) SetUserData(userData any) {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	body.userData = userData
}

// GetUserData is b3Body_GetUserData.
// Get the user data stored in a body
func (bodyId BodyID) GetUserData() any {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.userData
}

// GetMass is b3Body_GetMass.
// Get the mass of the body, usually in kilograms
func (bodyId BodyID) GetMass() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.mass
}

// GetLocalRotationalInertia is b3Body_GetLocalRotationalInertia.
// Get the rotational inertia of the body in local space, usually in kg*m^2
func (bodyId BodyID) GetLocalRotationalInertia() Matrix3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.inertia
}

// GetInverseMass is b3Body_GetInverseMass.
// Get the inverse mass of the body, usually in 1/kilograms
func (bodyId BodyID) GetInverseMass() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	sim := getBodySim(world, body)
	return sim.invMass
}

// GetWorldInverseRotationalInertia is b3Body_GetWorldInverseRotationalInertia.
// Get the inverse rotational inertia of the body in world space, usually in 1/kg*m^2
func (bodyId BodyID) GetWorldInverseRotationalInertia() Matrix3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	sim := getBodySim(world, body)
	return sim.invInertiaWorld
}

// GetLocalCenter is b3Body_GetLocalCenter.
// Get the center of mass position of the body in local space
func (bodyId BodyID) GetLocalCenter() Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.localCenter
}

// GetWorldCenter is b3Body_GetWorldCenter.
// Get the center of mass position of the body in world space
func (bodyId BodyID) GetWorldCenter() Pos {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.center
}

// SetMassData is b3Body_SetMassData.
// Override the body's mass properties. Normally this is computed automatically using the
// shape geometry and density. This information is lost if a shape is added or removed or if the
// body type changes.
func (bodyId BodyID) SetMassData(massData MassData) {
	assert(IsValidFloat(massData.Mass) && massData.Mass >= 0.0, "b3IsValidFloat( massData.mass ) && massData.mass >= 0.0f")
	assert(IsValidMatrix3(massData.Inertia), "b3IsValidMatrix3( massData.inertia )")
	assert(IsValidVec3(massData.Center), "b3IsValidVec3( massData.center )")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetMassData{bodyId, massData}
		recWriteBodySetMassData(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)

	// Mass is no longer dirty
	body.flags &^= dirtyMass
	syncBodyFlags(world, body)

	body.mass = massData.Mass
	bodySim.localCenter = massData.Center

	oldCenter := bodySim.center
	center := TransformWorldPoint(bodySim.transform, massData.Center)
	bodySim.center = center
	bodySim.center0 = center
	if body.mass > 0.0 {
		bodySim.invMass = 1.0 / body.mass
	} else {
		bodySim.invMass = 0.0
	}
	bodySim.inertia = massData.Inertia

	// Update center of mass velocity
	state := getBodyState(world, body)
	if state != nil {
		deltaLinear := Cross(state.angularVelocity, SubPos(bodySim.center, oldCenter))
		state.linearVelocity = Add(state.linearVelocity, deltaLinear)
	}

	det := Det(bodySim.inertia)
	assert(det >= 0.0, "det >= 0.0f")

	if det > 0.0 {
		// This call is faster than b3Invert
		bodySim.invInertiaLocal = InvertT(bodySim.inertia)

		rotationMatrix := MakeMatrixFromQuat(bodySim.transform.Q)
		bodySim.invInertiaWorld = MulMM(MulMM(rotationMatrix, bodySim.invInertiaLocal), Transpose(rotationMatrix))
	} else {
		bodySim.invInertiaLocal = Mat3Zero
		bodySim.invInertiaWorld = Mat3Zero
	}

	// Apply fixed rotation
	if (bodySim.flags & fixedRotation) == fixedRotation {
		bodySim.inertia = Mat3Zero
		bodySim.invInertiaLocal = Mat3Zero
		bodySim.invInertiaWorld = Mat3Zero
	}

	updateIsotropicInertia(world, body, bodySim)

	// Update extents using supplied mass center.
	bodySim.minExtent = Huge()
	bodySim.maxExtent = Vec3Zero
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		s := arrayGet(world.shapes, int(shapeId))
		extent := computeShapeExtent(s, massData.Center)
		bodySim.minExtent = MinFloat(bodySim.minExtent, extent.minExtent)
		bodySim.maxExtent = Max(bodySim.maxExtent, extent.maxExtent)
		shapeId = s.nextShapeId
	}

	// When the center of mass changes, any cached contact manifold becomes invalid.
	edgeKey := body.headContactKey
	for edgeKey != NullIndex {
		contactId := edgeKey >> 1
		edgeIndex := edgeKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		contact.flags &^= relativeTransformValid

		edgeKey = contact.edges[edgeIndex].nextKey
	}
}

// GetMassData is b3Body_GetMassData.
// Get the mass data for a body
func (bodyId BodyID) GetMassData() MassData {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	massData := MassData{body.mass, bodySim.localCenter, bodySim.inertia}
	return massData
}

// ApplyMassFromShapes is b3Body_ApplyMassFromShapes.
// This updates the mass properties to the sum of the mass properties of the shapes.
// This normally does not need to be called unless you called SetMassData to override
// the mass and you later want to reset the mass.
// You may also use this when automatic mass computation has been disabled.
// You should call this regardless of body type.
func (bodyId BodyID) ApplyMassFromShapes() {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyApplyMassFromShapes{bodyId}
		recWriteBodyApplyMassFromShapes(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	updateBodyMassData(world, body)
}

// SetLinearDamping is b3Body_SetLinearDamping.
// Adjust the linear damping. Normally this is set in BodyDef before creation.
func (bodyId BodyID) SetLinearDamping(linearDamping float32) {
	assert(IsValidFloat(linearDamping) && linearDamping >= 0.0, "b3IsValidFloat( linearDamping ) && linearDamping >= 0.0f")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetLinearDamping{bodyId, linearDamping}
		recWriteBodySetLinearDamping(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	bodySim.linearDamping = linearDamping
}

// GetLinearDamping is b3Body_GetLinearDamping.
// Get the current linear damping.
func (bodyId BodyID) GetLinearDamping() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.linearDamping
}

// SetAngularDamping is b3Body_SetAngularDamping.
// Adjust the angular damping. Normally this is set in BodyDef before creation.
func (bodyId BodyID) SetAngularDamping(angularDamping float32) {
	assert(IsValidFloat(angularDamping) && angularDamping >= 0.0, "b3IsValidFloat( angularDamping ) && angularDamping >= 0.0f")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetAngularDamping{bodyId, angularDamping}
		recWriteBodySetAngularDamping(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	bodySim.angularDamping = angularDamping
}

// GetAngularDamping is b3Body_GetAngularDamping.
// Get the current angular damping.
func (bodyId BodyID) GetAngularDamping() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.angularDamping
}

// SetGravityScale is b3Body_SetGravityScale.
// Adjust the gravity scale. Normally this is set in BodyDef before creation.
// See BodyDef.GravityScale.
func (bodyId BodyID) SetGravityScale(gravityScale float32) {
	assert(bodyId.IsValid(), "b3Body_IsValid( bodyId )")
	assert(IsValidFloat(gravityScale), "b3IsValidFloat( gravityScale )")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetGravityScale{bodyId, gravityScale}
		recWriteBodySetGravityScale(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	bodySim.gravityScale = gravityScale
}

// GetGravityScale is b3Body_GetGravityScale.
// Get the current gravity scale
func (bodyId BodyID) GetGravityScale() float32 {
	assert(bodyId.IsValid(), "b3Body_IsValid( bodyId )")
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.gravityScale
}

// IsAwake is b3Body_IsAwake.
// Returns true if this body is awake
func (bodyId BodyID) IsAwake() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.setIndex == awakeSet
}

// SetAwake is b3Body_SetAwake.
// Wake a body from sleep. This wakes the entire island the body is touching.
// Warning: Putting a body to sleep will put the entire island of bodies touching this body to sleep,
// which can be expensive and possibly unintuitive.
func (bodyId BodyID) SetAwake(awake bool) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetAwake{bodyId, awake}
		recWriteBodySetAwake(world.recording, &recArgs)
	}

	world.locked = true

	body := getBodyFullId(world, bodyId)

	if awake && body.setIndex >= firstSleepingSet {
		wakeBody(world, body)
	} else if awake == false && body.setIndex == awakeSet {
		island := arrayGet(world.islands, int(body.islandId))
		if island.constraintRemoveCount > 0 {
			// Must split the island before sleeping. This is expensive.
			splitIsland(world, int(body.islandId))
		}

		trySleepIsland(world, int(body.islandId))
	}

	world.locked = false
}

// IsEnabled is b3Body_IsEnabled.
// Returns true if this body is enabled
func (bodyId BodyID) IsEnabled() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.setIndex != disabledSet
}

// IsSleepEnabled is b3Body_IsSleepEnabled.
// Returns true if sleeping is enabled for this body
func (bodyId BodyID) IsSleepEnabled() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return (body.flags & enableSleep) == enableSleep
}

// SetSleepThreshold is b3Body_SetSleepThreshold.
// Set the sleep threshold, usually in meters per second
func (bodyId BodyID) SetSleepThreshold(sleepThreshold float32) {
	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodySetSleepThreshold{bodyId, sleepThreshold}
		recWriteBodySetSleepThreshold(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	body.sleepThreshold = sleepThreshold
}

// GetSleepThreshold is b3Body_GetSleepThreshold.
// Get the sleep threshold, usually in meters per second.
func (bodyId BodyID) GetSleepThreshold() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.sleepThreshold
}

// SetSafetyFactor is b3Body_SetSafetyFactor.
// Set the continuous collision safety factor. Smaller is safer but can lead to hitching. Recommended range [0.01, 0.5]. Non-dimensional.
func (bodyId BodyID) SetSafetyFactor(safetyFactor float32) {
	assert(IsValidFloat(safetyFactor) && safetyFactor >= 0.0, "b3IsValidFloat( safetyFactor ) && safetyFactor >= 0.0f")

	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetSafetyFactor{bodyId, safetyFactor}
		recWriteBodySetSafetyFactor(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	body.safetyFactor = safetyFactor
}

// GetSafetyFactor is b3Body_GetSafetyFactor.
// Get the continuous collision safety factor. Non-dimensional.
func (bodyId BodyID) GetSafetyFactor() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return body.safetyFactor
}

// EnableSleep is b3Body_EnableSleep.
// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
// PORT: the parameter enableSleep is named flag so the b3_enableSleep constant stays visible.
func (bodyId BodyID) EnableSleep(flag bool) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyEnableSleep{bodyId, flag}
		recWriteBodyEnableSleep(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)

	current := (body.flags & enableSleep) == enableSleep
	if flag == current {
		return
	}

	world.locked = true

	if flag {
		body.flags = body.flags | enableSleep
	} else {
		body.flags = body.flags &^ enableSleep
	}
	syncBodyFlags(world, body)

	if flag == false {
		wakeBody(world, body)
	}

	world.locked = false
}

// Disable is b3Body_Disable.
// Disable a body by removing it completely from the simulation. This is expensive.
//
// Disabling a body requires a lot of detailed bookkeeping, but it is a valuable feature.
// The most challenging aspect that joints may connect to bodies that are not disabled.
// PORT: the local solver set pointer disabledSet is disabledSolverSet so the set index constant
// stays visible.
func (bodyId BodyID) Disable() {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyDisable{bodyId}
		recWriteBodyDisable(world.recording, &recArgs)
	}

	world.locked = true

	body := getBodyFullId(world, bodyId)
	if body.setIndex == disabledSet {
		world.locked = false
		return
	}

	// Destroy contacts and wake bodies touching this body. This avoid floating bodies.
	// This is necessary even for static bodies.
	wakeBodies := true
	destroyBodyContacts(world, body, wakeBodies)

	// The current solver set of the body
	set := arrayGet(world.solverSets, int(body.setIndex))

	// Disabled bodies and connected joints are moved to the disabled set
	disabledSolverSet := arrayGet(world.solverSets, disabledSet)

	// Unlink joints and transfer them to the disabled set
	jointKey := body.headJointKey
	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))
		jointKey = joint.edges[edgeIndex].nextKey

		// joint may already be disabled by other body
		if joint.setIndex == disabledSet {
			continue
		}

		assert(joint.setIndex == set.setIndex || set.setIndex == staticSet || joint.setIndex == staticSet,
			"joint->setIndex == set->setIndex || set->setIndex == b3_staticSet || joint->setIndex == b3_staticSet")

		// Remove joint from island
		unlinkJoint(world, joint)

		// Transfer joint to disabled set
		jointSet := arrayGet(world.solverSets, int(joint.setIndex))
		transferJoint(world, disabledSolverSet, jointSet, joint)
	}

	// Remove shapes from broad-phase
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId
		destroyShapeProxy(shape, &world.broadPhase)
	}

	// Disabled bodies are not in an island. If the island becomes empty it will be destroyed.
	removeBodyFromIsland(world, body)

	// Transfer body sim
	transferBody(world, disabledSolverSet, set, body)

	validateConnectivity(world)
	validateSolverSets(world)

	world.locked = false
}

// Enable is b3Body_Enable.
// Enable a body by adding it to the simulation. This is expensive.
// PORT: the local solver set pointer disabledSet is disabledSolverSet so the set index constant
// stays visible.
func (bodyId BodyID) Enable() {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyEnable{bodyId}
		recWriteBodyEnable(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	if body.setIndex != disabledSet {
		return
	}

	disabledSolverSet := arrayGet(world.solverSets, disabledSet)
	setId := awakeSet
	if body.typ == StaticBody {
		setId = staticSet
	}
	targetSet := arrayGet(world.solverSets, setId)

	transferBody(world, targetSet, disabledSolverSet, body)

	transform := getBodyTransformQuick(world, body)

	// Add shapes to broad-phase
	proxyType := body.typ
	forcePairCreation := true
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		shapeId = shape.nextShapeId

		createShapeProxy(world, shape, proxyType, transform, forcePairCreation)
	}

	if setId != staticSet {
		createIslandForBody(world, setId, body)
	}

	// Transfer joints. If the other body is disabled, don't transfer.
	// If the other body is sleeping, wake it.
	jointKey := body.headJointKey
	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))
		assert(joint.setIndex == disabledSet, "joint->setIndex == b3_disabledSet")
		assert(joint.islandId == NullIndex, "joint->islandId == B3_NULL_INDEX")

		jointKey = joint.edges[edgeIndex].nextKey

		bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
		bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))

		if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
			// one body is still disabled
			continue
		}

		// Transfer joint first
		var jointSetId int
		if bodyA.typ != DynamicBody && bodyB.typ != DynamicBody {
			jointSetId = staticSet
		} else if bodyA.setIndex == staticSet {
			jointSetId = int(bodyB.setIndex)
		} else {
			jointSetId = int(bodyA.setIndex)
		}

		jointSet := arrayGet(world.solverSets, jointSetId)
		transferJoint(world, jointSet, disabledSolverSet, joint)

		// Now that the joint is in the correct set, I can link the joint in the island.
		if jointSetId != staticSet {
			linkJoint(world, joint)
		}
	}

	validateSolverSets(world)
}

// SetMotionLocks is b3Body_SetMotionLocks.
// Set the motion locks on this body.
func (bodyId BodyID) SetMotionLocks(locks MotionLocks) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetMotionLocks{bodyId, locks}
		recWriteBodySetMotionLocks(world.recording, &recArgs)
	}

	var newLocks uint32 = 0
	if locks.LinearX {
		newLocks |= lockLinearX
	}
	if locks.LinearY {
		newLocks |= lockLinearY
	}
	if locks.LinearZ {
		newLocks |= lockLinearZ
	}
	if locks.AngularX {
		newLocks |= lockAngularX
	}
	if locks.AngularY {
		newLocks |= lockAngularY
	}
	if locks.AngularZ {
		newLocks |= lockAngularZ
	}

	body := getBodyFullId(world, bodyId)
	if (body.flags & allLocks) == newLocks {
		return
	}

	fixedRotation1 := (body.flags & fixedRotation) == fixedRotation
	fixedRotation2 := (newLocks & fixedRotation) == fixedRotation

	body.flags &^= allLocks
	body.flags |= newLocks

	syncBodyFlags(world, body)

	state := getBodyState(world, body)

	if state != nil {
		if locks.LinearX {
			state.linearVelocity.X = 0.0
		}

		if locks.LinearY {
			state.linearVelocity.Y = 0.0
		}

		if locks.LinearZ {
			state.linearVelocity.Z = 0.0
		}

		if locks.AngularX {
			state.angularVelocity.X = 0.0
		}

		if locks.AngularY {
			state.angularVelocity.Y = 0.0
		}

		if locks.AngularZ {
			state.angularVelocity.Z = 0.0
		}
	}

	if fixedRotation1 != fixedRotation2 {
		updateBodyMassData(world, body)
	}
}

// GetMotionLocks is b3Body_GetMotionLocks.
// Get the motion locks for this body.
func (bodyId BodyID) GetMotionLocks() MotionLocks {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)

	var locks MotionLocks
	locks.LinearX = (body.flags & lockLinearX) != 0
	locks.LinearY = (body.flags & lockLinearY) != 0
	locks.LinearZ = (body.flags & lockLinearZ) != 0
	locks.AngularX = (body.flags & lockAngularX) != 0
	locks.AngularY = (body.flags & lockAngularY) != 0
	locks.AngularZ = (body.flags & lockAngularZ) != 0
	return locks
}

// SetBullet is b3Body_SetBullet.
// Set this body to be a bullet. A bullet does continuous collision detection
// against dynamic bodies (but not other bullets).
func (bodyId BodyID) SetBullet(flag bool) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodySetBullet{bodyId, flag}
		recWriteBodySetBullet(world.recording, &recArgs)
	}

	var newFlag uint32 = 0
	if flag {
		newFlag = isBullet
	}

	body := getBodyFullId(world, bodyId)
	if (body.flags & isBullet) == newFlag {
		return
	}

	body.flags &^= isBullet
	body.flags |= newFlag

	syncBodyFlags(world, body)
}

// IsBullet is b3Body_IsBullet.
// Is this body a bullet?
func (bodyId BodyID) IsBullet() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return (body.flags & isBullet) != 0
}

// AllowFastRotation is b3Body_AllowFastRotation.
// Allow this body to rotate fast. Useful for axially symmetric bodies, such as vehicle wheels.
// Normally rotation speed is clamped to improve CCD. However, this clamping is unnecessary for
// bodies that only rotate fast around an axis of symmetry.
func (bodyId BodyID) AllowFastRotation(flag bool) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyAllowFastRotation{bodyId, flag}
		recWriteBodyAllowFastRotation(world.recording, &recArgs)
	}

	var newFlag uint32 = 0
	if flag {
		newFlag = allowFastRotation
	}

	body := getBodyFullId(world, bodyId)
	if (body.flags & allowFastRotation) == newFlag {
		return
	}

	body.flags &^= allowFastRotation
	body.flags |= newFlag

	syncBodyFlags(world, body)
}

// IsFastRotationAllowed is b3Body_IsFastRotationAllowed.
// Is this body allowed to rotate fast?
func (bodyId BodyID) IsFastRotationAllowed() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return (body.flags & allowFastRotation) != 0
}

// EnableContactRecycling is b3Body_EnableContactRecycling.
// Enable or disable contact recycling for this body. Contact recycling is a performance optimization
// that reuses contact manifolds when bodies move slightly. Disabling it can avoid ghost collisions
// on characters at the cost of higher per-step work. Existing contacts retain their prior setting;
// only contacts created after this call see the new value.
// See BodyDef.EnableContactRecycling.
func (bodyId BodyID) EnableContactRecycling(flag bool) {
	world := getUnlockedWorld(int(bodyId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsBodyEnableContactRecycling{bodyId, flag}
		recWriteBodyEnableContactRecycling(world.recording, &recArgs)
	}

	var newFlag uint32 = 0
	if flag {
		newFlag = enableContactRecycling
	}

	body := getBodyFullId(world, bodyId)
	if (body.flags & enableContactRecycling) == newFlag {
		return
	}

	body.flags &^= enableContactRecycling
	body.flags |= newFlag

	syncBodyFlags(world, body)
}

// IsContactRecyclingEnabled is b3Body_IsContactRecyclingEnabled.
// Is contact recycling enabled on this body?
func (bodyId BodyID) IsContactRecyclingEnabled() bool {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return (body.flags & enableContactRecycling) != 0
}

// EnableHitEvents is b3Body_EnableHitEvents.
// Enable/disable hit events on all shapes
// See ShapeDef.EnableHitEvents.
func (bodyId BodyID) EnableHitEvents(flag bool) {
	world := getWorld(int(bodyId.World0))

	if world.recording != nil {
		recArgs := recArgsBodyEnableHitEvents{bodyId, flag}
		recWriteBodyEnableHitEvents(world.recording, &recArgs)
	}

	body := getBodyFullId(world, bodyId)
	shapeId := body.headShapeId
	for shapeId != NullIndex {
		shape := arrayGet(world.shapes, int(shapeId))
		if flag {
			shape.flags = shape.flags | uint8(enableHitEvents)
		} else {
			shape.flags = shape.flags &^ uint8(enableHitEvents)
		}
		shapeId = shape.nextShapeId
	}
}

// GetWorld is b3Body_GetWorld.
// Get the world that owns this body
func (bodyId BodyID) GetWorld() WorldID {
	world := getWorld(int(bodyId.World0))
	return WorldID{uint16(bodyId.World0 + 1), world.generation}
}

// GetShapeCount is b3Body_GetShapeCount.
// Get the number of shapes on this body
func (bodyId BodyID) GetShapeCount() int {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return int(body.shapeCount)
}

// GetShapes is b3Body_GetShapes.
// Get the shape ids for all shapes on this body, up to the provided capacity.
// Returns the number of shape ids stored in the user array
// PORT: shapeArray + capacity become one slice; the capacity is len(shapeArray).
func (bodyId BodyID) GetShapes(shapeArray []ShapeID) int {
	capacity := len(shapeArray)
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	shapeId := body.headShapeId
	shapeCount := 0
	for shapeId != NullIndex && shapeCount < capacity {
		shape := arrayGet(world.shapes, int(shapeId))
		id := ShapeID{shape.id + 1, bodyId.World0, shape.generation}
		shapeArray[shapeCount] = id
		shapeCount += 1

		shapeId = shape.nextShapeId
	}

	return shapeCount
}

// GetJointCount is b3Body_GetJointCount.
// Get the number of joints on this body
func (bodyId BodyID) GetJointCount() int {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	return int(body.jointCount)
}

// GetJoints is b3Body_GetJoints.
// Get the joint ids for all joints on this body, up to the provided capacity
// Returns the number of joint ids stored in the user array
// PORT: jointArray + capacity become one slice; the capacity is len(jointArray).
func (bodyId BodyID) GetJoints(jointArray []JointID) int {
	capacity := len(jointArray)
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	jointKey := body.headJointKey

	jointCount := 0
	for jointKey != NullIndex && jointCount < capacity {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1

		joint := arrayGet(world.joints, int(jointId))

		id := JointID{jointId + 1, bodyId.World0, joint.generation}
		jointArray[jointCount] = id
		jointCount += 1

		jointKey = joint.edges[edgeIndex].nextKey
	}

	return jointCount
}

// shouldBodiesCollide is b3ShouldBodiesCollide.
func shouldBodiesCollide(world *world, bodyA, bodyB *body) bool {
	if bodyA.typ != DynamicBody && bodyB.typ != DynamicBody {
		return false
	}

	var jointKey int32
	var otherBodyId int32
	if bodyA.jointCount < bodyB.jointCount {
		jointKey = bodyA.headJointKey
		otherBodyId = bodyB.id
	} else {
		jointKey = bodyB.headJointKey
		otherBodyId = bodyA.id
	}

	for jointKey != NullIndex {
		jointId := jointKey >> 1
		edgeIndex := jointKey & 1
		otherEdgeIndex := edgeIndex ^ 1

		joint := arrayGet(world.joints, int(jointId))
		if joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId {
			return false
		}

		jointKey = joint.edges[edgeIndex].nextKey
	}

	return true
}

// GetMinExtent is b3Body_GetMinExtent.
// The minimum distance from any shape to the shape centroid.
func (bodyId BodyID) GetMinExtent() float32 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.minExtent
}

// GetMaxExtent is b3Body_GetMaxExtent.
// The maximum extent vector from any point on the body shapes to the center of mass.
func (bodyId BodyID) GetMaxExtent() Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return bodySim.maxExtent
}

// GetMaxExtentOrigin is b3Body_GetMaxExtentOrigin.
// The maximum extent vector from any point on the body shapes to the body origin. Conservative.
func (bodyId BodyID) GetMaxExtentOrigin() Vec3 {
	world := getWorld(int(bodyId.World0))
	body := getBodyFullId(world, bodyId)
	bodySim := getBodySim(world, body)
	return Add(bodySim.maxExtent, Abs(bodySim.localCenter))
}
