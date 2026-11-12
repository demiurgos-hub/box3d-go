// Port of src/recording.h and src/recording.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"math"
	"os"
	"unsafe"
)

// ---- recording.h ----

// PORT: recording.h is ported in full (types, constants, inline). The arg structs and opcode
// constants it generates from the X-macro manifest are in recording_ops.go, together with the
// writer expansions of recording.c. The functions it declares are defined in the recording.c
// section below.
//
// B3_REC( world, Name, args... ) expands at the call site to
//
//	if world.recording != nil {
//		recArgs := recArgsName{args...}
//		recWriteName(world.recording, &recArgs)
//	}
//
// and B3_REC_CREATE( world, Name, id, args... ) to
//
//	if world.recording != nil {
//		recCreateArgs := recArgsName{args...}
//		recWriteRetName(world.recording, &recCreateArgs, id)
//	}

// FNV-1a 64-bit constants
const (
	snapFnvInit  uint64 = 14695981039346656037 // B3_SNAP_FNV_INIT
	snapFnvPrime uint64 = 1099511628211        // B3_SNAP_FNV_PRIME
)

// fnvMixPosition is b3FnvMixPosition.
// Mix a world position at full width so the determinism gate validates past float precision
// when the body is far from the origin.
// PORT: float build only (BOX3D_DOUBLE_PRECISION is not ported).
func fnvMixPosition(hash uint64, p Pos) uint64 {
	fx := math.Float32bits(p.X)
	fy := math.Float32bits(p.Y)
	fz := math.Float32bits(p.Z)
	bx, by, bz := uint64(fx), uint64(fy), uint64(fz)
	hash = (hash ^ bx) * snapFnvPrime
	hash = (hash ^ by) * snapFnvPrime
	hash = (hash ^ bz) * snapFnvPrime
	return hash
}

// Magic 'B3RC' in little-endian: bytes B(0x42) 3(0x33) R(0x52) C(0x43)
const recMagic uint32 = 0x43523342 // B3_REC_MAGIC

// Major recording version is bumped when writers change.
// Major version 6 widened the body def with the safety factor.
const recVersionMajor = 6 // B3_REC_VERSION_MAJOR

// Minor tracks op-stream additions that keep the 48 byte header shape.
// Minor version 7 made the contact body sim locators persistent
const recVersionMinor = 9 // B3_REC_VERSION_MINOR

// recHeader is b3RecHeader.
// File header, fixed 48 bytes. Contains the registry locator so the player
// can load geometry before replaying any ops.
type recHeader struct {
	magic             uint32 // 'B3RC' = 0x43523342
	versionMajor      uint16 // B3_REC_VERSION_MAJOR
	versionMinor      uint16 // B3_REC_VERSION_MINOR
	pointerWidth      uint8  // sizeof(void*), gates POD-def struct layout
	bigEndian         uint8  // 0 on all supported targets
	validationEnabled uint8  // 1 if built with BOX3D_VALIDATE, diagnostic only
	reserved          uint8
	lengthScale       float32 // b3GetLengthUnitsPerMeter()
	reserved2         uint32
	reserved3         uint32 // explicit pad so the 64-bit fields align with no implicit gap
	snapshotSize      uint64 // bytes of snapshot blob after the header (0 in Phase 1)
	registryOffset    uint64 // absolute offset to trailing registry block, backpatched at stop
	registryByteCount uint64 // size of the registry block
}

// PORT: _Static_assert( sizeof( b3RecHeader ) == 48 ) as constant expressions.
var _ = [1]struct{}{}[unsafe.Sizeof(recHeader{})-48] // recording header must be 48 bytes
var _ = [1]struct{}{}[48-unsafe.Sizeof(recHeader{})] // recording header must be 48 bytes

// recBuffer is b3RecBuffer.
// Growable append-only byte buffer. Doubles on demand. countOnly mode tallies size without
// allocating, used to size a buffer cheaply before a second filling pass.
// PORT: uint8_t* data is a slice; capacity and size are kept.
type recBuffer struct {
	data      []byte
	capacity  int32
	size      int32
	countOnly bool
}

// geometryKind is b3GeometryKind: geometry kinds for the trailing registry section
type geometryKind int32

const (
	geometryHull geometryKind = iota
	geometryMesh
	geometryHeightField
	geometryCompound
)

// geometryEntry is b3GeometryEntry.
// One entry per unique geometry blob. id == index in the entries array.
// hashNext chains entries that share a content hash so dedup stays exact under a hash collision,
// which the keyframe registry depends on to never grow during capture. B3_NULL_INDEX ends the chain.
// PORT: uint8_t* bytes is a slice; byteCount is kept.
type geometryEntry struct {
	contentHash uint64
	id          uint32
	kind        geometryKind
	byteCount   int32
	bytes       []byte
	hashNext    int32
}

// geometryRegistry is b3GeometryRegistry.
// Growable array of geometry entries. Ids are array indices, so the array is serialized in order.
// dedupMap maps content hash to entry id for O(1) dedup; it is opaque here and owned by recording.c.
// PORT: void* dedupMap is the geometryHashMap of recording.c (a Go map).
type geometryRegistry struct {
	entries  []geometryEntry
	dedupMap geometryHashMap
}

// Limit the maximum query name length to make recording simpler. Query names longer than this
// probably indicate a bug in user code.
const maxQueryNameLength = 64 // B3_MAX_QUERY_NAME_LENGTH

// recTag is b3RecTag.
// Query tag from b3QueryFilter.
// Stored once per key in the trailing block so a tagged query carries only the 8 byte key on the wire.
// Shared by the recorder (accumulate) and the player (load).
type recTag struct {
	// hash of (id, queryName)
	key       uint64
	id        uint64
	queryName [maxQueryNameLength + 1]byte
}

// Recording is b3Recording.
// User-owned recording buffer. The world appends into it while active; the host saves and
// destroys it. Opaque across the public API.
// PORT: exported because the public API (box3d.h) hands it out (b3CreateRecording,
// b3World_StartRecording); all fields are unexported. b3RecTag* tags is a slice with tagCount and
// tagCapacity kept (the slice length is tagCapacity); void* tagMap is the recTagMap of recording.c
// (a Go map).
type Recording struct {
	buffer      recBuffer
	recordStart int32  // offset of the 3-byte size field for u24 backpatch
	lock        *mutex // serializes record writes from concurrent threads
	registry    geometryRegistry

	// Interned query tags accumulated during capture, written to the tail of the registry block at stop.
	// tagMap maps a tag key to its index for O(1) dedup.
	tags        []recTag
	tagCount    int32
	tagCapacity int32
	tagMap      recTagMap

	// Union of world bounds over every recorded step, written at stop.
	accumulatedBounds AABB
	haveBounds        bool
}

// recQueryWriter is b3RecQueryWriter.
// Per-query writer context: holds the user fcn+ctx, the local payload buffer, and the hit counter
// PORT: union userFcn { overlapFcn; castFcn; planeFcn; moverFilterFcn }; all members are present.
// const char* tagName is a string (empty = NULL).
type recQueryWriter struct {
	overlapFcn     OverlapResultFcn
	castFcn        CastResultFcn
	planeFcn       PlaneResultFcn
	moverFilterFcn MoverFilterFcn
	userContext    any
	buf            recBuffer // per-call local payload, heap-backed
	countOffset    int32     // offset of the reserved u32 hit-count slot
	hitCount       uint32
	tagId          uint64 // caller query id, 0 = untagged. Emitted as a QueryTag before the record.
	tagName        string // caller query name, interned by id. NULL = none.
}

// ---- recording.c ----

// PORT: pointer + size parameters are slices; const char* is a string (the empty string is NULL);
// void* context is any. b3Alloc/b3GrowAlloc/b3Free become make/copy and dropped references. The
// two verstable maps of recording.c (b3GeometryHashMap, b3RecTagMap) are Go maps: they are only
// probed by key, never iterated, so their order cannot reach the output.

// Buffer helpers

// recBufAppend is b3RecBufAppend.
func recBufAppend(buf *recBuffer, data []byte) {
	size := int32(len(data))
	if size <= 0 {
		return
	}

	if buf.countOnly {
		buf.size += size
		return
	}

	if buf.size+size > buf.capacity {
		newCap := buf.capacity * 2
		if newCap < buf.size+size+64 {
			newCap = buf.size + size + 64
		}
		// PORT: b3Alloc / b3GrowAlloc: a fresh slice of newCap bytes keeping the written prefix.
		grown := make([]byte, newCap)
		copy(grown, buf.data[:buf.size])
		buf.data = grown
		buf.capacity = newCap
	}

	copy(buf.data[buf.size:], data)
	buf.size += size
}

// recBufFree is b3RecBufFree.
func recBufFree(buf *recBuffer) {
	if buf.data != nil {
		buf.data = nil
		buf.capacity = 0
		buf.size = 0
	}
}

// Write primitives

// recWU8 is b3RecW_U8.
func recWU8(buf *recBuffer, v uint8) {
	b := [1]byte{v}
	recBufAppend(buf, b[:])
}

// recWU16 is b3RecW_U16.
func recWU16(buf *recBuffer, v uint16) {
	b := [2]byte{uint8(v), uint8(v >> 8)}
	recBufAppend(buf, b[:])
}

// recWU32 is b3RecW_U32.
func recWU32(buf *recBuffer, v uint32) {
	b := [4]byte{uint8(v), uint8(v >> 8), uint8(v >> 16), uint8(v >> 24)}
	recBufAppend(buf, b[:])
}

// recWU64 is b3RecW_U64.
func recWU64(buf *recBuffer, v uint64) {
	b := [8]byte{uint8(v), uint8(v >> 8), uint8(v >> 16), uint8(v >> 24),
		uint8(v >> 32), uint8(v >> 40), uint8(v >> 48), uint8(v >> 56)}
	recBufAppend(buf, b[:])
}

// recWI32 is b3RecW_I32.
func recWI32(buf *recBuffer, v int32) {
	recWU32(buf, uint32(v))
}

// recWF32 is b3RecW_F32.
func recWF32(buf *recBuffer, v float32) {
	bits := math.Float32bits(v)
	recWU32(buf, bits)
}

// recWF64 is b3RecW_F64.
func recWF64(buf *recBuffer, v float64) {
	bits := math.Float64bits(v)
	recWU64(buf, bits)
}

// recWBOOL is b3RecW_BOOL.
func recWBOOL(buf *recBuffer, v bool) {
	if v {
		recWU8(buf, 1)
	} else {
		recWU8(buf, 0)
	}
}

// recWVEC3 is b3RecW_VEC3.
func recWVEC3(buf *recBuffer, v Vec3) {
	recWF32(buf, v.X)
	recWF32(buf, v.Y)
	recWF32(buf, v.Z)
}

// recWQUAT is b3RecW_QUAT.
func recWQUAT(buf *recBuffer, v Quat) {
	recWF32(buf, v.V.X)
	recWF32(buf, v.V.Y)
	recWF32(buf, v.V.Z)
	recWF32(buf, v.S)
}

// recWTRANSFORM is b3RecW_TRANSFORM.
func recWTRANSFORM(buf *recBuffer, v Transform) {
	recWVEC3(buf, v.P)
	recWQUAT(buf, v.Q)
}

// recWPOSITION is b3RecW_POSITION.
// World position at full precision so recordings reproduce the simulation far from the origin.
// In the float build this is three floats, wire-identical to VEC3.
// PORT: float build only (BOX3D_DOUBLE_PRECISION is not ported).
func recWPOSITION(buf *recBuffer, v Pos) {
	recWF32(buf, v.X)
	recWF32(buf, v.Y)
	recWF32(buf, v.Z)
}

// recWWORLDXF is b3RecW_WORLDXF.
func recWWORLDXF(buf *recBuffer, v WorldTransform) {
	recWPOSITION(buf, v.P)
	recWQUAT(buf, v.Q)
}

// recWMATRIX3 is b3RecW_MATRIX3.
func recWMATRIX3(buf *recBuffer, v Matrix3) {
	recWVEC3(buf, v.Cx)
	recWVEC3(buf, v.Cy)
	recWVEC3(buf, v.Cz)
}

// recWAABB is b3RecW_AABB.
func recWAABB(buf *recBuffer, v AABB) {
	recWVEC3(buf, v.LowerBound)
	recWVEC3(buf, v.UpperBound)
}

// recWQUERYFILTER is b3RecW_QUERYFILTER.
func recWQUERYFILTER(buf *recBuffer, v QueryFilter) {
	if v.IgnoreShapeMask || v.BackFaces {
		panic("box3d: Golem QueryFilter extensions are not supported by upstream record/replay")
	}
	recWU64(buf, v.CategoryBits)
	recWU64(buf, v.MaskBits)
}

// recWSHAPEPROXY is b3RecW_SHAPEPROXY.
// Variable length: count, then count points, then radius. The point cloud lives behind a pointer so
// it cannot ride along as POD.
func recWSHAPEPROXY(buf *recBuffer, v ShapeProxy) {
	count := v.Count
	if count < 0 {
		count = 0
	}
	if count > MaxShapeCastPoints {
		count = MaxShapeCastPoints
	}
	recWI32(buf, count)
	for i := 0; i < int(count); i++ {
		recWVEC3(buf, v.Points[i])
	}
	recWF32(buf, v.Radius)
}

// recWTREESTATS is b3RecW_TREESTATS.
func recWTREESTATS(buf *recBuffer, v TreeStats) {
	recWI32(buf, v.NodeVisits)
	recWI32(buf, v.LeafVisits)
}

// recWRAYRESULT is b3RecW_RAYRESULT.
func recWRAYRESULT(buf *recBuffer, v RayResult) {
	recWSHAPEID(buf, v.ShapeId)
	recWPOSITION(buf, v.Point)
	recWVEC3(buf, v.Normal)
	recWU64(buf, v.UserMaterialId)
	recWF32(buf, v.Fraction)
	recWI32(buf, v.TriangleIndex)
	recWI32(buf, v.ChildIndex)
	recWBOOL(buf, v.Hit)
}

// recWPLANERESULT is b3RecW_PLANERESULT.
func recWPLANERESULT(buf *recBuffer, v PlaneResult) {
	recWVEC3(buf, v.Plane.Normal)
	recWF32(buf, v.Plane.Offset)
	recWVEC3(buf, v.Point)
	recWI32(buf, v.TriangleIndex)
	recWI32(buf, v.ChildIndex)
	recWI32(buf, v.MaterialIndex)
}

// recWWORLDID is b3RecW_WORLDID.
func recWWORLDID(buf *recBuffer, v WorldID) {
	recWU32(buf, StoreWorldID(v))
}

// recWBODYID is b3RecW_BODYID.
func recWBODYID(buf *recBuffer, v BodyID) {
	recWU64(buf, StoreBodyID(v))
}

// recWSHAPEID is b3RecW_SHAPEID.
func recWSHAPEID(buf *recBuffer, v ShapeID) {
	recWU64(buf, StoreShapeID(v))
}

// recWJOINTID is b3RecW_JOINTID.
func recWJOINTID(buf *recBuffer, v JointID) {
	recWU64(buf, StoreJointID(v))
}

// PORT: b3RecW_SPHERE and b3RecW_CAPSULE append the raw struct. Sphere and Capsule are plain
// float32 structs with C's field order and no padding (16 and 28 bytes, checked here), so their
// native byte image is upstream's.
var (
	_ = [1]struct{}{}[unsafe.Sizeof(Sphere{})-16]
	_ = [1]struct{}{}[unsafe.Sizeof(Capsule{})-28]
)

// recWSPHERE is b3RecW_SPHERE.
// Pointer-free POD; pointerWidth in the header gates the layout on replay
func recWSPHERE(buf *recBuffer, v Sphere) {
	recBufAppend(buf, unsafe.Slice((*byte)(unsafe.Pointer(&v)), unsafe.Sizeof(v)))
}

// recWCAPSULE is b3RecW_CAPSULE.
func recWCAPSULE(buf *recBuffer, v Capsule) {
	recBufAppend(buf, unsafe.Slice((*byte)(unsafe.Pointer(&v)), unsafe.Sizeof(v)))
}

// recWGEOMID is b3RecW_GEOMID.
func recWGEOMID(buf *recBuffer, v uint32) {
	recWU32(buf, v)
}

// recWFILTER is b3RecW_FILTER.
func recWFILTER(buf *recBuffer, v Filter) {
	recWU64(buf, v.CategoryBits)
	recWU64(buf, v.MaskBits)
	recWI32(buf, v.GroupIndex)
}

// recWMATERIAL is b3RecW_MATERIAL.
func recWMATERIAL(buf *recBuffer, v SurfaceMaterial) {
	recWF32(buf, v.Friction)
	recWF32(buf, v.Restitution)
	recWF32(buf, v.RollingResistance)
	recWVEC3(buf, v.TangentVelocity)
	recWU64(buf, v.UserMaterialId)
	recWU32(buf, v.CustomColor)
}

// recWMASSDATA is b3RecW_MASSDATA.
func recWMASSDATA(buf *recBuffer, v MassData) {
	recWF32(buf, v.Mass)
	recWVEC3(buf, v.Center)
	recWMATRIX3(buf, v.Inertia)
}

// recWLOCKS is b3RecW_LOCKS.
func recWLOCKS(buf *recBuffer, v MotionLocks) {
	recWBOOL(buf, v.LinearX)
	recWBOOL(buf, v.LinearY)
	recWBOOL(buf, v.LinearZ)
	recWBOOL(buf, v.AngularX)
	recWBOOL(buf, v.AngularY)
	recWBOOL(buf, v.AngularZ)
}

// recWSTR is b3RecW_STR.
// PORT: const char* is a string and the empty string is NULL (0xFFFF), so a def name of "" records
// as NULL where upstream would record a zero length string. The C string stops at its first NUL;
// recWStrBytes does the length scan on bytes.
func recWSTR(buf *recBuffer, s string) {
	if s == "" {
		recWU16(buf, 0xFFFF)
		return
	}
	recWStrBytes(buf, []byte(s))
}

// recWStrBytes is the non-NULL branch of b3RecW_STR on a byte string (a NUL ends it).
func recWStrBytes(buf *recBuffer, s []byte) {
	len_ := 0
	for len_ < len(s) && s[len_] != 0 && len_ < 65534 {
		len_++
	}
	recWU16(buf, uint16(len_))
	if len_ > 0 {
		recBufAppend(buf, s[:len_])
	}
}

// Hand-written def helpers. Zero pointer and cookie fields before serializing.
// Readers call b3Default*Def() first to get the cookie, then overwrite fields.

// Tripwire: each def serializer below is paired with a reader in recording_replay.c, and the two must
// stay field-for-field in sync. Add a field to a def and the size changes, firing the matching assert
// so the writer and reader both get updated. Only enforced on the 64-bit target; each def lists the
// single-precision and double-precision sizes (equal for most), so either build configuration passes.
// PORT: the _Static_asserts check the C sizeof of the defs; the Go defs hold strings, slices and
// `any` and have other sizes. The tripwire is the DefTripwire spec in recording_golden_test.go,
// which pins the field count of each Go def.

// recWEXPLOSIONDEF is b3RecW_EXPLOSIONDEF.
func recWEXPLOSIONDEF(buf *recBuffer, v ExplosionDef) {
	recWU64(buf, v.MaskBits)
	recWPOSITION(buf, v.Position)
	recWF32(buf, v.Radius)
	recWF32(buf, v.Falloff)
	recWF32(buf, v.ImpulsePerArea)
}

// recWBODYDEF is b3RecW_BODYDEF.
func recWBODYDEF(buf *recBuffer, v BodyDef) {
	recWI32(buf, int32(v.Type))
	recWPOSITION(buf, v.Position)
	recWQUAT(buf, v.Rotation)
	recWVEC3(buf, v.LinearVelocity)
	recWVEC3(buf, v.AngularVelocity)
	recWF32(buf, v.LinearDamping)
	recWF32(buf, v.AngularDamping)
	recWF32(buf, v.GravityScale)
	recWF32(buf, v.SleepThreshold)
	recWF32(buf, v.SafetyFactor)
	recWSTR(buf, v.Name)
	// userData: not preserved
	recWU64(buf, 0)
	recWLOCKS(buf, v.MotionLocks)
	recWBOOL(buf, v.EnableSleep)
	recWBOOL(buf, v.IsAwake)
	recWBOOL(buf, v.IsBullet)
	recWBOOL(buf, v.IsEnabled)
	recWBOOL(buf, v.AllowFastRotation)
	recWBOOL(buf, v.EnableContactRecycling)
	// internalValue omitted
}

// recWSHAPEDEF is b3RecW_SHAPEDEF.
func recWSHAPEDEF(buf *recBuffer, v ShapeDef) {
	recWSTR(buf, v.Name)

	// userData: not preserved
	recWU64(buf, 0)
	// Per-triangle materials: length-prefixed so the reader can rebuild the array.
	// Guard NULL so a default def (materialCount=0, materials=NULL) round-trips cleanly.
	matCount := int32(0)
	if v.Materials != nil {
		matCount = v.MaterialCount
	}
	recWI32(buf, matCount)
	for i := 0; i < int(matCount); i++ {
		recWMATERIAL(buf, v.Materials[i])
	}
	recWMATERIAL(buf, v.BaseMaterial)
	recWF32(buf, v.Density)
	recWF32(buf, v.ExplosionScale)
	recWFILTER(buf, v.Filter)
	recWBOOL(buf, v.EnableCustomFiltering)
	recWBOOL(buf, v.IsSensor)
	recWBOOL(buf, v.EnableSensorEvents)
	recWBOOL(buf, v.EnableContactEvents)
	recWBOOL(buf, v.EnableHitEvents)
	recWBOOL(buf, v.EnablePreSolveEvents)
	recWBOOL(buf, v.InvokeContactCreation)
	recWBOOL(buf, v.UpdateBodyMass)
	recWBOOL(buf, v.EnableSpeculativeContact)
	// internalValue omitted
}

// recWJointBase is b3RecW_JointBase.
// Joint defs share a base. Body ids are written as packed ids for replay remapping.
func recWJointBase(buf *recBuffer, base *JointDef) {
	// userData: not preserved
	recWU64(buf, 0)
	recWBODYID(buf, base.BodyIdA)
	recWBODYID(buf, base.BodyIdB)
	recWTRANSFORM(buf, base.LocalFrameA)
	recWTRANSFORM(buf, base.LocalFrameB)
	recWF32(buf, base.ForceThreshold)
	recWF32(buf, base.TorqueThreshold)
	recWF32(buf, base.ConstraintHertz)
	recWF32(buf, base.ConstraintDampingRatio)
	recWF32(buf, base.DrawScale)
	recWBOOL(buf, base.CollideConnected)
	// internalValue omitted
}

// recWPARALLELJOINTDEF is b3RecW_PARALLELJOINTDEF.
func recWPARALLELJOINTDEF(buf *recBuffer, v ParallelJointDef) {
	recWJointBase(buf, &v.Base)
	recWF32(buf, v.Hertz)
	recWF32(buf, v.DampingRatio)
	recWF32(buf, v.MaxTorque)
}

// recWDISTANCEJOINTDEF is b3RecW_DISTANCEJOINTDEF.
func recWDISTANCEJOINTDEF(buf *recBuffer, v DistanceJointDef) {
	recWJointBase(buf, &v.Base)
	recWF32(buf, v.Length)
	recWBOOL(buf, v.EnableSpring)
	recWF32(buf, v.LowerSpringForce)
	recWF32(buf, v.UpperSpringForce)
	recWF32(buf, v.Hertz)
	recWF32(buf, v.DampingRatio)
	recWBOOL(buf, v.EnableLimit)
	recWF32(buf, v.MinLength)
	recWF32(buf, v.MaxLength)
	recWBOOL(buf, v.EnableMotor)
	recWF32(buf, v.MaxMotorForce)
	recWF32(buf, v.MotorSpeed)
}

// recWFILTERJOINTDEF is b3RecW_FILTERJOINTDEF.
func recWFILTERJOINTDEF(buf *recBuffer, v FilterJointDef) {
	recWJointBase(buf, &v.Base)
}

// recWMOTORJOINTDEF is b3RecW_MOTORJOINTDEF.
func recWMOTORJOINTDEF(buf *recBuffer, v MotorJointDef) {
	recWJointBase(buf, &v.Base)
	recWVEC3(buf, v.LinearVelocity)
	recWF32(buf, v.MaxVelocityForce)
	recWVEC3(buf, v.AngularVelocity)
	recWF32(buf, v.MaxVelocityTorque)
	recWF32(buf, v.LinearHertz)
	recWF32(buf, v.LinearDampingRatio)
	recWF32(buf, v.MaxSpringForce)
	recWF32(buf, v.AngularHertz)
	recWF32(buf, v.AngularDampingRatio)
	recWF32(buf, v.MaxSpringTorque)
}

// recWPRISMATICJOINTDEF is b3RecW_PRISMATICJOINTDEF.
func recWPRISMATICJOINTDEF(buf *recBuffer, v PrismaticJointDef) {
	recWJointBase(buf, &v.Base)
	recWBOOL(buf, v.EnableSpring)
	recWF32(buf, v.Hertz)
	recWF32(buf, v.DampingRatio)
	recWF32(buf, v.TargetTranslation)
	recWBOOL(buf, v.EnableLimit)
	recWF32(buf, v.LowerTranslation)
	recWF32(buf, v.UpperTranslation)
	recWBOOL(buf, v.EnableMotor)
	recWF32(buf, v.MaxMotorForce)
	recWF32(buf, v.MotorSpeed)
}

// recWREVOLUTEJOINTDEF is b3RecW_REVOLUTEJOINTDEF.
func recWREVOLUTEJOINTDEF(buf *recBuffer, v RevoluteJointDef) {
	recWJointBase(buf, &v.Base)
	recWF32(buf, v.TargetAngle)
	recWBOOL(buf, v.EnableSpring)
	recWF32(buf, v.Hertz)
	recWF32(buf, v.DampingRatio)
	recWBOOL(buf, v.EnableLimit)
	recWF32(buf, v.LowerAngle)
	recWF32(buf, v.UpperAngle)
	recWBOOL(buf, v.EnableMotor)
	recWF32(buf, v.MaxMotorTorque)
	recWF32(buf, v.MotorSpeed)
}

// recWSPHERICALJOINTDEF is b3RecW_SPHERICALJOINTDEF.
func recWSPHERICALJOINTDEF(buf *recBuffer, v SphericalJointDef) {
	recWJointBase(buf, &v.Base)
	recWBOOL(buf, v.EnableSpring)
	recWF32(buf, v.Hertz)
	recWF32(buf, v.DampingRatio)
	recWQUAT(buf, v.TargetRotation)
	recWBOOL(buf, v.EnableConeLimit)
	recWF32(buf, v.ConeAngle)
	recWBOOL(buf, v.EnableTwistLimit)
	recWF32(buf, v.LowerTwistAngle)
	recWF32(buf, v.UpperTwistAngle)
	recWBOOL(buf, v.EnableMotor)
	recWF32(buf, v.MaxMotorTorque)
	recWVEC3(buf, v.MotorVelocity)
}

// recWWELDJOINTDEF is b3RecW_WELDJOINTDEF.
func recWWELDJOINTDEF(buf *recBuffer, v WeldJointDef) {
	recWJointBase(buf, &v.Base)
	recWF32(buf, v.LinearHertz)
	recWF32(buf, v.AngularHertz)
	recWF32(buf, v.LinearDampingRatio)
	recWF32(buf, v.AngularDampingRatio)
}

// recWWHEELJOINTDEF is b3RecW_WHEELJOINTDEF.
func recWWHEELJOINTDEF(buf *recBuffer, v WheelJointDef) {
	recWJointBase(buf, &v.Base)
	recWBOOL(buf, v.EnableSuspensionSpring)
	recWF32(buf, v.SuspensionHertz)
	recWF32(buf, v.SuspensionDampingRatio)
	recWBOOL(buf, v.EnableSuspensionLimit)
	recWF32(buf, v.LowerSuspensionLimit)
	recWF32(buf, v.UpperSuspensionLimit)
	recWBOOL(buf, v.EnableSpinMotor)
	recWF32(buf, v.MaxSpinTorque)
	recWF32(buf, v.SpinSpeed)
	recWBOOL(buf, v.EnableSteering)
	recWF32(buf, v.SteeringHertz)
	recWF32(buf, v.SteeringDampingRatio)
	recWF32(buf, v.TargetSteeringAngle)
	recWF32(buf, v.MaxSteeringTorque)
	recWBOOL(buf, v.EnableSteeringLimit)
	recWF32(buf, v.LowerSteeringLimit)
	recWF32(buf, v.UpperSteeringLimit)
}

// Query recording. A query collects a variable number of hits through a user callback, so the count
// is not known until the callback stops firing. The record is built in a local buffer with a
// reserved hit-count slot, then committed whole under the lock so concurrent query threads never
// interleave records in the shared buffer.

// recReserveU32 is b3RecReserveU32.
func recReserveU32(buf *recBuffer) int {
	offset := int(buf.size)
	zero := [4]byte{0, 0, 0, 0}
	recBufAppend(buf, zero[:])
	return offset
}

// recPatchU32 is b3RecPatchU32.
func recPatchU32(buf *recBuffer, offset int, v uint32) {
	assert(offset >= 0 && offset+4 <= int(buf.size), "offset >= 0 && offset + 4 <= buf->size")
	p := buf.data[offset:]
	p[0] = uint8(v)
	p[1] = uint8(v >> 8)
	p[2] = uint8(v >> 16)
	p[3] = uint8(v >> 24)
}

// recCommitRecordLocked is b3RecCommitRecordLocked.
// Frame and append one record into the buffer. Caller holds rec->lock.
func recCommitRecordLocked(rec *Recording, opcode uint8, payload []byte) {
	payloadSize := len(payload)
	assert(payloadSize >= 0 && payloadSize < (1<<24), "payloadSize >= 0 && payloadSize < ( 1 << 24 )")
	recWU8(&rec.buffer, opcode)
	sz := [3]byte{uint8(payloadSize), uint8(payloadSize >> 8), uint8(payloadSize >> 16)}
	recBufAppend(&rec.buffer, sz[:])
	recBufAppend(&rec.buffer, payload)
}

// recCommitRecord is b3RecCommitRecord.
// Commit a finished query record under the lock. The payload buffer stays owned by the caller.
func recCommitRecord(rec *Recording, opcode uint8, payload []byte) {
	lockMutex(rec.lock)
	recCommitRecordLocked(rec, opcode, payload)
	unlockMutex(rec.lock)
}

// recQueryBegin is b3RecQueryBegin.
// PORT: setting the union member overlapFcn to NULL clears every member.
func recQueryBegin(w *recQueryWriter, context any, tagId uint64, tagName string) {
	w.buf = recBuffer{}
	w.overlapFcn = nil
	w.castFcn = nil
	w.planeFcn = nil
	w.moverFilterFcn = nil
	w.userContext = context
	w.hitCount = 0
	w.countOffset = 0
	w.tagId = tagId
	w.tagName = tagName
}

// recBufBytes is the written prefix of a buffer: ( buf.data, buf.size ).
func recBufBytes(buf *recBuffer) []byte {
	return buf.data[:buf.size]
}

// recQueryCommit is b3RecQueryCommit.
func recQueryCommit(rec *Recording, opcode uint8, w *recQueryWriter) {
	lockMutex(rec.lock)
	// A tagged query writes its identity key right before the query record, under one lock so the pair
	// stays adjacent even with concurrent queries. The key is the hash of the caller (id, name), which
	// are interned once into the trailing tag table so the viewer can show them.
	tagged := w.tagId != 0 || (w.tagName != "" && w.tagName[0] != 0)
	if tagged {
		key := hashQueryTag(w.tagId, w.tagName)
		recInternTag(rec, key, w.tagId, w.tagName)
		tagBuf := recBuffer{}
		recWU64(&tagBuf, key)
		recCommitRecordLocked(rec, recOpQueryTag, recBufBytes(&tagBuf))
		recBufFree(&tagBuf)
	}
	recCommitRecordLocked(rec, opcode, recBufBytes(&w.buf))
	unlockMutex(rec.lock)
	recBufFree(&w.buf)
}

// recOverlapTrampoline is b3RecOverlapTrampoline.
// PORT: the C union aliases overlapFcn and moverFilterFcn; Go keeps both members, so the
// trampoline calls whichever one the query installed.
func recOverlapTrampoline(id ShapeID, ctx any) bool {
	w := ctx.(*recQueryWriter)
	// The user fcn is NULL for an unfiltered mover cast: accept all, still record the decision so
	// replay reproduces the same per-shape accept stream.
	ret := true
	if w.overlapFcn != nil {
		ret = w.overlapFcn(id, w.userContext)
	} else if w.moverFilterFcn != nil {
		ret = w.moverFilterFcn(id, w.userContext)
	}
	recWSHAPEID(&w.buf, id)
	recWBOOL(&w.buf, ret)
	w.hitCount++
	return ret
}

// recCastTrampoline is b3RecCastTrampoline.
func recCastTrampoline(id ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64, triangleIndex int,
	childIndex int, ctx any) float32 {
	w := ctx.(*recQueryWriter)
	ret := w.castFcn(id, point, normal, fraction, userMaterialId, triangleIndex, childIndex, w.userContext)
	recWSHAPEID(&w.buf, id)
	recWPOSITION(&w.buf, point)
	recWVEC3(&w.buf, normal)
	recWF32(&w.buf, fraction)
	recWU64(&w.buf, userMaterialId)
	recWI32(&w.buf, int32(triangleIndex))
	recWI32(&w.buf, int32(childIndex))
	recWF32(&w.buf, ret)
	w.hitCount++
	return ret
}

// recPlaneTrampoline is b3RecPlaneTrampoline.
// 3D delivers every plane for one shape in a single call. Record the shape, its plane count, each
// plane, then the user return so replay reproduces the same per-shape batch. One hit per shape.
// PORT: const b3PlaneResult* planes + planeCount is a slice (as PlaneResultFcn).
func recPlaneTrampoline(id ShapeID, planes []PlaneResult, ctx any) bool {
	w := ctx.(*recQueryWriter)
	ret := w.planeFcn(id, planes, w.userContext)
	planeCount := len(planes)
	recWSHAPEID(&w.buf, id)
	recWI32(&w.buf, int32(planeCount))
	for i := 0; i < planeCount; i++ {
		recWPLANERESULT(&w.buf, planes[i])
	}
	recWBOOL(&w.buf, ret)
	w.hitCount++
	return ret
}

// Record framing

// recBeginRecord is b3RecBeginRecord.
func recBeginRecord(rec *Recording, opcode uint8) {
	recWU8(&rec.buffer, opcode)
	rec.recordStart = rec.buffer.size
	// Reserve 3 bytes for the u24 payload size, backpatched in b3RecEndRecord.
	zero := [3]byte{0, 0, 0}
	recBufAppend(&rec.buffer, zero[:])
}

// recEndRecord is b3RecEndRecord.
func recEndRecord(rec *Recording) {
	payloadSize := rec.buffer.size - rec.recordStart - 3
	assert(payloadSize >= 0 && payloadSize < (1<<24), "payloadSize >= 0 && payloadSize < ( 1 << 24 )")
	p := rec.buffer.data[rec.recordStart:]
	p[0] = uint8(payloadSize)
	p[1] = uint8(payloadSize >> 8)
	p[2] = uint8(payloadSize >> 16)
}

// PORT: the codegen passes of recording.c (b3RecWriteArgs_Name, b3RecWrite_Name and
// b3RecWriteRet_Name from the recording_ops.inl manifest) are written out in recording_ops.go.

// Geometry registry

// Content hash to chain head, so dedup is near O(1). Colliding hashes share a head and are walked
// through b3GeometryEntry::hashNext, so the byteCount + memcmp check always finds an existing blob.
// PORT: b3GeometryHashMap (verstable, uint64_t -> uint32_t) is geometryHashMap, a Go map.
type geometryHashMap = map[uint64]uint32

// Tag key to tag index, so interning a query tag is O(1) rather than a linear scan over the tag table.
// PORT: b3RecTagMap (verstable, uint64_t -> uint32_t) is recTagMap, a Go map.
type recTagMap = map[uint64]uint32

// registryPush is b3RegistryPush.
// Append a fresh entry and splice it onto the front of its hash chain. The map value is the chain head.
// PORT: the map iterator is replaced by the key: hashPresent says whether m[contentHash] exists.
func registryPush(reg *geometryRegistry, m geometryHashMap, hashPresent bool, kind geometryKind, contentHash uint64,
	bytes []byte) uint32 {
	byteCount := len(bytes)
	assert(byteCount > 0, "byteCount > 0")
	copy_ := make([]byte, byteCount)
	copy(copy_, bytes)

	id := uint32(len(reg.entries))
	entry := arrayEmplace(&reg.entries)
	entry.contentHash = contentHash
	entry.id = id
	entry.kind = kind
	entry.byteCount = int32(byteCount)
	entry.bytes = copy_
	if hashPresent {
		entry.hashNext = int32(m[contentHash])
	} else {
		entry.hashNext = NullIndex
	}

	m[contentHash] = id
	return id
}

// registryMap is b3RegistryMap.
func registryMap(reg *geometryRegistry) geometryHashMap {
	if reg.dedupMap == nil {
		fresh := make(geometryHashMap)
		reg.dedupMap = fresh
	}
	return reg.dedupMap
}

// internGeometry is b3InternGeometry.
func internGeometry(reg *geometryRegistry, kind geometryKind, contentHash uint64, bytes []byte) uint32 {
	m := registryMap(reg)

	head, hashPresent := m[contentHash]
	if hashPresent {
		// Walk every entry sharing this hash so a collision still finds the identical blob.
		for index := int32(head); index != NullIndex; index = reg.entries[index].hashNext {
			e := &reg.entries[index]
			if int(e.byteCount) == len(bytes) && string(e.bytes) == string(bytes) {
				return e.id
			}
		}
	}

	return registryPush(reg, m, hashPresent, kind, contentHash, bytes)
}

// appendGeometry is b3AppendGeometry.
// Append an entry unconditionally and return its id, which equals its array index. Unlike
// b3InternGeometry it never deduplicates, so the keyframe seed can mirror slots 1:1 even when an
// already-recorded file carries byte-identical duplicate slots (a hash collision wrote them apart).
func appendGeometry(reg *geometryRegistry, kind geometryKind, contentHash uint64, bytes []byte) uint32 {
	m := registryMap(reg)
	_, hashPresent := m[contentHash]
	return registryPush(reg, m, hashPresent, kind, contentHash, bytes)
}

// freeRegistry is b3FreeRegistry.
func freeRegistry(reg *geometryRegistry) {
	arrayDestroy(&reg.entries)
	reg.dedupMap = nil
}

// hashQueryTag is b3HashQueryTag.
// Hash a query (id, name) pair into the stable key the viewer tracks the query by. Never returns 0,
// so the key doubles as a tagged/untagged flag.
func hashQueryTag(id uint64, name string) uint64 {
	h := snapFnvInit
	for i := 0; i < 8; i++ {
		h = (h ^ ((id >> (8 * i)) & 0xFF)) * snapFnvPrime
	}
	for i := 0; i < len(name) && name[i] != 0; i++ {
		h = (h ^ uint64(name[i])) * snapFnvPrime
	}
	// Never 0 so the key doubles as the tagged flag.
	if h != 0 {
		return h
	}
	return 1
}

// recTags is b3RecTags.
func recTags(rec *Recording) recTagMap {
	if rec.tagMap == nil {
		fresh := make(recTagMap)
		rec.tagMap = fresh
	}
	return rec.tagMap
}

// recInternTag is b3RecInternTag.
// Record a key->(id, name) mapping once, deduped by key (a repeated key keeps its first id/name).
func recInternTag(rec *Recording, key uint64, id uint64, name string) {
	m := recTags(rec)
	if _, found := m[key]; found {
		return // first id/name for a key wins
	}

	if rec.tagCount == rec.tagCapacity {
		newCap := int32(8)
		if rec.tagCapacity != 0 {
			newCap = 2 * rec.tagCapacity
		}
		// PORT: b3GrowAlloc; the slice length is the capacity.
		grown := make([]recTag, newCap)
		copy(grown, rec.tags[:rec.tagCount])
		rec.tags = grown
		rec.tagCapacity = newCap
	}

	index := uint32(rec.tagCount)
	tag := &rec.tags[rec.tagCount]
	rec.tagCount++
	tag.key = key
	tag.id = id
	n := 0
	for n < len(name) && name[n] != 0 && n < maxQueryNameLength {
		tag.queryName[n] = name[n]
		n++
	}
	tag.queryName[n] = 0
	m[key] = index
}

// recWriteRegistry is b3RecWriteRegistry.
// Write the trailing registry block: u32 entryCount then per-entry { u8 kind, u32 byteCount, bytes },
// followed by the query-tag table { u32 tagCount, per-tag uu64 id, STR name }. A reader built before
// the tag table stops after the geometry entries and ignores the trailing tag bytes.
func recWriteRegistry(rec *Recording) {
	recWU32(&rec.buffer, uint32(len(rec.registry.entries)))
	for i := 0; i < len(rec.registry.entries); i++ {
		e := &rec.registry.entries[i]
		recWU8(&rec.buffer, uint8(e.kind))
		recWU32(&rec.buffer, uint32(e.byteCount))
		recBufAppend(&rec.buffer, e.bytes[:e.byteCount])
	}

	recWU32(&rec.buffer, uint32(rec.tagCount))
	for i := 0; i < int(rec.tagCount); i++ {
		recWU64(&rec.buffer, rec.tags[i].key)
		recWU64(&rec.buffer, rec.tags[i].id)
		// PORT: queryName is a char array, never NULL.
		recWStrBytes(&rec.buffer, rec.tags[i].queryName[:])
	}
}

// Lifecycle

// CreateRecording is b3CreateRecording: create an empty recording buffer. byteCapacity is the
// initial capacity (0 picks a default).
func CreateRecording(byteCapacity int) *Recording {
	rec := &Recording{}

	initCap := 65536
	if byteCapacity > 0 {
		initCap = byteCapacity
	}
	rec.buffer.data = make([]byte, initCap)
	rec.buffer.capacity = int32(initCap)
	rec.buffer.size = 0
	rec.lock = createMutex()
	return rec
}

// Destroy is b3DestroyRecording: destroy a recording created by CreateRecording or
// LoadRecordingFromFile.
func (recording *Recording) Destroy() {
	if recording == nil {
		return
	}

	recBufFree(&recording.buffer)
	freeRegistry(&recording.registry)
	recording.tags = nil
	recording.tagMap = nil
	destroyMutex(recording.lock)
	// PORT: b3Free( recording ) is dropped; the fields are cleared so a stale handle reads empty.
	*recording = Recording{}
}

// GetData is b3Recording_GetData: the recorded bytes.
// PORT: returns the recorded prefix of the buffer (GetSize bytes) instead of the raw pointer.
func (recording *Recording) GetData() []byte {
	return recording.buffer.data[:recording.buffer.size]
}

// GetSize is b3Recording_GetSize: the recorded byte count.
func (recording *Recording) GetSize() int {
	return int(recording.buffer.size)
}

// recAccumulateBounds is b3RecAccumulateBounds.
// Fold one step's world bounds into the running union.
func recAccumulateBounds(rec *Recording, bounds AABB) {
	if rec.haveBounds {
		rec.accumulatedBounds = rec.accumulatedBounds.Union(bounds)
	} else {
		rec.accumulatedBounds = bounds
	}
	rec.haveBounds = true
}

// recHeaderBytes is the native (little-endian) byte image of a b3RecHeader, as the memcpy of the
// struct in b3StartRecordingIntoBuffer writes it.
func recHeaderBytes(hdr *recHeader) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(hdr)), unsafe.Sizeof(*hdr))
}

// startRecordingIntoBuffer is b3StartRecordingIntoBuffer.
func startRecordingIntoBuffer(world *world, recording *Recording) {
	// Reset so a recording handle can be reused for a fresh session
	recording.buffer.size = 0
	recording.recordStart = 0
	recording.haveBounds = false
	freeRegistry(&recording.registry)
	if recording.tags != nil {
		recording.tags = nil
	}
	if recording.tagMap != nil {
		recording.tagMap = nil
	}
	recording.tagCount = 0
	recording.tagCapacity = 0

	hdr := recHeader{}
	hdr.magic = recMagic
	hdr.versionMajor = recVersionMajor
	hdr.versionMinor = recVersionMinor
	// PORT: sizeof( void* ) is written as 8. The Go def writers are field by field and do not depend
	// on the pointer size, so a Go recording is always the 64-bit C layout.
	hdr.pointerWidth = 8
	hdr.bigEndian = 0
	if validateEnabled {
		hdr.validationEnabled = 1
	}
	hdr.lengthScale = GetLengthUnitsPerMeter()
	hdr.registryOffset = 0 // backpatched in b3StopRecordingInternal
	hdr.registryByteCount = 0

	world.recording = recording

	// Every recording is snapshot-seeded. The seed blob follows the header so replay restores in
	// place and the world id stays stable across a restart or backward scrub. An empty world still
	// serializes a valid blob, so there is no from-creation special case.
	snapBuf := recBuffer{}
	serializeWorld(world, &snapBuf, recording)
	hdr.snapshotSize = uint64(snapBuf.size)

	recBufAppend(&recording.buffer, recHeaderBytes(&hdr))
	recBufAppend(&recording.buffer, recBufBytes(&snapBuf))
	recBufFree(&snapBuf)

	// Anchor the recording with the current world state hash so replay can assert
	// determinism from the very first step.
	worldId := WorldID{world.worldId + 1, world.generation}
	stateHash := recArgsStateHash{worldId, hashWorldState(world)}
	recWriteStateHash(recording, &stateHash)
}

// stopRecordingInternal is b3StopRecordingInternal.
func stopRecordingInternal(world *world) {
	if world.recording == nil {
		return
	}

	rec := world.recording
	world.recording = nil

	// Write accumulated bounds so a viewer can frame the whole recorded motion
	rb := recArgsRecordingBounds{}
	if rec.haveBounds {
		rb.bounds = rec.accumulatedBounds
	}
	recWriteRecordingBounds(rec, &rb)

	// End-of-stream marker; the buffer is now self-contained
	wid := WorldID{world.worldId + 1, world.generation}
	a := recArgsDestroyWorld{wid}
	recWriteDestroyWorld(rec, &a)

	// Write the trailing registry block
	registryOffset := rec.buffer.size
	recWriteRegistry(rec)
	registryByteCount := rec.buffer.size - registryOffset

	// Backpatch registryOffset and registryByteCount into the header
	hdrBytes := rec.buffer.data
	regOff := uint64(registryOffset)
	regSz := uint64(registryByteCount)
	// Little-endian backpatch in place; offsetof keeps this correct if the header layout shifts
	pOff := hdrBytes[unsafe.Offsetof(recHeader{}.registryOffset):]
	pSz := hdrBytes[unsafe.Offsetof(recHeader{}.registryByteCount):]
	for i := 0; i < 8; i++ {
		pOff[i] = uint8(regOff >> (8 * i))
		pSz[i] = uint8(regSz >> (8 * i))
	}
}

// Convenience file I/O

// SaveRecordingToFile is b3SaveRecordingToFile: write the recorded bytes to a file.
func SaveRecordingToFile(recording *Recording, path string) bool {
	if recording == nil || path == "" {
		return false
	}

	// PORT: fopen/fwrite/fclose become os.WriteFile.
	err := os.WriteFile(path, recording.buffer.data[:recording.buffer.size], 0o644)
	return err == nil
}

// LoadRecordingFromFile is b3LoadRecordingFromFile: load a recording file. The caller owns the
// returned recording and must destroy it with Recording.Destroy.
func LoadRecordingFromFile(path string) *Recording {
	if path == "" {
		return nil
	}

	// PORT: fopen/fseek/ftell/fread become os.ReadFile.
	data, err := os.ReadFile(path)
	if err != nil {
		return nil
	}

	fileSize := len(data)
	// Anything smaller than the fixed header can't be a recording
	if fileSize < int(unsafe.Sizeof(recHeader{})) || fileSize > math.MaxInt32 {
		return nil
	}

	rec := CreateRecording(fileSize)
	copy(rec.buffer.data, data)

	// Validate magic so a wrong file fails at load rather than deep in the player
	magic := binary.LittleEndian.Uint32(rec.buffer.data[0:4])
	if magic != recMagic {
		rec.Destroy()
		return nil
	}

	rec.buffer.size = int32(fileSize)
	return rec
}

// Geometry interning helpers

// recInternBlob is b3RecInternBlob.
func recInternBlob(rec *Recording, kind geometryKind, bytes []byte) uint32 {
	h := hash64NonZero(bytes)
	return internGeometry(&rec.registry, kind, h, bytes)
}

// recInternHull is b3RecInternHull.
func recInternHull(rec *Recording, hull *HullData) uint32 {
	return recInternBlob(rec, geometryHull, hullBytes(hull))
}

// recInternMesh is b3RecInternMesh.
func recInternMesh(rec *Recording, mesh *MeshData) uint32 {
	return recInternBlob(rec, geometryMesh, meshBytes(mesh))
}

// recInternHeightField is b3RecInternHeightField.
func recInternHeightField(rec *Recording, hf *HeightFieldData) uint32 {
	return recInternBlob(rec, geometryHeightField, heightFieldBytes(hf))
}

// recInternCompound is b3RecInternCompound.
func recInternCompound(rec *Recording, compound *CompoundData) uint32 {
	return recInternBlob(rec, geometryCompound, compoundBytes(compound))
}

// hashWorldState is b3HashWorldState.
// Deterministic hash over all body transforms and velocities.
// Called by both recorder and replayer to verify simulation reproduces exactly.
// PORT: pulled forward from recording.c (milestone 5) at the milestone 3 integration; the world
// tests compare it across worker counts. B3_HASH_FLOAT is written out with math.Float32bits.
func hashWorldState(world *world) uint64 {
	hash := snapFnvInit
	prime := snapFnvPrime

	bodyCount := len(world.bodies)
	for i := 0; i < bodyCount; i++ {
		body := &world.bodies[i]
		if int(body.id) != i {
			// Free or never-used slot
			continue
		}

		sim := getBodySim(world, body)

		hash = fnvMixPosition(hash, sim.transform.P)
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.X))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.Y))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.V.Z))) * prime
		hash = (hash ^ uint64(math.Float32bits(sim.transform.Q.S))) * prime

		state := getBodyState(world, body)
		if state != nil {
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.X))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.Y))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.linearVelocity.Z))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.X))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.Y))) * prime
			hash = (hash ^ uint64(math.Float32bits(state.angularVelocity.Z))) * prime
		}
	}

	return hash
}
