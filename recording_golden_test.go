// Fork reference: testdata/golem_post_toi applies only the documented native
// post-TOI contact correction to pinned C. Original upstream fixtures remain intact.
// Go-only test: no upstream counterpart. Records a joint-free scene and compares the recording
// byte for byte against the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867)
// compiled with -ffp-contract=off. See testdata/recording_golden_gen.c; this file mirrors it line
// by line. The seed snapshot after the header is Go-native (design: "Recording and replay"), so it
// is skipped: the header (except snapshotSize and registryOffset, which follow from the snapshot
// size), the op stream and the trailing registry block must match. The C output is identical at 1
// and 4 workers and for the BOX3D_DISABLE_SIMD build; BOX3D_VALIDATE only sets the header's
// validationEnabled byte. The Go run is checked at 1, 2 and 4 workers.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"fmt"
	"os"
	"reflect"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const recGoldenSteps = 90

func recGoldenOverlapFcn(shapeId ShapeID, context any) bool {
	count := context.(*int)
	*count += 1
	// Reject every third shape so the recorded accept stream has both values
	return (StoreShapeID(shapeId) % 3) != 0
}

func recGoldenCastFcn(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64, triangleIndex int,
	childIndex int, context any) float32 {
	count := context.(*int)
	*count += 1
	return fraction
}

func recGoldenPlaneFcn(shapeId ShapeID, planes []PlaneResult, context any) bool {
	count := context.(*int)
	*count += len(planes)
	return true
}

func recGoldenMoverFilterFcn(shapeId ShapeID, context any) bool {
	return true
}

func recGoldenCreateDynamic(worldId WorldID, p Pos, name string) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = p
	bodyDef.Name = name
	return worldId.CreateBody(&bodyDef)
}

// recGoldenRun runs the scene and returns the recording. The world stays alive for the caller,
// which destroys it.
func recGoldenRun(workerCount int) ([]byte, WorldID) {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	// Before recording: ground mesh, static box, a resting sphere
	mesh := CreateGridMesh(16, 16, 1.0, 1, true)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Name = "ground"
		groundId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		groundId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})
	}

	box := MakeBoxHull(2.0, 0.25, 1.5)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{5.0, 0.5, 0.0}
		bodyDef.Rotation = Quat{Vec3{0.0, 0.0, 0.6}, 0.8}
		staticId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		staticId.CreateHullShape(&shapeDef, &box.Base)
	}

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0

	{
		bodyId := recGoldenCreateDynamic(worldId, Pos{-1.0, 0.5, -1.0}, "")
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		bodyId.CreateSphereShape(&shapeDef, &sphere)
	}

	for range 10 {
		worldId.Step(1.0/60.0, 4)
	}

	recording := CreateRecording(0)
	worldId.StartRecording(recording)

	// World config
	worldId.SetGravity(Vec3{0.0, -9.75, 0.0})
	worldId.EnableSleeping(true)
	worldId.SetContactTuning(30.0, 10.0, 3.0)

	// Box stack with a duplicate cube hull
	cube := MakeCubeHull(0.5)
	var stack [3]BodyID
	for i := range 3 {
		stack[i] = recGoldenCreateDynamic(worldId, Pos{0.0, float32(0.5) + float32(1.0*float32(i)), 0.0}, "stack")
		stack[i].CreateHullShape(&shapeDef, &cube.Base)
	}

	// Capsule with a name on the shape
	capsuleBody := recGoldenCreateDynamic(worldId, Pos{-2.0, 3.0, 2.0}, "capsule")
	var capsuleShape ShapeID
	{
		capsule := Capsule{Vec3{-0.5, 0.0, 0.0}, Vec3{0.5, 0.0, 0.0}, 0.25}
		def := shapeDef
		def.Name = "capsule shape"
		capsuleShape = capsuleBody.CreateCapsuleShape(&def, &capsule)
	}

	// Rock hull
	rock := CreateRock(0.5)
	rockBody := recGoldenCreateDynamic(worldId, Pos{5.0, 4.0, 0.25}, "rock")
	rockBody.CreateHullShape(&shapeDef, rock)

	// Multi material mesh on a static body
	bumps := CreateGridMesh(4, 4, 0.5, 2, true)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{-6.0, 0.25, -6.0}
		bodyId := worldId.CreateBody(&bodyDef)
		materials := []SurfaceMaterial{DefaultSurfaceMaterial(), DefaultSurfaceMaterial()}
		materials[1].Friction = 0.25
		materials[1].UserMaterialId = 42
		def := DefaultShapeDef()
		def.Materials = materials
		def.MaterialCount = 2
		bodyId.CreateMeshShape(&def, bumps, Vec3{1.0, 1.0, -1.0})
	}

	// Height field on a static body
	heightField := CreateGrid(5, 5, Vec3{1.0, 0.5, 1.0}, false)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{8.0, 0.0, 8.0}
		bodyId := worldId.CreateBody(&bodyDef)
		def := DefaultShapeDef()
		bodyId.CreateHeightFieldShape(&def, heightField)
	}

	// Compound with a sphere, a capsule and a hull
	var compound *CompoundData
	{
		mat := DefaultSurfaceMaterial()
		sphere := CompoundSphereDef{Sphere{Vec3{-1.0, 0.0, 0.0}, 0.5}, mat}
		capsule := CompoundCapsuleDef{Capsule{Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, 0.25}, mat}
		small := MakeBoxHull(0.25, 0.25, 0.25)
		hull := CompoundHullDef{&small.Base, Transform{Vec3{1.0, 0.0, 0.0}, Quat{Vec3{0.0, 0.0, 0.0}, 1.0}}, mat}
		def := CompoundDef{}
		def.Spheres = []CompoundSphereDef{sphere}
		def.SphereCount = 1
		def.Capsules = []CompoundCapsuleDef{capsule}
		def.CapsuleCount = 1
		def.Hulls = []CompoundHullDef{hull}
		def.HullCount = 1
		compound = CreateCompound(&def)
	}
	compoundBody := recGoldenCreateDynamic(worldId, Pos{2.0, 3.0, -2.0}, "compound")
	compoundBody.CreateBakedCompoundShape(&shapeDef, compound)

	// A sphere to destroy later
	doomed := recGoldenCreateDynamic(worldId, Pos{3.0, 1.0, 3.0}, "")
	var doomedShape ShapeID
	{
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.25}
		doomedShape = doomed.CreateSphereShape(&shapeDef, &sphere)
		second := Sphere{Vec3{0.25, 0.0, 0.0}, 0.25}
		doomed.CreateSphereShape(&shapeDef, &second)
	}

	// Body and shape mutators
	capsuleBody.SetLinearVelocity(Vec3{1.0, 0.0, -0.5})
	rockBody.SetAngularVelocity(Vec3{0.0, 2.0, 0.0})
	stack[2].ApplyForceToCenter(Vec3{5.0, 0.0, 0.0}, true)
	compoundBody.ApplyLinearImpulse(Vec3{0.0, 1.0, 0.0}, Pos{2.0, 3.0, -1.75}, true)
	rockBody.SetAngularDamping(0.5)
	stack[0].SetName("bottom")
	capsuleShape.SetFriction(0.75)
	capsuleShape.SetName("")
	{
		filter := DefaultFilter()
		filter.GroupIndex = -2
		doomedShape.SetFilter(filter, true)
	}

	tagged := DefaultQueryFilter()
	tagged.Id = 7
	tagged.Name = "probe"
	nameOnly := DefaultQueryFilter()
	nameOnly.Name = "closest"
	untagged := DefaultQueryFilter()

	proxyPoints := []Vec3{{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.5, 0.0}}
	proxy := ShapeProxy{proxyPoints, 3, 0.125}
	mover := Capsule{Vec3{0.0, 0.25, 0.0}, Vec3{0.0, 1.25, 0.0}, 0.25}

	count := 0
	for step := 1; step <= recGoldenSteps; step++ {
		worldId.Step(1.0/60.0, 4)

		if step%15 == 0 {
			aabb := AABB{Vec3{-3.0, -1.0, -3.0}, Vec3{3.0, 4.0, 3.0}}
			worldId.OverlapAABB(aabb, tagged, recGoldenOverlapFcn, &count)
			worldId.OverlapShape(Pos{0.0, 0.5, 0.0}, &proxy, untagged, recGoldenOverlapFcn, &count)
			worldId.CastRay(Pos{-8.0, 1.0, 0.0}, Vec3{16.0, 0.0, 0.0}, untagged, recGoldenCastFcn, &count)
			worldId.CastRayClosest(Pos{0.0, 10.0, 0.0}, Vec3{0.0, -20.0, 0.0}, nameOnly)
			worldId.CastShape(Pos{0.0, 8.0, 0.5}, &proxy, Vec3{0.0, -10.0, 0.0}, tagged, recGoldenCastFcn, &count)
			worldId.CastMover(Pos{-4.0, 0.0, 0.0}, &mover, Vec3{8.0, 0.0, 0.0}, untagged, recGoldenMoverFilterFcn, nil)
			worldId.CastMover(Pos{-4.0, 0.0, 1.0}, &mover, Vec3{8.0, 0.0, 0.0}, tagged, nil, nil)
			worldId.CollideMover(Pos{0.0, 0.0, 0.0}, &mover, untagged, recGoldenPlaneFcn, &count)
		}

		if step == 30 {
			doomedShape.Destroy(true)
			stack[1].SetTransform(Pos{0.25, 2.0, 0.0}, Quat{Vec3{0.0, 0.6, 0.0}, 0.8})
		}

		if step == 45 {
			doomed.Destroy()
			explosion := DefaultExplosionDef()
			explosion.Position = Pos{0.0, 0.0, 0.0}
			explosion.Radius = 4.0
			explosion.Falloff = 1.0
			explosion.ImpulsePerArea = 2.0
			worldId.Explode(&explosion)
		}
	}

	worldId.StopRecording()

	Expect(count).To(Equal(96))
	Expect(worldId.GetAwakeBodyCount()).To(Equal(6))

	data := append([]byte(nil), recording.GetData()...)
	recording.Destroy()
	_ = mesh
	_ = bumps
	_ = heightField
	_ = rock
	return data, worldId
}

// recGoldenParts splits a recording into header, snapshot, op stream and registry block.
func recGoldenParts(data []byte) (header, snapshot, ops, registry []byte) {
	Expect(len(data)).To(BeNumerically(">=", 48))
	snapshotSize := binary.LittleEndian.Uint64(data[24:32])
	registryOffset := binary.LittleEndian.Uint64(data[32:40])
	registryByteCount := binary.LittleEndian.Uint64(data[40:48])
	Expect(48 + snapshotSize).To(BeNumerically("<=", registryOffset))
	Expect(registryOffset + registryByteCount).To(Equal(uint64(len(data))))
	return data[:48], data[48 : 48+snapshotSize], data[48+snapshotSize : registryOffset], data[registryOffset:]
}

func recGoldenFirstDiff(a, b []byte) int {
	for i := range min(len(a), len(b)) {
		if a[i] != b[i] {
			return i
		}
	}
	return min(len(a), len(b))
}

// recGoldenWorldOps are the opcodes whose first argument is the world id.
var recGoldenWorldOps = map[uint8]bool{}

func init() {
	for op := 0x01; op <= 0x0F; op++ {
		recGoldenWorldOps[uint8(op)] = true
	}
	for _, op := range []int{0x70, 0x80, 0x10, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xF1} {
		recGoldenWorldOps[uint8(op)] = true
	}
	for op := 0x90; op <= 0x98; op++ {
		recGoldenWorldOps[uint8(op)] = true
	}
}

// recGoldenSetWorldId rewrites the world id of every world op to worldId. The stored world id
// carries the world generation, which depends on how many worlds the process created before (the
// C generator runs one world per process). Body, shape and joint ids carry only the world slot,
// which is the same in both runs.
func recGoldenSetWorldId(ops []byte, worldId uint32) []byte {
	out := append([]byte(nil), ops...)
	for cursor := 0; cursor+4 <= len(out); {
		opcode := out[cursor]
		size := int(out[cursor+1]) | int(out[cursor+2])<<8 | int(out[cursor+3])<<16
		payload := out[cursor+4 : cursor+4+size]
		if recGoldenWorldOps[opcode] {
			binary.LittleEndian.PutUint32(payload, worldId)
		}
		cursor += 4 + size
	}
	return out
}

func recGoldenCompare(got []byte) {
	want, err := os.ReadFile("testdata/golem_post_toi/recording_golden.b3rec")
	Expect(err).NotTo(HaveOccurred())

	gh, _, gops, greg := recGoldenParts(got)
	wh, _, wops, wreg := recGoldenParts(want)

	// Header: everything but snapshotSize and registryOffset. validationEnabled follows the build.
	wantHeader := append([]byte(nil), wh...)
	if validateEnabled {
		wantHeader[10] = 1
	}
	Expect(gh[:24]).To(Equal(wantHeader[:24]))
	Expect(gh[40:48]).To(Equal(wantHeader[40:48]))

	// The first record is the StateHash anchor; take the C world id from it.
	Expect(wops[0]).To(Equal(uint8(recOpStateHash)))
	gops = recGoldenSetWorldId(gops, binary.LittleEndian.Uint32(wops[4:8]))

	if string(gops) != string(wops) {
		Fail(fmt.Sprintf("op stream differs at byte %d (got %d bytes, want %d)", recGoldenFirstDiff(gops, wops), len(gops), len(wops)))
	}
	if string(greg) != string(wreg) {
		Fail(fmt.Sprintf("registry differs at byte %d (got %d bytes, want %d)", recGoldenFirstDiff(greg, wreg), len(greg), len(wreg)))
	}
}

var _ = Describe("RecordingGoldenTest", func() {
	It("MatchesCorrectedC", func() {
		data, worldId := recGoldenRun(1)
		worldId.Destroy()
		recGoldenCompare(data)
	})

	It("MatchesCorrectedCWithWorkers", func() {
		for _, workers := range []int{2, 4} {
			data, worldId := recGoldenRun(workers)
			worldId.Destroy()
			recGoldenCompare(data)
		}
	})

	It("SaveAndLoad", func() {
		data, worldId := recGoldenRun(1)
		worldId.Destroy()

		path := GinkgoT().TempDir() + "/golden.b3rec"
		rec := CreateRecording(0)
		recBufAppend(&rec.buffer, data)
		Expect(SaveRecordingToFile(rec, path)).To(BeTrue())
		rec.Destroy()

		loaded := LoadRecordingFromFile(path)
		Expect(loaded).NotTo(BeNil())
		Expect(loaded.GetSize()).To(Equal(len(data)))
		Expect(loaded.GetData()).To(Equal(data))
		loaded.Destroy()

		// A file without the magic is rejected at load
		bad := append([]byte(nil), data...)
		bad[0] ^= 0xFF
		Expect(os.WriteFile(path, bad, 0o644)).To(Succeed())
		Expect(LoadRecordingFromFile(path)).To(BeNil())
		Expect(os.WriteFile(path, data[:47], 0o644)).To(Succeed())
		Expect(LoadRecordingFromFile(path)).To(BeNil())
	})

	// The tripwire of recording.c's def size _Static_asserts: a def that gains a field must update
	// its recW writer (and the replay.go reader) together with this count.
	It("DefTripwire", func() {
		fieldCounts := []struct {
			def   any
			count int
		}{
			{ExplosionDef{}, 5},
			{BodyDef{}, 20},
			{ShapeDef{}, 18},
			{JointDef{}, 12},
			{ParallelJointDef{}, 4},
			{DistanceJointDef{}, 13},
			{FilterJointDef{}, 1},
			{MotorJointDef{}, 11},
			{PrismaticJointDef{}, 11},
			{RevoluteJointDef{}, 11},
			{SphericalJointDef{}, 13},
			{WeldJointDef{}, 5},
			{WheelJointDef{}, 18},
		}
		for _, fc := range fieldCounts {
			t := reflect.TypeOf(fc.def)
			Expect(t.NumField()).To(Equal(fc.count), "%s changed: update recW%s and its reader together", t.Name(), t.Name())
		}
	})
})
