// Port of test/test_determinism.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"

	. "github.com/onsi/ginkgo/v2"
)

// PORT: TracyCFrameMark is dropped. printf goes to GinkgoWriter. Double precision is not ported,
// so only the float goldens are kept.
const (
	// Corrected pinned-C output: testdata/golem_post_toi/determinism.txt.
	ragdollSleepStep          = 371
	ragdollHash               = 0x3983DF54
	upstreamRagdollSleepStep  = 291
	upstreamRagdollHash       = 0xFF4C605B
	wavePileSleepStep         = 272
	wavePileHash              = 0x266F83B0
	querySpawnSleepStep       = 242
	querySpawnHash            = 0xF3AD9BA6
	querySpawnHitCount        = 59
	querySpawnQueryHash       = 0xE3271F3D
	meshDropSleepStep         = 196
	meshDropHash              = 0xBE70380B
	upstreamMeshDropSleepStep = 226
	upstreamMeshDropHash      = 0xD2FF069D
	determinismGoldenEnabled  = MaxManifoldPoints == 4
)

// The goldens above pin exact values for the default four point manifold. A build that
// overrides B3_MAX_MANIFOLD_POINTS produces a different contact set, so those checks drop
// out and only the run to run agreement below applies.
func ensureGolden(condition bool) {
	if determinismGoldenEnabled {
		GinkgoHelper()
		ensure(condition)
	}
}

// determinismNegZero is the C literal -0.0f (a Go constant -0.0 is +0).
var determinismNegZero = float32(math.Copysign(0, -1))

// determinismResult is DeterminismResult.
type determinismResult struct {
	sleepStep     int
	hash          uint32
	queryHitCount int
	queryHash     uint32
	seeded        bool
}

// Every worker count has to reproduce the first run exactly. Scenarios without queries leave
// the query fields zero in both the reference and the result.
func ensureRepeatable(reference *determinismResult, result determinismResult) int {
	GinkgoHelper()
	if reference.seeded == false {
		result.seeded = true
		*reference = result
		return 0
	}

	ensure(result.sleepStep == reference.sleepStep)
	ensure(result.hash == reference.hash)
	ensure(result.queryHitCount == reference.queryHitCount)
	ensure(result.queryHash == reference.queryHash)

	return 0
}

func singleMultithreadingTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)

	worldId := CreateWorld(&worldDef)

	data := createFallingRagdolls(worldId)

	timeStep := float32(1.0 / 60.0)

	stepLimit := 500
	for range stepLimit {
		subStepCount := 4
		worldId.Step(timeStep, subStepCount)

		done := updateFallingRagdolls(worldId, &data)
		if done {
			break
		}
	}

	worldId.Destroy()

	if data.sleepStep != ragdollSleepStep || data.hash != ragdollHash {
		fmt.Fprintf(GinkgoWriter, "  workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ensureGolden(data.sleepStep == ragdollSleepStep)
	ensureGolden(data.hash == ragdollHash)
	ensure(ensureRepeatable(reference, determinismResult{sleepStep: data.sleepStep, hash: data.hash}) == 0)

	destroyFallingRagdolls(&data)

	return 0
}

// Test multithreaded determinism.
func multithreadingTest() int {
	GinkgoHelper()
	reference := determinismResult{}
	for workerCount := 1; workerCount < 6; workerCount++ {
		result := singleMultithreadingTest(workerCount, &reference)
		ensure(result == 0)
	}

	return 0
}

// Test cross platform determinism.
func crossPlatformTest() int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldId := CreateWorld(&worldDef)

	data := createFallingRagdolls(worldId)

	timeStep := float32(1.0 / 60.0)

	done := false
	for done == false {
		subStepCount := 4
		worldId.Step(timeStep, subStepCount)

		done = updateFallingRagdolls(worldId, &data)
	}

	if data.sleepStep != ragdollSleepStep || data.hash != ragdollHash {
		fmt.Fprintf(GinkgoWriter, "  cross-platform sleepStep=%d hash=0x%08X\n", data.sleepStep, data.hash)
	}

	ensureGolden(data.sleepStep == ragdollSleepStep)
	ensureGolden(data.hash == ragdollHash)

	destroyFallingRagdolls(&data)

	worldId.Destroy()

	return 0
}

func singleWavePileTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)

	worldId := CreateWorld(&worldDef)

	data := createWavePile(worldId)

	timeStep := float32(1.0 / 60.0)

	// Rolling resistance must put the pile to sleep within 500 steps
	done := false
	for i := 0; i < 500 && done == false; i++ {
		subStepCount := 4
		worldId.Step(timeStep, subStepCount)

		done = updateWavePile(worldId, &data)
	}

	worldId.Destroy()

	if data.sleepStep != wavePileSleepStep || data.hash != wavePileHash {
		fmt.Fprintf(GinkgoWriter, "  wave pile workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ensure(done == true)
	ensureGolden(data.sleepStep == wavePileSleepStep)
	ensureGolden(data.hash == wavePileHash)
	ensure(ensureRepeatable(reference, determinismResult{sleepStep: data.sleepStep, hash: data.hash}) == 0)

	destroyWavePile(&data)

	return 0
}

// Test multithreaded determinism of a mixed convex pile on a wave height field.
func wavePileTest() int {
	GinkgoHelper()
	reference := determinismResult{}
	for workerCount := 1; workerCount <= 4; workerCount++ {
		result := singleWavePileTest(workerCount, &reference)
		ensure(result == 0)
	}

	return 0
}

func singleQuerySpawnTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)

	worldId := CreateWorld(&worldDef)

	data := createQuerySpawn(worldId)

	timeStep := float32(1.0 / 60.0)

	done := false
	for i := 0; i < 1000 && done == false; i++ {
		subStepCount := 4
		worldId.Step(timeStep, subStepCount)

		done = updateQuerySpawn(worldId, &data)
	}

	worldId.Destroy()

	if data.sleepStep != querySpawnSleepStep || data.hash != querySpawnHash || data.queryHitCount != querySpawnHitCount ||
		data.queryHash != querySpawnQueryHash {
		fmt.Fprintf(GinkgoWriter, "  query spawn workers=%d sleepStep=%d hash=0x%08X hits=%d queryHash=0x%08X\n", workerCount,
			data.sleepStep, data.hash, data.queryHitCount, data.queryHash)
	}

	ensure(done == true)
	ensure(data.spawnCount == querySpawnCount)
	ensureGolden(data.sleepStep == querySpawnSleepStep)
	ensureGolden(data.hash == querySpawnHash)
	ensureGolden(data.queryHitCount == querySpawnHitCount)
	ensureGolden(data.queryHash == querySpawnQueryHash)
	ensure(ensureRepeatable(reference, determinismResult{sleepStep: data.sleepStep,
		hash:          data.hash,
		queryHitCount: data.queryHitCount,
		queryHash:     data.queryHash}) == 0)

	destroyQuerySpawn(&data)

	return 0
}

// Test determinism of world queries by feeding their results back into the simulation.
func querySpawnTest() int {
	GinkgoHelper()
	reference := determinismResult{}
	for workerCount := 1; workerCount <= 4; workerCount++ {
		result := singleQuerySpawnTest(workerCount, &reference)
		ensure(result == 0)
	}

	return 0
}

func singleMeshDropTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)

	worldId := CreateWorld(&worldDef)

	data := createMeshDrop(worldId, PosZero)

	timeStep := float32(1.0 / 60.0)

	done := false
	for i := 0; i < 400 && done == false; i++ {
		subStepCount := 4
		worldId.Step(timeStep, subStepCount)

		done = updateMeshDrop(worldId, &data)
	}

	worldId.Destroy()

	if data.sleepStep != meshDropSleepStep || data.hash != meshDropHash {
		fmt.Fprintf(GinkgoWriter, "  mesh drop workers=%d sleepStep=%d hash=0x%08X\n", workerCount, data.sleepStep, data.hash)
	}

	ensure(done == true)
	ensureGolden(data.sleepStep == meshDropSleepStep)
	ensureGolden(data.hash == meshDropHash)
	ensure(ensureRepeatable(reference, determinismResult{sleepStep: data.sleepStep, hash: data.hash}) == 0)

	destroyMeshDrop(&data)

	return 0
}

// Test continuous collision determinism. Thin fast boxes need CCD against the wave mesh.
// The scene is large, so only the single threaded and widest schedules run.
func meshDropTest() int {
	GinkgoHelper()
	reference := determinismResult{}
	workerCounts := [2]int{1, 4}
	for i := range 2 {
		result := singleMeshDropTest(workerCounts[i], &reference)
		ensure(result == 0)
	}

	return 0
}

// hashFloats is HashFloats: b3Hash over the bytes of count floats.
// PORT: the floats are passed as a slice of the copied values instead of a pointer and a count.
func hashFloats(hash uint32, values []float32) uint32 {
	return Hash(hash, hashFloatBytes(values))
}

// determinismHashManifolds hashes the stored impulses of a body's contact manifolds, the inner
// loop shared by SingleRollingMixTest and SingleMeshMixTest.
func determinismHashManifolds(hash uint32, contactData []ContactData, contactCount int) uint32 {
	for c := range contactCount {
		for m := range int(contactData[c].ManifoldCount) {
			manifold := &contactData[c].Manifolds[m]
			hash = hashFloats(hash, []float32{manifold.RollingImpulse.X, manifold.RollingImpulse.Y, manifold.RollingImpulse.Z})
			hash = hashFloats(hash, []float32{manifold.FrictionImpulse.X, manifold.FrictionImpulse.Y, manifold.FrictionImpulse.Z})
			hash = hashFloats(hash, []float32{manifold.TwistImpulse})

			for p := range int(manifold.PointCount) {
				hash = hashFloats(hash, []float32{manifold.Points[p].NormalImpulse})
				hash = hashFloats(hash, []float32{manifold.Points[p].TotalNormalImpulse})
			}
		}
	}
	return hash
}

// Rolling resistance is mixed per contact and the manifolds have one, two, and four points, so a width 8
// constraint can mix lanes that width 4 keeps in separate constraints. The hash covers the stored contact
// impulses bitwise, including the sign of zero, because those survive into warm starting and contact data.
func singleRollingMixTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	groundDef := DefaultBodyDef()
	groundId := worldId.CreateBody(&groundDef)
	groundBox := MakeBoxHull(40.0, 1.0, 40.0)
	groundShapeDef := DefaultShapeDef()
	groundId.CreateHullShape(&groundShapeDef, &groundBox.Base)

	const (
		rowCount    = 8
		columnCount = 8
		bodyCount   = rowCount * columnCount
	)

	var bodyIds [bodyCount]BodyID
	box := MakeBoxHull(0.4, 0.4, 0.4)
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.4}
	capsule := Capsule{Vec3{-0.3, 0.0, 0.0}, Vec3{0.3, 0.0, 0.0}, 0.25}

	seed := uint32(271828)
	for i := range bodyCount {
		row := i / columnCount
		column := i % columnCount

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{float32(2.0*float32(column)) - 7.0, 1.45, float32(2.0*float32(row)) - 7.0}
		bodyIds[i] = worldId.CreateBody(&bodyDef)

		seed = seed*1664525 + 1013904223
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		if (seed >> 28) < 5 {
			shapeDef.BaseMaterial.RollingResistance = 0.2
		} else {
			shapeDef.BaseMaterial.RollingResistance = 0.0
		}

		switch i % 3 {
		case 0:
			bodyIds[i].CreateSphereShape(&shapeDef, &sphere)
		case 1:
			bodyIds[i].CreateCapsuleShape(&shapeDef, &capsule)
		default:
			bodyIds[i].CreateHullShape(&shapeDef, &box.Base)
		}

		sign := float32(1.0)
		if i&1 != 0 {
			sign = -1.0
		}
		bodyIds[i].SetLinearVelocity(Vec3{determinismNegZero, -0.5, sign * 0.5})
		bodyIds[i].SetAngularVelocity(Vec3{sign * 3.0, determinismNegZero, -2.0 * sign})
	}

	hash := uint32(HashInit)
	stepCount := 120
	for range stepCount {
		worldId.Step(1.0/60.0, 4)

		for i := range bodyCount {
			xf := bodyIds[i].GetTransform()
			hash = Hash(hash, scenesBytesOf(&xf))

			var contactData [4]ContactData
			contactCount := bodyIds[i].GetContactData(contactData[:])
			hash = determinismHashManifolds(hash, contactData[:], contactCount)
		}
	}

	worldId.Destroy()

	ensure(ensureRepeatable(reference, determinismResult{sleepStep: stepCount, hash: hash}) == 0)
	return 0
}

// Mesh contacts are grouped across lanes by manifold count and padded to the largest count in the group. Width 8 groups
// differ from width 4 groups, so mixing large and small shapes on a fine mesh gives each body a width dependent number
// of padded manifolds. The hash covers velocities and impulses bitwise to catch a padded manifold flipping a signed zero.
func singleMeshMixTest(workerCount int, reference *determinismResult) int {
	GinkgoHelper()
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	mesh := CreateWaveMesh(40, 40, 0.5, 0.3, 0.2, 0.3)
	{
		groundDef := DefaultBodyDef()
		groundDef.Position = Pos{-10.0, 0.0, -10.0}
		groundId := worldId.CreateBody(&groundDef)
		shapeDef := DefaultShapeDef()
		groundId.CreateMeshShape(&shapeDef, mesh, Vec3One)
	}

	const (
		rowCount    = 8
		columnCount = 8
		bodyCount   = rowCount * columnCount
	)

	var bodyIds [bodyCount]BodyID
	smallBox := MakeBoxHull(0.2, 0.2, 0.2)
	largeBox := MakeBoxHull(0.9, 0.3, 0.9)
	sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.3}
	capsule := Capsule{Vec3{-0.5, 0.0, 0.0}, Vec3{0.5, 0.0, 0.0}, 0.2}

	seed := uint32(314159)
	for i := range bodyCount {
		row := i / columnCount
		column := i % columnCount

		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{float32(2.2*float32(column)) - 7.7, 1.2, float32(2.2*float32(row)) - 7.7}
		bodyIds[i] = worldId.CreateBody(&bodyDef)

		seed = seed*1664525 + 1013904223
		shapeDef := DefaultShapeDef()
		shapeDef.Density = 1.0
		if (seed >> 28) < 5 {
			shapeDef.BaseMaterial.RollingResistance = 0.2
		} else {
			shapeDef.BaseMaterial.RollingResistance = 0.0
		}

		switch (seed >> 24) & 3 {
		case 0:
			bodyIds[i].CreateHullShape(&shapeDef, &largeBox.Base)
		case 1:
			bodyIds[i].CreateHullShape(&shapeDef, &smallBox.Base)
		case 2:
			bodyIds[i].CreateSphereShape(&shapeDef, &sphere)
		default:
			bodyIds[i].CreateCapsuleShape(&shapeDef, &capsule)
		}

		sign := float32(1.0)
		if i&1 != 0 {
			sign = -1.0
		}
		bodyIds[i].SetLinearVelocity(Vec3{determinismNegZero, -0.5, determinismNegZero})
		bodyIds[i].SetAngularVelocity(Vec3{determinismNegZero, sign * 0.5, determinismNegZero})
	}

	hash := uint32(HashInit)
	stepCount := 180
	for range stepCount {
		worldId.Step(1.0/60.0, 4)

		for i := range bodyCount {
			xf := bodyIds[i].GetTransform()
			hash = Hash(hash, scenesBytesOf(&xf))

			v := bodyIds[i].GetLinearVelocity()
			w := bodyIds[i].GetAngularVelocity()
			hash = hashFloats(hash, []float32{v.X, v.Y, v.Z})
			hash = hashFloats(hash, []float32{w.X, w.Y, w.Z})

			var contactData [8]ContactData
			contactCount := bodyIds[i].GetContactData(contactData[:])
			hash = determinismHashManifolds(hash, contactData[:], contactCount)
		}
	}

	worldId.Destroy()
	DestroyMesh(mesh)

	ensure(ensureRepeatable(reference, determinismResult{sleepStep: stepCount, hash: hash}) == 0)
	return 0
}

// sceneFcn is SceneFcn.
type sceneFcn func(workerCount int, reference *determinismResult) int

func runSceneAtWidth(scene sceneFcn, width int, result *determinismResult) int {
	GinkgoHelper()
	SetSIMDWidth(width)
	status := scene(1, result)
	SetSIMDWidth(suiteSIMDWidth)
	return status
}

// Width 4 and the native width must produce bitwise identical simulations.
func simdWidthTest() int {
	GinkgoHelper()
	SetSIMDWidth(0)
	nativeWidth := GetSIMDWidth()
	if nativeWidth == 4 {
		fmt.Fprintf(GinkgoWriter, "  subtest skipped: SIMDWidthTest, native SIMD width is 4\n")
		SetSIMDWidth(suiteSIMDWidth)
		return 0
	}

	scenes := []sceneFcn{
		singleMultithreadingTest, singleWavePileTest, singleQuerySpawnTest, singleMeshDropTest, singleRollingMixTest, singleMeshMixTest,
	}

	for i := range arrayCount(scenes) {
		narrow := determinismResult{}
		wide := determinismResult{}

		narrowStatus := runSceneAtWidth(scenes[i], 4, &narrow)
		wideStatus := runSceneAtWidth(scenes[i], nativeWidth, &wide)

		ensure(narrowStatus == 0)
		ensure(wideStatus == 0)
		ensure(narrow.seeded && wide.seeded)
		ensure(narrow.sleepStep == wide.sleepStep)
		ensure(narrow.hash == wide.hash)
		ensure(narrow.queryHitCount == wide.queryHitCount)
		ensure(narrow.queryHash == wide.queryHash)
	}

	return 0
}

var _ = Describe("DeterminismTest", func() {
	It("MultithreadingTest", func() { ensure(multithreadingTest() == 0) })
	It("CrossPlatformTest", func() { ensure(crossPlatformTest() == 0) })
	It("WavePileTest", func() { ensure(wavePileTest() == 0) })
	It("QuerySpawnTest", func() { ensure(querySpawnTest() == 0) })
	It("MeshDropTest", func() { ensure(meshDropTest() == 0) })
	It("SIMDWidthTest", func() { ensure(simdWidthTest() == 0) })

	// PORT: Go-only. Upstream runs the rolling mix and mesh mix scenes only inside SIMDWidthTest,
	// which skips at native width 4 (the default build, and the box3d_simd build without AVX2);
	// here they also run at 1, 2 and 4 workers so the run to run agreement across schedules is
	// checked at every native width.
	It("RollingMixAndMeshMixRepeatable", func() {
		for _, scene := range []sceneFcn{singleRollingMixTest, singleMeshMixTest} {
			reference := determinismResult{}
			for _, workerCount := range []int{1, 2, 4} {
				ensure(scene(workerCount, &reference) == 0)
			}
		}
	})
})
