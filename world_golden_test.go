// Fork reference: testdata/golem_post_toi applies only the documented native
// post-TOI contact correction to pinned C. Original upstream fixtures remain intact.
// Go-only test: no upstream counterpart. Steps a small world without joints (grid mesh ground,
// tilted static box, box stack, rolling sphere, capsule, rock and cylinder hulls, a bullet and a
// pile of small boxes and spheres) and compares every body's transform and velocities, the awake
// body count, the contact and island counters and b3HashWorldState bit for bit against the pinned
// upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off.
// See testdata/world_golden_gen.c; this file mirrors it line by line. The C output is identical
// at 1 and 4 workers and for the NEON, BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds; the Go run
// is checked at 1, 2 and 4 workers.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	worldGoldenSteps  = 240
	worldGoldenStride = 20
)

func worldGoldenVec(sb *strings.Builder, v Vec3) {
	fmt.Fprintf(sb, " %08x %08x %08x", math.Float32bits(v.X), math.Float32bits(v.Y), math.Float32bits(v.Z))
}

func worldGoldenCreateDynamic(worldId WorldID, p Pos, q Quat) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = p
	bodyDef.Rotation = q
	return worldId.CreateBody(&bodyDef)
}

func worldGoldenRunScene(workerCount int) string {
	var sb strings.Builder

	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	var bodies []BodyID

	// Ground: grid mesh
	mesh := CreateGridMesh(16, 16, 1.0, 1, true)
	{
		bodyDef := DefaultBodyDef()
		groundId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		groundId.CreateMeshShape(&shapeDef, mesh, Vec3{1.0, 1.0, 1.0})
		bodies = append(bodies, groundId)
	}

	// Tilted static box (exact rotation: 3-4-5 about z)
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{5.0, 0.5, 0.0}
		bodyDef.Rotation = Quat{Vec3{0.0, 0.0, 0.6}, 0.8}
		staticId := worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		box := MakeBoxHull(2.0, 0.25, 1.5)
		staticId.CreateHullShape(&shapeDef, &box.Base)
		bodies = append(bodies, staticId)
	}

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0

	// Box stack
	cube := MakeCubeHull(0.5)
	for i := range 5 {
		p := Pos{0.0, float32(0.5) + float32(1.0*float32(i)), 0.0}
		if i&1 != 0 {
			p.X = 0.125
		}
		bodyId := worldGoldenCreateDynamic(worldId, p, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodies = append(bodies, bodyId)
	}

	// Sphere rolling in from the side with spin
	{
		bodyId := worldGoldenCreateDynamic(worldId, Pos{-4.0, 2.0, 0.5}, QuatIdentity)
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.5}
		shapeDef.BaseMaterial.Friction = 0.6
		shapeDef.BaseMaterial.RollingResistance = 0.05
		bodyId.CreateSphereShape(&shapeDef, &sphere)
		bodyId.SetLinearVelocity(Vec3{3.0, 0.0, -0.5})
		bodyId.SetAngularVelocity(Vec3{0.0, 2.0, -6.0})
		bodies = append(bodies, bodyId)
		shapeDef.BaseMaterial = DefaultSurfaceMaterial()
	}

	// Capsule falling tilted onto the mesh
	{
		bodyId := worldGoldenCreateDynamic(worldId, Pos{-2.0, 3.0, 2.0}, Quat{Vec3{0.6, 0.0, 0.0}, 0.8})
		capsule := Capsule{Vec3{-0.5, 0.0, 0.0}, Vec3{0.5, 0.0, 0.0}, 0.25}
		bodyId.CreateCapsuleShape(&shapeDef, &capsule)
		bodies = append(bodies, bodyId)
	}

	// Rock hull falling onto the tilted static box
	rock := CreateRock(0.6)
	{
		bodyId := worldGoldenCreateDynamic(worldId, Pos{5.0, 4.0, 0.25}, Quat{Vec3{0.0, 0.8, 0.0}, 0.6})
		bodyId.CreateHullShape(&shapeDef, rock)
		bodies = append(bodies, bodyId)
	}

	// Cylinder hull
	cylinder := CreateCylinder(1.0, 0.4, 0.0, 12)
	{
		bodyId := worldGoldenCreateDynamic(worldId, Pos{2.0, 2.5, -2.0}, Quat{Vec3{0.0, 0.0, 0.6}, 0.8})
		bodyId.CreateHullShape(&shapeDef, cylinder)
		bodies = append(bodies, bodyId)
	}

	// Bullet sphere fired at the stack
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = DynamicBody
		bodyDef.Position = Pos{0.0, 2.5, -6.0}
		bodyDef.LinearVelocity = Vec3{0.0, 0.0, 40.0}
		bodyDef.IsBullet = true
		bodyId := worldId.CreateBody(&bodyDef)
		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.125}
		bodyId.CreateSphereShape(&shapeDef, &sphere)
		bodies = append(bodies, bodyId)
	}

	// Pile of small boxes and spheres
	small := MakeBoxHull(0.25, 0.125, 0.25)
	for i := range 4 {
		for j := range 3 {
			fi, fj := float32(i), float32(j)
			p := Pos{float32(-3.0) + float32(0.75*fi), float32(0.5) + float32(0.5*fj), float32(-3.0) + float32(0.25*fj)}
			bodyId := worldGoldenCreateDynamic(worldId, p, QuatIdentity)
			if (i+j)&1 != 0 {
				sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.25}
				bodyId.CreateSphereShape(&shapeDef, &sphere)
			} else {
				bodyId.CreateHullShape(&shapeDef, &small.Base)
			}
			bodies = append(bodies, bodyId)
		}
	}

	world := getWorldFromId(worldId)

	for step := 1; step <= worldGoldenSteps; step++ {
		worldId.Step(1.0/60.0, 4)

		if step%worldGoldenStride != 0 && step != worldGoldenSteps {
			continue
		}

		for i, bodyId := range bodies {
			xf := bodyId.GetTransform()
			v := bodyId.GetLinearVelocity()
			w := bodyId.GetAngularVelocity()
			fmt.Fprintf(&sb, "body %d %d |", step, i)
			worldGoldenVec(&sb, ToVec3(xf.P))
			worldGoldenVec(&sb, xf.Q.V)
			fmt.Fprintf(&sb, " %08x", math.Float32bits(xf.Q.S))
			worldGoldenVec(&sb, v)
			worldGoldenVec(&sb, w)
			fmt.Fprintf(&sb, " %d\n", boolInt(bodyId.IsAwake()))
		}

		counters := worldId.GetCounters()
		fmt.Fprintf(&sb, "world %d | %d %d %d %016x\n", step, worldId.GetAwakeBodyCount(), counters.ContactCount,
			counters.IslandCount, hashWorldState(world))
	}

	worldId.Destroy()
	DestroyHull(rock)
	DestroyHull(cylinder)
	DestroyMesh(mesh)

	return sb.String()
}

func worldGoldenCompare(got string) {
	data, err := os.ReadFile("testdata/golem_post_toi/world_golden.txt")
	Expect(err).NotTo(HaveOccurred())
	want := strings.Split(strings.TrimRight(string(data), "\n"), "\n")
	lines := strings.Split(strings.TrimRight(got, "\n"), "\n")
	for i := range min(len(want), len(lines)) {
		if want[i] != lines[i] {
			Fail(fmt.Sprintf("first mismatch at line %d:\n got  %s\n want %s", i+1, lines[i], want[i]))
		}
	}
	Expect(len(lines)).To(Equal(len(want)))
}

var _ = Describe("WorldGoldenTest", func() {
	It("MatchesCorrectedC", func() {
		worldGoldenCompare(worldGoldenRunScene(1))
	})

	It("MatchesCorrectedCWithWorkers", func() {
		worldGoldenCompare(worldGoldenRunScene(2))
		worldGoldenCompare(worldGoldenRunScene(4))
	})
})
