package box3d

import (
	"fmt"
	"reflect"
	"strings"
	"testing"
)

func TestGolemRoundedMeshCullMatchesUnculledContacts(t *testing.T) {
	g := mcGoldenGen{seed: 12345}
	g.setupMaterials()
	mesh := g.makeGrid(12, .25, .035, 0)
	defer DestroyMesh(mesh)
	a := shape{typ: MeshShape, mesh: Mesh{mesh, Vec3{1.25, 1, -1}}, material: g.materials[0]}
	xfA := WorldTransform{Pos{X: 2304, Y: -1, Z: 768}, NormalizeQuat(Quat{V: Vec3{X: .02, Y: .1}, S: 1})}
	for _, typ := range []ShapeType{SphereShape, CapsuleShape} {
		t.Run(fmt.Sprint(typ), func(t *testing.T) {
			var worlds [2]mcGoldenGen
			for i := range worlds {
				worlds[i].setupWorld()
			}
			defer func() {
				for i := range worlds {
					for j := range worlds[i].world.manifoldAllocators {
						destroyBlockAllocator(&worlds[i].world.manifoldAllocators[j])
					}
					destroyMutex(worlds[i].world.manifoldAllocatorMutex)
				}
			}()
			b := shape{typ: typ, material: mcGoldenMaterialB(), sphere: Sphere{Center: Vec3{X: .03}, Radius: .24}, capsule: Capsule{Center1: Vec3{X: -.17, Y: .03}, Center2: Vec3{X: .17, Y: .1}, Radius: .2}}
			var contacts [2]contact
			for i := range contacts {
				contacts[i].flags = enableSpeculativePoints
			}
			arena := createArena(1 << 20)
			defer destroyArena(&arena)
			differedCaches := false
			contactFrames := 0
			for frame := 0; frame < 12; frame++ {
				step := frame
				if step > 5 {
					step = 11 - frame
				}
				position := Vec3{X: .8 + float32(step)*.09, Y: .19, Z: -.8 - float32(step)*.06}
				xfB := WorldTransform{OffsetPos(xfA.P, RotateVector(xfA.Q, position)), MulQuat(xfA.Q, NormalizeQuat(Quat{V: Vec3{Y: float32(step) * .03}, S: 1}))}
				mcGoldenComputeShapeAABB(&b, xfB)
				for mode := range worlds {
					touching := computeMeshManifoldsWithOptimizations(&worlds[mode].world, 0, &contacts[mode], &a, nil, xfA, &b, xfB, false, arena, mode == 0)
					worlds[mode].sb.Reset()
					worlds[mode].printContact("oracle", frame, touching, &contacts[mode], typ)
				}
				got, want := strings.Split(worlds[0].sb.String(), " | t")[0], strings.Split(worlds[1].sb.String(), " | t")[0]
				if got != want || !reflect.DeepEqual(contacts[0].manifolds, contacts[1].manifolds) {
					t.Fatalf("frame%d optimized contacts differ from unculled\ngot %s\nwant%s", frame, got, want)
				}
				if contacts[0].manifoldCount > 0 {
					contactFrames++
				}
				left, right := contacts[0].meshContact.triangleCache, contacts[1].meshContact.triangleCache
				if len(left) != len(right) {
					t.Fatal("candidate cache changed")
				}
				for i := range left {
					if left[i].triangleIndex != right[i].triangleIndex {
						t.Fatal("triangle cache order changed")
					}
					if left[i].cache.simplexCache != right[i].cache.simplexCache {
						differedCaches = true
					}
				}
				for i := range contacts {
					mcGoldenStampImpulses(&contacts[i], frame)
				}
			}
			if contactFrames == 0 {
				t.Fatal("oracle never produced contacts")
			}
			if typ == CapsuleShape && !differedCaches {
				t.Fatal("oracle never exercised culled GJK work")
			}
			for i := range contacts {
				freeManifolds(&worlds[i].world, contacts[i].manifolds, int(contacts[i].manifoldCount))
			}
		})
	}
}

func TestGolemRoundedMeshCullBoundsAreOutward(t *testing.T) {
	for _, center := range []float32{0, 2304} {
		s := shape{typ: CapsuleShape, capsule: Capsule{Center1: Vec3{X: center, Y: .2}, Center2: Vec3{X: center + .3, Y: .7}, Radius: .4}}
		bounds, ok := meshRoundedContactBounds(&s)
		reach := s.capsule.Radius + SpeculativeDistance()
		boundary := AABB{Sub(Min(s.capsule.Center1, s.capsule.Center2), Vec3{reach, reach, reach}), Add(Max(s.capsule.Center1, s.capsule.Center2), Vec3{reach, reach, reach})}
		if !ok || !bounds.Contains(boundary) || bounds.LowerBound == boundary.LowerBound || bounds.UpperBound == boundary.UpperBound {
			t.Fatal("bounds did not round outward")
		}
	}
}
