package box3d

import "testing"

func TestGolemMeshStorageRoundedCapacityCannotTruncate(t *testing.T) {
	vertices := []Vec3{{X: -10, Z: -10}, {Z: 10}, {X: 10, Z: -10}}
	for _, typ := range []ShapeType{SphereShape, CapsuleShape} {
		pointsPerTriangle := 1
		if typ == CapsuleShape {
			pointsPerTriangle = 2
		}
		for _, count := range []int{0, 1, 2, MaxMeshContactTriangles} {
			capacity := meshContactPointCapacity(typ, count)
			points := make([]LocalManifoldPoint, capacity)
			processed, emitted := 0, 0
			// Exercise the unchanged production loop guard with a real kernel
			// emitting its maximum on every triangle, including the last.
			for processed < count && emitted+3 < capacity {
				manifold := LocalManifold{Points: points[emitted:]}
				if typ == SphereShape {
					CollideTriangleAndSphere(&manifold, capacity-emitted, vertices, &Sphere{Center: Vec3{Y: .1}, Radius: .2})
				} else {
					var cache SimplexCache
					CollideTriangleAndCapsule(&manifold, capacity-emitted, vertices, &Capsule{Center1: Vec3{X: -.5, Y: .1}, Center2: Vec3{X: .5, Y: .1}, Radius: .2}, &cache)
				}
				if int(manifold.PointCount) != pointsPerTriangle {
					t.Fatalf("shape %v expected maximum %d points, got %d", typ, pointsPerTriangle, manifold.PointCount)
				}
				emitted += int(manifold.PointCount)
				processed++
			}
			if processed != count || emitted != count*pointsPerTriangle {
				t.Fatalf("shape %v processed %d/%d triangles with capacity %d", typ, processed, count, capacity)
			}
			if capacity > maxPointsPerTriangle*count {
				t.Fatal("rounded storage exceeded original storage")
			}
		}
	}
	if meshContactPointCapacity(HullShape, MaxMeshContactTriangles) != maxPointsPerTriangle*MaxMeshContactTriangles {
		t.Fatal("hull clipping capacity changed")
	}
}

func TestGolemMeshStorageMetadataMatchesNative(t *testing.T) {
	g := mcGoldenGen{seed: 12345}
	mesh := g.makeGrid(1, 1, .02, 0)
	defer DestroyMesh(mesh)
	flags := getMeshFlagsWrite(mesh)
	flags[0] = uint8(ConcaveEdge1 | ConcaveEdge3 | InverseConcaveEdge2)
	flags[1] = uint8(ConcaveEdge2 | InverseConcaveEdge1 | InverseConcaveEdge3)
	for _, scaleX := range []float32{1.25, -1.25} {
		instance := Mesh{Data: mesh, Scale: Vec3{scaleX, 2, 3}}
		for index, tri := range GetMeshTriangles(mesh) {
			native := getMeshTriangle(&instance, index)
			want := meshTriangleInfo{native.i1, native.i2, native.i3, native.flags}
			got := meshTriangleMetadata(tri, flags[index], scaleX < 0)
			if got != want {
				t.Fatalf("triangle%d scale%g metadata %+v, want %+v", index, scaleX, got, want)
			}
		}
	}
}
