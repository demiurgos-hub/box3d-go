package box3d

import (
	"math"
	"testing"
)

func golemQueryWorld(t *testing.T) (WorldID, BodyID) {
	t.Helper()
	wd := DefaultWorldDef()
	w := CreateWorld(&wd)
	t.Cleanup(w.Destroy)
	bd := DefaultBodyDef()
	return w, w.CreateBody(&bd)
}

func golemNear(t *testing.T, got, want float32) {
	t.Helper()
	if AbsFloat(got-want) > 1e-4 {
		t.Fatalf("got %g, want %g", got, want)
	}
}

func golemFlatMesh() *MeshData {
	d := MeshDef{Vertices: []Vec3{{-2, 0, -2}, {-2, 0, 2}, {2, 0, -2}, {2, 0, 2}},
		Indices: []int32{0, 1, 2, 2, 1, 3}, VertexCount: 4, TriangleCount: 2,
		MaterialIndices: []uint8{0, 1}, IdentifyEdges: true}
	return CreateMesh(&d, nil)
}

func TestGolemClosestPointPrimitiveAndLifetime(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	sphere := Sphere{Center: Vec3{1, 0, 0}, Radius: 1}
	s := b.CreateSphereShape(&sd, &sphere)
	b.SetTransform(Pos{10, 2, 3}, QuatIdentity)
	r, ok := s.ClosestPointFeature(Pos{14, 2, 3}, 2)
	if !ok || r.Inside || r.Feature.ChildIndex != -1 || r.Feature.TriangleIndex != -1 {
		t.Fatalf("unexpected result: %+v, %v", r, ok)
	}
	golemNear(t, r.Point.X, 12)
	golemNear(t, r.Distance, 2)
	golemNear(t, r.Normal.X, 1)
	if _, ok = s.ClosestPointFeature(Pos{14, 2, 3}, 1.99); ok {
		t.Fatal("accepted beyond bound")
	}
	r, ok = s.ClosestPointFeature(Pos{11, 2, 3}, 0)
	if !ok || !r.Inside || r.Point != (Pos{11, 2, 3}) || r.Normal != Vec3Zero {
		t.Fatalf("inside: %+v", r)
	}
	if _, ok = s.ClosestPointFeature(Pos{}, float32(math.NaN())); ok {
		t.Fatal("accepted NaN")
	}
	s.Destroy(false)
	if _, ok = s.ClosestPointFeature(Pos{}, 1); ok {
		t.Fatal("accepted destroyed shape")
	}
	if _, _, ok = s.CapsulePenetrations(Pos{}, &Capsule{Radius: 1}, true, nil); ok {
		t.Fatal("accepted destroyed shape")
	}
}

func TestGolemClosestPointRotatedHullAndCapsule(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	h := MakeBoxHull(1, 2, 3)
	s := b.CreateHullShape(&sd, &h.Base)
	q := Quat{V: Vec3{0, 0, float32(math.Sqrt(0.5))}, S: float32(math.Sqrt(0.5))}
	b.SetTransform(Pos{5, 0, 0}, q)
	r, ok := s.ClosestPointFeature(Pos{5, 3, 0}, 3)
	if !ok {
		t.Fatal("hull not found")
	}
	golemNear(t, r.Point.Y, 1)
	golemNear(t, r.Distance, 2)
	r, ok = s.ClosestPointFeature(Pos{5, 0, 0}, 0)
	if !ok || !r.Inside {
		t.Fatalf("hull interior: %+v", r)
	}
	c := Capsule{Center1: Vec3{0, -1, 0}, Center2: Vec3{0, 1, 0}, Radius: 0.5}
	cs := b.CreateCapsuleShape(&sd, &c)
	r, ok = cs.ClosestPointFeature(Pos{5, 2, 0}, 2)
	if !ok {
		t.Fatal("capsule not found")
	}
	golemNear(t, r.Distance, 1.5)
}

func TestGolemClosestPointMeshScaleAndCompoundIdentity(t *testing.T) {
	_, b := golemQueryWorld(t)
	mesh := golemFlatMesh()
	m0, m1 := DefaultSurfaceMaterial(), DefaultSurfaceMaterial()
	m0.UserMaterialId, m1.UserMaterialId = 41, 42
	sd := DefaultShapeDef()
	sd.Materials = []SurfaceMaterial{m0, m1}
	sd.MaterialCount = 2
	s := b.CreateMeshShape(&sd, mesh, Vec3{-2, 1, 0.5})
	r, ok := s.ClosestPointFeature(Pos{-1, 1, 0.5}, 1)
	if !ok || r.Feature.TriangleIndex < 0 || r.MaterialID != 42 {
		t.Fatalf("mesh: %+v %v", r, ok)
	}
	golemNear(t, r.Distance, 1)
	golemNear(t, r.Point.Y, 0)
	cd := CompoundDef{Meshes: []CompoundMeshDef{{MeshData: mesh, Transform: Transform{P: Vec3{0, 3, 0}, Q: QuatIdentity}, Scale: Vec3One, Materials: []SurfaceMaterial{m0, m1}, MaterialCount: 2}}, MeshCount: 1}
	compound := CreateCompound(&cd)
	cs := b.CreateBakedCompoundShape(&sd, compound)
	r, ok = cs.ClosestPointFeature(Pos{1, 4, 1}, 1)
	if !ok || r.Feature.ChildIndex != 0 || r.Feature.TriangleIndex < 0 || r.MaterialID != 42 {
		t.Fatalf("compound: %+v %v", r, ok)
	}
	golemNear(t, r.Point.Y, 3)
}

func TestGolemHeightfieldHoleAndBoundedQuery(t *testing.T) {
	_, b := golemQueryWorld(t)
	d := HeightFieldDef{Heights: []float32{0, 0, 0, 0, 0, 0}, MaterialIndices: []uint8{HeightFieldHole, 0}, Scale: Vec3One, CountX: 3, CountZ: 2, GlobalMinimumHeight: -1, GlobalMaximumHeight: 1}
	h := CreateHeightField(&d)
	sd := DefaultShapeDef()
	s := b.CreateHeightFieldShape(&sd, h)
	if _, ok := s.ClosestPointFeature(Pos{0.25, 0.1, 0.5}, 0.2); ok {
		t.Fatal("hole became surface")
	}
	r, ok := s.ClosestPointFeature(Pos{1.5, 1, 0.5}, 2)
	if !ok || r.Feature.TriangleIndex < 2 {
		t.Fatalf("heightfield: %+v %v", r, ok)
	}
	golemNear(t, r.Point.Y, 0)
	if _, ok = s.ClosestPointFeature(Pos{1, 2, 0.5}, 100000); !ok {
		t.Fatal("bounded to terrain extent query failed")
	}
}

func TestGolemCapsulePenetrationDeepHullAndOverflow(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	h := MakeBoxHull(1, 1, 1)
	s := b.CreateHullShape(&sd, &h.Base)
	c := Capsule{Center1: Vec3{0, -0.5, 0}, Center2: Vec3{0, 0.5, 0}, Radius: 0.25}
	var all [8]CapsulePenetration
	n, total, ok := s.CapsulePenetrations(Pos{}, &c, false, all[:])
	if !ok || n == 0 || n != total {
		t.Fatalf("deep hull omitted: %d %d %v", n, total, ok)
	}
	for _, hit := range all[:n] {
		if hit.Depth < 1 || !IsNormalized(hit.Normal) {
			t.Fatalf("bad deep contact: %+v", hit)
		}
	}
	written, required, ok := s.CapsulePenetrations(Pos{}, &c, false, nil)
	if !ok || written != 0 || required != total {
		t.Fatalf("count-only: %d %d %v", written, required, ok)
	}
	if total > 1 {
		written, required, ok = s.CapsulePenetrations(Pos{}, &c, false, all[:1])
		if !ok || written != 1 || required != total {
			t.Fatal("overflow not reported")
		}
	}
	// Applying the deepest single-face correction clears this convex fixture.
	deepest := all[0]
	for _, hit := range all[:n] {
		if hit.Depth > deepest.Depth {
			deepest = hit
		}
	}
	delta := MulSV(deepest.Depth+0.001, deepest.Normal)
	c.Center1, c.Center2 = Add(c.Center1, delta), Add(c.Center2, delta)
	_, total, ok = s.CapsulePenetrations(Pos{}, &c, false, all[:])
	if !ok || total != 0 {
		t.Fatalf("correction did not clear hull: %d", total)
	}
}

func TestGolemCapsulePenetrationTriangleDeepAndBackfaces(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	s := b.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	c := Capsule{Center1: Vec3{0, -0.3, 0}, Center2: Vec3{0, 0.5, 0}, Radius: 0.25}
	var hits [16]CapsulePenetration
	n, total, ok := s.CapsulePenetrations(Pos{}, &c, false, hits[:])
	if !ok || n == 0 || n != total {
		t.Fatalf("deep triangle omitted: %d %d %v", n, total, ok)
	}
	for _, hit := range hits[:n] {
		if hit.Normal.Y < 0.9 || hit.Feature.TriangleIndex < 0 {
			t.Fatalf("triangle: %+v", hit)
		}
	}
	c = Capsule{Center1: Vec3{0, -0.1, 0}, Center2: Vec3{0, -0.1, 0}, Radius: 0.25}
	_, total, ok = s.CapsulePenetrations(Pos{}, &c, false, hits[:])
	if !ok || total != 0 {
		t.Fatal("backface admitted without opt-in")
	}
	n, total, ok = s.CapsulePenetrations(Pos{}, &c, true, hits[:])
	if !ok || n == 0 || total != n {
		t.Fatal("two-sided penetration missing")
	}
	for _, hit := range hits[:n] {
		if hit.Normal.Y > -0.9 {
			t.Fatalf("backface normal: %+v", hit)
		}
	}
}

func TestGolemCapsulePenetrationSphereAndCompound(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	sphere := Sphere{Radius: 1}
	s := b.CreateSphereShape(&sd, &sphere)
	c := Capsule{Center1: Vec3{1.1, 0, 0}, Center2: Vec3{1.1, 0, 0}, Radius: 0.25}
	var hits [8]CapsulePenetration
	n, _, ok := s.CapsulePenetrations(Pos{}, &c, false, hits[:])
	if !ok || n != 1 {
		t.Fatal("sphere missing")
	}
	golemNear(t, hits[0].Depth, 0.15)
	golemNear(t, hits[0].Normal.X, 1)
	m := DefaultSurfaceMaterial()
	m.UserMaterialId = 57
	d := CompoundDef{Spheres: []CompoundSphereDef{{Sphere: sphere, Material: m}}, SphereCount: 1}
	compound := CreateCompound(&d)
	cs := b.CreateBakedCompoundShape(&sd, compound)
	n, _, ok = cs.CapsulePenetrations(Pos{}, &c, false, hits[:])
	if !ok || n != 1 || hits[0].Feature.ChildIndex != 0 || hits[0].MaterialID != 57 {
		t.Fatalf("compound: %+v", hits[0])
	}
}
