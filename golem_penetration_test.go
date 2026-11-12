package box3d

import "testing"

func TestGolemPairPenetrationPrimitivesAndWitnesses(t *testing.T) {
	w, aBody := golemQueryWorld(t)
	bd := DefaultBodyDef()
	bd.Position = Pos{1.5, 0, 0}
	bBody := w.CreateBody(&bd)
	sd := DefaultShapeDef()
	sphere := Sphere{Radius: 1}
	a := aBody.CreateSphereShape(&sd, &sphere)
	b := bBody.CreateSphereShape(&sd, &sphere)
	r, valid := a.PairPenetration(b)
	if !valid {
		t.Fatal("valid pair rejected")
	}
	golemNear(t, r.Depth, 0.5)
	golemNear(t, r.Normal.X, 1)
	golemNear(t, r.PointA.X, 1)
	golemNear(t, r.PointB.X, 0.5)
	reverse, valid := b.PairPenetration(a)
	if !valid || reverse.PointA != r.PointB || reverse.PointB != r.PointA || reverse.Normal != Neg(r.Normal) {
		t.Fatalf("reversed pair: %+v vs %+v", reverse, r)
	}
	bBody.SetTransform(Pos{2, 0, 0}, QuatIdentity)
	r, valid = a.PairPenetration(b)
	if !valid || r.Depth != 0 || !r.Touching || r.Penetrating {
		t.Fatalf("touch became penetration: %+v", r)
	}
	bBody.SetTransform(Pos{2.0001, 0, 0}, QuatIdentity)
	r, valid = a.PairPenetration(b)
	if !valid || r.Depth != 0 || r.Touching || r.Penetrating {
		t.Fatalf("speculative contact became penetration: %+v", r)
	}
	bBody.SetTransform(Pos{1.9999, 0, 0}, QuatIdentity)
	r, valid = a.PairPenetration(b)
	if !valid || !r.Penetrating || r.Touching || r.Depth <= 0 || r.Depth > 0.0002 {
		t.Fatalf("small penetration lost: %+v", r)
	}
	b.Destroy(false)
	if _, valid = a.PairPenetration(b); valid {
		t.Fatal("destroyed other accepted")
	}
}

func TestGolemPairPenetrationHullDeepAndMesh(t *testing.T) {
	w, aBody := golemQueryWorld(t)
	sd := DefaultShapeDef()
	hull := MakeBoxHull(1, 1, 1)
	a := aBody.CreateHullShape(&sd, &hull.Base)
	bd := DefaultBodyDef()
	bd.Position = Pos{0.5, 0, 0}
	bBody := w.CreateBody(&bd)
	sphere := Sphere{Radius: 0.25}
	b := bBody.CreateSphereShape(&sd, &sphere)
	r, valid := a.PairPenetration(b)
	if !valid {
		t.Fatal("deep sphere rejected")
	}
	golemNear(t, r.Depth, 0.75)
	golemNear(t, r.PointA.X, 1)
	golemNear(t, r.PointB.X, 0.25)
	box := bBody.CreateHullShape(&sd, &hull.Base)
	r, valid = a.PairPenetration(box)
	if !valid {
		t.Fatal("hull pair rejected")
	}
	golemNear(t, r.Depth, 1.5)
	golemNear(t, r.PointA.X, 1)
	golemNear(t, r.PointB.X, -0.5)
	mesh := aBody.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	bBody.SetTransform(Pos{0.3, 0.75, 0.3}, QuatIdentity)
	r, valid = mesh.PairPenetration(box)
	if !valid || r.FeatureA.TriangleIndex < 0 {
		t.Fatalf("mesh/hull: %+v %v", r, valid)
	}
	golemNear(t, r.Depth, 0.25)
	golemNear(t, r.Normal.Y, 1)
	golemNear(t, r.PointA.Y, 0)
	golemNear(t, r.PointB.Y, -0.25)
	reverse, valid := box.PairPenetration(mesh)
	if !valid {
		t.Fatal("hull/mesh unsupported")
	}
	golemNear(t, reverse.Depth, r.Depth)
	golemNear(t, reverse.Normal.Y, -1)
	meshB := bBody.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	bBody.SetTransform(Pos{}, QuatIdentity)
	if _, valid = mesh.PairPenetration(meshB); valid {
		t.Fatal("triangle/triangle reported supported")
	}
}

func TestGolemProxyPenetrationOBBAndUnsupported(t *testing.T) {
	_, body := golemQueryWorld(t)
	sd := DefaultShapeDef()
	mesh := body.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	hull := MakeTransformedBoxHull(0.5, 0.5, 0.5, Transform{P: Vec3{0.2, 0.3, 0.2}, Q: QuatIdentity})
	proxy := ShapeProxy{GetHullPoints(&hull.Base), hull.Base.VertexCount, 0}
	var contacts [16]CapsulePenetration
	n, total, valid := mesh.ProxyPenetrations(Pos{}, &proxy, false, contacts[:])
	if !valid || n == 0 || n != total {
		t.Fatalf("OBB contact missing: %d %d %v", n, total, valid)
	}
	for _, c := range contacts[:n] {
		golemNear(t, c.Normal.Y, 1)
		golemNear(t, c.Depth, 0.2)
	}
	proxy.Radius = 0.1
	if _, _, valid = mesh.ProxyPenetrations(Pos{}, &proxy, true, contacts[:]); valid {
		t.Fatal("rounded hull silently weakened")
	}
	proxy = ShapeProxy{[]Vec3{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}}, 4, 0}
	if _, _, valid = mesh.ProxyPenetrations(Pos{}, &proxy, true, contacts[:]); valid {
		t.Fatal("coplanar hull silently weakened")
	}
}

func TestGolemHeightfieldCookedBoundsEncloseSurface(t *testing.T) {
	d := HeightFieldDef{Heights: []float32{0, 0, 0, 0}, Scale: Vec3One, CountX: 2, CountZ: 2, GlobalMinimumHeight: -1, GlobalMaximumHeight: 1}
	hf := CreateHeightField(&d)
	tri := getHeightFieldTriangle(hf, 0)
	bounds := ComputeHeightFieldQueryAABB(hf, TransformIdentity)
	for _, p := range tri.vertices {
		if p.Y < bounds.LowerBound.Y || p.Y > bounds.UpperBound.Y {
			t.Fatalf("cooked height %g outside %+v", p.Y, bounds)
		}
	}
}

func TestGolemDeepCapsuleMirroredSeamBothSides(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	s := b.CreateMeshShape(&sd, golemFlatMesh(), Vec3{-1, 1, 1})
	var hits [16]CapsulePenetration
	for _, side := range []float32{1, -1} {
		c := Capsule{Center1: Vec3{0, -0.3 * side, 0}, Center2: Vec3{0, 0.5 * side, 0}, Radius: 0.25}
		n, total, valid := s.CapsulePenetrations(Pos{}, &c, true, hits[:])
		if !valid || n == 0 || n != total {
			t.Fatal("mirrored seam contacts missing")
		}
		for _, hit := range hits[:n] {
			if hit.Normal.Y*side < 0.9 {
				t.Fatalf("artificial seam wall: %+v", hit)
			}
		}
	}
}

func TestGolemProxyExactTouchGapAndPenetration(t *testing.T) {
	_, b := golemQueryWorld(t)
	sd := DefaultShapeDef()
	sphere := Sphere{Radius: 1}
	s := b.CreateSphereShape(&sd, &sphere)
	var contacts [8]CapsulePenetration
	for _, x := range []float32{2, 2.0001, 1.9999} {
		proxy := ShapeProxy{[]Vec3{{x, 0, 0}}, 1, 1}
		n, total, valid := s.ProxyPenetrations(Pos{}, &proxy, false, contacts[:])
		if !valid || n != total {
			t.Fatal("valid proxy rejected")
		}
		if x > 2 && n != 0 {
			t.Fatal("positive separation admitted by solver slop")
		}
		if x == 2 && (n != 1 || contacts[0].Depth != 0) {
			t.Fatal("exact touch not emitted")
		}
		if x < 2 && (n != 1 || contacts[0].Depth <= 0) {
			t.Fatal("small penetration lost")
		}
	}
}
