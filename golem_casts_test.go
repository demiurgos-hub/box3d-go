package box3d

import "testing"

func TestGolemQueryMaskAndBackfaceRay(t *testing.T) {
	w, body := golemQueryWorld(t)
	sd := DefaultShapeDef()
	sd.Filter.CategoryBits, sd.Filter.MaskBits = 4, 0
	shape := body.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	f := DefaultQueryFilter()
	f.MaskBits = 4
	if hit := w.CastRayClosest(Pos{0.3, 1, 0.2}, Vec3{0, -2, 0}, f); hit.Hit {
		t.Fatal("upstream symmetric filter changed")
	}
	f.IgnoreShapeMask = true
	if hit := w.CastRayClosest(Pos{0.3, 1, 0.2}, Vec3{0, -2, 0}, f); !hit.Hit || hit.ShapeId != shape {
		t.Fatalf("query-only shape invisible: %+v", hit)
	}
	if hit := w.CastRayClosest(Pos{0.3, -1, 0.2}, Vec3{0, 2, 0}, f); hit.Hit {
		t.Fatal("backface admitted by default")
	}
	f.BackFaces = true
	hit := w.CastRayClosest(Pos{0.3, -1, 0.2}, Vec3{0, 2, 0}, f)
	if !hit.Hit || hit.Normal.Y > -0.9 || hit.TriangleIndex < 0 {
		t.Fatalf("backface opt-in: %+v", hit)
	}
	golemNear(t, hit.Fraction, 0.5)
	f.MaskBits = 8
	if hit = w.CastRayClosest(Pos{0.3, -1, 0.2}, Vec3{0, 2, 0}, f); hit.Hit {
		t.Fatal("query category selection bypassed")
	}
}

func TestGolemBackfaceShapeCastAndCompoundMaterial(t *testing.T) {
	w, body := golemQueryWorld(t)
	sd := DefaultShapeDef()
	m0, m1 := DefaultSurfaceMaterial(), DefaultSurfaceMaterial()
	m0.UserMaterialId, m1.UserMaterialId = 41, 42
	cd := CompoundDef{Meshes: []CompoundMeshDef{{MeshData: golemFlatMesh(), Transform: TransformIdentity, Scale: Vec3One,
		Materials: []SurfaceMaterial{m0, m1}, MaterialCount: 2}}, MeshCount: 1}
	shape := body.CreateBakedCompoundShape(&sd, CreateCompound(&cd))
	f := DefaultQueryFilter()
	proxy := ShapeProxy{[]Vec3{{0.3, -1, 0.2}}, 1, 0.2}
	var hits []WorldCastOutput
	callback := func(id ShapeID, point Pos, normal Vec3, fraction float32, material uint64, triangle, child int, _ any) float32 {
		if id != shape || material != 42 || child != 0 || triangle < 0 {
			t.Fatalf("identity/material lost: %v %d %d %d", id, material, child, triangle)
		}
		hits = append(hits, WorldCastOutput{Point: point, Normal: normal, Fraction: fraction})
		return fraction
	}
	w.CastShape(Pos{}, &proxy, Vec3{0, 2, 0}, f, callback, nil)
	if len(hits) != 0 {
		t.Fatal("default shape cast hit backface")
	}
	f.BackFaces = true
	w.CastShape(Pos{}, &proxy, Vec3{0, 2, 0}, f, callback, nil)
	if len(hits) != 1 || hits[0].Normal.Y > -0.9 {
		t.Fatalf("two-sided shape cast missing: %+v", hits)
	}
	// Upstream casts target radius minus LinearSlop; preserve that fraction.
	golemNear(t, hits[0].Fraction, 0.4025)
	// ShapeCast keeps upstream initial-overlap behavior. Penetrations are a separate query.
	hits = nil
	proxy.Points[0].Y = -0.1
	w.CastShape(Pos{}, &proxy, Vec3{0, 2, 0}, f, callback, nil)
	if len(hits) != 1 || hits[0].Fraction != 0 || hits[0].Normal != Vec3Zero {
		t.Fatal("shape cast initial-overlap semantics changed")
	}
}

func TestGolemExtendedFilterRecordingRejectsUnsupportedFormat(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Fatal("extension silently dropped from recording")
		}
	}()
	f := DefaultQueryFilter()
	f.IgnoreShapeMask = true
	recWQUERYFILTER(&recBuffer{}, f)
}
