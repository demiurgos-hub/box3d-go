package box3d

import "testing"

func TestGolemProxyDeepestMatchesFullDenseMesh(t *testing.T) {
	_, body := golemQueryWorld(t)
	const n = 20
	d := MeshDef{IdentifyEdges: true, VertexCount: (n + 1) * (n + 1), TriangleCount: 2 * n * n}
	for z := 0; z <= n; z++ {
		for x := 0; x <= n; x++ {
			d.Vertices = append(d.Vertices, Vec3{X: float32(x)/10 - 1, Z: float32(z)/10 - 1})
		}
	}
	for z := 0; z < n; z++ {
		for x := 0; x < n; x++ {
			a := int32(z*(n+1) + x)
			d.Indices = append(d.Indices, a, a+n+1, a+1, a+1, a+n+1, a+n+2)
		}
	}
	mesh := CreateMesh(&d, nil)
	defer DestroyMesh(mesh)
	sd := DefaultShapeDef()
	shape := body.CreateMeshShape(&sd, mesh, Vec3One)
	proxies := []ShapeProxy{{Points: []Vec3{{Y: .1}}, Count: 1, Radius: .8}, {Points: []Vec3{{Y: -.1}, {Y: .5}}, Count: 2, Radius: .8}}
	box := MakeTransformedBoxHull(.8, .3, .8, Transform{P: Vec3{Y: .1}, Q: QuatIdentity})
	proxies = append(proxies, ShapeProxy{Points: GetHullPoints(&box.Base), Count: box.Base.VertexCount})
	for i, proxy := range proxies {
		var small [8]CapsulePenetration
		_, total, valid := shape.ProxyPenetrations(Pos{}, &proxy, true, small[:])
		if !valid || total <= len(small) {
			t.Fatalf("proxy%d does not exercise overflow: %d valid=%v", i, total, valid)
		}
		all := make([]CapsulePenetration, total)
		written, again, valid := shape.ProxyPenetrations(Pos{}, &proxy, true, all)
		if !valid || written != total || again != total {
			t.Fatal("inconsistent full traversal")
		}
		want := all[0]
		for _, hit := range all[1:] {
			if hit.Depth > want.Depth {
				want = hit
			}
		}
		got, found, valid := shape.ProxyPenetrationDeepest(Pos{}, &proxy, true)
		if !valid || !found || got != want {
			t.Fatalf("proxy%d deepest changed: got=%+v want=%+v found=%v valid=%v", i, got, want, found, valid)
		}
	}
}

func TestGolemProxyDeepestTouchBackfaceAndReuse(t *testing.T) {
	_, body := golemQueryWorld(t)
	sd := DefaultShapeDef()
	sphere := body.CreateSphereShape(&sd, &Sphere{Radius: 1})
	for _, x := range []float32{2, 2.0001, 1.9999, 2} {
		proxy := ShapeProxy{Points: []Vec3{{X: x}}, Count: 1, Radius: 1}
		got, found, valid := sphere.ProxyPenetrationDeepest(Pos{}, &proxy, true)
		if !valid || found != (x <= 2) || found && got.Depth != MaxFloat(0, 2-x) {
			t.Fatalf("x=%g result=%+v found=%v valid=%v", x, got, found, valid)
		}
	}
	mesh := body.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	proxy := ShapeProxy{Points: []Vec3{{Y: -.1}}, Count: 1, Radius: .5}
	if _, found, valid := mesh.ProxyPenetrationDeepest(Pos{}, &proxy, false); !valid || found {
		t.Fatal("backface produced contact")
	}
	if _, found, valid := mesh.ProxyPenetrationDeepest(Pos{}, &proxy, true); !valid || !found {
		t.Fatal("two-sided contact lost")
	}
	if _, _, valid := mesh.ProxyPenetrationDeepest(Pos{}, nil, true); valid {
		t.Fatal("nil accepted")
	}
	proxy.Radius = -1
	if _, _, valid := mesh.ProxyPenetrationDeepest(Pos{}, &proxy, true); valid {
		t.Fatal("negative radius accepted")
	}
}
