package box3d

import (
	"math"
	"testing"
)

func TestGolemTriangleFaceDoesNotInventPlacementPenetration(t *testing.T) {
	w, body := golemQueryWorld(t)
	const n = 100
	d := MeshDef{IdentifyEdges: true, VertexCount: (n + 1) * (n + 1), TriangleCount: 2 * n * n}
	for z := 0; z <= n; z++ {
		for x := 0; x <= n; x++ {
			d.Vertices = append(d.Vertices, Vec3{X: -128 + 256*(float32(x)/n)/5, Z: -128 + 256*(float32(z)/n)/2})
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
	bd := DefaultBodyDef()
	bd.Position = Pos{X: -84.8, Z: -14}
	visitor := w.CreateBody(&bd)
	capsule := visitor.CreateCapsuleShape(&sd, &Capsule{Center1: Vec3{Y: .4}, Center2: Vec3{Y: 1.4}, Radius: .4})
	pair, valid := shape.PairPenetration(capsule)
	if !valid || !pair.Touching || pair.Penetrating || pair.Depth != 0 || pair.FeatureA.TriangleIndex <= 0 || pair.Normal != Vec3AxisY {
		t.Fatal("touching pair lost geometry identity", pair, valid)
	}
	for _, count := range []int32{1, 2} {
		for _, z := range []float32{-14, -5.6} {
			for _, height := range []float32{0, -.00001, -.0001, -.001, .00001, .0001} {
				proxy := ShapeProxy{Points: []Vec3{{Y: .4}, {Y: 1.4}}, Count: count, Radius: .4}
				got, found, valid := shape.ProxyPenetrationDeepest(Pos{X: -84.8, Y: height, Z: z}, &proxy, true)
				if !valid || found != (height <= 0) || found && math.Abs(float64(got.Depth+height)) > 2e-8 || height == 0 && got.Depth != 0 {
					t.Fatalf("count=%d height=%g: got=%+v found=%v valid=%v", count, height, got, found, valid)
				}
			}
		}
	}
}
