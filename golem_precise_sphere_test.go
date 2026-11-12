package box3d

import (
	"encoding/json"
	"math"
	"os"
	"testing"
)

func TestGolemPreciseSphereTriangleFeatures(t *testing.T) {
	leaf := queryLeaf{triangleIndex: 7, transform: TransformIdentity, triangle: [3]Vec3{{X: -2, Z: -2}, {Z: 2}, {X: 2, Z: -2}}}
	for _, tc := range []struct {
		name                  string
		center, point, normal Vec3
		distance              float32
	}{
		{"face", Vec3{Y: 2}, Vec3{}, Vec3{Y: 1}, 1.75},
		{"edge", Vec3{Y: 2, Z: -2.15}, Vec3{Z: -2}, Vec3{Y: .8, Z: -.6}, 1.8},
		{"vertex", Vec3{X: 2.15, Y: 2, Z: -2.15}, Vec3{X: 2, Z: -2}, Vec3{X: .6, Y: float32(math.Sqrt(.28)), Z: -.6}, 2 - .25*float32(math.Sqrt(.28))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			out, valid := preciseSphereTriangle(leaf, tc.center, .25, Vec3{Y: -4}, 1e-6)
			if !valid || !out.Hit || AbsFloat(out.Fraction*4-tc.distance) > 1e-5 || Distance(out.Point, tc.point) > 1e-5 || Distance(out.Normal, tc.normal) > 1e-5 || out.TriangleIndex != 7 {
				t.Fatalf("geometric feature: %+v valid=%v", out, valid)
			}
		})
	}
	if out, valid := preciseSphereTriangle(leaf, Vec3{Y: 2, Z: -2.3}, .25, Vec3{Y: -4}, 1e-6); !valid || out.Hit {
		t.Fatalf("near-edge miss: %+v %v", out, valid)
	}
	if out, valid := preciseSphereTriangle(leaf, Vec3{Y: 2}, .25, Vec3{Y: -1.7499995}, 1e-6); !valid || !out.Hit {
		t.Fatalf("endpoint tolerance band: %+v %v", out, valid)
	}
}

func TestGolemPreciseSphereAuthoredTerrainAnalyticReference(t *testing.T) {
	data, err := os.ReadFile("testdata/golem_precise_terrain_spheres.json")
	if err != nil {
		t.Fatal(err)
	}
	var fixtures []struct {
		Name                string
		Vertices            [3][3]float32
		Origin, Translation [3]float32
		Radius              float32
		Analytic            struct {
			Distance      float64
			Point, Normal [3]float64
		}
	}
	if err = json.Unmarshal(data, &fixtures); err != nil {
		t.Fatal(err)
	}
	for _, fixture := range fixtures {
		t.Run(fixture.Name, func(t *testing.T) {
			w, body := golemQueryWorld(t)
			points := make([]Vec3, 3)
			for i, v := range fixture.Vertices {
				points[i] = Vec3{v[0], v[1], v[2]}
			}
			mesh := CreateMesh(&MeshDef{Vertices: points, VertexCount: 3, Indices: []int32{0, 1, 2}, TriangleCount: 1}, nil)
			if mesh == nil {
				t.Fatal("mesh")
			}
			sd := DefaultShapeDef()
			body.CreateMeshShape(&sd, mesh, Vec3One)
			proxy := ShapeProxy{Points: []Vec3{{}}, Count: 1, Radius: fixture.Radius}
			translation := Vec3{fixture.Translation[0], fixture.Translation[1], fixture.Translation[2]}
			count := 0
			_, valid := w.PreciseCastShape(Pos{fixture.Origin[0], fixture.Origin[1], fixture.Origin[2]}, &proxy, translation, DefaultQueryFilter(), 1e-6, func(_ ShapeID, point Pos, normal Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
				count++
				distanceError := math.Abs(float64(f*Length(translation)) - fixture.Analytic.Distance)
				pointError := math.Sqrt(math.Pow(float64(point.X)-fixture.Analytic.Point[0], 2) + math.Pow(float64(point.Y)-fixture.Analytic.Point[1], 2) + math.Pow(float64(point.Z)-fixture.Analytic.Point[2], 2))
				normalError := math.Sqrt(math.Pow(float64(normal.X)-fixture.Analytic.Normal[0], 2) + math.Pow(float64(normal.Y)-fixture.Analytic.Normal[1], 2) + math.Pow(float64(normal.Z)-fixture.Analytic.Normal[2], 2))
				if distanceError > .0001 || pointError > .001 || normalError > 1e-5 {
					t.Fatalf("analytic distance/point/normal errors=%g/%g/%g", distanceError, pointError, normalError)
				}
				return 1
			}, nil)
			if !valid || count != 1 {
				t.Fatalf("analytic terrain query incomplete: valid=%v count=%d", valid, count)
			}
		})
	}
}

func TestGolemPreciseSphereTerrainLongSweepAndBackfaces(t *testing.T) {
	for _, offset := range []float32{0, 768, 2304, -2304} {
		w, body := golemQueryWorld(t)
		body.SetTransform(Pos{X: offset, Y: 50, Z: offset}, QuatIdentity)
		vertices := []Vec3{{X: 500, Z: 500}, {X: 500, Z: 502}, {X: 502, Z: 500}}
		mesh := CreateMesh(&MeshDef{Vertices: vertices, VertexCount: 3, Indices: []int32{0, 1, 2}, TriangleCount: 1}, nil)
		if mesh == nil {
			t.Fatal("mesh")
		}
		sd := DefaultShapeDef()
		body.CreateMeshShape(&sd, mesh, Vec3One)
		proxy := ShapeProxy{Points: []Vec3{{}}, Count: 1, Radius: .25}
		for _, backfaces := range []bool{false, true} {
			filter := DefaultQueryFilter()
			filter.BackFaces = backfaces
			count := 0
			_, valid := w.PreciseCastShape(Pos{X: offset + 500.5, Y: 138, Z: offset + 500.5}, &proxy, Vec3{Y: -512}, filter, 1e-6, func(_ ShapeID, point Pos, normal Vec3, f float32, _ uint64, _, _ int, _ any) float32 {
				count++
				if AbsFloat(f*512-87.75) > 1e-5 || AbsFloat(point.Y-50) > 1e-5 || normal.Y < .999999 {
					t.Fatalf("offset%g face: point=%v normal=%v distance=%g", offset, point, normal, f*512)
				}
				return 1
			}, nil)
			if !valid || count != 1 {
				t.Fatalf("offset%g long sweep: valid=%v count=%d", offset, valid, count)
			}
			count = 0
			_, valid = w.PreciseCastShape(Pos{X: offset + 500.5, Y: 0, Z: offset + 500.5}, &proxy, Vec3{Y: 100}, filter, 1e-6, func(_ ShapeID, _ Pos, _ Vec3, _ float32, _ uint64, _, _ int, _ any) float32 { count++; return 1 }, nil)
			if !valid || backfaces && count != 1 || !backfaces && count != 0 {
				t.Fatalf("backface=%v valid=%v count=%d", backfaces, valid, count)
			}
		}
	}
}
