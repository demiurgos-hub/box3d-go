package box3d

import (
	"bytes"
	"math"
	"testing"
)

func cookingLargeHull(t *testing.T) *HullData {
	t.Helper()
	var points []Vec3
	for i := 0; i < 170; i++ {
		y := 1 - 2*float64(i)/169
		r := math.Sqrt(1 - y*y)
		theta := math.Pi * (3 - math.Sqrt(5)) * float64(i)
		points = append(points, Vec3{float32(r * math.Cos(theta)), float32(y), float32(r * math.Sin(theta))})
	}
	hull := CreateHullUnreduced(points, len(points))
	if hull == nil || hull.VertexCount <= 128 || hull.FaceCount <= 255 || hull.EdgeCount <= 255 {
		t.Fatal("fixture did not exercise widened topology", hull)
	}
	return hull
}

func TestGolemLargeHullTopologyQueriesAndCopy(t *testing.T) {
	hull := cookingLargeHull(t)
	vertices, edges, faces := GetHullVertices(hull), GetHullEdges(hull), GetHullFaces(hull)
	for i, vertex := range vertices {
		if int(vertex.Edge) >= len(edges) || int(edges[vertex.Edge].Origin) != i {
			t.Fatal("vertex edge truncated", i, vertex)
		}
	}
	for i, edge := range edges {
		if int(edge.Next) >= len(edges) || int(edge.Twin) >= len(edges) || int(edge.Origin) >= len(vertices) || int(edge.Face) >= len(faces) || int(edges[edge.Twin].Twin) != i {
			t.Fatal("half-edge reference truncated", i, edge)
		}
	}
	for i, face := range faces {
		index, count := int(face.Edge), 0
		for {
			if index >= len(edges) || int(edges[index].Face) != i || count > len(edges) {
				t.Fatal("face loop invalid", i, index)
			}
			count++
			index = int(edges[index].Next)
			if index == int(face.Edge) {
				break
			}
		}
		if count < 3 {
			t.Fatal("degenerate face")
		}
	}
	clone := CloneHull(hull)
	if !bytes.Equal(MarshalHullData(hull), MarshalHullData(clone)) || clone.Version != HullVersion {
		t.Fatal("native image copy changed topology")
	}
	for _, direction := range GetHullPoints(hull) {
		input := RayCastInput{Origin: MulSV(3, direction), Translation: MulSV(-4, direction), MaxFraction: 1}
		hit := RayCastHull(hull, &input)
		if !hit.Hit || AbsFloat(hit.Fraction-.5) > 1e-5 {
			t.Fatalf("large hull ray lost vertex/support: %v %+v", direction, hit)
		}
	}
}

func TestGolemLargeHullContactCache(t *testing.T) {
	w := nativeTOIWorld(t, 1)
	wd := getWorldFromId(w)
	hull := cookingLargeHull(t)
	sd := DefaultShapeDef()
	d := DefaultBodyDef()
	ground := w.CreateBody(&d)
	ground.CreateHullShape(&sd, hull)
	d.Type, d.Position = DynamicBody, Pos{Y: 1.9}
	body := w.CreateBody(&d)
	body.CreateHullShape(&sd, hull)
	w.SetGravity(Vec3{Y: -10})
	var contacts [4]ContactData
	for range 40 {
		w.Step(1.0/80, 1)
		if !IsValidFloat(body.GetPosition().Y) || body.GetPosition().Y < 1.8 {
			t.Fatal("large hull contact escaped", body.GetPosition())
		}
		for _, c := range contacts[:body.GetContactData(contacts[:])] {
			for _, m := range c.Manifolds[:c.ManifoldCount] {
				for _, point := range m.Points[:m.PointCount] {
					if _, ok := DecodeFeaturePair(point.FeatureId); !ok && point.FeatureId != 0xffffffff {
						t.Fatal("invalid cached contact feature", point.FeatureId)
					}
				}
			}
		}
	}
	if len(wd.contacts) == 0 || body.GetContactData(contacts[:]) == 0 {
		t.Fatal("large hull fixture did not retain contact")
	}
}

func TestGolemUnreducedHullAcceptsRedundantInput(t *testing.T) {
	var points []Vec3
	for x := -1; x <= 1; x += 2 {
		for y := -1; y <= 1; y += 2 {
			for z := -1; z <= 1; z += 2 {
				points = append(points, Vec3{float32(x), float32(y), float32(z)})
			}
		}
	}
	for range 200 {
		points = append(points, points[:8]...)
		points = append(points, Vec3{X: .25})
	}
	hull := CreateHullUnreduced(points, len(points))
	if hull == nil || hull.VertexCount != 8 || hull.FaceCount != 6 {
		t.Fatal("redundant input was treated as output complexity", hull)
	}
	if hull.Volume != 8 {
		t.Fatal("hull geometry changed", hull.Volume)
	}
}

func TestGolemUnreducedHullRejectsBudgetSimplification(t *testing.T) {
	var points []Vec3
	for latitude := 1; latitude < 16; latitude++ {
		phi := math.Pi * float64(latitude) / 16
		for longitude := 0; longitude < 32; longitude++ {
			theta := 2 * math.Pi * float64(longitude) / 32
			points = append(points, Vec3{float32(math.Sin(phi) * math.Cos(theta)), float32(math.Cos(phi)), float32(math.Sin(phi) * math.Sin(theta))})
		}
	}
	if CreateHull(points, len(points), 16) == nil {
		t.Fatal("legacy reduced control failed")
	}
	if CreateHullUnreduced(points, len(points)) != nil {
		t.Fatal("accepted a budget-reduced hull")
	}
}

func TestGolemMeshRetainsSmallNonzeroTriangles(t *testing.T) {
	d := MeshDef{
		Vertices: []Vec3{{}, {X: 1}, {Z: 1}, {X: 2}, {X: 2.0001}, {X: 2, Z: .0001}},
		Indices:  []int32{0, 2, 1, 3, 5, 4}, VertexCount: 6, TriangleCount: 2, IdentifyEdges: true,
	}
	legacy := CreateMesh(&d, nil)
	mesh := CreateMeshPreserveSmallTriangles(&d, nil)
	if legacy == nil || legacy.TriangleCount != 1 || mesh == nil || mesh.TriangleCount != 2 || mesh.DegenerateCount != 0 {
		t.Fatalf("legacy=%+v preserved=%+v", legacy, mesh)
	}
	seen := [2]bool{}
	for _, triangle := range GetMeshTriangles(mesh) {
		if triangle == (MeshTriangle{Index1: 0, Index2: 2, Index3: 1}) {
			seen[0] = true
		}
		if triangle == (MeshTriangle{Index1: 3, Index2: 5, Index3: 4}) {
			seen[1] = true
		}
	}
	if !seen[0] || !seen[1] {
		t.Fatal("source triangle identity lost", seen)
	}
	d.Indices = append(d.Indices, 0, 1, 1)
	d.TriangleCount++
	mesh = CreateMeshPreserveSmallTriangles(&d, nil)
	if mesh == nil || mesh.TriangleCount != 2 {
		t.Fatal("zero-area face acquired collision geometry")
	}
}
