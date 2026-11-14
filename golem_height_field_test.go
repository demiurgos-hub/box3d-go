package box3d

import (
	"math/rand"
	"testing"
)

// golemField builds a field of explicit 16-bit samples with one hole.
func golemField(t *testing.T, count int, mainDiagonal bool) (*HeightFieldData, []uint16, []uint8) {
	t.Helper()
	random := rand.New(rand.NewSource(7))
	heights := make([]uint16, count*count)
	for i := range heights {
		heights[i] = uint16(random.Intn(32767))
	}
	materials := make([]uint8, (count-1)*(count-1))
	materials[len(materials)/2+1] = HeightFieldHole
	def := HeightFieldDef{CompressedHeights: heights, HeightStep: 128.0 / 32766, MaterialIndices: materials, Scale: Vec3{1.5, 1, 0.75},
		CountX: int32(count), CountZ: int32(count), MainDiagonal: mainDiagonal}
	return CreateHeightField(&def), heights, materials
}

func golemGridPoint(hf *HeightFieldData, heights []uint16, column, row int) Vec3 {
	h := mulAdd(hf.MinHeight, hf.HeightScale, float32(heights[row*int(hf.ColumnCount)+column]))
	return Mul(hf.Scale, Vec3{float32(column), h, float32(row)})
}

func sameCycle(a, b [3]Vec3) bool {
	for shift := 0; shift < 3; shift++ {
		if a[0] == b[shift] && a[1] == b[(shift+1)%3] && a[2] == b[(shift+2)%3] {
			return true
		}
	}
	return false
}

func TestGolemHeightFieldMainDiagonalTriangles(t *testing.T) {
	hf, heights, materials := golemField(t, 6, true)
	if hf.Diagonal != 1 || hf.HeightScale != 128.0/32766 || hf.MinHeight != 0 {
		t.Fatal("explicit layout or quantization was not recorded", hf.Diagonal, hf.HeightScale, hf.MinHeight)
	}
	for i, value := range GetHeightFieldCompressedHeights(hf) {
		if value != heights[i] {
			t.Fatal("explicit samples were requantized", i)
		}
	}
	columns := int(hf.ColumnCount)
	for cell := range materials {
		if materials[cell] == HeightFieldHole {
			continue
		}
		row, column := cell/(columns-1), cell%(columns-1)
		a, b := golemGridPoint(hf, heights, column, row), golemGridPoint(hf, heights, column+1, row)
		c, d := golemGridPoint(hf, heights, column, row+1), golemGridPoint(hf, heights, column+1, row+1)
		first, second := getHeightFieldTriangle(hf, 2*cell), getHeightFieldTriangle(hf, 2*cell+1)
		if !sameCycle(first.vertices, [3]Vec3{a, c, d}) || !sameCycle(second.vertices, [3]Vec3{a, d, b}) {
			t.Fatalf("cell %d is not split along its main diagonal: %v %v", cell, first.vertices, second.vertices)
		}
		if first.vertices[1] != d || first.vertices[2] != a || second.vertices[1] != a || second.vertices[2] != d {
			t.Fatalf("cell %d does not keep the shared diagonal as edge 2", cell)
		}
	}
}

func TestGolemHeightFieldGeneralFlagsMatchUpstream(t *testing.T) {
	for _, clockwise := range []bool{false, true} {
		hf := CreateWave(9, 11, Vec3{1, 2, 1.5}, 0.7, 1.3, true)
		if clockwise {
			def := HeightFieldDef{Heights: make([]float32, 99), Scale: Vec3{1, 1, 1}, CountX: 11, CountZ: 9, GlobalMinimumHeight: -1, GlobalMaximumHeight: 1, ClockwiseWinding: true}
			random := rand.New(rand.NewSource(3))
			for i := range def.Heights {
				def.Heights[i] = random.Float32()*2 - 1
			}
			def.MaterialIndices = make([]uint8, 80)
			def.MaterialIndices[17] = HeightFieldHole
			hf = CreateHeightField(&def)
		}
		count := int(hf.ColumnCount * hf.RowCount)
		heights := make([]float32, count)
		for i, q := range GetHeightFieldCompressedHeights(hf) {
			heights[i] = mulAdd(hf.MinHeight, hf.HeightScale, float32(q))
		}
		general := make([]uint8, len(GetHeightFieldFlags(hf)))
		computeHeightFieldFlags(hf, heights, GetHeightFieldMaterialIndices(hf), general, 0.9962)
		for i, value := range GetHeightFieldFlags(hf) {
			if general[i] != value {
				t.Fatalf("clockwise=%v triangle %d: general flags %02x, upstream %02x", clockwise, i, general[i], value)
			}
		}
	}
}

// golemMainDiagonalMesh triangulates the same field explicitly as a mesh oracle.
func golemMainDiagonalMesh(t *testing.T, hf *HeightFieldData, heights []uint16, materials []uint8) *Mesh {
	t.Helper()
	columns, rows := int(hf.ColumnCount), int(hf.RowCount)
	var vertices []Vec3
	for row := 0; row < rows; row++ {
		for column := 0; column < columns; column++ {
			vertices = append(vertices, golemGridPoint(hf, heights, column, row))
		}
	}
	var indices []int32
	for cell := range materials {
		if materials[cell] == HeightFieldHole {
			continue
		}
		row, column := cell/(columns-1), cell%(columns-1)
		a := int32(row*columns + column)
		b, c, d := a+1, a+int32(columns), a+int32(columns)+1
		indices = append(indices, a, c, d, a, d, b)
	}
	def := MeshDef{Vertices: vertices, Indices: indices, VertexCount: int32(len(vertices)), TriangleCount: int32(len(indices) / 3), IdentifyEdges: true}
	data := CreateMesh(&def, nil)
	if data == nil {
		t.Fatal("mesh oracle")
	}
	return &Mesh{Data: data, Scale: Vec3One}
}

func TestGolemHeightFieldMainDiagonalMatchesMeshQueries(t *testing.T) {
	hf, heights, materials := golemField(t, 9, true)
	mesh := golemMainDiagonalMesh(t, hf, heights, materials)
	random := rand.New(rand.NewSource(11))
	extentX, extentZ := hf.Scale.X*float32(hf.ColumnCount-1), hf.Scale.Z*float32(hf.RowCount-1)
	const tolerance = 1e-4
	for i := 0; i < 2000; i++ {
		origin := Vec3{random.Float32() * extentX, 140, random.Float32() * extentZ}
		target := Vec3{random.Float32() * extentX, -10, random.Float32() * extentZ}
		ray := RayCastInput{Origin: origin, Translation: Sub(target, origin), MaxFraction: 1}
		field, oracle := RayCastHeightField(hf, &ray), RayCastMesh(mesh, &ray)
		if field.Hit != oracle.Hit || field.Hit && (AbsFloat(field.Fraction-oracle.Fraction)*Length(ray.Translation) > tolerance || Dot(field.Normal, oracle.Normal) < 1-tolerance) {
			t.Fatalf("ray %d: field %+v, mesh %+v", i, field, oracle)
		}
		center := Vec3{random.Float32() * extentX, 140, random.Float32() * extentZ}
		cast := ShapeCastInput{Proxy: pointProxy(&center, 0.4), Translation: Vec3{random.Float32() - 0.5, -150, random.Float32() - 0.5}, MaxFraction: 1}
		field, oracle = ShapeCastHeightField(hf, &cast), ShapeCastMesh(mesh, &cast)
		if field.Hit != oracle.Hit || field.Hit && AbsFloat(field.Fraction-oracle.Fraction)*Length(cast.Translation) > tolerance {
			t.Fatalf("sphere cast %d: field %+v, mesh %+v", i, field, oracle)
		}
		probe := Vec3{random.Float32() * extentX, random.Float32() * 128, random.Float32() * extentZ}
		proxy := pointProxy(&probe, 0.5)
		if OverlapHeightField(hf, TransformIdentity, &proxy) != OverlapMesh(mesh, TransformIdentity, &proxy) {
			t.Fatalf("overlap %d disagrees at %v", i, probe)
		}
	}
}
