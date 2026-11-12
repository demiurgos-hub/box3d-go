// Go-only test: no upstream counterpart. Compares mesh, height field and compound creation (byte
// images through their byte count and content hash, height field dump/load, compound validation),
// triangle and child access, queries, ray and shape casts, overlaps and mover collision against the
// pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with
// -ffp-contract=off, bit for bit. The C output is identical for the NEON, SSE2,
// BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds. See testdata/geometry_golden_gen.c; this file
// mirrors it line by line. The selected corrected-C topology format is documented
// in testdata/golem_hull_topology/README.md; original fixtures remain unchanged.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const soupN = 6 // SOUP_N

type geometryGoldenGen struct {
	hullGoldenGen

	queryHash  uint64
	queryCount int
	queryStop  int

	proxyPoints [4]Vec3

	soupVertices  [soupN*soupN*6 + 9]Vec3
	soupIndices   [soupN*soupN*6 + 9]int32
	soupMaterials [soupN*soupN*2 + 3]uint8
	heights       [(soupN + 1) * (soupN + 1)]float32

	hfHeights   [hfColumns * hfRows]float32
	hfMaterials [(hfColumns - 1) * (hfRows - 1)]uint8
}

func (g *geometryGoldenGen) rndInt(n uint32) uint32 {
	g.seed ^= g.seed << 13
	g.seed ^= g.seed >> 17
	g.seed ^= g.seed << 5
	return g.seed % n
}

func (g *geometryGoldenGen) pbox(a AABB) {
	g.pv(a.LowerBound)
	g.pv(a.UpperBound)
}

func (g *geometryGoldenGen) pbool(b bool) {
	if b {
		g.printf(" 1")
	} else {
		g.printf(" 0")
	}
}

func (g *geometryGoldenGen) pco(o CastOutput) {
	g.pbool(o.Hit)
	g.pv(o.Normal)
	g.pv(o.Point)
	g.pf(o.Fraction)
	g.printf(" %d %d %d %d", o.Iterations, o.TriangleIndex, o.ChildIndex, o.MaterialIndex)
}

func (g *geometryGoldenGen) pplanes(planes []PlaneResult, count int) {
	g.printf(" %d", count)
	for i := range count {
		g.pv(planes[i].Plane.Normal)
		g.pf(planes[i].Plane.Offset)
		g.pv(planes[i].Point)
		g.printf(" %d %d %d", planes[i].TriangleIndex, planes[i].ChildIndex, planes[i].MaterialIndex)
	}
}

func (g *geometryGoldenGen) ptri(t triangle) {
	g.pv(t.vertices[0])
	g.pv(t.vertices[1])
	g.pv(t.vertices[2])
	g.printf(" %d %d %d %d", t.i1, t.i2, t.i3, t.flags)
}

func (g *geometryGoldenGen) hashWord(w uint32) {
	for i := range 4 {
		g.queryHash ^= uint64((w >> (8 * i)) & 0xFF)
		g.queryHash *= 0x100000001b3
	}
}

func (g *geometryGoldenGen) meshQueryFcn(a, b, c Vec3, triangleIndex int, context any) bool {
	g.hashWord(uint32(triangleIndex))
	for _, f := range [...]float32{a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z} {
		g.hashWord(math.Float32bits(f))
	}
	g.queryCount += 1
	return g.queryCount != g.queryStop
}

func (g *geometryGoldenGen) resetQuery(stop int) {
	g.queryHash = 0xcbf29ce484222325
	g.queryCount = 0
	g.queryStop = stop
}

func (g *geometryGoldenGen) pquery() {
	g.printf(" %d %016x", g.queryCount, g.queryHash)
}

func (g *geometryGoldenGen) pmesh(m *MeshData) {
	if m == nil {
		g.printf(" null")
		return
	}
	g.printf(" %d %016x %d %d %d %d %d %d %d", m.ByteCount, m.Hash, m.TreeHeight, m.NodeCount,
		m.VertexCount, m.TriangleCount, m.DegenerateCount, m.MaterialCount, GetHeight(m))
	g.pf(m.SurfaceArea)
	g.pbox(m.Bounds)
}

// rproxy is a random proxy: 1 to 4 points around center with a radius.
func (g *geometryGoldenGen) rproxy(center Vec3, spread float32) ShapeProxy {
	count := 1 + int(g.rndInt(4))
	for i := range count {
		offset := g.rv3(spread, spread, spread, 0.0, 0.0, 0.0)
		g.proxyPoints[i] = Add(center, offset)
	}
	var radius float32 = 0.0
	if g.rndInt(3) != 0 {
		radius = mulAdd(0.3, 0.25, g.rnd())
	}
	return ShapeProxy{g.proxyPoints[:], int32(count), radius}
}

func (g *geometryGoldenGen) soupDef() MeshDef {
	for i := range (soupN + 1) * (soupN + 1) {
		g.heights[i] = 0.5 * g.rnd()
	}

	v := 0
	t := 0
	for ix := range soupN {
		for iz := range soupN {
			p11 := Vec3{float32(ix), g.heights[ix*(soupN+1)+iz], float32(iz)}
			p12 := Vec3{float32(ix), g.heights[ix*(soupN+1)+iz+1], float32(iz + 1)}
			p21 := Vec3{float32(ix + 1), g.heights[(ix+1)*(soupN+1)+iz], float32(iz)}
			p22 := Vec3{float32(ix + 1), g.heights[(ix+1)*(soupN+1)+iz+1], float32(iz + 1)}

			g.soupVertices[v+0] = p11
			g.soupVertices[v+1] = p12
			g.soupVertices[v+2] = p22
			g.soupVertices[v+3] = p22
			g.soupVertices[v+4] = p21
			g.soupVertices[v+5] = p11
			for k := range 6 {
				g.soupIndices[v+k] = int32(v + k)
			}
			g.soupMaterials[t+0] = uint8(g.rndInt(5))
			g.soupMaterials[t+1] = uint8(g.rndInt(5))
			v += 6
			t += 2
		}
	}

	// Degenerates: a sliver (collinear), a repeated index and a tiny triangle.
	g.soupVertices[v+0] = Vec3{0.0, 1.0, 0.0}
	g.soupVertices[v+1] = Vec3{1.0, 1.0, 0.0}
	g.soupVertices[v+2] = Vec3{2.0, 1.0, 0.0}
	g.soupIndices[v+0] = int32(v + 0)
	g.soupIndices[v+1] = int32(v + 1)
	g.soupIndices[v+2] = int32(v + 2)
	g.soupVertices[v+3] = Vec3{0.0, 2.0, 0.0}
	g.soupVertices[v+4] = Vec3{1.0, 2.0, 0.0}
	g.soupVertices[v+5] = Vec3{0.0, 2.0, 1.0}
	g.soupIndices[v+3] = int32(v + 3)
	g.soupIndices[v+4] = int32(v + 3)
	g.soupIndices[v+5] = int32(v + 5)
	g.soupVertices[v+6] = Vec3{3.0, 2.0, 0.0}
	g.soupVertices[v+7] = Vec3{3.0001, 2.0, 0.0}
	g.soupVertices[v+8] = Vec3{3.0, 2.0, 0.0001}
	g.soupIndices[v+6] = int32(v + 6)
	g.soupIndices[v+7] = int32(v + 7)
	g.soupIndices[v+8] = int32(v + 8)
	g.soupMaterials[t+0] = 1
	g.soupMaterials[t+1] = 2
	g.soupMaterials[t+2] = 3
	v += 9
	t += 3

	def := MeshDef{}
	def.Vertices = g.soupVertices[:]
	def.Indices = g.soupIndices[:]
	def.MaterialIndices = g.soupMaterials[:]
	def.VertexCount = int32(v)
	def.TriangleCount = int32(t)
	return def
}

func (g *geometryGoldenGen) meshQueries(name string, data *MeshData, scale Vec3) {
	mesh := Mesh{data, scale}
	bounds := ComputeMeshAABB(data, TransformIdentity, scale)
	c := bounds.Center()
	e := bounds.Extents()

	g.printf("%s aabb |", name)
	{
		var xf Transform
		xf.P = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		g.pbox(ComputeMeshAABB(data, xf, scale))
	}
	g.printf("\n")

	g.printf("%s triangles |", name)
	for i := 0; i < int(data.TriangleCount); i += 1 + int(data.TriangleCount)/7 {
		g.ptri(getMeshTriangle(&mesh, i))
	}
	g.printf("\n")

	for k := range 12 {
		g.printf("%s ray %d |", name, k)
		var ray RayCastInput
		ray.Origin = g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		ray.Translation = MulSV(2.0, Sub(target, ray.Origin))
		ray.MaxFraction = 1.0
		if k%4 == 3 {
			ray.MaxFraction = 0.4
		}
		g.pco(RayCastMesh(&mesh, &ray))
		g.printf("\n")
	}

	for k := range 10 {
		g.printf("%s cast %d |", name, k)
		start := g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		var input ShapeCastInput
		input.Proxy = g.rproxy(start, 0.3)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		input.Translation = MulSV(2.0, Sub(target, start))
		input.MaxFraction = 1.0
		input.CanEncroach = k%3 == 1
		g.pco(ShapeCastMesh(&mesh, &input))
		g.printf("\n")
	}

	g.printf("%s overlap |", name)
	for range 16 {
		var xf Transform
		xf.P = g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		local := g.rv3(e.X, e.Y+0.5, e.Z, c.X, c.Y, c.Z)
		world := TransformPoint(xf, local)
		proxy := g.rproxy(world, 0.4)
		g.pbool(OverlapMesh(&mesh, xf, &proxy))
	}
	g.printf("\n")

	for k := range 8 {
		g.printf("%s mover %d |", name, k)
		var mover Capsule
		mover.Center1 = g.rv3(e.X, e.Y+0.3, e.Z, c.X, c.Y, c.Z)
		mover.Center2 = Add(mover.Center1, g.rv3(0.3, 0.3, 0.3, 0.0, 0.4, 0.0))
		mover.Radius = mulAdd(0.4, 0.3, g.rnd())
		var planes [16]PlaneResult
		capacity := 16
		if k == 5 {
			capacity = 2
		}
		count := collideMoverAndMesh(planes[:capacity], &mesh, &mover)
		g.pplanes(planes[:], count)
		g.printf("\n")
	}

	g.printf("%s query |", name)
	for k := range 6 {
		var box AABB
		p := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		h := g.rv3(0.5, 0.5, 0.5, 0.8, 0.8, 0.8)
		box.LowerBound = Sub(p, h)
		box.UpperBound = Add(p, h)
		stop := -1
		if k == 4 {
			stop = 3
		}
		g.resetQuery(stop)
		QueryMesh(&mesh, box, g.meshQueryFcn, nil)
		g.pquery()
	}
	g.printf("\n")
}

func (g *geometryGoldenGen) meshSection() {
	var meshes []*MeshData

	g.printf("mesh creators |")
	meshes = append(meshes, CreateGridMesh(5, 4, 0.75, 3, true))
	meshes = append(meshes, CreateGridMesh(3, 3, 1.0, 0, false))
	meshes = append(meshes, CreateWaveMesh(7, 6, 0.6, 0.4, 0.13, 0.21))
	meshes = append(meshes, CreateTorusMesh(10, 7, 2.0, 0.5))
	meshes = append(meshes, CreateBoxMesh(Vec3{0.1, 0.2, -0.3}, Vec3{0.5, 1.0, 1.5}, true))
	meshes = append(meshes, CreateHollowBoxMesh(Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 1.0, 1.5}))
	meshes = append(meshes, CreatePlatformMesh(Vec3{0.0, 1.0, 0.0}, 2.0, 1.0, 3.0))
	for _, m := range meshes {
		g.pmesh(m)
	}
	g.printf("\n")

	g.printf("mesh soup |")
	{
		saved := g.seed
		def := g.soupDef()
		degenerates := [4]int32{-1, -1, -1, -1}
		meshes = append(meshes, CreateMesh(&def, degenerates[:2]))
		g.pmesh(meshes[len(meshes)-1])
		g.printf(" %d %d %d %d", degenerates[0], degenerates[1], degenerates[2], degenerates[3])

		// Same soup, welded, SAH split
		g.seed = saved
		def = g.soupDef()
		def.WeldVertices = true
		def.WeldTolerance = 0.01
		def.IdentifyEdges = true
		meshes = append(meshes, CreateMesh(&def, nil))
		g.pmesh(meshes[len(meshes)-1])

		// Welded, median split, clockwise
		g.seed = saved
		def = g.soupDef()
		def.WeldVertices = true
		def.WeldTolerance = 0.01
		def.IdentifyEdges = true
		def.UseMedianSplit = true
		def.ClockWiseWinding = true
		meshes = append(meshes, CreateMesh(&def, nil))
		g.pmesh(meshes[len(meshes)-1])

		// Rejected defs
		def.Stride = 6
		g.pmesh(CreateMesh(&def, nil))
		def.Stride = 0
		def.VertexCount = 2
		g.pmesh(CreateMesh(&def, nil))
	}
	g.printf("\n")

	g.printf("mesh flags |")
	for _, m := range meshes {
		flags := GetMeshFlags(m)
		materials := GetMeshMaterialIndices(m)
		var sum uint32 = 0
		for j := range int(m.TriangleCount) {
			sum = 31*sum + uint32(flags[j]) + 7*uint32(materials[j])
		}
		g.printf(" %08x", sum)
	}
	g.printf("\n")

	scales := [4]Vec3{
		{1.0, 1.0, 1.0},
		{1.5, 0.5, 2.0},
		{-1.0, 1.0, 1.0},
		{0.75, -1.25, -0.5},
	}

	queried := [4]int{2, 3, 8, 9}
	for i := range 4 {
		for s := range 4 {
			name := fmt.Sprintf("mesh %d scale %d", queried[i], s)
			g.meshQueries(name, meshes[queried[i]], scales[s])
		}
	}

	for _, m := range meshes {
		DestroyMesh(m)
	}
}

func (g *geometryGoldenGen) phf(h *HeightFieldData) {
	if h == nil {
		g.printf(" null")
		return
	}
	g.printf(" %d %016x %d %d %d", h.ByteCount, h.Hash, h.ColumnCount, h.RowCount, h.Clockwise)
	g.pf(h.MinHeight)
	g.pf(h.MaxHeight)
	g.pf(h.HeightScale)
	g.pv(h.Scale)
	g.pbox(h.AABB)
}

const (
	hfColumns = 9 // HF_COLUMNS
	hfRows    = 7 // HF_ROWS
)

func (g *geometryGoldenGen) randomFieldDef(clockwise bool) HeightFieldDef {
	for i := range hfColumns * hfRows {
		// Some heights fall outside the global range and get clamped
		g.hfHeights[i] = 1.2 * g.rnd()
	}
	for i := range (hfColumns - 1) * (hfRows - 1) {
		r := g.rndInt(8)
		if r == 0 {
			g.hfMaterials[i] = HeightFieldHole
		} else {
			g.hfMaterials[i] = uint8(r)
		}
	}

	def := HeightFieldDef{}
	def.Heights = g.hfHeights[:]
	def.MaterialIndices = g.hfMaterials[:]
	def.Scale = Vec3{0.75, 2.0, 1.25}
	def.CountX = hfColumns
	def.CountZ = hfRows
	def.GlobalMinimumHeight = -1.0
	def.GlobalMaximumHeight = 1.0
	def.ClockwiseWinding = clockwise
	return def
}

func (g *geometryGoldenGen) heightFieldQueries(name string, hf *HeightFieldData) {
	bounds := hf.AABB
	c := bounds.Center()
	e := bounds.Extents()

	g.printf("%s aabb |", name)
	{
		var xf Transform
		xf.P = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		g.pbox(ComputeHeightFieldAABB(hf, xf))
	}
	g.printf("\n")

	g.printf("%s triangles |", name)
	{
		materials := GetHeightFieldMaterialIndices(hf)
		triangleCount := getHeightFieldTriangleCount(hf)
		for i := 0; i < triangleCount; i += 1 + triangleCount/9 {
			if materials[i>>1] == HeightFieldHole {
				g.printf(" hole")
				continue
			}
			g.ptri(getHeightFieldTriangle(hf, i))
			g.printf(" %d", getHeightFieldMaterial(hf, i))
		}
	}
	g.printf("\n")

	for k := range 12 {
		g.printf("%s ray %d |", name, k)
		var ray RayCastInput
		ray.Origin = g.rv3(1.2*e.X, mulAdd(1.0, 1.5, e.Y), 1.2*e.Z, c.X, c.Y, c.Z)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		ray.Translation = MulSV(2.0, Sub(target, ray.Origin))
		ray.MaxFraction = 1.0
		if k%4 == 3 {
			ray.MaxFraction = 0.4
		}
		g.pco(RayCastHeightField(hf, &ray))
		g.printf("\n")
	}

	for k := range 10 {
		g.printf("%s cast %d |", name, k)
		start := g.rv3(1.2*e.X, mulAdd(1.0, 1.5, e.Y), 1.2*e.Z, c.X, c.Y, c.Z)
		var input ShapeCastInput
		input.Proxy = g.rproxy(start, 0.3)
		target := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		input.Translation = MulSV(2.0, Sub(target, start))
		input.MaxFraction = 1.0
		input.CanEncroach = k%3 == 1
		g.pco(ShapeCastHeightField(hf, &input))
		g.printf("\n")
	}

	g.printf("%s overlap |", name)
	for range 16 {
		var xf Transform
		xf.P = g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
		xf.Q = g.rq()
		local := g.rv3(e.X, e.Y+0.5, e.Z, c.X, c.Y, c.Z)
		world := TransformPoint(xf, local)
		proxy := g.rproxy(world, 0.4)
		g.pbool(OverlapHeightField(hf, xf, &proxy))
	}
	g.printf("\n")

	for k := range 8 {
		g.printf("%s mover %d |", name, k)
		var mover Capsule
		mover.Center1 = g.rv3(e.X, e.Y+0.3, e.Z, c.X, c.Y, c.Z)
		mover.Center2 = Add(mover.Center1, g.rv3(0.3, 0.3, 0.3, 0.0, 0.4, 0.0))
		mover.Radius = mulAdd(0.4, 0.3, g.rnd())
		var planes [16]PlaneResult
		capacity := 16
		if k == 5 {
			capacity = 2
		}
		count := collideMoverAndHeightField(planes[:capacity], hf, &mover)
		g.pplanes(planes[:], count)
		g.printf("\n")
	}

	g.printf("%s query |", name)
	for range 6 {
		var box AABB
		p := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		h := g.rv3(0.5, 0.5, 0.5, 0.8, 0.8, 0.8)
		box.LowerBound = Sub(p, h)
		box.UpperBound = Add(p, h)
		g.resetQuery(-1)
		QueryHeightField(hf, box, g.meshQueryFcn, nil)
		g.pquery()
	}
	g.printf("\n")
}

func (g *geometryGoldenGen) heightFieldSection(dir string) {
	var fields []*HeightFieldData

	g.printf("hf creators |")
	fields = append(fields, CreateGrid(6, 5, Vec3{1.0, 1.0, 1.0}, false))
	fields = append(fields, CreateGrid(20, 18, Vec3{0.5, 2.0, 0.75}, true))
	fields = append(fields, CreateWave(10, 12, Vec3{2.0, 1.5, 2.0}, 0.1, 0.03333, false))
	fields = append(fields, CreateWave(19, 21, Vec3{1.0, 3.0, 1.5}, 0.07, 0.11, true))
	for _, f := range fields {
		g.phf(f)
	}
	g.printf("\n")

	g.printf("hf random |")
	{
		saved := g.seed
		def := g.randomFieldDef(false)
		fields = append(fields, CreateHeightField(&def))
		g.phf(fields[len(fields)-1])

		g.seed = saved
		def = g.randomFieldDef(true)
		fields = append(fields, CreateHeightField(&def))
		g.phf(fields[len(fields)-1])

		// No material array
		def.MaterialIndices = nil
		def.ClockwiseWinding = false
		g.phf(CreateHeightField(&def))

		// Dump and load: the text format must round trip to the same blob
		def = g.randomFieldDef(true)
		direct := CreateHeightField(&def)
		path := filepath.Join(dir, "geometry_golden_hf.dat")
		DumpHeightData(&def, path)
		loaded := LoadHeightField(path)
		os.Remove(path)
		g.phf(direct)
		g.phf(loaded)
		g.phf(LoadHeightField(filepath.Join(dir, "geometry_golden_missing.dat")))
	}
	g.printf("\n")

	g.printf("hf flags |")
	for _, f := range fields {
		flags := GetHeightFieldFlags(f)
		heights := GetHeightFieldCompressedHeights(f)
		var sum uint32 = 0
		for j := range getHeightFieldTriangleCount(f) {
			sum = 31*sum + uint32(flags[j])
		}
		for j := range int(f.RowCount * f.ColumnCount) {
			sum = 31*sum + uint32(heights[j])
		}
		g.printf(" %08x", sum)
	}
	g.printf("\n")

	for i, f := range fields {
		name := fmt.Sprintf("hf %d", i)
		g.heightFieldQueries(name, f)
	}
}

func (g *geometryGoldenGen) blobHash(data []byte) uint64 {
	g.resetQuery(-1)
	for i := 0; i+4 <= len(data); i += 4 {
		g.hashWord(binary.LittleEndian.Uint32(data[i:]))
	}
	return g.queryHash
}

func (g *geometryGoldenGen) pcompound(c *CompoundData) {
	if c == nil {
		g.printf(" null")
		return
	}
	g.printf(" %d %016x %d %d %d %d %d %d %d %d %d", c.ByteCount, g.blobHash(compoundBytes(c)),
		c.TreeHeight, c.NodeCount, c.MaterialCount, c.CapsuleCount, c.HullCount, c.SharedHullCount, c.MeshCount,
		c.SharedMeshCount, c.SphereCount)
	g.pbox(c.Bounds)
}

func (g *geometryGoldenGen) rmaterial() SurfaceMaterial {
	m := DefaultSurfaceMaterial()
	m.Friction = 0.1 * float32(g.rndInt(3))
	m.UserMaterialId = uint64(g.rndInt(4))
	return m
}

func (g *geometryGoldenGen) rxf(spread float32) Transform {
	var xf Transform
	xf.P = g.rv3(spread, spread, spread, 0.0, 0.0, 0.0)
	xf.Q = g.rq()
	return xf
}

func (g *geometryGoldenGen) compoundQueryFcn(compound *CompoundData, childIndex int, context any) bool {
	g.hashWord(uint32(childIndex))
	g.queryCount += 1
	return g.queryCount != g.queryStop
}

func childPoint(compound *CompoundData, childIndex int) Vec3 {
	child := GetCompoundChild(compound, childIndex)
	switch child.Type {
	case CapsuleShape:
		return Lerp(child.Capsule.Center1, child.Capsule.Center2, 0.5)
	case SphereShape:
		return child.Sphere.Center
	default:
		return child.Transform.P
	}
}

func (g *geometryGoldenGen) compoundQueries(name string, compound *CompoundData) {
	bounds := compound.Bounds
	c := bounds.Center()
	e := bounds.Extents()

	g.printf("%s aabb |", name)
	g.pbox(ComputeCompoundAABB(compound, g.rxf(2.0)))
	g.printf("\n")

	childCount := int(compound.CapsuleCount + compound.HullCount + compound.MeshCount + compound.SphereCount)
	g.printf("%s children |", name)
	for i := range childCount {
		child := GetCompoundChild(compound, i)
		g.printf(" %d", child.Type)
		g.pv(child.Transform.P)
		g.pv(child.Transform.Q.V)
		g.pf(child.Transform.Q.S)
		g.printf(" %d", child.MaterialCount)
		for j := range int(child.MaterialCount) {
			g.printf(" %d", child.MaterialIndices[j])
		}
		switch child.Type {
		case CapsuleShape:
			g.pv(child.Capsule.Center1)
			g.pv(child.Capsule.Center2)
			g.pf(child.Capsule.Radius)
		case HullShape:
			g.printf(" %d %016x", child.Hull.ByteCount, child.Hull.Hash)
		case MeshShape:
			g.printf(" %d %016x", child.Mesh.Data.ByteCount, child.Mesh.Data.Hash)
			g.pv(child.Mesh.Scale)
		default:
			g.pv(child.Sphere.Center)
			g.pf(child.Sphere.Radius)
		}
	}
	g.printf("\n")

	g.printf("%s sweep |", name)
	{
		xf := g.rxf(3.0)
		child := GetCompoundChild(compound, childCount-1)
		sweep := makeCompoundChildSweep(xf, child.Transform)
		g.pv(sweep.C1)
		g.pv(sweep.Q1.V)
		g.pf(sweep.Q1.S)
	}
	g.printf("\n")

	for k := range 12 {
		g.printf("%s ray %d |", name, k)
		var ray RayCastInput
		ray.Origin = g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		aim := childPoint(compound, int(g.rndInt(uint32(childCount))))
		target := g.rv3(0.3, 0.3, 0.3, aim.X, aim.Y, aim.Z)
		ray.Translation = MulSV(2.0, Sub(target, ray.Origin))
		ray.MaxFraction = 1.0
		if k%4 == 3 {
			ray.MaxFraction = 0.4
		}
		g.pco(RayCastCompound(compound, &ray))
		g.printf("\n")
	}

	for k := range 10 {
		g.printf("%s cast %d |", name, k)
		start := g.rv3(1.5*e.X, mulAdd(1.0, 1.5, e.Y), 1.5*e.Z, c.X, c.Y, c.Z)
		var input ShapeCastInput
		input.Proxy = g.rproxy(start, 0.3)
		aim := childPoint(compound, int(g.rndInt(uint32(childCount))))
		target := g.rv3(0.3, 0.3, 0.3, aim.X, aim.Y, aim.Z)
		input.Translation = MulSV(2.0, Sub(target, start))
		input.MaxFraction = 1.0
		input.CanEncroach = k%3 == 1
		g.pco(ShapeCastCompound(compound, &input))
		g.printf("\n")
	}

	g.printf("%s overlap |", name)
	for range 16 {
		xf := g.rxf(0.5)
		aim := childPoint(compound, int(g.rndInt(uint32(childCount))))
		local := g.rv3(0.8, 0.8, 0.8, aim.X, aim.Y, aim.Z)
		world := TransformPoint(xf, local)
		proxy := g.rproxy(world, 0.4)
		g.pbool(OverlapCompound(compound, xf, &proxy))
	}
	g.printf("\n")

	for k := range 8 {
		g.printf("%s mover %d |", name, k)
		var mover Capsule
		aim := childPoint(compound, int(g.rndInt(uint32(childCount))))
		mover.Center1 = g.rv3(0.8, 0.8, 0.8, aim.X, aim.Y, aim.Z)
		mover.Center2 = Add(mover.Center1, g.rv3(0.3, 0.3, 0.3, 0.0, 0.4, 0.0))
		mover.Radius = mulAdd(0.4, 0.3, g.rnd())
		var planes [16]PlaneResult
		capacity := 16
		if k == 5 {
			capacity = 2
		}
		count := collideMoverAndCompound(planes[:capacity], compound, &mover)
		g.pplanes(planes[:], count)
		g.printf("\n")
	}

	g.printf("%s query |", name)
	for k := range 6 {
		var box AABB
		p := g.rv3(e.X, e.Y, e.Z, c.X, c.Y, c.Z)
		h := g.rv3(0.5, 0.5, 0.5, 1.5, 1.5, 1.5)
		box.LowerBound = Sub(p, h)
		box.UpperBound = Add(p, h)
		stop := -1
		if k == 4 {
			stop = 2
		}
		g.resetQuery(stop)
		QueryCompound(compound, box, g.compoundQueryFcn, nil)
		g.pquery()
	}
	g.printf("\n")
}

const (
	maxCapsules = 6 // MAX_CAPSULES
	maxHulls    = 6 // MAX_HULLS
	maxMeshes   = 4 // MAX_MESHES
	maxSpheres  = 6 // MAX_SPHERES
)

func (g *geometryGoldenGen) compoundSection() {
	var boxes [3]BoxHull
	boxes[0] = MakeBoxHull(0.5, 0.5, 0.5)
	boxes[1] = MakeBoxHull(0.5, 0.5, 0.5)
	boxes[2] = MakeBoxHull(1.0, 0.25, 0.75)
	cylinder := CreateCylinder(1.0, 0.5, 0.0, 10)

	var meshes [3]*MeshData
	meshes[0] = CreateWaveMesh(4, 4, 0.5, 0.3, 0.2, 0.3)
	meshes[1] = CreateBoxMesh(Vec3Zero, Vec3{0.5, 0.5, 0.5}, true)
	meshes[2] = CreateGridMesh(3, 3, 0.5, 3, true)

	var compounds []*CompoundData

	g.printf("compound create |")
	for n := range 4 {
		var capsules [maxCapsules]CompoundCapsuleDef
		var hulls [maxHulls]CompoundHullDef
		var meshDefs [maxMeshes]CompoundMeshDef
		var meshMaterials [maxMeshes][4]SurfaceMaterial
		var spheres [maxSpheres]CompoundSphereDef

		def := CompoundDef{}
		def.Capsules = capsules[:]
		if n != 3 {
			def.CapsuleCount = 1 + int32(g.rndInt(maxCapsules))
		}
		def.Hulls = hulls[:]
		if n != 1 {
			def.HullCount = 1 + int32(g.rndInt(maxHulls))
		}
		def.Meshes = meshDefs[:]
		if n != 2 {
			def.MeshCount = 1 + int32(g.rndInt(maxMeshes))
		}
		def.Spheres = spheres[:]
		def.SphereCount = 1 + int32(g.rndInt(maxSpheres))

		for i := range int(def.CapsuleCount) {
			capsules[i].Capsule.Center1 = g.rv3(4.0, 4.0, 4.0, 0.0, 0.0, 0.0)
			capsules[i].Capsule.Center2 = Add(capsules[i].Capsule.Center1, g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
			capsules[i].Capsule.Radius = mulAdd(0.3, 0.2, g.rnd())
			capsules[i].Material = g.rmaterial()
		}
		for i := range int(def.HullCount) {
			which := g.rndInt(4)
			if which == 3 {
				hulls[i].Hull = cylinder
			} else {
				hulls[i].Hull = &boxes[which].Base
			}
			hulls[i].Transform = g.rxf(4.0)
			hulls[i].Material = g.rmaterial()
		}
		for i := range int(def.MeshCount) {
			md := meshes[g.rndInt(3)]
			meshDefs[i].MeshData = md
			meshDefs[i].Transform = g.rxf(4.0)
			mirror := g.rndInt(3)
			meshDefs[i].Scale = g.rv3(0.3, 0.3, 0.3, 1.0, 1.0, 1.0)
			if mirror == 0 {
				meshDefs[i].Scale.X = -meshDefs[i].Scale.X
			}
			for j := range int(md.MaterialCount) {
				meshMaterials[i][j] = g.rmaterial()
			}
			meshDefs[i].Materials = meshMaterials[i][:]
			meshDefs[i].MaterialCount = md.MaterialCount
		}
		for i := range int(def.SphereCount) {
			spheres[i].Sphere.Center = g.rv3(4.0, 4.0, 4.0, 0.0, 0.0, 0.0)
			spheres[i].Sphere.Radius = mulAdd(0.4, 0.3, g.rnd())
			spheres[i].Material = g.rmaterial()
		}

		compounds = append(compounds, CreateCompound(&def))
		g.pcompound(compounds[len(compounds)-1])
	}
	g.printf("\n")

	g.printf("compound validate |")
	for _, compound := range compounds {
		buffer := make([]byte, 8*4096)
		byteCount := int(compound.ByteCount)
		copy(buffer, compoundBytes(compound))
		cp := ValidateCompound(buffer, byteCount)
		g.pbool(cp != nil)
		g.pbool(ValidateCompound(buffer, byteCount-8) != nil)
		if cp != nil {
			var ray RayCastInput
			ray.Origin = Add(childPoint(cp, 0), Vec3{-10.0, 0.1, 0.2})
			ray.Translation = Vec3{20.0, 0.0, 0.0}
			ray.MaxFraction = 1.0
			g.pco(RayCastCompound(cp, &ray))
		}
	}
	g.printf("\n")

	for i, compound := range compounds {
		name := fmt.Sprintf("compound %d", i)
		g.compoundQueries(name, compound)
	}

	for _, compound := range compounds {
		DestroyCompound(compound)
	}
	for i := range 3 {
		DestroyMesh(meshes[i])
	}
	DestroyHull(cylinder)
}

func (g *geometryGoldenGen) run(dir string) string {
	g.seed = 31337

	g.meshSection()
	g.heightFieldSection(dir)
	g.compoundSection()

	return g.sb.String()
}

var _ = Describe("GeometryGoldenTest", func() {
	It("MatchesC", func() {
		SetLogFcn(func(string) {})
		defer SetLogFcn(defaultLogFcn)

		want, err := os.ReadFile("testdata/golem_hull_topology/geometry_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g geometryGoldenGen
		got := g.run(GinkgoT().TempDir())

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		for i := range min(len(wantLines), len(gotLines)) {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
		Expect(gotLines).To(HaveLen(len(wantLines)))
	})
})
