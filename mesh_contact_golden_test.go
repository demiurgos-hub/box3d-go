// Go-only test: no upstream counterpart (mesh_contact.c is exercised upstream only through world
// tests). Runs b3ComputeMeshManifolds on a hand assembled world against meshes (scaled, mirrored,
// multi material with and without a material map, flat regions, a dense mesh that fills the
// triangle buffer) and height fields (both windings, holes, materials) with sphere, capsule and
// hull shapes moving and turning over them, plus b3CullPoints and b3ReduceCluster directly, and
// compares every manifold, the mixed material values, the query bounds and the triangle caches
// against the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with
// -ffp-contract=off. See testdata/mesh_contact_golden_gen.c; this file mirrors it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	mcGoldenMaterialCount = 5
	mcGoldenGridMax       = 40
	mcGoldenHFColumns     = 9
	mcGoldenHFRows        = 7
)

var mcGoldenMaterialMap = []uint16{4, 3, 2, 1, 0, 2, 4, 1}

type mcGoldenGen struct {
	seed      uint32
	sb        strings.Builder
	materials [mcGoldenMaterialCount]SurfaceMaterial
	world     world
}

func (g *mcGoldenGen) rnd() float32 {
	g.seed ^= g.seed << 13
	g.seed ^= g.seed >> 17
	g.seed ^= g.seed << 5
	return float32(g.seed%20001)/10000.0 - 1.0
}

func (g *mcGoldenGen) rndInt(n uint32) uint32 {
	g.seed ^= g.seed << 13
	g.seed ^= g.seed >> 17
	g.seed ^= g.seed << 5
	return g.seed % n
}

func (g *mcGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }

func (g *mcGoldenGen) pf(f float32) { g.printf(" %08x", math.Float32bits(f)) }

func (g *mcGoldenGen) pv(v Vec3) {
	g.pf(v.X)
	g.pf(v.Y)
	g.pf(v.Z)
}

func mcGoldenMixFriction(frictionA float32, userMaterialIdA uint64, frictionB float32, userMaterialIdB uint64) float32 {
	f := frictionB
	if frictionA < frictionB {
		f = frictionA
	}
	return f + float32(userMaterialIdA+2*userMaterialIdB)
}

func mcGoldenMixRestitution(restitutionA float32, userMaterialIdA uint64, restitutionB float32, userMaterialIdB uint64) float32 {
	if restitutionA > restitutionB {
		return restitutionA
	}
	return restitutionB
}

func (g *mcGoldenGen) setupMaterials() {
	friction := [mcGoldenMaterialCount]float32{0.2, 0.5, 0.8, 0.3, 0.6}
	restitution := [mcGoldenMaterialCount]float32{0.0, 0.1, 0.2, 0.3, 0.4}
	rolling := [mcGoldenMaterialCount]float32{0.01, 0.02, 0.0, 0.05, 0.03}
	for i := range mcGoldenMaterialCount {
		g.materials[i] = SurfaceMaterial{}
		g.materials[i].Friction = friction[i]
		g.materials[i].Restitution = restitution[i]
		g.materials[i].RollingResistance = rolling[i]
		g.materials[i].TangentVelocity = Vec3{float32(0.25 * float32(i)), 0.0, -0.5}
		g.materials[i].UserMaterialId = uint64(10 + i)
	}
}

func mcGoldenMaterialB() SurfaceMaterial {
	m := SurfaceMaterial{}
	m.Friction = 0.4
	m.Restitution = 0.05
	m.RollingResistance = 0.02
	m.TangentVelocity = Vec3{0.1, 0.0, 0.2}
	m.UserMaterialId = 7
	return m
}

func (g *mcGoldenGen) setupWorld() {
	g.world = world{}
	arrayPush(&g.world.taskContexts, taskContext{})
	g.world.manifoldAllocatorMutex = createMutex()
	g.world.frictionCallback = mcGoldenMixFriction
	g.world.restitutionCallback = mcGoldenMixRestitution
}

// makeGrid mirrors the C indexed grid mesh with welded vertices and identified edges.
func (g *mcGoldenGen) makeGrid(n int, cell, amplitude float32, flatEvery int) *MeshData {
	vertices := make([]Vec3, (mcGoldenGridMax+1)*(mcGoldenGridMax+1))
	indices := make([]int32, mcGoldenGridMax*mcGoldenGridMax*6)
	materials := make([]uint8, mcGoldenGridMax*mcGoldenGridMax*2)

	v := 0
	for ix := 0; ix <= n; ix++ {
		for iz := 0; iz <= n; iz++ {
			h := float32(amplitude * g.rnd())
			if flatEvery > 0 && (ix%flatEvery) == 0 {
				h = 0.0
			}
			var p Vec3
			p.X = float32(cell * float32(ix))
			p.Y = h
			p.Z = float32(cell * float32(iz))
			vertices[v] = p
			v++
		}
	}

	k := 0
	t := 0
	for ix := 0; ix < n; ix++ {
		for iz := 0; iz < n; iz++ {
			i11 := int32(ix*(n+1) + iz)
			i12 := i11 + 1
			i21 := i11 + int32(n+1)
			i22 := i21 + 1
			indices[k+0] = i11
			indices[k+1] = i12
			indices[k+2] = i22
			indices[k+3] = i22
			indices[k+4] = i21
			indices[k+5] = i11
			k += 6
			materials[t] = uint8(g.rndInt(8))
			t++
			materials[t] = uint8(g.rndInt(8))
			t++
		}
	}

	def := MeshDef{}
	def.Vertices = vertices
	def.Indices = indices
	def.MaterialIndices = materials
	def.VertexCount = int32(v)
	def.TriangleCount = int32(t)
	def.WeldVertices = true
	def.WeldTolerance = 0.001
	def.IdentifyEdges = true
	return CreateMesh(&def, nil)
}

func (g *mcGoldenGen) makeField(clockwise bool) *HeightFieldData {
	heights := make([]float32, mcGoldenHFColumns*mcGoldenHFRows)
	materials := make([]uint8, (mcGoldenHFColumns-1)*(mcGoldenHFRows-1))
	for i := range heights {
		heights[i] = float32(0.3 * g.rnd())
	}
	for i := range materials {
		r := g.rndInt(10)
		if r == 0 {
			materials[i] = HeightFieldHole
		} else {
			materials[i] = uint8(r)
		}
	}

	def := HeightFieldDef{}
	def.Heights = heights
	def.MaterialIndices = materials
	def.Scale = Vec3{0.75, 2.0, 1.25}
	def.CountX = mcGoldenHFColumns
	def.CountZ = mcGoldenHFRows
	def.GlobalMinimumHeight = -1.0
	def.GlobalMaximumHeight = 1.0
	def.ClockwiseWinding = clockwise
	return CreateHeightField(&def)
}

func mcGoldenComputeShapeAABB(s *shape, xf WorldTransform) {
	t := ToRelativeTransform(xf, PosZero)
	switch s.typ {
	case SphereShape:
		s.aabb = ComputeSphereAABB(&s.sphere, t)
	case CapsuleShape:
		s.aabb = ComputeCapsuleAABB(&s.capsule, t)
	case HullShape:
		s.aabb = ComputeHullAABB(s.hull, t)
	}
}

// printContact mirrors the C printContact. b3ContactCache is a union upstream: hulls use the SAT
// cache, capsules the simplex cache, spheres neither.
func (g *mcGoldenGen) printContact(name string, frame int, touching bool, c *contact, typeB ShapeType) {
	t := 0
	if touching {
		t = 1
	}
	g.printf("%s f%d | %d %d", name, frame, t, c.manifoldCount)
	for i := 0; i < int(c.manifoldCount); i++ {
		m := &c.manifolds[i]
		g.printf(" m")
		g.pv(m.Normal)
		g.pf(m.TwistImpulse)
		g.pv(m.FrictionImpulse)
		g.pv(m.RollingImpulse)
		g.printf(" %d", m.PointCount)
		for j := 0; j < int(m.PointCount); j++ {
			p := &m.Points[j]
			g.printf(" p")
			g.pv(p.AnchorA)
			g.pv(p.AnchorB)
			g.pf(p.Separation)
			g.pf(p.NormalImpulse)
			persisted := 0
			if p.Persisted {
				persisted = 1
			}
			g.printf(" %08x %d %d", p.FeatureId, p.TriangleIndex, persisted)
		}
	}
	g.printf(" | c")
	g.pf(c.friction)
	g.pf(c.restitution)
	g.pf(c.rollingResistance)
	g.pv(c.tangentVelocity)
	g.printf(" | q")
	g.pv(c.meshContact.queryBounds.LowerBound)
	g.pv(c.meshContact.queryBounds.UpperBound)
	g.printf(" | t %d", len(c.meshContact.triangleCache))
	for i := range c.meshContact.triangleCache {
		tc := &c.meshContact.triangleCache[i]
		sat := &tc.cache.satCache
		sc := &tc.cache.simplexCache
		g.printf(" %d", tc.triangleIndex)
		if typeB == HullShape {
			g.printf(":s%08x,%d,%d,%d,%d", math.Float32bits(sat.Separation), sat.Type, sat.IndexA, sat.IndexB, sat.Hit)
		}
		if typeB == CapsuleShape {
			g.printf(":g%08x,%d", math.Float32bits(sc.Metric), sc.Count)
			for k := 0; k < int(sc.Count); k++ {
				g.printf(",%d,%d", sc.IndexA[k], sc.IndexB[k])
			}
		}
	}
	g.printf("\n")
}

// stampImpulses overwrites the impulses with known values so the next frame shows what matching
// preserved.
func mcGoldenStampImpulses(c *contact, frame int) {
	for i := 0; i < int(c.manifoldCount); i++ {
		m := &c.manifolds[i]
		base := float32(100*frame + 10*i)
		m.TwistImpulse = base + 1.0
		m.FrictionImpulse = Vec3{base + 2.0, base + 3.0, base + 4.0}
		m.RollingImpulse = Vec3{base + 5.0, base + 6.0, base + 7.0}
		for j := 0; j < int(m.PointCount); j++ {
			m.Points[j].NormalImpulse = mulAdd(base, float32(j+1), 0.25)
		}
	}
}

type mcGoldenScenario struct {
	name        string
	shapeA      *shape
	materialMap []uint16
	shapeB      *shape
	speculative bool
	frameCount  int
	start       Vec3
	velocity    Vec3
	rotation    Quat
	spin        Vec3
}

func (g *mcGoldenGen) runScenario(s *mcGoldenScenario, xfA WorldTransform) {
	c := &contact{}
	arrayCreate(&c.meshContact.triangleCache)
	if s.speculative {
		c.flags = enableSpeculativePoints
	}

	arena := createArena(1 << 20)

	for frame := 0; frame < s.frameCount; frame++ {
		local := MulAdd(s.start, float32(frame), s.velocity)
		var qLocal Quat
		qLocal.V = MulAdd(s.rotation.V, float32(frame), s.spin)
		qLocal.S = s.rotation.S
		qLocal = NormalizeQuat(qLocal)

		var xfB WorldTransform
		xfB.P = OffsetPos(xfA.P, RotateVector(xfA.Q, local))
		xfB.Q = MulQuat(xfA.Q, qLocal)
		mcGoldenComputeShapeAABB(s.shapeB, xfB)

		isFast := (frame % 3) == 2
		touching := computeMeshManifolds(&g.world, 0, c, s.shapeA, s.materialMap, xfA, s.shapeB, xfB, isFast, arena)
		g.printContact(s.name, frame, touching, c, s.shapeB.typ)
		mcGoldenStampImpulses(c, frame)
	}

	context := &g.world.taskContexts[0]
	g.printf("%s sat | %d %d\n", s.name, context.satCallCount, context.satCacheHitCount)
	context.satCallCount = 0
	context.satCacheHitCount = 0

	freeManifolds(&g.world, c.manifolds, int(c.manifoldCount))
	arrayDestroy(&c.meshContact.triangleCache)
	destroyArena(&arena)
}

func (g *mcGoldenGen) cullSection() {
	var points [16]Point2D
	for trial := 0; trial < 60; trial++ {
		count := 1 + int(g.rndInt(12))
		mode := trial % 4
		for i := 0; i < count; i++ {
			x := g.rnd()
			y := g.rnd()
			if mode == 1 {
				// collinear
				y = x
			} else if mode == 2 {
				// within the tolerance
				x = float32(0.0001 * x)
				y = float32(0.0001 * y)
			} else if mode == 3 && i > 0 {
				// duplicates
				x = points[i-1].P.X
				y = points[i-1].P.Y
			}
			points[i].P = Vec2{x, y}
			points[i].Separation = float32(0.01 * g.rnd())
			points[i].OriginalIndex = int32(i)
		}

		result := cullPoints(points[:], count)
		g.printf("cull %d | %d %d", trial, count, result)
		for i := 0; i < result; i++ {
			g.pf(points[i].P.X)
			g.pf(points[i].P.Y)
			g.pf(points[i].Separation)
			g.printf(" %d", points[i].OriginalIndex)
		}
		g.printf("\n")
	}
}

func (g *mcGoldenGen) reduceSection() {
	arena := createArena(1 << 16)
	var points [24]LocalManifoldPoint
	for trial := 0; trial < 30; trial++ {
		count := 1 + int(g.rndInt(20))
		var normal Vec3
		normal.X = g.rnd()
		normal.Y = g.rnd() + 2.0
		normal.Z = g.rnd()
		normal = Normalize(normal)
		for i := 0; i < count; i++ {
			points[i] = LocalManifoldPoint{}
			points[i].Point.X = g.rnd()
			points[i].Point.Y = float32(0.1 * g.rnd())
			points[i].Point.Z = g.rnd()
			points[i].Separation = float32(0.02 * g.rnd())
			points[i].Pair.Index1 = uint16(i)
			points[i].TriangleIndex = int32(g.rndInt(100))
		}

		result := reduceCluster(points[:], count, normal, arena)
		g.printf("reduce %d | %d %d", trial, count, result)
		for i := 0; i < result; i++ {
			g.pv(points[i].Point)
			g.pf(points[i].Separation)
			g.printf(" %d %d", points[i].Pair.Index1, points[i].TriangleIndex)
		}
		g.printf("\n")
	}
	destroyArena(&arena)
}

func (g *mcGoldenGen) run() string {
	g.seed = 4242
	g.setupMaterials()
	g.setupWorld()

	g.cullSection()
	g.reduceSection()

	grid := g.makeGrid(8, 1.0, 0.4, 0)
	flatGrid := g.makeGrid(8, 1.0, 0.3, 2)
	denseGrid := g.makeGrid(mcGoldenGridMax, 0.25, 0.05, 0)
	field := g.makeField(false)
	fieldCW := g.makeField(true)

	box := new(BoxHull)
	*box = MakeBoxHull(0.5, 0.3, 0.4)
	offsetBox := new(BoxHull)
	*offsetBox = MakeOffsetBoxHull(0.3, 0.2, 0.6, Vec3{0.1, 0.05, -0.1})

	mB := mcGoldenMaterialB()

	sphere := &shape{}
	sphere.typ = SphereShape
	sphere.sphere = Sphere{Vec3{0.0, 0.0, 0.0}, 0.45}
	sphere.material = mB

	bigSphere := new(shape)
	*bigSphere = *sphere
	bigSphere.sphere.Radius = 3.5

	capsule := &shape{}
	capsule.typ = CapsuleShape
	capsule.capsule = Capsule{Vec3{-0.4, 0.0, 0.1}, Vec3{0.4, 0.05, -0.1}, 0.25}
	capsule.material = mB

	hull := &shape{}
	hull.typ = HullShape
	hull.hull = &box.Base
	hull.material = mB

	offsetHull := new(shape)
	*offsetHull = *hull
	offsetHull.hull = &offsetBox.Base

	// Mesh shapes. Multi material shapes use the shared material array.
	meshMulti := &shape{}
	meshMulti.typ = MeshShape
	meshMulti.mesh = Mesh{grid, Vec3{1.0, 1.0, 1.0}}
	meshMulti.materials = g.materials[:]
	meshMulti.materialCount = mcGoldenMaterialCount

	meshFlipped := &shape{}
	meshFlipped.typ = MeshShape
	meshFlipped.mesh = Mesh{grid, Vec3{1.5, 0.8, -1.2}}
	meshFlipped.material = g.materials[2]

	meshScaled := new(shape)
	*meshScaled = *meshMulti
	meshScaled.mesh.Scale = Vec3{1.25, 1.0, 1.25}

	meshFlat := &shape{}
	meshFlat.typ = MeshShape
	meshFlat.mesh = Mesh{flatGrid, Vec3{-1.0, 1.0, 1.0}}
	meshFlat.material = g.materials[1]

	meshDense := new(shape)
	*meshDense = *meshMulti
	meshDense.mesh = Mesh{denseGrid, Vec3{1.0, 1.0, 1.0}}

	hf := &shape{}
	hf.typ = HeightShape
	hf.heightField = field
	hf.materials = g.materials[:]
	hf.materialCount = mcGoldenMaterialCount

	hfCW := &shape{}
	hfCW.typ = HeightShape
	hfCW.heightField = fieldCW
	hfCW.material = g.materials[3]

	var xfA WorldTransform
	xfA.P = Pos{0.25, -0.5, 0.125}
	xfA.Q = NormalizeQuat(Quat{Vec3{0.05, 0.1, -0.03}, 1.0})

	q0 := NormalizeQuat(Quat{Vec3{0.1, 0.2, 0.05}, 1.0})
	qTilt := NormalizeQuat(Quat{Vec3{0.3, -0.1, 0.2}, 1.0})
	spin := Vec3{0.02, 0.05, -0.03}
	noSpin := Vec3{0.0, 0.0, 0.0}

	scenarios := []mcGoldenScenario{
		{"mesh-sphere", meshMulti, nil, sphere, true, 16, Vec3{0.5, 0.55, 0.7}, Vec3{0.41, -0.03, 0.37}, q0, noSpin},
		{"mesh-capsule", meshFlipped, nil, capsule, true, 16, Vec3{1.0, 0.5, -1.0}, Vec3{0.5, -0.02, -0.45}, q0, spin},
		{"mesh-hull", meshScaled, mcGoldenMaterialMap, hull, true, 16, Vec3{1.2, 0.6, 1.1}, Vec3{0.43, -0.025, 0.38}, qTilt, spin},
		{"mesh-hull-nospec", meshFlipped, nil, offsetHull, false, 16, Vec3{1.0, 0.45, -1.5}, Vec3{0.55, -0.01, -0.4}, q0, spin},
		{"flat-sphere", meshFlat, nil, sphere, true, 16, Vec3{-0.5, 0.4, 0.3}, Vec3{-0.45, -0.005, 0.42}, q0, noSpin},
		{"flat-hull", meshFlat, nil, hull, true, 16, Vec3{-0.6, 0.32, 0.5}, Vec3{-0.4, -0.002, 0.4}, QuatIdentity, noSpin},
		{"hf-sphere", hf, nil, sphere, true, 16, Vec3{0.4, 0.5, 0.6}, Vec3{0.35, -0.03, 0.45}, q0, noSpin},
		{"hf-capsule", hf, nil, capsule, true, 16, Vec3{0.6, 0.4, 0.5}, Vec3{0.33, -0.015, 0.42}, q0, spin},
		{"hf-hull", hf, nil, hull, true, 16, Vec3{0.5, 0.55, 0.4}, Vec3{0.36, -0.03, 0.47}, qTilt, spin},
		{"hfcw-hull", hfCW, nil, offsetHull, false, 16, Vec3{0.7, 0.45, 0.6}, Vec3{0.3, -0.02, 0.4}, q0, spin},
		{"hfcw-sphere", hfCW, nil, sphere, true, 16, Vec3{0.8, 0.4, 0.7}, Vec3{0.31, -0.02, 0.41}, q0, noSpin},
		{"dense-sphere", meshDense, nil, bigSphere, true, 4, Vec3{5.0, 0.5, 5.0}, Vec3{0.1, -0.02, 0.1}, q0, noSpin},
		{"dense-hull", meshDense, mcGoldenMaterialMap, hull, true, 8, Vec3{5.0, 0.3, 5.0}, Vec3{0.13, -0.01, 0.11}, qTilt, spin},
	}

	for i := range scenarios {
		g.runScenario(&scenarios[i], xfA)
	}

	DestroyMesh(grid)
	DestroyMesh(flatGrid)
	DestroyMesh(denseGrid)
	DestroyHeightField(field)
	DestroyHeightField(fieldCW)
	return g.sb.String()
}

var _ = Describe("MeshContactGoldenTest", func() {
	It("MatchesC", func() {
		SetLogFcn(func(string) {})
		defer SetLogFcn(defaultLogFcn)

		want, err := os.ReadFile("testdata/mesh_contact_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g mcGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		for i := range min(len(wantLines), len(gotLines)) {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
		Expect(gotLines).To(HaveLen(len(wantLines)))
	})
})
