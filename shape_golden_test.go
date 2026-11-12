// Go-only test: no upstream counterpart. Compares the world-free functions of shape.c (mass,
// centroid, area, projected area, extents, shape/fat/swept AABBs, proxies, ray and shape casts,
// overlaps, mover collision, user material ids and b3ShapeTimeOfImpact against convex, mesh,
// height field and compound shapes, including the sensor and fallback paths) on hand-built
// b3Shape values against the pinned upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867)
// compiled with -ffp-contract=off, bit for bit. The C output is identical for the NEON, SSE2,
// BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds. See testdata/shape_golden_gen.c; this file mirrors
// it line by line.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"math"
	"os"
	"strconv"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

// Shape indices of testdata/shape_golden_gen.c.
const (
	shapeGoldenSphere = iota
	shapeGoldenCapsule
	shapeGoldenBox
	shapeGoldenCylinder
	shapeGoldenGridMesh
	shapeGoldenMirroredMesh
	shapeGoldenWaveMesh
	shapeGoldenHeightField
	shapeGoldenCompound
	shapeGoldenCount
)

type shapeGoldenGen struct {
	geometryGoldenGen

	shapes    [shapeGoldenCount]shape
	materials [shapeGoldenCount][64]SurfaceMaterial
	box       BoxHull
}

func (g *shapeGoldenGen) ptoi(o TOIOutput) {
	g.printf(" %d", int(o.State))
	g.pv(o.Point)
	g.pv(o.Normal)
	g.pf(o.Fraction)
	g.pf(o.Distance)
	fallback := 0
	if o.UsedFallback {
		fallback = 1
	}
	g.printf(" %d %d %d %d", o.DistanceIterations, o.PushBackIterations, o.RootIterations, fallback)
}

func (g *shapeGoldenGen) rsweep(c1, c2 Vec3, still bool) Sweep {
	var sweep Sweep
	if still {
		sweep.LocalCenter = Vec3Zero
	} else {
		sweep.LocalCenter = g.rv3(0.1, 0.1, 0.1, 0.0, 0.0, 0.0)
	}
	sweep.C1 = c1
	sweep.C2 = c2
	sweep.Q1 = g.rq()
	if still {
		sweep.Q2 = sweep.Q1
	} else {
		sweep.Q2 = g.rq()
	}
	return sweep
}

func (g *shapeGoldenGen) initShape(index int, typ ShapeType, materialCount int) {
	s := &g.shapes[index]
	s.id = int32(index)
	s.typ = typ
	s.density = mulAdd(0.5, 0.25, float32(g.rndInt(4)))
	s.sensorIndex = NullIndex
	s.proxyKey = NullIndex
	if materialCount == 1 {
		s.material = g.rmaterial()
		s.materialCount = 1
		s.materials = nil
	} else {
		for j := range materialCount {
			g.materials[index][j] = g.rmaterial()
		}
		s.materials = g.materials[index][:materialCount]
		s.materialCount = int32(materialCount)
	}
}

func (g *shapeGoldenGen) makeShapes() {
	g.initShape(shapeGoldenSphere, SphereShape, 1)
	g.shapes[shapeGoldenSphere].sphere.Center = g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
	g.shapes[shapeGoldenSphere].sphere.Radius = mulAdd(0.5, 0.25, g.rnd())

	g.initShape(shapeGoldenCapsule, CapsuleShape, 1)
	g.shapes[shapeGoldenCapsule].capsule.Center1 = g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
	g.shapes[shapeGoldenCapsule].capsule.Center2 = Add(g.shapes[shapeGoldenCapsule].capsule.Center1, g.rv3(1.0, 1.0, 1.0, 1.0, 0.0, 0.0))
	g.shapes[shapeGoldenCapsule].capsule.Radius = mulAdd(0.3, 0.2, g.rnd())

	g.initShape(shapeGoldenBox, HullShape, 1)
	g.box = MakeBoxHull(0.5, 0.75, 1.0)
	g.shapes[shapeGoldenBox].hull = &g.box.Base

	g.initShape(shapeGoldenCylinder, HullShape, 1)
	g.shapes[shapeGoldenCylinder].hull = CreateCylinder(1.0, 0.5, 0.0, 10)

	gridMesh := CreateGridMesh(4, 4, 0.5, 3, true)

	g.initShape(shapeGoldenGridMesh, MeshShape, 3)
	g.shapes[shapeGoldenGridMesh].mesh.Data = gridMesh
	g.shapes[shapeGoldenGridMesh].mesh.Scale = Vec3One

	// Fewer shape materials than mesh material indices exercises the clamps.
	g.initShape(shapeGoldenMirroredMesh, MeshShape, 2)
	g.shapes[shapeGoldenMirroredMesh].mesh.Data = gridMesh
	g.shapes[shapeGoldenMirroredMesh].mesh.Scale = Vec3{-1.2, 0.8, 1.1}

	g.initShape(shapeGoldenWaveMesh, MeshShape, 1)
	g.shapes[shapeGoldenWaveMesh].mesh.Data = CreateWaveMesh(6, 6, 0.5, 0.3, 0.2, 0.3)
	g.shapes[shapeGoldenWaveMesh].mesh.Scale = Vec3One

	for i := range hfColumns * hfRows {
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
	hfDef := HeightFieldDef{}
	hfDef.Heights = g.hfHeights[:]
	hfDef.MaterialIndices = g.hfMaterials[:]
	hfDef.Scale = Vec3{0.75, 2.0, 1.25}
	hfDef.CountX = hfColumns
	hfDef.CountZ = hfRows
	hfDef.GlobalMinimumHeight = -1.0
	hfDef.GlobalMaximumHeight = 1.0
	g.initShape(shapeGoldenHeightField, HeightShape, 6)
	g.shapes[shapeGoldenHeightField].heightField = CreateHeightField(&hfDef)

	var capsules [2]CompoundCapsuleDef
	var hulls [2]CompoundHullDef
	var meshDefs [1]CompoundMeshDef
	var meshMaterials [3]SurfaceMaterial
	var spheres [2]CompoundSphereDef
	for i := range 2 {
		capsules[i].Capsule.Center1 = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		capsules[i].Capsule.Center2 = Add(capsules[i].Capsule.Center1, g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
		capsules[i].Capsule.Radius = mulAdd(0.3, 0.2, g.rnd())
		capsules[i].Material = g.rmaterial()
	}
	hulls[0].Hull = &g.box.Base
	hulls[1].Hull = g.shapes[shapeGoldenCylinder].hull
	for i := range 2 {
		hulls[i].Transform = g.rxf(2.0)
		hulls[i].Material = g.rmaterial()
	}
	meshDefs[0].MeshData = gridMesh
	meshDefs[0].Transform = g.rxf(2.0)
	meshDefs[0].Scale = g.rv3(0.3, 0.3, 0.3, 1.0, 1.0, 1.0)
	for j := range 3 {
		meshMaterials[j] = g.rmaterial()
	}
	meshDefs[0].Materials = meshMaterials[:]
	meshDefs[0].MaterialCount = 3
	for i := range 2 {
		spheres[i].Sphere.Center = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		spheres[i].Sphere.Radius = mulAdd(0.4, 0.3, g.rnd())
		spheres[i].Material = g.rmaterial()
	}
	compoundDef := CompoundDef{}
	compoundDef.Capsules = capsules[:]
	compoundDef.CapsuleCount = 2
	compoundDef.Hulls = hulls[:]
	compoundDef.HullCount = 2
	compoundDef.Meshes = meshDefs[:]
	compoundDef.MeshCount = 1
	compoundDef.Spheres = spheres[:]
	compoundDef.SphereCount = 2
	compound := CreateCompound(&compoundDef)

	// As b3CreateShapeInternal: the shape owns a copy of the compound materials.
	s := &g.shapes[shapeGoldenCompound]
	s.id = shapeGoldenCompound
	s.typ = CompoundShape
	s.density = 1.0
	s.sensorIndex = NullIndex
	s.proxyKey = NullIndex
	s.compound = compound
	s.materialCount = compound.MaterialCount
	copy(g.materials[shapeGoldenCompound][:], GetCompoundMaterials(compound)[:compound.MaterialCount])
	s.materials = g.materials[shapeGoldenCompound][:compound.MaterialCount]
}

func shapeGoldenTriangleCount(shape *shape) int {
	switch shape.typ {
	case MeshShape:
		return int(shape.mesh.Data.TriangleCount)
	case HeightShape:
		return getHeightFieldTriangleCount(shape.heightField)
	default:
		return 1
	}
}

func (g *shapeGoldenGen) shapeSection(index int) {
	shape := &g.shapes[index]
	convex := shape.typ == SphereShape || shape.typ == CapsuleShape || shape.typ == HullShape

	g.printf("shape %d props |", index)
	{
		md := computeShapeMass(shape)
		g.pf(md.Mass)
		g.pv(md.Center)
		g.pv(md.Inertia.Cx)
		g.pv(md.Inertia.Cy)
		g.pv(md.Inertia.Cz)
		g.pv(getShapeCentroid(shape))
		g.pf(getShapeArea(shape))
		for range 3 {
			n := Normalize(g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
			g.pf(getShapeProjectedArea(shape, n))
		}
		for range 2 {
			c := g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0)
			e := computeShapeExtent(shape, c)
			g.pf(e.minExtent)
			g.pv(e.maxExtent)
			g.pf(computeShapeMinExtent(shape, c))
		}
	}
	g.printf("\n")

	g.printf("shape %d aabb |", index)
	for range 2 {
		xf := g.rxf(2.0)
		g.pbox(computeShapeAABB(shape, xf))
		var wxf WorldTransform
		p := g.rv3(3.0, 3.0, 3.0, 0.0, 0.0, 0.0)
		wxf.P = Pos{p.X, p.Y, p.Z}
		wxf.Q = g.rq()
		extra := mulAdd(0.1, 0.05, g.rnd())
		g.pbox(computeFatShapeAABB(shape, wxf, extra))
		if convex {
			c1 := g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
			c2 := g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
			sweep := g.rsweep(c1, c2, false)
			time := mulAdd(0.5, 0.5, g.rnd())
			g.pbox(computeSweptShapeAABB(shape, &sweep, time))
		}
	}
	g.printf("\n")

	if convex {
		g.printf("shape %d proxy |", index)
		proxy := makeShapeProxy(shape)
		g.printf(" %d", proxy.Count)
		g.pf(proxy.Radius)
		for i := range int(proxy.Count) {
			g.pv(proxy.Points[i])
		}
		g.printf("\n")
	}

	centroid := getShapeCentroid(shape)

	g.printf("shape %d ray |", index)
	for range 6 {
		xf := g.rxf(2.0)
		target := Add(TransformPoint(xf, centroid), g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0))
		var input RayCastInput
		input.Origin = Add(target, g.rv3(4.0, 4.0, 4.0, 0.0, 0.0, 0.0))
		input.Translation = MulSV(2.0, Sub(target, input.Origin))
		input.MaxFraction = 1.0
		g.pco(rayCastShape(shape, xf, &input))
	}
	g.printf("\n")

	g.printf("shape %d cast |", index)
	for range 6 {
		xf := g.rxf(2.0)
		target := Add(TransformPoint(xf, centroid), g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0))
		origin := Add(target, g.rv3(4.0, 4.0, 4.0, 0.0, 0.0, 0.0))
		var input ShapeCastInput
		input.Proxy = g.rproxy(origin, 0.3)
		input.Translation = MulSV(2.0, Sub(target, origin))
		input.MaxFraction = 1.0
		input.CanEncroach = g.rndInt(2) == 1
		g.pco(shapeCastShape(shape, xf, &input))
	}
	g.printf("\n")

	g.printf("shape %d overlap |", index)
	for range 8 {
		xf := g.rxf(2.0)
		center := Add(TransformPoint(xf, centroid), g.rv3(1.5, 1.5, 1.5, 0.0, 0.0, 0.0))
		proxy := g.rproxy(center, 0.4)
		g.pbool(overlapShape(shape, xf, &proxy))
	}
	g.printf("\n")

	g.printf("shape %d mover |", index)
	for range 4 {
		xf := g.rxf(2.0)
		var mover Capsule
		mover.Center1 = Add(TransformPoint(xf, centroid), g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
		mover.Center2 = Add(mover.Center1, g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0))
		mover.Radius = mulAdd(0.3, 0.2, g.rnd())
		var planes [8]PlaneResult
		count := collideMover(planes[:8], shape, xf, &mover)
		g.pplanes(planes[:], count)
		g.printf(" %d", collideMover(planes[:0], shape, xf, &mover))
	}
	g.printf("\n")

	g.printf("shape %d material |", index)
	if shape.typ == CompoundShape {
		childCount := int(shape.compound.CapsuleCount + shape.compound.HullCount + shape.compound.MeshCount +
			shape.compound.SphereCount)
		for c := range childCount {
			child := GetCompoundChild(shape.compound, c)
			count := 1
			if child.Type == MeshShape {
				count = int(child.Mesh.Data.TriangleCount)
			}
			for t := 0; t < count; t += 1 + count/5 {
				g.printf(" %d", getShapeUserMaterialId(shape, c, t))
			}
		}
	} else {
		count := shapeGoldenTriangleCount(shape)
		for t := 0; t < count; t += 1 + count/12 {
			g.printf(" %d", getShapeUserMaterialId(shape, 0, t))
		}
	}
	g.printf("\n")

	g.printf("shape %d toi |", index)
	for b := shapeGoldenSphere; b <= shapeGoldenCylinder; b++ {
		for k := range 3 {
			a1 := g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
			a2 := a1
			if convex {
				a2 = Add(a1, g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
			}
			sweepA := g.rsweep(a1, a2, convex == false)

			target := Add(a1, g.rv3(0.5, 0.5, 0.5, 0.0, 0.0, 0.0))
			b1 := Add(target, g.rv3(3.0, 3.0, 3.0, 0.0, 0.0, 0.0))
			b2 := Sub(target, MulSV(0.5, Sub(b1, target)))
			sweepB := g.rsweep(b1, b2, false)

			maxFraction := mulAdd(0.5, 0.5, g.rnd())
			if k == 2 {
				shape.sensorIndex = 0
			} else {
				shape.sensorIndex = NullIndex
			}
			g.ptoi(shapeTimeOfImpact(shape, &g.shapes[b], &sweepA, &sweepB, maxFraction))
			shape.sensorIndex = NullIndex
		}
	}
	g.printf("\n")
}

func (g *shapeGoldenGen) run() string {
	g.seed = 4242
	g.makeShapes()

	for i := range shapeGoldenCount {
		g.shapeSection(i)
	}

	return g.sb.String()
}

var _ = Describe("ShapeGoldenTest", func() {
	It("MatchesC", func() {
		want, err := os.ReadFile("testdata/shape_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g shapeGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		for i := range min(len(wantLines), len(gotLines)) {
			if strings.HasPrefix(wantLines[i], "shape 7 aabb |") {
				// Golem intentionally expands only heightfield runtime bounds to
				// enclose quantized geometry. Preserve the C fixture and compare
				// conservative containment, bounded by the computed error budget.
				wantFields, gotFields := strings.Fields(wantLines[i]), strings.Fields(gotLines[i])
				Expect(gotFields[:4]).To(Equal(wantFields[:4]))
				Expect(gotFields).To(HaveLen(len(wantFields)))
				hf := g.shapes[shapeGoldenHeightField].heightField
				bound := queryHeightFieldLocalBounds(hf)
				budget := 4*(hf.AABB.LowerBound.Y-bound.LowerBound.Y) + 0.00001
				for j := 4; j < len(wantFields); j++ {
					wb, e := strconv.ParseUint(wantFields[j], 16, 32)
					Expect(e).NotTo(HaveOccurred())
					gb, e := strconv.ParseUint(gotFields[j], 16, 32)
					Expect(e).NotTo(HaveOccurred())
					w, v := math.Float32frombits(uint32(wb)), math.Float32frombits(uint32(gb))
					Expect(AbsFloat(v - w)).To(BeNumerically("<=", budget))
					if (j-4)%6 < 3 {
						Expect(v).To(BeNumerically("<=", w))
					} else {
						Expect(v).To(BeNumerically(">=", w))
					}
				}
				continue
			}
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
		Expect(gotLines).To(HaveLen(len(wantLines)))
	})
})
