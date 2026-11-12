// Go-only test: no upstream counterpart. Compares hull construction, hull queries, sphere and
// capsule queries, mover planes, polygon clipping, GJK distance, shape cast and time of impact against the pinned upstream C sources
// (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off, bit for bit and
// byte for byte (hull images are compared through their byte count and content hash). See
// testdata/hull_golden_gen.c; this file mirrors it line by line. The selected
// corrected-C topology format and original-fixture preservation are documented
// in testdata/golem_hull_topology/README.md.
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

type hullGoldenGen struct {
	seed uint32
	sb   strings.Builder
}

func (g *hullGoldenGen) rnd() float32 {
	g.seed ^= g.seed << 13
	g.seed ^= g.seed >> 17
	g.seed ^= g.seed << 5
	return float32(g.seed%20001)/10000.0 - 1.0
}

func (g *hullGoldenGen) rv3(sx, sy, sz, ox, oy, oz float32) Vec3 {
	var v Vec3
	v.X = mulAdd(ox, sx, g.rnd())
	v.Y = mulAdd(oy, sy, g.rnd())
	v.Z = mulAdd(oz, sz, g.rnd())
	return v
}

func (g *hullGoldenGen) rq() Quat {
	var q Quat
	q.V = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
	q.S = g.rnd()
	return NormalizeQuat(q)
}

func (g *hullGoldenGen) printf(format string, args ...any) { fmt.Fprintf(&g.sb, format, args...) }
func (g *hullGoldenGen) pf(f float32)                      { g.printf(" %08x", math.Float32bits(f)) }
func (g *hullGoldenGen) pv(v Vec3)                         { g.pf(v.X); g.pf(v.Y); g.pf(v.Z) }
func (g *hullGoldenGen) pb(b bool) {
	if b {
		g.printf(" 1")
	} else {
		g.printf(" 0")
	}
}

func (g *hullGoldenGen) ph(h *HullData) {
	if h == nil {
		g.printf(" null")
		return
	}
	g.printf(" %d %016x", h.ByteCount, h.Hash)
	g.pf(h.Volume)
	g.pf(h.SurfaceArea)
	g.pf(h.InnerRadius)
	g.pv(h.Center)
}

func (g *hullGoldenGen) run() string {
	g.seed = 777

	g.printf("box |")
	{
		b := MakeBoxHull(0.5, 1.0, 2.0)
		g.ph(&b.Base)
		xf := Transform{Vec3{1.0, -2.0, 3.0}, NormalizeQuat(Quat{Vec3{0.1, 0.2, 0.3}, 0.9})}
		b = MakeTransformedBoxHull(0.25, 0.5, 0.75, xf)
		g.ph(&b.Base)
		b = MakeScaledBoxHull(Vec3{0.5, 0.25, 1.0}, xf, Vec3{-1.5, 2.0, 0.5})
		g.ph(&b.Base)
	}
	g.printf("\n")

	g.printf("shapes |")
	g.ph(CreateCylinder(2.0, 1.0, 0.5, 12))
	g.ph(CreateCone(1.5, 1.0, 0.25, 10))
	g.ph(CreateRock(0.7))
	g.ph(CreateComplexHull(1.3))
	g.printf("\n")

	for k := range 40 {
		var sphere Sphere
		sphere.Center = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
		sphere.Radius = mulAdd(0.5, 0.4, g.rnd())
		var capsule Capsule
		capsule.Center1 = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
		if k%5 == 0 {
			capsule.Center2 = capsule.Center1
		} else {
			capsule.Center2 = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
		}
		capsule.Radius = mulAdd(0.3, 0.2, g.rnd())
		var mover Capsule
		mover.Center1 = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		mover.Center2 = g.rv3(2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
		mover.Radius = mulAdd(0.5, 0.4, g.rnd())
		var ray RayCastInput
		ray.Origin = g.rv3(3.0, 3.0, 3.0, 0.0, 0.0, 0.0)
		if k%7 == 3 {
			// Parallel to the capsule axis.
			ray.Translation = MulSV(4.0, Sub(capsule.Center2, capsule.Center1))
		} else {
			ray.Translation = Sub(g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), ray.Origin)
		}
		ray.MaxFraction = 1.0
		var xf Transform
		xf.P = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
		xf.Q = g.rq()

		g.printf("prim %d |", k)
		md := ComputeSphereMass(&sphere, 1.5)
		g.pf(md.Mass)
		g.pv(md.Inertia.Cx)
		md = ComputeCapsuleMass(&capsule, 1.5)
		g.pf(md.Mass)
		g.pv(md.Center)
		g.pv(md.Inertia.Cx)
		g.pv(md.Inertia.Cy)
		g.pv(md.Inertia.Cz)
		box := ComputeCapsuleAABB(&capsule, xf)
		g.pv(box.LowerBound)
		g.pv(box.UpperBound)
		box = ComputeSphereAABB(&sphere, xf)
		g.pv(box.LowerBound)
		g.pv(box.UpperBound)

		o := RayCastSphere(&sphere, &ray)
		g.pv(o.Point)
		g.pv(o.Normal)
		g.pf(o.Fraction)
		g.pb(o.Hit)
		o = RayCastHollowSphere(&sphere, &ray)
		g.pv(o.Point)
		g.pv(o.Normal)
		g.pf(o.Fraction)
		g.pb(o.Hit)
		o = RayCastCapsule(&capsule, &ray)
		g.pv(o.Point)
		g.pv(o.Normal)
		g.pf(o.Fraction)
		g.pb(o.Hit)

		var pr PlaneResult
		n := collideMoverAndSphere(&pr, &sphere, &mover)
		g.printf(" %d", n)
		g.pv(pr.Plane.Normal)
		g.pf(pr.Plane.Offset)
		g.pv(pr.Point)
		n = collideMoverAndCapsule(&pr, &capsule, &mover)
		g.printf(" %d", n)
		g.pv(pr.Plane.Normal)
		g.pf(pr.Plane.Offset)
		g.pv(pr.Point)

		proxy := ShapeProxy{capsulePoints(&mover), 2, mover.Radius}
		g.pb(OverlapSphere(&sphere, xf, &proxy))
		g.pb(OverlapCapsule(&capsule, xf, &proxy))
		g.printf("\n")
	}

	for k := range 20 {
		// Clip a pentagon against a random plane through its neighborhood.
		var polygon [5]clipVertex
		for i := range 5 {
			angle := float32(1.25 * float32(i))
			cs := ComputeCosSin(angle)
			polygon[i].position = Vec3{cs.Cosine, float32(0.1 * float32(i)), cs.Sine}
			polygon[i].separation = 0.0
			polygon[i].pair = makeFeaturePair(featureShapeB, i, featureShapeB, (i+1)%5)
		}
		var clipPlane Plane
		clipPlane.Normal = Normalize(g.rv3(1.0, 0.2, 1.0, 0.0, 0.0, 0.0))
		clipPlane.Offset = float32(0.5 * g.rnd())
		refPlane := Plane{Vec3AxisY, 0.05}
		var out [10]clipVertex
		outCount := clipPolygon(out[:], polygon[:], 5, clipPlane, k, refPlane)
		g.printf("clip %d |", k)
		g.pv(clipPlane.Normal)
		g.pf(clipPlane.Offset)
		g.printf(" %d", outCount)
		for i := range outCount {
			pair := flipPair(out[i].pair)
			g.pv(out[i].position)
			g.pf(out[i].separation)
			g.printf(" %08x %08x", makeFeatureId(out[i].pair), makeFeatureId(pair))
		}
		g.printf("\n")
	}

	counts := []int{8, 20, 64, 300}
	maxCounts := []int{8, 16, 32, 64}
	for c := range 4 {
		for k := range 6 {
			var points [300]Vec3
			n := counts[c]
			g.printf("hull %d %d", n, maxCounts[k%4])
			for i := range n {
				points[i] = g.rv3(3.0, 2.0, 1.0, 0.0, 0.0, 0.0)
				if k&1 != 0 {
					points[i] = Normalize(points[i])
				}
				g.pv(points[i])
			}
			g.printf(" |")
			h := CreateHull(points[:], n, maxCounts[k%4])
			g.ph(h)
			if h != nil {
				var xf Transform
				xf.P = g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
				xf.Q = g.rq()
				scale := g.rv3(1.0, -1.0, 1.0, 1.0, -1.0, 0.5)
				g.pv(xf.P)
				g.pf(xf.Q.V.X)
				g.pf(xf.Q.V.Y)
				g.pf(xf.Q.V.Z)
				g.pf(xf.Q.S)
				g.pv(scale)
				t := CloneAndTransformHull(h, xf, scale)
				g.ph(t)

				dir := Normalize(g.rv3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
				g.pv(dir)
				supportVertex := findHullSupportVertex(h, dir)
				g.printf(" %d %d %d", supportVertex, findHullSupportFace(h, dir), findIncidentFace(h, dir, supportVertex))
				g.pf(computeHullProjectedArea(h, dir))

				var ray RayCastInput
				ray.Origin = g.rv3(5.0, 5.0, 5.0, 0.0, 0.0, 0.0)
				ray.Translation = Sub(g.rv3(0.1, 0.1, 0.1, 0.0, 0.0, 0.0), ray.Origin)
				ray.MaxFraction = 1.0
				g.pv(ray.Origin)
				g.pv(ray.Translation)
				ro := RayCastHull(h, &ray)
				g.pv(ro.Point)
				g.pv(ro.Normal)
				g.pf(ro.Fraction)
				g.pb(ro.Hit)

				// Distance and cast against a capsule-like segment.
				var seg [2]Vec3
				seg[0] = g.rv3(1.0, 1.0, 1.0, 4.0, 0.0, 0.0)
				seg[1] = g.rv3(1.0, 1.0, 1.0, 4.0, 1.0, 0.0)
				g.pv(seg[0])
				g.pv(seg[1])
				var di DistanceInput
				di.ProxyA = ShapeProxy{GetHullPoints(h), h.VertexCount, 0.0}
				di.ProxyB = ShapeProxy{seg[:], 2, 0.1}
				di.Transform = xf
				di.UseRadii = true
				cache := SimplexCache{}
				dout := ShapeDistance(&di, &cache, nil)
				g.pv(dout.PointA)
				g.pv(dout.PointB)
				g.pv(dout.Normal)
				g.pf(dout.Distance)
				g.printf(" %d", dout.Iterations)

				var ci ShapeCastPairInput
				ci.ProxyA = di.ProxyA
				ci.ProxyB = di.ProxyB
				ci.Transform = xf
				ci.TranslationB = MulSV(-8.0, xf.P)
				ci.MaxFraction = 1.0
				ci.CanEncroach = false
				co := ShapeCast(&ci)
				g.pv(co.Point)
				g.pv(co.Normal)
				g.pf(co.Fraction)
				g.pb(co.Hit)
				g.printf(" %d", co.Iterations)

				var ti TOIInput
				ti.ProxyA = di.ProxyA
				ti.ProxyB = di.ProxyB
				ti.SweepA = Sweep{Vec3Zero, Vec3Zero, Vec3Zero, QuatIdentity, xf.Q}
				ti.SweepB = Sweep{Vec3Zero, Vec3{6.0, 0.0, 0.0}, Vec3{-2.0, 0.5, 0.0}, QuatIdentity, QuatIdentity}
				ti.MaxFraction = 1.0
				to := TimeOfImpact(&ti)
				g.printf(" %d", to.State)
				g.pv(to.Point)
				g.pv(to.Normal)
				g.pf(to.Fraction)
				g.pf(to.Distance)

				mover := Capsule{seg[0], seg[1], 2.0}
				var pr PlaneResult
				planeCount := collideMoverAndHull(&pr, h, &mover)
				g.printf(" %d", planeCount)
				g.pv(pr.Plane.Normal)
				g.pf(pr.Plane.Offset)
				g.pv(pr.Point)
				proxy := ShapeProxy{seg[:], 2, 0.5}
				g.pb(OverlapHull(h, xf, &proxy))

				md := ComputeHullMass(h, 2.5)
				g.pf(md.Mass)
				g.pv(md.Inertia.Cx)
				g.pv(md.Inertia.Cy)
				g.pv(md.Inertia.Cz)
			}
			g.printf("\n")
		}
	}

	return g.sb.String()
}

var _ = Describe("HullGoldenTest", func() {
	It("MatchesC", func() {
		// Hull limit messages are silenced like the generator does.
		SetLogFcn(func(string) {})
		defer SetLogFcn(defaultLogFcn)

		want, err := os.ReadFile("testdata/golem_hull_topology/hull_golden.txt")
		Expect(err).NotTo(HaveOccurred())

		var g hullGoldenGen
		got := g.run()

		wantLines := strings.Split(string(want), "\n")
		gotLines := strings.Split(got, "\n")
		Expect(gotLines).To(HaveLen(len(wantLines)))
		for i := range wantLines {
			Expect(gotLines[i]).To(Equal(wantLines[i]), "golden line %d", i+1)
		}
	})
})
