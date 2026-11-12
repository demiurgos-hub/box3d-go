package box3d

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"
)

func TestNativeGeometryRotationalArcCandidate(t *testing.T) {
	for _, offset := range []float32{0, 768, -2304} {
		t.Run(fmt.Sprint(offset), func(t *testing.T) {
			w := motionStepWorld(t)
			fixed := motionStepBody(w, StaticBody, Pos{X: offset + 3}, Vec3Zero)
			motionStepSphere(fixed, .04)
			rod := motionStepBody(w, DynamicBody, Pos{X: offset}, Vec3Zero)
			q := Quat{V: Vec3{Z: float32(-math.Sin(math.Pi / 18))}, S: float32(math.Cos(math.Pi / 18))}
			rod.SetTransform(Pos{X: offset}, q)
			rod.SetAngularVelocity(Vec3{Z: 15})
			d := DefaultShapeDef()
			h := MakeBoxHull(3, .05, .05)
			rod.CreateHullShape(&d, &h.Base)
			w.Step(.05, 1)
			t.Logf("native rod rotation=%+v velocity=%+v", rod.GetRotation(), rod.GetAngularVelocity())
			if rod.GetRotation().V.Z >= 0 {
				t.Fatal("native CCD missed the intermediate rotation outside endpoint AABBs")
			}
		})
	}
}

func TestNativeGeometryStaticTranslationBoundsUnchanged(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	id := motionStepSphere(body, .7)
	s := getShape(getWorldFromId(w), id)
	a := Sweep{LocalCenter: Vec3{X: .3}, C1: Vec3{X: 2, Y: 3}, C2: Vec3{X: 2, Y: 3}, Q1: QuatIdentity, Q2: QuatIdentity}
	b := Sweep{C1: Vec3{X: 4, Y: 3}, C2: Vec3{X: 4, Y: -3}, Q1: QuatIdentity, Q2: QuatIdentity}
	want := computeSweptShapeAABB(s, &b, .75).Transform(InvertTransform(getFinalSweepTransform(&a)))
	if got := continuousMeshLocalBounds(s, &a, &b, .75); got != want {
		t.Fatalf("static translation bounds changed: %+v != %+v", got, want)
	}
}

func TestNativeGeometryBoundsContainIntermediatePoses(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	d := DefaultShapeDef()
	h := MakeBoxHull(3, .05, .3)
	ids := []ShapeID{
		body.CreateHullShape(&d, &h.Base),
		body.CreateSphereShape(&d, &Sphere{Center: Vec3{X: 2}, Radius: .17}),
		body.CreateCapsuleShape(&d, &Capsule{Center1: Vec3{X: -1, Y: 2}, Center2: Vec3{X: 2, Y: 1}, Radius: .2}),
	}
	world := getWorldFromId(w)
	random := rand.New(rand.NewPCG(7349, 8173))
	scalar := func() float32 { return random.Float32()*20 - 10 }
	vector := func() Vec3 { return Vec3{scalar(), scalar(), scalar()} }
	rotation := func() Quat { return NormalizeQuat(Quat{V: vector(), S: scalar()}) }
	for iteration := 0; iteration < 300; iteration++ {
		a := Sweep{LocalCenter: vector(), C1: vector(), C2: vector(), Q1: rotation(), Q2: rotation()}
		b := Sweep{LocalCenter: vector(), C1: vector(), C2: vector(), Q1: rotation(), Q2: rotation()}
		fraction := .01 + .99*random.Float32()
		if iteration%3 == 0 {
			a.Q2 = a.Q1
		}
		if iteration%5 == 0 {
			b.Q2 = b.Q1
		}
		for _, id := range ids {
			shape := getShape(world, id)
			bounds := continuousShapeBounds(shape, &b, fraction)
			relative := continuousMeshLocalBounds(shape, &a, &b, fraction)
			for sample := 0; sample <= 32; sample++ {
				at := fraction * float32(sample) / 32
				xa, xb := GetSweepTransform(&a, at), GetSweepTransform(&b, at)
				actual := computeShapeAABB(shape, xb)
				if !bounds.Contains(actual) {
					t.Fatalf("world sweep omitted geometry: iteration%d sample%d bounds%+v actual%+v", iteration, sample, bounds, actual)
				}
				actual = computeShapeAABB(shape, InvMulTransforms(xa, xb))
				if !relative.Contains(actual) {
					t.Fatalf("relative sweep omitted geometry: iteration%d sample%d bounds%+v actual%+v", iteration, sample, relative, actual)
				}
			}
		}
	}
}

func TestNativeGeometryMeshTOIPreservesMissAndBackface(t *testing.T) {
	w := motionStepWorld(t)
	meshBody := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
	d := DefaultShapeDef()
	meshID := meshBody.CreateMeshShape(&d, golemFlatMesh(), Vec3One)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	sphereID := motionStepSphere(body, .2)
	world := getWorldFromId(w)
	for _, tc := range []struct {
		name string
		a, b Sweep
	}{
		{"common translation", Sweep{C2: Vec3{Y: 4}, Q1: QuatIdentity, Q2: QuatIdentity}, Sweep{C1: Vec3{Y: 1}, C2: Vec3{Y: 5}, Q1: QuatIdentity, Q2: QuatIdentity}},
		{"backface", Sweep{C1: Vec3{Y: 2}, C2: Vec3{Y: -2}, Q1: QuatIdentity, Q2: QuatIdentity}, Sweep{Q1: QuatIdentity, Q2: QuatIdentity}},
		{"outside mesh", Sweep{C1: Vec3{Y: -2}, C2: Vec3{Y: 2}, Q1: QuatIdentity, Q2: QuatIdentity}, Sweep{C1: Vec3{X: 3}, C2: Vec3{X: 3}, Q1: QuatIdentity, Q2: QuatIdentity}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			hit := shapeTimeOfImpact(getShape(world, meshID), getShape(world, sphereID), &tc.a, &tc.b, 1)
			if hit.State == TOIStateHit {
				t.Fatalf("unexpected mesh hit: %+v", hit)
			}
		})
	}
}

func TestNativeGeometryRelativeMeshTOI(t *testing.T) {
	for _, rotate := range []bool{false, true} {
		name := "translate"
		if rotate {
			name = "rotate"
		}
		t.Run(name, func(t *testing.T) {
			w := motionStepWorld(t)
			meshBody := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
			d := DefaultShapeDef()
			meshID := meshBody.CreateMeshShape(&d, golemFlatMesh(), Vec3One)
			body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
			radius := float32(.5)
			if rotate {
				radius = .2
			}
			sphereID := motionStepSphere(body, radius)
			a := Sweep{C1: Vec3{Y: -2}, C2: Vec3{Y: 2}, Q1: QuatIdentity, Q2: QuatIdentity}
			b := Sweep{Q1: QuatIdentity, Q2: QuatIdentity}
			if rotate {
				a.C1 = Vec3Zero
				a.C2 = Vec3Zero
				a.Q2 = Quat{V: Vec3{Z: float32(math.Sin(math.Pi / 8))}, S: float32(math.Cos(math.Pi / 8))}
				b.C1 = Vec3{X: 1.5, Y: .5}
				b.C2 = b.C1
			}
			world := getWorldFromId(w)
			hit := shapeTimeOfImpact(getShape(world, meshID), getShape(world, sphereID), &a, &b, 1)
			t.Logf("relative mesh TOI=%+v", hit)
			if hit.State != TOIStateHit || hit.Fraction <= 0 || hit.Fraction >= .5 {
				t.Fatal("relative mesh candidate was culled before native triangle TOI")
			}
		})
	}
}
