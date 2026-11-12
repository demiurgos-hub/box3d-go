package box3d

import (
	"errors"
	"math"
	"testing"
)

func motionPose(x, y, z float32, rotation Quat) WorldTransform {
	return WorldTransform{Pos{x, y, z}, rotation}
}
func motionSphere(body BodyID, r float32, center Vec3) ShapeID {
	sd := DefaultShapeDef()
	return body.CreateSphereShape(&sd, &Sphere{Center: center, Radius: r})
}

func TestGolemMotionCrossingBodiesAndTranslationFrame(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	qa, qb := motionSphere(a, .25, Vec3Zero), motionSphere(b, .25, Vec3Zero)
	for _, offset := range []float32{0, 768, -2304, 4090} {
		x0, x1 := motionPose(offset-2, 0, 0, QuatIdentity), motionPose(offset+2, 0, 0, QuatIdentity)
		r, err := qa.CastMotion(qb, x0, x1, x1, x0, 1e-5, true)
		if err != nil || !r.Hit || AbsFloat(r.Fraction-.4375) > 2e-5 || r.Normal.X < .999 || r.Initial {
			t.Fatalf("crossing at %g: %+v %v", offset, r, err)
		}
		if AbsFloat(r.PointA.X-offset) > 1e-3 || AbsFloat(r.PointB.X-offset) > 1e-3 {
			t.Fatal(r)
		}
		if a.GetPosition() != (Pos{}) || b.GetPosition() != (Pos{}) {
			t.Fatal("cast mutated body")
		}
	}
	// Identical translation retains the gap and must not become a false collision.
	a0, a1 := motionPose(0, 0, 0, QuatIdentity), motionPose(100, 0, 0, QuatIdentity)
	b0, b1 := motionPose(1, 0, 0, QuatIdentity), motionPose(101, 0, 0, QuatIdentity)
	if r, err := qa.CastMotion(qb, a0, a1, b0, b1, 1e-5, true); err != nil || r.Hit {
		t.Fatalf("common translation: %+v %v", r, err)
	}
}

func TestGolemMotionEndpointInToleranceBand(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	qa, qb := motionSphere(a, .5, Vec3Zero), motionSphere(b, .5, Vec3Zero)
	start := motionPose(0, 0, 0, QuatIdentity)
	end := motionPose(.9995, 0, 0, QuatIdentity)
	target := motionPose(2, 0, 0, QuatIdentity)
	r, err := qa.CastMotion(qb, start, end, target, target, .001, true)
	if err != nil || !r.Hit || r.Fraction <= 0 || r.Fraction > 1 || r.Separation > .001 || r.Initial {
		t.Fatalf("endpoint tolerance band cleared: %+v %v", r, err)
	}
}

func TestGolemMotionRotationArcOutsideEndpointBounds(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	sd := DefaultShapeDef()
	box := MakeBoxHull(3, .1, .1)
	rod := a.CreateHullShape(&sd, &box.Base)
	sphere := motionSphere(b, .2, Vec3Zero)
	start, end := motionPose(0, 0, 0, QuatIdentity), motionPose(0, 0, 0, Quat{V: Vec3{Z: 1}})
	target := motionPose(0, 2, 0, QuatIdentity)
	r, err := rod.CastMotion(sphere, start, end, target, target, 1e-5, true)
	if err != nil || !r.Hit || r.Fraction < .44 || r.Fraction > .49 {
		t.Fatalf("rotating rod: %+v %v", r, err)
	}
	reverse, err := sphere.CastMotion(rod, target, target, start, end, 1e-5, true)
	if err != nil || !reverse.Hit || AbsFloat(r.Fraction-reverse.Fraction) > 2e-5 || Dot(r.Normal, reverse.Normal) > -.99 {
		t.Fatalf("reverse pair: %+v %v", reverse, err)
	}
}

func TestGolemMotionMovingAndRotatingMeshFeatures(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	bd.Type = KinematicBody
	b := w.CreateBody(&bd)
	mesh := golemFlatMesh()
	sd := DefaultShapeDef()
	target := b.CreateMeshShape(&sd, mesh, Vec3One)
	query := motionSphere(a, .25, Vec3Zero)
	q := motionPose(1, 1, 0, QuatIdentity)
	m0, m1 := motionPose(0, -1, 0, QuatIdentity), motionPose(0, 2, 0, QuatIdentity)
	r, err := target.CastMotion(query, m0, m1, q, q, 1e-5, true)
	if err != nil || !r.Hit || AbsFloat(r.Fraction-1.75/3) > 1e-4 || r.FeatureA.TriangleIndex < 0 || r.FeatureB.TriangleIndex != -1 || r.Normal.Y < .99 {
		t.Fatalf("moving mesh: %+v %v", r, err)
	}
	m0, m1 = motionPose(0, 0, 0, QuatIdentity), motionPose(0, 0, 0, Quat{V: Vec3{Z: float32(math.Sqrt(.5))}, S: float32(math.Sqrt(.5))})
	r, err = target.CastMotion(query, m0, m1, q, q, 1e-5, true)
	if err != nil || !r.Hit || r.Fraction < .35 || r.Fraction > .5 || r.FeatureA.TriangleIndex < 0 {
		t.Fatalf("rotating mesh: %+v %v", r, err)
	}
	if _, err = target.CastMotion(query, m0, m1, q, q, 1e-5, false); !errors.Is(err, ErrMotionCastUnsupported) {
		t.Fatalf("unsupported rotating sidedness silently cleared: %v", err)
	}
}

func TestGolemMotionInitialContactClassification(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	qa, qb := motionSphere(a, .5, Vec3Zero), motionSphere(b, .5, Vec3Zero)
	pose := motionPose(0, 0, 0, QuatIdentity)
	for _, tc := range []struct {
		x                         float32
		touch, penetrate, initial bool
	}{{1, true, false, true}, {.75, false, true, true}, {1.000005, false, false, false}} {
		other := motionPose(tc.x, 0, 0, QuatIdentity)
		r, err := qa.CastMotion(qb, pose, pose, other, other, 1e-5, true)
		if err != nil || !r.Hit || r.Fraction != 0 || r.Touching != tc.touch || r.Penetrating != tc.penetrate || r.Initial != tc.initial {
			t.Fatalf("x=%v: %+v %v", tc.x, r, err)
		}
		if r.Penetrating && AbsFloat(r.Depth-(1-tc.x)) > 1e-5 {
			t.Fatal(r)
		}
	}
}

func TestGolemMotionCommonRotatingFrameAndCompoundChild(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	sd := DefaultShapeDef()
	box := MakeBoxHull(3, .1, .1)
	rod := a.CreateHullShape(&sd, &box.Base)
	passenger := motionSphere(b, .2, Vec3{X: 2, Y: .31})
	start, end := motionPose(0, 0, 0, QuatIdentity), motionPose(0, 0, 0, Quat{V: Vec3{Z: 1}})
	if r, err := rod.CastMotion(passenger, start, end, start, end, 1e-5, true); err != nil || r.Hit {
		t.Fatalf("common rotation: %+v %v", r, err)
	}
	cd := CompoundDef{Hulls: []CompoundHullDef{{Hull: &box.Base, Transform: Transform{P: Vec3{Y: 1}, Q: QuatIdentity}}}, HullCount: 1}
	compound := CreateCompound(&cd)
	if compound == nil {
		t.Fatal("cook compound")
	}
	target := a.CreateBakedCompoundShape(&sd, compound)
	q0, q1 := motionPose(0, 3, 0, QuatIdentity), motionPose(0, -1, 0, QuatIdentity)
	r, err := target.CastMotion(motionSphere(b, .2, Vec3Zero), start, start, q0, q1, 1e-5, true)
	if err != nil || !r.Hit || r.FeatureA.ChildIndex != 0 || AbsFloat(r.Fraction-.425) > 1e-4 {
		t.Fatalf("compound local transform: %+v %v", r, err)
	}
}

func TestGolemMotionFailuresCannotMeanClear(t *testing.T) {
	w, a := golemQueryWorld(t)
	bd := DefaultBodyDef()
	b := w.CreateBody(&bd)
	qa, qb := motionSphere(a, .5, Vec3Zero), motionSphere(b, .5, Vec3Zero)
	pose := motionPose(0, 0, 0, QuatIdentity)
	if _, err := qa.CastMotion(qb, pose, pose, pose, pose, 0, true); !errors.Is(err, ErrMotionCastInvalid) {
		t.Fatal(err)
	}
	qb.Destroy(false)
	if _, err := qa.CastMotion(qb, pose, pose, pose, pose, 1e-5, true); !errors.Is(err, ErrMotionCastInvalid) {
		t.Fatal(err)
	}
	// A sub-ULP tolerance can prevent representable advancement. It must fail,
	// never turn an unresolved impact into a successful clear-path result.
	qb = motionSphere(b, .5, Vec3Zero)
	far := motionPose(3.12345, 0, 0, QuatIdentity)
	end := motionPose(8.12345, 0, 0, QuatIdentity)
	r, err := qa.CastMotion(qb, pose, end, far, far, 1e-20, true)
	if err == nil && !r.Hit {
		t.Fatal("unresolved cast returned clear")
	}
}
