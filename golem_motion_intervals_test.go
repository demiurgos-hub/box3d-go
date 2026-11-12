package box3d

import (
	"testing"
)

func TestGolemMotionIntervalsCrossingAndOverflow(t *testing.T) {
	w, a := golemQueryWorld(t)
	d := DefaultBodyDef()
	z := w.CreateBody(&d)
	sa, sz := motionSphere(a, .05, Vec3Zero), motionSphere(z, .01, Vec3Zero)
	a0, a1 := motionPose(-2, 0, 0, QuatIdentity), motionPose(3, 0, 0, QuatIdentity)
	b0 := motionPose(.5, 0, 0, QuatIdentity)
	var dst [2]MotionInterval
	r, err := sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
	if err != nil || r.Incomplete || r.Count != 1 || AbsFloat(dst[0].Enter.Fraction-.488) > 1e-5 || AbsFloat(dst[0].Exit.Fraction-.512) > 1e-5 {
		t.Fatalf("thin crossing: %+v %+v %v", r, dst, err)
	}
	r, err = sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, nil)
	if err != nil || !r.Incomplete || r.Total != 1 || r.Count != 0 {
		t.Fatalf("overflow: %+v %v", r, err)
	}
}

func TestGolemMotionIntervalsConcaveReentry(t *testing.T) {
	w, a := golemQueryWorld(t)
	d := DefaultBodyDef()
	z := w.CreateBody(&d)
	cube := MakeBoxHull(.01, 1, 1)
	compound := CreateCompound(&CompoundDef{HullCount: 3, Hulls: []CompoundHullDef{
		{Hull: &cube.Base, Transform: Transform{P: Vec3{X: -.5}, Q: QuatIdentity}},
		{Hull: &cube.Base, Transform: Transform{P: Vec3{X: .5}, Q: QuatIdentity}},
		{Hull: &cube.Base, Transform: Transform{P: Vec3{X: .51}, Q: QuatIdentity}},
	}})
	if compound == nil {
		t.Fatal("compound cook")
	}
	sd := DefaultShapeDef()
	sa, sz := motionSphere(a, .05, Vec3Zero), z.CreateBakedCompoundShape(&sd, compound)
	a0, a1, b0 := motionPose(-2, 0, 0, QuatIdentity), motionPose(3, 0, 0, QuatIdentity), motionPose(0, 0, 0, QuatIdentity)
	var dst [4]MotionInterval
	r, err := sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
	if err != nil || r.Incomplete || r.Count != 2 || dst[0].Exit.Fraction >= dst[1].Enter.Fraction || dst[1].Exit.FeatureB.ChildIndex != 2 {
		t.Fatalf("union/reentry: %+v %+v %v", r, dst, err)
	}
}

func TestGolemMotionIntervalsRotationCrossingAndReentry(t *testing.T) {
	w, a := golemQueryWorld(t)
	d := DefaultBodyDef()
	z := w.CreateBody(&d)
	sd := DefaultShapeDef()
	box := MakeBoxHull(2, .1, .1)
	sa, sz := a.CreateHullShape(&sd, &box.Base), motionSphere(z, .1, Vec3Zero)
	a0, a1, b0 := motionPose(0, 0, 0, QuatIdentity), motionPose(0, 0, 0, Quat{V: Vec3{Z: 1}}), motionPose(0, 1.5, 0, QuatIdentity)
	var dst [4]MotionInterval
	r, err := sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
	if err != nil || r.Incomplete || r.Count != 1 || dst[0].Enter.Fraction < .45 || dst[0].Exit.Fraction > .55 {
		t.Fatalf("rotation crossing: %+v %+v %v", r, dst, err)
	}
	b0 = motionPose(1.5, 0, 0, QuatIdentity)
	r, err = sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
	if err != nil || r.Incomplete || r.Count != 2 || dst[0].Enter.Fraction != 0 || dst[1].Exit.Fraction != 1 || dst[0].Exit.Fraction >= dst[1].Enter.Fraction {
		t.Fatalf("rotation reentry collapsed: %+v %+v %v", r, dst, err)
	}
}

func TestGolemMotionIntervalsRotatingHullsAndMesh(t *testing.T) {
	for _, mesh := range []bool{false, true} {
		w, a := golemQueryWorld(t)
		d := DefaultBodyDef()
		z := w.CreateBody(&d)
		sd := DefaultShapeDef()
		rod := MakeBoxHull(2, .1, .1)
		sa := a.CreateHullShape(&sd, &rod.Base)
		if mesh {
			sa = a.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
		}
		cube := MakeBoxHull(.1, .1, .1)
		sz := z.CreateHullShape(&sd, &cube.Base)
		a0, a1, b0 := motionPose(0, 0, 0, QuatIdentity), motionPose(0, 0, 0, Quat{V: Vec3{Z: 1}}), motionPose(0, 1.5, 0, QuatIdentity)
		var dst [4]MotionInterval
		r, err := sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
		if err != nil || r.Incomplete || r.Count != 1 || dst[0].Enter.Fraction < .4 || dst[0].Exit.Fraction > .6 {
			t.Fatalf("rotating mesh=%v: %+v %+v %v", mesh, r, dst, err)
		}
	}
}
