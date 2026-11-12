package box3d

import "testing"

func TestGolemMotionIntervalsExactRotatingEndpointTangency(t *testing.T) {
	for _, offset := range []float32{0, 2304, -2304} {
		for _, localX := range []float32{-1, 1} {
			w, a := golemQueryWorld(t)
			d := DefaultBodyDef()
			z := w.CreateBody(&d)
			sa, sz := motionSphere(a, .25, Vec3{X: localX}), motionSphere(z, .25, Vec3Zero)
			a0, a1 := motionPose(offset, 0, 0, QuatIdentity), motionPose(offset, 0, 0, Quat{V: Vec3{Y: 1}})
			b0 := motionPose(offset+1.5, 0, 0, QuatIdentity)
			var dst [4]MotionInterval
			r, err := sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
			if err != nil || r.Incomplete || r.Count != 1 {
				t.Fatalf("tangent localX=%g: %+v %v", localX, r, err)
			}
			v := dst[0]
			if localX < 0 && (v.Exit.Fraction != 1 || v.Enter.Fraction < .99) || localX > 0 && (v.Enter.Fraction != 0 || v.Exit.Fraction > .01) {
				t.Fatalf("wrong tangent interval: %+v", v)
			}
			for _, hit := range []MotionCastResult{v.Enter, v.Exit} {
				if hit.Separation > 1e-5 || !hit.Hit {
					t.Fatalf("tangent boundary exceeded tolerance: %+v", hit)
				}
			}
			first, err := sa.CastMotion(sz, a0, a1, b0, b0, 1e-5, true)
			if err != nil || !first.Hit || AbsFloat(first.Fraction-v.Enter.Fraction) > .001 {
				t.Fatalf("earliest tangent: %+v %v", first, err)
			}
			if offset == 0 && localX < 0 {
				near := b0
				near.P.X += .000005
				r, err = sa.MotionIntervals(sz, a0, a1, near, near, 1e-5, true, dst[:])
				if err != nil || r.Incomplete || r.Count != 0 {
					t.Fatalf("geometric near miss became an interval: %+v %v", r, err)
				}
				first, err = sa.CastMotion(sz, a0, a1, near, near, 1e-5, true)
				if err != nil || !first.Hit || first.Separation > 1e-5 || first.Touching || first.Penetrating {
					t.Fatalf("distance-band near miss was lost: %+v %v", first, err)
				}
			}
			// This gap is representable even at the selected float32 envelope offsets.
			b0.P.X += .0005
			r, err = sa.MotionIntervals(sz, a0, a1, b0, b0, 1e-5, true, dst[:])
			if err != nil || r.Incomplete || r.Count != 0 {
				t.Fatalf("near tangent miss became contact: %+v %v", r, err)
			}
		}
	}
}
