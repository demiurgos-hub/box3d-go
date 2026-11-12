package box3d

import (
	"errors"
	"math"
	"reflect"
	"testing"
)

func TestGolemObservedStepMatchesNativeElapsedForces(t *testing.T) {
	for _, continuous := range []bool{false, true} {
		t.Run(map[bool]string{false: "discrete", true: "continuous"}[continuous], func(t *testing.T) {
			makeWorld := func() (WorldID, BodyID) {
				d := DefaultWorldDef()
				d.EnableContinuous, d.EnableSleep = continuous, false
				w := CreateWorld(&d)
				t.Cleanup(w.Destroy)
				body := motionStepBody(w, DynamicBody, Pos{Y: 10}, Vec3{X: 1})
				motionStepSphere(body, .5)
				return w, body
			}
			native, a := makeWorld()
			observed, b := makeWorld()
			var samples []MotionStepSample
			for i := 0; i < 4; i++ {
				for _, body := range []BodyID{a, b} {
					body.ApplyForceToCenter(Vec3{X: 3}, true)
					body.ApplyTorque(Vec3{Y: 1}, true)
				}
				native.Step(.0125, 1)
				if err := observed.StepObserved(.0125, DefaultObservedStepOptions(), func(s MotionStepSample) error {
					if !getWorldFromId(observed).locked {
						t.Fatal("observer may mutate the world")
					}
					samples = append(samples, s)
					return nil
				}); err != nil {
					t.Fatal(err)
				}
				if a.GetTransform() != b.GetTransform() || a.GetLinearVelocity() != b.GetLinearVelocity() || a.GetAngularVelocity() != b.GetAngularVelocity() {
					t.Fatal("observation changed native motion or force duration")
				}
			}
			if len(samples) != 4 || len(samples[0].Bodies) != 1 || samples[0].Delta != .0125 || samples[0].StartFraction != 0 || samples[0].EndFraction != 1 {
				t.Fatalf("unexpected observation boundaries: %+v", samples)
			}
			first := samples[0].Bodies[0]
			if first.Start.P != (Pos{Y: 10}) || first.End != samples[1].Bodies[0].Start {
				t.Fatal("body-origin path has a gap")
			}
			b.SetTransform(Pos{Y: 20}, QuatIdentity)
			if samples[0].Bodies[0] != first {
				t.Fatal("retained observation aliases native bodies")
			}
		})
	}
}

func TestGolemObservedStepFailureDoesNotRewind(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3{X: 2})
	motionStepSphere(body, .5)
	want := errors.New("captured navigation safety rejected movement")
	options := DefaultObservedStepOptions()
	options.VelocityIterations, options.RelaxIterations = 4, 2
	err := w.StepObserved(.0125, options, func(MotionStepSample) error { return want })
	if !errors.Is(err, want) || body.GetPosition().X != .025 || getWorldFromId(w).locked {
		t.Fatalf("failed observer rewound or left world locked: %v %v", body.GetPosition(), err)
	}
	if got := motionIterationSettings[w.Index1-1]; got != (motionSolverIterations{}) {
		t.Fatalf("iteration override leaked: %+v", got)
	}
}

func TestGolemObservedStepRetainsContactSamples(t *testing.T) {
	w, _ := motionContactFixture(t, 2)
	var sample MotionStepSample
	if err := w.StepObserved(1.0/80, DefaultObservedStepOptions(), func(s MotionStepSample) error { sample = s; return nil }); err != nil {
		t.Fatal(err)
	}
	if len(sample.Contacts) != 2 {
		t.Fatalf("missing physical contacts: %d", len(sample.Contacts))
	}
	want := append([]Manifold(nil), sample.Contacts[0].Manifolds...)
	w.Step(1.0/80, 1)
	if !reflect.DeepEqual(sample.Contacts[0].Manifolds, want) {
		t.Fatal("contact sample changed with later solver step")
	}
}

func TestGolemObservedStepRejectsInvalidBeforeMutation(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3{X: 1})
	motionStepSphere(body, .5)
	for _, dt := range []float32{0, -1, float32(math.NaN()), float32(math.Inf(1))} {
		if !errors.Is(w.StepObserved(dt, DefaultObservedStepOptions(), nil), ErrObservedStepInvalid) || body.GetPosition() != (Pos{}) {
			t.Fatal("invalid step mutated the world")
		}
	}
	for _, options := range []ObservedStepOptions{{}, {0, 1}, {1, 0}, {17, 1}, {1, 17}} {
		if !errors.Is(w.StepObserved(.0125, options, nil), ErrObservedStepInvalid) || body.GetPosition() != (Pos{}) {
			t.Fatal("invalid solver settings mutated the world")
		}
	}
}
