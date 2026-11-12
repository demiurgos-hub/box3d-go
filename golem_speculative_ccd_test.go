package box3d

import (
	"fmt"
	"testing"
)

func speculativeWorld(t *testing.T) WorldID {
	t.Helper()
	w := motionStepWorld(t)
	if err := w.SetSpeculativeCCD(true); err != nil {
		t.Fatal(err)
	}
	return w
}

func observedFourSteps(t *testing.T, w WorldID) float32 {
	t.Helper()
	var impulse float32
	for i := 0; i < 4; i++ {
		if err := w.StepObserved(.0125, DefaultObservedStepOptions(), func(sample MotionStepSample) error {
			for _, c := range sample.Contacts {
				for _, m := range c.Manifolds {
					for j := int32(0); j < m.PointCount; j++ {
						impulse += m.Points[j].TotalNormalImpulse
					}
				}
			}
			return nil
		}); err != nil {
			t.Fatalf("step %d: %v", i, err)
		}
	}
	return impulse
}

func TestGolemSpeculativeCCDCrossingDynamics(t *testing.T) {
	for _, radius := range []float32{.5, .05, .005} {
		for _, bullet := range []bool{false, true} {
			t.Run(fmt.Sprintf("radius%g/bullet%v", radius, bullet), func(t *testing.T) {
				w := speculativeWorld(t)
				a := motionStepBody(w, DynamicBody, Pos{X: -2}, Vec3{X: 80})
				b := motionStepBody(w, DynamicBody, Pos{X: 2}, Vec3{X: -80})
				a.SetBullet(bullet)
				b.SetBullet(bullet)
				motionStepSphere(a, radius)
				motionStepSphere(b, radius)
				impulse := observedFourSteps(t, w)
				if a.GetPosition().X >= b.GetPosition().X || AbsFloat(a.GetLinearVelocity().X+b.GetLinearVelocity().X) > .001 || impulse <= 0 {
					t.Fatalf("crossed without equal/opposite response: a=%v v=%v b=%v v=%v impulse=%g", a.GetPosition(), a.GetLinearVelocity(), b.GetPosition(), b.GetLinearVelocity(), impulse)
				}
			})
		}
	}
}

func TestGolemSpeculativeCCDTranslatingMeshPushesPassiveBody(t *testing.T) {
	w := speculativeWorld(t)
	floor := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
	sd := DefaultShapeDef()
	sd.EnableContactEvents = true
	floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	motionStepSphere(body, .5)
	impulse := observedFourSteps(t, w)
	if body.GetPosition().Y < 2.45 || body.GetLinearVelocity().Y < 79 || impulse <= 0 {
		t.Fatalf("moving mesh lost contact response: position=%v velocity=%v impulse=%g", body.GetPosition(), body.GetLinearVelocity(), impulse)
	}
}

func TestGolemSpeculativeCCDRotatingMeshMovesPassiveBody(t *testing.T) {
	w := speculativeWorld(t)
	platform := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
	platform.SetAngularVelocity(Vec3{Z: 20})
	sd := DefaultShapeDef()
	sd.EnableContactEvents = true
	platform.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	body := motionStepBody(w, DynamicBody, Pos{X: 1.5, Y: .5}, Vec3Zero)
	motionStepSphere(body, .2)
	impulse := observedFourSteps(t, w)
	if body.GetPosition().Y < .6 || impulse <= 0 {
		t.Fatalf("rotating mesh lost contact response: position=%v velocity=%v impulse=%g", body.GetPosition(), body.GetLinearVelocity(), impulse)
	}
}

func TestGolemSpeculativeCCDRespectsPreSolveRejection(t *testing.T) {
	w := speculativeWorld(t)
	a := motionStepBody(w, DynamicBody, Pos{X: -1}, Vec3{X: 160})
	b := motionStepBody(w, DynamicBody, Pos{X: 1}, Vec3{X: -160})
	sa := motionStepSphere(a, .05)
	motionStepSphere(b, .05)
	sa.EnablePreSolveEvents(true)
	calls := 0
	w.SetPreSolveCallback(func(ShapeID, ShapeID, Pos, Vec3, any) bool { calls++; return false }, nil)
	if err := w.StepObserved(.0125, DefaultObservedStepOptions(), func(s MotionStepSample) error {
		if len(s.Contacts) != 0 {
			t.Fatal("filtered predictive contact was solved")
		}
		return nil
	}); err != nil {
		t.Fatal(err)
	}
	if calls != 1 || a.GetPosition().X != 1 || b.GetPosition().X != -1 || a.GetLinearVelocity().X != 160 || b.GetLinearVelocity().X != -160 {
		t.Fatalf("pre-solve filter was ignored: calls=%d positions=%v/%v", calls, a.GetPosition(), b.GetPosition())
	}
}

func TestGolemSpeculativeCCDDoesNotDuplicateForces(t *testing.T) {
	create := func(enabled bool) (WorldID, BodyID) {
		w := motionStepWorld(t)
		if err := w.SetSpeculativeCCD(enabled); err != nil {
			t.Fatal(err)
		}
		body := motionStepBody(w, DynamicBody, Pos{Y: 3}, Vec3{X: 20})
		motionStepSphere(body, .05)
		return w, body
	}
	a, bodyA := create(false)
	b, bodyB := create(true)
	for i := 0; i < 4; i++ {
		for _, body := range []BodyID{bodyA, bodyB} {
			body.ApplyForceToCenter(Vec3{Y: 100}, true)
			body.ApplyTorque(Vec3{Z: .1}, true)
		}
		a.Step(.0125, 1)
		b.Step(.0125, 1)
		if err := b.SpeculativeCCDError(); err != nil {
			t.Fatal(err)
		}
		if bodyA.GetTransform() != bodyB.GetTransform() || bodyA.GetLinearVelocity() != bodyB.GetLinearVelocity() || bodyA.GetAngularVelocity() != bodyB.GetAngularVelocity() {
			t.Fatal("prediction changed force, torque or elapsed integration")
		}
	}
}

func TestGolemSpeculativeCCDRecordingFailsClosed(t *testing.T) {
	w := speculativeWorld(t)
	recording := CreateRecording(4096)
	t.Cleanup(recording.Destroy)
	w.StartRecording(recording)
	if getWorldFromId(w).recording != nil {
		t.Fatal("unsupported CCD policy entered upstream recording")
	}
	if err := w.SetSpeculativeCCD(false); err != nil {
		t.Fatal(err)
	}
	w.StartRecording(recording)
	if getWorldFromId(w).recording == nil {
		t.Fatal("ordinary upstream recording was disabled")
	}
	if err := w.SetSpeculativeCCD(true); err == nil {
		t.Fatal("unrecordable CCD mutation was accepted")
	}
	w.StopRecording()
}
