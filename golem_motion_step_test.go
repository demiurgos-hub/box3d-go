package box3d

import (
	"errors"
	"testing"
)

func motionStepWorld(t *testing.T) WorldID {
	t.Helper()
	d := DefaultWorldDef()
	d.Gravity = Vec3Zero
	d.EnableSleep = false
	w := CreateWorld(&d)
	t.Cleanup(w.Destroy)
	return w
}
func motionStepBody(w WorldID, typ BodyType, position Pos, velocity Vec3) BodyID {
	d := DefaultBodyDef()
	d.Type = typ
	d.Position = position
	d.LinearVelocity = velocity
	return w.CreateBody(&d)
}
func motionStepSphere(body BodyID, r float32) ShapeID {
	d := DefaultShapeDef()
	d.EnableContactEvents = true
	d.EnableHitEvents = true
	return body.CreateSphereShape(&d, &Sphere{Radius: r})
}

func TestGolemMotionStepCrossingDynamicsUseSolverImpulse(t *testing.T) {
	w := motionStepWorld(t)
	a := motionStepBody(w, DynamicBody, Pos{X: -2}, Vec3{X: 80})
	b := motionStepBody(w, DynamicBody, Pos{X: 2}, Vec3{X: -80})
	motionStepSphere(a, .5)
	motionStepSphere(b, .5)
	impulse := float32(0)
	samples := []MotionStepSample{}
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		samples = append(samples, s)
		for _, c := range s.Contacts {
			for _, m := range c.Manifolds {
				for i := int32(0); i < m.PointCount; i++ {
					impulse += m.Points[i].TotalNormalImpulse
				}
			}
		}
		return nil
	})
	if err != nil || r.Microsteps < 2 || r.Elapsed != .05 || a.GetPosition().X > b.GetPosition().X || impulse <= 0 {
		t.Fatalf("crossing: result=%+v err=%v a=%v b=%v impulse=%v", r, err, a.GetPosition(), b.GetPosition(), impulse)
	}
	if AbsFloat(a.GetLinearVelocity().X) > 1 || AbsFloat(b.GetLinearVelocity().X) > 1 {
		t.Fatalf("no physical response: %v %v", a.GetLinearVelocity(), b.GetLinearVelocity())
	}
	if samples[0].Bodies[0].Start.P == samples[0].Bodies[0].End.P || samples[len(samples)-1].EndFraction != 1 {
		t.Fatal("missing accepted pose samples")
	}
}

func TestGolemMotionStepTranslatingMeshPushesPassiveBody(t *testing.T) {
	w := motionStepWorld(t)
	floor := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
	mesh := golemFlatMesh()
	sd := DefaultShapeDef()
	sd.EnableContactEvents = true
	floor.CreateMeshShape(&sd, mesh, Vec3One)
	sphere := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	motionStepSphere(sphere, .5)
	impulse := float32(0)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		for _, c := range s.Contacts {
			for _, m := range c.Manifolds {
				for i := int32(0); i < m.PointCount; i++ {
					impulse += m.Points[i].TotalNormalImpulse
				}
			}
		}
		return nil
	})
	if err != nil || r.Microsteps < 2 || sphere.GetPosition().Y < 2.45 || sphere.GetLinearVelocity().Y < 79 || impulse <= 0 {
		t.Fatalf("mesh push: %+v %v floor=%v sphere=%v velocity=%v impulse=%g", r, err, floor.GetPosition(), sphere.GetPosition(), sphere.GetLinearVelocity(), impulse)
	}
}

func TestGolemMotionStepRotatorStrikesStationaryPassenger(t *testing.T) {
	w := motionStepWorld(t)
	platform := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
	platform.SetAngularVelocity(Vec3{Z: 20})
	sd := DefaultShapeDef()
	sd.EnableContactEvents = true
	platform.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	passenger := motionStepBody(w, DynamicBody, Pos{X: 1.5, Y: .5}, Vec3Zero)
	motionStepSphere(passenger, .2)
	impulse := float32(0)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		for _, c := range s.Contacts {
			for _, m := range c.Manifolds {
				for i := int32(0); i < m.PointCount; i++ {
					impulse += m.Points[i].TotalNormalImpulse
				}
			}
		}
		return nil
	})
	if err != nil || r.Microsteps < 10 || passenger.GetPosition().Y < .6 || impulse <= 0 {
		t.Fatalf("rotator: %+v %v passenger=%v velocity=%v impulse=%g", r, err, passenger.GetPosition(), passenger.GetLinearVelocity(), impulse)
	}
}

func TestGolemMotionStepForcesTorqueAndGravitySpanExactTime(t *testing.T) {
	w := motionStepWorld(t)
	w.SetGravity(Vec3{Y: -9.81})
	body := motionStepBody(w, DynamicBody, Pos{Y: 10}, Vec3Zero)
	motionStepSphere(body, .5)
	rotator := motionStepBody(w, KinematicBody, Pos{X: 100}, Vec3Zero)
	rotator.SetAngularVelocity(Vec3{Z: 20})
	motionStepSphere(rotator, 1)
	body.ApplyForceToCenter(Vec3{X: 4 * body.GetMass()}, true)
	body.ApplyTorque(Vec3{Z: .2}, true)
	wantAngular := getBodySim(&worlds[w.Index1-1], &worlds[w.Index1-1].bodies[body.Index1-1]).invInertiaWorld.Cz.Z * .2 * .05
	sum := float32(0)
	first := float32(0)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		if s.Index == 0 {
			first = s.EndFraction
		}
		sum += s.Delta
		return nil
	})
	if err != nil || r.Microsteps < 10 || AbsFloat(sum-.05) > 1e-7 || first >= 1 || AbsFloat(body.GetLinearVelocity().X-.2) > 1e-5 || AbsFloat(body.GetLinearVelocity().Y+.4905) > 1e-5 || AbsFloat(body.GetAngularVelocity().Z-wantAngular) > 1e-5 {
		t.Fatalf("force partition: %+v %v sum=%v v=%v w=%v wantW=%v", r, err, sum, body.GetLinearVelocity(), body.GetAngularVelocity(), wantAngular)
	}
}

func TestGolemMotionStepLimitAndObserverFailureAreExplicit(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
	body.SetAngularVelocity(Vec3{Z: 20})
	motionStepSphere(body, 1)
	o := DefaultMotionStepOptions()
	o.MaxMicrosteps = 1
	r, err := w.StepMotion(.05, o, nil)
	if !errors.Is(err, ErrMotionStepLimit) || r.Microsteps != 1 || r.Elapsed <= 0 || r.Elapsed >= .05 {
		t.Fatalf("cap did not report partial work: %+v %v", r, err)
	}
	want := errors.New("observer failure")
	r, err = w.StepMotion(.05, DefaultMotionStepOptions(), func(MotionStepSample) error { return want })
	if !errors.Is(err, want) || r.Microsteps != 1 {
		t.Fatalf("observer failure: %+v %v", r, err)
	}
}

func TestGolemMotionStepPassengerUnderCeilingRejectsCrush(t *testing.T) {
	w := motionStepWorld(t)
	floor := motionStepBody(w, KinematicBody, Pos{}, Vec3{Y: 10})
	sd := DefaultShapeDef()
	floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	passenger := motionStepBody(w, DynamicBody, Pos{Y: .505}, Vec3Zero)
	motionStepSphere(passenger, .5)
	ceiling := motionStepBody(w, StaticBody, Pos{Y: 1.3}, Vec3Zero)
	box := MakeBoxHull(2, .1, 2)
	ceiling.CreateHullShape(&sd, &box.Base)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), nil)
	if (!errors.Is(err, ErrMotionStepPenetration) && !errors.Is(err, ErrMotionStepUnsafePath)) || r.Microsteps == 0 || r.Elapsed >= .05 {
		t.Fatalf("crush was accepted: %+v %v floor=%v passenger=%v", r, err, floor.GetPosition(), passenger.GetPosition())
	}
}

func TestGolemMotionStepRetainsEverySensorTransition(t *testing.T) {
	w := motionStepWorld(t)
	sensor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	sd := DefaultShapeDef()
	sd.IsSensor = true
	sd.EnableSensorEvents = true
	sensor.CreateSphereShape(&sd, &Sphere{Radius: .25})
	visitor := motionStepBody(w, DynamicBody, Pos{X: -2}, Vec3{X: 40})
	sd = DefaultShapeDef()
	sd.EnableSensorEvents = true
	visitor.CreateSphereShape(&sd, &Sphere{Radius: .25})
	rotator := motionStepBody(w, KinematicBody, Pos{X: 100}, Vec3Zero)
	rotator.SetAngularVelocity(Vec3{Z: 20})
	motionStepSphere(rotator, 1)
	var samples []MotionStepSample
	r, err := w.StepMotion(.1, DefaultMotionStepOptions(), func(s MotionStepSample) error { samples = append(samples, s); return nil })
	begin, end := 0, 0
	for _, s := range samples {
		begin += len(s.SensorEvents.BeginEvents)
		end += len(s.SensorEvents.EndEvents)
	}
	if err != nil || r.Microsteps < 10 || begin != 1 || end != 1 {
		t.Fatalf("lost intra-frame transitions: %+v %v begin=%d end=%d", r, err, begin, end)
	}
}

func TestGolemMotionStepAllowsExistingPenetrationRecovery(t *testing.T) {
	w := motionStepWorld(t)
	floor := motionStepBody(w, StaticBody, Pos{Y: -.5}, Vec3Zero)
	sd := DefaultShapeDef()
	box := MakeBoxHull(2, .5, 2)
	floor.CreateHullShape(&sd, &box.Base)
	body := motionStepBody(w, DynamicBody, Pos{Y: .25}, Vec3Zero)
	motionStepSphere(body, .5)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), nil)
	if err != nil || r.Elapsed != .05 || body.GetPosition().Y <= .25 {
		t.Fatalf("existing overlap did not recover: %+v %v position=%v", r, err, body.GetPosition())
	}
}

func TestGolemMotionStepObserverLockedAndSamplesOnlyMovableBodies(t *testing.T) {
	w := motionStepWorld(t)
	static := motionStepBody(w, StaticBody, Pos{X: 100}, Vec3Zero)
	motionStepSphere(static, 1)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3{X: 1})
	motionStepSphere(body, 1)
	_, err := w.StepMotion(.01, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		if !worlds[w.Index1-1].locked || len(s.Bodies) != 1 || s.Bodies[0].Body != body {
			t.Fatalf("observer contract: locked=%v bodies=%v", worlds[w.Index1-1].locked, s.Bodies)
		}
		return nil
	})
	if err != nil || worlds[w.Index1-1].locked {
		t.Fatalf("world lock not released: %v", err)
	}
}

func TestGolemMotionStepRechecksBrakingPrediction(t *testing.T) {
	w := motionStepWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3{X: 100})
	motionStepSphere(body, .1)
	body.ApplyForceToCenter(Vec3{X: -500 * body.GetMass()}, true)
	wall := motionStepBody(w, KinematicBody, Pos{X: 2.2}, Vec3Zero)
	motionStepSphere(wall, .1)
	r, err := w.StepMotion(.1, DefaultMotionStepOptions(), func(s MotionStepSample) error {
		for _, v := range s.Bodies {
			if v.Body == body && v.End.P.X > 2.01 {
				t.Fatalf("shortened semi-implicit step crossed wall: %+v", v)
			}
		}
		return nil
	})
	if err != nil || r.Elapsed != .1 || r.Microsteps < 2 {
		t.Fatalf("braking admission: %+v %v", r, err)
	}
}

func TestGolemMotionStepRejectsUnexpectedMotorCrossingMesh(t *testing.T) {
	w := motionStepWorld(t)
	floor := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
	sd := DefaultShapeDef()
	floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	body := motionStepBody(w, DynamicBody, Pos{Y: 2}, Vec3Zero)
	motionStepSphere(body, .1)
	anchor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	jd := DefaultMotorJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
	jd.LinearVelocity = Vec3{Y: -100}
	jd.MaxVelocityForce = 1e7
	w.CreateMotorJoint(&jd)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), nil)
	if !errors.Is(err, ErrMotionStepUnsafePath) || r.Microsteps != 1 {
		t.Fatalf("unpredicted motor crossing was accepted: %+v %v body=%v", r, err, body.GetPosition())
	}
}

func TestGolemMotionStepInitialCompoundLeafCannotHideLaterWall(t *testing.T) {
	w := motionStepWorld(t)
	w.EnableContinuous(false)
	obstacle := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	floor, wall := MakeBoxHull(4, .1, 2), MakeBoxHull(.1, 2, 2)
	cd := CompoundDef{HullCount: 2, Hulls: []CompoundHullDef{
		{Hull: &floor.Base, Transform: Transform{P: Vec3{Y: -.1}, Q: QuatIdentity}},
		{Hull: &wall.Base, Transform: Transform{P: Vec3{X: 2}, Q: QuatIdentity}},
	}}
	compound := CreateCompound(&cd)
	if compound == nil {
		t.Fatal("compound cook")
	}
	sd := DefaultShapeDef()
	if !obstacle.CreateBakedCompoundShape(&sd, compound).IsValid() {
		t.Fatal("compound shape")
	}
	body := motionStepBody(w, DynamicBody, Pos{Y: .25}, Vec3Zero)
	motionStepSphere(body, .25)
	anchor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	jd := DefaultMotorJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
	jd.LinearVelocity = Vec3{X: 100}
	jd.MaxVelocityForce = 1e7
	w.CreateMotorJoint(&jd)
	r, err := w.StepMotion(.05, DefaultMotionStepOptions(), nil)
	if !errors.Is(err, ErrMotionStepUnsafePath) || r.Microsteps != 1 {
		t.Fatalf("initial floor masked later wall: %+v %v body=%v", r, err, body.GetPosition())
	}
}

func TestGolemMotionStepFourFixedPartitionsAndRestingFaces(t *testing.T) {
	t.Run("mesh", func(t *testing.T) {
		w := motionStepWorld(t)
		floor := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
		sd := DefaultShapeDef()
		floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
		body := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
		motionStepSphere(body, .5)
		for i := 0; i < 4; i++ {
			if _, err := w.StepMotion(.0125, DefaultMotionStepOptions(), nil); err != nil {
				t.Fatalf("partition %d: %v", i, err)
			}
		}
		if body.GetPosition().Y < 2.45 {
			t.Fatal(body.GetPosition())
		}
	})
	t.Run("resting hull faces", func(t *testing.T) {
		w := motionStepWorld(t)
		w.SetGravity(Vec3{Y: -9.81})
		floor := motionStepBody(w, StaticBody, Pos{Y: -.5}, Vec3Zero)
		sd := DefaultShapeDef()
		flat := MakeBoxHull(4, .5, 4)
		floor.CreateHullShape(&sd, &flat.Base)
		body := motionStepBody(w, DynamicBody, Pos{Y: .505}, Vec3Zero)
		cube := MakeBoxHull(.5, .5, .5)
		body.CreateHullShape(&sd, &cube.Base)
		for i := 0; i < 120; i++ {
			if _, err := w.StepMotion(.0125, DefaultMotionStepOptions(), nil); err != nil {
				t.Fatalf("resting step %d: %v", i, err)
			}
		}
		if body.GetPosition().Y < .49 {
			t.Fatal(body.GetPosition())
		}
	})
}
