package box3d

import (
	"fmt"
	"testing"
)

func continuationWorld(t *testing.T) WorldID {
	t.Helper()
	w := nativeTOIWorld(t, 1)
	if err := w.SetTOIContinuation(true); err != nil {
		t.Fatal(err)
	}
	return w
}
func continuationEnergy(body BodyID) float32 {
	m := body.GetMassData()
	v := body.GetLinearVelocity()
	a := InvRotateVector(body.GetRotation(), body.GetAngularVelocity())
	return .5 * (m.Mass*Dot(v, v) + Dot(a, MulMV(m.Inertia, a)))
}
func continuationStep(t *testing.T, w WorldID) []MotionStepSample {
	t.Helper()
	var samples []MotionStepSample
	err := w.StepObserved(1.0/80, DefaultObservedStepOptions(), func(sample MotionStepSample) error { samples = append(samples, sample); return nil })
	if err != nil {
		t.Fatal(err)
	}
	elapsed, last := float32(0), float32(0)
	for i, s := range samples {
		if s.ImpulseDuration != 1.0/80 || s.Delta < 0 || s.StartFraction < last || s.EndFraction < s.StartFraction {
			t.Fatalf("bad owned sample%d: %+v", i, s)
		}
		if i > 0 && (s.TOIContinuation != (TOIContinuationStats{}) || s.SpeculativeCCD != (SpeculativeCCDStats{})) {
			t.Fatal("repeated whole-step work counters")
		}
		elapsed += s.Delta
		last = s.EndFraction
	}
	if AbsFloat(elapsed-1.0/80) > 1e-7 || last != 1 {
		t.Fatalf("time advanced incorrectly: %g fraction%g", elapsed, last)
	}
	return samples
}
func continuationImpulse(samples []MotionStepSample, shape ShapeID) Vec3 {
	var impulse Vec3
	for _, sample := range samples {
		for _, c := range append(append([]ContactData(nil), sample.Contacts...), sample.ImpulseContacts...) {
			sign := float32(0)
			if c.ShapeIdA == shape {
				sign = -1
			} else if c.ShapeIdB == shape {
				sign = 1
			}
			if sign == 0 {
				continue
			}
			for _, m := range c.Manifolds[:c.ManifoldCount] {
				for _, p := range m.Points[:m.PointCount] {
					impulse = MulAdd(impulse, sign*p.TotalNormalImpulse, m.Normal)
				}
			}
		}
	}
	return impulse
}
func TestNativeContinuationSupportedPlate(t *testing.T) {
	for _, mass := range []float32{.1, 1} {
		t.Run(fmt.Sprint(mass), func(t *testing.T) {
			w := continuationWorld(t)
			ball, sphere, plate, ps := nativeTOIPair(w, DynamicBody, 0, 0)
			nativeTOIMass(plate, mass)
			d := DefaultBodyDef()
			d.Position.X = .12
			wall := w.CreateBody(&d)
			sd := DefaultShapeDef()
			sd.BaseMaterial.Friction = 0
			h := MakeBoxHull(.1, 2, 2)
			ws := wall.CreateHullShape(&sd, &h.Base)
			initial := continuationEnergy(ball) + continuationEnergy(plate)
			maximum := float32(0)
			for i := 0; i < 240; i++ {
				continuationStep(t, w)
				depth, ok := ps.PairPenetration(ws)
				other, ok2 := sphere.PairPenetration(ws)
				maximum = MaxFloat(maximum, depth.Depth)
				if !ok || !ok2 || depth.Depth > LinearSlop()+1e-5 || other.Depth > LinearSlop()+1e-5 || plate.GetPosition().X > .02 || ball.GetPosition().X > .01 || continuationEnergy(ball)+continuationEnergy(plate) > initial*1.00001 {
					t.Fatalf("step%d plate%v ball%v depth%g/%g energy%g", i, plate.GetPosition(), ball.GetPosition(), depth.Depth, other.Depth, continuationEnergy(ball)+continuationEnergy(plate))
				}
			}
			t.Logf("maximum support depth %g", maximum)
		})
	}
}
func TestNativeContinuationMomentumRestitution(t *testing.T) {
	for _, typ := range []BodyType{StaticBody, DynamicBody} {
		for _, restitution := range []float32{0, .7} {
			t.Run(fmt.Sprintf("type%d/e%g", typ, restitution), func(t *testing.T) {
				w := continuationWorld(t)
				ball, shape, target, _ := nativeTOIPair(w, typ, restitution, 0)
				if typ == DynamicBody {
					nativeTOIMass(target, 1)
				}
				initial := continuationEnergy(ball) + continuationEnergy(target)
				observedLocal := false
				for i := 0; i < 12; i++ {
					before := ball.GetLinearVelocity()
					samples := continuationStep(t, w)
					impulse := continuationImpulse(samples, shape)
					if w.GetTOIContinuationStats().Events > 0 {
						for _, sample := range samples {
							observedLocal = observedLocal || len(sample.ImpulseContacts) > 0
							for _, c := range sample.Contacts {
								for _, m := range c.Manifolds {
									for _, p := range m.Points[:m.PointCount] {
										if p.TotalNormalImpulse != 0 {
											t.Fatal("geometry repeated local impulse")
										}
									}
								}
							}
						}
					}
					if residual := Sub(Sub(ball.GetLinearVelocity(), before), impulse); Length(residual) > .01 {
						t.Fatalf("step%d impulse accounting %v", i, residual)
					}
					if continuationEnergy(ball)+continuationEnergy(target) > initial*1.00001 {
						t.Fatal("energy increased")
					}
					if typ == DynamicBody && AbsFloat(ball.GetLinearVelocity().X+target.GetLinearVelocity().X-400) > .02 {
						t.Fatal("pair momentum changed")
					}
				}
				if !observedLocal {
					t.Fatal("fixture did not exercise owned local impulse samples")
				}
				if AbsFloat(target.GetLinearVelocity().X-ball.GetLinearVelocity().X-restitution*400) > .1 {
					t.Fatal("restitution velocity")
				}
			})
		}
	}
}
func TestNativeContinuationForceDuration(t *testing.T) {
	for _, gravity := range []bool{false, true} {
		t.Run(fmt.Sprint(gravity), func(t *testing.T) {
			w := continuationWorld(t)
			d := DefaultBodyDef()
			wall := w.CreateBody(&d)
			sd := DefaultShapeDef()
			sd.BaseMaterial.Friction = 0
			h := MakeBoxHull(.01, 10, 10)
			wall.CreateHullShape(&sd, &h.Base)
			d.Type = DynamicBody
			d.Position.X = -.115
			d.SafetyFactor = .01
			body := w.CreateBody(&d)
			shape := body.CreateSphereShape(&sd, &Sphere{Radius: .1})
			nativeTOIMass(body, 1)
			if gravity {
				w.SetGravity(Vec3{X: 1000})
			}
			for i := 0; i < 8; i++ {
				before := body.GetLinearVelocity()
				if !gravity {
					body.ApplyForceToCenter(Vec3{X: 1000}, true)
				}
				samples := continuationStep(t, w)
				expected := Add(continuationImpulse(samples, shape), Vec3{X: 1000.0 / 80})
				if residue := Sub(Sub(body.GetLinearVelocity(), before), expected); Length(residue) > .001 {
					t.Fatalf("step%d external impulse %v", i, residue)
				}
				if body.GetPosition().X > -.105 {
					t.Fatal("wall containment")
				}
			}
		})
	}
}
func TestNativeContinuationMotorSupports(t *testing.T) {
	for _, mesh := range []bool{false, true} {
		t.Run(fmt.Sprint(mesh), func(t *testing.T) {
			w := continuationWorld(t)
			sd := DefaultShapeDef()
			d := DefaultBodyDef()
			var body BodyID
			if mesh {
				d.Type = KinematicBody
				floor := w.CreateBody(&d)
				floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
				d.Type = DynamicBody
				d.Position = Pos{Y: .5}
				body = w.CreateBody(&d)
				body.CreateSphereShape(&sd, &Sphere{Radius: .1})
			} else {
				obstacle := w.CreateBody(&d)
				floor, wall := MakeOffsetBoxHull(4, .1, 2, Vec3{Y: -.1}), MakeOffsetBoxHull(.1, 2, 2, Vec3{X: 2})
				obstacle.CreateHullShape(&sd, &floor.Base)
				obstacle.CreateHullShape(&sd, &wall.Base)
				d.Type = DynamicBody
				d.Position = Pos{Y: .25}
				body = w.CreateBody(&d)
				body.CreateSphereShape(&sd, &Sphere{Radius: .25})
			}
			d = DefaultBodyDef()
			anchor := w.CreateBody(&d)
			jd := DefaultMotorJointDef()
			jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
			jd.LinearVelocity = Vec3{X: 100}
			if mesh {
				jd.LinearVelocity = Vec3{Y: -100}
			}
			jd.MaxVelocityForce = 1e7
			w.CreateMotorJoint(&jd)
			for i := 0; i < 4; i++ {
				continuationStep(t, w)
				p := body.GetPosition()
				if mesh && p.Y < .09 || !mesh && p.X > 1.66 {
					t.Fatalf("motor escaped step%d pose%v", i, p)
				}
			}
		})
	}
}

func TestNativeContinuationLifecycle(t *testing.T) {
	t.Run("presolve-rejection", func(t *testing.T) {
		w := continuationWorld(t)
		body, shape, _, wall := nativeTOIPair(w, StaticBody, 0, 0)
		wall.EnablePreSolveEvents(true)
		calls := 0
		w.SetPreSolveCallback(func(ShapeID, ShapeID, Pos, Vec3, any) bool { calls++; return false }, nil)
		samples := continuationStep(t, w)
		if calls == 0 || body.GetPosition().X < 1 || continuationImpulse(samples, shape) != Vec3Zero || w.GetTOIContinuationStats().Events != 0 {
			t.Fatalf("rejected impact response: calls%d pose%v impulse%v", calls, body.GetPosition(), continuationImpulse(samples, shape))
		}
	})
	t.Run("disabled-joint-endpoint", func(t *testing.T) {
		w := continuationWorld(t)
		_, _, target, _ := nativeTOIPair(w, DynamicBody, 0, 0)
		d := DefaultBodyDef()
		d.Type = DynamicBody
		d.Position = Pos{Y: 20}
		disabled := w.CreateBody(&d)
		sd := DefaultShapeDef()
		disabled.CreateSphereShape(&sd, &Sphere{Radius: .1})
		jd := DefaultDistanceJointDef()
		jd.Base.BodyIdA = target
		jd.Base.BodyIdB = disabled
		w.CreateDistanceJoint(&jd)
		disabled.Disable()
		before := disabled.GetTransform()
		continuationStep(t, w)
		native := getWorldFromId(w)
		if native.bodies[disabled.Index1-1].setIndex != disabledSet || disabled.GetTransform() != before {
			t.Fatal("disabled participant changed")
		}
	})
	t.Run("sleeping-kinematic", func(t *testing.T) {
		w := continuationWorld(t)
		body, _, target, _ := nativeTOIPair(w, KinematicBody, 0, 0)
		target.SetAwake(false)
		continuationStep(t, w)
		if body.GetPosition().X > -.01 || body.GetLinearVelocity().X > .01 {
			t.Fatalf("sleeping kinematic missed: %v %v", body.GetPosition(), body.GetLinearVelocity())
		}
	})
	t.Run("bullet-all-leaves", func(t *testing.T) {
		w := continuationWorld(t)
		d := DefaultBodyDef()
		wall := w.CreateBody(&d)
		sd := DefaultShapeDef()
		h := MakeBoxHull(.01, 2, 2)
		wall.CreateHullShape(&sd, &h.Base)
		d.Type = DynamicBody
		d.IsBullet = true
		d.Position = Pos{X: -11}
		d.LinearVelocity = Vec3{X: 160}
		body := w.CreateBody(&d)
		body.CreateSphereShape(&sd, &Sphere{Radius: 10})
		body.CreateSphereShape(&sd, &Sphere{Center: Vec3{Y: -20}, Radius: .01})
		continuationStep(t, w)
		if body.GetPosition().X > -10 || body.GetLinearVelocity().X > .01 {
			t.Fatalf("large leaf missed: %v %v", body.GetPosition(), body.GetLinearVelocity())
		}
	})
	t.Run("sleeping-contact-impulse", func(t *testing.T) {
		w := nativeTOIWorld(t, 1)
		d := DefaultBodyDef()
		floor := w.CreateBody(&d)
		sd := DefaultShapeDef()
		h := MakeBoxHull(2, .1, 2)
		floor.CreateHullShape(&sd, &h.Base)
		d.Type = DynamicBody
		d.Position = Pos{Y: .6}
		body := w.CreateBody(&d)
		shape := body.CreateSphereShape(&sd, &Sphere{Radius: .5})
		w.SetGravity(Vec3{Y: -10})
		w.Step(1.0/80, 1)
		w.Step(1.0/80, 1)
		body.SetAwake(false)
		native := getWorldFromId(w)
		rows := 0
		for i := range native.contacts {
			c := &native.contacts[i]
			if c.contactId == NullIndex {
				continue
			}
			for mi := range c.manifolds {
				for pi := range c.manifolds[mi].Points[:c.manifolds[mi].PointCount] {
					c.manifolds[mi].Points[pi].TotalNormalImpulse = 7
					rows++
				}
			}
		}
		samples := continuationStep(t, w)
		if rows == 0 || len(samples[0].Contacts) == 0 || continuationImpulse(samples, shape) != Vec3Zero {
			t.Fatalf("sleeping impulse replay: rows%d contacts%d impulse%v", rows, len(samples[0].Contacts), continuationImpulse(samples, shape))
		}
	})
}
