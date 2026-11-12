package box3d

import (
	"fmt"
	"testing"
)

func nativeTOIWorld(t *testing.T, workers uint32) WorldID {
	t.Helper()
	d := DefaultWorldDef()
	d.Gravity = Vec3Zero
	d.EnableSleep = false
	d.MaximumLinearSpeed = 1000
	d.WorkerCount = workers
	w := CreateWorld(&d)
	t.Cleanup(w.Destroy)
	return w
}

func nativeTOIMass(body BodyID, mass float32) {
	d := body.GetMassData()
	scale := mass / d.Mass
	d.Mass = mass
	d.Inertia.Cx = MulSV(scale, d.Inertia.Cx)
	d.Inertia.Cy = MulSV(scale, d.Inertia.Cy)
	d.Inertia.Cz = MulSV(scale, d.Inertia.Cz)
	body.SetMassData(d)
}

func nativeTOIPair(w WorldID, targetType BodyType, restitution, friction float32) (BodyID, ShapeID, BodyID, ShapeID) {
	d := DefaultBodyDef()
	d.Type = targetType
	target := w.CreateBody(&d)
	sd := DefaultShapeDef()
	sd.BaseMaterial.Restitution = restitution
	sd.BaseMaterial.Friction = friction
	hull := MakeBoxHull(.01, 2, 2)
	wall := target.CreateHullShape(&sd, &hull.Base)
	d.Type = DynamicBody
	d.Position = Pos{X: -2}
	d.IsBullet = targetType != StaticBody
	d.LinearVelocity = Vec3{X: 400}
	ball := w.CreateBody(&d)
	sphere := ball.CreateSphereShape(&sd, &Sphere{Radius: .01})
	nativeTOIMass(ball, 1)
	return ball, sphere, target, wall
}

func nativeTOIFlag(w WorldID, body BodyID) bool {
	world := getWorldFromId(w)
	return getBodySim(world, getBodyFullId(world, body)).flags&hadTimeOfImpact != 0
}

func TestGolemNativeTOIThinWall(t *testing.T) {
	for _, workers := range []uint32{1, 8} {
		for _, dt := range []float32{1.0 / 240, 1.0 / 80} {
			t.Run(fmt.Sprintf("workers%d/dt%g", workers, dt), func(t *testing.T) {
				w := nativeTOIWorld(t, workers)
				ball, sphere, _, wall := nativeTOIPair(w, StaticBody, 0, 0)
				hadTOI := false
				var impulse float32
				for range 12 {
					w.Step(dt, 1)
					hadTOI = hadTOI || nativeTOIFlag(w, ball)
					depth, valid := sphere.PairPenetration(wall)
					if !valid || ball.GetPosition().X > -.01 || depth.Depth > LinearSlop()+1e-5 {
						t.Fatalf("post-TOI escape: x=%g depth=%g valid=%v", ball.GetPosition().X, depth.Depth, valid)
					}
					var contacts [2]ContactData
					for _, c := range contacts[:ball.GetContactData(contacts[:])] {
						for _, m := range c.Manifolds[:c.ManifoldCount] {
							for _, p := range m.Points[:m.PointCount] {
								impulse += p.TotalNormalImpulse
							}
						}
					}
				}
				if !hadTOI || AbsFloat(ball.GetLinearVelocity().X) > 1e-3 || AbsFloat(impulse-400) > .01 {
					t.Fatalf("TOI=%v velocity=%v impulse=%g", hadTOI, ball.GetLinearVelocity(), impulse)
				}
			})
		}
	}
}

func TestGolemNativeTOIBounceFriction(t *testing.T) {
	for _, friction := range []float32{0, .8} {
		t.Run(fmt.Sprintf("friction%g", friction), func(t *testing.T) {
			w := nativeTOIWorld(t, 1)
			ball, _, _, _ := nativeTOIPair(w, StaticBody, .7, friction)
			ball.SetLinearVelocity(Vec3{X: 400, Y: 20})
			hadTOI := false
			for range 12 {
				w.Step(1.0/240, 1)
				hadTOI = hadTOI || nativeTOIFlag(w, ball)
				if ball.GetPosition().X > -.01 {
					t.Fatalf("crossed before bounce: %v", ball.GetPosition())
				}
			}
			v := ball.GetLinearVelocity()
			if !hadTOI || AbsFloat(v.X+280) > .1 {
				t.Fatalf("TOI=%v restitution velocity=%v", hadTOI, v)
			}
			// The native hull witness normal has float32 tangential error; at
			// 400 m/s its normal impulse changes Y by about 0.006 m/s.
			if friction == 0 && AbsFloat(v.Y-20) > .02 || friction > 0 && (v.Y < 0 || v.Y > 19) {
				t.Fatalf("friction response=%v", v)
			}
		})
	}
}

func TestGolemNativeTOIOrdinaryContactsUnchanged(t *testing.T) {
	type state struct {
		position        Pos
		rotation        Quat
		linear, angular Vec3
	}
	run := func(continuous bool) []state {
		d := DefaultWorldDef()
		d.EnableContinuous = continuous
		d.Gravity = Vec3{Y: -10}
		w := CreateWorld(&d)
		defer w.Destroy()
		bd := DefaultBodyDef()
		bd.Position.Y = -.5
		floor := w.CreateBody(&bd)
		sd := DefaultShapeDef()
		hull := MakeBoxHull(10, .5, 10)
		floor.CreateHullShape(&sd, &hull.Base)
		bd.Type = DynamicBody
		bd.Position.Y = .5
		bd.LinearVelocity.X = .2
		body := w.CreateBody(&bd)
		hull = MakeCubeHull(.5)
		body.CreateHullShape(&sd, &hull.Base)
		var states []state
		for range 240 {
			w.Step(1.0/60, 4)
			if nativeTOIFlag(w, body) {
				t.Fatal("ordinary resting fixture unexpectedly used TOI")
			}
			states = append(states, state{body.GetPosition(), body.GetRotation(), body.GetLinearVelocity(), body.GetAngularVelocity()})
		}
		return states
	}
	a, b := run(false), run(true)
	for i := range a {
		if a[i] != b[i] {
			t.Fatalf("ordinary contact changed at %d: %+v / %+v", i, a[i], b[i])
		}
	}
}

func TestGolemNativeTOIDynamicMomentum(t *testing.T) {
	for _, targetMass := range []float32{1, 10, 100} {
		for _, restitution := range []float32{0, .7} {
			t.Run(fmt.Sprintf("mass%g/e%g", targetMass, restitution), func(t *testing.T) {
				w := nativeTOIWorld(t, 8)
				ball, _, target, _ := nativeTOIPair(w, DynamicBody, restitution, 0)
				nativeTOIMass(target, targetMass)
				hadTOI := false
				for range 12 {
					w.Step(1.0/240, 1)
					hadTOI = hadTOI || nativeTOIFlag(w, ball)
					if ball.GetPosition().X > target.GetPosition().X-.01 {
						t.Fatalf("crossed moving target: ball=%v target=%v", ball.GetPosition(), target.GetPosition())
					}
				}
				va, vb := ball.GetLinearVelocity().X, target.GetLinearVelocity().X
				momentum := va + targetMass*vb
				if !hadTOI || AbsFloat(momentum-400) > .02 || AbsFloat((vb-va)-restitution*400) > .1 {
					t.Fatalf("TOI=%v momentum=%g velocities=%g/%g", hadTOI, momentum, va, vb)
				}
			})
		}
	}
}

func TestGolemNativeTOISimultaneousSupport(t *testing.T) {
	w := nativeTOIWorld(t, 8)
	ball, _, _, _ := nativeTOIPair(w, StaticBody, 0, 0)
	ball.SetTransform(Pos{X: -2, Y: .01}, QuatIdentity)
	d := DefaultBodyDef()
	d.Position = Pos{Y: -.1}
	floor := w.CreateBody(&d)
	sd := DefaultShapeDef()
	sd.BaseMaterial.Friction = 0
	hull := MakeBoxHull(5, .1, 5)
	floor.CreateHullShape(&sd, &hull.Base)
	w.SetGravity(Vec3{Y: -10})
	for range 240 {
		w.Step(1.0/240, 1)
		p := ball.GetPosition()
		if p.X > -.01 || p.Y < .0049 || p.Y > .011 {
			t.Fatalf("lost wall/support contact: p=%v v=%v", p, ball.GetLinearVelocity())
		}
	}
	if Length(ball.GetLinearVelocity()) > .001 {
		t.Fatalf("unsettled support: %v", ball.GetLinearVelocity())
	}
}

func TestGolemNativeTOIOverflowContacts(t *testing.T) {
	w := nativeTOIWorld(t, 1)
	ball, _, _, _ := nativeTOIPair(w, StaticBody, 0, 0)
	// A compound wall authored as independent overlapping objects exhausts
	// graph colors and exercises the scalar overflow solver alongside SIMD.
	for range GraphColorCount + 2 {
		d := DefaultBodyDef()
		wall := w.CreateBody(&d)
		sd := DefaultShapeDef()
		hull := MakeBoxHull(.01, 2, 2)
		wall.CreateHullShape(&sd, &hull.Base)
	}
	overflowImpact := false
	for range 12 {
		w.Step(1.0/240, 1)
		for _, c := range getWorldFromId(w).constraintGraph.colors[overflowIndex].contactConstraints {
			overflowImpact = overflowImpact || c.previousTOI
		}
		if ball.GetPosition().X > -.01 {
			t.Fatalf("overflow impact crossed wall: %v", ball.GetPosition())
		}
	}
	if !overflowImpact || AbsFloat(ball.GetLinearVelocity().X) > .001 {
		t.Fatalf("overflow=%v velocity=%v", overflowImpact, ball.GetLinearVelocity())
	}
}

func TestGolemNativeTOIWorkerRepeatability(t *testing.T) {
	var wantStep int
	var wantHash uint32
	for _, workers := range []uint32{1, 2, 4, 8} {
		d := DefaultWorldDef()
		d.WorkerCount = workers
		w := CreateWorld(&d)
		data := createFallingRagdolls(w)
		settled := false
		for range 500 {
			w.Step(1.0/60, 4)
			if updateFallingRagdolls(w, &data) {
				settled = true
				break
			}
		}
		w.Destroy()
		t.Logf("workers=%d sleepStep=%d hash=%08x", workers, data.sleepStep, data.hash)
		if !settled {
			t.Fatalf("workers=%d did not settle", workers)
		}
		if workers == 1 {
			wantStep, wantHash = data.sleepStep, data.hash
		} else if data.sleepStep != wantStep || data.hash != wantHash {
			t.Fatalf("workers=%d differs from single-worker simulation", workers)
		}
		destroyFallingRagdolls(&data)
	}
}
