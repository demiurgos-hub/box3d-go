// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"fmt"
	"testing"
)

func speculativeGeometryStep(t *testing.T, w WorldID, dt float32, substeps int) {
	t.Helper()
	w.Step(dt, substeps)
	if err := w.SpeculativeCCDError(); err != nil {
		t.Fatal(err)
	}
}

func TestGolemSpeculativeGeometrySleepingTarget(t *testing.T) {
	for _, mesh := range []bool{false, true} {
		t.Run(fmt.Sprintf("mesh%v", mesh), func(t *testing.T) {
			d := DefaultWorldDef()
			d.Gravity = Vec3Zero
			w := CreateWorld(&d)
			t.Cleanup(w.Destroy)
			if err := w.SetSpeculativeCCD(true); err != nil {
				t.Fatal(err)
			}
			mover := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
			if mesh {
				sd := DefaultShapeDef()
				mover.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
			} else {
				motionStepSphere(mover, .2)
			}
			db := DefaultBodyDef()
			db.Type, db.IsAwake = DynamicBody, false
			target := w.CreateBody(&db)
			motionStepSphere(target, .2)
			if target.IsAwake() {
				t.Fatal("target did not begin asleep")
			}
			speculativeGeometryStep(t, w, .05, 1)
			if !target.IsAwake() || target.GetPosition().Y < 2.15 || target.GetLinearVelocity().Y < 79 {
				t.Fatalf("sleeping target was not linked and solved: awake=%v p=%v v=%v", target.IsAwake(), target.GetPosition(), target.GetLinearVelocity())
			}
		})
	}
}

func TestGolemSpeculativeGeometryNearMissAndSeparating(t *testing.T) {
	for _, tc := range []struct {
		name   string
		a, b   Pos
		va, vb Vec3
	}{
		{"finite-near-miss", Pos{X: -2}, Pos{X: 2, Y: 1.05}, Vec3{X: 80}, Vec3{X: -80}},
		{"separating", Pos{X: -.6}, Pos{X: .6}, Vec3{X: -80}, Vec3{X: 80}},
		{"common-translation", Pos{X: -.6}, Pos{X: .6}, Vec3{X: 80}, Vec3{X: 80}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			w := speculativeWorld(t)
			a := motionStepBody(w, DynamicBody, tc.a, tc.va)
			b := motionStepBody(w, DynamicBody, tc.b, tc.vb)
			motionStepSphere(a, .5)
			motionStepSphere(b, .5)
			speculativeGeometryStep(t, w, .05, 1)
			if a.GetLinearVelocity() != tc.va || b.GetLinearVelocity() != tc.vb || w.GetContactEvents().BeginCount != 0 {
				t.Fatalf("clear path created response/events: a=%v b=%v events=%+v", a.GetLinearVelocity(), b.GetLinearVelocity(), w.GetContactEvents())
			}
		})
	}
}

func TestGolemSpeculativeGeometryMeshBackface(t *testing.T) {
	w := speculativeWorld(t)
	mover := motionStepBody(w, KinematicBody, Pos{Y: 2}, Vec3{Y: -80})
	sd := DefaultShapeDef()
	sd.EnableContactEvents = true
	mover.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	target := motionStepBody(w, DynamicBody, Pos{}, Vec3Zero)
	motionStepSphere(target, .2)
	speculativeGeometryStep(t, w, .05, 1)
	if target.GetPosition() != (Pos{}) || target.GetLinearVelocity() != Vec3Zero || w.GetContactEvents().BeginCount != 0 {
		t.Fatalf("back-facing mesh made a predictive constraint: p=%v v=%v events=%+v", target.GetPosition(), target.GetLinearVelocity(), w.GetContactEvents())
	}
}

func TestGolemSpeculativeGeometryTranslatedLocalCenters(t *testing.T) {
	for _, offset := range []float32{0, 768, -2304, 4090} {
		for _, localCenter := range []Vec3{{}, {.35, -.2, .15}} {
			t.Run(fmt.Sprintf("offset%g/center%v", offset, localCenter), func(t *testing.T) {
				w := speculativeWorld(t)
				rotation := MakeQuatFromAxisAngle(Normalize(Vec3{1, 2, 3}), .7)
				create := func(x, speed float32) BodyID {
					d := DefaultBodyDef()
					d.Type, d.Rotation = DynamicBody, rotation
					d.Position = ToPos(Sub(Vec3{offset + x, offset, offset}, RotateVector(rotation, localCenter)))
					d.LinearVelocity = Vec3{X: speed}
					body := w.CreateBody(&d)
					sd := DefaultShapeDef()
					body.CreateSphereShape(&sd, &Sphere{Center: localCenter, Radius: .05})
					return body
				}
				a, b := create(-1, 160), create(1, -160)
				speculativeGeometryStep(t, w, .0125, 1)
				ca, cb := a.GetWorldCenter(), b.GetWorldCenter()
				if ca.X >= cb.X || AbsFloat(a.GetLinearVelocity().X+b.GetLinearVelocity().X) > .001 || Length(a.GetAngularVelocity()) > .005 || Length(b.GetAngularVelocity()) > .005 {
					t.Fatalf("translated/off-center collision lost symmetry: centers=%v/%v v=%v/%v w=%v/%v", ca, cb, a.GetLinearVelocity(), b.GetLinearVelocity(), a.GetAngularVelocity(), b.GetAngularVelocity())
				}
			})
		}
	}
}

func TestGolemSpeculativeGeometryPredictorMatchesUnconstrainedNativeIntegration(t *testing.T) {
	for _, typ := range []BodyType{DynamicBody, KinematicBody} {
		for _, substeps := range []int{1, 4, 8} {
			t.Run(fmt.Sprintf("type%d/substeps%d", typ, substeps), func(t *testing.T) {
				w := speculativeWorld(t)
				w.SetGravity(Vec3{X: 1, Y: -9.81, Z: -2})
				d := DefaultBodyDef()
				d.Type, d.Position = typ, Pos{X: 768, Y: 30, Z: -2304}
				d.Rotation = MakeQuatFromAxisAngle(Normalize(Vec3{1, 2, 3}), .7)
				d.LinearVelocity, d.AngularVelocity = Vec3{X: 70, Y: -30, Z: 20}, Vec3{X: 12, Y: -5, Z: 3}
				d.LinearDamping, d.AngularDamping, d.GravityScale = .3, .2, 1.7
				d.MotionLocks = MotionLocks{LinearZ: true, AngularY: true}
				bodyID := w.CreateBody(&d)
				sd := DefaultShapeDef()
				box := MakeBoxHull(.7, .2, .1)
				bodyID.CreateHullShape(&sd, &box.Base)
				bodyID.ApplyForceToCenter(Vec3{X: 100, Y: 40, Z: 50}, true)
				bodyID.ApplyTorque(Vec3{X: 1, Y: 2, Z: 3}, true)
				world := getWorldFromId(w)
				body := &world.bodies[bodyID.Index1-1]
				predicted := predictiveBodySweep(world, body, .025, substeps)
				speculativeGeometryStep(t, w, .025, substeps)
				actual := getBodySim(world, body)
				if Distance(predicted.C2, ToVec3(actual.center)) > 1e-6 || AbsFloat(DotQuat(predicted.Q2, actual.transform.Q)) < 1-1e-6 {
					t.Fatalf("prediction differs from unconstrained integration: predicted=%+v actualCenter=%v actualRotation=%v", predicted, actual.center, actual.transform.Q)
				}
			})
		}
	}
}

func TestGolemSpeculativeGeometryMeshFeaturesAndMaterial(t *testing.T) {
	for _, center := range []Vec3{{X: -1, Z: -1}, {X: 1, Z: 1}} {
		t.Run(fmt.Sprint(center), func(t *testing.T) {
			w := speculativeWorld(t)
			meshBody := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
			sd := DefaultShapeDef()
			sd.Materials = []SurfaceMaterial{{Friction: .1, Restitution: .2, UserMaterialId: 100}, {Friction: .9, Restitution: .7, UserMaterialId: 101}}
			sd.MaterialCount, sd.EnableHitEvents = 2, true
			meshBody.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
			target := motionStepBody(w, DynamicBody, ToPos(center), Vec3Zero)
			targetShape := motionStepSphere(target, .2)
			targetShape.SetSurfaceMaterial(SurfaceMaterial{Friction: .4, Restitution: .1, UserMaterialId: 200})
			speculativeGeometryStep(t, w, .05, 1)
			world := getWorldFromId(w)
			points := 0
			for _, p := range world.speculativeCCD.contacts {
				if p.contactID == NullIndex {
					continue
				}
				c := &world.contacts[p.contactID]
				for _, m := range c.manifolds {
					for j := int32(0); j < m.PointCount; j++ {
						point := m.Points[j]
						points++
						if point.TriangleIndex < 0 || point.TriangleIndex > 1 {
							t.Fatalf("lost triangle identity: %+v", point)
						}
						material := sd.Materials[GetMeshMaterialIndices(world.shapes[c.shapeIdA].mesh.Data)[point.TriangleIndex]]
						if AbsFloat(c.friction-sqrtf(material.Friction*.4)) > 1e-6 || c.restitution != material.Restitution {
							t.Fatalf("wrong triangle material: triangle=%d friction=%g restitution=%g material=%+v", point.TriangleIndex, c.friction, c.restitution, material)
						}
					}
				}
			}
			if points == 0 {
				t.Fatal("no predictive contact witnesses")
			}
		})
	}
}

func TestGolemSpeculativeGeometryBlockedPredictionIsNotInteraction(t *testing.T) {
	w := speculativeWorld(t)
	anchor := motionStepBody(w, StaticBody, Pos{X: -2}, Vec3Zero)
	a := motionStepBody(w, DynamicBody, Pos{X: -1}, Vec3{X: 160})
	b := motionStepBody(w, DynamicBody, Pos{X: 1}, Vec3Zero)
	motionStepSphere(a, .1)
	motionStepSphere(b, .1)
	jd := DefaultDistanceJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB, jd.Length = anchor, a, 1
	w.CreateDistanceJoint(&jd)
	speculativeGeometryStep(t, w, .025, 1)
	world := getWorldFromId(w)
	constraints, interactions := 0, 0
	for _, p := range world.speculativeCCD.contacts {
		if p.contactID == NullIndex {
			continue
		}
		for _, m := range world.contacts[p.contactID].manifolds {
			for j := int32(0); j < m.PointCount; j++ {
				constraints++
				point := m.Points[j]
				// This is the same physical-interaction predicate used by the
				// portable adapter; native BeginTouch describes constraint lifetime.
				if point.Separation <= 0 || point.TotalNormalImpulse > 0 {
					interactions++
				}
			}
		}
	}
	if constraints == 0 || interactions != 0 || b.GetLinearVelocity() != Vec3Zero || w.GetContactEvents().HitCount != 0 {
		t.Fatalf("blocked prediction became physical interaction: constraints=%d interactions=%d targetVelocity=%v events=%+v", constraints, interactions, b.GetLinearVelocity(), w.GetContactEvents())
	}
}

func TestGolemSpeculativeGeometryInitialOverlapMatchesNative(t *testing.T) {
	create := func(enabled bool) (WorldID, BodyID, BodyID) {
		w := motionStepWorld(t)
		if err := w.SetSpeculativeCCD(enabled); err != nil {
			t.Fatal(err)
		}
		a := motionStepBody(w, DynamicBody, Pos{X: -.4}, Vec3{X: -30})
		b := motionStepBody(w, DynamicBody, Pos{X: .4}, Vec3{X: 30})
		motionStepSphere(a, .5)
		motionStepSphere(b, .5)
		return w, a, b
	}
	native, na, nb := create(false)
	extended, ea, eb := create(true)
	for i := 0; i < 3; i++ {
		speculativeGeometryStep(t, native, .0125, 1)
		speculativeGeometryStep(t, extended, .0125, 1)
		if na.GetTransform() != ea.GetTransform() || nb.GetTransform() != eb.GetTransform() || na.GetLinearVelocity() != ea.GetLinearVelocity() || nb.GetLinearVelocity() != eb.GetLinearVelocity() {
			t.Fatal("separating initial overlap changed native resolution")
		}
	}
}

func TestGolemSpeculativeGeometryFiltersAndSensors(t *testing.T) {
	for _, sensor := range []bool{false, true} {
		t.Run(fmt.Sprintf("sensor%v", sensor), func(t *testing.T) {
			w := speculativeWorld(t)
			a := motionStepBody(w, DynamicBody, Pos{X: -1}, Vec3{X: 160})
			b := motionStepBody(w, DynamicBody, Pos{X: 1}, Vec3{X: -160})
			sd := DefaultShapeDef()
			sd.IsSensor, sd.EnableContactEvents = sensor, true
			if !sensor {
				sd.Filter.MaskBits = 0
			}
			a.CreateSphereShape(&sd, &Sphere{Radius: .05})
			motionStepSphere(b, .05)
			speculativeGeometryStep(t, w, .0125, 1)
			if a.GetLinearVelocity().X != 160 || b.GetLinearVelocity().X != -160 || w.GetContactEvents().BeginCount != 0 || len(getWorldFromId(w).speculativeCCD.contacts) != 0 {
				t.Fatal("filtered/sensor shape entered the solid predictive solver")
			}
		})
	}
}

func TestGolemSpeculativeGeometryReusedContactSeparatesCleanly(t *testing.T) {
	w := speculativeWorld(t)
	a := motionStepBody(w, DynamicBody, Pos{X: -1}, Vec3{X: 160, Y: 10})
	b := motionStepBody(w, DynamicBody, Pos{X: 1}, Vec3{X: -160, Y: -10})
	motionStepSphere(a, .2)
	motionStepSphere(b, .2)
	speculativeGeometryStep(t, w, .0125, 1)
	if len(getWorldFromId(w).speculativeCCD.contacts) == 0 {
		t.Fatal("test did not create a predictive contact")
	}
	a.SetLinearVelocity(Vec3{X: -50})
	b.SetLinearVelocity(Vec3{X: 50})
	a.SetAngularVelocity(Vec3Zero)
	b.SetAngularVelocity(Vec3Zero)
	for i := 0; i < 3; i++ {
		speculativeGeometryStep(t, w, .0125, 1)
		if a.GetLinearVelocity() != (Vec3{X: -50}) || b.GetLinearVelocity() != (Vec3{X: 50}) || len(getWorldFromId(w).speculativeCCD.contacts) != 0 {
			t.Fatalf("predictive cache/warm-start obstructed separating pair: a=%v b=%v", a.GetLinearVelocity(), b.GetLinearVelocity())
		}
	}
}

func TestGolemSpeculativeGeometryRotatedManualCOMAnchors(t *testing.T) {
	for _, angle := range []float32{-.7, 0, .7} {
		t.Run(fmt.Sprint(angle), func(t *testing.T) {
			w := speculativeWorld(t)
			q := MakeQuatFromAxisAngle(Vec3AxisZ, angle)
			d := DefaultBodyDef()
			d.Type, d.Rotation, d.AngularVelocity = KinematicBody, q, Vec3{Z: 20}
			meshBody := w.CreateBody(&d)
			sd := DefaultShapeDef()
			meshBody.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
			d = DefaultBodyDef()
			d.Type, d.Rotation = DynamicBody, q
			d.Position = ToPos(RotateVector(q, Vec3{X: 1.5, Y: .5}))
			body := w.CreateBody(&d)
			motionStepSphere(body, .2)
			mass := body.GetMassData()
			mass.Center = Vec3{X: .07, Y: .03}
			body.SetMassData(mass)
			speculativeGeometryStep(t, w, .025, 1)
			world := getWorldFromId(w)
			points := 0
			for _, p := range world.speculativeCCD.contacts {
				if p.contactID == NullIndex {
					continue
				}
				c := &world.contacts[p.contactID]
				a, b := p.a, p.b
				if a.id != c.shapeIdA {
					a, b = b, a
				}
				for _, m := range c.manifolds {
					for j := int32(0); j < m.PointCount; j++ {
						point := m.Points[j]
						points++
						localA := Add(a.sweep.LocalCenter, InvRotateVector(a.sweep.Q1, point.AnchorA))
						localB := Add(b.sweep.LocalCenter, InvRotateVector(b.sweep.Q1, point.AnchorB))
						// Native contact points are surface midpoints and may differ
						// from either surface by half the native TOI slop.
						if AbsFloat(localA.Y) > LinearSlop() || AbsFloat(Length(localB)-.2) > LinearSlop() {
							t.Fatalf("COM/rotation conversion left authored geometry: localA=%v localB=%v point=%+v", localA, localB, point)
						}
					}
				}
			}
			if points == 0 {
				t.Fatal("test did not create a rotating predictive contact")
			}
		})
	}
}

func TestGolemSpeculativeGeometryPreSolveUsesWorldPoint(t *testing.T) {
	for _, offset := range []float32{0, 2304, -2304} {
		t.Run(fmt.Sprint(offset), func(t *testing.T) {
			w := speculativeWorld(t)
			a := motionStepBody(w, DynamicBody, Pos{offset - 1, offset, offset}, Vec3{X: 160})
			b := motionStepBody(w, DynamicBody, Pos{offset + 1, offset, offset}, Vec3{X: -160})
			sa := motionStepSphere(a, .05)
			sb := motionStepSphere(b, .05)
			sa.EnablePreSolveEvents(true)
			calls := 0
			w.SetPreSolveCallback(func(idA, idB ShapeID, point Pos, normal Vec3, _ any) bool {
				calls++
				if idA != sa || idB != sb || Distance(ToVec3(point), Vec3{offset, offset, offset}) > LinearSlop() || Distance(normal, Vec3AxisX) > 1e-6 {
					t.Errorf("callback lost world origin/shape identity: shapes=%v/%v point=%v normal=%v", idA, idB, point, normal)
				}
				return true
			}, nil)
			speculativeGeometryStep(t, w, .0125, 1)
			if calls != 1 {
				t.Fatalf("expected one predictive pre-solve callback, got %d", calls)
			}
		})
	}
}

func TestGolemSpeculativeGeometryPerpendicularCrossing(t *testing.T) {
	w := speculativeWorld(t)
	a := motionStepBody(w, DynamicBody, Pos{X: -1}, Vec3{X: 80})
	b := motionStepBody(w, DynamicBody, Pos{Y: -1}, Vec3{Y: 80})
	motionStepSphere(a, .05)
	motionStepSphere(b, .05)
	speculativeGeometryStep(t, w, .025, 1)
	separation := SubPos(b.GetWorldCenter(), a.GetWorldCenter())
	momentum := Add(a.GetLinearVelocity(), b.GetLinearVelocity())
	if Dot(separation, Vec3{X: 1, Y: -1}) <= 0 || Distance(momentum, Vec3{X: 80, Y: 80}) > .001 || a.GetLinearVelocity().Y <= 0 || b.GetLinearVelocity().X <= 0 {
		t.Fatalf("relative perpendicular paths missed physical response: separation=%v va=%v vb=%v", separation, a.GetLinearVelocity(), b.GetLinearVelocity())
	}
}

func TestGolemSpeculativeGeometryMovingMeshAndVisitor(t *testing.T) {
	w := speculativeWorld(t)
	mover := motionStepBody(w, KinematicBody, Pos{Y: -2}, Vec3{Y: 80})
	sd := DefaultShapeDef()
	mover.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	visitor := motionStepBody(w, DynamicBody, Pos{X: 1, Z: 1}, Vec3{Y: -80})
	motionStepSphere(visitor, .2)
	speculativeGeometryStep(t, w, .025, 1)
	if visitor.GetPosition().Y < mover.GetPosition().Y+.19 || visitor.GetLinearVelocity().Y < 79 {
		t.Fatalf("future visitor bounds lost moving mesh contact: mesh=%v visitor=%v velocity=%v", mover.GetPosition(), visitor.GetPosition(), visitor.GetLinearVelocity())
	}
}

func TestGolemSpeculativeGeometryWorkerRepeatability(t *testing.T) {
	type state struct {
		pose            WorldTransform
		linear, angular Vec3
	}
	var expected []state
	for _, workers := range []int{1, 2, 4, 8} {
		t.Run(fmt.Sprintf("workers%d", workers), func(t *testing.T) {
			d := DefaultWorldDef()
			d.WorkerCount, d.Gravity, d.EnableSleep = uint32(workers), Vec3Zero, false
			w := CreateWorld(&d)
			t.Cleanup(w.Destroy)
			if err := w.SetSpeculativeCCD(true); err != nil {
				t.Fatal(err)
			}
			bodies := make([]BodyID, 0, 256)
			for i := 0; i < 128; i++ {
				for _, side := range []float32{-1, 1} {
					b := motionStepBody(w, DynamicBody, Pos{X: side, Y: 3 * float32(i)}, Vec3{X: -160 * side})
					motionStepSphere(b, .05)
					bodies = append(bodies, b)
				}
			}
			for step := 0; step < 4; step++ {
				speculativeGeometryStep(t, w, .0125, 1)
			}
			for i, b := range bodies {
				got := state{b.GetTransform(), b.GetLinearVelocity(), b.GetAngularVelocity()}
				if workers == 1 {
					expected = append(expected, got)
				} else if got != expected[i] {
					t.Fatalf("worker ordering changed body%d: got=%+v expected=%+v", i, got, expected[i])
				}
			}
		})
	}
}
