// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"fmt"
	"testing"
)

// Native outcome migrations retain the physical fixtures from the retired
// global admission tests. Each call advances one fixed 80 Hz native interval.
func nativeOutcomeSteps(t *testing.T, w WorldID, count int, before func(), observe func(MotionStepSample) error) {
	t.Helper()
	for step := 0; step < count; step++ {
		if before != nil {
			before()
		}
		if err := w.StepObserved(.0125, DefaultObservedStepOptions(), observe); err != nil {
			t.Fatalf("native interval %d: %v", step, err)
		}
	}
}

func TestGolemNativeOutcomeBrakingAtObstacle(t *testing.T) {
	w := speculativeWorld(t)
	body := motionStepBody(w, DynamicBody, Pos{}, Vec3{X: 100})
	motionStepSphere(body, .1)
	wall := motionStepBody(w, KinematicBody, Pos{X: 2.2}, Vec3Zero)
	motionStepSphere(wall, .1)
	nativeOutcomeSteps(t, w, 8, func() {
		body.ApplyForceToCenter(Vec3{X: -500 * body.GetMass()}, true)
	}, func(s MotionStepSample) error {
		for _, sample := range s.Bodies {
			if sample.Body == body && sample.End.P.X > 2.01 {
				t.Fatalf("braking body crossed obstacle: pose=%v velocity=%v", sample.End, sample.LinearVelocity)
			}
		}
		return nil
	})
}

func TestGolemNativeOutcomeInitialPenetrationRecovers(t *testing.T) {
	w := speculativeWorld(t)
	floor := motionStepBody(w, StaticBody, Pos{Y: -.5}, Vec3Zero)
	sd := DefaultShapeDef()
	box := MakeBoxHull(2, .5, 2)
	floor.CreateHullShape(&sd, &box.Base)
	body := motionStepBody(w, DynamicBody, Pos{Y: .25}, Vec3Zero)
	motionStepSphere(body, .5)
	nativeOutcomeSteps(t, w, 4, nil, nil)
	if body.GetPosition().Y <= .25 {
		t.Fatalf("initial penetration did not recover: %v", body.GetPosition())
	}
}

func TestGolemNativeOutcomeRestingHullFaces(t *testing.T) {
	w := speculativeWorld(t)
	w.SetGravity(Vec3{Y: -9.81})
	floor := motionStepBody(w, StaticBody, Pos{Y: -.5}, Vec3Zero)
	sd := DefaultShapeDef()
	flat := MakeBoxHull(4, .5, 4)
	floor.CreateHullShape(&sd, &flat.Base)
	body := motionStepBody(w, DynamicBody, Pos{Y: .505}, Vec3Zero)
	cube := MakeBoxHull(.5, .5, .5)
	body.CreateHullShape(&sd, &cube.Base)
	nativeOutcomeSteps(t, w, 120, nil, nil)
	if body.GetPosition().Y < .49 {
		t.Fatalf("resting hull lost floor: %v", body.GetPosition())
	}
}

func TestGolemNativeOutcomeRisingSupportBelowCeiling(t *testing.T) {
	w := speculativeWorld(t)
	floor := motionStepBody(w, KinematicBody, Pos{}, Vec3{Y: 2})
	sd := DefaultShapeDef()
	floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
	passenger := motionStepBody(w, DynamicBody, Pos{Y: .505}, Vec3Zero)
	motionStepSphere(passenger, .5)
	ceiling := motionStepBody(w, StaticBody, Pos{Y: 1.3}, Vec3Zero)
	box := MakeBoxHull(2, .1, 2)
	ceiling.CreateHullShape(&sd, &box.Base)
	nativeOutcomeSteps(t, w, 4, nil, func(s MotionStepSample) error {
		for _, sample := range s.Bodies {
			if sample.Body == passenger && sample.End.P.Y > .71 {
				t.Fatalf("passenger crossed ceiling underside: %v", sample.End)
			}
		}
		return nil
	})
	if passenger.GetPosition().Y < floor.GetPosition().Y+.49 || passenger.GetPosition().Y <= .505 {
		t.Fatalf("passenger lost feasible rising support: floor=%v passenger=%v", floor.GetPosition(), passenger.GetPosition())
	}
}

func TestGolemNativeOutcomeMotorCannotCrossKinematicMesh(t *testing.T) {
	for _, startY := range []float32{2, .5} {
		t.Run(fmt.Sprint(startY), func(t *testing.T) {
			w := speculativeWorld(t)
			floor := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
			sd := DefaultShapeDef()
			floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
			body := motionStepBody(w, DynamicBody, Pos{Y: startY}, Vec3Zero)
			motionStepSphere(body, .1)
			anchor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
			jd := DefaultMotorJointDef()
			jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
			jd.LinearVelocity = Vec3{Y: -100}
			jd.MaxVelocityForce = 1e7
			w.CreateMotorJoint(&jd)
			nativeOutcomeSteps(t, w, 4, nil, func(s MotionStepSample) error {
				for _, sample := range s.Bodies {
					if sample.Body == body && sample.End.P.Y < .09 {
						t.Fatalf("solver-accelerated body crossed kinematic mesh: startY=%g pose=%v velocity=%v", startY, sample.End, sample.LinearVelocity)
					}
				}
				return nil
			})
		})
	}
}

func TestGolemNativeOutcomeInitialFloorCannotHideWall(t *testing.T) {
	w := speculativeWorld(t)
	obstacle := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	sd := DefaultShapeDef()
	// Portable compounds are flattened to persistent native child shapes.
	floor, wall := MakeOffsetBoxHull(4, .1, 2, Vec3{Y: -.1}), MakeOffsetBoxHull(.1, 2, 2, Vec3{X: 2})
	obstacle.CreateHullShape(&sd, &floor.Base)
	obstacle.CreateHullShape(&sd, &wall.Base)
	body := motionStepBody(w, DynamicBody, Pos{Y: .25}, Vec3Zero)
	motionStepSphere(body, .25)
	anchor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	jd := DefaultMotorJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
	jd.LinearVelocity = Vec3{X: 100}
	jd.MaxVelocityForce = 1e7
	w.CreateMotorJoint(&jd)
	nativeOutcomeSteps(t, w, 4, nil, func(s MotionStepSample) error {
		for _, sample := range s.Bodies {
			if sample.Body == body && sample.End.P.X > 1.66 {
				t.Fatalf("initial floor masked later wall: pose=%v velocity=%v", sample.End, sample.LinearVelocity)
			}
		}
		return nil
	})
}

func TestGolemNativeOutcomeTinyBullets(t *testing.T) {
	for _, radius := range []float32{.01, .001, .0001} {
		for _, speed := range []float32{100 * radius, 5, 500} {
			t.Run(fmt.Sprintf("radius%g/speed%g", radius, speed), func(t *testing.T) {
				w := speculativeWorld(t)
				gap := max(3*radius, .01*speed)
				a := motionStepBody(w, DynamicBody, Pos{X: -gap}, Vec3{X: speed})
				b := motionStepBody(w, DynamicBody, Pos{X: gap}, Vec3{X: -speed})
				a.SetBullet(true)
				b.SetBullet(true)
				motionStepSphere(a, radius)
				motionStepSphere(b, radius)
				nativeOutcomeSteps(t, w, 4, nil, func(s MotionStepSample) error {
					if a.GetPosition().X >= b.GetPosition().X {
						t.Fatalf("tiny bullets crossed: radius=%g speed=%g a=%v b=%v", radius, speed, a.GetPosition(), b.GetPosition())
					}
					return nil
				})
			})
		}
	}
}
