// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import (
	"fmt"
	"testing"
)

func kinematicGeometryMotorBody(w WorldID, position Pos, velocity Vec3, bullet bool) BodyID {
	body := motionStepBody(w, DynamicBody, position, Vec3Zero)
	body.SetBullet(bullet)
	sd := DefaultShapeDef()
	sd.EnableSensorEvents = true
	body.CreateSphereShape(&sd, &Sphere{Radius: .1})
	anchor := motionStepBody(w, StaticBody, Pos{}, Vec3Zero)
	jd := DefaultMotorJointDef()
	jd.Base.BodyIdA, jd.Base.BodyIdB = anchor, body
	jd.LinearVelocity, jd.MaxVelocityForce = velocity, 1e7
	w.CreateMotorJoint(&jd)
	return body
}

func TestGolemKinematicCCDGeometryBulletTarget(t *testing.T) {
	for _, bullet := range []bool{false, true} {
		for _, offset := range []float32{0, 2304} {
			t.Run(fmt.Sprintf("bullet%v/offset%g", bullet, offset), func(t *testing.T) {
				w := speculativeWorld(t)
				floor := motionStepBody(w, KinematicBody, Pos{Y: offset}, Vec3Zero)
				floor.SetBullet(bullet)
				sd := DefaultShapeDef()
				floor.CreateMeshShape(&sd, golemFlatMesh(), Vec3One)
				body := kinematicGeometryMotorBody(w, Pos{Y: offset + .5}, Vec3{Y: -100}, false)
				if err := w.StepObserved(.0125, DefaultObservedStepOptions(), nil); err != nil {
					t.Fatal(err)
				}
				if body.GetPosition().Y < offset+.09 {
					t.Fatalf("solver-accelerated visitor missed retained kinematic: bullet=%v position=%v", bullet, body.GetPosition())
				}
			})
		}
	}
}

func TestGolemKinematicCCDGeometrySensorTriangle(t *testing.T) {
	for _, extension := range []bool{false, true} {
		for _, hit := range []bool{false, true} {
			t.Run(fmt.Sprintf("extension%v/hit%v", extension, hit), func(t *testing.T) {
				w := motionStepWorld(t)
				if err := w.SetSpeculativeCCD(extension); err != nil {
					t.Fatal(err)
				}
				sensor := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
				sd := DefaultShapeDef()
				sd.IsSensor, sd.EnableSensorEvents = true, true
				mesh := CreateMesh(&MeshDef{Vertices: []Vec3{{-2, 0, -2}, {-2, 0, 2}, {2, 0, -2}}, Indices: []int32{0, 1, 2}, VertexCount: 3, TriangleCount: 1}, nil)
				sensor.CreateMeshShape(&sd, mesh, Vec3One)
				x := float32(1)
				if hit {
					x = -1
				}
				// Native bullet policy supplies the baseline kinematic query. The
				// new extension must retain the same hit/miss geometric distinction.
				visitor := kinematicGeometryMotorBody(w, Pos{x, .5, x}, Vec3{Y: -100}, true)
				if err := w.StepObserved(.0125, DefaultObservedStepOptions(), nil); err != nil {
					t.Fatal(err)
				}
				events := w.GetSensorEvents()
				if (events.BeginCount > 0) != hit || visitor.GetPosition().Y > -.7 {
					t.Fatalf("sensor TOI hit/miss or nonblocking semantics changed: hit=%v events=%+v position=%v", hit, events, visitor.GetPosition())
				}
			})
		}
	}
}

func TestGolemKinematicCCDGeometrySensorInitialOverlap(t *testing.T) {
	for _, extension := range []bool{false, true} {
		t.Run(fmt.Sprintf("extension%v", extension), func(t *testing.T) {
			w := motionStepWorld(t)
			if err := w.SetSpeculativeCCD(extension); err != nil {
				t.Fatal(err)
			}
			sensor := motionStepBody(w, KinematicBody, Pos{}, Vec3Zero)
			sd := DefaultShapeDef()
			sd.IsSensor, sd.EnableSensorEvents = true, true
			sensor.CreateSphereShape(&sd, &Sphere{Radius: .3})
			visitor := kinematicGeometryMotorBody(w, Pos{}, Vec3{X: 100}, true)
			if err := w.StepObserved(.0125, DefaultObservedStepOptions(), nil); err != nil {
				t.Fatal(err)
			}
			if w.GetSensorEvents().BeginCount != 1 || visitor.GetPosition().X < 1.2 {
				t.Fatalf("initial sensor overlap lost entry or blocked visitor: events=%+v position=%v", w.GetSensorEvents(), visitor.GetPosition())
			}
		})
	}
}

func TestGolemKinematicCCDGeometryParallelSensorTargets(t *testing.T) {
	for _, workers := range []uint32{1, 8} {
		t.Run(fmt.Sprint(workers), func(t *testing.T) {
			d := DefaultWorldDef()
			d.WorkerCount, d.Gravity, d.EnableSleep = workers, Vec3Zero, false
			w := CreateWorld(&d)
			t.Cleanup(w.Destroy)
			if err := w.SetSpeculativeCCD(true); err != nil {
				t.Fatal(err)
			}
			for i := 0; i < 128; i++ {
				x := float32(i) * 4
				sensor := motionStepBody(w, KinematicBody, Pos{X: x}, Vec3{X: 1})
				sd := DefaultShapeDef()
				sd.IsSensor, sd.EnableSensorEvents = true, true
				sensor.CreateSphereShape(&sd, &Sphere{Radius: .2})
				kinematicGeometryMotorBody(w, Pos{X: x, Y: .5}, Vec3{Y: -100}, false)
			}
			if err := w.StepObserved(.0125, DefaultObservedStepOptions(), nil); err != nil {
				t.Fatal(err)
			}
			if w.GetSensorEvents().BeginCount != 128 {
				t.Fatalf("retained sensor targets lost crossings: %+v", w.GetSensorEvents())
			}
		})
	}
}
