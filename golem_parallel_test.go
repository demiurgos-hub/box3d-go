// Golem regressions for the upstream parallel-joint solver. SPDX-License-Identifier: MIT
package box3d

import "testing"

func TestGolemParallelFreeTwistKeepsWarmStartHemisphere(t *testing.T) {
	for _, axis := range []Vec3{Vec3AxisY, Vec3AxisZ} {
		def := DefaultWorldDef()
		def.Gravity = Vec3Zero
		def.EnableSleep = false
		world := CreateWorld(&def)
		world.SetConsistentParallelWarmStart(true)
		bodyDef := DefaultBodyDef()
		anchor := world.CreateBody(&bodyDef)
		bodyDef.Type = DynamicBody
		body := world.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		body.CreateSphereShape(&shapeDef, &Sphere{Radius: .25})
		mass := body.GetMassData()
		mass.Mass = 1
		mass.Inertia = Matrix3{Cx: Vec3{X: .05}, Cy: Vec3{Y: .05}, Cz: Vec3{Z: .05}}
		body.SetMassData(mass)
		jointDef := DefaultParallelJointDef()
		jointDef.Base.BodyIdA = anchor
		jointDef.Base.BodyIdB = body
		jointDef.Hertz = 10
		jointDef.DampingRatio = 1
		jointDef.MaxTorque = 100
		if axis == Vec3AxisY {
			rotation := Quat{V: Vec3{X: -.70710677}, S: .70710677}
			jointDef.Base.LocalFrameA.Q = rotation
			jointDef.Base.LocalFrameB.Q = rotation
		}
		joint := world.CreateParallelJoint(&jointDef)
		for i := 0; i < 160; i++ {
			body.ApplyTorque(Add(Vec3{X: 1}, MulSV(2, axis)), true)
			world.Step(.0125, 1)
			up := RotateVector(body.GetRotation(), axis)
			if Dot(up, axis) < .995 || !IsValidVec3(joint.GetConstraintTorque()) {
				t.Fatalf("free twist injected tilt: axis=%v step=%d up=%v velocity=%v", axis, i, up, body.GetAngularVelocity())
			}
		}
		if Dot(body.GetAngularVelocity(), axis) < 10 {
			t.Fatal("constraint blocked free twist")
		}
		world.Destroy()
	}
}
