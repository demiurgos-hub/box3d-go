// Golem additions to Box3D-Go. SPDX-License-Identifier: MIT
package box3d

import "testing"

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
