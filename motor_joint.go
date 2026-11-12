// Port of src/motor_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3MotorJoint_* functions of box3d.h are methods on JointID named MotorJoint<Method>.
// B3_REC is expanded at the call site (see recording.go).

// MotorJointSetLinearVelocity is b3MotorJoint_SetLinearVelocity.
// Set the desired relative linear velocity in meters per second
func (jointId JointID) MotorJointSetLinearVelocity(velocity Vec3) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetLinearVelocity{jointId, velocity}
		recWriteMotorJointSetLinearVelocity(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.linearVelocity = velocity
}

// MotorJointGetLinearVelocity is b3MotorJoint_GetLinearVelocity.
// Get the desired relative linear velocity in meters per second
func (jointId JointID) MotorJointGetLinearVelocity() Vec3 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.linearVelocity
}

// MotorJointSetAngularVelocity is b3MotorJoint_SetAngularVelocity.
// Set the desired relative angular velocity in radians per second
func (jointId JointID) MotorJointSetAngularVelocity(velocity Vec3) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetAngularVelocity{jointId, velocity}
		recWriteMotorJointSetAngularVelocity(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.angularVelocity = velocity
}

// MotorJointGetAngularVelocity is b3MotorJoint_GetAngularVelocity.
// Get the desired relative angular velocity in radians per second
func (jointId JointID) MotorJointGetAngularVelocity() Vec3 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.angularVelocity
}

// MotorJointSetMaxVelocityTorque is b3MotorJoint_SetMaxVelocityTorque.
// Set the motor joint maximum torque, usually in newton-meters
func (jointId JointID) MotorJointSetMaxVelocityTorque(maxTorque float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetMaxVelocityTorque{jointId, maxTorque}
		recWriteMotorJointSetMaxVelocityTorque(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.maxVelocityTorque = maxTorque
}

// MotorJointGetMaxVelocityTorque is b3MotorJoint_GetMaxVelocityTorque.
// Get the motor joint maximum torque, usually in newton-meters
func (jointId JointID) MotorJointGetMaxVelocityTorque() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.maxVelocityTorque
}

// MotorJointSetMaxVelocityForce is b3MotorJoint_SetMaxVelocityForce.
// Set the motor joint maximum force, usually in newtons
func (jointId JointID) MotorJointSetMaxVelocityForce(maxForce float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetMaxVelocityForce{jointId, maxForce}
		recWriteMotorJointSetMaxVelocityForce(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.maxVelocityForce = maxForce
}

// MotorJointGetMaxVelocityForce is b3MotorJoint_GetMaxVelocityForce.
// Get the motor joint maximum force, usually in newtons
func (jointId JointID) MotorJointGetMaxVelocityForce() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.maxVelocityForce
}

// MotorJointSetLinearHertz is b3MotorJoint_SetLinearHertz.
// Set the spring linear hertz stiffness
func (jointId JointID) MotorJointSetLinearHertz(hertz float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetLinearHertz{jointId, hertz}
		recWriteMotorJointSetLinearHertz(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.linearHertz = hertz
}

// MotorJointGetLinearHertz is b3MotorJoint_GetLinearHertz.
// Get the spring linear hertz stiffness
func (jointId JointID) MotorJointGetLinearHertz() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.linearHertz
}

// MotorJointSetLinearDampingRatio is b3MotorJoint_SetLinearDampingRatio.
// Set the spring linear damping ratio. Use 1.0 for critical damping.
func (jointId JointID) MotorJointSetLinearDampingRatio(damping float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetLinearDampingRatio{jointId, damping}
		recWriteMotorJointSetLinearDampingRatio(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.linearDampingRatio = damping
}

// MotorJointGetLinearDampingRatio is b3MotorJoint_GetLinearDampingRatio.
// Get the spring linear damping ratio.
func (jointId JointID) MotorJointGetLinearDampingRatio() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.linearDampingRatio
}

// MotorJointSetAngularHertz is b3MotorJoint_SetAngularHertz.
// Set the spring angular hertz stiffness
func (jointId JointID) MotorJointSetAngularHertz(hertz float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetAngularHertz{jointId, hertz}
		recWriteMotorJointSetAngularHertz(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.angularHertz = hertz
}

// MotorJointGetAngularHertz is b3MotorJoint_GetAngularHertz.
// Get the spring angular hertz stiffness
func (jointId JointID) MotorJointGetAngularHertz() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.angularHertz
}

// MotorJointSetAngularDampingRatio is b3MotorJoint_SetAngularDampingRatio.
// Set the spring angular damping ratio. Use 1.0 for critical damping.
func (jointId JointID) MotorJointSetAngularDampingRatio(damping float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetAngularDampingRatio{jointId, damping}
		recWriteMotorJointSetAngularDampingRatio(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.angularDampingRatio = damping
}

// MotorJointGetAngularDampingRatio is b3MotorJoint_GetAngularDampingRatio.
// Get the spring angular damping ratio.
func (jointId JointID) MotorJointGetAngularDampingRatio() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.angularDampingRatio
}

// MotorJointSetMaxSpringForce is b3MotorJoint_SetMaxSpringForce.
// Set the maximum spring force in newtons.
func (jointId JointID) MotorJointSetMaxSpringForce(maxForce float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetMaxSpringForce{jointId, maxForce}
		recWriteMotorJointSetMaxSpringForce(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.maxSpringForce = MaxFloat(0.0, maxForce)
}

// MotorJointGetMaxSpringForce is b3MotorJoint_GetMaxSpringForce.
// Get the maximum spring force in newtons.
func (jointId JointID) MotorJointGetMaxSpringForce() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.maxSpringForce
}

// MotorJointSetMaxSpringTorque is b3MotorJoint_SetMaxSpringTorque.
// Set the maximum spring torque in newtons * meters
func (jointId JointID) MotorJointSetMaxSpringTorque(maxTorque float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsMotorJointSetMaxSpringTorque{jointId, maxTorque}
		recWriteMotorJointSetMaxSpringTorque(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, MotorJoint)
	joint.motorJoint.maxSpringTorque = MaxFloat(0.0, maxTorque)
}

// MotorJointGetMaxSpringTorque is b3MotorJoint_GetMaxSpringTorque.
// Get the maximum spring torque in newtons * meters
func (jointId JointID) MotorJointGetMaxSpringTorque() float32 {
	joint := getJointSimCheckType(jointId, MotorJoint)
	return joint.motorJoint.maxSpringTorque
}

// getMotorJointForce is b3GetMotorJointForce.
func getMotorJointForce(world *world, base *jointSim) Vec3 {
	force := MulSV(world.inv_h, Add(base.motorJoint.linearVelocityImpulse, base.motorJoint.linearSpringImpulse))
	return force
}

// getMotorJointTorque is b3GetMotorJointTorque.
func getMotorJointTorque(world *world, base *jointSim) Vec3 {
	return MulSV(world.inv_h, Add(base.motorJoint.angularVelocityImpulse, base.motorJoint.angularSpringImpulse))
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// prepareMotorJoint is b3PrepareMotorJoint.
func prepareMotorJoint(base *jointSim, context *stepContext) {
	assert(base.typ == MotorJoint, "base->type == b3_motorJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.motorJoint
	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, jointIntervalH(context))
	joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, jointIntervalH(context))

	joint.angularMass = InvertMatrix(invInertiaSum)

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.linearVelocityImpulse = Vec3Zero
		joint.angularVelocityImpulse = Vec3Zero
		joint.linearSpringImpulse = Vec3Zero
		joint.angularSpringImpulse = Vec3Zero
	}
}

// warmStartMotorJoint is b3WarmStartMotorJoint.
// PORT: as upstream, the impulses are applied without the b3_dynamicFlag check the other joints
// make, so a kinematic body in the awake set gets its velocity changed; kept as is (see
// LOOSE_ENDS.md).
func warmStartMotorJoint(base *jointSim, context *stepContext) {
	assert(base.typ == MotorJoint, "base->type == b3_motorJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	joint := &base.motorJoint

	// dummy state for static bodies
	dummyState := identityBodyState

	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	linearImpulse := Add(joint.linearVelocityImpulse, joint.linearSpringImpulse)
	angularImpulse := Add(joint.angularVelocityImpulse, joint.angularSpringImpulse)

	stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
	stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Add(Cross(rA, linearImpulse), angularImpulse)))
	stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
	stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Add(Cross(rB, linearImpulse), angularImpulse)))
}

// solveMotorJoint is b3SolveMotorJoint.
func solveMotorJoint(base *jointSim, context *stepContext) {
	assert(base.typ == MotorJoint, "base->type == b3_motorJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.motorJoint
	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.Q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.Q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	// angular spring
	if joint.maxSpringTorque > 0.0 && joint.angularHertz > 0.0 {
		targetQuat := QuatIdentity
		deltaRotation := deltaQuatToRotation(relQ, targetQuat)
		c := Neg(RotateVector(quatA, deltaRotation))

		bias := MulSV(joint.angularSpring.biasRate, c)
		massScale := joint.angularSpring.massScale
		impulseScale := joint.angularSpring.impulseScale

		cdot := Sub(wB, wA)

		maxImpulse := float32(jointIntervalH(context) * joint.maxSpringTorque)
		oldImpulse := joint.angularSpringImpulse
		impulse := MulSub(MulSV(-massScale, MulMV(joint.angularMass, Add(cdot, bias))), impulseScale, oldImpulse)
		joint.angularSpringImpulse = Add(oldImpulse, impulse)
		if LengthSquared(joint.angularSpringImpulse) > float32(maxImpulse*maxImpulse) {
			joint.angularSpringImpulse = MulSV(maxImpulse, Normalize(joint.angularSpringImpulse))
		}
		impulse = Sub(joint.angularSpringImpulse, oldImpulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// angular velocity
	if joint.maxVelocityTorque > 0.0 {
		cdot := Sub(Sub(wB, wA), joint.angularVelocity)
		impulse := Neg(MulMV(joint.angularMass, cdot))

		maxImpulse := float32(jointIntervalH(context) * joint.maxVelocityTorque)
		oldImpulse := joint.angularVelocityImpulse
		joint.angularVelocityImpulse = Add(oldImpulse, impulse)
		if LengthSquared(joint.angularVelocityImpulse) > float32(maxImpulse*maxImpulse) {
			joint.angularVelocityImpulse = MulSV(maxImpulse, Normalize(joint.angularVelocityImpulse))
		}
		impulse = Sub(joint.angularVelocityImpulse, oldImpulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	// linear spring
	if joint.maxSpringForce > 0.0 && joint.linearHertz > 0.0 {
		dcA := stateA.deltaPosition
		dcB := stateB.deltaPosition
		c := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

		bias := MulSV(joint.linearSpring.biasRate, c)
		massScale := joint.linearSpring.massScale
		impulseScale := joint.linearSpring.impulseScale

		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

		b := solvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		oldImpulse := joint.linearSpringImpulse
		impulse := MulSub(MulSV(-massScale, b), impulseScale, oldImpulse)
		maxImpulse := float32(jointIntervalH(context) * joint.maxSpringForce)
		joint.linearSpringImpulse = Add(joint.linearSpringImpulse, impulse)

		if LengthSquared(joint.linearSpringImpulse) > float32(maxImpulse*maxImpulse) {
			joint.linearSpringImpulse = MulSV(maxImpulse, Normalize(joint.linearSpringImpulse))
		}

		impulse = Sub(joint.linearSpringImpulse, oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = Sub(wA, MulMV(iA, Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = Add(wB, MulMV(iB, Cross(rB, impulse)))
	}

	// linear velocity
	if joint.maxVelocityForce > 0.0 {
		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))
		cdot = Sub(cdot, joint.linearVelocity)

		b := solvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, cdot)
		impulse := Neg(b)

		oldImpulse := joint.linearVelocityImpulse
		maxImpulse := float32(jointIntervalH(context) * joint.maxVelocityForce)
		joint.linearVelocityImpulse = Add(joint.linearVelocityImpulse, impulse)

		if LengthSquared(joint.linearVelocityImpulse) > float32(maxImpulse*maxImpulse) {
			joint.linearVelocityImpulse = MulSV(maxImpulse, Normalize(joint.linearVelocityImpulse))
		}

		impulse = Sub(joint.linearVelocityImpulse, oldImpulse)

		vA = MulSub(vA, mA, impulse)
		wA = Sub(wA, MulMV(iA, Cross(rA, impulse)))
		vB = MulAdd(vB, mB, impulse)
		wB = Add(wB, MulMV(iB, Cross(rB, impulse)))
	}

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}
