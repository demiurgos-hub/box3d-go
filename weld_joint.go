// Port of src/weld_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3WeldJoint_* functions of box3d.h are methods on JointID named WeldJoint<Method>.

// WeldJointSetLinearHertz is b3WeldJoint_SetLinearHertz.
// Set the weld joint linear stiffness in Hertz. 0 is rigid.
func (jointId JointID) WeldJointSetLinearHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWeldJointSetLinearHertz{jointId, hertz}
		recWriteWeldJointSetLinearHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, WeldJoint)
	base.weldJoint.linearHertz = hertz
}

// WeldJointGetLinearHertz is b3WeldJoint_GetLinearHertz.
// Get the weld joint linear stiffness in Hertz
func (jointId JointID) WeldJointGetLinearHertz() float32 {
	base := getJointSimCheckType(jointId, WeldJoint)
	return base.weldJoint.linearHertz
}

// WeldJointSetLinearDampingRatio is b3WeldJoint_SetLinearDampingRatio.
// Set the weld joint linear damping ratio (non-dimensional)
func (jointId JointID) WeldJointSetLinearDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWeldJointSetLinearDampingRatio{jointId, dampingRatio}
		recWriteWeldJointSetLinearDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, WeldJoint)
	base.weldJoint.linearDampingRatio = dampingRatio
}

// WeldJointGetLinearDampingRatio is b3WeldJoint_GetLinearDampingRatio.
// Get the weld joint linear damping ratio (non-dimensional)
func (jointId JointID) WeldJointGetLinearDampingRatio() float32 {
	base := getJointSimCheckType(jointId, WeldJoint)
	return base.weldJoint.linearDampingRatio
}

// WeldJointSetAngularHertz is b3WeldJoint_SetAngularHertz.
// Set the weld joint angular stiffness in Hertz. 0 is rigid.
func (jointId JointID) WeldJointSetAngularHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWeldJointSetAngularHertz{jointId, hertz}
		recWriteWeldJointSetAngularHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, WeldJoint)
	base.weldJoint.angularHertz = hertz
}

// WeldJointGetAngularHertz is b3WeldJoint_GetAngularHertz.
// Get the weld joint angular stiffness in Hertz
func (jointId JointID) WeldJointGetAngularHertz() float32 {
	base := getJointSimCheckType(jointId, WeldJoint)
	return base.weldJoint.angularHertz
}

// WeldJointSetAngularDampingRatio is b3WeldJoint_SetAngularDampingRatio.
// Set weld joint angular damping ratio, non-dimensional
func (jointId JointID) WeldJointSetAngularDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWeldJointSetAngularDampingRatio{jointId, dampingRatio}
		recWriteWeldJointSetAngularDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, WeldJoint)
	base.weldJoint.angularDampingRatio = dampingRatio
}

// WeldJointGetAngularDampingRatio is b3WeldJoint_GetAngularDampingRatio.
// Get the weld joint angular damping ratio, non-dimensional
func (jointId JointID) WeldJointGetAngularDampingRatio() float32 {
	base := getJointSimCheckType(jointId, WeldJoint)
	return base.weldJoint.angularDampingRatio
}

// getWeldJointForce is b3GetWeldJointForce.
func getWeldJointForce(world *world, base *jointSim) Vec3 {
	force := MulSV(world.inv_h, base.weldJoint.linearImpulse)
	return force
}

// getWeldJointTorque is b3GetWeldJointTorque.
func getWeldJointTorque(world *world, base *jointSim) Vec3 {
	return MulSV(world.inv_h, base.weldJoint.angularImpulse)
}

// prepareWeldJoint is b3PrepareWeldJoint.
func prepareWeldJoint(base *jointSim, context *stepContext) {
	assert(base.typ == WeldJoint, "base->type == b3_weldJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.weldJoint
	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)
	joint.angularMass = InvertMatrix(invInertiaSum)

	if joint.linearHertz == 0.0 {
		joint.linearSpring = base.constraintSoftness
	} else {
		joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, jointIntervalH(context))
	}

	if joint.angularHertz == 0.0 {
		joint.angularSpring = base.constraintSoftness
	} else {
		joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, jointIntervalH(context))
	}

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.linearImpulse = Vec3Zero
		joint.angularImpulse = Vec3Zero
	}
}

// warmStartWeldJoint is b3WarmStartWeldJoint.
func warmStartWeldJoint(base *jointSim, context *stepContext) {
	assert(base.typ == WeldJoint, "base->type == b3_weldJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.weldJoint

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

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	vA = MulSub(vA, mA, joint.linearImpulse)
	wA = Sub(wA, MulMV(iA, Add(Cross(rA, joint.linearImpulse), joint.angularImpulse)))

	vB = MulAdd(vB, mB, joint.linearImpulse)
	wB = Add(wB, MulMV(iB, Add(Cross(rB, joint.linearImpulse), joint.angularImpulse)))

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

// solveWeldJoint is b3SolveWeldJoint.
func solveWeldJoint(base *jointSim, context *stepContext, useBias bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.weldJoint
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

	fixedRotation := base.fixedRotation
	quatA := MulQuat(stateA.deltaRotation, joint.frameA.Q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.Q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	// angular constraint
	if fixedRotation == false {
		bias := Vec3Zero
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias || joint.angularHertz > 0.0 {
			targetQuat := QuatIdentity
			deltaRotation := deltaQuatToRotation(relQ, targetQuat)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias = MulSV(joint.angularSpring.biasRate, c)
			massScale = joint.angularSpring.massScale
			impulseScale = joint.angularSpring.impulseScale
		}

		cdot := Sub(wB, wA)
		impulse := MulSub(MulSV(-massScale, MulMV(joint.angularMass, Add(cdot, bias))), impulseScale, joint.angularImpulse)
		joint.angularImpulse = Add(joint.angularImpulse, impulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// linear constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
		rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

		cdot := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

		bias := Vec3Zero
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias || joint.linearHertz > 0.0 {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition

			separation := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

			bias = MulSV(joint.linearSpring.biasRate, separation)
			massScale = joint.linearSpring.massScale
			impulseScale = joint.linearSpring.impulseScale
		}

		b := solvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		impulse := MulSub(MulSV(-massScale, b), impulseScale, joint.linearImpulse)
		joint.linearImpulse = Add(joint.linearImpulse, impulse)

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

// drawWeldJoint is b3DrawWeldJoint.
func drawWeldJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	extents := Vec3{0.1 * scale, 0.05 * scale, 0.025 * scale}
	draw.DrawBoxFcn(extents, frameA, ColorDarkOrange, draw.Context)
	draw.DrawBoxFcn(extents, frameB, ColorDarkCyan, draw.Context)
}
