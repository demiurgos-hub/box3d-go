// Port of src/spherical_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3SphericalJoint_* functions of box3d.h are methods on JointID named
// SphericalJoint<Method>.

// SphericalJointEnableConeLimit is b3SphericalJoint_EnableConeLimit.
// Enable/disable the spherical joint cone limit
func (jointId JointID) SphericalJointEnableConeLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointEnableConeLimit{jointId, enableLimit}
		recWriteSphericalJointEnableConeLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	if enableLimit != base.sphericalJoint.enableConeLimit {
		base.sphericalJoint.swingImpulse = 0.0
	}
	base.sphericalJoint.enableConeLimit = enableLimit
}

// SphericalJointIsConeLimitEnabled is b3SphericalJoint_IsConeLimitEnabled.
// Is the spherical joint cone limit enabled?
func (jointId JointID) SphericalJointIsConeLimitEnabled() bool {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.enableConeLimit
}

// SphericalJointGetConeLimit is b3SphericalJoint_GetConeLimit.
// Get the spherical joint cone limit in radians
func (jointId JointID) SphericalJointGetConeLimit() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.coneAngle
}

// SphericalJointSetConeLimit is b3SphericalJoint_SetConeLimit.
// Set the spherical joint limits in radians
func (jointId JointID) SphericalJointSetConeLimit(angleRadians float32) {
	assert(IsValidFloat(angleRadians) && 0 <= angleRadians && angleRadians <= 0.5*Pi,
		"b3IsValidFloat( angleRadians ) && 0 <= angleRadians && angleRadians <= 0.5f * B3_PI")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetConeLimit{jointId, angleRadians}
		recWriteSphericalJointSetConeLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.coneAngle = angleRadians
}

// SphericalJointGetConeAngle is b3SphericalJoint_GetConeAngle.
// Get the spherical joint current cone angle in radians.
func (jointId JointID) SphericalJointGetConeAngle() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, SphericalJoint)
	transformA := getBodyTransform(world, int(base.bodyIdA))
	transformB := getBodyTransform(world, int(base.bodyIdB))

	quatA := MulQuat(transformA.Q, base.localFrameA.Q)
	quatB := MulQuat(transformB.Q, base.localFrameB.Q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the swing angle in the range [0, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	return GetSwingAngle(relQ)
}

// SphericalJointEnableTwistLimit is b3SphericalJoint_EnableTwistLimit.
// Enable/disable the spherical joint limit
func (jointId JointID) SphericalJointEnableTwistLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointEnableTwistLimit{jointId, enableLimit}
		recWriteSphericalJointEnableTwistLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	if enableLimit != base.sphericalJoint.enableTwistLimit {
		base.sphericalJoint.lowerTwistImpulse = 0.0
		base.sphericalJoint.upperTwistImpulse = 0.0
	}
	base.sphericalJoint.enableTwistLimit = enableLimit
}

// SphericalJointIsTwistLimitEnabled is b3SphericalJoint_IsTwistLimitEnabled.
// Is the spherical joint limit enabled?
func (jointId JointID) SphericalJointIsTwistLimitEnabled() bool {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.enableTwistLimit
}

// SphericalJointGetLowerTwistLimit is b3SphericalJoint_GetLowerTwistLimit.
// Get the spherical joint lower limit in radians
func (jointId JointID) SphericalJointGetLowerTwistLimit() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.lowerTwistAngle
}

// SphericalJointGetUpperTwistLimit is b3SphericalJoint_GetUpperTwistLimit.
// Get the spherical joint upper limit in radians
func (jointId JointID) SphericalJointGetUpperTwistLimit() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.upperTwistAngle
}

// SphericalJointSetTwistLimits is b3SphericalJoint_SetTwistLimits.
// Set the spherical joint limits in radians
func (jointId JointID) SphericalJointSetTwistLimits(lowerLimitRadians, upperLimitRadians float32) {
	assert(IsValidFloat(lowerLimitRadians) && IsValidFloat(upperLimitRadians),
		"b3IsValidFloat( lowerLimitRadians ) && b3IsValidFloat( upperLimitRadians )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetTwistLimits{jointId, lowerLimitRadians, upperLimitRadians}
		recWriteSphericalJointSetTwistLimits(world.recording, &recArgs)
	}

	lowerAngle := MinFloat(lowerLimitRadians, upperLimitRadians)
	upperAngle := MaxFloat(lowerLimitRadians, upperLimitRadians)

	// PORT: 0.99f * B3_PI folds to the same float as the C product (one rounding of an exact
	// product of two floats).
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.lowerTwistAngle = ClampFloat(lowerAngle, -float32(0.99)*Pi, float32(0.99)*Pi)
	base.sphericalJoint.upperTwistAngle = ClampFloat(upperAngle, -float32(0.99)*Pi, float32(0.99)*Pi)
}

// SphericalJointGetTwistAngle is b3SphericalJoint_GetTwistAngle.
// Get the spherical joint current twist angle in radians.
func (jointId JointID) SphericalJointGetTwistAngle() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, SphericalJoint)
	transformA := getBodyTransform(world, int(base.bodyIdA))
	transformB := getBodyTransform(world, int(base.bodyIdB))

	quatA := MulQuat(transformA.Q, base.localFrameA.Q)
	quatB := MulQuat(transformB.Q, base.localFrameB.Q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the twist angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	return GetTwistAngle(relQ)
}

// SphericalJointEnableSpring is b3SphericalJoint_EnableSpring.
// Enable/disable the spherical joint spring
func (jointId JointID) SphericalJointEnableSpring(enableSpring bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointEnableSpring{jointId, enableSpring}
		recWriteSphericalJointEnableSpring(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	if enableSpring != base.sphericalJoint.enableSpring {
		base.sphericalJoint.springImpulse = Vec3Zero
	}
	base.sphericalJoint.enableSpring = enableSpring
}

// SphericalJointIsSpringEnabled is b3SphericalJoint_IsSpringEnabled.
// Is the spherical angular spring enabled?
func (jointId JointID) SphericalJointIsSpringEnabled() bool {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.enableSpring
}

// SphericalJointSetTargetRotation is b3SphericalJoint_SetTargetRotation.
// Set the spherical joint spring target rotation
func (jointId JointID) SphericalJointSetTargetRotation(targetRotation Quat) {
	assert(IsValidQuat(targetRotation), "b3IsValidQuat( targetRotation )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetTargetRotation{jointId, targetRotation}
		recWriteSphericalJointSetTargetRotation(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.targetRotation = targetRotation
}

// SphericalJointGetTargetRotation is b3SphericalJoint_GetTargetRotation.
// Get the spherical joint spring target rotation
func (jointId JointID) SphericalJointGetTargetRotation() Quat {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.targetRotation
}

// SphericalJointSetSpringHertz is b3SphericalJoint_SetSpringHertz.
// Set the spherical joint spring stiffness in Hertz
func (jointId JointID) SphericalJointSetSpringHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetSpringHertz{jointId, hertz}
		recWriteSphericalJointSetSpringHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.hertz = hertz
}

// SphericalJointGetSpringHertz is b3SphericalJoint_GetSpringHertz.
// Get the spherical joint spring stiffness in Hertz
func (jointId JointID) SphericalJointGetSpringHertz() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.hertz
}

// SphericalJointSetSpringDampingRatio is b3SphericalJoint_SetSpringDampingRatio.
// Set the spherical joint spring damping ratio, non-dimensional
func (jointId JointID) SphericalJointSetSpringDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetSpringDampingRatio{jointId, dampingRatio}
		recWriteSphericalJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.dampingRatio = dampingRatio
}

// SphericalJointGetSpringDampingRatio is b3SphericalJoint_GetSpringDampingRatio.
// Get the spherical joint spring damping ratio, non-dimensional
func (jointId JointID) SphericalJointGetSpringDampingRatio() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.dampingRatio
}

// SphericalJointEnableMotor is b3SphericalJoint_EnableMotor.
// Enable/disable a spherical joint motor
func (jointId JointID) SphericalJointEnableMotor(enableMotor bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointEnableMotor{jointId, enableMotor}
		recWriteSphericalJointEnableMotor(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	if enableMotor != base.sphericalJoint.enableMotor {
		base.sphericalJoint.motorImpulse = Vec3Zero
	}
	base.sphericalJoint.enableMotor = enableMotor
}

// SphericalJointIsMotorEnabled is b3SphericalJoint_IsMotorEnabled.
// Is the spherical joint motor enabled?
func (jointId JointID) SphericalJointIsMotorEnabled() bool {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.enableMotor
}

// SphericalJointSetMotorVelocity is b3SphericalJoint_SetMotorVelocity.
// Set the spherical joint motor velocity in radians per second. This is the relative angular
// velocity between the two bodies in world space.
// motorVelocity = angularVelocityB - angularVelocityA
func (jointId JointID) SphericalJointSetMotorVelocity(motorVelocity Vec3) {
	assert(IsValidVec3(motorVelocity), "b3IsValidVec3( motorVelocity )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetMotorVelocity{jointId, motorVelocity}
		recWriteSphericalJointSetMotorVelocity(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.motorVelocity = motorVelocity
}

// SphericalJointGetMotorVelocity is b3SphericalJoint_GetMotorVelocity.
// Get the spherical joint motor velocity in radians per second
func (jointId JointID) SphericalJointGetMotorVelocity() Vec3 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.motorVelocity
}

// SphericalJointSetMaxMotorTorque is b3SphericalJoint_SetMaxMotorTorque.
// Set the spherical joint maximum motor torque, usually in newton-meters
func (jointId JointID) SphericalJointSetMaxMotorTorque(maxForce float32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0, "b3IsValidFloat( maxForce ) && maxForce >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsSphericalJointSetMaxMotorTorque{jointId, maxForce}
		recWriteSphericalJointSetMaxMotorTorque(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, SphericalJoint)
	base.sphericalJoint.maxMotorTorque = maxForce
}

// SphericalJointGetMaxMotorTorque is b3SphericalJoint_GetMaxMotorTorque.
// Get the spherical joint maximum motor torque, usually in newton-meters
func (jointId JointID) SphericalJointGetMaxMotorTorque() float32 {
	base := getJointSimCheckType(jointId, SphericalJoint)
	return base.sphericalJoint.maxMotorTorque
}

// SphericalJointGetMotorTorque is b3SphericalJoint_GetMotorTorque.
// Get the spherical joint current motor torque, usually in newton-meters
func (jointId JointID) SphericalJointGetMotorTorque() Vec3 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, SphericalJoint)
	return MulSV(world.inv_h, base.sphericalJoint.motorImpulse)
}

// getSphericalJointForce is b3GetSphericalJointForce.
func getSphericalJointForce(world *world, base *jointSim) Vec3 {
	force := MulSV(world.inv_h, base.sphericalJoint.linearImpulse)
	return force
}

// getSphericalJointTorque is b3GetSphericalJointTorque.
func getSphericalJointTorque(world *world, base *jointSim) Vec3 {
	xfA := getBodyTransform(world, int(base.bodyIdA))
	xfB := getBodyTransform(world, int(base.bodyIdB))
	qA := MulQuat(xfA.Q, base.localFrameA.Q)
	qB := MulQuat(xfB.Q, base.localFrameB.Q)

	// Cone axis is the z-axis of body A.
	coneAxis := RotateVector(qA, Vec3AxisZ)
	twistAxis := RotateVector(qB, Vec3AxisZ)
	swingAxis := Normalize(Cross(coneAxis, twistAxis))

	joint := &base.sphericalJoint
	impulse := Add(joint.springImpulse, joint.motorImpulse)
	impulse = MulAdd(impulse, joint.lowerTwistImpulse-joint.upperTwistImpulse, twistAxis)
	impulse = MulAdd(impulse, joint.swingImpulse, swingAxis)
	torque := MulSV(world.inv_h, impulse)
	return torque
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I r1_skew I -r2_skew ]
// K = J * invM * transpose(J)
// transpose(skew(r)) = -skew(r)
// K = diag(1/m1 + 1/m2) - r1_skew * invI1 * r1_skew - r2_skew * invI2 * r2_skew

// r_skew = R * skew(r_local) * RT
// invI = R * invI_local * RT
// r_skew * invI * r_skew = R * skew(r_local) * RT * R * invI_local * RT * R * r_skew * RT
//                        = R * ( skew(r_local) * invI_local * skew(r_local) ) * RT

// prepareSphericalJoint is b3PrepareSphericalJoint.
func prepareSphericalJoint(base *jointSim, context *stepContext) {
	assert(base.typ == SphericalJoint, "base->type == b3_sphericalJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.sphericalJoint
	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	// Cone axis is the z-axis of body A.
	coneAxis := RotateVector(joint.frameA.Q, Vec3AxisZ)

	// Twist axis is the z-axis of body B.
	twistAxis := RotateVector(joint.frameB.Q, Vec3AxisZ)

	if joint.enableConeLimit {
		// Swing axis may be zero
		swingAxis := Normalize(Cross(coneAxis, twistAxis))
		k := Dot(swingAxis, MulMV(invInertiaSum, swingAxis))
		joint.swingMass = 0.0
		if k > 0.0 {
			joint.swingMass = 1.0 / k
		}
		joint.swingAxis = swingAxis
	}

	if joint.enableTwistLimit {
		relQ := InvMulQuat(joint.frameA.Q, joint.frameB.Q)
		tanThetaOver2 := sqrtf((float32(relQ.V.X*relQ.V.X) + float32(relQ.V.Y*relQ.V.Y)) /
			(float32(relQ.V.Z*relQ.V.Z) + float32(relQ.S*relQ.S)))

		// todo verify this Jacobian using a finite difference, unit test?
		swingAxis := Normalize(Cross(coneAxis, twistAxis))
		perpAxis := Cross(swingAxis, coneAxis)
		twistJacobian := MulAdd(coneAxis, tanThetaOver2, perpAxis)
		k := Dot(twistJacobian, MulMV(invInertiaSum, twistJacobian))
		joint.twistMass = 0.0
		if k > 0.0 {
			joint.twistMass = 1.0 / k
		}
		joint.twistJacobian = twistJacobian
	}

	if base.fixedRotation == false {
		joint.rotationMass = InvertMatrix(invInertiaSum)
	} else {
		joint.rotationMass = Mat3Zero
	}

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, jointIntervalH(context))

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.linearImpulse = Vec3Zero
		joint.motorImpulse = Vec3Zero
		joint.springImpulse = Vec3Zero
		joint.swingImpulse = 0.0
		joint.lowerTwistImpulse = 0.0
		joint.upperTwistImpulse = 0.0
	}
}

// warmStartSphericalJoint is b3WarmStartSphericalJoint.
func warmStartSphericalJoint(base *jointSim, context *stepContext) {
	assert(base.typ == SphericalJoint, "base->type == b3_sphericalJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.sphericalJoint

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

	angularImpulse := Add(joint.springImpulse, joint.motorImpulse)
	angularImpulse = MulSub(angularImpulse, joint.swingImpulse, joint.swingAxis)
	angularImpulse = MulAdd(angularImpulse, joint.lowerTwistImpulse-joint.upperTwistImpulse, joint.twistJacobian)

	vA = MulSub(vA, mA, joint.linearImpulse)
	wA = Sub(wA, MulMV(iA, Add(Cross(rA, joint.linearImpulse), angularImpulse)))

	vB = MulAdd(vB, mB, joint.linearImpulse)
	wB = Add(wB, MulMV(iB, Add(Cross(rB, joint.linearImpulse), angularImpulse)))

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

// solveSphericalJoint is b3SolveSphericalJoint.
func solveSphericalJoint(base *jointSim, context *stepContext, useBias bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.sphericalJoint
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

	relQ := InvMulQuat(quatA, quatB)

	if fixedRotation == false &&
		(joint.enableSpring || joint.enableMotor || joint.enableTwistLimit || joint.enableConeLimit) {
		invInertiaSum := AddMM(iA, iB)
		cdot := Sub(wB, wA)
		angularImpulse := Vec3Zero

		// Solve spring
		if joint.enableSpring {
			// Rotation constraint error
			deltaRotation := deltaQuatToRotation(relQ, joint.targetRotation)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias := MulSV(joint.springSoftness.biasRate, c)
			massScale := joint.springSoftness.massScale
			impulseScale := joint.springSoftness.impulseScale

			impulse := MulSub(MulSV(-massScale, MulMV(joint.rotationMass, Add(cdot, bias))),
				impulseScale, joint.springImpulse)
			joint.springImpulse = Add(joint.springImpulse, impulse)

			cdot = Add(cdot, MulMV(invInertiaSum, impulse))
			angularImpulse = Add(angularImpulse, impulse)
		}

		if joint.enableMotor {
			// todo it might be more accurate to have the motor velocity in frameA due to sub-stepping.
			lambda := Neg(MulMV(joint.rotationMass, Sub(cdot, joint.motorVelocity)))
			newImpulse := Add(joint.motorImpulse, lambda)
			length := Length(newImpulse)
			maxImpulse := float32(joint.maxMotorTorque * jointIntervalH(context))
			if length > maxImpulse {
				newImpulse = MulSV(maxImpulse/length, newImpulse)
			}

			lambda = Sub(newImpulse, joint.motorImpulse)
			joint.motorImpulse = newImpulse

			cdot = Add(cdot, MulMV(invInertiaSum, lambda))
			angularImpulse = Add(angularImpulse, lambda)
		}

		if joint.enableTwistLimit {
			twistAngle := GetTwistAngle(relQ)

			// todo does an updated twist axis help?

			twistJacobian := joint.twistJacobian
			twistResponse := MulMV(invInertiaSum, twistJacobian)

			// Lower limit
			{
				c := twistAngle - joint.lowerTwistAngle
				var bias float32 = 0.0
				var massScale float32 = 1.0
				var impulseScale float32 = 0.0
				if c > 0.0 {
					// speculation
					bias = float32(c * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * c)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				twistCdot := Dot(cdot, twistJacobian)
				oldImpulse := joint.lowerTwistImpulse
				deltaImpulse :=
					MaxFloat(float32(float32(-massScale*joint.twistMass)*(twistCdot+bias))-float32(impulseScale*oldImpulse), -oldImpulse)
				joint.lowerTwistImpulse = oldImpulse + deltaImpulse

				cdot = MulAdd(cdot, deltaImpulse, twistResponse)
				angularImpulse = MulAdd(angularImpulse, deltaImpulse, twistJacobian)
			}

			// Upper limit
			{
				c := joint.upperTwistAngle - twistAngle
				var bias float32 = 0.0
				var massScale float32 = 1.0
				var impulseScale float32 = 0.0
				if c > 0.0 {
					// speculation
					bias = float32(c * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * c)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				// sign flipped on Cdot
				twistCdot := -Dot(cdot, twistJacobian)
				oldImpulse := joint.upperTwistImpulse
				deltaImpulse :=
					MaxFloat(float32(float32(-massScale*joint.twistMass)*(twistCdot+bias))-float32(impulseScale*oldImpulse), -oldImpulse)
				joint.upperTwistImpulse = oldImpulse + deltaImpulse

				// sign flipped on applied impulse
				cdot = MulSub(cdot, deltaImpulse, twistResponse)
				angularImpulse = MulSub(angularImpulse, deltaImpulse, twistJacobian)
			}
		}

		if joint.enableConeLimit {
			swingAngle := GetSwingAngle(relQ)

			// todo does an updated swing axis help?
			// b3Vec3 axisA = b3RotateVector( quatA, b3Vec3_axisZ );
			// b3Vec3 axisB = b3RotateVector( quatB, b3Vec3_axisZ );
			// b3Vec3 swingAxis = b3Normalize( b3Cross( axisA, axisB ) );
			// joint->swingAxis = swingAxis;

			swingAxis := joint.swingAxis

			c := joint.coneAngle - swingAngle
			var bias float32 = 0.0
			var massScale float32 = 1.0
			var impulseScale float32 = 0.0
			if c > 0.0 {
				// speculation
				bias = float32(c * context.inv_h)
			} else if useBias {
				bias = float32(base.constraintSoftness.biasRate * c)
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			// sign flipped on Cdot
			swingCdot := -Dot(cdot, swingAxis)
			oldImpulse := joint.swingImpulse
			deltaImpulse :=
				MaxFloat(float32(float32(-massScale*joint.swingMass)*(swingCdot+bias))-float32(impulseScale*oldImpulse), -oldImpulse)
			joint.swingImpulse = oldImpulse + deltaImpulse

			// sign flipped on applied impulse
			angularImpulse = MulSub(angularImpulse, deltaImpulse, swingAxis)
		}

		wA = Sub(wA, MulMV(iA, angularImpulse))
		wB = Add(wB, MulMV(iB, angularImpulse))
	}

	// Solve point-to-point constraint
	{
		rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
		rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

		cdot := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, rA))

		bias := Vec3Zero
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias {
			dcA := stateA.deltaPosition
			dcB := stateB.deltaPosition

			separation := Add(Sub(dcB, dcA), Sub(rB, rA))
			separation = Add(separation, joint.deltaCenter)

			bias = MulSV(base.constraintSoftness.biasRate, separation)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
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

// drawSphericalJoint is b3DrawSphericalJoint.
func drawSphericalJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)

	length1 := float32(0.1 * scale)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisX))), ColorRed,
		draw.Context)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisY))), ColorGreen,
		draw.Context)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisZ))), ColorBlue,
		draw.Context)

	frameB := MulWorldTransforms(transformB, base.localFrameB)

	length2 := float32(0.2 * scale)
	draw.DrawSegmentFcn(frameB.P, OffsetPos(frameB.P, MulSV(length2, RotateVector(frameB.Q, Vec3AxisZ))), ColorOrange,
		draw.Context)

	joint := &base.sphericalJoint
	const kSliceCount = 16

	// Twist limit
	if joint.enableTwistLimit {
		quatA := frameA.Q
		quatB := frameB.Q

		if DotQuat(quatA, quatB) < 0.0 {
			// this keeps the twist angle in the range [-pi, pi]
			quatB = NegateQuat(quatB)
		}

		relQ := InvMulQuat(quatA, quatB)

		wedgeRadius := float32(0.1 * scale)
		for index := 0; index < kSliceCount; index++ {
			// PORT: the conversion keeps Go from fusing the division (a multiply by 1/16) into 1 - t1.
			t1 := float32(float32(index+0) / kSliceCount)
			alpha1 := float32((1.0-t1)*joint.lowerTwistAngle) + float32(t1*joint.upperTwistAngle)
			t2 := float32(float32(index+1) / kSliceCount)
			alpha2 := float32((1.0-t2)*joint.lowerTwistAngle) + float32(t2*joint.upperTwistAngle)

			vertex1 := Vec3{wedgeRadius * Cos(alpha1), wedgeRadius * Sin(alpha1), 0.0}
			vertex2 := Vec3{wedgeRadius * Cos(alpha2), wedgeRadius * Sin(alpha2), 0.0}

			if index == 0 {
				draw.DrawSegmentFcn(frameA.P, TransformWorldPoint(frameA, vertex1), ColorCyan, draw.Context)
			}

			if index == kSliceCount-1 {
				draw.DrawSegmentFcn(TransformWorldPoint(frameA, vertex2), frameA.P, ColorCyan, draw.Context)
			}
			draw.DrawSegmentFcn(TransformWorldPoint(frameA, vertex1), TransformWorldPoint(frameA, vertex2), ColorCyan,
				draw.Context)
		}

		twistAngle := GetTwistAngle(relQ)
		p2 := Vec3{wedgeRadius * Cos(twistAngle), wedgeRadius * Sin(twistAngle), 0.0}
		draw.DrawSegmentFcn(frameA.P, TransformWorldPoint(frameA, p2), ColorYellow, draw.Context)
	}

	// Swing limit
	if joint.enableConeLimit {
		radius := float32(0.1 * scale)
		coneRadius := float32(radius * Sin(joint.coneAngle))
		coneHeight := float32(radius * Cos(joint.coneAngle))

		for index := 0; index < kSliceCount; index++ {
			phi1 := 2.0 * float32(index+0) / kSliceCount * Pi
			phi2 := 2.0 * float32(index+1) / kSliceCount * Pi

			vertex1 := Vec3{coneRadius * Cos(phi1), coneRadius * Sin(phi1), coneHeight}
			vertex2 := Vec3{coneRadius * Cos(phi2), coneRadius * Sin(phi2), coneHeight}

			draw.DrawSegmentFcn(frameA.P, TransformWorldPoint(frameA, vertex1), ColorCyan, draw.Context)
			draw.DrawSegmentFcn(TransformWorldPoint(frameA, vertex1), TransformWorldPoint(frameA, vertex2), ColorCyan,
				draw.Context)
		}
	}
}
