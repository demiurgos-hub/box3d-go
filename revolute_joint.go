// Port of src/revolute_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3RevoluteJoint_* functions of box3d.h are methods on JointID named
// RevoluteJoint<Method>. B3_REC is expanded at the call site (see recording.go).

// Point-to-point linear constraint
// C = pB - pA
// Cdot = vB - vA
//      = vB + cross(wB, rB) - vA - cross(wA, rA)
// Cdot = J * v
// J = [-E -skew(rA) E skew(rB) ]
//
// K = J * invM * JT
//   = [(1/mA + 1/mB) * E - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]

// Perpendicularity constraint
// frameA = qA * localFrameA
// frameB = qB * localFrameB
// qRel = conj(frameA) * frameB
// C = [qRel.x; qRel.y]
// qRelDot = 0.5 * conj(frameA) * (wB - wA) * frameB
// Cdot = [qRelDot.x, qRelDot.y]
// Pulling out wB and wA
// sr = qRel.s
// vr = qRel.v
// Jx = 0.5 * rotate(frameA, sr * ex + cross(vr, ex))
// Jy = 0.5 * rotate(frameA, sr * ey + cross(vr, ey))

// Motor constraint
// Cdot = wB - wA
// J = [0 0 -E 0 0 E]
// K = invIA + invIB

// RevoluteJointEnableLimit is b3RevoluteJoint_EnableLimit.
// Enable/disable the revolute joint limit
func (jointId JointID) RevoluteJointEnableLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointEnableLimit{jointId, enableLimit}
		recWriteRevoluteJointEnableLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	if enableLimit != base.revoluteJoint.enableLimit {
		base.revoluteJoint.lowerImpulse = 0.0
		base.revoluteJoint.upperImpulse = 0.0
	}
	base.revoluteJoint.enableLimit = enableLimit
}

// RevoluteJointIsLimitEnabled is b3RevoluteJoint_IsLimitEnabled.
// Is the revolute joint limit enabled?
func (jointId JointID) RevoluteJointIsLimitEnabled() bool {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.enableLimit
}

// RevoluteJointGetLowerLimit is b3RevoluteJoint_GetLowerLimit.
// Get the revolute joint lower limit in radians
func (jointId JointID) RevoluteJointGetLowerLimit() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.lowerAngle
}

// RevoluteJointGetUpperLimit is b3RevoluteJoint_GetUpperLimit.
// Get the revolute joint upper limit in radians
func (jointId JointID) RevoluteJointGetUpperLimit() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.upperAngle
}

// RevoluteJointSetLimits is b3RevoluteJoint_SetLimits.
// Set the revolute joint limits in radians
func (jointId JointID) RevoluteJointSetLimits(lowerLimitRadians, upperLimitRadians float32) {
	assert(IsValidFloat(lowerLimitRadians) && IsValidFloat(upperLimitRadians), "b3IsValidFloat( lowerLimitRadians ) && b3IsValidFloat( upperLimitRadians )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetLimits{jointId, lowerLimitRadians, upperLimitRadians}
		recWriteRevoluteJointSetLimits(world.recording, &recArgs)
	}

	lowerAngle := MinFloat(lowerLimitRadians, upperLimitRadians)
	upperAngle := MaxFloat(lowerLimitRadians, upperLimitRadians)

	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.lowerAngle = ClampFloat(lowerAngle, -0.99*Pi, 0.99*Pi)
	base.revoluteJoint.upperAngle = ClampFloat(upperAngle, -0.99*Pi, 0.99*Pi)
}

// RevoluteJointGetAngle is b3RevoluteJoint_GetAngle.
// Get the revolute joint current angle in radians relative to the reference angle
func (jointId JointID) RevoluteJointGetAngle() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, RevoluteJoint)
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

// RevoluteJointEnableSpring is b3RevoluteJoint_EnableSpring.
// Enable/disable the revolute joint spring
func (jointId JointID) RevoluteJointEnableSpring(enableSpring bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointEnableSpring{jointId, enableSpring}
		recWriteRevoluteJointEnableSpring(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	if enableSpring != base.revoluteJoint.enableSpring {
		base.revoluteJoint.springImpulse = 0.0
	}
	base.revoluteJoint.enableSpring = enableSpring
}

// RevoluteJointIsSpringEnabled is b3RevoluteJoint_IsSpringEnabled.
// Is the revolute angular spring enabled?
func (jointId JointID) RevoluteJointIsSpringEnabled() bool {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.enableSpring
}

// RevoluteJointSetTargetAngle is b3RevoluteJoint_SetTargetAngle.
// Set the revolute joint target angle in radians
func (jointId JointID) RevoluteJointSetTargetAngle(targetRadians float32) {
	assert(IsValidFloat(targetRadians) && -Pi <= targetRadians && targetRadians <= Pi, "b3IsValidFloat( targetRadians ) && -B3_PI <= targetRadians && targetRadians <= B3_PI")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetTargetAngle{jointId, targetRadians}
		recWriteRevoluteJointSetTargetAngle(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.targetAngle = targetRadians
}

// RevoluteJointGetTargetAngle is b3RevoluteJoint_GetTargetAngle.
// Get the revolute joint target angle in radians
func (jointId JointID) RevoluteJointGetTargetAngle() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.targetAngle
}

// RevoluteJointSetSpringHertz is b3RevoluteJoint_SetSpringHertz.
// Set the revolute joint spring stiffness in Hertz
func (jointId JointID) RevoluteJointSetSpringHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetSpringHertz{jointId, hertz}
		recWriteRevoluteJointSetSpringHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.hertz = hertz
}

// RevoluteJointGetSpringHertz is b3RevoluteJoint_GetSpringHertz.
// Get the revolute joint spring stiffness in Hertz
func (jointId JointID) RevoluteJointGetSpringHertz() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.hertz
}

// RevoluteJointSetSpringDampingRatio is b3RevoluteJoint_SetSpringDampingRatio.
// Set the revolute joint spring damping ratio, non-dimensional
func (jointId JointID) RevoluteJointSetSpringDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetSpringDampingRatio{jointId, dampingRatio}
		recWriteRevoluteJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.dampingRatio = dampingRatio
}

// RevoluteJointGetSpringDampingRatio is b3RevoluteJoint_GetSpringDampingRatio.
// Get the revolute joint spring damping ratio, non-dimensional
func (jointId JointID) RevoluteJointGetSpringDampingRatio() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.dampingRatio
}

// RevoluteJointEnableMotor is b3RevoluteJoint_EnableMotor.
// Enable/disable a revolute joint motor
func (jointId JointID) RevoluteJointEnableMotor(enableMotor bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointEnableMotor{jointId, enableMotor}
		recWriteRevoluteJointEnableMotor(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	if enableMotor != base.revoluteJoint.enableMotor {
		base.revoluteJoint.motorImpulse = 0.0
	}
	base.revoluteJoint.enableMotor = enableMotor
}

// RevoluteJointIsMotorEnabled is b3RevoluteJoint_IsMotorEnabled.
// Is the revolute joint motor enabled?
func (jointId JointID) RevoluteJointIsMotorEnabled() bool {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.enableMotor
}

// RevoluteJointSetMotorSpeed is b3RevoluteJoint_SetMotorSpeed.
// Set the revolute joint motor speed in radians per second
func (jointId JointID) RevoluteJointSetMotorSpeed(motorSpeed float32) {
	assert(IsValidFloat(motorSpeed), "b3IsValidFloat( motorSpeed )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetMotorSpeed{jointId, motorSpeed}
		recWriteRevoluteJointSetMotorSpeed(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.motorSpeed = motorSpeed
}

// RevoluteJointGetMotorSpeed is b3RevoluteJoint_GetMotorSpeed.
// Get the revolute joint motor speed in radians per second
func (jointId JointID) RevoluteJointGetMotorSpeed() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.motorSpeed
}

// RevoluteJointSetMaxMotorTorque is b3RevoluteJoint_SetMaxMotorTorque.
// Set the revolute joint maximum motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointSetMaxMotorTorque(maxForce float32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0, "b3IsValidFloat( maxForce ) && maxForce >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsRevoluteJointSetMaxMotorTorque{jointId, maxForce}
		recWriteRevoluteJointSetMaxMotorTorque(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, RevoluteJoint)
	base.revoluteJoint.maxMotorTorque = maxForce
}

// RevoluteJointGetMaxMotorTorque is b3RevoluteJoint_GetMaxMotorTorque.
// Get the revolute joint maximum motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointGetMaxMotorTorque() float32 {
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return base.revoluteJoint.maxMotorTorque
}

// RevoluteJointGetMotorTorque is b3RevoluteJoint_GetMotorTorque.
// Get the revolute joint current motor torque, usually in newton-meters
func (jointId JointID) RevoluteJointGetMotorTorque() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, RevoluteJoint)
	return float32(world.inv_h * base.revoluteJoint.motorImpulse)
}

// getRevoluteJointForce is b3GetRevoluteJointForce.
func getRevoluteJointForce(world *world, base *jointSim) Vec3 {
	force := MulSV(world.inv_h, base.revoluteJoint.linearImpulse)
	return force
}

// getRevoluteJointTorque is b3GetRevoluteJointTorque.
// PORT: as upstream, this getter overwrites the solver's perpAxisX/perpAxisY and adds the axial
// impulse twice (once along rotationAxisZ, once along the current axis); kept as is (see
// LOOSE_ENDS.md).
func getRevoluteJointTorque(world *world, base *jointSim) Vec3 {
	transformA := getBodyTransform(world, int(base.bodyIdA))
	joint := &base.revoluteJoint
	axis := RotateVector(base.localFrameA.Q, Vec3AxisZ)
	axis = RotateVector(transformA.Q, axis)

	relQ := InvMulQuat(joint.frameA.Q, joint.frameB.Q)

	// These are needed for warm starting
	joint.perpAxisX = MulSV(
		0.5, RotateVector(joint.frameA.Q, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
	joint.perpAxisY = MulSV(
		0.5, RotateVector(joint.frameA.Q, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	angularImpulse :=
		Add(MulSV(joint.perpImpulse.X, joint.perpAxisX), MulSV(joint.perpImpulse.Y, joint.perpAxisY))
	angularImpulse = MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)

	// todo add pivot torque
	impulse := MulAdd(angularImpulse,
		joint.springImpulse+joint.motorImpulse+joint.lowerImpulse-joint.upperImpulse, axis)
	torque := MulSV(world.inv_h, impulse)
	return torque
}

// prepareRevoluteJoint is b3PrepareRevoluteJoint.
func prepareRevoluteJoint(base *jointSim, context *stepContext) {
	assert(base.typ == RevoluteJoint, "base->type == b3_revoluteJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.revoluteJoint
	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	// Avoid round-off here as much as possible.
	// b3Vec3 pf = (xf.p - c) + rot(xf.q, f.p)
	// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	// pf = rot(xf.q, f.p - lc)
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	{
		// Rotation axis is the z-axis of body A.
		rotationAxisZ := RotateVector(joint.frameA.Q, Vec3AxisZ)
		k := Dot(rotationAxisZ, MulMV(invInertiaSum, rotationAxisZ))
		joint.axialMass = 0.0
		if k > 0.0 {
			joint.axialMass = 1.0 / k
		}
		joint.rotationAxisZ = rotationAxisZ
	}

	relQ := InvMulQuat(joint.frameA.Q, joint.frameB.Q)

	{
		// These are needed for warm starting
		joint.perpAxisX = MulSV(
			0.5, RotateVector(joint.frameA.Q, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
		joint.perpAxisY = MulSV(
			0.5, RotateVector(joint.frameA.Q, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))
	}

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, jointIntervalH(context))

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.linearImpulse = Vec3Zero
		joint.perpImpulse = Vec2{0.0, 0.0}
		joint.motorImpulse = 0.0
		joint.springImpulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
	}
}

// warmStartRevoluteJoint is b3WarmStartRevoluteJoint.
func warmStartRevoluteJoint(base *jointSim, context *stepContext) {
	assert(base.typ == RevoluteJoint, "base->type == b3_revoluteJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.revoluteJoint

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

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	angularImpulse :=
		Add(MulSV(joint.perpImpulse.X, joint.perpAxisX), MulSV(joint.perpImpulse.Y, joint.perpAxisY))
	angularImpulse = MulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)

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

// solveRevoluteJoint is b3SolveRevoluteJoint.
func solveRevoluteJoint(base *jointSim, context *stepContext, useBias bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.revoluteJoint
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

	if fixedRotation == false && (joint.enableSpring || joint.enableMotor || joint.enableLimit) {
		axis := joint.rotationAxisZ
		gA := MulMV(iA, axis)
		gB := MulMV(iB, axis)
		k := Dot(axis, Add(gA, gB))
		axialCdot := Dot(Sub(wB, wA), axis)
		var axialImpulse float32 = 0.0

		// Solve spring
		if joint.enableSpring {
			// Get the substep relative rotation
			targetAngle := joint.targetAngle
			angle := GetTwistAngle(relQ)
			c := angle - targetAngle

			bias := float32(joint.springSoftness.biasRate * c)
			massScale := joint.springSoftness.massScale
			impulseScale := joint.springSoftness.impulseScale

			deltaImpulse := float32(-massScale*joint.axialMass*(axialCdot+bias)) - float32(impulseScale*joint.springImpulse)
			joint.springImpulse += deltaImpulse

			axialCdot += float32(k * deltaImpulse)
			axialImpulse += deltaImpulse
		}

		if joint.enableMotor {
			cdot := axialCdot - joint.motorSpeed
			oldImpulse := joint.motorImpulse
			maxImpulse := float32(joint.maxMotorTorque * jointIntervalH(context))
			deltaImpulse := ClampFloat(float32(-joint.axialMass*cdot), -maxImpulse-oldImpulse, maxImpulse-oldImpulse)
			joint.motorImpulse = oldImpulse + deltaImpulse

			axialCdot += float32(k * deltaImpulse)
			axialImpulse += deltaImpulse
		}

		if joint.enableLimit {
			angle := GetTwistAngle(relQ)

			// todo does an updated twist axis help?

			// Lower limit
			{
				c := angle - joint.lowerAngle
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

				oldImpulse := joint.lowerImpulse
				deltaImpulse :=
					MaxFloat(float32(-massScale*joint.axialMass*(axialCdot+bias))-float32(impulseScale*oldImpulse), -oldImpulse)
				joint.lowerImpulse = oldImpulse + deltaImpulse

				axialCdot += float32(k * deltaImpulse)
				axialImpulse += deltaImpulse
			}

			// Upper limit
			{
				c := joint.upperAngle - angle
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
				cdot := -axialCdot
				oldImpulse := joint.upperImpulse
				deltaImpulse :=
					MaxFloat(float32(-massScale*joint.axialMass*(cdot+bias))-float32(impulseScale*oldImpulse), -oldImpulse)
				joint.upperImpulse = oldImpulse + deltaImpulse

				// sign flipped on applied impulse
				axialImpulse -= deltaImpulse
			}
		}

		wA = MulSub(wA, axialImpulse, gA)
		wB = MulAdd(wB, axialImpulse, gB)
	}

	// Collinearity constraint
	if fixedRotation == false {
		bias := Vec2{0.0, 0.0}
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0

		if useBias {
			c := Vec2{relQ.V.X, relQ.V.Y}
			bias = mulSV2(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		// Collinearity constraint as 2-by-2
		perpAxisX :=
			MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
		perpAxisY :=
			MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))
		joint.perpAxisX = perpAxisX
		joint.perpAxisY = perpAxisY

		invInertiaSum := AddMM(iA, iB)
		kxx := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisX))
		kyy := Dot(perpAxisY, MulMV(invInertiaSum, perpAxisY))
		kxy := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisY))

		k := matrix2{Vec2{kxx, kxy}, Vec2{kxy, kyy}}

		wRel := Sub(wB, wA)
		cdot := Vec2{Dot(wRel, perpAxisX), Dot(wRel, perpAxisY)}
		oldImpulse := joint.perpImpulse
		sol := solve2(k, add2(cdot, bias))
		deltaImpulse := sub2(mulSV2(-massScale, sol), mulSV2(impulseScale, oldImpulse))
		joint.perpImpulse = add2(joint.perpImpulse, deltaImpulse)

		angularImpulse := Add(MulSV(deltaImpulse.X, perpAxisX), MulSV(deltaImpulse.Y, perpAxisY))
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

			separation := Add(Add(Sub(dcB, dcA), Sub(rB, rA)), joint.deltaCenter)

			bias = MulSV(base.constraintSoftness.biasRate, separation)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		b := solvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, Add(cdot, bias))

		impulse := Sub(MulSV(-massScale, b), MulSV(impulseScale, joint.linearImpulse))
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

// drawRevoluteJoint is b3DrawRevoluteJoint.
func drawRevoluteJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)

	length1 := float32(0.1 * scale)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisX))), ColorRed,
		draw.Context)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisY))), ColorGreen,
		draw.Context)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length1, RotateVector(frameA.Q, Vec3AxisZ))), ColorBlue,
		draw.Context)

	frameB := MulWorldTransforms(transformB, base.localFrameB)

	joint := &base.revoluteJoint
	const kSliceCount = 16

	// Twist limit
	if joint.enableLimit {
		quatA := frameA.Q
		quatB := frameB.Q

		if DotQuat(quatA, quatB) < 0.0 {
			// this keeps the twist angle in the range [-pi, pi]
			quatB = NegateQuat(quatB)
		}

		relQ := InvMulQuat(quatA, quatB)

		wedgeRadius := float32(0.2 * scale)
		for index := 0; index < kSliceCount; index++ {
			// PORT: the quotient is wrapped in float32(): Go turns the division by 16 into a multiply and
			// fused it with 1.0 - t1.
			t1 := float32(float32(index+0) / kSliceCount)
			alpha1 := float32((1.0-t1)*joint.lowerAngle) + float32(t1*joint.upperAngle)
			t2 := float32(float32(index+1) / kSliceCount)
			alpha2 := float32((1.0-t2)*joint.lowerAngle) + float32(t2*joint.upperAngle)

			vertex1 := Vec3{float32(wedgeRadius * Cos(alpha1)), float32(wedgeRadius * Sin(alpha1)), 0.0}
			vertex2 := Vec3{float32(wedgeRadius * Cos(alpha2)), float32(wedgeRadius * Sin(alpha2)), 0.0}

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
		p2 := Vec3{float32(wedgeRadius * Cos(twistAngle)), float32(wedgeRadius * Sin(twistAngle)), 0.0}
		draw.DrawSegmentFcn(frameA.P, TransformWorldPoint(frameA, p2), ColorYellow, draw.Context)
	}
}
