// Port of src/wheel_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3WheelJoint_* functions of box3d.h are methods on JointID named WheelJoint<Method>.

// WheelJointEnableSuspension is b3WheelJoint_EnableSuspension.
// Enable/disable the wheel joint spring.
func (jointId JointID) WheelJointEnableSuspension(enableSpring bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointEnableSuspension{jointId, enableSpring}
		recWriteWheelJointEnableSuspension(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)

	if enableSpring != joint.wheelJoint.enableSuspensionSpring {
		joint.wheelJoint.enableSuspensionSpring = enableSpring
		joint.wheelJoint.suspensionSpringImpulse = 0.0
	}
}

// WheelJointIsSuspensionEnabled is b3WheelJoint_IsSuspensionEnabled.
// Is the wheel joint spring enabled?
func (jointId JointID) WheelJointIsSuspensionEnabled() bool {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.enableSuspensionSpring
}

// WheelJointSetSuspensionHertz is b3WheelJoint_SetSuspensionHertz.
// Set the wheel joint stiffness in Hertz.
func (jointId JointID) WheelJointSetSuspensionHertz(hertz float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSuspensionHertz{jointId, hertz}
		recWriteWheelJointSetSuspensionHertz(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.suspensionHertz = hertz
}

// WheelJointGetSuspensionHertz is b3WheelJoint_GetSuspensionHertz.
// Get the wheel joint stiffness in Hertz.
func (jointId JointID) WheelJointGetSuspensionHertz() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.suspensionHertz
}

// WheelJointSetSuspensionDampingRatio is b3WheelJoint_SetSuspensionDampingRatio.
// Set the wheel joint damping ratio, non-dimensional.
func (jointId JointID) WheelJointSetSuspensionDampingRatio(dampingRatio float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSuspensionDampingRatio{jointId, dampingRatio}
		recWriteWheelJointSetSuspensionDampingRatio(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.suspensionDampingRatio = dampingRatio
}

// WheelJointGetSuspensionDampingRatio is b3WheelJoint_GetSuspensionDampingRatio.
// Get the wheel joint damping ratio, non-dimensional.
func (jointId JointID) WheelJointGetSuspensionDampingRatio() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.suspensionDampingRatio
}

// WheelJointEnableSuspensionLimit is b3WheelJoint_EnableSuspensionLimit.
// Enable/disable the wheel joint limit.
func (jointId JointID) WheelJointEnableSuspensionLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointEnableSuspensionLimit{jointId, enableLimit}
		recWriteWheelJointEnableSuspensionLimit(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	if joint.wheelJoint.enableSuspensionLimit != enableLimit {
		joint.wheelJoint.lowerSuspensionImpulse = 0.0
		joint.wheelJoint.upperSuspensionImpulse = 0.0
		joint.wheelJoint.enableSuspensionLimit = enableLimit
	}
}

// WheelJointIsSuspensionLimitEnabled is b3WheelJoint_IsSuspensionLimitEnabled.
// Is the wheel joint limit enabled?
func (jointId JointID) WheelJointIsSuspensionLimitEnabled() bool {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.enableSuspensionLimit
}

// WheelJointGetLowerSuspensionLimit is b3WheelJoint_GetLowerSuspensionLimit.
// Get the wheel joint lower limit.
func (jointId JointID) WheelJointGetLowerSuspensionLimit() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.lowerSuspensionLimit
}

// WheelJointGetUpperSuspensionLimit is b3WheelJoint_GetUpperSuspensionLimit.
// Get the wheel joint upper limit.
func (jointId JointID) WheelJointGetUpperSuspensionLimit() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.upperSuspensionLimit
}

// WheelJointSetSuspensionLimits is b3WheelJoint_SetSuspensionLimits.
// Set the wheel joint limits.
func (jointId JointID) WheelJointSetSuspensionLimits(lower, upper float32) {
	assert(lower <= upper, "lower <= upper")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSuspensionLimits{jointId, lower, upper}
		recWriteWheelJointSetSuspensionLimits(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	if lower != joint.wheelJoint.lowerSuspensionLimit || upper != joint.wheelJoint.upperSuspensionLimit {
		joint.wheelJoint.lowerSuspensionLimit = lower
		joint.wheelJoint.upperSuspensionLimit = upper
		joint.wheelJoint.lowerSuspensionImpulse = 0.0
		joint.wheelJoint.upperSuspensionImpulse = 0.0
	}
}

// WheelJointEnableSpinMotor is b3WheelJoint_EnableSpinMotor.
// Enable/disable the wheel joint motor.
func (jointId JointID) WheelJointEnableSpinMotor(enableMotor bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointEnableSpinMotor{jointId, enableMotor}
		recWriteWheelJointEnableSpinMotor(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	if joint.wheelJoint.enableSpinMotor != enableMotor {
		joint.wheelJoint.spinImpulse = 0.0
		joint.wheelJoint.enableSpinMotor = enableMotor
	}
}

// WheelJointIsSpinMotorEnabled is b3WheelJoint_IsSpinMotorEnabled.
// Is the wheel joint motor enabled?
func (jointId JointID) WheelJointIsSpinMotorEnabled() bool {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.enableSpinMotor
}

// WheelJointSetSpinMotorSpeed is b3WheelJoint_SetSpinMotorSpeed.
// Set the wheel joint motor speed in radians per second.
func (jointId JointID) WheelJointSetSpinMotorSpeed(motorSpeed float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSpinMotorSpeed{jointId, motorSpeed}
		recWriteWheelJointSetSpinMotorSpeed(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.spinSpeed = motorSpeed
}

// WheelJointGetSpinMotorSpeed is b3WheelJoint_GetSpinMotorSpeed.
// Get the wheel joint motor speed in radians per second.
func (jointId JointID) WheelJointGetSpinMotorSpeed() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.spinSpeed
}

// WheelJointSetMaxSpinTorque is b3WheelJoint_SetMaxSpinTorque.
// Set the wheel joint maximum motor torque, usually in newton-meters.
func (jointId JointID) WheelJointSetMaxSpinTorque(torque float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetMaxSpinTorque{jointId, torque}
		recWriteWheelJointSetMaxSpinTorque(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.maxSpinTorque = torque
}

// WheelJointGetMaxSpinTorque is b3WheelJoint_GetMaxSpinTorque.
// Get the wheel joint maximum motor torque, usually in newton-meters.
func (jointId JointID) WheelJointGetMaxSpinTorque() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.maxSpinTorque
}

// WheelJointEnableSteering is b3WheelJoint_EnableSteering.
// Enable/disable wheel steering. Steering allows the wheel to rotate about the suspension axis.
func (jointId JointID) WheelJointEnableSteering(flag bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointEnableSteering{jointId, flag}
		recWriteWheelJointEnableSteering(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	if joint.wheelJoint.enableSteering != flag {
		joint.wheelJoint.angularImpulse = Vec2{0.0, 0.0}
		joint.wheelJoint.enableSteering = flag
	}
}

// WheelJointIsSteeringEnabled is b3WheelJoint_IsSteeringEnabled.
// Can the wheel steer?
func (jointId JointID) WheelJointIsSteeringEnabled() bool {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.enableSteering
}

// WheelJointSetSteeringHertz is b3WheelJoint_SetSteeringHertz.
// Set the wheel joint steering stiffness in Hertz.
func (jointId JointID) WheelJointSetSteeringHertz(hertz float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSteeringHertz{jointId, hertz}
		recWriteWheelJointSetSteeringHertz(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.steeringHertz = hertz
}

// WheelJointGetSteeringHertz is b3WheelJoint_GetSteeringHertz.
// Get the wheel joint steering stiffness in Hertz.
func (jointId JointID) WheelJointGetSteeringHertz() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.steeringHertz
}

// WheelJointSetSteeringDampingRatio is b3WheelJoint_SetSteeringDampingRatio.
// Set the wheel joint steering damping ratio, non-dimensional.
func (jointId JointID) WheelJointSetSteeringDampingRatio(dampingRatio float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSteeringDampingRatio{jointId, dampingRatio}
		recWriteWheelJointSetSteeringDampingRatio(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.steeringDampingRatio = dampingRatio
}

// WheelJointGetSteeringDampingRatio is b3WheelJoint_GetSteeringDampingRatio.
// Get the wheel joint steering damping ratio, non-dimensional.
func (jointId JointID) WheelJointGetSteeringDampingRatio() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.steeringDampingRatio
}

// WheelJointSetMaxSteeringTorque is b3WheelJoint_SetMaxSteeringTorque.
// Set the wheel joint maximum steering torque in N*m.
func (jointId JointID) WheelJointSetMaxSteeringTorque(maxTorque float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetMaxSteeringTorque{jointId, maxTorque}
		recWriteWheelJointSetMaxSteeringTorque(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.maxSteeringTorque = maxTorque
}

// WheelJointGetMaxSteeringTorque is b3WheelJoint_GetMaxSteeringTorque.
// Get the wheel joint maximum steering torque in N*m.
func (jointId JointID) WheelJointGetMaxSteeringTorque() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.maxSteeringTorque
}

// WheelJointEnableSteeringLimit is b3WheelJoint_EnableSteeringLimit.
// Enable/disable the wheel joint steering limit.
func (jointId JointID) WheelJointEnableSteeringLimit(flag bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointEnableSteeringLimit{jointId, flag}
		recWriteWheelJointEnableSteeringLimit(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	if joint.wheelJoint.enableSteeringLimit != flag {
		joint.wheelJoint.lowerSteeringImpulse = 0.0
		joint.wheelJoint.upperSteeringImpulse = 0.0
		joint.wheelJoint.enableSteeringLimit = flag
	}
}

// WheelJointIsSteeringLimitEnabled is b3WheelJoint_IsSteeringLimitEnabled.
// Is the wheel joint steering limit enabled?
func (jointId JointID) WheelJointIsSteeringLimitEnabled() bool {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.enableSteeringLimit
}

// WheelJointGetLowerSteeringLimit is b3WheelJoint_GetLowerSteeringLimit.
// Get the wheel joint lower steering limit in radians.
func (jointId JointID) WheelJointGetLowerSteeringLimit() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.lowerSteeringLimit
}

// WheelJointGetUpperSteeringLimit is b3WheelJoint_GetUpperSteeringLimit.
// Get the wheel joint upper steering limit in radians.
func (jointId JointID) WheelJointGetUpperSteeringLimit() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.upperSteeringLimit
}

// WheelJointSetSteeringLimits is b3WheelJoint_SetSteeringLimits.
// Set the wheel joint steering limits in radians.
func (jointId JointID) WheelJointSetSteeringLimits(lowerRadians, upperRadians float32) {
	assert(lowerRadians <= upperRadians, "lowerRadians <= upperRadians")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetSteeringLimits{jointId, lowerRadians, upperRadians}
		recWriteWheelJointSetSteeringLimits(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.lowerSteeringLimit = lowerRadians
	joint.wheelJoint.upperSteeringLimit = upperRadians
}

// WheelJointSetTargetSteeringAngle is b3WheelJoint_SetTargetSteeringAngle.
// Set the wheel joint target steering angle in radians.
func (jointId JointID) WheelJointSetTargetSteeringAngle(radians float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsWheelJointSetTargetSteeringAngle{jointId, radians}
		recWriteWheelJointSetTargetSteeringAngle(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, WheelJoint)
	joint.wheelJoint.targetSteeringAngle = radians
}

// WheelJointGetTargetSteeringAngle is b3WheelJoint_GetTargetSteeringAngle.
// Get the wheel joint target steering angle in radians.
func (jointId JointID) WheelJointGetTargetSteeringAngle() float32 {
	joint := getJointSimCheckType(jointId, WheelJoint)
	return joint.wheelJoint.targetSteeringAngle
}

// WheelJointGetSpinSpeed is b3WheelJoint_GetSpinSpeed.
// Get the current spin speed in radians per second.
func (jointId JointID) WheelJointGetSpinSpeed() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, WheelJoint)

	idA := int(base.bodyIdA)
	idB := int(base.bodyIdB)

	bodyA := arrayGet(world.bodies, idA)
	bodyB := arrayGet(world.bodies, idB)

	setB := arrayGet(world.solverSets, int(bodyB.setIndex))
	localIndexB := int(bodyB.localIndex)
	bodySimB := arrayGet(setB.bodySims, localIndexB)

	quatB := MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	spinAxis := RotateVector(quatB, Vec3AxisZ)

	wA := Vec3Zero
	stateA := getBodyState(world, bodyA)
	if stateA != nil {
		wA = stateA.angularVelocity
	}

	wB := Vec3Zero
	stateB := getBodyState(world, bodyB)
	if stateB != nil {
		wB = stateB.angularVelocity
	}

	speed := Dot(Sub(wB, wA), spinAxis)
	return speed
}

// WheelJointGetSpinTorque is b3WheelJoint_GetSpinTorque.
// Get the wheel joint current motor torque, usually in newton-meters.
func (jointId JointID) WheelJointGetSpinTorque() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointSimCheckType(jointId, WheelJoint)
	return float32(world.inv_h * joint.wheelJoint.spinImpulse)
}

// WheelJointGetSteeringAngle is b3WheelJoint_GetSteeringAngle.
// Get the current steering angle in radians.
func (jointId JointID) WheelJointGetSteeringAngle() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, WheelJoint)

	idA := int(base.bodyIdA)
	idB := int(base.bodyIdB)

	bodyA := arrayGet(world.bodies, idA)
	bodyB := arrayGet(world.bodies, idB)

	setA := arrayGet(world.solverSets, int(bodyA.setIndex))
	setB := arrayGet(world.solverSets, int(bodyB.setIndex))

	localIndexA := int(bodyA.localIndex)
	localIndexB := int(bodyB.localIndex)

	bodySimA := arrayGet(setA.bodySims, localIndexA)
	bodySimB := arrayGet(setB.bodySims, localIndexB)

	quatA := MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	quatB := MulQuat(bodySimB.transform.Q, base.localFrameB.Q)

	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	// Twist around x-axis
	cs := Dot(matrixB.Cz, matrixA.Cz)
	ss := -Dot(matrixB.Cz, matrixA.Cy)

	return Atan2(ss, cs)
}

// WheelJointGetSteeringTorque is b3WheelJoint_GetSteeringTorque.
// Get the current steering torque in N*m.
func (jointId JointID) WheelJointGetSteeringTorque() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointSimCheckType(jointId, WheelJoint)
	return float32(world.inv_h * joint.wheelJoint.steeringSpringImpulse)
}

// getWheelJointForce is b3GetWheelJointForce.
func getWheelJointForce(world *world, base *jointSim) Vec3 {
	transformA := getBodyTransform(world, int(base.bodyIdA))
	joint := &base.wheelJoint

	// impulse in joint space
	// PORT: upstream bug kept: the axial component adds lowerSuspensionLimit (a length) and
	// upperSuspensionImpulse instead of lowerSuspensionImpulse - upperSuspensionImpulse.
	impulse := Vec3{
		joint.linearImpulse.X,
		joint.linearImpulse.Y,
		joint.lowerSuspensionLimit + joint.upperSuspensionImpulse + joint.suspensionSpringImpulse,
	}

	// convert impulse to force
	force := MulSV(world.inv_h, impulse)

	// convert to body space
	force = RotateVector(base.localFrameA.Q, force)

	// convert to world space
	force = RotateVector(transformA.Q, force)
	return force
}

// getWheelJointTorque is b3GetWheelJointTorque.
func getWheelJointTorque(world *world, base *jointSim) Vec3 {
	assert(base.typ == WheelJoint, "base->type == b3_wheelJoint")

	// chase body id to the solver set where the body lives
	idA := int(base.bodyIdA)
	// int idB = base->bodyIdB;

	bodyA := arrayGet(world.bodies, idA)
	// b3Body* bodyB = b3Array_Get( world->bodies, idB  );

	setA := arrayGet(world.solverSets, int(bodyA.setIndex))
	// b3SolverSet* setB = b3Array_Get( world->solverSets, bodyB->setIndex  );

	localIndexA := int(bodyA.localIndex)
	// int localIndexB = bodyB->localIndex;

	bodySimA := arrayGet(setA.bodySims, localIndexA)
	// b3BodySim* bodySimB = b3Array_Get( setB->bodySims, localIndexB  );

	qA := MulQuat(bodySimA.transform.Q, base.localFrameA.Q)

	matrixA := MakeMatrixFromQuat(qA)

	return MulSV(float32(world.inv_h*base.wheelJoint.spinImpulse), matrixA.Cz)
}

// See constraints.pdf

// prepareWheelJoint is b3PrepareWheelJoint.
func prepareWheelJoint(base *jointSim, context *stepContext) {
	assert(base.typ == WheelJoint, "base->type == b3_wheelJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.wheelJoint

	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rA := joint.frameA.P
	rB := joint.frameB.P

	matrixA := MakeMatrixFromQuat(joint.frameA.Q)
	matrixB := MakeMatrixFromQuat(joint.frameB.Q)

	// todo use fresh effective masses in the sub-step to avoid divergence like I saw for the prismatic joint

	{
		suspensionAxis := matrixA.Cx
		rAn := Cross(rA, suspensionAxis)
		rBn := Cross(rB, suspensionAxis)

		k := base.invMassA + base.invMassB + Dot(rAn, MulMV(base.invIA, rAn)) +
			Dot(rBn, MulMV(base.invIB, rBn))
		joint.suspensionMass = 0.0
		if k > 0.0 {
			joint.suspensionMass = 1.0 / k
		}
	}

	joint.suspensionSoftness = makeSoft(joint.suspensionHertz, joint.suspensionDampingRatio, jointIntervalH(context))
	joint.steeringSoftness = makeSoft(joint.steeringHertz, joint.steeringDampingRatio, jointIntervalH(context))

	{
		// Rotation axis is the z-axis of body A.
		spinAxis := matrixB.Cz
		k := Dot(spinAxis, MulMV(invInertiaSum, spinAxis))
		joint.spinMass = 0.0
		if k > 0.0 {
			joint.spinMass = 1.0 / k
		}
	}

	{
		// Twist constraint around x-axis
		cs := Dot(matrixB.Cz, matrixA.Cz)
		ss := -Dot(matrixB.Cz, matrixA.Cy)
		den := float32(cs*cs) + float32(ss*ss)
		if den > 0.0 {
			den = 1.0 / den
		} else {
			den = 0.0
		}
		steeringAxis :=
			MulSV(den, Cross(matrixB.Cz, Sub(MulSV(-cs, matrixA.Cy), MulSV(ss, matrixA.Cz))))

		k := Dot(steeringAxis, MulMV(invInertiaSum, steeringAxis))
		joint.steeringMass = 0.0
		if k > 0.0 {
			joint.steeringMass = 1.0 / k
		}
	}

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.linearImpulse = Vec2{0.0, 0.0}
		joint.angularImpulse = Vec2{0.0, 0.0}
		joint.spinImpulse = 0.0
		joint.suspensionSpringImpulse = 0.0
		joint.lowerSuspensionImpulse = 0.0
		joint.upperSuspensionImpulse = 0.0
		joint.steeringSpringImpulse = 0.0
		joint.lowerSteeringImpulse = 0.0
		joint.upperSteeringImpulse = 0.0
	}
}

// warmStartWheelJoint is b3WarmStartWheelJoint.
func warmStartWheelJoint(base *jointSim, context *stepContext) {
	assert(base.typ == WheelJoint, "base->type == b3_wheelJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.wheelJoint

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

	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.Q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.Q)
	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	sAx := Cross(Add(d, rA), matrixA.Cx)
	sBx := Cross(rB, matrixA.Cx)
	sAy := Cross(Add(d, rA), matrixA.Cy)
	sBy := Cross(rB, matrixA.Cy)
	sAz := Cross(Add(d, rA), matrixA.Cz)
	sBz := Cross(rB, matrixA.Cz)

	suspensionImpulse := joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse

	linearImpulseY := joint.linearImpulse.X
	linearImpulseZ := joint.linearImpulse.Y
	angularImpulseX := joint.angularImpulse.X
	angularImpulseY := joint.angularImpulse.Y

	linearImpulse := blend3(suspensionImpulse, matrixA.Cx, linearImpulseY, matrixA.Cy, linearImpulseZ, matrixA.Cz)
	angularImpulseA := blend3(suspensionImpulse, sAx, linearImpulseY, sAy, linearImpulseZ, sAz)
	angularImpulseB := blend3(suspensionImpulse, sBx, linearImpulseY, sBy, linearImpulseZ, sBz)
	angularImpulse := MulSV(joint.spinImpulse, matrixA.Cz)

	spinAxis := matrixB.Cz

	if joint.enableSteering {
		// Twist constraint around x-axis
		cs := Dot(matrixB.Cz, matrixA.Cz)
		ss := -Dot(matrixB.Cz, matrixA.Cy)
		den := float32(cs*cs) + float32(ss*ss)
		if den > 0.0 {
			den = 1.0 / den
		} else {
			den = 0.0
		}
		steeringAxis :=
			MulSV(den, Cross(matrixB.Cz, Sub(MulSV(-cs, matrixA.Cy), MulSV(ss, matrixA.Cz))))

		perpAxis := Cross(spinAxis, matrixA.Cx)
		steeringImpulse := joint.steeringSpringImpulse + joint.lowerSteeringImpulse - joint.upperSteeringImpulse
		angularImpulse = blend3(angularImpulseX, perpAxis, joint.spinImpulse, spinAxis, steeringImpulse, steeringAxis)
	} else {
		relQ := InvMulQuat(quatA, quatB)
		perpAxisX := MulSV(
			0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
		perpAxisY := MulSV(
			0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))
		angularImpulse = Add(
			angularImpulse,
			blend3(angularImpulseX, perpAxisX, angularImpulseY, perpAxisY, joint.spinImpulse, spinAxis))
	}

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, linearImpulse)
		stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Add(angularImpulseA, angularImpulse)))
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, linearImpulse)
		stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Add(angularImpulseB, angularImpulse)))
	}
}

// solveWheelJoint is b3SolveWheelJoint.
func solveWheelJoint(base *jointSim, context *stepContext, useBias bool) {
	assert(base.typ == WheelJoint, "base->type == b3_wheelJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.wheelJoint

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

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	quatA := MulQuat(stateA.deltaRotation, joint.frameA.Q)
	quatB := MulQuat(stateB.deltaRotation, joint.frameB.Q)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)
	matrixA := MakeMatrixFromQuat(quatA)
	matrixB := MakeMatrixFromQuat(quatB)

	// b3Vec3 spinAxis = b3RotateVector( quatB, b3Vec3_axisZ );

	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))
	sAx := Cross(Add(d, rA), matrixA.Cx)
	sBx := Cross(rB, matrixA.Cx)
	sAy := Cross(Add(d, rA), matrixA.Cy)
	sBy := Cross(rB, matrixA.Cy)
	sAz := Cross(Add(d, rA), matrixA.Cz)
	sBz := Cross(rB, matrixA.Cz)

	translation := Dot(matrixA.Cx, d)

	// Steering param ib = cz_b, ia = cz_a, ja = -cy_a
	cs := Dot(matrixB.Cz, matrixA.Cz)
	ss := -Dot(matrixB.Cz, matrixA.Cy)
	den := float32(cs*cs) + float32(ss*ss)
	if den > 0.0 {
		den = 1.0 / den
	} else {
		den = 0.0
	}
	steeringAxis :=
		MulSV(den, Cross(matrixB.Cz, Sub(MulSV(-cs, matrixA.Cy), MulSV(ss, matrixA.Cz))))

	// motor constraint
	if joint.enableSpinMotor && fixedRotation == false {
		spinAxis := matrixB.Cz
		cdot := Dot(Sub(wB, wA), spinAxis) - joint.spinSpeed
		impulse := float32(-joint.spinMass * cdot)
		oldImpulse := joint.spinImpulse
		maxImpulse := float32(jointIntervalH(context) * joint.maxSpinTorque)
		joint.spinImpulse = ClampFloat(joint.spinImpulse+impulse, -maxImpulse, maxImpulse)
		impulse = joint.spinImpulse - oldImpulse

		wA = Sub(wA, MulMV(iA, MulSV(impulse, spinAxis)))
		wB = Add(wB, MulMV(iB, MulSV(impulse, spinAxis)))
	}

	// suspension
	if joint.enableSuspensionSpring {
		// This is a real spring and should be applied even during relax
		c := translation
		bias := float32(joint.suspensionSoftness.biasRate * c)
		massScale := joint.suspensionSoftness.massScale
		impulseScale := joint.suspensionSoftness.impulseScale

		cdot := Dot(matrixA.Cx, Sub(vB, vA)) + Dot(sBx, wB) - Dot(sAx, wA)
		impulse := float32(float32(-massScale*joint.suspensionMass)*(cdot+bias)) - float32(impulseScale*joint.suspensionSpringImpulse)
		joint.suspensionSpringImpulse += impulse

		linearImpulse := MulSV(impulse, matrixA.Cx)
		angularImpulseA := MulSV(impulse, sAx)
		angularImpulseB := MulSV(impulse, sBx)

		vA = MulSub(vA, mA, linearImpulse)
		wA = Sub(wA, MulMV(iA, angularImpulseA))
		vB = MulAdd(vB, mB, linearImpulse)
		wB = Add(wB, MulMV(iB, angularImpulseB))
	}

	// steering
	if joint.enableSteering && fixedRotation == false {
		steeringAngle := Atan2(ss, cs)

		{
			// This is a real spring and should be applied even during relax
			c := steeringAngle - joint.targetSteeringAngle
			bias := float32(joint.steeringSoftness.biasRate * c)
			massScale := joint.steeringSoftness.massScale
			impulseScale := joint.steeringSoftness.impulseScale

			cdot := Dot(steeringAxis, Sub(wB, wA))
			oldImpulse := joint.steeringSpringImpulse
			impulse := float32(float32(-massScale*joint.steeringMass)*(cdot+bias)) - float32(impulseScale*oldImpulse)
			maxImpulse := float32(jointIntervalH(context) * joint.maxSteeringTorque)
			joint.steeringSpringImpulse = ClampFloat(oldImpulse+impulse, -maxImpulse, maxImpulse)
			impulse = joint.steeringSpringImpulse - oldImpulse

			wA = Sub(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
			wB = Add(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
		}

		if joint.enableSteeringLimit {
			// Lower limit
			{
				c := steeringAngle - joint.lowerSteeringLimit
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

				cdot := Dot(steeringAxis, Sub(wB, wA))
				oldImpulse := joint.lowerSteeringImpulse
				impulse := float32(float32(-massScale*joint.steeringMass)*(cdot+bias)) - float32(impulseScale*oldImpulse)
				joint.lowerSteeringImpulse = MaxFloat(oldImpulse+impulse, 0.0)
				impulse = joint.lowerSteeringImpulse - oldImpulse

				wA = Sub(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
				wB = Add(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
			}

			// Upper limit
			// Note: signs are flipped to keep c positive when the constraint is satisfied.
			// This also keeps the impulse positive when the limit is active.
			{
				// sign flipped
				c := joint.upperSteeringLimit - steeringAngle
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

				// sign flipped on cdot
				cdot := Dot(steeringAxis, Sub(wA, wB))
				oldImpulse := joint.upperSteeringImpulse
				impulse := float32(float32(-massScale*joint.steeringMass)*(cdot+bias)) - float32(impulseScale*oldImpulse)
				joint.upperSteeringImpulse = MaxFloat(oldImpulse+impulse, 0.0)
				impulse = joint.upperSteeringImpulse - oldImpulse

				// sign flipped on applied impulse
				wA = Add(wA, MulMV(iA, MulSV(impulse, steeringAxis)))
				wB = Sub(wB, MulMV(iB, MulSV(impulse, steeringAxis)))
			}
		}
	}

	if joint.enableSuspensionLimit {
		// Lower limit
		{
			c := translation - joint.lowerSuspensionLimit
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

			cdot := Dot(matrixA.Cx, Sub(vB, vA)) + Dot(sBx, wB) - Dot(sAx, wA)
			impulse := float32(float32(-massScale*joint.suspensionMass)*(cdot+bias)) - float32(impulseScale*joint.lowerSuspensionImpulse)
			oldImpulse := joint.lowerSuspensionImpulse
			joint.lowerSuspensionImpulse = MaxFloat(oldImpulse+impulse, 0.0)
			impulse = joint.lowerSuspensionImpulse - oldImpulse

			linearImpulse := MulSV(impulse, matrixA.Cx)
			angularImpulseA := MulSV(impulse, sAx)
			angularImpulseB := MulSV(impulse, sBx)

			vA = MulSub(vA, mA, linearImpulse)
			wA = Sub(wA, MulMV(iA, angularImpulseA))
			vB = MulAdd(vB, mB, linearImpulse)
			wB = Add(wB, MulMV(iB, angularImpulseB))
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			c := joint.upperSuspensionLimit - translation
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

			// sign flipped on cdot
			cdot := Dot(matrixA.Cx, Sub(vA, vB)) + Dot(sAx, wA) - Dot(sBx, wB)
			impulse := float32(float32(-massScale*joint.suspensionMass)*(cdot+bias)) - float32(impulseScale*joint.upperSuspensionImpulse)
			oldImpulse := joint.upperSuspensionImpulse
			joint.upperSuspensionImpulse = MaxFloat(oldImpulse+impulse, 0.0)
			impulse = joint.upperSuspensionImpulse - oldImpulse

			linearImpulse := MulSV(impulse, matrixA.Cx)
			angularImpulseA := MulSV(impulse, sAx)
			angularImpulseB := MulSV(impulse, sBx)

			// sign flipped on applied impulse
			vA = MulAdd(vA, mA, linearImpulse)
			wA = Add(wA, MulMV(iA, angularImpulseA))
			vB = MulSub(vB, mB, linearImpulse)
			wB = Sub(wB, MulMV(iB, angularImpulseB))
		}
	}

	// Collinearity constraint
	if fixedRotation == false {
		if joint.enableSteering == true {
			var bias float32 = 0.0
			var massScale float32 = 1.0
			var impulseScale float32 = 0.0
			if useBias {
				c := Dot(matrixA.Cx, matrixB.Cz)

				bias = float32(base.constraintSoftness.biasRate * c)
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			u := Cross(matrixB.Cz, matrixA.Cx)
			cdot := Dot(Sub(wB, wA), u)

			invInertiaSum := AddMM(iA, iB)
			k := Dot(u, MulMV(invInertiaSum, u))
			var perpMass float32 = 0.0
			if k > 0.0 {
				perpMass = 1.0 / k
			}

			deltaImpulse := float32(float32(-massScale*perpMass)*(cdot+bias)) - float32(impulseScale*joint.angularImpulse.X)
			joint.angularImpulse.X += deltaImpulse

			wA = MulSub(wA, deltaImpulse, MulMV(iA, u))
			wB = MulAdd(wB, deltaImpulse, MulMV(iB, u))
		} else {
			bias := Vec2{0.0, 0.0}
			var massScale float32 = 1.0
			var impulseScale float32 = 0.0

			if useBias {
				c := Vec2{relQ.V.X, relQ.V.Y}
				bias = Vec2{float32(base.constraintSoftness.biasRate * c.X), float32(base.constraintSoftness.biasRate * c.Y)}
				massScale = base.constraintSoftness.massScale
				impulseScale = base.constraintSoftness.impulseScale
			}

			// Collinearity constraint as 2-by-2
			perpAxisX := MulSV(
				0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
			perpAxisY := MulSV(
				0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))

			invInertiaSum := AddMM(iA, iB)
			kxx := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisX))
			kyy := Dot(perpAxisY, MulMV(invInertiaSum, perpAxisY))
			kxy := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisY))

			k := matrix2{Vec2{kxx, kxy}, Vec2{kxy, kyy}}

			wRel := Sub(wB, wA)
			cdot := Vec2{Dot(wRel, perpAxisX), Dot(wRel, perpAxisY)}
			oldImpulse := joint.angularImpulse
			cdotPlusBias := Vec2{cdot.X + bias.X, cdot.Y + bias.Y}
			sol := solve2(k, cdotPlusBias)
			deltaImpulse := Vec2{
				float32(-massScale*sol.X) - float32(impulseScale*oldImpulse.X),
				float32(-massScale*sol.Y) - float32(impulseScale*oldImpulse.Y),
			}
			joint.angularImpulse = Vec2{oldImpulse.X + deltaImpulse.X, oldImpulse.Y + deltaImpulse.Y}

			angularImpulse := Blend2(deltaImpulse.X, perpAxisX, deltaImpulse.Y, perpAxisY)
			wA = Sub(wA, MulMV(iA, angularImpulse))
			wB = Add(wB, MulMV(iB, angularImpulse))
		}
	}

	// Solve point-to-line constraint
	{
		perpY := matrixA.Cy
		perpZ := matrixA.Cz

		bias := Vec2{0.0, 0.0}
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias {
			c := Vec2{Dot(perpY, d), Dot(perpZ, d)}
			bias = Vec2{float32(base.constraintSoftness.biasRate * c.X), float32(base.constraintSoftness.biasRate * c.Y)}
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Vec2{Dot(perpY, vRel), Dot(perpZ, vRel)}

		//// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		///// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]

		kyy := mA + mB + Dot(sAy, MulMV(iA, sAy)) + Dot(sBy, MulMV(iB, sBy))
		kyz := Dot(sAy, MulMV(iA, sAz)) + Dot(sBy, MulMV(iB, sBz))
		kzz := mA + mB + Dot(sAz, MulMV(iA, sAz)) + Dot(sBz, MulMV(iB, sBz))

		k := matrix2{Vec2{kyy, kyz}, Vec2{kyz, kzz}}

		oldImpulse := joint.linearImpulse
		cdotPlusBias := Vec2{cdot.X + bias.X, cdot.Y + bias.Y}
		sol := solve2(k, cdotPlusBias)
		deltaImpulse := Vec2{
			float32(-massScale*sol.X) - float32(impulseScale*oldImpulse.X),
			float32(-massScale*sol.Y) - float32(impulseScale*oldImpulse.Y),
		}
		joint.linearImpulse = Vec2{oldImpulse.X + deltaImpulse.X, oldImpulse.Y + deltaImpulse.Y}

		linearImpulse := Blend2(deltaImpulse.X, perpY, deltaImpulse.Y, perpZ)

		vA = MulSub(vA, mA, linearImpulse)
		wA = Sub(wA, MulMV(iA, Blend2(deltaImpulse.X, sAy, deltaImpulse.Y, sAz)))
		vB = MulAdd(vB, mB, linearImpulse)
		wB = Add(wB, MulMV(iB, Blend2(deltaImpulse.X, sBy, deltaImpulse.Y, sBz)))
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

// PORT: the #if 0 block with b3WheelJoint_Dump is not ported (compiled out upstream).

// drawWheelJoint is b3DrawWheelJoint.
func drawWheelJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	assert(base.typ == WheelJoint, "base->type == b3_wheelJoint")

	joint := &base.wheelJoint

	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	matrixA := MakeMatrixFromQuat(frameA.Q)
	matrixB := MakeMatrixFromQuat(frameB.Q)

	draw.DrawSegmentFcn(frameA.P, frameB.P, ColorBlue, draw.Context)

	if joint.enableSuspensionLimit {
		lower := OffsetPos(frameA.P, MulSV(joint.lowerSuspensionLimit, matrixA.Cx))
		upper := OffsetPos(frameA.P, MulSV(joint.upperSuspensionLimit, matrixA.Cx))
		perp := matrixA.Cy
		draw.DrawSegmentFcn(lower, upper, ColorGray, draw.Context)
		draw.DrawSegmentFcn(OffsetPos(lower, MulSV(-0.1*scale, perp)), OffsetPos(lower, MulSV(0.1*scale, perp)),
			ColorGreen, draw.Context)
		draw.DrawSegmentFcn(OffsetPos(upper, MulSV(-0.1*scale, perp)), OffsetPos(upper, MulSV(0.1*scale, perp)),
			ColorRed, draw.Context)
	} else {
		draw.DrawSegmentFcn(OffsetPos(frameA.P, MulSV(-1.0*scale, matrixA.Cx)),
			OffsetPos(frameA.P, MulSV(1.0*scale, matrixA.Cx)), ColorGray, draw.Context)
	}

	if joint.enableSteering && joint.enableSteeringLimit {
		// b3Quat quatA = frameA.q;
		// b3Quat quatB = frameB.q;

		// if ( b3DotQuat( quatA, quatB ) < 0.0f )
		//{
		//	// this keeps the twist angle in the range [-pi, pi]
		//	quatB = -quatB;
		// }

		// b3Quat relQ = b3InvMulQuat( quatA, quatB );

		frame := WorldTransform{
			P: frameB.P,
			Q: frameA.Q,
		}

		radius := float32(0.5 * scale)
		const sliceCount = 16
		lower := joint.lowerSteeringLimit
		upper := joint.upperSteeringLimit

		cs := ComputeCosSin(lower)
		vertex1 := TransformWorldPoint(frame, Vec3{0.0, float32(-radius * cs.Sine), float32(radius * cs.Cosine)})

		for index := 0; index < sliceCount; index++ {
			t2 := float32((float32(index) + 1.0) / sliceCount)
			phi := LerpFloat(lower, upper, t2)

			cs = ComputeCosSin(phi)
			vertex2 := TransformWorldPoint(frame, Vec3{0.0, float32(-radius * cs.Sine), float32(radius * cs.Cosine)})

			if index == 0 {
				draw.DrawSegmentFcn(frame.P, vertex1, ColorCyan, draw.Context)
			}

			if index == sliceCount-1 {
				draw.DrawSegmentFcn(vertex2, frame.P, ColorCyan, draw.Context)
			}
			draw.DrawSegmentFcn(vertex1, vertex2, ColorCyan, draw.Context)

			vertex1 = vertex2
		}
	}

	draw.DrawSegmentFcn(OffsetPos(frameB.P, MulSV(-0.5*scale, matrixB.Cz)),
		OffsetPos(frameB.P, MulSV(0.5*scale, matrixB.Cz)), ColorMagenta, draw.Context)

	draw.DrawPointFcn(frameA.P, 5.0, ColorGray, draw.Context)
	draw.DrawPointFcn(frameB.P, 5.0, ColorDimGray, draw.Context)
}
