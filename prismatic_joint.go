// Port of src/prismatic_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3PrismaticJoint_* functions of box3d.h are methods on JointID named
// PrismaticJoint<Method>. B3_REC is expanded at the call site (see recording.go).

// Linear constraint (point-to-line)
// joint axis is along joint frame A local z-axis
// perpX and perpY are world vectors fixed in A
//
// d = pB - pA = xB + rB - xA - rA
// Cx = dot(perpX, d)
// Cy = dot(perpY, d)

// CdotX = dot(d, cross(wA, perpX)) + dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perpX, vA) - dot(cross(d + rA, perpX), wA) + dot(perpX, vB) + dot(cross(rB, perpX), vB)
// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
// similar for perpY
//
// Simplification dropping dot(d, cross(wA, perpX)) (todo needs testing)
// CdotXs = dot(perpX, vB + cross(wB, rB) - vA - cross(wA, rA))
// Jxs = [-perpX, -cross(rA, perpX), perpX, cross(rB, perpX)]

// Motor/limit/spring linear constraint
// axis is the world joint axis fixed in A

// C = dot(axis, d)
// Cdot = dot(d, cross(wA, axis)) + dot(axis, vB + cross(wB, rB) - vA - cross(wA, rA))
// Cdot = -dot(axis, vA) - dot(cross(d + rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(d + rA, axis) axis cross(rB, axis)]
//
// Simplified (todo needs testing)
// Cdot = -dot(axis, vA) - dot(cross(rA, axis), wA) + dot(axis, vB) + dot(cross(rB, axis), vB)
// J = [-axis -cross(rA, axis) axis cross(rB, axis)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// PrismaticJointEnableLimit is b3PrismaticJoint_EnableLimit.
// Enable/disable a prismatic joint limit
func (jointId JointID) PrismaticJointEnableLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointEnableLimit{jointId, enableLimit}
		recWritePrismaticJointEnableLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	if enableLimit != base.prismaticJoint.enableLimit {
		base.prismaticJoint.lowerImpulse = 0.0
		base.prismaticJoint.upperImpulse = 0.0
	}
	base.prismaticJoint.enableLimit = enableLimit
}

// PrismaticJointIsLimitEnabled is b3PrismaticJoint_IsLimitEnabled.
// Is the prismatic joint limit enabled?
func (jointId JointID) PrismaticJointIsLimitEnabled() bool {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.enableLimit
}

// PrismaticJointGetLowerLimit is b3PrismaticJoint_GetLowerLimit.
// Get the prismatic joint lower limit
func (jointId JointID) PrismaticJointGetLowerLimit() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.lowerTranslation
}

// PrismaticJointGetUpperLimit is b3PrismaticJoint_GetUpperLimit.
// Get the prismatic joint upper limit
func (jointId JointID) PrismaticJointGetUpperLimit() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.upperTranslation
}

// PrismaticJointSetLimits is b3PrismaticJoint_SetLimits.
// Set the prismatic joint limits
func (jointId JointID) PrismaticJointSetLimits(lower, upper float32) {
	assert(IsValidFloat(lower) && IsValidFloat(upper), "b3IsValidFloat( lower ) && b3IsValidFloat( upper )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetLimits{jointId, lower, upper}
		recWritePrismaticJointSetLimits(world.recording, &recArgs)
	}
	lowerAngle := MinFloat(lower, upper)
	upperAngle := MaxFloat(lower, upper)

	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.lowerTranslation = lowerAngle
	base.prismaticJoint.upperTranslation = upperAngle
}

// PrismaticJointGetTranslation is b3PrismaticJoint_GetTranslation.
// Get the current joint translation, usually in meters.
func (jointId JointID) PrismaticJointGetTranslation() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, PrismaticJoint)
	transformA := getBodyTransform(world, int(base.bodyIdA))
	transformB := getBodyTransform(world, int(base.bodyIdB))

	jointAxis := RotateVector(base.localFrameA.Q, Vec3AxisX)
	jointAxis = RotateVector(transformA.Q, jointAxis)

	anchorA := RotateVector(transformA.Q, base.localFrameA.P)
	anchorB := RotateVector(transformB.Q, base.localFrameB.P)
	d := Add(SubPos(transformB.P, transformA.P), Sub(anchorB, anchorA))
	translation := Dot(d, jointAxis)
	return translation
}

// PrismaticJointEnableSpring is b3PrismaticJoint_EnableSpring.
// Enable/disable the joint spring.
func (jointId JointID) PrismaticJointEnableSpring(enableSpring bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointEnableSpring{jointId, enableSpring}
		recWritePrismaticJointEnableSpring(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	if enableSpring != base.prismaticJoint.enableSpring {
		base.prismaticJoint.springImpulse = 0.0
	}
	base.prismaticJoint.enableSpring = enableSpring
}

// PrismaticJointIsSpringEnabled is b3PrismaticJoint_IsSpringEnabled.
// Is the prismatic joint spring enabled or not?
func (jointId JointID) PrismaticJointIsSpringEnabled() bool {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.enableSpring
}

// PrismaticJointSetTargetTranslation is b3PrismaticJoint_SetTargetTranslation.
// Set the prismatic joint target translation. Usually in meters.
func (jointId JointID) PrismaticJointSetTargetTranslation(targetTranslation float32) {
	assert(IsValidFloat(targetTranslation), "b3IsValidFloat( targetTranslation )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetTargetTranslation{jointId, targetTranslation}
		recWritePrismaticJointSetTargetTranslation(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.targetTranslation = targetTranslation
}

// PrismaticJointGetTargetTranslation is b3PrismaticJoint_GetTargetTranslation.
// Get the prismatic joint target translation. Usually in meters.
func (jointId JointID) PrismaticJointGetTargetTranslation() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.targetTranslation
}

// PrismaticJointSetSpringHertz is b3PrismaticJoint_SetSpringHertz.
// Set the prismatic joint stiffness in Hertz.
// This should usually be less than a quarter of the simulation rate. For example, if the simulation
// runs at 60Hz then the joint stiffness should be 15Hz or less.
func (jointId JointID) PrismaticJointSetSpringHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetSpringHertz{jointId, hertz}
		recWritePrismaticJointSetSpringHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.hertz = hertz
}

// PrismaticJointGetSpringHertz is b3PrismaticJoint_GetSpringHertz.
// Get the prismatic joint stiffness in Hertz
func (jointId JointID) PrismaticJointGetSpringHertz() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.hertz
}

// PrismaticJointSetSpringDampingRatio is b3PrismaticJoint_SetSpringDampingRatio.
// Set the prismatic joint damping ratio (non-dimensional)
func (jointId JointID) PrismaticJointSetSpringDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetSpringDampingRatio{jointId, dampingRatio}
		recWritePrismaticJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.dampingRatio = dampingRatio
}

// PrismaticJointGetSpringDampingRatio is b3PrismaticJoint_GetSpringDampingRatio.
// Get the prismatic spring damping ratio (non-dimensional)
func (jointId JointID) PrismaticJointGetSpringDampingRatio() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.dampingRatio
}

// PrismaticJointEnableMotor is b3PrismaticJoint_EnableMotor.
// Enable/disable a prismatic joint motor
func (jointId JointID) PrismaticJointEnableMotor(enableMotor bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointEnableMotor{jointId, enableMotor}
		recWritePrismaticJointEnableMotor(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	if enableMotor != base.prismaticJoint.enableMotor {
		base.prismaticJoint.motorImpulse = 0.0
	}
	base.prismaticJoint.enableMotor = enableMotor
}

// PrismaticJointIsMotorEnabled is b3PrismaticJoint_IsMotorEnabled.
// Is the prismatic joint motor enabled?
func (jointId JointID) PrismaticJointIsMotorEnabled() bool {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.enableMotor
}

// PrismaticJointSetMotorSpeed is b3PrismaticJoint_SetMotorSpeed.
// Set the prismatic joint motor speed, usually in meters per second
func (jointId JointID) PrismaticJointSetMotorSpeed(motorSpeed float32) {
	assert(IsValidFloat(motorSpeed), "b3IsValidFloat( motorSpeed )")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetMotorSpeed{jointId, motorSpeed}
		recWritePrismaticJointSetMotorSpeed(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.motorSpeed = motorSpeed
}

// PrismaticJointGetMotorSpeed is b3PrismaticJoint_GetMotorSpeed.
// Get the prismatic joint motor speed, usually in meters per second
func (jointId JointID) PrismaticJointGetMotorSpeed() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.motorSpeed
}

// PrismaticJointSetMaxMotorForce is b3PrismaticJoint_SetMaxMotorForce.
// Set the prismatic joint maximum motor force, usually in newtons
func (jointId JointID) PrismaticJointSetMaxMotorForce(maxForce float32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0, "b3IsValidFloat( maxForce ) && maxForce >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsPrismaticJointSetMaxMotorForce{jointId, maxForce}
		recWritePrismaticJointSetMaxMotorForce(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, PrismaticJoint)
	base.prismaticJoint.maxMotorForce = maxForce
}

// PrismaticJointGetMaxMotorForce is b3PrismaticJoint_GetMaxMotorForce.
// Get the prismatic joint maximum motor force, usually in newtons
func (jointId JointID) PrismaticJointGetMaxMotorForce() float32 {
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return base.prismaticJoint.maxMotorForce
}

// PrismaticJointGetMotorForce is b3PrismaticJoint_GetMotorForce.
// Get the prismatic joint current motor force, usually in newtons
func (jointId JointID) PrismaticJointGetMotorForce() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, PrismaticJoint)
	return float32(world.inv_h * base.prismaticJoint.motorImpulse)
}

// PrismaticJointGetSpeed is b3PrismaticJoint_GetSpeed.
// Get the current joint translation speed, usually in meters per second.
func (jointId JointID) PrismaticJointGetSpeed() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, PrismaticJoint)

	bodyA := arrayGet(world.bodies, int(base.bodyIdA))
	bodyB := arrayGet(world.bodies, int(base.bodyIdB))
	bodySimA := getBodySim(world, bodyA)
	bodySimB := getBodySim(world, bodyB)
	stateA := getBodyState(world, bodyA)
	stateB := getBodyState(world, bodyB)

	qA := bodySimA.transform.Q
	qB := bodySimB.transform.Q

	axisA := RotateVector(qA, RotateVector(base.localFrameA.Q, Vec3AxisX))
	rA := RotateVector(qA, Sub(base.localFrameA.P, bodySimA.localCenter))
	rB := RotateVector(qB, Sub(base.localFrameB.P, bodySimB.localCenter))

	// Difference the centers in double so the speed stays exact far from the origin.
	d := Add(SubPos(bodySimB.center, bodySimA.center), Sub(rB, rA))

	vA, vB, wA, wB := Vec3Zero, Vec3Zero, Vec3Zero, Vec3Zero
	if stateA != nil {
		vA = stateA.linearVelocity
		wA = stateA.angularVelocity
	}
	if stateB != nil {
		vB = stateB.linearVelocity
		wB = stateB.angularVelocity
	}

	vRel := Sub(Add(vB, Cross(wB, rB)), Add(vA, Cross(wA, rA)))

	// The axis moves with body A, so account for its rotation.
	speed := Dot(d, Cross(wA, axisA)) + Dot(axisA, vRel)
	return speed
}

// getPrismaticJointForce is b3GetPrismaticJointForce.
// PORT: upstream adds upperImpulse here while b3GetJointReaction and the solver subtract it; kept
// as is (see LOOSE_ENDS.md).
func getPrismaticJointForce(world *world, base *jointSim) Vec3 {
	transformA := getBodyTransform(world, int(base.bodyIdA))
	joint := &base.prismaticJoint

	// impulse in joint space
	impulse := Vec3{
		joint.perpImpulse.X,
		joint.perpImpulse.Y,
		joint.motorImpulse + joint.lowerImpulse + joint.upperImpulse + joint.springImpulse,
	}

	// convert impulse to force
	force := MulSV(world.inv_h, impulse)

	// convert to body space
	force = RotateVector(base.localFrameA.Q, force)

	// convert to world space
	force = RotateVector(transformA.Q, force)
	return force
}

// getPrismaticJointTorque is b3GetPrismaticJointTorque.
func getPrismaticJointTorque(world *world, base *jointSim) Vec3 {
	transformA := getBodyTransform(world, int(base.bodyIdA))
	joint := &base.prismaticJoint

	torque := MulSV(world.inv_h, joint.angularImpulse)
	torque = RotateVector(base.localFrameA.Q, torque)
	torque = RotateVector(transformA.Q, torque)
	return torque
}

// preparePrismaticJoint is b3PreparePrismaticJoint.
func preparePrismaticJoint(base *jointSim, context *stepContext) {
	assert(base.typ == PrismaticJoint, "base->type == b3_prismaticJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.prismaticJoint
	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Q = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.frameA.P = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.frameB.Q = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	joint.frameB.P = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))

	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)
	joint.rotationMass = InvertMatrix(invInertiaSum)

	// Initial joint axes in world space
	matrixA := MakeMatrixFromQuat(joint.frameA.Q)
	joint.jointAxis = matrixA.Cx
	joint.perpAxisY = matrixA.Cy
	joint.perpAxisZ = matrixA.Cz

	joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, jointIntervalH(context))

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.perpImpulse = Vec2{0.0, 0.0}
		joint.angularImpulse = Vec3{0.0, 0.0, 0.0}
		joint.motorImpulse = 0.0
		joint.springImpulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
	}
}

// warmStartPrismaticJoint is b3WarmStartPrismaticJoint.
func warmStartPrismaticJoint(base *jointSim, context *stepContext) {
	assert(base.typ == PrismaticJoint, "base->type == b3_prismaticJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.prismaticJoint

	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	// todo make this code and the wheel joint more similar

	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)
	d := Add(Add(Sub(stateB.deltaPosition, stateA.deltaPosition), joint.deltaCenter), Sub(rB, rA))
	jointAxis := RotateVector(stateA.deltaRotation, joint.jointAxis)
	sAx := Cross(Add(rA, d), jointAxis)
	sBx := Cross(rB, jointAxis)

	perpY := RotateVector(stateA.deltaRotation, joint.perpAxisY)
	perpZ := RotateVector(stateA.deltaRotation, joint.perpAxisZ)
	sAy := Cross(Add(rA, d), perpY)
	sBy := Cross(rB, perpY)
	sAz := Cross(Add(rA, d), perpZ)
	sBz := Cross(rB, perpZ)

	axialImpulse := joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
	perpImpulse := joint.perpImpulse

	P := blend3(axialImpulse, jointAxis, perpImpulse.X, perpY, perpImpulse.Y, perpZ)
	LA := Add(blend3(axialImpulse, sAx, perpImpulse.X, sAy, perpImpulse.Y, sAz), joint.angularImpulse)
	LB := Add(blend3(axialImpulse, sBx, perpImpulse.X, sBy, perpImpulse.Y, sBz), joint.angularImpulse)

	vA := stateA.linearVelocity
	wA := stateA.angularVelocity
	vB := stateB.linearVelocity
	wB := stateB.angularVelocity
	vA = MulSub(vA, mA, P)
	wA = Sub(wA, MulMV(iA, LA))
	vB = MulAdd(vB, mB, P)
	wB = Add(wB, MulMV(iB, LB))

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

// solvePrismaticJoint is b3SolvePrismaticJoint.
func solvePrismaticJoint(base *jointSim, context *stepContext, useBias bool) {
	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.prismaticJoint
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
	rA := RotateVector(stateA.deltaRotation, joint.frameA.P)
	rB := RotateVector(stateB.deltaRotation, joint.frameB.P)

	dcA := stateA.deltaPosition
	dcB := stateB.deltaPosition
	d := Add(Add(Sub(dcB, dcA), joint.deltaCenter), Sub(rB, rA))

	jointAxis := RotateVector(stateA.deltaRotation, joint.jointAxis)
	sAx := Cross(Add(rA, d), jointAxis)
	sBx := Cross(rB, jointAxis)
	jointTranslation := Dot(d, jointAxis)
	targetTranslation := joint.targetTranslation

	// The axial effective mass must be fresh to avoid divergence when the joint is stressed
	ka := mA + mB + Dot(sAx, MulMV(iA, sAx)) + Dot(sBx, MulMV(iB, sBx))
	var axialMass float32 = 0.0
	if ka > 0.0 {
		axialMass = 1.0 / ka
	}

	// Solve spring
	if joint.enableSpring && fixedRotation == false {
		// Get the substep relative rotation
		c := jointTranslation - targetTranslation

		bias := float32(joint.springSoftness.biasRate * c)
		massScale := joint.springSoftness.massScale
		impulseScale := joint.springSoftness.impulseScale

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Dot(vRel, jointAxis)
		deltaImpulse := float32(-massScale*axialMass*(cdot+bias)) - float32(impulseScale*joint.springImpulse)
		joint.springImpulse += deltaImpulse

		P := MulSV(deltaImpulse, jointAxis)
		LA := MulSV(deltaImpulse, sAx)
		LB := MulSV(deltaImpulse, sBx)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, LA))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, LB))
	}

	if joint.enableMotor && fixedRotation == false {
		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Dot(vRel, jointAxis) - joint.motorSpeed

		deltaImpulse := float32(-axialMass * cdot)
		newImpulse := joint.motorImpulse + deltaImpulse
		maxImpulse := float32(joint.maxMotorForce * jointIntervalH(context))
		newImpulse = ClampFloat(newImpulse, -maxImpulse, maxImpulse)
		deltaImpulse = newImpulse - joint.motorImpulse
		joint.motorImpulse = newImpulse

		P := MulSV(deltaImpulse, jointAxis)
		LA := MulSV(deltaImpulse, sAx)
		LB := MulSV(deltaImpulse, sBx)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, LA))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, LB))
	}

	if joint.enableLimit && fixedRotation == false {
		speculativeDistance := float32(0.25 * (joint.upperTranslation - joint.lowerTranslation))

		// Lower limit
		{
			C := jointTranslation - joint.lowerTranslation

			if C < speculativeDistance {
				var bias float32 = 0.0
				var massScale float32 = 1.0
				var impulseScale float32 = 0.0
				if C > 0.0 {
					// speculation
					bias = float32(C * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * C)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
				cdot := Dot(vRel, jointAxis)
				oldImpulse := joint.lowerImpulse
				deltaImpulse := float32(-massScale*axialMass*(cdot+bias)) - float32(impulseScale*oldImpulse)
				joint.lowerImpulse = MaxFloat(oldImpulse+deltaImpulse, 0.0)
				deltaImpulse = joint.lowerImpulse - oldImpulse

				P := MulSV(deltaImpulse, jointAxis)
				LA := MulSV(deltaImpulse, sAx)
				LB := MulSV(deltaImpulse, sBx)

				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, LA))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, LB))
			} else {
				joint.lowerImpulse = 0.0
			}
		}

		// Upper limit
		{
			C := joint.upperTranslation - jointTranslation

			if C < speculativeDistance {
				var bias float32 = 0.0
				var massScale float32 = 1.0
				var impulseScale float32 = 0.0
				if C > 0.0 {
					// speculation
					bias = float32(C * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * C)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				// sign flipped on Cdot
				vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
				cdot := -Dot(vRel, jointAxis)
				oldImpulse := joint.upperImpulse
				deltaImpulse := float32(-massScale*axialMass*(cdot+bias)) - float32(impulseScale*oldImpulse)
				joint.upperImpulse = MaxFloat(oldImpulse+deltaImpulse, 0.0)

				// sign flipped on applied impulse
				negDeltaImpulse := oldImpulse - joint.upperImpulse
				P := MulSV(negDeltaImpulse, jointAxis)
				LA := MulSV(negDeltaImpulse, sAx)
				LB := MulSV(negDeltaImpulse, sBx)

				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, LA))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, LB))
			} else {
				joint.upperImpulse = 0.0
			}
		}
	}

	// Rotation constraint
	if fixedRotation == false {
		bias := Vec3{0.0, 0.0, 0.0}
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0

		if useBias {
			quatA := MulQuat(stateA.deltaRotation, joint.frameA.Q)
			quatB := MulQuat(stateB.deltaRotation, joint.frameB.Q)

			relQ := InvMulQuat(quatA, quatB)
			targetQuat := QuatIdentity
			deltaRotation := deltaQuatToRotation(relQ, targetQuat)
			c := Neg(RotateVector(quatA, deltaRotation))

			bias = MulSV(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		cdot := Sub(wB, wA)
		impulse := Sub(
			MulSV(-massScale, MulMV(joint.rotationMass, Add(cdot, bias))),
			MulSV(impulseScale, joint.angularImpulse))
		joint.angularImpulse = Add(joint.angularImpulse, impulse)

		wA = Sub(wA, MulMV(iA, impulse))
		wB = Add(wB, MulMV(iB, impulse))
	}

	// Solve point-to-line constraint
	{
		perpY := RotateVector(stateA.deltaRotation, joint.perpAxisY)
		perpZ := RotateVector(stateA.deltaRotation, joint.perpAxisZ)

		bias := Vec2{0.0, 0.0}
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias {
			c := Vec2{Dot(perpY, d), Dot(perpZ, d)}
			bias = mulSV2(base.constraintSoftness.biasRate, c)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		vRel := Sub(Sub(Add(vB, Cross(wB, rB)), vA), Cross(wA, Add(rA, d)))
		cdot := Vec2{Dot(perpY, vRel), Dot(perpZ, vRel)}

		// K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
		// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]
		sAy := Cross(Add(rA, d), perpY)
		sBy := Cross(rB, perpY)
		sAz := Cross(Add(rA, d), perpZ)
		sBz := Cross(rB, perpZ)

		kyy := mA + mB + Dot(sAy, MulMV(iA, sAy)) + Dot(sBy, MulMV(iB, sBy))
		kyz := Dot(sAy, MulMV(iA, sAz)) + Dot(sBy, MulMV(iB, sBz))
		kzz := mA + mB + Dot(sAz, MulMV(iA, sAz)) + Dot(sBz, MulMV(iB, sBz))

		K := matrix2{Vec2{kyy, kyz}, Vec2{kyz, kzz}}

		oldImpulse := joint.perpImpulse
		sol := solve2(K, add2(cdot, bias))
		deltaImpulse := sub2(mulSV2(-massScale, sol), mulSV2(impulseScale, oldImpulse))
		joint.perpImpulse = add2(oldImpulse, deltaImpulse)

		P := Blend2(deltaImpulse.X, perpY, deltaImpulse.Y, perpZ)

		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, Blend2(deltaImpulse.X, sAy, deltaImpulse.Y, sAz)))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, Blend2(deltaImpulse.X, sBy, deltaImpulse.Y, sBz)))
	}

	assert(IsValidVec3(vA), "b3IsValidVec3( vA )")
	assert(IsValidVec3(wA), "b3IsValidVec3( wA )")
	assert(IsValidVec3(vB), "b3IsValidVec3( vB )")
	assert(IsValidVec3(wB), "b3IsValidVec3( wB )")

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = vA
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = vB
		stateB.angularVelocity = wB
	}
}

// drawPrismaticJoint is b3DrawPrismaticJoint.
func drawPrismaticJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	frameA := MulWorldTransforms(transformA, base.localFrameA)
	frameB := MulWorldTransforms(transformB, base.localFrameB)

	R := MakeMatrixFromQuat(frameA.Q)
	axis := R.Cx
	perpY := R.Cy
	perpZ := R.Cz

	s := float32(0.2 * scale)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(s, perpY)), ColorGreen, draw.Context)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(s, perpZ)), ColorBlue, draw.Context)

	joint := &base.prismaticJoint
	if joint.enableLimit {
		p1 := OffsetPos(frameA.P, MulSV(joint.lowerTranslation, axis))
		p2 := OffsetPos(frameA.P, MulSV(joint.upperTranslation, axis))
		draw.DrawSegmentFcn(p1, p2, ColorOrange, draw.Context)
		draw.DrawPointFcn(p1, 10.0, ColorGreen, draw.Context)
		draw.DrawPointFcn(p2, 10.0, ColorRed, draw.Context)
	} else {
		p1 := OffsetPos(frameA.P, MulSV(float32(-0.5*scale), axis))
		p2 := OffsetPos(frameA.P, MulSV(float32(0.5*scale), axis))
		draw.DrawSegmentFcn(p1, p2, ColorOrange, draw.Context)
	}

	draw.DrawPointFcn(frameB.P, 8.0, ColorViolet, draw.Context)
}
