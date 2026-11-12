// Port of src/distance_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3DistanceJoint_* functions of box3d.h are methods on JointID named
// DistanceJoint<Method>. B3_REC is expanded at the call site (see recording.go).

// DistanceJointSetLength is b3DistanceJoint_SetLength.
// Set the rest length of a distance joint
func (jointId JointID) DistanceJointSetLength(length float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetLength{jointId, length}
		recWriteDistanceJointSetLength(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint

	joint.length = ClampFloat(length, LinearSlop(), Huge())
	joint.impulse = 0.0
	joint.lowerImpulse = 0.0
	joint.upperImpulse = 0.0
}

// DistanceJointGetLength is b3DistanceJoint_GetLength.
// Get the rest length of a distance joint
func (jointId JointID) DistanceJointGetLength() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	return joint.length
}

// DistanceJointEnableLimit is b3DistanceJoint_EnableLimit.
// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
// and the limit has no effect.
func (jointId JointID) DistanceJointEnableLimit(enableLimit bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointEnableLimit{jointId, enableLimit}
		recWriteDistanceJointEnableLimit(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	joint.enableLimit = enableLimit
}

// DistanceJointIsLimitEnabled is b3DistanceJoint_IsLimitEnabled.
// Is the distance joint limit enabled?
func (jointId JointID) DistanceJointIsLimitEnabled() bool {
	joint := getJointSimCheckType(jointId, DistanceJoint)
	return joint.distanceJoint.enableLimit
}

// DistanceJointSetLengthRange is b3DistanceJoint_SetLengthRange.
// Set the minimum and maximum length parameters of a distance joint
func (jointId JointID) DistanceJointSetLengthRange(minLength, maxLength float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetLengthRange{jointId, minLength, maxLength}
		recWriteDistanceJointSetLengthRange(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint

	minLength = ClampFloat(minLength, LinearSlop(), Huge())
	maxLength = ClampFloat(maxLength, LinearSlop(), Huge())
	joint.minLength = MinFloat(minLength, maxLength)
	joint.maxLength = MaxFloat(minLength, maxLength)
	joint.impulse = 0.0
	joint.lowerImpulse = 0.0
	joint.upperImpulse = 0.0
}

// DistanceJointGetMinLength is b3DistanceJoint_GetMinLength.
// Get the distance joint minimum length
func (jointId JointID) DistanceJointGetMinLength() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	return joint.minLength
}

// DistanceJointGetMaxLength is b3DistanceJoint_GetMaxLength.
// Get the distance joint maximum length
func (jointId JointID) DistanceJointGetMaxLength() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	return joint.maxLength
}

// DistanceJointGetCurrentLength is b3DistanceJoint_GetCurrentLength.
// Get the current length of a distance joint
func (jointId JointID) DistanceJointGetCurrentLength() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)

	world := getUnlockedWorld(int(jointId.World0))
	if world == nil {
		return 0.0
	}

	transformA := getBodyTransform(world, int(base.bodyIdA))
	transformB := getBodyTransform(world, int(base.bodyIdB))

	pA := TransformWorldPoint(transformA, base.localFrameA.P)
	pB := TransformWorldPoint(transformB, base.localFrameB.P)
	d := SubPos(pB, pA)
	length := Length(d)
	return length
}

// DistanceJointEnableSpring is b3DistanceJoint_EnableSpring.
// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
func (jointId JointID) DistanceJointEnableSpring(enableSpring bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointEnableSpring{jointId, enableSpring}
		recWriteDistanceJointEnableSpring(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	base.distanceJoint.enableSpring = enableSpring
}

// DistanceJointIsSpringEnabled is b3DistanceJoint_IsSpringEnabled.
// Is the distance joint spring enabled?
func (jointId JointID) DistanceJointIsSpringEnabled() bool {
	base := getJointSimCheckType(jointId, DistanceJoint)
	return base.distanceJoint.enableSpring
}

// DistanceJointSetSpringForceRange is b3DistanceJoint_SetSpringForceRange.
// Set the force range for the spring.
func (jointId JointID) DistanceJointSetSpringForceRange(lowerForce, upperForce float32) {
	assert(lowerForce <= upperForce, "lowerForce <= upperForce")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetSpringForceRange{jointId, lowerForce, upperForce}
		recWriteDistanceJointSetSpringForceRange(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	base.distanceJoint.lowerSpringForce = lowerForce
	base.distanceJoint.upperSpringForce = upperForce
}

// DistanceJointGetSpringForceRange is b3DistanceJoint_GetSpringForceRange.
// Get the force range for the spring.
// PORT: the lowerForce and upperForce out-parameters are the return values.
func (jointId JointID) DistanceJointGetSpringForceRange() (lowerForce, upperForce float32) {
	base := getJointSimCheckType(jointId, DistanceJoint)
	lowerForce = base.distanceJoint.lowerSpringForce
	upperForce = base.distanceJoint.upperSpringForce
	return lowerForce, upperForce
}

// DistanceJointSetSpringHertz is b3DistanceJoint_SetSpringHertz.
// Set the spring stiffness in Hertz
func (jointId JointID) DistanceJointSetSpringHertz(hertz float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetSpringHertz{jointId, hertz}
		recWriteDistanceJointSetSpringHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	base.distanceJoint.hertz = hertz
}

// DistanceJointSetSpringDampingRatio is b3DistanceJoint_SetSpringDampingRatio.
// Set the spring damping ratio, non-dimensional
func (jointId JointID) DistanceJointSetSpringDampingRatio(dampingRatio float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetSpringDampingRatio{jointId, dampingRatio}
		recWriteDistanceJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, DistanceJoint)
	base.distanceJoint.dampingRatio = dampingRatio
}

// DistanceJointGetSpringHertz is b3DistanceJoint_GetSpringHertz.
// Get the spring Hertz
func (jointId JointID) DistanceJointGetSpringHertz() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	return joint.hertz
}

// DistanceJointGetSpringDampingRatio is b3DistanceJoint_GetSpringDampingRatio.
// Get the spring damping ratio
func (jointId JointID) DistanceJointGetSpringDampingRatio() float32 {
	base := getJointSimCheckType(jointId, DistanceJoint)
	joint := &base.distanceJoint
	return joint.dampingRatio
}

// DistanceJointEnableMotor is b3DistanceJoint_EnableMotor.
// Enable/disable the distance joint motor
func (jointId JointID) DistanceJointEnableMotor(enableMotor bool) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointEnableMotor{jointId, enableMotor}
		recWriteDistanceJointEnableMotor(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, DistanceJoint)
	if enableMotor != joint.distanceJoint.enableMotor {
		joint.distanceJoint.enableMotor = enableMotor
		joint.distanceJoint.motorImpulse = 0.0
	}
}

// DistanceJointIsMotorEnabled is b3DistanceJoint_IsMotorEnabled.
// Is the distance joint motor enabled?
func (jointId JointID) DistanceJointIsMotorEnabled() bool {
	joint := getJointSimCheckType(jointId, DistanceJoint)
	return joint.distanceJoint.enableMotor
}

// DistanceJointSetMotorSpeed is b3DistanceJoint_SetMotorSpeed.
// Set the distance joint motor speed, usually in meters per second
func (jointId JointID) DistanceJointSetMotorSpeed(motorSpeed float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetMotorSpeed{jointId, motorSpeed}
		recWriteDistanceJointSetMotorSpeed(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, DistanceJoint)
	joint.distanceJoint.motorSpeed = motorSpeed
}

// DistanceJointGetMotorSpeed is b3DistanceJoint_GetMotorSpeed.
// Get the distance joint motor speed, usually in meters per second
func (jointId JointID) DistanceJointGetMotorSpeed() float32 {
	joint := getJointSimCheckType(jointId, DistanceJoint)
	return joint.distanceJoint.motorSpeed
}

// DistanceJointGetMotorForce is b3DistanceJoint_GetMotorForce.
// Get the distance joint current motor force, usually in newtons
func (jointId JointID) DistanceJointGetMotorForce() float32 {
	world := getWorld(int(jointId.World0))
	base := getJointSimCheckType(jointId, DistanceJoint)
	return float32(world.inv_h * base.distanceJoint.motorImpulse)
}

// DistanceJointSetMaxMotorForce is b3DistanceJoint_SetMaxMotorForce.
// Set the distance joint maximum motor force, usually in newtons
func (jointId JointID) DistanceJointSetMaxMotorForce(force float32) {
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsDistanceJointSetMaxMotorForce{jointId, force}
		recWriteDistanceJointSetMaxMotorForce(world.recording, &recArgs)
	}
	joint := getJointSimCheckType(jointId, DistanceJoint)
	joint.distanceJoint.maxMotorForce = force
}

// DistanceJointGetMaxMotorForce is b3DistanceJoint_GetMaxMotorForce.
// Get the distance joint maximum motor force, usually in newtons
func (jointId JointID) DistanceJointGetMaxMotorForce() float32 {
	joint := getJointSimCheckType(jointId, DistanceJoint)
	return joint.distanceJoint.maxMotorForce
}

// getDistanceJointForce is b3GetDistanceJointForce.
func getDistanceJointForce(world *world, base *jointSim) Vec3 {
	joint := &base.distanceJoint

	transformA := getBodyTransform(world, int(base.bodyIdA))
	transformB := getBodyTransform(world, int(base.bodyIdB))

	pA := TransformWorldPoint(transformA, base.localFrameA.P)
	pB := TransformWorldPoint(transformB, base.localFrameB.P)
	d := SubPos(pB, pA)
	axis := Normalize(d)
	force := float32((joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse) * world.inv_h)
	return MulSV(force, axis)
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

// prepareDistanceJoint is b3PrepareDistanceJoint.
func prepareDistanceJoint(base *jointSim, context *stepContext) {
	assert(base.typ == DistanceJoint, "base->type == b3_distanceJoint")

	bodySimA, localIndexA := jointBodyForContext(context, base.bodyIdA)
	bodySimB, localIndexB := jointBodyForContext(context, base.bodyIdB)
	assert(localIndexA != NullIndex || localIndexB != NullIndex, "joint has a solver body")

	mA := bodySimA.invMass
	iA := bodySimA.invInertiaWorld
	mB := bodySimB.invMass
	iB := bodySimB.invInertiaWorld

	base.invMassA = mA
	base.invMassB = mB
	base.invIA = iA
	base.invIB = iB

	joint := &base.distanceJoint

	joint.indexA = localIndexA
	joint.indexB = localIndexB

	// initial anchors in world space
	joint.anchorA = RotateVector(bodySimA.transform.Q, Sub(base.localFrameA.P, bodySimA.localCenter))
	joint.anchorB = RotateVector(bodySimB.transform.Q, Sub(base.localFrameB.P, bodySimB.localCenter))
	joint.deltaCenter = SubPos(bodySimB.center, bodySimA.center)

	rA := joint.anchorA
	rB := joint.anchorB
	separation := Add(Sub(rB, rA), joint.deltaCenter)
	axis := Normalize(separation)

	// compute effective mass
	crA := Cross(rA, axis)
	crB := Cross(rB, axis)
	k := mA + mB + Dot(crA, MulMV(iA, crA)) + Dot(crB, MulMV(iB, crB))
	joint.axialMass = 0.0
	if k > 0.0 {
		joint.axialMass = 1.0 / k
	}

	joint.distanceSoftness = makeSoft(joint.hertz, joint.dampingRatio, jointIntervalH(context))

	if context.enableWarmStarting == false && !context.retainJointImpulses {
		joint.impulse = 0.0
		joint.lowerImpulse = 0.0
		joint.upperImpulse = 0.0
		joint.motorImpulse = 0.0
	}
}

// warmStartDistanceJoint is b3WarmStartDistanceJoint.
func warmStartDistanceJoint(base *jointSim, context *stepContext) {
	assert(base.typ == DistanceJoint, "base->type == b3_distanceJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.distanceJoint
	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	ds := Add(Sub(stateB.deltaPosition, stateA.deltaPosition), Sub(rB, rA))
	separation := Add(joint.deltaCenter, ds)
	axis := Normalize(separation)

	axialImpulse := joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse
	P := MulSV(axialImpulse, axis)

	if stateA.flags&dynamicFlag != 0 {
		stateA.linearVelocity = MulSub(stateA.linearVelocity, mA, P)
		stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(iA, Cross(rA, P)))
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.linearVelocity = MulAdd(stateB.linearVelocity, mB, P)
		stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(iB, Cross(rB, P)))
	}
}

// solveDistanceJoint is b3SolveDistanceJoint.
func solveDistanceJoint(base *jointSim, context *stepContext, useBias bool) {
	assert(base.typ == DistanceJoint, "base->type == b3_distanceJoint")

	mA := base.invMassA
	mB := base.invMassB
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.distanceJoint
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

	// current anchors
	rA := RotateVector(stateA.deltaRotation, joint.anchorA)
	rB := RotateVector(stateB.deltaRotation, joint.anchorB)

	// current separation
	ds := Add(Sub(stateB.deltaPosition, stateA.deltaPosition), Sub(rB, rA))
	separation := Add(joint.deltaCenter, ds)

	length := Length(separation)
	axis := Normalize(separation)

	// joint is soft if
	// - spring is enabled
	// - and (joint limit is disabled or limits are not equal)
	if joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false) {
		// spring
		if joint.hertz > 0.0 {
			// Cdot = dot(u, v + cross(w, r))
			vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
			Cdot := Dot(axis, vr)
			C := length - joint.length
			bias := float32(joint.distanceSoftness.biasRate * C)

			m := float32(joint.distanceSoftness.massScale * joint.axialMass)
			oldImpulse := joint.impulse
			impulse := float32(-m*(Cdot+bias)) - float32(joint.distanceSoftness.impulseScale*oldImpulse)
			h := jointIntervalH(context)
			joint.impulse = ClampFloat(joint.impulse+impulse, float32(joint.lowerSpringForce*h), float32(joint.upperSpringForce*h))
			impulse = joint.impulse - oldImpulse

			P := MulSV(impulse, axis)
			vA = MulSub(vA, mA, P)
			wA = Sub(wA, MulMV(iA, Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = Add(wB, MulMV(iB, Cross(rB, P)))
		}

		if joint.enableLimit {
			// lower limit
			{
				vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
				Cdot := Dot(axis, vr)

				C := length - joint.minLength

				var bias float32 = 0.0
				var massCoeff float32 = 1.0
				var impulseCoeff float32 = 0.0
				if C > 0.0 {
					// speculative
					bias = float32(C * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * C)
					massCoeff = base.constraintSoftness.massScale
					impulseCoeff = base.constraintSoftness.impulseScale
				}

				impulse := float32(-massCoeff*joint.axialMass*(Cdot+bias)) - float32(impulseCoeff*joint.lowerImpulse)
				newImpulse := MaxFloat(0.0, joint.lowerImpulse+impulse)
				impulse = newImpulse - joint.lowerImpulse
				joint.lowerImpulse = newImpulse

				P := MulSV(impulse, axis)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}

			// upper
			{
				vr := Add(Sub(vA, vB), Sub(Cross(wA, rA), Cross(wB, rB)))
				Cdot := Dot(axis, vr)

				C := joint.maxLength - length

				var bias float32 = 0.0
				var massScale float32 = 1.0
				var impulseScale float32 = 0.0
				if C > 0.0 {
					// speculative
					bias = float32(C * context.inv_h)
				} else if useBias {
					bias = float32(base.constraintSoftness.biasRate * C)
					massScale = base.constraintSoftness.massScale
					impulseScale = base.constraintSoftness.impulseScale
				}

				impulse := float32(-massScale*joint.axialMass*(Cdot+bias)) - float32(impulseScale*joint.upperImpulse)
				newImpulse := MaxFloat(0.0, joint.upperImpulse+impulse)
				impulse = newImpulse - joint.upperImpulse
				joint.upperImpulse = newImpulse

				P := MulSV(-impulse, axis)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
		}

		if joint.enableMotor {
			vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
			Cdot := Dot(axis, vr)
			impulse := float32(joint.axialMass * (joint.motorSpeed - Cdot))
			oldImpulse := joint.motorImpulse
			maxImpulse := float32(jointIntervalH(context) * joint.maxMotorForce)
			joint.motorImpulse = ClampFloat(joint.motorImpulse+impulse, -maxImpulse, maxImpulse)
			impulse = joint.motorImpulse - oldImpulse

			P := MulSV(impulse, axis)
			vA = MulSub(vA, mA, P)
			wA = Sub(wA, MulMV(iA, Cross(rA, P)))
			vB = MulAdd(vB, mB, P)
			wB = Add(wB, MulMV(iB, Cross(rB, P)))
		}
	} else {
		// rigid constraint
		vr := Add(Sub(vB, vA), Sub(Cross(wB, rB), Cross(wA, rA)))
		Cdot := Dot(axis, vr)

		C := length - joint.length

		var bias float32 = 0.0
		var massScale float32 = 1.0
		var impulseScale float32 = 0.0
		if useBias {
			bias = float32(base.constraintSoftness.biasRate * C)
			massScale = base.constraintSoftness.massScale
			impulseScale = base.constraintSoftness.impulseScale
		}

		impulse := float32(-massScale*joint.axialMass*(Cdot+bias)) - float32(impulseScale*joint.impulse)
		joint.impulse += impulse

		P := MulSV(impulse, axis)
		vA = MulSub(vA, mA, P)
		wA = Sub(wA, MulMV(iA, Cross(rA, P)))
		vB = MulAdd(vB, mB, P)
		wB = Add(wB, MulMV(iB, Cross(rB, P)))
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

// drawDistanceJoint is b3DrawDistanceJoint.
func drawDistanceJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform) {
	assert(base.typ == DistanceJoint, "base->type == b3_distanceJoint")

	joint := &base.distanceJoint

	pA := TransformWorldPoint(transformA, base.localFrameA.P)
	pB := TransformWorldPoint(transformB, base.localFrameB.P)

	axis := Normalize(SubPos(pB, pA))

	if joint.minLength < joint.maxLength && joint.enableLimit {
		pMin := OffsetPos(pA, MulSV(joint.minLength, axis))
		pMax := OffsetPos(pA, MulSV(joint.maxLength, axis))

		if joint.minLength > LinearSlop() {
			draw.DrawPointFcn(pMin, 6.0, ColorLightGreen, draw.Context)
		}

		if joint.maxLength < Huge() {
			draw.DrawPointFcn(pMax, 6.0, ColorRed, draw.Context)
		}

		if joint.minLength > LinearSlop() && joint.maxLength < Huge() {
			draw.DrawSegmentFcn(pMin, pMax, ColorGray, draw.Context)
		}
	}

	draw.DrawSegmentFcn(pA, pB, ColorWhite, draw.Context)
	draw.DrawPointFcn(pA, 4.0, ColorWhite, draw.Context)
	draw.DrawPointFcn(pB, 4.0, ColorWhite, draw.Context)

	if joint.hertz > 0.0 && joint.enableSpring {
		pRest := OffsetPos(pA, MulSV(joint.length, axis))
		draw.DrawPointFcn(pRest, 4.0, ColorBlue, draw.Context)
	}
}
