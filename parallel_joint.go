// Port of src/parallel_joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// PORT: the b3ParallelJoint_* functions of box3d.h are methods on JointID named
// ParallelJoint<Method>.

// ParallelJointSetSpringHertz is b3ParallelJoint_SetSpringHertz.
// Set the spring stiffness in Hertz
func (jointId JointID) ParallelJointSetSpringHertz(hertz float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsParallelJointSetSpringHertz{jointId, hertz}
		recWriteParallelJointSetSpringHertz(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, ParallelJoint)
	base.parallelJoint.hertz = hertz
}

// ParallelJointGetSpringHertz is b3ParallelJoint_GetSpringHertz.
// Get the spring Hertz
func (jointId JointID) ParallelJointGetSpringHertz() float32 {
	base := getJointSimCheckType(jointId, ParallelJoint)
	return base.parallelJoint.hertz
}

// ParallelJointSetSpringDampingRatio is b3ParallelJoint_SetSpringDampingRatio.
// Set the spring damping ratio, non-dimensional
func (jointId JointID) ParallelJointSetSpringDampingRatio(dampingRatio float32) {
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsParallelJointSetSpringDampingRatio{jointId, dampingRatio}
		recWriteParallelJointSetSpringDampingRatio(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, ParallelJoint)
	base.parallelJoint.dampingRatio = dampingRatio
}

// ParallelJointGetSpringDampingRatio is b3ParallelJoint_GetSpringDampingRatio.
// Get the spring damping ratio
func (jointId JointID) ParallelJointGetSpringDampingRatio() float32 {
	base := getJointSimCheckType(jointId, ParallelJoint)
	return base.parallelJoint.dampingRatio
}

// ParallelJointSetMaxTorque is b3ParallelJoint_SetMaxTorque.
// Set the maximum spring torque, usually in newton-meters
func (jointId JointID) ParallelJointSetMaxTorque(maxForce float32) {
	assert(IsValidFloat(maxForce) && maxForce >= 0.0, "b3IsValidFloat( maxForce ) && maxForce >= 0.0f")
	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsParallelJointSetMaxTorque{jointId, maxForce}
		recWriteParallelJointSetMaxTorque(world.recording, &recArgs)
	}
	base := getJointSimCheckType(jointId, ParallelJoint)
	base.parallelJoint.maxTorque = maxForce
}

// ParallelJointGetMaxTorque is b3ParallelJoint_GetMaxTorque.
// Get the maximum spring torque, usually in newton-meters
func (jointId JointID) ParallelJointGetMaxTorque() float32 {
	base := getJointSimCheckType(jointId, ParallelJoint)
	return base.parallelJoint.maxTorque
}

// getParallelJointTorque is b3GetParallelJointTorque.
func getParallelJointTorque(world *world, base *jointSim) Vec3 {
	joint := &base.parallelJoint

	quatB := joint.quatB
	if consistentParallelWarmStart[world.worldId] && DotQuat(joint.quatA, quatB) < 0 {
		quatB = NegateQuat(quatB)
	}
	relQ := InvMulQuat(joint.quatA, quatB)
	joint.perpAxisX = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
	joint.perpAxisY = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))

	angularImpulse := Blend2(joint.perpImpulse.X, joint.perpAxisX, joint.perpImpulse.Y, joint.perpAxisY)
	torque := MulSV(world.inv_h, angularImpulse)
	return torque
}

// prepareParallelJoint is b3PrepareParallelJoint.
func prepareParallelJoint(base *jointSim, context *stepContext) {
	assert(base.typ == ParallelJoint, "base->type == b3_parallelJoint")

	world := context.world

	bodyA := arrayGet(world.bodies, int(base.bodyIdA))
	bodyB := arrayGet(world.bodies, int(base.bodyIdB))

	assert(bodyA.setIndex == awakeSet || bodyB.setIndex == awakeSet,
		"bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet")
	setA := arrayGet(world.solverSets, int(bodyA.setIndex))
	setB := arrayGet(world.solverSets, int(bodyB.setIndex))

	localIndexA := bodyA.localIndex
	localIndexB := bodyB.localIndex

	bodySimA := arrayGet(setA.bodySims, int(localIndexA))
	bodySimB := arrayGet(setB.bodySims, int(localIndexB))

	base.invMassA = bodySimA.invMass
	base.invMassB = bodySimB.invMass
	base.invIA = bodySimA.invInertiaWorld
	base.invIB = bodySimB.invInertiaWorld

	invInertiaSum := AddMM(base.invIA, base.invIB)
	base.fixedRotation = Det(invInertiaSum) < 1000.0*fltMin

	joint := &base.parallelJoint
	joint.indexA = NullIndex
	if bodyA.setIndex == awakeSet {
		joint.indexA = localIndexA
	}
	joint.indexB = NullIndex
	if bodyB.setIndex == awakeSet {
		joint.indexB = localIndexB
	}

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.quatA = MulQuat(bodySimA.transform.Q, base.localFrameA.Q)
	joint.quatB = MulQuat(bodySimB.transform.Q, base.localFrameB.Q)
	// Golem fix: use the solver's hemisphere for warm-start Jacobians as well.
	// Otherwise free twist past pi reverses the retained impulse's direction.
	if consistentParallelWarmStart[context.world.worldId] && DotQuat(joint.quatA, joint.quatB) < 0 {
		joint.quatB = NegateQuat(joint.quatB)
	}

	relQ := InvMulQuat(joint.quatA, joint.quatB)

	{
		// These are needed for warm starting
		joint.perpAxisX = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
		joint.perpAxisY = MulSV(0.5, RotateVector(joint.quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))
	}

	joint.softness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

	if context.enableWarmStarting == false {
		joint.perpImpulse = Vec2{0.0, 0.0}
	}
}

// warmStartParallelJoint is b3WarmStartParallelJoint.
func warmStartParallelJoint(base *jointSim, context *stepContext) {
	assert(base.typ == ParallelJoint, "base->type == b3_parallelJoint")

	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.parallelJoint

	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	wA := stateA.angularVelocity
	wB := stateB.angularVelocity

	angularImpulse := Blend2(joint.perpImpulse.X, joint.perpAxisX, joint.perpImpulse.Y, joint.perpAxisY)

	wA = Sub(wA, MulMV(iA, angularImpulse))
	wB = Add(wB, MulMV(iB, angularImpulse))

	if stateA.flags&dynamicFlag != 0 {
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.angularVelocity = wB
	}
}

// solveParallelJoint is b3SolveParallelJoint.
func solveParallelJoint(base *jointSim, context *stepContext) {
	iA := base.invIA
	iB := base.invIB

	// dummy state for static bodies
	dummyState := identityBodyState

	joint := &base.parallelJoint
	stateA := &dummyState
	if joint.indexA != NullIndex {
		stateA = &context.states[joint.indexA]
	}
	stateB := &dummyState
	if joint.indexB != NullIndex {
		stateB = &context.states[joint.indexB]
	}

	wA := stateA.angularVelocity
	wB := stateB.angularVelocity

	fixedRotation := base.fixedRotation
	quatA := MulQuat(stateA.deltaRotation, joint.quatA)
	quatB := MulQuat(stateB.deltaRotation, joint.quatB)

	if DotQuat(quatA, quatB) < 0.0 {
		// this keeps the rotation angle in the range [-pi, pi]
		quatB = NegateQuat(quatB)
	}

	relQ := InvMulQuat(quatA, quatB)

	if fixedRotation == false && joint.maxTorque > 0.0 {
		c := Vec2{relQ.V.X, relQ.V.Y}
		bias := Vec2{float32(joint.softness.biasRate * c.X), float32(joint.softness.biasRate * c.Y)}
		massScale := joint.softness.massScale
		impulseScale := joint.softness.impulseScale

		// Collinearity constraint as 2-by-2
		perpAxisX := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisX), Cross(relQ.V, Vec3AxisX))))
		perpAxisY := MulSV(0.5, RotateVector(quatA, Add(MulSV(relQ.S, Vec3AxisY), Cross(relQ.V, Vec3AxisY))))
		joint.perpAxisX = perpAxisX
		joint.perpAxisY = perpAxisY

		invInertiaSum := AddMM(iA, iB)
		kxx := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisX))
		kyy := Dot(perpAxisY, MulMV(invInertiaSum, perpAxisY))
		kxy := Dot(perpAxisX, MulMV(invInertiaSum, perpAxisY))

		k := matrix2{Vec2{kxx, kxy}, Vec2{kxy, kyy}}

		wRel := Sub(wB, wA)
		cdot := Vec2{Dot(wRel, perpAxisX), Dot(wRel, perpAxisY)}

		maxImpulse := float32(context.h * joint.maxTorque)
		oldImpulse := joint.perpImpulse
		cdotPlusBias := Vec2{cdot.X + bias.X, cdot.Y + bias.Y}
		sol := solve2(k, cdotPlusBias)
		deltaImpulse := Vec2{
			float32(-massScale*sol.X) - float32(impulseScale*oldImpulse.X),
			float32(-massScale*sol.Y) - float32(impulseScale*oldImpulse.Y),
		}
		joint.perpImpulse = Vec2{oldImpulse.X + deltaImpulse.X, oldImpulse.Y + deltaImpulse.Y}
		if lengthSquared2(joint.perpImpulse) > float32(maxImpulse*maxImpulse) {
			s := maxImpulse / length2(joint.perpImpulse)
			joint.perpImpulse = Vec2{float32(s * joint.perpImpulse.X), float32(s * joint.perpImpulse.Y)}
		}

		deltaImpulse = sub2(joint.perpImpulse, oldImpulse)

		angularImpulse := Blend2(deltaImpulse.X, perpAxisX, deltaImpulse.Y, perpAxisY)
		wA = Sub(wA, MulMV(iA, angularImpulse))
		wB = Add(wB, MulMV(iB, angularImpulse))
	}

	if stateA.flags&dynamicFlag != 0 {
		stateA.angularVelocity = wA
	}

	if stateB.flags&dynamicFlag != 0 {
		stateB.angularVelocity = wB
	}
}

// drawParallelJoint is b3DrawParallelJoint.
func drawParallelJoint(draw *DebugDraw, base *jointSim, transformA, transformB WorldTransform, scale float32) {
	length := float32(0.1 * scale)

	frameA := MulWorldTransforms(transformA, base.localFrameA)
	draw.DrawSegmentFcn(frameA.P, OffsetPos(frameA.P, MulSV(length, RotateVector(frameA.Q, Vec3AxisZ))), ColorGreen, draw.Context)

	frameB := MulWorldTransforms(transformB, base.localFrameB)
	draw.DrawSegmentFcn(frameB.P, OffsetPos(frameB.P, MulSV(length, RotateVector(frameB.Q, Vec3AxisZ))), ColorBlue, draw.Context)
}
