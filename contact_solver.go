// Port of src/contact_solver.h and src/contact_solver.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// ---- contact_solver.h ----

// manifoldConstraintPoint is b3ManifoldConstraintPoint.
type manifoldConstraintPoint struct {
	rA, rB             Vec3
	baseSeparation     float32
	relativeVelocity   float32
	normalImpulse      float32
	totalNormalImpulse float32
	restitutionImpulse float32
	normalMass         float32
	leverArm           float32
}

// manifoldConstraint is b3ManifoldConstraint.
type manifoldConstraint struct {
	points     [MaxManifoldPoints]manifoldConstraintPoint
	pointCount int32
	normal     Vec3
	tangent1   Vec3
	tangent2   Vec3
	// Friction centers
	centerA, centerB Vec3
	twistMass        float32
	twistImpulse     float32
	tangentMass      matrix2
	frictionImpulse  Vec2
	rollingImpulse   Vec3
	tangentVelocity1 float32
	tangentVelocity2 float32
}

// contactConstraint is b3ContactConstraint.
// PORT: b3ManifoldConstraint* constraints is a slice into the color's manifold constraints.
type contactConstraint struct {
	constraints        []manifoldConstraint
	contact            *contact
	indexA             int32
	indexB             int32
	invMassA, invMassB float32
	invIA, invIB       Matrix3
	softness           softness
	previousTOI        bool
	rollingMass        Matrix3
	friction           float32
	restitution        float32
	rollingResistance  float32
	manifoldCount      int32
}

// PORT: the b3*_ConvexW4 / b3*_MeshWideW4 declarations and the W4 byte counts are the kernels of
// contact_solver_wide.go (W suffix: solveContactsConvexW, …). The W8 declarations are the
// wideKernels table (wide_kernels.go), filled by the width 8 backend.

// wideBuffer is the typed form of upstream's `void*` wide constraint buffers
// (b3StepContext.wideConstraints / wideMeshConstraints, b3GraphColor.wideConstraints /
// wideMeshConstraints), generic over the lane width's constraint and manifold types
// (wideConstraintBuffer at width 4, wideConstraintBuffer8 at width 8).
//
// PORT: upstream stores a heterogeneous byte stream: each b3ContactConstraintWide is followed by
// its b3ContactManifoldWide array (one manifold for convex contacts, manifoldCount for mesh
// contacts), and b3GetConstraintW / b3NextConstraintW / b3GetManifoldsW walk it with byte
// offsets. Go keeps the two element types in two parallel slices instead, so the buffers are typed
// and need no unsafe:
//
//   - constraint index i is constraints[i];
//   - its manifolds start at manifolds[i*fixedManifoldCount] for convex constraints
//     (fixedManifoldCount == 1) and at manifolds[manifoldStarts[i]-manifoldStarts[0]] for mesh
//     constraints, exactly the offsets b3GetConstraintW computes;
//   - a byte pointer into the stream (base + constraintIndex*sizeof(constraint) +
//     manifoldIndex*sizeof(manifold), as solver.c slices the buffer per color) is the pair of
//     sub-slices {constraints[constraintIndex:], manifolds[manifoldIndex:]};
//   - NULL is the zero value.
//
// solver.c allocates the two slices with stackAlloc in place of one byte allocation sized with
// b3GetWide*ByteCount (allocWideBuffer, sliceWideBuffer and freeWideBuffer). The kernels and their
// helpers (b3GetConstraintW and friends) carry the constraint and manifold positions as indices
// into these slices.
type wideBuffer[C, M any] struct {
	constraints []C
	manifolds   []M
}

// allocWideBuffer allocates a wide buffer of constraintCount constraints and manifoldCount
// manifolds on the stack allocator, constraints first.
// PORT: the one b3AllocateArenaItem of solver.c, split into the two typed slices of wideBuffer.
func allocWideBuffer[C, M any](stack *stack, constraintCount, manifoldCount int, constraintName, manifoldName string) wideBuffer[C, M] {
	var buffer wideBuffer[C, M]
	buffer.constraints = stackAlloc[C](stack, constraintCount, constraintName)
	buffer.manifolds = stackAlloc[M](stack, manifoldCount, manifoldName)
	return buffer
}

// sliceWideBuffer is the part of buffer starting at constraint constraintBase and manifold
// manifoldBase, holding constraintCount constraints and the manifolds up to manifoldEnd.
// PORT: the byte offset solver.c adds to the buffer base when it slices the buffer per color.
func sliceWideBuffer[C, M any](buffer wideBuffer[C, M], constraintBase, constraintCount, manifoldBase, manifoldEnd int) wideBuffer[C, M] {
	return wideBuffer[C, M]{
		constraints: buffer.constraints[constraintBase : constraintBase+constraintCount],
		manifolds:   buffer.manifolds[manifoldBase:manifoldEnd],
	}
}

// freeWideBuffer frees a buffer of allocWideBuffer, in the reverse order of the allocation.
func freeWideBuffer[C, M any](stack *stack, buffer wideBuffer[C, M]) {
	stackFree(stack, buffer.manifolds)
	stackFree(stack, buffer.constraints)
}

// ---- contact_solver.c ----

// todo clean this file up because it is only used for overflow now

// contact separation for sub-stepping
// s = s0 + dot(cB + rB - cA - rA, normal)
// normal is held constant
// body positions c can translation and anchors r can rotate
// s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
// s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
// s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
// s_base = s0 + dot(cB0 - cA0, normal)

// prepareContactsMesh is b3PrepareContacts_Mesh.
// Prepare overflow constraints
func prepareContactsMesh(block solverBlock, context *stepContext) {
	world := context.world
	bodySims := context.sims
	states := context.states

	warmStartScale := float32(0.0)
	if world.enableWarmStarting {
		warmStartScale = 1.0
	}
	anyRestitution := false

	// Used for friction center weighting.
	invTau := 1.0 / SpeculativeDistance()

	assert(block.blockType == overflowBlockType, "block.blockType == b3_overflowBlock")
	overflow := &world.constraintGraph.colors[overflowIndex]
	spans := context.overflowSpans
	manifoldBase := overflow.manifoldConstraints
	base := overflow.contactConstraints

	index := int(block.startIndex)
	endIndex := int(block.startIndex) + int(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex := 0
	for int(spans[colorIndex+1].start) <= index {
		colorIndex += 1
	}

	// Loop over block
	for index < endIndex {
		colorStart := int(spans[colorIndex].start)
		colorEndIndex := MinInt(int(spans[colorIndex+1].start), endIndex)
		specs := spans[colorIndex].contacts

		// Loop over color
		for ; index < colorEndIndex; index++ {
			contactConstraint := &base[index]

			localIndex := index - colorStart
			assert(0 <= localIndex && localIndex < int(spans[colorIndex].count), "0 <= localIndex && localIndex < spans[colorIndex].count")
			contactId := specs[localIndex].contactId
			contact := arrayGet(world.contacts, int(contactId))
			assert(contact.contactId == contactId, "contact->contactId == contactId")

			indexA := decodeAwakeIndex(int(contact.encodedBodySimA))
			indexB := decodeAwakeIndex(int(contact.encodedBodySimB))

			// PORT: #if B3_ENABLE_VALIDATION becomes a branch on validateEnabled.
			if validateEnabled && context.localJointBodies == nil {
				bodyA := arrayGet(world.bodies, int(contact.edges[0].bodyId))
				bodyB := arrayGet(world.bodies, int(contact.edges[1].bodyId))
				assert(int(contact.encodedBodySimA) == encodeBodySimIndex(bodyA), "contact->encodedBodySimA == b3EncodeBodySimIndex( bodyA )")
				assert(int(contact.encodedBodySimB) == encodeBodySimIndex(bodyB), "contact->encodedBodySimB == b3EncodeBodySimIndex( bodyB )")
			}

			// Body A data
			var mA float32
			var iA Matrix3

			if indexA == NullIndex {
				mA = 0.0
				iA = Mat3Zero
			} else {
				simA := &bodySims[indexA]
				mA = simA.invMass
				iA = simA.invInertiaWorld
			}

			// Body B data
			var mB float32
			var iB Matrix3

			if indexB == NullIndex {
				mB = 0.0
				iB = Mat3Zero
			} else {
				simB := &bodySims[indexB]
				mB = simB.invMass
				iB = simB.invInertiaWorld
			}

			manifoldCount := int(contact.manifoldCount)
			contactConstraint.contact = contact
			contactConstraint.manifoldCount = int32(manifoldCount)
			contactConstraint.indexA = int32(indexA)
			contactConstraint.indexB = int32(indexB)
			contactConstraint.invIA = iA
			contactConstraint.invMassA = mA
			contactConstraint.invIB = iB
			contactConstraint.invMassB = mB
			contactConstraint.rollingMass = InvertMatrix(AddMM(iA, iB))
			if (contact.flags & contactStaticFlag) != 0 {
				contactConstraint.softness = context.staticSoftness
			} else {
				contactConstraint.softness = context.contactSoftness
			}
			contactConstraint.friction = contact.friction
			contactConstraint.restitution = contact.restitution
			contactConstraint.rollingResistance = contact.rollingResistance
			contactConstraint.previousTOI = world.enableContinuous &&
				((indexA != NullIndex && bodySims[indexA].flags&hadTimeOfImpact != 0) ||
					(indexB != NullIndex && bodySims[indexB].flags&hadTimeOfImpact != 0))

			// Only sample contact point normal velocity if needed.
			haveRestitution := contact.restitution > 0.0
			hitEvents := (contact.flags & simEnableHitEvent) != 0
			sampleVelocity := haveRestitution || hitEvents || contactConstraint.previousTOI
			anyRestitution = anyRestitution || haveRestitution

			vA := Vec3Zero
			wA := Vec3Zero
			vB := Vec3Zero
			wB := Vec3Zero

			if sampleVelocity {
				if indexA != NullIndex {
					vA = states[indexA].linearVelocity
					wA = states[indexA].angularVelocity
				}

				if indexB != NullIndex {
					vB = states[indexB].linearVelocity
					wB = states[indexB].angularVelocity
				}
			}

			// PORT: the manifold constraint pointer is a slice of manifoldCount constraints.
			manifoldStart := int(specs[localIndex].manifoldStart)
			manifoldConstraints := manifoldBase[manifoldStart : manifoldStart+manifoldCount]
			contactConstraint.constraints = manifoldConstraints

			for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
				manifold := &contact.manifolds[manifoldIndex]
				constraint := &manifoldConstraints[manifoldIndex]
				pointCount := int(manifold.PointCount)
				normal := manifold.Normal
				tangent1 := Perp(normal)
				tangent2 := Cross(tangent1, normal)

				constraint.pointCount = int32(pointCount)
				constraint.normal = normal
				constraint.tangent1 = tangent1
				constraint.tangent2 = tangent2

				// Stiffer for static contacts to avoid bodies getting pushed through the ground
				constraint.tangentVelocity1 = Dot(contact.tangentVelocity, constraint.tangent1)
				constraint.tangentVelocity2 = Dot(contact.tangentVelocity, constraint.tangent2)

				centerA := Vec3Zero
				centerB := Vec3Zero
				totalFrictionWeight := float32(0.0)

				for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
					cp := &constraint.points[pointIndex]

					// Copy data from manifold point
					mp := &manifold.Points[pointIndex]
					cp.rA = mp.AnchorA
					cp.rB = mp.AnchorB

					s := mp.Separation
					cp.baseSeparation = s - Dot(Sub(cp.rB, cp.rA), normal)
					cp.normalImpulse = float32(warmStartScale * mp.NormalImpulse)
					cp.totalNormalImpulse = 0.0
					cp.restitutionImpulse = 0.0

					rA := cp.rA
					rB := cp.rB

					rnA := Cross(rA, normal)
					rnB := Cross(rB, normal)
					kNormal := mA + mB + Dot(rnA, MulMV(iA, rnA)) + Dot(rnB, MulMV(iB, rnB))
					if kNormal > 0.0 {
						cp.normalMass = 1.0 / kNormal
					} else {
						cp.normalMass = 0.0
					}

					// Only compute the normal velocity terms if needed.
					if sampleVelocity {
						vrA := Add(vA, Cross(wA, rA))
						vrB := Add(vB, Cross(wB, rB))
						vn := Dot(normal, Sub(vrB, vrA))

						cp.relativeVelocity = vn
						if hitEvents {
							mp.NormalVelocity = vn
						} else {
							mp.NormalVelocity = 0.0
						}
					} else {
						cp.relativeVelocity = 0.0
						mp.NormalVelocity = 0.0
					}

					// C0 friction center decay. Needed to prevent spinning top drift (GyroscopicPrecession sample).
					// Contacts with separation greater than twice the speculative distance only matter for CCD and
					// should not contribute to the friction center. They are not important for jitter reduction. Closer
					// points may begin to touch on and off, so the friction center needs to move smoothly.
					// Epsilon to avoid a branch below (or divide by zero). Small enough to get washed out normally.
					//
					// Dirk suggested to weight the points by their normal impulse. This would use the total impulse from
					// the previous time step. I suspect this could jump around if the active point set jitters, leading
					// to a limit cycle.
					weight := ClampFloat(2.0-float32(s*invTau), MinFrictionWeight, 1.0)
					centerA = MulAdd(centerA, weight, rA)
					centerB = MulAdd(centerB, weight, rB)
					totalFrictionWeight += weight
				}

				invWeight := 1.0 / totalFrictionWeight
				centerA = MulSV(invWeight, centerA)
				centerB = MulSV(invWeight, centerB)
				constraint.centerA = centerA
				constraint.centerB = centerB

				for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
					cp := &constraint.points[pointIndex]
					cp.leverArm = Distance(cp.rA, centerA)
				}

				rtA1 := Cross(centerA, tangent1)
				rtA2 := Cross(centerA, tangent2)
				rtB1 := Cross(centerB, tangent1)
				rtB2 := Cross(centerB, tangent2)

				{
					var k matrix2
					k.cx.X = mA + mB + Dot(rtA1, MulMV(iA, rtA1)) + Dot(rtB1, MulMV(iB, rtB1))
					k.cy.Y = mA + mB + Dot(rtA2, MulMV(iA, rtA2)) + Dot(rtB2, MulMV(iB, rtB2))
					k.cy.X = Dot(rtA1, MulMV(iA, rtA2)) + Dot(rtB1, MulMV(iB, rtB2))
					k.cx.Y = k.cy.X

					constraint.tangentMass = invert2(k)
					constraint.frictionImpulse.X = float32(warmStartScale * Dot(manifold.FrictionImpulse, tangent1))
					constraint.frictionImpulse.Y = float32(warmStartScale * Dot(manifold.FrictionImpulse, tangent2))
				}

				{
					k := Dot(normal, MulMV(AddMM(iA, iB), normal))
					if k > 0.0 {
						constraint.twistMass = 1.0 / k
					} else {
						constraint.twistMass = 0.0
					}
					constraint.twistImpulse = float32(warmStartScale * manifold.TwistImpulse)
				}

				{
					constraint.rollingImpulse = MulSV(warmStartScale, manifold.RollingImpulse)
				}
			}
		}

		// Advance to next color
		colorIndex += 1
	}

	if anyRestitution {
		atomicStoreInt(&context.anyRestitution, 1)
	}
}

// warmStartContactsMesh is b3WarmStartContacts_Mesh.
func warmStartContactsMesh(block solverBlock, context *stepContext) {
	world := context.world
	color := &world.constraintGraph.colors[block.colorIndex]
	// PORT: awakeSet is the set index constant; the solver set is awakeSolverSet.
	awakeSolverSet := arrayGet(world.solverSets, awakeSet)
	states := awakeSolverSet.bodyStates
	constraints := color.contactConstraints

	// This is a dummy state to represent a static body because static bodies don't have a solver body.
	dummyState := identityBodyState

	startIndex := int(block.startIndex)
	endIndex := startIndex + int(block.count)

	for constraintIndex := startIndex; constraintIndex < endIndex; constraintIndex++ {
		contactConstraint := &constraints[constraintIndex]

		linearImpulse := Vec3Zero
		angularImpulseA := Vec3Zero
		angularImpulseB := Vec3Zero

		manifoldCount := int(contactConstraint.manifoldCount)
		for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
			constraint := &contactConstraint.constraints[manifoldIndex]

			// Normal impulses
			normal := constraint.normal
			totalNormalImpulse := float32(0.0)
			momentA := Vec3Zero
			momentB := Vec3Zero

			pointCount := int(constraint.pointCount)
			for j := 0; j < pointCount; j++ {
				cp := &constraint.points[j]

				// fixed anchors
				totalNormalImpulse += cp.normalImpulse
				momentA = MulAdd(momentA, cp.normalImpulse, cp.rA)
				momentB = MulAdd(momentB, cp.normalImpulse, cp.rB)
			}

			// Central friction
			frictionImpulse := MulSV(constraint.frictionImpulse.X, constraint.tangent1)
			frictionImpulse = MulAdd(frictionImpulse, constraint.frictionImpulse.Y, constraint.tangent2)

			linearImpulse = Add(linearImpulse, MulAdd(frictionImpulse, totalNormalImpulse, normal))

			// Central twist friction
			twistImpulse := MulSV(constraint.twistImpulse, normal)

			angularA :=
				Add(Add(Cross(momentA, normal), Cross(constraint.centerA, frictionImpulse)), twistImpulse)
			angularB :=
				Add(Add(Cross(momentB, normal), Cross(constraint.centerB, frictionImpulse)), twistImpulse)

			// Rolling resistance
			angularImpulseA = Add(angularImpulseA, Add(angularA, constraint.rollingImpulse))
			angularImpulseB = Add(angularImpulseB, Add(angularB, constraint.rollingImpulse))
		}

		indexA := int(contactConstraint.indexA)
		indexB := int(contactConstraint.indexB)

		stateA := &dummyState
		if indexA != NullIndex {
			stateA = &states[indexA]
		}
		stateB := &dummyState
		if indexB != NullIndex {
			stateB = &states[indexB]
		}

		if stateA.flags&dynamicFlag != 0 {
			stateA.linearVelocity = MulSub(stateA.linearVelocity, contactConstraint.invMassA, linearImpulse)
			stateA.angularVelocity = Sub(stateA.angularVelocity, MulMV(contactConstraint.invIA, angularImpulseA))
		}

		if stateB.flags&dynamicFlag != 0 {
			stateB.linearVelocity = MulAdd(stateB.linearVelocity, contactConstraint.invMassB, linearImpulse)
			stateB.angularVelocity = Add(stateB.angularVelocity, MulMV(contactConstraint.invIB, angularImpulseB))
		}
	}
}

// pushContactsMesh is b3PushContacts_Mesh.
// Merged normal and friction loops. This is much more stable for the Jenga stack.
// Solve the non-penetration constraints with the soft bias. No friction and no restitution.
func pushContactsMesh(block solverBlock, context *stepContext) {
	world := context.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context.states

	// This is a dummy state to represent a static body because static bodies have no solver body.
	dummyState := identityBodyState

	// The last block might not be full
	startIndex := int(block.startIndex)
	endIndex := startIndex + int(block.count)

	inv_h := context.inv_h
	contactSpeed := context.world.contactSpeed

	for i := startIndex; i < endIndex; i++ {
		contactConstraint := &contactConstraints[i]
		manifoldCount := int(contactConstraint.manifoldCount)

		indexA := int(contactConstraint.indexA)
		indexB := int(contactConstraint.indexB)

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := &dummyState
		if indexA != NullIndex {
			stateA = &states[indexA]
		}
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := &dummyState
		if indexB != NullIndex {
			stateB = &states[indexB]
		}
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)
		softness := contactConstraint.softness

		for j := 0; j < manifoldCount; j++ {
			constraint := &contactConstraint.constraints[j]

			pointCount := int(constraint.pointCount)
			normal := constraint.normal

			for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
				cp := &constraint.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.rA
				rB := cp.rB

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
				s := Dot(ds, normal) + cp.baseSeparation

				var velocityBias float32
				var massScale float32
				var impulseScale float32
				if s > 0.0 {
					// speculative bias is positive
					velocityBias = float32(s * inv_h)
					massScale = 1.0
					impulseScale = 0.0
				} else {
					// overlap bias is negative
					velocityBias = MaxFloat(float32(float32(softness.massScale*softness.biasRate)*s), -contactSpeed)
					massScale = softness.massScale
					impulseScale = softness.impulseScale
				}
				// A TOI clips position but retains impact velocity for this solve.
				// Soft compliance must not advance that velocity through the surface
				// before relaxation. Keep the existing bounded recovery bias.
				if contactConstraint.previousTOI && cp.relativeVelocity < 0.0 {
					massScale = 1.0
					impulseScale = 0.0
				}

				// relative normal velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				// incremental normal impulse
				deltaImpulse := float32(-cp.normalMass*(float32(massScale*vn)+velocityBias)) - float32(impulseScale*cp.normalImpulse)

				// clamp the accumulated impulse
				newImpulse := MaxFloat(cp.normalImpulse+deltaImpulse, 0.0)
				deltaImpulse = newImpulse - cp.normalImpulse
				cp.normalImpulse = newImpulse

				// apply normal impulse
				P := MulSV(deltaImpulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
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
}

// solveContactsMesh is b3SolveContacts_Mesh.
// Solve contacts: normal, friction and rolling resistance.
func solveContactsMesh(block solverBlock, context *stepContext) {
	world := context.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context.states

	// This is a dummy state to represent a static body because static bodies have no solver body.
	dummyState := identityBodyState

	// The last block might not be full
	startIndex := int(block.startIndex)
	endIndex := startIndex + int(block.count)

	inv_h := context.inv_h

	for i := startIndex; i < endIndex; i++ {
		contactConstraint := &contactConstraints[i]
		manifoldCount := int(contactConstraint.manifoldCount)

		indexA := int(contactConstraint.indexA)
		indexB := int(contactConstraint.indexB)

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := &dummyState
		if indexA != NullIndex {
			stateA = &states[indexA]
		}
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := &dummyState
		if indexB != NullIndex {
			stateB = &states[indexB]
		}
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)
		friction := contactConstraint.friction
		rollingResistance := contactConstraint.rollingResistance

		for j := 0; j < manifoldCount; j++ {
			constraint := &contactConstraint.constraints[j]

			pointCount := int(constraint.pointCount)
			normal := constraint.normal

			totalNormalImpulse := float32(0.0)
			totalTwistLimit := float32(0.0)

			for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
				cp := &constraint.points[pointIndex]

				// Fixed anchor points for applying impulses
				rA := cp.rA
				rB := cp.rB

				// compute current separation
				// this is subject to round-off error if the anchor is far from the body center of mass
				ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
				s := Dot(ds, normal) + cp.baseSeparation

				// speculative bias, zero when overlapped
				velocityBias := float32(0.0)
				if s > 0.0 {
					velocityBias = float32(s * inv_h)
				}

				// relative normal velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				// incremental normal impulse
				deltaImpulse := float32(-cp.normalMass * (vn + velocityBias))

				// clamp the accumulated impulse
				newImpulse := MaxFloat(cp.normalImpulse+deltaImpulse, 0.0)
				deltaImpulse = newImpulse - cp.normalImpulse
				cp.normalImpulse = newImpulse
				if !context.skipContactImpulseAccumulation {
					cp.totalNormalImpulse += newImpulse
				}
				totalNormalImpulse += newImpulse
				totalTwistLimit += float32(cp.leverArm * newImpulse)

				// apply normal impulse
				P := MulSV(deltaImpulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}

			// Central twist friction
			{
				twistSpeed := Dot(constraint.normal, Sub(wB, wA))
				maxImpulse := float32(friction * totalTwistLimit)
				deltaImpulse := float32(-constraint.twistMass * twistSpeed)
				oldImpulse := constraint.twistImpulse
				constraint.twistImpulse = ClampFloat(oldImpulse+deltaImpulse, -maxImpulse, maxImpulse)
				deltaImpulse = constraint.twistImpulse - oldImpulse

				wA = Sub(wA, MulMV(iA, MulSV(deltaImpulse, constraint.normal)))
				wB = Add(wB, MulMV(iB, MulSV(deltaImpulse, constraint.normal)))
			}

			// Rolling resistance
			if rollingResistance > 0.0 {
				deltaImpulse := Neg(MulMV(contactConstraint.rollingMass, Sub(wB, wA)))
				oldImpulse := constraint.rollingImpulse
				constraint.rollingImpulse = Add(oldImpulse, deltaImpulse)

				maxImpulse := float32(rollingResistance * totalNormalImpulse)
				magSqr := Dot(constraint.rollingImpulse, constraint.rollingImpulse)
				if magSqr > float32(maxImpulse*maxImpulse)+fltEpsilon {
					constraint.rollingImpulse = MulSV(maxImpulse/sqrtf(magSqr), constraint.rollingImpulse)
				}

				deltaImpulse = Sub(constraint.rollingImpulse, oldImpulse)

				wA = Sub(wA, MulMV(iA, deltaImpulse))
				wB = Add(wB, MulMV(iB, deltaImpulse))
			}

			// Central friction
			{
				tangent1 := constraint.tangent1
				tangent2 := constraint.tangent2

				// Fixed anchor points for applying impulses
				rA := constraint.centerA
				rB := constraint.centerB

				// Relative tangent velocity at contact
				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vr := Sub(vrB, vrA)
				vt := Vec2{
					Dot(vr, tangent1) - constraint.tangentVelocity1,
					Dot(vr, tangent2) - constraint.tangentVelocity2,
				}

				// Incremental tangent impulse
				tm := mulMV2(constraint.tangentMass, vt)
				deltaImpulse := Vec2{-tm.X, -tm.Y}
				newImpulse := Vec2{
					constraint.frictionImpulse.X + deltaImpulse.X,
					constraint.frictionImpulse.Y + deltaImpulse.Y,
				}

				maxImpulse := float32(friction * totalNormalImpulse)

				// Clamp the accumulated impulse
				lengthSquared := dot2(newImpulse, newImpulse)
				if lengthSquared > float32(maxImpulse*maxImpulse) {
					scale := maxImpulse / sqrtf(lengthSquared)
					newImpulse.X = float32(newImpulse.X * scale)
					newImpulse.Y = float32(newImpulse.Y * scale)
				}
				deltaImpulse = sub2(newImpulse, constraint.frictionImpulse)
				constraint.frictionImpulse = newImpulse

				// Apply delta impulse
				P := Blend2(deltaImpulse.X, tangent1, deltaImpulse.Y, tangent2)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))
				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
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
}

// applyRestitutionMesh is b3ApplyRestitution_Mesh.
func applyRestitutionMesh(block solverBlock, context *stepContext) {
	world := context.world
	color := &world.constraintGraph.colors[block.colorIndex]
	contactConstraints := color.contactConstraints
	states := context.states

	threshold := world.restitutionThreshold
	inv_h := context.inv_h
	propagate := world.enableRestitutionPropagation

	dummyState := identityBodyState

	startIndex := int(block.startIndex)
	endIndex := startIndex + int(block.count)

	for i := startIndex; i < endIndex; i++ {
		contactConstraint := &contactConstraints[i]
		restitution := contactConstraint.restitution
		if propagate == false && restitution == 0.0 {
			continue
		}

		manifoldCount := int(contactConstraint.manifoldCount)

		indexA := int(contactConstraint.indexA)
		indexB := int(contactConstraint.indexB)

		mA := contactConstraint.invMassA
		iA := contactConstraint.invIA
		mB := contactConstraint.invMassB
		iB := contactConstraint.invIB

		stateA := &dummyState
		if indexA != NullIndex {
			stateA = &states[indexA]
		}
		vA := stateA.linearVelocity
		wA := stateA.angularVelocity
		dqA := stateA.deltaRotation

		stateB := &dummyState
		if indexB != NullIndex {
			stateB = &states[indexB]
		}
		vB := stateB.linearVelocity
		wB := stateB.angularVelocity
		dqB := stateB.deltaRotation

		dp := Sub(stateB.deltaPosition, stateA.deltaPosition)

		for j := 0; j < manifoldCount; j++ {
			constraint := &contactConstraint.constraints[j]

			pointCount := int(constraint.pointCount)
			normal := constraint.normal

			for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
				cp := &constraint.points[pointIndex]

				rA := cp.rA
				rB := cp.rB

				// The total normal impulse is 0 for speculative points.
				compressionImpulse := cp.totalNormalImpulse - cp.restitutionImpulse
				armed := restitution > 0.0 && cp.relativeVelocity < -threshold && compressionImpulse > 0.0

				var velocityBias float32
				if armed {
					velocityBias = float32(restitution * cp.relativeVelocity)
				} else {
					ds := Add(dp, Sub(RotateVector(dqB, rB), RotateVector(dqA, rA)))
					s := Dot(ds, normal) + cp.baseSeparation

					if s > 0.0 {
						velocityBias = float32(s * inv_h)
					} else {
						velocityBias = 0.0
					}
				}

				vrA := Add(vA, Cross(wA, rA))
				vrB := Add(vB, Cross(wB, rB))
				vn := Dot(Sub(vrB, vrA), normal)

				impulse := float32(-cp.normalMass * (vn + velocityBias))

				newImpulse := MaxFloat(cp.normalImpulse+impulse, 0.0)
				impulse = newImpulse - cp.normalImpulse

				approachImpulse := MinFloat(MaxFloat(float32(-cp.normalMass*vn), 0.0), MaxFloat(impulse, 0.0))

				if armed {
					// Poisson kinetic restitution guarantees no energy gain.
					allowance := float32(restitution*(compressionImpulse+approachImpulse)) - cp.restitutionImpulse
					impulse = MinFloat(impulse, approachImpulse+MaxFloat(allowance, 0.0))
				}

				cp.normalImpulse += impulse
				cp.restitutionImpulse += impulse - approachImpulse
				cp.totalNormalImpulse += impulse

				P := MulSV(impulse, normal)
				vA = MulSub(vA, mA, P)
				wA = Sub(wA, MulMV(iA, Cross(rA, P)))

				vB = MulAdd(vB, mB, P)
				wB = Add(wB, MulMV(iB, Cross(rB, P)))
			}
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
}

// storeImpulsesMesh is b3StoreImpulses_Mesh.
// Don't need to use spans for colors for this because the constraint to contact association
// is already linked by pointer.
func storeImpulsesMesh(block solverBlock, context *stepContext, workerIndex int) {
	world := context.world

	assert(block.blockType == overflowBlockType, "block.blockType == b3_overflowBlock")
	overflow := &world.constraintGraph.colors[overflowIndex]
	spans := context.overflowSpans
	base := overflow.contactConstraints

	taskContext := &world.taskContexts[workerIndex]
	hitEventBitSet := &taskContext.hitEventBitSet
	hasHitEvents := taskContext.hasHitEvents
	negHitThreshold := -world.hitEventThreshold

	index := int(block.startIndex)
	endIndex := int(block.startIndex) + int(block.count)

	// Find color for start index. Linear search but fast.
	colorIndex := 0
	for int(spans[colorIndex+1].start) <= index {
		colorIndex += 1
	}

	// Loop over block
	for index < endIndex {
		colorStart := int(spans[colorIndex].start)
		colorEndIndex := MinInt(int(spans[colorIndex+1].start), endIndex)

		// Loop over color
		for ; index < colorEndIndex; index++ {
			contactConstraint := &base[index]

			localIndex := index - colorStart
			assert(0 <= localIndex && localIndex < int(spans[colorIndex].count), "0 <= localIndex && localIndex < spans[colorIndex].count")

			// Having this contact pointer simplifies impulse storage
			contact := contactConstraint.contact
			assert(contact != nil, "contact != NULL")

			// Catches the wrong-(base, spans) pairing: the contact pointer stashed by
			// b3PrepareContacts_Mesh at this flat slot must reference the same contact
			// the span at this slot describes.
			validate(contact.contactId == spans[colorIndex].contacts[localIndex].contactId, "contact->contactId == spans[colorIndex].contacts[localIndex].contactId")

			manifoldCount := int(contactConstraint.manifoldCount)
			assert(manifoldCount == int(contact.manifoldCount), "manifoldCount == contact->manifoldCount")

			checkHitEvents := (contact.flags & simEnableHitEvent) != 0
			flagged := false

			for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
				manifold := &contact.manifolds[manifoldIndex]
				constraint := &contactConstraint.constraints[manifoldIndex]
				manifold.TwistImpulse = constraint.twistImpulse
				manifold.FrictionImpulse = Blend2(constraint.frictionImpulse.X, constraint.tangent1,
					constraint.frictionImpulse.Y, constraint.tangent2)
				manifold.RollingImpulse = constraint.rollingImpulse

				count := int(constraint.pointCount)
				assert(count == int(manifold.PointCount), "count == manifold->pointCount")
				for pointIndex := 0; pointIndex < count; pointIndex++ {
					cp := &constraint.points[pointIndex]
					mp := &manifold.Points[pointIndex]
					mp.NormalImpulse = cp.normalImpulse
					mp.TotalNormalImpulse = cp.totalNormalImpulse

					if checkHitEvents && flagged == false && mp.NormalVelocity < negHitThreshold &&
						mp.TotalNormalImpulse > 0.0 {
						setBit(hitEventBitSet, uint32(contact.contactId))
						hasHitEvents = true
						flagged = true
					}
				}
			}
		}

		// Advance to next color
		colorIndex += 1
	}

	taskContext.hasHitEvents = hasHitEvents
}

// prepareContactsOverflow is b3PrepareContacts_Overflow.
func prepareContactsOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	assert(len(color.contacts) <= math.MaxUint16, "color->contacts.count <= UINT16_MAX")
	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	prepareContactsMesh(block, context)
}

// warmStartContactsOverflow is b3WarmStartContacts_Overflow.
func warmStartContactsOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	warmStartContactsMesh(block, context)
}

// pushContactsOverflow is b3PushContacts_Overflow.
func pushContactsOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	pushContactsMesh(block, context)
}

// solveContactsOverflow is b3SolveContacts_Overflow.
func solveContactsOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	solveContactsMesh(block, context)
}

// applyRestitutionOverflow is b3ApplyRestitution_Overflow.
func applyRestitutionOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	applyRestitutionMesh(block, context)
}

// storeImpulsesOverflow is b3StoreImpulses_Overflow.
func storeImpulsesOverflow(context *stepContext) {
	graph := context.graph
	color := &graph.colors[overflowIndex]

	count := uint16(len(color.contacts))
	if count == 0 {
		return
	}

	block := solverBlock{
		startIndex: 0,
		count:      count,
		blockType:  overflowBlockType,
		colorIndex: overflowIndex,
	}

	storeImpulsesMesh(block, context, 0)
}

// PORT: the B3_SIMD_HAS_WIDTH_8 branches of the dispatchers below call the W8 kernels through the
// wide8 table (wide_kernels.go). world.simdWidth is 8 only when wide8 is set (detectSIMDWidth), so
// the branches need no nil check.

// prepareContactsConvex is b3PrepareContacts_Convex.
func prepareContactsConvex(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.prepareContactsConvex(block, context)
		return
	}

	prepareContactsConvexW(block, context)
}

// warmStartContactsConvex is b3WarmStartContacts_Convex.
func warmStartContactsConvex(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.warmStartContactsConvex(block, context)
		return
	}

	warmStartContactsConvexW(block, context)
}

// pushContactsConvex is b3PushContacts_Convex.
func pushContactsConvex(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.pushContactsConvex(block, context)
		return
	}

	pushContactsConvexW(block, context)
}

// solveContactsConvex is b3SolveContacts_Convex.
func solveContactsConvex(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.solveContactsConvex(block, context)
		return
	}

	solveContactsConvexW(block, context)
}

// applyRestitutionConvex is b3ApplyRestitution_Convex.
func applyRestitutionConvex(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.applyRestitutionConvex(block, context)
		return
	}

	applyRestitutionConvexW(block, context)
}

// storeImpulsesConvex is b3StoreImpulses_Convex.
func storeImpulsesConvex(block solverBlock, context *stepContext, workerIndex int) {
	if context.world.simdWidth == 8 {
		wide8.storeImpulsesConvex(block, context, workerIndex)
		return
	}

	storeImpulsesConvexW(block, context, workerIndex)
}

// getWideContactConstraintByteCount is b3GetWideContactConstraintByteCount.
func getWideContactConstraintByteCount(simdWidth int) int {
	if simdWidth == 8 {
		return wide8.contactConstraintByteCount()
	}

	return getWideContactConstraintByteCountW()
}

// prepareContactsMeshWide is b3PrepareContacts_MeshWide.
func prepareContactsMeshWide(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.prepareContactsMeshWide(block, context)
		return
	}

	prepareContactsMeshWideW(block, context)
}

// warmStartContactsMeshWide is b3WarmStartContacts_MeshWide.
func warmStartContactsMeshWide(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.warmStartContactsMeshWide(block, context)
		return
	}

	warmStartContactsMeshWideW(block, context)
}

// pushContactsMeshWide is b3PushContacts_MeshWide.
func pushContactsMeshWide(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.pushContactsMeshWide(block, context)
		return
	}

	pushContactsMeshWideW(block, context)
}

// solveContactsMeshWide is b3SolveContacts_MeshWide.
func solveContactsMeshWide(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.solveContactsMeshWide(block, context)
		return
	}

	solveContactsMeshWideW(block, context)
}

// applyRestitutionMeshWide is b3ApplyRestitution_MeshWide.
func applyRestitutionMeshWide(block solverBlock, context *stepContext) {
	if context.world.simdWidth == 8 {
		wide8.applyRestitutionMeshWide(block, context)
		return
	}

	applyRestitutionMeshWideW(block, context)
}

// storeImpulsesMeshWide is b3StoreImpulses_MeshWide.
func storeImpulsesMeshWide(block solverBlock, context *stepContext, workerIndex int) {
	if context.world.simdWidth == 8 {
		wide8.storeImpulsesMeshWide(block, context, workerIndex)
		return
	}

	storeImpulsesMeshWideW(block, context, workerIndex)
}

// getWideMeshConstraintByteCount is b3GetWideMeshConstraintByteCount.
func getWideMeshConstraintByteCount(simdWidth int) int {
	if simdWidth == 8 {
		return wide8.meshConstraintByteCount()
	}

	return getWideMeshConstraintByteCountW()
}

// getWideMeshManifoldByteCount is b3GetWideMeshManifoldByteCount.
func getWideMeshManifoldByteCount(simdWidth int) int {
	if simdWidth == 8 {
		return wide8.meshManifoldByteCount()
	}

	return getWideMeshManifoldByteCountW()
}
