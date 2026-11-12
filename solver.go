// Port of src/solver.h and src/solver.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
)

// ---- solver.h ----

// Solver work is partitioned into fixed-size blocks that worker threads claim
// in parallel via atomic CAS on a per-block syncIndex. The descriptor (b3SolverBlock)
// and the atomic counter sit in a wrapping b3SyncBlock so the CAS-winner can
// pass the descriptor by value into stage tasks without aliasing the atomic
// memory other threads are CAS-writing. Three properties of this design
// matter for performance:
//
// 1. Distributed contention. Per-block atomic syncIndex avoids the cache line stampede
//    that a single shared fetch_add counter would cause. Once a worker
//    settles into a block range, its CAS targets live in its own L1.
//
// 2. Monotonic syncIndex across iterations. Iterative stages (warm start,
//    solve, relax) reuse the same block array every sub-step iteration.
//    syncIndex grows each iteration; workers CAS (prev, prev+1), so the
//    main thread never touches any per-block state between iterations.
//    Non-iterative stages simply use syncIndex 1.
//
// 3. L2 affinity across iterations. Each worker picks a start offset from
//    its workerIndex, then scans forward and (after wrap) backward:
//
//      blocks:   [0] [1] [2] [3] [4] [5] [6] [7]
//                 ^           ^           ^   ^
//                 W0          W1          W2  W3   <- start offsets
//
//    W0 claims 0,1,2,3 (forward), W1 claims 4,5, etc. Under balanced load
//    each worker re-hits the same block range every iteration, keeping that
//    range's hot data resident in its L2. A failed CAS means a neighbour
//    already claimed the block, so the stealing worker stops -- preserving
//    locality under mild imbalance while still draining the queue.
//
// A graph color stage lays out joint blocks first, then contact blocks:
//
//      stage->blocks ->
//        +------+------+------+------+------+------+------+
//        |  J0  |  J1  |  J2  |  C0  |  C1  |  C2  |  C3  |
//        +------+------+------+------+------+------+------+
//        <-- graphJointBlocks --><---- graphContactBlocks ---->
//
// Each block carries its type so the dispatcher routes J-blocks to the joint
// solver and C-blocks to the SIMD contact solver; both kinds run concurrently
// within the stage -- no barrier between them. The type tag lives on the
// block (not the stage) so that mixed-type stages can keep the concurrency.
//
// The solver threading model is inspired by https://github.com/bepu/bepuphysics2

// solverStageType is b3SolverStageType: solver stages
type solverStageType int32

const (
	stagePrepareJoints solverStageType = iota
	stagePrepareWideContacts
	stagePrepareContacts
	stageIntegrateVelocities
	stageWarmStart
	stageSolve
	stageIntegratePositions
	stageRelax
	stageRestitution
	stageStoreWideImpulses
	stageStoreImpulses
)

// solverBlockType is b3SolverBlockType.
// PORT: untyped constants; upstream stores the value in the uint8_t solverBlock.blockType.
const (
	// Block for iterating across bodies.
	bodyBlock = iota

	// Block for iterating across joints. For prepare.
	jointBlock

	// Block for iterating across wide contacts. For prepare and store.
	wideContactBlock

	// Block for iterating across contacts. For prepare and store.
	contactBlock

	// Block for iterating across joints of a single graph color.
	graphJointBlock

	// Block for iterating across wide contacts of a single graph color.
	graphWideContactBlock

	// Block for iterating across contacts of a single graph color.
	graphContactBlock

	// Block for processing overflow constraints
	// PORT: b3_overflowBlock; renamed because overflowBlock is the arena allocator's b3OverflowBlock.
	overflowBlockType
)

// solverBlock is b3SolverBlock.
// Solver block describes a multithreaded unit of work.
type solverBlock struct {
	startIndex int32
	count      uint16
	// b3SolverBlockType
	blockType  uint8
	colorIndex uint8
}

// syncBlock is b3SyncBlock.
// A unit of multithreaded work along with atomic synchronization. The syncIndex grows
// monotonically allowing the solver block to be re-used across sub-steps.
type syncBlock struct {
	block     solverBlock
	syncIndex atomicInt
}

// solverStage is b3SolverStage.
// Each stage must be completed before going to the next stage.
// Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
// PORT: b3SyncBlock* blocks is a slice; blockCount is kept.
type solverStage struct {
	blocks          []syncBlock
	typ             solverStageType
	blockCount      int32
	colorIndex      uint8
	completionCount atomicInt
}

// softness is b3Softness: constraint softness
type softness struct {
	biasRate     float32
	massScale    float32
	impulseScale float32
}

// widePrepareSpan is b3WidePrepareSpan.
// Prepare/store run as a flat parallel-for over the whole wide-constraint
// range. Each span maps a slice of that range back to the owning color's
// contacts so workers can decode flat wide-slot indices without touching
// graph state. The spans array has one entry per active color plus a sentinel
// whose start == wideContactCount.
// PORT: int* contacts is a slice.
type widePrepareSpan struct {
	start    int32
	count    int32
	contacts []int32
}

// contactPrepareSpan is b3ContactPrepareSpan.
// PORT: b3ContactSpec* contacts is a slice.
type contactPrepareSpan struct {
	start    int32
	count    int32
	contacts []contactSpec
}

// meshPrepareSpan is b3MeshPrepareSpan.
// PORT: the pointers are slices.
type meshPrepareSpan struct {
	start    int32
	count    int32
	contacts []contactSpec
	order    []int32
}

// jointPrepareSpan is b3JointPrepareSpan.
// PORT: b3JointSim* joints is a slice.
type jointPrepareSpan struct {
	start  int32
	count  int32
	joints []jointSim
}

// stepContext is b3StepContext.
// Context for a time step. Recreated each time step.
// PORT: pointer + count pairs are slices with the count fields kept; the void* wide constraint
// buffers are wideConstraintBuffer (contact_solver_wide.go), or wideConstraintBuffer8 in the `8`
// fields when the world runs at width 8; the false-sharing padding is kept.
type stepContext struct {
	// time step
	dt float32

	// inverse time step (0 if dt == 0).
	inv_dt float32

	// sub-step
	h     float32
	inv_h float32

	subStepCount int32

	contactSoftness softness
	staticSoftness  softness

	maxLinearVelocity float32

	world *world
	graph *constraintGraph

	// shortcut to body states from awake set
	states []bodyState

	// shortcut to body sims from awake set
	sims []bodySim

	// array of all shape ids for shapes that have enlarged AABBs
	enlargedShapes     []int32
	enlargedShapeCount int32

	// Array of bullet bodies that need continuous collision handling
	bulletBodies    []int32
	bulletBodyCount atomicInt

	anyRestitution atomicInt

	// Contact ids for simplified parallel-for access. Used in narrow-phase.
	// These contacts may or may not be touching. They are associated with awake bodies.
	awakeContactIndices []int32

	// Flat view of the wide contact constraint array used by prepare and store.
	// prepareSpans has activeColorCount + 1 entries, the last being a sentinel
	// at wideContactCount. wideContactConstraints is the contiguous base
	// pointer. Per-color slices live at colors[i].wideConstraints.
	wideConstraints  wideConstraintBuffer
	wideConstraints8 wideConstraintBuffer8
	widePrepareSpans []widePrepareSpan
	wideContactCount int32

	// Similar for overflow contact constraints.
	overflowSpans          []contactPrepareSpan
	wideMeshConstraints    wideConstraintBuffer
	wideMeshConstraints8   wideConstraintBuffer8
	wideMeshManifoldStarts []int32
	meshPrepareSpans       []meshPrepareSpan
	jointPrepareSpans      []jointPrepareSpan

	activeColorCount                                int32
	workerCount                                     int32
	motionVelocityIterations, motionRelaxIterations int
	skipContactImpulseAccumulation                  bool

	stages             []solverStage
	stageCount         int32
	enableWarmStarting bool

	// padding to prevent false sharing
	padding1 [64]byte

	// This atomic is central to multi-threaded solver task synchronization.
	// It prevents ABA problems by monotonically growing as the solver advances.
	// This means a delayed worker thread will catch up without repeating already completed
	// work (causing a race condition).
	// sync index (16-bits) | stage type (16-bits)
	atomicSyncBits atomicU32

	// padding to prevent false sharing
	padding2 [64]byte

	// Race flag claimed by whichever runner reaches b3SolverTask with workerIndex 0 first.
	// The calling thread of b3World_Step also races for this slot so the orchestrator can
	// always make progress, regardless of how the user's task system schedules tasks (out
	// of order, fewer threads than workers, or synchronously inside enqueueTaskFcn). The
	// loser of the race no-ops as workerIndex 0.
	mainClaimed atomicInt

	// padding to prevent false sharing
	padding3 [64]byte
}

// b3Solve is declared here and defined in solver.c (below).

// makeSoft is b3MakeSoft.
func makeSoft(hertz, zeta, h float32) softness {
	if hertz == 0.0 {
		return softness{
			biasRate:     0.0,
			massScale:    0.0,
			impulseScale: 0.0,
		}
	}

	omega := float32(2.0*Pi) * hertz
	a1 := float32(2.0*zeta) + float32(h*omega)
	a2 := float32(float32(h*omega) * a1)
	a3 := 1.0 / (1.0 + a2)

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// If z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// if w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// In all cases:
	// massScale + impulseScale == 1

	return softness{
		biasRate:     omega / a1,
		massScale:    a2 * a3,
		impulseScale: a3,
	}
}

// ---- solver.c ----

// these are useful for solver testing
const (
	solverIterations      = 1 // ITERATIONS
	solverRelaxIterations = 1 // RELAX_ITERATIONS
)

// pause is b3Pause.
// PORT: upstream issues a pause/yield instruction (x86 pause, ARM yield). Go has no intrinsic for
// either, so this is upstream's fallback branch: an empty function. Go's asynchronous preemption
// keeps the spin loops that call it from starving other goroutines.
func pause() {
}

// workerContext is b3WorkerContext.
type workerContext struct {
	context     *stepContext
	workerIndex int
	userTask    any
}

// integrateVelocitiesTask is b3IntegrateVelocitiesTask.
// Integrate velocities, apply damping, and gyroscopic torque
func integrateVelocitiesTask(block solverBlock, context *stepContext) {
	validate(int(block.startIndex)+int(block.count) <= len(context.world.solverSets[awakeSet].bodyStates),
		"block.startIndex + block.count <= context->world->solverSets.data[b3_awakeSet].bodyStates.count")

	states := context.states
	sims := context.sims

	gravity := context.world.gravity
	h := context.h

	for i := int(block.startIndex); i < int(block.startIndex)+int(block.count); i++ {
		sim := &sims[i]
		state := &states[i]

		v := state.linearVelocity
		w := state.angularVelocity

		// Damping math
		// Differential equation: dv/dt + c * v = 0
		// Solution: v(t) = v0 * exp(-c * t)
		// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		// v2 = exp(-c * dt) * v1
		// Pade approximation:
		// v2 = v1 * 1 / (1 + c * dt)
		linearDamping := 1.0 / (1.0 + float32(h*sim.linearDamping))
		angularDamping := 1.0 / (1.0 + float32(h*sim.angularDamping))

		// Gravity scale will be zero for kinematic bodies
		var gravityScale float32
		if sim.invMass > 0.0 {
			gravityScale = sim.gravityScale
		} else {
			gravityScale = 0.0
		}

		linearVelocityDelta := Blend2(float32(h*sim.invMass), sim.force, float32(h*gravityScale), gravity)
		v = MulAdd(linearVelocityDelta, linearDamping, v)

		angularVelocityDelta := MulSV(h, MulMV(sim.invInertiaWorld, sim.torque))
		w = MulAdd(angularVelocityDelta, angularDamping, w)

		// Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
		// I * (w2 - w1) + h * cross(w2, I * w2) = 0
		// This is all done in local coordinates where the Jacobian is easier to compute.
		// This improves the simulation of long skinny bodies.
		if sim.flags&enableGyroscopicTorque != 0 {
			// Get current rotation.
			q0 := sim.transform.Q
			q := MulQuat(state.deltaRotation, q0)

			// Compute local angular velocity
			omega1 := InvRotateVector(q, w)
			omega2 := omega1

			// Symmetric inertia tensor: 6 unique entries (column-major)
			i00 := sim.inertia.Cx.X
			i01 := sim.inertia.Cy.X
			i02 := sim.inertia.Cz.X
			i11 := sim.inertia.Cy.Y
			i12 := sim.inertia.Cz.Y
			i22 := sim.inertia.Cz.Z

			for gyroIteration := 0; gyroIteration < GyroscopicIterations; gyroIteration++ {
				w1 := omega2.X
				w2 := omega2.Y
				w3 := omega2.Z

				// Iw = I * omega2 (shared between residual and Jacobian)
				Iw1 := float32(i00*w1) + float32(i01*w2) + float32(i02*w3)
				Iw2 := float32(i01*w1) + float32(i11*w2) + float32(i12*w3)
				Iw3 := float32(i02*w1) + float32(i12*w2) + float32(i22*w3)

				// Residual: b = I * (omega2 - omega1) + h * cross(omega2, I * omega2)
				dw := Sub(omega2, omega1)
				b := Vec3{
					float32(i00*dw.X) + float32(i01*dw.Y) + float32(i02*dw.Z) + float32(h*(float32(w2*Iw3)-float32(w3*Iw2))),
					float32(i01*dw.X) + float32(i11*dw.Y) + float32(i12*dw.Z) + float32(h*(float32(w3*Iw1)-float32(w1*Iw3))),
					float32(i02*dw.X) + float32(i12*dw.Y) + float32(i22*dw.Z) + float32(h*(float32(w1*Iw2)-float32(w2*Iw1))),
				}

				// Jacobian J = I + h * (skew(omega2) * I - skew(I * omega2))
				// Jacobian derived by Erin Catto, Ph.D. Do not attempt to do this without a Ph.D.
				// Doubled inertia terms above fold into Iw, e.g. row 2 col 1: i00*w3 - i02*w1 - Iw3.
				J := Matrix3{
					Vec3{i00 + float32(h*(float32(w2*i02)-float32(w3*i01))), i01 + float32(h*(float32(w3*i00)-float32(w1*i02)-Iw3)),
						i02 + float32(h*(float32(w1*i01)-float32(w2*i00)+Iw2))},
					Vec3{i01 + float32(h*(float32(w2*i12)-float32(w3*i11)+Iw3)), i11 + float32(h*(float32(w3*i01)-float32(w1*i12))),
						i12 + float32(h*(float32(w1*i11)-float32(w2*i01)-Iw1))},
					Vec3{i02 + float32(h*(float32(w2*i22)-float32(w3*i12)-Iw2)), i12 + float32(h*(float32(w3*i02)-float32(w1*i22)+Iw1)),
						i22 + float32(h*(float32(w1*i12)-float32(w2*i02)))},
				}

				omega2 = Sub(omega2, Solve3(J, b))
			}

			w = RotateVector(q, omega2)
		}

		state.linearVelocity = v
		state.angularVelocity = w
	}
}

// integratePositionsTask is b3IntegratePositionsTask.
func integratePositionsTask(block solverBlock, context *stepContext) {
	validate(int(block.startIndex)+int(block.count) <= len(context.world.solverSets[awakeSet].bodyStates),
		"block.startIndex + block.count <= context->world->solverSets.data[b3_awakeSet].bodyStates.count")

	states := context.states
	h := context.h
	maxLinearSpeed := context.maxLinearVelocity
	maxAngularSpeed := MaxRotation * context.inv_dt
	maxLinearSpeedSquared := float32(maxLinearSpeed * maxLinearSpeed)
	maxAngularSpeedSquared := float32(maxAngularSpeed * maxAngularSpeed)

	for i := int(block.startIndex); i < int(block.startIndex)+int(block.count); i++ {
		state := &states[i]

		v := state.linearVelocity
		w := state.angularVelocity

		// Motion locks - these can be viewed as a constraint that come last
		if state.flags&lockLinearX != 0 {
			v.X = 0.0
		}
		if state.flags&lockLinearY != 0 {
			v.Y = 0.0
		}
		if state.flags&lockLinearZ != 0 {
			v.Z = 0.0
		}
		if state.flags&lockAngularX != 0 {
			w.X = 0.0
		}
		if state.flags&lockAngularY != 0 {
			w.Y = 0.0
		}
		if state.flags&lockAngularZ != 0 {
			w.Z = 0.0
		}

		// Clamp to max linear speed
		if Dot(v, v) > maxLinearSpeedSquared {
			ratio := maxLinearSpeed / Length(v)
			v = MulSV(ratio, v)
			state.flags |= isSpeedCapped
		}

		// Clamp to max angular speed
		if Dot(w, w) > maxAngularSpeedSquared && (state.flags&allowFastRotation) == 0 {
			ratio := maxAngularSpeed / Length(w)
			w = MulSV(ratio, w)
			state.flags |= isSpeedCapped
		}

		state.linearVelocity = v
		state.angularVelocity = w
		state.deltaPosition = MulAdd(state.deltaPosition, h, v)
		state.deltaRotation = integrateRotation(state.deltaRotation, MulSV(h, w))
	}
}

// prepareJointsTask is b3PrepareJointsTask.
func prepareJointsTask(block solverBlock, context *stepContext) {
	spans := context.jointPrepareSpans

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
		joints := spans[colorIndex].joints

		// Loop over color
		for ; index < colorEndIndex; index++ {
			assert(0 <= index-colorStart && index-colorStart < int(spans[colorIndex].count),
				"0 <= index - colorStart && index - colorStart < spans[colorIndex].count")
			joint := &joints[index-colorStart]
			prepareJoint(joint, context)
		}

		// Advance to next color
		colorIndex += 1
	}
}

// warmStartJointsTask is b3WarmStartJointsTask.
func warmStartJointsTask(block solverBlock, context *stepContext) {
	color := &context.graph.colors[block.colorIndex]
	joints := color.jointSims

	for i := int(block.startIndex); i < int(block.startIndex)+int(block.count); i++ {
		joint := &joints[i]
		warmStartJoint(joint, context)
	}
}

// solveJointsTask is b3SolveJointsTask.
func solveJointsTask(block solverBlock, context *stepContext, useBias bool, workerIndex int) {
	color := &context.graph.colors[block.colorIndex]
	joints := color.jointSims

	assert(0 <= block.startIndex && int(block.startIndex)+int(block.count) <= len(color.jointSims),
		"0 <= block.startIndex && block.startIndex + block.count <= color->jointSims.count")

	jointStateBitSet := &context.world.taskContexts[workerIndex].jointStateBitSet

	for i := int(block.startIndex); i < int(block.startIndex)+int(block.count); i++ {
		joint := &joints[i]
		solveJoint(joint, context, useBias)

		if useBias && (joint.forceThreshold < fltMax || joint.torqueThreshold < fltMax) &&
			getBit(jointStateBitSet, uint32(joint.jointId)) == false {
			force, torque := getJointReaction(context.world, joint, context.inv_h)

			// Check thresholds. A zero threshold means all awake joints get reported.
			if force >= joint.forceThreshold || torque >= joint.torqueThreshold {
				// Flag this joint for processing.
				setBit(jointStateBitSet, uint32(joint.jointId))
			}
		}
	}
}

// maxContinuousSensorHits is B2_MAX_CONTINUOUS_SENSOR_HITS.
const maxContinuousSensorHits = 8

// continuousContext is b3ContinuousContext.
type continuousContext struct {
	world                *world
	fastBodySim          *bodySim
	fastShape            *shape
	centroid1, centroid2 Vec3
	sweep                Sweep
	// World base for re-centering sweeps. Keeps TOI in float precision far from the origin.
	base            Pos
	fraction        float32
	sensorHits      [maxContinuousSensorHits]sensorHit
	sensorFractions [maxContinuousSensorHits]float32
	sensorCount     int

	distanceIterations int
	pushBackIterations int
	rootIterations     int
}

// continuousQueryCallback is b3ContinuousQueryCallback.
// This is called from b3DynamicTree_Query for continuous collision
func continuousQueryCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(int32(userData))
	continuousContext := context.(*continuousContext)

	fastShape := continuousContext.fastShape
	fastBodySim := continuousContext.fastBodySim

	assert(fastShape.sensorIndex == NullIndex, "fastShape->sensorIndex == B3_NULL_INDEX")

	// Skip same shape
	if shapeId == int(fastShape.id) {
		return true
	}

	world := continuousContext.world
	shape := arrayGet(world.shapes, shapeId)
	kinematic, retainedKinematic := world.speculativeCCD.kinematics[int32(shapeId)]
	if retainedKinematic {
		shape = &kinematic.shape
	}

	// Skip same body
	if shape.bodyId == fastShape.bodyId {
		return true
	}

	// Skip sensors unless both shapes want sensor events
	isSensor := shape.sensorIndex != NullIndex
	if isSensor && ((shape.flags&uint8(enableSensorEvents)) == 0 || (fastShape.flags&uint8(enableSensorEvents)) == 0) {
		return true
	}

	// Skip filtered shapes
	canCollide := shouldShapesCollide(fastShape.filter, shape.filter)
	if canCollide == false {
		return true
	}

	body := arrayGet(world.bodies, int(shape.bodyId))

	assert(body.typ == StaticBody || retainedKinematic || (fastBodySim.flags&isBullet) != 0,
		"static, retained kinematic, or bullet CCD target")

	// Dynamic bullet ordering remains native. Retained kinematics have an
	// immutable trajectory, including when their author selected bullet policy.
	if !retainedKinematic && body.flags&isBullet != 0 {
		return true
	}

	// Skip filtered bodies
	fastBody := arrayGet(world.bodies, int(fastBodySim.bodyId))
	canCollide = shouldBodiesCollide(world, fastBody, body)
	if canCollide == false {
		return true
	}

	// Custom user filtering
	if (shape.flags&uint8(enableCustomFiltering)) != 0 || (fastShape.flags&uint8(enableCustomFiltering)) != 0 {
		customFilterFcn := world.customFilterFcn
		if customFilterFcn != nil {
			idA := ShapeID{shape.id + 1, world.worldId, shape.generation}
			idB := ShapeID{fastShape.id + 1, world.worldId, fastShape.generation}
			canCollide = customFilterFcn(idA, idB, world.customFilterContext)
			if canCollide == false {
				return true
			}
		}
	}

	// todo does having a sweep on shapeA help with bullets?
	var sweepA Sweep
	if retainedKinematic {
		sweepA = kinematic.sweep
		sweepA.C1 = Sub(sweepA.C1, ToVec3(continuousContext.base))
		sweepA.C2 = Sub(sweepA.C2, ToVec3(continuousContext.base))
	} else {
		sweepA = makeRelativeSweep(getBodySim(world, body), continuousContext.base)
	}

	// Time of impact versus shape. Supports all shape types
	output := shapeTimeOfImpact(shape, fastShape, &sweepA, &continuousContext.sweep, continuousContext.fraction)
	if isSensor {
		// Only accept a sensor hit that is sooner than the current solid hit.
		if (output.State == TOIStateHit || output.State == TOIStateOverlapped) && output.Fraction <= continuousContext.fraction && continuousContext.sensorCount < maxContinuousSensorHits {
			index := continuousContext.sensorCount

			// The hit shape is a sensor
			sensorHit := sensorHit{
				sensorId:  shape.id,
				visitorId: fastShape.id,
			}

			continuousContext.sensorHits[index] = sensorHit
			continuousContext.sensorFractions[index] = output.Fraction
			continuousContext.sensorCount += 1
		}
	} else if 0.0 < output.Fraction && output.Fraction < continuousContext.fraction {
		didHit := true

		if didHit && ((shape.flags&uint8(enablePreSolveEvents)) != 0 || (fastShape.flags&uint8(enablePreSolveEvents)) != 0) {
			shapeIdA := ShapeID{shape.id + 1, world.worldId, shape.generation}
			shapeIdB := ShapeID{fastShape.id + 1, world.worldId, fastShape.generation}
			point := OffsetPos(continuousContext.base, output.Point)
			didHit = world.preSolveFcn(shapeIdA, shapeIdB, point, output.Normal, world.preSolveContext)
		}

		if didHit {
			continuousContext.fraction = output.Fraction
			continuousContext.distanceIterations = MaxInt(continuousContext.distanceIterations, int(output.DistanceIterations))
			continuousContext.pushBackIterations = MaxInt(continuousContext.pushBackIterations, int(output.PushBackIterations))
			continuousContext.rootIterations = MaxInt(continuousContext.rootIterations, int(output.RootIterations))
		}
	}

	// Continue query
	return true
}

// isShapeFast is b3IsShapeFast.
func isShapeFast(shape *shape, centroid1, centroid2 Vec3, rotationChord, safetyFactor float32) bool {
	var radius float32
	switch shape.typ {
	case SphereShape:
		radius = shape.sphere.Radius

	case CapsuleShape:
		radius = float32(0.5*Distance(shape.capsule.Center1, shape.capsule.Center2)) + shape.capsule.Radius

	case HullShape:
		farthestPoint := farthestPointOnAABB(shape.hull.AABB, shape.localCentroid)
		radius = Distance(farthestPoint, shape.localCentroid)

	default:
		return true
	}

	minExtent := computeShapeMinExtent(shape, shape.localCentroid)
	maxMotion := Distance(centroid1, centroid2) + float32(rotationChord*radius)
	return maxMotion > float32(safetyFactor*minExtent)
}

// solveContinuous is b3SolveContinuous.
// Continuous collision of dynamic versus static
func solveContinuous(world *world, bodySimIndex int, taskContext *taskContext, dt float32) {
	awakeSet := arrayGet(world.solverSets, awakeSet)
	fastBodySim := arrayGet(awakeSet.bodySims, bodySimIndex)
	validate(fastBodySim.flags&isFast != 0, "fastBodySim->flags & b3_isFast")

	// Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
	base := fastBodySim.center0

	sweep := makeRelativeSweep(fastBodySim, base)

	var xf1 Transform
	xf1.Q = sweep.Q1
	xf1.P = Sub(sweep.C1, RotateVector(sweep.Q1, sweep.LocalCenter))

	var xf2 Transform
	xf2.Q = sweep.Q2
	xf2.P = Sub(sweep.C2, RotateVector(sweep.Q2, sweep.LocalCenter))

	staticTree := &world.broadPhase.trees[StaticBody]
	kinematicTree := &world.broadPhase.trees[KinematicBody]
	dynamicTree := &world.broadPhase.trees[DynamicBody]
	fastBody := arrayGet(world.bodies, int(fastBodySim.bodyId))

	// PORT: the context is passed to the tree query as `any`, so a local would escape to the heap
	// (one allocation per fast body); it lives in the worker's task context instead.
	context := &taskContext.continuous
	*context = continuousContext{}
	context.world = world
	context.sweep = sweep
	context.base = base
	context.fastBodySim = fastBodySim
	context.fraction = 1.0

	isBullet := (fastBodySim.flags & isBullet) != 0
	safetyFactor := fastBody.safetyFactor
	rotationChord := 2.0 * Length(InvMulQuat(sweep.Q1, sweep.Q2).V)

	shapeId := int(fastBody.headShapeId)
	for shapeId != NullIndex {
		fastShape := arrayGet(world.shapes, shapeId)
		shapeId = int(fastShape.nextShapeId)

		context.fastShape = fastShape
		context.centroid1 = TransformPoint(xf1, fastShape.localCentroid)
		context.centroid2 = TransformPoint(xf2, fastShape.localCentroid)

		box1 := fastShape.aabb
		// xf2 is relative to the base, so translate the box back to world space, rounding outward
		box2 := OffsetAABB(computeShapeAABB(fastShape, xf2), base)

		// Store this to avoid double computation in the case there is no impact event
		fastShape.aabb = box2

		// No continuous collision for meshes
		if fastShape.typ == MeshShape || fastShape.typ == HeightShape {
			continue
		}

		// No continuous collision for sensors
		if fastShape.sensorIndex != NullIndex {
			continue
		}

		if isBullet == false {
			isShapeFast := isShapeFast(fastShape, context.centroid1, context.centroid2, rotationChord, safetyFactor)
			if isShapeFast == false {
				continue
			}
		}

		sweptBox := box1.Union(box2)
		if sweep.Q1 != sweep.Q2 {
			sweptBox = sweptBox.Union(continuousOffsetBounds(continuousShapeBounds(fastShape, &sweep, context.fraction), base))
		}
		staticTree.Query(sweptBox, DefaultMaskBits, false, continuousQueryCallback, context)
		if len(world.speculativeCCD.kinematics) > 0 {
			world.speculativeCCD.kinematicTree.Query(sweptBox, DefaultMaskBits, false, continuousQueryCallback, context)
		}

		if isBullet {
			if !world.speculativeCCD.enabled {
				kinematicTree.Query(sweptBox, DefaultMaskBits, false, continuousQueryCallback, context)
			}
			dynamicTree.Query(sweptBox, DefaultMaskBits, false, continuousQueryCallback, context)
		}
	}

	speculativeScalar := SpeculativeDistance()

	if context.fraction < 1.0 {
		// Handle time of impact event. The sweep is relative to the base, so re-add the base
		// to return the advanced pose to world space.
		q := NLerp(sweep.Q1, sweep.Q2, context.fraction)
		c := Lerp(sweep.C1, sweep.C2, context.fraction)
		origin := Sub(c, RotateVector(q, sweep.LocalCenter))

		// Advance body
		transform := WorldTransform{OffsetPos(base, origin), q}
		center := OffsetPos(base, c)
		fastBodySim.transform = transform
		fastBodySim.center = center
		fastBodySim.rotation0 = q
		fastBodySim.center0 = center
		fastBodySim.flags |= hadTimeOfImpact

		// Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
		fastBodyState := arrayGet(awakeSet.bodyStates, bodySimIndex)
		v := fastBodyState.linearVelocity
		timeLoss := (1.0 - context.fraction) * dt
		dv := MulSV(-timeLoss*fastBodySim.gravityScale, world.gravity)
		if fastBodyState.flags&lockLinearX != 0 {
			dv.X = 0.0
		}
		if fastBodyState.flags&lockLinearY != 0 {
			dv.Y = 0.0
		}
		if fastBodyState.flags&lockLinearZ != 0 {
			dv.Z = 0.0
		}
		fastBodyState.linearVelocity = Add(v, dv)

		// The move event was written before CCD, so correct it with the impact pose
		event := arrayGet(world.bodyMoveEvents, bodySimIndex)
		event.Transform = fastBodySim.transform

		// Prepare AABBs for broad-phase.
		// Even though a body is fast, it may not move much. So the AABB may not need enlargement.

		shapeId = int(fastBody.headShapeId)
		for shapeId != NullIndex {
			shape := arrayGet(world.shapes, shapeId)

			// Must recompute aabb at the interpolated transform
			aabb := computeFatShapeAABB(shape, transform, speculativeScalar)
			shape.aabb = aabb

			shapeFatAABB := &world.fatAABBs[shapeId]
			if shapeFatAABB.Contains(aabb) == false {
				*shapeFatAABB = aabb.Inflate(shape.aabbMargin)

				// Regular bodies mark the hierarchy as moved using atomic operations.
				// Bullets are handled separately at a later stage.
				if isBullet == false {
					world.broadPhase.markProxyMoved(int(shape.proxyKey), *shapeFatAABB)
				} else {
					fastBodySim.flags |= enlargeBulletBounds
				}
			}

			shapeId = int(shape.nextShapeId)
		}
	} else {
		// No time of impact event

		// Advance body
		fastBodySim.rotation0 = fastBodySim.transform.Q
		fastBodySim.center0 = fastBodySim.center

		// Prepare AABBs for broad-phase
		shapeId = int(fastBody.headShapeId)
		for shapeId != NullIndex {
			shape := arrayGet(world.shapes, shapeId)

			// shape->aabb is still valid from above

			shapeFatAABB := &world.fatAABBs[shapeId]
			if shapeFatAABB.Contains(shape.aabb) == false {
				*shapeFatAABB = shape.aabb.Inflate(shape.aabbMargin)

				if isBullet == false {
					world.broadPhase.markProxyMoved(int(shape.proxyKey), *shapeFatAABB)
				} else {
					fastBodySim.flags |= enlargeBulletBounds
				}
			}

			shapeId = int(shape.nextShapeId)
		}
	}

	// Push sensor hits on the the task context for serial processing.
	for i := 0; i < context.sensorCount; i++ {
		// Skip any sensor hits that occurred after a solid hit
		if context.sensorFractions[i] < context.fraction {
			arrayPush(&taskContext.sensorHits, context.sensorHits[i])
		}
	}

	taskContext.distanceIterations = int32(MaxInt(int(taskContext.distanceIterations), context.distanceIterations))
	taskContext.pushBackIterations = int32(MaxInt(int(taskContext.pushBackIterations), context.pushBackIterations))
	taskContext.rootIterations = int32(MaxInt(int(taskContext.rootIterations), context.rootIterations))
}

// finalizeBodiesTask is b3FinalizeBodiesTask.
// Implements b3ParallelForCallback
func finalizeBodiesTask(startIndex, endIndex, workerIndex int, context any) {
	stepContext := context.(*stepContext)
	world := stepContext.world
	bodies := world.bodies
	sims := stepContext.sims
	states := stepContext.states

	assert(endIndex <= len(world.bodyMoveEvents), "endIndex <= world->bodyMoveEvents.count")

	// PORT: upstream's local is enableSleep, which would shadow the b3_enableSleep body flag below.
	worldEnableSleep := world.enableSleep
	enableContinuous := world.enableContinuous
	timeStep := stepContext.dt
	invTimeStep := stepContext.inv_dt
	worldId := world.worldId

	// The body move event array has should already have the correct size
	moveEvents := world.bodyMoveEvents

	taskContext := &world.taskContexts[workerIndex]
	awakeIslandBitSet := &taskContext.awakeIslandBitSet

	speculativeScalar := SpeculativeDistance()

	for simIndex := startIndex; simIndex < endIndex; simIndex++ {
		state := &states[simIndex]
		sim := &sims[simIndex]

		v := state.linearVelocity
		w := state.angularVelocity
		localOmega := InvRotateVector(sim.transform.Q, w)
		localDeltaRotation := InvRotateVector(sim.transform.Q, state.deltaRotation.V)

		if IsValidVec3(v) == false || IsValidVec3(w) == false {
			name := findNameWithDefault(&world.names, bodies[sim.bodyId].nameId, "NULL")
			log("unstable: %s", name)
		}

		assert(IsValidVec3(v), "b3IsValidVec3( v )")
		assert(IsValidVec3(w), "b3IsValidVec3( w )")

		sim.center = OffsetPos(sim.center, state.deltaPosition)
		sim.transform.Q = NormalizeQuat(MulQuat(state.deltaRotation, sim.transform.Q))

		// Use the velocity of the farthest point on the body to account for rotation.
		velocityArc := modifiedCross(Abs(localOmega), sim.maxExtent)
		maxVelocity := Length(v) + Length(velocityArc)

		// Sleep needs to observe position correction as well as true velocity.
		// q = [sin(theta/2) * v, cos(theta/2)]
		// for small angles abs(theta) ~= 2 * length(sin(theta/2) * v)
		rotationArc := modifiedCross(Abs(localDeltaRotation), sim.maxExtent)
		maxDeltaPosition := Length(state.deltaPosition) + float32(2.0*Length(rotationArc))

		// Position correction is not as important for sleep as true velocity.
		positionSleepFactor := float32(0.5)
		sleepVelocity := MaxFloat(maxVelocity, float32(float32(positionSleepFactor*invTimeStep)*maxDeltaPosition))

		// reset state deltas
		state.deltaPosition = Vec3Zero
		state.deltaRotation = QuatIdentity

		sim.transform.P = OffsetPos(sim.center, Neg(RotateVector(sim.transform.Q, sim.localCenter)))

		// cache miss here, however I need the shape list below
		body := &bodies[sim.bodyId]
		body.bodyMoveIndex = int32(simIndex)
		body.sleepVelocity = sleepVelocity

		moveEvents[simIndex].UserData = body.userData
		moveEvents[simIndex].Transform = sim.transform
		moveEvents[simIndex].BodyId = BodyID{sim.bodyId + 1, worldId, body.generation}
		moveEvents[simIndex].FellAsleep = false

		// reset applied force and torque
		sim.force = Vec3Zero
		sim.torque = Vec3Zero

		// If you hit this then it means you deferred mass computation but never called b3Body_ApplyMassFromShapes
		// or b3Body_SetMassData.
		assert((body.flags&dirtyMass) == 0, "( body->flags & b3_dirtyMass ) == 0")

		// Clear the transient flags (fast, speed capped, had TOI). These flags are conditionally set
		// as part of the code below.
		body.flags &^= bodyTransientFlags
		sim.flags &^= (isFast | bodyTransientFlags)

		// The body state flag knows about speed capping (used for debug draw).
		body.flags |= (state.flags & isSpeedCapped)
		state.flags &^= bodyTransientFlags

		if worldEnableSleep == false || (body.flags&enableSleep) == 0 || sleepVelocity > body.sleepThreshold {
			// Body is not sleepy
			body.sleepTime = 0.0

			safetyFactor := body.safetyFactor
			maxMotion := MaxFloat(maxDeltaPosition, float32(maxVelocity*timeStep))
			if body.typ == DynamicBody && enableContinuous && maxMotion > float32(safetyFactor*sim.minExtent) {
				// This flag is used for debug draw and contact recycling.
				sim.flags |= isFast

				// Store in fast array for the continuous collision stage
				// This is deterministic because the order of TOI sweeps doesn't matter
				if sim.flags&isBullet != 0 {
					bulletIndex := atomicFetchAddInt(&stepContext.bulletBodyCount, 1)
					stepContext.bulletBodies[bulletIndex] = int32(simIndex)
				} else {
					solveContinuous(world, simIndex, taskContext, timeStep)
				}
			} else {
				// Body is safe to advance
				sim.center0 = sim.center
				sim.rotation0 = sim.transform.Q
			}
		} else {
			// Body is safe to advance and is falling asleep
			sim.center0 = sim.center
			sim.rotation0 = sim.transform.Q
			body.sleepTime += timeStep
		}

		// Update world space inverse inertia tensor.
		rotationMatrix := MakeMatrixFromQuat(sim.transform.Q)
		sim.invInertiaWorld = MulMM(MulMM(rotationMatrix, sim.invInertiaLocal), Transpose(rotationMatrix))

		// Any single body in an island can keep it awake
		island := arrayGet(world.islands, int(body.islandId))
		if body.sleepTime < TimeToSleep {
			// keep island awake
			islandIndex := island.localIndex
			setBit(awakeIslandBitSet, uint32(islandIndex))
		} else if island.constraintRemoveCount > 0 {
			// Body wants to sleep but its island needs splitting first. Track the sleepiest candidate.
			// Break sleep time ties using the island id to ensure determinism. The cross worker reduction
			// breaks ties the same way.
			if body.sleepTime > taskContext.splitSleepTime ||
				(body.sleepTime == taskContext.splitSleepTime && body.islandId > taskContext.splitIslandId) {
				// pick the sleepiest candidate
				taskContext.splitIslandId = body.islandId
				taskContext.splitSleepTime = body.sleepTime
			}
		}

		// Update shapes AABBs
		// For fast non-bullet bodies the AABB has already been updated in b3SolveContinuous
		// For fast bullet bodies the AABB will be updated at a later stage
		if (sim.flags & isFast) == 0 {
			transform := sim.transform
			shapeId := int(body.headShapeId)
			for shapeId != NullIndex {
				shape := arrayGet(world.shapes, shapeId)

				aabb := computeFatShapeAABB(shape, transform, speculativeScalar)
				shape.aabb = aabb

				shapeFatAABB := &world.fatAABBs[shapeId]
				if shapeFatAABB.Contains(aabb) == false {
					*shapeFatAABB = aabb.Inflate(shape.aabbMargin)

					// Mark the hierarchy as moved using atomic operations.
					world.broadPhase.markProxyMoved(int(shape.proxyKey), *shapeFatAABB)
				}

				shapeId = int(shape.nextShapeId)
			}
		}
	}
}

// blockDim is b3BlockDim.
type blockDim struct {
	// number of items per block (except last block)
	size int

	// total number of blocks
	count int
}

// computeBlockCount is b3ComputeBlockCount.
// A block is a range of tasks, a start index and count as a sub-array. Each worker receives at
// most M blocks of work. The workers may receive less blocks if there is not sufficient work.
// Each block of work has a minimum number of elements (block size). This in turn may limit the
// number of blocks. If there are many elements then the block size is increased so there are
// still at most M blocks of work per worker. M is a tunable number that has two goals:
// 1. keep M small to reduce overhead
// 2. keep M large enough for other workers to be able to steal work
// The block size is a power of two to make math efficient.
func computeBlockCount(itemCount, minSize, maxBlockCount int) blockDim {
	dim := blockDim{}
	if itemCount == 0 {
		return dim
	}

	if itemCount <= minSize*maxBlockCount {
		dim.size = minSize
	} else {
		dim.size = (itemCount + maxBlockCount - 1) / maxBlockCount
	}

	dim.count = (itemCount + dim.size - 1) / dim.size

	assert(dim.count >= 1, "dim.count >= 1")
	assert(dim.size*dim.count >= itemCount, "dim.size * dim.count >= itemCount")

	return dim
}

// initBlocks is b3InitBlocks.
// Initialize solver blocks for a contiguous range of items. Computes block size internally
// from the same parameters used by b3ComputeBlockCount. The atomic claim counter is zeroed
// so workers can CAS (0, 1) on the first stage that owns these blocks.
// PORT: b3SyncBlock* blocks is a slice starting at the first block.
func initBlocks(blocks []syncBlock, dim blockDim, itemCount int, blockType uint8, colorIndex uint8) {
	if dim.count == 0 {
		return
	}

	assert(itemCount >= dim.count, "itemCount >= dim.count")

	// Compute the number of elements per block
	blockSize := dim.size

	// Simulation too big
	assert(blockSize <= math.MaxUint16, "blockSize <= UINT16_MAX")

	for i := 0; i < dim.count; i++ {
		blocks[i].block.startIndex = int32(i * blockSize)
		blocks[i].block.count = uint16(blockSize)
		blocks[i].block.blockType = blockType
		blocks[i].block.colorIndex = colorIndex
		atomicStoreInt(&blocks[i].syncIndex, 0)
	}

	// The last block may not be full
	blocks[dim.count-1].block.count = uint16(itemCount - (dim.count-1)*blockSize)

	validate(int(blocks[dim.count-1].block.count) <= blockSize, "blocks[dim.count - 1].block.count <= blockSize")
	validate((dim.count-1)*dim.size+int(blocks[dim.count-1].block.count) == itemCount,
		"( dim.count - 1 ) * dim.size + blocks[dim.count - 1].block.count == itemCount")
}

// initStage is b3InitStage.
// PORT: the stage pointer is an index into stages; the return value is the next index (stage + 1).
func initStage(stages []solverStage, stage int, typ solverStageType, blocks []syncBlock, blockCount int,
	colorIndex uint8) int {
	s := &stages[stage]
	s.typ = typ
	s.blocks = blocks
	s.blockCount = int32(blockCount)
	s.colorIndex = colorIndex
	atomicStoreInt(&s.completionCount, 0)
	return stage + 1
}

// initColorStages is b3InitColorStages.
// Initialize one stage per color for each iteration. Used for warm start, solve, relax, and restitution.
// All iterations of a given color share the same b3SyncBlock array so the per-block syncIndex
// grows monotonically across stages within that color.
// PORT: the stage pointer is an index into stages (see initStage); the arrays are slices.
func initColorStages(stages []solverStage, stage int, typ solverStageType, iterations, activeColorCount int,
	colorBlocks [][]syncBlock, colorBlockCounts []int, activeColorIndices []int) int {
	for j := 0; j < iterations; j++ {
		for i := 0; i < activeColorCount; i++ {
			stage = initStage(stages, stage, typ, colorBlocks[i], colorBlockCounts[i], uint8(activeColorIndices[i]))
		}
	}
	return stage
}

// executeBlock is b3ExecuteBlock.
func executeBlock(stage *solverStage, context *stepContext, block solverBlock, workerIndex int) {
	stageType := stage.typ
	blockType := block.blockType

	switch stageType {
	case stagePrepareJoints:
		prepareJointsTask(block, context)

	case stagePrepareWideContacts:
		prepareContactsConvex(block, context)

	case stagePrepareContacts:
		prepareContactsMeshWide(block, context)

	case stageIntegrateVelocities:
		integrateVelocitiesTask(block, context)

	case stageWarmStart:
		if blockType == graphJointBlock {
			warmStartJointsTask(block, context)
		} else if blockType == graphWideContactBlock {
			warmStartContactsConvex(block, context)
		} else {
			warmStartContactsMeshWide(block, context)
		}

	case stageSolve:
		if blockType == graphJointBlock {
			useBias := true
			solveJointsTask(block, context, useBias, workerIndex)
		} else if blockType == graphWideContactBlock {
			pushContactsConvex(block, context)
		} else {
			pushContactsMeshWide(block, context)
		}

	case stageIntegratePositions:
		integratePositionsTask(block, context)

	case stageRelax:
		if blockType == graphJointBlock {
			useBias := false
			solveJointsTask(block, context, useBias, workerIndex)
		} else if blockType == graphWideContactBlock {
			solveContactsConvex(block, context)
		} else {
			solveContactsMeshWide(block, context)
		}

	case stageRestitution:
		if blockType == graphWideContactBlock {
			applyRestitutionConvex(block, context)
		} else if blockType == graphContactBlock {
			applyRestitutionMeshWide(block, context)
		}

	case stageStoreWideImpulses:
		storeImpulsesConvex(block, context, workerIndex)

	case stageStoreImpulses:
		storeImpulsesMeshWide(block, context, workerIndex)
	}
}

// getWorkerStartIndex is GetWorkerStartIndex.
// This staggers the worker start indices so they avoid touching the same solver blocks
func getWorkerStartIndex(workerIndex, blockCount, workerCount int) int {
	if blockCount <= workerCount {
		if workerIndex < blockCount {
			return workerIndex
		}
		return NullIndex
	}

	blocksPerWorker := blockCount / workerCount
	remainder := blockCount - blocksPerWorker*workerCount
	return blocksPerWorker*workerIndex + MinInt(remainder, workerIndex)
}

// executeStage is b3ExecuteStage.
// Execute a stage, which is an array of solver blocks, each controlled with an atomic sync index.
// Each worker starts at its home index and sweeps the ring, CAS-claiming any unclaimed blocks.
func executeStage(stage *solverStage, context *stepContext, previousSyncIndex, syncIndex, workerIndex int) {
	completedCount := 0
	blocks := stage.blocks
	blockCount := int(stage.blockCount)

	startIndex := getWorkerStartIndex(workerIndex, blockCount, int(context.workerCount))
	if startIndex == NullIndex {
		return
	}

	assert(0 <= startIndex && startIndex < blockCount, "0 <= startIndex && startIndex < blockCount")

	blockIndex := startIndex
	for i := 0; i < blockCount; i++ {
		if atomicCompareExchangeInt(&blocks[blockIndex].syncIndex, int32(previousSyncIndex), int32(syncIndex)) {
			assert(completedCount < blockCount, "completedCount < blockCount")

			// Pass the descriptor by value -- the wrapping b3SyncBlock holds the atomic
			// syncIndex but we only copy .block, so the struct copy never aliases the CAS target.
			executeBlock(stage, context, blocks[blockIndex].block, workerIndex)
			completedCount += 1
		}

		blockIndex += 1
		if blockIndex >= blockCount {
			blockIndex = 0
		}
	}

	atomicFetchAddInt(&stage.completionCount, int32(completedCount))
}

// executeMainStage is b3ExecuteMainStage.
// Execute a stage on worker 0 (main thread).
func executeMainStage(stage *solverStage, context *stepContext, syncBits uint32) {
	blockCount := int(stage.blockCount)
	if blockCount == 0 {
		return
	}

	const workerIndex = 0

	if blockCount == 1 {
		executeBlock(stage, context, stage.blocks[0].block, workerIndex)
	} else {
		atomicStoreU32(&context.atomicSyncBits, syncBits)

		syncIndex := int((syncBits >> 16) & 0xFFFF)
		assert(syncIndex > 0, "syncIndex > 0")
		previousSyncIndex := syncIndex - 1

		executeStage(stage, context, previousSyncIndex, syncIndex, workerIndex)

		// Spin waiting for thieves to finish
		for int(atomicLoadInt(&stage.completionCount)) != blockCount {
			pause()
		}

		atomicStoreInt(&stage.completionCount, 0)
	}
}

// solverTask is b3SolverTask.
// Parallel solver task
func solverTask(taskContext any) {
	workerContext := taskContext.(*workerContext)
	workerIndex := workerContext.workerIndex
	context := workerContext.context
	solverIterations, solverRelaxIterations := context.motionVelocityIterations, context.motionRelaxIterations
	activeColorCount := int(context.activeColorCount)
	stages := context.stages
	profile := &context.world.profile

	if workerIndex == 0 {
		// The orchestrator slot is a race. The calling thread of b3World_Step also enters here
		// as worker 0, so progress is guaranteed even if the user's task system schedules tasks
		// out of order, has fewer threads than workerCount, or runs the task synchronously
		// inside enqueueTaskFcn. Whoever wins the CAS becomes the orchestrator; the loser
		// returns and lets the spinner-only path handle workers >0.
		if atomicCompareExchangeInt(&context.mainClaimed, 0, 1) == false {
			return
		}

		// Main thread synchronizes the workers and does work itself.
		//
		// This needs to be a task for the main thread because the user's task system may execute
		// the tasks serially and this is the first task. This single task is able to fully
		// complete all work even if all other workers are blocked.

		// Stages are re-used by loops so that I don't need more stages for large substep counts.
		// The sync indices grow monotonically for the body/graph/constraint groupings because they share solver blocks.
		// The stage index and sync indices are combined in to sync bits for atomic synchronization.
		// The workers need to compute the previous sync index for a given stage so that CAS works correctly. This
		// setup makes this easy to do.

		/*
			b3_stagePrepareJoints,
			b3_stagePrepareContacts,
			b3_stageIntegrateVelocities,
			b3_stageWarmStart,
			b3_stageSolve,
			b3_stageIntegratePositions,
			b3_stageRelax,
			b3_stageStoreImpulses
		*/

		ticks := GetTicks()

		bodySyncIndex := uint32(1)
		stageIndex := 0

		// Prepare joint constraints
		jointSyncIndex := uint32(1)
		syncBits := (jointSyncIndex << 16) | uint32(stageIndex)
		assert(stages[stageIndex].typ == stagePrepareJoints, "stages[stageIndex].type == b3_stagePrepareJoints")
		executeMainStage(&stages[stageIndex], context, syncBits)
		stageIndex += 1
		jointSyncIndex += 1

		// Prepare convex contact constraints
		convexSyncIndex := uint32(1)
		syncBits = (convexSyncIndex << 16) | uint32(stageIndex)
		assert(stages[stageIndex].typ == stagePrepareWideContacts, "stages[stageIndex].type == b3_stagePrepareWideContacts")
		executeMainStage(&stages[stageIndex], context, syncBits)
		stageIndex += 1
		convexSyncIndex += 1

		// Prepare mesh contact constraints
		meshSyncIndex := uint32(1)
		syncBits = (meshSyncIndex << 16) | uint32(stageIndex)
		assert(stages[stageIndex].typ == stagePrepareContacts, "stages[stageIndex].type == b3_stagePrepareContacts")
		executeMainStage(&stages[stageIndex], context, syncBits)
		stageIndex += 1
		meshSyncIndex += 1

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		prepareJointsOverflow(context)
		prepareContactsOverflow(context)

		profile.PrepareConstraints += GetMillisecondsAndReset(&ticks)

		graphSyncIndex := uint32(1)
		subStepCount := int(context.subStepCount)
		for subStepIndex := 0; subStepIndex < subStepCount; subStepIndex++ {
			// stageIndex restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			iterationStageIndex := stageIndex

			// Integrate velocities
			syncBits = (bodySyncIndex << 16) | uint32(iterationStageIndex)
			assert(stages[iterationStageIndex].typ == stageIntegrateVelocities,
				"stages[iterationStageIndex].type == b3_stageIntegrateVelocities")
			executeMainStage(&stages[iterationStageIndex], context, syncBits)
			iterationStageIndex += 1
			bodySyncIndex += 1

			profile.IntegrateVelocities += GetMillisecondsAndReset(&ticks)

			// Warm start constraints
			warmStartJointsOverflow(context)
			warmStartContactsOverflow(context)

			for colorIndex := 0; colorIndex < activeColorCount; colorIndex++ {
				syncBits = (graphSyncIndex << 16) | uint32(iterationStageIndex)
				assert(stages[iterationStageIndex].typ == stageWarmStart, "stages[iterationStageIndex].type == b3_stageWarmStart")
				executeMainStage(&stages[iterationStageIndex], context, syncBits)
				iterationStageIndex += 1
			}
			graphSyncIndex += 1

			profile.WarmStart += GetMillisecondsAndReset(&ticks)

			// Solve constraints
			useBias := true
			for j := 0; j < solverIterations; j++ {
				// Overflow constraints have lower priority. Typically these are dynamic-vs-dynamic.
				solveJointsOverflow(context, useBias)
				pushContactsOverflow(context)

				for colorIndex := 0; colorIndex < activeColorCount; colorIndex++ {
					syncBits = (graphSyncIndex << 16) | uint32(iterationStageIndex)
					assert(stages[iterationStageIndex].typ == stageSolve, "stages[iterationStageIndex].type == b3_stageSolve")
					executeMainStage(&stages[iterationStageIndex], context, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.SolveImpulses += GetMillisecondsAndReset(&ticks)

			// Integrate positions
			assert(stages[iterationStageIndex].typ == stageIntegratePositions,
				"stages[iterationStageIndex].type == b3_stageIntegratePositions")
			syncBits = (bodySyncIndex << 16) | uint32(iterationStageIndex)
			executeMainStage(&stages[iterationStageIndex], context, syncBits)
			iterationStageIndex += 1
			bodySyncIndex += 1

			profile.IntegratePositions += GetMillisecondsAndReset(&ticks)

			// Relax constraints
			useBias = false
			for j := 0; j < solverRelaxIterations; j++ {
				// Repeated constraint convergence at one time boundary is not
				// another physical impulse. Accumulate the final relaxed total.
				context.skipContactImpulseAccumulation = j+1 < solverRelaxIterations
				solveJointsOverflow(context, useBias)
				solveContactsOverflow(context)

				for colorIndex := 0; colorIndex < activeColorCount; colorIndex++ {
					syncBits = (graphSyncIndex << 16) | uint32(iterationStageIndex)
					assert(stages[iterationStageIndex].typ == stageRelax, "stages[iterationStageIndex].type == b3_stageRelax")
					executeMainStage(&stages[iterationStageIndex], context, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.RelaxImpulses += GetMillisecondsAndReset(&ticks)
		}

		// Advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + solverIterations*activeColorCount + 1 + solverRelaxIterations*activeColorCount

		restitutionIterations := int(context.world.restitutionIterations)
		if restitutionIterations > 0 && atomicLoadInt(&context.anyRestitution) != 0 {
			for j := 0; j < restitutionIterations; j++ {
				applyRestitutionOverflow(context)

				iterationStageIndex := stageIndex
				for colorIndex := 0; colorIndex < activeColorCount; colorIndex++ {
					syncBits = (graphSyncIndex << 16) | uint32(iterationStageIndex)
					assert(stages[iterationStageIndex].typ == stageRestitution,
						"stages[iterationStageIndex].type == b3_stageRestitution")
					executeMainStage(&stages[iterationStageIndex], context, syncBits)
					iterationStageIndex += 1
				}
				graphSyncIndex += 1
			}

			profile.Restitution += GetMillisecondsAndReset(&ticks)
		}

		stageIndex += activeColorCount

		// Store impulses
		storeImpulsesOverflow(context)

		syncBits = (convexSyncIndex << 16) | uint32(stageIndex)
		assert(stages[stageIndex].typ == stageStoreWideImpulses, "stages[stageIndex].type == b3_stageStoreWideImpulses")
		executeMainStage(&stages[stageIndex], context, syncBits)
		stageIndex += 1

		syncBits = (meshSyncIndex << 16) | uint32(stageIndex)
		assert(stages[stageIndex].typ == stageStoreImpulses, "stages[stageIndex].type == b3_stageStoreImpulses")
		executeMainStage(&stages[stageIndex], context, syncBits)
		stageIndex += 1

		profile.StoreImpulses += GetMillisecondsAndReset(&ticks)

		// Signal workers to finish
		atomicStoreU32(&context.atomicSyncBits, math.MaxUint32)

		assert(stageIndex == int(context.stageCount), "stageIndex == context->stageCount")
		return
	}

	// Worker spins and waits for work
	lastSyncBits := uint32(0)
	// uint64_t maxSpinTime = 10;
	for {
		// Spin until main thread bumps changes the sync bits. This can waste significant time overall, but it is necessary for
		// parallel simulation with graph coloring.
		// todo improve this spinner
		var syncBits uint32
		spinCount := 0
		for {
			syncBits = atomicLoadU32(&context.atomicSyncBits)
			if syncBits != lastSyncBits {
				break
			}

			if spinCount > 5 {
				Yield()
				spinCount = 0
			} else {
				// Using the cycle counter helps to account for variation in mm_pause timing across different
				// CPUs. However, this is X64 only.
				// uint64_t prev = __rdtsc();
				// do
				//{
				//	b3Pause();
				//}
				// while ((__rdtsc() - prev) < maxSpinTime);
				// maxSpinTime += 10;
				pause()
				pause()
				spinCount += 1
			}
		}

		if syncBits == math.MaxUint32 {
			// sentinel hit
			break
		}

		stageIndex := int(syncBits & 0xFFFF)
		assert(stageIndex < int(context.stageCount), "stageIndex < context->stageCount")

		syncIndex := int((syncBits >> 16) & 0xFFFF)
		assert(syncIndex > 0, "syncIndex > 0")

		previousSyncIndex := syncIndex - 1

		stage := &stages[stageIndex]
		executeStage(stage, context, previousSyncIndex, syncIndex, workerIndex)

		lastSyncBits = syncBits
	}
}

// bulletBodyTask is b3BulletBodyTask.
func bulletBodyTask(startIndex, endIndex, workerIndex int, context any) {
	stepContext := context.(*stepContext)
	taskContext := arrayGet(stepContext.world.taskContexts, workerIndex)

	assert(startIndex <= endIndex, "startIndex <= endIndex")

	for i := startIndex; i < endIndex; i++ {
		simIndex := int(stepContext.bulletBodies[i])
		solveContinuous(stepContext.world, simIndex, taskContext, stepContext.dt)
	}
}

// solverTaskNames holds the task names b3Solve formats with snprintf( "solve[%d]" ).
// PORT: precomputed so that enqueueing the solver tasks does not allocate.
var solverTaskNames = func() (names [MaxWorkers]string) {
	for i := range names {
		names[i] = fmt.Sprintf("solve[%d]", i)
	}
	return names
}()

// solve is b3Solve.
// Solve with graph coloring
func solve(world *world, stepContext *stepContext) {
	solverIterations, solverRelaxIterations := motionIterations(world)
	stepContext.motionVelocityIterations, stepContext.motionRelaxIterations = solverIterations, solverRelaxIterations
	simdShift := 2
	if world.simdWidth == 8 {
		simdShift = 3
	}

	// Only count steps that advance the simulation
	world.stepIndex += 1

	// PORT: upstream's local is awakeSet, which would shadow the b3_awakeSet constant the event
	// asserts below use.
	awakeSolverSet := arrayGet(world.solverSets, awakeSet)
	awakeBodyCount := len(awakeSolverSet.bodySims)
	if awakeBodyCount == 0 {
		return
	}

	// Solve constraints using graph coloring
	{
		setupTicks := GetTicks()

		// Prepare buffers for continuous collision (fast bodies)
		atomicStoreInt(&stepContext.bulletBodyCount, 0)
		stepContext.bulletBodies = stackAlloc[int32](&world.stack, awakeBodyCount, "bullet bodies")

		graph := &world.constraintGraph
		colors := &graph.colors

		stepContext.sims = awakeSolverSet.bodySims
		stepContext.states = awakeSolverSet.bodyStates

		// count contacts, joints, and colors
		activeColorCount := 0
		for i := 0; i < GraphColorCount-1; i++ {
			perColorContactCount := len(colors[i].convexContacts) + len(colors[i].contacts)
			perColorJointCount := len(colors[i].jointSims)
			occupancyCount := perColorContactCount + perColorJointCount
			if occupancyCount > 0 {
				activeColorCount += 1
			}
		}

		// prepare for move events
		arrayResize(&world.bodyMoveEvents, awakeBodyCount)

		workerCount := int(world.workerCount)

		// Target 4 blocks per worker to allow work stealing
		maxBlockCount := 4 * workerCount

		// Body blocks are for parallel iteration over bodies directly (integration, update transforms)
		minBodiesPerBlock := 32
		bodyDim := computeBlockCount(awakeBodyCount, minBodiesPerBlock, maxBlockCount)

		const minContactsPerBlock = 4
		const minJointsPerBlock = 4

		// Configure blocks for tasks parallel-for each active graph color
		// The blocks are a mix of convex contact, mesh contact, and joint blocks
		var activeColorIndices [GraphColorCount]int
		var colorWideContactCounts [GraphColorCount]int
		var colorMeshContactCounts [GraphColorCount]int
		var colorMeshGroupCounts [GraphColorCount]int
		var colorJointCounts [GraphColorCount]int
		var graphWideContactDims [GraphColorCount]blockDim
		var graphMeshContactDims [GraphColorCount]blockDim
		var graphJointDims [GraphColorCount]blockDim
		graphBlockCount := 0

		// c is the active color index
		wideContactCount := 0
		meshContactCount := 0
		meshGroupCount := 0
		jointCount := 0
		c := 0
		for i := 0; i < GraphColorCount-1; i++ {
			color := &colors[i]
			colorConvexContactCount := len(color.convexContacts)
			colorMeshContactCount := len(color.contacts)
			colorJointCount := len(color.jointSims)

			if colorConvexContactCount+colorMeshContactCount+colorJointCount == 0 {
				continue
			}

			activeColorIndices[c] = i

			// Ceiling for wide constraint count
			colorWideConstraintCount := 0
			if colorConvexContactCount > 0 {
				colorWideConstraintCount = ((colorConvexContactCount - 1) >> simdShift) + 1
			}
			wideContactCount += colorWideConstraintCount
			colorWideContactCounts[c] = colorWideConstraintCount

			colorMeshContactCounts[c] = colorMeshContactCount
			meshContactCount += colorMeshContactCount

			colorMeshGroupCount := 0
			if colorMeshContactCount > 0 {
				colorMeshGroupCount = ((colorMeshContactCount - 1) >> simdShift) + 1
			}
			colorMeshGroupCounts[c] = colorMeshGroupCount
			meshGroupCount += colorMeshGroupCount

			colorJointCounts[c] = colorJointCount
			jointCount += colorJointCount

			// Solver block dimensions
			graphWideContactDims[c] = computeBlockCount(colorWideConstraintCount, minContactsPerBlock, maxBlockCount)
			graphMeshContactDims[c] = computeBlockCount(colorMeshGroupCount, minContactsPerBlock, maxBlockCount)
			graphJointDims[c] = computeBlockCount(colorJointCount, minJointsPerBlock, maxBlockCount)
			graphBlockCount += graphWideContactDims[c].count + graphMeshContactDims[c].count + graphJointDims[c].count

			c += 1
		}
		activeColorCount = c

		// Prepare and store run as one flat parallel-for over the entire wide constraint range,
		// partitioned into uniformly sized blocks. Color info is consulted inside the task via
		// a small span array, so blocks do not need to honor color boundaries here.
		convexPrepareDim := computeBlockCount(wideContactCount, minContactsPerBlock, maxBlockCount)
		meshPrepareDim := computeBlockCount(meshGroupCount, minContactsPerBlock, maxBlockCount)
		jointPrepareDim := computeBlockCount(jointCount, minJointsPerBlock, maxBlockCount)

		// PORT: upstream allocates one byte buffer of wideContactCount * b3GetWideContactConstraintByteCount
		// (a b3ConvexConstraintWide each: the constraint followed by its single manifold). The typed
		// buffer is the pair of slices (see wideBuffer), one manifold per constraint. At width 8 it
		// is the wideConstraintBuffer8; the other buffer stays empty.
		var wideConstraints wideConstraintBuffer
		var wideConstraints8 wideConstraintBuffer8
		if world.simdWidth == 8 {
			wideConstraints8 = allocWideBuffer[contactConstraintWide8, contactManifoldWide8](&world.stack, wideContactCount,
				wideContactCount, "wide contacts", "wide contact manifolds")
		} else {
			wideConstraints = allocWideBuffer[contactConstraintWide, contactManifoldWide](&world.stack, wideContactCount,
				wideContactCount, "wide contacts", "wide contact manifolds")
		}

		meshLaneOrder := stackAlloc[int32](&world.stack, meshContactCount, "mesh lane order")
		meshManifoldStarts := stackAlloc[int32](&world.stack, meshGroupCount+1, "mesh manifold starts")

		// Sort mesh contacts by manifold count so that contacts with more manifolds are solved
		// together keeping the SIMD lanes fuller and less ragged.
		meshSlotCount := 0
		{
			maxManifoldCount := 0
			for i := 0; i < activeColorCount; i++ {
				color := &colors[activeColorIndices[i]]
				specs := color.contacts
				colorContactCount := colorMeshContactCounts[i]
				for j := 0; j < colorContactCount; j++ {
					maxManifoldCount = MaxInt(maxManifoldCount, int(specs[j].manifoldCount))
				}
			}

			bucketStarts := stackAlloc[int](&world.stack, maxManifoldCount+1, "mesh buckets")

			orderBase := 0
			groupBase := 0
			for i := 0; i < activeColorCount; i++ {
				color := &colors[activeColorIndices[i]]
				colorContactCount := colorMeshContactCounts[i]
				if colorContactCount == 0 {
					continue
				}

				specs := color.contacts
				order := meshLaneOrder[orderBase:]

				for k := 0; k <= maxManifoldCount; k++ {
					bucketStarts[k] = 0
				}

				for j := 0; j < colorContactCount; j++ {
					bucketStarts[specs[j].manifoldCount] += 1
				}

				start := 0
				for k := maxManifoldCount; k >= 0; k-- {
					count := bucketStarts[k]
					bucketStarts[k] = start
					start += count
				}

				for j := 0; j < colorContactCount; j++ {
					order[bucketStarts[specs[j].manifoldCount]] = int32(j)
					bucketStarts[specs[j].manifoldCount]++
				}

				colorMeshGroupCount := colorMeshGroupCounts[i]
				for g := 0; g < colorMeshGroupCount; g++ {
					slotCount := int(specs[order[g<<simdShift]].manifoldCount)
					meshManifoldStarts[groupBase+g] = int32(meshSlotCount)
					meshSlotCount += slotCount
				}

				orderBase += colorContactCount
				groupBase += colorMeshGroupCount
			}

			assert(orderBase == meshContactCount, "orderBase == meshContactCount")
			assert(groupBase == meshGroupCount, "groupBase == meshGroupCount")
			meshManifoldStarts[meshGroupCount] = int32(meshSlotCount)

			stackFree(&world.stack, bucketStarts)
		}

		// PORT: upstream allocates one byte buffer of meshGroupCount * b3GetWideMeshConstraintByteCount +
		// meshSlotCount * b3GetWideMeshManifoldByteCount; the typed buffer holds meshGroupCount
		// constraints and meshSlotCount manifolds (see wideBuffer), wideConstraintBuffer8 at width 8.
		var wideMeshConstraints wideConstraintBuffer
		var wideMeshConstraints8 wideConstraintBuffer8
		if world.simdWidth == 8 {
			wideMeshConstraints8 = allocWideBuffer[contactConstraintWide8, contactManifoldWide8](&world.stack, meshGroupCount,
				meshSlotCount, "wide mesh constraints", "wide mesh manifolds")
		} else {
			wideMeshConstraints = allocWideBuffer[contactConstraintWide, contactManifoldWide](&world.stack, meshGroupCount,
				meshSlotCount, "wide mesh constraints", "wide mesh manifolds")
		}

		overflow := &colors[overflowIndex]
		overflowCount := len(overflow.contacts)
		overflowManifoldCount := 0
		for i := 0; i < overflowCount; i++ {
			overflow.contacts[i].manifoldStart = int32(overflowManifoldCount)
			overflowManifoldCount += int(overflow.contacts[i].manifoldCount)
		}

		overflow.contactConstraints = stackAlloc[contactConstraint](&world.stack, overflowCount, "overflow contacts")
		overflow.manifoldConstraints = stackAlloc[manifoldConstraint](&world.stack, overflowManifoldCount, "overflow manifolds")

		// Build the span table for the flat prepare/store parallel-for while I slice the
		// wide constraint buffer across colors. One entry per active color plus a sentinel
		// at wideContactCount.
		// PORT: these C stack arrays are referenced from the step context, so they live in the
		// world (stepScratch) and are cleared here; locals would escape to the heap every step.
		widePrepareSpans := &world.stepScratch.widePrepareSpans
		meshPrepareSpans := &world.stepScratch.meshPrepareSpans
		jointPrepareSpans := &world.stepScratch.jointPrepareSpans
		clear(widePrepareSpans[:])
		clear(meshPrepareSpans[:])
		clear(jointPrepareSpans[:])

		// Distribute transient constraints to each graph color and prepare spans
		{
			wideBase := 0
			contactBase := 0
			meshGroupBase := 0
			jointBase := 0
			for i := 0; i < activeColorCount; i++ {
				j := activeColorIndices[i]
				color := &colors[j]

				colorConvexContactCount := len(color.convexContacts)
				widePrepareSpans[i].start = int32(wideBase)
				widePrepareSpans[i].count = int32(colorConvexContactCount)
				widePrepareSpans[i].contacts = color.convexContacts

				if colorConvexContactCount == 0 {
					color.wideConstraints = wideConstraintBuffer{}
					color.wideConstraints8 = wideConstraintBuffer8{}
					color.wideConstraintCount = 0
				} else {
					colorContactCountW := ((colorConvexContactCount - 1) >> simdShift) + 1

					// PORT: the byte offset wideBase * wideContactByteCount is constraint and manifold
					// index wideBase; the sub-slices end at the color's last wide constraint.
					if world.simdWidth == 8 {
						color.wideConstraints8 = sliceWideBuffer(wideConstraints8, wideBase, colorContactCountW, wideBase,
							wideBase+colorContactCountW)
					} else {
						color.wideConstraints = sliceWideBuffer(wideConstraints, wideBase, colorContactCountW, wideBase,
							wideBase+colorContactCountW)
					}

					color.wideConstraintCount = int32(colorContactCountW)
					wideBase += colorContactCountW
				}

				colorContactCount := len(color.contacts)
				meshPrepareSpans[i].start = int32(meshGroupBase)
				meshPrepareSpans[i].count = int32(colorContactCount)
				meshPrepareSpans[i].contacts = color.contacts
				meshPrepareSpans[i].order = meshLaneOrder[contactBase:]

				if colorContactCount == 0 {
					color.wideMeshConstraints = wideConstraintBuffer{}
					color.wideMeshConstraints8 = wideConstraintBuffer8{}
					color.wideMeshManifoldStarts = nil
					color.wideMeshConstraintCount = 0
				} else {
					colorMeshGroupCount := colorMeshGroupCounts[i]

					// PORT: the byte offset meshGroupBase * wideMeshConstraintByteCount +
					// meshManifoldStarts[meshGroupBase] * wideMeshManifoldByteCount is constraint index
					// meshGroupBase and manifold index meshManifoldStarts[meshGroupBase]; the sub-slices
					// end at the color's last group and its last manifold.
					manifoldBase := int(meshManifoldStarts[meshGroupBase])
					manifoldEnd := int(meshManifoldStarts[meshGroupBase+colorMeshGroupCount])
					if world.simdWidth == 8 {
						color.wideMeshConstraints8 = sliceWideBuffer(wideMeshConstraints8, meshGroupBase, colorMeshGroupCount,
							manifoldBase, manifoldEnd)
					} else {
						color.wideMeshConstraints = sliceWideBuffer(wideMeshConstraints, meshGroupBase, colorMeshGroupCount,
							manifoldBase, manifoldEnd)
					}
					color.wideMeshManifoldStarts = meshManifoldStarts[meshGroupBase:]
					color.wideMeshConstraintCount = int32(colorMeshGroupCount)
					meshGroupBase += colorMeshGroupCount
					contactBase += colorContactCount
				}

				jointPrepareSpans[i].start = int32(jointBase)
				jointPrepareSpans[i].count = int32(len(color.jointSims))
				jointPrepareSpans[i].joints = color.jointSims
				jointBase += len(color.jointSims)
			}

			// Sentinels
			widePrepareSpans[activeColorCount].start = int32(wideContactCount)
			widePrepareSpans[activeColorCount].count = 0
			widePrepareSpans[activeColorCount].contacts = nil
			assert(wideBase == wideContactCount, "wideBase == wideContactCount")

			meshPrepareSpans[activeColorCount].start = int32(meshGroupCount)
			meshPrepareSpans[activeColorCount].count = 0
			meshPrepareSpans[activeColorCount].contacts = nil
			meshPrepareSpans[activeColorCount].order = nil
			assert(contactBase == meshContactCount, "contactBase == meshContactCount")
			assert(meshGroupBase == meshGroupCount, "meshGroupBase == meshGroupCount")

			jointPrepareSpans[activeColorCount].start = int32(jointCount)
			jointPrepareSpans[activeColorCount].count = 0
			jointPrepareSpans[activeColorCount].joints = nil
			assert(jointBase == jointCount, "jointBase == jointCount")
		}

		//// Special span for overflow to allow for function re-use
		// PORT: world owned, see widePrepareSpans.
		overflowSpans := &world.stepScratch.overflowSpans
		clear(overflowSpans[:])
		overflowSpans[0].start = 0
		overflowSpans[0].count = int32(len(overflow.contacts))
		overflowSpans[0].contacts = overflow.contacts
		overflowSpans[1].start = int32(len(overflow.contacts))
		overflowSpans[1].count = 0
		overflowSpans[1].contacts = nil

		stageCount := 0

		// b3_stagePrepareJoints
		stageCount += 1
		// b3_stagePrepareWideContacts
		stageCount += 1
		// b3_stagePrepareContacts
		stageCount += 1
		// b3_stageIntegrateVelocities
		stageCount += 1
		// b3_stageWarmStart
		stageCount += activeColorCount
		// b3_stageSolve
		stageCount += solverIterations * activeColorCount
		// b3_stageIntegratePositions
		stageCount += 1
		// b3_stageRelax
		stageCount += solverRelaxIterations * activeColorCount
		// b3_stageRestitution
		stageCount += activeColorCount
		// b3_stageStoreWideImpulses
		stageCount += 1
		// b3_stageStoreImpulses
		stageCount += 1

		stages := stackAlloc[solverStage](&world.stack, stageCount, "stages")
		bodyBlocks := stackAlloc[syncBlock](&world.stack, bodyDim.count, "body blocks")
		convexBlocks := stackAlloc[syncBlock](&world.stack, convexPrepareDim.count, "convex blocks")
		meshBlocks := stackAlloc[syncBlock](&world.stack, meshPrepareDim.count, "mesh blocks")
		jointBlocks := stackAlloc[syncBlock](&world.stack, jointPrepareDim.count, "joint blocks")
		graphBlocks := stackAlloc[syncBlock](&world.stack, graphBlockCount, "graph blocks")

		// Split an awake island. This modifies:
		// - stack allocator
		// - world island array and solver set
		// - island indices on bodies, contacts, and joints
		// I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
		// Note: cannot split islands in parallel with FinalizeBodies
		var splitIslandUserTask any
		if world.splitIslandId != NullIndex {
			if world.taskCount < MaxTasks {
				splitIslandUserTask = world.enqueueTaskFcn(splitIslandTask, world, world.userTaskContext, "split")
				world.taskCount += 1
				if splitIslandUserTask != nil {
					world.activeTaskCount += 1
				}
			} else {
				splitIslandTask(world)
			}
		}

		// Prepare body blocks
		initBlocks(bodyBlocks, bodyDim, awakeBodyCount, bodyBlock, math.MaxUint8)

		// Prepare blocks as a single flat parallel-for over the whole constraint range.
		// The task walks spans to decode flat slot indices back to per-color arrays.
		initBlocks(convexBlocks, convexPrepareDim, wideContactCount, wideContactBlock, math.MaxUint8)
		initBlocks(meshBlocks, meshPrepareDim, meshGroupCount, contactBlock, math.MaxUint8)
		initBlocks(jointBlocks, jointPrepareDim, jointCount, jointBlock, math.MaxUint8)

		// Prepare graph work blocks. Each color gets joint blocks followed by contact blocks.
		var graphColorBlocks [GraphColorCount][]syncBlock
		baseGraphBlock := 0
		var graphBlockCounts [GraphColorCount]int
		for i := 0; i < activeColorCount; i++ {
			graphColorBlocks[i] = graphBlocks[baseGraphBlock:]

			colorIndex := uint8(activeColorIndices[i])
			initBlocks(graphBlocks[baseGraphBlock:], graphJointDims[i], colorJointCounts[i], graphJointBlock, colorIndex)
			baseGraphBlock += graphJointDims[i].count

			initBlocks(graphBlocks[baseGraphBlock:], graphWideContactDims[i], colorWideContactCounts[i], graphWideContactBlock,
				colorIndex)
			baseGraphBlock += graphWideContactDims[i].count

			initBlocks(graphBlocks[baseGraphBlock:], graphMeshContactDims[i], colorMeshGroupCounts[i], graphContactBlock, colorIndex)
			baseGraphBlock += graphMeshContactDims[i].count

			graphBlockCounts[i] = graphJointDims[i].count + graphWideContactDims[i].count + graphMeshContactDims[i].count
		}

		assert(baseGraphBlock == graphBlockCount, "(ptrdiff_t)( baseGraphBlock - graphBlocks ) == graphBlockCount")

		stage := 0
		stage = initStage(stages, stage, stagePrepareJoints, jointBlocks, jointPrepareDim.count, math.MaxUint8)
		stage = initStage(stages, stage, stagePrepareWideContacts, convexBlocks, convexPrepareDim.count, math.MaxUint8)
		stage = initStage(stages, stage, stagePrepareContacts, meshBlocks, meshPrepareDim.count, math.MaxUint8)
		stage = initStage(stages, stage, stageIntegrateVelocities, bodyBlocks, bodyDim.count, math.MaxUint8)
		stage = initColorStages(stages, stage, stageWarmStart, 1, activeColorCount, graphColorBlocks[:], graphBlockCounts[:],
			activeColorIndices[:])
		stage = initColorStages(stages, stage, stageSolve, solverIterations, activeColorCount, graphColorBlocks[:],
			graphBlockCounts[:], activeColorIndices[:])
		stage = initStage(stages, stage, stageIntegratePositions, bodyBlocks, bodyDim.count, math.MaxUint8)
		stage = initColorStages(stages, stage, stageRelax, solverRelaxIterations, activeColorCount, graphColorBlocks[:],
			graphBlockCounts[:], activeColorIndices[:])
		stage = initColorStages(stages, stage, stageRestitution, 1, activeColorCount, graphColorBlocks[:], graphBlockCounts[:],
			activeColorIndices[:])
		stage = initStage(stages, stage, stageStoreWideImpulses, convexBlocks, convexPrepareDim.count, math.MaxUint8)
		stage = initStage(stages, stage, stageStoreImpulses, meshBlocks, meshPrepareDim.count, math.MaxUint8)

		assert(stage == stageCount, "(int)( stage - stages ) == stageCount")

		assert(workerCount <= MaxWorkers, "workerCount <= B3_MAX_WORKERS")
		// PORT: the task system receives pointers into this C stack array, so it lives in the world
		// (stepScratch) and is cleared here.
		workerContexts := &world.stepScratch.workerContexts
		clear(workerContexts[:])

		stepContext.graph = graph
		stepContext.activeColorCount = int32(activeColorCount)
		stepContext.workerCount = int32(workerCount)
		stepContext.stageCount = int32(stageCount)
		stepContext.stages = stages
		stepContext.wideConstraints = wideConstraints
		stepContext.wideConstraints8 = wideConstraints8
		stepContext.widePrepareSpans = widePrepareSpans[:]
		stepContext.wideContactCount = int32(wideContactCount)
		stepContext.overflowSpans = overflowSpans[:]
		stepContext.wideMeshConstraints = wideMeshConstraints
		stepContext.wideMeshConstraints8 = wideMeshConstraints8
		stepContext.wideMeshManifoldStarts = meshManifoldStarts
		stepContext.meshPrepareSpans = meshPrepareSpans[:]
		stepContext.jointPrepareSpans = jointPrepareSpans[:]
		atomicStoreU32(&stepContext.atomicSyncBits, 0)
		atomicStoreInt(&stepContext.mainClaimed, 0)

		world.profile.SolverSetup = GetMillisecondsAndReset(&setupTicks)

		constraintTicks := GetTicks()

		jointIdCapacity := getIdCapacity(&world.jointIdPool)
		contactIdCapacity := getIdCapacity(&world.contactIdPool)
		for i := 0; i < workerCount; i++ {
			taskContext := arrayGet(world.taskContexts, i)
			setBitCountAndClear(&taskContext.jointStateBitSet, uint32(jointIdCapacity))
			setBitCountAndClear(&taskContext.hitEventBitSet, uint32(contactIdCapacity))
			taskContext.hasHitEvents = false

			workerContexts[i].context = stepContext
			workerContexts[i].workerIndex = i

			if world.taskCount < MaxTasks {
				workerContexts[i].userTask =
					world.enqueueTaskFcn(solverTask, &workerContexts[i], world.userTaskContext, solverTaskNames[i])
				world.taskCount += 1
				if workerContexts[i].userTask != nil {
					world.activeTaskCount += 1
				}
			} else {
				workerContexts[i].userTask = nil
				solverTask(&workerContexts[i])
			}
		}

		// The calling thread of b3World_Step also enters b3SolverTask as worker 0 and races for the
		// orchestrator slot via the CAS inside. This guarantees progress even when the user's task
		// system can't run the queued worker 0 promptly: it might schedule out of order, have fewer
		// threads than workerCount, or invert priority by parking the calling thread in finishTaskFcn.
		// Whoever wins the CAS becomes the orchestrator; the loser returns and lets the spinner-only
		// path handle workers >0.
		callerContext := workerContext{stepContext, 0, nil}
		solverTask(&callerContext)

		// Finish constraint solve
		for i := 0; i < workerCount; i++ {
			if workerContexts[i].userTask != nil {
				world.finishTaskFcn(workerContexts[i].userTask, world.userTaskContext)
				world.activeTaskCount -= 1
			}
		}

		// Finish island split
		if splitIslandUserTask != nil {
			world.finishTaskFcn(splitIslandUserTask, world.userTaskContext)
			world.activeTaskCount -= 1
		}
		world.splitIslandId = NullIndex

		world.profile.Constraints = GetMillisecondsAndReset(&constraintTicks)

		transformTicks := GetTicks()

		// Prepare the island bit set used in body finalization.
		awakeIslandCount := len(awakeSolverSet.islandSims)
		for i := 0; i < int(world.workerCount); i++ {
			taskContext := &world.taskContexts[i]
			arrayClear(&taskContext.sensorHits)
			setBitCountAndClear(&taskContext.awakeIslandBitSet, uint32(awakeIslandCount))
			taskContext.splitIslandId = NullIndex
			taskContext.splitSleepTime = 0.0
		}

		// Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
		// broad-phase.
		if world.userTreeTask != nil {
			world.finishTaskFcn(world.userTreeTask, world.userTaskContext)
			world.userTreeTask = nil
			world.activeTaskCount -= 1
		}

		validateNoMoved(&world.broadPhase)

		// Finalize bodies. Must happen after the constraint solver and after island splitting.
		parallelFor(world, finalizeBodiesTask, awakeBodyCount, 8, stepContext, "ccd")

		// Free in reverse order
		stackFree(&world.stack, graphBlocks)
		stackFree(&world.stack, jointBlocks)
		stackFree(&world.stack, meshBlocks)
		stackFree(&world.stack, convexBlocks)
		stackFree(&world.stack, bodyBlocks)
		stackFree(&world.stack, stages)
		stackFree(&world.stack, overflow.manifoldConstraints)
		stackFree(&world.stack, overflow.contactConstraints)
		if world.simdWidth == 8 {
			freeWideBuffer(&world.stack, wideMeshConstraints8)
		} else {
			freeWideBuffer(&world.stack, wideMeshConstraints)
		}
		stackFree(&world.stack, meshManifoldStarts)
		stackFree(&world.stack, meshLaneOrder)
		if world.simdWidth == 8 {
			freeWideBuffer(&world.stack, wideConstraints8)
		} else {
			freeWideBuffer(&world.stack, wideConstraints)
		}

		world.profile.Transforms = GetMilliseconds(transformTicks)
	}

	// Report joint events
	{
		jointEventTicks := GetTicks()

		// Gather bits for all joints that have force/torque events
		jointStateBitSet := &world.taskContexts[0].jointStateBitSet
		for i := 1; i < int(world.workerCount); i++ {
			inPlaceUnion(jointStateBitSet, &world.taskContexts[i].jointStateBitSet)
		}

		{
			wordCount := jointStateBitSet.blockCount
			bits := jointStateBitSet.bits

			jointArray := world.joints
			worldIndex0 := world.worldId

			for k := uint32(0); k < wordCount; k++ {
				word := bits[k]
				for word != 0 {
					ctz := ctz64(word)
					jointId := int(64*k + ctz)

					assert(jointId < cap(world.joints), "jointId < world->joints.capacity")

					joint := &jointArray[jointId]

					assert(joint.setIndex == awakeSet, "joint->setIndex == b3_awakeSet")

					event := JointEvent{
						JointId: JointID{
							Index1:     int32(jointId + 1),
							World0:     worldIndex0,
							Generation: joint.generation,
						},
						UserData: joint.userData,
					}

					arrayPush(&world.jointEvents, event)

					// Clear the smallest set bit
					word = word & (word - 1)
				}
			}
		}

		world.profile.JointEvents = GetMilliseconds(jointEventTicks)
	}

	// Report hit events
	{
		hitTicks := GetTicks()

		assert(len(world.contactHitEvents) == 0, "world->contactHitEvents.count == 0")

		// Fast path: if no worker flagged any hit-event candidates during b2StoreImpulsesTask, skip entirely.
		anyHitEvents := false
		for i := 0; i < int(world.workerCount); i++ {
			if world.taskContexts[i].hasHitEvents {
				anyHitEvents = true
				break
			}
		}

		if anyHitEvents {
			// Union per-worker bits into worker 0's bit set.
			hitEventBitSet := &world.taskContexts[0].hitEventBitSet
			for i := 1; i < int(world.workerCount); i++ {
				if world.taskContexts[i].hasHitEvents {
					inPlaceUnion(hitEventBitSet, &world.taskContexts[i].hitEventBitSet)
				}
			}

			threshold := world.hitEventThreshold
			contactArray := world.contacts
			worldId := world.worldId

			wordCount := hitEventBitSet.blockCount
			bits := hitEventBitSet.bits
			for k := uint32(0); k < wordCount; k++ {
				word := bits[k]
				for word != 0 {
					ctz := ctz64(word)
					contactId := int(64*k + ctz)

					contact := &contactArray[contactId]
					assert(contact.setIndex == awakeSet && contact.colorIndex != NullIndex,
						"contact->setIndex == b3_awakeSet && contact->colorIndex != B3_NULL_INDEX")

					shapeA := arrayGet(world.shapes, int(contact.shapeIdA))
					shapeB := arrayGet(world.shapes, int(contact.shapeIdB))
					bodyA := arrayGet(world.bodies, int(shapeA.bodyId))
					bodyB := arrayGet(world.bodies, int(shapeB.bodyId))
					simA := getBodySim(world, bodyA)
					simB := getBodySim(world, bodyB)
					midCenter := LerpPosition(simA.center, simB.center, 0.5)

					event := ContactHitEvent{}
					event.ApproachSpeed = threshold

					found := false
					triangleIndex := 0
					manifoldCount := int(contact.manifoldCount)
					for i := 0; i < manifoldCount; i++ {
						manifold := &contact.manifolds[i]
						pointCount := int(manifold.PointCount)
						for p := 0; p < pointCount; p++ {
							mp := &manifold.Points[p]
							approachSpeed := -mp.NormalVelocity

							// Need to check total impulse because the point may be speculative and not colliding
							if approachSpeed > event.ApproachSpeed && mp.TotalNormalImpulse > 0.0 {
								event.ApproachSpeed = approachSpeed
								event.Point = OffsetPos(midCenter, Lerp(mp.AnchorA, mp.AnchorB, 0.5))
								event.Normal = manifold.Normal
								triangleIndex = int(mp.TriangleIndex)
								found = true
							}
						}
					}

					if found == true {
						event.ShapeIdA = ShapeID{shapeA.id + 1, worldId, shapeA.generation}
						event.ShapeIdB = ShapeID{shapeB.id + 1, worldId, shapeB.generation}

						event.ContactId = ContactID{
							Index1:     contact.contactId + 1,
							World0:     worldId,
							Padding:    0,
							Generation: contact.generation,
						}

						// shapeB is never a compound today (asserted in b3CreateContact), so the
						// childIndex argument is irrelevant for it. shapeA carries the compound.
						event.UserMaterialIdA = getShapeUserMaterialId(shapeA, int(contact.childIndex), triangleIndex)
						event.UserMaterialIdB = getShapeUserMaterialId(shapeB, 0, triangleIndex)

						arrayPush(&world.contactHitEvents, event)
					}

					// Clear the smallest set bit
					word = word & (word - 1)
				}
			}
		}

		world.profile.HitEvents = GetMilliseconds(hitTicks)
	}

	{
		refitTicks := GetTicks()

		world.broadPhase.trees[KinematicBody].refit()
		world.broadPhase.trees[DynamicBody].refit()

		validateBroadPhase(&world.broadPhase)

		world.profile.Refit = GetMilliseconds(refitTicks)
	}

	bulletBodyCount := int(atomicLoadInt(&stepContext.bulletBodyCount))
	if bulletBodyCount > 0 {
		bulletTicks := GetTicks()

		// Fast bullet bodies
		// Note: a bullet body may be moving slow
		minRange := 8
		parallelFor(world, bulletBodyTask, bulletBodyCount, minRange, stepContext, "bullets")

		// Serially enlarge broad-phase proxies for bullet shapes
		broadPhase := &world.broadPhase
		dynamicTree := &broadPhase.trees[DynamicBody]

		// Fast array access is important here
		bodyArray := world.bodies
		bodySimArray := awakeSolverSet.bodySims
		shapeArray := world.shapes

		// Serially enlarge broad-phase proxies for bullet shapes
		bulletBodySimIndices := stepContext.bulletBodies

		// This loop has non-deterministic order but it shouldn't affect the result
		for i := 0; i < bulletBodyCount; i++ {
			bulletBodySim := &bodySimArray[bulletBodySimIndices[i]]
			if (bulletBodySim.flags & enlargeBulletBounds) == 0 {
				continue
			}

			// Clear flag
			bulletBodySim.flags &^= enlargeBulletBounds

			bodyId := int(bulletBodySim.bodyId)
			assert(0 <= bodyId && bodyId < len(world.bodies), "0 <= bodyId && bodyId < world->bodies.count")
			bulletBody := &bodyArray[bodyId]

			shapeId := int(bulletBody.headShapeId)
			for shapeId != NullIndex {
				shape := &shapeArray[shapeId]

				proxyKey := int(shape.proxyKey)
				proxyId := proxyId(proxyKey)
				validate(proxyType(proxyKey) == DynamicBody, "B3_PROXY_TYPE( proxyKey ) == b3_dynamicBody")

				treeAABB := dynamicTree.GetAABB(proxyId)
				shapeFatAABB := world.fatAABBs[shapeId]

				if treeAABB.Contains(shapeFatAABB) == false {
					dynamicTree.EnlargeProxy(proxyId, shapeFatAABB)
				}

				shapeId = int(shape.nextShapeId)
			}
		}

		world.profile.Bullets = GetMilliseconds(bulletTicks)
	}

	stackFree(&world.stack, stepContext.bulletBodies)
	stepContext.bulletBodies = nil
	atomicStoreInt(&stepContext.bulletBodyCount, 0)

	// Report sensor hits. This may include bullets sensor hits.
	{
		sensorHitTicks := GetTicks()

		workerCount := int(world.workerCount)
		assert(workerCount == len(world.taskContexts), "workerCount == world->taskContexts.count")

		for i := 0; i < workerCount; i++ {
			taskContext := &world.taskContexts[i]
			hitCount := len(taskContext.sensorHits)
			hits := taskContext.sensorHits

			for j := 0; j < hitCount; j++ {
				hit := hits[j]
				sensorShape := arrayGet(world.shapes, int(hit.sensorId))
				visitor_ := arrayGet(world.shapes, int(hit.visitorId))

				sensor := arrayGet(world.sensors, int(sensorShape.sensorIndex))
				shapeRef := visitor{
					shapeId:    hit.visitorId,
					generation: visitor_.generation,
				}
				arrayPush(&sensor.hits, shapeRef)
			}
		}

		world.profile.SensorHits = GetMilliseconds(sensorHitTicks)
	}

	// Island sleeping
	// This must be done last because putting islands to sleep invalidates the enlarged body bits.
	// todo_erin figure out how to do this in parallel with tree refit
	if world.enableSleep == true {
		sleepTicks := GetTicks()

		// Collect split island candidate for the next time step. No need to split if sleeping is disabled.
		assert(world.splitIslandId == NullIndex, "world->splitIslandId == B3_NULL_INDEX")
		splitSleepTimer := float32(0.0)
		for i := 0; i < int(world.workerCount); i++ {
			taskContext := &world.taskContexts[i]
			if taskContext.splitIslandId != NullIndex && taskContext.splitSleepTime >= splitSleepTimer {
				assert(taskContext.splitSleepTime > 0.0, "taskContext->splitSleepTime > 0.0f")

				// Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
				if taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < world.splitIslandId {
					continue
				}

				world.splitIslandId = taskContext.splitIslandId
				splitSleepTimer = taskContext.splitSleepTime
			}
		}

		awakeIslandBitSet := &world.taskContexts[0].awakeIslandBitSet
		for i := 1; i < int(world.workerCount); i++ {
			inPlaceUnion(awakeIslandBitSet, &world.taskContexts[i].awakeIslandBitSet)
		}

		// Need to process in reverse because this moves islands to sleeping solver sets.
		islands := awakeSolverSet.islandSims
		count := len(awakeSolverSet.islandSims)
		for islandIndex := count - 1; islandIndex >= 0; islandIndex -= 1 {
			if getBit(awakeIslandBitSet, uint32(islandIndex)) == true {
				// this island is still awake
				continue
			}

			island := &islands[islandIndex]
			islandId := int(island.islandId)

			trySleepIsland(world, islandId)
		}

		validateSolverSets(world)

		world.profile.SleepIslands = GetMilliseconds(sleepTicks)
	}
}
