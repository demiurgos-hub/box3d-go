// Fork reference: testdata/golem_post_toi applies only the documented native
// post-TOI contact correction to pinned C. Original upstream fixtures remain intact.
// Go-only test: no upstream counterpart. Recording and replay with joints, against the pinned
// upstream C sources (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off.
// testdata/replay_joints_golden_gen.c records a scene with every joint type (distance, motor,
// prismatic, revolute, spherical, weld, wheel, parallel and filter) on the empty world, calls every
// recorded generic b3Joint_* mutator and b3<Kind>Joint_* setter, creates and destroys joints
// (directly and through b3DestroyBody) and wakes a sleeping island through b3Joint_WakeBodies;
// rjRun mirrors it line by line. The tests check, at 1, 2 and 4 workers:
//   - the C recording replays in Go bit for bit, headless and through the RecPlayer: every recorded
//     StateHash, the per-step awake count and b3HashWorldState, and the final bodies and joints;
//   - the Go simulation of the scene matches the C run step for step, and the Go recording's op
//     stream and registry are byte-identical to the C recording's (world ids masked as in
//     recording_golden_test.go; the seed snapshot is Go-native and skipped);
//   - the player seeks and scrubs (backward to keyframe and off-keyframe frames, forward, Restart)
//     on the C and Go recordings and on a Go recording started mid-life, whose seed snapshot holds
//     every joint type, always landing on the state of a straight replay.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const rjSteps = 180

// rjPrintWorld mirrors PrintWorld in testdata/replay_joints_golden_gen.c.
func rjPrintWorld(sb *strings.Builder, tag string, worldId WorldID) {
	replayGoldenPrintBodies(sb, tag, worldId)
	world := getWorldFromId(worldId)
	for i := range world.joints {
		j := &world.joints[i]
		if int(j.jointId) != i {
			continue
		}
		jointId := JointID{int32(i + 1), world.worldId, j.generation}
		fmt.Fprintf(sb, "%s joint %d %d | %d %d %d", tag, i, j.generation, int(jointId.GetType()),
			jointId.GetBodyA().Index1, jointId.GetBodyB().Index1)
		replayGoldenVec(sb, jointId.GetConstraintForce())
		fmt.Fprintf(sb, " %08x\n", math.Float32bits(jointId.GetLinearSeparation()))
	}
	fmt.Fprintf(sb, "%s world | %d %016x\n", tag, worldId.GetAwakeBodyCount(), hashWorldState(world))
}

// replayGoldenPrintBodies prints the body lines of replayGoldenPrintWorld.
func replayGoldenPrintBodies(sb *strings.Builder, tag string, worldId WorldID) {
	var all strings.Builder
	replayGoldenPrintWorld(&all, tag, worldId)
	lines := strings.Split(strings.TrimRight(all.String(), "\n"), "\n")
	for _, line := range lines[:len(lines)-1] {
		sb.WriteString(line)
		sb.WriteString("\n")
	}
}

func rjStepLine(step int, worldId WorldID) string {
	return fmt.Sprintf("step %d | %d %016x", step, worldId.GetAwakeBodyCount(), hashWorldState(getWorldFromId(worldId)))
}

func rjFinal(tag string, worldId WorldID) []string {
	var sb strings.Builder
	rjPrintWorld(&sb, tag, worldId)
	return strings.Split(strings.TrimRight(sb.String(), "\n"), "\n")
}

func rjCreateDynamic(worldId WorldID, p Pos, q Quat) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = DynamicBody
	bodyDef.Position = p
	bodyDef.Rotation = q
	return worldId.CreateBody(&bodyDef)
}

// rjResult is one run of the scene.
type rjResult struct {
	data  []byte   // the recording
	steps []string // "step" lines after every step of the whole run
	final []string // "original" lines after the last step
}

// rjRun runs the scene of testdata/replay_joints_golden_gen.c. With startStep 0 the recording
// starts on the empty world as in C; otherwise the scene is built and stepped unrecorded and the
// recording starts before the mutators of step startStep.
func rjRun(workerCount int, startStep int) rjResult {
	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	recording := CreateRecording(0)
	if startStep == 0 {
		worldId.StartRecording(recording)
	}

	// Ground: static box, top face at y = 0
	var groundId BodyID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Name = "ground"
		bodyDef.Position = Pos{0.0, -0.5, 0.0}
		groundId = worldId.CreateBody(&bodyDef)
		shapeDef := DefaultShapeDef()
		box := MakeBoxHull(20.0, 0.5, 20.0)
		groundId.CreateHullShape(&shapeDef, &box.Base)
	}

	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	cube := MakeCubeHull(0.25)
	bar := MakeBoxHull(0.5, 0.125, 0.125)

	// Cone axis (frame z) pointing down
	down := MakeQuatFromAxisAngle(Vec3AxisX, 0.5*Pi)
	// Suspension axis (frame x) pointing up
	up := MakeQuatFromAxisAngle(Vec3AxisZ, 0.5*Pi)
	// 3-4-5 rotation about z
	tilt := Quat{Vec3{0.0, 0.0, 0.6}, 0.8}

	// Distance joint: spring, limit and motor
	var distanceId JointID
	{
		bodyId := rjCreateDynamic(worldId, Pos{-6.0, 3.0, -4.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetLinearVelocity(Vec3{2.0, 0.0, 1.0})

		def := DefaultDistanceJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{-6.0, 5.5, -4.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.25, 0.0}, QuatIdentity}
		def.Length = 2.0
		def.EnableSpring = true
		def.LowerSpringForce = -50.0
		def.UpperSpringForce = 50.0
		def.Hertz = 2.0
		def.DampingRatio = 0.5
		def.EnableLimit = true
		def.MinLength = 1.0
		def.MaxLength = 3.0
		def.EnableMotor = true
		def.MaxMotorForce = 10.0
		def.MotorSpeed = 0.5
		distanceId = worldId.CreateDistanceJoint(&def)
	}

	// Motor joint chasing a velocity with springs
	var motorBodyId BodyID
	var motorId JointID
	{
		motorBodyId = rjCreateDynamic(worldId, Pos{-3.0, 1.0, -4.0}, QuatIdentity)
		motorBodyId.CreateHullShape(&shapeDef, &cube.Base)

		def := DefaultMotorJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = motorBodyId
		def.Base.LocalFrameA = Transform{Vec3{-3.0, 1.0, -4.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, QuatIdentity}
		def.LinearVelocity = Vec3{0.5, 0.0, 0.0}
		def.MaxVelocityForce = 20.0
		def.AngularVelocity = Vec3{0.0, 1.0, 0.0}
		def.MaxVelocityTorque = 5.0
		def.LinearHertz = 2.0
		def.LinearDampingRatio = 0.75
		def.MaxSpringForce = 50.0
		def.AngularHertz = 1.0
		def.AngularDampingRatio = 0.5
		def.MaxSpringTorque = 10.0
		motorId = worldId.CreateMotorJoint(&def)
	}

	// Prismatic slider on a tilted axis: spring, limit and motor
	var prismaticId JointID
	{
		bodyId := rjCreateDynamic(worldId, Pos{0.0, 1.5, -4.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)

		def := DefaultPrismaticJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{0.0, 2.0, -4.0}, tilt}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, tilt}
		def.EnableSpring = true
		def.Hertz = 1.0
		def.DampingRatio = 0.5
		def.TargetTranslation = 0.5
		def.EnableLimit = true
		def.LowerTranslation = -1.0
		def.UpperTranslation = 1.0
		def.EnableMotor = true
		def.MaxMotorForce = 20.0
		def.MotorSpeed = 1.0
		prismaticId = worldId.CreatePrismaticJoint(&def)
	}

	// Revolute hinge: spring, limit and motor
	var revoluteId JointID
	{
		bodyId := rjCreateDynamic(worldId, Pos{3.0, 2.0, -4.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &bar.Base)

		def := DefaultRevoluteJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{2.5, 2.5, -4.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{-0.5, 0.0, 0.0}, QuatIdentity}
		def.TargetAngle = 0.25
		def.EnableSpring = true
		def.Hertz = 1.0
		def.DampingRatio = 0.25
		def.EnableLimit = true
		def.LowerAngle = -0.5
		def.UpperAngle = 0.75
		def.EnableMotor = true
		def.MaxMotorTorque = 2.0
		def.MotorSpeed = 1.0
		revoluteId = worldId.CreateRevoluteJoint(&def)
	}

	// Spherical pendulum with cone limit, twist limit and motor
	var sphericalLimitId JointID
	{
		bodyId := rjCreateDynamic(worldId, Pos{-6.0, 4.0, 0.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetLinearVelocity(Vec3{3.0, 0.0, 1.0})
		bodyId.SetAngularVelocity(Vec3{0.0, 4.0, 0.0})

		def := DefaultSphericalJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{-6.0, 5.5, 0.0}, down}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 1.0, 0.0}, down}
		def.EnableConeLimit = true
		def.ConeAngle = 0.5
		def.EnableTwistLimit = true
		def.LowerTwistAngle = -0.25
		def.UpperTwistAngle = 0.25
		def.EnableMotor = true
		def.MaxMotorTorque = 0.5
		def.MotorVelocity = Vec3{0.0, 1.0, 0.0}
		sphericalLimitId = worldId.CreateSphericalJoint(&def)
	}

	// Spherical pendulum with a spring towards a target rotation
	var sphericalSpringId JointID
	{
		bodyId := rjCreateDynamic(worldId, Pos{-8.0, 4.0, 0.0}, QuatIdentity)
		bodyId.CreateHullShape(&shapeDef, &cube.Base)
		bodyId.SetAngularVelocity(Vec3{1.0, 0.0, -2.0})

		def := DefaultSphericalJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = Transform{Vec3{-8.0, 5.0, 0.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.5, 0.0}, QuatIdentity}
		def.EnableSpring = true
		def.Hertz = 2.0
		def.DampingRatio = 0.25
		def.TargetRotation = Quat{Vec3{0.0, 0.6, 0.0}, 0.8}
		sphericalSpringId = worldId.CreateSphericalJoint(&def)
	}

	// Static pillar with a rigid and a soft weld
	var weldIds [2]JointID
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Position = Pos{-3.0, 1.5, 4.0}
		pillarId := worldId.CreateBody(&bodyDef)
		pillar := MakeBoxHull(0.25, 1.5, 0.25)
		pillarId.CreateHullShape(&shapeDef, &pillar.Base)

		prevId := pillarId
		prevAnchor := Vec3{0.25, 1.25, 0.0}
		for i := range 2 {
			bodyId := rjCreateDynamic(worldId, Pos{-2.5 + 0.5*float32(i), 2.75, 4.0}, QuatIdentity)
			bodyId.CreateHullShape(&shapeDef, &cube.Base)

			def := DefaultWeldJointDef()
			def.Base.BodyIdA = prevId
			def.Base.BodyIdB = bodyId
			def.Base.LocalFrameA = Transform{prevAnchor, QuatIdentity}
			def.Base.LocalFrameB = Transform{Vec3{-0.25, 0.0, 0.0}, QuatIdentity}
			if i == 1 {
				def.LinearHertz = 5.0
				def.LinearDampingRatio = 0.75
				def.AngularHertz = 3.0
				def.AngularDampingRatio = 0.5
			}
			weldIds[i] = worldId.CreateWeldJoint(&def)

			prevId = bodyId
			prevAnchor = Vec3{0.25, 0.0, 0.0}
		}
	}

	// Car: chassis and four sphere wheels on wheel joints
	var wheelIds [4]JointID
	{
		chassisId := rjCreateDynamic(worldId, Pos{4.0, 1.0, 4.0}, QuatIdentity)
		chassis := MakeBoxHull(1.0, 0.25, 0.5)
		chassisId.CreateHullShape(&shapeDef, &chassis.Base)

		sphere := Sphere{Vec3{0.0, 0.0, 0.0}, 0.375}
		wheelShapeDef := shapeDef
		wheelShapeDef.BaseMaterial.Friction = 0.875

		for i := range 4 {
			x := float32(0.75)
			if i&1 != 0 {
				x = -0.75
			}
			z := float32(0.75)
			if i&2 != 0 {
				z = -0.75
			}
			front := x > 0.0

			wheelId := rjCreateDynamic(worldId, Pos{4.0 + x, 0.5, 4.0 + z}, QuatIdentity)
			wheelId.CreateSphereShape(&wheelShapeDef, &sphere)

			def := DefaultWheelJointDef()
			def.Base.BodyIdA = chassisId
			def.Base.BodyIdB = wheelId
			def.Base.LocalFrameA = Transform{Vec3{x, -0.5, z}, up}
			def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, QuatIdentity}
			def.EnableSuspensionSpring = true
			def.SuspensionHertz = 4.0
			def.SuspensionDampingRatio = 0.625
			def.EnableSuspensionLimit = true
			def.LowerSuspensionLimit = -0.25
			def.UpperSuspensionLimit = 0.125
			if front {
				def.EnableSteering = true
				def.SteeringHertz = 5.0
				def.SteeringDampingRatio = 0.75
				def.MaxSteeringTorque = 50.0
				def.TargetSteeringAngle = 0.25
				def.EnableSteeringLimit = true
				def.LowerSteeringLimit = -0.375
				def.UpperSteeringLimit = 0.375
			} else {
				def.EnableSpinMotor = true
				def.MaxSpinTorque = 20.0
				def.SpinSpeed = -8.0
			}
			wheelIds[i] = worldId.CreateWheelJoint(&def)
		}
	}

	// Parallel joint holding a spinning box parallel to the ground
	var parallelBodyId BodyID
	var parallelId JointID
	{
		parallelBodyId = rjCreateDynamic(worldId, Pos{8.0, 2.0, -3.0}, QuatIdentity)
		parallelBodyId.CreateHullShape(&shapeDef, &cube.Base)
		parallelBodyId.SetAngularVelocity(Vec3{3.0, 1.0, 2.0})

		def := DefaultParallelJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = parallelBodyId
		def.Base.LocalFrameA = Transform{Vec3{8.0, 2.5, -3.0}, QuatIdentity}
		def.Base.LocalFrameB = Transform{Vec3{0.0, 0.0, 0.0}, QuatIdentity}
		def.Base.CollideConnected = true
		def.Hertz = 2.0
		def.DampingRatio = 0.5
		def.MaxTorque = 5.0
		parallelId = worldId.CreateParallelJoint(&def)
	}

	// Filter joint between two overlapping boxes
	var filterBodyId BodyID
	var filterId JointID
	{
		filterBodyId = rjCreateDynamic(worldId, Pos{-8.0, 0.25, 4.0}, QuatIdentity)
		filterBodyId.CreateHullShape(&shapeDef, &cube.Base)
		upperId := rjCreateDynamic(worldId, Pos{-8.0, 0.625, 4.125}, QuatIdentity)
		upperId.CreateHullShape(&shapeDef, &cube.Base)

		def := DefaultFilterJointDef()
		def.Base.BodyIdA = filterBodyId
		def.Base.BodyIdB = upperId
		filterId = worldId.CreateFilterJoint(&def)
	}

	lateBodyId := NullBodyID
	lateJointId := NullJointID

	var res rjResult
	for step := 1; step <= rjSteps; step++ {
		if step == startStep {
			worldId.StartRecording(recording)
		}

		switch step {
		case 20:
			revoluteId.SetLocalFrameA(Transform{Vec3{2.5, 2.25, -4.0}, QuatIdentity})
			distanceId.SetLocalFrameB(Transform{Vec3{0.0, 0.25, 0.125}, QuatIdentity})
			weldIds[1].SetCollideConnected(true)
			prismaticId.SetConstraintTuning(30.0, 5.0)
			distanceId.SetForceThreshold(5.0)
			revoluteId.SetTorqueThreshold(1.0)
		case 30:
			distanceId.DistanceJointSetLength(1.5)
			distanceId.DistanceJointEnableSpring(true)
			distanceId.DistanceJointSetSpringForceRange(-20.0, 40.0)
			distanceId.DistanceJointSetSpringHertz(3.0)
			distanceId.DistanceJointSetSpringDampingRatio(0.25)
			distanceId.DistanceJointEnableLimit(true)
			distanceId.DistanceJointSetLengthRange(0.75, 2.5)
			distanceId.DistanceJointEnableMotor(true)
			distanceId.DistanceJointSetMotorSpeed(-1.0)
			distanceId.DistanceJointSetMaxMotorForce(5.0)
		case 40:
			motorId.MotorJointSetLinearVelocity(Vec3{0.0, 0.0, 0.5})
			motorId.MotorJointSetAngularVelocity(Vec3{0.5, 0.0, 0.0})
			motorId.MotorJointSetMaxVelocityForce(10.0)
			motorId.MotorJointSetMaxVelocityTorque(2.0)
			motorId.MotorJointSetLinearHertz(4.0)
			motorId.MotorJointSetLinearDampingRatio(0.5)
			motorId.MotorJointSetAngularHertz(2.0)
			motorId.MotorJointSetAngularDampingRatio(0.25)
			motorId.MotorJointSetMaxSpringForce(25.0)
			motorId.MotorJointSetMaxSpringTorque(5.0)
		case 50:
			bodyDef := DefaultBodyDef()
			bodyDef.Type = DynamicBody
			bodyDef.Name = "late"
			bodyDef.Position = Pos{0.0, 4.0, 2.0}
			lateBodyId = worldId.CreateBody(&bodyDef)
			capsule := Capsule{Vec3{-0.5, 0.0, 0.0}, Vec3{0.5, 0.0, 0.0}, 0.125}
			lateBodyId.CreateCapsuleShape(&shapeDef, &capsule)

			def := DefaultRevoluteJointDef()
			def.Base.BodyIdA = groundId
			def.Base.BodyIdB = lateBodyId
			def.Base.LocalFrameA = Transform{Vec3{-0.5, 4.5, 2.0}, QuatIdentity}
			def.Base.LocalFrameB = Transform{Vec3{-0.5, 0.0, 0.0}, QuatIdentity}
			lateJointId = worldId.CreateRevoluteJoint(&def)
		case 60:
			prismaticId.PrismaticJointEnableSpring(true)
			prismaticId.PrismaticJointSetSpringHertz(2.0)
			prismaticId.PrismaticJointSetSpringDampingRatio(0.75)
			prismaticId.PrismaticJointSetTargetTranslation(-0.5)
			prismaticId.PrismaticJointEnableLimit(true)
			prismaticId.PrismaticJointSetLimits(-0.75, 0.5)
			prismaticId.PrismaticJointEnableMotor(true)
			prismaticId.PrismaticJointSetMotorSpeed(-2.0)
			prismaticId.PrismaticJointSetMaxMotorForce(10.0)

			revoluteId.RevoluteJointEnableSpring(true)
			revoluteId.RevoluteJointSetSpringHertz(2.0)
			revoluteId.RevoluteJointSetSpringDampingRatio(0.5)
			revoluteId.RevoluteJointSetTargetAngle(-0.25)
			revoluteId.RevoluteJointEnableLimit(true)
			revoluteId.RevoluteJointSetLimits(-0.75, 0.5)
			revoluteId.RevoluteJointEnableMotor(true)
			revoluteId.RevoluteJointSetMotorSpeed(-1.5)
			revoluteId.RevoluteJointSetMaxMotorTorque(1.0)
		case 70:
			filterBodyId.SetAwake(false)
		case 75:
			filterId.WakeBodies()
		case 80:
			sphericalLimitId.SphericalJointEnableConeLimit(true)
			sphericalLimitId.SphericalJointSetConeLimit(0.75)
			sphericalLimitId.SphericalJointEnableTwistLimit(true)
			sphericalLimitId.SphericalJointSetTwistLimits(-0.5, 0.5)
			sphericalLimitId.SphericalJointEnableMotor(true)
			sphericalLimitId.SphericalJointSetMotorVelocity(Vec3{0.5, 0.0, 0.0})
			sphericalLimitId.SphericalJointSetMaxMotorTorque(1.0)
			sphericalSpringId.SphericalJointEnableSpring(true)
			sphericalSpringId.SphericalJointSetSpringHertz(3.0)
			sphericalSpringId.SphericalJointSetSpringDampingRatio(0.5)
			sphericalSpringId.SphericalJointSetTargetRotation(QuatIdentity)
		case 90:
			weldIds[0].WeldJointSetLinearHertz(8.0)
			weldIds[0].WeldJointSetLinearDampingRatio(0.5)
			weldIds[1].WeldJointSetAngularHertz(0.0)
			weldIds[1].WeldJointSetAngularDampingRatio(1.0)
			parallelId.ParallelJointSetSpringHertz(4.0)
			parallelId.ParallelJointSetSpringDampingRatio(0.25)
			parallelId.ParallelJointSetMaxTorque(0.5)
		case 100:
			lateJointId.Destroy(true)
		case 110:
			wheelIds[1].WheelJointEnableSuspension(true)
			wheelIds[0].WheelJointSetSuspensionHertz(6.0)
			wheelIds[0].WheelJointSetSuspensionDampingRatio(0.5)
			wheelIds[3].WheelJointEnableSuspensionLimit(true)
			wheelIds[3].WheelJointSetSuspensionLimits(-0.125, 0.25)
			wheelIds[0].WheelJointEnableSpinMotor(true)
			wheelIds[1].WheelJointSetSpinMotorSpeed(4.0)
			wheelIds[3].WheelJointSetMaxSpinTorque(10.0)
			wheelIds[2].WheelJointEnableSteering(true)
			wheelIds[0].WheelJointSetSteeringHertz(3.0)
			wheelIds[0].WheelJointSetSteeringDampingRatio(0.5)
			wheelIds[0].WheelJointSetMaxSteeringTorque(30.0)
			wheelIds[2].WheelJointEnableSteeringLimit(true)
			wheelIds[2].WheelJointSetSteeringLimits(-0.125, 0.125)
			wheelIds[0].WheelJointSetTargetSteeringAngle(-0.25)
		case 120:
			// Joins the late body's island to the parallel box's
			def := DefaultDistanceJointDef()
			def.Base.BodyIdA = lateBodyId
			def.Base.BodyIdB = parallelBodyId
			def.Base.LocalFrameA = Transform{Vec3{0.5, 0.0, 0.0}, QuatIdentity}
			def.Base.LocalFrameB = Transform{Vec3{0.0, 0.25, 0.0}, QuatIdentity}
			def.Length = 3.0
			lateJointId = worldId.CreateDistanceJoint(&def)
		case 130:
			filterId.Destroy(false)
			motorBodyId.Destroy()
		case 140:
			distanceId.DistanceJointEnableSpring(false)
			revoluteId.RevoluteJointEnableLimit(false)
			prismaticId.PrismaticJointEnableMotor(false)
			wheelIds[0].WheelJointEnableSteering(false)
			sphericalSpringId.SphericalJointEnableSpring(false)
			lateJointId.SetCollideConnected(true)
		case 150:
			sphericalSpringId.WakeBodies()
			wheelIds[3].SetLocalFrameB(Transform{Vec3{0.0, 0.0, 0.125}, QuatIdentity})
			lateJointId.SetConstraintTuning(60.0, 2.0)
		case 160:
			lateBodyId.Destroy()
		}

		worldId.Step(1.0/60.0, 4)
		res.steps = append(res.steps, rjStepLine(step, worldId))
	}

	worldId.StopRecording()

	res.final = rjFinal("original", worldId)
	res.data = append([]byte(nil), recording.GetData()...)
	recording.Destroy()
	worldId.Destroy()
	return res
}

// rjGolden returns the lines of testdata/replay_joints_golden.txt with the given tag.
func rjGolden(tag string) []string {
	data, err := os.ReadFile("testdata/golem_post_toi/replay_joints_golden.txt")
	Expect(err).NotTo(HaveOccurred())
	var out []string
	for _, line := range strings.Split(strings.TrimRight(string(data), "\n"), "\n") {
		if strings.HasPrefix(line, tag+" ") {
			out = append(out, line)
		}
	}
	Expect(out).NotTo(BeEmpty())
	return out
}

// rjGoldenFinal returns the final lines with the given tag, the tag replaced by "original".
func rjGoldenFinal(tag string) []string {
	lines := rjGolden(tag)
	for i := range lines {
		lines[i] = "original" + strings.TrimPrefix(lines[i], tag)
	}
	return lines
}

func rjCData() []byte {
	data, err := os.ReadFile("testdata/golem_post_toi/replay_joints_golden.b3rec")
	Expect(err).NotTo(HaveOccurred())
	return data
}

func rjCompareLines(what string, got, want []string) {
	for i := range min(len(want), len(got)) {
		if want[i] != got[i] {
			Fail(fmt.Sprintf("%s: first mismatch at line %d:\n got  %s\n want %s", what, i+1, got[i], want[i]))
		}
	}
	Expect(len(got)).To(Equal(len(want)), what)
}

// rjHeadless replays a recording that starts on the empty world with a reader that has no player
// (the headless path of b3RecReader). It returns the step lines and the final lines.
func rjHeadless(data []byte, workerCount int) ([]string, []string) {
	hdr := recDecodeHeader(data)
	Expect(hdr.magic).To(Equal(recMagic))
	headerEnd := recHeaderSize + int(hdr.snapshotSize)
	registryEnd := int(hdr.registryOffset)

	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)
	defer worldId.Destroy()

	var rdr recReader
	rdr.data = data
	rdr.size = len(data)
	rdr.cursor = headerEnd
	rdr.replayWorldId = worldId
	rdr.ok = true
	Expect(recLoadSlots(&rdr, data, len(data), hdr.registryOffset, hdr.registryByteCount)).To(BeTrue())
	// ground, cube, bar, pillar and chassis boxes
	Expect(rdr.slotCount).To(Equal(5))

	seed, ok := recReadForeignEmptySeed(data[recHeaderSize:headerEnd])
	if !ok {
		// A Go recording carries a Go-native seed image of the empty world.
		Expect(deserializeIntoShell(data[recHeaderSize:headerEnd], getWorldFromId(worldId), &rdr)).To(BeTrue())
	} else {
		recApplyForeignSeed(getWorldFromId(worldId), &seed)
	}

	var steps []string
	for rdr.cursor < registryEnd && rdr.ok {
		op := recDispatchOne(&rdr)
		if op < 0 || op == recOpDestroyWorld {
			break
		}
		if op == recOpStep {
			steps = append(steps, rjStepLine(len(steps)+1, worldId))
		}
		Expect(rdr.diverged).To(BeFalse(), fmt.Sprintf("diverged at frame %d (op 0x%02X)", len(steps), op))
	}
	Expect(rdr.ok).To(BeTrue())
	Expect(rdr.diverged).To(BeFalse())
	return steps, rjFinal("original", worldId)
}

// rjPlayerChecks plays a recording with the RecPlayer at the given worker count: straight to the
// end, then seeks and scrubs. steps holds the expected step line of every frame (frame f is
// steps[f-1], numbered from 1 in the recording) and final the expected final lines.
func rjPlayerChecks(data []byte, workerCount int, steps []string, final []string) {
	Expect(ValidateReplay(data, workerCount)).To(BeTrue())

	player := CreatePlayer(data, workerCount)
	Expect(player).NotTo(BeNil())
	defer player.Destroy()
	frameCount := len(steps)
	Expect(player.GetFrameCount()).To(Equal(frameCount))
	Expect(player.GetKeyframeInterval()).To(Equal(16))

	frameLine := func(f int) string {
		// The step number in the expected lines is the step of the original run.
		line := steps[f-1]
		return line[strings.Index(line, " | "):]
	}
	check := func(what string, f int) {
		Expect(player.HasDiverged()).To(BeFalse(), what)
		Expect(player.GetFrame()).To(Equal(f), what)
		got := rjStepLine(0, player.GetWorldId())
		Expect(got[strings.Index(got, " | "):]).To(Equal(frameLine(f)), fmt.Sprintf("%s at frame %d", what, f))
	}

	for player.StepFrame() {
		check("straight", player.GetFrame())
	}
	Expect(player.IsAtEnd()).To(BeTrue())
	check("end", frameCount)
	// Keyframes (world snapshots with joints) were captured on the way.
	Expect(player.GetKeyframeBytes()).To(BeNumerically(">", 0))
	rjCompareLines("player final", rjFinal("original", player.GetWorldId()), final)

	// Backward seeks onto keyframes (16, 64) and between them, forward seeks, and single steps
	// after a seek.
	for _, f := range []int{37, 64, 16, 99, 5, 130, 129, 1, 150, frameCount - 1, 48, frameCount, 17} {
		if f > frameCount {
			continue
		}
		player.SeekFrame(f)
		check("seek", f)
		if f < frameCount {
			Expect(player.StepFrame()).To(BeTrue())
			check("step after seek", f+1)
		}
	}

	// Scrub back and forth one frame at a time around a keyframe.
	for f := 70; f >= 60; f-- {
		player.SeekFrame(f)
		check("scrub back", f)
	}
	for f := 60; f <= 70; f++ {
		player.SeekFrame(f)
		check("scrub forward", f)
	}

	// From an off-keyframe frame to the end.
	player.SeekFrame(37)
	check("seek", 37)
	for player.StepFrame() {
	}
	check("end after seek", frameCount)
	rjCompareLines("final after seek", rjFinal("original", player.GetWorldId()), final)

	player.Restart()
	Expect(player.GetFrame()).To(Equal(0))
	for player.StepFrame() {
		check("after restart", player.GetFrame())
	}
	rjCompareLines("final after restart", rjFinal("original", player.GetWorldId()), final)
}

// rjCompareOps compares the op stream and registry of a Go recording with the C recording, as
// recGoldenCompare does.
func rjCompareOps(got []byte) {
	want := rjCData()
	gh, _, gops, greg := recGoldenParts(got)
	wh, _, wops, wreg := recGoldenParts(want)

	wantHeader := append([]byte(nil), wh...)
	if validateEnabled {
		wantHeader[10] = 1
	}
	Expect(gh[:24]).To(Equal(wantHeader[:24]))
	Expect(gh[40:48]).To(Equal(wantHeader[40:48]))

	Expect(wops[0]).To(Equal(uint8(recOpStateHash)))
	gops = recGoldenSetWorldId(gops, binary.LittleEndian.Uint32(wops[4:8]))
	if string(gops) != string(wops) {
		Fail(fmt.Sprintf("op stream differs at byte %d (got %d bytes, want %d)", recGoldenFirstDiff(gops, wops), len(gops), len(wops)))
	}
	if string(greg) != string(wreg) {
		Fail(fmt.Sprintf("registry differs at byte %d (got %d bytes, want %d)", recGoldenFirstDiff(greg, wreg), len(greg), len(wreg)))
	}
}

var _ = Describe("ReplayJointsGoldenTest", func() {
	workers := []int{1, 2, 4}

	It("CRunIsSelfConsistent", func() {
		Expect(rjGolden("validate")).To(Equal([]string{"validate 1"}))
		Expect(rjGolden("player")).To(Equal([]string{fmt.Sprintf("player frames %d diverged 0", rjSteps)}))
		Expect(rjGolden("seek")).To(Equal([]string{"seek frame 37", fmt.Sprintf("seek frames %d diverged 0", rjSteps)}))
		Expect(rjGoldenFinal("replay")).To(Equal(rjGoldenFinal("original")))
		Expect(rjGoldenFinal("cseek")).To(Equal(rjGoldenFinal("original")))
		Expect(rjGolden("step")).To(HaveLen(rjSteps))
	})

	It("HeadlessReplayOfCRecordingMatchesCorrectedC", func() {
		for _, w := range workers {
			steps, final := rjHeadless(rjCData(), w)
			rjCompareLines(fmt.Sprintf("steps (%d workers)", w), steps, rjGolden("step"))
			rjCompareLines(fmt.Sprintf("final (%d workers)", w), final, rjGoldenFinal("original"))
		}
	})

	It("PlayerReplayOfCRecordingMatchesCorrectedC", func() {
		for _, w := range workers {
			rjPlayerChecks(rjCData(), w, rjGolden("step"), rjGoldenFinal("original"))
		}
	})

	It("GoRecordingMatchesCorrectedC", func() {
		for _, w := range workers {
			res := rjRun(w, 0)
			rjCompareLines(fmt.Sprintf("Go run steps (%d workers)", w), res.steps, rjGolden("step"))
			rjCompareLines(fmt.Sprintf("Go run final (%d workers)", w), res.final, rjGoldenFinal("original"))
			rjCompareOps(res.data)
		}
	})

	It("GoRecordingReplays", func() {
		res := rjRun(1, 0)
		for _, w := range workers {
			steps, final := rjHeadless(res.data, w)
			rjCompareLines(fmt.Sprintf("headless steps (%d workers)", w), steps, res.steps)
			rjCompareLines(fmt.Sprintf("headless final (%d workers)", w), final, res.final)
			rjPlayerChecks(res.data, w, res.steps, res.final)
		}
	})

	It("MidLifeGoRecordingReplays", func() {
		// The seed snapshot holds every joint type (the filter and motor joints are still alive).
		const startStep = 45
		for _, w := range workers {
			res := rjRun(w, startStep)
			rjCompareLines(fmt.Sprintf("Go run steps (%d workers)", w), res.steps, rjGolden("step"))
			rjPlayerChecks(res.data, w, res.steps[startStep-1:], res.final)
		}
	})
})
