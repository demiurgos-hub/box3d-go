// Fork reference: testdata/golem_post_toi applies only the documented native
// post-TOI contact correction to pinned C. Original upstream fixtures remain intact.
// Go-only test: no upstream counterpart. Steps a world with distance, motor, prismatic and revolute
// joints (motors, limits and springs enabled, parameters changed through the <Kind>Joint* setters
// at steps 60, 120 and 180) and compares every body's transform and velocities, every joint's
// constraint force and torque, linear and angular separation and per-type getters, the awake body
// count and b3HashWorldState bit for bit against the pinned upstream C sources
// (efca0c184d40fd6a82016983f1601c4d23965867) compiled with -ffp-contract=off. See
// testdata/joints_a_golden_gen.c; this file mirrors it line by line. The C output is identical at
// 1 and 4 workers and for the NEON, BOX3D_DISABLE_SIMD and BOX3D_VALIDATE builds; the Go run is
// checked at 1, 2 and 4 workers.
//
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"math"
	"os"
	"strings"

	. "github.com/onsi/ginkgo/v2"
	. "github.com/onsi/gomega"
)

const (
	jointsAGoldenSteps  = 240
	jointsAGoldenStride = 20
)

func jointsAGoldenBits(f float32) uint32 {
	return math.Float32bits(f)
}

func jointsAGoldenCreateBox(worldId WorldID, typ BodyType, p Pos, q Quat, box *BoxHull) BodyID {
	bodyDef := DefaultBodyDef()
	bodyDef.Type = typ
	bodyDef.Position = p
	bodyDef.Rotation = q
	bodyId := worldId.CreateBody(&bodyDef)
	shapeDef := DefaultShapeDef()
	shapeDef.Density = 1.0
	bodyId.CreateHullShape(&shapeDef, &box.Base)
	return bodyId
}

func jointsAGoldenMakeFrame(p Vec3, q Quat) Transform {
	frame := Transform{p, q}
	return frame
}

func jointsAGoldenChangeJoints(step int, joints []JointID) {
	switch step {
	case 60:
		joints[0].DistanceJointSetLength(2.5)

		joints[1].DistanceJointSetLengthRange(0.8, 1.2)
		joints[1].DistanceJointEnableMotor(false)
		joints[1].DistanceJointSetSpringHertz(4.0)
		joints[1].DistanceJointSetSpringDampingRatio(0.5)
		joints[1].DistanceJointSetSpringForceRange(-10.0, 10.0)

		joints[3].MotorJointSetLinearVelocity(Vec3{-0.5, 0.0, 0.25})
		joints[3].MotorJointSetAngularVelocity(Vec3{0.5, -1.0, 0.0})
		joints[3].MotorJointSetMaxVelocityForce(10.0)
		joints[3].MotorJointSetMaxVelocityTorque(2.0)

		joints[4].PrismaticJointSetLimits(1.0, -0.5)
		joints[4].PrismaticJointSetTargetTranslation(-0.25)
		joints[4].PrismaticJointSetMotorSpeed(-1.0)
		joints[4].PrismaticJointSetMaxMotorForce(10.0)

		joints[6].RevoluteJointSetLimits(0.3, -0.3)
		joints[6].RevoluteJointSetTargetAngle(-0.2)
		joints[6].RevoluteJointSetMotorSpeed(-2.0)
		joints[6].RevoluteJointSetMaxMotorTorque(5.0)
		joints[6].RevoluteJointSetSpringHertz(3.0)
		joints[6].RevoluteJointSetSpringDampingRatio(0.25)

	case 120:
		joints[1].DistanceJointEnableLimit(false)
		joints[1].DistanceJointEnableMotor(true)
		joints[1].DistanceJointSetMotorSpeed(-0.75)
		joints[1].DistanceJointSetMaxMotorForce(3.0)

		joints[3].MotorJointSetLinearHertz(4.0)
		joints[3].MotorJointSetLinearDampingRatio(1.0)
		joints[3].MotorJointSetAngularHertz(3.0)
		joints[3].MotorJointSetAngularDampingRatio(0.25)
		joints[3].MotorJointSetMaxSpringForce(25.0)
		joints[3].MotorJointSetMaxSpringTorque(-1.0)

		joints[4].PrismaticJointEnableSpring(false)
		joints[4].PrismaticJointSetSpringHertz(2.0)
		joints[4].PrismaticJointSetSpringDampingRatio(0.75)
		joints[5].PrismaticJointEnableLimit(false)
		joints[5].PrismaticJointEnableMotor(false)

		joints[6].RevoluteJointEnableLimit(false)
		joints[6].RevoluteJointEnableSpring(false)
		joints[8].RevoluteJointEnableMotor(false)

	case 180:
		joints[1].DistanceJointEnableSpring(false)
		joints[1].DistanceJointEnableLimit(true)

		joints[4].PrismaticJointEnableSpring(true)
		joints[4].PrismaticJointEnableMotor(false)
		joints[5].PrismaticJointEnableLimit(true)

		joints[6].RevoluteJointEnableLimit(true)
		joints[6].RevoluteJointEnableMotor(false)
		joints[7].RevoluteJointEnableSpring(true)
		joints[8].RevoluteJointEnableMotor(true)
	}
}

func jointsAGoldenPrintJoint(sb *strings.Builder, step, index int, jointId JointID) {
	b := jointsAGoldenBits
	fmt.Fprintf(sb, "joint %d %d |", step, index)
	worldGoldenVec(sb, jointId.GetConstraintForce())
	worldGoldenVec(sb, jointId.GetConstraintTorque())
	fmt.Fprintf(sb, " %08x %08x", b(jointId.GetLinearSeparation()), b(jointId.GetAngularSeparation()))

	switch jointId.GetType() {
	case DistanceJoint:
		lower, upper := jointId.DistanceJointGetSpringForceRange()
		fmt.Fprintf(sb, " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", b(jointId.DistanceJointGetCurrentLength()),
			b(jointId.DistanceJointGetMotorForce()), b(jointId.DistanceJointGetLength()),
			b(jointId.DistanceJointGetMinLength()), b(jointId.DistanceJointGetMaxLength()),
			b(jointId.DistanceJointGetSpringHertz()), b(jointId.DistanceJointGetSpringDampingRatio()),
			b(jointId.DistanceJointGetMotorSpeed()), b(jointId.DistanceJointGetMaxMotorForce()),
			b(upper-lower), boolInt(jointId.DistanceJointIsSpringEnabled()), boolInt(jointId.DistanceJointIsLimitEnabled()),
			boolInt(jointId.DistanceJointIsMotorEnabled()))

	case MotorJoint:
		fmt.Fprintf(sb, " |")
		worldGoldenVec(sb, jointId.MotorJointGetLinearVelocity())
		worldGoldenVec(sb, jointId.MotorJointGetAngularVelocity())
		fmt.Fprintf(sb, " %08x %08x %08x %08x %08x %08x %08x %08x", b(jointId.MotorJointGetMaxVelocityForce()),
			b(jointId.MotorJointGetMaxVelocityTorque()), b(jointId.MotorJointGetLinearHertz()),
			b(jointId.MotorJointGetLinearDampingRatio()), b(jointId.MotorJointGetAngularHertz()),
			b(jointId.MotorJointGetAngularDampingRatio()), b(jointId.MotorJointGetMaxSpringForce()),
			b(jointId.MotorJointGetMaxSpringTorque()))

	case PrismaticJoint:
		fmt.Fprintf(sb, " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", b(jointId.PrismaticJointGetTranslation()),
			b(jointId.PrismaticJointGetSpeed()), b(jointId.PrismaticJointGetMotorForce()),
			b(jointId.PrismaticJointGetLowerLimit()), b(jointId.PrismaticJointGetUpperLimit()),
			b(jointId.PrismaticJointGetTargetTranslation()), b(jointId.PrismaticJointGetSpringHertz()),
			b(jointId.PrismaticJointGetSpringDampingRatio()), b(jointId.PrismaticJointGetMotorSpeed()),
			b(jointId.PrismaticJointGetMaxMotorForce()), boolInt(jointId.PrismaticJointIsSpringEnabled()),
			boolInt(jointId.PrismaticJointIsLimitEnabled()), boolInt(jointId.PrismaticJointIsMotorEnabled()))

	case RevoluteJoint:
		fmt.Fprintf(sb, " | %08x %08x %08x %08x %08x %08x %08x %08x %08x %d%d%d", b(jointId.RevoluteJointGetAngle()),
			b(jointId.RevoluteJointGetMotorTorque()), b(jointId.RevoluteJointGetLowerLimit()),
			b(jointId.RevoluteJointGetUpperLimit()), b(jointId.RevoluteJointGetTargetAngle()),
			b(jointId.RevoluteJointGetSpringHertz()), b(jointId.RevoluteJointGetSpringDampingRatio()),
			b(jointId.RevoluteJointGetMotorSpeed()), b(jointId.RevoluteJointGetMaxMotorTorque()),
			boolInt(jointId.RevoluteJointIsSpringEnabled()), boolInt(jointId.RevoluteJointIsLimitEnabled()),
			boolInt(jointId.RevoluteJointIsMotorEnabled()))

	default:
	}

	fmt.Fprintf(sb, "\n")
}

func jointsAGoldenRunScene(workerCount int) string {
	var sb strings.Builder

	worldDef := DefaultWorldDef()
	worldDef.WorkerCount = uint32(workerCount)
	worldId := CreateWorld(&worldDef)

	var bodies []BodyID
	var joints []JointID

	groundBox := MakeBoxHull(20.0, 0.5, 20.0)
	box := MakeBoxHull(0.25, 0.25, 0.25)
	plank := MakeBoxHull(0.5, 0.125, 0.25)

	qz := Quat{Vec3{0.0, 0.0, 0.6}, 0.8}
	qx := Quat{Vec3{0.6, 0.0, 0.0}, 0.8}
	qy := Quat{Vec3{0.0, 0.8, 0.0}, 0.6}

	groundId := jointsAGoldenCreateBox(worldId, StaticBody, Pos{0.0, -0.5, 0.0}, QuatIdentity, &groundBox)
	bodies = append(bodies, groundId)

	// 0: rigid distance pendulum
	{
		bodyId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{-6.0, 4.0, 0.0}, QuatIdentity, &box)
		bodyId.SetLinearVelocity(Vec3{0.0, 0.0, 3.0})
		bodies = append(bodies, bodyId)

		def := DefaultDistanceJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA.P = Vec3{-6.0, 6.5, 0.0}
		def.Base.LocalFrameB.P = Vec3{0.25, 0.0, 0.0}
		def.Length = 2.0
		joints = append(joints, worldId.CreateDistanceJoint(&def))
	}

	// 1, 2: hinged box carrying a soft distance joint (spring, limit, motor, spring force range)
	{
		hingeId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{-3.0, 4.0, 0.0}, QuatIdentity, &plank)
		bodies = append(bodies, hingeId)
		bobId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{-2.5, 2.75, 0.5}, qy, &box)
		bodies = append(bodies, bobId)

		def := DefaultDistanceJointDef()
		def.Base.BodyIdA = hingeId
		def.Base.BodyIdB = bobId
		def.Base.LocalFrameA.P = Vec3{0.5, 0.0, 0.0}
		def.Base.LocalFrameB.P = Vec3{0.0, 0.25, 0.0}
		def.Length = 1.0
		def.EnableSpring = true
		def.Hertz = 3.0
		def.DampingRatio = 0.2
		def.LowerSpringForce = -20.0
		def.UpperSpringForce = 20.0
		def.EnableLimit = true
		def.MinLength = 0.6
		def.MaxLength = 1.6
		def.EnableMotor = true
		def.MotorSpeed = 0.5
		def.MaxMotorForce = 2.0
		joints = append(joints, worldId.CreateDistanceJoint(&def))

		hinge := DefaultRevoluteJointDef()
		hinge.Base.BodyIdA = groundId
		hinge.Base.BodyIdB = hingeId
		hinge.Base.LocalFrameA = jointsAGoldenMakeFrame(Vec3{-3.5, 4.5, 0.0}, qx)
		hinge.Base.LocalFrameB = jointsAGoldenMakeFrame(Vec3{-0.5, 0.0, 0.0}, qx)
		joints = append(joints, worldId.CreateRevoluteJoint(&hinge))
	}

	// 3: motor joint chasing a velocity with linear and angular springs
	{
		bodyId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{0.0, 2.0, 0.0}, qz, &plank)
		bodies = append(bodies, bodyId)

		def := DefaultMotorJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA.P = Vec3{0.0, 2.5, 0.0}
		def.LinearVelocity = Vec3{0.5, 0.0, 0.0}
		def.MaxVelocityForce = 20.0
		def.AngularVelocity = Vec3{0.0, 1.0, 0.0}
		def.MaxVelocityTorque = 5.0
		def.LinearHertz = 2.0
		def.LinearDampingRatio = 0.7
		def.MaxSpringForce = 50.0
		def.AngularHertz = 1.5
		def.AngularDampingRatio = 0.5
		def.MaxSpringTorque = 10.0
		joints = append(joints, worldId.CreateMotorJoint(&def))
	}

	// 4: prismatic slider on a rotated axis with spring, limit and motor
	{
		bodyId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{3.0, 1.5, 0.0}, QuatIdentity, &box)
		bodies = append(bodies, bodyId)

		def := DefaultPrismaticJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = jointsAGoldenMakeFrame(Vec3{3.0, 2.0, 0.0}, qz)
		def.Base.LocalFrameB = jointsAGoldenMakeFrame(Vec3{0.0, 0.0, 0.0}, qz)
		def.EnableSpring = true
		def.Hertz = 1.0
		def.DampingRatio = 0.5
		def.TargetTranslation = 0.5
		def.EnableLimit = true
		def.LowerTranslation = -1.0
		def.UpperTranslation = 1.5
		def.EnableMotor = true
		def.MotorSpeed = 2.0
		def.MaxMotorForce = 30.0
		joints = append(joints, worldId.CreatePrismaticJoint(&def))
	}

	// 5: prismatic pair resting on the ground with limit and motor
	{
		bodyIdA := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{6.0, 0.25, 0.0}, QuatIdentity, &box)
		bodies = append(bodies, bodyIdA)
		bodyIdB := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{7.0, 0.25, 0.0}, QuatIdentity, &box)
		bodies = append(bodies, bodyIdB)

		def := DefaultPrismaticJointDef()
		def.Base.BodyIdA = bodyIdA
		def.Base.BodyIdB = bodyIdB
		def.Base.LocalFrameA.P = Vec3{0.25, 0.0, 0.0}
		def.Base.LocalFrameB.P = Vec3{-0.25, 0.0, 0.0}
		def.EnableLimit = true
		def.LowerTranslation = 0.25
		def.UpperTranslation = 1.0
		def.EnableMotor = true
		def.MotorSpeed = -1.0
		def.MaxMotorForce = 5.0
		joints = append(joints, worldId.CreatePrismaticJoint(&def))
	}

	// 6: revolute hinge with spring, limit and motor
	{
		bodyId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{9.5, 3.0, 0.0}, QuatIdentity, &plank)
		bodies = append(bodies, bodyId)

		def := DefaultRevoluteJointDef()
		def.Base.BodyIdA = groundId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA.P = Vec3{9.0, 3.5, 0.0}
		def.Base.LocalFrameB.P = Vec3{-0.5, 0.0, 0.0}
		def.EnableSpring = true
		def.Hertz = 2.0
		def.DampingRatio = 0.1
		def.TargetAngle = 0.3
		def.EnableLimit = true
		def.LowerAngle = -0.5
		def.UpperAngle = 0.8
		def.EnableMotor = true
		def.MotorSpeed = 1.5
		def.MaxMotorTorque = 10.0
		joints = append(joints, worldId.CreateRevoluteJoint(&def))
	}

	// 7: revolute driven by a kinematic body on a tilted axis
	{
		bodyDef := DefaultBodyDef()
		bodyDef.Type = KinematicBody
		bodyDef.Position = Pos{12.0, 2.0, 0.0}
		bodyDef.AngularVelocity = Vec3{0.0, 0.0, 1.0}
		kinematicId := worldId.CreateBody(&bodyDef)
		bodies = append(bodies, kinematicId)

		bodyId := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{13.0, 2.0, 0.0}, QuatIdentity, &plank)
		bodies = append(bodies, bodyId)

		def := DefaultRevoluteJointDef()
		def.Base.BodyIdA = kinematicId
		def.Base.BodyIdB = bodyId
		def.Base.LocalFrameA = jointsAGoldenMakeFrame(Vec3{0.5, 0.0, 0.0}, qx)
		def.Base.LocalFrameB = jointsAGoldenMakeFrame(Vec3{-0.5, 0.0, 0.0}, qx)
		def.Hertz = 1.0
		def.DampingRatio = 0.5
		def.EnableLimit = true
		def.LowerAngle = -1.0
		def.UpperAngle = 1.0
		joints = append(joints, worldId.CreateRevoluteJoint(&def))
	}

	// 8: free falling revolute pair with a motor
	{
		bodyIdA := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{15.0, 5.0, 0.0}, qy, &plank)
		bodies = append(bodies, bodyIdA)
		bodyIdB := jointsAGoldenCreateBox(worldId, DynamicBody, Pos{15.0, 5.0, 1.0}, qy, &plank)
		bodies = append(bodies, bodyIdB)

		def := DefaultRevoluteJointDef()
		def.Base.BodyIdA = bodyIdA
		def.Base.BodyIdB = bodyIdB
		def.Base.LocalFrameA = jointsAGoldenMakeFrame(Vec3{0.0, 0.0, 0.5}, qz)
		def.Base.LocalFrameB = jointsAGoldenMakeFrame(Vec3{0.0, 0.0, -0.5}, qz)
		def.EnableMotor = true
		def.MotorSpeed = 4.0
		def.MaxMotorTorque = 1.0
		joints = append(joints, worldId.CreateRevoluteJoint(&def))
	}

	world := getWorldFromId(worldId)

	for step := 1; step <= jointsAGoldenSteps; step++ {
		jointsAGoldenChangeJoints(step, joints)

		worldId.Step(1.0/60.0, 4)

		if step%jointsAGoldenStride != 0 && step != jointsAGoldenSteps {
			continue
		}

		for i, bodyId := range bodies {
			xf := bodyId.GetTransform()
			v := bodyId.GetLinearVelocity()
			w := bodyId.GetAngularVelocity()
			fmt.Fprintf(&sb, "body %d %d |", step, i)
			worldGoldenVec(&sb, ToVec3(xf.P))
			worldGoldenVec(&sb, xf.Q.V)
			fmt.Fprintf(&sb, " %08x", math.Float32bits(xf.Q.S))
			worldGoldenVec(&sb, v)
			worldGoldenVec(&sb, w)
			fmt.Fprintf(&sb, " %d\n", boolInt(bodyId.IsAwake()))
		}

		for i, jointId := range joints {
			jointsAGoldenPrintJoint(&sb, step, i, jointId)
		}

		fmt.Fprintf(&sb, "world %d | %d %016x\n", step, worldId.GetAwakeBodyCount(), hashWorldState(world))
	}

	worldId.Destroy()

	return sb.String()
}

func jointsAGoldenCompare(got string) {
	data, err := os.ReadFile("testdata/golem_post_toi/joints_a_golden.txt")
	Expect(err).NotTo(HaveOccurred())
	want := strings.Split(strings.TrimRight(string(data), "\n"), "\n")
	lines := strings.Split(strings.TrimRight(got, "\n"), "\n")
	for i := range min(len(want), len(lines)) {
		if want[i] != lines[i] {
			Fail(fmt.Sprintf("first mismatch at line %d:\n got  %s\n want %s", i+1, lines[i], want[i]))
		}
	}
	Expect(len(lines)).To(Equal(len(want)))
}

var _ = Describe("JointsAGoldenTest", func() {
	It("MatchesCorrectedC", func() {
		jointsAGoldenCompare(jointsAGoldenRunScene(1))
	})

	It("MatchesCorrectedCWithWorkers", func() {
		jointsAGoldenCompare(jointsAGoldenRunScene(2))
		jointsAGoldenCompare(jointsAGoldenRunScene(4))
	})
})
