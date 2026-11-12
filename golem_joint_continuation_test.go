package box3d

import (
	"fmt"
	"reflect"
	"strings"
	"testing"
)

func continuationJointSeed(typ JointType) jointSim {
	v, v2 := Vec3{1, 2, 3}, Vec2{4, 5}
	return jointSim{
		typ: typ, bodyIdA: 0, bodyIdB: 1,
		localFrameA: Transform{P: Vec3{.2, .3, .4}, Q: QuatIdentity}, localFrameB: Transform{P: Vec3{-.1, .4, .2}, Q: QuatIdentity},
		constraintHertz: 30, constraintDampingRatio: 10,
		distanceJoint:  distanceJoint{length: 1, hertz: 3, dampingRatio: .7, impulse: 1, lowerImpulse: 2, upperImpulse: 3, motorImpulse: 4},
		motorJoint:     motorJoint{linearHertz: 3, angularHertz: 4, linearDampingRatio: .7, angularDampingRatio: .8, linearVelocityImpulse: v, angularVelocityImpulse: v, linearSpringImpulse: v, angularSpringImpulse: v},
		parallelJoint:  parallelJoint{hertz: 3, dampingRatio: .7, perpImpulse: v2},
		prismaticJoint: prismaticJoint{hertz: 3, dampingRatio: .7, perpImpulse: v2, angularImpulse: v, springImpulse: 1, motorImpulse: 2, lowerImpulse: 3, upperImpulse: 4},
		revoluteJoint:  revoluteJoint{hertz: 3, dampingRatio: .7, linearImpulse: v, perpImpulse: v2, springImpulse: 1, motorImpulse: 2, lowerImpulse: 3, upperImpulse: 4},
		sphericalJoint: sphericalJoint{hertz: 3, dampingRatio: .7, targetRotation: QuatIdentity, linearImpulse: v, springImpulse: v, motorImpulse: v, lowerTwistImpulse: 1, upperTwistImpulse: 2, swingImpulse: 3},
		weldJoint:      weldJoint{linearHertz: 3, angularHertz: 4, linearDampingRatio: .7, angularDampingRatio: .8, linearImpulse: v, angularImpulse: v},
		wheelJoint:     wheelJoint{suspensionHertz: 3, steeringHertz: 4, suspensionDampingRatio: .7, steeringDampingRatio: .8, linearImpulse: v2, angularImpulse: v2, spinImpulse: 1, suspensionSpringImpulse: 2, lowerSuspensionImpulse: 3, upperSuspensionImpulse: 4, steeringSpringImpulse: 5, lowerSteeringImpulse: 6, upperSteeringImpulse: 7},
	}
}

func continuationImpulseValues(j jointSim) []float64 {
	var out []float64
	var visit func(reflect.Value, bool)
	visit = func(v reflect.Value, inImpulse bool) {
		if v.Kind() == reflect.Float32 {
			if inImpulse {
				out = append(out, v.Float())
			}
			return
		}
		if v.Kind() != reflect.Struct {
			return
		}
		for i := 0; i < v.NumField(); i++ {
			visit(v.Field(i), inImpulse || strings.HasSuffix(strings.ToLower(v.Type().Field(i).Name), "impulse"))
		}
	}
	visit(reflect.ValueOf(j), false)
	return out
}

func continuationContext(h float32, staticA bool) (*stepContext, []bodySim) {
	sims := []bodySim{
		{transform: WorldTransform{Q: QuatIdentity}, invMass: 1, invInertiaWorld: Mat3Identity, invInertiaLocal: Mat3Identity},
		{transform: WorldTransform{Q: QuatIdentity}, invMass: .5, invInertiaWorld: Mat3Identity, invInertiaLocal: Mat3Identity},
	}
	states := []bodyState{identityBodyState, identityBodyState}
	states[0].flags = dynamicFlag
	states[1].flags = dynamicFlag
	if staticA {
		sims[0].invMass = 0
		sims[0].invInertiaWorld = Mat3Zero
		states[0].flags = 0
		sims[1].invMass = 1
	}
	ctx := &stepContext{world: &world{}, h: h, inv_h: 1 / h, dt: h, inv_dt: 1 / h, states: states, sims: sims}
	ctx.localJointBodies = func(id int32) (*bodySim, int32) {
		if id == 0 && staticA {
			return &sims[id], NullIndex
		}
		return &sims[id], id
	}
	return ctx, sims
}

func TestLocalJointPreparationUsesLocalPosesAndRetainsEveryFamily(t *testing.T) {
	for _, typ := range []JointType{DistanceJoint, MotorJoint, ParallelJoint, PrismaticJoint, RevoluteJoint, SphericalJoint, WeldJoint, WheelJoint, FilterJoint} {
		t.Run(fmt.Sprint(typ), func(t *testing.T) {
			ctx, sims := continuationContext(.003, false)
			ctx.retainJointImpulses = true
			ctx.jointBudgetH = .0125
			sims[0].center = Pos{X: 10, Y: 2, Z: -3}
			sims[1].center = Pos{X: 11, Y: 3, Z: -2}
			sims[0].transform.Q = MakeQuatFromAxisAngle(Vec3AxisY, .7)
			sims[1].transform.Q = MakeQuatFromAxisAngle(Vec3AxisX, -.3)
			sims[0].localCenter = Vec3{.1, .2, -.1}
			sims[1].localCenter = Vec3{-.3, .1, .2}
			sims[0].invInertiaWorld = Matrix3{Cx: Vec3{2, 0, 0}, Cy: Vec3{0, 3, 0}, Cz: Vec3{0, 0, 4}}
			beforeStates := append([]bodyState(nil), ctx.states...)
			j := continuationJointSeed(typ)
			impulses := continuationImpulseValues(j)
			prepareJoint(&j, ctx)
			if !reflect.DeepEqual(impulses, continuationImpulseValues(j)) {
				t.Fatal("preparation reset retained impulses")
			}
			if !reflect.DeepEqual(beforeStates, ctx.states) {
				t.Fatal("preparation reapplied impulses")
			}
			if typ == FilterJoint {
				return
			}
			if j.invMassA != sims[0].invMass || j.invMassB != sims[1].invMass || j.invIA != sims[0].invInertiaWorld {
				t.Fatal("used original-world mass/inertia")
			}
			wantDelta := SubPos(sims[1].center, sims[0].center)
			wantA := RotateVector(sims[0].transform.Q, Sub(j.localFrameA.P, sims[0].localCenter))
			var delta, anchor Vec3
			switch typ {
			case DistanceJoint:
				delta, anchor = j.distanceJoint.deltaCenter, j.distanceJoint.anchorA
			case MotorJoint:
				delta, anchor = j.motorJoint.deltaCenter, j.motorJoint.frameA.P
			case PrismaticJoint:
				delta, anchor = j.prismaticJoint.deltaCenter, j.prismaticJoint.frameA.P
			case RevoluteJoint:
				delta, anchor = j.revoluteJoint.deltaCenter, j.revoluteJoint.frameA.P
			case SphericalJoint:
				delta, anchor = j.sphericalJoint.deltaCenter, j.sphericalJoint.frameA.P
			case WeldJoint:
				delta, anchor = j.weldJoint.deltaCenter, j.weldJoint.frameA.P
			case WheelJoint:
				delta, anchor = j.wheelJoint.deltaCenter, j.wheelJoint.frameA.P
			case ParallelJoint:
				if j.parallelJoint.quatA != MulQuat(sims[0].transform.Q, j.localFrameA.Q) {
					t.Fatal("stale parallel orientation")
				}
				return
			}
			if delta != wantDelta || anchor != wantA {
				t.Fatalf("stale local geometry delta=%v anchor=%v", delta, anchor)
			}
		})
	}
}

func TestLocalJointPreparationOrdinaryArithmetic(t *testing.T) {
	for _, typ := range []JointType{DistanceJoint, MotorJoint, ParallelJoint, PrismaticJoint, RevoluteJoint, SphericalJoint, WeldJoint, WheelJoint} {
		ctx, sims := continuationContext(.0125, false)
		ctx.enableWarmStarting = true
		ctx.world.bodies = []body{{id: 0, setIndex: awakeSet, localIndex: 0}, {id: 1, setIndex: awakeSet, localIndex: 1}}
		ctx.world.solverSets = make([]solverSet, awakeSet+1)
		ctx.world.solverSets[awakeSet].bodySims = sims
		j, k := continuationJointSeed(typ), continuationJointSeed(typ)
		prepareJoint(&j, ctx)
		ctx.localJointBodies = nil
		prepareJoint(&k, ctx)
		if j != k {
			t.Fatalf("type %v ordinary and local lookup changed arithmetic", typ)
		}
	}
}

func TestLocalJointMotorAndHingeUseOneIntervalBudget(t *testing.T) {
	for _, typ := range []JointType{MotorJoint, RevoluteJoint} {
		t.Run(fmt.Sprint(typ), func(t *testing.T) {
			ctx, _ := continuationContext(.05, true)
			j := jointSim{typ: typ, bodyIdA: 0, bodyIdB: 1, localFrameA: Transform{Q: QuatIdentity}, localFrameB: Transform{Q: QuatIdentity}}
			if typ == MotorJoint {
				j.motorJoint.linearVelocity = Vec3{X: 100}
				j.motorJoint.maxVelocityForce = 10
			} else {
				j.revoluteJoint.enableMotor = true
				j.revoluteJoint.motorSpeed = 100
				j.revoluteJoint.maxMotorTorque = 20
			}
			prepareJoint(&j, ctx)
			solveJoint(&j, ctx, false)
			expected := ctx.states[1]
			if typ == MotorJoint && AbsFloat(expected.linearVelocity.X-.5) > 1e-6 || typ == RevoluteJoint && AbsFloat(expected.angularVelocity.Z-1) > 1e-6 {
				t.Fatalf("initial budget not exercised: %+v", expected)
			}
			ctx.retainJointImpulses = true
			ctx.jointBudgetH = .05
			for n := 0; n < 20; n++ {
				ctx.h = .04 / float32(n+1)
				ctx.inv_h = 1 / ctx.h
				prepareJoint(&j, ctx)
				solveJoint(&j, ctx, false)
				if ctx.states[1] != expected {
					t.Fatalf("event%d duplicated/shrank budget: got %+v want %+v", n, ctx.states[1], expected)
				}
			}
		})
	}
}

func TestLocalJointSpringEquationDoesNotRestart(t *testing.T) {
	ctx, sims := continuationContext(.05, true)
	sims[1].center = Pos{X: 1}
	j := jointSim{typ: MotorJoint, bodyIdA: 0, bodyIdB: 1, localFrameA: Transform{Q: QuatIdentity}, localFrameB: Transform{Q: QuatIdentity}, motorJoint: motorJoint{linearHertz: 2, linearDampingRatio: .7, maxSpringForce: 1000}}
	prepareJoint(&j, ctx)
	for range 8 {
		solveJoint(&j, ctx, false)
	}
	expected := ctx.states[1].linearVelocity
	if AbsFloat(expected.X) < .1 {
		t.Fatal("spring was not exercised")
	}
	ctx.retainJointImpulses = true
	ctx.jointBudgetH = .05
	for n := 0; n < 20; n++ {
		ctx.h = .04 / float32(n+1)
		ctx.inv_h = 1 / ctx.h
		prepareJoint(&j, ctx)
		solveJoint(&j, ctx, false)
		if Distance(ctx.states[1].linearVelocity, expected) > 1e-5 {
			t.Fatalf("event%d restarted spring: %v vs %v", n, ctx.states[1].linearVelocity, expected)
		}
	}
}

func TestLocalJointWeldTransfersImpactWithoutWarmStart(t *testing.T) {
	ctx, _ := continuationContext(.01, false)
	j := jointSim{typ: WeldJoint, bodyIdA: 0, bodyIdB: 1, localFrameA: Transform{Q: QuatIdentity}, localFrameB: Transform{Q: QuatIdentity}}
	ctx.states[0].linearVelocity = Vec3{X: 3}
	prepareJoint(&j, ctx)
	solveJoint(&j, ctx, false)
	if AbsFloat(ctx.states[0].linearVelocity.X-1) > 1e-6 || AbsFloat(ctx.states[1].linearVelocity.X-1) > 1e-6 {
		t.Fatalf("mass1:2 momentum transfer: %+v", ctx.states)
	}
	before := append([]bodyState(nil), ctx.states...)
	oldImpulse := j.weldJoint.linearImpulse
	ctx.retainJointImpulses = true
	ctx.jointBudgetH = .01
	ctx.h = .004
	ctx.inv_h = 250
	prepareJoint(&j, ctx)
	solveJoint(&j, ctx, false)
	if !reflect.DeepEqual(before, ctx.states) || j.weldJoint.linearImpulse != oldImpulse {
		t.Fatal("continuation reapplied passive weld impulse")
	}
}
