// Port of src/joint.h and src/joint.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "fmt"

// ---- joint.h ----

// PORT: the prototypes of joint.h are the functions of the joint.c section below and of the
// per-type files (<kind>_joint.go); the static inline b3SolvePointConstraint sits at the end of this
// section.

// jointEdge is b3JointEdge.
// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
type jointEdge struct {
	bodyId  int32
	prevKey int32
	nextKey int32
}

// joint is b3Joint: map from b3JointId to b3Joint in the solver sets
type joint struct {
	userData any

	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex int32

	// index into the constraint graph color array, may be B3_NULL_INDEX for sleeping/disabled joints
	// B3_NULL_INDEX when slot is free
	colorIndex int32

	// joint index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex int32

	edges [2]jointEdge

	jointId  int32
	islandId int32

	// Index into the island's joints array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	drawScale float32

	typ JointType // PORT: type is a Go keyword

	// This is monotonically advanced when a body is allocated in this slot
	// Used to check for invalid b3JointId
	generation uint16

	collideConnected bool
}

// distanceJoint is b3DistanceJoint.
type distanceJoint struct {
	length           float32
	hertz            float32
	dampingRatio     float32
	lowerSpringForce float32
	upperSpringForce float32
	minLength        float32
	maxLength        float32

	maxMotorForce float32
	motorSpeed    float32

	impulse      float32
	lowerImpulse float32
	upperImpulse float32
	motorImpulse float32

	indexA           int32
	indexB           int32
	anchorA          Vec3
	anchorB          Vec3
	deltaCenter      Vec3
	distanceSoftness softness
	axialMass        float32

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// motorJoint is b3MotorJoint.
type motorJoint struct {
	linearVelocity      Vec3
	angularVelocity     Vec3
	maxVelocityForce    float32
	maxVelocityTorque   float32
	linearHertz         float32
	linearDampingRatio  float32
	maxSpringForce      float32
	angularHertz        float32
	angularDampingRatio float32
	maxSpringTorque     float32

	linearVelocityImpulse  Vec3
	angularVelocityImpulse Vec3
	linearSpringImpulse    Vec3
	angularSpringImpulse   Vec3

	linearSpring  softness
	angularSpring softness

	indexA      int32
	indexB      int32
	frameA      Transform
	frameB      Transform
	deltaCenter Vec3
	angularMass Matrix3
}

// parallelJoint is b3ParallelJoint.
type parallelJoint struct {
	hertz        float32
	dampingRatio float32
	maxTorque    float32

	perpImpulse Vec2
	perpAxisX   Vec3
	perpAxisY   Vec3

	quatA    Quat
	quatB    Quat
	indexA   int32
	indexB   int32
	softness softness
}

// prismaticJoint is b3PrismaticJoint.
type prismaticJoint struct {
	perpImpulse       Vec2
	angularImpulse    Vec3
	springImpulse     float32
	motorImpulse      float32
	lowerImpulse      float32
	upperImpulse      float32
	hertz             float32
	dampingRatio      float32
	maxMotorForce     float32
	motorSpeed        float32
	targetTranslation float32
	lowerTranslation  float32
	upperTranslation  float32

	indexA         int32
	indexB         int32
	frameA         Transform
	frameB         Transform
	jointAxis      Vec3
	perpAxisY      Vec3
	perpAxisZ      Vec3
	deltaCenter    Vec3
	deltaAngle     float32
	rotationMass   Matrix3
	springSoftness softness

	enableSpring bool
	enableLimit  bool
	enableMotor  bool
}

// revoluteJoint is b3RevoluteJoint.
type revoluteJoint struct {
	linearImpulse  Vec3
	perpImpulse    Vec2
	springImpulse  float32
	motorImpulse   float32
	lowerImpulse   float32
	upperImpulse   float32
	hertz          float32
	dampingRatio   float32
	maxMotorTorque float32
	motorSpeed     float32
	targetAngle    float32
	lowerAngle     float32
	upperAngle     float32

	indexA         int32
	indexB         int32
	frameA         Transform
	frameB         Transform
	rotationAxisZ  Vec3
	perpAxisX      Vec3
	perpAxisY      Vec3
	deltaCenter    Vec3
	deltaAngle     float32
	axialMass      float32
	springSoftness softness

	enableSpring bool
	enableMotor  bool
	enableLimit  bool
}

// sphericalJoint is b3SphericalJoint.
type sphericalJoint struct {
	linearImpulse     Vec3
	springImpulse     Vec3
	motorImpulse      Vec3
	lowerTwistImpulse float32
	upperTwistImpulse float32
	swingImpulse      float32
	hertz             float32
	dampingRatio      float32
	maxMotorTorque    float32
	motorVelocity     Vec3
	lowerTwistAngle   float32
	upperTwistAngle   float32
	coneAngle         float32
	targetRotation    Quat

	indexA        int32
	indexB        int32
	frameA        Transform
	frameB        Transform
	deltaCenter   Vec3
	swingAxis     Vec3
	twistJacobian Vec3

	rotationMass   Matrix3
	swingMass      float32
	twistMass      float32
	springSoftness softness

	enableSpring     bool
	enableMotor      bool
	enableConeLimit  bool
	enableTwistLimit bool
}

// weldJoint is b3WeldJoint.
type weldJoint struct {
	linearHertz         float32
	linearDampingRatio  float32
	angularHertz        float32
	angularDampingRatio float32

	linearSpring   softness
	angularSpring  softness
	linearImpulse  Vec3
	angularImpulse Vec3

	indexA      int32
	indexB      int32
	frameA      Transform
	frameB      Transform
	deltaCenter Vec3

	angularMass Matrix3
}

// wheelJoint is b3WheelJoint.
type wheelJoint struct {
	linearImpulse           Vec2
	angularImpulse          Vec2
	spinImpulse             float32
	maxSpinTorque           float32
	spinSpeed               float32
	suspensionSpringImpulse float32
	lowerSuspensionImpulse  float32
	upperSuspensionImpulse  float32
	lowerSuspensionLimit    float32
	upperSuspensionLimit    float32
	suspensionHertz         float32
	suspensionDampingRatio  float32
	steeringSpringImpulse   float32
	lowerSteeringImpulse    float32
	upperSteeringImpulse    float32
	lowerSteeringLimit      float32
	upperSteeringLimit      float32
	targetSteeringAngle     float32
	maxSteeringTorque       float32
	steeringHertz           float32
	steeringDampingRatio    float32

	indexA             int32
	indexB             int32
	frameA             Transform
	frameB             Transform
	deltaCenter        Vec3
	spinMass           float32
	suspensionMass     float32
	steeringMass       float32
	suspensionSoftness softness
	steeringSoftness   softness

	enableSpinMotor        bool
	enableSuspensionSpring bool
	enableSuspensionLimit  bool
	enableSteering         bool
	enableSteeringLimit    bool
	enableSteeringMotor    bool
}

// jointSim is b3JointSim.
// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type jointSim struct {
	jointId int32

	bodyIdA int32
	bodyIdB int32

	typ JointType // PORT: type is a Go keyword

	// Joint frames local to body origin
	localFrameA Transform
	localFrameB Transform

	invMassA, invMassB float32
	invIA, invIB       Matrix3

	constraintHertz        float32
	constraintDampingRatio float32

	constraintSoftness softness

	forceThreshold  float32
	torqueThreshold float32

	fixedRotation bool

	// PORT: anonymous union of the joint types. All members are present; typ says which one is
	// valid.
	distanceJoint  distanceJoint
	motorJoint     motorJoint
	parallelJoint  parallelJoint
	revoluteJoint  revoluteJoint
	sphericalJoint sphericalJoint
	prismaticJoint prismaticJoint
	weldJoint      weldJoint
	wheelJoint     wheelJoint
}

// solvePointConstraint is b3SolvePointConstraint.
// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
func solvePointConstraint(mA, mB float32, iA, iB Matrix3, rA, rB, rhs Vec3) Vec3 {
	axy := float32(rA.Z*iA.Cy.Y) - float32(rA.Y*iA.Cz.Y)
	axz := float32(rA.Z*iA.Cy.Z) - float32(rA.Y*iA.Cz.Z)
	ay := Sub(MulSV(rA.X, iA.Cz), MulSV(rA.Z, iA.Cx))
	az := Sub(MulSV(rA.Y, iA.Cx), MulSV(rA.X, iA.Cy))

	bxy := float32(rB.Z*iB.Cy.Y) - float32(rB.Y*iB.Cz.Y)
	bxz := float32(rB.Z*iB.Cy.Z) - float32(rB.Y*iB.Cz.Z)
	by := Sub(MulSV(rB.X, iB.Cz), MulSV(rB.Z, iB.Cx))
	bz := Sub(MulSV(rB.Y, iB.Cx), MulSV(rB.X, iB.Cy))

	m := mA + mB
	kxx := (float32(axy*rA.Z) - float32(axz*rA.Y)) + (float32(bxy*rB.Z) - float32(bxz*rB.Y)) + m
	kyy := (float32(ay.Z*rA.X) - float32(ay.X*rA.Z)) + (float32(by.Z*rB.X) - float32(by.X*rB.Z)) + m
	kzz := (float32(az.X*rA.Y) - float32(az.Y*rA.X)) + (float32(bz.X*rB.Y) - float32(bz.Y*rB.X)) + m
	kxy := (float32(ay.Y*rA.Z) - float32(ay.Z*rA.Y)) + (float32(by.Y*rB.Z) - float32(by.Z*rB.Y))
	kxz := (float32(az.Y*rA.Z) - float32(az.Z*rA.Y)) + (float32(bz.Y*rB.Z) - float32(bz.Z*rB.Y))
	kyz := (float32(az.Z*rA.X) - float32(az.X*rA.Z)) + (float32(bz.Z*rB.X) - float32(bz.X*rB.Z))

	cxx := float32(kyy*kzz) - float32(kyz*kyz)
	cyy := float32(kxx*kzz) - float32(kxz*kxz)
	czz := float32(kxx*kyy) - float32(kxy*kxy)
	cxy := float32(kxz*kyz) - float32(kxy*kzz)
	cxz := float32(kxy*kyz) - float32(kyy*kxz)
	cyz := float32(kxy*kxz) - float32(kxx*kyz)

	det := float32(kxx*cxx) + float32(kxy*cxy) + float32(kxz*cxz)
	if AbsFloat(det) > 1000.0*fltMin {
		invDet := 1.0 / det
		b := Vec3{
			invDet * (float32(cxx*rhs.X) + float32(cxy*rhs.Y) + float32(cxz*rhs.Z)),
			invDet * (float32(cxy*rhs.X) + float32(cyy*rhs.Y) + float32(cyz*rhs.Z)),
			invDet * (float32(cxz*rhs.X) + float32(cyz*rhs.Y) + float32(czz*rhs.Z)),
		}
		return b
	}

	return Vec3Zero
}

// ---- joint.c ----

// PORT: the b3Joint_* functions of box3d.h are methods on JointID, b3Create<Kind>Joint is a method
// on WorldID taking *<Kind>JointDef, b3DestroyJoint is JointID.Destroy. The per-type functions this
// file dispatches to live in <kind>_joint.go. B3_REC / B3_REC_CREATE are expanded at the call site
// (see recording.go).

// defaultJointDef is b3DefaultJointDef.
func defaultJointDef() JointDef {
	def := JointDef{}
	def.LocalFrameA.Q = QuatIdentity
	def.LocalFrameB.Q = QuatIdentity
	def.ForceThreshold = fltMax
	def.TorqueThreshold = fltMax
	def.ConstraintHertz = 60.0
	def.ConstraintDampingRatio = 2.0
	def.DrawScale = GetLengthUnitsPerMeter()
	def.internalValue = secretCookie
	return def
}

// DefaultParallelJointDef is b3DefaultParallelJointDef.
// Use this to initialize your joint definition
func DefaultParallelJointDef() ParallelJointDef {
	def := ParallelJointDef{}
	def.Base = defaultJointDef()
	def.Hertz = 1.0
	def.DampingRatio = 1.0
	def.MaxTorque = fltMax
	return def
}

// DefaultDistanceJointDef is b3DefaultDistanceJointDef.
// Use this to initialize your joint definition
func DefaultDistanceJointDef() DistanceJointDef {
	def := DistanceJointDef{}
	def.Base = defaultJointDef()
	def.LowerSpringForce = -fltMax
	def.UpperSpringForce = fltMax
	def.Length = 1.0
	def.MaxLength = Huge()
	return def
}

// DefaultMotorJointDef is b3DefaultMotorJointDef.
// Use this to initialize your joint definition
func DefaultMotorJointDef() MotorJointDef {
	def := MotorJointDef{}
	def.Base = defaultJointDef()
	return def
}

// DefaultFilterJointDef is b3DefaultFilterJointDef.
// Use this to initialize your joint definition
func DefaultFilterJointDef() FilterJointDef {
	def := FilterJointDef{}
	def.Base = defaultJointDef()
	return def
}

// DefaultPrismaticJointDef is b3DefaultPrismaticJointDef.
// Use this to initialize your joint definition
func DefaultPrismaticJointDef() PrismaticJointDef {
	def := PrismaticJointDef{}
	def.Base = defaultJointDef()
	return def
}

// DefaultRevoluteJointDef is b3DefaultRevoluteJointDef.
// Use this to initialize your joint definition.
func DefaultRevoluteJointDef() RevoluteJointDef {
	def := RevoluteJointDef{}
	def.Base = defaultJointDef()
	return def
}

// DefaultSphericalJointDef is b3DefaultSphericalJointDef.
// Use this to initialize your joint definition.
func DefaultSphericalJointDef() SphericalJointDef {
	def := SphericalJointDef{}
	def.Base = defaultJointDef()
	def.TargetRotation = QuatIdentity
	return def
}

// DefaultWeldJointDef is b3DefaultWeldJointDef.
// Use this to initialize your joint definition
func DefaultWeldJointDef() WeldJointDef {
	def := WeldJointDef{}
	def.Base = defaultJointDef()
	return def
}

// DefaultWheelJointDef is b3DefaultWheelJointDef.
// Use this to initialize your joint definition
func DefaultWheelJointDef() WheelJointDef {
	def := WheelJointDef{}
	def.Base = defaultJointDef()
	def.EnableSuspensionSpring = true
	def.SuspensionHertz = 1.0
	def.SuspensionDampingRatio = 0.7
	def.SteeringHertz = 1.0
	def.SteeringDampingRatio = 0.7
	return def
}

// DefaultExplosionDef is b3DefaultExplosionDef.
// Use this to initialize your explosion definition
func DefaultExplosionDef() ExplosionDef {
	def := ExplosionDef{}
	def.MaskBits = DefaultMaskBits
	return def
}

// getJointFullId is b3GetJointFullId.
func getJointFullId(world *world, jointId JointID) *joint {
	id := int(jointId.Index1 - 1)
	joint := arrayGet(world.joints, id)
	assert(int(joint.jointId) == id && joint.generation == jointId.Generation,
		"joint->jointId == id && joint->generation == jointId.generation")
	return joint
}

// getJointSim is b3GetJointSim.
func getJointSim(world *world, joint *joint) *jointSim {
	if joint.setIndex == awakeSet {
		assert(0 <= joint.colorIndex && joint.colorIndex < GraphColorCount,
			"0 <= joint->colorIndex && joint->colorIndex < B3_GRAPH_COLOR_COUNT")
		color := &world.constraintGraph.colors[joint.colorIndex]
		return arrayGet(color.jointSims, int(joint.localIndex))
	}

	set := arrayGet(world.solverSets, int(joint.setIndex))
	return arrayGet(set.jointSims, int(joint.localIndex))
}

// getJointSimCheckType is b3GetJointSimCheckType.
func getJointSimCheckType(jointId JointID, typ JointType) *jointSim {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	assert(joint.typ == typ, "joint->type == type")
	jointSim := getJointSim(world, joint)
	assert(jointSim.typ == typ, "jointSim->type == type")
	return jointSim
}

// destroyContactsBetweenBodies is b3DestroyContactsBetweenBodies.
func destroyContactsBetweenBodies(world *world, bodyA *body, bodyB *body) {
	var contactKey int32
	var otherBodyId int32

	// use the smaller of the two contact lists
	if bodyA.contactCount < bodyB.contactCount {
		contactKey = bodyA.headContactKey
		otherBodyId = bodyB.id
	} else {
		contactKey = bodyB.headContactKey
		otherBodyId = bodyA.id
	}

	// no need to wake bodies when a joint removes collision between them
	wakeBodies := false

	// destroy the contacts
	for contactKey != NullIndex {
		contactId := contactKey >> 1
		edgeIndex := contactKey & 1

		contact := arrayGet(world.contacts, int(contactId))
		contactKey = contact.edges[edgeIndex].nextKey

		otherEdgeIndex := edgeIndex ^ 1
		if contact.edges[otherEdgeIndex].bodyId == otherBodyId {
			// Careful, this removes the contact from the current doubly linked list
			destroyContact(world, contact, wakeBodies)
		}
	}

	validateSolverSets(world)
}

// jointPair is b3JointPair.
type jointPair struct {
	joint    *joint
	jointSim *jointSim
}

// createJoint is b3CreateJoint.
func createJoint(world *world, def *JointDef, typ JointType) jointPair {
	assert(IsValidTransform(def.LocalFrameA), "b3IsValidTransform( def->localFrameA )")
	assert(IsValidTransform(def.LocalFrameB), "b3IsValidTransform( def->localFrameB )")
	assert(world.worldId == def.BodyIdA.World0, "world->worldId == def->bodyIdA.world0")
	assert(world.worldId == def.BodyIdB.World0, "world->worldId == def->bodyIdB.world0")
	assert((def.BodyIdA == def.BodyIdB) == false, "B3_ID_EQUALS( def->bodyIdA, def->bodyIdB ) == false")

	bodyA := getBodyFullId(world, def.BodyIdA)
	bodyB := getBodyFullId(world, def.BodyIdB)

	// If the joint prevents collisions, then destroy all contacts between attached bodies.
	// Do this first so world arrays are not disturbed in the following code.
	if def.CollideConnected == false {
		destroyContactsBetweenBodies(world, bodyA, bodyB)
	}

	bodyIdA := bodyA.id
	bodyIdB := bodyB.id
	maxSetIndex := MaxInt(int(bodyA.setIndex), int(bodyB.setIndex))

	// Create joint id and joint
	jointId := allocId(&world.jointIdPool)
	if jointId == len(world.joints) {
		arrayPush(&world.joints, joint{})
	}

	joint := arrayGet(world.joints, jointId)
	joint.jointId = int32(jointId)
	joint.userData = def.UserData
	joint.generation += 1
	joint.setIndex = NullIndex
	joint.colorIndex = NullIndex
	joint.localIndex = NullIndex
	joint.islandId = NullIndex
	joint.islandIndex = NullIndex
	joint.drawScale = def.DrawScale
	joint.typ = typ
	joint.collideConnected = def.CollideConnected

	// Doubly linked list on bodyA
	joint.edges[0].bodyId = bodyIdA
	joint.edges[0].prevKey = NullIndex
	joint.edges[0].nextKey = bodyA.headJointKey

	keyA := int32(jointId<<1) | 0
	if bodyA.headJointKey != NullIndex {
		jointA := arrayGet(world.joints, int(bodyA.headJointKey>>1))
		edgeA := &jointA.edges[bodyA.headJointKey&1]
		edgeA.prevKey = keyA
	}
	bodyA.headJointKey = keyA
	bodyA.jointCount += 1

	// Doubly linked list on bodyB
	joint.edges[1].bodyId = bodyIdB
	joint.edges[1].prevKey = NullIndex
	joint.edges[1].nextKey = bodyB.headJointKey

	keyB := int32(jointId<<1) | 1
	if bodyB.headJointKey != NullIndex {
		jointB := arrayGet(world.joints, int(bodyB.headJointKey>>1))
		edgeB := &jointB.edges[bodyB.headJointKey&1]
		edgeB.prevKey = keyB
	}
	bodyB.headJointKey = keyB
	bodyB.jointCount += 1

	var jointSim *jointSim

	if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
		// if either body is disabled, create in disabled set
		set := arrayGet(world.solverSets, disabledSet)
		joint.setIndex = disabledSet
		joint.localIndex = int32(len(set.jointSims))

		// PORT: the memset to zero is implied: arrayEmplace returns a zeroed element.
		jointSim = arrayEmplace(&set.jointSims)

		jointSim.jointId = int32(jointId)
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else if bodyA.typ != DynamicBody && bodyB.typ != DynamicBody {
		// joint is not attached to a dynamic body
		set := arrayGet(world.solverSets, staticSet)
		joint.setIndex = staticSet
		joint.localIndex = int32(len(set.jointSims))

		// PORT: the memset to zero is implied: arrayEmplace returns a zeroed element.
		jointSim = arrayEmplace(&set.jointSims)

		jointSim.jointId = int32(jointId)
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else if bodyA.setIndex == awakeSet || bodyB.setIndex == awakeSet {
		// if either body is sleeping, wake it
		if maxSetIndex >= firstSleepingSet {
			wakeSolverSet(world, maxSetIndex)
		}

		joint.setIndex = awakeSet

		jointSim = createJointInGraph(world, joint)
		jointSim.jointId = int32(jointId)
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB
	} else {
		// joint connected between sleeping and/or static bodies
		assert(bodyA.setIndex >= firstSleepingSet || bodyB.setIndex >= firstSleepingSet,
			"bodyA->setIndex >= b3_firstSleepingSet || bodyB->setIndex >= b3_firstSleepingSet")
		assert(bodyA.setIndex != staticSet || bodyB.setIndex != staticSet,
			"bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet")

		// joint should go into the sleeping set (not static set)
		setIndex := maxSetIndex

		set := arrayGet(world.solverSets, setIndex)
		joint.setIndex = int32(setIndex)
		joint.localIndex = int32(len(set.jointSims))

		// PORT: the memset to zero is implied: arrayEmplace returns a zeroed element.
		jointSim = arrayEmplace(&set.jointSims)

		jointSim.jointId = int32(jointId)
		jointSim.bodyIdA = bodyIdA
		jointSim.bodyIdB = bodyIdB

		if bodyA.setIndex != bodyB.setIndex && bodyA.setIndex >= firstSleepingSet &&
			bodyB.setIndex >= firstSleepingSet {
			// merge sleeping sets
			mergeSolverSets(world, int(bodyA.setIndex), int(bodyB.setIndex))
			assert(bodyA.setIndex == bodyB.setIndex, "bodyA->setIndex == bodyB->setIndex")

			// fix potentially invalid set index
			setIndex = int(bodyA.setIndex)

			mergedSet := arrayGet(world.solverSets, setIndex)

			// Careful! The joint sim pointer was orphaned by the set merge.
			jointSim = arrayGet(mergedSet.jointSims, int(joint.localIndex))
		}

		assert(int(joint.setIndex) == setIndex, "joint->setIndex == setIndex")
	}

	jointSim.localFrameA = def.LocalFrameA
	jointSim.localFrameB = def.LocalFrameB
	jointSim.typ = typ
	jointSim.constraintHertz = def.ConstraintHertz
	jointSim.constraintDampingRatio = def.ConstraintDampingRatio
	jointSim.constraintSoftness = softness{
		biasRate:     0.0,
		massScale:    1.0,
		impulseScale: 0.0,
	}

	assert(IsValidFloat(def.ForceThreshold) && def.ForceThreshold >= 0.0,
		"b3IsValidFloat( def->forceThreshold ) && def->forceThreshold >= 0.0f")
	assert(IsValidFloat(def.TorqueThreshold) && def.TorqueThreshold >= 0.0,
		"b3IsValidFloat( def->torqueThreshold ) && def->torqueThreshold >= 0.0f")

	jointSim.forceThreshold = def.ForceThreshold
	jointSim.torqueThreshold = def.TorqueThreshold

	assert(int(jointSim.jointId) == jointId, "jointSim->jointId == jointId")
	assert(jointSim.bodyIdA == bodyIdA, "jointSim->bodyIdA == bodyIdA")
	assert(jointSim.bodyIdB == bodyIdB, "jointSim->bodyIdB == bodyIdB")

	if joint.setIndex > disabledSet {
		// Add edge to island graph
		linkJoint(world, joint)
	}

	validateSolverSets(world)

	return jointPair{joint, jointSim}
}

// SetConstraintTuning is b3Joint_SetConstraintTuning.
// Set the joint constraint tuning. Advanced feature.
// hertz is the stiffness in Hertz (cycles per second), dampingRatio the non-dimensional damping
// ratio (one for critical damping).
func (jointId JointID) SetConstraintTuning(hertz, dampingRatio float32) {
	assert(IsValidFloat(hertz) && hertz >= 0.0, "b3IsValidFloat( hertz ) && hertz >= 0.0f")
	assert(IsValidFloat(dampingRatio) && dampingRatio >= 0.0, "b3IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f")

	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsJointSetConstraintTuning{jointId, hertz, dampingRatio}
		recWriteJointSetConstraintTuning(world.recording, &recArgs)
	}
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	base.constraintHertz = hertz
	base.constraintDampingRatio = dampingRatio
}

// GetConstraintTuning is b3Joint_GetConstraintTuning.
// Get the joint constraint tuning. Advanced feature.
// PORT: the hertz and dampingRatio out-parameters are the return values.
func (jointId JointID) GetConstraintTuning() (hertz, dampingRatio float32) {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	hertz = base.constraintHertz
	dampingRatio = base.constraintDampingRatio
	return hertz, dampingRatio
}

// SetForceThreshold is b3Joint_SetForceThreshold.
// Set the force threshold for joint events (Newtons)
func (jointId JointID) SetForceThreshold(threshold float32) {
	assert(IsValidFloat(threshold) && threshold >= 0.0, "b3IsValidFloat( threshold ) && threshold >= 0.0f")

	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsJointSetForceThreshold{jointId, threshold}
		recWriteJointSetForceThreshold(world.recording, &recArgs)
	}
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	base.forceThreshold = threshold
}

// GetForceThreshold is b3Joint_GetForceThreshold.
// Get the force threshold for joint events (Newtons)
func (jointId JointID) GetForceThreshold() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	return base.forceThreshold
}

// SetTorqueThreshold is b3Joint_SetTorqueThreshold.
// Set the torque threshold for joint events (N-m)
func (jointId JointID) SetTorqueThreshold(threshold float32) {
	assert(IsValidFloat(threshold) && threshold >= 0.0, "b3IsValidFloat( threshold ) && threshold >= 0.0f")

	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsJointSetTorqueThreshold{jointId, threshold}
		recWriteJointSetTorqueThreshold(world.recording, &recArgs)
	}
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	base.torqueThreshold = threshold
}

// GetTorqueThreshold is b3Joint_GetTorqueThreshold.
// Get the torque threshold for joint events (N-m)
func (jointId JointID) GetTorqueThreshold() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)
	return base.torqueThreshold
}

// CreateDistanceJoint is b3CreateDistanceJoint.
// Create a distance joint
// See DistanceJointDef for details
func (worldId WorldID) CreateDistanceJoint(def *DistanceJointDef) JointID {
	checkDef(def.Base.internalValue)
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	assert(IsValidFloat(def.Length) && def.Length > 0.0, "b3IsValidFloat( def->length ) && def->length > 0.0f")
	assert(def.LowerSpringForce <= def.UpperSpringForce, "def->lowerSpringForce <= def->upperSpringForce")

	pair := createJoint(world, &def.Base, DistanceJoint)

	joint := pair.jointSim

	joint.distanceJoint = distanceJoint{}
	joint.distanceJoint.length = MaxFloat(def.Length, LinearSlop())
	joint.distanceJoint.hertz = def.Hertz
	joint.distanceJoint.dampingRatio = def.DampingRatio
	joint.distanceJoint.lowerSpringForce = def.LowerSpringForce
	joint.distanceJoint.upperSpringForce = def.UpperSpringForce
	joint.distanceJoint.minLength = MaxFloat(def.MinLength, LinearSlop())
	joint.distanceJoint.maxLength = MaxFloat(def.MinLength, def.MaxLength)
	joint.distanceJoint.maxMotorForce = def.MaxMotorForce
	joint.distanceJoint.motorSpeed = def.MotorSpeed
	joint.distanceJoint.enableSpring = def.EnableSpring
	joint.distanceJoint.enableLimit = def.EnableLimit
	joint.distanceJoint.enableMotor = def.EnableMotor
	joint.distanceJoint.impulse = 0.0
	joint.distanceJoint.lowerImpulse = 0.0
	joint.distanceJoint.upperImpulse = 0.0
	joint.distanceJoint.motorImpulse = 0.0

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateDistanceJoint{worldId, *def}
		recWriteRetCreateDistanceJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateMotorJoint is b3CreateMotorJoint.
// Create a motor joint
// See MotorJointDef for details
func (worldId WorldID) CreateMotorJoint(def *MotorJointDef) JointID {
	checkDef(def.Base.internalValue)
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, MotorJoint)
	joint := pair.jointSim

	joint.motorJoint = motorJoint{}
	joint.motorJoint.linearVelocity = def.LinearVelocity
	joint.motorJoint.maxVelocityForce = def.MaxVelocityForce
	joint.motorJoint.angularVelocity = def.AngularVelocity
	joint.motorJoint.maxVelocityTorque = def.MaxVelocityTorque
	joint.motorJoint.linearHertz = def.LinearHertz
	joint.motorJoint.linearDampingRatio = def.LinearDampingRatio
	joint.motorJoint.maxSpringForce = def.MaxSpringForce
	joint.motorJoint.angularHertz = def.AngularHertz
	joint.motorJoint.angularDampingRatio = def.AngularDampingRatio
	joint.motorJoint.maxSpringTorque = def.MaxSpringTorque

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateMotorJoint{worldId, *def}
		recWriteRetCreateMotorJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateFilterJoint is b3CreateFilterJoint.
// Create a filter joint.
// See FilterJointDef for details
func (worldId WorldID) CreateFilterJoint(def *FilterJointDef) JointID {
	checkDef(def.Base.internalValue)
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, FilterJoint)

	joint := pair.jointSim

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateFilterJoint{worldId, *def}
		recWriteRetCreateFilterJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateParallelJoint is b3CreateParallelJoint.
// Create a parallel joint
// See ParallelJointDef for details
func (worldId WorldID) CreateParallelJoint(def *ParallelJointDef) JointID {
	checkDef(def.Base.internalValue)
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	assert(IsValidFloat(def.Hertz) && def.Hertz >= 0.0, "b3IsValidFloat( def->hertz ) && def->hertz >= 0.0f")
	assert(IsValidFloat(def.DampingRatio) && def.DampingRatio >= 0.0, "b3IsValidFloat( def->dampingRatio ) && def->dampingRatio >= 0.0f")
	assert(IsValidFloat(def.MaxTorque) && def.MaxTorque >= 0.0, "b3IsValidFloat( def->maxTorque ) && def->maxTorque >= 0.0f")

	pair := createJoint(world, &def.Base, ParallelJoint)

	joint := pair.jointSim

	joint.parallelJoint = parallelJoint{}
	joint.parallelJoint.hertz = def.Hertz
	joint.parallelJoint.dampingRatio = def.DampingRatio
	joint.parallelJoint.maxTorque = def.MaxTorque

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateParallelJoint{worldId, *def}
		recWriteRetCreateParallelJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreatePrismaticJoint is b3CreatePrismaticJoint.
// Create a prismatic (slider) joint.
// See PrismaticJointDef for details
func (worldId WorldID) CreatePrismaticJoint(def *PrismaticJointDef) JointID {
	checkDef(def.Base.internalValue)
	assert(def.LowerTranslation <= def.UpperTranslation, "def->lowerTranslation <= def->upperTranslation")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, PrismaticJoint)

	joint := pair.jointSim

	joint.prismaticJoint = prismaticJoint{}
	joint.prismaticJoint.hertz = def.Hertz
	joint.prismaticJoint.dampingRatio = def.DampingRatio
	joint.prismaticJoint.targetTranslation = def.TargetTranslation
	joint.prismaticJoint.lowerTranslation = def.LowerTranslation
	joint.prismaticJoint.upperTranslation = def.UpperTranslation
	joint.prismaticJoint.maxMotorForce = def.MaxMotorForce
	joint.prismaticJoint.motorSpeed = def.MotorSpeed
	joint.prismaticJoint.enableSpring = def.EnableSpring
	joint.prismaticJoint.enableLimit = def.EnableLimit
	joint.prismaticJoint.enableMotor = def.EnableMotor

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreatePrismaticJoint{worldId, *def}
		recWriteRetCreatePrismaticJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateRevoluteJoint is b3CreateRevoluteJoint.
// Create a revolute joint
// See RevoluteJointDef for details
func (worldId WorldID) CreateRevoluteJoint(def *RevoluteJointDef) JointID {
	checkDef(def.Base.internalValue)

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, RevoluteJoint)

	joint := pair.jointSim

	joint.revoluteJoint = revoluteJoint{}
	joint.revoluteJoint.hertz = def.Hertz
	joint.revoluteJoint.dampingRatio = def.DampingRatio
	joint.revoluteJoint.targetAngle = ClampFloat(def.TargetAngle, -Pi, Pi)

	lowerAngle := MinFloat(def.LowerAngle, def.UpperAngle)
	upperAngle := MaxFloat(def.LowerAngle, def.UpperAngle)
	joint.revoluteJoint.lowerAngle = ClampFloat(lowerAngle, -0.99*Pi, 0.99*Pi)
	joint.revoluteJoint.upperAngle = ClampFloat(upperAngle, -0.99*Pi, 0.99*Pi)

	joint.revoluteJoint.maxMotorTorque = def.MaxMotorTorque
	joint.revoluteJoint.motorSpeed = def.MotorSpeed
	joint.revoluteJoint.enableSpring = def.EnableSpring
	joint.revoluteJoint.enableLimit = def.EnableLimit
	joint.revoluteJoint.enableMotor = def.EnableMotor

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateRevoluteJoint{worldId, *def}
		recWriteRetCreateRevoluteJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateSphericalJoint is b3CreateSphericalJoint.
// Create a spherical joint
// See SphericalJointDef for details
func (worldId WorldID) CreateSphericalJoint(def *SphericalJointDef) JointID {
	checkDef(def.Base.internalValue)
	assert(0.0 <= def.ConeAngle && def.ConeAngle <= 0.99*Pi, "0.0f <= def->coneAngle && def->coneAngle <= 0.99f * B3_PI")
	assert(IsValidQuat(def.TargetRotation), "b3IsValidQuat( def->targetRotation )")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, SphericalJoint)

	joint := pair.jointSim

	joint.sphericalJoint = sphericalJoint{}
	joint.sphericalJoint.hertz = def.Hertz
	joint.sphericalJoint.dampingRatio = def.DampingRatio
	joint.sphericalJoint.targetRotation = def.TargetRotation
	joint.sphericalJoint.coneAngle = ClampFloat(def.ConeAngle, 0.0, 0.5*Pi)

	lowerAngle := MinFloat(def.LowerTwistAngle, def.UpperTwistAngle)
	upperAngle := MaxFloat(def.LowerTwistAngle, def.UpperTwistAngle)
	joint.sphericalJoint.lowerTwistAngle = ClampFloat(lowerAngle, -0.99*Pi, 0.99*Pi)
	joint.sphericalJoint.upperTwistAngle = ClampFloat(upperAngle, -0.99*Pi, 0.99*Pi)

	joint.sphericalJoint.maxMotorTorque = def.MaxMotorTorque
	joint.sphericalJoint.motorVelocity = def.MotorVelocity
	joint.sphericalJoint.enableSpring = def.EnableSpring
	joint.sphericalJoint.enableConeLimit = def.EnableConeLimit
	joint.sphericalJoint.enableTwistLimit = def.EnableTwistLimit
	joint.sphericalJoint.enableMotor = def.EnableMotor

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateSphericalJoint{worldId, *def}
		recWriteRetCreateSphericalJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateWeldJoint is b3CreateWeldJoint.
// Create a weld joint
// See WeldJointDef for details
func (worldId WorldID) CreateWeldJoint(def *WeldJointDef) JointID {
	checkDef(def.Base.internalValue)
	assert(0.0 <= def.AngularHertz, "0.0f <= def->angularHertz")
	assert(0.0 <= def.AngularDampingRatio, "0.0f <= def->angularDampingRatio")
	assert(0.0 <= def.LinearHertz, "0.0f <= def->linearHertz")
	assert(0.0 <= def.LinearDampingRatio, "0.0f <= def->linearDampingRatio")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, WeldJoint)

	joint := pair.jointSim

	joint.weldJoint = weldJoint{}
	joint.weldJoint.linearHertz = def.LinearHertz
	joint.weldJoint.linearDampingRatio = def.LinearDampingRatio
	joint.weldJoint.angularHertz = def.AngularHertz
	joint.weldJoint.angularDampingRatio = def.AngularDampingRatio

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateWeldJoint{worldId, *def}
		recWriteRetCreateWeldJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// CreateWheelJoint is b3CreateWheelJoint.
// Create a wheel joint
// See WheelJointDef for details.
func (worldId WorldID) CreateWheelJoint(def *WheelJointDef) JointID {
	checkDef(def.Base.internalValue)
	assert(def.LowerSuspensionLimit <= def.UpperSuspensionLimit, "def->lowerSuspensionLimit <= def->upperSuspensionLimit")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointID{}
	}

	pair := createJoint(world, &def.Base, WheelJoint)

	joint := pair.jointSim

	joint.wheelJoint = wheelJoint{}
	joint.wheelJoint.enableSuspensionSpring = def.EnableSuspensionSpring
	joint.wheelJoint.suspensionHertz = def.SuspensionHertz
	joint.wheelJoint.suspensionDampingRatio = def.SuspensionDampingRatio
	joint.wheelJoint.enableSuspensionLimit = def.EnableSuspensionLimit
	joint.wheelJoint.lowerSuspensionLimit = def.LowerSuspensionLimit
	joint.wheelJoint.upperSuspensionLimit = def.UpperSuspensionLimit
	joint.wheelJoint.enableSpinMotor = def.EnableSpinMotor
	joint.wheelJoint.maxSpinTorque = def.MaxSpinTorque
	joint.wheelJoint.spinSpeed = def.SpinSpeed

	joint.wheelJoint.enableSteering = def.EnableSteering
	joint.wheelJoint.steeringHertz = def.SteeringHertz
	joint.wheelJoint.steeringDampingRatio = def.SteeringDampingRatio
	joint.wheelJoint.targetSteeringAngle = def.TargetSteeringAngle
	joint.wheelJoint.maxSteeringTorque = def.MaxSteeringTorque
	joint.wheelJoint.enableSteeringLimit = def.EnableSteeringLimit
	joint.wheelJoint.lowerSteeringLimit = def.LowerSteeringLimit
	joint.wheelJoint.upperSteeringLimit = def.UpperSteeringLimit

	jointId := JointID{joint.jointId + 1, world.worldId, pair.joint.generation}
	if world.recording != nil {
		recCreateArgs := recArgsCreateWheelJoint{worldId, *def}
		recWriteRetCreateWheelJoint(world.recording, &recCreateArgs, jointId)
	}
	return jointId
}

// destroyJointInternal is b3DestroyJointInternal.
func destroyJointInternal(world *world, joint *joint, wakeBodies bool) {
	jointId := joint.jointId

	edgeA := &joint.edges[0]
	edgeB := &joint.edges[1]

	idA := edgeA.bodyId
	idB := edgeB.bodyId
	bodyA := arrayGet(world.bodies, int(idA))
	bodyB := arrayGet(world.bodies, int(idB))

	// Remove from body A
	if edgeA.prevKey != NullIndex {
		prevJoint := arrayGet(world.joints, int(edgeA.prevKey>>1))
		prevEdge := &prevJoint.edges[edgeA.prevKey&1]
		prevEdge.nextKey = edgeA.nextKey
	}

	if edgeA.nextKey != NullIndex {
		nextJoint := arrayGet(world.joints, int(edgeA.nextKey>>1))
		nextEdge := &nextJoint.edges[edgeA.nextKey&1]
		nextEdge.prevKey = edgeA.prevKey
	}

	edgeKeyA := (jointId << 1) | 0
	if bodyA.headJointKey == edgeKeyA {
		bodyA.headJointKey = edgeA.nextKey
	}

	bodyA.jointCount -= 1

	// Remove from body B
	if edgeB.prevKey != NullIndex {
		prevJoint := arrayGet(world.joints, int(edgeB.prevKey>>1))
		prevEdge := &prevJoint.edges[edgeB.prevKey&1]
		prevEdge.nextKey = edgeB.nextKey
	}

	if edgeB.nextKey != NullIndex {
		nextJoint := arrayGet(world.joints, int(edgeB.nextKey>>1))
		nextEdge := &nextJoint.edges[edgeB.nextKey&1]
		nextEdge.prevKey = edgeB.prevKey
	}

	edgeKeyB := (jointId << 1) | 1
	if bodyB.headJointKey == edgeKeyB {
		bodyB.headJointKey = edgeB.nextKey
	}

	bodyB.jointCount -= 1

	if joint.islandId != NullIndex {
		assert(joint.setIndex > disabledSet, "joint->setIndex > b3_disabledSet")
		unlinkJoint(world, joint)
	} else {
		assert(joint.setIndex <= disabledSet, "joint->setIndex <= b3_disabledSet")
	}

	// Remove joint from solver set that owns it
	setIndex := int(joint.setIndex)
	localIndex := int(joint.localIndex)

	if setIndex == awakeSet {
		removeJointFromGraph(world, int(joint.edges[0].bodyId), int(joint.edges[1].bodyId), int(joint.colorIndex), localIndex)
	} else {
		set := arrayGet(world.solverSets, setIndex)
		movedIndex := arrayRemoveSwap(&set.jointSims, localIndex)
		if movedIndex != NullIndex {
			// Fix moved joint
			movedJointSim := &set.jointSims[localIndex]
			movedId := movedJointSim.jointId
			movedJoint := arrayGet(world.joints, int(movedId))
			assert(int(movedJoint.localIndex) == movedIndex, "movedJoint->localIndex == movedIndex")
			movedJoint.localIndex = int32(localIndex)
		}
	}

	// Free joint and id (preserve joint revision)
	joint.setIndex = NullIndex
	joint.localIndex = NullIndex
	joint.colorIndex = NullIndex
	joint.jointId = NullIndex
	freeId(&world.jointIdPool, int(jointId))

	if wakeBodies {
		wakeBody(world, bodyA)
		wakeBody(world, bodyB)
	}

	validateSolverSets(world)
}

// Destroy is b3DestroyJoint.
// Destroy a joint
func (jointId JointID) Destroy(wakeAttached bool) {
	world := getWorld(int(jointId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsDestroyJoint{jointId, wakeAttached}
		recWriteDestroyJoint(world.recording, &recArgs)
	}

	joint := getJointFullId(world, jointId)

	destroyJointInternal(world, joint, wakeAttached)
}

// GetType is b3Joint_GetType.
// Get the joint type
func (jointId JointID) GetType() JointType {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return joint.typ
}

// GetBodyA is b3Joint_GetBodyA.
// Get body A id on a joint
func (jointId JointID) GetBodyA() BodyID {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return makeBodyId(world, int(joint.edges[0].bodyId))
}

// GetBodyB is b3Joint_GetBodyB.
// Get body B id on a joint
func (jointId JointID) GetBodyB() BodyID {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return makeBodyId(world, int(joint.edges[1].bodyId))
}

// GetWorld is b3Joint_GetWorld.
// Get the world that owns this joint
func (jointId JointID) GetWorld() WorldID {
	world := getWorld(int(jointId.World0))
	return WorldID{uint16(jointId.World0 + 1), world.generation}
}

// SetLocalFrameA is b3Joint_SetLocalFrameA.
// Set the local frame on bodyA
func (jointId JointID) SetLocalFrameA(localFrame Transform) {
	assert(IsValidTransform(localFrame), "b3IsValidTransform( localFrame )")

	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsJointSetLocalFrameA{jointId, localFrame}
		recWriteJointSetLocalFrameA(world.recording, &recArgs)
	}
	joint := getJointFullId(world, jointId)
	jointSim := getJointSim(world, joint)
	jointSim.localFrameA = localFrame
}

// GetLocalFrameA is b3Joint_GetLocalFrameA.
// Get the local frame on bodyA
func (jointId JointID) GetLocalFrameA() Transform {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	jointSim := getJointSim(world, joint)
	return jointSim.localFrameA
}

// SetLocalFrameB is b3Joint_SetLocalFrameB.
// Set the local frame on bodyB
func (jointId JointID) SetLocalFrameB(localFrame Transform) {
	assert(IsValidTransform(localFrame), "b3IsValidTransform( localFrame )")

	world := getWorld(int(jointId.World0))
	if world.recording != nil {
		recArgs := recArgsJointSetLocalFrameB{jointId, localFrame}
		recWriteJointSetLocalFrameB(world.recording, &recArgs)
	}
	joint := getJointFullId(world, jointId)
	jointSim := getJointSim(world, joint)
	jointSim.localFrameB = localFrame
}

// GetLocalFrameB is b3Joint_GetLocalFrameB.
// Get the local frame on bodyB
func (jointId JointID) GetLocalFrameB() Transform {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	jointSim := getJointSim(world, joint)
	return jointSim.localFrameB
}

// SetCollideConnected is b3Joint_SetCollideConnected.
// Toggle collision between connected bodies
func (jointId JointID) SetCollideConnected(shouldCollide bool) {
	world := getUnlockedWorld(int(jointId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsJointSetCollideConnected{jointId, shouldCollide}
		recWriteJointSetCollideConnected(world.recording, &recArgs)
	}

	joint := getJointFullId(world, jointId)
	if joint.collideConnected == shouldCollide {
		return
	}

	joint.collideConnected = shouldCollide

	bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
	bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))

	if shouldCollide {
		// need to tell the broad-phase to look for new pairs for one of the
		// two bodies. Pick the one with the fewest shapes.
		shapeCountA := bodyA.shapeCount
		shapeCountB := bodyB.shapeCount

		shapeId := bodyB.headShapeId
		if shapeCountA < shapeCountB {
			shapeId = bodyA.headShapeId
		}
		for shapeId != NullIndex {
			shape := arrayGet(world.shapes, int(shapeId))

			if shape.proxyKey != NullIndex {
				world.broadPhase.markProxyMovedSerial(int(shape.proxyKey))
			}

			shapeId = shape.nextShapeId
		}
	} else {
		destroyContactsBetweenBodies(world, bodyA, bodyB)
	}
}

// GetCollideConnected is b3Joint_GetCollideConnected.
// Is collision allowed between connected bodies?
func (jointId JointID) GetCollideConnected() bool {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return joint.collideConnected
}

// SetUserData is b3Joint_SetUserData.
// Set the user data on a joint
func (jointId JointID) SetUserData(userData any) {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	joint.userData = userData
}

// GetUserData is b3Joint_GetUserData.
// Get the user data on a joint
func (jointId JointID) GetUserData() any {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return joint.userData
}

// WakeBodies is b3Joint_WakeBodies.
// Wake the bodies connect to this joint
func (jointId JointID) WakeBodies() {
	world := getUnlockedWorld(int(jointId.World0))
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsJointWakeBodies{jointId}
		recWriteJointWakeBodies(world.recording, &recArgs)
	}

	world.locked = true

	joint := getJointFullId(world, jointId)
	bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
	bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))

	wakeBody(world, bodyA)
	wakeBody(world, bodyB)

	world.locked = false
}

// IsAwake is b3Joint_IsAwake.
// Is the joint awake? If true then an attached body is awake.
func (jointId JointID) IsAwake() bool {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)

	// Cheaper than checking if the bodies are awake.
	return joint.setIndex == awakeSet
}

// getJointReaction is b3GetJointReaction.
// PORT: the force and torque out-parameters are the return values.
func getJointReaction(world *world, sim *jointSim, invTimeStep float32) (force float32, torque float32) {
	return getJointReactionAt(world, sim, invTimeStep, nil)
}

// Local continuation evaluates spherical reaction axes at its prepared impact
// pose; ordinary native callers retain their original world-transform lookup.
func getJointReactionAt(world *world, sim *jointSim, invTimeStep float32, transform func(int32) WorldTransform) (force float32, torque float32) {
	var linearImpulse float32 = 0.0
	var angularImpulse float32 = 0.0

	switch sim.typ {
	case ParallelJoint:
		joint := &sim.parallelJoint
		impulse := Vec3{
			X: joint.perpImpulse.X,
			Y: joint.perpImpulse.Y,
			Z: 0.0,
		}
		angularImpulse = Length(impulse)

	case DistanceJoint:
		joint := &sim.distanceJoint
		linearImpulse = AbsFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse)

	case MotorJoint:
		joint := &sim.motorJoint
		linearImpulse = Length(Add(joint.linearVelocityImpulse, joint.linearSpringImpulse))
		angularImpulse = Length(Add(joint.angularVelocityImpulse, joint.angularSpringImpulse))

	case PrismaticJoint:
		joint := &sim.prismaticJoint
		impulse := Vec3{
			X: joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
			Y: joint.perpImpulse.X,
			Z: joint.perpImpulse.Y,
		}
		linearImpulse = Length(impulse)
		angularImpulse = Length(joint.angularImpulse)

	case RevoluteJoint:
		joint := &sim.revoluteJoint
		linearImpulse = Length(joint.linearImpulse)
		impulse := Vec3{
			X: joint.perpImpulse.X,
			Y: joint.perpImpulse.Y,
			Z: joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
		}
		angularImpulse = Length(impulse)

	case SphericalJoint:
		// todo improve performance
		joint := &sim.sphericalJoint
		linearImpulse = Length(joint.linearImpulse)

		var xfA, xfB WorldTransform
		if transform == nil {
			xfA = getBodyTransform(world, int(sim.bodyIdA))
			xfB = getBodyTransform(world, int(sim.bodyIdB))
		} else {
			xfA = transform(sim.bodyIdA)
			xfB = transform(sim.bodyIdB)
		}
		qA := MulQuat(xfA.Q, sim.localFrameA.Q)
		qB := MulQuat(xfB.Q, sim.localFrameB.Q)

		// Cone axis is the z-axis of body A.
		coneAxis := RotateVector(qA, Vec3AxisZ)
		twistAxis := RotateVector(qB, Vec3AxisZ)
		swingAxis := Normalize(Cross(coneAxis, twistAxis))

		impulse := Add(joint.springImpulse, joint.motorImpulse)
		impulse = MulAdd(impulse, joint.lowerTwistImpulse-joint.upperTwistImpulse, twistAxis)
		impulse = MulAdd(impulse, joint.swingImpulse, swingAxis)

		angularImpulse = Length(impulse)

	case WeldJoint:
		joint := &sim.weldJoint
		linearImpulse = Length(joint.linearImpulse)
		angularImpulse = Length(joint.angularImpulse)

	case WheelJoint:
		// todo probably wrong
		joint := &sim.wheelJoint
		perpImpulse := joint.linearImpulse
		axialImpulse := joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse
		linearImpulse = sqrtf(float32(perpImpulse.X*perpImpulse.X) + float32(perpImpulse.Y*perpImpulse.Y) + float32(axialImpulse*axialImpulse))
		angularImpulse = AbsFloat(joint.spinImpulse)

	default:
	}

	force = linearImpulse * invTimeStep
	torque = angularImpulse * invTimeStep
	return force, torque
}

// getJointConstraintForce is b3GetJointConstraintForce.
func getJointConstraintForce(world *world, joint *joint) Vec3 {
	base := getJointSim(world, joint)

	switch joint.typ {
	case ParallelJoint:
		return Vec3Zero

	case DistanceJoint:
		return getDistanceJointForce(world, base)

	case FilterJoint:
		return Vec3Zero

	case MotorJoint:
		return getMotorJointForce(world, base)

	case PrismaticJoint:
		return getPrismaticJointForce(world, base)

	case RevoluteJoint:
		return getRevoluteJointForce(world, base)

	case SphericalJoint:
		return getSphericalJointForce(world, base)

	case WeldJoint:
		return getWeldJointForce(world, base)

	case WheelJoint:
		return getWheelJointForce(world, base)

	default:
		assert(false, "false")
		return Vec3Zero
	}
}

// getJointConstraintTorque is b3GetJointConstraintTorque.
func getJointConstraintTorque(world *world, joint *joint) Vec3 {
	base := getJointSim(world, joint)

	switch joint.typ {
	case ParallelJoint:
		return getParallelJointTorque(world, base)

	case DistanceJoint:
		return Vec3Zero

	case FilterJoint:
		return Vec3Zero

	case MotorJoint:
		return getMotorJointTorque(world, base)

	case PrismaticJoint:
		return getPrismaticJointTorque(world, base)

	case RevoluteJoint:
		return getRevoluteJointTorque(world, base)

	case SphericalJoint:
		return getSphericalJointTorque(world, base)

	case WeldJoint:
		return getWeldJointTorque(world, base)

	case WheelJoint:
		return getWheelJointTorque(world, base)

	default:
		assert(false, "false")
		return Vec3Zero
	}
}

// GetConstraintForce is b3Joint_GetConstraintForce.
// Get the current constraint force for this joint
func (jointId JointID) GetConstraintForce() Vec3 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return getJointConstraintForce(world, joint)
}

// GetConstraintTorque is b3Joint_GetConstraintTorque.
// Get the current constraint torque for this joint
func (jointId JointID) GetConstraintTorque() Vec3 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	return getJointConstraintTorque(world, joint)
}

// GetLinearSeparation is b3Joint_GetLinearSeparation.
// Get the current linear separation error for this joint. Does not consider admissible movement. Usually in meters.
func (jointId JointID) GetLinearSeparation() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)

	xfA := getBodyTransform(world, int(joint.edges[0].bodyId))
	xfB := getBodyTransform(world, int(joint.edges[1].bodyId))

	pA := TransformWorldPoint(xfA, base.localFrameA.P)
	pB := TransformWorldPoint(xfB, base.localFrameB.P)
	dp := SubPos(pB, pA)

	switch joint.typ {
	case ParallelJoint:
		return 0.0

	case DistanceJoint:
		distanceJoint := &base.distanceJoint
		length := Length(dp)
		if distanceJoint.enableSpring {
			if distanceJoint.enableLimit {
				if length < distanceJoint.minLength {
					return distanceJoint.minLength - length
				}

				if length > distanceJoint.maxLength {
					return length - distanceJoint.maxLength
				}

				return 0.0
			}

			return 0.0
		}

		return AbsFloat(length - distanceJoint.length)

	case MotorJoint:
		return 0.0

	case FilterJoint:
		return 0.0

	case PrismaticJoint:
		prismaticJoint := &base.prismaticJoint
		axisA := RotateVector(xfA.Q, Vec3AxisX)
		perpA := Perp(axisA)
		perpendicularSeparation := AbsFloat(Dot(perpA, dp))
		var limitSeparation float32 = 0.0

		if prismaticJoint.enableLimit {
			translation := Dot(axisA, dp)
			if translation < prismaticJoint.lowerTranslation {
				limitSeparation = prismaticJoint.lowerTranslation - translation
			}

			if prismaticJoint.upperTranslation < translation {
				limitSeparation = translation - prismaticJoint.upperTranslation
			}
		}

		return sqrtf(float32(perpendicularSeparation*perpendicularSeparation) + float32(limitSeparation*limitSeparation))

	case RevoluteJoint:
		return Length(dp)

	case SphericalJoint:
		return Length(dp)

	case WeldJoint:
		weldJoint := &base.weldJoint
		if weldJoint.linearHertz == 0.0 {
			return Length(dp)
		}

		return 0.0

	case WheelJoint:
		wheelJoint := &base.wheelJoint
		axisA := RotateVector(xfA.Q, Vec3AxisX)
		perpA := Perp(axisA)
		perpendicularSeparation := AbsFloat(Dot(perpA, dp))
		var limitSeparation float32 = 0.0

		if wheelJoint.enableSuspensionLimit {
			translation := Dot(axisA, dp)
			if translation < wheelJoint.lowerSuspensionLimit {
				limitSeparation = wheelJoint.lowerSuspensionLimit - translation
			}

			if wheelJoint.upperSuspensionLimit < translation {
				limitSeparation = translation - wheelJoint.upperSuspensionLimit
			}
		}

		return sqrtf(float32(perpendicularSeparation*perpendicularSeparation) + float32(limitSeparation*limitSeparation))

	default:
		assert(false, "false")
		return 0.0
	}
}

// GetAngularSeparation is b3Joint_GetAngularSeparation.
// Get the current angular separation error for this joint. Does not consider admissible movement. Usually in radians.
func (jointId JointID) GetAngularSeparation() float32 {
	world := getWorld(int(jointId.World0))
	joint := getJointFullId(world, jointId)
	base := getJointSim(world, joint)

	xfA := getBodyTransform(world, int(joint.edges[0].bodyId))
	xfB := getBodyTransform(world, int(joint.edges[1].bodyId))

	relQ := InvMulQuat(xfA.Q, xfB.Q)

	switch joint.typ {
	case ParallelJoint:
		// Remove hinge angle
		relQ.V.Z = 0.0
		return GetQuatAngle(relQ)

	case DistanceJoint:
		return 0.0

	case MotorJoint:
		return 0.0

	case FilterJoint:
		return 0.0

	case PrismaticJoint:
		return GetQuatAngle(relQ)

	case RevoluteJoint:
		revoluteJoint := &base.revoluteJoint
		if revoluteJoint.enableLimit {
			angle := GetTwistAngle(relQ)
			if angle < revoluteJoint.lowerAngle {
				return GetQuatAngle(relQ)
			}

			if revoluteJoint.upperAngle < angle {
				return GetQuatAngle(relQ)
			}
		}

		// Remove hinge angle
		relQ.V.Z = 0.0
		return GetQuatAngle(relQ)

	case SphericalJoint:
		sphericalJoint := &base.sphericalJoint
		var sum float32 = 0.0
		if sphericalJoint.enableConeLimit {
			swingAngle := GetSwingAngle(relQ)
			sum += MaxFloat(0.0, swingAngle-sphericalJoint.coneAngle)
		}

		if sphericalJoint.enableTwistLimit {
			twistAngle := GetTwistAngle(relQ)
			sum += MaxFloat(0.0, sphericalJoint.lowerTwistAngle-twistAngle)
			sum += MaxFloat(0.0, twistAngle-sphericalJoint.upperTwistAngle)
		}

		return sum

	case WeldJoint:
		weldJoint := &base.weldJoint
		if weldJoint.angularHertz == 0.0 {
			return GetQuatAngle(relQ)
		}

		return 0.0

	case WheelJoint:
		// todo
		assert(false, "false")
		return 0.0

	default:
		assert(false, "false")
		return 0.0
	}
}

// PORT: the #if 0 block with b3Joint_SetSpringRotationTarget is not ported (compiled out
// upstream).

// prepareJoint is b3PrepareJoint.
func prepareJoint(joint *jointSim, context *stepContext) {
	// Clamp joint hertz based on the time step to reduce jitter.
	hertz := MinFloat(joint.constraintHertz, 0.25*context.inv_h)
	joint.constraintSoftness = makeSoft(hertz, joint.constraintDampingRatio, context.h)

	switch joint.typ {
	case ParallelJoint:
		prepareParallelJoint(joint, context)

	case DistanceJoint:
		prepareDistanceJoint(joint, context)

	case FilterJoint:
		break

	case MotorJoint:
		prepareMotorJoint(joint, context)

	case PrismaticJoint:
		preparePrismaticJoint(joint, context)

	case RevoluteJoint:
		prepareRevoluteJoint(joint, context)

	case SphericalJoint:
		prepareSphericalJoint(joint, context)

	case WeldJoint:
		prepareWeldJoint(joint, context)

	case WheelJoint:
		prepareWheelJoint(joint, context)

	default:
		assert(false, "false")
	}
}

// warmStartJoint is b3WarmStartJoint.
func warmStartJoint(joint *jointSim, context *stepContext) {
	switch joint.typ {
	case ParallelJoint:
		warmStartParallelJoint(joint, context)

	case DistanceJoint:
		warmStartDistanceJoint(joint, context)

	case FilterJoint:
		break

	case MotorJoint:
		warmStartMotorJoint(joint, context)

	case PrismaticJoint:
		warmStartPrismaticJoint(joint, context)

	case RevoluteJoint:
		warmStartRevoluteJoint(joint, context)

	case SphericalJoint:
		warmStartSphericalJoint(joint, context)

	case WeldJoint:
		warmStartWeldJoint(joint, context)

	case WheelJoint:
		warmStartWheelJoint(joint, context)

	default:
		assert(false, "false")
	}
}

// solveJoint is b3SolveJoint.
func solveJoint(joint *jointSim, context *stepContext, useBias bool) {
	switch joint.typ {
	case ParallelJoint:
		solveParallelJoint(joint, context)

	case DistanceJoint:
		solveDistanceJoint(joint, context, useBias)

	case FilterJoint:
		break

	case MotorJoint:
		solveMotorJoint(joint, context)

	case PrismaticJoint:
		solvePrismaticJoint(joint, context, useBias)

	case RevoluteJoint:
		solveRevoluteJoint(joint, context, useBias)

	case SphericalJoint:
		solveSphericalJoint(joint, context, useBias)

	case WeldJoint:
		solveWeldJoint(joint, context, useBias)

	case WheelJoint:
		solveWheelJoint(joint, context, useBias)

	default:
		assert(false, "false")
	}
}

// prepareJointsOverflow is b3PrepareJoints_Overflow.
func prepareJointsOverflow(context *stepContext) {
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		prepareJoint(joint, context)
	}
}

// warmStartJointsOverflow is b3WarmStartJoints_Overflow.
func warmStartJointsOverflow(context *stepContext) {
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		warmStartJoint(joint, context)
	}
}

// solveJointsOverflow is b3SolveJoints_Overflow.
func solveJointsOverflow(context *stepContext, useBias bool) {
	graph := context.graph
	joints := graph.colors[overflowIndex].jointSims
	jointCount := len(joints)

	for i := 0; i < jointCount; i++ {
		joint := &joints[i]
		solveJoint(joint, context, useBias)
	}
}

// drawJoint is b3DrawJoint.
func drawJoint(draw *DebugDraw, world *world, joint *joint) {
	bodyA := arrayGet(world.bodies, int(joint.edges[0].bodyId))
	bodyB := arrayGet(world.bodies, int(joint.edges[1].bodyId))
	if bodyA.setIndex == disabledSet || bodyB.setIndex == disabledSet {
		return
	}

	jointSim := getJointSim(world, joint)

	transformA := getBodyTransformQuick(world, bodyA)
	transformB := getBodyTransformQuick(world, bodyB)
	pA := TransformWorldPoint(transformA, jointSim.localFrameA.P)
	pB := TransformWorldPoint(transformB, jointSim.localFrameB.P)

	color := ColorDarkSeaGreen

	scale := MaxFloat(0.0001, draw.JointScale*joint.drawScale)

	switch joint.typ {
	case ParallelJoint:
		drawParallelJoint(draw, jointSim, transformA, transformB, scale)

	case DistanceJoint:
		drawDistanceJoint(draw, jointSim, transformA, transformB)

	case FilterJoint:
		draw.DrawSegmentFcn(pA, pB, ColorGold, draw.Context)

	case MotorJoint:
		draw.DrawSegmentFcn(pA, pB, ColorPlum, draw.Context)
		draw.DrawPointFcn(pA, 8.0, ColorYellowGreen, draw.Context)
		draw.DrawPointFcn(pB, 8.0, ColorPlum, draw.Context)

	case PrismaticJoint:
		drawPrismaticJoint(draw, jointSim, transformA, transformB, scale)

	case RevoluteJoint:
		drawRevoluteJoint(draw, jointSim, transformA, transformB, scale)

	case SphericalJoint:
		drawSphericalJoint(draw, jointSim, transformA, transformB, scale)

	case WeldJoint:
		drawWeldJoint(draw, jointSim, transformA, transformB, scale)

	case WheelJoint:
		drawWheelJoint(draw, jointSim, transformA, transformB, scale)

	default:
		draw.DrawSegmentFcn(transformA.P, pA, color, draw.Context)
		draw.DrawSegmentFcn(pA, pB, color, draw.Context)
		draw.DrawSegmentFcn(transformB.P, pB, color, draw.Context)
	}

	if draw.DrawGraphColors {
		graphColors := [GraphColorCount]HexColor{
			ColorRed, ColorOrange, ColorYellow, ColorGreen, ColorCyan, ColorBlue,
			ColorViolet, ColorPink, ColorChocolate, ColorGoldenRod, ColorCoral, ColorRosyBrown,
			ColorAqua, ColorPeru, ColorLime, ColorGold, ColorPlum, ColorSnow,
			ColorTeal, ColorKhaki, ColorSalmon, ColorPeachPuff, ColorHoneyDew, ColorBlack,
		}

		colorIndex := joint.colorIndex
		if colorIndex != NullIndex {
			p := LerpPosition(pA, pB, 0.5)
			draw.DrawPointFcn(p, 5.0, graphColors[colorIndex], draw.Context)
		}
	}

	if draw.DrawJointExtras {
		force := getJointConstraintForce(world, joint)
		torque := getJointConstraintTorque(world, joint)
		p := LerpPosition(pA, pB, 0.5)

		draw.DrawSegmentFcn(p, OffsetPos(p, MulSV(0.001, force)), ColorAzure, draw.Context)

		// PORT: snprintf( buffer, 64, "f = %g, t = %g", ... ) is fmt.Sprintf with %.6g (C's %g
		// precision); no 64 byte truncation.
		buffer := fmt.Sprintf("f = %.6g, t = %.6g", Length(force), Length(torque))
		draw.DrawStringFcn(p, buffer, ColorAzure, draw.Context)
	}
}
