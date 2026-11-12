// Port of src/contact.h and src/contact.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// ---- contact.h ----

// forceGhostCollisions is B3_FORCE_GHOST_COLLISIONS.
const forceGhostCollisions = false

// contactCache is b3ContactCache.
// PORT: union { satCache; simplexCache }. Both members are present; the shape pair decides which
// one is used.
type contactCache struct {
	satCache     SATCache
	simplexCache SimplexCache
}

// triangleCache is b3TriangleCache.
type triangleCache struct {
	triangleIndex int32
	cache         contactCache
}

// b3ContactFlags. PORT: untyped constants so they mix with the uint32 flag fields.
const (
	// Set when the solid shapes are touching.
	contactTouchingFlag = 0x00000001

	// Contact has a hit event
	contactHitEventFlag = 0x00000002

	// This contact wants contact events
	contactEnableContactEvents = 0x00000004

	// This is contact is between a dynamic and static body
	contactStaticFlag = 0x00000008

	contactRecycleFlag = 0x00000010

	// Set when the shapes are touching
	simTouchingFlag = 0x00010000

	// This contact no longer has overlapping AABBs
	simDisjoint = 0x00020000

	// This contact started touching
	simStartedTouching = 0x00040000

	// This contact stopped touching
	simStoppedTouching = 0x00080000

	// This contact has a hit event
	simEnableHitEvent = 0x00100000

	// This contact wants pre-solve events
	simEnablePreSolveEvents = 0x00200000

	// This is a mesh contact
	simMeshContact = 0x00400000

	// Relative transform is cached for contact recycling
	relativeTransformValid = 0x00800000

	// Enable speculative contact points
	enableSpeculativePoints = 0x01000000
)

// contactEdge is b3ContactEdge.
// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
type contactEdge struct {
	bodyId  int32
	prevKey int32
	nextKey int32
}

// meshContact is b3MeshContact.
type meshContact struct {
	triangleCache []triangleCache
	queryBounds   AABB
}

// convexContact is b3ConvexContact.
type convexContact struct {
	cache contactCache
}

// contact is b3Contact: represents the persistent interaction between two shapes
type contact struct {
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	setIndex int32

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	colorIndex int32

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	localIndex int32

	edges      [2]contactEdge
	shapeIdA   int32
	shapeIdB   int32
	childIndex int32

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	islandId int32

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	islandIndex int32

	// Back index into b3World::contacts
	contactId int32

	// Encoded body sim indices to avoid a cache miss accessing the body in the narrow phase.
	encodedBodySimA int32
	encodedBodySimB int32

	// b3ContactFlags
	flags uint32

	// todo embed single manifold with ability to allocate overfloat, use union.
	// PORT: b3Manifold* is a slice from the manifold allocator; manifoldCount is kept.
	manifolds     []Manifold
	manifoldCount int32

	// Cache for contact recycling.
	cachedRotationA    Quat
	cachedRotationB    Quat
	cachedRelativePose Transform

	// Mixed friction and restitution
	friction float32

	// Usage determined by b3_simMeshContact in simFlags
	// PORT: union { convexContact; meshContact }. Both members are present.
	convexContact convexContact
	meshContact   meshContact

	restitution       float32
	rollingResistance float32
	tangentVelocity   Vec3

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	generation uint32
}

// contactSpec is b3ContactSpec.
type contactSpec struct {
	contactId int32

	// Start of the global manifold constraint array
	manifoldStart int32
	manifoldCount uint16
}

// ---- contact.c ----

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b3Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b3UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b3ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.
// typedef b3Manifold b3ManifoldFcn( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
//								  b3ContactCache* cache );

// getContactFullId is b3GetContactFullId.
func getContactFullId(world *world, contactId ContactID) *contact {
	id := int(contactId.Index1) - 1
	contact := arrayGet(world.contacts, id)
	assert(int(contact.contactId) == id && contact.generation == contactId.Generation,
		"contact->contactId == id && contact->generation == contactId.generation")
	return contact
}

// GetData is b3Contact_GetData.
func (contactId ContactID) GetData() ContactData {
	world := getWorld(int(contactId.World0))
	contact := getContactFullId(world, contactId)

	shapeA := arrayGet(world.shapes, int(contact.shapeIdA))
	shapeB := arrayGet(world.shapes, int(contact.shapeIdB))

	data := ContactData{}
	data.ContactId = contactId
	data.ShapeIdA = ShapeID{
		Index1:     shapeA.id + 1,
		World0:     contactId.World0,
		Generation: shapeA.generation,
	}
	data.ShapeIdB = ShapeID{
		Index1:     shapeB.id + 1,
		World0:     contactId.World0,
		Generation: shapeB.generation,
	}

	if contact.manifoldCount > 0 {
		// PORT: the slice points at the contact's manifolds, as upstream's pointer does.
		data.Manifolds = contact.manifolds[:contact.manifoldCount]
		data.ManifoldCount = contact.manifoldCount
	} else {
		data.Manifolds = nil
		data.ManifoldCount = 0
	}

	return data
}

// contactRegister is struct b3ContactRegister.
type contactRegister struct {
	// b3ManifoldFcn* fcn;
	supported bool
	primary   bool
}

// contactRegisters is s_registers.
var contactRegisters [ShapeTypeCount][ShapeTypeCount]contactRegister

// contactRegistersInitialized is s_initialized.
var contactRegistersInitialized = false

// contactAddType is b3AddType.
func contactAddType(type1, type2 ShapeType) {
	assert(0 <= type1 && type1 < ShapeTypeCount, "0 <= type1 && type1 < b3_shapeTypeCount")
	assert(0 <= type2 && type2 < ShapeTypeCount, "0 <= type2 && type2 < b3_shapeTypeCount")

	contactRegisters[type1][type2].supported = true
	contactRegisters[type1][type2].primary = true

	if type1 != type2 {
		contactRegisters[type2][type1].supported = true
		contactRegisters[type2][type1].primary = false
	}
}

// initializeContactRegisters is b3InitializeContactRegisters.
func initializeContactRegisters() {
	if contactRegistersInitialized == false {
		contactAddType(SphereShape, SphereShape)
		contactAddType(CapsuleShape, SphereShape)
		contactAddType(CapsuleShape, CapsuleShape)
		contactAddType(CompoundShape, SphereShape)
		contactAddType(CompoundShape, CapsuleShape)
		contactAddType(CompoundShape, HullShape)
		contactAddType(HullShape, SphereShape)
		contactAddType(HullShape, CapsuleShape)
		contactAddType(HullShape, HullShape)
		contactAddType(MeshShape, SphereShape)
		contactAddType(MeshShape, CapsuleShape)
		contactAddType(MeshShape, HullShape)
		contactAddType(HeightShape, SphereShape)
		contactAddType(HeightShape, CapsuleShape)
		contactAddType(HeightShape, HullShape)
		contactRegistersInitialized = true
	}
}

// createContact is b3CreateContact.
func createContact(world *world, shapeA, shapeB *shape, childIndex int) {
	typeA := shapeA.typ
	typeB := shapeB.typ

	assert(0 <= typeA && typeA < ShapeTypeCount, "0 <= typeA && typeA < b3_shapeTypeCount")
	assert(0 <= typeB && typeB < ShapeTypeCount, "0 <= typeB && typeB < b3_shapeTypeCount")

	if contactRegisters[typeA][typeB].supported == false {
		// For example, no mesh vs mesh collision
		return
	}

	if contactRegisters[typeA][typeB].primary == false {
		// flip order
		createContact(world, shapeB, shapeA, childIndex)
		return
	}

	bodyA := arrayGet(world.bodies, int(shapeA.bodyId))
	bodyB := arrayGet(world.bodies, int(shapeB.bodyId))

	assert(bodyA.setIndex != disabledSet && bodyB.setIndex != disabledSet,
		"bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet")
	assert(bodyA.setIndex != staticSet || bodyB.setIndex != staticSet,
		"bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet")

	var setIndex int
	if bodyA.setIndex == awakeSet || bodyB.setIndex == awakeSet {
		setIndex = awakeSet
	} else {
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island

		// This is possible if a shape moves slightly then falls asleep
		setIndex = disabledSet
	}

	set := arrayGet(world.solverSets, setIndex)

	// Create contact key and contact
	contactId := allocId(&world.contactIdPool)
	if contactId == len(world.contacts) {
		emptyContact := contact{}
		arrayPush(&world.contacts, emptyContact)
	}

	shapeIdA := int(shapeA.id)
	shapeIdB := int(shapeB.id)

	contact := arrayGet(world.contacts, contactId)
	generation := contact.generation
	*contact = zeroContact // PORT: (b3Contact){ 0 }; the local contact shadows the type name
	contact.contactId = int32(contactId)
	contact.generation = generation + 1
	contact.setIndex = int32(setIndex)
	contact.colorIndex = NullIndex
	contact.localIndex = int32(len(set.contactIndices))
	contact.islandId = NullIndex
	contact.islandIndex = NullIndex
	contact.shapeIdA = int32(shapeIdA)
	contact.shapeIdB = int32(shapeIdB)
	contact.childIndex = int32(childIndex)
	contact.encodedBodySimA = int32(encodeBodySimIndex(bodyA))
	contact.encodedBodySimB = int32(encodeBodySimIndex(bodyB))

	if shapeA.typ == MeshShape || shapeA.typ == HeightShape {
		contact.flags |= simMeshContact
	} else if shapeA.typ == CompoundShape {
		child := GetCompoundChild(shapeA.compound, childIndex)
		if child.Type == MeshShape {
			contact.flags |= simMeshContact
		}
	}

	// todo impose these restrictions to make life easier
	assert(shapeB.typ == SphereShape || shapeB.typ == CapsuleShape || shapeB.typ == HullShape,
		"shapeB->type == b3_sphereShape || shapeB->type == b3_capsuleShape || shapeB->type == b3_hullShape")
	// B3_ASSERT( bodyB->type != b3_staticBody );

	// Is either body static?
	// Note: it is possible to have a dynamic mesh collide with a static convex shape. Maybe I should disallow this.
	if bodyA.typ == StaticBody || bodyB.typ == StaticBody {
		contact.flags |= contactStaticFlag
	}

	assert(shapeA.sensorIndex == NullIndex && shapeB.sensorIndex == NullIndex,
		"shapeA->sensorIndex == B3_NULL_INDEX && shapeB->sensorIndex == B3_NULL_INDEX")

	if (shapeA.flags&uint8(enableContactEvents)) != 0 || (shapeB.flags&uint8(enableContactEvents)) != 0 {
		contact.flags |= contactEnableContactEvents
	}

	if (shapeA.flags&uint8(enableSpeculative)) != 0 && (shapeB.flags&uint8(enableSpeculative)) != 0 {
		contact.flags |= enableSpeculativePoints
	}

	// Disabling speculative collision also disables recycling.
	if (bodyA.flags&enableContactRecycling) != 0 && (bodyB.flags&enableContactRecycling) != 0 &&
		(contact.flags&enableSpeculativePoints) != 0 {
		contact.flags |= contactRecycleFlag
	}

	// Connect to body A
	{
		contact.edges[0].bodyId = shapeA.bodyId
		contact.edges[0].prevKey = NullIndex
		contact.edges[0].nextKey = bodyA.headContactKey

		keyA := int32(contactId<<1) | 0
		headContactKey := bodyA.headContactKey
		if headContactKey != NullIndex {
			headContact := arrayGet(world.contacts, int(headContactKey>>1))
			headContact.edges[headContactKey&1].prevKey = keyA
		}
		bodyA.headContactKey = keyA
		bodyA.contactCount += 1
	}

	// Connect to body B
	{
		contact.edges[1].bodyId = shapeB.bodyId
		contact.edges[1].prevKey = NullIndex
		contact.edges[1].nextKey = bodyB.headContactKey

		keyB := int32(contactId<<1) | 1
		headContactKey := bodyB.headContactKey
		if bodyB.headContactKey != NullIndex {
			headContact := arrayGet(world.contacts, int(headContactKey>>1))
			headContact.edges[headContactKey&1].prevKey = keyB
		}
		bodyB.headContactKey = keyB
		bodyB.contactCount += 1
	}

	// Add to pair set for fast lookup
	pairKey := shapePairKey(shapeIdA, shapeIdB, childIndex)
	addKey(&world.broadPhase.pairSet, pairKey)

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	arrayPush(&set.contactIndices, int32(contactId))

	radiusA := float32(0.0)
	if typeA == SphereShape {
		radiusA = shapeA.sphere.Radius
	} else if typeA == CapsuleShape {
		radiusA = shapeA.capsule.Radius
	}

	radiusB := float32(0.0)
	if typeB == SphereShape {
		radiusB = shapeB.sphere.Radius
	} else if typeB == CapsuleShape {
		radiusB = shapeB.capsule.Radius
	}

	maxRadius := MaxFloat(radiusA, radiusB)

	// Assuming the rolling resistance doesn't change
	contact.rollingResistance =
		MaxFloat(getShapeMaterials(shapeA)[0].RollingResistance, getShapeMaterials(shapeB)[0].RollingResistance) *
			maxRadius

	if (shapeA.flags&uint8(enablePreSolveEvents)) != 0 || (shapeB.flags&uint8(enablePreSolveEvents)) != 0 {
		contact.flags |= simEnablePreSolveEvents
	}
}

// PORT: b3CreateContact's `(b3Contact){ 0 }` compound literal is the package var zeroContact
// (b3_zeroContact of contact_solver_wide.go, never written); createContact names its local
// `contact`, which shadows the type. Assigning it drops the slot's old manifold and triangle cache
// slices exactly as upstream overwrites the pointers.

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified

// destroyContact is b3DestroyContact.
func destroyContact(world *world, contact *contact, wakeBodies bool) {
	// Remove pair from set
	pairKey := shapePairKey(int(contact.shapeIdA), int(contact.shapeIdB), int(contact.childIndex))
	removeKey(&world.broadPhase.pairSet, pairKey)

	freeManifolds(world, contact.manifolds, int(contact.manifoldCount))
	contact.manifolds = nil
	contact.manifoldCount = 0

	edgeA := &contact.edges[0]
	edgeB := &contact.edges[1]

	bodyIdA := int(edgeA.bodyId)
	bodyIdB := int(edgeB.bodyId)
	bodyA := arrayGet(world.bodies, bodyIdA)
	bodyB := arrayGet(world.bodies, bodyIdB)

	flags := contact.flags
	touching := (flags & contactTouchingFlag) != 0

	// End touch event
	if touching && (flags&contactEnableContactEvents) != 0 {
		worldId := world.worldId
		shapeA := arrayGet(world.shapes, int(contact.shapeIdA))
		shapeB := arrayGet(world.shapes, int(contact.shapeIdB))
		shapeIdA := ShapeID{shapeA.id + 1, worldId, shapeA.generation}
		shapeIdB := ShapeID{shapeB.id + 1, worldId, shapeB.generation}

		contactId := ContactID{
			Index1:     contact.contactId + 1,
			World0:     world.worldId,
			Padding:    0,
			Generation: contact.generation,
		}

		event := ContactEndTouchEvent{
			ShapeIdA:  shapeIdA,
			ShapeIdB:  shapeIdB,
			ContactId: contactId,
		}

		arrayPush(&world.contactEndEvents[world.endEventArrayIndex], event)
	}

	// Remove from body A
	if edgeA.prevKey != NullIndex {
		prevContact := arrayGet(world.contacts, int(edgeA.prevKey>>1))
		prevEdge := &prevContact.edges[edgeA.prevKey&1]
		prevEdge.nextKey = edgeA.nextKey
	}

	if edgeA.nextKey != NullIndex {
		nextContact := arrayGet(world.contacts, int(edgeA.nextKey>>1))
		nextEdge := &nextContact.edges[edgeA.nextKey&1]
		nextEdge.prevKey = edgeA.prevKey
	}

	contactId := int(contact.contactId)

	edgeKeyA := int32(contactId<<1) | 0
	if bodyA.headContactKey == edgeKeyA {
		bodyA.headContactKey = edgeA.nextKey
	}

	bodyA.contactCount -= 1

	// Remove from body B
	if edgeB.prevKey != NullIndex {
		prevContact := arrayGet(world.contacts, int(edgeB.prevKey>>1))
		prevEdge := &prevContact.edges[edgeB.prevKey&1]
		prevEdge.nextKey = edgeB.nextKey
	}

	if edgeB.nextKey != NullIndex {
		nextContact := arrayGet(world.contacts, int(edgeB.nextKey>>1))
		nextEdge := &nextContact.edges[edgeB.nextKey&1]
		nextEdge.prevKey = edgeB.prevKey
	}

	edgeKeyB := int32(contactId<<1) | 1
	if bodyB.headContactKey == edgeKeyB {
		bodyB.headContactKey = edgeB.nextKey
	}

	bodyB.contactCount -= 1

	if (contact.flags & simMeshContact) != 0 {
		arrayDestroy(&contact.meshContact.triangleCache)
	}

	// Remove contact from the array that owns it
	if contact.islandId != NullIndex {
		unlinkContact(world, contact)
	}

	if contact.colorIndex != NullIndex {
		// contact is an active constraint
		assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")
		meshContact := (contact.flags & simMeshContact) != 0
		removeContactFromGraph(world, bodyIdA, bodyIdB, int(contact.colorIndex), int(contact.localIndex), meshContact)
	} else {
		// contact is non-touching or is sleeping or is a sensor
		assert(contact.setIndex != awakeSet || (contact.flags&contactTouchingFlag) == 0,
			"contact->setIndex != b3_awakeSet || ( contact->flags & b3_contactTouchingFlag ) == 0")
		set := arrayGet(world.solverSets, int(contact.setIndex))

		localIndex := int(contact.localIndex)
		movedIndex := arrayRemoveSwap(&set.contactIndices, localIndex)
		if movedIndex != NullIndex {
			movedContactIndex := int(set.contactIndices[localIndex])
			movedContact := arrayGet(world.contacts, movedContactIndex)
			movedContact.localIndex = int32(localIndex)
		}
	}

	// Free contact and id (preserve generation)
	contact.contactId = NullIndex
	contact.setIndex = NullIndex
	contact.colorIndex = NullIndex
	contact.localIndex = NullIndex
	freeId(&world.contactIdPool, contactId)

	if wakeBodies && touching {
		wakeBody(world, bodyA)
		wakeBody(world, bodyB)
	}
}

// computeConvexManifold is b3ComputeConvexManifold.
func computeConvexManifold(world *world, workerIndex int, contact *contact, shapeA *shape,
	xfA WorldTransform, shapeB *shape, xfB WorldTransform, arena arena) bool {
	typeA := shapeA.typ
	typeB := shapeB.typ

	cache := &contact.convexContact.cache

	pointCapacity := MaxManifoldPoints
	pointBuffer := bump[LocalManifoldPoint](&arena, pointCapacity)

	geomManifold := LocalManifold{}
	geomManifold.Points = pointBuffer

	transformBtoA := InvMulWorldTransforms(xfA, xfB)

	if typeA == SphereShape {
		assert(typeB == SphereShape, "typeB == b3_sphereShape")
		CollideSpheres(&geomManifold, pointCapacity, &shapeA.sphere, &shapeB.sphere, transformBtoA)
	} else if typeA == CapsuleShape {
		if typeB == SphereShape {
			CollideCapsuleAndSphere(&geomManifold, pointCapacity, &shapeA.capsule, &shapeB.sphere, transformBtoA)
		} else {
			assert(typeB == CapsuleShape, "typeB == b3_capsuleShape")
			CollideCapsules(&geomManifold, pointCapacity, &shapeA.capsule, &shapeB.capsule, transformBtoA)
		}
	} else {
		assert(typeA == HullShape, "typeA == b3_hullShape")

		if typeB == SphereShape {
			CollideHullAndSphere(&geomManifold, pointCapacity, shapeA.hull, &shapeB.sphere, transformBtoA,
				&cache.simplexCache)
		} else if typeB == CapsuleShape {
			CollideHullAndCapsule(&geomManifold, pointCapacity, shapeA.hull, &shapeB.capsule, transformBtoA,
				&cache.simplexCache)
		} else {
			assert(typeB == HullShape, "typeB == b3_hullShape")
			collideHullsAtWidth(&geomManifold, pointCapacity, shapeA.hull, shapeB.hull, transformBtoA, &cache.satCache,
				int(world.simdWidth))
			world.taskContexts[workerIndex].satCallCount += 1
			world.taskContexts[workerIndex].satCacheHitCount += int32(cache.satCache.Hit)
		}
	}

	if geomManifold.PointCount == 0 {
		if contact.manifoldCount > 0 {
			freeManifolds(world, contact.manifolds, int(contact.manifoldCount))
			contact.manifolds = nil
			contact.manifoldCount = 0
		}

		return false
	}

	var oldPoints [MaxManifoldPoints]ManifoldPoint
	oldCount := 0

	if contact.manifoldCount == 0 {
		contact.manifolds = allocateManifolds(world, 1)
		contact.manifoldCount = 1
	} else {
		oldCount = int(contact.manifolds[0].PointCount)
		copy(oldPoints[:oldCount], contact.manifolds[0].Points[:oldCount])
	}

	manifold := &contact.manifolds[0]
	assert(0 < geomManifold.PointCount && geomManifold.PointCount <= MaxManifoldPoints,
		"0 < geomManifold.pointCount && geomManifold.pointCount <= B3_MAX_MANIFOLD_POINTS")
	manifold.PointCount = geomManifold.PointCount

	matrixA := MakeMatrixFromQuat(xfA.Q)
	manifold.Normal = MulMV(matrixA, geomManifold.Normal)

	// Store point data in contact
	for i := 0; i < int(geomManifold.PointCount); i++ {
		source := &geomManifold.Points[i]
		target := &manifold.Points[i]

		// Contact points are computed in frame A
		target.AnchorA = MulMV(matrixA, source.Point)
		target.AnchorB = Add(target.AnchorA, SubPos(xfA.P, xfB.P))
		target.Separation = source.Separation
		target.FeatureId = makeFeatureId(source.Pair)
		target.TriangleIndex = NullIndex
		target.NormalVelocity = 0.0
	}

	// Copy impulses from old points
	for i := 0; i < int(geomManifold.PointCount); i++ {
		pt2 := &manifold.Points[i]
		pt2.TotalNormalImpulse = 0.0
		pt2.Persisted = false

		for j := 0; j < oldCount; j++ {
			pt1 := &oldPoints[j]

			if pt2.FeatureId == pt1.FeatureId {
				pt2.NormalImpulse = pt1.NormalImpulse
				pt2.Persisted = true

				// claimed
				pt1.FeatureId = math.MaxUint32

				break
			}
		}

		if pt2.Persisted == false {
			pt2.NormalImpulse = 0.0
		}
	}

	return true
}

// updateConvexContact is b3UpdateConvexContact.
func updateConvexContact(world *world, workerIndex int, contact *contact, shapeA *shape, xfA WorldTransform,
	shapeB *shape, xfB WorldTransform, flip bool, arena arena) bool {
	// Compute new manifold
	touching := computeConvexManifold(world, workerIndex, contact, shapeA, xfA, shapeB, xfB, arena)

	if touching == false {
		assert(contact.manifolds == nil && contact.manifoldCount == 0, "contact->manifolds == NULL && contact->manifoldCount == 0")
		return false
	}

	assert(contact.manifoldCount == 1, "contact->manifoldCount == 1")

	if flip {
		// Not flipping the feature ids because they just need to match and flipping is consistent.
		manifold := &contact.manifolds[0]
		manifold.Normal = Neg(manifold.Normal)
		pointCount := int(manifold.PointCount)
		for i := 0; i < pointCount; i++ {
			mp := &manifold.Points[i]
			mp.AnchorA, mp.AnchorB = mp.AnchorB, mp.AnchorA
		}
	}

	materialA := &getShapeMaterials(shapeA)[0]
	materialB := &getShapeMaterials(shapeB)[0]

	// Keep these updated in case the values on the shapes are modified
	contact.friction =
		world.frictionCallback(materialA.Friction, materialA.UserMaterialId, materialB.Friction, materialB.UserMaterialId)
	contact.restitution = world.restitutionCallback(materialA.Restitution, materialA.UserMaterialId, materialB.Restitution,
		materialB.UserMaterialId)

	if materialA.RollingResistance > 0.0 || materialB.RollingResistance > 0.0 {
		typeA := shapeA.typ
		typeB := shapeB.typ

		radiusA := float32(0.0)
		if typeA == SphereShape {
			radiusA = shapeA.sphere.Radius
		} else if typeA == CapsuleShape {
			radiusA = shapeA.capsule.Radius
		} else if typeA == HullShape {
			radiusA = 0.25 * shapeA.hull.InnerRadius
		}

		radiusB := float32(0.0)
		if typeB == SphereShape {
			radiusB = shapeB.sphere.Radius
		} else if typeB == CapsuleShape {
			radiusB = shapeB.capsule.Radius
		} else if typeB == HullShape {
			radiusB = 0.25 * shapeB.hull.InnerRadius
		}

		maxRadius := MaxFloat(radiusA, radiusB)
		contact.rollingResistance = MaxFloat(materialA.RollingResistance, materialB.RollingResistance) * maxRadius
	} else {
		contact.rollingResistance = 0.0
	}

	tangentVelocityA := RotateVector(xfA.Q, materialA.TangentVelocity)
	tangentVelocityB := RotateVector(xfB.Q, materialB.TangentVelocity)
	contact.tangentVelocity = Sub(tangentVelocityA, tangentVelocityB)

	if world.preSolveFcn != nil && (contact.flags&simEnablePreSolveEvents) != 0 {
		shapeIdA := ShapeID{shapeA.id + 1, world.worldId, shapeA.generation}
		shapeIdB := ShapeID{shapeB.id + 1, world.worldId, shapeB.generation}

		// this call assumes thread safety
		point := OffsetPos(xfA.P, contact.manifolds[0].Points[0].AnchorA)
		point = predictiveWorldPoint(world, contact, point)
		normal := contact.manifolds[0].Normal
		touching = world.preSolveFcn(shapeIdA, shapeIdB, point, normal, world.preSolveContext)
		if touching == false {
			rejectPredictiveContact(world, contact)
			// disable contact
			freeManifolds(world, contact.manifolds, int(contact.manifoldCount))
			contact.manifolds = nil
			contact.manifoldCount = 0

			// Invalidate recycling cache.
			contact.flags &^= relativeTransformValid
			return false
		}
	}

	if (shapeA.flags&uint8(enableHitEvents)) != 0 || (shapeB.flags&uint8(enableHitEvents)) != 0 {
		contact.flags |= simEnableHitEvent
	} else {
		contact.flags &^= simEnableHitEvent
	}

	return true
}

// updateContact is b3UpdateContact.
// Update the contact manifold and touching status.
// Note: do not assume the shape AABBs are overlapping or are valid.
func updateContact(world *world, workerIndex int, contact *contact, shapeA *shape, localCenterA Vec3, xfA WorldTransform,
	shapeB *shape, localCenterB Vec3, xfB WorldTransform, isFast bool, arena arena) bool {
	var touching bool

	assert(shapeB.typ != CompoundShape, "shapeB->type != b3_compoundShape")

	if shapeA.typ == CompoundShape {
		childIndex := int(contact.childIndex)
		child := GetCompoundChild(shapeA.compound, childIndex)

		// Temporary child shape to match existing function signatures
		childShapeA := *shapeA

		childShapeA.typ = child.Type

		// Handle child material for non-meshes.
		if child.Type != MeshShape {
			assert(0 <= child.MaterialIndices[0] && int32(child.MaterialIndices[0]) < shapeA.materialCount,
				"0 <= child.materialIndices[0] && child.materialIndices[0] < shapeA->materialCount")
			parentMaterials := getShapeMaterials(shapeA)
			childShapeA.material = parentMaterials[child.MaterialIndices[0]]
			childShapeA.materials = nil
			childShapeA.materialCount = 1
		}

		if child.Type == CapsuleShape {
			childShapeA.capsule = child.Capsule
			if shapeB.typ == HullShape {
				// Flip
				flip := true
				touching = updateConvexContact(world, workerIndex, contact, shapeB, xfB, &childShapeA, xfA, flip, arena)
			} else {
				flip := false
				touching = updateConvexContact(world, workerIndex, contact, &childShapeA, xfA, shapeB, xfB, flip, arena)
			}
		} else if child.Type == HullShape {
			childShapeA.hull = child.Hull
			xfChild := MulWorldTransforms(xfA, child.Transform)
			flip := false
			touching = updateConvexContact(world, workerIndex, contact, &childShapeA, xfChild, shapeB, xfB, flip, arena)
		} else if child.Type == MeshShape {
			childShapeA.mesh = child.Mesh
			xfChild := MulWorldTransforms(xfA, child.Transform)

			touching = computeMeshManifolds(world, workerIndex, contact, &childShapeA, child.MaterialIndices, xfChild, shapeB,
				xfB, isFast, arena)

			if touching && ((shapeA.flags&uint8(enableHitEvents)) != 0 || (shapeB.flags&uint8(enableHitEvents)) != 0) {
				contact.flags |= simEnableHitEvent
			} else {
				contact.flags &^= simEnableHitEvent
			}

			assert((touching == true && contact.manifoldCount > 0) ||
				(touching == false && contact.manifoldCount == 0),
				"( touching == true && contact->manifoldCount > 0 ) || ( touching == false && contact->manifoldCount == 0 )")
		} else {
			assert(child.Type == SphereShape, "child.type == b3_sphereShape")

			childShapeA.sphere = child.Sphere
			if shapeB.typ == CapsuleShape || shapeB.typ == HullShape {
				// Flip
				flip := true
				touching = updateConvexContact(world, workerIndex, contact, shapeB, xfB, &childShapeA, xfA, flip, arena)
			} else {
				flip := false
				touching = updateConvexContact(world, workerIndex, contact, &childShapeA, xfA, shapeB, xfB, flip, arena)
			}
		}

		// The anchor is relative to the child origin but oriented in world space.
		// Offset the anchor to be relative to the compound origin.
		manifoldCount := int(contact.manifoldCount)
		offset := RotateVector(xfA.Q, child.Transform.P)
		for i := 0; i < manifoldCount; i++ {
			manifold := &contact.manifolds[i]
			pointCount := int(manifold.PointCount)
			for j := 0; j < pointCount; j++ {
				mp := &manifold.Points[j]
				mp.AnchorA = Add(mp.AnchorA, offset)
			}
		}
	} else if shapeA.typ == MeshShape || shapeA.typ == HeightShape {
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = computeMeshManifolds(world, workerIndex, contact, shapeA, nil, xfA, shapeB, xfB, isFast, arena)

		if touching && ((shapeA.flags&uint8(enableHitEvents)) != 0 || (shapeB.flags&uint8(enableHitEvents)) != 0) {
			contact.flags |= simEnableHitEvent
		} else {
			contact.flags &^= simEnableHitEvent
		}

		assert((touching == true && contact.manifoldCount > 0) || (touching == false && contact.manifoldCount == 0),
			"( touching == true && contact->manifoldCount > 0 ) || ( touching == false && contact->manifoldCount == 0 )")
	} else {
		// Convex-vs-convex
		flip := false
		touching = updateConvexContact(world, workerIndex, contact, shapeA, xfA, shapeB, xfB, flip, arena)
	}

	if !touching && updatePredictiveContact(world, workerIndex, contact, shapeA, shapeB, arena) {
		return true
	}
	if touching {
		centerA := RotateVector(xfA.Q, localCenterA)
		centerB := RotateVector(xfB.Q, localCenterB)

		// Adjust anchors to be relative to center of mass
		for i := 0; i < int(contact.manifoldCount); i++ {
			manifold := &contact.manifolds[i]
			for j := 0; j < int(manifold.PointCount); j++ {
				mp := &manifold.Points[j]
				mp.AnchorA = Sub(mp.AnchorA, centerA)
				mp.AnchorB = Sub(mp.AnchorB, centerB)
			}
		}

		contact.flags |= simTouchingFlag
	} else {
		contact.flags &^= simTouchingFlag
	}

	return touching
}
