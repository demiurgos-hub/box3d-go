// Port of src/physics_world.h and src/physics_world.c from Box3D
// (https://github.com/erincatto/box3d) at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import (
	"fmt"
	"unsafe"
)

// PORT: physics_world.c is split in two Go files (spec: left to the builder). The world queries
// (WorldQueryContext … b3World_CastMover) are in physics_world_queries.go; everything else is here
// in upstream order.

// ---- physics_world.h ----

const (
	debugPointCapacity = 64 // B3_DEBUG_POINT_CAPACITY
	debugLineCapacity  = 64 // B3_DEBUG_LINE_CAPACITY
)

// b3SetType
const (
	staticSet        = 0
	disabledSet      = 1
	awakeSet         = 2
	firstSleepingSet = 3
)

// debugPoint is b3DebugPoint.
type debugPoint struct {
	p     Pos
	label int32
	value float32
	color HexColor
}

// debugLine is b3DebugLine.
type debugLine struct {
	p1, p2 Pos
	label  int32
	color  HexColor
}

// taskContext is b3TaskContext: per thread task storage
type taskContext struct {
	arena arena

	// Collect per thread sensor continuous hit events.
	sensorHits []sensorHit

	// Broad-phase pairs.
	pairKeys []uint64

	// These bits align with the b3ConstraintGraph::contactBlocks and signal a change in contact status
	contactStateBitSet bitSet

	// These bits align with the joint id capacity and signal a change in contact status
	jointStateBitSet bitSet

	// These bits align with the contact id capacity and signal a hit event.
	hitEventBitSet bitSet

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	hasHitEvents bool

	// Used to put islands to sleep
	awakeIslandBitSet bitSet

	// Per worker split island candidate
	splitSleepTime float32
	splitIslandId  int32

	// Profiling
	satCallCount       int32
	satCacheHitCount   int32
	distanceIterations int32
	pushBackIterations int32
	rootIterations     int32

	// Number of contacts recycled this step (collide pass).
	recycledContactCount int32

	points     [debugPointCapacity]debugPoint
	pointCount int32

	lines     [debugLineCapacity]debugLine
	lineCount int32

	manifoldCounts [ContactManifoldCountBuckets]int32
	// PORT: C stack locals of b3SolveContinuous and b3SelfPairsTask / b3CrossPairsTask that are
	// handed to queries as pointers; per worker storage so that Step does not allocate.
	continuous  continuousContext
	pairContext pairContext

	// Prevent false sharing
	cacheLine [64]byte
}

// stepScratch holds C stack locals of one world step that the task system or a query receives
// pointers to. PORT: as Go locals they would escape to the heap on every step; the world owns
// them instead (spec: scratch owned by the world/step context, reused across steps). Each user
// clears its part before use, as the C locals are (re)initialized.
type stepScratch struct {
	stepContext       stepContext                           // b3World_Step context
	widePrepareSpans  [GraphColorCount + 1]widePrepareSpan  // b3SolverStep
	meshPrepareSpans  [GraphColorCount + 1]meshPrepareSpan  // b3SolverStep
	jointPrepareSpans [GraphColorCount + 1]jointPrepareSpan // b3SolverStep
	overflowSpans     [2]contactPrepareSpan                 // b3SolverStep
	workerContexts    [MaxWorkers]workerContext             // b3SolverStep
	crossSeeds        [2 * crossSeedCount]nodePair          // b3UpdateBroadPhasePairs
	crossContext      crossContext                          // b3UpdateBroadPhasePairs
	parallelForShared parallelForShared                     // b3ParallelFor
	parallelForTasks  [MaxWorkers]parallelForTask           // b3ParallelFor
}

// world is b3World.
// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
type world struct {
	speculativeCCD  speculativeCCDScratch
	stack           stack
	broadPhase      broadPhase
	constraintGraph constraintGraph

	// Manifold allocators have one allocator for each manifold count.
	// PORT: allocator i hands out []Manifold of i + 1 manifolds (see blockAllocator).
	manifoldAllocators     []blockAllocator[Manifold]
	manifoldAllocatorMutex *mutex

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b3Body.
	bodyIdPool idPool

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	bodies []body

	// Provides free list for solver sets.
	solverSetIdPool idPool

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	solverSets []solverSet

	// Used to create stable ids for joints
	jointIdPool idPool

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	joints []joint

	// Used to create stable ids for contacts
	contactIdPool idPool

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	contacts []contact

	// Used to create stable ids for islands
	islandIdPool idPool

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	islands []island

	shapeIdPool idPool

	// These are sparse arrays that point into the pools above
	shapes []shape

	// This follows the shape array and is split out for the hot path in the narrow phase.
	fatAABBs []AABB

	// Compound shapes are static only, so only the broad-phase static pass can meet one. This
	// lets a world without compounds skip the shape type read in the pair batch.
	compoundShapeCount int32

	// Reference counted store of shared hull data keyed by content. Shapes hold a
	// pointer to the hull stored in the db. Type erased to avoid leaking the verstable map
	// type into this header.
	// PORT: typed as the hull map (Go has no header leak to avoid).
	hullDatabase *hullMap

	// Name cache for shape and body names. This works with recording.
	names nameCache

	// This is a dense array of sensor data.
	sensors []sensor

	// Per thread storage
	taskContexts       []taskContext
	sensorTaskContexts []sensorTaskContext

	bodyMoveEvents     []BodyMoveEvent
	sensorBeginEvents  []SensorBeginTouchEvent
	contactBeginEvents []ContactBeginTouchEvent

	// End events are double buffered so that the user doesn't need to flush events
	sensorEndEvents    [2][]SensorEndTouchEvent
	contactEndEvents   [2][]ContactEndTouchEvent
	endEventArrayIndex int32

	contactHitEvents []ContactHitEvent
	jointEvents      []JointEvent

	// Used to track debug draw
	debugBodySet          bitSet
	debugJointSet         bitSet
	debugContactSet       bitSet
	debugIslandSet        bitSet
	createDebugShape      CreateDebugShapeCallback
	destroyDebugShape     DestroyDebugShapeCallback
	userDebugShapeContext any

	// Id that is incremented every time step
	stepIndex uint64

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	splitIslandId int32

	gravity                      Vec3
	hitEventThreshold            float32
	restitutionThreshold         float32
	restitutionIterations        int32
	enableRestitutionPropagation bool
	maxLinearSpeed               float32
	isotropicInertiaTolerance    float32
	contactSpeed                 float32
	contactHertz                 float32
	contactDampingRatio          float32
	contactRecycleDistance       float32

	frictionCallback    FrictionCallback
	restitutionCallback RestitutionCallback

	generation uint16

	profile          Profile
	satCallCount     int32
	satCacheHitCount int32
	manifoldCounts   [ContactManifoldCountBuckets]int32

	maxCapacity Capacity

	preSolveFcn     PreSolveFcn
	preSolveContext any

	customFilterFcn     CustomFilterFcn
	customFilterContext any

	workerCount     int32
	simdWidth       int32
	enqueueTaskFcn  EnqueueTaskCallback
	finishTaskFcn   FinishTaskCallback
	userTaskContext any
	userTreeTask    any

	scheduler *scheduler

	userData any

	// Non-NULL while a recording session is active. Set by b3World_StartRecording,
	// cleared by b3World_StopRecording. Hooks in mutators check this before writing.
	recording *Recording

	// latest inverse sub-step
	inv_h float32

	// latest inverse full-step
	inv_dt float32

	activeTaskCount int32
	taskCount       int32

	worldId uint16

	enableSleep bool

	// This indicates there is a world write operation in progress. This is for debugging and
	// not a real mutex. This should have minimal performance impact.
	locked             bool
	enableWarmStarting bool
	enableContinuous   bool
	enableSpeculative  bool
	inUse              bool

	// PORT: world owned storage for step locals, see stepScratch.
	stepScratch stepScratch
}

// PORT: b3GetUnlockedWorldFromId, b3GetWorldFromId, b3GetUnlockedWorld, b3GetWorld,
// b3World_RebuildStaticTree, b3ValidateConnectivity, b3ValidateSolverSets, b3ValidateContacts,
// b3AddHullToDatabase, b3AddOwnedHullToDatabase and b3RemoveHullFromDatabase are declared here and
// defined in physics_world.c (below and in physics_world_queries.go).

// allocateManifolds is b3AllocateManifolds.
// PORT: returns a slice of count manifolds (nil for zero) from allocator count - 1.
func allocateManifolds(world *world, count int) []Manifold {
	if count == 0 {
		return nil
	}

	index := count - 1

	// Need lock because this is called from the parallel narrow phase
	lockMutex(world.manifoldAllocatorMutex)
	currentCount := len(world.manifoldAllocators)
	for i := currentCount; i < count; i++ {
		allocator := createBlockAllocator[Manifold](i+1, 2*blockSize)
		arrayPush(&world.manifoldAllocators, allocator)
	}

	allocator := arrayGet(world.manifoldAllocators, index)
	manifolds := allocateElement(allocator)
	unlockMutex(world.manifoldAllocatorMutex)
	clear(manifolds[:count])
	return manifolds
}

// freeManifolds is b3FreeManifolds.
func freeManifolds(world *world, manifolds []Manifold, count int) {
	if count == 0 {
		return
	}

	index := count - 1
	lockMutex(world.manifoldAllocatorMutex)
	allocator := arrayGet(world.manifoldAllocators, index)
	freeElement(allocator, manifolds)
	unlockMutex(world.manifoldAllocatorMutex)
}

// ---- physics_world.c ----

// PORT: the B3_MAX_WORLDS _Static_asserts become overflowing constant pairs.
const (
	_ uint = MaxWorlds - 1
	_ uint = 0xFFFF - 1 - MaxWorlds
)

// worlds is b3_worlds.
var worlds [MaxWorlds]world

// worldCount is b3_worldCount.
var worldCount atomicInt

// maxWorldCount is b3_maxWorldCount.
var maxWorldCount int

// addHullToDatabase is b3AddHullToDatabase.
// Register a hull in the world database, returning the owned shared copy. Identical hulls
// share one copy with a reference count. The input may be freed after this call.
func addHullToDatabase(world *world, src *HullData) *HullData {
	database := world.hullDatabase

	// Compare by content to de-duplicate. Not trusting the hash.
	itr := database.get(src)
	if database.isEnd(itr) == false {
		// Bump reference count.
		itr.data.val += 1
		return itr.data.key
	}

	clone := CloneHull(src)
	assert(clone != nil, "clone != NULL")

	// Start with reference count of 1.
	database.insert(clone, 1)
	return clone
}

// addOwnedHullToDatabase is b3AddOwnedHullToDatabase.
// Like b3AddHullToDatabase but takes ownership of a heap hull: inserted directly on a miss,
// freed on a hit. Avoids cloning data the caller already allocated.
func addOwnedHullToDatabase(world *world, owned *HullData) *HullData {
	database := world.hullDatabase

	itr := database.get(owned)
	if database.isEnd(itr) == false {
		itr.data.val += 1
		DestroyHull(owned)
		return itr.data.key
	}

	// Take ownership of input hull.
	database.insert(owned, 1)
	return owned
}

// removeHullFromDatabase is b3RemoveHullFromDatabase.
// Release a reference to a shared hull. The owned copy is freed when the count reaches zero.
func removeHullFromDatabase(world *world, data *HullData) {
	database := world.hullDatabase

	itr := database.get(data)
	assert(database.isEnd(itr) == false, "b3HullMap_is_end( itr ) == false")

	itr.data.val -= 1
	if itr.data.val == 0 {
		// Erase through the iterator we already have so the lookup runs once.
		owned := itr.data.key
		database.eraseItr(itr)
		DestroyHull(owned)
	}
}

// getUnlockedWorldFromId is b3GetUnlockedWorldFromId.
func getUnlockedWorldFromId(id WorldID) *world {
	assert(1 <= id.Index1 && id.Index1 <= MaxWorlds, "1 <= id.index1 && id.index1 <= B3_MAX_WORLDS")
	world := &worlds[id.Index1-1]
	assert(id.Index1 == world.worldId+1, "id.index1 == world->worldId + 1")
	assert(id.Generation == world.generation, "id.generation == world->generation")

	// A world accessed from an id should not be locked
	if world.locked {
		assert(false, "false")
		return nil
	}
	return world
}

// getWorldFromId is b3GetWorldFromId.
func getWorldFromId(id WorldID) *world {
	assert(1 <= id.Index1 && id.Index1 <= MaxWorlds, "1 <= id.index1 && id.index1 <= B3_MAX_WORLDS")
	world := &worlds[id.Index1-1]
	assert(id.Index1 == world.worldId+1, "id.index1 == world->worldId + 1")
	assert(id.Generation == world.generation, "id.generation == world->generation")
	return world
}

// getWorld is b3GetWorld.
func getWorld(index int) *world {
	assert(0 <= index && index < MaxWorlds, "0 <= index && index < B3_MAX_WORLDS")
	world := &worlds[index]
	assert(int(world.worldId) == index, "world->worldId == index")
	return world
}

// getUnlockedWorld is b3GetUnlockedWorld.
func getUnlockedWorld(index int) *world {
	assert(0 <= index && index < MaxWorlds, "0 <= index && index < B3_MAX_WORLDS")
	world := &worlds[index]
	assert(int(world.worldId) == index, "world->worldId == index")
	if world.locked {
		assert(false, "false")
		return nil
	}

	return world
}

// defaultAddTaskFcn is b3DefaultAddTaskFcn.
func defaultAddTaskFcn(task TaskCallback, taskContext any, userContext any, name string) any {
	task(taskContext)
	return nil
}

// defaultFinishTaskFcn is b3DefaultFinishTaskFcn.
func defaultFinishTaskFcn(userTask any, userContext any) {
}

// defaultFrictionCallback is b3DefaultFrictionCallback.
func defaultFrictionCallback(frictionA float32, materialA uint64, frictionB float32, materialB uint64) float32 {
	return sqrtf(frictionA * frictionB)
}

// defaultRestitutionCallback is b3DefaultRestitutionCallback.
func defaultRestitutionCallback(restitutionA float32, materialA uint64, restitutionB float32, materialB uint64) float32 {
	return MaxFloat(restitutionA, restitutionB)
}

// createWorkerContexts is b3CreateWorkerContexts.
func createWorkerContexts(world *world) {
	arrayResize(&world.taskContexts, int(world.workerCount))
	arrayMemZero(world.taskContexts)

	arrayResize(&world.sensorTaskContexts, int(world.workerCount))
	arrayMemZero(world.sensorTaskContexts)

	for i := 0; i < int(world.workerCount); i++ {
		world.taskContexts[i].arena = createArena(128 * 1024)
		arrayReserve(&world.taskContexts[i].sensorHits, 8)
		arrayReserve(&world.taskContexts[i].pairKeys, 64)
		world.taskContexts[i].contactStateBitSet = createBitSet(1024)
		world.taskContexts[i].hitEventBitSet = createBitSet(1024)
		world.taskContexts[i].hasHitEvents = false
		world.taskContexts[i].jointStateBitSet = createBitSet(1024)
		world.taskContexts[i].awakeIslandBitSet = createBitSet(256)
		world.taskContexts[i].splitIslandId = NullIndex

		world.sensorTaskContexts[i].eventBits = createBitSet(128)
	}
}

// destroyWorkerContexts is b3DestroyWorkerContexts.
func destroyWorkerContexts(world *world) {
	for i := 0; i < int(world.workerCount); i++ {
		destroyArena(&world.taskContexts[i].arena)
		arrayDestroy(&world.taskContexts[i].sensorHits)
		arrayDestroy(&world.taskContexts[i].pairKeys)
		destroyBitSet(&world.taskContexts[i].contactStateBitSet)
		destroyBitSet(&world.taskContexts[i].hitEventBitSet)
		destroyBitSet(&world.taskContexts[i].jointStateBitSet)
		destroyBitSet(&world.taskContexts[i].awakeIslandBitSet)

		destroyBitSet(&world.sensorTaskContexts[i].eventBits)
	}

	arrayDestroy(&world.taskContexts)
	arrayDestroy(&world.sensorTaskContexts)
}

// CreateWorld is b3CreateWorld: create a world for rigid body simulation. A world contains bodies,
// shapes, and constraints. You make create up to 128 worlds. Each world is completely independent
// and may be simulated in parallel. Returns the world id.
func CreateWorld(def *WorldDef) WorldID {
	checkDef(def.internalValue)

	assert(LinearSlop() <= MeshRestOffset(), "B3_LINEAR_SLOP <= B3_MESH_REST_OFFSET")
	assert(MeshRestOffset() < SpeculativeDistance(), "B3_MESH_REST_OFFSET < B3_SPECULATIVE_DISTANCE")

	worldId := NullIndex
	for i := 0; i < MaxWorlds; i++ {
		if worlds[i].inUse == false {
			worldId = i
			break
		}
	}

	if worldId == NullIndex {
		log("B3_MAX_WORLDS of %d exceeded!!!", MaxWorlds)
		assert(worldId != NullIndex, "worldId != B3_NULL_INDEX")
		return WorldID{}
	}

	// b3Log( "b3_lengthUnitsPerMeter = %g", b3_lengthUnitsPerMeter );

	oldCount := atomicFetchAddInt(&worldCount, 1)
	maxWorldCount = MaxInt(maxWorldCount, int(oldCount)+1)

	// b3Log( "Created world %d", worldId );

	initializeContactRegisters()

	world := &worlds[worldId]
	revision := world.generation

	// PORT: memset( world, 0, sizeof( b3World ) )
	clear(unsafe.Slice(world, 1))

	world.simdWidth = int32(GetSIMDWidth())

	world.worldId = uint16(worldId)
	consistentParallelWarmStart[worldId] = false
	world.generation = revision
	world.inUse = true

	world.stack = createStack(2048)

	arrayReserve(&world.manifoldAllocators, 16)
	world.manifoldAllocatorMutex = createMutex()

	createBroadPhase(&world.broadPhase, &def.Capacity)
	createGraph(&world.constraintGraph, 16)

	// pools
	world.bodyIdPool = createIdPool()

	bodyCapacity := MaxInt(16, int(def.Capacity.StaticBodyCount+def.Capacity.DynamicBodyCount))
	arrayReserve(&world.bodies, bodyCapacity)
	arrayReserve(&world.solverSets, 8)

	// add empty static, active, and disabled body sets
	world.solverSetIdPool = createIdPool()
	var set solverSet

	// static set
	set.setIndex = int32(allocId(&world.solverSetIdPool))
	arrayPush(&world.solverSets, set)
	arrayReserve(&world.solverSets[staticSet].bodySims, MaxInt(16, int(def.Capacity.StaticBodyCount)))
	assert(world.solverSets[staticSet].setIndex == staticSet, "world->solverSets.data[b3_staticSet].setIndex == b3_staticSet")

	// disabled set
	set.setIndex = int32(allocId(&world.solverSetIdPool))
	arrayPush(&world.solverSets, set)
	assert(world.solverSets[disabledSet].setIndex == disabledSet,
		"world->solverSets.data[b3_disabledSet].setIndex == b3_disabledSet")

	// awake set
	set.setIndex = int32(allocId(&world.solverSetIdPool))
	arrayPush(&world.solverSets, set)
	arrayReserve(&world.solverSets[awakeSet].bodySims, MaxInt(16, int(def.Capacity.DynamicBodyCount)))
	arrayReserve(&world.solverSets[awakeSet].bodyStates, MaxInt(16, int(def.Capacity.DynamicBodyCount)))
	arrayReserve(&world.solverSets[awakeSet].contactIndices, MaxInt(16, int(def.Capacity.ContactCount)))
	assert(world.solverSets[awakeSet].setIndex == awakeSet, "world->solverSets.data[b3_awakeSet].setIndex == b3_awakeSet")

	world.shapeIdPool = createIdPool()

	shapeCapacity := MaxInt(16, int(def.Capacity.StaticShapeCount+def.Capacity.DynamicShapeCount))
	arrayReserve(&world.shapes, shapeCapacity)
	arrayReserve(&world.fatAABBs, shapeCapacity)

	// PORT: b3Alloc( sizeof( b3HullMap ) ) + b3HullMap_init
	world.hullDatabase = new(hullMap)
	hullMapInit(world.hullDatabase)

	world.names = createNameCache()

	world.contactIdPool = createIdPool()
	arrayReserve(&world.contacts, MaxInt(16, int(def.Capacity.ContactCount)))

	world.jointIdPool = createIdPool()
	arrayReserve(&world.joints, 16)

	world.islandIdPool = createIdPool()
	arrayReserve(&world.islands, MaxInt(16, int(def.Capacity.DynamicBodyCount)))

	arrayReserve(&world.sensors, 4)

	arrayReserve(&world.bodyMoveEvents, 4)
	arrayReserve(&world.sensorBeginEvents, 4)
	arrayReserve(&world.sensorEndEvents[0], 4)
	arrayReserve(&world.sensorEndEvents[1], 4)
	arrayReserve(&world.contactBeginEvents, 4)
	arrayReserve(&world.contactEndEvents[0], 4)
	arrayReserve(&world.contactEndEvents[1], 4)
	arrayReserve(&world.contactHitEvents, 4)
	arrayReserve(&world.jointEvents, 4)
	world.endEventArrayIndex = 0

	world.stepIndex = 0
	world.splitIslandId = NullIndex
	world.activeTaskCount = 0
	world.taskCount = 0
	world.gravity = def.Gravity
	world.hitEventThreshold = def.HitEventThreshold
	world.restitutionThreshold = def.RestitutionThreshold
	world.restitutionIterations = int32(ClampInt(int(def.RestitutionIterations), 0, MaxRestitutionIterations))
	world.enableRestitutionPropagation = def.EnableRestitutionPropagation
	world.maxLinearSpeed = def.MaximumLinearSpeed
	world.isotropicInertiaTolerance = ClampFloat(def.IsotropicInertiaTolerance, 0.0, 1.0)
	world.contactSpeed = def.ContactSpeed
	world.contactHertz = def.ContactHertz
	world.contactDampingRatio = def.ContactDampingRatio
	world.contactRecycleDistance = ContactRecycleDistance()

	if def.FrictionCallback == nil {
		world.frictionCallback = defaultFrictionCallback
	} else {
		world.frictionCallback = def.FrictionCallback
	}

	if def.RestitutionCallback == nil {
		world.restitutionCallback = defaultRestitutionCallback
	} else {
		world.restitutionCallback = def.RestitutionCallback
	}

	world.enableSleep = def.EnableSleep
	world.locked = false
	world.enableWarmStarting = true
	world.enableContinuous = def.EnableContinuous
	world.enableSpeculative = true
	world.userTreeTask = nil
	world.userData = def.UserData

	if def.WorkerCount > 0 && def.EnqueueTask != nil && def.FinishTask != nil {
		// External task system
		world.workerCount = int32(MinInt(int(def.WorkerCount), MaxWorkers))
		world.enqueueTaskFcn = def.EnqueueTask
		world.finishTaskFcn = def.FinishTask
		world.userTaskContext = def.UserTaskContext
		world.scheduler = nil
	} else if def.WorkerCount > 1 {
		// Built-in scheduler
		world.workerCount = int32(MinInt(int(def.WorkerCount), MaxWorkers))
		world.scheduler = createScheduler(int(world.workerCount))
		world.enqueueTaskFcn = schedulerEnqueueTask
		world.finishTaskFcn = schedulerFinishTask
		world.userTaskContext = world.scheduler
	} else {
		// Serial fallback
		world.workerCount = 1
		world.enqueueTaskFcn = defaultAddTaskFcn
		world.finishTaskFcn = defaultFinishTaskFcn
		world.userTaskContext = nil
		world.scheduler = nil
	}

	createWorkerContexts(world)

	world.debugBodySet = createBitSet(256)
	world.debugJointSet = createBitSet(256)
	world.debugContactSet = createBitSet(256)
	world.debugIslandSet = createBitSet(256)
	world.createDebugShape = def.CreateDebugShape
	world.destroyDebugShape = def.DestroyDebugShape
	world.userDebugShapeContext = def.UserDebugShapeContext

	// add one to worldId so that 0 represents a null b3WorldId
	return WorldID{uint16(worldId + 1), world.generation}
}

// Destroy is b3DestroyWorld: destroy a world.
func (worldId WorldID) Destroy() {
	atomicFetchAddInt(&worldCount, -1)

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	world.locked = true

	// Detach any recording before teardown. The user owns and frees the recording buffer.
	stopRecordingInternal(world)

	if world.scheduler != nil {
		destroyScheduler(world.scheduler)
		world.scheduler = nil
	}

	destroyBitSet(&world.debugBodySet)
	destroyBitSet(&world.debugJointSet)
	destroyBitSet(&world.debugContactSet)
	destroyBitSet(&world.debugIslandSet)

	destroyWorkerContexts(world)

	arrayDestroy(&world.bodyMoveEvents)
	arrayDestroy(&world.sensorBeginEvents)
	arrayDestroy(&world.sensorEndEvents[0])
	arrayDestroy(&world.sensorEndEvents[1])
	arrayDestroy(&world.contactBeginEvents)
	arrayDestroy(&world.contactEndEvents[0])
	arrayDestroy(&world.contactEndEvents[1])
	arrayDestroy(&world.contactHitEvents)
	arrayDestroy(&world.jointEvents)

	sensorCount := len(world.sensors)
	for i := 0; i < sensorCount; i++ {
		arrayDestroy(&world.sensors[i].hits)
		arrayDestroy(&world.sensors[i].overlaps1)
		arrayDestroy(&world.sensors[i].overlaps2)
	}

	arrayDestroy(&world.sensors)
	arrayDestroy(&world.bodies)

	shapeCapacity := len(world.shapes)
	shapes := world.shapes
	for i := 0; i < shapeCapacity; i++ {
		shape := &shapes[i]
		if shape.id != NullIndex {
			destroyShapeAllocations(world, &shapes[i])
		}
	}

	contactCapacity := len(world.contacts)
	contacts := world.contacts
	for i := 0; i < contactCapacity; i++ {
		contact := &contacts[i]
		if contact.contactId != NullIndex {
			if contact.flags&simMeshContact != 0 {
				arrayDestroy(&contact.meshContact.triangleCache)
			}
		}
	}

	// Destroying every shape above released all hull references, so the database is empty.
	assert(world.hullDatabase.size() == 0, "b3HullMap_size( (b3HullMap*)world->hullDatabase ) == 0")
	world.hullDatabase.cleanup()
	world.hullDatabase = nil

	destroyNameCache(&world.names)

	arrayDestroy(&world.shapes)
	arrayDestroy(&world.fatAABBs)
	arrayDestroy(&world.contacts)
	arrayDestroy(&world.joints)

	for i := 0; i < len(world.islands); i++ {
		arrayDestroy(&world.islands[i].bodies)
		arrayDestroy(&world.islands[i].contacts)
		arrayDestroy(&world.islands[i].joints)
	}
	arrayDestroy(&world.islands)

	// Destroy solver sets
	setCapacity := len(world.solverSets)
	for i := 0; i < setCapacity; i++ {
		set := &world.solverSets[i]
		if set.setIndex != NullIndex {
			destroySolverSet(world, i)
		}
	}

	arrayDestroy(&world.solverSets)

	destroyGraph(&world.constraintGraph)
	destroyBroadPhase(&world.broadPhase)

	destroyIdPool(&world.bodyIdPool)
	destroyIdPool(&world.shapeIdPool)
	destroyIdPool(&world.contactIdPool)
	destroyIdPool(&world.jointIdPool)
	destroyIdPool(&world.islandIdPool)
	destroyIdPool(&world.solverSetIdPool)

	for i := 0; i < len(world.manifoldAllocators); i++ {
		destroyBlockAllocator(&world.manifoldAllocators[i])
	}
	arrayDestroy(&world.manifoldAllocators)
	destroyMutex(world.manifoldAllocatorMutex)

	destroyStack(&world.stack)

	// Wipe world but preserve generation
	generation := world.generation
	// PORT: memset( world, 0, sizeof( b3World ) )
	clear(unsafe.Slice(world, 1))
	world.generation = generation + 1

	// b3Log( "Destroyed world %d", worldId.index1 - 1 );
}

// GetWorldCount is b3GetWorldCount: get the number of active worlds.
func GetWorldCount() int {
	return int(atomicLoadInt(&worldCount))
}

// GetMaxWorldCount is b3GetMaxWorldCount: get the maximum number of worlds that were active at the
// same time.
func GetMaxWorldCount() int {
	return maxWorldCount
}

// prefetchContact is b3PrefetchContact.
// Issues T0 prefetches across the cache lines of a b3Contact (216 B / 4 lines).
// Used to hide the random-access latency of contact lookups while we work on an
// earlier index.
// PORT: b3Prefetch is dropped (platform.go), so this does nothing.
func prefetchContact(contact *contact) {
}

// resolveContactBodySim is b3ResolveContactBodySim.
// PORT: awakeSims and staticSims are the set slices; staticSims - encodedIndex - 2 is an index.
func resolveContactBodySim(world *world, awakeSims, staticSims []bodySim, encodedIndex int, bodyId int) *bodySim {
	if encodedIndex >= 0 {
		return &awakeSims[encodedIndex]
	}

	if isStaticSimIndex(encodedIndex) {
		return &staticSims[-encodedIndex-2]
	}

	body := arrayGet(world.bodies, bodyId)
	set := arrayGet(world.solverSets, int(body.setIndex))
	return arrayGet(set.bodySims, int(body.localIndex))
}

// collideTask is b3CollideTask.
func collideTask(startIndex, endIndex, workerIndex int, context any) {
	stepContext := context.(*stepContext)
	world := stepContext.world
	graph := &world.constraintGraph
	taskContext := &world.taskContexts[workerIndex]
	contactIndices := stepContext.awakeContactIndices
	contacts := world.contacts
	shapes := world.shapes
	fatAABBs := world.fatAABBs
	awakeSims := world.solverSets[awakeSet].bodySims
	staticSims := world.solverSets[staticSet].bodySims

	assert(startIndex < endIndex, "startIndex < endIndex")

	recycleDistance := world.contactRecycleDistance
	speculativeDistance := SpeculativeDistance()
	recycleDistanceNonTouching := MinFloat(recycleDistance, speculativeDistance)

	const contactPrefetchDistance = 4
	prefetchEnd := endIndex - contactPrefetchDistance

	for i := startIndex; i < endIndex; i++ {
		if i < prefetchEnd {
			prefetchContact(&contacts[contactIndices[i+contactPrefetchDistance]])
		}

		contactIndex := int(contactIndices[i])
		assert(contactIndex < len(world.contacts), "contactIndex < world->contacts.count")

		contact := &contacts[contactIndex]
		validate(int(contact.contactId) == contactIndex, "contact->contactId == contactIndex")

		shapeIdA := int(contact.shapeIdA)
		shapeIdB := int(contact.shapeIdB)

		// Do proxies still overlap?
		overlap := overlapV(&fatAABBs[shapeIdA], &fatAABBs[shapeIdB])
		if overlap == false && predictiveContactFor(world, contact) == nil {
			// This contact will be destroyed
			contact.flags |= simDisjoint
			contact.flags &^= simTouchingFlag
			setBit(&taskContext.contactStateBitSet, uint32(contactIndex))
			continue
		}

		shapeA := &shapes[shapeIdA]
		shapeB := &shapes[shapeIdB]

		// Update contact respecting shape/body order (A,B)
		encodedA := int(contact.encodedBodySimA)
		encodedB := int(contact.encodedBodySimB)
		bodySimA := resolveContactBodySim(world, awakeSims, staticSims, encodedA, int(contact.edges[0].bodyId))
		bodySimB := resolveContactBodySim(world, awakeSims, staticSims, encodedB, int(contact.edges[1].bodyId))

		wasTouching := contact.flags&simTouchingFlag != 0

		transformA := bodySimA.transform
		transformB := bodySimB.transform

		isFast := ((bodySimA.flags|bodySimB.flags)&isFast) != 0 || predictiveContactFor(world, contact) != nil

		recycleTolerance := recycleDistanceNonTouching
		if wasTouching {
			recycleTolerance = recycleDistance
		}

		// Contact recycling optimization. Please cite this library if you use this optimization.
		// This is inspired by persistent contact manifolds used in some physics engines, such as PhysX.
		// However, this allows larger relative motion and has fewer tuning parameters (just one).
		if isFast == false && recycleDistance > 0.0 && (contact.flags&relativeTransformValid) != 0 &&
			(contact.flags&contactRecycleFlag) != 0 {
			// The scalar part of b3InvMulQuat is just the quaternion dot product.
			// cos(relative_angle/2) = scalar(conj(q1) * q2) = dot(q1, q2)
			// A small relative angle means this value is close to 1. Need to use abs or square
			// due to double cover.
			angleA := DotQuat(transformA.Q, contact.cachedRotationA)
			angleB := DotQuat(transformB.Q, contact.cachedRotationB)
			angularDistance := MinFloat(angleA*angleA, angleB*angleB)

			xf := InvMulWorldTransforms(transformA, transformB)
			xfc := contact.cachedRelativePose
			maxExtentA := bodySimA.maxExtent
			if isStaticSimIndex(encodedA) {
				maxExtentA = Vec3Zero
			}
			maxExtentB := bodySimB.maxExtent
			if isStaticSimIndex(encodedB) {
				maxExtentB = Vec3Zero
			}
			maxExtent := Max(maxExtentA, maxExtentB)

			// Variation of Conservative Advancement
			// distance + 2 * length(modified_cross(|qr.v|, maxExtent)) < recycleTolerance.
			// 2*|qr.v| == 2*|sin(theta/2)| ~= theta for small angles.
			distSquared := DistanceSquared(xf.P, xfc.P)

			if angularDistance > ContactRecycleAngularDistance && distSquared < recycleTolerance*recycleTolerance {
				distance := sqrtf(distSquared)
				slack := recycleTolerance - distance

				// qr = inv( inv(qA0) * qB0 ) * inv(qA) * qB
				//    = inv(qB0) * qA0 * inv(qA) * qB
				// Suppose A is static
				// qr = inv(qB0) * qA0 * inv(qA0) * qB
				//    = inv(qB0) * qB
				// qB = qB0 * qr
				// Therefore qr is associated with the local angular velocity of body B when A is static.
				qr := InvMulQuat(xfc.Q, xf.Q)
				arc := modifiedCross(Abs(qr.V), maxExtent)

				arcSq := 4.0 * LengthSquared(arc)
				if arcSq < slack*slack {
					dqA := MulQuat(transformA.Q, Conjugate(contact.cachedRotationA))
					dqB := MulQuat(transformB.Q, Conjugate(contact.cachedRotationB))
					matrixA := MakeMatrixFromQuat(dqA)
					matrixB := MakeMatrixFromQuat(dqB)

					// Minimize round-off
					dc := SubPos(bodySimB.center, bodySimA.center)

					manifoldCount := int(contact.manifoldCount)
					for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
						manifold := &contact.manifolds[manifoldIndex]
						normal := manifold.Normal

						pointCount := int(manifold.PointCount)
						for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
							// Keep anchors but update separation, same as sub-stepping. This eliminates jitter.
							mp := &manifold.Points[pointIndex]
							rA := MulMV(matrixA, mp.AnchorA)
							rB := MulMV(matrixB, mp.AnchorB)
							dp := Add(dc, Sub(rB, rA))
							mp.Separation = mp.BaseSeparation + Dot(dp, normal)
							mp.NormalVelocity = 0.0
							mp.Persisted = true
						}
					}

					// Diagnostics
					taskContext.recycledContactCount += 1
					bucketIndex := MinInt(manifoldCount, ContactManifoldCountBuckets-1)
					if bucketIndex > 0 {
						taskContext.manifoldCounts[bucketIndex-1] += 1
					}

					// Contact is recycled. This also skips updating other aspects of the contact
					// such as material parameters.
					continue
				}
			}
		}

		// Caching for contact recycling.
		contact.cachedRotationA = transformA.Q
		contact.cachedRotationB = transformB.Q
		contact.cachedRelativePose = InvMulWorldTransforms(transformA, transformB)
		contact.flags |= relativeTransformValid

		// This updates solid contacts
		touching := updateContact(world, workerIndex, contact, shapeA, bodySimA.localCenter, transformA, shapeB,
			bodySimB.localCenter, transformB, isFast, taskContext.arena)

		bucketIndex := MinInt(int(contact.manifoldCount), ContactManifoldCountBuckets-1)
		if bucketIndex > 0 {
			taskContext.manifoldCounts[bucketIndex-1] += 1
		}

		// Update the mesh contact spec
		if touching == true && wasTouching == true && (contact.flags&simMeshContact) != 0 {
			assert(contact.colorIndex != NullIndex, "contact->colorIndex != B3_NULL_INDEX")
			assert(0 <= contact.colorIndex && contact.colorIndex < GraphColorCount,
				"0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT")
			color := &graph.colors[contact.colorIndex]
			spec := arrayGet(color.contacts, int(contact.localIndex))
			spec.manifoldCount = uint16(contact.manifoldCount)
		}

		// State changes that affect island connectivity. Also affects contact events.
		if touching == true && wasTouching == false {
			contact.flags |= simStartedTouching
			setBit(&taskContext.contactStateBitSet, uint32(contactIndex))
		} else if touching == false && wasTouching == true {
			contact.flags |= simStoppedTouching
			setBit(&taskContext.contactStateBitSet, uint32(contactIndex))
		}

		if touching {
			manifoldCount := int(contact.manifoldCount)
			for manifoldIndex := 0; manifoldIndex < manifoldCount; manifoldIndex++ {
				manifold := &contact.manifolds[manifoldIndex]
				pointCount := int(manifold.PointCount)
				for pointIndex := 0; pointIndex < pointCount; pointIndex++ {
					// Cache separation
					mp := &manifold.Points[pointIndex]
					mp.BaseSeparation = mp.Separation
				}
			}
		}
	}
}

// addNonTouchingContact is b3AddNonTouchingContact.
func addNonTouchingContact(world *world, contact *contact) {
	assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")
	set := arrayGet(world.solverSets, awakeSet)
	contact.colorIndex = NullIndex
	contact.localIndex = int32(len(set.contactIndices))
	arrayPush(&set.contactIndices, contact.contactId)
}

// removeNonTouchingContact is b3RemoveNonTouchingContact.
func removeNonTouchingContact(world *world, setIndex int, localIndex int) {
	set := arrayGet(world.solverSets, setIndex)
	movedIndex := arrayRemoveSwap(&set.contactIndices, localIndex)
	if movedIndex != NullIndex {
		movedContactIndex := int(set.contactIndices[localIndex])
		movedContact := arrayGet(world.contacts, movedContactIndex)
		assert(int(movedContact.setIndex) == setIndex, "movedContact->setIndex == setIndex")
		assert(movedContact.colorIndex == NullIndex, "movedContact->colorIndex == B3_NULL_INDEX")
		assert(int(movedContact.localIndex) == movedIndex, "movedContact->localIndex == movedIndex")
		movedContact.localIndex = int32(localIndex)
	}
}

// collide is b3Collide.
// Narrow-phase collision
func collide(context *stepContext) {
	world := context.world

	assert(world.workerCount > 0, "world->workerCount > 0")

	// Gather contacts from all the graph colors into a single array for easier parallel-for
	touchingCount := 0

	graphColors := &world.constraintGraph.colors
	for i := 0; i < GraphColorCount; i++ {
		touchingCount += len(graphColors[i].convexContacts) + len(graphColors[i].contacts)
	}

	// PORT: the local b3SolverSet* awakeSet is awakeSolverSet (awakeSet is the b3_awakeSet constant).
	awakeSolverSet := arrayGet(world.solverSets, awakeSet)
	nonTouchingCount := len(awakeSolverSet.contactIndices)

	contactCount := touchingCount + nonTouchingCount

	if contactCount == 0 {
		return
	}

	contactIndices := stackAlloc[int32](&world.stack, contactCount, "contact indices")

	contactIndex := 0
	for i := 0; i < GraphColorCount; i++ {
		color := &graphColors[i]
		count := len(color.convexContacts)
		for j := 0; j < count; j++ {
			contactIndices[contactIndex] = color.convexContacts[j]
			contactIndex += 1
		}

		count = len(color.contacts)
		for j := 0; j < count; j++ {
			contactIndices[contactIndex] = color.contacts[j].contactId
			contactIndex += 1
		}
	}

	assert(contactIndex == touchingCount, "contactIndex == touchingCount")

	if nonTouchingCount > 0 {
		nonTouchingIndices := awakeSolverSet.contactIndices
		copy(contactIndices[touchingCount:], nonTouchingIndices[:nonTouchingCount])
	}

	context.awakeContactIndices = contactIndices

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	contactIdCapacity := getIdCapacity(&world.contactIdPool)
	for i := 0; i < int(world.workerCount); i++ {
		taskContext := &world.taskContexts[i]
		setBitCountAndClear(&taskContext.contactStateBitSet, uint32(contactIdCapacity))
		taskContext.satCallCount = 0
		taskContext.satCacheHitCount = 0
		taskContext.recycledContactCount = 0
		taskContext.manifoldCounts = [ContactManifoldCountBuckets]int32{}
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	minRange := 20
	parallelFor(world, collideTask, contactCount, minRange, context, "collide")

	stackFree(&world.stack, contactIndices)
	context.awakeContactIndices = nil
	contactIndices = nil

	// Serially update contact state
	// todo bring this zone together with island merge

	satMultiplier := int32(0)
	if context.dt > 0.0 {
		satMultiplier = 1
	}

	// Bitwise OR all contact bits
	bitSet := &world.taskContexts[0].contactStateBitSet
	world.satCallCount = satMultiplier * world.taskContexts[0].satCallCount
	world.satCacheHitCount = satMultiplier * world.taskContexts[0].satCacheHitCount
	world.manifoldCounts = world.taskContexts[0].manifoldCounts

	for i := 1; i < int(world.workerCount); i++ {
		inPlaceUnion(bitSet, &world.taskContexts[i].contactStateBitSet)
		world.satCallCount += world.taskContexts[i].satCallCount
		world.satCacheHitCount += world.taskContexts[i].satCacheHitCount
		for j := 0; j < ContactManifoldCountBuckets; j++ {
			world.manifoldCounts[j] += world.taskContexts[i].manifoldCounts[j]
		}
	}

	// Release per-step overflow blocks and grow the backing capacity if last
	// step's demand exceeded it. Contact processing is the only consumer of
	// these arenas and is finished by this point.
	for i := 0; i < int(world.workerCount); i++ {
		arenaSync(&world.taskContexts[i].arena)
	}

	endEventArrayIndex := world.endEventArrayIndex

	shapes := world.shapes
	worldId := world.worldId

	// Process contact state changes. Iterate over set bits
	for k := uint32(0); k < bitSet.blockCount; k++ {
		bits := bitSet.bits[k]
		for bits != 0 {
			ctz := ctz64(bits)
			contactId := int(64*k + ctz)

			contact := arrayGet(world.contacts, contactId)
			assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")

			shapeA := &shapes[contact.shapeIdA]
			shapeB := &shapes[contact.shapeIdB]
			shapeIdA := ShapeID{shapeA.id + 1, worldId, shapeA.generation}
			shapeIdB := ShapeID{shapeB.id + 1, worldId, shapeB.generation}
			contactFullId := ContactID{
				Index1:     int32(contactId + 1),
				World0:     worldId,
				Padding:    0,
				Generation: contact.generation,
			}
			flags := contact.flags

			if flags&simDisjoint != 0 {
				// Bounding boxes no longer overlap
				destroyContact(world, contact, false)
				contact = nil
			} else if flags&simStartedTouching != 0 {
				assert(contact.islandId == NullIndex, "contact->islandId == B3_NULL_INDEX")

				if flags&contactEnableContactEvents != 0 {
					event := ContactBeginTouchEvent{shapeIdA, shapeIdB, contactFullId}
					arrayPush(&world.contactBeginEvents, event)
				}

				assert(contact.manifoldCount > 0, "contact->manifoldCount > 0")
				assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")

				// Link first because this wakes colliding bodies and ensures the body sims
				// are in the correct place.
				contact.flags &^= simStartedTouching
				contact.flags |= contactTouchingFlag
				linkContact(world, contact)

				// Make sure these didn't change
				assert(contact.colorIndex == NullIndex, "contact->colorIndex == B3_NULL_INDEX")

				// Contact sim pointer may have become orphaned due to awake set growth,
				// so I just need to refresh it.

				oldLocalIndex := int(contact.localIndex)

				addContactToGraph(world, contact)
				removeNonTouchingContact(world, awakeSet, oldLocalIndex)
			} else if flags&simStoppedTouching != 0 {
				contact.flags &^= simStoppedTouching
				contact.flags &^= contactTouchingFlag

				if contact.flags&contactEnableContactEvents != 0 {
					event := ContactEndTouchEvent{shapeIdA, shapeIdB, contactFullId}
					arrayPush(&world.contactEndEvents[endEventArrayIndex], event)
				}

				assert(contact.manifoldCount == 0, "contact->manifoldCount == 0")

				// Cache these here for the remove below
				colorIndex := int(contact.colorIndex)
				localIndex := int(contact.localIndex)

				unlinkContact(world, contact)
				bodyIdA := int(contact.edges[0].bodyId)
				bodyIdB := int(contact.edges[1].bodyId)

				addNonTouchingContact(world, contact)

				isMeshContact := contact.flags&simMeshContact != 0
				removeContactFromGraph(world, bodyIdA, bodyIdB, colorIndex, localIndex, isMeshContact)
				contact = nil
			}

			// Clear the smallest set bit
			bits = bits & (bits - 1)
		}
	}

	validateSolverSets(world)
	validateContacts(world)
}

// Step is b3World_Step: simulate a world for one time step. This performs collision detection,
// integration, and constraint solution.
// timeStep: the amount of time to simulate, this should be a fixed number. Usually 1/60.
// subStepCount: the number of sub-steps, increasing the sub-step count can increase accuracy.
// Usually 4.
func (worldId WorldID) Step(timeStep float32, subStepCount int) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsStep{worldId, timeStep, int32(subStepCount)}
		recWriteStep(world.recording, &recArgs)
	}

	world.locked = true

	// Clear debug buffers
	for i := 0; i < int(world.workerCount); i++ {
		world.taskContexts[i].pointCount = 0
		world.taskContexts[i].lineCount = 0
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	arrayClear(&world.bodyMoveEvents)
	arrayClear(&world.sensorBeginEvents)
	arrayClear(&world.contactBeginEvents)
	arrayClear(&world.contactHitEvents)
	arrayClear(&world.jointEvents)

	world.profile = Profile{}

	world.activeTaskCount = 0
	world.taskCount = 0

	if world.scheduler != nil {
		resetScheduler(world.scheduler)
	}

	stepTicks := GetTicks()

	{
		c := &world.maxCapacity
		c.StaticShapeCount = int32(MaxInt(int(c.StaticShapeCount), int(world.broadPhase.trees[StaticBody].ProxyCount)))
		c.DynamicShapeCount = int32(MaxInt(int(c.DynamicShapeCount), int(world.broadPhase.trees[DynamicBody].ProxyCount)))

		staticBodyCount := len(world.solverSets[staticSet].bodySims)
		c.StaticBodyCount = int32(MaxInt(int(c.StaticBodyCount), staticBodyCount))

		// this includes kinematic bodies
		totalBodyCount := getIdCount(&world.bodyIdPool)
		c.DynamicBodyCount = int32(MaxInt(int(c.DynamicBodyCount), totalBodyCount-staticBodyCount))

		totalContactCount := getIdCount(&world.contactIdPool)
		c.ContactCount = int32(MaxInt(int(c.ContactCount), totalContactCount))
	}

	// Update collision pairs and create contacts
	{
		pairTicks := GetTicks()
		updateBroadPhasePairs(world)
		prepareSpeculativeCCD(world, timeStep, subStepCount)
		world.profile.Pairs = GetMilliseconds(pairTicks)
	}

	// PORT: the local b3SolverSet* awakeSet is awakeSolverSet (awakeSet is the b3_awakeSet constant).
	awakeSolverSet := arrayGet(world.solverSets, awakeSet)

	// PORT: b3StepContext context = { 0 } on the C stack; the parallel-for tasks receive a pointer
	// to it, so it lives in the world (stepScratch) and is zeroed here.
	context := &world.stepScratch.stepContext
	*context = stepContext{}
	context.world = world
	context.states = awakeSolverSet.bodyStates
	context.dt = timeStep
	context.subStepCount = int32(MaxInt(1, subStepCount))

	if timeStep > 0.0 {
		context.inv_dt = 1.0 / timeStep
		context.h = timeStep / float32(context.subStepCount)
		context.inv_h = float32(context.subStepCount) * context.inv_dt
	} else {
		context.inv_dt = 0.0
		context.h = 0.0
		context.inv_h = 0.0
	}

	world.inv_h = context.inv_h
	world.inv_dt = context.inv_dt

	// Hertz values get reduced for large time steps
	contactHertz := MinFloat(world.contactHertz, float32(0.125*context.inv_h))
	context.contactSoftness = makeSoft(contactHertz, world.contactDampingRatio, context.h)
	context.staticSoftness = makeSoft(float32(2.0*contactHertz), float32(0.5*world.contactDampingRatio), context.h)

	context.maxLinearVelocity = world.maxLinearSpeed
	context.enableWarmStarting = world.enableWarmStarting

	// Narrow phase : update contacts
	{
		collideTicks := GetTicks()
		collide(context)
		world.profile.Collide = GetMilliseconds(collideTicks)
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if timeStep > 0.0 {
		solveTicks := GetTicks()
		solve(world, context)
		finishSpeculativeCCD(world)
		world.profile.Solve = GetMilliseconds(solveTicks)
	}

	// Finish the tree task in case b3Solve didn't finish it
	if world.userTreeTask != nil {
		world.finishTaskFcn(world.userTreeTask, world.userTaskContext)
		world.userTreeTask = nil
		world.activeTaskCount -= 1
		validateNoMoved(&world.broadPhase)
	}

	// Update sensors
	{
		sensorTicks := GetTicks()
		overlapSensors(world)
		world.profile.Sensors = GetMilliseconds(sensorTicks)
	}

	world.profile.Step = GetMilliseconds(stepTicks)

	assert(world.stack.allocation == 0, "world->stack.allocation == 0")

	// Ensure stack is large enough
	growStack(&world.stack)

	// Make sure all tasks that were started were also finished
	assert(world.activeTaskCount == 0, "world->activeTaskCount == 0")

	// Swap end event array buffers
	world.endEventArrayIndex = 1 - world.endEventArrayIndex
	arrayClear(&world.sensorEndEvents[world.endEventArrayIndex])
	arrayClear(&world.contactEndEvents[world.endEventArrayIndex])
	world.locked = false

	if world.recording != nil {
		hash := hashWorldState(world)
		stateHash := recArgsStateHash{worldId, hash}
		recWriteStateHash(world.recording, &stateHash)

		// Fold this step's world bounds into the recording so a viewer can frame the whole motion.
		worldBounds := AABB{}
		haveBounds := false
		for i := 0; i < int(BodyTypeCount); i++ {
			tree := &world.broadPhase.trees[i]
			if tree.GetProxyCount() == 0 {
				continue
			}
			bounds := tree.GetRootBounds()
			if haveBounds {
				worldBounds = worldBounds.Union(bounds)
			} else {
				worldBounds = bounds
			}
			haveBounds = true
		}
		if haveBounds {
			recAccumulateBounds(world.recording, worldBounds)
		}
	}
}

// drawContext is DrawContext.
type drawContext struct {
	world *world
	draw  *DebugDraw
}

// drawQueryCallback is DrawQueryCallback.
func drawQueryCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(userData)

	drawContext := context.(*drawContext)
	world := drawContext.world
	draw := drawContext.draw

	shape := arrayGet(world.shapes, shapeId)
	assert(int(shape.id) == shapeId, "shape->id == shapeId")

	setBit(&world.debugBodySet, uint32(shape.bodyId))

	if draw.DrawShapes {
		body := arrayGet(world.bodies, int(shape.bodyId))
		bodySim := getBodySim(world, body)

		var color HexColor

		surfaceMaterial := getShapeMaterials(shape)
		if surfaceMaterial[0].CustomColor != 0 {
			// May already carry a packed material preset, pass through unchanged
			color = HexColor(surfaceMaterial[0].CustomColor)
		} else {
			// Hue carries the state, material carries its energy. Calm matte for the
			// resting masses, glossy for fast bodies, metallic for the driven kinematic.
			// Diagnostic states keep a saturated hue and the default material so they pop.
			var rgb HexColor
			material := DebugMaterialDefault

			if body.typ == DynamicBody && body.mass == 0.0 {
				// Bad body
				rgb = ColorRed
			} else if body.setIndex == disabledSet {
				rgb = ColorSlateGray
			} else if shape.sensorIndex != NullIndex {
				rgb = ColorWheat
			} else if bodySim.flags&hadTimeOfImpact != 0 {
				rgb = ColorLime
			} else if (bodySim.flags&isBullet) != 0 && body.setIndex == awakeSet {
				rgb = ColorTurquoise
			} else if body.flags&isSpeedCapped != 0 {
				rgb = ColorYellow
			} else if bodySim.flags&isFast != 0 {
				rgb = ColorOrange
				material = DebugMaterialGlossy
			} else if body.typ == StaticBody {
				rgb = ColorDarkGray
				material = DebugMaterialMatte
			} else if body.typ == KinematicBody {
				if body.setIndex == awakeSet {
					rgb = ColorSteelBlue
					material = DebugMaterialMetallic
				} else {
					rgb = ColorLightSteelBlue
					material = DebugMaterialMatte
				}
			} else if body.setIndex == awakeSet {
				rgb = ColorTan
				material = DebugMaterialSoft
			} else {
				rgb = ColorLightSlateGray
				material = DebugMaterialDead
			}

			color = HexColor(MakeDebugColor(rgb, material))
		}

		if shape.userShape == nil && world.createDebugShape != nil {
			debugShape := DebugShape{}
			debugShape.ShapeId = ShapeID{
				World0:     world.worldId,
				Index1:     int32(shapeId + 1),
				Generation: shape.generation,
			}
			debugShape.Type = shape.typ

			switch shape.typ {
			case CapsuleShape:
				debugShape.Capsule = &shape.capsule
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case CompoundShape:
				debugShape.Compound = shape.compound
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case HeightShape:
				debugShape.HeightField = shape.heightField
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case HullShape:
				debugShape.Hull = shape.hull
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case MeshShape:
				debugShape.Mesh = &shape.mesh
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			case SphereShape:
				debugShape.Sphere = &shape.sphere
				shape.userShape = world.createDebugShape(&debugShape, world.userDebugShapeContext)
			default:
				assert(false, "false")
			}
		}

		if shape.userShape != nil {
			draw.DrawShapeFcn(shape.userShape, bodySim.transform, color, draw.Context)
		}
	}

	if draw.DrawBounds {
		draw.DrawBoundsFcn(world.fatAABBs[shapeId], ColorGold, draw.Context)
	}

	return true
}

// Draw is b3World_Draw: call this to draw shapes and other debug draw data.
// PORT: the snprintf buffers become fmt.Sprintf strings (no 32 byte truncation).
func (worldId WorldID) Draw(draw *DebugDraw, maskBits uint64) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	assert(IsValidAABB(draw.DrawingBounds), "b3IsValidAABB( draw->drawingBounds )")

	lengthScale := GetLengthUnitsPerMeter()
	axisScale := float32(0.3 * lengthScale)
	farColor := ColorDarkBlue
	speculativeColor := ColorBlue
	addColor := ColorLimeGreen
	persistColor := ColorLightBlue
	// b3HexColor normalColors[3] = { b3_colorLightGray, b3_colorLightSalmon, b3_colorLightSeaGreen };
	impulseColor := ColorMagenta
	frictionColor := ColorYellow

	graphColors := [GraphColorCount]HexColor{
		ColorRed, ColorOrange, ColorYellow, ColorGreen, ColorCyan, ColorBlue,
		ColorViolet, ColorPink, ColorChocolate, ColorGoldenRod, ColorCoral, ColorRosyBrown,
		ColorAqua, ColorPeru, ColorLime, ColorGold, ColorPlum, ColorSnow,
		ColorTeal, ColorKhaki, ColorSalmon, ColorPeachPuff, ColorHoneyDew, ColorBlack,
	}

	bodyCapacity := getIdCapacity(&world.bodyIdPool)
	setBitCountAndClear(&world.debugBodySet, uint32(bodyCapacity))

	jointCapacity := getIdCapacity(&world.jointIdPool)
	setBitCountAndClear(&world.debugJointSet, uint32(jointCapacity))

	contactCapacity := getIdCapacity(&world.contactIdPool)
	setBitCountAndClear(&world.debugContactSet, uint32(contactCapacity))

	islandCapacity := getIdCapacity(&world.islandIdPool)
	setBitCountAndClear(&world.debugIslandSet, uint32(islandCapacity))

	drawContext := drawContext{world, draw}

	for i := 0; i < int(BodyTypeCount); i++ {
		world.broadPhase.trees[i].Query(draw.DrawingBounds, maskBits, false, drawQueryCallback, &drawContext)
	}

	wordCount := world.debugBodySet.blockCount
	bits := world.debugBodySet.bits
	for wordIndex := uint32(0); wordIndex < wordCount; wordIndex++ {
		word := bits[wordIndex]
		for word != 0 {
			ctz := ctz64(word)
			bodyId := int(64*wordIndex + ctz)

			body := arrayGet(world.bodies, bodyId)
			bodySim := getBodySim(world, body)

			if draw.DrawBodyNames && body.nameId != nullName {
				offset := Vec3{0.05, 0.05, 0.05}
				transform := WorldTransform{bodySim.center, bodySim.transform.Q}
				p := TransformWorldPoint(transform, offset)
				name := findName(&world.names, body.nameId)
				if name != "" {
					draw.DrawStringFcn(p, name, ColorWhite, draw.Context)
				}
			}

			if draw.DrawMass {
				transform := WorldTransform{bodySim.center, bodySim.transform.Q}
				draw.DrawTransformFcn(transform, draw.Context)

				if body.typ == DynamicBody {
					offset := Vec3{0.05, 0.05, 0.05}
					p := TransformWorldPoint(transform, offset)
					buffer := fmt.Sprintf("%.2f", body.mass)
					draw.DrawStringFcn(p, buffer, ColorWhite, draw.Context)
				}
			}

			if draw.DrawSleep {
				bodyState := getBodyState(world, body)

				if bodyState != nil {
					colors := [4]HexColor{ColorBlue, ColorSkyBlue, ColorOrange, ColorRed}

					color := ColorBlack
					if body.sleepThreshold > 0.0 {
						ratio := body.sleepVelocity / body.sleepThreshold
						index := ClampInt(int(ratio), 0, 3)
						color = colors[index]
					}

					center := bodySim.center
					draw.DrawPointFcn(center, 10.0, color, draw.Context)

					offset := Vec3{0.1, 0.1, 0.1}
					p := OffsetPos(center, offset)

					buffer := fmt.Sprintf("  %.3f", body.sleepVelocity)
					draw.DrawStringFcn(p, buffer, color, draw.Context)
				}
			}

			if draw.DrawJoints {
				jointKey := int(body.headJointKey)
				for jointKey != NullIndex {
					jointId := jointKey >> 1
					edgeIndex := jointKey & 1
					joint := arrayGet(world.joints, jointId)

					// avoid double draw
					if getBit(&world.debugJointSet, uint32(jointId)) == false {
						drawJoint(draw, world, joint)
						setBit(&world.debugJointSet, uint32(jointId))
					}

					jointKey = int(joint.edges[edgeIndex].nextKey)
				}
			}

			speculativeDistance := SpeculativeDistance()
			if draw.DrawContacts && body.typ == DynamicBody && body.setIndex == awakeSet {
				contactKey := int(body.headContactKey)
				for contactKey != NullIndex {
					contactId := contactKey >> 1
					edgeIndex := contactKey & 1
					contact := arrayGet(world.contacts, contactId)
					contactKey = int(contact.edges[edgeIndex].nextKey)

					if contact.setIndex != awakeSet || contact.colorIndex == NullIndex {
						continue
					}

					// avoid double draw
					if getBit(&world.debugContactSet, uint32(contactId)) == false {
						bodyA := arrayGet(world.bodies, int(contact.edges[0].bodyId))
						bodySimA := getBodySim(world, bodyA)
						bodyB := arrayGet(world.bodies, int(contact.edges[1].bodyId))
						bodySimB := getBodySim(world, bodyB)

						for manifoldIndex := 0; manifoldIndex < int(contact.manifoldCount); manifoldIndex++ {
							manifold := &contact.manifolds[manifoldIndex]
							assert(manifold.PointCount > 0, "manifold->pointCount > 0")

							normal := manifold.Normal

							// Average the anchors not the world points so the friction center stays exact far from the origin
							contactCenter := bodySimB.center
							if draw.DrawAnchorA {
								contactCenter = bodySimA.center
							}
							frictionAnchor := Vec3Zero
							totalWeight := float32(0.0)
							invTau := 1.0 / SpeculativeDistance()

							points := manifold.Points[:]
							for pointIndex := 0; pointIndex < int(manifold.PointCount); pointIndex++ {
								mp := &points[pointIndex]

								var buffer string

								anchor := mp.AnchorB
								if draw.DrawAnchorA {
									anchor = mp.AnchorA
								}
								p := OffsetPos(contactCenter, anchor)

								// See similar friction anchor weights in b3PrepareContacts_Mesh.
								weight := ClampFloat(mulSub(2.0, mp.Separation, invTau), MinFrictionWeight, 1.0)
								frictionAnchor = MulAdd(frictionAnchor, weight, anchor)
								totalWeight += weight

								if draw.DrawContactNormals {
									p1 := p
									p2 := OffsetPos(p1, MulSV(axisScale, normal))
									draw.DrawSegmentFcn(p1, p2, ColorLightGray, draw.Context)

									buffer = fmt.Sprintf("   %.2f", 100.0*mp.Separation)
									draw.DrawStringFcn(p, buffer, ColorWhite, draw.Context)
								} else if draw.DrawContactForces {
									// Hack inv_dt for single step debugging
									inv_dt := float32(60.0)
									if world.inv_dt > 0.0 {
										inv_dt = world.inv_dt
									}
									force := float32(mp.TotalNormalImpulse * inv_dt)
									p1 := p
									p2 := OffsetPos(p1, MulSV(float32(draw.ForceScale*force), normal))
									draw.DrawSegmentFcn(p1, p2, impulseColor, draw.Context)
									buffer = fmt.Sprintf("  %.1f", force)
									draw.DrawStringFcn(p1, buffer, ColorWhite, draw.Context)
								} else if draw.DrawContactFeatures {
									// PORT: C's %#X prints a zero without the 0X prefix; Go always adds it.
									buffer = fmt.Sprintf("   %#X", mp.FeatureId)
									draw.DrawStringFcn(p, buffer, ColorWhite, draw.Context)
								}

								if draw.DrawGraphColors {
									// graph color
									pointSize := float32(10.0)
									if contact.colorIndex == overflowIndex {
										pointSize = 15.0
									}
									draw.DrawPointFcn(p, pointSize, graphColors[contact.colorIndex], draw.Context)
									// g_draw.DrawString(point->position, "%d", point->color);
								} else if mp.Persisted == false {
									// Add
									draw.DrawPointFcn(p, 20.0, addColor, draw.Context)
								} else if mp.Separation > speculativeDistance {
									// Speculative
									draw.DrawPointFcn(p, 10.0, farColor, draw.Context)
								} else if mp.Separation > 0.0 {
									// Speculative
									draw.DrawPointFcn(p, 10.0, speculativeColor, draw.Context)
								} else {
									// Persist
									draw.DrawPointFcn(p, 10.0, persistColor, draw.Context)
								}
							}

							if draw.DrawContactForces {
								// Hack inv_dt for single step debugging
								inv_dt := float32(60.0)
								if world.inv_dt > 0.0 {
									inv_dt = world.inv_dt
								}
								frictionAnchor = MulSV(1.0/totalWeight, frictionAnchor)
								p1 := OffsetPos(contactCenter, frictionAnchor)
								frictionForce := MulSV(float32(0.5*inv_dt), manifold.FrictionImpulse)
								p2 := OffsetPos(p1, MulSV(draw.ForceScale, frictionForce))
								draw.DrawSegmentFcn(p1, p2, frictionColor, draw.Context)
								draw.DrawPointFcn(p1, 5.0, frictionColor, draw.Context)

								p1 = OffsetPos(p1, MulSV(float32(0.05*lengthScale), normal))
								buffer := fmt.Sprintf("%.2f", Length(frictionForce))
								draw.DrawStringFcn(p1, buffer, ColorWhite, draw.Context)
							}
						}

						setBit(&world.debugContactSet, uint32(contactId))
					}

					// PORT: upstream assigns contactKey a second time here (same value as above). Kept.
					contactKey = int(contact.edges[edgeIndex].nextKey)
				}
			}

			if draw.DrawIslands {
				islandId := int(body.islandId)
				if islandId != NullIndex && getBit(&world.debugIslandSet, uint32(islandId)) == false {
					island := &world.islands[islandId]
					if island.setIndex == NullIndex {
						// PORT: as upstream, this continue skips clearing the bit below (see LOOSE_ENDS.md).
						continue
					}

					shapeCount := 0
					aabb := AABB{
						LowerBound: Vec3{fltMax, fltMax, fltMax},
						UpperBound: Vec3{-fltMax, -fltMax, -fltMax},
					}

					for b := 0; b < len(island.bodies); b++ {
						islandBodyId := int(island.bodies[b])
						islandBody := arrayGet(world.bodies, islandBodyId)
						shapeId := int(islandBody.headShapeId)
						for shapeId != NullIndex {
							shape := arrayGet(world.shapes, shapeId)
							aabb = aabb.Union(world.fatAABBs[shapeId])
							shapeCount += 1
							shapeId = int(shape.nextShapeId)
						}
					}

					if shapeCount > 0 {
						draw.DrawBoundsFcn(aabb, ColorOrangeRed, draw.Context)
					}

					setBit(&world.debugIslandSet, uint32(islandId))
				}
			}

			// Clear the smallest set bit
			word = word & (word - 1)
		}
	}

	lengthUnits := GetLengthUnitsPerMeter()
	offset := Vec3{float32(0.002 * lengthUnits), float32(0.002 * lengthUnits), float32(0.002 * lengthUnits)}
	for i := 0; i < int(world.workerCount); i++ {
		pointCount := int(world.taskContexts[i].pointCount)
		points := world.taskContexts[i].points[:]
		for j := 0; j < pointCount; j++ {
			point := &points[j]
			p := point.p
			draw.DrawPointFcn(p, 5.0, point.color, draw.Context)
			buffer := fmt.Sprintf("   %d, %.2f", point.label, point.value)
			ps := OffsetPos(p, offset)
			draw.DrawStringFcn(ps, buffer, point.color, draw.Context)
		}

		lineCount := int(world.taskContexts[i].lineCount)
		lines := world.taskContexts[i].lines[:]
		for j := 0; j < lineCount; j++ {
			line := &lines[j]
			p1 := line.p1
			p2 := line.p2
			draw.DrawSegmentFcn(p1, p2, line.color, draw.Context)
			draw.DrawPointFcn(p1, 10.0, line.color, draw.Context)
			buffer := fmt.Sprintf("%d", line.label)
			ps := OffsetPos(p1, offset)
			draw.DrawStringFcn(ps, buffer, line.color, draw.Context)
		}
	}
}

// GetBounds is b3World_GetBounds: get the bounding box of all shapes in the world.
func (worldId WorldID) GetBounds() AABB {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return AABB{}
	}

	worldBounds := AABB{}
	haveBounds := false

	for i := 0; i < int(BodyTypeCount); i++ {
		tree := &world.broadPhase.trees[i]
		if tree.GetProxyCount() == 0 {
			continue
		}

		bounds := world.broadPhase.trees[i].GetRootBounds()

		if haveBounds {
			worldBounds = worldBounds.Union(bounds)
		} else {
			worldBounds = bounds
			haveBounds = true
		}
	}

	return worldBounds
}

// GetBodyEvents is b3World_GetBodyEvents: get the body events for the current time step. The event
// data is transient. Do not store a reference to this data.
func (worldId WorldID) GetBodyEvents() BodyEvents {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return BodyEvents{}
	}

	count := len(world.bodyMoveEvents)
	events := BodyEvents{world.bodyMoveEvents, int32(count)}
	return events
}

// GetSensorEvents is b3World_GetSensorEvents: get sensor events for the current time step. The
// event data is transient. Do not store a reference to this data.
func (worldId WorldID) GetSensorEvents() SensorEvents {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return SensorEvents{}
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - world.endEventArrayIndex

	beginCount := len(world.sensorBeginEvents)
	endCount := len(world.sensorEndEvents[endEventArrayIndex])

	events := SensorEvents{
		BeginEvents: world.sensorBeginEvents,
		EndEvents:   world.sensorEndEvents[endEventArrayIndex],
		BeginCount:  int32(beginCount),
		EndCount:    int32(endCount),
	}
	return events
}

// GetContactEvents is b3World_GetContactEvents: get contact events for this current time step. The
// event data is transient. Do not store a reference to this data.
func (worldId WorldID) GetContactEvents() ContactEvents {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return ContactEvents{}
	}

	// Careful to use previous buffer
	endEventArrayIndex := 1 - world.endEventArrayIndex

	beginCount := len(world.contactBeginEvents)
	endCount := len(world.contactEndEvents[endEventArrayIndex])
	hitCount := len(world.contactHitEvents)

	events := ContactEvents{
		BeginEvents: world.contactBeginEvents,
		EndEvents:   world.contactEndEvents[endEventArrayIndex],
		HitEvents:   world.contactHitEvents,
		BeginCount:  int32(beginCount),
		EndCount:    int32(endCount),
		HitCount:    int32(hitCount),
	}

	return events
}

// GetJointEvents is b3World_GetJointEvents: get the joint events for the current time step. The
// event data is transient. Do not store a reference to this data.
func (worldId WorldID) GetJointEvents() JointEvents {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return JointEvents{}
	}

	count := len(world.jointEvents)
	events := JointEvents{world.jointEvents, int32(count)}
	return events
}

// IsValid is b3World_IsValid: world id validation. Provides validation for up to 64K allocations.
func (id WorldID) IsValid() bool {
	if id.Index1 < 1 || MaxWorlds < id.Index1 {
		return false
	}

	world := &worlds[id.Index1-1]

	if world.worldId != id.Index1-1 {
		// world is not allocated
		return false
	}

	return id.Generation == world.generation
}

// IsValid is b3Body_IsValid: body identifier validation. A valid body exists in a world and is
// non-null. This can be used to detect orphaned ids. Provides validation for up to 64K allocations.
func (id BodyID) IsValid() bool {
	if MaxWorlds <= id.World0 {
		// invalid world
		return false
	}

	world := &worlds[id.World0]
	if world.worldId != id.World0 {
		// world is free
		return false
	}

	if id.Index1 < 1 || int32(len(world.bodies)) < id.Index1 {
		// invalid index
		return false
	}

	body := &world.bodies[id.Index1-1]
	if body.setIndex == NullIndex {
		// this was freed
		return false
	}

	assert(body.localIndex != NullIndex, "body->localIndex != B3_NULL_INDEX")

	if body.generation != id.Generation {
		// this id is orphaned
		return false
	}

	return true
}

// IsValid is b3Shape_IsValid: shape identifier validation. Provides validation for up to 64K
// allocations.
func (id ShapeID) IsValid() bool {
	if MaxWorlds <= id.World0 {
		return false
	}

	world := &worlds[id.World0]
	if world.worldId != id.World0 {
		// world is free
		return false
	}

	shapeId := int(id.Index1 - 1)
	if shapeId < 0 || len(world.shapes) <= shapeId {
		return false
	}

	shape := &world.shapes[shapeId]
	if shape.id == NullIndex {
		// shape is free
		return false
	}

	assert(int(shape.id) == shapeId, "shape->id == shapeId")

	return id.Generation == shape.generation
}

// IsValid is b3Joint_IsValid: joint identifier validation. Provides validation for up to 64K
// allocations.
func (id JointID) IsValid() bool {
	if MaxWorlds <= id.World0 {
		return false
	}

	world := &worlds[id.World0]
	if world.worldId != id.World0 {
		// world is free
		return false
	}

	jointId := int(id.Index1 - 1)
	if jointId < 0 || len(world.joints) <= jointId {
		return false
	}

	joint := &world.joints[jointId]
	if joint.jointId == NullIndex {
		// joint is free
		return false
	}

	assert(int(joint.jointId) == jointId, "joint->jointId == jointId")

	return id.Generation == joint.generation
}

// IsValid is b3Contact_IsValid: contact identifier validation. Provides validation for up to 2^32
// allocations.
func (id ContactID) IsValid() bool {
	if MaxWorlds <= id.World0 {
		return false
	}

	world := &worlds[id.World0]
	if world.worldId != id.World0 {
		// world is free
		return false
	}

	contactId := int(id.Index1 - 1)
	if contactId < 0 || len(world.contacts) <= contactId {
		return false
	}

	contact := &world.contacts[contactId]
	if contact.contactId == NullIndex {
		// contact is free
		return false
	}

	assert(int(contact.contactId) == contactId, "contact->contactId == contactId")

	return id.Generation == contact.generation
}

// EnableSleeping is b3World_EnableSleeping: enable/disable sleep. If your application does not
// need sleeping, you can gain some performance by disabling sleep completely at the world level.
// See WorldDef.
func (worldId WorldID) EnableSleeping(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if flag == world.enableSleep {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldEnableSleeping{worldId, flag}
		recWriteWorldEnableSleeping(world.recording, &recArgs)
	}

	world.enableSleep = flag

	if flag == false {
		setCount := len(world.solverSets)
		for i := firstSleepingSet; i < setCount; i++ {
			set := arrayGet(world.solverSets, i)
			if len(set.bodySims) > 0 {
				wakeSolverSet(world, i)
			}
		}
	}
}

// IsSleepingEnabled is b3World_IsSleepingEnabled: is body sleeping enabled?
func (worldId WorldID) IsSleepingEnabled() bool {
	world := getWorldFromId(worldId)
	return world.enableSleep
}

// EnableWarmStarting is b3World_EnableWarmStarting: enable/disable constraint warm starting.
// Advanced feature for testing. Disabling warm starting greatly reduces stability and provides no
// performance gain.
func (worldId WorldID) EnableWarmStarting(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldEnableWarmStarting{worldId, flag}
		recWriteWorldEnableWarmStarting(world.recording, &recArgs)
	}

	world.enableWarmStarting = flag
}

// IsWarmStartingEnabled is b3World_IsWarmStartingEnabled: is constraint warm starting enabled?
func (worldId WorldID) IsWarmStartingEnabled() bool {
	world := getWorldFromId(worldId)
	return world.enableWarmStarting
}

// EnableSSE2Fallback is b3World_EnableSSE2Fallback.
func (worldId WorldID) EnableSSE2Fallback(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if flag {
		world.simdWidth = 4
	} else {
		world.simdWidth = int32(GetSIMDWidth())
	}
}

// GetAwakeBodyCount is b3World_GetAwakeBodyCount: get the number of awake bodies.
func (worldId WorldID) GetAwakeBodyCount() int {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return 0
	}
	awakeSolverSet := arrayGet(world.solverSets, awakeSet)
	return len(awakeSolverSet.bodySims)
}

// EnableContinuous is b3World_EnableContinuous: enable/disable continuous collision between dynamic
// and static bodies. Generally you should keep continuous collision enabled to prevent fast moving
// objects from going through static objects. The performance gain from disabling continuous
// collision is minor.
func (worldId WorldID) EnableContinuous(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldEnableContinuous{worldId, flag}
		recWriteWorldEnableContinuous(world.recording, &recArgs)
	}

	world.enableContinuous = flag
}

// IsContinuousEnabled is b3World_IsContinuousEnabled: is continuous collision enabled?
func (worldId WorldID) IsContinuousEnabled() bool {
	world := getWorldFromId(worldId)
	return world.enableContinuous
}

// SetRestitutionThreshold is b3World_SetRestitutionThreshold: adjust the restitution threshold. It
// is recommended not to make this value very small because it will prevent bodies from sleeping.
// Usually in meters per second.
func (worldId WorldID) SetRestitutionThreshold(value float32) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetRestitutionThreshold{worldId, value}
		recWriteWorldSetRestitutionThreshold(world.recording, &recArgs)
	}

	world.restitutionThreshold = ClampFloat(value, 0.0, fltMax)
}

// GetRestitutionThreshold is b3World_GetRestitutionThreshold: get the restitution speed threshold.
// Usually in meters per second.
func (worldId WorldID) GetRestitutionThreshold() float32 {
	world := getWorldFromId(worldId)
	return world.restitutionThreshold
}

// SetRestitutionIterations is b3World_SetRestitutionIterations: set the number of restitution
// iterations. See MaxRestitutionIterations.
func (worldId WorldID) SetRestitutionIterations(iterations int) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	iterations = ClampInt(iterations, 0, MaxRestitutionIterations)
	if iterations == int(world.restitutionIterations) {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetRestitutionIterations{worldId, int32(iterations)}
		recWriteWorldSetRestitutionIterations(world.recording, &recArgs)
	}

	world.restitutionIterations = int32(iterations)
}

// GetRestitutionIterations is b3World_GetRestitutionIterations: get the number of restitution
// iterations.
func (worldId WorldID) GetRestitutionIterations() int {
	world := getWorldFromId(worldId)
	return int(world.restitutionIterations)
}

// EnableRestitutionPropagation is b3World_EnableRestitutionPropagation: enable full contact
// propagation in the restitution solver.
func (worldId WorldID) EnableRestitutionPropagation(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if flag == world.enableRestitutionPropagation {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldEnableRestitutionPropagation{worldId, flag}
		recWriteWorldEnableRestitutionPropagation(world.recording, &recArgs)
	}

	world.enableRestitutionPropagation = flag
}

// IsRestitutionPropagationEnabled is b3World_IsRestitutionPropagationEnabled.
func (worldId WorldID) IsRestitutionPropagationEnabled() bool {
	world := getWorldFromId(worldId)
	return world.enableRestitutionPropagation
}

// SetHitEventThreshold is b3World_SetHitEventThreshold: adjust the hit event threshold. This
// controls the collision speed needed to generate a ContactHitEvent. Usually in meters per second.
func (worldId WorldID) SetHitEventThreshold(value float32) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetHitEventThreshold{worldId, value}
		recWriteWorldSetHitEventThreshold(world.recording, &recArgs)
	}

	world.hitEventThreshold = ClampFloat(value, 0.0, fltMax)
}

// GetHitEventThreshold is b3World_GetHitEventThreshold: get the hit event speed threshold. Usually
// in meters per second.
func (worldId WorldID) GetHitEventThreshold() float32 {
	world := getWorldFromId(worldId)
	return world.hitEventThreshold
}

// SetContactTuning is b3World_SetContactTuning: adjust contact tuning parameters.
// hertz: the contact stiffness (cycles per second)
// dampingRatio: the contact bounciness with 1 being critical damping (non-dimensional)
// contactSpeed: the maximum contact constraint push out speed (meters per second)
// Note: Advanced feature
func (worldId WorldID) SetContactTuning(hertz, dampingRatio, contactSpeed float32) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetContactTuning{worldId, hertz, dampingRatio, contactSpeed}
		recWriteWorldSetContactTuning(world.recording, &recArgs)
	}

	world.contactHertz = ClampFloat(hertz, 0.0, fltMax)
	world.contactDampingRatio = ClampFloat(dampingRatio, 0.0, fltMax)
	world.contactSpeed = ClampFloat(contactSpeed, 0.0, fltMax)
}

// SetContactRecycleDistance is b3World_SetContactRecycleDistance: set the contact recycling
// distance. Zero disables recycling.
func (worldId WorldID) SetContactRecycleDistance(recycleDistance float32) {
	world := getWorldFromId(worldId)
	assert(world.locked == false, "world->locked == false")
	if world.locked {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetContactRecycleDistance{worldId, recycleDistance}
		recWriteWorldSetContactRecycleDistance(world.recording, &recArgs)
	}

	world.contactRecycleDistance = ClampFloat(recycleDistance, 0.0, fltMax)
}

// GetContactRecycleDistance is b3World_GetContactRecycleDistance.
func (worldId WorldID) GetContactRecycleDistance() float32 {
	world := getWorldFromId(worldId)
	return world.contactRecycleDistance
}

// SetMaximumLinearSpeed is b3World_SetMaximumLinearSpeed: set the maximum linear speed. Usually in
// m/s.
func (worldId WorldID) SetMaximumLinearSpeed(maximumLinearSpeed float32) {
	assert(IsValidFloat(maximumLinearSpeed) && maximumLinearSpeed > 0.0,
		"b3IsValidFloat( maximumLinearSpeed ) && maximumLinearSpeed > 0.0f")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetMaximumLinearSpeed{worldId, maximumLinearSpeed}
		recWriteWorldSetMaximumLinearSpeed(world.recording, &recArgs)
	}

	world.maxLinearSpeed = maximumLinearSpeed
}

// GetMaximumLinearSpeed is b3World_GetMaximumLinearSpeed: get the maximum linear speed. Usually in
// m/s.
func (worldId WorldID) GetMaximumLinearSpeed() float32 {
	world := getWorldFromId(worldId)
	return world.maxLinearSpeed
}

// SetIsotropicInertiaTolerance is b3World_SetIsotropicInertiaTolerance: use this to disable
// gyroscopic torque on sphere-like objects automatically. Range is [0, 1].
func (worldId WorldID) SetIsotropicInertiaTolerance(tolerance float32) {
	assert(IsValidFloat(tolerance), "b3IsValidFloat( tolerance )")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldSetIsotropicInertiaTolerance{worldId, tolerance}
		recWriteWorldSetIsotropicInertiaTolerance(world.recording, &recArgs)
	}

	world.isotropicInertiaTolerance = ClampFloat(tolerance, 0.0, 1.0)
}

// GetIsotropicInertiaTolerance is b3World_GetIsotropicInertiaTolerance.
func (worldId WorldID) GetIsotropicInertiaTolerance() float32 {
	world := getWorldFromId(worldId)
	return world.isotropicInertiaTolerance
}

// GetProfile is b3World_GetProfile: get the current world performance profile.
func (worldId WorldID) GetProfile() Profile {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return Profile{}
	}
	return world.profile
}

// GetMaxCapacity is b3World_GetMaxCapacity: get the maximum capacity reached by the world. Use this
// to tune WorldDef.Capacity.
func (worldId WorldID) GetMaxCapacity() Capacity {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return Capacity{}
	}
	return world.maxCapacity
}

// SetUserData is b3World_SetUserData: set the user data.
func (worldId WorldID) SetUserData(userData any) {
	world := getWorldFromId(worldId)
	world.userData = userData
}

// GetUserData is b3World_GetUserData: get the user data.
func (worldId WorldID) GetUserData() any {
	world := getWorldFromId(worldId)
	return world.userData
}

// SetFrictionCallback is b3World_SetFrictionCallback: set the friction callback. Passing nil
// resets to default.
func (worldId WorldID) SetFrictionCallback(callback FrictionCallback) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if callback != nil {
		world.frictionCallback = callback
	} else {
		world.frictionCallback = defaultFrictionCallback
	}
}

// SetRestitutionCallback is b3World_SetRestitutionCallback: set the restitution callback. Passing
// nil resets to default.
func (worldId WorldID) SetRestitutionCallback(callback RestitutionCallback) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if callback != nil {
		world.restitutionCallback = callback
	} else {
		world.restitutionCallback = defaultRestitutionCallback
	}
}

// SetWorkerCount is b3World_SetWorkerCount: set the worker count. This is clamped to
// [1, MaxWorkers].
func (worldId WorldID) SetWorkerCount(count int) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if count == int(world.workerCount) {
		return
	}

	destroyWorkerContexts(world)
	world.workerCount = int32(ClampInt(count, 1, MaxWorkers))
	createWorkerContexts(world)
}

// GetWorkerCount is b3World_GetWorkerCount: get the worker count.
func (worldId WorldID) GetWorkerCount() int {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return 0
	}

	return int(world.workerCount)
}

// StartRecording is b3World_StartRecording: begin recording into a recording created by
// CreateRecording. Must be called at a step boundary. Worlds using the opt-in
// speculative CCD extension are refused because that mode is not in the format.
func (worldId WorldID) StartRecording(recording *Recording) {
	// Must be a step boundary, so refuse a locked world
	world := getUnlockedWorldFromId(worldId)

	if world == nil || recording == nil || world.recording != nil || world.speculativeCCD.enabled {
		return
	}

	startRecordingIntoBuffer(world, recording)
}

// StopRecording is b3World_StopRecording: stop the active recording.
func (worldId WorldID) StopRecording() {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	stopRecordingInternal(world)
}

// getWorldByteCount is b3GetWorldByteCount.
// PORT: sizeof of internal structs is the Go size (unsafe.Sizeof), and array bytes use the slice
// capacity (arrayByteCount), so totals differ from the C build. Diagnostic only.
func getWorldByteCount(world *world, log_ bool) uint64 {
	total := uint64(0)

	// id pools
	bodyIdBytes := getIdBytes(&world.bodyIdPool)
	solverSetIdBytes := getIdBytes(&world.solverSetIdPool)
	jointIdBytes := getIdBytes(&world.jointIdPool)
	contactIdBytes := getIdBytes(&world.contactIdPool)
	islandIdBytes := getIdBytes(&world.islandIdPool)
	shapeIdBytes := getIdBytes(&world.shapeIdPool)
	total += uint64(bodyIdBytes + solverSetIdBytes + jointIdBytes + contactIdBytes + islandIdBytes + shapeIdBytes)

	if log_ {
		log("id pools")
		log("body ids: %d", bodyIdBytes)
		log("solver set ids: %d", solverSetIdBytes)
		log("joint ids: %d", jointIdBytes)
		log("contact ids: %d", contactIdBytes)
		log("island ids: %d", islandIdBytes)
		log("shape ids: %d", shapeIdBytes)
	}

	// Islands own per-island body/contact/joint link arrays
	islandLinkBytes := 0
	for i := 0; i < len(world.islands); i++ {
		island := &world.islands[i]
		islandLinkBytes += arrayByteCount(island.bodies)
		islandLinkBytes += arrayByteCount(island.contacts)
		islandLinkBytes += arrayByteCount(island.joints)
	}

	// world arrays
	bodyArrayBytes := arrayByteCount(world.bodies)
	solverSetArrayBytes := arrayByteCount(world.solverSets)
	jointArrayBytes := arrayByteCount(world.joints)
	contactArrayBytes := arrayByteCount(world.contacts)
	islandArrayBytes := arrayByteCount(world.islands)
	shapeArrayBytes := arrayByteCount(world.shapes)
	sensorArrayBytes := arrayByteCount(world.sensors)
	total += uint64(bodyArrayBytes + solverSetArrayBytes + jointArrayBytes + contactArrayBytes + islandArrayBytes +
		islandLinkBytes + shapeArrayBytes + sensorArrayBytes)

	if log_ {
		log("world arrays")
		log("bodies: %d", bodyArrayBytes)
		log("solver sets: %d", solverSetArrayBytes)
		log("joints: %d", jointArrayBytes)
		log("contacts: %d", contactArrayBytes)
		log("islands: %d", islandArrayBytes)
		log("island links: %d", islandLinkBytes)
		log("shapes: %d", shapeArrayBytes)
		log("sensors: %d", sensorArrayBytes)
	}

	// Sensors own overlap tracking arrays. The sensor array is dense.
	sensorOverlapBytes := 0
	for i := 0; i < len(world.sensors); i++ {
		sensor := &world.sensors[i]
		sensorOverlapBytes += arrayByteCount(sensor.hits)
		sensorOverlapBytes += arrayByteCount(sensor.overlaps1)
		sensorOverlapBytes += arrayByteCount(sensor.overlaps2)
	}
	total += uint64(sensorOverlapBytes)

	if log_ {
		log("owned arrays")
		log("sensor overlaps: %d", sensorOverlapBytes)
	}

	// Shared hull database. The map owns a combined bucket and metadata allocation
	// plus the small map struct. Each stored key is an owned clone sized by byteCount.
	hullDatabase := world.hullDatabase
	hullCount := hullDatabase.size()
	hullBucketCount := hullDatabase.bucketCount()
	hullMapBytes := uint64(hullMapByteCount(hullDatabase))
	hullDataBytes := uint64(0)
	for itr := hullDatabase.first(); hullDatabase.isEnd(itr) == false; itr = hullDatabase.next(itr) {
		hullDataBytes += uint64(itr.data.key.ByteCount)
	}
	total += hullMapBytes + hullDataBytes

	if log_ {
		log("hulls")
		log("database: %d (%d, %d)", int(hullMapBytes), hullCount, hullBucketCount)
		log("hull data: %d", int(hullDataBytes))
	}

	// broad-phase
	staticTreeBytes := world.broadPhase.trees[StaticBody].GetByteCount()
	kinematicTreeBytes := world.broadPhase.trees[KinematicBody].GetByteCount()
	dynamicTreeBytes := world.broadPhase.trees[DynamicBody].GetByteCount()
	pairSet := &world.broadPhase.pairSet
	pairSetBytes := getHashSetBytes(pairSet)
	total += uint64(staticTreeBytes + kinematicTreeBytes + dynamicTreeBytes + pairSetBytes)

	if log_ {
		log("broad-phase")
		log("static tree: %d", staticTreeBytes)
		log("kinematic tree: %d", kinematicTreeBytes)
		log("dynamic tree: %d", dynamicTreeBytes)
		log("pairSet: %d (%d, %d)", pairSetBytes, pairSet.count, pairSet.capacity)
	}

	// Manifold block allocators, one per manifold point count
	// PORT: elementSize counts manifolds, so the block bytes multiply by the Go manifold size.
	manifoldArrayBytes := arrayByteCount(world.manifoldAllocators)
	manifoldBlockBytes := 0
	for i := 0; i < len(world.manifoldAllocators); i++ {
		allocator := &world.manifoldAllocators[i]
		manifoldBlockBytes += arrayByteCount(allocator.blocks)
		manifoldBlockBytes += len(allocator.blocks) * blockSize * allocator.elementSize * int(unsafe.Sizeof(Manifold{}))
	}
	total += uint64(manifoldArrayBytes + manifoldBlockBytes)

	if log_ {
		log("manifold allocators")
		log("allocator array: %d", manifoldArrayBytes)
		log("blocks: %d", manifoldBlockBytes)
	}

	// solver sets
	bodySimCapacity := 0
	bodyStateCapacity := 0
	jointSimCapacity := 0
	contactIndexCapacity := 0
	islandSimCapacity := 0
	solverSetCapacity := len(world.solverSets)
	for i := 0; i < solverSetCapacity; i++ {
		set := &world.solverSets[i]
		if set.setIndex == NullIndex {
			continue
		}

		bodySimCapacity += cap(set.bodySims)
		bodyStateCapacity += cap(set.bodyStates)
		jointSimCapacity += cap(set.jointSims)
		contactIndexCapacity += cap(set.contactIndices)
		islandSimCapacity += cap(set.islandSims)
	}

	setBodySimBytes := bodySimCapacity * int(unsafe.Sizeof(bodySim{}))
	setBodyStateBytes := bodyStateCapacity * int(unsafe.Sizeof(bodyState{}))
	setJointSimBytes := jointSimCapacity * int(unsafe.Sizeof(jointSim{}))
	setContactSimBytes := contactIndexCapacity * int(unsafe.Sizeof(int32(0)))
	setIslandSimBytes := islandSimCapacity * int(unsafe.Sizeof(islandSim{}))
	total += uint64(setBodySimBytes + setBodyStateBytes + setJointSimBytes + setContactSimBytes + setIslandSimBytes)

	if log_ {
		log("solver sets")
		log("body sim: %d", setBodySimBytes)
		log("body state: %d", setBodyStateBytes)
		log("joint sim: %d", setJointSimBytes)
		log("contact sim: %d", setContactSimBytes)
		log("island sim: %d", setIslandSimBytes)
	}

	// constraint graph
	bodyBitSetBytes := 0
	graphContactBytes := 0
	graphJointSimBytes := 0
	for i := 0; i < GraphColorCount; i++ {
		c := &world.constraintGraph.colors[i]
		bodyBitSetBytes += getBitSetBytes(&c.bodySet)
		graphContactBytes += arrayByteCount(c.convexContacts) + arrayByteCount(c.contacts)
		graphJointSimBytes += arrayByteCount(c.jointSims)
	}
	total += uint64(bodyBitSetBytes + graphJointSimBytes + graphContactBytes)

	if log_ {
		log("constraint graph")
		log("body bit sets: %d", bodyBitSetBytes)
		log("joint sim: %d", graphJointSimBytes)
		log("contact sim: %d", graphContactBytes)
	}

	// Per worker task storage and its bit sets
	taskContextBytes := arrayByteCount(world.taskContexts)
	for i := 0; i < len(world.taskContexts); i++ {
		taskContext := &world.taskContexts[i]
		taskContextBytes += arrayByteCount(taskContext.sensorHits)
		taskContextBytes += arrayByteCount(taskContext.pairKeys)
		taskContextBytes += getBitSetBytes(&taskContext.contactStateBitSet)
		taskContextBytes += getBitSetBytes(&taskContext.jointStateBitSet)
		taskContextBytes += getBitSetBytes(&taskContext.hitEventBitSet)
		taskContextBytes += getBitSetBytes(&taskContext.awakeIslandBitSet)
	}

	sensorTaskContextBytes := arrayByteCount(world.sensorTaskContexts)
	for i := 0; i < len(world.sensorTaskContexts); i++ {
		taskContext := &world.sensorTaskContexts[i]
		sensorTaskContextBytes += getBitSetBytes(&taskContext.eventBits)
	}
	total += uint64(taskContextBytes + sensorTaskContextBytes)

	if log_ {
		log("task contexts")
		log("worker: %d", taskContextBytes)
		log("sensor: %d", sensorTaskContextBytes)
	}

	// Double buffered event arrays
	eventBytes := 0
	eventBytes += arrayByteCount(world.bodyMoveEvents)
	eventBytes += arrayByteCount(world.sensorBeginEvents)
	eventBytes += arrayByteCount(world.contactBeginEvents)
	eventBytes += arrayByteCount(world.sensorEndEvents[0])
	eventBytes += arrayByteCount(world.sensorEndEvents[1])
	eventBytes += arrayByteCount(world.contactEndEvents[0])
	eventBytes += arrayByteCount(world.contactEndEvents[1])
	eventBytes += arrayByteCount(world.contactHitEvents)
	eventBytes += arrayByteCount(world.jointEvents)
	total += uint64(eventBytes)

	if log_ {
		log("events: %d", eventBytes)
	}

	// Debug draw bit sets
	debugBytes := 0
	debugBytes += getBitSetBytes(&world.debugBodySet)
	debugBytes += getBitSetBytes(&world.debugJointSet)
	debugBytes += getBitSetBytes(&world.debugContactSet)
	debugBytes += getBitSetBytes(&world.debugIslandSet)
	total += uint64(debugBytes)

	if log_ {
		log("debug draw: %d", debugBytes)
	}

	// stack allocator
	total += uint64(world.stack.capacity)

	if log_ {
		log("stack allocator: %d", world.stack.capacity)
		log("total: %d KB", uint32(total/1024))
	}

	return total
}

// DumpMemoryStats is b3World_DumpMemoryStats: dump memory stats to the log.
func (worldId WorldID) DumpMemoryStats() {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	getWorldByteCount(world, true)
}

// GetCounters is b3World_GetCounters: get world counters and sizes.
func (worldId WorldID) GetCounters() Counters {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return Counters{}
	}

	s := Counters{}
	s.BodyCount = int32(getIdCount(&world.bodyIdPool))
	s.ShapeCount = int32(getIdCount(&world.shapeIdPool))
	s.ContactCount = int32(getIdCount(&world.contactIdPool))
	s.JointCount = int32(getIdCount(&world.jointIdPool))
	s.IslandCount = int32(getIdCount(&world.islandIdPool))

	staticTree := &world.broadPhase.trees[StaticBody]
	s.StaticTreeHeight = int32(staticTree.GetHeight())

	dynamicTree := &world.broadPhase.trees[DynamicBody]
	kinematicTree := &world.broadPhase.trees[KinematicBody]
	s.TreeHeight = int32(MaxInt(dynamicTree.GetHeight(), kinematicTree.GetHeight()))

	s.SatCallCount = world.satCallCount
	s.SatCacheHitCount = world.satCacheHitCount
	s.ManifoldCounts = world.manifoldCounts
	s.StackUsed = int32(world.stack.maxAllocation)
	s.ByteCount = int64(getWorldByteCount(world, false))
	s.TaskCount = world.taskCount

	// PORT: _Static_assert( B3_GRAPH_COLOR_COUNT == B3_ARRAY_COUNT( s.colorCounts ) ) as an
	// overflowing constant pair.
	const (
		_ uint = uint(GraphColorCount - len(Counters{}.ColorCounts))
		_ uint = uint(len(Counters{}.ColorCounts) - GraphColorCount)
	)

	s.AwakeContactCount = 0
	for i := 0; i < GraphColorCount; i++ {
		color := &world.constraintGraph.colors[i]
		colorContactCount := len(color.convexContacts) + len(color.contacts)
		s.ColorCounts[i] = int32(colorContactCount + len(color.jointSims))
		s.AwakeContactCount += int32(colorContactCount)
	}
	s.AwakeContactCount += int32(len(world.solverSets[awakeSet].contactIndices))

	s.RecycledContactCount = 0
	s.ArenaCapacity = 0
	s.DistanceIterations = 0
	s.PushBackIterations = 0
	s.RootIterations = 0
	for i := 0; i < int(world.workerCount); i++ {
		s.RecycledContactCount += world.taskContexts[i].recycledContactCount

		s.DistanceIterations = int32(MaxInt(int(s.DistanceIterations), int(world.taskContexts[i].distanceIterations)))
		s.PushBackIterations = int32(MaxInt(int(s.PushBackIterations), int(world.taskContexts[i].pushBackIterations)))
		s.RootIterations = int32(MaxInt(int(s.RootIterations), int(world.taskContexts[i].rootIterations)))

		peak := world.taskContexts[i].arena.shared.peakDemand
		if peak > int(s.ArenaCapacity) {
			s.ArenaCapacity = int32(peak)
		}
	}

	return s
}

// PORT: b3World_OverlapAABB through b3World_CastMover (WorldQueryContext … MoverCastCallback) are
// in physics_world_queries.go, in upstream order. Upstream continues here with
// b3World_SetCustomFilterCallback.

// SetCustomFilterCallback is b3World_SetCustomFilterCallback: register the custom filter callback.
// This is optional.
func (worldId WorldID) SetCustomFilterCallback(fcn CustomFilterFcn, context any) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}
	world.customFilterFcn = fcn
	world.customFilterContext = context
}

// SetPreSolveCallback is b3World_SetPreSolveCallback: register the pre-solve callback. This is
// optional.
func (worldId WorldID) SetPreSolveCallback(fcn PreSolveFcn, context any) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}
	world.preSolveFcn = fcn
	world.preSolveContext = context
}

// SetGravity is b3World_SetGravity: set the gravity vector for the entire world. Box3D has no
// concept of an up direction and this is left as a decision for the application. Usually in m/s^2.
func (worldId WorldID) SetGravity(gravity Vec3) {
	world := getWorldFromId(worldId)

	if world.recording != nil {
		recArgs := recArgsWorldSetGravity{worldId, gravity}
		recWriteWorldSetGravity(world.recording, &recArgs)
	}

	world.gravity = gravity
}

// GetGravity is b3World_GetGravity: get the gravity vector.
func (worldId WorldID) GetGravity() Vec3 {
	world := getWorldFromId(worldId)
	return world.gravity
}

// explosionContext is ExplosionContext.
type explosionContext struct {
	world          *world
	position       Pos
	radius         float32
	falloff        float32
	impulsePerArea float32
}

// explosionCallback is ExplosionCallback.
func explosionCallback(proxyId int, userData uint64, context any) bool {
	shapeId := int(userData)
	explosionContext := context.(*explosionContext)
	world := explosionContext.world

	shape := arrayGet(world.shapes, shapeId)
	if shape.explosionScale == 0.0 {
		return true
	}

	body := arrayGet(world.bodies, int(shape.bodyId))
	assert(body.typ == DynamicBody, "body->type == b3_dynamicBody")

	xf := getBodyTransformQuick(world, body)

	// Re-center the explosion into the shape local frame so distance and direction stay precise
	// far from the origin. Everything below runs in that near-origin frame.
	localPosition := InvTransformWorldPoint(xf, explosionContext.position)

	var input DistanceInput
	input.ProxyA = makeShapeProxy(shape)
	// PORT: ( b3ShapeProxy ){ &localPosition, 1, 0.0f } aliases the local.
	input.ProxyB = ShapeProxy{unsafe.Slice(&localPosition, 1), 1, 0.0}
	input.Transform = TransformIdentity
	input.UseRadii = true

	cache := SimplexCache{}
	output := ShapeDistance(&input, &cache, nil)

	radius := explosionContext.radius
	falloff := explosionContext.falloff
	if output.Distance > radius+falloff {
		return true
	}

	wakeBody(world, body)

	if body.setIndex != awakeSet {
		return true
	}

	// Witness point is already in the body local query frame
	closestPoint := output.PointA
	if output.Distance == 0.0 {
		closestPoint = getShapeCentroid(shape)
	}

	direction := Sub(closestPoint, localPosition)
	if LengthSquared(direction) > 100.0*fltEpsilon*fltEpsilon {
		direction = Normalize(direction)
	} else {
		direction = Vec3{1.0, 0.0, 0.0}
	}

	area := getShapeProjectedArea(shape, direction)
	scale := float32(1.0)
	if output.Distance > radius && falloff > 0.0 {
		scale = ClampFloat((radius+falloff-output.Distance)/falloff, 0.0, 1.0)
	}

	magnitude := explosionContext.impulsePerArea * area * scale * shape.explosionScale
	impulse := MulSV(magnitude, RotateVector(xf.Q, direction))

	localIndex := int(body.localIndex)
	set := arrayGet(world.solverSets, awakeSet)
	state := arrayGet(set.bodyStates, localIndex)
	bodySim := arrayGet(set.bodySims, localIndex)
	state.linearVelocity = MulAdd(state.linearVelocity, bodySim.invMass, impulse)

	// Lever arm from the center of mass to the closest point, rotated to world
	r := RotateVector(xf.Q, Sub(closestPoint, bodySim.localCenter))
	state.angularVelocity = Add(state.angularVelocity, MulMV(bodySim.invInertiaWorld, Cross(r, impulse)))

	return true
}

// Explode is b3World_Explode: apply a radial explosion.
func (worldId WorldID) Explode(explosionDef *ExplosionDef) {
	maskBits := explosionDef.MaskBits
	position := explosionDef.Position
	radius := explosionDef.Radius
	falloff := explosionDef.Falloff
	impulsePerArea := explosionDef.ImpulsePerArea

	assert(IsValidPosition(position), "b3IsValidPosition( position )")
	assert(IsValidFloat(radius) && radius >= 0.0, "b3IsValidFloat( radius ) && radius >= 0.0f")
	assert(IsValidFloat(falloff) && falloff >= 0.0, "b3IsValidFloat( falloff ) && falloff >= 0.0f")
	assert(IsValidFloat(impulsePerArea), "b3IsValidFloat( impulsePerArea )")

	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldExplode{worldId, *explosionDef}
		recWriteWorldExplode(world.recording, &recArgs)
	}

	// Locked due to waking
	world.locked = true

	explosionContext := explosionContext{world, position, radius, falloff, impulsePerArea}

	// The broad-phase tree is float, so translate a local query box out to world with outward rounding
	extent := radius + falloff
	localBox := AABB{Vec3{-extent, -extent, -extent}, Vec3{extent, extent, extent}}
	aabb := OffsetAABB(localBox, position)

	world.broadPhase.trees[DynamicBody].Query(aabb, maskBits, false, explosionCallback, &explosionContext)

	world.locked = false
}

// RebuildStaticTree is b3World_RebuildStaticTree: rebuild the static tree. This is only needed if
// you have many static bodies with moving shapes.
func (worldId WorldID) RebuildStaticTree() {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldRebuildStaticTree{worldId}
		recWriteWorldRebuildStaticTree(world.recording, &recArgs)
	}

	staticTree := &world.broadPhase.trees[StaticBody]
	staticTree.Rebuild(true)
}

// EnableSpeculative is b3World_EnableSpeculative: enable/disable speculative collision. This is a
// testing feature.
func (worldId WorldID) EnableSpeculative(flag bool) {
	world := getUnlockedWorldFromId(worldId)
	if world == nil {
		return
	}

	if world.recording != nil {
		recArgs := recArgsWorldEnableSpeculative{worldId, flag}
		recWriteWorldEnableSpeculative(world.recording, &recArgs)
	}

	world.enableSpeculative = flag
}

// PORT: #if B3_ENABLE_VALIDATION selects between these bodies and empty functions. Here each
// function returns early unless validateEnabled (box3d_validate build tag).

// validateConnectivity is b3ValidateConnectivity.
// This validates island graph connectivity for each body
func validateConnectivity(world *world) {
	if !validateEnabled {
		return
	}

	bodies := world.bodies
	bodyCapacity := len(world.bodies)

	for bodyIndex := 0; bodyIndex < bodyCapacity; bodyIndex++ {
		body := &bodies[bodyIndex]
		if body.id == NullIndex {
			validateFreeId(&world.bodyIdPool, bodyIndex)
			continue
		}

		assert(bodyIndex == int(body.id), "bodyIndex == body->id")

		// Need to get the root island because islands are not merged until the next time step
		bodyIslandId := body.islandId
		bodySetIndex := body.setIndex

		contactKey := int(body.headContactKey)
		for contactKey != NullIndex {
			contactId := contactKey >> 1
			edgeIndex := contactKey & 1

			contact := arrayGet(world.contacts, contactId)

			touching := (contact.flags & contactTouchingFlag) != 0
			if touching {
				if bodySetIndex != staticSet {
					contactIslandId := contact.islandId
					assert(contactIslandId == bodyIslandId, "contactIslandId == bodyIslandId")
				}
			} else {
				assert(contact.islandId == NullIndex, "contact->islandId == B3_NULL_INDEX")
			}

			contactKey = int(contact.edges[edgeIndex].nextKey)
		}

		jointKey := int(body.headJointKey)
		for jointKey != NullIndex {
			jointId := jointKey >> 1
			edgeIndex := jointKey & 1

			joint := arrayGet(world.joints, jointId)

			otherEdgeIndex := edgeIndex ^ 1

			otherBody := arrayGet(world.bodies, int(joint.edges[otherEdgeIndex].bodyId))

			if bodySetIndex == disabledSet || otherBody.setIndex == disabledSet {
				assert(joint.islandId == NullIndex, "joint->islandId == B3_NULL_INDEX")
			} else if bodySetIndex == staticSet {
				// Intentional nesting
				if otherBody.setIndex == staticSet {
					assert(joint.islandId == NullIndex, "joint->islandId == B3_NULL_INDEX")
				}
			} else if body.typ != DynamicBody && otherBody.typ != DynamicBody {
				assert(joint.islandId == NullIndex, "joint->islandId == B3_NULL_INDEX")
			} else {
				jointIslandId := joint.islandId
				assert(jointIslandId == bodyIslandId, "jointIslandId == bodyIslandId")
			}

			jointKey = int(joint.edges[edgeIndex].nextKey)
		}
	}
}

// validateSolverSets is b3ValidateSolverSets.
// Validates solver sets, but not island connectivity
func validateSolverSets(world *world) {
	if !validateEnabled {
		return
	}

	assert(getIdCapacity(&world.bodyIdPool) == len(world.bodies), "b3GetIdCapacity( &world->bodyIdPool ) == world->bodies.count")
	assert(getIdCapacity(&world.contactIdPool) == len(world.contacts),
		"b3GetIdCapacity( &world->contactIdPool ) == world->contacts.count")
	assert(getIdCapacity(&world.jointIdPool) == len(world.joints), "b3GetIdCapacity( &world->jointIdPool ) == world->joints.count")
	assert(getIdCapacity(&world.islandIdPool) == len(world.islands),
		"b3GetIdCapacity( &world->islandIdPool ) == world->islands.count")
	assert(getIdCapacity(&world.solverSetIdPool) == len(world.solverSets),
		"b3GetIdCapacity( &world->solverSetIdPool ) == world->solverSets.count")

	// Ensure the encoded body sim indices are correct.
	for contactIndex := 0; contactIndex < len(world.contacts); contactIndex++ {
		contact := &world.contacts[contactIndex]
		if contact.contactId == NullIndex {
			continue
		}

		bodyA := arrayGet(world.bodies, int(contact.edges[0].bodyId))
		bodyB := arrayGet(world.bodies, int(contact.edges[1].bodyId))
		assert(int(contact.encodedBodySimA) == encodeBodySimIndex(bodyA), "contact->encodedBodySimA == b3EncodeBodySimIndex( bodyA )")
		assert(int(contact.encodedBodySimB) == encodeBodySimIndex(bodyB), "contact->encodedBodySimB == b3EncodeBodySimIndex( bodyB )")
	}

	activeSetCount := 0
	totalBodyCount := 0
	totalJointCount := 0
	totalContactCount := 0
	totalIslandCount := 0

	// Validate all solver sets
	setCount := len(world.solverSets)
	for setIndex := 0; setIndex < setCount; setIndex++ {
		set := &world.solverSets[setIndex]
		if set.setIndex != NullIndex {
			activeSetCount += 1

			if setIndex == staticSet {
				assert(len(set.contactIndices) == 0, "set->contactIndices.count == 0")
				assert(len(set.islandSims) == 0, "set->islandSims.count == 0")
				assert(len(set.bodyStates) == 0, "set->bodyStates.count == 0")
			} else if setIndex == disabledSet {
				assert(len(set.islandSims) == 0, "set->islandSims.count == 0")
				assert(len(set.bodyStates) == 0, "set->bodyStates.count == 0")
			} else if setIndex == awakeSet {
				assert(len(set.bodySims) == len(set.bodyStates), "set->bodySims.count == set->bodyStates.count")
				assert(len(set.jointSims) == 0, "set->jointSims.count == 0")
			} else {
				assert(len(set.bodyStates) == 0, "set->bodyStates.count == 0")
			}

			// Validate bodies
			{
				bodies := world.bodies
				assert(len(set.bodySims) >= 0, "set->bodySims.count >= 0")
				totalBodyCount += len(set.bodySims)
				for i := 0; i < len(set.bodySims); i++ {
					bodySim := &set.bodySims[i]

					bodyId := int(bodySim.bodyId)
					assert(0 <= bodyId && bodyId < len(world.bodies), "0 <= bodyId && bodyId < world->bodies.count")
					body := &bodies[bodyId]
					assert(int(body.setIndex) == setIndex, "body->setIndex == setIndex")
					assert(int(body.localIndex) == i, "body->localIndex == i")

					syncedFlags := body.flags &^ (isFast | bodyTransientFlags)
					assert((bodySim.flags&syncedFlags) == syncedFlags, "( bodySim->flags & syncedFlags ) == syncedFlags")

					bodyState := getBodyState(world, body)
					if bodyState != nil {
						assert((bodyState.flags&syncedFlags) == syncedFlags, "( bodyState->flags & syncedFlags ) == syncedFlags")
					}

					if body.typ == DynamicBody {
						assert(body.flags&dynamicFlag != 0, "body->flags & b3_dynamicFlag")
					}

					if setIndex == disabledSet {
						assert(body.headContactKey == NullIndex, "body->headContactKey == B3_NULL_INDEX")
					}

					// Validate body shapes
					prevShapeId := int32(NullIndex)
					shapeId := int(body.headShapeId)
					for shapeId != NullIndex {
						shape := arrayGet(world.shapes, shapeId)
						assert(int(shape.id) == shapeId, "shape->id == shapeId")
						assert(shape.prevShapeId == prevShapeId, "shape->prevShapeId == prevShapeId")

						if setIndex == disabledSet {
							assert(shape.proxyKey == NullIndex, "shape->proxyKey == B3_NULL_INDEX")
						} else if setIndex == staticSet {
							assert(proxyType(int(shape.proxyKey)) == StaticBody, "B3_PROXY_TYPE( shape->proxyKey ) == b3_staticBody")
						} else {
							proxyType := proxyType(int(shape.proxyKey))
							assert(proxyType == KinematicBody || proxyType == DynamicBody,
								"proxyType == b3_kinematicBody || proxyType == b3_dynamicBody")
						}

						prevShapeId = int32(shapeId)
						shapeId = int(shape.nextShapeId)
					}

					// Validate body contacts
					contactKey := int(body.headContactKey)
					for contactKey != NullIndex {
						contactId := contactKey >> 1
						edgeIndex := contactKey & 1

						contact := arrayGet(world.contacts, contactId)
						assert(contact.setIndex != staticSet, "contact->setIndex != b3_staticSet")
						assert(int(contact.edges[0].bodyId) == bodyId || int(contact.edges[1].bodyId) == bodyId,
							"contact->edges[0].bodyId == bodyId || contact->edges[1].bodyId == bodyId")
						contactKey = int(contact.edges[edgeIndex].nextKey)
					}

					// Validate body joints
					jointKey := int(body.headJointKey)
					for jointKey != NullIndex {
						jointId := jointKey >> 1
						edgeIndex := jointKey & 1

						joint := arrayGet(world.joints, jointId)

						otherEdgeIndex := edgeIndex ^ 1

						otherBody := arrayGet(world.bodies, int(joint.edges[otherEdgeIndex].bodyId))

						if setIndex == disabledSet || otherBody.setIndex == disabledSet {
							assert(joint.setIndex == disabledSet, "joint->setIndex == b3_disabledSet")
						} else if setIndex == staticSet && otherBody.setIndex == staticSet {
							assert(joint.setIndex == staticSet, "joint->setIndex == b3_staticSet")
						} else if body.typ != DynamicBody && otherBody.typ != DynamicBody {
							assert(joint.setIndex == staticSet, "joint->setIndex == b3_staticSet")
						} else if setIndex == awakeSet {
							assert(joint.setIndex == awakeSet, "joint->setIndex == b3_awakeSet")
						} else if setIndex >= firstSleepingSet {
							assert(int(joint.setIndex) == setIndex, "joint->setIndex == setIndex")
						}

						jointSim := getJointSim(world, joint)
						assert(int(jointSim.jointId) == jointId, "jointSim->jointId == jointId")
						assert(jointSim.bodyIdA == joint.edges[0].bodyId, "jointSim->bodyIdA == joint->edges[0].bodyId")
						assert(jointSim.bodyIdB == joint.edges[1].bodyId, "jointSim->bodyIdB == joint->edges[1].bodyId")

						jointKey = int(joint.edges[edgeIndex].nextKey)
					}
				}
			}

			// Validate contacts
			{
				assert(len(set.contactIndices) >= 0, "set->contactIndices.count >= 0")
				totalContactCount += len(set.contactIndices)
				for i := 0; i < len(set.contactIndices); i++ {
					contactIndex := int(set.contactIndices[i])
					contact := arrayGet(world.contacts, contactIndex)
					if setIndex == awakeSet {
						// contact should be non-touching if awake
						// or it could be this contact hasn't been transferred yet
						assert(contact.manifoldCount == 0 || (contact.flags&simStartedTouching) != 0,
							"contact->manifoldCount == 0 || ( contact->flags & b3_simStartedTouching ) != 0")
					}
					assert(int(contact.setIndex) == setIndex, "contact->setIndex == setIndex")
					assert(contact.colorIndex == NullIndex, "contact->colorIndex == B3_NULL_INDEX")
					assert(int(contact.localIndex) == i, "contact->localIndex == i")
				}
			}

			// Validate joints
			{
				assert(len(set.jointSims) >= 0, "set->jointSims.count >= 0")
				totalJointCount += len(set.jointSims)
				for i := 0; i < len(set.jointSims); i++ {
					jointSim := &set.jointSims[i]
					joint := arrayGet(world.joints, int(jointSim.jointId))
					assert(int(joint.setIndex) == setIndex, "joint->setIndex == setIndex")
					assert(joint.colorIndex == NullIndex, "joint->colorIndex == B3_NULL_INDEX")
					assert(int(joint.localIndex) == i, "joint->localIndex == i")
				}
			}

			// Validate islands
			{
				assert(len(set.islandSims) >= 0, "set->islandSims.count >= 0")
				totalIslandCount += len(set.islandSims)
				for i := 0; i < len(set.islandSims); i++ {
					islandSim := &set.islandSims[i]
					island := arrayGet(world.islands, int(islandSim.islandId))
					assert(int(island.setIndex) == setIndex, "island->setIndex == setIndex")
					assert(int(island.localIndex) == i, "island->localIndex == i")
				}
			}
		} else {
			assert(len(set.bodySims) == 0, "set->bodySims.count == 0")
			assert(len(set.contactIndices) == 0, "set->contactIndices.count == 0")
			assert(len(set.jointSims) == 0, "set->jointSims.count == 0")
			assert(len(set.islandSims) == 0, "set->islandSims.count == 0")
			assert(len(set.bodyStates) == 0, "set->bodyStates.count == 0")
		}
	}

	setIdCount := getIdCount(&world.solverSetIdPool)
	assert(activeSetCount == setIdCount, "activeSetCount == setIdCount")

	bodyIdCount := getIdCount(&world.bodyIdPool)
	assert(totalBodyCount == bodyIdCount, "totalBodyCount == bodyIdCount")

	islandIdCount := getIdCount(&world.islandIdPool)
	assert(totalIslandCount == islandIdCount, "totalIslandCount == islandIdCount")

	// Validate constraint graph
	for colorIndex := 0; colorIndex < GraphColorCount; colorIndex++ {
		color := &world.constraintGraph.colors[colorIndex]
		bitCount := 0

		assert(len(color.convexContacts) >= 0, "color->convexContacts.count >= 0")
		totalContactCount += len(color.convexContacts)
		for i := 0; i < len(color.convexContacts); i++ {
			contactId := int(color.convexContacts[i])
			contact := arrayGet(world.contacts, contactId)
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			assert(contact.manifoldCount > 0 || (contact.flags&(simStoppedTouching|simDisjoint)) != 0,
				"contact->manifoldCount > 0 || ( contact->flags & ( b3_simStoppedTouching | b3_simDisjoint ) ) != 0")
			assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")
			assert(int(contact.colorIndex) == colorIndex, "contact->colorIndex == colorIndex")
			assert(int(contact.localIndex) == i, "contact->localIndex == i")

			bodyIdA := int(contact.edges[0].bodyId)
			bodyIdB := int(contact.edges[1].bodyId)

			if colorIndex < overflowIndex {
				bodyA := arrayGet(world.bodies, bodyIdA)
				bodyB := arrayGet(world.bodies, bodyIdB)
				assert(getBit(&color.bodySet, uint32(bodyIdA)) == (bodyA.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b3_dynamicBody )")
				assert(getBit(&color.bodySet, uint32(bodyIdB)) == (bodyB.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b3_dynamicBody )")

				bitCount += boolInt(bodyA.typ == DynamicBody)
				bitCount += boolInt(bodyB.typ == DynamicBody)
			}
		}

		totalContactCount += len(color.contacts)
		for i := 0; i < len(color.contacts); i++ {
			contactId := int(color.contacts[i].contactId)
			contact := arrayGet(world.contacts, contactId)
			// contact should be touching in the constraint graph or awaiting transfer to non-touching
			assert(contact.manifoldCount > 0 || (contact.flags&(simStoppedTouching|simDisjoint)) != 0,
				"contact->manifoldCount > 0 || ( contact->flags & ( b3_simStoppedTouching | b3_simDisjoint ) ) != 0")
			assert(contact.setIndex == awakeSet, "contact->setIndex == b3_awakeSet")
			assert(int(contact.colorIndex) == colorIndex, "contact->colorIndex == colorIndex")
			assert(int(contact.localIndex) == i, "contact->localIndex == i")

			bodyIdA := int(contact.edges[0].bodyId)
			bodyIdB := int(contact.edges[1].bodyId)

			if colorIndex < overflowIndex {
				bodyA := arrayGet(world.bodies, bodyIdA)
				bodyB := arrayGet(world.bodies, bodyIdB)
				assert(getBit(&color.bodySet, uint32(bodyIdA)) == (bodyA.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b3_dynamicBody )")
				assert(getBit(&color.bodySet, uint32(bodyIdB)) == (bodyB.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b3_dynamicBody )")

				bitCount += boolInt(bodyA.typ == DynamicBody)
				bitCount += boolInt(bodyB.typ == DynamicBody)
			}
		}

		assert(len(color.jointSims) >= 0, "color->jointSims.count >= 0")
		totalJointCount += len(color.jointSims)
		for i := 0; i < len(color.jointSims); i++ {
			jointSim := &color.jointSims[i]
			joint := arrayGet(world.joints, int(jointSim.jointId))
			assert(joint.setIndex == awakeSet, "joint->setIndex == b3_awakeSet")
			assert(int(joint.colorIndex) == colorIndex, "joint->colorIndex == colorIndex")
			assert(int(joint.localIndex) == i, "joint->localIndex == i")

			bodyIdA := int(joint.edges[0].bodyId)
			bodyIdB := int(joint.edges[1].bodyId)

			if colorIndex < overflowIndex {
				bodyA := arrayGet(world.bodies, bodyIdA)
				bodyB := arrayGet(world.bodies, bodyIdB)
				assert(getBit(&color.bodySet, uint32(bodyIdA)) == (bodyA.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdA ) == ( bodyA->type == b3_dynamicBody )")
				assert(getBit(&color.bodySet, uint32(bodyIdB)) == (bodyB.typ == DynamicBody),
					"b3GetBit( &color->bodySet, bodyIdB ) == ( bodyB->type == b3_dynamicBody )")

				bitCount += boolInt(bodyA.typ == DynamicBody)
				bitCount += boolInt(bodyB.typ == DynamicBody)
			}
		}

		// Validate the bit population for this graph color
		assert(bitCount == countSetBits(&color.bodySet), "bitCount == b3CountSetBits( &color->bodySet )")
	}

	contactIdCount := getIdCount(&world.contactIdPool)
	assert(totalContactCount == contactIdCount, "totalContactCount == contactIdCount")
	assert(totalContactCount == int(world.broadPhase.pairSet.count), "totalContactCount == (int)world->broadPhase.pairSet.count")

	jointIdCount := getIdCount(&world.jointIdPool)
	assert(totalJointCount == jointIdCount, "totalJointCount == jointIdCount")

	// Validate shapes
	// This is very slow on compounds
	// PORT: the #if 0 shape validation block is not ported (compiled out upstream).
}

// validateContacts is b3ValidateContacts.
// Validate contact touching status.
func validateContacts(world *world) {
	if !validateEnabled {
		return
	}

	graph := &world.constraintGraph
	contactCount := len(world.contacts)
	assert(contactCount == getIdCapacity(&world.contactIdPool), "contactCount == b3GetIdCapacity( &world->contactIdPool )")
	allocatedContactCount := 0

	for contactIndex := 0; contactIndex < contactCount; contactIndex++ {
		contact := arrayGet(world.contacts, contactIndex)
		if contact.contactId == NullIndex {
			continue
		}

		assert(int(contact.contactId) == contactIndex, "contact->contactId == contactIndex")

		allocatedContactCount += 1

		touching := (contact.flags & contactTouchingFlag) != 0

		setId := int(contact.setIndex)
		set := arrayGet(world.solverSets, setId)

		if setId == awakeSet {
			if touching {
				assert(0 <= contact.colorIndex && contact.colorIndex < GraphColorCount,
					"0 <= contact->colorIndex && contact->colorIndex < B3_GRAPH_COLOR_COUNT")

				if (contact.flags&simMeshContact) != 0 || contact.colorIndex == overflowIndex {
					color := &graph.colors[contact.colorIndex]
					contactId := int(arrayGet(color.contacts, int(contact.localIndex)).contactId)
					assert(contactId == contactIndex, "contactId == contactIndex")
				} else {
					color := &graph.colors[contact.colorIndex]
					contactId := int(*arrayGet(color.convexContacts, int(contact.localIndex)))
					assert(contactId == contactIndex, "contactId == contactIndex")
				}
			} else {
				assert(contact.colorIndex == NullIndex, "contact->colorIndex == B3_NULL_INDEX")
				assert(contact.manifolds == nil, "contact->manifolds == NULL")
				assert(contact.manifoldCount == 0, "contact->manifoldCount == 0")

				index := arrayGet(set.contactIndices, int(contact.localIndex))
				assert(int(*index) == contactIndex, "*index == contactIndex")
			}
		} else if setId >= firstSleepingSet {
			// Only touching contacts allowed in a sleeping set
			assert(touching == true, "touching == true")
			assert(contact.manifolds != nil, "contact->manifolds != NULL")
			assert(contact.manifoldCount > 0, "contact->manifoldCount > 0")
			index := arrayGet(set.contactIndices, int(contact.localIndex))
			assert(int(*index) == contactIndex, "*index == contactIndex")
		} else {
			// Sleeping and non-touching contacts belong in the disabled set
			assert(touching == false && setId == disabledSet, "touching == false && setId == b3_disabledSet")
			assert(contact.manifolds == nil, "contact->manifolds == NULL")
			assert(contact.manifoldCount == 0, "contact->manifoldCount == 0")
			index := arrayGet(set.contactIndices, int(contact.localIndex))
			assert(int(*index) == contactIndex, "*index == contactIndex")
		}

		if contact.flags&simMeshContact != 0 {
			cacheCount := len(contact.meshContact.triangleCache)
			if cacheCount > 0 {
				assert(contact.meshContact.triangleCache != nil, "contact->meshContact.triangleCache.data != NULL")
				assert(cap(contact.meshContact.triangleCache) >= cacheCount,
					"contact->meshContact.triangleCache.capacity >= cacheCount")

				shapeA := arrayGet(world.shapes, int(contact.shapeIdA))
				if shapeA.typ == MeshShape {
					triangleCount := int(shapeA.mesh.Data.TriangleCount)
					for i := 0; i < cacheCount; i++ {
						triangleIndex := int(contact.meshContact.triangleCache[i].triangleIndex)
						assert(0 <= triangleIndex && triangleIndex < triangleCount, "0 <= triangleIndex && triangleIndex < triangleCount")
					}
				} else if shapeA.typ == HeightShape {
					triangleCount := getHeightFieldTriangleCount(shapeA.heightField)
					for i := 0; i < cacheCount; i++ {
						triangleIndex := int(contact.meshContact.triangleCache[i].triangleIndex)
						assert(0 <= triangleIndex && triangleIndex < triangleCount, "0 <= triangleIndex && triangleIndex < triangleCount")
					}
				} else {
					assert(shapeA.typ == CompoundShape, "shapeA->type == b3_compoundShape")
					child := GetCompoundChild(shapeA.compound, int(contact.childIndex))
					assert(child.Type == MeshShape, "child.type == b3_meshShape")

					triangleCount := int(child.Mesh.Data.TriangleCount)
					for i := 0; i < cacheCount; i++ {
						triangleIndex := int(contact.meshContact.triangleCache[i].triangleIndex)
						assert(0 <= triangleIndex && triangleIndex < triangleCount, "0 <= triangleIndex && triangleIndex < triangleCount")
					}
				}
			}
		}
	}

	contactIdCount := getIdCount(&world.contactIdPool)
	assert(allocatedContactCount == contactIdCount, "allocatedContactCount == contactIdCount")
}
