// Port of include/box3d/types.h and src/types.c from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

import "math"

// PORT: translation of the public structs.
//   - `void*` becomes `any`, `const char*` becomes `string`.
//   - A pointer plus count pair becomes a slice plus the count field, both kept so call sites stay
//     line for line. The count is authoritative, as upstream.
//   - Stored `int` fields are int32. Callback parameters typed `int` are Go `int`.
//   - Function pointer typedefs are func types; a nil func is a null callback.
//   - internalValue is unexported so only the DefaultXDef functions can set the cookie.

// ---- types.h ----

const (
	DefaultCategoryBits uint64 = math.MaxUint64 // B3_DEFAULT_CATEGORY_BITS
	DefaultMaskBits     uint64 = math.MaxUint64 // B3_DEFAULT_MASK_BITS
)

// TaskCallback is the task interface.
// This is the prototype for a Box3D task. Your task system is expected to run this callback on a worker thread,
// exactly once per enqueue, passing back the same taskContext pointer supplied to EnqueueTaskCallback.
type TaskCallback func(taskContext any)

// EnqueueTaskCallback can be provided to Box3D to invoke a task system.
// Returns a pointer to the user's task object. May be nil. A nil indicates to Box3D that the work was executed
// serially within the callback and there is no need to call FinishTaskCallback. Otherwise the returned
// value must be non-nil will be passed to FinishTaskCallback as the userTask.
//
//	task the Box3D task to be called by the scheduler
//	taskContext the Box3D context object that the scheduler must pass to the task
//	userContext the scheduler context object that is opaque to Box3D
//	taskName the Box3D task name that the scheduler can use for diagnostics
type EnqueueTaskCallback func(task TaskCallback, taskContext any, userContext any, taskName string) any

// FinishTaskCallback finishes a user task object that wraps a Box3D task. This must block until the task has completed.
// The step blocks here on the tasks it spawned, so World.Step holds its stack across every
// fork/join. Drive it from a thread you can dedicate to the step, or from a fiber this callback can
// park to free the underlying thread. In a job system that cannot park a job's stack, do not call
// World.Step from inside a job: a job that blocks on its own sub-jobs without yielding its thread
// can deadlock. The in-tree scheduler instead runs other pending tasks on the waiting thread.
type FinishTaskCallback func(userTask any, userContext any)

// CreateDebugShapeCallback: the user needs to be able to create debug draw shapes for multi-pass rendering to work efficiently.
// These user shapes are created and destroyed via callback so they can be bound to shape lifetime and scaling updates.
type CreateDebugShapeCallback func(debugShape *DebugShape, userContext any) any

// DestroyDebugShapeCallback destroys a user debug shape.
type DestroyDebugShapeCallback func(userShape any, userContext any)

// FrictionCallback is the optional friction mixing callback. This intentionally provides no context objects because this is called
// from a worker thread.
// Warning: This function should not attempt to modify Box3D state or user application state.
type FrictionCallback func(frictionA float32, userMaterialIdA uint64, frictionB float32, userMaterialIdB uint64) float32

// RestitutionCallback is the optional restitution mixing callback. This intentionally provides no context objects because this is called
// from a worker thread.
// Warning: This function should not attempt to modify Box3D state or user application state.
type RestitutionCallback func(restitutionA float32, userMaterialIdA uint64, restitutionB float32, userMaterialIdB uint64) float32

// CustomFilterFcn is the prototype for a contact filter callback.
// This is called when a contact pair is considered for collision. This allows you to
// perform custom logic to prevent collision between shapes. This is only called if
// one of the two shapes has custom filtering enabled. See ShapeDef.
// Notes:
//   - this function must be thread-safe
//   - this is only called if one of the two shapes has enabled custom filtering
//   - this is called only for awake dynamic bodies
//
// Return false if you want to disable the collision
// Warning: Do not attempt to modify the world inside this callback
type CustomFilterFcn func(shapeIdA, shapeIdB ShapeID, context any) bool

// PreSolveFcn is the prototype for a pre-solve callback.
// This is called after a contact is updated. This allows you to inspect a
// collision before it goes to the solver.
// Notes:
//   - this function must be thread-safe
//   - this is only called if the shape has enabled pre-solve events
//   - this may be called for awake dynamic bodies and sensors
//   - this is not called for sensors
//
// Return false if you want to disable the contact this step
// This has limited information because it is used during CCD which does not have the
// full contact manifold.
// Warning: Do not attempt to modify the world inside this callback
type PreSolveFcn func(shapeIdA, shapeIdB ShapeID, point Pos, normal Vec3, context any) bool

// OverlapResultFcn is the prototype callback for overlap queries.
// Called for each shape found in the query.
// See World.OverlapAABB
// Return false to terminate the query.
type OverlapResultFcn func(shapeId ShapeID, context any) bool

// CastResultFcn is the prototype callback for ray casts.
// Called for each shape found in the query. You control how the ray cast
// proceeds by returning a float:
// return -1: ignore this shape and continue
// return 0: terminate the ray cast
// return fraction: clip the ray to this point
// return 1: don't clip the ray and continue
//
//	shapeId the shape hit by the ray
//	point the point of initial intersection
//	normal the normal vector at the point of intersection
//	fraction the fraction along the ray at the point of intersection
//	userMaterialId the shape or triangle surface type
//	triangleIndex the triangle index for mesh or height field shapes or -1 for other shape types
//	childIndex the child shape index for compound shapes
//	context the user context
//
// Return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
// See World.CastRay
type CastResultFcn func(shapeId ShapeID, point Pos, normal Vec3, fraction float32, userMaterialId uint64,
	triangleIndex int, childIndex int, context any) float32

// Capacity holds optional world capacities that can be use to avoid run-time allocations
type Capacity struct {
	// Number of expected static shapes.
	StaticShapeCount int32

	// Number of expected dynamic and kinematic shapes.
	DynamicShapeCount int32

	// Number of expected static bodies.
	StaticBodyCount int32

	// Number of expected dynamic and kinematic bodies.
	DynamicBodyCount int32

	// Number of expected contacts.
	ContactCount int32
}

// WorldDef is the world definition used to create a simulation world. Must be initialized using DefaultWorldDef.
type WorldDef struct {
	// Gravity vector. Box3D has no up-vector defined.
	Gravity Vec3

	// Restitution speed threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	RestitutionThreshold float32

	// Number of iterations of the restitution solver. More iterations can lead to less box spinning.
	// See MaxRestitutionIterations
	RestitutionIterations int32

	// Enable full contact propagation in the restitution solver. Expensive.
	EnableRestitutionPropagation bool

	// Hit event speed threshold, usually in m/s. Collisions above this
	// speed can generate hit events if the shape also enables hit events.
	HitEventThreshold float32

	// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	ContactHertz float32

	// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	// the trade-off that overlap resolution becomes more energetic.
	ContactDampingRatio float32

	// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	// decreasing the damping ratio.
	ContactSpeed float32

	// Maximum linear speed. Usually meters per second.
	MaximumLinearSpeed float32

	// Use this to disable gyroscopic torque on sphere-like objects automatically.
	// Range is [0, 1]. A sphere is 0 and a 1x1x2 box is 0.5.
	IsotropicInertiaTolerance float32

	// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	FrictionCallback FrictionCallback

	// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	RestitutionCallback RestitutionCallback

	// Can bodies go to sleep to improve performance
	EnableSleep bool

	// Enable continuous collision
	EnableContinuous bool

	// Number of workers to use with the provided task system. Box3D performs best when using only
	// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	// little benefit and may even harm performance.
	// This is clamped to the range [1, MaxWorkers]. Using a value above 1 will turn on multithreading.
	// If task callbacks are provided then Box3D will use the user provided task system. Otherwise Box3D
	// will create threads and use an internal scheduler.
	WorkerCount uint32

	// function to spawn task
	EnqueueTask EnqueueTaskCallback

	// function to finish a task
	FinishTask FinishTaskCallback

	// User context that is provided to enqueueTask and finishTask
	UserTaskContext any

	// User data associated with a world
	UserData any

	// Used to create debug draw shapes. This is called when a shape is
	// first drawn using DebugDraw.
	CreateDebugShape CreateDebugShapeCallback

	// Used to destroy debug draw shapes. This is called when a shape is modified or destroyed.
	DestroyDebugShape DestroyDebugShapeCallback

	// This is passed to the debug shape callbacks to provide a user context.
	UserDebugShapeContext any

	// Optional initial capacities
	Capacity Capacity

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int32
}

// DefaultWorldDef: use this to initialize your world definition. Implemented below with types.c.

// BodyType is the body simulation type.
// Each body is one of these three types. The type determines how the body behaves in the simulation.
type BodyType int32

const (
	// zero mass, zero velocity, may be manually moved
	StaticBody BodyType = 0

	// zero mass, velocity set by user, moved by solver
	KinematicBody BodyType = 1

	// positive mass, velocity determined by forces, moved by solver
	DynamicBody BodyType = 2

	// number of body types
	BodyTypeCount BodyType = 3
)

// MotionLocks restrict the body movement
type MotionLocks struct {
	// Prevent translation along the x-axis
	LinearX bool

	// Prevent translation along the y-axis
	LinearY bool

	// Prevent translation along the z-axis
	LinearZ bool

	// Prevent rotation around the x-axis
	AngularX bool

	// Prevent rotation around the y-axis
	AngularY bool

	// Prevent rotation around the z-axis
	AngularZ bool
}

// BodyDef is a body definition that holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
// Body definitions are temporary objects used to bundle creation parameters.
// Must be initialized using DefaultBodyDef().
type BodyDef struct {
	// The body type: static, kinematic, or dynamic.
	Type BodyType

	// The initial world position of the body. Bodies should be created with the desired position.
	// Note: Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	// if the body is moved after shapes have been added.
	Position Pos

	// The initial world rotation of the body.
	Rotation Quat

	// The initial linear velocity of the body's origin. Usually in meters per second.
	LinearVelocity Vec3

	// The initial angular velocity of the body. Radians per second.
	AngularVelocity Vec3

	// Linear damping is used to reduce the linear velocity. The damping parameter
	// can be larger than 1 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Generally linear damping is undesirable because it makes objects move slowly
	// as if they are floating.
	LinearDamping float32

	// Angular damping is used to reduce the angular velocity. The damping parameter
	// can be larger than 1.0f but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Angular damping can be used to slow down rotating bodies.
	AngularDamping float32

	// Scale the gravity applied to this body. Non-dimensional.
	GravityScale float32

	// Sleep speed threshold, default is 0.05 meters per second
	SleepThreshold float32

	// Continuous collision safety factor. The solver only uses continuous collision if there is a
	// risk of tunneling. If the body is moving fast enough to risk tunneling then it is considered a "fast body".
	// This improves performance and prevents movement hitches. If a body moving N meter risks tunneling, then the
	// body will be considered fast if it moves more than a safetyFactor times N meters over one full time step.
	// Non-dimensional. Recommended range [0.01, 0.5]. Default is 0.5 for high performance with low tunneling risk.
	SafetyFactor float32

	// Optional body name for debugging.
	Name string

	// Use this to store application specific body data.
	UserData any

	// Motions locks to restrict linear and angular movement
	MotionLocks MotionLocks

	// Set this flag to false if this body should never fall asleep.
	EnableSleep bool

	// Is this body initially awake or sleeping?
	IsAwake bool

	// Treat this body as a high speed object that performs continuous collision detection
	// against dynamic and kinematic bodies, but not other bullet bodies.
	// Warning: Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	// continuous collision. They do not guarantee accurate collision if both bodies are fast moving because
	// the bullet does a continuous check after all non-bullet bodies have moved. You could get unlucky and have
	// the bullet body end a time step very close to a non-bullet body and the non-bullet body then moves over
	// the bullet body. In continuous collision, initial overlap is ignored to avoid freezing bodies in place.
	// I do not recommend using them for game projectiles if precise collision timing is needed. Instead consider
	// using a ray or shape cast. You can use a marching ray or shape cast for projectile that moves over time.
	// If you want a fast moving projectile to collide with a fast moving target, you need to consider the relative
	// movement in your ray or shape cast. This is out of the scope of Box3D.
	// So what are good use cases for bullets? Pinball games or games with dynamic containers that hold other objects.
	// It should be a use case where it doesn't break the game if there is a collision missed, but having them
	// captured improves the quality of the game.
	IsBullet bool

	// Used to disable a body. A disabled body does not move or collide.
	IsEnabled bool

	// This allows this body to bypass rotational speed limits. Should only be used
	// for circular objects, like wheels.
	AllowFastRotation bool

	// Enable contact recycling. True by default. Leaving this enabled improves performance
	// but may lead to ghost collision that should be avoided on characters.
	EnableContactRecycling bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int32
}

// DefaultBodyDef: use this to initialize your body definition. Implemented below with types.c.

// Filter is used to filter collision on shapes. It affects shape-vs-shape collision
// and shape-versus-query collision (such as World.CastRay).
type Filter struct {
	// The collision category bits. Normally you would just set one bit. The category bits should
	// represent your application object types. For example:
	//
	//	const (
	//	   Static  = 0x00000001
	//	   Dynamic = 0x00000002
	//	   Debris  = 0x00000004
	//	   Player  = 0x00000008
	//	   // etc
	//	)
	CategoryBits uint64

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	// For example, you may want your player to only collide with static objects
	// and other players.
	//
	//	maskBits = Static | Player
	MaskBits uint64

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	// always wins against the mask bits.
	// For example, you may want ragdolls to collide with other ragdolls but you don't want
	// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	// and apply that group index to all shapes on the ragdoll.
	GroupIndex int32
}

// DefaultFilter: use this to initialize your filter. Implemented below with types.c.

// SurfaceMaterial holds material properties supported per triangle on meshes and height fields
type SurfaceMaterial struct {
	// The Coulomb (dry) friction coefficient, usually in the range [0,1].
	Friction float32

	// The coefficient of restitution (bounce) usually in the range [0,1].
	// https://en.wikipedia.org/wiki/Coefficient_of_restitution
	Restitution float32

	// The rolling resistance usually in the range [0,1]. This is only used for spheres and capsules.
	RollingResistance float32

	// The tangent velocity for conveyor belts. This is local to the shape and will be projected
	// onto the contact surface.
	TangentVelocity Vec3

	// User material identifier. This is passed with query results and to friction and restitution
	// combining functions. It is not used internally.
	UserMaterialId uint64

	// Custom debug draw color. Ignored if 0. The low 24 bits are RGB. The high byte may
	// carry a DebugMaterial preset, see MakeDebugColor.
	// See HexColor
	CustomColor uint32

	// Explicit padding. Must be zero.
	Padding uint32
}

// DefaultSurfaceMaterial: use this to initialize your surface material. Implemented below with types.c.

// ShapeType is the shape type
type ShapeType int32

const (
	// A capsule is an extruded sphere
	CapsuleShape ShapeType = iota

	// A baked compound shape composed of spheres, capsules, hulls, and meshes
	CompoundShape

	// A height field useful for terrain
	HeightShape

	// A convex hull
	HullShape

	// A triangle soup
	MeshShape

	// A sphere with an offset
	SphereShape

	// The number of shape types
	ShapeTypeCount
)

// ShapeDef is used to create a shape
type ShapeDef struct {
	// Optional shape name for debugging
	Name string

	// Use this to store application specific shape data.
	UserData any

	// Surface material used on mesh shapes per triangle. Ignored for convex shapes. Ignored for compound shapes.
	Materials []SurfaceMaterial

	// Surface material count.
	MaterialCount int32

	// The base surface material. Ignored for compound shapes.
	BaseMaterial SurfaceMaterial

	// The density, usually in kg/m^3.
	Density float32

	// Explosion scale for World.Explode. non-dimensional
	ExplosionScale float32

	// Contact filtering data.
	Filter Filter

	// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See WorldDef.
	EnableCustomFiltering bool

	// A sensor shape generates overlap events but never generates a collision response.
	// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	// Sensors still contribute to the body mass if they have non-zero density.
	// Note: Sensor events are disabled by default.
	// See EnableSensorEvents
	IsSensor bool

	// Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.
	// Only convex shapes may act as sensor visitors.
	EnableSensorEvents bool

	// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	EnableContactEvents bool

	// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	EnableHitEvents bool

	// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	// and must be carefully handled due to multithreading. Ignored for sensors.
	EnablePreSolveEvents bool

	// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	// static body creation when there are many static shapes.
	// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	InvokeContactCreation bool

	// Should the body update the mass properties when this shape is created. Default is true.
	// Warning: if this is false, you MUST call Body.ApplyMassFromShapes or Body.SetMassData before simulating the world.
	UpdateBodyMass bool

	// Enable speculative collision. Leave this true unless you care about reducing ghost collision
	// more than continuous collision under rotation.
	// Experimental: this can only disable speculative contact between hulls and triangles (meshes and height fields).
	EnableSpeculativeContact bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int32
}

// DefaultShapeDef: use this to initialize your shape definition. Implemented below with types.c.

// Profile holds profiling data. Times are in milliseconds.
type Profile struct {
	Step                float32
	Pairs               float32
	Collide             float32
	Solve               float32
	SolverSetup         float32
	Constraints         float32
	PrepareConstraints  float32
	IntegrateVelocities float32
	WarmStart           float32
	SolveImpulses       float32
	IntegratePositions  float32
	RelaxImpulses       float32
	Restitution         float32
	StoreImpulses       float32
	SplitIslands        float32
	Transforms          float32
	SensorHits          float32
	JointEvents         float32
	HitEvents           float32
	Refit               float32
	Bullets             float32
	SleepIslands        float32
	Sensors             float32
}

// Counters give details of the simulation size.
type Counters struct {
	ByteCount        int64
	BodyCount        int32
	ShapeCount       int32
	ContactCount     int32
	JointCount       int32
	IslandCount      int32
	StackUsed        int32
	ArenaCapacity    int32
	StaticTreeHeight int32
	TreeHeight       int32
	SatCallCount     int32
	SatCacheHitCount int32
	TaskCount        int32
	ColorCounts      [24]int32
	ManifoldCounts   [ContactManifoldCountBuckets]int32

	// Number of contacts touched by the collide pass
	// graph contacts + awake-set non-touching
	AwakeContactCount int32

	// Number of contacts recycled in the most recent step.
	RecycledContactCount int32

	// Maximum number of time of impact outer iterations
	DistanceIterations int32

	// Maximum TOI push backs.
	PushBackIterations int32

	// Maximum TOI root solver iterations.
	RootIterations int32
}

// JointType enumeration. This is useful because all joint types use JointID and sometimes you
// want to get the type of a joint.
type JointType int32

const (
	ParallelJoint JointType = iota
	DistanceJoint
	FilterJoint
	MotorJoint
	PrismaticJoint
	RevoluteJoint
	SphericalJoint
	WeldJoint
	WheelJoint
)

// JointDef is the base joint definition used by all joint types. The local frames are measured from the
// body's origin rather than the center of mass because:
// 1. You might not know where the center of mass will be.
// 2. If you add/remove shapes from a body and recompute the mass, the joints will be broken.
type JointDef struct {
	// User data pointer
	UserData any

	// The first attached body
	BodyIdA BodyID

	// The second attached body
	BodyIdB BodyID

	// The first local joint frame
	LocalFrameA Transform

	// The second local joint frame
	LocalFrameB Transform

	// Force threshold for joint events
	ForceThreshold float32

	// Torque threshold for joint events
	TorqueThreshold float32

	// Constraint hertz (advanced feature)
	ConstraintHertz float32

	// Constraint damping ratio (advanced feature)
	ConstraintDampingRatio float32

	// Debug draw scale
	DrawScale float32

	// Set this flag to true if the attached bodies should collide
	CollideConnected bool

	// Used internally to detect a valid definition. DO NOT SET.
	internalValue int32
}

// DistanceJointDef is the distance joint definition.
// Connects a point on body A with a point on body B by a segment.
// Useful for ropes and springs.
type DistanceJointDef struct {
	// Base joint definition
	Base JointDef

	// The rest length of this joint. Clamped to a stable minimum value.
	Length float32

	// Enable the distance constraint to behave like a spring. If false
	// then the distance joint will be rigid, overriding the limit and motor.
	EnableSpring bool

	// The lower spring force controls how much tension it can sustain
	LowerSpringForce float32

	// The upper spring force controls how much compression it can sustain
	UpperSpringForce float32

	// The spring linear stiffness Hertz, cycles per second
	Hertz float32

	// The spring linear damping ratio, non-dimensional
	DampingRatio float32

	// Enable/disable the joint limit
	EnableLimit bool

	// Minimum length. Clamped to a stable minimum value.
	MinLength float32

	// Maximum length. Must be greater than or equal to the minimum length.
	MaxLength float32

	// Enable/disable the joint motor
	EnableMotor bool

	// The maximum motor force, usually in newtons
	MaxMotorForce float32

	// The desired motor speed, usually in meters per second
	MotorSpeed float32
}

// MotorJointDef: a motor joint is used to control the relative position and velocity between two bodies.
type MotorJointDef struct {
	// Base joint definition
	Base JointDef

	// The desired linear velocity
	LinearVelocity Vec3

	// The maximum motor force in newtons
	MaxVelocityForce float32

	// The desired angular velocity
	AngularVelocity Vec3

	// The maximum motor torque in newton-meters
	MaxVelocityTorque float32

	// Linear spring hertz for position control
	LinearHertz float32

	// Linear spring damping ratio
	LinearDampingRatio float32

	// Maximum spring force in newtons
	MaxSpringForce float32

	// Angular spring hertz for position control
	AngularHertz float32

	// Angular spring damping ratio
	AngularDampingRatio float32

	// Maximum spring torque in newton-meters
	MaxSpringTorque float32
}

// FilterJointDef: a filter joint is used to disable collision between two specific bodies.
type FilterJointDef struct {
	// Base joint definition
	Base JointDef
}

// ParallelJointDef is the parallel joint definition. Constrains the angle between axis z in body A and axis z in body B
// using a spring. Useful to keep a body upright.
type ParallelJointDef struct {
	// Base joint definition
	Base JointDef

	// The spring stiffness Hertz, cycles per second
	Hertz float32

	// The spring damping ratio, non-dimensional
	DampingRatio float32

	// The maximum spring torque, typically in newton-meters.
	MaxTorque float32
}

// PrismaticJointDef is the prismatic joint definition. Body B may slide along the x-axis in local frame A.
// Body B cannot rotate relative to body A. The joint translation is zero when the
// local frame origins coincide in world space.
type PrismaticJointDef struct {
	// Base joint definition
	Base JointDef

	// Enable a linear spring along the prismatic joint axis
	EnableSpring bool

	// The spring stiffness Hertz, cycles per second
	Hertz float32

	// The spring damping ratio, non-dimensional
	DampingRatio float32

	// The target translation for the joint in meters. The spring-damper will drive
	// to this translation.
	TargetTranslation float32

	// Enable/disable the joint limit
	EnableLimit bool

	// The lower translation limit
	LowerTranslation float32

	// The upper translation limit
	UpperTranslation float32

	// Enable/disable the joint motor
	EnableMotor bool

	// The maximum motor force, typically in newtons
	MaxMotorForce float32

	// The desired motor speed, typically in meters per second
	MotorSpeed float32
}

// RevoluteJointDef is the revolute joint definition. A point on body B is fixed to a point on body A.
// Allows relative rotation about the z-axis.
type RevoluteJointDef struct {
	// Base joint definition.
	Base JointDef

	// The bodyB angle minus bodyA angle in the reference state (radians).
	// This defines the zero angle for the joint limit.
	TargetAngle float32

	// Enable a rotational spring on the revolute hinge axis.
	EnableSpring bool

	// The spring stiffness Hertz, cycles per second.
	Hertz float32

	// The spring damping ratio, non-dimensional.
	DampingRatio float32

	// A flag to enable joint limits.
	EnableLimit bool

	// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
	LowerAngle float32

	// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
	UpperAngle float32

	// A flag to enable the joint motor.
	EnableMotor bool

	// The maximum motor torque, typically in newton-meters.
	MaxMotorTorque float32

	// The desired motor speed in radians per second.
	MotorSpeed float32
}

// SphericalJointDef is the spherical joint definition. A point on body B is fixed to a point on body A.
// Allows rotation about the shared point.
type SphericalJointDef struct {
	// Base joint definition
	Base JointDef

	// Enable a rotational spring that attempts to align the two joint frames.
	EnableSpring bool

	// The spring stiffness Hertz, cycles per second. This may be clamped internally
	// according to the time step to maintain stability. Non-negative number.
	Hertz float32

	// The spring damping ratio, non-dimensional. Non-negative number.
	DampingRatio float32

	// Target spring rotation, joint frame B relative to joint frame A.
	TargetRotation Quat

	// A flag to enable the cone limit. The cone is centered on the frameA z-axis.
	EnableConeLimit bool

	// The angle for the cone limit in radians. Valid range is [0, pi]
	ConeAngle float32

	// A flag to enable the twist limit. The twist is centered on the frameB z-axis.
	EnableTwistLimit bool

	// The angle for the lower twist limit in radians. Minimum of -0.99*pi radians.
	LowerTwistAngle float32

	// The angle for the upper twist limit in radians. Maximum of 0.99*pi radians.
	UpperTwistAngle float32

	// A flag to enable the joint motor
	EnableMotor bool

	// The maximum motor torque, typically in newton-meters. Non-negative number.
	MaxMotorTorque float32

	// The desired motor angular velocity in radians per second. This is the relative angular
	// velocity between the two bodies in world space.
	// motorVelocity = angularVelocityB - angularVelocityA
	MotorVelocity Vec3
}

// WeldJointDef is the weld joint definition
// Connects two bodies together rigidly. This constraint provides springs to mimic
// soft-body simulation.
// Note: The approximate solver in Box3D cannot hold many bodies together rigidly
type WeldJointDef struct {
	// Base joint definition
	Base JointDef

	// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	LinearHertz float32

	// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	AngularHertz float32

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	LinearDampingRatio float32

	// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	AngularDampingRatio float32
}

// WheelJointDef is the wheel joint definition
// Body A is the chassis and body B is the wheel.
// The wheel rotates around the local z-axis in frame B.
// The wheel translates along the local x-axis in frame A.
// The wheel can optionally steer along the x-axis in frame A.
type WheelJointDef struct {
	// Base joint definition
	Base JointDef

	// Enable a linear spring along the local axis
	EnableSuspensionSpring bool

	// Spring stiffness in Hertz
	SuspensionHertz float32

	// Spring damping ratio, non-dimensional
	SuspensionDampingRatio float32

	// Enable/disable the joint linear limit
	EnableSuspensionLimit bool

	// The lower suspension translation limit
	LowerSuspensionLimit float32

	// The upper translation limit
	UpperSuspensionLimit float32

	// Enable/disable the joint rotational motor
	EnableSpinMotor bool

	// The maximum motor torque, typically in newton-meters
	MaxSpinTorque float32

	// The desired motor speed in radians per second
	SpinSpeed float32

	// Enable steering, otherwise the steering is fixed forward
	EnableSteering bool

	// Steering stiffness in Hertz
	SteeringHertz float32

	// Spring damping ratio, non-dimensional
	SteeringDampingRatio float32

	// The target steering angle in radians
	TargetSteeringAngle float32

	// The maximum steering torque in N*m
	MaxSteeringTorque float32

	// Enable/disable the steering angular limit
	EnableSteeringLimit bool

	// The lower steering angle in radians
	LowerSteeringLimit float32

	// The upper steering angle in radians
	UpperSteeringLimit float32
}

// ExplosionDef is used to configure options for explosions. Explosions
// consider shape geometry when computing the impulse.
type ExplosionDef struct {
	// Mask bits to filter shapes
	MaskBits uint64

	// The center of the explosion in world space
	Position Pos

	// The radius of the explosion
	Radius float32

	// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	Falloff float32

	// Impulse per unit area. This applies an impulse according to the shape area that
	// is facing the explosion. Explosions only apply to spheres, capsules, and hulls. This
	// may be negative for implosions.
	ImpulsePerArea float32
}

// The DefaultXJointDef and DefaultExplosionDef functions are implemented in joint.go.

// Events
//
// Events are used to collect events that occur during the world time step. These events
// are then available to query after the time step is complete. This is preferable to callbacks
// because Box3D uses multithreaded simulation.
//
// Also when events occur in the simulation step it may be problematic to modify the world, which is
// often what applications want to do when events occur.
//
// With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
// that some event data may become invalid. There are several samples that show how to do this safely.

// SensorBeginTouchEvent is generated when a shape starts to overlap a sensor shape.
type SensorBeginTouchEvent struct {
	// The id of the sensor shape
	SensorShapeId ShapeID

	// The id of the shape that began touching the sensor shape
	VisitorShapeId ShapeID
}

// SensorEndTouchEvent is generated when a shape stops overlapping a sensor shape.
// These include things like setting the transform, destroying a body or shape, or changing
// a filter. You will also get an end event if the sensor or visitor are destroyed.
// Therefore you should always confirm the shape id is valid using Shape.IsValid.
type SensorEndTouchEvent struct {
	// The id of the sensor shape
	// Warning: this shape may have been destroyed
	SensorShapeId ShapeID

	// The id of the shape that stopped touching the sensor shape
	// Warning: this shape may have been destroyed
	VisitorShapeId ShapeID
}

// SensorEvents are buffered in the world and are available
// as begin/end overlap event arrays after the time step is complete.
// Note: these may become invalid if bodies and/or shapes are destroyed
type SensorEvents struct {
	// Array of sensor begin touch events
	BeginEvents []SensorBeginTouchEvent

	// Array of sensor end touch events
	EndEvents []SensorEndTouchEvent

	// The number of begin touch events
	BeginCount int32

	// The number of end touch events
	EndCount int32
}

// ContactBeginTouchEvent is generated when two shapes begin touching.
type ContactBeginTouchEvent struct {
	// Id of the first shape
	ShapeIdA ShapeID

	// Id of the second shape
	ShapeIdB ShapeID

	// The transient contact id. This contact may be destroyed automatically when the world is modified or simulated.
	// Use Contact.IsValid before using this id.
	ContactId ContactID
}

// ContactEndTouchEvent is generated when two shapes stop touching.
// You will get an end event if you do anything that destroys contacts previous to the last
// world step. These include things like setting the transform, destroying a body
// or shape, or changing a filter or body type.
type ContactEndTouchEvent struct {
	// Id of the first shape
	// Warning: this shape may have been destroyed
	ShapeIdA ShapeID

	// Id of the first shape
	// Warning: this shape may have been destroyed
	ShapeIdB ShapeID

	// Id of the contact.
	// Warning: this contact may have been destroyed
	ContactId ContactID
}

// ContactHitEvent is generated when two shapes collide with a speed faster than the hit speed threshold.
// This may be reported for speculative contacts that have a confirmed impulse.
type ContactHitEvent struct {
	// Id of the first shape
	ShapeIdA ShapeID

	// Id of the second shape
	ShapeIdB ShapeID

	// Id of the contact.
	// Warning: this contact may have been destroyed
	ContactId ContactID

	// Point where the shapes hit at the beginning of the time step.
	// This is a mid-point between the two surfaces. It could be at speculative
	// point where the two shapes were not touching at the beginning of the time step.
	Point Pos

	// Normal vector pointing from shape A to shape B
	Normal Vec3

	// The speed the shapes are approaching. Always positive. Typically in meters per second.
	ApproachSpeed float32

	// User material on shape A
	UserMaterialIdA uint64

	// User material on shape B
	UserMaterialIdB uint64
}

// ContactEvents are buffered in the world and are available
// as event arrays after the time step is complete.
// Note: these may become invalid if bodies and/or shapes are destroyed
type ContactEvents struct {
	// Array of begin touch events
	BeginEvents []ContactBeginTouchEvent

	// Array of end touch events
	EndEvents []ContactEndTouchEvent

	// Array of hit events
	HitEvents []ContactHitEvent

	// Number of begin touch events
	BeginCount int32

	// Number of end touch events
	EndCount int32

	// Number of hit events
	HitCount int32
}

// BodyMoveEvent: body move events triggered when a body moves.
// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
// This also has a flag to indicate that the body went to sleep so the application can also
// sleep that actor/entity/object associated with the body.
// On the other hand if the flag does not indicate the body went to sleep then the application
// can treat the actor/entity/object associated with the body as awake.
// This is an efficient way for an application to update game object transforms rather than
// calling functions such as Body.GetTransform() because this data is delivered as a contiguous array
// and it is only populated with bodies that have moved.
// Note: If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
type BodyMoveEvent struct {
	// The body user data.
	UserData any

	// The body transform.
	Transform WorldTransform

	// The body id.
	BodyId BodyID

	// Did the body fall asleep this time step?
	FellAsleep bool
}

// BodyEvents are buffered in the world and are available
// as event arrays after the time step is complete.
// Note: this data becomes invalid if bodies are destroyed
type BodyEvents struct {
	// Array of move events
	MoveEvents []BodyMoveEvent

	// Number of move events
	MoveCount int32
}

// JointEvent: joint events report joints that are awake and have a force and/or torque exceeding the threshold
// The observed forces and torques are not returned for efficiency reasons.
type JointEvent struct {
	// The joint id
	JointId JointID

	// The user data from the joint for convenience
	UserData any
}

// JointEvents are buffered in the world and are available
// as event arrays after the time step is complete.
// Note: this data becomes invalid if joints are destroyed
type JointEvents struct {
	// Array of events
	JointEvents []JointEvent

	// Number of events
	Count int32
}

// ContactData is the contact data for two shapes. By convention the manifold normal points
// from shape A to shape B.
// See Shape.GetContactData() and Body.GetContactData()
type ContactData struct {
	// The contact id. You may hold onto this to track a contact across time steps.
	// This id may become orphaned. Use Contact.IsValid before using it for other functions.
	ContactId ContactID

	// The first shape id.
	ShapeIdA ShapeID

	// The second shape id.
	ShapeIdB ShapeID

	// The contact manifold. This points to internal data and may become invalid. Do not store
	// this slice.
	Manifolds []Manifold

	// The number of contact manifolds. For mesh and height-field collision there can be multiple manifolds.
	ManifoldCount int32
}

// Query
//
// Queries include ray casts, shapes casts, overlap, distance, and time of impact.

// QueryFilter is used to filter collisions between queries and shapes. For example,
// you may want a ray-cast representing a projectile to hit players and the static environment
// but not debris.
type QueryFilter struct {
	// The collision category bits of this query. Normally you would just set one bit.
	CategoryBits uint64

	// The collision mask bits. This states the shape categories that this
	// query would accept for collision.
	MaskBits uint64

	// Optional id combined with Name to identify this query in a recording, e.g. an entity id.
	// Need not be unique on its own. 0 with an empty name means untagged. Ignored when not recording.
	Id uint64

	// Optional label combined with Id to identify this query, e.g. "bullet". Need not be unique
	// on its own. The recorder hashes (id, name) into one stable key the viewer tracks the query by,
	// so the same id and name pair identifies the same query across frames. Empty means none. Ignored
	// when not recording.
	// PORT: an empty string stands for NULL.
	Name string

	// IgnoreShapeMask accepts shapes by (shape.CategoryBits & MaskBits) alone.
	// This separates query visibility from solid collision participation. False
	// preserves the upstream symmetric category/mask test.
	IgnoreShapeMask bool

	// BackFaces includes triangle backfaces in ray/shape casts. False preserves
	// upstream one-sided casts. OverlapShape remains an unoriented volume test.
	// Upstream record/replay cannot encode either extension flag and rejects them.
	BackFaces bool
}

// DefaultQueryFilter: use this to initialize your query filter. Implemented below with types.c.

// RayCastInput is the low level ray cast input data.
type RayCastInput struct {
	// Start point of the ray cast.
	Origin Vec3

	// Translation of the ray cast.
	// end = start + translation.
	Translation Vec3

	// The maximum fraction of the translation to consider, typically 1
	MaxFraction float32
}

// RayResult is the result from World.RayCastClosest.
type RayResult struct {
	// The shape hit.
	ShapeId ShapeID

	// The world point of the hit.
	Point Pos

	// The world normal of the shape surface at the hit point.
	Normal Vec3

	// The user material id at the hit point. This can be per triangle
	// if the shape is a mesh, height-field, or compound with child mesh.
	UserMaterialId uint64

	// The fraction of the input ray.
	Fraction float32

	// The triangle index if the shape is a mesh, height-field, or compound with
	// child mesh.
	TriangleIndex int32

	// The child index if the shape is a compound.
	ChildIndex int32

	// The number of BVH nodes visited. Diagnostic.
	NodeVisits int32

	// The number of BVH leaves visited. Diagnostic.
	LeafVisits int32

	// Did the ray hit? If false, all other data is invalid.
	Hit bool
}

// ShapeProxy is used by the GJK algorithm. It can represent a convex shape.
type ShapeProxy struct {
	// The point cloud.
	Points []Vec3

	// The number of points. Do not exceed MaxShapeCastPoints.
	Count int32

	// The external radius of the point cloud.
	Radius float32
}

// ShapeCastInput is the low level shape cast input in generic form. This allows casting an arbitrary point
// cloud wrap with a radius. For example, a sphere is a single point with a non-zero radius.
// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
type ShapeCastInput struct {
	// A generic query shape.
	Proxy ShapeProxy

	// The translation of the shape cast.
	Translation Vec3

	// The maximum fraction of the translation to consider, typically 1.
	MaxFraction float32

	// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	CanEncroach bool
}

// BoxCastInput is the input for sweeping an AABB through a dynamic tree. The box is in the tree's world float frame.
// The caller folds the cast shape radius and any world origin into the box, so the tree traversal
// stays a conservative box sweep and the precise narrow phase happens per shape in the callback.
type BoxCastInput struct {
	// The AABB to cast, in the tree's frame.
	Box AABB

	// The sweep translation.
	Translation Vec3

	// The maximum fraction of the translation to consider, typically 1.
	MaxFraction float32
}

// CastOutput is the low level ray cast or shape-cast output data.
type CastOutput struct {
	// The surface normal at the hit point.
	Normal Vec3

	// The surface hit point.
	Point Vec3

	// The fraction of the input translation at collision.
	Fraction float32

	// The number of iterations used.
	Iterations int32

	// The index of the mesh or height field triangle hit.
	TriangleIndex int32

	// The index of the compound child shape.
	ChildIndex int32

	// The material index. May be -1 for null.
	MaterialIndex int32

	// Did the cast hit?
	Hit bool
}

// WorldCastOutput is the ray cast or shape-cast output in world space. The hit point is a world position so the result
// stays precise far from the world origin. Mirrors CastOutput with a world position point.
// PORT: upstream makes this a typedef of b3CastOutput in single precision. It is a distinct type
// here, like Pos, using the large world layout.
type WorldCastOutput struct {
	// The surface normal at the hit point.
	Normal Vec3

	// The surface hit point in world space.
	Point Pos

	// The fraction of the input translation at collision.
	Fraction float32

	// The number of iterations used.
	Iterations int32

	// The index of the mesh or height field triangle hit.
	TriangleIndex int32

	// The index of the compound child shape.
	ChildIndex int32

	// The material index. May be -1 for null.
	MaterialIndex int32

	// Did the cast hit?
	Hit bool
}

// BodyCastResult is the body cast result for ray and shape casts.
type BodyCastResult struct {
	// The shape hit.
	ShapeId ShapeID

	// The world point on the shape surface.
	Point Pos

	// The world normal vector on the shape surface.
	Normal Vec3

	// The fraction along the ray hit.
	// hit point = origin + fraction * translation
	Fraction float32

	// The triangle index if the shape is a mesh or height-field.
	TriangleIndex int32

	// The user material id at the hit point. This can be per triangle
	// if the shape is a mesh, height-field, or compound with child mesh.
	UserMaterialId uint64

	// The number of iterations used. Diagnostic.
	Iterations int32

	// Did the cast hit? If false, all other fields are invalid.
	Hit bool
}

// SimplexCache is used to warm start the GJK simplex. If you call this function multiple times with nearby
// transforms this might improve performance. Otherwise you can zero initialize this.
// The distance cache must be initialized to zero on the first call.
// Users should generally just zero initialize this structure for each call.
type SimplexCache struct {
	// Value use to compare length, area, volume of two simplexes.
	Metric float32

	// todo use an index of 0xFF as a sentinel and remove the count
	// The number of stored simplex points
	Count uint16

	// The cached simplex indices on shape A
	IndexA [4]uint8

	// The cached simplex indices on shape B
	IndexB [4]uint8
}

// EmptyDistanceCache is b3_emptyDistanceCache.
var EmptyDistanceCache = SimplexCache{}

// ShapeCastPairInput holds the input parameters for ShapeCast
type ShapeCastPairInput struct {
	ProxyA       ShapeProxy // The proxy for shape A
	ProxyB       ShapeProxy // The proxy for shape B
	Transform    Transform  // Transform of shape B in shape A's frame, the relative pose B in A
	TranslationB Vec3       // The translation of shape B, in A's frame
	MaxFraction  float32    // The fraction of the translation to consider, typically 1
	CanEncroach  bool       // Allows shapes with a radius to move slightly closer if already touching
}

// DistanceInput is the input for ShapeDistance
type DistanceInput struct {
	// The proxy for shape A
	ProxyA ShapeProxy

	// The proxy for shape B
	ProxyB ShapeProxy

	// Transform of shape B in shape A's frame, the relative pose B in A
	// (InvMulWorldTransforms( worldA, worldB )). The query is origin independent and runs in frame A.
	Transform Transform

	// Should the proxy radius be considered?
	UseRadii bool
}

// DistanceOutput is the output for ShapeDistance
type DistanceOutput struct {
	PointA       Vec3    // Closest point on shapeA, in shape A's frame
	PointB       Vec3    // Closest point on shapeB, in shape A's frame
	Normal       Vec3    // A to B normal in shape A's frame. Invalid if distance is zero.
	Distance     float32 // The final distance, zero if overlapped
	Iterations   int32   // Number of GJK iterations used
	SimplexCount int32   // The number of simplexes stored in the simplex array
}

// SimplexVertex is a simplex vertex for debugging the GJK algorithm
type SimplexVertex struct {
	WA     Vec3    // support point in proxyA
	WB     Vec3    // support point in proxyB
	W      Vec3    // wB - wA
	A      float32 // barycentric coordinates
	IndexA int32   // wA index
	IndexB int32   // wB index
}

// Simplex from the GJK algorithm
type Simplex struct {
	Vertices [4]SimplexVertex // vertices
	Count    int32            // number of valid vertices
}

// Sweep describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
// position.
type Sweep struct {
	LocalCenter Vec3 // Local center of mass position
	C1          Vec3 // Starting center of mass world position
	C2          Vec3 // Ending center of mass world position
	Q1          Quat // Starting world rotation
	Q2          Quat // Ending world rotation
}

// TOIInput is the time of impact input
type TOIInput struct {
	ProxyA      ShapeProxy // The proxy for shape A
	ProxyB      ShapeProxy // The proxy for shape B
	SweepA      Sweep      // The movement of shape A
	SweepB      Sweep      // The movement of shape B
	MaxFraction float32    // Defines the sweep interval [0, tMax]
}

// TOIState describes the TOI output
type TOIState int32

const (
	TOIStateUnknown TOIState = iota
	TOIStateFailed
	TOIStateOverlapped
	TOIStateHit
	TOIStateSeparated
)

// TOIOutput is the time of impact output.
type TOIOutput struct {
	// The type of result.
	State TOIState

	// The hit point. A shared point if overlapped.
	Point Vec3

	// The hit normal. Zero if overlapped.
	Normal Vec3

	// The sweep time of the collision. 0 if overlapped.
	Fraction float32

	// The final distance. 0 if overlapped.
	Distance float32

	// Number of outer iterations.
	DistanceIterations int32

	// Total number of push back iterations.
	PushBackIterations int32

	// Total number of root iterations.
	RootIterations int32

	// Indicates that the time of impact detected initial
	// overlap and used a fallback sphere as a last ditch effort
	// to prevent tunneling.
	UsedFallback bool
}

// Dynamic Tree
//
// The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
//
// Box3D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
// This data structure may have uses in games for organizing other geometry data and may be used independently
// of Box3D rigid body simulation.
//
// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
// A dynamic tree arranges data in a binary tree to accelerate
// queries such as AABB queries and ray casts. Leaf nodes are proxies
// with an AABB. These are used to hold a user collision object.
// Nodes are pooled and relocatable, so I use node indices rather than pointers.
// The dynamic tree is made available for advanced users that would like to use it to organize
// spatial game data besides rigid bodies.

// TreeNode is a node in the dynamic tree. Siblings sit together at an even index so two nodes fit in
// a 64 byte cache line. The root is at index zero and index one always empty.
type TreeNode struct {
	// The node bounding box
	AABB AABB // 24

	// bit 31 : 1 for leaf node
	// bit 30 : 1 for moved flag
	// bits 0-29 : index of the sibling pair node or the proxy id for a leaf
	FlagIndex uint32 // 4

	// PORT: anonymous union { int32_t height; int32_t shapeIndex; }. Both members are one int32:
	// the height of an internal node (a leaf has zero height) or the shape index for a leaf
	// (truncated from proxy user data).
	Height int32 // 4
}

// TreeProxy is the separate storage for tree leaves.
type TreeProxy struct {
	// User data is an index instead of void* because it is used internally as a shape index.
	UserData uint64

	// Category bits for collision filtering.
	CategoryBits uint64

	// The leaf node. NullIndex for a free proxy.
	Node int32

	// Next free proxy.
	Next int32
}

// DynamicTreeVersion is the dynamic tree version for compatibility testing.
const DynamicTreeVersion uint64 = 0x1D6F4C2A73B80E91

// DynamicTree is the dynamic tree structure. This should be considered private data.
// It is placed here for performance reasons.
type DynamicTree struct {
	// The dynamic tree version. Always the first field. Useful
	// if the tree is serialized.
	Version uint64

	// Array of nodes. The root is at index zero and index 1 is empty.
	// Otherwise siblings are paired at even indices. Has holes for free node pairs.
	Nodes []TreeNode

	// Parent index per node. The free list is interweaved.
	Parents []int32

	// Proxy data split from node array as cold data.
	Proxies []TreeProxy

	// Every allocated node has a lower index than this.
	NodeEnd int32

	// The allocated node space
	NodeCapacity int32

	// Free pairs below nodeEnd
	PairFreeList int32

	// Number of proxies created
	ProxyCount int32

	// The allocated proxy space
	ProxyCapacity int32

	// Proxy free list
	ProxyFreeList int32

	// Array of nodes for rebuild.
	SwapNodes []TreeNode

	// Leaf indices for rebuild
	LeafIndices []int32

	// Leaves for the rebuild. May represent a proxy or a retained subtree.
	LeafNodes []TreeNode

	// Leaf bounding boxes for rebuild
	LeafBoxes []AABB

	// Leaf bounding box centers for rebuild
	LeafCenters []Vec3

	// Bins for sorting during rebuild
	BinIndices []int32

	// Allocated space for rebuilding
	RebuildCapacity int32

	// Rebuild orders the nodes so the children follow parents. Cache friendly for queries
	// and refitting. The order can be disrupted by proxy creation.
	DfsOrdered bool
}

// TreeStats are performance results returned by dynamic tree queries.
type TreeStats struct {
	// Number of internal nodes visited during the query
	NodeVisits int32

	// Number of leaf nodes visited during the query
	LeafVisits int32
}

// TreeQueryCallbackFcn receives proxies found in the AABB query.
// Return true if the query should continue
type TreeQueryCallbackFcn func(proxyId int, userData uint64, context any) bool

// TreeQueryClosestCallbackFcn receives the minimum distance squared so far and proxy to check in the closest query.
// Return minimum distance squared to user objects in the proxy
type TreeQueryClosestCallbackFcn func(distanceSqrMin float32, proxyId int, userData uint64, context any) float32

// TreeBoxCastCallbackFcn receives clipped AABB cast input for a proxy. The function returns the new cast
// fraction.
//   - return a value of 0 to terminate the cast
//   - return a value less than input.MaxFraction to clip the cast
//   - return a value of input.MaxFraction to continue the cast without clipping
type TreeBoxCastCallbackFcn func(input *BoxCastInput, proxyId int, userData uint64, context any) float32

// TreeRayCastCallbackFcn receives clipped ray cast input for a proxy. The function
// returns the new ray fraction.
//   - return a value of 0 to terminate the ray cast
//   - return a value less than input.MaxFraction to clip the ray
//   - return a value of input.MaxFraction to continue the ray cast without clipping
type TreeRayCastCallbackFcn func(input *RayCastInput, proxyId int, userData uint64, context any) float32

// Character Mover
//
// Character movement solver

// PlaneResult is the plane between a character mover and a shape
type PlaneResult struct {
	// Outward pointing plane.
	Plane Plane

	// Closest point on the shape. May not be unique.
	Point Vec3

	// The index of the mesh or height field triangle hit.
	TriangleIndex int32

	// The index of the compound child shape.
	ChildIndex int32

	// The material index.
	MaterialIndex int32
}

// CollisionPlane: these are collision planes that can be fed to SolvePlanes. Normally
// this is assembled by the user from plane results in PlaneResult.
type CollisionPlane struct {
	// The collision plane between the mover and some shape.
	Plane Plane

	// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	// make the plane collision soft. Usually in meters.
	PushLimit float32

	// The push on the mover determined by SolvePlanes. Usually in meters.
	Push float32

	// Indicates if ClipVector should clip against this plane. Should be false for soft collision.
	ClipVelocity bool
}

// PlaneSolverResult is the result returned by SolvePlanes.
type PlaneSolverResult struct {
	// The final relative translation.
	Delta Vec3

	// The number of iterations used by the plane solver. For diagnostics.
	IterationCount int32
}

// BodyPlaneResult is the body plane result for movers.
type BodyPlaneResult struct {
	// The shape id on the body.
	ShapeId ShapeID

	// The plane result.
	Result PlaneResult
}

// BodyTOIResult is the body time of impact result for movers.
type BodyTOIResult struct {
	// The hit point in world space.
	Point Pos

	// The hit normal. Points from the body to the mover.
	Normal Vec3

	// The sweep time of the collision.
	Fraction float32

	// The hit shape.
	ShapeId ShapeID
}

// PlaneResultFcn is used to collect collision planes for character movers.
// Return true to continue gathering planes.
// PORT: the plane pointer + planeCount pair is a slice.
type PlaneResultFcn func(shapeId ShapeID, planes []PlaneResult, context any) bool

// MoverFilterFcn is used to filter shapes for shape casting character movers.
// Return true to accept the collision
type MoverFilterFcn func(shapeId ShapeID, context any) bool

// Geometry
//
// Definitions of spheres, capsules, hulls, meshes, height fields, and compounds.

// MassData holds the mass data computed for a shape.
type MassData struct {
	// The shape mass
	Mass float32

	// The local center of mass position.
	Center Vec3

	// The inertia tensor about the shape center of mass.
	Inertia Matrix3
}

// Sphere is a solid sphere
type Sphere struct {
	// The local center
	Center Vec3

	// The radius
	Radius float32
}

// Capsule is a solid capsule can be viewed as two hemispheres connected by a cylinder.
type Capsule struct {
	// Local center of the first hemisphere
	Center1 Vec3

	// Local center of the second hemisphere
	Center2 Vec3

	// The radius of the hemispheres
	Radius float32
}

// PORT: the geometry blob headers below (HullData, BoxHull, MeshNode, MeshData, HeightFieldData,
// CompoundData) carry upstream's header fields. Upstream stores the arrays after the header in
// one allocation addressed by byte offsets. How the Go types own that byte image, and the
// accessors in collision.h, land with hull.c / mesh.c / height_field.c / compound.c in milestone 2.

// HullVertex is a hull vertex. Identified by a half-edge with this vertex as its tail.
type HullVertex struct {
	// A half-edge that has this vertex as the origin
	// Can be used along with edge twins and winding order
	// to traverse all the edges connected to this vertex.
	Edge uint8
}

// HullHalfEdge is the half-edge for hull data structure
type HullHalfEdge struct {
	// Next edge index CCW
	Next uint8

	// Twin edge index
	Twin uint8

	// index of origin vertex and point
	Origin uint8

	// Face to the left of this edge
	Face uint8
}

// HullFace is a hull face. Hulls use a half-edge data structure, so a face
// can be determined from a single half-edge index.
type HullFace struct {
	// An arbitrary half-edge on this face
	Edge uint8
}

// HullVersion is the 64-bit hull version. Useful for validating serialized data.
const HullVersion uint64 = 0x6B1E39D4A87C25F3

// HullData is a convex hull.
// Note: This data structure has data hanging off the end and cannot be directly copied.
type HullData struct {
	// Version must be first and match HullVersion
	Version uint64

	// Hash of this hull (this field is zero when the hash is computed).
	Hash uint64

	// Axis-aligned box in local space.
	AABB AABB

	// Surface area, typically in squared meters.
	SurfaceArea float32

	// Volume, typically in m^3.
	Volume float32

	// The radius of the largest sphere at the center.
	InnerRadius float32

	// The local centroid.
	Center Vec3

	// The inertia tensor about the centroid.
	CentralInertia Matrix3

	// The vertex count.
	VertexCount int32

	// Offset of the vertex array in bytes from the struct address.
	VertexOffset int32

	// Offset of the point array in bytes from the struct address.
	PointOffset int32

	// This is the half-edge count (double the edge count).
	EdgeCount int32

	// Offset of the edge array in bytes from the struct address.
	EdgeOffset int32

	// The face count. Hulls faces are convex polygons.
	FaceCount int32

	// Offset of the face plane array in bytes from the struct address.
	PlaneOffset int32

	// Offset of the face array in bytes from the struct address.
	FaceOffset int32

	// Offset of structure of array (SOA) vertices.
	SoaVertexOffset int32

	// Offset of structure of array (SOA) unit normal vectors.
	SoaNormalOffset int32

	// Offset of dot(n1, n2) for each full edge.
	EdgeCosineOffset int32

	// The total number of bytes for this hull.
	ByteCount int32

	// Explicit padding for determinism.
	Padding int32

	// Any padding must be explicit.
}

// BoxHull is an efficient box hull
type BoxHull struct {
	// The embedded hull. So the offsets index into the arrays that follow.
	Base        HullData
	BoxVertices [8]HullVertex    // Box vertices.
	BoxPoints   [8]Vec3          // Box points.
	BoxEdges    [24]HullHalfEdge // Box half-edges.
	BoxPlanes   [6]Plane         // Box face planes.
	BoxFaces    [6]HullFace      // Box faces.
	Padding     [2]uint8         // Explicit padding.
	Vx          [8]float32       // vertex x
	Vy          [8]float32       // vertex y
	Vz          [8]float32       // vertex z
	Nx          [8]float32       // normal x, padded to multiple of 8
	Ny          [8]float32       // normal y, padded to multiple of 8
	Nz          [8]float32       // normal z, padded to multiple of 8
	EdgeCosines [12]float32      // dot(n1, n2) for each full edge.
}

// Triangle Mesh

// MeshDef is used to create a re-usable collision mesh. No pointers
// are held to this data in MeshData. So all this data can be temporary.
type MeshDef struct {
	// Triangle vertices.
	Vertices []Vec3

	// Stride between vertices. Use 0 for contiguous vertices.
	// PORT: in bytes as upstream; with a []Vec3 only 0 or 12 are meaningful. Revisit with mesh.c.
	Stride int

	// Triangle vertex indices. 3 for each triangle. CCW winding unless CW is indicated below.
	Indices []int32

	// Triangle material index. 1 per triangle. Indexes into ShapeDef.Materials.
	// This allows different run-time material data to be associated with different
	// instances of this mesh.
	MaterialIndices []uint8

	// Tolerance for vertex welding in length units.
	WeldTolerance float32

	// The vertex count. Must be 3 or more.
	VertexCount int32

	// The triangle count. Must be 1 or more.
	TriangleCount int32

	// Optionally weld nearby vertices.
	WeldVertices bool

	// Use the median split instead of SAH to speed up mesh creation. Good
	// for meshes that are structured like a grid.
	UseMedianSplit bool

	// Compute triangle adjacency information using shared edges
	IdentifyEdges bool

	// Input indices have clockWise winding order.
	ClockWiseWinding bool
}

// MeshVersion is the 64-bit mesh version. Useful for validating serialized data.
const MeshVersion uint64 = 0xAAAB9A00F1A8AAF7

// MeshEdgeFlags are the triangle mesh edge flags.
type MeshEdgeFlags int32

const (
	ConcaveEdge1 MeshEdgeFlags = 0x01
	ConcaveEdge2 MeshEdgeFlags = 0x02
	ConcaveEdge3 MeshEdgeFlags = 0x04

	InverseConcaveEdge1 MeshEdgeFlags = 0x10
	InverseConcaveEdge2 MeshEdgeFlags = 0x20
	InverseConcaveEdge3 MeshEdgeFlags = 0x40

	AllConcaveEdges = ConcaveEdge1 | ConcaveEdge2 | ConcaveEdge3

	FlatEdge1 = ConcaveEdge1 | InverseConcaveEdge1
	FlatEdge2 = ConcaveEdge2 | InverseConcaveEdge2
	FlatEdge3 = ConcaveEdge3 | InverseConcaveEdge3

	AllFlatEdges = FlatEdge1 | FlatEdge2 | FlatEdge3
)

// MeshTriangle is a mesh triangle.
type MeshTriangle struct {
	Index1 int32 // Index of vertex 1.
	Index2 int32 // Index of vertex 2.
	Index3 int32 // Index of vertex 3.
}

// MeshNode is a mesh BVH node.
type MeshNode struct {
	// The lower bound of the node AABB. Strategic placement for SIMD.
	LowerBound Vec3

	// PORT: anonymous union of two 32-bit bit fields:
	//   asNode: axis : 2 (split axis 0, 1, or 2), childOffset : 30 (offset of the second child node)
	//   asLeaf: type : 2 (3 if this is a leaf), triangleCount : 30 (number of triangles for this leaf)
	// The low 2 bits are axis/type, the high 30 bits the offset/count (GCC/Clang/MSVC layout on
	// little endian). Accessors land with mesh.c.
	Data uint32

	// The upper bound of the node AABB.  Strategic placement for SIMD.
	UpperBound Vec3

	// The index of the leaf triangles.
	TriangleOffset uint32
}

// MeshData is a sorted triangle collision bounding volume hierarchy.
// Note: This struct has data hanging off the end and cannot be directly copied.
type MeshData struct {
	// Version must be first.
	Version uint64

	// Hash of this mesh (this field is zero when the hash is computed)
	Hash uint64

	// The total number of bytes for this mesh.
	ByteCount int32

	// Local axis-aligned box.
	Bounds AABB

	// Combined surface area of all triangles. Single-sided.
	SurfaceArea float32

	// The height of the bounding volume hierarchy.
	TreeHeight int32

	// The number of degenerate triangles. Diagnostic.
	DegenerateCount int32

	// Offset of the node array in bytes from the struct address.
	NodeOffset int32

	// The number of BVH nodes.
	NodeCount int32

	// Offset of the vertex array in bytes from the struct address.
	VertexOffset int32

	// The number of vertices.
	VertexCount int32

	// Offset of the triangle array in bytes from the struct address.
	TriangleOffset int32

	// The number of triangles.
	TriangleCount int32

	// Offset of the material array in bytes from the struct address.
	MaterialOffset int32

	// The number of materials.
	MaterialCount int32

	// Offset of the triangle flag array in bytes from the struct address.
	FlagsOffset int32

	// Explicit padding.
	Padding int32
}

// Mesh allows mesh data to be re-used with different scales.
type Mesh struct {
	// Immutable pointer to the mesh data.
	Data *MeshData

	// This scale may be non-uniform and have negative components. However,
	// no component may be very small in magnitude.
	Scale Vec3
}

// Height Field

// HeightFieldDef is the data used to create a height field. No pointers are held to this data.
type HeightFieldDef struct {
	// Grid point heights
	// count = countX * countZ
	Heights []float32

	// Grid cell material
	// A value of 0xFF is reserved for holes
	// count = (countX - 1) * (countZ - 1)
	MaterialIndices []uint8

	// The height field scale. All components must be positive values.
	Scale Vec3

	// The number of grid lines along the x-axis.
	CountX int32

	// The number of grid lines along the z-axis.
	CountZ int32

	// Global minimum and maximum heights used for quantization. This is important
	// if you want height fields to be placed next to each other and line up exactly.
	// In that case, both height fields should use the same minimum and maximum heights.
	// All height values are clamped to this range.
	// These values are in unscaled space.
	GlobalMinimumHeight float32

	// The maximum.
	GlobalMaximumHeight float32

	// Use clock-wise winding. This effectively inverts the height-field along the y-axis.
	ClockwiseWinding bool
}

// HeightFieldHole is the material index used to designate holes in a height field.
const HeightFieldHole = 0xFF

// HeightFieldVersion is the 64-bit height-field version. Useful for validating serialized data.
const HeightFieldVersion uint64 = 0x8E41E5FB084848F8

// HeightFieldData is a height field with compressed storage.
// Note: This data structure has data hanging off the end and cannot be directly copied.
type HeightFieldData struct {
	// Version must be first and match HeightFieldVersion
	Version uint64

	// Hash of this height field (this field is zero when the hash is computed).
	Hash uint64

	// The total number of bytes for this height field.
	ByteCount int32

	// The local axis-aligned bounding box.
	AABB AABB

	// The minimum y value.
	MinHeight float32

	// The maximum y value
	MaxHeight float32

	// The quantization scale.
	HeightScale float32

	// The overall scale.
	Scale Vec3

	// The number of grid columns along the local x-axis.
	ColumnCount int32

	// The number of grid rows along the local z-axis.
	RowCount int32

	// Offset of the compressed height array in bytes from the struct address.
	// uint16_t, one per grid point.
	HeightsOffset int32

	// Offset of the material index array in bytes from the struct address.
	// uint8_t, one per cell.
	MaterialOffset int32

	// Offset of the flag array in bytes from the struct address.
	// uint8_t, one per triangle.
	FlagsOffset int32

	// Triangle winding.
	Clockwise uint8

	// Explicit padding.
	Padding [7]uint8
}

// Compound

// CompoundCapsuleDef is the definition for a capsule in a compound shape.
type CompoundCapsuleDef struct {
	// Local capsule.
	Capsule Capsule

	// Material properties.
	Material SurfaceMaterial
}

// CompoundHullDef is the definition for a convex hull in a compound shape.
type CompoundHullDef struct {
	// Shared hull.
	Hull *HullData

	// Transform of the shared hull into compound local space.
	Transform Transform

	// Material properties.
	Material SurfaceMaterial
}

// CompoundMeshDef is the definition for a triangle mesh in a compound shape.
type CompoundMeshDef struct {
	// Shared mesh.
	MeshData *MeshData

	// Transform of the shared mesh into compound local space.
	Transform Transform

	// Local space non-uniform mesh scale. May have negative components.
	Scale Vec3

	// Material properties.
	// This array must line up with the material indices on the triangles.
	Materials []SurfaceMaterial

	// Number of materials.
	MaterialCount int32
}

// CompoundSphereDef is the definition for a sphere in a compound shape.
type CompoundSphereDef struct {
	// Local sphere.
	Sphere Sphere

	// Material properties.
	Material SurfaceMaterial
}

// CompoundDef is the definition for creating a compound shape. All this data is fully cloned
// into the run-time compound shape.
type CompoundDef struct {
	// Capsule instances.
	Capsules []CompoundCapsuleDef

	// Number of capsules.
	CapsuleCount int32

	// Hulls instances.
	Hulls []CompoundHullDef

	// Number of hull instances.
	HullCount int32

	// Mesh instances.
	Meshes []CompoundMeshDef

	// Number of mesh instances.
	MeshCount int32

	// Sphere instances.
	Spheres []CompoundSphereDef

	// Number of spheres.
	SphereCount int32
}

// CompoundVersion: the baked compound version depends on the mesh and hull versions.
const CompoundVersion uint64 = 0x9E4B17D3A25C68F1 ^ MeshVersion ^ HullVersion

// CompoundData is the data for a baked compound shape. This is a potentially large yet highly optimized
// data structure. It can contain thousands of child shapes, yet at runtime it populates
// into the world as a single shape in the runtime broad-phase.
// This data structure has data living off the end and must be accessed using offsets.
// Accessors are provided for user relevant data.
// Note: you don't need to use this to create runtime compounds. For runtime compounds you can
// add multiple shapes to a body using the regular shape creation functions.
type CompoundData struct {
	// The compound version is always first.
	Version uint64

	// The total number of bytes for this compound.
	ByteCount int32

	// Bounds of the tree.
	Bounds AABB

	// Tree height for diagnostics.
	TreeHeight int32

	// Offset of the tree node array in bytes from the struct address.
	NodeOffset int32

	// The number of tree nodes.
	NodeCount int32

	// Offset of the material array in bytes from the struct address.
	MaterialOffset int32

	// The number of materials.
	MaterialCount int32

	// Offset of the capsule array in bytes from the struct address.
	CapsuleOffset int32

	// The number of capsules.
	CapsuleCount int32

	// Offset of the hull instance array in bytes from the struct address.
	HullOffset int32

	// The number of hull instances.
	HullCount int32

	// The number of unique hulls. Diagnostic.
	SharedHullCount int32

	// Offset of the mesh instance array in bytes from the struct address.
	MeshOffset int32

	// The number of mesh instances.
	MeshCount int32

	// The number of unique meshes. Diagnostic.
	SharedMeshCount int32

	// Offset of the sphere array in bytes from the struct address.
	SphereOffset int32

	// The number of spheres.
	SphereCount int32
}

// CompoundCapsule is a capsule that lives in a compound.
type CompoundCapsule struct {
	// Local capsule.
	Capsule Capsule

	// Index to a shared material.
	MaterialIndex uint16

	// Padding for determinism.
	Padding uint16
}

// CompoundHull is a hull that lives in a compound.
type CompoundHull struct {
	// Pointer to the unique shared hull.
	Hull *HullData

	// The transform of this hull instance.
	Transform Transform

	// Index to a shared material.
	MaterialIndex uint16

	// Padding for determinism.
	Padding uint16
}

// CompoundMesh is a mesh with non-uniform scale that lives in a compound.
type CompoundMesh struct {
	// Pointer to the unique shared mesh.
	MeshData *MeshData

	// The transform of this mesh instance.
	Transform Transform

	// Non-uniform scale of this mesh instance.
	Scale Vec3

	// This is used to access the surface material from GetCompoundMaterials.
	// Requires an extra level of indirection.
	// materialIndex = materialIndices[triangle->materialIndex]
	MaterialIndices []uint16

	// The number of materials. 1 for convex hapes.
	MaterialCount int32
}

// CompoundSphere is a sphere that lives in a compound.
type CompoundSphere struct {
	// Local sphere.
	Sphere Sphere

	// Index to a shared material.
	MaterialIndex uint16

	// Padding for determinism.
	Padding uint16
}

// ChildShape is a child shape of a compound
type ChildShape struct {
	// PORT: tagged union { capsule; hull; mesh; sphere }. Only the member selected by Type is
	// meaningful.
	Capsule Capsule   // Capsule.
	Hull    *HullData // Hull.
	Mesh    Mesh      // Mesh.
	Sphere  Sphere    // Sphere.

	// Transform of the shape into compound local space.
	Transform Transform

	// Material indices. Index 0 is used for convex shapes.
	MaterialIndices []uint16

	// The number of materials. This is one for convex shapes. For
	// meshes it is determined by the maximum triangle material index.
	MaterialCount int32

	// The shape type (union tag).
	Type ShapeType
}

// CompoundQueryFcn is the callback for compound overlap queries.
type CompoundQueryFcn func(compound *CompoundData, childIndex int, context any) bool

// Shape Collision
// Collide pairs of shapes.

// ManifoldPoint is a contact point belonging to a contact manifold.
// It holds details related to the geometry and dynamics of the contact points.
// Box3D uses speculative collision so some contact points may be separated.
// You may use the maxNormalImpulse to determine if there was an interaction during
// the time step.
type ManifoldPoint struct {
	// Location of the contact point relative to the bodyA center of mass in world space.
	AnchorA Vec3

	// Location of the contact point relative to the bodyB center of mass in world space.
	AnchorB Vec3

	// The separation of the contact point, negative if penetrating
	Separation float32

	// The impulse along the manifold normal vector. Since Box3D uses sub-stepping, this is
	// result from the final sub-step.
	NormalImpulse float32

	// The total normal impulse applied across sub-stepping and restitution. This is important
	// to identify speculative contact points that had an interaction in the time step.
	// This includes the warm starting impulse, the sub-step delta impulse, and the restitution
	// impulse.
	TotalNormalImpulse float32

	// Relative normal velocity pre-solve. Negative when approaching. This is only
	// computed if hit events are enabled.
	NormalVelocity float32

	// Cached separation used for contact recycling
	BaseSeparation float32

	// Local point for matching
	// Uniquely identifies a contact point between two shapes
	FeatureId uint32

	// Triangle index if one of the shapes is a mesh or height field
	TriangleIndex int32

	// Did this contact point exist in the previous step?
	Persisted bool
}

// Manifold is a contact manifold describes the contact points between colliding shapes.
// Note: Box3D uses speculative collision so some contact points may be separated.
type Manifold struct {
	// The manifold points. There may be 1 to 4 valid points.
	Points [MaxManifoldPoints]ManifoldPoint

	// The unit normal vector in world space, points from shape A to shape B
	Normal Vec3

	// Central friction angular impulse (applied about the normal)
	TwistImpulse float32

	// Central friction linear impulse
	FrictionImpulse Vec3

	// Rolling resistance angular impulse
	RollingImpulse Vec3

	// The number of contact points, will be 0 to 4
	PointCount int32
}

// SeparatingFeature is the cached separating axis feature.
type SeparatingFeature int32

const (
	InvalidAxis SeparatingFeature = iota
	BacksideAxis
	FaceAxisA
	FaceAxisB
	EdgePairAxis
	ClosestPointsAxis

	// These are for testing
	ManualFaceAxisA
	ManualFaceAxisB
	ManualEdgePairAxis
)

// TriangleFeature is the cached triangle feature.
type TriangleFeature int32

const (
	FeatureNone TriangleFeature = iota
	FeatureTriangleFace
	FeatureHullFace
	// v1-v2
	FeatureEdge1
	// v2-v3
	FeatureEdge2
	// v3-v1
	FeatureEdge3
	FeatureVertex1
	FeatureVertex2
	FeatureVertex3
)

// SATCache is the separating axis test cache. Provides temporal acceleration of collision routines.
type SATCache struct {
	// The separation when the cache is populated. Negative for overlap.
	Separation float32

	// SeparatingFeature.
	Type uint8

	// Index of the feature on shape A.
	IndexA uint8

	// Index of the feature on shape B.
	IndexB uint8

	// Was the cache re-used?
	Hit uint8
}

// FeaturePair: contact points are always the result of two edges intersecting.
// It can be two edges of the same shape, which is just a shape vertex.
// Or a contact point can be the result of two edges crossing from different shapes.
// This is designed to support hull versus hull, but it is adapted to work
// with all shape types. The feature pair is used to identify contact points
// for temporal coherence and warm starting.
type FeaturePair struct {
	// Incoming type (either edge on shape A or shape B)
	Owner1 uint8
	// Incoming edge index (into associated shape array)
	Index1 uint8
	// Outgoing type (either edge on shape A or shape B)
	Owner2 uint8
	// Outgoing edge index (into associated shape array)
	Index2 uint8
}

// LocalManifoldPoint is a local manifold point and normal in frame A.
type LocalManifoldPoint struct {
	// Local point in frame A.
	Point Vec3

	// The contact point separation. Negative for overlap.
	Separation float32

	// The feature pair for this point.
	Pair FeaturePair

	// The triangle index when collide with a mesh or height-field.
	TriangleIndex int32
}

// LocalManifold is a local manifold with no dynamic information. Used by Collide functions.
type LocalManifold struct {
	// Local normal in frame A.
	Normal Vec3

	// The triangle normal.
	TriangleNormal Vec3

	// The manifold points. From a point buffer.
	Points []LocalManifoldPoint

	// The number of manifold points. Only bounded by the buffer capacity.
	PointCount int32

	// The index of the triangle.
	TriangleIndex int32

	I1 int32 // Vertex 1 index.
	I2 int32 // Vertex 2 index.
	I3 int32 // Vertex 3 index.

	// The squared distance of a sphere from a triangle. For ghost collision reduction.
	SquaredDistance float32

	// The triangle feature involved.
	Feature TriangleFeature

	// MeshEdgeFlags.
	TriangleFlags int32
}

// Debug Draw

// HexColor: these colors are used for debug draw and mostly match the named SVG colors.
// See https://www.rapidtables.com/web/color/index.html
// https://johndecember.com/html/spec/colorsvg.html
// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
type HexColor uint32

const (
	ColorAliceBlue            HexColor = 0xF0F8FF
	ColorAntiqueWhite         HexColor = 0xFAEBD7
	ColorAqua                 HexColor = 0x00FFFF
	ColorAquamarine           HexColor = 0x7FFFD4
	ColorAzure                HexColor = 0xF0FFFF
	ColorBeige                HexColor = 0xF5F5DC
	ColorBisque               HexColor = 0xFFE4C4
	ColorBlack                HexColor = 0x000000
	ColorBlanchedAlmond       HexColor = 0xFFEBCD
	ColorBlue                 HexColor = 0x0000FF
	ColorBlueViolet           HexColor = 0x8A2BE2
	ColorBrown                HexColor = 0xA52A2A
	ColorBurlywood            HexColor = 0xDEB887
	ColorCadetBlue            HexColor = 0x5F9EA0
	ColorChartreuse           HexColor = 0x7FFF00
	ColorChocolate            HexColor = 0xD2691E
	ColorCoral                HexColor = 0xFF7F50
	ColorCornflowerBlue       HexColor = 0x6495ED
	ColorCornsilk             HexColor = 0xFFF8DC
	ColorCrimson              HexColor = 0xDC143C
	ColorCyan                 HexColor = 0x00FFFF
	ColorDarkBlue             HexColor = 0x00008B
	ColorDarkCyan             HexColor = 0x008B8B
	ColorDarkGoldenRod        HexColor = 0xB8860B
	ColorDarkGray             HexColor = 0xA9A9A9
	ColorDarkGreen            HexColor = 0x006400
	ColorDarkKhaki            HexColor = 0xBDB76B
	ColorDarkMagenta          HexColor = 0x8B008B
	ColorDarkOliveGreen       HexColor = 0x556B2F
	ColorDarkOrange           HexColor = 0xFF8C00
	ColorDarkOrchid           HexColor = 0x9932CC
	ColorDarkRed              HexColor = 0x8B0000
	ColorDarkSalmon           HexColor = 0xE9967A
	ColorDarkSeaGreen         HexColor = 0x8FBC8F
	ColorDarkSlateBlue        HexColor = 0x483D8B
	ColorDarkSlateGray        HexColor = 0x2F4F4F
	ColorDarkTurquoise        HexColor = 0x00CED1
	ColorDarkViolet           HexColor = 0x9400D3
	ColorDeepPink             HexColor = 0xFF1493
	ColorDeepSkyBlue          HexColor = 0x00BFFF
	ColorDimGray              HexColor = 0x696969
	ColorDodgerBlue           HexColor = 0x1E90FF
	ColorFireBrick            HexColor = 0xB22222
	ColorFloralWhite          HexColor = 0xFFFAF0
	ColorForestGreen          HexColor = 0x228B22
	ColorFuchsia              HexColor = 0xFF00FF
	ColorGainsboro            HexColor = 0xDCDCDC
	ColorGhostWhite           HexColor = 0xF8F8FF
	ColorGold                 HexColor = 0xFFD700
	ColorGoldenRod            HexColor = 0xDAA520
	ColorGray                 HexColor = 0x808080
	ColorGreen                HexColor = 0x008000
	ColorGreenYellow          HexColor = 0xADFF2F
	ColorHoneyDew             HexColor = 0xF0FFF0
	ColorHotPink              HexColor = 0xFF69B4
	ColorIndianRed            HexColor = 0xCD5C5C
	ColorIndigo               HexColor = 0x4B0082
	ColorIvory                HexColor = 0xFFFFF0
	ColorKhaki                HexColor = 0xF0E68C
	ColorLavender             HexColor = 0xE6E6FA
	ColorLavenderBlush        HexColor = 0xFFF0F5
	ColorLawnGreen            HexColor = 0x7CFC00
	ColorLemonChiffon         HexColor = 0xFFFACD
	ColorLightBlue            HexColor = 0xADD8E6
	ColorLightCoral           HexColor = 0xF08080
	ColorLightCyan            HexColor = 0xE0FFFF
	ColorLightGoldenRodYellow HexColor = 0xFAFAD2
	ColorLightGray            HexColor = 0xD3D3D3
	ColorLightGreen           HexColor = 0x90EE90
	ColorLightPink            HexColor = 0xFFB6C1
	ColorLightSalmon          HexColor = 0xFFA07A
	ColorLightSeaGreen        HexColor = 0x20B2AA
	ColorLightSkyBlue         HexColor = 0x87CEFA
	ColorLightSlateGray       HexColor = 0x778899
	ColorLightSteelBlue       HexColor = 0xB0C4DE
	ColorLightYellow          HexColor = 0xFFFFE0
	ColorLime                 HexColor = 0x00FF00
	ColorLimeGreen            HexColor = 0x32CD32
	ColorLinen                HexColor = 0xFAF0E6
	ColorMagenta              HexColor = 0xFF00FF
	ColorMaroon               HexColor = 0x800000
	ColorMediumAquaMarine     HexColor = 0x66CDAA
	ColorMediumBlue           HexColor = 0x0000CD
	ColorMediumOrchid         HexColor = 0xBA55D3
	ColorMediumPurple         HexColor = 0x9370DB
	ColorMediumSeaGreen       HexColor = 0x3CB371
	ColorMediumSlateBlue      HexColor = 0x7B68EE
	ColorMediumSpringGreen    HexColor = 0x00FA9A
	ColorMediumTurquoise      HexColor = 0x48D1CC
	ColorMediumVioletRed      HexColor = 0xC71585
	ColorMidnightBlue         HexColor = 0x191970
	ColorMintCream            HexColor = 0xF5FFFA
	ColorMistyRose            HexColor = 0xFFE4E1
	ColorMoccasin             HexColor = 0xFFE4B5
	ColorNavajoWhite          HexColor = 0xFFDEAD
	ColorNavy                 HexColor = 0x000080
	ColorOldLace              HexColor = 0xFDF5E6
	ColorOlive                HexColor = 0x808000
	ColorOliveDrab            HexColor = 0x6B8E23
	ColorOrange               HexColor = 0xFFA500
	ColorOrangeRed            HexColor = 0xFF4500
	ColorOrchid               HexColor = 0xDA70D6
	ColorPaleGoldenRod        HexColor = 0xEEE8AA
	ColorPaleGreen            HexColor = 0x98FB98
	ColorPaleTurquoise        HexColor = 0xAFEEEE
	ColorPaleVioletRed        HexColor = 0xDB7093
	ColorPapayaWhip           HexColor = 0xFFEFD5
	ColorPeachPuff            HexColor = 0xFFDAB9
	ColorPeru                 HexColor = 0xCD853F
	ColorPink                 HexColor = 0xFFC0CB
	ColorPlum                 HexColor = 0xDDA0DD
	ColorPowderBlue           HexColor = 0xB0E0E6
	ColorPurple               HexColor = 0x800080
	ColorRebeccaPurple        HexColor = 0x663399
	ColorRed                  HexColor = 0xFF0000
	ColorRosyBrown            HexColor = 0xBC8F8F
	ColorRoyalBlue            HexColor = 0x4169E1
	ColorSaddleBrown          HexColor = 0x8B4513
	ColorSalmon               HexColor = 0xFA8072
	ColorSandyBrown           HexColor = 0xF4A460
	ColorSeaGreen             HexColor = 0x2E8B57
	ColorSeaShell             HexColor = 0xFFF5EE
	ColorSienna               HexColor = 0xA0522D
	ColorSilver               HexColor = 0xC0C0C0
	ColorSkyBlue              HexColor = 0x87CEEB
	ColorSlateBlue            HexColor = 0x6A5ACD
	ColorSlateGray            HexColor = 0x708090
	ColorSnow                 HexColor = 0xFFFAFA
	ColorSpringGreen          HexColor = 0x00FF7F
	ColorSteelBlue            HexColor = 0x4682B4
	ColorTan                  HexColor = 0xD2B48C
	ColorTeal                 HexColor = 0x008080
	ColorThistle              HexColor = 0xD8BFD8
	ColorTomato               HexColor = 0xFF6347
	ColorTurquoise            HexColor = 0x40E0D0
	ColorViolet               HexColor = 0xEE82EE
	ColorWheat                HexColor = 0xF5DEB3
	ColorWhite                HexColor = 0xFFFFFF
	ColorWhiteSmoke           HexColor = 0xF5F5F5
	ColorYellow               HexColor = 0xFFFF00
	ColorYellowGreen          HexColor = 0x9ACD32
	ColorBox2DRed             HexColor = 0xDC3132
	ColorBox2DBlue            HexColor = 0x30AEBF
	ColorBox2DGreen           HexColor = 0x8CC924
	ColorBox2DYellow          HexColor = 0xFFEE8C
)

// DebugMaterial is a debug draw material preset. Optionally packed into the unused high byte of a
// HexColor (or SurfaceMaterial.CustomColor) to drive the renderer's PBR
// roughness and metalness. The low 24 bits stay RGB, so a plain 0xRRGGBB color
// reads as DebugMaterialDefault and keeps the renderer's per-body-type look.
type DebugMaterial int32

const (
	DebugMaterialDefault DebugMaterial = iota
	DebugMaterialMatte
	DebugMaterialSoft
	DebugMaterialDead
	DebugMaterialGlossy
	DebugMaterialMetallic
)

// MakeDebugColor packs an RGB color with a material preset for debug draw. The preset rides in
// the high byte where the color converters ignore it.
func MakeDebugColor(rgb HexColor, material DebugMaterial) uint32 {
	return (uint32(rgb) & 0x00FFFFFF) | (uint32(material) << 24)
}

// GetGraphColor is implemented in constraint_graph.go.

// DebugShape is sent to the user for debug shape creation. The user should know the type in case they have
// custom sphere or capsule rendering.
type DebugShape struct {
	// Shape id.
	ShapeId ShapeID

	// Shape type.
	Type ShapeType

	// PORT: tagged union of pointers. Only the member selected by Type is set.
	Capsule     *Capsule         // Capsule shape.
	Compound    *CompoundData    // Compound shape.
	HeightField *HeightFieldData // Height-field shape.
	Hull        *HullData        // Convex hull shape.
	Mesh        *Mesh            // Mesh shape with scale.
	Sphere      *Sphere          // Sphere shape.
}

// DebugDraw is passed to World.Draw to draw a debug view of the simulation world.
// Callbacks receive world coordinates. In large world mode the translation is double precision so
// it stays accurate far from the origin. Shift into your own camera frame inside the callbacks.
type DebugDraw struct {
	// Draws a user shape. The userShape pointer is owned by the application and is known to Box3D as
	// an opaque pointer returned from CreateDebugShapeCallback. When this is called the drawn shape has
	// passed a culling test against drawingBounds below.
	DrawShapeFcn func(userShape any, transform WorldTransform, color HexColor, context any)

	// Draw a line segment.
	DrawSegmentFcn func(p1, p2 Pos, color HexColor, context any)

	// Draw a transform. Choose your own length scale.
	DrawTransformFcn func(transform WorldTransform, context any)

	// Draw a point.
	DrawPointFcn func(p Pos, size float32, color HexColor, context any)

	// Draw a sphere.
	DrawSphereFcn func(p Pos, radius float32, color HexColor, alpha float32, context any)

	// Draw a capsule.
	DrawCapsuleFcn func(p1, p2 Pos, radius float32, color HexColor, alpha float32, context any)

	// Draw a bounding box.
	DrawBoundsFcn func(aabb AABB, color HexColor, context any)

	// Draw an oriented box.
	DrawBoxFcn func(extents Vec3, transform WorldTransform, color HexColor, context any)

	// Draw a string in world space
	DrawStringFcn func(p Pos, s string, color HexColor, context any)

	// World bounds to use for debug draw
	DrawingBounds AABB

	// Scale to use when drawing forces
	ForceScale float32

	// Global scaling for joint drawing
	JointScale float32

	// Option to draw shapes
	DrawShapes bool

	// Option to draw joints
	DrawJoints bool

	// Option to draw additional information for joints
	DrawJointExtras bool

	// Option to draw the bounding boxes for shapes
	DrawBounds bool

	// Option to draw the mass and center of mass of dynamic bodies
	DrawMass bool

	// Option to draw the sleep information for dynamic and kinematic bodies
	DrawSleep bool

	// Option to draw body names
	DrawBodyNames bool

	// Option to draw contact points
	DrawContacts bool

	// Draw contact anchor A or B
	DrawAnchorA bool

	// Option to visualize the graph coloring used for contacts and joints
	DrawGraphColors bool

	// Option to draw contact features
	DrawContactFeatures bool

	// Option to draw contact normals
	DrawContactNormals bool

	// Option to draw contact normal forces
	DrawContactForces bool

	// Option to draw islands as bounding boxes
	DrawIslands bool

	// User context that is passed as an argument to drawing callback functions
	Context any
}

// DefaultDebugDraw: create a debug draw struct with default values. Implemented below with types.c.

// ---- types.c ----

// DefaultWorldDef: use this to initialize your world definition
func DefaultWorldDef() WorldDef {
	lengthUnits := GetLengthUnitsPerMeter()

	var def WorldDef
	def.Gravity.X = 0.0
	def.Gravity.Y = -10.0
	def.HitEventThreshold = 1.0 * lengthUnits
	def.RestitutionThreshold = 1.0 * lengthUnits
	def.RestitutionIterations = 2
	def.EnableRestitutionPropagation = false
	def.ContactSpeed = 3.0 * lengthUnits
	def.ContactHertz = 30.0
	def.ContactDampingRatio = 10.0

	// 400 meters per second, faster than the speed of sound
	def.MaximumLinearSpeed = 400.0 * lengthUnits

	def.IsotropicInertiaTolerance = 0.2

	def.EnableSleep = true
	def.EnableContinuous = true
	def.internalValue = secretCookie
	return def
}

// DefaultBodyDef: use this to initialize your body definition
func DefaultBodyDef() BodyDef {
	var def BodyDef
	def.Type = StaticBody
	def.Rotation = QuatIdentity
	def.SleepThreshold = 0.05 * GetLengthUnitsPerMeter()
	def.SafetyFactor = 0.5
	def.GravityScale = 1.0
	def.EnableSleep = true
	def.IsAwake = true
	def.IsEnabled = true
	def.EnableContactRecycling = true
	def.internalValue = secretCookie
	return def
}

// DefaultFilter: use this to initialize your filter
func DefaultFilter() Filter {
	filter := Filter{DefaultCategoryBits, DefaultMaskBits, 0}
	return filter
}

// DefaultQueryFilter: use this to initialize your query filter
func DefaultQueryFilter() QueryFilter {
	filter := QueryFilter{CategoryBits: DefaultCategoryBits, MaskBits: DefaultMaskBits}
	return filter
}

// DefaultSurfaceMaterial: use this to initialize your surface material
func DefaultSurfaceMaterial() SurfaceMaterial {
	var surfaceMaterial SurfaceMaterial
	surfaceMaterial.Friction = 0.6
	return surfaceMaterial
}

// DefaultShapeDef: use this to initialize your shape definition
func DefaultShapeDef() ShapeDef {
	lengthUnits := GetLengthUnitsPerMeter()

	var def ShapeDef
	def.BaseMaterial = DefaultSurfaceMaterial()
	// density of water
	def.Density = 1000.0 / (lengthUnits * lengthUnits * lengthUnits)
	def.ExplosionScale = 1.0
	def.Filter = DefaultFilter()
	def.UpdateBodyMass = true
	def.InvokeContactCreation = true
	def.EnableSpeculativeContact = true
	def.internalValue = secretCookie
	return def
}

func emptyDrawShape(userShape any, transform WorldTransform, color HexColor, context any) {}

func emptyDrawSegment(p1, p2 Pos, color HexColor, context any) {}

func emptyDrawTransform(transform WorldTransform, context any) {}

func emptyDrawPoint(p Pos, size float32, color HexColor, context any) {}

func emptyDrawSphere(p Pos, radius float32, color HexColor, alpha float32, context any) {}

func emptyDrawCapsule(p1, p2 Pos, radius float32, color HexColor, alpha float32, context any) {}

func emptyDrawBounds(aabb AABB, color HexColor, context any) {}

func emptyDrawBox(extents Vec3, transform WorldTransform, color HexColor, context any) {}

func emptyDrawString(p Pos, s string, color HexColor, context any) {}

// DefaultDebugDraw creates a debug draw struct with default values.
func DefaultDebugDraw() DebugDraw {
	var draw DebugDraw

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawShapeFcn = emptyDrawShape
	draw.DrawSegmentFcn = emptyDrawSegment
	draw.DrawTransformFcn = emptyDrawTransform
	draw.DrawPointFcn = emptyDrawPoint
	draw.DrawSphereFcn = emptyDrawSphere
	draw.DrawCapsuleFcn = emptyDrawCapsule
	draw.DrawBoundsFcn = emptyDrawBounds
	draw.DrawBoxFcn = emptyDrawBox
	draw.DrawStringFcn = emptyDrawString

	// Not too small, not too big.
	h := 100.0 * GetLengthUnitsPerMeter()
	draw.DrawingBounds = AABB{
		LowerBound: Vec3{-h, -h, -h},
		UpperBound: Vec3{h, h, h},
	}

	draw.JointScale = 1.0
	draw.ForceScale = 1.0

	return draw
}
