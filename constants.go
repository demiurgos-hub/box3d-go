// Port of include/box3d/constants.h from Box3D (https://github.com/erincatto/box3d)
// at commit efca0c184d40fd6a82016983f1601c4d23965867.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// Go port: github.com/gataengine/box3d-go authors.

package box3d

// SetLengthUnitsPerMeter and GetLengthUnitsPerMeter are implemented in core.go.
//
// Box3D bases all length units on meters, but you may need different units for your game.
// You can set this value to use different units. This should be done at application startup
// and only modified once. Default value is 1.
// Warning: This must be modified before any calls to Box3D

// PORT: macros that depend on b3GetLengthUnitsPerMeter() are functions. The float32
// multiplication happens at run time exactly as in C.

// Huge is B3_HUGE. Used to detect bad values. In float mode positions greater than about 16km
// have precision problems, so 100km is a safe limit.
// PORT: double precision (BOX3D_DOUBLE_PRECISION, 1.0e9f) is not ported.
func Huge() float32 {
	return float32(1.0e5 * GetLengthUnitsPerMeter())
}

// MaxWorkers is the maximum parallel workers. Used for some fixed size arrays.
const MaxWorkers = 32

// MaxTasks is the maximum number of tasks queued per world step. EnqueueTaskCallback will never be called
// more than this per world step. This is related to MaxWorkers. With 32 workers,
// the maximum observed task count is 130. This allows an external task system to use a fixed
// size array for Box3D task, which may help with creating stable user task pointers.
const MaxTasks = 256

// GraphColorCount is the maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
// The compound barrel benchmark has minor overflow with 24 colors
const GraphColorCount = 24

// ContactManifoldCountBuckets is the number of contact point buckets for counting the number of contact points per
// shape contact pair. This is just for reporting and doesn't affect simulation.
const ContactManifoldCountBuckets = 8

// LinearSlop is B3_LINEAR_SLOP. A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// Warning: modifying this can have a significant impact on stability
func LinearSlop() float32 {
	return float32(0.005 * GetLengthUnitsPerMeter())
}

// MinCapsuleLength is the minimum length of a capsules. Very short capsules should be created as spheres
// to avoid numerical problems.
func MinCapsuleLength() float32 {
	return LinearSlop()
}

// MinFrictionWeight is the minimum contact point friction weight, lower bound for speculative points. Made small
// enough to be washed away by weights that hit 1.
const MinFrictionWeight float32 = 1e-10

// OverlapSlop is B3_OVERLAP_SLOP. The distance between shapes where they are considered overlapped. This is needed
// because GJK may return small positive values for overlapped shapes in degenerate
// configurations.
func OverlapSlop() float32 {
	return float32(0.1 * LinearSlop())
}

// MaxWorlds is the maximum number of simultaneous worlds that can be allocated
const MaxWorlds = 128

// MaxRotation is the maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// Warning: increasing this to 0.5f * B3_PI or greater will break continuous collision.
const MaxRotation float32 = 0.25 * Pi

// SpeculativeDistance is B3_SPECULATIVE_DISTANCE.
// Warning: modifying this can have a significant impact on performance and stability
func SpeculativeDistance() float32 {
	return float32(4.0 * LinearSlop())
}

// MeshRestOffset is B3_MESH_REST_OFFSET. The rest offset is used for mesh contact to reduce ghost
// collisions and assist with CCD. The rest offset adjusts the contact point separation value,
// making the solver push the shapes apart by this distance.
// Must be at least B3_LINEAR_SLOP and less than B3_SPECULATIVE_DISTANCE.
func MeshRestOffset() float32 {
	return float32(1.0 * LinearSlop())
}

// ContactRecycleDistance is the default contact recycling distance.
func ContactRecycleDistance() float32 {
	return float32(10.0 * LinearSlop())
}

// ContactRecycleAngularDistance is the default contact recycling world angle threshold. For performance this value
// is cos(angle/2)^2. This value corresponds to 10 degrees.
const ContactRecycleAngularDistance float32 = 0.99240388

// MaxAABBMargin is B3_MAX_AABB_MARGIN. This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
// Warning: modifying this can have a significant impact on performance
func MaxAABBMargin() float32 {
	return float32(0.05 * GetLengthUnitsPerMeter())
}

// AABBMarginFraction is the per-shape AABB margin as a fraction of the shape extent (capped by B3_MAX_AABB_MARGIN).
// Small shapes get small margins; large shapes are clamped to the cap.
const AABBMarginFraction float32 = 0.125

// TimeToSleep is the time that a body must be still before it will go to sleep. In seconds.
const TimeToSleep float32 = 0.5

// MaxRestitutionIterations is the maximum number of restitution iterations. Needed to avoid a solver overflow.
const MaxRestitutionIterations = 63

// MaxManifoldPoints is the maximum number of contact points between two touching shapes.
// The default and minimum is 4 and this case uses a fast approximate hull to reduce the
// point count. A larger value builds a 2D convex hull of the candidate points and then
// simplifies it to the target by dropping the vertices that contribute the least area.
// Useful for testing, but I don't recommend shipping with it adjusted.
// WARNING: if you change this you risk breaking ABI and a corrupted runtime.
const MaxManifoldPoints = 4

// GyroscopicIterations is the number of iterations for gyroscopic torques.
const GyroscopicIterations = 1

// MaxHullVertices is the maximum number of convex hull vertices. This is fixed for performance reasons.
const MaxHullVertices = 256

// MaxHullFaces is the maximum number of convex hull faces.
const MaxHullFaces = 512

// MaxHullEdges is the maximum number of convex hull edges. Full edges, not half-edges.
const MaxHullEdges = 768

// ParallelEdgeTol is the relative tolerance used to determine if two edges are parallel.
const ParallelEdgeTol float32 = 0.005

// MaxShapeCastPoints is the maximum number points to use for shape cast proxies (swept point cloud).
const MaxShapeCastPoints = MaxHullVertices

// These generous limits allow for easy hashing. See shapePairKey.
const (
	ShapePower     = 22
	ChildPower     = 64 - 2*ShapePower
	MaxShapes      = 1 << ShapePower
	MaxChildShapes = 1 << ChildPower
)

// MaxMeshContactTriangles is the limit on how many mesh or heightfield triangles a single convex shape can collide with.
// Increasing this will increase stack usage, so be careful. I recommend to simplify your collision data
// before increasing this. For example, using render mesh for collision often leads to poor performance.
const MaxMeshContactTriangles = 512
