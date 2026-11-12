# Golem physics fork

Base: `gataengine/box3d-go` commit `2a00e926a8d92ae1d57a2e425aadebc4610b0ed5`.
Underlying Box3D source: `erincatto/box3d` commit `efca0c184d40fd6a82016983f1601c4d23965867`.
Original MIT attribution and port notes remain intact. This fork uses Go 1.27.1
and float32 positions, geometry and simulation. It does not change default cast semantics
or implement character pushing through manual impulses.

## Additive query extensions

- Motion traversal uses caller-owned small proxy arrays and nonescaping compound
  callbacks. Per-call admission/contact scratch is reused across microsteps;
  observer samples retain independent body/contact/manifold storage. Allocation
  regressions cover mesh/compound casts and retained contacts. These changes do
  not reduce geometry traversal, alter solver cadence, or establish capacity.
- `ShapeID.CastMotion` reads both body trajectories with linear origins and
  shortest-arc slerped rotations. Full rotational bounds and existing leaf BVHs
  retain child/triangle identity. Conservative advancement uses support-plane
  lower bounds and reports nonconvergence instead of clearance. Exact initial
  touching/penetration remains distinct from its positive tolerance band.
  Near face contact, existing manifold normals can resolve a GJK witness that
  is no longer a separating plane. A stalled representable fraction gets a
  geometry probe bounded by one quarter of the query tolerance; an unresolved
  probe remains an explicit error. Fixed-substep resting hull/mesh regressions
  cover these cases in addition to the large-displacement fixtures.
  Triangle/triangle and one-sided rotating triangle queries explicitly fail.
  `MotionBounds` exposes the complete swept bound for caller candidate indexing.
- `WorldID.PreciseCastShape` uses geometric conservative advancement with a
  caller-selected distance tolerance, independent of the solver's rest gap.
  This fixes the approximately 5 mm rounded-shape sweep bias for Golem queries
  without changing upstream `ShapeCast`. Nonconvergence is an explicit failure.
  Focused regressions cover capsule/box distance and sphere casts at origin,
  ±768, ±2304 and near the ±4096 metre coordinate envelope.
- `ShapeID.ClosestPointFeature`: bounded closest points on primitive solids,
  triangle meshes, heightfields (including holes), and baked compounds. Results
  include child/triangle/local feature identity and user material identity;
  equal-distance ties have an explicit ordering.
- `ShapeID.CapsulePenetrations`: caller-buffer contacts including deep hull and
  triangle core overlap, using the existing collision manifolds. Returns the
  full required contact count so overflow cannot masquerade as a complete result.
  One-sided/two-sided triangle behavior is explicit. Contacts are individual
  constraints, not a single global minimum translation for a concave scene.
- `ShapeID.ProxyPenetrations`: equivalent contact results for spheres, capsules
  and unrounded full-dimensional convex proxies (including rotated boxes).
  Convex proxies cook a temporary hull; rounded or degenerate proxies fail.
- `ShapeID.PairPenetration`: the deepest leaf contact, A-to-B normal, witnesses,
  per-side features/materials, and exact touching/penetrating state. No solver
  slop is subtracted. Triangle/triangle pairs explicitly remain unsupported.
- `QueryFilter.IgnoreShapeMask`: opt-in query visibility independent of solid
  collision masks; the requested category mask is always enforced.
- `QueryFilter.BackFaces`: opt-in two-sided ray/shape casts for mesh, terrain and
  compound leaves. Original `ShapeCast` numerical margins, fraction clipping and
  initial-overlap behavior are retained. The stale upstream ShapeCast comment
  now describes the actual fraction-zero initial-overlap result. OverlapShape
  remains an unoriented candidate query; ProxyPenetrations supplies side policy.
- Deep capsules crossing a cooked coplanar seam use face contacts when adjacency
  identifies an artificial flat edge. MeshDef.IdentifyEdges must be enabled.
  Reflected geometry and reverse-side queries preserve edge identity/flatness.
- `ComputeHeightFieldQueryAABB` conservatively includes compression/float32
  rounding error. Internal world bounds, terrain sweep traversal and new queries
  use it. Legacy `ComputeHeightFieldAABB` and cooked source bytes stay unchanged.
  The C shape golden's single heightfield-AABB line checks conservative bounds
  within the error budget; every other C line and registry remains byte-exact.
- `MarshalHullData` and `MarshalMeshData` return owned cooked backend images for
  versioned artifacts. They accept only geometry built by this package, and do
  not introduce a parser for untrusted images or a portable mesh format.
- Queries reject invalid/stale inputs and locked worlds. They read current poses,
  apply no filters and may inspect disabled-body geometry. They require the same
  external tick ownership as the rest of the library.

Feature identifiers are valid only for unchanged cooked geometry, not durable
content IDs. No new query is currently added to upstream record/replay: callers
must record their inputs/results separately when reproducing gameplay queries.
Flagged queries explicitly reject upstream record/replay because its format has
no fields for the extensions; default recorder bytes are preserved. Engine query
recording must retain the independent contracts. One-sided rotational queries,
triangle/triangle penetration, rounded generic convex penetration, snapshot query
views, solver qualification and cross-platform runtime execution remain separate
work. No solver parity or workload qualification is inferred from unit tests.

## Opt-in motion admission

`WorldID.StepMotion` subdivides a requested duration, predicts both moving bodies,
and uses the ordinary solver for every contact impulse. Admission recomputes
semi-implicit acceleration after reducing time and bounds near-contact predicted
free motion to one quarter of the speculative distance. It does not shrink this
time budget toward zero as penetration approaches its allowed bound: a finite
solver step is necessary for contact recovery. Actual-path and endpoint checks
still enforce the unchanged penetration bound. Its casts target geometric contact,
using contact distance only as an early stopping accuracy; unlike public motion
queries, they do not require intersection with an inflated tolerance band.
Actual solver trajectories are
checked afterward, including every convex/triangle leaf, so unpredicted joint or
gyroscopic motion cannot silently pass a new obstacle. Conservative unresolved
paths, excessive overlap and work caps fault the enclosing frame; this is not a
rollback or a guarantee that every motor command succeeds. Existing penetration
may recover within a monotonically decreasing initial depth bound. Kinematic
crush prevention remains a host transport policy; an unresolvable crush faults.

The default upstream `Step`, solver CCD, recording format and arithmetic remain
unchanged. This extension preserves external forces and torques over internal
microsteps, copies every microstep's body-origin poses/manifolds/events, and locks
the world during observation. Samples omit static bodies. Actual microstep and
candidate-pair counts are returned. Moving bounds use sweep-and-prune; dynamic
shapes query the existing static tree. Dense moving overlap can still be costly,
and these tests are not a capacity qualification.

For moving meshes, the opt-in path invalidates the upstream world-space triangle
cache before each solver step. Otherwise a stationary passenger can retain an
empty triangle cache while the mesh rotates into it. Motion admission is
two-sided, but the ordinary mesh solver still obeys authored triangle winding.
Dynamic concave geometry and arbitrary rotating-mesh equivalence remain
unqualified. Default settings use a 1 mm solver admission contact distance,
independent of caller-selected precise query tolerances and a separate 1 cm solver
penetration limit, 0.01 rad maximum rotation and a 2,048 microstep cap per call.
The admission distance exceeds a float32 position ULP at Golem's ±4,096 metre
envelope; that comparison alone is not a geometry-error qualification. It does
not inflate placement or geometric query results.

## Validation

Focused regression tests cover deep/initial overlaps, exact touching versus
positive gaps, rotated convexes, both pair orders, witnesses, mirrored triangle
seams, compound/material/feature identity, terrain holes and quantization,
buffer overflow, destroyed handles, invalid inputs, independent query masks,
backface casts and unchanged initial-overlap cast behavior. Motion fixtures cover
crossing dynamics, translating mesh push, a rotating mesh striking a stationary
passenger, exact force/gravity duration, sensor entry/exit retention, braking,
initial overlap recovery, crush failure, and explicit rejection of motor-driven
crossings hidden by initial contact with a different compound leaf. Full upstream tests
also check C golden compatibility and compile-time no-FMA behavior on amd64 and
arm64. Test logs, caches and temporary files belong under
`D:/Demiurgos/tests/physics-validation/box3d` (caches at its parent), outside Git.

## Scalar mesh transform specialization

The default scalar lane backend now transforms candidate mesh triangles directly
instead of copying four-lane vectors through each multiplication and addition.
The multiplication/addition parentheses, float32 rounding, inclusive culling
boundaries, triangle order, SAT-cache clearing and all manifold policies are
unchanged. This does not reuse stale manifolds, skip candidate geometry, change
solver slop, or alter the existing SIMD backend. Moving-mesh cache invalidation
remains the separate motion extension's responsibility.

`TestGolemMeshScalarMatchesWideTransforms` checks bit equality against the original
wide expression across 2,000 seeded input batches; the boundary/cache regression
checks inclusive contact culling and survivor order. The full upstream scalar
golden/determinism suite and no-FMA compile guard pass under Go 1.27.1. The existing
`GOEXPERIMENT=simd -tags box3d_simd` backend also passed its upstream suite before
this specialization; build experiments and target CPU requirements must remain
explicit in qualification manifests. No workload performance pass is implied.

## Query scratch and deepest-contact reduction

`ShapeID.ProxyPenetrationDeepest` returns the first deepest result in the existing
`ProxyPenetrations` traversal order, including exact touching and one-sided
backface behavior. It retains one result even when a dense mesh produces many
contacts; callers do not allocate and retry an all-contact buffer. Both APIs use
the same collision/manifold path. Convex proxy cooking and internal geometry
traversal can still allocate; this is not an allocation-free query guarantee.

Proxy penetration and precise world casts now pool per-call callback state,
clearing caller references before returning it. The cast leaf math, candidate
ordering, child/triangle tie ordering, callback clipping and failure semantics
are unchanged. Each nested or concurrent reader owns separate scratch state.
Regressions compare sphere/capsule/hull deepest results against overflowing
dense-mesh contact lists, exact touch/gap/penetration, backfaces, invalid inputs,
reentrant clipping callbacks and concurrent readers. Qualification must still
measure the complete current source; no timing or release pass follows from
these allocation changes alone.

## Parallel-joint warm-start hemisphere

The parallel constraint permits free twist around its frame Z axis. The solve
pass selects a consistent quaternion hemisphere, but the upstream prepare and
torque-reporting paths previously used the unflipped orientation. After free
twist passed pi, their Jacobians could reverse against the retained impulses and
inject large tilt velocities. The opt-in world policy
`SetConsistentParallelWarmStart(true)` makes `parallel_joint.go` use the solve
hemisphere for preparation and force reporting too. Golem enables it for every
world, including non-CCD worlds. Upstream defaults and C-reference recordings
remain unchanged. The policy is reset on world creation and lies outside the
upstream serialized layout; native recordings do not retain it. Golem portable
checkpoints recreate the policy with the adapter. This changes solver Jacobians,
without adding movement impulses or changing casts.

`TestGolemParallelFreeTwistKeepsWarmStartHemisphere` continuously twists native-Z
and rotated-Y constraints through several revolutions while applying a transverse
load, checking alignment and finite torque. Engine fixtures also verify that an
upright Y-axis constraint permits yaw while resisting tilt.

## Exact triangle-face touching in geometric queries

Solver-style reciprocal normalization could leave a flat triangle normal one
float32 ULP short of unit length, reporting positive penetration for an exactly
touching capsule. Query-only sphere/capsule face contacts now use a scaled cross
product, direct component division and local plane distance. Poses, geometry and
the calculation remain float32. No overlap tolerance, upstream cast or solver
manifold is changed. Pair queries also retain feature identity and witnesses for
touching contacts, instead of returning a classification with empty identity.
The same local triangle plane bounds approximate edge contacts, so a finite
triangle cannot report deeper penetration than its containing infinite plane.
This covers GJK edge witnesses on a coplanar mesh without treating small positive
penetrations as zero.

`TestGolemTriangleFaceDoesNotInventPlacementPenetration` reproduces the rejected
workload spawn on its authored mesh grid, then tests exact touch, positive gaps,
10 micrometre overlap, and the 0.1/1 mm reference thresholds for spheres/capsules.

## Motion cadence and constraint convergence

`StepMotion` defaults to a maximum 1/240-second internal step, four velocity
iterations and two relaxation iterations. The opt-in counts are bounded at 16;
ordinary `Step` keeps its upstream counts and golden behavior. Additional
constraint passes use the same time boundary and do not integrate positions or
forces again. Only the final relaxation pass accumulates its physical normal
impulse, including scalar, overflow and wide/SIMD kernels, so force reporting and
restitution do not multiply with the relaxation count. A rounding-sized time
remainder is integrated in the preceding admitted step instead of a nanosecond
step that would rescale cached impulses and interrupt sleeping.

A regression reproduces a falling jointed grid that previously spent thousands
of microsteps near 9.97mm penetration. It preserves the server's float32 substep
endpoint arithmetic; four identical literal deltas hid that failure. The fixed
admission keeps the existing 5mm predicted-motion cap, 240Hz maximum delta and
actual geometric validation. Work-limit errors retain the last limiting pair's
depth, bound and predicted closure for diagnosis; no query tolerance is changed.

Actual path validation refines ambiguous leaf intervals against fresh geometry,
with an explicit bounded refinement count (default 8,192). It does not treat a
stale projected normal as fresh penetration or collapse compound children into
a union MTD. The 10mm solver penetration policy has a separate 1-micrometre
float32 classification allowance; reported depths remain unchanged and no
query or placement threshold changes. Initial overlap may recover without
increasing its inherited bound. Exhaustion, real excess penetration and
unresolved motion still reject the whole frame.

Engine regressions cover a dense 5,000-triangle floor with rotating, falling,
jointed boxes at one/eight workers, a sleeping six-body mass-ratio stack,
restitution energy, 100:1 momentum transfer and complete manifold force. These
checks preserve the original contact tuning and do not qualify workload capacity.

## Complete motion contact intervals

`ShapeID.MotionIntervals` retains connected convex-leaf contact intervals, then
unions overlapping children/triangles without collapsing concave exit/re-entry.
Translation uses forward/reverse convex casts. Rotation recursively bounds both
separation and overlap: a support plane certifies separation, while an inscribed
ball in an inner Minkowski support hull certifies overlap. Boundary brackets
enclose geometric contact within the requested distance tolerance. The deepest
manifold point is never treated as a union MTD or an interior certificate.

The result reports caller-buffer truncation and bounded refinement work. More
than 16,384 retained leaf intervals or refinements fails explicitly; unsupported
triangle/triangle and one-sided rotating-triangle pairs do not become misses.
Regressions cover a thin sphere crossing, compound unions and re-entry, rotating
initial/final overlap with a separated middle, hull/hull and triangle/hull motion.
Engine integration replays swept sensor boundaries after the locked solver has
returned, before each retained endpoint, using only private pending contact
history. Accepted shutdown history is unchanged by rejected frames. Normal
sensor-free simulation retains its immediate observer path.

## Analytic precise sphere/triangle sweeps

The additive precise-cast path solves sphere/triangle face, finite edge-capsule
and vertex intersections directly. This avoids stalled GJK fractions on long
casts against real terrain vertices. It prefers the actual geometric contact;
the caller's distance band is only used when the exact cast misses. Tiny edges
that the upstream capsule-ray helper approximates remain on the generic precise
path, and upstream casts/solver semantics are unchanged.

Frozen float32 triangles and queries from the two-tile Unity reference capture
(source SHA-256 `1945a6b09cb71b9757ffe15f93aaf999489e9022f07c75ca4e835858cd575aae`)
are checked against independently calculated analytic roots. At +2,304m, the
remaining strict Unity-comparison discrepancies have Golem distance error below
7 micrometres and world-point error below 0.116mm. Unity's rounded world-witness
normal differs from the geometric face normal in two cases; one Unity distance
is 0.129mm from the independent analytic result. Preserve both comparison records
instead of changing geometric normals or relaxing query thresholds. These checks
do not establish full scene, platform or workload qualification.
