# Golem physics fork

Base: `gataengine/box3d-go` commit `2a00e926a8d92ae1d57a2e425aadebc4610b0ed5`.
Underlying Box3D source: `erincatto/box3d` commit `efca0c184d40fd6a82016983f1601c4d23965867`.
Original MIT attribution and port notes remain intact. This fork uses Go 1.27.1
and float32 positions, geometry and simulation. It does not change default cast semantics
or implement character pushing through manual impulses.

## Unreduced hulls, widened topology and small mesh triangles

`CreateHullUnreduced` accepts raw point clouds larger than the native output
limits, including repeated and interior points, but fails when construction
still has outside points after exhausting its budget. It uses the existing
float32 hull tolerances and topology limits; it does not silently return the
budget-reduced hull that `CreateHull` intentionally permits. The original API
and its intentional vertex-budget simplification remain available.

Hull topology and SAT feature indices now use uint16, with bounded limits of
256 vertices, 512 faces and 768 full edges. Hull native image version changes
from `0x6B1E39D4A87C25F3` to `0x6B1E39D4A87C25F4`; native world snapshot version
is 12. Old native images require recooking; old snapshots are rejected. Compound
version derives from the new hull version. Float32 geometry is unchanged.
Simplex vertex caches remain uint8 because the maximum vertex index is 255.
Support queries above 128 points use an exact scalar scan, preserving the
original mantissa-index fast path for smaller hulls without reducing precision.

Contact feature pairs hold uint16 indices, but `EncodeFeaturePair` preserves all
legacy uint32 identities and contact/query layouts. Two 14-bit indices and two
one-bit owners occupy documented disjoint fields; bits 15 and 31 are reserved,
so the invalid all-ones sentinel cannot alias a valid pair. Named encode/decode
helpers reject out-of-range inputs and regression tests exercise legacy values,
high indices, roundtrips, injectivity and sentinel rejection.

`CreateMeshPreserveSmallTriangles` retains every float32 nonzero-area triangle
instead of applying the default `0.01 * LinearSlop()^2` area cutoff. Zero-area
faces retain native omission/counting behavior. The adapter still rejects any
triangle-count change, so no authored face is silently lost. Original mesh
cooking defaults, mesh image layout and vertex-welding behavior are unchanged.

Focused regression evidence is under `cook-support-20261112-01` in the external
validation workspace. The actual deco-95 capture retains 63,182 triangles,
including 84 nonzero faces discarded by the default cutoff. The same captured
priority sources now cook unreduced hulls up to 167 vertices, 284 faces and 894
half-edges. Prior failures remain preserved. Representative topology, contact,
query and independent native-image reference checks cover this format change;
they do not qualify the full content catalog or workload capacity.

## Retirement of global physical motion refinement

The unshipped `WorldID.StepMotion` API, its subdivision settings/results/errors,
global admission predictor, post-solve path certification and universal
penetration guard have been removed. Earlier ledger entries describing that
experiment are historical records, not current solver behavior. Exact source,
held working changes and their hashes were preserved before removal under
`motion-retirement-20261112-01` in the external validation workspace.

`StepObserved` retains exactly one native step, owned body/contact/event samples,
observer locking and the existing solver-iteration override. Shared observation
types, event-copy scratch, iteration access and moving-mesh cache invalidation
now live in focused files. Observation itself performs no additional candidate
search; enabled native CCD policies can perform their ordinary candidate casts.
`CastMotion`, `MotionBounds`, `MotionIntervals` and their bounded local geometric
search remain active public query and swept-sensor functionality. They do not
subdivide or re-run physical simulation.

Physical regressions migrate to fixed 80Hz native steps: braking near an obstacle,
initial penetration recovery, resting hulls, feasible rising support under a
ceiling, tiny bullets and the portable compound's flattened floor/wall leaves.
Migration checks over committed `0a96446` expose motor-induced motion through a
stationary kinematic mesh and repeated motor motion through the flattened wall
after its first TOI clip. Both remain explicit required native coverage cases
until native scheduling/response is corrected.
The direct baked-native-compound floor/wall failure is separately preserved as
an upstream capability diagnostic; portable compounds use flattened leaves and
passing that path does not qualify the direct baked representation.

The old CCD-disabled compound case retains its disabled-CCD meaning. The old
kinematic compression fixture eventually leaves less room than its passenger's
diameter; it requires host transport clipping/rejection, not an impossible native
nonpenetration guarantee. Its exact fixture and diagnostic remain preserved.
Static low-ceiling character sweeps are only partial host-policy coverage;
moving-support passenger transport requires its own integration regression.

## Immutable kinematic targets and native sensor hits

Enabled speculative-CCD worlds retain kinematic shape metadata, full swept
bounds and trajectories before parallel body finalization. The native post-solve
continuous pass queries this immutable tree for every fast dynamic body, including
motion introduced by a motor or contact solve after initial prediction. A target's
bullet flag cannot exclude its fixed kinematic trajectory. Upstream dynamic-bullet
ordering remains unchanged. This is coverage for detection; it does not by itself
repair repeated motor contact response.

Native swept sensor admission now checks the TOI state as well as its fraction.
A mesh miss with a default zero fraction must not synthesize an entry. Valid
crossings and initial overlap retain their native classifications.

Focused geometry, scalar/SIMD and race evidence is preserved under
`motion-retirement-20261112-01` in the external validation workspace, including
zero/+2304m target poses, bullet policies and one/eight-worker sensor cases.
The native upstream core suite also passes with the sensor-state correction;
the separately recorded required motor-continuation failures remain open.

## Opt-in native speculative CCD

`WorldID.SetSpeculativeCCD` adds targeted moving-pair prediction to native
stepping. It is disabled by default. The engine enables it with its continuous
world policy; ordinary upstream worlds keep their native scheduling. The normal
velocity and position integrators predict unconstrained motion on copies,
including forces, damping, rotation, locks and speed limits. Shapes whose motion
exceeds the native safety-factor extent threshold query a temporary swept-bounds
tree; moving meshes always participate because their inner extent cannot safely
represent a thin surface. Candidate pairs are deduplicated and sorted before
the native time-of-impact call. Sensors and collision filters retain their roles.

An upcoming hit supplies a native contact manifold to the normal constraint
solver. Future contact generation uses nearby pair coordinates and private
future AABBs, retaining mesh features/materials and avoiding distant-world COM
rounding. Contact anchors are returned to their start orientations and carry
the initial separation along the predicted normal. Response, friction and
equal/opposite impulses remain solver-owned. This adds no adaptive time steps,
global penetration certificate or manually applied push impulse. Native static
sweeps remain separate. Portable compounds use individual native convex leaves;
direct native CompoundShape predictive pairs keep upstream coverage.

Nonconvergent TOI, absent manifolds and incompatible rotating predictive planes
are explicit errors. `StepObserved` still returns the attempted step's owned
body/contact sample before propagating the error; a transactional engine must
fault the complete frame. Raw Step callers must check `SpeculativeCCDError`.
Pre-solve rejection is a valid contact filter, and its callback receives world
coordinates. Predictive manifolds and native pair-begin notifications do not
establish geometric touching or a gameplay hit. The adapter filters separating
zero-impulse contacts and observes sensors independently.

`SpeculativeCCDStats` counts unique swept candidates, filtered moving-pair TOI
calls and predictive manifolds for the attempted step; it excludes all built-in
post-solve continuous sweeps. Upstream recording refuses this mode because the recording
format does not encode the policy. Engine portable checkpoints reconstruct it
from their versioned world configuration instead.

Regressions cover fast crossing spheres, translating/rotating meshes, sleeping
targets, common motion, perpendicular crossing paths, filters, pre-solve
rejection, contact reuse, material/triangle identity, nonzero local COM, callback
coordinates at 0 and +/-2304m, force-duration parity and worker repeatability.
Focused immutable-source scalar, SIMD and race checks are retained under
`speculative-geometry-20261112-01` in the external validation workspace. These
checks do not qualify the full workload or every solver-induced acceleration.

## Native post-TOI contact response

The pinned upstream C `src/solver.c` clips a time-of-impact pose but retains its
impact velocity for the next contact solve. The pinned `contact_solver.c` and
`contact_solver_wide.inl` do not distinguish that handoff from an ordinary soft
overlap constraint. A 10mm-radius sphere moving at 400m/s through a 20mm wall is
detected and clipped to X=-0.015m, then the following soft solve integrates to
X=+0.071274m before relaxation removes its velocity. This behavior is inherited
from upstream, not a missing Go translation.

Native continuous stepping now uses a hard normal-velocity constraint for
closing contact points whose participating body had a TOI in the previous step.
The existing bounded recovery bias, accumulated impulses, friction, restitution
and equal/opposite dynamic-body response remain in the ordinary solver. The
marker is sampled during preparation, before integration clears transient body
flags. Separated speculative contacts retain their original bias. Scalar
overflow, four-wide and generated eight-wide kernels implement the same rule.
There are no extra steps, swept-pair scans, gameplay impulses or query tolerance
changes. Non-TOI contacts retain their original arithmetic.

Regressions cover the small fast sphere at 240/80Hz and one/eight workers,
restitution/friction, dynamic-target momentum at 1:1/1:10/1:100 mass ratios,
simultaneous floor support, overflow constraints, ordinary contact parity and
worker-repeatable falling ragdolls. Old pinned-C golden trajectories change at
TOI contacts. `testdata/golem_post_toi` contains independently generated corrected
C references, the exact C patch, compiler/options/hashes and baseline comparison
provenance; original upstream fixtures remain unchanged. Before applying the
patch, the same compiler reproduces original text/operation/registry references
and determinism hashes. Corrected C one/four-worker output agrees with Go;
raw recording seed-snapshot padding is excluded under the original fixture's
comparison contract. Production remains `CGO_ENABLED=0`; C is offline evidence.

This fixes the demonstrated free-impact handoff, not arbitrary multi-contact
crushing. A fast small bullet hitting a thin dynamic plate backed by a static
wall still exposes insufficient native one-pass contact propagation. The failing
fixture and iteration diagnostic are retained under the external validation
workspace; no tolerance or global-step change masks that qualification gap.

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

## Observed native stepping

`WorldID.StepObserved` performs exactly one native `Step(dt, 1)` and copies body
paths, contact manifolds and events for a locked, read-only observer. Constraint
passes are explicit and default to the upstream 1/1 values. It does not perform
global motion admission, geometric path certification, adaptive subdivision or
an additional penetration check. Native world CCD remains independent of this
observation seam. Observer failure returns after simulation; transactional hosts
must reject the whole frame without rewinding an individual body. Existing
moving-mesh cache invalidation remains necessary before collision detection.

Focused tests compare the observed and ordinary native paths, force/torque and
elapsed time, retain owned contact data, and verify invalid-input rejection and
post-step observer failure. This observation seam is not a qualification claim
for native CCD coverage or workload capacity.

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

Interval enumeration certifies all swept-bounds leaf candidates directly, rather
than requiring an earlier conservative-advancement hit. That redundant cast could
approach an exact rotational endpoint tangency asymptotically. Public additive
motion casts now use a bounded distance-band search only after rotational
advancement fails to converge. This search distinguishes the positive query band
from geometric overlap and never reports clear on unresolved band evidence.
Solver admission retains its original advancement policy. Adjacent numerical contact fragments
merge only when the entire intervening span is certified within the original
distance tolerance; no tolerance increase or time epsilon is used. Regression
cases cover both tangent endpoints, separated near misses and float32 world
offsets of 0 and ±2,304m. Translation and upstream default casts are unchanged.

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

## Native rotational and relative mesh CCD candidates

Native `Step` now includes intermediate rotational geometry in its CCD candidate
bounds. Endpoint AABBs can miss a long body passing an obstacle between its two
endpoint orientations. Rotating geometry is conservatively enclosed by its
centre-of-mass segment and geometry radius; this changes candidate traversal,
not the native TOI algorithm, contact slop or physical geometry. Float32 bounds
include an arithmetic allowance and world-coordinate conversion rounds outward.

Mesh TOI traversal uses both bodies' relative translation and rotation. The mesh
endpoint transform is used for its endpoint-side test, and an endpoint-only
plane rejection is disabled when either orientation changes. Initial backface
policy remains unchanged. Static mesh versus nonrotating shape retains the
original candidate-bound arithmetic. Per-triangle native `TimeOfImpact` still
decides whether a candidate is hit. These corrections can change native CCD
outcomes for previously missed rotational or moving-mesh candidates; they do
not add motion microsteps, post-solve guards or manual response impulses.

Regressions cover a native rotating rod whose impact lies outside endpoint
bounds at 0, +768 and -2304 metres, translating/rotating mesh TOI, common-motion
and backface misses, and sampled intermediate bounds for sphere/capsule/hull
sweeps with nonzero centres of mass. Frozen native-step diagnostics separately
retain translating/rotating kinematic meshes passing passive bodies: those
require CCD scheduling and solver-response coverage beyond this geometry patch.
Dynamic concave simulation remains outside the supported contract. No workload,
platform or complete moving-mesh qualification follows from these regressions.

`testdata/golem_native_sweep` retains independently generated pinned-C reference
outputs after the documented post-TOI contact correction and candidate-bounds
patch. Only affected world/recording/replay/shape and deterministic expectations
select this reference; original upstream and contact-only fixtures remain
unchanged. The C text results agree at one/four workers. This is an explicit
corrected-source comparison, not a claim that unmodified upstream matches.
