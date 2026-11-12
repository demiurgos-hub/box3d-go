# Golem physics fork

Base: `gataengine/box3d-go` commit `2a00e926a8d92ae1d57a2e425aadebc4610b0ed5`.
Underlying Box3D source: `erincatto/box3d` commit `efca0c184d40fd6a82016983f1601c4d23965867`.
Original MIT attribution and port notes remain intact. This fork uses Go 1.27.1
and float32 positions, geometry and simulation. It does not change default cast semantics
or implement character pushing through manual impulses.

## Additive query extensions

- `ShapeID.CastMotion` reads both body trajectories with linear origins and
  shortest-arc slerped rotations. Full rotational bounds and existing leaf BVHs
  retain child/triangle identity. Conservative advancement uses support-plane
  lower bounds and reports nonconvergence instead of clearance. Exact initial
  touching/penetration remains distinct from its positive tolerance band.
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
semi-implicit acceleration after reducing time. Actual solver trajectories are
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
unqualified. Default settings use a 1e-5 m query tolerance, a separate 1 cm solver
penetration limit, 0.01 rad maximum rotation and a 2,048 microstep cap per call.

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
