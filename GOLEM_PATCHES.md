# Golem physics fork

Base: `gataengine/box3d-go` commit `2a00e926a8d92ae1d57a2e425aadebc4610b0ed5`.
Underlying Box3D source: `erincatto/box3d` commit `efca0c184d40fd6a82016983f1601c4d23965867`.
Original MIT attribution and port notes remain intact. This fork uses Go 1.27.1
and float32 positions, geometry and simulation. It does not change default cast semantics
or implement character pushing through manual impulses.

## Additive query extensions

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
recording must retain the independent contracts. Moving triangle-mesh CCD,
triangle/triangle penetration, rounded generic convex penetration, snapshot query
views, solver qualification and cross-platform runtime execution remain separate
work. No solver parity or workload qualification is inferred from unit tests.

## Validation

Focused regression tests cover deep/initial overlaps, exact touching versus
positive gaps, rotated convexes, both pair orders, witnesses, mirrored triangle
seams, compound/material/feature identity, terrain holes and quantization,
buffer overflow, destroyed handles, invalid inputs, independent query masks,
backface casts and unchanged initial-overlap cast behavior. Full upstream tests
also check C golden compatibility and compile-time no-FMA behavior on amd64 and
arm64. Test logs, caches and temporary files belong under
`D:/Demiurgos/tests/physics-validation/box3d` (caches at its parent), outside Git.
