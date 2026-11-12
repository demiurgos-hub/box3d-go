# Native CCD candidate reference

These fixtures were generated independently by the C Box3D sources pinned at
`efca0c184d40fd6a82016983f1601c4d23965867`, with two explicit corrections applied
in order: `../golem_post_toi/upstream.patch`, then `geometry.patch` here. They are
corrected-source references, not unchanged upstream outputs. All original and
contact-only fixtures remain in their existing locations.

`geometry.patch` matches the Go rotational candidate bounds and relative mesh
traversal change. C generation uses Clang, `-ffp-contract=off`,
`BOX3D_DISABLE_SIMD`, and the existing `*_golden_gen.c` generators. The expanded
`determinism_reference.c` also records the wave-pile scenario. `provenance.json`
records exact source hashes, compiler, commands and output hashes. Text outputs
matched between one and four C workers; recording headers retain worker setup.

These files validate the native geometry/contact changes only. They do not
establish full moving-body CCD scheduling, workload capacity, platform release
qualification or a native runtime dependency.
