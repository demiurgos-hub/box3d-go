# Widened hull image references

These two outputs were generated independently by the pinned C source
`efca0c184d40fd6a82016983f1601c4d23965867`, with corrections applied in order:
`../golem_post_toi/upstream.patch`, `../golem_native_sweep/geometry.patch`, then
`topology.patch` here. The geometry and topology patches use zero context and
require `git apply --unidiff-zero`.
Original upstream and earlier corrected fixtures remain unchanged.

The final patch widens bounded hull/SAT topology and feature indices, preserves
legacy uint32 contact IDs, changes HullVersion to `0x6B1E39D4A87C25F4`, and adjusts
box padding and large-hull support. These are corrected-source references, not
claims of unchanged upstream bytes. The existing `hull_golden_gen.c` and
`geometry_golden_gen.c` generated the outputs; the only generator edit disabled
stdout buffering. Clang used `-ffp-contract=off` and `BOX3D_DISABLE_SIMD`.
`provenance.json` records compiler, commands, patch and output hashes. Windows C
stdout CRLF is normalized to LF for the checked-in text; both hashes are recorded.

Recording/replay fixtures additionally apply `topology-recording.patch` instead
of `topology.patch`; it includes the same topology changes plus native snapshot
version 12. `recording-provenance.json` records that independent C build and its
three existing generators. Their embedded geometry registry now uses the new
hull/compound images; earlier recordings remain preserved and incompatible
native snapshot versions are explicitly rejected.

The oracle builds and intermediate attempts are preserved externally under
`cook-support-20261112-01`. This migration checks native hull/compound images
and the existing geometric query comparisons. It does not qualify solver
robustness, full catalog parity, performance, or platform release dependencies.
