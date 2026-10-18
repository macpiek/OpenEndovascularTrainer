# Anatomy for captured regressions

`anatomy-2026-09-19.bin.gz` pins the two historical plain-aorta models used by:

| Regression | Variant | Reason |
| --- | --- | --- |
| `anatomy-wire-569.80-poor-prediction.json.gz` | `open` | Recorded before outlet closure |
| `closed-root-wire-840.40-friction-fallback.json.gz` | `closed` | Recorded with closed outlets, before subclavian deformation and surface repair |
| Segment crossing at 835.27 mm | `closed` | The original four sample clearances and intervening wall crossing belong to this surface |

The bundle preserves the original geometry coordinates, triangle order, and
collision fields. Replays store mesh-specific face indices and clearance
certificates: changing their mesh identity to the current anatomy would make
them invalid. The helper checks the collision asset SHA-256, its source STL
hash, and the exact transformed/indexed mesh identity before returning a field.
The ordinary replay loader still rejects mismatched geometry.

The closed model comes from commit
`e1fc5c46d303ed223efe8dc50bdc7861b6e4c283`; the open model comes from its parent.
Outlet closure only appended triangles, so the bundle stores one vertex table
and corner sequence, expanding the appropriate prefix for each variant. The
two collision fields share one complete asset and a lossless copy/literal delta.
Gzip compresses the bundle to approximately 40 MiB. This avoids checking in two
full STL/collision pairs (approximately 220 MiB uncompressed).

Regenerate with a full local Git history:

```sh
node scripts/physics/build-replay-anatomy-fixture.mjs
node --test --test-concurrency=1 tests/kirchhoffSharedAxisCertifiedSamples.test.js tests/kirchhoffSharedAxisClosedRootFallback.test.js tests/kirchhoffSharedAxisSegmentContact.test.js
```

Running tests requires only the committed bundle, not Git history or network
access. Modern-anatomy integration tests, including the 330 mm guidewire
regression, continue to use `loadCoupledRuntimeAnatomy` and the shipped assets.
