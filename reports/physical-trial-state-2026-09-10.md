# Reduced physical trial state — 2026-09-10

The built-in position-history solver now opts into a narrower apply/measure snapshot. Custom kernels without this explicit contract, split-motion solvers and whole-candidate friction retries retain the complete snapshot.

## State ownership audit

| Data | Trial treatment |
|---|---|
| Positions, orientations, material multipliers, wall/tool projections and contact history | Still copied and restored |
| Mass/inertia arrays, radii, rest lengths/rotations, material compliance, material labels and prescribed position targets | Read-only inputs prepared before apply/measure; omitted from copying |
| Fold row lambda, active hints, per-node lambda arrays and pending flag | Retained in a separate reusable force journal |
| Fold gradients, row indices, branch axes and limit configuration | Frozen build; not recopied |
| Fold frames, geometry and measurement output | Scratch overwritten before its next use; not restored |
| Unknown numeric body fields and other contact state | Conservative complete snapshot remains |

Frozen fold owner/version/step, row identities and multiplier-storage identities are validated before any rollback writes. The force journal also excludes aliases through the combined boundary rows. No physical law, tolerance, collision query or trial acceptance rule changed.

## Measurement

2386 fixed steps at 120 Hz, actual anatomy, Berenstein catheter advance/hold/withdraw, shaft stiffness 25× and tip 5×. Solver-only measurements; not the current interactive browser pose. Runs are sequential and absolute timing depends on background load.

| Stage | Before mean (ms/step) | After mean (ms/step) | Change |
|---|---:|---:|---:|
| Save | 4.030 | 2.573 | -36.2% |
| Restore | 4.660 | 2.622 | -43.7% |
| Whole physics step | 20.380 | 17.294 | -15.1% |
| Save + restore | 8.690 | 5.194 | -40.2% |

In the final measured step, the general snapshot traversed at most 1024 objects instead of 2199. Copied typed-buffer bytes (including the new fold force journal) fell from 987569 to 873741. These are final-step maxima, not whole-run averages.

## Verification

- Matching pose/velocity/orientation fingerprints for both tools after all 2386 steps.
- Identical accepted/rejected trial bins, maximum geometry errors and 800 unconverged steps.
- A real coupled apply/measure audit compares physical rollback with a complete snapshot, including contact history and omitted read-only input bytes.
- Wall-witness, radius-torque and static-to-kinetic retry tests pass with both complete and physical snapshots.
- 120 focused tests and production build passed.
- Existing convergence failures remain; the change reduces bookkeeping cost without altering their acceptance.

## Reproduction

Save the reference source tree (src, package.json and access to node_modules), then run sequentially:

```sh
node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/before.json /path/to/reference-tree
node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/after.json
```

Reference source commit: `fbab7afcca0edd4f7faf3bfaed87876d864f37b2`.

[Full profiles and per-step fingerprints](physical-trial-state-2026-09-10.json).
