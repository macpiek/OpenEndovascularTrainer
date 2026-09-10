# Trial-state save/restore optimization — 2026-09-10

Dense ordinary arrays and ordinary writable contact records now retain reusable value buffers. Accessors, sparse arrays, hidden fields and filtered owner roots keep the descriptor-aware path. Newly added properties and changed graph references remain checked. Boundary and wall-witness gradient records now have sealed field layouts so their keys need not be rediscovered. No physical tolerance, contact equation or trial acceptance rule changed.

## Measurement

2386 fixed steps at 120 Hz, actual anatomy, Berenstein catheter advance/hold/withdraw, shaft stiffness 25× and tip 5×. This is a solver-only workload, not the current interactive browser pose. Browser activity and system load affect absolute timings.

| Stage | Before mean (ms/step) | After mean (ms/step) | Change |
|---|---:|---:|---:|
| Save | 6.855 | 6.416 | -6.4% |
| Restore | 8.959 | 7.065 | -21.1% |
| Whole physics step | 30.268 | 28.672 | -5.3% |
| Save + restore | 15.814 | 13.481 | -14.8% |

## Verification

- Matching position, velocity and orientation fingerprints for both bodies after every one of the 2386 steps.
- Identical accepted/rejected trial bins, rejected-contact categories and 800 unconverged steps.
- 73 focused tests passed, including exact rollback bytes, contact/friction ownership, hidden/accessor fields, sparse/dense transitions, shared-tool runtime and wall constraints.
- Production build passed.
- The existing convergence failures remain; this change reduces bookkeeping cost without changing mechanics.

## Reproduction

Before editing, preserve a source tree (including package.json and access to node_modules). Run these sequentially under similar load:

```sh
node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/before.json /path/to/saved-source-tree
node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/after.json
```

[Full profiles and per-step fingerprints](trial-state-optimization-2026-09-10.json).
