# Candidate-local geometry and friction reuse

Point 2, on top of the compact contact snapshots from point 1. Default position-history physics now shares:

1. The smooth material-side violation/witness computed while collecting contacts with the subsequent nonlinear containment measurement. It includes samples even when they do not emit an active contact record. Portal/sliding-mouth checks still execute separately. The bank is cleared at the start of every measurement and recollected after any cone correction; it never certifies a later pose or rollback.
2. Previous node positions across contacts in one synchronous surface-friction batch. An epoch invalidates the weak body cache on every subsequent build/measure, just as for the existing quaternion cache. Normal loads, material friction coefficients and tangential history are read afresh.
3. The just-evaluated friction geometry/kinematics when preparing an immediate final-load cone correction. Only its missing Jacobian rows are materialized. The owner makes this call synchronously without intervening pose/history/feature edits. Promotion checks batch identity, timestep, contact identity/kind/segment, normal, normal/tangent load and friction coefficients, and refuses a used/frozen build batch. After applying a correction, all candidate geometry is collected again.

No changes to physical equations, timestep, tolerances, iteration budgets, material properties or friction coefficients. There is no cross-frame or cross-trial geometry cache. Split physical/bias geometry retains the prior separate collection path.

## Reproduction

```
OET_VERIFY_WITHDRAWAL=1 OET_REUSE_CANDIDATE_EVALUATION=0 node scripts/physics/profile-overlap-309.mjs /tmp/before.json
OET_VERIFY_WITHDRAWAL=1 node scripts/physics/profile-overlap-309.mjs /tmp/after.json
```

Same real anatomy and defaults, guidewire 309 mm, catheter 100 mm, 120 Hz. Compact trial capture remains enabled on BOTH sides. Statistics cover the final 47 feed/withdraw steps per interval and 60 hold steps. This is Node physics timing, not a rendered FPS measurement or the live browser's exact contact history. Hashing/report serialization is outside measured step time.

| Coupled phase | Residual measurement before → after | Full step before → after |
|---|---:|---:|
| Feed to 100 mm | 22.48 → 18.17 ms (19.2% less) | 50.55 → 46.23 ms (8.6% less) |
| First hold steps | 21.52 → 17.32 ms | 55.12 → 50.53 ms |
| Withdraw to 80 mm | 17.52 → 14.46 ms | 40.55 → 38.56 ms |
| Refeed to 100 mm | 23.56 → 19.00 ms | 52.48 → 49.59 ms |

Feed to 100 mm reuses an average 218 side-sample evaluations and 5.64 friction batches per step. These counters represent work avoided within residual measurement, not fewer physics iterations. The small-depth full-step times remain noisy (the final 50 mm sample was slower despite a lower residual-measurement cost). No universal FPS or real-time claim is made.

A reverse-order timing pair before the final extra promotion guards measured 50.55 → 48.37 ms for feed and 55.12 → 48.79 ms for hold. An earlier pair measured 49.36 → 45.27 ms for feed. Timing varies with JIT/system activity; comparisons are sequential rather than simultaneous CPU loads.

## Verification

- 2578 steps per trajectory: wire-only, catheter-only and coupled feed/hold/withdraw/refeed.
- No differences in pose fingerprints, SHA-256 hashes of all body typed arrays and manifold scalar/typed-array fields, or outer/trial/backtrack/factorization/failure decisions.
- One pre-existing nonlinear closure failure during coupled withdrawal occurs identically in both variants. All feed, hold and refeed steps retain their prior convergence behavior.
- 90 focused tests passed, including surface friction/KKT, cone corrections, snapshot/rollback, geometry and early rejection. The final additional load/feature guards passed the 20 directly affected friction-row and cone-repair tests.
- Tests exercise previous-pose edits, topology changes, full rollback, stale loads/features, invalid previous positions, exact promoted-vs-fresh Jacobians/corrections and reciprocal reactions.
- Production build passed. The existing bundle-size advisory remains.

`comparison.json` and compressed trajectories contain the final verification. `reverse-order-summary.json` records the additional timing pair. Remaining total cost still substantially exceeds the 8.33 ms budget at 120 Hz.
