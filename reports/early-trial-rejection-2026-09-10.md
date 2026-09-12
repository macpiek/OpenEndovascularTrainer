# Early rejection of nonlinear trials — 2026-09-10

The default joint-active-coulomb solver can now stop a trial measurement once its boundary error proves that neither merit decrease nor final convergence is possible. The geometry refresh and any positional cone repair still run before this decision. Candidates that might be accepted always receive the complete original measurement.

## Acceptance proof

Let `b` be the freshly computed boundary residual, `t` the containment tolerance, and `M = previousMerit * (1 - 1e-4 * scale)`. The final merit includes `b/t` in its maximum. If both `b > t` and `b/t > M`, the candidate cannot pass either final convergence or merit acceptance. Strict comparisons preserve equality cases. Nonfinite inputs and zero tolerances take the complete path.

The shortcut is disabled on the unconditional first pass, the last trial, split-motion cone filters, wall-witness discovery and custom kernels without explicit opt-in. The last failed trial is still fully measured, preserving complete failure diagnostics. A diagnostic shadow hook can finish the measurement once and verify each certificate against the original gates.

## Paired measurement

2386 fixed steps at 120 Hz, actual anatomy, Berenstein catheter advance/hold/withdraw with shaft stiffness 25× and tip 5×. Execution order alternates at each step. Rendering is omitted; these are not live-browser FPS measurements.

| Metric | Complete measurement | Early rejection | Change |
|---|---:|---:|---:|
| Full evaluations per step | 30.252 | 13.949 | -53.9% |
| Constraint evaluation (ms/step) | 5.340 | 4.208 | -21.2% |
| Whole physics step (ms/step) | 20.642 | 19.000 | -8.0% |

Early rejection certificates: **38899**. Trial counts, accepted/rejected scale bins and pose fingerprints match after every paired step. Both runs retain 800 unconverged steps.

The earlier sequential comparison reduced evaluation cost by 16.3% but increased total time by 2.6%; untouched stages also varied. Both runs are retained in the JSON report. The alternating paired measurement reduces this order/background-load bias; timing gains still depend on workload and system load.

## Verification

- 124 focused tests and production build passed.
- Unit checks cover tolerance/merit equality and nonfinite fallback.
- Shadow evaluation proves early certificates against complete residuals.
- Real coupled mechanics with a deliberately unproductive shared translation exercise repeated rejection, exact pose/force/contact-history comparison and complete failure diagnostics.
- Converging corrections retain complete measurements.
- Wall-witness tests verify that discovery remains on the full path, even with early rejection requested.

## Reproduction

```sh
node scripts/physics/benchmark-early-rejection.mjs 2386 /tmp/paired.json
OET_EARLY_TRIAL_REJECTION=0 node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/reference.json
node scripts/physics/benchmark-trial-state.mjs 2386 /tmp/early.json
```

[Full profiles and sequential traces](early-trial-rejection-2026-09-10.json).
