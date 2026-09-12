# Bounded World fast-path review

Reviewed frozen World SHA-256: `14bdf78a498cd145bfb3321e3fee4fffced9841a4ac71b496a2e2685130cbc1e`. Source snapshot and dependency hashes are recorded in `reports/world-fastpath-review-source.json`. The model's concurrent SplitMotion work was not treated as part of this World patch. No source was edited in root or the model workspace.

Outcome: one P2 issue was independently reproduced in the independent/partitioned transition and reported immediately. Root fixed that same issue during review. No additional defect was found in the bias fast path or owned rejected-candidate pose.

## P2 found and fixed: completed split state survived loss of joint eligibility

In the reviewed version, `stepFixed()` allowed independent preparation when no eligible joint and no pending split dt existed, but `#stepFixedImpl` only deleted `_splitMotion` and `surfaceMotion` when there was a connected pair switching to position-history. A former split pair that became ineligible retained its completed phase state.

Concrete diagnostic witness on unchanged World `14bdf78a...`:

1. Native affine-wall fixture at y=-.5 completes an accepted split step.
2. Disable its containment, set incoming wire vx=1, and call `stepFixed()` again.
3. The second call returns undefined, stepCount becomes 2, solver is `independent`, joint factor count is zero, and `lastStepResult` is null.
4. Nevertheless, `getStats().jointMotion` still reports the previous `split-physical-bias` mode, `certified:true`, physicalPasses=1, biasPasses=1, and historyCommits=1. Both old phase objects remain attached to the constraint.

There was also a mechanical consequence from the same cause. Start with a stress-free accepted split pair, add a third rod so no joint is eligible while the original containment stays enabled, prescribe wire vy=1, and set radialVelocityDamping=.5. Both this run and a fresh matching setup use the partitioned solver. The old `_splitMotion` makes the condition at reviewed World line 2354 skip the native containment velocity stabilizer:

| Partitioned setup | Final wire vy |
| --- | --- |
| Has previous completed split state | `[.9999999404, .9999999404, .9999999404]` |
| Fresh otherwise matching setup | `[.4999999702, .4999999702, .9999999404]` |

This is one lifecycle bug with diagnostic and mechanical effects, not two separate findings.

Root's follow-up World SHA `f7c76681cb275a2f994b6b4c0a33dc1029826bf652e5b19678642def772be3c5` was read after the fix. Before the early-sleep path it now determines the eligible joint, clears body phase-local motion, and removes `_splitMotion`/`surfaceMotion` from all constraints except the currently eligible split pair. This covers both reproduced paths. The accepted checkpoint remains separately retained in `_acceptedPhysicalMotion`. Root supplied a split→disabled independent→split regression; the fixed implementation was verified by inspection here rather than rerunning the entire root test suite.

## Bias pass-zero fast path

No additional issue found. The call order matters:

- `beginKirchhoffSplitBias` parks physical body multiplier arrays, boundary/fold/orientation/external-friction state, tool reaction state, and lumen reaction values before the phase-specific `begin*` calls. Consequently their resets apply to bias state, not the parked physical reactions.
- Hard orientation targets are prescribed and wall witnesses refreshed before the initial bias check.
- `buildKirchhoffCoupledFoldRows` establishes the fresh fold range, limits, rows, and timestep required by fold measurement. It does not apply a physical correction.
- The common measurement refreshes lumen geometry/normal gradients, raw gaps/contact motion, wall geometry, controls, sheaths, tool/release rows, endpoint/sweep witnesses, material residuals, fold director and positional-angle residuals, and compliant orientation residuals. Orientation measurement evaluates current poses directly and does not require a solve-row build first.
- Bias friction builders intentionally return empty physical-friction batches. This does not certify the parked physical friction early: after either the fast path or a bias solve, physical banks are restored and the full physical measurement runs again. Final velocity/history certification still follows.
- The fast path sets bias closure success only. It cannot replace `physicalAccepted`, override final material/contact gates, or itself commit physical history. A previously failed physical phase remains a failed transaction.

The initial bias measurement is not completely free of mutation: it refreshes witnesses, hints, contact metadata, and disposable measurement scratch. The bank split and subsequent final physical checks are what preserve the physical reaction/history contract. It would be inaccurate to describe it as a pure read of cached values.

## Rejected `candidatePose`

No issue found. The pose and motion arrays are copied before restoring the transaction; activeStart/activeEnd and body id are included. They are owned diagnostic data rather than aliases into body storage.

A tiny native probe forces physical rejection with incoming vx=2. The result contains candidate x coordinates `[-.9833333492, .0166666526, 1.0166666508]`, while the world has restored `[-1,0,1]`, the result is accepted=false, and historyCommits=0. Writing 999 into the returned candidate array does not change the body. The rollback therefore does not erase the rejected pose needed for analysis, and the diagnostic copy does not mutate restored mechanics.

Validation limits: root reported 36 targeted tests passing; that full batch was not rerun here. This review used source inspection and the small diagnostic/mechanical/candidate witnesses above. No large scene, new contact model, benchmark, or tolerance change was introduced.
