# Exact condensation of compliant contact reactions — experimental

The opt-in `condenseCompliantContacts` reduces each fixed active-set matrix. It preserves the existing spring contact law and force/complementarity tolerances. It is **not enabled by the app factory**, and the complete-cycle 60 Hz objective remains unmet.

For a compliant active row, `J dx + C dλ = -(g + C λ)`. Its stationarity column is `B = -Jᵀ + frictionColumn`. Eliminate its dual unknown by adding `-B J/C` to the primal block and `-B(g+Cλ)/C` to the residual. Recover `dλ` afterwards. All geometric Hessians, fixed masks, nonsymmetric friction columns, hard length/bend/sheath rows, active-set transitions and numerical fallback remain in place. Only rows whose condensed support fits the existing primal band are reduced.

The reduced direction must pass a separate check of the **original uncondensed equations**. Otherwise the original full working-set LU is retried and both attempts are counted. Tests include near-rigid compliance `1e-18`, which deliberately triggers this fallback rather than accepting cancellation error. Condensation is algebraically equivalent for a fixed working set; floating-point arithmetic and nonlinear trajectories are not bit-identical.

## Complete cycle

5757 movement steps completed: wire 0→1000 mm, catheter 0→1000 mm, catheter withdrawal, wire withdrawal. dt=1/60, original feed speeds, closed anatomy, continuous axis guard. The candidate uses `wallCompliance=1e-6`, `batchActivationSize=64`, `simultaneousContactRelease=true`, plus condensation.

| Node timing/work | Exact app reference | Compliant + batch release | Add condensation |
|---|---:|---:|---:|
| Mean ms | 33.915 | 30.980 | 32.477 |
| P95 ms | 93.964 | 75.512 | 67.554 |
| Maximum ms | 1612.864 | 2583.079 | 8979.022 |
| Steps over 16.67 ms | 4006 | 3959 | 3988 |
| LU factorizations | 87269 | 69607 | 78008 |
| Newton iterations | 20741 | 23312 | 24690 |
| Full assemblies | 32058 | 34686 | 36503 |
| Residual assemblies | 40185 | 45143 | 49991 |

Lower P95 does not outweigh the mean/tail regression; this is not a default performance upgrade. Timing includes retries/subdivisions but excludes browser rendering/UI. No new browser 60 Hz claim.

## Worst-step diagnosis

Worst index 1017, wire insertion 745.80 mm: 586 Newton iterations / 5576 LU / 8.979 s. Replaying its incoming state reproduced exactly the iteration and factor counts. The observed Newton directions account for 1218 LU; **4358 (78.2%) are trial constraint corrections**. There were 1218 condensation attempts, 72,031 eliminated row occurrences, and zero condensation fallbacks. Thus the worst case is not explained by failed reduced-system accuracy checks.

In the complete run, this step spent 4.842 s assembling equations, including 4.252 s on residual assemblies, and 3.521 s on linear work, including 2.549 s on projections. The replay observer adds overhead; its separate wall time is not substituted for benchmark timing.

The earlier difficult incoming states preserve 13 Newton / 144 LU (reference index 1144) and 121 / 1454 (compliant index 1464). Peak packed matrix entries change from 40,399 to 31,071 and 51,781 to 35,464 respectively. Projection matrices are still present in these peaks.

## Physical evidence and limits

All accepted states finite, maximum original force/friction residual 9.9983e-5, maximum relative edge-length error 3.481e-8. Sampled physical indentation 0.04179 mm; the compliant residual does not hide it. Peak speed 277.08 mm/s. Independent checks on 96 snapshots: zero axis-crossing frames, zero far-outside warnings across 51,978 lumen samples, continuous capsule overlap up to 0.10590 mm. These are snapshot audits, not temporal collision proof.

Shape RMS relative to the exact app reference: 2.34 / 4.28 / 7.18 / 6.33 mm by phase; maximum local differences 32.42 / 11.58 / 16.65 / 24.27 mm. Relative to the otherwise identical compliant model, RMS 2.13 / 1.51 / 0.074 / 0.257 mm; maximum 25.26 / 6.55 / 0.495 / 3.23 mm. Full real-time visual verification is still outstanding.

## Validation

Five new tests cover analytic parallel load sharing, nonsymmetric tangent/friction, geometric Hessians, hard constraints and fixed support, original-equation verification, numerical fallback accounting, and nonlocal support exclusion. Across the broader selected suite, **91 of 92 tests passed**. The existing `anatomy-berenstein-feed-312.87-projection` fixture fails its requirement to exercise a nonlinear trial correction. The identical failure was reproduced with both changed runtime files restored to their pre-condensation contents; current sources were then restored byte-for-byte. The assertion was not weakened. Vite build and `git diff --check` pass. Logs are archived.

```sh
OPTIONS='{"wallCompliance":0.000001,"batchActivationSize":64,"simultaneousContactRelease":true,"condenseCompliantContacts":true}' node scripts/physics/profile-shared-axis-full-cycle.mjs /tmp/oet-condensed-cycle
```

Next distinct experiment: replace expensive exact trial-projection solves with a bounded iterative correction used only as a private line-search proposal. The original final Newton equilibrium and geometry checks must still govern publication. This has not yet been implemented.
