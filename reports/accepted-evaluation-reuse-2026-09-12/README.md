# Reuse of accepted contact kinematics

Point 1: avoid reconstructing unchanged lumen-friction surface kinematics when the accepted candidate becomes the next outer iteration's base state. Enabled by default; `coupledSystem.reuseAcceptedEvaluation: false` restores fresh evaluation for A/B comparison.

The next iteration still refreshes contacts and normal Jacobians. It then compares the exact pose/history arrays, body identity, material inputs, ordered contact identities, feature geometry and interpolation stencils against the preceding evaluation. Numeric rod arrays use reusable typed banks and bulk copies; only a reuse request compares their contents. Comparisons retain signed-zero semantics and use no tolerance or hash shortcut. The guard is conservative and covers entire rod arrays.

On a match, the evaluator retains contact witnesses, local frames, levers, interpolation and relative surface displacement. It reads normal/tangential loads and the manifold force basis again and recomputes force projection, friction KKT residuals and reaction diagnostics. This matters because contact refresh can reproject force components even at an unchanged pose. On a mismatch it evaluates everything afresh. Failed evaluation invalidates the cache. A residual bank promoted to gradient rows for cone repair returns to residual-only form before reuse; fresh and reused merit therefore follow the same arithmetic path.

This is partial reuse, not reuse of the whole accepted convergence certificate. Material, wall, portal and contact-motion stopping checks stay current, including the new post-pass motion reference. Velocity/split and wall-witness paths do not enable the runtime cache. Physics parameters, iteration budgets and tolerances are unchanged.

## Controlled cost measurement

`frozen-contact-benchmark.json` measures a real-anatomy state at wire 309 mm / catheter 100 mm (77 contact records). One sample times 16 pairs of candidate evaluation plus repeated base evaluation. Both variants are warmed up, then alternated in AB/BA order for 80 rounds in one process. This includes capturing/checking the guard and recomputing current loads/KKT. All 1280 proposed repeated evaluations reused kinematics.

| Cost per evaluation pair | Fresh | Reuse | Reduction |
|---|---:|---:|---:|
| Mean | 0.582 ms | 0.454 ms | 22.0% |
| Median | 0.575 ms | 0.450 ms | 21.8% |

The absolute saving is approximately 0.128 ms per pair at this state. This is a local residual-evaluation benchmark, not a 22% improvement to the whole step or rendered FPS.

## Whole-step replay

Four sequential replays in ABBA order used the same source and only toggled `OET_REUSE_ACCEPTED_EVALUATION`. There were no simultaneous benchmark/test processes. The desktop/browser remained in normal use; timings are noisy. Representative mean step times:

| Overlap phase | Fresh run 1 | Reuse run 1 | Reuse run 2 | Fresh run 2 |
|---|---:|---:|---:|---:|
| Feed to 100 mm | 49.23 ms | 52.41 ms | 49.66 ms | 61.43 ms |
| Hold at 100 mm | 88.20 ms | 66.59 ms | 56.59 ms | 63.74 ms |
| Refeed to 100 mm | 72.26 ms | 52.08 ms | 56.05 ms | 64.74 ms |

The high run-to-run variance does not support a reliable percentage improvement for the whole step. Feed reuses about 5.49 batches per measured step; the existing geometry, KKT, assembly, solve and rollback work still dominates. This change does not establish 60 FPS or real-time physics at 120 Hz.

## Correctness and checks

- 2578 steps per replay: wire-only, catheter-only and coupled feed/hold/withdraw/refeed.
- 7734 step comparisons against the first fresh replay, with **zero differences** in all body typed-array/manifold hashes, pose fingerprints, insertion depth, penetration, iteration/trial/backtrack/row/factorization decisions, rejection or failures.
- The same existing nonlinear closure failure occurs during coupled withdrawal in all variants. No feed/hold/refeed failure is introduced.
- 95 focused tests pass. New cases compare cached results with fresh results after load and tangent-basis changes, cone-row promotion, pose/history/radius/feature/stencil/topology changes, signed zero, timestep changes, rollback and invalid inputs.
- Production build passes; Vite's existing large-bundle advisory remains.

Reproduce from the repository root:

```sh
OET_VERIFY_WITHDRAWAL=1 OET_REUSE_ACCEPTED_EVALUATION=0 node scripts/physics/profile-overlap-309.mjs /tmp/fresh.json
OET_VERIFY_WITHDRAWAL=1 OET_REUSE_ACCEPTED_EVALUATION=1 node scripts/physics/profile-overlap-309.mjs /tmp/reuse.json
node scripts/physics/benchmark-accepted-evaluation.mjs /tmp/frozen-contacts.json
```

`comparison.json` records source hashes, Node version, per-phase metrics and comparison fields. Compressed JSON files preserve all four full trajectories. Test and build logs are included.
