# Exact batched WASM strain differentiation — 9 September 2026

The continuous material-frame tape now records scalar values/local partials in JS and evaluates first derivatives and differentiated reverse adjoints in batched WASM calls. The operation order is preserved; no finite differences, fast-math reassociation, Hessian approximation or changed physical gates are introduced. One fixed, non-growing memory is reused for both derivative orders, both tools and successive steps. Finalization only computes the changed suffix after a native-frame checkpoint.

## Evidence

865/865 composite tests passed; build passed. This is not the whole npm test suite. The new test covers finalization, suffix invalidation, nonfinite rejection/retry, fixed memory and dimension reuse; existing tests compare exact full forward derivatives, physical finite differences, signed reactions, history and rollback.

Paired full synthetic joint steps, 12 warmup pairs and 30 alternating measured pairs; two own-history steps agree in every compared numeric field (maximum difference zero), with identical accepted counters. Baseline files match the finalized 864-stage source manifest. No contact, anatomy, advancing insertion trajectory or rendering is included.

| Case | Median before / after (ms) | P95 before / after (ms) | Frame scratch before / after (bytes) |
| --- | ---: | ---: | ---: |
| 3 nodes/tool, curved, fixed 0/1 | 13.906 / 9.066 | 15.440 / 10.340 | 698368 / 524288 |
| 6 nodes/tool, fixed 0 | 47.642 / 24.470 | 49.993 / 25.747 | 1288192 / 917504 |
| 6 nodes/tool, fixed 0/1 | 47.700 / 24.429 | 49.068 / 25.205 | 1288192 / 917504 |

Cost remains far outside the mean 4 ms / P95 6 ms budget. An independent CPU profile (240 fixed-state replays, including startup/warmup) attributes about 31% of samples to the two WASM derivative sweeps, 6.3% to continuous basis sampling and 5.8% to scalar-node allocation/recording. These are attribution samples, not timings. Whole-step direction median is only about .23 ms in the last paired case; factorization is not its principal remaining cost.

## Reproduction

```sh
node scripts/physics/benchmark-composite-continuous-tape.mjs --baseline-report=reports/composite-continuous-strain-wasm --output=/tmp/composite-strain-wasm.json
node --cpu-prof --cpu-prof-dir=/tmp reports/composite-continuous-strain-wasm/profile.mjs
npm run test:physics:composite
npm run build
```

Generated kernel: `scripts/physics/build-composite-strain-kernel.mjs`, 1716 bytes, optional build-only `wabt` resolved locally or through `OET_WABT_PATH`. Runtime uses embedded bytes without that dependency.

Curved contact, finite material-surface slip history, moving boundaries/profiles, metric-controlled adaptation and the complete UI feed/render path remain unfinished. The newest solver is not selected by the current application URL. No 60 FPS or full runtime acceptance claim.
