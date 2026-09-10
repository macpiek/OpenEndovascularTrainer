# Compute only the required lumen contact derivatives

The common solver now avoids normal-contact Hessians during gradient-only trial evaluation and when the normal force is exactly zero. It still validates and queries every original sample and computes its actual gap, gap Jacobian and signed force column. At Fn=0, the geometric contribution -Fn*DB is exactly zero; nonzero signed private Fn restores the full derivative. This is not a small-force threshold or a removed contact.

Side geometry exposes gradient, witness and full orders. Witness order includes derivatives of the normal and outer projection fraction, which the surface-friction operator needs, without computing the unused normal-contact Hessian. Previous surface geometry uses gradient order because it is fixed input. The affine contact pullback also supports exact G/B without DB. Unavailable arrays are NaN and explicit validity flags prevent a stale full tangent from being reused.

## Whole-step comparison

Both roots use the same step solver, material data, full relative coordinates, constraints, force/history state, numerical budgets and acceptance criteria. Each case uses 10 warmup pairs and 24 measured pairs with alternating order and reusable workspaces. All accepted states, material momenta, reactions, force balances and physical certificates are exactly equal. Direction/evaluation/query counts are also equal.

| Synthetic case | Whole-step median, before → after | P95, before → after | Median reduction |
| --- | ---: | ---: | ---: |
| Open lumen, 65 nodes | 44.053 → 41.408 ms | 53.160 → 51.702 ms | 6.0% |
| Loaded Coulomb lumen, 17 nodes | 37.727 → 32.131 ms | 43.572 → 35.262 ms | 14.8% |
| Loaded Coulomb lumen, 65 nodes | 188.072 → 146.179 ms | 212.885 → 175.252 ms | 22.3% |

These are Node synthetic-material fixtures, not actual anatomy or browser FPS. They remain far outside the target budget. The final loaded 65-node case uses 9 directions, 58 evaluations and 7552 original contact queries. Even the open case spends a median 26.700 ms preparing the step. Repeated preparation and repeated full evaluations remain major targets; matrix factorization alone cannot recover that time.

`benchmark.json` preserves samples and benchmark source hashes. Two documentation comments were added afterwards; executable code is unchanged. To reproduce, create two isolated copies of current sources with the same dependencies, apply `baseline.patch` only to the baseline copy, then run `node scripts/benchmark-composite-lumen-differentials.mjs BASELINE_ROOT CANDIDATE_ROOT output.json`. The first attempted long loaded fixture crossed an exact zero radial normal and was rejected by both the original source policy and its initial-friction validation before timing. That failure is retained in `first-attempt.txt`; the measured long fixture has an explicit smaller slope and does not cross the axis. This does not resolve general zero-normal contact-history initialization.

## Validation and remaining work

Full composite suite: **781/781 PASS**, 16.853 s. Build PASS, 1.63 s, with the existing bundle-size warning. Two new tests compare each derivative order against full physical derivatives and reject stale Hessians. Existing coupled/friction, World, feed and rollback tests pass. The earlier experimental search-subspace tests are archived separately and are not counted in this production suite.

The adaptive relative search experiment reduced linear unknowns but increased total time. Its runtime path was removed; code, tests and measurements remain in `../composite-relative-search/experiment.patch` and the adjacent report. The original relative-direction solver was restored byte-for-byte to the previous validated source. This optimization is not the final adaptive mechanical representation.

Still open: general moving-material and angular/surface history transport, original sheath/portal/tip/external source mappings, loaded anatomy lifecycle, mesh/range transfer, error-controlled mechanical reduction, UI integration and deep/max insertion browser measurements. The current app selects the earlier solver. The 60 FPS/120 Hz goal remains active and unachieved.
