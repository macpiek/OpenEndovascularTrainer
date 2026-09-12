# Bounded Coulomb prediction: negative A/B result

The fixed four-factor prediction experiment is **not an acceleration** on this sequence. Keep it optional and disconnected from the default solver. Two of 23 attempts certified; the measured path time rose from **1364.215 ms to 2479.882 ms (+81.8%)**, even before charging accepted-history snapshot publication. Factor counts fell from 2472 to 2292, but that reduction does not represent a speedup. No budget increase, alternative map search, or additional predictor variant was run after this result.

## Scope and acceptance

The patch adds pure metadata/history helpers in `kirchhoffCoulombPrediction.js`, and the exported opt-in `solveCoulombWithPrediction` wrapper in CoulombNewton. It does not call the wrapper from World, SurfaceFriction, CoupledSystem, Direct, or ActiveCondensed, and leaves existing seeded behavior unchanged. The root P3 aggregation of all attempted LU diagnostics is preserved.

The wrapper takes a caller-owned numerical increment, the exact existing fallback callback, and a mandatory independent reconstructed-response certificate. It makes one full-original-system band-LU Newton attempt with at most four actual factorizations (default projection map). No prediction map retry or cold retry is added. A candidate must pass Newton's original full load KKT and the independent certificate. If it fails, the unchanged callback runs once with original inputs and hints. Its own existing internal retries remain intact. Rejected prediction costs are added to diagnostic totals, including LU errors/pivot metrics. The fallback's stored diagnostics are copied before aggregation.

The history consists of actual accepted post-remap/cone-repair trial totals. Material/Fn increments are accepted total minus current lambda. Ft is reconstructed as a world-space force from the accepted trial's actual tangent basis, then projected onto the current actual row axes before subtracting current lambda. The unscaled linear solve target is never mechanical history. Misses produce zero initial increment and leave every equation and cone in the solve. No force threshold, interpolation, or mechanical state write is introduced.

History requires the same explicit run namespace, history version, residual-law version, and dt, plus an earlier solve and a non-future physical step. The law version is checked against the current system metadata, so a stale supplied contract cannot conceal a law change. These captures use World hash `cf7f36eea9364cd872c083115ccf11234190d9f5c71ec902b8eb38da53373b9d` and SurfaceFriction hash `865fb6567377aa03943f937bea6c60ece1ef6e9acb5c8f75e6cc12279c689803`. A future physical-motion/projection-bias split must invalidate this history. These numerical results do not validate that future physical law.

## Sequence and original-input fidelity

All 24 successive systems from `/tmp/oet-coupled-sequence-200.jsonl` were evaluated, covering six physical steps 4747–4752 near feed 200 mm and four hold steps. There are 24 provisional trial records and 24 matching accepted outcomes; all accepted scales are 1 in this sequence. Capture `complete:false` means the deliberate 24-system capture limit, with no capture failure. There is no new world/prefix/browser replay here.

The top-level captured `lower/upper` arrays contain each solved result's final bounds. Therefore both variants use the original **per-row `rows[i].lower/upper`**, original groups, and recorded `initialFree`. Baseline factor counts match the captured runtime on every one of the 24 systems, ranging from 11 to 303. Top-level bounds differ from input bounds on hundreds of Ft rows; using them would measure a different problem.

Outcome records are indexed offline as acceptance annotations of already-applied trials. Each predictor uses only the immediately preceding recorded accepted mechanics, published after that predecessor system; no later force values or candidate outputs are fed back into history. This is a frozen causal-trajectory comparison, not a closed-loop trajectory or FPS benchmark.

## Mapping coverage

There is no history for solve 1. Typical within-step matches are 2094–2106 rows: all 1626 material rows and 156–160 lumen Fn / 312–320 Ft rows when exact labels survive. At feed transition solve 5, coverage falls to 1542/2454 (1380 material, 54 Fn, 108 Ft). Exact material coordinates and the outer material cell remain in the key, so relabelled material rows and migrated contacts are conservative misses. Hold transition coverage is 2055, 2094, 2094, and 2094 at solves 9, 14, 18, and 22.

Wall, sheath, and fold rows remain unpredicted because this capture does not provide their actual accepted trial totals. External friction and tool-release mapping are not demonstrated by this sequence and have no inferred transfer. Only the captured lumen force-side convention 0 is mapped. Duplicate keys are discarded. Detailed family/miss counts for all systems are in the coverage JSON.

## Full A/B result

| Metric | Baseline | Optional prediction |
| --- | ---: | ---: |
| Measured path time, one alternating-order pass | 1364.215 ms | 2479.882 ms |
| Total actual factorizations | 2472 | 2292 |
| Prediction attempts / certified | — | 23 / 2 |
| Prediction factorizations / rejected | — | 84 / 82 |

The successful cases are solve 13 (225 baseline factors to one predicted Newton factor) and solve 21 (39 to one). Of the 21 rejected attempts, 20 use four factors and one uses two. The first system skips prediction because no history exists. All 22 cases that use the baseline callback have byte-identical force increments to the independent baseline.

Both variants pass full-original-matrix KKT and independent `rhs - alpha*x - J*(W*Jᵀ*x)` certification on all 24 systems, including bounds and load-dependent cones. Maximum full KKT is **0.0001925882421972911**, and maximum reconstructed KKT is **0.00019258824219729097**, each below the original **0.0002** tolerance. The maximum candidate/baseline primal-correction difference is **3.61199339020407e-8**. Differences only occur in the two independently certified predictor successes.

Timing includes mapping (40.883 ms in total), every attempted Newton solve, rejected factorizations/backtracking, exact fallback work, and all invoked Jdx certificates. JSONL parsing, accepted-history snapshot creation, and the additional full-matrix audit are outside the timers. Snapshot creation would add predictor overhead, so the measured loss already occurs before its inclusion. There is only one alternating-order pass over different systems, with no repeated timing distribution. No general runtime or physical-law performance claim is supported.

| Solve | Mapped / rows | Baseline factors | Candidate total | Prediction factors | Accepted |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 0 / 2446 | 34 | 34 | 0 | no |
| 2 | 2097 / 2446 | 283 | 287 | 4 | no |
| 3 | 2094 / 2443 | 213 | 217 | 4 | no |
| 4 | 2097 / 2446 | 195 | 199 | 4 | no |
| 5 | 1542 / 2454 | 22 | 26 | 4 | no |
| 6 | 2106 / 2457 | 40 | 44 | 4 | no |
| 7 | 2106 / 2457 | 44 | 48 | 4 | no |
| 8 | 2103 / 2454 | 43 | 47 | 4 | no |
| 9 | 2055 / 2459 | 26 | 30 | 4 | no |
| 10 | 2106 / 2459 | 91 | 95 | 4 | no |
| 11 | 2106 / 2455 | 217 | 221 | 4 | no |
| 12 | 2103 / 2452 | 150 | 152 | 2 | no |
| 13 | 2106 / 2455 | 225 | 1 | 1 | yes |
| 14 | 2094 / 2451 | 20 | 24 | 4 | no |
| 15 | 2103 / 2451 | 303 | 307 | 4 | no |
| 16 | 2103 / 2451 | 89 | 93 | 4 | no |
| 17 | 2106 / 2454 | 73 | 77 | 4 | no |
| 18 | 2094 / 2454 | 11 | 15 | 4 | no |
| 19 | 2106 / 2455 | 110 | 114 | 4 | no |
| 20 | 2106 / 2455 | 41 | 45 | 4 | no |
| 21 | 2103 / 2452 | 39 | 1 | 1 | yes |
| 22 | 2094 / 2452 | 33 | 37 | 4 | no |
| 23 | 2103 / 2451 | 107 | 111 | 4 | no |
| 24 | 2106 / 2454 | 63 | 67 | 4 | no |

## Regression checks and handoff

Read-only `git apply --check` against the root passes. Removing the new export leaves the existing Newton source byte-identical to the root baseline, including P3 diagnostics. The portable reproduction script passes `node --check`. All 30 files in the frozen dependency closure match their manifest hashes.

75/75 targeted tests pass, including 14 prediction/history tests, both real consecutive-system pairs (2 and 13), existing Newton/band LU/bound recovery/load KKT/ActiveCondensed tests, and the root's P3 all-attempt LU regression. The pair fixture omits predecessor matrix/rhs/increment, retaining actual accepted mechanics; tests check source immutability, law invalidation, basis mapping, exact conservative misses, the four-factor cap, the mandatory certificate, fallback equality, and full KKT/Jdx.

The text patch contains only Newton's new export, the pure helper, and two new test files. Apply it against the root Newton baseline SHA recorded in the manifest. Copy the **2,070,045-byte** `tests/fixtures/kirchhoff-coulomb-prediction-pairs.json.gz` separately. Do not replace the root's full `kirchhoff-coupled-sequence-200.jsonl.gz` fixture. Reports and reproduction scripts are separate review artifacts.

The source/test dependency closure and required small regression fixtures are frozen at `/tmp/oet-coulomb-prediction-frozen`; file hashes and validation command are in the manifest. The raw measured script is retained verbatim as `kirchhoff-coulomb-prediction-measured.mjs` (original absolute paths). The readable portable `kirchhoff-coulomb-prediction-reproduce.mjs` takes a JSONL or gzipped JSONL path and output JSON path; it is a reproduction aid, not an additional executed variant. Run only if a fresh bounded rerun is requested.
