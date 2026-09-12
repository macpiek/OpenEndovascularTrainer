# Final independent review — joint tip integration and commit fixes

**Both confirmed public commit defects are closed by independent retest. No outstanding concrete finding remains in the reviewed scope.** The five requested test files pass **52/52**, and both independent transaction witnesses pass. Source files were not edited by this reviewer.

## Exact reviewed source

A new isolated stage was made from the original 41-file frozen integration bundle and overlaid with exactly the final two source/test files. No live root RelativeDirection or other concurrent solver work was imported.

- Base manifest SHA256: `b19dae0f91e9d276b7d4f671f52525f08100a6c9835368dcc59543ccbaf240f2`; all 41 listed hashes match.
- Final fix manifest SHA256: `fbbc7f40ee10bd8496fd032aa6066caaa596b5334cadc4aea77e302591503223`; all three entries match (two replacement files plus the supplied log).
- Final `src/physics/kirchhoffCompositeJointLumenRows.js`: `fe3b4f5fbdf9f60e08f12dcafdc23b052c136d544a66e7df728d6bed5397a3af`.
- Final `tests/kirchhoffCompositeJointTipTimeStep.test.js`: `db5a1e48f04bcd0c9f822d6d7c62e1abdf573d62749a008687428634b753aee8`.

`source-audit.json` records every staged file; copied base/fix manifests preserve the inputs. The borrowed Aorta and Three runtime hashes are recorded separately. Both changed files also passed Node syntax checks.

## Independent transaction results

The independent strict-rim fixture remains identical to the original finding: wire [1.8,.45,0]→[2.3,.55,0], catheter [0,0,0]→[2,0,0], lumen=.5, inner=.16, fillet=.15. The original gap is approximately zero and the certified load is Fn=1.

| Witness | Final result |
|---|---|
| Change Fn to -1e-30, NaN or 2 after refresh | Commit rejects; no added query |
| Move wire y by +.01 after refresh | Commit rejects the stale physical geometry |
| Restore exact previously queried force and points | Same valid history can be recovered without a query |
| New converged Fn=2 refresh | Fresh history and nodal force both carry Fn=2 |
| Fresh penetrating refresh | Original gap/NCP/work reject; earlier commit authority is revoked |
| Set returned nonconverged certificate to `converged=true` | Commit still rejects penetrating gap -.01 and Fn=1 |
| Set returned valid certificate to `converged=false` | Public diagnostics cannot alter the actual private acceptance proof |
| Failed query after a valid refresh, then mutate old diagnostics | Commit remains disabled until a new successful refresh |

Earlier returned history copies remain unchanged. `commit-freshness-retest.mjs/.json` and `returned-certificate-retest.mjs/.json` preserve executable and raw evidence. These are two independent witness scripts in addition to the 52 node tests, not two extra whole-step tests.

The final implementation combines exact force/physical-point snapshots with a private `commitReady` boolean established from the actual refresh result and reset at refresh entry. The public certificate no longer authorizes commit. Commit performs no query and does not silently reconstruct or clamp forces.

## Integration conclusions retained from the original review

The original G/B/DB audit found no defect: fillet keeps B=G and the full catheter axis lever couple; rim keeps separate gap G and unit-resultant B=G/m with the full nonsymmetric derivative. Physical pullback and the shared direction preserve both matrix halves. Each declared fillet sample has its own row; no affine-side gauge is applied to either tip feature.

Original absent rim is allowed only with literal Fn=0 and is reported as applicable:false/gap:null; strict-open clamped/fallback contacts require Fn=0 and positive original gap. Loaded unsupported exits reject. Query failure invalidates certificates and published force/row scratch while preserving caller residuals and incoming history. The requested tests cover loaded next dt, release, cold/reuse, late rejection/retry, original physical force/moment/unit-Fn checks and bounded single-material startup.

NCP globalization uses the actual mechanical/gap/sign/NCP equations while retaining original `abs(Fn*g)` as a mandatory acceptance gate. Rim physical virtual power remains `(Fn/m)*dg`; no spurious m scaling enters mechanics. The whole Step requires a fresh final evaluation before commit. Neither original public-API defect was claimed to demonstrate an invalid accepted whole timestep.

## Historical evidence and limits

The original finding and first-fix results remain frozen separately:

- `/tmp/oet-composite-joint-tip-review-8996`: original 51/51 run and demonstrated stale force/geometry commit defect.
- `/tmp/oet-composite-joint-tip-fixed-review-8996`: first fix, 52/52 run, original defect closed and mutable returned-certificate bypass demonstrated.
- This bundle: final two-file fix, 52/52 run and both independent witnesses closed.

The supplied original root-suite log was hash-verified and reports 490/490 PASS; this reviewer did not rerun that full suite and makes no assertion about subsequent concurrent root changes. No long World replay or benchmark was run.

Evidence is limited to fixed declared normal-contact charts and the requested fixtures. There is **no claim of full variable ownership, friction, general feature/winner transfer, CCD or World integration**. No further source changes or scope expansion were made.
