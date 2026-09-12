# Certification of the existing discrete normal contact law

The final split-motion certificate now tests the same history-aware normal equation that the joint solver already solves. Closing velocity remains an explicit diagnostic. It is not independently capped and is not reset or projected. This is a full-step discrete contact approximation, **not a terminal collision/TOI/restitution solver**.

Only `kirchhoffSplitMotion.js` production source changes. The physical/bias solves, body state, impulses, coefficients and World tolerances are unchanged. Local tests use the untouched World SHA `2d8201499fd77caa3c1260344229c208a109d26222cd0e960baa292c441d1eb5`; root is integrating its separate transaction and bias precheck work.

## Certificate

`measureKirchhoffSplitNormalCertificate` traverses all current lumen and boundary normal rows (wall, point-wall, sweep, tool and sheath), without filtering out rows according to their current gap. It recomputes motion from the current physical velocity channel:

```text
h = max(0, g_start) + dt * J_current * v_physical
q = h + alpha * lambda_physical
lambda_physical >= 0
q = 0 when loaded, otherwise q >= 0
```

Existing loaded thresholds are retained: 1e-8 for lumen and 1e-10 for additional boundaries. The maximum residual must satisfy the existing `world.coupledContainmentTolerance` (.001 mm). A cached settled measurement cannot substitute for current `Jv`.

Fresh geometric penetration is also checked explicitly. Hard rows require raw `g_actual >= -tolerance`. A compliant geometric bias may retain its already solved compression, so the admissibility condition is `g_actual + alpha*lambda_bias >= -tolerance`, using the separate bias normal bank. This does not silently demand zero raw gap for a compliant contact, and does not permit a new hard penetration hidden by an unchanged velocity/history equation. Nonfinite quantities or negative normal loads cannot certify.

Final acceptance still requires both phases accepted, the existing full fresh World KKT/material/control/fold/friction gates settled, and no unverified history. The new normal certificate replaces only the independent `dt*maximumOutwardContactVelocity <= tolerance` requirement. Raw outward velocity and raw penetration remain diagnostics.

## Bounded first-closure observations

The independent plane fixture uses a straight two-node wire of length .5 mm, radius .5, unit node mass and segment inertia, zero material strain, zero friction/damping/force and incoming velocity +2 mm/s toward the y=0 wall, dt=1/120. A noncontacting fixed three-node companion provides the JOINT pair. Coordinates x=[1.25,1.75] avoid a nonrepresentable Float32 edge. Sweep count is zero. The same seven observations were executed with the exact pre-change SplitMotion and with the new certificate.

**All seven baseline/new observations have exactly equal raw gaps, physical velocities, spin, normal impulses, momentum changes and energies. Only certification changes.** For .01 initial gap and for the compliant case, the old velocity cap rejected an otherwise World-admissible discrete solution; the new certificate accepts its existing equation. No state or force was repaired by certification.

| Hard initial stored gap | First closing velocity | First raw final gaps | New certified/history |
|---|---|---|---|
| 0 | at most 1.12e-8 mm/s | [0,0] | true / 1 |
| .000500023365020752 mm | approximately .06000281 mm/s | [0,0] | true / 1 |
| .009999990463256836 mm | 1.1999988555908203 mm/s | [0,0] | true / 1 |

The strict .01-mm first-impact and next-step oracle passes. First impulse is 1.600005614841436, independent momentum loss 1.6000056266784668; final energy is 1.4399972534192784 versus predictor 4.000006675723. On the next unforced dt, raw gaps stay [0,0], closing velocities are about [-5.33e-9,6.67e-9], the additional impulse is 2.39999767429286 and remaining energy about 2.28e-16. Thus the next step starts at approximately zero clearance and obeys the corresponding discrete stop; no unaccounted velocity reset is used.

Manually zeroing the first step's velocity while retaining its old physical lambda produces fresh normal residual .009999990463256836 and is rejected even if a stale caller measurement says settled. The rejection changes no reaction and adds no history commit. A separate adversarial rigid translation into the wall leaves the history equation settled but creates .004999995231628418 mm raw penetration; the fresh geometric certificate rejects it.

## Explicit strict-oracle failures retained

The original handoff's six-test file had **4 PASS / 2 FAIL**. The original patch,
assertions and raw logs remain archived. Root classifies these two additional
strict checks as an accuracy diagnostic beyond the existing World contract.
The integrated default tests independently check the same equations at the
original .001-mm World accuracy for these two cases; the exact .01-mm closure
and next-stop checks retain their stricter bounds. No production tolerance,
physical equation, state or force changed for this classification.
Run with `OET_SPLIT_STRICT_ANALYTIC=1` to retain the handoff's tighter checks;
the integrated root still reports **4 PASS / 2 FAIL** in that mode.

1. **.0005-mm case, second unforced dt.** Both the exact old and new implementation return raw gaps [0,-.00010001659393310547], velocityY approximately [0,.012000570073723793], omegaZ=.024001127747644194, and physical KKT .00010000475061436493 mm. These are within the original World .001-mm criterion and both versions certify, but violate the newly written strict raw gap bound 2e-6 mm and velocity-stop bound 5e-4 mm/s. Normal impulse .10800503427162766 agrees with independent momentum loss .1080050369312287; energy decreases from about .00360034 to .0003600339. This is existing solve accuracy, not a weakened certificate or an exact analytic stop.
2. **Compliant wall C=1e-4.** Both versions produce raw gaps about [-.0015544593,-.0016473532], with bias-supported geometric compression residual only 1.2371100581009403e-8 mm. Physical normal KKT is .00022715311497449858 mm, within the original World criterion, but outside the new strict analytic residual bound 2e-6 mm. All physical quantities are exactly equal baseline/new. The new certificate allows this declared compliant compression; it does not claim that the strict analytic residual passed.

The original World .001-mm criterion remains required. The stricter failures are recorded separately and do not justify increasing that tolerance or rebuilding the solver in this delivery.

## Regression and unchanged translated failure

- Existing split certification/preserve-strain/sheath tests: **13/13 PASS**.
- Unchanged independent World+wall tests: **11/11 PASS**.
- New requested/strict tests: **4/6 PASS, 2 strict failures detailed above**.
- Translated-mouth, same probe with preserve-strain: **certified=false, historyCommits=0**, physical and fresh history residual .0016996190233046005 mm. Raw penetration/geometry violation is about 5.94e-10 mm, outward velocity remains .20395428279655206 mm/s diagnostically, bias elastic energy change 0. The unresolved fresh physical contact still fails; removing the conflicting cap does not make it pass.

Artifacts are bundled with the patch under `/tmp/oet-discrete-contact-final-8996/results/`: exact baseline/new JSON observations, the retained failing strict-test log, passing regression logs, translated output and an equality comparison. The test can export fresh observations through `OET_DISCRETE_CONTACT_REPORT`:

```sh
OET_DISCRETE_CONTACT_REPORT=/tmp/discrete-contact-observations.json node --test --test-concurrency=1 tests/kirchhoffSplitDiscreteContactCertificate.test.js
OET_SPLIT_STRICT_ANALYTIC=1 node --test tests/kirchhoffSplitDiscreteContactCertificate.test.js
```

No World edit, terminal impulse, CCD, outer reclosure, anatomy, profile change or solver-tolerance change is included.

## Root integration checkpoint

With the separately integrated World bias precheck and whole-step transaction,
the six tests pass at the existing World accuracy. The full coupled suite has
383 tests: 382 pass and the unchanged position-history translated-mouth
lifecycle remains failing. The optional strict diagnostic retains its two
documented baseline failures.

[`split-consecutive-coupled-checkpoint.json`](split-consecutive-coupled-checkpoint.json)
records four accepted regular catheter steps after preparing 12.1 mm of wire,
advancing the catheter from 4.333 to 5.633 mm. Every accepted step certifies both
phases, all final gates and one history commit. The next command to 6.067 mm
is rejected solely by the conservative sheath axial-feature history guard:
catheter node 0 leaves the proximal slab with approximately .058 mm radial
clearance and zero recorded normal load. Its physical KKT is .00016948 mm,
fresh normal certificate 6.94e-6 mm and geometric violation zero. The guard
remains active pending a proof of unloaded feature transitions.

The World remains at 46 accepted timesteps after the rejection. The fixture's
operator command is prepared before the World transaction, so the rejected
snapshot reports the prepared catheter command; it is not an accepted advance.
Observed cold Node step costs (about 13–43 ms for the four accepted steps) include
diagnostic hooks and are neither a clean benchmark nor evidence of 60 FPS.
