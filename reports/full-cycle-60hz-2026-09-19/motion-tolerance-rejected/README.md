# Rejected: stop Newton at a small remaining correction

The experiment completed the full 5757-step cycle, but was slower and changed the trajectory substantially. Its runtime changes have been removed; the exact force criterion and the preceding opt-in compliant-contact work are restored. Source, tests, patches, incoming replays and complete benchmark data are archived here.

The proposed inexact criterion required a successfully solved **full Newton** direction below 0.005 mm translation, 0.001 rad bend plus spin, and 1% normalized reaction correction. Current constraints also had to pass their original tolerance. The remaining correction was not applied. Gauss–Newton and modified directions could not trigger acceptance. Diagnostics explicitly reported `converged-motion`, the actual residual and `forceConverged: false`; the estimate was never an absolute trajectory error bound.

The experiment used `wallCompliance=1e-6`, `batchActivationSize=64`, and `simultaneousContactRelease=true`. On the difficult incoming step 1144, Newton iterations fell from 13 to 11, but LU only changed from 144 to 143. A second incoming case initially worsened from 1454 to 2704 LU because a frozen friction chart repeatedly needed correction. Restricting motion acceptance to the refreshed coupled chart (or no loaded friction) restored that case to exactly 121 Newton / 1454 LU. This guard was present in the full benchmark. Five dedicated tests passed, including missing normal reaction, frozen-friction force convergence and rollback.

| Complete Node cycle | Exact app reference | Compliant + batch release | Add motion stopping |
|---|---:|---:|---:|
| Mean ms | 33.915 | 30.980 | 48.133 |
| P95 ms | 93.964 | 75.512 | 130.770 |
| Maximum ms | 1612.864 | 2583.079 | 13950.164 |
| Steps over 16.67 ms | 4006 | 3959 | 3923 |
| LU factorizations | 87269 | 69607 | 133917 |
| Newton iterations | 20741 | 23312 | 21033 |
| Full assemblies | 32058 | 34686 | 40451 |
| Residual assemblies | 40185 | 45143 | 45080 |

5025 accepted steps ended at motion resolution. Worst step: wire-in index 1340, insertion 982.67 mm, 536 Newton iterations and 8168 LU across all retries/substeps. All timing includes failed attempts. Node excludes browser rendering/UI and cannot prove browser 60 Hz.

Compared with the exact reference at 96 saved snapshots, shape RMS was 22.99 / 76.04 / 67.96 / 22.39 mm for wire-in / catheter-in / catheter-out / wire-out; maximum local difference 274.34 mm. Thus the small single-direction estimate did not preserve the full trajectory. Maximum actual force/torque residual was 866.48, sampled penetration 0.08302 mm, relative edge-length error 1.318e-4, and speed 399.74 mm/s.

Independent snapshot audits found zero axis crossings and zero far-outside warnings in 52,267 lumen samples. Continuous capsule overlap reached 0.12257 mm (reference 0.10964 mm). These checks do not rescue the failed trajectory/performance requirements; nor are snapshots a swept containment proof.

Do not enable this criterion or infer that 0.005 mm stopping implies 0.005 mm simulation error. The global 60 Hz goal remains unmet. The next distinct candidate is algebraic elimination of compliant normal-reaction unknowns from each fixed active-set matrix, retaining the original force and complementarity checks instead of relaxing convergence.

After restoration, 58 targeted regression tests passed, including the closed-root replay, axis guard, timestep rollback, friction assembly, native/app solver and retained compliant-contact model. `git diff --check` passed. No motion-stopping code remains in `src` or `tests`.
