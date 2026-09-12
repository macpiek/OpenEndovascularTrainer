# One original residual evaluation per joint direction

`kirchhoffCompositeRelativeDirection` now contracts the original matrix once when checking a direction. Prescribed primal increments and held dual increments are exactly zero. Consequently, every remaining row has the same residual in the original and boundary-modified systems. The correction RHS is obtained by copying the original residual and zeroing only prescribed/held rows. Their original physical residuals remain in the certificate. At the initial zero increment the original residual is exactly the input F, without a matrix contraction.

The compensated summation, original equations, matrix assembly, factorization, row scaling, refinement limits, force/torque/constraint gates and boundary reaction calculation are unchanged. In particular, an incompatible prescribed constraint still fails; its error is not hidden by the zero used for the held linear increment.

Validation: 17 Direction tests, including an independent contraction of every correction RHS over three factorizations with a nonzero incompatible held row. Together with JointAssembly and JointTimeStep, **43/43 PASS**. The full Composite suite after workspace reuse and this change passed **392/392**, 18.999 s (`/tmp/oet-composite-workspace-residual-full-suite.txt`).

The [isolated direction measurement](composite-single-residual-direction.json) alternates 30 warmup and 60 measured pairs per size with identical synthetic GN/inertia/nonsymmetric local rows. Increments, original residuals, proofs and support reactions are bit-identical. Median direction times in ms:

| Nodes | Before | One residual |
| --- | ---: | ---: |
| 33 | 1.242 | 0.883 |
| 65 | 2.394 | 1.779 |
| 201 | 7.140 | 4.741 |

The separate [whole-step comparison](composite-single-residual-timestep.json) uses immutable copies of the reusable workspace version and changes **only** RelativeDirection. Both arms reuse their own workspace. It runs a 33-node synthetic full-overlap contact-free fixture at dt=1/120 s, 30 warmup pairs and 50 measured pairs per arm, alternating arm order. Each pair's second physical step uses its first accepted state and material velocities. All 200 measured dt accepted with bit-identical states, original certificates and per-tool impulse/support balances. Source hashes and all samples are retained. Whole-step medians / P95 in ms:

| Step | Before | One residual |
| --- | ---: | ---: |
| First | 9.239 / 17.142 | 8.953 / 13.693 |
| Continued | 6.137 / 11.519 | 5.446 / 9.181 |

Thus the measured improvement of an individual direction is larger than the improvement of the complete step. This run must not be compared directly with an earlier workspace run to attribute additional gains: warming and host load differ. The remaining full-step costs and tails still exceed the target. These are Node diagnostics without real anatomy, profiles, contact, rendering or an FPS acceptance claim.
