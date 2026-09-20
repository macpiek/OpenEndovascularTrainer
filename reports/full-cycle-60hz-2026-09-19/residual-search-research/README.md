# Residual globalization experiments — rejected

Neither experiment is enabled in the runtime. The original Newton acceptance and exact projection were restored. Compliance and contact condensation remain separate opt-in experiments.

Strict normalized-residual line search improved one difficult incoming state from 586 Newton iterations / 5576 LU to 43 / 580, but failed the cycle at 104.87 mm when first contact required temporary force imbalance during geometric restoration. See `strict-rejected/`.

Using original feasibility restoration before residual globalization completed all 5757 movement steps, but averaged 34.116 ms with P95 72.729 ms and maximum 5386.868 ms. There were 3994 steps over 16.67 ms, 24438 Newton iterations and 84383 LU factorizations. This does not achieve 60 Hz.

The trajectory differed from the exact reference by up to 443.82 mm. The approximate lumen audit flagged 55 snapshots, despite no spatial rod-axis crossings in the 96 stored snapshots. Containment therefore remains unresolved for this rejected trajectory. Ray parity alone cannot establish whether a point is in the lumen: this STL models wall volume, with both lumen and surrounding space outside that volume. See `feasibility-first-rejected/` for raw measurements and experimental source.

The restored implementation passed 63 selected solver/contact/application regression tests. During the experiment, the broader selection had 94 passes and the previously observed projection-fixture failure; the production build passed. No full-suite success is claimed.

Replay diagnostics now report top-level `elapsedMs` around the complete advance, including subdivisions. The nested `result.ms` may describe only the final substep and must not be used as total step time.
