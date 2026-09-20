# Feasibility-first residual globalization — rejected

The original energy/constraint restoration was retained for infeasible iterates. Once feasible, line search required sufficient normalized force/torque/constraint residual decrease. Final physical tolerances were unchanged. Experimental source and tests are archived here; this code is absent from the live runtime.

All 5757 movement steps completed. Mean 34.116487 ms, P95 72.728667 ms, maximum 5386.867625 ms; 3994 steps exceeded 16.67 ms. Totals: 24438 Newton iterations, 84383 LU factorizations, 36668 full assemblies, 54061 residual assemblies. The worst step was wire insertion 914.47 mm, with 165 Newton iterations and 2561 LU factorizations.

Maximum trajectory discrepancy from the exact reference was 443.82 mm. Phase RMS differences were 35.56 / 127.99 / 127.60 / 48.09 mm. Even the initial pose changed by 0.00413 mm, so this was not a modification restricted to stalled Newton solves.

All accepted states were finite and met the original reported equilibrium bounds. Sampled penetration was at most 0.040196 mm; an independent capsule audit on 96 saved snapshots found 0.126429 mm. Spatial axis crossings were zero in these snapshots, but the approximate lumen audit flagged 55 frames. The first warning occurred at wire insertion 923.27 mm. These checks do not prove containment between snapshots or across unsealed openings.

Additional six-direction ray queries at the first flagged segment point and tip returned intersection counts [0,0,0,2,0,0] and [0,0,0,0,0,0]. Because the anatomy describes wall volume, even parity alone is not a lumen-versus-exterior classifier. Containment requires further investigation; no watertightness claim is made.

The variant was rejected on both performance and trajectory grounds. A future stagnation-only experiment would need new full-cycle and containment validation.
