# Residual line search only after stagnation

Retained as `stagnationResidualSearch: true`, disabled by default. Two consecutive eight-iteration windows must show no meaningful improvement before switching globalization. The ordinary energy/constraint policy remains unchanged before activation, including first-contact feasibility restoration. After activation, feasible trials must reduce the normalized force/torque/constraint residual. All original final equilibrium and friction certificates remain mandatory. This changes the search strategy, not the material or stopping tolerances.

The trigger also covers frozen-friction solves, which could previously spend all 160 Newton iterations alternating between nearly identical residuals. Geometry discovery resets the observation window. An observer event records activation for replay diagnosis.

## Full-cycle results

Each candidate completed all 5757 movement steps at the original physical timestep and input speeds. These are Node measurements; none proves browser 60 Hz. Totals below exclude the initialization step, unlike the comparison script's quality totals.

| Variant | Mean ms | P95 ms | Worst ms | Newton | LU | Full assemblies | Residual assemblies |
|---|---:|---:|---:|---:|---:|---:|---:|
| Rigid reference | 33.915 | 93.964 | 1612.864 | 20741 | 87269 | 32058 | 40185 |
| Rigid + stagnation switch | 34.397 | 97.732 | 1804.902 | 20562 | 86823 | 31885 | 39989 |
| Compliant condensed reference | 32.477 | 67.554 | 8979.022 | 24685 | 77995 | 36498 | 49985 |
| Compliant condensed + switch | 31.087 | 68.591 | 3274.873 | 23572 | 73183 | 35146 | 47803 |

The compliant comparison reduces LU work by 6.2%, mean time by 4.3%, and the observed maximum by 63.5%. This is useful recovery behavior, but 3986 steps still exceed 16.67 ms. The rigid version gives no meaningful overall speedup and is not enabled in the app.

The incoming compliant step at wire 745.80 mm improves from 586 Newton / 5576 LU to 99 / 1068, with the original dt and final force/friction bound 6.0903e-5. The cold diagnostic replay takes 2445.96 ms, including observation overhead; it is not a paired benchmark timing.

## Trajectory and geometry

Rigid candidate: all 96 saved snapshots are identical to the reference in the initial, wire-in, catheter-in and catheter-out phases. Wire-out RMS difference is 0.0000641 mm, maximum 0.0007781 mm. The spatial-axis audit found zero crossing frames. It does not prove swept containment between saved snapshots.

Compliant candidate: `shapes.json` is byte-identical to its compliant condensed reference, SHA-256 `348416de6f0c4b2a9bddd22c4d2921a1211fae4fac5cbd4cac1cd3bd40aab3af`. The existing audits in `../contact-condensation-research/` therefore apply to these same saved positions: zero spatial-axis crossings and approximate far-outside lumen flags; maximum capsule overlap 0.105895 mm. This is still a sampled trajectory check, not a proof that all model openings are sealed.

Compared with the rigid reference, the compliant physics retains its earlier differences: phase RMS 2.34 / 4.28 / 7.18 / 6.33 mm, maximum 32.42 mm. The stagnation switch introduces no additional saved-shape deviation in that comparison. All states are finite; maximum reported sampled penetration 0.041793 mm and force/friction certificate below 1e-4.

## Validation and next bottleneck

65 selected regression tests passed and the production build passed. New tests check bit-exact ordinary bending/unloading, actual stagnation activation, monotone accepted feasible residual-search trials, preserved input state, equilibrium bounds and reduction of repeated work on the captured difficult case.

A representative 100-step catheter-withdrawal window in the rigid candidate averaged 30.78 ms: 16.30 ms assembly, 7.51 ms linear solves, 1.45 ms explicit friction refresh. These timers do not account for every application operation. Typical-step assembly remains the main next target; reducing rare Newton stalls alone cannot meet the objective.

Raw complete cycles, source hashes, slow incoming states, comparisons, logs and the implementation patch are archived here. The 60 Hz objective remains unachieved.
