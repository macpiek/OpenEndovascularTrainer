# Exact gradient assembly in the complete timestep

`assemblyPolicy:'auto'` now uses the original Exact gradient without a Hessian
for the final acceptance check when `elementBackend:'wasm-exact'` and compiled
inertia are selected. That check needs fresh forces and gaps, but never solves
another direction. Trials still assemble full tangents by default. The full
oracle remains available through `assemblyPolicy:'full'`.

The explicit experimental `assemblyPolicy:'lazy'` also evaluates trial forces
without a tangent and rebuilds a full tangent only before a required direction.
Every full or gradient constitutive assembly consumes the same global evaluation
budget, including additional lazy rebuilds. An invalid/stale Hessian cannot enter
Chain or MixedDirection. Contact queries are not repeated for a tangent rebuild
at an already evaluated position. Physical residuals, rollback and acceptance
tolerances are unchanged.

The compiled element gradient is byte-identical to full Exact assembly; compiled
inertia preserves the same energy, velocities and gradient while leaving unused
Hessian bytes untouched. Nine timestep tests cover six real-profile cases over
three consecutive steps, full/auto/lazy equality, budgets, retry and poisoned
Hessian storage. The integrated Composite suite passed **248/248** before the
separate RelativeCluster addition. The latter subsequently passed its related
**38/38** tests, including its nine new tests.

## Paired complete-step measurements

The final probe compares full, certificate-only (auto), and lazy policies on the
same prepared state with their own persistent numeric workspaces. Each of six
fixed-topology cases has one cold, three warmup and eight measured consecutive
steps per policy. All **216 calls accepted** with identical physical states,
certificates, directions and contact-query counts. Assembly counts differ as
intended. No feed, moving topology, lumen clearance, friction or anatomy is
included, so these measurements do not establish browser FPS.

Median wall time in milliseconds from
[the final three-policy run](composite-timestep-gradient.json):

| Contact | Catheter mm | Full | Auto | Lazy |
|---|---:|---:|---:|---:|
| Free | 9 | 15.674 | 18.010 | 15.061 |
| Free | 160 | 14.248 | 12.928 | 13.923 |
| Free | 310 | 18.992 | 15.570 | 14.520 |
| Plane | 9 | 14.812 | 13.348 | 11.932 |
| Plane | 160 | 23.255 | 20.964 | 22.273 |
| Plane | 310 | 21.802 | 20.725 | 20.380 |

The JSON also records current-thread CPU time, process CPU time, every sample,
cold setup, original certificates and before/after source hashes. Current-thread
CPU excludes background compiler threads, but is still not a rendering/FPS
measurement. For the deepest wall case its medians are 18.010, 17.628 and
17.690 ms respectively. Host/JIT variability is substantial.

Two earlier runs remain preserved:
[initial wall-time run](composite-timestep-gradient-initial.json) and
[process-CPU run](composite-timestep-gradient-process-cpu.json). They contain
regressions as well as improvements; in particular the first deep-wall lazy
comparison worsened from 26.44 to 36.92 ms. Absolute times also differ materially
from the earlier workspace benchmark. These results do not support a universal
whole-step lazy speedup or the goal's 4 ms average / 6 ms P95 budget.

For that reason lazy trials remain explicit. Auto only replaces the final unused
tangent with the same original gradient, without introducing an additional
assembly. Further performance work must address nonlinear convergence and
measure the complete integrated model.
