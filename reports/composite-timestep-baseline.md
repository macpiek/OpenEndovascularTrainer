# Actual CompositeTimeStep baseline — frozen handoff

The full step is not yet within the6ms gate. The new single-chain operator is small; repeated full assembly in nonlinear backtracking is the dominant cost. This measurement does not claim60FPS.

## Scope and reproducibility

Executed immutable source snapshot: /tmp/oet-composite-timestep-snapshot-KhSyxw/manifest.json. Every captured source file was byte-identical before/after the run. TimeStep SHA72a69161026fa0e6fcdf6c38041cf06f7660fe59858d04f68d4c302da81c6619. It predates root's next mixed solve and later backsolve/history accounting integration. Complete graph includes the private pinned Three.js module dependency. Existing World/Simulator source is captured as mass/tolerance provenance only, never executed as a solver core.

Real nominal Glidewire/Berenstein constitutive profiles, wire318mm, catheter9/160/310mm, exact fixed tip/material boundaries, one straight spatial chain, independent tool spins and material maps. No feed or remeshing occurs between dt. Common-axis geometry remains a model candidate; finite-clearance admission is external.

Body uses model node masses1/1.4 and nominal spacing5/4mm. The benchmark declares their corresponding continuous mass densities0.2/0.35 model units per material mm; these are existing mechanical conventions, not measured kg. No UI stiffness-scale multiplier is claimed: materials are the built-in nominal Kirchhoff profiles. Old physical material velocities are sampled at the unchanged material labels from the accepted previous dt; rejected states retain the same initial/prepared history.

Contact-free: first two nodes clamped, explicit root spin for both tools, distal transverse load0.005 model-force units, initially zero velocity. Plane control: initial height0.82mm, velocity[0,-10,0]mm/s, no positional clamp, distal load-0.005, exact outermost tool radius ownership and explicit frictionless plane through the same capsule query interface. This is a controlled wall example, not anatomy. Existing anatomical prepared states contain two distinct curves; using one as the other would require unvalidated snapping/history transfer, so anatomy was deliberately not mapped.

Strict series retains original TimeStep test gates: force1e-7,torque1e-8,length1e-8,linear5e-10 (wall test linear1e-10),wallgap1e-8,wallforce/work1e-7. The separate runtime-geometric series changes ONLY World length0.002mm and wallgap0.001mm. No dimensionally compatible old-World force/torque tolerances exist for this new original-force certificate, so strict force/torque/linear/work gates remain explicit. Bounds are the original tests'120directions/20outer/1000evaluation/24line-search for free motion and150/32/2000/30 for wall. dt=1/120s in every case.

Two paired repeats, with reversed strict/runtime order in the second repeat, and two attempted dt each. The second attempt is a genuine consecutive loaded dt after acceptance, or the identical pending dt after rejection.48 total attempts:8 accepted,40 rejected. All24 repeat-pair original/input/result hashes match. All48 original-state rollback hashes match; accepted output increments time/step/history exactly once, rejection retains the original object, time,step and all histories. Actual WASM exports are transparently wrapped only for counts/timing; wrapper overhead is included. The CPU sampler profiles the whole run separately. Frozen test suite passes18/18 original TimeStep+WallTimeStep tests.

## Full-dt results

Setup/material compilation is reported separately. fullDtMs includes prepared inertia/input construction and the complete advance/commit or rollback result. Never interpret failed-attempt duration as an executed dt.

| Scenario / catheter | Strict accepted / rejected | Full time evidence | Full assemblies / directions / backsolves |
| --- | --- | --- | --- |
| Free9mm |4 /0 | cold70.793ms; second paired run12.224/13.134ms |5 /2 /2 |
| Free160mm |0 /4 | rejected median1676.413ms |1000 /70 /70 |
| Free310mm |0 /4 | rejected median1800.902ms |1000 /73 /73 |
| Plane9mm |0 /4 | rejected median388.461ms |233 /17 /17 |
| Plane160mm |0 /4 | rejected median2202.523ms |1246 /150 /150 |
| Plane310mm |0 /4 | rejected median150.897ms |62 /3 /3 |

The runtime-geometric series has exactly the same accepted statuses, step work counts and paired results; failed medians1628.660/1848.718ms free160/310 and383.783/2447.763/138.385ms plane9/160/310. Free9 runtime-geometric accepts4/4, second paired run13.364/9.557ms. Relaxing the geometric thresholds to the existing runtime values does not remove the stalled force/torque solve. The plane310 rejection is faster only because its line-search gives up earlier; this is NOT better simulation throughput.

Node counts67/69/68 and DOFs270/310/336 grow modestly. In contrast, full evaluation count rises5→1000. The short9mm insertion includes only the start of the Berenstein shape; full deeper exposure releases the complete preform, with initial force~2141–2144 versus6.95. The benchmark is intentionally this cold physically loaded pose, not a silently equilibrated seed.

The initial harness's summary used the upper middle sample for even-N medians. The immutable raw rows are preserved; analysis.json recomputes conventional medians correctly. Frozen harness v2 corrects that presentation detail and adds explicit compiled-cell hashes/pair fields/source-convention guards; it does not change numerical fixtures or the execution loop.

## Why cost exceeds6ms

CPU samples classified by the physical call stack: timestep residual/copy/objective glue14.679s (~30.1% of physics), inertia preparation/evaluation/scatter13.999s (~28.7%), elastic chain/element13.592s (~27.8%), lengths3.632s (~7.4%), wall2.209s (~4.5%), and complete linear-direction work0.717s (~1.5%). The counted native backsolves alone use~1–3ms over an entire rejected1.5–2.5s dt. Root should target repeated assembly/nonlinear convergence rather than a global contact Schur or faster backsolve.

Hot self functions include TimeStep.evaluate11.806s, Kinematics.vector validation/allocation6.940s, FastElement finite-output checking5.058s, and inertia assembly4.313s. A prepared inertia operator is constant in Hessian and affine in geometry during one frozen dt; revalidating maps/masses and rebuilding sample arrays on every trial is avoidable. Nevertheless, raw-kernel optimization alone cannot compensate for hundreds of rejected line-search evaluations.

## Separate cold310 diagnostic trace

Trace copy /tmp/oet-composite-timestep-trace-ucJBlC; instrumented TimeStep SHAad2db20f1a72bd75a0deccb598b6533da867235e6111c5a9320fac91070677fd. The immutable timing source was not edited. Diagnostic-only injected records contain every original/augmented certificate and every accepted alpha; the trace elapsed2.106s is not substituted for the baseline timing.

It is a FIRST-inner-solve stagnation, not repeated AL dual cycling. outerIterations1,dualUpdates0,physical length multipliers all0 throughout. Initial original force2141.1815,length0. There are72 accepted alphas, including45 at1/16384 and8 at2^-21. Last accepted direction: alpha2^-21,maxdirection7.54e-10,slope-1.37e-16, objective1621.249600399096→exactly the same Float64 value. The last augmented force4.60e-7/torque9.48e-8 remains above inner targets1e-8/1e-9; original force32.0704 and length0.00191828 remain unsatisfied with zero physical multipliers.1000 evaluations are consumed before the first dual update.

This directly motivates root's local mixed q/spin/length-wall multiplier solve and a merit evaluation not dominated by a large constant stored energy. It does not authorize dropping residual gates or treating tiny increments as convergence.

## Files and commands

Frozen root harness files and SHA values are in manifest.json. Baseline.json,analysis.json,cold310-trace.json,tests.txt are bundled. CPU profile remains at /tmp/oet-composite-timestep-initial.json.cpuprofile with its SHA recorded in the manifest.

Run a NEW snapshot/benchmark: node scripts/physics/benchmark-composite-timestep.mjs /tmp/oet-composite-timestep-next.json
Trace the existing baseline only: node scripts/physics/benchmark-composite-timestep.mjs --trace /tmp/oet-composite-timestep-snapshot-KhSyxw /tmp/oet-composite-timestep-cold310-trace-next.json

No production source, browser tab or server was changed. No further baseline repeats are needed before the solver/assembly changes; root can retest the same harness after its next integration.
