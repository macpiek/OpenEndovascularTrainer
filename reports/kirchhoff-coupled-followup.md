# Narrow capacity follow-up and deferred solver proposals

2026-09-06, cwd `/Users/macpiek/.codex/worktrees/f8f6/OpenEndovascularTrainer`.
The root has already copied the validated solver freeze. This follow-up changes
ONLY `kirchhoffCoupledLinearSolver.js`, `kirchhoffCoupledFrictionSolver.js`, plus
`tests/kirchhoffCoupledCapacity.test.js`. The previous two solver files are saved
in `/tmp/oet-coupled-frozen-20260906`. Apply the narrow patch in
`kirchhoff-coupled-geometric-capacity.patch`, or copy those two solver files and
the test. No parent files were modified by this task.

## Implemented: geometric numerical capacity

Row/vector and band-entry capacities grow to powers of two. The base QP keeps
its boolean free hints across replacement of a WASM memory instance. The cone
workspace and its reduced-matrix storage also grow geometrically. Material,
geometry, coordinates, forces and all current matrix/RHS values remain owned and
refreshed exactly as before. There is no factor reuse or physical warm start.
The tradeoff is reserved memory (less than two times each required capacity)
for fewer instance/array allocations. A growth replaces views without detaching
old WASM memory; ordinary solves still reinitialize their complete used prefixes.

Two deterministic allocation/solution tests cover 16→32 rows followed by
shrink/regrow for the box QP and the disk QP. Each path creates 3 outer WASM
instances rather than one at every row increase: initialization, row-capacity
growth, one independent band-storage growth. Counts 18–21 share one kernel.
Analytical solutions remain identical. This is an allocation count, not a CPU
speedup claim. Timed replay/microbenchmark was deliberately not run while the
root measured its baseline; the existing benchmark script is ready for the
same known 479-contact case when CPU ownership is free.

Validation: all 58 tests in CoupledCapacity, CoupledSystem, CoupledFrictionSolver
and BundleRuntime pass. The capacity patch changes no source outside the two numerical solvers.

A separate root-discovered correctness fix was subsequently mirrored in
`kirchhoffCoupledSystem.js`: `Array.from(members, ...)` instead of
`members.map(...)`. Integer typed row indices must not propagate their integer
storage type into mapped previous multipliers. The added regression uses
Uint32 rowIndices and fractional nonzero lambdas, checks total disk force and
plain-array parity. Root already applied the one-line source fix; the capacity
patch intentionally does not contain it. Full suite after this regression: 59 tests.

## Assessed only: reuse a factor as a preconditioner

Prefer this experiment before a cross-step force warm start. Always assemble
current J, W, alpha and current RHS. A cached factor may be an SPD preconditioner
for the CURRENT free-principal system, never an accepted lagged Jacobian solve.

A useful initial reuse gate should match stable row identities/order, free
principal membership, dimensions, scaling convention and (for cone tangents)
group representation. Reject after topology/material transport and large
normalized matrix changes. Treat that change bound as a performance heuristic,
not proof of physical accuracy. Every stopping test remains the original full
KKT and physical-reconstruction residual. Preserve cached scaling consistently:
if P=S_old A_old S_old, its physical inverse action is S_old P^-1 S_old r.
Do not accidentally feed new scaled coordinates into an old differently scaled
factor and assume the same operator.

For safety, a cached preconditioned direction p needs current-A descent control.
With residual r, cap its line step by (r·p)/(p·A_current·p), then by multiplier
bounds; near-null directions require proper bound handling. Alternatively use
PCG on a fixed free subproblem with rank/bound handling. The present unit-step
proximal iteration is safe for its freshly factored shifted matrix, but cannot
be assumed stable for an arbitrary old factor. Rebuild after a small number
(e.g. 2) of ineffective refinements, changed activity or failed descent. Measure
saved factorizations against extra matvec/triangular work. Full cone profiling
should establish whether this is worthwhile before implementation.

## Assessed only: consistent previous-load warm prediction

For a frozen translational J, the invariant is

```
q - qPred = W Jᵀ lambda.
```

An initial guess `lambdaWarm` must be paired with
`qWarm=qPred+W J_currentᵀ lambdaWarm`, including all material, normal, wall,
control and friction contributions. Local frame increments use quaternion exp.
Setting old multipliers without the matching generalized correction violates
stationarity. Fixed-step lambda scales with dt² if preserving previous force:
`lambdaWarm=lambdaOld*(dtNew/dtOld)²`. Recheck current normal feasibility and the
fixed-load friction envelope. Invalidate on transport/remesh until an audited
material/contact mapping exists. Any trust-region reduction must scale the
whole initial correction AND every warm multiplier together.

Balanced stress may yield a small net correction in hold, but this is a hypothesis.
For nonlinear XPBD, the current update integrates a sum of
`W J_kᵀ deltaLambda_k`, which is not generally `W J_finalᵀ lambda_final`.
Small constraint residuals alone therefore do not prove cold/warm trajectory
parity. Before implementation test the predictor/stationarity invariant,
loaded hold, unload, force/rotation reversal, topology invalidation, force/shape
and energy against the cold solution. A need for explicit nonlinear primal
stationarity repair is a separate solver change, not an allocation optimization.
No physical warm-start code was added here.
