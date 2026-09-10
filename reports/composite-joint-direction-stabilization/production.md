# Production globalization of the common Joint step

The initial .011 mm wall-penetration witness now succeeds in the actual World bridge, including a following dt with loaded wall/lumen material history. The previous unregularized result remains reproducible with `globalization:'newton'`; adaptive search is now the default. This closes that specific convergence defect, not the full anatomy/performance goal.

## Numerical method and unchanged physics

The normal path remains the original unshifted Newton band solve. If that numerical system fails, a line search fails, or an unshifted line search would try alpha below 2^-12, the same prepared timestep can activate diagonal stabilization of its scaled common/relative/dual search matrix. The numerical shift starts at .01 and may increase to .1 and 1. It persists only within that attempted dt. These are algorithm choices, not physical material parameters or weakened contact tolerances. Existing direction, evaluation, query and linear-solve budgets bound and account for every attempt.

`solveCompositeRelativeDirection` now accepts explicit `numericalShift`, default zero. Original matrix A, residual F, residual array and `proof` remain unmodified in meaning; its `converged` still means the original unshifted linear equations converge. The separate `numericalProof` / `numericalConverged` describe the modified search system. An incompatible held original row remains visible in both proofs; fixed and held increments remain exactly zero. No diagonal is inserted into the physical assembly or final acceptance law.

The full timestep may use a converged regularized search direction only as a trial direction. It still rebuilds original material/contact residuals and applies the original nonlinear merit and fresh final certificate, including forces, torques, lengths, boundaries, gap inequalities, NCP/complementarity, friction cone/slip/work and their original tolerances. Thus a regularized linear solve is explicitly not a certificate of physical equilibrium.

Before restarting a failed/tiny line search, the code restores positions, independent spins, relative coordinates, length and boundary multipliers, normal reactions, both lumen and wall tractions and the wall chart checkpoint. It then refreshes row mapping and reassembles at the same base state. No command is re-prepared and no simulation time is consumed during that restart. The outer World transaction remains accepted-only.

## Evidence

- Three new direction tests compare the shifted solve with an independently assembled dense system, explicitly verify its distinct unshifted residual, preserve original A/F, reject incompatible held rows and invalid shifts, and verify reuse returns to the original unshifted result.
- Three new World tests show ordinary adaptive/Newton states, reactions, certificates and counts are identical for three dt with no stabilization; the penetrating case now accepts and continues with loaded history; direction/linear/evaluation/query budget failures during stabilization retain both bodies and one pending dt, with exact retry equality.
- The old explicit Newton penetration failure remains a negative rollback control. It is no longer presented as the default algorithm's outcome.
- The repaired witness takes 11 directions / 91 evaluations, with one early activation at .01. Final original force residual is 4.543312510009988e-8 (limit 1e-7), and the remaining original physical certificates converge. This large evaluation count is a robustness result, not a real-time result.

Root `npm run test:physics:composite`: **752/752 PASS**, 16.869 s. Build PASS, 1.89 s, with the existing bundle-size warning. `production-source.json` records current sources and logs. The full composite suite includes existing pure-direction failure tests, all whole-step friction tests and the new World tests. The actual simulator still does not select this new solver; original vessel/sheath/portal input mapping, per-surface static/kinetic friction, full feed/profile/mesh lifecycle, adaptive reduction and deep/max browser performance remain unfinished.
