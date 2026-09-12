# Rejected search-subspace performance experiment

A provisional principal subspace omitted unloaded relative increments, preserved all original constraint-Jacobian directions, and enriched from the full linear residual. It preserved the full accepted state and physical equations. Five mechanics tests passed, including independent feed/spin, transverse enrichment, curved geometry, loaded lumen Coulomb, second dt and atomic budget/query retry.

The measured benefit did not justify runtime integration. At 201 straight nodes, unknowns fell from 2006 to 1604, but whole-step median increased 13.871 → 14.220 ms. A transverse 33-node case increased 3.253 → 3.859 ms and needed 4 rather than 2 linear solves. A loaded 17-node case increased 36.399 → 37.653 ms. All A/B physical errors were zero or within the unchanged original tolerances. Measurements are synthetic Node cases, not anatomy/FPS.

The experiment has been removed from production source. `experiment.patch` retains the exact optional implementation, tests and benchmark script for isolated replay; it is not enabled in the application. `benchmark.json` and `experiment-tests.txt` preserve the results. Applying the patch requires the adjacent fixture `tests/fixtures/compositeRelativeSearch.js` already in the worktree. Do not infer final mechanical-model reduction or performance success from its smaller matrices.

The next reduction must reduce repeated material/contact assembly and preparation, not just the final factor. The separate production optimization in `../composite-lumen-needed-derivatives/README.md` removes unused derivatives without changing any contact or force equation.
