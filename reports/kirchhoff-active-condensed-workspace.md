# Active material condensation: persistent storage and exact full residual

The adjacent patch applies cleanly to the current root baseline and changes
only `kirchhoffActiveCondensedSolver.js`, the WAT/generated kernel bytes, and
adds `kirchhoffActiveCondensedWorkspace.test.js`. Also copy the new fixture
`tests/fixtures/kirchhoff-active-condensed-1999.json.gz` (the patch does not
embed binary files). Source hashes and all measured trial times are in the
adjacent JSON. Root files were only read; no root mutation or replay occurred.

## Change

Internal material factor/solve buffers, the full-matrix residual workspace,
index maps, response/coupling storage and local Schur buffers now grow
geometrically and are reused through
`options.workspace.activeCondensed`. An explicit
`options.condensedWorkspace` is also supported. The existing caller already
supplies a persistent `workspace`, so no world changes are required.

Every current matrix, material factor, response column, identity map, bound,
and group mapping is recomputed on every solve. Storage reuse is not factor
reuse or a physical force warm start. The response map is cleared each solve;
pooled columns are overwritten before use. All material coordinates and
responses outside the contact region remain included. Public force, residual,
bound, and free-mask arrays remain owned snapshots.

The new WASM `residualBandProfile` computes the full original residual in
exactly the original JS subtraction order: initialize from rhs, then process
each diagonal and its lower entries in increasing row/column order. Its
profile omits only exact leading zeros. The same original-matrix KKT and
final normal-load friction cone still control acceptance. No geometry,
tolerance, material stiffness, active-set rules or contact ownership changed.

## Evidence

The frozen common-relative case has 1,999 rows, band 150, 1,438 material
equalities and 102 retained rows. Both versions take 22 factorizations,
21 iterations and two expansions, with original KKT residual
3.81546321930222e-5 and zero cone violation at tolerance 2e-4.

The entire force increment and residual arrays retain the baseline SHA-256:

- Increment: `403c2d086511d18ee2bf36d3ecc368980a2cd7d9efe4a3a5424b978578fd557d`
- Residual: `7382dc03aa5fef01a9b29f5a1fd84f1cc6c5864147cef0540bf6f555df9db84d`

17 tests pass, including the existing 13 condensed tests and four new tests.
They independently reconstruct every original residual entry, check all
force bounds and final-load cone KKT, compare fresh/reused workspaces as
matrices, dimensions and group identities change, verify prior outputs stay
unchanged, and check zero-row/equality-only inputs. Six existing kernel
regressions also pass for skyline factors, the unchanged numerical floor,
equilibration/compression and multiplication order.

## Bounded timing

The supplied probe ran 30 solves per version with shared caller workspaces;
the first ten were excluded for warm statistics. Baseline and candidate ran
sequentially before the later CPU reservation. Node version: v24.6.0.

| Frozen linear solve | Baseline | Candidate |
| --- | ---: | ---: |
| Warm median, ms | 8.082 | 5.610 |
| Warm mean, ms | 7.918 | 5.624 |
| First trial, ms | 27.499 | 31.851 |

This is about a 31% lower warm median on this isolated case. Cold startup did
not improve. It is not a whole-world timing or an FPS claim. No larger CPU
benchmark was run during the parent's reserved replay window.

## Reproduction

```sh
OET_WABT_PATH=/private/tmp/oet-wasm-tools/node_modules/wabt node scripts/physics/build-linear-kernel.mjs
node --test tests/kirchhoffActiveCondensedSolver.test.js tests/kirchhoffActiveCondensedWorkspace.test.js
```

Preserved source baseline: `/tmp/oet-active-condensed-source-baseline`.
Frozen candidate: `/tmp/oet-active-condensed-workspace-frozen`.
Module-selectable copy of the supplied timing probe:
`/tmp/oet-benchmark-active-condensed-module.mjs` (arguments: absolute solver
module path, optional repetition count). The original supplied probe remains
unchanged and still targets root.
