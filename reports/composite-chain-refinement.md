# Cold310.017 composite solve — frozen numerical correction

The cold cached-material direction now passes the ORIGINAL1e-7 absolute maximum free-row residual threshold in both JavaScript and WASM element backends. One refinement correction reuses the same unshifted Cholesky factor. No pivot shift/floor, tolerance change, norm scaling or constitutive change was made.

## Independent frozen-matrix diagnosis

The341DOF/band13 matrix and RHS SHA is f45421d27acec32b08800f5b60c5bcc0c97ff5c950f933b1a0718affc8829b8a in both backends before and after the patch. Mandatory wire/core and catheter/plateau boundaries are294mm and294.017mm; the0.017mm interval amplifies arithmetic error.

| Quantity | Before | After one refinement |
| --- | --- | --- |
| Application original residual | 1.8032869775e-7 (naive product/sum) | 5.8007439121e-8 (compensated) |
| Independent Decimal80 original residual of supplied Float64 direction | 1.2418481922e-7 | 5.8007439121e-8 |
| Maximum absolute direction error against Decimal80 LDL | 7.4841323339e-10 | 2.3923265752e-16 |
| Status at tolerance1e-7 | failed | converged |
| Factorizations / triangular solves | 1 / 1 | 1 / 2 |

The independent80-digit unshifted LDL solution has residual3.6730e-71. Rounding that solution once to Float64 gives residual5.8007315732e-8. Thus improved summation alone would not fix the old direction: its TRUE1.24185e-7 residual still exceeded the threshold. A Float64 solution satisfying the unchanged criterion exists, and one correction reaches the representation floor. The application and independent Decimal oracle agree on the final residual.

The Python probe uses Decimal.from_float for every original matrix/RHS bit and its own LDL factorization, with no production kernel/equilibration/Cholesky reuse. The Node regression additionally computes every row's residual by exact BigInt products and summation of binary64 dyadic rationals; it requires no Python at test runtime. The frozen JSON contains the original constitutive H/gradient, declared diagonal3 and independently rounded reference direction, so source changes cannot silently replace the witness.

## Implementation contract

- Original-matrix residual uses compensated summation AND error-free product splitting. A bit-truncation fallback prevents the splitter multiplication from overflowing for very large finite matrix entries. The final finite guard still rejects nonfinite original residuals.
- Residual is evaluated row by row in original physical units. Only free rows participate in the unchanged maximum-absolute norm. Fixed increments remain exactly0 and fixed reactions remain minus the ORIGINAL matrix/RHS residual.
- When that norm fails, at most2 corrections by default solve A*delta=r using the existing equilibrated factor. Existing residual storage is temporarily borrowed for the correction; no matrix/RHS/factor mutation or new per-step workspace buffers are introduced. The original residual is recomputed before deciding acceptance.
- Optional maxRefinementSteps is an integer0..3 (default2). Diagnostics add initialMaximumResidual,refinementSteps,linearSolves. Existing return fields/criteria remain compatible. factorization count stays1. Root was notified to relay this additive API to the TimeStep task.
- A tighter-than-representable criterion stays converged:false after its exact correction budget. A singular original pivot stays rejected regardless of refinement budget. Refinement is not a nonlinear step or a substitute for physical boundary/inertia terms.

## Validation and cost

76/76 tests passed across Topology, Element, Chain, FastElement, Mesh, MaterialCache and9 new refinement regressions. Tests include both element backends, real cold geometry, frozen exact A/RHS and independent high-precision direction, original load/matrix/factor immutability, fixed reactions/zero increments, finite extreme1e300 entries, explicit iteration budget and still-rejected singular modes. Syntax and git diff --check passed. The earlier frozen cache/Mesh sources remain byte-identical.

Short paired solve-only probes measure the EXTRA refinement with accurate residual in both arms: JS0.222→0.286ms and WASM0.292→0.391ms; unrefined attempts remain explicitly FAILED. These are six paired measurements after four warmup pairs, not a runtime/FPS certificate.

The unchanged full material-cache benchmark rerun with this Chain reports all27 pairs (warmup+measured) converged in both arms, including the previously failed cold case. Median cached topology+Mesh+workspace+assembly+solve: cat9/160/310mm =2.217/3.628/3.864ms in this host-noisy short run; paired uncached values13.141/24.744/31.448ms. Source hashes are stable. All original pose/input hashes and operator parity checks remain in pipeline.json. These measurements still exclude accepted-dt dynamics/contact/remesh history transfer and do not assert60FPS.

Reproduce tests: node --test --test-concurrency=1 tests/kirchhoffCompositeChain.test.js tests/kirchhoffCompositeChainRefinement.test.js
Reproduce independent oracle: node scripts/physics/probe-composite-cold-residual.mjs /tmp/oet-composite-cold-residual.json

Only existing source changed: src/physics/kirchhoffCompositeChain.js. The other five files are new regression/probe/fixture files. No Cache/Mesh/Simulator/World/Element or other root module edits; no browser/server actions. Root may register the new test and integrate the frozen Chain.
