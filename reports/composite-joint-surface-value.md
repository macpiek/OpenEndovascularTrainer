# Value-only surface evaluation for nonlinear trials

The existing finite surface increment, instantaneous physical force map and
`JointLumenSurface` now accept `order:'full'|'value'`, defaulting to `full`.
The value path returns the **same finite increment and physical B**, while
omitting G/DB that a residual-only line-search trial does not use. It changes
evaluation order, not the geometry, transport rule, physical map, material
history, contact policy, tolerances or nonlinear acceptance criteria.

Only the owned SurfaceMotion and JointLumenSurface sources, their two tests
and new reports changed. The manager and JointTimeStep were not edited.
Earlier immutable baselines remain unchanged.

## Shared algebra with fewer derivatives

`evaluateCompositeJointSurfaceIncrement` evaluates the same finite algebra
against either its existing first-order arena or a new dimension-zero scalar
arena. The scalar arena has an explicit `variable → constant` adapter: it
never writes or reads a fake derivative entry adjacent to scalar storage.
The entire geometric/reference-frame/material-map validation path remains.

`evaluateCompositeJointSurfaceForceMap` uses the existing second-order triad
arena and differentiated output arena for full B/DB. Value mode instead uses
a **first-order triad arena in all 7T configurations**, because those director
derivatives are physically required for B, and a scalar-only output arena.
Its identical `omega_j = 1/2 Σ d×d_j` and force-lever algebra still computes B.
It omits only the triad Hessian and the derivative of B. There is no repeated
unit-rate evaluation, approximation, frozen numerical operator, numeric FD,
zero angular velocity, diagonal substitution or duplicated physics formula.

The added arenas are allocated once by each workspace constructor. A
two-tool lumen workspace adds 304 KiB of reusable typed arena storage, shared
across samples and time steps, not per contact. The existing owned diagnostic
responses still allocate small result objects/arrays; this patch does not
claim allocation-free evaluation.

The lumen wrapper skips its current witness derivative, query-chain assembly,
G and DB loops in value mode. It still calls the existing `SideGeometry` twice
to validate the two original records; that existing helper also computes its
small 12D derivatives. This retained cost is included in the reported timings.
No additional detector queries are introduced, and neither current nor
previous provenance/history checks are skipped.

## Explicit validity contract

The two direct SurfaceMotion exports return null derivative arrays in value
mode, while retaining their physical configuration/query dimension metadata.
The finite increment has `incrementValid:true`, all Jacobian validity flags
false and `nonlinearReady:false`. The physical B response has
`forceMapValid:true` and all derivative validity flags false. Full mode keeps
the numerical derivative outputs and sets those flags true.

`JointLumenSurface` continues to expose its reusable preallocated buffers:

| Value-mode field | State |
| --- | --- |
| `increment`, `incrementValid` | finite, true |
| `forceMap`, `forceMapValid` | finite, true |
| `slipJacobian`, `slipJacobianValid` | all NaN, false |
| `DforceMap`, `DforceMapValid` | all NaN, false |
| `currentQueryJacobian` | all NaN |
| `operatorReady` | false |
| `supported` | true after successful value evaluation |

Every refresh invalidates the prior scratch first, including invalid order
and late geometry/history failures. A subsequent full evaluation rebuilds all
derivatives from the current geometry; no G/DB is retained across value calls.

For a value-only call the manager should pass only the valid B to pullback:

```js
const surface = evaluateCompositeJointLumenSurface({ ...input, order: 'value' }, workspace);
pullbackCompositeJointSurface({
  tools: surface.tools,
  forceMap: surface.forceMap,
  forceMapValid: surface.forceMapValid
}, pullback);
```

Passing the whole value result, including present-but-invalid G/DB, correctly
rejects. Full mode continues to pass the entire provider output as before.
The physical force map remains distinct from the finite slip Jacobian.

## Correctness and paired cost

**40/40 owned tests** (29 SurfaceMotion and 11 JointLumenSurface), and
**64/64 affected tests**, pass with syntax checks. All pre-existing full
derivative, common-rigid-motion, winding, independent feed, current-label
transport and virtual-power tests remain. Four new tests verify bit-identical
full/value finite increments and B for one/two tools, ownership, no fabricated
derivative arrays, exact reuse after full→value→invalid→full, current/previous
geometry/history rejection, force-only pullback and shared arena reuse.

The frozen benchmark additionally compares against the unmodified **prior
full implementation** on the same original fixture and six freshly detected
configuration perturbations. All five full numeric buffers are bit-identical:
increment, B, G, DB and current-query chain map. The value increment and B are
bit-identical to that prior implementation on all seven cases.

One sequential process measures 30 warmups and 200 paired repetitions per
method, rotating oldFull/newFull/newValue order. New full and value share one
workspace, alternating evaluation order. The immutable detector input, own
profiles/maps/angles and source hashes are checked before and after. The
measured region is the **whole local provider call**, including validation,
both original-record geometry checks, witness construction and its full or
value surface evaluation. Prepared input and workspace creation are outside
the timed region. There are zero measured detector queries.

| Provider call | Mean ms | Median ms | P95 ms |
| --- | ---: | ---: | ---: |
| Previous full | 0.420508 | 0.407416 | 0.490666 |
| New default full | 0.404959 | 0.388167 | 0.473042 |
| New value | 0.125771 | 0.120125 | 0.173375 |

Value mode reduces the measured median call cost by **70.52%**, or **3.39×**,
relative to the prior full call. The smaller full/full timing difference is
not claimed as a separate optimization. Raw paired durations, exact hashes,
fixture identity and parity checks are preserved in
`composite-joint-surface-value-benchmark.json`. This measures a single local
contact-provider call, not a full accepted/rejected dt or rendering; it does
not establish FPS.

The immutable bundle is `/tmp/oet-composite-joint-surface-value-final`, with
both new sources/tests, frozen transitive dependencies, the benchmark,
reports and SHA-256 manifest. Its benchmark identifies the prior immutable
bundle `/tmp/oet-composite-joint-lumen-surface-final` by manifest hash.
The parent's manager can now request value mode for residual-only trials and
fresh full mode before assembling the physical Newton direction.
