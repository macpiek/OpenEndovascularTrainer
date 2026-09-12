# Normal lumen contact in the full joint step, with an exact affine-side envelope

This patch connects both actual physical axes to one joint band solve in `kirchhoffCompositeJointTimeStep.js`. It adds normal contact only: explicit fixed pairs and quadrature, fixed chart, quasi-static torsion and `friction:'none'`. The application does not yet use this operator. This is not a vessel, portal, continuous-containment, complete-physics or FPS certificate.

The new `kirchhoffCompositeJointLumenRows.js` owns each declared sample's original gap and physical Fn history. Every sample is queried independently through the existing detector with `quadrature:[s]`, then differentiated by `LumenSideGeometry` and mapped by `ContactPullback`. The common/relative residual receives `-Fn B` once; the joint tangent receives `-Fn DB` and the force column `-B`. Actual tool nodal loads close both separate material momentum balances.

## Contract and original acceptance

`contacts` is either `'none'`, or:

```js
{
  mode: 'lumen-normal', friction: 'none', chartId,
  forcePerLength: 1, // N/mm: a numerical NCP scale, NOT contact stiffness
  pairs: [{
    id, innerToolId: 'wire', outerToolId: 'catheter',
    innerEdge, outerEdge, innerMaterialSegmentId, outerMaterialSegmentId,
    lumenRadius, innerRadius, quadrature: [.25, .5, .75],
    openDistal: false, portalFilletRadius: 0
  }]
}
```

Each physical material pair/support must be unique; every quadrature coordinate must be unique and in [0,1]. Local support is at most two edges, as required by the existing pullback. Provenance freezes actual coordinates, active edge tools, mode nodes/bases, chart ID, material IDs, sample IDs, supports and radii. A changed chart cannot inherit Fn by retaining a human-readable chart ID. The numerical NCP scale is not physical provenance and may change without changing physical Fn.

The scalar equation is `R=(Fn-max(0,Fn-k*g))/k` in **mm**, evaluated without cancellation as `g` on the active branch and `Fn/k` on the inactive branch. Its multiplier derivative is respectively 0 or 1/k. Original acceptance requires nonnegative Fn, gap >= -1e-8 mm, |R| <=1e-8 mm and |Fn*g| <=1e-9 Nmm at every declared sample. Existing force (1e-7 N), torque (1e-8 Nmm), length (1e-8 mm), boundary (1e-9 mm), and original linear gates are unchanged. Changing k=.1/1/10 passes the same physical gates and gives the same physical response within those gates.

Private active Newton Fn values remain signed. For an already prepared inactive branch only, its known linear solution is Fn_target=0: the trial uses `(1-alpha)*Fn_base`. This is algebraic back-substitution, with no force clamp, sign test, force threshold, changed branch or penalty. Geometry, NCP and mechanics are recomputed for every trial and final commit.

Only a zero radial normal with **exactly zero Fn and strictly positive original gap** is eliminated; no normal is invented. Other unsupported original endpoint/clamped/portal/provenance branches reject. Contacts:'none' cannot silently discard a nonzero accepted normal history. Accepted histories own copied full sample maps and forces; query/direction/evaluation/late rejection returns the untouched input state and permits identical retry.

## Exact reduction of redundant inequalities

For one pair of straight segments on the strict interior outer-side branch, `P(s)=A+s(W-A)`, projection `t(s)` is affine, and radial vector `r(s)=P(s)-C-t(s)(D-C)` is affine. For the smallest/largest **declared** samples a,b and u=(s-a)/(b-a), convexity of the norm gives:

`g(s) = R-|r(s)| >= (1-u)g(a)+u*g(b)`.

Thus the two extreme declared inequalities imply the others. Their duals are the only lumen unknowns in the band. Interior duals use an explicit redundant-inequality gauge Fn=0 from initial preparation; all original samples, including those interior samples, still get fresh query/gap/NCP/work/force gates. This is not a winner/max sample or a tolerance-based rank prune. Single-sample pairs keep their one original row. Input ordering does not define the extremes.

A nonzero incoming interior Fn requires a separate transfer proof on the **incoming state's actual physical geometry, before new prescribed motion**. The implementation conservatively requires identical stored normals and exactly affine stored projection t, then verifies `B_s=(1-u)B_a+uB_b` with only a floating arithmetic error bound (8 epsilon times the sum of the interpolation magnitudes). Physical acceptance tolerances cannot authorize a transfer. If this cannot be proved, preparation rejects. Otherwise its Fn is added to the endpoint forces with weights (1-u),u. The original history is never mutated.

This preserves every physical nodal force, each tool's force/moment about any origin and all virtual work. It does **not** preserve the old geometric stress tangent. A regression has identical forces but maximum DB difference .5 for Fn [1,2,3] becoming [2,0,4]. The solver therefore always assembles fresh endpoint `-Fn DB`; it does not interpolate or reuse the old full-sample tangent or factors.

The independent model witness has exact rational rank 31 for the original 32x32 parallel-contact system, with dual null vector [1,-2,1]. Removing the proven redundant middle inequality yields rank31 for31 unknowns. The frozen original nonlinear baseline rejects that loaded step with `original-linear-equations`; the current endpoint-envelope step accepts it with all original gates. No pivot flooring, diagonal shift, penalty or relaxed tolerance is used.

## Validation and raw results

`tests/kirchhoffCompositeJointLumenTimeStep.test.js`: **13/13 PASS**. The complete affected Step + Assembly + RelativeDirection + ContactPullback + SideGeometry + new lumen set is **73/73 PASS**, 1.747 s in the recorded run. Both modified production files pass `node --check`. Raw output is in `composite-joint-lumen-tests.txt`.

Controls cover two loaded dt followed by release, each material's accepted velocities, separate feed/spin and fixed catheter boundary, independent original detector work and per-material impulse, k=.1/1/10, cold/reuse equality, radial-zero branch, duplicate semantics, unsupported sample, stale chart/material/basis, query/direction/evaluation budgets, late rollback and deterministic retry. Affine controls additionally check per-tool force/wrench/virtual work, changed DB, varying/opposite normals, unsorted samples, nontransferable loaded history and the actual parallel-loaded nonlinear dt.

`composite-joint-lumen-probe.json` contains paired cold/reuse and frozen unreduced results with state hashes, accepted status, per-tool positions/forces/balances and complete diagnostics. Synthetic n=3 physical fixture: coordinates [0,2,4] mm, wire density .13 and catheter .24 per reference arclength, original test stiffnesses [2,3,1] and [8,11,4], fixed material metrics; these are explicit test parameters, **not measured clinical material data**. No mass, stiffness or tolerance was changed for the 120 Hz control.

| Series | Accepted Fn sequence (first sample) | Newton directions | Original vs envelope position/Fn difference |
|---|---|---|---|
| dt=.1, loaded .4N/.4N/release | .259759843689 / .457000470078 / 0 | 3 / 3 / 3 | exactly 0 in all three |
| dt=1/120, loaded .4N/.4N/release | .375112183176 / .375048919002 / 0 | 3 / 2 / 2 | exactly 0 in all three |

All six paired cold/reuse results have bit-identical physical states and certificates. Release attempts limited to one direction reject with untouched previous Fn; successful release commits literal zero after signed private trials. Contacts:'none' state/certificate/balances are bit-identical to the earlier frozen no-contact workspace oracle.

The parallel fixture has wire [x+.25,.25,0], catheter [x,0,0], clearance .25, quadrature [.25,.5,.75] and old Fn [1,1,1]. The extra wire load gives an initial force residual .1 N. Current full dt accepts in 2 Newton directions / 2 fresh factorizations / 6 evaluations / 21 original queries (3 for old-geometry transfer,18 for trial/final certificates). It commits Fn [1.6084242672214486,0,1.4668316141618076], maximum original force residual 1.3227463e-10 N, length3.5172e-13 mm and minimum gap -3.2691e-13 mm. A second dt and late-rejection/retry controls pass. The original unreduced system rejects after its first direction; it is not counted as an executed dt.

Raw timings separately record constructor preparation, iteration work including gauge queries, and commit. This short correctness probe is not warmed or sized for a speed claim: host/JIT overhead is visible even for three nodes. Parallel current cold preparation1.797 ms +iteration5.061 ms, total6.949 ms; the rejected original total6.443 ms is not a performance competitor. The unknown reduction32→31 fixes singularity but does not itself establish the target full-device frame budget.

## Frozen evidence

The unreduced 9-lumen-test baseline was frozen **before** envelope edits at `/tmp/oet-composite-joint-lumen-unreduced/manifest.json`, SHA256 `938c28ca785cfb41ba1122c559b0980d077377c9f57099d1a61d47acb4db37b8`. It owns transitive sources, raw three-step controls and a self-contained probe. Prior manifests remain unchanged. The final envelope bundle is `/tmp/oet-composite-joint-lumen-final/manifest.json`; its exact hash is in the handoff/source manifest.
