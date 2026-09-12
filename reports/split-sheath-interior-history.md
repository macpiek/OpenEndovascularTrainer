# Exact interior sheath history — bounded prototype handoff

Stable contact with the interior of the existing analytic sheath now uses exact historical cylindrical clearance and can certify. This replaces the previous generic `sheath-tangent-plane` limitation for that supported case. End transitions, geometry/membership changes and material identity changes remain explicitly unverified; this is not sheath CCD or a new release solver.

Only `kirchhoffCoupledBoundaryRows.js` and `kirchhoffSplitMotion.js` production sources change. World is untouched. No collision query, axial cutoff, radius clearance, force gradient, compliance, material profile, time step or tolerance was changed. The existing sheath has radial normal rows and no tangential friction law; this implementation does not add one.

## Historical witness and certification

At the beginning of the physical step, split state copies each sheath's origin, unit axis, effective axial interval, inner radius, selected bodies, active supported-node ranges, node radii and node material coordinates. The axial interval uses the existing exact inclusion rules: `[-proximalExtension-1e-5, length+1e-5]`. At boundary collection, the same current numeric geometry and each radial witness are attached to the owned boundary state/row.

For the same material node and unchanged sheath, the historical geometry is evaluated directly:

```text
u_start = x_start - origin
axial_start = dot(u_start, axis)
r_start = length(u_start - axial_start * axis)
g_start = max(0, innerRadius - nodeRadius_start) - r_start
physical_gap = max(0, g_start) + dt * J_current * v_physical
```

Bias still sees the original actual radial geometric gap. Current J and its gradients are the existing analytic radial normal. No old normal-displacement projection is substituted for `g_start`.

History audit visits supported material nodes even when the current collector omits their row outside the axial slab. Crossing between interior and proximal/distal exterior near the radial contact region marks `sheath-axial-feature-changed`. It does not infer a valid release from zeroing/discarding a current row. A changed sheath origin/axis/radius/interval, selected body/range/radius/material coordinate, or membership is likewise unverified. Degenerate loaded radial witnesses cannot silently certify. Unsupported cases retain `certified=false` and `historyCommits=0`; there is no impulse migration/reconciliation or whole-step rollback in this patch.

Final outward physical-velocity certification now includes actual sheath rows. Public `jointMotion.contacts` includes `kind: 'sheath'`, ids `sheath:<snapshot-index>:<side>:<node>`, physical and bias normal multipliers, and zero tangential multipliers/coefficient. Reaction units remain XPBD multiplier; impulse is lambda/dt.

## Independent analytic checks

The World fixtures use a free three-node wire, a fixed five-node companion, dt=1/120, unit node mass, wire radius .25, sheath inner radius .75, x-axis sheath from 0 to 5 with proximal extension 2, and the already integrated preserve-strain bias option. All physical inputs and World gates are unchanged.

| Case | Actual raw radial gaps, mm | Physical motion | Sum sheath lambda physical / bias | Final certificate |
|---|---|---|---|---|
| Bias-only initial y=.75, vx=4 | [0,0,0] | vx=4 exactly; vy=0; omega=0 | 0 / .7499999965 | true, history1 |
| Touching y=.5, incoming vy=1, vx=4 | [0,0,0] | vx=4; max vy=6.25e-9; max omega=1.26e-9 | .024999976012416845 / 0 | true, history1 |
| Next dt away vy=-2, vx=3 | [.0166666805744,...] | vx≈2.99999809, vy≈-2.00000167; max omega=6.19e-10 | 0 / 0 | true, history1 |

The incoming normal impulse `sum(lambda)/dt` agrees with independently computed radial momentum loss within the existing Float32 fixture tolerance. Physical KKT residual is at most 5.21e-11 mm in these three cases, and bias elastic energy change is zero. Axial motion experiences no invented drag.

An independent oblique-axis test uses origin [2,-3,4], axis [.6,.8,0] and material points whose radial witness turns while preserving radius. The oracle obtains distance from `|cross(x_start-origin,axis)|/|axis|`, independently of the production projection formula. Exact start gaps are approximately [-4.77e-8,-1.08e-7,-1.48e-7] mm (Float32 positions); the old tangent-plane construction instead gives approximately [.019999926,.019999853,.019999815] mm. The new history agrees with the cross-product oracle to 1e-12 mm.

Four explicit unsupported tests pass: a tip entering through the distal end, a previously loaded row disappearing beyond the distal end, sheath geometry changing during the step, and material labels changing at an unchanged runtime node. Each gets final certificate false/history0. In the loaded exit, the dropped row's impulse is not reconciled; the unsupported flag is the result, not a claim of a correct force-free release.

## Frozen verification and scope

- New sheath tests: **8/8 PASS**, `/tmp/oet-sheath-interior-tests-8996.txt`.
- Existing independent World+wall oracles, unchanged assertions/fixtures and preserve-strain opt-in: **11/11 PASS**, `/tmp/oet-sheath-interior-world-wall-8996.txt`.
- Existing BoundaryRows and split certification/preserve-strain tests: **9/9 PASS**, `/tmp/oet-sheath-interior-default-8996.txt`.
- Raw diagnostic observations: `/tmp/oet-sheath-interior-observations-8996.json`.

Reproduce the new checks and their optional data export:

```sh
OET_SHEATH_HISTORY_REPORT=/tmp/oet-sheath-interior-observations.json node --test --test-concurrency=1 tests/kirchhoffSplitSheathHistory.test.js
```

No anatomy, full scenario, smoke replay, outer reclosure or World transaction was run or changed. The root smoke's outward-contact velocity still requires separate physical closure; this handoff establishes exact history only for a stable analytic interior sheath witness. Existing wall-foot ownership and other split-motion limitations remain outside this patch.
