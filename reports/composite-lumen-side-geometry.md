# Frozen lumen-side geometry

Exactly two NEW files. Existing detection, manifold, contact selection and solver files are unchanged. The helper calls neither `evaluateKirchhoffLumenSegmentContact` nor the manifold; it only differentiates a supplied original record.

```js
const workspace = createCompositeLumenSideGeometryWorkspace();
const geometry = differentiateCompositeLumenSideContact({ input, contact: original.side }, workspace);
```

`input` is the same argument object used for the original detector: inner/outer endpoints, lumen and inner radii, material IDs, and any quadrature/openDistal/fillet/feature options. Vector arrays, typed arrays and `{x,y,z}` are supported. Optional defaults match the detector, including rejection of explicit null values where the original default applies only to undefined.

Physical order is `[wire0.xyz, wire1.xyz, cat0.xyz, cat1.xyz]`. Output owns reusable buffers:

- `gapJacobian[12]` = G.
- `normalForceColumn[12]` = B = Gᵀ: inward wire forces and outward catheter forces per unit positive Fn.
- `forceColumn[12]` = -B for the mechanical residual.
- `normalDerivative[144]` = DB = Hgap, row major; `gapHessian[144]` contains the same entries in its own buffer.
- `outerTGradient[12]` and `normalJacobian[36]` differentiate the projection parameter and detector's OUTWARD radial normal. The normal Jacobian is 3×12, row major, and is distinct from the 12×12 physical force-column derivative.
- Points, normal, weights, scalar gap/clearance/distance, fixed sample coordinate, dynamic outer projection coordinate, local indices 0..11, and an owned `rawContact` provenance record.

With fixed inner sample s, p=(1-s)A+sW, d=D-C, v=p-C, t=(v·d)/|d|², r=p-C-t d and n=r/|r|:

```
g = clearance - |r|
dt = [d·(dp-dC) + (v-2td)·(dD-dC)] / |d|²
dr = dp-dC-t(dD-dC)-d dt
dn = (I-n nᵀ) dr / |r|
B = [-(1-s)n, -s n, (1-t)n, t n]
dB = [-(1-s)dn, -s dn, (1-t)dn-n dt, t dn+n dt]
```

The ±n dt terms are retained. Holding outerT/outer weights fixed gives the wrong Hessian under rotation of the outer segment.

Admission requires original `kind:'side'`, matching material/feature identity, exact membership of innerT in the supplied quadrature, strict 0<outerT<1 for both record and current projection, and a nonfallback radial normal. Gap, clearance, radial distance, outerT, normal and both sets of weights must match current input; original gradients are also validated if present. The existing point-to-segment squared-length threshold and radial-normal threshold are both 1e-12. Side eligibility under openDistal/fillet options is checked at the selected sample.

Endpoints/clamped projections, degenerate/fallback radial branches, distal rim/fillet records, portal-owned samples, stale geometry and invalid provenance are explicitly unsupported. Malformed input throws with the differential arrays already invalid. Any unsuccessful refresh leaves geometry/derivative arrays as NaN and `supported:false`; the static local DOF indices remain valid. No prior successful differential may be reused after failure.

**Selection scope is deliberately limited.** This is the derivative of the supplied frozen side sample. It never remaximizes over quadrature, changes the selected sample, certifies a unique winner, resolves sample ties, or handles runtime endpoint/portal/fillet transitions. `selectionCertified:false` and `certified:false` remain explicit even on success. A tie test confirms that the original detector's supplied sample is preserved; a later caller-selected sample gets a new branch signature. The generic physical→common/rho pullback can consume the 12D G/B/DB directly, while the caller/contact block owns selection and multiple-contact policy.

Validation: 11 new tests passed, together with the unchanged original detector test (12 Node test entries). Tests independently perturb the original detector in all 12 physical directions with a unique stable winner; rotate only the outer segment; verify G, outerT and full DB; test force/moment balance, translation null modes, rigid covariance and length-unit scaling; count detector/manifold calls; and exercise ownership, invalidation, ties and rejected branches. No additional detector calls or existing source edits occurred.
