# Relative motion: two transverse coordinates are a reduction

Read-only diagnosis of the current Chain, TimeStep, Kinematics, RelativeCluster, LengthConstraints and plan sections 4–5. No production change belongs to this diagnosis.

**Decision:** preserve full three-dimensional relative displacement at every represented overlap node for the reference model. Solve all common, relative, independent-spin and original constraint equations in one local band. Two transverse coordinates with frozen material maps are a reduced model; they are not a complete change of variables.

## Geometric counterexample

Let the catheter be fixed at `q_i=(i h,0,0)` with edge rest length `h`, and reconstruct wire `y_i=q_i+B rho_i` using a fixed transverse basis. With the same wire edge rest length,

`|y_(i+1)-y_i|² = h² + |rho_(i+1)-rho_i|²`.

Exact wire inextensibility therefore requires every transverse offset to be constant. At zero offset the first derivative of wire length with respect to transverse rho is zero: a first-order direction can appear admissible while the finite configuration violates length at second order. Adding the same length row twice on the coincident geometry can also make the joint constraint Jacobian rank deficient. Current `TimeStep` allocates one length multiplier per common edge and checks only `|delta q|-delta x`; `RelativeCluster` does not add a wire length constraint. Thus current common-length success does not certify an offset wire's length.

For `h=2 mm`, offsets `[0, 0.0405, 0] mm` give wire edges `2.00041002047 mm`, a residual of `0.00041002047 mm` per edge. This fails a `1e-8 mm` length gate despite lying within the stated radial clearance. At `h=1 mm`, neighboring offsets on opposite sides of the lumen (`±0.0405 mm`) give a `0.00327514 mm` edge residual, also exceeding the existing `0.002 mm` runtime geometry gate.

A third relative coordinate solves this local geometry without stretching. Set axial increments

`delta a = sqrt(h² - |delta rho|²) - h`.

For the first example, `delta a=-0.000410104546 mm`. Wire points are `[0,0,0]`, `[1.999589895454,0.0405,0]`, `[3.999179790907,0,0]`; both wire lengths are exactly `2 mm` and catheter lengths remain `2 mm`. The wire's distal point slides axially by `-0.000820209093 mm`. The distal wire boundary is free to slide in this witness. Fixing both wire endpoints at the catheter's equally distant endpoints would physically prevent bending even in a complete model; that is a separate taut-boundary incompatibility.

## Two possible complete descriptions

| Description | Full local motion | What changes |
| --- | --- | --- |
| Common `q` plus full relative vector `d` | `y_cat=q`, `y_wire=q+d`; invertible linear change of six physical position coordinates into `q,d` | Existing per-material element/inertia derivatives can be pulled back directly. Keep separate tool geometry, rest-length constraints and histories. |
| Common `q`, two normal coordinates and an unknown wire material label field `s_wire(x,t)` | The label field supplies the missing tangential material motion through the shape | Requires derivatives of label transport, material sampling, `u=-s_t/s_x`, integration weights, history interpolation and frame transport. Current frozen-map kernels are insufficient for this unknown. |

The second representation is possible in a valid local cross-section chart, but it still needs the missing scalar field; it is not a five-coordinate full model. Removing that field by integrating arc lengths from a boundary creates prefix/global dependencies unless a local mixed formulation is retained. Updating labels from geometry after the solve without its momentum/reaction equations would hide axial stretch or locking. Keeping only the current external `dsDt` input does not fix this: it permits prepared feed on the coincident curve, but cannot supply candidate-dependent material redistribution caused by varying offsets.

The first description is the implementable next step. Use a fixed complete orthonormal basis `Q` (the world basis is sufficient) and `d=Q rho`, with three scalar coordinates. `Q` stays fixed during Newton, so `y_wire=q+Q rho` has a constant exact Jacobian and energy/Hessian pullback needs no missing basis derivatives. At overlap nodes, all three relative coordinates are required for a full-rank reference; allowing two at selected nodes explicitly reduces physical motion there.

## Nonlinear assembly and state contract

Next structural API: each mode carries `{node, basis:[b0,b1] or [b0,b1,b2], relativeDofs}` with contiguous offsets determined by the sum of preceding mode dimensions. Three-dimensional bases need not be transverse to the current chord; two-dimensional bases retain the existing transverse restriction and reduced scope. Workspace structure freezes dimensions/bases while each direction supplies fresh H/C. This structural generalization remains a linearization at zero relative offset until a separate nonlinear assembler exists.

For that assembler, use per-material position maps, e.g. catheter `q`, wire `q+d`, independent spins, physical rest lengths per tool, and separate current/previous positions. Evaluate each affected physical material hinge and each physical inertia edge once at its actual geometry, then scatter the exact pullback. The common q block is the sum of those current contributions; it must not retain an old coincident-wire block and add the shifted wire a second time. Root's new per-material compiled inertia evaluator is suitable for this path.

Each overlap edge needs original catheter and wire length equations using their respective reconstructed endpoints and physical rest spans, with independent signed multipliers. If labels are physical rest-length coordinates, the target is their material span. If labels have another scale, require an explicit rest-length metric `d ell0/ds`; neither silently equating `delta s` to `delta x` nor treating arbitrary `dsDx` as physical stretch is sound. The current one-common-length operator remains only the coincident-axis approximation. At three-coordinate overlap, axial relative columns distinguish the two length Jacobians even at zero offset.

Physical wire velocity is `v_wire = q_t + d_t + u_wire (q_x+d_x)`, while catheter velocity uses its own shape and map. Previous positions must be each material's previous spatial geometry at the prepared mesh coordinate, and old physical velocities must be sampled at each current material label. Independent reference tangents/directors and reference-twist winding histories must follow each actual material axis. Current TimeStep's single shared `reference`, `acceptedFrames`, and common-geometry velocity commit are insufficient once axes differ. Keep these histories transactional, frozen through all trial evaluations and committed together only after original equations pass.

Feed must have one stated coordinate convention. Within a prepared fixed-topology step, a frozen map can account for the prescribed through-mesh transport while d represents physical relative shape/displacement. Do not also apply the same feed as a duplicate axial displacement. Over longer slip, keep correspondence local through an explicit accepted rechart/material transfer at exact tips/material boundaries. If a material/cell crossing changes the prepared local support, rebuild that chart with validated transfer; do not clamp axial d or move labels silently. A full material-following chart with fixed labels is also valid, but then feed is carried by its physical boundary motion and insertion/remeshing, not by a second independent map advection.

At a tip/interface, represent both endpoint positions whenever both materials are present. On a one-material section, q is that material's position and no relative coordinates are needed. Reconstruct physical wire continuity using its own mapped endpoint across the interface. The current interior-only Cluster modes do not yet provide this endpoint/history machinery; a full-rank claim is limited to the overlap nodes actually represented.

## Performance and acceptance

Adding the third scalar increases an overlap node from `3 q + 2 rho + 2 spins = 7` primal coordinates to eight. A second physical length multiplier adds another local row. A bounded number of coordinates/constraints per node still gives bounded bandwidth and linear storage in one monolithic LU; there is no alternating rod solve or global Schur elimination. Full finite-offset wire/catheter geometry may require separate local element evaluations. This is a correctness reference, not proof of the 4/6 ms budget.

Only after its force/shape/slip/contact histories agree with an independent full reference should modes be reduced or condensed under an explicit error budget, as plan section 5.3 requires. Safe reduction must test original wire and catheter lengths, axial force/momentum balance, independent feed/slip, contact gap and work, not merely the residual of the reduced equations. This diagnosis deliberately does not declare any nonlinear step, FPS target, remesh transfer or complete finite-clearance model achieved.
