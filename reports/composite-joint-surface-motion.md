# Physical affine surface motion: instantaneous operator and finite parallel control

This bounded prototype uses each tool's **own actual affine physical axis**, its own accepted frame and unwrapped spin. It introduces no quadratic/quintic centerline, interpolation framework, geometry node or change to Step/Assembly/World. It is not integrated friction and does not certify a nonlinear physical dt or FPS target.

There are two explicitly different exports in `kirchhoffCompositeJointSurfaceMotion.js`:

1. `evaluateCompositeJointSurfaceMotion(input, workspace?)`: instantaneous surface velocity with exact configuration/rate AD Jacobians and a virtual-power force map.
2. `evaluateCompositeJointParallelSurfaceIncrement(input)`: finite **parallel-axis** contact-frame control, explicitly `nonlinearReady:false`, `finiteJacobian:null`. This establishes the finite-motion rule and its limit; it is not a general Newton friction row.

## Instantaneous physical operator

Each of one or two tools supplies two current/previous physical endpoints, their fixed coordinates and query coordinate, its own accepted reference tangent/director, own current/previous unwrapped angle and **explicit instantaneous** endpoint/angle rates. For one tool, the wall velocity at the same point is explicit; a stationary wall must provide [0,0,0]. Both levers derive from the same supplied world contact point and the two distinct affine centerline feet. Contact tangent axes and feet are frozen query inputs for this local differentiation; it is not the total derivative of a detector whose selected feet/point/axes also move.

```js
{
  dt, rateMode: 'instantaneous',
  contact: {point: [x,y,z], axes: [[3],[3]]},
  tools: [{
    id, edgeId, coordinates: [x0,x1], coordinate,
    positions: [[3],[3]], previousPositions: [[3],[3]],
    reference: {tangent:[3], director:[3]},
    angle, previousAngle, positionRates:[[3],[3]], angleRate,
    materialMap: {sStart, dsDx, dsDt},
    materialPath: {
      kind:'linear-affine-maps', previousEdgeId,
      previousMap:{sStart,dsDx}, previousTrace // only at a boundary
    },
    trace // only at a boundary
  }],
  wall: {velocity:[3]} // only for one-tool mode
}
```

The material map is affine in space and is explicitly interpolated linearly between the two history times. Its supplied query dsDt must agree with that path, up to an arithmetic error bound. The same **current material label** locates the old material point; label motion is not applied twice. Both end fractions must remain in the same actual edge. Positive affine slopes make this inverse label path monotone, so the interval endpoints establish the same-edge condition. At an edge endpoint, `left`/`right` traces are explicit. A crossed edge/hinge or missing angular/rate history rejects with required-transport details instead of setting unknown rotation to zero.

Inside one open affine DER edge its orientation field is spatially constant. This is a statement about that physical discrete field, not an approximation inferred from equal samples. It has no artificial midpoint seam. Its time frame is its own accepted director parallel-transported onto its current tangent, followed by its own unwrapped spin. With physical material triad d_j:

```
v       = N0*qdot0 + N1*qdot1 + u*q_x,  u=-s_t/s_x
omega   = 1/2 sum_j d_j cross (D_configuration d_j * generalizedRates)
lever_i = commonContactPoint - ownAffineFoot_i
v_side  = v + omega cross lever
slipRate= tangentAxes dot (v_side_inner-v_side_outer)
```

The second AD arena differentiates the time-PT chart; first derivatives of the velocity include all needed second frame derivatives. No runtime finite difference is used. Local configurations are `[q0.xyz,q1.xyz,theta]` per tool, followed by their independent generalized rates. Two tools have14 configuration and14 rate columns. `configurationJacobian` and `rateJacobian` are separate; the physical instantaneous force map is **transpose(rateJacobian)**. Configuration and BE-increment derivatives must not be substituted for that work-conjugate map. Prescribed feed/wall velocity contributes explicit additional power.

This rate operator does not infer a finite rotation path from endpoint chords. Its outputs say `finiteStepSlipKnown:false`; returned arrays are owned and survive workspace reuse. Previous geometry/reference mismatch, nonfinite/overflow data and unsupported material paths reject without modifying input histories.

## Why the old finite velocity construction is insufficient

For a common finite rotation R, centerline BE velocity differences use `(I-R^-1)(c_inner-c_outer)/dt`. A single angular cross-product is skew; the finite chord operator generally is not. Combining the centerline chord with angle/dt rotation is not the derivative of one rigid path.

A concrete regular contact witness has parallel +x axes, radii .25/.5 mm and one shared surface point. Both axes undergo a common90° rotation about x in1s. BE center velocity plus the correct unwrapped spin rate gives a **false tangential relative rate -0.1426990817 mm/s**. True instantaneous rigid rates give zero. The new contact-frame finite control gives exactly zero for this witness.

Using only endpoint relative material poses H=F_outer^-1 F_inner does not preserve all winding either. With both centers fixed and both own spins +2π, H_new=H_old and relative winding is0. Actual slip at the same contact point is nevertheless `2π*(r_inner-r_outer)=-π/2 mm`. Both separate spin histories are necessary. A surface-position chord or principal relative log misses this loop.

## Finite parallel contact-frame rule

The finite control requires `rotationPath:'short-contact-frame-own-unwrapped-spins'` and positive fixed radii. It uses parallel affine axes in both states, regular radial separation, and no material hinge crossing. Each old center is sampled at the same current physical material label. It does **not** require an invented instantaneous angular velocity or endpoint rate.

Let C=(n,t cross n,t), where t is the outer physical tangent and n is the radial component of the actual center separation. This geometric contact frame transforms with common world rigid motion. Parallel-transport old n along the shortest regular old-tangent→new-tangent chart, then measure the remaining signed normal turn beta about new t. The normal-turn branch is short and rejects its ambiguous π boundary. For each material:

```
deltaPhi_i_in_C = (theta_i_new-theta_i_old) - beta
D_new = C_new^T (center_inner_new-center_outer_new)
D_old = C_old^T (center_inner_old_at_current_label-center_outer_old_at_current_label)
increment_axial = D_new.axial-D_old.axial
increment_circ  = D_new.circ-D_old.circ
                + r_inner*deltaPhi_inner_in_C - r_outer*deltaPhi_outer_in_C
```

A stationary-plane, one-tool variant has an explicit fixed wall normal/tangent/point and zero wall velocity. Its outer radius/spin are the known stationary-wall values, not missing tool data.

Common finite rigid motion makes both contact-frame center differences and both relative-to-contact spin increments zero, including unequal levers. Fixed-center own spin loops keep their separate2π increments. Opposite material feed/spin agrees with the instantaneous velocity rule. Contact-frame transport is an explicitly chosen **discrete rule**; endpoint data cannot distinguish an unknown complete common geometric orbit from no orbit. The short-branch contract excludes that ambiguity rather than claiming to reconstruct an arbitrary unknown trajectory.

The finite control currently has no full configuration Jacobian outside the restricted parallel chart, and says so explicitly. Its fixed-radius parallel rule and stationary-plane variant do not generalize automatically to arbitrary nonparallel deforming contacts. A chart-validity guard is not an error certificate for finite general surface transport. The next nonlinear friction rows must differentiate the same chosen motion and combine detector feet/contact-frame derivatives with the physical virtual-power force map.

## Independent validation and the next material-history change

**13 new tests PASS;31/31 including the unchanged original SurfaceMotion and SurfaceContinuity suites PASS.** Syntax passes. Raw diagnostics are in `composite-joint-surface-motion-probe.json`; test output is `composite-joint-surface-motion-tests.txt`.

Independent finite differences cover all28 configuration/rate columns for two nonidentical affine axes, including moving levers and frame connection derivatives. Maximum slip-rate Jacobian error in the raw probe is2.2231e-10. An independent finite difference of the full physical triad recovers bending omega with zero feed. Exact virtual-work checks include prescribed feed and moving-wall power.

Common instantaneous rigid motion gives zero relative velocity at the same point with unequal levers. Common finite rotation with changing tangent and separate reference gauges gives residual increments below7e-17 mm. The finite90° false-BE witness changes from−.1426990817 to0. Both own+2π spins retain−1.5707963268 mm. Opposite feed/spin and one-tool stationary-wall cases pass.

A family with common bending rotation/translation plus independent axial motion, feed, spin and nonunit material metrics checks the virtual-power limit. For dt=.01,.001,.0001, finite increments/dt agree with the instantaneous rate to9.62e-11 mm/s and traction power agrees within6.74e-11. This admissible parallel family has an exact linear relative increment; the last-digit error is dominated by label arithmetic, not evidence of a measured convergence order for general finite transport.

No midpoint reconstruction is introduced. A true physical hinge crossing returns the current label and previous/current fractions requiring transport. The minimal next state/history work is to retain **own accepted material surface orientation/point identity and separate unwrapped spin paths by material label**, alongside the existing translational history. A query crossing old edge boundaries must retrieve both one-sided frames/angles and the ordered hinge connection lifts (time-PT plus signed reference-twist/spin connection), with explicit crossing ownership. The finite contact-frame rule then needs piecewise transport through those physical events. Existing per-edge reference/angle and material-label records can seed that history; unknown angular velocity need not be invented. Finite contact behavior through those hinge events is not implemented or certified by this patch.

Frozen bundle: `/tmp/oet-composite-joint-surface-motion-final/manifest.json`. It includes source, tests, raw probe and dependencies. No package script or production timestep was edited.
