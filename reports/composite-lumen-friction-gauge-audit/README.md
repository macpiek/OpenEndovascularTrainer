# Coulomb friction versus the affine normal-pressure gauge — independent audit 8996

**The existing normal-only endpoint gauge is not, in general, an exact reduction once local Coulomb surface friction is enabled.** It preserves the current normal nodal force and wrench but changes local friction capacities, dissipated work and physical torque when slip varies along the segment. Restricting interior Ft to zero can satisfy the local KKT of the *new pressure choice* while excluding valid states of the originally declared multisample law.

This is a finding about extending the normal-only reduction to friction. The normal-only code explicitly documents its bounded scope. No existing source was edited; the deliverables are an independent probe, raw evidence, this report and a frozen manifest of 20 source/dependency files.

## Immediate policy for the shared step

For an affected side group with nonzero friction coefficients, **reject implicit normal-pressure elimination/transfer** when the normal adapter marks any original sample redundant. In the current implementation this can be checked from `normal.gauge.redundantSamples`/sample redundancy before creating tangent rows. Do not silently set interior Fn/Ft to zero and claim equivalence to the original multisample Coulomb system. A recorded normal-only interior-force transfer is likewise insufficient proof for friction.

The smallest supported policy now is the root's current **one declared pressure/contact sample per physical pair**, retaining its original normal row and both tangent rows. Two explicitly declared samples with no interior elimination are also a discrete model that may be solved with its original gates; there is no need to reject every multiple-sample request solely because its count exceeds one. Any actual singular system must retain the existing honest rejection.

Alternatively, an application may explicitly choose an endpoint-supported pressure discretization **from its initial state**, with interior samples serving only as gap checks. That is a physical modelling choice: its pressure and friction supports differ from the original pressure-at-every-sample model. It must be named/serialized as such, and loaded interior Fn/Ft history from another model must not be silently migrated. Merely adding the name “gauge” does not prove equivalence.

If all friction coefficients on an affected group are exactly zero, the tangential law is identically Ft=0 and the normal-only argument remains applicable. Do not infer that exception from small current slip, zero *current* Fn, a temporarily sticking contact or nearly equal tangential forces; the next trial can change those facts.

For future full multisample friction, retain the original local laws/pressure variables or derive a genuinely coupled reduction valid for force maps, surface kinematics, constitutive laws and accepted history. Normal B equality alone is insufficient. Unresolved rank deficiency remains explicit; this report proposes neither dense fallback nor artificial regularization.

## Why normal equality is insufficient

For the same anisotropic coefficients at all points, let

```
phi(v) = hypot(muU*vU, muV*vV)
```

The minimum local Coulomb power is `-Fn*phi(v)`. Let an interior sample lie at interpolation fraction u between retained samples a,b, with an affine tangent slip/rate `v_s=(1-u)*v_a+u*v_b`. Redistributing its normal force F to `(1-u)F` and `uF` changes dissipation by

```
DeltaPower = -F*((1-u)*phi(v_a)+u*phi(v_b)-phi(v_s)) <= 0.
```

Convexity gives the inequality; it is generally strict. Opposite endpoint slips and a stationary interior point give an immediate strict example. Equality of current normal G/B says nothing about this nonlinear support function.

Even equality of power for one velocity is not a complete reduction proof. A special same-direction slide can share a normalized traction law, but preserving full physical response additionally requires the appropriate tangential force maps and their complete global derivatives to transform consistently, along with finite-slip maps, cone parameters, active/stick branches and history. Under stick, a cone of permissible Ft remains; matching one selected Ft does not match the whole original law. Different per-sample mu can destroy equivalence even for constant slip.

## Exact normal-load witness using the actual normal adapter

Current geometry in mm:

```
wire A=[.25,.25,0], W=[2.25,.25,0]
catheter C=[0,0,0], D=[2,0,0]
innerRadius=.1, lumenRadius=.35, closed distal side
s=[.25,.5,.75], outer t=[.375,.625,.875]
```

All original side gaps are `-2.7755575615628914e-17` (zero to roundoff). The original normal loads `[0,1,0]` and endpoint loads `[.5,0,.5]` give **bit-identical full physical12 nodal force**:

```
[0,-.5,0, 0,-.5,0, 0,.375,0, 0,.625,0]
```

The wire resultant is [0,-1,0] with moment [0,0,-1.25]; the catheter has exactly the opposite force/moment. The probe calls the existing `JointLumenRows.prepareGauge`, which accepts this transfer with three original queries and the exact normal-column proof. Its original normal certificate passes. Thus the counterexample does not rely on an approximate or unsupported normal transfer.

## Physical power and torque counterexample, uniform actual mu

Use the actual coefficients `mu=[.015,.006]`. At the same current geometry, apply the physical wire endpoint rate

```
vA=[0,0,-1], vW=[0,0,1], own spin rate=0,
catheter endpoint/spin rates=0.
```

This is instantaneous rigid rotation about the transverse y axis at omega_y=-1 s^-1 through x=1.25. The actual surface force map, including its lever arms, gives tangent rates:

| s | Tangent rate [U,V], mm/s | Old Fn | New Fn | New local Coulomb Ft [U,V], N |
|---|---|---:|---:|---|
| .25 | [0,-.5] | 0 | .5 | [0,+.003] |
| .5 | [0,0] | 1 | 0 | [0,0] |
| .75 | [0,+.5] | 0 | .5 | [0,-.003] |

Originally Ft=0 at all three points is valid: the only pressure is at the stationary central contact. Its friction force, torque and power are all zero. The endpoint-supported law requires the nonzero sliding tractions above.

The new net tangent force remains zero, but the actual `SurfacePullback`/load map gives:

- Wire moment about y: **+0.003 Nmm**.
- Catheter moment about y: **−0.003 Nmm**.
- Physical power `physicalLoad · endpointAndSpinRates`: **−0.003 Nmm/s**, versus zero before.
- Total internal force and wrench across both tools remain zero.

Keeping the old Ft=0 after transferring Fn fails the original local Coulomb KKT at both moving loaded endpoints. Recomputing endpoint Ft to satisfy KKT produces the changed torque/power. There is no operation here that preserves both the old physical response and the new endpoint laws.

The rate witness uses instantaneous physical power; Coulomb direction is unchanged by a positive conversion from rate to displacement. It does not substitute a rate map for the finite displacement Jacobian. The next witness separately evaluates the actual finite displacement law.

## Finite-step surface witness using the new JointLumenSurface

The current geometry and normal loads above remain unchanged. The old wire is a length-preserving .02 rad rotation about y, shifted inward to y=.24:

```
old A=[1.25-cos(.02), .24, +sin(.02)]
old W=[1.25+cos(.02), .24, -sin(.02)]
```

The catheter is stationary, maps have no feed and own spins are zero, with the correct own previous reference frames and the declared same-edge material history. Both wire lengths are exactly 2 in the evaluation. The original old gaps are all positive:

```
[.009791784762931632, .009999999999999981, .009791784762931632]
```

For the same three selected original samples, the frozen new `JointLumenSurface` evaluates finite G, physical B and DB successfully, with zero helper queries. Current surface witness separation is exactly zero at all three samples. The finite material increments are:

```
[-.00014266121721280713, -.010205955759277985]
[0, 0]
[+.00014266121721258509, +.010205955759277985]
```

The original central-only pressure again permits Ft=0 and zero discrete friction work. Endpoint pressure with the same mu requires approximately opposite tractions:

```
[+.000261931963668433, +.002998169885684500]
[0, 0]
[-.000261931963668026, -.002998169885684506]
```

Each distribution passes its own original local KKT and the new nonlinear friction equations; the normal gap/Fn gates are unchanged. Yet endpoint pressure gives **work −6.12731134897188e-5 Nmm** and wire moment **+0.0029981698856845094 Nmm about y**, with opposite catheter moment. Central pressure gives zero.

These are two admissible current constitutive states on the same finite motion, not a claim that a positive old pressure existed at the open old geometry. The static actual `prepareGauge` check above separately proves that the normal-only transfer is admitted at the current geometry. The finite witness shows that setting interior pressure/Ft to zero from initialization also selects a different physical response, even when no loaded history is transferred.

The force calculation uses separate physical B through `SurfacePullback`, not the finite-slip Jacobian as a force map. No complete timestep or equilibrium solve is claimed by this constitutive counterexample.

## Coefficient variation, separately labelled

A supplementary per-sample-material thought experiment uses constant axial slip [1,0], endpoint muU=.01 and central muU=.04. Central pressure produces axial friction −.04 N and work −.04; endpoint pressure produces −.01 N and work −.01. Corresponding wire moments about z are .014 versus .0035 Nmm. All normal loads remain identical as above.

This illustrates the additional condition required if coefficients vary with material/sample; it does not assert that the current manager already exposes per-sample mu. The uniform-actual-mu counterexamples already invalidate the proposed general reduction.

## Source evidence and reproduction

`reports/probe-lumen-friction-gauge.mjs` is the only new executable. It imports the frozen modules, queries the original detector, invokes the real normal gauge and both surface/load modules, and asserts normal equality, original KKT roots, physical force/wrench, zero current surface separation and the changed power/torque. It never edits a source or solves a whole step.

```sh
node reports/probe-lumen-friction-gauge.mjs
```

Default source is `/tmp/oet-lumen-friction-gauge-audit-8996/stage`; an explicit `OET_GAUGE_SOURCE` may point to another complete snapshot. `evidence.json` contains all Fn/Ft, G/B-derived loads, common/relative loads, KKT metrics, gaps, surface increments and wrench values. `source.json` and the final manifest identify the exact 20 frozen dependency files, including the versions read while Singer completed the surface module.

This report makes no claim about variable ownership, friction across remeshing/feature transitions, continuous collision or World behaviour. Its finding is already present on one strict affine side pair with fixed identities, equal current normals and uniform friction coefficients.
