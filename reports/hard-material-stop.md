# Native wall static-friction dual certificate

**Result: 48/48 PASS**, including the original unchanged six native World tests and its three consecutive static timesteps. Handoff is only `src/physics/kirchhoffWallFrictionMode.js`, the dedicated `tests/kirchhoffWallFrictionHardMaterialStop.test.js`, and this report/evidence. World, SplitMotion and TwoChannelSystem were not edited. No tolerances, coefficients, published velocities, reaction banks, or force inputs were changed.

Source provenance and hashes: `hard-material-stop-source.json`. Validation ran against a frozen complete native runtime copied from root 901c, with only this helper replaced. The original helper was SHA-256 `9e4cc22f6be6683a770e494caa150221ebf6060b907546fceb045a4275aa5cae`. The final helper is `addc39a79f0ad6d533a85e0ec79692b4756b4197db498776fb01369e3fa6e688`; dedicated test `e31574a3d8f3b54e0582b527933cc61681161fffd715746f15300d1b9731b1ea`.

## Reproduction and mechanical cause

The untouched native World fixture has three mobile masses, two hard adaptation segments, fixed material frames, an affine wall, and μstatic=.6 / μkinetic=.2. Each dt applies total tangent impulse 1.2 and normal impulse 3. The original helper accepted two dt, then rejected the third as ambiguous. Its zero-load distal point had lost the stop certificate, selecting kinetic mode at the next dt. The physical/material/normal closure itself was settled.

A first investigated endpoint relation was exact on free coordinates: Jt(node2)−Jt(node0)=E0+E1. Its combined start/end RHS was zero although individual adaptation RHS were not. That narrower proof retained point2's history, but the unchanged test still failed: at the third dt, both node0 contacts lay on their static cone boundaries, while the only independently certified interior contact was node1. Its single-edge relation had nonzero RHS. Therefore exact homogeneous-rate propagation was insufficient and is **not** the implemented fix.

The actual issue is a nonunique dual solution. A redundant wall witness can lie on its cone boundary even though another admissible distribution is strictly static. Requiring every particular reaction to be interior overconstrains the classification.

## Implemented certificate

The existing caller must supply a fresh converged **whole physical closure**, as required by the helper API. The candidate's original q, v, all normal multipliers, and all material/contact banks remain untouched.

For one native hard chain with fixed frames and a single wall feature:

1. Require every incoming contact mode to be `stick`, every current tangent displacement to satisfy the existing solver tolerance, αadaptation=0 before/after prediction, unchanged rest lengths/material labels/frames, zero known angular motion, free translations, and no enabled positional or orientation controls. Missing native sparse gradients or immutable dt history rejects the certificate. A sliding incoming contact is never promoted.
2. Form F=ΣFt and N=ΣFn. Require N>0 and |F|<μstatic*N using the existing cone margin. Propose alternate tangential multipliers Ft_i'=Fn_i*F/N, keeping every original Fn_i. The final loaded contact receives the arithmetic remainder. Every alternate cone and the explicit static tangent-velocity equalities are checked at the existing tolerances.
3. Compute the changed contact generalized load g=Jtᵀ(Ft'−Ft). Prefix sums along the chain construct a bilateral adaptation-multiplier increment δλE that balances g.
4. **Independently multiply the actual native sparse Jacobian** by δλE, rather than verifying the prefix construction against itself. Require each mobile component of JtᵀδFt+EᵀδλE to vanish within a propagated IEEE arithmetic bound. Also require the mobility-weighted correction change to remain within the existing solver tolerance.
5. Check α*δλE=0 exactly. Thus every original constitutive residual C+αλ is unchanged, including a nonzero C. The primal state and normal multipliers are unchanged, so all previously certified normal equations, bounds, and untouched rows retain their full physical-closure evidence. Only the changed friction blocks need their alternate cone/KKT check, which is performed explicitly.

The arithmetic bound carries magnitudes of the original and alternate loads and all prefix additions. Bounding only the small resulting difference is incorrect: a final prefix residual of 3.8e-19 can come from subtracting loads around 1e-4. This is a force-balance arithmetic check, not a stiction-speed threshold.

The owned certificate records the alternative reactions, material increments, all mobile balance residuals/bounds, and fixed-support torque changes. Native assembly omits fixed rotational columns; their adaptation torque is recovered analytically only for the support-reaction witness. Known angular rate is exactly zero, so these reaction changes do no work. None of the witness multipliers is written back to simulation banks.

The stop classification represents static feasibility at the existing numerical precision, not a claim that every stored floating-point velocity is literally zero. Material/motion signatures bind this certificate to the next incoming state; externally changed tiny incoming motion invalidates it.

## Native numerical evidence

At the third accepted dt:

| Quantity | Value |
|---|---:|
| Total original normal multiplier | 0.024999975574247702 |
| Total original tangent multiplier x | −0.009999993488231619 |
| Total static capacity | 0.014999985344548621 |
| Alternate wall0 tangent multiplier x | −0.004999996860894726 |
| Original wall0 tangent multiplier x, preserved | −0.007499992847442627 |
| Alternate wall1 tangent multiplier x | −0.004999996488365574 |
| Original wall1 tangent multiplier x, preserved | −0.0025000004323320795 |
| Adaptation row3 δλ | +0.0024999960560334947 |
| Maximum mobile Jᵀδλ residual / correction change | 0 / 0 mm |
| Maximum explicit tangent displacement residual | 9.34534449825719e-11 mm |
| Alternate cone violation | 0 |

Fixed-support torque-multiplier increments at segments 0/1 are +0.0012499980280167473 and −0.0012499980280167473 in material axis 1. Their angular impulse equivalents are ±0.14999976336200968 at dt=1/120; their sum is zero. These are alternative support reactions only.

At the second dt, maximum mobile balance residual is 3.7947076036992655e-19, inside the explicitly propagated bound. The actual published velocities remain nonzero and unequal. At the third dt they are [1.1214400075232334e-8, 2.161349321738726e-9, 1.1214413397908629e-8]; they were not projected or reset.

## Validation and bounds

`hard-material-stop-final-tests.txt`: **48 PASS, 0 FAIL** = original 22 helper/wrench tests + original 6 native World tests + 20 dedicated tests. The original World fixture, drive, μ, acceptance checks, and tolerances are byte-identical to root.

Dedicated checks cover an independent straight-rod force/torque balance; exact preservation of original reaction/primal arrays; nonzero material RHS preserved by αδλ=0; owned evidence surviving later native scratch/bank mutation; finite compliance; nonzero commanded control RHS; moving translational/angular supports; incoming angular motion; changed frames/rest/material identity; mixed wall feature/plane; missing immutable history; free unloaded contacts; equal tiny speeds without hard rows; unresolved tangent equality; incoming sliding; deliberately corrupted native gradients; invalidation after changed material data; and externally commanded tiny incoming velocity.

Reproduce after importing into root:

```sh
node --test tests/kirchhoffWallFrictionMode.test.js tests/kirchhoffWallFrictionModeWrench.test.js tests/kirchhoffWallFrictionModeWorld.test.js tests/kirchhoffWallFrictionHardMaterialStop.test.js
```

For isolated validation, the dedicated test supports `OET_HARD_MATERIAL_STOP_SOURCE_ROOT`; it uses that runtime's native fixture/helper/solver. The probe `probe-hard-material-stop.mjs` reads the frozen path in the provenance JSON and writes an owned three-step replay. Baseline evidence is `hard-material-stop-baseline.json`; final evidence is `hard-material-stop-dual-replay.json`.

This bounded certificate is not a general nonlinear static feasibility solver. Compliant material, mobile rotations, active controls, mixed wall features, missing native evidence, unresolved slip, or incoming sliding retain the existing guarded path. No anatomy, interactive runtime, or performance claim is made by this handoff.
