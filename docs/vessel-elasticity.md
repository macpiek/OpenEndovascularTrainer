# Local elastic vessel response

The default shared-axis and shared-axis-adaptive application solvers enable a
bounded normal elastic foundation at anatomical vessel witnesses. Wire,
catheter and sheath contribute through the existing outer-material selection
and shared-axis mechanics. Introducer boundary constraints, bend limits and
stent-graft contacts keep their own laws.

For compressive reaction `lambda`, compliance `C` and displacement limit `L`:

```
d = C * lambda / (1 + C * lambda / L)
effectiveGap = referenceGap + d
lambda >= 0, effectiveGap >= 0, lambda * effectiveGap = 0
```

The exact tangent and stored elastic energy enter the mixed Newton solve.
The limit is the smaller of 0.35 mm and 70% of the outer tool radius, keeping
the tool axis inside the reference surface. The initial compliance is 1e-7
in the existing rod model's force/mm units. Parameters are conservative
simulation settings, not clinically calibrated material constants.

This changes the tool equilibrium, not just its appearance. Wall deflection
recovers on unloading; there is no plastic damage or prescribed visual lag.
Mechanical length, force and friction tolerances remain unchanged.

Accepted contact patches are published after both access systems commit the
shared timestep. A sparse GPU spatial index applies a smooth, bounded local
displacement to the STL debug surface, its depth pass and the same triangles
used for contrast. Failed/cancelled solves retain the previous visible wall.
The reference STL/BVH is immutable: its distances, together with the elastic
gap law, define contact. The whole anatomical BVH is not rebuilt every frame.

Scope: this is a local indentation model, not a full deformable vessel wall or
fluid–structure solver. The displayed skin interpolates independent contact
patches; it does not solve tangential tissue tension, longitudinal tethering,
or coupling of multiple contacts through a wall finite-element mesh. It does
not yet translate/straighten an entire artery, redistribute hydraulic volumes
or modify outlet resistances. Experimental projective/modified-Newton and
benchmark variants retain their existing rigid wall model.

Verification:

- `node --test tests/vesselElasticity.test.js tests/vesselDeformation.test.js`
  checks the constitutive derivative/energy, analytic reaction, free-rod
  equilibrium, unload, cap, cancellation and sparse display indexing.
- `node --test tests/vesselElasticityAnatomy.test.js` checks 30 triaxial
  insertion/withdrawal/hold steps on the renal anatomy with unchanged acceptance
  tolerances.
- `/tests/vesselDeformation.browser.html` checks actual GPU silhouettes,
  contrast coverage, unloading and the debug/depth shader paths.

The local replay has steady steps near the 16.7 ms frame budget but slower
contact transitions. These checks do not establish sustained 60 Hz in the app.
