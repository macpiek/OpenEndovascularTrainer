# Same-state inexact Newton tightening

**Confirmed: tightening the frozen full joint solve from 2e-4 mm to 2e-5 mm at the existing stagnation point makes the entire World timestep pass its original gates.** A second setting, 5e-6 mm, yields the identical accepted state and diagnostics. No normal/tangent force clipping, frame/history rotation, coefficient change, final-tolerance change, or merit change was used.

This extends `cone-storage-audit.md`. The actual initial defect is a lumen anisotropic-cone violation created by fresh geometry. Wall/tool Float32 storage is independently defective but is absent from this replay. The follow-up question was whether a more accurate coupled Newton direction closes the fresh cone without changing the physical model.

## Controlled input

The isolated preview reproduces the original path through world step49, pass4. Only at **step49/pass5**, immediately before the full two-channel solve, the probe changes `options.tolerance`; it retains that value for later nonlinear passes in the same dt. Earlier timesteps, all five preceding nonlinear passes, the force kick, and all final World gates are unchanged. Each successful variant stops immediately after this critical dt.

Every variant has the identical captured input SHA-256:

`0829a4fc8611ebeacc6abc03ace1f4903a177d48b1caeb018f24398c23b9f155`

The snapshot includes all typed body arrays, physical motion arrays, both physical poses, contact geometry and multipliers, and every additional row's RHS/alpha/bounds/gradients. All recorded earlier trials are also exactly equal across the five variants. This is a common mechanical input, not a comparison of trajectories already diverged by earlier tolerance changes.

## Results

| Linear tolerance mm | Newton iterations | Independent full J*dq residual mm | Critical dt result |
|---:|---:|---:|---|
| 2e-4, baseline | 0 | 1.358486513617016e-4 | Rejected after 8 trial scales; final cone 8.013592633915323e-8 |
| 2e-5 | 1 | 3.102116925779441e-6 | Accepted at existing half-scale backtrack; final cone 0 |
| 5e-6 | 1 | 3.102116925779441e-6 | Identical accepted state/diagnostics; final cone 0 |
| 2e-6 | 17 | 3.102116925779441e-6 | Linear line-search failure; direction not applied |
| 2e-8 | 17 | 3.102116925779441e-6 | Identical linear failure; direction not applied |

“0 iterations” refers to retained Coulomb Newton. Native material condensation still computes its response; it does not imply an identically zero full direction.

For both successful settings:

- Full-scale trial still fails the original cone gate: fresh cone 1.194707888174662e-9, merit 0.021141635495946186.
- Existing half-scale trial has fresh cone 0, merit 0.021006992884139354, and `settled=true`.
- Final World `certified=true`, `historyCommits=1`, `physicalPasses=6`, no physical failure.
- Final physical KKT residual is 6.794626814745058e-5 mm. This is measured after the half-scale application; it need not equal the frozen full-direction residual. The unchanged final physical gate passes.
- Executed steps advance from49 to50. State fingerprints are `3483af8a` / `bc7114c5` at wire12.1 mm, catheter7.36666666666667 mm.
- The complete successful diagnostics and snapshots are identical for 2e-5 and 5e-6.

The baseline solve accepts its initial retained iterate because its full reconstructed residual is below 2e-4. That direction is insufficient to make the later fresh cone settle. One additional Newton iteration, requested by the tighter tolerance, supplies a coupled direction whose actual physical application plus the existing backtracking closes the cone. This proves a useful local refinement, without introducing a global `cone/1e-9` merit.

## Why the tighter settings cannot pass

An extra owned system capture verifies the exact residual floor independently of solver status:

| Native original row | Sorted row | Full physical row | α | Mobile gradient entries | RHS mm |
|---:|---:|---:|---:|---:|---:|
| 3 | 108 | 216 | 0 | 0 | +3.102116925779441e-6 |
| 5 | 110 | 220 | 0 | 0 | −2.714359792710397e-6 |

Both are native material equations retained by condensation. With zero mobile J and α=0, no multiplier increment can change those RHS values. Therefore a full-equation certificate below 3.102116925779441e-6 is impossible for this frozen input. The full J*dq check correctly rejects the 2e-6 / 2e-8 solves. Their 17 iterations, 19 factorizations, and 262 backtracks do not supply a more accurate admissible physical direction. The rows were neither removed nor assigned artificial compliance.

`cone-tight-floor-proof-zero-rows.json` contains the exact native indices, residuals, gradient counts, full-system diagnostics, and common seed hash. JSON serializes its infinite bilateral bounds as null; these are the unbounded material rows, not zero-width constraints.

## Bounded runtime implication

The evidence supports a **same-base full joint re-solve at a tighter linear tolerance when fresh acceptance stagnates**, with unchanged final gates and the same pending physical dt. In this fixture a tenfold tightening, 2e-4→2e-5, suffices. The original rejected candidate must be rolled back before applying the new coupled direction; neither the force kick nor history commit is repeated. Retain the existing full J*dq verification, normal/material equations, cone checks, and common physical/bias apply scale.

This is not evidence for globally tightening every timestep or accepting a failed frozen solve. Refinement should be bounded and must respect demonstrable exact-zero-row residual floors. No general adaptive policy or source change was implemented in this read-only task; root owns that integration and its wider validation.

## Evidence and reproduction

- `cone-tightening-comparison.json`: all five outcomes, identical-input/prefix checks, each critical trial and full linear diagnostic.
- `cone-tight-{baseline,2e-5,5e-6,2e-6,2e-8}-seed.json`: complete input snapshots; byte-identical content.
- Corresponding `-replay.json`: complete applied World runs and final diagnostics.
- `cone-tight-floor-proof-zero-rows.json`: independent structural floor proof.
- `cone-tightening-source.json`: isolated runtime/source and probe hashes.

The shared preview and root sources remain unchanged. The probe uses optional isolated debug hooks around solve/apply/commit and the existing `debugJointTrial` hook for fresh measurements. Only its critical-solve callback changes an option, under an explicit test parameter:

```sh
OET_CONE_LINEAR_TOLERANCE=0.00002 OET_CONE_OUTPUT_LABEL=cone-tight-2e-5 node reports/probe-cone-storage.mjs
```

The frozen path is read from `cone-storage-source.json`. Adding `OET_CONE_CAPTURE_SYSTEM=1` records zero-row evidence without changing the solve. Original final containment, angular, length, and cone gates remain those of the copied World source.
