# Direct Kirchhoff solver only

The guidewire and catheter now always use the direct discrete Kirchhoff solver.
This includes Glidewire, Steel J-wire, Berenstein, Pigtail and SIM1. No URL
parameter is required; the old `physics` and `wireSolver` selectors no longer
select an alternative implementation. Runtime diagnostics identify the mode as
`kirchhoff-direct` and both bodies as `constitutiveSolver: direct`.

## Removed code

- `ElasticRod` dynamics and `GuidewireSolver` projection, route replay,
  straightening and fold-repair implementation.
- Local Kirchhoff bend/twist sweeps, their block variants, and unused numerical
  helpers. The direct solver still uses the shared adaptation and orientation
  boundary kernels, and the global length preconditioner.
- Catheter-owned free-rod dynamics, collision handling, guidewire attraction
  and reaction approximation.
- World-space rest-shape targets, chord springs, direction targets, closure
  tethers, one-way containment projections and material-locked distal portals.
- Associated buffers, options, diagnostics, smoothing UI, legacy demos and the
  standalone legacy guidewire regression script.

`RodState` contains material-node storage and geometry diagnostics.
`GuidewireTransport` prescribes feed at the introducer and collects read-only
contact diagnostics. Free material dynamics and reciprocal tool contact belong
to `EndovascularPhysicsWorld` and `kirchhoffDirectSolver`.

The seven affected rod/transport/world modules decreased from 21,085 to 11,577
lines relative to the working-tree snapshot immediately before this cleanup,
including the two replacement modules: 9,508 fewer lines.

Catheter reference EI remains 50,000, matching the previous direct application
profile. The application uses relaxation rate 1 for both tools. Contact geometry,
the nonlinear convergence tolerance and the coupled iteration budget were not
retuned in this cleanup.

## Validation

- Production build and generated documentation check pass.
- Direct rod mechanics, analytical coupling (12 cases), material profiles,
  quaternion/adaptation kernels, dense-reference optimization comparisons,
  shared-world wall/sheath/friction/sweep tests and fixed-step scheduling pass.
- Catheter-over-wire regression passes for Berenstein, Pigtail and SIM1 at the
  full 1,200-step feed duration. A combined test process exceeded its 300-second
  harness timeout; each case subsequently passed independently.
- The real-aorta coupled scenarios reach their assertions for both guidewire
  types. The late-wire Pigtail scenario also passes using the spatial aperture
  residual instead of the retired material-portal diagnostics.
- A 35-second browser smoke run from `/` completed without JavaScript errors;
  both bodies used `direct`, remained finite, and finished with relative segment
  length errors below 0.003%. It averaged 41.6 FPS during concurrent engineering
  checks; this is a functional smoke result, not performance acceptance.

For Berenstein, Pigtail and SIM1 on a held 660 mm guidewire, 240 feed steps plus
60 held steps at 120 Hz produced **exactly identical complete position and
material-frame quaternion arrays** before and after cleanup, using the previous
direct backend as the reference. The serialized reference/result SHA-256 is
`f4d6c65ee92ecaf60afc6f96498665e9ee4f14250325c9b5475157499dcb4ce0`.

Tests of retired algorithms were removed or migrated: stiffness and preform
checks use Kirchhoff rest strain; transport checks exercise the inlet boundary;
coupling checks use reciprocal contact and a sliding geometric aperture.

## Remaining acceptance failures

The complete regression suite is **not green**. These assertions also fail with
the same numerical values when explicitly selecting the direct solver in the
saved source from before cleanup. Their tolerances remain unchanged:

| Test | Direct result before and after cleanup | Required threshold |
| --- | --- | --- |
| `guidewireTransportVelocity.test.js` | Reconstructed wall-response speed 1.2302526924 mm/s | Below 1 mm/s |
| `guidewireContinuousTransport.test.js` | Bend at release 1.3991170891 → 1.3538216419 rad | Change below 0.03 rad |
| `aortaXpbdRegression.test.js`, solo Berenstein | Tip route distance 41.7347755432 mm | At least 45 mm |

`npm test` stops at the first failure above. The continuous-transport check and
the remaining suites were also run independently. The solo-aorta suite stops
at Berenstein, so later solo Pigtail acceptance cases are not certified here.

The collision benchmark now runs the shared direct world and specifies
intrinsic material turns instead of positional shape targets. Its old acceptance
budget is not met: the 2,400-step synthetic workload recorded 8.12 ms average,
12.93 ms p95, and a peak relative segment error of 6.95%, with finite rods and
zero reported wall penetration. It needs further solver/fixture work; removing
the legacy code does not resolve the catheter-related FPS drop.

Useful checks:

```sh
npm run build
npm run docs:check
npm run test:guidewire:mechanics
npm run test:catheter:mechanics
npm run test:physics:optimization
OET_ONLY_LATE_GUIDEWIRE_REGRESSION=1 OET_ONLY_COUPLED_AORTA=1 node tests/aortaXpbdRegression.test.js
npm test
```
