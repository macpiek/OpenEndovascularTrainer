# Contact data sharing

The baseline already includes the residual-only optimization from point 1.

## Changes

- Reuse the current interpolation centers, validated material node indices and weights between coupled contact geometry and surface friction.
- Normalize/read each segment's current and previous orientation once per synchronous contact evaluation. Weak body caches use a monotonically increasing evaluation token outside snapshot state. Every new build/measurement creates a new token, including after a rollback. This deliberately does not cache across changed poses or integrator steps.
- Keep gradient objects in a separate reusable pool when active rows shrink, become zero, or switch through residual-only evaluation. Numeric gradient values, reactions and friction bounds remain fresh.
- Solve and measurement banks remain separate; no force/history cache is introduced and physical equations/tolerances are unchanged.

## CPU measurements

80 synthetic side contacts, 100 warm-up evaluations per variant, nine alternating-order batches of 150 evaluations each, residual diagnostics plus line-search merit, no renderer. Reference modules were captured before point 2. The first case uses distinct segments; the second has four contacts per segment.

| Evaluation | Before | After | Reduction |
|---|---:|---:|---:|
| Distinct segments | 0.594 ms | 0.514 ms | 13.6% |
| Shared segments | 0.575 ms | 0.403 ms | 29.8% |

These are medians for the isolated evaluation, not FPS improvements. Round timings are retained in the JSON, including scheduling outliers.

Paired runtime comparison used current user defaults, the transformed aortic anatomy, 120 Hz and the 5 mm catheter grid. Through 796 complete steps, positions/orientations were exactly identical, as were convergence decisions and line-search counts. Both variants failed at zero-based schedule step 796 with `Invalid friction data`, approximately 85.37 mm catheter feed after 220 mm wire insertion. This pre-existing failure still prevents full-depth validation. Whole-step timing had substantial scheduling outliers and is not used as an FPS claim.

## Validation

36 focused tests passed, including shared quaternion read counts, direct uncached row/kinematic parity, topology changes, complete trial rollback and gradient identity retention. The coupled suite passed 768 tests with the same seven known failures (source defaults audit, two Coulomb format tests, two external mouth transitions, two experimental two-channel refinement cases). Production build passed outside tracked dist.

Raw results: `friction-data-sharing-2026-09-10.json`.
