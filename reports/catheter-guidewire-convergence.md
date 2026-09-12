# Catheter–guidewire contact convergence

The final closure previously required every active node of both tools to move
less than 0.001 mm per iteration whenever a catheter lumen was enabled. A distant
free part of the wire could therefore demand 64 complete material/contact solves
even when the short catheter's contact was already satisfied.

## Implementation

- `kirchhoffCoupledConvergence.js` measures relative displacement at the actual
  contact interpolation stencil, including cubic neighbours and the sliding
  mouth. Remote relaxation and common translation do not count as contact error.
  Normal motion remains constrained during sliding; sticking also checks the
  tangential coordinates. Open, unloaded contacts do not impose a motion gate.
- The outer loop retains the full direct Kirchhoff solve of both rods, their
  complete length constraints, orientation/feed boundaries, fold limits and
  vessel contact. The global direct contact response in the primary iterations
  is unchanged. No response is truncated at the catheter tip.
- The contact block can do additional inexpensive sweeps without refactorizing
  the distant free wire. It returns to the outer solve immediately if material
  length or contact-coordinate motion still needs a global update. It also
  returns on stagnation. The inner budget is bounded by the active problem size;
  tolerances and spatial sampling are unchanged.
- Smooth side quadrature uses pooled slots 0–3. The distal linear side contact
  now owns slot 4, so it cannot overwrite the first smooth quadrature record and
  share its multiplier accidentally. Fillet/rim/sliding-mouth slots remain 6/5/7.
- Diagnostics expose outer iterations, contact sweeps, contact motion and the
  compliant contact residual separately. No simulation time is discarded.

The contact stopping tolerance remains 0.001 mm and the whole-rod relative length
tolerance remains 0.002 (0.2%). EI/GJ, friction, radii, collision sampling, timestep
1/120 s and the global iteration cap remain unchanged.

## Reproducible browser measurement

Debug → **Cewnik 1/2/5 cm** runs 5605 physical steps (46.7083 seconds):
999.9 mm guidewire feed, 3 seconds of settling, 5 seconds without catheter,
then catheter feeds to exactly 10, 20 and 50 mm with 5 seconds held at each depth.
The report includes per-phase FPS, step time, closure time, convergence, length,
wall/lumen diagnostics and backlog. It stops by physical step count, so a slow
run cannot skip the difficult part of the trajectory.

Replay reset restores current wire frames without changing manufactured strain,
and resets adaptation sweep parity. End-of-phase pose fingerprints allow the
uncoupled setup to be checked between versions. The benchmark uses the actual
application, anatomy and scheduler, not a separate simplified solver.

## Measured result (2026-09-05)

Same host, one visible Codex in-app browser tab, fluoroscopy view, no concurrent
Node tests. Glidewire shaft/tip stiffness 10/4.55; Berenstein 25/5; both relaxation
rates 1. The before/after runs used identical benchmark code. Pose fingerprints
of **both tools match after every uncoupled phase** (feed, settling, wire-only).
Selected per-phase browser data is in
[catheter-guidewire-convergence-browser.json](catheter-guidewire-convergence-browser.json).

| Held catheter depth | FPS before → after | Mean physics step before → after | Mean outer passes before → after |
| --- | --- | --- | --- |
| 0 mm | 60.0 → 60.0 | 2.75 → 2.77 ms | 2.37 → 2.37 |
| 10 mm | 43.1 → 60.0 | 21.68 → 5.04 ms | 63.92 → 6.44 |
| 20 mm | 41.6 → 60.0 | 22.50 → 5.96 ms | 63.87 → 7.87 |
| 50 mm | 36.4 → 60.0 | 25.46 → 6.28 ms | 63.95 → 6.23 |

Mean physics CPU cost falls by 73.5–76.8% in the held coupled phases. During the
three feed intervals the new mean step times are 4.96, 5.03 and 6.62 ms, also at
approximately 60 FPS. Overall 1% low is 59.89 FPS. All 1918 coupled physical steps
converge; no physical steps are dropped. Both runs execute 5605 total steps.

The complete replay takes 81.96 → 50.66 seconds of wall time for 46.708 seconds
of physics. Final backlog decreases from 35.26 to 3.96 seconds. This remaining
backlog predates catheter feed: the optimized run has 4.33 seconds pending just
before feed, and 3.96 seconds at the end. The 20/50 mm held phases still add about
0.05/0.15 seconds locally; 60 FPS should not be read as a guarantee of an exact
120 Hz physics rate in every phase.

Accuracy remains bounded by the configured tolerances, not by the earlier
incidental over-solving. During catheter feed/hold, maximum relative segment
error is below 0.2% and reported vessel-wall penetration is at most 0.03066 mm.
The held lumen violation is below 0.000009 mm; the fractional entering-cell
contact reaches 0.01979 mm during feed. Its compliant solver residual is below
0.001 mm. The old held runs had smaller length and wall errors in some phases,
but repeatedly exhausted the outer iteration cap; the new stopping criterion
does not claim bit-identical coupled trajectories.

Before the catheter is inserted, both versions have the same existing full-wire
transient problems: reported wall penetration peaks at 4.37438 mm, and relative
segment error reaches 3.80% during feed/settling. Their matching pose fingerprints
and diagnostics separate these issues from the catheter optimization.

## Mechanical regression coverage

- An unloaded 1 cm catheter does not add outer iterations for a distal wall
  correction; the entire wire response agrees with the uncoupled solve to
  0.00001 mm.
- A contact at 0–20 mm transfers a reaction to free wire at 80 mm in the same
  physical step. Unequal rod masses retain their mass-weighted centre under the
  internal reaction, both with and without contact batching.
- Cubic support, common motion, sticking, admissible axial slip, unloaded contact
  and independent portal/quadrature multipliers have explicit regression tests.
- Existing direct-beam, reciprocal contact, sliding, compliant contact,
  manufactured-curvature and full catheter-feed tests remain in the test suite.

Final validation:

- `npm run test:physics:optimization`: passed, including the new convergence and
  fixed-step benchmark tests.
- Direct mechanics, world constraints/scheduler, length closure, containment,
  loading support, orientation boundary and runtime regressions: passed. The
  orientation test's plain mock needed its `rodModel: 'kirchhoff'` identity
  restored after the preceding legacy cleanup; runtime solver selection remains
  immutable.
- Full 1200-step catheter-over-held-wire regression: Berenstein, Pigtail and SIM1
  passed. The final Berenstein run took 48.97 seconds without another heavy test
  running. Pigtail/SIM1 were run concurrently with other acceptance checks, so
  their timings are not used for performance comparison.
- Real-aorta late-feed regression (Pigtail rotation 91°, wire 211 → 362 mm,
  catheter held at 215 mm, then 360 held steps): passed. Final aperture angle
  2.292°, wire bend at the aperture 2.534°, segment error 0.0008 mm; reported node,
  segment and rendered containment escape zero.
- `npm run build`, `npm run docs:check`, `git diff --check`: passed. Vite retains
  its existing warning about the large simulator bundle.
- `npm test` still stops at the pre-existing guidewire wall-projection velocity
  failure: 1.230252692423265 mm/s against 1.0. The separately rerun transport
  release regression also retains its exact previous failure
  (1.399117089146877 → 1.353821641870113 rad). Their assertions were not relaxed.
  The previously recorded solo-catheter aorta failure is documented in
  [direct-only-solver.md](direct-only-solver.md); it was not rerun in this change.

## Scope

This change targets the extra cost of catheter coupling. The pre-existing full
guidewire insertion case has its own transient length/contact errors and can
already accumulate physics backlog before the catheter is inserted. Per-phase
timing and backlog must be considered alongside FPS; this benchmark is not a
claim that every possible manoeuvre meets the application's strict long-run
acceptance thresholds.
