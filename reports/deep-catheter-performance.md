# Deep catheter insertion: measured limitation

This report records the **2026-09-05 baseline before the shared contact block**.
For its implementation and subsequent measurement, see
[the contact-block report](catheter-contact-block.md).


The contact convergence change fixes the short-catheter performance cliff. It
does **not** make long catheter–guidewire overlaps run in real time. This report
measures the current solver; no physics or convergence tolerance was changed for
this investigation.

## Reproduction

Debug → **Cewnik 10–60 cm**. The new replay prepares a 999.9 mm guidewire exactly
as the short replay does, feeds a Berenstein catheter to 100/200/400/600 mm, and
holds each depth for five physical seconds. At 600 mm it commands two seconds of
rotation in each direction (324° and −324° at the existing control speed), then
holds for another five seconds. It executes 8553 fixed steps, or 71.275 seconds of
physics, regardless of wall time. The JSON report has a `deepCatheterPhases` field.

Measured on 2026-09-05 in one visible Codex in-app browser tab, fluoroscopy,
with no concurrent Node tests. Wire shaft/tip settings 10/4.55, catheter 25/5,
both relaxation rates 1. One run; other catheter shapes and withdrawal were not
measured. Both tools' pose fingerprints after all three preparation phases match
the preceding short-catheter benchmark. Selected rounded DOM report values are
in [deep-catheter-performance-browser.json](deep-catheter-performance-browser.json).

## Results

The feed rows describe the whole interval leading to the indicated depth. Held
rows describe five seconds of physics at that depth.

| Catheter depth | FPS during feed | FPS held | Mean physics step held | Mean outer passes held | Steps not converged while held |
| --- | ---: | ---: | ---: | ---: | ---: |
| 100 mm | 59.7 | 58.9 | 15.42 ms | 20.16 | 0 / 600 |
| 200 mm | 24.3 | 17.5 | 54.12 ms | 62.38 | 522 / 600 |
| 400 mm | 13.9 | 11.0 | 87.06 ms | 46.03 | 30 / 600 |
| 600 mm | 8.4 | 6.5 | 150.01 ms | 59.75 | 351 / 600 |

Rotation at 600 mm gives 7.62 / 7.45 FPS and 126.70 / 129.68 ms per physics step.
The final held phase gives 6.88 FPS and 140.53 ms per step. Whole-run 1% low is
5.05 FPS. No JavaScript errors were reported.

The complete replay takes **478.715 wall seconds** for **71.275 physical seconds**,
with **407.439 seconds of final backlog** and zero dropped steps. Even the
apparently smooth 100 mm held phase takes 10.18 wall seconds for five physical
seconds: its backlog grows from 5.45 to 10.62 seconds. Rendering FPS alone would
miss that slowdown.

## Where the time goes

At 600 mm, final telemetry reports 479 lumen contact records. The held phase
averages 59.75 full outer iterations and 106.70 contact sweeps per step; the outer
limit is 64. Coupled closure consumes 106.21 of the 150.01 ms mean step (70.8%).
The final individual step spends another 40.6 ms in primary constraints, so
eliminating closure alone would still leave substantial work.

The code explains the scaling: every outer pass solves both complete rods,
their wall/boundary constraints, and resamples the overlap. In the first two
contact sweeps, every loaded material cell can additionally request a full-rod
direct mobility response for each tool. Increasing overlap therefore increases
both the contact work per sweep and the number of full-length responses.
Separating distant free-wire motion from local contact convergence removes the
short-overlap pathology, but does not remove this long-overlap coupling cost.

During coupled phases, reported relative segment length error stays below 0.2%,
but contact residuals can exceed the 0.001 mm stopping tolerance. At 600 mm the
held solver residual peaks at 0.00991 mm; during feed it reaches 0.02076 mm. The
held raw lumen violation peaks at 0.00213 mm and reported wall penetration at
0.12958 mm. The all-run 4.37438 mm wall maximum occurs during the unchanged
guidewire preparation, before the catheter is introduced. A bounded trajectory
does not establish either contact convergence or real-time performance.

## Next solver change indicated by this result

Long-overlap optimization needs to address the coupled system itself: a block
solve for active lumen contact and both rods, so a group of contact reactions
propagates through each complete rod together. A candidate is a contact-space
Schur solve or a joint banded/preconditioned rod–contact solve, with the active
set and Coulomb friction updated between block iterations. This is a proposed
next implementation, not a measured speedup. The acceptance replay above should
check feed, held and rotated phases separately, together with backlog, contact
residuals, wall/length errors and reciprocal force transmission.

Lowering iteration limits or dropping scheduled physics steps would not resolve
the measured convergence problem. Existing EI/GJ, timestep, collision sampling
and contact tolerances were preserved.

## Validation of the benchmark addition

- Short and deep fixed-step replay tests verify exact feed distances, held step
  counts, both rotation directions and floating-point phase boundaries.
- Syntax checks, browser scenario tests, production build and documentation
  check pass. The live run completes every phase and reports the actual depth.
- No physics implementation changed in this addition. The previously documented
  guidewire regression failures remain outside this measurement.
