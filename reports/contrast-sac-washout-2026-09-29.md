# Sac contrast washout — 2026-09-29

## Reproduced defects
The live image retained a dense sac shadow. Offline reproduction on the aneurysm atlas demonstrated slow washout: the old sac operator only mixed neighbouring finite volumes and drained the distal cells. It had no advective transport connecting the gate inflow to patent native outlets. Geometry revisions also discarded the sac texture history and initialized replacement graft concentration buffers to zero, causing brightness ramps.

## Changes
- Factor the native sac tree once per geometry revision to obtain a steady flow distribution from gate inflows to patent outlets.
- Use a conservative, positive implicit upwind sweep, with the existing symmetric dispersion/mixing retained. No mass fade or removal is used; discharged iodine enters the downstream arteries. Disconnected pockets and a sealed sac are retained.
- Reuse the sac mesh/texture across geometry revisions; advance previous-frame samples only when simulation time advances. Initialize replacement graft concentration buffers from existing concentrations.

## Quantitative regression
100 mg bolus in the same deployed main-body aneurysm-atlas fixture (open contralateral gate); fixed 1/30 s transport steps:

| Simulation time | Previous sac iodine, mg | New sac iodine, mg |
| --- | ---: | ---: |
| 1 s | 99.6494 | 38.1548 |
| 5 s | 96.3372 | 5.9518 |
| 10 s | 90.1003 | 2.9700 |
| 30 s | 71.5227 | 1.2411 |
| 60 s | 54.1943 | 0.3937 |

The sum of circulating, sac and outlet iodine remains 100 mg (error below 1e-7 mg). This is a reduced tree transport model, not a three-dimensional CFD or clinical washout calibration. The fixture reproduces the numerical mechanism; it is not an export of the exact live tool pose.

## Validation
17 contrast/graft tests passed, including atlas partial/full release, branch exclusion, iodine conservation, tiny-cell positivity, sealed pockets, and repeated geometry-refresh interpolation. Production build passed (existing large-bundle warning). Browser preview verifies bolus arrival followed by declining sac opacity in a simplified Y geometry. Main-scene application reloads the current scene.

## Live-session limitation
While checking the app log after application, repeated pre-reload errors were found at 19:13:51–52 UTC: `Simulation post-commit update failed: Crown roots exceed the fixed metal span` in `updateSuprarenalCrown`. `commitSimulationStep` calls this before the contrast update, so this separate mechanical failure can freeze contrast time as well. It is not fixed by this transport/rendering change. After HMR the tools were reset to 0 cm and the app ran at 60 FPS / 60 Hz, with no later errors observed. The exact previous instrument pose has not been replayed.
