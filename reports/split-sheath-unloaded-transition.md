# Certified unloaded sheath boundary transitions

A strictly clear, reaction-free material node may now cross an end of the existing open analytic sheath without an unsupported-history rejection. Loaded, touching, relabelled or history-lost transitions remain rejected. Only SplitMotion production source changes; World, geometry, numerical thresholds and profiles remain untouched.

## Whole-step proof of the reported node

Before changing production code, the regular root adapter was observed through 33 guidewire commands followed by 14 catheter commands, ending at the fifth actual coupled step. No force or geometry was injected. The observer recorded every target row's base lambda, every actually applied common-scale increment and resulting lambda, current/physical/bias/legacy sheath banks, geometry and material identity. This records load before a collector can erase it by dropping a row.

For sheath0/side1/node0 in the failed step:

- Start axial coordinate: -89.99912318852675 mm, inside the unchanged lower limit -90.00001 mm.
- End axial coordinate: -90.00342814482242 mm, outside the proximal slab.
- Start radial clearance: .058183063771332776 mm; final reported clearance .05816302730677003 mm.
- Minimum radial clearance among all 17 phase observations: .05803232654464611 mm.
- Every observed current, physical, bias and legacy sheath lambda: exactly 0.
- Every actual applied sheath delta-lambda and resulting lambda: exactly 0.
- Material identity unchanged; observed target control lambda also 0.

This is a free sheath witness, not a loaded contact whose reaction merely vanished from the final row set. Other loaded sheath nodes remain in the same mechanical system and retain their reactions.

## Implemented proof and reaction journal

At physical-step start, each captured sheath/material node receives separate physical, bias and legacy reaction-activity arrays. The existing SplitMotion apply hook records `abs(lambda_before)`, `abs(actual_scale * delta_lambda)` and `abs(lambda_after)` for sheath rows before pose application and normal-bank commit. This happens for both physical and bias phases. Current and stored phase banks are also observed. Collecting/preparing rows never resets the journal.

The journal belongs to the owned split trial state and is restored with rejected trial mechanics. An unloading increment cannot erase an earlier load; any nonzero activity remains nonzero for the accepted path of the physical dt. Missing/nonfinite journal data cannot prove a free transition.

A boundary transition is accepted only if:

1. Sheath origin/axis/radius/axial limits, selected support range, node radius and material label still match the captured witness.
2. Both start and current radial gaps are strictly positive (`>0`), without adding a skin or tuning epsilon.
3. Physical, bias and legacy reaction journals and observed banks are identically zero for that material witness throughout the step.

The radial cross-section of the fixed cylinder is convex. The radial norm on the straight start/end chord is bounded by its endpoint maximum, so strictly positive endpoint clearance proves that this chord cannot require a sheath radial impulse. This is a free-path check for the existing discrete model; it is not a terminal impact, general trajectory, end-cap or TOI/CCD treatment.

The check is independent of contact activation distance. Any physical or bias reaction, a touching endpoint, changed material/geometry or missing history preserves an unsupported guard. An earlier provisional free-transition observation is invalidated if later reaction activity appears. Existing physical normal, material, friction and raw geometric certificates are unchanged.

## Tests

New `tests/kirchhoffSplitSheathUnloadedTransition.test.js`: **10/10 PASS**:

- Strictly clear unloaded entry and exit at both proximal and distal ends.
- Zero-load touching transition remains unsupported.
- Positive endpoint gaps do not erase an earlier physically applied reaction.
- Positive endpoint gaps do not erase an earlier bias reaction.
- Missing reaction history and changed material labels remain unsupported.
- An actual half-scale solved trial records the applied delta and restores the journal, identity and body mechanics on rejection, using the production `reusePropertyLayout` and `frozenFrictionBatches` settings.

The new tests plus existing split/sheath/preserve-strain checks give **23/23 PASS**. The unchanged independent World+wall oracles give **11/11 PASS**. Existing strict-accuracy failures from the previous handoff were neither modified nor investigated further.

## Same-World adapter comparison

The root World changed during the initial observation/implementation interval. For a valid final comparison, an isolated reference and candidate tree were built from the **same frozen World** (`e993cec61125fe003e58d3bd081b24b15ec91e158351427319b2216900522f62`), with only SplitMotion differing. Root files were never edited.

Both reference and candidate source sets remained stable during execution. All 17 fifth-step target observations (axial/radial geometry, material identity, control and reaction banks) are exactly equal, and the first four accepted fixture snapshots are exactly equal. The fifth step changes from rejected to accepted with `historyCommits=1`, while its normal-history residual remains 6.938737638677306e-6 mm and geometric violation remains 0. All five coupled steps certify. The run stops there; no sixth step, long replay or anatomy was added.

Reproducer: `reports/probe-sheath-unloaded-boundary.mjs`. It uses the normal adapter and can select a source snapshot via `OET_SHEATH_TRANSITION_SOURCE_ROOT`. The patch bundle's `results/` contains the initial whole-dt audit, the matched-World reference/candidate JSON, their source hashes, regression logs and a compact equality comparison.

Scope remains narrow: no loaded-end release mechanism, remeshing migration, geometry change, CCD, terminal impulse, World transaction change, tolerance adjustment or material-profile adjustment is included.
