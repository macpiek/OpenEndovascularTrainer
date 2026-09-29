# Stent-graft bifurcation attachment — 2026-09-28

## Cause and change

The three fabric parts were projected independently. Wire-length corrections could shift their end sections apart. The 2 mm proximal overlap also nearly doubled the first limb interval despite an unchanged material coordinate.

Limb-mouth vertices now depend on fixed material coordinates in the trunk outlet, including the septum. The coupled solve pulls cloth/wire gradients back to shared particles; it does not repair the seam by stretching metal after rendering. The same positions supply the released collision surface. Added local longitudinal constraints around the bifurcation, reduced overlap to 0.2 scaled mm, and stopped the separator plane from moving covered rows. Limb wire inset is 0.2 mm (previously 0.15 mm) to avoid overlapping wires at the shared fabric septum.

Bindings and constraint stencils are cached. The common solve starts at outlet exposure, has a finite iteration budget and skips unchanged geometry. Standalone limbs keep their existing solver.

## Validation

- 51 targeted tests passed (attachment, capture, wire kinematics, sewn anchors, release, interaction, limb threading, geometry cache).
- After caching and specializing the shared distance constraints, all 23 affected tests passed again.
- Additional earlier checks covered contrast anatomy/partial release and anatomy wall fit; no full end-to-end user-session replay was recorded.
- Build passed; existing bundle-size warning remains.
- Right/left fixtures exercise 120 release commands, 30 recapture commands and rotation. Every mouth vertex stays within 0.00005 mm of its sewn material binding. First five branch intervals stay within 0.25 mm of their nominal center spacing. Metal relative length error stays below 0.0001.
- A coarse command crossing the junction before limb constraint initialization remains finite.
- Browser preview: `tests/stentGraftJunction.browser.html` (partial, branch release and full deployment).

This remains a reduced cloth/rod approximation, not an exactly inextensible shell. The spacing bound above applies to tested section centers near the seam, not every meridional fabric edge in all configurations.

## CPU cost

`scripts/benchmark-stent-graft.mjs`: component update and delivery geometry only, excluding rod Newton solve and GPU. Single local runs, sensitive to host load.

| Body release phase | Before mean ms | After mean ms |
|---|---:|---:|
| 0–25% | 3.07 | 3.15 |
| 25–50% | 3.63 | 6.84 |
| 50–75% | 5.17 | 9.65 |
| 75–100% | 3.81 | 7.73 |
| Idle | 0.33 | 0.36 |

The stability fix adds work while the bifurcation opens. It is not a claimed physics-Hz speedup. Raw measurements are in the accompanying JSON.
