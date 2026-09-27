# Withdrawal rejection diagnosis — 2026-09-27

Captured rejection: 33e7a3ef-c5fb-4290-bdaf-7a5ae9acdf58, 2026-09-27T10:34:22.010Z.
Replay: tests/fixtures/shared-axis/stent-graft-proximal-rim-withdrawal.json.gz.

## Observed failure

Right-access physics stopped during delivery-system withdrawal. Terminal status was line-search, not a JavaScript exception. The attempt consumed 2311.8 ms CPU (2556.3 ms wall), 54 Newton iterations, 260 factorizations, 138 full and 1050 residual assemblies, and 980 backtracks. Subdivisions 1, 2, 4 and 8 did not recover.

The last blocking contact is the guidewire, radius 0.4445 mm, on segment [290,295] mm at face 3412 of the graft cloth. The contact position is [4.980359548690538,-193.53923562967688,-0.44061744441263295], near the proximal open edge. Fixation capture is null. The delivery core has already withdrawn to native insertion 49.4563 mm.

## Geometric reproduction

Reconstructed the captured base cloth with its indices and BVH. Its reference guidewire segment has no hit within radius 0.4445 mm. After applying the captured exterior indentation patches (449) and other-access patches through graftDisplacementSampler and refitting the BVH:

- Reference segment: distance 0.000029049090633734468 mm, face 3412, crossing false.
- Trial segment: distance zero, face 3412, crossing true.
- Saved recovery interval: [219,226] mm; the blocking [290,295] mm segment is outside it.

This demonstrates pre-existing finite-radius overlap introduced by the deformed cloth. A tiny motion crosses the mathematical surface and is rejected by continuous collision detection.

## Code-level cause

stentGraftCompliance.js validates candidate cloth deformation with axisOnly queries, so it can accept cloth almost touching the wire axis, without room for the physical wire radius. stentGraftContacts.js reopens recovery for centerline crossings, but unchanged-revision proximity without a crossing does not create a fresh interval. These rules are inconsistent with finite-radius contact forces and leave this state unable to settle.

A repair should keep finite-radius clearance during cloth relaxation and support bounded recovery of existing overlaps without disabling normal contact or allowing arbitrary passage through the graft.

No production code was changed and the user scene was not reset during this diagnosis. This is a geometric reproduction of the saved blocking contact; a complete dynamic withdrawal replay has not been validated here.

## Repair validation

The compliance candidate now checks continuous segment-to-triangle distance over each material interval using its physical outer radius (including delivery-cover/core splits). It preserves the better available clearance from the rest and accepted cloth states. Failed relaxation can only fall back to the rest sheet if that sheet passes the same check; otherwise it retains the accepted indentation. Normal fabric forces and CCD were not disabled or weakened.

Exact captured-pose withdrawal regression: before the fix the continuation fails at step 78; after the fix all 125 steps pass, with the delivery insertion reaching zero and the guidewire insertion unchanged. Every accepted step is checked for vessel penetration and centerline/fabric crossings. Maximum 10 Newton iterations and 38 factorizations; 571 total iterations for 125 steps. Initial clearance at the archived blocking rim is restored to at least the full 0.4445 mm guidewire radius.

14 focused tests passed (apposition, indentation, captured withdrawal and delivery withdrawal). Build passed with the existing bundle-size warning. Historical rejection fixtures in stentGraftSessionRejection and two cannulation tests cannot restore because their vessel mesh identity differs from the current anatomy; these are not counted as passing.

Additional safety run: 31 passed (gate cannulation, branch recovery, partial deployment, sliding, contact lifetime, bifurcation stability), 2 historical replay mesh mismatches as noted above. Total unique passing tests: 45. Applied the validated source and two regression-test files to the live workspace.
