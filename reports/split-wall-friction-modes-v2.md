# Wall static/kinetic mode handoff

Two production files only; World and SplitMotion integration belongs to root.

The controller implements the agreed history-dependent discrete law. Incoming motion must be captured before force/damping prediction. Static candidates use the exact static coefficient, but can commit only with a zero-motion certificate (exact zero or strict cone interior with displacement within the explicit solver error bound). A converged static-cone candidate with resolved sliding requests a whole-physical-phase restore and re-solve with the kinetic coefficient. Small nonzero boundary slip is ambiguous. Non-convergence is never evidence of breakaway. Mode demotions are monotone within the dt. Capture enables stick eligibility on the next accepted step; there is no within-step event time claim.

Native wall rows retain the existing shared surface force/radius moment Jacobian. Applied reaction history records the actual scaled increment and rolls back with mechanics. Missing controller preserves the unequal-coefficient guard. Missing incoming material/witness identity and loaded foot/feature changes are unsupported. Root supplies wallFrictionWitness branchId, faceIndex and planeOffset for endpoint/sweep rows. Equal coefficient rows retain their numerical construction. The root builder version increment is preserved.

Validation: 19 new tests PASS, including native full two-rod response with free angular DOFs, plus 23 previous local regressions PASS and 11 unmodified independent World/wall oracles PASS. New tests and raw scalar witnesses are in results/. The free-angular test reports impulse/moment agreement within 1e-9 and kinetic KKT residual at most 2.712e-10 mm. Physical-phase World restart hooks and actual-anatomy acceptance are not tested by this narrow patch; root owns their integration.

API (kirchhoffWallFrictionMode.js):
- captureKirchhoffWallFrictionIncoming(joint, world), before prediction; null for all equal coefficients.
- initializeKirchhoffWallFrictionModes(joint, incoming, {displacementToleranceMm, coneTolerance=1e-9}), after beginSplit and before phase snapshot.
- evaluateKirchhoffWallFrictionCandidate(joint, freshWallBatch, {converged}), after whole physical closure; returns accepted/restart/ambiguous/unconverged/unsupported.
- prepareKirchhoffWallFrictionRetry(joint, decision), after phase restore; reinstalls all owned overrides and attempt number.
- certifyKirchhoffWallFrictionModes(joint, freshWallBatch, {converged}), after bias; final failures become guards, no restart.
- commitKirchhoffWallFrictionHistory(joint, certificate), within accepted split history commit, after diagnostics.certified=true and phase=complete; once only.

Persistent state is joint._wallFrictionHistory; trial state is joint._splitMotion.wallFrictionModes. History stores identity/mode/stop evidence and input motion signatures, never multipliers or friction budgets. Root must publish the actual final body velocities before commit so the saved signature matches the next incoming step.

V2: resolved static slip with valid final KKT proves breakaway even if numerical shift leaves the force slightly inside the cone. A new regression with a 3e-8 relative force deficit failed before the fix and passes after it. No tolerance or contact-law gate changed.
