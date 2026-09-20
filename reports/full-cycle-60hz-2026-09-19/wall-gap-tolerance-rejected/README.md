# Separate wall activation gap tolerance — rejected

Keep linear stationarity/reaction tolerance unchanged, but activate inactive wall rows only beyond 0/1e-5/1e-4/5e-4 mm positional violation; same budget for newly discovered trial rows. Native validates this budget does not exceed its final length/gap tolerance (1e-3 mm here). This does not soften active equality rows. Two tests check force precision and mandatory activation beyond the selected budget.

Five warmed replays, two warmups/four alternating measured samples per variant. No useful reduction: step777 stays279 LU, step842 stays178 LU or worsens183, normal step1300 remains3, step4245 rises8->9 at1e-4. Withdrawal4895 rises114->234->651 LU at0/1e-4/5e-4, mean-order timings147->286->761 ms. No full run justified. Prototype and tests reverted; no application setting changed.

Unit-test status: one passed; the other failed only its strict +0 versus -0 reaction assertion, after the force-precision and activation-count checks. This rejected prototype was not promoted or claimed to pass its test gate.
