# Rejected: bounded iterative trial projection

A private dual coordinate-descent correction replaced the exact identity-metric projection used in Newton line search. Eight alternating sweeps produced a candidate increment without LU. Partial results were explicitly `applicable`, not `converged`; physical reactions were unchanged. Original nonlinear force/friction and geometry checks still governed publication. A hybrid tried the exact correction after unsuccessful iterative proposals. Neither mode was enabled by default, and both runtime paths have now been removed.

Four dedicated tests covered the identity-QP solution, coupled constraints, unilateral release, duplicate supports, fixed nodes, partial-result reporting and cancellation. Together with native mechanics tests, 17 tests passed before benchmarking.

On incoming step 1017 from the condensed-contact cycle, reference 586 Newton / 5576 LU became 590 / 4998 with the hybrid (slower: duplicate correction/assembly work). Iterative-only became 594 / 1251. On incoming step 1464, 121 / 1454 became 122 / 233. These isolated reductions did not translate to a complete-cycle improvement.

The iterative-only candidate completed all **5757 movement steps**, dt 1/60, wire and catheter both to 1000 mm and back in reverse order. It retained `wallCompliance=1e-6`, batch release and contact condensation. Node mean **53.574 ms**, P95 **104.688 ms**, maximum **26,892.379 ms**, 3946 steps over 16.67 ms. Totals: **157404 LU**, **27384 Newton**, **43267 full assemblies**, **60028 residual assemblies**. The preceding condensed cycle had 32.477 ms mean and 78008 LU. Worst index 1245, wire 913.0 mm, took 750 Newton / 15778 LU.

All accepted outputs were finite, original force/friction residual <=9.965e-5, maximum relative edge-length error 3.481e-8, sampled physical indentation 0.06752 mm. Nevertheless, shape RMS versus the exact app reference was 23.67 / 11.87 / 11.18 / 8.48 mm by phase; maximum difference 135.48 mm. Geometry checks on 96 snapshots found zero axis-crossing frames, zero far-outside warnings in 51,727 lumen samples, and maximum continuous capsule overlap 0.06562 mm. Snapshot checks are not swept containment proof, and do not overcome the performance/trajectory regression.

Runtime files were restored from their reviewed pre-experiment copies, preserving the preceding opt-in contact condensation. Experimental sources, tests, patches, full data and slow-step replays are archived here. The replay diagnostic now additionally records friction modes and live-normal-load state, and measures the complete advance duration separately from the last substep's `result.ms`.

The next experiment changes Newton globalization: require reduction of the normalized force/torque/constraint residual rather than allowing energy descent to accept a growing imbalance. This is a separate algorithmic choice; no relaxation of the final force/length thresholds is intended. Stable complete-cycle 60 Hz remains unmet.
