# Rejected 5 mm contact sampling

Full 5757-step Node cycle completed with continuous axis guard, closed anatomy and unchanged physical parameters. Contact spacing was the only solver override.

| Metric | Reference 2 mm | Candidate 5 mm |
|---|---:|---:|
| Mean step ms | 33.915 | 36.936 |
| P95 ms | 93.964 | 95.579 |
| Max ms | 1612.864 | 18681.646 |
| LU factorizations | 87269 | 107044 |
| Steps over 16.67 ms | 4006 | 3742 |
| Continuous capsule overlap, worst mm | 0.10964 | 0.37926 |

Independent capsule audit covers 96 saved snapshots, not swept motion. The reference itself has small geometric overlaps between discrete contact samples; its grid penetration certificate is not a whole-capsule clearance certificate. Candidate wire insertion trajectory RMS 11.60 mm, max 81.80 mm. Reduced contact queries do not offset increased nonlinear work. Candidate runtime changes reverted; default app was never switched. Archived source files reproduce the optional experiment.
