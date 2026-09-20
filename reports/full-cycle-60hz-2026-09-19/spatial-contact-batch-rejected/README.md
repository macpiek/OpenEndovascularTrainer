# Spatial contact activation batching — rejected

In each activation batch, add at most one violated wall row per exact DOF support. Keep every row and certify all remaining inequalities on subsequent pivots; a corner can acquire multiple active faces in later pivots. Single-pivot fallback unchanged.

Focused tests passed (9). Immutable step-777 systems: 231 to 214 LU, approximately 130 to 117 ms after warmup. Complete step replay: 279 to 262 LU, 16 Newton iterations. Full 5757-step cycle: 28832 to 27311 LU, 16468 to 16496 Newton; mean Node step 12.647 to 12.632 ms, p95 21.561 to 20.714 ms, max 229.723 to 220.518 ms. These are separate Node runs, not paired browser Hz measurements.

Physical certificates remained within limits but trajectory differences reached 72.19 mm; catheter insertion RMS 14.13 mm. Negligible mean speed improvement does not justify this behavioral change. Both implementation files and added test reverted. Never enabled in the application.
