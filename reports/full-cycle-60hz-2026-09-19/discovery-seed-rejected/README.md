# Rejected discovery working-set warm start

Reuse the previous feasible active set and duals when discovery appends wall rows at the same pose; certify all current rows and retain the original single-pivot fallback. Opt-in only, never enabled in the app.

The full 5757-step cycle completed, but factorization count fell only from 28832 to 27933 (3.1%), Newton iterations rose from 16468 to 16557, and saved shapes diverged by up to 77.53 mm (catheter-in RMS 14.07 mm). Mean Node wall step was 12.722 ms versus 12.647 ms for the reference; these are not browser Hz measurements. This does not justify the behavioral change. Reverted both implementation files and the two added unit tests; the prior WASM assembly optimization is retained.

The isolated warm replay at step 777 improved from 279 to 178 factorizations and retained the final pose, but this did not generalize to the whole cycle. See comparison.json, warm-comparison.json and experiment.patch. The 18 focused tests passed before rollback.
