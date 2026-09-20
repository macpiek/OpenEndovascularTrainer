# Rejected reuse of the last successful nonlinear strategy

An opt-in four-step memory tried the preceding accepted friction/prediction strategy before the ordinary solver. Failed private attempts rolled back and retried the full original algorithm. Force/length tolerances and physics unchanged.

A single hard incoming state (reference step 1144) benefited from live outer friction: 913 to 289 LU, 58 to 23 Newton iterations. This did not predict the full-cycle benefit.

All 5757 movement steps completed, 200 hint attempts, 3 failed hinted attempts. Mean 37.840 ms versus reference 33.915; P95 110.280 versus 93.964; max 12935.961 versus 1612.864. Movement LU 101340 versus 87269; 3975 versus 4006 over-budget steps. Trajectory changes persisted into catheter phases (RMS 13.25 mm). Strategy memory is rejected and runtime/replay changes reverted. App defaults were never switched. Node timings exclude UI/rendering.
