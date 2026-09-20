# Collision geometry simplification research (not enabled)

The current STL has 1,014,411 triangles. Meshoptimizer 1.1.1 was installed from the local npm cache in /tmp/oet-mesh-tools only; the application dependencies and shipped geometry are unchanged. The experimental generator writes an alternate WORLD-mm binary STL and a source/mesh SHA256 manifest. The benchmark accepts COLLISION_WORLD_STL only explicitly, checks source/mesh hashes, and preserves the original packed lumen/sign field. This isolates finite triangle contacts; it is not a consistent rebuilt production collision asset. Replays from these runs require the same alternate geometry and cannot be replayed against the default anatomy.

Use scripts/physics/simplify-collision-experiment.mjs with MESHOPT_MODULE pointing to meshopt_simplifier.js. It first removes unused duplicate vertex storage after exact welding; otherwise the simplifier conservatively retains every triangle. No approximate weld is used in the retained experiments. LockBorder is enabled; neither component pruning nor sloppy simplification is used. CHUNK_MM enables independent spatial chunks in local coordinates, locking shared borders.

The error returned by meshoptimizer is a quadric error metric, NOT a maximum surface distance. Bidirectional vertex, edge midpoint and centroid sampling is saved separately; these ~140k samples per direction are measurements, not a Hausdorff guarantee.

| Variant | Triangles | Sampled maximum distance (either direction), mm | Cycle |
| --- | ---: | ---: | --- |
| Global, error .05 | 646787 | .4480 | failed wire insertion 312.40 mm, linear-solve |
| Global, error .005 | 814241 | .3152 | failed wire insertion 635.07 mm, linear-solve |
| Global, error .000001 | 815059 | .3152 | not run; geometric error floor |
| 32 mm chunks, error .05 | 371995 | .4935 | not run; large local surface errors |
| 16 mm chunks, error .001 | 988673 | .01050 | failed wire withdrawal at 587.13 mm (step 4956), linear-solve |

Global simplification's error floor prompted local-coordinate chunking. Local chunking with a small tolerance reduces measured error but removes only 2.5% of triangles. Do not infer a performance gain or 60 Hz from these incomplete Node cycles. No new browser performance claim is made.

The 16 mm / .001 variant completed both insertion phases and catheter withdrawal, then failed during wire withdrawal. The rejected transaction took 2771 ms with 91 Newton iterations and 3855 factorizations. Thus even a ~0.011 mm sampled surface change does not establish robustness of the current contact solve.

The fourth cycle candidate, 16 mm chunks / .01 tolerance (877081 triangles, bidirectional sampled maximum .1003 mm), failed wire insertion at 573.47 mm. None of the four attempted cycles completed. No alternate mesh was enabled in the app.

Replaying the 16 mm / .001 withdrawal failure with the correct geometry reproduces exactly 91 Newton iterations and 3855 factorizations: 50 direction solves reported active-set-cycle, 78 incompatible-active-constraints, and 7 active-set-limit (96 converged direction solves; the physical transaction still failed). The next investigation is contact-set feasibility/cycling, not further raw triangle-count reduction.

Research replay now restores collision provenance from the embedded capture or its metadata.json sidecar and verifies the exact mesh hash before restoring face-indexed contact history. A wrong-mesh replay is rejected before integration; the existing reference slow-777 replay still converges with 16 iterations / 279 factorizations. Syntax checks and git diff --check pass. Generated STL files remain in /tmp; JSON reports and generator are retained for reproducibility. The newest manifest includes the generator hash and meshoptimizer version. Older report manifests predate those fields; all used meshoptimizer 1.1.1.
