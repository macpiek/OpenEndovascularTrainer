# Reuse unchanged mechanics after the friction chart refresh

Opt-in `reuseFrictionAssembly`, enabled for the separate realtime factory only. `frictionAssembly=0` provides the browser control. The adaptive reference keeps its previous behavior.

An accepted Newton trial already owns its material/inertia gradient and constraint rows. After the friction chart changes at that same pose, rebuild only the friction force, then add normal/length forces in their original order. Do not subtract large old forces from a small residual. The refreshed measure has no valid tangent: the next direction still requires full assembly. Pose, dynamic-step, material identity, loads, reactions, topology and geometry-only declarations guard reuse. Unknown/non-geometric evaluators retain ordinary assembly. Final nonlinear and friction certificates stay unchanged.

The initial prototype merely delayed full assembly and still recomputed material/geometry residuals. It worsened assembly time in several warmed replays and was replaced by the guarded force-only refresh; its patch/results are archived here.

## Node results

Complete 5757-step cycle; all 96 trajectory snapshots exactly equal, 28832 LU /16468 Newton unchanged. Mean step 12.647 → 12.281 ms; mean process CPU 15.663 → 15.248 ms; P95 21.561 → 21.215 ms; max 229.723 → 230.714 ms. Full assemblies including initialization 27953 → 22515 (19.46% fewer). Residual evaluations increase 16577 → 32203, but the new refresh only assembles the changed force. Collision query counts are unchanged. No evidence of fixed 60 Hz from Node timing.

Five warmed incoming-state comparisons (two warmups/four alternating samples per variant) show exact positions and unchanged iteration/factorization counts. Typical step 1300 assembly median 7.281 → 6.031 ms. Hard steps remain dominated by active-set LU work. Scripts reference local /tmp fixtures.

## Browser validation

Sequential enabled/control runs use the same source hashes and no simultaneous Node benchmark. Enabled run completes 5757 steps in 95.955 s, mean physics 59.997 Hz. This is NOT constant 60 Hz: minimum 60-step window 22.954 Hz, max accepted interval 250.9 ms, 944 steps exceed 16.667 ms, peak scheduler backlog 3.400 s. Final backlog 0.033 s, zero dropped steps. Node and browser factorization counts differ (browser 28626), so comparisons are kept within each runtime.

Control completed: 49.843 Hz average, 19.474 Hz slowest window, mean CPU 16.701 ms, assembly 8.014 ms, linear 3.910 ms. Enabled mean CPU 12.583 ms, assembly 6.058 ms, linear 2.795 ms. Because unchanged linear work also became faster, the entire difference cannot yet be attributed to this optimization. Enabled repeat completed: mean 51.316 Hz, minimum window 19.311 Hz, mean CPU 16.387 ms, assembly 7.615 ms, linear 4.005 ms; see browser-enabled-repeat.json. Against the adjacent control this is about 3% mean-Hz improvement and 5% lower assembly cost, consistent with the modest Node gain. The first near-60-Hz mean was not reproducible. Worst repeat steps remain 235.1 ms /232 LU at wire 687.13 mm and 234.0 ms /281 LU at 647.53 mm. No constant-60-Hz claim. These are compact summaries read from the report textbox, not the complete raw browser arrays.

## Validation

The final combined validation passed 54 tests, including checks for exact refreshed force after stick/slide transitions, invalidation before mutation, cancellation rollback, existing coupled friction and promotion. Two anatomical replay tests additionally preserve every direction, accepted trial, complete physical state and final certificates. Incoming geometry discovery caches intentionally survive private attempts and are excluded from physical-state equality assertions.

Build passed. All 5758 Node diagnostic rows (including initialization) have exactly equal status, quality, physical residuals, Newton/LU counts, backtracks, geometry restarts and substep attempts. Temporary browser tab closed; user tab untouched.
