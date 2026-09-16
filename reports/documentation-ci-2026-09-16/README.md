# Documentation CI and final centerline spacing

The documentation workflow run 35124750715 generated and checked the API documentation successfully. Its combined validation step then failed in `tests/aortaPreprocess.test.js:114`; consequently neither the build nor the documentation commit ran.

## Changes

- Documentation generation, validation and commit remain in `documentation-goal.yml`.
- Application tests and build run independently in `application-checks.yml`, including PRs targeting dev/main/master. Both checks remain visible; a test failure does not suppress the build or block the documentation workflow. Repository branch-protection settings are unchanged.
- The medial centerline pipeline now resamples after final invalid-chain rerouting and cycle pruning, before calculating the published diagnostics. Previously, rerouting could add long edges after the last resampling pass. This only subdivides straight edges and interpolates radii; it does not move the existing path. Diagnostic counters expose the added nodes and time.

## Verification

Local Node v24.6.0 on the repository anatomy:

| Measurement | Before | After |
| --- | ---: | ---: |
| Maximum final segment length | 4.4820274085645035 mm | 1.9999974410810009 mm |
| Maximum normalized centering offset | 0.6747849458049552 | 0.6747849458049552 |
| Invalid final segments | 1 | 1 |
| Sharp turns | 89 | 89 |
| Maximum deflection | 82.32540722537209 degrees | 82.32540722537209 degrees |

`node tests/aortaPreprocess.test.js` reproduced the spacing failure before the change. Afterward, the spacing assertion passes and the test stops at the next existing violation: maximum normalized centering offset must be below 0.37. The table documents that this value was already present before the fix. The full anatomy test is therefore still **failing**, and no acceptance threshold was relaxed.

- Production build passed, with the existing large-chunk warning.
- Documentation generation followed by `--check` passed in an isolated copy using the actual source directory.
- Both workflow files parse as YAML; `node --check src/stlCenterline.js` and `git diff --check` pass.
- Final resampling took 1.834 ms in the measured run. This is not a solver performance benchmark.

No claim is made that all application tests now pass. Remaining anatomy quality problems and previously known physics-test failures require separate repairs.
