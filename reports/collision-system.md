# Collision system

## Architecture

`npm run collision:build` transforms `res/Aorta_plain.stl` exactly as the simulator does and writes `res/Aorta_plain.collision.bin`. The browser validates the STL SHA-256 and transform metadata, decodes the asset, and creates one `VesselContactField`. It does not run centerline, lumen, or SDF preprocessing at startup.

The asset contains:

- a 3,039-segment acyclic medial centerline tree with endpoint radii, topology, conservative safe radii, and a numeric spatial index;
- packed multi-axis lumen contours used to refine ambiguous inside/outside queries;
- a 0.5 mm sparse signed-distance field in 8 x 8 x 8 bricks, a +/-4 mm wall band, and 0.02 mm distance quantization;
- STL hash, transform, schema version, dimensions, and generation diagnostics.

The centerline is only a broad phase, safe-core proof, and branch locator. No XPBD constraint attracts a tool to the centerline.

## Contact API

`src/physics/collision/vesselContactField.js` exposes allocation-free hot-path calls:

```js
field.querySphere(position, radius, result);
field.queryCapsule(start, end, radius, result);
field.sweepSphere(previous, current, radius, result);
field.queryBatch(positions, radii, count, output);
```

Callers reuse `createContactResult()` or `createBatchContactOutput(capacity)`. Results include `signedDistance`, `signedGap`, `penetration`, `closestPoint`, inward `normal`, `branchId`, `segmentT`, and `timeOfImpact`. Sparse SDF is the primary narrow phase; the STL BVH validates its wall band and is the fallback.

## XPBD world

`EndovascularPhysicsWorld` stores rod state in SoA `Float32Array` buffers and runs at 1/120 s with at most two substeps. Six iterations are used normally and eight above 0.15 mm penetration. The fixed constraint order is sheath, controls, length, bending/rest shape, containment/tool contact, wall, then Coulomb friction. Up to eight allocation-free global length-polish passes enforce inextensibility after the coupled constraints, followed by at most three distributed wall-correction passes. Wall and tool multipliers are warm-started, range changes invalidate only the affected constraint cache, fast motion uses sphere sweeps, and unchanged bodies can sleep.

Containment predicts the matching catheter segment from physical distance and the two tools' segment lengths, not from a normalized active-range ratio. External tool contact includes the guidewire segment crossing the open distal tip so it can contact the catheter side wall without colliding with a fictitious end cap. Newly activated nodes reset velocity and swept history before entering the solver.

The guidewire keeps a straight rest configuration and uses `2e-5` bending compliance, so catheter or wall contact can bend it while an unloaded section progressively recovers toward a straight line. Guidewire-in-catheter containment applies two-way radial reaction plus configurable Coulomb lumen friction (`0.04` by default). Changing the contained distal range invalidates its warm-start state, and the segment crossing the open catheter tip participates in side-wall contact without creating a closed distal cap. Unsupported catheter sections receive their own local shaft straightening even when a shorter guidewire remains proximally inside the catheter. After the main XPBD iterations, up to five coupled wall/fold/length passes resolve conflicting constraints; the final operation is a distributed wall projection with a 0.01 mm settling margin, so a last length projection cannot push a tool back through the wall.

The default URL mode is `xpbd-contact-v1`; `?physics=legacy` selects the previous solver. Tool profiles define a 0.97 mm catheter ID and 1.80 mm sheath ID. Catheter rendering uses an `InstancedMesh`.

## Regeneration

```bash
npm run collision:build
npm test
npm run benchmark:collision
npm run build
```

Commit the regenerated `res/Aorta_plain.collision.bin` with code changes. `out/collision-asset-report.json` is a local diagnostic and is ignored.

## Functional coverage

| Requirement | Automated coverage |
| --- | --- |
| Wall contact in representative anatomy | Cylinder, bend, stenosis, taper, and bifurcation fixtures in `tests/endovascularPhysicsWorld.test.js` verify penetration, length preservation, and finite state. |
| Analytic sheath lumen | Guidewire and catheter are tested simultaneously in one sheath; the open distal end and default configurable 1.80 mm ID are covered. |
| Guidewire inside catheter | Radial containment, unequal sampling, physical-distance mapping, offset lumen start, and explicit two-way catheter reaction are covered; the default configurable catheter ID is 0.97 mm. |
| Catheter without guidewire | Insertion from the sheath, finite unsupported shaft motion, distal progress without bunching, fold limits, guidewire-supported straightening, and preformed-tip recovery after guidewire withdrawal are covered. |
| Elastic guidewire recovery | A bent, unloaded guidewire must reduce maximum local curvature by at least 65% in four seconds while preserving segment length. |
| Tool-to-tool contact | External capsule separation, open distal transition, and Coulomb sliding friction are covered. |
| Friction and fast motion | Coulomb wall, catheter-lumen, and external tool friction plus swept contact at four times control speed are covered. |
| Long-run solver behavior | A 10,000-step run checks finite state, segment length, and fold limits; replay tests compare 30, 60, and 120 FPS inputs and cover sleep/wake plus warm-start reset. |
| Real anatomy | `tests/vesselContactField.test.js`, `tests/aortaXpbdRegression.test.js`, and `tests/guidewireSolver.test.js` validate SDF accuracy/sign, main and small branches, insertion, withdrawal, and absence of wall breaches on `Aorta_plain.stl`. |

## Verification

Final local results on Apple M3:

| Check | Result |
| --- | --- |
| Centerline | 3,039 segments, 109 leaves, 105 branch nodes, one component, no cycles, crossings, backtracks, or wall-crossing segments |
| Coverage | 10,158/10,158 medial samples; 15,887 runtime centerline samples inside the lumen |
| Centering | 0.113 mm average, 1.945 mm maximum; normalized 0.0247 average, 0.2847 maximum |
| Simplicity | 36 sharp degree-two turns, no severe backtracks, 76.86 degree maximum deflection |
| Collision asset | 29.20 MB encoded, 29.18 MB decoded; 31.67 MiB including runtime lookup and sign caches |
| SDF | 0.1578 mm maximum BVH error over 10,858 wall-band samples; zero sign mismatches |
| Real-aorta XPBD | 0.120 mm injected penetration; 0.0001/0.0040 mm settled on main/small branch; 0.0010% maximum segment error |
| Stability | 10,000 steps finite, 0.0001% length error, no acute fold; deterministic at 30/60/120 render FPS |
| Apple M3 Node benchmark | 0.255 ms average, 0.509 ms p95 total physics; 0.019/0.058 ms narrow phase; 0.018 mm post-step penetration; 0.177% length error |
| Legacy comparison | 14.199 ms average, 31.159 ms p95 physics; 1.995 mm maximum penetration |
| Chrome 10-minute acceptance | PASS; 59.952 FPS average, 56.70 FPS 1% low, 1.343/0.700 ms physics average/p95 |
| Chrome frame and memory | 18.7 ms p99 and 83.3 ms maximum frame; 14/4 frames above 33/50 ms; 1.12 MB heap growth, 3.40 MB heap range, zero narrow-phase result allocations |
| Chrome 10-minute geometry | 0.179 mm maximum post-step penetration, 0.235% length error, 106.98 degree maximum bend, finite state over 70,178 measured fixed steps |
| Fresh browser inspection | Skeleton visible after asynchronous load; solo Berenstein advances without guidewire or sheath-exit bunching, follows a guidewire when supported, remains stable during guidewire withdrawal, and recovers its bend when advanced unsupported. Debug overlay reports about 0.10 ms XPBD with zero penetration at rest. |
| Offline generation | 32.1 s for centerline extraction and validation; SDF/sign data reused when source geometry is unchanged |
| Build and tests | `npm test` and `npm run build` pass |

The detailed before/after benchmark is in `reports/collision-benchmark.md` and `.json`. It is a deterministic Node engineering benchmark, not a browser FPS result.

## Browser acceptance

The app exposes allocation-free frame collection and a deterministic guidewire-plus-catheter workload through:

```js
window.__OET_BENCHMARK__.startScenario(); // defaults to 600,000 ms
window.__OET_BENCHMARK__.getScenarioStatus();
window.__OET_BENCHMARK__.getReport();
```

Each 72-second cycle performs guidewire insertion, simultaneous catheter insertion, preformed-tip rotation, catheter withdrawal, guidewire withdrawal, and settling. Pigtail and Berenstein alternate between cycles; the default ten-minute run covers more than eight complete cycles. The report includes average FPS, p99 and maximum frame time, 1% low FPS, counts of frames over 33/50 ms, XPBD phase timings, contact-field counters, heap size when the browser exposes it, and a sampled physics envelope with maximum post-step penetration, length error, bend angle, and finite-state status. Acceptance flags cover duration, 1% low, visible pauses, geometry limits, finite state, and the XPBD mode. The buffer holds 40,000 frames, enough for ten minutes at 60 FPS. `stopScenario()` ends a run early and `getLastScenarioReport()` returns the automatically captured final result.

Chrome on the MacBook Air M3 is the release browser for this version. The current reference is `reports/browser-acceptance-chrome-final-v2.json`. Its full foreground 600-second workload passed every required gate with 59.952 FPS average, 56.70 FPS 1% low, stable heap, no focus loss, zero narrow-phase result allocations, and 0.179 mm maximum post-step penetration. Earlier `browser-acceptance-chrome.json` and `browser-acceptance-chrome-final.json` files are retained as invalid-run diagnostics for focus loss and the wall/length ordering regression respectively; they must not be used to judge the final implementation.

Run the automated browser checks with:

```bash
npm run benchmark:browser:chrome -- --endpoint http://127.0.0.1:9224
```

Safari support remains available for manual testing, but it is not an acceptance gate for this version.

The browser benchmark is a regression guardrail. Re-run the full foreground workload after material physics or rendering changes and before a release. During active physics-engine development, prefer correctness tests and the short smoke check over micro-optimizing isolated scheduler frames.
