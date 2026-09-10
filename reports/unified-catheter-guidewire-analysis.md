## 2026-09-10: catheter feed now uses a fixed material reservoir through the sheath

User explicitly requested the same boundary-driven insertion mechanism as the guidewire. Added catheterSheathTransport: stable full material chain, constant rest lengths, prescribed position/velocity only inside the straight sheath, moving active window, and persistent exposed node state. PigtailCatheter uses it whenever an introducer is present; it no longer records the guidewire route or runs external shape/path seeding for app feed. Constitutive profiles and handle twist still use the existing common solver. Simulator containment arc offset and length are shifted by the proximal material origin; shielding still uses actual insertion distance. Existing coupled idle velocity policy retained. Preview without a physical sheath retains its old layout path.

Eight focused transport tests pass, including exposed state/history preservation, signed feed, fixed labels, boundaries, no wire-path reads, Float32 idle sleep and explicit reset. Before final reset/idle additions, combined14/14 transport/parity/three-profile over-wire tests passed; later quick updated11/11 transport+parity passed. Over-wire fixture now measures moving material inlet and shifted arc origin while retaining its tracking/inextensibility/kink gates. Build passed before final minor reset/render changes.

Browser default solo20s probe advanced catheter to55.2cm (1275 steps/10.625 simulated seconds), finite;86 nonlinear failed steps, mean47.42FPS,9.409s backlog, meanstep10.915ms/P9546.9ms. This is NOT full-cycle/performance acceptance. Short-catheter20s and interrupted60s probes did not reach catheter feed (wire prefeed dominated); no browser coupled insertion acceptance yet. Broader nonlinear/contact and60FPS goal remains incomplete.

A user noticed the proximal catheter disappeared: render had incorrectly used only the free solver active range. Restored the visible proximal reservoir independently of active physics, using its already prescribed axis positions. This adds no free mechanical DOFs. Server5173 started from this worktree.

## 2026-09-10: principal Cholesky deletion reduces frozen contact solve cost

Implemented optional choleskyDeletes for dense source QPs. A working-set transition removing exactly one free row updates the existing Cholesky factor by a stable rank-one update; other changes and failed updates use original full factorization. Original residual/KKT, numerical shift, physical constraints and tolerances stay unchanged. Added factorUpdates accounting through friction/load/seed. Helper and integration tests pass12/12;35 related friction/load/skyline tests pass. Integration covers the actual185-row anatomy fixture and changed matrices, sizes and working sets in skyline/band modes.

Replay now alternates variant order with4 warmup and24 measured samples per variant. On the saved browser operator: baseline mean12.881ms/median12.787/P9514.996 versus deletion mean10.304/median10.327/P9511.489. Full factors146→13 plus133updates. Both original maximum KKT residuals exactly1.217729144908919e-6, cone violation2.22e-16. Evidence supports about20% lower time on this frozen inner problem only. reports/wall-witness-frozen-replay.json holds samples. A bounded browser probe was started, but its result was not retrieved before browser context changed; it supplies NO performance evidence. Removed runtime activation pending browser verification; optional implementation remains available.

The default solo catheter already enters the global joint-components physical solve. Its incomplete nonlinear convergence remains unresolved; faster linear algebra alone does not establish global relaxation, deep/max60FPS, adaptive discretization or completion of the goal.

## 2026-09-10: Newton-first experiment rejected; reproducible browser operator captured

Added optional tryNewtonBeforeSeed: a bounded Newton probe may skip seed ONLY after its original full KKT succeeds. Failure discards the probe iterate and uses the unchanged seeded path, accounting for both attempts.3 tests verify seed bypass, exact fallback outputs and independent saved-system KKT. Browser20s probe:668steps,24.16FPS,3 nonlinear failures, last813.8ms/core671.9ms/seed292.2ms. No runtime improvement established; removed probe flag from witness variant. Default and witness selection now have no experimental seed overrides.

Added opt-in captureCoupledOperator=1 and localhost-only Vite development capture route writing fixed /tmp/oet-frozen-coupled.json. Captures an actual loaded seed system (original bounds, source Coulomb groups, numerical shift, active hints), not altered final radii. Stored185row/56group operator in tests/fixtures/kirchhoff-wall-witness-browser.json.gz. Captured seed has4 friction iterations,145 inner iterations/factorizations and21 near-null pivots. This is a repeatable inner solve; it does not recreate the entire nonlinear world step.

scripts/physics/replay-wall-witness-solve.mjs runs four repeated identical-operator solves per variant and independently reconstructs rhs−Aλ to audit original KKT/cones. Report: reports/wall-witness-frozen-replay.json. Warm fourth samples on this machine: baseline19.15ms/146factors, bandLU14.29ms/146factors, shortSeed26.91ms/69factors, NewtonFirst17.30ms/6factors. All pass original tolerance. This fixed-order small sample is not a rigorous speed claim; fewer factors alone do not ensure lower CPU time. Next optimize/profile the145 active-set factorizations in the exact seed fixture, now without browser trajectory confounding. Full mechanics/adaptive discretization/60FPS goal remains incomplete.

## 2026-09-10: reject inner-seed cap and full-band runtime experiments

Added optional maxSeedActiveSetIterations passed only into helper fixed-load QP. Saved-system tests now compare default, outerseed4 and outer4/inner16; all pass independent original full-operator KKT/cones/bounds. Browser outer4/inner16,20s wall-limited:669steps,23.60FPS,9 nonlinear failures, max penetration.0115378mm, last1538ms step (core1314ms/seed422.7ms,21snapshots/30restores). Worse nonlinear result than preceding outer4 run; removed both seed caps from witness selection. Options/tests remain available, not active defaults.

Experimented existing full-band original-row solver with witness contacts and no seed caps.20s diagnostic run:607steps,22.36FPS,0 nonlinear failures only in that incomplete range, max penetration.000961805mm, segment error.0119650%; mean28.5824ms/P95135.6ms. Last847row full-band band-LU solve105.1ms (118.4ms step), final original residual1.0e-10. It progressed less far than condensed677steps, so no evidence of runtime advantage. Restored witness selection to activeCondensation+simultaneousCoulomb without added overrides; ordinary default remains unchanged. These truncated runs do not establish full mechanics/60FPS.

Read-only provider fixture also confirms exact duplicate HARD normal rows at adjacent segment0/t1 and segment1/t0 on one triangle: identical translational J and strain, nullspace in Gram. Tangent rows however use different material-frame angular DOFs, so merging by endpoint would remove radius torque and alter mechanics. Nonzero normal compliance further rules out naive deletion. Proper algebraic force-allocation/nullspace handling or a certified Newton-first predictor is the next justified experiment; no geometry/force deduplication was implemented. Goal remains incomplete.

## 2026-09-10: fixed-load seed dominates the measured contact solve

Added active-condensation stage timings and seedMs accounting (including cold retry). A bounded browser witness run stopped at677steps: core259.4ms, contact solve242.5ms, fixed-load seed215.0ms, Schur11.3ms, equality setup1.2ms. Thus the fixed-load seed dominates this sample; the earlier code-only suspicion that dense Newton LU dominates is NOT supported here. Full-band changes alone are not yet justified. Existing older fixtures show no uniform full-band advantage.

Added optional maxSeedFrictionIterations; it limits ONLY the helper fixed-load friction iterations. Final simultaneous Newton receives unchanged options/tolerances and independent original KKT still gates acceptance. Explicit witness variant currently experiments with4; ordinary application default remains unchanged. Two new saved-system regressions independently reconstruct full rhs−Aλ, check cones/bounds and input ownership; both pass (2002row friction-cycle,2000row normal-load-cycle). Seed iterations observed<=4.44 relevant existing tests also pass.

Follow-up20s diagnostic browser run ended at678steps, mean24.24FPS,3 nonlinear failures, peak segment error.0122676%, poststep penetration.00458905mm. Last step core581.6ms, seed490.9ms, contact550.5ms, Schur21.5ms: shorter outer seed iterations alone do not remove the pathology. Different terminal step/trajectory means no controlled speedup claim. Next inspect inner active-set QP work inside the seed, capture/replay the actual slow operator and compare bounded seed strategies without changing final gates.

Added optional benchmarkWallLimitMs URL parameter to stop diagnostic runs at a safe step boundary with reason diagnostic-wall-limit. It preserves simulated-time accounting, marks incomplete duration and cannot count as full-cycle acceptance. Needed because an earlier unbounded probe fell below1FPS and UI Stop commands timed out; navigation recovered it but lost that unfinished profile. No full/deep/max60FPS acceptance; goal remains active.

## 2026-09-10: rollback graph reduction and core-solver cost isolation

Added per-step jointCosts: assembly, core solve, snapshot, restore and measurement durations plus snapshot count/bytes/object count. Removed duplicate wall friction measurement assembly (world passes a descriptor; measure owns one independent row bank). TrialState now excludes disposable witness solve/measure graphs, preserves committed flags/version guards for frozen solve batches, and excludes _jointTrialState from whole-component base captures. Physical witness normal/tangent journals remain captured. Measurement aliases through _jointStateMeasurement are also barriers. Added exact rollback/partial-retry and repeated-capture bounded-size tests.

Representative204row/68witness probe: trial28748→2528 objects,324234→59146 typed-buffer bytes, single measurement53.2→3.6ms. Whole-base217723→11213 objects,972702→145642 typed bytes,503.7→16.0ms. These timings are a single Node probe, not browser performance proof; typed bytes exclude JS metadata. /tmp/oet-trial-barriers-result.json contains the bounded comparison.

Browser after main graph reduction: manually stopped at727steps, mean13.22FPS,36.2246s backlog,5 nonlinear failures. Physics mean53.9527ms/P95387.1ms/max1265.1ms (different stopping depth from675step baseline, not a controlled speedup claim). Last728.3ms step: core solve620.9ms, measurement70.1ms, snapshots13.8ms, restore10.1ms, row assembly8.4ms;248rows,7snapshots,12restores, snapshot4736objects/775124bytes. Heap range.498GB. The dominant remaining cost is inside solveKirchhoffCoupledSystem / active-condensed Coulomb, NOT collision query or snapshot. Next profile its assembly/condensation/active-load solve/reconstruction and resolve nonlinear friction/fold failures. Full goal remains incomplete.

66 focused tests passed before the final measurement-alias addition; delegated final31 rollback/friction/world checks passed after it. No physical tolerance, step frequency or feed law changed. Application default remains joint-active-coulomb; witness variant remains explicit.

## 2026-09-10: static/kinetic world retry and first browser witness run

Integrated position-history wall mode controller into the complete predicted-component solve. Feed/prediction execute once; a converged static-slip candidate triggers restoration of the predicted pose and zero reaction journal before cumulative kinetic overrides. Fresh measurement.settled is required alongside previous closure. New real nonlinear corner test uses app wire coefficients.006/.002 and asserts identical pose/history at each attempt, zero applied forces on retry, one time increment and restart→accepted.63 focused/controller/world/selection tests pass. Added explicit joint-wall-witnesses application variant; default unchanged.

Browser initially stopped after199 steps at8.6cm. Added firstError/lastAttempt to benchmark scheduler report: original error was Active wall has no represented finite face, later masked by World repeated preparation of a pending timestep. Fixed active-wall refresh to preserve finite query mode. More decisively, catheter topology remap copied wallActive/branch/gap but omitted wallFaceIndex; added face, inside clearance and sample-count transport and face=-1 reset. Browser then advances beyond the failure. A proposed synthetic topology fixture did not exercise a shift and was removed rather than claiming coverage; browser reproduction is the current integration evidence.

Post-fix browser trial stopped manually after675 steps /5.625 simulated seconds in45.060s wall time (not a full cycle). Mean11.95FPS,38.3931s backlog, zero dropped steps. Physics mean59.6973ms/P95 161.3ms/max3262.3ms, dominated by constraints (mean59.3547ms); narrow phase mean.2514ms. Last206 rows. Heap peak1.75GB vs starting.741GB (end.793GB).3 nonlinear failures, firststep667 dominated by witness friction/fold merit; peak poststep penetration.000833955mm, peak segment error.0126176%. This is emphatically NOT acceptance and not comparable to a full-cycle baseline. Need profile nonlinear assembly/snapshot/friction blocks and close residuals; exact queries alone are not dominant. Restored browser to default joint-active-coulomb after test. Goal remains active; adaptive model and60FPS deep/max still incomplete.

## 2026-09-10: finite mesh query coverage for opt-in witness world

Added explicit finiteMeshContacts argument to queryCapsuleSoA. It uses the existing capsule sample grid and sign classification, but evaluates exact closest-triangle distance at EVERY sample before choosing the minimum. Merely refining the SDF-selected winner is insufficient. This mode bypasses the BVH proximity gate and safe-core physical gap, requires a BVH, isolates endpoint caches from ordinary/physical-SDF queries, and restores transient mode state. World requests it only for coupledSystem.wallWitnesses; application selection remains unchanged.

Real aorta STL tests compare the result against independent MeshBVH.closestPointToPoint over all9 samples and verify the same winning sample/distance. Captured step663 exact contact matches the previous independent mesh reference, including face325170 and positive clearance where default SDF reports penetration.6 query tests pass;10 combined contact-field/sampling/world tests pass; build passes. This removes missing face coverage from this query route, not all unsupported signed/material transitions in the witness provider. It is a correctness-first opt-in, with no browser performance claim.

Unequal static/kinetic coefficients remain another integration prerequisite (app wire.006/.002, catheter.002/.002). Delegated work prepares a static-candidate/kinetic-retry controller preserving position-history static-first behavior; whole-step rollback/retry is not yet integrated in world. Do not enable the path by altering profile coefficients. Goal remains incomplete.

## 2026-09-10: wall-witness friction integrated; retain discoveries across rejected trials

The opt-in position-history witness path now assembles per-witness two-dimensional Coulomb friction with the normal rows and radius torque in the actual common Kirchhoff block. No environment DOFs enter the solve. The surface wrench ledger tracks signed tangential forces and moments; release scales normal and tangent histories together while normal wallProjection remains separate. World skips the old post-step tangential Coulomb subtraction for this path, preventing duplicate friction.

A transport carrier removes the difference between the applied tangent wrench journal and the current tangent basis/arm representation in the SAME global block. Partial corrections retain the remainder. Measurement reports physical transport displacement/angle with actual mobility and hard-frame masks; existing mm/rad tolerances govern acceptance. Nonzero sub-tolerance journals are not deleted. Pending transport count is diagnostic, not an impossible exact-zero geometric closure gate.

The new sliding-catheter/two-finite-wall world test initially failed at pass3: an accepted contact correction exposed another endpoint/face, merit rose from7.8258 to9.0834, and rollback forgot that feature. World now restores pose/history and retains ONLY new geometry identities with zero force, re-linearizing at the restored base. Test asserts zero normal/tangent/wrench state for retained candidates and converges with unchanged final gates. Step-local unloaded candidates are pruned after the physical step.

Validation:65 focused tests pass. Full coupled suite:705/708 pass; three failures remain (nonsymmetric matrix-format expectation and two loaded external mouth lifecycle cases). All three reproduce using the saved pre-integration world through a Node loader; they are unresolved baseline issues, not a green full-suite claim. Build passes. Provider finite-edge tests verify actual block reaction transport at scales1 and.25.

Application default remains unchanged: wallWitnesses is NOT enabled in coupledSolverSelection. Still unsupported: unequal static/kinetic wall coefficients, active contacts without finite mesh face, retained signed branch/material support changes. Must complete real anatomy coverage and compare actual full insertion before enabling. No new browser/FPS result. Goal remains active: full mechanics, adaptive discretization and60FPS/120Hz acceptance are incomplete.

## 2026-09-10: multi-witness normal provider integrated into Kirchhoff world (opt-in)

Implemented kirchhoffWallWitnessRows with separate material/face/sample identities, finite-triangle geometry, per-witness reaction ledger, global release rows and final pending-discovery/release gates. New features start at zero lambda. Loaded old features remain independent. No scalar-force transfer to a new face. Collection is called both before solve and after exact trial contact refresh; frozen commit precedes trial recollection. World switch is coupledSystem.wallWitnesses=true; default application selection remains false.

World integration test uses actual coupled Kirchhoff material solver at two finite perpendicular walls. It discovers and retains more wall rows than segments and achieves common nonlinear closure without local wall projection.57 targeted provider/world/geometry/ledger/rollback/baseline tests pass; build passes. Tests include200 unloaded face/t/material changes with bounded storage, retention of lambda0 but nonzero moment, partial/full releases and nonlinear rollback identities. Added tests to coupled suite.

Explicit remaining integration limits: current provider is position-history, represented finite mesh faces/interior branch only; unsupported domain/material changes fail explicitly. Multi-witness wall friction is NOT integrated, so world rejects opt-in with nonzero static/kinetic wall friction. Normal-load aggregation alone is insufficient for tangential reactions; do not enable in app by turning friction off. Next implement physical tangential state/reaction per witness using existing Coulomb machinery, and finite-domain release/discovery coverage for real anatomy; then app/browser deep/max validation. No new FPS claim and goal still active.

## 2026-09-10: finite wall witness and reaction primitives implemented

Added kirchhoffWallWitnessGeometry: evaluates a retained finite indexed triangle with closest point, barycentric face/edge/vertex support, unsigned distance and direction. Explicit zero-distance undefined normal and degenerate rejection; no infinite tangent plane or global winner/sign assumption. Tests include actual mesh faces325178/325433 and independent numeric derivatives.

Added kirchhoffWallReactionLedger: captures PRE-APPLY wall forces and moments, journals ACTUALLY scaled multiplier increments per witness, and builds fixed-carrier release rows for the existing Kirchhoff coupled system. Independent Three.js wrench oracle checks translation/rotation transport. Real common solves verify release at scales1,.5,.125 and wallProjection bookkeeping. Existing nonlinear snapshot restores ledger identity/values and removes newly discovered map entry on rejected trial. All11 primitive tests pass; added to coupled test command.

These primitives are NOT yet wired into world boundary collection. Next implement multi-witness provider with persistent material/face/sample ownership, evaluate retained finite support, discover new features with zero lambda, include simultaneous reaction releases and final pending-release gate. Preserve borrowed field barriers during snapshot. Do not route through rejected composite mechanics; only reuse bounded geometry primitives. No new browser performance claim this turn; app remains physicalGap with original BVH band and original nonlinear merit.

## 2026-09-10: mesh reference identifies SDF artifact; nearwall-only integration rejected

Actual STL/BVH step663 reference returns +.016040404mm clearance where SDF returns -.004506161mm penetration. Stable face325170, numeric unit-gradient agreement ~6e-12 directional. Captured reference regression remains in vesselPhysicalCapsuleGap tests using explicit .2 BVH configuration; it is NOT the application configuration.

Experiment: physicalGap-only BVH band .2 in winner and refinement gates. Full28sim cycle took96.7345s,28.4088FPS,68.7305s backlog,1683 nonlinear failed steps (vs164), peak length2.28617%, penetration.140293mm. First failure664 switches face325178→325433 under small trial while retaining one segment lambda. This is strong evidence the one-wall-row-per-segment representation cannot robustly globalize changes in active surface. The .2 runtime band was reverted; default runtime still physicalGap safe-core exclusion with original BVH settings. No final accuracy criteria changed.

Next structural task: persistent loaded wall witnesses with separate multipliers/gradients for newly discovered features, simultaneous release/activation and reciprocal reaction; avoid silently transporting one loaded multiplier to a new face. Validate on captured first failure before enabling mesh-based contact runtime again. Retain ordinary collision detection behavior. Goal remains incomplete, including adaptive mesh and60FPS coupled deep/max.

## 2026-09-10: physical-gap derivative audit and merit combination

Actual step663 replay (`scripts/physics/probe-capsule-asset-derivative.mjs`, `reports/solo-catheter-step663-derivative.json`) shows continuous gap but a trilinear cell kink at z11.5. The trial direction improves the gap before the boundary and worsens it after. Returned SDF normal is normalized whereas gap uses unnormalized interpolation; gradient norm .954923 at base and .819743 at rejected state. Thus the boundary Jacobian is inexact for the SDF gap, even away from the cell kink. Merely replacing the gradient would also require auditing multiplier/normal force/friction units; no such unreviewed change made.

Natural-map fold/boundary merit retried on top of physicalGap: full28sim seconds in71.238s,31.117FPS,205 nonlinear failures,2.34051% max length error,.184719mm penetration,43.2473s backlog. Compared with164 failures/35.09FPS this did not resolve the problem; world merit reverted to original, keeping physicalGap and wall ownership fixes.52 targeted tests passed on experiment. Next compare this same contact to actual STL/BVH to distinguish physical geometry from SDF interpolation artifacts.

## 2026-09-10: reproduced safe-core gap discontinuity; separate physical query

Actual browser step1050 is now replayable in `scripts/physics/probe-capsule-asset-replay.mjs` using `reports/solo-catheter-wall-jump-replay-input.json`. With actual collision asset and app STL/BVH settings, gap jumps .942691342 mm for endpoint displacement1.86e-15 mm: centerline-safe-core (t.5, branch354) switches to sparse-sdf (t1, branch355). Samples remain2, faces-1; same without BVH. This is a conservative clearance certificate incorrectly used as physical contact gap, distinct from the synthetic coarse-winner/BVH discontinuity.

Added opt-in `queryCapsuleSoA(..., physicalGap=false)`. Physical mode omits safe-core shortcut, samples all existing capsule samples, and separates endpoint cache by mode. Default collision queries unchanged. Common world wall rows use it for joint and joint-components. Replay test proves removal of this particular threshold jump; no claim of global SDF/BVH continuity. All41 focused integration/query/rollback tests pass and build passes.

Full browser solo cycle after physical-gap integration: 62.3814 wall seconds for28 simulated seconds,35.0895 FPS,34.3833s backlog,164 nonlinear failed steps (previous189); peak length2.45335% (previous1.55113%), penetration.193329mm. Thus the representation bug is fixed but whole mechanics/performance still fail; retain explicit evidence, do not claim acceptance. First failure step663 now wall node97/t1/branch351 near z11.5, normal changes strongly across a small move and penetration grows .00450485→.00451779mm despite correct linear solve. Peak1050 includes the known discontinuous final-fold KKT used in merit. Next audit SDF gradient vs distance interpolation near voxel boundary and reconsider continuous merit after geometry representation is consistent. Final tolerances unchanged.

## 2026-09-10: fixed singleton wall reaction ownership

`#prepareWallContacts` treated `joint-components` as the old route: it silently zeroed a loaded wall multiplier when gap exceeded activation or branch changed. Both guards now preserve the loaded row exactly as `joint`, allowing the coupled solver to unload the reaction. Regression tests use real solve/apply; both failed on the saved pre-fix world (expected lambda 0.009999990366978635, got 0), and pass after the fix. Added to test:physics:coupled.

Full browser solo cycle after this fix: 3360 steps, 28 simulated seconds, 65.2569 wall seconds, mean 34.3058 FPS, backlog 37.2659 seconds; nonlinear failed steps 189 (baseline 321), maximum length error 1.55113% (baseline 2.18968%), maximum post-step penetration 0.138455 mm. Not accepted: nonlinear closure, length and performance goal still fail. At peak step1050 material and positional fold decrease, but boundary merit jumps 125.1217 to1068.3526 even at scale0.288724/128. Restored merit exactly equals base. Next investigate which wall/sheath/control row changes and its contact feature/force ownership.

The separate equation-only merit experiment (natural-map fold/boundary; length/positional fold omitted from line search only, final gates unchanged) was rejected: 1633 nonlinear failures, 24.7648 FPS, peak length2.45023%. It was reverted BEFORE the wall guard fix. Current world retains the original merit and exposes per-term baseline/rejected/restored failure diagnostics.

Numerical derivative audits of boundary and director fold pass through actual correction application. They also demonstrate that positional-fold derivative differs from the director row off the adaptation manifold; this alone does not establish a working globalization replacement.

## 2026-09-10: solo catheter nonlinear diagnosis — continuous merit experiment rejected

The browser full solo cycle (3360 fixed steps, 28 simulated seconds) confirms `joint-components`, individual basis, 13590 converged linear solves and zero linear failures. It still has 217 nonlinear failed steps. A continuous natural-map merit for boundary and fold rows reduced the previous 321 failures but increased peak segment error from 2.18968% to 3.21287%, peak post-step penetration to 0.188125 mm; mean FPS 30.8419 and wall duration 71.888 s, backlog 43.8827 s. Therefore the world merit integration was reverted; tested provider APIs remain unused for future controlled work. This is not a physics or performance acceptance.

First failure step 668: previous/restored merit both 2.96789700678346, rejected merit 3.0951956978526596 after eight trials. Peak length failure step 1080: previous/restored merit both 133.08107815182987, rejected 147.53160663602796. Rollback reproduces the base merit; failure is nonlinear acceptance, not failed linear solve. Final geometry and KKT tolerances were never relaxed. Need address nonlinear globalization and geometry consistency before declaring global relaxation fixed.

Provider/world/rollback targeted tests: 47/47. Build passed for experiment. Latest runtime restores previous merit with additional failure diagnostics.

# Wspólny model cewnika i prowadnika: diagnoza wydajności i projekt przebudowy

## Wynik pełnego cyklu solo — 2026-09-10

Próba wykonała 3360 kroków / 28 s fizyki w 67,51 s rzeczywistych, czyli
obejmowała hold i withdraw. Średnia 31,80 FPS, backlog 39,51 s. Wszystkie
13574 rozwiązania liniowe zbieżne, ale maksymalny błąd długości 2,19%
(catheter node145, step1050) nie przeszedł istniejącej bramki lengthPass.
Maksymalna penetracja po kroku 0,116 mm. Nie uznawać mechaniki za zaliczoną.
Wybrane pola w browser-solo-catheter-full-cycle-2026-09-10.json.

Dodano osobną diagnostykę nieliniowej porażki kroku (material/boundary/
fold/orientation, line-search, numer iteracji). Envelope liczy takie kroki
i zachowuje pierwszy przypadek oraz stan przy maksymalnym błędzie długości.
Nie zmienia to równań ani kryteriów; kolejna próba musi rozróżnić limit
iteracji od odrzucenia nieliniowej korekty przy zbieżnym układzie liniowym.
Root potwierdził testy nowej osi singla i dotychczasowej pary poleceniem
node --test AxialSingleComponent/AxialReducedSystem/AxialLayout/AxialWorld.

## Pełny cykl solo i dalsza redukcja — 2026-09-10

Próba catheter-only liczy teraz zakończenie i postęp z czasu wykonanej
symulacji, jak istniejące próby deep/short-catheter. UI „Sam cewnik pełny
cykl” musi dojść do advance/hold/withdraw/hold łącznie 28 s fizyki; przy
zaległości trwa dłużej w czasie rzeczywistym. Pomiary wydajności nadal
bazują na rzeczywistym czasie. 22 testy scenariuszy/single world/istniejącego
BundleDiscretization przechodzą. Nowego pełnego przebiegu jeszcze nie
wykonano, więc hold/withdraw w anatomii pozostają niezweryfikowane.

Audyt adaptacji: buildAdaptiveBundleMesh istnieje wyłącznie jako moduł
z testami, bez podłączenia do obecnego solvera Kirchhoffa. Nie uznawać
go za ukończoną adaptacyjną dyskretyzację aplikacji. Delegowano tożsamościowy
layout i reduced assembly dla singletona; root usunął wcześniejszy jawny
reject includeAxialLayout, testy dostarczanego rozszerzenia nadal w toku.

## Sen niezależnych komponentów — 2026-09-10

Usunięto regresję: wspólna pętla budziła także cały uśpiony, niezwiązany
komponent przy ruchu drugiego narzędzia. Teraz można pominąć jego ponowne
rozwiązanie wyłącznie gdy wszystkie ciała śpią i komponent ma potwierdzoną
zbieżną równowagę. Nie pomija to dt ani ruchu aktywnego komponentu.
Nieudana równowaga budzi ciało i nie pozwala sleepCounter ukryć błędu.

Selektor obserwuje także aktywną parę. Ponowne wejście właściciela po
merge/split unieważnia dawną ocenę równowagi, zachowując jego tożsamość.
Testy sprawdzają brak rozwiązania uśpionego niezależnego narzędzia,
ponowienie po błędzie oraz merge/split obu śpiących ciał. Wpływ na czas
kroku aplikacji nie został jeszcze zmierzony.

## Sam cewnik w anatomii i zerowy overlap — 2026-09-10

Dodano UI „Sam cewnik 28 s”: Berenstein, 12 s advance, 2 s hold, 12 s
withdraw, 2 s hold czasu symulacji, prowadnik zawsze nieruchomy. Test
scenariusza obejmuje granice faz. Pierwsza próba wykryła, że aplikacja
włącza containment przy zerowym inserted: indeksy były równe, lecz
containedLength=0. Warunek wymaga teraz inserted>0.

Powtórna próba potwierdza joint-components, 5438 solve, zero niezbieżnych,
wire 0 cm, catheter około 49,4 cm. Średnia 28,79 FPS, backlog 18,61 s,
ostatni krok 95,2 ms. Wykonano tylko 9,49 s fizyki w 28 s ściennych,
więc faza hold/withdraw NIE została przetestowana. Nie uznawać tego za
dowód poprawnej relaksacji po zwolnieniu przycisku. Raport wybranych pól:
browser-solo-catheter-components-2026-09-10.json. Cel wydajności niezaliczony.

## Włączenie komponentów w aplikacji — 2026-09-10

joint-active-coulomb ustawia independentComponents=true. Samodzielne
narzędzia korzystają ze wspólnej pętli materiał+normalna ściana. Test świata
z płaską ścianą potwierdza reakcję odległego węzła, brak penetracji ponad
obecny próg i nieliniową resztę materiałową. Test przejścia do existing lumen
i z powrotem zachowuje tożsamość właścicieli. Tarcie ściany pozostaje w
istniejącym velocity pass; nie twierdzić, że jest już globalnie sprzężone.

Próba UI 35 s: independentComponents=true, 10419 solve, zero niezbieżnych,
średnia 34,77 FPS, backlog 17,58 s, ostatni krok 366,6 ms. Odczyt w 30 s:
wire 66 cm, catheter 12,7 cm. Brak wyjątków w odczycie konsoli. Wybrane
pola raportu: browser-independent-components-2026-09-10.json. Nie spełnia
celu wydajności i nie jest osobną próbą samego cewnika w anatomii.
Axial-sections jeszcze nie obsługuje singla i nie dostał tej flagi.

## Krok świata dla komponentów — 2026-09-10

Zintegrowano selectKirchhoffMechanicalComponents. Przy jawnym
coupledSystem.independentComponents i position-history, bez aktywnego
joint containment, świat rozwiązuje rzeczywiste komponenty tą samą pętlą
nieliniową. Kolektory lumen geometry/residual pomijane tylko przy braku
lumen; materiał, boundary, fold, orientation i normalna ściana pozostają.
Kontakty zewnętrzne są filtrowane według członków komponentu. Zachowano
stare ścieżki bez flagi; UI jeszcze jej nie ustawia.

84 testy komponentów/world/core/providerów przechodzą; build poprawny.
Nowy test wykonuje stepFixed pojedynczego ciała, potwierdza wywołanie
wspólnego solvera i nieliniową resztę materiałową poniżej obecnych progów.
Nie obejmuje jeszcze kanału naczyniowego ani przejścia do overlap.
Nowe testy świata i doboru komponentów dodano do listy testów package.json.

## Providery komponentów i tarcie zewnętrzne — 2026-09-10

Zintegrowano delegowane boundary/fold/orientation dla rzeczywistej liczby
ciał. Bezpośrednie testy providerów obejmują zgodność równań singla z parą,
commit/reset oraz zmiany tożsamości i topologii. ExternalFrictionRows
obsługuje jawne bodies dla pary; geometria powierzchni nadal dostaje dwa
rzeczywiste ciała poprzez własny adapter. Singleton nie tworzy tarcia
narzędzie–narzędzie i nie alokuje stanu kontaktu bez partnera. To nie zmienia
tarcia ściany. Commit odrzuca podmienioną kolejność ciał.

Wspólny zestaw testów: ExternalFrictionRows, BoundaryRows, FoldRows,
OrientationRows, CoupledSystem, TrialState i AxialWorld przechodzi.
Delegowano osobny moduł doboru komponentów grafu kontaktów; integracja
world i próby solo w UI pozostają następne. Obecne zmiany nie są dowodem
60 FPS ani ukończonej relaksacji pojedynczego narzędzia w aplikacji.

## Rollback i residual pojedynczego komponentu — 2026-09-10

capture/restoreKirchhoffCoupledTrialState korzysta z jawnej listy rzeczywistych
ciał. Liczba korzeni i filtry snapshotu odpowiadają liczbie ciał; restore
sprawdza także niezmienioną liczbę i tożsamość przed zapisem. Pomiar
nieliniowej reszty materiałowej obsługuje tę samą listę.
46 testów TrialState/CoupledSystem/AxialWorld przechodzi. Nowe przypadki
sprawdzają przywrócenie mnożników materiału/ściany oraz lokalnego boundary,
niezmienianie drugiego narzędzia, ponowne użycie snapshotu i odrzucenie
zmienionego ciała. Residual singla porównany z wkładem w niezależnej parze.
Integracja w world pozostaje nieukończona; lumen geometry/friction i
external friction nadal wymagają rozdzielenia providerów według rzeczywistych
kontaktów komponentu. Nie włączono niepełnej ścieżki solo do UI.

## Jednociało w jądrze wspólnego solvera — 2026-09-10

Dodano jawny kontrakt component.bodies (1 lub 2 różne ciała), zgodny z
legacy innerBody/outerBody. assemble/solve/apply iterują po rzeczywistych
ciałach; wynik zawiera bodies/responses oraz aliasy inner/outer dla pary.
Apply sprawdza tożsamość ciał przed zmianą stanu. Nie tworzy się drugiej
siatki. Lumenowe rekordy singla są odrzucane. Axial layout singla jest
jeszcze jawnie nieobsługiwany, więc zmiana nie uruchamia go w UI.

75 testów CoupledSystem/AxialReducedSystem/AxialWorld przechodzi.
Nowy test materiał+normalny kontakt ściany dla pojedynczego ciała porównuje
całą korektę i reakcję z odpowiednim blokiem niezależnej pary, sprawdza brak
dodatkowych DOF oraz aplikację i odrzucenie zmienionej tożsamości ciała.
To test kierunku liniowego, nie pełnej nieliniowej relaksacji w naczyniu.
Delegowano uogólnienie providerów boundary/fold/orientation; krok świata,
trial-state, residual, dobór komponentów oraz integracja UI pozostają do
wykonania. Nie traktować jądra singla jako zakończonej naprawy solo cewnika.

## Walidacja bez ponownego wyznaczania pasma — 2026-09-10

Wydzielono validateCoulombGeneralBandMatrix: sprawdza te same wartości
Float64 i zakresy Int32 bez wyznaczania nieużywanego layoutu. Redukcja
odcinków korzysta z walidatora; właściwe createCoulombBandLayout zachowuje
swoje wyznaczanie pasma. 89 testów przechodzi, także BandLU.
Próba 8 mm: 494 solve, zero niezbieżnych, średnia 29,47 FPS, backlog 18,97 s,
ostatni krok 188,3 ms. Topology 5,95 s, redukcja 10,20 s. Nadal brak
wymaganej wydajności. Przywrócono 32 mm i dotychczasowy URL.

Audyt delegowany pojedynczego narzędzia: nie wystarczy zmiana warunku
world.bodies.length. Aplikacja ma dwa ciała także bez overlap; containment
włącza się dopiero dla rzeczywistego wspólnego zakresu. Poprawne uogólnienie
powinno rozwiązywać komponenty grafu aktywnych kontaktów, z bodies:[body]
dla rozłącznego narzędzia, bez fikcyjnego outer. Assembly i providery
boundary/fold/orientation/trial-state mają obecnie założenie dwóch ciał.
Ponadto joint-active nadal stosuje tarcie ściany później w velocity pass;
nie twierdzić, że jego cała równowaga tarciowa jest już globalna.

## Usunięcie nadmiarowych pamięci WASM — 2026-09-10

Przechwycono pierwotny wyjątek 8 mm: WebAssembly.Memory could not allocate
memory w factorLocal. Każdy mały odcinek posiadał osobną pamięć WASM.
Zastąpiono to pojedynczą areną na workspace, z rozłącznymi widokami faktorów.
Arena rośnie geometrycznie. Zmiana podziału unieważnia wszystkie stare
fingerprinty przed ponownym użyciem storage; wcześniejsze rekonstrukcje
nadal posiadają własne dane. Adapter kroku zachowuje pierwszy wyjątek
w firstError, a log aplikacji używa go zamiast wtórnego błędu retry.
Nie zmieniono akceptacji ani sposobu konsumowania dt.

92 testy przechodzą, obejmując 128 małych odcinków, zmianę podziału,
wiele prawych stron i zachowanie pierwszego błędu. Próba UI 8 mm teraz
kończy się raportem: 499 solve, zero niezbieżnych, średnia 29,40 FPS,
backlog 19,07 s. Błąd alokacji w tej próbie usunięty; wydajność niezaliczona.
Faktory 0,175 s, odpowiedzi 0,985 s, topology 6,171 s. Krótsze odcinki
istotnie zmieniają rozkład pracy; teraz największy koszt to przygotowanie
licznych odcinków. Wybrane pola w axial-browser-shared-arena-2026-09-10.json.
Przywrócono ustawienie 32 mm i dotychczasowy URL po eksperymencie.

## Próba krótszych odcinków 8 mm — 2026-09-10

Tymczasowo zmieniono tylko sectionSpan z 32 na 8 mm. Próba UI zatrzymała
się przy wire 66,0 cm / catheter 0,4 cm. Licznik renderowania wskazywał
60 FPS, ale raport pozostał Running 35/35 s. Nie zaliczać jako wydajności.
Logi zawierały powtarzany błąd `World repeated preparation of a pending
timestep` z fixedStepTransaction.js:46 i world.advance:1336. Pierwotny
wyjątek nie znajdował się już w dostępnych 2000 ostatnich wpisach.

Przywrócono sectionSpan 32 mm i URL joint-active-coulomb. Wniosek:
krótszy podział ujawnił nieobsłużoną ścieżkę błędu; przed kolejnym
porównaniem należy przechwycić pierwszy wyjątek. Adapter zachowuje
przygotowane wejście, natomiast position-history world.advance po wyjątku
ponownie wywołuje beforeSubstep (nie ma _pendingSplitSubstep). Kolejny
błąd maskuje pierwotną przyczynę. Nie naprawiać przez ponowny posuw ani
pominięcie dt, bo krok mógł częściowo zmienić stan.

## Trwały graf macierzy odcinków — 2026-09-10

Workspace zachowuje wiersze Map i kolumny, aktualizując wartości gdy
rzeczywista struktura niezerowych wpisów pozostaje identyczna. Nie opiera
się tylko na szerokości pasma. Zmiana aktywności kontaktu przebudowuje graf,
a legalność podziału odcinków jest sprawdzana także przy ponownym użyciu.
Kolumny przechowują same indeksy, bez nieużywanych kopii wartości.
78 testów przechodzi, w tym dodanie/usunięcie wpisu oraz zmiana właściciela
odcinka i zachowanie wcześniejszej rekonstrukcji.

Próba UI: 433 solve, zero niezbieżnych, backlog 19,08 s, średnia 29,03 FPS,
ostatni krok 137,8 ms. Topologia 2,59 s, redukcja 11,91 s, odpowiedzi
3,13 s. Brak istotnej poprawy całości; wciąż nie spełnia celu.
Wybrane pola w `axial-browser-topology-cache-2026-09-10.json`.

## Zakresy trójkątnych faktorów — 2026-09-10

Po LU wyznaczany jest dokładny pierwszy niezerowy współczynnik L i ostatni
niezerowy współczynnik U każdego wiersza. Kernel solveDenseProfileLU pomija
zewnętrzne zera przy kolejnych prawych stronach. Zakresy wyznaczane po
pivotingu obejmują fill-in; żadnego progu obcinania. 77 testów przechodzi,
w tym wielokrotne obciążenia z wymuszonymi odległymi zamianami wierszy,
porównane z pełnym kernelem i niezależnym rozwiązaniem.

Próba UI: 430 solve, zero niezbieżnych, backlog 19,14 s, średnia 28,93 FPS,
ostatni krok 329,3 ms. Odpowiedzi 3,109 s, faktory 2,701 s, cała redukcja
12,096 s. Nie ma dowodu istotnej poprawy FPS. Raport wybranych pól:
`axial-browser-triangular-profile-2026-09-10.json`. Wciąż do wykonania
trwała struktura składania, ścieżka pojedynczego narzędzia i adaptacyjna
dyskretyzacja oraz docelowe pomiary głębokiego/maksymalnego wsunięcia.

## Szczegółowy profil redukcji — 2026-09-10

Dodano sumowane czasy topology/packing/factor/responses/reactions do raportu
UI, także przez agregację prób Newtona. Są to podzbiory sectionAssemblyMs,
nie dodatkowe koszty. Factor obejmuje też rozwiązanie lokalnego prawego boku.

Próba 35 s: 423 solve, zero niezbieżnych, backlog 19,24 s. Koszt redukcji
12,18 s: topology 2,74 s, packing 1,30 s, factor 2,63 s, responses 3,24 s,
reactions 1,97 s. Globalny boundarySolve 0,72 s. Dane zapisano w
`axial-browser-local-cost-profile-2026-09-10.json`.

Wniosek do następnej zmiany: nie optymalizować wyłącznie LU. Przebudowa
struktur Map/kolumn oraz powtarzane rozwiązania odpowiedzi mają porównywalny
koszt. Sprawdzić trwałą strukturę odcinków i rzadkie podstawianie dla
wielu prawych stron, zachowując pivoting i dokładną rekonstrukcję reakcji.
Cel 60 FPS nadal niespełniony; pomiar nie obejmuje głębokiego cewnika.

## Zerowe mnożniki lokalnego LU — 2026-09-10

Kernel factorDenseLU pomija teraz aktualizację ogona wiersza wyłącznie
przy dokładnie zerowym mnożniku eliminacji. Pivoting i progi niezmienione.
76 testów przechodzi. Mikropomiar 128-wierszowej macierzy trójdiagonalnej,
500 faktoryzacji: pięć próbek starego kernela 151–157 ms, nowego 7,8–8,6 ms;
faktory LU identyczne. To nie jest reprezentatywna gwarancja FPS.

Pomiar aplikacji 35 s: 426 rozwiązań, zero niezbieżnych, średnia 29,00 FPS,
backlog 19,0663 s, ostatni krok 152,2 ms. Składanie odcinków 12,090 s,
Newton 15,055 s. Cel wydajności nadal niezaliczony. Rzeczywisty układ
ma inne wypełnienie i dodatkowy koszt składania oraz wielu prawych stron;
mikroprzyspieszenie nie przeniosło się na istotną poprawę aplikacji.
Przywrócono URL joint-active-coulomb.

## Pomiar cache Schura i rzadszego składania odcinków — 2026-09-10

Próba cache iloczynów: 394 solve, zero niezbieżnych, średnia 28,79 FPS,
backlog 19,12 s, ostatni krok 177,4 ms. Następnie usunięto odczyty pustych
wpisów lokalnej macierzy (składanie po niezerowych wpisach) i mnożenia
zerowych współczynników reakcji. Nie zmieniono tolerancji ani równań.
76/76 testów przechodzi po tej zmianie.

Druga próba UI: 419 solve, zero niezbieżnych, średnia 28,96 FPS,
backlog 19,07 s, ostatni krok 242,8 ms. Łączne składanie odcinków 12,29 s
wobec 12,95 s pierwszej próby; liczby solve i trajektorie nie są identyczne,
więc nie traktować różnicy jako kontrolowanego pomiaru przyspieszenia.
Obie próby nie spełniają celu. Dane drugiej próby zapisano w
`axial-browser-sparse-section-probe-2026-09-10.json`.
Przeglądarkę przywrócono do joint-active-coulomb.

## Dokładne ponowne użycie reakcji odcinków — 2026-09-10

Lokalna faktoryzacja LU działa w Float64 w istniejącym kernelu WASM.
Workspace faktorów jest utrzymywany między rozwiązaniami; nieudana
faktoryzacja unieważnia poprzedni wpis przed zmianą pamięci.
Dodatkowo redukcja odcinków przechowuje iloczyny Schura dla niezmienionych
wierszy reakcji i niezmienionych rozwiązanych kolumn odpowiedzi.
Porównania są dokładne, bez progów przybliżenia. Klucze używają oryginalnych
indeksów, a nie bieżącej numeracji granic. Zmiana wejściowej lub wyjściowej
reakcji wymusza odpowiednie przeliczenie. Nowy prawy bok zawsze aktualizuje
obciążenie; wcześniejsze funkcje rekonstrukcji zachowują własne dane.

Weryfikacja: 76/76 testów SectionCondensation, AxialWorld,
AxialReducedSystem i CoulombNewtonSolver. Test zmiennych reakcji sprawdza
zarówno ponowne użycie, jak i unieważnienie oraz zgodność z pełnym układem.
Nie wykonano jeszcze pomiaru FPS dla cache iloczynów Schura; zmiana nie
stanowi dowodu osiągnięcia budżetu wydajności.

Potwierdzono również lukę integracji pojedynczego narzędzia:
`#jointCoupledConstraint` wymaga aktywnego containment obu ciał.
Bez niego globalny solver materiałowy Kirchhoffa przeplata się z osobnymi
projekcjami ścian. Ujednolicenie tej ścieżki pozostaje do wykonania.

## Wiążące doprecyzowanie użytkownika — 2026-09-09

Przywrócić działającą fizykę i kolizje poprzedniego solvera. Celem jest
rozpatrywanie cewnika i prowadnika jako jednego układu osiowego, z lokalnymi
właściwościami odcinków i mniejszą liczbą obliczeń. Podstawą dalszych zmian
jest istniejąca mechanika Kirchhoffa `position-history` i jej prawa fizyczne,
kontakt, tarcie oraz sposób wsuwania. Zachować niezależny posuw i obrót.
Zmieniać wspólną reprezentację i rozwiązywanie równań, eliminując podwójną
pracę na odcinku nakładania się narzędzi. Poniższe wcześniejsze propozycje
nie upoważniają do dalszej wymiany modelu fizycznego lub kolizji.

Korekta przywrócenia: `joint-two-channel` także zmienia mechanikę — włącza
split physical/bias i transakcyjną akceptację całego kroku. W UI odtworzono
zatrzymanie przy wire 32,6 cm / catheter 0,4 cm oraz 1,4–2,1 FPS. Odrzucony
krok zachowuje przygotowane wejście i uniemożliwia pobranie dalszego posuwu.
Nie traktować tego wariantu jako działającej wcześniejszej fizyki.

Aplikacja wybiera teraz `joint-active-coulomb`: wcześniejsze `position-history`
i wspólny blok kontaktów obu narzędzi. Domyślny URL oraz zakładki z
`composite-joint` i `joint-two-channel` wybierają tę ścieżkę. Testy split
pozostają dostępne przez jawne `experimentalSplitMotion=1` wraz z
`coupledSolver=joint-two-channel`. Nie pomijać certyfikacji w eksperymencie.
Przywrócenie mechaniki nie oznacza ukończenia docelowej redukcji obliczeń.

Weryfikacja w przeglądarce: próba 35 s na `joint-active-coulomb` przeszła
od pustych narzędzi do wire 66,0 cm / catheter 11,6 cm. Raport potwierdził
`position-history`, 1115 wspólnych solve, zero niezbieżnych wyników,
2068 wykonanych kroków i skończone stany obu narzędzi. Średnia 33,1 FPS
(końcowy odczyt 7,5 FPS) i backlog 17,88 s oznaczają, że wydajność nadal
nie spełnia celu. Próba potwierdza usunięcie blokady startu cewnika,
nie realizację 60 FPS. Późniejszy test ponownego ręcznego D miał timeout
automatyzacji, więc nie stanowi osobnego potwierdzenia wznowienia posuwu.
Testy przywróconej mechaniki: 28/28; build poprawny.

### Redukcja powtórnego montażu wspólnego układu

`kirchhoffActiveCondensedSolver` zachowuje współczynniki Schura podczas
rozszerzania aktywnego zbioru kontaktów w jednym zamrożonym solve. Nowa
geometria zawsze rozpoczyna pusty cache. W full200 liczba obliczonych par
spadła z 30 531 do 16 836; 13 695 współczynników wykorzystano ponownie
(44,9% mniej obliczanych par). Pozostają 35 faktoryzacji, dwie ekspansje
i niezmieniona kontrola oryginalnych równań (residual 6,17e-5). Regresje
obejmują wcześniejsze hashe bajtów sił/reszt, zmianę rozmiaru workspace
i dodanie kontaktu przed już obecnymi w kolejności osiowej. Nie jest to
jeszcze pomiar poprawy całego kroku ani realizacja docelowej dyskretyzacji.

Odrzucony eksperyment `newton-first`: próba pominięcia fixed-load seed
zmniejszała faktoryzacje 35→4 w jednym fixture100mm, lecz w full200
zwiększała je 35→51 przez nieudane próby i fallback. Browser35s: 33,106 FPS,
2065 kroków, backlog17,891s, wobec33,061FPS/2068kroków/17,883s dla seeded.
Nie wykazano poprawy runtime, dlatego eksperyment usunięto z aplikacji i
solvera. Dalsze zmiany mają redukować pracę wspólnego układu przy zachowaniu
dotychczasowej mechaniki, a nie zastępować ją fizyką split.

### Wspólna oś — luka implementacyjna i warunek redukcji

Audyt 2026-09-10: obecny `BundleRuntime` jest odwracalną zmianą bazy,
nie redukcją ruchomych DOF. `BundleDiscretization` nie jest używany przez
runtime. Lokalny `condenseBundleSection` nie eliminuje globalnych węzłów.
Nie przedstawiać tych modułów jako wdrożenia modelu jednej osi.

Docelowy opis pokrycia musi zachować wspólną oś, niezależny przesuw wzdłuż
niej i obroty oraz odtwarzany offset poprzeczny. Parowanie najbliższych
węzłów nie wystarczy do redukcji: trzeba dopasować przekroje osiowo przez
interpolację, uwzględniając ruch granicy cewnika. Redukowana mobilność
wynika z metryki masy `P (Pᵀ M P)⁻¹ Pᵀ`, nie z dowolnego skasowania
kolumn. Sama redukcja kolumn nie usuwa wierszy materiałowych.

Dodano `measureKirchhoffBundleMobility` oraz opcjonalny `mobilityAudit`
w istniejącym solverze. Sprawdza oryginalne `dq - W Jᵀ dλ`, osobno
w jednostkach pozycji i radianach, oraz wskazuje węzły/kierunki wymagające
wzbogacenia. Test wykazuje, że sztucznie związana para może spełniać
`rhs - alpha*dλ - J*dq = 0`, a mimo to mieć błąd mobilności 0,5.
Nowy certyfikat odrzuca taki przypadek. To warunek bezpiecznej redukcji,
nie wdrożona redukcja ani dowód zwiększenia FPS. Pozostają interpolacja
przekrojów, eliminacja wewnętrznych niewiadomych, adaptacja i integracja.

`kirchhoffAxialLayout` implementuje teraz mapę wspólnej osi prowadnika
i offsetu cewnika w interpolowanym przekroju. Nie łączy najbliższych węzłów:
mapuje według etykiet osiowych i rzeczywistego pokrycia. Węzły cewnika poza
pokryciem pozostają niezależne. Początkowy zakrzywiony offset i oryginalne
indeksy materiałowe są zachowane, podobnie jak osobne zmienne kątowe.
Adapter `kirchhoffAxialSystemLayout` wiąże opis z aktywnymi węzłami starego
solvera, używając jego materialnego okna posuwu (startNode, outerStartNode,
innerArcOffset) albo jawnych współrzędnych. Odmawia zgadywania dopasowania
z mediany przestrzennych kontaktów. `includeAxialLayout` udostępnia opis
w zmontowanym oryginalnym układzie; test potwierdza niezmienione A/RHS.
To gotowe mapowanie wejściowe do redukcji, nie jeszcze mniejszy solver.

Mapa ma także dokładny pullback sił `Pᵀf` i akcję mobilności
`P⁻¹ W P⁻ᵀ` dla pełnej odwracalnej reprezentacji (oś + wszystkie offsety).
Zachowuje pozadiagonalne sprzężenia między punktami cewnika opartymi na
wspólnych węzłach prowadnika. Test z natywnymi kolumnami J obu narzędzi
potwierdza rekonstrukcję oryginalnego Wf, również przy nierównych masach
i unieruchomieniu. Nie mylić tej pełnej metryki z mobilnością projekcji
po faktycznym usunięciu modów: kolejny etap wymaga eliminacji równań
wewnętrznych/offsetów i rekonstrukcji, nie ich wyzerowania.

### Profil kosztu zbieżnego rozwiązania odcinkowego

Rozszerzono cache o odpowiedzi Aee^-1 Aer. Ponowne użycie wymaga dokładnej
zgodności faktora i kolumny sprzężenia; zmiana Aer unieważnia odpowiedź,
a zmiana Are nadal przelicza reakcję Schura. Cache przechowuje oryginalne
indeksy, a wynik nową numerację granic. Testy zachowują własność starszych
rekonstrukcji i rozróżniają zmianę obu kierunków sprzężenia.

Pomiar35s:405/405solves zbieżne, średnio28.85FPS, backlog19.12s,
ostatni krok199.8ms. Zsumowane fazy: axialAssembly1796.5ms,
Newton15682.3ms, recovery120.5ms. Wewnątrz Newtona sectionAssembly13627.5ms,
boundaryPreparation187.5ms,boundarySolve224.6ms,sectionRecovery129.7ms.
Nie sumować faz wewnętrznych ponownie do Newtona. 302524odpowiedzi policzono,
297267użyto ponownie. Główne ograniczenie to lokalny montaż/faktoryzacja/
Schur w JS, nie globalny bandLU. Następny krok powinien przyspieszyć
właśnie ten lokalny etap. Dane: `reports/axial-browser-phase-profile-2026-09-10.json`.

### Poprawka nieaktywnego twardego kontaktu ściany

Trwały lastFailure selektora i worstScalarConstraint ustaliły źródło
residualu0.10356: dodatkowy wiersz wall prowadnika,node88,lambda3.38e-18,
lower0,alpha0,rhs−0.10351. Kontakt był rozdzielony, ale FB zostawiał
numeryczną śladową siłę dodatnią. Dokładny oryginalny KKT traktował ją jako
aktywną i żądał zerowego luzu. Dotychczasowe odzyskiwanie granicy natural-map
pomijało wiersze z zerową przekątną, typowe dla twardych kontaktów mieszanych.

Naprawa używa tej samej dodatniej skali projekcji co evaluate() także dla
zerowej/ujemnej przekątnej general operator. Siła trafia na dokładną granicę
z gałęzi natural-map, bez epsilon-cutoff; następnie sprawdzane są wszystkie
równania, odtworzone siły i stożki tarcia. Test izoluje zero-diagonalny kontakt
ze śladową siłą, bez wykonywania iteracji Newtona. 66 testów przechodzi.

Benchmark35s po poprawce: 397/397rozwiązań zbieżnych, końcowe closuretrue,
wire66cm/catheter4.9cm, maxpenetration0.0024mm. Błędy długości obu narzędzi
około5.4e-6. Jednak średnio28.79FPS/końcowo4.8FPS, backlog19.14s;
ostatni krok202.7ms,3closurepasses,16faktoryzacji. Globalnie381wierszy,
lokalnie1235. Następny etap to koszt montażu/faktoryzacji poprawnie zbieżnego
układu oraz głębokie wsunięcia. Ten wynik NIE zalicza wydajności ani pełnej
walidacji mechaniki. Dane w `reports/axial-browser-wall-recovery-2026-09-10.json`.
Po próbie karta przywrócona do joint-active-coulomb.

### Redukcja obu narzędzi i poprawianie reszty liniowej

Wariant aplikacji joint-axial-sections używa teraz sectionScope=all:
partycja obejmuje również wnętrze długiego prowadnika oraz jego równania.
Nie zmienia prawa materiałowego, siatki ani fizycznych kryteriów. W próbie
bez kontaktów (N16/32/64, spacing1, span8) globalny układ wyniósł24/72/168
wierszy, a lokalny195/387/771. Warianty z samym cewnikiem pozostają testowane.

Powtórny benchmark35s przed poprawianiem reszty nadal NIE przeszedł:
średnio26.49FPS, ostatni odczyt1.2FPS, wire66cm/catheter1.9cm,
1844kroki/15.367s fizyki, backlog20.15s; 33z44rozwiązań niezbieżne.
Ostatni krok574.9ms, Newton85iteracji/87faktoryzacji/57gradientfallbacks;
residual1.859, maksymalny backward error1.60e-10. Pole global/local w tej
próbie pozostawało inicjalne po odrzuceniu kierunku; poprawiono raportowanie,
aby pokazywało rozmiar rzeczywiście zmontowanego Schura również przy błędzie.
Po pomiarze przywrócono kartę joint-active-coulomb.

Następnie dodano do section-band-lu maksymalnie trzy poprawki rozwiązania
przez resztę pełnego układu liniowego. Te same lokalne faktory służą do
rozwiązania poprawki; kryterium64*n*EPS pozostaje niezmienione. Test
[[1e-12,1],[1,0]] z RHS[.3,.7] wymusza błąd rekonstrukcji, a poprawka go
usuwa. Próg akceptacji i oryginalny KKT nie są luzowane. 59 testów mechaniki,
rekonstrukcji i kroków świata przechodzi.

Benchmark z poprawianiem reszty: wire66cm/catheter4.3cm,
1900kroków=15.833s symulacji; średnio28.34FPS, końcowyodczyt3.8FPS,
backlog19.35s. 13z320rozwiązań niezbieżne (wcześniej33z44 przy mniejszym
wsunięciu); nie jest to porównanie identycznej zamrożonej macierzy.
Ostatni układ1591wierszy redukowany do370globalnych/1221lokalnych,
zero gradientfallbacks i zero linearResidualFailures. Mimo tego Newton
osiąga81iteracji/82faktoryzacje/830backtracks i residual0.10356,
przy materialResidual3.1e-6, contactResidual2.4e-17 i frictionResidual5.6e-17.
Następne dochodzenie powinno zidentyfikować niedomknięty dodatkowy więz,
a nie dalej zwiększać liczbę iteracji. Raport:
`reports/axial-browser-refinement-probe-2026-09-10.json`.
Karta została przywrócona do joint-active-coulomb; cel nadal niezaliczony.

### Próba adaptera w przeglądarce — niezaliczona

Jawny wariant `joint-axial-sections` (sectionSpan32mm) jest podłączony w
simulatorze, z tą samą ścieżką position-history. Domyślny wariant pozostaje
joint-active-coulomb. Raport selektora pokazuje axialReduction/global/local.

Próba Start35s w przeglądarce: wire66cm, catheterokoło2.5cm,
średnio26.99FPS, końcowookoło2FPS; wykonano1858kroków=15.483s fizyki,
backlog19.9s, bez opuszczonych kroków. 47z58rozwiązań nie zbiegło się.
Ostatni krok461.9ms; Newton112iteracji/113faktoryzacji/93gradientfallbacks,
statuscoulomb-line-search, residual1.657. Układ mieszany1508zmiennych,
globalnie1340/lokalnie168, wobec1130wierszy oryginalnego dualnego modelu.
Rzeczywiste półpasmo63; dotychczasowy parametr `band=1508` w raporcie
Newtona był metadanymi wywołania general-band, nie szerokością macierzy.
Penetracja0.875mm i niedomknięty coupled closure także naruszają wymagania.

Skrót odczytu DOM: `reports/axial-browser-probe-2026-09-10.json`.
Po próbie otwartą kartę przywrócono do joint-active-coulomb.
Ten wynik wymaga naprawy kondycjonowania pełnego układu i większej redukcji
niepokrytego prowadnika; lokalne zielone testy nie wystarczyły do wykazania
poprawności runtime. Nie przedstawiać wariantu jako gotowej optymalizacji.

### Adapter do istniejącego kroku fizyki

`solveKirchhoffAxialCoupledSystem` używa nowego kierunku osiowego przez
`assemblySolver` dotychczasowego `solveKirchhoffCoupledSystem`. Zachowane są
oryginalne wiersze kolizji, odtwarzanie korekt, kontrola KKT, aktywne wskazówki,
trust-region scale i apply. Grupy oraz granice tarcia wracają do oryginalnej
numeracji mnożników. Nie reinterpretujemy starego dualnego warm-startu jako
mieszanych współrzędnych pozycji/reakcji.

Testy 12 lokalnych wariantów porównują format wyniku adaptera z dotychczasowym
solverem, włącznie z materiałowymi i kontaktowymi mnożnikami oraz scale.
Nowy test świata wykonuje po trzy kroki dla pary pięciowęzłowej z luzem i z
kontaktem. Oba kierunki mają w tym porównaniu tolerance=1e-10; pozycje oraz
prędkości różnią się mniej niż1e-5, wszystkie wywołania nowego kierunku przy
kontakcie zbiegają się i oba narzędzia otrzymują reakcję. Przy pierwotnej
luźniejszej tolerancji świata różnica pierwszej pozycji wyniosła3.45e-5mm:
nie należy mylić tego z dowodem identyczności torów przy innych tolerancjach.

Rozszerzono próbę świata o obciążone tarcie osiowe i skrętne .015, ruch
prowadnika ±4mm/s i spin2rad/s przez trzy kroki. Referencja używa teraz
activeCondensation+simultaneousCoulomb, zgodnie z joint-active-coulomb.
Początkowo wystąpiły niedomknięte kroki i błąd oryginalnego KKT ~4e-10
pomimo akceptacji residualu mieszanego przy1e-10. Dodano bramkę
acceptCandidate do Newtona: kierunek musi spełnić także oryginalny KKT
po rekonstrukcji. Nie zmieniono tolerancji. Domyślny FB może przy braku
zbieżności ponowić rozwiązanie mapą projekcyjną; raport sumuje pracę obu
prób. Start projekcyjny był wolniejszy w tej próbie i nie jest domyślny.

Oba kierunki ruchu przechodzą. Pozycje i składowe kwaternionów porównuje
próg1e-5; prędkości są porównywane przez ich przemieszczenie w dt=1/120,
również przy progu1e-5mm (nie stosuje się wspólnego progu liczbowego do
różnych jednostek). Wszystkie wywołania nowego kierunku w tych kontaktowych
próbach zbiegają się. To nadal mała para pięciowęzłowa, nie anatomia.

Adapter jest podłączony do pełnego kroku w testach bez renderowania.
Domyślny interfejs nadal używa joint-active-coulomb. Powyższa próba
przeglądarkowa ujawniła porażkę już przy krótkim wsunięciu; głębokie i
maksymalne wsunięcie z adapterem nie zostały zaliczone.

### Lokalna redukcja wnętrza odcinków w Newtonie

Opcjonalne `sectionSpan` w `assembleKirchhoffAxialSystem` tworzy lokalne
bloki zmiennych cewnika i równań z jego udziałem. Wszystkie przekraczające
granice bloków sprzężenia pozostają globalne; partycja uwzględnia również
możliwe mieszanie wierszy tarcia z naciskiem. To nie jest adaptacyjny dobór
siatki: span jest na razie jawnym parametrem eksperymentu.

`solveCoulombNewton(...,{localSections})` eliminuje lokalne bloki bieżącego
Jacobiana przez LU z pivotowaniem. Globalnie rozwiązuje Schur granic, potem
odtwarza lokalne ruchy i mnożniki. Niesymetria tarcia i nieokreśloność
bloków mieszanych są zachowane. Przy osobliwym wnętrzu stosuje pełne
rozwiązanie pasmowe. Oryginalny KKT oraz próg błędu wstecznego pozostają
niezmienione. Lokalne faktory są teraz używane ponownie wyłącznie przy dokładnie
identycznym bloku macierzy (nie tolerancji bliskości). Zmiana współczynników
unieważnia faktor. Pamięć solvera granic jest ponownie używana dla zgodnych
zakresów pasma. Rekonstrukcja wcześniejszego wyniku pozostaje niezależna
od późniejszego montażu.

Testy adaptera obejmują teraz 36 kombinacji ścieżek gęstej/pasmowej/odcinkowej,
kontaktu, tarcia, offsetu i pinningów. Oddzielny test Newtona sprawdza
lokalne normalne/styczne reakcje podczas obciążania i odciążania.
Próba dwóch narzędzi 16/32/64 węzły, segmentLength=1, sectionSpan=8,
bez kontaktów, zmniejszyła globalny układ 219→144, 459→312, 939→648
(zero fallbacków). KKT pozostał poniżej 1e-10. Zimne czasy solve
15.8/27.9/22.9 ms nadal przekraczają budżet. Wcześniejsza próba skalowania
miała segmentLength=N; nie należy porównywać jej czasów bezpośrednio.

Pomiar po 5 rozgrzewkowych wywołaniach i 20 próbach dla N=64, bez kontaktów:
montaż pasmowy średnio11.8ms/P9523.4ms, solve2.9ms/P955.9ms;
ścieżka odcinkowa montaż16.1ms/P9529.7ms, solve12.3ms/P9522.1ms.
Profil CPU tej próby wskazał ~231ms w montażu sparse, ~146ms w GC,
~124ms w solveCoulombNewton i ~118ms w wyznaczaniu pasm (czasy sumaryczne
całego procesu, nie jednego kroku). Usunięto kopię map H oraz budowanie
map gradientów miękkich wierszy, których lokalny kontaktowy montaż nie
używa. Testy własności rekonstruowanych wyników nadal przechodzą.
Powtórne czasy wykazały duży rozrzut (np. P95 montażu pasmowego191.5ms),
więc nie potwierdzają przyspieszenia na podstawie tych dwóch przebiegów.

Redukcja rozmiaru nie dała w tej próbie przyspieszenia. Aktualny koszt
lokalnych map/Schura przewyższa oszczędność globalnej faktoryzacji;
nie wolno włączać tej implementacji jako rzekomej optymalizacji FPS.

To nadal etap pośredni: pozostaje globalna część równań dwóch narzędzi,
montaż lokalnych Jacobianów kontaktowych oraz koszt faktoryzacji. Nie jest
to jeszcze ukończony model jednej osi o lokalnych właściwościach, nie jest
włączony w aplikacji i nie dowodzi osiągnięcia 60 FPS.

### Lokalna eliminacja offsetu — implementacja referencyjna

`kirchhoffAxialCondensation` liczy dokładny Schur i rekonstrukcję lokalnego
bloku przez skalowany Cholesky, bez przesunięć diagonalnych. Adapter
`kirchhoffAxialReducedSystem` korzysta z **niezrealizowanej jeszcze**
natywnej assembly (przed solve), aby nie przejąć zmienionych końcowych
grup/bounds tarcia. Składa `H=M+Jsoftᵀ alpha⁻¹ Jsoft` w współrzędnych
osi i offsetu; zachowuje twarde wiersze materiałowe, kontakty i kąty.
Eliminuje ruchome translacyjne offsety cewnika w pokryciu. Wyeliminowane
zmienne odtwarza wraz z reakcjami kontaktów, a miękkie mnożniki odzyskuje
z oryginalnych równań materiałowych. Kontrola obejmuje oryginalny KKT,
mobilność, korekty i siły wcześniejszego rozwiązania.

W testach lokalnych: brak kontaktu, twardy kontakt normalny i tarcie
zależne od nacisku, przesunięcie osiowe 0/0,25 oraz różne pinningi.
Korekty i reakcje zgadzają się z poprzednim solverem; rekonstrukcja
pozostaje poprawna po kolejnej native assembly. Przypadek tarcia ma
alpha=.01 w testowych równaniach stycznych; nie jest dowodem obsługi
pełnych osobliwych zestawów twardego tarcia w anatomii.

Adapter ma teraz opcjonalny montaż `matrixFormat: general-band`, bez
alokowania gęstego H ani gęstej macierzy mieszanej. Współrzędne i jawne
wiersze są przeplatane według położenia osiowego. Przy dokładnie diagonalnym
H_ee offsety eliminują lokalne aktualizacje rzędu jeden, bez faktoryzacji.
Jeśli H_ee zawiera sprzężenia, ścieżka pasmowa zachowuje te zmienne w układzie,
zamiast pomijać sprzężenia. Certyfikat oryginalnych równań liczy J W Jᵀ
bez przechowywania kopii macierzy dualnej. `assembleKirchhoffAxialSystem` pobiera teraz wyłącznie oryginalne wiersze,
Jacobiany i mobilności (`jacobianOnly`), pomijając budowanie dualnego Grama
oraz dodatkowego common-relative bundle. Stary solver zachowuje domyślną
pełną assembly i odrzuca próbę solve bez macierzy.

24 testy adaptera porównują montaż gęsty/pasmowy z dotychczasowym solverem.
Dodatkowy test łańcucha 32/64/128 sekcji potwierdza liniowy wzrost pamięci
pasmowej dla lokalnych sprzężeń; nie jest pomiarem FPS ani pełnej anatomii.
Próba bez kontaktów dla 16/32/64 węzłów (po dwa narzędzia, offset .25)
dała stałe półpasmo 21, pamięć pasma 5171/11464/23171 elementów i zbieżność
przy oryginalnym KKT poniżej 1.2e-10. Pojedyncze zimne pomiary montażu
18.2/15.4/25.6 ms i solve 12.4/23.1/10.2 ms NIE spełniają budżetu;
nie stanowią miarodajnego benchmarku rozgrzanej aplikacji. W szczególności
nadal pozostają globalne jawne równania obu narzędzi i kontaktów. To etap
pośredni, nie ukończona lokalna mechanika pokrytego odcinka.

Wersja gęsta pozostaje punktem odniesienia. Nowa ścieżka NIE jest podłączona
do kroku aplikacji. Mniejsza liczba zmiennych niż w pełnym mieszanym
układzie nie oznacza mniejszej macierzy od wcześniejszego solvera dualnego.
Do realizacji celu pozostają integracja pasmowej ścieżki, dalsza lokalna
eliminacja równań wnętrza odcinków, adaptacja i pełne pomiary wsuwania.

### Eksperymenty algebraiczne (nie dowodzą zgodności fizyki split z wcześniejszą)

Pełny układ obu kanałów może być teraz montowany bez gęstego etapu pośredniego:
`condensation:'none', matrixStorage:'general-band'`. Przedziały wierszy
wynikają z oryginalnego pasma i kolejności osiowej. Test 40-węzłowych narzędzi
porównuje każdy współczynnik z dense, a test rozwiązania sprawdza osobno
obie reakcje oraz oryginalne `J*dq`. Połączony zestaw regresji: 31/31.
Do weryfikacji w aplikacji dostępny jest parametr URL
`coupledLinearSolver=axial-band` przy `joint-two-channel&experimentalSplitMotion=1`.
Parametr zmienia wyłącznie algebrę w obrębie eksperymentalnej fizyki split.
Domyślna aplikacja nie używa tej ścieżki.

`reports/two-channel-compact-storage-benchmark.json` zapisuje porównanie
tego samego zamrożonego układu 298 równań. Mediany siedmiu prób po rozgrzaniu:
dotychczasowa kondensacja 13,25 ms, pełny dense 135,96 ms, pełne pasmo 5,22 ms.
Wszystkie wyniki przechodzą oryginalny KKT przy niezmienionej tolerancji 2e-4.
Pasmo i dense dają ten sam maksymalny residual 1,08e-5. Pomiar kondensacji
obejmuje jej przygotowanie i odzyskanie reakcji; pełne operatory przygotowano
przed pomiarem. To przesłanka do pomiaru runtime, nie dowód 60 FPS.

Audyt potwierdza, że runtime już używa wspólnej bazy oraz dokładnej
kondensacji par równań materiałowych. W zapisanym przypadku 155 równań
natywnych daje 298 równań obu kanałów, redukowanych do 62 równań Schura.
Pozostała macierz jest jednak gęsta, a operator materiałowy i jego
odpowiedzi nadal generują istotny koszt. Sama zmiana nazwy lub bazy
współrzędnych nie rozwiązuje problemu wydajności.

Wprowadzono ponowne wykorzystanie areny i instancji Wasm kondensacji.
Wyniki macierzy, RHS i odzyskanych reakcji są bitowo zgodne z poprzednią
implementacją; testy własności danych i kolejnych rozwiązań przechodzą.
Zaszumiony mikrobenchmark nie potwierdził jeszcze przyspieszenia całej
kondensacji, więc nie traktować tej zmiany jako osiągnięcia budżetu czasu.

Solver Coulomba obsługuje teraz opcjonalnie dokładne niesymetryczne pasmo
dla operatora row-major. Kierunkowe sprzężenia i dowolnie małe niezerowe
współczynniki pozostają; nic nie jest symetryzowane ani obcinane. Test
60 równań z normalną zależnością limitu tarcia porównuje wynik z dense LU
i niezależnie sprawdza reszty oryginalnej macierzy. Ta ścieżka nie jest
domyślną strategią runtime. Kompaktowy montaż i zamrożony pomiar wykonano
(powyżej), ale nie walidują one całej mechaniki split w aplikacji.

`reports/restored-coupled-baseline.json` przechowuje pomiar syntetycznego
układu 201/197 węzłów i 479 kontaktów. Obejmuje tylko solve zamrożonego
układu, bez anatomii, wsuwania i renderowania; nie dowodzi FPS aplikacji.

Analiza z 2026-09-06. Wniosek: największy potencjał ma wspólny opis mechaniczny
odcinka cewnik–prowadnik, z osobnym przesuwem i skręceniem obu narzędzi oraz
kontrolowaną reprezentacją luzu. Dotychczasowe optymalizacje przyspieszyły
podproblemy, ale pozostawiły kosztowne naprzemienne uzgadnianie dwóch prętów.
Wspólny blok kontaktów normalnych nie jest jeszcze wspólnym solverem całej
mechaniki. Osiągnięcie 60 FPS i fizyki działającej w czasie rzeczywistym wymaga
pomiaru nowego prototypu; poniższy projekt nie jest obietnicą gotowej wydajności.

## 1. Jaki budżet faktycznie trzeba spełnić

Aplikacja liczy fizykę przy 120 Hz: jeden krok odpowiada 8,333 ms czasu
symulowanego. Przy 60 FPS trzeba średnio wykonać dwa kroki na klatkę.
Obecny harmonogram rezerwuje 3,5 ms na pozostałą pracę, więc orientacyjny
budżet obliczeniowy jednego kroku to `(16,667 − 3,5) / 2 = 6,58 ms`.
To nie jest gwarantowany termin wykonania w przeglądarce. Istniejący warunek
`physicsBudgetPass` jest ostrzejszy: średnia ≤4 ms i P95 ≤6 ms. Należy go zachować.

Ostatni pełny pomiar przeglądarkowy, już po optymalizacjach wspólnego bloku:

| Stan | Średni krok fizyki | P95 kroku | FPS |
| --- | ---: | ---: | ---: |
| Cewnik 20 cm, utrzymanie | 8,79 ms | 9,7 ms | 60,0 |
| Cewnik 40 cm, utrzymanie | 12,51 ms | 14,4 ms | 59,9 |
| Wsuwanie z 40 do 60 cm | 27,78 ms | 45,5 ms | 32,6 |
| Cewnik 60 cm, utrzymanie | 20,11 ms | 25,6 ms | 45,3 |

Źródło: [pomiar w przeglądarce](catheter-contact-block-browser.json).
Parametry: prowadnik 999,9 mm, Glidewire 10/4,55, Berenstein 25/5, relaksacja 1.
Był to jeden pełny przebieg na tym komputerze, a nie seria statystyczna.

**Wyświetlane 60 FPS nie oznaczało fizyki 120 Hz w czasie rzeczywistym.**
Zaległość symulacji wzrosła z 4,15 s przed wsuwaniem cewnika do 18,48 s po
fazie 40 cm. Cały scenariusz 71,275 s fizyki trwał 121,07 s; na końcu pozostawało
49,79 s zaległości. Nie pomijano kroków. Sam licznik FPS ukrywał część problemu.

Przy 60 cm potrzeba około 3–4,2-krotnego przyspieszenia średniego kroku,
żeby zejść do 6,58 ms; osiągnięcie obecnego kryterium 4 ms wymaga około
5–7-krotnego przyspieszenia. Duże skoki P95 wymagają osobnej kontroli.

## 2. Co nadal kosztuje

Wykonałem dodatkowy profil CPU na kopii bieżącego silnika, z rzeczywistą
anatomią i pełnymi fazami przygotowania oraz wsuwania do 60 cm. Liczniki
dodałem wyłącznie do kopii w katalogu tymczasowym. Profil Node obejmuje
osobno wsuwanie do 60 cm i 600 kroków utrzymania.

Ten profil służy identyfikacji kosztów. Jego odciski pozycji różnią się od
przeglądarki również przed wsuwaniem cewnika, więc nie jest identycznym
odtworzeniem jej trajektorii. Czasów Node nie używam jako nowych wyników FPS.
Metoda, odciski źródeł, liczniki i próbki widma są zapisane w
[danych analizy](unified-catheter-guidewire-analysis.json).

| Praca w jednym kroku, średnia | Wsuwanie do 60 cm | Utrzymanie 60 cm |
| --- | ---: | ---: |
| Czas fizyki w profilu Node | 24,67 ms | 18,23 ms |
| Dodatkowe zewnętrzne iteracje domknięcia | 11,65 | 5,03 |
| Wszystkie przebiegi bloku kontaktów | 17,65 | 11,24 |
| Faktoryzacje materiałowe obu prętów | 34,70 | 23,68 |
| Ponowne użycia faktoryzacji prętów | 2,59 | 0,37 |
| Faktoryzacje preconditionera kontaktów | 123,02 | 64,51 |
| Przetworzone rekordy kontaktów, ze wszystkimi powtórzeniami | 7214 | 5380 |

Na końcu pozostaje 201 aktywnych węzłów prowadnika, 197 cewnika i 479
rekordów kontaktów. Wszystkie kroki obu profilowanych faz osiągnęły obecne
kryteria zbieżności. Niska wydajność utrzymuje się także po ograniczeniu
liczby zewnętrznych iteracji do około pięciu.

Rozłączny podział próbek CPU, wsuwanie / utrzymanie:

- Wspólny blok kontaktów normalnych, łącznie z wywołaną przezeń odpowiedzią
  całych prętów: **33,8% / 32,3%**.
- Osobne rozwiązania materiałowe prętów: **23,5% / 23,0%**.
- Geometria i interpolacja kontaktów wewnątrz cewnika: **17,6% / 19,7%**.
- Pozostałe projekcje poszczególnych kontaktów: **8,0% / 7,4%**.
- Zapytania geometrii/kontaktu naczyń: **2,9% / 2,3%**.
- Pomiar zbieżności kontaktów: **0,4% / 2,2%**.
- Garbage collection: **0,3% / 0,3%**; pozostała praca około 13%.

W tym pomiarze głównym problemem jest obliczanie sprzężenia, a nie zwalnianie
pamięci. Podział Node nie mierzy renderowania GPU.

## 3. Dlaczego poprzednie zmiany nie wystarczyły

**Podział równań wciąż wymusza poprawianie jednego narzędzia po drugim.**
W `endovascularPhysicsWorld.js` pierwszy blok normalny każdego kroku używa
pełnej podatności mechanicznej obu prętów. Następne używają tańszej podatności
wynikającej z mas; pełna odpowiedź materiałowa wraca w zewnętrznej iteracji.
Tarcie osiowe, obwodowe, skrętne i naprawy kontaktu ze ścianą mają kolejne
etapy. Korekta kontaktu zmienia naprężenia pręta, a korekta pręta ponownie
zmienia kontakt. Pojedynczy blok jest wspólny, ale cały krok nadal jest
rozwiązywany naprzemiennie.

**Koszt pojedynczego przebiegu również jest za duży.** Przy 60 cm w pomiarze
przeglądarkowym samo odjęcie całego czasu dodatkowego domknięcia zostawiłoby
10,26 ms podczas wsuwania i 10,79 ms podczas utrzymania. To wciąż przekracza
budżet. Jest to tylko rachunek kosztu, a nie wykonalny wariant algorytmu:
domknięcie jest potrzebne do spełnienia równań.

**Długi odcinek wspólny powiela geometrię i niewiadome kontaktowe.** Cztery
próbki na komórkę prowadnika poprawiają ciągłą kontrolę światła, lecz są
również osobnymi niewiadomymi siły. W każdej iteracji wracają wyszukiwanie
punktów, interpolacja, budowanie macierzy i zmiany zbioru aktywnego.
Przesuw materiału oraz zmiana orientacji ograniczają użycie już policzonych
faktoryzacji.

Sam bezpośredni solver pręta nie usuwa tego problemu. Solver Deula i wsp.
wykorzystuje strukturę bez cykli. Dwa pręty połączone wieloma kontaktami
tworzą układ z cyklami; szybkie rozwiązanie każdego pręta osobno nie gwarantuje
szybkiej zbieżności sprzężenia. To moja interpretacja struktury obecnego
kodu w świetle [Direct Position-Based Solver for Stiff Rods](https://animation.rwth-aachen.de/publication/0557/).
Jednocześnie uporządkowany układ dwóch łańcuchów nadal ma lokalną strukturę
blokową — cykle nie oznaczają konieczności użycia gęstej macierzy.

Sprawdziłem też, czy problem można wyjaśnić wyłącznie osobliwością kontaktów.
Pełna lokalna macierz Grama 479 rekordów ma prawie zależne kierunki, ale
podmacierze kontaktów obciążonych w dwóch końcowych próbkach są pełnego rzędu:
140 i 114 kontaktów. Ich diagonalnie znormalizowane uwarunkowanie wynosi
około 1678 i 84. Nie daje to podstaw do stwierdzenia, że każdy aktualny zbiór
kontaktów jest osobliwy. Potwierdzonym problemem jest przede wszystkim
**liczba powtarzanych rozwiązań i koszt ich przygotowania**. Te dwie próbki
nie mierzą widma pełnego operatora uwzględniającego sztywność prętów.

## 4. Proponowany wspólny opis mechaniczny

Rekomenduję jeden układ równań z modelem odcinka współosiowego oraz osobnym
odcinkiem prowadnika wystającym z cewnika. W odcinku wspólnym stan powinien
zawierać:

- wspólną krzywą odniesienia i jej zginanie;
- osobne współrzędne materiałowe obu narzędzi, aktualizowane przy wsuwaniu
  i wycofywaniu;
- osobne pola obrotu i skręcenia prowadnika oraz cewnika;
- względne przesunięcie poprzeczne tam, gdzie luz ma znaczenie;
- reakcje normalne i stan tarcia, z których wynikają opory poślizgu i obrotu.

Na odcinku, na którym przybliżenie wspólnej krzywizny jest poprawne, momenty
zginające obu materiałów sumują się. Dla izotropowego zginania:

```text
EI_eff(s) = EI_w(s_w) + EI_c(s_c)

kappa0_eff(s) = [EI_w * kappa0_w_rot + EI_c * kappa0_c_rot] / EI_eff

M_total = EI_eff * (kappa - kappa0_eff)
```

Krzywizny własne trzeba wyrazić we wspólnym układzie po uwzględnieniu
aktualnych obrotów. Dla zginania anizotropowego stosuje się sumę macierzy
sztywności. Taka redukcja wynika z modelowania współosiowych elastycznych
rurek; literatura zachowuje także względne skręcenie wzdłuż długości.
[Dupont i wsp., Design and Control of Concentric-Tube Robots](https://www.bu.edu/biorobotics/publications/CTR_TRO.pdf).

Wzór określa prawo materiałowe; nie narzuca bezpośrednio kształtu. Aktualną
krzywiznę nadal wyznaczają obciążenia, kontakt z naczyniem i warunki brzegowe.
Przesuwanie końca cewnika zmienia zasięg wspólnego odcinka oraz to, które
fragmenty profili sztywności i krzywizny własnej się spotykają.

Nie należy dodawać wartości suwaków ani sumować samych compliance.
Trzeba składać energie i właściwe EI. Nie należy również narzucać obu
narzędziom jednego obrotu ani sumować GJ tak, jakby były trwale połączone.
Utrata względnego skręcenia usunęłaby poślizg obrotowy i zmieniła sterowanie
końcówką. Jeśli algebraicznie kondensujemy zginanie, trzeba zachować część
energii zależną od względnego obrotu i przesunięcia — samo wyznaczenie
uśrednionej krzywizny jej nie zastępuje.

Istnieje też praktyczny precedens wspólnej siatki dla narzędzi
wewnątrznaczyniowych: SOFA BeamAdapter ma przykład cewnika, prowadnika i
spirali z osobnymi interpolacjami materiałów na wspólnych stopniach swobody.
To wskazówka architektoniczna, nie dowód osiągalnych FPS w naszym silniku.
[Oficjalny przykład BeamAdapter](https://github.com/sofa-framework/BeamAdapter/blob/master/examples/3instruments_collis.scn).

### Luz i końcówka cewnika

Obecna średnica prowadnika wynosi 0,889 mm, a średnica wewnętrzna cewnika
0,97 mm. Daje to **0,0405 mm luzu promieniowego**. Dokładne utożsamienie obu
osi jest więc przybliżeniem, nawet gdy wizualnie różnica jest mała. Może
zmienić siłę normalną, próg tarcia i moment pojawienia się kontaktu.

Dlatego bezpieczną kolejnością jest najpierw wspólny solver zachowujący
ruch względny, a następnie sprawdzona redukcja liczby jego niewiadomych.
Pełniejszy opis dwóch osi powinien pozostać przy ujściu cewnika, zmianach
kontaktu, silnej krzywiźnie i lokalnym rozdzieleniu osi. Zakres tej strefy
powinien wynikać z estymacji błędu, a nie arbitralnej stałej długości.
Odsłonięty prowadnik pozostaje samodzielnym prętem, sprzęgniętym z całym
układem przez siłę i moment na granicy.

Zewnętrzny kontakt na odcinku wspólnym wynika z powierzchni cewnika.
Poza nim z powierzchni odsłoniętego narzędzia. Ruchoma granica musi zachować
równowagę sił i momentów oraz przenoszenie materiału. Nie należy maskować
zmiany sztywności sztucznym wygładzaniem, które zmienia fizykę.

## 5. Jak z tego zrobić szybszy solver

1. **Jeden wspólny krok mechaniczny.** Złożyć energie materiałowe,
   bezwładność, warunki długości i aktywne reakcje w jednym układzie
   linearyzowanym. Rozwiązywać wspólny kierunek korekty, z kontrolą kroku
   i zbieżności. Tarcie nadal wymaga warunków przyczepności/poślizgu;
   nie jest zwykłą gładką sprężyną.
2. **Uporządkowanie według położenia wzdłuż narzędzi.** Grupować współrzędne
   wspólne, względne i reakcje w lokalne bloki. Utrzymywać rzadką strukturę
   i ponownie wykorzystywać jej analizę symboliczną. Układ siodłowy z
   mnożnikami wymaga odpowiedniego LDLT/pivotingu lub kontrolowanej
   eliminacji więzów; dotychczasowej faktoryzacji SPD nie można bezpośrednio
   zastosować do dowolnego KKT.
3. **Redukcja dopiero po porównaniu fizyki.** Wspólne i względne współrzędne
   mogą początkowo być tylko zmianą zmiennych. Mniejszą liczbę względnych
   stopni swobody wprowadzać tam, gdzie błąd sił i kształtu jest akceptowalny.
   Lokalnie można kondensować wewnętrzne niewiadome elementów. Eliminacja
   całego pola względnego sprzężonego między elementami może zagęścić
   macierz, więc nie należy zakładać, że każda kondensacja jest tania.
4. **Siatka mechaniczna niezależna od detekcji kolizji.** Gęstsza przy
   końcówkach, zmianach materiału i kontaktu; rzadsza na gładkim trzonie.
   Zachować gęstą kontrolę powierzchni, interpolowaną z elementów belkowych.
   Liczba próbek sprawdzających kolizję nie musi równać się liczbie
   niezależnych niewiadomych siły, ale każda pominięta nierówność musi być
   nadal sprawdzana i w razie potrzeby aktywowana.
5. **WASM po wyborze struktury.** Obecnie mały kernel przyspiesza już
   faktoryzacje i rozwiązywanie układów. Przeniesienie większej części
   składania i rozwiązywania do WASM może ograniczyć koszt JS, ale samo nie
   usuwa powtarzanych iteracji. Worker może poprawić responsywność interfejsu;
   nie zapewni fizyki 120 Hz, jeśli nadal potrzebuje 20–28 ms na krok.

Pierwszy prototyp powinien porównać: obecny solver, wspólny solver z pełnym
ruchem względnym oraz wariant z redukcją. W ten sposób można oddzielić zysk
ze wspólnego rozwiązywania od zysku wynikającego ze zmiany modelu.
Warto zacząć od odcinka prostego i zakrzywionego z przesuwem oraz obrotem,
a dopiero potem podłączyć poruszające się ujście i anatomię.

## 6. Kryteria zakończenia przebudowy

- Zachowanie rzeczywistego czasu: 120 kroków na sekundę średnio, bez
  rosnącej zaległości i bez pomijania kroków; 60 FPS z raportowaniem P95/P99
  czasu klatki. Docelowo obecny budżet fizyki: średnia ≤4 ms, P95 ≤6 ms.
- Powtarzane pełne przebiegi w przeglądarce do 60 cm i do maksymalnego
  dopuszczalnego wsunięcia, ze wsuwaniem, wycofaniem, obrotem, nawrotem
  obrotu i utrzymaniem; różne typy cewnika oraz zakresy sztywności.
- Równowaga sił i momentów, prawidłowa sztywność wspólnego zginania,
  zachowany swobodny przesuw/obrót bez tarcia i prawidłowy próg tarcia przy
  obciążeniu. Odzyskiwanie kształtu własnego po odsłonięciu końcówki.
- Porównanie kształtu, sił, obrotu końcówki i pracy tarcia z dokładniejszym
  rozwiązaniem; badanie niezależności od siatki. Sam mały residual nowego,
  uproszczonego modelu nie dowodzi zgodności z modelem pełnym.
- Zachowanie dotychczasowych progów zbieżności kontaktu i długości;
  kontrola rzeczywistej penetracji niezależnie od residualu z compliance.
  Osobny znany przejściowy problem prowadnika przed nasunięciem cewnika
  pozostaje w zestawie regresji, nie może zostać ukryty przez ten projekt.

Parametry sztywności w `kirchhoffMaterialProfile.js` są obecnie skalą
strojenia symulatora, nie pomiarami EI w N·mm². Dla realizmu konkretnego
prowadnika hydrofilnego i cewnika potrzebna będzie kalibracja zginania,
skręcania i tarcia. Nowa reprezentacja może lepiej przenosić siły, lecz sama
zmiana solvera nie kalibruje materiałów.

Zakres tej pracy: analiza źródeł, dodatkowe profilowanie i projekt przebudowy.
Pliki wykonywalne aplikacji nie zostały w tej analizie zmienione.
