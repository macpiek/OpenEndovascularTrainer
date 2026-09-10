# Joint World bridge: integration stage, 8 September 2026

Current stage: [regular taut-length normal form](../composite-taut-length-normal-form/README.md), 864 composite tests passing. The previously rejected two-prescribed-node straight-root case now accepts consecutive joint steps, with original arclength checks, full endpoint reactions, independent rotations and atomic retry. Transverse normal reactions retain a separate world-vector history; the redundant axial multiplier has an explicit zero gauge. Exact endpoint-metric eligibility does not solve rounded moving app boundaries. No new performance or UI claim.

Previous stage: [exact reverse strain/elastic-density derivatives](../composite-continuous-reverse-tape/README.md), 859 composite tests passing. Paired synthetic complete joint steps improve median 39.967→13.255 ms and 187.953→47.805 ms with matching physical states, reactions and iteration counts. A straight two-prescribed-node case fails both versions; a separate witness confirms its nonregular continuous length constraint. No contact/anatomy/insertion/UI or realtime claim.

Previous stage: [continuous arclength constraints with frame elasticity and C2 inertia](../composite-continuous-joint-length/README.md), 855 composite tests passing. Separate curved tool lengths, full signed reaction tangents, prescribed support handling, two loaded steps and atomic retry are verified. Integrated length does not enforce pointwise material metric; its within-element variation is reported. Curved contact, finite surface history, error-controlled adaptation, cost and UI remain incomplete. No new timing claim.

Previous stage: [continuous frame elasticity and inertia in consecutive joint steps](../composite-continuous-joint-elasticity/README.md), 847 composite tests passing. Exact strain derivatives, wider position/spin assembly, independent loaded tools and atomic retry are verified. Shared arenas and batched strains avoid per-element derivative buffers; an algebra simplification reduces measured local batch medians about 9–11%. Cost remains outside the realtime budget. Chord lengths, curved contact, finite surface history, geometric interface continuity, adaptation and UI remain incomplete.


Previous stage: [continuous inertia and polynomial history in the full joint step](../composite-continuous-joint-inertia/README.md), 839 composite tests passing. Wider common/relative assembly, consecutive nonlinear steps, opposite material feeds and exact retry are verified. Elasticity and lengths still use the native discrete rod; continuous constitutive/length/contact/frame integration, app and FPS remain incomplete. No new timing claim.


Previous stage: [continuous position/material-velocity geometry, matching inertia and polynomial history](../composite-continuous-material-geometry/README.md), 824 composite tests passing. The explicit local inertia factory and own-label history preserve the new C2 field through opposite feeds and old boundaries. Continuous material directors, curved contact/force mapping and wider JointTimeStep assembly remain unfinished. This is not active in the app and makes no new timing claim.

Previous stage: [explicit initial physical material surface velocity](../composite-initial-material-surface-motion/README.md), 809 composite tests passing. Moving sources on a fixed initial material chart now reach two-branch wall friction with the original wall witness and own imported translational/angular rates. Both tools, second dt and atomic retry pass. Full feed/angular-history transport, runtime adaptation, UI and FPS remain incomplete. No new timing measurement is claimed.

Previous stage: [indexed joint coefficient scans and scatter](../composite-direction-coefficient-loops/README.md), 801 composite tests passing. After 60 warmup pairs, the 15-node/128-contact whole-step median is 22.759→20.879 ms with identical physical results and certificates. Short-warmup regressions are preserved in the report; the geometry-buffer experiment was reverted. Runtime adaptation, full feed/history transport, app lifecycle and FPS remain incomplete.

Previous stage: [bounded compiled contact supports and edge geometry reuse](../composite-contact-stencil-reuse/README.md), 800 composite tests passing. The 15-node/128-contact whole-step median is 25.928→23.214 ms and preparation 4.828→2.611 ms with identical physical results and certificates. Timing improvements are not universal. Runtime adaptation, full feed/history transport, app lifecycle and FPS remain incomplete.

Previous stage: [direct sparse original assembly/certification](../composite-direct-sparse-direction/README.md), 797 composite tests passing. The 15-node/128-contact whole-step median is 32.641→27.507 ms with identical state/reactions/certificates. Not every timing case improves. Runtime adaptation/history transfer, app lifecycle and FPS remain incomplete.

Previous stage: [independent mechanical grid/contact samples and exact zero-dual factor compression](../composite-mechanical-grid-contact-samples/README.md), 795 composite tests passing. A synthetic 15-node mechanical grid retains 128 contact samples with separately checked shape/reaction errors. Runtime adaptation/history transfer, direct sparse assembly, app lifecycle and FPS remain incomplete.

Previous optimization: [safeguarded Coulomb-cone backtracking](../composite-friction-cone-backtracking/README.md), 791 composite tests passing. Loaded 65-node whole-step median 127.860→41.042 ms, 58→9 evaluations. App integration, full mechanical reduction and real-time performance remain incomplete.

Previous optimization: [shared owned contact charts](../composite-contact-chart-sharing/README.md), 784 composite tests passing. App integration and real-time performance remain incomplete. Older stage evidence follows.

Latest constitutive integration: [static/kinetic wall friction](../composite-joint-static-kinetic/README.md), 779 composite tests passing. General moving-source/material-label transport, app lifecycle and FPS remain incomplete.

Earlier source integration: [actual World wall adapter](../composite-joint-world-wall-source/README.md), 768 composite tests passing. This adds physical range/radius coverage and small original-Aorta and feed-path checks; full app lifecycle/FPS remain incomplete.

Earlier update: [production globalization](../composite-joint-direction-stabilization/production.md) closes the initial-penetration witness below; 752 composite tests pass. This report preserves the earlier bridge-stage evidence. App/anatomy/FPS remain incomplete.

The production Joint timestep is now executable through `EndovascularPhysicsWorld.wholeStepSystem` and `createCompositeJointWorldAdapter`. This is an opt-in complete-step integration. The simulator and its solver selector still do not select it; `?coupledSolver=joint-two-channel` remains the earlier implementation.

## Delivered behavior

- World calls the whole-step system before old prediction, integration, damping or contact passes. Only acceptance consumes the funded dt and increments clocks. Numerical rejection retains the single prepared command and its system identity; direct retry does not prepare controls again.
- The bridge owns Float64 Joint state, material velocity history, independent material reference frames and unwrapped spins. Numeric preparation is copied once. Collision fields and validated, recursively frozen prepared surface-pose paths retain their original identities. A structurally cloned path is rejected because it lacks preparation provenance.
- Publication stages both bodies before writing either. A later publication failure restores all numeric buffers and previous complete-view descriptors. Original body nodes and selected edge quaternions remain rendering views; `body.jointStateView` retains every union node, own subedge orientation, unwrapped angle and one-sided material velocity. These public views never become the next physics state.
- The new fixed-union World importer preserves every original active coordinate and hinge, independent physical curves, explicit density and material support profiles. Its exact union supports differently spaced tools. It is an initialization layer, not a moving-mesh transfer or an importer of legacy contact forces.
- The wall friction manager now consumes prepared own surface-pose paths and all their physical derivative columns. The bridge preserves those handles across retry. Exact old endpoint labels select the old surface trace; subtracting a label origin and dividing again is no longer used to determine endpoint identity on this path.

## Validation in authoritative worktree

`npm run test:physics:composite`: **746/746 PASS**, 15.949 s (`full-suite.txt`). `npm run build`: PASS, 1.82 s (`build.txt`), with the existing bundle-size warning. These are mechanics/integration checks and do not measure browser FPS.

The hook has 12 tests. The bridge has 10 tests covering actual shared lumen Coulomb steps, publication rollback, one-time preparation, original-World coverage rejection, genuine pose-handle retention, and explicit rejection of an unresolved initial-penetration case. The importer has 32 tests. Two additional tests run actual World rods at spacings 5 and 4 through importer → bridge → production Joint step → complete publication for two dt, including opposite +4π/−6π lifts, a wire-only exposed tail, mutable-view corruption and exact numerical retry.

Seven catheter feed tests exercise the production Joint step and its actual World bridge with a declared straight entrance relation, positive catheter insertion (`dsDt=-0.3`), independent spin, free position/spin DOFs, two nodal wall pressures, all original capsule gaps, material history, late rollback, 5/50/500 numerical penalties and different dt. The final test runs two World steps at 120 Hz with a second-step rejection and unchanged accepted state before retry. This is a single-catheter analytic-wall fixture, not full catheter-over-wire insertion on anatomy.

The initial importer state and direct production outcomes agree exactly in the integration fixtures. Accepted force and torque certificates retain their original thresholds. A previous unrelated monolithic World/profile regression still expects bend limit 18 while the current unchanged profile/body produces 60; this stage's full-composite result does not claim all of `npm test` passed.

## Explicit remaining work and live evidence

The bridge currently rejects active original World vessel fields, sheath constraints, containments and external tool contacts until complete source mappings exist. Same field identity alone cannot prove surface/radius/friction coverage; the independent audit demonstrated an empty owner list accepting outside anatomy. That bypass is closed. Do not replace the current guard with a caller-provided boolean or an empty certificate.

Actual simulator input has wire static friction 0.006 and kinetic friction 0.002, assigned to `wallStaticFriction` and `wallKineticFriction`. `wallFriction` is only the kinetic alias. The current Joint `mu` pair describes tangent axes, not static/kinetic coefficients. Per-surface friction and its static/kinetic law, complete exposure and original vessel-field source mapping, sheath/portal/tip laws, material profile refresh under feed, active-range/mesh state transfer, continuous internal-hinge surface history, adaptive relative-mode reduction and app factory/selector integration remain necessary.

A specific convergence witness also remains open: `addPreparedWall(f,{height:.49})` in the bridge test starts wire y0=.341 with radius=.16, giving .011 mm initial penetration. It reaches `original-linear-equations` with nonconverged wall friction. The explicit negative regression proves complete rollback and retained dt, **not** physical success. `initial-penetration-failure.txt` preserves the original detailed failure. The successful immutable-path bridge fixture starts at the wall (height .501); it does not resolve this penetration case.

No full-anatomy 60 FPS / 120 Hz result, deep/max insertion acceptance, mean ≤4 ms or P95 ≤6 ms has been established. The full goal remains active. The three delegated workers stopped with usage-limit errors after leaving useful code/evidence; root inspected and integrated their completed files and reran the full composite checks.

## Initial penetration: bounded diagnostic follow-up

A private snapshot added only linear-proof logging and varied the iterative-refinement count. At the third nonlinear direction the original linear force residual is 1.603e−8 with one correction, above the unchanged 5e−10 limit; all individual constraint residuals already pass. The existing maximum of two corrections lowers force residual to 6.864e−9 but still rejects. An isolated experimental limit of eight corrections stalls at the same 6.864e−9 after nine solves. Increasing iteration limits therefore does not fix the underlying numerical issue and was not applied to production. Next investigation should inspect conditioning/rank and original-equation accumulation in this contact direction. The nonlinear force/friction certificate also remains nonconverged; it must not be accepted using a weaker linear threshold.

The three JSON diagnostics preserve that evidence. The initial four/eight requests against the unmodified 0..2 API limit were invalid-option probes, not convergence results; only `penetration-eight-corrections-experiment.json` uses the private extended limit. No physical tolerances, production Step source or production direction solver changed in this investigation.
