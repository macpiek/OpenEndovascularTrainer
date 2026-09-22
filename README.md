# Open Endovascular Trainer

Open Endovascular Trainer is a browser-based endovascular training prototype built with Three.js. It lets a user practice guidewire and pigtail catheter manipulation inside a modeled aorta, head-and-neck circulation, and bilateral upper- and lower-limb arterial tree, switch between debug and fluoroscopy-style views, move a virtual C-arm, and inject contrast to observe wash-in and washout.

> This project is a simulation prototype for education, research, and interaction design. It is not a medical device and must not be used for clinical decision-making.

## Current Features

- Real-time WebGL simulator with a full-screen Three.js scene.
- Imported aortic, cerebral, iliac, and bilateral upper- and lower-limb arterial anatomy in
  `res/Aorta_plain.stl`, plus the skeleton from `res/skeleton.obj`.
- Shared Kirchhoff model for the guidewire and catheter with independent material feed and spin, segment-length preservation, wall and lumen contact, Coulomb friction, and resistance feedback.
- Precompiled sparse signed-distance collision field backed by a MeshBVH validator; production startup does not generate the centerline or collision field.
- Pigtail and Berenstein catheter shapes rendered with instanced segments instead of rebuilding `TubeGeometry` every frame.
- Introducer sheath positioned in the iliac branch, with retraction limits that keep the wire inside the sheath.
- Fluoroscopy rendering mode with persistence, pulse rate, noise, scatter, collimation, bone visibility, edge enhancement, brightness, contrast, and auto exposure controls.
- Hybrid contrast injection model with conservative pulsatile 1D transport on
  the complete STL centerline tree and a local deterministic 3D plume at the
  selected sheath or catheter outlet. A cell-resolved lumen mesh fills the
  vessel cross-section continuously, while the unmixed outlet jet uses
  volume-conserving directional streaks.
- C-arm controls for LAO/RAO, CRA/CAU, roll, table-plane movement, height, readouts, and a miniature C-arm preview.
- Patient monitor and procedure readouts for inserted length, pigtail length, contrast dose, kV, mA, FPS, and memory use.
- Remotion composition for creating a short promotional video from the project assets.

## Quick Start

Install dependencies and run the local dev server:

```bash
npm install
npm run dev
```

Open the URL printed by Vite, usually:

```text
http://localhost:5173
```

Node.js 20 or newer is recommended for the Vite 7 toolchain used by this project.

## Controls

The simulator starts in fluoroscopy mode.

DSA preparation, frame acquisition and cine playback use completed contrast
simulation time. Slower or paused physics therefore slows or pauses acquisition
and cine as well. Live fluoroscopy may continue refreshing, but it cannot fill
the DSA archive with repeated simulation instants. The 120-frame memory limit
remains (about 8 simulated seconds at 15 pulses/s); gallery times are simulation
seconds. Cine speed multiplies this simulation clock.

| Action | Control |
| --- | --- |
| Advance guidewire | `W` or `ArrowUp` |
| Retract guidewire | `S` or `ArrowDown` |
| Advance pigtail catheter | `D` or the `Advance` button |
| Withdraw pigtail catheter | `A` or the `Withdraw` button |
| Rotate pigtail catheter left/right | `Q` / `E` or the rotate buttons |
| Inject contrast through the selected sheath/catheter source | `I`, `C`, or the `Inject` button |
| Stop active injection | `Stop Injection` |
| Toggle debug/fluoroscopy view | `Debug` / `Fluoroscopy` button |

The Injection panel exposes an explicit sheath/catheter source, volume, and
rate. Duration is calculated from volume/rate. The other panels expose runtime
sliders for guidewire stiffness, smoothing, wall friction, image quality,
contrast display, and C-arm position.

Use `npm run test:contrast` for the quantitative hybrid-model checks. The
clinician review scenarios and acceptance scorecard are in
[`reports/contrast-clinical-validation.md`](reports/contrast-clinical-validation.md).

### Calibrated pigtail

In **Tool selection → Catheter**, choose **Pigtail skalowany · 1 cm**.
The existing loop-base band is the zero reference; additional 1 mm-wide
radiopaque bands mark the shaft every 10 mm proximally. They appear dark in
fluoroscopy and gold in debug view. Spacing follows the catheter's material
coordinates through bending and adaptive remeshing, rather than screen distance.
Each femoral access keeps its own selection. Mechanics and contrast side ports
are identical to the ordinary pigtail; withdraw to 0 cm before changing devices.

Run `node --test tests/catheterGraduationMarkers.test.js` for spacing, remeshing,
selection, and injection-port regression checks.

## Vessel Geometry

### Mesh repair and validation

Both shipped arterial meshes have a conservative seam repair recorded in
`res/Aorta_plain.mesh-repair.json` and
`res/Aorta_infrarenal_aneurysm.mesh-repair.json`. It removes zero-area and duplicate
triangles, welds compatible boundary vertices within 0.0001 source units,
subdivides unmatched T-junctions, and corrects orientable patches. It does not
cap anatomical branches or Boolean-union the overlapping outlet plugs.
Residual ambiguous junctions are reported; the mesh is not claimed to be a
watertight manifold. The existing outlet and lumen-clearance checks remain in use.

Run `npm run test:anatomy:mesh` to check the shipped STL/collision hashes, triangle
validity, closed outlets, subclavian clearance, and aneurysm geometry.
`scripts/anatomy/repair-vessel-mesh.py INPUT.stl OUTPUT.stl REPORT.json` requires
Python with NumPy and refuses to overwrite its input. Its regression checks are
in `tests/vesselMeshRepair.test.py`. `compareVesselSurfaces.mjs` independently
samples surface deviation. Collision builds accept the repair report as the
fourth positional argument (after the output report path), preserving branch identities while
recomputing the lumen and distance fields. Keep the original collision asset
beside the source STL until that build finishes.

### Stentgraft interaction

In **Tool selection → Catheter → Stentgraft…**, a modal selects an Endurant II-inspired
bifurcated body or a contralateral limb. Main bodies are selected by illustrated
cards: IIs 103 mm or II 124/145/166 mm, followed by proximal and ipsilateral
outlet diameters. Available combinations follow the manufacturer sizing table
(e.g. no 36 mm × 124 mm body). The same catalogue profile drives packed and
released geometry and cover travel; the short contralateral gate no longer
inherits its crotch position from the anatomical aortic bifurcation. Withdraw the
catheter before selecting it. The device can be selected before advancing the
guidewire, but insertion requires guidewire support. Use the existing catheter
buttons, **D / A**, or automatic withdrawal to move the delivery system along
the active sheath's guidewire. Release the control to stop. The tapered nose marks the leading end; the cyan band marks the moving sheath edge. Selections are independent per sheath.

During partial deployment, the catheter buttons and **D / A** still move the
whole attached assembly along its reference path; **Q / E** rotates it. These
commands can be held together with **J / K** cover motion. Starting cover motion
does not clear a held translation or rotation command.
The packed/deploying graft follows accepted mechanical rotation until suprarenal
capture is fully released, even if the cover has only partly retracted. Subsequent
translation and rotation affect only the delivery device; graft placement and
orientation stay fixed. Cover exposure is then measured relative to that fixed
implant, so withdrawing the delivery device continues to uncover it. An eccentric nose-base marker shows delivery rotation. Partially
released contact faces are bound to fabric triangles, avoiding another Boolean
union on each rotation step; prior solver snapshots remain immutable.

After positioning, hold **Zsuń koszulkę / J** or **Nasuń koszulkę / K** to move the
outer cover in either direction at 12 mm/s of committed physics time.
Hold **Uwolnij mocowanie / L** independently to release tip capture.
The suprarenal peaks remain captured throughout latch travel and snap to their
wall-constrained expanded shape when capture reaches full release; stopping
halfway does not leave a half-expanded crown.
Releasing the button or key immediately pauses its command;
an already prepared solver step consumes its captured input when it commits.
The outer cover is transparent in debug and radiolucent in X-ray; only its distal
marker is drawn. The covered graft stays compressed on the delivery axis, and only sections
passed by the cover edge expand. There is no stop at the contralateral gate.
Before full detachment, advancing the cover refolds the covered sections and
updates their mechanical contacts. Cover movement and capture release can be
combined, with opposite cover commands cancelling each other. Once fully
uncovered and detached, the graft remains implanted; the cover can still move
but cannot recapture it or undo its contrast seal.
The separate limb uses sheath withdrawal without a suprarenal capture stage.
After releasing tip capture, hold **N** or **Ściągnij nosecone do koszulki** to pull the
nose and its visible inner shaft back at 12 mm/s of committed simulation time.
Its base stops at the current distal cover marker, without moving the implant
or the cover. This moves the flexible nose geometry, not a separate elastic rod.
Changing access, losing focus, or cancelling a touch cancels the held command.
The renderer includes M-profile nitinol struts, an uncovered suprarenal crown,
radiopaque end markers, an e-shaped orientation marker, shaft and a 60 mm
tapered nose. The nose bends along the simulated guidewire, with tangent
continuation beyond its tip; it has no independent elastic rod. Body struts
have an illustrative 0.045 mm radius and crown struts 0.055 mm. Continuous
round-ended line segments follow the fabric circumference. A one-pixel sampling footprint prevents subpixel wire dropouts, with opacity
scaled by actual projected wire diameter rather than an opaque minimum thickness.
This applies to both debug and the single metal projection pass. Radiopaque
markers are 2.2 mm longitudinal strips and render after the shaft/scaffold so
they remain distinct on the packed system. The short gate retains its lateral
offset while both anatomical routes share the aorta; the offset decreases only
when the iliac routes diverge. The packed fabric, metal
struts, crown and markers are visible through the transparent cover while
advancing the loaded system; this preview does not create an implant or
contact surface and is replaced when deployment starts.
In the aortoiliac lumen, the released proximal rim follows the vessel axis even
when the delivery wire lies near a side wall. The bare crown follows a curved
continuation of that axis; its subdivided struts and anchoring details are fitted
to the lumen with clearance throughout tip release. This is a geometric wall
fit, not a radial-force or elastic apposition solver. Release uses the current delivery position and is not restricted
to the renal landing zone. A limb can also be released away from the body, in
which case it remains unconnected and does not seal the aneurysm flow model.
Nominal body diameter is adjustable from 20–36 mm. Withdraw and
remove the delivery system; the implant retains its deployed pose. Switch to the
opposite sheath, place its wire through the open cyan gate, load the separate
limb (10–20 mm nominal diameter, 80 mm long), position its nose with 10 mm overlap,
and deploy. The free gate becomes connected and cannot accept another limb.
The same workflow works with either side as the main access, in baseline anatomy
and in the aneurysm preset. Implant struts and delivery markers appear in
fluoroscopy/DSA; debug additionally shows the fabric.

The delivery system occupies the catheter material in the shared-axis Kirchhoff
solver: a straight, stiff rod mechanically braces the guidewire and participates
in vessel contact. Its covered bending rigidity is 8,000,000 simulator units;
the exposed inner shaft uses 400,000, with a 3 mm transition at the cover edge.
These are tunable presets, not manufacturer measurements. The 3 mm collision
radius uses a matching 3.2 mm introducer clearance. Insertion/withdrawal and
public device position commit with the coupled rod step, including rollback.

Release timing remains prescribed by the cover. The expanded target uses a reduced
self-expansion approximation: nominal circular sections, a relaxed vessel-guided
axis, parallel-transport frames, and coupled circumferential/axial radial springs
constrained by wall clearance. A free section returns to its nominal diameter;
it does not enlarge to fill the aneurysm. Wall contact deforms neighbouring
sections smoothly. Rotation moves whole material sections and rebuilds from the
nominal shape, avoiding accumulated dents or shearing from independent vertex
projections. The bare crown uses the same coupled shape fitting. These weights
are simulation parameters, not calibrated nitinol/material measurements.
Fully exposed fabric sections publish an
immutable contact surface during release, before the component is fully deployed.
The compressed section and 2 mm opening transition remain represented by the
delivery rod. Exterior faces are selected from the component union, avoiding
artificial membranes between its trunk and limbs. Contact snapshots advance
only when another face opens; covered geometry follows the committed wire.
For the owning access, released sections also apply a one-sided lumen constraint
using conservative inscribed ring radii. This recovery guide does not replace
contact with the actual fabric once the incoming tool is inside the guide. This pulls an eccentric wire inside an
incoming graft instead of leaving it on the exterior side of a thin sheet.
It adds transverse forces only; the opposite access retains two-sided contact.
These material-coordinate sections are anchored at implantation, independently
of later delivery withdrawal, and are included in rejected-step replays.
Both guidewires (and exposed catheters) have elastic, frictionless contact with
either side of this surface. The normal response uses a stiff penalty plus a
logarithmic barrier near the fabric centre surface, with consistent energy,
gradient and tangent. It supports finite wire/catheter radii before the crossing
guard triggers; limited compliant overlap remains possible. Open contralateral
ports permit wire cannulation and a wider catheter can follow over the wire,
contacting and sliding along the rim. Continuous crossing checks remain enabled.
The contact coefficients are simulator parameters, not measured fabric properties. Open ends stay open; overlapping components have no internal membrane.
Contact energy, forces and tangents participate in Newton equilibrium, with
sweep checks to reject new crossings. Existing overlaps at kinematic release
relax elastically, allowing already present tools to be withdrawn. A surface revision wakes sleeping solvers; an in-flight
cooperative step finishes against its original surface snapshot.

After the body and contralateral limb are fully released and connected, the
contrast model assumes ideal proximal/distal sealing. Its finite-volume network
uses graft calibre, excludes covered side branches, and keeps both distal iliac
outlets connected. Local jets and the rendered lumen are confined by the same
fabric surface. Iodine already excluded at sealing is retained in a trapped
compartment and the pre-existing vascular signal remains visible; newly injected
iodine cannot refill it. An open contralateral gate keeps the unsealed model.
The visual design is inspired by the
[Endurant II/IIs manufacturer IFU, §§1.1–1.2 and 11.2.3–11.2.7](https://www.accessdata.fda.gov/cdrh_docs/pdf10/P100021S063D.pdf).
This is a kinematic approximation, not an exact product CAD or validated device
mechanics model. Covered lengths and diameters use the
[Medtronic sizing sheet](https://www.medtronic.com/content/dam/medtronic-wide/public/western-europe/products/cardiac-vascular/cardiovascular/aortic-stent-grafts/endurant-ii-sizing-sheet-print-en-gb.pdf);
crotch geometry, gate length, ring spacing and marker placement remain approximate.
The existing two-component ideal-seal contrast model is retained; selecting the
IIs body does not add a separate ipsilateral extension workflow. Independent reversible cover controls
are a simulation feature, not a reproduction of the device release interlocks. Partial
release has fabric contact but does not remodel flow until components are fully
released and connected. The bare suprarenal struts have no independent rod-contact
model, and radial deployment is not a bidirectional elastic shell simulation. Endoleaks, collateral flow and fabric elasticity are not modelled.

Run `npm run test:stentgraft` for both-access deployment, docking, wall containment,
simulation-clock and implant-persistence checks against both anatomy assets.

### Anatomy variants

The **Anatomy** tab offers the baseline model and a generated **infrarenal
aortic aneurysm** preset. Choose the variant, then apply it. Applying anatomy
reloads the scene and resets the guidewire and catheter in **both** femoral
sheaths; solver settings in the URL are retained. The baseline remains available.

The aneurysm preset is a lumen enlargement without mural thrombus,
measuring approximately 50 mm at the reference transverse section (Y = -230 mm).
It begins below the renal origins and remains enlarged down to the iliac
bifurcation, without a distal aortic neck. A smooth 20 mm transition below the
bifurcation blends the enlargement into the two iliac limbs.
The sac preferentially expands anteriorly. Deformation is restricted to the
connected infrarenal surface and iliac transition inside the longitudinal slab;
neighboring mesenteric branches retain their original vertices and centerlines.
Renal origins, the iliac bifurcation center, access sites and skeleton
retain their baseline coordinates. The classification follows the
[University of Michigan AAA diagram](https://www.med.umich.edu/1libr/Surgery/VascularSurgery/Illustrations/TypesofAAA.pdf).

Each variant has its own matching STL and precompiled collision/centerline
asset. Both tool solvers and contrast use the selected anatomy. The preset
loads with `?anatomy=infrarenal-aneurysm&panel=anatomy`; the expensive geometry
and collision preparation runs offline, not during a physics step.

Rebuild the preset with `npm run anatomy:aneurysm`; generation and collision
validation run in a temporary directory before publishing the matching assets,
so a failed or unfinished build keeps the previous model available. Verify it with
`npm run test:anatomy:aneurysm`. The generator pins baseline landmarks to a
source SHA and rejects a different baseline until those landmarks are updated.
Dimensions, source identity and deformation parameters are recorded in
`res/Aorta_infrarenal_aneurysm.json`.

The vessel centerline metadata is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. A short introducer sheath extends from the distal left branch with a 30 degree tilt against the vessel wall toward +Z.

The visible vessel, tool-wall contacts, and contrast-flow tree are driven by
the imported STL aorta and `res/Aorta_plain.collision.bin`. The procedural
vessel data is retained only for control and tool-path metadata.

The STL continues both external iliac arteries through the common and
superficial femoral, deep femoral, popliteal, anterior tibial, posterior
tibial, tibioperoneal trunk, fibular, dorsalis pedis, medial and lateral
plantar, deep plantar, and transverse plantar arch arteries. Smaller branches
include the circumflex and perforating femoral, genicular, and anterior tibial
recurrent arteries. Four plantar metatarsal branches divide into proper plantar
digital arteries reaching all five toes. These are hollow, tapered vessel walls
joined to the original lumen rather than overlaid visual tubes. The
repeatable `npm run anatomy:extend-lower-limbs` generator starts from the
original STL and rebuilds this Boolean extension before the collision asset is
regenerated.

Both subclavian endpoints continue through the axillary and brachial arteries.
Each upper limb includes the deep brachial, radial, ulnar, common/anterior
interosseous, superficial and deep palmar arch, and four common palmar digital
branches. It also includes the thoracoacromial, lateral thoracic,
subscapular/thoracodorsal, circumflex scapular and humeral, collateral,
recurrent, and posterior interosseous branches. The common digital branches
divide into proper digital arteries, with separate princeps pollicis and thumb
digital arteries. Their paths are aligned anterior-medial to the humerus, on the
appropriate sides of the paired forearm bones, and anterior to the hand
skeleton. They share the same hollow Boolean wall, collision field, and
contrast-flow tree as the pre-existing vessels.

The shipped subclavian arteries have a subsequent bilateral correction relative
to the loaded clavicles and first ribs. Their arches turn medially and posteriorly
before descending toward the axillae. The rendered STL and collision asset share
this correction. This is a model-based anatomical approximation; a single AP
angiogram cannot determine a patient-specific 3D course.

The reproducible offline correction is `scripts/align-subclavian-arteries.mjs`.
It requires Python with NumPy (`PYTHON` selects the interpreter) and accepts
distinct input/output resource directories. It checks the input STL hash against
`scripts/anatomy/subclavian-landmarks.json`; the source revision is recorded there.
It locally subdivides the surface, transports both subclavian regions and outlet
landmarks, and reconstructs hollow extension junctions and small shoulder branches.
After this step, regenerate `Aorta_plain.collision.bin` with `npm run collision:build`
in the prepared project, and install the STL, collision binary and both JSON
manifests together. `npm run test:anatomy` and `node --test tests/aortaOutletClosures.test.js` check
the shared assets, the subclavian clearance and the preserved terminal walls.
Running the older `anatomy:rebuild` generator alone restores the earlier course;
it does not apply this later correction or the outlet closures.

At the unchanged superior endpoints of the source model, the STL also
continues both common carotid and vertebral arteries. It includes the external
and internal carotids, vertebrobasilar system, complete Circle of Willis, and
distal ACA, MCA, and PCA branches. The extensions follow the original terminal
centerlines while the pre-existing aorta and lower-limb geometry retain their
original positions.

The STL centerline is extracted offline as one acyclic medial tree. Each lumen cross-section is thinned to a topological medial axis, the resulting 3D graph is reduced with a clearance-weighted TEASAR pass, and every final edge is checked against the STL wall BVH. The centerline is used only for broad-phase lookup and branch identity; it never pulls a simulated tool toward the vessel axis.

## Development Scripts

Pełna dokumentacja architektury i indeks funkcji są dostępne jako statyczna
strona w [`docs/index.html`](docs/index.html). Indeks można odświeżyć poleceniem
`npm run docs:generate`; workflow aktualizuje go także automatycznie po merge.

```bash
npm run dev          # start Vite development server
npm run build        # build the browser app
npm run anatomy:rebuild # rebuild the hollow limb, neck, and cerebral arteries
npm run collision:build # regenerate the versioned centerline and sparse SDF asset
npm run benchmark:collision # write direct Kirchhoff/contact timing reports to reports/
npm run benchmark:browser:chrome # run the foreground Chrome acceptance workload
npm run benchmark:browser:safari # run the same workload through Safari WebDriver
npm run centerline:diagnostics # export centerline metrics and orthogonal projections
npm run test:contrast # run hybrid contrast conservation/performance tests
npm run preview      # preview the production build
npm test             # run simulator syntax checks and regression tests
npm run video:studio # open the Remotion studio
npm run video:still  # render a still frame to out/endovascular-trainer-frame.png
npm run video:render # render the promotional video to out/endovascular-trainer-ad.mp4
```

For a simple static server without Vite, the helper script can serve the repository root:

```bash
./scripts/run-browser.sh
```

## Project Layout

```text
index.html                  Main simulator shell and controls
style.css                   Simulator UI styling
src/simulator.js            Main scene, physics loop, rendering passes, and integration
src/physics/endovascularPhysicsWorld.js Shared XPBD rod/contact world
src/physics/collision/         Packed collision asset and VesselContactField
src/physics/rodState.js        Material-node storage shared with rendering
src/physics/guidewireTransport.js Prescribed inlet feed and contact diagnostics
src/pigtailCatheter.js      Pigtail catheter behavior and mesh generation
src/contrast/               Hybrid 1D/3D contrast transport and volume renderer
src/vesselGeometry.js       Vessel centerline, sheath, flow, and branch metadata
src/aortaModel.js           STL loading and vessel collision setup
src/aortaPreprocess.js      Offline/source lumen preprocessing helpers
src/lowerLimbArteries.js    Bilateral lower-limb path and attachment definitions
src/upperLimbArteries.js    Bilateral upper-limb path and attachment definitions
src/headArteries.js         Carotid, vertebrobasilar, and cerebral path definitions
src/boneModel.js            Skeleton asset loading
src/carmControls.js         C-arm movement controls
src/ui/                    UI widgets, monitor, and C-arm preview
res/                        Aorta STL and skeleton OBJ assets
tests/                      Physics and solver regression tests
video/                      Remotion video composition
out/                        Generated preview frame and video
```

## Collision And Physics

The default application uses `shared-axis` with a reference 5 mm mechanical grid.
The **Debug → Solver fizyki** selector also offers the experimental
`shared-axis-adaptive` variant and older solvers. Apply the selection with
**Zmień solver i zresetuj scenę**; switching reloads the scene. The adaptive
variant merges quiet shaft regions up to 20 mm, preserves fine tip/curvature/contact
regions, and uses a 0.15 mm local shape simplification threshold. This threshold
is not a bound on accumulated trajectory error. It also relaxes the nonlinear
force/torque residual to `1e-4` and geometric residual to `1e-3`; the reference
retains `1e-6` / `1e-5`. Actual tool endpoints can create shorter cells.
In the debug view, **Węzły pręta** draws accepted mechanical nodes (cyan wire,
orange catheter); the panel reports the shared node count, DOFs and segment lengths.
The experimental mode is also available at `?coupledSolver=shared-axis-adaptive`.

Debug also offers **Newton: ponowne użycie macierzy i faktoryzacji** for both
shared-axis grids. It is opt-in (`modifiedNewton=1`), applied with the solver
restart button. The experiment retains the full KKT matrix and pivoted LU for
at most two additional directions after a small, productive Newton step.
Changed active contacts/supports, poor progress, rejected directions or newly
discovered contacts refresh the tangent. Current nonlinear residuals and
contact/friction acceptance remain mandatory; a failed experimental timestep
retries the original method. The Debug counter reports accepted frozen-matrix
attempts and full-matrix fallbacks. This is an approximation, not exact reuse
of a current Jacobian, and its speed benefit depends on the motion.
See `reports/modified-newton-2026-09-17/` for comparisons and limitations.

Debug additionally offers **Projective Dynamics — pręt podatny, siatka adaptacyjna**
(`?coupledSolver=shared-axis-projective&solverDebug=1`). This opt-in experiment
uses local SO(3)/length/contact projections and a reusable SPD band Cholesky
factor, with 16 local/global iterations per substep. The shared centerline and
independent material frames preserve both tool profiles and proximal rotation.
It is a different, approximate Cosserat penalty model: finite stretch/shear,
chordal bend/twist, penalty wall contact and a positional Coulomb approximation.
It does not claim the reference solver's force/friction certificate. Debug shows
iterations, factorizations, geometric errors and whether the iteration budget
was exhausted. Accepted steps must stay within 3% segment length error, 0.1 mm
wall penetration, a 0.1 director/tangent mismatch and the bend limit + 2 degrees.
The adaptive mesh sliders also work in PD. Modified Newton is unavailable for PD.
See [the PD experiment report](reports/projective-dynamics-2026-09-17/README.md)
for measurements and limitations. The default solver remains `shared-axis`.

Historical composite integration notes below describe an earlier rollout:
the application then used the previous `joint-two-channel` solver while
an insertion regression in the new model is being repaired. The experimental
`?coupledSolver=composite-joint` model solves both tools in one common/relative
system with independent material feed, spin, stiffness and contact reactions.
It currently stalls near 131.27 mm in the captured anatomy replay and is not
ready to replace the default solver.
The world uses a fixed 1/120 s step. Numerical work can continue across render
frames, and a timestep is consumed only after acceptance. The on-screen solver
status distinguishes accepted physical steps from rendering FPS. Performance
optimization and the 60 FPS acceptance workload remain a subsequent phase. The
manufactured curvature lives in material frames, while the introducer boundary
controls feeding. Vessel, sheath and tool contacts remain unilateral constraints.
See [the UI integration status and known convergence limits](reports/composite-joint-ui-integration.md)
before treating this development version as a completed simulator.

Earlier solver variants remain available for reproducing previous measurements.
The earlier simultaneous material/contact solver is available at
`?coupledSolver=joint-active-coulomb`. It enables exact active material elimination
and simultaneous Coulomb Newton; `?coupledSolver=reference` selects the previous direct implementation. Debug benchmark
reports identify the selected variant and count its actual solves. This variant
does not yet meet the real-time target: inspect both FPS and physics backlog.
The additional `?coupledSolver=joint-full-band` variant retains the full local
dual operator and uses nonsymmetric band LU for Coulomb Newton. Frozen comparisons
do not show a consistent speed advantage, so it is also an explicit experiment.
See [current integration and validation](reports/coupled-rebuild-progress.md).

For a deterministic Node replay of that variant, use:

```bash
node scripts/benchmark-coupled-rebuild.mjs --solver joint-active-coulomb --deep
npm run test:physics:coupled
```

`scripts/physics/compare-coupled-timesteps.mjs --hz 60,120 --prepare-hz 120`
compares timestep choices after the same guidewire preparation. It is a CPU
diagnostic, not a browser FPS or scheduler acceptance test.

An additional World prototype separates physical motion from geometric bias:
`jointMotionMode: 'split-physical-bias'` with a joint solver installed. It is
available in the timestep diagnostic through `--motion-mode split-physical-bias`;
add `--bias-material-mode preserve-strain` for the strain-preserving variant.
The browser default is unchanged. `configureKirchhoffSplitBias(joint,
{ materialMode: 'preserve-strain' })` additionally preserves the physical
material strain during geometric repair. Small analytic tests pass, but new
contacts after bias, wall witness migration and sheath history still require
closure before general runtime use. See the [prototype handoff](reports/split-physical-bias-handoff.md)
and [strain preservation comparison](reports/split-bias-preserve-strain.md).

For an eligible split joint, `world.stepFixed()` returns an acceptance result.
A rejected step restores the mechanical state and preserves elapsed time;
`world.advance()` retries its prepared inputs without invoking the input callback
again. Independent guidewire preparation remains available before the pair is
eligible. The bias phase first evaluates the existing nonlinear accuracy gates
and skips its linear solve when they already pass. These changes do not establish
the deep-insertion performance target. See [step transactions](reports/split-step-transaction.md).

Run `npm run test:guidewire:mechanics` and `npm run test:catheter:mechanics` for
analytical bending, unloading, torsion and catheter-over-wire regressions.
See [the removal and validation report](reports/direct-only-solver.md) for the
scope of the cleanup and remaining acceptance failures inherited from the direct solver.

See [the contact-block report](reports/catheter-contact-block.md) for the coupled
solve, numerical kernel and deep-insertion performance measurements.

Regenerate the collision asset whenever `Aorta_plain.stl`, its transform, or the offline centerline/SDF pipeline changes. To reproduce all arterial extensions from the original model, run the anatomy generator first:

```bash
npm run anatomy:rebuild
npm run collision:build
npm test
npm run build
```

See `reports/collision-system.md` for the contact API, asset layout, benchmark results, and acceptance status.
For the foreground ten-minute browser workload, open the `Debug` tab, select `Start 10 min`, and leave the simulator in the foreground until the acceptance report appears. The automated Chrome and Safari commands use the same deterministic workload and a two-cycle warmup; Safari WebDriver additionally requires `Allow remote automation` in Safari's Developer settings.
Treat the long browser workload as a regression gate for major physics/rendering changes and releases. During active solver development, use the deterministic unit/regression suite plus a short browser smoke run instead of tuning isolated frame-time outliers.

## Tests and benchmarks

```bash
npm test
npm run test:guidewire:mechanics
npm run test:catheter:mechanics
npm run benchmark:kirchhoff
npm run benchmark:collision
```

### Dwa dostępy udowe

Przyciski **Prawa koszulka** i **Lewa koszulka** w zakładce **Tool selection**
wybierają aktywny dostęp.
Każdy dostęp ma osobny prowadnik, cewnik, ustawienia sztywności i historię solvera.
Klawisze oraz przyciski sterują aktywnym zestawem. Zmiana dostępu kończy bieżący
krok, zatrzymuje automatyczne wycofywanie i podawanie kontrastu; drugi zestaw
pozostaje widoczny i jest nadal symulowany, z zerowym poleceniem wsuwania/obrotu.
Oba solvery mają niezależne zegary, kroki w toku i punkty odtwarzania po odrzuceniu.
Wskaźnik fizyki pokazuje osobno Hz prawego (P) i lewego (L) dostępu. Kontrast jest następnie podawany z wybranej koszulki lub jej cewnika.
W debug aktywna koszulka jest żółta. Kolizje między narzędziami należącymi do
różnych dostępów nie są obecnie rozwiązywane; sprzężenie prowadnik–cewnik działa
osobno dla każdego dostępu.

Testy przełączania i położenia koszulek: `node --test tests/femoralAccess.test.js`.
