Schema `oet-world-physical-replay-v1` — frozen data for a one-time JointWorldAdapter import.

Canonical input: `composite-world-replay/targetPrepared.json`. It is the fully prepared zero-command dt at 318 mm wire / 9 mm catheter, captured immediately before the original `World.stepFixed`. It is not an executable World checkpoint or an accepted Joint state.

The single implementation is `scripts/physics/capture-composite-world-replay.mjs`. It exports `encodeReplayData`, `importReplayData`, `captureWorldReplay`, and `verifyWorldReplay`. Import does not load source modules, recreate class methods, prepare actuation, query anatomy, or call a solver. The separate verification CLI explicitly loads the frozen anatomy and runs original field queries only.

**Wire format, exact rules**

The envelope is `{schema, byteOrder, contentSha256, root, nodes}`. `schema` equals the literal above; `byteOrder` is `LE` for this bundle. The importer rejects a different host byte order. `contentSha256` is SHA-256 of UTF-8 `JSON.stringify({root,nodes})` in this property order. File SHA-256, including the final newline, is separately listed in `index.json` and the frozen artifact manifest.

A value is a JSON primitive or exactly one reference/tag token. Finite ordinary numbers use JSON numbers. Special values use `{$number:'NaN'|'Infinity'|'-Infinity'|'-0'}`; undefined uses `{$undefined:true}`; bigint uses `{$bigint:decimalString}`. `{$ref:i}` addresses the zero-based node table. `{$external:id}` remains an explicit data token; body IDs are `body:guidewire` and `body:catheter`, and anatomy/fixture references use their recorded strings. `{$function:name,callable:false}` is a non-executable token. No function source is exported or evaluated.

Each node has one of these forms:

| kind | Required contents |
| --- | --- |
| `buffer` | `type`, `length` (null for ArrayBuffer/DataView), `byteLength`, `sha256`, `base64`. The bytes are the exact visible range of the original view, including NaN payloads, signed zero and unused/inactive storage. Type is a named JS numeric typed array, DataView or ArrayBuffer. Import allocates fresh owned storage and validates length and hash. |
| `array` | `items`, an ordered array of encoded values. |
| `map` | `entries`, ordered encoded `[key,value]` pairs. |
| `set` | `values`, ordered encoded values. |
| `object` | `className` and `properties`, with own enumerable keys sorted during encoding. Import creates plain data with inert class metadata; prototype methods/accessors are not recreated. |

Repeated object/view references and cycles use `$ref` and survive import. Distinct typed-array views receive separately owned byte storage; backing-buffer overlap between different views is not a runtime alias contract. Two imports share no numeric storage. Native getter-backed helper vectors remain their captured own data; consumers should use `bodies[*].x/y/z` and quaternion arrays for the authoritative physical body values.

**Decoded semantic structure**

| Path | Meaning |
| --- | --- |
| `phase`, `attempt` | Exact capture boundary and one-based original attempted dt. Target is `prepared-before-world-stepFixed`, attempt 890. |
| `command` | Requested four normalized controls and catheter type, dt, before/after progress and wrapped handle angles, actual progress deltas. `preparedOnce:true` means fixture actuation already ran. Do not call fixture.step to import these data. |
| `fixture.snapshot/config` | Actual progress, counters and explicit benchmark settings. Fingerprints supplement the bytes; they do not replace them. |
| `bodies` | Two complete physical body data objects, keyed by original `id`. All 104 direct numeric views per body are present at original capacity, not sliced to active range. Includes current/previous geometry, translational/angular velocities, quaternion/history, masses/inertias, material labels, natural rest rotations, constitutive compliances, rest lengths, controls, pinned flags, and legacy reaction/contact arrays. All own scalar settings and ranges are retained. |
| `constraints.sheaths/containments/externalContacts` | Original constraint metadata and numeric history, including stable body-reference tokens, manifold maps, portal records, closest-segment/material data, lambdas and friction settings. No values are converted into Joint multipliers. |
| `fixture.rodState.nodeStorage` | Original rod storage, separately owned from the World body. |
| `fixture.transport/catheter` | Original numeric preparation/history data, centerline labels, pending feed/rotation, frame helpers and available catheter profile discretization. Rendering objects and functions are omitted explicitly. |
| `profiles` | Actual profile IDs, shaft/tip scale fields, applied ranges and tip-coordinate conventions. Authoritative constitutive arrays remain in body data. Wire profile application covered 0..200 before its active range was reduced; catheter covered 0..17. Profile factory definitions are in the frozen source manifest. |
| `world` | Own primitive World settings/counters, pending step metadata and last returned result. No solver implementation or timing workspace is imported. |
| `originalContactHistory` | Original manifold and Kirchhoff contact records at the snapshot boundary. At prepared phase these may precede the newly prepared actuation; they are not a newly evaluated contact set. |
| `missing` | Explicit nulls for continuous winding, unwrapped handle rotations, arbitrary-label material velocity sampler, Joint IDs, SI mass/rigidity calibration and accepted-only history-commit counter. Do not infer these from zero commands or quaternions. |
| `omissions` | Exact excluded top-level paths. `catheter.material` is its THREE rendering material; mechanical profile metadata/arrays are included elsewhere. `body.kirchhoffScratch` is the old numerical workspace, not a material-state source. |

`run.json` and `capture-outcome.json` use the same graph encoding for diagnostics, preserving nonfinite values. `index.json` is plain JSON with artifact hashes, per-body buffer inventory and roundtrip evidence. It contains no physical buffer payload. `targetPrepared-contacts.json` has `{freshWall,originalHistory}` in the same graph format. Fresh wall records identify body, segment, query hints, method, original returned source/feature/distance/normal and the activation threshold. Original non-wall history stays separately labelled.

Snapshot roles: `initial` is reset state; `terminalPrepared` is the last attempted dt before World; `terminalAfter` is its returned state; `lastAccepted` is the most recent state after an accepted fixture.step. Here `targetPrepared == terminalPrepared` and `lastAccepted == terminalAfter` byte-for-byte, because the terminal zero-command reference step passed. If a reference gate fails, capture stops immediately, targetPrepared is absent, lastAccepted remains the earlier accepted state, and the actual prepared/returned failed states plus exact result/status are retained. A legacy void-return failed closure is explicitly labelled `reference-uncertified-closure`; it is not counted as accepted even if legacy counters advanced.

All numeric labels/units keep their original model meaning. Wire reference labels and catheter centerline-distance labels are not silently identified; no chart resampling, frame unwrap, density derivation, force conversion or remap is performed. Whole-file hashes may include captured diagnostic timings; they identify these immutable artifacts, not a promise that a new physical rerun reproduces wall-clock values.
