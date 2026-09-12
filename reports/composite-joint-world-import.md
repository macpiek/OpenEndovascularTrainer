# World → fixed union Joint chart import

## Delivered scope

`src/physics/kirchhoffCompositeJointWorldImport.js` exports `importCompositeJointWorld`. It imports actual World body buffers through explicit own physical/material metadata and returns an initial Float64 Joint state, publication bindings, full refinement maps and owned history. It neither advances the solver nor writes World buffers. No existing World, Step, adapter or older physics module is changed.

The chart is the exact sorted union of all original active own node coordinates. There is no approximate coordinate merge, resampling onto either tool's spacing, common-axis averaging, or invented endpoint. Every original active node survives. Each new own edge belongs wholly to one original own edge. At an inserted node, position and physical velocity use the affine restriction at the actual source-edge fraction `u`. Own edge reference frames and known unwrapped angles are copied. An inserted straight subdivision hinge gets zero reference twist; every original own hinge retains its explicit winding.

The catheter supplies common `q` wherever it physically exists; wire-only nodes use the wire position. Every overlap node, including material tips, has three independent relative coordinates and an identity basis for `rho = wire - catheter`. Original own node coordinates and indices are retained exactly. Reconstruction after the change of variables is checked against a purely arithmetic bound for subtraction and addition: `4 * EPSILON * (abs(q) + abs(rho) + abs(source)) + 8 * MIN_VALUE`, evaluated with scaling before summation to avoid overflow. This conservatively bounds the two IEEE-754 rounding operations and subnormal underflow, without introducing a geometric tolerance. Refined affine positions can differ by Float64 addition roundoff, which is reported as `maxAffinePositionError`; their own frame/geometry consistency also passes the existing Joint state validation.

This is a fixed-chart initial import. `remapReady`, `fullFeedLifecycleReady`, `solverAdvanced`, and `certified` are all false. The root adapter owns transaction/publication and must retain the authoritative Joint state across steps. An evolved union curve or independent subedge spin field generally cannot be losslessly coarsened into the legacy body buffers and then reimported.

## Input contract

```js
const imported = importCompositeJointWorld({
  tools: [{
    body,                         // actual World body, with activeStart/activeEnd
    toolId: 'wire',               // explicit guidewire body.id -> physical wire
    nodeCoordinates,             // active node count, strictly increasing
    reference,                   // active edge count; own {tangent, director}
    angles,                      // active edge count; known unwrapped angles
    referenceTwists,              // active interior node count; known winding
    winding: 'explicit-unwrapped',
    materialLabels,              // active node count; explicit reference labels
    velocityInterpretation: 'physical-material-velocity',
    material: {
      dsDx,                     // positive constant reference arclength slope
      massPerMaterialLength,    // positive explicit physical line density
      materialAt(request) { return compiledOwnSupportMaterial; }
    }
  }],
  time: 0,
  step: 0
});
```

One or two descriptors are supported. `body.id === 'guidewire'` or `'wire'` maps explicitly to `toolId === 'wire'`; `'catheter'` maps to `'catheter'`. Duplicate bodies/tools and unknown mappings reject. Descriptor arrays cover only the inclusive actual active node range, not the entire allocated body. Inactive buffer values are ignored; tests intentionally fill them with NaNs. Only actual active `x/y/z`, `velocityX/Y/Z`, `restLength` and material quaternion buffers are read. Legacy inverse masses, material-coordinate buffers and angular velocities do not supply missing physical inputs.

Provided frames must be unit, orthogonal and belong to the actual source chord. Source quaternion material directors must agree with the supplied own reference plus angle, modulo full turns, within `1e-9`. The quaternion is only a physical orientation check; it never supplies an unwrapped angle or twist. Supplied reference-twist phase must agree with spatial minimal parallel transport modulo `2π`, while its explicitly known lift is retained. Antiparallel source hinges need a different chart and reject.

Material labels must agree with the declared affine map to ordinary Float64 roundoff (64 machine epsilons relative to label magnitude). Original labels are retained exactly in source history and at original coordinates in the edge history. New labels follow the declared map. Each source rest length must equal `dsDx * sourceDx`: for actual Float32 World rest storage this means exact equality to `Math.fround(dsDx * sourceDx)`; other storage permits only Float64 roundoff. The maximum acknowledged Float32 storage quantization is reported. Any additional slewed/rest mismatch rejects. Geometric strain is retained; physical chord length is not substituted for rest length.

The provider receives `{toolId, bodyId, vertex, coordinate, start, end, materialCoordinate, materialStart, materialEnd, dsDx, sourceEdges}` at every active own Joint hinge. `start/end` delimit the actual dual support between neighboring union-edge midpoints. `sourceEdges` lists actual original edge indices intersecting that support. The provider supplies the compiled stiffness/intrinsic material for that entire support, including any required profile integration; the importer does not infer constitutive parameters from legacy compliance. Output must have finite flat stiffness length 9 and intrinsic length 3; it is validated as SPD with `compileCompositeMaterial`, copied and deeply frozen.

The Joint material provider is an owned immutable snapshot of these exact supports and label maps. It does not call a mutable external profile closure during later assembly. A changed support request explicitly rejects. Moving material maps or changing profiles require explicit preparation by the later lifecycle layer; this importer does not implement that refresh.

## Returned maps and physical history

`bindings` matches the root adapter's publication schema:

```js
[{ toolId, body,
   nodes: [{ node, jointNode, trace: 'left' | 'right' }],
   edges: [{ edge, jointEdge }]
}]
```

Each original active node and edge appears exactly once. Original nodes use the outgoing/right velocity trace except the terminal node, which uses left. An original edge's orientation view selects the union child containing its reference-coordinate midpoint; a cut exactly at the midpoint selects the outgoing/right child. These are legacy view choices only. `mappings.get(toolId)` additionally retains every original-edge → child-edge/fraction relation, every child → original edge relation, original node coordinates and the source fraction/original-node identity of every active union node.

`state.materialVelocities` and the separate `history.materialVelocities` contain own physical material velocities on every active union edge, with `sStart/sEnd` labels and the explicit interpretation. Source nodal velocities are copied, inserted endpoint velocities are affine restrictions, and both output histories own their arrays separately. `history.sourceSnapshots` deeply freezes original coordinates, physical positions, velocities, labels, actual rest buffers, own frames, unwrapped angles/twists, source quaternions and density. `history.materialSamples` exposes the immutable compiled support samples.

All angular velocity, material spin and frame spin history fields are explicitly null, and `includesAngularVelocity` is false. Known orientation history is present, but it does not determine an angular rate. No old force/contact/friction history is fabricated or transferred. Joint length/boundary multiplier initialization is the existing Joint state's unadvanced default, not a certified equilibrium or imported force history.

## Explicit limitations and rejection behavior

Disconnected active intervals, tools touching only at one coordinate, fewer than three union nodes, inconsistent metadata, degenerate own physical edges, unknown winding, stale orientation, missing density/profile and rest-map mismatch reject. An original-node reconstruction error above the arithmetic bound rejects with `code: 'joint-world-import-unsupported'` and reason `original-node-reconstruction-exceeds-arithmetic-roundoff`. A tested irregular curved case has maximum Float64 reconstruction error `1.1102230246251565e-16` and is accepted within this bound. It publishes identically to the original Float32 World positions. `evidence.maxOriginalNodeRoundoff` and `originalNodeRoundoff` record maximum/per-node component errors and bounds; `exactOriginalNodeCoordinateCoverage` is separate from `bitExactOriginalNodeReconstruction`. Actual Float32 publication differences are reported in each node's `float32PublicationErrors`, `maxFloat32PublicationError` and `float32PublicationMismatchNodeCount`. Nearby representable nodes remain distinct in the exact union. Extremely small intervals remain subject to the existing own-frame numerical validity checks.

Inactive portions of `state.toolPositions` are the existing Joint state placeholders and do not represent a physical extension of that tool. Consumers must obey `layout.edgeToolIds` / the own active mappings. This module structurally validates World buffers without importing the World class, avoiding an import cycle when World later uses the root adapter.

## Validation

**32/32 tests pass** in `tests/kirchhoffCompositeJointWorldImport.test.js`, using actual `EndovascularPhysicsWorld.createRod` bodies. Coverage includes:

- Spacing 5/4, nonzero active starts, independent curved/offset axes, known multi-turn own spins and hinge gauge winding, mixed exposed/overlap spans and unequal ends.
- Exact original own node roundtrip (maximum error 0), affine subdivisions, full relative endpoint modes, complete child coverage and copied own frames/spins/winding.
- Actual Float32 rest quantization and rejection of additional rest slewing; explicit density, labels and compiled material support snapshots consumed by authoritative Joint assembly.
- Publication coverage, outgoing midpoint tie, separate history/source ownership, physical velocities and no angular/force-history inference.
- Single-tool charts, staggered opposite exposed ends, distinct nearby coordinates, arithmetic coordinate-change roundoff and 21 malformed-input cases, with no body mutation.

Validation used an isolated dependency snapshot at `/tmp/oet-joint-world-import-validation-8996`: 83 files, with only the new source and new test from workspace 8996 and all 81 existing dependencies copied read-only from authoritative root workspace 901c. The snapshot records per-file origin and SHA-256 in `dependency-manifest.json`. The temporary `node_modules` link points to root's existing dependencies. No authoritative root files were written.

Command: `node --test tests/kirchhoffCompositeJointWorldImport.test.js` in that snapshot. Source syntax validation also passes. The frozen handoff bundle contains only the three new owned deliverable files, their patch, hashes, test output and the dependency manifest.
