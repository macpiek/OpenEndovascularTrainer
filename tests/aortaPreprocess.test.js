import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { createMeshLumenCollider } from '../src/aortaModel.js';
import { createLumenField, preprocessAortaGeometry } from '../src/aortaPreprocess.js';
import { buildStlSliceCenterline } from '../src/stlCenterline.js';
import { createCenterlineCapsuleBroadPhase } from '../src/vesselBroadPhase.js';
import { generateVessel } from '../src/vesselGeometry.js';

function assertFinitePoint(point, label) {
    assert.ok(Number.isFinite(point.x), `${label}.x should be finite`);
    assert.ok(Number.isFinite(point.y), `${label}.y should be finite`);
    assert.ok(Number.isFinite(point.z), `${label}.z should be finite`);
}

function assertSegmentDoesNotCrossWall(segment, geometry) {
    const delta = new THREE.Vector3().subVectors(segment.end, segment.start);
    const length = delta.length();
    if (length < 1e-5) return;
    const ray = new THREE.Ray(segment.start, delta.multiplyScalar(1 / length));
    const hit = geometry.boundsTree?.raycastFirst?.(
        ray,
        THREE.DoubleSide,
        1e-4,
        Math.max(1e-4, length - 1e-4)
    );
    assert.ok(!hit, `centerline connector should not cross vessel wall from ${segment.source}`);
}

function loadTransformedAorta() {
    const buffer = fs.readFileSync('res/Aorta_plain.stl');
    const arrayBuffer = buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
    const geometry = new STLLoader().parse(arrayBuffer);
    geometry.computeBoundingBox();

    const sourceBox = geometry.boundingBox;
    const sourceSize = sourceBox.getSize(new THREE.Vector3());
    const sourceCenter = sourceBox.getCenter(new THREE.Vector3());
    const { vessel } = generateVessel(140, 0);
    const ys = [];
    for (const seg of vessel.segments || []) {
        if (!seg.isSheath) ys.push(seg.start.y, seg.end.y);
    }

    const top = Math.max(...ys, 0) + 15;
    const bottom = Math.min(...ys, -420) - 15;
    const targetCenter = new THREE.Vector3(
        vessel.branchPoint?.x || 0,
        (top + bottom) * 0.5 + 40,
        vessel.branchPoint?.z || 0
    );
    const targetLength = Math.max(300, top - bottom);
    const scale = targetLength * 1.3 / Math.max(1e-6, sourceSize.z);

    geometry.translate(-sourceCenter.x, -sourceCenter.y, -sourceCenter.z);
    geometry.rotateX(-Math.PI / 2);
    geometry.scale(scale, scale, scale);
    geometry.translate(targetCenter.x, targetCenter.y, targetCenter.z);
    return geometry;
}

const geometry = loadTransformedAorta();
const preprocessing = preprocessAortaGeometry(geometry);
const field = preprocessing.lumenField;

console.log('aorta lumen slices', preprocessing.lumenSlices.length);
console.log('aorta lumen samples', preprocessing.interiorSamples.length);
console.log('aorta contour debug segments', preprocessing.lumenContourDebugSegments.length / 6);

assert.ok(preprocessing.lumenSlices.length > 50, 'STL preprocessing should extract lumen slices');
assert.ok(preprocessing.interiorSamples.length > 100, 'STL preprocessing should extract lumen interior samples');

const { vessel: centerlineVessel } = generateVessel(140, 0);
const stlCenterline = buildStlSliceCenterline(geometry, { lumenField: field });
console.log('aorta lumen cast triangles', stlCenterline.lumenCast?.diagnostics?.triangleCount || 0);
assert.ok(
    stlCenterline.lumenCast?.geometry?.attributes?.position?.count > 0,
    'lumen cast should generate an inner vessel surface mesh'
);
assert.equal(
    stlCenterline.lumenCast?.diagnostics?.source,
    'lumen-cast-stl-inner-surface',
    'lumen cast should be the exact inner STL surface, not a generated proxy mesh'
);
const centerlineBroadPhase = createCenterlineCapsuleBroadPhase({
    vessel: centerlineVessel,
    centerlineSegments: stlCenterline.segments,
    lumenSlices: preprocessing.lumenSlices,
    lumenField: field
});
console.log('aorta centerline segments', centerlineBroadPhase.segments.length);
console.log('aorta centerline uncovered nodes', centerlineBroadPhase.diagnostics.uncoveredNodeCount);
console.log('aorta centerline source', centerlineBroadPhase.diagnostics.source);
console.log('aorta centerline components', centerlineBroadPhase.diagnostics.componentCount);
const maxCenterlineSegmentLength = Math.max(
    ...stlCenterline.segments.map(segment => segment.start.distanceTo(segment.end))
);
console.log('aorta centerline max segment length', maxCenterlineSegmentLength);
console.log('aorta centerline centering avg offset', stlCenterline.diagnostics.centerlineCenteringAverageOffset);
console.log('aorta centerline centering max offset', stlCenterline.diagnostics.centerlineCenteringMaxOffset);
console.log('aorta centerline centering avg normalized', stlCenterline.diagnostics.centerlineCenteringAverageNormalizedOffset);
console.log('aorta centerline centering max normalized', stlCenterline.diagnostics.centerlineCenteringMaxNormalizedOffset);
console.log('aorta centerline outlier relaxed nodes', stlCenterline.diagnostics.centerlineOutlierRelaxedNodeCount);
console.log('aorta centerline coverage', JSON.stringify(stlCenterline.diagnostics.centerlineCoverage));
console.log('aorta centerline primary rescue', JSON.stringify({
    candidates: stlCenterline.diagnostics.primaryAxisRescueCandidateComponentCount,
    attempted: stlCenterline.diagnostics.primaryAxisRescueAttemptedComponentCount,
    rescued: stlCenterline.diagnostics.primaryAxisRescuedComponentCount,
    failures: stlCenterline.diagnostics.primaryAxisRescueFailures
}));
console.log('aorta centerline invalid segments', {
    before: stlCenterline.diagnostics.centerlineInvalidSegmentCountBeforeReroute,
    after: stlCenterline.diagnostics.centerlineInvalidSegmentCountAfterReroute,
    final: stlCenterline.diagnostics.centerlineInvalidSegmentCountFinal
});
console.log('aorta centerline topology', {
    before: stlCenterline.diagnostics.centerlineTopologyBeforeCleanup,
    after: stlCenterline.diagnostics.centerlineTopologyAfterCleanup
});
console.log('aorta centerline timings', stlCenterline.diagnostics.timings);
assert.ok(centerlineBroadPhase.segments.length > 240, 'centerline broad phase should cover distal branches');
assert.equal(centerlineBroadPhase.diagnostics.uncoveredNodeCount, 0, 'all lumen contours should have a centerline segment or stub');
assert.equal(centerlineBroadPhase.diagnostics.source, 'lumen-cast-centerline', 'centerline should come from the lumen cast pipeline');
assert.equal(centerlineBroadPhase.diagnostics.componentCount, 1, 'centerline should be one connected branching tree');
assert.equal(
    stlCenterline.diagnostics.centerlineGraphCycleCount,
    0,
    'centerline graph should be acyclic after final simplification'
);
assert.ok(
    stlCenterline.diagnostics.centerlineGraphNodeCount > 1000,
    'centerline graph diagnostics should account for generated centerline nodes'
);
assert.ok(
    maxCenterlineSegmentLength <= stlCenterline.diagnostics.centerlineNodeSpacing + 1e-6,
    'centerline segments should be resampled to the requested node spacing'
);
assert.ok(
    stlCenterline.diagnostics.insertedResampleNodeCount > 0,
    'centerline resampling should add intermediate nodes on long vessel paths'
);
assert.ok(
    stlCenterline.diagnostics.centerlineRefinedNodeCount > 1000,
    'centerline refinement should recenter a substantial number of generated nodes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringAverageOffset < 0.18,
    'centerline nodes should be close to the local lumen center on average'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringMaxOffset < 4.5,
    'centerline should not retain severely off-center nodes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringAverageNormalizedOffset < 0.035,
    'centerline normalized centering error should remain low across vessel sizes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringMaxNormalizedOffset < 0.88,
    'individual centerline nodes should not drift close to the vessel wall'
);
assert.ok(
    stlCenterline.diagnostics.centerlineOutlierRelaxedNodeCount > 0,
    'clearance-based outlier relaxation should improve the worst centerline nodes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineOutlierRelaxDirectCenteredNodeCount > 0,
    'outlier relaxation should directly recenter measured off-axis centerline nodes'
);
assert.equal(
    stlCenterline.diagnostics.centerlinePostPruneDiscardedSegmentCount,
    0,
    'centerline cleanup should not disconnect and discard vessel branches'
);
assert.ok(
    stlCenterline.diagnostics.centerlineRoutedBranchOriginCount > 0,
    'long branch-origin connectors should be routed through the lumen instead of kept as straight shortcuts'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteredBranchRouteNodeCount > 0,
    'routed branch-origin connectors should be explicitly recentered in the lumen'
);
assert.ok(
    stlCenterline.diagnostics.centerlineRoutedBranchPathSegmentCount >=
        stlCenterline.diagnostics.centerlineRoutedBranchOriginCount,
    'routed branch origins should expand into center-biased path segments'
);
assert.ok(
    stlCenterline.diagnostics.axisDiagnostics.some(axis => axis.obliqueRejectedSegmentCount > 0),
    'centerline extraction should reject oblique axial slice artifacts'
);
assert.ok(
    stlCenterline.diagnostics.axisDiagnostics.every(axis => axis.usedLumenSurfaceSlices),
    'centerline nodes should be extracted from the lumen surface model'
);
assert.ok(
    stlCenterline.diagnostics.axisDiagnostics.every(axis => axis.centerMode === 'medial-lumen-surface'),
    'centerline node points should use medial centers of lumen cross-sections'
);

for (const segment of stlCenterline.segments) {
    if (segment.source !== 'stl-slice-branch-origin') continue;
    assertSegmentDoesNotCrossWall(segment, geometry);
}

for (const sample of preprocessing.interiorSamples) {
    const state = field.query(sample);
    assert.ok(
        state.signedDistance > 0,
        `interior sample should be inside lumen, got ${state.signedDistance}`
    );
}

const insideMain = field.query(new THREE.Vector3(0, -300, 0));
const outsideMain = field.query(new THREE.Vector3(0, -300, 80));
const insideIliac = field.query(new THREE.Vector3(-74, -390, 16));
const outsideIliac = field.query(new THREE.Vector3(-74, -390, 70));

assert.ok(insideMain.inside, 'known central lumen point should be inside');
assert.ok(!outsideMain.inside, 'known external main-vessel point should be outside');
assert.ok(insideIliac.inside, 'known iliac lumen point should be inside');
assert.ok(!outsideIliac.inside, 'known external iliac point should be outside');

const collider = createMeshLumenCollider(geometry, { lumenField: field });
assert.equal(collider.pointContact(new THREE.Vector3(0, -300, 0), 0.45).violation, false);
assert.equal(collider.pointContact(new THREE.Vector3(0, -300, 80), 0.45).violation, true);

const emptyField = createLumenField([]);
const emptyCollider = createMeshLumenCollider(new THREE.BufferGeometry(), { lumenField: emptyField });
const malformedSample = new THREE.Vector3(12, -34, 56);
const emptyContact = emptyCollider.pointContact(malformedSample, 0.45);
assert.equal(emptyContact.violation, true, 'empty lumen should still report a boundary violation');
assert.equal(emptyContact.signedDistance, -Infinity, 'empty lumen should preserve signed-distance failure state');
assertFinitePoint(emptyContact.target, 'empty non-scratch contact target');

const scratchContact = { query: {} };
emptyCollider.pointContact(malformedSample, 0.45, scratchContact);
assert.equal(scratchContact.violation, true, 'empty scratch lumen should still report a boundary violation');
assert.equal(scratchContact.signedDistance, -Infinity, 'empty scratch lumen should preserve signed-distance failure state');
assertFinitePoint(scratchContact.target, 'empty scratch contact target');
