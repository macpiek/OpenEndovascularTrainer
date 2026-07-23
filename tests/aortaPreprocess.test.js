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
console.log('aorta direct medial slices', stlCenterline.lumenCast?.diagnostics?.sliceCount || 0);
assert.equal(
    stlCenterline.lumenCast?.diagnostics?.source,
    'direct-medial-slices',
    'centerline generation should use direct lumen slices without building a proxy surface'
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
console.log('aorta centerline coverage', JSON.stringify(stlCenterline.diagnostics.centerlineCoverage));
console.log('aorta centerline invalid segments', {
    before: stlCenterline.diagnostics.centerlineInvalidSegmentCountBeforeReroute,
    after: stlCenterline.diagnostics.centerlineInvalidSegmentCountAfterReroute,
    final: stlCenterline.diagnostics.centerlineInvalidSegmentCountFinal
});
console.log('aorta centerline topology', {
    before: stlCenterline.diagnostics.centerlineTopologyBeforeCleanup,
    after: stlCenterline.diagnostics.centerlineTopologyAfterCleanup
});
console.log('aorta medial tree', JSON.stringify(stlCenterline.diagnostics.medialTree));
console.log('aorta centerline component segments', stlCenterline.diagnostics.componentSegmentCounts);
console.log('aorta centerline refinement', {
    refined: stlCenterline.diagnostics.centerlineRefinedNodeCount,
    failed: stlCenterline.diagnostics.centerlineRefinementFailedNodeCount,
    wallRejected: stlCenterline.diagnostics.centerlineRefinementWallRejectedSegmentCount,
    wallClamped: stlCenterline.diagnostics.centerlineRefinementWallClampedSegmentCount
});
console.log('aorta centerline clearance', stlCenterline.diagnostics.centerlineClearanceMaximization);
console.log('aorta centerline timings', stlCenterline.diagnostics.timings);
assert.ok(centerlineBroadPhase.segments.length > 2500, 'centerline broad phase should cover the full distal tree');
assert.equal(centerlineBroadPhase.diagnostics.uncoveredNodeCount, 0, 'all lumen contours should have a centerline segment or stub');
assert.equal(centerlineBroadPhase.diagnostics.source, 'medial-slice-teasar', 'centerline should come from the medial tree pipeline');
assert.equal(centerlineBroadPhase.diagnostics.componentCount, 1, 'centerline should be one connected branching tree');
assert.equal(
    stlCenterline.diagnostics.centerlineGraphCycleCount,
    0,
    'centerline graph should be acyclic after final simplification'
);
assert.ok(
    stlCenterline.diagnostics.centerlineGraphNodeCount > 2500,
    'centerline graph diagnostics should account for generated centerline nodes'
);
assert.ok(
    maxCenterlineSegmentLength <= stlCenterline.diagnostics.centerlineNodeSpacing + 1e-6,
    'centerline segments should be resampled to the requested node spacing'
);
assert.ok(
    stlCenterline.diagnostics.centerlineRefinedNodeCount > 4000,
    'centerline refinement should recenter a substantial number of generated nodes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringAverageOffset < 0.14,
    'centerline nodes should be close to the local lumen center on average'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringMaxOffset < 2.7,
    'centerline should not retain severely off-center nodes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringAverageNormalizedOffset < 0.03,
    'centerline normalized centering error should remain low across vessel sizes'
);
assert.ok(
    stlCenterline.diagnostics.centerlineCenteringMaxNormalizedOffset < 0.33,
    'individual centerline nodes should not drift close to the vessel wall'
);
assert.equal(
    stlCenterline.diagnostics.centerlineInvalidSegmentCountFinal,
    0,
    'every final centerline segment should remain inside the STL lumen'
);
assert.ok(
    stlCenterline.diagnostics.centerlineClearanceMaximization.movedNodeCount > 1000,
    'wall-clearance maximization should improve the generated medial paths'
);
assert.ok(
    stlCenterline.diagnostics.centerlineClearanceMaximization.averageClearanceGain > 0.2,
    'wall-clearance maximization should materially increase distance from vessel walls'
);
assert.equal(
    stlCenterline.diagnostics.centerlineTopologyAfterCleanup.severeBacktrackNodeCount,
    0,
    'simulation centerline should contain no severe backtracks'
);
assert.ok(
    stlCenterline.diagnostics.centerlineTopologyAfterCleanup.sharpTurnNodeCount < 50,
    'centerline should keep the number of sharp degree-two turns low'
);
assert.ok(
    stlCenterline.diagnostics.centerlineTopologyAfterCleanup.maxDeflectionDegrees < 82,
    'centerline should not contain near-reversals that destabilize physics'
);
assert.ok(
    stlCenterline.diagnostics.centerlineTopologyAfterCleanup.leafNodeCount >= 100,
    'centerline should retain the many distal endpoints of the supplied arterial tree'
);
assert.ok(
    stlCenterline.diagnostics.centerlineTopologyAfterCleanup.branchNodeCount >= 95,
    'centerline should retain distal bifurcations instead of collapsing small vessels'
);
assert.equal(
    stlCenterline.diagnostics.centerlineCoverage.uncoveredSampleCount,
    0,
    'all retained medial samples should be represented by the final tree'
);
assert.equal(
    stlCenterline.diagnostics.centerlineCoverage.coverageRate,
    1,
    'the retained lumen skeleton should have complete centerline coverage'
);
assert.equal(
    stlCenterline.diagnostics.medialTree.rejectedTwigCount,
    0,
    'short connected vessel paths must be represented rather than counted as covered and omitted'
);
assert.ok(
    stlCenterline.diagnostics.medialTree.retainedGraphNodeCount > 10000,
    'medial extraction should preserve dense evidence from small vessels'
);
assert.ok(
    stlCenterline.diagnostics.medialTree.discardedDisconnectedNodeCount <= 30,
    'only isolated sub-grid artifacts may be discarded from the connected vessel tree'
);
assert.ok(
    stlCenterline.diagnostics.medialTree.graphComponentDetails.slice(1).every(component =>
        component.averageClearance < stlCenterline.diagnostics.medialTree.gridSpacing * 0.7
    ),
    'discarded disconnected components should be thinner than the extraction grid'
);
assert.ok(
    stlCenterline.diagnostics.axisDiagnostics.every(axis => axis.usedLumenSurfaceSlices),
    'centerline nodes should be extracted directly from lumen contours'
);
assert.ok(
    stlCenterline.diagnostics.axisDiagnostics.every(axis => axis.centerMode === 'topological-medial-axis'),
    'centerline node points should use topological medial axes of lumen cross-sections'
);
assert.ok(
    stlCenterline.diagnostics.timings.totalMs < 60000,
    'centerline extraction should remain practical for simulator startup'
);

for (const segment of stlCenterline.segments) {
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
