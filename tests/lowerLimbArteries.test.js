import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import { AORTA_TRANSFORM_VERSION } from '../src/aortaTransform.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { VesselContactField } from '../src/physics/collision/vesselContactField.js';

function arrayBuffer(bytes) {
    return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

const stlBytes = fs.readFileSync('res/Aorta_plain.stl');
const collisionBytes = fs.readFileSync('res/Aorta_plain.collision.bin');
const asset = decodeCollisionAsset(arrayBuffer(collisionBytes));
const sourceHash = crypto.createHash('sha256').update(stlBytes).digest('hex');

assert.equal(asset.metadata.source.stlSha256, sourceHash);
assert.equal(asset.metadata.transform.version, AORTA_TRANSFORM_VERSION);
assert.ok(asset.metadata.bounds.min[1] < -1380, 'STL should include the plantar arches');
assert.ok(asset.metadata.centerline.segmentCount > 4500);

const diagnostics = asset.metadata.centerline.diagnostics;
assert.equal(diagnostics.componentCount, 1);
assert.equal(diagnostics.centerlineGraphCycleCount, 0);
assert.equal(diagnostics.centerlineInvalidSegmentCountFinal, 0);
assert.equal(
    diagnostics.centerlineTopologyAfterCleanup.severeBacktrackNodeCount,
    0
);

const stride = asset.metadata.centerline.stride;
const segments = asset.arrays.centerlineSegments;
const edges = asset.arrays.centerlineEdges;
const nodes = Array.from(
    { length: asset.metadata.centerline.nodeCount },
    () => ({ degree: 0, point: null })
);
for (let segmentIndex = 0; segmentIndex < asset.metadata.centerline.segmentCount; segmentIndex++) {
    const offset = segmentIndex * stride;
    const startNode = nodes[edges[segmentIndex * 2]];
    const endNode = nodes[edges[segmentIndex * 2 + 1]];
    startNode.degree++;
    endNode.degree++;
    startNode.point ||= [segments[offset], segments[offset + 1], segments[offset + 2]];
    endNode.point ||= [segments[offset + 3], segments[offset + 4], segments[offset + 5]];
}

const lowerLimbLeaves = nodes
    .filter(node =>
        node.degree === 1 &&
        node.point[1] < -600 &&
        Math.abs(node.point[0]) < 150
    )
    .map(node => node.point);
const patientRightLeaves = lowerLimbLeaves.filter(point => point[0] < 0);
const patientLeftLeaves = lowerLimbLeaves.filter(point => point[0] > 0);

// The medial simulation graph is deliberately acyclic. The physical plantar
// loops remain complete in the STL/SDF, while the spanning tree may cut each
// loop at a junction or along an edge and therefore retain a slightly
// different leaf count on either side.
assert.ok(lowerLimbLeaves.length >= 8 && lowerLimbLeaves.length <= 12);
assert.ok(patientRightLeaves.length >= 4);
assert.ok(patientLeftLeaves.length >= 4);
assert.ok(Math.abs(patientRightLeaves.length - patientLeftLeaves.length) <= 1);
assert.ok(Math.min(...patientRightLeaves.map(point => point[1])) < -1285);
assert.ok(Math.min(...patientLeftLeaves.map(point => point[1])) < -1285);

const field = new VesselContactField(asset);
const plantarArchSamples = [
    [-51.5, -1386, 48.5],
    [-72, -1386, 47],
    [-95, -1385, 44],
    [76, -1383.5, 47],
    [99, -1386, 47],
    [124, -1385, 44]
];
for (const [x, y, z] of plantarArchSamples) {
    const contact = field.querySphere({ x, y, z }, 0);
    const packedLumen = field.packedLumenField.query({ x, y, z });
    assert.ok(
        contact.inside && contact.signedDistance > 0,
        `plantar arch sample ${x},${y},${z} should be inside the lumen`
    );
    assert.ok(
        packedLumen.signedDistance > 0.5,
        `plantar arch sample ${x},${y},${z} should retain measurable lumen clearance`
    );
}

const skeletalAlignmentSamples = [
    ['right deep femoral origin', -100, -538, -13],
    ['right popliteal division', -69, -910, -66],
    ['right anterior tibial passage', -75, -942, -24],
    ['right posterior tibial calf', -59, -1010, -70],
    ['right fibular calf', -84, -1010, -69],
    ['left deep femoral origin', 98, -538, -13],
    ['left popliteal division', 84, -910, -66],
    ['left anterior tibial passage', 94, -942, -24],
    ['left posterior tibial calf', 80, -1010, -70],
    ['left fibular calf', 104, -1010, -69]
];
for (const [name, x, y, z] of skeletalAlignmentSamples) {
    const contact = field.querySphere({ x, y, z }, 0);
    const packedLumen = field.packedLumenField.query({ x, y, z });
    assert.ok(
        contact.inside && contact.signedDistance > 0,
        `${name} should be inside the generated lumen`
    );
    assert.ok(
        packedLumen.signedDistance > 1,
        `${name} should retain at least 1 mm of packed lumen clearance`
    );
}

console.log('lower-limb arterial leaves', {
    right: patientRightLeaves,
    left: patientLeftLeaves,
    plantarArchSamples: plantarArchSamples.length,
    skeletalAlignmentSamples: skeletalAlignmentSamples.length
});
