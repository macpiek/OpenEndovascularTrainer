import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import { AORTA_TRANSFORM_VERSION } from '../src/aortaTransform.js';
import { LOWER_LIMB_TOE_DISTAL_Z_MM } from '../src/lowerLimbArteries.js';
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
assert.ok(
    asset.metadata.bounds.max[2] > LOWER_LIMB_TOE_DISTAL_Z_MM,
    'STL should include arteries reaching the distal great toes'
);
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
    () => ({ degree: 0, point: null, neighbours: [] })
);
for (let segmentIndex = 0; segmentIndex < asset.metadata.centerline.segmentCount; segmentIndex++) {
    const offset = segmentIndex * stride;
    const startNode = nodes[edges[segmentIndex * 2]];
    const endNode = nodes[edges[segmentIndex * 2 + 1]];
    startNode.degree++;
    endNode.degree++;
    startNode.point ||= [segments[offset], segments[offset + 1], segments[offset + 2]];
    endNode.point ||= [segments[offset + 3], segments[offset + 4], segments[offset + 5]];
    startNode.neighbours.push([segments[offset + 3], segments[offset + 4], segments[offset + 5]]);
    endNode.neighbours.push([segments[offset], segments[offset + 1], segments[offset + 2]]);
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
assert.ok(lowerLimbLeaves.length >= 32);
assert.ok(patientRightLeaves.length >= 16);
assert.ok(patientLeftLeaves.length >= 16);
assert.ok(Math.abs(patientRightLeaves.length - patientLeftLeaves.length) <= 2);
assert.ok(Math.min(...patientRightLeaves.map(point => point[1])) < -1285);
assert.ok(Math.min(...patientLeftLeaves.map(point => point[1])) < -1285);

const toeLeaves = nodes.filter(node =>
    node.degree === 1 &&
    node.point[1] < -1370 &&
    node.point[2] > 85 &&
    Math.abs(node.point[0]) < 150
);
const rightToeLeaves = toeLeaves.filter(node => node.point[0] < 0);
const leftToeLeaves = toeLeaves.filter(node => node.point[0] > 0);
assert.ok(rightToeLeaves.length >= 6, 'each right toe should retain a distal arterial outlet');
assert.ok(leftToeLeaves.length >= 6, 'each left toe should retain a distal arterial outlet');
for (const [side, leaves, distalLevels] of [
    ['right', rightToeLeaves, [126, 115, 107, 99, 91]],
    ['left', leftToeLeaves, [126, 115, 107, 99, 91]]
]) {
    for (const distalZ of distalLevels) {
        assert.ok(
            leaves.some(node => Math.abs(node.point[2] - distalZ) < 2),
            `${side} toe at distal Z=${distalZ} should retain a digital outlet`
        );
    }
}

const field = new VesselContactField(asset);
const plantarArchSamples = [
    [-50.4, -1382.5, 46.7],
    [-72, -1386, 47],
    [-95, -1385, 44],
    [76.3, -1381.8, 47.5],
    [100, -1385.13, 46.38],
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
    ['right deep femoral origin', -100, -538, -13, 1],
    ['right popliteal division', -69, -910, -66, 1],
    ['right anterior tibial passage', -75, -942, -24, 1],
    ['right posterior tibial calf', -59, -1010, -70, 1],
    ['right fibular calf', -84, -1010, -69, 1],
    ['right lateral circumflex femoral', -122, -548, -8, 0.5],
    ['right first perforating femoral', -103, -620, -56, 0.45],
    ['right descending genicular', -55, -770, -29, 0.45],
    ['right anterior tibial recurrent', -83, -919, -12, 0.15],
    ['left deep femoral origin', 98, -538, -13, 1],
    ['left popliteal division', 84, -910, -66, 1],
    ['left anterior tibial passage', 94, -942, -24, 1],
    ['left posterior tibial calf', 80, -1010, -70, 1],
    ['left fibular calf', 104, -1010, -69, 1],
    ['left lateral circumflex femoral', 120, -548, -8, 0.5],
    ['left first perforating femoral', 101, -620, -56, 0.45],
    ['left descending genicular', 68, -770, -29, 0.45],
    ['left anterior tibial recurrent', 102, -919, -12, 0.15]
];
for (const [name, x, y, z, minimumClearance] of skeletalAlignmentSamples) {
    const contact = field.querySphere({ x, y, z }, 0);
    const packedLumen = field.packedLumenField.query({ x, y, z });
    assert.ok(
        contact.inside && contact.signedDistance > 0,
        `${name} should be inside the generated lumen`
    );
    assert.ok(
        packedLumen.signedDistance > minimumClearance,
        `${name} should retain ${minimumClearance} mm of packed lumen clearance`
    );
}

for (const leaf of toeLeaves) {
    const neighbour = leaf.neighbours[0];
    const point = {
        x: leaf.point[0] * 0.6 + neighbour[0] * 0.4,
        y: leaf.point[1] * 0.6 + neighbour[1] * 0.4,
        z: leaf.point[2] * 0.6 + neighbour[2] * 0.4
    };
    const contact = field.querySphere(point, 0);
    assert.ok(
        contact.inside && contact.signedDistance > 0.1,
        `toe artery at ${leaf.point} should retain lumen before its terminal cap`
    );
}

console.log('lower-limb arterial leaves', {
    right: patientRightLeaves,
    left: patientLeftLeaves,
    toeLeaves: toeLeaves.length,
    plantarArchSamples: plantarArchSamples.length,
    skeletalAlignmentSamples: skeletalAlignmentSamples.length
});
