import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import * as THREE from 'three';
import { AORTA_TRANSFORM_VERSION } from '../src/aortaTransform.js';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { VesselContactField } from '../src/physics/collision/vesselContactField.js';
import {
    UPPER_LIMB_FINGER_DISTAL_Y_MM,
    UPPER_LIMB_HAND_DISTAL_Y_MM,
    UPPER_LIMB_LATERAL_EXTENT_MM
} from '../src/upperLimbArteries.js';

function arrayBuffer(bytes) {
    return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

function unpackCenterline(asset) {
    const packed = asset.arrays.centerlineSegments;
    const edges = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    const segments = [];
    for (let index = 0; index < asset.metadata.centerline.segmentCount; index++) {
        const offset = index * stride;
        segments.push({
            id: index,
            start: new THREE.Vector3(
                packed[offset],
                packed[offset + 1],
                packed[offset + 2]
            ),
            end: new THREE.Vector3(
                packed[offset + 3],
                packed[offset + 4],
                packed[offset + 5]
            ),
            radiusStart: packed[offset + 6],
            radiusEnd: packed[offset + 7],
            safeRadius: packed[offset + 8],
            nodeStartId: edges[index * 2],
            nodeEndId: edges[index * 2 + 1]
        });
    }
    return segments;
}

const stlBytes = fs.readFileSync('res/Aorta_plain.stl');
const collisionBytes = fs.readFileSync('res/Aorta_plain.collision.bin');
const asset = decodeCollisionAsset(arrayBuffer(collisionBytes));
const sourceHash = crypto.createHash('sha256').update(stlBytes).digest('hex');

assert.equal(asset.metadata.source.stlSha256, sourceHash);
assert.equal(asset.metadata.transform.version, AORTA_TRANSFORM_VERSION);
assert.ok(
    asset.metadata.bounds.min[0] < -UPPER_LIMB_LATERAL_EXTENT_MM,
    'right upper-limb vessels should reach beyond the wrist skeleton'
);
assert.ok(
    asset.metadata.bounds.max[0] > UPPER_LIMB_LATERAL_EXTENT_MM,
    'left upper-limb vessels should reach beyond the wrist skeleton'
);
assert.ok(
    asset.metadata.bounds.min[1] < UPPER_LIMB_FINGER_DISTAL_Y_MM,
    'upper-limb vessels should reach the distal fingers'
);
assert.ok(asset.metadata.centerline.segmentCount > 8000);

const diagnostics = asset.metadata.centerline.diagnostics;
assert.equal(diagnostics.componentCount, 1);
assert.equal(diagnostics.centerlineGraphCycleCount, 0);
assert.equal(diagnostics.centerlineInvalidSegmentCountFinal, 0);
assert.equal(
    diagnostics.centerlineTopologyAfterCleanup.severeBacktrackNodeCount,
    0
);

const centerlineSegments = unpackCenterline(asset);
const nodes = new Map();
for (const segment of centerlineSegments) {
    for (const [nodeId, point, neighbour] of [
        [segment.nodeStartId, segment.start, segment.end],
        [segment.nodeEndId, segment.end, segment.start]
    ]) {
        const node = nodes.get(nodeId) || { degree: 0, point, neighbours: [] };
        node.degree++;
        node.neighbours.push(neighbour);
        nodes.set(nodeId, node);
    }
}
const upperLimbLeaves = [...nodes.values()].filter(node =>
    node.degree === 1 &&
    Math.abs(node.point.x) > 100 &&
    node.point.y > -680 &&
    node.point.y < 110
);
const rightLeaves = upperLimbLeaves.filter(node => node.point.x < 0);
const leftLeaves = upperLimbLeaves.filter(node => node.point.x > 0);
assert.ok(rightLeaves.length >= 20, 'right arm should retain its fine distal branches');
assert.ok(leftLeaves.length >= 20, 'left arm should retain its fine distal branches');
assert.ok(
    Math.abs(rightLeaves.length - leftLeaves.length) <= 2,
    'paired upper-limb trees should retain comparable outlet counts'
);

const field = new VesselContactField(asset);
const lumenSamples = [
    ['right axillary', -154, 64, -39, 1],
    ['right brachial', -185, -112, -62, 1],
    ['right deep brachial', -211, -158, -116, 0.45],
    ['right radial', -218, -345, -51, 0.75],
    ['right ulnar', -198, -345, -64, 0.75],
    ['right anterior interosseous', -207, -365, -60, 0.3],
    ['right thoracoacromial', -136, 75, -27, 0.25],
    ['right thoracodorsal', -138, -65, -90, 0.2],
    ['right posterior interosseous', -217, -390, -75, 0.18],
    ['right superficial palmar arch', -209, -535, 26, 0.35],
    ['right deep palmar arch', -207, -519, 13, 0.35],
    ['left axillary', 150, 64, -39, 1],
    ['left brachial', 179, -112, -62, 1],
    ['left deep brachial', 211, -158, -116, 0.45],
    ['left radial', 220, -345, -51, 0.75],
    ['left ulnar', 198, -345, -64, 0.75],
    ['left anterior interosseous', 207, -365, -60, 0.3],
    ['left thoracoacromial', 136, 75, -27, 0.25],
    ['left thoracodorsal', 138, -65, -90, 0.2],
    ['left posterior interosseous', 217, -390, -75, 0.18],
    ['left superficial palmar arch', 209, -535, 26, 0.35],
    ['left deep palmar arch', 207, -519, 13, 0.35]
];
for (const [name, x, y, z, minimumClearance] of lumenSamples) {
    const point = { x, y, z };
    const contact = field.querySphere(point, 0);
    const packedLumen = field.packedLumenField.query(point);
    assert.ok(
        contact.inside && contact.signedDistance > 0,
        `${name} should be inside the generated lumen`
    );
    assert.ok(
        packedLumen.signedDistance >= minimumClearance,
        `${name} should retain ${minimumClearance} mm of lumen clearance`
    );
}

const digitalLeaves = upperLimbLeaves.filter(node => node.point.y < -650);
assert.equal(
    digitalLeaves.length,
    16,
    'eight proper digital outlets should remain on each hand'
);
const thumbLeaves = upperLimbLeaves.filter(node =>
    node.point.y < -620 &&
    node.point.y > -640 &&
    Math.abs(node.point.x) > 200
);
assert.equal(thumbLeaves.length, 4, 'two proper digital outlets should remain on each thumb');
for (const leaf of [...digitalLeaves, ...thumbLeaves]) {
    const sample = leaf.point.clone().lerp(leaf.neighbours[0], 0.4);
    const contact = field.querySphere(sample, 0);
    assert.ok(
        contact.inside && contact.signedDistance > 0.1,
        `digital branch at ${leaf.point.toArray()} should retain lumen before its terminal cap`
    );
}

const flowNetwork = new ContrastFlowNetwork(centerlineSegments);
const topology = flowNetwork.getTopologyDiagnostics();
assert.equal(topology.disconnectedSourceSegmentCount, 0);
assert.ok(
    topology.physiologicalTopologyRepair.repaired ||
        topology.physiologicalTopologyRepair.reason === 'no-carotid-attachment-gap',
    'the flow tree should either repair the carotid junction or already be continuous'
);

const distalFlowSamples = [
    ['right brachial', -185, -112, -62],
    ['right radial', -218, -345, -51],
    ['right ulnar', -198, -345, -64],
    ['left brachial', 179, -112, -62],
    ['left radial', 220, -345, -51],
    ['left ulnar', 198, -345, -64]
];
for (const [name, x, y, z] of distalFlowSamples) {
    const location = flowNetwork.findNearestLocation(
        new THREE.Vector3(x, y, z)
    );
    assert.ok(location.distance < 1, `${name} should have a nearby flow axis`);
    const edge = flowNetwork.edges[location.edgeIndex];
    assert.ok(
        edge.axis.y < -0.7,
        `${name} should carry blood distally (axis.y=${edge.axis.y})`
    );
}

console.log('upper-limb arterial anatomy passed', {
    segmentCount: centerlineSegments.length,
    rightLeaves: rightLeaves.length,
    leftLeaves: leftLeaves.length,
    lumenSamples: lumenSamples.length,
    digitalSamples: digitalLeaves.length + thumbLeaves.length
});
