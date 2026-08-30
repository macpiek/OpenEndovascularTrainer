import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import * as THREE from 'three';
import { AORTA_TRANSFORM_VERSION } from '../src/aortaTransform.js';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { HEAD_ARTERY_SUPERIOR_Y_MM } from '../src/headArteries.js';
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
assert.ok(
    asset.metadata.bounds.max[1] > HEAD_ARTERY_SUPERIOR_Y_MM,
    'STL should reach the cerebral arteries'
);
assert.ok(
    asset.metadata.bounds.max[1] < 380,
    'cerebral arteries should remain inside the cranial vault'
);
assert.ok(asset.metadata.bounds.min[1] < -1380, 'existing plantar arches must remain in place');
assert.ok(asset.metadata.centerline.segmentCount > 5600);

const diagnostics = asset.metadata.centerline.diagnostics;
assert.equal(diagnostics.componentCount, 1);
assert.equal(diagnostics.centerlineGraphCycleCount, 0);
assert.equal(diagnostics.centerlineInvalidSegmentCountFinal, 0);
assert.equal(
    diagnostics.centerlineTopologyAfterCleanup.severeBacktrackNodeCount,
    0
);

const packedSegments = asset.arrays.centerlineSegments;
const packedEdges = asset.arrays.centerlineEdges;
const centerlineStride = asset.metadata.centerline.stride;
const centerlineSegments = [];
for (let index = 0; index < packedSegments.length / centerlineStride; index++) {
    const offset = index * centerlineStride;
    centerlineSegments.push({
        id: index,
        start: new THREE.Vector3(
            packedSegments[offset],
            packedSegments[offset + 1],
            packedSegments[offset + 2]
        ),
        end: new THREE.Vector3(
            packedSegments[offset + 3],
            packedSegments[offset + 4],
            packedSegments[offset + 5]
        ),
        radiusStart: packedSegments[offset + 6],
        radiusEnd: packedSegments[offset + 7],
        safeRadius: packedSegments[offset + 8],
        nodeStartId: packedEdges[index * 2],
        nodeEndId: packedEdges[index * 2 + 1]
    });
}

const flowNetwork = new ContrastFlowNetwork(centerlineSegments);
const flowTopology = flowNetwork.getTopologyDiagnostics();
assert.ok(
    flowTopology.physiologicalTopologyRepair.repaired ||
        flowTopology.physiologicalTopologyRepair.reason ===
            'no-carotid-attachment-gap',
    'the full arterial tree should repair the carotid attachment or already be continuous'
);
assert.equal(
    flowTopology.disconnectedSourceSegmentCount,
    0,
    'the repaired arterial flow graph should remain fully connected'
);
assert.ok(
    flowNetwork.rootNode.point.distanceTo(new THREE.Vector3(0, 0, 0)) < 20,
    'arterial flow should start at the aortic inlet near the heart'
);

function assertCranialFlow(name, point, minimumYDirection = 0.8) {
    const location = flowNetwork.findNearestLocation(point);
    assert.ok(location, `${name} should have a centerline flow location`);
    const edge = flowNetwork.edges[location.edgeIndex];
    assert.ok(
        edge.axis.y >= minimumYDirection,
        `${name} should carry blood cranially (axis.y=${edge.axis.y})`
    );
}

assertCranialFlow(
    'initial ascending aorta',
    new THREE.Vector3(9, 3, 12),
    0.8
);
assertCranialFlow(
    'left common carotid',
    new THREE.Vector3(10, 160.45, -1)
);
assertCranialFlow(
    'left internal carotid',
    new THREE.Vector3(15, 257.2, -28)
);

const field = new VesselContactField(asset);
const headLumenSamples = [
    ['right common carotid', -30, 160.45, -1, 1],
    ['left common carotid', 10, 160.45, -1, 1],
    ['right carotid bifurcation', -32, 205.45, 0, 1],
    ['left carotid bifurcation', 12, 205.45, 0, 1],
    ['right cervical ICA', -35, 257.2, -28, 0.8],
    ['left cervical ICA', 15, 257.2, -28, 0.8],
    ['right vertebral V2', -32, 182.95, -35, 0.5],
    ['left vertebral V2', 12, 182.95, -35, 0.5],
    ['vertebrobasilar junction', -10, 304.45, -22, 0.7],
    ['basilar artery', -10, 313.45, -18, 0.8],
    ['anterior communicating artery', -10, 320.2, 30, 0.35],
    ['right posterior communicating artery', -30, 320.2, 7, 0.35],
    ['left posterior communicating artery', 10, 320.2, 7, 0.35],
    ['right ACA A2', -15, 333.7, 38, 0.45],
    ['left ACA A2', -5, 333.7, 38, 0.45],
    ['right MCA M1', -52, 319.3, 28, 0.55],
    ['left MCA M1', 32, 319.3, 28, 0.55],
    ['right PCA P2', -52, 334.6, -35, 0.4],
    ['left PCA P2', 32, 334.6, -35, 0.4]
];

for (const [name, x, y, z, minimumClearance] of headLumenSamples) {
    const contact = field.querySphere({ x, y, z }, 0);
    const packedLumen = field.packedLumenField.query({ x, y, z });
    assert.ok(
        contact.inside && contact.signedDistance > 0,
        `${name} should be inside the generated lumen`
    );
    assert.ok(
        packedLumen.signedDistance >= minimumClearance,
        `${name} should retain ${minimumClearance} mm of lumen clearance`
    );
}

console.log('head arterial lumen samples passed', {
    samples: headLumenSamples.length,
    bounds: asset.metadata.bounds,
    segmentCount: asset.metadata.centerline.segmentCount
});
