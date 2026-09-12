import assert from 'node:assert/strict';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import {
    buildCenterlineFlowArrowSamples,
    createBroadPhaseDebugGroup
} from '../src/vesselBroadPhase.js';
import {
    findAorticArchDebugAnchors,
    findAortoiliacDebugAnchors
} from '../src/anatomyDebugLabels.js';

function segment(id, nodeStartId, nodeEndId, start, end, radiusStart, radiusEnd) {
    const startPoint = new THREE.Vector3(...start);
    const endPoint = new THREE.Vector3(...end);
    const axis = endPoint.clone().sub(startPoint);
    const length = axis.length();
    return {
        id,
        nodeStartId,
        nodeEndId,
        start: startPoint,
        end: endPoint,
        axis: axis.multiplyScalar(1 / length),
        length,
        radiusStart,
        radiusEnd,
        safeRadius: Math.min(radiusStart, radiusEnd)
    };
}

const centerlineSegments = [
    segment('aorta', 0, 1, [0, 0, 0], [0, 12, 0], 6, 5),
    segment('left-branch', 1, 2, [0, 12, 0], [-8, 20, 0], 4, 3),
    segment('right-branch', 1, 3, [0, 12, 0], [8, 20, 0], 4, 3)
];
const network = new ContrastFlowNetwork(centerlineSegments, {
    rootPoint: new THREE.Vector3(0, 0, 0)
});
const arrowOptions = {
    spacing: 4,
    offset: 2
};
const samples = buildCenterlineFlowArrowSamples(network.edges, arrowOptions);

assert.ok(samples.length >= 7, 'debug centerline should contain frequent, small flow arrows');
assert.equal(
    new Set(samples.map(sample => sample.edgeIndex)).size,
    network.edges.length,
    'flow arrows should continue into every arterial branch'
);

const flowEdgesWithHiddenConnector = network.edges.map(edge => ({ ...edge }));
flowEdgesWithHiddenConnector[1].renderExcluded = true;
const visibleSamples = buildCenterlineFlowArrowSamples(
    flowEdgesWithHiddenConnector,
    arrowOptions
);
assert.ok(
    visibleSamples.every(sample => sample.edgeIndex !== 1),
    'flow arrows should not reveal hidden topology-repair connectors'
);

for (const sample of samples) {
    const edge = network.edges[sample.edgeIndex];
    assert.ok(
        sample.direction.dot(edge.axis) > 0.999999,
        'every arrow must point in the directed heart-to-periphery flow direction'
    );
    const projected = sample.position.clone().sub(edge.start).dot(edge.axis);
    assert.ok(
        projected >= -1e-8 && projected <= edge.length + 1e-8,
        'every flow arrow should lie on its centerline edge'
    );
}

const debugGroup = createBroadPhaseDebugGroup(
    { segments: centerlineSegments },
    {
        flowEdges: network.edges,
        flowArrowOptions: arrowOptions
    }
);
assert.equal(
    debugGroup.userData.centerlineFlowArrowCount,
    samples.length,
    'debug metadata should expose the number of rendered flow arrows'
);
assert.equal(
    debugGroup.userData.centerlineFlowDirection,
    'heart-to-periphery',
    'debug metadata should identify the physiological arrow direction'
);

const arrowHeads = debugGroup.children
    .flatMap(child => child.children || [])
    .find(child => child.isInstancedMesh && child.userData.flowDirection === 'heart-to-periphery');
assert.ok(arrowHeads, 'debug centerline should render instanced arrowheads');
assert.equal(arrowHeads.count, samples.length, 'every sampled direction should have an arrowhead');
assert.equal(
    arrowHeads.userData.debugLayer,
    'centerline',
    'flow arrows should follow the existing centerline debug toggle'
);

const instanceMatrix = new THREE.Matrix4();
const instanceDirection = new THREE.Vector3();
for (let index = 0; index < arrowHeads.count; index++) {
    arrowHeads.getMatrixAt(index, instanceMatrix);
    instanceDirection.setFromMatrixColumn(instanceMatrix, 1).normalize();
    assert.ok(
        instanceDirection.dot(samples[index].direction) > 0.999999,
        'rendered arrowhead orientation must match its sampled flow direction'
    );
}

console.log('centerline flow arrows', samples.length);
console.log('centerline flow branches covered', network.edges.length);

const aortoiliacSegments = [
    segment('aorta', 0, 1, [0, -240, 0], [0, -290, 0], 9, 8),
    segment('left-iliac', 1, 2, [0, -290, 0], [-40, -340, 0], 7, 6),
    segment('right-iliac', 1, 3, [0, -290, 0], [40, -340, 0], 7, 6),
    segment('right-external-iliac', 2, 4, [-40, -340, 0], [-68, -400, 0], 5.5, 4.5),
    segment('right-internal-iliac', 2, 5, [-40, -340, 0], [-24, -372, 0], 3.8, 3.2)
];
const aortoiliacNetwork = new ContrastFlowNetwork(aortoiliacSegments, {
    rootPoint: new THREE.Vector3(0, -240, 0)
});
const anatomyAnchors = findAortoiliacDebugAnchors(aortoiliacNetwork);
assert.ok(anatomyAnchors, 'debug labels should locate the aortoiliac bifurcation');
assert.ok(
    anatomyAnchors.leftIliac.x > anatomyAnchors.rightIliac.x,
    'left and right labels must follow the anatomical patient-side convention'
);
assert.ok(
    anatomyAnchors.aorta.y > -290,
    'the aorta label should be anchored above the bifurcation'
);
assert.equal(
    aortoiliacNetwork.edges[
        anatomyAnchors.rightInternalIliacRootEdgeIndex
    ].sourceIndex,
    4,
    'the lower-flow pelvic branch should be identified as the patient-right internal iliac artery'
);
assert.ok(
    anatomyAnchors.rightInternalIliac.x > -40,
    'the patient-right internal iliac marker should lie on the medial pelvic branch'
);

const archSegments = [
    segment('ascending-aorta', 0, 1, [0, 0, 0], [0, 10, 0], 12, 12),
    segment('arch-1', 1, 2, [0, 10, 0], [0, 20, 0], 11, 11),
    segment('brachiocephalic', 1, 10, [0, 10, 0], [-5, 14, 0], 5, 5),
    segment('right-carotid', 10, 11, [-5, 14, 0], [-8, 30, 0], 3.5, 3),
    segment('right-subclavian', 10, 12, [-5, 14, 0], [-20, 17, 0], 3.5, 3),
    segment('arch-2', 2, 3, [0, 20, 0], [0, 30, 0], 10, 10),
    segment('left-common-carotid', 2, 13, [0, 20, 0], [8, 55, 0], 4, 3),
    segment('descending-aorta', 3, 4, [0, 30, 0], [0, -290, 0], 9, 8),
    segment('left-subclavian', 3, 14, [0, 30, 0], [24, 42, 0], 4.5, 3.5),
    segment('left-iliac', 4, 5, [0, -290, 0], [-40, -340, 0], 7, 6),
    segment('right-iliac', 4, 6, [0, -290, 0], [40, -340, 0], 7, 6)
];
const archNetwork = new ContrastFlowNetwork(archSegments, {
    rootPoint: new THREE.Vector3(0, 0, 0)
});
const archAnchors = findAorticArchDebugAnchors(archNetwork);
assert.ok(archAnchors, 'debug labels should locate all three aortic arch branches');
assert.equal(
    archNetwork.edges[archAnchors.brachiocephalicTrunkRootEdgeIndex].sourceIndex,
    2,
    'the first supra-aortic branch should be the brachiocephalic trunk'
);
assert.equal(
    archNetwork.edges[archAnchors.leftCommonCarotidRootEdgeIndex].sourceIndex,
    6,
    'the second supra-aortic branch should be the left common carotid'
);
assert.equal(
    archNetwork.edges[archAnchors.leftSubclavianRootEdgeIndex].sourceIndex,
    8,
    'the third supra-aortic branch should be the left subclavian'
);
assert.ok(
    archAnchors.brachiocephalicTrunk.distanceTo(
        archNetwork.edges[archAnchors.brachiocephalicTrunkEdgeIndex].start
    ) < archNetwork.edges[archAnchors.brachiocephalicTrunkEdgeIndex].length,
    'the brachiocephalic label should remain on the trunk before its bifurcation'
);
assert.ok(
    archAnchors.leftCommonCarotid.y >
        archNetwork.edges[archAnchors.leftCommonCarotidRootEdgeIndex].start.y,
    'the left common carotid marker should lie on the vessel, beyond the arch'
);
assert.ok(
    archAnchors.leftSubclavian.x >
        archNetwork.edges[archAnchors.leftSubclavianRootEdgeIndex].start.x,
    'the left subclavian marker should lie on the vessel, beyond the arch'
);

const extractedArchSegments = [
    segment('ascending-aorta', 0, 1, [0, 0, 0], [0, 12, 0], 12, 12),
    segment('arch-1', 1, 2, [0, 12, 0], [0, 24, 0], 11, 11),
    segment('shared-supra-aortic-connector', 1, 10, [0, 12, 0], [1, 13, 0], 11, 10.5),
    segment('brachiocephalic', 10, 11, [1, 13, 0], [-8, 46, 0], 6, 5),
    segment('right-carotid', 11, 12, [-8, 46, 0], [-10, 72, 0], 3.5, 3),
    segment('right-subclavian', 11, 13, [-8, 46, 0], [-35, 50, 0], 4, 3),
    segment('left-common-carotid', 10, 14, [1, 13, 0], [12, 80, 0], 6, 3),
    segment('arch-2', 2, 3, [0, 24, 0], [0, 36, 0], 10, 10),
    segment('left-subclavian', 2, 15, [0, 24, 0], [35, 76, 0], 5, 3.5),
    segment('arch-centerline-artifact', 3, 16, [0, 36, 0], [28, 40, -35], 9, 8),
    segment('descending-aorta', 3, 4, [0, 36, 0], [0, -290, 0], 9, 8),
    segment('left-iliac', 4, 5, [0, -290, 0], [-40, -340, 0], 7, 6),
    segment('right-iliac', 4, 6, [0, -290, 0], [40, -340, 0], 7, 6)
];
const extractedArchNetwork = new ContrastFlowNetwork(extractedArchSegments, {
    rootPoint: new THREE.Vector3(0, 0, 0)
});
const extractedArchAnchors = findAorticArchDebugAnchors(extractedArchNetwork);
assert.ok(
    extractedArchAnchors,
    'debug labels should handle the shared connector produced by centerline extraction'
);
assert.equal(
    extractedArchNetwork.edges[
        extractedArchAnchors.brachiocephalicTrunkRootEdgeIndex
    ].sourceIndex,
    3,
    'the connector child on patient-right should be identified as the brachiocephalic trunk'
);
assert.equal(
    extractedArchNetwork.edges[
        extractedArchAnchors.leftCommonCarotidRootEdgeIndex
    ].sourceIndex,
    6,
    'the other connector child should be identified as the left common carotid'
);
assert.equal(
    extractedArchNetwork.edges[
        extractedArchAnchors.leftSubclavianRootEdgeIndex
    ].sourceIndex,
    8,
    'the next true side branch should be identified as the left subclavian, not an aortic artifact'
);
assert.ok(
    extractedArchAnchors.brachiocephalicTrunk.y > 30,
    'the brachiocephalic marker should be placed on the trunk rather than on the arch'
);
assert.ok(
    extractedArchAnchors.leftCommonCarotid.y > 60,
    'the left common carotid marker should be placed on the artery rather than on the arch'
);
assert.ok(
    extractedArchAnchors.leftSubclavian.y > 55,
    'the left subclavian marker should be placed on the artery rather than on the arch'
);
