import assert from 'node:assert/strict';
import * as THREE from 'three';
import {
    BRAIN_MIDLINE_X_MM,
    createHeadArteryPaths,
    HEAD_ARTERY_SUPERIOR_Y_MM
} from '../src/headArteries.js';

const roots = [
    {
        anatomicalSide: 'right',
        sideSign: -1,
        carotid: {
            point: new THREE.Vector3(-18.6, 124, -4.4),
            radius: 1.86,
            outward: new THREE.Vector3(0, 1, 0)
        },
        vertebral: {
            point: new THREE.Vector3(-17.9, 124.9, -18),
            radius: 0.6,
            outward: new THREE.Vector3(0, 1, 0)
        }
    },
    {
        anatomicalSide: 'left',
        sideSign: 1,
        carotid: {
            point: new THREE.Vector3(20.5, 123.1, -4.9),
            radius: 2.16,
            outward: new THREE.Vector3(0, 1, 0)
        },
        vertebral: {
            point: new THREE.Vector3(26.2, 123.2, -19.1),
            radius: 0.88,
            outward: new THREE.Vector3(0, 1, 0)
        }
    }
];
const paths = createHeadArteryPaths(roots);

function vessel(name) {
    const result = paths.find(path => path.name === name);
    assert.ok(result, `missing path ${name}`);
    return result;
}

function samePosition(a, b, message) {
    assert.ok(
        a.position.distanceTo(b.position) < 1e-9,
        `${message}: ${a.position.toArray()} / ${b.position.toArray()}`
    );
}

for (const side of ['right', 'left']) {
    const common = vessel(`${side}-common-carotid`);
    const external = vessel(`${side}-external-carotid`);
    const internal = vessel(`${side}-internal-carotid`);
    const acaA1 = vessel(`${side}-anterior-cerebral-a1`);
    const acaDistal = vessel(`${side}-anterior-cerebral-distal`);
    const mca = vessel(`${side}-middle-cerebral-main`);
    const mcaInferior = vessel(`${side}-middle-cerebral-inferior`);
    const pcom = vessel(`${side}-posterior-communicating`);
    const pcaP1 = vessel(`${side}-posterior-cerebral-p1`);
    const pca = vessel(`${side}-posterior-cerebral-main`);
    const vertebral = vessel(`${side}-vertebral`);

    samePosition(common.points.at(-1), external.points[0], `${side} ECA origin`);
    samePosition(common.points.at(-1), internal.points[0], `${side} ICA origin`);
    samePosition(internal.points.at(-1), acaA1.points[0], `${side} ACA origin`);
    samePosition(internal.points.at(-1), mca.points[0], `${side} MCA origin`);
    samePosition(internal.points.at(-1), pcom.points[0], `${side} PCom origin`);
    samePosition(acaA1.points.at(-1), acaDistal.points[0], `${side} ACA A2`);
    samePosition(mca.points[1], mcaInferior.points[0], `${side} MCA division`);
    samePosition(pcaP1.points.at(-1), pcom.points.at(-1), `${side} PCom-PCA`);
    samePosition(pcaP1.points.at(-1), pca.points[0], `${side} PCA P2`);

    assert.ok(
        external.points[1].position.z > internal.points[1].position.z + 20,
        `${side} ECA should course anterior to the cervical ICA`
    );
    assert.ok(
        vertebral.points[5].position.z < internal.points[2].position.z - 10,
        `${side} vertebral artery should remain posterior to the ICA in the neck`
    );
}

const rightAca = vessel('right-anterior-cerebral-a1');
const leftAca = vessel('left-anterior-cerebral-a1');
const acom = vessel('anterior-communicating');
samePosition(rightAca.points.at(-1), acom.points[0], 'right ACom junction');
samePosition(leftAca.points.at(-1), acom.points.at(-1), 'left ACom junction');

const rightVertebral = vessel('right-vertebral');
const leftVertebral = vessel('left-vertebral');
const basilar = vessel('basilar');
samePosition(rightVertebral.points.at(-1), leftVertebral.points.at(-1), 'vertebral union');
samePosition(rightVertebral.points.at(-1), basilar.points[0], 'basilar origin');

const rightP1 = vessel('right-posterior-cerebral-p1');
const leftP1 = vessel('left-posterior-cerebral-p1');
samePosition(basilar.points.at(-1), rightP1.points[0], 'right PCA origin');
samePosition(basilar.points.at(-1), leftP1.points[0], 'left PCA origin');

for (const side of ['right', 'left']) {
    const sign = side === 'right' ? -1 : 1;
    for (const suffix of [
        'anterior-cerebral-distal',
        'middle-cerebral-main',
        'posterior-cerebral-main'
    ]) {
        const terminal = vessel(`${side}-${suffix}`).points.at(-1).position;
        assert.equal(
            Math.sign(terminal.x - BRAIN_MIDLINE_X_MM),
            sign,
            `${side} ${suffix} should remain on its cerebral hemisphere`
        );
        assert.ok(terminal.y >= 343, `${side} ${suffix} should reach the brain`);
        assert.ok(
            terminal.y <= HEAD_ARTERY_SUPERIOR_Y_MM,
            `${side} ${suffix} should remain inside the cranial vault`
        );
    }
}

assert.equal(paths.length, 28);
console.log('head arterial path definitions passed');
