import assert from 'node:assert/strict';
import * as THREE from 'three';
import {
    createUpperLimbArteryPaths,
    findUpperLimbAttachmentRoots,
    UPPER_LIMB_FINGER_DISTAL_Y_MM,
    UPPER_LIMB_HAND_DISTAL_Y_MM
} from '../src/upperLimbArteries.js';

function segment(id, nodeStartId, nodeEndId, start, end, radiusStart, radiusEnd) {
    return {
        id,
        nodeStartId,
        nodeEndId,
        start: new THREE.Vector3(...start),
        end: new THREE.Vector3(...end),
        radiusStart,
        radiusEnd
    };
}

const attachmentSegments = [
    segment(0, 0, 1, [0, 90, 0], [-138.66, 123.21, 13.5], 4, 1.82),
    segment(1, 0, 2, [0, 90, 0], [-139.65, 84.92, -11.36], 4, 0.6),
    segment(2, 0, 3, [0, 90, 0], [142.08, 97.99, -2.06], 4, 0.6),
    segment(3, 0, 4, [0, 90, 0], [139.61, 83.99, 0], 4, 0.6)
];
const roots = findUpperLimbAttachmentRoots(attachmentSegments);
assert.equal(roots[0].anatomicalSide, 'right');
assert.equal(roots[1].anatomicalSide, 'left');
assert.ok(roots[0].point.distanceTo(attachmentSegments[0].end) < 1e-9);
assert.ok(roots[1].point.distanceTo(attachmentSegments[2].end) < 1e-9);

const paths = createUpperLimbArteryPaths(roots);

function vessel(side, suffix) {
    const result = paths.find(path => path.name === `${side}-${suffix}`);
    assert.ok(result, `missing ${side}-${suffix}`);
    return result;
}

function samePosition(a, b, message) {
    assert.ok(
        a.position.distanceTo(b.position) < 1e-9,
        `${message}: ${a.position.toArray()} / ${b.position.toArray()}`
    );
}

for (const side of ['right', 'left']) {
    const sideSign = side === 'right' ? -1 : 1;
    const brachial = vessel(side, 'axillary-brachial');
    const deepBrachial = vessel(side, 'deep-brachial');
    const radial = vessel(side, 'radial');
    const ulnar = vessel(side, 'ulnar');
    const interosseous = vessel(side, 'common-interosseous');
    const superficialArch = vessel(side, 'superficial-palmar-arch');
    const deepArch = vessel(side, 'deep-palmar-arch');
    const thoracoacromial = vessel(side, 'thoracoacromial');
    const lateralThoracic = vessel(side, 'lateral-thoracic');
    const subscapular = vessel(side, 'subscapular-thoracodorsal');
    const circumflexScapular = vessel(side, 'circumflex-scapular');
    const superiorUlnarCollateral = vessel(side, 'superior-ulnar-collateral');
    const inferiorUlnarCollateral = vessel(side, 'inferior-ulnar-collateral');
    const radialRecurrent = vessel(side, 'radial-recurrent');
    const ulnarRecurrent = vessel(side, 'ulnar-recurrent');
    const posteriorInterosseous = vessel(side, 'posterior-interosseous');
    const princepsPollicis = vessel(side, 'princeps-pollicis');

    const brachialPointAtY = y => brachial.points.find(item =>
        item.position.y === y
    );

    samePosition(
        brachial.points.at(-1),
        radial.points[0],
        `${side} radial origin`
    );
    samePosition(
        brachial.points.at(-1),
        ulnar.points[0],
        `${side} ulnar origin`
    );
    samePosition(
        ulnar.points[1],
        interosseous.points[0],
        `${side} common interosseous origin`
    );
    samePosition(
        radial.points.at(-1),
        deepArch.points[0],
        `${side} deep palmar origin`
    );
    samePosition(
        ulnar.points.at(-1),
        superficialArch.points[0],
        `${side} superficial palmar origin`
    );
    samePosition(
        brachialPointAtY(64),
        thoracoacromial.points[0],
        `${side} thoracoacromial origin`
    );
    samePosition(
        brachialPointAtY(64),
        lateralThoracic.points[0],
        `${side} lateral thoracic origin`
    );
    samePosition(
        brachialPointAtY(64),
        subscapular.points[0],
        `${side} subscapular origin`
    );
    samePosition(
        subscapular.points[2],
        circumflexScapular.points[0],
        `${side} circumflex scapular origin`
    );
    samePosition(
        brachialPointAtY(-112),
        superiorUlnarCollateral.points[0],
        `${side} superior ulnar collateral origin`
    );
    samePosition(
        brachialPointAtY(-190),
        inferiorUlnarCollateral.points[0],
        `${side} inferior ulnar collateral origin`
    );
    samePosition(
        radial.points[1],
        radialRecurrent.points[0],
        `${side} radial recurrent origin`
    );
    samePosition(
        ulnar.points[1],
        ulnarRecurrent.points[0],
        `${side} ulnar recurrent origin`
    );
    samePosition(
        interosseous.points[1],
        posteriorInterosseous.points[0],
        `${side} posterior interosseous origin`
    );
    samePosition(
        deepArch.points[1],
        princepsPollicis.points[0],
        `${side} princeps pollicis origin`
    );

    assert.ok(
        sideSign * radial.points[2].position.x >
            sideSign * ulnar.points[2].position.x + 15,
        `${side} radial artery should remain lateral to the ulnar artery`
    );
    assert.ok(
        deepBrachial.points[2].position.z < brachial.points[5].position.z - 40,
        `${side} deep brachial artery should pass posterior to the brachial artery`
    );
    assert.ok(
        superficialArch.points.slice(1).every(item => item.position.z > 15),
        `${side} superficial arch should lie anterior to the hand skeleton`
    );
    assert.ok(
        deepArch.points.at(-1).position.x * sideSign <
            deepArch.points[1].position.x * sideSign,
        `${side} deep palmar arch should cross toward the ulnar side`
    );

    const digitalPaths = Array.from({ length: 4 }, (_, index) =>
        vessel(side, `common-palmar-digital-${index + 1}`)
    );
    for (const digital of digitalPaths) {
        assert.equal(digital.terminal, false);
        assert.equal(
            digital.points.at(-1).position.y,
            UPPER_LIMB_HAND_DISTAL_Y_MM
        );
        assert.ok(
            Math.abs(digital.points.at(-1).position.x) <
                Math.abs(digital.points[0].position.x),
            `${digital.name} should follow the distal hand toward the fingers`
        );
    }

    digitalPaths.forEach((digital, index) => {
        const radialProper = vessel(
            side,
            `proper-palmar-digital-${index + 1}-radial`
        );
        const ulnarProper = vessel(
            side,
            `proper-palmar-digital-${index + 1}-ulnar`
        );
        samePosition(
            digital.points.at(-1),
            radialProper.points[0],
            `${side} radial proper digital ${index + 1} origin`
        );
        samePosition(
            digital.points.at(-1),
            ulnarProper.points[0],
            `${side} ulnar proper digital ${index + 1} origin`
        );
        for (const proper of [radialProper, ulnarProper]) {
            assert.equal(proper.terminal, true);
            assert.equal(
                proper.points.at(-1).position.y,
                UPPER_LIMB_FINGER_DISTAL_Y_MM
            );
        }
        assert.ok(
            sideSign * radialProper.points.at(-1).position.x >
                sideSign * ulnarProper.points.at(-1).position.x,
            `${side} proper digital pair ${index + 1} should flank its finger`
        );
    });

    for (const surface of ['radial', 'ulnar']) {
        const thumb = vessel(
            side,
            `proper-palmar-digital-thumb-${surface}`
        );
        samePosition(
            princepsPollicis.points.at(-1),
            thumb.points[0],
            `${side} proper thumb ${surface} origin`
        );
        assert.equal(thumb.terminal, true);
    }
}

assert.equal(paths.length, 66, 'thirty-three named arterial paths should be generated per arm');
console.log('upper-limb skeletal path definitions passed');
