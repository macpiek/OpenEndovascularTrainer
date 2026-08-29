import * as THREE from 'three';

const BRAIN_MIDLINE_X_MM = -10;
const MINIMUM_CAROTID_RADIUS_MM = 1.5;
const MINIMUM_VERTEBRAL_RADIUS_MM = 0.5;
const MAXIMUM_VERTEBRAL_RADIUS_MM = 1.49;
const SUPERIOR_ATTACHMENT_Y_MM = 120;
export const HEAD_ARTERY_Y_ANCHOR_MM = 124;
export const HEAD_ARTERY_Y_SCALE = 0.45;
export const HEAD_ARTERY_CIRCLE_Y_MM = HEAD_ARTERY_Y_ANCHOR_MM +
    (560 - HEAD_ARTERY_Y_ANCHOR_MM) * HEAD_ARTERY_Y_SCALE;
export const HEAD_ARTERY_SUPERIOR_Y_MM = HEAD_ARTERY_Y_ANCHOR_MM +
    (675 - HEAD_ARTERY_Y_ANCHOR_MM) * HEAD_ARTERY_Y_SCALE;

function endpoint(segment, atStart) {
    const point = (atStart ? segment.start : segment.end).clone();
    const neighbour = (atStart ? segment.end : segment.start).clone();
    return {
        nodeId: atStart ? segment.nodeStartId : segment.nodeEndId,
        point,
        neighbour,
        radius: atStart ? segment.radiusStart : segment.radiusEnd,
        segmentId: segment.id,
        outward: point.clone().sub(neighbour).normalize()
    };
}

function terminalLeaves(centerlineSegments) {
    const degree = new Map();
    for (const segment of centerlineSegments) {
        degree.set(
            segment.nodeStartId,
            (degree.get(segment.nodeStartId) || 0) + 1
        );
        degree.set(
            segment.nodeEndId,
            (degree.get(segment.nodeEndId) || 0) + 1
        );
    }

    const leaves = [];
    for (const segment of centerlineSegments) {
        if (degree.get(segment.nodeStartId) === 1) {
            leaves.push(endpoint(segment, true));
        }
        if (degree.get(segment.nodeEndId) === 1) {
            leaves.push(endpoint(segment, false));
        }
    }
    return leaves;
}

function centerlineAdjacency(centerlineSegments) {
    const adjacency = new Map();
    for (const segment of centerlineSegments) {
        const endpoints = [
            [segment.nodeStartId, segment.nodeEndId, true],
            [segment.nodeEndId, segment.nodeStartId, false]
        ];
        for (const [nodeId, otherNodeId, atStart] of endpoints) {
            const edges = adjacency.get(nodeId) || [];
            edges.push({ segment, otherNodeId, atStart });
            adjacency.set(nodeId, edges);
        }
    }
    return adjacency;
}

function traceInward(root, adjacency, minimumDistance = 16) {
    const samples = [{
        position: root.point.clone(),
        radius: root.radius
    }];
    let currentNodeId = root.nodeId;
    let previousNodeId = null;
    let distance = 0;

    while (distance < minimumDistance) {
        const edges = adjacency.get(currentNodeId) || [];
        const edge = previousNodeId === null
            ? edges.find(item => item.segment.id === root.segmentId)
            : edges.find(item => item.otherNodeId !== previousNodeId);
        if (!edge || (previousNodeId !== null && edges.length !== 2)) break;

        const { segment, atStart, otherNodeId } = edge;
        const nextPosition = (atStart ? segment.end : segment.start).clone();
        const nextRadius = atStart
            ? segment.radiusEnd
            : segment.radiusStart;
        distance += samples.at(-1).position.distanceTo(nextPosition);
        samples.push({ position: nextPosition, radius: nextRadius });
        previousNodeId = currentNodeId;
        currentNodeId = otherNodeId;
    }

    return samples.reverse();
}

function bilateralRoots(candidates, vesselName) {
    if (candidates.length !== 2) {
        throw new Error(
            `Expected two ${vesselName} attachment roots, found ${candidates.length}`
        );
    }
    candidates.sort((a, b) => a.point.x - b.point.x);
    return candidates.map((root, index) => ({
        ...root,
        anatomicalSide: index === 0 ? 'right' : 'left',
        sideSign: index === 0 ? -1 : 1
    }));
}

/**
 * Finds the paired terminal common carotid and short vertebral stubs already
 * present in the supra-aortic STL. Patient-right is -X in the imported model.
 */
export function findHeadArteryAttachmentRoots(centerlineSegments) {
    const leaves = terminalLeaves(centerlineSegments);
    const adjacency = centerlineAdjacency(centerlineSegments);
    const centralSuperiorLeaves = leaves.filter(leaf =>
        leaf.point.y >= SUPERIOR_ATTACHMENT_Y_MM &&
        Math.abs(leaf.point.x) < 35
    );
    const carotids = bilateralRoots(
        centralSuperiorLeaves.filter(leaf =>
            leaf.radius >= MINIMUM_CAROTID_RADIUS_MM &&
            leaf.point.z > -12
        ),
        'common carotid'
    );
    const vertebrals = bilateralRoots(
        centralSuperiorLeaves.filter(leaf =>
            leaf.radius >= MINIMUM_VERTEBRAL_RADIUS_MM &&
            leaf.radius <= MAXIMUM_VERTEBRAL_RADIUS_MM &&
            leaf.point.z < -12
        ),
        'vertebral'
    );

    return carotids.map((carotid, index) => ({
        anatomicalSide: carotid.anatomicalSide,
        sideSign: carotid.sideSign,
        carotid: {
            ...carotid,
            inwardTrace: traceInward(carotid, adjacency)
        },
        vertebral: {
            ...vertebrals[index],
            inwardTrace: traceInward(vertebrals[index], adjacency)
        }
    }));
}

function point(x, y, z, radius) {
    return { position: new THREE.Vector3(x, y, z), radius };
}

function headPoint(x, y, z, radius) {
    return point(
        x,
        HEAD_ARTERY_Y_ANCHOR_MM +
            (y - HEAD_ARTERY_Y_ANCHOR_MM) * HEAD_ARTERY_Y_SCALE,
        z,
        radius
    );
}

function clonedPoint(source, radius = source.radius) {
    return point(
        source.position.x,
        source.position.y,
        source.position.z,
        radius
    );
}

function rootPoint(root, distance, radius) {
    const position = root.point.clone().addScaledVector(root.outward, distance);
    return point(position.x, position.y, position.z, radius);
}

function inletPoints(root, fallbackDistance, inletRadius) {
    if (!root.inwardTrace?.length) {
        return [
            rootPoint(root, fallbackDistance, inletRadius * 0.68),
            point(root.point.x, root.point.y, root.point.z, inletRadius)
        ];
    }
    return root.inwardTrace.map((sample, index, samples) => point(
        sample.position.x,
        sample.position.y,
        sample.position.z,
        index === samples.length - 1
            ? inletRadius
            : Math.min(inletRadius * 0.78, sample.radius * 0.72)
    ));
}

function lateral(sideSign, distanceFromMidline) {
    return BRAIN_MIDLINE_X_MM + sideSign * distanceFromMidline;
}

function path(
    name,
    anatomicalSide,
    vesselNames,
    points,
    terminal = false,
    outerStartIndex = 0
) {
    return {
        name,
        anatomicalSide,
        vesselNames,
        points,
        terminal,
        outerStartIndex
    };
}

/**
 * Defines a continuous bilateral head-and-neck arterial network in simulator
 * millimetres. The complete Circle of Willis is represented by the ACom and
 * paired PCom connections between the internal-carotid and vertebrobasilar
 * circulations. Distal ACA, MCA and PCA paths follow the medial, lateral and
 * posterior cerebral surfaces respectively.
 */
export function createHeadArteryPaths(attachmentRoots) {
    if (attachmentRoots.length !== 2) {
        throw new Error('Two paired head-artery attachment roots are required');
    }

    const paths = [];
    const shared = new Map();

    for (const roots of attachmentRoots) {
        const { anatomicalSide: side, sideSign, carotid, vertebral } = roots;
        const carotidInletRadius = THREE.MathUtils.clamp(
            carotid.radius * 1.08,
            1.9,
            2.45
        );
        const vertebralInletRadius = THREE.MathUtils.clamp(
            vertebral.radius * 1.12,
            0.78,
            1.12
        );
        const carotidEntry = rootPoint(carotid, 12, carotidInletRadius);
        const carotidInlet = inletPoints(carotid, -14, carotidInletRadius);
        const carotidBifurcation = headPoint(lateral(sideSign, 22), 305, 0, 2.25);
        const internalCarotidTerminus = headPoint(
            lateral(sideSign, 20),
            550,
            20,
            1.85
        );
        const anteriorCommunicatingJunction = headPoint(
            lateral(sideSign, 4),
            560,
            30,
            1.22
        );
        const anteriorCerebralBranch = headPoint(
            lateral(sideSign, 6),
            630,
            45,
            1.02
        );
        const middleCerebralBranch = headPoint(
            lateral(sideSign, 42),
            558,
            28,
            1.42
        );
        const posteriorCerebralJunction = headPoint(
            lateral(sideSign, 20),
            572,
            -8,
            1.28
        );
        const posteriorCerebralBranch = headPoint(
            lateral(sideSign, 42),
            592,
            -35,
            1.02
        );
        const vertebralEntry = rootPoint(vertebral, 10, vertebralInletRadius);
        const vertebralInlet = inletPoints(vertebral, -12, vertebralInletRadius);
        const vertebrobasilarJunction = headPoint(
            BRAIN_MIDLINE_X_MM,
            525,
            -22,
            1.45
        );

        paths.push(path(
            `${side}-common-carotid`,
            side,
            ['common-carotid'],
            [
                ...carotidInlet,
                carotidEntry,
                headPoint(lateral(sideSign, 20), 205, -1, 2.3),
                headPoint(lateral(sideSign, 21), 270, 0, 2.3),
                carotidBifurcation
            ],
            false,
            carotidInlet.length - 1
        ));
        paths.push(path(
            `${side}-external-carotid`,
            side,
            ['external-carotid'],
            [
                clonedPoint(carotidBifurcation, 1.72),
                headPoint(lateral(sideSign, 27), 355, 14, 1.55),
                headPoint(lateral(sideSign, 33), 415, 29, 1.25),
                headPoint(lateral(sideSign, 38), 468, 42, 0.88)
            ],
            true
        ));
        paths.push(path(
            `${side}-internal-carotid`,
            side,
            ['internal-carotid-cervical', 'internal-carotid-siphon'],
            [
                clonedPoint(carotidBifurcation, 1.95),
                headPoint(lateral(sideSign, 24), 350, -12, 1.95),
                headPoint(lateral(sideSign, 25), 420, -28, 1.92),
                headPoint(lateral(sideSign, 18), 485, -30, 1.88),
                headPoint(lateral(sideSign, 16), 510, -9, 1.85),
                headPoint(lateral(sideSign, 14), 523, 16, 1.82),
                headPoint(lateral(sideSign, 17), 535, 29, 1.82),
                internalCarotidTerminus
            ]
        ));
        paths.push(path(
            `${side}-anterior-cerebral-a1`,
            side,
            ['anterior-cerebral-a1'],
            [internalCarotidTerminus, anteriorCommunicatingJunction]
        ));
        paths.push(path(
            `${side}-anterior-cerebral-distal`,
            side,
            ['anterior-cerebral-a2', 'pericallosal'],
            [
                clonedPoint(anteriorCommunicatingJunction, 1.08),
                headPoint(lateral(sideSign, 5), 590, 38, 1.1),
                anteriorCerebralBranch,
                headPoint(lateral(sideSign, 7), 675, 35, 0.76)
            ],
            true
        ));
        paths.push(path(
            `${side}-callosomarginal`,
            side,
            ['callosomarginal'],
            [
                clonedPoint(anteriorCerebralBranch, 0.88),
                headPoint(lateral(sideSign, 22), 650, 52, 0.78),
                headPoint(lateral(sideSign, 30), 674, 58, 0.62)
            ],
            true
        ));
        paths.push(path(
            `${side}-middle-cerebral-main`,
            side,
            ['middle-cerebral-m1', 'middle-cerebral-m2-superior'],
            [
                clonedPoint(internalCarotidTerminus, 1.55),
                middleCerebralBranch,
                headPoint(lateral(sideSign, 58), 574, 38, 1.08),
                headPoint(lateral(sideSign, 66), 612, 38, 0.72)
            ],
            true
        ));
        paths.push(path(
            `${side}-middle-cerebral-inferior`,
            side,
            ['middle-cerebral-m2-inferior'],
            [
                clonedPoint(middleCerebralBranch, 1.08),
                headPoint(lateral(sideSign, 58), 548, 49, 0.9),
                headPoint(lateral(sideSign, 68), 568, 60, 0.66)
            ],
            true
        ));
        paths.push(path(
            `${side}-posterior-communicating`,
            side,
            ['posterior-communicating'],
            [
                clonedPoint(internalCarotidTerminus, 0.82),
                headPoint(lateral(sideSign, 20), 560, 7, 0.78),
                clonedPoint(posteriorCerebralJunction, 0.82)
            ]
        ));
        paths.push(path(
            `${side}-posterior-cerebral-main`,
            side,
            ['posterior-cerebral-p2', 'posterior-cerebral-p3'],
            [
                clonedPoint(posteriorCerebralJunction, 1.18),
                posteriorCerebralBranch,
                headPoint(lateral(sideSign, 55), 620, -45, 0.86),
                headPoint(lateral(sideSign, 62), 655, -46, 0.64)
            ],
            true
        ));
        paths.push(path(
            `${side}-parieto-occipital`,
            side,
            ['parieto-occipital'],
            [
                clonedPoint(posteriorCerebralBranch, 0.84),
                headPoint(lateral(sideSign, 48), 625, -22, 0.72),
                headPoint(lateral(sideSign, 50), 663, -12, 0.58)
            ],
            true
        ));
        paths.push(path(
            `${side}-vertebral`,
            side,
            ['vertebral-v1', 'vertebral-v2', 'vertebral-v3', 'vertebral-v4'],
            [
                ...vertebralInlet,
                vertebralEntry,
                headPoint(lateral(sideSign, 20), 185, -24, 1.12),
                headPoint(lateral(sideSign, 22), 255, -35, 1.15),
                headPoint(lateral(sideSign, 24), 330, -46, 1.16),
                headPoint(lateral(sideSign, 22), 405, -48, 1.18),
                headPoint(lateral(sideSign, 18), 465, -42, 1.2),
                headPoint(lateral(sideSign, 12), 505, -30, 1.25),
                vertebrobasilarJunction
            ],
            false,
            vertebralInlet.length - 1
        ));

        shared.set(side, {
            anteriorCommunicatingJunction,
            internalCarotidTerminus,
            posteriorCerebralJunction,
            vertebrobasilarJunction
        });
    }

    const right = shared.get('right');
    const left = shared.get('left');
    const basilarApex = headPoint(BRAIN_MIDLINE_X_MM, 572, -12, 1.52);
    paths.push(path(
        'anterior-communicating',
        'midline',
        ['anterior-communicating'],
        [
            clonedPoint(right.anteriorCommunicatingJunction, 0.78),
            headPoint(BRAIN_MIDLINE_X_MM, 560, 30, 0.78),
            clonedPoint(left.anteriorCommunicatingJunction, 0.78)
        ]
    ));
    paths.push(path(
        'basilar',
        'midline',
        ['basilar'],
        [
            clonedPoint(right.vertebrobasilarJunction, 1.45),
            headPoint(BRAIN_MIDLINE_X_MM, 545, -18, 1.5),
            headPoint(BRAIN_MIDLINE_X_MM, 560, -15, 1.52),
            basilarApex
        ]
    ));
    for (const side of ['right', 'left']) {
        paths.push(path(
            `${side}-posterior-cerebral-p1`,
            side,
            ['posterior-cerebral-p1'],
            [
                clonedPoint(basilarApex, 1.28),
                clonedPoint(shared.get(side).posteriorCerebralJunction, 1.24)
            ]
        ));
    }

    return paths;
}

export { BRAIN_MIDLINE_X_MM };
