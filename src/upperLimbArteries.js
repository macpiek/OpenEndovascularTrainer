import * as THREE from 'three';

const ATTACHMENT_MINIMUM_ABS_X_MM = 125;
const ATTACHMENT_MINIMUM_Y_MM = 60;
const ATTACHMENT_MAXIMUM_Y_MM = 135;
const ATTACHMENT_TARGET_ABS_X_MM = 145;
const ATTACHMENT_TARGET_Y_MM = 100;
const INWARD_TRACE_LENGTH_MM = 18;

export const UPPER_LIMB_LATERAL_EXTENT_MM = 225;
export const UPPER_LIMB_HAND_DISTAL_Y_MM = -625;
export const UPPER_LIMB_FINGER_DISTAL_Y_MM = -660;

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

function traceInward(root, adjacency) {
    const samples = [{ position: root.point.clone(), radius: root.radius }];
    let currentNodeId = root.nodeId;
    let previousNodeId = null;
    let distance = 0;

    while (distance < INWARD_TRACE_LENGTH_MM) {
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

function attachmentScore(leaf) {
    return leaf.radius * 30 -
        Math.abs(Math.abs(leaf.point.x) - ATTACHMENT_TARGET_ABS_X_MM) * 0.2 -
        Math.abs(leaf.point.y - ATTACHMENT_TARGET_Y_MM) * 0.4 -
        Math.abs(leaf.point.z) * 0.1;
}

/**
 * Finds the distal right and left subclavian stumps in the original aortic
 * STL. Patient-right is -X and patient-left is +X in simulator coordinates.
 * The radius-weighted score favours the true right subclavian continuation,
 * while the skeletal shoulder target disambiguates the equally small left
 * terminal branches produced by medial-axis extraction.
 */
export function findUpperLimbAttachmentRoots(centerlineSegments) {
    const adjacency = centerlineAdjacency(centerlineSegments);
    const candidates = terminalLeaves(centerlineSegments).filter(leaf =>
        Math.abs(leaf.point.x) >= ATTACHMENT_MINIMUM_ABS_X_MM &&
        leaf.point.y >= ATTACHMENT_MINIMUM_Y_MM &&
        leaf.point.y <= ATTACHMENT_MAXIMUM_Y_MM
    );

    return [-1, 1].map((sideSign, index) => {
        const sideCandidates = candidates
            .filter(leaf => Math.sign(leaf.point.x) === sideSign)
            .sort((a, b) => attachmentScore(b) - attachmentScore(a));
        if (!sideCandidates.length) {
            throw new Error(
                `Expected a ${sideSign < 0 ? 'right' : 'left'} subclavian attachment root`
            );
        }
        const root = sideCandidates[0];
        return {
            ...root,
            anatomicalSide: index === 0 ? 'right' : 'left',
            sideSign,
            inwardTrace: traceInward(root, adjacency)
        };
    });
}

function point(x, y, z, radius) {
    return { position: new THREE.Vector3(x, y, z), radius };
}

function clonedPoint(source, radius = source.radius) {
    return point(
        source.position.x,
        source.position.y,
        source.position.z,
        radius
    );
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

function rootPoint(root, distance, radius) {
    const position = root.point.clone().addScaledVector(root.outward, distance);
    return point(position.x, position.y, position.z, radius);
}

function inletPoints(root, inletRadius) {
    if (!root.inwardTrace?.length) {
        return [
            rootPoint(root, -12, inletRadius * 0.58),
            point(root.point.x, root.point.y, root.point.z, inletRadius)
        ];
    }
    return root.inwardTrace.map((sample, index, samples) => {
        const progress = index / Math.max(1, samples.length - 1);
        const overlapRadius = root.anatomicalSide === 'right'
            ? Math.max(
                inletRadius,
                sample.radius + 0.8,
                THREE.MathUtils.lerp(4.5, 2.8, progress)
            )
            : Math.max(inletRadius * 0.55, sample.radius + 0.35);
        return point(
            sample.position.x,
            sample.position.y,
            sample.position.z,
            index === samples.length - 1 ? inletRadius : overlapRadius
        );
    });
}

function skeletalGuide(root) {
    if (root.anatomicalSide === 'right') {
        return {
            axillaryX: -154,
            proximalBrachialX: -178,
            midBrachialX: -185,
            distalBrachialX: -188,
            elbowX: -192,
            radialForearmX: -218,
            ulnarForearmX: -198,
            radialWristX: -222,
            ulnarWristX: -198
        };
    }
    return {
        axillaryX: 150,
        proximalBrachialX: 172,
        midBrachialX: 179,
        distalBrachialX: 184,
        elbowX: 190,
        radialForearmX: 220,
        ulnarForearmX: 198,
        radialWristX: 222,
        ulnarWristX: 198
    };
}

/**
 * Defines the bilateral upper-limb arterial tree in simulator millimetres.
 * The paths follow the loaded skeleton: axillary and brachial arteries remain
 * anterior-medial to the humerus, radial and ulnar arteries flank the paired
 * forearm bones, and the palmar vessels sit anterior to the hand skeleton.
 */
export function createUpperLimbArteryPaths(attachmentRoots) {
    if (attachmentRoots.length !== 2) {
        throw new Error('Two paired upper-limb attachment roots are required');
    }

    const paths = [];
    for (const root of attachmentRoots) {
        const side = root.anatomicalSide;
        const sideSign = root.sideSign;
        const guide = skeletalGuide(root);
        const inletRadius = THREE.MathUtils.clamp(
            root.radius * 1.12,
            2.2,
            2.85
        );
        const inlet = inletPoints(root, inletRadius);
        // The original right subclavian stump initially points superiorly,
        // whereas the left stump already points toward the axilla. Give the
        // right side a rounded turn over the first rib with intermediate
        // control points instead of one long hairpin span; this keeps the
        // extracted medial axis clear of the wall throughout the transition.
        const subclavianTransition = side === 'right'
            ? [
                rootPoint(root, 7, 3.75),
                rootPoint(root, 14, 4.5),
                point(-147, 132, 14, 4.25),
                point(-151, 110, -7, 3.4)
            ]
            : [rootPoint(root, 14, 2.75)];
        const axillary = point(guide.axillaryX, 64, -39, 2.75);
        const subscapularOrigin = point(
            side === 'right' ? -165 : 162,
            28,
            -43,
            1.08
        );
        const proximalBrachial = point(
            guide.proximalBrachialX,
            -18,
            -47,
            2.62
        );
        const deepBrachialOrigin = point(
            THREE.MathUtils.lerp(
                guide.proximalBrachialX,
                guide.midBrachialX,
                0.28
            ),
            -48,
            -52,
            2.56
        );
        const midBrachial = point(guide.midBrachialX, -112, -62, 2.48);
        const distalBrachial = point(
            guide.distalBrachialX,
            -190,
            -70,
            2.32
        );
        const cubitalFossa = point(guide.elbowX, -235, -58, 2.18);
        const brachialDivision = point(
            guide.elbowX + sideSign * 1.5,
            -258,
            -56,
            1.98
        );
        const ulnarOrigin = point(
            guide.ulnarForearmX,
            -282,
            -70,
            1.72
        );
        const radialProximal = point(sideSign * 207, -286, -64, 1.68);
        const interosseousDivision = point(
            sideSign * 204,
            -306,
            -78,
            0.94
        );
        const radialWrist = point(guide.radialWristX, -475, -5, 1.16);
        const ulnarWrist = point(guide.ulnarWristX, -475, -7, 1.18);

        paths.push(path(
            `${side}-axillary-brachial`,
            side,
            ['subclavian-distal', 'axillary', 'brachial'],
            [
                ...inlet,
                ...subclavianTransition,
                axillary,
                proximalBrachial,
                deepBrachialOrigin,
                midBrachial,
                distalBrachial,
                cubitalFossa,
                brachialDivision
            ],
            false,
            0
        ));

        paths.push(path(
            `${side}-thoracoacromial`,
            side,
            ['thoracoacromial'],
            [
                clonedPoint(axillary, 1.02),
                point(sideSign * 136, 75, -27, 0.88),
                point(sideSign * 120, 88, -12, 0.56)
            ],
            true
        ));

        paths.push(path(
            `${side}-lateral-thoracic`,
            side,
            ['lateral-thoracic'],
            [
                clonedPoint(axillary, 0.92),
                point(sideSign * 145, 22, -26, 0.78),
                point(sideSign * 136, -25, -18, 0.56)
            ],
            true
        ));

        const subscapularDivision = point(
            sideSign * 146,
            -5,
            -75,
            0.88
        );
        paths.push(path(
            `${side}-subscapular-thoracodorsal`,
            side,
            ['subscapular', 'thoracodorsal'],
            [
                clonedPoint(axillary, 1.15),
                subscapularOrigin,
                subscapularDivision,
                point(sideSign * 138, -65, -90, 0.7),
                point(sideSign * 134, -125, -92, 0.56)
            ],
            true
        ));

        paths.push(path(
            `${side}-circumflex-scapular`,
            side,
            ['circumflex-scapular'],
            [
                clonedPoint(subscapularDivision, 0.72),
                point(sideSign * 128, 10, -94, 0.62),
                point(sideSign * 113, 29, -84, 0.52)
            ],
            true
        ));

        paths.push(path(
            `${side}-anterior-circumflex-humeral`,
            side,
            ['anterior-circumflex-humeral'],
            [
                clonedPoint(axillary, 0.78),
                point(sideSign * 178, 20, -23, 0.64),
                point(sideSign * 185, 38, -9, 0.52)
            ],
            true
        ));

        paths.push(path(
            `${side}-posterior-circumflex-humeral`,
            side,
            ['posterior-circumflex-humeral'],
            [
                clonedPoint(axillary, 0.86),
                point(sideSign * 189, 23, -67, 0.7),
                point(sideSign * 201, 40, -56, 0.54)
            ],
            true
        ));

        paths.push(path(
            `${side}-deep-brachial`,
            side,
            ['deep-brachial'],
            [
                clonedPoint(deepBrachialOrigin, 1.42),
                point(sideSign * 202, -92, -101, 1.22),
                point(sideSign * 211, -158, -116, 0.96),
                point(sideSign * 205, -207, -91, 0.7)
            ],
            true
        ));

        paths.push(path(
            `${side}-radial`,
            side,
            ['radial'],
            [
                clonedPoint(brachialDivision, 1.78),
                radialProximal,
                point(guide.radialForearmX, -345, -51, 1.52),
                point(guide.radialForearmX + sideSign * 2, -410, -27, 1.34),
                radialWrist
            ]
        ));

        paths.push(path(
            `${side}-ulnar`,
            side,
            ['ulnar'],
            [
                clonedPoint(brachialDivision, 1.82),
                ulnarOrigin,
                point(guide.ulnarForearmX, -345, -64, 1.56),
                point(guide.ulnarForearmX, -410, -38, 1.36),
                ulnarWrist
            ]
        ));

        paths.push(path(
            `${side}-common-interosseous`,
            side,
            ['common-interosseous', 'anterior-interosseous'],
            [
                clonedPoint(ulnarOrigin, 1.02),
                interosseousDivision,
                point(sideSign * 207, -365, -60, 0.76),
                point(sideSign * 207, -432, -31, 0.56)
            ],
            true
        ));

        paths.push(path(
            `${side}-superior-ulnar-collateral`,
            side,
            ['superior-ulnar-collateral'],
            [
                clonedPoint(midBrachial, 0.82),
                point(sideSign * 174, -151, -72, 0.68),
                point(sideSign * 168, -215, -61, 0.54)
            ],
            true
        ));

        paths.push(path(
            `${side}-inferior-ulnar-collateral`,
            side,
            ['inferior-ulnar-collateral'],
            [
                clonedPoint(distalBrachial, 0.74),
                point(sideSign * 179, -207, -56, 0.62),
                point(sideSign * 177, -242, -46, 0.52)
            ],
            true
        ));

        paths.push(path(
            `${side}-radial-recurrent`,
            side,
            ['radial-recurrent'],
            [
                clonedPoint(radialProximal, 0.72),
                point(sideSign * 213, -260, -76, 0.62),
                point(sideSign * 209, -225, -81, 0.52)
            ],
            true
        ));

        paths.push(path(
            `${side}-ulnar-recurrent`,
            side,
            ['ulnar-recurrent'],
            [
                clonedPoint(ulnarOrigin, 0.74),
                point(sideSign * 188, -268, -83, 0.62),
                point(sideSign * 183, -230, -75, 0.52)
            ],
            true
        ));

        paths.push(path(
            `${side}-posterior-interosseous`,
            side,
            ['posterior-interosseous'],
            [
                clonedPoint(interosseousDivision, 0.78),
                point(sideSign * 214, -332, -92, 0.68),
                point(sideSign * 217, -390, -75, 0.6),
                point(sideSign * 214, -440, -41, 0.52)
            ],
            true
        ));

        const superficialUlnar = point(sideSign * 198, -514, 19, 1.02);
        const superficialMedial = point(sideSign * 203, -529, 24, 0.94);
        const superficialCenter = point(sideSign * 209, -535, 26, 0.88);
        const superficialLateral = point(sideSign * 216, -530, 23, 0.78);
        paths.push(path(
            `${side}-superficial-palmar-arch`,
            side,
            ['superficial-palmar-arch'],
            [
                clonedPoint(ulnarWrist, 1.15),
                superficialUlnar,
                superficialMedial,
                superficialCenter,
                superficialLateral
            ]
        ));

        const deepArchRadial = point(sideSign * 220, -500, 5, 1.04);
        paths.push(path(
            `${side}-deep-palmar-arch`,
            side,
            ['deep-palmar-arch'],
            [
                clonedPoint(radialWrist, 1.12),
                deepArchRadial,
                point(sideSign * 214, -516, 11, 0.94),
                point(sideSign * 207, -519, 13, 0.84),
                point(sideSign * 199, -514, 10, 0.7)
            ],
            true
        ));

        const princepsPollicisDivision = point(
            sideSign * 226,
            -585,
            22,
            0.64
        );
        paths.push(path(
            `${side}-princeps-pollicis`,
            side,
            ['princeps-pollicis'],
            [
                clonedPoint(deepArchRadial, 0.76),
                point(sideSign * 229, -540, 14, 0.7),
                princepsPollicisDivision
            ]
        ));
        for (const [surface, distalMagnitude] of [
            ['radial', 218],
            ['ulnar', 211]
        ]) {
            paths.push(path(
                `${side}-proper-palmar-digital-thumb-${surface}`,
                side,
                [`proper-palmar-digital-thumb-${surface}`],
                [
                    clonedPoint(princepsPollicisDivision, 0.6),
                    point(
                        sideSign * THREE.MathUtils.lerp(
                            226,
                            distalMagnitude,
                            0.55
                        ),
                        -607,
                        21,
                        0.58
                    ),
                    point(
                        sideSign * distalMagnitude,
                        -630,
                        19,
                        0.56
                    )
                ],
                true
            ));
        }

        const digitalDefinitions = [
            [superficialLateral, 218, 208, 0.68],
            [superficialCenter, 210, 196, 0.66],
            [superficialMedial, 202, 183, 0.62],
            [superficialUlnar, 196, 170, 0.58]
        ];
        digitalDefinitions.forEach(([
            origin,
            middleMagnitude,
            terminalMagnitude,
            originRadius
        ], index) => {
            const commonDigitalTerminal = point(
                sideSign * terminalMagnitude,
                UPPER_LIMB_HAND_DISTAL_Y_MM,
                20 - index * 2,
                0.62 - index * 0.015
            );
            paths.push(path(
                `${side}-common-palmar-digital-${index + 1}`,
                side,
                [`common-palmar-digital-${index + 1}`],
                [
                    clonedPoint(origin, originRadius),
                    point(
                        sideSign * middleMagnitude,
                        -573,
                        27 - index * 2,
                        originRadius * 0.86
                    ),
                    commonDigitalTerminal
                ]
            ));

            const distalCenterMagnitude = [187, 176, 164, 153][index];
            for (const [surface, offset] of [
                ['radial', 3],
                ['ulnar', -3]
            ]) {
                const distalMagnitude = distalCenterMagnitude + offset;
                paths.push(path(
                    `${side}-proper-palmar-digital-${index + 1}-${surface}`,
                    side,
                    [`proper-palmar-digital-${index + 1}-${surface}`],
                    [
                        clonedPoint(commonDigitalTerminal, 0.6),
                        point(
                            sideSign * THREE.MathUtils.lerp(
                                terminalMagnitude,
                                distalMagnitude,
                                0.55
                            ),
                            -643,
                            19 - index * 2,
                            0.58
                        ),
                        point(
                            sideSign * distalMagnitude,
                            UPPER_LIMB_FINGER_DISTAL_Y_MM,
                            18 - index * 2,
                            0.56
                        )
                    ],
                    true
                ));
            }
        });
    }
    return paths;
}
