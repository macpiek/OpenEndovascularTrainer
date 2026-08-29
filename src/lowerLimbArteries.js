import * as THREE from 'three';

const ATTACHMENT_MINIMUM_RADIUS_MM = 2.5;
const ATTACHMENT_DISTAL_BAND_MM = 8;

function endpoint(segment, atStart) {
    return {
        nodeId: atStart ? segment.nodeStartId : segment.nodeEndId,
        point: (atStart ? segment.start : segment.end).clone(),
        neighbour: (atStart ? segment.end : segment.start).clone(),
        radius: atStart ? segment.radiusStart : segment.radiusEnd,
        segmentId: segment.id
    };
}

/**
 * Finds the two large inferior terminal vessels used to attach the leg tree.
 * Patient-right is -X and patient-left is +X in the imported anatomy.
 */
export function findLowerLimbAttachmentRoots(centerlineSegments) {
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
    const distalY = Math.min(...leaves.map(leaf => leaf.point.y));
    const candidates = leaves.filter(leaf =>
        leaf.radius >= ATTACHMENT_MINIMUM_RADIUS_MM &&
        leaf.point.y <= distalY + ATTACHMENT_DISTAL_BAND_MM
    );
    if (candidates.length !== 2) {
        throw new Error(
            `Expected two distal femoral attachment roots, found ${candidates.length}`
        );
    }

    candidates.sort((a, b) => a.point.x - b.point.x);
    return candidates.map((candidate, index) => ({
        ...candidate,
        anatomicalSide: index === 0 ? 'right' : 'left',
        sideSign: index === 0 ? -1 : 1,
        outward: candidate.point.clone().sub(candidate.neighbour).normalize()
    }));
}

function point(x, y, z, radius) {
    return { position: new THREE.Vector3(x, y, z), radius };
}

function skeletalGuide(root) {
    if (root.anatomicalSide === 'right') {
        return {
            upperFemoralX: -92,
            adductorX: -76,
            distalFemoralX: -68,
            kneeX: -67,
            interosseousX: -75,
            tibialX: -59,
            fibularX: -84,
            anteriorAnkleX: -68,
            medialAnkleX: -51,
            dorsalisX: -70,
            dorsalisForefootX: -49,
            posteriorFootX: -55,
            lateralPlantarX: -95,
            plantarCenterX: -72,
            medialPlantarX: -43
        };
    }
    return {
        upperFemoralX: 90,
        adductorX: 84,
        distalFemoralX: 81,
        kneeX: 82,
        interosseousX: 94,
        tibialX: 80,
        fibularX: 104,
        anteriorAnkleX: 91,
        medialAnkleX: 74,
        dorsalisX: 93,
        dorsalisForefootX: 76,
        posteriorFootX: 78,
        lateralPlantarX: 124,
        plantarCenterX: 99,
        medialPlantarX: 70
    };
}

function guidedPoint(x, y, z, radius) {
    return point(x, y, z, radius);
}

/**
 * Defines the bilateral lower-limb arterial tree in simulator millimetres.
 * Radii describe the lumen. The STL generator builds a wall around these
 * paths and intersects every child path deeply with its parent so both the
 * vessel wall and the carved lumen remain connected after Boolean operations.
 */
export function createLowerLimbArteryPaths(attachmentRoots) {
    const paths = [];
    for (const root of attachmentRoots) {
        const guide = skeletalGuide(root);
        const inletRadius = THREE.MathUtils.clamp(root.radius * 1.08, 3, 3.45);
        const embedded = root.point.clone().addScaledVector(root.outward, -7);
        const entry = root.point.clone().addScaledVector(root.outward, 22);
        const commonFemoral = guidedPoint(
            THREE.MathUtils.lerp(root.point.x, guide.upperFemoralX, 0.45),
            -520,
            1,
            3.05
        );
        const upperFemoral = guidedPoint(
            guide.upperFemoralX,
            -585,
            -2,
            2.95
        );
        const adductor = guidedPoint(guide.adductorX, -700, -10, 2.75);
        const distalFemoral = guidedPoint(
            guide.distalFemoralX,
            -815,
            -28,
            2.55
        );
        const popliteal = guidedPoint(guide.kneeX, -875, -61, 2.35);
        const poplitealDivision = guidedPoint(
            guide.kneeX + root.sideSign * 2,
            -910,
            -66,
            2.15
        );
        const tibioperonealMid = guidedPoint(
            guide.kneeX + root.sideSign * 3,
            -932,
            -69,
            2.02
        );
        const tibioperonealSplit = guidedPoint(
            guide.kneeX + root.sideSign * 4,
            -958,
            -70,
            1.9
        );
        const anteriorPassage = guidedPoint(
            guide.interosseousX,
            -942,
            -24,
            1.88
        );
        const anteriorAnkle = guidedPoint(
            guide.anteriorAnkleX,
            -1260,
            -22,
            1.3
        );
        const dorsalisHindfoot = guidedPoint(
            guide.dorsalisX,
            -1293,
            -14,
            1.18
        );
        const dorsalisMidfoot = guidedPoint(
            THREE.MathUtils.lerp(
                guide.dorsalisX,
                guide.dorsalisForefootX,
                0.42
            ),
            -1317,
            18,
            1.02
        );
        const dorsalisForefoot = guidedPoint(
            guide.dorsalisForefootX,
            -1333,
            47,
            0.86
        );
        const posteriorAnkle = guidedPoint(
            guide.medialAnkleX,
            -1260,
            -76,
            1.4
        );
        const plantarDivision = guidedPoint(
            guide.posteriorFootX,
            -1308,
            -66,
            1.16
        );
        const plantarHeel = guidedPoint(
            guide.posteriorFootX + root.sideSign * 4,
            -1352,
            -47,
            1.08
        );
        const medialPlantarHeel = guidedPoint(
            THREE.MathUtils.lerp(
                guide.posteriorFootX,
                guide.medialPlantarX,
                0.35
            ),
            -1349,
            -43,
            0.96
        );
        const lateralPlantarMidfoot = guidedPoint(
            THREE.MathUtils.lerp(
                guide.posteriorFootX,
                guide.lateralPlantarX,
                0.68
            ),
            -1367,
            -12,
            1
        );
        const lateralPlantarForefoot = guidedPoint(
            guide.lateralPlantarX,
            -1380,
            30,
            0.9
        );
        const plantarArchLateral = guidedPoint(
            guide.lateralPlantarX,
            -1385,
            44,
            0.86
        );
        const plantarArchCenter = guidedPoint(
            guide.plantarCenterX,
            -1386,
            47,
            0.82
        );
        const plantarArchMedial = guidedPoint(
            guide.medialPlantarX + root.sideSign * 3,
            -1382,
            47,
            0.78
        );
        const deepPlantar = guidedPoint(
            guide.dorsalisForefootX,
            -1368,
            48,
            0.8
        );
        const medialPlantarForefoot = guidedPoint(
            guide.medialPlantarX,
            -1380,
            31,
            0.68
        );

        paths.push({
            name: `${root.anatomicalSide}-femoral-popliteal`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: [
                'common-femoral',
                'superficial-femoral',
                'popliteal'
            ],
            points: [
                point(embedded.x, embedded.y, embedded.z, inletRadius),
                point(entry.x, entry.y, entry.z, 3.15),
                commonFemoral,
                upperFemoral,
                adductor,
                distalFemoral,
                popliteal,
                poplitealDivision
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-deep-femoral`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['deep-femoral'],
            points: [
                point(entry.x, entry.y, entry.z, 3.15),
                commonFemoral,
                guidedPoint(
                    guide.upperFemoralX + root.sideSign * 8,
                    -538,
                    -13,
                    2.4
                ),
                guidedPoint(
                    guide.upperFemoralX + root.sideSign * 20,
                    -600,
                    -27,
                    1.9
                ),
                guidedPoint(
                    guide.upperFemoralX + root.sideSign * 32,
                    -675,
                    -34,
                    1.25
                )
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-anterior-tibial`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['anterior-tibial', 'dorsalis-pedis'],
            points: [
                popliteal,
                poplitealDivision,
                anteriorPassage,
                guidedPoint(
                    guide.interosseousX - root.sideSign * 2,
                    -1005,
                    -14,
                    1.72
                ),
                guidedPoint(
                    guide.interosseousX - root.sideSign * 4,
                    -1110,
                    -16,
                    1.52
                ),
                anteriorAnkle,
                dorsalisHindfoot,
                dorsalisMidfoot,
                dorsalisForefoot
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-tibioperoneal-trunk`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['tibioperoneal-trunk'],
            points: [
                popliteal,
                poplitealDivision,
                tibioperonealMid,
                tibioperonealSplit
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-posterior-tibial`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['posterior-tibial'],
            points: [
                tibioperonealMid,
                tibioperonealSplit,
                guidedPoint(guide.tibialX, -1010, -70, 1.76),
                guidedPoint(
                    guide.tibialX - root.sideSign * 3,
                    -1115,
                    -72,
                    1.58
                ),
                posteriorAnkle,
                plantarDivision
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-fibular`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['fibular'],
            points: [
                tibioperonealMid,
                tibioperonealSplit,
                guidedPoint(guide.fibularX, -1010, -69, 1.55),
                guidedPoint(
                    guide.fibularX - root.sideSign * 1.5,
                    -1120,
                    -73,
                    1.25
                ),
                guidedPoint(
                    guide.fibularX - root.sideSign * 3,
                    -1225,
                    -70,
                    0.95
                )
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-lateral-plantar`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['lateral-plantar'],
            points: [
                plantarDivision,
                plantarHeel,
                lateralPlantarMidfoot,
                lateralPlantarForefoot,
                plantarArchLateral
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-deep-plantar-arch`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['deep-plantar', 'plantar-arch'],
            points: [
                dorsalisForefoot,
                deepPlantar,
                plantarArchMedial,
                plantarArchCenter,
                plantarArchLateral
            ]
        });

        paths.push({
            name: `${root.anatomicalSide}-medial-plantar`,
            anatomicalSide: root.anatomicalSide,
            vesselNames: ['medial-plantar'],
            points: [
                plantarDivision,
                medialPlantarHeel,
                guidedPoint(
                    THREE.MathUtils.lerp(
                        guide.posteriorFootX,
                        guide.medialPlantarX,
                        0.6
                    ),
                    -1366,
                    -5,
                    0.84
                ),
                medialPlantarForefoot
            ]
        });
    }
    return paths;
}
