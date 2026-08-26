import assert from 'node:assert/strict';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import {
    CONTRAST_SOURCE_CATHETER,
    CONTRAST_SOURCE_SHEATH,
    HybridContrastSystem
} from '../src/contrast/hybridContrastSystem.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import { ContinuousInjectionFlowCoupler } from '../src/contrast/continuousInjectionFlowCoupler.js';

function segment(id, startNodeId, endNodeId, start, end, radiusStart, radiusEnd = radiusStart) {
    return {
        id,
        nodeStartId: startNodeId,
        nodeEndId: endNodeId,
        start: new THREE.Vector3(...start),
        end: new THREE.Vector3(...end),
        radiusStart,
        radiusEnd,
        safeRadius: Math.min(radiusStart, radiusEnd)
    };
}

function branchingTree() {
    return [
        segment(0, 0, 1, [0, 0, 0], [0, 60, 0], 10, 8),
        segment(1, 1, 2, [0, 60, 0], [-30, 110, 0], 7, 5),
        segment(2, 1, 3, [0, 60, 0], [30, 110, 0], 5, 4)
    ];
}

function symmetricBranchingTree() {
    return [
        segment(0, 0, 1, [0, 0, 0], [0, 60, 0], 8),
        segment(1, 1, 2, [0, 60, 0], [-30, 110, 0], 5),
        segment(2, 1, 3, [0, 60, 0], [30, 110, 0], 5)
    ];
}

function sideBranchTree() {
    return [
        segment(0, 0, 1, [0, 0, 0], [0, 60, 0], 8),
        segment(1, 1, 2, [0, 60, 0], [0, 120, 0], 8),
        segment(2, 1, 3, [0, 60, 0], [30, 82, 0], 4)
    ];
}

function straightTree() {
    return [
        segment(0, 0, 1, [0, 0, 0], [0, 140, 0], 7, 7)
    ];
}

const network = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const topology = network.getTopologyDiagnostics();
assert.equal(topology.directedEdgeCount, 3);
assert.equal(topology.outletCount, 2);
assert.equal(topology.disconnectedSourceSegmentCount, 0);
const rootFlow = network.edges[0].meanFlowMm3PerS;
const childFlow = network.edges[0].childEdgeIndices.reduce(
    (sum, edgeIndex) => sum + network.edges[edgeIndex].meanFlowMm3PerS,
    0
);
assert.ok(Math.abs(rootFlow - childFlow) / rootFlow < 1e-12,
    'hydraulic flow must be conserved at each bifurcation');

const commonLumenInjectionLocation = network.findInjectionLocation(
    new THREE.Vector3(1, 61, 0),
    new THREE.Vector3(0, 1, 0)
);
assert.equal(
    commonLumenInjectionLocation.edgeIndex,
    0,
    'a catheter inside the large common lumen must not attach to a nearby small branch centerline'
);
assert.equal(
    commonLumenInjectionLocation.selectionMode,
    'containing-lumen'
);
const selectiveBranchInjectionLocation = network.findInjectionLocation(
    new THREE.Vector3(12, 80, 0),
    new THREE.Vector3(30, 50, 0).normalize()
);
assert.equal(
    selectiveBranchInjectionLocation.edgeIndex,
    2,
    'a catheter beyond the common lumen should still support selective branch injection'
);

const tunableNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    heartRateBpm: 72
});
const baselineFlows = tunableNetwork.edges.map(edge => edge.meanFlowMm3PerS);
tunableNetwork.setHemodynamics({
    cardiacOutputMlPerMin: 1800,
    heartRateBpm: 120
});
for (let index = 0; index < tunableNetwork.edges.length; index++) {
    assert.ok(
        Math.abs(tunableNetwork.edges[index].meanFlowMm3PerS / baselineFlows[index] - 2) <
            1e-12,
        'changing cardiac output should rescale flow through every vessel'
    );
}
assert.equal(tunableNetwork.hemodynamics.heartRateBpm, 120,
    'changing heart rate should update the pulsatile waveform immediately');
assert.throws(
    () => tunableNetwork.setHemodynamics({ cardiacOutputMlPerMin: 0 }),
    /positive/,
    'invalid debug hemodynamics should be rejected'
);

assert.ok(network.depositIodine(0, 0, 120));
for (let step = 0; step < 360; step++) network.update(1 / 120);
const transported = network.getMassBalanceSnapshot();
assert.ok(
    Math.abs(
        transported.intravascularIodineMassMg +
        transported.outletIodineMassMg -
        120
    ) < 1e-6,
    'finite-volume transport must conserve iodine mass'
);
for (let step = 0; step < 240; step++) network.update(1 / 120);
assert.ok(network.getIodineMassMg() < 1e-4,
    'contrast must wash out completely after traversing a bifurcation');
const nearest = network.findNearestLocation(new THREE.Vector3(-10, 78, 0));
assert.ok(nearest.edgeIndex >= 0);
assert.ok(nearest.distance < 1);

const continuousSplitNetwork = new ContrastFlowNetwork(straightTree(), {
    cardiacOutputMlPerMin: 900,
    heartRateBpm: 72
});
const continuousSplitCoupler = new ContinuousInjectionFlowCoupler(
    continuousSplitNetwork
);
const upstreamPort = {
    kind: 'sheath-end',
    position: new THREE.Vector3(0, 70, 0),
    direction: new THREE.Vector3(0, -1, 0),
    weight: 1,
    valid: true
};
const splitLocation = continuousSplitNetwork.findInjectionLocation(
    upstreamPort.position,
    upstreamPort.direction
);
const proximalFaceIndex = splitLocation.cellIndex;
const signedProximalFlows = [1, 4, 7, 10, 13, 16, 19].map(rateMlPerSec => {
    continuousSplitNetwork.clearFlowOverrides();
    continuousSplitCoupler.beginStep();
    continuousSplitCoupler.configure({
        ports: [upstreamPort],
        rateMlPerSec,
        portFlowRatesMlPerSec: [rateMlPerSec]
    });
    return continuousSplitNetwork.getFaceSignedFlowMm3PerS(
        splitLocation.edgeIndex,
        proximalFaceIndex,
        1
    );
});
for (let index = 1; index < signedProximalFlows.length; index++) {
    assert.ok(
        Math.abs(
            signedProximalFlows[index] -
            signedProximalFlows[index - 1] +
            2850
        ) < 1e-9,
        `proximal flow must vary continuously with injection rate (${signedProximalFlows})`
    );
}
assert.ok(
    signedProximalFlows.some(flow => flow > 0) &&
        signedProximalFlows.some(flow => flow < 0),
    `one signed-flow equation must cross zero without a behavior switch (${signedProximalFlows})`
);
continuousSplitNetwork.clearFlowOverrides();
continuousSplitCoupler.beginStep();
const catheterEquivalentPort = {
    ...upstreamPort,
    kind: 'catheter-end'
};
continuousSplitCoupler.configure({
    ports: [catheterEquivalentPort],
    rateMlPerSec: 10,
    portFlowRatesMlPerSec: [10]
});
const catheterEquivalentFlow =
    continuousSplitNetwork.getFaceSignedFlowMm3PerS(
        splitLocation.edgeIndex,
        proximalFaceIndex,
        1
    );
assert.ok(
    Math.abs(catheterEquivalentFlow - signedProximalFlows[3]) < 1e-9,
    'sheath and catheter labels must use the same flow-coupling physics for identical outlet parameters'
);

const symmetricJunctionNetwork = new ContrastFlowNetwork(
    symmetricBranchingTree(),
    {
        cardiacOutputMlPerMin: 900,
        axialDispersionMm2PerS: 10
    }
);
const symmetricParent = symmetricJunctionNetwork.edges[0];
const symmetricChildren = symmetricParent.childEdgeIndices.map(
    edgeIndex => symmetricJunctionNetwork.edges[edgeIndex]
);
symmetricJunctionNetwork.depositIodine(
    symmetricParent.index,
    symmetricParent.cellCount - 1,
    symmetricParent.volumes[symmetricParent.cellCount - 1] * 0.3
);
symmetricJunctionNetwork.update(1 / 120);
const symmetricChildMasses = symmetricChildren.map(
    edge => edge.massMg.reduce((sum, mass) => sum + mass, 0)
);
assert.ok(
    Math.abs(symmetricChildMasses[0] - symmetricChildMasses[1]) <
        Math.max(1e-10, symmetricChildMasses[0] * 1e-9),
    `a symmetric bifurcation must mix without child-order bias (${symmetricChildMasses})`
);

function runRetrogradeSheathInjection(rateMlPerSec) {
    const sheath = {
        start: new THREE.Vector3(-30, 110, 0),
        end: new THREE.Vector3(-24, 100, 0),
        radius: 1
    };
    const system = new HybridContrastSystem({
        centerlineSegments: branchingTree(),
        sheath,
        hemodynamics: {
            cardiacOutputMlPerMin: 900,
            heartRateBpm: 72,
            axialDispersionMm2PerS: 8
        },
        localOptions: {
            randomSeed: 20260730
        }
    });
    const sourceLocation = system.flowNetwork.findNearestLocation(sheath.end);
    const target = system.flowNetwork.findUpstreamMixingJunction(sourceLocation);
    assert.ok(target, 'the iliac sheath should have a large upstream mixing junction');
    const oppositeEdgeIndex = target.childEdgeIndices.find(
        edgeIndex => edgeIndex !== target.sourceChildEdgeIndex
    );
    assert.ok(oppositeEdgeIndex >= 0, 'the mixing junction should expose the opposite iliac');
    const pathVolumeMl = target.pathEdgeIndices.reduce(
        (sum, edgeIndex) =>
            sum + system.flowNetwork.edges[edgeIndex].totalVolume,
        0
    ) / 1000;
    const injectionVolumeMl = Math.max(3, pathVolumeMl * 1.4);
    assert.equal(system.startInjection({
        source: CONTRAST_SOURCE_SHEATH,
        volumeMl: injectionVolumeMl,
        rateMlPerSec
    }).ok, true);
    let maximumOppositeIliacMassMg = 0;
    let firstSourceNetworkMassSeconds = Infinity;
    let firstOppositeIliacMassSeconds = Infinity;
    const stepCount = Math.ceil(
        (injectionVolumeMl / rateMlPerSec + 2) * 120
    );
    for (let step = 0; step < stepCount; step++) {
        system.update(1 / 120);
        const sourceEdge = system.flowNetwork.edges[sourceLocation.edgeIndex];
        const sourceMassMg = sourceEdge.massMg.reduce(
            (sum, mass) => sum + mass,
            0
        );
        if (sourceMassMg > 1e-4 && !Number.isFinite(firstSourceNetworkMassSeconds)) {
            firstSourceNetworkMassSeconds = (step + 1) / 120;
        }
        const oppositeEdge = system.flowNetwork.edges[oppositeEdgeIndex];
        const oppositeMassMg = oppositeEdge.massMg.reduce(
            (sum, mass) => sum + mass,
            0
        );
        maximumOppositeIliacMassMg = Math.max(
            maximumOppositeIliacMassMg,
            oppositeMassMg
        );
        if (
            oppositeMassMg > 1e-4 &&
            !Number.isFinite(firstOppositeIliacMassSeconds)
        ) {
            firstOppositeIliacMassSeconds = (step + 1) / 120;
        }
    }
    return {
        maximumOppositeIliacMassMg,
        firstSourceNetworkMassSeconds,
        firstOppositeIliacMassSeconds,
        pathVolumeMl,
        metrics: system.getMetrics()
    };
}

const slowRetrogradeInjection = runRetrogradeSheathInjection(1);
const fastRetrogradeInjection = runRetrogradeSheathInjection(15);
assert.ok(
    slowRetrogradeInjection.maximumOppositeIliacMassMg < 1e-6,
    'a slow sheath injection should be carried down the cannulated iliac'
);
assert.ok(
    fastRetrogradeInjection.maximumOppositeIliacMassMg > 1e-3,
    'a pressure-dominant sheath injection should reach the opposite iliac through signed retrograde flow'
);
assert.ok(
    fastRetrogradeInjection.firstOppositeIliacMassSeconds >
        fastRetrogradeInjection.firstSourceNetworkMassSeconds + 0.05,
    `opposite-iliac filling must follow finite transport time instead of teleporting (${JSON.stringify(fastRetrogradeInjection)})`
);
assert.equal(
    slowRetrogradeInjection.metrics.retrogradeColumn.activationCount,
    0,
    'sub-dominant sheath pressure must not reverse the iliac flow'
);
assert.equal(
    slowRetrogradeInjection.metrics.continuousFlowSplit.maximumReversedEdgeCount,
    0,
    'sub-dominant injection should remain antegrade without a mode switch'
);
assert.ok(
    fastRetrogradeInjection.metrics.continuousFlowSplit.maximumReversedEdgeCount > 0,
    'the same continuous face-flow equation should cross zero for a dominant injection'
);
assert.equal(
    fastRetrogradeInjection.metrics.retrogradeColumn.activationCount,
    0,
    'a fast sheath injection must stay on the shared local-solver path instead of switching models'
);
assert.ok(
    fastRetrogradeInjection.metrics.sourceHandoffIodineMassMg > 0,
    'the shared local solver should hand the sheath iodine into the resolved source vessel'
);
assert.ok(
    Math.abs(fastRetrogradeInjection.metrics.relativeBalanceError) < 1e-5,
    `retrograde-to-antegrade handoff must conserve iodine mass (${JSON.stringify(fastRetrogradeInjection.metrics)})`
);

const selectiveNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const selectiveChildIndex = selectiveNetwork.edges[0].childEdgeIndices[0];
selectiveNetwork.depositIodine(selectiveChildIndex, 0, 30);
for (let step = 0; step < 120; step++) selectiveNetwork.update(1 / 120);
assert.ok(
    selectiveNetwork.edges[0].massMg.every(mass => mass < 1e-12),
    'mixed 1D contrast must not diffuse upstream into unrelated territories'
);
const selectiveRenderer = new ContrastVolumeRenderer({
    flowNetwork: selectiveNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: selectiveNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
selectiveRenderer.update();
assert.equal(selectiveRenderer._flowDisplayConcentration[0], 0,
    'display smoothing must not create upstream staining at a bifurcation');
selectiveRenderer.dispose();

const branchArtifactNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
branchArtifactNetwork.depositIodine(0, branchArtifactNetwork.edges[0].cellCount - 1, 30);
const branchArtifactRenderer = new ContrastVolumeRenderer({
    flowNetwork: branchArtifactNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: branchArtifactNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
branchArtifactRenderer.update();
const inactiveChildIndices = new Set(
    branchArtifactNetwork.edges[0].childEdgeIndices
);
let maximumFalseBranchConcentration = 0;
for (
    let vertexIndex = 0;
    vertexIndex < branchArtifactRenderer._flowVertexConcentration.length;
    vertexIndex++
) {
    if (!inactiveChildIndices.has(
        branchArtifactRenderer._flowVertexConcentrationEdgeIndex[vertexIndex]
    )) continue;
    maximumFalseBranchConcentration = Math.max(
        maximumFalseBranchConcentration,
        branchArtifactRenderer._flowVertexConcentration[vertexIndex]
    );
}
assert.equal(
    maximumFalseBranchConcentration,
    0,
    'an unopacified side branch must not inherit a dark protrusion from its parent'
);
branchArtifactRenderer.dispose();

const junctionEntryNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const junctionParent = junctionEntryNetwork.edges[0];
const [
    emptyChildIndex,
    enteringChildIndex
] = junctionParent.childEdgeIndices;
const enteringChild = junctionEntryNetwork.edges[enteringChildIndex];
junctionEntryNetwork.depositIodine(
    junctionParent.index,
    junctionParent.cellCount - 1,
    junctionParent.volumes[junctionParent.cellCount - 1] * 0.3
);
junctionEntryNetwork.depositIodine(
    junctionParent.index,
    junctionParent.cellCount - 2,
    junctionParent.volumes[junctionParent.cellCount - 2] * 0.3
);
junctionEntryNetwork.depositIodine(
    enteringChildIndex,
    0,
    enteringChild.volumes[0] * 0.3 * 0.0001
);
const junctionEntryRenderer = new ContrastVolumeRenderer({
    flowNetwork: junctionEntryNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: junctionEntryNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
junctionEntryRenderer.update();
let enteringChildJunctionSignal = 0;
let emptyChildJunctionSignal = 0;
for (
    let vertexIndex = 0;
    vertexIndex < junctionEntryRenderer.flowTubeVertexCount;
    vertexIndex++
) {
    if (
        junctionEntryRenderer._flowVertexConcentrationEdgeT[vertexIndex] >
        1e-8
    ) continue;
    const edgeIndex =
        junctionEntryRenderer._flowVertexConcentrationEdgeIndex[vertexIndex];
    if (edgeIndex === enteringChildIndex) {
        enteringChildJunctionSignal = Math.max(
            enteringChildJunctionSignal,
            junctionEntryRenderer._flowVertexConcentration[vertexIndex]
        );
    }
    if (edgeIndex === emptyChildIndex) {
        emptyChildJunctionSignal = Math.max(
            emptyChildJunctionSignal,
            junctionEntryRenderer._flowVertexConcentration[vertexIndex]
        );
    }
}
assert.ok(
    enteringChildJunctionSignal <= 1e-8,
    `a sub-visible trace must not be promoted into a dark junction ring (${enteringChildJunctionSignal})`
);
const junctionParentTerminalDisplay =
    junctionEntryRenderer._flowCellDisplayConcentration[
        junctionEntryRenderer._flowCellOffset[junctionParent.index] +
        junctionParent.cellCount - 1
    ];
assert.ok(
    junctionParentTerminalDisplay > 0.1,
    `the opacified parent should remain visible up to the partitioned tube union (${junctionParentTerminalDisplay})`
);
assert.equal(
    emptyChildJunctionSignal,
    0,
    'junction continuity must not paint a branch that has received no iodine'
);
junctionEntryRenderer._flowCellDisplayConcentration.fill(0);
const visibleJunctionBoundaryIndices = [
    junctionEntryRenderer._flowCellOffset[junctionParent.index] +
        junctionParent.cellCount - 1,
    ...junctionParent.childEdgeIndices.map(
        edgeIndex => junctionEntryRenderer._flowCellOffset[edgeIndex]
    )
];
for (let index = 0; index < visibleJunctionBoundaryIndices.length; index++) {
    junctionEntryRenderer._flowCellDisplayConcentration[
        visibleJunctionBoundaryIndices[index]
    ] = [0.9, 0.3, 0.6][index];
}
junctionEntryRenderer._equalizeRenderedJunctionBoundaryConcentrations(
    junctionEntryRenderer._flowCellDisplayConcentration
);
const equalizedBoundaryConcentrations =
    visibleJunctionBoundaryIndices.map(
        flatIndex =>
            junctionEntryRenderer._flowCellDisplayConcentration[flatIndex]
    );
assert.ok(
    Math.max(...equalizedBoundaryConcentrations) -
        Math.min(...equalizedBoundaryConcentrations) < 1e-7,
    `one physically shared junction must expose one area-weighted display concentration (${equalizedBoundaryConcentrations})`
);
assert.ok(
    equalizedBoundaryConcentrations[0] > 0.3 &&
        equalizedBoundaryConcentrations[0] < 0.9,
    'junction display mixing must smooth a visible node without inventing concentration outside the incident range'
);
junctionEntryRenderer._flowCellDisplayConcentration.fill(0);
junctionEntryRenderer._flowCellDisplayConcentration[
    visibleJunctionBoundaryIndices[0]
] = 0.9;
junctionEntryRenderer._flowCellDisplayConcentration[
    visibleJunctionBoundaryIndices[1]
] = 0.3;
junctionEntryRenderer._equalizeRenderedJunctionBoundaryConcentrations(
    junctionEntryRenderer._flowCellDisplayConcentration
);
assert.equal(
    junctionEntryRenderer._flowCellDisplayConcentration[
        visibleJunctionBoundaryIndices[2]
    ],
    0,
    'mixing visible junction arms must not opacify an empty branch'
);
assert.ok(
    Math.abs(
        junctionEntryRenderer._flowCellDisplayConcentration[
            visibleJunctionBoundaryIndices[0]
        ] -
        junctionEntryRenderer._flowCellDisplayConcentration[
            visibleJunctionBoundaryIndices[1]
        ]
    ) < 1e-7,
    'an empty side branch must not block smooth mixing between the visible parent and continuation'
);
junctionEntryRenderer.dispose();

const washoutBandingSegments = [];
for (let edgeIndex = 0; edgeIndex < 9; edgeIndex++) {
    washoutBandingSegments.push(segment(
        edgeIndex,
        edgeIndex,
        edgeIndex + 1,
        [0, edgeIndex * 2, 0],
        [0, (edgeIndex + 1) * 2, 0],
        4
    ));
}
const washoutBandingNetwork = new ContrastFlowNetwork(
    washoutBandingSegments,
    {
        cardiacOutputMlPerMin: 600,
        axialDispersionMm2PerS: 8
    }
);
const washoutBandingSystem = {
    flowNetwork: washoutBandingNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: washoutBandingNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
};
const lateWashoutConcentrations = [
    0.08,
    0.07,
    0.06,
    0.05,
    0.15,
    0.03,
    0.02,
    0.01,
    0
];
const applyWashoutProfile = concentrations => {
    for (let edgeIndex = 0;
        edgeIndex < washoutBandingNetwork.edges.length;
        edgeIndex++
    ) {
        const edge = washoutBandingNetwork.edges[edgeIndex];
        edge.massMg[0] =
            concentrations[edgeIndex] *
            0.3 *
            edge.volumes[0];
        edge.meanConcentrationMgPerMm3 =
            edge.massMg[0] / edge.totalVolume;
    }
};
const readWashoutDisplay = renderer =>
    washoutBandingNetwork.edges.map(
        (edge, edgeIndex) =>
            renderer._flowCellDisplayConcentration[
                renderer._flowCellOffset[edgeIndex]
            ]
    );
applyWashoutProfile(lateWashoutConcentrations);
const washoutBandingRenderer = new ContrastVolumeRenderer(
    washoutBandingSystem
);
washoutBandingRenderer.update();
const washoutDisplay = readWashoutDisplay(washoutBandingRenderer);
assert.ok(
    washoutDisplay[4] <=
        Math.max(washoutDisplay[3], washoutDisplay[5]) * 1.2,
    `an isolated late-washout segment must blend into the continuous column (${washoutDisplay.slice(3, 6)})`
);
assert.equal(
    washoutDisplay[8],
    0,
    'topology-aware washout smoothing must not paint iodine into an empty downstream segment'
);

applyWashoutProfile([
    0.08,
    0.07,
    0.06,
    0,
    0.04,
    0.03,
    0.02,
    0.01,
    0
]);
washoutBandingRenderer.update();
const closedGapDisplay = readWashoutDisplay(washoutBandingRenderer);
assert.ok(
    closedGapDisplay[3] >
        Math.min(closedGapDisplay[2], closedGapDisplay[4]) * 0.3,
    `a one-cell internal washout gap should be visually closed (${closedGapDisplay.slice(2, 5)})`
);

applyWashoutProfile([
    0.08,
    0.07,
    0.06,
    0,
    0,
    0,
    0.04,
    0.03,
    0
]);
washoutBandingRenderer.update();
const closedMultiCellGapDisplay = readWashoutDisplay(
    washoutBandingRenderer
);
assert.ok(
    closedMultiCellGapDisplay.slice(3, 6).every(
        concentration => concentration > 0
    ),
    `a multi-cell internal washout gap must not split one vascular column (${closedMultiCellGapDisplay.slice(2, 7)})`
);

applyWashoutProfile([
    0,
    0,
    0,
    0,
    0.3,
    0,
    0,
    0,
    0
]);
washoutBandingRenderer.update();
const isolatedResidueDisplay = readWashoutDisplay(
    washoutBandingRenderer
);
assert.equal(
    isolatedResidueDisplay[4],
    0,
    'a one-cell late residue with no connected contrast should not remain as a visible segment'
);
washoutBandingRenderer.dispose();

const radiusValleySegments = [];
const radiusAtNode = nodeIndex =>
    nodeIndex >= 8 && nodeIndex <= 13 ? 0.55 : 4;
for (let edgeIndex = 0; edgeIndex < 21; edgeIndex++) {
    radiusValleySegments.push(segment(
        edgeIndex,
        edgeIndex,
        edgeIndex + 1,
        [0, edgeIndex * 2, 0],
        [0, (edgeIndex + 1) * 2, 0],
        radiusAtNode(edgeIndex),
        radiusAtNode(edgeIndex + 1)
    ));
}
const radiusValleyNetwork = new ContrastFlowNetwork(radiusValleySegments, {
    cardiacOutputMlPerMin: 600
});
const radiusValleyRenderer = new ContrastVolumeRenderer({
    flowNetwork: radiusValleyNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: radiusValleyNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
const minimumCorrectedValleyRadius = Math.min(
    ...radiusValleyRenderer.flowMesh.geometry.attributes.flowRadius.array
);
assert.ok(
    minimumCorrectedValleyRadius > 3,
    `short radius collapse should be regularized throughout the vascular tree (${minimumCorrectedValleyRadius} mm)`
);
radiusValleyRenderer.dispose();

const topologyRenderer = new ContrastVolumeRenderer({
    flowNetwork: network,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: network,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
const topologyRadii =
    topologyRenderer.flowMesh.geometry.attributes.flowRadius.array;
const topologyEdgeIndices =
    topologyRenderer._flowVertexConcentrationEdgeIndex;
const topologyEdgeTs =
    topologyRenderer._flowVertexConcentrationEdgeT;
const renderedBoundaryRadii = (edgeIndex, edgeT) => {
    const values = [];
    for (
        let vertexIndex = 0;
        vertexIndex < topologyRenderer.flowTubeVertexCount;
        vertexIndex++
    ) {
        if (
            topologyEdgeIndices[vertexIndex] === edgeIndex &&
            Math.abs(topologyEdgeTs[vertexIndex] - edgeT) < 1e-6
        ) {
            values.push(topologyRadii[vertexIndex]);
        }
    }
    return values;
};
const topologyParent = network.edges[0];
const topologyChildren = topologyParent.childEdgeIndices.map(
    edgeIndex => network.edges[edgeIndex]
);
assert.equal(
    topologyRenderer.flowJunctionUnionDiagnostics.length,
    1,
    'the synthetic fork should expose one local tube-union diagnostic'
);
const topologyJunctionUnionDiagnostic =
    topologyRenderer.flowJunctionUnionDiagnostics[0];
assert.equal(
    topologyJunctionUnionDiagnostic.geometryKind,
    'implicit-radius-matched-y-union',
    'a true bifurcation must use a fitted implicit Y instead of a sphere or cone'
);
assert.equal(
    topologyJunctionUnionDiagnostic.signalMode,
    'implicit-maximum-optical-union',
    'the fitted Y must render as one optical volume without additive node overlap'
);
assert.ok(
    topologyJunctionUnionDiagnostic.unionLengthMm >=
        topologyJunctionUnionDiagnostic.maximumIncidentRadius * 1.349,
    'the partitioned-union zone should cover the physical overlap of the incident tubes'
);
assert.equal(
    topologyJunctionUnionDiagnostic.armRadii.length,
    1 + topologyChildren.length,
    'the Y union should have one radius-matched arm per incident vessel'
);
assert.ok(
    Math.abs(
        Math.max(...topologyJunctionUnionDiagnostic.armRadii) -
        topologyJunctionUnionDiagnostic.maximumIncidentRadius
    ) < 1e-9,
    'at least one union arm should retain the full widest incident calibre'
);
assert.equal(
    topologyRenderer.flowTubeVertexCount +
        topologyRenderer.flowJunctionConnectorVertexCount,
    topologyRenderer._flowVertexConcentration.length,
    'the fitted connector vertices and anatomical tube vertices should account for the complete flow mesh'
);
assert.ok(
    topologyRenderer.flowJunctionConnectorVertexCount > 0 &&
        topologyJunctionUnionDiagnostic.connectorVertexCount > 0,
    'the fork must contain a real radius-matched Y surface rather than an open tube end'
);
assert.ok(
    topologyRenderer.flowJunctionIndexCount > 0,
    'the anatomical tube segments around the fork should be assigned to the partitioned-union material'
);
const parentExitRadii = renderedBoundaryRadii(
    topologyParent.index,
    1
);
assert.ok(parentExitRadii.length >= 24);
assert.ok(
    Math.min(...parentExitRadii) >= topologyParent.radiusEnd * 0.999,
    'topology-aware smoothing must preserve the encoded parent radius at a bifurcation'
);
for (const child of topologyChildren) {
    const childEntryRadii = renderedBoundaryRadii(child.index, 0);
    assert.ok(
        childEntryRadii.length >= 24,
        `every bifurcation child ${child.index} must start its own symmetric ostium ring`
    );
    assert.ok(
        Math.min(...childEntryRadii) >= child.radiusStart * 0.999,
        `topology-aware smoothing must preserve child ${child.index} ostium calibre`
    );
}
assert.equal(
    topologyRenderer.flowTopologyRadiusAnchors.length,
    1 + topologyChildren.length,
    'a binary fork should anchor the parent exit and both child entries'
);
const topologyAnchorNodeIds = new Set(
    topologyRenderer.flowTopologyRadiusAnchors.map(anchor => anchor.nodeId)
);
assert.equal(
    topologyAnchorNodeIds.size,
    1,
    'all incident rings of one bifurcation should share one optical-union node'
);
assert.ok(
    topologyRenderer.flowTopologyRadiusAnchors.every(
        anchor => anchor.incidentCount === 1 + topologyChildren.length
    ),
    'the junction optical union should include its parent and every child'
);
assert.ok(
    topologyRenderer.flowTopologyRadiusAnchors.every(
        anchor => Math.abs(anchor.opticalWeight - 1) < 1e-9
    ),
    'every fork arm should retain full optical strength because maximum blending, not energy partitioning, removes overlap'
);
const topologyOpticalWeightAttribute =
    topologyRenderer.flowMesh.geometry.attributes.flowOpticalWeight.array;
const topologyJunctionSlot =
    topologyRenderer.flowJunctionUnionDiagnostics.findIndex(
        diagnostic => diagnostic.parentEdgeIndex === topologyParent.index
    );
assert.ok(topologyJunctionSlot >= 0);
assert.equal(
    topologyRenderer._flowDynamicOpticalVertexIndices.length,
    0,
    'maximum-union junctions should not need concentration-dependent optical rescaling'
);
const parentExitOpticalWeights = [];
for (
    let vertexIndex = 0;
    vertexIndex < topologyRenderer.flowTubeVertexCount;
    vertexIndex++
) {
    if (
        topologyEdgeIndices[vertexIndex] === topologyParent.index &&
        Math.abs(topologyEdgeTs[vertexIndex] - 1) < 1e-6
    ) {
        parentExitOpticalWeights.push(
            topologyOpticalWeightAttribute[vertexIndex]
        );
    }
}
assert.ok(
    parentExitOpticalWeights.length >= 24 &&
        parentExitOpticalWeights.every(weight => weight > 0.999),
    `the parent tube must remain full strength under maximum union blending (${parentExitOpticalWeights})`
);
const parentConnectorOpticalWeights = [];
for (
    let vertexIndex = topologyRenderer.flowTubeVertexCount;
    vertexIndex < topologyRenderer._flowVertexConcentration.length;
    vertexIndex++
) {
    if (
        topologyEdgeIndices[vertexIndex] === topologyParent.index &&
        topologyRenderer._flowVertexIsJunctionConnector[vertexIndex]
    ) {
        parentConnectorOpticalWeights.push(
            topologyOpticalWeightAttribute[vertexIndex]
        );
    }
}
assert.ok(
    parentConnectorOpticalWeights.length > 12 &&
        Math.max(...parentConnectorOpticalWeights) > 0.999,
    'the closed Y surface must take over at full strength when only the parent contains the bolus front'
);
assert.ok(
    Math.max(...parentConnectorOpticalWeights) > 0.999,
    'the fitted Y must preserve its full-strength outer contour after every arm fills'
);
const topologyIndices = topologyRenderer.flowMesh.geometry.index.array;
const topologyOpticalWeights =
    topologyRenderer.flowMesh.geometry.attributes.flowOpticalWeight.array;
let minimumTopologyTubeOpticalWeight = Infinity;
let maximumTopologyTubeOpticalWeight = -Infinity;
for (
    let indexOffset = 0;
    indexOffset < topologyRenderer.flowTubeIndexCount;
    indexOffset++
) {
    const opticalWeight =
        topologyOpticalWeights[topologyIndices[indexOffset]];
    minimumTopologyTubeOpticalWeight = Math.min(
        minimumTopologyTubeOpticalWeight,
        opticalWeight
    );
    maximumTopologyTubeOpticalWeight = Math.max(
        maximumTopologyTubeOpticalWeight,
        opticalWeight
    );
}
assert.ok(
    minimumTopologyTubeOpticalWeight > 0.999,
    'ordinary tube segments should retain their physical optical depth'
);
assert.ok(
    maximumTopologyTubeOpticalWeight > 0.999,
    'ordinary tube portions must recover full optical depth away from the fork'
);
assert.equal(
    topologyRenderer.flowTubeMaterial.blending,
    THREE.AdditiveBlending,
    'independent vessels crossing only in projection must remain additive'
);
let minimumTopologyJunctionOpticalWeight = Infinity;
let maximumTopologyJunctionOpticalWeight = -Infinity;
for (
    let indexOffset = topologyRenderer.flowTubeIndexCount;
    indexOffset < topologyIndices.length;
    indexOffset++
) {
    const opticalWeight =
        topologyOpticalWeights[topologyIndices[indexOffset]];
    minimumTopologyJunctionOpticalWeight = Math.min(
        minimumTopologyJunctionOpticalWeight,
        opticalWeight
    );
    maximumTopologyJunctionOpticalWeight = Math.max(
        maximumTopologyJunctionOpticalWeight,
        opticalWeight
    );
}
assert.ok(
    minimumTopologyJunctionOpticalWeight > 0.999,
    'maximum blending should allow every true-fork surface to retain full physical optical depth'
);
assert.ok(
    maximumTopologyJunctionOpticalWeight > 0.9,
    'the smooth Y connector should retain full optical depth under max blending'
);
assert.equal(
    topologyRenderer.flowJunctionMaterial.blending,
    THREE.CustomBlending,
    'true-fork tube shoulders must use the same non-additive optical union as the connector'
);
assert.equal(
    topologyRenderer.flowJunctionMaterial.blendEquation,
    THREE.MaxEquation,
    'true-fork tube shoulders must take the strongest signal instead of producing a bead'
);
assert.equal(
    topologyRenderer.flowJunctionMaterial.blendSrc,
    THREE.OneFactor,
    'maximum-union segments must consume the shader-premultiplied optical transfer directly'
);
assert.equal(
    topologyRenderer.flowJunctionMaterial.uniforms.maximumBlend.value,
    true,
    'maximum-union materials must premultiply optical transfer in the shader because WebGL MAX ignores blend factors'
);
assert.equal(
    topologyRenderer.flowJunctionConnectorMaterial.blending,
    THREE.CustomBlending,
    'the single implicit Y volume should use a dedicated optical-union blend'
);
assert.equal(
    topologyRenderer.flowJunctionConnectorMaterial.blendEquation,
    THREE.MaxEquation,
    'overlapping faces of one fitted Y must take the maximum optical depth instead of adding a bead'
);
assert.equal(
    topologyRenderer.flowJunctionMaterial.uniforms.signalGain.value,
    topologyRenderer.flowTubeMaterial.uniforms.signalGain.value,
    'junction and ordinary segments must have identical signal gain to avoid dark node bands'
);
topologyRenderer.setDebugMode(true);
assert.equal(
    topologyRenderer.flowJunctionMaterial.blending,
    THREE.CustomBlending,
    'debug overlay should retain the true-fork maximum union'
);
topologyRenderer.setDebugMode(false);
assert.equal(
    topologyRenderer.flowJunctionMaterial.blending,
    THREE.CustomBlending
);
topologyRenderer.dispose();

const sideBranchNetwork = new ContrastFlowNetwork(sideBranchTree(), {
    cardiacOutputMlPerMin: 900
});
const sideBranchRenderer = new ContrastVolumeRenderer({
    flowNetwork: sideBranchNetwork,
    localSolver: new LocalContrastInjectionSolver({
        flowNetwork: sideBranchNetwork,
        capacity: 16
    }),
    medium: { iodineMgPerMl: 300 }
});
const sideBranchJunction =
    sideBranchRenderer.flowJunctionUnionDiagnostics[0];
assert.ok(
    sideBranchJunction.continuationAlignment > 0.999,
    'the straight child should be detected as a continuation of the parent'
);
assert.ok(
    sideBranchJunction.signalMode === 'implicit-maximum-optical-union',
    'every side branch should render as one non-additive optical volume'
);
assert.equal(
    sideBranchJunction.geometryKind,
    'implicit-radius-matched-side-ostium-union',
    'a side branch must replace intersecting tubes with a fitted ostium surface'
);
assert.equal(
    sideBranchJunction.surfaceMode,
    'exact-union-smoothed-normals',
    'the fitted ostium should smooth shading without expanding beyond the anatomical tube union'
);
assert.ok(
    sideBranchJunction.connectorVertexCount > 12,
    'a side branch must contribute a non-degenerate saddle-shaped connector'
);
assert.ok(
    sideBranchJunction.maximumOutwardErrorMm <=
        sideBranchJunction.minimumIncidentRadius * 0.08,
    `the fitted side ostium escapes its exact lumen envelope by ${sideBranchJunction.maximumOutwardErrorMm} mm`
);
assert.equal(
    sideBranchRenderer.flowSideOstiumMaterial.blending,
    THREE.CustomBlending,
    'side-ostium tube shoulders must use maximum blending'
);
assert.equal(
    sideBranchRenderer.flowSideOstiumMaterial.blendEquation,
    THREE.MaxEquation,
    'side-ostium tube shoulders must not add into an external black contour'
);
assert.equal(
    sideBranchRenderer.flowSideOstiumConnectorMaterial.blending,
    THREE.CustomBlending,
    'the clipped side-ostium connector must use the same maximum optical union'
);
const sideBranchOpticalWeights =
    sideBranchRenderer.flowTopologyRadiusAnchors
        .map(anchor => anchor.opticalWeight)
        .sort((a, b) => a - b);
assert.ok(
    sideBranchOpticalWeights.length === 2 &&
        sideBranchOpticalWeights.every(
            weight => Math.abs(weight - 1) < 1e-9
        ),
    `the main tube and lateral ostium should retain full strength under maximum blending (${sideBranchOpticalWeights})`
);
sideBranchRenderer.dispose();

const makeStraightNetwork = () => new ContrastFlowNetwork(straightTree(), {
    cardiacOutputMlPerMin: 600,
    axialDispersionMm2PerS: 8
});
const fixedStepNetwork = makeStraightNetwork();
const variableStepNetwork = makeStraightNetwork();
fixedStepNetwork.depositIodine(0, 0, 80);
variableStepNetwork.depositIodine(0, 0, 80);
for (let step = 0; step < 120; step++) fixedStepNetwork.update(1 / 120);
for (let step = 0; step < 40; step++) {
    variableStepNetwork.update(1 / 90);
    variableStepNetwork.update(1 / 180);
}
const fixedMass = fixedStepNetwork.getIodineMassMg();
const variableMass = variableStepNetwork.getIodineMassMg();
assert.ok(
    Math.abs(fixedMass - variableMass) / Math.max(1, fixedMass) < 0.05,
    'transport should remain stable under a variable frame timestep'
);

const localNetwork = makeStraightNetwork();
const localA = new LocalContrastInjectionSolver({
    flowNetwork: localNetwork,
    randomSeed: 1234
});
const localB = new LocalContrastInjectionSolver({
    flowNetwork: makeStraightNetwork(),
    randomSeed: 1234
});
const testPort = {
    position: new THREE.Vector3(0, 2, 0),
    direction: new THREE.Vector3(0, 1, 0),
    radiusMm: 0.5,
    areaMm2: Math.PI * 0.25,
    weight: 1
};
localA.emit({
    ports: [testPort],
    volumeMl: 1,
    rateMlPerSec: 2,
    emissionWindowSeconds: 1 / 120
});
localB.emit({
    ports: [testPort],
    volumeMl: 1,
    rateMlPerSec: 2,
    emissionWindowSeconds: 1 / 120
});
localA.update(1 / 120);
localB.update(1 / 120);
assert.equal(localA.count, localB.count);
assert.ok(localA.count >= 300,
    'the local plume needs enough parcels to render as a continuous volume');
let minimumPlumeY = Infinity;
let maximumPlumeY = -Infinity;
for (let index = 0; index < localA.count; index++) {
    minimumPlumeY = Math.min(minimumPlumeY, localA.positionY[index]);
    maximumPlumeY = Math.max(maximumPlumeY, localA.positionY[index]);
}
assert.ok(maximumPlumeY - minimumPlumeY > 4,
    'sub-frame emission timing must prevent discrete axial particle shells');
assert.deepEqual(
    [...localA.positionY.subarray(0, localA.count)],
    [...localB.positionY.subarray(0, localB.count)],
    'the local 3D plume must be deterministic for a fixed seed'
);
const localFluoroRenderer = new ContrastVolumeRenderer({
    flowNetwork: localNetwork,
    localSolver: localA,
    medium: { iodineMgPerMl: 300 }
});
localFluoroRenderer.setDebugMode(false);
localFluoroRenderer.update();
assert.ok(
    Math.max(...localFluoroRenderer._flowCellDisplayConcentration) > 0.05,
    'local catheter parcels must be projected into the fluoroscopic lumen before 1D handoff'
);
assert.equal(
    localNetwork.getIodineMassMg(),
    0,
    'fluoroscopic plume projection must not duplicate iodine in the physical flow network'
);
assert.equal(
    localFluoroRenderer.flowMesh.visible,
    true,
    'the injected stream must be visible in fluoroscopy while it remains local'
);
assert.equal(
    localFluoroRenderer.plumeMesh.visible,
    false,
    'fluoroscopy should render the local stream as an opacified lumen instead of debug streaks'
);
localFluoroRenderer.setDebugMode(true);
assert.equal(
    localFluoroRenderer.plumeMesh.visible,
    true,
    'debug mode should retain directional plume streaks'
);
localFluoroRenderer.dispose();
for (let step = 0; step < 300; step++) {
    localA.update(1 / 120);
    localNetwork.update(1 / 120);
}
const localAccountedMass =
    localA.getIodineMassMg() +
    localNetwork.getIodineMassMg() +
    localNetwork.outletIodineMassMg;
assert.ok(Math.abs(localAccountedMass - 300) < 1e-3,
    '3D to 1D handoff must not duplicate or lose iodine');

const bulkRefluxNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const bulkRefluxSolver = new LocalContrastInjectionSolver({
    flowNetwork: bulkRefluxNetwork,
    randomSeed: 20260802
});
const bulkRefluxSource = bulkRefluxNetwork.edges[0].childEdgeIndices
    .map(edgeIndex => bulkRefluxNetwork.edges[edgeIndex])
    .sort((left, right) =>
        right.meanFlowMm3PerS - left.meanFlowMm3PerS
    )[0];
const bulkRefluxPort = {
    kind: 'berenstein-end',
    position: bulkRefluxSource.start.clone().lerp(
        bulkRefluxSource.end,
        0.6
    ),
    direction: bulkRefluxSource.axis.clone().multiplyScalar(-1),
    radiusMm: 0.4,
    areaMm2: Math.PI * 0.16,
    weight: 1
};
bulkRefluxSolver.emit({
    ports: [bulkRefluxPort],
    volumeMl: 20,
    rateMlPerSec: 20,
    emissionWindowSeconds: 1
});
bulkRefluxSolver.update(1 / 240);
bulkRefluxNetwork.update(1 / 240);
assert.equal(
    bulkRefluxSolver.retrogradeHandoffIodineMassMg,
    0,
    'the retrograde front should still be inside the local jet before reaching its mixing target'
);
assert.equal(
    bulkRefluxSolver.retrogradeEntrainedIodineMassMg,
    0,
    'a coherent catheter jet must not shed an artificial dilute precursor'
);
assert.ok(
    bulkRefluxNetwork.getIodineMassMg() < 1e-12,
    'the vascular model must remain empty until the coherent front reaches the handoff target'
);
for (let step = 0; step < 480; step++) {
    bulkRefluxSolver.update(1 / 120);
    bulkRefluxNetwork.update(1 / 120);
}
assert.equal(
    bulkRefluxSolver.retrogradeTargetedIodineMassMg,
    0,
    'a narrow jet must not receive a remote target beyond its outlet-scale entrainment length'
);
assert.ok(
    bulkRefluxSolver.maximumRetrogradeProgressMm <=
        bulkRefluxSource.radiusStart * 4,
    `the resolved plume should mix locally instead of shooting through the parent vessel (${bulkRefluxSolver.maximumRetrogradeProgressMm} mm)`
);
assert.equal(
    bulkRefluxSolver.count,
    0,
    'no detached local parcels may remain after the outlet-scale jet has mixed'
);
assert.ok(
    bulkRefluxSolver.sourceHandoffIodineMassMg > 5999,
    'a sub-dominant catheter jet should join the catheterized branch locally'
);

const longRetrogradeTree = [
    segment(0, 0, 1, [0, 0, 0], [0, 80, 0], 10, 9),
    segment(1, 1, 2, [0, 80, 0], [-18, 280, 0], 3.2, 3),
    segment(2, 1, 3, [0, 80, 0], [18, 280, 0], 3.2, 3)
];
const longRetrogradeNetwork = new ContrastFlowNetwork(longRetrogradeTree, {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const longRetrogradeSolver = new LocalContrastInjectionSolver({
    flowNetwork: longRetrogradeNetwork,
    randomSeed: 20260804
});
const longRetrogradeSource = longRetrogradeNetwork.edges[1];
const longRetrogradePort = {
    kind: 'berenstein-end',
    position: longRetrogradeSource.start.clone().lerp(
        longRetrogradeSource.end,
        0.72
    ),
    direction: longRetrogradeSource.axis.clone().multiplyScalar(-1),
    radiusMm: 0.4,
    areaMm2: Math.PI * 0.16,
    weight: 1
};
const longRetrogradeLocation = longRetrogradeNetwork.findInjectionLocation(
    longRetrogradePort.position,
    longRetrogradePort.direction
);
const longRetrogradeTarget = longRetrogradeNetwork.findUpstreamMixingJunction(
    longRetrogradeLocation,
    {
        minimumParentRadiusMm: longRetrogradeLocation.radius * 2 * 0.425,
        minimumChildRadiusMm: longRetrogradeLocation.radius * 2 * 0.275,
        preferFarthest: true
    }
);
assert.ok(
    longRetrogradeTarget,
    'the long catheter path should expose a physical upstream mixing junction'
);
longRetrogradeSolver.emit({
    ports: [longRetrogradePort],
    volumeMl: 0.34,
    rateMlPerSec: 10.3,
    emissionWindowSeconds: 1 / 30
});
const longTargetedParticle = longRetrogradeSolver.preferredHandoffEdgeIndex
    .findIndex(edgeIndex => edgeIndex === longRetrogradeTarget.edgeIndex);
assert.equal(
    longTargetedParticle,
    -1,
    'a distant junction must not extend the lifetime of a narrow catheter jet'
);
for (let step = 0; step < 180; step++) {
    longRetrogradeSolver.update(1 / 240);
}
assert.equal(
    longRetrogradeSolver.count,
    0,
    'the local jet must be fully entrained within 0.75 s'
);
assert.ok(
    longRetrogradeSolver.maximumRetrogradeProgressMm <
        longRetrogradeTarget.distanceMm * 0.2,
    `the outlet-scale plume must remain far short of the remote junction (${longRetrogradeSolver.maximumRetrogradeProgressMm} of ${longRetrogradeTarget.distanceMm} mm)`
);

const expiringRetrogradeTree = [
    segment(0, 0, 1, [0, 0, 0], [0, 80, 0], 10, 9),
    segment(1, 1, 2, [0, 80, 0], [-18, 400, 0], 3.2, 3),
    segment(2, 1, 3, [0, 80, 0], [18, 400, 0], 3.2, 3),
    segment(3, 3, 4, [18, 400, 0], [18, 1000, 0], 3),
    segment(4, 4, 5, [18, 1000, 0], [18, 1100, 0], 3)
];
const expiringRetrogradeNetwork = new ContrastFlowNetwork(
    expiringRetrogradeTree,
    {
        cardiacOutputMlPerMin: 900,
        axialDispersionMm2PerS: 8
    }
);
const expiringRetrogradeSolver = new LocalContrastInjectionSolver({
    flowNetwork: expiringRetrogradeNetwork,
    randomSeed: 20260805
});
const expiringRetrogradeSource = expiringRetrogradeNetwork.edges[1];
const expiringRetrogradePort = {
    kind: 'berenstein-end',
    position: expiringRetrogradeSource.start.clone().lerp(
        expiringRetrogradeSource.end,
        0.72
    ),
    direction: expiringRetrogradeSource.axis.clone().multiplyScalar(-1),
    radiusMm: 0.4,
    areaMm2: Math.PI * 0.16,
    weight: 1
};
expiringRetrogradeSolver.emit({
    ports: [expiringRetrogradePort],
    volumeMl: 0.1,
    rateMlPerSec: 29.1,
    emissionWindowSeconds: 1 / 30
});
// Curved anatomy can leave a pressure-driven target just beyond the actual
// jet trajectory even though its scalar penetration estimate supported it.
// Force that generic terminal state without depending on one asset edge id.
for (let index = 0; index < expiringRetrogradeSolver.count; index++) {
    expiringRetrogradeSolver.preferredHandoffEdgeIndex[index] = 4;
    expiringRetrogradeSolver.preferredHandoffCellIndex[index] =
        expiringRetrogradeNetwork.edges[4].cellCount - 1;
    expiringRetrogradeSolver.handoffDistanceMm[index] = 10000;
}
for (let step = 0; step < 120; step++) {
    expiringRetrogradeSolver.update(1 / 240);
}
assert.equal(
    expiringRetrogradeSolver.count,
    0,
    'all local parcels should mix into the vascular model within 0.5 s'
);
assert.equal(
    expiringRetrogradeSolver.retrogradeHandoffIodineMassMg,
    0,
    'an unreachable remote target must not receive teleported iodine'
);
assert.equal(
    expiringRetrogradeSolver.sourceHandoffIodineMassMg,
    0,
    'an expired targeted jet must not jump back to the catheter source cell'
);
assert.ok(
    expiringRetrogradeNetwork.getIodineMassMg() > 29,
    'entrained parcels should mix where their visible front actually stopped'
);
assert.ok(
    expiringRetrogradeSolver.retrogradeEntrainedIodineMassMg > 29,
    'unreachable targeted parcels should be continuously entrained instead of surviving as late fragments'
);

const selectiveRefluxNetwork = new ContrastFlowNetwork(branchingTree(), {
    cardiacOutputMlPerMin: 900,
    axialDispersionMm2PerS: 8
});
const selectiveRefluxSolver = new LocalContrastInjectionSolver({
    flowNetwork: selectiveRefluxNetwork,
    randomSeed: 20260803
});
const selectiveRefluxSource = selectiveRefluxNetwork.edges[0].childEdgeIndices
    .map(edgeIndex => selectiveRefluxNetwork.edges[edgeIndex])
    .sort((left, right) =>
        left.meanFlowMm3PerS - right.meanFlowMm3PerS
    )[0];
assert.ok(
    selectiveRefluxSource,
    'the test network should expose a lower-flow selective branch'
);
selectiveRefluxSolver.emit({
    ports: [{
        kind: 'berenstein-end',
        position: selectiveRefluxSource.start.clone().lerp(
            selectiveRefluxSource.end,
            0.6
        ),
        direction: selectiveRefluxSource.axis.clone(),
        radiusMm: 0.4,
        areaMm2: Math.PI * 0.16,
        weight: 1
    }],
    volumeMl: 20,
    rateMlPerSec: 22.6,
    emissionWindowSeconds: 0.88
});
let selectiveSourcePeakMassMg = 0;
let selectiveSiblingPeakMassMg = 0;
const selectiveSibling = selectiveRefluxNetwork.edges[
    selectiveRefluxNetwork.edges[0].childEdgeIndices.find(
        edgeIndex => edgeIndex !== selectiveRefluxSource.index
    )
];
for (let step = 0; step < 480; step++) {
    selectiveRefluxSolver.update(1 / 120);
    selectiveRefluxNetwork.update(1 / 120);
    selectiveSourcePeakMassMg = Math.max(
        selectiveSourcePeakMassMg,
        ...selectiveRefluxSource.massMg
    );
    selectiveSiblingPeakMassMg = Math.max(
        selectiveSiblingPeakMassMg,
        ...selectiveSibling.massMg
    );
}
assert.equal(
    selectiveRefluxSolver.retrogradeTargetedIodineMassMg,
    0,
    'injection rate alone must not manufacture a retrograde target'
);
assert.ok(
    selectiveSourcePeakMassMg > 0,
    'the selectively catheterized branch should receive the bolus'
);
assert.ok(
    selectiveRefluxNetwork.edges[0].massMg.every(mass => mass < 1e-12) &&
        selectiveSiblingPeakMassMg < 1e-12,
    'a selective branch bolus must not fill the upstream aorta or the sibling branch'
);

const catheterPort = {
    kind: 'berenstein-end',
    position: new THREE.Vector3(0, 10, 0),
    direction: new THREE.Vector3(0, 1, 0),
    radiusMm: 0.4,
    areaMm2: Math.PI * 0.16,
    weight: 1
};
const catheter = {
    type: 'berenstein',
    getInjectionPorts(out) {
        out.length = 0;
        out.push(catheterPort);
        return out;
    }
};
const hybrid = new HybridContrastSystem({
    centerlineSegments: straightTree(),
    sheath: {
        start: new THREE.Vector3(0, -20, 0),
        end: new THREE.Vector3(0, 0, 0),
        radius: 1
    },
    catheter,
    hemodynamics: {
        cardiacOutputMlPerMin: 600,
        axialDispersionMm2PerS: 8
    }
});
assert.equal(hybrid.getSourceStatus(CONTRAST_SOURCE_SHEATH).valid, true);
assert.equal(hybrid.getSourceStatus(CONTRAST_SOURCE_CATHETER).valid, true);
assert.equal(hybrid.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 1,
    rateMlPerSec: 2
}).ok, true);
for (let step = 0; step < 120; step++) hybrid.update(1 / 120);
const metrics = hybrid.getMetrics();
assert.ok(Math.abs(metrics.totalDeliveredVolumeMl - 1) < 1e-6);
assert.ok(Math.abs(metrics.relativeBalanceError) < 1e-5);
assert.ok(
    metrics.catheterSourceHandoffIodineMassMg >=
        metrics.totalInjectedIodineMassMg * 0.99,
    'catheter iodine should enter the arterial network at its source-aligned segment'
);
assert.equal(metrics.lastCatheterSourceEdgeIndex, 0);
assert.equal(
    metrics.lastCatheterSourceSelectionMode,
    'containing-lumen',
    'catheter diagnostics should report the containing-lumen source selection'
);
assert.equal(metrics.injection, null);

catheterPort.position.set(0, -5, 0);
assert.equal(hybrid.getSourceStatus(CONTRAST_SOURCE_CATHETER).valid, false,
    'catheter injection must be unavailable while its port remains inside the sheath');

const portCatheter = new PigtailCatheter({
    wire: { nodes: [] },
    segmentLength: 4,
    guidewireLength: 100,
    tailProgressRef: () => 0,
    vessel: null
});
const catheterBody = {
    x: new Float64Array(10),
    y: new Float64Array(10),
    z: new Float64Array(10)
};
for (let index = 0; index < 10; index++) catheterBody.y[index] = index * 4;
portCatheter.physicsBody = catheterBody;
portCatheter.physicsActiveCount = 10;
const pigtailPorts = portCatheter.getInjectionPorts([]);
assert.equal(pigtailPorts.length, 8, 'pigtail should expose eight distributed side ports');
for (const port of pigtailPorts) {
    assert.ok(Math.abs(port.direction.y) < 1e-6,
        'pigtail side-port jets should leave radially rather than along the shaft');
}
portCatheter.physicsBody = null;
portCatheter.setType('berenstein');
portCatheter.physicsBody = catheterBody;
portCatheter.physicsActiveCount = 10;
const berensteinPorts = portCatheter.getInjectionPorts([]);
assert.equal(berensteinPorts.length, 1, 'Berenstein should expose one distal end port');
assert.ok(berensteinPorts[0].direction.y > 0.99,
    'Berenstein end-port jet should follow the distal shaft tangent');
portCatheter.dispose();

const volumeRenderer = new ContrastVolumeRenderer(hybrid);
assert.equal(volumeRenderer.flowMesh.isMesh, true,
    'vascular iodine must use one connected lumen surface');
assert.ok(volumeRenderer.flowMesh.geometry.attributes.normal,
    'the lumen shader requires radial normals to integrate its optical chord');
assert.ok(volumeRenderer.flowMesh.geometry.attributes.flowRadius,
    'the lumen optical depth must scale with the physical vessel radius');
assert.ok(volumeRenderer.flowMesh.geometry.attributes.flowOpticalWeight,
    'junction union geometry must encode its physical optical depth');
assert.ok(volumeRenderer.flowMesh.geometry.index.count > 0,
    'the vascular lumen must contain connected triangles');
assert.ok(volumeRenderer.flowRingCount > hybrid.flowNetwork.edges.length,
    'edge boundaries must share rings so the vascular column remains continuous');
assert.equal(volumeRenderer.plumeMesh.isMesh, true,
    'the local injection plume must render as directional streak quads');
assert.equal(
    volumeRenderer.plumeMesh.geometry.isInstancedBufferGeometry,
    true,
    'the local plume must avoid circular point sprites and visible spheres'
);
volumeRenderer.dispose();

console.log('hybrid contrast model tests passed');
