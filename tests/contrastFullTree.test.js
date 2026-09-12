import assert from 'node:assert/strict';
import fs from 'node:fs';
import { performance } from 'node:perf_hooks';
import * as THREE from 'three';
import {
    HybridContrastSystem,
    CONTRAST_SOURCE_CATHETER,
    CONTRAST_SOURCE_SHEATH
} from '../src/contrast/hybridContrastSystem.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { VesselContactField } from '../src/physics/collision/vesselContactField.js';
import { buildCenterlineFlowArrowSamples } from '../src/vesselBroadPhase.js';
import { findAortoiliacDebugAnchors } from '../src/anatomyDebugLabels.js';

const sourceBuffer = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
const arrayBuffer = sourceBuffer.buffer.slice(
    sourceBuffer.byteOffset,
    sourceBuffer.byteOffset + sourceBuffer.byteLength
);
const asset = decodeCollisionAsset(arrayBuffer);
const data = asset.arrays.centerlineSegments;
const nodeEdges = asset.arrays.centerlineEdges;
const stride = asset.metadata.centerline.stride;
const centerlineSegments = [];
for (let index = 0; index < data.length / stride; index++) {
    const offset = index * stride;
    centerlineSegments.push({
        id: index,
        start: new THREE.Vector3(data[offset], data[offset + 1], data[offset + 2]),
        end: new THREE.Vector3(data[offset + 3], data[offset + 4], data[offset + 5]),
        radiusStart: data[offset + 6],
        radiusEnd: data[offset + 7],
        safeRadius: data[offset + 8],
        nodeStartId: nodeEdges[index * 2],
        nodeEndId: nodeEdges[index * 2 + 1]
    });
}

const system = new HybridContrastSystem({
    centerlineSegments,
    contactField: new VesselContactField(asset),
    sheath: {
        start: new THREE.Vector3(-89.014, -447.056, 28.020),
        end: new THREE.Vector3(-73, -383, 14),
        radius: 1
    },
    localOptions: {
        capacity: 4000,
        randomSeed: 20260725
    }
});
const topology = system.flowNetwork.getTopologyDiagnostics();
const aortoiliacAnchors = findAortoiliacDebugAnchors(system.flowNetwork);
assert.ok(
    aortoiliacAnchors?.rightInternalIliacRootEdgeIndex >= 0,
    'the full anatomy should expose the patient-right internal iliac root'
);
assert.equal(
    topology.sourceSegmentCount,
    asset.metadata.centerline.segmentCount,
    'every packed centerline segment must enter the contrast network'
);
assert.equal(topology.directedEdgeCount, asset.metadata.centerline.segmentCount);
assert.equal(topology.nodeCount, asset.metadata.centerline.nodeCount);
assert.ok(topology.outletCount >= 100, 'the contrast tree should retain distal arterial outlets');
assert.equal(
    topology.disconnectedSourceSegmentCount,
    0,
    'the complete packed STL centerline tree must drive contrast flow'
);
const fullTreeFlowArrowSamples = buildCenterlineFlowArrowSamples(
    system.flowNetwork.edges
);
assert.ok(
    fullTreeFlowArrowSamples.length > 300,
    'the debug view should distribute small flow arrows throughout the full packed arterial tree'
);
for (const sample of fullTreeFlowArrowSamples) {
    const edge = system.flowNetwork.edges[sample.edgeIndex];
    assert.ok(
        sample.direction.dot(edge.axis) > 0.999999,
        'full-tree debug arrows must point away from the heart along directed flow edges'
    );
}
console.log('full-tree centerline flow arrows', fullTreeFlowArrowSamples.length);

for (const node of system.flowNetwork.nodes.values()) {
    if (!node.childEdgeIndices.length) continue;
    const incoming = node.id === system.flowNetwork.rootNode.id
        ? system.flowNetwork.hemodynamics.cardiacOutputMlPerMin * 1000 / 60
        : system.flowNetwork.edges[node.parentEdgeIndex].meanFlowMm3PerS;
    const outgoing = node.childEdgeIndices.reduce(
        (sum, edgeIndex) => sum + system.flowNetwork.edges[edgeIndex].meanFlowMm3PerS,
        0
    );
    assert.ok(Math.abs(incoming - outgoing) / Math.max(1, incoming) < 1e-10,
        `flow must be conserved at centerline node ${node.id}`);
}

const sourceStatus = system.getSourceStatus(CONTRAST_SOURCE_SHEATH);
assert.equal(sourceStatus.valid, true, sourceStatus.reason);
const sheathLocation = system.flowNetwork.findNearestLocation(sourceStatus.ports[0].position);
const iliacMixingTarget =
    system.flowNetwork.findUpstreamMixingJunction(sheathLocation);
assert.ok(iliacMixingTarget, 'the packed anatomy should expose the aortoiliac mixing junction');
assert.ok(
    iliacMixingTarget.distanceMm > 100 && iliacMixingTarget.distanceMm < 180,
    `the aortoiliac junction should be anatomically upstream (${iliacMixingTarget.distanceMm} mm)`
);
const oppositeIliacEdgeIndex = iliacMixingTarget.childEdgeIndices.find(
    edgeIndex => edgeIndex !== iliacMixingTarget.sourceChildEdgeIndex
);
const iliacPathVolumeMl = iliacMixingTarget.pathEdgeIndices.reduce(
    (sum, edgeIndex) => sum + system.flowNetwork.edges[edgeIndex].totalVolume,
    0
) / 1000;
assert.ok(oppositeIliacEdgeIndex >= 0, 'the aortoiliac junction should expose the opposite iliac');
assert.equal(system.startInjection({
    source: CONTRAST_SOURCE_SHEATH,
    volumeMl: 30,
    rateMlPerSec: 15
}).ok, true);
const washoutRenderer = new ContrastVolumeRenderer(system);

const startedAt = performance.now();
const startedCpu = process.cpuUsage();
let maximumOppositeIliacMassMg = 0;
let oppositeIliacMassAfterFirstFrameMg = 0;
let maximumRightInternalIliacMassMg = 0;
let maximumVisibleIliacPathFraction = 0;
let maximumIliacPathMassMg = 0;
let iliacPathMassAfterWashoutMg = 0;
const reachedIliacPathEdges = new Set();
const aorticBranchCoverage = new Map();
const washoutColumnSnapshots = [];
const distalSheathPathCells = [];
let distalEdge = system.flowNetwork.edges[sheathLocation.edgeIndex]
    ?.childEdgeIndices.reduce((best, edgeIndex) => {
        const edge = system.flowNetwork.edges[edgeIndex];
        return !best || edge.meanFlowMm3PerS > best.meanFlowMm3PerS
            ? edge
            : best;
    }, null);
let distalDistanceMm = 0;
const visitedDistalEdges = new Set();
while (
    distalEdge &&
    !visitedDistalEdges.has(distalEdge.index) &&
    distalDistanceMm < 80
) {
    visitedDistalEdges.add(distalEdge.index);
    for (
        let cellIndex = 0;
        cellIndex < distalEdge.cellCount && distalDistanceMm < 80;
        cellIndex++
    ) {
        distalSheathPathCells.push({
            edgeIndex: distalEdge.index,
            cellIndex
        });
        distalDistanceMm += distalEdge.cellLength;
    }
    distalEdge = distalEdge.childEdgeIndices.reduce(
        (best, edgeIndex) => {
            const edge = system.flowNetwork.edges[edgeIndex];
            return !best || edge.meanFlowMm3PerS > best.meanFlowMm3PerS
                ? edge
                : best;
        },
        null
    );
}
let maximumRenderedDistalCoverage = 0;
let maximumDistalRendererSuppression = 0;
const updateDistalSheathCoverage = frame => {
    if (frame % 4 !== 0 || !distalSheathPathCells.length) return;
    washoutRenderer.update();
    let physicalVisibleCells = 0;
    let renderedVisibleCells = 0;
    for (const { edgeIndex, cellIndex } of distalSheathPathCells) {
        const edge = system.flowNetwork.edges[edgeIndex];
        const physicalConcentration = edge.massMg[cellIndex] /
            Math.max(1e-9, edge.volumes[cellIndex]) / 0.3;
        if (physicalConcentration >= 0.02) physicalVisibleCells++;
        if (
            washoutRenderer._flowCellDisplayConcentration[
                washoutRenderer._flowCellOffset[edgeIndex] + cellIndex
            ] >= 0.02
        ) {
            renderedVisibleCells++;
        }
    }
    const physicalCoverage =
        physicalVisibleCells / distalSheathPathCells.length;
    const renderedCoverage =
        renderedVisibleCells / distalSheathPathCells.length;
    maximumRenderedDistalCoverage = Math.max(
        maximumRenderedDistalCoverage,
        renderedCoverage
    );
    if (physicalCoverage >= 0.2) {
        maximumDistalRendererSuppression = Math.max(
            maximumDistalRendererSuppression,
            physicalCoverage - renderedCoverage
        );
    }
};
const washoutSnapshotFrames = new Set([239, 247, 255, 263, 287, 335]);
const captureWashoutColumnSnapshot = frame => {
    if (!washoutSnapshotFrames.has(frame)) return;
    washoutRenderer.update();
    const pathCells =
        system.pressureDrivenRetrogradeColumn.activePath?.cells || [];
    const stockConcentrationMgPerMm3 =
        system.medium.iodineMgPerMl / 1000;
    const concentrations = pathCells.map(({ edgeIndex, cellIndex }) => {
        const edge = system.flowNetwork.edges[edgeIndex];
        return edge.massMg[cellIndex] /
            Math.max(1e-9, edge.volumes[cellIndex]) /
            stockConcentrationMgPerMm3;
    });
    const visible = concentrations.map(value => value >= 0.02);
    const renderedVisible = pathCells.map(({ edgeIndex, cellIndex }) =>
        washoutRenderer._flowCellDisplayConcentration[
            washoutRenderer._flowCellOffset[edgeIndex] + cellIndex
        ] > 0
    );
    const renderedConcentrations = pathCells.map(
        ({ edgeIndex, cellIndex }) =>
            washoutRenderer._flowCellDisplayConcentration[
                washoutRenderer._flowCellOffset[edgeIndex] +
                cellIndex
            ]
    );
    let maximumConcentrationIndex = -1;
    let maximumConcentration = 0;
    for (let index = 0; index < concentrations.length; index++) {
        if (concentrations[index] <= maximumConcentration) continue;
        maximumConcentration = concentrations[index];
        maximumConcentrationIndex = index;
    }
    const maximumCell = pathCells[maximumConcentrationIndex];
    const maximumEdge = maximumCell
        ? system.flowNetwork.edges[maximumCell.edgeIndex]
        : null;
    let visibleCellCount = 0;
    let visibleAfterFirstGap = 0;
    let encounteredGap = false;
    let transitionCount = 0;
    let renderedTransitionCount = 0;
    let renderedVisibleAfterFirstGap = 0;
    let renderedVisibleCellCount = 0;
    let renderedEncounteredGap = false;
    let maximumRenderedUpstreamIncrease = 0;
    for (let index = 0; index < visible.length; index++) {
        if (visible[index]) {
            visibleCellCount++;
            if (encounteredGap) visibleAfterFirstGap++;
        } else {
            encounteredGap = visibleCellCount > 0;
        }
        if (index > 0 && visible[index] !== visible[index - 1]) {
            transitionCount++;
        }
        if (renderedVisible[index]) {
            renderedVisibleCellCount++;
            if (renderedEncounteredGap) renderedVisibleAfterFirstGap++;
        } else {
            renderedEncounteredGap = renderedVisibleCellCount > 0;
        }
        if (
            index > 0 &&
            renderedVisible[index] !== renderedVisible[index - 1]
        ) {
            renderedTransitionCount++;
        }
        if (index > 0) {
            maximumRenderedUpstreamIncrease = Math.max(
                maximumRenderedUpstreamIncrease,
                renderedConcentrations[index] -
                    renderedConcentrations[index - 1]
            );
        }
    }
    washoutColumnSnapshots.push({
        frame,
        timeSeconds: (frame + 1) / 120,
        cellCount: concentrations.length,
        visibleCellCount,
        visibleAfterFirstGap,
        transitionCount,
        renderedVisibleAfterFirstGap,
        renderedTransitionCount,
        disconnectedBranchFade:
            washoutRenderer.lastDisconnectedBranchFade,
        maximumRenderedUpstreamIncrease,
        maximumConcentrationIndex,
        maximumEdgeIndex: maximumCell?.edgeIndex ?? -1,
        maximumCellIndex: maximumCell?.cellIndex ?? -1,
        maximumCellVolumeMm3: maximumEdge
            ? maximumEdge.volumes[maximumCell.cellIndex]
            : 0,
        maximumCellFlowMm3PerS:
            maximumEdge?.meanFlowMm3PerS ?? 0,
        minimumVisibleConcentration: concentrations.reduce(
            (minimum, value) =>
                value >= 0.02 ? Math.min(minimum, value) : minimum,
            Infinity
        ),
        maximumVisibleConcentration: concentrations.reduce(
            (maximum, value) =>
                value >= 0.02 ? Math.max(maximum, value) : maximum,
            0
        )
    });
};
const updateIliacPathCoverage = () => {
    let visibleEdgeCount = 0;
    let pathMassMg = 0;
    for (const edgeIndex of iliacMixingTarget.pathEdgeIndices) {
        const edge = system.flowNetwork.edges[edgeIndex];
        const visible = edge.massMg.some(
            mass => mass > 1e-5
        );
        pathMassMg += edge.massMg.reduce((sum, mass) => sum + mass, 0);
        if (visible) {
            visibleEdgeCount++;
            reachedIliacPathEdges.add(edgeIndex);
        }
    }
    maximumVisibleIliacPathFraction = Math.max(
        maximumVisibleIliacPathFraction,
        visibleEdgeCount / iliacMixingTarget.pathEdgeIndices.length
    );
    maximumIliacPathMassMg = Math.max(maximumIliacPathMassMg, pathMassMg);
    iliacPathMassAfterWashoutMg = pathMassMg;
    const rightInternalIliac = system.flowNetwork.edges[
        aortoiliacAnchors.rightInternalIliacRootEdgeIndex
    ];
    maximumRightInternalIliacMassMg = Math.max(
        maximumRightInternalIliacMassMg,
        rightInternalIliac.massMg.reduce((sum, mass) => sum + mass, 0)
    );
};
const updateAorticBranchCoverage = frame => {
    const aorticEdgeIndices =
        system.pressureDrivenRetrogradeColumn.activePath?.aorticEdgeIndices;
    if (!aorticEdgeIndices?.length) return;
    const aorticEdgeSet = new Set(aorticEdgeIndices);
    for (const aorticEdgeIndex of aorticEdgeIndices) {
        const aorticEdge = system.flowNetwork.edges[aorticEdgeIndex];
        for (const childIndex of aorticEdge.childEdgeIndices) {
            if (aorticEdgeSet.has(childIndex)) continue;
            const child = system.flowNetwork.edges[childIndex];
            const radiusMm = Math.max(
                child.radiusStart,
                child.radiusEnd
            );
            if (
                radiusMm < 1.5 ||
                child.meanFlowMm3PerS < 75
            ) continue;
            if (!aorticBranchCoverage.has(childIndex)) {
                aorticBranchCoverage.set(childIndex, {
                    edgeIndex: childIndex,
                    parentEdgeIndex: aorticEdgeIndex,
                    radiusMm,
                    flowMlPerSec:
                        child.meanFlowMm3PerS / 1000,
                    maximumEntryStockFraction: 0,
                    maximumFirstThreeCellStockFraction: 0,
                    maximumParentTerminalStockFraction: 0,
                    maximumEntryFrame: -1
                });
            }
        }
    }

    for (const coverage of aorticBranchCoverage.values()) {
        const edge = system.flowNetwork.edges[coverage.edgeIndex];
        const parent =
            system.flowNetwork.edges[coverage.parentEdgeIndex];
        const entryStockFraction =
            edge.massMg[0] /
            Math.max(1e-9, edge.volumes[0]) /
            0.3;
        let firstCellsMassMg = 0;
        let firstCellsVolumeMm3 = 0;
        for (
            let cellIndex = 0;
            cellIndex < Math.min(3, edge.cellCount);
            cellIndex++
        ) {
            firstCellsMassMg += edge.massMg[cellIndex];
            firstCellsVolumeMm3 += edge.volumes[cellIndex];
        }
        if (
            entryStockFraction >
            coverage.maximumEntryStockFraction
        ) {
            coverage.maximumEntryStockFraction =
                entryStockFraction;
            coverage.maximumEntryFrame = frame;
        }
        coverage.maximumFirstThreeCellStockFraction = Math.max(
            coverage.maximumFirstThreeCellStockFraction,
            firstCellsMassMg /
                Math.max(1e-9, firstCellsVolumeMm3) /
                0.3
        );
        coverage.maximumParentTerminalStockFraction = Math.max(
            coverage.maximumParentTerminalStockFraction,
            parent.massMg[parent.cellCount - 1] /
                Math.max(
                    1e-9,
                    parent.volumes[parent.cellCount - 1]
                ) /
                0.3
        );
    }
};
for (let frame = 0; frame < 120; frame++) {
    system.update(1 / 120);
    updateDistalSheathCoverage(frame);
    updateAorticBranchCoverage(frame);
    updateIliacPathCoverage();
    const oppositeIliac = system.flowNetwork.edges[oppositeIliacEdgeIndex];
    const oppositeIliacMassMg = oppositeIliac.massMg.reduce(
        (sum, mass) => sum + mass,
        0
    );
    if (frame === 0) {
        oppositeIliacMassAfterFirstFrameMg = oppositeIliacMassMg;
    }
    maximumOppositeIliacMassMg = Math.max(
        maximumOppositeIliacMassMg,
        oppositeIliacMassMg
    );
}
const elapsedMs = performance.now() - startedAt;
const elapsedCpu = process.cpuUsage(startedCpu);
const elapsedCpuMs = (elapsedCpu.user + elapsedCpu.system) / 1000;
for (let frame = 120; frame < 360; frame++) {
    system.update(1 / 120);
    updateDistalSheathCoverage(frame);
    updateAorticBranchCoverage(frame);
    updateIliacPathCoverage();
    captureWashoutColumnSnapshot(frame);
    const oppositeIliac = system.flowNetwork.edges[oppositeIliacEdgeIndex];
    maximumOppositeIliacMassMg = Math.max(
        maximumOppositeIliacMassMg,
        oppositeIliac.massMg.reduce((sum, mass) => sum + mass, 0)
    );
}
console.log('sheath path washout snapshots', washoutColumnSnapshots);
for (const snapshot of washoutColumnSnapshots) {
    assert.equal(
        snapshot.visibleAfterFirstGap,
        0,
        `washout must retain one contiguous sheath-path column at ${snapshot.timeSeconds.toFixed(3)} s`
    );
    assert.ok(
        snapshot.transitionCount <= 1,
        `washout must expose at most one blood/contrast interface at ${snapshot.timeSeconds.toFixed(3)} s`
    );
    assert.equal(
        snapshot.renderedVisibleAfterFirstGap,
        0,
        `rendering must not split the coherent sheath-path column at ${snapshot.timeSeconds.toFixed(3)} s`
    );
    assert.ok(
        snapshot.renderedTransitionCount <= 1,
        `rendering must preserve one sheath-path interface at ${snapshot.timeSeconds.toFixed(3)} s`
    );
    assert.ok(
        snapshot.maximumRenderedUpstreamIncrease <= 1e-7,
        `the rendered sheath column must not darken again beyond its washout front at ${snapshot.timeSeconds.toFixed(3)} s`
    );
    assert.ok(
        snapshot.maximumVisibleConcentration <= 1.25,
        `washout must not create a stationary hyper-concentrated cell (${snapshot.maximumVisibleConcentration}× stock at ${snapshot.timeSeconds.toFixed(3)} s)`
    );
}
const earlyBranchRunoffSnapshot = washoutColumnSnapshots.find(
    snapshot => snapshot.frame === 263
);
const lateBranchRunoffSnapshot = washoutColumnSnapshots.find(
    snapshot => snapshot.frame === 335
);
assert.ok(
    earlyBranchRunoffSnapshot?.disconnectedBranchFade > 0.9,
    'aortic branches must remain fully visible during early post-injection runoff'
);
assert.ok(
    lateBranchRunoffSnapshot?.disconnectedBranchFade > 0.99,
    'the legacy column cleanup mask must stay disabled on the shared local-solver path'
);
const metrics = system.getMetrics();
const fullTreeRenderer = washoutRenderer;
fullTreeRenderer.update();
const pathEdgeSet = new Set(iliacMixingTarget.pathEdgeIndices);
const renderedRadii =
    fullTreeRenderer.flowMesh.geometry.attributes.flowRadius.array;
const renderedBoundaryMinimumRadius = new Map();
for (
    let vertexIndex = 0;
    vertexIndex < fullTreeRenderer.flowTubeVertexCount;
    vertexIndex++
) {
    const edgeIndex =
        fullTreeRenderer._flowVertexConcentrationEdgeIndex[vertexIndex];
    const edgeT =
        fullTreeRenderer._flowVertexConcentrationEdgeT[vertexIndex];
    const boundary = edgeT < 1e-6
        ? 'start'
        : edgeT > 1 - 1e-6
            ? 'end'
            : null;
    if (!boundary) continue;
    const key = `${edgeIndex}:${boundary}`;
    renderedBoundaryMinimumRadius.set(
        key,
        Math.min(
            renderedBoundaryMinimumRadius.get(key) ?? Infinity,
            renderedRadii[vertexIndex]
        )
    );
}
let checkedRenderedForkCount = 0;
let checkedRenderedOstiumCount = 0;
let minimumRenderedForkRadiusRatio = Infinity;
for (const parent of system.flowNetwork.edges) {
    if (parent.transportExcluded || parent.renderExcluded) continue;
    const renderedChildren = parent.childEdgeIndices
        .map(edgeIndex => system.flowNetwork.edges[edgeIndex])
        .filter(child =>
            child &&
            !child.transportExcluded &&
            !child.renderExcluded
        );
    if (renderedChildren.length < 2) continue;
    checkedRenderedForkCount++;
    const incidentBoundaries = [
        {
            edge: parent,
            boundary: 'end',
            anatomicalRadius: parent.radiusEnd
        },
        ...renderedChildren
            .filter(
                child =>
                    child.index !==
                    fullTreeRenderer.flowContinuationChild[parent.index]
            )
            .map(child => ({
                edge: child,
                boundary: 'start',
                anatomicalRadius: child.radiusStart
            }))
    ];
    for (const incident of incidentBoundaries) {
        const key = `${incident.edge.index}:${incident.boundary}`;
        const renderedRadius = renderedBoundaryMinimumRadius.get(key);
        assert.ok(
            Number.isFinite(renderedRadius),
            `rendered fork boundary ${key} should have an explicit tube ring`
        );
        const radiusRatio =
            renderedRadius / Math.max(0.2, incident.anatomicalRadius);
        minimumRenderedForkRadiusRatio = Math.min(
            minimumRenderedForkRadiusRatio,
            radiusRatio
        );
        checkedRenderedOstiumCount++;
        assert.ok(
            radiusRatio >= 0.999,
            `renderer invented a narrowing at fork boundary ${key} (${renderedRadius}/${incident.anatomicalRadius} mm)`
        );
    }
}
assert.ok(
    checkedRenderedForkCount >= 80,
    `the full-tree fork regression should cover the complete arterial topology (${checkedRenderedForkCount})`
);
assert.ok(
    checkedRenderedOstiumCount >= checkedRenderedForkCount * 2,
    `every rendered fork should check its parent and every separately rendered child ostium (${checkedRenderedOstiumCount})`
);
const fullTreeOpticalUnions = new Map();
for (const anchor of fullTreeRenderer.flowTopologyRadiusAnchors) {
    if (!(anchor.opticalFadeLengthMm > 0)) continue;
    const junctionKey = `${anchor.nodeId}:${anchor.parentEdgeIndex}`;
    if (!fullTreeOpticalUnions.has(junctionKey)) {
        fullTreeOpticalUnions.set(junctionKey, []);
    }
    fullTreeOpticalUnions.get(junctionKey).push(anchor);
}
assert.ok(
    fullTreeOpticalUnions.size >= 80,
    `optical-union regression should cover the complete arterial topology (${fullTreeOpticalUnions.size})`
);
for (const [junctionKey, anchors] of fullTreeOpticalUnions) {
    const nodeId = anchors[0].nodeId;
    const parentEdgeIndex = anchors[0].parentEdgeIndex;
    const incidentCount = anchors[0].incidentCount;
    assert.equal(
        anchors.length,
        incidentCount,
        `junction ${nodeId} should expose one optical anchor per incident tube (${JSON.stringify(anchors)})`
    );
    const junctionDiagnostic =
        fullTreeRenderer.flowJunctionUnionDiagnostics.find(
        diagnostic =>
            diagnostic.nodeId === nodeId &&
            diagnostic.parentEdgeIndex === parentEdgeIndex
    );
    const sideOstium = junctionDiagnostic?.geometryKind ===
        'implicit-radius-matched-side-ostium-union';
    assert.ok(
        anchors.every(
            anchor => Math.abs(anchor.opticalWeight - 1) < 1e-9
        ),
        `${sideOstium ? 'side ostium' : 'true fork'} ${junctionKey} must retain full arm strength under maximum blending (${JSON.stringify(anchors)})`
    );
}
assert.equal(
    fullTreeRenderer.flowJunctionUnionDiagnostics.length,
    fullTreeRenderer.flowJunctionCount,
    'every rendered bifurcation should expose its tube-union geometry'
);
let trueForkUnionCount = 0;
let flowContinuationUnionCount = 0;
let anatomicallyClippedSideOstiumVertexCount = 0;
let suppressedSideOstiumConnectorCount = 0;
let retainedConnectorCount = 0;
let retainedConnectorVertexCount = 0;
for (const diagnostic of fullTreeRenderer.flowJunctionUnionDiagnostics) {
    if (!diagnostic.connectorSurfaceSuppressed) {
        retainedConnectorCount++;
        retainedConnectorVertexCount += diagnostic.connectorVertexCount;
        assert.ok(
            diagnostic.connectorVertexCount > 12,
            `rendered junction ${diagnostic.nodeId} must contain a non-degenerate implicit connector`
        );
    }
    if (diagnostic.geometryKind === 'implicit-radius-matched-y-union') {
        assert.equal(
            diagnostic.geometryKind,
            'implicit-radius-matched-y-union',
            `true fork ${diagnostic.nodeId} must use a fitted Y instead of a spherical or conical patch`
        );
        trueForkUnionCount++;
    } else {
        assert.equal(
            diagnostic.geometryKind,
            'implicit-radius-matched-side-ostium-union',
            `side branch ${diagnostic.nodeId} should replace intersecting cylinders with a fitted ostium`
        );
        flowContinuationUnionCount++;
    }
    assert.equal(
        diagnostic.signalMode,
        'implicit-maximum-optical-union',
        `junction ${diagnostic.nodeId} should report the optical union used by its rendered geometry`
    );
    if (
        diagnostic.geometryKind ===
        'implicit-radius-matched-side-ostium-union'
    ) {
        assert.equal(
            diagnostic.surfaceMode,
            'contact-field-max-tube-union',
            `side ostium ${diagnostic.nodeId} should replace its faceted saddle with the anatomical MAX-blended tube union`
        );
        assert.ok(
            diagnostic.maximumOutwardErrorMm <=
                diagnostic.minimumIncidentRadius * 0.08,
            `side ostium ${diagnostic.nodeId} escapes its exact lumen envelope by ${diagnostic.maximumOutwardErrorMm} mm`
        );
        assert.equal(
            diagnostic.anatomicalClipMode,
            'side-ostium-contact-field',
            `side ostium ${diagnostic.nodeId} should use the anatomical lumen mask`
        );
        anatomicallyClippedSideOstiumVertexCount +=
            diagnostic.anatomicalClippedVertexCount;
        if (diagnostic.connectorSurfaceSuppressed) {
            suppressedSideOstiumConnectorCount++;
        }
        assert.ok(
            diagnostic.minimumAnatomicalCoverage >= 0 &&
                diagnostic.minimumAnatomicalCoverage <= 1,
            `side ostium ${diagnostic.nodeId} produced invalid anatomical coverage`
        );
    } else {
        assert.equal(
            diagnostic.surfaceMode,
            'exact-union-smoothed-normals',
            `true fork ${diagnostic.nodeId} should retain its exact fitted Y surface`
        );
        assert.equal(
            diagnostic.anatomicalClipMode,
            'disabled',
            `true fork ${diagnostic.nodeId} must bypass side-ostium clipping`
        );
        assert.equal(
            diagnostic.anatomicalClippedVertexCount,
            0,
            `true fork ${diagnostic.nodeId} must not lose connector vertices`
        );
        assert.equal(
            diagnostic.minimumAnatomicalCoverage,
            1,
            `true fork ${diagnostic.nodeId} must retain full optical coverage`
        );
        assert.equal(
            diagnostic.connectorSurfaceSuppressed,
            false,
            `true fork ${diagnostic.nodeId} must retain its fitted Y surface`
        );
    }
    assert.ok(
        diagnostic.unionLengthMm >= Math.max(
            diagnostic.sideBranchDominance > 0 ? 2.5 : 0.75,
            diagnostic.maximumIncidentRadius * 1.349
        ),
        `junction ${diagnostic.nodeId} partitioned-union zone does not cover its tube overlap`
    );
    assert.equal(
        diagnostic.armRadii.length,
        diagnostic.incidentCount,
        `junction ${diagnostic.nodeId} must match every incident lumen`
    );
    assert.ok(
        Math.abs(
            Math.max(...diagnostic.armRadii) -
            diagnostic.maximumIncidentRadius
        ) <= 1e-9,
        `junction ${diagnostic.nodeId} tube union does not retain the encoded vessel calibre`
    );
}
assert.ok(
    anatomicallyClippedSideOstiumVertexCount > 0,
    'the packed anatomy should exercise side-ostium clipping outside the true lumen'
);
assert.ok(
    suppressedSideOstiumConnectorCount > 0,
    'an anatomically mismatched side ostium should fall back to its MAX-blended tube union'
);
assert.equal(
    suppressedSideOstiumConnectorCount,
    flowContinuationUnionCount,
    'an exact contact field should replace every synthetic side-ostium saddle with its MAX-blended tube union'
);
assert.ok(
    trueForkUnionCount >= 1 && flowContinuationUnionCount >= 1,
    `the full tree should distinguish real forks from flow-dominant side branches (${trueForkUnionCount}/${flowContinuationUnionCount})`
);
assert.equal(
    fullTreeRenderer.flowTubeVertexCount +
        fullTreeRenderer.flowJunctionConnectorVertexCount,
    fullTreeRenderer.flowMesh.geometry.attributes.position.count,
    'the complete tree should contain only anatomical tubes and fitted Y connectors'
);
assert.ok(
    retainedConnectorVertexCount > retainedConnectorCount * 12,
    'every retained connector surface should contribute non-degenerate fitted geometry'
);
assert.ok(
    fullTreeRenderer.flowJunctionConnectorVertexCount >=
        retainedConnectorVertexCount,
    'the connector vertex buffer should contain every retained fitted connector'
);
assert.ok(
    fullTreeRenderer.flowJunctionIndexCount > 0,
    'the full tree should render its local tube-overlap zones as partitioned unions'
);
const aortoiliacParent =
    system.flowNetwork.edges[iliacMixingTarget.edgeIndex];
const aortoiliacContinuationChildEdgeIndex =
    fullTreeRenderer.flowContinuationChild[aortoiliacParent.index];
const aortoiliacParentBoundaryRadius =
    renderedBoundaryMinimumRadius.get(`${aortoiliacParent.index}:end`);
const aortoiliacBoundaryRadii = [
    aortoiliacParentBoundaryRadius,
    ...iliacMixingTarget.childEdgeIndices.map(
        edgeIndex =>
            edgeIndex === aortoiliacContinuationChildEdgeIndex
                ? aortoiliacParentBoundaryRadius
                : renderedBoundaryMinimumRadius.get(`${edgeIndex}:start`)
    )
];
assert.ok(
    aortoiliacBoundaryRadii.every(Number.isFinite),
    'the aortoiliac parent and separately rendered iliac ostium must expose boundary rings'
);
assert.ok(
    Math.min(...aortoiliacBoundaryRadii) >=
        Math.min(
            aortoiliacParent.radiusEnd,
            ...iliacMixingTarget.childEdgeIndices.map(
                edgeIndex => system.flowNetwork.edges[edgeIndex].radiusStart
            )
        ) * 0.999,
    `aortoiliac bifurcation must retain its full encoded calibre (${aortoiliacBoundaryRadii.join(', ')} mm)`
);
const aortoiliacJunctionSlot =
    fullTreeRenderer.flowJunctionUnionDiagnostics.findIndex(
        diagnostic =>
            diagnostic.parentEdgeIndex === aortoiliacParent.index
    );
assert.ok(
    aortoiliacJunctionSlot >= 0,
    'the full-tree renderer must expose a dynamic aortoiliac optical union'
);
const aortoiliacJunctionDiagnostic =
    fullTreeRenderer.flowJunctionUnionDiagnostics[aortoiliacJunctionSlot];
if (aortoiliacJunctionDiagnostic.geometryKind === 'implicit-radius-matched-y-union') {
    assert.equal(
        aortoiliacJunctionDiagnostic.connectorSurfaceSuppressed,
        false,
        'an aortoiliac true Y must retain its fitted connector surface'
    );
} else {
    assert.equal(
        aortoiliacJunctionDiagnostic.geometryKind,
        'implicit-radius-matched-side-ostium-union'
    );
    assert.equal(
        aortoiliacJunctionDiagnostic.connectorSurfaceSuppressed,
        true,
        'an oblique aortoiliac side ostium should use the exact contact-field fallback'
    );
    assert.ok(
        aortoiliacJunctionDiagnostic.anatomicalClippedVertexCount > 0,
        'the aortoiliac side-ostium fallback should be triggered by anatomical clipping'
    );
}
const aortoiliacSurfaceOverlapWeights = [];
for (
    let vertexIndex = 0;
    vertexIndex < fullTreeRenderer.flowTubeVertexCount;
    vertexIndex++
) {
    if (
        fullTreeRenderer._flowVertexJunctionOpticalSlot[vertexIndex] ===
        aortoiliacJunctionSlot
    ) {
        aortoiliacSurfaceOverlapWeights.push(
            fullTreeRenderer._flowVertexJunctionOpticalBlend[vertexIndex]
        );
    }
}
assert.ok(
    aortoiliacSurfaceOverlapWeights.length >= 72 &&
        Math.min(...aortoiliacSurfaceOverlapWeights) < 1e-6 &&
        Math.max(...aortoiliacSurfaceOverlapWeights) > 0.5,
    `the anatomical aortoiliac Y must retain a full outer contour while partitioning only its internal overlap (${Math.min(...aortoiliacSurfaceOverlapWeights)}/${Math.max(...aortoiliacSurfaceOverlapWeights)})`
);
assert.equal(
    fullTreeRenderer._flowDynamicOpticalVertexIndices.length,
    0,
    'the full tree should remove overlap with maximum blending rather than concentration-dependent node weights'
);
fullTreeRenderer.update();
let minimumRenderedIliacRadiusMm = Infinity;
for (
    let vertexIndex = 0;
        vertexIndex < fullTreeRenderer.flowTubeVertexCount;
    vertexIndex++
) {
    if (!pathEdgeSet.has(
        fullTreeRenderer._flowVertexConcentrationEdgeIndex[vertexIndex]
    )) continue;
    minimumRenderedIliacRadiusMm = Math.min(
        minimumRenderedIliacRadiusMm,
        renderedRadii[vertexIndex]
    );
}
console.log('aortoiliac retrograde diagnostics', {
    targetDistanceMm: iliacMixingTarget.distanceMm,
    iliacPathVolumeMl,
    iliacPathEdgeCount: iliacMixingTarget.pathEdgeIndices.length,
    maximumVisibleIliacPathFraction,
    maximumIliacPathMassMg,
    iliacPathMassAfterWashoutMg,
    minimumRenderedIliacRadiusMm,
    checkedRenderedForkCount,
    checkedRenderedOstiumCount,
    minimumRenderedForkRadiusRatio,
    aortoiliacBoundaryRadii,
    reachedIliacPathEdgeFraction:
        reachedIliacPathEdges.size / iliacMixingTarget.pathEdgeIndices.length,
    oppositeIliacEdgeIndex,
    sourceIliacEdgeIndex:
        iliacMixingTarget.sourceChildEdgeIndex,
    maximumOppositeIliacMassMg,
    maximumRightInternalIliacMassMg,
    local: system.localSolver.getDiagnostics()
});
console.log(
    'aortic branch opacification',
    [...aorticBranchCoverage.values()].filter(
        coverage =>
            coverage.maximumParentTerminalStockFraction >= 0.02
    )
);
assert.ok(Math.abs(metrics.totalDeliveredVolumeMl - 30) < 1e-6);
assert.ok(Math.abs(metrics.relativeBalanceError) < 0.005,
    `full-tree iodine balance error was ${metrics.relativeBalanceError}`);
assert.ok(metrics.maxWallPenetrationMm <= 0.2,
    `local plume wall penetration was ${metrics.maxWallPenetrationMm} mm`);
assert.ok(
    oppositeIliacMassAfterFirstFrameMg < 1e-6,
    `a local sheath plume must not be teleported across the aortic bifurcation in one frame (${oppositeIliacMassAfterFirstFrameMg} mg)`
);
assert.ok(
    maximumOppositeIliacMassMg < metrics.totalInjectedIodineMassMg * 5e-3,
    `opposite-iliac spill should stay below 0.5% of injected iodine (${maximumOppositeIliacMassMg}/${metrics.totalInjectedIodineMassMg} mg)`
);
assert.equal(
    aorticBranchCoverage.size,
    0,
    'a local sheath plume should not create synthetic aortic branch inlets'
);
const reachedRetrogradeAorticBranches =
    [...aorticBranchCoverage.values()].filter(
        coverage =>
            coverage.maximumParentTerminalStockFraction >= 0.02
    );
for (const coverage of reachedRetrogradeAorticBranches) {
    assert.ok(
        coverage.maximumEntryStockFraction >= 0.02,
        `a reached aortic branch ${coverage.edgeIndex} failed to opacify at its inlet`
    );
    assert.ok(
        coverage.maximumEntryStockFraction <= 1.25,
        `aortic junction ${coverage.parentEdgeIndex}/${coverage.edgeIndex} concentrated iodine to ${coverage.maximumEntryStockFraction}× stock`
    );
}
assert.ok(
    metrics.retrogradeColumn.activationCount === 0 &&
        metrics.retrogradeColumn.totalInjectedIodineMassMg === 0,
    'strong sheath injection must remain on the shared local-solver path'
);
assert.ok(
    maximumRenderedDistalCoverage >= 0.9,
    `the shared solver should visibly fill the distal vessel during sheath injection (${maximumRenderedDistalCoverage})`
);
assert.ok(
    maximumDistalRendererSuppression <= 0.2,
    `the renderer must not hide a physically filled distal sheath path (${maximumDistalRendererSuppression})`
);
assert.ok(
    metrics.continuousFlowSplit.maximumReversedEdgeCount > 0,
    'a dominant sheath injection must generate signed retrograde face flow without switching solvers'
);
assert.ok(
    maximumRightInternalIliacMassMg > 1e-3,
    `continuous reflux should opacify the patient-right internal iliac during injection (${maximumRightInternalIliacMassMg} mg)`
);
assert.ok(
    iliacPathMassAfterWashoutMg <= maximumIliacPathMassMg + 1e-9,
    `physiological flow must not increase upstream path mass after injection (${iliacPathMassAfterWashoutMg}/${maximumIliacPathMassMg} mg)`
);
assert.ok(
    minimumRenderedIliacRadiusMm > 1.5,
    `rendered iliac lumen should reject needle-like bend artefacts (${minimumRenderedIliacRadiusMm} mm)`
);
assert.equal(
    fullTreeRenderer.flowTubeMaterial.side,
    THREE.FrontSide,
    'the filled-lumen projection should avoid double-sided overlap spikes'
);
assert.equal(
    fullTreeRenderer.flowJunctionMaterial.blending,
    THREE.CustomBlending,
    'true-fork tube shoulders must participate in the non-additive optical union'
);
assert.equal(
    fullTreeRenderer.flowJunctionMaterial.blendEquation,
    THREE.MaxEquation,
    'true-fork tube shoulders must not add into a dark node'
);
assert.equal(
    fullTreeRenderer.flowJunctionMaterial.uniforms.signalGain.value,
    fullTreeRenderer.flowTubeMaterial.uniforms.signalGain.value,
    'junction and ordinary tube segments should have identical signal gain'
);
assert.equal(
    fullTreeRenderer.flowJunctionConnectorMaterial.blending,
    THREE.CustomBlending,
    'the fitted Y surface should use a local optical-union blend'
);
assert.equal(
    fullTreeRenderer.flowJunctionConnectorMaterial.blendEquation,
    THREE.MaxEquation,
    'faces belonging to one Y volume must not add into a dark node'
);
assert.equal(
    fullTreeRenderer.flowSideOstiumMaterial.blendEquation,
    THREE.MaxEquation,
    'side-ostium tube shoulders must not add into a black external contour'
);
assert.equal(
    fullTreeRenderer.flowSideOstiumConnectorMaterial.blendEquation,
    THREE.MaxEquation,
    'the clipped side-ostium surface must render as one maximum optical union'
);
assert.ok(
    fullTreeRenderer.flowMesh.geometry.attributes.position.count /
        fullTreeRenderer.flowRingCount >= 24,
    'the contrasted lumen needs enough radial resolution for a smooth silhouette'
);

// Stateful browser regression: after a sheath bolus the renderer used to
// retain its one-path cleanup mask forever. A later catheter injection was
// transported into both iliacs but only the previously cannulated side was
// visible.
const postSheathAorticEdge =
    (() => {
        const archConnector = topology.intraluminalAorticConnectors
            .connectors[0];
        let edge = system.flowNetwork.edges[
            archConnector.parentEdgeIndex
        ];
        let upstreamDistanceMm = 0;
        while (
            edge.parentEdgeIndex >= 0 &&
            upstreamDistanceMm < 20
        ) {
            upstreamDistanceMm += edge.length;
            edge = system.flowNetwork.edges[edge.parentEdgeIndex];
        }
        return edge;
    })();
const postSheathCatheterPort = {
    kind: 'catheter-end',
    position: postSheathAorticEdge.start.clone().lerp(
        postSheathAorticEdge.end,
        0.5
    ),
    direction: postSheathAorticEdge.axis.clone(),
    radiusMm: 0.48,
    areaMm2: Math.PI * 0.48 ** 2,
    weight: 1,
    valid: true
};
system.catheter = {
    type: 'berenstein',
    getInjectionPorts(out) {
        out.length = 0;
        out.push(postSheathCatheterPort);
        return out;
    }
};
assert.equal(system.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 12,
    rateMlPerSec: 12
}).ok, true);
const postSheathIliacDisplayPeaks = new Map(
    iliacMixingTarget.childEdgeIndices.map(edgeIndex => [edgeIndex, 0])
);
const postSheathArchDisplayPeaks = new Map(
    [
        topology.intraluminalAorticConnectors.connectors[0]
            .branchRootEdgeIndices,
        topology.aorticBranchPrefixes.paths
            .map(path => path.rootEdgeIndex)
    ].flat().map(edgeIndex => [edgeIndex, 0])
);
// The physiological aortic root makes the arch-to-iliac residence time about
// 4.1 s; observe beyond that transit instead of relying on the old reversed
// root's artificially short path.
for (let frame = 0; frame < 720; frame++) {
    system.update(1 / 120);
    if (frame % 4 !== 0) continue;
    fullTreeRenderer.update();
    for (const [edgeIndex, peak] of postSheathIliacDisplayPeaks) {
        postSheathIliacDisplayPeaks.set(
            edgeIndex,
            Math.max(
                peak,
                fullTreeRenderer._flowCellDisplayConcentration[
                    fullTreeRenderer._flowCellOffset[edgeIndex]
                ]
            )
        );
    }
    for (const [edgeIndex, peak] of postSheathArchDisplayPeaks) {
        postSheathArchDisplayPeaks.set(
            edgeIndex,
            Math.max(
                peak,
                fullTreeRenderer._flowCellDisplayConcentration[
                    fullTreeRenderer._flowCellOffset[edgeIndex]
                ]
            )
        );
    }
}
assert.ok(
    Math.min(...postSheathIliacDisplayPeaks.values()) >= 0.1,
    `a catheter bolus after sheath washout must render both iliacs (${JSON.stringify(Object.fromEntries(postSheathIliacDisplayPeaks))})`
);
assert.ok(
    Math.min(...postSheathArchDisplayPeaks.values()) >= 0.1,
    `a catheter bolus after sheath washout must render every supra-aortic root (${JSON.stringify(Object.fromEntries(postSheathArchDisplayPeaks))})`
);
assert.equal(
    fullTreeRenderer._pressureDrivenColumnEndTime,
    null,
    'a later non-column injection must clear the completed pressure-driven path mask'
);
fullTreeRenderer.dispose();
// CPU time measures solver cost without failing when the concurrently running
// browser/server deschedules this test process. Wall time is still reported so
// a loaded development machine remains visible in diagnostics.
const maximumAverageUpdateCpuMs = Math.max(
    10,
    topology.directedEdgeCount / 300
);
assert.ok(elapsedCpuMs / 120 < maximumAverageUpdateCpuMs,
    `full-tree hybrid update averaged ${(elapsedCpuMs / 120).toFixed(3)} ms CPU time (limit ${maximumAverageUpdateCpuMs.toFixed(3)} ms for ${topology.directedEdgeCount} edges)`);

const aorticInflowNetwork = new ContrastFlowNetwork(centerlineSegments);
const aorticInletEdges =
    aorticInflowNetwork.rootNode.childEdgeIndices.map(
        edgeIndex => aorticInflowNetwork.edges[edgeIndex]
    );
assert.ok(
    aorticInletEdges.length > 0,
    'the directed full tree should expose an aortic inlet edge'
);
const aorticJunctionCoverage = [];
for (const parent of aorticInflowNetwork.edges) {
    if (
        parent.childEdgeIndices.length < 2 ||
        Math.max(parent.radiusStart, parent.radiusEnd) < 5
    ) continue;
    for (const childIndex of parent.childEdgeIndices) {
        const child = aorticInflowNetwork.edges[childIndex];
        if (
            Math.max(child.radiusStart, child.radiusEnd) < 1.5 ||
            child.meanFlowMm3PerS < 75
        ) continue;
        aorticJunctionCoverage.push({
            parentEdgeIndex: parent.index,
            childEdgeIndex: child.index,
            childRadiusMm: Math.max(
                child.radiusStart,
                child.radiusEnd
            ),
            childFlowMlPerSec:
                child.meanFlowMm3PerS / 1000,
            maximumParentStockFraction: 0,
            maximumChildEntryStockFraction: 0
        });
    }
}
const inletStockConcentrationMgPerMm3 = 0.3;
for (let frame = 0; frame < 480; frame++) {
    if (frame < 120) {
        for (const inletEdge of aorticInletEdges) {
            aorticInflowNetwork.depositIodine(
                inletEdge.index,
                0,
                inletEdge.meanFlowMm3PerS *
                    inletStockConcentrationMgPerMm3 /
                    120
            );
        }
    }
    aorticInflowNetwork.update(1 / 120);
    for (const coverage of aorticJunctionCoverage) {
        const parent =
            aorticInflowNetwork.edges[coverage.parentEdgeIndex];
        const child =
            aorticInflowNetwork.edges[coverage.childEdgeIndex];
        const parentCellIndex = parent.cellCount - 1;
        coverage.maximumParentStockFraction = Math.max(
            coverage.maximumParentStockFraction,
            parent.massMg[parentCellIndex] /
                Math.max(1e-9, parent.volumes[parentCellIndex]) /
                inletStockConcentrationMgPerMm3
        );
        coverage.maximumChildEntryStockFraction = Math.max(
            coverage.maximumChildEntryStockFraction,
            child.massMg[0] /
                Math.max(1e-9, child.volumes[0]) /
                inletStockConcentrationMgPerMm3
        );
    }
}
const reachedAorticJunctionChildren =
    aorticJunctionCoverage.filter(
        coverage => coverage.maximumParentStockFraction >= 0.2
    );
console.log(
    'antegrade aortic junction coverage',
    reachedAorticJunctionChildren
);
assert.ok(
    reachedAorticJunctionChildren.length >= 8,
    `the antegrade bolus should reach multiple aortic bifurcations (${reachedAorticJunctionChildren.length})`
);
for (const coverage of reachedAorticJunctionChildren) {
    assert.ok(
        coverage.maximumChildEntryStockFraction /
            coverage.maximumParentStockFraction >= 0.6,
        `aortic child ${coverage.childEdgeIndex} opacified poorly relative to parent ${coverage.parentEdgeIndex}`
    );
}

const washoutNetwork = new ContrastFlowNetwork(centerlineSegments);
const perfusedWashoutEdges = washoutNetwork.edges.filter(
    edge => !edge.transportExcluded && edge.meanFlowMm3PerS > 0
);
const slowestEdge = perfusedWashoutEdges.reduce((slowest, edge) =>
    edge.totalVolume / edge.meanFlowMm3PerS >
    slowest.totalVolume / slowest.meanFlowMm3PerS
        ? edge
        : slowest
);
const slowestResidenceSeconds =
    slowestEdge.totalVolume / slowestEdge.meanFlowMm3PerS;
assert.ok(slowestResidenceSeconds < 1,
    `slowest visible segment residence time was ${slowestResidenceSeconds.toFixed(3)} s`);
const upstreamEdge = washoutNetwork.edges[slowestEdge.parentEdgeIndex];
washoutNetwork.depositIodine(slowestEdge.index, 0, 300);
for (let frame = 0; frame < 720; frame++) washoutNetwork.update(1 / 120);
const maximumResidualStockFraction = washoutNetwork.edges.reduce(
    (maximum, edge) => {
        for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
            maximum = Math.max(
                maximum,
                edge.massMg[cellIndex] / edge.volumes[cellIndex] / 0.3
            );
        }
        return maximum;
    },
    0
);
assert.ok(washoutNetwork.getIodineMassMg() < 1.5,
    `slowest branch retained ${washoutNetwork.getIodineMassMg().toFixed(3)} mg after 6 s; max stock fraction ${maximumResidualStockFraction}`);
if (upstreamEdge) {
    assert.ok(upstreamEdge.massMg.every(mass => mass < 1e-12),
        'selective contrast must not leak upstream across a junction');
}

console.log(JSON.stringify({
    topology,
    deliveredVolumeMl: metrics.totalDeliveredVolumeMl,
    relativeBalanceError: metrics.relativeBalanceError,
    maxWallPenetrationMm: metrics.maxWallPenetrationMm,
    activeParticleCount: metrics.activeParticleCount,
    averageUpdateMs: elapsedCpuMs / 120,
    averageWallUpdateMs: elapsedMs / 120,
    slowestResidenceSeconds,
    slowestBranchMassAfter6SecondsMg: washoutNetwork.getIodineMassMg(),
    maximumResidualStockFraction,
    maximumVisibleIliacPathFraction,
    maximumIliacPathMassMg,
    iliacPathMassAfterWashoutMg,
    minimumRenderedIliacRadiusMm,
    checkedRenderedForkCount,
    checkedRenderedOstiumCount,
    minimumRenderedForkRadiusRatio,
    aortoiliacBoundaryRadii,
    reachedIliacPathEdgeFraction:
        reachedIliacPathEdges.size / iliacMixingTarget.pathEdgeIndices.length,
    maximumOppositeIliacMassMg
}));
