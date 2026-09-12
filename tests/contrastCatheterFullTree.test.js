import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {
    CONTRAST_SOURCE_CATHETER,
    HybridContrastSystem
} from '../src/contrast/hybridContrastSystem.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { findAorticArchDebugAnchors } from '../src/anatomyDebugLabels.js';

function loadCenterlineSegments() {
    const sourceBuffer = fs.readFileSync(
        new URL('../res/Aorta_plain.collision.bin', import.meta.url)
    );
    const arrayBuffer = sourceBuffer.buffer.slice(
        sourceBuffer.byteOffset,
        sourceBuffer.byteOffset + sourceBuffer.byteLength
    );
    const asset = decodeCollisionAsset(arrayBuffer);
    const data = asset.arrays.centerlineSegments;
    const nodeEdges = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride;
    const segments = [];
    for (let index = 0; index < data.length / stride; index++) {
        const offset = index * stride;
        segments.push({
            id: index,
            start: new THREE.Vector3(
                data[offset],
                data[offset + 1],
                data[offset + 2]
            ),
            end: new THREE.Vector3(
                data[offset + 3],
                data[offset + 4],
                data[offset + 5]
            ),
            radiusStart: data[offset + 6],
            radiusEnd: data[offset + 7],
            safeRadius: data[offset + 8],
            nodeStartId: nodeEdges[index * 2],
            nodeEndId: nodeEdges[index * 2 + 1]
        });
    }
    return segments;
}

function edgeEntryStockFraction(edge, stockConcentrationMgPerMm3) {
    return edge.massMg[0] /
        Math.max(1e-9, edge.volumes[0]) /
        stockConcentrationMgPerMm3;
}

function buildDominantPathCells(network, firstEdge, maximumDistanceMm) {
    const cells = [];
    let edge = firstEdge;
    let coveredDistanceMm = 0;
    const visited = new Set();
    while (
        edge &&
        !visited.has(edge.index) &&
        coveredDistanceMm < maximumDistanceMm
    ) {
        visited.add(edge.index);
        for (
            let cellIndex = 0;
            cellIndex < edge.cellCount &&
                coveredDistanceMm < maximumDistanceMm;
            cellIndex++
        ) {
            cells.push({ edgeIndex: edge.index, cellIndex });
            coveredDistanceMm += edge.cellLength;
        }
        edge = edge.childEdgeIndices.reduce((widest, childIndex) => {
            const child = network.edges[childIndex];
            if (!widest) return child;
            return child.meanFlowMm3PerS > widest.meanFlowMm3PerS
                ? child
                : widest;
        }, null);
    }
    return { cells, coveredDistanceMm };
}

function renderedPathCoverage(renderer, path, threshold = 0.02) {
    if (!path.cells.length) return 0;
    let visibleCellCount = 0;
    for (const { edgeIndex, cellIndex } of path.cells) {
        const flatIndex =
            renderer._flowCellOffset[edgeIndex] + cellIndex;
        if (
            renderer._flowCellDisplayConcentration[flatIndex] >=
            threshold
        ) {
            visibleCellCount++;
        }
    }
    return visibleCellCount / path.cells.length;
}

function physicalPathCoverage(
    network,
    path,
    stockConcentrationMgPerMm3,
    threshold = 0.02
) {
    if (!path.cells.length) return 0;
    let visibleCellCount = 0;
    for (const { edgeIndex, cellIndex } of path.cells) {
        const edge = network.edges[edgeIndex];
        const normalized =
            edge.massMg[cellIndex] /
            Math.max(1e-9, edge.volumes[cellIndex]) /
            stockConcentrationMgPerMm3;
        if (normalized >= threshold) visibleCellCount++;
    }
    return visibleCellCount / path.cells.length;
}

function findAortoiliacParent(network) {
    return network.edges.find(edge =>
        edge.end.y < -275 &&
        edge.end.y > -305 &&
        edge.radiusEnd > 6 &&
        edge.childEdgeIndices.length === 2 &&
        edge.childEdgeIndices.every(
            childIndex => network.edges[childIndex].radiusStart > 6
        )
    );
}

function findAorticBranchOrigin(network, branchRootEdgeIndex) {
    let branchEdge = network.edges[branchRootEdgeIndex];
    while (branchEdge?.parentEdgeIndex >= 0) {
        const parent = network.edges[branchEdge.parentEdgeIndex];
        const aorticContinuation = parent.childEdgeIndices
            .filter(edgeIndex => edgeIndex !== branchEdge.index)
            .map(edgeIndex => network.edges[edgeIndex])
            .find(edge =>
                Math.min(edge.radiusStart, edge.radiusEnd) >= 10
            );
        if (aorticContinuation) {
            return { parent, aorticContinuation };
        }
        branchEdge = parent;
    }
    return null;
}

function pathResidenceSeconds(network, rootEdgeIndex, targetEdgeIndex) {
    const path = [];
    let edge = network.edges[targetEdgeIndex];
    while (edge && edge.index !== rootEdgeIndex) {
        path.push(edge);
        edge = edge.parentEdgeIndex >= 0
            ? network.edges[edge.parentEdgeIndex]
            : null;
    }
    assert.ok(
        edge,
        `edge ${targetEdgeIndex} should descend from ${rootEdgeIndex}`
    );
    path.push(edge);
    return path.reduce(
        (seconds, pathEdge) =>
            seconds +
            pathEdge.totalVolume /
                Math.max(1e-9, pathEdge.meanFlowMm3PerS),
        0
    );
}

const catheterPort = {
    kind: 'berenstein-end',
    position: new THREE.Vector3(),
    direction: new THREE.Vector3(),
    radiusMm: 0.48,
    areaMm2: Math.PI * 0.48 ** 2,
    weight: 1,
    valid: true
};
let activeCatheterPorts = [catheterPort];
const catheter = {
    type: 'berenstein',
    getInjectionPorts(out) {
        out.length = 0;
        out.push(...activeCatheterPorts);
        return out;
    }
};
const system = new HybridContrastSystem({
    centerlineSegments: loadCenterlineSegments(),
    catheter,
    localOptions: {
        capacity: 16000,
        randomSeed: 20260730
    }
});
const network = system.flowNetwork;

const archAnchors = findAorticArchDebugAnchors(network);
assert.ok(archAnchors, 'the packed anatomy should expose the aortic arch branches');
const archPrefixDiagnostics =
    network.getTopologyDiagnostics().aorticBranchPrefixes;
const aorticArtifactDiagnostics =
    network.getTopologyDiagnostics().intraluminalAorticArtifacts;
const aorticConnectorDiagnostics =
    network.getTopologyDiagnostics().intraluminalAorticConnectors;
assert.equal(
    aorticConnectorDiagnostics.contractedConnectorCount,
    1,
    'the false aorta-calibre connector between arch branches should be contracted'
);
const [contractedArchConnector] =
    aorticConnectorDiagnostics.connectors;
const contractedConnectorEdge =
    network.edges[contractedArchConnector.connectorEdgeIndex];
const contractedConnectorParent =
    network.edges[contractedArchConnector.parentEdgeIndex];
assert.equal(
    contractedConnectorEdge.transportExcluded,
    undefined,
    'the connector must remain in the transport tree to preserve branch continuity'
);
assert.equal(
    contractedConnectorEdge.renderExcluded,
    true,
    'the intraluminal supra-aortic connector should not be drawn as a separate vessel'
);
assert.equal(
    contractedConnectorEdge.lumenMappingExcluded,
    true,
    'catheter ports should map to the surrounding anatomical aorta instead of the connector'
);
const archPrefixEdgeIndices = new Set(
    archPrefixDiagnostics.paths.flatMap(path => path.edgeIndices)
);
const overlappingArchPrefixPath = archPrefixDiagnostics.paths.find(
    path =>
        path.rootEdgeIndex ===
        archAnchors.brachiocephalicTrunkRootEdgeIndex
);
assert.ok(
    overlappingArchPrefixPath,
    'the brachiocephalic technical prefix should be present in the packed-arch diagnostics'
);
let remainingOverlapDistanceMm =
    overlappingArchPrefixPath.lengthMm * 0.28;
let overlappingArchPrefixEdge = null;
let overlappingArchPrefixPoint = null;
for (const edgeIndex of overlappingArchPrefixPath.edgeIndices) {
    const edge = network.edges[edgeIndex];
    if (remainingOverlapDistanceMm <= edge.length) {
        overlappingArchPrefixEdge = edge;
        overlappingArchPrefixPoint = edge.start.clone().addScaledVector(
            edge.axis,
            remainingOverlapDistanceMm
        );
        break;
    }
    remainingOverlapDistanceMm -= edge.length;
}
assert.ok(
    overlappingArchPrefixEdge,
    'the packed arch should contain a technical branch prefix overlapping the high-flow aortic lumen'
);
const overlappingArchLocation = network.findInjectionLocation(
    overlappingArchPrefixPoint,
    overlappingArchPrefixEdge.axis
);
assert.ok(
    !archPrefixEdgeIndices.has(overlappingArchLocation.edgeIndex),
    `a catheter in the overlapping arch lumen must map to the aorta, not technical branch prefix ${overlappingArchLocation.edgeIndex}`
);
assert.ok(
    Math.min(
        network.edges[overlappingArchLocation.edgeIndex].radiusStart,
        network.edges[overlappingArchLocation.edgeIndex].radiusEnd
    ) > Math.max(
        overlappingArchPrefixEdge.radiusStart,
        overlappingArchPrefixEdge.radiusEnd
    ),
    'the overlapping-lumen source should resolve to the wider anatomical aorta'
);
const retrogradeArchPort = {
    ...catheterPort,
    position: overlappingArchPrefixPoint.clone(),
    direction: network.edges[overlappingArchLocation.edgeIndex].axis
        .clone()
        .multiplyScalar(-1)
};
const retrogradeArchSystem = new HybridContrastSystem({
    centerlineSegments: loadCenterlineSegments(),
    catheter: {
        type: 'berenstein',
        getInjectionPorts(out) {
            out.length = 0;
            out.push(retrogradeArchPort);
            return out;
        }
    },
    localOptions: {
        capacity: 16000,
        randomSeed: 20260802
    }
});
const retrogradeArchNetwork = retrogradeArchSystem.flowNetwork;
const retrogradeArchRenderer = new ContrastVolumeRenderer(
    retrogradeArchSystem
);
const retrogradeArchAnchors = findAorticArchDebugAnchors(
    retrogradeArchNetwork
);
const retrogradeArchBranchPeaks = new Map([
    retrogradeArchAnchors.brachiocephalicTrunkRootEdgeIndex,
    retrogradeArchAnchors.leftCommonCarotidRootEdgeIndex,
    retrogradeArchAnchors.leftSubclavianRootEdgeIndex
].map(edgeIndex => [edgeIndex, 0]));
let retrogradeArchAortaPeak = 0;
assert.equal(retrogradeArchSystem.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 43.5,
    rateMlPerSec: 39
}).ok, true);
for (let frame = 0; frame < 720; frame++) {
    retrogradeArchSystem.update(1 / 120);
    retrogradeArchRenderer.update();
    const sourceEdgeIndex =
        retrogradeArchSystem.getMetrics().lastCatheterSourceEdgeIndex;
    if (sourceEdgeIndex >= 0) {
        const flatIndex =
            retrogradeArchRenderer._flowCellOffset[sourceEdgeIndex];
        retrogradeArchAortaPeak = Math.max(
            retrogradeArchAortaPeak,
            retrogradeArchRenderer._flowCellDisplayConcentration[flatIndex]
        );
    }
    for (const [edgeIndex, peak] of retrogradeArchBranchPeaks) {
        const flatIndex =
            retrogradeArchRenderer._flowCellOffset[edgeIndex];
        retrogradeArchBranchPeaks.set(
            edgeIndex,
            Math.max(
                peak,
                retrogradeArchRenderer._flowCellDisplayConcentration[flatIndex]
            )
        );
    }
}
const retrogradeArchMetrics = retrogradeArchSystem.getMetrics();
assert.ok(
    !archPrefixEdgeIndices.has(
        retrogradeArchMetrics.lastCatheterSourceEdgeIndex
    ),
    'the high-rate arch injection source must remain on the anatomical aorta'
);
assert.equal(
    retrogradeArchMetrics.retrogradeTargetedIodineMassMg,
    0,
    'the continuous signed-flow model must not teleport a parcel core to a preselected arch junction'
);
assert.ok(
    retrogradeArchMetrics.continuousFlowSplit.maximumUpstreamRateMlPerSec >
        retrogradeArchMetrics.continuousFlowSplit.maximumDownstreamRateMlPerSec,
    'an upstream-facing catheter should continuously bias its volume source proximally even when aortic blood flow prevents whole-lumen reversal'
);
assert.ok(
    retrogradeArchMetrics.catheterSourceHandoffIodineMassMg > 0,
    'blood entrainment must mix part of the bolus locally instead of sending the complete injection as one ballistic core'
);
assert.ok(
    retrogradeArchMetrics.maximumRetrogradeProgressMm <=
        retrogradeArchMetrics.lastJetMixingLengthMm * 1.1,
    `the visible arch jet must remain inside its outlet-scale mixing length (${retrogradeArchMetrics.maximumRetrogradeProgressMm} vs ${retrogradeArchMetrics.lastJetMixingLengthMm} mm)`
);
assert.equal(
    retrogradeArchMetrics.activeParticleCount,
    0,
    'the completed arch injection must leave no detached local contrast fragments'
);
assert.ok(
    retrogradeArchAortaPeak >= 0.06,
    `the arch catheter bolus must opacify the aorta (${retrogradeArchAortaPeak})`
);
assert.equal(
    retrogradeArchMetrics.continuousFlowSplit.maximumReversedEdgeCount,
    0,
    'a pressure-limited Berenstein injection should not reverse the higher-flow aortic strand'
);
assert.ok(
    Math.max(...retrogradeArchBranchPeaks.values()) < 0.04,
    `a subcritical retrograde jet must not teleport contrast into supra-aortic branches (${JSON.stringify(Object.fromEntries(retrogradeArchBranchPeaks))})`
);
retrogradeArchRenderer.dispose();

const selectiveCarotidSystem = new HybridContrastSystem({
    centerlineSegments: loadCenterlineSegments(),
    catheter,
    localOptions: {
        capacity: 2400,
        parcelVolumeMl: 0.01,
        randomSeed: 20260803
    }
});
const selectiveCarotidNetwork = selectiveCarotidSystem.flowNetwork;
const selectiveCarotidAnchors = findAorticArchDebugAnchors(
    selectiveCarotidNetwork
);
const selectiveCarotidEdge = selectiveCarotidNetwork.edges[
    selectiveCarotidAnchors.leftCommonCarotidEdgeIndex
];
const selectiveCarotidPort = {
    kind: 'berenstein-end',
    position: selectiveCarotidEdge.start.clone().lerp(
        selectiveCarotidEdge.end,
        0.55
    ),
    direction: selectiveCarotidEdge.axis.clone(),
    radiusMm: 0.48,
    areaMm2: Math.PI * 0.48 ** 2,
    weight: 1,
    valid: true
};
activeCatheterPorts = [selectiveCarotidPort];
assert.equal(selectiveCarotidSystem.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 20,
    rateMlPerSec: 22.6
}).ok, true);
let selectiveCarotidPeakMassMg = 0;
let selectiveCarotidAortaPeakMassMg = 0;
let selectiveCarotidSiblingPeakMassMg = 0;
const selectiveCarotidSiblingRootEdgeIndices = [
    selectiveCarotidAnchors.brachiocephalicTrunkRootEdgeIndex,
    selectiveCarotidAnchors.leftSubclavianRootEdgeIndex
];
const selectiveCarotidUpstreamEdges = [];
let selectiveCarotidUpstreamEdge = selectiveCarotidEdge;
while (selectiveCarotidUpstreamEdge.parentEdgeIndex >= 0) {
    selectiveCarotidUpstreamEdge = selectiveCarotidNetwork.edges[
        selectiveCarotidUpstreamEdge.parentEdgeIndex
    ];
    selectiveCarotidUpstreamEdges.push(selectiveCarotidUpstreamEdge);
}
for (let frame = 0; frame < 480; frame++) {
    selectiveCarotidSystem.update(1 / 120);
    selectiveCarotidPeakMassMg = Math.max(
        selectiveCarotidPeakMassMg,
        ...selectiveCarotidEdge.massMg
    );
    for (const edge of selectiveCarotidUpstreamEdges) {
        selectiveCarotidAortaPeakMassMg = Math.max(
            selectiveCarotidAortaPeakMassMg,
            ...edge.massMg
        );
    }
    for (const edgeIndex of selectiveCarotidSiblingRootEdgeIndices) {
        selectiveCarotidSiblingPeakMassMg = Math.max(
            selectiveCarotidSiblingPeakMassMg,
            ...selectiveCarotidNetwork.edges[edgeIndex].massMg
        );
    }
}
activeCatheterPorts = [catheterPort];
const selectiveCarotidMetrics = selectiveCarotidSystem.getMetrics();
assert.equal(
    selectiveCarotidMetrics.lastCatheterSourceEdgeIndex,
    selectiveCarotidEdge.index,
    'the selective bolus should remain mapped to the catheterized carotid'
);
assert.equal(
    selectiveCarotidMetrics.retrogradeTargetedIodineMassMg,
    0,
    'a distally directed carotid jet must not acquire a retrograde target'
);
assert.ok(
    selectiveCarotidPeakMassMg > 0,
    'the catheterized carotid should receive the selective bolus'
);
assert.ok(
    selectiveCarotidAortaPeakMassMg < 1e-12 &&
        selectiveCarotidSiblingPeakMassMg < 1e-12,
    `selective carotid injection must not fill the aorta or sibling arch branches (${selectiveCarotidAortaPeakMassMg}, ${selectiveCarotidSiblingPeakMassMg})`
);

const connectorBranchFlowMm3PerS =
    contractedArchConnector.branchRootEdgeIndices.reduce(
        (sum, edgeIndex) =>
            sum + network.edges[edgeIndex].meanFlowMm3PerS,
        0
    );
assert.ok(
    Math.abs(
        contractedConnectorEdge.meanFlowMm3PerS -
        connectorBranchFlowMm3PerS
    ) < 1e-8,
    'the connector should deliver the complete combined flow to both branch roots'
);
assert.ok(
    contractedConnectorParent.childEdgeIndices.includes(
        contractedConnectorEdge.index
    ),
    'the connector should remain attached to its geometrical aortic origin'
);
for (const branchRootEdgeIndex of
    contractedArchConnector.branchRootEdgeIndices) {
    assert.ok(
        contractedConnectorEdge.childEdgeIndices.includes(
            branchRootEdgeIndex
        ),
        `real branch ${branchRootEdgeIndex} should remain connected through the short bridge`
    );
    assert.equal(
        network.edges[branchRootEdgeIndex].parentEdgeIndex,
        contractedConnectorEdge.index,
        `branch ${branchRootEdgeIndex} should keep its geometrical parent`
    );
}
assert.equal(
    aorticArtifactDiagnostics.suppressedEdgeCount,
    aorticArtifactDiagnostics.roots.reduce(
        (sum, root) => sum + root.edgeCount,
        0
    ),
    'every detected parallel dead-end strand should be excluded from transport'
);
assert.ok(
    aorticArtifactDiagnostics.suppressedRootCount <= 1,
    'the packed arch should contain at most one confidently detected intraluminal duplicate centreline'
);
for (const suppressedAorticArtifact of
    aorticArtifactDiagnostics.roots) {
    const artifactParent =
        network.edges[suppressedAorticArtifact.parentEdgeIndex];
    const aorticContinuation =
        network.edges[
            suppressedAorticArtifact.mainContinuationEdgeIndex
        ];
    const artifactRoot =
        network.edges[suppressedAorticArtifact.rootEdgeIndex];
    assert.equal(
        artifactRoot.transportExcluded,
        true,
        'a false strand should remain addressable but be excluded from contrast transport'
    );
    assert.ok(
        !artifactParent.childEdgeIndices.includes(artifactRoot.index),
        'the aortic flow divider must not include an intraluminal duplicate'
    );
    assert.ok(
        Math.abs(
            artifactParent.meanFlowMm3PerS -
            aorticContinuation.meanFlowMm3PerS
        ) < 1e-8,
        'all post-subclavian aortic flow should continue down the anatomical aorta'
    );
    assert.equal(
        artifactRoot.meanFlowMm3PerS,
        0,
        'a false blind strand must not steal cardiac output'
    );
    const remappedArtifactOutlet = network.findNearestLocation(
        network.edges[
            suppressedAorticArtifact.terminalEdgeIndex
        ].end
    );
    assert.notEqual(
        network.edges[remappedArtifactOutlet.edgeIndex]
            ?.transportExcluded,
        true,
        'lumen lookup near a duplicate strand should map to a perfused aortic edge'
    );
}
const archGeometryRegressionRenderer = new ContrastVolumeRenderer(system);
const expectedPerfusedCellCount = network.edges.reduce(
    (sum, edge) => sum + (
        edge.transportExcluded || edge.renderExcluded
            ? 0
            : edge.cellCount
    ),
    0
);
assert.equal(
    archGeometryRegressionRenderer.flowAxialSegmentCount,
    expectedPerfusedCellCount,
    'the false post-subclavian strand must not be rendered as an opacifiable lumen'
);
archGeometryRegressionRenderer.dispose();

const archBolusNetwork = new ContrastFlowNetwork(
    loadCenterlineSegments()
);
const archBolusArtifact =
    archBolusNetwork.intraluminalAorticArtifactDiagnostics.roots[0] ||
    null;
const archBolusConnector =
    archBolusNetwork.intraluminalAorticConnectorDiagnostics.connectors[0];
const archBolusConnectorEdge =
    archBolusNetwork.edges[archBolusConnector.connectorEdgeIndex];
const archBranchOrigin = archBolusArtifact
    ? {
        parent: archBolusNetwork.edges[
            archBolusArtifact.parentEdgeIndex
        ],
        aorticContinuation: archBolusNetwork.edges[
            archBolusArtifact.mainContinuationEdgeIndex
        ]
    }
    : findAorticBranchOrigin(
        archBolusNetwork,
        archAnchors.leftSubclavianRootEdgeIndex
    );
assert.ok(
    archBranchOrigin,
    'the packed tree should expose the common supra-aortic origin'
);
const archBolusParent = archBranchOrigin.parent;
const archBolusContinuation = archBranchOrigin.aorticContinuation;
let archBolusSource = archBolusParent;
let archBolusUpstreamDistanceMm = 0;
while (
    archBolusSource.parentEdgeIndex >= 0 &&
    archBolusUpstreamDistanceMm < 20
) {
    archBolusUpstreamDistanceMm += archBolusSource.length;
    archBolusSource =
        archBolusNetwork.edges[archBolusSource.parentEdgeIndex];
}
const archBolusVolumeMl = 43.5;
const archBolusRateMlPerSec = 39;
const archBolusStockConcentrationMgPerMm3 = 0.3;
let archBolusDeliveredMl = 0;
let archBolusContinuationPeak = 0;
let archBolusArtifactPeakMassMg = 0;
let archBolusConnectorPeakMassMg = 0;
const archBolusBranchPeaks = new Map([
    archAnchors.brachiocephalicTrunkRootEdgeIndex,
    archAnchors.leftCommonCarotidRootEdgeIndex,
    archAnchors.leftSubclavianRootEdgeIndex
].map(edgeIndex => [edgeIndex, 0]));
for (let frame = 0; frame < 360; frame++) {
    if (archBolusDeliveredMl < archBolusVolumeMl) {
        const deliveredMl = Math.min(
            archBolusRateMlPerSec / 120,
            archBolusVolumeMl - archBolusDeliveredMl
        );
        archBolusNetwork.depositIodine(
            archBolusSource.index,
            0,
            deliveredMl * 300
        );
        archBolusDeliveredMl += deliveredMl;
    }
    archBolusNetwork.update(1 / 120);
    archBolusContinuationPeak = Math.max(
        archBolusContinuationPeak,
        archBolusContinuation.massMg[0] /
            Math.max(1e-9, archBolusContinuation.volumes[0]) /
            archBolusStockConcentrationMgPerMm3
    );
    let artifactMassMg = 0;
    const artifactQueue = archBolusArtifact
        ? [archBolusArtifact.rootEdgeIndex]
        : [];
    for (let queueIndex = 0; queueIndex < artifactQueue.length; queueIndex++) {
        const edge = archBolusNetwork.edges[artifactQueue[queueIndex]];
        artifactMassMg += edge.massMg.reduce(
            (sum, mass) => sum + mass,
            0
        );
        artifactQueue.push(...edge.childEdgeIndices);
    }
    archBolusArtifactPeakMassMg = Math.max(
        archBolusArtifactPeakMassMg,
        artifactMassMg
    );
    archBolusConnectorPeakMassMg = Math.max(
        archBolusConnectorPeakMassMg,
        archBolusConnectorEdge.massMg.reduce(
            (sum, mass) => sum + mass,
            0
        )
    );
    for (const [edgeIndex, peak] of archBolusBranchPeaks) {
        const edge = archBolusNetwork.edges[edgeIndex];
        archBolusBranchPeaks.set(
            edgeIndex,
            Math.max(
                peak,
                edge.massMg[0] /
                    Math.max(1e-9, edge.volumes[0]) /
                    archBolusStockConcentrationMgPerMm3
            )
        );
    }
}
assert.ok(
    archBolusContinuationPeak >= 0.5,
    `43.5 ml at 39 ml/s should create one strong post-subclavian aortic column (${archBolusContinuationPeak})`
);
assert.equal(
    archBolusArtifactPeakMassMg,
    0,
    'the reported high-rate injection must not pool in the duplicate aortic strand'
);
assert.ok(
    archBolusConnectorEdge.totalVolume < 400,
    `the bridge should be a sub-0.4 ml transit volume (${archBolusConnectorEdge.totalVolume} mm3)`
);
assert.ok(
    archBolusConnectorEdge.totalVolume /
        Math.max(1e-9, archBolusConnectorEdge.meanFlowMm3PerS) < 0.03,
    'contrast should cross the bridge in less than 30 ms'
);
assert.ok(
    archBolusConnectorPeakMassMg > 0,
    'the retained bridge should carry, rather than delete, branch-bound contrast'
);
for (const [edgeIndex, peak] of archBolusBranchPeaks) {
    assert.ok(
        peak >= 0.25,
        `supra-aortic branch ${edgeIndex} should opacify during the reported bolus (${peak})`
    );
}
const archBolusConnectorResidualFraction =
    Math.max(...archBolusConnectorEdge.massMg.map(
        (mass, cellIndex) => mass / Math.max(
            1e-9,
            archBolusConnectorEdge.volumes[cellIndex] *
                archBolusStockConcentrationMgPerMm3
        )
    ));
assert.ok(
    archBolusConnectorResidualFraction < 0.02,
    `the bridge should wash out after the bolus (${archBolusConnectorResidualFraction})`
);
const correctedArchRootIndices = new Set(
    archPrefixDiagnostics.paths.map(path => path.rootEdgeIndex)
);
for (const rootEdgeIndex of [
    archAnchors.brachiocephalicTrunkRootEdgeIndex,
    archAnchors.leftCommonCarotidRootEdgeIndex,
    archAnchors.leftSubclavianRootEdgeIndex
]) {
    assert.ok(
        correctedArchRootIndices.has(rootEdgeIndex),
        `aorta-caliber technical prefix ${rootEdgeIndex} should be volume-corrected`
    );
}
const archResidenceSeconds = {
    brachiocephalic: pathResidenceSeconds(
        network,
        archAnchors.brachiocephalicTrunkRootEdgeIndex,
        archAnchors.brachiocephalicTrunkEdgeIndex
    ),
    leftCommonCarotid: pathResidenceSeconds(
        network,
        archAnchors.leftCommonCarotidRootEdgeIndex,
        archAnchors.leftCommonCarotidEdgeIndex
    ),
    leftSubclavian: pathResidenceSeconds(
        network,
        archAnchors.leftSubclavianRootEdgeIndex,
        archAnchors.leftSubclavianEdgeIndex
    )
};
assert.ok(
    archResidenceSeconds.leftCommonCarotid <= 2,
    `left common carotid arrival should not be delayed by a false ostial reservoir (${archResidenceSeconds.leftCommonCarotid}s)`
);
assert.ok(
    archResidenceSeconds.leftSubclavian <= 1.35,
    `left subclavian arrival should not be delayed by a false ostial reservoir (${archResidenceSeconds.leftSubclavian}s)`
);
assert.ok(
    Math.max(...Object.values(archResidenceSeconds)) -
        Math.min(...Object.values(archResidenceSeconds)) <= 1.5,
    `supra-aortic arrival times should remain in one angiographic phase (${JSON.stringify(archResidenceSeconds)})`
);
console.log('supra-aortic branch residence seconds', archResidenceSeconds);

const archWashoutNetwork = new ContrastFlowNetwork(
    loadCenterlineSegments()
);
const archWashoutPaths =
    archWashoutNetwork.aorticBranchPrefixDiagnostics.paths;
for (const path of archWashoutPaths) {
    for (const edgeIndex of path.edgeIndices) {
        const edge = archWashoutNetwork.edges[edgeIndex];
        for (
            let cellIndex = 0;
            cellIndex < edge.cellCount;
            cellIndex++
        ) {
            archWashoutNetwork.depositIodine(
                edgeIndex,
                cellIndex,
                edge.volumes[cellIndex]
            );
        }
    }
}
for (let frame = 0; frame < 360; frame++) {
    archWashoutNetwork.update(1 / 120);
}
for (const path of archWashoutPaths) {
    const maximumResidualFraction = Math.max(
        ...path.edgeIndices.flatMap(edgeIndex => {
            const edge = archWashoutNetwork.edges[edgeIndex];
            return [...edge.massMg].map(
                (mass, cellIndex) =>
                    mass / Math.max(1e-9, edge.volumes[cellIndex])
            );
        })
    );
    assert.ok(
        maximumResidualFraction < 0.02,
        `contrast should wash out of corrected arch prefix ${path.rootEdgeIndex} (${maximumResidualFraction})`
    );
}

const aortoiliacParent = findAortoiliacParent(network);
assert.ok(aortoiliacParent, 'the packed anatomy should expose the aortoiliac bifurcation');
const iliacChildren = aortoiliacParent.childEdgeIndices.map(
    edgeIndex => network.edges[edgeIndex]
);

let catheterSourceEdge = aortoiliacParent;
let distanceUpstreamMm = 0;
while (
    catheterSourceEdge.parentEdgeIndex >= 0 &&
    distanceUpstreamMm < 120
) {
    distanceUpstreamMm += catheterSourceEdge.length;
    catheterSourceEdge =
        network.edges[catheterSourceEdge.parentEdgeIndex];
}
assert.ok(
    distanceUpstreamMm >= 100 &&
    catheterSourceEdge.radiusEnd > 7,
    'the catheter test source should be in the abdominal aorta'
);
catheterPort.position.copy(catheterSourceEdge.start).lerp(
    catheterSourceEdge.end,
    0.5
);
catheterPort.direction.copy(catheterSourceEdge.axis);

const mainPathEdges = [];
const sideBranchEdges = [];
let pathEdge = catheterSourceEdge;
while (pathEdge && pathEdge.index !== aortoiliacParent.index) {
    mainPathEdges.push(pathEdge);
    if (!pathEdge.childEdgeIndices.length) break;
    const nextEdgeIndex = pathEdge.childEdgeIndices.reduce(
        (bestIndex, childIndex) => {
            if (bestIndex < 0) return childIndex;
            return network.edges[childIndex].meanFlowMm3PerS >
                network.edges[bestIndex].meanFlowMm3PerS
                ? childIndex
                : bestIndex;
        },
        -1
    );
    for (const childIndex of pathEdge.childEdgeIndices) {
        if (childIndex !== nextEdgeIndex) {
            sideBranchEdges.push(network.edges[childIndex]);
        }
    }
    pathEdge = network.edges[nextEdgeIndex];
}
assert.equal(
    pathEdge?.index,
    aortoiliacParent.index,
    'the selected catheter source should feed the aortoiliac bifurcation'
);
assert.ok(
    sideBranchEdges.length >= 4,
    'the catheter regression should cross several abdominal aortic branches'
);

const renderer = new ContrastVolumeRenderer(system);
assert.ok(
    renderer.flowJunctionCount > 0,
    'the lumen renderer should include finite-volume bifurcation unions'
);
const aortoiliacJunctionSlot =
    renderer.flowJunctionUnionDiagnostics.findIndex(
        diagnostic =>
            diagnostic.parentEdgeIndex === aortoiliacParent.index
    );
const aortoiliacJunctionDiagnostic =
    renderer.flowJunctionUnionDiagnostics[aortoiliacJunctionSlot];
assert.ok(
    aortoiliacJunctionDiagnostic,
    'the aortoiliac junction should expose its tube-union diagnostic'
);
if (aortoiliacJunctionDiagnostic.geometryKind === 'implicit-radius-matched-y-union') {
    assert.equal(aortoiliacJunctionDiagnostic.connectorSurfaceSuppressed, false);
} else {
    assert.equal(
        aortoiliacJunctionDiagnostic.geometryKind,
        'implicit-radius-matched-side-ostium-union'
    );
    assert.equal(
        aortoiliacJunctionDiagnostic.anatomicalClipMode,
        'disabled'
    );
    assert.equal(aortoiliacJunctionDiagnostic.connectorSurfaceSuppressed, false);
    assert.ok(aortoiliacJunctionDiagnostic.connectorVertexCount > 0);
}
assert.equal(
    renderer.flowTubeVertexCount +
        renderer.flowJunctionConnectorVertexCount,
    renderer._flowVertexConcentration.length,
    'the catheter scenario should contain only anatomical tubes and fitted Y connector vertices'
);
assert.ok(
    aortoiliacJunctionDiagnostic.connectorVertexCount > 0,
    'the aortoiliac fork must contain a fitted or contact-field union surface'
);
const flowIndices = renderer.flowMesh.geometry.index.array;
let aortoiliacUnionIndexReferenceCount = 0;
for (
    let indexOffset = renderer.flowTubeIndexCount;
    indexOffset < flowIndices.length;
    indexOffset++
) {
    const vertexIndex = flowIndices[indexOffset];
    const edgeIndex =
        renderer._flowVertexConcentrationEdgeIndex[vertexIndex];
    const edgeT = renderer._flowVertexConcentrationEdgeT[vertexIndex];
    if (
        (
            edgeIndex === aortoiliacParent.index &&
            edgeT > 0.9
        ) ||
        (
            aortoiliacParent.childEdgeIndices.includes(edgeIndex) &&
            edgeT < 0.1
        )
    ) {
        aortoiliacUnionIndexReferenceCount++;
    }
}
assert.ok(
    aortoiliacUnionIndexReferenceCount > 48,
    'the aortoiliac overlap should be rendered as a partitioned union of its anatomical tube segments'
);
const renderedAortoiliacJunctionVertices = [];
for (
    let vertexIndex = 0;
    vertexIndex < renderer._flowVertexConcentration.length;
    vertexIndex++
) {
    const edgeIndex =
        renderer._flowVertexConcentrationEdgeIndex[vertexIndex];
    const edgeT =
        renderer._flowVertexConcentrationEdgeT[vertexIndex];
    if (
        (
            edgeIndex === aortoiliacParent.index &&
            edgeT > 0.999
        ) ||
        (
            aortoiliacParent.childEdgeIndices.includes(edgeIndex) &&
            edgeT < 0.001
        )
    ) {
        renderedAortoiliacJunctionVertices.push(vertexIndex);
    }
}
const pressureLimitedAorticStart = system.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 15
});
assert.equal(pressureLimitedAorticStart.ok, true);
assert.ok(
    pressureLimitedAorticStart.actualRateMlPerSec < 15,
    `the full-tree Berenstein injection should use its pressure-limited flow (${pressureLimitedAorticStart.actualRateMlPerSec})`
);
assert.equal(pressureLimitedAorticStart.pressureLimited, true);

const stockConcentrationMgPerMm3 =
    system.medium.iodineMgPerMl / 1000;
const maximumSideBranchEntry = new Map(
    sideBranchEdges.map(edge => [edge.index, 0])
);
const maximumIliacEntry = new Map(
    iliacChildren.map(edge => [edge.index, 0])
);
const sideBranchPaths = new Map(sideBranchEdges.map(edge => [
    edge.index,
    buildDominantPathCells(network, edge, 30)
]));
const iliacPaths = new Map(iliacChildren.map(edge => [
    edge.index,
    buildDominantPathCells(network, edge, 60)
]));
const maximumSideBranchPathCoverage = new Map(
    sideBranchEdges.map(edge => [edge.index, 0])
);
const maximumIliacPathCoverage = new Map(
    iliacChildren.map(edge => [edge.index, 0])
);
const firstFullSideBranchFrame = new Map(
    sideBranchEdges.map(edge => [edge.index, -1])
);
const firstFullIliacFrame = new Map(
    iliacChildren.map(edge => [edge.index, -1])
);
let maximumSimultaneousSideBranchCoverage = 0;
let maximumSimultaneousIliacCoverage = 0;
let maximumRenderedAortoiliacJunctionMinimum = 0;
let maximumAortoiliacJunctionConcentrationOvershoot = 0;
for (let frame = 0; frame < 720; frame++) {
    system.update(1 / 120);
    for (const edge of sideBranchEdges) {
        maximumSideBranchEntry.set(
            edge.index,
            Math.max(
                maximumSideBranchEntry.get(edge.index),
                edgeEntryStockFraction(edge, stockConcentrationMgPerMm3)
            )
        );
    }
    for (const edge of iliacChildren) {
        maximumIliacEntry.set(
            edge.index,
            Math.max(
                maximumIliacEntry.get(edge.index),
                edgeEntryStockFraction(edge, stockConcentrationMgPerMm3)
            )
        );
    }

    if (frame % 4 === 0) {
        renderer.update();
        const dynamicProfile =
            renderer._flowJunctionDynamicProfiles[
                aortoiliacJunctionSlot
            ];
        const armConcentrations = dynamicProfile.arms.map(arm =>
            Math.max(...arm.samples.map(
                sample => renderer._sampleFlowCellConcentration(
                    sample.edgeIndex,
                    sample.edgeT
                )
            ))
        );
        const strongestArmConcentration = Math.max(
            ...armConcentrations
        );
        if (strongestArmConcentration > 1e-6) {
            const strongestRenderedJunctionConcentration = Math.max(
                ...renderedAortoiliacJunctionVertices.map(
                    vertexIndex =>
                        renderer._flowVertexConcentration[vertexIndex]
                )
            );
            maximumAortoiliacJunctionConcentrationOvershoot = Math.max(
                maximumAortoiliacJunctionConcentrationOvershoot,
                Math.max(
                    0,
                    strongestRenderedJunctionConcentration /
                        strongestArmConcentration - 1
                )
            );
        }
        const sideCoverages = sideBranchEdges.map(edge => {
            const coverage = renderedPathCoverage(
                renderer,
                sideBranchPaths.get(edge.index)
            );
            maximumSideBranchPathCoverage.set(
                edge.index,
                Math.max(
                    maximumSideBranchPathCoverage.get(edge.index),
                    coverage
                )
            );
            if (
                coverage >= 0.8 &&
                firstFullSideBranchFrame.get(edge.index) < 0
            ) {
                firstFullSideBranchFrame.set(edge.index, frame);
            }
            return coverage;
        });
        maximumSimultaneousSideBranchCoverage = Math.max(
            maximumSimultaneousSideBranchCoverage,
            Math.min(...sideCoverages)
        );
        const iliacCoverages = iliacChildren.map(edge => {
            const coverage = renderedPathCoverage(
                renderer,
                iliacPaths.get(edge.index)
            );
            maximumIliacPathCoverage.set(
                edge.index,
                Math.max(
                    maximumIliacPathCoverage.get(edge.index),
                    coverage
                )
            );
            if (
                coverage >= 0.8 &&
                firstFullIliacFrame.get(edge.index) < 0
            ) {
                firstFullIliacFrame.set(edge.index, frame);
            }
            return coverage;
        });
        maximumSimultaneousIliacCoverage = Math.max(
            maximumSimultaneousIliacCoverage,
            Math.min(...iliacCoverages)
        );
        maximumRenderedAortoiliacJunctionMinimum = Math.max(
            maximumRenderedAortoiliacJunctionMinimum,
            Math.min(
                ...renderedAortoiliacJunctionVertices.map(
                    vertexIndex =>
                        renderer._flowVertexConcentration[vertexIndex]
                )
            )
        );
    }
}

console.log('catheter aortic side branches', Object.fromEntries(
    maximumSideBranchEntry
));
console.log('catheter iliac entries', Object.fromEntries(
    maximumIliacEntry
));
console.log('catheter aortic branch path coverage', Object.fromEntries(
    maximumSideBranchPathCoverage
));
console.log('catheter iliac path coverage', Object.fromEntries(
    maximumIliacPathCoverage
));
console.log('catheter simultaneous path coverage', {
    aorticBranches: maximumSimultaneousSideBranchCoverage,
    iliacs: maximumSimultaneousIliacCoverage
});
console.log('catheter first 80% path fill seconds', {
    aorticBranches: Object.fromEntries(
        [...firstFullSideBranchFrame].map(
            ([edgeIndex, frame]) => [edgeIndex, frame / 120]
        )
    ),
    iliacs: Object.fromEntries(
        [...firstFullIliacFrame].map(
            ([edgeIndex, frame]) => [edgeIndex, frame / 120]
        )
    )
});
console.log(
    'catheter rendered aortoiliac junction minimum',
    maximumRenderedAortoiliacJunctionMinimum
);

for (const [edgeIndex, maximumEntry] of maximumSideBranchEntry) {
    assert.ok(
        maximumEntry >= 0.1,
        `a catheter bolus passing aortic branch ${edgeIndex} reached only ${maximumEntry}× stock`
    );
}
for (const [edgeIndex, maximumEntry] of maximumIliacEntry) {
    assert.ok(
        maximumEntry >= 0.1,
        `a top-down catheter bolus failed to opacify iliac ${edgeIndex} (${maximumEntry}× stock)`
    );
}
for (const [edgeIndex, maximumCoverage] of maximumSideBranchPathCoverage) {
    assert.ok(
        maximumCoverage >= 0.8,
        `aortic branch ${edgeIndex} filled only ${maximumCoverage * 100}% of its first 30 mm`
    );
    assert.ok(
        firstFullSideBranchFrame.get(edgeIndex) <= 360,
        `aortic branch ${edgeIndex} did not fill during the angiographic phase`
    );
}
for (const [edgeIndex, maximumCoverage] of maximumIliacPathCoverage) {
    assert.ok(
        maximumCoverage >= 0.8,
        `iliac ${edgeIndex} filled only ${maximumCoverage * 100}% of its first 60 mm`
    );
    assert.ok(
        firstFullIliacFrame.get(edgeIndex) <= 360,
        `iliac ${edgeIndex} did not fill during the angiographic phase`
    );
}
assert.ok(
    maximumSimultaneousSideBranchCoverage >= 0.65,
    `aortic branches did not remain visible together (${maximumSimultaneousSideBranchCoverage})`
);
assert.ok(
    maximumSimultaneousIliacCoverage >= 0.8,
    `both iliacs did not fill together (${maximumSimultaneousIliacCoverage})`
);
assert.ok(
    maximumRenderedAortoiliacJunctionMinimum >= 0.1,
    `the complete aortoiliac junction should fill volumetrically (${maximumRenderedAortoiliacJunctionMinimum})`
);
assert.ok(
    maximumAortoiliacJunctionConcentrationOvershoot < 1e-6,
    `the catheter time series created a concentration bead at the aortoiliac union (${maximumAortoiliacJunctionConcentrationOvershoot})`
);
assert.equal(
    renderer.flowJunctionMaterial.uniforms.maximumBlend.value,
    true,
    'the full-tree catheter renderer must premultiply MAX-union signal instead of producing dark node bands'
);

const metrics = system.getMetrics();
assert.equal(
    metrics.injectionHydraulics.actualRateMlPerSec,
    pressureLimitedAorticStart.actualRateMlPerSec,
    'clinical review metrics must retain the actual delivered flow after injection'
);
assert.ok(
    metrics.catheterSourceHandoffIodineMassMg >=
        metrics.totalInjectedIodineMassMg * 0.99,
    'the catheter plume should hand its iodine back to the source-aligned arterial volume'
);
assert.ok(
    Math.abs(metrics.relativeBalanceError) < 1e-8,
    `catheter injection must conserve iodine mass (${metrics.relativeBalanceError})`
);
renderer.dispose();

const distalSystem = new HybridContrastSystem({
    centerlineSegments: loadCenterlineSegments(),
    catheter,
    localOptions: {
        capacity: 16000,
        randomSeed: 20260731
    }
});
const distalNetwork = distalSystem.flowNetwork;
const distalAortoiliacParent =
    findAortoiliacParent(distalNetwork);
assert.ok(
    distalAortoiliacParent,
    'the distal catheter scenario should expose the aortoiliac bifurcation'
);
const simulatorSheathStart = new THREE.Vector3(
    -89.60763512409662,
    -449.43054049638647,
    28.531680733584537
);
const simulatorSheathEnd = new THREE.Vector3(-73, -383, 14);
const displayedCatheterInsertionMm = 240;
const simulatorSheathLengthMm =
    simulatorSheathStart.distanceTo(simulatorSheathEnd);
const sheathLumenLocation =
    distalNetwork.findNearestLocation(simulatorSheathEnd);
const sheathToAortoiliacTarget =
    distalNetwork.findUpstreamMixingJunction(sheathLumenLocation);
assert.ok(
    sheathToAortoiliacTarget,
    'the simulator sheath should connect to the aortoiliac bifurcation'
);
const distalAorticAdvanceMm =
    displayedCatheterInsertionMm -
    simulatorSheathLengthMm -
    sheathToAortoiliacTarget.distanceMm;
assert.ok(
    distalAorticAdvanceMm > 35 &&
    distalAorticAdvanceMm < 45,
    `24 cm of catheter should place its tip in the distal aorta (${distalAorticAdvanceMm} mm above the bifurcation)`
);
let distalSourceEdge = distalAortoiliacParent;
let distalUpstreamDistanceMm = 0;
while (
    distalSourceEdge.parentEdgeIndex >= 0 &&
    distalUpstreamDistanceMm < distalAorticAdvanceMm
) {
    distalUpstreamDistanceMm += distalSourceEdge.length;
    distalSourceEdge =
        distalNetwork.edges[distalSourceEdge.parentEdgeIndex];
}
catheterPort.position.copy(distalSourceEdge.start).lerp(
    distalSourceEdge.end,
    0.5
);
catheterPort.direction.copy(distalSourceEdge.axis);
const distalSideBranchEdges = [];
let distalPathEdge = distalSourceEdge;
while (
    distalPathEdge &&
    distalPathEdge.index !== distalAortoiliacParent.index
) {
    if (!distalPathEdge.childEdgeIndices.length) break;
    const nextEdgeIndex = distalPathEdge.childEdgeIndices.reduce(
        (bestIndex, childIndex) => {
            if (bestIndex < 0) return childIndex;
            return distalNetwork.edges[childIndex].meanFlowMm3PerS >
                distalNetwork.edges[bestIndex].meanFlowMm3PerS
                ? childIndex
                : bestIndex;
        },
        -1
    );
    for (const childIndex of distalPathEdge.childEdgeIndices) {
        if (childIndex !== nextEdgeIndex) {
            distalSideBranchEdges.push(
                distalNetwork.edges[childIndex]
            );
        }
    }
    distalPathEdge = distalNetwork.edges[nextEdgeIndex];
}
assert.equal(
    distalPathEdge?.index,
    distalAortoiliacParent.index,
    'the distal catheter source should feed the aortoiliac bifurcation'
);
assert.ok(
    distalSideBranchEdges.length >= 2,
    'a catheter inserted 24 cm should pass lower aortic branches'
);
const pigtailReference = Math.abs(distalSourceEdge.axis.y) < 0.85
    ? new THREE.Vector3(0, 1, 0)
    : new THREE.Vector3(1, 0, 0);
const pigtailNormal = new THREE.Vector3()
    .crossVectors(distalSourceEdge.axis, pigtailReference)
    .normalize();
const pigtailBinormal = new THREE.Vector3()
    .crossVectors(distalSourceEdge.axis, pigtailNormal)
    .normalize();
activeCatheterPorts = Array.from({ length: 8 }, (_, index) => {
    const angle = index / 8 * Math.PI * 2;
    return {
        kind: 'pigtail-side',
        position: catheterPort.position.clone().addScaledVector(
            distalSourceEdge.axis,
            (index - 3.5) * 1.5
        ),
        direction: pigtailNormal.clone()
            .multiplyScalar(Math.cos(angle))
            .addScaledVector(pigtailBinormal, Math.sin(angle))
            .normalize(),
        radiusMm: 0.22,
        areaMm2: Math.PI * 0.22 ** 2,
        weight: 1,
        valid: true
    };
});
catheter.type = 'pigtail';
const distalIliacChildren =
    distalAortoiliacParent.childEdgeIndices.map(
        edgeIndex => distalNetwork.edges[edgeIndex]
    );
const distalRenderer = new ContrastVolumeRenderer(distalSystem);
const distalRenderedJunctionVertices = [];
for (
    let vertexIndex = 0;
    vertexIndex < distalRenderer._flowVertexConcentration.length;
    vertexIndex++
) {
    const edgeIndex =
        distalRenderer._flowVertexConcentrationEdgeIndex[vertexIndex];
    const edgeT =
        distalRenderer._flowVertexConcentrationEdgeT[vertexIndex];
    if (
        (
            edgeIndex === distalAortoiliacParent.index &&
            edgeT > 0.999
        ) ||
        (
            distalAortoiliacParent.childEdgeIndices.includes(edgeIndex) &&
            edgeT < 0.001
        )
    ) {
        distalRenderedJunctionVertices.push(vertexIndex);
    }
}
assert.equal(distalSystem.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 18,
    rateMlPerSec: 12
}).ok, true);
const distalMaximumIliacEntry = new Map(
    distalIliacChildren.map(edge => [edge.index, 0])
);
const distalMaximumSideBranchEntry = new Map(
    distalSideBranchEdges.map(edge => [edge.index, 0])
);
const distalIliacPaths = new Map(distalIliacChildren.map(edge => [
    edge.index,
    buildDominantPathCells(distalNetwork, edge, 60)
]));
const distalSideBranchPaths = new Map(
    distalSideBranchEdges.map(edge => [
        edge.index,
        buildDominantPathCells(distalNetwork, edge, 30)
    ])
);
const distalMaximumIliacPathCoverage = new Map(
    distalIliacChildren.map(edge => [edge.index, 0])
);
const distalMaximumSideBranchPathCoverage = new Map(
    distalSideBranchEdges.map(edge => [edge.index, 0])
);
let distalMaximumSimultaneousIliacCoverage = 0;
let distalMaximumSimultaneousSideBranchCoverage = 0;
let distalRenderedJunctionMinimum = 0;
for (let frame = 0; frame < 600; frame++) {
    distalSystem.update(1 / 120);
    for (const edge of distalSideBranchEdges) {
        distalMaximumSideBranchEntry.set(
            edge.index,
            Math.max(
                distalMaximumSideBranchEntry.get(edge.index),
                edgeEntryStockFraction(
                    edge,
                    stockConcentrationMgPerMm3
                )
            )
        );
    }
    for (const edge of distalIliacChildren) {
        distalMaximumIliacEntry.set(
            edge.index,
            Math.max(
                distalMaximumIliacEntry.get(edge.index),
                edgeEntryStockFraction(
                    edge,
                    stockConcentrationMgPerMm3
                )
            )
        );
    }
    if (frame % 4 === 0) {
        distalRenderer.update();
        const distalSideBranchCoverages =
            distalSideBranchEdges.map(edge => {
                const coverage = renderedPathCoverage(
                    distalRenderer,
                    distalSideBranchPaths.get(edge.index)
                );
                distalMaximumSideBranchPathCoverage.set(
                    edge.index,
                    Math.max(
                        distalMaximumSideBranchPathCoverage.get(
                            edge.index
                        ),
                        coverage
                    )
                );
                return coverage;
            });
        distalMaximumSimultaneousSideBranchCoverage = Math.max(
            distalMaximumSimultaneousSideBranchCoverage,
            Math.min(...distalSideBranchCoverages)
        );
        const distalIliacCoverages =
            distalIliacChildren.map(edge => {
                const coverage = renderedPathCoverage(
                    distalRenderer,
                    distalIliacPaths.get(edge.index)
                );
                distalMaximumIliacPathCoverage.set(
                    edge.index,
                    Math.max(
                        distalMaximumIliacPathCoverage.get(edge.index),
                        coverage
                    )
                );
                return coverage;
            });
        distalMaximumSimultaneousIliacCoverage = Math.max(
            distalMaximumSimultaneousIliacCoverage,
            Math.min(...distalIliacCoverages)
        );
        distalRenderedJunctionMinimum = Math.max(
            distalRenderedJunctionMinimum,
            Math.min(
                ...distalRenderedJunctionVertices.map(
                    vertexIndex =>
                        distalRenderer._flowVertexConcentration[vertexIndex]
                )
            )
        );
    }
}
console.log('distal catheter aortic branch entries', Object.fromEntries(
    distalMaximumSideBranchEntry
));
console.log('distal catheter iliac entries', Object.fromEntries(
    distalMaximumIliacEntry
));
console.log('distal catheter aortic branch path coverage', Object.fromEntries(
    distalMaximumSideBranchPathCoverage
));
console.log('distal catheter iliac path coverage', Object.fromEntries(
    distalMaximumIliacPathCoverage
));
console.log(
    'distal catheter rendered aortoiliac junction minimum',
    distalRenderedJunctionMinimum
);
for (const [edgeIndex, maximumEntry] of
    distalMaximumSideBranchEntry) {
    assert.ok(
        maximumEntry >= 0.1,
        `distal catheter failed to opacify lower aortic branch ${edgeIndex} (${maximumEntry}× stock)`
    );
}
for (const [edgeIndex, maximumEntry] of distalMaximumIliacEntry) {
    assert.ok(
        maximumEntry >= 0.1,
        `distal aortic catheter failed to opacify iliac ${edgeIndex} (${maximumEntry}× stock)`
    );
}
for (const [edgeIndex, maximumCoverage] of
    distalMaximumSideBranchPathCoverage) {
    assert.ok(
        maximumCoverage >= 0.8,
        `distal catheter filled only ${maximumCoverage * 100}% of lower aortic branch ${edgeIndex}`
    );
}
for (const [edgeIndex, maximumCoverage] of
    distalMaximumIliacPathCoverage) {
    assert.ok(
        maximumCoverage >= 0.8,
        `distal catheter filled only ${maximumCoverage * 100}% of iliac ${edgeIndex}`
    );
}
assert.ok(
    distalMaximumSimultaneousIliacCoverage >= 0.8,
    `the distal catheter did not fill both iliacs together (${distalMaximumSimultaneousIliacCoverage})`
);
assert.ok(
    distalMaximumSimultaneousSideBranchCoverage >= 0.65,
    `the lower aortic branches did not fill together (${distalMaximumSimultaneousSideBranchCoverage})`
);
const distalIliacPeaks = [...distalMaximumIliacEntry.values()];
assert.ok(
    Math.min(...distalIliacPeaks) /
        Math.max(1e-9, Math.max(...distalIliacPeaks)) >= 0.85,
    `the two iliacs should fill together from above (${distalIliacPeaks})`
);
assert.ok(
    distalRenderedJunctionMinimum >= 0.1,
    `the distal catheter should fill the complete junction (${distalRenderedJunctionMinimum})`
);
const distalMetrics = distalSystem.getMetrics();
assert.ok(
    Math.abs(distalMetrics.relativeBalanceError) < 1e-8,
    `distal catheter injection must conserve iodine mass (${distalMetrics.relativeBalanceError})`
);
distalRenderer.dispose();

// Regression for the browser case reported at 22.3 cm guidewire / 24.1 cm
// Berenstein. The previous 24 cm scenario changed the device to a pigtail,
// therefore it could pass without ever exercising the single end-hole port.
const berensteinDistalSystem = new HybridContrastSystem({
    centerlineSegments: loadCenterlineSegments(),
    catheter,
    localOptions: {
        capacity: 16000,
        randomSeed: 20260802
    }
});
const berensteinDistalNetwork = berensteinDistalSystem.flowNetwork;
const berensteinAortoiliacParent =
    findAortoiliacParent(berensteinDistalNetwork);
assert.ok(
    berensteinAortoiliacParent,
    'the Berenstein regression should expose the aortoiliac bifurcation'
);
let berensteinSourceEdge = berensteinAortoiliacParent;
let berensteinSourceDistanceMm = 0;
while (
    berensteinSourceEdge.parentEdgeIndex >= 0 &&
    berensteinSourceDistanceMm < 46
) {
    berensteinSourceDistanceMm += berensteinSourceEdge.length;
    berensteinSourceEdge = berensteinDistalNetwork.edges[
        berensteinSourceEdge.parentEdgeIndex
    ];
}
assert.ok(
    berensteinSourceDistanceMm >= 46 &&
        berensteinSourceDistanceMm < 50,
    `the Berenstein port should be about 5 cm above the bifurcation (${berensteinSourceDistanceMm} mm)`
);
const berensteinReference = Math.abs(berensteinSourceEdge.axis.y) < 0.85
    ? new THREE.Vector3(0, 1, 0)
    : new THREE.Vector3(1, 0, 0);
const berensteinRadial = new THREE.Vector3()
    .crossVectors(berensteinSourceEdge.axis, berensteinReference)
    .normalize();
catheterPort.position.copy(berensteinSourceEdge.end).addScaledVector(
    berensteinRadial,
    Math.min(6.1, berensteinSourceEdge.radiusEnd * 0.78)
);
catheterPort.direction.copy(berensteinSourceEdge.axis);
activeCatheterPorts = [catheterPort];
catheter.type = 'berenstein';
const berensteinMappedLocation =
    berensteinDistalNetwork.findInjectionLocation(
        catheterPort.position,
        catheterPort.direction
    );
assert.ok(
    new Set([
        berensteinSourceEdge.index,
        berensteinSourceEdge.parentEdgeIndex,
        ...berensteinSourceEdge.childEdgeIndices
    ]).has(berensteinMappedLocation.edgeIndex) &&
        Math.min(
            berensteinDistalNetwork.edges[
                berensteinMappedLocation.edgeIndex
            ].radiusStart,
            berensteinDistalNetwork.edges[
                berensteinMappedLocation.edgeIndex
            ].radiusEnd
        ) > 6,
    `the near-wall Berenstein end hole should map to the same common-aortic segment chain (${berensteinSourceEdge.index}/${berensteinMappedLocation.edgeIndex})`
);
assert.equal(
    berensteinMappedLocation.selectionMode,
    'containing-lumen'
);
const berensteinIliacs = berensteinAortoiliacParent.childEdgeIndices.map(
    edgeIndex => berensteinDistalNetwork.edges[edgeIndex]
);
const berensteinIliacPaths = new Map(berensteinIliacs.map(edge => [
    edge.index,
    buildDominantPathCells(berensteinDistalNetwork, edge, 60)
]));
const berensteinRenderer = new ContrastVolumeRenderer(
    berensteinDistalSystem
);
assert.equal(berensteinDistalSystem.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 15
}).ok, true);
const berensteinIliacPeaks = new Map(
    berensteinIliacs.map(edge => [edge.index, 0])
);
const berensteinFirstArrivalFrame = new Map(
    berensteinIliacs.map(edge => [edge.index, -1])
);
let berensteinMaximumSimultaneousCoverage = 0;
for (let frame = 0; frame < 480; frame++) {
    berensteinDistalSystem.update(1 / 120);
    for (const edge of berensteinIliacs) {
        const entry = edgeEntryStockFraction(
            edge,
            stockConcentrationMgPerMm3
        );
        berensteinIliacPeaks.set(
            edge.index,
            Math.max(berensteinIliacPeaks.get(edge.index), entry)
        );
        if (
            entry >= 0.05 &&
            berensteinFirstArrivalFrame.get(edge.index) < 0
        ) {
            berensteinFirstArrivalFrame.set(edge.index, frame);
        }
    }
    if (frame % 4 === 0) {
        berensteinRenderer.update();
        berensteinMaximumSimultaneousCoverage = Math.max(
            berensteinMaximumSimultaneousCoverage,
            Math.min(...berensteinIliacs.map(edge =>
                renderedPathCoverage(
                    berensteinRenderer,
                    berensteinIliacPaths.get(edge.index)
                )
            ))
        );
    }
}
const berensteinPeaks = [...berensteinIliacPeaks.values()];
const berensteinArrivals = [...berensteinFirstArrivalFrame.values()];
console.log('Berenstein 24.1 cm iliac regression', {
    sourceEdgeIndex: berensteinSourceEdge.index,
    sourceDistanceMm: berensteinSourceDistanceMm,
    mappedDistanceMm: berensteinMappedLocation.distance,
    iliacPeaks: berensteinPeaks,
    arrivalSeconds: berensteinArrivals.map(frame => frame / 120),
    maximumSimultaneousCoverage:
        berensteinMaximumSimultaneousCoverage
});
assert.ok(
    berensteinArrivals.every(frame => frame >= 0),
    `Berenstein contrast did not reach both iliacs (${berensteinArrivals})`
);
assert.ok(
    Math.max(...berensteinArrivals) - Math.min(...berensteinArrivals) <= 6,
    `the two iliacs should start filling together (${berensteinArrivals})`
);
assert.ok(
    Math.min(...berensteinPeaks) /
        Math.max(1e-9, Math.max(...berensteinPeaks)) >= 0.85,
    `the near-wall Berenstein source filled the iliacs asymmetrically (${berensteinPeaks})`
);
assert.ok(
    berensteinMaximumSimultaneousCoverage >= 0.8,
    `Berenstein failed to render both iliac columns together (${berensteinMaximumSimultaneousCoverage})`
);
const berensteinDiagnostics = berensteinDistalSystem.getMetrics();
assert.equal(
    berensteinDiagnostics.catheterSourceMappingChangeCount,
    0,
    'a stationary Berenstein port must keep one topological source during injection'
);
assert.ok(
    Math.abs(berensteinDiagnostics.relativeBalanceError) < 1e-8,
    `Berenstein regression must conserve iodine mass (${berensteinDiagnostics.relativeBalanceError})`
);
berensteinRenderer.dispose();

activeCatheterPorts = [catheterPort];
catheter.type = 'berenstein';
let proximalSourceEdge = aortoiliacParent;
let proximalSourceDistanceMm = 0;
while (
    proximalSourceEdge.parentEdgeIndex >= 0 &&
    proximalSourceDistanceMm < 300
) {
    proximalSourceDistanceMm += proximalSourceEdge.length;
    proximalSourceEdge =
        network.edges[proximalSourceEdge.parentEdgeIndex];
}
assert.ok(
    proximalSourceDistanceMm >= 300,
    'the proximal catheter source should be in the upper aorta'
);
catheterPort.position.copy(proximalSourceEdge.start).lerp(
    proximalSourceEdge.end,
    0.5
);
catheterPort.direction.copy(proximalSourceEdge.axis);
const proximalMappedLocation =
    network.findInjectionLocation(
        catheterPort.position,
        catheterPort.direction
    );
assert.equal(
    proximalMappedLocation.edgeIndex,
    proximalSourceEdge.index,
    'the proximal aortic catheter should map back to its containing lumen'
);
const proximalIliacChildren =
    aortoiliacParent.childEdgeIndices.map(
        edgeIndex => network.edges[edgeIndex]
    );
const proximalIliacPaths = new Map(
    proximalIliacChildren.map(edge => [
        edge.index,
        buildDominantPathCells(network, edge, 60)
    ])
);
assert.equal(system.startInjection({
    source: CONTRAST_SOURCE_CATHETER,
    volumeMl: 30,
    rateMlPerSec: 15
}).ok, true);
let proximalFirstSimultaneousIliacFrame = -1;
let proximalMaximumSimultaneousIliacCoverage = 0;
for (let frame = 0; frame < 960; frame++) {
    system.update(1 / 120);
    const coverages = proximalIliacChildren.map(edge =>
        physicalPathCoverage(
            network,
            proximalIliacPaths.get(edge.index),
            stockConcentrationMgPerMm3
        )
    );
    const simultaneousCoverage = Math.min(...coverages);
    proximalMaximumSimultaneousIliacCoverage = Math.max(
        proximalMaximumSimultaneousIliacCoverage,
        simultaneousCoverage
    );
    if (
        simultaneousCoverage >= 0.8 &&
        proximalFirstSimultaneousIliacFrame < 0
    ) {
        proximalFirstSimultaneousIliacFrame = frame;
    }
}
console.log('proximal catheter iliac arrival', {
    sourceEdgeIndex: proximalSourceEdge.index,
    sourceDistanceMm: proximalSourceDistanceMm,
    firstSimultaneousFillSeconds:
        proximalFirstSimultaneousIliacFrame / 120,
    maximumSimultaneousCoverage:
        proximalMaximumSimultaneousIliacCoverage
});
assert.ok(
    proximalFirstSimultaneousIliacFrame >= 0 &&
    proximalFirstSimultaneousIliacFrame <= 720,
    'contrast from the upper aorta should reach both iliacs within six seconds'
);
assert.ok(
    proximalMaximumSimultaneousIliacCoverage >= 0.8,
    `the proximal catheter filled only ${proximalMaximumSimultaneousIliacCoverage * 100}% of both iliacs`
);
const proximalMetrics = system.getMetrics();
assert.ok(
    Math.abs(proximalMetrics.relativeBalanceError) < 1e-8,
    `proximal catheter injection must conserve iodine mass (${proximalMetrics.relativeBalanceError})`
);

console.log('catheter full-tree contrast test passed');
