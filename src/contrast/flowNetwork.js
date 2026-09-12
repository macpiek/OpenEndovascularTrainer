import * as THREE from 'three';
import { repairArterialFlowTopology } from '../arterialFlowTopology.js';

const MM3_PER_ML = 1000;
const SECONDS_PER_MINUTE = 60;
const MIN_RADIUS_MM = 0.35;
const DEFAULT_CELL_LENGTH_MM = 2.5;
const DEFAULT_SPATIAL_CELL_MM = 12;
const DEFAULT_TERMINAL_LENGTH_MM = 800;
const DEFAULT_TERMINAL_FLOW_RADIUS_EXPONENT = 2;
const DEFAULT_MINIMUM_TERMINAL_PERFUSION_RADIUS_MM = 1.2;
const DEFAULT_LOCAL_CALIBER_FLOW_BLEND = 0.72;
const DEFAULT_LOCAL_CALIBER_FLOW_EXPONENT = 2;
const DEFAULT_BRANCH_CALIBER_SAMPLE_DISTANCE_MM = 6;
const DEFAULT_MAXIMUM_CALIBER_CORRECTED_TERMINAL_OUTLETS = 2;
const DEFAULT_CFL_LIMIT = 0.72;
const MIN_ACTIVE_EDGE_MASS_MG = 5e-4;
const MAX_TRANSPORT_SUBSTEPS = 24;
const RADIUS_VALLEY_WINDOW_MM = 28;
const RADIUS_VALLEY_SUPPORT_MM = 6;
const RADIUS_VALLEY_RATIO = 0.78;
const RADIUS_VALLEY_RESTORE_RATIO = 0.92;
const AORTIC_PREFIX_MIN_PARENT_RADIUS_MM = 10;
const AORTIC_PREFIX_START_RADIUS_RATIO = 0.78;
const AORTIC_PREFIX_EXIT_RADIUS_RATIO = 0.65;
const AORTIC_PREFIX_MAX_LENGTH_MM = 35;
const AORTIC_PREFIX_CALIBER_PROBE_MM = 4;
const AORTIC_PREFIX_OSTIAL_FLARE_RATIO = 1.35;
const AORTIC_PREFIX_MAX_BRANCH_RADIUS_MM = 6.5;
const AORTIC_DUPLICATE_MIN_PARENT_RADIUS_MM = 10;
const AORTIC_DUPLICATE_MIN_RADIUS_RATIO = 0.68;
const AORTIC_DUPLICATE_MIN_DIRECTION_DOT = 0.7;
const AORTIC_DUPLICATE_MAX_LENGTH_MM = 80;
const AORTIC_DUPLICATE_MIN_MAIN_OUTLETS = 3;
const AORTIC_DUPLICATE_MAX_REJOIN_DISTANCE_RATIO = 0.35;
const AORTIC_CONNECTOR_MAX_LENGTH_MM = 5;
const AORTIC_CONNECTOR_MIN_RADIUS_RATIO = 0.78;
const AORTIC_CONNECTOR_BRANCH_EXIT_RADIUS_RATIO = 0.65;
const AORTIC_CONNECTOR_BRANCH_PROBE_MM = 35;

export const DEFAULT_HEMODYNAMICS = Object.freeze({
    heartRateBpm: 72,
    cardiacOutputMlPerMin: 5000,
    bloodDensityKgPerM3: 1060,
    bloodViscosityPaS: 0.0035,
    axialDispersionMm2PerS: 10
});

function endpointRadius(segment, nodeId) {
    if (segment.nodeStartId === nodeId) return segment.radiusStart;
    if (segment.nodeEndId === nodeId) return segment.radiusEnd;
    return 0;
}

function clonePoint(point) {
    return point?.clone?.() || new THREE.Vector3(point?.x || 0, point?.y || 0, point?.z || 0);
}

function cellKey(x, y, z) {
    return `${x}|${y}|${z}`;
}

function frustumVolume(length, radiusStart, radiusEnd) {
    return Math.PI * length *
        (radiusStart * radiusStart + radiusStart * radiusEnd + radiusEnd * radiusEnd) / 3;
}

function createNode(nodeId, point) {
    return {
        id: nodeId,
        point: clonePoint(point),
        links: [],
        parentEdgeIndex: -1,
        childEdgeIndices: [],
        equivalentResistance: Infinity,
        terminalRadius: MIN_RADIUS_MM,
        perfusionWeight: 0,
        terminalOutletCount: 0
    };
}

function buildUndirectedNodes(sourceSegments) {
    const nodes = new Map();
    const ensureNode = (nodeId, point) => {
        let node = nodes.get(nodeId);
        if (!node) {
            node = createNode(nodeId, point);
            nodes.set(nodeId, node);
        }
        return node;
    };

    sourceSegments.forEach((segment, sourceIndex) => {
        const startNode = ensureNode(segment.nodeStartId, segment.start);
        const endNode = ensureNode(segment.nodeEndId, segment.end);
        startNode.links.push({ sourceIndex, otherId: endNode.id });
        endNode.links.push({ sourceIndex, otherId: startNode.id });
        startNode.terminalRadius = Math.max(
            startNode.terminalRadius,
            endpointRadius(segment, startNode.id) || 0
        );
        endNode.terminalRadius = Math.max(
            endNode.terminalRadius,
            endpointRadius(segment, endNode.id) || 0
        );
    });
    return nodes;
}

function chooseRootNode(nodes, sourceSegments, rootPoint = null) {
    const endpoints = [...nodes.values()].filter(node => node.links.length === 1);
    const candidates = endpoints.length ? endpoints : [...nodes.values()];
    if (rootPoint) {
        const target = clonePoint(rootPoint);
        return candidates.reduce((best, node) =>
            !best || node.point.distanceToSquared(target) < best.point.distanceToSquared(target)
                ? node
                : best, null);
    }
    return candidates.reduce((best, node) => {
        const radius = node.links.length
            ? endpointRadius(sourceSegments[node.links[0].sourceIndex], node.id)
            : 0;
        if (!best || radius > best.radius) return { node, radius };
        return best;
    }, null)?.node || null;
}

function orientedEdge(source, sourceIndex, startNode, endNode, edgeIndex, {
    cellLengthMm,
    bloodViscosityPaS
}) {
    const forward = source.nodeStartId === startNode.id;
    const start = clonePoint(forward ? source.start : source.end);
    const end = clonePoint(forward ? source.end : source.start);
    const radiusStart = Math.max(
        MIN_RADIUS_MM,
        forward ? source.radiusStart : source.radiusEnd
    );
    const radiusEnd = Math.max(
        MIN_RADIUS_MM,
        forward ? source.radiusEnd : source.radiusStart
    );
    const axis = end.clone().sub(start);
    const length = Math.max(1e-6, axis.length());
    axis.multiplyScalar(1 / length);
    const cellCount = Math.max(1, Math.ceil(length / cellLengthMm));
    const actualCellLength = length / cellCount;
    const volumes = new Float64Array(cellCount);
    const areas = new Float64Array(cellCount);
    let totalVolume = 0;
    for (let cellIndex = 0; cellIndex < cellCount; cellIndex++) {
        const t0 = cellIndex / cellCount;
        const t1 = (cellIndex + 1) / cellCount;
        const r0 = THREE.MathUtils.lerp(radiusStart, radiusEnd, t0);
        const r1 = THREE.MathUtils.lerp(radiusStart, radiusEnd, t1);
        const volume = frustumVolume(actualCellLength, r0, r1);
        volumes[cellIndex] = volume;
        areas[cellIndex] = Math.PI * ((r0 + r1) * 0.5) ** 2;
        totalVolume += volume;
    }
    const effectiveRadius = Math.max(MIN_RADIUS_MM, (radiusStart + radiusEnd) * 0.5);
    const resistance = 8 * bloodViscosityPaS * length /
        (Math.PI * effectiveRadius ** 4);
    return {
        index: edgeIndex,
        sourceIndex,
        sourceId: source.id ?? sourceIndex,
        startNodeId: startNode.id,
        endNodeId: endNode.id,
        start,
        end,
        axis,
        length,
        radiusStart,
        radiusEnd,
        rawRadiusStart: radiusStart,
        rawRadiusEnd: radiusEnd,
        safeRadius: source.safeRadius || Math.min(radiusStart, radiusEnd),
        cellCount,
        cellLength: actualCellLength,
        volumes,
        areas,
        totalVolume,
        resistance,
        equivalentResistance: Infinity,
        meanFlowMm3PerS: 0,
        active: false,
        massMg: new Float64Array(cellCount),
        nextMassMg: new Float64Array(cellCount),
        dispersionDeltaMg: new Float64Array(cellCount),
        // Additive signed-flow corrections at the cell faces. A device outlet
        // inside an edge is a volume source, so the proximal and distal faces
        // of its source cell generally carry different flows. Keeping this at
        // face resolution avoids reversing an entire anatomical segment just
        // because the catheter tip lies part-way along it.
        faceFlowDeltaMm3PerS: new Float64Array(cellCount + 1),
        meanConcentrationMgPerMm3: 0,
        childEdgeIndices: []
    };
}

export function arterialWaveform(timeSeconds, heartRateBpm = 72) {
    const phase = timeSeconds * heartRateBpm / SECONDS_PER_MINUTE * Math.PI * 2;
    return Math.max(
        0.18,
        1 +
        Math.sin(phase - 0.35) * 0.52 +
        Math.sin(phase * 2 - 1.05) * 0.18 +
        Math.sin(phase * 3 - 1.7) * 0.07
    );
}

export class ContrastFlowNetwork {
    constructor(sourceSegments, {
        rootPoint = null,
        cellLengthMm = DEFAULT_CELL_LENGTH_MM,
        spatialCellMm = DEFAULT_SPATIAL_CELL_MM,
        terminalResistanceLengthMm = DEFAULT_TERMINAL_LENGTH_MM,
        terminalFlowRadiusExponent = DEFAULT_TERMINAL_FLOW_RADIUS_EXPONENT,
        minimumTerminalPerfusionRadiusMm =
            DEFAULT_MINIMUM_TERMINAL_PERFUSION_RADIUS_MM,
        localCaliberFlowBlend = DEFAULT_LOCAL_CALIBER_FLOW_BLEND,
        localCaliberFlowExponent = DEFAULT_LOCAL_CALIBER_FLOW_EXPONENT,
        branchCaliberSampleDistanceMm =
            DEFAULT_BRANCH_CALIBER_SAMPLE_DISTANCE_MM,
        maximumCaliberCorrectedTerminalOutlets =
            DEFAULT_MAXIMUM_CALIBER_CORRECTED_TERMINAL_OUTLETS,
        cflLimit = DEFAULT_CFL_LIMIT,
        ...hemodynamicOverrides
    } = {}) {
        if (!Array.isArray(sourceSegments) || !sourceSegments.length) {
            throw new TypeError('A non-empty centerline segment array is required');
        }
        this.hemodynamics = {
            ...DEFAULT_HEMODYNAMICS,
            ...hemodynamicOverrides
        };
        const topologyRepair = repairArterialFlowTopology(sourceSegments);
        this.sourceSegments = topologyRepair.segments;
        this.topologyRepairDiagnostics = topologyRepair.diagnostics;
        this.nodes = buildUndirectedNodes(this.sourceSegments);
        this.rootNode = chooseRootNode(
            this.nodes,
            this.sourceSegments,
            rootPoint || topologyRepair.rootPoint
        );
        if (!this.rootNode) throw new Error('Unable to select the arterial inlet');
        this.cellLengthMm = cellLengthMm;
        this.spatialCellMm = spatialCellMm;
        this.terminalResistanceLengthMm = terminalResistanceLengthMm;
        this.terminalFlowRadiusExponent = THREE.MathUtils.clamp(
            terminalFlowRadiusExponent,
            1,
            4
        );
        this.minimumTerminalPerfusionRadiusMm = Math.max(
            MIN_RADIUS_MM,
            minimumTerminalPerfusionRadiusMm
        );
        this.localCaliberFlowBlend = THREE.MathUtils.clamp(
            localCaliberFlowBlend,
            0,
            1
        );
        this.localCaliberFlowExponent = THREE.MathUtils.clamp(
            localCaliberFlowExponent,
            1,
            4
        );
        this.branchCaliberSampleDistanceMm = Math.max(
            0,
            branchCaliberSampleDistanceMm
        );
        this.maximumCaliberCorrectedTerminalOutlets = Math.max(
            0,
            Math.floor(maximumCaliberCorrectedTerminalOutlets)
        );
        this.cflLimit = cflLimit;
        this.edges = [];
        this.nodeOrder = [];
        this.disconnectedSourceSegmentCount = 0;
        this.time = 0;
        this.outletIodineMassMg = 0;
        this.totalIodineMassMg = 0;
        this._edgeOutMassMg = new Float64Array(this.sourceSegments.length);
        this._edgeUpstreamOutMassMg = new Float64Array(this.sourceSegments.length);
        this._flowOverridesMm3PerS = new Float64Array(this.sourceSegments.length);
        this._flowOverridesMm3PerS.fill(Number.NaN);
        this._faceFlowDeltaEdgeIndices = new Set();
        this._branchInletFlowMm3PerS =
            new Float64Array(this.sourceSegments.length);
        this._branchInletConcentrationMgPerMm3 =
            new Float64Array(this.sourceSegments.length);
        this._branchInletEdgeIndices = new Set();
        this._spatialIndex = new Map();
        this._queryMarks = new Uint32Array(this.sourceSegments.length);
        this._queryEpoch = 0;
        this._locationScratch = {};
        this._activeEdgeIndices = new Set();
        this._touchMarks = new Uint32Array(this.sourceSegments.length);
        this._touchEpoch = 0;
        this._touchedEdgeIndices = [];
        this._buildDirectedTree();
        this._contractIntraluminalAorticBranchConnectors();
        this._suppressIntraluminalAorticDeadEnds();
        this._regularizeShortRadiusValleys();
        this._regularizeAorticBranchPrefixes();
        this._regularizeIntraluminalAorticConnectorVolumes();
        this._edgeOutMassMg = new Float64Array(this.edges.length);
        this._edgeUpstreamOutMassMg = new Float64Array(this.edges.length);
        this._flowOverridesMm3PerS = new Float64Array(this.edges.length);
        this._flowOverridesMm3PerS.fill(Number.NaN);
        this._branchInletFlowMm3PerS =
            new Float64Array(this.edges.length);
        this._branchInletConcentrationMgPerMm3 =
            new Float64Array(this.edges.length);
        this._queryMarks = new Uint32Array(this.edges.length);
        this._touchMarks = new Uint32Array(this.edges.length);
        this._computeHydraulicDistribution();
        this._buildSpatialIndex();
        this.minimumCellLength = this.edges.reduce(
            (minimum, edge) => Math.min(minimum, edge.cellLength),
            Infinity
        );
        this._updateMaximumMeanVelocity();
    }

    _updateMaximumMeanVelocity() {
        this.maximumMeanVelocity = this.edges.reduce((maximum, edge) => {
            let edgeMaximum = 0;
            for (let index = 0; index < edge.cellCount; index++) {
                edgeMaximum = Math.max(
                    edgeMaximum,
                    edge.meanFlowMm3PerS / Math.max(1e-6, edge.areas[index])
                );
            }
            return Math.max(maximum, edgeMaximum);
        }, 0);
    }

    setHemodynamics({ cardiacOutputMlPerMin, heartRateBpm } = {}) {
        let flowChanged = false;
        if (cardiacOutputMlPerMin !== undefined) {
            if (!Number.isFinite(cardiacOutputMlPerMin) || cardiacOutputMlPerMin <= 0) {
                throw new RangeError('cardiacOutputMlPerMin must be positive');
            }
            flowChanged =
                Math.abs(cardiacOutputMlPerMin - this.hemodynamics.cardiacOutputMlPerMin) >
                1e-9;
            this.hemodynamics.cardiacOutputMlPerMin = cardiacOutputMlPerMin;
        }
        if (heartRateBpm !== undefined) {
            if (!Number.isFinite(heartRateBpm) || heartRateBpm <= 0) {
                throw new RangeError('heartRateBpm must be positive');
            }
            this.hemodynamics.heartRateBpm = heartRateBpm;
        }
        if (flowChanged) {
            this._computeHydraulicDistribution();
            this._updateMaximumMeanVelocity();
        }
        return { ...this.hemodynamics };
    }

    clearFlowOverrides() {
        this._flowOverridesMm3PerS.fill(Number.NaN);
        for (const edgeIndex of this._faceFlowDeltaEdgeIndices) {
            this.edges[edgeIndex]?.faceFlowDeltaMm3PerS.fill(0);
        }
        this._faceFlowDeltaEdgeIndices.clear();
        this._branchInletFlowMm3PerS.fill(0);
        this._branchInletConcentrationMgPerMm3.fill(0);
        this._branchInletEdgeIndices.clear();
    }

    setFlowOverride(edgeIndices, signedFlowMm3PerS) {
        if (!Number.isFinite(signedFlowMm3PerS)) {
            throw new RangeError('signedFlowMm3PerS must be finite');
        }
        for (const edgeIndex of edgeIndices) {
            if (this.edges[edgeIndex]) {
                this._flowOverridesMm3PerS[edgeIndex] = signedFlowMm3PerS;
            }
        }
    }

    getSignedFlowMm3PerS(edgeIndex, waveform = 1) {
        const override = this._flowOverridesMm3PerS[edgeIndex];
        const baseFlow = Number.isFinite(override)
            ? override
            : (this.edges[edgeIndex]?.meanFlowMm3PerS || 0) * waveform;
        const edge = this.edges[edgeIndex];
        if (!edge) return baseFlow;
        const faces = edge.faceFlowDeltaMm3PerS;
        return baseFlow + (faces[0] + faces[faces.length - 1]) * 0.5;
    }

    getFaceSignedFlowMm3PerS(edgeIndex, faceIndex, waveform = 1) {
        const edge = this.edges[edgeIndex];
        if (!edge) return 0;
        const override = this._flowOverridesMm3PerS[edgeIndex];
        const baseFlow = Number.isFinite(override)
            ? override
            : edge.meanFlowMm3PerS * waveform;
        const resolvedFaceIndex = THREE.MathUtils.clamp(
            Math.floor(faceIndex),
            0,
            edge.cellCount
        );
        return baseFlow +
            edge.faceFlowDeltaMm3PerS[resolvedFaceIndex];
    }

    addFaceFlowDelta(
        edgeIndex,
        startFaceIndex,
        endFaceIndexExclusive,
        deltaMm3PerS
    ) {
        const edge = this.edges[edgeIndex];
        if (!edge || !Number.isFinite(deltaMm3PerS) || !deltaMm3PerS) {
            return false;
        }
        const start = THREE.MathUtils.clamp(
            Math.floor(startFaceIndex),
            0,
            edge.cellCount + 1
        );
        const end = THREE.MathUtils.clamp(
            Math.ceil(endFaceIndexExclusive),
            start,
            edge.cellCount + 1
        );
        for (let faceIndex = start; faceIndex < end; faceIndex++) {
            edge.faceFlowDeltaMm3PerS[faceIndex] += deltaMm3PerS;
        }
        if (end > start) this._faceFlowDeltaEdgeIndices.add(edgeIndex);
        return end > start;
    }

    pushRetrogradeColumn({
        pathCells,
        aorticPathStartIndex = pathCells?.length ?? 0,
        volumeMm3,
        stockConcentrationMgPerMm3
    }) {
        if (
            !pathCells?.length ||
            !(volumeMm3 > 0) ||
            !(stockConcentrationMgPerMm3 > 0)
        ) return false;

        const oldMassMg = new Float64Array(pathCells.length);
        let pathVolumeMm3 = 0;
        let oldPathMassMg = 0;
        let oldAorticMassMg = 0;
        for (let index = 0; index < pathCells.length; index++) {
            const cell = pathCells[index];
            const edge = this.edges[cell.edgeIndex];
            cell.volumeStartMm3 = pathVolumeMm3;
            pathVolumeMm3 += edge.volumes[cell.cellIndex];
            cell.volumeEndMm3 = pathVolumeMm3;
            oldMassMg[index] = edge.massMg[cell.cellIndex];
            oldPathMassMg += oldMassMg[index];
            if (index >= aorticPathStartIndex) {
                oldAorticMassMg += oldMassMg[index];
            }
        }

        let newPathMassMg = 0;
        let newAorticMassMg = 0;
        for (let destinationIndex = 0;
            destinationIndex < pathCells.length;
            destinationIndex++
        ) {
            const destination = pathCells[destinationIndex];
            let destinationMassMg = 0;

            const boundaryEnd = Math.min(
                destination.volumeEndMm3,
                volumeMm3
            );
            if (boundaryEnd > destination.volumeStartMm3) {
                destinationMassMg +=
                    (boundaryEnd - destination.volumeStartMm3) *
                    stockConcentrationMgPerMm3;
            }

            const sourceStartMm3 = Math.max(
                0,
                destination.volumeStartMm3 - volumeMm3
            );
            const sourceEndMm3 = Math.min(
                pathVolumeMm3,
                destination.volumeEndMm3 - volumeMm3
            );
            if (sourceEndMm3 > sourceStartMm3) {
                for (let sourceIndex = 0;
                    sourceIndex < pathCells.length;
                    sourceIndex++
                ) {
                    const source = pathCells[sourceIndex];
                    const overlapStart = Math.max(
                        sourceStartMm3,
                        source.volumeStartMm3
                    );
                    const overlapEnd = Math.min(
                        sourceEndMm3,
                        source.volumeEndMm3
                    );
                    if (overlapEnd <= overlapStart) continue;
                    const sourceVolume =
                        source.volumeEndMm3 - source.volumeStartMm3;
                    destinationMassMg += oldMassMg[sourceIndex] *
                        (overlapEnd - overlapStart) /
                        Math.max(1e-9, sourceVolume);
                }
            }

            const edge = this.edges[destination.edgeIndex];
            edge.massMg[destination.cellIndex] = destinationMassMg;
            edge.active = edge.active || destinationMassMg > 0;
            if (edge.active) this._activeEdgeIndices.add(edge.index);
            newPathMassMg += destinationMassMg;
            if (destinationIndex >= aorticPathStartIndex) {
                newAorticMassMg += destinationMassMg;
            }
        }

        const injectedMassMg = volumeMm3 * stockConcentrationMgPerMm3;
        const overflowMassMg = Math.max(
            0,
            oldPathMassMg + injectedMassMg - newPathMassMg
        );
        if (overflowMassMg > 0) {
            // The resolved retrograde path reaches the arterial inlet. Excess
            // iodine has left the represented tree and must not be piled into
            // one terminal cell, which would create an unphysical stationary
            // black segment during washout.
            this.outletIodineMassMg += overflowMassMg;
        }
        const aorticMassMg =
            Math.max(0, newAorticMassMg - oldAorticMassMg) +
            overflowMassMg;
        this.totalIodineMassMg += injectedMassMg;
        return {
            injectedMassMg,
            aorticMassMg,
            overflowMassMg
        };
    }

    configureSideBranchInlets({
        pathEdgeIndices,
        excludedEdgeIndices = [],
        availableFlowMm3PerS,
        stockConcentrationMgPerMm3
    }) {
        if (
            !pathEdgeIndices?.length ||
            !(availableFlowMm3PerS > 0) ||
            !(stockConcentrationMgPerMm3 > 0)
        ) {
            return { inletFlowMm3PerS: 0, perfusedBranchCount: 0 };
        }
        const excluded = new Set(excludedEdgeIndices);
        const waveform = arterialWaveform(
            this.time,
            this.hemodynamics.heartRateBpm
        );
        const candidates = [];
        let desiredFlowMm3PerS = 0;

        for (const parentEdgeIndex of pathEdgeIndices) {
            const parent = this.edges[parentEdgeIndex];
            if (!parent?.childEdgeIndices.length) continue;
            const parentCellIndex = parent.cellCount - 1;
            const parentVolume = Math.max(
                1e-9,
                parent.volumes[parentCellIndex]
            );
            const parentMassMg =
                parent.massMg[parentCellIndex];
            const parentConcentration =
                parentMassMg / parentVolume /
                stockConcentrationMgPerMm3;
            if (!(parentConcentration > 0.02)) continue;

            for (const childIndex of parent.childEdgeIndices) {
                if (
                    excluded.has(childIndex) ||
                    Number.isFinite(
                        this._flowOverridesMm3PerS[childIndex]
                    )
                ) continue;
                const child = this.edges[childIndex];
                if (!(child.meanFlowMm3PerS > 0)) continue;
                const desiredChildFlowMm3PerS =
                    child.meanFlowMm3PerS * waveform;
                candidates.push({
                    child,
                    desiredChildFlowMm3PerS
                });
                desiredFlowMm3PerS +=
                    desiredChildFlowMm3PerS;
            }
        }

        if (!(desiredFlowMm3PerS > 0)) {
            return { inletFlowMm3PerS: 0, perfusedBranchCount: 0 };
        }
        const flowScale = Math.min(
            1,
            availableFlowMm3PerS / desiredFlowMm3PerS
        );
        let inletFlowMm3PerS = 0;
        for (const {
            child,
            desiredChildFlowMm3PerS
        } of candidates) {
            const flowMm3PerS =
                desiredChildFlowMm3PerS * flowScale;
            this._branchInletFlowMm3PerS[child.index] =
                flowMm3PerS;
            this._branchInletConcentrationMgPerMm3[child.index] =
                stockConcentrationMgPerMm3;
            this._branchInletEdgeIndices.add(child.index);
            inletFlowMm3PerS += flowMm3PerS;
        }
        return {
            inletFlowMm3PerS,
            perfusedBranchCount: candidates.length
        };
    }

    _buildDirectedTree() {
        const queue = [this.rootNode];
        const visitedNodes = new Set([this.rootNode.id]);
        let queueIndex = 0;
        while (queueIndex < queue.length) {
            const node = queue[queueIndex++];
            this.nodeOrder.push(node.id);
            for (const link of node.links) {
                if (visitedNodes.has(link.otherId)) continue;
                const child = this.nodes.get(link.otherId);
                if (!child) continue;
                visitedNodes.add(child.id);
                const edge = orientedEdge(
                    this.sourceSegments[link.sourceIndex],
                    link.sourceIndex,
                    node,
                    child,
                    this.edges.length,
                    {
                        cellLengthMm: this.cellLengthMm,
                        bloodViscosityPaS: this.hemodynamics.bloodViscosityPaS
                    }
                );
                this.edges.push(edge);
                node.childEdgeIndices.push(edge.index);
                child.parentEdgeIndex = edge.index;
                queue.push(child);
            }
        }
        this.disconnectedSourceSegmentCount = this.sourceSegments.length - this.edges.length;
        for (const edge of this.edges) {
            edge.childEdgeIndices = this.nodes.get(edge.endNodeId)?.childEdgeIndices || [];
            edge.parentEdgeIndex = this.nodes.get(edge.startNodeId)?.parentEdgeIndex ?? -1;
        }
    }

    /**
     * Some supra-aortic branches share a very short medial-axis connector
     * whose full aortic radius comes from samples still inside the arch. It
     * has no independent anatomical lumen, so transporting contrast through
     * its large volume creates an ostial reservoir between neighbouring
     * branch origins. Preserve it as a topological bridge so transport and
     * rendering stay continuous, but do not map catheter ports to it or draw
     * it as a separate lumen.
     */
    _contractIntraluminalAorticBranchConnectors() {
        const reachesBranchCaliber = (rootEdgeIndex, parentRadiusMm) => {
            let edge = this.edges[rootEdgeIndex];
            let distanceMm = 0;
            while (edge && distanceMm <= AORTIC_CONNECTOR_BRANCH_PROBE_MM) {
                distanceMm += edge.length;
                if (
                    edge.rawRadiusEnd <=
                        parentRadiusMm *
                            AORTIC_CONNECTOR_BRANCH_EXIT_RADIUS_RATIO
                ) return true;
                if (edge.childEdgeIndices.length !== 1) return false;
                edge = this.edges[edge.childEdgeIndices[0]];
            }
            return false;
        };

        const candidates = [];
        for (const node of this.nodes.values()) {
            if (node.parentEdgeIndex < 0) continue;
            const parent = this.edges[node.parentEdgeIndex];
            const parentRadiusMm = parent?.rawRadiusEnd || 0;
            if (parentRadiusMm < AORTIC_DUPLICATE_MIN_PARENT_RADIUS_MM) {
                continue;
            }
            for (const connectorEdgeIndex of node.childEdgeIndices) {
                const connector = this.edges[connectorEdgeIndex];
                if (
                    !connector ||
                    connector.length > AORTIC_CONNECTOR_MAX_LENGTH_MM ||
                    connector.rawRadiusStart <
                        parentRadiusMm *
                            AORTIC_CONNECTOR_MIN_RADIUS_RATIO ||
                    connector.rawRadiusEnd <
                        parentRadiusMm *
                            AORTIC_CONNECTOR_MIN_RADIUS_RATIO ||
                    connector.childEdgeIndices.length < 2 ||
                    !connector.childEdgeIndices.every(childIndex =>
                        reachesBranchCaliber(
                            childIndex,
                            parentRadiusMm
                        )
                    )
                ) continue;
                candidates.push({
                    parentNode: node,
                    parentEdgeIndex: parent.index,
                    connectorEdgeIndex: connector.index,
                    branchRootEdgeIndices: [
                        ...connector.childEdgeIndices
                    ],
                    lengthMm: connector.length,
                    volumeMm3: connector.totalVolume,
                    radiusMm:
                        (connector.rawRadiusStart +
                            connector.rawRadiusEnd) * 0.5
                });
            }
        }

        for (const candidate of candidates) {
            const connector = this.edges[candidate.connectorEdgeIndex];
            connector.intraluminalAorticConnector = true;
            connector.renderExcluded = true;
            connector.lumenMappingExcluded = true;
        }
        this.intraluminalAorticConnectorDiagnostics = {
            contractedConnectorCount: candidates.length,
            excludedVolumeMm3: 0,
            connectors: candidates.map(candidate => ({
                parentEdgeIndex: candidate.parentEdgeIndex,
                connectorEdgeIndex: candidate.connectorEdgeIndex,
                branchRootEdgeIndices:
                    candidate.branchRootEdgeIndices,
                lengthMm: candidate.lengthMm,
                volumeMm3: candidate.volumeMm3,
                radiusMm: candidate.radiusMm
            }))
        };
    }

    /**
     * Give the retained topological bridge only the volume needed to carry
     * the combined branch flow. Its source radius is sampled inside the
     * aortic wall and is therefore not an anatomical lumen radius. Branch
     * prefixes are corrected first so their inlet calibers can be used here.
     */
    _regularizeIntraluminalAorticConnectorVolumes() {
        const diagnostics =
            this.intraluminalAorticConnectorDiagnostics;
        if (!diagnostics?.connectors?.length) return;

        let removedVolumeMm3 = 0;
        for (const candidate of diagnostics.connectors) {
            const connector = this.edges[candidate.connectorEdgeIndex];
            const branchRadiiMm = candidate.branchRootEdgeIndices
                .map(edgeIndex => this.edges[edgeIndex])
                .filter(Boolean)
                .map(edge => Math.max(
                    MIN_RADIUS_MM,
                    Math.min(edge.radiusStart, edge.radiusEnd)
                ));
            if (!branchRadiiMm.length) continue;

            const originalVolumeMm3 = connector.totalVolume;
            const transportRadiusMm = Math.max(...branchRadiiMm);
            connector.radiusStart = Math.min(
                connector.radiusStart,
                transportRadiusMm
            );
            connector.radiusEnd = Math.min(
                connector.radiusEnd,
                transportRadiusMm
            );
            this._rebuildEdgeTransportGeometry(connector);
            const removed = Math.max(
                0,
                originalVolumeMm3 - connector.totalVolume
            );
            removedVolumeMm3 += removed;
            candidate.transportRadiusMm = transportRadiusMm;
            candidate.transportVolumeMm3 = connector.totalVolume;
            candidate.removedVolumeMm3 = removed;
        }
        diagnostics.excludedVolumeMm3 = removedVolumeMm3;
    }

    /**
     * A medial-axis extractor can emit two nearly parallel centreline strands
     * inside one wide, curved aortic lumen. One strand continues into the
     * systemic tree while the other stays aorta-calibre, converges back onto
     * the main strand and terminates without an anatomical outlet. Treating
     * that strand as a vessel diverts cardiac output into a false dead end and
     * produces a stationary pool followed by a sharp bolus discontinuity.
     *
     * Keep source edge indices stable, but remove a confidently identified
     * duplicate subtree from transport, lumen mapping and rendering.
     */
    _suppressIntraluminalAorticDeadEnds() {
        const summarizeSubtree = rootEdgeIndex => {
            const edgeIndices = [];
            const outletEdgeIndices = [];
            const queue = [rootEdgeIndex];
            let queueIndex = 0;
            let totalLengthMm = 0;
            let minimumRawRadiusMm = Infinity;
            let linear = true;
            while (queueIndex < queue.length) {
                const edge = this.edges[queue[queueIndex++]];
                if (!edge) continue;
                edgeIndices.push(edge.index);
                totalLengthMm += edge.length;
                minimumRawRadiusMm = Math.min(
                    minimumRawRadiusMm,
                    edge.rawRadiusStart,
                    edge.rawRadiusEnd
                );
                linear &&= edge.childEdgeIndices.length <= 1;
                if (!edge.childEdgeIndices.length) {
                    outletEdgeIndices.push(edge.index);
                } else {
                    queue.push(...edge.childEdgeIndices);
                }
            }
            return {
                rootEdgeIndex,
                edgeIndices,
                outletEdgeIndices,
                outletCount: outletEdgeIndices.length,
                totalLengthMm,
                minimumRawRadiusMm,
                linear
            };
        };
        const distanceToEdge = (point, edge) => {
            const relative = point.clone().sub(edge.start);
            const axial = THREE.MathUtils.clamp(
                relative.dot(edge.axis),
                0,
                edge.length
            );
            return point.distanceTo(
                edge.start.clone().addScaledVector(edge.axis, axial)
            );
        };

        const suppressedRoots = [];
        const suppressedEdgeIndices = new Set();
        const junctions = [...this.nodes.values()];
        for (const node of junctions) {
            if (
                node.parentEdgeIndex < 0 ||
                node.childEdgeIndices.length < 2
            ) continue;
            const parent = this.edges[node.parentEdgeIndex];
            const parentRadiusMm = parent?.rawRadiusEnd || 0;
            if (parentRadiusMm < AORTIC_DUPLICATE_MIN_PARENT_RADIUS_MM) {
                continue;
            }

            const summaries = node.childEdgeIndices.map(summarizeSubtree);
            for (const candidate of summaries) {
                const root = this.edges[candidate.rootEdgeIndex];
                if (
                    !candidate.linear ||
                    candidate.outletCount !== 1 ||
                    candidate.totalLengthMm >
                        AORTIC_DUPLICATE_MAX_LENGTH_MM ||
                    candidate.minimumRawRadiusMm <
                        parentRadiusMm * AORTIC_DUPLICATE_MIN_RADIUS_RATIO
                ) continue;

                const main = summaries
                    .filter(summary =>
                        summary.rootEdgeIndex !==
                            candidate.rootEdgeIndex &&
                        summary.outletCount >=
                            AORTIC_DUPLICATE_MIN_MAIN_OUTLETS
                    )
                    .sort((a, b) => b.outletCount - a.outletCount)[0];
                if (!main) continue;
                const mainRoot = this.edges[main.rootEdgeIndex];
                if (
                    root.axis.dot(mainRoot.axis) <
                        AORTIC_DUPLICATE_MIN_DIRECTION_DOT
                ) continue;

                const terminalEdge = this.edges[
                    candidate.outletEdgeIndices[0]
                ];
                let nearestMainDistanceMm = Infinity;
                for (const mainEdgeIndex of main.edgeIndices) {
                    nearestMainDistanceMm = Math.min(
                        nearestMainDistanceMm,
                        distanceToEdge(
                            terminalEdge.end,
                            this.edges[mainEdgeIndex]
                        )
                    );
                }
                if (
                    nearestMainDistanceMm >
                        parentRadiusMm *
                            AORTIC_DUPLICATE_MAX_REJOIN_DISTANCE_RATIO
                ) continue;

                suppressedRoots.push({
                    rootEdgeIndex: root.index,
                    mainContinuationEdgeIndex: mainRoot.index,
                    parentEdgeIndex: parent.index,
                    terminalEdgeIndex: terminalEdge.index,
                    edgeCount: candidate.edgeIndices.length,
                    lengthMm: candidate.totalLengthMm,
                    terminalRadiusMm:
                        terminalEdge.rawRadiusEnd,
                    nearestMainDistanceMm
                });
                for (const edgeIndex of candidate.edgeIndices) {
                    suppressedEdgeIndices.add(edgeIndex);
                }
            }
        }

        for (const entry of suppressedRoots) {
            const root = this.edges[entry.rootEdgeIndex];
            const junction = this.nodes.get(root.startNodeId);
            const childOffset = junction?.childEdgeIndices.indexOf(
                root.index
            ) ?? -1;
            if (childOffset >= 0) {
                junction.childEdgeIndices.splice(childOffset, 1);
            }
        }
        for (const edgeIndex of suppressedEdgeIndices) {
            this.edges[edgeIndex].transportExcluded = true;
        }
        this.intraluminalAorticArtifactDiagnostics = {
            suppressedRootCount: suppressedRoots.length,
            suppressedEdgeCount: suppressedEdgeIndices.size,
            roots: suppressedRoots
        };
    }

    _regularizeShortRadiusValleys() {
        const rawRadiusStart = Float64Array.from(
            this.edges,
            edge => edge.radiusStart
        );
        const rawRadiusEnd = Float64Array.from(
            this.edges,
            edge => edge.radiusEnd
        );
        const restoredEdgeRadius = new Float64Array(this.edges.length);
        const edgePeakRadius = edgeIndex => Math.max(
            rawRadiusStart[edgeIndex],
            rawRadiusEnd[edgeIndex]
        );

        let candidateEdgeCount = 0;
        for (const edge of this.edges) {
            const currentRadius =
                (rawRadiusStart[edge.index] +
                    rawRadiusEnd[edge.index]) * 0.5;
            let upstreamPeak = currentRadius;
            let upstreamDistance = 0;
            let upstreamEdge = edge;
            while (
                upstreamEdge.parentEdgeIndex >= 0 &&
                upstreamDistance <= RADIUS_VALLEY_WINDOW_MM
            ) {
                upstreamEdge =
                    this.edges[upstreamEdge.parentEdgeIndex];
                upstreamDistance += upstreamEdge.length;
                upstreamPeak = Math.max(
                    upstreamPeak,
                    edgePeakRadius(upstreamEdge.index)
                );
            }

            let downstreamPeak = currentRadius;
            let maximumDownstreamDistance = 0;
            const queue = edge.childEdgeIndices.map(
                childIndex => ({
                    edgeIndex: childIndex,
                    distance: this.edges[childIndex].length
                })
            );
            let queueIndex = 0;
            while (queueIndex < queue.length) {
                const entry = queue[queueIndex++];
                if (entry.distance > RADIUS_VALLEY_WINDOW_MM) continue;
                const downstreamEdge = this.edges[entry.edgeIndex];
                maximumDownstreamDistance = Math.max(
                    maximumDownstreamDistance,
                    entry.distance
                );
                downstreamPeak = Math.max(
                    downstreamPeak,
                    edgePeakRadius(downstreamEdge.index)
                );
                for (const childIndex of downstreamEdge.childEdgeIndices) {
                    queue.push({
                        edgeIndex: childIndex,
                        distance:
                            entry.distance +
                            this.edges[childIndex].length
                    });
                }
            }

            const supportedRadius = Math.min(
                upstreamPeak,
                downstreamPeak
            );
            if (
                upstreamDistance >= RADIUS_VALLEY_SUPPORT_MM &&
                maximumDownstreamDistance >= RADIUS_VALLEY_SUPPORT_MM &&
                currentRadius < supportedRadius * RADIUS_VALLEY_RATIO
            ) {
                restoredEdgeRadius[edge.index] =
                    supportedRadius * RADIUS_VALLEY_RESTORE_RATIO;
                candidateEdgeCount++;
            }
        }

        const nodeRadius = new Map();
        const applyNodeRadius = (nodeId, radius) => {
            nodeRadius.set(
                nodeId,
                Math.max(nodeRadius.get(nodeId) || 0, radius)
            );
        };
        for (const edge of this.edges) {
            const restoredRadius = restoredEdgeRadius[edge.index];
            applyNodeRadius(
                edge.startNodeId,
                Math.max(rawRadiusStart[edge.index], restoredRadius)
            );
            applyNodeRadius(
                edge.endNodeId,
                Math.max(rawRadiusEnd[edge.index], restoredRadius)
            );
        }

        let correctedEdgeCount = 0;
        let minimumRawRadiusMm = Infinity;
        let minimumCorrectedRadiusMm = Infinity;
        for (const edge of this.edges) {
            const originalStart = edge.radiusStart;
            const originalEnd = edge.radiusEnd;
            edge.radiusStart = Math.max(
                MIN_RADIUS_MM,
                nodeRadius.get(edge.startNodeId) || originalStart
            );
            edge.radiusEnd = Math.max(
                MIN_RADIUS_MM,
                nodeRadius.get(edge.endNodeId) || originalEnd
            );
            minimumRawRadiusMm = Math.min(
                minimumRawRadiusMm,
                originalStart,
                originalEnd
            );
            minimumCorrectedRadiusMm = Math.min(
                minimumCorrectedRadiusMm,
                edge.radiusStart,
                edge.radiusEnd
            );
            if (
                edge.radiusStart > originalStart + 1e-9 ||
                edge.radiusEnd > originalEnd + 1e-9
            ) {
                correctedEdgeCount++;
            }

            edge.totalVolume = 0;
            for (
                let cellIndex = 0;
                cellIndex < edge.cellCount;
                cellIndex++
            ) {
                const t0 = cellIndex / edge.cellCount;
                const t1 = (cellIndex + 1) / edge.cellCount;
                const radius0 = THREE.MathUtils.lerp(
                    edge.radiusStart,
                    edge.radiusEnd,
                    t0
                );
                const radius1 = THREE.MathUtils.lerp(
                    edge.radiusStart,
                    edge.radiusEnd,
                    t1
                );
                const volume = frustumVolume(
                    edge.cellLength,
                    radius0,
                    radius1
                );
                edge.volumes[cellIndex] = volume;
                edge.areas[cellIndex] =
                    Math.PI * ((radius0 + radius1) * 0.5) ** 2;
                edge.totalVolume += volume;
            }
            const effectiveRadius = Math.max(
                MIN_RADIUS_MM,
                (edge.radiusStart + edge.radiusEnd) * 0.5
            );
            edge.resistance =
                8 * this.hemodynamics.bloodViscosityPaS * edge.length /
                (Math.PI * effectiveRadius ** 4);
        }

        this.radiusRegularizationDiagnostics = {
            candidateEdgeCount,
            correctedEdgeCount,
            minimumRawRadiusMm,
            minimumCorrectedRadiusMm
        };
    }

    _rebuildEdgeTransportGeometry(edge) {
        edge.totalVolume = 0;
        for (
            let cellIndex = 0;
            cellIndex < edge.cellCount;
            cellIndex++
        ) {
            const t0 = cellIndex / edge.cellCount;
            const t1 = (cellIndex + 1) / edge.cellCount;
            const radius0 = THREE.MathUtils.lerp(
                edge.radiusStart,
                edge.radiusEnd,
                t0
            );
            const radius1 = THREE.MathUtils.lerp(
                edge.radiusStart,
                edge.radiusEnd,
                t1
            );
            const volume = frustumVolume(
                edge.cellLength,
                radius0,
                radius1
            );
            edge.volumes[cellIndex] = volume;
            edge.areas[cellIndex] =
                Math.PI * ((radius0 + radius1) * 0.5) ** 2;
            edge.totalVolume += volume;
        }
        const effectiveRadius = Math.max(
            MIN_RADIUS_MM,
            (edge.radiusStart + edge.radiusEnd) * 0.5
        );
        edge.resistance =
            8 * this.hemodynamics.bloodViscosityPaS * edge.length /
            (Math.PI * effectiveRadius ** 4);
    }

    /**
     * The packed centerline begins some side branches while their paths are
     * still inside the aortic lumen. Those technical prefixes inherit the
     * aortic radius and would otherwise become large 1D reservoirs: contrast
     * reaches the real branch late and remains as an ostial plug. Preserve the
     * raw radii for diagnostics, but use the downstream branch caliber for
     * transport, rendering and catheter mapping along each unbranched prefix.
     */
    _regularizeAorticBranchPrefixes() {
        const correctedEdges = new Set();
        const correctedPaths = [];
        let removedVolumeMm3 = 0;

        for (const node of this.nodes.values()) {
            if (
                node.parentEdgeIndex < 0 ||
                node.childEdgeIndices.length < 2
            ) continue;
            const parent = this.edges[node.parentEdgeIndex];
            const parentRadius = parent?.rawRadiusEnd || 0;
            if (parentRadius < AORTIC_PREFIX_MIN_PARENT_RADIUS_MM) {
                continue;
            }

            for (const rootEdgeIndex of node.childEdgeIndices) {
                const rootEdge = this.edges[rootEdgeIndex];
                if (
                    !rootEdge ||
                    rootEdge.rawRadiusStart <
                        parentRadius * AORTIC_PREFIX_START_RADIUS_RATIO
                ) continue;

                const path = [];
                let edge = rootEdge;
                let pathLengthMm = 0;
                let branchRadiusMm = 0;
                while (
                    edge &&
                    pathLengthMm < AORTIC_PREFIX_MAX_LENGTH_MM
                ) {
                    path.push(edge);
                    pathLengthMm += edge.length;
                    if (
                        edge.rawRadiusEnd <=
                            parentRadius * AORTIC_PREFIX_EXIT_RADIUS_RATIO
                    ) {
                        branchRadiusMm = Math.max(
                            MIN_RADIUS_MM,
                            edge.rawRadiusEnd
                        );
                        break;
                    }
                    if (edge.childEdgeIndices.length !== 1) break;
                    edge = this.edges[edge.childEdgeIndices[0]];
                }
                if (!(branchRadiusMm > 0)) continue;

                let probeDistanceMm = 0;
                while (
                    edge.childEdgeIndices.length === 1 &&
                    probeDistanceMm < AORTIC_PREFIX_CALIBER_PROBE_MM
                ) {
                    edge = this.edges[edge.childEdgeIndices[0]];
                    if (!edge) break;
                    path.push(edge);
                    pathLengthMm += edge.length;
                    probeDistanceMm += edge.length;
                    branchRadiusMm = Math.min(
                        branchRadiusMm,
                        Math.max(MIN_RADIUS_MM, edge.rawRadiusEnd)
                    );
                }
                if (
                    branchRadiusMm >
                        AORTIC_PREFIX_MAX_BRANCH_RADIUS_MM
                ) continue;

                const ostialRadiusMm = Math.min(
                    parentRadius,
                    branchRadiusMm * AORTIC_PREFIX_OSTIAL_FLARE_RATIO
                );
                let distanceMm = 0;
                let pathRemovedVolumeMm3 = 0;
                for (const prefixEdge of path) {
                    const startT = distanceMm / pathLengthMm;
                    const endT =
                        (distanceMm + prefixEdge.length) / pathLengthMm;
                    const desiredStartRadius = THREE.MathUtils.lerp(
                        ostialRadiusMm,
                        branchRadiusMm,
                        startT
                    );
                    const desiredEndRadius = THREE.MathUtils.lerp(
                        ostialRadiusMm,
                        branchRadiusMm,
                        endT
                    );
                    const originalVolume = prefixEdge.totalVolume;
                    prefixEdge.radiusStart = Math.min(
                        prefixEdge.radiusStart,
                        desiredStartRadius
                    );
                    prefixEdge.radiusEnd = Math.min(
                        prefixEdge.radiusEnd,
                        desiredEndRadius
                    );
                    prefixEdge.lumenMappingUsesCorrectedRadius = true;
                    this._rebuildEdgeTransportGeometry(prefixEdge);
                    const removed = Math.max(
                        0,
                        originalVolume - prefixEdge.totalVolume
                    );
                    pathRemovedVolumeMm3 += removed;
                    removedVolumeMm3 += removed;
                    correctedEdges.add(prefixEdge.index);
                    distanceMm += prefixEdge.length;
                }
                correctedPaths.push({
                    rootEdgeIndex,
                    edgeIndices: path.map(prefixEdge => prefixEdge.index),
                    lengthMm: pathLengthMm,
                    parentRadiusMm: parentRadius,
                    branchRadiusMm,
                    removedVolumeMm3: pathRemovedVolumeMm3
                });
            }
        }

        this.aorticBranchPrefixDiagnostics = {
            correctedPathCount: correctedPaths.length,
            correctedEdgeCount: correctedEdges.size,
            removedVolumeMm3,
            paths: correctedPaths
        };
    }

    _computeHydraulicDistribution() {
        for (let orderIndex = this.nodeOrder.length - 1; orderIndex >= 0; orderIndex--) {
            const node = this.nodes.get(this.nodeOrder[orderIndex]);
            if (!node.childEdgeIndices.length) {
                node.perfusionWeight = Math.max(
                    this.minimumTerminalPerfusionRadiusMm,
                    node.terminalRadius
                ) ** this.terminalFlowRadiusExponent;
                node.equivalentResistance =
                    this.terminalResistanceLengthMm / Math.max(MIN_RADIUS_MM, node.terminalRadius) ** 4;
                node.terminalOutletCount = 1;
                continue;
            }
            let conductance = 0;
            let perfusionWeight = 0;
            let terminalOutletCount = 0;
            for (const edgeIndex of node.childEdgeIndices) {
                const edge = this.edges[edgeIndex];
                const child = this.nodes.get(edge.endNodeId);
                edge.equivalentResistance = edge.resistance + child.equivalentResistance;
                conductance += 1 / Math.max(1e-12, edge.equivalentResistance);
                perfusionWeight += child.perfusionWeight;
                terminalOutletCount += child.terminalOutletCount;
            }
            node.perfusionWeight = perfusionWeight;
            node.terminalOutletCount = terminalOutletCount;
            node.equivalentResistance = conductance > 0 ? 1 / conductance : Infinity;
        }

        const inletFlowMm3PerS =
            this.hemodynamics.cardiacOutputMlPerMin * MM3_PER_ML / SECONDS_PER_MINUTE;
        const queue = [{ node: this.rootNode, flow: inletFlowMm3PerS }];
        let queueIndex = 0;
        while (queueIndex < queue.length) {
            const { node, flow } = queue[queueIndex++];
            if (!node.childEdgeIndices.length) continue;
            let totalPerfusionWeight = 0;
            let totalCaliberWeight = 0;
            for (const edgeIndex of node.childEdgeIndices) {
                const edge = this.edges[edgeIndex];
                const child = this.nodes.get(edge.endNodeId);
                totalPerfusionWeight += child.perfusionWeight;
                edge.flowSplitRadiusMm =
                    this._sampleBranchCaliberMm(edgeIndex);
                edge.flowSplitCaliberWeight =
                    edge.flowSplitRadiusMm ** this.localCaliberFlowExponent;
                totalCaliberWeight += edge.flowSplitCaliberWeight;
            }
            let totalRawSplitShare = 0;
            for (const edgeIndex of node.childEdgeIndices) {
                const edge = this.edges[edgeIndex];
                const child = this.nodes.get(edge.endNodeId);
                const equalShare = 1 / node.childEdgeIndices.length;
                const perfusionShare = totalPerfusionWeight > 0
                    ? child.perfusionWeight / totalPerfusionWeight
                    : equalShare;
                const caliberShare = totalCaliberWeight > 0
                    ? edge.flowSplitCaliberWeight / totalCaliberWeight
                    : equalShare;
                const isUnderresolvedBranch =
                    child.terminalOutletCount <=
                        this.maximumCaliberCorrectedTerminalOutlets;
                edge.rawFlowSplitShare = isUnderresolvedBranch
                    ? perfusionShare * (1 - this.localCaliberFlowBlend) +
                        caliberShare * this.localCaliberFlowBlend
                    : perfusionShare;
                totalRawSplitShare += edge.rawFlowSplitShare;
            }
            for (const edgeIndex of node.childEdgeIndices) {
                const edge = this.edges[edgeIndex];
                const child = this.nodes.get(edge.endNodeId);
                edge.flowSplitShare =
                    edge.rawFlowSplitShare /
                    Math.max(1e-12, totalRawSplitShare);
                edge.meanFlowMm3PerS = flow * edge.flowSplitShare;
                queue.push({
                    node: child,
                    flow: edge.meanFlowMm3PerS
                });
            }
        }
    }

    _sampleBranchCaliberMm(edgeIndex) {
        let edge = this.edges[edgeIndex];
        if (!edge) return MIN_RADIUS_MM;
        let remainingDistance = this.branchCaliberSampleDistanceMm;
        let sampledRadius = edge.radiusStart;
        const visited = new Set();

        while (edge && !visited.has(edge.index)) {
            visited.add(edge.index);
            if (remainingDistance <= edge.length) {
                const t = edge.length > 1e-9
                    ? remainingDistance / edge.length
                    : 1;
                return Math.max(
                    MIN_RADIUS_MM,
                    THREE.MathUtils.lerp(
                        edge.radiusStart,
                        edge.radiusEnd,
                        THREE.MathUtils.clamp(t, 0, 1)
                    )
                );
            }
            remainingDistance -= edge.length;
            sampledRadius = edge.radiusEnd;
            if (!edge.childEdgeIndices.length) break;
            edge = edge.childEdgeIndices.reduce((widest, childIndex) => {
                const child = this.edges[childIndex];
                if (!widest) return child;
                const childRadius =
                    (child.radiusStart + child.radiusEnd) * 0.5;
                const widestRadius =
                    (widest.radiusStart + widest.radiusEnd) * 0.5;
                return childRadius > widestRadius ? child : widest;
            }, null);
        }
        return Math.max(MIN_RADIUS_MM, sampledRadius);
    }

    _buildSpatialIndex() {
        const cellSize = this.spatialCellMm;
        for (const edge of this.edges) {
            if (edge.transportExcluded || edge.lumenMappingExcluded) continue;
            const padding = Math.max(edge.radiusStart, edge.radiusEnd, edge.safeRadius, 1);
            const minX = Math.floor((Math.min(edge.start.x, edge.end.x) - padding) / cellSize);
            const minY = Math.floor((Math.min(edge.start.y, edge.end.y) - padding) / cellSize);
            const minZ = Math.floor((Math.min(edge.start.z, edge.end.z) - padding) / cellSize);
            const maxX = Math.floor((Math.max(edge.start.x, edge.end.x) + padding) / cellSize);
            const maxY = Math.floor((Math.max(edge.start.y, edge.end.y) + padding) / cellSize);
            const maxZ = Math.floor((Math.max(edge.start.z, edge.end.z) + padding) / cellSize);
            for (let x = minX; x <= maxX; x++) {
                for (let y = minY; y <= maxY; y++) {
                    for (let z = minZ; z <= maxZ; z++) {
                        const key = cellKey(x, y, z);
                        let bucket = this._spatialIndex.get(key);
                        if (!bucket) {
                            bucket = [];
                            this._spatialIndex.set(key, bucket);
                        }
                        bucket.push(edge.index);
                    }
                }
            }
        }
    }

    _nextQueryEpoch() {
        this._queryEpoch = (this._queryEpoch + 1) >>> 0;
        if (this._queryEpoch === 0) {
            this._queryMarks.fill(0);
            this._queryEpoch = 1;
        }
        return this._queryEpoch;
    }

    findNearestLocation(point, out = {}) {
        return this.findNearestLocationCoordinates(point.x, point.y, point.z, out);
    }

    findInjectionLocation(point, direction = null, out = {}) {
        const x = point.x;
        const y = point.y;
        const z = point.z;
        const directionLength = Math.hypot(
            direction?.x || 0,
            direction?.y || 0,
            direction?.z || 0
        );
        const directionX = directionLength > 1e-9
            ? direction.x / directionLength
            : 0;
        const directionY = directionLength > 1e-9
            ? direction.y / directionLength
            : 0;
        const directionZ = directionLength > 1e-9
            ? direction.z / directionLength
            : 0;
        const cellSize = this.spatialCellMm;
        const cellX = Math.floor(x / cellSize);
        const cellY = Math.floor(y / cellSize);
        const cellZ = Math.floor(z / cellSize);
        const epoch = this._nextQueryEpoch();
        let bestContainingScore = -Infinity;
        let bestContainingEdge = null;
        let bestContainingT = 0;
        let bestContainingDistanceSq = Infinity;
        let nearestDistanceSq = Infinity;
        let nearestEdge = null;
        let nearestT = 0;

        const evaluateBucket = bucket => {
            if (!bucket) return;
            for (const edgeIndex of bucket) {
                if (this._queryMarks[edgeIndex] === epoch) continue;
                this._queryMarks[edgeIndex] = epoch;
                const edge = this.edges[edgeIndex];
                if (edge.transportExcluded || edge.lumenMappingExcluded) continue;
                const relX = x - edge.start.x;
                const relY = y - edge.start.y;
                const relZ = z - edge.start.z;
                const axial = THREE.MathUtils.clamp(
                    relX * edge.axis.x +
                        relY * edge.axis.y +
                        relZ * edge.axis.z,
                    0,
                    edge.length
                );
                const t = axial / edge.length;
                const centerX = edge.start.x + edge.axis.x * axial;
                const centerY = edge.start.y + edge.axis.y * axial;
                const centerZ = edge.start.z + edge.axis.z * axial;
                const distanceSq =
                    (x - centerX) ** 2 +
                    (y - centerY) ** 2 +
                    (z - centerZ) ** 2;
                if (distanceSq < nearestDistanceSq) {
                    nearestDistanceSq = distanceSq;
                    nearestEdge = edge;
                    nearestT = t;
                }

                // Catheter mapping must use the corrected transport lumen.
                // Raw centreline radii can still carry the surrounding
                // aortic calibre for several centimetres into a branch. In
                // overlapping arch geometry that made a port physically in
                // the aorta map to the technical supra-aortic prefix instead.
                const selectionRadius = THREE.MathUtils.lerp(
                    edge.lumenMappingUsesCorrectedRadius
                        ? edge.radiusStart
                        : edge.rawRadiusStart,
                    edge.lumenMappingUsesCorrectedRadius
                        ? edge.radiusEnd
                        : edge.rawRadiusEnd,
                    t
                );
                const distance = Math.sqrt(distanceSq);
                if (distance > selectionRadius * 1.05) continue;
                const axisAlignment = directionLength > 1e-9
                    ? Math.abs(
                        edge.axis.x * directionX +
                        edge.axis.y * directionY +
                        edge.axis.z * directionZ
                    )
                    : 0;
                const wallClearance = selectionRadius - distance;
                const containingScore =
                    wallClearance +
                    selectionRadius * 0.5 +
                    axisAlignment * selectionRadius * 0.15;
                if (containingScore <= bestContainingScore) continue;
                bestContainingScore = containingScore;
                bestContainingEdge = edge;
                bestContainingT = t;
                bestContainingDistanceSq = distanceSq;
            }
        };

        for (let dx = -2; dx <= 2; dx++) {
            for (let dy = -2; dy <= 2; dy++) {
                for (let dz = -2; dz <= 2; dz++) {
                    evaluateBucket(this._spatialIndex.get(
                        cellKey(cellX + dx, cellY + dy, cellZ + dz)
                    ));
                }
            }
        }
        if (!nearestEdge) {
            for (const edge of this.edges) evaluateBucket([edge.index]);
        }

        const bestEdge = bestContainingEdge || nearestEdge;
        const bestT = bestContainingEdge
            ? bestContainingT
            : nearestT;
        const bestDistanceSq = bestContainingEdge
            ? bestContainingDistanceSq
            : nearestDistanceSq;
        if (!bestEdge) {
            out.edgeIndex = -1;
            out.distance = Infinity;
            return out;
        }

        out.edgeIndex = bestEdge.index;
        out.sourceIndex = bestEdge.sourceIndex;
        out.t = bestT;
        out.cellIndex = Math.min(
            bestEdge.cellCount - 1,
            Math.max(0, Math.floor(bestT * bestEdge.cellCount))
        );
        out.distance = Math.sqrt(bestDistanceSq);
        out.radius = THREE.MathUtils.lerp(
            bestEdge.radiusStart,
            bestEdge.radiusEnd,
            bestT
        );
        out.centerX = THREE.MathUtils.lerp(
            bestEdge.start.x,
            bestEdge.end.x,
            bestT
        );
        out.centerY = THREE.MathUtils.lerp(
            bestEdge.start.y,
            bestEdge.end.y,
            bestT
        );
        out.centerZ = THREE.MathUtils.lerp(
            bestEdge.start.z,
            bestEdge.end.z,
            bestT
        );
        out.tangentX = bestEdge.axis.x;
        out.tangentY = bestEdge.axis.y;
        out.tangentZ = bestEdge.axis.z;
        out.selectionMode = bestContainingEdge
            ? 'containing-lumen'
            : 'nearest-centerline';
        return out;
    }

    findUpstreamMixingJunction(location, {
        minimumParentRadiusMm = 6,
        minimumChildRadiusMm = 4,
        maximumDistanceMm = Infinity,
        preferFarthest = false
    } = {}) {
        let edge = this.edges[location?.edgeIndex];
        if (!edge) return null;
        let distanceMm = THREE.MathUtils.clamp(location.t ?? 0, 0, 1) * edge.length;
        const pathEdgeIndices = [];
        let bestTarget = null;

        while (edge) {
            if (distanceMm > maximumDistanceMm) break;
            pathEdgeIndices.push(edge.index);
            const junction = this.nodes.get(edge.startNodeId);
            const parentEdge = junction?.parentEdgeIndex >= 0
                ? this.edges[junction.parentEdgeIndex]
                : null;
            if (junction && parentEdge && junction.childEdgeIndices.length >= 2) {
                const majorChildren = junction.childEdgeIndices.filter(edgeIndex => {
                    const child = this.edges[edgeIndex];
                    return child &&
                        Math.max(child.radiusStart, child.radiusEnd) >=
                            minimumChildRadiusMm;
                });
                if (
                    parentEdge.radiusEnd >=
                        minimumParentRadiusMm &&
                    majorChildren.length >= 2
                ) {
                    bestTarget = {
                        edgeIndex: parentEdge.index,
                        cellIndex: parentEdge.cellCount - 1,
                        junctionNodeId: junction.id,
                        distanceMm,
                        sourceChildEdgeIndex: edge.index,
                        childEdgeIndices: [...majorChildren],
                        pathEdgeIndices: [...pathEdgeIndices]
                    };
                    if (!preferFarthest) return bestTarget;
                }
            }
            if (!parentEdge) break;
            distanceMm += parentEdge.length;
            edge = parentEdge;
        }
        return bestTarget;
    }

    findNearestLocationCoordinates(x, y, z, out = {}) {
        const cellSize = this.spatialCellMm;
        const cellX = Math.floor(x / cellSize);
        const cellY = Math.floor(y / cellSize);
        const cellZ = Math.floor(z / cellSize);
        const epoch = this._nextQueryEpoch();
        let bestDistanceSq = Infinity;
        let bestEdge = null;
        let bestT = 0;
        const evaluateBucket = bucket => {
            if (!bucket) return;
            for (const edgeIndex of bucket) {
                if (this._queryMarks[edgeIndex] === epoch) continue;
                this._queryMarks[edgeIndex] = epoch;
                const edge = this.edges[edgeIndex];
                if (edge.transportExcluded || edge.lumenMappingExcluded) continue;
                const relX = x - edge.start.x;
                const relY = y - edge.start.y;
                const relZ = z - edge.start.z;
                const axial = THREE.MathUtils.clamp(
                    relX * edge.axis.x + relY * edge.axis.y + relZ * edge.axis.z,
                    0,
                    edge.length
                );
                const centerX = edge.start.x + edge.axis.x * axial;
                const centerY = edge.start.y + edge.axis.y * axial;
                const centerZ = edge.start.z + edge.axis.z * axial;
                const distanceSq =
                    (x - centerX) ** 2 +
                    (y - centerY) ** 2 +
                    (z - centerZ) ** 2;
                if (distanceSq < bestDistanceSq) {
                    bestDistanceSq = distanceSq;
                    bestEdge = edge;
                    bestT = axial / edge.length;
                }
            }
        };

        for (let searchRadius = 0; searchRadius <= 2 && !bestEdge; searchRadius++) {
            for (let dx = -searchRadius; dx <= searchRadius; dx++) {
                for (let dy = -searchRadius; dy <= searchRadius; dy++) {
                    for (let dz = -searchRadius; dz <= searchRadius; dz++) {
                        if (
                            searchRadius > 0 &&
                            Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) !== searchRadius
                        ) continue;
                        evaluateBucket(this._spatialIndex.get(
                            cellKey(cellX + dx, cellY + dy, cellZ + dz)
                        ));
                    }
                }
            }
        }
        if (!bestEdge) {
            for (const edge of this.edges) evaluateBucket([edge.index]);
        }
        if (!bestEdge) {
            out.edgeIndex = -1;
            out.distance = Infinity;
            return out;
        }

        const centerX = THREE.MathUtils.lerp(bestEdge.start.x, bestEdge.end.x, bestT);
        const centerY = THREE.MathUtils.lerp(bestEdge.start.y, bestEdge.end.y, bestT);
        const centerZ = THREE.MathUtils.lerp(bestEdge.start.z, bestEdge.end.z, bestT);
        out.edgeIndex = bestEdge.index;
        out.sourceIndex = bestEdge.sourceIndex;
        out.t = bestT;
        out.cellIndex = Math.min(
            bestEdge.cellCount - 1,
            Math.max(0, Math.floor(bestT * bestEdge.cellCount))
        );
        out.distance = Math.sqrt(bestDistanceSq);
        out.radius = THREE.MathUtils.lerp(bestEdge.radiusStart, bestEdge.radiusEnd, bestT);
        out.centerX = centerX;
        out.centerY = centerY;
        out.centerZ = centerZ;
        out.tangentX = bestEdge.axis.x;
        out.tangentY = bestEdge.axis.y;
        out.tangentZ = bestEdge.axis.z;
        return out;
    }

    sampleVelocityCoordinates(x, y, z, out = {}) {
        const location = this.findNearestLocationCoordinates(x, y, z, out);
        if (location.edgeIndex < 0) {
            out.velocityX = 0;
            out.velocityY = 0;
            out.velocityZ = 0;
            out.speed = 0;
            out.meanSpeed = 0;
            return out;
        }
        const edge = this.edges[location.edgeIndex];
        const radius = Math.max(MIN_RADIUS_MM, location.radius);
        const area = Math.PI * radius * radius;
        const radialFraction = THREE.MathUtils.clamp(location.distance / radius, 0, 1);
        const profile = THREE.MathUtils.clamp(
            2 * (1 - radialFraction * radialFraction),
            0.12,
            2
        );
        const waveform = arterialWaveform(
            this.time,
            this.hemodynamics.heartRateBpm
        );
        const leftFaceFlow = this.getFaceSignedFlowMm3PerS(
            edge.index,
            location.cellIndex,
            waveform
        );
        const rightFaceFlow = this.getFaceSignedFlowMm3PerS(
            edge.index,
            location.cellIndex + 1,
            waveform
        );
        const localFlow = (leftFaceFlow + rightFaceFlow) * 0.5;
        const speed = localFlow /
            Math.max(1e-6, area) * profile;
        out.meanSpeed = localFlow / Math.max(1e-6, area) * profile;
        out.speed = speed;
        out.velocityX = location.tangentX * speed;
        out.velocityY = location.tangentY * speed;
        out.velocityZ = location.tangentZ * speed;
        return out;
    }

    depositIodineAtPoint(point, iodineMassMg) {
        return this.depositIodineAtCoordinates(point.x, point.y, point.z, iodineMassMg);
    }

    depositIodineAtCoordinates(x, y, z, iodineMassMg) {
        if (!(iodineMassMg > 0)) return false;
        const location = this.findNearestLocationCoordinates(x, y, z, this._locationScratch);
        if (location.edgeIndex < 0) return false;
        this.edges[location.edgeIndex].massMg[location.cellIndex] += iodineMassMg;
        this.edges[location.edgeIndex].active = true;
        this._activeEdgeIndices.add(location.edgeIndex);
        this.totalIodineMassMg += iodineMassMg;
        return true;
    }

    depositIodine(edgeIndex, cellIndex, iodineMassMg) {
        const edge = this.edges[edgeIndex];
        if (!edge || !(iodineMassMg > 0)) return false;
        const index = THREE.MathUtils.clamp(Math.floor(cellIndex), 0, edge.cellCount - 1);
        edge.massMg[index] += iodineMassMg;
        edge.active = true;
        this._activeEdgeIndices.add(edge.index);
        this.totalIodineMassMg += iodineMassMg;
        return true;
    }

    update(dt) {
        if (!(dt > 0)) return;
        if (
            !(this.totalIodineMassMg > 1e-10) &&
            !this._branchInletEdgeIndices.size
        ) {
            this.time += dt;
            return;
        }
        const waveform = arterialWaveform(this.time, this.hemodynamics.heartRateBpm);
        let maximumActiveCourantRate = 0;
        const includeCourantRate = edge => {
            const override = this._flowOverridesMm3PerS[edge.index];
            const baseFlow = Number.isFinite(override)
                ? override
                : edge.meanFlowMm3PerS * waveform;
            const faceDeltas = edge.faceFlowDeltaMm3PerS;
            for (let index = 0; index < edge.cellCount; index++) {
                const leftFaceFlow = baseFlow + faceDeltas[index];
                const rightFaceFlow = baseFlow + faceDeltas[index + 1];
                // Only volume leaving a cell constrains its explicit upwind
                // step. At an injection cell both faces can point outward, so
                // their rates must be added instead of taking one edge-wide
                // absolute value.
                const outwardFlow =
                    Math.max(0, -leftFaceFlow) +
                    Math.max(0, rightFaceFlow);
                maximumActiveCourantRate = Math.max(
                    maximumActiveCourantRate,
                    outwardFlow /
                        Math.max(1e-6, edge.volumes[index])
                );
            }
        };
        for (const edgeIndex of this._activeEdgeIndices) {
            const edge = this.edges[edgeIndex];
            includeCourantRate(edge);
            for (const childIndex of edge.childEdgeIndices) {
                includeCourantRate(this.edges[childIndex]);
            }
        }
        for (const edgeIndex of this._branchInletEdgeIndices) {
            const edge = this.edges[edgeIndex];
            includeCourantRate(edge);
        }
        const advectiveSteps = Math.max(
            1,
            Math.ceil(
                maximumActiveCourantRate * dt /
                Math.max(1e-6, this.cflLimit)
            )
        );
        const dispersion = Math.max(0, this.hemodynamics.axialDispersionMm2PerS);
        const dispersiveSteps = dispersion > 0
            ? Math.max(1, Math.ceil(
                dispersion * dt /
                Math.max(1e-6, this.minimumCellLength ** 2 * 0.45)
            ))
            : 1;
        const substepCount = Math.min(
            MAX_TRANSPORT_SUBSTEPS,
            Math.max(advectiveSteps, dispersiveSteps)
        );
        this.lastTransportSubstepCount = substepCount;
        const substep = dt / substepCount;
        for (let step = 0; step < substepCount; step++) {
            this._transportSubstep(substep);
            this.time += substep;
        }
        this._updateEdgeConcentrations();
    }

    _transportSubstep(dt) {
        const waveform = arterialWaveform(this.time, this.hemodynamics.heartRateBpm);
        this._touchEpoch = (this._touchEpoch + 1) >>> 0;
        if (this._touchEpoch === 0) {
            this._touchMarks.fill(0);
            this._touchEpoch = 1;
        }
        const epoch = this._touchEpoch;
        const touched = this._touchedEdgeIndices;
        touched.length = 0;
        const markTouched = edgeIndex => {
            if (edgeIndex < 0 || this._touchMarks[edgeIndex] === epoch) return;
            this._touchMarks[edgeIndex] = epoch;
            touched.push(edgeIndex);
        };

        for (const edgeIndex of this._activeEdgeIndices) {
            const edge = this.edges[edgeIndex];
            markTouched(edgeIndex);
            for (const childIndex of edge.childEdgeIndices) markTouched(childIndex);
            const parentIndex = edge.parentEdgeIndex;
            if (parentIndex >= 0) {
                const parent = this.edges[parentIndex];
                markTouched(parentIndex);
                for (const siblingIndex of parent.childEdgeIndices) {
                    markTouched(siblingIndex);
                }
            }
        }
        for (const edgeIndex of this._branchInletEdgeIndices) {
            const edge = this.edges[edgeIndex];
            markTouched(edgeIndex);
            for (const childIndex of edge.childEdgeIndices) {
                markTouched(childIndex);
            }
        }

        for (const edgeIndex of touched) {
            const edge = this.edges[edgeIndex];
            this._edgeOutMassMg[edgeIndex] = 0;
            this._edgeUpstreamOutMassMg[edgeIndex] = 0;
            // Start from the complete previous stock so inflow and transport
            // remain conservative during this substep.
            edge.nextMassMg.set(edge.massMg);
        }

        for (const edgeIndex of this._activeEdgeIndices) {
            const edge = this.edges[edgeIndex];
            const source = edge.massMg;
            const next = edge.nextMassMg;
            const override = this._flowOverridesMm3PerS[edge.index];
            const baseFlow = Number.isFinite(override)
                ? override
                : edge.meanFlowMm3PerS * waveform;
            const faceDeltas = edge.faceFlowDeltaMm3PerS;
            for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
                const leftFaceFlow = baseFlow + faceDeltas[cellIndex];
                const rightFaceFlow = baseFlow + faceDeltas[cellIndex + 1];
                const leftOutflow = Math.max(0, -leftFaceFlow);
                const rightOutflow = Math.max(0, rightFaceFlow);
                const totalOutflow = leftOutflow + rightOutflow;
                if (!(totalOutflow > 0) || !(source[cellIndex] > 0)) {
                    continue;
                }
                const transportedFraction = Math.min(
                    1,
                    totalOutflow * dt /
                        Math.max(1e-9, edge.volumes[cellIndex])
                );
                const totalOutflowMass =
                    source[cellIndex] * transportedFraction;
                const leftMass = totalOutflowMass *
                    leftOutflow / totalOutflow;
                const rightMass = totalOutflowMass - leftMass;
                next[cellIndex] -= totalOutflowMass;
                if (leftMass > 0) {
                    if (cellIndex > 0) {
                        next[cellIndex - 1] += leftMass;
                    } else {
                        this._edgeUpstreamOutMassMg[edge.index] +=
                            leftMass;
                    }
                }
                if (rightMass > 0) {
                    if (cellIndex + 1 < edge.cellCount) {
                        next[cellIndex + 1] += rightMass;
                    } else {
                        this._edgeOutMassMg[edge.index] += rightMass;
                    }
                }
            }
            this._applyAxialDispersion(edge, dt);
        }
        for (const edgeIndex of this._branchInletEdgeIndices) {
            const inletMassMg =
                this._branchInletFlowMm3PerS[edgeIndex] *
                this._branchInletConcentrationMgPerMm3[edgeIndex] *
                dt;
            if (!(inletMassMg > 0)) continue;
            const edge = this.edges[edgeIndex];
            edge.nextMassMg[0] += inletMassMg;
        }
        this._applyJunctionDispersion(dt, touched);

        // Mix every signed inflow at its actual graph node, then distribute it
        // to every face whose flow leaves that node. This one rule handles
        // normal bifurcation, reflux from a child, simultaneous parent/child
        // inflow and smooth reversal through zero without anatomy-specific
        // gates.
        const nodeInflows = new Map();
        const addNodeInflow = (nodeId, edge, boundaryCellIndex, massMg) => {
            if (!(massMg > 0)) return;
            let inlet = nodeInflows.get(nodeId);
            if (!inlet) {
                inlet = { massMg: 0, origins: [] };
                nodeInflows.set(nodeId, inlet);
            }
            inlet.massMg += massMg;
            inlet.origins.push({ edge, boundaryCellIndex, massMg });
        };
        for (const edgeIndex of this._activeEdgeIndices) {
            const edge = this.edges[edgeIndex];
            const upstreamOutflowMass = this._edgeUpstreamOutMassMg[edgeIndex];
            if (upstreamOutflowMass > 0) {
                addNodeInflow(
                    edge.startNodeId,
                    edge,
                    0,
                    upstreamOutflowMass
                );
            }
            const outflowMass = this._edgeOutMassMg[edgeIndex];
            if (outflowMass > 0) {
                addNodeInflow(
                    edge.endNodeId,
                    edge,
                    edge.cellCount - 1,
                    outflowMass
                );
            }
        }

        for (const [nodeId, inlet] of nodeInflows) {
            const node = this.nodes.get(nodeId);
            if (!node) {
                this.outletIodineMassMg += inlet.massMg;
                continue;
            }
            const outlets = [];
            let totalOutletFlow = 0;
            if (node.parentEdgeIndex >= 0) {
                const parent = this.edges[node.parentEdgeIndex];
                const signedFlow = this.getFaceSignedFlowMm3PerS(
                    parent.index,
                    parent.cellCount,
                    waveform
                );
                if (signedFlow < 0) {
                    const flow = -signedFlow;
                    outlets.push({
                        edge: parent,
                        cellIndex: parent.cellCount - 1,
                        flow
                    });
                    totalOutletFlow += flow;
                }
            }
            for (const childIndex of node.childEdgeIndices) {
                const child = this.edges[childIndex];
                const signedFlow = this.getFaceSignedFlowMm3PerS(
                    child.index,
                    0,
                    waveform
                );
                if (signedFlow > 0) {
                    outlets.push({ edge: child, cellIndex: 0, flow: signedFlow });
                    totalOutletFlow += signedFlow;
                }
            }
            if (totalOutletFlow > 0) {
                for (const outlet of outlets) {
                    outlet.edge.nextMassMg[outlet.cellIndex] +=
                        inlet.massMg * outlet.flow / totalOutletFlow;
                    markTouched(outlet.edge.index);
                }
                continue;
            }
            const isOpenBoundary =
                node.parentEdgeIndex < 0 ||
                node.childEdgeIndices.length === 0;
            if (isOpenBoundary) {
                this.outletIodineMassMg += inlet.massMg;
                continue;
            }
            // A transient zero-flow internal node has finite blood volume even
            // though the graph node does not. Return mass to its donor boundary
            // until a signed outlet reappears instead of deleting or teleporting
            // the contrast.
            for (const origin of inlet.origins) {
                origin.edge.nextMassMg[origin.boundaryCellIndex] +=
                    origin.massMg;
            }
        }

        for (const edgeIndex of touched) {
            const edge = this.edges[edgeIndex];
            const swap = edge.massMg;
            edge.massMg = edge.nextMassMg;
            edge.nextMassMg = swap;
            let activeMass = 0;
            for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
                activeMass += edge.massMg[cellIndex];
            }
            edge.active = activeMass > MIN_ACTIVE_EDGE_MASS_MG;
            if (edge.active) this._activeEdgeIndices.add(edgeIndex);
            else {
                this._activeEdgeIndices.delete(edgeIndex);
                // Sub-resolution remnants represent iodine that has entered
                // terminal microcirculation beyond the resolved centerline
                // tree. Account it as outlet mass instead of leaving a
                // permanent numerical stain in a tiny vessel.
                this.outletIodineMassMg += activeMass;
                edge.massMg.fill(0);
                edge.nextMassMg.fill(0);
            }
        }
    }

    _applyJunctionDispersion(dt, touchedEdgeIndices = this.edges.map(edge => edge.index)) {
        const coefficient = Math.max(0, this.hemodynamics.axialDispersionMm2PerS);
        if (!(coefficient > 0)) return;
        const waveform = arterialWaveform(
            this.time,
            this.hemodynamics.heartRateBpm
        );
        for (const edgeIndex of touchedEdgeIndices) {
            const edge = this.edges[edgeIndex];
            if (!edge.childEdgeIndices.length) continue;
            if (!edge.active && !edge.childEdgeIndices.some(
                childIndex => this.edges[childIndex].active
            )) continue;
            const parentIndex = edge.cellCount - 1;
            const parentVolume = Math.max(
                1e-9,
                edge.volumes[parentIndex]
            );
            const parentConcentration =
                edge.nextMassMg[parentIndex] / parentVolume;
            const transfers = [];
            let requestedTransferMassMg = 0;
            const parentEndFlow = this.getFaceSignedFlowMm3PerS(
                edge.index,
                edge.cellCount,
                waveform
            );
            if (!(parentEndFlow > 0)) continue;
            for (const childIndex of edge.childEdgeIndices) {
                const child = this.edges[childIndex];
                const childStartFlow = this.getFaceSignedFlowMm3PerS(
                    child.index,
                    0,
                    waveform
                );
                if (!(childStartFlow > 0)) continue;
                const childVolume = Math.max(
                    1e-9,
                    child.volumes[0]
                );
                const childConcentration =
                    child.nextMassMg[0] / childVolume;
                // Once the local 3D jet has mixed into the 1D arterial model,
                // junction exchange must remain antegrade. Allowing symmetric
                // diffusion here slowly leaks iodine into the upstream parent
                // and then into unrelated sibling territories, producing
                // non-physiological late vessel staining.
                if (!(parentConcentration > childConcentration)) continue;
                const interfaceArea = Math.min(
                    edge.areas[parentIndex],
                    child.areas[0]
                );
                const distance = Math.max(
                    1e-6,
                    (edge.cellLength + child.cellLength) * 0.5
                );
                let requestedTransfer = coefficient * interfaceArea *
                    (parentConcentration - childConcentration) / distance * dt;
                const equilibriumTransfer =
                    (parentConcentration - childConcentration) /
                    (1 / parentVolume + 1 / childVolume);
                requestedTransfer = Math.min(
                    requestedTransfer,
                    equilibriumTransfer
                );
                if (!(requestedTransfer > 0)) continue;
                transfers.push({ child, requestedTransfer });
                requestedTransferMassMg += requestedTransfer;
            }

            if (!(requestedTransferMassMg > 0)) continue;
            const maximumTransferMassMg =
                edge.nextMassMg[parentIndex] * 0.25;
            const transferScale = Math.min(
                1,
                maximumTransferMassMg / requestedTransferMassMg
            );
            let transferredMassMg = 0;
            for (const { child, requestedTransfer } of transfers) {
                const transfer = requestedTransfer * transferScale;
                child.nextMassMg[0] += transfer;
                transferredMassMg += transfer;
            }
            edge.nextMassMg[parentIndex] -= transferredMassMg;
        }
    }

    _applyAxialDispersion(edge, dt) {
        const coefficient = Math.max(0, this.hemodynamics.axialDispersionMm2PerS);
        if (!(coefficient > 0) || edge.cellCount < 2) return;
        const next = edge.nextMassMg;
        const delta = edge.dispersionDeltaMg;
        delta.fill(0);
        for (let cellIndex = 0; cellIndex < edge.cellCount - 1; cellIndex++) {
            const leftConcentration = next[cellIndex] / Math.max(1e-9, edge.volumes[cellIndex]);
            const rightConcentration = next[cellIndex + 1] /
                Math.max(1e-9, edge.volumes[cellIndex + 1]);
            const interfaceArea = (edge.areas[cellIndex] + edge.areas[cellIndex + 1]) * 0.5;
            let transfer = coefficient * interfaceArea *
                (leftConcentration - rightConcentration) /
                Math.max(1e-6, edge.cellLength) * dt;
            if (transfer > 0) transfer = Math.min(transfer, next[cellIndex] * 0.45);
            else transfer = -Math.min(-transfer, next[cellIndex + 1] * 0.45);
            delta[cellIndex] -= transfer;
            delta[cellIndex + 1] += transfer;
        }
        for (let cellIndex = 0; cellIndex < edge.cellCount; cellIndex++) {
            next[cellIndex] = Math.max(0, next[cellIndex] + delta[cellIndex]);
        }
    }

    _updateEdgeConcentrations() {
        let totalMass = 0;
        for (const edge of this.edges) {
            let mass = 0;
            for (let index = 0; index < edge.cellCount; index++) mass += edge.massMg[index];
            edge.meanConcentrationMgPerMm3 = mass / Math.max(1e-9, edge.totalVolume);
            totalMass += mass;
        }
        this.totalIodineMassMg = totalMass;
    }

    getIodineMassMg() {
        return this.totalIodineMassMg;
    }

    getMassBalanceSnapshot() {
        return {
            intravascularIodineMassMg: this.getIodineMassMg(),
            outletIodineMassMg: this.outletIodineMassMg
        };
    }

    getTopologyDiagnostics() {
        const outletCount = [...this.nodes.values()].filter(
            node => {
                if (
                    node.id === this.rootNode.id ||
                    node.childEdgeIndices.length !== 0
                ) return false;
                const parent = node.parentEdgeIndex >= 0
                    ? this.edges[node.parentEdgeIndex]
                    : null;
                return !parent?.transportExcluded;
            }
        ).length;
        return {
            sourceSegmentCount: this.sourceSegments.length,
            directedEdgeCount: this.edges.length,
            nodeCount: this.nodes.size,
            outletCount,
            rootNodeId: this.rootNode.id,
            disconnectedSourceSegmentCount:
                this.disconnectedSourceSegmentCount,
            radiusRegularization:
                this.radiusRegularizationDiagnostics,
            aorticBranchPrefixes:
                this.aorticBranchPrefixDiagnostics,
            intraluminalAorticArtifacts:
                this.intraluminalAorticArtifactDiagnostics,
            intraluminalAorticConnectors:
                this.intraluminalAorticConnectorDiagnostics,
            physiologicalTopologyRepair:
                this.topologyRepairDiagnostics
        };
    }
}
