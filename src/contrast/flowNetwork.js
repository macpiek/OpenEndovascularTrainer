import {DilatedLumenMixing} from './dilatedLumenMixing.js';
import {mixRefluxJunctionBoundaries} from './refluxJunctionMixing.js';
import * as THREE from 'three';
import {FlowAdvection} from './flowAdvection.js';
import {initializeOutletResistances} from './outletResistance.js';
import {reattachArchOstia} from './archOstiumReattachment.js';
import { repairArterialFlowTopology } from '../arterialFlowTopology.js';

const MM3_PER_ML = 1000;
const SECONDS_PER_MINUTE = 60;
const MIN_RADIUS_MM = 0.35;
const DEFAULT_CELL_LENGTH_MM = 2.5;
const DEFAULT_SPATIAL_CELL_MM = 12;
// Reference tissue-bed resistance (Pa s / mm³), before resolved arteries.
// At 5 L/min, 0.144 corresponds to a 12 kPa tissue pressure drop.
const DEFAULT_PERIPHERAL_RESISTANCE = 0.144;
const DEFAULT_MINIMUM_OUTLET_TERRITORY_RADIUS_MM = 1.2;
const DEFAULT_CFL_LIMIT = 0.72;
const MIN_ACTIVE_EDGE_MASS_MG = 5e-4;
const RADIUS_VALLEY_WINDOW_MM = 28;
const RADIUS_VALLEY_SUPPORT_MM = 6;
const RADIUS_VALLEY_RATIO = 0.78;
const RADIUS_VALLEY_RESTORE_RATIO = 0.92;
const AORTIC_PREFIX_MIN_PARENT_RADIUS_MM = 3;
const AORTIC_PREFIX_START_RADIUS_RATIO = 0.78;
const AORTIC_PREFIX_EXIT_RADIUS_RATIO = 0.85;
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
    axialDispersionMm2PerS: 10,
    axialFlowDispersionFactor: 0.04
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

// Exact Poiseuille integral for a linearly tapered radius. Using a mean
// radius here would change resistance when the same vessel is subdivided.
function arterialResistance(length, r0, r1, viscosity) {
    return 8 * viscosity * length * (r0 * r0 + r0 * r1 + r1 * r1) /
        (3 * Math.PI * r0 ** 3 * r1 ** 3);
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
    const resistance = arterialResistance(length, radiusStart, radiusEnd, bloodViscosityPaS);
    return {
        index: edgeIndex,
        sourceIndex,
        sourceId: source.id ?? sourceIndex,
        renalSide: source.renalSide ?? null,
        renalSector: source.renalSector ?? -1,
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
        peripheralResistancePaSPerMm3 = DEFAULT_PERIPHERAL_RESISTANCE,
        minimumOutletTerritoryRadiusMm = DEFAULT_MINIMUM_OUTLET_TERRITORY_RADIUS_MM,
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
        if (!(peripheralResistancePaSPerMm3 > 0) || !Number.isFinite(peripheralResistancePaSPerMm3))
            throw new RangeError('Peripheral resistance must be finite and positive');
        this.peripheralResistancePaSPerMm3 = peripheralResistancePaSPerMm3;
        this.minimumOutletTerritoryRadiusMm = Math.max(MIN_RADIUS_MM, minimumOutletTerritoryRadiusMm);
        this.cflLimit = cflLimit;
        this.edges = [];
        this.nodeOrder = [];
        this.disconnectedSourceSegmentCount = 0;
        this.time = 0;
        this.outletIodineMassMg = 0;
        this.totalIodineMassMg = 0;
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
        this._buildDirectedTree();
        this._contractIntraluminalAorticBranchConnectors();
        this._suppressIntraluminalAorticDeadEnds();
        this._suppressContainedTerminalSpurs();
        this._regularizeShortRadiusValleys();
        this._regularizeAorticBranchPrefixes();
        this._regularizeSideJunctionRadiusPeaks();
        this._regularizeIntraluminalAorticConnectorVolumes();
        this.archOstiumReattachmentDiagnostics = reattachArchOstia(this);
        this._flowOverridesMm3PerS = new Float64Array(this.edges.length);
        this._flowOverridesMm3PerS.fill(Number.NaN);
        this._branchInletFlowMm3PerS =
            new Float64Array(this.edges.length);
        this._branchInletConcentrationMgPerMm3 =
            new Float64Array(this.edges.length);
        this._queryMarks = new Uint32Array(this.edges.length);
        // Uniform per-edge source responses stay dense; local device outlets
        // retain separate face deltas on the two sides of their source cell.
        this._edgeFlowDeltasMm3PerS = new Float64Array(this.edges.length);
        initializeOutletResistances(this);
        this._computeHydraulicDistribution();
        this._buildSpatialIndex();
        this.minimumCellLength = this.edges.reduce(
            (minimum, edge) => Math.min(minimum, edge.cellLength),
            Infinity
        );
        this._updateMaximumMeanVelocity();
        this._advection = new FlowAdvection(this);
        this.dilatedLumenMixing = new DilatedLumenMixing(this, {includeNormal:true});
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
        this._edgeFlowDeltasMm3PerS.fill(0);
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
            if (this.edges[edgeIndex]&&!this.edges[edgeIndex].transportExcluded) {
                this._flowOverridesMm3PerS[edgeIndex] = signedFlowMm3PerS;
            }
        }
    }

    getSignedFlowMm3PerS(edgeIndex, waveform = 1) {
        if(this.edges[edgeIndex]?.transportExcluded)return 0;
        const override = this._flowOverridesMm3PerS[edgeIndex];
        const baseFlow = Number.isFinite(override)
            ? override
            : (this.edges[edgeIndex]?.meanFlowMm3PerS || 0) * waveform;
        const edge = this.edges[edgeIndex];
        if (!edge) return baseFlow;
        const faces = edge.faceFlowDeltaMm3PerS;
        return baseFlow + this._edgeFlowDeltasMm3PerS[edgeIndex] + (faces[0] + faces[faces.length - 1]) * 0.5;
    }

    getFaceSignedFlowMm3PerS(edgeIndex, faceIndex, waveform = 1) {
        const edge = this.edges[edgeIndex];
        if (!edge||edge.transportExcluded) return 0;
        const override = this._flowOverridesMm3PerS[edgeIndex];
        const baseFlow = Number.isFinite(override)
            ? override
            : edge.meanFlowMm3PerS * waveform;
        const resolvedFaceIndex = THREE.MathUtils.clamp(
            Math.floor(faceIndex),
            0,
            edge.cellCount
        );
        return baseFlow + this._edgeFlowDeltasMm3PerS[edgeIndex] +
            edge.faceFlowDeltaMm3PerS[resolvedFaceIndex];
    }

    getFaceDispersionFlowMm3PerS(edgeIndex, faceIndex, waveform = 1) {
        const edge = this.edges[edgeIndex];
        if (!edge || edge.transportExcluded) return 0;
        const override = this._flowOverridesMm3PerS[edgeIndex];
        const carrier = Number.isFinite(override) ? override : edge.meanFlowMm3PerS * waveform;
        const delta = this._edgeFlowDeltasMm3PerS[edgeIndex] +
            edge.faceFlowDeltaMm3PerS[Math.max(0, Math.min(edge.cellCount, faceIndex))];
        // Reduced unresolved shear: opposing injection and blood do not stop
        // mixing when their NET transport cancels. This is |Qnet| plus twice
        // the opposing component, using the existing axial-dispersion closure.
        // With no injection (or co-directed flow) it equals the transport speed.
        return Math.abs(carrier) + Math.abs(delta);
    }

    addFaceFlowDelta(
        edgeIndex,
        startFaceIndex,
        endFaceIndexExclusive,
        deltaMm3PerS
    ) {
        const edge = this.edges[edgeIndex];
        if (!edge || edge.transportExcluded || !Number.isFinite(deltaMm3PerS) || !deltaMm3PerS) {
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
        // A connector can have a longer shared stem before its final split.
        // Correct that stem bottom-up too, but stop as soon as one daughter
        // is the uncorrected continuing aorta. Otherwise each nested split
        // retains another aorta-sized reservoir upstream of the repaired mouths.
        const supported = new Set(diagnostics.connectors.map(c => c.connectorEdgeIndex));
        const distance = new Float64Array(this.edges.length);
        diagnostics.sharedStems = [];
        for (let order = this.nodeOrder.length - 1; order >= 0; order--) {
            const node = this.nodes.get(this.nodeOrder[order]);
            const edge = this.edges[node.parentEdgeIndex];
            if (!edge || edge.transportExcluded || supported.has(edge.index) ||
                edge.renalSide || !node.childEdgeIndices.length) continue;
            if (!node.childEdgeIndices.every(i => supported.has(i))) continue;
            const children = node.childEdgeIndices.map(i => this.edges[i]);
            const length = edge.length + Math.max(...children.map(e => distance[e.index]));
            if (length > AORTIC_PREFIX_MAX_LENGTH_MM ||
                Math.min(edge.rawRadiusStart, edge.rawRadiusEnd) < AORTIC_DUPLICATE_MIN_PARENT_RADIUS_MM) continue;
            const radius = Math.max(...children.map(e => e.radiusStart));
            const before = edge.totalVolume;
            edge.radiusStart = Math.min(edge.radiusStart, radius);
            edge.radiusEnd = Math.min(edge.radiusEnd, radius);
            edge.lumenMappingUsesCorrectedRadius = true;
            this._rebuildEdgeTransportGeometry(edge);
            supported.add(edge.index);
            distance[edge.index] = length;
            const removed = before - edge.totalVolume;
            removedVolumeMm3 += removed;
            diagnostics.sharedStems.push({edgeIndex: edge.index, removedVolumeMm3: removed});
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

    _suppressContainedTerminalSpurs() {
        // Short TEASAR strands can taper to an artificial outlet while staying
        // entirely inside another artery. They are duplicate blood volume,
        // not a side vessel: their low terminal flow leaves an isolated plug.
        const roots = [];
        const downstreamLength = new Float64Array(this.edges.length);
        for (let i = this.nodeOrder.length - 1; i >= 0; i--) {
            const node = this.nodes.get(this.nodeOrder[i]);
            if (node.parentEdgeIndex < 0) continue;
            const e = this.edges[node.parentEdgeIndex];
            downstreamLength[e.index] = e.length + Math.max(0,
                ...node.childEdgeIndices.map(index => downstreamLength[index]));
        }
        const longestContinuation = (a, b) => downstreamLength[b.index] - downstreamLength[a.index];
        for (const node of this.nodes.values()) {
            const parent = this.edges[node.parentEdgeIndex];
            if (!parent || parent.renalSide || parent.transportExcluded ||
                node.childEdgeIndices.length < 2) continue;
            for (const rootIndex of [...node.childEdgeIndices]) {
                const path = [];
                let edge = this.edges[rootIndex], length = 0;
                while (edge && !edge.renalSide && !edge.transportExcluded) {
                    path.push(edge);
                    length += edge.length;
                    if (length > 80 ||
                        edge.childEdgeIndices.length !== 1) break;
                    edge = this.edges[edge.childEdgeIndices[0]];
                }
                if (!path.length || path[path.length - 1] !== edge || edge.childEdgeIndices.length ||
                    length > 80) continue;
                const main = node.childEdgeIndices.filter(i => i !== rootIndex)
                    .map(i => this.edges[i]).sort(longestContinuation)[0];
                if (!main || main.transportExcluded) continue;
                // The real artery may fork again alongside the duplicate, or
                // share a junction a few samples upstream. Check the connected
                // neighbourhood, not just the longest daughter at each fork.
                let anchor = parent, upstream = 0;
                while (anchor.parentEdgeIndex >= 0 && upstream < Math.max(4, parent.rawRadiusEnd * 2)) {
                    upstream += anchor.length;
                    anchor = this.edges[anchor.parentEdgeIndex];
                }
                const continuation = [], queue = [{edge: anchor, distance: 0}];
                const candidateIndices = new Set(path.map(e => e.index));
                for (let q = 0; q < queue.length; q++) {
                    const {edge: current, distance} = queue[q];
                    if (candidateIndices.has(current.index) || current.transportExcluded || current.renalSide) continue;
                    continuation.push(current);
                    if (distance > upstream + length * 2 + parent.rawRadiusEnd) continue;
                    for (const index of current.childEdgeIndices) {
                        queue.push({edge: this.edges[index], distance: distance + current.length});
                    }
                }
                if (downstreamLength[anchor.index] - upstream < length * 1.5) continue;
                const contained = path.every(e => [0, 0.5, 1].every(t => {
                    const point = e.start.clone().lerp(e.end, t);
                    return continuation.some(trunk => {
                        const axial = point.clone().sub(trunk.start).dot(trunk.axis);
                        if (axial < -1e-6 || axial > trunk.length + 1e-6 ||
                            e.axis.dot(trunk.axis) < 0.7) return false;
                        const radius = THREE.MathUtils.lerp(trunk.rawRadiusStart,
                            trunk.rawRadiusEnd, axial / trunk.length);
                        return point.distanceTo(trunk.start.clone().addScaledVector(trunk.axis, axial))
                            < radius * 0.5;
                    });
                }));
                if (!contained) continue;
                node.childEdgeIndices.splice(node.childEdgeIndices.indexOf(rootIndex), 1);
                for (const e of path) {
                    e.transportExcluded = true;
                    e.renderExcluded = true;
                    e.lumenMappingExcluded = true;
                }
                roots.push({rootEdgeIndex: rootIndex, mainContinuationEdgeIndex: main.index,
                    edgeIndices: path.map(e => e.index), lengthMm: length});
            }
        }
        this.containedTerminalSpurDiagnostics = {roots};
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
            edge.resistance = arterialResistance(edge.length, edge.radiusStart,
                edge.radiusEnd, this.hemodynamics.bloodViscosityPaS);
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
        edge.resistance = arterialResistance(edge.length, edge.radiusStart,
            edge.radiusEnd, this.hemodynamics.bloodViscosityPaS);
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
            // A sampled valley at the ostium may already have been repaired.
            // Using only the raw value here misses branches inside a dilated
            // aorta and gives their technical prefix the parent's full volume.
            const parentRadius = Math.max(parent.rawRadiusEnd, parent.radiusEnd);
            if (parentRadius < AORTIC_PREFIX_MIN_PARENT_RADIUS_MM) {
                continue;
            }

            for (const rootEdgeIndex of node.childEdgeIndices) {
                const rootEdge = this.edges[rootEdgeIndex];
                if (
                    !rootEdge ||
                    Math.max(rootEdge.rawRadiusStart, rootEdge.radiusStart) <
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
                        edge.radiusEnd <= Math.min(
                            parentRadius * AORTIC_PREFIX_EXIT_RADIUS_RATIO,
                            AORTIC_PREFIX_MAX_BRANCH_RADIUS_MM
                        )
                    ) {
                        branchRadiusMm = Math.max(
                            MIN_RADIUS_MM,
                            edge.radiusEnd
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
                    // Use the repaired downstream lumen at the join. A raw
                    // medial-axis valley must not narrow a whole iliac limb
                    // and then jump back to its real radius after this prefix.
                    branchRadiusMm = Math.max(MIN_RADIUS_MM, edge.radiusEnd);
                }
                if (
                    branchRadiusMm >
                        AORTIC_PREFIX_MAX_BRANCH_RADIUS_MM
                ) continue;

                // Identify a side ostium by its downstream calibre, not by an
                // aneurysm-sized parent. At a balanced fork both daughters
                // naturally taper; neither may be mistaken for a side pocket.
                const siblingProbeDistance=Math.min(AORTIC_PREFIX_MAX_LENGTH_MM,
                    Math.max(pathLengthMm,parentRadius*4));
                // A technical connector lies inside the aorta and may split
                // into two genuine side vessels. Compare both with the aorta,
                // not with each other, or their inherited aortic-sized mouths
                // are incorrectly protected as a balanced bifurcation.
                const siblingRadius = Math.max(parent.intraluminalAorticConnector ? parentRadius : 0,...node.childEdgeIndices
                    .filter(index=>index!==rootEdgeIndex)
                    .map(index=>{
                        let sibling=this.edges[index],distance=0;
                        while(sibling && distance+sibling.length<siblingProbeDistance && sibling.childEdgeIndices.length){
                            distance+=sibling.length;
                            sibling=sibling.childEdgeIndices.map(i=>this.edges[i])
                                .reduce((a,b)=>a.radiusEnd>b.radiusEnd?a:b);
                        }
                        return sibling?Math.max(sibling.rawRadiusEnd,sibling.radiusEnd):0;
                    }));
                if(branchRadiusMm>siblingRadius*.80)continue;

                const ostialRadiusMm = Math.min(
                    parentRadius,
                    branchRadiusMm * (parentRadius < 10 ? 1.15 : AORTIC_PREFIX_OSTIAL_FLARE_RATIO)
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

    _regularizeSideJunctionRadiusPeaks() {
        // Medial-axis samples near a side mouth can alternate between the
        // trunk wall and the shared junction envelope. Remove local peaks
        // along the continuing artery, using intact lumen at both ends.
        // Apply this to transport and display together: a visual-only fix
        // would leave the same oversized reservoir in the flow model.
        const diagnostics = [];
        for (const prefix of this.aorticBranchPrefixDiagnostics.paths) {
            const side = this.edges[prefix.rootEdgeIndex];
            const parent = this.edges[side.parentEdgeIndex];
            if (!parent || parent.radiusEnd < 3 || parent.radiusEnd > 6.5 ||
                parent.childEdgeIndices.length !== 2) continue;
            const main = this.edges[parent.childEdgeIndices.find(i => i !== side.index)];
            const reach = THREE.MathUtils.clamp(parent.radiusEnd * 3, 8, 16);
            const upstream = [], downstream = [];
            let edge = parent, distance = 0;
            while (edge) {
                upstream.unshift(edge);
                distance += edge.length;
                const previous = this.edges[edge.parentEdgeIndex];
                if (distance >= reach || !previous || previous.childEdgeIndices.length !== 1) break;
                edge = previous;
            }
            edge = main;
            distance = 0;
            while (edge) {
                downstream.push(edge);
                distance += edge.length;
                if (distance >= reach || edge.childEdgeIndices.length !== 1) break;
                edge = this.edges[edge.childEdgeIndices[0]];
            }
            const path = [...upstream, ...downstream];
            const length = path.reduce((sum, e) => sum + e.length, 0);
            const startRadius = path[0].radiusStart, endRadius = path.at(-1).radiusEnd;
            // A side junction correction must never flatten a large dilatation.
            if (Math.max(startRadius, endRadius) > 6.5) continue;
            distance = 0;
            let removedVolumeMm3 = 0;
            for (const e of path) {
                const volume = e.totalVolume;
                e.radiusStart = Math.min(e.radiusStart,
                    THREE.MathUtils.lerp(startRadius, endRadius, distance / length));
                distance += e.length;
                e.radiusEnd = Math.min(e.radiusEnd,
                    THREE.MathUtils.lerp(startRadius, endRadius, distance / length));
                this._rebuildEdgeTransportGeometry(e);
                if (e.totalVolume < volume - 1e-9) e.lumenMappingUsesCorrectedRadius = true;
                removedVolumeMm3 += volume - e.totalVolume;
            }
            diagnostics.push({ parentEdgeIndex: parent.index, edgeIndices: path.map(e => e.index),
                startRadius, endRadius, lengthMm: length, removedVolumeMm3 });
        }
        this.sideJunctionRadiusDiagnostics = diagnostics;
    }

    setOutletResistance(nodeId, resistancePaSPerMm3) {
        const node = this.nodes.get(nodeId);
        if (!node?.nativeOutlet || !(resistancePaSPerMm3 > 0) || !Number.isFinite(resistancePaSPerMm3))
            throw new RangeError('A native outlet and a finite positive resistance are required');
        node.outletResistancePaSPerMm3 = resistancePaSPerMm3;
        this._computeHydraulicDistribution();
        this._updateMaximumMeanVelocity();
    }

    _computeHydraulicDistribution() {
        this.hydraulicRevision = (this.hydraulicRevision || 0) + 1;
        // Resistances are in Pa s/mm³. Resolved arteries are in series with
        // their downstream tissue beds; daughters are connected in parallel.
        // Bed resistances are initialized once, not rebalanced after a device
        // occludes a branch or changes its lumen.
        for (let i = this.nodeOrder.length - 1; i >= 0; i--) {
            const node = this.nodes.get(this.nodeOrder[i]);
            let conductance = 0, outlets = 0;
            for (const index of node.childEdgeIndices) {
                const edge = this.edges[index], child = this.nodes.get(edge.endNodeId);
                edge.equivalentResistance = edge.transportExcluded
                    ? Infinity : edge.resistance + child.equivalentResistance;
                conductance += 1 / edge.equivalentResistance;
                if (!edge.transportExcluded) outlets += child.terminalOutletCount;
            }
            node.terminalOutletCount = node.nativeOutlet ? 1 : outlets;
            node.equivalentResistance = node.nativeOutlet
                ? node.outletResistancePaSPerMm3
                : conductance > 0 ? 1 / conductance : Infinity;
            node.pressurePa = 0;
        }
        for (const edge of this.edges) {
            edge.meanFlowMm3PerS = 0;
            edge.flowSplitShare = 0;
        }
        const inletFlow = this.hemodynamics.cardiacOutputMlPerMin * MM3_PER_ML / SECONDS_PER_MINUTE;
        this.rootNode.pressurePa = Number.isFinite(this.rootNode.equivalentResistance)
            ? inletFlow * this.rootNode.equivalentResistance : 0;
        for (const id of this.nodeOrder) {
            const node = this.nodes.get(id);
            for (const index of node.childEdgeIndices) {
                const edge = this.edges[index], child = this.nodes.get(edge.endNodeId);
                if (edge.transportExcluded || !Number.isFinite(edge.equivalentResistance)) continue;
                edge.meanFlowMm3PerS = node.pressurePa / edge.equivalentResistance;
                edge.flowSplitShare = node.equivalentResistance / edge.equivalentResistance;
                child.pressurePa = edge.meanFlowMm3PerS * child.equivalentResistance;
            }
        }
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
        const section=bestEdge.graftSections?.[out.cellIndex];
        if(section?.center&&section.tangent) {
            const tangent=section.tangent,dx=x-section.center.x,dy=y-section.center.y,dz=z-section.center.z;
            const axial=dx*tangent.x+dy*tangent.y+dz*tangent.z;
            const distance=Math.hypot(dx-axial*tangent.x,dy-axial*tangent.y,dz-axial*tangent.z);
            if(this.stentGraftRemodeling?.surface.sealed||distance<section.radius) {
                const direction=Math.sign(tangent.dot(bestEdge.axis))||1;
                out.centerX=section.center.x+axial*tangent.x;out.centerY=section.center.y+axial*tangent.y;out.centerZ=section.center.z+axial*tangent.z;
                out.distance=distance;out.radius=section.radius;
                out.tangentX=tangent.x*direction;out.tangentY=tangent.y*direction;out.tangentZ=tangent.z*direction;
            }
        }
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

    depositGraftSacAtCoordinates(x,y,z,mass) {
        const remodeling=this.stentGraftRemodeling;
        if(!remodeling?.sac||!(mass>0))return false;
        const point={x,y,z};
        if(!remodeling.surface.bounds.containsPoint(point)||remodeling.surface.contains(point))return false;
        const location=this.findNearestLocationCoordinates(x,y,z,this._locationScratch);
        const entry=remodeling.trapped.get(location.edgeIndex),cell=location.cellIndex;
        if(!entry||!(entry.volumes[cell]>0))return false;
        entry.mass[cell]+=mass;remodeling.trappedIodineMassMg+=mass;return true;
    }

    depositIodineAtCoordinates(x, y, z, iodineMassMg) {
        if (!(iodineMassMg > 0)) return false;
        if(this.depositGraftSacAtCoordinates(x,y,z,iodineMassMg))return true;
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
        if(edge.transportExcluded) {
            const entry=this.stentGraftRemodeling?.trapped.get(edgeIndex);
            if(!entry)return false;
            entry.mass[Math.max(0,Math.min(edge.cellCount-1,Math.floor(cellIndex)))]+=iodineMassMg;
            this.stentGraftRemodeling.trappedIodineMassMg+=iodineMassMg;
            return true;
        }
        const index = THREE.MathUtils.clamp(Math.floor(cellIndex), 0, edge.cellCount - 1);
        edge.massMg[index] += iodineMassMg;
        edge.active = true;
        this._activeEdgeIndices.add(edge.index);
        this.totalIodineMassMg += iodineMassMg;
        return true;
    }

    // Entrainment delivered between two transport boundaries is continuous
    // during that next interval. Ordinary depositIodine remains an instant
    // bolus for callers which intentionally set an initial concentration.
    queueIodineSource(edgeIndex, cellIndex, iodineMassMg) {
        const edge = this.edges[edgeIndex];
        if (!edge || !(iodineMassMg > 0) || !Number.isFinite(iodineMassMg)) return false;
        if (edge.transportExcluded) return this.depositIodine(edgeIndex, cellIndex, iodineMassMg);
        const cell = THREE.MathUtils.clamp(Math.floor(cellIndex), 0, edge.cellCount - 1);
        this._advection.queueSource(edgeIndex, cell, iodineMassMg);
        // Pending source iodine has already left the local jet and belongs to
        // this network. Count it once even before update consumes the queue.
        this.totalIodineMassMg += iodineMassMg;
        return true;
    }

    update(dt) {
        if (!(dt > 0)) return;
        if (
            !(this.totalIodineMassMg > 1e-10) &&
            !this._advection.queuedSourceVertices.length &&
            !this._branchInletEdgeIndices.size &&
            !(this.stentGraftRemodeling?.sac && this.stentGraftRemodeling.trappedIodineMassMg > 1e-10)
        ) {
            this.time += dt;
            return;
        }
        // Accuracy steps resolve the pulse; stability no longer depends
        // on the smallest atlas cell or an arbitrary cap on CFL substeps.
        const count = Math.max(1, Math.ceil(dt*30)), step = dt/count;
        this._advection.sourceIntervalSeconds = dt;
        this.lastTransportSubstepCount = count;
        for (let i = 0; i < count; i++) {
            const waveform=arterialWaveform(this.time+step*.5,this.hemodynamics.heartRateBpm);
            this._advection.prepare(waveform);
            this._transportSubstep(step,waveform);
            this.stentGraftRemodeling?.sac?.update(step);
            this.time += step;
        }
        this._advection.finishSourceInterval();
        this._updateEdgeConcentrations(true);
    }

    _transportSubstep(dt,waveform=1) {
        const touched = this._advection.update(dt), marks = this._advection.edgeMarks;
        // Diffusion can enter a clear daughter after the directed sweep.
        const reached = touched.length;
        for (let k = 0; k < reached; k++) {
            const edge = this.edges[touched[k]];
            edge.active = true;
            for (const childIndex of edge.childEdgeIndices) if (!marks[childIndex]) {
                marks[childIndex] = 1; touched.push(childIndex);
                this.edges[childIndex].nextMassMg.set(this.edges[childIndex].massMg);
            }
        }
        this._applyJunctionDispersion(dt, touched, reached, marks, waveform);
        this.dilatedLumenMixing.update(dt, touched, marks,waveform);
        this._activeEdgeIndices.clear();
        for (const edgeIndex of touched) {
            const edge = this.edges[edgeIndex];
            const swap = edge.massMg; edge.massMg = edge.nextMassMg; edge.nextMassMg = swap;
            let mass = 0;
            for (const value of edge.massMg) mass += value;
            edge.active = mass > MIN_ACTIVE_EDGE_MASS_MG;
            if (edge.active) this._activeEdgeIndices.add(edge.index);
            else {
                this.outletIodineMassMg += mass;
                edge.massMg.fill(0); edge.nextMassMg.fill(0);
                edge.meanConcentrationMgPerMm3 = 0;
            }
        }
    }

    _applyJunctionDispersion(dt, touched, count, marks, waveform) {
        const base = Math.max(0, this.hemodynamics.axialDispersionMm2PerS);
        if (!(base > 0)) return;
        const shear = Math.max(0, this.hemodynamics.axialFlowDispersionFactor ?? .04);
        const junctions = new Set();
        for (let i = 0; i < count; i++) {
            const edge = this.edges[touched[i]];
            for (const id of [edge.startNodeId, edge.endNodeId]) {
                const node = this.nodes.get(id);
                if (node?.parentEdgeIndex >= 0 && node.childEdgeIndices.length > 1) junctions.add(node);
            }
        }
        for (const node of junctions) {
            const incoming = [], outgoing = [];
            const add = (edgeIndex, atEnd) => {
                const edge = this.edges[edgeIndex], cell = atEnd ? edge.cellCount - 1 : 0;
                if (edge.transportExcluded || edge.graftSections?.[cell]) return;
                const q = (atEnd ? 1 : -1) * this.getFaceSignedFlowMm3PerS(edgeIndex, atEnd ? edge.cellCount : 0, waveform);
                if (!q) return;
                const mass = (marks[edgeIndex] ? edge.nextMassMg : edge.massMg)[cell];
                const volume = Math.max(1e-9, edge.volumes[cell]);
                (q > 0 ? incoming : outgoing).push({edge, cell, face: atEnd ? edge.cellCount : 0, q: Math.abs(q), mass, volume, c: mass / volume});
            };
            add(node.parentEdgeIndex, true);
            for (const child of node.childEdgeIndices) add(child, false);
            if (!incoming.length || !outgoing.length) continue;
            // Antegrade blood and an actual refluxing daughter meet in the
            // common lumen. Exchange their boundary-cell contents locally;
            // otherwise two disjoint axial chains pin a hard front to the node.
            if (mixRefluxJunctionBoundaries(incoming, node.parentEdgeIndex, dt, shear)) {
                for (const a of incoming) {
                    if (!marks[a.edge.index]) {
                        marks[a.edge.index] = 1; touched.push(a.edge.index);
                        a.edge.nextMassMg.set(a.edge.massMg);
                    }
                    a.edge.nextMassMg[a.cell] = a.mass;
                }
            }
            const inflow = incoming.reduce((sum, a) => sum + a.q, 0);
            const iodineFlux = incoming.reduce((sum, a) => sum + a.q * a.c, 0);
            if (!(iodineFlux > 0)) continue;
            // Incoming blood and refluxing contrast form one flow-weighted
            // mixture. A reversed daughter can be a donor, and a reversed
            // parent a receiver; anatomical parent/child labels are not flow.
            const mixed = iodineFlux / inflow;
            const inletLength = incoming.reduce((sum, a) => sum + a.q * a.edge.cellLength, 0) / inflow;
            const inletArea = incoming.reduce((sum, a) => sum + a.edge.areas[a.cell], 0);
            let requested = 0;
            for (const b of outgoing) {
                const difference = mixed - b.c;
                if (!(difference > 0)) continue;
                const area = Math.min(inletArea, b.edge.areas[b.cell]);
                const dispersionFlow = this.getFaceDispersionFlowMm3PerS(
                    b.edge.index, b.face, waveform);
                const speed = dispersionFlow / Math.max(1e-9, area);
                const diffusion = base + shear * speed * Math.sqrt(area / Math.PI);
                b.transfer = Math.min(
                    dt * diffusion * area * difference / Math.max(1e-6, (inletLength + b.edge.cellLength) * .5),
                    difference * b.volume
                );
                requested += b.transfer;
            }
            if (!(requested > 0)) continue;
            let available = Infinity;
            for (const a of incoming) {
                a.share = a.q * a.c / iodineFlux;
                if (a.share > 0) available = Math.min(available, .25 * a.mass / a.share);
            }
            const scale = Math.min(1, available / requested);
            const change = (boundary, amount) => {
                const {edge, cell} = boundary;
                if (!marks[edge.index]) {
                    marks[edge.index] = 1; touched.push(edge.index);
                    edge.nextMassMg.set(edge.massMg);
                }
                edge.nextMassMg[cell] += amount;
            };
            // Directed exchange cannot diffuse a selective injection into an
            // unrelated inflow arm. Every added microgram leaves a donor cell.
            for (const b of outgoing) if (b.transfer > 0) change(b, b.transfer * scale);
            for (const a of incoming) if (a.share > 0) change(a, -requested * scale * a.share);
        }
    }

    _updateEdgeConcentrations(activeOnly=false) {
        let totalMass = 0;
        const indices=activeOnly?this._activeEdgeIndices:this.edges.keys();
        for (const index of indices) {
            const edge=this.edges[index];
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
            archOstiumReattachments: this.archOstiumReattachmentDiagnostics,
            physiologicalTopologyRepair:
                this.topologyRepairDiagnostics
        };
    }
}
