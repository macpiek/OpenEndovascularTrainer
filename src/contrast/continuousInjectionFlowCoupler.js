import { arterialWaveform } from './flowNetwork.js';

const MM3_PER_ML = 1000;
function conductance(resistance) {
    return Number.isFinite(resistance) && resistance > 0 ? 1 / resistance : 0;
}

function resistanceFraction(edge, t) {
    // Preserve the exact tapered-artery resistance when the source lies inside
    // an edge; the finite-volume source is located at its control-volume centre.
    const a = edge.radiusStart, b = edge.radiusEnd, r = a + (b - a) * t;
    return t * (a * a + a * r + r * r) / (a * a + a * b + b * b) * (b / r) ** 3;
}

/**
 * Incremental Kirchhoff solution for device volume sources on the arterial
 * resistor tree. The cardiac inlet retains its prescribed waveform; all added
 * fluid leaves through tissue-bed resistances. Local reflux is possible, but
 * outlet direction does not prescribe bulk reversal of every ancestor vessel.
 * The local jet solver remains responsible for directional nozzle momentum.
 */
export class ContinuousInjectionFlowCoupler {
    constructor(flowNetwork) {
        if (!flowNetwork) throw new TypeError('A flow network is required');
        this.flowNetwork = flowNetwork;
        this.active = false;
        this.currentUpstreamRateMlPerSec = 0;
        this.currentDownstreamRateMlPerSec = 0;
        this.currentUpstreamFraction = 0;
        this.currentReversedEdgeCount = 0;
        this.maximumReversedEdgeCount = 0;
        this.maximumUpstreamRateMlPerSec = 0;
        this.maximumDownstreamRateMlPerSec = 0;
        this.minimumSignedFlowMlPerSec = 0;
        this.configurationCount = 0;
        this._sourceEdgeIndices = new Uint8Array(flowNetwork.edges.length);
        this._locationScratch = {};
        this._hydraulicRevision = -1;
        this._upstreamResistance = new Map();
        this._edgeProximalResistance = new Float64Array(flowNetwork.edges.length);
        this._responses = new Map();
        this._uniformDeltas = new Float64Array(flowNetwork.edges.length);
    }

    beginStep() {
        this.active = false;
        this.currentUpstreamRateMlPerSec = 0;
        this.currentDownstreamRateMlPerSec = 0;
        this.currentUpstreamFraction = 0;
        this.currentReversedEdgeCount = 0;
        this.minimumSignedFlowMlPerSec = 0;
        this._sourceEdgeIndices.fill(0);
    }

    configure({
        ports,
        rateMlPerSec,
        portFlowRatesMlPerSec = null
    } = {}) {
        if (!Array.isArray(ports) || !ports.length || !(rateMlPerSec > 0)) {
            return [];
        }

        const activePorts = ports.filter(
            port => port?.position && port.valid !== false
        );
        if (!activePorts.length) return [];

        const resolvedRates =
            Array.isArray(portFlowRatesMlPerSec) &&
            portFlowRatesMlPerSec.length === activePorts.length &&
            portFlowRatesMlPerSec.every(
                rate => Number.isFinite(rate) && rate >= 0
            )
                ? portFlowRatesMlPerSec
                : null;
        const totalWeight = resolvedRates
            ? resolvedRates.reduce((sum, rate) => sum + rate, 0)
            : activePorts.reduce(
                (sum, port) => sum + Math.max(0, port.weight || 1),
                0
            ) || activePorts.length;
        this._uniformDeltas.fill(0);
        const splits = [];
        let weightedUpstreamFraction = 0;

        for (let portIndex = 0; portIndex < activePorts.length; portIndex++) {
            const port = activePorts[portIndex];
            const portRateMlPerSec = resolvedRates
                ? resolvedRates[portIndex]
                : rateMlPerSec *
                    Math.max(0, port.weight || 1) /
                    totalWeight;
            const location = this.flowNetwork.findInjectionLocation(
                port.position,
                port.direction,
                this._locationScratch
            );
            if (location.edgeIndex < 0 || !(portRateMlPerSec > 0)) {
                splits.push({
                    upstreamFraction: 0.5,
                    downstreamTargets: []
                });
                continue;
            }

            const response = this._sourceResponse(location);
            const upstreamFraction = response.upstreamFraction;
            const upstreamRateMlPerSec = portRateMlPerSec * upstreamFraction;
            const downstreamRateMlPerSec = portRateMlPerSec - upstreamRateMlPerSec;
            weightedUpstreamFraction += upstreamRateMlPerSec;
            this.currentUpstreamRateMlPerSec += upstreamRateMlPerSec;
            this.currentDownstreamRateMlPerSec += downstreamRateMlPerSec;
            const sourceEdge = this.flowNetwork.edges[location.edgeIndex];
            this._sourceEdgeIndices[sourceEdge.index] = 1;
            this._addFlowDelta(sourceEdge.index, 0, location.cellIndex + 1,
                -upstreamRateMlPerSec * MM3_PER_ML);
            this._addFlowDelta(sourceEdge.index, location.cellIndex + 1, sourceEdge.cellCount + 1,
                downstreamRateMlPerSec * MM3_PER_ML);
            const injectedFlow = portRateMlPerSec * MM3_PER_ML;
            const fractions = response.edgeFractions;
            for (let index = 0; index < fractions.length; index++)
                this._uniformDeltas[index] += injectedFlow * fractions[index];
            const downstreamTargets = this._downstreamTargets(location.edgeIndex);
            splits.push({
                upstreamFraction,
                downstreamTargets,
                location: {
                    edgeIndex: location.edgeIndex,
                    cellIndex: location.cellIndex,
                    t: location.t,
                    radius: location.radius,
                    distance: location.distance,
                    selectionMode: location.selectionMode,
                    tangentX: location.tangentX,
                    tangentY: location.tangentY,
                    tangentZ: location.tangentZ
                }
            });
        }

        const uniform = this.flowNetwork._edgeFlowDeltasMm3PerS;
        for (let index = 0; index < this._uniformDeltas.length; index++)
            uniform[index] += this._uniformDeltas[index];

        const waveform = arterialWaveform(
            this.flowNetwork.time,
            this.flowNetwork.hemodynamics.heartRateBpm
        );
        let minimumSignedFlowMm3PerS = 0;
        let reversedEdgeCount = 0;
        for (let edgeIndex = 0; edgeIndex < uniform.length; edgeIndex++) {
            const override = this.flowNetwork._flowOverridesMm3PerS[edgeIndex];
            if (uniform[edgeIndex] >= 0 && !this._sourceEdgeIndices[edgeIndex] && !(override < 0)) continue;
            const edge = this.flowNetwork.edges[edgeIndex];
            const base = (Number.isFinite(override) ? override : edge.meanFlowMm3PerS * waveform) + uniform[edgeIndex];
            // Only source edges have a different delta on each face. The other
            // cached resistor responses are constant along an entire edge.
            let minimum = base + edge.faceFlowDeltaMm3PerS[0];
            if (this._sourceEdgeIndices[edgeIndex])
                for (let face = 1; face <= edge.cellCount; face++)
                    minimum = Math.min(minimum, base + edge.faceFlowDeltaMm3PerS[face]);
            minimumSignedFlowMm3PerS = Math.min(minimumSignedFlowMm3PerS, minimum);
            if (minimum < 0) reversedEdgeCount++;
        }

        this.active = splits.length > 0;
        this.currentUpstreamFraction =
            weightedUpstreamFraction / Math.max(1e-9, rateMlPerSec);
        this.currentReversedEdgeCount = reversedEdgeCount;
        this.maximumReversedEdgeCount = Math.max(
            this.maximumReversedEdgeCount,
            reversedEdgeCount
        );
        this.maximumUpstreamRateMlPerSec = Math.max(
            this.maximumUpstreamRateMlPerSec,
            this.currentUpstreamRateMlPerSec
        );
        this.maximumDownstreamRateMlPerSec = Math.max(
            this.maximumDownstreamRateMlPerSec,
            this.currentDownstreamRateMlPerSec
        );
        this.minimumSignedFlowMlPerSec =
            minimumSignedFlowMm3PerS / MM3_PER_ML;
        this.configurationCount++;
        return splits;
    }

    _addFlowDelta(
        edgeIndex,
        startFaceIndex,
        endFaceIndexExclusive,
        deltaMm3PerS
    ) {
        this.flowNetwork.addFaceFlowDelta(
            edgeIndex,
            startFaceIndex,
            endFaceIndexExclusive,
            deltaMm3PerS
        );
    }

    _prepareResistances() {
        const net = this.flowNetwork;
        if (this._hydraulicRevision === net.hydraulicRevision) return;
        this._hydraulicRevision = net.hydraulicRevision;
        this._responses.clear();
        this._upstreamResistance.clear();
        // A prescribed cardiac inflow is an open circuit for incremental flow,
        // so an injection cannot eject fluid into the heart through this inlet.
        this._upstreamResistance.set(net.rootNode.id, Infinity);
        for (const id of net.nodeOrder) {
            const node = net.nodes.get(id);
            const parentConductance = conductance(this._upstreamResistance.get(id));
            for (const index of node.childEdgeIndices) {
                let otherConductance = parentConductance;
                for (const sibling of node.childEdgeIndices)
                    if (sibling !== index)
                        otherConductance += conductance(net.edges[sibling].equivalentResistance);
                const proximal = otherConductance > 0 ? 1 / otherConductance : Infinity;
                const edge = net.edges[index];
                this._edgeProximalResistance[index] = proximal;
                this._upstreamResistance.set(edge.endNodeId,
                    edge.transportExcluded ? Infinity : edge.resistance + proximal);
            }
        }
    }

    _sourceResponse(location) {
        this._prepareResistances();
        const key = `${location.edgeIndex}:${location.cellIndex}`;
        if (this._responses.has(key)) return this._responses.get(key);
        const net = this.flowNetwork, source = net.edges[location.edgeIndex];
        const t = (location.cellIndex + 0.5) / source.cellCount;
        const fraction = resistanceFraction(source, t);
        const proximal = this._edgeProximalResistance[source.index] + source.resistance * fraction;
        const distal = net.nodes.get(source.endNodeId).equivalentResistance + source.resistance * (1 - fraction);
        const upG = conductance(proximal), downG = conductance(distal);
        const upstreamFraction = upG + downG > 0 ? upG / (upG + downG) : 0;
        const edgeFractions = new Float64Array(net.edges.length);
        const descend = (nodeId, amount) => {
            const stack = [[nodeId, amount]];
            while (stack.length) {
                const [id, current] = stack.pop(), node = net.nodes.get(id);
                if (!(current > 0) || node.nativeOutlet) continue;
                const totalG = conductance(node.equivalentResistance);
                if (!(totalG > 0)) continue;
                for (const index of node.childEdgeIndices) {
                    const edge = net.edges[index];
                    const part = current * conductance(edge.equivalentResistance) / totalG;
                    if (!(part > 0)) continue;
                    edgeFractions[index] += part;
                    stack.push([edge.endNodeId, part]);
                }
            }
        };
        descend(source.endNodeId, 1 - upstreamFraction);
        let child = source, current = upstreamFraction;
        while (current > 0) {
            const node = net.nodes.get(child.startNodeId);
            const totalG = conductance(this._edgeProximalResistance[child.index]);
            if (!(totalG > 0)) break;
            for (const siblingIndex of node.childEdgeIndices) {
                if (siblingIndex === child.index) continue;
                const sibling = net.edges[siblingIndex];
                const part = current * conductance(sibling.equivalentResistance) / totalG;
                if (!(part > 0)) continue;
                edgeFractions[siblingIndex] += part;
                descend(sibling.endNodeId, part);
            }
            current *= conductance(this._upstreamResistance.get(node.id)) / totalG;
            if (node.parentEdgeIndex < 0 || !(current > 0)) break;
            child = net.edges[node.parentEdgeIndex];
            edgeFractions[child.index] -= current;
        }
        const response = {upstreamFraction, edgeFractions};
        // Moving devices must not accumulate a full-tree response for every
        // visited source cell. Reuse a small working set for multi-hole ports.
        if (this._responses.size >= 32) this._responses.delete(this._responses.keys().next().value);
        this._responses.set(key, response);
        return response;
    }

    _downstreamTargets(sourceEdgeIndex) {
        const edge = this.flowNetwork.edges[sourceEdgeIndex];
        if (!edge?.childEdgeIndices.length) return [];
        const children = edge.childEdgeIndices
            .map(edgeIndex => this.flowNetwork.edges[edgeIndex])
            .filter(Boolean);
        const totalFlow = children.reduce(
            (sum, child) => sum + Math.max(0, child.meanFlowMm3PerS),
            0
        );
        return children.map(child => ({
            edgeIndex: child.index,
            weight: totalFlow > 0
                ? child.meanFlowMm3PerS / totalFlow
                : 1 / children.length
        }));
    }

    getDiagnostics() {
        return {
            active: this.active,
            upstreamRateMlPerSec: this.currentUpstreamRateMlPerSec,
            downstreamRateMlPerSec: this.currentDownstreamRateMlPerSec,
            upstreamFraction: this.currentUpstreamFraction,
            reversedEdgeCount: this.currentReversedEdgeCount,
            maximumReversedEdgeCount: this.maximumReversedEdgeCount,
            maximumUpstreamRateMlPerSec:
                this.maximumUpstreamRateMlPerSec,
            maximumDownstreamRateMlPerSec:
                this.maximumDownstreamRateMlPerSec,
            minimumSignedFlowMlPerSec: this.minimumSignedFlowMlPerSec,
            configurationCount: this.configurationCount
        };
    }
}
