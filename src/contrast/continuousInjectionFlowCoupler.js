import { arterialWaveform } from './flowNetwork.js';

const MM3_PER_ML = 1000;
const MIN_DIRECTIONAL_FRACTION = 0.05;
const MAX_DIRECTIONAL_FRACTION = 0.95;

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
}

function normalizedDirection(direction) {
    const x = direction?.x || 0;
    const y = direction?.y || 0;
    const z = direction?.z || 0;
    const length = Math.hypot(x, y, z) || 1;
    return { x: x / length, y: y / length, z: z / length };
}

function directionDotFlow(direction, location) {
    return direction.x * location.tangentX +
        direction.y * location.tangentY +
        direction.z * location.tangentZ;
}

/**
 * Couples a device outlet to the signed 1D arterial flow without selecting a
 * second contrast algorithm. Injection is represented as a continuous volume
 * source: its proximal share reduces the normal antegrade flow on every
 * ancestor edge and may carry it smoothly through zero, while the distal share
 * augments downstream flow. Iodine enters the same source control volume and
 * the resulting signed face flows divide it conservatively.
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
        this._affectedEdgeIndices = new Set();
        this._locationScratch = {};
    }

    beginStep() {
        this.active = false;
        this.currentUpstreamRateMlPerSec = 0;
        this.currentDownstreamRateMlPerSec = 0;
        this.currentUpstreamFraction = 0;
        this.currentReversedEdgeCount = 0;
        this.minimumSignedFlowMlPerSec = 0;
        this._affectedEdgeIndices.clear();
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

            const direction = normalizedDirection(port.direction);
            const axialAlignment = clamp(
                directionDotFlow(direction, location),
                -1,
                1
            );
            // The outlet direction biases a pressure source but never turns
            // either side completely off. Radial ports split evenly; an axial
            // upstream or downstream outlet approaches a 95/5 split.
            const upstreamFraction = clamp(
                0.5 - axialAlignment * 0.45,
                MIN_DIRECTIONAL_FRACTION,
                MAX_DIRECTIONAL_FRACTION
            );
            const upstreamRateMlPerSec =
                portRateMlPerSec * upstreamFraction;
            const downstreamRateMlPerSec =
                portRateMlPerSec - upstreamRateMlPerSec;
            weightedUpstreamFraction +=
                upstreamFraction * portRateMlPerSec;
            this.currentUpstreamRateMlPerSec += upstreamRateMlPerSec;
            this.currentDownstreamRateMlPerSec += downstreamRateMlPerSec;

            this._addUpstreamDelta(
                location,
                -upstreamRateMlPerSec * MM3_PER_ML
            );
            const downstreamTargets = this._downstreamTargets(
                location.edgeIndex
            );
            this._addDownstreamDelta(
                location,
                downstreamTargets,
                downstreamRateMlPerSec * MM3_PER_ML
            );
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

        const waveform = arterialWaveform(
            this.flowNetwork.time,
            this.flowNetwork.hemodynamics.heartRateBpm
        );
        let minimumSignedFlowMm3PerS = 0;
        let reversedEdgeCount = 0;
        for (const edgeIndex of this._affectedEdgeIndices) {
            const edge = this.flowNetwork.edges[edgeIndex];
            if (!edge) continue;
            let edgeReversed = false;
            for (let faceIndex = 0; faceIndex <= edge.cellCount; faceIndex++) {
                const signedFlowMm3PerS =
                    this.flowNetwork.getFaceSignedFlowMm3PerS(
                        edgeIndex,
                        faceIndex,
                        waveform
                    );
                edgeReversed ||= signedFlowMm3PerS < 0;
                minimumSignedFlowMm3PerS = Math.min(
                    minimumSignedFlowMm3PerS,
                    signedFlowMm3PerS
                );
            }
            if (edgeReversed) reversedEdgeCount++;
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
        const added = this.flowNetwork.addFaceFlowDelta(
            edgeIndex,
            startFaceIndex,
            endFaceIndexExclusive,
            deltaMm3PerS
        );
        if (added) this._affectedEdgeIndices.add(edgeIndex);
    }

    _addUpstreamDelta(location, deltaMm3PerS) {
        let edge = this.flowNetwork.edges[location.edgeIndex];
        if (!edge) return;
        // Face `cellIndex` is the proximal face of the finite volume which
        // contains the outlet. The volume source changes every face from that
        // point back to the arterial root, but leaves distal faces independent.
        this._addFlowDelta(
            edge.index,
            0,
            location.cellIndex + 1,
            deltaMm3PerS
        );
        const visited = new Set();
        edge = edge.parentEdgeIndex >= 0
            ? this.flowNetwork.edges[edge.parentEdgeIndex]
            : null;
        while (edge && !visited.has(edge.index)) {
            visited.add(edge.index);
            this._addFlowDelta(
                edge.index,
                0,
                edge.cellCount + 1,
                deltaMm3PerS
            );
            edge = edge.parentEdgeIndex >= 0
                ? this.flowNetwork.edges[edge.parentEdgeIndex]
                : null;
        }
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

    _addDownstreamDelta(location, targets, addedFlowMm3PerS) {
        const sourceEdge = this.flowNetwork.edges[location.edgeIndex];
        if (!sourceEdge) return;
        this._addFlowDelta(
            sourceEdge.index,
            location.cellIndex + 1,
            sourceEdge.cellCount + 1,
            addedFlowMm3PerS
        );
        const stack = targets.map(target => ({
            edgeIndex: target.edgeIndex,
            addedFlowMm3PerS: addedFlowMm3PerS * target.weight
        }));
        const visited = new Set();
        while (stack.length) {
            const current = stack.pop();
            if (!current || visited.has(current.edgeIndex)) continue;
            visited.add(current.edgeIndex);
            const edge = this.flowNetwork.edges[current.edgeIndex];
            if (!edge) continue;
            this._addFlowDelta(
                edge.index,
                0,
                edge.cellCount + 1,
                current.addedFlowMm3PerS
            );
            if (!edge.childEdgeIndices.length) continue;
            const children = edge.childEdgeIndices
                .map(edgeIndex => this.flowNetwork.edges[edgeIndex])
                .filter(Boolean);
            const totalFlow = children.reduce(
                (sum, child) =>
                    sum + Math.max(0, child.meanFlowMm3PerS),
                0
            );
            for (const child of children) {
                const share = totalFlow > 0
                    ? child.meanFlowMm3PerS / totalFlow
                    : 1 / children.length;
                stack.push({
                    edgeIndex: child.index,
                    addedFlowMm3PerS:
                        current.addedFlowMm3PerS * share
                });
            }
        }
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
