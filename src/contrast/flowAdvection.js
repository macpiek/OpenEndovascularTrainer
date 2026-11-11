// Conservative characteristic transport of cell-average iodine. A cell's
// outgoing concentration is its initial stock for V/Q seconds, followed by
// its incoming profile delayed by V/Q. Piecewise-constant time profiles pass
// through zero-volume junctions in flow order, including split and reflux.
// This preserves finite transit time without a smallest-cell CFL restriction.
function integral(pulses) {
    let mass = 0;
    for (let i = 0; i < pulses.length; i += 3) mass += pulses[i + 2] * (pulses[i + 1] - pulses[i]);
    return mass;
}

export class FlowAdvection {
    constructor(network) {
        this.network = network;
        const edges = network.edges,
            nodes = [...network.nodes.values()];
        // Hydraulic redistribution changes these together. Keep the large
        // immutable atlas topology separate from each tick's small source
        // arrays, avoiding scattered object reads for every blood face.
        this.meanFlows = new Float64Array(edges.length);
        this.excluded = new Uint8Array(edges.length);
        this.faceDeltas = edges.map(edge => edge.faceFlowDeltaMm3PerS);
        this.faceCounts = Uint32Array.from(edges, edge => edge.cellCount + 1);
        this.faceDeltaEdges = new Uint8Array(edges.length);
        this.hydraulicRevision = -1;
        this.offsets = new Uint32Array(edges.length);
        let cellCount = 0;
        for (const edge of edges) {
            this.offsets[edge.index] = cellCount;
            cellCount += edge.cellCount;
        }
        this.cellCount = cellCount;
        // Device entrainment is a rate over the next network interval, not
        // an initial cell bolus. Reuse sparse typed storage across updates.
        this.queuedSourceMassMg = new Float64Array(cellCount);
        this.queuedSourceVertices = [];
        this.sourceIntervalSeconds = 0;
        this.touchedEdges = [];
        this.edgeMarks = new Uint8Array(edges.length);
        const count = cellCount + nodes.length;
        const nodeIndex = new Map(nodes.map((node, i) => [node.id, cellCount + i]));
        this.open = new Uint8Array(count);
        this.nodeIds = nodes.map(node => node.id);
        nodes.forEach((node, i) => {
            this.open[cellCount + i] =
                node.parentEdgeIndex < 0 || node.childEdgeIndices.length === 0;
        });
        const from = [],
            to = [];
        for (const edge of edges) {
            const offset = this.offsets[edge.index];
            for (let face = 0; face <= edge.cellCount; face++) {
                from.push(face ? offset + face - 1 : nodeIndex.get(edge.startNodeId));
                to.push(face < edge.cellCount ? offset + face : nodeIndex.get(edge.endNodeId));
            }
        }
        this.from = Uint32Array.from(from);
        this.to = Uint32Array.from(to);
        this.flows = new Float64Array(from.length);
        this.signs = new Int8Array(from.length).fill(2);
        this.outflow = new Float64Array(count);
        this.events = Array.from({ length: count }, () => []);
        this.eventVertices = [];
        this.eventMarks = new Uint8Array(count);
        this.outEvents = [];
        this.mass = new Float64Array(cellCount);
        this.order = new Uint32Array(count);
        this.rank = new Uint32Array(count);
        this.pendingOrder = new Uint32Array(Math.ceil(count / 32));
        this.indegree = new Uint16Array(count);
        this.head = new Int32Array(count);
        this.next = new Int32Array(from.length);
        this.linkMass = new Float64Array(from.length);
        this.cellEdge = new Uint32Array(cellCount);
        this.cellIndex = new Uint32Array(cellCount);
        for (const edge of edges)
            for (let i = 0; i < edge.cellCount; i++) {
                this.cellEdge[this.offsets[edge.index] + i] = edge.index;
                this.cellIndex[this.offsets[edge.index] + i] = i;
            }
    }

    prepare(waveform) {
        const net = this.network,
            flows = this.flows, signs = this.signs,
            outflow = this.outflow, from = this.from, to = this.to,
            overrides = net._flowOverridesMm3PerS, uniform = net._edgeFlowDeltasMm3PerS,
            meanFlows = this.meanFlows, excluded = this.excluded, faceDeltas = this.faceDeltas,
            faceCounts = this.faceCounts, faceDeltaEdges = this.faceDeltaEdges;
        if (this.hydraulicRevision !== net.hydraulicRevision || net.hydraulicRevision === undefined) {
            for (const edge of net.edges) {
                meanFlows[edge.index] = edge.meanFlowMm3PerS;
                excluded[edge.index] = Boolean(edge.transportExcluded);
            }
            this.hydraulicRevision = net.hydraulicRevision;
        }
        // Only actual source edges have per-face corrections. In particular,
        // do not dereference tens of thousands of separate zero arrays.
        faceDeltaEdges.fill(net._faceFlowDeltaEdgeIndices ? 0 : 1);
        for (const index of net._faceFlowDeltaEdgeIndices ?? []) faceDeltaEdges[index] = 1;
        let topologyChanged = false;
        outflow.fill(0);
        let link = 0;
        for (let edgeIndex = 0; edgeIndex < meanFlows.length; edgeIndex++) {
            // Resolve the shared arterial/override term once per edge. Faces
            // are already in range and ordered when the topology is compiled.
            const delta = faceDeltaEdges[edgeIndex] ? faceDeltas[edgeIndex] : null;
            const override = overrides[edgeIndex], blocked = excluded[edgeIndex];
            const base = (Number.isFinite(override) ? override : meanFlows[edgeIndex] * waveform) +
                uniform[edgeIndex];
            const end = link + faceCounts[edgeIndex];
            for (let face = 0; link < end; face++, link++) {
                const q = blocked ? 0 : base + (delta ? delta[face] : 0);
                const sign = Math.sign(q),
                    magnitude = Math.abs(q);
                if (sign !== signs[link]) {
                    signs[link] = sign;
                    topologyChanged = true;
                }
                flows[link] = magnitude;
                if (sign) outflow[sign > 0 ? from[link] : to[link]] += magnitude;
            }
        }
        if (topologyChanged) this._orderByFlow();
    }

    queueSource(edgeIndex, cellIndex, mass) {
        const vertex = this.offsets[edgeIndex] + cellIndex;
        if (!(this.queuedSourceMassMg[vertex] > 0)) this.queuedSourceVertices.push(vertex);
        this.queuedSourceMassMg[vertex] += mass;
    }

    finishSourceInterval() {
        for (const vertex of this.queuedSourceVertices) this.queuedSourceMassMg[vertex] = 0;
        this.queuedSourceVertices.length = 0;
        this.sourceIntervalSeconds = 0;
    }

    update(dt) {
        const net = this.network,
            edges = net.edges;
        for (const vertex of this.eventVertices) this.events[vertex].length = 0;
        this.eventVertices.length = 0;
        this.eventMarks.fill(0);
        this.linkMass.fill(0);
        this.mass.fill(0);
        this.edgeMarks.fill(0);
        this.pendingOrder.fill(0);
        this.touchedEdges.length = 0;
        for (const edgeIndex of net._activeEdgeIndices) {
            const edge = edges[edgeIndex],
                offset = this.offsets[edgeIndex];
            for (let i = 0; i < edge.cellCount; i++) {
                const vertex = offset + i;
                this.mass[vertex] = edge.massMg[i];
                if (edge.massMg[i] > 0) this._queueVertex(vertex);
            }
            this.edgeMarks[edgeIndex] = 1;
            this.touchedEdges.push(edgeIndex);
        }
        for (const edgeIndex of net._branchInletEdgeIndices) {
            const vertex = this.offsets[edgeIndex];
            this.events[vertex].push(
                0,
                dt,
                net._branchInletFlowMm3PerS[edgeIndex] *
                    net._branchInletConcentrationMgPerMm3[edgeIndex]
            );
            this.eventMarks[vertex] = 1;
            this.eventVertices.push(vertex);
            this._queueVertex(vertex);
        }
        if (this.sourceIntervalSeconds > 0) {
            for (const vertex of this.queuedSourceVertices) {
                // A steady source remains present after a high-Courant sweep.
                // Depositing it as initial mass instead created short plugs
                // separated by an empty source neighbourhood every frame.
                this.events[vertex].push(0, dt,
                    this.queuedSourceMassMg[vertex] / this.sourceIntervalSeconds);
                if (!this.eventMarks[vertex]) {
                    this.eventMarks[vertex] = 1;
                    this.eventVertices.push(vertex);
                    this._queueVertex(vertex);
                }
            }
        }
        const sac = net.stentGraftRemodeling?.sac;
        // Traverse the same topological order, but visit only vertices with
        // initial iodine or an arriving time profile. Clear upstream blood is
        // still included in outflow, so junction dilution is unchanged.
        for (let word = 0; word < this.pendingOrder.length; word++) while (this.pendingOrder[word]) {
            const bits = this.pendingOrder[word], bit = 31 - Math.clz32(bits & -bits);
            this.pendingOrder[word] = bits & (bits - 1);
            const vertex = this.order[word * 32 + bit];
            const incoming = this.events[vertex],
                q = this.outflow[vertex];
            let outgoing = incoming,
                outgoingMass;
            if (vertex < this.cellCount) {
                const initial = this.mass[vertex];
                if (!(initial > 0 || incoming.length)) continue;
                const edgeIndex = this.cellEdge[vertex];
                if (!this.edgeMarks[edgeIndex]) {
                    this.edgeMarks[edgeIndex] = 1;
                    this.touchedEdges.push(edgeIndex);
                }
                outgoing = this.outEvents;
                outgoing.length = 0;
                const incomingMass = integral(incoming);
                if (q > 0) {
                    const transit =
                        Math.max(1e-9, edges[edgeIndex].volumes[this.cellIndex[vertex]]) / q;
                    if (initial > 0) {
                        const rate = initial / transit;
                        outgoing.push(0, Math.min(transit, dt), rate);
                    }
                    for (let i = 0; i < incoming.length; i += 3) {
                        const time = incoming[i] + transit;
                        if (time < dt)
                            outgoing.push(
                                time,
                                Math.min(dt, incoming[i + 1] + transit),
                                incoming[i + 2]
                            );
                    }
                }
                outgoingMass = Math.min(initial + incomingMass, integral(outgoing));
                this.mass[vertex] = initial + incomingMass - outgoingMass;
            } else outgoingMass = integral(outgoing);
            if (!(outgoingMass > 0)) continue;
            if (q > 0) {
                for (let link = this.head[vertex]; link >= 0; link = this.next[link]) {
                    const sign = this.signs[link],
                        target = sign > 0 ? this.to[link] : this.from[link];
                    let fraction = this.flows[link] / q,
                        amount = outgoingMass * fraction;
                    if (sac && vertex < this.cellCount) {
                        const diverted = sac.divert(
                            this.cellEdge[vertex],
                            this.cellIndex[vertex],
                            sign,
                            amount
                        );
                        fraction *= 1 - diverted / amount;
                        amount -= diverted;
                    }
                    if (!(amount > 0)) continue;
                    if (!this.eventMarks[target]) {
                        this.eventMarks[target] = 1;
                        this.eventVertices.push(target);
                        this._queueVertex(target);
                    }
                    const targetEvents = this.events[target];
                    for (let i = 0; i < outgoing.length; i += 3)
                        targetEvents.push(outgoing[i], outgoing[i + 1], outgoing[i + 2] * fraction);
                    this.linkMass[link] = amount;
                }
            } else if (this.open[vertex]) {
                const retained = net.receiveOutletIodine?.(this.nodeIds[vertex-this.cellCount],outgoingMass);
                if(!retained) net.outletIodineMassMg += outgoingMass;
            }
            else if (vertex >= this.cellCount) this._returnClosedNodeMass(vertex);
        }
        for (const edgeIndex of this.touchedEdges) {
            const edge = edges[edgeIndex],
                offset = this.offsets[edgeIndex];
            for (let i = 0; i < edge.cellCount; i++) edge.nextMassMg[i] = this.mass[offset + i];
        }
        return this.touchedEdges;
    }

    _orderByFlow() {
        this.head.fill(-1);
        this.next.fill(-1);
        this.indegree.fill(0);
        for (let link = 0; link < this.signs.length; link++) {
            const sign = this.signs[link];
            if (!sign) continue;
            const from = sign > 0 ? this.from[link] : this.to[link],
                to = sign > 0 ? this.to[link] : this.from[link];
            this.indegree[to]++;
            this.next[link] = this.head[from];
            this.head[from] = link;
        }
        let count = 0;
        for (let i = 0; i < this.indegree.length; i++)
            if (!this.indegree[i]) this.order[count++] = i;
        for (let i = 0; i < count; i++)
            for (let link = this.head[this.order[i]]; link >= 0; link = this.next[link]) {
                const target = this.signs[link] > 0 ? this.to[link] : this.from[link];
                if (--this.indegree[target] === 0) this.order[count++] = target;
            }
        if (count !== this.order.length)
            throw new Error('Contrast transport requires an acyclic vessel tree');
        for (let i = 0; i < count; i++) this.rank[this.order[i]] = i;
    }

    _queueVertex(vertex) {
        const rank = this.rank[vertex];
        this.pendingOrder[rank >>> 5] |= 1 << (rank & 31);
    }

    _returnClosedNodeMass(node) {
        // A graph junction has no reservoir. At a temporarily closed internal
        // node return iodine to its donating boundary cells, as before.
        for (let link = 0; link < this.signs.length; link++) {
            const sign = this.signs[link];
            const target = sign > 0 ? this.to[link] : this.from[link];
            if (sign && target === node)
                this.mass[sign > 0 ? this.from[link] : this.to[link]] += this.linkMass[link];
        }
    }
}
