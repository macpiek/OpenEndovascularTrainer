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
        this.offsets = new Uint32Array(edges.length);
        let cellCount = 0;
        for (const edge of edges) {
            this.offsets[edge.index] = cellCount;
            cellCount += edge.cellCount;
        }
        this.cellCount = cellCount;
        this.touchedEdges = [];
        this.edgeMarks = new Uint8Array(edges.length);
        const count = cellCount + nodes.length;
        const nodeIndex = new Map(nodes.map((node, i) => [node.id, cellCount + i]));
        this.open = new Uint8Array(count);
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
            edges = net.edges;
        let topologyChanged = false;
        this.outflow.fill(0);
        let link = 0;
        for (const edge of edges) {
            // Resolve the shared arterial/override term once per edge. Faces
            // are already in range and ordered when the topology is compiled.
            const delta = edge.faceFlowDeltaMm3PerS;
            const override = net._flowOverridesMm3PerS[edge.index];
            const base = Number.isFinite(override) ? override : edge.meanFlowMm3PerS * waveform;
            for (let face = 0; face <= edge.cellCount; face++, link++) {
                const q = edge.transportExcluded ? 0 : base + delta[face];
                const sign = Math.sign(q),
                    magnitude = Math.abs(q);
                if (sign !== this.signs[link]) {
                    this.signs[link] = sign;
                    topologyChanged = true;
                }
                this.flows[link] = magnitude;
                if (sign) this.outflow[sign > 0 ? this.from[link] : this.to[link]] += magnitude;
            }
        }
        if (topologyChanged) this._orderByFlow();
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
        this.touchedEdges.length = 0;
        for (const edgeIndex of net._activeEdgeIndices) {
            const edge = edges[edgeIndex],
                offset = this.offsets[edgeIndex];
            for (let i = 0; i < edge.cellCount; i++) this.mass[offset + i] = edge.massMg[i];
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
        }
        const sac = net.stentGraftRemodeling?.sac;
        for (const vertex of this.order) {
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
                    }
                    const targetEvents = this.events[target];
                    for (let i = 0; i < outgoing.length; i += 3)
                        targetEvents.push(outgoing[i], outgoing[i + 1], outgoing[i + 2] * fraction);
                    this.linkMass[link] = amount;
                }
            } else if (this.open[vertex]) net.outletIodineMassMg += outgoingMass;
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
