// Presentation reconstruction on the connected vessel graph. Cell centres
// are samples, not constant-colour pieces of tube; shared endpoints interpolate
// the adjacent centres in physical distance, even across one-cell edges.
export class FlowConcentrationField {
    constructor(network, offsets, cellCount) {
        this.edges = network.edges;
        this.offsets = offsets;
        this.cellCount = cellCount;
        this.cellLengths = new Float64Array(cellCount);
        this.nodes = [...network.nodes.values()];
        const nodeIndices = new Map(this.nodes.map((node, i) => [node.id, cellCount + i]));
        this.values = new Float32Array(cellCount + this.nodes.length);
        this.start = new Uint32Array(this.edges.length);
        this.end = new Uint32Array(this.edges.length);
        this.upstream = new Int32Array(cellCount).fill(-1);
        this.downstream = new Int32Array(cellCount).fill(-1);
        this.branches = [];
        for (const edge of this.edges) {
            const offset = offsets[edge.index];
            this.cellLengths.fill(edge.cellLength, offset, offset + edge.cellCount);
            this.start[edge.index] = nodeIndices.get(edge.startNodeId);
            this.end[edge.index] = nodeIndices.get(edge.endNodeId);
            for (let i = 0; i < edge.cellCount; i++) {
                this.upstream[offset + i] = i
                    ? offset + i - 1
                    : edge.parentEdgeIndex >= 0
                      ? offsets[edge.parentEdgeIndex] +
                        this.edges[edge.parentEdgeIndex].cellCount -
                        1
                      : -1;
                this.downstream[offset + i] =
                    i + 1 < edge.cellCount
                        ? offset + i + 1
                        : edge.childEdgeIndices.length === 1
                          ? offsets[edge.childEdgeIndices[0]]
                          : -1;
            }
            if (edge.childEdgeIndices.length > 1) this.branches.push(edge);
        }
    }

    smooth(source, target, blend) {
        for (let i = 0; i < this.cellCount; i++) {
            target[i] = blend(
                source[i],
                this.upstream[i] < 0 ? 0 : source[this.upstream[i]],
                this.downstream[i] < 0 ? 0 : source[this.downstream[i]]
            );
        }
        for (const edge of this.branches) {
            let concentration = 0,
                flow = 0;
            for (const childIndex of edge.childEdgeIndices) {
                const weight = Math.max(0, this.edges[childIndex].meanFlowMm3PerS);
                concentration += source[this.offsets[childIndex]] * weight;
                flow += weight;
            }
            const i = this.offsets[edge.index] + edge.cellCount - 1;
            target[i] = blend(
                source[i],
                this.upstream[i] < 0 ? 0 : source[this.upstream[i]],
                flow > 0 ? concentration / flow : 0
            );
        }
    }

    update(source) {
        this.values.set(source);
        for (let n = 0; n < this.nodes.length; n++) {
            const node = this.nodes[n];
            let weighted = 0,
                total = 0;
            if (node.parentEdgeIndex >= 0) {
                const edge = this.edges[node.parentEdgeIndex],
                    i = edge.cellCount - 1;
                if (!edge.transportExcluded) {
                    const weight = edge.areas[i] / edge.cellLength;
                    weighted += source[this.offsets[edge.index] + i] * weight;
                    total += weight;
                }
            }
            for (const childIndex of node.childEdgeIndices) {
                const edge = this.edges[childIndex];
                if (edge.transportExcluded) continue;
                const weight = edge.areas[0] / edge.cellLength;
                weighted += source[this.offsets[childIndex]] * weight;
                total += weight;
            }
            this.values[this.cellCount + n] = total > 0 ? weighted / total : 0;
        }
    }

    stencil(edgeIndex, t) {
        const edge = this.edges[edgeIndex],
            offset = this.offsets[edgeIndex];
        const x = Math.max(0, Math.min(1, t)) * edge.cellCount - 0.5;
        // Gate empty arms only at forks. Along an unbranched chain the front
        // must interpolate into the next half-cell, including a zero centre;
        // gating that centre would restore a hard seam at every atlas edge.
        if (x < 0)
            return [
                this.start[edgeIndex],
                offset,
                (x + 0.5) * 2,
                this.nodes[this.start[edgeIndex] - this.cellCount].childEdgeIndices.length > 1
                    ? offset
                    : -1
            ];
        if (x > edge.cellCount - 1)
            return [
                offset + edge.cellCount - 1,
                this.end[edgeIndex],
                (x - edge.cellCount + 1) * 2,
                edge.childEdgeIndices.length > 1 ? offset + edge.cellCount - 1 : -1
            ];
        const lower = Math.floor(x);
        return [offset + lower, offset + Math.min(edge.cellCount - 1, lower + 1), x - lower, -1];
    }

    compileSamples(edges, positions) {
        const lower = new Uint32Array(edges.length),
            upper = lower.slice();
        const fraction = new Float32Array(edges.length),
            owner = new Int32Array(edges.length);
        for (let i = 0; i < edges.length; i++) {
            [lower[i], upper[i], fraction[i], owner[i]] = this.stencil(edges[i], positions[i]);
        }
        return { lower, upper, fraction, owner };
    }

    sample(edgeIndex, t) {
        const [a, b, f, owner] = this.stencil(edgeIndex, t);
        // Do not paint an unperfused side branch from its opacified parent.
        if (owner >= 0 && this.values[owner] === 0) return 0;
        return this.values[a] * (1 - f) + this.values[b] * f;
    }

    sampleInto(stencil, target) {
        const { lower, upper, fraction, owner } = stencil,
            values = this.values;
        for (let i = 0; i < target.length; i++)
            target[i] =
                owner[i] >= 0 && values[owner[i]] === 0
                    ? 0
                    : values[lower[i]] * (1 - fraction[i]) + values[upper[i]] * fraction[i];
    }
}
