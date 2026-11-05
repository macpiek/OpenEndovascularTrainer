import {sampleFlowConcentration} from './flowConcentrationField.js';
import {junctionStarWeights} from './junctionConcentrationKernel.js';

const GRID_MM = 24;
const smooth = x => { x = Math.max(0, Math.min(1, x)); return x * x * (3 - 2 * x); };

// A sparse reconstruction of scalar concentration in the finite common lumen
// of a bifurcation. All positions and weights are fixed at anatomy load; only
// existing transported concentrations are interpolated during imaging.
export class JunctionSurfaceReconstruction {
    constructor(network, field, territories, continuations, enabled = true) {
        this.network = network; this.field = field; this.offset = field.values?.length ?? 0;
        this.junctions = []; this.grid = new Map(); this.records = [];
        this.updatedSamples = 0; this.activeJunctions = 0;
        if (!enabled || !network.nodes || !field.values) return;
        const walk = (edge, distance, backwards) => {
            let travelled = 0;
            const path = [];
            while (distance > edge.length) {
                path.push({edge: edge.index, start: travelled, length: edge.length});
                distance -= edge.length; travelled += edge.length;
                const next = backwards ? edge.parentEdgeIndex : continuations[edge.index];
                if (next < 0 || (backwards && continuations[next] !== edge.index)) return null;
                edge = network.edges[next];
            }
            path.push({edge: edge.index, start: travelled, length: distance});
            const t = backwards ? 1 - distance / edge.length : distance / edge.length;
            return {point: edge.start.clone().lerp(edge.end, t), edge: edge.index,
                stencil: field.stencil(edge.index, t), travelled: travelled + distance, path, backwards};
        };
        for (const node of network.nodes.values()) {
            const parent = network.edges[node.parentEdgeIndex];
            const children = node.childEdgeIndices.map(i => network.edges[i])
                .filter(e => !e.transportExcluded && !e.lumenMappingExcluded);
            if (!parent || parent.transportExcluded || children.length !== 2) continue;
            const radius = parent.radiusEnd;
            // Sub-pixel twigs and narrow side ostia do not form a broad common
            // chamber. Their existing exact-lumen reconstruction is retained.
            if (radius < 1.5 || children.some(e => e.radiusStart < radius * .35)) continue;
            const reach = radius * 2.5;
            const anchors = [walk(parent, reach, true), ...children.map(e => walk(e, reach, false))];
            // Do not span another true fork: each reconstruction belongs to
            // one connected physical junction, not nearby unrelated branches.
            if (anchors.some(a => !a)) continue;
            const center = node.point.clone();
            for (const anchor of anchors) anchor.direction = anchor.point.clone().sub(center).normalize();
            const axis = anchors[0].direction;
            const junction = {index: this.junctions.length, center, radius, axis, anchors,
                territories: new Set([parent, ...children].map(e => territories[e.index])),
                edges: [parent, ...children].map(e => e.index),
                boundary: [field.offsets[parent.index] + parent.cellCount - 1,
                    ...children.map(e => field.offsets[e.index])],
                nodeSample: field.end[parent.index], dependencies: new Set(), records: [], wasActive: false};
            for (const anchor of anchors) for (const i of [anchor.stencil[0], anchor.stencil[1], anchor.stencil[3]])
                if (i >= 0) junction.dependencies.add(i);
            junction.dependencies.add(junction.nodeSample);
            for (const i of junction.boundary) junction.dependencies.add(i);
            this.junctions.push(junction);
            const limit = radius * 4;
            for (let x = Math.floor((center.x - limit) / GRID_MM); x <= Math.floor((center.x + limit) / GRID_MM); x++)
                for (let y = Math.floor((center.y - limit) / GRID_MM); y <= Math.floor((center.y + limit) / GRID_MM); y++)
                    for (let z = Math.floor((center.z - limit) / GRID_MM); z <= Math.floor((center.z + limit) / GRID_MM); z++) {
                        const key = `${x},${y},${z}`;
                        if (!this.grid.has(key)) this.grid.set(key, []);
                        this.grid.get(key).push(junction);
                    }
        }
        this.territories = territories;
    }
    addSample(point, edgeIndex, stencil) {
        const key = `${Math.floor(point.x / GRID_MM)},${Math.floor(point.y / GRID_MM)},${Math.floor(point.z / GRID_MM)}`;
        const candidates = this.grid.get(key);
        if (!candidates) return -1;
        const influences = [];
        for (const junction of candidates) {
            if (!junction.territories.has(this.territories[edgeIndex])) continue;
            const dx = point.x - junction.center.x, dy = point.y - junction.center.y, dz = point.z - junction.center.z;
            const axial = dx * junction.axis.x + dy * junction.axis.y + dz * junction.axis.z;
            const transverse2 = Math.max(0, dx * dx + dy * dy + dz * dz - axial * axial);
            // Cover the wide radial wall of the actual common lumen without
            // extending the core equally far upstream along the aorta.
            const metric = Math.sqrt((axial / (junction.radius * 1.2)) ** 2 + transverse2 / (junction.radius * 2) ** 2);
            const weight = 1 - smooth(metric - 1);
            if (weight > 0) {
                const local = junction.anchors.map((anchor, arm) => {
                    let projected = dx * anchor.direction.x + dy * anchor.direction.y + dz * anchor.direction.z;
                    // The proximal extension of a daughter is the connected
                    // parent blood column, not a plateau of node mixture.
                    // Keep the signed distance continuous at the shared node.
                    if (arm > 0 && projected < 0 && axial > 0) {
                        anchor = junction.anchors[0]; projected = -projected;
                    }
                    const distance = Math.max(0, Math.min(anchor.travelled, projected));
                    const part = anchor.path.find(part => distance <= part.start + part.length + 1e-8) ?? anchor.path.at(-1);
                    const edge = this.network.edges[part.edge];
                    const fraction = Math.max(0, Math.min(1, (distance - part.start) / edge.length));
                    const stencil = this.field.stencil(edge.index, anchor.backwards ? 1 - fraction : fraction);
                    // One shared endpoint value belongs to the common blood
                    // volume. The spatial reconstruction, not triangle owner,
                    // now determines its transition to each arm's local cells.
                    if (stencil[0] === junction.nodeSample || stencil[1] === junction.nodeSample) stencil[3] = -1;
                    return stencil;
                });
                // On the upstream side the hub contribution follows the
                // transported parent profile as well. Extending the mixed
                // node value over a radius would manufacture a dense aortic
                // column despite dilute parent blood.
                let hub = [junction.nodeSample, junction.nodeSample, 0, -1];
                if (axial > 0) {
                    const anchor = junction.anchors[0], distance = Math.min(anchor.travelled, axial);
                    const part = anchor.path.find(part => distance <= part.start + part.length + 1e-8) ?? anchor.path.at(-1);
                    const edge = this.network.edges[part.edge];
                    hub = this.field.stencil(edge.index, 1 - Math.max(0, Math.min(1, (distance - part.start) / edge.length)));
                    if (hub[0] === junction.nodeSample || hub[1] === junction.nodeSample) hub[3] = -1;
                }
                local.unshift(hub);
                influences.push({junction: junction.index, blend: weight, local,
                    weights: Array.from(junctionStarWeights(point, junction.center, junction.anchors.map(a => a.point)))});
            }
        }
        if (!influences.length) return -1;
        const index = this.records.length;
        this.records.push({influences, stencil: Array.from(stencil)});
        for (const influence of influences) {
            const junction = this.junctions[influence.junction];
            for (const s of [stencil, ...influence.local])
                for (const i of [s[0], s[1], s[3]]) if (i >= 0) junction.dependencies.add(i);
        }
        return index;
    }
    finish() {
        const count = this.records.length;
        const contributions = this.records.reduce((n, r) => n + r.influences.length, 0);
        this.weights = new Float32Array(contributions * 4); this.stencils = new Float32Array(count * 4);
        this.ownerCells = new Uint32Array(count);
        this.localStencils = new Float32Array(contributions * 16);
        this.blends = new Float32Array(contributions); this.junctionIndices = new Uint16Array(contributions);
        this.starts = new Uint32Array(count + 1);
        let contribution = 0;
        this.records.forEach((record, i) => {
            this.stencils.set(record.stencil, i * 4); this.starts[i] = contribution;
            const [a, b, t, owner] = record.stencil;
            this.ownerCells[i] = owner >= 0 ? owner : a >= this.field.cellCount ? b :
                b >= this.field.cellCount ? a : t < .5 ? a : b;
            for (const influence of record.influences) {
                this.weights.set(influence.weights, contribution * 4);
                for (let arm = 0; arm < 4; arm++) this.localStencils.set(influence.local[arm], contribution * 16 + arm * 4);
                this.blends[contribution] = influence.blend;
                this.junctionIndices[contribution++] = influence.junction;
            }
        });
        this.starts[count] = contribution;
        this.sampleCount = count;
        this.records = null; this.grid = null; this.territories = null;
        for (const junction of this.junctions) {
            junction.dependencies = Uint32Array.from(junction.dependencies);
            junction.samples = new Float32Array(4); junction.update = false;
            delete junction.records;
        }
        this.memoryBytes = this.weights.byteLength + this.stencils.byteLength + this.localStencils.byteLength + this.blends.byteLength +
            this.junctionIndices.byteLength + this.starts.byteLength + this.ownerCells.byteLength +
            this.junctions.reduce((sum, j) => sum + j.dependencies.byteLength + j.samples.byteLength, 0);
    }
    addChunkDependencies(sampleIndex, edges) {
        for (let k = this.starts[sampleIndex]; k < this.starts[sampleIndex + 1]; k++) {
            const j = this.junctions[this.junctionIndices[k]];
            for (const anchor of j.anchors) for (const part of anchor.path) edges.add(part.edge);
            for (const edge of j.edges) edges.add(edge);
        }
    }
    update(values, excludedCells) {
        this.updatedSamples = 0; this.activeJunctions = 0;
        let anyUpdate = false;
        for (const junction of this.junctions) {
            const active = junction.dependencies.some(i => values[i] > 0);
            junction.update = active || junction.wasActive;
            junction.wasActive = active;
            if (!junction.update) continue;
            anyUpdate = true;
            junction.samples[0] = Math.max(0, values[junction.nodeSample]);
            if (active) this.activeJunctions++;
        }
        if (!anyUpdate) return;
        for (let i = 0; i < this.sampleCount; i++) {
            // Explicit fabric/removed-lumen masks differ from clear blood.
            // Reconstruction may mix clear blood in a common lumen, but must
            // never recolour a cell deliberately replaced by graft material.
            if (excludedCells?.[this.ownerCells[i]]) { values[this.offset + i] = 0; continue; }
            let affected = false;
            for (let k = this.starts[i]; k < this.starts[i + 1]; k++)
                affected ||= this.junctions[this.junctionIndices[k]].update;
            if (!affected) continue;
            const offset = i * 4;
            const own = sampleFlowConcentration(values, this.stencils[offset], this.stencils[offset + 1],
                this.stencils[offset + 2], this.stencils[offset + 3]);
            let weighted = 0, sum = 0, remaining = 1;
            for (let k = this.starts[i]; k < this.starts[i + 1]; k++) {
                const junction = this.junctions[this.junctionIndices[k]];
                const blend = this.blends[k];
                let spatial = 0;
                for (let arm = 0; arm < 4; arm++) {
                    const o = k * 16 + arm * 4;
                    const local = sampleFlowConcentration(values, this.localStencils[o], this.localStencils[o + 1],
                        this.localStencils[o + 2], this.localStencils[o + 3]);
                    spatial += local * this.weights[k * 4 + arm];
                }
                weighted += spatial * blend; sum += blend; remaining *= 1 - blend;
            }
            // A positive partition handles overlapping finite junctions. No
            // nearest-junction selection can introduce a new ownership seam.
            values[this.offset + i] = sum > 0 ? own * remaining + weighted / sum * (1 - remaining) : own;
            this.updatedSamples++;
        }
    }
}
