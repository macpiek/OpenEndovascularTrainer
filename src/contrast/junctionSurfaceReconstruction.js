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
            let travelled = 0, supportLength = 0;
            const path = [];
            while (distance > edge.length) {
                const next = backwards ? edge.parentEdgeIndex : continuations[edge.index];
                const bridge = backwards && edge.intraluminalAorticConnector && !edge.transportExcluded;
                if (next < 0 || (backwards && !bridge && continuations[next] !== edge.index)) {
                    // Nearby arch ostia often leave less than 2.5 radii of
                    // uninterrupted artery. Keep the local arm instead of
                    // dropping the entire common-lumen field. Stop at its
                    // last cell centre: sampling the next fork's endpoint
                    // would borrow its mixture before contrast arrives here.
                    distance = Math.max(0, edge.length - edge.cellLength * .5);
                    supportLength = travelled + edge.length;
                    break;
                }
                path.push({edge: edge.index, start: travelled, length: edge.length});
                distance -= edge.length; travelled += edge.length;
                edge = network.edges[next];
            }
            path.push({edge: edge.index, start: travelled, length: distance});
            const t = backwards ? 1 - distance / edge.length : distance / edge.length;
            const optical = path.find(part => territories[part.edge] >= 0);
            return {point: edge.start.clone().lerp(edge.end, t), edge: edge.index,
                stencil: field.stencil(edge.index, t), travelled: travelled + distance, path, backwards,
                opticalEdgeIndex: optical?.edge ?? -1, opticalStartDistance: optical?.start ?? 0, supportLength};
        };
        for (const node of network.nodes.values()) {
            const parent = network.edges[node.parentEdgeIndex];
            const children = node.childEdgeIndices.map(i => network.edges[i])
                .filter(e => !e.transportExcluded && !e.lumenMappingExcluded);
            if (!parent || parent.transportExcluded ||
                (parent.lumenMappingExcluded && !parent.intraluminalAorticConnector) || children.length !== 2) continue;
            // An unequal-caliber fork still has a common blood volume. Scale
            // that volume with its narrowest daughter instead of removing it
            // at a caliber threshold: otherwise the daughter's wall triangles
            // suddenly lose the neighbouring junction's shared mixture.
            const daughterRadius = Math.min(...children.map(e => e.radiusStart));
            const narrow = daughterRadius < parent.radiusEnd * .35;
            const radius = narrow ? daughterRadius : parent.radiusEnd;
            if (radius < 1.5) continue;
            const reach = radius * 2.5;
            const anchors = [walk(parent, reach, true), ...children.map(e => walk(e, reach, false))];
            // Every arm remains on this fork's connected local path. A
            // transport-only intraluminal bridge can precede its visible
            // parent, but must never create an independent optical lumen.
            if (anchors.some(a => !(a.travelled > 0) || a.opticalEdgeIndex < 0)) continue;
            const center = node.point.clone();
            for (const anchor of anchors) anchor.direction = anchor.point.clone().sub(center).normalize();
            const axis = anchors[0].direction;
            const junction = {index: this.junctions.length, center, radius, axis, anchors, narrow,
                territories: new Set(anchors.map(a => territories[a.opticalEdgeIndex])),
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
        // Closely spaced ostia form overlapping common blood volumes. Their
        // wall labels are interchangeable only when their local transported
        // arms actually overlap along the same edge. Spatial proximity alone
        // must not connect separate crossing arteries.
        const roots = this.junctions.map((_, i) => i);
        const root = i => roots[i] === i ? i : (roots[i] = root(roots[i]));
        const edgeArms = new Map();
        for (const junction of this.junctions) for (const anchor of junction.anchors) for (const part of anchor.path) {
            const length = network.edges[part.edge].length;
            const lo = anchor.backwards ? length - part.length : 0;
            const hi = anchor.backwards ? length : part.length;
            const arms = edgeArms.get(part.edge) ?? [];
            for (const arm of arms) if (Math.max(lo, arm.lo) <= Math.min(hi, arm.hi) + 1e-7)
                roots[root(junction.index)] = root(arm.junction);
            arms.push({lo, hi, junction: junction.index}); edgeArms.set(part.edge, arms);
        }
        const clusterTerritories = new Map();
        for (const junction of this.junctions) {
            const id = root(junction.index), paths = clusterTerritories.get(id) ?? new Set();
            for (const path of junction.territories) paths.add(path);
            clusterTerritories.set(id, paths);
        }
        for (const junction of this.junctions) {
            const paths=clusterTerritories.get(root(junction.index));
            junction.boundedToArms=junction.narrow||paths.size>junction.territories.size;
            junction.territories=paths;
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
            let weight = 1 - smooth(metric - 1);
            for (const anchor of junction.anchors) if (anchor.supportLength > 0) {
                const projected = dx * anchor.direction.x + dy * anchor.direction.y + dz * anchor.direction.z;
                // A short arm must not repeat its final cell beyond the next
                // true fork. Fade its common volume before that boundary;
                // the neighbouring junction supplies its own local field.
                weight *= 1 - smooth(2 * projected / anchor.supportLength - 1);
            }
            if(weight>0&&junction.boundedToArms){
                let coverage=0;
                const feather=Math.max(.08,Math.min(1,junction.radius*.12));
                for(const anchor of junction.anchors){
                    for(const part of anchor.path){
                        const edge=this.network.edges[part.edge],length=edge.length;
                        const lo=anchor.backwards?1-part.length/length:0,hi=anchor.backwards?1:part.length/length;
                        const x=edge.end.x-edge.start.x,y=edge.end.y-edge.start.y,z=edge.end.z-edge.start.z;
                        const t=Math.max(lo,Math.min(hi,((point.x-edge.start.x)*x+(point.y-edge.start.y)*y+(point.z-edge.start.z)*z)/(length*length)));
                        const distance=Math.hypot(point.x-edge.start.x-t*x,point.y-edge.start.y-t*y,point.z-edge.start.z-t*z);
                        const score=distance-(edge.radiusStart+t*(edge.radiusEnd-edge.radiusStart));
                        coverage=Math.max(coverage,1-smooth((score+feather)/feather));
                        if(coverage===1)break;
                    }
                    if(coverage===1)break;
                }
                weight*=coverage;
            }
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
