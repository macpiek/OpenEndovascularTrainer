import * as THREE from 'three';
import {createFlowColumnRayCandidates} from './flowColumnRayCandidates.js';
import {createColumnJunctionMixing} from './columnJunctionMixing.js';

function makeTexture(records) {
    const width = Math.min(2048, Math.max(1, Math.ceil(Math.sqrt(records.length / 4))));
    const height = Math.max(1, Math.ceil(records.length / 4 / width));
    const values = new Float32Array(width * height * 4); values.set(records);
    const result = new THREE.DataTexture(values, width, height, THREE.RGBAFormat, THREE.FloatType);
    result.minFilter = result.magFilter = THREE.NearestFilter;
    result.generateMipmaps = false; result.needsUpdate = true;
    return result;
}

// Geometry-local concentration support. Surface territory labels cannot decide
// whether a visible ray contains a daughter: a complete small ostium may have
// both boundary triangles assigned to its larger parent.
export function createFlowJunctionProjection(network, integral, territories, spatial, junctionField = null) {
    const shared = createColumnJunctionMixing(network, integral, territories, junctionField);
    const components = new Int32Array(network.edges.length).fill(-1);
    const maxRadii = new Float32Array(network.edges.length);
    for (const edge of network.edges) if (territories[edge.index] >= 0) {
        const path = territories[edge.index];
        maxRadii[path] = Math.max(maxRadii[path], edge.radiusStart, edge.radiusEnd);
    }
    // A conservative rounding allowance keeps the lower score bound valid
    // for interpolated Float32 geometry on the GPU as well as the CPU.
    for (const path of integral.ranges.keys()) maxRadii[path] += Math.max(1e-6, maxRadii[path] * 1e-6);
    function component(path) {
        if (components[path] >= 0) return components[path];
        const parent = network.edges[path].parentEdgeIndex;
        return components[path] = parent < 0 || territories[parent] < 0 ? path : component(territories[parent]);
    }
    for (const path of integral.ranges.keys()) component(path);
    const junctions = [], leaves = new Array(network.edges.length * 4).fill(0);
    const eligible = edge => edge && !edge.transportExcluded && !edge.lumenMappingExcluded && territories[edge.index] >= 0;
    for (const node of network.nodes?.values?.() ?? []) {
        const parent = network.edges[node.parentEdgeIndex];
        if (!eligible(parent)) continue;
        const arms = [parent, ...node.childEdgeIndices.map(index => network.edges[index]).filter(eligible)];
        const seeds = new Map();
        for (const arm of arms) seeds.set(territories[arm.index], integral.edgeStart[arm.index] + (arm === parent ? arm.length : 0));
        if (seeds.size < 2) continue;
        const radius = Math.max(...arms.map(edge => Math.max(edge.radiusStart, edge.radiusEnd)));
        const reach = 3 * radius + .25, center = node.point ?? parent.end;
        const first = leaves.length / 4, root = component(territories[parent.index]);
        leaves.push(seeds.size, root, radius, (shared.indexByParent.get(parent.index) ?? -1) + 1);
        for (const [path, s] of seeds) leaves.push(path, s, 0, 0);
        junctions.push({min: [center.x - reach, center.y - reach, center.z - reach],
            max: [center.x + reach, center.y + reach, center.z + reach], first, root, seeds});
    }
    const nodes = [];
    function build(items) {
        const index = nodes.length, min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
        for (const item of items) for (let axis = 0; axis < 3; axis++) {
            min[axis] = Math.min(min[axis], item.min[axis]); max[axis] = Math.max(max[axis], item.max[axis]);
        }
        const node = {min, max, escape: -1, first: -1}; nodes.push(node);
        if (items.length === 1) node.first = items[0].first;
        else {
            let axis = 0;
            for (let k = 1; k < 3; k++) if (max[k] - min[k] > max[axis] - min[axis]) axis = k;
            items.sort((a, b) => a.min[axis] + a.max[axis] - b.min[axis] - b.max[axis]);
            const middle = items.length >> 1; build(items.slice(0, middle)); build(items.slice(middle));
        }
        node.escape = nodes.length;
        return index;
    }
    if (junctions.length) build(junctions.slice());
    const bounds = new Float32Array(Math.max(1, nodes.length) * 8);
    nodes.forEach((node, i) => bounds.set([...node.min, node.escape, ...node.max, node.first], i * 8));
    // One static reference list per territory gives a unique contribution
    // even when the aorta occurs in several overlapping junctions.
    const references = new Map();
    nodes.forEach((node, index) => {
        if (node.first < 0) return;
        const count = leaves[node.first * 4];
        for (let incident = 0; incident < count; incident++) {
            const record = node.first + 1 + incident, path = leaves[record * 4];
            if (!references.has(path)) references.set(path, []);
            references.get(path).push([index, record, leaves[record * 4 + 1], leaves[node.first * 4 + 2]]);
        }
    });
    for (const [path, refs] of references) {
        leaves[path * 4] = leaves.length / 4; leaves[path * 4 + 1] = refs.length;
        for (const reference of refs) leaves.push(...reference);
    }
    const rayCandidates = createFlowColumnRayCandidates(spatial);
    const rayNodeOffset = leaves.length / 4;
    const sharedOffset = (leaves.length + rayCandidates.data.length) / 4;
    const combined = new Float32Array(leaves.length + rayCandidates.data.length + shared.records.length);
    combined.set(leaves); combined.set(rayCandidates.data, leaves.length);
    combined.set(shared.records, sharedOffset * 4);
    const boundsTexture = makeTexture(bounds), recordsTexture = makeTexture(combined);
    const records = recordsTexture.image.data;
    const connected = (first, a, b) => records[first * 4 + 1] === components[a] || records[first * 4 + 1] === components[b];
    function segmentHits(entry, exit, node) {
        let lo = 0, hi = 1;
        for (let axis = 0; axis < 3; axis++) {
            const start = entry.getComponent(axis), direction = exit.getComponent(axis) - start;
            if (Math.abs(direction) < 1e-10) {if (start < node.min[axis] || start > node.max[axis]) return false;}
            else {
                const a = (node.min[axis] - start) / direction, b = (node.max[axis] - start) / direction;
                lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b));
                if (hi < lo) return false;
            }
        }
        return true;
    }
    function hasSupport(entry, exit, a, b = a) {
        for (let index = 0; index < nodes.length;) {
            const node = nodes[index];
            if (!segmentHits(entry, exit, node)) {index = node.escape; continue;}
            if (node.first >= 0 && connected(node.first, a, b)) return true;
            index++;
        }
        return false;
    }
    function rayCache(entry, exit, a, b = a, limit = 16) {
        const junctionNodes = [];
        for (let index = 0; index < nodes.length;) {
            const node = nodes[index];
            if (!segmentHits(entry, exit, node)) {index = node.escape; continue;}
            if (node.first >= 0 && connected(node.first, a, b)) {
                junctionNodes.push(index);
                if (junctionNodes.length > limit) return {valid: false};
            }
            index++;
        }
        const candidates = rayCandidates.query(entry, exit, {limit, accept: path =>
            path !== a && path !== b && (components[path] === components[a] || components[path] === components[b])});
        return {valid: !candidates.overflow, junctionNodes, paths: candidates.paths};
    }
    const sample = {};
    const taper = (point, node) => {
        let inside = Infinity;
        for (let axis = 0; axis < 3; axis++) inside = Math.min(inside,
            point.getComponent(axis) - node.min[axis], node.max[axis] - point.getComponent(axis));
        const radius = records[node.first * 4 + 2];
        const u = Math.max(0, Math.min(1, inside / Math.max(radius, 1e-8)));
        return u * u * (3 - 2 * u);
    };
    function pathInfo(path, point, fallback) {
        const first = records[path * 4], count = records[path * 4 + 1];
        let alpha = 0, firstActive = Infinity, firstDistance = Infinity, secondDistance = Infinity;
        let seed = fallback, secondSeed = fallback;
        for (let i = 0; i < count; i++) {
            const at = (first + i) * 4, node = nodes[records[at]], weight = taper(point, node);
            if (weight > 0) {alpha = Math.max(alpha, weight); firstActive = Math.min(firstActive, records[at + 1]);}
            let distance = 0;
            for (let axis = 0; axis < 3; axis++) distance += (point.getComponent(axis) - .5 * (node.min[axis] + node.max[axis])) ** 2;
            if (distance < firstDistance) {
                secondDistance = firstDistance; secondSeed = seed; firstDistance = distance; seed = records[at + 2];
            } else if (distance < secondDistance) {secondDistance = distance; secondSeed = records[at + 2];}
        }
        if (count < 2) secondSeed = seed;
        return {alpha, firstActive, seed, secondSeed};
    }
    function pointValue(point, a, sa, b, sb, result = null, cache = null) {
        const choices = new Map([[a, {seed: sa, own: true}]]);
        if (b !== a) choices.set(b, {seed: sb, own: true});
        let beta = 0, localRadius = 0, sharedWeighted = 0, sharedTotal = 0, remaining = 1;
        let cursor = 0;
        for (let index = 0; cache?.valid ? cursor < cache.junctionNodes.length : index < nodes.length;) {
            if (cache?.valid) index = cache.junctionNodes[cursor++];
            const node = nodes[index];
            if ([0, 1, 2].some(axis => point.getComponent(axis) < node.min[axis] || point.getComponent(axis) > node.max[axis])) {
                index = node.escape; continue;
            }
            if (node.first >= 0 && connected(node.first, a, b)) {
                const weight = taper(point, node);
                if (weight > 0) {
                    beta = Math.max(beta, weight); localRadius = Math.max(localRadius, weight * records[node.first * 4 + 2]);
                    const common = records[node.first * 4 + 3] - 1;
                    if (common >= 0 && shared.hasPath(common, a, b)) {
                        const value = shared.sample(common, point), amount = weight * value.blend;
                        sharedWeighted += amount * value.value; sharedTotal += amount; remaining *= 1 - amount;
                    }
                    const count = records[node.first * 4];
                    for (let incident = 0; incident < count; incident++) {
                        const at = (node.first + 1 + incident) * 4, path = records[at];
                        if ((!cache?.valid || cache.paths.has(path)) && !choices.has(path)) choices.set(path, {seed: records[at + 1], own: false});
                    }
                }
            }
            index++;
        }
        if (remaining === 0) {
            if (result) {result.owner = -1; result.support = beta;}
            return sharedWeighted / sharedTotal;
        }
        const width = Math.max(.08, Math.min(1.2, .2 * localRadius));
        let best = Infinity, owner = a;
        for (const [path, choice] of choices) {
            const info = pathInfo(path, point, choice.seed); let alpha = choice.own ? 1 : info.alpha;
            if (alpha <= 0) {choice.effective = Infinity; continue;}
            spatial.nearest(path, point, info.seed, 6, sample);
            let distance = point.distanceToSquared(sample.point), coordinate = sample.s, score = sample.score, localRadius = sample.radius;
            if (info.secondSeed !== info.seed) {
                spatial.nearest(path, point, info.secondSeed, 6, sample);
                const next = point.distanceToSquared(sample.point);
                if (next < distance || (next === distance && sample.s < coordinate)) {coordinate = sample.s; score = sample.score; localRadius = sample.radius;}
            }
            if (!choice.own) {
                const fade = Math.min(.5 * width, .2 * Math.max(localRadius, 1e-4));
                const u = Math.max(0, Math.min(1, -score / fade));
                alpha *= u * u * (3 - 2 * u);
            }
            choice.effective = alpha > 0 ? score - width * Math.log(alpha) : Infinity; choice.coordinate = coordinate;
            if (choice.effective < best || (choice.effective === best && path < owner)) {best = choice.effective; owner = path;}
        }
        let weighted = 0, total = 0;
        for (const [path, choice] of choices) {
            const u = Math.max(0, Math.min(1, 1 - (choice.effective - best) / width));
            const weight = u * u * (3 - 2 * u);
            if (weight > 0) {weighted += weight * integral.sample(path, choice.coordinate); total += weight;}
        }
        if (result) {result.owner = owner; result.support = beta;}
        const baseline = total > 0 ? weighted / total : 0;
        return sharedTotal > 0 ? baseline * remaining + sharedWeighted / sharedTotal * (1 - remaining) : baseline;
    }
    function average(a, sa, b, sb, entry, exit) {
        let baseline;
        const baseTotal = () => baseline ??= a === b ? integral.average(a, sa, sb) : spatial.spatialAverage(a, sa, b, sb, entry, exit);
        if (!hasSupport(entry, exit, a, b)) return baseTotal();
        const cache = rayCache(entry, exit, a, b);
        const point = new THREE.Vector3();
        const evaluate = t => {
            point.copy(entry).lerp(exit, t);
            const state = {}, concentration = pointValue(point, a, sa, b, sb, state, cache);
            return [concentration, state.owner, state.support];
        };
        const base = (lo, hi) => a === b ? integral.average(a, sa + (sb - sa) * lo, sa + (sb - sa) * hi) : baseTotal();
        let sum = 0, previous = evaluate(0);
        for (let i = 1; i <= 24; i++) {
            const current = evaluate(i / 24);
            const stack = [[(i - 1) / 24, i / 24, previous, current, 0]];
            while (stack.length) {
                const [lo, hi, low, high, depth] = stack.pop();
                if (depth === 4 || low[1] === high[1] || low[0] === high[0]) {
                    const beta = .5 * (low[2] + high[2]);
                    sum += (hi - lo) * ((beta < 1 ? (1 - beta) * base(lo, hi) : 0) + .5 * (low[2] * low[0] + high[2] * high[0]));
                    continue;
                }
                const middle = .5 * (lo + hi), sample = evaluate(middle);
                stack.push([middle, hi, sample, high, depth + 1], [lo, middle, low, sample, depth + 1]);
            }
            previous = current;
        }
        return sum;
    }
    return {junctions, nodes, components, maxRadii, boundsTexture, recordsTexture, hasSupport, pointValue, average,
        rayCandidates, rayNodeOffset, rayCache, shared, sharedOffset,
        memoryBytes: boundsTexture.image.data.byteLength + recordsTexture.image.data.byteLength + shared.memoryBytes,
        dispose() {boundsTexture.dispose(); recordsTexture.dispose(); shared.dispose();}};
}
