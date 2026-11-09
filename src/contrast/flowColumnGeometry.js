import * as THREE from 'three';

// Static geometry only: concentration still comes from the conservative flow
// field. Uniform arclength addressing makes a local spatial query independent
// of the size of the vascular graph.
export function createFlowColumnGeometry(network, integral, territories, continuations) {
    const ranges = new Map(), records = [];
    const point = new THREE.Vector3();
    for (const [path, column] of integral.ranges) {
        let radius = Infinity;
        for (let edge = network.edges[path]; edge; edge = network.edges[continuations[edge.index]])
            radius = Math.min(radius, edge.radiusStart, edge.radiusEnd);
        const count = Math.max(2, Math.ceil(column.length / Math.min(1, Math.max(.1, radius * .5))) + 1);
        const step = column.length / (count - 1), start = records.length / 4;
        ranges.set(path, {start, count, length: column.length, step});
        let edge = network.edges[path];
        for (let i = 0; i < count; i++) {
            const s = i * step;
            while (continuations[edge.index] >= 0 && s > integral.edgeStart[edge.index] + edge.length)
                edge = network.edges[continuations[edge.index]];
            const t = Math.max(0, Math.min(1, (s - integral.edgeStart[edge.index]) / Math.max(edge.length, 1e-12)));
            point.copy(edge.start).lerp(edge.end, t);
            records.push(point.x, point.y, point.z, edge.radiusStart + (edge.radiusEnd - edge.radiusStart) * t);
        }
    }
    const width = Math.min(2048, Math.max(1, Math.ceil(Math.sqrt(records.length / 4))));
    const height = Math.max(1, Math.ceil(records.length / 4 / width));
    const data = new Float32Array(width * height * 4); data.set(records);
    const texture = new THREE.DataTexture(data, width, height, THREE.RGBAFormat, THREE.FloatType);
    texture.minFilter = texture.magFilter = THREE.NearestFilter;
    texture.generateMipmaps = false; texture.needsUpdate = true;
    function sample(path, s, out = {}) {
        const range = ranges.get(path);
        if (!range) return null;
        const x = Math.max(0, Math.min(range.count - 1, s / range.step));
        const index = Math.min(range.count - 2, Math.floor(x)), fraction = x - index;
        const at = (range.start + index) * 4;
        out.point ??= new THREE.Vector3(); out.derivative ??= new THREE.Vector3();
        out.point.set(data[at], data[at + 1], data[at + 2]);
        out.derivative.set(data[at + 4] - data[at], data[at + 5] - data[at + 1], data[at + 6] - data[at + 2]);
        out.point.addScaledVector(out.derivative, fraction);
        out.derivative.divideScalar(range.step);
        out.radius = data[at + 3] + fraction * (data[at + 7] - data[at + 3]);
        out.radiusDerivative = (data[at + 7] - data[at + 3]) / range.step;
        return out;
    }
    const local = {}, delta = new THREE.Vector3();
    function nearest(path, p, seed, iterations = 3, out = {}) {
        const range = ranges.get(path);
        let s = Math.max(0, Math.min(range.length, seed)), bestS = s, bestDistance = Infinity;
        // A Newton step may cross a polyline corner and oscillate. Retain the
        // exact closest point on every visited atlas segment, not merely the
        // final iterate (whose side of the corner depends on iteration parity).
        for (let i = 0; i <= iterations; i++) {
            sample(path, s, local);
            const next = s + delta.subVectors(p, local.point).dot(local.derivative) / Math.max(1e-8, local.derivative.lengthSq());
            const segment = Math.min(range.count - 2, Math.floor(s / range.step));
            const candidate = Math.max(segment * range.step, Math.min((segment + 1) * range.step, next));
            const distance = delta.copy(local.point).addScaledVector(local.derivative, candidate - s).distanceToSquared(p);
            if (distance < bestDistance || (distance === bestDistance && candidate < bestS)) {bestDistance = distance; bestS = candidate;}
            const bounded = Math.max(0, Math.min(range.length, next));
            if (bounded === s) break;
            s = bounded;
        }
        sample(path, bestS, out); out.s = bestS;
        out.score = p.distanceTo(out.point) - out.radius;
        return out;
    }
    const depths = new Map();
    const parentOf = path => {
        const edge = network.edges[path], parent = edge.parentEdgeIndex;
        return parent < 0 ? null : {path: territories[parent], s: integral.edgeStart[parent] + network.edges[parent].length};
    };
    function depth(path) {
        if (depths.has(path)) return depths.get(path);
        const parent = parentOf(path);
        const value = !parent || parent.path < 0 ? 0 : depth(parent.path) + 1;
        depths.set(path, value); return value;
    }
    function candidates(a, sa, b, sb) {
        const result = [{path: a, s: sa}];
        if (b !== a) result.push({path: b, s: sb});
        for (let step = 0; step < 64 && a !== b; step++) {
            const stepA = depth(a) >= depth(b), parent = parentOf(stepA ? a : b);
            if (!parent || parent.path < 0) break;
            const found = result.find(item => item.path === parent.path);
            if (found) found.s = .5 * (found.s + parent.s);
            else if (result.length < 16) result.push({...parent});
            else result[15] = {...parent};
            if (stepA) {a = parent.path; sa = parent.s;} else {b = parent.path; sb = parent.s;}
        }
        return result;
    }
    function spatialAverage(a, sa, b, sb, entry, exit) {
        const choices = candidates(a, sa, b, sb), midpoint = new THREE.Vector3().copy(entry).lerp(exit, .5);
        const p = new THREE.Vector3(), selected = {};
        for (const candidate of choices) {
            candidate.s = nearest(candidate.path, midpoint, candidate.s, 3, selected).s;
            candidate.s = nearest(candidate.path, midpoint, candidate.s, 3, selected).s;
        }
        let sum = 0;
        for (let i = 0; i < 24; i++) {
            p.copy(entry).lerp(exit, (i + .5) / 24);
            let score = Infinity, path = choices[0].path, coordinate = 0;
            for (const candidate of choices) {
                nearest(candidate.path, p, candidate.s, 3, selected);
                if (selected.score < score || (selected.score === score && candidate.path < path)) {
                    score = selected.score; path = candidate.path; coordinate = selected.s;
                }
            }
            sum += integral.sample(path, coordinate);
        }
        return sum / 24;
    }
    return {ranges, texture, data, sample, nearest, candidates, spatialAverage,
        memoryBytes: data.byteLength, dispose() {texture.dispose();}};
}
