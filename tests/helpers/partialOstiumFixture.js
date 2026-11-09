import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {ContrastFlowNetwork} from '../../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../../src/contrast/flowConcentrationField.js';
import {buildAnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';
import {createFlowColumnIntegral} from '../../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../../src/contrast/flowColumnGeometry.js';

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
function mergeIntervals(intervals) {
    const sorted = intervals.filter(([a, b]) => b > a).sort((a, b) => a[0] - b[0]), result = [];
    for (const interval of sorted) {
        const previous = result.at(-1);
        if (previous && interval[0] <= previous[1]) previous[1] = Math.max(previous[1], interval[1]);
        else result.push([...interval]);
    }
    return result;
}
function intersectIntervals(a, b) {
    const result = [];
    for (const [loA, hiA] of a) for (const [loB, hiB] of b) {
        const lo = Math.max(loA, loB), hi = Math.min(hiA, hiB);
        if (hi > lo) result.push([lo, hi]);
    }
    return mergeIntervals(result);
}
function subtractIntervals(intervals, excluded) {
    let result = intervals.map(interval => [...interval]);
    for (const [cutLo, cutHi] of excluded) {
        const next = [];
        for (const [lo, hi] of result) {
            if (cutHi <= lo || cutLo >= hi) {next.push([lo, hi]); continue;}
            if (lo < cutLo) next.push([lo, cutLo]);
            if (hi > cutHi) next.push([cutHi, hi]);
        }
        result = next;
    }
    return result;
}
const totalLength = intervals => intervals.reduce((sum, [a, b]) => sum + b - a, 0);

function sphereInterval(ray, center, radius) {
    const offset = ray.origin.clone().sub(center), along = offset.dot(ray.direction);
    const discriminant = along * along - offset.lengthSq() + radius * radius;
    return discriminant > 0 ? [[-along - Math.sqrt(discriminant), -along + Math.sqrt(discriminant)]] : [];
}
/** Exact ray intersections with a conservative, constant-radius centerline capsule. */
function capsuleIntervals(ray, edge, radius) {
    const offset = ray.origin.clone().sub(edge.start), u = edge.axis;
    const originAlong = offset.dot(u), directionAlong = ray.direction.dot(u);
    const radialOrigin = offset.clone().addScaledVector(u, -originAlong);
    const radialDirection = ray.direction.clone().addScaledVector(u, -directionAlong);
    const aa = radialDirection.lengthSq(), bb = radialOrigin.dot(radialDirection);
    const cc = radialOrigin.lengthSq() - radius * radius;
    const radial = aa < 1e-14 ? cc <= 0 ? [[-Infinity, Infinity]] : [] :
        bb * bb - aa * cc > 0 ? [[(-bb - Math.sqrt(bb * bb - aa * cc)) / aa,
            (-bb + Math.sqrt(bb * bb - aa * cc)) / aa]] : [];
    const axial = Math.abs(directionAlong) < 1e-14 ?
        originAlong >= 0 && originAlong <= edge.length ? [[-Infinity, Infinity]] : [] :
        [[-originAlong / directionAlong, (edge.length - originAlong) / directionAlong].sort((a, b) => a - b)];
    return mergeIntervals([...intersectIntervals(radial, axial), ...sphereInterval(ray, edge.start, radius),
        ...sphereInterval(ray, edge.end, radius)]);
}

/**
 * Independent physical-space oracle. C is prescribed in world space, not
 * inferred from surface ownership or a graph route between entry and exit.
 * The ramp is linear over [frontY-width, frontY] and saturated below it.
 */
export function slabConcentration(y, frontY = -254.5, width = .6) {
    return clamp((frontY - y) / width, 0, 1);
}
export function spatialSlabIntegral(ray, intervals, frontY = -254.5, width = .6) {
    let integral = 0;
    for (const [lo, hi] of intervals) {
        const breaks = [lo, hi];
        if (Math.abs(ray.direction.y) > 1e-12) for (const y of [frontY - width, frontY]) {
            const t = (y - ray.origin.y) / ray.direction.y;
            if (t > lo && t < hi) breaks.push(t);
        }
        breaks.sort((a, b) => a - b);
        for (let i = 1; i < breaks.length; i++) {
            const a = breaks[i - 1], b = breaks[i];
            integral += .5 * (slabConcentration(ray.origin.y + a * ray.direction.y, frontY, width) +
                slabConcentration(ray.origin.y + b * ray.direction.y, frontY, width)) * (b - a);
        }
    }
    return integral;
}

/** CPU-only diagnostic fixture. JSON is supplied by caller so the geometry can also be reused in browser tests. */
export function partialOstiumFixture(fixture) {
    const {centerline: data, centerlineEdges: links, centerlineStride: stride} = fixture;
    const network = new ContrastFlowNetwork(Array.from({length: links.length / 2}, (_, id) => ({id,
        start: new THREE.Vector3().fromArray(data, id * stride), end: new THREE.Vector3().fromArray(data, id * stride + 3),
        radiusStart: data[id * stride + 6], radiusEnd: data[id * stride + 7], safeRadius: data[id * stride + 8],
        nodeStartId: links[id * 2], nodeEndId: links[id * 2 + 1]})));
    const geometry = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(fixture.positions, 3));
    const sourceTree = new MeshBVH(geometry, {indirect: true});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const start = count; count += edge.cellCount; return start;});
    const field = new FlowConcentrationField(network, offsets, count);
    const built = buildAnatomicalContrastSurface(network, {fallbackGeometry: geometry}, field, {projection: 'column'});
    const tree = new MeshBVH(built.geometry, {indirect: true});
    const integral = createFlowColumnIntegral(network, field, built.territories, built.continuations);
    const target = new THREE.Vector3(2.34375, -255.71875, 10.3);
    const nearestLarge = network.edges.filter(edge => edge.radiusStart > 6)
        .sort((a, b) => a.start.distanceToSquared(target) - b.start.distanceToSquared(target))[0];
    const aortaTerritory = built.territories[nearestLarge.index];
    const pathEdges = new Map();
    function edgesForPath(path) {
        if (!pathEdges.has(path)) pathEdges.set(path, network.edges.filter(edge => built.territories[edge.index] === path));
        return pathEdges.get(path);
    }
    function coreIntervals(ray, path, radiusScale = .7, arcRange = null) {
        const blood = intervalsFor(ray, sourceTree).map(hit => hit.range);
        const core = mergeIntervals(edgesForPath(path).flatMap(edge => {
            if (!arcRange) return capsuleIntervals(ray, edge, radiusScale * Math.min(edge.radiusStart, edge.radiusEnd));
            const startS = integral.edgeStart[edge.index];
            const lo = Math.max(0, arcRange[0] - startS), hi = Math.min(edge.length, arcRange[1] - startS);
            if (hi <= lo) return [];
            const radiusAt = s => edge.radiusStart + (edge.radiusEnd - edge.radiusStart) * s / edge.length;
            const clipped = {start: edge.start.clone().addScaledVector(edge.axis, lo),
                end: edge.start.clone().addScaledVector(edge.axis, hi), axis: edge.axis, length: hi - lo};
            return capsuleIntervals(ray, clipped, radiusScale * Math.min(radiusAt(lo), radiusAt(hi)));
        }));
        return intersectIntervals(blood, core);
    }
    const parents = path => {
        const edge = network.edges[path], parent = edge.parentEdgeIndex;
        return parent < 0 ? null : {path: built.territories[parent], s: integral.edgeStart[parent] + network.edges[parent].length};
    };
    const depth = path => {let n = 0; while ((path = parents(path)?.path) !== undefined) n++; return n;};
    function coordinate(hit) {
        const p = built.geometry.attributes.position, c = built.geometry.attributes.columnCoordinate, idx = built.geometry.index;
        const vertices = [0, 1, 2].map(k => idx ? idx.getX(hit.faceIndex * 3 + k) : hit.faceIndex * 3 + k);
        const triangle = new THREE.Triangle(...vertices.map(i => new THREE.Vector3().fromBufferAttribute(p, i)));
        const bary = triangle.getBarycoord(hit.point, new THREE.Vector3());
        return {path: c.getX(vertices[0]), s: vertices.reduce((sum, v, k) => sum + bary.getComponent(k) * c.getY(v), 0)};
    }
    function hitsFor(ray, bvh) {
        return bvh.raycast(ray, THREE.DoubleSide).sort((a, b) => a.distance - b.distance)
            .filter((hit, i, hits) => !i || hit.distance - hits[i - 1].distance > 1e-5);
    }
    function intervalsFor(ray, bvh, coordinates = false) {
        const hits = hitsFor(ray, bvh), result = [];
        for (let i = 0; i + 1 < hits.length; i++) {
            const a = hits[i], b = hits[i + 1];
            if (a.face.normal.dot(ray.direction) <= 0 || b.face.normal.dot(ray.direction) >= 0) continue;
            result.push({range: [a.distance, b.distance], ...(coordinates ? {entry: coordinate(a), exit: coordinate(b)} : {})});
        }
        return result;
    }
    // A reproduction of the old route mean is deliberately NOT the oracle.
    // Keeping it in this diagnostic makes a large false transparent region
    // demonstrable without launching a potentially unstable GPU workload.
    function legacyRouteMean(a, sa, b, sb) {
        let mass = 0, distance = 0;
        const originalA = a, originalSa = sa, originalB = b, originalSb = sb;
        for (let i = 0; i < 64; i++) {
            if (a === b) {
                const span = Math.abs(sa - sb);
                if (distance + span < .001) return integral.sample(a, (sa + sb) * .5);
                return (mass + Math.abs(integral.integral(a, sa, sb))) / (distance + span);
            }
            const stepA = depth(a) >= depth(b), path = stepA ? a : b, s = stepA ? sa : sb, parent = parents(path);
            if (!parent) break;
            mass += Math.max(0, integral.integral(path, 0, s)); distance += s;
            if (stepA) {a = parent.path; sa = parent.s;} else {b = parent.path; sb = parent.s;}
        }
        return .5 * (integral.sample(originalA, originalSa) + integral.sample(originalB, originalSb));
    }
    return {
        network, field, offsets, geometry, built, integral, aortaTerritory, coreIntervals,
        branchCoreOutsideParent(ray, path, radiusScale = .7, arcRange = null) {
            // The externally exposed daughter core avoids any ambiguity
            // about which stream owns the parent/daughter overlap region.
            return subtractIntervals(coreIntervals(ray, path, radiusScale, arcRange), coreIntervals(ray, aortaTerritory, 1));
        },
        ray(angle = 0, x = target.x, y = target.y, z = target.z) {
            const direction = new THREE.Vector3(Math.sin(angle * Math.PI / 180), 0, -Math.cos(angle * Math.PI / 180));
            return new THREE.Ray(new THREE.Vector3(x, y, z).addScaledVector(direction, -100), direction);
        },
        fill(concentration) {
            const values = new Float32Array(count), point = new THREE.Vector3();
            for (const edge of network.edges) for (let cell = 0; cell < edge.cellCount; cell++) {
                point.copy(edge.start).lerp(edge.end, (cell + .5) / edge.cellCount);
                const c = concentration(point, edge, built.territories[edge.index]);
                values[offsets[edge.index] + cell] = c;
                edge.massMg[cell] = .3 * c * edge.volumes[cell];
            }
            field.update(values); integral.update(field.values);
        },
        sample(ray) {
            const sourceIntervals = intervalsFor(ray, sourceTree).map(hit => hit.range);
            const intervals = intervalsFor(ray, tree, true);
            let legacyIodine = 0;
            for (const hit of intervals) {
                hit.mean = legacyRouteMean(hit.entry.path, hit.entry.s, hit.exit.path, hit.exit.s);
                legacyIodine += (hit.range[1] - hit.range[0]) * hit.mean;
            }
            // Inner 70%-radius aortic capsules give a conservative filled parent
            // core. Intersect with ORIGINAL blood geometry, so this oracle
            // neither widens vessels nor treats adjacent branches as parent.
            const parentIntervals = coreIntervals(ray, aortaTerritory);
            return {sourceIntervals, intervals, chord: totalLength(sourceIntervals), legacyIodine,
                parentCoreIodine: totalLength(parentIntervals), spatialSlabIodine: spatialSlabIntegral(ray, sourceIntervals)};
        },
        dispose() {integral.dispose(); built.geometry.dispose(); geometry.dispose();}
    };
}


/** Frozen CPU copy of the first spatial-projection revision, for before images. */
export function createOstiumBeforeProjection(fixture) {
    const {network, integral, built} = fixture;
    const geometry = createFlowColumnGeometry(network, integral, built.territories, built.continuations);
    const parentOf = path => {
        const parent = network.edges[path].parentEdgeIndex;
        return parent < 0 ? null : {path: built.territories[parent], s: integral.edgeStart[parent] + network.edges[parent].length};
    };
    const depth = path => {let n = 0; while ((path = parentOf(path)?.path) !== undefined) n++; return n;};
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
    const local = {}, delta = new THREE.Vector3();
    function nearest(path, point, seed, out = {}) {
        const range = geometry.ranges.get(path);
        let s = clamp(seed, 0, range.length);
        for (let i = 0; i < 3; i++) {
            geometry.sample(path, s, local);
            s = clamp(s + delta.subVectors(point, local.point).dot(local.derivative) /
                Math.max(1e-8, local.derivative.lengthSq()), 0, range.length);
        }
        geometry.sample(path, s, out); out.s = s; out.score = point.distanceTo(out.point) - out.radius;
        return out;
    }
    function average(a, b, entry, exit) {
        if (a.path === b.path) return integral.average(a.path, a.s, b.s);
        const choices = candidates(a.path, a.s, b.path, b.s), midpoint = entry.clone().lerp(exit, .5);
        const point = new THREE.Vector3(), selected = {};
        for (const candidate of choices) {
            candidate.s = nearest(candidate.path, midpoint, candidate.s, selected).s;
            candidate.s = nearest(candidate.path, midpoint, candidate.s, selected).s;
        }
        let sum = 0;
        for (let i = 0; i < 24; i++) {
            point.copy(entry).lerp(exit, (i + .5) / 24);
            let score = Infinity, path = choices[0].path, coordinate = 0;
            for (const candidate of choices) {
                nearest(candidate.path, point, candidate.s, selected);
                if (selected.score < score || (selected.score === score && candidate.path < path)) {
                    score = selected.score; path = candidate.path; coordinate = selected.s;
                }
            }
            sum += integral.sample(path, coordinate);
        }
        return sum / 24;
    }
    return {
        average,
        iodine(ray, sample = fixture.sample(ray)) {
            let value = 0;
            for (const {entry, exit, range} of sample.intervals) value += (range[1] - range[0]) *
                average(entry, exit, ray.at(range[0], new THREE.Vector3()), ray.at(range[1], new THREE.Vector3()));
            return value;
        },
        dispose() {geometry.dispose();}
    };
}

/** Finite ostial washout remnant: smooth ends and a known fully filled central plateau. */
export function ostialBolusConcentration(s) {
    const smooth = x => {x = clamp(x, 0, 1); return x * x * (3 - 2 * x);};
    return smooth((s - 8) / 2) * smooth((16 - s) / 2);
}
