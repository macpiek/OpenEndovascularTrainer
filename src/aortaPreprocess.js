import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';

const DEFAULT_INTERIOR_SAMPLE_SPACING = 6;
const DEFAULT_EDGE_PRECISION = 3;
const DEFAULT_CONTOUR_POINT_TOLERANCE = 0.08;
const DEFAULT_LUMEN_CONTOUR_MIN_AREA = 10;
const INTERIOR_SAMPLE_GRID_STEPS = 17;
const MAX_DEBUG_BOUNDARY_SEGMENTS = 6000;
const MAX_DEBUG_LUMEN_CONTOUR_SEGMENTS = 12000;
const FIELD_POLYGON_SIMPLIFY_TOLERANCE = 0.35;

function vertexKey(position, index, precision = DEFAULT_EDGE_PRECISION) {
    const x = position.getX(index).toFixed(precision);
    const y = position.getY(index).toFixed(precision);
    const z = position.getZ(index).toFixed(precision);
    return `${x},${y},${z}`;
}

function edgeKey(a, b) {
    return a < b ? `${a}|${b}` : `${b}|${a}`;
}

function computeTopologyDiagnostics(geometry) {
    const position = geometry.attributes.position;
    const edges = new Map();
    let degenerateTriangleCount = 0;
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const c = new THREE.Vector3();
    const ab = new THREE.Vector3();
    const ac = new THREE.Vector3();

    for (let i = 0; i < position.count; i += 3) {
        const k0 = vertexKey(position, i);
        const k1 = vertexKey(position, i + 1);
        const k2 = vertexKey(position, i + 2);
        a.fromBufferAttribute(position, i);
        b.fromBufferAttribute(position, i + 1);
        c.fromBufferAttribute(position, i + 2);
        ab.subVectors(b, a);
        ac.subVectors(c, a);
        if (ab.cross(ac).lengthSq() < 1e-8) degenerateTriangleCount++;

        for (const [from, to] of [[k0, k1], [k1, k2], [k2, k0]]) {
            const key = edgeKey(from, to);
            const entry = edges.get(key) || { count: 0, from, to };
            entry.count++;
            edges.set(key, entry);
        }
    }

    const boundaryEdges = [];
    let nonManifoldEdgeCount = 0;
    for (const edge of edges.values()) {
        if (edge.count === 1) boundaryEdges.push(edge);
        else if (edge.count > 2) nonManifoldEdgeCount++;
    }

    return {
        boundaryEdges,
        boundaryEdgeCount: boundaryEdges.length,
        degenerateTriangleCount,
        edgeCount: edges.size,
        nonManifoldEdgeCount
    };
}

function buildBoundaryDebugSegments(boundaryEdges) {
    const positions = [];
    const step = Math.max(1, Math.ceil(boundaryEdges.length / MAX_DEBUG_BOUNDARY_SEGMENTS));
    for (let i = 0; i < boundaryEdges.length; i += step) {
        const edge = boundaryEdges[i];
        const from = edge.from.split(',').map(Number);
        const to = edge.to.split(',').map(Number);
        positions.push(from[0], from[1], from[2], to[0], to[1], to[2]);
    }
    return new Float32Array(positions);
}

function contourKey(point, tolerance) {
    const x = Math.round(point.x / tolerance);
    const z = Math.round(point.z / tolerance);
    return `${x}|${z}`;
}

function contourEdgeKey(a, b) {
    return a < b ? `${a}>${b}` : `${b}>${a}`;
}

function polygonArea(points) {
    let sum = 0;
    for (let i = 0; i < points.length; i++) {
        const a = points[i];
        const b = points[(i + 1) % points.length];
        sum += a.x * b.z - b.x * a.z;
    }
    return sum * 0.5;
}

function polygonCentroid(points) {
    let areaTwice = 0;
    let x = 0;
    let z = 0;
    for (let i = 0; i < points.length; i++) {
        const a = points[i];
        const b = points[(i + 1) % points.length];
        const cross = a.x * b.z - b.x * a.z;
        areaTwice += cross;
        x += (a.x + b.x) * cross;
        z += (a.z + b.z) * cross;
    }

    if (Math.abs(areaTwice) < 1e-8) {
        return {
            x: points.reduce((sum, point) => sum + point.x, 0) / points.length,
            z: points.reduce((sum, point) => sum + point.z, 0) / points.length
        };
    }

    return {
        x: x / (3 * areaTwice),
        z: z / (3 * areaTwice)
    };
}

function pointInPolygon(point, polygon) {
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const a = polygon[i];
        const b = polygon[j];
        if (
            (a.z > point.z) !== (b.z > point.z) &&
            point.x < (b.x - a.x) * (point.z - a.z) / (b.z - a.z + 1e-12) + a.x
        ) {
            inside = !inside;
        }
    }
    return inside;
}

function pointSegmentDistanceSq(point, a, b) {
    const dx = b.x - a.x;
    const dz = b.z - a.z;
    const lengthSq = dx * dx + dz * dz || 1;
    const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.z - a.z) * dz) / lengthSq));
    const x = a.x + dx * t;
    const z = a.z + dz * t;
    const px = point.x - x;
    const pz = point.z - z;
    return px * px + pz * pz;
}

function closestPointOnSegment(point, a, b) {
    const dx = b.x - a.x;
    const dz = b.z - a.z;
    const lengthSq = dx * dx + dz * dz || 1;
    const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.z - a.z) * dz) / lengthSq));
    const x = a.x + dx * t;
    const z = a.z + dz * t;
    const px = point.x - x;
    const pz = point.z - z;
    return {
        point: { x, z },
        distanceSq: px * px + pz * pz
    };
}

function distanceToPolygonSq(point, polygon) {
    let distanceSq = Infinity;
    for (let i = 0; i < polygon.length; i++) {
        distanceSq = Math.min(distanceSq, pointSegmentDistanceSq(point, polygon[i], polygon[(i + 1) % polygon.length]));
    }
    return distanceSq;
}

function closestPointOnPolygon(point, polygon) {
    let best = null;
    for (let i = 0; i < polygon.length; i++) {
        const candidate = closestPointOnSegment(point, polygon[i], polygon[(i + 1) % polygon.length]);
        if (!best || candidate.distanceSq < best.distanceSq) best = candidate;
    }
    return best || { point: { x: point.x, z: point.z }, distanceSq: 0 };
}

function polygonBounds(points) {
    const bounds = {
        minX: Infinity,
        maxX: -Infinity,
        minZ: Infinity,
        maxZ: -Infinity
    };
    for (const point of points) {
        bounds.minX = Math.min(bounds.minX, point.x);
        bounds.maxX = Math.max(bounds.maxX, point.x);
        bounds.minZ = Math.min(bounds.minZ, point.z);
        bounds.maxZ = Math.max(bounds.maxZ, point.z);
    }
    return bounds;
}

function simplifyClosedPolygon(points, tolerance = FIELD_POLYGON_SIMPLIFY_TOLERANCE) {
    if (points.length <= 18 || tolerance <= 0) return points;
    const toleranceSq = tolerance * tolerance;
    let simplified = points.slice();

    for (let pass = 0; pass < 8 && simplified.length > 18; pass++) {
        const next = [];
        let changed = false;
        for (let i = 0; i < simplified.length; i++) {
            const previous = simplified[(i - 1 + simplified.length) % simplified.length];
            const current = simplified[i];
            const following = simplified[(i + 1) % simplified.length];
            const distanceSq = pointSegmentDistanceSq(current, previous, following);
            if (distanceSq < toleranceSq && simplified.length - next.length > 18) {
                changed = true;
                continue;
            }
            next.push(current);
        }
        simplified = next;
        if (!changed) break;
    }

    return simplified.length >= 18 ? simplified : points;
}

function boundsDistanceSq(point, bounds) {
    const dx = point.x < bounds.minX
        ? bounds.minX - point.x
        : point.x > bounds.maxX
            ? point.x - bounds.maxX
            : 0;
    const dz = point.z < bounds.minZ
        ? bounds.minZ - point.z
        : point.z > bounds.maxZ
            ? point.z - bounds.maxZ
            : 0;
    return dx * dx + dz * dz;
}

function signedDistanceToContour(point, contour) {
    const closest = closestPointOnPolygon(point, contour.polygon);
    const distance = Math.sqrt(closest.distanceSq);
    const inside = pointInPolygon(point, contour.polygon);
    let x = inside ? point.x - closest.point.x : closest.point.x - point.x;
    let z = inside ? point.z - closest.point.z : closest.point.z - point.z;
    const length = Math.hypot(x, z);
    if (length > 1e-8) {
        x /= length;
        z /= length;
    } else {
        x = contour.sample.x - closest.point.x;
        z = contour.sample.z - closest.point.z;
        const fallbackLength = Math.hypot(x, z) || 1;
        x /= fallbackLength;
        z /= fallbackLength;
    }

    return {
        signedDistance: inside ? distance : -distance,
        inside,
        inward: { x, z },
        closestPoint: closest.point,
        contour
    };
}

function bestInteriorPoint(polygon) {
    let minX = Infinity;
    let maxX = -Infinity;
    let minZ = Infinity;
    let maxZ = -Infinity;
    for (const point of polygon) {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minZ = Math.min(minZ, point.z);
        maxZ = Math.max(maxZ, point.z);
    }

    let best = null;
    let bestDistanceSq = -1;
    const consider = point => {
        if (!pointInPolygon(point, polygon)) return;
        const distanceSq = distanceToPolygonSq(point, polygon);
        if (distanceSq > bestDistanceSq) {
            best = point;
            bestDistanceSq = distanceSq;
        }
    };

    consider(polygonCentroid(polygon));
    for (let ix = 0; ix < INTERIOR_SAMPLE_GRID_STEPS; ix++) {
        for (let iz = 0; iz < INTERIOR_SAMPLE_GRID_STEPS; iz++) {
            consider({
                x: minX + (maxX - minX) * (ix + 0.5) / INTERIOR_SAMPLE_GRID_STEPS,
                z: minZ + (maxZ - minZ) * (iz + 0.5) / INTERIOR_SAMPLE_GRID_STEPS
            });
        }
    }

    return best || polygonCentroid(polygon);
}

function intersectTriangleAtY(position, index, y) {
    const vertices = [
        { x: position.getX(index), y: position.getY(index), z: position.getZ(index) },
        { x: position.getX(index + 1), y: position.getY(index + 1), z: position.getZ(index + 1) },
        { x: position.getX(index + 2), y: position.getY(index + 2), z: position.getZ(index + 2) }
    ];
    const intersections = [];
    for (const [a, b] of [[vertices[0], vertices[1]], [vertices[1], vertices[2]], [vertices[2], vertices[0]]]) {
        if ((a.y < y && b.y > y) || (a.y > y && b.y < y)) {
            const t = (y - a.y) / (b.y - a.y);
            intersections.push({
                x: a.x + (b.x - a.x) * t,
                z: a.z + (b.z - a.z) * t
            });
        }
    }
    return intersections.length === 2 ? intersections : null;
}

function buildContourLoops(segments, tolerance, minArea) {
    const points = new Map();
    const adjacency = new Map();
    const edges = new Set();

    const addPoint = point => {
        const key = contourKey(point, tolerance);
        if (!points.has(key)) points.set(key, point);
        return key;
    };
    const addSegment = (a, b) => {
        const keyA = addPoint(a);
        const keyB = addPoint(b);
        if (keyA === keyB) return;
        const key = contourEdgeKey(keyA, keyB);
        if (edges.has(key)) return;
        edges.add(key);
        if (!adjacency.has(keyA)) adjacency.set(keyA, new Set());
        if (!adjacency.has(keyB)) adjacency.set(keyB, new Set());
        adjacency.get(keyA).add(keyB);
        adjacency.get(keyB).add(keyA);
    };

    for (const segment of segments) addSegment(segment[0], segment[1]);

    const used = new Set();
    const loops = [];
    for (const edge of edges) {
        if (used.has(edge)) continue;
        const [start, firstNext] = edge.split('>');
        let previous = null;
        let current = start;
        let next = firstNext;
        const loopKeys = [start];
        let closed = false;

        for (let guard = 0; guard < 6000; guard++) {
            used.add(contourEdgeKey(current, next));
            previous = current;
            current = next;
            loopKeys.push(current);
            if (current === start) {
                closed = true;
                break;
            }

            const neighbors = [...(adjacency.get(current) || [])];
            let candidate = neighbors.find(neighbor =>
                neighbor !== previous &&
                !used.has(contourEdgeKey(current, neighbor))
            );
            if (!candidate) candidate = neighbors.find(neighbor => neighbor !== previous);
            if (!candidate) break;
            next = candidate;
        }

        if (!closed || loopKeys.length < 9) continue;
        const polygon = loopKeys.slice(0, -1).map(key => points.get(key));
        const area = Math.abs(polygonArea(polygon));
        if (area < minArea) continue;
        loops.push({
            polygon,
            area,
            centroid: polygonCentroid(polygon)
        });
    }

    loops.sort((a, b) => b.area - a.area);
    for (const loop of loops) {
        loop.depth = 0;
        for (const other of loops) {
            if (other === loop || other.area <= loop.area) continue;
            if (pointInPolygon(loop.centroid, other.polygon)) loop.depth++;
        }
    }
    return loops;
}

function buildLumenSlices(geometry, spacing, contourTolerance, lumenContourMinArea) {
    const position = geometry.attributes.position;
    const box = geometry.boundingBox;
    const slices = [];
    const samples = [];
    if (!box || !Number.isFinite(box.min.y) || !Number.isFinite(box.max.y)) {
        return { slices, samples };
    }

    const firstY = box.min.y + spacing * 0.37;
    const sliceYs = [];
    for (let y = firstY; y <= box.max.y - spacing * 0.15; y += spacing) {
        sliceYs.push(y);
    }
    if (!sliceYs.length) return { slices, samples };
    const segmentsBySlice = sliceYs.map(() => []);

    for (let i = 0; i < position.count; i += 3) {
        const y0 = position.getY(i);
        const y1 = position.getY(i + 1);
        const y2 = position.getY(i + 2);
        const minY = Math.min(y0, y1, y2);
        const maxY = Math.max(y0, y1, y2);
        if (maxY <= firstY || minY >= sliceYs[sliceYs.length - 1]) continue;

        const start = Math.max(0, Math.ceil((minY - firstY) / spacing));
        const end = Math.min(sliceYs.length - 1, Math.floor((maxY - firstY) / spacing));
        for (let sliceIndex = start; sliceIndex <= end; sliceIndex++) {
            const y = sliceYs[sliceIndex];
            if (!(minY < y && maxY > y)) continue;
            const intersections = intersectTriangleAtY(position, i, y);
            if (intersections) segmentsBySlice[sliceIndex].push(intersections);
        }
    }

    for (let sliceIndex = 0; sliceIndex < sliceYs.length; sliceIndex++) {
        const loops = buildContourLoops(segmentsBySlice[sliceIndex], contourTolerance, lumenContourMinArea);
        const contours = [];
        for (const loop of loops) {
            if (loop.depth % 2 !== 1) continue;
            const sample = bestInteriorPoint(loop.polygon);
            const polygon = simplifyClosedPolygon(loop.polygon);
            const contour = {
                polygon,
                area: loop.area,
                bounds: polygonBounds(polygon),
                centroid: loop.centroid,
                sample
            };
            contours.push(contour);
            samples.push(new THREE.Vector3(sample.x, sliceYs[sliceIndex], sample.z));
        }
        if (contours.length) {
            slices.push({
                y: sliceYs[sliceIndex],
                contours
            });
        }
    }

    return { slices, samples };
}

function buildLumenContourDebugSegments(slices) {
    const allEdges = [];
    for (const slice of slices) {
        for (const contour of slice.contours) {
            for (let i = 0; i < contour.polygon.length; i++) {
                const a = contour.polygon[i];
                const b = contour.polygon[(i + 1) % contour.polygon.length];
                allEdges.push({ y: slice.y, a, b });
            }
        }
    }

    const positions = [];
    const step = Math.max(1, Math.ceil(allEdges.length / MAX_DEBUG_LUMEN_CONTOUR_SEGMENTS));
    for (let i = 0; i < allEdges.length; i += step) {
        const edge = allEdges[i];
        positions.push(
            edge.a.x, edge.y, edge.a.z,
            edge.b.x, edge.y, edge.b.z
        );
    }
    return new Float32Array(positions);
}

function querySlice(slice, x, z) {
    const point = { x, z };
    let best = null;
    for (const contour of slice.contours) {
        const boundDistanceSq = boundsDistanceSq(point, contour.bounds);
        if (
            best &&
            best.signedDistance < 0 &&
            -Math.sqrt(boundDistanceSq) <= best.signedDistance
        ) {
            continue;
        }

        const candidate = signedDistanceToContour(point, contour);
        if (!best || candidate.signedDistance > best.signedDistance) best = candidate;
    }
    return best || {
        signedDistance: -Infinity,
        inside: false,
        inward: { x: 1, z: 0 },
        closestPoint: { x, z },
        contour: null
    };
}

function findSliceInterval(slices, y) {
    if (slices.length <= 1) return { lower: 0, upper: 0, t: 0 };
    if (y <= slices[0].y) return { lower: 0, upper: 0, t: 0 };
    const last = slices.length - 1;
    if (y >= slices[last].y) return { lower: last, upper: last, t: 0 };

    let lo = 0;
    let hi = last;
    while (hi - lo > 1) {
        const mid = Math.floor((lo + hi) * 0.5);
        if (slices[mid].y <= y) lo = mid;
        else hi = mid;
    }
    const span = Math.max(1e-6, slices[hi].y - slices[lo].y);
    return {
        lower: lo,
        upper: hi,
        t: Math.max(0, Math.min(1, (y - slices[lo].y) / span))
    };
}

function normalize3(x, y, z, fallback = { x: 1, y: 0, z: 0 }) {
    const length = Math.hypot(x, y, z);
    if (length < 1e-8) return { ...fallback };
    return { x: x / length, y: y / length, z: z / length };
}

export function createLumenField(lumenSlices) {
    const slices = (lumenSlices || [])
        .filter(slice => slice?.contours?.length)
        .slice()
        .sort((a, b) => a.y - b.y);

    function query(input) {
        const x = input.x;
        const y = input.y;
        const z = input.z;
        if (!slices.length) {
            const fallback = new THREE.Vector3(1, 0, 0);
            return {
                inside: false,
                signedDistance: -Infinity,
                distance: Infinity,
                inward: fallback,
                normal: fallback.clone().multiplyScalar(-1),
                closestPoint: new THREE.Vector3(x, y, z),
                targetAtClearance: clearance => new THREE.Vector3(x + clearance, y, z)
            };
        }

        const interval = findSliceInterval(slices, y);
        const lowerSlice = slices[interval.lower];
        const upperSlice = slices[interval.upper];
        const lower = querySlice(lowerSlice, x, z);
        const upper = interval.upper === interval.lower
            ? lower
            : querySlice(upperSlice, x, z);
        const t = interval.t;
        const signedDistance = lower.signedDistance * (1 - t) + upper.signedDistance * t;
        const dy = Math.max(1e-6, Math.abs(upperSlice.y - lowerSlice.y));
        const yGradient = interval.upper === interval.lower
            ? 0
            : Math.max(-0.85, Math.min(0.85, (upper.signedDistance - lower.signedDistance) / dy));
        const inward = normalize3(
            lower.inward.x * (1 - t) + upper.inward.x * t,
            yGradient,
            lower.inward.z * (1 - t) + upper.inward.z * t
        );
        const closestPoint = new THREE.Vector3(
            x - inward.x * signedDistance,
            y - inward.y * signedDistance,
            z - inward.z * signedDistance
        );
        const inwardVector = new THREE.Vector3(inward.x, inward.y, inward.z);
        const normal = inwardVector.clone().multiplyScalar(-1);

        return {
            inside: signedDistance >= 0,
            signedDistance,
            distance: Math.abs(signedDistance),
            inward: inwardVector,
            normal,
            closestPoint,
            lowerSlice,
            upperSlice,
            targetAtClearance(clearance = 0) {
                const correction = Math.max(0, clearance - signedDistance);
                return new THREE.Vector3(x, y, z).addScaledVector(inwardVector, correction);
            }
        };
    }

    return {
        slices,
        query,
        containsPoint: point => query(point).inside
    };
}

export function preprocessAortaGeometry(geometry, {
    transform = {},
    interiorSampleSpacing = DEFAULT_INTERIOR_SAMPLE_SPACING,
    contourPointTolerance = DEFAULT_CONTOUR_POINT_TOLERANCE,
    lumenContourMinArea = DEFAULT_LUMEN_CONTOUR_MIN_AREA
} = {}) {
    geometry.computeVertexNormals();
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    geometry.boundsTree = new MeshBVH(geometry);

    const box = geometry.boundingBox.clone();
    const size = box.getSize(new THREE.Vector3());
    const topology = computeTopologyDiagnostics(geometry);
    const lumen = buildLumenSlices(
        geometry,
        interiorSampleSpacing,
        contourPointTolerance,
        lumenContourMinArea
    );
    const boundaryDebugSegments = buildBoundaryDebugSegments(topology.boundaryEdges);
    const lumenContourDebugSegments = buildLumenContourDebugSegments(lumen.slices);
    const lumenField = createLumenField(lumen.slices);

    return {
        geometry,
        interiorSamples: lumen.samples,
        lumenSlices: lumen.slices,
        lumenField,
        boundaryDebugSegments,
        lumenContourDebugSegments,
        diagnostics: {
            boundingBox: box,
            boundaryEdgeCount: topology.boundaryEdgeCount,
            degenerateTriangleCount: topology.degenerateTriangleCount,
            edgeCount: topology.edgeCount,
            interiorSampleCount: lumen.samples.length,
            lumenSliceCount: lumen.slices.length,
            nonManifoldEdgeCount: topology.nonManifoldEdgeCount,
            size,
            transform,
            triangleCount: geometry.attributes.position.count / 3,
            vertexCount: geometry.attributes.position.count
        }
    };
}
