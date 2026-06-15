import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';

const DEFAULT_INTERIOR_SAMPLE_SPACING = 6;
const DEFAULT_EDGE_PRECISION = 3;
const DEFAULT_CONTOUR_POINT_TOLERANCE = 0.08;
const DEFAULT_LUMEN_CONTOUR_MIN_AREA = 10;
const INTERIOR_SAMPLE_GRID_STEPS = 17;
const MAX_DEBUG_BOUNDARY_SEGMENTS = 6000;

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

function distanceToPolygonSq(point, polygon) {
    let distanceSq = Infinity;
    for (let i = 0; i < polygon.length; i++) {
        distanceSq = Math.min(distanceSq, pointSegmentDistanceSq(point, polygon[i], polygon[(i + 1) % polygon.length]));
    }
    return distanceSq;
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

function buildInteriorReferenceSamples(geometry, spacing, contourTolerance, lumenContourMinArea) {
    const position = geometry.attributes.position;
    const box = geometry.boundingBox;
    const samples = [];
    if (!box || !Number.isFinite(box.min.y) || !Number.isFinite(box.max.y)) return samples;

    const firstY = box.min.y + spacing * 0.37;
    const sliceYs = [];
    for (let y = firstY; y <= box.max.y - spacing * 0.15; y += spacing) {
        sliceYs.push(y);
    }
    if (!sliceYs.length) return samples;
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
        for (const loop of loops) {
            if (loop.depth % 2 !== 1) continue;
            const sample = bestInteriorPoint(loop.polygon);
            samples.push(new THREE.Vector3(sample.x, sliceYs[sliceIndex], sample.z));
        }
    }

    return samples;
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
    const interiorSamples = buildInteriorReferenceSamples(
        geometry,
        interiorSampleSpacing,
        contourPointTolerance,
        lumenContourMinArea
    );
    const boundaryDebugSegments = buildBoundaryDebugSegments(topology.boundaryEdges);

    return {
        geometry,
        interiorSamples,
        boundaryDebugSegments,
        diagnostics: {
            boundingBox: box,
            boundaryEdgeCount: topology.boundaryEdgeCount,
            degenerateTriangleCount: topology.degenerateTriangleCount,
            edgeCount: topology.edgeCount,
            interiorSampleCount: interiorSamples.length,
            nonManifoldEdgeCount: topology.nonManifoldEdgeCount,
            size,
            transform,
            triangleCount: geometry.attributes.position.count / 3,
            vertexCount: geometry.attributes.position.count
        }
    };
}
