import * as THREE from 'three';

const DEFAULT_CELL_SIZE = 36;
const DEFAULT_INFLATION = 4;
const DEFAULT_MIN_RADIUS = 1.5;
const DEFAULT_MAX_DEBUG_CAPSULES = 900;
const MAX_SLICE_LINK_GAP = 3;
const MAX_BRIDGE_SLICE_LINK_GAP = 8;
const MAX_COVERAGE_SLICE_LINK_GAP = 18;
const LUMEN_SEGMENT_SAMPLE_SPACING = 3.5;
const LUMEN_SEGMENT_MIN_MARGIN = -0.45;
const LUMEN_BRIDGE_MIN_MARGIN = 0.08;
const LUMEN_COVERAGE_MIN_MARGIN = -0.55;
const CONTOUR_PROXIMITY_PADDING = 3.5;
const CONTOUR_OVERLAP_PADDING = 1.25;
const SLICE_GAP_COST = 0.82;
const BRIDGE_EDGE_COST = 4.75;
const COVERAGE_EDGE_COST = 9.5;
const COVERAGE_CANDIDATES_PER_NODE = 4;
const SPATIAL_COVERAGE_MAX_DISTANCE = 42;
const SPATIAL_COVERAGE_CANDIDATES_PER_NODE = 7;
const SPATIAL_COVERAGE_EDGE_COST = 11.5;
const ISOLATED_STUB_MIN_LENGTH = 3;
const ISOLATED_STUB_MAX_LENGTH = 12;
const VOLUME_GRID_SPACING = 3.5;
const VOLUME_INSIDE_MARGIN = 0;
const VOLUME_RIDGE_MIN_DISTANCE = 0.85;
const VOLUME_RIDGE_TOLERANCE = 0.55;
const VOLUME_EDGE_RADIUS = 12.5;
const VOLUME_LONG_EDGE_RADIUS = 48;
const VOLUME_NEIGHBOR_LIMIT = 8;
const VOLUME_MAX_RIDGE_SAMPLES = 1800;
const VOLUME_PATH_ANCHOR_SNAP_RADIUS = 20;
const VOLUME_PATH_EDGE_MIN_MARGIN = -0.55;
const VOLUME_PATH_CONNECTOR_MIN_MARGIN = -0.55;
const VOLUME_PATH_CENTER_BIAS = 2.2;
const VOLUME_ANCHOR_EDGE_RADIUS = 36;
const VOLUME_ANCHOR_EDGE_LIMIT = 16;
const VOLUME_RESCUE_EDGE_RADIUS = 48;
const VOLUME_RESCUE_EDGE_LIMIT = 3;
const CENTERLINE_COLOR = 0xfff36a;
const CENTERLINE_NODE_COLOR = 0xffffff;
const CENTERLINE_BRANCH_NODE_COLOR = 0xff4fd8;
const CENTERLINE_NODE_RADIUS = 1.05;
const CENTERLINE_BRANCH_NODE_RADIUS = 1.75;
const CAPSULE_COLOR = 0x39a6ff;
const CAPSULE_EDGE_COLOR = 0x8fd8ff;

function toVector3(point) {
    return point?.isVector3
        ? point.clone()
        : new THREE.Vector3(point?.x || 0, point?.y || 0, point?.z || 0);
}

function finiteRadius(value, fallback = DEFAULT_MIN_RADIUS) {
    return Number.isFinite(value) && value > 0 ? value : fallback;
}

function pointInPolygonCoords(x, z, polygon) {
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const a = polygon[i];
        const b = polygon[j];
        if (
            (a.z > z) !== (b.z > z) &&
            x < (b.x - a.x) * (z - a.z) / (b.z - a.z + 1e-12) + a.x
        ) {
            inside = !inside;
        }
    }
    return inside;
}

function contourCenter(contour, y) {
    const centroid = contour.centroid;
    const source = centroid && contour.polygon && pointInPolygonCoords(centroid.x, centroid.z, contour.polygon)
        ? centroid
        : (contour.sample || contour.centroid || { x: 0, z: 0 });
    return new THREE.Vector3(source.x || 0, y, source.z || 0);
}

function contourRadius(contour) {
    const areaRadius = Number.isFinite(contour?.area) && contour.area > 0
        ? Math.sqrt(contour.area / Math.PI)
        : DEFAULT_MIN_RADIUS;
    const bounds = contour?.bounds;
    if (!bounds) return Math.max(DEFAULT_MIN_RADIUS, areaRadius);
    const halfX = Math.max(
        Math.abs((bounds.maxX ?? bounds.max?.x ?? 0) - (bounds.minX ?? bounds.min?.x ?? 0)) * 0.5,
        DEFAULT_MIN_RADIUS
    );
    const halfZ = Math.max(
        Math.abs((bounds.maxZ ?? bounds.max?.z ?? 0) - (bounds.minZ ?? bounds.min?.z ?? 0)) * 0.5,
        DEFAULT_MIN_RADIUS
    );
    return Math.max(DEFAULT_MIN_RADIUS, Math.min(Math.max(halfX, halfZ), areaRadius * 1.35));
}

function branchSampleKey(sliceIndex, contourIndex) {
    return `${sliceIndex}:${contourIndex}`;
}

function buildSliceSamples(lumenSlices) {
    let nodeId = 0;
    return (lumenSlices || [])
        .filter(slice => Number.isFinite(slice?.y) && slice.contours?.length)
        .map((slice, sliceIndex) => ({
            y: slice.y,
            contours: slice.contours.map((contour, contourIndex) => ({
                nodeId: nodeId++,
                key: branchSampleKey(sliceIndex, contourIndex),
                point: contourCenter(contour, slice.y),
                radius: contourRadius(contour),
                area: Number.isFinite(contour.area) ? contour.area : 0,
                polygon: contour.polygon || [],
                bounds: contour.bounds || null
            }))
        }))
        .sort((a, b) => a.y - b.y);
}

function boundsDistanceSq(a, b) {
    if (!a || !b) return Infinity;
    const dx = a.maxX < b.minX
        ? b.minX - a.maxX
        : b.maxX < a.minX
            ? a.minX - b.maxX
            : 0;
    const dz = a.maxZ < b.minZ
        ? b.minZ - a.maxZ
        : b.maxZ < a.minZ
            ? a.minZ - b.maxZ
            : 0;
    return dx * dx + dz * dz;
}

function pointSegmentDistanceSq2D(point, a, b) {
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

function contourMajorAxis(node) {
    const polygon = node.polygon || [];
    if (polygon.length < 3) return new THREE.Vector3(1, 0, 0);

    let meanX = 0;
    let meanZ = 0;
    for (const point of polygon) {
        meanX += point.x;
        meanZ += point.z;
    }
    meanX /= polygon.length;
    meanZ /= polygon.length;

    let xx = 0;
    let xz = 0;
    let zz = 0;
    for (const point of polygon) {
        const dx = point.x - meanX;
        const dz = point.z - meanZ;
        xx += dx * dx;
        xz += dx * dz;
        zz += dz * dz;
    }

    const angle = Math.abs(xx - zz) < 1e-8 && Math.abs(xz) < 1e-8
        ? 0
        : 0.5 * Math.atan2(2 * xz, xx - zz);
    const axis = new THREE.Vector3(Math.cos(angle), 0, Math.sin(angle));
    return axis.lengthSq() > 1e-8 ? axis.normalize() : new THREE.Vector3(1, 0, 0);
}

function createIsolatedStubSegment(node, lumenField) {
    const center = node.point;
    const directions = [
        contourMajorAxis(node),
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 1)
    ];
    const baseHalfLength = Math.max(
        ISOLATED_STUB_MIN_LENGTH * 0.5,
        Math.min(ISOLATED_STUB_MAX_LENGTH * 0.5, node.radius * 1.15)
    );

    for (const direction of directions) {
        let halfLength = baseHalfLength;
        for (let attempt = 0; attempt < 8; attempt++) {
            const start = center.clone().addScaledVector(direction, -halfLength);
            const end = center.clone().addScaledVector(direction, halfLength);
            const startSigned = lumenField?.query
                ? lumenField.query(start)?.signedDistance
                : Infinity;
            const endSigned = lumenField?.query
                ? lumenField.query(end)?.signedDistance
                : Infinity;
            if (
                (!Number.isFinite(startSigned) || startSigned >= LUMEN_COVERAGE_MIN_MARGIN) &&
                (!Number.isFinite(endSigned) || endSigned >= LUMEN_COVERAGE_MIN_MARGIN)
            ) {
                return {
                    start,
                    end,
                    nodeStartId: node.nodeId,
                    nodeEndId: node.nodeId,
                    radiusStart: node.radius,
                    radiusEnd: node.radius,
                    source: 'lumen-isolated-stub'
                };
            }
            halfLength *= 0.68;
        }
    }

    return null;
}

function polygonDistanceSq(a, b) {
    if (!a?.polygon?.length || !b?.polygon?.length) return Infinity;
    if (pointInPolygonCoords(a.point.x, a.point.z, b.polygon)) return 0;
    if (pointInPolygonCoords(b.point.x, b.point.z, a.polygon)) return 0;

    let best = Infinity;
    for (const point of a.polygon) {
        if (pointInPolygonCoords(point.x, point.z, b.polygon)) return 0;
        for (let i = 0; i < b.polygon.length; i++) {
            best = Math.min(best, pointSegmentDistanceSq2D(point, b.polygon[i], b.polygon[(i + 1) % b.polygon.length]));
        }
    }
    for (const point of b.polygon) {
        if (pointInPolygonCoords(point.x, point.z, a.polygon)) return 0;
        for (let i = 0; i < a.polygon.length; i++) {
            best = Math.min(best, pointSegmentDistanceSq2D(point, a.polygon[i], a.polygon[(i + 1) % a.polygon.length]));
        }
    }
    return best;
}

function interpolatePoint(a, b, t) {
    return new THREE.Vector3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    );
}

function validateLumenSegment(a, b, lumenField, minMargin = LUMEN_SEGMENT_MIN_MARGIN) {
    if (!lumenField?.query) {
        return {
            valid: true,
            minSignedDistance: Infinity
        };
    }

    const length = a.point.distanceTo(b.point);
    const sampleCount = Math.max(4, Math.ceil(length / LUMEN_SEGMENT_SAMPLE_SPACING));
    let minSignedDistance = Infinity;
    for (let i = 0; i <= sampleCount; i++) {
        const point = interpolatePoint(a.point, b.point, i / sampleCount);
        const query = lumenField.query(point);
        const signedDistance = Number.isFinite(query?.signedDistance)
            ? query.signedDistance
            : -Infinity;
        minSignedDistance = Math.min(minSignedDistance, signedDistance);
        if (signedDistance < minMargin) {
            return {
                valid: false,
                minSignedDistance
            };
        }
    }

    return {
        valid: true,
        minSignedDistance
    };
}

function contourCandidate(a, b, gap, spacing, lumenField) {
    const boundsGap = Math.sqrt(boundsDistanceSq(a.bounds, b.bounds));
    const polygonGap = Math.sqrt(polygonDistanceSq(a, b));
    const centerDistance = a.point.distanceTo(b.point);
    const radiusScale = Math.max(1, Math.min(a.radius, b.radius));
    const proximityLimit = Math.max(
        CONTOUR_PROXIMITY_PADDING + spacing * 0.35,
        radiusScale * 0.52 + CONTOUR_OVERLAP_PADDING
    );
    const has2DContinuity =
        boundsGap <= proximityLimit ||
        polygonGap <= proximityLimit ||
        centerDistance <= Math.max(a.radius + b.radius + spacing * 0.85, spacing * 2.3);
    if (!has2DContinuity) return null;

    const lumen = validateLumenSegment(a, b, lumenField);
    if (!lumen.valid) return null;

    return {
        a,
        b,
        gap,
        distance: centerDistance,
        minSignedDistance: lumen.minSignedDistance,
        cost:
            polygonGap / radiusScale * 1.85 +
            boundsGap / radiusScale * 0.75 +
            centerDistance / Math.max(1, a.radius + b.radius + spacing) * 0.65 +
            (gap - 1) * SLICE_GAP_COST -
            Math.min(0.45, Math.max(0, lumen.minSignedDistance) / Math.max(1, radiusScale) * 0.08)
    };
}

function bridgeCandidate(a, b, gap, spacing, lumenField) {
    if (!lumenField?.query) return null;

    const centerDistance = a.point.distanceTo(b.point);
    const radiusScale = Math.max(1, Math.min(a.radius, b.radius));
    const bridgeLimit = Math.max(
        a.radius + b.radius + spacing * 1.35,
        spacing * 2.25,
        18
    );
    if (centerDistance > bridgeLimit) return null;

    const lumen = validateLumenSegment(a, b, lumenField, LUMEN_BRIDGE_MIN_MARGIN);
    if (!lumen.valid) return null;

    return {
        a,
        b,
        gap,
        bridge: true,
        distance: centerDistance,
        minSignedDistance: lumen.minSignedDistance,
        cost:
            BRIDGE_EDGE_COST +
            centerDistance / Math.max(1, a.radius + b.radius + spacing) +
            (gap - 1) * SLICE_GAP_COST * 1.35 -
            Math.min(0.3, lumen.minSignedDistance / Math.max(1, radiusScale) * 0.05)
    };
}

function coverageCandidate(a, b, gap, spacing, lumenField) {
    if (!lumenField?.query || gap <= 0) return null;

    const centerDistance = a.point.distanceTo(b.point);
    const radiusScale = Math.max(1, Math.min(a.radius, b.radius));
    const coverageLimit = Math.max(
        24,
        a.radius + b.radius + spacing * 1.65,
        spacing * 2.45
    );
    if (centerDistance > coverageLimit) return null;

    const lumen = validateLumenSegment(a, b, lumenField, LUMEN_COVERAGE_MIN_MARGIN);
    if (!lumen.valid) return null;

    return {
        a,
        b,
        gap,
        coverage: true,
        distance: centerDistance,
        minSignedDistance: lumen.minSignedDistance,
        cost:
            COVERAGE_EDGE_COST +
            centerDistance / Math.max(1, a.radius + b.radius + spacing) +
            (gap - 1) * SLICE_GAP_COST * 1.55 -
            Math.min(0.25, Math.max(0, lumen.minSignedDistance) / Math.max(1, radiusScale) * 0.04)
    };
}

function spatialCoverageCandidate(a, b, lumenField) {
    if (!lumenField?.query) return null;

    const centerDistance = a.point.distanceTo(b.point);
    const radiusScale = Math.max(1, Math.min(a.radius, b.radius));
    const distanceLimit = Math.max(
        SPATIAL_COVERAGE_MAX_DISTANCE,
        (a.radius + b.radius) * 3.2
    );
    if (centerDistance > distanceLimit) return null;

    const lumen = validateLumenSegment(a, b, lumenField, LUMEN_COVERAGE_MIN_MARGIN);
    if (!lumen.valid) return null;

    return {
        a,
        b,
        spatialCoverage: true,
        distance: centerDistance,
        minSignedDistance: lumen.minSignedDistance,
        cost:
            SPATIAL_COVERAGE_EDGE_COST +
            centerDistance / Math.max(1, radiusScale * 2.2) -
            Math.min(0.25, Math.max(0, lumen.minSignedDistance) / Math.max(1, radiusScale) * 0.04)
    };
}

class UnionFind {
    constructor(size) {
        this.parent = Array.from({ length: size }, (_, index) => index);
        this.rank = new Array(size).fill(0);
    }

    find(index) {
        let root = index;
        while (this.parent[root] !== root) root = this.parent[root];
        while (this.parent[index] !== index) {
            const parent = this.parent[index];
            this.parent[index] = root;
            index = parent;
        }
        return root;
    }

    union(a, b) {
        let rootA = this.find(a);
        let rootB = this.find(b);
        if (rootA === rootB) return false;
        if (this.rank[rootA] < this.rank[rootB]) {
            const tmp = rootA;
            rootA = rootB;
            rootB = tmp;
        }
        this.parent[rootB] = rootA;
        if (this.rank[rootA] === this.rank[rootB]) this.rank[rootA]++;
        return true;
    }
}

function collectNodes(samples) {
    const nodes = [];
    for (const slice of samples) {
        for (const contour of slice.contours) nodes.push(contour);
    }
    return nodes;
}

function appendSpatialCoverageCandidates(nodes, lumenField, addCandidate) {
    if (!lumenField?.query) return;
    const byNode = new Map();
    for (let i = 0; i < nodes.length; i++) {
        const a = nodes[i];
        const bucket = [];
        for (let j = 0; j < nodes.length; j++) {
            if (i === j) continue;
            const b = nodes[j];
            const distance = a.point.distanceTo(b.point);
            if (distance <= 1e-5 || distance > SPATIAL_COVERAGE_MAX_DISTANCE) continue;
            bucket.push({ b, distance });
        }
        bucket.sort((left, right) => left.distance - right.distance);
        byNode.set(a.nodeId, bucket.slice(0, SPATIAL_COVERAGE_CANDIDATES_PER_NODE));
    }

    for (const a of nodes) {
        const bucket = byNode.get(a.nodeId) || [];
        for (const item of bucket) {
            addCandidate(spatialCoverageCandidate(a, item.b, lumenField));
        }
    }
}

function buildCandidateEdges(samples, nodes, lumenField) {
    const candidates = [];
    const coverageCandidatesByNode = new Map();
    const seen = new Set();
    const addCandidate = candidate => {
        if (!candidate) return;
        const a = Math.min(candidate.a.nodeId, candidate.b.nodeId);
        const b = Math.max(candidate.a.nodeId, candidate.b.nodeId);
        const key = `${a}->${b}`;
        if (seen.has(key)) return;
        seen.add(key);
        candidates.push(candidate);
    };
    const rememberCoverageCandidate = candidate => {
        if (!candidate) return;
        for (const node of [candidate.a, candidate.b]) {
            let bucket = coverageCandidatesByNode.get(node.nodeId);
            if (!bucket) {
                bucket = [];
                coverageCandidatesByNode.set(node.nodeId, bucket);
            }
            bucket.push(candidate);
        }
    };

    for (const slice of samples) {
        for (let i = 0; i < slice.contours.length; i++) {
            for (let j = i + 1; j < slice.contours.length; j++) {
                addCandidate(bridgeCandidate(slice.contours[i], slice.contours[j], 0, 0, lumenField));
            }
        }
    }

    for (let sliceIndex = 0; sliceIndex < samples.length - 1; sliceIndex++) {
        const lower = samples[sliceIndex];
        for (let gap = 1; gap <= MAX_BRIDGE_SLICE_LINK_GAP && sliceIndex + gap < samples.length; gap++) {
            const upper = samples[sliceIndex + gap];
            const spacing = Math.max(1, Math.abs(upper.y - lower.y));
            for (const a of lower.contours) {
                for (const b of upper.contours) {
                    if (gap <= MAX_SLICE_LINK_GAP) {
                        addCandidate(contourCandidate(a, b, gap, spacing, lumenField));
                    }
                    addCandidate(bridgeCandidate(a, b, gap, spacing, lumenField));
                }
            }
        }
        for (let gap = MAX_BRIDGE_SLICE_LINK_GAP + 1; gap <= MAX_COVERAGE_SLICE_LINK_GAP && sliceIndex + gap < samples.length; gap++) {
            const upper = samples[sliceIndex + gap];
            const spacing = Math.max(1, Math.abs(upper.y - lower.y));
            for (const a of lower.contours) {
                for (const b of upper.contours) {
                    rememberCoverageCandidate(coverageCandidate(a, b, gap, spacing, lumenField));
                }
            }
        }
    }

    for (const bucket of coverageCandidatesByNode.values()) {
        bucket.sort((a, b) => a.cost - b.cost);
        for (let i = 0; i < Math.min(COVERAGE_CANDIDATES_PER_NODE, bucket.length); i++) {
            addCandidate(bucket[i]);
        }
    }
    appendSpatialCoverageCandidates(nodes, lumenField, addCandidate);
    return candidates.sort((a, b) => a.cost - b.cost);
}

function graphDiagnostics(nodes, edges) {
    const unionFind = new UnionFind(nodes.length);
    const degree = new Array(nodes.length).fill(0);
    for (const edge of edges) {
        unionFind.union(edge.start.nodeId, edge.end.nodeId);
        degree[edge.start.nodeId]++;
        degree[edge.end.nodeId]++;
    }

    const components = new Map();
    for (const node of nodes) {
        const root = unionFind.find(node.nodeId);
        const component = components.get(root) || {
            count: 0,
            isolated: 0
        };
        component.count++;
        if (degree[node.nodeId] === 0) component.isolated++;
        components.set(root, component);
    }

    return {
        componentCount: components.size,
        isolatedNodeCount: degree.filter(value => value === 0).length,
        endpointNodeCount: degree.filter(value => value === 1).length,
        branchNodeCount: degree.filter(value => value > 2).length,
        largestComponentSize: Math.max(0, ...Array.from(components.values(), component => component.count))
    };
}

function selectCenterlineEdges(nodes, candidates) {
    const unionFind = new UnionFind(nodes.length);
    const edges = [];
    for (const candidate of candidates) {
        if (!unionFind.union(candidate.a.nodeId, candidate.b.nodeId)) continue;
        edges.push({
            start: candidate.a,
            end: candidate.b,
            cost: candidate.cost,
            minSignedDistance: candidate.minSignedDistance
        });
    }
    return {
        edges,
        diagnostics: graphDiagnostics(nodes, edges)
    };
}

function buildSegmentsFromLumenSlices(lumenSlices, lumenField) {
    const samples = buildSliceSamples(lumenSlices);
    const nodes = collectNodes(samples);
    const candidates = buildCandidateEdges(samples, nodes, lumenField);
    const selection = selectCenterlineEdges(nodes, candidates);
    const edges = selection.edges;
    const degree = new Array(nodes.length).fill(0);
    for (const edge of edges) {
        degree[edge.start.nodeId]++;
        degree[edge.end.nodeId]++;
    }
    const segments = [];
    for (const edge of edges) {
        const start = edge.start.point;
        const end = edge.end.point;
        const length = start.distanceTo(end);
        if (length < 1e-4) continue;
        segments.push({
            start,
            end,
            nodeStartId: edge.start.nodeId,
            nodeEndId: edge.end.nodeId,
            radiusStart: edge.start.radius,
            radiusEnd: edge.end.radius,
            source: 'lumen-slices'
        });
    }
    let stubSegmentCount = 0;
    for (const node of nodes) {
        if (degree[node.nodeId] !== 0) continue;
        const stub = createIsolatedStubSegment(node, lumenField);
        if (!stub) continue;
        segments.push(stub);
        stubSegmentCount++;
    }
    segments.diagnostics = {
        candidateCount: candidates.length,
        edgeCount: edges.length,
        stubSegmentCount,
        ...selection.diagnostics,
        uncoveredNodeCount: Math.max(0, selection.diagnostics.isolatedNodeCount - stubSegmentCount),
        nodeCount: nodes.length,
        sliceCount: samples.length
    };
    return segments;
}

function lumenSliceBounds(lumenSlices) {
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    let minZ = Infinity;
    let maxZ = -Infinity;

    for (const slice of lumenSlices || []) {
        if (!Number.isFinite(slice?.y)) continue;
        for (const contour of slice.contours || []) {
            const bounds = contour.bounds;
            if (!bounds) continue;
            minX = Math.min(minX, bounds.minX);
            maxX = Math.max(maxX, bounds.maxX);
            minY = Math.min(minY, slice.y);
            maxY = Math.max(maxY, slice.y);
            minZ = Math.min(minZ, bounds.minZ);
            maxZ = Math.max(maxZ, bounds.maxZ);
        }
    }

    if (![minX, maxX, minY, maxY, minZ, maxZ].every(Number.isFinite)) return null;
    return { minX, maxX, minY, maxY, minZ, maxZ };
}

function volumeKey(ix, iy, iz) {
    return `${ix},${iy},${iz}`;
}

function buildVolumeCells(lumenField, lumenSlices, spacing = VOLUME_GRID_SPACING) {
    const bounds = lumenSliceBounds(lumenSlices);
    if (!bounds || !lumenField?.query) return { cells: [], cellMap: new Map(), bounds: null };

    const pad = spacing * 1.5;
    const minIx = Math.floor((bounds.minX - pad) / spacing);
    const maxIx = Math.ceil((bounds.maxX + pad) / spacing);
    const minIy = Math.floor((bounds.minY - pad) / spacing);
    const maxIy = Math.ceil((bounds.maxY + pad) / spacing);
    const minIz = Math.floor((bounds.minZ - pad) / spacing);
    const maxIz = Math.ceil((bounds.maxZ + pad) / spacing);
    const cells = [];
    const cellMap = new Map();

    for (let iy = minIy; iy <= maxIy; iy++) {
        const y = iy * spacing;
        for (let ix = minIx; ix <= maxIx; ix++) {
            const x = ix * spacing;
            for (let iz = minIz; iz <= maxIz; iz++) {
                const z = iz * spacing;
                const query = lumenField.query(new THREE.Vector3(x, y, z));
                const signedDistance = Number.isFinite(query?.signedDistance)
                    ? query.signedDistance
                    : -Infinity;
                if (signedDistance < VOLUME_INSIDE_MARGIN) continue;
                const cell = {
                    ix,
                    iy,
                    iz,
                    key: volumeKey(ix, iy, iz),
                    point: new THREE.Vector3(x, y, z),
                    signedDistance
                };
                cellMap.set(cell.key, cell);
                cells.push(cell);
            }
        }
    }

    return { cells, cellMap, bounds };
}

function volumeNeighbourCells(cell, cellMap) {
    const neighbours = [];
    for (let dx = -1; dx <= 1; dx++) {
        for (let dy = -1; dy <= 1; dy++) {
            for (let dz = -1; dz <= 1; dz++) {
                if (dx === 0 && dy === 0 && dz === 0) continue;
                const neighbour = cellMap.get(volumeKey(cell.ix + dx, cell.iy + dy, cell.iz + dz));
                if (neighbour) neighbours.push(neighbour);
            }
        }
    }
    return neighbours;
}

function volumeGraphNeighbours(cell, cellMap, extraNeighbours = null) {
    const neighbours = volumeNeighbourCells(cell, cellMap);
    const extra = extraNeighbours?.get(cell);
    if (!extra?.length) return neighbours;
    const seen = new Set(neighbours.map(neighbour => neighbour.key));
    for (const neighbour of extra) {
        if (seen.has(neighbour.key)) continue;
        seen.add(neighbour.key);
        neighbours.push(neighbour);
    }
    return neighbours;
}

function isVolumeRidgeCell(cell, cellMap) {
    if (cell.signedDistance < VOLUME_RIDGE_MIN_DISTANCE) return false;
    const neighbours = volumeNeighbourCells(cell, cellMap);
    if (neighbours.length < 2) return false;
    let maxNeighbourDistance = -Infinity;
    for (const neighbour of neighbours) {
        maxNeighbourDistance = Math.max(maxNeighbourDistance, neighbour.signedDistance);
    }
    return cell.signedDistance >= maxNeighbourDistance - VOLUME_RIDGE_TOLERANCE;
}

function addSkeletonSample(samples, seen, point, radius, source, forced = false) {
    const key = [
        Math.round(point.x * 2),
        Math.round(point.y * 2),
        Math.round(point.z * 2)
    ].join(',');
    if (seen.has(key)) return null;
    const sample = {
        nodeId: samples.length,
        point: point.clone(),
        radius: finiteRadius(radius, DEFAULT_MIN_RADIUS),
        source,
        forced
    };
    seen.add(key);
    samples.push(sample);
    return sample;
}

function contourAnchorSamples(lumenSlices, lumenField) {
    const anchors = [];
    const seen = new Set();
    for (const slice of buildSliceSamples(lumenSlices)) {
        for (const contour of slice.contours) {
            const query = lumenField?.query?.(contour.point);
            const radius = Math.max(
                contour.radius,
                Number.isFinite(query?.signedDistance) ? query.signedDistance : 0
            );
            const sample = addSkeletonSample(anchors, seen, contour.point, radius, 'volume-anchor', true);
            if (!sample) continue;
            sample.polygon = contour.polygon;
            sample.bounds = contour.bounds;
            sample.area = contour.area;
        }
    }
    return anchors;
}

function volumeSkeletonSamples(lumenField, lumenSlices) {
    const anchors = contourAnchorSamples(lumenSlices, lumenField);
    const samples = [];
    const seen = new Set();
    for (const anchor of anchors) {
        addSkeletonSample(samples, seen, anchor.point, anchor.radius, anchor.source, true);
    }

    const { cells, cellMap } = buildVolumeCells(lumenField, lumenSlices);
    const ridgeCells = cells
        .filter(cell => isVolumeRidgeCell(cell, cellMap))
        .sort((a, b) => b.signedDistance - a.signedDistance);
    const keptRidges = ridgeCells.slice(0, VOLUME_MAX_RIDGE_SAMPLES);
    for (const cell of keptRidges) {
        addSkeletonSample(samples, seen, cell.point, cell.signedDistance, 'volume-ridge');
    }

    return {
        samples,
        diagnostics: {
            anchorCount: anchors.length,
            insideCellCount: cells.length,
            ridgeCellCount: ridgeCells.length,
            keptRidgeCellCount: keptRidges.length
        }
    };
}

function buildSampleBuckets(samples, bucketSize) {
    const buckets = new Map();
    for (const sample of samples) {
        const ix = Math.floor(sample.point.x / bucketSize);
        const iy = Math.floor(sample.point.y / bucketSize);
        const iz = Math.floor(sample.point.z / bucketSize);
        const key = volumeKey(ix, iy, iz);
        let bucket = buckets.get(key);
        if (!bucket) {
            bucket = [];
            buckets.set(key, bucket);
        }
        bucket.push(sample);
    }
    return buckets;
}

function nearbySkeletonSamples(sample, buckets, bucketSize, radius) {
    const ix = Math.floor(sample.point.x / bucketSize);
    const iy = Math.floor(sample.point.y / bucketSize);
    const iz = Math.floor(sample.point.z / bucketSize);
    const span = Math.ceil(radius / bucketSize);
    const candidates = [];
    for (let dx = -span; dx <= span; dx++) {
        for (let dy = -span; dy <= span; dy++) {
            for (let dz = -span; dz <= span; dz++) {
                const bucket = buckets.get(volumeKey(ix + dx, iy + dy, iz + dz));
                if (!bucket) continue;
                for (const candidate of bucket) {
                    if (candidate.nodeId === sample.nodeId) continue;
                    const distance = sample.point.distanceTo(candidate.point);
                    if (distance > radius || distance < 1e-4) continue;
                    candidates.push({ sample: candidate, distance });
                }
            }
        }
    }
    candidates.sort((a, b) => a.distance - b.distance);
    return candidates;
}

function skeletonCandidate(a, b, distance, lumenField, longEdge = false) {
    const lumen = validateLumenSegment(a, b, lumenField, longEdge ? LUMEN_COVERAGE_MIN_MARGIN : LUMEN_BRIDGE_MIN_MARGIN);
    if (!lumen.valid) return null;
    const radiusScale = Math.max(1, Math.min(a.radius, b.radius));
    return {
        a,
        b,
        distance,
        minSignedDistance: lumen.minSignedDistance,
        cost:
            (longEdge ? 6 : 0) +
            distance / Math.max(1, radiusScale * 1.65) -
            Math.min(0.55, Math.max(0, lumen.minSignedDistance) / Math.max(1, radiusScale) * 0.08)
    };
}

function buildVolumeSkeletonCandidates(samples, lumenField) {
    const buckets = buildSampleBuckets(samples, VOLUME_EDGE_RADIUS);
    const candidates = [];
    const seen = new Set();
    const addCandidate = candidate => {
        if (!candidate) return;
        const a = Math.min(candidate.a.nodeId, candidate.b.nodeId);
        const b = Math.max(candidate.a.nodeId, candidate.b.nodeId);
        const key = `${a}->${b}`;
        if (seen.has(key)) return;
        seen.add(key);
        candidates.push(candidate);
    };

    for (const sample of samples) {
        const neighbours = nearbySkeletonSamples(sample, buckets, VOLUME_EDGE_RADIUS, VOLUME_EDGE_RADIUS)
            .slice(0, VOLUME_NEIGHBOR_LIMIT);
        for (const neighbour of neighbours) {
            addCandidate(skeletonCandidate(sample, neighbour.sample, neighbour.distance, lumenField));
        }
    }

    const degreeProbe = new Array(samples.length).fill(0);
    for (const candidate of candidates) {
        degreeProbe[candidate.a.nodeId]++;
        degreeProbe[candidate.b.nodeId]++;
    }

    const longBuckets = buildSampleBuckets(samples, VOLUME_LONG_EDGE_RADIUS);
    for (const sample of samples) {
        if (degreeProbe[sample.nodeId] > 0 && !sample.forced) continue;
        const neighbours = nearbySkeletonSamples(sample, longBuckets, VOLUME_LONG_EDGE_RADIUS, VOLUME_LONG_EDGE_RADIUS)
            .slice(0, VOLUME_NEIGHBOR_LIMIT);
        for (const neighbour of neighbours) {
            addCandidate(skeletonCandidate(sample, neighbour.sample, neighbour.distance, lumenField, true));
        }
    }

    return candidates.sort((a, b) => a.cost - b.cost);
}

function selectVolumeSkeletonEdges(samples, candidates) {
    const unionFind = new UnionFind(samples.length);
    const edges = [];
    for (const candidate of candidates) {
        if (!unionFind.union(candidate.a.nodeId, candidate.b.nodeId)) continue;
        edges.push({
            start: candidate.a,
            end: candidate.b,
            cost: candidate.cost,
            minSignedDistance: candidate.minSignedDistance
        });
    }
    return {
        edges,
        diagnostics: graphDiagnostics(samples, edges)
    };
}

function buildSegmentsFromVolumeSkeleton(lumenField, lumenSlices) {
    if (!lumenField?.query) return [];
    const { samples, diagnostics: sampleDiagnostics } = volumeSkeletonSamples(lumenField, lumenSlices);
    if (samples.length < 3) return [];
    const candidates = buildVolumeSkeletonCandidates(samples, lumenField);
    const selection = selectVolumeSkeletonEdges(samples, candidates);
    const degree = new Array(samples.length).fill(0);
    for (const edge of selection.edges) {
        degree[edge.start.nodeId]++;
        degree[edge.end.nodeId]++;
    }

    const segments = [];
    for (const edge of selection.edges) {
        const start = edge.start.point;
        const end = edge.end.point;
        const length = start.distanceTo(end);
        if (length < 1e-4) continue;
        segments.push({
            start,
            end,
            nodeStartId: edge.start.nodeId,
            nodeEndId: edge.end.nodeId,
            radiusStart: edge.start.radius,
            radiusEnd: edge.end.radius,
            source: 'volume-skeleton'
        });
    }

    let stubSegmentCount = 0;
    for (const sample of samples) {
        if (!sample.forced || degree[sample.nodeId] !== 0) continue;
        const stub = createIsolatedStubSegment(sample, lumenField);
        if (!stub) continue;
        stub.source = 'volume-anchor-stub';
        segments.push(stub);
        stubSegmentCount++;
    }

    segments.diagnostics = {
        ...sampleDiagnostics,
        candidateCount: candidates.length,
        edgeCount: selection.edges.length,
        stubSegmentCount,
        ...selection.diagnostics,
        uncoveredNodeCount: Math.max(0, selection.diagnostics.isolatedNodeCount - stubSegmentCount),
        nodeCount: samples.length,
        source: 'volume-skeleton'
    };
    return segments;
}

class MinHeap {
    constructor() {
        this.items = [];
    }

    get size() {
        return this.items.length;
    }

    push(item) {
        this.items.push(item);
        this.bubbleUp(this.items.length - 1);
    }

    pop() {
        if (!this.items.length) return null;
        const root = this.items[0];
        const last = this.items.pop();
        if (this.items.length) {
            this.items[0] = last;
            this.bubbleDown(0);
        }
        return root;
    }

    bubbleUp(index) {
        while (index > 0) {
            const parent = Math.floor((index - 1) / 2);
            if (this.items[parent].cost <= this.items[index].cost) break;
            const tmp = this.items[parent];
            this.items[parent] = this.items[index];
            this.items[index] = tmp;
            index = parent;
        }
    }

    bubbleDown(index) {
        while (true) {
            const left = index * 2 + 1;
            const right = left + 1;
            let smallest = index;
            if (left < this.items.length && this.items[left].cost < this.items[smallest].cost) {
                smallest = left;
            }
            if (right < this.items.length && this.items[right].cost < this.items[smallest].cost) {
                smallest = right;
            }
            if (smallest === index) break;
            const tmp = this.items[smallest];
            this.items[smallest] = this.items[index];
            this.items[index] = tmp;
            index = smallest;
        }
    }
}

function buildVolumeComponents(cells, cellMap, extraNeighbours = null) {
    for (let i = 0; i < cells.length; i++) {
        cells[i].id = i;
        cells[i].componentId = -1;
    }

    const components = [];
    for (const cell of cells) {
        if (cell.componentId !== -1) continue;
        const componentId = components.length;
        const component = [];
        const stack = [cell];
        cell.componentId = componentId;
        while (stack.length) {
            const current = stack.pop();
            component.push(current);
            for (const neighbour of volumeGraphNeighbours(current, cellMap, extraNeighbours)) {
                if (neighbour.componentId !== -1) continue;
                neighbour.componentId = componentId;
                stack.push(neighbour);
            }
        }
        components.push(component);
    }

    return components;
}

function chooseVolumeComponentRoot(component) {
    if (!component.length) return null;
    let maxY = -Infinity;
    for (const cell of component) maxY = Math.max(maxY, cell.point.y);
    const proximalBandMinY = maxY - VOLUME_GRID_SPACING * 5;
    let best = null;
    for (const cell of component) {
        if (cell.point.y < proximalBandMinY) continue;
        if (
            !best ||
            cell.signedDistance > best.signedDistance ||
            (cell.signedDistance === best.signedDistance && cell.point.y > best.point.y)
        ) {
            best = cell;
        }
    }
    if (best) return best;
    for (const cell of component) {
        if (!best || cell.signedDistance > best.signedDistance) best = cell;
    }
    return best;
}

function volumePathEdgeWeight(cell, neighbour, lumenField) {
    const validation = validateLumenSegment(cell, neighbour, lumenField, VOLUME_PATH_EDGE_MIN_MARGIN);
    if (!validation.valid) return Infinity;
    const clearance = Math.max(
        0.35,
        Math.min(cell.signedDistance, neighbour.signedDistance, validation.minSignedDistance)
    );
    const distance = cell.point.distanceTo(neighbour.point);
    return distance * (1 + VOLUME_PATH_CENTER_BIAS / clearance);
}

function runVolumeDijkstra(component, root, cellMap, lumenField, extraNeighbours = null) {
    const distances = new Map();
    const previous = new Map();
    const heap = new MinHeap();
    for (const cell of component) distances.set(cell.id, Infinity);
    distances.set(root.id, 0);
    heap.push({ cell: root, cost: 0 });

    while (heap.size) {
        const item = heap.pop();
        if (!item || item.cost !== distances.get(item.cell.id)) continue;
        for (const neighbour of volumeGraphNeighbours(item.cell, cellMap, extraNeighbours)) {
            if (neighbour.componentId !== item.cell.componentId) continue;
            const edgeWeight = volumePathEdgeWeight(item.cell, neighbour, lumenField);
            if (!Number.isFinite(edgeWeight)) continue;
            const nextCost = item.cost + edgeWeight;
            if (nextCost >= distances.get(neighbour.id)) continue;
            distances.set(neighbour.id, nextCost);
            previous.set(neighbour.id, item.cell.id);
            heap.push({ cell: neighbour, cost: nextCost });
        }
    }

    return { distances, previous };
}

function nearestValidVolumeCell(anchor, cells, lumenField, componentPaths = null) {
    let best = null;
    for (const cell of cells) {
        const distance = anchor.point.distanceTo(cell.point);
        if (distance > VOLUME_PATH_ANCHOR_SNAP_RADIUS) continue;
        const validation = validateLumenSegment(anchor, cell, lumenField, VOLUME_PATH_CONNECTOR_MIN_MARGIN);
        if (!validation.valid) continue;
        const pathState = componentPaths?.get(cell.componentId);
        const rank = pathState?.componentSize > 1 ? 0 : 1;
        if (
            best &&
            (rank > best.rank || (rank === best.rank && distance >= best.distance))
        ) {
            continue;
        }
        best = { cell, distance, validation, rank };
    }
    return best;
}

function appendAnchorVolumeCells(anchors, cells, cellMap, lumenField, spacing = VOLUME_GRID_SPACING) {
    let added = 0;
    for (const anchor of anchors) {
        const query = lumenField?.query?.(anchor.point);
        const signedDistance = Number.isFinite(query?.signedDistance)
            ? query.signedDistance
            : anchor.radius;
        if (signedDistance < VOLUME_PATH_CONNECTOR_MIN_MARGIN) continue;

        const baseIx = Math.round(anchor.point.x / spacing);
        const baseIy = Math.round(anchor.point.y / spacing);
        const baseIz = Math.round(anchor.point.z / spacing);
        let chosen = null;
        for (let radius = 0; radius <= 2 && !chosen; radius++) {
            for (let dx = -radius; dx <= radius && !chosen; dx++) {
                for (let dy = -radius; dy <= radius && !chosen; dy++) {
                    for (let dz = -radius; dz <= radius; dz++) {
                        if (Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) !== radius) continue;
                        const ix = baseIx + dx;
                        const iy = baseIy + dy;
                        const iz = baseIz + dz;
                        const key = volumeKey(ix, iy, iz);
                        if (cellMap.has(key)) continue;
                        chosen = { ix, iy, iz, key };
                        break;
                    }
                }
            }
        }
        if (!chosen) continue;

        const cell = {
            ...chosen,
            point: anchor.point.clone(),
            signedDistance: Math.max(signedDistance, anchor.radius * 0.75, DEFAULT_MIN_RADIUS),
            anchorNodeId: anchor.nodeId,
            source: 'volume-anchor-cell'
        };
        cellMap.set(cell.key, cell);
        cells.push(cell);
        added++;
    }
    return added;
}

function addExtraVolumeNeighbour(extraNeighbours, a, b) {
    let aList = extraNeighbours.get(a);
    if (!aList) {
        aList = [];
        extraNeighbours.set(a, aList);
    }
    let bList = extraNeighbours.get(b);
    if (!bList) {
        bList = [];
        extraNeighbours.set(b, bList);
    }
    aList.push(b);
    bList.push(a);
}

function buildAnchorVolumeEdges(cells, lumenField) {
    const anchorCells = cells.filter(cell => cell.source === 'volume-anchor-cell');
    const extraNeighbours = new Map();
    let edgeCount = 0;
    for (const anchorCell of anchorCells) {
        const candidates = [];
        for (const cell of cells) {
            if (cell === anchorCell) continue;
            const distance = anchorCell.point.distanceTo(cell.point);
            if (distance > VOLUME_ANCHOR_EDGE_RADIUS || distance < 1e-4) continue;
            candidates.push({ cell, distance });
        }
        candidates.sort((a, b) => a.distance - b.distance);

        let added = 0;
        for (const candidate of candidates) {
            if (added >= VOLUME_ANCHOR_EDGE_LIMIT) break;
            const validation = validateLumenSegment(
                anchorCell,
                candidate.cell,
                lumenField,
                VOLUME_PATH_EDGE_MIN_MARGIN
            );
            if (!validation.valid) continue;
            addExtraVolumeNeighbour(extraNeighbours, anchorCell, candidate.cell);
            added++;
            edgeCount++;
        }
    }
    extraNeighbours.edgeCount = edgeCount;
    return extraNeighbours;
}

function volumePathSegment(start, end, radiusStart, radiusEnd, source, nodeStartId = undefined, nodeEndId = undefined) {
    if (start.distanceTo(end) < 1e-4) return null;
    return {
        start: start.clone(),
        end: end.clone(),
        nodeStartId,
        nodeEndId,
        radiusStart: finiteRadius(radiusStart),
        radiusEnd: finiteRadius(radiusEnd),
        source
    };
}

function addRescueAnchorSegments({
    anchors,
    pendingAnchors,
    connectedAnchorIds,
    lumenField,
    segments
}) {
    const pendingIds = new Set(pendingAnchors.map(anchor => anchor.nodeId));
    const rescuedIds = new Set();
    const seenEdges = new Set();
    let rescueSegmentCount = 0;

    const addRescueEdge = (a, b) => {
        const low = Math.min(a.nodeId, b.nodeId);
        const high = Math.max(a.nodeId, b.nodeId);
        const key = `${low}->${high}`;
        if (seenEdges.has(key)) return false;
        const validation = validateLumenSegment(a, b, lumenField, VOLUME_PATH_CONNECTOR_MIN_MARGIN);
        if (!validation.valid) return false;
        const segment = volumePathSegment(
            a.point,
            b.point,
            a.radius,
            b.radius,
            'volume-anchor-rescue',
            a.nodeId,
            b.nodeId
        );
        if (!segment) return false;
        seenEdges.add(key);
        segments.push(segment);
        rescuedIds.add(a.nodeId);
        rescuedIds.add(b.nodeId);
        rescueSegmentCount++;
        return true;
    };

    for (const anchor of pendingAnchors) {
        const candidates = [];
        for (const candidate of anchors) {
            if (candidate.nodeId === anchor.nodeId) continue;
            const distance = anchor.point.distanceTo(candidate.point);
            if (distance > VOLUME_RESCUE_EDGE_RADIUS) continue;
            candidates.push({
                anchor: candidate,
                distance,
                connected: connectedAnchorIds.has(candidate.nodeId),
                pending: pendingIds.has(candidate.nodeId)
            });
        }
        candidates.sort((a, b) => {
            if (a.connected !== b.connected) return a.connected ? -1 : 1;
            if (a.pending !== b.pending) return a.pending ? -1 : 1;
            return a.distance - b.distance;
        });

        let added = 0;
        for (const candidate of candidates) {
            if (added >= VOLUME_RESCUE_EDGE_LIMIT) break;
            if (!addRescueEdge(anchor, candidate.anchor)) continue;
            added++;
        }
    }

    return { rescuedIds, rescueSegmentCount };
}

function buildSegmentsFromVolumePaths(lumenField, lumenSlices) {
    if (!lumenField?.query) return [];
    const anchors = contourAnchorSamples(lumenSlices, lumenField);
    const { cells, cellMap } = buildVolumeCells(lumenField, lumenSlices);
    if (!anchors.length || !cells.length) return [];
    const anchorCellCount = appendAnchorVolumeCells(anchors, cells, cellMap, lumenField);
    const extraNeighbours = buildAnchorVolumeEdges(cells, lumenField);

    const components = buildVolumeComponents(cells, cellMap, extraNeighbours);
    const componentPaths = new Map();
    for (const component of components) {
        const root = chooseVolumeComponentRoot(component);
        if (!root) continue;
        componentPaths.set(component[0].componentId, {
            root,
            componentSize: component.length,
            ...runVolumeDijkstra(component, root, cellMap, lumenField, extraNeighbours)
        });
    }

    const segments = [];
    const usedPathEdges = new Set();
    let targetAnchorCount = 0;
    let connectedAnchorCount = 0;
    let connectorSegmentCount = 0;
    let stubSegmentCount = 0;
    let pathEdgeCount = 0;
    let rescueSegmentCount = 0;
    let rescuedAnchorCount = 0;
    const connectedAnchorIds = new Set();
    const pendingStubAnchors = [];

    const addPathEdge = (a, b) => {
        const low = Math.min(a.id, b.id);
        const high = Math.max(a.id, b.id);
        const key = `${low}->${high}`;
        if (usedPathEdges.has(key)) return;
        usedPathEdges.add(key);
        const segment = volumePathSegment(
            a.point,
            b.point,
            a.signedDistance,
            b.signedDistance,
            'volume-path',
            a.id,
            b.id
        );
        if (!segment) return;
        segments.push(segment);
        pathEdgeCount++;
    };

    const addStub = anchor => {
        const stub = createIsolatedStubSegment(anchor, lumenField);
        if (!stub) return false;
        stub.source = 'volume-anchor-stub';
        segments.push(stub);
        stubSegmentCount++;
        return true;
    };

    for (const anchor of anchors) {
        const snap = nearestValidVolumeCell(anchor, cells, lumenField, componentPaths);
        if (!snap) {
            pendingStubAnchors.push(anchor);
            continue;
        }

        targetAnchorCount++;
        const pathState = componentPaths.get(snap.cell.componentId);
        if (!pathState || !Number.isFinite(pathState.distances.get(snap.cell.id))) {
            pendingStubAnchors.push(anchor);
            continue;
        }
        if (
            pathState.componentSize <= 1 &&
            snap.distance <= VOLUME_GRID_SPACING * 0.35 &&
            !pathState.previous.has(snap.cell.id)
        ) {
            pendingStubAnchors.push(anchor);
            continue;
        }

        connectedAnchorCount++;
        connectedAnchorIds.add(anchor.nodeId);
        if (snap.distance > VOLUME_GRID_SPACING * 0.35) {
            const connector = volumePathSegment(
                anchor.point,
                snap.cell.point,
                anchor.radius,
                snap.cell.signedDistance,
                'volume-anchor-connector',
                anchor.nodeId,
                snap.cell.id
            );
            if (connector) {
                segments.push(connector);
                connectorSegmentCount++;
            }
        }

        let currentId = snap.cell.id;
        while (pathState.previous.has(currentId)) {
            const previousId = pathState.previous.get(currentId);
            addPathEdge(cells[currentId], cells[previousId]);
            currentId = previousId;
        }
    }

    if (pendingStubAnchors.length) {
        const rescue = addRescueAnchorSegments({
            anchors,
            pendingAnchors: pendingStubAnchors,
            connectedAnchorIds,
            lumenField,
            segments
        });
        rescueSegmentCount = rescue.rescueSegmentCount;
        rescuedAnchorCount = rescue.rescuedIds.size;
        for (const anchor of pendingStubAnchors) {
            if (rescue.rescuedIds.has(anchor.nodeId)) continue;
            addStub(anchor);
        }
    }

    segments.diagnostics = {
        anchorCount: anchors.length,
        anchorCellCount,
        anchorExtraEdgeCount: extraNeighbours.edgeCount || 0,
        insideCellCount: cells.length,
        componentCount: components.length,
        targetAnchorCount,
        connectedAnchorCount,
        rescuedAnchorCount,
        connectorSegmentCount,
        rescueSegmentCount,
        stubSegmentCount,
        pathEdgeCount,
        edgeCount: pathEdgeCount + connectorSegmentCount + rescueSegmentCount,
        uncoveredNodeCount: Math.max(0, anchors.length - connectedAnchorCount - rescuedAnchorCount - stubSegmentCount),
        nodeCount: cells.length,
        source: 'volume-path'
    };
    return segments;
}

function buildSegmentsFromVessel(vessel, includeSheath) {
    const segments = [];
    for (const source of vessel?.segments || []) {
        if (source.isSheath && !includeSheath) continue;
        const start = toVector3(source.start);
        const end = toVector3(source.end);
        const length = start.distanceTo(end);
        if (length < 1e-4) continue;
        const radius = finiteRadius(source.radius, vessel?.radius || DEFAULT_MIN_RADIUS);
        segments.push({
            start,
            end,
            radiusStart: finiteRadius(source.radiusStart, radius),
            radiusEnd: finiteRadius(source.radiusEnd, radius),
            source: source.isSheath ? 'sheath' : 'vessel'
        });
    }
    return segments;
}

function buildSegmentsFromDirectCenterline(centerlineSegments) {
    const segments = [];
    for (const source of centerlineSegments || []) {
        const start = toVector3(source.start);
        const end = toVector3(source.end);
        const length = start.distanceTo(end);
        if (length < 1e-4) continue;
        const radius = finiteRadius(source.radius, DEFAULT_MIN_RADIUS);
        segments.push({
            start,
            end,
            nodeStartId: source.nodeStartId,
            nodeEndId: source.nodeEndId,
            radiusStart: finiteRadius(source.radiusStart, radius),
            radiusEnd: finiteRadius(source.radiusEnd, radius),
            source: source.source || 'direct-centerline'
        });
    }
    segments.diagnostics = centerlineSegments?.diagnostics || {
        source: 'direct-centerline',
        edgeCount: segments.length,
        nodeCount: 0,
        uncoveredNodeCount: 0
    };
    return segments;
}

function segmentAabb(segment, inflation) {
    const maxRadius = Math.max(segment.radiusStart, segment.radiusEnd) + inflation;
    const min = new THREE.Vector3(
        Math.min(segment.start.x, segment.end.x) - maxRadius,
        Math.min(segment.start.y, segment.end.y) - maxRadius,
        Math.min(segment.start.z, segment.end.z) - maxRadius
    );
    const max = new THREE.Vector3(
        Math.max(segment.start.x, segment.end.x) + maxRadius,
        Math.max(segment.start.y, segment.end.y) + maxRadius,
        Math.max(segment.start.z, segment.end.z) + maxRadius
    );
    return { min, max };
}

function cellKey(ix, iy, iz) {
    return `${ix},${iy},${iz}`;
}

function pointSegmentState(point, segment, margin = 0) {
    const rel = point.clone().sub(segment.start);
    const t = THREE.MathUtils.clamp(rel.dot(segment.axis) / Math.max(1e-8, segment.length), 0, 1);
    const center = segment.start.clone().lerp(segment.end, t);
    const radius = segment.radiusStart * (1 - t) + segment.radiusEnd * t + margin;
    const radialDistance = point.distanceTo(center);
    return {
        segment,
        t,
        center,
        radius,
        radialDistance,
        signedDistance: radius - radialDistance,
        hit: radialDistance <= radius
    };
}

export function createCenterlineCapsuleBroadPhase({
    vessel = null,
    centerlineSegments = null,
    lumenSlices = null,
    lumenField = null,
    includeSheath = false,
    inflation = DEFAULT_INFLATION,
    cellSize = DEFAULT_CELL_SIZE
} = {}) {
    const directSegments = buildSegmentsFromDirectCenterline(centerlineSegments);
    const volumePathSegments = directSegments.length
        ? []
        : buildSegmentsFromVolumePaths(lumenField, lumenSlices);
    const volumeSegments = directSegments.length || volumePathSegments.length
        ? volumePathSegments
        : buildSegmentsFromVolumeSkeleton(lumenField, lumenSlices);
    const sourceSegments = directSegments.length
        ? directSegments
        : volumeSegments.length
            ? volumeSegments
            : buildSegmentsFromLumenSlices(lumenSlices, lumenField);
    const diagnostics = sourceSegments.diagnostics || null;
    if (!sourceSegments.length) {
        sourceSegments.push(...buildSegmentsFromVessel(vessel, includeSheath));
    }

    const segments = sourceSegments.map((segment, index) => {
        const start = segment.start.clone();
        const end = segment.end.clone();
        const delta = end.clone().sub(start);
        const length = delta.length();
        const axis = length > 1e-8 ? delta.multiplyScalar(1 / length) : new THREE.Vector3(0, 1, 0);
        return {
            id: index,
            start,
            end,
            axis,
            length,
            radiusStart: finiteRadius(segment.radiusStart),
            radiusEnd: finiteRadius(segment.radiusEnd),
            nodeStartId: segment.nodeStartId,
            nodeEndId: segment.nodeEndId,
            source: segment.source || 'unknown',
            aabb: null
        };
    }).filter(segment => segment.length > 1e-4);

    const grid = new Map();
    const insertIntoGrid = segment => {
        segment.aabb = segmentAabb(segment, inflation);
        const minIx = Math.floor(segment.aabb.min.x / cellSize);
        const minIy = Math.floor(segment.aabb.min.y / cellSize);
        const minIz = Math.floor(segment.aabb.min.z / cellSize);
        const maxIx = Math.floor(segment.aabb.max.x / cellSize);
        const maxIy = Math.floor(segment.aabb.max.y / cellSize);
        const maxIz = Math.floor(segment.aabb.max.z / cellSize);
        for (let ix = minIx; ix <= maxIx; ix++) {
            for (let iy = minIy; iy <= maxIy; iy++) {
                for (let iz = minIz; iz <= maxIz; iz++) {
                    const key = cellKey(ix, iy, iz);
                    let bucket = grid.get(key);
                    if (!bucket) {
                        bucket = [];
                        grid.set(key, bucket);
                    }
                    bucket.push(segment.id);
                }
            }
        }
    };

    for (const segment of segments) insertIntoGrid(segment);

    function collectCandidateIds(point, margin = 0) {
        const span = Math.max(0, Math.ceil(margin / cellSize));
        const ix = Math.floor(point.x / cellSize);
        const iy = Math.floor(point.y / cellSize);
        const iz = Math.floor(point.z / cellSize);
        const ids = new Set();
        for (let dx = -span; dx <= span; dx++) {
            for (let dy = -span; dy <= span; dy++) {
                for (let dz = -span; dz <= span; dz++) {
                    const bucket = grid.get(cellKey(ix + dx, iy + dy, iz + dz));
                    if (!bucket) continue;
                    for (const id of bucket) ids.add(id);
                }
            }
        }
        return ids;
    }

    function queryPoint(input, margin = 0, out = null) {
        const point = toVector3(input);
        const ids = collectCandidateIds(point, margin + inflation);
        const candidates = out?.candidates || [];
        candidates.length = 0;
        let nearest = null;

        for (const id of ids) {
            const state = pointSegmentState(point, segments[id], margin);
            if (!nearest || state.signedDistance > nearest.signedDistance) nearest = state;
            if (state.hit) candidates.push(state);
        }

        if (!nearest) {
            for (const segment of segments) {
                const state = pointSegmentState(point, segment, margin);
                if (!nearest || state.signedDistance > nearest.signedDistance) nearest = state;
            }
        }
        if (!candidates.length && nearest?.hit) {
            candidates.push(nearest);
        }

        const result = out || {};
        result.hit = candidates.length > 0;
        result.candidates = candidates;
        result.nearest = nearest;
        result.candidateCount = ids.size;
        return result;
    }

    function querySegment(from, to, margin = 0, out = null) {
        const start = toVector3(from);
        const end = toVector3(to);
        const delta = end.clone().sub(start);
        const length = delta.length();
        const samples = Math.max(2, Math.ceil(length / Math.max(4, cellSize * 0.45)));
        const result = out || { hits: [] };
        result.hits = result.hits || [];
        result.hits.length = 0;
        let hit = false;
        for (let i = 0; i <= samples; i++) {
            const point = start.clone().addScaledVector(delta, i / samples);
            const sample = queryPoint(point, margin);
            if (!sample.hit) continue;
            hit = true;
            result.hits.push({
                t: i / samples,
                point,
                candidates: sample.candidates
            });
        }
        result.hit = hit;
        return result;
    }

    return {
        type: 'centerline-capsule-broadphase',
        source: sourceSegments[0]?.source || 'none',
        diagnostics,
        inflation,
        cellSize,
        segments,
        grid,
        queryPoint,
        querySegment
    };
}

function orientAlongSegment(object, segment) {
    object.position.copy(segment.start).lerp(segment.end, 0.5);
    object.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), segment.axis);
}

function addCapsuleMesh(group, segment, material, endMaterial) {
    const cylinder = new THREE.Mesh(
        new THREE.CylinderGeometry(
            segment.radiusEnd,
            segment.radiusStart,
            segment.length,
            18,
            1,
            true
        ),
        material
    );
    orientAlongSegment(cylinder, segment);
    cylinder.renderOrder = 4.5;
    cylinder.userData.debugLayer = 'capsules';
    group.add(cylinder);

    const startSphere = new THREE.Mesh(
        new THREE.SphereGeometry(segment.radiusStart, 16, 8),
        endMaterial
    );
    startSphere.position.copy(segment.start);
    startSphere.renderOrder = 4.4;
    startSphere.userData.debugLayer = 'capsules';
    group.add(startSphere);

    const endSphere = new THREE.Mesh(
        new THREE.SphereGeometry(segment.radiusEnd, 16, 8),
        endMaterial
    );
    endSphere.position.copy(segment.end);
    endSphere.renderOrder = 4.4;
    endSphere.userData.debugLayer = 'capsules';
    group.add(endSphere);
}

function roundedPointKey(point) {
    return [
        Math.round(point.x * 8),
        Math.round(point.y * 8),
        Math.round(point.z * 8)
    ].join(',');
}

function addCenterlineNode(nodes, key, point, radius) {
    let node = nodes.get(key);
    if (!node) {
        node = {
            point: new THREE.Vector3(),
            radius: 0,
            degree: 0,
            weight: 0
        };
        nodes.set(key, node);
    }
    node.point.add(point);
    node.radius = Math.max(node.radius, finiteRadius(radius));
    node.degree++;
    node.weight++;
}

function collectCenterlineNodes(segments) {
    const nodes = new Map();
    for (const segment of segments) {
        if (segment.nodeStartId !== undefined && segment.nodeStartId === segment.nodeEndId) {
            const midpoint = segment.start.clone().lerp(segment.end, 0.5);
            addCenterlineNode(
                nodes,
                `node:${segment.nodeStartId}`,
                midpoint,
                Math.max(segment.radiusStart, segment.radiusEnd)
            );
            continue;
        }

        const startKey = segment.nodeStartId !== undefined
            ? `node:${segment.nodeStartId}:${roundedPointKey(segment.start)}`
            : `point:${roundedPointKey(segment.start)}`;
        const endKey = segment.nodeEndId !== undefined
            ? `node:${segment.nodeEndId}:${roundedPointKey(segment.end)}`
            : `point:${roundedPointKey(segment.end)}`;
        addCenterlineNode(nodes, startKey, segment.start, segment.radiusStart);
        addCenterlineNode(nodes, endKey, segment.end, segment.radiusEnd);
    }

    return [...nodes.values()].map(node => ({
        ...node,
        point: node.point.multiplyScalar(1 / Math.max(1, node.weight))
    }));
}

function createCenterlineNodeMesh(nodes, {
    radius,
    color,
    opacity,
    renderOrder
}) {
    if (!nodes.length) return null;
    const mesh = new THREE.InstancedMesh(
        new THREE.SphereGeometry(radius, 10, 6),
        new THREE.MeshBasicMaterial({
            color,
            transparent: true,
            opacity,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        }),
        nodes.length
    );
    const matrix = new THREE.Matrix4();
    const scale = new THREE.Vector3();
    for (let i = 0; i < nodes.length; i++) {
        const nodeRadius = Math.max(0.72, Math.min(1.65, nodes[i].radius * 0.16));
        scale.setScalar(nodeRadius);
        matrix.compose(nodes[i].point, new THREE.Quaternion(), scale);
        mesh.setMatrixAt(i, matrix);
    }
    mesh.frustumCulled = false;
    mesh.instanceMatrix.needsUpdate = true;
    mesh.renderOrder = renderOrder;
    mesh.userData.debugLayer = 'centerline';
    return mesh;
}

export function createBroadPhaseDebugGroup(broadPhase, {
    maxCapsules = DEFAULT_MAX_DEBUG_CAPSULES
} = {}) {
    const group = new THREE.Group();
    if (!broadPhase?.segments?.length) return group;

    const lineSegments = broadPhase.segments;
    const linePositions = new Float32Array(lineSegments.length * 6);
    for (let i = 0; i < lineSegments.length; i++) {
        const segment = lineSegments[i];
        linePositions[i * 6] = segment.start.x;
        linePositions[i * 6 + 1] = segment.start.y;
        linePositions[i * 6 + 2] = segment.start.z;
        linePositions[i * 6 + 3] = segment.end.x;
        linePositions[i * 6 + 4] = segment.end.y;
        linePositions[i * 6 + 5] = segment.end.z;
    }

    const lineGeometry = new THREE.BufferGeometry();
    lineGeometry.setAttribute('position', new THREE.BufferAttribute(linePositions, 3));
    const lines = new THREE.LineSegments(
        lineGeometry,
        new THREE.LineBasicMaterial({
            color: CENTERLINE_COLOR,
            transparent: true,
            opacity: 0.96,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        })
    );
    lines.frustumCulled = false;
    lines.renderOrder = 9.7;
    lines.userData.debugLayer = 'centerline';
    group.add(lines);

    const centerlineNodes = collectCenterlineNodes(lineSegments);
    const regularNodes = centerlineNodes.filter(node => node.degree === 2);
    const branchNodes = centerlineNodes.filter(node => node.degree !== 2);
    const regularNodeMesh = createCenterlineNodeMesh(regularNodes, {
        radius: CENTERLINE_NODE_RADIUS,
        color: CENTERLINE_NODE_COLOR,
        opacity: 0.92,
        renderOrder: 9.78
    });
    if (regularNodeMesh) group.add(regularNodeMesh);
    const branchNodeMesh = createCenterlineNodeMesh(branchNodes, {
        radius: CENTERLINE_BRANCH_NODE_RADIUS,
        color: CENTERLINE_BRANCH_NODE_COLOR,
        opacity: 0.98,
        renderOrder: 9.82
    });
    if (branchNodeMesh) group.add(branchNodeMesh);

    const capsuleStep = Number.isFinite(maxCapsules) && maxCapsules > 0
        ? Math.max(1, Math.ceil(broadPhase.segments.length / maxCapsules))
        : 1;
    const shownSegments = broadPhase.segments.filter((_, index) => index % capsuleStep === 0);
    const capsuleMaterial = new THREE.MeshBasicMaterial({
        color: CAPSULE_COLOR,
        transparent: true,
        opacity: 0.105,
        depthTest: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        toneMapped: false
    });
    const endMaterial = capsuleMaterial.clone();
    endMaterial.opacity = 0.075;
    for (const segment of shownSegments) {
        addCapsuleMesh(group, segment, capsuleMaterial, endMaterial);
    }

    const edgePositions = new Float32Array(shownSegments.length * 6);
    for (let i = 0; i < shownSegments.length; i++) {
        const segment = shownSegments[i];
        edgePositions[i * 6] = segment.start.x;
        edgePositions[i * 6 + 1] = segment.start.y;
        edgePositions[i * 6 + 2] = segment.start.z;
        edgePositions[i * 6 + 3] = segment.end.x;
        edgePositions[i * 6 + 4] = segment.end.y;
        edgePositions[i * 6 + 5] = segment.end.z;
    }
    const edgeGeometry = new THREE.BufferGeometry();
    edgeGeometry.setAttribute('position', new THREE.BufferAttribute(edgePositions, 3));
    const edges = new THREE.LineSegments(
        edgeGeometry,
        new THREE.LineBasicMaterial({
            color: CAPSULE_EDGE_COLOR,
            transparent: true,
            opacity: 0.38,
            depthTest: false,
            depthWrite: false,
            toneMapped: false
        })
    );
    edges.frustumCulled = false;
    edges.renderOrder = 9.55;
    edges.userData.debugLayer = 'capsules';
    group.add(edges);

    group.userData.broadPhase = broadPhase;
    group.userData.displayedSegmentCount = shownSegments.length;
    group.userData.centerlineNodeCount = centerlineNodes.length;
    group.userData.centerlineBranchNodeCount = branchNodes.length;
    return group;
}
