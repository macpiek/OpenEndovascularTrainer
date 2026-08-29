import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { buildMedialCenterlineTree } from './medialCenterlineTree.js';

const DEFAULT_TARGET_SLICE_COUNT = 1000;
const DEFAULT_SLICE_SPACING = null;
const DEFAULT_CONTOUR_TOLERANCE = 0.06;
const DEFAULT_MIN_LUMEN_AREA = 2;
const DEFAULT_MIN_COMPACTNESS = 0.1;
const DEFAULT_MAX_LINK_GAP = 12;
const DEFAULT_SECONDARY_AXIS_SPACING_MULTIPLIER = 1.5;
const DEFAULT_AXIS_SLICE_SPACING_MULTIPLIERS = {
    y: 1,
    x: 1,
    z: 1
};
const DEFAULT_SECONDARY_AXIS_MAX_RADIUS = 18;
const DEFAULT_CENTERLINE_NODE_SPACING = 2;
const DEFAULT_CENTERLINE_MIN_LUMEN_AREA = 0.5;
const DEFAULT_CENTERLINE_MIN_COMPACTNESS = 0.04;
const DEFAULT_MAX_ADAPTIVE_DIRECTIONS = 4;
const DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES = 24;
const DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES = 0;
const DEFAULT_CENTERLINE_REFINE_ITERATIONS = 5;
const DEFAULT_CENTERLINE_REFINE_PASSES = 2;
const DEFAULT_CENTERLINE_LEAF_PRUNE_PASSES = 4;
const DEFAULT_CENTERLINE_CLEARANCE_RELAX_PASSES = 0;
const CENTERLINE_CLEARANCE_RELAX_DIRECTIONS = 16;
const CENTERLINE_CLEARANCE_RELAX_MIN_GAIN = 0.04;
const CENTERLINE_OUTLIER_RELAX_PASSES = 1;
const CENTERLINE_OUTLIER_PATCH_PASSES = 0;
const CENTERLINE_OUTLIER_REROUTE_PASSES = 1;
const CENTERLINE_OUTLIER_REROUTE_MAX_CANDIDATES = 18;
const CENTERLINE_OUTLIER_REROUTE_SIDE_STEPS = 12;
const CENTERLINE_OUTLIER_REROUTE_MIN_SPAN = 8;
const CENTERLINE_OUTLIER_REROUTE_MAX_SPAN = 92;
const CENTERLINE_OUTLIER_ENABLE_VIA_CENTER_REROUTE = true;
const CENTERLINE_OUTLIER_USE_CHAIN_ROUTE_BOUNDS = true;
const CENTERLINE_OUTLIER_CHAIN_PATCH_PASSES = 6;
const CENTERLINE_OUTLIER_CHAIN_PATCH_MAX_CANDIDATES = 12;
const CENTERLINE_OUTLIER_CHAIN_PATCH_SIDE_STEPS = 5;
const CENTERLINE_OUTLIER_CHAIN_PATCH_MIN_NORMALIZED = 0.32;
const CENTERLINE_OUTLIER_CHAIN_PATCH_MEASURED = true;
const CENTERLINE_OUTLIER_LATE_CHAIN_PATCH_PASSES = 8;
const CENTERLINE_FINAL_CHAIN_PATCH_PASSES = 32;
const CENTERLINE_FINAL_REROUTE_PASSES = 6;
const CENTERLINE_OUTLIER_RECONNECT_PASSES = 1;
const CENTERLINE_OUTLIER_REFERENCE_FIELD_PASSES = 1;
const CENTERLINE_OUTLIER_REFERENCE_RECONNECT_PASSES = 1;
const CENTERLINE_OUTLIER_RECONNECT_MAX_CANDIDATES = 3;
const CENTERLINE_OUTLIER_RECONNECT_SIDE_STEPS = 6;
const CENTERLINE_OUTLIER_RECONNECT_MAX_SPAN = 70;
const CENTERLINE_OUTLIER_RECONNECT_MIN_OFFSET = 3.5;
const CENTERLINE_OUTLIER_RECONNECT_MIN_NORMALIZED = 0.55;
const CENTERLINE_OUTLIER_RELAX_MAX_CANDIDATES = 32;
const CENTERLINE_OUTLIER_RELAX_MIN_OFFSET = 1.25;
const CENTERLINE_OUTLIER_RELAX_MIN_NORMALIZED = 0.55;
const CENTERLINE_BACKTRACK_MIN_DEFLECTION_DEG = 90;
const CENTERLINE_BACKTRACK_MAX_PASSES = 10;
const CENTERLINE_INVALID_REROUTE_MAX_CHAINS = 64;
const OBLIQUE_ARTIFACT_MIN_RADIUS = 9;
const OBLIQUE_ARTIFACT_MIN_ALIGNMENT = 0.18;
const TREE_COVER_MARGIN = 1.5;
const BRANCH_ATTACH_MARGIN = 14;
const FINAL_BRANCH_ATTACH_MARGIN = 36;
const DEFAULT_BRANCH_ORIGIN_MAX_LENGTH = Infinity;
const DEFAULT_CONNECTOR_LUMEN_CLEARANCE = -0.05;
const CONNECTOR_LUMEN_SAMPLE_SPACING = 2.2;
const BRANCH_ATTACHMENT_CENTERING_WEIGHT = 0.55;
const BRANCH_ATTACHMENT_DISTANCE_WEIGHT = 0.08;
const BRANCH_ATTACHMENT_SCORE_EPSILON = 0.02;
const BRANCH_ATTACHMENT_LOW_CLEARANCE_FRACTION = 0.24;
const BRANCH_ATTACHMENT_MIN_CLEARANCE_GAIN = 0.65;
const BRANCH_ROUTE_MIN_LENGTH = 7.5;
const BRANCH_ROUTE_GRID_SPACING = 3.2;
const BRANCH_ROUTE_MAX_CELLS = 1400;
const BRANCH_ROUTE_MAX_SEGMENTS = 120;
const BRANCH_ROUTE_PADDING = 5.5;
const BRANCH_ROUTE_CONNECT_RADIUS = 7;
const BRANCH_ROUTE_CENTER_BIAS = 4.5;
const BRANCH_ROUTE_RELAX_PASSES = 0;
const BRANCH_ROUTE_CANDIDATE_RELAX_PASSES = 2;
const BRANCH_ROUTE_RELAX_MIN_GAIN = 0.05;
const DEBUG_MAX_CONTOUR_SEGMENTS = 18000;
const LUMEN_CAST_RING_SEGMENTS = 28;
const LUMEN_CAST_GRID_SPACING = 2.4;
const LUMEN_CAST_ISO_LEVEL = -0.35;
const DEFAULT_LUMEN_CAST_TAG_TARGET_SLICE_COUNT = DEFAULT_TARGET_SLICE_COUNT;
const MIN_RADIUS = 1.2;
const SPHERE_DIRECTION_CACHE = new Map();

function finiteRadius(value, fallback = MIN_RADIUS) {
    return Number.isFinite(value) && value > 0 ? value : fallback;
}

function nowMs() {
    return typeof performance !== 'undefined' && typeof performance.now === 'function'
        ? performance.now()
        : Date.now();
}

function directionAxis(id, x, y, z) {
    return {
        id,
        direction: new THREE.Vector3(x, y, z).normalize()
    };
}

const CENTERLINE_AXES = [
    'y',
    'x',
    'z',
    directionAxis('xy+', 1, 1, 0),
    directionAxis('xy-', 1, -1, 0),
    directionAxis('xz+', 1, 0, 1),
    directionAxis('xz-', 1, 0, -1),
    directionAxis('yz+', 0, 1, 1),
    directionAxis('yz-', 0, 1, -1),
    directionAxis('xyz+++', 1, 1, 1),
    directionAxis('xyz++-', 1, 1, -1),
    directionAxis('xyz+-+', 1, -1, 1),
    directionAxis('xyz-++', -1, 1, 1)
];

function resolveWallBvh(geometry, wallBvh) {
    if (wallBvh) return wallBvh;
    if (geometry?.boundsTree) return geometry.boundsTree;
    if (!geometry?.attributes?.position) return null;
    geometry.boundsTree = new MeshBVH(geometry);
    return geometry.boundsTree;
}

function segmentIntersectsWall(segment, wallBvh) {
    if (!wallBvh?.raycastFirst) return false;
    const delta = new THREE.Vector3().subVectors(segment.end, segment.start);
    const length = delta.length();
    if (length < 1e-5) return false;
    const direction = delta.multiplyScalar(1 / length);
    const ray = new THREE.Ray(segment.start, direction);
    const hit = wallBvh.raycastFirst(ray, THREE.DoubleSide, 1e-4, Math.max(1e-4, length - 1e-4));
    return Boolean(hit);
}

function segmentStaysInsideLumen(segment, lumenField, minSignedDistance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE) {
    if (!lumenField?.query) return true;
    const length = segment.start.distanceTo(segment.end);
    const sampleCount = Math.max(2, Math.ceil(length / CONNECTOR_LUMEN_SAMPLE_SPACING));
    const point = new THREE.Vector3();
    for (let i = 0; i <= sampleCount; i++) {
        const t = i / sampleCount;
        point.lerpVectors(segment.start, segment.end, t);
        const query = lumenField.query(point);
        if (!query || query.signedDistance < minSignedDistance) return false;
    }
    return true;
}

function connectorStaysInsideVessel(
    segment,
    wallBvh,
    lumenField,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE
) {
    if (segmentIntersectsWall(segment, wallBvh)) return false;
    return wallBvh
        ? true
        : segmentStaysInsideLumen(segment, lumenField, connectorLumenClearance);
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
            const swap = this.items[parent];
            this.items[parent] = this.items[index];
            this.items[index] = swap;
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
            const swap = this.items[smallest];
            this.items[smallest] = this.items[index];
            this.items[index] = swap;
            index = smallest;
        }
    }
}

function branchRouteKey(ix, iy, iz) {
    return `${ix},${iy},${iz}`;
}

function branchRouteEdgeValid(a, b, lumenField, wallBvh, connectorLumenClearance) {
    const segment = {
        start: a.point,
        end: b.point
    };
    return segmentStaysInsideLumen(segment, lumenField, connectorLumenClearance);
}

function branchRouteEdgeWeight(
    a,
    b,
    lumenField,
    wallBvh,
    connectorLumenClearance,
    validateEdge = false,
    allowWallWhenInside = false
) {
    const distance = a.point.distanceTo(b.point);
    if (validateEdge) {
        const edge = { start: a.point, end: b.point };
        const inside = segmentStaysInsideLumen(
            edge,
            lumenField,
            connectorLumenClearance
        );
        if (!inside) return Infinity;
        if (segmentIntersectsWall(edge, wallBvh)) {
            const certifiedInside = allowWallWhenInside && segmentStaysInsideLumen(
                edge,
                lumenField,
                Math.max(0.12, connectorLumenClearance + 0.18)
            );
            if (!certifiedInside) return Infinity;
        }
    }
    const clearance = Math.max(
        0.25,
        Math.min(
            a.clearance,
            b.clearance,
            Number.isFinite(a.lumenClearance) ? a.lumenClearance : Infinity,
            Number.isFinite(b.lumenClearance) ? b.lumenClearance : Infinity
        )
    );
    return distance * (1 + BRANCH_ROUTE_CENTER_BIAS / clearance);
}

function branchRouteNode(point, clearance, id, key = null) {
    return {
        id,
        key,
        point: point.clone(),
        clearance: Math.max(0.25, clearance)
    };
}

function buildBranchRouteNodes(origin, lumenField, connectorLumenClearance) {
    const length = origin.start.distanceTo(origin.end);
    const maxRadius = Math.max(origin.radiusStart || 0, origin.radiusEnd || 0);
    let spacing = Number.isFinite(origin.routeGridSpacing) && origin.routeGridSpacing > 0
        ? origin.routeGridSpacing
        : BRANCH_ROUTE_GRID_SPACING;
    const maxCells = Number.isFinite(origin.routeMaxCells) && origin.routeMaxCells > 0
        ? origin.routeMaxCells
        : BRANCH_ROUTE_MAX_CELLS;
    const minimumPadding = Number.isFinite(origin.routeMinimumPadding)
        ? Math.max(0.5, origin.routeMinimumPadding)
        : BRANCH_ROUTE_PADDING;
    const radiusPaddingScale = Number.isFinite(origin.routeRadiusPaddingScale)
        ? Math.max(0, origin.routeRadiusPaddingScale)
        : 0.75;
    const radiusPaddingBase = Number.isFinite(origin.routeRadiusPaddingBase)
        ? Math.max(0, origin.routeRadiusPaddingBase)
        : 2;
    const padding = Math.max(
        minimumPadding,
        Math.min(22, maxRadius * radiusPaddingScale + radiusPaddingBase),
        Number.isFinite(origin.routePadding) ? origin.routePadding : 0
    );
    const boundsPoints = [origin.start, origin.end];
    if (Array.isArray(origin.routeBoundsPoints)) {
        for (const point of origin.routeBoundsPoints) {
            if (
                Number.isFinite(point?.x) &&
                Number.isFinite(point?.y) &&
                Number.isFinite(point?.z)
            ) {
                boundsPoints.push(point);
            }
        }
    }
    const min = new THREE.Vector3(Infinity, Infinity, Infinity);
    const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    for (const point of boundsPoints) {
        min.min(point);
        max.max(point);
    }
    min.addScalar(-padding);
    max.addScalar(padding);
    let nx = Math.ceil((max.x - min.x) / spacing) + 1;
    let ny = Math.ceil((max.y - min.y) / spacing) + 1;
    let nz = Math.ceil((max.z - min.z) / spacing) + 1;
    const estimatedCells = nx * ny * nz;
    if (estimatedCells > maxCells) {
        spacing *= Math.cbrt(estimatedCells / maxCells);
        nx = Math.ceil((max.x - min.x) / spacing) + 1;
        ny = Math.ceil((max.y - min.y) / spacing) + 1;
        nz = Math.ceil((max.z - min.z) / spacing) + 1;
    }

    const nodes = [];
    const nodeMap = new Map();
    const startQuery = lumenField.query(origin.start);
    const endQuery = lumenField.query(origin.end);
    const start = branchRouteNode(
        origin.start,
        Math.max(origin.radiusStart || 0, startQuery?.signedDistance || 0, 0.5),
        0,
        'start'
    );
    const end = branchRouteNode(
        origin.end,
        Math.max(origin.radiusEnd || 0, endQuery?.signedDistance || 0, 0.5),
        1,
        'end'
    );
    nodes.push(start, end);

    const point = new THREE.Vector3();
    for (let ix = 0; ix < nx; ix++) {
        point.x = min.x + ix * spacing;
        for (let iy = 0; iy < ny; iy++) {
            point.y = min.y + iy * spacing;
            for (let iz = 0; iz < nz; iz++) {
                point.z = min.z + iz * spacing;
                const query = lumenField.query(point);
                const signedDistance = Number.isFinite(query?.signedDistance)
                    ? query.signedDistance
                    : -Infinity;
                if (signedDistance < connectorLumenClearance) continue;
                const key = branchRouteKey(ix, iy, iz);
                const node = branchRouteNode(point, signedDistance, nodes.length, key);
                node.ix = ix;
                node.iy = iy;
                node.iz = iz;
                nodeMap.set(key, node);
                nodes.push(node);
            }
        }
    }

    return { nodes, nodeMap, spacing, length };
}

function branchRouteNeighbours(node, route) {
    const neighbours = [];
    if (node.key === 'start' || node.key === 'end') {
        const radius = Math.max(BRANCH_ROUTE_CONNECT_RADIUS, route.spacing * 2.25);
        const radiusSq = radius * radius;
        for (let i = 2; i < route.nodes.length; i++) {
            const candidate = route.nodes[i];
            if (candidate.point.distanceToSquared(node.point) <= radiusSq) neighbours.push(candidate);
        }
        if (route.nodes[0].point.distanceTo(route.nodes[1].point) <= radius) {
            neighbours.push(node.key === 'start' ? route.nodes[1] : route.nodes[0]);
        }
        neighbours.sort((a, b) => a.point.distanceToSquared(node.point) - b.point.distanceToSquared(node.point));
        return neighbours.slice(0, 24);
    }

    for (let dx = -1; dx <= 1; dx++) {
        for (let dy = -1; dy <= 1; dy++) {
            for (let dz = -1; dz <= 1; dz++) {
                if (dx === 0 && dy === 0 && dz === 0) continue;
                const neighbour = route.nodeMap.get(branchRouteKey(node.ix + dx, node.iy + dy, node.iz + dz));
                if (neighbour) neighbours.push(neighbour);
            }
        }
    }

    const startDistanceSq = node.point.distanceToSquared(route.nodes[0].point);
    const endDistanceSq = node.point.distanceToSquared(route.nodes[1].point);
    const connectRadius = Math.max(BRANCH_ROUTE_CONNECT_RADIUS, route.spacing * 2.25);
    const connectRadiusSq = connectRadius * connectRadius;
    if (startDistanceSq <= connectRadiusSq) neighbours.push(route.nodes[0]);
    if (endDistanceSq <= connectRadiusSq) neighbours.push(route.nodes[1]);
    return neighbours;
}

function branchRouteRelaxationDirections() {
    if (branchRouteRelaxationDirections.cache) return branchRouteRelaxationDirections.cache;
    const directions = [];
    for (let x = -1; x <= 1; x++) {
        for (let y = -1; y <= 1; y++) {
            for (let z = -1; z <= 1; z++) {
                if (x === 0 && y === 0 && z === 0) continue;
                directions.push(new THREE.Vector3(x, y, z).normalize());
            }
        }
    }
    branchRouteRelaxationDirections.cache = directions;
    return directions;
}

function branchRoutePointClearance(point, lumenField) {
    const query = lumenField?.query?.(point);
    return Number.isFinite(query?.signedDistance) ? query.signedDistance : -Infinity;
}

function bestContourAtSlicePoint(slice, x, z) {
    if (!slice?.contours?.length) return null;
    let best = null;
    const point = { x, z };
    for (const contour of slice.contours) {
        if (!contour?.polygon?.length) continue;
        const distance = Math.sqrt(distanceToPolygonSq(point, contour.polygon));
        const inside = pointInPolygonCoords(x, z, contour.polygon);
        const signedDistance = inside ? distance : -distance;
        if (!best || signedDistance > best.signedDistance) {
            best = {
                contour,
                signedDistance
            };
        }
    }
    return best;
}

function contourMedialPoint2D(contour) {
    if (!contour) return null;
    const source = contour.sample || contour.center || contour.linkCenter || contour.centroid;
    if (Number.isFinite(source?.x) && Number.isFinite(source?.z)) {
        return { x: source.x, z: source.z };
    }
    if (!contour.polygon?.length) return null;
    const bounds = contour.bounds || polygonBounds(contour.polygon);
    return bestInteriorPoint(contour.polygon, polygonCentroid(contour.polygon), bounds);
}

function referenceFieldMedialTarget(point, referenceField) {
    if (!referenceField?.query) return null;
    const query = referenceField.query(point);
    const lowerSlice = query?.lowerSlice;
    const upperSlice = query?.upperSlice;
    if (!lowerSlice?.contours?.length || !upperSlice?.contours?.length) return null;

    const lower = bestContourAtSlicePoint(lowerSlice, point.x, point.z);
    const upper = bestContourAtSlicePoint(upperSlice, point.x, point.z);
    if (!lower || !upper) return null;

    const lowerCenter = contourMedialPoint2D(lower.contour);
    const upperCenter = contourMedialPoint2D(upper.contour);
    if (!lowerCenter || !upperCenter) return null;

    const span = upperSlice.y - lowerSlice.y;
    const t = Math.abs(span) < 1e-6
        ? 0
        : THREE.MathUtils.clamp((point.y - lowerSlice.y) / span, 0, 1);
    return {
        point: new THREE.Vector3(
            THREE.MathUtils.lerp(lowerCenter.x, upperCenter.x, t),
            point.y,
            THREE.MathUtils.lerp(lowerCenter.z, upperCenter.z, t)
        ),
        signedDistance: Number.isFinite(query?.signedDistance) ? query.signedDistance : -Infinity,
        lowerSignedDistance: lower.signedDistance,
        upperSignedDistance: upper.signedDistance
    };
}

function branchRoutePointIsConnected(path, index, point, lumenField, connectorLumenClearance) {
    const scratch = new THREE.Vector3();
    for (const neighbour of [path[index - 1].point, path[index + 1].point]) {
        for (const t of [0.35, 0.65]) {
            scratch.copy(neighbour).lerp(point, t);
            if (branchRoutePointClearance(scratch, lumenField) < connectorLumenClearance) return false;
        }
    }
    return true;
}

function relaxBranchRoutePath(
    path,
    lumenField,
    connectorLumenClearance,
    spacing,
    passCount = BRANCH_ROUTE_RELAX_PASSES
) {
    if (!lumenField?.query || path.length < 3) {
        return {
            relaxedNodeCount: 0,
            totalClearanceGain: 0,
            maxClearanceGain: 0
        };
    }

    const directions = branchRouteRelaxationDirections();
    let relaxedNodeCount = 0;
    let totalClearanceGain = 0;
    let maxClearanceGain = 0;
    for (let pass = 0; pass < passCount; pass++) {
        const step = Math.max(0.55, spacing * 0.72 * Math.pow(0.55, pass));
        for (let index = 1; index < path.length - 1; index++) {
            const node = path[index];
            let bestPoint = node.point;
            let bestClearance = Math.max(node.clearance || 0, branchRoutePointClearance(node.point, lumenField));
            const candidatePoint = new THREE.Vector3();
            for (const direction of directions) {
                candidatePoint.copy(node.point).addScaledVector(direction, step);
                const clearance = branchRoutePointClearance(candidatePoint, lumenField);
                if (clearance <= bestClearance + BRANCH_ROUTE_RELAX_MIN_GAIN) continue;
                if (!branchRoutePointIsConnected(path, index, candidatePoint, lumenField, connectorLumenClearance)) continue;
                bestPoint = candidatePoint.clone();
                bestClearance = clearance;
            }
            const gain = bestClearance - Math.max(node.clearance || 0, branchRoutePointClearance(node.point, lumenField));
            if (gain <= BRANCH_ROUTE_RELAX_MIN_GAIN) continue;
            node.point = bestPoint;
            node.clearance = bestClearance;
            relaxedNodeCount++;
            totalClearanceGain += gain;
            maxClearanceGain = Math.max(maxClearanceGain, gain);
        }
    }

    return {
        relaxedNodeCount,
        totalClearanceGain,
        maxClearanceGain
    };
}

function scoreBranchRoutePath(path, lumenGeometry) {
    if (!Array.isArray(path) || path.length < 2 || !lumenGeometry?.attributes?.position) {
        return null;
    }
    const lumenBvh = lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry));
    const sampled = [];
    const pushSample = (point, clearance) => {
        if (sampled.length && sampled[sampled.length - 1].point.distanceTo(point) < 0.35) return;
        sampled.push({
            point: point.clone(),
            clearance: Math.max(0.5, clearance || MIN_RADIUS)
        });
    };
    for (let i = 0; i < path.length - 1; i++) {
        const start = path[i];
        const end = path[i + 1];
        const segmentLength = start.point.distanceTo(end.point);
        const stepCount = Math.max(1, Math.ceil(segmentLength / DEFAULT_CENTERLINE_NODE_SPACING));
        for (let step = 0; step < stepCount; step++) {
            const t = step / stepCount;
            pushSample(
                start.point.clone().lerp(end.point, t),
                THREE.MathUtils.lerp(start.clearance, end.clearance, t)
            );
        }
    }
    pushSample(path[path.length - 1].point, path[path.length - 1].clearance);

    let measuredCount = 0;
    let failedCount = 0;
    let offsetSum = 0;
    let maxOffset = 0;
    let normalizedSum = 0;
    let maxNormalizedOffset = 0;
    let length = 0;

    for (let i = 0; i < sampled.length; i++) {
        if (i < sampled.length - 1) {
            length += sampled[i].point.distanceTo(sampled[i + 1].point);
        }
        const directions = [];
        if (i > 0) {
            const previous = new THREE.Vector3().subVectors(sampled[i - 1].point, sampled[i].point);
            if (previous.lengthSq() > 1e-8) directions.push(previous.normalize());
        }
        if (i < sampled.length - 1) {
            const next = new THREE.Vector3().subVectors(sampled[i + 1].point, sampled[i].point);
            if (next.lengthSq() > 1e-8) directions.push(next.normalize());
        }
        if (!directions.length) continue;
        const node = {
            point: sampled[i].point,
            radius: sampled[i].clearance || MIN_RADIUS,
            directions
        };
        const measurement = nodeCenterShift(node, node.point, lumenBvh, {
            radialSamples: DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
            sphereSamples: DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES
        });
        if (!measurement.pairCount) {
            failedCount++;
            continue;
        }
        const offset = measurement.shift.length();
        const radius = Math.max(0.5, measurement.meanRadius || node.radius || MIN_RADIUS);
        const normalizedOffset = offset / radius;
        measuredCount++;
        offsetSum += offset;
        maxOffset = Math.max(maxOffset, offset);
        normalizedSum += normalizedOffset;
        maxNormalizedOffset = Math.max(maxNormalizedOffset, normalizedOffset);
    }

    return {
        measuredCount,
        failedCount,
        averageOffset: measuredCount ? offsetSum / measuredCount : Infinity,
        maxOffset: measuredCount ? maxOffset : Infinity,
        averageNormalizedOffset: measuredCount ? normalizedSum / measuredCount : Infinity,
        maxNormalizedOffset: measuredCount ? maxNormalizedOffset : Infinity,
        length,
        nodeCount: sampled.length
    };
}

function branchRouteScoreCost(score) {
    if (!score) return Infinity;
    return (
        score.maxOffset * 1.9 +
        score.averageOffset * 0.75 +
        score.maxNormalizedOffset * 8.5 +
        score.averageNormalizedOffset * 2.2 +
        score.failedCount * 12 +
        score.length * 0.008 +
        score.nodeCount * 0.012
    );
}

function summarizeBranchRouteScore(score) {
    if (!score) return null;
    return {
        averageOffset: score.averageOffset,
        maxOffset: score.maxOffset,
        averageNormalizedOffset: score.averageNormalizedOffset,
        maxNormalizedOffset: score.maxNormalizedOffset,
        failedCount: score.failedCount,
        length: score.length,
        nodeCount: score.nodeCount,
        cost: branchRouteScoreCost(score)
    };
}

function centerBranchRoutePath(route, lumenGeometry, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
    iterations = DEFAULT_CENTERLINE_REFINE_ITERATIONS,
    passes = 2
} = {}) {
    if (!route?.path?.length || route.path.length < 3 || !lumenGeometry?.attributes?.position) return null;
    const centered = cloneBranchRoute(route, `${route.routeType || 'grid'}-centered`);
    if (!centered) return null;
    const lumenBvh = lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry));
    const passCount = Math.max(1, Math.round(passes));
    let centeredNodeCount = 0;
    let totalShift = 0;
    let maxShift = 0;

    for (let pass = 0; pass < passCount; pass++) {
        const updates = centered.path.map(node => node.point.clone());
        let movedThisPass = 0;
        for (let index = 1; index < centered.path.length - 1; index++) {
            const previous = centered.path[index - 1].point;
            const current = centered.path[index].point;
            const next = centered.path[index + 1].point;
            const directions = [];
            const previousDirection = new THREE.Vector3().subVectors(previous, current);
            if (previousDirection.lengthSq() > 1e-8) directions.push(previousDirection.normalize());
            const nextDirection = new THREE.Vector3().subVectors(next, current);
            if (nextDirection.lengthSq() > 1e-8) directions.push(nextDirection.normalize());
            if (!directions.length) continue;

            const node = {
                point: current,
                radius: centered.path[index].clearance || MIN_RADIUS,
                directions
            };
            const refinement = refineCenterlineNodeToLumen(node, lumenBvh, {
                radialSamples,
                sphereSamples,
                iterations
            });
            if (!refinement.refined || !refinement.point?.clone) continue;
            const shift = refinement.point.distanceTo(current);
            if (shift < 1e-4) continue;
            updates[index] = refinement.point.clone();
            movedThisPass++;
            centeredNodeCount++;
            totalShift += shift;
            maxShift = Math.max(maxShift, shift);
        }
        if (!movedThisPass) break;
        for (let index = 1; index < centered.path.length - 1; index++) {
            centered.path[index].point.copy(updates[index]);
        }
    }

    if (!centeredNodeCount) return null;
    centered.centering = {
        centeredNodeCount,
        averageShift: totalShift / centeredNodeCount,
        maxShift
    };
    return centered;
}

function rankBranchRoutes(gridRoute, lumenGeometry, extraRoutes = []) {
    if (!gridRoute && !extraRoutes.length) return null;
    const candidateRoutes = [gridRoute, ...extraRoutes].filter(Boolean);
    const centeredRoutes = [];
    for (const route of candidateRoutes) {
        route.routeType = route.routeType || 'grid';
        route.centeringScore = scoreBranchRoutePath(route.path, lumenGeometry);
        const centered = centerBranchRoutePath(route, lumenGeometry);
        if (centered) {
            centered.centeringScore = scoreBranchRoutePath(centered.path, lumenGeometry);
            centeredRoutes.push(centered);
        }
    }

    return [...candidateRoutes, ...centeredRoutes]
        .filter(Boolean)
        .map(route => ({
            route,
            cost: branchRouteScoreCost(route.centeringScore)
        }))
        .sort((a, b) => a.cost - b.cost);
}

function chooseBranchRoute(gridRoute, lumenGeometry, extraRoutes = []) {
    return rankBranchRoutes(gridRoute, lumenGeometry, extraRoutes)?.[0]?.route || null;
}

function cloneBranchRoute(route, routeType) {
    if (!route?.path?.length) return null;
    return {
        ...route,
        routeType,
        path: route.path.map((node, index) => branchRouteNode(
            node.point,
            node.clearance,
            index,
            node.key || null
        )),
        relaxation: {
            relaxedNodeCount: 0,
            totalClearanceGain: 0,
            maxClearanceGain: 0
        }
    };
}

function makeRelaxedGridRoute(route, lumenField, connectorLumenClearance) {
    const relaxed = cloneBranchRoute(route, 'grid-relaxed');
    if (!relaxed || relaxed.path.length < 3) return null;
    const relaxation = relaxBranchRoutePath(
        relaxed.path,
        lumenField,
        connectorLumenClearance,
        route.spacing || BRANCH_ROUTE_GRID_SPACING,
        BRANCH_ROUTE_CANDIDATE_RELAX_PASSES
    );
    if (!relaxation.relaxedNodeCount) return null;
    relaxed.relaxation = relaxation;
    return relaxed;
}

function findBranchRoute(origin, lumenField, wallBvh, connectorLumenClearance) {
    if (!lumenField?.query) return null;
    if (
        !origin.forceGridRoute &&
        origin.start.distanceTo(origin.end) < BRANCH_ROUTE_MIN_LENGTH
    ) {
        return null;
    }
    const route = buildBranchRouteNodes(origin, lumenField, connectorLumenClearance);
    if (route.nodes.length < 4) return null;

    const distances = new Float64Array(route.nodes.length);
    distances.fill(Infinity);
    const previous = new Int32Array(route.nodes.length);
    previous.fill(-1);
    const heap = new MinHeap();
    const edgeWeightCache = origin.validateRouteEdges ? new Map() : null;
    distances[0] = 0;
    heap.push({ node: route.nodes[0], cost: 0 });

    while (heap.size) {
        const item = heap.pop();
        if (!item || item.cost !== distances[item.node.id]) continue;
        if (item.node.id === 1) break;
        for (const neighbour of branchRouteNeighbours(item.node, route)) {
            const edgeKey = edgeWeightCache
                ? item.node.id < neighbour.id
                    ? `${item.node.id}:${neighbour.id}`
                    : `${neighbour.id}:${item.node.id}`
                : null;
            let weight = edgeKey ? edgeWeightCache.get(edgeKey) : undefined;
            if (weight === undefined) {
                weight = branchRouteEdgeWeight(
                    item.node,
                    neighbour,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    Boolean(origin.validateRouteEdges),
                    Boolean(origin.allowWallWhenInside)
                );
                if (edgeKey) edgeWeightCache.set(edgeKey, weight);
            }
            if (!Number.isFinite(weight)) continue;
            const nextCost = item.cost + weight;
            if (nextCost >= distances[neighbour.id]) continue;
            distances[neighbour.id] = nextCost;
            previous[neighbour.id] = item.node.id;
            heap.push({ node: neighbour, cost: nextCost });
        }
    }

    if (!Number.isFinite(distances[1]) || previous[1] < 0) return null;
    const path = [];
    for (let id = 1; id >= 0; id = previous[id]) {
        path.push(route.nodes[id]);
        if (id === 0) break;
    }
    path.reverse();
    if (path.length < 2) return null;
    return {
        path,
        routeType: 'grid',
        spacing: route.spacing,
        relaxation: relaxBranchRoutePath(path, lumenField, connectorLumenClearance, route.spacing)
    };
}

function branchRouteToSegments(origin, path, routeType = 'grid') {
    const routeKey = `${origin.nodeStartId}:${origin.nodeEndId}:${connectedNodeKey(origin.start)}:${connectedNodeKey(origin.end)}`;
    const segments = [];
    const source = origin.routeSegmentSource || 'stl-slice-branch-origin';
    const isBranchOriginRoute = source === 'stl-slice-branch-origin';
    for (let i = 0; i < path.length - 1; i++) {
        const a = path[i];
        const b = path[i + 1];
        if (a.point.distanceTo(b.point) < 1e-4) continue;
        segments.push({
            ...origin,
            start: a.point.clone(),
            end: b.point.clone(),
            nodeStartId: i === 0 ? origin.nodeStartId : `route:${routeKey}:${i}`,
            nodeEndId: i === path.length - 2 ? origin.nodeEndId : `route:${routeKey}:${i + 1}`,
            radiusStart: i === 0 ? origin.radiusStart : a.clearance,
            radiusEnd: i === path.length - 2 ? origin.radiusEnd : b.clearance,
            source,
            branchOriginRouted: isBranchOriginRoute,
            centerlineLocalRouted: !isBranchOriginRoute,
            branchOriginRouteType: routeType
        });
    }
    return segments.length ? segments : [origin];
}

function routeBranchOriginSegments(segments, lumenField, wallBvh, connectorLumenClearance, lumenGeometry = null) {
    let routedSegmentCount = 0;
    let routedPathSegmentCount = 0;
    let gridRouteCount = 0;
    let gridRoutePathSegmentCount = 0;
    let failedRouteCount = 0;
    let relaxedRouteNodeCount = 0;
    let centeredRouteNodeCount = 0;
    let totalRouteClearanceGain = 0;
    let maxRouteClearanceGain = 0;
    const routeChoiceDiagnostics = [];
    const candidates = segments
        .map((segment, index) => ({
            segment,
            index,
            length: segment.start.distanceTo(segment.end)
        }))
        .filter(({ segment, length }) =>
            segment.source === 'stl-slice-branch-origin' &&
            !segment.branchOriginRouted &&
            length >= BRANCH_ROUTE_MIN_LENGTH
        )
        .sort((a, b) => b.length - a.length)
        .slice(0, BRANCH_ROUTE_MAX_SEGMENTS)
        .sort((a, b) => b.index - a.index);

    for (const { segment, index, length } of candidates) {
        const gridRoute = findBranchRoute(segment, lumenField, wallBvh, connectorLumenClearance);
        const relaxedGridRoute = makeRelaxedGridRoute(
            gridRoute,
            lumenField,
            connectorLumenClearance
        );
        const rankedRoutes = rankBranchRoutes(
            gridRoute,
            lumenGeometry,
            relaxedGridRoute ? [relaxedGridRoute] : []
        );
        let route = null;
        let routed = null;
        for (const candidate of rankedRoutes || []) {
            const candidateRoute = candidate.route;
            const candidateSegments = branchRouteToSegments(segment, candidateRoute.path, candidateRoute.routeType || 'grid');
            const valid = candidateSegments.length > 1 && candidateSegments.every(routedSegment =>
                !segmentIntersectsWall(routedSegment, wallBvh) &&
                segmentStaysInsideLumen(routedSegment, lumenField, connectorLumenClearance)
            );
            if (!valid) continue;
            route = candidateRoute;
            routed = candidateSegments;
            break;
        }
        routeChoiceDiagnostics.push({
            selectedType: route?.routeType || null,
            length,
            start: {
                x: segment.start.x,
                y: segment.start.y,
                z: segment.start.z
            },
            end: {
                x: segment.end.x,
                y: segment.end.y,
                z: segment.end.z
            },
            grid: summarizeBranchRouteScore(gridRoute?.centeringScore),
            relaxedGrid: summarizeBranchRouteScore(relaxedGridRoute?.centeringScore),
            candidates: (rankedRoutes || []).map(candidate => ({
                type: candidate.route.routeType || 'grid',
                score: summarizeBranchRouteScore(candidate.route.centeringScore),
                centeredNodeCount: candidate.route.centering?.centeredNodeCount || 0,
                centeredAverageShift: candidate.route.centering?.averageShift || 0,
                centeredMaxShift: candidate.route.centering?.maxShift || 0
            }))
        });
        if (!route) {
            failedRouteCount++;
            continue;
        }
        segments.splice(index, 1, ...routed);
        routedSegmentCount++;
        routedPathSegmentCount += routed.length;
        gridRouteCount++;
        gridRoutePathSegmentCount += routed.length;
        relaxedRouteNodeCount += route.relaxation?.relaxedNodeCount || 0;
        centeredRouteNodeCount += route.centering?.centeredNodeCount || 0;
        totalRouteClearanceGain += route.relaxation?.totalClearanceGain || 0;
        maxRouteClearanceGain = Math.max(maxRouteClearanceGain, route.relaxation?.maxClearanceGain || 0);
    }
    return {
        routedSegmentCount,
        routedPathSegmentCount,
        gridRouteCount,
        gridRoutePathSegmentCount,
        failedRouteCount,
        relaxedRouteNodeCount,
        centeredRouteNodeCount,
        totalRouteClearanceGain,
        maxRouteClearanceGain,
        routeChoiceDiagnostics
    };
}

function segmentEndpointKeys(segment, endpointKeys) {
    const cached = endpointKeys.get(segment);
    if (cached?.start && cached?.end) return cached;
    return {
        start: centerlineEndpointKey(segment, 'start'),
        end: centerlineEndpointKey(segment, 'end')
    };
}

function otherSegmentEndpointKey(segment, fromKey, endpointKeys) {
    const keys = segmentEndpointKeys(segment, endpointKeys);
    if (keys.start === fromKey) return keys.end;
    if (keys.end === fromKey) return keys.start;
    return null;
}

function walkOutlierChainSide(centerKey, firstSegment, incidence, endpointKeys, maxSteps) {
    const segments = [];
    const keys = [centerKey];
    let currentKey = centerKey;
    let currentSegment = firstSegment;

    for (let step = 0; step < maxSteps && currentSegment; step++) {
        const nextKey = otherSegmentEndpointKey(currentSegment, currentKey, endpointKeys);
        if (!nextKey || keys.includes(nextKey)) break;
        segments.push(currentSegment);
        keys.push(nextKey);

        const nextIncidence = incidence.get(nextKey) || [];
        if (nextIncidence.length !== 2) break;
        const next = nextIncidence.find(item => item.segment !== currentSegment);
        if (!next) break;
        currentKey = nextKey;
        currentSegment = next.segment;
    }

    return {
        segments,
        keys,
        farKey: keys[keys.length - 1]
    };
}

function nodeInfoForEndpointKey(key, incidence, lumenField = null) {
    const incident = incidence.get(key);
    if (!incident?.length) return null;
    const point = currentNodePointFromIncidence(key, incidence);
    if (!point) return null;
    let radius = 0;
    let nodeId = key;
    for (const { segment, endpoint } of incident) {
        radius = Math.max(radius, endpoint === 'start' ? segment.radiusStart || 0 : segment.radiusEnd || 0);
        nodeId = endpoint === 'start' ? segment.nodeStartId : segment.nodeEndId;
        if (nodeId !== undefined && nodeId !== null) break;
    }
    const clearance = branchRoutePointClearance(point, lumenField);
    return {
        key,
        point,
        nodeId: nodeId ?? key,
        radius: finiteRadius(radius, Number.isFinite(clearance) ? clearance : MIN_RADIUS),
        clearance: Number.isFinite(clearance) ? Math.max(0.25, clearance) : finiteRadius(radius)
    };
}

function buildOutlierRerouteChain(key, incidence, endpointKeys, lumenField, {
    sideSteps = CENTERLINE_OUTLIER_REROUTE_SIDE_STEPS,
    minSpan = CENTERLINE_OUTLIER_REROUTE_MIN_SPAN,
    maxSpan = CENTERLINE_OUTLIER_REROUTE_MAX_SPAN
} = {}) {
    const incident = incidence.get(key);
    if (!incident || incident.length !== 2) return null;
    const left = walkOutlierChainSide(key, incident[0].segment, incidence, endpointKeys, sideSteps);
    const right = walkOutlierChainSide(key, incident[1].segment, incidence, endpointKeys, sideSteps);
    const orderedKeys = [
        ...left.keys.slice().reverse(),
        ...right.keys.slice(1)
    ];
    const uniqueKeys = new Set(orderedKeys);
    if (uniqueKeys.size !== orderedKeys.length || orderedKeys.length < 3) return null;

    const segmentSet = new Set([...left.segments, ...right.segments]);
    if (segmentSet.size < 2) return null;
    const nodes = [];
    for (const nodeKey of orderedKeys) {
        const info = nodeInfoForEndpointKey(nodeKey, incidence, lumenField);
        if (!info) return null;
        nodes.push(info);
    }

    const start = nodes[0];
    const end = nodes[nodes.length - 1];
    const span = start.point.distanceTo(end.point);
    let pathLength = 0;
    for (let i = 0; i < nodes.length - 1; i++) {
        pathLength += nodes[i].point.distanceTo(nodes[i + 1].point);
    }
    if (span < minSpan || span > maxSpan) {
        return null;
    }

    return {
        key,
        start,
        end,
        center: nodeInfoForEndpointKey(key, incidence, lumenField),
        nodes,
        segments: [...segmentSet],
        span,
        pathLength
    };
}

function routeFromOutlierChain(chain) {
    return {
        path: chain.nodes.map((node, index) => branchRouteNode(
            node.point,
            node.clearance || node.radius,
            index,
            `original:${index}`
        )),
        routeType: 'original',
        spacing: DEFAULT_CENTERLINE_NODE_SPACING
    };
}

function segmentValidityFailureReason(segment, wallBvh, lumenField, connectorLumenClearance, {
    allowWallWhenInside = false,
    wallFallbackClearance = 0.18
} = {}) {
    if (segmentIntersectsWall(segment, wallBvh)) {
        const fallbackClearance = Math.max(0.08, connectorLumenClearance + wallFallbackClearance);
        if (
            allowWallWhenInside &&
            segmentStaysInsideLumen(segment, lumenField, fallbackClearance)
        ) {
            return null;
        }
        return 'wall';
    }
    if (!segmentStaysInsideLumen(segment, lumenField, connectorLumenClearance)) return 'lumen';
    return null;
}

function segmentSetValidityFailureReason(
    segments,
    wallBvh,
    lumenField,
    connectorLumenClearance,
    options = {}
) {
    for (const segment of segments) {
        const reason = segmentValidityFailureReason(
            segment,
            wallBvh,
            lumenField,
            connectorLumenClearance,
            options
        );
        if (reason) return reason;
    }
    return null;
}

function bestValidatedRouteForOrigin(origin, lumenField, wallBvh, connectorLumenClearance, lumenGeometry) {
    const directLength = origin.start.distanceTo(origin.end);
    if (directLength < BRANCH_ROUTE_MIN_LENGTH || origin.allowLongDirectRoute) {
        const directSegment = {
            ...origin,
            source: origin.routeSegmentSource || origin.source || 'stl-slice-local-reroute',
            centerlineLocalRouted: true,
            branchOriginRouteType: 'direct'
        };
        if (
            directSegment.start.distanceTo(directSegment.end) > 1e-4 &&
            !segmentValidityFailureReason(
                directSegment,
                wallBvh,
                lumenField,
                connectorLumenClearance,
                { allowWallWhenInside: Boolean(origin.allowWallWhenInside) }
            )
        ) {
            const path = [
                branchRouteNode(origin.start, origin.radiusStart, 0, 'direct-start'),
                branchRouteNode(origin.end, origin.radiusEnd, 1, 'direct-end')
            ];
            return {
                segments: [directSegment],
                path,
                routeType: 'direct',
                routeScore: scoreBranchRoutePath(path, lumenGeometry),
                centeredNodeCount: 0
            };
        }
    }

    const gridRoute = findBranchRoute(origin, lumenField, wallBvh, connectorLumenClearance);
    const relaxedGridRoute = makeRelaxedGridRoute(gridRoute, lumenField, connectorLumenClearance);
    const rankedRoutes = rankBranchRoutes(
        gridRoute,
        lumenGeometry,
        relaxedGridRoute ? [relaxedGridRoute] : []
    );
    for (const candidate of rankedRoutes || []) {
        const route = candidate.route;
        const routedSegments = branchRouteToSegments(origin, route.path, route.routeType || 'grid');
        const valid = routedSegments.length > 0 && !segmentSetValidityFailureReason(
            routedSegments,
            wallBvh,
            lumenField,
            connectorLumenClearance,
            {
                allowWallWhenInside: Boolean(origin.allowWallWhenInside)
            }
        );
        if (!valid) continue;
        return {
            segments: routedSegments,
            path: route.path,
            routeType: route.routeType || 'grid',
            routeScore: route.centeringScore || scoreBranchRoutePath(route.path, lumenGeometry),
            centeredNodeCount: route.centering?.centeredNodeCount || 0
        };
    }
    return null;
}

function routeOutlierChain(chain, lumenField, wallBvh, connectorLumenClearance, lumenGeometry, failureDiagnostics = null) {
    const noteFailure = (stage, details = {}) => {
        if (!failureDiagnostics) return;
        if (!failureDiagnostics.stages) failureDiagnostics.stages = [];
        if (failureDiagnostics.stages.length >= 16) return;
        failureDiagnostics.stages.push({
            stage,
            ...details
        });
    };
    const origin = {
        start: chain.start.point.clone(),
        end: chain.end.point.clone(),
        nodeStartId: chain.start.nodeId,
        nodeEndId: chain.end.nodeId,
        radiusStart: chain.start.radius,
        radiusEnd: chain.end.radius,
        source: 'stl-slice-local-reroute',
        routeSegmentSource: 'stl-slice-local-reroute',
        routeBoundsPoints: CENTERLINE_OUTLIER_USE_CHAIN_ROUTE_BOUNDS
            ? chain.nodes.map(node => node.point)
            : null,
        routePadding: CENTERLINE_OUTLIER_USE_CHAIN_ROUTE_BOUNDS
            ? Math.max(BRANCH_ROUTE_PADDING, Math.min(26, chain.span * 0.52))
            : null,
        routeGridSpacing: Math.max(1.45, Math.min(BRANCH_ROUTE_GRID_SPACING, chain.span * 0.13)),
        routeMaxCells: 9000,
        allowWallWhenInside: true
    };
    const originalRoute = routeFromOutlierChain(chain);
    const originalScore = scoreBranchRoutePath(originalRoute.path, lumenGeometry);
    const originalCost = branchRouteScoreCost(originalScore);
    const passesScoreGate = routeScore => {
        const routeCost = branchRouteScoreCost(routeScore);
        if (!Number.isFinite(routeCost)) return false;
        const improvesWorst = !Number.isFinite(originalCost) || (
            routeScore.maxNormalizedOffset < originalScore.maxNormalizedOffset - 0.025 ||
            routeScore.maxOffset < originalScore.maxOffset - 0.08 ||
            routeCost < originalCost * 0.92
        );
        const preservesAverage = !originalScore || !Number.isFinite(originalScore.averageNormalizedOffset) || (
            routeScore.averageNormalizedOffset <= Math.max(0.04, originalScore.averageNormalizedOffset + 0.015) &&
            routeScore.averageOffset <= Math.max(0.22, originalScore.averageOffset + 0.08)
        );
        return improvesWorst && preservesAverage;
    };
    const hasMeaningfulRouteGain = routeScore => {
        const routeCost = branchRouteScoreCost(routeScore);
        if (!originalScore || !Number.isFinite(originalCost)) return Number.isFinite(routeCost);
        return (
            routeScore.maxNormalizedOffset <= originalScore.maxNormalizedOffset - 0.08 ||
            routeScore.maxOffset <= originalScore.maxOffset - 0.35 ||
            routeCost <= originalCost * 0.86
        );
    };

    const measuredShift = new THREE.Vector3(
        chain.candidate?.shift?.x || 0,
        chain.candidate?.shift?.y || 0,
        chain.candidate?.shift?.z || 0
    );
    if (chain.center && measuredShift.lengthSq() > 1e-8) {
        for (const scale of [1, 0.78, 0.58, 0.4]) {
            const centerPoint = chain.center.point.clone().addScaledVector(measuredShift, scale);
            const centerClearance = branchRoutePointClearance(centerPoint, lumenField);
            if (!Number.isFinite(centerClearance) || centerClearance < connectorLumenClearance + 0.12) {
                noteFailure('via-center-clearance', { scale, centerClearance });
                continue;
            }
            const centerNodeId = `local-reroute-center:${chain.key}:${scale}`;
            const centerRadius = Math.max(0.5, Math.min(
                Number.isFinite(centerClearance) && centerClearance > 0 ? centerClearance : Infinity,
                chain.candidate?.meanRadius || chain.center.radius || centerClearance
            ));
            if (CENTERLINE_OUTLIER_ENABLE_VIA_CENTER_REROUTE) {
                const firstOrigin = {
                    ...origin,
                    end: centerPoint.clone(),
                    nodeEndId: centerNodeId,
                    radiusEnd: centerRadius
                };
                const secondOrigin = {
                    ...origin,
                    start: centerPoint.clone(),
                    nodeStartId: centerNodeId,
                    radiusStart: centerRadius
                };
                const firstLeg = bestValidatedRouteForOrigin(
                    firstOrigin,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    lumenGeometry
                );
                const secondLeg = bestValidatedRouteForOrigin(
                    secondOrigin,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    lumenGeometry
                );
                if (firstLeg && secondLeg) {
                    const combinedPath = [
                        ...firstLeg.path,
                        ...secondLeg.path.slice(1)
                    ];
                    const combinedScore = scoreBranchRoutePath(combinedPath, lumenGeometry);
                    if (passesScoreGate(combinedScore)) {
                        const combinedSegments = [...firstLeg.segments, ...secondLeg.segments];
                        const failureReason = segmentSetValidityFailureReason(
                            combinedSegments,
                            wallBvh,
                            lumenField,
                            connectorLumenClearance,
                            {
                                allowWallWhenInside: Boolean(origin.allowWallWhenInside)
                            }
                        );
                        const validCombined = hasMeaningfulRouteGain(combinedScore) &&
                            combinedSegments.length > 1 &&
                            !failureReason;
                        if (validCombined) {
                            return {
                                segments: combinedSegments,
                                routeType: `via-center:${firstLeg.routeType}+${secondLeg.routeType}`,
                                originalScore: summarizeBranchRouteScore(originalScore),
                                routeScore: summarizeBranchRouteScore(combinedScore),
                                replacedSegmentCount: chain.segments.length,
                                insertedSegmentCount: combinedSegments.length,
                                span: chain.span,
                                pathLength: chain.pathLength,
                                centeredNodeCount: 1 + firstLeg.centeredNodeCount + secondLeg.centeredNodeCount
                            };
                        }
                        noteFailure('via-center-validity', {
                            scale,
                            failureReason: failureReason || 'gain',
                            routeScore: summarizeBranchRouteScore(combinedScore)
                        });
                    } else {
                        noteFailure('via-center-score', {
                            scale,
                            routeScore: summarizeBranchRouteScore(combinedScore)
                        });
                    }
                } else {
                    noteFailure('via-center-leg', {
                        scale,
                        firstLeg: Boolean(firstLeg),
                        secondLeg: Boolean(secondLeg)
                    });
                }
            }

            const forcedSegments = [
                {
                    ...origin,
                    end: centerPoint.clone(),
                    nodeEndId: centerNodeId,
                    radiusEnd: centerRadius,
                    source: 'stl-slice-local-reroute',
                    centerlineLocalRouted: true,
                    branchOriginRouteType: 'forced-center'
                },
                {
                    ...origin,
                    start: centerPoint.clone(),
                    nodeStartId: centerNodeId,
                    radiusStart: centerRadius,
                    source: 'stl-slice-local-reroute',
                    centerlineLocalRouted: true,
                    branchOriginRouteType: 'forced-center'
                }
            ].filter(segment => segment.start.distanceTo(segment.end) > 1e-4);
            if (forcedSegments.length < 2) continue;
            const forcedFailureReason = segmentSetValidityFailureReason(
                forcedSegments,
                wallBvh,
                lumenField,
                connectorLumenClearance,
                {
                    allowWallWhenInside: Boolean(origin.allowWallWhenInside)
                }
            );
            if (forcedFailureReason) {
                noteFailure('forced-center-validity', { scale, failureReason: forcedFailureReason });
                continue;
            }
            const forcedPath = [
                branchRouteNode(origin.start, origin.radiusStart, 0, 'forced-start'),
                branchRouteNode(centerPoint, centerRadius, 1, 'forced-center'),
                branchRouteNode(origin.end, origin.radiusEnd, 2, 'forced-end')
            ];
            const forcedScore = scoreBranchRoutePath(forcedPath, lumenGeometry);
            if (!passesScoreGate(forcedScore)) {
                noteFailure('forced-center-score', { scale, routeScore: summarizeBranchRouteScore(forcedScore) });
                continue;
            }
            if (!hasMeaningfulRouteGain(forcedScore)) {
                noteFailure('forced-center-gain', { scale, routeScore: summarizeBranchRouteScore(forcedScore) });
                continue;
            }
            return {
                segments: forcedSegments,
                routeType: 'forced-center',
                originalScore: summarizeBranchRouteScore(originalScore),
                routeScore: summarizeBranchRouteScore(forcedScore),
                replacedSegmentCount: chain.segments.length,
                insertedSegmentCount: forcedSegments.length,
                span: chain.span,
                pathLength: chain.pathLength,
                centeredNodeCount: 1
            };
        }
    }

    const gridRoute = findBranchRoute(origin, lumenField, wallBvh, connectorLumenClearance);
    const relaxedGridRoute = makeRelaxedGridRoute(gridRoute, lumenField, connectorLumenClearance);
    const rankedRoutes = rankBranchRoutes(
        gridRoute,
        lumenGeometry,
        relaxedGridRoute ? [relaxedGridRoute] : []
    );
    for (const candidate of rankedRoutes || []) {
        const route = candidate.route;
        const routeScore = route.centeringScore || scoreBranchRoutePath(route.path, lumenGeometry);
        if (!passesScoreGate(routeScore)) {
            noteFailure('grid-score', {
                routeType: route.routeType || 'grid',
                routeScore: summarizeBranchRouteScore(routeScore)
            });
            continue;
        }

        const routedSegments = branchRouteToSegments(origin, route.path, route.routeType || 'grid');
        const failureReason = routedSegments.length > 1
            ? segmentSetValidityFailureReason(
                routedSegments,
                wallBvh,
                lumenField,
                connectorLumenClearance,
                {
                    allowWallWhenInside: Boolean(origin.allowWallWhenInside)
                }
            )
            : 'too-short';
        const valid = !failureReason;
        if (!valid) {
            noteFailure('grid-validity', {
                routeType: route.routeType || 'grid',
                failureReason,
                routeScore: summarizeBranchRouteScore(routeScore)
            });
            continue;
        }
        return {
            segments: routedSegments,
            routeType: route.routeType || 'grid',
            originalScore: summarizeBranchRouteScore(originalScore),
            routeScore: summarizeBranchRouteScore(routeScore),
            replacedSegmentCount: chain.segments.length,
            insertedSegmentCount: routedSegments.length,
            span: chain.span,
            pathLength: chain.pathLength,
            centeredNodeCount: route.centering?.centeredNodeCount || 0
        };
    }

    if (!gridRoute) noteFailure('grid-missing');
    else if (!rankedRoutes?.length) noteFailure('grid-unranked');
    return null;
}

function replaceCenterlineSegmentSet(segments, removeSet, replacements) {
    const indices = [];
    for (let i = 0; i < segments.length; i++) {
        if (removeSet.has(segments[i])) indices.push(i);
    }
    if (!indices.length) return false;
    const insertIndex = indices[0];
    for (let i = indices.length - 1; i >= 0; i--) {
        segments.splice(indices[i], 1);
    }
    segments.splice(insertIndex, 0, ...replacements);
    return true;
}

function replaceCenterlineSegmentSetPreservingComponents(segments, removeSet, replacements) {
    const previousSegments = segments.slice();
    const previousComponentCount = segmentComponents(segments).length;
    if (!replaceCenterlineSegmentSet(segments, removeSet, replacements)) return false;
    if (segmentComponents(segments).length <= previousComponentCount) return true;
    segments.length = 0;
    segments.push(...previousSegments);
    return false;
}

function oppositeEndpointInfo(incidentEntry, endpointKeys) {
    const { segment, endpoint } = incidentEntry;
    const keys = segmentEndpointKeys(segment, endpointKeys);
    if (endpoint === 'start') {
        return {
            key: keys.end,
            point: segment.end,
            nodeId: segment.nodeEndId,
            radius: segment.radiusEnd
        };
    }
    return {
        key: keys.start,
        point: segment.start,
        nodeId: segment.nodeStartId,
        radius: segment.radiusStart
    };
}

function centerlineNodeDeflectionDegrees(key, incident, incidence, endpointKeys) {
    if (!incident || incident.length !== 2) return 0;
    const center = currentNodePointFromIncidence(key, incidence);
    if (!center) return 0;
    const first = oppositeEndpointInfo(incident[0], endpointKeys);
    const second = oppositeEndpointInfo(incident[1], endpointKeys);
    const firstDirection = new THREE.Vector3().subVectors(first.point, center);
    const secondDirection = new THREE.Vector3().subVectors(second.point, center);
    if (firstDirection.lengthSq() < 1e-8 || secondDirection.lengthSq() < 1e-8) return 0;
    firstDirection.normalize();
    secondDirection.normalize();
    const angle = THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(
        firstDirection.dot(secondDirection),
        -1,
        1
    )));
    return 180 - angle;
}

function measureCenterlineTopology(segments) {
    const incidence = collectNodeIncidence(segments);
    const endpointKeys = collectSegmentEndpointKeyMap(incidence);
    let leafNodeCount = 0;
    let branchNodeCount = 0;
    let degreeTwoNodeCount = 0;
    let sharpTurnNodeCount = 0;
    let severeBacktrackNodeCount = 0;
    let maxDeflectionDegrees = 0;
    for (const [key, incident] of incidence.entries()) {
        if (incident.length === 1) leafNodeCount++;
        else if (incident.length > 2) branchNodeCount++;
        if (incident.length !== 2) continue;
        degreeTwoNodeCount++;
        const deflection = centerlineNodeDeflectionDegrees(key, incident, incidence, endpointKeys);
        maxDeflectionDegrees = Math.max(maxDeflectionDegrees, deflection);
        if (deflection > 45) sharpTurnNodeCount++;
        if (deflection >= CENTERLINE_BACKTRACK_MIN_DEFLECTION_DEG) severeBacktrackNodeCount++;
    }
    return {
        nodeCount: incidence.size,
        leafNodeCount,
        branchNodeCount,
        degreeTwoNodeCount,
        sharpTurnNodeCount,
        severeBacktrackNodeCount,
        maxDeflectionDegrees
    };
}

function measureCenterlineSampleCoverage(samples, segments, cellSize = 10) {
    if (!samples.length || !segments.length) {
        return {
            sampleCount: samples.length,
            coveredSampleCount: 0,
            uncoveredSampleCount: samples.length,
            averageDistance: Infinity,
            maxDistance: Infinity,
            averageNormalizedDistance: Infinity,
            maxNormalizedDistance: Infinity,
            worstSamples: []
        };
    }

    const grid = new Map();
    const keyFor = (ix, iy, iz) => `${ix},${iy},${iz}`;
    const cellFor = value => Math.floor(value / cellSize);
    for (const segment of segments) {
        const minX = cellFor(Math.min(segment.start.x, segment.end.x));
        const minY = cellFor(Math.min(segment.start.y, segment.end.y));
        const minZ = cellFor(Math.min(segment.start.z, segment.end.z));
        const maxX = cellFor(Math.max(segment.start.x, segment.end.x));
        const maxY = cellFor(Math.max(segment.start.y, segment.end.y));
        const maxZ = cellFor(Math.max(segment.start.z, segment.end.z));
        for (let ix = minX; ix <= maxX; ix++) {
            for (let iy = minY; iy <= maxY; iy++) {
                for (let iz = minZ; iz <= maxZ; iz++) {
                    const key = keyFor(ix, iy, iz);
                    let bucket = grid.get(key);
                    if (!bucket) {
                        bucket = [];
                        grid.set(key, bucket);
                    }
                    bucket.push(segment);
                }
            }
        }
    }

    let coveredSampleCount = 0;
    let distanceSum = 0;
    let normalizedDistanceSum = 0;
    let maxDistance = 0;
    let maxNormalizedDistance = 0;
    const worstSamples = [];
    for (const sample of samples) {
        const radius = Math.max(0.5, finiteRadius(sample.radius));
        const coverageDistance = Math.max(1.6, radius * 0.72);
        const searchCellRadius = Math.max(1, Math.ceil(coverageDistance / cellSize) + 1);
        const cx = cellFor(sample.point.x);
        const cy = cellFor(sample.point.y);
        const cz = cellFor(sample.point.z);
        const candidates = new Set();
        for (let dx = -searchCellRadius; dx <= searchCellRadius; dx++) {
            for (let dy = -searchCellRadius; dy <= searchCellRadius; dy++) {
                for (let dz = -searchCellRadius; dz <= searchCellRadius; dz++) {
                    for (const segment of grid.get(keyFor(cx + dx, cy + dy, cz + dz)) || []) {
                        candidates.add(segment);
                    }
                }
            }
        }
        let distance = Infinity;
        const distanceCandidates = candidates.size ? candidates : segments;
        for (const segment of distanceCandidates) {
            distance = Math.min(distance, pointCenterlineSegmentDistance(sample.point, segment));
        }
        const normalizedDistance = distance / radius;
        if (distance <= coverageDistance) coveredSampleCount++;
        distanceSum += distance;
        normalizedDistanceSum += normalizedDistance;
        maxDistance = Math.max(maxDistance, distance);
        maxNormalizedDistance = Math.max(maxNormalizedDistance, normalizedDistance);
        const entry = {
            axis: sample.axis,
            radius,
            distance,
            normalizedDistance,
            point: {
                x: sample.point.x,
                y: sample.point.y,
                z: sample.point.z
            }
        };
        worstSamples.push(entry);
        worstSamples.sort((a, b) => b.normalizedDistance - a.normalizedDistance);
        if (worstSamples.length > 16) worstSamples.pop();
    }

    return {
        sampleCount: samples.length,
        coveredSampleCount,
        uncoveredSampleCount: samples.length - coveredSampleCount,
        averageDistance: distanceSum / samples.length,
        maxDistance,
        averageNormalizedDistance: normalizedDistanceSum / samples.length,
        maxNormalizedDistance,
        worstSamples
    };
}

function simplifyCenterlineBacktracks(
    segments,
    lumenField,
    wallBvh,
    connectorLumenClearance,
    {
        minDeflectionDegrees = CENTERLINE_BACKTRACK_MIN_DEFLECTION_DEG,
        maxPasses = CENTERLINE_BACKTRACK_MAX_PASSES
    } = {}
) {
    let collapsedNodeCount = 0;
    let removedSegmentCount = 0;
    let insertedSegmentCount = 0;
    let rejectedNodeCount = 0;
    let pathLengthReduction = 0;
    let passCount = 0;

    for (let pass = 0; pass < maxPasses; pass++) {
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const candidates = [];
        for (const [key, incident] of incidence.entries()) {
            if (incident.length !== 2 || incident[0].segment === incident[1].segment) continue;
            const deflection = centerlineNodeDeflectionDegrees(key, incident, incidence, endpointKeys);
            if (deflection < minDeflectionDegrees) continue;
            candidates.push({ key, incident, deflection });
        }
        candidates.sort((a, b) => b.deflection - a.deflection);
        if (!candidates.length) break;

        const usedSegments = new Set();
        let collapsedThisPass = 0;
        for (const candidate of candidates) {
            const [firstEntry, secondEntry] = candidate.incident;
            if (usedSegments.has(firstEntry.segment) || usedSegments.has(secondEntry.segment)) continue;
            if (!segments.includes(firstEntry.segment) || !segments.includes(secondEntry.segment)) continue;
            const first = oppositeEndpointInfo(firstEntry, endpointKeys);
            const second = oppositeEndpointInfo(secondEntry, endpointKeys);
            if (!first.key || !second.key || first.key === second.key) {
                rejectedNodeCount++;
                continue;
            }
            const firstLength = firstEntry.segment.start.distanceTo(firstEntry.segment.end);
            const secondLength = secondEntry.segment.start.distanceTo(secondEntry.segment.end);
            const replacementLength = first.point.distanceTo(second.point);
            const originalLength = firstLength + secondLength;
            if (
                replacementLength < 1e-4 ||
                replacementLength >= originalLength * 0.94
            ) {
                rejectedNodeCount++;
                continue;
            }

            const replacement = {
                ...firstEntry.segment,
                start: first.point.clone(),
                end: second.point.clone(),
                nodeStartId: first.nodeId,
                nodeEndId: second.nodeId,
                radiusStart: first.radius,
                radiusEnd: second.radius,
                source: firstEntry.segment.source === secondEntry.segment.source
                    ? firstEntry.segment.source
                    : 'stl-slice-centerline-simplified',
                centerlineBacktrackCollapsed: true
            };
            const failureReason = segmentValidityFailureReason(
                replacement,
                wallBvh,
                lumenField,
                connectorLumenClearance
            );
            if (failureReason) {
                rejectedNodeCount++;
                continue;
            }
            // Replacing the two edges incident to a degree-two node with the
            // validated edge between their opposite endpoints preserves the
            // component by construction. A full component traversal here made
            // the simplification quadratic on dense medial trees.
            if (!replaceCenterlineSegmentSet(
                segments,
                new Set([firstEntry.segment, secondEntry.segment]),
                [replacement]
            )) {
                rejectedNodeCount++;
                continue;
            }
            usedSegments.add(firstEntry.segment);
            usedSegments.add(secondEntry.segment);
            collapsedNodeCount++;
            removedSegmentCount += 2;
            insertedSegmentCount++;
            pathLengthReduction += originalLength - replacementLength;
            collapsedThisPass++;
        }
        if (!collapsedThisPass) break;
        passCount++;
    }

    return {
        passCount,
        collapsedNodeCount,
        removedSegmentCount,
        insertedSegmentCount,
        rejectedNodeCount,
        pathLengthReduction
    };
}

function invalidCenterlineSegments(
    segments,
    lumenField,
    wallBvh,
    connectorLumenClearance
) {
    return segments.filter(segment => segmentValidityFailureReason(
        segment,
        wallBvh,
        lumenField,
        connectorLumenClearance,
        { allowWallWhenInside: Boolean(segment.allowWallWhenInside) }
    ));
}

function connectedInvalidSegmentComponents(invalidSegments, incidence, endpointKeys) {
    const pending = new Set(invalidSegments);
    const components = [];
    while (pending.size) {
        const seed = pending.values().next().value;
        pending.delete(seed);
        const component = [];
        const stack = [seed];
        while (stack.length) {
            const segment = stack.pop();
            component.push(segment);
            const keys = segmentEndpointKeys(segment, endpointKeys);
            for (const key of [keys.start, keys.end]) {
                for (const { segment: neighbour } of incidence.get(key) || []) {
                    if (!pending.has(neighbour)) continue;
                    pending.delete(neighbour);
                    stack.push(neighbour);
                }
            }
        }
        components.push(component);
    }
    return components.sort((a, b) => b.length - a.length);
}

function invalidComponentEndpointKeys(component, incidence, endpointKeys) {
    const componentSet = new Set(component);
    const endpointKeysFound = new Set();
    for (const segment of component) {
        const keys = segmentEndpointKeys(segment, endpointKeys);
        for (const key of [keys.start, keys.end]) {
            const invalidDegree = (incidence.get(key) || [])
                .filter(entry => componentSet.has(entry.segment))
                .length;
            if (invalidDegree === 1) endpointKeysFound.add(key);
        }
    }
    return [...endpointKeysFound];
}

function expandInvalidComponentEndpoint(
    startKey,
    removalSet,
    incidence,
    endpointKeys,
    lumenField,
    maxSteps = 3
) {
    let key = startKey;
    for (let step = 0; step < maxSteps; step++) {
        const outward = (incidence.get(key) || [])
            .filter(entry => !removalSet.has(entry.segment));
        if (outward.length !== 1) break;
        const nextKey = otherSegmentEndpointKey(outward[0].segment, key, endpointKeys);
        if (!nextKey || nextKey === key) break;
        removalSet.add(outward[0].segment);
        key = nextKey;
    }
    return nodeInfoForEndpointKey(key, incidence, lumenField);
}

function findInvalidComponentMedialHub(removalSet, lumenField) {
    const bounds = new THREE.Box3();
    const centroid = new THREE.Vector3();
    let pointCount = 0;
    for (const segment of removalSet) {
        bounds.expandByPoint(segment.start);
        bounds.expandByPoint(segment.end);
        centroid.add(segment.start).add(segment.end);
        pointCount += 2;
    }
    if (!pointCount || bounds.isEmpty()) return null;
    centroid.multiplyScalar(1 / pointCount);
    bounds.expandByScalar(4);
    const size = bounds.getSize(new THREE.Vector3());
    const spacing = Math.max(1.1, Math.min(1.8, Math.max(size.x, size.y, size.z) / 12));
    const nx = Math.max(2, Math.ceil(size.x / spacing));
    const ny = Math.max(2, Math.ceil(size.y / spacing));
    const nz = Math.max(2, Math.ceil(size.z / spacing));
    const point = new THREE.Vector3();
    let best = null;
    for (let ix = 0; ix <= nx; ix++) {
        point.x = THREE.MathUtils.lerp(bounds.min.x, bounds.max.x, ix / nx);
        for (let iy = 0; iy <= ny; iy++) {
            point.y = THREE.MathUtils.lerp(bounds.min.y, bounds.max.y, iy / ny);
            for (let iz = 0; iz <= nz; iz++) {
                point.z = THREE.MathUtils.lerp(bounds.min.z, bounds.max.z, iz / nz);
                const clearance = branchRoutePointClearance(point, lumenField);
                if (!Number.isFinite(clearance) || clearance <= 0.2) continue;
                const score = clearance - point.distanceTo(centroid) * 0.018;
                if (!best || score > best.score) {
                    best = { point: point.clone(), clearance, score };
                }
            }
        }
    }
    return best;
}

function routeInvalidBranchComponent(
    component,
    componentEndpoints,
    incidence,
    endpointKeys,
    lumenField,
    wallBvh,
    connectorLumenClearance,
    lumenGeometry,
    expansionSteps
) {
    const removalSet = new Set(component);
    const endpoints = [];
    for (const endpointKey of componentEndpoints) {
        const endpoint = expandInvalidComponentEndpoint(
            endpointKey,
            removalSet,
            incidence,
            endpointKeys,
            lumenField,
            expansionSteps
        );
        if (!endpoint) return null;
        const clearance = branchRoutePointClearance(endpoint.point, lumenField);
        if (!Number.isFinite(clearance) || clearance <= connectorLumenClearance + 0.12) {
            return null;
        }
        if (endpoints.some(existing => existing.key === endpoint.key)) return null;
        endpoints.push(endpoint);
    }
    const hub = findInvalidComponentMedialHub(removalSet, lumenField);
    if (!hub) return null;

    const boundsPoints = [];
    let pathLength = 0;
    for (const segment of removalSet) {
        boundsPoints.push(segment.start, segment.end);
        pathLength += segment.start.distanceTo(segment.end);
    }
    const hubNodeId = `invalid-hub:${connectedNodeKey(hub.point)}`;
    const replacements = [];
    const routeTypes = [];
    for (const endpoint of endpoints) {
        const span = endpoint.point.distanceTo(hub.point);
        if (span < 1e-4) continue;
        const vesselScale = Math.max(
            0.45,
            Math.min(1.35, Math.min(endpoint.radius, hub.clearance) * 0.45)
        );
        const origin = {
            start: endpoint.point.clone(),
            end: hub.point.clone(),
            nodeStartId: endpoint.nodeId,
            nodeEndId: hubNodeId,
            radiusStart: endpoint.radius,
            radiusEnd: hub.clearance,
            source: 'stl-slice-invalid-reroute',
            routeSegmentSource: 'stl-slice-invalid-reroute',
            routeBoundsPoints: boundsPoints,
            routePadding: Math.max(1.2, vesselScale * 1.8, Math.min(8, span * 0.24)),
            routeMinimumPadding: Math.max(0.8, vesselScale * 1.4),
            routeRadiusPaddingScale: 0.3,
            routeRadiusPaddingBase: 0.8,
            routeGridSpacing: vesselScale * (expansionSteps >= 2 ? 0.75 : 1),
            routeMaxCells: expansionSteps >= 2 ? 36000 : 18000,
            forceGridRoute: true,
            validateRouteEdges: true,
            allowWallWhenInside: false
        };
        const route = bestValidatedRouteForOrigin(
            origin,
            lumenField,
            wallBvh,
            connectorLumenClearance,
            lumenGeometry
        );
        if (!route?.segments?.length) return null;
        if (segmentSetValidityFailureReason(
            route.segments,
            wallBvh,
            lumenField,
            connectorLumenClearance
        )) {
            return null;
        }
        replacements.push(...route.segments);
        routeTypes.push(route.routeType || 'unknown');
    }
    if (replacements.length < componentEndpoints.length) return null;
    return {
        removalSet,
        route: {
            segments: replacements,
            routeType: `branch-star:${routeTypes.join('+')}`
        },
        expansionSteps,
        span: Math.max(...endpoints.map(endpoint => endpoint.point.distanceTo(hub.point))),
        pathLength
    };
}

function rerouteInvalidCenterlineChains(
    segments,
    lumenField,
    wallBvh,
    connectorLumenClearance,
    lumenGeometry,
    { maxChains = CENTERLINE_INVALID_REROUTE_MAX_CHAINS } = {}
) {
    const initialInvalidSegmentCount = invalidCenterlineSegments(
        segments,
        lumenField,
        wallBvh,
        connectorLumenClearance
    ).length;
    const blockedSegments = new Set();
    let attemptedChainCount = 0;
    let routedChainCount = 0;
    let replacedSegmentCount = 0;
    let insertedSegmentCount = 0;
    const choices = [];
    const blockedComponents = [];

    while (routedChainCount < maxChains) {
        const currentInvalidSegments = invalidCenterlineSegments(
            segments,
            lumenField,
            wallBvh,
            connectorLumenClearance
        ).filter(segment => !blockedSegments.has(segment));
        if (!currentInvalidSegments.length) break;

        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const components = connectedInvalidSegmentComponents(
            currentInvalidSegments,
            incidence,
            endpointKeys
        );
        const component = components[0];
        const componentEndpoints = invalidComponentEndpointKeys(component, incidence, endpointKeys);
        let selected = null;
        if (componentEndpoints.length === 2) {
            for (const expansionSteps of [1, 2, 3, 5, 8]) {
                const removalSet = new Set(component);
                const start = expandInvalidComponentEndpoint(
                    componentEndpoints[0],
                    removalSet,
                    incidence,
                    endpointKeys,
                    lumenField,
                    expansionSteps
                );
                const end = expandInvalidComponentEndpoint(
                    componentEndpoints[1],
                    removalSet,
                    incidence,
                    endpointKeys,
                    lumenField,
                    expansionSteps
                );
                if (!start || !end || start.key === end.key) continue;
                const startClearance = branchRoutePointClearance(start.point, lumenField);
                const endClearance = branchRoutePointClearance(end.point, lumenField);
                if (
                    !Number.isFinite(startClearance) ||
                    !Number.isFinite(endClearance) ||
                    startClearance <= connectorLumenClearance + 0.12 ||
                    endClearance <= connectorLumenClearance + 0.12
                ) continue;

                const boundsPoints = [];
                let pathLength = 0;
                for (const segment of removalSet) {
                    boundsPoints.push(segment.start, segment.end);
                    pathLength += segment.start.distanceTo(segment.end);
                }
                const span = start.point.distanceTo(end.point);
                const vesselScale = Math.max(
                    0.45,
                    Math.min(1.35, Math.min(start.radius, end.radius) * 0.45)
                );
                attemptedChainCount++;
                const origin = {
                    start: start.point.clone(),
                    end: end.point.clone(),
                    nodeStartId: start.nodeId,
                    nodeEndId: end.nodeId,
                    radiusStart: start.radius,
                    radiusEnd: end.radius,
                    source: 'stl-slice-invalid-reroute',
                    routeSegmentSource: 'stl-slice-invalid-reroute',
                    routeBoundsPoints: boundsPoints,
                    routePadding: Math.max(1.2, vesselScale * 1.8, Math.min(8, span * 0.22)),
                    routeMinimumPadding: Math.max(0.8, vesselScale * 1.4),
                    routeRadiusPaddingScale: 0.3,
                    routeRadiusPaddingBase: 0.8,
                    routeGridSpacing: vesselScale * (
                        expansionSteps === 1 ? 1.25 : expansionSteps === 2 ? 0.9 : 0.7
                    ),
                    routeMaxCells: expansionSteps === 1 ? 14000 : expansionSteps === 2 ? 28000 : 48000,
                    forceGridRoute: true,
                    validateRouteEdges: true,
                    allowWallWhenInside: false
                };
                const route = bestValidatedRouteForOrigin(
                    origin,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    lumenGeometry
                );
                if (!route?.segments?.length) continue;
                const failureReason = segmentSetValidityFailureReason(
                    route.segments,
                    wallBvh,
                    lumenField,
                    connectorLumenClearance
                );
                if (failureReason) continue;
                selected = {
                    removalSet,
                    route,
                    expansionSteps,
                    span,
                    pathLength
                };
                break;
            }
            if (!selected) {
                for (const expansionSteps of [1, 2, 3]) {
                    attemptedChainCount++;
                    selected = routeInvalidBranchComponent(
                        component,
                        componentEndpoints,
                        incidence,
                        endpointKeys,
                        lumenField,
                        wallBvh,
                        connectorLumenClearance,
                        lumenGeometry,
                        expansionSteps
                    );
                    if (selected) break;
                }
            }
        } else if (componentEndpoints.length >= 3 && componentEndpoints.length <= 6) {
            for (const expansionSteps of [1, 2]) {
                attemptedChainCount++;
                selected = routeInvalidBranchComponent(
                    component,
                    componentEndpoints,
                    incidence,
                    endpointKeys,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    lumenGeometry,
                    expansionSteps
                );
                if (selected) break;
            }
        }

        if (!selected || !replaceCenterlineSegmentSetPreservingComponents(
            segments,
            selected.removalSet,
            selected.route.segments
        )) {
            for (const segment of component) blockedSegments.add(segment);
            if (blockedComponents.length < 12) {
                blockedComponents.push({
                    segmentCount: component.length,
                    endpointCount: componentEndpoints.length,
                    sources: [...new Set(component.map(segment => segment.source || null))],
                    bounds: component.reduce((box, segment) => (
                        box.expandByPoint(segment.start).expandByPoint(segment.end)
                    ), new THREE.Box3())
                });
            }
            continue;
        }
        routedChainCount++;
        replacedSegmentCount += selected.removalSet.size;
        insertedSegmentCount += selected.route.segments.length;
        if (choices.length < 16) {
            choices.push({
                sources: [...new Set(component.map(segment => segment.source || null))],
                expansionSteps: selected.expansionSteps,
                span: selected.span,
                pathLength: selected.pathLength,
                replacedSegmentCount: selected.removalSet.size,
                insertedSegmentCount: selected.route.segments.length,
                routeType: selected.route.routeType || null
            });
        }
    }

    const remainingInvalidSegmentCount = invalidCenterlineSegments(
        segments,
        lumenField,
        wallBvh,
        connectorLumenClearance
    ).length;
    return {
        initialInvalidSegmentCount,
        remainingInvalidSegmentCount,
        attemptedChainCount,
        routedChainCount,
        replacedSegmentCount,
        insertedSegmentCount,
        blockedSegmentCount: blockedSegments.size,
        choices,
        blockedComponents
    };
}

function rerouteMeasuredOutlierChains(segments, centering, {
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    lumenGeometry = null,
    maxCandidates = CENTERLINE_OUTLIER_REROUTE_MAX_CANDIDATES,
    maxRoutedChains = Infinity
} = {}) {
    const candidates = (centering?.outlierCorrectionCandidates || []).filter(candidate => (
        candidate.normalizedOffset >= CENTERLINE_OUTLIER_RELAX_MIN_NORMALIZED ||
        (
            candidate.offset >= CENTERLINE_OUTLIER_RELAX_MIN_OFFSET &&
            candidate.normalizedOffset >= 0.35
        )
    )).sort((a, b) =>
        b.normalizedOffset - a.normalizedOffset ||
        b.offset - a.offset
    );
    if (!segments.length || !candidates.length || !lumenField?.query || !lumenGeometry?.attributes?.position) {
        return {
            attemptedChainCount: 0,
            routedChainCount: 0,
            rejectedChainCount: 0,
            replacedSegmentCount: 0,
            insertedSegmentCount: 0,
            centeredRouteNodeCount: 0,
            routeChoices: [],
            rejectedChains: []
        };
    }

    let attemptedChainCount = 0;
    let routedChainCount = 0;
    let rejectedChainCount = 0;
    let replacedSegmentCount = 0;
    let insertedSegmentCount = 0;
    let centeredRouteNodeCount = 0;
    const routeChoices = [];
    const rejectedChains = [];
    const seenKeys = new Set();
    const recordRejectedChain = (candidate, reason, details = null) => {
        if (rejectedChains.length >= 12) return;
        rejectedChains.push({
            key: candidate?.key || null,
            reason,
            offset: candidate?.offset || 0,
            normalizedOffset: candidate?.normalizedOffset || 0,
            sources: candidate?.sources || [],
            details
        });
    };

    for (const candidate of candidates) {
        if (attemptedChainCount >= maxCandidates) break;
        if (!candidate?.key || candidate.degree !== 2 || seenKeys.has(candidate.key)) continue;
        seenKeys.add(candidate.key);
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const chain = buildOutlierRerouteChain(candidate.key, incidence, endpointKeys, lumenField);
        if (!chain) {
            recordRejectedChain(candidate, 'chain');
            rejectedChainCount++;
            continue;
        }
        chain.candidate = candidate;
        attemptedChainCount++;
        const routeFailure = {
            span: chain.span,
            pathLength: chain.pathLength,
            nodeCount: chain.nodes.length
        };
        const routed = routeOutlierChain(
            chain,
            lumenField,
            wallBvh,
            connectorLumenClearance,
            lumenGeometry,
            routeFailure
        );
        if (!routed) {
            recordRejectedChain(candidate, 'route', routeFailure);
            rejectedChainCount++;
            continue;
        }
        if (!replaceCenterlineSegmentSet(segments, new Set(chain.segments), routed.segments)) {
            recordRejectedChain(candidate, 'replace');
            rejectedChainCount++;
            continue;
        }
        routedChainCount++;
        replacedSegmentCount += routed.replacedSegmentCount;
        insertedSegmentCount += routed.insertedSegmentCount;
        centeredRouteNodeCount += routed.centeredNodeCount;
        if (routeChoices.length < 12) {
            routeChoices.push({
                key: candidate.key,
                routeType: routed.routeType,
                span: routed.span,
                pathLength: routed.pathLength,
                replacedSegmentCount: routed.replacedSegmentCount,
                insertedSegmentCount: routed.insertedSegmentCount,
                originalScore: routed.originalScore,
                routeScore: routed.routeScore
            });
        }
        if (routedChainCount >= maxRoutedChains) break;
    }

    return {
        attemptedChainCount,
        routedChainCount,
        rejectedChainCount,
        replacedSegmentCount,
        insertedSegmentCount,
        centeredRouteNodeCount,
        routeChoices,
        rejectedChains
    };
}

function chainPatchNodeWeight(index, centerIndex, lastIndex, includeEndpoints = false) {
    if (!includeEndpoints && (index <= 0 || index >= lastIndex)) return 0;
    const sideWidth = index <= centerIndex
        ? centerIndex + (includeEndpoints ? 1 : 0)
        : lastIndex - centerIndex + (includeEndpoints ? 1 : 0);
    const distance = Math.abs(index - centerIndex);
    return Math.max(0, 1 - distance / Math.max(1, sideWidth));
}

function buildOutlierChainPatchUpdates(chain, candidate, scale, mode = 'both', includeEndpoints = false) {
    const measuredShift = new THREE.Vector3(
        candidate?.shift?.x || 0,
        candidate?.shift?.y || 0,
        candidate?.shift?.z || 0
    );
    if (measuredShift.lengthSq() < 1e-8 || !chain?.nodes?.length) return null;
    const centerIndex = chain.nodes.findIndex(node => node.key === candidate.key);
    if (centerIndex <= 0 || centerIndex >= chain.nodes.length - 1) return null;
    const lastIndex = chain.nodes.length - 1;
    const updates = new Map();

    const startIndex = includeEndpoints ? 0 : 1;
    const endIndex = includeEndpoints ? lastIndex : lastIndex - 1;
    for (let i = startIndex; i <= endIndex; i++) {
        if (mode === 'left' && i > centerIndex) continue;
        if (mode === 'right' && i < centerIndex) continue;
        const node = chain.nodes[i];
        const weight = chainPatchNodeWeight(i, centerIndex, lastIndex, includeEndpoints);
        if (weight <= 0.02) continue;
        updates.set(node.key, node.point.clone().addScaledVector(measuredShift, scale * weight));
    }
    return updates.size ? updates : null;
}

function buildMeasuredOutlierChainPatchUpdates(
    chain,
    refinementNodes,
    lumenBvh,
    {
        scale = 1,
        radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
        sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
        includeEndpoints = false
    } = {}
) {
    if (!chain?.nodes?.length || !refinementNodes?.size || !lumenBvh) return null;
    const centerIndex = chain.nodes.findIndex(node => node.key === chain.key);
    if (centerIndex <= 0 || centerIndex >= chain.nodes.length - 1) return null;
    const lastIndex = chain.nodes.length - 1;
    const updates = new Map();

    const startIndex = includeEndpoints ? 0 : 1;
    const endIndex = includeEndpoints ? lastIndex : lastIndex - 1;
    for (let i = startIndex; i <= endIndex; i++) {
        const chainNode = chain.nodes[i];
        const node = refinementNodes.get(chainNode.key);
        if (!node) continue;
        const weight = chainPatchNodeWeight(i, centerIndex, lastIndex, includeEndpoints);
        if (weight <= 0.02) continue;
        const measurement = nodeCenterShift(node, chainNode.point, lumenBvh, { radialSamples, sphereSamples });
        if (!measurement.pairCount) continue;
        const shift = measurement.shift.clone();
        const shiftLength = shift.length();
        if (shiftLength < 0.035) continue;
        const maxStep = Math.max(0.35, Math.min(5.5, (measurement.meanRadius || node.radius || 1) * 0.9));
        if (shiftLength > maxStep) shift.multiplyScalar(maxStep / shiftLength);
        updates.set(chainNode.key, chainNode.point.clone().addScaledVector(shift, scale * weight));
    }

    return updates.size ? updates : null;
}

function buildReferenceFieldOutlierChainPatchUpdates(
    chain,
    referenceField,
    {
        scale = 1,
        includeEndpoints = false
    } = {}
) {
    if (!chain?.nodes?.length || !referenceField?.query) return null;
    const centerIndex = chain.nodes.findIndex(node => node.key === chain.key);
    if (centerIndex <= 0 || centerIndex >= chain.nodes.length - 1) return null;
    const lastIndex = chain.nodes.length - 1;
    const updates = new Map();
    const startIndex = includeEndpoints ? 0 : 1;
    const endIndex = includeEndpoints ? lastIndex : lastIndex - 1;

    for (let i = startIndex; i <= endIndex; i++) {
        const chainNode = chain.nodes[i];
        const target = referenceFieldMedialTarget(chainNode.point, referenceField);
        if (!target?.point) continue;
        const shift = new THREE.Vector3().subVectors(target.point, chainNode.point);
        const shiftLength = shift.length();
        if (shiftLength < 0.08) continue;
        const radius = Math.max(0.5, chainNode.radius || 1);
        const maxShift = Math.max(1.2, Math.min(7.5, radius * 1.45));
        if (shiftLength > maxShift) shift.multiplyScalar(maxShift / shiftLength);
        const needsCorrection = (
            target.signedDistance < 0.18 ||
            target.lowerSignedDistance < -0.45 ||
            target.upperSignedDistance < -0.45 ||
            shiftLength > Math.max(0.75, radius * 0.18)
        );
        if (!needsCorrection) continue;
        const weight = chainPatchNodeWeight(i, centerIndex, lastIndex, includeEndpoints);
        if (weight <= 0.02) continue;
        updates.set(chainNode.key, chainNode.point.clone().addScaledVector(shift, scale * weight));
    }

    return updates.size ? updates : null;
}

function scoreOutlierChainPatchUpdates(
    updates,
    refinementNodes,
    lumenBvh,
    {
        radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
        sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES
    } = {}
) {
    if (!updates?.size || !refinementNodes?.size || !lumenBvh) return null;
    let measuredCount = 0;
    let failedCount = 0;
    let offsetSum = 0;
    let normalizedSum = 0;
    let maxOffset = 0;
    let maxNormalizedOffset = 0;
    for (const [key, point] of updates.entries()) {
        const node = refinementNodes.get(key);
        if (!node) continue;
        const measurement = nodeCenterShift(node, point, lumenBvh, { radialSamples, sphereSamples });
        if (!measurement.pairCount) {
            failedCount++;
            continue;
        }
        const offset = measurement.shift.length();
        const radius = Math.max(0.5, measurement.meanRadius || node.radius || 1);
        const normalizedOffset = offset / radius;
        measuredCount++;
        offsetSum += offset;
        normalizedSum += normalizedOffset;
        maxOffset = Math.max(maxOffset, offset);
        maxNormalizedOffset = Math.max(maxNormalizedOffset, normalizedOffset);
    }
    if (!measuredCount) return null;
    const averageOffset = offsetSum / measuredCount;
    const averageNormalizedOffset = normalizedSum / measuredCount;
    return {
        measuredCount,
        failedCount,
        averageOffset,
        maxOffset,
        averageNormalizedOffset,
        maxNormalizedOffset,
        cost:
            maxNormalizedOffset * 8 +
            averageNormalizedOffset * 2 +
            maxOffset * 0.8 +
            averageOffset * 0.25 +
            failedCount * 6
    };
}

function scoreOutlierChainPatchNeighborhood(
    updates,
    incidence,
    endpointKeys,
    lumenBvh,
    options,
    applyUpdates
) {
    const measuredKeys = new Set(updates.keys());
    for (const key of updates.keys()) {
        for (const neighbourKey of neighbourKeysForNode(key, incidence, endpointKeys)) {
            measuredKeys.add(neighbourKey);
        }
    }

    const affectedSegments = new Set();
    for (const key of measuredKeys) {
        for (const { segment } of incidence.get(key) || []) affectedSegments.add(segment);
    }
    const previewSegments = [];
    const previewKeyByOriginalKey = new Map();
    for (const segment of affectedSegments) {
        const keys = endpointKeys.get(segment);
        const previewSegment = {
            ...segment,
            start: applyUpdates && keys && updates.has(keys.start)
                ? updates.get(keys.start).clone()
                : segment.start.clone(),
            end: applyUpdates && keys && updates.has(keys.end)
                ? updates.get(keys.end).clone()
                : segment.end.clone()
        };
        previewSegments.push(previewSegment);
        if (keys?.start) {
            previewKeyByOriginalKey.set(
                keys.start,
                centerlineEndpointKey(previewSegment, 'start')
            );
        }
        if (keys?.end) {
            previewKeyByOriginalKey.set(
                keys.end,
                centerlineEndpointKey(previewSegment, 'end')
            );
        }
    }

    const previewNodes = collectRefinementNodes(previewSegments);
    const previewPoints = new Map();
    for (const key of measuredKeys) {
        const previewKey = previewKeyByOriginalKey.get(key) || key;
        const node = previewNodes.get(previewKey);
        if (node) previewPoints.set(previewKey, node.point.clone());
    }
    return scoreOutlierChainPatchUpdates(previewPoints, previewNodes, lumenBvh, options);
}

function scoreOutlierChainPatchImprovement(before, after) {
    if (!before || !after || after.failedCount > before.failedCount) return null;

    const maxNormalizedGain = before.maxNormalizedOffset - after.maxNormalizedOffset;
    const maxOffsetGain = before.maxOffset - after.maxOffset;
    const averageNormalizedGain = before.averageNormalizedOffset - after.averageNormalizedOffset;
    const averageOffsetGain = before.averageOffset - after.averageOffset;
    const costGain = before.cost - after.cost;
    const improvesWorst = maxNormalizedGain >= 0.003 || maxOffsetGain >= 0.025;
    const improvesAverage = averageNormalizedGain >= 0.0005 || averageOffsetGain >= 0.004;
    const preservesWorst = (
        after.maxNormalizedOffset <= before.maxNormalizedOffset + 0.001 &&
        after.maxOffset <= before.maxOffset + 0.015
    );
    const preservesAverage = (
        after.averageNormalizedOffset <= before.averageNormalizedOffset + 0.00035 &&
        after.averageOffset <= before.averageOffset + 0.003
    );
    if (
        costGain <= 0.005 ||
        !preservesAverage ||
        !(improvesWorst || (improvesAverage && preservesWorst))
    ) {
        return null;
    }

    return {
        costGain,
        relativeCostGain: costGain / Math.max(0.001, before.cost),
        maxNormalizedGain,
        maxOffsetGain,
        averageNormalizedGain,
        averageOffsetGain
    };
}

function patchMeasuredOutlierChains(segments, centering, {
    lumenField = null,
    referenceField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    maxCandidates = CENTERLINE_OUTLIER_CHAIN_PATCH_MAX_CANDIDATES,
    lumenGeometry = null,
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES
} = {}) {
    const candidates = (centering?.outlierCorrectionCandidates || [])
        .filter(candidate =>
            candidate?.degree === 2 &&
            candidate.normalizedOffset >= CENTERLINE_OUTLIER_CHAIN_PATCH_MIN_NORMALIZED
        )
        .sort((a, b) =>
            b.normalizedOffset - a.normalizedOffset ||
            b.offset - a.offset
        );
    if (!segments.length || !candidates.length || !lumenField?.query) {
        return {
            attemptedChainCount: 0,
            patchedChainCount: 0,
            patchedNodeCount: 0,
            rejectedChainCount: 0,
            averageShift: 0,
            maxShift: 0,
            choices: [],
            rejectedChains: []
        };
    }

    let attemptedChainCount = 0;
    let patchedChainCount = 0;
    let patchedNodeCount = 0;
    let rejectedChainCount = 0;
    let totalShift = 0;
    let maxShift = 0;
    const choices = [];
    const rejectedChains = [];
    const recordRejectedChain = (candidate, reason, details = null) => {
        if (rejectedChains.length >= 12) return;
        rejectedChains.push({
            key: candidate?.key || null,
            reason,
            offset: candidate?.offset || 0,
            normalizedOffset: candidate?.normalizedOffset || 0,
            sources: candidate?.sources || [],
            details
        });
    };

    for (const candidate of candidates) {
        if (attemptedChainCount >= maxCandidates) break;
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const chain = buildOutlierRerouteChain(candidate.key, incidence, endpointKeys, lumenField, {
            sideSteps: CENTERLINE_OUTLIER_CHAIN_PATCH_SIDE_STEPS,
            minSpan: 1.5,
            maxSpan: 56
        });
        attemptedChainCount++;
        if (!chain) {
            recordRejectedChain(candidate, 'chain');
            rejectedChainCount++;
            continue;
        }

        let bestUpdates = null;
        let bestScale = 0;
        let bestMode = 'both';
        let bestPatchScore = null;
        let bestBaselineScore = null;
        let bestImprovement = null;
        const failureDetails = [];
        const noteFailure = (mode, scale, reason) => {
            if (failureDetails.length >= 16) return;
            failureDetails.push({ mode, scale, reason });
        };
        const lumenBvh = CENTERLINE_OUTLIER_CHAIN_PATCH_MEASURED && lumenGeometry?.attributes?.position
            ? (lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry)))
            : null;
        const refinementNodes = lumenBvh ? collectRefinementNodes(segments) : null;
        const patchAttempts = [];
        if (lumenBvh && refinementNodes) {
            for (const scale of [1, 0.78, 0.58, 0.4, 0.25, 0.14]) {
                patchAttempts.push({
                    mode: 'measured',
                    scale,
                    updates: buildMeasuredOutlierChainPatchUpdates(
                        chain,
                        refinementNodes,
                        lumenBvh,
                        { scale, radialSamples, sphereSamples }
                    )
                });
            }
            for (const scale of [0.78, 0.58, 0.4, 0.25, 0.14]) {
                patchAttempts.push({
                    mode: 'measured-anchor',
                    scale,
                    updates: buildMeasuredOutlierChainPatchUpdates(
                        chain,
                        refinementNodes,
                        lumenBvh,
                        { scale, radialSamples, sphereSamples, includeEndpoints: true }
                    )
                });
            }
        }
        if (referenceField?.query) {
            for (const scale of [1, 0.82, 0.64, 0.46, 0.3, 0.18]) {
                patchAttempts.push({
                    mode: 'reference-field',
                    scale,
                    updates: buildReferenceFieldOutlierChainPatchUpdates(
                        chain,
                        referenceField,
                        { scale }
                    )
                });
            }
            for (const scale of [0.82, 0.64, 0.46, 0.3, 0.18]) {
                patchAttempts.push({
                    mode: 'reference-field-anchor',
                    scale,
                    updates: buildReferenceFieldOutlierChainPatchUpdates(
                        chain,
                        referenceField,
                        { scale, includeEndpoints: true }
                    )
                });
            }
        }
        for (const mode of ['both', 'left', 'right']) {
            for (const scale of [1, 0.78, 0.58, 0.4, 0.25, 0.14]) {
                patchAttempts.push({
                    mode,
                    scale,
                    updates: buildOutlierChainPatchUpdates(chain, candidate, scale, mode)
                });
            }
        }
        for (const mode of ['both', 'left', 'right']) {
            for (const scale of [0.78, 0.58, 0.4, 0.25, 0.14]) {
                patchAttempts.push({
                    mode: `${mode}-anchor`,
                    scale,
                    updates: buildOutlierChainPatchUpdates(chain, candidate, scale, mode, true)
                });
            }
        }
        for (const attempt of patchAttempts) {
            const { mode, scale, updates } = attempt;
            if (!updates) continue;
            let failureReason = nodePatchSegmentsValidityFailureReason(
                updates,
                incidence,
                endpointKeys,
                lumenField,
                connectorLumenClearance,
                wallBvh
            );
            let valid = !failureReason;
            let usedFallback = false;
            if (!valid && scale <= 0.4) {
                const fallbackMargin = connectorLumenClearance + (scale <= 0.25 ? 0.08 : 0.12);
                let pointsHaveClearance = true;
                for (const point of updates.values()) {
                    const clearance = branchRoutePointClearance(point, lumenField);
                    if (!Number.isFinite(clearance) || clearance <= fallbackMargin + 0.08) {
                        pointsHaveClearance = false;
                        break;
                    }
                }
                failureReason = pointsHaveClearance
                    ? nodePatchSegmentsValidityFailureReason(
                        updates,
                        incidence,
                        endpointKeys,
                        lumenField,
                        fallbackMargin,
                        null
                    )
                    : 'point-clearance';
                valid = !failureReason;
                usedFallback = valid;
            }
            if (!valid && mode.startsWith('reference-field')) {
                const fallbackMargin = 0.02;
                let pointsHaveClearance = true;
                for (const point of updates.values()) {
                    const clearance = branchRoutePointClearance(point, referenceField);
                    if (!Number.isFinite(clearance) || clearance <= fallbackMargin) {
                        pointsHaveClearance = false;
                        break;
                    }
                }
                failureReason = pointsHaveClearance
                    ? nodePatchSegmentsValidityFailureReason(
                        updates,
                        incidence,
                        endpointKeys,
                        referenceField,
                        fallbackMargin,
                        wallBvh,
                        {
                            allowWallWhenInside: true,
                            wallFallbackClearance: 0.16
                        }
                    )
                    : 'point-clearance';
                valid = !failureReason;
                usedFallback = valid;
            }
            if (!valid) {
                noteFailure(mode, scale, failureReason || 'invalid');
                continue;
            }
            const scoreOptions = { radialSamples, sphereSamples };
            const patchScore = scoreOutlierChainPatchNeighborhood(
                updates,
                incidence,
                endpointKeys,
                lumenBvh,
                scoreOptions,
                true
            );
            const baselineScore = scoreOutlierChainPatchNeighborhood(
                updates,
                incidence,
                endpointKeys,
                lumenBvh,
                scoreOptions,
                false
            );
            const improvement = scoreOutlierChainPatchImprovement(baselineScore, patchScore);
            if (!improvement) {
                noteFailure(mode, scale, 'no-local-gain');
                continue;
            }
            if (
                bestUpdates &&
                improvement.relativeCostGain <= bestImprovement.relativeCostGain + 1e-6
            ) {
                continue;
            }
            bestUpdates = updates;
            bestScale = scale;
            bestMode = usedFallback ? `${mode}-lumen` : mode;
            bestPatchScore = patchScore;
            bestBaselineScore = baselineScore;
            bestImprovement = improvement;
        }
        if (!bestUpdates) {
            recordRejectedChain(candidate, 'validity', failureDetails);
            rejectedChainCount++;
            continue;
        }

        for (const [key, point] of bestUpdates.entries()) {
            const currentPoint = currentNodePointFromIncidence(key, incidence);
            if (!currentPoint) continue;
            const shift = point.distanceTo(currentPoint);
            totalShift += shift;
            maxShift = Math.max(maxShift, shift);
            patchedNodeCount++;
        }
        applyNodePatch(bestUpdates, incidence);
        patchedChainCount++;
        if (choices.length < 12) {
            choices.push({
                key: candidate.key,
                offset: candidate.offset || 0,
                normalizedOffset: candidate.normalizedOffset || 0,
                scale: bestScale,
                mode: bestMode,
                beforeScore: bestBaselineScore ? {
                    averageOffset: bestBaselineScore.averageOffset,
                    maxOffset: bestBaselineScore.maxOffset,
                    averageNormalizedOffset: bestBaselineScore.averageNormalizedOffset,
                    maxNormalizedOffset: bestBaselineScore.maxNormalizedOffset,
                    failedCount: bestBaselineScore.failedCount,
                    cost: bestBaselineScore.cost
                } : null,
                score: bestPatchScore ? {
                    averageOffset: bestPatchScore.averageOffset,
                    maxOffset: bestPatchScore.maxOffset,
                    averageNormalizedOffset: bestPatchScore.averageNormalizedOffset,
                    maxNormalizedOffset: bestPatchScore.maxNormalizedOffset,
                    failedCount: bestPatchScore.failedCount,
                    cost: bestPatchScore.cost
                } : null,
                improvement: bestImprovement,
                patchedNodeCount: bestUpdates.size,
                span: chain.span,
                pathLength: chain.pathLength,
                sources: candidate.sources || []
            });
        }
        break;
    }

    return {
        attemptedChainCount,
        patchedChainCount,
        patchedNodeCount,
        rejectedChainCount,
        averageShift: patchedNodeCount ? totalShift / patchedNodeCount : 0,
        maxShift,
        choices,
        rejectedChains
    };
}

function removeMeasuredOutlierChainsForReconnect(segments, centering, {
    lumenField = null,
    maxCandidates = CENTERLINE_OUTLIER_RECONNECT_MAX_CANDIDATES,
    minOffset = CENTERLINE_OUTLIER_RECONNECT_MIN_OFFSET,
    minNormalizedOffset = CENTERLINE_OUTLIER_RECONNECT_MIN_NORMALIZED
} = {}) {
    const candidates = (centering?.outlierCorrectionCandidates || [])
        .filter(candidate =>
            candidate?.degree === 2 &&
            candidate.offset >= minOffset &&
            candidate.normalizedOffset >= minNormalizedOffset
        )
        .sort((a, b) =>
            b.normalizedOffset - a.normalizedOffset ||
            b.offset - a.offset
        );
    if (!segments.length || !candidates.length) {
        return {
            attemptedChainCount: 0,
            removedChainCount: 0,
            removedSegmentCount: 0,
            rejectedChainCount: 0,
            choices: []
        };
    }

    let attemptedChainCount = 0;
    let removedChainCount = 0;
    let removedSegmentCount = 0;
    let rejectedChainCount = 0;
    const choices = [];

    for (const candidate of candidates) {
        if (attemptedChainCount >= maxCandidates) break;
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const chain = buildOutlierRerouteChain(candidate.key, incidence, endpointKeys, lumenField, {
            sideSteps: CENTERLINE_OUTLIER_RECONNECT_SIDE_STEPS,
            minSpan: 1.5,
            maxSpan: CENTERLINE_OUTLIER_RECONNECT_MAX_SPAN
        });
        attemptedChainCount++;
        if (!chain || !chain.segments.length) {
            rejectedChainCount++;
            continue;
        }
        const removeSet = new Set(chain.segments);
        if (!replaceCenterlineSegmentSet(segments, removeSet, [])) {
            rejectedChainCount++;
            continue;
        }
        removedChainCount++;
        removedSegmentCount += chain.segments.length;
        if (choices.length < 12) {
            choices.push({
                key: candidate.key,
                offset: candidate.offset,
                normalizedOffset: candidate.normalizedOffset,
                span: chain.span,
                pathLength: chain.pathLength,
                removedSegmentCount: chain.segments.length,
                sources: candidate.sources || []
            });
        }
    }

    return {
        attemptedChainCount,
        removedChainCount,
        removedSegmentCount,
        rejectedChainCount,
        choices
    };
}

function contourKey(point, tolerance) {
    const x = Math.round(point.x / tolerance);
    const z = Math.round(point.z / tolerance);
    return `${x}|${z}`;
}

function edgeKey(a, b) {
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

function polygonPerimeter(points) {
    let sum = 0;
    for (let i = 0; i < points.length; i++) {
        const a = points[i];
        const b = points[(i + 1) % points.length];
        sum += Math.hypot(a.x - b.x, a.z - b.z);
    }
    return sum;
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

function pointInPolygon(point, polygon) {
    return pointInPolygonCoords(point.x, point.z, polygon);
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
    let best = Infinity;
    for (let i = 0; i < polygon.length; i++) {
        best = Math.min(best, pointSegmentDistanceSq(point, polygon[i], polygon[(i + 1) % polygon.length]));
    }
    return best;
}

function boundsDistanceSq(a, b) {
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

function bestInteriorPoint(polygon, centroid, bounds) {
    const consider = point => {
        if (!pointInPolygon(point, polygon)) return;
        const distance = distanceToPolygonSq(point, polygon);
        if (distance > bestDistance) {
            bestDistance = distance;
            best = { x: point.x, z: point.z };
        }
    };

    let best = null;
    let bestDistance = -Infinity;
    consider(centroid);
    consider({
        x: (bounds.minX + bounds.maxX) * 0.5,
        z: (bounds.minZ + bounds.maxZ) * 0.5
    });

    const width = bounds.maxX - bounds.minX;
    const height = bounds.maxZ - bounds.minZ;
    const coarseSteps = 17;
    for (let ix = 0; ix < coarseSteps; ix++) {
        const x = bounds.minX + width * (ix + 0.5) / coarseSteps;
        for (let iz = 0; iz < coarseSteps; iz++) {
            const z = bounds.minZ + height * (iz + 0.5) / coarseSteps;
            consider({ x, z });
        }
    }

    if (!best) return centroid;

    let step = Math.max(width, height) / coarseSteps;
    for (let pass = 0; pass < 7; pass++) {
        const origin = best;
        for (let ix = -2; ix <= 2; ix++) {
            for (let iz = -2; iz <= 2; iz++) {
                consider({
                    x: origin.x + ix * step,
                    z: origin.z + iz * step
                });
            }
        }
        step *= 0.45;
    }

    return best;
}

function centeredInteriorPoint(polygon, centroid, bounds, radius, compactness) {
    const medialCenter = bestInteriorPoint(polygon, centroid, bounds);
    if (!pointInPolygon(centroid, polygon)) return medialCenter;

    const dx = medialCenter.x - centroid.x;
    const dz = medialCenter.z - centroid.z;
    const shift = Math.hypot(dx, dz);
    if (shift < 1e-5) return medialCenter;

    const maxShift = Math.max(0.45, radius * (compactness > 0.48 ? 0.18 : 0.08));
    const blend = Math.min(0.36, maxShift / shift);
    const candidate = {
        x: centroid.x + dx * blend,
        z: centroid.z + dz * blend
    };
    return pointInPolygon(candidate, polygon) ? candidate : centroid;
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
    if (intersections.length !== 2) return null;
    intersections.triangleIndex = index / 3;
    return intersections;
}

function buildContourLoops(segments, tolerance, minArea) {
    const points = new Map();
    const adjacency = new Map();
    const edges = new Set();
    const edgeTriangles = new Map();

    const addPoint = point => {
        const key = contourKey(point, tolerance);
        if (!points.has(key)) points.set(key, point);
        return key;
    };
    const addEdgeTriangle = (key, triangleIndex) => {
        if (!Number.isInteger(triangleIndex)) return;
        let triangles = edgeTriangles.get(key);
        if (!triangles) {
            triangles = new Set();
            edgeTriangles.set(key, triangles);
        }
        triangles.add(triangleIndex);
    };
    const addSegment = (a, b, triangleIndex) => {
        const keyA = addPoint(a);
        const keyB = addPoint(b);
        if (keyA === keyB) return;
        const key = edgeKey(keyA, keyB);
        addEdgeTriangle(key, triangleIndex);
        if (edges.has(key)) return;
        edges.add(key);
        if (!adjacency.has(keyA)) adjacency.set(keyA, new Set());
        if (!adjacency.has(keyB)) adjacency.set(keyB, new Set());
        adjacency.get(keyA).add(keyB);
        adjacency.get(keyB).add(keyA);
    };

    for (const segment of segments) addSegment(segment[0], segment[1], segment.triangleIndex);

    const used = new Set();
    const loops = [];
    for (const edge of edges) {
        if (used.has(edge)) continue;
        const [start, firstNext] = edge.split('>');
        let previous = null;
        let current = start;
        let next = firstNext;
        const loopKeys = [start];
        const loopEdges = [];
        let closed = false;

        for (let guard = 0; guard < 8000; guard++) {
            const currentEdge = edgeKey(current, next);
            used.add(currentEdge);
            loopEdges.push(currentEdge);
            previous = current;
            current = next;
            loopKeys.push(current);
            if (current === start) {
                closed = true;
                break;
            }
            const neighbours = [...(adjacency.get(current) || [])];
            let candidate = neighbours.find(neighbour =>
                neighbour !== previous &&
                !used.has(edgeKey(current, neighbour))
            );
            if (!candidate) candidate = neighbours.find(neighbour => neighbour !== previous);
            if (!candidate) break;
            next = candidate;
        }

        if (!closed || loopKeys.length < 9) continue;
        const polygon = loopKeys.slice(0, -1).map(key => points.get(key));
        const signedArea = polygonArea(polygon);
        const area = Math.abs(signedArea);
        if (area < minArea) continue;
        const triangleIndices = new Set();
        for (const loopEdge of loopEdges) {
            for (const triangleIndex of edgeTriangles.get(loopEdge) || []) {
                triangleIndices.add(triangleIndex);
            }
        }
        loops.push({
            polygon,
            area,
            triangleIndices: [...triangleIndices],
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

function extractStlSlices(
    geometry,
    spacing,
    contourTolerance,
    minArea,
    minCompactness,
    maxRadius = Infinity,
    { solidLumen = false, centerMode = 'blended' } = {}
) {
    geometry.computeBoundingBox();
    const position = geometry.attributes.position;
    const box = geometry.boundingBox;
    const slices = [];
    if (!position || !box) return slices;

    const firstY = box.min.y + spacing * 0.32;
    const sliceYs = [];
    for (let y = firstY; y <= box.max.y - spacing * 0.2; y += spacing) {
        sliceYs.push(y);
    }
    const segmentsBySlice = sliceYs.map(() => []);
    if (!sliceYs.length) return slices;

    for (let i = 0; i < position.count; i += 3) {
        const y0 = position.getY(i);
        const y1 = position.getY(i + 1);
        const y2 = position.getY(i + 2);
        const minY = Math.min(y0, y1, y2);
        const maxY = Math.max(y0, y1, y2);
        if (maxY <= sliceYs[0] || minY >= sliceYs[sliceYs.length - 1]) continue;

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
        const loops = buildContourLoops(segmentsBySlice[sliceIndex], contourTolerance, minArea);
        const contours = [];
        for (const loop of loops) {
            if (solidLumen ? loop.depth % 2 !== 0 : loop.depth % 2 !== 1) continue;
            const bounds = polygonBounds(loop.polygon);
            const perimeter = polygonPerimeter(loop.polygon);
            const compactness = perimeter > 1e-6
                ? 4 * Math.PI * loop.area / (perimeter * perimeter)
                : 0;
            if (compactness < minCompactness) continue;
            const radius = Math.max(MIN_RADIUS, Math.sqrt(loop.area / Math.PI));
            if (radius > maxRadius) continue;
            const medialCenter = bestInteriorPoint(loop.polygon, loop.centroid, bounds);
            const linkCenter = pointInPolygon(loop.centroid, loop.polygon)
                ? loop.centroid
                : medialCenter;
            const center = centerMode === 'medial'
                ? medialCenter
                : centeredInteriorPoint(loop.polygon, loop.centroid, bounds, radius, compactness);
            contours.push({
                polygon: loop.polygon,
                area: loop.area,
                bounds,
                compactness,
                center,
                linkCenter,
                radius,
                triangleIndices: loop.triangleIndices || []
            });
        }
        if (contours.length) {
            slices.push({
                rawIndex: sliceIndex,
                y: sliceYs[sliceIndex],
                contours
            });
        }
    }
    return slices;
}

function cloneSlicesForCenterline(slices) {
    return (slices || []).map(slice => ({
        ...slice,
        contours: slice.contours.map(contour => ({
            ...contour,
            center: { ...contour.center },
            linkCenter: { ...(contour.linkCenter || contour.center) },
            bounds: { ...contour.bounds },
            polygon: contour.polygon.map(point => ({ x: point.x, z: point.z })),
            triangleIndices: [...(contour.triangleIndices || [])]
        }))
    }));
}

function contourWorldCenter(slice, contour) {
    const center = contour.center || contour.linkCenter || contour.centroid || { x: 0, z: 0 };
    return new THREE.Vector3(center.x, slice.y, center.z);
}

function contourOverlapScore(a, b) {
    const dx = a.bounds.maxX < b.bounds.minX
        ? b.bounds.minX - a.bounds.maxX
        : b.bounds.maxX < a.bounds.minX
            ? a.bounds.minX - b.bounds.maxX
            : 0;
    const dz = a.bounds.maxZ < b.bounds.minZ
        ? b.bounds.minZ - a.bounds.maxZ
        : b.bounds.maxZ < a.bounds.minZ
            ? a.bounds.minZ - b.bounds.maxZ
            : 0;
    return Math.hypot(dx, dz);
}

function contourPairCandidate(sliceA, contourA, sliceB, contourB) {
    const centerA = contourWorldCenter(sliceA, contourA);
    const centerB = contourWorldCenter(sliceB, contourB);
    const centerDistance = centerA.distanceTo(centerB);
    const boundsGap = contourOverlapScore(contourA, contourB);
    const radiusSum = contourA.radius + contourB.radius;
    const limit = Math.max(8, radiusSum * 1.25 + Math.abs(sliceB.y - sliceA.y) * 0.65);
    if (centerDistance > limit && boundsGap > Math.max(3, Math.min(contourA.radius, contourB.radius))) return null;
    const areaRatio = Math.max(contourA.area, contourB.area) / Math.max(1, Math.min(contourA.area, contourB.area));
    return {
        contourA,
        contourB,
        cost: centerDistance / Math.max(1, radiusSum) + boundsGap * 0.08 + Math.abs(Math.log(areaRatio)) * 0.18
    };
}

function matchContourRings(sliceA, sliceB) {
    const candidates = [];
    for (const contourA of sliceA.contours) {
        for (const contourB of sliceB.contours) {
            const candidate = contourPairCandidate(sliceA, contourA, sliceB, contourB);
            if (candidate) candidates.push(candidate);
        }
    }
    candidates.sort((a, b) => a.cost - b.cost);
    const usedA = new Set();
    const usedB = new Set();
    const matches = [];
    for (const candidate of candidates) {
        if (usedA.has(candidate.contourA) || usedB.has(candidate.contourB)) continue;
        usedA.add(candidate.contourA);
        usedB.add(candidate.contourB);
        matches.push(candidate);
    }
    return matches;
}

function polygonPerimeterWithSegments(points) {
    const lengths = [];
    let total = 0;
    for (let i = 0; i < points.length; i++) {
        const a = points[i];
        const b = points[(i + 1) % points.length];
        total += Math.hypot(a.x - b.x, a.z - b.z);
        lengths.push(total);
    }
    return { lengths, total };
}

function resamplePolygonRing(points, count = LUMEN_CAST_RING_SEGMENTS) {
    if (!points.length) return [];
    const { lengths, total } = polygonPerimeterWithSegments(points);
    if (total < 1e-6) return Array.from({ length: count }, () => ({ ...points[0] }));
    const ring = [];
    let edgeIndex = 0;
    for (let i = 0; i < count; i++) {
        const target = total * i / count;
        while (edgeIndex < lengths.length - 1 && lengths[edgeIndex] < target) edgeIndex++;
        const previousLength = edgeIndex === 0 ? 0 : lengths[edgeIndex - 1];
        const edgeLength = Math.max(1e-6, lengths[edgeIndex] - previousLength);
        const t = (target - previousLength) / edgeLength;
        const a = points[edgeIndex];
        const b = points[(edgeIndex + 1) % points.length];
        ring.push({
            x: a.x + (b.x - a.x) * t,
            z: a.z + (b.z - a.z) * t
        });
    }
    return ring;
}

function ringAlignmentCost(reference, candidate, offset, reversed = false) {
    let cost = 0;
    const count = reference.length;
    for (let i = 0; i < count; i++) {
        const point = reversed
            ? candidate[(offset - i + count) % count]
            : candidate[(offset + i) % count];
        const dx = reference[i].x - point.x;
        const dz = reference[i].z - point.z;
        cost += dx * dx + dz * dz;
    }
    return cost;
}

function alignRingToReference(reference, ring) {
    if (!reference?.length || reference.length !== ring?.length) return ring || [];
    let bestOffset = 0;
    let bestReversed = false;
    let bestCost = Infinity;
    for (let offset = 0; offset < ring.length; offset++) {
        const cost = ringAlignmentCost(reference, ring, offset, false);
        if (cost < bestCost) {
            bestCost = cost;
            bestOffset = offset;
            bestReversed = false;
        }
        const reversedCost = ringAlignmentCost(reference, ring, offset, true);
        if (reversedCost < bestCost) {
            bestCost = reversedCost;
            bestOffset = offset;
            bestReversed = true;
        }
    }
    return Array.from({ length: ring.length }, (_, index) => {
        const sourceIndex = bestReversed
            ? (bestOffset - index + ring.length) % ring.length
            : (bestOffset + index) % ring.length;
        return ring[sourceIndex];
    });
}

function pushCastVertex(positions, point, y) {
    positions.push(point.x, y, point.z);
}

function appendCastQuad(positions, a0, a1, b0, b1, yA, yB) {
    pushCastVertex(positions, a0, yA);
    pushCastVertex(positions, b0, yB);
    pushCastVertex(positions, a1, yA);
    pushCastVertex(positions, a1, yA);
    pushCastVertex(positions, b0, yB);
    pushCastVertex(positions, b1, yB);
}

function buildLumenCastGeometry(slices) {
    const positions = [];
    let ringPairCount = 0;
    let skippedPairCount = 0;

    for (const slice of slices) {
        for (const contour of slice.contours) {
            contour.castRing = resamplePolygonRing(contour.polygon);
        }
    }

    for (let i = 0; i < slices.length - 1; i++) {
        const sliceA = slices[i];
        const sliceB = slices[i + 1];
        const matches = matchContourRings(sliceA, sliceB);
        if (!matches.length) {
            skippedPairCount++;
            continue;
        }
        for (const { contourA, contourB } of matches) {
            const ringA = contourA.castRing;
            const ringB = alignRingToReference(ringA, contourB.castRing);
            if (!ringA?.length || ringA.length !== ringB?.length) continue;
            for (let ringIndex = 0; ringIndex < ringA.length; ringIndex++) {
                appendCastQuad(
                    positions,
                    ringA[ringIndex],
                    ringA[(ringIndex + 1) % ringA.length],
                    ringB[ringIndex],
                    ringB[(ringIndex + 1) % ringB.length],
                    sliceA.y,
                    sliceB.y
                );
            }
            ringPairCount++;
        }
    }

    for (const slice of slices) {
        for (const contour of slice.contours) delete contour.castRing;
    }

    const castGeometry = new THREE.BufferGeometry();
    castGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    castGeometry.computeVertexNormals();
    castGeometry.computeBoundingBox();
    castGeometry.computeBoundingSphere();
    castGeometry.userData = {
        type: 'lumen-cast-loft',
        ringPairCount,
        skippedPairCount
    };
    return {
        geometry: castGeometry,
        diagnostics: {
            source: 'lumen-cast-loft',
            ringPairCount,
            skippedPairCount,
            triangleCount: positions.length / 9,
            vertexCount: positions.length / 3
        }
    };
}

function buildSliceTaggedInnerSurfaceGeometry(geometry, slices) {
    const position = geometry?.attributes?.position;
    if (!position) return null;

    const taggedTriangles = new Set();
    for (const slice of slices || []) {
        for (const contour of slice.contours || []) {
            for (const triangleIndex of contour.triangleIndices || []) {
                taggedTriangles.add(triangleIndex);
            }
        }
    }
    if (taggedTriangles.size < 1000) return null;

    const triangleCount = Math.floor(position.count / 3);
    const vertexKeyAt = index => [
        Math.round(position.getX(index) * 1000),
        Math.round(position.getY(index) * 1000),
        Math.round(position.getZ(index) * 1000)
    ].join(',');
    const triangleEdgeKeys = triangleIndex => {
        const offset = triangleIndex * 3;
        const keys = [
            vertexKeyAt(offset),
            vertexKeyAt(offset + 1),
            vertexKeyAt(offset + 2)
        ];
        return [
            keys[0] < keys[1] ? `${keys[0]}|${keys[1]}` : `${keys[1]}|${keys[0]}`,
            keys[1] < keys[2] ? `${keys[1]}|${keys[2]}` : `${keys[2]}|${keys[1]}`,
            keys[2] < keys[0] ? `${keys[2]}|${keys[0]}` : `${keys[0]}|${keys[2]}`
        ];
    };
    const edgeTriangles = new Map();
    const normals = new Float32Array(triangleCount * 3);
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const c = new THREE.Vector3();
    const ab = new THREE.Vector3();
    const ac = new THREE.Vector3();
    const normal = new THREE.Vector3();

    for (let triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++) {
        const offset = triangleIndex * 3;
        a.fromBufferAttribute(position, offset);
        b.fromBufferAttribute(position, offset + 1);
        c.fromBufferAttribute(position, offset + 2);
        ab.subVectors(b, a);
        ac.subVectors(c, a);
        normal.crossVectors(ab, ac);
        if (normal.lengthSq() > 1e-10) normal.normalize();
        normals[triangleIndex * 3] = normal.x;
        normals[triangleIndex * 3 + 1] = normal.y;
        normals[triangleIndex * 3 + 2] = normal.z;

        for (const edgeKeyValue of triangleEdgeKeys(triangleIndex)) {
            let bucket = edgeTriangles.get(edgeKeyValue);
            if (!bucket) {
                bucket = [];
                edgeTriangles.set(edgeKeyValue, bucket);
            }
            bucket.push(triangleIndex);
        }
    }

    const accepted = new Uint8Array(triangleCount);
    const expansionDepth = new Int8Array(triangleCount);
    expansionDepth.fill(-1);
    const stack = [];
    for (const triangleIndex of taggedTriangles) {
        if (triangleIndex < 0 || triangleIndex >= triangleCount || accepted[triangleIndex]) continue;
        accepted[triangleIndex] = 1;
        expansionDepth[triangleIndex] = 0;
        stack.push(triangleIndex);
    }

    const normalDot = (aIndex, bIndex) => Math.abs(
        normals[aIndex * 3] * normals[bIndex * 3] +
        normals[aIndex * 3 + 1] * normals[bIndex * 3 + 1] +
        normals[aIndex * 3 + 2] * normals[bIndex * 3 + 2]
    );
    const SMOOTH_SURFACE_DOT = 0.52;
    const MAX_TOPOLOGY_EXPANSION_RINGS = 3;
    while (stack.length) {
        const triangleIndex = stack.pop();
        const depth = expansionDepth[triangleIndex];
        if (depth >= MAX_TOPOLOGY_EXPANSION_RINGS) continue;
        for (const edgeKeyValue of triangleEdgeKeys(triangleIndex)) {
            for (const neighbour of edgeTriangles.get(edgeKeyValue) || []) {
                if (accepted[neighbour]) continue;
                if (normalDot(triangleIndex, neighbour) < SMOOTH_SURFACE_DOT) continue;
                accepted[neighbour] = 1;
                expansionDepth[neighbour] = depth + 1;
                stack.push(neighbour);
            }
        }
    }

    const positions = [];
    const triangleIndices = [];
    for (let triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++) {
        if (accepted[triangleIndex]) triangleIndices.push(triangleIndex);
    }
    for (const triangleIndex of triangleIndices) {
        const offset = triangleIndex * 3;
        if (offset + 2 >= position.count) continue;
        positions.push(
            position.getX(offset), position.getY(offset), position.getZ(offset),
            position.getX(offset + 1), position.getY(offset + 1), position.getZ(offset + 1),
            position.getX(offset + 2), position.getY(offset + 2), position.getZ(offset + 2)
        );
    }

    const castGeometry = new THREE.BufferGeometry();
    castGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    castGeometry.computeVertexNormals();
    castGeometry.computeBoundingBox();
    castGeometry.computeBoundingSphere();
    castGeometry.userData = {
        type: 'lumen-cast-stl-inner-surface',
        taggedTriangleCount: taggedTriangles.size,
        expandedTriangleCount: triangleIndices.length
    };

    return {
        geometry: castGeometry,
        diagnostics: {
            source: 'lumen-cast-stl-inner-surface',
            triangleCount: positions.length / 9,
            vertexCount: positions.length / 3,
            taggedTriangleCount: taggedTriangles.size,
            expandedTriangleCount: triangleIndices.length,
            topologyExpansionRings: MAX_TOPOLOGY_EXPANSION_RINGS,
            sourceTriangleCount: position.count / 3
        }
    };
}

function buildInnerSurfaceLumenCastGeometry(geometry, lumenField, sampleOffsets = [0.28, 0.5, 0.8]) {
    const position = geometry?.attributes?.position;
    if (!position || !lumenField?.query) return null;
    const offsets = Array.isArray(sampleOffsets) ? sampleOffsets : [sampleOffsets];

    const positions = [];
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const c = new THREE.Vector3();
    const ab = new THREE.Vector3();
    const ac = new THREE.Vector3();
    const normal = new THREE.Vector3();
    const center = new THREE.Vector3();
    const sample = new THREE.Vector3();
    const queryScratch = {};
    let degenerateTriangleCount = 0;
    let includedTriangleCount = 0;
    let oneSidedLumenTriangleCount = 0;
    let rejectedDistantTriangleCount = 0;
    let rejectedAmbiguousTriangleCount = 0;

    const pushTriangle = () => {
        positions.push(
            a.x, a.y, a.z,
            b.x, b.y, b.z,
            c.x, c.y, c.z
        );
        includedTriangleCount++;
    };

    const signedDistanceAt = offset => {
        sample.copy(center).addScaledVector(normal, offset);
        const state = lumenField.query(sample, queryScratch);
        return Number.isFinite(state?.signedDistance) ? state.signedDistance : -Infinity;
    };

    const sideMaxSignedDistance = sign => {
        let best = -Infinity;
        for (const offset of offsets) {
            best = Math.max(best, signedDistanceAt(sign * offset));
        }
        return best;
    };

    for (let i = 0; i < position.count; i += 3) {
        a.fromBufferAttribute(position, i);
        b.fromBufferAttribute(position, i + 1);
        c.fromBufferAttribute(position, i + 2);
        ab.subVectors(b, a);
        ac.subVectors(c, a);
        normal.crossVectors(ab, ac);
        const normalLength = normal.length();
        if (normalLength < 1e-8) {
            degenerateTriangleCount++;
            continue;
        }
        normal.multiplyScalar(1 / normalLength);
        center.copy(a).add(b).add(c).multiplyScalar(1 / 3);
        const centerSignedDistance = signedDistanceAt(0);
        if (centerSignedDistance < -1.0) {
            rejectedDistantTriangleCount++;
            continue;
        }

        const positiveSide = sideMaxSignedDistance(1);
        const negativeSide = sideMaxSignedDistance(-1);
        const positiveTouchesLumen = positiveSide > 0.02;
        const negativeTouchesLumen = negativeSide > 0.02;
        if (positiveTouchesLumen === negativeTouchesLumen) {
            rejectedAmbiguousTriangleCount++;
            continue;
        }
        oneSidedLumenTriangleCount++;
        if (positiveTouchesLumen || negativeTouchesLumen) {
            pushTriangle();
        }
    }

    if (includedTriangleCount < 1000) return null;

    const castGeometry = new THREE.BufferGeometry();
    castGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    castGeometry.computeVertexNormals();
    castGeometry.computeBoundingBox();
    castGeometry.computeBoundingSphere();
    castGeometry.userData = {
        type: 'lumen-cast-inner-surface',
        sampleOffsets: offsets
    };

    return {
        geometry: castGeometry,
        diagnostics: {
            source: 'lumen-cast-inner-surface',
            sampleOffsets: offsets,
            triangleCount: positions.length / 9,
            vertexCount: positions.length / 3,
            includedTriangleCount,
            oneSidedLumenTriangleCount,
            rejectedDistantTriangleCount,
            rejectedAmbiguousTriangleCount,
            sourceTriangleCount: position.count / 3,
            degenerateTriangleCount
        }
    };
}

function analyzeIndexedTriangleComponents(indices, vertexCount) {
    const triangleCount = indices.length / 3;
    if (!triangleCount) {
        return {
            componentCount: 0,
            largestTriangleCount: 0,
            discardedTriangleCount: 0
        };
    }

    const trianglesByVertex = Array.from({ length: vertexCount }, () => []);
    for (let triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++) {
        trianglesByVertex[indices[triangleIndex * 3]].push(triangleIndex);
        trianglesByVertex[indices[triangleIndex * 3 + 1]].push(triangleIndex);
        trianglesByVertex[indices[triangleIndex * 3 + 2]].push(triangleIndex);
    }

    const seen = new Uint8Array(triangleCount);
    const components = [];
    for (let triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++) {
        if (seen[triangleIndex]) continue;
        const stack = [triangleIndex];
        const component = [];
        seen[triangleIndex] = 1;

        while (stack.length) {
            const current = stack.pop();
            component.push(current);
            for (let corner = 0; corner < 3; corner++) {
                const vertexIndex = indices[current * 3 + corner];
                for (const neighbour of trianglesByVertex[vertexIndex]) {
                    if (seen[neighbour]) continue;
                    seen[neighbour] = 1;
                    stack.push(neighbour);
                }
            }
        }
        components.push(component);
    }

    components.sort((a, b) => b.length - a.length);
    const largest = components[0] || [];
    return {
        componentCount: components.length,
        largestTriangleCount: largest.length,
        discardedTriangleCount: triangleCount - largest.length,
        componentSizes: components.slice(0, 8).map(component => component.length)
    };
}

function gridIndex(i, j, k, nx, ny) {
    return (k * ny + j) * nx + i;
}

function cellIndex(i, j, k, nx, ny) {
    return (k * ny + j) * nx + i;
}

function queryLumenCastSlice(slice, x, z) {
    const point = { x, z };
    let bestSignedDistance = -Infinity;
    for (const contour of slice.contours) {
        const bounds = contour.bounds;
        if (bestSignedDistance < 0 && bounds) {
            const dx = x < bounds.minX
                ? bounds.minX - x
                : x > bounds.maxX
                    ? x - bounds.maxX
                    : 0;
            const dz = z < bounds.minZ
                ? bounds.minZ - z
                : z > bounds.maxZ
                    ? z - bounds.maxZ
                    : 0;
            if (dx || dz) {
                const boundSignedDistance = -Math.hypot(dx, dz);
                if (boundSignedDistance <= bestSignedDistance) continue;
            }
        }

        const inside = pointInPolygonCoords(x, z, contour.polygon);
        const distance = Math.sqrt(distanceToPolygonSq(point, contour.polygon));
        const signedDistance = inside ? distance : -distance;
        if (signedDistance > bestSignedDistance) bestSignedDistance = signedDistance;
    }
    return bestSignedDistance;
}

function findLumenCastSliceInterval(slices, y, cache) {
    if (slices.length <= 1) return { lower: 0, upper: 0, t: 0 };
    if (y <= slices[0].y) return { lower: 0, upper: 0, t: 0 };
    const last = slices.length - 1;
    if (y >= slices[last].y) return { lower: last, upper: last, t: 0 };

    if (cache) {
        let lower = Math.max(0, Math.min(last, cache.lower || 0));
        let upper = Math.max(lower, Math.min(last, cache.upper || lower + 1));
        while (lower > 0 && y < slices[lower].y) {
            upper = lower;
            lower--;
        }
        while (upper < last && y > slices[upper].y) {
            lower = upper;
            upper++;
        }
        if (lower !== upper && slices[lower].y <= y && y <= slices[upper].y) {
            const span = Math.max(1e-6, slices[upper].y - slices[lower].y);
            cache.lower = lower;
            cache.upper = upper;
            cache.t = THREE.MathUtils.clamp((y - slices[lower].y) / span, 0, 1);
            return cache;
        }
    }

    let lo = 0;
    let hi = last;
    while (hi - lo > 1) {
        const mid = Math.floor((lo + hi) * 0.5);
        if (slices[mid].y <= y) lo = mid;
        else hi = mid;
    }
    const span = Math.max(1e-6, slices[hi].y - slices[lo].y);
    const interval = {
        lower: lo,
        upper: hi,
        t: THREE.MathUtils.clamp((y - slices[lo].y) / span, 0, 1)
    };
    if (cache) Object.assign(cache, interval);
    return interval;
}

function createSliceLumenCastField(slices) {
    const sortedSlices = (slices || [])
        .filter(slice => slice?.contours?.length)
        .slice()
        .sort((a, b) => a.y - b.y);
    const intervalCache = { lower: 0, upper: 1, t: 0 };

    return {
        query(input, out = null) {
            if (!sortedSlices.length) {
                const result = out || {};
                result.signedDistance = -Infinity;
                result.inside = false;
                return result;
            }

            const interval = findLumenCastSliceInterval(sortedSlices, input.y, intervalCache);
            const lowerSlice = sortedSlices[interval.lower];
            const upperSlice = sortedSlices[interval.upper];
            const lower = queryLumenCastSlice(lowerSlice, input.x, input.z);
            const upper = interval.upper === interval.lower
                ? lower
                : queryLumenCastSlice(upperSlice, input.x, input.z);
            const signedDistance = lower * (1 - interval.t) + upper * interval.t;
            const result = out || {};
            result.signedDistance = signedDistance;
            result.inside = signedDistance >= 0;
            return result;
        }
    };
}

function createMultiAxisLumenCastField(axisSlices) {
    const axisFields = axisSlices
        .filter(entry => entry?.slices?.length)
        .map(entry => ({
            axis: entry.axis,
            field: createSliceLumenCastField(entry.slices)
        }));

    return {
        query(input, out = null) {
            let bestSignedDistance = -Infinity;
            for (const { axis, field } of axisFields) {
                const local = axisPointToLocal(input, axis);
                const query = field.query(local);
                if (query.signedDistance > bestSignedDistance) {
                    bestSignedDistance = query.signedDistance;
                }
            }
            const result = out || {};
            result.signedDistance = bestSignedDistance;
            result.inside = bestSignedDistance >= 0;
            return result;
        }
    };
}

function extractLumenCastTaggingSlices(
    geometry,
    baseSliceSpacing,
    contourTolerance,
    minLumenArea,
    minCompactness,
    primarySlices = null
) {
    return CENTERLINE_AXES.map((axis, axisIndex) => {
        const descriptor = centerlineAxisDescriptor(axis, axisIndex);
        const axisId = descriptor.id;
        const axisGeometry = axisId === 'y'
            ? geometry
            : buildAxisGeometry(geometry, descriptor);
        const slices = axisId === 'y' && primarySlices
            ? primarySlices
            : axisId === 'y'
            ? extractStlSlices(
                geometry,
                baseSliceSpacing,
                contourTolerance,
                minLumenArea,
                minCompactness
            )
            : extractStlSlices(
                axisGeometry,
                baseSliceSpacing,
                contourTolerance,
                minLumenArea,
                minCompactness
            );
        return {
            axis: descriptor,
            axisId,
            slices,
            contourCount: slices.reduce((sum, slice) => sum + slice.contours.length, 0),
            taggedTriangleCount: new Set(slices.flatMap(slice =>
                slice.contours.flatMap(contour => contour.triangleIndices || [])
            )).size
        };
    });
}

function buildVolumeLumenCastGeometry(geometry, lumenField, {
    gridSpacing = LUMEN_CAST_GRID_SPACING,
    isoLevel = LUMEN_CAST_ISO_LEVEL
} = {}) {
    if (!geometry?.attributes?.position || !lumenField?.query) return null;
    geometry.computeBoundingBox();
    const box = geometry.boundingBox;
    if (!box) return null;

    const size = box.getSize(new THREE.Vector3());
    const nx = Math.max(2, Math.ceil(size.x / gridSpacing) + 1);
    const ny = Math.max(2, Math.ceil(size.y / gridSpacing) + 1);
    const nz = Math.max(2, Math.ceil(size.z / gridSpacing) + 1);
    const stepX = size.x / Math.max(1, nx - 1);
    const stepY = size.y / Math.max(1, ny - 1);
    const stepZ = size.z / Math.max(1, nz - 1);
    const values = new Float32Array(nx * ny * nz);
    const queryPoint = new THREE.Vector3();
    const queryScratch = {};

    for (let k = 0; k < nz; k++) {
        const z = box.min.z + k * stepZ;
        for (let j = 0; j < ny; j++) {
            const y = box.min.y + j * stepY;
            for (let i = 0; i < nx; i++) {
                const x = box.min.x + i * stepX;
                queryPoint.set(x, y, z);
                const state = lumenField.query(queryPoint, queryScratch);
                values[gridIndex(i, j, k, nx, ny)] = Number.isFinite(state?.signedDistance)
                    ? state.signedDistance
                    : -gridSpacing;
            }
        }
    }

    const cellNx = nx - 1;
    const cellNy = ny - 1;
    const cellNz = nz - 1;
    const cellVertices = new Int32Array(cellNx * cellNy * cellNz);
    cellVertices.fill(-1);
    const vertices = [];
    const indices = [];
    const cornerOffsets = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1]
    ];
    const cornerEdges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ];
    const cornerValues = new Array(8);
    const cornerPositions = new Array(8);

    const makeCornerPosition = (i, j, k, offset) => new THREE.Vector3(
        box.min.x + (i + offset[0]) * stepX,
        box.min.y + (j + offset[1]) * stepY,
        box.min.z + (k + offset[2]) * stepZ
    );

    for (let k = 0; k < cellNz; k++) {
        for (let j = 0; j < cellNy; j++) {
            for (let i = 0; i < cellNx; i++) {
                let hasInside = false;
                let hasOutside = false;
                for (let cornerIndex = 0; cornerIndex < 8; cornerIndex++) {
                    const offset = cornerOffsets[cornerIndex];
                    const value = values[gridIndex(i + offset[0], j + offset[1], k + offset[2], nx, ny)];
                    cornerValues[cornerIndex] = value;
                    cornerPositions[cornerIndex] = makeCornerPosition(i, j, k, offset);
                    if (value >= isoLevel) hasInside = true;
                    else hasOutside = true;
                }
                if (!hasInside || !hasOutside) continue;

                const vertex = new THREE.Vector3();
                let intersectionCount = 0;
                for (const [aIndex, bIndex] of cornerEdges) {
                    const aValue = cornerValues[aIndex];
                    const bValue = cornerValues[bIndex];
                    if ((aValue >= isoLevel) === (bValue >= isoLevel)) continue;
                    const t = THREE.MathUtils.clamp((isoLevel - aValue) / (bValue - aValue), 0, 1);
                    vertex.add(cornerPositions[aIndex].clone().lerp(cornerPositions[bIndex], t));
                    intersectionCount++;
                }
                if (!intersectionCount) continue;

                vertex.multiplyScalar(1 / intersectionCount);
                const vertexIndex = vertices.length / 3;
                vertices.push(vertex.x, vertex.y, vertex.z);
                cellVertices[cellIndex(i, j, k, cellNx, cellNy)] = vertexIndex;
            }
        }
    }

    const valueAt = (i, j, k) => values[gridIndex(i, j, k, nx, ny)];
    const cellVertexAt = (i, j, k) => {
        if (i < 0 || j < 0 || k < 0 || i >= cellNx || j >= cellNy || k >= cellNz) return -1;
        return cellVertices[cellIndex(i, j, k, cellNx, cellNy)];
    };
    const edgeCrosses = (a, b) => (a >= isoLevel) !== (b >= isoLevel);
    const addQuad = (a, b, c, d) => {
        if (a < 0 || b < 0 || c < 0 || d < 0) return;
        indices.push(a, b, c, a, c, d);
    };

    for (let k = 1; k < nz - 1; k++) {
        for (let j = 1; j < ny - 1; j++) {
            for (let i = 0; i < nx - 1; i++) {
                if (!edgeCrosses(valueAt(i, j, k), valueAt(i + 1, j, k))) continue;
                addQuad(
                    cellVertexAt(i, j - 1, k - 1),
                    cellVertexAt(i, j, k - 1),
                    cellVertexAt(i, j, k),
                    cellVertexAt(i, j - 1, k)
                );
            }
        }
    }

    for (let k = 1; k < nz - 1; k++) {
        for (let j = 0; j < ny - 1; j++) {
            for (let i = 1; i < nx - 1; i++) {
                if (!edgeCrosses(valueAt(i, j, k), valueAt(i, j + 1, k))) continue;
                addQuad(
                    cellVertexAt(i - 1, j, k - 1),
                    cellVertexAt(i, j, k - 1),
                    cellVertexAt(i, j, k),
                    cellVertexAt(i - 1, j, k)
                );
            }
        }
    }

    for (let k = 0; k < nz - 1; k++) {
        for (let j = 1; j < ny - 1; j++) {
            for (let i = 1; i < nx - 1; i++) {
                if (!edgeCrosses(valueAt(i, j, k), valueAt(i, j, k + 1))) continue;
                addQuad(
                    cellVertexAt(i - 1, j - 1, k),
                    cellVertexAt(i, j - 1, k),
                    cellVertexAt(i, j, k),
                    cellVertexAt(i - 1, j, k)
                );
            }
        }
    }

    if (indices.length < 3000) return null;
    const componentDiagnostics = analyzeIndexedTriangleComponents(indices, vertices.length / 3);

    const indexedGeometry = new THREE.BufferGeometry();
    indexedGeometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    indexedGeometry.setIndex(indices);
    indexedGeometry.computeVertexNormals();
    indexedGeometry.computeBoundingBox();
    indexedGeometry.computeBoundingSphere();

    const castGeometry = indexedGeometry.toNonIndexed();
    castGeometry.computeVertexNormals();
    castGeometry.computeBoundingBox();
    castGeometry.computeBoundingSphere();
    castGeometry.userData = {
        type: 'lumen-cast-volume',
        gridSpacing,
        isoLevel
    };

    return {
        geometry: castGeometry,
        diagnostics: {
            source: 'lumen-cast-volume',
            gridSpacing,
            isoLevel,
            gridSize: { x: nx, y: ny, z: nz },
            cellVertexCount: vertices.length / 3,
            componentCount: componentDiagnostics.componentCount,
            largestComponentTriangleCount: componentDiagnostics.largestTriangleCount,
            disconnectedTriangleCount: componentDiagnostics.discardedTriangleCount,
            componentSizes: componentDiagnostics.componentSizes,
            triangleCount: indices.length / 3,
            vertexCount: castGeometry.attributes.position.count
        }
    };
}

export function buildStlLumenCast(geometry, {
    sliceSpacing = DEFAULT_SLICE_SPACING,
    targetSliceCount = DEFAULT_TARGET_SLICE_COUNT,
    taggingTargetSliceCount = DEFAULT_LUMEN_CAST_TAG_TARGET_SLICE_COUNT,
    contourTolerance = DEFAULT_CONTOUR_TOLERANCE,
    minLumenArea = DEFAULT_MIN_LUMEN_AREA,
    minCompactness = DEFAULT_MIN_COMPACTNESS,
    lumenField = null,
    fieldOnly = false
} = {}) {
    const baseSliceSpacing = resolveSliceSpacing(geometry, sliceSpacing, targetSliceCount);
    const slices = extractStlSlices(
        geometry,
        baseSliceSpacing,
        contourTolerance,
        minLumenArea,
        minCompactness
    );
    const debugSegments = buildDebugSegments(slices);
    const taggingSliceSpacing = Math.max(
        baseSliceSpacing,
        resolveSliceSpacing(geometry, null, taggingTargetSliceCount)
    );
    const taggingAxisSlices = extractLumenCastTaggingSlices(
        geometry,
        taggingSliceSpacing,
        contourTolerance,
        minLumenArea,
        minCompactness,
        slices
    );
    const allTaggingSlices = taggingAxisSlices.flatMap(entry => entry.slices);
    const sliceLumenCastField = createMultiAxisLumenCastField(taggingAxisSlices);
    const baseDiagnostics = {
        sliceCount: slices.length,
        contourCount: slices.reduce((sum, slice) => sum + slice.contours.length, 0),
        debugSegmentCount: debugSegments.length / 6,
        requestedSliceCount: targetSliceCount,
        sliceSpacing: baseSliceSpacing,
        taggingRequestedSliceCount: taggingTargetSliceCount,
        taggingSliceSpacing,
        minCompactness,
        taggingAxes: taggingAxisSlices.map(entry => ({
            axis: entry.axisId,
            sliceCount: entry.slices.length,
            contourCount: entry.contourCount,
            taggedTriangleCount: entry.taggedTriangleCount
        }))
    };
    if (fieldOnly) {
        return {
            geometry: null,
            slices,
            axisSlices: taggingAxisSlices,
            debugSegments,
            field: sliceLumenCastField,
            diagnostics: {
                source: 'multi-axis-slice-field',
                ...baseDiagnostics
            }
        };
    }
    const taggedSurfaceCast = buildSliceTaggedInnerSurfaceGeometry(geometry, allTaggingSlices);
    const lumenSurfaceCast = taggedSurfaceCast || buildInnerSurfaceLumenCastGeometry(geometry, sliceLumenCastField);
    const volumeCast = lumenSurfaceCast ? null : buildVolumeLumenCastGeometry(geometry, sliceLumenCastField);
    const fallbackSurfaceCast = volumeCast || buildInnerSurfaceLumenCastGeometry(geometry, lumenField);
    const { geometry: castGeometry, diagnostics: castDiagnostics } = lumenSurfaceCast || fallbackSurfaceCast || buildLumenCastGeometry(slices);
    return {
        geometry: castGeometry,
        slices,
        axisSlices: taggingAxisSlices,
        debugSegments,
        field: sliceLumenCastField,
        diagnostics: {
            ...castDiagnostics,
            ...baseDiagnostics
        }
    };
}

function makeNodes(slices) {
    let id = 0;
    for (let sliceIndex = 0; sliceIndex < slices.length; sliceIndex++) {
        const slice = slices[sliceIndex];
        slice.nodes = slice.contours.map((contour, contourIndex) => ({
            id: id++,
            sliceIndex,
            rawSliceIndex: slice.rawIndex,
            contourIndex,
            point: new THREE.Vector3(contour.linkCenter.x, slice.y, contour.linkCenter.z),
            centerPoint: new THREE.Vector3(contour.center.x, slice.y, contour.center.z),
            radius: contour.radius,
            area: contour.area,
            bounds: contour.bounds,
            polygon: contour.polygon
        }));
    }
    return id;
}

function continuityCandidate(a, b, yGap) {
    const dx = a.point.x - b.point.x;
    const dz = a.point.z - b.point.z;
    const lateral = Math.hypot(dx, dz);
    const boundsGap = Math.sqrt(boundsDistanceSq(a.bounds, b.bounds));
    const radiusSum = a.radius + b.radius;
    const minRadius = Math.max(MIN_RADIUS, Math.min(a.radius, b.radius));
    const maxRadius = Math.max(a.radius, b.radius);
    const contains =
        pointInPolygonCoords(a.point.x, a.point.z, b.polygon) ||
        pointInPolygonCoords(b.point.x, b.point.z, a.polygon);
    const nearBounds = boundsGap <= Math.max(2.5, minRadius * 0.55 + yGap * 0.55);
    const lateralLimit = Math.max(9, radiusSum * 1.1 + yGap * 0.85, maxRadius * 1.35);
    const absoluteLateralLimit = Math.max(12, radiusSum * 1.05 + yGap * 0.6, maxRadius * 1.65);
    if (lateral > absoluteLateralLimit) return null;
    if (!contains && !nearBounds && lateral > lateralLimit) return null;

    const areaRatio = Math.max(a.area, b.area) / Math.max(1, Math.min(a.area, b.area));
    const cost =
        lateral / Math.max(1, radiusSum) +
        boundsGap / Math.max(1, minRadius * 2) +
        Math.abs(Math.log(areaRatio)) * 0.22 +
        yGap * 0.025;

    return { a, b, lateral, boundsGap, contains, nearBounds, minRadius, maxRadius, cost };
}

function addCenterlineEdge(edgeSet, segments, incoming, outgoing, candidate) {
    const key = `${candidate.a.id}->${candidate.b.id}`;
    if (edgeSet.has(key)) return false;
    edgeSet.add(key);

    const segment = {
        start: candidate.a.centerPoint.clone(),
        end: candidate.b.centerPoint.clone(),
        nodeStartId: candidate.a.id,
        nodeEndId: candidate.b.id,
        radiusStart: candidate.a.radius,
        radiusEnd: candidate.b.radius,
        source: 'stl-slice-centerline'
    };
    segments.push(segment);
    incoming.set(candidate.b.id, (incoming.get(candidate.b.id) || 0) + 1);
    outgoing.set(candidate.a.id, (outgoing.get(candidate.a.id) || 0) + 1);
    return true;
}

function hasBranchContinuity(candidate) {
    return candidate.contains || candidate.nearBounds || candidate.boundsGap <= Math.max(2, candidate.minRadius * 0.8);
}

function isSplitCandidate(candidate) {
    if (!hasBranchContinuity(candidate)) return false;
    const areaRatio = candidate.a.area / Math.max(1, candidate.b.area);
    if (areaRatio < 1.18) return false;
    const lateralLimit = candidate.contains || candidate.boundsGap <= Math.max(2, candidate.minRadius * 0.7)
        ? Math.max(candidate.a.radius + candidate.b.radius * 0.8, candidate.maxRadius * 1.85, 10)
        : Math.max(candidate.b.radius * 1.35, candidate.a.radius * 0.82, 8);
    return candidate.lateral <= lateralLimit;
}

function isMergeCandidate(candidate) {
    if (!hasBranchContinuity(candidate)) return false;
    const areaRatio = candidate.b.area / Math.max(1, candidate.a.area);
    if (areaRatio < 1.18) return false;
    const lateralLimit = candidate.contains || candidate.boundsGap <= Math.max(2, candidate.minRadius * 0.7)
        ? Math.max(candidate.a.radius * 0.8 + candidate.b.radius, candidate.maxRadius * 1.85, 10)
        : Math.max(candidate.a.radius * 1.35, candidate.b.radius * 0.82, 8);
    return candidate.lateral <= lateralLimit;
}

function connectSlicePair(sliceA, sliceB, state) {
    const yGap = Math.abs(sliceB.y - sliceA.y);
    const candidates = [];

    for (const a of sliceA.nodes) {
        for (const b of sliceB.nodes) {
            const candidate = continuityCandidate(a, b, yGap);
            if (!candidate) continue;
            candidates.push(candidate);
        }
    }
    candidates.sort((a, b) => a.cost - b.cost);

    const matchedSources = new Set();
    const matchedTargets = new Set();
    const localOutgoing = new Map();
    const localIncoming = new Map();

    const addCandidate = candidate => {
        const added = addCenterlineEdge(state.edgeSet, state.segments, state.incoming, state.outgoing, candidate);
        if (!added) return false;
        matchedSources.add(candidate.a.id);
        matchedTargets.add(candidate.b.id);
        localOutgoing.set(candidate.a.id, (localOutgoing.get(candidate.a.id) || 0) + 1);
        localIncoming.set(candidate.b.id, (localIncoming.get(candidate.b.id) || 0) + 1);
        return true;
    };

    let addedCount = 0;
    for (const candidate of candidates) {
        if (matchedSources.has(candidate.a.id) || matchedTargets.has(candidate.b.id)) continue;
        if (addCandidate(candidate)) addedCount++;
    }

    for (const candidate of candidates) {
        if (matchedTargets.has(candidate.b.id)) continue;
        if (!matchedSources.has(candidate.a.id)) continue;
        if ((localOutgoing.get(candidate.a.id) || 0) >= 2) continue;
        if (!isSplitCandidate(candidate)) continue;
        if (addCandidate(candidate)) addedCount++;
    }

    for (const candidate of candidates) {
        if (matchedSources.has(candidate.a.id)) continue;
        if (!matchedTargets.has(candidate.b.id)) continue;
        if ((localIncoming.get(candidate.b.id) || 0) >= 2) continue;
        if (!isMergeCandidate(candidate)) continue;
        if (addCandidate(candidate)) addedCount++;
    }
    return addedCount;
}

function makeStubSegment(node, spacing) {
    const half = Math.max(1, Math.min(spacing * 0.5, node.radius * 0.65));
    return {
        start: new THREE.Vector3(node.centerPoint.x, node.centerPoint.y - half, node.centerPoint.z),
        end: new THREE.Vector3(node.centerPoint.x, node.centerPoint.y + half, node.centerPoint.z),
        nodeStartId: node.id,
        nodeEndId: node.id,
        radiusStart: node.radius,
        radiusEnd: node.radius,
        source: 'stl-slice-stub'
    };
}

function buildCenterlineFromSlices(slices, spacing, maxLinkGap) {
    const nodeCount = makeNodes(slices);
    const state = {
        edgeSet: new Set(),
        incoming: new Map(),
        outgoing: new Map(),
        segments: []
    };

    for (let i = 0; i < slices.length - 1; i++) {
        for (let gap = 1; gap <= maxLinkGap && i + gap < slices.length; gap++) {
            const sliceA = slices[i];
            const sliceB = slices[i + gap];
            const rawGap = sliceB.rawIndex - sliceA.rawIndex;
            if (rawGap > maxLinkGap) break;
            if (connectSlicePair(sliceA, sliceB, state) > 0) break;
        }
    }

    let stubSegmentCount = 0;
    for (const slice of slices) {
        for (const node of slice.nodes) {
            if (state.incoming.has(node.id) || state.outgoing.has(node.id)) continue;
            state.segments.push(makeStubSegment(node, spacing));
            stubSegmentCount++;
        }
    }

    const incomingDegrees = [...state.incoming.values()];
    const outgoingDegrees = [...state.outgoing.values()];
    state.segments.diagnostics = {
        source: 'stl-slice-centerline',
        sliceCount: slices.length,
        nodeCount,
        edgeCount: state.edgeSet.size,
        stubSegmentCount,
        isolatedNodeCount: stubSegmentCount,
        uncoveredNodeCount: 0,
        splitNodeCount: outgoingDegrees.filter(value => value > 1).length,
        mergeNodeCount: incomingDegrees.filter(value => value > 1).length
    };
    return state.segments;
}

function buildDebugSegments(slices) {
    const edges = [];
    for (const slice of slices) {
        for (const contour of slice.contours) {
            for (let i = 0; i < contour.polygon.length; i++) {
                edges.push({
                    y: slice.y,
                    a: contour.polygon[i],
                    b: contour.polygon[(i + 1) % contour.polygon.length]
                });
            }
        }
    }

    const positions = [];
    const step = Math.max(1, Math.ceil(edges.length / DEBUG_MAX_CONTOUR_SEGMENTS));
    for (let i = 0; i < edges.length; i += step) {
        const edge = edges[i];
        positions.push(edge.a.x, edge.y, edge.a.z, edge.b.x, edge.y, edge.b.z);
    }
    return new Float32Array(positions);
}

function resolveSliceSpacing(geometry, sliceSpacing, targetSliceCount) {
    if (Number.isFinite(sliceSpacing) && sliceSpacing > 0) return sliceSpacing;
    geometry.computeBoundingBox();
    const height = geometry.boundingBox
        ? geometry.boundingBox.max.y - geometry.boundingBox.min.y
        : 0;
    if (!Number.isFinite(height) || height <= 0) return 1;
    const count = Math.max(2, Math.round(targetSliceCount || DEFAULT_TARGET_SLICE_COUNT));
    return height / count;
}

function centerlineAxisId(axis) {
    return typeof axis === 'string' ? axis : axis?.id;
}

function canonicalDirection(direction) {
    const vector = direction.clone().normalize();
    const values = [vector.x, vector.y, vector.z];
    const largestIndex = values
        .map(Math.abs)
        .reduce((best, value, index, array) => value > array[best] ? index : best, 0);
    if (values[largestIndex] < 0) vector.multiplyScalar(-1);
    return vector;
}

function centerlineAxisDescriptor(axis, index = 0) {
    if (axis?.basis) return axis;
    if (typeof axis === 'string') {
        if (axis === 'x') {
            return {
                id: 'x',
                basis: true,
                u: new THREE.Vector3(0, 1, 0),
                n: new THREE.Vector3(1, 0, 0),
                v: new THREE.Vector3(0, 0, 1),
                isIdentity: false,
                isCardinal: true
            };
        }
        if (axis === 'z') {
            return {
                id: 'z',
                basis: true,
                u: new THREE.Vector3(1, 0, 0),
                n: new THREE.Vector3(0, 0, 1),
                v: new THREE.Vector3(0, 1, 0),
                isIdentity: false,
                isCardinal: true
            };
        }
        return {
            id: 'y',
            basis: true,
            u: new THREE.Vector3(1, 0, 0),
            n: new THREE.Vector3(0, 1, 0),
            v: new THREE.Vector3(0, 0, 1),
            isIdentity: true,
            isCardinal: true
        };
    }

    const n = canonicalDirection(axis?.direction || new THREE.Vector3(0, 1, 0));
    const helper = Math.abs(n.y) < 0.88
        ? new THREE.Vector3(0, 1, 0)
        : new THREE.Vector3(1, 0, 0);
    const u = new THREE.Vector3().crossVectors(helper, n).normalize();
    const v = new THREE.Vector3().crossVectors(n, u).normalize();
    return {
        id: axis?.id || `dir${index}`,
        basis: true,
        u,
        n,
        v,
        isIdentity: false,
        isCardinal: false
    };
}

function axisOptionValue(option, axis) {
    if (!option || typeof option !== 'object') return undefined;
    return option[centerlineAxisId(axis)];
}

function resolveAxisSliceSpacing(
    axis,
    baseSliceSpacing,
    axisSliceSpacing,
    axisSliceSpacingMultipliers,
    secondaryAxisSpacingMultiplier
) {
    const explicitSpacing = axisOptionValue(axisSliceSpacing, axis);
    if (Number.isFinite(explicitSpacing) && explicitSpacing > 0) return explicitSpacing;

    const axisMultiplier = axisOptionValue(axisSliceSpacingMultipliers, axis);
    if (Number.isFinite(axisMultiplier) && axisMultiplier > 0) {
        return baseSliceSpacing * axisMultiplier;
    }

    const axisId = centerlineAxisId(axis);
    if ((axisId === 'x' || axisId === 'z') && Number.isFinite(secondaryAxisSpacingMultiplier) && secondaryAxisSpacingMultiplier > 0) {
        return baseSliceSpacing * secondaryAxisSpacingMultiplier;
    }

    return baseSliceSpacing;
}

function axisPointToLocal(point, axis) {
    const descriptor = centerlineAxisDescriptor(axis);
    return {
        x: point.x * descriptor.u.x + point.y * descriptor.u.y + point.z * descriptor.u.z,
        y: point.x * descriptor.n.x + point.y * descriptor.n.y + point.z * descriptor.n.z,
        z: point.x * descriptor.v.x + point.y * descriptor.v.y + point.z * descriptor.v.z
    };
}

function axisPointFromLocal(point, axis) {
    const descriptor = centerlineAxisDescriptor(axis);
    return new THREE.Vector3()
        .addScaledVector(descriptor.u, point.x)
        .addScaledVector(descriptor.n, point.y)
        .addScaledVector(descriptor.v, point.z);
}

function buildAxisGeometry(geometry, axis) {
    const descriptor = centerlineAxisDescriptor(axis);
    if (descriptor.isIdentity) return geometry;
    const position = geometry.attributes.position;
    const mapped = new Float32Array(position.count * 3);
    for (let i = 0; i < position.count; i++) {
        const local = axisPointToLocal({
            x: position.getX(i),
            y: position.getY(i),
            z: position.getZ(i)
        }, axis);
        mapped[i * 3] = local.x;
        mapped[i * 3 + 1] = local.y;
        mapped[i * 3 + 2] = local.z;
    }
    const axisGeometry = new THREE.BufferGeometry();
    axisGeometry.setAttribute('position', new THREE.BufferAttribute(mapped, 3));
    axisGeometry.computeBoundingBox();
    return axisGeometry;
}

function mapAxisSegment(segment, axis, nodeOffset) {
    const axisId = centerlineAxisId(axis);
    const sourcePrefix = segment.source === 'stl-slice-stub'
        ? 'stl-slice-stub'
        : 'stl-slice-centerline';
    return {
        ...segment,
        start: axisPointFromLocal(segment.start, axis),
        end: axisPointFromLocal(segment.end, axis),
        nodeStartId: `${axisId}:${nodeOffset + segment.nodeStartId}`,
        nodeEndId: `${axisId}:${nodeOffset + segment.nodeEndId}`,
        source: `${sourcePrefix}-${axisId}`
    };
}

function isObliqueAxisArtifact(segment, descriptor) {
    if (descriptor.id !== 'y') return false;
    if (segment.source?.startsWith('stl-slice-stub')) return false;
    const maxRadius = Math.max(segment.radiusStart || 0, segment.radiusEnd || 0);
    if (maxRadius < OBLIQUE_ARTIFACT_MIN_RADIUS) return false;
    const direction = new THREE.Vector3().subVectors(segment.end, segment.start);
    const length = direction.length();
    if (length < 1e-5) return false;
    direction.multiplyScalar(1 / length);
    const alignment = Math.abs(direction.dot(descriptor.n));
    return alignment < OBLIQUE_ARTIFACT_MIN_ALIGNMENT;
}

function pointSegmentDistance(point, segment) {
    const dx = segment.end.x - segment.start.x;
    const dy = segment.end.y - segment.start.y;
    const dz = segment.end.z - segment.start.z;
    const lengthSq = dx * dx + dy * dy + dz * dz;
    if (lengthSq < 1e-8) return point.distanceTo(segment.start);
    const relX = point.x - segment.start.x;
    const relY = point.y - segment.start.y;
    const relZ = point.z - segment.start.z;
    const t = THREE.MathUtils.clamp((relX * dx + relY * dy + relZ * dz) / lengthSq, 0, 1);
    const centerX = segment.start.x + dx * t;
    const centerY = segment.start.y + dy * t;
    const centerZ = segment.start.z + dz * t;
    const radius = segment.radiusStart * (1 - t) + segment.radiusEnd * t;
    return Math.hypot(point.x - centerX, point.y - centerY, point.z - centerZ) - radius;
}

function pointCenterlineSegmentDistance(point, segment) {
    const dx = segment.end.x - segment.start.x;
    const dy = segment.end.y - segment.start.y;
    const dz = segment.end.z - segment.start.z;
    const lengthSq = dx * dx + dy * dy + dz * dz;
    if (lengthSq < 1e-8) return point.distanceTo(segment.start);
    const relX = point.x - segment.start.x;
    const relY = point.y - segment.start.y;
    const relZ = point.z - segment.start.z;
    const t = THREE.MathUtils.clamp((relX * dx + relY * dy + relZ * dz) / lengthSq, 0, 1);
    return Math.hypot(
        point.x - (segment.start.x + dx * t),
        point.y - (segment.start.y + dy * t),
        point.z - (segment.start.z + dz * t)
    );
}

function pointSegmentAttachment(point, segment) {
    const dx = segment.end.x - segment.start.x;
    const dy = segment.end.y - segment.start.y;
    const dz = segment.end.z - segment.start.z;
    const lengthSq = dx * dx + dy * dy + dz * dz;
    if (lengthSq < 1e-8) {
        const radius = Math.max(segment.radiusStart, segment.radiusEnd);
        return {
            point: segment.start.clone(),
            nodeId: segment.nodeStartId,
            radius,
            distance: point.distanceTo(segment.start),
            signedDistance: point.distanceTo(segment.start) - radius,
            segment,
            t: 0
        };
    }
    const relX = point.x - segment.start.x;
    const relY = point.y - segment.start.y;
    const relZ = point.z - segment.start.z;
    const t = THREE.MathUtils.clamp((relX * dx + relY * dy + relZ * dz) / lengthSq, 0, 1);
    const center = new THREE.Vector3(
        segment.start.x + dx * t,
        segment.start.y + dy * t,
        segment.start.z + dz * t
    );
    const radius = segment.radiusStart * (1 - t) + segment.radiusEnd * t;
    if (t <= 0.04) {
        const endpointDistance = point.distanceTo(segment.start);
        return {
            point: segment.start.clone(),
            nodeId: segment.nodeStartId,
            radius: segment.radiusStart,
            distance: endpointDistance,
            signedDistance: endpointDistance - segment.radiusStart,
            segment,
            t: 0
        };
    }
    if (t >= 0.96) {
        const endpointDistance = point.distanceTo(segment.end);
        return {
            point: segment.end.clone(),
            nodeId: segment.nodeEndId,
            radius: segment.radiusEnd,
            distance: endpointDistance,
            signedDistance: endpointDistance - segment.radiusEnd,
            segment,
            t: 1
        };
    }
    const distance = point.distanceTo(center);
    return {
        point: center,
        nodeId: t < 0.5 ? segment.nodeStartId : segment.nodeEndId,
        radius,
        distance,
        signedDistance: distance - radius,
        segment,
        t
    };
}

function attachmentClearance(attachment, lumenField) {
    const clearance = branchRoutePointClearance(attachment.point, lumenField);
    return Number.isFinite(clearance) ? clearance : -Infinity;
}

function attachmentSelectionScore(attachment, rawBest, lumenField) {
    const clearance = attachmentClearance(attachment, lumenField);
    const centeredCredit = Math.max(
        0,
        Math.min(clearance, finiteRadius(attachment.radius) * 1.15)
    );
    const distancePenalty = rawBest
        ? Math.max(0, attachment.distance - rawBest.distance) * BRANCH_ATTACHMENT_DISTANCE_WEIGHT
        : 0;
    return attachment.signedDistance +
        distancePenalty -
        centeredCredit * BRANCH_ATTACHMENT_CENTERING_WEIGHT;
}

function attachmentConnectorValid(
    point,
    branchNodeId,
    branchRadius,
    attachment,
    lumenField,
    wallBvh,
    connectorLumenClearance
) {
    const origin = {
        start: attachment.point,
        end: point,
        nodeStartId: attachment.nodeId,
        nodeEndId: branchNodeId || 'branch',
        radiusStart: attachment.radius,
        radiusEnd: finiteRadius(branchRadius, attachment.radius),
        source: 'stl-slice-branch-origin'
    };
    return connectorStaysInsideVessel(origin, wallBvh, lumenField, connectorLumenClearance);
}

function nearestTreeAttachment(point, treeSegments, {
    lumenField = null,
    wallBvh = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    branchNodeId = null,
    branchRadius = null,
    maxSignedDistance = Infinity,
    requireConnector = false
} = {}) {
    let rawBest = null;
    const candidates = [];
    for (const segment of treeSegments) {
        const attachment = pointSegmentAttachment(point, segment);
        if (attachment.signedDistance > maxSignedDistance) continue;
        candidates.push(attachment);
        if (rawBest && attachment.signedDistance >= rawBest.signedDistance) continue;
        rawBest = attachment;
    }
    if (!rawBest || !lumenField?.query) return rawBest;

    if (
        requireConnector &&
        !attachmentConnectorValid(
            point,
            branchNodeId,
            branchRadius,
            rawBest,
            lumenField,
            wallBvh,
            connectorLumenClearance
        )
    ) {
        return rawBest;
    }

    const rawClearance = attachmentClearance(rawBest, lumenField);
    rawBest.clearance = rawClearance;
    rawBest.selectionScore = attachmentSelectionScore(rawBest, rawBest, lumenField);
    const lowClearanceThreshold = Math.max(
        0.45,
        Math.min(1.4, finiteRadius(rawBest.radius) * BRANCH_ATTACHMENT_LOW_CLEARANCE_FRACTION)
    );
    if (!Number.isFinite(rawClearance) || rawClearance >= lowClearanceThreshold) return rawBest;

    let best = rawBest;
    let bestScore = rawBest.selectionScore;
    const radius = finiteRadius(rawBest.radius);
    const slack = Math.max(0.75, Math.min(2.2, radius * 0.45));
    const requiredClearanceGain = Math.max(
        BRANCH_ATTACHMENT_MIN_CLEARANCE_GAIN,
        radius * 0.22
    );
    for (const attachment of candidates) {
        if (attachment === rawBest) continue;
        if (
            attachment.signedDistance > rawBest.signedDistance + slack &&
            attachment.distance > rawBest.distance + slack
        ) {
            continue;
        }
        if (
            requireConnector &&
            !attachmentConnectorValid(
                point,
                branchNodeId,
                branchRadius,
                attachment,
                lumenField,
                wallBvh,
                connectorLumenClearance
            )
        ) {
            continue;
        }

        const score = attachmentSelectionScore(attachment, rawBest, lumenField);
        const clearance = attachmentClearance(attachment, lumenField);
        if (clearance < rawClearance + requiredClearanceGain) continue;
        attachment.selectionScore = score;
        attachment.clearance = clearance;
        if (
            score < bestScore - BRANCH_ATTACHMENT_SCORE_EPSILON ||
            (
                clearance > best.clearance + requiredClearanceGain * 1.5 &&
                score <= bestScore + requiredClearanceGain * 0.35
            )
        ) {
            best = attachment;
            bestScore = score;
        }
    }
    return best;
}

function rankedTreeAttachments(point, treeSegments, lumenField, maxSignedDistance = Infinity, limit = 8) {
    const attachments = [];
    for (const segment of treeSegments) {
        const attachment = pointSegmentAttachment(point, segment);
        if (attachment.signedDistance > maxSignedDistance) continue;
        attachment.clearance = attachmentClearance(attachment, lumenField);
        attachments.push(attachment);
    }
    attachments.sort((a, b) =>
        a.signedDistance - b.signedDistance ||
        b.clearance - a.clearance
    );
    return attachments.slice(0, limit);
}

function attachBranchOriginToTree(origin, finalSegments, treeSegments, connectedNodes) {
    const attachment = origin.attachment;
    const sourceSegment = attachment?.segment;
    const t = attachment?.t;
    if (!sourceSegment || !Number.isFinite(t) || t <= 0.04 || t >= 0.96) return origin;

    const finalIndex = finalSegments.indexOf(sourceSegment);
    const treeIndex = treeSegments.indexOf(sourceSegment);
    if (finalIndex < 0 || treeIndex < 0) return origin;

    const attachmentNodeId = `attach:${sourceSegment.nodeStartId}:${sourceSegment.nodeEndId}:${connectedNodeKey(attachment.point)}`;
    const attachPoint = attachment.point.clone();
    const attachRadius = THREE.MathUtils.lerp(sourceSegment.radiusStart, sourceSegment.radiusEnd, t);
    const first = {
        ...sourceSegment,
        end: attachPoint.clone(),
        nodeEndId: attachmentNodeId,
        radiusEnd: attachRadius
    };
    const second = {
        ...sourceSegment,
        start: attachPoint.clone(),
        nodeStartId: attachmentNodeId,
        radiusStart: attachRadius
    };
    const replacements = [first, second].filter(segment => segment.start.distanceTo(segment.end) > 1e-4);
    finalSegments.splice(finalIndex, 1, ...replacements);
    treeSegments.splice(treeIndex, 1, ...replacements);
    for (const segment of replacements) markSegmentConnected(segment, connectedNodes);

    origin.start = attachPoint.clone();
    origin.nodeStartId = attachmentNodeId;
    origin.radiusStart = attachRadius;
    return origin;
}

function isCoveredByPrimarySegments(point, primarySegments) {
    for (const segment of primarySegments) {
        if (pointSegmentDistance(point, segment) <= TREE_COVER_MARGIN) return true;
    }
    return false;
}

function shouldKeepSecondarySegment(segment, primarySegments) {
    if (!primarySegments.length) return true;
    const startCovered = isCoveredByPrimarySegments(segment.start, primarySegments);
    const endCovered = isCoveredByPrimarySegments(segment.end, primarySegments);
    return !(startCovered && endCovered);
}

function connectedNodeKey(point) {
    return [
        Math.round(point.x * 4),
        Math.round(point.y * 4),
        Math.round(point.z * 4)
    ].join(',');
}

function markSegmentConnected(segment, connectedNodes) {
    connectedNodes.add(segment.nodeStartId);
    connectedNodes.add(segment.nodeEndId);
    connectedNodes.add(connectedNodeKey(segment.start));
    connectedNodes.add(connectedNodeKey(segment.end));
}

function addSegmentToTree(segment, treeSegments, connectedNodes) {
    treeSegments.push(segment);
    markSegmentConnected(segment, connectedNodes);
}

function segmentTouchesConnectedTree(segment, connectedNodes, treeSegments) {
    if (
        connectedNodes.has(segment.nodeStartId) ||
        connectedNodes.has(segment.nodeEndId) ||
        connectedNodes.has(connectedNodeKey(segment.start)) ||
        connectedNodes.has(connectedNodeKey(segment.end))
    ) {
        return true;
    }
    return (
        isCoveredByPrimarySegments(segment.start, treeSegments) ||
        isCoveredByPrimarySegments(segment.end, treeSegments)
    );
}

function segmentFullyCoveredByTree(segment, treeSegments) {
    return (
        isCoveredByPrimarySegments(segment.start, treeSegments) &&
        isCoveredByPrimarySegments(segment.end, treeSegments)
    );
}

function makeBranchOriginSegment(
    segment,
    treeSegments,
    lumenField,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    maxLength = DEFAULT_BRANCH_ORIGIN_MAX_LENGTH
) {
    const startAttachment = nearestTreeAttachment(segment.start, treeSegments, {
        lumenField,
        connectorLumenClearance,
        wallBvh,
        branchNodeId: segment.nodeStartId,
        branchRadius: segment.radiusStart,
        maxSignedDistance: BRANCH_ATTACH_MARGIN,
        requireConnector: true
    });
    const endAttachment = nearestTreeAttachment(segment.end, treeSegments, {
        lumenField,
        connectorLumenClearance,
        wallBvh,
        branchNodeId: segment.nodeEndId,
        branchRadius: segment.radiusEnd,
        maxSignedDistance: BRANCH_ATTACH_MARGIN,
        requireConnector: true
    });
    const useStart = startAttachment && (
        !endAttachment ||
        (startAttachment.selectionScore ?? startAttachment.signedDistance) <=
            (endAttachment.selectionScore ?? endAttachment.signedDistance)
    );
    const attachment = useStart ? startAttachment : endAttachment;
    if (!attachment || attachment.signedDistance > BRANCH_ATTACH_MARGIN) return null;
    const branchPoint = useStart ? segment.start : segment.end;
    const branchNodeId = useStart ? segment.nodeStartId : segment.nodeEndId;
    const branchRadius = useStart ? segment.radiusStart : segment.radiusEnd;
    if (attachment.point.distanceTo(branchPoint) < 1e-4) return null;
    if (Number.isFinite(maxLength) && maxLength > 0 && attachment.point.distanceTo(branchPoint) > maxLength) {
        return null;
    }
    const origin = {
        start: attachment.point.clone(),
        end: branchPoint.clone(),
        nodeStartId: attachment.nodeId,
        nodeEndId: branchNodeId,
        radiusStart: attachment.radius,
        radiusEnd: branchRadius,
        source: 'stl-slice-branch-origin',
        attachment
    };
    return connectorStaysInsideVessel(origin, wallBvh, lumenField, connectorLumenClearance) ? origin : null;
}

function componentEndpointNodes(component) {
    const nodes = new Map();
    const add = (id, point, radius) => {
        let node = nodes.get(id);
        if (!node) {
            node = {
                id,
                point,
                radius,
                degree: 0
            };
            nodes.set(id, node);
        }
        node.radius = Math.max(node.radius, radius);
        node.degree++;
    };
    for (const segment of component) {
        add(segment.nodeStartId, segment.start, segment.radiusStart);
        add(segment.nodeEndId, segment.end, segment.radiusEnd);
    }
    const endpoints = [...nodes.values()].filter(node => node.degree <= 1);
    return endpoints.length ? endpoints : [...nodes.values()];
}

function componentFullyCoveredByTree(component, treeSegments) {
    return component.every(segment => segmentFullyCoveredByTree(segment, treeSegments));
}

function nearestComponentOrigin(
    component,
    treeSegments,
    attachMargin = BRANCH_ATTACH_MARGIN,
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    maxLength = DEFAULT_BRANCH_ORIGIN_MAX_LENGTH
) {
    let best = null;
    for (const node of componentEndpointNodes(component)) {
        const attachment = nearestTreeAttachment(node.point, treeSegments, {
            lumenField,
            connectorLumenClearance,
            wallBvh,
            branchNodeId: node.id,
            branchRadius: node.radius,
            maxSignedDistance: attachMargin,
            requireConnector: true
        });
        if (!attachment) continue;
        if (best && attachment.signedDistance >= best.attachment.signedDistance) continue;
        best = { node, attachment };
    }
    if (!best || best.attachment.signedDistance > attachMargin) return null;
    if (
        Number.isFinite(maxLength) &&
        maxLength > 0 &&
        best.attachment.point.distanceTo(best.node.point) > maxLength
    ) {
        return null;
    }
    const origin = {
        start: best.attachment.point.clone(),
        end: best.node.point.clone(),
        nodeStartId: best.attachment.nodeId,
        nodeEndId: best.node.id,
        radiusStart: best.attachment.radius,
        radiusEnd: best.node.radius,
        source: 'stl-slice-branch-origin',
        attachment: best.attachment
    };
    return connectorStaysInsideVessel(origin, wallBvh, lumenField, connectorLumenClearance) ? origin : null;
}

function rootCenterlineComponents(
    segments,
    lumenField,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null
) {
    const components = segmentComponents(segments);
    if (components.length <= 1) {
        return {
            segments,
            addedBranchOriginCount: 0,
            discardedComponentCount: 0,
            discardedSegmentCount: 0
        };
    }

    const rootedSegments = components[0].slice();
    const treeSegments = rootedSegments.slice();
    const pending = components.slice(1);
    let addedBranchOriginCount = 0;
    let accepted = true;

    while (accepted && pending.length) {
        accepted = false;
        for (let i = 0; i < pending.length; i++) {
            const component = pending[i];
            const branchOrigin = nearestComponentOrigin(
                component,
                treeSegments,
                FINAL_BRANCH_ATTACH_MARGIN,
                lumenField,
                connectorLumenClearance,
                wallBvh
            );
            if (!branchOrigin) continue;

            attachBranchOriginToTree(branchOrigin, rootedSegments, treeSegments, new Set());
            rootedSegments.push(branchOrigin);
            treeSegments.push(branchOrigin);
            addedBranchOriginCount++;
            for (const segment of component) {
                rootedSegments.push(segment);
                treeSegments.push(segment);
            }
            pending.splice(i, 1);
            i--;
            accepted = true;
        }
    }

    return {
        segments: rootedSegments,
        addedBranchOriginCount,
        discardedComponentCount: pending.length,
        discardedSegmentCount: pending.reduce((sum, component) => sum + component.length, 0)
    };
}

function segmentHasGraphConnection(segment, connectedNodes) {
    return (
        connectedNodes.has(segment.nodeStartId) ||
        connectedNodes.has(segment.nodeEndId) ||
        connectedNodes.has(connectedNodeKey(segment.start)) ||
        connectedNodes.has(connectedNodeKey(segment.end))
    );
}

function segmentHasNodeConnection(segment, connectedNodes) {
    return (
        connectedNodes.has(segment.nodeStartId) ||
        connectedNodes.has(segment.nodeEndId)
    );
}

function growConnectedSecondarySegments(
    axisSegments,
    treeSegments,
    connectedNodes,
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null
) {
    const pending = axisSegments.slice();
    const kept = [];
    let accepted = true;
    while (accepted && pending.length) {
        accepted = false;
        for (let i = pending.length - 1; i >= 0; i--) {
            const segment = pending[i];
            if (segmentFullyCoveredByTree(segment, treeSegments)) {
                pending.splice(i, 1);
                continue;
            }
            if (!segmentTouchesConnectedTree(segment, connectedNodes, treeSegments)) continue;
            const branchOrigin = segmentHasGraphConnection(segment, connectedNodes)
                ? null
                : makeBranchOriginSegment(segment, treeSegments, lumenField, connectorLumenClearance, wallBvh);
            pending.splice(i, 1);
            if (branchOrigin) {
                kept.push(branchOrigin);
                addSegmentToTree(branchOrigin, treeSegments, connectedNodes);
            }
            kept.push(segment);
            addSegmentToTree(segment, treeSegments, connectedNodes);
            accepted = true;
        }
    }
    return kept;
}

function growConnectedComponents(
    components,
    treeSegments,
    connectedNodes,
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null
) {
    const pending = components.slice();
    const kept = [];
    let accepted = true;
    while (accepted && pending.length) {
        accepted = false;
        for (let i = 0; i < pending.length; i++) {
            const component = pending[i];
            if (componentFullyCoveredByTree(component, treeSegments)) {
                pending.splice(i, 1);
                i--;
                continue;
            }
            let touchesTree = false;
            for (const segment of component) {
                if (segmentTouchesConnectedTree(segment, connectedNodes, treeSegments)) {
                    touchesTree = true;
                    break;
                }
            }
            const hasNodeConnection = component.some(segment => segmentHasNodeConnection(segment, connectedNodes));
            const branchOrigin = hasNodeConnection
                ? null
                : nearestComponentOrigin(
                    component,
                    treeSegments,
                    BRANCH_ATTACH_MARGIN,
                    lumenField,
                    connectorLumenClearance,
                    wallBvh
                );
            if (!touchesTree && !branchOrigin) continue;
            if (!hasNodeConnection && !branchOrigin) continue;

            pending.splice(i, 1);
            i--;
            if (branchOrigin) {
                kept.push(branchOrigin);
                addSegmentToTree(branchOrigin, treeSegments, connectedNodes);
            }
            for (const segment of component) {
                if (segmentFullyCoveredByTree(segment, treeSegments)) continue;
                kept.push(segment);
                addSegmentToTree(segment, treeSegments, connectedNodes);
            }
            accepted = true;
        }
    }
    return kept;
}

function componentPathLength(component) {
    return component.reduce((sum, segment) => sum + segment.start.distanceTo(segment.end), 0);
}

function buildConnectedCenterlineTree(
    componentEntries,
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null
) {
    const entries = componentEntries
        .filter(entry => entry?.segments?.length)
        .map((entry, index) => ({
            ...entry,
            index,
            pathLength: componentPathLength(entry.segments),
            nonStubSegmentCount: entry.segments.filter(segment => !segment.source.startsWith('stl-slice-stub')).length,
            axisPriority: entry.axis === 'y'
                ? 3
                : entry.pass === 'adaptive'
                    ? 2
                    : 1
        }))
        .sort((a, b) =>
            b.axisPriority - a.axisPriority ||
            b.nonStubSegmentCount - a.nonStubSegmentCount ||
            b.pathLength - a.pathLength ||
            b.segments.length - a.segments.length
        );

    if (!entries.length) {
        return {
            segments: [],
            acceptedComponentCount: 0,
            coveredDuplicateComponentCount: 0,
            addedBranchOriginCount: 0,
            discardedComponentCount: 0,
            discardedSegmentCount: 0,
            candidateComponentCount: 0,
            discardedEntries: []
        };
    }

    const finalSegments = [];
    const treeSegments = [];
    const connectedNodes = new Set();
    const seed = entries[0];
    const pending = entries.slice(1);
    let acceptedComponentCount = 1;
    let coveredDuplicateComponentCount = 0;
    let addedBranchOriginCount = 0;

    const addFinalSegment = segment => {
        finalSegments.push(segment);
        addSegmentToTree(segment, treeSegments, connectedNodes);
    };

    const tryAddComponent = component => {
        if (componentFullyCoveredByTree(component, treeSegments)) {
            return { status: 'covered', branchOriginCount: 0 };
        }

        let touchesTree = false;
        let hasNodeConnection = false;
        for (const segment of component) {
            if (segmentTouchesConnectedTree(segment, connectedNodes, treeSegments)) touchesTree = true;
            if (segmentHasNodeConnection(segment, connectedNodes)) hasNodeConnection = true;
            if (touchesTree && hasNodeConnection) break;
        }

        const branchOrigin = hasNodeConnection
            ? null
            : nearestComponentOrigin(
                component,
                treeSegments,
                FINAL_BRANCH_ATTACH_MARGIN,
                lumenField,
                connectorLumenClearance,
                wallBvh
            );

        if (!touchesTree && !branchOrigin) return { status: 'pending', branchOriginCount: 0 };
        if (!hasNodeConnection && !branchOrigin) return { status: 'pending', branchOriginCount: 0 };

        let addedSegmentCount = 0;
        let branchOriginCount = 0;
        if (branchOrigin) {
            attachBranchOriginToTree(branchOrigin, finalSegments, treeSegments, connectedNodes);
            addFinalSegment(branchOrigin);
            branchOriginCount++;
        }
        for (const segment of component) {
            if (segmentFullyCoveredByTree(segment, treeSegments)) continue;
            addFinalSegment(segment);
            addedSegmentCount++;
        }

        return {
            status: addedSegmentCount || branchOriginCount ? 'accepted' : 'covered',
            branchOriginCount
        };
    };

    for (const segment of seed.segments) addFinalSegment(segment);

    let accepted = true;
    while (accepted && pending.length) {
        accepted = false;
        for (let i = 0; i < pending.length; i++) {
            const entry = pending[i];
            const component = entry.segments;
            if (componentFullyCoveredByTree(component, treeSegments)) {
                coveredDuplicateComponentCount++;
                pending.splice(i, 1);
                i--;
                continue;
            }

            const residualSegments = component.filter(segment => !segmentFullyCoveredByTree(segment, treeSegments));
            if (!residualSegments.length) {
                coveredDuplicateComponentCount++;
                pending.splice(i, 1);
                i--;
                accepted = true;
                continue;
            }

            const residualComponents = segmentComponents(residualSegments);
            const retained = [];
            let acceptedResidualCount = 0;
            let coveredResidualCount = 0;
            let branchOriginCount = 0;
            for (const residualComponent of residualComponents) {
                const result = tryAddComponent(residualComponent);
                if (result.status === 'accepted') {
                    acceptedResidualCount++;
                    branchOriginCount += result.branchOriginCount;
                } else if (result.status === 'covered') {
                    coveredResidualCount++;
                } else {
                    retained.push({
                        ...entry,
                        segments: residualComponent,
                        pathLength: componentPathLength(residualComponent),
                        nonStubSegmentCount: residualComponent.filter(segment =>
                            !segment.source.startsWith('stl-slice-stub')
                        ).length
                    });
                }
            }

            if (!acceptedResidualCount && !coveredResidualCount) continue;

            pending.splice(i, 1, ...retained);
            i--;
            if (acceptedResidualCount) {
                acceptedComponentCount += acceptedResidualCount;
                addedBranchOriginCount += branchOriginCount;
                accepted = true;
            }
            if (coveredResidualCount) {
                coveredDuplicateComponentCount += coveredResidualCount;
                accepted = true;
            }
        }
    }

    return {
        segments: finalSegments,
        acceptedComponentCount,
        coveredDuplicateComponentCount,
        addedBranchOriginCount,
        discardedComponentCount: pending.length,
        discardedSegmentCount: pending.reduce((sum, entry) => sum + entry.segments.length, 0),
        candidateComponentCount: entries.length,
        discardedEntries: pending
    };
}

function rescuePrimaryAxisComponents(
    finalSegments,
    discardedEntries,
    lumenField,
    connectorLumenClearance,
    wallBvh,
    lumenGeometry,
    maxComponents = 16
) {
    const candidates = (discardedEntries || [])
        .filter(entry =>
            entry.axis === 'y' &&
            entry.segments?.length &&
            entry.segments.some(segment => !segment.source.startsWith('stl-slice-stub'))
        )
        .sort((a, b) => b.pathLength - a.pathLength);
    if (!finalSegments.length || !candidates.length) {
        return {
            candidateComponentCount: candidates.length,
            attemptedComponentCount: 0,
            rescuedComponentCount: 0,
            failedComponentCount: candidates.length,
            addedOriginSegmentCount: 0,
            addedComponentSegmentCount: 0,
            choices: [],
            failures: []
        };
    }

    const treeSegments = finalSegments.slice();
    const connectedNodes = new Set();
    for (const segment of treeSegments) markSegmentConnected(segment, connectedNodes);
    let attemptedComponentCount = 0;
    let rescuedComponentCount = 0;
    let addedOriginSegmentCount = 0;
    let addedComponentSegmentCount = 0;
    const choices = [];
    const failures = [];

    const makeOrigin = (node, attachment, overrides = {}) => ({
        start: attachment.point.clone(),
        end: node.point.clone(),
        nodeStartId: attachment.nodeId,
        nodeEndId: node.id,
        radiusStart: attachment.radius,
        radiusEnd: node.radius,
        source: 'stl-slice-branch-origin',
        routeSegmentSource: 'stl-slice-branch-origin',
        attachment,
        validateRouteEdges: true,
        allowWallWhenInside: true,
        ...overrides
    });

    const sampledDirectRoute = origin => {
        const length = origin.start.distanceTo(origin.end);
        if (length < 1e-4) return null;
        const stepCount = Math.max(1, Math.ceil(length / DEFAULT_CENTERLINE_NODE_SPACING));
        const path = [];
        for (let index = 0; index <= stepCount; index++) {
            const point = origin.start.clone().lerp(origin.end, index / stepCount);
            const clearance = branchRoutePointClearance(point, lumenField);
            path.push(branchRouteNode(
                point,
                Number.isFinite(clearance) ? clearance : MIN_RADIUS,
                index,
                `primary-direct:${index}`
            ));
        }
        const directRoute = {
            path,
            routeType: 'direct-sampled',
            spacing: DEFAULT_CENTERLINE_NODE_SPACING
        };
        const directSegments = branchRouteToSegments(
            origin,
            directRoute.path,
            directRoute.routeType
        );
        if (segmentSetValidityFailureReason(
            directSegments,
            wallBvh,
            lumenField,
            connectorLumenClearance,
            { allowWallWhenInside: true }
        )) {
            return null;
        }

        const variants = [directRoute];
        const centered = centerBranchRoutePath(directRoute, lumenGeometry, { passes: 3 });
        if (centered) variants.push(centered);

        let best = null;
        for (const variant of variants) {
            const segments = branchRouteToSegments(origin, variant.path, variant.routeType);
            if (segmentSetValidityFailureReason(
                segments,
                wallBvh,
                lumenField,
                connectorLumenClearance,
                { allowWallWhenInside: true }
            )) {
                continue;
            }
            const routeScore = scoreBranchRoutePath(variant.path, lumenGeometry);
            const cost = branchRouteScoreCost(routeScore);
            if (!best || cost < best.cost) {
                best = {
                    segments,
                    path: variant.path,
                    routeType: variant.routeType,
                    routeScore,
                    cost
                };
            }
        }
        return best;
    };

    const endpointContinuationDirection = (node, segments) => {
        for (const segment of segments) {
            if (segment.nodeStartId === node.id) {
                return new THREE.Vector3()
                    .subVectors(segment.start, segment.end)
                    .normalize();
            }
            if (segment.nodeEndId === node.id) {
                return new THREE.Vector3()
                    .subVectors(segment.end, segment.start)
                    .normalize();
            }
        }
        return new THREE.Vector3();
    };

    for (const entry of candidates) {
        if (attemptedComponentCount >= maxComponents) break;
        if (componentFullyCoveredByTree(entry.segments, treeSegments)) continue;
        attemptedComponentCount++;
        const attachmentCandidates = [];
        for (const node of componentEndpointNodes(entry.segments)) {
            const continuation = endpointContinuationDirection(node, entry.segments);
            for (const attachment of rankedTreeAttachments(
                node.point,
                treeSegments,
                lumenField,
                120,
                48
            )) {
                const towardAttachment = new THREE.Vector3()
                    .subVectors(attachment.point, node.point);
                const distance = towardAttachment.length();
                const alignment = distance > 1e-5 && continuation.lengthSq() > 1e-8
                    ? continuation.dot(towardAttachment.multiplyScalar(1 / distance))
                    : -1;
                const directionPenalty = alignment >= 0
                    ? (1 - alignment) * 12
                    : 12 + Math.abs(alignment) * 28;
                attachmentCandidates.push({
                    node,
                    attachment,
                    alignment,
                    routePriority: distance + directionPenalty
                });
            }
        }
        attachmentCandidates.sort((a, b) =>
            a.routePriority - b.routePriority ||
            b.alignment - a.alignment ||
            a.attachment.distance - b.attachment.distance ||
            a.attachment.signedDistance - b.attachment.signedDistance ||
            b.attachment.clearance - a.attachment.clearance
        );
        let selected = null;
        const directRoutes = [];
        for (const { node, attachment } of attachmentCandidates) {
            const origin = makeOrigin(node, attachment);
            const route = sampledDirectRoute(origin);
            if (!route) continue;
            directRoutes.push({
                origin,
                route,
                span: origin.start.distanceTo(origin.end),
                cost: route.cost
            });
        }
        directRoutes.sort((a, b) => a.cost - b.cost || a.span - b.span);
        selected = directRoutes[0] || null;

        const limitedAttachments = attachmentCandidates.slice(0, 12);
        for (
            let attachmentIndex = 0;
            !selected && attachmentIndex < limitedAttachments.length;
            attachmentIndex++
        ) {
            const { node, attachment } = limitedAttachments[attachmentIndex];
            const span = attachment.point.distanceTo(node.point);
            if (span < 1e-4) continue;
            const vesselScale = Math.max(
                0.5,
                Math.min(1.6, Math.min(node.radius, attachment.radius) * 0.48)
            );
            const routeConfigurations = [{
                padding: Math.max(1.4, vesselScale * 2, Math.min(7, span * 0.18)),
                minimumPadding: Math.max(1, vesselScale * 1.5),
                spacing: vesselScale,
                maxCells: 7000,
                boundsPoints: null
            }];
            for (const configuration of routeConfigurations) {
                const origin = makeOrigin(node, attachment, {
                    routeBoundsPoints: configuration.boundsPoints,
                    routePadding: configuration.padding,
                    routeMinimumPadding: configuration.minimumPadding,
                    routeRadiusPaddingScale: 0.35,
                    routeRadiusPaddingBase: 1,
                    routeGridSpacing: configuration.spacing,
                    routeMaxCells: configuration.maxCells,
                    forceGridRoute: true,
                    allowLongDirectRoute: false
                });
                const route = bestValidatedRouteForOrigin(
                    origin,
                    lumenField,
                    wallBvh,
                    connectorLumenClearance,
                    lumenGeometry
                );
                if (!route?.segments?.length) continue;
                if (segmentSetValidityFailureReason(
                    route.segments,
                    wallBvh,
                    lumenField,
                    connectorLumenClearance,
                    { allowWallWhenInside: true }
                )) {
                    continue;
                }
                selected = { origin, route, span };
                break;
            }
            if (selected) break;
        }
        if (!selected) {
            if (failures.length < 8) {
                const bounds = entry.segments.reduce((box, segment) => (
                    box.expandByPoint(segment.start).expandByPoint(segment.end)
                ), new THREE.Box3());
                failures.push({
                    pathLength: entry.pathLength,
                    segmentCount: entry.segments.length,
                    endpointCount: componentEndpointNodes(entry.segments).length,
                    attachmentCandidateCount: attachmentCandidates.length,
                    bounds,
                    rankedAttachments: attachmentCandidates.slice(0, 12).map(candidate => ({
                        endpoint: {
                            x: candidate.node.point.x,
                            y: candidate.node.point.y,
                            z: candidate.node.point.z
                        },
                        attachment: {
                            x: candidate.attachment.point.x,
                            y: candidate.attachment.point.y,
                            z: candidate.attachment.point.z
                        },
                        distance: candidate.attachment.distance,
                        signedDistance: candidate.attachment.signedDistance,
                        clearance: candidate.attachment.clearance,
                        alignment: candidate.alignment,
                        routePriority: candidate.routePriority
                    }))
                });
            }
            continue;
        }

        attachBranchOriginToTree(
            selected.origin,
            finalSegments,
            treeSegments,
            connectedNodes
        );
        const firstRouteSegment = selected.route.segments[0];
        firstRouteSegment.start = selected.origin.start.clone();
        firstRouteSegment.nodeStartId = selected.origin.nodeStartId;
        firstRouteSegment.radiusStart = selected.origin.radiusStart;
        for (const segment of selected.route.segments) {
            finalSegments.push(segment);
            addSegmentToTree(segment, treeSegments, connectedNodes);
        }
        for (const segment of entry.segments) {
            finalSegments.push(segment);
            addSegmentToTree(segment, treeSegments, connectedNodes);
        }
        rescuedComponentCount++;
        addedOriginSegmentCount += selected.route.segments.length;
        addedComponentSegmentCount += entry.segments.length;
        if (choices.length < 12) {
            choices.push({
                pathLength: entry.pathLength,
                span: selected.span,
                componentSegmentCount: entry.segments.length,
                routeSegmentCount: selected.route.segments.length,
                routeType: selected.route.routeType || null
            });
        }
    }

    return {
        candidateComponentCount: candidates.length,
        attemptedComponentCount,
        rescuedComponentCount,
        failedComponentCount: candidates.length - rescuedComponentCount,
        addedOriginSegmentCount,
        addedComponentSegmentCount,
        choices,
        failures
    };
}

function collectAdaptiveCenterlineAxes(segments, axes, maxCount = DEFAULT_MAX_ADAPTIVE_DIRECTIONS) {
    if (!Number.isFinite(maxCount) || maxCount <= 0) return [];
    const existingDirections = axes.map((axis, index) => centerlineAxisDescriptor(axis, index).n);
    const selected = [];
    const candidates = segments
        .map(segment => {
            const direction = new THREE.Vector3().subVectors(segment.end, segment.start);
            const length = direction.length();
            if (length < DEFAULT_CENTERLINE_NODE_SPACING * 1.5) return null;
            return {
                direction: canonicalDirection(direction.multiplyScalar(1 / length)),
                length
            };
        })
        .filter(Boolean)
        .sort((a, b) => b.length - a.length);
    const minDot = Math.cos(THREE.MathUtils.degToRad(15));

    for (const candidate of candidates) {
        if (existingDirections.some(direction => Math.abs(direction.dot(candidate.direction)) >= minDot)) continue;
        if (selected.some(axis => Math.abs(axis.direction.dot(candidate.direction)) >= minDot)) continue;
        selected.push(directionAxis(
            `local${selected.length + 1}`,
            candidate.direction.x,
            candidate.direction.y,
            candidate.direction.z
        ));
        if (selected.length >= maxCount) break;
    }
    return selected;
}

function segmentComponents(segments) {
    const byNode = new Map();
    for (let index = 0; index < segments.length; index++) {
        const segment = segments[index];
        for (const nodeKey of [centerlineEndpointKey(segment, 'start'), centerlineEndpointKey(segment, 'end')]) {
            let bucket = byNode.get(nodeKey);
            if (!bucket) {
                bucket = [];
                byNode.set(nodeKey, bucket);
            }
            bucket.push(index);
        }
    }

    const seen = new Set();
    const components = [];
    for (let index = 0; index < segments.length; index++) {
        if (seen.has(index)) continue;
        const stack = [index];
        const componentSegments = [];
        seen.add(index);
        while (stack.length) {
            const currentIndex = stack.pop();
            const segment = segments[currentIndex];
            componentSegments.push(segment);
            for (const nodeKey of [centerlineEndpointKey(segment, 'start'), centerlineEndpointKey(segment, 'end')]) {
                for (const neighbourIndex of byNode.get(nodeKey) || []) {
                    if (seen.has(neighbourIndex)) continue;
                    seen.add(neighbourIndex);
                    stack.push(neighbourIndex);
                }
            }
        }
        components.push(componentSegments);
    }
    components.sort((a, b) => b.length - a.length);
    return components;
}

function resampleCenterlineSegments(segments, spacing = DEFAULT_CENTERLINE_NODE_SPACING) {
    if (!Number.isFinite(spacing) || spacing <= 0) {
        return {
            segments,
            insertedNodeCount: 0,
            originalSegmentCount: segments.length
        };
    }

    const resampled = [];
    let insertedNodeCount = 0;
    for (let segmentIndex = 0; segmentIndex < segments.length; segmentIndex++) {
        const segment = segments[segmentIndex];
        const length = segment.start.distanceTo(segment.end);
        const stepCount = Math.max(1, Math.ceil(length / spacing));
        if (stepCount <= 1) {
            resampled.push(segment);
            continue;
        }

        const resampleKey = `${segment.nodeStartId}:${segment.nodeEndId}:${segmentIndex}`;
        for (let stepIndex = 0; stepIndex < stepCount; stepIndex++) {
            const t0 = stepIndex / stepCount;
            const t1 = (stepIndex + 1) / stepCount;
            resampled.push({
                ...segment,
                start: segment.start.clone().lerp(segment.end, t0),
                end: segment.start.clone().lerp(segment.end, t1),
                nodeStartId: stepIndex === 0
                    ? segment.nodeStartId
                    : `${resampleKey}:r${stepIndex}`,
                nodeEndId: stepIndex === stepCount - 1
                    ? segment.nodeEndId
                    : `${resampleKey}:r${stepIndex + 1}`,
                radiusStart: THREE.MathUtils.lerp(segment.radiusStart, segment.radiusEnd, t0),
                radiusEnd: THREE.MathUtils.lerp(segment.radiusStart, segment.radiusEnd, t1)
            });
        }
        insertedNodeCount += stepCount - 1;
    }

    return {
        segments: resampled,
        insertedNodeCount,
        originalSegmentCount: segments.length
    };
}

function centerlineEndpointKey(segment, endpoint) {
    const point = endpoint === 'start' ? segment.start : segment.end;
    const nodeId = endpoint === 'start' ? segment.nodeStartId : segment.nodeEndId;
    if (nodeId !== undefined && nodeId !== null && segment.nodeStartId !== segment.nodeEndId) {
        return `node:${nodeId}:${connectedNodeKey(point)}`;
    }
    return `point:${connectedNodeKey(point)}`;
}

function collectRefinementNodes(segments) {
    const nodes = new Map();
    const ensureNode = (key, point, radius, source) => {
        let node = nodes.get(key);
        if (!node) {
            node = {
                key,
                point: new THREE.Vector3(),
                radius: 0,
                weight: 0,
                directions: [],
                sources: new Set()
            };
            nodes.set(key, node);
        }
        node.point.add(point);
        node.radius = Math.max(node.radius, radius);
        node.weight++;
        if (source) node.sources.add(source);
        return node;
    };

    for (const segment of segments) {
        const startKey = centerlineEndpointKey(segment, 'start');
        const endKey = centerlineEndpointKey(segment, 'end');
        const diagnosticSource = segment.branchOriginRouted
            ? `${segment.source}:${segment.branchOriginRouteType || 'routed'}`
            : segment.source;
        ensureNode(startKey, segment.start, segment.radiusStart, diagnosticSource);
        ensureNode(endKey, segment.end, segment.radiusEnd, diagnosticSource);
    }

    for (const node of nodes.values()) {
        node.point.multiplyScalar(1 / Math.max(1, node.weight));
    }

    for (const segment of segments) {
        const direction = new THREE.Vector3().subVectors(segment.end, segment.start);
        const length = direction.length();
        if (length < 1e-5) continue;
        direction.multiplyScalar(1 / length);
        const startNode = nodes.get(centerlineEndpointKey(segment, 'start'));
        const endNode = nodes.get(centerlineEndpointKey(segment, 'end'));
        if (startNode) startNode.directions.push(direction.clone());
        if (endNode) endNode.directions.push(direction.clone().multiplyScalar(-1));
    }

    return nodes;
}

function nodeTangent(node) {
    if (!node.directions.length) return null;
    const sum = node.directions[0].clone();
    for (let i = 1; i < node.directions.length; i++) {
        const direction = node.directions[i].clone();
        if (direction.dot(sum) < 0) direction.multiplyScalar(-1);
        sum.add(direction);
    }
    if (sum.lengthSq() < 1e-8) return node.directions[0].clone().normalize();
    return sum.normalize();
}

function transverseBasis(tangent) {
    const helper = Math.abs(tangent.y) < 0.86
        ? new THREE.Vector3(0, 1, 0)
        : new THREE.Vector3(1, 0, 0);
    const u = new THREE.Vector3().crossVectors(helper, tangent).normalize();
    const v = new THREE.Vector3().crossVectors(tangent, u).normalize();
    return { u, v };
}

function raycastDistanceFrom(point, direction, bvh, maxDistance) {
    const ray = new THREE.Ray(point, direction);
    const hit = bvh.raycastFirst(ray, THREE.DoubleSide, 1e-4, maxDistance);
    return Number.isFinite(hit?.distance) ? hit.distance : null;
}

function sphereDirectionPairs(sampleCount = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES) {
    const count = Math.max(8, Math.round(sampleCount));
    if (SPHERE_DIRECTION_CACHE.has(count)) return SPHERE_DIRECTION_CACHE.get(count);

    const directions = [];
    const goldenAngle = Math.PI * (3 - Math.sqrt(5));
    for (let i = 0; i < count; i++) {
        const t = (i + 0.5) / count;
        const z = t;
        const radius = Math.sqrt(Math.max(0, 1 - z * z));
        const angle = i * goldenAngle;
        directions.push(new THREE.Vector3(
            Math.cos(angle) * radius,
            Math.sin(angle) * radius,
            z
        ).normalize());
    }
    SPHERE_DIRECTION_CACHE.set(count, directions);
    return directions;
}

function centerShiftForDirections(point, directions, lumenBvh, {
    maxRayDistance = 32,
    minDiameter = 1.2,
    maxDiameterMultiplier = 1.7,
    minDiameterMultiplier = 0.42
} = {}) {
    const pairs = [];
    for (const direction of directions) {
        const positive = raycastDistanceFrom(point, direction, lumenBvh, maxRayDistance);
        if (!Number.isFinite(positive)) continue;
        const negativeDirection = direction.clone().multiplyScalar(-1);
        const negative = raycastDistanceFrom(point, negativeDirection, lumenBvh, maxRayDistance);
        if (!Number.isFinite(negative)) continue;
        const diameter = positive + negative;
        if (diameter < minDiameter) continue;
        pairs.push({
            direction,
            diameter,
            shiftDistance: (positive - negative) * 0.5
        });
    }

    const shift = new THREE.Vector3();
    if (!pairs.length) {
        return {
            shift,
            pairCount: 0,
            meanRadius: 0
        };
    }

    const sortedDiameters = pairs.map(pair => pair.diameter).sort((a, b) => a - b);
    const medianDiameter = sortedDiameters[Math.floor(sortedDiameters.length * 0.5)];
    const maxDiameter = Math.max(minDiameter, medianDiameter * maxDiameterMultiplier);
    const minRobustDiameter = Math.max(minDiameter, medianDiameter * minDiameterMultiplier);
    let pairCount = 0;
    let radiusSum = 0;
    for (const pair of pairs) {
        if (pair.diameter > maxDiameter || pair.diameter < minRobustDiameter) continue;
        shift.addScaledVector(pair.direction, pair.shiftDistance);
        radiusSum += pair.diameter * 0.5;
        pairCount++;
    }
    if (!pairCount) {
        return {
            shift,
            pairCount: 0,
            meanRadius: 0
        };
    }
    shift.multiplyScalar(1 / pairCount);
    return {
        shift,
        pairCount,
        meanRadius: radiusSum / pairCount
    };
}

function representativeNodeDirections(node) {
    if (!node.directions.length) return [];
    const directions = [];
    const duplicateDot = Math.cos(THREE.MathUtils.degToRad(10));
    for (const source of node.directions) {
        const direction = source.clone();
        if (direction.lengthSq() < 1e-8) continue;
        direction.normalize();
        if (directions.some(existing => Math.abs(existing.dot(direction)) >= duplicateDot)) continue;
        directions.push(direction);
    }
    return directions;
}

function centerShiftForPlane(point, u, v, lumenBvh, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    maxRayDistance = 32,
    minDiameter = 1.2
} = {}) {
    const sampleCount = Math.max(8, Math.round(radialSamples));
    const pairs = [];
    for (let i = 0; i < sampleCount; i++) {
        const angle = 2 * Math.PI * i / sampleCount;
        const direction = u.clone()
            .multiplyScalar(Math.cos(angle))
            .addScaledVector(v, Math.sin(angle))
            .normalize();
        const positive = raycastDistanceFrom(point, direction, lumenBvh, maxRayDistance);
        if (!Number.isFinite(positive)) continue;
        const negativeDirection = direction.clone().multiplyScalar(-1);
        const negative = raycastDistanceFrom(point, negativeDirection, lumenBvh, maxRayDistance);
        if (!Number.isFinite(negative)) continue;
        const diameter = positive + negative;
        if (diameter < minDiameter) continue;
        pairs.push({
            direction,
            diameter,
            shiftDistance: (positive - negative) * 0.5
        });
    }

    const shift = new THREE.Vector3();
    if (!pairs.length) {
        return {
            shift,
            pairCount: 0,
            meanRadius: 0
        };
    }

    const sortedDiameters = pairs.map(pair => pair.diameter).sort((a, b) => a - b);
    const medianDiameter = sortedDiameters[Math.floor(sortedDiameters.length * 0.5)];
    const maxDiameter = Math.max(minDiameter, medianDiameter * 1.8);
    const minRobustDiameter = Math.max(minDiameter, medianDiameter * 0.45);
    let pairCount = 0;
    let radiusSum = 0;
    for (const pair of pairs) {
        if (pair.diameter > maxDiameter || pair.diameter < minRobustDiameter) continue;
        shift.addScaledVector(pair.direction, pair.shiftDistance);
        radiusSum += pair.diameter * 0.5;
        pairCount++;
    }
    if (!pairCount) {
        return {
            shift,
            pairCount: 0,
            meanRadius: 0
        };
    }
    shift.multiplyScalar(1 / pairCount);
    return {
        shift,
        pairCount,
        meanRadius: radiusSum / pairCount
    };
}

function centerShiftForSphere(point, lumenBvh, {
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
    maxRayDistance = 32,
    minDiameter = 1.2
} = {}) {
    if (!Number.isFinite(sphereSamples) || sphereSamples <= 0) {
        return {
            shift: new THREE.Vector3(),
            pairCount: 0,
            meanRadius: 0
        };
    }
    return centerShiftForDirections(
        point,
        sphereDirectionPairs(sphereSamples),
        lumenBvh,
        {
            maxRayDistance,
            minDiameter,
            maxDiameterMultiplier: 1.55,
            minDiameterMultiplier: 0.48
        }
    );
}

function nodeCenterShift(node, point, lumenBvh, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES
} = {}) {
    const regularTangent = node.directions.length <= 2 ? nodeTangent(node) : null;
    const tangents = regularTangent ? [regularTangent] : representativeNodeDirections(node);
    const maxRayDistance = Math.max(8, Math.min(80, node.radius * 3.8 + 8));
    const minDiameter = Math.max(1.2, node.radius * 0.45);
    if (!tangents.length) {
        const sphereOnly = centerShiftForSphere(point, lumenBvh, {
            sphereSamples,
            maxRayDistance,
            minDiameter
        });
        return {
            ...sphereOnly,
            source: 'sphere'
        };
    }

    const shift = new THREE.Vector3();
    let pairCount = 0;
    let radiusSum = 0;
    for (const tangent of tangents) {
        const { u, v } = transverseBasis(tangent);
        const planeShift = centerShiftForPlane(point, u, v, lumenBvh, {
            radialSamples,
            maxRayDistance,
            minDiameter
        });
        if (planeShift.pairCount < Math.max(3, Math.floor(Math.max(8, radialSamples) * 0.22))) continue;
        shift.addScaledVector(planeShift.shift, planeShift.pairCount);
        radiusSum += planeShift.meanRadius * planeShift.pairCount;
        pairCount += planeShift.pairCount;
    }

    if (!pairCount) {
        const sphereOnly = centerShiftForSphere(point, lumenBvh, {
            sphereSamples,
            maxRayDistance,
            minDiameter
        });
        return {
            ...sphereOnly,
            source: 'sphere'
        };
    }
    shift.multiplyScalar(1 / pairCount);

    const planeShiftLength = shift.length();
    const shouldCrossCheckWithSphere = (
        sphereSamples > 0 &&
        (
            node.directions.length !== 2 ||
            planeShiftLength > Math.max(0.35, node.radius * 0.2)
        )
    );
    if (shouldCrossCheckWithSphere) {
        const sphere = centerShiftForSphere(point, lumenBvh, {
            sphereSamples,
            maxRayDistance,
            minDiameter
        });
        if (sphere.pairCount >= Math.max(4, Math.floor(Math.max(8, sphereSamples) * 0.35))) {
            const planeWeight = pairCount;
            const sphereWeight = sphere.pairCount * 0.85;
            shift.multiplyScalar(planeWeight)
                .addScaledVector(sphere.shift, sphereWeight)
                .multiplyScalar(1 / (planeWeight + sphereWeight));
            radiusSum += sphere.meanRadius * sphereWeight;
            pairCount += sphereWeight;
        }
    }

    return {
        shift,
        pairCount,
        meanRadius: radiusSum / pairCount
    };
}

function refineCenterlineNodeToLumen(node, lumenBvh, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
    iterations = DEFAULT_CENTERLINE_REFINE_ITERATIONS
} = {}) {
    if (!node.directions.length) return { point: node.point, refined: false, reason: 'missing-tangent' };
    const point = node.point.clone();
    let lastPairCount = 0;
    const iterationCount = Math.max(1, Math.round(iterations));
    const branchNode = node.directions.length > 2;
    const maxStep = Math.max(0.75, Math.min(4.5, node.radius * (branchNode ? 0.38 : 0.55)));
    let totalShift = 0;

    for (let iteration = 0; iteration < iterationCount; iteration++) {
        const measurement = nodeCenterShift(node, point, lumenBvh, { radialSamples, sphereSamples });
        if (measurement.pairCount < Math.max(3, Math.floor(Math.max(8, radialSamples) * 0.22))) break;
        const shift = measurement.shift;
        const shiftLength = shift.length();
        if (shiftLength > maxStep) shift.multiplyScalar(maxStep / shiftLength);
        point.add(shift);
        totalShift += shift.length();
        lastPairCount = measurement.pairCount;
        if (shift.length() < 0.035) break;
    }

    return {
        point,
        refined: totalShift > 1e-5,
        shift: point.distanceTo(node.point),
        pairCount: lastPairCount
    };
}

function isRoutedBranchOnlyNode(node) {
    const sources = [...(node.sources || [])];
    return sources.length > 0 && sources.every(source =>
        typeof source === 'string' && source.startsWith('stl-slice-branch-origin:')
    );
}

function refineCenterlineSegmentsToLumen(segments, lumenGeometry, options = {}) {
    if (!segments.length || !lumenGeometry?.attributes?.position) {
        return {
            refinedNodeCount: 0,
            skippedNodeCount: 0,
            failedNodeCount: 0,
            wallRejectedSegmentCount: 0,
            wallClampedSegmentCount: 0,
            averageShift: 0,
            maxShift: 0
        };
    }

    const lumenBvh = lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry));
    const nodes = collectRefinementNodes(segments);
    const originalSegments = segments.map(segment => ({
        segment,
        start: segment.start.clone(),
        end: segment.end.clone(),
        startKey: centerlineEndpointKey(segment, 'start'),
        endKey: centerlineEndpointKey(segment, 'end')
    }));
    let refinedNodeCount = 0;
    let skippedNodeCount = 0;
    let failedNodeCount = 0;
    let totalShift = 0;
    let maxShift = 0;
    for (const node of nodes.values()) {
        if (options.skipRoutedBranchNodes && isRoutedBranchOnlyNode(node)) {
            node.refinedPoint = node.point.clone();
            skippedNodeCount++;
            continue;
        }
        const refinement = refineCenterlineNodeToLumen(node, lumenBvh, options);
        node.refinedPoint = refinement.point.clone ? refinement.point.clone() : node.point.clone();
        if (refinement.refined) {
            refinedNodeCount++;
            totalShift += refinement.shift || 0;
            maxShift = Math.max(maxShift, refinement.shift || 0);
        } else if (refinement.reason === 'branch') {
            skippedNodeCount++;
        } else {
            failedNodeCount++;
        }
    }

    for (const segment of segments) {
        const startNode = nodes.get(centerlineEndpointKey(segment, 'start'));
        const endNode = nodes.get(centerlineEndpointKey(segment, 'end'));
        if (startNode?.refinedPoint) segment.start = startNode.refinedPoint.clone();
        if (endNode?.refinedPoint) segment.end = endNode.refinedPoint.clone();
    }

    let wallRejectedSegmentCount = 0;
    let wallClampedSegmentCount = 0;
    if (options.wallBvh) {
        const endpointOverrides = new Map();
        const addEndpointOverride = (key, point) => {
            let override = endpointOverrides.get(key);
            if (!override) {
                override = {
                    point: new THREE.Vector3(),
                    count: 0
                };
                endpointOverrides.set(key, override);
            }
            override.point.add(point);
            override.count++;
        };

        for (const original of originalSegments) {
            if (!segmentIntersectsWall(original.segment, options.wallBvh)) continue;
            const refinedStart = original.segment.start.clone();
            const refinedEnd = original.segment.end.clone();
            let bestStart = original.start.clone();
            let bestEnd = original.end.clone();
            let low = 0;
            let high = 1;
            const candidate = {
                ...original.segment,
                start: new THREE.Vector3(),
                end: new THREE.Vector3()
            };
            for (let i = 0; i < 10; i++) {
                const t = (low + high) * 0.5;
                candidate.start.copy(original.start).lerp(refinedStart, t);
                candidate.end.copy(original.end).lerp(refinedEnd, t);
                if (segmentIntersectsWall(candidate, options.wallBvh)) {
                    high = t;
                } else {
                    low = t;
                    bestStart.copy(candidate.start);
                    bestEnd.copy(candidate.end);
                }
            }
            addEndpointOverride(original.startKey, bestStart);
            addEndpointOverride(original.endKey, bestEnd);
            if (low > 1e-4) wallClampedSegmentCount++;
            else wallRejectedSegmentCount++;
        }
        for (const override of endpointOverrides.values()) {
            override.point.multiplyScalar(1 / Math.max(1, override.count));
        }
        for (const original of originalSegments) {
            const startOverride = endpointOverrides.get(original.startKey);
            const endOverride = endpointOverrides.get(original.endKey);
            if (startOverride) original.segment.start = startOverride.point.clone();
            if (endOverride) original.segment.end = endOverride.point.clone();
        }
    }

    return {
        refinedNodeCount,
        skippedNodeCount,
        failedNodeCount,
        averageShift: refinedNodeCount ? totalShift / refinedNodeCount : 0,
        maxShift,
        wallRejectedSegmentCount,
        wallClampedSegmentCount
    };
}

function collectNodeIncidence(segments) {
    const incidence = new Map();
    const add = (key, segment, endpoint) => {
        let bucket = incidence.get(key);
        if (!bucket) {
            bucket = [];
            incidence.set(key, bucket);
        }
        bucket.push({ segment, endpoint });
    };
    for (const segment of segments) {
        add(centerlineEndpointKey(segment, 'start'), segment, 'start');
        add(centerlineEndpointKey(segment, 'end'), segment, 'end');
    }
    return incidence;
}

function collectSegmentEndpointKeyMap(incidence) {
    const endpointKeys = new Map();
    for (const [key, incident] of incidence.entries()) {
        for (const { segment, endpoint } of incident) {
            let entry = endpointKeys.get(segment);
            if (!entry) {
                entry = {};
                endpointKeys.set(segment, entry);
            }
            entry[endpoint] = key;
        }
    }
    return endpointKeys;
}

function currentNodePointFromIncidence(key, incidence) {
    const incident = incidence.get(key);
    if (!incident?.length) return null;
    const point = new THREE.Vector3();
    let count = 0;
    for (const { segment, endpoint } of incident) {
        point.add(endpoint === 'start' ? segment.start : segment.end);
        count++;
    }
    return count ? point.multiplyScalar(1 / count) : null;
}

function nodeCandidateSegmentsValidityFailureReason(
    key,
    candidate,
    incidence,
    lumenField,
    connectorLumenClearance,
    wallBvh,
    options = {}
) {
    const incident = incidence.get(key);
    if (!incident?.length) return 'missing-incidence';
    for (const { segment, endpoint } of incident) {
        const candidateSegment = {
            ...segment,
            start: endpoint === 'start' ? candidate : segment.start,
            end: endpoint === 'end' ? candidate : segment.end
        };
        if (candidateSegment.start.distanceTo(candidateSegment.end) < 1e-4) continue;
        const reason = segmentValidityFailureReason(
            candidateSegment,
            wallBvh,
            lumenField,
            connectorLumenClearance,
            options
        );
        if (reason) return reason;
    }
    return null;
}

function nodeCandidateSegmentsStayValid(
    key,
    candidate,
    incidence,
    lumenField,
    connectorLumenClearance,
    wallBvh,
    options = {}
) {
    return !nodeCandidateSegmentsValidityFailureReason(
        key,
        candidate,
        incidence,
        lumenField,
        connectorLumenClearance,
        wallBvh,
        options
    );
}

function nodePatchSegmentsValidityFailureReason(
    updates,
    incidence,
    endpointKeys,
    lumenField,
    connectorLumenClearance,
    wallBvh,
    options = {}
) {
    const affectedSegments = new Set();
    for (const key of updates.keys()) {
        for (const { segment } of incidence.get(key) || []) affectedSegments.add(segment);
    }
    for (const segment of affectedSegments) {
        const keys = endpointKeys.get(segment);
        if (!keys) return 'missing-keys';
        const start = updates.get(keys.start) || segment.start;
        const end = updates.get(keys.end) || segment.end;
        const candidateSegment = {
            ...segment,
            start,
            end
        };
        if (candidateSegment.start.distanceTo(candidateSegment.end) < 1e-4) continue;
        const reason = segmentValidityFailureReason(
            candidateSegment,
            wallBvh,
            lumenField,
            connectorLumenClearance,
            options
        );
        if (reason) return reason;
    }
    return null;
}

function nodePatchSegmentsStayValid(
    updates,
    incidence,
    endpointKeys,
    lumenField,
    connectorLumenClearance,
    wallBvh,
    options = {}
) {
    return !nodePatchSegmentsValidityFailureReason(
        updates,
        incidence,
        endpointKeys,
        lumenField,
        connectorLumenClearance,
        wallBvh,
        options
    );
}

function applyNodeCandidate(key, candidate, incidence) {
    for (const { segment, endpoint } of incidence.get(key) || []) {
        if (endpoint === 'start') segment.start = candidate.clone();
        else segment.end = candidate.clone();
    }
}

function applyNodePatch(updates, incidence) {
    for (const [key, point] of updates.entries()) {
        applyNodeCandidate(key, point, incidence);
    }
}

function neighbourKeysForNode(key, incidence, endpointKeys) {
    const keys = [];
    for (const { segment, endpoint } of incidence.get(key) || []) {
        const segmentKeys = endpointKeys.get(segment);
        if (!segmentKeys) continue;
        const neighbourKey = endpoint === 'start' ? segmentKeys.end : segmentKeys.start;
        if (neighbourKey && neighbourKey !== key && !keys.includes(neighbourKey)) keys.push(neighbourKey);
    }
    return keys;
}

function buildOutlierPatchUpdates(
    key,
    currentPoint,
    measuredShift,
    incidence,
    endpointKeys,
    scale,
    neighbourScale,
    cachedNeighbourKeys = null,
    cachedNeighbourPoints = null
) {
    if (measuredShift.lengthSq() < 1e-8) return null;
    const updates = new Map();
    updates.set(key, currentPoint.clone().addScaledVector(measuredShift, scale));
    const neighbourKeys = cachedNeighbourKeys || neighbourKeysForNode(key, incidence, endpointKeys);
    for (let index = 0; index < neighbourKeys.length; index++) {
        const neighbourKey = neighbourKeys[index];
        const neighbourPoint = cachedNeighbourPoints?.[index] || currentNodePointFromIncidence(neighbourKey, incidence);
        if (!neighbourPoint) continue;
        updates.set(neighbourKey, neighbourPoint.clone().addScaledVector(measuredShift, scale * neighbourScale));
    }
    return updates.size > 1 ? updates : null;
}

function nodeNeighbourMidpoint(key, incidence) {
    const incident = incidence.get(key);
    if (!incident?.length) return null;
    const midpoint = new THREE.Vector3();
    let count = 0;
    for (const { segment, endpoint } of incident) {
        midpoint.add(endpoint === 'start' ? segment.end : segment.start);
        count++;
    }
    return count ? midpoint.multiplyScalar(1 / count) : null;
}

function transverseClearanceDirections(node, sampleCount = CENTERLINE_CLEARANCE_RELAX_DIRECTIONS) {
    const tangent = node.directions.length <= 2 ? nodeTangent(node) : null;
    if (!tangent) return branchRouteRelaxationDirections();
    const { u, v } = transverseBasis(tangent);
    const count = Math.max(8, Math.round(sampleCount));
    const directions = [];
    for (let i = 0; i < count; i++) {
        const angle = 2 * Math.PI * i / count;
        directions.push(u.clone()
            .multiplyScalar(Math.cos(angle))
            .addScaledVector(v, Math.sin(angle))
            .normalize());
    }
    return directions;
}

function relaxCenterlineNodesByLumenClearance(segments, {
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    passCount = DEFAULT_CENTERLINE_CLEARANCE_RELAX_PASSES,
    directionCount = CENTERLINE_CLEARANCE_RELAX_DIRECTIONS
} = {}) {
    if (!segments.length || !lumenField?.query || !Number.isFinite(passCount) || passCount <= 0) {
        return {
            adjustedNodeCount: 0,
            passCount: 0,
            averageShift: 0,
            maxShift: 0,
            averageClearanceGain: 0,
            maxClearanceGain: 0,
            rejectedNodeCount: 0
        };
    }

    let adjustedNodeCount = 0;
    let rejectedNodeCount = 0;
    let totalShift = 0;
    let maxShift = 0;
    let totalClearanceGain = 0;
    let maxClearanceGain = 0;
    let completedPassCount = 0;
    const scratchPoint = new THREE.Vector3();

    for (let pass = 0; pass < Math.max(1, Math.round(passCount)); pass++) {
        const incidence = collectNodeIncidence(segments);
        const nodes = collectRefinementNodes(segments);
        let movedThisPass = 0;
        const orderedNodes = [...nodes.values()]
            .map(node => ({
                node,
                clearance: branchRoutePointClearance(node.point, lumenField)
            }))
            .filter(item => Number.isFinite(item.clearance))
            .sort((a, b) => a.clearance - b.clearance);

        for (const { node, clearance: baseClearance } of orderedNodes) {
            if (!node.directions.length) continue;
            const currentPoint = currentNodePointFromIncidence(node.key, incidence);
            if (!currentPoint) continue;
            const currentClearance = branchRoutePointClearance(currentPoint, lumenField);
            if (!Number.isFinite(currentClearance)) continue;

            const neighbourMidpoint = nodeNeighbourMidpoint(node.key, incidence);
            const smoothWeight = node.directions.length === 2 && neighbourMidpoint ? 0.018 : 0;
            const scoreFor = (point, clearance) => {
                const smoothPenalty = smoothWeight ? point.distanceTo(neighbourMidpoint) * smoothWeight : 0;
                return clearance - smoothPenalty;
            };
            let bestPoint = null;
            let bestClearance = currentClearance;
            let bestScore = scoreFor(currentPoint, currentClearance);
            const radius = Math.max(0.5, node.radius || currentClearance || 1);
            const maxStep = Math.max(0.4, Math.min(2.4, radius * 0.48));
            const steps = [
                maxStep,
                maxStep * 0.58,
                maxStep * 0.32,
                maxStep * 0.16
            ];

            for (const direction of transverseClearanceDirections(node, directionCount)) {
                for (const step of steps) {
                    scratchPoint.copy(currentPoint).addScaledVector(direction, step);
                    const clearance = branchRoutePointClearance(scratchPoint, lumenField);
                    if (!Number.isFinite(clearance)) continue;
                    if (clearance <= currentClearance + CENTERLINE_CLEARANCE_RELAX_MIN_GAIN) continue;
                    const score = scoreFor(scratchPoint, clearance);
                    if (score <= bestScore + CENTERLINE_CLEARANCE_RELAX_MIN_GAIN * 0.55) continue;
                    if (!nodeCandidateSegmentsStayValid(
                        node.key,
                        scratchPoint,
                        incidence,
                        lumenField,
                        connectorLumenClearance,
                        wallBvh
                    )) {
                        continue;
                    }
                    bestPoint = scratchPoint.clone();
                    bestClearance = clearance;
                    bestScore = score;
                }
            }

            if (!bestPoint) {
                rejectedNodeCount++;
                continue;
            }

            const shift = bestPoint.distanceTo(currentPoint);
            const clearanceGain = bestClearance - currentClearance;
            applyNodeCandidate(node.key, bestPoint, incidence);
            adjustedNodeCount++;
            movedThisPass++;
            totalShift += shift;
            maxShift = Math.max(maxShift, shift);
            totalClearanceGain += clearanceGain;
            maxClearanceGain = Math.max(maxClearanceGain, clearanceGain);
        }

        completedPassCount++;
        if (!movedThisPass) break;
    }

    return {
        adjustedNodeCount,
        passCount: completedPassCount,
        averageShift: adjustedNodeCount ? totalShift / adjustedNodeCount : 0,
        maxShift,
        averageClearanceGain: adjustedNodeCount ? totalClearanceGain / adjustedNodeCount : 0,
        maxClearanceGain,
        rejectedNodeCount
    };
}

function relaxMeasuredOutlierNodesByClearance(segments, centering, {
    lumenField = null,
    referenceField = null,
    referenceFieldOnly = false,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    enablePatches = false
} = {}) {
    const candidates = (centering?.outlierCorrectionCandidates || []).filter(candidate => (
        candidate.normalizedOffset >= CENTERLINE_OUTLIER_RELAX_MIN_NORMALIZED ||
        (
            candidate.offset >= CENTERLINE_OUTLIER_RELAX_MIN_OFFSET &&
            candidate.normalizedOffset >= 0.35
        )
    ));
    if (!segments.length || !candidates.length || !lumenField?.query) {
        return {
            adjustedNodeCount: 0,
            rejectedNodeCount: 0,
            averageShift: 0,
            maxShift: 0,
            averageClearanceGain: 0,
            maxClearanceGain: 0,
            directCenteredNodeCount: 0,
            directWallFallbackNodeCount: 0,
            referenceFieldCenteredNodeCount: 0,
            patchedNodeCount: 0,
            rejectedCandidates: []
        };
    }

    const incidence = collectNodeIncidence(segments);
    const endpointKeys = collectSegmentEndpointKeyMap(incidence);
    const baseDirections = branchRouteRelaxationDirections();
    const seenKeys = new Set();
    let adjustedNodeCount = 0;
    let rejectedNodeCount = 0;
    let totalShift = 0;
    let maxShift = 0;
    let totalClearanceGain = 0;
    let maxClearanceGain = 0;
    let directCenteredNodeCount = 0;
    let directWallFallbackNodeCount = 0;
    let referenceFieldCenteredNodeCount = 0;
    let patchedNodeCount = 0;
    const rejectedCandidates = [];
    const directScaleSteps = [1, 0.78, 0.58, 0.4, 0.25, 0.14, 0.08, 0.04];
    const referenceScaleSteps = [1, 0.82, 0.64, 0.46, 0.3, 0.18, 0.1];

    for (const candidate of candidates) {
        if (!candidate?.key || seenKeys.has(candidate.key)) continue;
        seenKeys.add(candidate.key);
        const currentPoint = currentNodePointFromIncidence(candidate.key, incidence);
        if (!currentPoint) continue;
        const baseClearance = branchRoutePointClearance(currentPoint, lumenField);
        if (!Number.isFinite(baseClearance)) continue;
        const shiftDirection = new THREE.Vector3(
            candidate.shift?.x || 0,
            candidate.shift?.y || 0,
            candidate.shift?.z || 0
        );
        const radius = Math.max(0.5, candidate.meanRadius || candidate.radius || 1);
        const step = Math.max(0.35, Math.min(1.6, radius * 0.38));
        const directions = shiftDirection.lengthSq() > 1e-8
            ? [shiftDirection.normalize(), ...baseDirections]
            : baseDirections;
        let bestPoint = null;
        let bestClearance = baseClearance;
        let usedDirectCentering = false;
        let usedDirectWallFallback = false;
        let usedReferenceFieldCentering = false;
        let referenceRejection = null;
        const measuredRejections = [];
        const noteMeasuredRejection = (mode, scale, reason) => {
            if (measuredRejections.length >= 16) return;
            measuredRejections.push({ mode, scale, reason });
        };

        const measuredShift = new THREE.Vector3(
            candidate.shift?.x || 0,
            candidate.shift?.y || 0,
            candidate.shift?.z || 0
        );
        const referenceTarget = candidate.degree <= 2
            ? referenceFieldMedialTarget(currentPoint, referenceField)
            : null;
        if (referenceTarget?.point) {
            const referenceShift = new THREE.Vector3().subVectors(referenceTarget.point, currentPoint);
            const referenceShiftLength = referenceShift.length();
            const targetClearance = branchRoutePointClearance(referenceTarget.point, lumenField);
            const needsReferenceCentering = (
                referenceTarget.signedDistance < 0.12 ||
                referenceTarget.lowerSignedDistance < -0.45 ||
                referenceTarget.upperSignedDistance < -0.45
            );
            referenceRejection = {
                reason: 'not-tested',
                shift: referenceShiftLength,
                targetClearance,
                referenceSignedDistance: referenceTarget.signedDistance,
                lowerSignedDistance: referenceTarget.lowerSignedDistance,
                upperSignedDistance: referenceTarget.upperSignedDistance
            };
            if (
                needsReferenceCentering &&
                referenceShiftLength >= 0.18 &&
                referenceShiftLength <= Math.max(7.5, radius * 2.2) &&
                Number.isFinite(targetClearance) &&
                targetClearance >= connectorLumenClearance + 0.08
            ) {
                referenceRejection.reason = 'validity';
                for (const scale of referenceScaleSteps) {
                    const point = currentPoint.clone().addScaledVector(referenceShift, scale);
                    const clearance = branchRoutePointClearance(point, lumenField);
                    if (!Number.isFinite(clearance) || clearance < connectorLumenClearance + 0.02) continue;
                    let valid = nodeCandidateSegmentsStayValid(
                        candidate.key,
                        point,
                        incidence,
                        lumenField,
                        connectorLumenClearance,
                        wallBvh
                    );
                    let wallFallback = false;
                    if (!valid && scale <= 0.46) {
                        const fallbackMargin = connectorLumenClearance + 0.1;
                        valid = clearance > fallbackMargin + 0.08 && nodeCandidateSegmentsStayValid(
                            candidate.key,
                            point,
                            incidence,
                            lumenField,
                            fallbackMargin,
                            null
                        );
                        wallFallback = valid;
                    }
                    if (!valid) continue;
                    bestPoint = point;
                    bestClearance = clearance;
                    usedDirectCentering = true;
                    usedDirectWallFallback = wallFallback;
                    usedReferenceFieldCentering = true;
                    referenceRejection = null;
                    break;
                }
            } else if (!needsReferenceCentering) {
                referenceRejection.reason = 'not-needed';
            } else if (referenceShiftLength < 0.18 || referenceShiftLength > Math.max(7.5, radius * 2.2)) {
                referenceRejection.reason = 'shift-limit';
            } else if (!Number.isFinite(targetClearance) || targetClearance < connectorLumenClearance + 0.08) {
                referenceRejection.reason = 'target-clearance';
            }
        } else if (referenceFieldOnly) {
            referenceRejection = { reason: 'missing-target' };
        }
        if (!referenceFieldOnly && measuredShift.lengthSq() > 1e-8) {
            for (const scale of directScaleSteps) {
                if (bestPoint) break;
                const point = currentPoint.clone().addScaledVector(measuredShift, scale);
                let failureReason = nodeCandidateSegmentsValidityFailureReason(
                    candidate.key,
                    point,
                    incidence,
                    lumenField,
                    connectorLumenClearance,
                    wallBvh
                );
                let valid = !failureReason;
                let wallFallback = false;
                if (!valid && scale <= 0.58) {
                    const clearance = branchRoutePointClearance(point, lumenField);
                    const fallbackMargin = scale <= 0.25
                        ? connectorLumenClearance + 0.08
                        : connectorLumenClearance + 0.18;
                    failureReason = clearance > fallbackMargin + 0.08
                        ? nodeCandidateSegmentsValidityFailureReason(
                        candidate.key,
                        point,
                        incidence,
                        lumenField,
                        fallbackMargin,
                        null
                    )
                        : 'point-clearance';
                    valid = !failureReason;
                    wallFallback = valid;
                }
                if (!valid) {
                    noteMeasuredRejection('measured', scale, failureReason || 'invalid');
                    continue;
                }
                bestPoint = point;
                bestClearance = branchRoutePointClearance(point, lumenField);
                usedDirectCentering = true;
                usedDirectWallFallback = wallFallback;
                break;
            }
        }

        let bestPatch = null;
        let bestPatchPoint = null;
        let bestPatchClearance = baseClearance;
        if (!referenceFieldOnly && enablePatches && !bestPoint && candidate.degree === 2 && measuredShift.lengthSq() > 1e-8) {
            for (const scale of [0.78, 0.58, 0.4, 0.25]) {
                for (const neighbourScale of [0.42, 0.28, 0.16]) {
                    const updates = buildOutlierPatchUpdates(
                        candidate.key,
                        currentPoint,
                        measuredShift,
                        incidence,
                        endpointKeys,
                        scale,
                        neighbourScale
                    );
                    if (!updates) continue;
                    if (!nodePatchSegmentsStayValid(
                        updates,
                        incidence,
                        endpointKeys,
                        lumenField,
                        connectorLumenClearance,
                        wallBvh
                    )) {
                        continue;
                    }
                    const candidatePoint = updates.get(candidate.key);
                    const candidateClearance = branchRoutePointClearance(candidatePoint, lumenField);
                    if (!Number.isFinite(candidateClearance)) continue;
                    const score = (
                        scale * 1.8 +
                        THREE.MathUtils.clamp(candidateClearance - baseClearance, -0.5, 1.5) * 0.12 -
                        neighbourScale * 0.08
                    );
                    const bestScore = bestPatch
                        ? bestPatch.scale * 1.8 +
                            THREE.MathUtils.clamp(bestPatchClearance - baseClearance, -0.5, 1.5) * 0.12 -
                            bestPatch.neighbourScale * 0.08
                        : -Infinity;
                    if (score <= bestScore) continue;
                    bestPatch = {
                        updates,
                        scale,
                        neighbourScale
                    };
                    bestPatchPoint = candidatePoint;
                    bestPatchClearance = candidateClearance;
                }
            }
        }

        if (!referenceFieldOnly && !bestPoint) {
            for (const direction of directions) {
                for (const scale of [1, 0.65, 0.4, 0.22]) {
                    const point = currentPoint.clone().addScaledVector(direction, step * scale);
                    const clearance = branchRoutePointClearance(point, lumenField);
                    if (clearance <= bestClearance + BRANCH_ROUTE_RELAX_MIN_GAIN) continue;
                    let failureReason = nodeCandidateSegmentsValidityFailureReason(
                        candidate.key,
                        point,
                        incidence,
                        lumenField,
                        connectorLumenClearance,
                        wallBvh
                    );
                    let valid = !failureReason;
                    let wallFallback = false;
                    if (!valid && scale <= 0.4) {
                        const fallbackMargin = connectorLumenClearance + 0.12;
                        failureReason = clearance > fallbackMargin + 0.08
                            ? nodeCandidateSegmentsValidityFailureReason(
                            candidate.key,
                            point,
                            incidence,
                            lumenField,
                            fallbackMargin,
                            null
                        )
                            : 'point-clearance';
                        valid = !failureReason;
                        wallFallback = valid;
                    }
                    if (!valid) {
                        noteMeasuredRejection('clearance', scale, failureReason || 'invalid');
                        continue;
                    }
                    bestPoint = point;
                    bestClearance = clearance;
                    usedDirectCentering = false;
                    usedDirectWallFallback = wallFallback;
                }
            }
        }

        if (!bestPoint && !bestPatch) {
            if (rejectedCandidates.length < 12) {
                rejectedCandidates.push({
                    key: candidate.key,
                    offset: candidate.offset || 0,
                    normalizedOffset: candidate.normalizedOffset || 0,
                    meanRadius: candidate.meanRadius || 0,
                    sources: candidate.sources || [],
                    point: candidate.point || null,
                    shift: candidate.shift || null,
                    baseClearance,
                    referenceRejection,
                    measuredRejections
                });
            }
            rejectedNodeCount++;
            continue;
        }
        const appliedShift = (bestPoint || bestPatchPoint).distanceTo(currentPoint);
        const clearanceGain = (bestPoint ? bestClearance : bestPatchClearance) - baseClearance;
        if (bestPoint) {
            applyNodeCandidate(candidate.key, bestPoint, incidence);
        } else {
            applyNodePatch(bestPatch.updates, incidence);
            patchedNodeCount++;
        }
        adjustedNodeCount++;
        totalShift += appliedShift;
        maxShift = Math.max(maxShift, appliedShift);
        totalClearanceGain += clearanceGain;
        maxClearanceGain = Math.max(maxClearanceGain, clearanceGain);
        if (usedDirectCentering || bestPatch) directCenteredNodeCount++;
        if (usedDirectWallFallback) directWallFallbackNodeCount++;
        if (usedReferenceFieldCentering) referenceFieldCenteredNodeCount++;
    }

    return {
        adjustedNodeCount,
        rejectedNodeCount,
        averageShift: adjustedNodeCount ? totalShift / adjustedNodeCount : 0,
        maxShift,
        averageClearanceGain: adjustedNodeCount ? totalClearanceGain / adjustedNodeCount : 0,
        maxClearanceGain,
        directCenteredNodeCount,
        directWallFallbackNodeCount,
        referenceFieldCenteredNodeCount,
        patchedNodeCount,
        rejectedCandidates
    };
}

function measureCenterlineCentering(segments, lumenGeometry, options = {}) {
    if (!segments.length || !lumenGeometry?.attributes?.position) {
        return {
            measuredNodeCount: 0,
            failedNodeCount: 0,
            averageOffset: 0,
            maxOffset: 0,
            averageNormalizedOffset: 0,
            maxNormalizedOffset: 0
        };
    }

    const lumenBvh = lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry));
    const nodes = collectRefinementNodes(segments);
    let measuredNodeCount = 0;
    let failedNodeCount = 0;
    let offsetSum = 0;
    let maxOffset = 0;
    let normalizedOffsetSum = 0;
    let maxNormalizedOffset = 0;
    const worstOffsets = [];
    const worstNormalizedOffsets = [];
    const outlierCorrectionCandidates = [];
    const makeWorstEntry = (key, node, measurement, offset, normalizedOffset) => ({
        key,
        offset,
        normalizedOffset,
        degree: node.directions.length,
        radius: node.radius,
        meanRadius: measurement.meanRadius || 0,
        sources: [...(node.sources || [])].slice(0, 6),
        point: {
            x: node.point.x,
            y: node.point.y,
            z: node.point.z
        },
        shift: {
            x: measurement.shift.x,
            y: measurement.shift.y,
            z: measurement.shift.z
        }
    });
    const recordWorstOffset = (key, node, measurement, offset, normalizedOffset) => {
        const entry = makeWorstEntry(key, node, measurement, offset, normalizedOffset);
        worstOffsets.push(entry);
        worstOffsets.sort((a, b) => b.offset - a.offset);
        if (worstOffsets.length > 12) worstOffsets.pop();
        worstNormalizedOffsets.push(entry);
        worstNormalizedOffsets.sort((a, b) => b.normalizedOffset - a.normalizedOffset);
        if (worstNormalizedOffsets.length > 12) worstNormalizedOffsets.pop();
        const shouldTryCorrection = (
            node.directions.length <= 2 &&
            (
                normalizedOffset >= Math.min(
                    CENTERLINE_OUTLIER_RELAX_MIN_NORMALIZED,
                    CENTERLINE_OUTLIER_CHAIN_PATCH_MIN_NORMALIZED
                ) ||
                (offset >= CENTERLINE_OUTLIER_RELAX_MIN_OFFSET && normalizedOffset >= 0.35)
            )
        );
        if (shouldTryCorrection) {
            outlierCorrectionCandidates.push({
                ...entry,
                score: normalizedOffset * 3 + Math.min(offset, 6) * 0.35
            });
            outlierCorrectionCandidates.sort((a, b) => b.score - a.score);
            if (outlierCorrectionCandidates.length > CENTERLINE_OUTLIER_RELAX_MAX_CANDIDATES) {
                outlierCorrectionCandidates.pop();
            }
        }
    };
    for (const [key, node] of nodes.entries()) {
        const measurement = nodeCenterShift(node, node.point, lumenBvh, options);
        if (!measurement.pairCount) {
            failedNodeCount++;
            continue;
        }
        const offset = measurement.shift.length();
        const radius = Math.max(0.5, measurement.meanRadius || node.radius || 1);
        const normalizedOffset = offset / radius;
        measuredNodeCount++;
        offsetSum += offset;
        maxOffset = Math.max(maxOffset, offset);
        normalizedOffsetSum += normalizedOffset;
        maxNormalizedOffset = Math.max(maxNormalizedOffset, normalizedOffset);
        recordWorstOffset(key, node, measurement, offset, normalizedOffset);
    }

    return {
        measuredNodeCount,
        failedNodeCount,
        averageOffset: measuredNodeCount ? offsetSum / measuredNodeCount : 0,
        maxOffset,
        averageNormalizedOffset: measuredNodeCount ? normalizedOffsetSum / measuredNodeCount : 0,
        maxNormalizedOffset,
        worstOffsets,
        worstNormalizedOffsets,
        outlierCorrectionCandidates
    };
}

function pruneOffCenterLeafSegments(segments, lumenGeometry, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    sphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
    maxPasses = DEFAULT_CENTERLINE_LEAF_PRUNE_PASSES,
    absoluteOffset = 4.5,
    normalizedOffset = 0.55
} = {}) {
    if (!segments.length || !lumenGeometry?.attributes?.position) {
        return {
            prunedSegmentCount: 0,
            prunedPassCount: 0
        };
    }

    const lumenBvh = lumenGeometry.boundsTree || (lumenGeometry.boundsTree = new MeshBVH(lumenGeometry));
    let prunedSegmentCount = 0;
    let prunedPassCount = 0;
    for (let pass = 0; pass < maxPasses; pass++) {
        const nodes = collectRefinementNodes(segments);
        const endpointDegree = new Map();
        const addEndpointDegree = key => {
            endpointDegree.set(key, (endpointDegree.get(key) || 0) + 1);
        };
        for (const segment of segments) {
            addEndpointDegree(centerlineEndpointKey(segment, 'start'));
            addEndpointDegree(centerlineEndpointKey(segment, 'end'));
        }
        const leafKeysToPrune = new Set();
        for (const [key, node] of nodes.entries()) {
            if (node.directions.length !== 1) continue;
            const measurement = nodeCenterShift(node, node.point, lumenBvh, { radialSamples, sphereSamples });
            if (!measurement.pairCount) continue;
            const offset = measurement.shift.length();
            const radius = Math.max(0.5, measurement.meanRadius || node.radius || 1);
            const normalized = offset / radius;
            if (offset >= absoluteOffset && normalized >= normalizedOffset) {
                leafKeysToPrune.add(key);
            }
        }
        if (!leafKeysToPrune.size) break;

        let removedThisPass = 0;
        for (let i = segments.length - 1; i >= 0; i--) {
            const segment = segments[i];
            const startKey = centerlineEndpointKey(segment, 'start');
            const endKey = centerlineEndpointKey(segment, 'end');
            const startIsGraphLeaf = (endpointDegree.get(startKey) || 0) <= 1;
            const endIsGraphLeaf = (endpointDegree.get(endKey) || 0) <= 1;
            if (
                !(leafKeysToPrune.has(startKey) && startIsGraphLeaf) &&
                !(leafKeysToPrune.has(endKey) && endIsGraphLeaf)
            ) {
                continue;
            }
            segments.splice(i, 1);
            removedThisPass++;
        }
        if (!removedThisPass) break;
        prunedSegmentCount += removedThisPass;
        prunedPassCount++;
    }

    return {
        prunedSegmentCount,
        prunedPassCount
    };
}

function sourceTreePriority(segment) {
    if (segment.source?.startsWith('stl-slice-stub')) return 0;
    if (segment.source === 'stl-slice-branch-origin') return 1;
    if (segment.source?.includes('branch-origin')) return 1;
    return 2;
}

function removeCenterlineCycles(segments) {
    if (segments.length <= 1) {
        return {
            removedSegmentCount: 0,
            nodeCount: segments.length ? 2 : 0,
            cycleCount: 0
        };
    }

    const nodeIndex = new Map();
    const getNodeIndex = key => {
        let index = nodeIndex.get(key);
        if (index === undefined) {
            index = nodeIndex.size;
            nodeIndex.set(key, index);
        }
        return index;
    };
    const edges = segments.map((segment, index) => {
        const startKey = centerlineEndpointKey(segment, 'start');
        const endKey = centerlineEndpointKey(segment, 'end');
        return {
            index,
            segment,
            start: getNodeIndex(startKey),
            end: getNodeIndex(endKey),
            priority: sourceTreePriority(segment),
            length: segment.start.distanceTo(segment.end)
        };
    });

    const parent = Array.from({ length: nodeIndex.size }, (_, index) => index);
    const rank = new Uint8Array(nodeIndex.size);
    const findRoot = index => {
        let root = index;
        while (parent[root] !== root) root = parent[root];
        while (parent[index] !== index) {
            const next = parent[index];
            parent[index] = root;
            index = next;
        }
        return root;
    };
    const union = (a, b) => {
        const rootA = findRoot(a);
        const rootB = findRoot(b);
        if (rootA === rootB) return false;
        if (rank[rootA] < rank[rootB]) parent[rootA] = rootB;
        else if (rank[rootA] > rank[rootB]) parent[rootB] = rootA;
        else {
            parent[rootB] = rootA;
            rank[rootA]++;
        }
        return true;
    };

    const sorted = edges.slice().sort((a, b) =>
        b.priority - a.priority ||
        a.length - b.length ||
        a.index - b.index
    );
    const keep = new Set();
    let removedSegmentCount = 0;
    for (const edge of sorted) {
        if (edge.start === edge.end || !union(edge.start, edge.end)) {
            removedSegmentCount++;
            continue;
        }
        keep.add(edge.index);
    }

    if (!removedSegmentCount) {
        return {
            removedSegmentCount: 0,
            nodeCount: nodeIndex.size,
            cycleCount: Math.max(0, segments.length - nodeIndex.size + 1)
        };
    }

    const filtered = segments.filter((_, index) => keep.has(index));
    segments.length = 0;
    segments.push(...filtered);
    return {
        removedSegmentCount,
        nodeCount: nodeIndex.size,
        cycleCount: Math.max(0, segments.length - nodeIndex.size + 1)
    };
}

function appendAxisDebugSegments(target, source, axis) {
    for (let i = 0; i < source.length; i += 3) {
        const mapped = axisPointFromLocal({ x: source[i], y: source[i + 1], z: source[i + 2] }, axis);
        target.push(mapped.x, mapped.y, mapped.z);
    }
}

function thinDebugPositions(positions) {
    const edgeCount = positions.length / 6;
    if (edgeCount <= DEBUG_MAX_CONTOUR_SEGMENTS) return new Float32Array(positions);
    const thinned = [];
    const step = Math.ceil(edgeCount / DEBUG_MAX_CONTOUR_SEGMENTS);
    for (let edgeIndex = 0; edgeIndex < edgeCount; edgeIndex += step) {
        const offset = edgeIndex * 6;
        for (let i = 0; i < 6; i++) thinned.push(positions[offset + i]);
    }
    return new Float32Array(thinned);
}

function smoothCenterlineForSimulation(segments, geometry, wallBvh, {
    passes = 4,
    radialSamples = 16,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE
} = {}) {
    if (!segments.length || !geometry?.attributes?.position) {
        return { passCount: 0, movedNodeCount: 0, averageShift: 0, maxShift: 0 };
    }
    const lumenBvh = geometry.boundsTree || (geometry.boundsTree = new MeshBVH(geometry));
    const originalSegments = segments.map(segment => ({
        start: segment.start.clone(),
        end: segment.end.clone()
    }));
    let passCount = 0;
    let movedNodeCount = 0;
    let shiftSum = 0;
    let maxShift = 0;
    for (let pass = 0; pass < passes; pass++) {
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const nodes = collectRefinementNodes(segments);
        const candidates = [];
        for (const [key, incident] of incidence) {
            if (incident.length !== 2) continue;
            const deflection = centerlineNodeDeflectionDegrees(key, incident, incidence, endpointKeys);
            if (deflection < 5) continue;
            candidates.push({ key, deflection });
        }
        candidates.sort((a, b) => b.deflection - a.deflection);
        let movedThisPass = 0;
        for (const entry of candidates) {
            const node = nodes.get(entry.key);
            const current = currentNodePointFromIncidence(entry.key, incidence);
            const midpoint = nodeNeighbourMidpoint(entry.key, incidence);
            if (!node || !current || !midpoint) continue;
            const candidate = current.clone().lerp(midpoint, 0.45);
            const shift = candidate.distanceTo(current);
            if (shift < 0.01) continue;
            if (nodeCandidateSegmentsValidityFailureReason(
                entry.key,
                candidate,
                incidence,
                null,
                connectorLumenClearance,
                wallBvh
            )) {
                continue;
            }
            const before = nodeCenterShift(node, current, lumenBvh, {
                radialSamples,
                sphereSamples: 0
            });
            const after = nodeCenterShift(node, candidate, lumenBvh, {
                radialSamples,
                sphereSamples: 0
            });
            if (!before.pairCount || !after.pairCount) continue;
            const beforeOffset = before.shift.length();
            const afterOffset = after.shift.length();
            const beforeNormalized = beforeOffset / Math.max(0.5, before.meanRadius || node.radius || 1);
            const afterNormalized = afterOffset / Math.max(0.5, after.meanRadius || node.radius || 1);
            if (afterOffset > beforeOffset + 0.06 || afterNormalized > beforeNormalized + 0.012) continue;
            applyNodeCandidate(entry.key, candidate, incidence);
            movedNodeCount++;
            movedThisPass++;
            shiftSum += shift;
            maxShift = Math.max(maxShift, shift);
        }
        if (!movedThisPass) break;
        passCount++;
    }
    return {
        passCount,
        movedNodeCount,
        averageShift: movedNodeCount ? shiftSum / movedNodeCount : 0,
        maxShift
    };
}

function maximizeCenterlineWallClearance(segments, geometry, wallBvh, {
    passes = 3,
    directionCount = 16,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE
} = {}) {
    if (!segments.length || !geometry?.attributes?.position || !wallBvh?.closestPointToPoint) {
        return {
            passCount: 0,
            movedNodeCount: 0,
            averageShift: 0,
            maxShift: 0,
            averageClearanceGain: 0,
            maxClearanceGain: 0
        };
    }
    let passCount = 0;
    let movedNodeCount = 0;
    let shiftSum = 0;
    let maxShift = 0;
    let clearanceGainSum = 0;
    let maxClearanceGain = 0;
    const hitScratch = { point: new THREE.Vector3() };
    const wallDistance = (point, maxDistance = Infinity) => (
        wallBvh.closestPointToPoint(point, hitScratch, 0, maxDistance)?.distance ?? 0
    );
    for (let pass = 0; pass < passes; pass++) {
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const nodes = collectRefinementNodes(segments);
        const candidates = [];
        for (const [key, node] of nodes) {
            if (node.directions.length !== 2) continue;
            const point = currentNodePointFromIncidence(key, incidence);
            if (!point) continue;
            const clearance = wallDistance(point);
            candidates.push({
                key,
                node,
                point,
                clearance,
                closestWallPoint: hitScratch.point.clone(),
                deflection: centerlineNodeDeflectionDegrees(key, incidence.get(key), incidence, endpointKeys)
            });
        }
        candidates.sort((a, b) => a.clearance - b.clearance || b.deflection - a.deflection);
        const lockedKeys = new Set();
        let movedThisPass = 0;
        const candidateScales = directionCount > 8 ? [1, 0.65, 0.4] : [1, 0.65];
        for (const entry of candidates) {
            if (lockedKeys.has(entry.key)) continue;
            // An applied patch locks every node it changes, so an unlocked
            // candidate still has the point and clearance measured above.
            const current = entry.point;
            const currentClearance = entry.clearance;
            const tangent = nodeTangent(entry.node);
            if (!tangent) continue;
            const { u, v } = transverseBasis(tangent);
            const step = Math.max(0.22, Math.min(1.25, currentClearance * 0.32));
            const neighbourKeys = neighbourKeysForNode(entry.key, incidence, endpointKeys);
            const neighbourPoints = neighbourKeys
                .map(key => currentNodePointFromIncidence(key, incidence))
                .filter(Boolean);
            if (neighbourPoints.length !== 2) continue;
            let best = null;
            const choices = [];
            for (let directionIndex = 0; directionIndex < directionCount; directionIndex++) {
                const angle = 2 * Math.PI * directionIndex / directionCount;
                const direction = u.clone()
                    .multiplyScalar(Math.cos(angle))
                    .addScaledVector(v, Math.sin(angle))
                    .normalize();
                for (const scale of candidateScales) {
                    const shift = direction.clone().multiplyScalar(step * scale);
                    const candidatePoint = current.clone().add(shift);
                    const clearanceUpperBound = candidatePoint.distanceTo(entry.closestWallPoint);
                    if (clearanceUpperBound < currentClearance + 0.035 - 1e-9) continue;
                    const clearance = wallDistance(
                        candidatePoint,
                        clearanceUpperBound + 1e-4
                    );
                    const gain = clearance - currentClearance;
                    if (gain < 0.035) continue;
                    for (const neighbourScale of [0.62, 0.45]) {
                        const updates = buildOutlierPatchUpdates(
                            entry.key,
                            current,
                            shift,
                            incidence,
                            endpointKeys,
                            1,
                            neighbourScale,
                            neighbourKeys,
                            neighbourPoints
                        );
                        if (!updates) continue;
                        const updatedNeighbourPoints = neighbourKeys
                            .map(key => updates.get(key) || currentNodePointFromIncidence(key, incidence))
                            .filter(Boolean);
                        if (updatedNeighbourPoints.length !== 2) continue;
                        const first = updatedNeighbourPoints[0].clone().sub(candidatePoint).normalize();
                        const second = updatedNeighbourPoints[1].clone().sub(candidatePoint).normalize();
                        const deflection = 180 - THREE.MathUtils.radToDeg(Math.acos(THREE.MathUtils.clamp(
                            first.dot(second),
                            -1,
                            1
                        )));
                        if (deflection > Math.max(42, entry.deflection + 4)) continue;
                        const score = gain - Math.max(0, deflection - entry.deflection) * 0.006;
                        choices.push({ updates, shift: shift.length(), gain, score });
                    }
                }
            }
            choices.sort((a, b) => b.score - a.score);
            for (const choice of choices) {
                if (!nodePatchSegmentsStayValid(
                    choice.updates,
                    incidence,
                    endpointKeys,
                    null,
                    connectorLumenClearance,
                    wallBvh
                )) {
                    continue;
                }
                best = choice;
                break;
            }
            if (!best) continue;
            applyNodePatch(best.updates, incidence);
            movedNodeCount++;
            movedThisPass++;
            shiftSum += best.shift;
            maxShift = Math.max(maxShift, best.shift);
            clearanceGainSum += best.gain;
            maxClearanceGain = Math.max(maxClearanceGain, best.gain);
            lockedKeys.add(entry.key);
            for (const neighbourKey of neighbourKeysForNode(entry.key, incidence, endpointKeys)) {
                lockedKeys.add(neighbourKey);
            }
        }
        if (!movedThisPass) break;
        passCount++;
    }
    return {
        passCount,
        movedNodeCount,
        averageShift: movedNodeCount ? shiftSum / movedNodeCount : 0,
        maxShift,
        averageClearanceGain: movedNodeCount ? clearanceGainSum / movedNodeCount : 0,
        maxClearanceGain
    };
}

function correctCenterlineCenteringOutliers(segments, geometry, wallBvh, {
    radialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    maxNormalizedOffset = 0.3,
    maxOffset = 2.2,
    maxCandidates = 24,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE
} = {}) {
    const centeringBefore = measureCenterlineCentering(segments, geometry, {
        radialSamples,
        sphereSamples: 0
    });
    const candidatesByKey = new Map();
    for (const candidate of [
        ...(centeringBefore.worstNormalizedOffsets || []),
        ...(centeringBefore.worstOffsets || [])
    ]) {
        if (
            candidate.degree > 2 ||
            (candidate.normalizedOffset < maxNormalizedOffset && candidate.offset < maxOffset)
        ) {
            continue;
        }
        candidatesByKey.set(candidate.key, candidate);
    }
    const candidates = [...candidatesByKey.values()]
        .sort((a, b) =>
            Math.max(b.normalizedOffset / maxNormalizedOffset, b.offset / maxOffset) -
            Math.max(a.normalizedOffset / maxNormalizedOffset, a.offset / maxOffset)
        )
        .slice(0, maxCandidates);
    if (!candidates.length) {
        return {
            attemptedNodeCount: 0,
            correctedNodeCount: 0,
            averageShift: 0,
            maxShift: 0,
            centeringBefore,
            centeringAfter: centeringBefore
        };
    }

    const lumenBvh = geometry.boundsTree || (geometry.boundsTree = new MeshBVH(geometry));
    let correctedNodeCount = 0;
    let shiftSum = 0;
    let maxShift = 0;
    for (const entry of candidates) {
        const incidence = collectNodeIncidence(segments);
        const endpointKeys = collectSegmentEndpointKeyMap(incidence);
        const incident = incidence.get(entry.key);
        if (!incident || incident.length > 2) continue;
        const node = collectRefinementNodes(segments).get(entry.key);
        const current = currentNodePointFromIncidence(entry.key, incidence);
        if (!node || !current) continue;
        const before = nodeCenterShift(node, current, lumenBvh, {
            radialSamples,
            sphereSamples: 0
        });
        if (!before.pairCount) continue;
        const beforeOffset = before.shift.length();
        const beforeNormalized = beforeOffset / Math.max(0.5, before.meanRadius || node.radius || 1);
        const beforeDeflection = centerlineNodeDeflectionDegrees(
            entry.key,
            incident,
            incidence,
            endpointKeys
        );
        let best = null;
        for (const scale of [1, 0.82, 0.64, 0.46, 0.3]) {
            const point = current.clone().addScaledVector(before.shift, scale);
            if (nodeCandidateSegmentsValidityFailureReason(
                entry.key,
                point,
                incidence,
                null,
                connectorLumenClearance,
                wallBvh
            )) {
                continue;
            }
            const after = nodeCenterShift(node, point, lumenBvh, {
                radialSamples,
                sphereSamples: 0
            });
            if (!after.pairCount) continue;
            const afterOffset = after.shift.length();
            const afterNormalized = afterOffset / Math.max(0.5, after.meanRadius || node.radius || 1);
            applyNodeCandidate(entry.key, point, incidence);
            const afterDeflection = centerlineNodeDeflectionDegrees(
                entry.key,
                incidence.get(entry.key),
                incidence,
                collectSegmentEndpointKeyMap(incidence)
            );
            applyNodeCandidate(entry.key, current, incidence);
            if (
                afterOffset >= beforeOffset - 0.01 ||
                afterNormalized >= beforeNormalized - 0.01 ||
                afterDeflection > Math.max(42, beforeDeflection + 3)
            ) {
                continue;
            }
            if (!best || afterNormalized < best.normalizedOffset) {
                best = { point, normalizedOffset: afterNormalized };
            }
        }
        if (!best) continue;
        const shift = best.point.distanceTo(current);
        applyNodeCandidate(entry.key, best.point, incidence);
        correctedNodeCount++;
        shiftSum += shift;
        maxShift = Math.max(maxShift, shift);
    }

    let centeringAfter = measureCenterlineCentering(segments, geometry, {
        radialSamples,
        sphereSamples: 0
    });
    const rolledBack = correctedNodeCount > 0 && (
        centeringAfter.maxOffset > centeringBefore.maxOffset + 0.005 ||
        centeringAfter.maxNormalizedOffset > centeringBefore.maxNormalizedOffset + 0.001 ||
        centeringAfter.averageOffset > centeringBefore.averageOffset + 0.001 ||
        centeringAfter.averageNormalizedOffset > centeringBefore.averageNormalizedOffset + 0.001
    );
    if (rolledBack) {
        for (let index = 0; index < segments.length; index++) {
            segments[index].start = originalSegments[index].start;
            segments[index].end = originalSegments[index].end;
        }
        correctedNodeCount = 0;
        shiftSum = 0;
        maxShift = 0;
        centeringAfter = centeringBefore;
    }
    return {
        attemptedNodeCount: candidates.length,
        correctedNodeCount,
        averageShift: correctedNodeCount ? shiftSum / correctedNodeCount : 0,
        maxShift,
        rolledBack,
        centeringBefore,
        centeringAfter
    };
}

function buildMedialSliceCenterline(geometry, {
    sliceSpacing,
    targetSliceCount,
    contourTolerance,
    centerlineMinLumenArea,
    centerlineMinCompactness,
    centerlineNodeSpacing,
    centerlineRefinement,
    centerlineRefinementRadialSamples,
    centerlineRefinementIterations,
    lumenField,
    wallBvh
}) {
    const startedAt = nowMs();
    const resolvedWallBvh = resolveWallBvh(geometry, wallBvh);
    const requestedSpacing = resolveSliceSpacing(
        geometry,
        sliceSpacing,
        Math.min(700, Math.max(180, targetSliceCount))
    );
    const medialSliceSpacing = Math.max(0.9, requestedSpacing);
    const gridSpacing = Math.max(0.85, Math.min(1.35, medialSliceSpacing * 0.95));
    const slices = extractStlSlices(
        geometry,
        medialSliceSpacing,
        contourTolerance,
        Math.max(0.35, Math.min(1, centerlineMinLumenArea)),
        Math.max(0.005, Math.min(0.03, centerlineMinCompactness)),
        Infinity,
        { solidLumen: false, centerMode: 'medial' }
    );
    const extractionMs = nowMs() - startedAt;
    const medial = buildMedialCenterlineTree(slices, {
        lumenField,
        wallBvh: resolvedWallBvh,
        gridSpacing,
        nodeSpacing: centerlineNodeSpacing,
        smoothingPasses: 4
    });
    const allSegments = medial.segments;
    const treeMs = nowMs() - startedAt - extractionMs;

    let refinement = {
        refinedNodeCount: 0,
        skippedNodeCount: 0,
        failedNodeCount: 0,
        wallRejectedSegmentCount: 0,
        wallClampedSegmentCount: 0,
        averageShift: 0,
        maxShift: 0
    };
    const refinementStartedAt = nowMs();
    if (centerlineRefinement && allSegments.length) {
        let shiftSum = 0;
        for (let pass = 0; pass < 2; pass++) {
            const passRefinement = refineCenterlineSegmentsToLumen(allSegments, geometry, {
                radialSamples: Math.max(12, Math.min(24, centerlineRefinementRadialSamples)),
                sphereSamples: 0,
                iterations: Math.max(2, Math.min(4, centerlineRefinementIterations)),
                wallBvh: resolvedWallBvh
            });
            refinement.refinedNodeCount += passRefinement.refinedNodeCount;
            refinement.skippedNodeCount += passRefinement.skippedNodeCount;
            refinement.failedNodeCount += passRefinement.failedNodeCount;
            refinement.wallRejectedSegmentCount += passRefinement.wallRejectedSegmentCount;
            refinement.wallClampedSegmentCount += passRefinement.wallClampedSegmentCount;
            shiftSum += passRefinement.averageShift * passRefinement.refinedNodeCount;
            refinement.maxShift = Math.max(refinement.maxShift, passRefinement.maxShift);
            const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
            if (resampled.segments !== allSegments) {
                allSegments.length = 0;
                allSegments.push(...resampled.segments);
            }
        }
        refinement.averageShift = refinement.refinedNodeCount
            ? shiftSum / refinement.refinedNodeCount
            : 0;
    }
    const refinementMs = nowMs() - refinementStartedAt;
    const clearanceStartedAt = nowMs();
    const clearanceMaximization = maximizeCenterlineWallClearance(
        allSegments,
        geometry,
        resolvedWallBvh,
        { passes: 1, directionCount: 16 }
    );
    const clearancePolish = maximizeCenterlineWallClearance(
        allSegments,
        geometry,
        resolvedWallBvh,
        { passes: 4, directionCount: 8 }
    );
    const clearanceMovedNodeCount = clearanceMaximization.movedNodeCount + clearancePolish.movedNodeCount;
    clearanceMaximization.passCount += clearancePolish.passCount;
    clearanceMaximization.averageShift = clearanceMovedNodeCount
        ? (
            clearanceMaximization.averageShift * clearanceMaximization.movedNodeCount +
            clearancePolish.averageShift * clearancePolish.movedNodeCount
        ) / clearanceMovedNodeCount
        : 0;
    clearanceMaximization.averageClearanceGain = clearanceMovedNodeCount
        ? (
            clearanceMaximization.averageClearanceGain * clearanceMaximization.movedNodeCount +
            clearancePolish.averageClearanceGain * clearancePolish.movedNodeCount
        ) / clearanceMovedNodeCount
        : 0;
    clearanceMaximization.movedNodeCount = clearanceMovedNodeCount;
    clearanceMaximization.maxShift = Math.max(
        clearanceMaximization.maxShift,
        clearancePolish.maxShift
    );
    clearanceMaximization.maxClearanceGain = Math.max(
        clearanceMaximization.maxClearanceGain,
        clearancePolish.maxClearanceGain
    );
    const clearanceMs = nowMs() - clearanceStartedAt;
    const backtrackStartedAt = nowMs();
    const backtrackSimplification = simplifyCenterlineBacktracks(
        allSegments,
        resolvedWallBvh ? null : lumenField,
        resolvedWallBvh,
        DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
        { minDeflectionDegrees: 82, maxPasses: 12 }
    );
    if (backtrackSimplification.collapsedNodeCount) {
        const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        if (resampled.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampled.segments);
        }
    }
    const backtrackMs = nowMs() - backtrackStartedAt;
    const simulationSmoothingStartedAt = nowMs();
    const simulationSmoothing = smoothCenterlineForSimulation(
        allSegments,
        geometry,
        resolvedWallBvh,
        {
            passes: 5,
            radialSamples: Math.max(12, Math.min(20, centerlineRefinementRadialSamples))
        }
    );
    if (simulationSmoothing.movedNodeCount) {
        const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        if (resampled.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampled.segments);
        }
    }
    const simulationSmoothingMs = nowMs() - simulationSmoothingStartedAt;
    const finalBacktrackStartedAt = nowMs();
    const finalBacktrackSimplification = simplifyCenterlineBacktracks(
        allSegments,
        resolvedWallBvh ? null : lumenField,
        resolvedWallBvh,
        DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
        { minDeflectionDegrees: 74, maxPasses: 12 }
    );
    if (finalBacktrackSimplification.collapsedNodeCount) {
        const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        if (resampled.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampled.segments);
        }
    }
    const finalBacktrackMs = nowMs() - finalBacktrackStartedAt;
    const centeringCorrectionStartedAt = nowMs();
    const centeringCorrection = correctCenterlineCenteringOutliers(
        allSegments,
        geometry,
        resolvedWallBvh,
        {
            radialSamples: Math.max(12, Math.min(24, centerlineRefinementRadialSamples))
        }
    );
    if (centeringCorrection.correctedNodeCount) {
        const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        if (resampled.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampled.segments);
        }
    }
    const centeringCorrectionMs = nowMs() - centeringCorrectionStartedAt;
    const postCenteringBacktrackStartedAt = nowMs();
    const postCenteringBacktrackSimplification = simplifyCenterlineBacktracks(
        allSegments,
        resolvedWallBvh ? null : lumenField,
        resolvedWallBvh,
        DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
        { minDeflectionDegrees: 74, maxPasses: 12 }
    );
    if (postCenteringBacktrackSimplification.collapsedNodeCount) {
        const resampled = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        if (resampled.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampled.segments);
        }
    }
    const postCenteringBacktrackMs = nowMs() - postCenteringBacktrackStartedAt;
    const cyclePruning = removeCenterlineCycles(allSegments);
    const components = segmentComponents(allSegments);
    const topology = measureCenterlineTopology(allSegments);
    const centeringStartedAt = nowMs();
    const centering = measureCenterlineCentering(allSegments, geometry, {
        radialSamples: Math.max(12, Math.min(24, centerlineRefinementRadialSamples)),
        sphereSamples: 0
    });
    const centeringMs = nowMs() - centeringStartedAt;
    const invalidSegments = invalidCenterlineSegments(
        allSegments,
        resolvedWallBvh ? null : lumenField,
        resolvedWallBvh,
        DEFAULT_CONNECTOR_LUMEN_CLEARANCE
    );
    const debugSegments = thinDebugPositions(buildDebugSegments(slices));
    const totalMs = nowMs() - startedAt;
    const coverage = {
        sampleCount: medial.diagnostics.retainedGraphNodeCount,
        coveredSampleCount: medial.diagnostics.coveredNodeCount,
        uncoveredSampleCount: medial.diagnostics.uncoveredNodeCount,
        coverageRate: medial.diagnostics.retainedGraphNodeCount
            ? medial.diagnostics.coveredNodeCount / medial.diagnostics.retainedGraphNodeCount
            : 0
    };
    const diagnostics = {
        source: 'medial-slice-teasar',
        algorithm: 'medial-slice-teasar',
        timings: {
            extractionMs,
            treeMs,
            refinementMs,
            clearanceMs,
            backtrackMs,
            simulationSmoothingMs,
            finalBacktrackMs,
            centeringCorrectionMs,
            postCenteringBacktrackMs,
            centeringMs,
            totalMs
        },
        axes: 'y-medial-skeleton',
        axisDiagnostics: [{
            axis: 'y',
            pass: 'medial-skeleton',
            sliceCount: slices.length,
            contourCount: medial.diagnostics.contourCount,
            candidateSegmentCount: medial.diagnostics.graphEdgeCount,
            componentCount: medial.diagnostics.graphComponentCount,
            usedLumenSurfaceSlices: true,
            centerMode: 'topological-medial-axis',
            sliceSpacing: medialSliceSpacing,
            elapsedMs: extractionMs + treeMs
        }],
        sliceCount: slices.length,
        contourCount: medial.diagnostics.contourCount,
        edgeCount: allSegments.length,
        stubSegmentCount: 0,
        isolatedNodeCount: 0,
        componentCount: components.length,
        componentSegmentCounts: components.map(component => component.length),
        uncoveredNodeCount: coverage.uncoveredSampleCount,
        centerlineCoverage: coverage,
        centerlineNodeSpacing,
        centerlineMinLumenArea,
        centerlineMinCompactness,
        centerlineGraphNodeCount: topology.nodeCount,
        centerlineGraphCycleCount: 0,
        centerlineCyclePrunedSegmentCount: cyclePruning.removedSegmentCount,
        centerlineTopologyBeforeCleanup: topology,
        centerlineTopologyAfterCleanup: topology,
        centerlineBacktrackCollapsedNodeCount:
            backtrackSimplification.collapsedNodeCount +
            finalBacktrackSimplification.collapsedNodeCount +
            postCenteringBacktrackSimplification.collapsedNodeCount,
        centerlineBacktrackRemovedSegmentCount:
            backtrackSimplification.removedSegmentCount +
            finalBacktrackSimplification.removedSegmentCount +
            postCenteringBacktrackSimplification.removedSegmentCount,
        centerlineBacktrackInsertedSegmentCount:
            backtrackSimplification.insertedSegmentCount +
            finalBacktrackSimplification.insertedSegmentCount +
            postCenteringBacktrackSimplification.insertedSegmentCount,
        centerlineBacktrackRejectedNodeCount:
            backtrackSimplification.rejectedNodeCount +
            finalBacktrackSimplification.rejectedNodeCount +
            postCenteringBacktrackSimplification.rejectedNodeCount,
        centerlineBacktrackPathLengthReduction:
            backtrackSimplification.pathLengthReduction +
            finalBacktrackSimplification.pathLengthReduction +
            postCenteringBacktrackSimplification.pathLengthReduction,
        centerlineSimulationSmoothing: simulationSmoothing,
        centerlineCenteringCorrection: {
            attemptedNodeCount: centeringCorrection.attemptedNodeCount,
            correctedNodeCount: centeringCorrection.correctedNodeCount,
            averageShift: centeringCorrection.averageShift,
            maxShift: centeringCorrection.maxShift,
            rolledBack: centeringCorrection.rolledBack || false
        },
        centerlineInvalidSegmentCountBeforeReroute: invalidSegments.length,
        centerlineInvalidSegmentCountAfterReroute: invalidSegments.length,
        centerlineInvalidSegmentCountFinal: invalidSegments.length,
        centerlineCenteringMeasuredNodeCount: centering.measuredNodeCount,
        centerlineCenteringFailedNodeCount: centering.failedNodeCount,
        centerlineCenteringAverageOffset: centering.averageOffset,
        centerlineCenteringMaxOffset: centering.maxOffset,
        centerlineCenteringAverageNormalizedOffset: centering.averageNormalizedOffset,
        centerlineCenteringMaxNormalizedOffset: centering.maxNormalizedOffset,
        centerlineCenteringWorstOffsets: centering.worstOffsets,
        centerlineCenteringWorstNormalizedOffsets: centering.worstNormalizedOffsets,
        centerlineRefinement,
        centerlineRefinedNodeCount: refinement.refinedNodeCount,
        centerlineRefinementSkippedNodeCount: refinement.skippedNodeCount,
        centerlineRefinementFailedNodeCount: refinement.failedNodeCount,
        centerlineRefinementWallRejectedSegmentCount: refinement.wallRejectedSegmentCount,
        centerlineRefinementWallClampedSegmentCount: refinement.wallClampedSegmentCount,
        centerlineRefinementAverageShift: refinement.averageShift,
        centerlineRefinementMaxShift: refinement.maxShift,
        centerlineClearanceMaximization: clearanceMaximization,
        medialTree: medial.diagnostics,
        wallValidation: resolvedWallBvh ? 'stl-bvh' : 'lumen-field'
    };
    allSegments.diagnostics = diagnostics;
    return {
        slices,
        lumenCast: {
            geometry: null,
            slices,
            field: lumenField,
            diagnostics: {
                source: 'direct-medial-slices',
                sliceCount: slices.length,
                contourCount: medial.diagnostics.contourCount
            }
        },
        segments: allSegments,
        debugSegments,
        diagnostics
    };
}

export function buildStlSliceCenterline(geometry, {
    sliceSpacing = DEFAULT_SLICE_SPACING,
    targetSliceCount = DEFAULT_TARGET_SLICE_COUNT,
    contourTolerance = DEFAULT_CONTOUR_TOLERANCE,
    minLumenArea = DEFAULT_MIN_LUMEN_AREA,
    minCompactness = DEFAULT_MIN_COMPACTNESS,
    maxLinkGap = DEFAULT_MAX_LINK_GAP,
    secondaryAxisSpacingMultiplier = DEFAULT_SECONDARY_AXIS_SPACING_MULTIPLIER,
    axisSliceSpacing = null,
    axisSliceSpacingMultipliers = DEFAULT_AXIS_SLICE_SPACING_MULTIPLIERS,
    secondaryAxisMaxRadius = DEFAULT_SECONDARY_AXIS_MAX_RADIUS,
    axes = CENTERLINE_AXES,
    lumenField = null,
    connectorLumenClearance = DEFAULT_CONNECTOR_LUMEN_CLEARANCE,
    wallBvh = null,
    lumenCast = null,
    centerlineNodeSpacing = DEFAULT_CENTERLINE_NODE_SPACING,
    centerlineMinLumenArea = DEFAULT_CENTERLINE_MIN_LUMEN_AREA,
    centerlineMinCompactness = DEFAULT_CENTERLINE_MIN_COMPACTNESS,
    maxAdaptiveDirections = DEFAULT_MAX_ADAPTIVE_DIRECTIONS,
    centerlineRefinement = true,
    centerlineRefinementRadialSamples = DEFAULT_CENTERLINE_REFINE_RADIAL_SAMPLES,
    centerlineRefinementSphereSamples = DEFAULT_CENTERLINE_REFINE_SPHERE_SAMPLES,
    centerlineRefinementIterations = DEFAULT_CENTERLINE_REFINE_ITERATIONS,
    centerlineRefinementPasses = DEFAULT_CENTERLINE_REFINE_PASSES,
    centerlineClearanceRelaxPasses = DEFAULT_CENTERLINE_CLEARANCE_RELAX_PASSES,
    centerlineOutlierReroutePasses = CENTERLINE_OUTLIER_REROUTE_PASSES,
    centerlineOutlierRerouteMaxCandidates = CENTERLINE_OUTLIER_REROUTE_MAX_CANDIDATES,
    algorithm = 'medial-slice-teasar'
} = {}) {
    if (algorithm === 'medial-slice-teasar' && lumenField?.query) {
        return buildMedialSliceCenterline(geometry, {
            sliceSpacing,
            targetSliceCount,
            contourTolerance,
            centerlineMinLumenArea,
            centerlineMinCompactness,
            centerlineNodeSpacing,
            centerlineRefinement,
            centerlineRefinementRadialSamples,
            centerlineRefinementIterations,
            lumenField,
            wallBvh
        });
    }
    const buildStartedAt = nowMs();
    const timings = {};
    const profileStages = typeof process !== 'undefined' &&
        process?.env?.CENTERLINE_PROFILE === '1';
    const recordTiming = (name, startedAt) => {
        timings[name] = (timings[name] || 0) + nowMs() - startedAt;
        if (profileStages) {
            console.log(`[centerline] ${name}: ${timings[name].toFixed(1)} ms`);
        }
    };
    const allSegments = [];
    const debugPositions = [];
    const axisDiagnostics = [];
    const candidateComponents = [];
    const primaryCoverageSamples = [];
    let nodeOffset = 0;
    let totalSlices = 0;
    let totalContours = 0;
    let totalSplits = 0;
    let totalMerges = 0;
    const baseSliceSpacing = resolveSliceSpacing(geometry, sliceSpacing, targetSliceCount);
    const effectiveCenterlineMinArea = Number.isFinite(centerlineMinLumenArea) && centerlineMinLumenArea > 0
        ? centerlineMinLumenArea
        : minLumenArea;
    const effectiveCenterlineMinCompactness = Number.isFinite(centerlineMinCompactness) && centerlineMinCompactness >= 0
        ? centerlineMinCompactness
        : minCompactness;
    let stageStartedAt = nowMs();
    const resolvedWallBvh = resolveWallBvh(geometry, wallBvh);
    const resolvedLumenCast = lumenCast || buildStlLumenCast(geometry, {
        sliceSpacing,
        targetSliceCount,
        contourTolerance,
        minLumenArea: Math.min(minLumenArea, effectiveCenterlineMinArea),
        minCompactness: Math.min(minCompactness, effectiveCenterlineMinCompactness),
        lumenField
    });
    recordTiming('lumenCastMs', stageStartedAt);
    const connectorField = resolvedLumenCast.field || lumenField;
    const processedAxes = [];

    const collectAxisCandidates = (axis, axisIndex, pass = 'base') => {
        const axisStartedAt = nowMs();
        const descriptor = centerlineAxisDescriptor(axis, axisIndex);
        const axisId = descriptor.id;
        const resolvedSliceSpacing = resolveAxisSliceSpacing(
            descriptor,
            baseSliceSpacing,
            axisSliceSpacing,
            axisSliceSpacingMultipliers,
            secondaryAxisSpacingMultiplier
        );
        const axisGeometry = axisId === 'y'
            ? resolvedLumenCast.geometry
            : buildAxisGeometry(resolvedLumenCast.geometry, descriptor);
        let slices = extractStlSlices(
            axisGeometry,
            resolvedSliceSpacing,
            contourTolerance,
            effectiveCenterlineMinArea,
            effectiveCenterlineMinCompactness,
            axisId === 'y' ? Infinity : secondaryAxisMaxRadius,
            { solidLumen: true, centerMode: 'medial' }
        );
        let axisDebugSegments = buildDebugSegments(slices);
        let usedLumenSurfaceSlices = true;
        if (axisId === 'y' && !slices.length) {
            slices = cloneSlicesForCenterline(resolvedLumenCast.slices);
            axisDebugSegments = resolvedLumenCast.debugSegments;
            usedLumenSurfaceSlices = false;
        }
        if (axisId === 'y') {
            for (const slice of slices) {
                for (const contour of slice.contours) {
                    const center = contour.center || contour.linkCenter || contour.centroid;
                    if (!center) continue;
                    primaryCoverageSamples.push({
                        axis: axisId,
                        point: axisPointFromLocal({ x: center.x, y: slice.y, z: center.z }, descriptor),
                        radius: contour.radius
                    });
                }
            }
        }
        const axisSegments = buildCenterlineFromSlices(slices, resolvedSliceSpacing, maxLinkGap);
        const mappedSegments = [];
        let obliqueRejectedSegmentCount = 0;
        for (const segment of axisSegments) {
            const mappedSegment = mapAxisSegment(segment, descriptor, nodeOffset);
            if (isObliqueAxisArtifact(mappedSegment, descriptor)) {
                obliqueRejectedSegmentCount++;
                continue;
            }
            mappedSegments.push(mappedSegment);
        }
        const components = segmentComponents(mappedSegments);
        for (let componentIndex = 0; componentIndex < components.length; componentIndex++) {
            candidateComponents.push({
                axis: axisId,
                pass,
                componentIndex,
                segments: components[componentIndex]
            });
        }
        appendAxisDebugSegments(debugPositions, axisDebugSegments, descriptor);

        const diagnostics = {
            ...axisSegments.diagnostics,
            axis: axisId,
            pass,
            contourCount: slices.reduce((sum, slice) => sum + slice.contours.length, 0),
            debugSegmentCount: axisDebugSegments.length / 6,
            componentCount: components.length,
            candidateSegmentCount: mappedSegments.length,
            candidateStubCount: mappedSegments.filter(segment => segment.source.startsWith('stl-slice-stub')).length,
            obliqueRejectedSegmentCount,
            wallRejectedSegmentCount: 0,
            usedLumenSurfaceSlices,
            centerMode: usedLumenSurfaceSlices ? 'medial-lumen-surface' : 'fallback-original-slices',
            explicitAxisSliceSpacing: axisOptionValue(axisSliceSpacing, descriptor) || null,
            axisSliceSpacingMultiplier: axisOptionValue(axisSliceSpacingMultipliers, descriptor) || null,
            requestedSliceCount: targetSliceCount,
            sliceSpacing: resolvedSliceSpacing,
            normal: {
                x: descriptor.n.x,
                y: descriptor.n.y,
                z: descriptor.n.z
            },
            elapsedMs: nowMs() - axisStartedAt
        };
        axisDiagnostics.push(diagnostics);
        processedAxes.push(descriptor);
        nodeOffset += diagnostics.nodeCount;
        totalSlices += diagnostics.sliceCount;
        totalContours += diagnostics.contourCount;
        totalSplits += diagnostics.splitNodeCount;
        totalMerges += diagnostics.mergeNodeCount;
    };

    stageStartedAt = nowMs();
    axes.forEach((axis, axisIndex) => collectAxisCandidates(axis, axisIndex, 'base'));
    recordTiming('baseAxesMs', stageStartedAt);

    stageStartedAt = nowMs();
    const preliminaryTree = buildConnectedCenterlineTree(
        candidateComponents,
        connectorField,
        connectorLumenClearance,
        resolvedWallBvh
    );
    const adaptiveAxes = collectAdaptiveCenterlineAxes(
        preliminaryTree.segments,
        processedAxes,
        maxAdaptiveDirections
    );
    recordTiming('preliminaryTreeMs', stageStartedAt);

    stageStartedAt = nowMs();
    adaptiveAxes.forEach((axis, axisIndex) => collectAxisCandidates(axis, axes.length + axisIndex, 'adaptive'));
    recordTiming('adaptiveAxesMs', stageStartedAt);

    stageStartedAt = nowMs();
    const tree = buildConnectedCenterlineTree(
        candidateComponents,
        connectorField,
        connectorLumenClearance,
        resolvedWallBvh
    );
    allSegments.push(...tree.segments);
    recordTiming('connectedTreeMs', stageStartedAt);

    stageStartedAt = nowMs();
    const primaryAxisRescue = rescuePrimaryAxisComponents(
        allSegments,
        tree.discardedEntries,
        connectorField,
        connectorLumenClearance,
        resolvedWallBvh,
        resolvedLumenCast.geometry
    );
    recordTiming('primaryAxisRescueMs', stageStartedAt);

    stageStartedAt = nowMs();
    const rooting = {
        addedBranchOriginCount:
            tree.addedBranchOriginCount + primaryAxisRescue.rescuedComponentCount,
        discardedComponentCount: Math.max(
            0,
            tree.discardedComponentCount - primaryAxisRescue.rescuedComponentCount
        ),
        discardedSegmentCount: Math.max(
            0,
            tree.discardedSegmentCount - primaryAxisRescue.addedComponentSegmentCount
        )
    };
    if (allSegments.length) {
        const rooted = rootCenterlineComponents(allSegments, connectorField, connectorLumenClearance, resolvedWallBvh);
        if (rooted.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...rooted.segments);
        }
        rooting.addedBranchOriginCount += rooted.addedBranchOriginCount;
        rooting.discardedComponentCount += rooted.discardedComponentCount;
        rooting.discardedSegmentCount += rooted.discardedSegmentCount;
    }
    recordTiming('rootingMs', stageStartedAt);

    stageStartedAt = nowMs();
    const branchRouting = routeBranchOriginSegments(
        allSegments,
        connectorField,
        resolvedWallBvh,
        connectorLumenClearance,
        resolvedLumenCast.geometry
    );
    recordTiming('branchRoutingMs', stageStartedAt);

    stageStartedAt = nowMs();
    const resampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
    if (resampling.segments !== allSegments) {
        allSegments.length = 0;
        allSegments.push(...resampling.segments);
    }
    recordTiming('initialResamplingMs', stageStartedAt);
    const refinementPassCount = centerlineRefinement
        ? Math.max(1, Math.round(centerlineRefinementPasses))
        : 0;
    const refinement = {
        refinedNodeCount: 0,
        skippedNodeCount: 0,
        failedNodeCount: 0,
        wallRejectedSegmentCount: 0,
        wallClampedSegmentCount: 0,
        averageShift: 0,
        maxShift: 0
    };
    let refinementShiftSum = 0;
    let postRefinementInsertedResampleNodeCount = 0;
    stageStartedAt = nowMs();
    for (let refinementPass = 0; refinementPass < refinementPassCount; refinementPass++) {
        const passRefinement = refineCenterlineSegmentsToLumen(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples,
            iterations: centerlineRefinementIterations,
            wallBvh: resolvedWallBvh,
            skipRoutedBranchNodes: true
        });
        refinement.refinedNodeCount += passRefinement.refinedNodeCount;
        refinement.skippedNodeCount += passRefinement.skippedNodeCount;
        refinement.failedNodeCount += passRefinement.failedNodeCount;
        refinement.wallRejectedSegmentCount += passRefinement.wallRejectedSegmentCount;
        refinement.wallClampedSegmentCount += passRefinement.wallClampedSegmentCount;
        refinementShiftSum += passRefinement.averageShift * passRefinement.refinedNodeCount;
        refinement.maxShift = Math.max(refinement.maxShift, passRefinement.maxShift);

        const passResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += passResampling.insertedNodeCount;
        if (passResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...passResampling.segments);
        }
    }
    recordTiming('refinementMs', stageStartedAt);

    stageStartedAt = nowMs();
    const leafPruning = pruneOffCenterLeafSegments(allSegments, resolvedLumenCast.geometry, {
        radialSamples: centerlineRefinementRadialSamples,
        sphereSamples: centerlineRefinementSphereSamples
    });
    recordTiming('leafPruningMs', stageStartedAt);

    stageStartedAt = nowMs();
    const postPruneRooting = {
        addedBranchOriginCount: 0,
        discardedComponentCount: 0,
        discardedSegmentCount: 0
    };
    const postPruneComponents = segmentComponents(allSegments);
    if (postPruneComponents.length > 1) {
        const rooted = rootCenterlineComponents(allSegments, connectorField, connectorLumenClearance, resolvedWallBvh);
        if (rooted.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...rooted.segments);
        }
        postPruneRooting.addedBranchOriginCount += rooted.addedBranchOriginCount;
        postPruneRooting.discardedComponentCount += rooted.discardedComponentCount;
        postPruneRooting.discardedSegmentCount += rooted.discardedSegmentCount;
        rooting.addedBranchOriginCount += rooted.addedBranchOriginCount;
        rooting.discardedComponentCount += rooted.discardedComponentCount;
        rooting.discardedSegmentCount += rooted.discardedSegmentCount;

        const reconnectBranchRouting = routeBranchOriginSegments(
            allSegments,
            connectorField,
            resolvedWallBvh,
            connectorLumenClearance,
            resolvedLumenCast.geometry
        );
        branchRouting.routedSegmentCount += reconnectBranchRouting.routedSegmentCount;
        branchRouting.routedPathSegmentCount += reconnectBranchRouting.routedPathSegmentCount;
        branchRouting.gridRouteCount += reconnectBranchRouting.gridRouteCount;
        branchRouting.gridRoutePathSegmentCount += reconnectBranchRouting.gridRoutePathSegmentCount;
        branchRouting.failedRouteCount += reconnectBranchRouting.failedRouteCount;
        branchRouting.routeChoiceDiagnostics.push(...(reconnectBranchRouting.routeChoiceDiagnostics || []));
        branchRouting.relaxedRouteNodeCount += reconnectBranchRouting.relaxedRouteNodeCount;
        branchRouting.centeredRouteNodeCount += reconnectBranchRouting.centeredRouteNodeCount;
        branchRouting.totalRouteClearanceGain += reconnectBranchRouting.totalRouteClearanceGain;
        branchRouting.maxRouteClearanceGain = Math.max(
            branchRouting.maxRouteClearanceGain,
            reconnectBranchRouting.maxRouteClearanceGain
        );

        const reconnectResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += reconnectResampling.insertedNodeCount;
        if (reconnectResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...reconnectResampling.segments);
        }
        if (centerlineRefinement) {
            const reconnectRefinement = refineCenterlineSegmentsToLumen(allSegments, resolvedLumenCast.geometry, {
                radialSamples: centerlineRefinementRadialSamples,
                sphereSamples: centerlineRefinementSphereSamples,
                iterations: centerlineRefinementIterations,
                wallBvh: resolvedWallBvh,
                skipRoutedBranchNodes: true
            });
            refinement.refinedNodeCount += reconnectRefinement.refinedNodeCount;
            refinement.skippedNodeCount += reconnectRefinement.skippedNodeCount;
            refinement.failedNodeCount += reconnectRefinement.failedNodeCount;
            refinement.wallRejectedSegmentCount += reconnectRefinement.wallRejectedSegmentCount;
            refinement.wallClampedSegmentCount += reconnectRefinement.wallClampedSegmentCount;
            refinementShiftSum += reconnectRefinement.averageShift * reconnectRefinement.refinedNodeCount;
            refinement.maxShift = Math.max(refinement.maxShift, reconnectRefinement.maxShift);

            const finalReconnectResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
            postRefinementInsertedResampleNodeCount += finalReconnectResampling.insertedNodeCount;
            if (finalReconnectResampling.segments !== allSegments) {
                allSegments.length = 0;
                allSegments.push(...finalReconnectResampling.segments);
            }
        }
    }
    recordTiming('postPruneReconnectMs', stageStartedAt);
    refinement.averageShift = refinement.refinedNodeCount
        ? refinementShiftSum / refinement.refinedNodeCount
        : 0;
    stageStartedAt = nowMs();
    let cyclePruning = removeCenterlineCycles(allSegments);
    const clearanceRelaxation = relaxCenterlineNodesByLumenClearance(allSegments, {
        lumenField: connectorField,
        connectorLumenClearance,
        wallBvh: resolvedWallBvh,
        passCount: centerlineClearanceRelaxPasses
    });
    if (clearanceRelaxation.adjustedNodeCount > 0) {
        const clearanceRelaxResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += clearanceRelaxResampling.insertedNodeCount;
        if (clearanceRelaxResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...clearanceRelaxResampling.segments);
        }
        const clearanceRelaxCyclePruning = removeCenterlineCycles(allSegments);
        cyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + clearanceRelaxCyclePruning.removedSegmentCount,
            nodeCount: clearanceRelaxCyclePruning.nodeCount,
            cycleCount: clearanceRelaxCyclePruning.cycleCount
        };
    }
    recordTiming('cycleAndClearanceMs', stageStartedAt);

    stageStartedAt = nowMs();
    let centering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
        radialSamples: centerlineRefinementRadialSamples,
        sphereSamples: centerlineRefinementSphereSamples
    });
    recordTiming('centeringMeasureMs', stageStartedAt);
    const preOutlierRelax = {
        averageOffset: centering.averageOffset,
        maxOffset: centering.maxOffset,
        averageNormalizedOffset: centering.averageNormalizedOffset,
        maxNormalizedOffset: centering.maxNormalizedOffset
    };
    const outlierRelaxation = {
        adjustedNodeCount: 0,
        keptNodeCount: 0,
        rejectedNodeCount: 0,
        averageShift: 0,
        maxShift: 0,
        averageClearanceGain: 0,
        maxClearanceGain: 0,
        directCenteredNodeCount: 0,
        directWallFallbackNodeCount: 0,
        referenceFieldCenteredNodeCount: 0,
        patchedNodeCount: 0,
        rejectedCandidates: [],
        passCount: 0,
        keptPassCount: 0,
        rolledBackCount: 0
    };
    const outlierRerouting = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        routedChainCount: 0,
        keptChainCount: 0,
        rejectedChainCount: 0,
        replacedSegmentCount: 0,
        insertedSegmentCount: 0,
        centeredRouteNodeCount: 0,
        rolledBackCount: 0,
        routeChoices: [],
        rejectedChains: []
    };
    const outlierReconnect = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        removedChainCount: 0,
        keptChainCount: 0,
        removedSegmentCount: 0,
        rejectedChainCount: 0,
        rolledBackCount: 0,
        rootedBranchOriginCount: 0,
        routedBranchOriginCount: 0,
        routedPathSegmentCount: 0,
        centeredRouteNodeCount: 0,
        choices: [],
        rollbackDiagnostics: []
    };
    const outlierChainPatch = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        patchedChainCount: 0,
        keptChainCount: 0,
        patchedNodeCount: 0,
        rejectedChainCount: 0,
        rolledBackCount: 0,
        averageShift: 0,
        maxShift: 0,
        choices: [],
        rejectedChains: [],
        rollbackDiagnostics: []
    };
    const outlierReferenceCentering = {
        passCount: 0,
        keptPassCount: 0,
        attemptedNodeCount: 0,
        keptNodeCount: 0,
        rejectedNodeCount: 0,
        rolledBackCount: 0,
        averageShift: 0,
        maxShift: 0,
        choices: [],
        rollbackDiagnostics: []
    };
    const outlierReferenceReconnect = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        removedChainCount: 0,
        keptChainCount: 0,
        removedSegmentCount: 0,
        rejectedChainCount: 0,
        rolledBackCount: 0,
        rootedBranchOriginCount: 0,
        routedBranchOriginCount: 0,
        routedPathSegmentCount: 0,
        centeredRouteNodeCount: 0,
        choices: [],
        rollbackDiagnostics: []
    };
    let outlierShiftSum = 0;
    let outlierClearanceGainSum = 0;
    stageStartedAt = nowMs();
    for (let outlierPass = 0; outlierPass < CENTERLINE_OUTLIER_RELAX_PASSES; outlierPass++) {
        const preOutlierSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preOutlierCyclePruning = { ...cyclePruning };
        const preOutlierInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const passRelaxation = relaxMeasuredOutlierNodesByClearance(allSegments, centering, {
            lumenField: connectorField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh
        });
        outlierRelaxation.passCount++;
        outlierRelaxation.adjustedNodeCount += passRelaxation.adjustedNodeCount;
        outlierRelaxation.rejectedNodeCount += passRelaxation.rejectedNodeCount;
        outlierRelaxation.directCenteredNodeCount += passRelaxation.directCenteredNodeCount || 0;
        outlierRelaxation.directWallFallbackNodeCount += passRelaxation.directWallFallbackNodeCount || 0;
        outlierRelaxation.referenceFieldCenteredNodeCount += passRelaxation.referenceFieldCenteredNodeCount || 0;
        outlierRelaxation.patchedNodeCount += passRelaxation.patchedNodeCount || 0;
        outlierRelaxation.maxShift = Math.max(outlierRelaxation.maxShift, passRelaxation.maxShift || 0);
        outlierRelaxation.maxClearanceGain = Math.max(
            outlierRelaxation.maxClearanceGain,
            passRelaxation.maxClearanceGain || 0
        );
        outlierShiftSum += (passRelaxation.averageShift || 0) * passRelaxation.adjustedNodeCount;
        outlierClearanceGainSum += (passRelaxation.averageClearanceGain || 0) * passRelaxation.adjustedNodeCount;
        if (outlierRelaxation.rejectedCandidates.length < 12) {
            outlierRelaxation.rejectedCandidates.push(...(passRelaxation.rejectedCandidates || []).slice(
                0,
                12 - outlierRelaxation.rejectedCandidates.length
            ));
        }
        if (passRelaxation.adjustedNodeCount <= 0) break;

        const outlierRelaxResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += outlierRelaxResampling.insertedNodeCount;
        if (outlierRelaxResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...outlierRelaxResampling.segments);
        }
        const outlierRelaxCyclePruning = removeCenterlineCycles(allSegments);
        const relaxedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + outlierRelaxCyclePruning.removedSegmentCount,
            nodeCount: outlierRelaxCyclePruning.nodeCount,
            cycleCount: outlierRelaxCyclePruning.cycleCount
        };
        const relaxedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const improvesWorst = (
            relaxedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.005 ||
            relaxedCentering.maxOffset < centering.maxOffset - 0.05
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.035
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.18
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            relaxedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            relaxedCentering.averageOffset <= allowedAverageOffset
        );
        const preservesWorstNormalized = (
            relaxedCentering.maxNormalizedOffset <= centering.maxNormalizedOffset + 0.001
        );
        if (improvesWorst && preservesAverage && preservesWorstNormalized) {
            centering = relaxedCentering;
            cyclePruning = relaxedCyclePruning;
            outlierRelaxation.keptPassCount++;
            outlierRelaxation.keptNodeCount += passRelaxation.adjustedNodeCount;
        } else {
            allSegments.length = 0;
            allSegments.push(...preOutlierSegments);
            cyclePruning = preOutlierCyclePruning;
            postRefinementInsertedResampleNodeCount = preOutlierInsertedResampleNodeCount;
            outlierRelaxation.rolledBackCount++;
            break;
        }
    }

    const reroutePassCount = Math.max(0, Math.round(centerlineOutlierReroutePasses || 0));
    for (let reroutePass = 0; reroutePass < reroutePassCount; reroutePass++) {
        const preRerouteSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preRerouteCyclePruning = { ...cyclePruning };
        const preRerouteInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const passRerouting = rerouteMeasuredOutlierChains(allSegments, centering, {
            lumenField: connectorField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh,
            lumenGeometry: resolvedLumenCast.geometry,
            maxCandidates: centerlineOutlierRerouteMaxCandidates
        });
        outlierRerouting.passCount++;
        outlierRerouting.attemptedChainCount += passRerouting.attemptedChainCount;
        outlierRerouting.routedChainCount += passRerouting.routedChainCount;
        outlierRerouting.rejectedChainCount += passRerouting.rejectedChainCount;
        outlierRerouting.replacedSegmentCount += passRerouting.replacedSegmentCount;
        outlierRerouting.insertedSegmentCount += passRerouting.insertedSegmentCount;
        outlierRerouting.centeredRouteNodeCount += passRerouting.centeredRouteNodeCount;
        if (outlierRerouting.routeChoices.length < 12) {
            outlierRerouting.routeChoices.push(...(passRerouting.routeChoices || []).slice(
                0,
                12 - outlierRerouting.routeChoices.length
            ));
        }
        if (outlierRerouting.rejectedChains.length < 12) {
            outlierRerouting.rejectedChains.push(...(passRerouting.rejectedChains || []).slice(
                0,
                12 - outlierRerouting.rejectedChains.length
            ));
        }
        if (passRerouting.routedChainCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...preRerouteSegments);
            cyclePruning = preRerouteCyclePruning;
            postRefinementInsertedResampleNodeCount = preRerouteInsertedResampleNodeCount;
            break;
        }

        const rerouteResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += rerouteResampling.insertedNodeCount;
        if (rerouteResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...rerouteResampling.segments);
        }
        const rerouteCyclePruning = removeCenterlineCycles(allSegments);
        const reroutedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + rerouteCyclePruning.removedSegmentCount,
            nodeCount: rerouteCyclePruning.nodeCount,
            cycleCount: rerouteCyclePruning.cycleCount
        };
        const reroutedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const improvesWorst = (
            reroutedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.008 ||
            reroutedCentering.maxOffset < centering.maxOffset - 0.08
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            reroutedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            reroutedCentering.averageOffset <= allowedAverageOffset
        );
        if (improvesWorst && preservesAverage) {
            centering = reroutedCentering;
            cyclePruning = reroutedCyclePruning;
            outlierRerouting.keptPassCount++;
            outlierRerouting.keptChainCount += passRerouting.routedChainCount;
        } else {
            allSegments.length = 0;
            allSegments.push(...preRerouteSegments);
            cyclePruning = preRerouteCyclePruning;
            postRefinementInsertedResampleNodeCount = preRerouteInsertedResampleNodeCount;
            outlierRerouting.rolledBackCount++;
            break;
        }
    }

    let chainPatchShiftSum = 0;
    for (let chainPatchPass = 0; chainPatchPass < CENTERLINE_OUTLIER_CHAIN_PATCH_PASSES; chainPatchPass++) {
        const prePatchSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const prePatchCyclePruning = { ...cyclePruning };
        const prePatchInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const chainPatch = patchMeasuredOutlierChains(allSegments, centering, {
            lumenField: connectorField,
            referenceField: lumenField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh,
            lumenGeometry: resolvedLumenCast.geometry,
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        outlierChainPatch.passCount++;
        outlierChainPatch.attemptedChainCount += chainPatch.attemptedChainCount;
        outlierChainPatch.patchedChainCount += chainPatch.patchedChainCount;
        outlierChainPatch.patchedNodeCount += chainPatch.patchedNodeCount;
        outlierChainPatch.rejectedChainCount += chainPatch.rejectedChainCount;
        outlierChainPatch.maxShift = Math.max(outlierChainPatch.maxShift, chainPatch.maxShift || 0);
        chainPatchShiftSum += (chainPatch.averageShift || 0) * chainPatch.patchedNodeCount;
        if (outlierChainPatch.choices.length < 12) {
            outlierChainPatch.choices.push(...(chainPatch.choices || []).slice(
                0,
                12 - outlierChainPatch.choices.length
            ));
        }
        if (outlierChainPatch.rejectedChains.length < 12) {
            outlierChainPatch.rejectedChains.push(...(chainPatch.rejectedChains || []).slice(
                0,
                12 - outlierChainPatch.rejectedChains.length
            ));
        }
        if (chainPatch.patchedChainCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...prePatchSegments);
            cyclePruning = prePatchCyclePruning;
            postRefinementInsertedResampleNodeCount = prePatchInsertedResampleNodeCount;
            break;
        }

        const chainPatchResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += chainPatchResampling.insertedNodeCount;
        if (chainPatchResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...chainPatchResampling.segments);
        }
        const chainPatchCyclePruning = removeCenterlineCycles(allSegments);
        const patchedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + chainPatchCyclePruning.removedSegmentCount,
            nodeCount: chainPatchCyclePruning.nodeCount,
            cycleCount: chainPatchCyclePruning.cycleCount
        };
        const patchedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const improvesWorst = (
            patchedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.006 ||
            patchedCentering.maxOffset < centering.maxOffset - 0.06
        );
        const improvesAverage = (
            patchedCentering.averageNormalizedOffset < centering.averageNormalizedOffset - 0.00005 ||
            patchedCentering.averageOffset < centering.averageOffset - 0.0002
        );
        const preservesWorst = (
            patchedCentering.maxNormalizedOffset <= centering.maxNormalizedOffset + 0.001 &&
            patchedCentering.maxOffset <= centering.maxOffset + 0.02
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            patchedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            patchedCentering.averageOffset <= allowedAverageOffset
        );
        const preservesTree = (
            segmentComponents(allSegments).length === 1 &&
            patchedCyclePruning.cycleCount === 0
        );
        if ((improvesWorst || (improvesAverage && preservesWorst)) && preservesAverage && preservesTree) {
            centering = patchedCentering;
            cyclePruning = patchedCyclePruning;
            outlierChainPatch.keptPassCount++;
            outlierChainPatch.keptChainCount += chainPatch.patchedChainCount;
        } else {
            if (outlierChainPatch.rollbackDiagnostics.length < 8) {
                outlierChainPatch.rollbackDiagnostics.push({
                    improvesWorst,
                    improvesAverage,
                    preservesWorst,
                    preservesAverage,
                    preservesTree,
                    before: {
                        averageOffset: centering.averageOffset,
                        maxOffset: centering.maxOffset,
                        averageNormalizedOffset: centering.averageNormalizedOffset,
                        maxNormalizedOffset: centering.maxNormalizedOffset
                    },
                    after: {
                        averageOffset: patchedCentering.averageOffset,
                        maxOffset: patchedCentering.maxOffset,
                        averageNormalizedOffset: patchedCentering.averageNormalizedOffset,
                        maxNormalizedOffset: patchedCentering.maxNormalizedOffset
                    }
                });
            }
            allSegments.length = 0;
            allSegments.push(...prePatchSegments);
            cyclePruning = prePatchCyclePruning;
            postRefinementInsertedResampleNodeCount = prePatchInsertedResampleNodeCount;
            outlierChainPatch.rolledBackCount++;
            break;
        }
    }
    outlierChainPatch.averageShift = outlierChainPatch.patchedNodeCount
        ? chainPatchShiftSum / outlierChainPatch.patchedNodeCount
        : 0;

    for (let reconnectPass = 0; reconnectPass < CENTERLINE_OUTLIER_RECONNECT_PASSES; reconnectPass++) {
        const preReconnectSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preReconnectCyclePruning = { ...cyclePruning };
        const preReconnectInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const preReconnectBranchRouting = { ...branchRouting };
        const preReconnectRooting = { ...rooting };
        const preReconnectRefinement = { ...refinement };
        const preReconnectRefinementShiftSum = refinementShiftSum;
        const reconnectCut = removeMeasuredOutlierChainsForReconnect(allSegments, centering, {
            lumenField: connectorField
        });
        outlierReconnect.passCount++;
        outlierReconnect.attemptedChainCount += reconnectCut.attemptedChainCount;
        outlierReconnect.removedChainCount += reconnectCut.removedChainCount;
        outlierReconnect.removedSegmentCount += reconnectCut.removedSegmentCount;
        outlierReconnect.rejectedChainCount += reconnectCut.rejectedChainCount;
        if (outlierReconnect.choices.length < 12) {
            outlierReconnect.choices.push(...(reconnectCut.choices || []).slice(
                0,
                12 - outlierReconnect.choices.length
            ));
        }
        if (reconnectCut.removedChainCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...preReconnectSegments);
            cyclePruning = preReconnectCyclePruning;
            postRefinementInsertedResampleNodeCount = preReconnectInsertedResampleNodeCount;
            break;
        }

        const rooted = rootCenterlineComponents(
            allSegments,
            connectorField,
            connectorLumenClearance,
            resolvedWallBvh
        );
        if (rooted.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...rooted.segments);
        }
        const reconnectBranchRouting = routeBranchOriginSegments(
            allSegments,
            connectorField,
            resolvedWallBvh,
            connectorLumenClearance,
            resolvedLumenCast.geometry
        );
        const reconnectResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += reconnectResampling.insertedNodeCount;
        if (reconnectResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...reconnectResampling.segments);
        }
        if (centerlineRefinement) {
            const reconnectRefinement = refineCenterlineSegmentsToLumen(allSegments, resolvedLumenCast.geometry, {
                radialSamples: centerlineRefinementRadialSamples,
                sphereSamples: centerlineRefinementSphereSamples,
                iterations: centerlineRefinementIterations,
                wallBvh: resolvedWallBvh,
                skipRoutedBranchNodes: true
            });
            refinement.refinedNodeCount += reconnectRefinement.refinedNodeCount;
            refinement.skippedNodeCount += reconnectRefinement.skippedNodeCount;
            refinement.failedNodeCount += reconnectRefinement.failedNodeCount;
            refinement.wallRejectedSegmentCount += reconnectRefinement.wallRejectedSegmentCount;
            refinement.wallClampedSegmentCount += reconnectRefinement.wallClampedSegmentCount;
            refinementShiftSum += reconnectRefinement.averageShift * reconnectRefinement.refinedNodeCount;
            refinement.maxShift = Math.max(refinement.maxShift, reconnectRefinement.maxShift);
        }
        const reconnectCyclePruning = removeCenterlineCycles(allSegments);
        const reconnectedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + reconnectCyclePruning.removedSegmentCount,
            nodeCount: reconnectCyclePruning.nodeCount,
            cycleCount: reconnectCyclePruning.cycleCount
        };
        const reconnectedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const componentCount = segmentComponents(allSegments).length;
        const improvesWorst = (
            reconnectedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.008 ||
            reconnectedCentering.maxOffset < centering.maxOffset - 0.08
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            reconnectedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            reconnectedCentering.averageOffset <= allowedAverageOffset
        );
        const dropsOnlyTinyArtifactTail = (
            rooted.discardedSegmentCount > 0 &&
            rooted.discardedSegmentCount <= 3 &&
            reconnectCut.removedSegmentCount >= 6 &&
            reconnectedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.12 &&
            reconnectedCentering.averageOffset < centering.averageOffset - 0.02
        );
        const preservesTree = (
            componentCount === 1 &&
            (rooted.discardedSegmentCount === 0 || dropsOnlyTinyArtifactTail) &&
            reconnectedCyclePruning.cycleCount === 0
        );
        if (improvesWorst && preservesAverage && preservesTree) {
            centering = reconnectedCentering;
            cyclePruning = reconnectedCyclePruning;
            rooting.addedBranchOriginCount += rooted.addedBranchOriginCount;
            rooting.discardedComponentCount += rooted.discardedComponentCount;
            rooting.discardedSegmentCount += rooted.discardedSegmentCount;
            branchRouting.routedSegmentCount += reconnectBranchRouting.routedSegmentCount;
            branchRouting.routedPathSegmentCount += reconnectBranchRouting.routedPathSegmentCount;
            branchRouting.gridRouteCount += reconnectBranchRouting.gridRouteCount;
            branchRouting.gridRoutePathSegmentCount += reconnectBranchRouting.gridRoutePathSegmentCount;
            branchRouting.failedRouteCount += reconnectBranchRouting.failedRouteCount;
            branchRouting.relaxedRouteNodeCount += reconnectBranchRouting.relaxedRouteNodeCount;
            branchRouting.centeredRouteNodeCount += reconnectBranchRouting.centeredRouteNodeCount;
            branchRouting.totalRouteClearanceGain += reconnectBranchRouting.totalRouteClearanceGain;
            branchRouting.maxRouteClearanceGain = Math.max(
                branchRouting.maxRouteClearanceGain,
                reconnectBranchRouting.maxRouteClearanceGain
            );
            branchRouting.routeChoiceDiagnostics.push(...(reconnectBranchRouting.routeChoiceDiagnostics || []));
            outlierReconnect.keptPassCount++;
            outlierReconnect.keptChainCount += reconnectCut.removedChainCount;
            outlierReconnect.rootedBranchOriginCount += rooted.addedBranchOriginCount;
            outlierReconnect.routedBranchOriginCount += reconnectBranchRouting.routedSegmentCount;
            outlierReconnect.routedPathSegmentCount += reconnectBranchRouting.routedPathSegmentCount;
            outlierReconnect.centeredRouteNodeCount += reconnectBranchRouting.centeredRouteNodeCount;
        } else {
            if (outlierReconnect.rollbackDiagnostics.length < 8) {
                outlierReconnect.rollbackDiagnostics.push({
                    improvesWorst,
                    preservesAverage,
                    preservesTree,
                    componentCount,
                    rootedDiscardedSegmentCount: rooted.discardedSegmentCount,
                    rootedDiscardedComponentCount: rooted.discardedComponentCount,
                    rootedBranchOriginCount: rooted.addedBranchOriginCount,
                    routedBranchOriginCount: reconnectBranchRouting.routedSegmentCount,
                    routedPathSegmentCount: reconnectBranchRouting.routedPathSegmentCount,
                    cycleCount: reconnectedCyclePruning.cycleCount,
                    before: {
                        averageOffset: centering.averageOffset,
                        maxOffset: centering.maxOffset,
                        averageNormalizedOffset: centering.averageNormalizedOffset,
                        maxNormalizedOffset: centering.maxNormalizedOffset
                    },
                    after: {
                        averageOffset: reconnectedCentering.averageOffset,
                        maxOffset: reconnectedCentering.maxOffset,
                        averageNormalizedOffset: reconnectedCentering.averageNormalizedOffset,
                        maxNormalizedOffset: reconnectedCentering.maxNormalizedOffset
                    }
                });
            }
            allSegments.length = 0;
            allSegments.push(...preReconnectSegments);
            cyclePruning = preReconnectCyclePruning;
            postRefinementInsertedResampleNodeCount = preReconnectInsertedResampleNodeCount;
            Object.assign(branchRouting, preReconnectBranchRouting);
            Object.assign(rooting, preReconnectRooting);
            Object.assign(refinement, preReconnectRefinement);
            refinementShiftSum = preReconnectRefinementShiftSum;
            outlierReconnect.rolledBackCount++;
            break;
        }
    }

    for (let referenceReconnectPass = 0;
        referenceReconnectPass < CENTERLINE_OUTLIER_REFERENCE_RECONNECT_PASSES;
        referenceReconnectPass++
    ) {
        if (!lumenField?.query) break;
        const preReconnectSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preReconnectCyclePruning = { ...cyclePruning };
        const preReconnectInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const preReconnectBranchRouting = { ...branchRouting };
        const preReconnectRooting = { ...rooting };
        const reconnectCut = removeMeasuredOutlierChainsForReconnect(allSegments, centering, {
            lumenField,
            maxCandidates: 1,
            minOffset: 2.4,
            minNormalizedOffset: 0.55
        });
        outlierReferenceReconnect.passCount++;
        outlierReferenceReconnect.attemptedChainCount += reconnectCut.attemptedChainCount;
        outlierReferenceReconnect.removedChainCount += reconnectCut.removedChainCount;
        outlierReferenceReconnect.removedSegmentCount += reconnectCut.removedSegmentCount;
        outlierReferenceReconnect.rejectedChainCount += reconnectCut.rejectedChainCount;
        if (outlierReferenceReconnect.choices.length < 12) {
            outlierReferenceReconnect.choices.push(...(reconnectCut.choices || []).slice(
                0,
                12 - outlierReferenceReconnect.choices.length
            ));
        }
        if (reconnectCut.removedChainCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...preReconnectSegments);
            cyclePruning = preReconnectCyclePruning;
            postRefinementInsertedResampleNodeCount = preReconnectInsertedResampleNodeCount;
            break;
        }

        const rooted = rootCenterlineComponents(
            allSegments,
            lumenField,
            connectorLumenClearance,
            resolvedWallBvh
        );
        if (rooted.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...rooted.segments);
        }
        const reconnectBranchRouting = routeBranchOriginSegments(
            allSegments,
            lumenField,
            resolvedWallBvh,
            connectorLumenClearance,
            resolvedLumenCast.geometry
        );
        const reconnectResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += reconnectResampling.insertedNodeCount;
        if (reconnectResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...reconnectResampling.segments);
        }
        const reconnectCyclePruning = removeCenterlineCycles(allSegments);
        const reconnectedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + reconnectCyclePruning.removedSegmentCount,
            nodeCount: reconnectCyclePruning.nodeCount,
            cycleCount: reconnectCyclePruning.cycleCount
        };
        const reconnectedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const componentCount = segmentComponents(allSegments).length;
        const improvesWorst = (
            reconnectedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.008 ||
            reconnectedCentering.maxOffset < centering.maxOffset - 0.08
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            reconnectedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            reconnectedCentering.averageOffset <= allowedAverageOffset
        );
        const dropsOnlyTinyArtifactTail = (
            rooted.discardedSegmentCount > 0 &&
            rooted.discardedSegmentCount <= 3 &&
            reconnectCut.removedSegmentCount >= 6 &&
            reconnectedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.08 &&
            reconnectedCentering.averageOffset < centering.averageOffset - 0.0005
        );
        const preservesTree = (
            componentCount === 1 &&
            (rooted.discardedSegmentCount === 0 || dropsOnlyTinyArtifactTail) &&
            reconnectedCyclePruning.cycleCount === 0
        );
        if (improvesWorst && preservesAverage && preservesTree) {
            centering = reconnectedCentering;
            cyclePruning = reconnectedCyclePruning;
            rooting.addedBranchOriginCount += rooted.addedBranchOriginCount;
            rooting.discardedComponentCount += rooted.discardedComponentCount;
            rooting.discardedSegmentCount += rooted.discardedSegmentCount;
            branchRouting.routedSegmentCount += reconnectBranchRouting.routedSegmentCount;
            branchRouting.routedPathSegmentCount += reconnectBranchRouting.routedPathSegmentCount;
            branchRouting.gridRouteCount += reconnectBranchRouting.gridRouteCount;
            branchRouting.gridRoutePathSegmentCount += reconnectBranchRouting.gridRoutePathSegmentCount;
            branchRouting.failedRouteCount += reconnectBranchRouting.failedRouteCount;
            branchRouting.relaxedRouteNodeCount += reconnectBranchRouting.relaxedRouteNodeCount;
            branchRouting.centeredRouteNodeCount += reconnectBranchRouting.centeredRouteNodeCount;
            branchRouting.totalRouteClearanceGain += reconnectBranchRouting.totalRouteClearanceGain;
            branchRouting.maxRouteClearanceGain = Math.max(
                branchRouting.maxRouteClearanceGain,
                reconnectBranchRouting.maxRouteClearanceGain
            );
            branchRouting.routeChoiceDiagnostics.push(...(reconnectBranchRouting.routeChoiceDiagnostics || []));
            outlierReferenceReconnect.keptPassCount++;
            outlierReferenceReconnect.keptChainCount += reconnectCut.removedChainCount;
            outlierReferenceReconnect.rootedBranchOriginCount += rooted.addedBranchOriginCount;
            outlierReferenceReconnect.routedBranchOriginCount += reconnectBranchRouting.routedSegmentCount;
            outlierReferenceReconnect.routedPathSegmentCount += reconnectBranchRouting.routedPathSegmentCount;
            outlierReferenceReconnect.centeredRouteNodeCount += reconnectBranchRouting.centeredRouteNodeCount;
        } else {
            if (outlierReferenceReconnect.rollbackDiagnostics.length < 8) {
                outlierReferenceReconnect.rollbackDiagnostics.push({
                    improvesWorst,
                    preservesAverage,
                    preservesTree,
                    componentCount,
                    rootedDiscardedSegmentCount: rooted.discardedSegmentCount,
                    routedBranchOriginCount: reconnectBranchRouting.routedSegmentCount,
                    routedPathSegmentCount: reconnectBranchRouting.routedPathSegmentCount,
                    cycleCount: reconnectedCyclePruning.cycleCount,
                    before: {
                        averageOffset: centering.averageOffset,
                        maxOffset: centering.maxOffset,
                        averageNormalizedOffset: centering.averageNormalizedOffset,
                        maxNormalizedOffset: centering.maxNormalizedOffset
                    },
                    after: {
                        averageOffset: reconnectedCentering.averageOffset,
                        maxOffset: reconnectedCentering.maxOffset,
                        averageNormalizedOffset: reconnectedCentering.averageNormalizedOffset,
                        maxNormalizedOffset: reconnectedCentering.maxNormalizedOffset
                    }
                });
            }
            allSegments.length = 0;
            allSegments.push(...preReconnectSegments);
            cyclePruning = preReconnectCyclePruning;
            postRefinementInsertedResampleNodeCount = preReconnectInsertedResampleNodeCount;
            Object.assign(branchRouting, preReconnectBranchRouting);
            Object.assign(rooting, preReconnectRooting);
            outlierReferenceReconnect.rolledBackCount++;
            break;
        }
    }

    for (let referencePass = 0; referencePass < CENTERLINE_OUTLIER_REFERENCE_FIELD_PASSES; referencePass++) {
        const preReferenceSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preReferenceCyclePruning = { ...cyclePruning };
        const preReferenceInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const passReferenceCentering = relaxMeasuredOutlierNodesByClearance(allSegments, centering, {
            lumenField: connectorField,
            referenceField: lumenField,
            referenceFieldOnly: true,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh
        });
        outlierReferenceCentering.passCount++;
        outlierReferenceCentering.attemptedNodeCount += passReferenceCentering.adjustedNodeCount;
        outlierReferenceCentering.rejectedNodeCount += passReferenceCentering.rejectedNodeCount;
        outlierReferenceCentering.maxShift = Math.max(
            outlierReferenceCentering.maxShift,
            passReferenceCentering.maxShift || 0
        );
        if (passReferenceCentering.adjustedNodeCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...preReferenceSegments);
            cyclePruning = preReferenceCyclePruning;
            postRefinementInsertedResampleNodeCount = preReferenceInsertedResampleNodeCount;
            break;
        }

        const referenceResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += referenceResampling.insertedNodeCount;
        if (referenceResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...referenceResampling.segments);
        }
        const referenceCyclePruning = removeCenterlineCycles(allSegments);
        const referencedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + referenceCyclePruning.removedSegmentCount,
            nodeCount: referenceCyclePruning.nodeCount,
            cycleCount: referenceCyclePruning.cycleCount
        };
        const referencedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const improvesWorst = (
            referencedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.006 ||
            referencedCentering.maxOffset < centering.maxOffset - 0.06
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            referencedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            referencedCentering.averageOffset <= allowedAverageOffset
        );
        const preservesTree = (
            segmentComponents(allSegments).length === 1 &&
            referencedCyclePruning.cycleCount === 0
        );
        if (improvesWorst && preservesAverage && preservesTree) {
            centering = referencedCentering;
            cyclePruning = referencedCyclePruning;
            outlierReferenceCentering.keptPassCount++;
            outlierReferenceCentering.keptNodeCount += passReferenceCentering.adjustedNodeCount;
            outlierReferenceCentering.averageShift = passReferenceCentering.averageShift || 0;
        } else {
            if (outlierReferenceCentering.rollbackDiagnostics.length < 8) {
                outlierReferenceCentering.rollbackDiagnostics.push({
                    improvesWorst,
                    preservesAverage,
                    preservesTree,
                    before: {
                        averageOffset: centering.averageOffset,
                        maxOffset: centering.maxOffset,
                        averageNormalizedOffset: centering.averageNormalizedOffset,
                        maxNormalizedOffset: centering.maxNormalizedOffset
                    },
                    after: {
                        averageOffset: referencedCentering.averageOffset,
                        maxOffset: referencedCentering.maxOffset,
                        averageNormalizedOffset: referencedCentering.averageNormalizedOffset,
                        maxNormalizedOffset: referencedCentering.maxNormalizedOffset
                    }
                });
            }
            allSegments.length = 0;
            allSegments.push(...preReferenceSegments);
            cyclePruning = preReferenceCyclePruning;
            postRefinementInsertedResampleNodeCount = preReferenceInsertedResampleNodeCount;
            outlierReferenceCentering.rolledBackCount++;
            break;
        }
    }

    for (let latePatchPass = 0; latePatchPass < CENTERLINE_OUTLIER_LATE_CHAIN_PATCH_PASSES; latePatchPass++) {
        const preLatePatchSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const preLatePatchCyclePruning = { ...cyclePruning };
        const preLatePatchInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
        const lateChainPatch = patchMeasuredOutlierChains(allSegments, centering, {
            lumenField: connectorField,
            referenceField: lumenField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh,
            lumenGeometry: resolvedLumenCast.geometry,
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        outlierChainPatch.passCount++;
        outlierChainPatch.attemptedChainCount += lateChainPatch.attemptedChainCount;
        outlierChainPatch.patchedChainCount += lateChainPatch.patchedChainCount;
        outlierChainPatch.patchedNodeCount += lateChainPatch.patchedNodeCount;
        outlierChainPatch.rejectedChainCount += lateChainPatch.rejectedChainCount;
        outlierChainPatch.maxShift = Math.max(outlierChainPatch.maxShift, lateChainPatch.maxShift || 0);
        chainPatchShiftSum += (lateChainPatch.averageShift || 0) * lateChainPatch.patchedNodeCount;
        if (outlierChainPatch.choices.length < 12) {
            outlierChainPatch.choices.push(...(lateChainPatch.choices || []).slice(
                0,
                12 - outlierChainPatch.choices.length
            ));
        }
        if (outlierChainPatch.rejectedChains.length < 12) {
            outlierChainPatch.rejectedChains.push(...(lateChainPatch.rejectedChains || []).slice(
                0,
                12 - outlierChainPatch.rejectedChains.length
            ));
        }
        if (lateChainPatch.patchedChainCount <= 0) {
            allSegments.length = 0;
            allSegments.push(...preLatePatchSegments);
            cyclePruning = preLatePatchCyclePruning;
            postRefinementInsertedResampleNodeCount = preLatePatchInsertedResampleNodeCount;
            break;
        }

        const latePatchResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += latePatchResampling.insertedNodeCount;
        if (latePatchResampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...latePatchResampling.segments);
        }
        const latePatchCyclePruning = removeCenterlineCycles(allSegments);
        const latePatchedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + latePatchCyclePruning.removedSegmentCount,
            nodeCount: latePatchCyclePruning.nodeCount,
            cycleCount: latePatchCyclePruning.cycleCount
        };
        const latePatchedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const improvesWorst = (
            latePatchedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.001 ||
            latePatchedCentering.maxOffset < centering.maxOffset - 0.03
        );
        const improvesAverage = (
            latePatchedCentering.averageNormalizedOffset < centering.averageNormalizedOffset - 0.00005 ||
            latePatchedCentering.averageOffset < centering.averageOffset - 0.0002
        );
        const preservesWorst = (
            latePatchedCentering.maxNormalizedOffset <= centering.maxNormalizedOffset + 0.001 &&
            latePatchedCentering.maxOffset <= centering.maxOffset + 0.02
        );
        const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
            ? 0.036
            : centering.averageNormalizedOffset + 0.002;
        const allowedAverageOffset = centering.averageOffset <= 0.18
            ? 0.19
            : centering.averageOffset + 0.02;
        const preservesAverage = (
            latePatchedCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
            latePatchedCentering.averageOffset <= allowedAverageOffset
        );
        const preservesTree = (
            segmentComponents(allSegments).length === 1 &&
            latePatchedCyclePruning.cycleCount === 0
        );
        if ((improvesWorst || (improvesAverage && preservesWorst)) && preservesAverage && preservesTree) {
            centering = latePatchedCentering;
            cyclePruning = latePatchedCyclePruning;
            outlierChainPatch.keptPassCount++;
            outlierChainPatch.keptChainCount += lateChainPatch.patchedChainCount;
        } else {
            if (outlierChainPatch.rollbackDiagnostics.length < 8) {
                outlierChainPatch.rollbackDiagnostics.push({
                    late: true,
                    improvesWorst,
                    improvesAverage,
                    preservesWorst,
                    preservesAverage,
                    preservesTree,
                    before: {
                        averageOffset: centering.averageOffset,
                        maxOffset: centering.maxOffset,
                        averageNormalizedOffset: centering.averageNormalizedOffset,
                        maxNormalizedOffset: centering.maxNormalizedOffset
                    },
                    after: {
                        averageOffset: latePatchedCentering.averageOffset,
                        maxOffset: latePatchedCentering.maxOffset,
                        averageNormalizedOffset: latePatchedCentering.averageNormalizedOffset,
                        maxNormalizedOffset: latePatchedCentering.maxNormalizedOffset
                    }
                });
            }
            allSegments.length = 0;
            allSegments.push(...preLatePatchSegments);
            cyclePruning = preLatePatchCyclePruning;
            postRefinementInsertedResampleNodeCount = preLatePatchInsertedResampleNodeCount;
            outlierChainPatch.rolledBackCount++;
            break;
        }
    }
    outlierChainPatch.averageShift = outlierChainPatch.patchedNodeCount
        ? chainPatchShiftSum / outlierChainPatch.patchedNodeCount
        : 0;

    if (outlierRelaxation.rolledBackCount === 0 && CENTERLINE_OUTLIER_PATCH_PASSES > 0) {
        for (let patchPass = 0; patchPass < CENTERLINE_OUTLIER_PATCH_PASSES; patchPass++) {
            const prePatchSegments = allSegments.map(segment => ({
                ...segment,
                start: segment.start.clone(),
                end: segment.end.clone()
            }));
            const prePatchCyclePruning = { ...cyclePruning };
            const prePatchInsertedResampleNodeCount = postRefinementInsertedResampleNodeCount;
            const patchRelaxation = relaxMeasuredOutlierNodesByClearance(allSegments, centering, {
                lumenField: connectorField,
                connectorLumenClearance,
                wallBvh: resolvedWallBvh,
                enablePatches: true
            });
            if (patchRelaxation.adjustedNodeCount <= 0 || patchRelaxation.patchedNodeCount <= 0) {
                allSegments.length = 0;
                allSegments.push(...prePatchSegments);
                cyclePruning = prePatchCyclePruning;
                postRefinementInsertedResampleNodeCount = prePatchInsertedResampleNodeCount;
                break;
            }

            const patchResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
            postRefinementInsertedResampleNodeCount += patchResampling.insertedNodeCount;
            if (patchResampling.segments !== allSegments) {
                allSegments.length = 0;
                allSegments.push(...patchResampling.segments);
            }
            const patchCyclePruning = removeCenterlineCycles(allSegments);
            const relaxedCyclePruning = {
                removedSegmentCount: cyclePruning.removedSegmentCount + patchCyclePruning.removedSegmentCount,
                nodeCount: patchCyclePruning.nodeCount,
                cycleCount: patchCyclePruning.cycleCount
            };
            const patchCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
                radialSamples: centerlineRefinementRadialSamples,
                sphereSamples: centerlineRefinementSphereSamples
            });
            const improvesWorst = (
                patchCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.005 ||
                patchCentering.maxOffset < centering.maxOffset - 0.05
            );
            const allowedAverageNormalizedOffset = centering.averageNormalizedOffset <= 0.035
                ? 0.035
                : centering.averageNormalizedOffset + 0.002;
            const allowedAverageOffset = centering.averageOffset <= 0.18
                ? 0.18
                : centering.averageOffset + 0.02;
            const preservesAverage = (
                patchCentering.averageNormalizedOffset <= allowedAverageNormalizedOffset &&
                patchCentering.averageOffset <= allowedAverageOffset
            );
            if (improvesWorst && preservesAverage) {
                centering = patchCentering;
                cyclePruning = relaxedCyclePruning;
                outlierRelaxation.adjustedNodeCount += patchRelaxation.adjustedNodeCount;
                outlierRelaxation.rejectedNodeCount += patchRelaxation.rejectedNodeCount;
                outlierRelaxation.directCenteredNodeCount += patchRelaxation.directCenteredNodeCount || 0;
                outlierRelaxation.directWallFallbackNodeCount += patchRelaxation.directWallFallbackNodeCount || 0;
                outlierRelaxation.referenceFieldCenteredNodeCount += patchRelaxation.referenceFieldCenteredNodeCount || 0;
                outlierRelaxation.patchedNodeCount += patchRelaxation.patchedNodeCount || 0;
                outlierRelaxation.maxShift = Math.max(outlierRelaxation.maxShift, patchRelaxation.maxShift || 0);
                outlierRelaxation.maxClearanceGain = Math.max(
                    outlierRelaxation.maxClearanceGain,
                    patchRelaxation.maxClearanceGain || 0
                );
                outlierShiftSum += (patchRelaxation.averageShift || 0) * patchRelaxation.adjustedNodeCount;
                outlierClearanceGainSum +=
                    (patchRelaxation.averageClearanceGain || 0) * patchRelaxation.adjustedNodeCount;
                outlierRelaxation.keptPassCount++;
                outlierRelaxation.keptNodeCount += patchRelaxation.adjustedNodeCount;
            } else {
                allSegments.length = 0;
                allSegments.push(...prePatchSegments);
                cyclePruning = prePatchCyclePruning;
                postRefinementInsertedResampleNodeCount = prePatchInsertedResampleNodeCount;
                outlierRelaxation.rolledBackCount++;
                break;
            }
        }
    }

    recordTiming('outlierRelaxMs', stageStartedAt);
    stageStartedAt = nowMs();
    const topologyBeforeCleanup = measureCenterlineTopology(allSegments);
    const initialBacktrackSimplification = simplifyCenterlineBacktracks(
        allSegments,
        connectorField,
        resolvedWallBvh,
        connectorLumenClearance
    );
    const invalidChainRerouting = rerouteInvalidCenterlineChains(
        allSegments,
        connectorField,
        resolvedWallBvh,
        connectorLumenClearance,
        resolvedLumenCast.geometry
    );
    const finalBacktrackSimplification = simplifyCenterlineBacktracks(
        allSegments,
        connectorField,
        resolvedWallBvh,
        connectorLumenClearance
    );
    const cleanupResampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
    postRefinementInsertedResampleNodeCount += cleanupResampling.insertedNodeCount;
    if (cleanupResampling.segments !== allSegments) {
        allSegments.length = 0;
        allSegments.push(...cleanupResampling.segments);
    }
    const cleanupCyclePruning = removeCenterlineCycles(allSegments);
    cyclePruning = {
        removedSegmentCount: cyclePruning.removedSegmentCount + cleanupCyclePruning.removedSegmentCount,
        nodeCount: cleanupCyclePruning.nodeCount,
        cycleCount: cleanupCyclePruning.cycleCount
    };
    centering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
        radialSamples: centerlineRefinementRadialSamples,
        sphereSamples: centerlineRefinementSphereSamples
    });
    const finalChainPatch = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        keptChainCount: 0,
        rejectedChainCount: 0,
        rolledBackCount: 0,
        choices: [],
        rollbackDiagnostics: []
    };
    let remainingInvalidSegmentCount = invalidCenterlineSegments(
        allSegments,
        connectorField,
        resolvedWallBvh,
        connectorLumenClearance
    ).length;
    for (let pass = 0; pass < CENTERLINE_FINAL_CHAIN_PATCH_PASSES; pass++) {
        const prePatchSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const prePatchCyclePruning = { ...cyclePruning };
        const prePatchCentering = centering;
        const prePatchInvalidSegmentCount = remainingInvalidSegmentCount;
        const patch = patchMeasuredOutlierChains(allSegments, centering, {
            lumenField: connectorField,
            referenceField: lumenField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh,
            lumenGeometry: resolvedLumenCast.geometry,
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        finalChainPatch.passCount++;
        finalChainPatch.attemptedChainCount += patch.attemptedChainCount;
        finalChainPatch.rejectedChainCount += patch.rejectedChainCount;
        if (patch.choices?.length && finalChainPatch.choices.length < 12) {
            finalChainPatch.choices.push(...patch.choices.slice(
                0,
                12 - finalChainPatch.choices.length
            ));
        }
        if (patch.patchedChainCount <= 0) break;

        const resampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += resampling.insertedNodeCount;
        if (resampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampling.segments);
        }
        const patchCyclePruning = removeCenterlineCycles(allSegments);
        const patchedCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + patchCyclePruning.removedSegmentCount,
            nodeCount: patchCyclePruning.nodeCount,
            cycleCount: patchCyclePruning.cycleCount
        };
        const patchedCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const patchedInvalidSegmentCount = invalidCenterlineSegments(
            allSegments,
            connectorField,
            resolvedWallBvh,
            connectorLumenClearance
        ).length;
        const improvesWorst = (
            patchedCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.001 ||
            patchedCentering.maxOffset < centering.maxOffset - 0.03
        );
        const improvesAverage = (
            patchedCentering.averageNormalizedOffset < centering.averageNormalizedOffset - 0.00003 ||
            patchedCentering.averageOffset < centering.averageOffset - 0.0002
        );
        const preservesWorst = (
            patchedCentering.maxNormalizedOffset <= centering.maxNormalizedOffset + 0.001 &&
            patchedCentering.maxOffset <= centering.maxOffset + 0.02
        );
        const preservesAverage = (
            patchedCentering.averageNormalizedOffset <= centering.averageNormalizedOffset + 0.0008 &&
            patchedCentering.averageOffset <= centering.averageOffset + 0.004
        );
        const preservesTree = (
            segmentComponents(allSegments).length === 1 &&
            patchedCyclePruning.cycleCount === 0
        );
        const preservesValidity = patchedInvalidSegmentCount <= prePatchInvalidSegmentCount;
        if (
            (improvesWorst || (improvesAverage && preservesWorst)) &&
            preservesAverage &&
            preservesTree &&
            preservesValidity
        ) {
            centering = patchedCentering;
            cyclePruning = patchedCyclePruning;
            remainingInvalidSegmentCount = patchedInvalidSegmentCount;
            finalChainPatch.keptPassCount++;
            finalChainPatch.keptChainCount += patch.patchedChainCount;
            continue;
        }

        allSegments.length = 0;
        allSegments.push(...prePatchSegments);
        cyclePruning = prePatchCyclePruning;
        centering = prePatchCentering;
        remainingInvalidSegmentCount = prePatchInvalidSegmentCount;
        finalChainPatch.rolledBackCount++;
        if (finalChainPatch.rollbackDiagnostics.length < 8) {
            finalChainPatch.rollbackDiagnostics.push({
                improvesWorst,
                improvesAverage,
                preservesWorst,
                preservesAverage,
                preservesTree,
                preservesValidity,
                before: {
                    maxNormalizedOffset: prePatchCentering.maxNormalizedOffset,
                    averageNormalizedOffset: prePatchCentering.averageNormalizedOffset,
                    invalidSegmentCount: prePatchInvalidSegmentCount
                },
                after: {
                    maxNormalizedOffset: patchedCentering.maxNormalizedOffset,
                    averageNormalizedOffset: patchedCentering.averageNormalizedOffset,
                    invalidSegmentCount: patchedInvalidSegmentCount
                }
            });
        }
        break;
    }
    const finalRerouting = {
        passCount: 0,
        keptPassCount: 0,
        attemptedChainCount: 0,
        routedChainCount: 0,
        keptChainCount: 0,
        rejectedChainCount: 0,
        rolledBackCount: 0,
        choices: [],
        rollbackDiagnostics: []
    };
    for (let pass = 0; pass < CENTERLINE_FINAL_REROUTE_PASSES; pass++) {
        const beforeSegments = allSegments.map(segment => ({
            ...segment,
            start: segment.start.clone(),
            end: segment.end.clone()
        }));
        const beforeCyclePruning = { ...cyclePruning };
        const beforeCentering = centering;
        const beforeInvalidSegmentCount = remainingInvalidSegmentCount;
        const rerouted = rerouteMeasuredOutlierChains(allSegments, centering, {
            lumenField: connectorField,
            connectorLumenClearance,
            wallBvh: resolvedWallBvh,
            lumenGeometry: resolvedLumenCast.geometry,
            maxCandidates: 12,
            maxRoutedChains: 1
        });
        finalRerouting.passCount++;
        finalRerouting.attemptedChainCount += rerouted.attemptedChainCount;
        finalRerouting.routedChainCount += rerouted.routedChainCount;
        finalRerouting.rejectedChainCount += rerouted.rejectedChainCount;
        if (rerouted.routeChoices?.length && finalRerouting.choices.length < 12) {
            finalRerouting.choices.push(...rerouted.routeChoices.slice(
                0,
                12 - finalRerouting.choices.length
            ));
        }
        if (rerouted.routedChainCount <= 0) break;

        const resampling = resampleCenterlineSegments(allSegments, centerlineNodeSpacing);
        postRefinementInsertedResampleNodeCount += resampling.insertedNodeCount;
        if (resampling.segments !== allSegments) {
            allSegments.length = 0;
            allSegments.push(...resampling.segments);
        }
        const passCyclePruning = removeCenterlineCycles(allSegments);
        const nextCyclePruning = {
            removedSegmentCount: cyclePruning.removedSegmentCount + passCyclePruning.removedSegmentCount,
            nodeCount: passCyclePruning.nodeCount,
            cycleCount: passCyclePruning.cycleCount
        };
        const nextCentering = measureCenterlineCentering(allSegments, resolvedLumenCast.geometry, {
            radialSamples: centerlineRefinementRadialSamples,
            sphereSamples: centerlineRefinementSphereSamples
        });
        const nextInvalidSegmentCount = invalidCenterlineSegments(
            allSegments,
            connectorField,
            resolvedWallBvh,
            connectorLumenClearance
        ).length;
        const improvesWorst = (
            nextCentering.maxNormalizedOffset < centering.maxNormalizedOffset - 0.003 ||
            nextCentering.maxOffset < centering.maxOffset - 0.08
        );
        const preservesAverage = (
            nextCentering.averageNormalizedOffset <= centering.averageNormalizedOffset + 0.001 &&
            nextCentering.averageOffset <= centering.averageOffset + 0.006
        );
        const preservesTree = (
            segmentComponents(allSegments).length === 1 &&
            nextCyclePruning.cycleCount === 0
        );
        const preservesValidity = nextInvalidSegmentCount <= beforeInvalidSegmentCount;
        if (improvesWorst && preservesAverage && preservesTree && preservesValidity) {
            centering = nextCentering;
            cyclePruning = nextCyclePruning;
            remainingInvalidSegmentCount = nextInvalidSegmentCount;
            finalRerouting.keptPassCount++;
            finalRerouting.keptChainCount += rerouted.routedChainCount;
            continue;
        }

        allSegments.length = 0;
        allSegments.push(...beforeSegments);
        cyclePruning = beforeCyclePruning;
        centering = beforeCentering;
        remainingInvalidSegmentCount = beforeInvalidSegmentCount;
        finalRerouting.rolledBackCount++;
        finalRerouting.rollbackDiagnostics.push({
            improvesWorst,
            preservesAverage,
            preservesTree,
            preservesValidity,
            before: {
                maxOffset: beforeCentering.maxOffset,
                maxNormalizedOffset: beforeCentering.maxNormalizedOffset
            },
            after: {
                maxOffset: nextCentering.maxOffset,
                maxNormalizedOffset: nextCentering.maxNormalizedOffset
            }
        });
        break;
    }
    const topologyAfterCleanup = measureCenterlineTopology(allSegments);
    recordTiming('topologyCleanupMs', stageStartedAt);
    outlierRelaxation.averageShift = outlierRelaxation.adjustedNodeCount
        ? outlierShiftSum / outlierRelaxation.adjustedNodeCount
        : 0;
    outlierRelaxation.averageClearanceGain = outlierRelaxation.adjustedNodeCount
        ? outlierClearanceGainSum / outlierRelaxation.adjustedNodeCount
        : 0;
    const centerlineCoverage = measureCenterlineSampleCoverage(
        primaryCoverageSamples,
        allSegments
    );
    const debugSegments = thinDebugPositions(debugPositions);
    const finalComponents = segmentComponents(allSegments);
    const finalStubSegmentCount = allSegments.filter(segment => segment.source.startsWith('stl-slice-stub')).length;
    const finalEdgeCount = allSegments.length - finalStubSegmentCount;
    timings.totalMs = nowMs() - buildStartedAt;
    allSegments.diagnostics = {
        source: 'lumen-cast-centerline',
        timings,
        axes: processedAxes.map(axis => axis.id).join(','),
        axisDiagnostics,
        sliceCount: totalSlices,
        nodeCount: nodeOffset,
        edgeCount: finalEdgeCount,
        stubSegmentCount: finalStubSegmentCount,
        isolatedNodeCount: finalComponents.length > 1 ? finalStubSegmentCount : 0,
        componentCount: finalComponents.length,
        candidateComponentCount: tree.candidateComponentCount,
        acceptedComponentCount: tree.acceptedComponentCount,
        coveredDuplicateComponentCount: tree.coveredDuplicateComponentCount,
        primaryAxisRescueCandidateComponentCount:
            primaryAxisRescue.candidateComponentCount,
        primaryAxisRescueAttemptedComponentCount:
            primaryAxisRescue.attemptedComponentCount,
        primaryAxisRescuedComponentCount:
            primaryAxisRescue.rescuedComponentCount,
        primaryAxisRescueFailedComponentCount:
            primaryAxisRescue.failedComponentCount,
        primaryAxisRescueAddedOriginSegmentCount:
            primaryAxisRescue.addedOriginSegmentCount,
        primaryAxisRescueAddedComponentSegmentCount:
            primaryAxisRescue.addedComponentSegmentCount,
        primaryAxisRescueChoices: primaryAxisRescue.choices,
        primaryAxisRescueFailures: primaryAxisRescue.failures,
        rootedBranchOriginCount: rooting.addedBranchOriginCount,
        discardedDisconnectedComponentCount: rooting.discardedComponentCount,
        discardedDisconnectedSegmentCount: rooting.discardedSegmentCount,
        uncoveredNodeCount: centerlineCoverage.uncoveredSampleCount,
        centerlineCoverage,
        splitNodeCount: totalSplits,
        mergeNodeCount: totalMerges,
        contourCount: totalContours,
        debugSegmentCount: debugSegments.length / 6,
        requestedSliceCount: targetSliceCount,
        sliceSpacing: baseSliceSpacing,
        centerlineNodeSpacing,
        centerlineMinLumenArea: effectiveCenterlineMinArea,
        centerlineMinCompactness: effectiveCenterlineMinCompactness,
        preResampleSegmentCount: resampling.originalSegmentCount,
        resampledSegmentCount: allSegments.length,
        insertedResampleNodeCount: resampling.insertedNodeCount +
            postRefinementInsertedResampleNodeCount,
        postRefinementInsertedResampleNodeCount,
        centerlineRefinement,
        centerlineRefinementRadialSamples,
        centerlineRefinementSphereSamples,
        centerlineRefinementIterations,
        centerlineRefinementPasses: refinementPassCount,
        centerlineRefinedNodeCount: refinement.refinedNodeCount,
        centerlineRefinementSkippedNodeCount: refinement.skippedNodeCount,
        centerlineRefinementFailedNodeCount: refinement.failedNodeCount,
        centerlineRefinementWallRejectedSegmentCount: refinement.wallRejectedSegmentCount,
        centerlineRefinementWallClampedSegmentCount: refinement.wallClampedSegmentCount,
        centerlineRefinementAverageShift: refinement.averageShift,
        centerlineRefinementMaxShift: refinement.maxShift,
        centerlineClearanceRelaxPasses,
        centerlineClearanceRelaxActualPassCount: clearanceRelaxation.passCount,
        centerlineClearanceRelaxedNodeCount: clearanceRelaxation.adjustedNodeCount,
        centerlineClearanceRelaxRejectedNodeCount: clearanceRelaxation.rejectedNodeCount,
        centerlineClearanceRelaxAverageShift: clearanceRelaxation.averageShift,
        centerlineClearanceRelaxMaxShift: clearanceRelaxation.maxShift,
        centerlineClearanceRelaxAverageGain: clearanceRelaxation.averageClearanceGain,
        centerlineClearanceRelaxMaxGain: clearanceRelaxation.maxClearanceGain,
        centerlineOutlierRelaxPassCount: outlierRelaxation.passCount,
        centerlineOutlierRelaxKeptPassCount: outlierRelaxation.keptPassCount,
        centerlineOutlierRelaxAttemptedNodeCount: outlierRelaxation.adjustedNodeCount,
        centerlineOutlierRelaxedNodeCount: outlierRelaxation.keptNodeCount,
        centerlineOutlierRelaxRejectedNodeCount: outlierRelaxation.rejectedNodeCount,
        centerlineOutlierRelaxRolledBack: outlierRelaxation.rolledBackCount > 0,
        centerlineOutlierRelaxRolledBackCount: outlierRelaxation.rolledBackCount,
        centerlineOutlierRelaxAverageShift: outlierRelaxation.keptNodeCount ? outlierRelaxation.averageShift : 0,
        centerlineOutlierRelaxMaxShift: outlierRelaxation.keptNodeCount ? outlierRelaxation.maxShift : 0,
        centerlineOutlierRelaxAverageClearanceGain: outlierRelaxation.keptNodeCount ? outlierRelaxation.averageClearanceGain : 0,
        centerlineOutlierRelaxMaxClearanceGain: outlierRelaxation.keptNodeCount ? outlierRelaxation.maxClearanceGain : 0,
        centerlineOutlierRelaxDirectCenteredNodeCount: outlierRelaxation.keptNodeCount ? outlierRelaxation.directCenteredNodeCount : 0,
        centerlineOutlierRelaxDirectWallFallbackNodeCount: outlierRelaxation.keptNodeCount ? outlierRelaxation.directWallFallbackNodeCount : 0,
        centerlineOutlierRelaxReferenceFieldCenteredNodeCount: outlierRelaxation.keptNodeCount
            ? outlierRelaxation.referenceFieldCenteredNodeCount
            : 0,
        centerlineOutlierRelaxPatchedNodeCount: outlierRelaxation.keptNodeCount ? outlierRelaxation.patchedNodeCount : 0,
        centerlineOutlierRelaxRejectedCandidates: outlierRelaxation.rejectedCandidates || [],
        centerlineOutlierRerouteRequestedPasses: reroutePassCount,
        centerlineOutlierRerouteMaxCandidates: centerlineOutlierRerouteMaxCandidates,
        centerlineOutlierReroutePassCount: outlierRerouting.passCount,
        centerlineOutlierRerouteKeptPassCount: outlierRerouting.keptPassCount,
        centerlineOutlierRerouteAttemptedChainCount: outlierRerouting.attemptedChainCount,
        centerlineOutlierRerouteRoutedChainCount: outlierRerouting.routedChainCount,
        centerlineOutlierRerouteKeptChainCount: outlierRerouting.keptChainCount,
        centerlineOutlierRerouteRejectedChainCount: outlierRerouting.rejectedChainCount,
        centerlineOutlierRerouteRolledBackCount: outlierRerouting.rolledBackCount,
        centerlineOutlierRerouteReplacedSegmentCount: outlierRerouting.keptChainCount
            ? outlierRerouting.replacedSegmentCount
            : 0,
        centerlineOutlierRerouteInsertedSegmentCount: outlierRerouting.keptChainCount
            ? outlierRerouting.insertedSegmentCount
            : 0,
        centerlineOutlierRerouteCenteredNodeCount: outlierRerouting.keptChainCount
            ? outlierRerouting.centeredRouteNodeCount
            : 0,
        centerlineOutlierRerouteChoices: outlierRerouting.routeChoices || [],
        centerlineOutlierRerouteRejectedChains: outlierRerouting.rejectedChains || [],
        centerlineOutlierChainPatchPassCount: outlierChainPatch.passCount,
        centerlineOutlierChainPatchKeptPassCount: outlierChainPatch.keptPassCount,
        centerlineOutlierChainPatchAttemptedChainCount: outlierChainPatch.attemptedChainCount,
        centerlineOutlierChainPatchPatchedChainCount: outlierChainPatch.patchedChainCount,
        centerlineOutlierChainPatchKeptChainCount: outlierChainPatch.keptChainCount,
        centerlineOutlierChainPatchPatchedNodeCount: outlierChainPatch.keptChainCount
            ? outlierChainPatch.patchedNodeCount
            : 0,
        centerlineOutlierChainPatchRejectedChainCount: outlierChainPatch.rejectedChainCount,
        centerlineOutlierChainPatchRolledBackCount: outlierChainPatch.rolledBackCount,
        centerlineOutlierChainPatchAverageShift: outlierChainPatch.keptChainCount
            ? outlierChainPatch.averageShift
            : 0,
        centerlineOutlierChainPatchMaxShift: outlierChainPatch.keptChainCount
            ? outlierChainPatch.maxShift
            : 0,
        centerlineOutlierChainPatchChoices: outlierChainPatch.choices || [],
        centerlineOutlierChainPatchRejectedChains: outlierChainPatch.rejectedChains || [],
        centerlineOutlierChainPatchRollbackDiagnostics: outlierChainPatch.rollbackDiagnostics || [],
        centerlineOutlierReconnectPassCount: outlierReconnect.passCount,
        centerlineOutlierReconnectKeptPassCount: outlierReconnect.keptPassCount,
        centerlineOutlierReconnectAttemptedChainCount: outlierReconnect.attemptedChainCount,
        centerlineOutlierReconnectRemovedChainCount: outlierReconnect.removedChainCount,
        centerlineOutlierReconnectKeptChainCount: outlierReconnect.keptChainCount,
        centerlineOutlierReconnectRemovedSegmentCount: outlierReconnect.keptChainCount
            ? outlierReconnect.removedSegmentCount
            : 0,
        centerlineOutlierReconnectRejectedChainCount: outlierReconnect.rejectedChainCount,
        centerlineOutlierReconnectRolledBackCount: outlierReconnect.rolledBackCount,
        centerlineOutlierReconnectRootedBranchOriginCount: outlierReconnect.rootedBranchOriginCount,
        centerlineOutlierReconnectRoutedBranchOriginCount: outlierReconnect.routedBranchOriginCount,
        centerlineOutlierReconnectRoutedPathSegmentCount: outlierReconnect.routedPathSegmentCount,
        centerlineOutlierReconnectCenteredNodeCount: outlierReconnect.centeredRouteNodeCount,
        centerlineOutlierReconnectChoices: outlierReconnect.choices || [],
        centerlineOutlierReconnectRollbackDiagnostics: outlierReconnect.rollbackDiagnostics || [],
        centerlineOutlierReferenceReconnectPassCount: outlierReferenceReconnect.passCount,
        centerlineOutlierReferenceReconnectKeptPassCount: outlierReferenceReconnect.keptPassCount,
        centerlineOutlierReferenceReconnectAttemptedChainCount:
            outlierReferenceReconnect.attemptedChainCount,
        centerlineOutlierReferenceReconnectRemovedChainCount:
            outlierReferenceReconnect.removedChainCount,
        centerlineOutlierReferenceReconnectKeptChainCount:
            outlierReferenceReconnect.keptChainCount,
        centerlineOutlierReferenceReconnectRemovedSegmentCount:
            outlierReferenceReconnect.keptChainCount
                ? outlierReferenceReconnect.removedSegmentCount
                : 0,
        centerlineOutlierReferenceReconnectRejectedChainCount:
            outlierReferenceReconnect.rejectedChainCount,
        centerlineOutlierReferenceReconnectRolledBackCount:
            outlierReferenceReconnect.rolledBackCount,
        centerlineOutlierReferenceReconnectRootedBranchOriginCount:
            outlierReferenceReconnect.rootedBranchOriginCount,
        centerlineOutlierReferenceReconnectRoutedBranchOriginCount:
            outlierReferenceReconnect.routedBranchOriginCount,
        centerlineOutlierReferenceReconnectRoutedPathSegmentCount:
            outlierReferenceReconnect.routedPathSegmentCount,
        centerlineOutlierReferenceReconnectCenteredNodeCount:
            outlierReferenceReconnect.centeredRouteNodeCount,
        centerlineOutlierReferenceReconnectChoices:
            outlierReferenceReconnect.choices || [],
        centerlineOutlierReferenceReconnectRollbackDiagnostics:
            outlierReferenceReconnect.rollbackDiagnostics || [],
        centerlineOutlierReferenceFieldPassCount: outlierReferenceCentering.passCount,
        centerlineOutlierReferenceFieldKeptPassCount: outlierReferenceCentering.keptPassCount,
        centerlineOutlierReferenceFieldAttemptedNodeCount: outlierReferenceCentering.attemptedNodeCount,
        centerlineOutlierReferenceFieldKeptNodeCount: outlierReferenceCentering.keptNodeCount,
        centerlineOutlierReferenceFieldRejectedNodeCount: outlierReferenceCentering.rejectedNodeCount,
        centerlineOutlierReferenceFieldRolledBackCount: outlierReferenceCentering.rolledBackCount,
        centerlineOutlierReferenceFieldAverageShift: outlierReferenceCentering.keptNodeCount
            ? outlierReferenceCentering.averageShift
            : 0,
        centerlineOutlierReferenceFieldMaxShift: outlierReferenceCentering.keptNodeCount
            ? outlierReferenceCentering.maxShift
            : 0,
        centerlineOutlierReferenceFieldRollbackDiagnostics:
            outlierReferenceCentering.rollbackDiagnostics || [],
        centerlinePreOutlierRelaxAverageOffset: preOutlierRelax.averageOffset,
        centerlinePreOutlierRelaxMaxOffset: preOutlierRelax.maxOffset,
        centerlinePreOutlierRelaxAverageNormalizedOffset: preOutlierRelax.averageNormalizedOffset,
        centerlinePreOutlierRelaxMaxNormalizedOffset: preOutlierRelax.maxNormalizedOffset,
        centerlineLeafPrunedSegmentCount: leafPruning.prunedSegmentCount,
        centerlineLeafPrunePassCount: leafPruning.prunedPassCount,
        centerlinePostPruneRootedBranchOriginCount: postPruneRooting.addedBranchOriginCount,
        centerlinePostPruneDiscardedComponentCount: postPruneRooting.discardedComponentCount,
        centerlinePostPruneDiscardedSegmentCount: postPruneRooting.discardedSegmentCount,
        centerlineRoutedBranchOriginCount: branchRouting.routedSegmentCount,
        centerlineRoutedBranchPathSegmentCount: branchRouting.routedPathSegmentCount,
        centerlineGridBranchRouteCount: branchRouting.gridRouteCount,
        centerlineGridBranchRoutePathSegmentCount: branchRouting.gridRoutePathSegmentCount,
        centerlineFailedBranchRouteCount: branchRouting.failedRouteCount,
        centerlineRelaxedBranchRouteNodeCount: branchRouting.relaxedRouteNodeCount,
        centerlineCenteredBranchRouteNodeCount: branchRouting.centeredRouteNodeCount,
        centerlineBranchRouteTotalClearanceGain: branchRouting.totalRouteClearanceGain,
        centerlineBranchRouteMaxClearanceGain: branchRouting.maxRouteClearanceGain,
        centerlineBranchRouteChoices: branchRouting.routeChoiceDiagnostics,
        centerlineCyclePrunedSegmentCount: cyclePruning.removedSegmentCount,
        centerlineGraphNodeCount: cyclePruning.nodeCount,
        centerlineGraphCycleCount: cyclePruning.cycleCount,
        centerlineTopologyBeforeCleanup: topologyBeforeCleanup,
        centerlineTopologyAfterCleanup: topologyAfterCleanup,
        centerlineBacktrackCollapsedNodeCount:
            initialBacktrackSimplification.collapsedNodeCount +
            finalBacktrackSimplification.collapsedNodeCount,
        centerlineBacktrackRemovedSegmentCount:
            initialBacktrackSimplification.removedSegmentCount +
            finalBacktrackSimplification.removedSegmentCount,
        centerlineBacktrackInsertedSegmentCount:
            initialBacktrackSimplification.insertedSegmentCount +
            finalBacktrackSimplification.insertedSegmentCount,
        centerlineBacktrackRejectedNodeCount:
            initialBacktrackSimplification.rejectedNodeCount +
            finalBacktrackSimplification.rejectedNodeCount,
        centerlineBacktrackPathLengthReduction:
            initialBacktrackSimplification.pathLengthReduction +
            finalBacktrackSimplification.pathLengthReduction,
        centerlineInvalidSegmentCountBeforeReroute:
            invalidChainRerouting.initialInvalidSegmentCount,
        centerlineInvalidSegmentCountAfterReroute:
            invalidChainRerouting.remainingInvalidSegmentCount,
        centerlineInvalidSegmentCountFinal: remainingInvalidSegmentCount,
        centerlineInvalidRerouteAttemptedChainCount:
            invalidChainRerouting.attemptedChainCount,
        centerlineInvalidReroutedChainCount: invalidChainRerouting.routedChainCount,
        centerlineInvalidRerouteReplacedSegmentCount:
            invalidChainRerouting.replacedSegmentCount,
        centerlineInvalidRerouteInsertedSegmentCount:
            invalidChainRerouting.insertedSegmentCount,
        centerlineInvalidRerouteBlockedSegmentCount:
            invalidChainRerouting.blockedSegmentCount,
        centerlineInvalidRerouteChoices: invalidChainRerouting.choices,
        centerlineInvalidRerouteBlockedComponents:
            invalidChainRerouting.blockedComponents,
        centerlineFinalChainPatchPassCount: finalChainPatch.passCount,
        centerlineFinalChainPatchKeptPassCount: finalChainPatch.keptPassCount,
        centerlineFinalChainPatchAttemptedChainCount:
            finalChainPatch.attemptedChainCount,
        centerlineFinalChainPatchKeptChainCount: finalChainPatch.keptChainCount,
        centerlineFinalChainPatchRejectedChainCount:
            finalChainPatch.rejectedChainCount,
        centerlineFinalChainPatchRolledBackCount:
            finalChainPatch.rolledBackCount,
        centerlineFinalChainPatchChoices: finalChainPatch.choices,
        centerlineFinalChainPatchRollbackDiagnostics:
            finalChainPatch.rollbackDiagnostics,
        centerlineFinalReroutePassCount: finalRerouting.passCount,
        centerlineFinalRerouteKeptPassCount: finalRerouting.keptPassCount,
        centerlineFinalRerouteAttemptedChainCount:
            finalRerouting.attemptedChainCount,
        centerlineFinalRerouteRoutedChainCount:
            finalRerouting.routedChainCount,
        centerlineFinalRerouteKeptChainCount:
            finalRerouting.keptChainCount,
        centerlineFinalRerouteRejectedChainCount:
            finalRerouting.rejectedChainCount,
        centerlineFinalRerouteRolledBackCount:
            finalRerouting.rolledBackCount,
        centerlineFinalRerouteChoices: finalRerouting.choices,
        centerlineFinalRerouteRollbackDiagnostics:
            finalRerouting.rollbackDiagnostics,
        centerlineCenteringMeasuredNodeCount: centering.measuredNodeCount,
        centerlineCenteringFailedNodeCount: centering.failedNodeCount,
        centerlineCenteringAverageOffset: centering.averageOffset,
        centerlineCenteringMaxOffset: centering.maxOffset,
        centerlineCenteringAverageNormalizedOffset: centering.averageNormalizedOffset,
        centerlineCenteringMaxNormalizedOffset: centering.maxNormalizedOffset,
        centerlineCenteringWorstOffsets: centering.worstOffsets,
        centerlineCenteringWorstNormalizedOffsets: centering.worstNormalizedOffsets,
        adaptiveDirectionCount: adaptiveAxes.length,
        axisSliceSpacing,
        axisSliceSpacingMultipliers,
        secondaryAxisSpacingMultiplier,
        secondaryAxisMaxRadius,
        minCompactness,
        connectorLumenClearance,
        lumenCast: resolvedLumenCast.diagnostics,
        wallValidation: resolvedWallBvh ? 'stl-bvh' : 'lumen-field'
    };

    return {
        slices: resolvedLumenCast.slices,
        lumenCast: resolvedLumenCast,
        segments: allSegments,
        debugSegments,
        diagnostics: allSegments.diagnostics
    };
}
