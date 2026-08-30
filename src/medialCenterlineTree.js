import * as THREE from 'three';

const DEFAULT_GRID_SPACING = 1.2;
const DEFAULT_NODE_SPACING = 2;
const MAX_THINNING_PASSES = 512;
const NEIGHBOUR_OFFSETS = [
    [0, -1], [1, -1], [1, 0], [1, 1],
    [0, 1], [-1, 1], [-1, 0], [-1, -1]
];

class MinHeap {
    constructor() {
        this.items = [];
    }

    get size() {
        return this.items.length;
    }

    push(item) {
        this.items.push(item);
        let index = this.items.length - 1;
        while (index > 0) {
            const parent = Math.floor((index - 1) / 2);
            if (this.items[parent].cost <= item.cost) break;
            this.items[index] = this.items[parent];
            index = parent;
        }
        this.items[index] = item;
    }

    pop() {
        if (!this.items.length) return null;
        const root = this.items[0];
        const last = this.items.pop();
        if (!this.items.length) return root;
        let index = 0;
        while (true) {
            const left = index * 2 + 1;
            if (left >= this.items.length) break;
            const right = left + 1;
            const child = right < this.items.length && this.items[right].cost < this.items[left].cost
                ? right
                : left;
            if (this.items[child].cost >= last.cost) break;
            this.items[index] = this.items[child];
            index = child;
        }
        this.items[index] = last;
        return root;
    }
}

function pointInPolygon(x, z, polygon) {
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const a = polygon[i];
        const b = polygon[j];
        if (((a.z > z) !== (b.z > z)) &&
            x < (b.x - a.x) * (z - a.z) / (b.z - a.z) + a.x) {
            inside = !inside;
        }
    }
    return inside;
}

function pointSegmentDistanceSq2d(x, z, a, b) {
    const dx = b.x - a.x;
    const dz = b.z - a.z;
    const lengthSq = dx * dx + dz * dz;
    if (lengthSq < 1e-12) return (x - a.x) ** 2 + (z - a.z) ** 2;
    const t = Math.max(0, Math.min(1, ((x - a.x) * dx + (z - a.z) * dz) / lengthSq));
    const px = a.x + dx * t;
    const pz = a.z + dz * t;
    return (x - px) ** 2 + (z - pz) ** 2;
}

function polygonClearance(x, z, polygon) {
    let bestSq = Infinity;
    for (let index = 0; index < polygon.length; index++) {
        bestSq = Math.min(
            bestSq,
            pointSegmentDistanceSq2d(x, z, polygon[index], polygon[(index + 1) % polygon.length])
        );
    }
    return Math.sqrt(bestSq);
}

function polygonBounds(polygon) {
    let minX = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxZ = -Infinity;
    for (const point of polygon) {
        minX = Math.min(minX, point.x);
        minZ = Math.min(minZ, point.z);
        maxX = Math.max(maxX, point.x);
        maxZ = Math.max(maxZ, point.z);
    }
    return { minX, minZ, maxX, maxZ };
}

function transitionCount(mask, width, x, z) {
    let transitions = 0;
    let previous = mask[(z - 1) * width + (x - 1)];
    for (let index = 0; index < NEIGHBOUR_OFFSETS.length; index++) {
        const [dx, dz] = NEIGHBOUR_OFFSETS[index];
        const current = mask[(z + dz) * width + x + dx];
        if (!previous && current) transitions++;
        previous = current;
    }
    return transitions;
}

function neighbourCount(mask, width, x, z) {
    let count = 0;
    for (const [dx, dz] of NEIGHBOUR_OFFSETS) count += mask[(z + dz) * width + x + dx];
    return count;
}

function thinMask(mask, width, height) {
    const removals = [];
    let passCount = 0;
    let changed = true;
    while (changed && passCount < MAX_THINNING_PASSES) {
        changed = false;
        for (let phase = 0; phase < 2; phase++) {
            removals.length = 0;
            for (let z = 1; z < height - 1; z++) {
                for (let x = 1; x < width - 1; x++) {
                    const offset = z * width + x;
                    if (!mask[offset]) continue;
                    const neighbours = neighbourCount(mask, width, x, z);
                    if (neighbours < 2 || neighbours > 6) continue;
                    if (transitionCount(mask, width, x, z) !== 1) continue;
                    const north = mask[(z - 1) * width + x];
                    const east = mask[z * width + x + 1];
                    const south = mask[(z + 1) * width + x];
                    const west = mask[z * width + x - 1];
                    const firstConstraint = phase === 0
                        ? north * east * south
                        : north * east * west;
                    const secondConstraint = phase === 0
                        ? east * south * west
                        : north * south * west;
                    if (firstConstraint || secondConstraint) continue;
                    removals.push(offset);
                }
            }
            if (removals.length) {
                changed = true;
                for (const offset of removals) mask[offset] = 0;
            }
        }
        passCount++;
    }
    return passCount;
}

function rasterizeContour(contour, spacing) {
    const polygon = contour.polygon || [];
    if (polygon.length < 3) return null;
    const bounds = contour.bounds || polygonBounds(polygon);
    const padding = spacing * 2;
    const originX = bounds.minX - padding;
    const originZ = bounds.minZ - padding;
    const width = Math.max(5, Math.ceil((bounds.maxX - bounds.minX + padding * 2) / spacing) + 1);
    const height = Math.max(5, Math.ceil((bounds.maxZ - bounds.minZ + padding * 2) / spacing) + 1);
    const mask = new Uint8Array(width * height);
    let insideCount = 0;
    let best = null;
    for (let z = 1; z < height - 1; z++) {
        const worldZ = originZ + z * spacing;
        for (let x = 1; x < width - 1; x++) {
            const worldX = originX + x * spacing;
            if (!pointInPolygon(worldX, worldZ, polygon)) continue;
            mask[z * width + x] = 1;
            insideCount++;
            const clearance = polygonClearance(worldX, worldZ, polygon);
            if (!best || clearance > best.clearance) best = { x, z, worldX, worldZ, clearance };
        }
    }
    if (!insideCount || !best) return null;
    const thinningPassCount = thinMask(mask, width, height);
    const pixels = [];
    for (let z = 1; z < height - 1; z++) {
        for (let x = 1; x < width - 1; x++) {
            if (!mask[z * width + x]) continue;
            const worldX = originX + x * spacing;
            const worldZ = originZ + z * spacing;
            pixels.push({
                x,
                z,
                worldX,
                worldZ,
                clearance: polygonClearance(worldX, worldZ, polygon)
            });
        }
    }
    if (!pixels.length) pixels.push(best);
    return { width, height, mask, pixels, thinningPassCount, insideCount };
}

function edgeKey(a, b) {
    return a < b ? `${a}|${b}` : `${b}|${a}`;
}

function addGraphEdge(nodes, edgeSet, aId, bId, length = null) {
    if (aId === bId) return false;
    const key = edgeKey(aId, bId);
    if (edgeSet.has(key)) return false;
    const a = nodes[aId];
    const b = nodes[bId];
    const distance = length ?? a.point.distanceTo(b.point);
    if (!Number.isFinite(distance) || distance < 1e-6) return false;
    edgeSet.add(key);
    a.neighbours.set(bId, distance);
    b.neighbours.set(aId, distance);
    return true;
}

function segmentIntersectsWall(a, b, wallBvh) {
    if (!wallBvh?.raycastFirst) return false;
    const direction = new THREE.Vector3().subVectors(b, a);
    const length = direction.length();
    if (length < 1e-6) return false;
    direction.multiplyScalar(1 / length);
    const hit = wallBvh.raycastFirst(
        new THREE.Ray(a, direction),
        THREE.DoubleSide,
        1e-4,
        Math.max(1e-4, length - 1e-4)
    );
    return Boolean(hit);
}

function segmentStaysInside(a, b, lumenField, wallBvh, spacing) {
    if (segmentIntersectsWall(a, b, wallBvh)) return false;
    if (wallBvh) return true;
    if (!lumenField?.query) return true;
    const length = a.distanceTo(b);
    const sampleCount = Math.max(2, Math.ceil(length / Math.max(0.6, spacing * 0.7)));
    const point = new THREE.Vector3();
    const scratch = {};
    for (let index = 0; index <= sampleCount; index++) {
        point.lerpVectors(a, b, index / sampleCount);
        const query = lumenField.query(point, scratch);
        if (!query || query.signedDistance < -0.08) return false;
    }
    return true;
}

function makeGraph(slices, { gridSpacing, lumenField, wallBvh }) {
    const nodes = [];
    const edgeSet = new Set();
    const sliceLayers = [];
    let contourCount = 0;
    let rasterCellCount = 0;
    let thinningPassCount = 0;

    for (let sliceIndex = 0; sliceIndex < slices.length; sliceIndex++) {
        const slice = slices[sliceIndex];
        const layerNodeIds = [];
        for (let contourIndex = 0; contourIndex < slice.contours.length; contourIndex++) {
            const contour = slice.contours[contourIndex];
            const raster = rasterizeContour(contour, gridSpacing);
            if (!raster) continue;
            contourCount++;
            rasterCellCount += raster.insideCount;
            thinningPassCount += raster.thinningPassCount;
            const pixelNode = new Map();
            for (const pixel of raster.pixels) {
                const id = nodes.length;
                const point = new THREE.Vector3(pixel.worldX, slice.y, pixel.worldZ);
                const fieldClearance = lumenField?.query?.(point)?.signedDistance;
                const clearance = Math.max(
                    gridSpacing * 0.45,
                    Number.isFinite(fieldClearance) && fieldClearance > 0
                        ? Math.min(pixel.clearance, fieldClearance)
                        : pixel.clearance
                );
                nodes.push({
                    id,
                    point,
                    clearance,
                    sliceIndex,
                    rawSliceIndex: slice.rawIndex ?? sliceIndex,
                    contourIndex,
                    pixelX: pixel.x,
                    pixelZ: pixel.z,
                    neighbours: new Map()
                });
                layerNodeIds.push(id);
                pixelNode.set(`${pixel.x}|${pixel.z}`, id);
            }
            for (const pixel of raster.pixels) {
                const aId = pixelNode.get(`${pixel.x}|${pixel.z}`);
                for (const [dx, dz] of [[1, 0], [0, 1], [1, 1], [1, -1]]) {
                    const bId = pixelNode.get(`${pixel.x + dx}|${pixel.z + dz}`);
                    if (bId === undefined) continue;
                    addGraphEdge(nodes, edgeSet, aId, bId);
                }
            }
        }
        sliceLayers.push({
            sliceIndex,
            rawSliceIndex: slice.rawIndex ?? sliceIndex,
            y: slice.y,
            nodeIds: layerNodeIds
        });
    }

    const nonEmptyLayers = sliceLayers.filter(layer => layer.nodeIds.length);
    const spatialCell = Math.max(2.4, gridSpacing * 2.4);
    for (let layerIndex = 0; layerIndex < nonEmptyLayers.length - 1; layerIndex++) {
        const lower = nonEmptyLayers[layerIndex];
        let upper = null;
        for (let lookahead = layerIndex + 1; lookahead < nonEmptyLayers.length; lookahead++) {
            const candidate = nonEmptyLayers[lookahead];
            if (candidate.rawSliceIndex - lower.rawSliceIndex > 3) break;
            upper = candidate;
            break;
        }
        if (!upper) continue;
        const yGap = Math.abs(upper.y - lower.y);
        const hash = new Map();
        for (const nodeId of upper.nodeIds) {
            const point = nodes[nodeId].point;
            const key = `${Math.floor(point.x / spatialCell)}|${Math.floor(point.z / spatialCell)}`;
            let bucket = hash.get(key);
            if (!bucket) {
                bucket = [];
                hash.set(key, bucket);
            }
            bucket.push(nodeId);
        }
        for (const lowerId of lower.nodeIds) {
            const node = nodes[lowerId];
            const ix = Math.floor(node.point.x / spatialCell);
            const iz = Math.floor(node.point.z / spatialCell);
            const candidates = [];
            for (let dz = -2; dz <= 2; dz++) {
                for (let dx = -2; dx <= 2; dx++) {
                    for (const upperId of hash.get(`${ix + dx}|${iz + dz}`) || []) {
                        const other = nodes[upperId];
                        const distance = node.point.distanceTo(other.point);
                        const limit = Math.max(
                            gridSpacing * 2.6,
                            yGap * 3.2,
                            Math.min(8, (node.clearance + other.clearance) * 0.9)
                        );
                        if (distance > limit) continue;
                        candidates.push({ id: upperId, distance });
                    }
                }
            }
            candidates.sort((a, b) => a.distance - b.distance);
            let added = 0;
            for (const candidate of candidates) {
                if (added >= 3) break;
                if (!segmentStaysInside(
                    node.point,
                    nodes[candidate.id].point,
                    lumenField,
                    wallBvh,
                    gridSpacing
                )) {
                    continue;
                }
                if (addGraphEdge(nodes, edgeSet, lowerId, candidate.id, candidate.distance)) added++;
            }
        }
    }

    return {
        nodes,
        edgeSet,
        diagnostics: {
            sliceCount: slices.length,
            contourCount,
            rasterCellCount,
            thinningPassCount,
            graphNodeCount: nodes.length,
            graphEdgeCount: edgeSet.size
        }
    };
}

function graphComponents(nodes) {
    const seen = new Uint8Array(nodes.length);
    const components = [];
    for (const start of nodes) {
        if (seen[start.id]) continue;
        const stack = [start.id];
        const component = [];
        seen[start.id] = 1;
        while (stack.length) {
            const id = stack.pop();
            component.push(id);
            for (const neighbourId of nodes[id].neighbours.keys()) {
                if (seen[neighbourId]) continue;
                seen[neighbourId] = 1;
                stack.push(neighbourId);
            }
        }
        components.push(component);
    }
    components.sort((a, b) => b.length - a.length);
    return components;
}

function findLocalComponentRoute(origin, target, lumenField, wallBvh, baseSpacing) {
    if (!lumenField?.query || !wallBvh) return null;
    const directDistance = origin.point.distanceTo(target.point);
    const spacing = Math.max(
        0.42,
        Math.min(0.7, Math.min(origin.clearance, target.clearance, baseSpacing) * 0.78)
    );
    const padding = Math.max(4, Math.min(12, directDistance * 1.5));
    const box = new THREE.Box3()
        .setFromPoints([origin.point, target.point])
        .expandByScalar(padding);
    const size = box.getSize(new THREE.Vector3());
    const nx = Math.max(2, Math.ceil(size.x / spacing) + 1);
    const ny = Math.max(2, Math.ceil(size.y / spacing) + 1);
    const nz = Math.max(2, Math.ceil(size.z / spacing) + 1);
    if (nx * ny * nz > 350000) return null;

    const cells = new Map();
    const queryScratch = {};
    const point = new THREE.Vector3();
    for (let iz = 0; iz < nz; iz++) {
        for (let iy = 0; iy < ny; iy++) {
            for (let ix = 0; ix < nx; ix++) {
                point.set(
                    box.min.x + ix * spacing,
                    box.min.y + iy * spacing,
                    box.min.z + iz * spacing
                );
                const query = lumenField.query(point, queryScratch);
                const key = `${ix}|${iy}|${iz}`;
                cells.set(key, {
                    key,
                    ix,
                    iy,
                    iz,
                    point: point.clone(),
                    // The primary Y-slice lumen field can miss a narrow,
                    // oblique branch such as the right subclavian hairpin.
                    // Wall crossings remain the authoritative barrier for
                    // local connector routing, so retain those grid cells and
                    // use a conservative fallback clearance.
                    clearance: Number.isFinite(query?.signedDistance) &&
                        query.signedDistance > 0
                        ? query.signedDistance
                        : 0.08
                });
            }
        }
    }
    if (!cells.size) return null;

    const anchorCell = node => {
        let best = null;
        for (const cell of cells.values()) {
            const distance = node.point.distanceTo(cell.point);
            if (distance > spacing * 2.6 || (best && distance >= best.distance)) continue;
            if (!segmentStaysInside(node.point, cell.point, lumenField, wallBvh, spacing)) continue;
            best = { cell, distance };
        }
        return best?.cell || null;
    };
    const start = anchorCell(origin);
    const goal = anchorCell(target);
    if (!start || !goal) return null;

    const offsets = [];
    for (let dz = -1; dz <= 1; dz++) {
        for (let dy = -1; dy <= 1; dy++) {
            for (let dx = -1; dx <= 1; dx++) {
                if (dx || dy || dz) offsets.push([dx, dy, dz]);
            }
        }
    }
    const distances = new Map([[start.key, 0]]);
    const predecessors = new Map();
    const heap = new MinHeap();
    heap.push({ key: start.key, cost: start.point.distanceTo(goal.point), pathCost: 0 });
    let reached = false;
    let expandedCellCount = 0;
    while (heap.size && expandedCellCount < 50000) {
        const currentEntry = heap.pop();
        const current = cells.get(currentEntry.key);
        if (!current) continue;
        const knownDistance = distances.get(current.key);
        if (currentEntry.pathCost !== knownDistance) continue;
        expandedCellCount++;
        if (current.key === goal.key) {
            reached = true;
            break;
        }
        for (const [dx, dy, dz] of offsets) {
            const neighbour = cells.get(`${current.ix + dx}|${current.iy + dy}|${current.iz + dz}`);
            if (!neighbour) continue;
            if (!segmentStaysInside(current.point, neighbour.point, lumenField, wallBvh, spacing)) continue;
            const length = current.point.distanceTo(neighbour.point);
            const clearance = Math.max(0.08, Math.min(current.clearance, neighbour.clearance));
            const nextDistance = knownDistance + length * (1 + 0.7 / Math.pow(clearance + 0.18, 1.35));
            if (nextDistance >= (distances.get(neighbour.key) ?? Infinity)) continue;
            distances.set(neighbour.key, nextDistance);
            predecessors.set(neighbour.key, current.key);
            heap.push({
                key: neighbour.key,
                pathCost: nextDistance,
                cost: nextDistance + neighbour.point.distanceTo(goal.point)
            });
        }
    }
    if (!reached) return null;

    const reversed = [];
    let cursor = goal.key;
    for (let guard = 0; guard < cells.size; guard++) {
        reversed.push(cells.get(cursor).point.clone());
        if (cursor === start.key) break;
        cursor = predecessors.get(cursor);
        if (!cursor) return null;
    }
    const rawPath = [origin.point.clone(), ...reversed.reverse(), target.point.clone()];
    const simplified = [rawPath[0]];
    let pathIndex = 0;
    while (pathIndex < rawPath.length - 1) {
        let nextIndex = rawPath.length - 1;
        while (nextIndex > pathIndex + 1 && !segmentStaysInside(
            rawPath[pathIndex],
            rawPath[nextIndex],
            lumenField,
            wallBvh,
            spacing
        )) {
            nextIndex--;
        }
        simplified.push(rawPath[nextIndex]);
        pathIndex = nextIndex;
    }
    return simplified;
}

function addGraphRoute(graph, aId, bId, path, lumenField) {
    let previousId = aId;
    const scratch = {};
    for (let index = 1; index < path.length - 1; index++) {
        const point = path[index];
        const signedDistance = lumenField?.query?.(point, scratch)?.signedDistance;
        const id = graph.nodes.length;
        graph.nodes.push({
            id,
            point: point.clone(),
            clearance: Number.isFinite(signedDistance) && signedDistance > 0 ? signedDistance : 0.4,
            sliceIndex: null,
            rawSliceIndex: null,
            contourIndex: null,
            pixelX: null,
            pixelZ: null,
            neighbours: new Map()
        });
        addGraphEdge(graph.nodes, graph.edgeSet, previousId, id);
        previousId = id;
    }
    addGraphEdge(graph.nodes, graph.edgeSet, previousId, bId);
}

function connectGraphComponents(graph, lumenField, wallBvh, gridSpacing) {
    const minimumRoutedComponentNodeCount = 101;
    let components = graphComponents(graph.nodes);
    let addedEdgeCount = 0;
    let routedConnectorCount = 0;
    let failedRouteCount = 0;
    while (components.length > 1) {
        const connected = new Set(components[0]);
        const connectorCellSize = 20;
        const connectedHash = new Map();
        for (const nodeId of connected) {
            const point = graph.nodes[nodeId].point;
            const key = [
                Math.floor(point.x / connectorCellSize),
                Math.floor(point.y / connectorCellSize),
                Math.floor(point.z / connectorCellSize)
            ].join('|');
            const bucket = connectedHash.get(key) || [];
            bucket.push(nodeId);
            connectedHash.set(key, bucket);
        }
        let best = null;
        const nearestPairs = [];
        for (let componentIndex = 1; componentIndex < components.length; componentIndex++) {
            const component = components[componentIndex];
            for (const aId of component) {
                const a = graph.nodes[aId];
                const ix = Math.floor(a.point.x / connectorCellSize);
                const iy = Math.floor(a.point.y / connectorCellSize);
                const iz = Math.floor(a.point.z / connectorCellSize);
                for (let dz = -1; dz <= 1; dz++) {
                    for (let dy = -1; dy <= 1; dy++) {
                        for (let dx = -1; dx <= 1; dx++) {
                            for (const bId of connectedHash.get(
                                `${ix + dx}|${iy + dy}|${iz + dz}`
                            ) || []) {
                                const b = graph.nodes[bId];
                                const distance = a.point.distanceTo(b.point);
                                const limit = Math.max(
                                    7,
                                    Math.min(
                                        18,
                                        (a.clearance + b.clearance) * 1.6
                                    )
                                );
                                if (distance > Math.max(20, limit * 1.5)) {
                                    continue;
                                }
                                nearestPairs.push({
                                    aId,
                                    bId,
                                    distance,
                                    componentNodeCount: component.length
                                });
                                nearestPairs.sort((first, second) =>
                                    first.distance - second.distance
                                );
                                if (nearestPairs.length > 8) nearestPairs.pop();
                                if (
                                    distance > limit ||
                                    (best && distance >= best.distance)
                                ) {
                                    continue;
                                }
                                if (!segmentStaysInside(
                                    a.point,
                                    b.point,
                                    lumenField,
                                    wallBvh,
                                    gridSpacing
                                )) {
                                    continue;
                                }
                                best = { aId, bId, distance };
                            }
                        }
                    }
                }
            }
        }
        if (best) {
            addGraphEdge(graph.nodes, graph.edgeSet, best.aId, best.bId, best.distance);
            addedEdgeCount++;
        } else {
            // Authored digital branches can leave a sub-voxel cap artifact at
            // an otherwise shared plantar-arch control point. Bridge only a
            // very short gap belonging to a substantial component; this does
            // not apply to the wider right-subclavian hairpin or small debris.
            const shortAuthoredJunction = nearestPairs.find(pair =>
                pair.componentNodeCount >= 100 && pair.distance <= 8
            );
            if (shortAuthoredJunction) {
                addGraphEdge(
                    graph.nodes,
                    graph.edgeSet,
                    shortAuthoredJunction.aId,
                    shortAuthoredJunction.bId,
                    shortAuthoredJunction.distance
                );
                addedEdgeCount++;
                components = graphComponents(graph.nodes);
                continue;
            }
            let routed = false;
            for (const pair of nearestPairs.filter(pair =>
                pair.componentNodeCount >= minimumRoutedComponentNodeCount
            )) {
                const path = findLocalComponentRoute(
                    graph.nodes[pair.aId],
                    graph.nodes[pair.bId],
                    lumenField,
                    wallBvh,
                    gridSpacing
                );
                if (!path?.length) continue;
                addGraphRoute(graph, pair.aId, pair.bId, path, lumenField);
                routedConnectorCount++;
                routed = true;
                break;
            }
            if (!routed) {
                failedRouteCount++;
                break;
            }
        }
        components = graphComponents(graph.nodes);
    }
    return { components, addedEdgeCount, routedConnectorCount, failedRouteCount };
}

function summarizeGraphComponents(nodes, components) {
    const primary = components[0] || [];
    return components.map((component, componentIndex) => {
        const box = new THREE.Box3();
        let clearanceSum = 0;
        let edgeLength = 0;
        const componentSet = new Set(component);
        for (const id of component) {
            const node = nodes[id];
            box.expandByPoint(node.point);
            clearanceSum += node.clearance;
            for (const [neighbourId, length] of node.neighbours) {
                if (id < neighbourId && componentSet.has(neighbourId)) edgeLength += length;
            }
        }
        let nearestPrimaryDistance = componentIndex === 0 ? 0 : Infinity;
        if (componentIndex > 0) {
            for (const id of component) {
                for (const primaryId of primary) {
                    nearestPrimaryDistance = Math.min(
                        nearestPrimaryDistance,
                        nodes[id].point.distanceTo(nodes[primaryId].point)
                    );
                }
            }
        }
        return {
            nodeCount: component.length,
            edgeLength,
            averageClearance: clearanceSum / Math.max(1, component.length),
            nearestPrimaryDistance,
            min: { x: box.min.x, y: box.min.y, z: box.min.z },
            max: { x: box.max.x, y: box.max.y, z: box.max.z }
        };
    });
}

function dijkstraTree(nodes, rootId) {
    const distances = new Float64Array(nodes.length);
    distances.fill(Infinity);
    const predecessors = new Int32Array(nodes.length);
    predecessors.fill(-1);
    const heap = new MinHeap();
    distances[rootId] = 0;
    heap.push({ id: rootId, cost: 0 });
    while (heap.size) {
        const current = heap.pop();
        if (current.cost !== distances[current.id]) continue;
        const node = nodes[current.id];
        for (const [neighbourId, length] of node.neighbours) {
            const neighbour = nodes[neighbourId];
            const clearance = Math.max(0.2, Math.min(node.clearance, neighbour.clearance));
            const centeringPenalty = 1 + 3.5 / Math.pow(clearance + 0.35, 1.55);
            const nextCost = current.cost + length * centeringPenalty;
            if (nextCost >= distances[neighbourId]) continue;
            distances[neighbourId] = nextCost;
            predecessors[neighbourId] = current.id;
            heap.push({ id: neighbourId, cost: nextCost });
        }
    }
    return { distances, predecessors };
}

function spatialHash(nodes, cellSize) {
    const hash = new Map();
    for (const node of nodes) {
        const key = [
            Math.floor(node.point.x / cellSize),
            Math.floor(node.point.y / cellSize),
            Math.floor(node.point.z / cellSize)
        ].join('|');
        let bucket = hash.get(key);
        if (!bucket) {
            bucket = [];
            hash.set(key, bucket);
        }
        bucket.push(node.id);
    }
    return hash;
}

function markCovered(nodes, hash, covered, pathIds, cellSize, gridSpacing) {
    let newlyCovered = 0;
    for (const pathId of pathIds) {
        const pathNode = nodes[pathId];
        const radius = Math.max(gridSpacing * 2.4, pathNode.clearance * 1.18);
        const cellRadius = Math.ceil(radius / cellSize);
        const ix = Math.floor(pathNode.point.x / cellSize);
        const iy = Math.floor(pathNode.point.y / cellSize);
        const iz = Math.floor(pathNode.point.z / cellSize);
        for (let dz = -cellRadius; dz <= cellRadius; dz++) {
            for (let dy = -cellRadius; dy <= cellRadius; dy++) {
                for (let dx = -cellRadius; dx <= cellRadius; dx++) {
                    for (const nodeId of hash.get(`${ix + dx}|${iy + dy}|${iz + dz}`) || []) {
                        if (covered[nodeId]) continue;
                        if (nodes[nodeId].point.distanceTo(pathNode.point) > radius) continue;
                        covered[nodeId] = 1;
                        newlyCovered++;
                    }
                }
            }
        }
    }
    return newlyCovered;
}

function extractMedialTree(graph, { gridSpacing }) {
    const { nodes } = graph;
    if (!nodes.length) return { selectedNodeIds: new Set(), selectedEdges: new Set(), diagnostics: {} };
    const allComponents = graphComponents(nodes);
    const components = allComponents.length ? [allComponents[0]] : [];
    const selectedNodeIds = new Set();
    const selectedEdges = new Set();
    const covered = new Uint8Array(nodes.length);
    const hashCellSize = Math.max(gridSpacing * 2.5, 2.5);
    const hash = spatialHash(nodes, hashCellSize);
    let acceptedBranchCount = 0;
    let rejectedTwigCount = 0;
    let unreachableNodeCount = 0;

    for (const component of components) {
        const rootId = component.reduce((bestId, id) =>
            nodes[id].clearance > nodes[bestId].clearance ? id : bestId
        , component[0]);
        const { distances, predecessors } = dijkstraTree(nodes, rootId);
        selectedNodeIds.add(rootId);
        markCovered(nodes, hash, covered, [rootId], hashCellSize, gridSpacing);

        while (true) {
            let endpointId = -1;
            let farthest = -Infinity;
            for (const id of component) {
                if (covered[id] || !Number.isFinite(distances[id])) continue;
                if (distances[id] > farthest) {
                    farthest = distances[id];
                    endpointId = id;
                }
            }
            if (endpointId < 0) break;

            const path = [endpointId];
            let cursor = endpointId;
            let physicalLength = 0;
            let reachedTree = selectedNodeIds.has(cursor);
            for (let guard = 0; guard < nodes.length && !reachedTree; guard++) {
                const predecessor = predecessors[cursor];
                if (predecessor < 0) break;
                physicalLength += nodes[cursor].point.distanceTo(nodes[predecessor].point);
                cursor = predecessor;
                path.push(cursor);
                reachedTree = selectedNodeIds.has(cursor);
            }
            if (!reachedTree) {
                unreachableNodeCount++;
                covered[endpointId] = 1;
                continue;
            }

            if (physicalLength > 1e-6) {
                for (let index = 0; index < path.length - 1; index++) {
                    selectedNodeIds.add(path[index]);
                    selectedNodeIds.add(path[index + 1]);
                    selectedEdges.add(edgeKey(path[index], path[index + 1]));
                }
                acceptedBranchCount++;
            } else {
                rejectedTwigCount++;
            }
            markCovered(nodes, hash, covered, path, hashCellSize, gridSpacing);
        }
    }

    const retainedGraphNodeCount = components.reduce((sum, component) => sum + component.length, 0);
    const coveredNodeCount = components.reduce(
        (sum, component) => sum + component.reduce((count, id) => count + covered[id], 0),
        0
    );
    return {
        selectedNodeIds,
        selectedEdges,
        diagnostics: {
            componentCount: components.length,
            discardedDisconnectedComponentCount: Math.max(0, allComponents.length - components.length),
            discardedDisconnectedNodeCount: allComponents
                .slice(components.length)
                .reduce((sum, component) => sum + component.length, 0),
            retainedGraphNodeCount,
            acceptedBranchCount,
            rejectedTwigCount,
            unreachableNodeCount,
            coveredNodeCount,
            uncoveredNodeCount: retainedGraphNodeCount - coveredNodeCount
        }
    };
}

function selectedAdjacency(nodes, selectedEdges) {
    const adjacency = new Map();
    const add = (a, b) => {
        let neighbours = adjacency.get(a);
        if (!neighbours) {
            neighbours = new Set();
            adjacency.set(a, neighbours);
        }
        neighbours.add(b);
    };
    for (const key of selectedEdges) {
        const [a, b] = key.split('|').map(Number);
        add(a, b);
        add(b, a);
    }
    for (const id of adjacency.keys()) nodes[id].treePoint = nodes[id].point.clone();
    return adjacency;
}

function smoothTree(nodes, adjacency, lumenField, wallBvh, gridSpacing, passes = 4) {
    let movedNodeCount = 0;
    let totalShift = 0;
    let maxShift = 0;
    const scratch = {};
    for (let pass = 0; pass < passes; pass++) {
        const updates = [];
        for (const [id, neighbours] of adjacency) {
            if (neighbours.size !== 2) continue;
            const [aId, bId] = [...neighbours];
            const node = nodes[id];
            const candidate = nodes[aId].treePoint.clone()
                .add(nodes[bId].treePoint)
                .multiplyScalar(0.5)
                .lerp(node.treePoint, 0.25);
            const query = lumenField?.query?.(candidate, scratch);
            if (query && query.signedDistance < Math.max(0.1, node.clearance * 0.78)) continue;
            if (!segmentStaysInside(nodes[aId].treePoint, candidate, lumenField, wallBvh, gridSpacing)) continue;
            if (!segmentStaysInside(candidate, nodes[bId].treePoint, lumenField, wallBvh, gridSpacing)) continue;
            updates.push({ id, candidate, shift: candidate.distanceTo(node.treePoint) });
        }
        if (!updates.length) break;
        for (const update of updates) {
            nodes[update.id].treePoint.copy(update.candidate);
            movedNodeCount++;
            totalShift += update.shift;
            maxShift = Math.max(maxShift, update.shift);
        }
    }
    return {
        movedNodeCount,
        averageShift: movedNodeCount ? totalShift / movedNodeCount : 0,
        maxShift
    };
}

function collectTreeChains(adjacency) {
    const critical = new Set();
    for (const [id, neighbours] of adjacency) {
        if (neighbours.size !== 2) critical.add(id);
    }
    if (!critical.size && adjacency.size) critical.add(adjacency.keys().next().value);
    const visitedEdges = new Set();
    const chains = [];
    for (const startId of critical) {
        for (const nextId of adjacency.get(startId) || []) {
            const firstKey = edgeKey(startId, nextId);
            if (visitedEdges.has(firstKey)) continue;
            const chain = [startId];
            let previous = startId;
            let current = nextId;
            visitedEdges.add(firstKey);
            for (let guard = 0; guard < adjacency.size + 1; guard++) {
                chain.push(current);
                if (critical.has(current)) break;
                const next = [...(adjacency.get(current) || [])].find(id => id !== previous);
                if (next === undefined) break;
                visitedEdges.add(edgeKey(current, next));
                previous = current;
                current = next;
            }
            if (chain.length >= 2) chains.push(chain);
        }
    }
    return chains;
}

function resampleChain(nodes, chain, spacing) {
    const source = chain.map(id => nodes[id].treePoint.clone());
    const cumulative = [0];
    for (let index = 1; index < source.length; index++) {
        cumulative.push(cumulative[index - 1] + source[index - 1].distanceTo(source[index]));
    }
    const length = cumulative[cumulative.length - 1];
    if (length < 1e-6) return [];
    const segmentCount = Math.max(1, Math.ceil(length / spacing));
    const samples = [];
    let sourceIndex = 1;
    for (let index = 0; index <= segmentCount; index++) {
        const distance = length * index / segmentCount;
        while (sourceIndex < cumulative.length - 1 && cumulative[sourceIndex] < distance) sourceIndex++;
        const fromIndex = Math.max(0, sourceIndex - 1);
        const span = Math.max(1e-8, cumulative[sourceIndex] - cumulative[fromIndex]);
        const t = Math.max(0, Math.min(1, (distance - cumulative[fromIndex]) / span));
        samples.push(source[fromIndex].clone().lerp(source[sourceIndex], t));
    }
    return samples;
}

function treeToSegments(nodes, adjacency, lumenField, nodeSpacing) {
    const chains = collectTreeChains(adjacency);
    const criticalIds = new Set(
        [...adjacency].filter(([, neighbours]) => neighbours.size !== 2).map(([id]) => id)
    );
    const criticalNodeNames = new Map([...criticalIds].map(id => [id, `medial-junction:${id}`]));
    const segments = [];
    const scratch = {};
    let internalNodeIndex = 0;
    for (let chainIndex = 0; chainIndex < chains.length; chainIndex++) {
        const chain = chains[chainIndex];
        const samples = resampleChain(nodes, chain, nodeSpacing);
        if (samples.length < 2) continue;
        const ids = samples.map((_, index) => {
            if (index === 0 && criticalNodeNames.has(chain[0])) return criticalNodeNames.get(chain[0]);
            if (index === samples.length - 1 && criticalNodeNames.has(chain[chain.length - 1])) {
                return criticalNodeNames.get(chain[chain.length - 1]);
            }
            return `medial-node:${internalNodeIndex++}`;
        });
        const radii = samples.map(point => {
            const signedDistance = lumenField?.query?.(point, scratch)?.signedDistance;
            return Number.isFinite(signedDistance) && signedDistance > 0 ? signedDistance : 0.6;
        });
        for (let index = 0; index < samples.length - 1; index++) {
            segments.push({
                start: samples[index],
                end: samples[index + 1],
                nodeStartId: ids[index],
                nodeEndId: ids[index + 1],
                radiusStart: radii[index],
                radiusEnd: radii[index + 1],
                source: 'stl-medial-tree'
            });
        }
    }
    return { segments, chainCount: chains.length };
}

function topologyDiagnostics(adjacency) {
    let endpointCount = 0;
    let junctionCount = 0;
    for (const neighbours of adjacency.values()) {
        if (neighbours.size === 1) endpointCount++;
        else if (neighbours.size > 2) junctionCount++;
    }
    return {
        treeNodeCount: adjacency.size,
        treeEdgeCount: [...adjacency.values()].reduce((sum, neighbours) => sum + neighbours.size, 0) / 2,
        endpointCount,
        junctionCount
    };
}

export function buildMedialCenterlineTree(slices, {
    lumenField = null,
    wallBvh = null,
    gridSpacing = DEFAULT_GRID_SPACING,
    nodeSpacing = DEFAULT_NODE_SPACING,
    smoothingPasses = 4
} = {}) {
    const startedAt = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const graph = makeGraph(slices, { gridSpacing, lumenField, wallBvh });
    const connection = connectGraphComponents(graph, lumenField, wallBvh, gridSpacing);
    const graphComponentDetails = summarizeGraphComponents(graph.nodes, connection.components);
    const tree = extractMedialTree(graph, { gridSpacing });
    const adjacency = selectedAdjacency(graph.nodes, tree.selectedEdges);
    const smoothing = smoothTree(
        graph.nodes,
        adjacency,
        lumenField,
        wallBvh,
        gridSpacing,
        smoothingPasses
    );
    const output = treeToSegments(graph.nodes, adjacency, lumenField, nodeSpacing);
    const topology = topologyDiagnostics(adjacency);
    const endedAt = typeof performance !== 'undefined' ? performance.now() : Date.now();

    return {
        segments: output.segments,
        diagnostics: {
            source: 'medial-slice-teasar',
            gridSpacing,
            nodeSpacing,
            ...graph.diagnostics,
            graphComponentCount: connection.components.length,
            graphComponentSizes: connection.components.map(component => component.length),
            graphComponentDetails,
            graphConnectorCount: connection.addedEdgeCount,
            graphRoutedConnectorCount: connection.routedConnectorCount,
            graphFailedRouteCount: connection.failedRouteCount,
            ...tree.diagnostics,
            ...topology,
            chainCount: output.chainCount,
            smoothing,
            elapsedMs: endedAt - startedAt
        }
    };
}
