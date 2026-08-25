const LEAF_SENTINEL = 0xffff;
const NODE_WORDS = 8;
const TRIANGLE_CACHE_SIZE = 1 << 14;
const TRIANGLE_CACHE_MASK = TRIANGLE_CACHE_SIZE - 1;
const TRIANGLE_CACHE_STRIDE = 9;

export function createScalarMeshBvhScratch(maximumDepth = 64) {
    const capacity = Math.max(8, Math.ceil(maximumDepth) + 2);
    const scratch = {
        nodeStack: new Int32Array(capacity),
        distanceStack: new Float64Array(capacity),
        closestX: 0,
        closestY: 0,
        closestZ: 0,
        closestDistanceSquared: Infinity,
        closestFaceIndex: -1,
        boundsTree: null,
        indices: null,
        positions: null,
        triangleCount: 0,
        rootFloat32: [],
        rootUint32: [],
        rootUint16: [],
        triangleCacheKeys: new Int32Array(TRIANGLE_CACHE_SIZE),
        // The source STL positions are Float32.  Cache those vertices at their
        // native precision and reconstruct edges in JS doubles on every hit.
        // This is numerically identical to a cache miss, while doubling the
        // direct-mapped working set at the same byte cost as the previous
        // Float64 edge cache.
        triangleCacheValues: new Float32Array(
            TRIANGLE_CACHE_SIZE * TRIANGLE_CACHE_STRIDE
        )
    };
    scratch.triangleCacheKeys.fill(-1);
    return scratch;
}

function pointBoxDistanceSquared(x, y, z, bounds, node) {
    const dx = x < bounds[node]
        ? bounds[node] - x
        : x > bounds[node + 3]
            ? x - bounds[node + 3]
            : 0;
    const dy = y < bounds[node + 1]
        ? bounds[node + 1] - y
        : y > bounds[node + 4]
            ? y - bounds[node + 4]
            : 0;
    const dz = z < bounds[node + 2]
        ? bounds[node + 2] - z
        : z > bounds[node + 5]
            ? z - bounds[node + 5]
            : 0;
    return dx * dx + dy * dy + dz * dz;
}

function considerTriangle(
    pointX,
    pointY,
    pointZ,
    triangleIndex,
    indices,
    positions,
    scratch
) {
    const cacheSlot = triangleIndex & TRIANGLE_CACHE_MASK;
    const cacheOffset = cacheSlot * TRIANGLE_CACHE_STRIDE;
    const cacheValues = scratch.triangleCacheValues;
    let ax;
    let ay;
    let az;
    let bx;
    let by;
    let bz;
    let cx;
    let cy;
    let cz;
    let abx;
    let aby;
    let abz;
    let acx;
    let acy;
    let acz;
    if (scratch.triangleCacheKeys[cacheSlot] === triangleIndex) {
        ax = cacheValues[cacheOffset];
        ay = cacheValues[cacheOffset + 1];
        az = cacheValues[cacheOffset + 2];
        bx = cacheValues[cacheOffset + 3];
        by = cacheValues[cacheOffset + 4];
        bz = cacheValues[cacheOffset + 5];
        cx = cacheValues[cacheOffset + 6];
        cy = cacheValues[cacheOffset + 7];
        cz = cacheValues[cacheOffset + 8];
        abx = bx - ax;
        aby = by - ay;
        abz = bz - az;
        acx = cx - ax;
        acy = cy - ay;
        acz = cz - az;
    } else {
        const base = triangleIndex * 3;
        const ia = indices ? indices[base] * 3 : base * 3;
        const ib = indices ? indices[base + 1] * 3 : (base + 1) * 3;
        const ic = indices ? indices[base + 2] * 3 : (base + 2) * 3;
        ax = positions[ia];
        ay = positions[ia + 1];
        az = positions[ia + 2];
        bx = positions[ib];
        by = positions[ib + 1];
        bz = positions[ib + 2];
        cx = positions[ic];
        cy = positions[ic + 1];
        cz = positions[ic + 2];
        abx = bx - ax;
        aby = by - ay;
        abz = bz - az;
        acx = cx - ax;
        acy = cy - ay;
        acz = cz - az;
        scratch.triangleCacheKeys[cacheSlot] = triangleIndex;
        cacheValues[cacheOffset] = ax;
        cacheValues[cacheOffset + 1] = ay;
        cacheValues[cacheOffset + 2] = az;
        cacheValues[cacheOffset + 3] = bx;
        cacheValues[cacheOffset + 4] = by;
        cacheValues[cacheOffset + 5] = bz;
        cacheValues[cacheOffset + 6] = cx;
        cacheValues[cacheOffset + 7] = cy;
        cacheValues[cacheOffset + 8] = cz;
    }
    const apx = pointX - ax;
    const apy = pointY - ay;
    const apz = pointZ - az;
    const d1 = abx * apx + aby * apy + abz * apz;
    const d2 = acx * apx + acy * apy + acz * apz;
    let closestX;
    let closestY;
    let closestZ;
    if (d1 <= 0 && d2 <= 0) {
        closestX = ax;
        closestY = ay;
        closestZ = az;
    } else {
        const bpx = pointX - bx;
        const bpy = pointY - by;
        const bpz = pointZ - bz;
        const d3 = abx * bpx + aby * bpy + abz * bpz;
        const d4 = acx * bpx + acy * bpy + acz * bpz;
        if (d3 >= 0 && d4 <= d3) {
            closestX = bx;
            closestY = by;
            closestZ = bz;
        } else {
            const vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0) {
                const v = d1 / (d1 - d3);
                closestX = ax + abx * v;
                closestY = ay + aby * v;
                closestZ = az + abz * v;
            } else {
                const cpx = pointX - cx;
                const cpy = pointY - cy;
                const cpz = pointZ - cz;
                const d5 = abx * cpx + aby * cpy + abz * cpz;
                const d6 = acx * cpx + acy * cpy + acz * cpz;
                if (d6 >= 0 && d5 <= d6) {
                    closestX = cx;
                    closestY = cy;
                    closestZ = cz;
                } else {
                    const vb = d5 * d2 - d1 * d6;
                    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
                        const w = d2 / (d2 - d6);
                        closestX = ax + acx * w;
                        closestY = ay + acy * w;
                        closestZ = az + acz * w;
                    } else {
                        const va = d3 * d6 - d5 * d4;
                        if (
                            va <= 0 &&
                            d4 - d3 >= 0 &&
                            d5 - d6 >= 0
                        ) {
                            const w = (d4 - d3) /
                                ((d4 - d3) + (d5 - d6));
                            closestX = bx + (cx - bx) * w;
                            closestY = by + (cy - by) * w;
                            closestZ = bz + (cz - bz) * w;
                        } else {
                            const inverseDenominator = 1 / (va + vb + vc);
                            const v = vb * inverseDenominator;
                            const w = vc * inverseDenominator;
                            closestX = ax + abx * v + acx * w;
                            closestY = ay + aby * v + acy * w;
                            closestZ = az + abz * v + acz * w;
                        }
                    }
                }
            }
        }
    }
    const dx = pointX - closestX;
    const dy = pointY - closestY;
    const dz = pointZ - closestZ;
    const distanceSquared = dx * dx + dy * dy + dz * dz;
    if (distanceSquared >= scratch.closestDistanceSquared) return;
    scratch.closestDistanceSquared = distanceSquared;
    scratch.closestX = closestX;
    scratch.closestY = closestY;
    scratch.closestZ = closestZ;
    scratch.closestFaceIndex = triangleIndex;
}

/**
 * Allocation-free specialization of MeshBVH.closestPointToPoint for static,
 * non-interleaved triangle geometry. It traverses the same packed node bounds
 * and evaluates the same Ericson point-triangle regions as THREE.Triangle.
 * Returns false when the BVH layout is unsupported so callers can retain the
 * library implementation as an exact fallback.
 */
export function closestPointToPointScalarBvh(
    boundsTree,
    pointX,
    pointY,
    pointZ,
    target,
    scratch,
    maximumDistance = Infinity,
    hintFaceIndex = -1
) {
    let indices;
    let positions;
    if (scratch.boundsTree !== boundsTree) {
        const geometry = boundsTree?.geometry;
        const position = geometry?.attributes?.position;
        const roots = boundsTree?._roots;
        if (
            !geometry ||
            boundsTree.indirect ||
            position?.isInterleavedBufferAttribute ||
            position?.itemSize !== 3 ||
            !position.array ||
            !Array.isArray(roots) ||
            roots.length === 0
        ) return false;
        const index = geometry.index;
        if (index?.isInterleavedBufferAttribute) return false;
        indices = index?.array ?? null;
        positions = position.array;
        scratch.boundsTree = boundsTree;
        scratch.indices = indices;
        scratch.positions = positions;
        scratch.triangleCount = Math.floor(
            (indices?.length ?? positions.length / 3) / 3
        );
        scratch.triangleCacheKeys.fill(-1);
        scratch.rootFloat32.length = roots.length;
        scratch.rootUint32.length = roots.length;
        scratch.rootUint16.length = roots.length;
        for (let rootIndex = 0; rootIndex < roots.length; rootIndex++) {
            scratch.rootFloat32[rootIndex] = new Float32Array(roots[rootIndex]);
            scratch.rootUint32[rootIndex] = new Uint32Array(roots[rootIndex]);
            scratch.rootUint16[rootIndex] = new Uint16Array(roots[rootIndex]);
        }
    } else {
        // The specialization is explicitly for a static mesh.  Reuse the
        // validated typed-array views instead of walking the Three.js object
        // graph for every wall-contact projection.
        indices = scratch.indices;
        positions = scratch.positions;
    }
    const maximumDistanceSquared = maximumDistance * maximumDistance;
    scratch.closestDistanceSquared = Infinity;
    scratch.closestFaceIndex = -1;
    let seededHintFaceIndex = -1;
    if (
        Number.isInteger(hintFaceIndex) &&
        hintFaceIndex >= 0 &&
        hintFaceIndex < scratch.triangleCount
    ) {
        seededHintFaceIndex = hintFaceIndex;
        considerTriangle(
            pointX,
            pointY,
            pointZ,
            hintFaceIndex,
            indices,
            positions,
            scratch
        );
    }
    const nodeStack = scratch.nodeStack;
    const distanceStack = scratch.distanceStack;
    for (
        let rootIndex = 0;
        rootIndex < scratch.rootFloat32.length;
        rootIndex++
    ) {
        const bounds = scratch.rootFloat32[rootIndex];
        const uint32 = scratch.rootUint32[rootIndex];
        const uint16 = scratch.rootUint16[rootIndex];
        let stackSize = 1;
        nodeStack[0] = 0;
        distanceStack[0] = 0;
        while (stackSize > 0) {
            stackSize--;
            const node = nodeStack[stackSize];
            const nodeDistanceSquared = distanceStack[stackSize];
            if (
                nodeDistanceSquared >= scratch.closestDistanceSquared ||
                nodeDistanceSquared >= maximumDistanceSquared
            ) continue;
            const node16 = node * 2;
            if (uint16[node16 + 15] === LEAF_SENTINEL) {
                const offset = uint32[node + 6];
                const count = uint16[node16 + 14];
                for (let triangle = offset; triangle < offset + count; triangle++) {
                    // The hinted triangle already seeded the exact same point
                    // and strict-distance bound before traversal. Repeating it
                    // cannot replace itself because MeshBVH uses a strict '<'
                    // comparison as well.
                    if (triangle === seededHintFaceIndex) continue;
                    considerTriangle(
                        pointX,
                        pointY,
                        pointZ,
                        triangle,
                        indices,
                        positions,
                        scratch
                    );
                }
                continue;
            }
            const left = node + NODE_WORDS;
            const right = uint32[node + 6];
            const leftDistanceSquared = pointBoxDistanceSquared(
                pointX,
                pointY,
                pointZ,
                bounds,
                left
            );
            const rightDistanceSquared = pointBoxDistanceSquared(
                pointX,
                pointY,
                pointZ,
                bounds,
                right
            );
            const near = leftDistanceSquared <= rightDistanceSquared
                ? left
                : right;
            const nearDistanceSquared = leftDistanceSquared <= rightDistanceSquared
                ? leftDistanceSquared
                : rightDistanceSquared;
            const far = near === left ? right : left;
            const farDistanceSquared = near === left
                ? rightDistanceSquared
                : leftDistanceSquared;
            if (
                farDistanceSquared < scratch.closestDistanceSquared &&
                farDistanceSquared < maximumDistanceSquared
            ) {
                if (stackSize >= nodeStack.length) return false;
                nodeStack[stackSize] = far;
                distanceStack[stackSize] = farDistanceSquared;
                stackSize++;
            }
            if (
                nearDistanceSquared < scratch.closestDistanceSquared &&
                nearDistanceSquared < maximumDistanceSquared
            ) {
                if (stackSize >= nodeStack.length) return false;
                nodeStack[stackSize] = near;
                distanceStack[stackSize] = nearDistanceSquared;
                stackSize++;
            }
        }
    }
    if (scratch.closestFaceIndex < 0) return false;
    target.point.set(
        scratch.closestX,
        scratch.closestY,
        scratch.closestZ
    );
    target.distance = Math.sqrt(scratch.closestDistanceSquared);
    target.faceIndex = scratch.closestFaceIndex;
    return true;
}
