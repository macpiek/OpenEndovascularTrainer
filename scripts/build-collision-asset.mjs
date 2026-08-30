import crypto from 'node:crypto';
import fs from 'node:fs';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { preprocessAortaGeometry } from '../src/aortaPreprocess.js';
import { transformAortaGeometry, transformMetadataForPreprocess } from '../src/aortaTransform.js';
import {
    decodeCollisionAsset,
    encodeCollisionAsset
} from '../src/physics/collision/collisionAssetFormat.js';
import { PackedLumenField } from '../src/physics/collision/packedLumenField.js';
import { buildStlLumenCast, buildStlSliceCenterline } from '../src/stlCenterline.js';
import { generateVessel } from '../src/vesselGeometry.js';

const SOURCE_PATH = 'res/Aorta_plain.stl';
const OUTPUT_PATH = 'res/Aorta_plain.collision.bin';
const REPORT_PATH = 'out/collision-asset-report.json';
const VOXEL_SIZE = 0.5;
const BRICK_SIZE = 8;
const BRICK_WORLD_SIZE = VOXEL_SIZE * BRICK_SIZE;
const SDF_BAND = 4;
const SDF_QUANTIZATION = 0.02;
const ANATOMY_SLICE_SPACING = 0.9;
// Full lower-limb anatomy exceeds the ±655 mm range of Int16 at 0.02 mm.
// A 0.05 mm contour grid remains well below the 0.5 mm SDF voxel size while
// covering the complete bilateral tree without changing the packed format.
const LUMEN_POINT_QUANTIZATION = 0.05;
const BROAD_PHASE_CELL_SIZE = 16;
const CENTERLINE_STRIDE = 9;
// Fine bilateral limb branches and proper digital arteries add several
// thousand sparse SDF bricks. Keep the existing 0.5 mm voxel resolution and a
// bounded ceiling while allowing the complete fingers-and-toes collision field.
const MAX_DECODED_BYTES = 64 * 1024 * 1024;

function nowMs() {
    return globalThis.performance?.now?.() ?? Date.now();
}

function arrayBufferFromBuffer(buffer) {
    return buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
}

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function gridIndex(ix, iy, iz, dimensions) {
    return ix + dimensions[0] * (iy + dimensions[1] * iz);
}

function decodeGridIndex(key, dimensions) {
    const plane = dimensions[0] * dimensions[1];
    const iz = Math.floor(key / plane);
    const remaining = key - iz * plane;
    const iy = Math.floor(remaining / dimensions[0]);
    const ix = remaining - iy * dimensions[0];
    return [ix, iy, iz];
}

function alignedOrigin(min, cellSize, padding) {
    return [
        Math.floor((min.x - padding) / cellSize) * cellSize,
        Math.floor((min.y - padding) / cellSize) * cellSize,
        Math.floor((min.z - padding) / cellSize) * cellSize
    ];
}

function gridDimensions(box, origin, cellSize, padding) {
    return [
        Math.max(1, Math.ceil((box.max.x + padding - origin[0]) / cellSize)),
        Math.max(1, Math.ceil((box.max.y + padding - origin[1]) / cellSize)),
        Math.max(1, Math.ceil((box.max.z + padding - origin[2]) / cellSize))
    ];
}

function closestDistance(boundsTree, point, target) {
    target.distance = Infinity;
    const hit = boundsTree.closestPointToPoint(point, target);
    return hit?.distance ?? point.distanceTo(target.point);
}

function normalizeNodeIds(segments) {
    const ids = new Map();
    const nodeId = (segment, endpoint) => {
        const explicit = endpoint === 0 ? segment.nodeStartId : segment.nodeEndId;
        const point = endpoint === 0 ? segment.start : segment.end;
        const key = explicit === undefined || explicit === null
            ? `${point.x.toFixed(5)},${point.y.toFixed(5)},${point.z.toFixed(5)}`
            : `id:${explicit}`;
        let id = ids.get(key);
        if (id === undefined) {
            id = ids.size;
            ids.set(key, id);
        }
        return id;
    };
    const edges = new Uint32Array(segments.length * 2);
    for (let index = 0; index < segments.length; index++) {
        edges[index * 2] = nodeId(segments[index], 0);
        edges[index * 2 + 1] = nodeId(segments[index], 1);
    }
    return { edges, nodeCount: ids.size };
}

function buildCenterlineArrays(segments, boundsTree) {
    const data = new Float32Array(segments.length * CENTERLINE_STRIDE);
    const { edges, nodeCount } = normalizeNodeIds(segments);
    const point = new THREE.Vector3();
    const target = { point: new THREE.Vector3(), distance: Infinity, faceIndex: -1 };
    let minSafeRadius = Infinity;
    let maxSafeRadius = 0;

    for (let index = 0; index < segments.length; index++) {
        const segment = segments[index];
        const start = segment.start;
        const end = segment.end;
        const radiusStart = Math.max(0, segment.radiusStart ?? segment.radius ?? 0);
        const radiusEnd = Math.max(0, segment.radiusEnd ?? segment.radius ?? 0);
        const length = start.distanceTo(end);
        const samples = Math.max(2, Math.ceil(length / VOXEL_SIZE));
        let exactMinimum = Math.min(radiusStart, radiusEnd);
        for (let sampleIndex = 0; sampleIndex <= samples; sampleIndex++) {
            point.lerpVectors(start, end, sampleIndex / samples);
            exactMinimum = Math.min(exactMinimum, closestDistance(boundsTree, point, target));
        }
        const safeRadius = Math.max(0, exactMinimum - VOXEL_SIZE * 0.55);
        minSafeRadius = Math.min(minSafeRadius, safeRadius);
        maxSafeRadius = Math.max(maxSafeRadius, safeRadius);

        const offset = index * CENTERLINE_STRIDE;
        data[offset] = start.x;
        data[offset + 1] = start.y;
        data[offset + 2] = start.z;
        data[offset + 3] = end.x;
        data[offset + 4] = end.y;
        data[offset + 5] = end.z;
        data[offset + 6] = radiusStart;
        data[offset + 7] = radiusEnd;
        data[offset + 8] = safeRadius;
    }

    return {
        data,
        edges,
        nodeCount,
        minSafeRadius: Number.isFinite(minSafeRadius) ? minSafeRadius : 0,
        maxSafeRadius
    };
}

function packLumenSlices(lumenSlices, { preserveOrder = false } = {}) {
    const slices = (lumenSlices || [])
        .filter(slice => slice?.contours?.length)
        .slice();
    if (!preserveOrder) slices.sort((a, b) => a.y - b.y);
    let contourCount = 0;
    let pointCount = 0;
    for (const slice of slices) {
        contourCount += slice.contours.length;
        for (const contour of slice.contours) pointCount += contour.polygon.length;
    }

    const sliceYs = new Float32Array(slices.length);
    const sliceContourOffsets = new Uint32Array(slices.length + 1);
    const contourPointOffsets = new Uint32Array(contourCount + 1);
    const contourBounds = new Float32Array(contourCount * 4);
    const contourSamples = new Float32Array(contourCount * 2);
    const points = new Int16Array(pointCount * 2);
    let contourIndex = 0;
    let pointIndex = 0;

    for (let sliceIndex = 0; sliceIndex < slices.length; sliceIndex++) {
        const slice = slices[sliceIndex];
        sliceYs[sliceIndex] = slice.y;
        sliceContourOffsets[sliceIndex] = contourIndex;
        for (const contour of slice.contours) {
            contourPointOffsets[contourIndex] = pointIndex;
            contourBounds[contourIndex * 4] = contour.bounds.minX;
            contourBounds[contourIndex * 4 + 1] = contour.bounds.maxX;
            contourBounds[contourIndex * 4 + 2] = contour.bounds.minZ;
            contourBounds[contourIndex * 4 + 3] = contour.bounds.maxZ;
            const sample = contour.sample || contour.center || contour.centroid || contour.polygon[0];
            contourSamples[contourIndex * 2] = sample.x;
            contourSamples[contourIndex * 2 + 1] = sample.z;
            for (const point of contour.polygon) {
                const quantizedX = Math.round(point.x / LUMEN_POINT_QUANTIZATION);
                const quantizedZ = Math.round(point.z / LUMEN_POINT_QUANTIZATION);
                if (
                    quantizedX < -32768 || quantizedX > 32767 ||
                    quantizedZ < -32768 || quantizedZ > 32767
                ) {
                    throw new RangeError('Lumen contour coordinate exceeds Int16 quantization range');
                }
                points[pointIndex * 2] = quantizedX;
                points[pointIndex * 2 + 1] = quantizedZ;
                pointIndex++;
            }
            contourIndex++;
        }
    }
    sliceContourOffsets[slices.length] = contourIndex;
    contourPointOffsets[contourCount] = pointIndex;
    return {
        sliceYs,
        sliceContourOffsets,
        contourPointOffsets,
        contourBounds,
        contourSamples,
        points,
        sliceCount: slices.length,
        contourCount,
        pointCount
    };
}

function packLumenAxes(axisEntries, fallbackSlices) {
    const entries = (axisEntries || [])
        .filter(entry => entry?.slices?.some(slice => slice?.contours?.length));
    if (!entries.length) {
        entries.push({
            axis: {
                u: new THREE.Vector3(1, 0, 0),
                n: new THREE.Vector3(0, 1, 0),
                v: new THREE.Vector3(0, 0, 1)
            },
            slices: fallbackSlices || []
        });
    }

    const axisBases = new Float32Array(entries.length * 9);
    const axisSliceOffsets = new Uint32Array(entries.length + 1);
    const slices = [];
    for (let axisIndex = 0; axisIndex < entries.length; axisIndex++) {
        const entry = entries[axisIndex];
        const basis = entry.axis;
        const offset = axisIndex * 9;
        axisBases.set([
            basis.u.x, basis.u.y, basis.u.z,
            basis.n.x, basis.n.y, basis.n.z,
            basis.v.x, basis.v.y, basis.v.z
        ], offset);
        axisSliceOffsets[axisIndex] = slices.length;
        const axisSlices = entry.slices
            .filter(slice => slice?.contours?.length)
            .slice()
            .sort((a, b) => a.y - b.y);
        slices.push(...axisSlices);
    }
    axisSliceOffsets[entries.length] = slices.length;
    return {
        ...packLumenSlices(slices, { preserveOrder: true }),
        axisBases,
        axisSliceOffsets,
        axisCount: entries.length
    };
}

function packedLumenSignature(packedLumen) {
    const hash = crypto.createHash('sha256');
    for (const array of [
        packedLumen.axisBases,
        packedLumen.axisSliceOffsets,
        packedLumen.sliceYs,
        packedLumen.sliceContourOffsets,
        packedLumen.contourPointOffsets,
        packedLumen.contourBounds,
        packedLumen.points
    ]) {
        hash.update(Buffer.from(array.buffer, array.byteOffset, array.byteLength));
    }
    hash.update(String(LUMEN_POINT_QUANTIZATION));
    return hash.digest('hex');
}

function buildSdfInsideBits(sdf, packedLumen) {
    const field = new PackedLumenField(
        { lumen: { pointQuantization: LUMEN_POINT_QUANTIZATION } },
        {
            lumenAxisBases: packedLumen.axisBases,
            lumenAxisSliceOffsets: packedLumen.axisSliceOffsets,
            lumenSliceYs: packedLumen.sliceYs,
            lumenSliceContourOffsets: packedLumen.sliceContourOffsets,
            lumenContourPointOffsets: packedLumen.contourPointOffsets,
            lumenContourBounds: packedLumen.contourBounds,
            lumenContourSamples: packedLumen.contourSamples,
            lumenPoints: packedLumen.points
        }
    );
    const valuesPerBrick = BRICK_SIZE ** 3;
    const insideBits = new Uint8Array(Math.ceil(sdf.distances.length / 8));
    for (let brickIndex = 0; brickIndex < sdf.brickKeys.length; brickIndex++) {
        const [brickX, brickY, brickZ] = decodeGridIndex(sdf.brickKeys[brickIndex], sdf.dimensions);
        for (let localZ = 0; localZ < BRICK_SIZE; localZ++) {
            for (let localY = 0; localY < BRICK_SIZE; localY++) {
                for (let localX = 0; localX < BRICK_SIZE; localX++) {
                    const x = sdf.origin[0] + (brickX * BRICK_SIZE + localX) * VOXEL_SIZE;
                    const y = sdf.origin[1] + (brickY * BRICK_SIZE + localY) * VOXEL_SIZE;
                    const z = sdf.origin[2] + (brickZ * BRICK_SIZE + localZ) * VOXEL_SIZE;
                    if (!field.isInsideCoordinates(x, y, z)) continue;
                    const localIndex = localX + BRICK_SIZE * (localY + BRICK_SIZE * localZ);
                    const valueIndex = brickIndex * valuesPerBrick + localIndex;
                    insideBits[valueIndex >> 3] |= 1 << (valueIndex & 7);
                }
            }
        }
        if ((brickIndex + 1) % 1000 === 0 || brickIndex + 1 === sdf.brickKeys.length) {
            process.stdout.write(`\rSDF signs ${brickIndex + 1}/${sdf.brickKeys.length}`);
        }
    }
    process.stdout.write('\n');
    return insideBits;
}

function validateCenterlineAgainstPackedLumen(segments, wallBvh, packedLumen) {
    const field = new PackedLumenField(
        { lumen: { pointQuantization: LUMEN_POINT_QUANTIZATION } },
        {
            lumenAxisBases: packedLumen.axisBases,
            lumenAxisSliceOffsets: packedLumen.axisSliceOffsets,
            lumenSliceYs: packedLumen.sliceYs,
            lumenSliceContourOffsets: packedLumen.sliceContourOffsets,
            lumenContourPointOffsets: packedLumen.contourPointOffsets,
            lumenContourBounds: packedLumen.contourBounds,
            lumenContourSamples: packedLumen.contourSamples,
            lumenPoints: packedLumen.points
        }
    );
    const point = new THREE.Vector3();
    let wallIntersectionCount = 0;
    let acceptedInternalWallSeamCount = 0;
    let invalidSegmentCount = 0;
    const invalidSegments = [];
    for (const segment of segments) {
        const delta = new THREE.Vector3().subVectors(segment.end, segment.start);
        const length = delta.length();
        if (length < 1e-5) continue;
        const hit = wallBvh.raycastFirst(
            new THREE.Ray(segment.start, delta.multiplyScalar(1 / length)),
            THREE.DoubleSide,
            1e-4,
            Math.max(1e-4, length - 1e-4)
        );
        if (!hit) continue;
        wallIntersectionCount++;
        const sampleCount = Math.max(2, Math.ceil(length / 0.35));
        let minimumClearance = Infinity;
        let staysInside = true;
        for (let sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex++) {
            point.lerpVectors(segment.start, segment.end, sampleIndex / sampleCount);
            const sample = field.query(point);
            minimumClearance = Math.min(minimumClearance, sample?.signedDistance ?? -Infinity);
            if (!sample?.inside || sample.signedDistance < 0.1) {
                staysInside = false;
                break;
            }
        }
        if (staysInside) {
            acceptedInternalWallSeamCount++;
            continue;
        }
        invalidSegmentCount++;
        if (invalidSegments.length < 16) {
            invalidSegments.push({
                start: segment.start.toArray(),
                end: segment.end.toArray(),
                length,
                minimumClearance
            });
        }
    }
    return {
        wallIntersectionCount,
        acceptedInternalWallSeamCount,
        invalidSegmentCount,
        invalidSegments
    };
}

function segmentBounds(data, index, padding) {
    const offset = index * CENTERLINE_STRIDE;
    const radius = Math.max(data[offset + 6], data[offset + 7]) + padding;
    return {
        minX: Math.min(data[offset], data[offset + 3]) - radius,
        minY: Math.min(data[offset + 1], data[offset + 4]) - radius,
        minZ: Math.min(data[offset + 2], data[offset + 5]) - radius,
        maxX: Math.max(data[offset], data[offset + 3]) + radius,
        maxY: Math.max(data[offset + 1], data[offset + 4]) + radius,
        maxZ: Math.max(data[offset + 2], data[offset + 5]) + radius
    };
}

function buildCenterlineBroadPhase(centerlineData, box) {
    const origin = alignedOrigin(box.min, BROAD_PHASE_CELL_SIZE, SDF_BAND);
    const dimensions = gridDimensions(box, origin, BROAD_PHASE_CELL_SIZE, SDF_BAND);
    const cellCount = dimensions[0] * dimensions[1] * dimensions[2];
    const counts = new Uint32Array(cellCount);
    const segmentCount = centerlineData.length / CENTERLINE_STRIDE;

    const visitCells = (segmentIndex, visitor) => {
        const bounds = segmentBounds(centerlineData, segmentIndex, SDF_BAND);
        const minX = clamp(Math.floor((bounds.minX - origin[0]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[0] - 1);
        const minY = clamp(Math.floor((bounds.minY - origin[1]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[1] - 1);
        const minZ = clamp(Math.floor((bounds.minZ - origin[2]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[2] - 1);
        const maxX = clamp(Math.floor((bounds.maxX - origin[0]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[0] - 1);
        const maxY = clamp(Math.floor((bounds.maxY - origin[1]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[1] - 1);
        const maxZ = clamp(Math.floor((bounds.maxZ - origin[2]) / BROAD_PHASE_CELL_SIZE), 0, dimensions[2] - 1);
        for (let iz = minZ; iz <= maxZ; iz++) {
            for (let iy = minY; iy <= maxY; iy++) {
                for (let ix = minX; ix <= maxX; ix++) {
                    visitor(gridIndex(ix, iy, iz, dimensions));
                }
            }
        }
    };

    for (let segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
        visitCells(segmentIndex, cellIndex => counts[cellIndex]++);
    }

    const offsets = new Uint32Array(cellCount + 1);
    for (let index = 0; index < cellCount; index++) offsets[index + 1] = offsets[index] + counts[index];
    const ids = new Uint32Array(offsets[cellCount]);
    const cursors = offsets.slice(0, cellCount);
    for (let segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
        visitCells(segmentIndex, cellIndex => {
            ids[cursors[cellIndex]++] = segmentIndex;
        });
    }

    return { origin, dimensions, offsets, ids };
}

function collectSdfBrickKeys(geometry, origin, dimensions) {
    const positions = geometry.attributes.position;
    const padding = SDF_BAND + VOXEL_SIZE * 1.5;
    const keys = new Set();

    for (let index = 0; index < positions.count; index += 3) {
        const ax = positions.getX(index);
        const ay = positions.getY(index);
        const az = positions.getZ(index);
        const bx = positions.getX(index + 1);
        const by = positions.getY(index + 1);
        const bz = positions.getZ(index + 1);
        const cx = positions.getX(index + 2);
        const cy = positions.getY(index + 2);
        const cz = positions.getZ(index + 2);
        const minX = clamp(Math.floor((Math.min(ax, bx, cx) - padding - origin[0]) / BRICK_WORLD_SIZE), 0, dimensions[0] - 1);
        const minY = clamp(Math.floor((Math.min(ay, by, cy) - padding - origin[1]) / BRICK_WORLD_SIZE), 0, dimensions[1] - 1);
        const minZ = clamp(Math.floor((Math.min(az, bz, cz) - padding - origin[2]) / BRICK_WORLD_SIZE), 0, dimensions[2] - 1);
        const maxX = clamp(Math.floor((Math.max(ax, bx, cx) + padding - origin[0]) / BRICK_WORLD_SIZE), 0, dimensions[0] - 1);
        const maxY = clamp(Math.floor((Math.max(ay, by, cy) + padding - origin[1]) / BRICK_WORLD_SIZE), 0, dimensions[1] - 1);
        const maxZ = clamp(Math.floor((Math.max(az, bz, cz) + padding - origin[2]) / BRICK_WORLD_SIZE), 0, dimensions[2] - 1);
        for (let iz = minZ; iz <= maxZ; iz++) {
            for (let iy = minY; iy <= maxY; iy++) {
                for (let ix = minX; ix <= maxX; ix++) keys.add(gridIndex(ix, iy, iz, dimensions));
            }
        }
    }
    return [...keys].sort((a, b) => a - b);
}

function filterSdfBrickKeys(keys, origin, dimensions, boundsTree) {
    const point = new THREE.Vector3();
    const target = { point: new THREE.Vector3(), distance: Infinity, faceIndex: -1 };
    const threshold = SDF_BAND + Math.sqrt(3) * BRICK_WORLD_SIZE * 0.5 + VOXEL_SIZE;
    return keys.filter(key => {
        const [ix, iy, iz] = decodeGridIndex(key, dimensions);
        point.set(
            origin[0] + (ix + 0.5) * BRICK_WORLD_SIZE,
            origin[1] + (iy + 0.5) * BRICK_WORLD_SIZE,
            origin[2] + (iz + 0.5) * BRICK_WORLD_SIZE
        );
        return closestDistance(boundsTree, point, target) <= threshold;
    });
}

function buildSparseSdf(geometry) {
    const origin = alignedOrigin(geometry.boundingBox.min, BRICK_WORLD_SIZE, SDF_BAND + VOXEL_SIZE * 2);
    const dimensions = gridDimensions(
        geometry.boundingBox,
        origin,
        BRICK_WORLD_SIZE,
        SDF_BAND + VOXEL_SIZE * 2
    );
    const candidateKeys = collectSdfBrickKeys(geometry, origin, dimensions);
    const filteredKeys = filterSdfBrickKeys(candidateKeys, origin, dimensions, geometry.boundsTree);
    const brickKeys = Uint32Array.from(filteredKeys);
    const valuesPerBrick = BRICK_SIZE ** 3;
    const distances = new Uint8Array(brickKeys.length * valuesPerBrick);
    const point = new THREE.Vector3();
    const target = { point: new THREE.Vector3(), distance: Infinity, faceIndex: -1 };
    const clampDistance = Math.min(255 * SDF_QUANTIZATION, SDF_BAND + VOXEL_SIZE * 2);

    for (let brickIndex = 0; brickIndex < brickKeys.length; brickIndex++) {
        const [brickX, brickY, brickZ] = decodeGridIndex(brickKeys[brickIndex], dimensions);
        for (let localZ = 0; localZ < BRICK_SIZE; localZ++) {
            for (let localY = 0; localY < BRICK_SIZE; localY++) {
                for (let localX = 0; localX < BRICK_SIZE; localX++) {
                    point.set(
                        origin[0] + (brickX * BRICK_SIZE + localX) * VOXEL_SIZE,
                        origin[1] + (brickY * BRICK_SIZE + localY) * VOXEL_SIZE,
                        origin[2] + (brickZ * BRICK_SIZE + localZ) * VOXEL_SIZE
                    );
                    const unsignedDistance = Math.min(
                        clampDistance,
                        closestDistance(geometry.boundsTree, point, target)
                    );
                    const localIndex = localX + BRICK_SIZE * (localY + BRICK_SIZE * localZ);
                    const valueIndex = brickIndex * valuesPerBrick + localIndex;
                    distances[valueIndex] = Math.round(unsignedDistance / SDF_QUANTIZATION);
                }
            }
        }
        if ((brickIndex + 1) % 500 === 0 || brickIndex + 1 === brickKeys.length) {
            process.stdout.write(`\rSDF bricks ${brickIndex + 1}/${brickKeys.length}`);
        }
    }
    process.stdout.write('\n');
    return {
        origin,
        dimensions,
        brickKeys,
        distances,
        candidateBrickCount: candidateKeys.length
    };
}

function loadReusableSdf(stlSha256, transform, lumenSignature) {
    if (!fs.existsSync(OUTPUT_PATH)) return null;
    try {
        const bytes = fs.readFileSync(OUTPUT_PATH);
        const previous = decodeCollisionAsset(arrayBufferFromBuffer(bytes));
        const metadata = previous.metadata;
        const sdf = metadata?.sdf;
        const compatible = (
            metadata?.source?.stlSha256 === stlSha256 &&
            JSON.stringify(metadata?.transform) === JSON.stringify(transform) &&
            sdf?.voxelSize === VOXEL_SIZE &&
            sdf?.brickSize === BRICK_SIZE &&
            sdf?.band === SDF_BAND &&
            (sdf?.distanceQuantization ?? sdf?.quantization) === SDF_QUANTIZATION &&
            previous.arrays?.sdfBrickKeys?.length === sdf?.brickCount &&
            previous.arrays?.sdfDistances?.length === sdf.brickCount * BRICK_SIZE ** 3
        );
        if (!compatible) return null;
        const expectedSignBytes = Math.ceil(previous.arrays.sdfDistances.length / 8);
        const reusableSigns = (
            metadata?.lumen?.signature === lumenSignature &&
            previous.arrays?.sdfInsideBits?.length === expectedSignBytes
        ) ? previous.arrays.sdfInsideBits : null;
        return {
            origin: sdf.origin,
            dimensions: sdf.dimensions,
            brickKeys: previous.arrays.sdfBrickKeys,
            distances: previous.arrays.sdfDistances,
            insideBits: reusableSigns,
            candidateBrickCount: sdf.candidateBrickCount,
            reused: true,
            signsReused: reusableSigns !== null
        };
    } catch (error) {
        console.warn(`Existing collision SDF cannot be reused: ${error.message}`);
        return null;
    }
}

function loadReusablePackedLumen(stlSha256, transform) {
    if (!fs.existsSync(OUTPUT_PATH)) return null;
    try {
        const bytes = fs.readFileSync(OUTPUT_PATH);
        const previous = decodeCollisionAsset(arrayBufferFromBuffer(bytes));
        const metadata = previous.metadata;
        const lumen = metadata?.lumen;
        const arrays = previous.arrays;
        const compatible = (
            metadata?.source?.stlSha256 === stlSha256 &&
            JSON.stringify(metadata?.transform) === JSON.stringify(transform) &&
            lumen?.pointQuantization === LUMEN_POINT_QUANTIZATION &&
            arrays?.lumenAxisBases?.length === lumen.axisCount * 9 &&
            arrays?.lumenAxisSliceOffsets?.length === lumen.axisCount + 1 &&
            arrays?.lumenSliceYs?.length === lumen.sliceCount &&
            arrays?.lumenSliceContourOffsets?.length === lumen.sliceCount + 1 &&
            arrays?.lumenContourPointOffsets?.length === lumen.contourCount + 1 &&
            arrays?.lumenContourBounds?.length === lumen.contourCount * 4 &&
            arrays?.lumenContourSamples?.length === lumen.contourCount * 2 &&
            arrays?.lumenPoints?.length === lumen.pointCount * 2
        );
        if (!compatible) return null;
        return {
            axisBases: arrays.lumenAxisBases,
            axisSliceOffsets: arrays.lumenAxisSliceOffsets,
            sliceYs: arrays.lumenSliceYs,
            sliceContourOffsets: arrays.lumenSliceContourOffsets,
            contourPointOffsets: arrays.lumenContourPointOffsets,
            contourBounds: arrays.lumenContourBounds,
            contourSamples: arrays.lumenContourSamples,
            points: arrays.lumenPoints,
            axisCount: lumen.axisCount,
            sliceCount: lumen.sliceCount,
            contourCount: lumen.contourCount,
            pointCount: lumen.pointCount,
            signature: lumen.signature,
            diagnostics: lumen.diagnostics,
            reused: true
        };
    } catch (error) {
        console.warn(`Existing packed lumen cannot be reused: ${error.message}`);
        return null;
    }
}

const started = nowMs();
const sourceBytes = fs.readFileSync(SOURCE_PATH);
const stlSha256 = crypto.createHash('sha256').update(sourceBytes).digest('hex');
const arrayBuffer = sourceBytes.buffer.slice(sourceBytes.byteOffset, sourceBytes.byteOffset + sourceBytes.byteLength);
const geometry = new STLLoader().parse(arrayBuffer);
const { vessel } = generateVessel(140, 0);
const transform = transformAortaGeometry(geometry, vessel);

const preprocessStarted = nowMs();
const preprocessing = preprocessAortaGeometry(geometry, {
    transform: transformMetadataForPreprocess(transform)
});
const preprocessMs = nowMs() - preprocessStarted;

const centerlineStarted = nowMs();
const centerline = buildStlSliceCenterline(geometry, {
    lumenField: preprocessing.lumenField,
    sliceSpacing: ANATOMY_SLICE_SPACING
});
const centerlineMs = nowMs() - centerlineStarted;
const centerlineArrays = buildCenterlineArrays(centerline.segments, geometry.boundsTree);
const broadPhase = buildCenterlineBroadPhase(centerlineArrays.data, geometry.boundingBox);

const lumenStarted = nowMs();
const reusablePackedLumen = loadReusablePackedLumen(stlSha256, transform);
const collisionLumen = reusablePackedLumen ? null : buildStlLumenCast(geometry, {
    lumenField: preprocessing.lumenField,
    sliceSpacing: ANATOMY_SLICE_SPACING,
    fieldOnly: true
});
const packedLumen = reusablePackedLumen ||
    packLumenAxes(collisionLumen.axisSlices, collisionLumen.slices);
const lumenSignature = reusablePackedLumen?.signature || packedLumenSignature(packedLumen);
const lumenMs = nowMs() - lumenStarted;
const packedCenterlineValidation = validateCenterlineAgainstPackedLumen(
    centerline.segments,
    geometry.boundsTree,
    packedLumen
);
const centerlineDiagnostics = {
    ...centerline.diagnostics,
    centerlineWallIntersectionCount:
        packedCenterlineValidation.wallIntersectionCount,
    centerlineAcceptedInternalWallSeamCount:
        packedCenterlineValidation.acceptedInternalWallSeamCount,
    centerlineInvalidSegmentCountFinal:
        packedCenterlineValidation.invalidSegmentCount,
    centerlineInvalidSegmentDetailsFinal:
        packedCenterlineValidation.invalidSegments,
    wallValidation: 'stl-bvh-with-packed-lumen-seam-check'
};

const sdfStarted = nowMs();
const sdf = loadReusableSdf(stlSha256, transform, lumenSignature) || buildSparseSdf(geometry);
const sdfMs = nowMs() - sdfStarted;
const sdfSignStarted = nowMs();
if (!sdf.insideBits) sdf.insideBits = buildSdfInsideBits(sdf, packedLumen);
const sdfSignMs = nowMs() - sdfSignStarted;
const decodedBytes = centerlineArrays.data.byteLength + centerlineArrays.edges.byteLength +
    broadPhase.offsets.byteLength + broadPhase.ids.byteLength + sdf.brickKeys.byteLength +
    sdf.distances.byteLength + sdf.insideBits.byteLength + packedLumen.sliceYs.byteLength +
    packedLumen.axisBases.byteLength + packedLumen.axisSliceOffsets.byteLength +
    packedLumen.sliceContourOffsets.byteLength + packedLumen.contourPointOffsets.byteLength +
    packedLumen.contourBounds.byteLength + packedLumen.contourSamples.byteLength + packedLumen.points.byteLength;
if (decodedBytes > MAX_DECODED_BYTES) {
    throw new Error(
        `Decoded collision asset is ${(decodedBytes / 1048576).toFixed(2)} MB; ` +
        `limit is ${MAX_DECODED_BYTES / 1048576} MB`
    );
}

const metadata = {
    source: {
        path: SOURCE_PATH,
        byteLength: sourceBytes.byteLength,
        stlSha256,
        triangleCount: geometry.attributes.position.count / 3
    },
    transform,
    bounds: {
        min: geometry.boundingBox.min.toArray(),
        max: geometry.boundingBox.max.toArray()
    },
    centerline: {
        stride: CENTERLINE_STRIDE,
        segmentCount: centerline.segments.length,
        nodeCount: centerlineArrays.nodeCount,
        minSafeRadius: centerlineArrays.minSafeRadius,
        maxSafeRadius: centerlineArrays.maxSafeRadius,
        diagnostics: centerlineDiagnostics
    },
    broadPhase: {
        cellSize: BROAD_PHASE_CELL_SIZE,
        origin: broadPhase.origin,
        dimensions: broadPhase.dimensions,
        entryCount: broadPhase.ids.length
    },
    lumen: {
        axisCount: packedLumen.axisCount,
        sliceCount: packedLumen.sliceCount,
        contourCount: packedLumen.contourCount,
        pointCount: packedLumen.pointCount,
        pointQuantization: LUMEN_POINT_QUANTIZATION,
        signature: lumenSignature,
        diagnostics: collisionLumen?.diagnostics || reusablePackedLumen?.diagnostics || null
    },
    sdf: {
        voxelSize: VOXEL_SIZE,
        brickSize: BRICK_SIZE,
        band: SDF_BAND,
        quantization: SDF_QUANTIZATION,
        distanceQuantization: SDF_QUANTIZATION,
        origin: sdf.origin,
        dimensions: sdf.dimensions,
        brickCount: sdf.brickKeys.length,
        candidateBrickCount: sdf.candidateBrickCount,
        signEncoding: 'inside-bitset-lsb',
        signByteLength: sdf.insideBits.byteLength
    },
    decodedBytes,
    timings: {
        preprocessMs,
        centerlineMs,
        lumenMs,
        lumenReused: reusablePackedLumen?.reused === true,
        sdfMs,
        sdfReused: sdf.reused === true,
        sdfSignMs,
        sdfSignsReused: sdf.signsReused === true,
        totalMs: nowMs() - started
    }
};

const encoded = encodeCollisionAsset(metadata, {
    centerlineSegments: centerlineArrays.data,
    centerlineEdges: centerlineArrays.edges,
    broadPhaseOffsets: broadPhase.offsets,
    broadPhaseIds: broadPhase.ids,
    lumenAxisBases: packedLumen.axisBases,
    lumenAxisSliceOffsets: packedLumen.axisSliceOffsets,
    lumenSliceYs: packedLumen.sliceYs,
    lumenSliceContourOffsets: packedLumen.sliceContourOffsets,
    lumenContourPointOffsets: packedLumen.contourPointOffsets,
    lumenContourBounds: packedLumen.contourBounds,
    lumenContourSamples: packedLumen.contourSamples,
    lumenPoints: packedLumen.points,
    sdfBrickKeys: sdf.brickKeys,
    sdfDistances: sdf.distances,
    sdfInsideBits: sdf.insideBits
});
fs.writeFileSync(OUTPUT_PATH, Buffer.from(encoded));
fs.mkdirSync('out', { recursive: true });
fs.writeFileSync(REPORT_PATH, JSON.stringify({
    ...metadata,
    encodedBytes: encoded.byteLength
}, null, 2));

console.log(JSON.stringify({
    output: OUTPUT_PATH,
    report: REPORT_PATH,
    centerlineSegments: centerline.segments.length,
    sdfBricks: sdf.brickKeys.length,
    decodedMB: decodedBytes / 1048576,
    encodedMB: encoded.byteLength / 1048576,
    timings: metadata.timings
}, null, 2));
