import * as THREE from 'three';
import { decodeCollisionAsset } from './collisionAssetFormat.js';
import { createPackedLumenField, createPackedLumenQueryResult } from './packedLumenField.js';
import {
    closestPointToPointScalarBvh,
    createScalarMeshBvhScratch
} from './scalarMeshBvhClosestPoint.js';

const SOURCE_SAFE_CORE = 'centerline-safe-core';
const SOURCE_SDF = 'sparse-sdf';
const SOURCE_SDF_BVH = 'sparse-sdf-bvh';
const SOURCE_FALLBACK = 'fallback';
const SOURCE_CENTERLINE = 'centerline-estimate';
const EPSILON = 1e-8;
const SIGN_CACHE_SIZE = 1 << 13;
const SIGN_CACHE_SET_COUNT = SIGN_CACHE_SIZE >> 1;
const SIGN_CACHE_SET_MASK = SIGN_CACHE_SET_COUNT - 1;
const SIGN_CACHE_INV_SPACING = 200;
const SDF_MISSING_BRICK_16 = 0xffff;
const SDF_MISSING_BRICK_32 = 0xffffffff;
const STAT_POINT_QUERIES = 0;
const STAT_CAPSULE_QUERIES = 1;
const STAT_CAPSULE_SAMPLES = 2;
const STAT_SWEEP_QUERIES = 3;
const STAT_SWEEP_SAMPLES = 4;
const STAT_BATCH_QUERIES = 5;
const STAT_SAFE_CORE_HITS = 6;
const STAT_SDF_HITS = 7;
const STAT_BVH_REFINEMENTS = 8;
const STAT_SIGN_REFINEMENTS = 9;
const STAT_SIGN_CACHE_HITS = 10;
const STAT_SIGN_CACHE_MISSES = 11;
const STAT_FALLBACK_HITS = 12;
const STAT_CENTERLINE_ESTIMATE_HITS = 13;
const STAT_RESULT_ALLOCATIONS = 14;
const STAT_BVH_CLEARANCE_REFINEMENTS = 15;
const STAT_BVH_CONTACT_REFINEMENTS = 16;
const STAT_KNOWN_INSIDE_NEAR_WALL_HITS = 17;
const STAT_COUNT = 18;
const CONTACT_SIGNED_DISTANCE = 0;
const CONTACT_SIGNED_GAP = 1;
const CONTACT_DISTANCE = 2;
const CONTACT_PENETRATION = 3;
const CONTACT_BRANCH_ID = 4;
const CONTACT_SEGMENT_T = 5;
const CONTACT_TIME_OF_IMPACT = 6;
const CENTERLINE_BRANCH_ID = 0;
const CENTERLINE_T = 1;
const CENTERLINE_SIGNED_DISTANCE = 2;
const CENTERLINE_SAFE_DISTANCE = 3;
const CENTERLINE_SAFE_BRANCH_ID = 4;
const CENTERLINE_SAFE_INWARD_X = 5;
const CENTERLINE_SAFE_INWARD_Y = 6;
const CENTERLINE_SAFE_INWARD_Z = 7;
const CENTERLINE_NEAREST_DISTANCE = 8;
const CENTERLINE_INWARD_X = 9;
const CENTERLINE_INWARD_Y = 10;
const CENTERLINE_INWARD_Z = 11;
const DISTANCE_SIGNED_DISTANCE = 0;
const DISTANCE_INWARD_X = 1;
const DISTANCE_INWARD_Y = 2;
const DISTANCE_INWARD_Z = 3;
const DISTANCE_BRANCH_ID = 4;
const FINE_BROAD_PHASE_SCALE = 4;
const FINE_CELLS_PER_BROAD_CELL = FINE_BROAD_PHASE_SCALE ** 3;
// Keep exact BVH clearance certificates at their original 0.25 mm seed cap;
// increasing it changes which exact wall checks run and therefore the solve.
const INSIDE_CLEARANCE_CERTIFICATE_MM = 0.25;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function setVector(target, x, y, z) {
    target.x = x;
    target.y = y;
    target.z = z;
    return target;
}

function setContact(target, source) {
    target.inside = source.inside;
    target.violation = source.violation;
    target.conservative = source.conservative;
    target.source = source.source;
    target.faceIndex = source.faceIndex;
    target.insideClearance = source.insideClearance;
    target.capsuleSampleCount = source.capsuleSampleCount;
    const targetValues = target.values;
    const sourceValues = source.values;
    targetValues[0] = sourceValues[0];
    targetValues[1] = sourceValues[1];
    targetValues[2] = sourceValues[2];
    targetValues[3] = sourceValues[3];
    targetValues[4] = sourceValues[4];
    targetValues[5] = sourceValues[5];
    targetValues[6] = sourceValues[6];
    copyContactVector(target.point.values, source.point.values);
    copyContactVector(target.target.values, source.target.values);
    copyContactVector(target.closestPoint.values, source.closestPoint.values);
    copyContactVector(target.normal.values, source.normal.values);
    copyContactVector(target.inward.values, source.inward.values);
    return target;
}

function copyContactVector(target, source) {
    target[0] = source[0];
    target[1] = source[1];
    target[2] = source[2];
}

function considerKnownInsideBranchSegment(
    field,
    segmentId,
    x,
    y,
    z,
    scratch
) {
    const prepared = field.centerlinePrepared;
    const offset = segmentId * field.centerlinePreparedStride;
    const ax = prepared[offset];
    const ay = prepared[offset + 1];
    const az = prepared[offset + 2];
    const dx = prepared[offset + 3];
    const dy = prepared[offset + 4];
    const dz = prepared[offset + 5];
    const lengthSquared = prepared[offset + 6];
    const t = clamp(
        ((x - ax) * dx + (y - ay) * dy + (z - az) * dz) /
            Math.max(EPSILON, lengthSquared),
        0,
        1
    );
    const radialX = ax + dx * t - x;
    const radialY = ay + dy * t - y;
    const radialZ = az + dz * t - z;
    const radialDistanceSquared =
        radialX * radialX +
        radialY * radialY +
        radialZ * radialZ;
    const radius = prepared[offset + 7] * (1 - t) +
        prepared[offset + 8] * t;
    const safeRadius = prepared[offset + 9];
    const bestBranchId = scratch[0];
    const bestSignedDistance = scratch[1];
    const bestSafeDistance = scratch[2];
    if (bestBranchId >= 0) {
        const signedThreshold = radius - bestSignedDistance;
        const safeThreshold = safeRadius - bestSafeDistance;
        const roundoffGuard = (
            radialDistanceSquared +
            signedThreshold * signedThreshold +
            safeThreshold * safeThreshold +
            1
        ) * 1e-13;
        const cannotImproveSigned = signedThreshold < 0 ||
            radialDistanceSquared >
                signedThreshold * signedThreshold + roundoffGuard;
        const cannotImproveSafe = safeThreshold < 0 ||
            radialDistanceSquared >
                safeThreshold * safeThreshold + roundoffGuard;
        if (cannotImproveSigned && cannotImproveSafe) return;
    }
    const radialDistance = Math.sqrt(radialDistanceSquared);
    const signedDistance = radius - radialDistance;
    const safeDistance = safeRadius - radialDistance;
    if (safeDistance > bestSafeDistance) scratch[2] = safeDistance;
    if (
        bestBranchId < 0 ||
        signedDistance > bestSignedDistance ||
        (signedDistance === bestSignedDistance && segmentId < bestBranchId)
    ) {
        scratch[0] = segmentId;
        scratch[1] = signedDistance;
    }
}

class ContactVector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.values = new Float64Array([x, y, z]);
    }

    get x() { return this.values[0]; }
    set x(value) { this.values[0] = value; }
    get y() { return this.values[1]; }
    set y(value) { this.values[1] = value; }
    get z() { return this.values[2]; }
    set z(value) { this.values[2] = value; }
}

class ContactResult {
    constructor() {
        this.values = new Float64Array([-Infinity, -Infinity, Infinity, Infinity, -1, 0, 1]);
        this.inside = false;
        this.violation = false;
        this.conservative = false;
        this.source = SOURCE_FALLBACK;
        this.faceIndex = -1;
        this.insideClearance = 0;
        this.capsuleSampleCount = 0;
        this.point = new ContactVector3();
        this.target = new ContactVector3();
        this.closestPoint = new ContactVector3();
        this.normal = new ContactVector3(1, 0, 0);
        this.inward = new ContactVector3(1, 0, 0);
    }

    get signedDistance() { return this.values[0]; }
    set signedDistance(value) { this.values[0] = value; }
    get signedGap() { return this.values[1]; }
    set signedGap(value) { this.values[1] = value; }
    get distance() { return this.values[2]; }
    set distance(value) { this.values[2] = value; }
    get penetration() { return this.values[3]; }
    set penetration(value) { this.values[3] = value; }
    get branchId() { return this.values[4]; }
    set branchId(value) { this.values[4] = value; }
    get segmentT() { return this.values[5]; }
    set segmentT(value) { this.values[5] = value; }
    get timeOfImpact() { return this.values[6]; }
    set timeOfImpact(value) { this.values[6] = value; }
}

class CenterlineQueryState {
    constructor() {
        this.values = new Float64Array(12);
        this.found = false;
        this.branchId = -1;
        this.signedDistance = -Infinity;
        this.safeDistance = -Infinity;
        this.safeBranchId = -1;
        this.safeInwardX = 1;
        this.nearestDistance = Infinity;
        this.inwardX = 1;
    }

    get branchId() { return this.values[0]; }
    set branchId(value) { this.values[0] = value; }
    get t() { return this.values[1]; }
    set t(value) { this.values[1] = value; }
    get signedDistance() { return this.values[2]; }
    set signedDistance(value) { this.values[2] = value; }
    get safeDistance() { return this.values[3]; }
    set safeDistance(value) { this.values[3] = value; }
    get safeBranchId() { return this.values[4]; }
    set safeBranchId(value) { this.values[4] = value; }
    get safeInwardX() { return this.values[5]; }
    set safeInwardX(value) { this.values[5] = value; }
    get safeInwardY() { return this.values[6]; }
    set safeInwardY(value) { this.values[6] = value; }
    get safeInwardZ() { return this.values[7]; }
    set safeInwardZ(value) { this.values[7] = value; }
    get nearestDistance() { return this.values[8]; }
    set nearestDistance(value) { this.values[8] = value; }
    get inwardX() { return this.values[9]; }
    set inwardX(value) { this.values[9] = value; }
    get inwardY() { return this.values[10]; }
    set inwardY(value) { this.values[10] = value; }
    get inwardZ() { return this.values[11]; }
    set inwardZ(value) { this.values[11] = value; }
}

class DistanceQueryState {
    constructor() {
        this.values = new Float64Array([-Infinity, 1, 0, 0, -1]);
        this.conservative = false;
        this.source = SOURCE_FALLBACK;
        this.faceIndex = -1;
    }

    get signedDistance() { return this.values[0]; }
    set signedDistance(value) { this.values[0] = value; }
    get inwardX() { return this.values[1]; }
    set inwardX(value) { this.values[1] = value; }
    get inwardY() { return this.values[2]; }
    set inwardY(value) { this.values[2] = value; }
    get inwardZ() { return this.values[3]; }
    set inwardZ(value) { this.values[3] = value; }
    get branchId() { return this.values[4]; }
    set branchId(value) { this.values[4] = value; }
}

export function createContactResult() {
    return new ContactResult();
}

export function createBatchContactOutput(capacity) {
    return {
        signedDistances: new Float32Array(capacity),
        signedGaps: new Float32Array(capacity),
        penetrations: new Float32Array(capacity),
        normals: new Float32Array(capacity * 3),
        targets: new Float32Array(capacity * 3),
        branchIds: new Int32Array(capacity),
        violations: new Uint8Array(capacity),
        count: 0
    };
}

function createStats() {
    return new Uint32Array(STAT_COUNT);
}

export class VesselContactField {
    constructor(asset, {
        fallbackCollider = null,
        fallbackGeometry = null,
        bvhValidationDistance = 0.85,
        capsuleBvhValidation = true
    } = {}) {
        const decoded = asset instanceof ArrayBuffer ? decodeCollisionAsset(asset) : asset;
        if (!decoded?.metadata || !decoded?.arrays) throw new TypeError('Decoded collision asset is required');

        this.metadata = decoded.metadata;
        this.arrays = decoded.arrays;
        this.fallbackCollider = fallbackCollider;
        this.fallbackGeometry = fallbackGeometry;
        this.bvhValidationDistance = bvhValidationDistance;
        this.capsuleBvhValidationGap = capsuleBvhValidation === true
            ? 0.05
            : Number.isFinite(capsuleBvhValidation)
                ? capsuleBvhValidation
                : -Infinity;
        this.packedLumenField = createPackedLumenField(decoded.metadata, decoded.arrays);
        this.centerline = decoded.arrays.centerlineSegments;
        this.centerlineStride = decoded.metadata.centerline.stride;
        const centerlineSegmentCount =
            this.centerline.length / this.centerlineStride;
        // Runtime queries visit the same centerline segments millions of
        // times. Expand the packed endpoints once into exact Float64 segment
        // coefficients so the hot path does not repeatedly subtract endpoints
        // and rebuild squared lengths. Float64 preserves the previous JS
        // arithmetic and therefore the contact result.
        this.centerlinePreparedStride = 10;
        this.centerlinePrepared = new Float64Array(
            centerlineSegmentCount * this.centerlinePreparedStride
        );
        this.centerlineBoundsStride = 8;
        this.centerlineBounds = new Float32Array(
            centerlineSegmentCount * this.centerlineBoundsStride
        );
        for (let segmentId = 0; segmentId < centerlineSegmentCount; segmentId++) {
            const source = segmentId * this.centerlineStride;
            const target = segmentId * this.centerlinePreparedStride;
            const bounds = segmentId * this.centerlineBoundsStride;
            const ax = this.centerline[source];
            const ay = this.centerline[source + 1];
            const az = this.centerline[source + 2];
            const bx = this.centerline[source + 3];
            const by = this.centerline[source + 4];
            const bz = this.centerline[source + 5];
            const dx = bx - ax;
            const dy = by - ay;
            const dz = bz - az;
            this.centerlinePrepared[target] = ax;
            this.centerlinePrepared[target + 1] = ay;
            this.centerlinePrepared[target + 2] = az;
            this.centerlinePrepared[target + 3] = dx;
            this.centerlinePrepared[target + 4] = dy;
            this.centerlinePrepared[target + 5] = dz;
            this.centerlinePrepared[target + 6] =
                dx * dx + dy * dy + dz * dz;
            this.centerlinePrepared[target + 7] = this.centerline[source + 6];
            this.centerlinePrepared[target + 8] = this.centerline[source + 7];
            this.centerlinePrepared[target + 9] = this.centerline[source + 8];
            this.centerlineBounds[bounds] = Math.min(ax, bx);
            this.centerlineBounds[bounds + 1] = Math.min(ay, by);
            this.centerlineBounds[bounds + 2] = Math.min(az, bz);
            this.centerlineBounds[bounds + 3] = Math.max(ax, bx);
            this.centerlineBounds[bounds + 4] = Math.max(ay, by);
            this.centerlineBounds[bounds + 5] = Math.max(az, bz);
            this.centerlineBounds[bounds + 6] = Math.max(
                this.centerline[source + 6],
                this.centerline[source + 7]
            );
            this.centerlineBounds[bounds + 7] = this.centerline[source + 8];
        }
        this.broadPhaseOffsets = decoded.arrays.broadPhaseOffsets;
        this.broadPhaseIds = decoded.arrays.broadPhaseIds;
        this.sdfBrickKeys = decoded.arrays.sdfBrickKeys;
        this.sdfDistances = decoded.arrays.sdfDistances;
        this.sdfInsideBits = decoded.arrays.sdfInsideBits || null;

        const sdf = decoded.metadata.sdf;
        this.voxelSize = sdf.voxelSize;
        this.brickSize = sdf.brickSize;
        this.valuesPerBrick = this.brickSize ** 3;
        this.sdfQuantization = sdf.distanceQuantization ?? sdf.quantization;
        this.sdfBvhDistancePadding =
            this.voxelSize * Math.sqrt(3) +
            Math.max(0, this.sdfQuantization || 0);
        this.capsuleBvhValidationDistance = Math.min(
            this.bvhValidationDistance,
            0.25
        );
        this.sdfOrigin = sdf.origin;
        this.sdfDimensions = sdf.dimensions;
        const lookupLength = this.sdfDimensions[0] * this.sdfDimensions[1] * this.sdfDimensions[2];
        const SdfLookupArray = this.sdfBrickKeys.length < SDF_MISSING_BRICK_16
            ? Uint16Array
            : Uint32Array;
        this.sdfMissingBrick = SdfLookupArray === Uint16Array
            ? SDF_MISSING_BRICK_16
            : SDF_MISSING_BRICK_32;
        this.sdfBrickLookup = new SdfLookupArray(lookupLength);
        this.sdfBrickLookup.fill(this.sdfMissingBrick);
        for (let index = 0; index < this.sdfBrickKeys.length; index++) {
            this.sdfBrickLookup[this.sdfBrickKeys[index]] = index;
        }
        this.signCacheKeyLow = new Int32Array(SIGN_CACHE_SIZE);
        this.signCacheKeyHigh = new Int32Array(SIGN_CACHE_SIZE);
        this.signCacheInside = new Uint8Array(SIGN_CACHE_SIZE);
        this.signCacheValid = new Uint8Array(SIGN_CACHE_SIZE);
        this.signCacheVictim = new Uint8Array(SIGN_CACHE_SET_COUNT);

        const broadPhase = decoded.metadata.broadPhase;
        this.broadPhaseOrigin = broadPhase.origin;
        this.broadPhaseDimensions = broadPhase.dimensions;
        this.broadPhaseCellSize = broadPhase.cellSize;
        // Candidate membership is generated from each padded segment AABB.
        // Retain those six integer cell bounds so a persistent branch hint can
        // be validated in O(1) instead of linearly scanning the cell list on
        // every capsule sample.
        const CenterlineCellBoundsArray = this.broadPhaseDimensions.every(
            dimension => dimension <= 0xff
        ) ? Uint8Array : Uint16Array;
        this.centerlineCellBounds = new CenterlineCellBoundsArray(
            centerlineSegmentCount * 6
        );
        const centerlinePadding = Math.max(
            0,
            decoded.metadata.sdf?.band ?? 0
        );
        this.centerlinePadding = centerlinePadding;
        this.centerlineSafeWithinRadius = true;
        for (let segmentId = 0; segmentId < centerlineSegmentCount; segmentId++) {
            const source = segmentId * this.centerlineStride;
            const target = segmentId * 6;
            const radius = Math.max(
                this.centerline[source + 6],
                this.centerline[source + 7]
            ) + centerlinePadding;
            this.centerlineSafeWithinRadius =
                this.centerlineSafeWithinRadius &&
                this.centerline[source + 8] <= radius - centerlinePadding;
            const minX = Math.floor((
                Math.min(this.centerline[source], this.centerline[source + 3]) -
                    radius - this.broadPhaseOrigin[0]
            ) / this.broadPhaseCellSize);
            const minY = Math.floor((
                Math.min(this.centerline[source + 1], this.centerline[source + 4]) -
                    radius - this.broadPhaseOrigin[1]
            ) / this.broadPhaseCellSize);
            const minZ = Math.floor((
                Math.min(this.centerline[source + 2], this.centerline[source + 5]) -
                    radius - this.broadPhaseOrigin[2]
            ) / this.broadPhaseCellSize);
            const maxX = Math.floor((
                Math.max(this.centerline[source], this.centerline[source + 3]) +
                    radius - this.broadPhaseOrigin[0]
            ) / this.broadPhaseCellSize);
            const maxY = Math.floor((
                Math.max(this.centerline[source + 1], this.centerline[source + 4]) +
                    radius - this.broadPhaseOrigin[1]
            ) / this.broadPhaseCellSize);
            const maxZ = Math.floor((
                Math.max(this.centerline[source + 2], this.centerline[source + 5]) +
                    radius - this.broadPhaseOrigin[2]
            ) / this.broadPhaseCellSize);
            this.centerlineCellBounds[target] = clamp(
                minX,
                0,
                this.broadPhaseDimensions[0] - 1
            );
            this.centerlineCellBounds[target + 1] = clamp(
                minY,
                0,
                this.broadPhaseDimensions[1] - 1
            );
            this.centerlineCellBounds[target + 2] = clamp(
                minZ,
                0,
                this.broadPhaseDimensions[2] - 1
            );
            this.centerlineCellBounds[target + 3] = clamp(
                maxX,
                0,
                this.broadPhaseDimensions[0] - 1
            );
            this.centerlineCellBounds[target + 4] = clamp(
                maxY,
                0,
                this.broadPhaseDimensions[1] - 1
            );
            this.centerlineCellBounds[target + 5] = clamp(
                maxZ,
                0,
                this.broadPhaseDimensions[2] - 1
            );
        }
        // Near-wall closure repeatedly queries the same dense aortic cells.
        // A finer sparse grid contains the exact subset of padded segment
        // AABBs around the point. It is used only after a persistent branch
        // proves a signed-distance winner within the same padding. Segments
        // outside the cell cannot improve that winner or the safe core.
        this.centerlineFineCellSize =
            this.broadPhaseCellSize / FINE_BROAD_PHASE_SCALE;
        this.centerlineFineDimensions = this.broadPhaseDimensions.map(
            dimension => dimension * FINE_BROAD_PHASE_SCALE
        );
        const fineCellCount = this.centerlineFineDimensions[0] *
            this.centerlineFineDimensions[1] *
            this.centerlineFineDimensions[2];
        const fineCounts = new Uint32Array(fineCellCount);
        const fineCellHalfDiagonal =
            this.centerlineFineCellSize * Math.sqrt(3) * 0.5;
        const visitFineCells = callback => {
            for (
                let segmentId = 0;
                segmentId < centerlineSegmentCount;
                segmentId++
            ) {
                const source = segmentId * this.centerlineStride;
                const radius = Math.max(
                    this.centerline[source + 6],
                    this.centerline[source + 7]
                ) + centerlinePadding;
                const ax = this.centerline[source];
                const ay = this.centerline[source + 1];
                const az = this.centerline[source + 2];
                const dx = this.centerline[source + 3] - ax;
                const dy = this.centerline[source + 4] - ay;
                const dz = this.centerline[source + 5] - az;
                const lengthSquared = dx * dx + dy * dy + dz * dz;
                const maximumCenterDistance =
                    radius + fineCellHalfDiagonal;
                const maximumCenterDistanceSquared =
                    maximumCenterDistance * maximumCenterDistance;
                const minimumX = clamp(Math.floor((
                    Math.min(
                        this.centerline[source],
                        this.centerline[source + 3]
                    ) - radius - this.broadPhaseOrigin[0]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[0] - 1);
                const minimumY = clamp(Math.floor((
                    Math.min(
                        this.centerline[source + 1],
                        this.centerline[source + 4]
                    ) - radius - this.broadPhaseOrigin[1]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[1] - 1);
                const minimumZ = clamp(Math.floor((
                    Math.min(
                        this.centerline[source + 2],
                        this.centerline[source + 5]
                    ) - radius - this.broadPhaseOrigin[2]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[2] - 1);
                const maximumX = clamp(Math.floor((
                    Math.max(
                        this.centerline[source],
                        this.centerline[source + 3]
                    ) + radius - this.broadPhaseOrigin[0]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[0] - 1);
                const maximumY = clamp(Math.floor((
                    Math.max(
                        this.centerline[source + 1],
                        this.centerline[source + 4]
                    ) + radius - this.broadPhaseOrigin[1]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[1] - 1);
                const maximumZ = clamp(Math.floor((
                    Math.max(
                        this.centerline[source + 2],
                        this.centerline[source + 5]
                    ) + radius - this.broadPhaseOrigin[2]
                ) / this.centerlineFineCellSize), 0, this.centerlineFineDimensions[2] - 1);
                for (let cellZ = minimumZ; cellZ <= maximumZ; cellZ++) {
                    const centerZ = this.broadPhaseOrigin[2] +
                        (cellZ + 0.5) * this.centerlineFineCellSize;
                    for (let cellY = minimumY; cellY <= maximumY; cellY++) {
                        const centerY = this.broadPhaseOrigin[1] +
                            (cellY + 0.5) * this.centerlineFineCellSize;
                        for (let cellX = minimumX; cellX <= maximumX; cellX++) {
                            const centerX = this.broadPhaseOrigin[0] +
                                (cellX + 0.5) *
                                    this.centerlineFineCellSize;
                            const t = clamp((
                                (centerX - ax) * dx +
                                (centerY - ay) * dy +
                                (centerZ - az) * dz
                            ) / Math.max(EPSILON, lengthSquared), 0, 1);
                            const radialX = centerX - (ax + dx * t);
                            const radialY = centerY - (ay + dy * t);
                            const radialZ = centerZ - (az + dz * t);
                            const centerDistanceSquared =
                                radialX * radialX +
                                radialY * radialY +
                                radialZ * radialZ;
                            // Every point in the voxel lies within its
                            // circumscribed sphere. If even that sphere cannot
                            // touch the segment's padded influence radius, the
                            // segment cannot improve the already-proven hinted
                            // winner for any query in this cell. This tightens
                            // the static candidate set without approximating a
                            // runtime distance or changing candidate order.
                            if (
                                centerDistanceSquared >
                                    maximumCenterDistanceSquared
                            ) continue;
                            callback(
                                cellX + this.centerlineFineDimensions[0] * (
                                    cellY + this.centerlineFineDimensions[1] * cellZ
                                ),
                                segmentId
                            );
                        }
                    }
                }
            }
        };
        visitFineCells(cell => {
            fineCounts[cell]++;
        });
        let fineNonemptyCellCount = 0;
        let fineIdCount = 0;
        for (let cell = 0; cell < fineCellCount; cell++) {
            if (fineCounts[cell] > 0) fineNonemptyCellCount++;
            fineIdCount += fineCounts[cell];
        }
        const fineKeys = new Uint32Array(fineNonemptyCellCount);
        const fineIdOffsets = new Uint32Array(
            fineNonemptyCellCount + 1
        );
        const fineSlots = new Int32Array(fineCellCount);
        fineSlots.fill(-1);
        let fineSlot = 0;
        for (let cell = 0; cell < fineCellCount; cell++) {
            const count = fineCounts[cell];
            if (count === 0) continue;
            fineKeys[fineSlot] = cell;
            fineSlots[cell] = fineSlot;
            fineIdOffsets[fineSlot + 1] =
                fineIdOffsets[fineSlot] + count;
            fineSlot++;
        }
        const fineIds = centerlineSegmentCount <= 0xffff
            ? new Uint16Array(fineIdCount)
            : new Uint32Array(fineIdCount);
        const fineCursors = fineIdOffsets.slice(
            0,
            fineNonemptyCellCount
        );
        visitFineCells((cell, segmentId) => {
            const slot = fineSlots[cell];
            fineIds[fineCursors[slot]++] = segmentId;
        });
        this.centerlineFineRunOffsets = new Uint32Array(
            fineNonemptyCellCount + 1
        );
        let fineRunCount = 0;
        for (let slot = 0; slot < fineNonemptyCellCount; slot++) {
            const start = fineIdOffsets[slot];
            const end = fineIdOffsets[slot + 1];
            let previousId = -2;
            for (let entry = start; entry < end; entry++) {
                const segmentId = fineIds[entry];
                if (segmentId !== previousId + 1) fineRunCount++;
                previousId = segmentId;
            }
            this.centerlineFineRunOffsets[slot + 1] = fineRunCount;
        }
        const FineRunArray = centerlineSegmentCount <= 0xffff
            ? Uint16Array
            : Uint32Array;
        this.centerlineFineRuns = new FineRunArray(fineRunCount * 2);
        let fineRun = 0;
        for (let slot = 0; slot < fineNonemptyCellCount; slot++) {
            const start = fineIdOffsets[slot];
            const end = fineIdOffsets[slot + 1];
            let runStart = -1;
            let runLength = 0;
            for (let entry = start; entry < end; entry++) {
                const segmentId = fineIds[entry];
                if (runLength > 0 && segmentId !== runStart + runLength) {
                    this.centerlineFineRuns[fineRun * 2] = runStart;
                    this.centerlineFineRuns[fineRun * 2 + 1] = runLength;
                    fineRun++;
                    runStart = segmentId;
                    runLength = 1;
                } else {
                    if (runLength === 0) runStart = segmentId;
                    runLength++;
                }
            }
            if (runLength > 0) {
                this.centerlineFineRuns[fineRun * 2] = runStart;
                this.centerlineFineRuns[fineRun * 2 + 1] = runLength;
                fineRun++;
            }
        }
        // Every fine cell belongs to exactly one broad-phase cell. Store only
        // the 4x4x4 local slot blocks of broad cells that actually contain a
        // centerline candidate. This is an exact two-level lookup: unlike the
        // previous high-load hash table it has no probes, collisions or
        // candidate reordering.
        const FineBlockArray = fineNonemptyCellCount <= 0x7fff
            ? Int16Array
            : Int32Array;
        this.centerlineFineBlockByBroadCell = new FineBlockArray(
            this.broadPhaseOffsets.length - 1
        );
        this.centerlineFineBlockByBroadCell.fill(-1);
        const finePlaneForLookup =
            this.centerlineFineDimensions[0] *
            this.centerlineFineDimensions[1];
        let fineBlockCount = 0;
        for (let slot = 0; slot < fineNonemptyCellCount; slot++) {
            const cell = fineKeys[slot];
            const fineZ = Math.floor(cell / finePlaneForLookup);
            const remainder = cell - fineZ * finePlaneForLookup;
            const fineY = Math.floor(
                remainder / this.centerlineFineDimensions[0]
            );
            const fineX = remainder -
                fineY * this.centerlineFineDimensions[0];
            const broadCell = Math.floor(
                fineX / FINE_BROAD_PHASE_SCALE
            ) + this.broadPhaseDimensions[0] * (
                Math.floor(fineY / FINE_BROAD_PHASE_SCALE) +
                this.broadPhaseDimensions[1] *
                    Math.floor(fineZ / FINE_BROAD_PHASE_SCALE)
            );
            if (this.centerlineFineBlockByBroadCell[broadCell] < 0) {
                this.centerlineFineBlockByBroadCell[broadCell] =
                    fineBlockCount++;
            }
        }
        const FineSlotArray = fineNonemptyCellCount < 0xffff
            ? Uint16Array
            : Uint32Array;
        this.centerlineFineSlotByBlock = new FineSlotArray(
            fineBlockCount * FINE_CELLS_PER_BROAD_CELL
        );
        for (let slot = 0; slot < fineNonemptyCellCount; slot++) {
            const cell = fineKeys[slot];
            const fineZ = Math.floor(cell / finePlaneForLookup);
            const remainder = cell - fineZ * finePlaneForLookup;
            const fineY = Math.floor(
                remainder / this.centerlineFineDimensions[0]
            );
            const fineX = remainder -
                fineY * this.centerlineFineDimensions[0];
            const broadX = Math.floor(fineX / FINE_BROAD_PHASE_SCALE);
            const broadY = Math.floor(fineY / FINE_BROAD_PHASE_SCALE);
            const broadZ = Math.floor(fineZ / FINE_BROAD_PHASE_SCALE);
            const broadCell = broadX + this.broadPhaseDimensions[0] * (
                broadY + this.broadPhaseDimensions[1] * broadZ
            );
            const localX = fineX - broadX * FINE_BROAD_PHASE_SCALE;
            const localY = fineY - broadY * FINE_BROAD_PHASE_SCALE;
            const localZ = fineZ - broadZ * FINE_BROAD_PHASE_SCALE;
            const localCell = localX + FINE_BROAD_PHASE_SCALE * (
                localY + FINE_BROAD_PHASE_SCALE * localZ
            );
            const block = this.centerlineFineBlockByBroadCell[broadCell];
            this.centerlineFineSlotByBlock[
                block * FINE_CELLS_PER_BROAD_CELL + localCell
            ] = slot + 1;
        }
        this.broadPhaseOrderedIds = centerlineSegmentCount <= 0xffff
            ? new Uint16Array(this.broadPhaseIds)
            : new Uint32Array(this.broadPhaseIds);
        const broadPhasePlane =
            this.broadPhaseDimensions[0] * this.broadPhaseDimensions[1];
        for (let cell = 0; cell + 1 < this.broadPhaseOffsets.length; cell++) {
            const start = this.broadPhaseOffsets[cell];
            const end = this.broadPhaseOffsets[cell + 1];
            if (end - start < 2) continue;
            const cellZ = Math.floor(cell / broadPhasePlane);
            const cellRemainder = cell - cellZ * broadPhasePlane;
            const cellY = Math.floor(
                cellRemainder / this.broadPhaseDimensions[0]
            );
            const cellX = cellRemainder -
                cellY * this.broadPhaseDimensions[0];
            const centerX = this.broadPhaseOrigin[0] +
                (cellX + 0.5) * this.broadPhaseCellSize;
            const centerY = this.broadPhaseOrigin[1] +
                (cellY + 0.5) * this.broadPhaseCellSize;
            const centerZ = this.broadPhaseOrigin[2] +
                (cellZ + 0.5) * this.broadPhaseCellSize;
            const ordered = Array.from(
                this.broadPhaseOrderedIds.subarray(start, end)
            );
            ordered.sort((first, second) => {
                const score = segmentId => {
                    const bounds = segmentId * this.centerlineBoundsStride;
                    const dx = centerX < this.centerlineBounds[bounds]
                        ? this.centerlineBounds[bounds] - centerX
                        : centerX > this.centerlineBounds[bounds + 3]
                            ? centerX - this.centerlineBounds[bounds + 3]
                            : 0;
                    const dy = centerY < this.centerlineBounds[bounds + 1]
                        ? this.centerlineBounds[bounds + 1] - centerY
                        : centerY > this.centerlineBounds[bounds + 4]
                            ? centerY - this.centerlineBounds[bounds + 4]
                            : 0;
                    const dz = centerZ < this.centerlineBounds[bounds + 2]
                        ? this.centerlineBounds[bounds + 2] - centerZ
                        : centerZ > this.centerlineBounds[bounds + 5]
                            ? centerZ - this.centerlineBounds[bounds + 5]
                            : 0;
                    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    const source = segmentId * this.centerlineStride;
                    return Math.max(
                        this.centerlineBounds[bounds + 6],
                        this.centerline[source + 8]
                    ) - distance;
                };
                const difference = score(second) - score(first);
                return difference || first - second;
            });
            this.broadPhaseOrderedIds.set(ordered, start);
        }
        this._sdfCornerScratch = new Float64Array(8);
        this._scalarBvhScratch = createScalarMeshBvhScratch();
        this.runtimeBytes = decoded.metadata.decodedBytes + this.sdfBrickLookup.byteLength +
            this.signCacheKeyLow.byteLength + this.signCacheKeyHigh.byteLength +
            this.signCacheInside.byteLength + this.signCacheValid.byteLength +
            this.signCacheVictim.byteLength + this._sdfCornerScratch.byteLength +
            this.centerlinePrepared.byteLength +
            this.centerlineBounds.byteLength +
            this.broadPhaseOrderedIds.byteLength +
            this.centerlineCellBounds.byteLength +
            this.centerlineFineBlockByBroadCell.byteLength +
            this.centerlineFineSlotByBlock.byteLength +
            this.centerlineFineRunOffsets.byteLength +
            this.centerlineFineRuns.byteLength +
            this._scalarBvhScratch.nodeStack.byteLength +
            this._scalarBvhScratch.distanceStack.byteLength +
            this._scalarBvhScratch.triangleCacheKeys.byteLength +
            this._scalarBvhScratch.triangleCacheValues.byteLength;
        this.stats = createStats();
        this._centerlineState = new CenterlineQueryState();
        this._distanceState = new DistanceQueryState();
        this._point = { x: 0, y: 0, z: 0 };
        this._skipBvhValidation = false;
        this._bvhPoint = new THREE.Vector3();
        this._bvhClosest = { point: new THREE.Vector3(), distance: Infinity, faceIndex: -1 };
        this._lumenQuery = createPackedLumenQueryResult();
        this._capsuleContact = createContactResult();
        this._capsuleEndpointContact = createContactResult();
        this._capsuleEndpointX = NaN;
        this._capsuleEndpointY = NaN;
        this._capsuleEndpointZ = NaN;
        this._capsuleEndpointRadius = NaN;
        this._capsuleBranchHint = -1;
        this._sweepContact = createContactResult();
        this._sweepProbe = createContactResult();
        this._fallbackContact = {
            query: {
                inward: { x: 0, y: 0, z: 0 },
                normal: { x: 0, y: 0, z: 0 },
                closestPoint: { x: 0, y: 0, z: 0 }
            },
            target: { x: 0, y: 0, z: 0 },
            closestPoint: { x: 0, y: 0, z: 0 },
            inward: { x: 0, y: 0, z: 0 },
            normal: { x: 0, y: 0, z: 0 }
        };
    }

    resetStats() {
        this.stats.fill(0);
    }

    getStats() {
        const stats = this.stats;
        return {
            pointQueries: stats[STAT_POINT_QUERIES],
            capsuleQueries: stats[STAT_CAPSULE_QUERIES],
            capsuleSamples: stats[STAT_CAPSULE_SAMPLES],
            sweepQueries: stats[STAT_SWEEP_QUERIES],
            sweepSamples: stats[STAT_SWEEP_SAMPLES],
            batchQueries: stats[STAT_BATCH_QUERIES],
            safeCoreHits: stats[STAT_SAFE_CORE_HITS],
            sdfHits: stats[STAT_SDF_HITS],
            bvhRefinements: stats[STAT_BVH_REFINEMENTS],
            signRefinements: stats[STAT_SIGN_REFINEMENTS],
            signCacheHits: stats[STAT_SIGN_CACHE_HITS],
            signCacheMisses: stats[STAT_SIGN_CACHE_MISSES],
            fallbackHits: stats[STAT_FALLBACK_HITS],
            centerlineEstimateHits: stats[STAT_CENTERLINE_ESTIMATE_HITS],
            resultAllocations: stats[STAT_RESULT_ALLOCATIONS],
            bvhClearanceRefinements:
                stats[STAT_BVH_CLEARANCE_REFINEMENTS],
            bvhContactRefinements:
                stats[STAT_BVH_CONTACT_REFINEMENTS],
            knownInsideNearWallHits:
                stats[STAT_KNOWN_INSIDE_NEAR_WALL_HITS],
            runtimeBytes: this.runtimeBytes
        };
    }

    setFallbackCollider(collider) {
        this.fallbackCollider = collider;
    }

    setFallbackGeometry(geometry) {
        this.fallbackGeometry = geometry;
    }

    querySphere(position, radius = 0, out = null) {
        if (!out) this.stats[STAT_RESULT_ALLOCATIONS]++;
        const target = out || createContactResult();
        const x = position.x ?? position[0] ?? 0;
        const y = position.y ?? position[1] ?? 0;
        const z = position.z ?? position[2] ?? 0;
        return this.#querySphereCoordinates(x, y, z, Math.max(0, radius || 0), target);
    }

    #querySphereCoordinates(
        x,
        y,
        z,
        toolRadius,
        target,
        knownInside = false,
        knownBranchId = -1,
        knownNearWall = false
    ) {
        this.stats[STAT_POINT_QUERIES]++;
        const state = this.#queryDistance(
            x,
            y,
            z,
            toolRadius,
            knownInside,
            knownBranchId,
            knownNearWall
        );
        const stateValues = state.values;
        const signedDistance = stateValues[DISTANCE_SIGNED_DISTANCE];
        const signedGap = signedDistance - toolRadius;
        const penetration = Math.max(0, -signedGap);
        const violation = signedGap < 0;
        const nx = stateValues[DISTANCE_INWARD_X];
        const ny = stateValues[DISTANCE_INWARD_Y];
        const nz = stateValues[DISTANCE_INWARD_Z];
        const targetValues = target.values;

        target.inside = signedDistance >= 0;
        target.violation = violation;
        target.conservative = state.conservative;
        target.faceIndex = state.faceIndex;
        targetValues[CONTACT_SIGNED_DISTANCE] = signedDistance;
        targetValues[CONTACT_SIGNED_GAP] = signedGap;
        targetValues[CONTACT_DISTANCE] = Math.max(0, signedDistance);
        targetValues[CONTACT_PENETRATION] = penetration;
        targetValues[CONTACT_BRANCH_ID] = stateValues[DISTANCE_BRANCH_ID];
        targetValues[CONTACT_SEGMENT_T] = 0;
        targetValues[CONTACT_TIME_OF_IMPACT] = violation ? 0 : 1;
        target.source = state.source;
        const pointValues = target.point.values;
        pointValues[0] = x;
        pointValues[1] = y;
        pointValues[2] = z;
        const normalValues = target.normal.values;
        normalValues[0] = nx;
        normalValues[1] = ny;
        normalValues[2] = nz;
        const inwardValues = target.inward.values;
        inwardValues[0] = nx;
        inwardValues[1] = ny;
        inwardValues[2] = nz;
        const closestValues = target.closestPoint.values;
        closestValues[0] = x - nx * signedDistance;
        closestValues[1] = y - ny * signedDistance;
        closestValues[2] = z - nz * signedDistance;
        const correctionValues = target.target.values;
        correctionValues[0] = x + nx * penetration;
        correctionValues[1] = y + ny * penetration;
        correctionValues[2] = z + nz * penetration;
        return target;
    }

    queryCapsule(start, end, radius = 0, out = null) {
        const ax = start.x ?? start[0] ?? 0;
        const ay = start.y ?? start[1] ?? 0;
        const az = start.z ?? start[2] ?? 0;
        const bx = end.x ?? end[0] ?? 0;
        const by = end.y ?? end[1] ?? 0;
        const bz = end.z ?? end[2] ?? 0;
        return this.queryCapsuleCoordinates(ax, ay, az, bx, by, bz, radius, out);
    }

    queryCapsuleCoordinates(ax, ay, az, bx, by, bz, radius = 0, out = null) {
        return this.#queryCapsuleStored(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            radius,
            out
        );
    }

    queryCapsuleSoA(
        x,
        y,
        z,
        radii,
        index,
        out = null,
        hintFaceIndex = -1,
        knownInside = false,
        measureInsideClearance = false,
        hintBranchId = -1,
        knownNearWall = false,
        precomputedLength = -1,
        precomputedSampleCount = 0
    ) {
        // Reuse the exact closest triangle from this material segment's
        // previous query as a branch-and-bound upper bound. The BVH still
        // searches for every closer triangle, so this changes traversal cost
        // without changing the geometric result.
        if (Number.isInteger(hintFaceIndex) && hintFaceIndex >= 0) {
            this._bvhClosest.faceIndex = hintFaceIndex;
        }
        this._capsuleBranchHint = hintBranchId;
        return this.#queryCapsuleStored(
            x[index],
            y[index],
            z[index],
            x[index + 1],
            y[index + 1],
            z[index + 1],
            Math.max(radii[index], radii[index + 1]),
            out,
            knownInside,
            measureInsideClearance,
            knownNearWall,
            precomputedLength,
            precomputedSampleCount
        );
    }

    #queryCapsuleStored(
        ax,
        ay,
        az,
        bx,
        by,
        bz,
        radius,
        out,
        knownInside = false,
        measureInsideClearance = false,
        knownNearWall = false,
        precomputedLength = -1,
        precomputedSampleCount = 0
    ) {
        if (!out) this.stats[STAT_RESULT_ALLOCATIONS]++;
        const target = out || createContactResult();
        const toolRadius = Math.max(0, radius || 0);
        const dx = bx - ax;
        const dy = by - ay;
        const dz = bz - az;
        const hasPrecomputedSampling = precomputedSampleCount > 0;
        const length = hasPrecomputedSampling
            ? precomputedLength
            : Math.sqrt(dx * dx + dy * dy + dz * dz);
        const sampleCount = hasPrecomputedSampling
            ? precomputedSampleCount
            : Math.max(
                1,
                Math.ceil(
                    length /
                    Math.max(
                        this.voxelSize * 4,
                        Math.max(0.5, toolRadius)
                    )
                )
            );
        let bestGap = Infinity;
        let exactInsideClearance = 0;
        let allSamplesInside = true;
        let bestT = 0;
        let bestSampleIndex = 0;
        this.stats[STAT_CAPSULE_QUERIES]++;

        this._skipBvhValidation = true;
        const reusesPreviousEndpoint =
            ax === this._capsuleEndpointX &&
            ay === this._capsuleEndpointY &&
            az === this._capsuleEndpointZ &&
            toolRadius === this._capsuleEndpointRadius;
        let contact;
        if (reusesPreviousEndpoint) {
            contact = setContact(this._capsuleContact, this._capsuleEndpointContact);
        } else {
            contact = this.#querySphereCoordinates(
                ax,
                ay,
                az,
                toolRadius,
                this._capsuleContact,
                knownInside,
                this._capsuleBranchHint,
                knownNearWall
            );
            this.stats[STAT_CAPSULE_SAMPLES]++;
        }
        bestGap = contact.values[CONTACT_SIGNED_GAP];
        if (measureInsideClearance) allSamplesInside = contact.inside;
        const startGap = bestGap;
        const startNormalX = contact.inward.values[0];
        const startNormalY = contact.inward.values[1];
        const startNormalZ = contact.inward.values[2];
        let endGap = startGap;
        let normalAgreement = 1;
        setContact(target, contact);

        if (length > EPSILON) {
            contact = this.#querySphereCoordinates(
                bx,
                by,
                bz,
                toolRadius,
                this._capsuleContact,
                knownInside,
                this._capsuleBranchHint,
                knownNearWall
            );
            this.stats[STAT_CAPSULE_SAMPLES]++;
            if (measureInsideClearance) {
                allSamplesInside = allSamplesInside && contact.inside;
            }
            setContact(this._capsuleEndpointContact, contact);
            this._capsuleEndpointX = bx;
            this._capsuleEndpointY = by;
            this._capsuleEndpointZ = bz;
            this._capsuleEndpointRadius = toolRadius;
            endGap = contact.values[CONTACT_SIGNED_GAP];
            normalAgreement =
                startNormalX * contact.inward.values[0] +
                startNormalY * contact.inward.values[1] +
                startNormalZ * contact.inward.values[2];
            if (endGap < bestGap) {
                bestGap = endGap;
                bestT = 1;
                bestSampleIndex = sampleCount;
                setContact(target, contact);
            }
        }
        if (length <= EPSILON) {
            setContact(this._capsuleEndpointContact, contact);
            this._capsuleEndpointX = bx;
            this._capsuleEndpointY = by;
            this._capsuleEndpointZ = bz;
            this._capsuleEndpointRadius = toolRadius;
        }

        const needsInteriorSamples = sampleCount > 1 &&
            (Math.min(startGap, endGap) <= this.voxelSize || normalAgreement < 0.85);
        for (let sampleIndex = 1; needsInteriorSamples && sampleIndex < sampleCount; sampleIndex++) {
            const t = sampleIndex / sampleCount;
            contact = this.#querySphereCoordinates(
                ax + dx * t,
                ay + dy * t,
                az + dz * t,
                toolRadius,
                this._capsuleContact,
                knownInside,
                this._capsuleBranchHint,
                knownNearWall
            );
            this.stats[STAT_CAPSULE_SAMPLES]++;
            if (measureInsideClearance) {
                allSamplesInside = allSamplesInside && contact.inside;
            }
            const gap = contact.values[CONTACT_SIGNED_GAP];
            if (gap < bestGap) {
                bestGap = gap;
                bestT = t;
                bestSampleIndex = sampleIndex;
                setContact(target, contact);
            }
        }
        this._skipBvhValidation = false;
        const winnerRefinedWithBvh =
            bestGap <= this.capsuleBvhValidationGap;
        if (winnerRefinedWithBvh) {
            bestT = bestSampleIndex / sampleCount;
            this.#refineContactWithBvh(
                ax + dx * bestT,
                ay + dy * bestT,
                az + dz * bestT,
                toolRadius,
                target
            );
        }
        if (measureInsideClearance && allSamplesInside) {
            exactInsideClearance = Infinity;
            let startClearance;
            if (winnerRefinedWithBvh && bestSampleIndex === 0) {
                startClearance = Math.abs(
                    target.values[CONTACT_SIGNED_DISTANCE]
                );
            } else if (
                reusesPreviousEndpoint &&
                this._capsuleEndpointContact.insideClearance > 0
            ) {
                // Adjacent capsules share this point exactly. Its distance to
                // the static vessel surface is therefore identical; retain
                // the previous exact certificate instead of traversing the
                // same BVH again.
                startClearance =
                    this._capsuleEndpointContact.insideClearance;
            } else {
                startClearance = this.#insideClearanceCertificate(
                    ax,
                    ay,
                    az
                );
            }
            exactInsideClearance = startClearance;

            let endClearance = startClearance;
            if (length > EPSILON) {
                endClearance = winnerRefinedWithBvh &&
                    bestSampleIndex === sampleCount
                    ? Math.abs(target.values[CONTACT_SIGNED_DISTANCE])
                    : this.#insideClearanceCertificate(
                        bx,
                        by,
                        bz
                    );
                exactInsideClearance = Math.min(
                    exactInsideClearance,
                    endClearance
                );
            }
            this._capsuleEndpointContact.insideClearance = endClearance;

            if (needsInteriorSamples) {
                for (
                    let sampleIndex = 1;
                    sampleIndex < sampleCount;
                    sampleIndex++
                ) {
                    const sampleClearance = winnerRefinedWithBvh &&
                        sampleIndex === bestSampleIndex
                        ? Math.abs(target.values[CONTACT_SIGNED_DISTANCE])
                        : this.#insideClearanceCertificate(
                            ax + dx * (sampleIndex / sampleCount),
                            ay + dy * (sampleIndex / sampleCount),
                            az + dz * (sampleIndex / sampleCount)
                        );
                    exactInsideClearance = Math.min(
                        exactInsideClearance,
                        sampleClearance
                    );
                }
            }
        } else if (measureInsideClearance) {
            // The certificate is useful only when every sampled capsule point
            // is inside. Do not leak an endpoint certificate through a query
            // that has already observed an exterior sample.
            this._capsuleEndpointContact.insideClearance = 0;
        }
        target.values[CONTACT_SEGMENT_T] = bestT;
        target.capsuleSampleCount = sampleCount;
        target.insideClearance = Number.isFinite(exactInsideClearance)
            ? Math.max(0, exactInsideClearance)
            : 0;
        return target;
    }

    #insideClearanceCertificate(x, y, z) {
        const boundsTree = this.fallbackGeometry?.boundsTree;
        if (!boundsTree) return 0;
        const previousFace = this._bvhClosest.faceIndex;
        this._bvhPoint.set(x, y, z);
        this._bvhClosest.distance = Infinity;
        let scalarHit = closestPointToPointScalarBvh(
            boundsTree,
            x,
            y,
            z,
            this._bvhClosest,
            this._scalarBvhScratch,
            INSIDE_CLEARANCE_CERTIFICATE_MM,
            previousFace
        );
        // A miss at the small certificate radius proves only that the exact
        // surface is farther away than the cap.  Keep the scalar traversal for
        // the rare unbounded retry as well; falling back to MeshBVH's generic
        // Vector3/Triangle path here creates temporary objects even though the
        // static geometry layout is already supported by the exact scalar
        // implementation.
        if (!scalarHit) {
            this._bvhClosest.distance = Infinity;
            scalarHit = closestPointToPointScalarBvh(
                boundsTree,
                x,
                y,
                z,
                this._bvhClosest,
                this._scalarBvhScratch,
                Infinity,
                previousFace
            );
        }
        let hit = scalarHit ? this._bvhClosest : null;
        if (!hit) {
            this._bvhClosest.distance = Infinity;
            hit = boundsTree.closestPointToPoint(
                this._bvhPoint,
                this._bvhClosest
            );
        }
        const distance = hit?.distance ??
            this._bvhPoint.distanceTo(this._bvhClosest.point);
        this._bvhClosest.faceIndex = previousFace;
        this._bvhClosest.distance = Infinity;
        this.stats[STAT_BVH_REFINEMENTS]++;
        this.stats[STAT_BVH_CLEARANCE_REFINEMENTS]++;
        // This value is only a conservative inside certificate used to skip
        // redundant sign classification until a material point has moved by
        // the certified amount. If the closest surface is farther away, the
        // exact excess is irrelevant. Capping the certificate lets the BVH
        // reject every node outside the small proof radius instead of finding
        // the globally closest triangle.
        return Number.isFinite(distance)
            ? Math.min(distance, INSIDE_CLEARANCE_CERTIFICATE_MM)
            : 0;
    }

    #refineContactWithBvh(x, y, z, toolRadius, target) {
        // The capsule search has already evaluated this exact sample with the
        // centerline and sparse SDF while BVH validation was suspended. Reuse
        // that result and perform only the exact triangle refinement instead
        // of repeating the complete broad phase, SDF interpolation and lumen
        // sign query at the same coordinates.
        const state = this._distanceState;
        const stateValues = state.values;
        const targetValues = target.values;
        const targetInward = target.inward.values;
        stateValues[DISTANCE_SIGNED_DISTANCE] =
            targetValues[CONTACT_SIGNED_DISTANCE];
        stateValues[DISTANCE_INWARD_X] = targetInward[0];
        stateValues[DISTANCE_INWARD_Y] = targetInward[1];
        stateValues[DISTANCE_INWARD_Z] = targetInward[2];
        stateValues[DISTANCE_BRANCH_ID] = targetValues[CONTACT_BRANCH_ID];
        state.conservative = target.conservative;
        state.source = target.source;
        state.faceIndex = target.faceIndex;
        this.#refineWithBvh(x, y, z, toolRadius, state);

        const signedDistance = stateValues[DISTANCE_SIGNED_DISTANCE];
        const signedGap = signedDistance - toolRadius;
        const penetration = Math.max(0, -signedGap);
        const nx = stateValues[DISTANCE_INWARD_X];
        const ny = stateValues[DISTANCE_INWARD_Y];
        const nz = stateValues[DISTANCE_INWARD_Z];
        target.inside = signedDistance >= 0;
        target.violation = signedGap < 0;
        target.conservative = state.conservative;
        target.source = state.source;
        target.faceIndex = state.faceIndex;
        targetValues[CONTACT_SIGNED_DISTANCE] = signedDistance;
        targetValues[CONTACT_SIGNED_GAP] = signedGap;
        targetValues[CONTACT_DISTANCE] = Math.max(0, signedDistance);
        targetValues[CONTACT_PENETRATION] = penetration;
        const normalValues = target.normal.values;
        normalValues[0] = nx;
        normalValues[1] = ny;
        normalValues[2] = nz;
        targetInward[0] = nx;
        targetInward[1] = ny;
        targetInward[2] = nz;
        const closestValues = target.closestPoint.values;
        closestValues[0] = x - nx * signedDistance;
        closestValues[1] = y - ny * signedDistance;
        closestValues[2] = z - nz * signedDistance;
        const correctionValues = target.target.values;
        correctionValues[0] = x + nx * penetration;
        correctionValues[1] = y + ny * penetration;
        correctionValues[2] = z + nz * penetration;
        return target;
    }

    sweepSphere(previous, current, radius = 0, out = null) {
        if (!out) this.stats[STAT_RESULT_ALLOCATIONS]++;
        const target = out || createContactResult();
        const ax = previous.x ?? previous[0] ?? 0;
        const ay = previous.y ?? previous[1] ?? 0;
        const az = previous.z ?? previous[2] ?? 0;
        const bx = current.x ?? current[0] ?? 0;
        const by = current.y ?? current[1] ?? 0;
        const bz = current.z ?? current[2] ?? 0;
        const dx = bx - ax;
        const dy = by - ay;
        const dz = bz - az;
        const length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const spacing = Math.max(this.voxelSize * 0.5, Math.max(0.1, radius * 0.5));
        const sampleCount = Math.max(1, Math.ceil(length / spacing));
        this.stats[STAT_SWEEP_QUERIES]++;
        this._point.x = ax;
        this._point.y = ay;
        this._point.z = az;
        let previousContact = this.#querySphereCoordinates(
            this._point.x, this._point.y, this._point.z,
            Math.max(0, radius || 0), this._sweepContact
        );
        this.stats[STAT_SWEEP_SAMPLES]++;
        if (previousContact.violation) {
            setContact(target, previousContact);
            target.values[CONTACT_TIME_OF_IMPACT] = 0;
            return target;
        }

        for (let sampleIndex = 1; sampleIndex <= sampleCount; sampleIndex++) {
            const t = sampleIndex / sampleCount;
            this._point.x = ax + dx * t;
            this._point.y = ay + dy * t;
            this._point.z = az + dz * t;
            const contact = this.#querySphereCoordinates(
                this._point.x, this._point.y, this._point.z,
                Math.max(0, radius || 0), this._sweepProbe
            );
            this.stats[STAT_SWEEP_SAMPLES]++;
            if (!contact.violation) {
                const swap = previousContact;
                previousContact = contact;
                this._sweepProbe = swap;
                continue;
            }

            let low = (sampleIndex - 1) / sampleCount;
            let high = t;
            for (let iteration = 0; iteration < 7; iteration++) {
                const mid = (low + high) * 0.5;
                this._point.x = ax + dx * mid;
                this._point.y = ay + dy * mid;
                this._point.z = az + dz * mid;
                const probe = this.#querySphereCoordinates(
                    this._point.x, this._point.y, this._point.z,
                    Math.max(0, radius || 0), this._sweepContact
                );
                this.stats[STAT_SWEEP_SAMPLES]++;
                if (probe.violation) high = mid;
                else low = mid;
            }
            this._point.x = ax + dx * high;
            this._point.y = ay + dy * high;
            this._point.z = az + dz * high;
            setContact(target, this.#querySphereCoordinates(
                this._point.x, this._point.y, this._point.z,
                Math.max(0, radius || 0), this._sweepContact
            ));
            target.values[CONTACT_TIME_OF_IMPACT] = high;
            return target;
        }

        this._point.x = bx;
        this._point.y = by;
        this._point.z = bz;
        setContact(target, this.#querySphereCoordinates(
            this._point.x, this._point.y, this._point.z,
            Math.max(0, radius || 0), this._sweepContact
        ));
        target.values[CONTACT_TIME_OF_IMPACT] = 1;
        return target;
    }

    queryBatch(positions, radii, count, output) {
        if (!output || output.signedGaps.length < count) {
            throw new RangeError('Preallocated batch contact output is too small');
        }
        this.stats[STAT_BATCH_QUERIES]++;
        const contact = this._capsuleContact;
        for (let index = 0; index < count; index++) {
            const offset = index * 3;
            this._point.x = positions[offset];
            this._point.y = positions[offset + 1];
            this._point.z = positions[offset + 2];
            this.#querySphereCoordinates(
                this._point.x, this._point.y, this._point.z,
                Math.max(0, radii[index] || 0), contact
            );
            output.signedDistances[index] = contact.values[CONTACT_SIGNED_DISTANCE];
            output.signedGaps[index] = contact.values[CONTACT_SIGNED_GAP];
            output.penetrations[index] = contact.values[CONTACT_PENETRATION];
            output.normals[offset] = contact.normal.values[0];
            output.normals[offset + 1] = contact.normal.values[1];
            output.normals[offset + 2] = contact.normal.values[2];
            output.targets[offset] = contact.target.values[0];
            output.targets[offset + 1] = contact.target.values[1];
            output.targets[offset + 2] = contact.target.values[2];
            output.branchIds[index] = contact.values[CONTACT_BRANCH_ID];
            output.violations[index] = contact.violation ? 1 : 0;
        }
        output.count = count;
        return output;
    }

    #queryDistance(
        x,
        y,
        z,
        radius,
        knownInside = false,
        knownBranchId = -1,
        knownNearWall = false
    ) {
        const state = this._distanceState;
        const stateValues = state.values;
        state.faceIndex = -1;
        const safeCoreThreshold = radius + this.voxelSize * 0.25;
        // Repeated wall-repair projections already carry a conservative proof
        // that the sample remains inside. In the exact near-wall band the
        // complete query cannot take the deep safe-core shortcut, so compute
        // the same SDF first and resolve only the two centerline scalars still
        // needed by the solver: the exact branch winner and maximum safe-core
        // distance. The helper below follows the same fine-cell candidates,
        // bounds, roundoff guard, and tie order as #queryCenterline, but omits
        // normals and nearest-distance fields that the SDF/BVH replaces.
        if (
            knownNearWall &&
            this.sdfInsideBits &&
            Number.isInteger(knownBranchId) &&
            knownBranchId >= 0 &&
            this.#querySdf(x, y, z, state, knownInside, true)
        ) {
            const signedGap =
                stateValues[DISTANCE_SIGNED_DISTANCE] - radius;
            const normalLengthSquared =
                stateValues[DISTANCE_INWARD_X] * stateValues[DISTANCE_INWARD_X] +
                stateValues[DISTANCE_INWARD_Y] * stateValues[DISTANCE_INWARD_Y] +
                stateValues[DISTANCE_INWARD_Z] * stateValues[DISTANCE_INWARD_Z];
            if (
                stateValues[DISTANCE_SIGNED_DISTANCE] >= 0 &&
                signedGap <= this.capsuleBvhValidationGap &&
                normalLengthSquared >= EPSILON * EPSILON
            ) {
                const branchId = this.#queryKnownInsideBranchId(
                    x,
                    y,
                    z,
                    knownBranchId,
                    safeCoreThreshold
                );
                if (
                    branchId >= 0 &&
                    this._knownInsideBranchSafeDistance <= safeCoreThreshold
                ) {
                    stateValues[DISTANCE_BRANCH_ID] = branchId;
                    state.conservative = false;
                    state.source = SOURCE_SDF;
                    this.stats[STAT_SDF_HITS]++;
                    this.stats[STAT_KNOWN_INSIDE_NEAR_WALL_HITS]++;
                    if (!this._skipBvhValidation) {
                        this.#refineWithBvh(x, y, z, radius, state);
                    }
                    return state;
                }
            }
        }
        const centerline = (
            knownInside && Number.isInteger(knownBranchId) &&
                knownBranchId >= 0
                ? this.#queryKnownInsideCenterline(
                    x,
                    y,
                    z,
                    knownBranchId,
                    safeCoreThreshold
                )
                : null
        ) ?? this.#queryCenterline(
            x,
            y,
            z,
            knownBranchId,
            !knownInside,
            safeCoreThreshold
        );
        const centerlineValues = centerline.values;
        state.faceIndex = -1;
        if (
            centerline.found &&
            centerlineValues[CENTERLINE_SAFE_DISTANCE] > safeCoreThreshold
        ) {
            stateValues[DISTANCE_SIGNED_DISTANCE] = centerlineValues[CENTERLINE_SAFE_DISTANCE];
            stateValues[DISTANCE_INWARD_X] = centerlineValues[CENTERLINE_SAFE_INWARD_X];
            stateValues[DISTANCE_INWARD_Y] = centerlineValues[CENTERLINE_SAFE_INWARD_Y];
            stateValues[DISTANCE_INWARD_Z] = centerlineValues[CENTERLINE_SAFE_INWARD_Z];
            stateValues[DISTANCE_BRANCH_ID] = centerlineValues[CENTERLINE_SAFE_BRANCH_ID];
            state.conservative = true;
            state.source = SOURCE_SAFE_CORE;
            this.stats[STAT_SAFE_CORE_HITS]++;
            return state;
        }

        if (this.#querySdf(x, y, z, state, knownInside)) {
            if (
                Math.sqrt(
                    stateValues[DISTANCE_INWARD_X] * stateValues[DISTANCE_INWARD_X] +
                    stateValues[DISTANCE_INWARD_Y] * stateValues[DISTANCE_INWARD_Y] +
                    stateValues[DISTANCE_INWARD_Z] * stateValues[DISTANCE_INWARD_Z]
                ) < EPSILON && centerline.found
            ) {
                stateValues[DISTANCE_INWARD_X] = centerlineValues[CENTERLINE_INWARD_X];
                stateValues[DISTANCE_INWARD_Y] = centerlineValues[CENTERLINE_INWARD_Y];
                stateValues[DISTANCE_INWARD_Z] = centerlineValues[CENTERLINE_INWARD_Z];
            }
            stateValues[DISTANCE_BRANCH_ID] = centerlineValues[CENTERLINE_BRANCH_ID];
            state.conservative = false;
            state.source = SOURCE_SDF;
            this.stats[STAT_SDF_HITS]++;
            if (!this._skipBvhValidation) this.#refineWithBvh(x, y, z, radius, state);
            return state;
        }

        if (this.fallbackCollider?.pointContact) {
            this._point.x = x;
            this._point.y = y;
            this._point.z = z;
            const contact = this.fallbackCollider.pointContact(this._point, 0, this._fallbackContact);
            if (Number.isFinite(contact?.signedDistance)) {
                const inward = contact.inward || contact.normal;
                const inwardX = inward?.x || 0;
                const inwardY = inward?.y || 0;
                const inwardZ = inward?.z || 0;
                const length = Math.sqrt(
                    inwardX * inwardX + inwardY * inwardY + inwardZ * inwardZ
                );
                stateValues[DISTANCE_SIGNED_DISTANCE] = contact.signedDistance;
                if (length > EPSILON) {
                    const sign = contact.inward ? 1 : -1;
                    stateValues[DISTANCE_INWARD_X] = inward.x / length * sign;
                    stateValues[DISTANCE_INWARD_Y] = inward.y / length * sign;
                    stateValues[DISTANCE_INWARD_Z] = inward.z / length * sign;
                } else {
                    stateValues[DISTANCE_INWARD_X] = centerlineValues[CENTERLINE_INWARD_X];
                    stateValues[DISTANCE_INWARD_Y] = centerlineValues[CENTERLINE_INWARD_Y];
                    stateValues[DISTANCE_INWARD_Z] = centerlineValues[CENTERLINE_INWARD_Z];
                }
                stateValues[DISTANCE_BRANCH_ID] = centerlineValues[CENTERLINE_BRANCH_ID];
                state.conservative = false;
                state.source = SOURCE_FALLBACK;
                this.stats[STAT_FALLBACK_HITS]++;
                return state;
            }
        }

        stateValues[DISTANCE_SIGNED_DISTANCE] = centerlineValues[CENTERLINE_SIGNED_DISTANCE];
        stateValues[DISTANCE_INWARD_X] = centerlineValues[CENTERLINE_INWARD_X];
        stateValues[DISTANCE_INWARD_Y] = centerlineValues[CENTERLINE_INWARD_Y];
        stateValues[DISTANCE_INWARD_Z] = centerlineValues[CENTERLINE_INWARD_Z];
        stateValues[DISTANCE_BRANCH_ID] = centerlineValues[CENTERLINE_BRANCH_ID];
        state.conservative = true;
        state.source = SOURCE_CENTERLINE;
        this.stats[STAT_CENTERLINE_ESTIMATE_HITS]++;
        return state;
    }

    #queryKnownInsideBranchId(
        x,
        y,
        z,
        hintBranchId,
        safeDistanceFloor
    ) {
        this._knownInsideBranchSafeDistance = -Infinity;
        if (!this.centerlineSafeWithinRadius) return -1;
        const cellX = Math.floor(
            (x - this.broadPhaseOrigin[0]) / this.broadPhaseCellSize
        );
        const cellY = Math.floor(
            (y - this.broadPhaseOrigin[1]) / this.broadPhaseCellSize
        );
        const cellZ = Math.floor(
            (z - this.broadPhaseOrigin[2]) / this.broadPhaseCellSize
        );
        if (
            cellX < 0 || cellY < 0 || cellZ < 0 ||
            cellX >= this.broadPhaseDimensions[0] ||
            cellY >= this.broadPhaseDimensions[1] ||
            cellZ >= this.broadPhaseDimensions[2]
        ) return -1;

        const hintBounds = hintBranchId * 6;
        if (
            hintBranchId >= this.centerlineCellBounds.length / 6 ||
            cellX < this.centerlineCellBounds[hintBounds] ||
            cellY < this.centerlineCellBounds[hintBounds + 1] ||
            cellZ < this.centerlineCellBounds[hintBounds + 2] ||
            cellX > this.centerlineCellBounds[hintBounds + 3] ||
            cellY > this.centerlineCellBounds[hintBounds + 4] ||
            cellZ > this.centerlineCellBounds[hintBounds + 5]
        ) return -1;

        const scratch = this._knownInsideBranchScratch ??=
            new Float64Array(3);
        scratch[0] = -1;
        scratch[1] = -Infinity;
        scratch[2] = safeDistanceFloor;
        considerKnownInsideBranchSegment(
            this,
            hintBranchId,
            x,
            y,
            z,
            scratch
        );
        if (scratch[1] <= -this.centerlinePadding) return -1;

        const fineX = Math.floor(
            (x - this.broadPhaseOrigin[0]) / this.centerlineFineCellSize
        );
        const fineY = Math.floor(
            (y - this.broadPhaseOrigin[1]) / this.centerlineFineCellSize
        );
        const fineZ = Math.floor(
            (z - this.broadPhaseOrigin[2]) / this.centerlineFineCellSize
        );
        let localX = fineX - cellX * FINE_BROAD_PHASE_SCALE;
        let localY = fineY - cellY * FINE_BROAD_PHASE_SCALE;
        let localZ = fineZ - cellZ * FINE_BROAD_PHASE_SCALE;
        let broadFineCell = cellX + this.broadPhaseDimensions[0] * (
            cellY + this.broadPhaseDimensions[1] * cellZ
        );
        if (
            localX < 0 || localY < 0 || localZ < 0 ||
            localX >= FINE_BROAD_PHASE_SCALE ||
            localY >= FINE_BROAD_PHASE_SCALE ||
            localZ >= FINE_BROAD_PHASE_SCALE
        ) {
            const broadFineX = Math.floor(
                fineX / FINE_BROAD_PHASE_SCALE
            );
            const broadFineY = Math.floor(
                fineY / FINE_BROAD_PHASE_SCALE
            );
            const broadFineZ = Math.floor(
                fineZ / FINE_BROAD_PHASE_SCALE
            );
            broadFineCell = broadFineX + this.broadPhaseDimensions[0] * (
                broadFineY + this.broadPhaseDimensions[1] * broadFineZ
            );
            localX = fineX - broadFineX * FINE_BROAD_PHASE_SCALE;
            localY = fineY - broadFineY * FINE_BROAD_PHASE_SCALE;
            localZ = fineZ - broadFineZ * FINE_BROAD_PHASE_SCALE;
        }
        const block = this.centerlineFineBlockByBroadCell[broadFineCell];
        if (block < 0) return -1;
        const localCell = localX + FINE_BROAD_PHASE_SCALE * (
            localY + FINE_BROAD_PHASE_SCALE * localZ
        );
        const fineSlot = this.centerlineFineSlotByBlock[
            block * FINE_CELLS_PER_BROAD_CELL + localCell
        ] - 1;
        if (fineSlot < 0) return -1;

        const bounds = this.centerlineBounds;
        const boundsStride = this.centerlineBoundsStride;
        const runs = this.centerlineFineRuns;
        const start = this.centerlineFineRunOffsets[fineSlot];
        const end = this.centerlineFineRunOffsets[fineSlot + 1];
        for (let runEntry = start; runEntry < end; runEntry++) {
            const runStart = runs[runEntry * 2];
            const runEnd = runStart + runs[runEntry * 2 + 1];
            for (let segmentId = runStart; segmentId < runEnd; segmentId++) {
                if (segmentId === hintBranchId) continue;
                const offset = segmentId * boundsStride;
                const boundX = x < bounds[offset]
                    ? bounds[offset] - x
                    : x > bounds[offset + 3]
                        ? x - bounds[offset + 3]
                        : 0;
                const boundY = y < bounds[offset + 1]
                    ? bounds[offset + 1] - y
                    : y > bounds[offset + 4]
                        ? y - bounds[offset + 4]
                        : 0;
                const boundZ = z < bounds[offset + 2]
                    ? bounds[offset + 2] - z
                    : z > bounds[offset + 5]
                        ? z - bounds[offset + 5]
                        : 0;
                const lowerDistanceSquared =
                    boundX * boundX +
                    boundY * boundY +
                    boundZ * boundZ;
                const signedThreshold =
                    bounds[offset + 6] - scratch[1];
                const safeThreshold =
                    bounds[offset + 7] - scratch[2];
                const cannotImproveSigned = signedThreshold < 0 ||
                    lowerDistanceSquared > signedThreshold * signedThreshold;
                const cannotImproveSafe = safeThreshold < 0 ||
                    lowerDistanceSquared > safeThreshold * safeThreshold;
                if (cannotImproveSigned && cannotImproveSafe) continue;
                considerKnownInsideBranchSegment(
                    this,
                    segmentId,
                    x,
                    y,
                    z,
                    scratch
                );
            }
        }
        this._knownInsideBranchSafeDistance = scratch[2];
        return scratch[0];
    }

    // Active wall contacts carry both an inside certificate and their exact
    // preceding centerline winner. Specialize that dominant query shape so
    // V8 sees one monomorphic fine-grid loop instead of the general nearest /
    // coarse / fallback branches. Returning null delegates every unproven
    // case to #queryCenterline; the accepted path visits the same candidates
    // in the same order and performs the same comparisons.
    #queryKnownInsideCenterline(
        x,
        y,
        z,
        hintBranchId,
        safeDistanceFloor
    ) {
        if (!this.centerlineSafeWithinRadius) return null;
        const cellX = Math.floor(
            (x - this.broadPhaseOrigin[0]) / this.broadPhaseCellSize
        );
        const cellY = Math.floor(
            (y - this.broadPhaseOrigin[1]) / this.broadPhaseCellSize
        );
        const cellZ = Math.floor(
            (z - this.broadPhaseOrigin[2]) / this.broadPhaseCellSize
        );
        if (
            cellX < 0 || cellY < 0 || cellZ < 0 ||
            cellX >= this.broadPhaseDimensions[0] ||
            cellY >= this.broadPhaseDimensions[1] ||
            cellZ >= this.broadPhaseDimensions[2]
        ) return null;
        const hintBounds = hintBranchId * 6;
        if (
            hintBranchId >= this.centerlineCellBounds.length / 6 ||
            cellX < this.centerlineCellBounds[hintBounds] ||
            cellY < this.centerlineCellBounds[hintBounds + 1] ||
            cellZ < this.centerlineCellBounds[hintBounds + 2] ||
            cellX > this.centerlineCellBounds[hintBounds + 3] ||
            cellY > this.centerlineCellBounds[hintBounds + 4] ||
            cellZ > this.centerlineCellBounds[hintBounds + 5]
        ) return null;

        const state = this._centerlineState;
        const stateValues = state.values;
        state.found = false;
        stateValues[CENTERLINE_BRANCH_ID] = -1;
        stateValues[CENTERLINE_T] = 0;
        stateValues[CENTERLINE_SIGNED_DISTANCE] = -Infinity;
        stateValues[CENTERLINE_SAFE_DISTANCE] = safeDistanceFloor;
        stateValues[CENTERLINE_SAFE_BRANCH_ID] = -1;
        stateValues[CENTERLINE_SAFE_INWARD_X] = 1;
        stateValues[CENTERLINE_SAFE_INWARD_Y] = 0;
        stateValues[CENTERLINE_SAFE_INWARD_Z] = 0;
        stateValues[CENTERLINE_NEAREST_DISTANCE] = Infinity;
        stateValues[CENTERLINE_INWARD_X] = 1;
        stateValues[CENTERLINE_INWARD_Y] = 0;
        stateValues[CENTERLINE_INWARD_Z] = 0;
        this.#considerCenterlineSegment(
            hintBranchId,
            state,
            x,
            y,
            z,
            true,
            false
        );
        if (
            stateValues[CENTERLINE_SIGNED_DISTANCE] <=
                -this.centerlinePadding
        ) return null;

        const fineX = Math.floor(
            (x - this.broadPhaseOrigin[0]) / this.centerlineFineCellSize
        );
        const fineY = Math.floor(
            (y - this.broadPhaseOrigin[1]) / this.centerlineFineCellSize
        );
        const fineZ = Math.floor(
            (z - this.broadPhaseOrigin[2]) / this.centerlineFineCellSize
        );
        let localX = fineX - cellX * FINE_BROAD_PHASE_SCALE;
        let localY = fineY - cellY * FINE_BROAD_PHASE_SCALE;
        let localZ = fineZ - cellZ * FINE_BROAD_PHASE_SCALE;
        let broadFineCell = cellX + this.broadPhaseDimensions[0] * (
            cellY + this.broadPhaseDimensions[1] * cellZ
        );
        if (
            localX < 0 || localY < 0 || localZ < 0 ||
            localX >= FINE_BROAD_PHASE_SCALE ||
            localY >= FINE_BROAD_PHASE_SCALE ||
            localZ >= FINE_BROAD_PHASE_SCALE
        ) {
            const broadFineX = Math.floor(
                fineX / FINE_BROAD_PHASE_SCALE
            );
            const broadFineY = Math.floor(
                fineY / FINE_BROAD_PHASE_SCALE
            );
            const broadFineZ = Math.floor(
                fineZ / FINE_BROAD_PHASE_SCALE
            );
            broadFineCell = broadFineX + this.broadPhaseDimensions[0] * (
                broadFineY + this.broadPhaseDimensions[1] * broadFineZ
            );
            localX = fineX - broadFineX * FINE_BROAD_PHASE_SCALE;
            localY = fineY - broadFineY * FINE_BROAD_PHASE_SCALE;
            localZ = fineZ - broadFineZ * FINE_BROAD_PHASE_SCALE;
        }
        const block = this.centerlineFineBlockByBroadCell[broadFineCell];
        if (block < 0) return null;
        const localCell = localX + FINE_BROAD_PHASE_SCALE * (
            localY + FINE_BROAD_PHASE_SCALE * localZ
        );
        const fineSlot = this.centerlineFineSlotByBlock[
            block * FINE_CELLS_PER_BROAD_CELL + localCell
        ] - 1;
        if (fineSlot < 0) return null;

        const centerlineBounds = this.centerlineBounds;
        const centerlineBoundsStride = this.centerlineBoundsStride;
        const candidateRuns = this.centerlineFineRuns;
        const start = this.centerlineFineRunOffsets[fineSlot];
        const end = this.centerlineFineRunOffsets[fineSlot + 1];
        for (let runEntry = start; runEntry < end; runEntry++) {
            const runStart = candidateRuns[runEntry * 2];
            const runEnd = runStart + candidateRuns[runEntry * 2 + 1];
            for (let segmentId = runStart; segmentId < runEnd; segmentId++) {
                if (segmentId === hintBranchId) continue;
                if (state.found) {
                    const boundsOffset = segmentId * centerlineBoundsStride;
                    const minX = centerlineBounds[boundsOffset];
                    const minY = centerlineBounds[boundsOffset + 1];
                    const minZ = centerlineBounds[boundsOffset + 2];
                    const maxX = centerlineBounds[boundsOffset + 3];
                    const maxY = centerlineBounds[boundsOffset + 4];
                    const maxZ = centerlineBounds[boundsOffset + 5];
                    const boundX = x < minX
                        ? minX - x
                        : x > maxX ? x - maxX : 0;
                    const boundY = y < minY
                        ? minY - y
                        : y > maxY ? y - maxY : 0;
                    const boundZ = z < minZ
                        ? minZ - z
                        : z > maxZ ? z - maxZ : 0;
                    const lowerDistanceSquared =
                        boundX * boundX + boundY * boundY + boundZ * boundZ;
                    const signedDistanceThreshold =
                        centerlineBounds[boundsOffset + 6] -
                        stateValues[CENTERLINE_SIGNED_DISTANCE];
                    const safeDistanceThreshold =
                        centerlineBounds[boundsOffset + 7] -
                        stateValues[CENTERLINE_SAFE_DISTANCE];
                    const cannotImproveSigned =
                        signedDistanceThreshold < 0 ||
                        lowerDistanceSquared >
                            signedDistanceThreshold * signedDistanceThreshold;
                    const cannotImproveSafe =
                        safeDistanceThreshold < 0 ||
                        lowerDistanceSquared >
                            safeDistanceThreshold * safeDistanceThreshold;
                    if (cannotImproveSigned && cannotImproveSafe) continue;
                }
                this.#considerCenterlineSegment(
                    segmentId,
                    state,
                    x,
                    y,
                    z,
                    true,
                    false
                );
            }
        }
        return state;
    }

    #queryCenterline(
        x,
        y,
        z,
        hintBranchId = -1,
        trackNearestDistance = true,
        safeDistanceFloor = -Infinity
    ) {
        const state = this._centerlineState;
        const stateValues = state.values;
        const centerlineBounds = this.centerlineBounds;
        const centerlineBoundsStride = this.centerlineBoundsStride;
        state.found = false;
        stateValues[CENTERLINE_BRANCH_ID] = -1;
        stateValues[CENTERLINE_T] = 0;
        stateValues[CENTERLINE_SIGNED_DISTANCE] = -Infinity;
        // Values at or below the safe-core acceptance threshold are never
        // observed by the caller. Starting from that floor lets exact
        // branch-and-bound discard candidates that could improve the unused
        // sub-threshold maximum while preserving both the pass/fail decision
        // and the exact winning safe core whenever it is accepted.
        stateValues[CENTERLINE_SAFE_DISTANCE] = safeDistanceFloor;
        stateValues[CENTERLINE_SAFE_BRANCH_ID] = -1;
        stateValues[CENTERLINE_SAFE_INWARD_X] = 1;
        stateValues[CENTERLINE_SAFE_INWARD_Y] = 0;
        stateValues[CENTERLINE_SAFE_INWARD_Z] = 0;
        stateValues[CENTERLINE_NEAREST_DISTANCE] = Infinity;
        stateValues[CENTERLINE_INWARD_X] = 1;
        stateValues[CENTERLINE_INWARD_Y] = 0;
        stateValues[CENTERLINE_INWARD_Z] = 0;

        const cellX = Math.floor((x - this.broadPhaseOrigin[0]) / this.broadPhaseCellSize);
        const cellY = Math.floor((y - this.broadPhaseOrigin[1]) / this.broadPhaseCellSize);
        const cellZ = Math.floor((z - this.broadPhaseOrigin[2]) / this.broadPhaseCellSize);
        if (
            cellX >= 0 && cellY >= 0 && cellZ >= 0 &&
            cellX < this.broadPhaseDimensions[0] &&
            cellY < this.broadPhaseDimensions[1] &&
            cellZ < this.broadPhaseDimensions[2]
        ) {
            const cellIndex = cellX + this.broadPhaseDimensions[0] * (
                cellY + this.broadPhaseDimensions[1] * cellZ
            );
            let start = this.broadPhaseOffsets[cellIndex];
            let end = this.broadPhaseOffsets[cellIndex + 1];
            let candidateIds = this.broadPhaseOrderedIds;
            let candidateRuns = null;
            let hintedCandidate = -1;
            if (
                Number.isInteger(hintBranchId) &&
                hintBranchId >= 0 &&
                hintBranchId < this.centerlineCellBounds.length / 6
            ) {
                const bounds = hintBranchId * 6;
                if (
                    cellX >= this.centerlineCellBounds[bounds] &&
                    cellY >= this.centerlineCellBounds[bounds + 1] &&
                    cellZ >= this.centerlineCellBounds[bounds + 2] &&
                    cellX <= this.centerlineCellBounds[bounds + 3] &&
                    cellY <= this.centerlineCellBounds[bounds + 4] &&
                    cellZ <= this.centerlineCellBounds[bounds + 5]
                ) {
                    hintedCandidate = hintBranchId;
                    this.#considerCenterlineSegment(
                        hintedCandidate,
                        state,
                        x,
                        y,
                        z,
                        true,
                        trackNearestDistance
                    );
                }
            }
            if (
                !trackNearestDistance &&
                this.centerlineSafeWithinRadius &&
                hintedCandidate >= 0 &&
                stateValues[CENTERLINE_SIGNED_DISTANCE] >
                    -this.centerlinePadding
            ) {
                const fineX = Math.floor((x - this.broadPhaseOrigin[0]) /
                    this.centerlineFineCellSize);
                const fineY = Math.floor((y - this.broadPhaseOrigin[1]) /
                    this.centerlineFineCellSize);
                const fineZ = Math.floor((z - this.broadPhaseOrigin[2]) /
                    this.centerlineFineCellSize);
                let localX = fineX - cellX * FINE_BROAD_PHASE_SCALE;
                let localY = fineY - cellY * FINE_BROAD_PHASE_SCALE;
                let localZ = fineZ - cellZ * FINE_BROAD_PHASE_SCALE;
                let broadFineCell = cellIndex;
                // The coarse and fine grids share an exact integer scale.
                // Only a floating-point coordinate lying exactly on a cell
                // boundary can make their independent floor operations choose
                // adjacent cells; retain the global mapping for that rare case.
                if (
                    localX < 0 || localY < 0 || localZ < 0 ||
                    localX >= FINE_BROAD_PHASE_SCALE ||
                    localY >= FINE_BROAD_PHASE_SCALE ||
                    localZ >= FINE_BROAD_PHASE_SCALE
                ) {
                    const broadFineX = Math.floor(
                        fineX / FINE_BROAD_PHASE_SCALE
                    );
                    const broadFineY = Math.floor(
                        fineY / FINE_BROAD_PHASE_SCALE
                    );
                    const broadFineZ = Math.floor(
                        fineZ / FINE_BROAD_PHASE_SCALE
                    );
                    broadFineCell = broadFineX +
                        this.broadPhaseDimensions[0] * (
                            broadFineY +
                            this.broadPhaseDimensions[1] * broadFineZ
                        );
                    localX = fineX -
                        broadFineX * FINE_BROAD_PHASE_SCALE;
                    localY = fineY -
                        broadFineY * FINE_BROAD_PHASE_SCALE;
                    localZ = fineZ -
                        broadFineZ * FINE_BROAD_PHASE_SCALE;
                }
                const block = this.centerlineFineBlockByBroadCell[
                    broadFineCell
                ];
                const localCell = localX + FINE_BROAD_PHASE_SCALE * (
                    localY + FINE_BROAD_PHASE_SCALE * localZ
                );
                const slotValue = block >= 0
                    ? this.centerlineFineSlotByBlock[
                        block * FINE_CELLS_PER_BROAD_CELL + localCell
                    ]
                    : 0;
                const fineSlot = slotValue - 1;
                if (fineSlot >= 0) {
                    start = this.centerlineFineRunOffsets[fineSlot];
                    end = this.centerlineFineRunOffsets[fineSlot + 1];
                    candidateIds = null;
                    candidateRuns = this.centerlineFineRuns;
                }
            }
            if (candidateRuns) {
                for (let runEntry = start; runEntry < end; runEntry++) {
                    const runStart = candidateRuns[runEntry * 2];
                    const runEnd = runStart + candidateRuns[runEntry * 2 + 1];
                    for (let segmentId = runStart; segmentId < runEnd; segmentId++) {
                        if (segmentId === hintedCandidate) continue;
                        if (state.found) {
                            const boundsOffset = segmentId * centerlineBoundsStride;
                            const minX = centerlineBounds[boundsOffset];
                            const minY = centerlineBounds[boundsOffset + 1];
                            const minZ = centerlineBounds[boundsOffset + 2];
                            const maxX = centerlineBounds[boundsOffset + 3];
                            const maxY = centerlineBounds[boundsOffset + 4];
                            const maxZ = centerlineBounds[boundsOffset + 5];
                            const boundX = x < minX ? minX - x : x > maxX ? x - maxX : 0;
                            const boundY = y < minY ? minY - y : y > maxY ? y - maxY : 0;
                            const boundZ = z < minZ ? minZ - z : z > maxZ ? z - maxZ : 0;
                            const lowerDistanceSquared =
                                boundX * boundX + boundY * boundY + boundZ * boundZ;
                            const signedDistanceThreshold =
                                centerlineBounds[boundsOffset + 6] -
                                stateValues[CENTERLINE_SIGNED_DISTANCE];
                            const safeDistanceThreshold =
                                centerlineBounds[boundsOffset + 7] -
                                stateValues[CENTERLINE_SAFE_DISTANCE];
                            const cannotImproveSigned =
                                signedDistanceThreshold < 0 ||
                                lowerDistanceSquared >
                                    signedDistanceThreshold * signedDistanceThreshold;
                            const cannotImproveSafe =
                                safeDistanceThreshold < 0 ||
                                lowerDistanceSquared >
                                    safeDistanceThreshold * safeDistanceThreshold;
                            if (
                                cannotImproveSigned &&
                                cannotImproveSafe
                            ) continue;
                        }
                        this.#considerCenterlineSegment(
                            segmentId,
                            state,
                            x,
                            y,
                            z,
                            true,
                            trackNearestDistance
                        );
                    }
                }
            } else {
                for (let entry = start; entry < end; entry++) {
                    const segmentId = candidateIds[entry];
                    if (segmentId === hintedCandidate) continue;
                    if (state.found) {
                        const boundsOffset = segmentId * centerlineBoundsStride;
                        const minX = centerlineBounds[boundsOffset];
                        const minY = centerlineBounds[boundsOffset + 1];
                        const minZ = centerlineBounds[boundsOffset + 2];
                        const maxX = centerlineBounds[boundsOffset + 3];
                        const maxY = centerlineBounds[boundsOffset + 4];
                        const maxZ = centerlineBounds[boundsOffset + 5];
                        const boundX = x < minX ? minX - x : x > maxX ? x - maxX : 0;
                        const boundY = y < minY ? minY - y : y > maxY ? y - maxY : 0;
                        const boundZ = z < minZ ? minZ - z : z > maxZ ? z - maxZ : 0;
                        const lowerDistanceSquared =
                            boundX * boundX + boundY * boundY + boundZ * boundZ;
                        const nearestDistance =
                            stateValues[CENTERLINE_NEAREST_DISTANCE];
                        const signedDistanceThreshold =
                            centerlineBounds[boundsOffset + 6] -
                            stateValues[CENTERLINE_SIGNED_DISTANCE];
                        const safeDistanceThreshold =
                            centerlineBounds[boundsOffset + 7] -
                            stateValues[CENTERLINE_SAFE_DISTANCE];
                        const cannotImproveNearest =
                            !trackNearestDistance ||
                            lowerDistanceSquared >=
                                nearestDistance * nearestDistance;
                        const cannotImproveSigned =
                            signedDistanceThreshold < 0 ||
                            lowerDistanceSquared >
                                signedDistanceThreshold * signedDistanceThreshold;
                        const cannotImproveSafe =
                            safeDistanceThreshold < 0 ||
                            lowerDistanceSquared >
                                safeDistanceThreshold * safeDistanceThreshold;
                        if (
                            cannotImproveNearest &&
                            cannotImproveSigned &&
                            cannotImproveSafe
                        ) continue;
                    }
                    this.#considerCenterlineSegment(
                        segmentId,
                        state,
                        x,
                        y,
                        z,
                        true,
                        trackNearestDistance
                    );
                }
            }
        }

        if (!state.found) {
            const segmentCount = this.centerline.length / this.centerlineStride;
            for (let segmentId = 0; segmentId < segmentCount; segmentId++) {
                this.#considerCenterlineSegment(
                    segmentId,
                    state,
                    x,
                    y,
                    z,
                    false,
                    trackNearestDistance
                );
            }
        }
        return state;
    }

    #considerCenterlineSegment(
        segmentId,
        state,
        x,
        y,
        z,
        boundsAlreadyChecked = false,
        trackNearestDistance = true
    ) {
        const stateValues = state.values;
        const offset = segmentId * this.centerlinePreparedStride;
        if (state.found && !boundsAlreadyChecked) {
            const boundsOffset = segmentId * this.centerlineBoundsStride;
            const minX = this.centerlineBounds[boundsOffset];
            const minY = this.centerlineBounds[boundsOffset + 1];
            const minZ = this.centerlineBounds[boundsOffset + 2];
            const maxX = this.centerlineBounds[boundsOffset + 3];
            const maxY = this.centerlineBounds[boundsOffset + 4];
            const maxZ = this.centerlineBounds[boundsOffset + 5];
            const boundX = x < minX ? minX - x : x > maxX ? x - maxX : 0;
            const boundY = y < minY ? minY - y : y > maxY ? y - maxY : 0;
            const boundZ = z < minZ ? minZ - z : z > maxZ ? z - maxZ : 0;
            const lowerDistance = Math.sqrt(
                boundX * boundX + boundY * boundY + boundZ * boundZ
            );
            const canImproveNearest = trackNearestDistance &&
                lowerDistance < stateValues[CENTERLINE_NEAREST_DISTANCE];
            const canImproveSigned =
                this.centerlineBounds[boundsOffset + 6] - lowerDistance >=
                    stateValues[CENTERLINE_SIGNED_DISTANCE];
            const canImproveSafe =
                this.centerlineBounds[boundsOffset + 7] - lowerDistance >=
                    stateValues[CENTERLINE_SAFE_DISTANCE];
            if (!canImproveNearest && !canImproveSigned && !canImproveSafe) {
                return;
            }
        }
        const ax = this.centerlinePrepared[offset];
        const ay = this.centerlinePrepared[offset + 1];
        const az = this.centerlinePrepared[offset + 2];
        const dx = this.centerlinePrepared[offset + 3];
        const dy = this.centerlinePrepared[offset + 4];
        const dz = this.centerlinePrepared[offset + 5];
        const lengthSq = this.centerlinePrepared[offset + 6];
        const t = clamp(((x - ax) * dx + (y - ay) * dy + (z - az) * dz) / Math.max(EPSILON, lengthSq), 0, 1);
        const cx = ax + dx * t;
        const cy = ay + dy * t;
        const cz = az + dz * t;
        const rx = cx - x;
        const ry = cy - y;
        const rz = cz - z;
        const radialDistanceSquared = rx * rx + ry * ry + rz * rz;
        const radius = this.centerlinePrepared[offset + 7] * (1 - t) +
            this.centerlinePrepared[offset + 8] * t;
        const safeRadius = this.centerlinePrepared[offset + 9];
        if (state.found) {
            const signedThreshold = radius -
                stateValues[CENTERLINE_SIGNED_DISTANCE];
            const safeThreshold = safeRadius -
                stateValues[CENTERLINE_SAFE_DISTANCE];
            const nearestDistance =
                stateValues[CENTERLINE_NEAREST_DISTANCE];
            const nearestDistanceSquared = trackNearestDistance
                ? nearestDistance * nearestDistance
                : 0;
            // The AABB test above is deliberately loose. Once the exact
            // closest point on this centerline segment is known, squared
            // comparisons can prove that it cannot improve any observable
            // field without paying for a square root. Keep a wide roundoff
            // guard; every candidate near a winning/tie boundary follows the
            // original sqrt and comparison path below.
            const roundoffGuard = (
                radialDistanceSquared +
                signedThreshold * signedThreshold +
                safeThreshold * safeThreshold +
                nearestDistanceSquared + 1
            ) * 1e-13;
            const cannotImproveSigned = signedThreshold < 0 ||
                radialDistanceSquared >
                    signedThreshold * signedThreshold + roundoffGuard;
            const cannotImproveSafe = safeThreshold < 0 ||
                radialDistanceSquared >
                    safeThreshold * safeThreshold + roundoffGuard;
            const cannotImproveNearest = !trackNearestDistance ||
                radialDistanceSquared >
                    nearestDistanceSquared + roundoffGuard;
            if (
                cannotImproveSigned &&
                cannotImproveSafe &&
                cannotImproveNearest
            ) return;
        }
        const radialDistance = Math.sqrt(radialDistanceSquared);
        const signedDistance = radius - radialDistance;
        const safeDistance = safeRadius - radialDistance;
        if (trackNearestDistance) {
            stateValues[CENTERLINE_NEAREST_DISTANCE] = Math.min(
                stateValues[CENTERLINE_NEAREST_DISTANCE],
                radialDistance
            );
        }

        // Most broad-phase candidates cannot replace either the best
        // conservative core or the best lumen estimate. Their inward normal
        // was previously normalized anyway and then discarded. Decide which
        // fields can change first; this preserves candidate order, strict tie
        // handling and every stored scalar while removing three divisions
        // (or the axial fallback construction) from rejected candidates.
        const improvesSafe = safeDistance >
            stateValues[CENTERLINE_SAFE_DISTANCE] || (
            safeDistance === stateValues[CENTERLINE_SAFE_DISTANCE] &&
            segmentId < stateValues[CENTERLINE_SAFE_BRANCH_ID]
        );
        const improvesSigned = !state.found || signedDistance >
            stateValues[CENTERLINE_SIGNED_DISTANCE] || (
            signedDistance === stateValues[CENTERLINE_SIGNED_DISTANCE] &&
            segmentId < stateValues[CENTERLINE_BRANCH_ID]
        );
        if (!improvesSafe && !improvesSigned) return;

        let inwardX;
        let inwardY;
        let inwardZ;
        if (radialDistance > EPSILON) {
            inwardX = rx / radialDistance;
            inwardY = ry / radialDistance;
            inwardZ = rz / radialDistance;
        } else {
            const tangentLength = Math.sqrt(lengthSq);
            const tx = tangentLength > EPSILON ? dx / tangentLength : 0;
            const ty = tangentLength > EPSILON ? dy / tangentLength : 1;
            const tz = tangentLength > EPSILON ? dz / tangentLength : 0;
            const helperX = Math.abs(ty) < 0.85 ? 0 : 1;
            const helperY = Math.abs(ty) < 0.85 ? 1 : 0;
            const crossX = -tz * helperY;
            const crossY = tz * helperX;
            const crossZ = tx * helperY - ty * helperX;
            const crossLength = Math.sqrt(
                crossX * crossX + crossY * crossY + crossZ * crossZ
            ) || 1;
            inwardX = crossX / crossLength;
            inwardY = crossY / crossLength;
            inwardZ = crossZ / crossLength;
        }

        if (improvesSafe) {
            stateValues[CENTERLINE_SAFE_DISTANCE] = safeDistance;
            stateValues[CENTERLINE_SAFE_BRANCH_ID] = segmentId;
            stateValues[CENTERLINE_SAFE_INWARD_X] = inwardX;
            stateValues[CENTERLINE_SAFE_INWARD_Y] = inwardY;
            stateValues[CENTERLINE_SAFE_INWARD_Z] = inwardZ;
        }
        if (!improvesSigned) return;

        state.found = true;
        stateValues[CENTERLINE_BRANCH_ID] = segmentId;
        stateValues[CENTERLINE_T] = t;
        stateValues[CENTERLINE_SIGNED_DISTANCE] = signedDistance;
        stateValues[CENTERLINE_INWARD_X] = inwardX;
        stateValues[CENTERLINE_INWARD_Y] = inwardY;
        stateValues[CENTERLINE_INWARD_Z] = inwardZ;
    }

    #querySdf(
        x,
        y,
        z,
        state,
        knownInside = false,
        deferExteriorCorrection = false
    ) {
        const stateValues = state.values;
        const centerlineValues = this._centerlineState.values;
        const corners = this._sdfCornerScratch;
        const gx = (x - this.sdfOrigin[0]) / this.voxelSize;
        const gy = (y - this.sdfOrigin[1]) / this.voxelSize;
        const gz = (z - this.sdfOrigin[2]) / this.voxelSize;
        const ix = Math.floor(gx);
        const iy = Math.floor(gy);
        const iz = Math.floor(gz);
        const fx = gx - ix;
        const fy = gy - iy;
        const fz = gz - iz;
        const brickSize = this.brickSize;
        const brickX = Math.floor(ix / brickSize);
        const brickY = Math.floor(iy / brickSize);
        const brickZ = Math.floor(iz / brickSize);
        const localX = ix - brickX * brickSize;
        const localY = iy - brickY * brickSize;
        const localZ = iz - brickZ * brickSize;
        const sameBrick =
            localX >= 0 && localY >= 0 && localZ >= 0 &&
            localX + 1 < brickSize && localY + 1 < brickSize && localZ + 1 < brickSize &&
            brickX >= 0 && brickY >= 0 && brickZ >= 0 &&
            brickX < this.sdfDimensions[0] &&
            brickY < this.sdfDimensions[1] &&
            brickZ < this.sdfDimensions[2];
        let sameBrickBase = -1;
        if (sameBrick) {
            const key = brickX + this.sdfDimensions[0] * (
                brickY + this.sdfDimensions[1] * brickZ
            );
            const brickIndex = this.sdfBrickLookup[key];
            if (brickIndex !== this.sdfMissingBrick) {
                const row = brickSize;
                const plane = brickSize * brickSize;
                sameBrickBase = brickIndex * this.valuesPerBrick + localX + row * localY + plane * localZ;
                const values = this.sdfDistances;
                const quantization = this.sdfQuantization;
                corners[0] = values[sameBrickBase] * quantization;
                corners[1] = values[sameBrickBase + 1] * quantization;
                corners[2] = values[sameBrickBase + row] * quantization;
                corners[3] = values[sameBrickBase + row + 1] * quantization;
                corners[4] = values[sameBrickBase + plane] * quantization;
                corners[5] = values[sameBrickBase + plane + 1] * quantization;
                corners[6] = values[sameBrickBase + plane + row] * quantization;
                corners[7] = values[sameBrickBase + plane + row + 1] * quantization;
            }
        }
        if (sameBrickBase < 0) {
            this.#writeSdfValue(corners, 0, ix, iy, iz);
            this.#writeSdfValue(corners, 1, ix + 1, iy, iz);
            this.#writeSdfValue(corners, 2, ix, iy + 1, iz);
            this.#writeSdfValue(corners, 3, ix + 1, iy + 1, iz);
            this.#writeSdfValue(corners, 4, ix, iy, iz + 1);
            this.#writeSdfValue(corners, 5, ix + 1, iy, iz + 1);
            this.#writeSdfValue(corners, 6, ix, iy + 1, iz + 1);
            this.#writeSdfValue(corners, 7, ix + 1, iy + 1, iz + 1);
        }
        if (
            !Number.isFinite(corners[0]) || !Number.isFinite(corners[1]) ||
            !Number.isFinite(corners[2]) || !Number.isFinite(corners[3]) ||
            !Number.isFinite(corners[4]) || !Number.isFinite(corners[5]) ||
            !Number.isFinite(corners[6]) || !Number.isFinite(corners[7])
        ) return false;

        const x00 = corners[0] + (corners[1] - corners[0]) * fx;
        const x10 = corners[2] + (corners[3] - corners[2]) * fx;
        const x01 = corners[4] + (corners[5] - corners[4]) * fx;
        const x11 = corners[6] + (corners[7] - corners[6]) * fx;
        const y0 = x00 + (x10 - x00) * fy;
        const y1 = x01 + (x11 - x01) * fy;
        const unsignedDistance = y0 + (y1 - y0) * fz;
        let normalMultiplier;
        if (this.sdfInsideBits) {
            if (knownInside) {
                // Active wall contacts carry a conservative, exact
                // inside-clearance certificate from the preceding BVH query.
                // Their sign is therefore already known. Reading and
                // interpolating eight occupancy bits cannot change the result.
                normalMultiplier = 1;
                stateValues[DISTANCE_SIGNED_DISTANCE] = unsignedDistance;
            } else {
            let s000;
            let s100;
            let s010;
            let s110;
            let s001;
            let s101;
            let s011;
            let s111;
            if (sameBrickBase >= 0) {
                const row = brickSize;
                const plane = brickSize * brickSize;
                const bits = this.sdfInsideBits;
                const i100 = sameBrickBase + 1;
                const i010 = sameBrickBase + row;
                const i110 = i010 + 1;
                const i001 = sameBrickBase + plane;
                const i101 = i001 + 1;
                const i011 = i001 + row;
                const i111 = i011 + 1;
                s000 = (bits[sameBrickBase >> 3] & (1 << (sameBrickBase & 7))) !== 0 ? 1 : 0;
                s100 = (bits[i100 >> 3] & (1 << (i100 & 7))) !== 0 ? 1 : 0;
                s010 = (bits[i010 >> 3] & (1 << (i010 & 7))) !== 0 ? 1 : 0;
                s110 = (bits[i110 >> 3] & (1 << (i110 & 7))) !== 0 ? 1 : 0;
                s001 = (bits[i001 >> 3] & (1 << (i001 & 7))) !== 0 ? 1 : 0;
                s101 = (bits[i101 >> 3] & (1 << (i101 & 7))) !== 0 ? 1 : 0;
                s011 = (bits[i011 >> 3] & (1 << (i011 & 7))) !== 0 ? 1 : 0;
                s111 = (bits[i111 >> 3] & (1 << (i111 & 7))) !== 0 ? 1 : 0;
            } else {
                s000 = this.#sdfInsideValue(ix, iy, iz);
                s100 = this.#sdfInsideValue(ix + 1, iy, iz);
                s010 = this.#sdfInsideValue(ix, iy + 1, iz);
                s110 = this.#sdfInsideValue(ix + 1, iy + 1, iz);
                s001 = this.#sdfInsideValue(ix, iy, iz + 1);
                s101 = this.#sdfInsideValue(ix + 1, iy, iz + 1);
                s011 = this.#sdfInsideValue(ix, iy + 1, iz + 1);
                s111 = this.#sdfInsideValue(ix + 1, iy + 1, iz + 1);
            }
            const sx00 = s000 + (s100 - s000) * fx;
            const sx10 = s010 + (s110 - s010) * fx;
            const sx01 = s001 + (s101 - s001) * fx;
            const sx11 = s011 + (s111 - s011) * fx;
            const sy0 = sx00 + (sx10 - sx00) * fy;
            const sy1 = sx01 + (sx11 - sx01) * fy;
            const insideCount = s000 + s100 + s010 + s110 + s001 + s101 + s011 + s111;
            if (
                insideCount > 0 &&
                insideCount < 8 &&
                this.packedLumenField
            ) {
                normalMultiplier = this.#cachedLumenInside(x, y, z) ? 1 : -1;
                this.stats[STAT_SIGN_REFINEMENTS]++;
            } else {
                normalMultiplier = sy0 + (sy1 - sy0) * fz >= 0.5 ? 1 : -1;
            }
            stateValues[DISTANCE_SIGNED_DISTANCE] = unsignedDistance * normalMultiplier;
            }
        } else {
            const lumenDistance = this.packedLumenField
                ? this.packedLumenField.queryCoordinates(x, y, z, this._lumenQuery).signedDistance
                : centerlineValues[CENTERLINE_SIGNED_DISTANCE];
            normalMultiplier = lumenDistance >= 0 ? 1 : -1;
            stateValues[DISTANCE_SIGNED_DISTANCE] = unsignedDistance * normalMultiplier;
        }

        const dx0 = (corners[1] - corners[0]) * (1 - fy) + (corners[3] - corners[2]) * fy;
        const dx1 = (corners[5] - corners[4]) * (1 - fy) + (corners[7] - corners[6]) * fy;
        const dy0 = (corners[2] - corners[0]) * (1 - fx) + (corners[3] - corners[1]) * fx;
        const dy1 = (corners[6] - corners[4]) * (1 - fx) + (corners[7] - corners[5]) * fx;
        let nx = (dx0 * (1 - fz) + dx1 * fz) / this.voxelSize;
        let ny = (dy0 * (1 - fz) + dy1 * fz) / this.voxelSize;
        let nz = (y1 - y0) / this.voxelSize;
        const length = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (length > EPSILON) {
            nx /= length;
            ny /= length;
            nz /= length;
        }
        if (
            !deferExteriorCorrection &&
            stateValues[DISTANCE_SIGNED_DISTANCE] < 0 &&
            (
                centerlineValues[CENTERLINE_NEAREST_DISTANCE] <= 0.001 ||
                centerlineValues[CENTERLINE_NEAREST_DISTANCE] + 0.2 <
                    -stateValues[DISTANCE_SIGNED_DISTANCE]
            )
        ) {
            stateValues[DISTANCE_SIGNED_DISTANCE] = -stateValues[DISTANCE_SIGNED_DISTANCE];
            normalMultiplier = -normalMultiplier;
        }
        stateValues[DISTANCE_INWARD_X] = nx * normalMultiplier;
        stateValues[DISTANCE_INWARD_Y] = ny * normalMultiplier;
        stateValues[DISTANCE_INWARD_Z] = nz * normalMultiplier;
        return true;
    }

    #cachedLumenInside(x, y, z) {
        const qx = Math.round((x - this.sdfOrigin[0]) * SIGN_CACHE_INV_SPACING);
        const qy = Math.round((y - this.sdfOrigin[1]) * SIGN_CACHE_INV_SPACING);
        const qz = Math.round((z - this.sdfOrigin[2]) * SIGN_CACHE_INV_SPACING);
        if (qx < 0 || qx > 0xffff || qy < 0 || qy > 0x1ffff || qz < 0 || qz > 0xffff) {
            this.stats[STAT_SIGN_CACHE_MISSES]++;
            return this.packedLumenField.isInsideCoordinates(x, y, z);
        }
        const keyLow = (qx & 0xffff) | ((qy & 0xffff) << 16);
        const keyHigh = (qy >>> 16) | (qz << 1);
        const setIndex = (
            Math.imul(qx, 73856093) ^
            Math.imul(qy, 19349663) ^
            Math.imul(qz, 83492791)
        ) & SIGN_CACHE_SET_MASK;
        const firstSlot = setIndex << 1;
        const secondSlot = firstSlot + 1;
        if (
            this.signCacheValid[firstSlot] &&
            this.signCacheKeyLow[firstSlot] === keyLow &&
            this.signCacheKeyHigh[firstSlot] === keyHigh
        ) {
            this.signCacheVictim[setIndex] = 1;
            this.stats[STAT_SIGN_CACHE_HITS]++;
            return this.signCacheInside[firstSlot] !== 0;
        }
        if (
            this.signCacheValid[secondSlot] &&
            this.signCacheKeyLow[secondSlot] === keyLow &&
            this.signCacheKeyHigh[secondSlot] === keyHigh
        ) {
            this.signCacheVictim[setIndex] = 0;
            this.stats[STAT_SIGN_CACHE_HITS]++;
            return this.signCacheInside[secondSlot] !== 0;
        }

        const inside = this.packedLumenField.isInsideCoordinates(x, y, z);
        let slot;
        if (!this.signCacheValid[firstSlot]) slot = firstSlot;
        else if (!this.signCacheValid[secondSlot]) slot = secondSlot;
        else slot = firstSlot + this.signCacheVictim[setIndex];
        this.signCacheKeyLow[slot] = keyLow;
        this.signCacheKeyHigh[slot] = keyHigh;
        this.signCacheInside[slot] = inside ? 1 : 0;
        this.signCacheValid[slot] = 1;
        this.signCacheVictim[setIndex] = slot === firstSlot ? 1 : 0;
        this.stats[STAT_SIGN_CACHE_MISSES]++;
        return inside;
    }

    #refineWithBvh(x, y, z, radius, state) {
        const boundsTree = this.fallbackGeometry?.boundsTree;
        const stateValues = state.values;
        const signedGap = stateValues[DISTANCE_SIGNED_DISTANCE] - radius;
        const validationDistance = radius > 0
            ? this.capsuleBvhValidationDistance
            : this.bvhValidationDistance;
        if (
            !boundsTree ||
            (Math.abs(signedGap) > validationDistance && (radius <= 0 || signedGap >= -0.2))
        ) return false;
        this._bvhPoint.set(x, y, z);
        const hintedFace = this._bvhClosest.faceIndex;
        this._bvhClosest.distance = Infinity;
        // The trilinear SDF value bounds the exact distance up to one voxel
        // diagonal (plus distance quantization). Give that conservative bound
        // to the BVH so it can reject remote subtrees before descending them.
        // This still returns the exact closest triangle; if an asset ever
        // violates the bound, retry unbounded instead of changing contact.
        const sdfDistance = Math.abs(
            stateValues[DISTANCE_SIGNED_DISTANCE]
        );
        const sdfMaximumDistance = sdfDistance +
            this.sdfBvhDistancePadding;
        let scalarHit = closestPointToPointScalarBvh(
            boundsTree,
            x,
            y,
            z,
            this._bvhClosest,
            this._scalarBvhScratch,
            sdfMaximumDistance,
            hintedFace
        );
        // The SDF bound is conservative for production assets, but retain an
        // exact unbounded retry for malformed or unusually quantized assets.
        // Use the same allocation-free scalar BVH before delegating to the
        // library's generic object-heavy implementation.  Both searches solve
        // the identical closest-triangle query and therefore do not alter the
        // contact equations or tolerances.
        if (!scalarHit) {
            this._bvhClosest.distance = Infinity;
            scalarHit = closestPointToPointScalarBvh(
                boundsTree,
                x,
                y,
                z,
                this._bvhClosest,
                this._scalarBvhScratch,
                Infinity,
                hintedFace
            );
        }
        let hit = scalarHit ? this._bvhClosest : null;
        if (!hit) {
            // Unsupported geometry/layout retains the library's general path.
            this._bvhClosest.distance = Infinity;
            hit = boundsTree.closestPointToPoint(
                this._bvhPoint,
                this._bvhClosest,
                0,
                sdfMaximumDistance
            );
            if (!hit) {
                this._bvhClosest.distance = Infinity;
                hit = boundsTree.closestPointToPoint(
                    this._bvhPoint,
                    this._bvhClosest
                );
            }
        }
        const distance = hit?.distance ?? this._bvhPoint.distanceTo(this._bvhClosest.point);
        if (!Number.isFinite(distance)) return false;
        const sign = stateValues[DISTANCE_SIGNED_DISTANCE] >= 0 ? 1 : -1;
        stateValues[DISTANCE_SIGNED_DISTANCE] = distance * sign;
        if (distance > EPSILON) {
            stateValues[DISTANCE_INWARD_X] = (x - this._bvhClosest.point.x) / distance * sign;
            stateValues[DISTANCE_INWARD_Y] = (y - this._bvhClosest.point.y) / distance * sign;
            stateValues[DISTANCE_INWARD_Z] = (z - this._bvhClosest.point.z) / distance * sign;
        }
        state.source = SOURCE_SDF_BVH;
        state.faceIndex = this._bvhClosest.faceIndex;
        this.stats[STAT_BVH_REFINEMENTS]++;
        this.stats[STAT_BVH_CONTACT_REFINEMENTS]++;
        return true;
    }

    #writeSdfValue(output, outputIndex, ix, iy, iz) {
        if (ix < 0 || iy < 0 || iz < 0) {
            output[outputIndex] = NaN;
            return;
        }
        const brickX = Math.floor(ix / this.brickSize);
        const brickY = Math.floor(iy / this.brickSize);
        const brickZ = Math.floor(iz / this.brickSize);
        if (
            brickX >= this.sdfDimensions[0] ||
            brickY >= this.sdfDimensions[1] ||
            brickZ >= this.sdfDimensions[2]
        ) {
            output[outputIndex] = NaN;
            return;
        }
        const key = brickX + this.sdfDimensions[0] * (brickY + this.sdfDimensions[1] * brickZ);
        const brickIndex = this.sdfBrickLookup[key];
        if (brickIndex === this.sdfMissingBrick) {
            output[outputIndex] = NaN;
            return;
        }
        const localX = ix - brickX * this.brickSize;
        const localY = iy - brickY * this.brickSize;
        const localZ = iz - brickZ * this.brickSize;
        const localIndex = localX + this.brickSize * (localY + this.brickSize * localZ);
        const valueIndex = brickIndex * this.valuesPerBrick + localIndex;
        output[outputIndex] = this.sdfDistances[valueIndex] * this.sdfQuantization;
    }

    #sdfInsideValue(ix, iy, iz) {
        if (ix < 0 || iy < 0 || iz < 0) return 0;
        const brickX = Math.floor(ix / this.brickSize);
        const brickY = Math.floor(iy / this.brickSize);
        const brickZ = Math.floor(iz / this.brickSize);
        if (
            brickX >= this.sdfDimensions[0] ||
            brickY >= this.sdfDimensions[1] ||
            brickZ >= this.sdfDimensions[2]
        ) return 0;
        const key = brickX + this.sdfDimensions[0] * (brickY + this.sdfDimensions[1] * brickZ);
        const brickIndex = this.sdfBrickLookup[key];
        if (brickIndex === this.sdfMissingBrick) return 0;
        const localX = ix - brickX * this.brickSize;
        const localY = iy - brickY * this.brickSize;
        const localZ = iz - brickZ * this.brickSize;
        const localIndex = localX + this.brickSize * (localY + this.brickSize * localZ);
        const valueIndex = brickIndex * this.valuesPerBrick + localIndex;
        return (this.sdfInsideBits[valueIndex >> 3] & (1 << (valueIndex & 7))) !== 0 ? 1 : 0;
    }
}

export async function loadVesselContactField(url, options = {}) {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`Failed to load collision asset: ${response.status} ${response.statusText}`);
    const buffer = await response.arrayBuffer();
    return new VesselContactField(decodeCollisionAsset(buffer), options);
}
