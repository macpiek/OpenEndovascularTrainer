import * as THREE from 'three';
import { decodeCollisionAsset } from './collisionAssetFormat.js';
import { createPackedLumenField, createPackedLumenQueryResult } from './packedLumenField.js';

const SOURCE_SAFE_CORE = 'centerline-safe-core';
const SOURCE_SDF = 'sparse-sdf';
const SOURCE_SDF_BVH = 'sparse-sdf-bvh';
const SOURCE_FALLBACK = 'fallback';
const SOURCE_CENTERLINE = 'centerline-estimate';
const EPSILON = 1e-8;
const SIGN_CACHE_SIZE = 1 << 17;
const SIGN_CACHE_SET_COUNT = SIGN_CACHE_SIZE >> 1;
const SIGN_CACHE_SET_MASK = SIGN_CACHE_SET_COUNT - 1;
const SIGN_CACHE_INV_SPACING = 200;
const SDF_MISSING_BRICK = 0xffff;
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
const STAT_COUNT = 15;
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
        this.sdfOrigin = sdf.origin;
        this.sdfDimensions = sdf.dimensions;
        const lookupLength = this.sdfDimensions[0] * this.sdfDimensions[1] * this.sdfDimensions[2];
        if (this.sdfBrickKeys.length >= SDF_MISSING_BRICK) {
            throw new RangeError('Sparse SDF has too many bricks for its runtime lookup');
        }
        this.sdfBrickLookup = new Uint16Array(lookupLength);
        this.sdfBrickLookup.fill(SDF_MISSING_BRICK);
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
        this._sdfCornerScratch = new Float64Array(8);
        this._capsuleCoordinateScratch = new Float64Array(7);
        this._centerlineQueryScratch = new Float64Array(3);
        this.runtimeBytes = decoded.metadata.decodedBytes + this.sdfBrickLookup.byteLength +
            this.signCacheKeyLow.byteLength + this.signCacheKeyHigh.byteLength +
            this.signCacheInside.byteLength + this.signCacheValid.byteLength +
            this.signCacheVictim.byteLength + this._sdfCornerScratch.byteLength +
            this._capsuleCoordinateScratch.byteLength + this._centerlineQueryScratch.byteLength;
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

    #querySphereCoordinates(x, y, z, toolRadius, target) {
        this.stats[STAT_POINT_QUERIES]++;
        const state = this.#queryDistance(x, y, z, toolRadius);
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
        const coordinates = this._capsuleCoordinateScratch;
        coordinates[0] = ax;
        coordinates[1] = ay;
        coordinates[2] = az;
        coordinates[3] = bx;
        coordinates[4] = by;
        coordinates[5] = bz;
        coordinates[6] = radius;
        return this.#queryCapsuleStored(out);
    }

    queryCapsuleSoA(x, y, z, radii, index, out = null) {
        const coordinates = this._capsuleCoordinateScratch;
        coordinates[0] = x[index];
        coordinates[1] = y[index];
        coordinates[2] = z[index];
        coordinates[3] = x[index + 1];
        coordinates[4] = y[index + 1];
        coordinates[5] = z[index + 1];
        coordinates[6] = Math.max(radii[index], radii[index + 1]);
        return this.#queryCapsuleStored(out);
    }

    #queryCapsuleStored(out) {
        const coordinates = this._capsuleCoordinateScratch;
        const ax = coordinates[0];
        const ay = coordinates[1];
        const az = coordinates[2];
        const bx = coordinates[3];
        const by = coordinates[4];
        const bz = coordinates[5];
        const radius = coordinates[6];
        if (!out) this.stats[STAT_RESULT_ALLOCATIONS]++;
        const target = out || createContactResult();
        const toolRadius = Math.max(0, radius || 0);
        const dx = bx - ax;
        const dy = by - ay;
        const dz = bz - az;
        const length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const spacing = Math.max(this.voxelSize * 4, Math.max(0.5, toolRadius));
        const sampleCount = Math.max(1, Math.ceil(length / spacing));
        let bestGap = Infinity;
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
            contact = this.#querySphereCoordinates(ax, ay, az, toolRadius, this._capsuleContact);
            this.stats[STAT_CAPSULE_SAMPLES]++;
        }
        bestGap = contact.values[CONTACT_SIGNED_GAP];
        const startGap = bestGap;
        const startNormalX = contact.inward.values[0];
        const startNormalY = contact.inward.values[1];
        const startNormalZ = contact.inward.values[2];
        let endGap = startGap;
        let normalAgreement = 1;
        setContact(target, contact);

        if (length > EPSILON) {
            contact = this.#querySphereCoordinates(bx, by, bz, toolRadius, this._capsuleContact);
            this.stats[STAT_CAPSULE_SAMPLES]++;
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
                this._capsuleContact
            );
            this.stats[STAT_CAPSULE_SAMPLES]++;
            const gap = contact.values[CONTACT_SIGNED_GAP];
            if (gap < bestGap) {
                bestGap = gap;
                bestT = t;
                bestSampleIndex = sampleIndex;
                setContact(target, contact);
            }
        }
        this._skipBvhValidation = false;
        if (bestGap <= this.capsuleBvhValidationGap) {
            bestT = bestSampleIndex / sampleCount;
            setContact(target, this.#querySphereCoordinates(
                ax + dx * bestT,
                ay + dy * bestT,
                az + dz * bestT,
                toolRadius,
                this._capsuleContact
            ));
        }
        target.values[CONTACT_SEGMENT_T] = bestT;
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

    #queryDistance(x, y, z, radius) {
        const centerline = this.#queryCenterline(x, y, z);
        const centerlineValues = centerline.values;
        const state = this._distanceState;
        const stateValues = state.values;
        if (
            centerline.found &&
            centerlineValues[CENTERLINE_SAFE_DISTANCE] > radius + this.voxelSize * 0.25
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

        if (this.#querySdf(x, y, z, state)) {
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

    #queryCenterline(x, y, z) {
        const query = this._centerlineQueryScratch;
        query[0] = x;
        query[1] = y;
        query[2] = z;
        const state = this._centerlineState;
        const stateValues = state.values;
        state.found = false;
        stateValues[CENTERLINE_BRANCH_ID] = -1;
        stateValues[CENTERLINE_T] = 0;
        stateValues[CENTERLINE_SIGNED_DISTANCE] = -Infinity;
        stateValues[CENTERLINE_SAFE_DISTANCE] = -Infinity;
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
            const start = this.broadPhaseOffsets[cellIndex];
            const end = this.broadPhaseOffsets[cellIndex + 1];
            for (let entry = start; entry < end; entry++) {
                this.#considerCenterlineSegment(this.broadPhaseIds[entry], state);
            }
        }

        if (!state.found) {
            const segmentCount = this.centerline.length / this.centerlineStride;
            for (let segmentId = 0; segmentId < segmentCount; segmentId++) {
                this.#considerCenterlineSegment(segmentId, state);
            }
        }
        return state;
    }

    #considerCenterlineSegment(segmentId, state) {
        const query = this._centerlineQueryScratch;
        const x = query[0];
        const y = query[1];
        const z = query[2];
        const stateValues = state.values;
        const offset = segmentId * this.centerlineStride;
        const ax = this.centerline[offset];
        const ay = this.centerline[offset + 1];
        const az = this.centerline[offset + 2];
        const dx = this.centerline[offset + 3] - ax;
        const dy = this.centerline[offset + 4] - ay;
        const dz = this.centerline[offset + 5] - az;
        const lengthSq = dx * dx + dy * dy + dz * dz;
        const t = clamp(((x - ax) * dx + (y - ay) * dy + (z - az) * dz) / Math.max(EPSILON, lengthSq), 0, 1);
        const cx = ax + dx * t;
        const cy = ay + dy * t;
        const cz = az + dz * t;
        const rx = cx - x;
        const ry = cy - y;
        const rz = cz - z;
        const radialDistance = Math.sqrt(rx * rx + ry * ry + rz * rz);
        const radius = this.centerline[offset + 6] * (1 - t) + this.centerline[offset + 7] * t;
        const safeRadius = this.centerline[offset + 8];
        const signedDistance = radius - radialDistance;
        const safeDistance = safeRadius - radialDistance;
        stateValues[CENTERLINE_NEAREST_DISTANCE] = Math.min(
            stateValues[CENTERLINE_NEAREST_DISTANCE],
            radialDistance
        );

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

        if (safeDistance > stateValues[CENTERLINE_SAFE_DISTANCE]) {
            stateValues[CENTERLINE_SAFE_DISTANCE] = safeDistance;
            stateValues[CENTERLINE_SAFE_BRANCH_ID] = segmentId;
            stateValues[CENTERLINE_SAFE_INWARD_X] = inwardX;
            stateValues[CENTERLINE_SAFE_INWARD_Y] = inwardY;
            stateValues[CENTERLINE_SAFE_INWARD_Z] = inwardZ;
        }
        if (state.found && signedDistance <= stateValues[CENTERLINE_SIGNED_DISTANCE]) return;

        state.found = true;
        stateValues[CENTERLINE_BRANCH_ID] = segmentId;
        stateValues[CENTERLINE_T] = t;
        stateValues[CENTERLINE_SIGNED_DISTANCE] = signedDistance;
        stateValues[CENTERLINE_INWARD_X] = inwardX;
        stateValues[CENTERLINE_INWARD_Y] = inwardY;
        stateValues[CENTERLINE_INWARD_Z] = inwardZ;
    }

    #querySdf(x, y, z, state) {
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
            if (brickIndex !== SDF_MISSING_BRICK) {
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
            if (insideCount > 0 && insideCount < 8 && this.packedLumenField) {
                normalMultiplier = this.#cachedLumenInside(x, y, z) ? 1 : -1;
                this.stats[STAT_SIGN_REFINEMENTS]++;
            } else {
                normalMultiplier = sy0 + (sy1 - sy0) * fz >= 0.5 ? 1 : -1;
            }
            stateValues[DISTANCE_SIGNED_DISTANCE] = unsignedDistance * normalMultiplier;
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
            ? Math.min(this.bvhValidationDistance, 0.25)
            : this.bvhValidationDistance;
        if (
            !boundsTree ||
            (Math.abs(signedGap) > validationDistance && (radius <= 0 || signedGap >= -0.2))
        ) return false;
        this._bvhPoint.set(x, y, z);
        this._bvhClosest.distance = Infinity;
        const hit = boundsTree.closestPointToPoint(this._bvhPoint, this._bvhClosest);
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
        this.stats[STAT_BVH_REFINEMENTS]++;
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
        if (brickIndex === SDF_MISSING_BRICK) {
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
        if (brickIndex === SDF_MISSING_BRICK) return 0;
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
