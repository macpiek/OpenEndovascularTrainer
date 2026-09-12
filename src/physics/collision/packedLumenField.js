const EPSILON = 1e-8;
const SLICE_SIGNED_DISTANCE = 0;
const SLICE_INWARD_X = 1;
const SLICE_INWARD_Z = 2;
const SLICE_CLOSEST_X = 3;
const SLICE_CLOSEST_Z = 4;
const SLICE_CONTOUR_INDEX = 5;
const INTERVAL_LOWER = 0;
const INTERVAL_UPPER = 1;
const INTERVAL_T = 2;

function setVector(target, x, y, z) {
    target.x = x;
    target.y = y;
    target.z = z;
    return target;
}

function createSliceScratch() {
    const scratch = new Float64Array(6);
    scratch[SLICE_SIGNED_DISTANCE] = -Infinity;
    scratch[SLICE_INWARD_X] = 1;
    scratch[SLICE_CONTOUR_INDEX] = -1;
    return scratch;
}

export function createPackedLumenQueryResult() {
    return {
        inside: false,
        signedDistance: -Infinity,
        distance: Infinity,
        inward: { x: 1, y: 0, z: 0 },
        normal: { x: -1, y: 0, z: 0 },
        closestPoint: { x: 0, y: 0, z: 0 },
        lowerSliceIndex: -1,
        upperSliceIndex: -1
    };
}

export class PackedLumenField {
    constructor(metadata, arrays) {
        this.metadata = metadata.lumen;
        this.sliceYs = arrays.lumenSliceYs;
        this.sliceContourOffsets = arrays.lumenSliceContourOffsets;
        this.contourPointOffsets = arrays.lumenContourPointOffsets;
        this.contourBounds = arrays.lumenContourBounds;
        this.contourSamples = arrays.lumenContourSamples;
        this.points = arrays.lumenPoints;
        this.pointQuantization = this.points instanceof Int16Array
            ? this.metadata.pointQuantization || 0.02
            : 1;
        this.axisBases = arrays.lumenAxisBases || new Float32Array([
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        ]);
        this.axisSliceOffsets = arrays.lumenAxisSliceOffsets || new Uint32Array([0, this.sliceYs.length]);
        this.axisCount = Math.max(1, this.axisSliceOffsets.length - 1);
        this._lower = createSliceScratch();
        this._upper = createSliceScratch();
        this._interval = new Float64Array(3);
        this._lastLower = new Int32Array(this.axisCount);
        this._lastUpper = new Int32Array(this.axisCount);
        for (let axisIndex = 0; axisIndex < this.axisCount; axisIndex++) {
            const start = this.axisSliceOffsets[axisIndex];
            const end = this.axisSliceOffsets[axisIndex + 1];
            this._lastLower[axisIndex] = start;
            this._lastUpper[axisIndex] = Math.min(start + 1, Math.max(start, end - 1));
        }
    }

    query(input, out = null) {
        return this.queryCoordinates(input.x, input.y, input.z, out);
    }

    isInsideCoordinates(x, y, z) {
        for (let axisIndex = 0; axisIndex < this.axisCount; axisIndex++) {
            const basisOffset = axisIndex * 9;
            const localX =
                x * this.axisBases[basisOffset] +
                y * this.axisBases[basisOffset + 1] +
                z * this.axisBases[basisOffset + 2];
            const localY =
                x * this.axisBases[basisOffset + 3] +
                y * this.axisBases[basisOffset + 4] +
                z * this.axisBases[basisOffset + 5];
            const localZ =
                x * this.axisBases[basisOffset + 6] +
                y * this.axisBases[basisOffset + 7] +
                z * this.axisBases[basisOffset + 8];
            const interval = this.#findInterval(localY, axisIndex);
            if (interval[INTERVAL_LOWER] < 0) continue;
            if (
                !this.#sliceBoundsContain(interval[INTERVAL_LOWER], localX, localZ) &&
                (
                    interval[INTERVAL_UPPER] === interval[INTERVAL_LOWER] ||
                    !this.#sliceBoundsContain(interval[INTERVAL_UPPER], localX, localZ)
                )
            ) continue;
            const lower = this.#querySliceSignedDistance(
                interval[INTERVAL_LOWER],
                localX,
                localZ
            );
            const upper = interval[INTERVAL_UPPER] === interval[INTERVAL_LOWER]
                ? lower
                : this.#querySliceSignedDistance(
                    interval[INTERVAL_UPPER],
                    localX,
                    localZ
                );
            const t = interval[INTERVAL_T];
            if (lower * (1 - t) + upper * t >= 0) {
                return true;
            }
        }
        return false;
    }

    /** Sufficient (not necessary) inside proof for a WORLD-coordinate ball.
     * One axis suffices because axis interiors form a union. The entire ball
     * must stay in one finite interpolation slab, and both signed polygon
     * distances must exceed its projected XZ radius. Signed polygon distance
     * is 1-Lipschitz; their positive interpolation then stays inside. The XZ
     * Frobenius bound remains conservative for nonorthonormal stored bases.
     * No query result, interval hint or cache is changed by this diagnostic.
     */
    certifyInsideBallCoordinates(x, y, z, radius) {
        if (![x, y, z, radius].every(Number.isFinite) || radius < 0) {
            throw new RangeError('Finite ball coordinates and nonnegative radius required');
        }
        const lowerHint = this._lastLower.slice(), upperHint = this._lastUpper.slice(), intervalScratch = this._interval.slice();
        let reason = 'no-supported-axis-slab';
        try {
            for (let axis = 0; axis < this.axisCount; axis++) {
                const offset = axis * 9, basis = Array.from(this.axisBases.slice(offset, offset + 9));
                if (basis.length !== 9 || !basis.every(Number.isFinite)) continue;
                const local = [0, 1, 2].map(i => x * basis[3*i] + y * basis[3*i+1] + z * basis[3*i+2]);
                const normX = Math.hypot(...basis.slice(0, 3)), normY = Math.hypot(...basis.slice(3, 6)), normZ = Math.hypot(...basis.slice(6, 9));
                if (!local.every(Number.isFinite) || !(normY > 0)) continue;
                const interval = this.#findInterval(local[1], axis), lower = interval[INTERVAL_LOWER], upper = interval[INTERVAL_UPPER];
                if (lower < 0 || upper <= lower) continue;
                const bottom = this.sliceYs[lower], top = this.sliceYs[upper];
                if (![bottom, top].every(Number.isFinite) || !(top > bottom)) continue;
                const yRadius = radius * normY, xzRadius = radius * Math.hypot(normX, normZ);
                const guard = 64 * Number.EPSILON * Math.max(1, Math.abs(x), Math.abs(y), Math.abs(z), ...local.map(Math.abs), Math.abs(bottom), Math.abs(top), yRadius, xzRadius);
                const slabMargin = Math.min(local[1] - bottom, top - local[1]) - yRadius - guard;
                if (!(slabMargin >= 0)) { reason = 'ball-exits-axis-slab'; continue; }
                const lowerDistance = this.#querySliceSignedDistance(lower, local[0], local[2]);
                const upperDistance = this.#querySliceSignedDistance(upper, local[0], local[2]);
                const sliceMargin = Math.min(lowerDistance, upperDistance) - xzRadius - guard;
                if (!Number.isFinite(sliceMargin) || !(sliceMargin > 0)) { reason = 'slice-clearance-not-proved'; continue; }
                return {supported: true, axis, slab: {lowerSliceIndex: lower, upperSliceIndex: upper, lower: bottom, upper: top},
                    radius, localCenter: local, localYRadius: yRadius, localXZRadius: xzRadius,
                    lowerDistance, upperDistance, slabMargin, sliceMargin, minimumMargin: Math.min(slabMargin, sliceMargin),
                    numericalGuard: guard, scope: 'one-axis-slab-two-signed-polygon-distances'};
            }
            return {supported: false, axis: null, slab: null, minimumMargin: null, reason};
        } finally {
            this._lastLower.set(lowerHint); this._lastUpper.set(upperHint); this._interval.set(intervalScratch);
        }
    }

    #sliceBoundsContain(sliceIndex, x, z) {
        const contourStart = this.sliceContourOffsets[sliceIndex];
        const contourEnd = this.sliceContourOffsets[sliceIndex + 1];
        for (let contourIndex = contourStart; contourIndex < contourEnd; contourIndex++) {
            const offset = contourIndex * 4;
            if (
                x >= this.contourBounds[offset] &&
                x <= this.contourBounds[offset + 1] &&
                z >= this.contourBounds[offset + 2] &&
                z <= this.contourBounds[offset + 3]
            ) return true;
        }
        return false;
    }

    queryCoordinates(x, y, z, out = null) {
        const target = out || createPackedLumenQueryResult();
        if (!this.sliceYs.length) {
            target.inside = false;
            target.signedDistance = -Infinity;
            target.distance = Infinity;
            setVector(target.inward, 1, 0, 0);
            setVector(target.normal, -1, 0, 0);
            setVector(target.closestPoint, x, y, z);
            target.lowerSliceIndex = -1;
            target.upperSliceIndex = -1;
            return target;
        }

        let bestSignedDistance = -Infinity;
        let bestInwardX = 1;
        let bestInwardY = 0;
        let bestInwardZ = 0;
        let bestLowerSlice = -1;
        let bestUpperSlice = -1;
        for (let axisIndex = 0; axisIndex < this.axisCount; axisIndex++) {
            const basisOffset = axisIndex * 9;
            const localX =
                x * this.axisBases[basisOffset] +
                y * this.axisBases[basisOffset + 1] +
                z * this.axisBases[basisOffset + 2];
            const localY =
                x * this.axisBases[basisOffset + 3] +
                y * this.axisBases[basisOffset + 4] +
                z * this.axisBases[basisOffset + 5];
            const localZ =
                x * this.axisBases[basisOffset + 6] +
                y * this.axisBases[basisOffset + 7] +
                z * this.axisBases[basisOffset + 8];
            const interval = this.#findInterval(localY, axisIndex);
            if (interval[INTERVAL_LOWER] < 0) continue;
            const lower = this.#querySlice(interval[INTERVAL_LOWER], localX, localZ, this._lower);
            const upper = interval[INTERVAL_UPPER] === interval[INTERVAL_LOWER]
                ? lower
                : this.#querySlice(interval[INTERVAL_UPPER], localX, localZ, this._upper);
            const t = interval[INTERVAL_T];
            const signedDistance = lower[SLICE_SIGNED_DISTANCE] * (1 - t) + upper[SLICE_SIGNED_DISTANCE] * t;
            if (signedDistance <= bestSignedDistance) continue;
            const dy = Math.max(
                EPSILON,
                Math.abs(this.sliceYs[interval[INTERVAL_UPPER]] - this.sliceYs[interval[INTERVAL_LOWER]])
            );
            const yGradient = interval[INTERVAL_UPPER] === interval[INTERVAL_LOWER]
                ? 0
                : Math.max(-0.85, Math.min(0.85, (
                    upper[SLICE_SIGNED_DISTANCE] - lower[SLICE_SIGNED_DISTANCE]
                ) / dy));
            let localInwardX = lower[SLICE_INWARD_X] * (1 - t) + upper[SLICE_INWARD_X] * t;
            let localInwardY = yGradient;
            let localInwardZ = lower[SLICE_INWARD_Z] * (1 - t) + upper[SLICE_INWARD_Z] * t;
            const localLength = Math.sqrt(
                localInwardX * localInwardX +
                localInwardY * localInwardY +
                localInwardZ * localInwardZ
            );
            if (localLength > EPSILON) {
                localInwardX /= localLength;
                localInwardY /= localLength;
                localInwardZ /= localLength;
            } else {
                localInwardX = 1;
                localInwardY = 0;
                localInwardZ = 0;
            }
            let worldInwardX =
                this.axisBases[basisOffset] * localInwardX +
                this.axisBases[basisOffset + 3] * localInwardY +
                this.axisBases[basisOffset + 6] * localInwardZ;
            let worldInwardY =
                this.axisBases[basisOffset + 1] * localInwardX +
                this.axisBases[basisOffset + 4] * localInwardY +
                this.axisBases[basisOffset + 7] * localInwardZ;
            let worldInwardZ =
                this.axisBases[basisOffset + 2] * localInwardX +
                this.axisBases[basisOffset + 5] * localInwardY +
                this.axisBases[basisOffset + 8] * localInwardZ;
            const worldLength = Math.sqrt(
                worldInwardX * worldInwardX +
                worldInwardY * worldInwardY +
                worldInwardZ * worldInwardZ
            ) || 1;
            worldInwardX /= worldLength;
            worldInwardY /= worldLength;
            worldInwardZ /= worldLength;
            bestSignedDistance = signedDistance;
            bestInwardX = worldInwardX;
            bestInwardY = worldInwardY;
            bestInwardZ = worldInwardZ;
            bestLowerSlice = interval[INTERVAL_LOWER];
            bestUpperSlice = interval[INTERVAL_UPPER];
        }

        target.inside = bestSignedDistance >= 0;
        target.signedDistance = bestSignedDistance;
        target.distance = Math.abs(bestSignedDistance);
        setVector(target.inward, bestInwardX, bestInwardY, bestInwardZ);
        setVector(target.normal, -bestInwardX, -bestInwardY, -bestInwardZ);
        setVector(
            target.closestPoint,
            x - bestInwardX * bestSignedDistance,
            y - bestInwardY * bestSignedDistance,
            z - bestInwardZ * bestSignedDistance
        );
        target.lowerSliceIndex = bestLowerSlice;
        target.upperSliceIndex = bestUpperSlice;
        return target;
    }

    #findInterval(y, axisIndex) {
        const interval = this._interval;
        const first = this.axisSliceOffsets[axisIndex];
        const last = this.axisSliceOffsets[axisIndex + 1] - 1;
        if (last < first) {
            interval[INTERVAL_LOWER] = -1;
            interval[INTERVAL_UPPER] = -1;
            interval[INTERVAL_T] = 0;
            return interval;
        }
        if (last === first || y <= this.sliceYs[first]) {
            this._lastLower[axisIndex] = first;
            this._lastUpper[axisIndex] = first;
            interval[INTERVAL_LOWER] = first;
            interval[INTERVAL_UPPER] = first;
            interval[INTERVAL_T] = 0;
            return interval;
        }
        if (y >= this.sliceYs[last]) {
            this._lastLower[axisIndex] = last;
            this._lastUpper[axisIndex] = last;
            interval[INTERVAL_LOWER] = last;
            interval[INTERVAL_UPPER] = last;
            interval[INTERVAL_T] = 0;
            return interval;
        }

        let lower = this._lastLower[axisIndex];
        let upper = this._lastUpper[axisIndex];
        if (
            lower >= first && upper > lower && upper <= last &&
            this.sliceYs[lower] <= y && y <= this.sliceYs[upper]
        ) {
            const span = Math.max(EPSILON, this.sliceYs[upper] - this.sliceYs[lower]);
            interval[INTERVAL_LOWER] = lower;
            interval[INTERVAL_UPPER] = upper;
            interval[INTERVAL_T] = (y - this.sliceYs[lower]) / span;
            return interval;
        }

        let lo = first;
        let hi = last;
        while (hi - lo > 1) {
            const mid = Math.floor((lo + hi) * 0.5);
            if (this.sliceYs[mid] <= y) lo = mid;
            else hi = mid;
        }
        lower = lo;
        upper = hi;
        this._lastLower[axisIndex] = lower;
        this._lastUpper[axisIndex] = upper;
        const span = Math.max(EPSILON, this.sliceYs[upper] - this.sliceYs[lower]);
        interval[INTERVAL_LOWER] = lower;
        interval[INTERVAL_UPPER] = upper;
        interval[INTERVAL_T] = (y - this.sliceYs[lower]) / span;
        return interval;
    }

    #querySlice(sliceIndex, x, z, out) {
        let bestSignedDistance = -Infinity;
        let bestInwardX = 1;
        let bestInwardZ = 0;
        let bestClosestX = x;
        let bestClosestZ = z;
        let bestContour = -1;
        const contourStart = this.sliceContourOffsets[sliceIndex];
        const contourEnd = this.sliceContourOffsets[sliceIndex + 1];

        for (let contourIndex = contourStart; contourIndex < contourEnd; contourIndex++) {
            const boundsOffset = contourIndex * 4;
            let boundsDx = 0;
            let boundsDz = 0;
            if (x < this.contourBounds[boundsOffset]) boundsDx = this.contourBounds[boundsOffset] - x;
            else if (x > this.contourBounds[boundsOffset + 1]) boundsDx = x - this.contourBounds[boundsOffset + 1];
            if (z < this.contourBounds[boundsOffset + 2]) boundsDz = this.contourBounds[boundsOffset + 2] - z;
            else if (z > this.contourBounds[boundsOffset + 3]) boundsDz = z - this.contourBounds[boundsOffset + 3];
            if (
                Number.isFinite(bestSignedDistance) && bestSignedDistance < 0 &&
                -Math.sqrt(boundsDx * boundsDx + boundsDz * boundsDz) <= bestSignedDistance
            ) continue;

            const pointStart = this.contourPointOffsets[contourIndex];
            const pointEnd = this.contourPointOffsets[contourIndex + 1];
            if (pointEnd <= pointStart) continue;
            let inside = false;
            let closestX = x;
            let closestZ = z;
            let closestDistanceSq = Infinity;
            const previousIndex = pointEnd - 1;
            // Polygon edges are traversed in order. Carry the previous
            // decoded endpoint instead of loading and dequantizing it again
            // for every edge; the arithmetic and winding test stay exactly
            // the same as the original point-pair formulation.
            let bx = this.points[previousIndex * 2] * this.pointQuantization;
            let bz = this.points[previousIndex * 2 + 1] * this.pointQuantization;

            for (let pointIndex = pointStart; pointIndex < pointEnd; pointIndex++) {
                const ax = this.points[pointIndex * 2] * this.pointQuantization;
                const az = this.points[pointIndex * 2 + 1] * this.pointQuantization;
                if (
                    (az > z) !== (bz > z) &&
                    x < (bx - ax) * (z - az) / (bz - az + 1e-12) + ax
                ) inside = !inside;

                const dx = bx - ax;
                const dz = bz - az;
                const lengthSq = dx * dx + dz * dz || 1;
                const edgeT = Math.max(0, Math.min(1, ((x - ax) * dx + (z - az) * dz) / lengthSq));
                const candidateX = ax + dx * edgeT;
                const candidateZ = az + dz * edgeT;
                const px = x - candidateX;
                const pz = z - candidateZ;
                const distanceSq = px * px + pz * pz;
                if (distanceSq < closestDistanceSq) {
                    closestDistanceSq = distanceSq;
                    closestX = candidateX;
                    closestZ = candidateZ;
                }
                bx = ax;
                bz = az;
            }

            const distance = Math.sqrt(closestDistanceSq);
            const signedDistance = inside ? distance : -distance;
            if (signedDistance <= bestSignedDistance) continue;
            let inwardX = inside ? x - closestX : closestX - x;
            let inwardZ = inside ? z - closestZ : closestZ - z;
            const inwardLength = Math.sqrt(inwardX * inwardX + inwardZ * inwardZ);
            if (inwardLength > EPSILON) {
                inwardX /= inwardLength;
                inwardZ /= inwardLength;
            } else {
                inwardX = this.contourSamples[contourIndex * 2] - closestX;
                inwardZ = this.contourSamples[contourIndex * 2 + 1] - closestZ;
                const fallbackLength = Math.sqrt(inwardX * inwardX + inwardZ * inwardZ) || 1;
                inwardX /= fallbackLength;
                inwardZ /= fallbackLength;
            }
            bestSignedDistance = signedDistance;
            bestInwardX = inwardX;
            bestInwardZ = inwardZ;
            bestClosestX = closestX;
            bestClosestZ = closestZ;
            bestContour = contourIndex;
        }

        out[SLICE_SIGNED_DISTANCE] = bestSignedDistance;
        out[SLICE_INWARD_X] = bestInwardX;
        out[SLICE_INWARD_Z] = bestInwardZ;
        out[SLICE_CLOSEST_X] = bestClosestX;
        out[SLICE_CLOSEST_Z] = bestClosestZ;
        out[SLICE_CONTOUR_INDEX] = bestContour;
        return out;
    }

    // Exact sign-only counterpart of #querySlice. Mixed SDF voxels call
    // isInsideCoordinates very frequently, but that path only consumes the
    // interpolated signed distance. Avoid constructing the closest point and
    // inward normal that the full contact query needs; edge traversal,
    // winding and distance arithmetic stay identical.
    #querySliceSignedDistance(sliceIndex, x, z) {
        let bestSignedDistance = -Infinity;
        const contourStart = this.sliceContourOffsets[sliceIndex];
        const contourEnd = this.sliceContourOffsets[sliceIndex + 1];

        for (let contourIndex = contourStart; contourIndex < contourEnd; contourIndex++) {
            const boundsOffset = contourIndex * 4;
            let boundsDx = 0;
            let boundsDz = 0;
            if (x < this.contourBounds[boundsOffset]) {
                boundsDx = this.contourBounds[boundsOffset] - x;
            } else if (x > this.contourBounds[boundsOffset + 1]) {
                boundsDx = x - this.contourBounds[boundsOffset + 1];
            }
            if (z < this.contourBounds[boundsOffset + 2]) {
                boundsDz = this.contourBounds[boundsOffset + 2] - z;
            } else if (z > this.contourBounds[boundsOffset + 3]) {
                boundsDz = z - this.contourBounds[boundsOffset + 3];
            }
            if (
                Number.isFinite(bestSignedDistance) &&
                bestSignedDistance < 0 &&
                -Math.sqrt(boundsDx * boundsDx + boundsDz * boundsDz) <=
                    bestSignedDistance
            ) continue;

            const pointStart = this.contourPointOffsets[contourIndex];
            const pointEnd = this.contourPointOffsets[contourIndex + 1];
            if (pointEnd <= pointStart) continue;
            let inside = false;
            let closestDistanceSq = Infinity;
            const previousIndex = pointEnd - 1;
            let bx = this.points[previousIndex * 2] * this.pointQuantization;
            let bz = this.points[previousIndex * 2 + 1] * this.pointQuantization;

            for (let pointIndex = pointStart; pointIndex < pointEnd; pointIndex++) {
                const ax = this.points[pointIndex * 2] * this.pointQuantization;
                const az = this.points[pointIndex * 2 + 1] * this.pointQuantization;
                if (
                    (az > z) !== (bz > z) &&
                    x < (bx - ax) * (z - az) / (bz - az + 1e-12) + ax
                ) inside = !inside;

                const dx = bx - ax;
                const dz = bz - az;
                const lengthSq = dx * dx + dz * dz || 1;
                const edgeT = Math.max(
                        0,
                        Math.min(
                            1,
                            ((x - ax) * dx + (z - az) * dz) / lengthSq
                        )
                    );
                const candidateX = ax + dx * edgeT;
                const candidateZ = az + dz * edgeT;
                const px = x - candidateX;
                const pz = z - candidateZ;
                const distanceSq = px * px + pz * pz;
                if (distanceSq < closestDistanceSq) {
                    closestDistanceSq = distanceSq;
                }
                bx = ax;
                bz = az;
            }

            const distance = Math.sqrt(closestDistanceSq);
            const signedDistance = inside ? distance : -distance;
            if (signedDistance > bestSignedDistance) {
                bestSignedDistance = signedDistance;
            }
        }
        return bestSignedDistance;
    }
}

export function createPackedLumenField(metadata, arrays) {
    if (!metadata?.lumen || !arrays?.lumenSliceYs?.length) return null;
    return new PackedLumenField(metadata, arrays);
}
