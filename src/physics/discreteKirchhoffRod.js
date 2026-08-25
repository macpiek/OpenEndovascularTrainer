const EPSILON = 1e-12;
const SMALL_ANGLE = 1e-7;
const TRIG_SERIES_ANGLE_SQUARED = 0.0625;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function setVector(out, x, y, z) {
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

function setQuaternion(out, x, y, z, w) {
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

function vectorLength(vector) {
    return Math.sqrt(
        vector.x * vector.x +
        vector.y * vector.y +
        vector.z * vector.z
    );
}

function normalizeVector(vector, out = {}) {
    const length = vectorLength(vector);
    if (length < EPSILON) return setVector(out, 0, 0, 0);
    const inverse = 1 / length;
    return setVector(
        out,
        vector.x * inverse,
        vector.y * inverse,
        vector.z * inverse
    );
}

function crossVectors(a, b, out = {}) {
    return setVector(
        out,
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

function dotVectors(a, b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Quaternion convention used by this module:
 * - q rotates vectors from material/local coordinates into world coordinates;
 * - multiplication a*b applies b first and a second;
 * - incremental rotations are world-space, left-multiplied rotations.
 */
export function normalizeQuaternion(quaternion, out = {}) {
    const length = Math.sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    if (length < EPSILON) return setQuaternion(out, 0, 0, 0, 1);
    const inverse = 1 / length;
    return setQuaternion(
        out,
        quaternion.x * inverse,
        quaternion.y * inverse,
        quaternion.z * inverse,
        quaternion.w * inverse
    );
}

export function conjugateQuaternion(quaternion, out = {}) {
    return setQuaternion(
        out,
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
        quaternion.w
    );
}

export function multiplyQuaternions(a, b, out = {}) {
    const ax = a.x;
    const ay = a.y;
    const az = a.z;
    const aw = a.w;
    const bx = b.x;
    const by = b.y;
    const bz = b.z;
    const bw = b.w;
    return setQuaternion(
        out,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz
    );
}

/** Convert a world-space rotation vector (axis * angle, radians) to a quaternion. */
export function quaternionExp(rotation, out = {}) {
    const angleSquared =
        rotation.x * rotation.x +
        rotation.y * rotation.y +
        rotation.z * rotation.z;
    let vectorScale;
    let scalar;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        // The eighth-order series is below double-precision roundoff over
        // |angle| < 0.25 rad.  Avoiding sqrt/sin/cos here preserves the same
        // exponential map while making the many small XPBD frame updates
        // substantially cheaper.
        const angleFourth = angleSquared * angleSquared;
        const angleSixth = angleFourth * angleSquared;
        const angleEighth = angleFourth * angleFourth;
        vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840 -
            angleSixth / 645120 + angleEighth / 185794560;
        scalar = 1 - angleSquared / 8 + angleFourth / 384 -
            angleSixth / 46080 + angleEighth / 10321920;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        vectorScale = Math.sin(halfAngle) / angle;
        scalar = Math.cos(halfAngle);
    }
    return setQuaternion(
        out,
        rotation.x * vectorScale,
        rotation.y * vectorScale,
        rotation.z * vectorScale,
        scalar
    );
}

/**
 * Principal quaternion logarithm as a rotation vector. q and -q intentionally
 * return the same result. Local rod rotations must remain below pi; this is the
 * only unavoidable branch cut of an SO(3) logarithm.
 */
export function quaternionLog(quaternion, out = {}, normalized = false) {
    const norm = normalized ? 1 : Math.sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    if (norm < EPSILON) return setVector(out, 0, 0, 0);
    const sign = quaternion.w < 0 ? -1 : 1;
    const inverseNorm = sign / norm;
    const x = quaternion.x * inverseNorm;
    const y = quaternion.y * inverseNorm;
    const z = quaternion.z * inverseNorm;
    const w = quaternion.w * inverseNorm;
    const vectorNorm = Math.sqrt(x * x + y * y + z * z);
    if (vectorNorm < SMALL_ANGLE) {
        const inverseW = 1 / Math.max(EPSILON, w);
        const scale = 2 * inverseW;
        return setVector(
            out,
            x * scale,
            y * scale,
            z * scale
        );
    }
    const angle = 2 * Math.atan2(vectorNorm, clamp(w, 0, 1));
    const scale = angle / vectorNorm;
    return setVector(
        out,
        x * scale,
        y * scale,
        z * scale
    );
}

export function rotateVectorByQuaternion(quaternion, vector, out = {}) {
    const norm = Math.sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    if (norm < EPSILON) {
        return setVector(out, vector.x, vector.y, vector.z);
    }
    const inverseNorm = 1 / norm;
    const qx = quaternion.x * inverseNorm;
    const qy = quaternion.y * inverseNorm;
    const qz = quaternion.z * inverseNorm;
    const qw = quaternion.w * inverseNorm;
    const tx = 2 * (qy * vector.z - qz * vector.y);
    const ty = 2 * (qz * vector.x - qx * vector.z);
    const tz = 2 * (qx * vector.y - qy * vector.x);
    return setVector(
        out,
        vector.x + qw * tx + qy * tz - qz * ty,
        vector.y + qw * ty + qz * tx - qx * tz,
        vector.z + qw * tz + qx * ty - qy * tx
    );
}

export function inverseRotateVectorByQuaternion(quaternion, vector, out = {}) {
    const norm = Math.sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    if (norm < EPSILON) {
        return setVector(out, vector.x, vector.y, vector.z);
    }
    const inverseNorm = 1 / norm;
    const qx = -quaternion.x * inverseNorm;
    const qy = -quaternion.y * inverseNorm;
    const qz = -quaternion.z * inverseNorm;
    const qw = quaternion.w * inverseNorm;
    const tx = 2 * (qy * vector.z - qz * vector.y);
    const ty = 2 * (qz * vector.x - qx * vector.z);
    const tz = 2 * (qx * vector.y - qy * vector.x);
    return setVector(
        out,
        vector.x + qw * tx + qy * tz - qz * ty,
        vector.y + qw * ty + qz * tx - qx * tz,
        vector.z + qw * tz + qx * ty - qy * tx
    );
}

export function slerpQuaternions(a, b, amount, out = {}) {
    const qa = normalizeQuaternion(a, {});
    const qb = normalizeQuaternion(b, {});
    let bx = qb.x;
    let by = qb.y;
    let bz = qb.z;
    let bw = qb.w;
    let cosine = qa.x * bx + qa.y * by + qa.z * bz + qa.w * bw;
    if (cosine < 0) {
        cosine = -cosine;
        bx = -bx;
        by = -by;
        bz = -bz;
        bw = -bw;
    }
    const t = clamp(amount, 0, 1);
    if (cosine > 0.9995) {
        return normalizeQuaternion({
            x: qa.x + (bx - qa.x) * t,
            y: qa.y + (by - qa.y) * t,
            z: qa.z + (bz - qa.z) * t,
            w: qa.w + (bw - qa.w) * t
        }, out);
    }
    const angle = Math.acos(clamp(cosine, -1, 1));
    const sine = Math.sin(angle);
    const scaleA = Math.sin((1 - t) * angle) / sine;
    const scaleB = Math.sin(t * angle) / sine;
    return normalizeQuaternion({
        x: qa.x * scaleA + bx * scaleB,
        y: qa.y * scaleA + by * scaleB,
        z: qa.z * scaleA + bz * scaleB,
        w: qa.w * scaleA + bw * scaleB
    }, out);
}

function quaternionFromBasis(d1, d2, d3, out = {}) {
    // Basis vectors are columns of the local-to-world rotation matrix.
    const m00 = d1.x;
    const m01 = d2.x;
    const m02 = d3.x;
    const m10 = d1.y;
    const m11 = d2.y;
    const m12 = d3.y;
    const m20 = d1.z;
    const m21 = d2.z;
    const m22 = d3.z;
    const trace = m00 + m11 + m22;
    let x;
    let y;
    let z;
    let w;
    if (trace > 0) {
        const scale = Math.sqrt(trace + 1) * 2;
        w = 0.25 * scale;
        x = (m21 - m12) / scale;
        y = (m02 - m20) / scale;
        z = (m10 - m01) / scale;
    } else if (m00 > m11 && m00 > m22) {
        const scale = Math.sqrt(1 + m00 - m11 - m22) * 2;
        w = (m21 - m12) / scale;
        x = 0.25 * scale;
        y = (m01 + m10) / scale;
        z = (m02 + m20) / scale;
    } else if (m11 > m22) {
        const scale = Math.sqrt(1 + m11 - m00 - m22) * 2;
        w = (m02 - m20) / scale;
        x = (m01 + m10) / scale;
        y = 0.25 * scale;
        z = (m12 + m21) / scale;
    } else {
        const scale = Math.sqrt(1 + m22 - m00 - m11) * 2;
        w = (m10 - m01) / scale;
        x = (m02 + m20) / scale;
        y = (m12 + m21) / scale;
        z = 0.25 * scale;
    }
    setQuaternion(out, x, y, z, w);
    return normalizeQuaternion(out, out);
}

export function materialFrameDirectors(quaternion, out = {}) {
    out.d1 ??= {};
    out.d2 ??= {};
    out.d3 ??= {};
    rotateVectorByQuaternion(quaternion, { x: 1, y: 0, z: 0 }, out.d1);
    rotateVectorByQuaternion(quaternion, { x: 0, y: 1, z: 0 }, out.d2);
    rotateVectorByQuaternion(quaternion, { x: 0, y: 0, z: 1 }, out.d3);
    return out;
}

function leastAlignedAxis(tangent) {
    const ax = Math.abs(tangent.x);
    const ay = Math.abs(tangent.y);
    const az = Math.abs(tangent.z);
    if (ax <= ay && ax <= az) return { x: 1, y: 0, z: 0 };
    if (ay <= az) return { x: 0, y: 1, z: 0 };
    return { x: 0, y: 0, z: 1 };
}

/** Create a deterministic, twist-free material frame for one tangent. */
export function createBishopFrame(tangent, preferredD1 = null, out = {}) {
    const scratch = out._bishopScratch ??= {
        d1: {},
        d2: {},
        d3: {}
    };
    const d3 = normalizeVector(tangent, scratch.d3);
    if (vectorLength(d3) < EPSILON) {
        return setQuaternion(out, 0, 0, 0, 1);
    }
    const preferred = preferredD1 ?? leastAlignedAxis(d3);
    const axial = dotVectors(preferred, d3);
    const d1 = scratch.d1;
    d1.x = preferred.x - d3.x * axial;
    d1.y = preferred.y - d3.y * axial;
    d1.z = preferred.z - d3.z * axial;
    if (vectorLength(d1) < SMALL_ANGLE) {
        const fallback = leastAlignedAxis(d3);
        const fallbackAxial = dotVectors(fallback, d3);
        d1.x = fallback.x - d3.x * fallbackAxial;
        d1.y = fallback.y - d3.y * fallbackAxial;
        d1.z = fallback.z - d3.z * fallbackAxial;
    }
    normalizeVector(d1, d1);
    const d2 = scratch.d2;
    crossVectors(d3, d1, d2);
    normalizeVector(d2, d2);
    return quaternionFromBasis(d1, d2, d3, out);
}

function shortestArcQuaternion(from, to, fallbackAxis, out = {}) {
    const a = normalizeVector(from, {});
    const b = normalizeVector(to, {});
    const cosine = clamp(dotVectors(a, b), -1, 1);
    if (cosine > 1 - 1e-12) return setQuaternion(out, 0, 0, 0, 1);
    if (cosine < -1 + 1e-10) {
        let axis = fallbackAxis ? normalizeVector(fallbackAxis, {}) : null;
        if (!axis || vectorLength(axis) < SMALL_ANGLE || Math.abs(dotVectors(axis, a)) > 1e-5) {
            axis = normalizeVector(crossVectors(a, leastAlignedAxis(a), {}), {});
        }
        return setQuaternion(out, axis.x, axis.y, axis.z, 0);
    }
    const cross = crossVectors(a, b, {});
    return normalizeQuaternion({
        x: cross.x,
        y: cross.y,
        z: cross.z,
        w: 1 + cosine
    }, out);
}

/**
 * Parallel-transport a frame by the shortest rotation between tangents. This
 * changes no material twist. The old d1 is the deterministic antiparallel axis.
 */
export function transportBishopFrame(frame, targetTangent, out = {}) {
    const directors = materialFrameDirectors(frame, {});
    const transport = shortestArcQuaternion(
        directors.d3,
        targetTangent,
        directors.d1,
        {}
    );
    return normalizeQuaternion(multiplyQuaternions(transport, frame, {}), out);
}

/** Align material director d3 to an edge without adding spin around the edge. */
export function adaptMaterialFrameToEdge(frame, edge, out = {}) {
    if (vectorLength(edge) < EPSILON) return normalizeQuaternion(frame, out);
    return transportBishopFrame(frame, edge, out);
}

/** Convert pointwise Kirchhoff strains (1/length) into hinge rotations. */
export function restRotationFromCurvature(
    curvature1,
    curvature2,
    intrinsicTwist,
    voronoiLength,
    out = {}
) {
    const length = Math.max(0, voronoiLength);
    return setVector(
        out,
        curvature1 * length,
        curvature2 * length,
        intrinsicTwist * length
    );
}

function relativeQuaternion(a, b, out = {}, normalized = false) {
    const normA = normalized ? 1 : Math.sqrt(
        a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w
    );
    const normB = normalized ? 1 : Math.sqrt(
        b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w
    );
    if (normA < EPSILON || normB < EPSILON) {
        return setQuaternion(out, 0, 0, 0, 1);
    }
    const inverseNormA = 1 / normA;
    const inverseNormB = 1 / normB;
    const ax = -a.x * inverseNormA;
    const ay = -a.y * inverseNormA;
    const az = -a.z * inverseNormA;
    const aw = a.w * inverseNormA;
    const bx = b.x * inverseNormB;
    const by = b.y * inverseNormB;
    const bz = b.z * inverseNormB;
    const bw = b.w * inverseNormB;
    return setQuaternion(
        out,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz
    );
}

function rotationMatrix(
    quaternion,
    out = new Float64Array(9),
    normalized = false
) {
    const norm = normalized ? 1 : Math.sqrt(
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z +
        quaternion.w * quaternion.w
    );
    const inverseNorm = norm < EPSILON ? 0 : 1 / norm;
    const qx = quaternion.x * inverseNorm;
    const qy = quaternion.y * inverseNorm;
    const qz = quaternion.z * inverseNorm;
    const qw = norm < EPSILON ? 1 : quaternion.w * inverseNorm;
    const xx = qx * qx;
    const yy = qy * qy;
    const zz = qz * qz;
    const xy = qx * qy;
    const xz = qx * qz;
    const yz = qy * qz;
    const wx = qw * qx;
    const wy = qw * qy;
    const wz = qw * qz;
    out[0] = 1 - 2 * (yy + zz);
    out[1] = 2 * (xy - wz);
    out[2] = 2 * (xz + wy);
    out[3] = 2 * (xy + wz);
    out[4] = 1 - 2 * (xx + zz);
    out[5] = 2 * (yz - wx);
    out[6] = 2 * (xz - wy);
    out[7] = 2 * (yz + wx);
    out[8] = 1 - 2 * (xx + yy);
    return out;
}

function transposeMatrix(matrix, out = new Float64Array(9)) {
    out[0] = matrix[0];
    out[1] = matrix[3];
    out[2] = matrix[6];
    out[3] = matrix[1];
    out[4] = matrix[4];
    out[5] = matrix[7];
    out[6] = matrix[2];
    out[7] = matrix[5];
    out[8] = matrix[8];
    return out;
}

function multiplyMatrices(a, b, out = new Float64Array(9)) {
    const a00 = a[0]; const a01 = a[1]; const a02 = a[2];
    const a10 = a[3]; const a11 = a[4]; const a12 = a[5];
    const a20 = a[6]; const a21 = a[7]; const a22 = a[8];
    const b00 = b[0]; const b01 = b[1]; const b02 = b[2];
    const b10 = b[3]; const b11 = b[4]; const b12 = b[5];
    const b20 = b[6]; const b21 = b[7]; const b22 = b[8];
    out[0] = a00 * b00 + a01 * b10 + a02 * b20;
    out[1] = a00 * b01 + a01 * b11 + a02 * b21;
    out[2] = a00 * b02 + a01 * b12 + a02 * b22;
    out[3] = a10 * b00 + a11 * b10 + a12 * b20;
    out[4] = a10 * b01 + a11 * b11 + a12 * b21;
    out[5] = a10 * b02 + a11 * b12 + a12 * b22;
    out[6] = a20 * b00 + a21 * b10 + a22 * b20;
    out[7] = a20 * b01 + a21 * b11 + a22 * b21;
    out[8] = a20 * b02 + a21 * b12 + a22 * b22;
    return out;
}

/** Inverse left Jacobian of SO(3), stable at zero and at the pi branch. */
function inverseLeftJacobian(rotation, out = new Float64Array(9)) {
    const x = rotation.x;
    const y = rotation.y;
    const z = rotation.z;
    const angleSquared = x * x + y * y + z * z;
    let secondOrder;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        const fourth = angleSquared * angleSquared;
        const sixth = fourth * angleSquared;
        const eighth = fourth * fourth;
        secondOrder = 1 / 12 + angleSquared / 720 + fourth / 30240 +
            sixth / 1209600 + eighth / 47900160;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        const cotangentHalf = Math.cos(halfAngle) / Math.max(EPSILON, Math.sin(halfAngle));
        secondOrder = (1 - halfAngle * cotangentHalf) / angleSquared;
    }
    const kSquared00 = -(y * y + z * z);
    const kSquared01 = x * y;
    const kSquared02 = x * z;
    const kSquared10 = x * y;
    const kSquared11 = -(x * x + z * z);
    const kSquared12 = y * z;
    const kSquared20 = x * z;
    const kSquared21 = y * z;
    const kSquared22 = -(x * x + y * y);
    out[0] = 1 + secondOrder * kSquared00;
    out[1] = 0.5 * z + secondOrder * kSquared01;
    out[2] = -0.5 * y + secondOrder * kSquared02;
    out[3] = -0.5 * z + secondOrder * kSquared10;
    out[4] = 1 + secondOrder * kSquared11;
    out[5] = 0.5 * x + secondOrder * kSquared12;
    out[6] = 0.5 * y + secondOrder * kSquared20;
    out[7] = -0.5 * x + secondOrder * kSquared21;
    out[8] = 1 + secondOrder * kSquared22;
    return out;
}

/**
 * Evaluate the local no-shear/inextensibility constraint
 *     C = (x1 - x0) - restLength * d3(q).
 * orientationGradient contains one world angular gradient row per C component.
 */
export function evaluateAdaptationConstraint(
    x0,
    x1,
    orientation,
    restLength,
    out = {}
) {
    out.constraint ??= {};
    out.director ??= {};
    out.orientationGradient ??= new Float64Array(9);
    rotateVectorByQuaternion(
        orientation,
        { x: 0, y: 0, z: 1 },
        out.director
    );
    const length = Math.max(0, restLength);
    setVector(
        out.constraint,
        x1.x - x0.x - length * out.director.x,
        x1.y - x0.y - length * out.director.y,
        x1.z - x0.z - length * out.director.z
    );
    const dx = out.director.x;
    const dy = out.director.y;
    const dz = out.director.z;
    // -L * (d3 cross axis), stored row-major by constraint component.
    out.orientationGradient[0] = 0;
    out.orientationGradient[1] = -length * dz;
    out.orientationGradient[2] = length * dy;
    out.orientationGradient[3] = length * dz;
    out.orientationGradient[4] = 0;
    out.orientationGradient[5] = -length * dx;
    out.orientationGradient[6] = -length * dy;
    out.orientationGradient[7] = length * dx;
    out.orientationGradient[8] = 0;
    out.residual = vectorLength(out.constraint);
    return out;
}

/**
 * Evaluate bend/twist strain Log(Rrest^-1 * R0^-1 * R1). Gradient rows map
 * world-space left angular perturbations of q0/q1 to each strain component.
 */
export function evaluateBendTwistConstraint(
    orientation0,
    orientation1,
    restRotation = { x: 0, y: 0, z: 0 },
    out = {}
) {
    out.strain ??= {};
    out.gradient0 ??= new Float64Array(9);
    out.gradient1 ??= new Float64Array(9);
    const scratch = out._scratch ??= {
        restQuaternion: {},
        relative: {},
        error: {},
        jacobian: new Float64Array(9),
        restMatrix: new Float64Array(9),
        restTranspose: new Float64Array(9),
        orientation0Matrix: new Float64Array(9),
        orientation0Transpose: new Float64Array(9),
        errorCoordinatesFromWorld: new Float64Array(9),
        cachedRestX: NaN,
        cachedRestY: NaN,
        cachedRestZ: NaN
    };
    const restChanged =
        scratch.cachedRestX !== restRotation.x ||
        scratch.cachedRestY !== restRotation.y ||
        scratch.cachedRestZ !== restRotation.z;
    if (restChanged) {
        quaternionExp(restRotation, scratch.restQuaternion);
        transposeMatrix(
            rotationMatrix(scratch.restQuaternion, scratch.restMatrix),
            scratch.restTranspose
        );
        scratch.cachedRestX = restRotation.x;
        scratch.cachedRestY = restRotation.y;
        scratch.cachedRestZ = restRotation.z;
    }
    const restQuaternion = scratch.restQuaternion;
    const relative = relativeQuaternion(
        orientation0,
        orientation1,
        scratch.relative
    );
    const error = relativeQuaternion(
        restQuaternion,
        relative,
        scratch.error
    );
    quaternionLog(error, out.strain);

    const jacobian = inverseLeftJacobian(out.strain, scratch.jacobian);
    const restTranspose = scratch.restTranspose;
    const orientation0Transpose = transposeMatrix(
        rotationMatrix(orientation0, scratch.orientation0Matrix),
        scratch.orientation0Transpose
    );
    const errorCoordinatesFromWorld = multiplyMatrices(
        restTranspose,
        orientation0Transpose,
        scratch.errorCoordinatesFromWorld
    );
    const gradient1 = multiplyMatrices(
        jacobian,
        errorCoordinatesFromWorld,
        out.gradient1
    );
    for (let index = 0; index < 9; index++) {
        out.gradient0[index] = -gradient1[index];
    }
    out.residual = vectorLength(out.strain);
    return out;
}

// Normalized hot-path form used by the block solver. The world-space
// gradient in evaluateBendTwistConstraint is
//   J^-1 Log(error) * R_rest^T * R0^T.
// Keeping the first two factors in q0's material coordinates lets the block
// solver apply the diagonal material inertia directly, without rotating all
// three gradient rows to world space and back for both adjacent frames.
function evaluateBendTwistLocalConstraintNormalized(
    orientation0,
    orientation1,
    restRotation,
    out
) {
    out.strain ??= {};
    out.localGradient ??= new Float64Array(9);
    const scratch = out._scratch ??= {
        restQuaternion: {},
        relative: {},
        restTranspose: new Float64Array(9),
        cachedRestX: NaN,
        cachedRestY: NaN,
        cachedRestZ: NaN
    };
    if (
        scratch.cachedRestX !== restRotation.x ||
        scratch.cachedRestY !== restRotation.y ||
        scratch.cachedRestZ !== restRotation.z
    ) {
        const restX = restRotation.x;
        const restY = restRotation.y;
        const restZ = restRotation.z;
        const angleSquared = restX * restX + restY * restY + restZ * restZ;
        let vectorScale;
        let restW;
        if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
            const angleFourth = angleSquared * angleSquared;
            const angleSixth = angleFourth * angleSquared;
            const angleEighth = angleFourth * angleFourth;
            vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840 -
                angleSixth / 645120 + angleEighth / 185794560;
            restW = 1 - angleSquared / 8 + angleFourth / 384 -
                angleSixth / 46080 + angleEighth / 10321920;
        } else {
            const angle = Math.sqrt(angleSquared);
            const halfAngle = angle * 0.5;
            vectorScale = Math.sin(halfAngle) / angle;
            restW = Math.cos(halfAngle);
        }
        const restXq = restX * vectorScale;
        const restYq = restY * vectorScale;
        const restZq = restZ * vectorScale;
        const restQuaternion = scratch.restQuaternion;
        restQuaternion.x = restXq;
        restQuaternion.y = restYq;
        restQuaternion.z = restZq;
        restQuaternion.w = restW;
        const xx = restXq * restXq;
        const yy = restYq * restYq;
        const zz = restZq * restZq;
        const xy = restXq * restYq;
        const xz = restXq * restZq;
        const yz = restYq * restZq;
        const wx = restW * restXq;
        const wy = restW * restYq;
        const wz = restW * restZq;
        const restTranspose = scratch.restTranspose;
        restTranspose[0] = 1 - 2 * (yy + zz);
        restTranspose[1] = 2 * (xy + wz);
        restTranspose[2] = 2 * (xz - wy);
        restTranspose[3] = 2 * (xy - wz);
        restTranspose[4] = 1 - 2 * (xx + zz);
        restTranspose[5] = 2 * (yz + wx);
        restTranspose[6] = 2 * (xz + wy);
        restTranspose[7] = 2 * (yz - wx);
        restTranspose[8] = 1 - 2 * (xx + yy);
        scratch.cachedRestX = restRotation.x;
        scratch.cachedRestY = restRotation.y;
        scratch.cachedRestZ = restRotation.z;
    }
    const q0x = orientation0.x;
    const q0y = orientation0.y;
    const q0z = orientation0.z;
    const q0w = orientation0.w;
    const q1x = orientation1.x;
    const q1y = orientation1.y;
    const q1z = orientation1.z;
    const q1w = orientation1.w;
    const relativeX = q0w * q1x - q0x * q1w -
        q0y * q1z + q0z * q1y;
    const relativeY = q0w * q1y + q0x * q1z -
        q0y * q1w - q0z * q1x;
    const relativeZ = q0w * q1z - q0x * q1y +
        q0y * q1x - q0z * q1w;
    const relativeW = q0w * q1w + q0x * q1x +
        q0y * q1y + q0z * q1z;
    const relative = scratch.relative;
    relative.x = relativeX;
    relative.y = relativeY;
    relative.z = relativeZ;
    relative.w = relativeW;
    const restQuaternion = scratch.restQuaternion;
    let errorX = restQuaternion.w * relativeX -
        restQuaternion.x * relativeW -
        restQuaternion.y * relativeZ +
        restQuaternion.z * relativeY;
    let errorY = restQuaternion.w * relativeY +
        restQuaternion.x * relativeZ -
        restQuaternion.y * relativeW -
        restQuaternion.z * relativeX;
    let errorZ = restQuaternion.w * relativeZ -
        restQuaternion.x * relativeY +
        restQuaternion.y * relativeX -
        restQuaternion.z * relativeW;
    let errorW = restQuaternion.w * relativeW +
        restQuaternion.x * relativeX +
        restQuaternion.y * relativeY +
        restQuaternion.z * relativeZ;
    if (errorW < 0) {
        errorX = -errorX;
        errorY = -errorY;
        errorZ = -errorZ;
        errorW = -errorW;
    }
    const vectorNorm = Math.sqrt(
        errorX * errorX + errorY * errorY + errorZ * errorZ
    );
    let strainX;
    let strainY;
    let strainZ;
    if (vectorNorm < SMALL_ANGLE) {
        const scale = 2 / Math.max(EPSILON, errorW);
        strainX = errorX * scale;
        strainY = errorY * scale;
        strainZ = errorZ * scale;
    } else {
        const angle = 2 * Math.atan2(vectorNorm, clamp(errorW, 0, 1));
        const scale = angle / vectorNorm;
        strainX = errorX * scale;
        strainY = errorY * scale;
        strainZ = errorZ * scale;
    }
    out.strain.x = strainX;
    out.strain.y = strainY;
    out.strain.z = strainZ;
    const angleSquared = strainX * strainX +
        strainY * strainY + strainZ * strainZ;
    let secondOrder;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        const fourth = angleSquared * angleSquared;
        const sixth = fourth * angleSquared;
        const eighth = fourth * fourth;
        secondOrder = 1 / 12 + angleSquared / 720 + fourth / 30240 +
            sixth / 1209600 + eighth / 47900160;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        const cotangentHalf = Math.cos(halfAngle) /
            Math.max(EPSILON, Math.sin(halfAngle));
        secondOrder = (1 - halfAngle * cotangentHalf) / angleSquared;
    }
    const j00 = 1 - secondOrder *
        (strainY * strainY + strainZ * strainZ);
    const j01 = 0.5 * strainZ + secondOrder * strainX * strainY;
    const j02 = -0.5 * strainY + secondOrder * strainX * strainZ;
    const j10 = -0.5 * strainZ + secondOrder * strainX * strainY;
    const j11 = 1 - secondOrder *
        (strainX * strainX + strainZ * strainZ);
    const j12 = 0.5 * strainX + secondOrder * strainY * strainZ;
    const j20 = 0.5 * strainY + secondOrder * strainX * strainZ;
    const j21 = -0.5 * strainX + secondOrder * strainY * strainZ;
    const j22 = 1 - secondOrder *
        (strainX * strainX + strainY * strainY);
    const r = scratch.restTranspose;
    const gradient = out.localGradient;
    gradient[0] = j00 * r[0] + j01 * r[3] + j02 * r[6];
    gradient[1] = j00 * r[1] + j01 * r[4] + j02 * r[7];
    gradient[2] = j00 * r[2] + j01 * r[5] + j02 * r[8];
    gradient[3] = j10 * r[0] + j11 * r[3] + j12 * r[6];
    gradient[4] = j10 * r[1] + j11 * r[4] + j12 * r[7];
    gradient[5] = j10 * r[2] + j11 * r[5] + j12 * r[8];
    gradient[6] = j20 * r[0] + j21 * r[3] + j22 * r[6];
    gradient[7] = j20 * r[1] + j21 * r[4] + j22 * r[7];
    gradient[8] = j20 * r[2] + j21 * r[5] + j22 * r[8];
    return out;
}

export function evaluateKirchhoffHingeEnergy(
    orientation0,
    orientation1,
    restRotation,
    {
        bendStiffness1 = 1,
        bendStiffness2 = bendStiffness1,
        twistStiffness = bendStiffness1,
        voronoiLength = 1
    } = {},
    out = {}
) {
    const state = evaluateBendTwistConstraint(
        orientation0,
        orientation1,
        restRotation,
        out
    );
    const inverseLength = 1 / Math.max(EPSILON, voronoiLength);
    state.bendEnergy = 0.5 * inverseLength * (
        Math.max(0, bendStiffness1) * state.strain.x * state.strain.x +
        Math.max(0, bendStiffness2) * state.strain.y * state.strain.y
    );
    state.twistEnergy = 0.5 * inverseLength *
        Math.max(0, twistStiffness) * state.strain.z * state.strain.z;
    state.totalEnergy = state.bendEnergy + state.twistEnergy;
    return state;
}

function componentValue(value, component, fallback = 0) {
    if (Number.isFinite(value)) return value;
    if (Array.isArray(value) || ArrayBuffer.isView(value)) {
        return Number.isFinite(value[component]) ? value[component] : fallback;
    }
    const key = component === 0 ? 'x' : component === 1 ? 'y' : 'z';
    return Number.isFinite(value?.[key]) ? value[key] : fallback;
}

function setComponent(value, component, next) {
    if (Array.isArray(value) || ArrayBuffer.isView(value)) {
        value[component] = next;
        return;
    }
    const key = component === 0 ? 'x' : component === 1 ? 'y' : 'z';
    value[key] = next;
}

function vectorComponent(vector, component) {
    return component === 0 ? vector.x : component === 1 ? vector.y : vector.z;
}

function applyWorldInverseInertiaComponents(
    orientation,
    inverseX,
    inverseY,
    inverseZ,
    gradient,
    out,
    normalized = false
) {
    if (inverseX === inverseY && inverseY === inverseZ) {
        return setVector(
            out,
            gradient.x * inverseX,
            gradient.y * inverseY,
            gradient.z * inverseZ
        );
    }
    const norm = normalized ? 1 : Math.sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w
    );
    if (norm < EPSILON) return setVector(out, 0, 0, 0);
    const inverseNorm = 1 / norm;
    const qx = orientation.x * inverseNorm;
    const qy = orientation.y * inverseNorm;
    const qz = orientation.z * inverseNorm;
    const qw = orientation.w * inverseNorm;

    // local = R(q)^T * gradient. Keep the two rotations in one scalar hot
    // path so the quaternion is normalized only once per inertia product.
    let tx = 2 * (-qy * gradient.z + qz * gradient.y);
    let ty = 2 * (-qz * gradient.x + qx * gradient.z);
    let tz = 2 * (-qx * gradient.y + qy * gradient.x);
    const localX = (
        gradient.x + qw * tx - qy * tz + qz * ty
    ) * inverseX;
    const localY = (
        gradient.y + qw * ty - qz * tx + qx * tz
    ) * inverseY;
    const localZ = (
        gradient.z + qw * tz - qx * ty + qy * tx
    ) * inverseZ;

    // out = R(q) * local.
    tx = 2 * (qy * localZ - qz * localY);
    ty = 2 * (qz * localX - qx * localZ);
    tz = 2 * (qx * localY - qy * localX);
    return setVector(
        out,
        localX + qw * tx + qy * tz - qz * ty,
        localY + qw * ty + qz * tx - qx * tz,
        localZ + qw * tz + qx * ty - qy * tx
    );
}

function applyAngularCorrection(orientation, correction) {
    const angleSquared = correction.x * correction.x +
        correction.y * correction.y + correction.z * correction.z;
    if (angleSquared < EPSILON * EPSILON) return orientation;
    let vectorScale;
    let scalar;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        const angleFourth = angleSquared * angleSquared;
        const angleSixth = angleFourth * angleSquared;
        const angleEighth = angleFourth * angleFourth;
        vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840 -
            angleSixth / 645120 + angleEighth / 185794560;
        scalar = 1 - angleSquared / 8 + angleFourth / 384 -
            angleSixth / 46080 + angleEighth / 10321920;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        vectorScale = Math.sin(halfAngle) / angle;
        scalar = Math.cos(halfAngle);
    }
    const ix = correction.x * vectorScale;
    const iy = correction.y * vectorScale;
    const iz = correction.z * vectorScale;
    const iw = scalar;
    const ox = orientation.x;
    const oy = orientation.y;
    const oz = orientation.z;
    const ow = orientation.w;
    const x = iw * ox + ix * ow + iy * oz - iz * oy;
    const y = iw * oy - ix * oz + iy * ow + iz * ox;
    const z = iw * oz + ix * oy - iy * ox + iz * ow;
    const w = iw * ow - ix * ox - iy * oy - iz * oz;
    const norm = Math.sqrt(x * x + y * y + z * z + w * w);
    if (norm < EPSILON) return setQuaternion(orientation, 0, 0, 0, 1);
    const inverseNorm = 1 / norm;
    return setQuaternion(
        orientation,
        x * inverseNorm,
        y * inverseNorm,
        z * inverseNorm,
        w * inverseNorm
    );
}

function evaluateAdaptationComponent(
    x0,
    x1,
    orientation,
    restLength,
    component,
    gradient,
    normalized = false
) {
    const norm = normalized ? 1 : Math.sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w
    );
    const inverseNorm = norm < EPSILON ? 0 : 1 / norm;
    const qx = orientation.x * inverseNorm;
    const qy = orientation.y * inverseNorm;
    const qz = orientation.z * inverseNorm;
    const qw = norm < EPSILON ? 1 : orientation.w * inverseNorm;
    const d3x = 2 * (qx * qz + qw * qy);
    const d3y = 2 * (qy * qz - qw * qx);
    const d3z = 1 - 2 * (qx * qx + qy * qy);
    const length = Math.max(0, restLength);
    if (component === 0) {
        setVector(gradient, 0, -length * d3z, length * d3y);
        return x1.x - x0.x - length * d3x;
    }
    if (component === 1) {
        setVector(gradient, length * d3z, 0, -length * d3x);
        return x1.y - x0.y - length * d3y;
    }
    setVector(gradient, -length * d3y, length * d3x, 0);
    return x1.z - x0.z - length * d3z;
}

function evaluateBendTwistComponent(
    orientation0,
    orientation1,
    restRotation,
    component,
    scratch,
    gradient1,
    normalized = false
) {
    scratch.restQuaternion ??= {};
    scratch.relative ??= {};
    scratch.error ??= {};
    scratch.strain ??= {};
    scratch.jacobian ??= new Float64Array(9);
    scratch.restMatrix ??= new Float64Array(9);
    scratch.restTranspose ??= new Float64Array(9);
    scratch.orientation0Matrix ??= new Float64Array(9);
    scratch.orientation0Transpose ??= new Float64Array(9);
    if (
        scratch.cachedRestX !== restRotation.x ||
        scratch.cachedRestY !== restRotation.y ||
        scratch.cachedRestZ !== restRotation.z
    ) {
        quaternionExp(restRotation, scratch.restQuaternion);
        transposeMatrix(
            rotationMatrix(
                scratch.restQuaternion,
                scratch.restMatrix,
                true
            ),
            scratch.restTranspose
        );
        scratch.cachedRestX = restRotation.x;
        scratch.cachedRestY = restRotation.y;
        scratch.cachedRestZ = restRotation.z;
    }
    relativeQuaternion(
        orientation0,
        orientation1,
        scratch.relative,
        normalized
    );
    relativeQuaternion(
        scratch.restQuaternion,
        scratch.relative,
        scratch.error,
        normalized
    );
    quaternionLog(scratch.error, scratch.strain, normalized);
    inverseLeftJacobian(scratch.strain, scratch.jacobian);
    transposeMatrix(
        rotationMatrix(
            orientation0,
            scratch.orientation0Matrix,
            normalized
        ),
        scratch.orientation0Transpose
    );

    const row = component * 3;
    const j0 = scratch.jacobian[row];
    const j1 = scratch.jacobian[row + 1];
    const j2 = scratch.jacobian[row + 2];
    const rest = scratch.restTranspose;
    const a0 = j0 * rest[0] + j1 * rest[3] + j2 * rest[6];
    const a1 = j0 * rest[1] + j1 * rest[4] + j2 * rest[7];
    const a2 = j0 * rest[2] + j1 * rest[5] + j2 * rest[8];
    const world = scratch.orientation0Transpose;
    setVector(
        gradient1,
        a0 * world[0] + a1 * world[3] + a2 * world[6],
        a0 * world[1] + a1 * world[4] + a2 * world[7],
        a0 * world[2] + a1 * world[5] + a2 * world[8]
    );
    return vectorComponent(scratch.strain, component);
}

function applyWorldInverseInertiaNormalizedScalar(
    qx,
    qy,
    qz,
    qw,
    inverseX,
    inverseY,
    inverseZ,
    gradientX,
    gradientY,
    gradientZ,
    out
) {
    if (inverseX === inverseY && inverseY === inverseZ) {
        out[0] = gradientX * inverseX;
        out[1] = gradientY * inverseY;
        out[2] = gradientZ * inverseZ;
        return;
    }
    let tx = 2 * (-qy * gradientZ + qz * gradientY);
    let ty = 2 * (-qz * gradientX + qx * gradientZ);
    let tz = 2 * (-qx * gradientY + qy * gradientX);
    const localX = (
        gradientX + qw * tx - qy * tz + qz * ty
    ) * inverseX;
    const localY = (
        gradientY + qw * ty - qz * tx + qx * tz
    ) * inverseY;
    const localZ = (
        gradientZ + qw * tz - qx * ty + qy * tx
    ) * inverseZ;
    tx = 2 * (qy * localZ - qz * localY);
    ty = 2 * (qz * localX - qx * localZ);
    tz = 2 * (qx * localY - qy * localX);
    out[0] = localX + qw * tx + qy * tz - qz * ty;
    out[1] = localY + qw * ty + qz * tx - qx * tz;
    out[2] = localZ + qw * tz + qx * ty - qy * tx;
}

function applyWorldInverseInertiaNormalizedScalarAt(
    qx,
    qy,
    qz,
    qw,
    inverseX,
    inverseY,
    inverseZ,
    gradientX,
    gradientY,
    gradientZ,
    out,
    outOffset
) {
    if (inverseX === inverseY && inverseY === inverseZ) {
        out[outOffset] = gradientX * inverseX;
        out[outOffset + 1] = gradientY * inverseY;
        out[outOffset + 2] = gradientZ * inverseZ;
        return;
    }
    let tx = 2 * (-qy * gradientZ + qz * gradientY);
    let ty = 2 * (-qz * gradientX + qx * gradientZ);
    let tz = 2 * (-qx * gradientY + qy * gradientX);
    const localX = (
        gradientX + qw * tx - qy * tz + qz * ty
    ) * inverseX;
    const localY = (
        gradientY + qw * ty - qz * tx + qx * tz
    ) * inverseY;
    const localZ = (
        gradientZ + qw * tz - qx * ty + qy * tx
    ) * inverseZ;
    tx = 2 * (qy * localZ - qz * localY);
    ty = 2 * (qz * localX - qx * localZ);
    tz = 2 * (qx * localY - qy * localX);
    out[outOffset] = localX + qw * tx + qy * tz - qz * ty;
    out[outOffset + 1] = localY + qw * ty + qz * tx - qx * tz;
    out[outOffset + 2] = localZ + qw * tz + qx * ty - qy * tx;
}

function applyAngularCorrectionNormalizedScalar(
    qx,
    qy,
    qz,
    qw,
    correctionX,
    correctionY,
    correctionZ,
    out
) {
    const angleSquared = correctionX * correctionX +
        correctionY * correctionY + correctionZ * correctionZ;
    if (angleSquared < EPSILON * EPSILON) {
        out[0] = qx;
        out[1] = qy;
        out[2] = qz;
        out[3] = qw;
        return;
    }
    let vectorScale;
    let scalar;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        const angleFourth = angleSquared * angleSquared;
        const angleSixth = angleFourth * angleSquared;
        const angleEighth = angleFourth * angleFourth;
        vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840 -
            angleSixth / 645120 + angleEighth / 185794560;
        scalar = 1 - angleSquared / 8 + angleFourth / 384 -
            angleSixth / 46080 + angleEighth / 10321920;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        vectorScale = Math.sin(halfAngle) / angle;
        scalar = Math.cos(halfAngle);
    }
    const ix = correctionX * vectorScale;
    const iy = correctionY * vectorScale;
    const iz = correctionZ * vectorScale;
    const x = scalar * qx + ix * qw + iy * qz - iz * qy;
    const y = scalar * qy - ix * qz + iy * qw + iz * qx;
    const z = scalar * qz + ix * qy - iy * qx + iz * qw;
    const w = scalar * qw - ix * qx - iy * qy - iz * qz;
    const norm = Math.sqrt(x * x + y * y + z * z + w * w);
    if (norm < EPSILON) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        out[3] = 1;
        return;
    }
    const inverseNorm = 1 / norm;
    out[0] = x * inverseNorm;
    out[1] = y * inverseNorm;
    out[2] = z * inverseNorm;
    out[3] = w * inverseNorm;
}

// q rotates material coordinates to world coordinates. Therefore applying a
// local angular increment on the right is exactly equivalent to rotating that
// increment to world space and left-multiplying it, with fewer operations.
function applyLocalAngularCorrectionNormalizedScalar(
    qx,
    qy,
    qz,
    qw,
    correctionX,
    correctionY,
    correctionZ,
    out
) {
    const angleSquared = correctionX * correctionX +
        correctionY * correctionY + correctionZ * correctionZ;
    if (angleSquared < EPSILON * EPSILON) {
        out[0] = qx;
        out[1] = qy;
        out[2] = qz;
        out[3] = qw;
        return;
    }
    let vectorScale;
    let scalar;
    if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
        const angleFourth = angleSquared * angleSquared;
        const angleSixth = angleFourth * angleSquared;
        const angleEighth = angleFourth * angleFourth;
        vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840 -
            angleSixth / 645120 + angleEighth / 185794560;
        scalar = 1 - angleSquared / 8 + angleFourth / 384 -
            angleSixth / 46080 + angleEighth / 10321920;
    } else {
        const angle = Math.sqrt(angleSquared);
        const halfAngle = angle * 0.5;
        vectorScale = Math.sin(halfAngle) / angle;
        scalar = Math.cos(halfAngle);
    }
    const ix = correctionX * vectorScale;
    const iy = correctionY * vectorScale;
    const iz = correctionZ * vectorScale;
    const x = qw * ix + qx * scalar + qy * iz - qz * iy;
    const y = qw * iy - qx * iz + qy * scalar + qz * ix;
    const z = qw * iz + qx * iy - qy * ix + qz * scalar;
    const w = qw * scalar - qx * ix - qy * iy - qz * iz;
    const inverseNorm = 1 / Math.max(
        EPSILON,
        Math.sqrt(x * x + y * y + z * z + w * w)
    );
    out[0] = x * inverseNorm;
    out[1] = y * inverseNorm;
    out[2] = z * inverseNorm;
    out[3] = w * inverseNorm;
}

function solvePositiveSymmetric3x3(
    a00,
    a01,
    a02,
    a11,
    a12,
    a22,
    b0,
    b1,
    b2,
    out
) {
    if (a00 <= EPSILON) return false;
    const l00 = Math.sqrt(a00);
    const l10 = a01 / l00;
    const l20 = a02 / l00;
    const diagonal1 = a11 - l10 * l10;
    if (diagonal1 <= EPSILON) return false;
    const l11 = Math.sqrt(diagonal1);
    const l21 = (a12 - l20 * l10) / l11;
    const diagonal2 = a22 - l20 * l20 - l21 * l21;
    if (diagonal2 <= EPSILON) return false;
    const l22 = Math.sqrt(diagonal2);
    const y0 = b0 / l00;
    const y1 = (b1 - l10 * y0) / l11;
    const y2 = (b2 - l20 * y0 - l21 * y1) / l22;
    out[2] = y2 / l22;
    out[1] = (y1 - l21 * out[2]) / l11;
    out[0] = (y0 - l10 * out[1] - l20 * out[2]) / l00;
    return true;
}

function fillLocalBendTwistBlock(
    gradient0,
    gradient1,
    inertia00,
    inertia01,
    inertia02,
    inertia10,
    inertia11,
    inertia12,
    alpha0,
    alpha1,
    alpha2,
    out
) {
    const a00 = gradient0[0]; const a01 = gradient0[1]; const a02 = gradient0[2];
    const a10 = gradient0[3]; const a11 = gradient0[4]; const a12 = gradient0[5];
    const a20 = gradient0[6]; const a21 = gradient0[7]; const a22 = gradient0[8];
    const b00 = gradient1[0]; const b01 = gradient1[1]; const b02 = gradient1[2];
    const b10 = gradient1[3]; const b11 = gradient1[4]; const b12 = gradient1[5];
    const b20 = gradient1[6]; const b21 = gradient1[7]; const b22 = gradient1[8];
    out[0] = a00 * inertia00 * a00 + a01 * inertia01 * a01 +
        a02 * inertia02 * a02 + b00 * inertia10 * b00 +
        b01 * inertia11 * b01 + b02 * inertia12 * b02 + alpha0;
    out[1] = a00 * inertia00 * a10 + a01 * inertia01 * a11 +
        a02 * inertia02 * a12 + b00 * inertia10 * b10 +
        b01 * inertia11 * b11 + b02 * inertia12 * b12;
    out[2] = a00 * inertia00 * a20 + a01 * inertia01 * a21 +
        a02 * inertia02 * a22 + b00 * inertia10 * b20 +
        b01 * inertia11 * b21 + b02 * inertia12 * b22;
    out[3] = a10 * inertia00 * a10 + a11 * inertia01 * a11 +
        a12 * inertia02 * a12 + b10 * inertia10 * b10 +
        b11 * inertia11 * b11 + b12 * inertia12 * b12 + alpha1;
    out[4] = a10 * inertia00 * a20 + a11 * inertia01 * a21 +
        a12 * inertia02 * a22 + b10 * inertia10 * b20 +
        b11 * inertia11 * b21 + b12 * inertia12 * b22;
    out[5] = a20 * inertia00 * a20 + a21 * inertia01 * a21 +
        a22 * inertia02 * a22 + b20 * inertia10 * b20 +
        b21 * inertia11 * b21 + b22 * inertia12 * b22 + alpha2;
}

/**
 * Coordinate-invariant block solve of the same three-component adaptation
 * constraint used by solveAdaptationXPBDArraySweep. Solving the 3-vector in
 * one SPD block evaluates d3 and applies one SO(3) update per edge instead of
 * repeating them for three coordinate rows. The constraint, compliance and
 * generalized masses are unchanged.
 */
export function solveAdaptationXPBDBlockArraySweep(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    const x = body.x;
    const y = body.y;
    const z = body.z;
    const orientationX = body.orientationX;
    const orientationY = body.orientationY;
    const orientationZ = body.orientationZ;
    const orientationW = body.orientationW;
    const inverseMass = body.inverseMass;
    const inverseInertia1 = body.inverseInertia1;
    const inverseInertia2 = body.inverseInertia2;
    const inverseInertia3 = body.inverseInertia3;
    const restLength = body.restLength;
    const lambdaX = body.adaptationLambdaX;
    const lambdaY = body.adaptationLambdaY;
    const lambdaZ = body.adaptationLambdaZ;
    const alpha = Math.max(0, body.adaptationCompliance) /
        Math.max(EPSILON, dt * dt);
    const scratch = body.kirchhoffScratch.adaptationBlock ??= {
        weighted: new Float64Array(9),
        delta: new Float64Array(3),
        quaternion: new Float64Array(4)
    };
    const weighted = scratch.weighted;
    const delta = scratch.delta;
    const corrected = scratch.quaternion;
    for (
        let segment = reverse ? end - 1 : start;
        reverse ? segment >= start : segment < end;
        segment += reverse ? -1 : 1
    ) {
        const next = segment + 1;
        const qx = orientationX[segment];
        const qy = orientationY[segment];
        const qz = orientationZ[segment];
        const qw = orientationW[segment];
        const d3x = 2 * (qx * qz + qw * qy);
        const d3y = 2 * (qy * qz - qw * qx);
        const d3z = 1 - 2 * (qx * qx + qy * qy);
        const length = Math.max(0, restLength[segment]);
        const c0 = x[next] - x[segment] - length * d3x;
        const c1 = y[next] - y[segment] - length * d3y;
        const c2 = z[next] - z[segment] - length * d3z;
        const b0 = -c0 - alpha * lambdaX[segment];
        const b1 = -c1 - alpha * lambdaY[segment];
        const b2 = -c2 - alpha * lambdaZ[segment];
        if (b0 === 0 && b1 === 0 && b2 === 0) continue;

        const g00 = 0;
        const g01 = -length * d3z;
        const g02 = length * d3y;
        const g10 = length * d3z;
        const g11 = 0;
        const g12 = -length * d3x;
        const g20 = -length * d3y;
        const g21 = length * d3x;
        const g22 = 0;
        const inertia1 = inverseInertia1[segment];
        const inertia2 = inverseInertia2[segment];
        const inertia3 = inverseInertia3[segment];
        if (inertia1 === inertia2) {
            weighted[0] = g00 * inertia1;
            weighted[1] = g01 * inertia1;
            weighted[2] = g02 * inertia1;
            weighted[3] = g10 * inertia1;
            weighted[4] = g11 * inertia1;
            weighted[5] = g12 * inertia1;
            weighted[6] = g20 * inertia1;
            weighted[7] = g21 * inertia1;
            weighted[8] = g22 * inertia1;
        } else {
            applyWorldInverseInertiaNormalizedScalarAt(
                qx, qy, qz, qw,
                inertia1, inertia2, inertia3,
                g00, g01, g02,
                weighted,
                0
            );
            applyWorldInverseInertiaNormalizedScalarAt(
                qx, qy, qz, qw,
                inertia1, inertia2, inertia3,
                g10, g11, g12,
                weighted,
                3
            );
            applyWorldInverseInertiaNormalizedScalarAt(
                qx, qy, qz, qw,
                inertia1, inertia2, inertia3,
                g20, g21, g22,
                weighted,
                6
            );
        }
        const translationalWeight =
            inverseMass[segment] + inverseMass[next];
        const k00 = translationalWeight + alpha +
            g00 * weighted[0] + g01 * weighted[1] + g02 * weighted[2];
        const k01 =
            g00 * weighted[3] + g01 * weighted[4] + g02 * weighted[5];
        const k02 =
            g00 * weighted[6] + g01 * weighted[7] + g02 * weighted[8];
        const k11 = translationalWeight + alpha +
            g10 * weighted[3] + g11 * weighted[4] + g12 * weighted[5];
        const k12 =
            g10 * weighted[6] + g11 * weighted[7] + g12 * weighted[8];
        const k22 = translationalWeight + alpha +
            g20 * weighted[6] + g21 * weighted[7] + g22 * weighted[8];
        if (!solvePositiveSymmetric3x3(
            k00, k01, k02, k11, k12, k22,
            b0, b1, b2,
            delta
        )) continue;
        const delta0 = delta[0];
        const delta1 = delta[1];
        const delta2 = delta[2];
        const mass0 = inverseMass[segment];
        const mass1 = inverseMass[next];
        x[segment] -= mass0 * delta0;
        y[segment] -= mass0 * delta1;
        z[segment] -= mass0 * delta2;
        x[next] += mass1 * delta0;
        y[next] += mass1 * delta1;
        z[next] += mass1 * delta2;
        applyAngularCorrectionNormalizedScalar(
            qx,
            qy,
            qz,
            qw,
            weighted[0] * delta0 + weighted[3] * delta1 +
                weighted[6] * delta2,
            weighted[1] * delta0 + weighted[4] * delta1 +
                weighted[7] * delta2,
            weighted[2] * delta0 + weighted[5] * delta1 +
                weighted[8] * delta2,
            corrected
        );
        orientationX[segment] = corrected[0];
        orientationY[segment] = corrected[1];
        orientationZ[segment] = corrected[2];
        orientationW[segment] = corrected[3];
        lambdaX[segment] += delta0;
        lambdaY[segment] += delta1;
        lambdaZ[segment] += delta2;
    }
}

/**
 * Allocation-free array sweep equivalent to solveAdaptationXPBD with
 * normalizedOrientations/objectVectors enabled. Components and segments keep
 * exactly the same Gauss-Seidel order; only object marshalling is removed.
 */
export function solveAdaptationXPBDArraySweep(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    const x = body.x;
    const y = body.y;
    const z = body.z;
    const orientationX = body.orientationX;
    const orientationY = body.orientationY;
    const orientationZ = body.orientationZ;
    const orientationW = body.orientationW;
    const inverseMass = body.inverseMass;
    const inverseInertia1 = body.inverseInertia1;
    const inverseInertia2 = body.inverseInertia2;
    const inverseInertia3 = body.inverseInertia3;
    const restLength = body.restLength;
    const lambdaX = body.adaptationLambdaX;
    const lambdaY = body.adaptationLambdaY;
    const lambdaZ = body.adaptationLambdaZ;
    const alpha = Math.max(0, body.adaptationCompliance) /
        Math.max(EPSILON, dt * dt);
    const scalarScratch = body.kirchhoffScratch.arraySweep ??= {
        weighted: new Float64Array(3),
        quaternion: new Float64Array(4)
    };
    const weighted = scalarScratch.weighted;
    for (
        let segment = reverse ? end - 1 : start;
        reverse ? segment >= start : segment < end;
        segment += reverse ? -1 : 1
    ) {
        const next = segment + 1;
        let x0 = x[segment];
        let y0 = y[segment];
        let z0 = z[segment];
        let x1 = x[next];
        let y1 = y[next];
        let z1 = z[next];
        let qx = orientationX[segment];
        let qy = orientationY[segment];
        let qz = orientationZ[segment];
        let qw = orientationW[segment];
        let lx = lambdaX[segment];
        let ly = lambdaY[segment];
        let lz = lambdaZ[segment];
        const length = Math.max(0, restLength[segment]);
        const inverseMass0 = inverseMass[segment];
        const inverseMass1Value = inverseMass[next];
        const inertia1 = inverseInertia1[segment];
        const inertia2 = inverseInertia2[segment];
        const inertia3 = inverseInertia3[segment];
        const circularBending = inertia1 === inertia2;
        // Keep the exact scalar Gauss-Seidel order X -> Y -> Z, but spell out
        // the three rows so the browser JIT does not execute a component
        // branch and generic lambda dispatch in this hottest loop.
        {
            const d3x = 2 * (qx * qz + qw * qy);
            const d3y = 2 * (qy * qz - qw * qx);
            const d3z = 1 - 2 * (qx * qx + qy * qy);
            const gradientX = 0;
            const gradientY = -length * d3z;
            const gradientZ = length * d3y;
            const numerator = -(x1 - x0 - length * d3x) - alpha * lx;
            if (numerator !== 0) {
                let weightedX;
                let weightedY;
                let weightedZ;
                if (circularBending) {
                    weightedX = gradientX * inertia1;
                    weightedY = gradientY * inertia1;
                    weightedZ = gradientZ * inertia1;
                } else {
                    applyWorldInverseInertiaNormalizedScalar(
                        qx, qy, qz, qw,
                        inertia1, inertia2, inertia3,
                        gradientX, gradientY, gradientZ,
                        weighted
                    );
                    weightedX = weighted[0];
                    weightedY = weighted[1];
                    weightedZ = weighted[2];
                }
                const denominator = inverseMass0 + inverseMass1Value +
                    gradientX * weightedX +
                    gradientY * weightedY +
                    gradientZ * weightedZ + alpha;
                if (denominator >= EPSILON) {
                    const deltaLambda = numerator / denominator;
                    lx += deltaLambda;
                    x0 -= inverseMass0 * deltaLambda;
                    x1 += inverseMass1Value * deltaLambda;
                    const correctionX = weightedX * deltaLambda;
                    const correctionY = weightedY * deltaLambda;
                    const correctionZ = weightedZ * deltaLambda;
                    const angleSquared =
                        correctionX * correctionX +
                        correctionY * correctionY +
                        correctionZ * correctionZ;
                    if (angleSquared >= EPSILON * EPSILON) {
                        let vectorScale;
                        let scalar;
                        if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
                            const angleFourth = angleSquared * angleSquared;
                            const angleSixth = angleFourth * angleSquared;
                            const angleEighth = angleFourth * angleFourth;
                            vectorScale =
                                0.5 - angleSquared / 48 +
                                angleFourth / 3840 -
                                angleSixth / 645120 +
                                angleEighth / 185794560;
                            scalar =
                                1 - angleSquared / 8 +
                                angleFourth / 384 -
                                angleSixth / 46080 +
                                angleEighth / 10321920;
                        } else {
                            const angle = Math.sqrt(angleSquared);
                            const halfAngle = angle * 0.5;
                            vectorScale = Math.sin(halfAngle) / angle;
                            scalar = Math.cos(halfAngle);
                        }
                        const ix = correctionX * vectorScale;
                        const iy = correctionY * vectorScale;
                        const iz = correctionZ * vectorScale;
                        const nextQx =
                            scalar * qx + ix * qw + iy * qz - iz * qy;
                        const nextQy =
                            scalar * qy - ix * qz + iy * qw + iz * qx;
                        const nextQz =
                            scalar * qz + ix * qy - iy * qx + iz * qw;
                        const nextQw =
                            scalar * qw - ix * qx - iy * qy - iz * qz;
                        const norm = Math.sqrt(
                            nextQx * nextQx +
                            nextQy * nextQy +
                            nextQz * nextQz +
                            nextQw * nextQw
                        );
                        if (norm < EPSILON) {
                            qx = 0;
                            qy = 0;
                            qz = 0;
                            qw = 1;
                        } else {
                            const inverseNorm = 1 / norm;
                            qx = nextQx * inverseNorm;
                            qy = nextQy * inverseNorm;
                            qz = nextQz * inverseNorm;
                            qw = nextQw * inverseNorm;
                        }
                    }
                }
            }
        }
        {
            const d3x = 2 * (qx * qz + qw * qy);
            const d3y = 2 * (qy * qz - qw * qx);
            const d3z = 1 - 2 * (qx * qx + qy * qy);
            const gradientX = length * d3z;
            const gradientY = 0;
            const gradientZ = -length * d3x;
            const numerator = -(y1 - y0 - length * d3y) - alpha * ly;
            if (numerator !== 0) {
                let weightedX;
                let weightedY;
                let weightedZ;
                if (circularBending) {
                    weightedX = gradientX * inertia1;
                    weightedY = gradientY * inertia1;
                    weightedZ = gradientZ * inertia1;
                } else {
                    applyWorldInverseInertiaNormalizedScalar(
                        qx, qy, qz, qw,
                        inertia1, inertia2, inertia3,
                        gradientX, gradientY, gradientZ,
                        weighted
                    );
                    weightedX = weighted[0];
                    weightedY = weighted[1];
                    weightedZ = weighted[2];
                }
                const denominator = inverseMass0 + inverseMass1Value +
                    gradientX * weightedX +
                    gradientY * weightedY +
                    gradientZ * weightedZ + alpha;
                if (denominator >= EPSILON) {
                    const deltaLambda = numerator / denominator;
                    ly += deltaLambda;
                    y0 -= inverseMass0 * deltaLambda;
                    y1 += inverseMass1Value * deltaLambda;
                    const correctionX = weightedX * deltaLambda;
                    const correctionY = weightedY * deltaLambda;
                    const correctionZ = weightedZ * deltaLambda;
                    const angleSquared =
                        correctionX * correctionX +
                        correctionY * correctionY +
                        correctionZ * correctionZ;
                    if (angleSquared >= EPSILON * EPSILON) {
                        let vectorScale;
                        let scalar;
                        if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
                            const angleFourth = angleSquared * angleSquared;
                            const angleSixth = angleFourth * angleSquared;
                            const angleEighth = angleFourth * angleFourth;
                            vectorScale =
                                0.5 - angleSquared / 48 +
                                angleFourth / 3840 -
                                angleSixth / 645120 +
                                angleEighth / 185794560;
                            scalar =
                                1 - angleSquared / 8 +
                                angleFourth / 384 -
                                angleSixth / 46080 +
                                angleEighth / 10321920;
                        } else {
                            const angle = Math.sqrt(angleSquared);
                            const halfAngle = angle * 0.5;
                            vectorScale = Math.sin(halfAngle) / angle;
                            scalar = Math.cos(halfAngle);
                        }
                        const ix = correctionX * vectorScale;
                        const iy = correctionY * vectorScale;
                        const iz = correctionZ * vectorScale;
                        const nextQx =
                            scalar * qx + ix * qw + iy * qz - iz * qy;
                        const nextQy =
                            scalar * qy - ix * qz + iy * qw + iz * qx;
                        const nextQz =
                            scalar * qz + ix * qy - iy * qx + iz * qw;
                        const nextQw =
                            scalar * qw - ix * qx - iy * qy - iz * qz;
                        const norm = Math.sqrt(
                            nextQx * nextQx +
                            nextQy * nextQy +
                            nextQz * nextQz +
                            nextQw * nextQw
                        );
                        if (norm < EPSILON) {
                            qx = 0;
                            qy = 0;
                            qz = 0;
                            qw = 1;
                        } else {
                            const inverseNorm = 1 / norm;
                            qx = nextQx * inverseNorm;
                            qy = nextQy * inverseNorm;
                            qz = nextQz * inverseNorm;
                            qw = nextQw * inverseNorm;
                        }
                    }
                }
            }
        }
        {
            const d3x = 2 * (qx * qz + qw * qy);
            const d3y = 2 * (qy * qz - qw * qx);
            const d3z = 1 - 2 * (qx * qx + qy * qy);
            const gradientX = -length * d3y;
            const gradientY = length * d3x;
            const gradientZ = 0;
            const numerator = -(z1 - z0 - length * d3z) - alpha * lz;
            if (numerator !== 0) {
                let weightedX;
                let weightedY;
                let weightedZ;
                if (circularBending) {
                    weightedX = gradientX * inertia1;
                    weightedY = gradientY * inertia1;
                    weightedZ = gradientZ * inertia1;
                } else {
                    applyWorldInverseInertiaNormalizedScalar(
                        qx, qy, qz, qw,
                        inertia1, inertia2, inertia3,
                        gradientX, gradientY, gradientZ,
                        weighted
                    );
                    weightedX = weighted[0];
                    weightedY = weighted[1];
                    weightedZ = weighted[2];
                }
                const denominator = inverseMass0 + inverseMass1Value +
                    gradientX * weightedX +
                    gradientY * weightedY +
                    gradientZ * weightedZ + alpha;
                if (denominator >= EPSILON) {
                    const deltaLambda = numerator / denominator;
                    lz += deltaLambda;
                    z0 -= inverseMass0 * deltaLambda;
                    z1 += inverseMass1Value * deltaLambda;
                    const correctionX = weightedX * deltaLambda;
                    const correctionY = weightedY * deltaLambda;
                    const correctionZ = weightedZ * deltaLambda;
                    const angleSquared =
                        correctionX * correctionX +
                        correctionY * correctionY +
                        correctionZ * correctionZ;
                    if (angleSquared >= EPSILON * EPSILON) {
                        let vectorScale;
                        let scalar;
                        if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
                            const angleFourth = angleSquared * angleSquared;
                            const angleSixth = angleFourth * angleSquared;
                            const angleEighth = angleFourth * angleFourth;
                            vectorScale =
                                0.5 - angleSquared / 48 +
                                angleFourth / 3840 -
                                angleSixth / 645120 +
                                angleEighth / 185794560;
                            scalar =
                                1 - angleSquared / 8 +
                                angleFourth / 384 -
                                angleSixth / 46080 +
                                angleEighth / 10321920;
                        } else {
                            const angle = Math.sqrt(angleSquared);
                            const halfAngle = angle * 0.5;
                            vectorScale = Math.sin(halfAngle) / angle;
                            scalar = Math.cos(halfAngle);
                        }
                        const ix = correctionX * vectorScale;
                        const iy = correctionY * vectorScale;
                        const iz = correctionZ * vectorScale;
                        const nextQx =
                            scalar * qx + ix * qw + iy * qz - iz * qy;
                        const nextQy =
                            scalar * qy - ix * qz + iy * qw + iz * qx;
                        const nextQz =
                            scalar * qz + ix * qy - iy * qx + iz * qw;
                        const nextQw =
                            scalar * qw - ix * qx - iy * qy - iz * qz;
                        const norm = Math.sqrt(
                            nextQx * nextQx +
                            nextQy * nextQy +
                            nextQz * nextQz +
                            nextQw * nextQw
                        );
                        if (norm < EPSILON) {
                            qx = 0;
                            qy = 0;
                            qz = 0;
                            qw = 1;
                        } else {
                            const inverseNorm = 1 / norm;
                            qx = nextQx * inverseNorm;
                            qy = nextQy * inverseNorm;
                            qz = nextQz * inverseNorm;
                            qw = nextQw * inverseNorm;
                        }
                    }
                }
            }
        }
        x[segment] = x0;
        y[segment] = y0;
        z[segment] = z0;
        x[next] = x1;
        y[next] = y1;
        z[next] = z1;
        orientationX[segment] = qx;
        orientationY[segment] = qy;
        orientationZ[segment] = qz;
        orientationW[segment] = qw;
        lambdaX[segment] = lx;
        lambdaY[segment] = ly;
        lambdaZ[segment] = lz;
    }
}

/**
 * Allocation-free array sweep equivalent to solveBendTwistXPBD for normalized
 * material frames. It evaluates only the active Jacobian row for each scalar
 * XPBD component instead of constructing three complete 3x3 matrices.
 */
export function solveBendTwistXPBDArraySweep(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    const orientationX = body.orientationX;
    const orientationY = body.orientationY;
    const orientationZ = body.orientationZ;
    const orientationW = body.orientationW;
    const inverseInertia1 = body.inverseInertia1;
    const inverseInertia2 = body.inverseInertia2;
    const inverseInertia3 = body.inverseInertia3;
    const restRotation1 = body.restRotation1;
    const restRotation2 = body.restRotation2;
    const restRotation3 = body.restRotation3;
    const compliance1 = body.kirchhoffBendCompliance1;
    const compliance2 = body.kirchhoffBendCompliance2;
    const compliance3 = body.kirchhoffTwistCompliance;
    const lambda1 = body.bendTwistLambda1;
    const lambda2 = body.bendTwistLambda2;
    const lambda3 = body.bendTwistLambda3;
    const inverseDtSquared = 1 / Math.max(EPSILON, dt * dt);
    const scalarScratch = body.kirchhoffScratch.arraySweep ??= {
        weighted: new Float64Array(3),
        quaternion: new Float64Array(4)
    };
    const weighted0 = scalarScratch.weighted;
    const weighted1 = scalarScratch.weighted1 ??=
        new Float64Array(3);
    const corrected0 = scalarScratch.quaternion;
    const corrected1 = scalarScratch.quaternion1 ??=
        new Float64Array(4);
    let cachedRestX = NaN;
    let cachedRestY = NaN;
    let cachedRestZ = NaN;
    let restQx = 0;
    let restQy = 0;
    let restQz = 0;
    let restQw = 1;
    let restR00 = 1;
    let restR01 = 0;
    let restR02 = 0;
    let restR10 = 0;
    let restR11 = 1;
    let restR12 = 0;
    let restR20 = 0;
    let restR21 = 0;
    let restR22 = 1;
    const jointCount = end - start;
    for (let offset = 0; offset < jointCount; offset++) {
        const joint = reverse ? end - 1 - offset : start + offset;
        const previousSegment = joint - 1;
        const nextSegment = joint;
        let q0x = orientationX[previousSegment];
        let q0y = orientationY[previousSegment];
        let q0z = orientationZ[previousSegment];
        let q0w = orientationW[previousSegment];
        let q1x = orientationX[nextSegment];
        let q1y = orientationY[nextSegment];
        let q1z = orientationZ[nextSegment];
        let q1w = orientationW[nextSegment];
        let l1 = lambda1[joint];
        let l2 = lambda2[joint];
        let l3 = lambda3[joint];
        const restX = restRotation1[joint];
        const restY = restRotation2[joint];
        const restZ = restRotation3[joint];
        if (
            restX !== cachedRestX ||
            restY !== cachedRestY ||
            restZ !== cachedRestZ
        ) {
            const angleSquared = restX * restX + restY * restY + restZ * restZ;
            const angle = Math.sqrt(angleSquared);
            let vectorScale;
            if (angle < SMALL_ANGLE) {
                const angleFourth = angleSquared * angleSquared;
                vectorScale = 0.5 - angleSquared / 48 + angleFourth / 3840;
                restQw = 1 - angleSquared / 8 + angleFourth / 384;
            } else {
                const halfAngle = angle * 0.5;
                vectorScale = Math.sin(halfAngle) / angle;
                restQw = Math.cos(halfAngle);
            }
            restQx = restX * vectorScale;
            restQy = restY * vectorScale;
            restQz = restZ * vectorScale;
            const xx = restQx * restQx;
            const yy = restQy * restQy;
            const zz = restQz * restQz;
            const xy = restQx * restQy;
            const xz = restQx * restQz;
            const yz = restQy * restQz;
            const wx = restQw * restQx;
            const wy = restQw * restQy;
            const wz = restQw * restQz;
            restR00 = 1 - 2 * (yy + zz);
            restR01 = 2 * (xy - wz);
            restR02 = 2 * (xz + wy);
            restR10 = 2 * (xy + wz);
            restR11 = 1 - 2 * (xx + zz);
            restR12 = 2 * (yz - wx);
            restR20 = 2 * (xz - wy);
            restR21 = 2 * (yz + wx);
            restR22 = 1 - 2 * (xx + yy);
            cachedRestX = restX;
            cachedRestY = restY;
            cachedRestZ = restZ;
        }
        const inertia0X = inverseInertia1[previousSegment];
        const inertia0Y = inverseInertia2[previousSegment];
        const inertia0Z = inverseInertia3[previousSegment];
        const inertia1X = inverseInertia1[nextSegment];
        const inertia1Y = inverseInertia2[nextSegment];
        const inertia1Z = inverseInertia3[nextSegment];
        for (let component = 0; component < 3; component++) {
            const relativeX = q0w * q1x - q0x * q1w -
                q0y * q1z + q0z * q1y;
            const relativeY = q0w * q1y + q0x * q1z -
                q0y * q1w - q0z * q1x;
            const relativeZ = q0w * q1z - q0x * q1y +
                q0y * q1x - q0z * q1w;
            const relativeW = q0w * q1w + q0x * q1x +
                q0y * q1y + q0z * q1z;
            let errorX = restQw * relativeX - restQx * relativeW -
                restQy * relativeZ + restQz * relativeY;
            let errorY = restQw * relativeY + restQx * relativeZ -
                restQy * relativeW - restQz * relativeX;
            let errorZ = restQw * relativeZ - restQx * relativeY +
                restQy * relativeX - restQz * relativeW;
            let errorW = restQw * relativeW + restQx * relativeX +
                restQy * relativeY + restQz * relativeZ;
            if (errorW < 0) {
                errorX = -errorX;
                errorY = -errorY;
                errorZ = -errorZ;
                errorW = -errorW;
            }
            const vectorNorm = Math.sqrt(
                errorX * errorX + errorY * errorY + errorZ * errorZ
            );
            let strainX;
            let strainY;
            let strainZ;
            if (vectorNorm < SMALL_ANGLE) {
                const scale = 2 / Math.max(EPSILON, errorW);
                strainX = errorX * scale;
                strainY = errorY * scale;
                strainZ = errorZ * scale;
            } else {
                const angle = 2 * Math.atan2(
                    vectorNorm,
                    clamp(errorW, 0, 1)
                );
                const scale = angle / vectorNorm;
                strainX = errorX * scale;
                strainY = errorY * scale;
                strainZ = errorZ * scale;
            }
            let j0;
            let j1;
            let j2;
            let constraint;
            let oldLambda;
            let compliance;
            if (component === 0) {
                constraint = strainX;
                oldLambda = l1;
                compliance = compliance1[joint];
            } else if (component === 1) {
                constraint = strainY;
                oldLambda = l2;
                compliance = compliance2[joint];
            } else {
                constraint = strainZ;
                oldLambda = l3;
                compliance = compliance3[joint];
            }
            const alpha = Math.max(0, compliance) * inverseDtSquared;
            const numerator = -constraint - alpha * oldLambda;
            // Straight rest-state shaft joints commonly land here exactly.
            // Their XPBD correction is mathematically zero, so the Jacobian,
            // world inertia and quaternion exponentials cannot affect state.
            if (numerator === 0) continue;
            const strainAngleSquared = strainX * strainX +
                strainY * strainY + strainZ * strainZ;
            const strainAngle = Math.sqrt(strainAngleSquared);
            let secondOrder;
            if (strainAngle < SMALL_ANGLE) {
                secondOrder = 1 / 12 + strainAngleSquared / 720 +
                    strainAngleSquared * strainAngleSquared / 30240;
            } else {
                const halfAngle = strainAngle * 0.5;
                const cotangentHalf = Math.cos(halfAngle) /
                    Math.max(EPSILON, Math.sin(halfAngle));
                secondOrder = (1 - halfAngle * cotangentHalf) /
                    strainAngleSquared;
            }
            if (component === 0) {
                j0 = 1 - secondOrder *
                    (strainY * strainY + strainZ * strainZ);
                j1 = 0.5 * strainZ +
                    secondOrder * strainX * strainY;
                j2 = -0.5 * strainY +
                    secondOrder * strainX * strainZ;
            } else if (component === 1) {
                j0 = -0.5 * strainZ +
                    secondOrder * strainX * strainY;
                j1 = 1 - secondOrder *
                    (strainX * strainX + strainZ * strainZ);
                j2 = 0.5 * strainX +
                    secondOrder * strainY * strainZ;
            } else {
                j0 = 0.5 * strainY +
                    secondOrder * strainX * strainZ;
                j1 = -0.5 * strainX +
                    secondOrder * strainY * strainZ;
                j2 = 1 - secondOrder *
                    (strainX * strainX + strainY * strainY);
            }
            const a0 = j0 * restR00 + j1 * restR01 + j2 * restR02;
            const a1 = j0 * restR10 + j1 * restR11 + j2 * restR12;
            const a2 = j0 * restR20 + j1 * restR21 + j2 * restR22;
            const q0xx = q0x * q0x;
            const q0yy = q0y * q0y;
            const q0zz = q0z * q0z;
            const q0xy = q0x * q0y;
            const q0xz = q0x * q0z;
            const q0yz = q0y * q0z;
            const q0wx = q0w * q0x;
            const q0wy = q0w * q0y;
            const q0wz = q0w * q0z;
            const gradientX = a0 * (1 - 2 * (q0yy + q0zz)) +
                a1 * (2 * (q0xy - q0wz)) +
                a2 * (2 * (q0xz + q0wy));
            const gradientY = a0 * (2 * (q0xy + q0wz)) +
                a1 * (1 - 2 * (q0xx + q0zz)) +
                a2 * (2 * (q0yz - q0wx));
            const gradientZ = a0 * (2 * (q0xz - q0wy)) +
                a1 * (2 * (q0yz + q0wx)) +
                a2 * (1 - 2 * (q0xx + q0yy));
            applyWorldInverseInertiaNormalizedScalar(
                q0x,
                q0y,
                q0z,
                q0w,
                inertia0X,
                inertia0Y,
                inertia0Z,
                -gradientX,
                -gradientY,
                -gradientZ,
                weighted0
            );
            applyWorldInverseInertiaNormalizedScalar(
                q1x,
                q1y,
                q1z,
                q1w,
                inertia1X,
                inertia1Y,
                inertia1Z,
                gradientX,
                gradientY,
                gradientZ,
                weighted1
            );
            const denominator =
                -gradientX * weighted0[0] -
                gradientY * weighted0[1] -
                gradientZ * weighted0[2] +
                gradientX * weighted1[0] +
                gradientY * weighted1[1] +
                gradientZ * weighted1[2] + alpha;
            if (denominator < EPSILON) continue;
            const deltaLambda = numerator / denominator;
            if (component === 0) l1 = oldLambda + deltaLambda;
            else if (component === 1) l2 = oldLambda + deltaLambda;
            else l3 = oldLambda + deltaLambda;
            applyAngularCorrectionNormalizedScalar(
                q0x,
                q0y,
                q0z,
                q0w,
                weighted0[0] * deltaLambda,
                weighted0[1] * deltaLambda,
                weighted0[2] * deltaLambda,
                corrected0
            );
            applyAngularCorrectionNormalizedScalar(
                q1x,
                q1y,
                q1z,
                q1w,
                weighted1[0] * deltaLambda,
                weighted1[1] * deltaLambda,
                weighted1[2] * deltaLambda,
                corrected1
            );
            q0x = corrected0[0];
            q0y = corrected0[1];
            q0z = corrected0[2];
            q0w = corrected0[3];
            q1x = corrected1[0];
            q1y = corrected1[1];
            q1z = corrected1[2];
            q1w = corrected1[3];
        }
        orientationX[previousSegment] = q0x;
        orientationY[previousSegment] = q0y;
        orientationZ[previousSegment] = q0z;
        orientationW[previousSegment] = q0w;
        orientationX[nextSegment] = q1x;
        orientationY[nextSegment] = q1y;
        orientationZ[nextSegment] = q1z;
        orientationW[nextSegment] = q1w;
        lambda1[joint] = l1;
        lambda2[joint] = l2;
        lambda3[joint] = l3;
    }
}

/**
 * Allocation-free scalar hot path for the coordinate-invariant bend/twist
 * block. This expands exactly the same Log(R), inverse left Jacobian, local
 * material gradients and Cholesky solve as the reference matrix form below.
 */
function solveBendTwistXPBDBlockArraySweepScalar(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    const orientationX = body.orientationX;
    const orientationY = body.orientationY;
    const orientationZ = body.orientationZ;
    const orientationW = body.orientationW;
    const inverseInertia1 = body.inverseInertia1;
    const inverseInertia2 = body.inverseInertia2;
    const inverseInertia3 = body.inverseInertia3;
    const restRotation1 = body.restRotation1;
    const restRotation2 = body.restRotation2;
    const restRotation3 = body.restRotation3;
    const compliance1 = body.kirchhoffBendCompliance1;
    const compliance2 = body.kirchhoffBendCompliance2;
    const compliance3 = body.kirchhoffTwistCompliance;
    const lambda1 = body.bendTwistLambda1;
    const lambda2 = body.bendTwistLambda2;
    const lambda3 = body.bendTwistLambda3;
    const inverseDtSquared = 1 / Math.max(EPSILON, dt * dt);
    const scratch = body.kirchhoffScratch.bendTwistScalar ??= {
        gradient: new Float64Array(9),
        nextGradient: new Float64Array(9),
        corrected0: new Float64Array(4),
        corrected1: new Float64Array(4)
    };
    const gradient = scratch.gradient;
    const nextGradient = scratch.nextGradient;
    let cachedRestX = NaN;
    let cachedRestY = NaN;
    let cachedRestZ = NaN;
    let restQx = 0;
    let restQy = 0;
    let restQz = 0;
    let restQw = 1;
    let restR00 = 1;
    let restR01 = 0;
    let restR02 = 0;
    let restR10 = 0;
    let restR11 = 1;
    let restR12 = 0;
    let restR20 = 0;
    let restR21 = 0;
    let restR22 = 1;
    const jointCount = end - start;
    for (let offset = 0; offset < jointCount; offset++) {
        const joint = reverse ? end - 1 - offset : start + offset;
        const previousSegment = joint - 1;
        const nextSegment = joint;
        const q0x = orientationX[previousSegment];
        const q0y = orientationY[previousSegment];
        const q0z = orientationZ[previousSegment];
        const q0w = orientationW[previousSegment];
        const q1x = orientationX[nextSegment];
        const q1y = orientationY[nextSegment];
        const q1z = orientationZ[nextSegment];
        const q1w = orientationW[nextSegment];
        const restX = restRotation1[joint];
        const restY = restRotation2[joint];
        const restZ = restRotation3[joint];
        const identityRestRotation =
            restX === 0 && restY === 0 && restZ === 0;
        if (
            restX !== cachedRestX ||
            restY !== cachedRestY ||
            restZ !== cachedRestZ
        ) {
            if (identityRestRotation) {
                restQx = 0;
                restQy = 0;
                restQz = 0;
                restQw = 1;
                restR00 = 1; restR01 = 0; restR02 = 0;
                restR10 = 0; restR11 = 1; restR12 = 0;
                restR20 = 0; restR21 = 0; restR22 = 1;
            } else {
                const restAngleSquared = restX * restX +
                    restY * restY + restZ * restZ;
                let vectorScale;
                if (restAngleSquared < TRIG_SERIES_ANGLE_SQUARED) {
                    const fourth = restAngleSquared * restAngleSquared;
                    const sixth = fourth * restAngleSquared;
                    const eighth = fourth * fourth;
                    vectorScale = 0.5 - restAngleSquared / 48 +
                        fourth / 3840 - sixth / 645120 +
                        eighth / 185794560;
                    restQw = 1 - restAngleSquared / 8 + fourth / 384 -
                        sixth / 46080 + eighth / 10321920;
                } else {
                    const angle = Math.sqrt(restAngleSquared);
                    const halfAngle = angle * 0.5;
                    vectorScale = Math.sin(halfAngle) / angle;
                    restQw = Math.cos(halfAngle);
                }
                restQx = restX * vectorScale;
                restQy = restY * vectorScale;
                restQz = restZ * vectorScale;
                const xx = restQx * restQx;
                const yy = restQy * restQy;
                const zz = restQz * restQz;
                const xy = restQx * restQy;
                const xz = restQx * restQz;
                const yz = restQy * restQz;
                const wx = restQw * restQx;
                const wy = restQw * restQy;
                const wz = restQw * restQz;
                restR00 = 1 - 2 * (yy + zz);
                restR01 = 2 * (xy - wz);
                restR02 = 2 * (xz + wy);
                restR10 = 2 * (xy + wz);
                restR11 = 1 - 2 * (xx + zz);
                restR12 = 2 * (yz - wx);
                restR20 = 2 * (xz - wy);
                restR21 = 2 * (yz + wx);
                restR22 = 1 - 2 * (xx + yy);
            }
            cachedRestX = restX;
            cachedRestY = restY;
            cachedRestZ = restZ;
        }

        const relativeX = q0w * q1x - q0x * q1w -
            q0y * q1z + q0z * q1y;
        const relativeY = q0w * q1y + q0x * q1z -
            q0y * q1w - q0z * q1x;
        const relativeZ = q0w * q1z - q0x * q1y +
            q0y * q1x - q0z * q1w;
        const relativeW = q0w * q1w + q0x * q1x +
            q0y * q1y + q0z * q1z;
        let errorX = identityRestRotation
            ? relativeX
            : restQw * relativeX - restQx * relativeW -
                restQy * relativeZ + restQz * relativeY;
        let errorY = identityRestRotation
            ? relativeY
            : restQw * relativeY + restQx * relativeZ -
                restQy * relativeW - restQz * relativeX;
        let errorZ = identityRestRotation
            ? relativeZ
            : restQw * relativeZ - restQx * relativeY +
                restQy * relativeX - restQz * relativeW;
        let errorW = identityRestRotation
            ? relativeW
            : restQw * relativeW + restQx * relativeX +
                restQy * relativeY + restQz * relativeZ;
        if (errorW < 0) {
            errorX = -errorX;
            errorY = -errorY;
            errorZ = -errorZ;
            errorW = -errorW;
        }
        const vectorNorm = Math.sqrt(
            errorX * errorX + errorY * errorY + errorZ * errorZ
        );
        let strainX;
        let strainY;
        let strainZ;
        if (vectorNorm < SMALL_ANGLE) {
            const scale = 2 / Math.max(EPSILON, errorW);
            strainX = errorX * scale;
            strainY = errorY * scale;
            strainZ = errorZ * scale;
        } else {
            const angle = 2 * Math.atan2(vectorNorm, clamp(errorW, 0, 1));
            const scale = angle / vectorNorm;
            strainX = errorX * scale;
            strainY = errorY * scale;
            strainZ = errorZ * scale;
        }
        const alpha0 = Math.max(0, compliance1[joint]) * inverseDtSquared;
        const alpha1 = Math.max(0, compliance2[joint]) * inverseDtSquared;
        const alpha2 = Math.max(0, compliance3[joint]) * inverseDtSquared;
        const b0 = -strainX - alpha0 * lambda1[joint];
        const b1 = -strainY - alpha1 * lambda2[joint];
        const b2 = -strainZ - alpha2 * lambda3[joint];
        if (b0 === 0 && b1 === 0 && b2 === 0) continue;

        const strainAngleSquared = strainX * strainX +
            strainY * strainY + strainZ * strainZ;
        let secondOrder;
        if (strainAngleSquared < TRIG_SERIES_ANGLE_SQUARED) {
            const fourth = strainAngleSquared * strainAngleSquared;
            const sixth = fourth * strainAngleSquared;
            const eighth = fourth * fourth;
            secondOrder = 1 / 12 + strainAngleSquared / 720 +
                fourth / 30240 + sixth / 1209600 + eighth / 47900160;
        } else {
            const angle = Math.sqrt(strainAngleSquared);
            const halfAngle = angle * 0.5;
            const cotangentHalf = Math.cos(halfAngle) /
                Math.max(EPSILON, Math.sin(halfAngle));
            secondOrder = (1 - halfAngle * cotangentHalf) /
                strainAngleSquared;
        }
        const j00 = 1 - secondOrder *
            (strainY * strainY + strainZ * strainZ);
        const j01 = 0.5 * strainZ + secondOrder * strainX * strainY;
        const j02 = -0.5 * strainY + secondOrder * strainX * strainZ;
        const j10 = -0.5 * strainZ + secondOrder * strainX * strainY;
        const j11 = 1 - secondOrder *
            (strainX * strainX + strainZ * strainZ);
        const j12 = 0.5 * strainX + secondOrder * strainY * strainZ;
        const j20 = 0.5 * strainY + secondOrder * strainX * strainZ;
        const j21 = -0.5 * strainX + secondOrder * strainY * strainZ;
        const j22 = 1 - secondOrder *
            (strainX * strainX + strainY * strainY);
        if (identityRestRotation) {
            gradient[0] = j00; gradient[1] = j01; gradient[2] = j02;
            gradient[3] = j10; gradient[4] = j11; gradient[5] = j12;
            gradient[6] = j20; gradient[7] = j21; gradient[8] = j22;
        } else {
            gradient[0] = j00 * restR00 + j01 * restR01 + j02 * restR02;
            gradient[1] = j00 * restR10 + j01 * restR11 + j02 * restR12;
            gradient[2] = j00 * restR20 + j01 * restR21 + j02 * restR22;
            gradient[3] = j10 * restR00 + j11 * restR01 + j12 * restR02;
            gradient[4] = j10 * restR10 + j11 * restR11 + j12 * restR12;
            gradient[5] = j10 * restR20 + j11 * restR21 + j12 * restR22;
            gradient[6] = j20 * restR00 + j21 * restR01 + j22 * restR02;
            gradient[7] = j20 * restR10 + j21 * restR11 + j22 * restR12;
            gradient[8] = j20 * restR20 + j21 * restR21 + j22 * restR22;
        }
        for (let row = 0; row < 3; row++) {
            const rowOffset = row * 3;
            const gx = gradient[rowOffset];
            const gy = gradient[rowOffset + 1];
            const gz = gradient[rowOffset + 2];
            const tx = 2 * (-relativeY * gz + relativeZ * gy);
            const ty = 2 * (-relativeZ * gx + relativeX * gz);
            const tz = 2 * (-relativeX * gy + relativeY * gx);
            nextGradient[rowOffset] = gx + relativeW * tx -
                relativeY * tz + relativeZ * ty;
            nextGradient[rowOffset + 1] = gy + relativeW * ty -
                relativeZ * tx + relativeX * tz;
            nextGradient[rowOffset + 2] = gz + relativeW * tz -
                relativeX * ty + relativeY * tx;
        }
        const d00 = inverseInertia1[previousSegment];
        const d01 = inverseInertia2[previousSegment];
        const d02 = inverseInertia3[previousSegment];
        const d10 = inverseInertia1[nextSegment];
        const d11 = inverseInertia2[nextSegment];
        const d12 = inverseInertia3[nextSegment];
        const g00 = gradient[0]; const g01 = gradient[1]; const g02 = gradient[2];
        const g10 = gradient[3]; const g11 = gradient[4]; const g12 = gradient[5];
        const g20 = gradient[6]; const g21 = gradient[7]; const g22 = gradient[8];
        const n00 = nextGradient[0]; const n01 = nextGradient[1]; const n02 = nextGradient[2];
        const n10 = nextGradient[3]; const n11 = nextGradient[4]; const n12 = nextGradient[5];
        const n20 = nextGradient[6]; const n21 = nextGradient[7]; const n22 = nextGradient[8];
        const a00 = g00 * d00 * g00 + g01 * d01 * g01 +
            g02 * d02 * g02 + n00 * d10 * n00 +
            n01 * d11 * n01 + n02 * d12 * n02 + alpha0;
        if (a00 <= EPSILON) continue;
        const a01 = g00 * d00 * g10 + g01 * d01 * g11 +
            g02 * d02 * g12 + n00 * d10 * n10 +
            n01 * d11 * n11 + n02 * d12 * n12;
        const a02 = g00 * d00 * g20 + g01 * d01 * g21 +
            g02 * d02 * g22 + n00 * d10 * n20 +
            n01 * d11 * n21 + n02 * d12 * n22;
        const a11 = g10 * d00 * g10 + g11 * d01 * g11 +
            g12 * d02 * g12 + n10 * d10 * n10 +
            n11 * d11 * n11 + n12 * d12 * n12 + alpha1;
        const a12 = g10 * d00 * g20 + g11 * d01 * g21 +
            g12 * d02 * g22 + n10 * d10 * n20 +
            n11 * d11 * n21 + n12 * d12 * n22;
        const a22 = g20 * d00 * g20 + g21 * d01 * g21 +
            g22 * d02 * g22 + n20 * d10 * n20 +
            n21 * d11 * n21 + n22 * d12 * n22 + alpha2;
        const l00 = Math.sqrt(a00);
        const l10 = a01 / l00;
        const l20 = a02 / l00;
        const diagonal1 = a11 - l10 * l10;
        if (diagonal1 <= EPSILON) continue;
        const l11 = Math.sqrt(diagonal1);
        const l21 = (a12 - l20 * l10) / l11;
        const diagonal2 = a22 - l20 * l20 - l21 * l21;
        if (diagonal2 <= EPSILON) continue;
        const l22 = Math.sqrt(diagonal2);
        const y0 = b0 / l00;
        const y1 = (b1 - l10 * y0) / l11;
        const y2 = (b2 - l20 * y0 - l21 * y1) / l22;
        const delta2 = y2 / l22;
        const delta1 = (y1 - l21 * delta2) / l11;
        const delta0 = (y0 - l10 * delta1 - l20 * delta2) / l00;
        const local0X = -(g00 * delta0 + g10 * delta1 +
            g20 * delta2) * d00;
        const local0Y = -(g01 * delta0 + g11 * delta1 +
            g21 * delta2) * d01;
        const local0Z = -(g02 * delta0 + g12 * delta1 +
            g22 * delta2) * d02;
        const local1X = (n00 * delta0 + n10 * delta1 +
            n20 * delta2) * d10;
        const local1Y = (n01 * delta0 + n11 * delta1 +
            n21 * delta2) * d11;
        const local1Z = (n02 * delta0 + n12 * delta1 +
            n22 * delta2) * d12;
        applyLocalAngularCorrectionNormalizedScalar(
            q0x, q0y, q0z, q0w,
            local0X, local0Y, local0Z,
            scratch.corrected0
        );
        applyLocalAngularCorrectionNormalizedScalar(
            q1x, q1y, q1z, q1w,
            local1X, local1Y, local1Z,
            scratch.corrected1
        );
        orientationX[previousSegment] = scratch.corrected0[0];
        orientationY[previousSegment] = scratch.corrected0[1];
        orientationZ[previousSegment] = scratch.corrected0[2];
        orientationW[previousSegment] = scratch.corrected0[3];
        orientationX[nextSegment] = scratch.corrected1[0];
        orientationY[nextSegment] = scratch.corrected1[1];
        orientationZ[nextSegment] = scratch.corrected1[2];
        orientationW[nextSegment] = scratch.corrected1[3];
        lambda1[joint] += delta0;
        lambda2[joint] += delta1;
        lambda3[joint] += delta2;
    }
}

export function solveBendTwistXPBDBlockArraySweep(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    return solveBendTwistXPBDBlockArraySweepScalar(
        body,
        start,
        end,
        reverse,
        dt
    );
}

/** Reference matrix implementation retained for numerical parity tests. */
function solveBendTwistXPBDBlockArraySweepReference(
    body,
    start,
    end,
    reverse = false,
    dt = 1 / 120
) {
    const orientationX = body.orientationX;
    const orientationY = body.orientationY;
    const orientationZ = body.orientationZ;
    const orientationW = body.orientationW;
    const inverseInertia1 = body.inverseInertia1;
    const inverseInertia2 = body.inverseInertia2;
    const inverseInertia3 = body.inverseInertia3;
    const restRotation1 = body.restRotation1;
    const restRotation2 = body.restRotation2;
    const restRotation3 = body.restRotation3;
    const compliance1 = body.kirchhoffBendCompliance1;
    const compliance2 = body.kirchhoffBendCompliance2;
    const compliance3 = body.kirchhoffTwistCompliance;
    const lambda1 = body.bendTwistLambda1;
    const lambda2 = body.bendTwistLambda2;
    const lambda3 = body.bendTwistLambda3;
    const inverseDtSquared = 1 / Math.max(EPSILON, dt * dt);
    const scratch = body.kirchhoffScratch.bendTwistBlock ??= {
        orientation0: {},
        orientation1: {},
        restRotation: {},
        state: {},
        nextLocalGradient: new Float64Array(9),
        corrected0: new Float64Array(4),
        corrected1: new Float64Array(4)
    };
    const jointCount = end - start;
    for (let offset = 0; offset < jointCount; offset++) {
        const joint = reverse ? end - 1 - offset : start + offset;
        const previousSegment = joint - 1;
        const nextSegment = joint;
        const q0 = scratch.orientation0;
        const q1 = scratch.orientation1;
        const rest = scratch.restRotation;
        q0.x = orientationX[previousSegment];
        q0.y = orientationY[previousSegment];
        q0.z = orientationZ[previousSegment];
        q0.w = orientationW[previousSegment];
        q1.x = orientationX[nextSegment];
        q1.y = orientationY[nextSegment];
        q1.z = orientationZ[nextSegment];
        q1.w = orientationW[nextSegment];
        rest.x = restRotation1[joint];
        rest.y = restRotation2[joint];
        rest.z = restRotation3[joint];
        const state = evaluateBendTwistLocalConstraintNormalized(
            q0,
            q1,
            rest,
            scratch.state
        );
        const alpha0 = Math.max(0, compliance1[joint]) * inverseDtSquared;
        const alpha1 = Math.max(0, compliance2[joint]) * inverseDtSquared;
        const alpha2 = Math.max(0, compliance3[joint]) * inverseDtSquared;
        const b0 = -state.strain.x - alpha0 * lambda1[joint];
        const b1 = -state.strain.y - alpha1 * lambda2[joint];
        const b2 = -state.strain.z - alpha2 * lambda3[joint];
        if (b0 === 0 && b1 === 0 && b2 === 0) continue;
        const gradient = state.localGradient;
        const nextGradient = scratch.nextLocalGradient;
        const relative = state._scratch.relative;
        const rx = relative.x;
        const ry = relative.y;
        const rz = relative.z;
        const rw = relative.w;
        for (let row = 0; row < 3; row++) {
            const rowOffset = row * 3;
            const gx = gradient[rowOffset];
            const gy = gradient[rowOffset + 1];
            const gz = gradient[rowOffset + 2];
            // R(relative)^T maps a q0-local vector into q1-local
            // coordinates. This is the scalar inverse quaternion rotation.
            const tx = 2 * (-ry * gz + rz * gy);
            const ty = 2 * (-rz * gx + rx * gz);
            const tz = 2 * (-rx * gy + ry * gx);
            nextGradient[rowOffset] = gx + rw * tx - ry * tz + rz * ty;
            nextGradient[rowOffset + 1] = gy + rw * ty - rz * tx + rx * tz;
            nextGradient[rowOffset + 2] = gz + rw * tz - rx * ty + ry * tx;
        }
        const d00 = inverseInertia1[previousSegment];
        const d01 = inverseInertia2[previousSegment];
        const d02 = inverseInertia3[previousSegment];
        const d10 = inverseInertia1[nextSegment];
        const d11 = inverseInertia2[nextSegment];
        const d12 = inverseInertia3[nextSegment];
        const g00 = gradient[0];
        const g01 = gradient[1];
        const g02 = gradient[2];
        const g10 = gradient[3];
        const g11 = gradient[4];
        const g12 = gradient[5];
        const g20 = gradient[6];
        const g21 = gradient[7];
        const g22 = gradient[8];
        const n00 = nextGradient[0];
        const n01 = nextGradient[1];
        const n02 = nextGradient[2];
        const n10 = nextGradient[3];
        const n11 = nextGradient[4];
        const n12 = nextGradient[5];
        const n20 = nextGradient[6];
        const n21 = nextGradient[7];
        const n22 = nextGradient[8];
        const a00 = g00 * d00 * g00 + g01 * d01 * g01 +
            g02 * d02 * g02 + n00 * d10 * n00 +
            n01 * d11 * n01 + n02 * d12 * n02 + alpha0;
        if (a00 <= EPSILON) continue;
        const a01 = g00 * d00 * g10 + g01 * d01 * g11 +
            g02 * d02 * g12 + n00 * d10 * n10 +
            n01 * d11 * n11 + n02 * d12 * n12;
        const a02 = g00 * d00 * g20 + g01 * d01 * g21 +
            g02 * d02 * g22 + n00 * d10 * n20 +
            n01 * d11 * n21 + n02 * d12 * n22;
        const a11 = g10 * d00 * g10 + g11 * d01 * g11 +
            g12 * d02 * g12 + n10 * d10 * n10 +
            n11 * d11 * n11 + n12 * d12 * n12 + alpha1;
        const a12 = g10 * d00 * g20 + g11 * d01 * g21 +
            g12 * d02 * g22 + n10 * d10 * n20 +
            n11 * d11 * n21 + n12 * d12 * n22;
        const a22 = g20 * d00 * g20 + g21 * d01 * g21 +
            g22 * d02 * g22 + n20 * d10 * n20 +
            n21 * d11 * n21 + n22 * d12 * n22 + alpha2;
        const l00 = Math.sqrt(a00);
        const l10 = a01 / l00;
        const l20 = a02 / l00;
        const diagonal1 = a11 - l10 * l10;
        if (diagonal1 <= EPSILON) continue;
        const l11 = Math.sqrt(diagonal1);
        const l21 = (a12 - l20 * l10) / l11;
        const diagonal2 = a22 - l20 * l20 - l21 * l21;
        if (diagonal2 <= EPSILON) continue;
        const l22 = Math.sqrt(diagonal2);
        const y0 = b0 / l00;
        const y1 = (b1 - l10 * y0) / l11;
        const y2 = (b2 - l20 * y0 - l21 * y1) / l22;
        const delta2 = y2 / l22;
        const delta1 = (y1 - l21 * delta2) / l11;
        const delta0 = (y0 - l10 * delta1 - l20 * delta2) / l00;
        const local0X = -(gradient[0] * delta0 +
            gradient[3] * delta1 + gradient[6] * delta2) * d00;
        const local0Y = -(gradient[1] * delta0 +
            gradient[4] * delta1 + gradient[7] * delta2) * d01;
        const local0Z = -(gradient[2] * delta0 +
            gradient[5] * delta1 + gradient[8] * delta2) * d02;
        const local1X = (nextGradient[0] * delta0 +
            nextGradient[3] * delta1 + nextGradient[6] * delta2) * d10;
        const local1Y = (nextGradient[1] * delta0 +
            nextGradient[4] * delta1 + nextGradient[7] * delta2) * d11;
        const local1Z = (nextGradient[2] * delta0 +
            nextGradient[5] * delta1 + nextGradient[8] * delta2) * d12;
        applyLocalAngularCorrectionNormalizedScalar(
            q0.x,
            q0.y,
            q0.z,
            q0.w,
            local0X,
            local0Y,
            local0Z,
            scratch.corrected0
        );
        applyLocalAngularCorrectionNormalizedScalar(
            q1.x,
            q1.y,
            q1.z,
            q1.w,
            local1X,
            local1Y,
            local1Z,
            scratch.corrected1
        );
        orientationX[previousSegment] = scratch.corrected0[0];
        orientationY[previousSegment] = scratch.corrected0[1];
        orientationZ[previousSegment] = scratch.corrected0[2];
        orientationW[previousSegment] = scratch.corrected0[3];
        orientationX[nextSegment] = scratch.corrected1[0];
        orientationY[nextSegment] = scratch.corrected1[1];
        orientationZ[nextSegment] = scratch.corrected1[2];
        orientationW[nextSegment] = scratch.corrected1[3];
        lambda1[joint] += delta0;
        lambda2[joint] += delta1;
        lambda3[joint] += delta2;
    }
}

/** One local XPBD sweep for C = edge - restLength*d3. Mutates its arguments. */
export function solveAdaptationXPBD({
    x0,
    x1,
    orientation,
    restLength,
    inverseMass0 = 0,
    inverseMass1 = 0,
    inverseInertia = 0,
    compliance = 0,
    dt = 1 / 120,
    lambda = { x: 0, y: 0, z: 0 },
    scratch = {},
    returnState = true,
    normalizedOrientations = false,
    objectVectors = false
}) {
    const state = scratch.adaptationState ??= {};
    const gradient = scratch.adaptationGradient ??= {};
    const weightedGradient = scratch.adaptationWeightedGradient ??= {};
    const dtSquared = Math.max(EPSILON, dt * dt);
    const inverseInertiaX = objectVectors
        ? inverseInertia.x
        : componentValue(inverseInertia, 0, 0);
    const inverseInertiaY = objectVectors
        ? inverseInertia.y
        : componentValue(inverseInertia, 1, 0);
    const inverseInertiaZ = objectVectors
        ? inverseInertia.z
        : componentValue(inverseInertia, 2, 0);
    const complianceX = componentValue(compliance, 0, 0);
    const complianceY = componentValue(compliance, 1, 0);
    const complianceZ = componentValue(compliance, 2, 0);
    for (let component = 0; component < 3; component++) {
        const constraint = evaluateAdaptationComponent(
            x0,
            x1,
            orientation,
            restLength,
            component,
            gradient,
            normalizedOrientations
        );
        applyWorldInverseInertiaComponents(
            orientation,
            inverseInertiaX,
            inverseInertiaY,
            inverseInertiaZ,
            gradient,
            weightedGradient,
            normalizedOrientations
        );
        const rotationalWeight = dotVectors(gradient, weightedGradient);
        const complianceValue = component === 0
            ? complianceX
            : component === 1 ? complianceY : complianceZ;
        const alpha = Math.max(0, complianceValue) / dtSquared;
        const denominator = inverseMass0 + inverseMass1 + rotationalWeight + alpha;
        if (denominator < EPSILON) continue;
        const oldLambda = objectVectors
            ? component === 0 ? lambda.x : component === 1 ? lambda.y : lambda.z
            : componentValue(lambda, component, 0);
        const deltaLambda = (-constraint - alpha * oldLambda) / denominator;
        if (objectVectors) {
            if (component === 0) lambda.x = oldLambda + deltaLambda;
            else if (component === 1) lambda.y = oldLambda + deltaLambda;
            else lambda.z = oldLambda + deltaLambda;
        } else {
            setComponent(lambda, component, oldLambda + deltaLambda);
        }
        if (component === 0) {
            x0.x -= inverseMass0 * deltaLambda;
            x1.x += inverseMass1 * deltaLambda;
        } else if (component === 1) {
            x0.y -= inverseMass0 * deltaLambda;
            x1.y += inverseMass1 * deltaLambda;
        } else {
            x0.z -= inverseMass0 * deltaLambda;
            x1.z += inverseMass1 * deltaLambda;
        }
        weightedGradient.x *= deltaLambda;
        weightedGradient.y *= deltaLambda;
        weightedGradient.z *= deltaLambda;
        applyAngularCorrection(orientation, weightedGradient);
    }
    return returnState
        ? evaluateAdaptationConstraint(x0, x1, orientation, restLength, state)
        : null;
}

/** One local XPBD sweep of the two bend and one twist strains. */
export function solveBendTwistXPBD({
    orientation0,
    orientation1,
    restRotation = { x: 0, y: 0, z: 0 },
    inverseInertia0 = 0,
    inverseInertia1 = 0,
    compliance = 0,
    dt = 1 / 120,
    lambda = { x: 0, y: 0, z: 0 },
    scratch = {},
    returnState = true,
    normalizedOrientations = false,
    objectVectors = false
}) {
    const state = scratch.bendTwistState ??= {};
    const gradient0 = scratch.bendTwistGradient0 ??= {};
    const gradient1 = scratch.bendTwistGradient1 ??= {};
    const weighted0 = scratch.bendTwistWeighted0 ??= {};
    const weighted1 = scratch.bendTwistWeighted1 ??= {};
    const componentScratch = scratch.bendTwistComponent ??= {};
    const dtSquared = Math.max(EPSILON, dt * dt);
    const inverseInertia0X = objectVectors
        ? inverseInertia0.x
        : componentValue(inverseInertia0, 0, 0);
    const inverseInertia0Y = objectVectors
        ? inverseInertia0.y
        : componentValue(inverseInertia0, 1, 0);
    const inverseInertia0Z = objectVectors
        ? inverseInertia0.z
        : componentValue(inverseInertia0, 2, 0);
    const inverseInertia1X = objectVectors
        ? inverseInertia1.x
        : componentValue(inverseInertia1, 0, 0);
    const inverseInertia1Y = objectVectors
        ? inverseInertia1.y
        : componentValue(inverseInertia1, 1, 0);
    const inverseInertia1Z = objectVectors
        ? inverseInertia1.z
        : componentValue(inverseInertia1, 2, 0);
    const complianceX = objectVectors
        ? compliance.x
        : componentValue(compliance, 0, 0);
    const complianceY = objectVectors
        ? compliance.y
        : componentValue(compliance, 1, 0);
    const complianceZ = objectVectors
        ? compliance.z
        : componentValue(compliance, 2, 0);
    for (let component = 0; component < 3; component++) {
        const constraint = evaluateBendTwistComponent(
            orientation0,
            orientation1,
            restRotation,
            component,
            componentScratch,
            gradient1,
            normalizedOrientations
        );
        setVector(gradient0, -gradient1.x, -gradient1.y, -gradient1.z);
        applyWorldInverseInertiaComponents(
            orientation0,
            inverseInertia0X,
            inverseInertia0Y,
            inverseInertia0Z,
            gradient0,
            weighted0,
            normalizedOrientations
        );
        applyWorldInverseInertiaComponents(
            orientation1,
            inverseInertia1X,
            inverseInertia1Y,
            inverseInertia1Z,
            gradient1,
            weighted1,
            normalizedOrientations
        );
        const complianceValue = component === 0
            ? complianceX
            : component === 1 ? complianceY : complianceZ;
        const alpha = Math.max(0, complianceValue) / dtSquared;
        const denominator =
            dotVectors(gradient0, weighted0) +
            dotVectors(gradient1, weighted1) +
            alpha;
        if (denominator < EPSILON) continue;
        const oldLambda = objectVectors
            ? component === 0 ? lambda.x : component === 1 ? lambda.y : lambda.z
            : componentValue(lambda, component, 0);
        const deltaLambda = (-constraint - alpha * oldLambda) / denominator;
        if (objectVectors) {
            if (component === 0) lambda.x = oldLambda + deltaLambda;
            else if (component === 1) lambda.y = oldLambda + deltaLambda;
            else lambda.z = oldLambda + deltaLambda;
        } else {
            setComponent(lambda, component, oldLambda + deltaLambda);
        }
        weighted0.x *= deltaLambda;
        weighted0.y *= deltaLambda;
        weighted0.z *= deltaLambda;
        weighted1.x *= deltaLambda;
        weighted1.y *= deltaLambda;
        weighted1.z *= deltaLambda;
        applyAngularCorrection(orientation0, weighted0);
        applyAngularCorrection(orientation1, weighted1);
    }
    return returnState
        ? evaluateBendTwistConstraint(
            orientation0,
            orientation1,
            restRotation,
            state
        )
        : null;
}
