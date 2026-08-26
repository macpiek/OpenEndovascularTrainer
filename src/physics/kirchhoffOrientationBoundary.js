import {
    createBishopFrame,
    multiplyQuaternions,
    normalizeQuaternion,
    quaternionExp
} from './discreteKirchhoffRod.js';

const EDGE_EPSILON = 1e-9;

/**
 * Builds a material frame whose d3 director follows `tangent` and whose
 * rotation about d3 is the operator-supplied twist. The intrinsic curvature
 * remains expressed in material coordinates; neither the rest strain nor the
 * world-space rod geometry is rotated here.
 */
export function createProximalMaterialFrame(
    tangent,
    twist = 0,
    preferredD1 = null,
    out = {}
) {
    const length = Math.hypot(tangent?.x ?? 0, tangent?.y ?? 0, tangent?.z ?? 0);
    if (!Number.isFinite(length) || length < EDGE_EPSILON) {
        throw new RangeError('A finite non-zero proximal tangent is required');
    }
    if (!Number.isFinite(twist)) {
        throw new TypeError('Proximal material twist must be finite');
    }
    const scratch = out._proximalFrameScratch ??= {
        d3: {},
        bishop: {},
        twistVector: {},
        twistRotation: {}
    };
    const d3 = scratch.d3;
    d3.x = tangent.x / length;
    d3.y = tangent.y / length;
    d3.z = tangent.z / length;
    const bishop = createBishopFrame(d3, preferredD1, scratch.bishop);
    const twistVector = scratch.twistVector;
    twistVector.x = d3.x * twist;
    twistVector.y = d3.y * twist;
    twistVector.z = d3.z * twist;
    const twistRotation = quaternionExp(
        twistVector,
        scratch.twistRotation
    );
    return normalizeQuaternion(
        multiplyQuaternions(twistRotation, bishop, out),
        out
    );
}

/** Apply only an orientation/twist boundary to a Kirchhoff rod edge. */
export function applyProximalTwistBoundary(
    body,
    {
        twist = 0,
        segment = body?.activeStart ?? 0,
        preferredD1 = null,
        compliance = 0,
        out = {}
    } = {}
) {
    if (
        !body ||
        body.rodModel !== 'kirchhoff' ||
        typeof body.setProximalOrientationControl !== 'function'
    ) {
        throw new TypeError('A Kirchhoff rod body is required');
    }
    const targetSegment = Math.max(
        0,
        Math.min(body.segmentCount - 1, Math.floor(segment))
    );
    const scratch = out._proximalBoundaryScratch ??= { tangent: {} };
    const tangent = scratch.tangent;
    tangent.x = body.x[targetSegment + 1] - body.x[targetSegment];
    tangent.y = body.y[targetSegment + 1] - body.y[targetSegment];
    tangent.z = body.z[targetSegment + 1] - body.z[targetSegment];
    const frame = createProximalMaterialFrame(
        tangent,
        twist,
        preferredD1,
        out
    );
    body.setProximalOrientationControl(
        frame.x,
        frame.y,
        frame.z,
        frame.w,
        Math.max(0, compliance),
        targetSegment
    );
    return frame;
}
