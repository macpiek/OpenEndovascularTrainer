const EPSILON = 1e-8;

/** Rebuild once while geometry is frozen; never reuse across position updates. */
export function prepareKirchhoffCenterlineSearch(body, cache = {}) {
    const count = body.count - 1;
    if (!cache.data || cache.data.length < count * 24) cache.data = new Float64Array(count * 24);
    const data = cache.data;
    for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
        const cubic = segment - 1 >= Math.max(0, body.activeStart) &&
            segment + 2 <= Math.min(body.count - 1, body.activeEnd);
        for (let axis = 0; axis < 3; axis++) {
            const positions = axis === 0 ? body.x : axis === 1 ? body.y : body.z;
            const p0 = positions[segment], p1 = positions[segment + 1];
            const offset = segment * 24 + axis * 8;
            const chord = p1 - p0;
            data[offset + 6] = p0;
            data[offset + 7] = chord;
            if (cubic) {
                const previous = positions[segment - 1], next = positions[segment + 2];
                const a = (previous + 4 * p0 + p1) / 6;
                const b1 = (2 * p0 + p1) / 3;
                const b2 = (p0 + 2 * p1) / 3;
                const b3 = (p0 + 4 * p1 + next) / 6;
                data[offset] = a;
                data[offset + 1] = (p1 - previous) / 2;
                data[offset + 2] = (previous - 2 * p0 + p1) / 2;
                data[offset + 3] = (-previous + 3 * p0 - 3 * p1 + next) / 6;
                data[offset + 4] = Math.min(a, b1, b2, b3);
                data[offset + 5] = Math.max(a, b1, b2, b3);
            } else {
                data[offset] = p0;
                data[offset + 1] = chord;
                data[offset + 2] = data[offset + 3] = 0;
                data[offset + 4] = Math.min(p0, p1);
                data[offset + 5] = Math.max(p0, p1);
            }
        }
    }
    return cache;
}



/** Closest point on the existing cubic B-spline/linear centerline.
 * Uses the same chord seed, four Newton updates and stopping tolerance as
 * the contact sampler. Only the winning curve needs interpolation stencils
 * and a normalized tangent; candidate evaluation uses polynomial scalars.
 * The Bezier control hull bounds the ENTIRE cubic, including curved spans.
 */
export function closestKirchhoffCenterlinePoint(
    body, point, firstSegment, lastSegment, preferredSegment, out = {}, prepared = null
) {
    const data = (prepared ?? prepareKirchhoffCenterlineSearch(body, out.cache ??= {})).data;
    let bestDistanceSquared = Infinity;
    let bestSegment = -1;
    let bestT = 0;
    out.evaluatedCandidates = 0;
    out.rejectedCandidates = 0;
    // Start at the material hint so its distance can prune both neighbors.
    for (let candidate = firstSegment - 1; candidate <= lastSegment; candidate++) {
        const segment = candidate < firstSegment ? preferredSegment : candidate;
        if (candidate === preferredSegment) continue;
        let lowerBoundSquared = 0;
        let chordProjection = 0;
        let chordLengthSquared = 0;
        for (let axis = 0; axis < 3; axis++) {
            const offset = segment * 24 + axis * 8;
            const chord = data[offset + 7];
            chordProjection += (point[axis] - data[offset + 6]) * chord;
            chordLengthSquared += chord * chord;
            const distance = Math.max(0, data[offset + 4] - point[axis], point[axis] - data[offset + 5]);
            lowerBoundSquared += distance * distance;
        }
        // Strict comparison retains the original lower-segment tie break.
        // The margin covers roundoff in conversion to the Bezier hull.
        if (lowerBoundSquared > bestDistanceSquared + 1e-10) {
            out.rejectedCandidates++;
            continue;
        }
        out.evaluatedCandidates++;
        let t = chordLengthSquared > EPSILON
            ? Math.max(0, Math.min(1, chordProjection / chordLengthSquared)) : 0;
        for (let refinement = 0; refinement < 4; refinement++) {
            let numerator = 0;
            let denominator = 0;
            for (let axis = 0; axis < 3; axis++) {
                const offset = segment * 24 + axis * 8;
                const a = data[offset], b = data[offset + 1];
                const c = data[offset + 2], d = data[offset + 3];
                const distance = point[axis] - (((d * t + c) * t + b) * t + a);
                const derivative = (3 * d * t + 2 * c) * t + b;
                numerator += distance * derivative;
                denominator += derivative * derivative - distance * (6 * d * t + 2 * c);
            }
            if (Math.abs(denominator) <= EPSILON) break;
            const nextT = Math.max(0, Math.min(1, t + numerator / denominator));
            const settled = Math.abs(nextT - t) <= 1e-5;
            t = nextT;
            if (settled) break;
        }
        let distanceSquared = 0;
        for (let axis = 0; axis < 3; axis++) {
            const offset = segment * 24 + axis * 8;
            const distance = point[axis] - (((data[offset + 3] * t +
                data[offset + 2]) * t + data[offset + 1]) * t + data[offset]);
            distanceSquared += distance * distance;
        }
        if (distanceSquared < bestDistanceSquared ||
            (distanceSquared === bestDistanceSquared && segment < bestSegment)) {
            bestDistanceSquared = distanceSquared;
            bestSegment = segment;
            bestT = t;
        }
    }
    out.segment = bestSegment;
    out.t = bestT;
    out.distanceSquared = bestDistanceSquared;
    return out;
}
