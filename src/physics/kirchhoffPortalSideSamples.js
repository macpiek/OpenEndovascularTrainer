/** Exact endpoints of a straight wire cell clipped to the distal shaft side.
 * Radial distance to a straight shaft is convex along a wire cell, so checking
 * both endpoints enforces its whole side interval. Separate endpoint identities
 * avoid transferring one deepest-sample multiplier between distant witnesses.
 * Full endpoint Jacobians include the moving clipping planes and shaft axis.
 */
export function buildKirchhoffPortalSideSamples(inner, outer, innerSegment, outerSegment,
    clearance, filletRadius = 0, out = {}) {
    const pool = out.pool ??= [], samples = out.samples ??= [];
    samples.length = 0;
    const xyz = ['x', 'y', 'z'];
    const a = out.a ??= new Float64Array(3), p = out.p ??= new Float64Array(3);
    const d = out.d ??= new Float64Array(3), axis = out.axis ??= new Float64Array(3);
    for (let k = 0; k < 3; k++) {
        a[k] = inner[xyz[k]][innerSegment]; p[k] = outer[xyz[k]][outerSegment];
        d[k] = inner[xyz[k]][innerSegment + 1] - a[k];
        axis[k] = outer[xyz[k]][outerSegment + 1] - p[k];
    }
    const length = Math.hypot(...axis);
    if (!(length > 1e-12) || length < filletRadius) return out;
    let z0 = 0, denominator = 0;
    for (let k = 0; k < 3; k++) { axis[k] /= length; z0 += (a[k] - p[k]) * axis[k]; denominator += d[k] * axis[k]; }
    const end = length - filletRadius;
    let low = 0, high = 1, lowPlane = -1, highPlane = -1;
    if (Math.abs(denominator) <= 1e-12) {
        if (z0 < 0 || z0 > end) return out;
    } else {
        const startT = -z0 / denominator, endT = (end - z0) / denominator;
        const enter = Math.min(startT, endT), leave = Math.max(startT, endT);
        if (enter > low) { low = enter; lowPlane = denominator > 0 ? 0 : 1; }
        if (leave < high) { high = leave; highPlane = denominator > 0 ? 1 : 0; }
        if (low > high || high < 0 || low > 1) return out;
    }
    for (let index = 0; index < 2; index++) {
        const t = index ? high : low, plane = index ? highPlane : lowPlane;
        if (index && high === low) continue;
        const sample = pool[index] ??= { normal: new Float64Array(3), radial: new Float64Array(3), gradients: [] };
        const z = z0 + t * denominator, u = z / length;
        for (let k = 0; k < 3; k++) sample.radial[k] = a[k] + t * d[k] - p[k] - z * axis[k];
        const radius = Math.hypot(...sample.radial);
        if (radius > 1e-12) for (let k = 0; k < 3; k++) sample.normal[k] = sample.radial[k] / radius;
        else {
            // The radial norm has no unique derivative on the axis. Any
            // perpendicular direction is valid and the open gap carries no load.
            const reference = Math.abs(axis[0]) < .8 ? 0 : 1;
            for (let k = 0; k < 3; k++) sample.normal[k] = Number(k === reference) - axis[reference] * axis[k];
            const norm = Math.hypot(...sample.normal);
            for (let k = 0; k < 3; k++) sample.normal[k] /= norm;
        }
        let radialSlope = 0;
        for (let k = 0; k < 3; k++) radialSlope += sample.normal[k] * d[k];
        const chain = plane >= 0 ? radialSlope / denominator : 0;
        sample.innerT = t; sample.outerT = Math.max(0, Math.min(1, u)); sample.radius = radius;
        sample.gap = clearance - radius; sample.plane = plane; sample.endpoint = index;
        let count = 0;
        for (let side = 0; side < 2; side++) for (let endpoint = 0; endpoint < 2; endpoint++) for (let k = 0; k < 3; k++) {
            const weight = endpoint ? (side ? u : t) : 1 - (side ? u : t);
            let value = (side ? 1 : -1) * weight * sample.normal[k];
            if (plane >= 0) {
                const gradPlane = side === 0 ? (endpoint ? t : 1 - t) * axis[k]
                    : endpoint ? sample.radial[k] / length - (plane === 1 ? axis[k] : 0)
                        : -sample.radial[k] / length - (plane === 0 ? axis[k] : 0);
                value += chain * gradPlane;
            }
            if (value === 0) continue;
            const gradient = sample.gradients[count++] ??= {};
            gradient.side = side; gradient.dof = ((side ? outerSegment : innerSegment) + endpoint) * 6 + k;
            gradient.value = value;
        }
        sample.gradients.length = count; samples.push(sample);
    }
    return out;
}
