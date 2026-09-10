/**
 * Sliding aperture recovery: locate the wire under the catheter mouth using
 * material overlap only as a broad-phase hint. The closest-point coordinate
 * is free to slide. A finite clearance supplies a unilateral radial joint;
 * there is no equality between catheter and guidewire arclengths.
 */
export function evaluateKirchhoffSlidingPortal(constraint, out = {}) {
    const inner = constraint.innerBody;
    const outer = constraint.outerBody;
    const tip = outer.activeEnd;
    const hint = Math.min(inner.activeEnd - 1, Math.max(inner.activeStart, constraint.endNode));
    const window = Math.max(2, constraint.searchWindow);
    const start = Math.max(inner.activeStart, hint - window);
    const end = Math.min(inner.activeEnd - 1, hint + window);
    out.segment = -1;
    out.distance = Infinity;
    out.violation = 0;
    if (!constraint.enforceDistalPortal || !constraint.openDistal ||
        !Number.isFinite(constraint.containedLength) || tip <= outer.activeStart) return out;
    const wireTip = inner.activeEnd;
    if (wireTip <= inner.activeStart) return out;
    if (hint >= wireTip - window &&
        (outer.x[tip] - inner.x[wireTip]) * (inner.x[wireTip] - inner.x[wireTip - 1]) +
        (outer.y[tip] - inner.y[wireTip]) * (inner.y[wireTip] - inner.y[wireTip - 1]) +
        (outer.z[tip] - inner.z[wireTip]) * (inner.z[wireTip] - inner.z[wireTip - 1]) > 0) return out;
    for (let s = start; s <= end; s++) {
        const dx = inner.x[s + 1] - inner.x[s];
        const dy = inner.y[s + 1] - inner.y[s];
        const dz = inner.z[s + 1] - inner.z[s];
        const lengthSquared = dx * dx + dy * dy + dz * dz;
        if (lengthSquared < 1e-12) continue;
        const rawT = ((outer.x[tip] - inner.x[s]) * dx +
            (outer.y[tip] - inner.y[s]) * dy + (outer.z[tip] - inner.z[s]) * dz) / lengthSquared;
        // Once the wire tip has withdrawn through the mouth, it supplies no
        // spherical cap which could tow the catheter during further withdrawal.
        if (s === inner.activeEnd - 1 && rawT > 1) continue;
        const t = Math.max(0, Math.min(1, rawT));
        const x = inner.x[s] + t * dx - outer.x[tip];
        const y = inner.y[s] + t * dy - outer.y[tip];
        const z = inner.z[s] + t * dz - outer.z[tip];
        const distance = Math.hypot(x, y, z);
        if (distance >= out.distance) continue;
        out.segment = s;
        out.t = t;
        out.distance = distance;
        out.x = x;
        out.y = y;
        out.z = z;
    }
    if (out.segment >= 0) {
        out.clearance = Math.max(0, constraint.innerRadius - Math.max(
            inner.nodeRadius[out.segment], inner.nodeRadius[out.segment + 1]));
        out.violation = Math.max(0, out.distance - out.clearance);
    }
    return out;
}
