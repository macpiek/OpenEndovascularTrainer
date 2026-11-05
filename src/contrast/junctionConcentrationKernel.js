// Geometry-only reconstruction in the shared lumen of a three-arm junction.
// Anchors belong to the connected parent and daughter paths, never vessels
// selected merely because they happen to lie nearby in the projection.
export function closestTriangleWeights(point, anchors, out = new Float64Array(3)) {
    const [a, b, c] = anchors;
    const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    const apx = point.x - a.x, apy = point.y - a.y, apz = point.z - a.z;
    const ab2 = abx * abx + aby * aby + abz * abz;
    const ac2 = acx * acx + acy * acy + acz * acz;
    const dot = abx * acx + aby * acy + abz * acz;
    // A collinear/near-collinear atlas junction is a line, not an invertible
    // triangle. Its longest segment gives a continuous, bounded fallback.
    if (ab2 * ac2 - dot * dot <= 1e-12 * Math.max(ab2 * ac2, 1e-30)) {
        const bc2 = (c.x - b.x) ** 2 + (c.y - b.y) ** 2 + (c.z - b.z) ** 2;
        const [i, j, length2] = ab2 >= ac2 && ab2 >= bc2 ? [0, 1, ab2]
            : ac2 >= bc2 ? [0, 2, ac2] : [1, 2, bc2];
        out.fill(0);
        if (!(length2 > 0)) { out[0] = 1; return out; }
        const p = anchors[i], q = anchors[j];
        const t = Math.max(0, Math.min(1, ((point.x - p.x) * (q.x - p.x) +
            (point.y - p.y) * (q.y - p.y) + (point.z - p.z) * (q.z - p.z)) / length2));
        out[i] = 1 - t; out[j] = t;
        return out;
    }
    const d1 = abx * apx + aby * apy + abz * apz;
    const d2 = acx * apx + acy * apy + acz * apz;
    if (d1 <= 0 && d2 <= 0) { out.set([1, 0, 0]); return out; }
    const bpx = point.x - b.x, bpy = point.y - b.y, bpz = point.z - b.z;
    const d3 = abx * bpx + aby * bpy + abz * bpz;
    const d4 = acx * bpx + acy * bpy + acz * bpz;
    if (d3 >= 0 && d4 <= d3) { out.set([0, 1, 0]); return out; }
    const vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const v = d1 / (d1 - d3); out.set([1 - v, v, 0]); return out;
    }
    const cpx = point.x - c.x, cpy = point.y - c.y, cpz = point.z - c.z;
    const d5 = abx * cpx + aby * cpy + abz * cpz;
    const d6 = acx * cpx + acy * cpy + acz * cpz;
    if (d6 >= 0 && d5 <= d6) { out.set([0, 0, 1]); return out; }
    const vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const w = d2 / (d2 - d6); out.set([1 - w, 0, w]); return out;
    }
    const va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
        const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        out.set([0, 1 - w, w]); return out;
    }
    const denominator = va + vb + vc, v = vb / denominator, w = vc / denominator;
    out.set([1 - v - w, v, w]);
    return out;
}

// Include the actual flow-weighted junction blood as a fourth, interior
// anchor. Three affine triangles meet at that hub, so concentrations agree
// on their shared edges without selecting a nearest parent/daughter lumen.
export function junctionStarWeights(point, center, anchors, out = new Float64Array(4)) {
    const hubWeights = closestTriangleWeights(center, anchors);
    const hub = {x: 0, y: 0, z: 0};
    for (let i = 0; i < 3; i++) {
        hub.x += hubWeights[i] * anchors[i].x;
        hub.y += hubWeights[i] * anchors[i].y;
        hub.z += hubWeights[i] * anchors[i].z;
    }
    const [a, b, c] = anchors;
    const abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    const nx = aby * acz - abz * acy;
    const ny = abz * acx - abx * acz;
    const nz = abx * acy - aby * acx;
    const normal2 = nx * nx + ny * ny + nz * nz;
    const ab2 = abx * abx + aby * aby + abz * abz;
    const ac2 = acx * acx + acy * acy + acz * acz;
    let projected;
    if (normal2 > 1e-12 * Math.max(ab2 * ac2, 1e-30)) {
        const offset = ((point.x - a.x) * nx + (point.y - a.y) * ny + (point.z - a.z) * nz) / normal2;
        projected = {x: point.x - offset * nx, y: point.y - offset * ny, z: point.z - offset * nz};
    } else {
        // Degenerate star: retain the bounded longest-segment interpolation.
        const weights = closestTriangleWeights(point, anchors);
        out[0] = 0;
        for (let i = 0; i < 3; i++) out[i + 1] = weights[i];
        return out;
    }
    let closest = Infinity;
    const weights = new Float64Array(3);
    for (let i = 0; i < 3; i++) {
        const j = (i + 1) % 3, one = anchors[i], two = anchors[j];
        closestTriangleWeights(projected, [hub, one, two], weights);
        const x = weights[0] * hub.x + weights[1] * one.x + weights[2] * two.x;
        const y = weights[0] * hub.y + weights[1] * one.y + weights[2] * two.y;
        const z = weights[0] * hub.z + weights[1] * one.z + weights[2] * two.z;
        const distance2 = (projected.x - x) ** 2 + (projected.y - y) ** 2 + (projected.z - z) ** 2;
        if (distance2 < closest) {
            closest = distance2;
            out.fill(0);
            out[0] = weights[0]; out[i + 1] = weights[1]; out[j + 1] = weights[2];
        }
    }
    return out;
}

// A compact C1 partition: interpolation returns to the ordinary axial field
// outside the physical junction. Neither field values nor screen pixels set
// this support; its radii come from the anatomical common-lumen region.
export function junctionBlendWeight(point, center, coreRadius, outerRadius) {
    const distance = Math.hypot(point.x - center.x, point.y - center.y, point.z - center.z);
    if (!(outerRadius > coreRadius && coreRadius >= 0)) return 0;
    if (distance <= coreRadius) return 1;
    if (distance >= outerRadius) return 0;
    const t = (distance - coreRadius) / (outerRadius - coreRadius);
    return 1 - t * t * (3 - 2 * t);
}
