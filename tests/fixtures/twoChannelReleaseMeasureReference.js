// Literal baseline measurement from Release SHA-256
// 8b38077b5d2451d70251fd976966d6ca268ee2b5a92086b3bf900d1638d124c6.
// Only the measurement is compared. The unchanged public gradient/history
// helpers are shared so this oracle isolates the local-norm optimization.
import { currentKirchhoffBiasReleaseGradients, kirchhoffBiasReleaseIssue } from '../../src/physics/kirchhoffTwoChannelRelease.js';
const bodiesOf = joint => [joint.innerBody, joint.outerBody];
const betaOf = entry => entry.bank[entry.slot];

function mobility(body, dof) {
    const node = Math.floor(dof / 6), axis = dof % 6;
    const start = Math.max(0, body.activeStart), end = Math.min(body.segmentCount, body.activeEnd);
    if (body.sleeping || end <= start || node < start || node > end || (node === end && axis >= 3)) return 0;
    if (axis < 3) return body.inverseMass[node];
    if (body.orientationControlCompliance === 0 && node === body.orientationControlSegment) return 0;
    return body['inverseInertia' + (axis - 2)][node];
}
function norms(corrections) {
    let positionMm = 0, angleRad = 0;
    for (const c of corrections) for (let i = 0; i < c.length; i += 6) {
        positionMm = Math.max(positionMm, Math.hypot(c[i], c[i + 1], c[i + 2]));
        angleRad = Math.max(angleRad, Math.hypot(c[i + 3], c[i + 4], c[i + 5]));
    }
    return { positionMm, angleRad };
}

export function measureKirchhoffBiasReleasesReference(joint) {
    const bodies = bodiesOf(joint), make = () => bodies.map(b => new Float64Array(b.count * 6));
    const correction = make(), pendingReleases = [], issues = [];
    for (const r of joint._splitMotion.twoChannel.rows.releases) {
        const beta = betaOf(r);
        if (beta === 0) continue;
        const failure = kirchhoffBiasReleaseIssue(r);
        if (failure) { issues.push({ reason: failure, kind: r.kind, key: r.key, id: r.id, beta }); continue; }
        const own = make();
        for (const g of currentKirchhoffBiasReleaseGradients(joint, r)) {
            const delta = -beta * g.value * mobility(bodies[g.side], g.dof);
            own[g.side][g.dof] += delta; correction[g.side][g.dof] += delta;
        }
        pendingReleases.push({ id: r.id, mode: r.mode, kind: r.kind, key: r.key, side: r.side, node: r.node,
            reason: r.reason, ...(r.mode === 'difference' ? { carrier: beta } : { beta }), ...norms(own) });
    }
    const n = norms(correction);
    return { releasePositionMm: n.positionMm, releaseAngleRad: n.angleRad, releaseCorrection: correction,
        pendingReleases, issues, finite: !issues.length && correction.every(c => c.every(Number.isFinite)) };
}
