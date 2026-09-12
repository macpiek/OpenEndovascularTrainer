import { inverseRotateVectorByQuaternion } from './discreteKirchhoffRod.js';

export const KIRCHHOFF_BIAS_RELEASE = 'two-channel-bias-release';
const XYZ = ['x', 'y', 'z'];
const bodiesOf = joint => [joint.innerBody, joint.outerBody];
const betaOf = entry => entry.bank[entry.slot];

function sumWrenches(terms) {
    const sum = new Map();
    for (const { gradients, scale } of terms) for (const g of gradients ?? []) {
        const key = g.side + ':' + g.dof;
        sum.set(key, (sum.get(key) ?? 0) + scale * g.value);
    }
    return [...sum].filter(([, value]) => value !== 0).map(([key, value]) => ({
        side: Number(key.split(':')[0]), dof: Number(key.split(':')[1]), value }))
        .sort((a, b) => a.side - b.side || a.dof - b.dof);
}
function mergeGuards(old, current) {
    const guards = old.map(g => ({ ...g }));
    for (const g of current) {
        const previous = guards.find(p => p.object === g.object && p.key === g.key && p.at === g.at);
        if (previous && !Object.is(previous.value, g.value)) throw new Error('Bias difference material history changed');
        if (!previous) guards.push({ ...g });
    }
    return guards;
}
function ownedWrench(entry) {
    const difference = entry.difference;
    return sumWrenches([{ gradients: entry.frozenWorldGradients, scale: betaOf(entry) },
        { gradients: difference?.frozenWorldGradients, scale: difference ? betaOf(difference) : 0 }]);
}

/** Only after actual common-scale commits. Between applications this anchor
 * remains fixed, so J0->J1->J2->J0 telescopes exactly, including zero support
 * coefficients; no chain of successively rounded difference rows is kept. */
export function refreshKirchhoffBiasReactionAnchors(joint) {
    for (const entry of joint._splitMotion.twoChannel.rows.entries) if (entry.semanticIdentity)
        entry.reactionAnchor = ownedWrench(entry);
}

/** Same semantic contact/law, changed continuous J:
 *   beta*Jold + priorDifference = beta*Jnew + newDifference.
 * This changes only representation. A unit carrier holds the full WORLD
 * wrench difference. Its remaining fractional reaction is folded into the
 * fixed anchor before another transport, never dropped or force-clamped. */
export function transportKirchhoffBiasNormal(joint, entry, spec) {
    const state = joint._splitMotion.twoChannel.rows;
    const anchor = entry.reactionAnchor ?? ownedWrench(entry);
    const defect = sumWrenches([{ gradients: anchor, scale: 1 }, { gradients: spec.worldGradients, scale: -betaOf(entry) }]);
    if (!defect.every(g => Number.isFinite(g.value))) throw new Error('Nonfinite bias difference wrench');
    let difference = entry.difference;
    if (difference || defect.length) {
        const historyGuards = mergeGuards(difference?.historyGuards ?? entry.historyGuards, spec.historyGuards);
        if (!difference) {
            difference = { id: ++state.releaseVersion, mode: 'difference', owner: entry.owner, key: entry.key,
                kind: entry.kind, side: entry.side, node: entry.node, segment: entry.segment, component: entry.component,
                reason: 'continuous-bias-wrench-transport', bank: { lambda: 0 }, slot: 'lambda', identity: entry.identity.slice() };
            state.releases.push(difference); entry.difference = difference;
        }
        difference.historyGuards = historyGuards;
        difference.frozenWorldGradients = defect;
        // Exactly zero WORLD wrench is a zero reaction, regardless of carrier.
        // There is no numerical force or Jacobian epsilon in this decision.
        difference.bank.lambda = defect.length ? 1 : 0;
    }
    entry.reactionAnchor = anchor;
    entry.identity = spec.identity.slice(); entry.semanticIdentity = spec.semanticIdentity.slice();
    entry.historyGuards = spec.historyGuards.map(g => ({ ...g }));
    entry.frozenWorldGradients = spec.worldGradients.map(g => ({ ...g }));
    entry.frozenGradients = spec.gradients.map(g => ({ ...g }));
    state.historyVersion++;
}

/** A force may outlive its contact record, but never its material labels,
 * mobility or owner geometry. These guards contain copied values, not pooled
 * geometry rows; changed topology remains an explicit unsupported case. */
export function kirchhoffBiasReleaseIssue(entry) {
    const beta = betaOf(entry);
    if (!Number.isFinite(beta) || beta < 0) return 'invalid-bias-release-force';
    if (!entry.historyGuards || !entry.frozenWorldGradients) return 'missing-bias-release-force-history';
    for (const g of entry.historyGuards) {
        const value = g.at === null ? g.object[g.key] : g.object[g.key]?.[g.at];
        if (!Object.is(value, g.value)) return 'bias-release-material-or-mobility-changed';
    }
    if (!entry.frozenWorldGradients.every(g => Number.isFinite(g.value))) return 'invalid-bias-release-wrench';
    return null;
}

/** A bookkeeping transfer of the WHOLE old reaction. No mechanical response
 * is applied here. The canonical slot becomes available for a new witness,
 * while the retired slot remains owned, measured and included in every solve. */
export function retireKirchhoffBiasNormal(joint, entry, reason) {
    const state = joint._splitMotion.twoChannel.rows, beta = betaOf(entry);
    const failure = kirchhoffBiasReleaseIssue(entry);
    if (failure) throw new Error(failure);
    // A pending difference belongs to the OLD semantic contact. Detach it
    // from the canonical slot; it remains an independently solved release.
    entry.difference = null; entry.reactionAnchor = null; state.historyVersion++;
    if (beta === 0) return null;
    const release = { id: ++state.releaseVersion, mode: 'full', owner: entry.owner, key: entry.key, kind: entry.kind,
        side: entry.side, node: entry.node, segment: entry.segment, component: entry.component,
        reason, bank: { lambda: beta }, slot: 'lambda', identity: entry.identity.slice(),
        historyGuards: entry.historyGuards.map(g => ({ ...g })),
        frozenWorldGradients: entry.frozenWorldGradients.map(g => ({ ...g })) };
    state.releases.push(release);
    entry.bank[entry.slot] = 0;
    return release;
}

/** Frozen torques are WORLD vectors. Each new assembly transports them to
 * CURRENT local/right material coordinates; reusing old local J is wrong. */
export function currentKirchhoffBiasReleaseGradients(joint, release) {
    const bodies = bodiesOf(joint), result = [], angular = new Map();
    for (const g of release.frozenWorldGradients) {
        const axis = g.dof % 6, node = Math.floor(g.dof / 6);
        if (axis < 3) { result.push({ ...g }); continue; }
        const key = g.side + ':' + node;
        if (!angular.has(key)) angular.set(key, { side: g.side, node, world: { x: 0, y: 0, z: 0 } });
        angular.get(key).world[XYZ[axis - 3]] += g.value;
    }
    for (const { side, node, world } of angular.values()) {
        const body = bodies[side], q = Object.fromEntries(['x', 'y', 'z', 'w'].map(a => [a, body['orientation' + a.toUpperCase()][node]]));
        const norm = Math.hypot(...Object.values(q));
        if (!(norm > 0) || !Number.isFinite(norm)) throw new Error('Invalid current bias release frame');
        for (const a of Object.keys(q)) q[a] /= norm;
        const local = inverseRotateVectorByQuaternion(q, world, {});
        XYZ.forEach((a, i) => result.push({ side, dof: node * 6 + 3 + i, value: local[a] }));
    }
    return result.sort((a, b) => a.side - b.side || a.dof - b.dof);
}

/** Appended after every existing row/group offset. The physical row is fixed
 * at zero; its independent bias counterpart is fixed at total beta=0, so the
 * native full system must apply J_old^T*(-oldBeta), including all cross terms. */
export function appendKirchhoffBiasReleaseRows(joint, rows) {
    const appended = [];
    for (const release of joint._splitMotion.twoChannel.rows.releases) {
        if (betaOf(release) === 0) continue;
        const failure = kirchhoffBiasReleaseIssue(release);
        if (failure) throw new Error(failure);
        const row = { kind: KIRCHHOFF_BIAS_RELEASE, releaseId: release.id, alpha: 0, strain: 0, lambda: 0,
            lower: 0, upper: 0, gradients: currentKirchhoffBiasReleaseGradients(joint, release) };
        appended.push({ release, row, index: rows.length, beta: betaOf(release) });
        rows.push(row);
    }
    return appended;
}

// This is precisely the diagonal mobility/mask used by native Direct and
// CoupledSystem for extra rows. Reading it does not prescribe or mutate qG.
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

/** Exact remaining generalized response with fresh frames/mobility. No force
 * epsilon determines whether an entry exists. Contributions are summed before
 * measuring vector norms, matching the full native mechanical response. */
export function measureKirchhoffBiasReleases(joint) {
    const bodies = bodiesOf(joint), make = () => bodies.map(b => new Float64Array(b.count * 6));
    const correction = make(), pendingReleases = [], issues = [], own = new Float64Array(6);
    for (const r of joint._splitMotion.twoChannel.rows.releases) {
        const beta = betaOf(r);
        if (beta === 0) continue;
        const failure = kirchhoffBiasReleaseIssue(r);
        if (failure) { issues.push({ reason: failure, kind: r.kind, key: r.key, id: r.id, beta }); continue; }
        const gradients = currentKirchhoffBiasReleaseGradients(joint, r);
        let positionMm = 0, angleRad = 0;
        // The existing helper sorts by side/dof. Accumulate duplicate DOFs in
        // that SAME order, then measure only the touched node's six values.
        // Untouched zero nodes cannot change either maximum. The full shared
        // correction still receives every delta in its original order.
        for (let i = 0; i < gradients.length;) {
            const side = gradients[i].side, node = Math.floor(gradients[i].dof / 6);
            own.fill(0);
            do {
                const g = gradients[i], delta = -beta * g.value * mobility(bodies[g.side], g.dof);
                // Preserve the old full typed array's out-of-range behavior.
                if (g.dof >= 0 && g.dof < correction[g.side].length) own[g.dof % 6] += delta;
                correction[g.side][g.dof] += delta;
                i++;
            } while (i < gradients.length && gradients[i].side === side && Math.floor(gradients[i].dof / 6) === node);
            positionMm = Math.max(positionMm, Math.hypot(own[0], own[1], own[2]));
            angleRad = Math.max(angleRad, Math.hypot(own[3], own[4], own[5]));
        }
        pendingReleases.push({ id: r.id, mode: r.mode, kind: r.kind, key: r.key, side: r.side, node: r.node,
            reason: r.reason, ...(r.mode === 'difference' ? { carrier: beta } : { beta }), positionMm, angleRad });
    }
    const n = norms(correction);
    return { releasePositionMm: n.positionMm, releaseAngleRad: n.angleRad, releaseCorrection: correction,
        pendingReleases, issues, finite: !issues.length && correction.every(c => c.every(Number.isFinite)) };
}
