import { buildKirchhoffCoupledFrictionRows, commitKirchhoffCoupledFrictionMultipliers } from './kirchhoffCoupledFrictionRows.js';
import { projectKirchhoffSurfaceFriction } from './kirchhoffSurfaceFriction.js';
import { quaternionExp, multiplyQuaternions, normalizeQuaternion } from './discreteKirchhoffRod.js';

const poseFields = ['x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW'];

/** Prepare a FINAL-LOAD cone repair with the SAME generalized surface reaction.
 * This is a small feasibility corrector, not a substitute for nonlinear solve.
 * Every projected dLambda acts through W J^T on BOTH rods, including material-
 * frame moments. Fn, previous poses, velocities and material multipliers stay
 * unchanged. Caller must refresh geometry and recheck ALL original residuals
 * after apply; the returned frozen-geometry feasibility is not a convergence
 * claim. Oversized corrections are rejected before any state is committed.
 */
export function prepareKirchhoffCoupledConeRepair(constraint, dt, {
    maximumPositionCorrectionMm = .001, maximumAngleCorrectionRad = .005
} = {}, out = {}) {
    if (!(Number.isFinite(maximumPositionCorrectionMm) && maximumPositionCorrectionMm >= 0) ||
        !(Number.isFinite(maximumAngleCorrectionRad) && maximumAngleCorrectionRad >= 0)) throw new RangeError('Invalid cone repair limits');
    const batch = buildKirchhoffCoupledFrictionRows(constraint, dt, out.batch ??= {});
    out.constraint = constraint; out.applied = false; out.changedContacts = 0;
    out.maximumPositionCorrectionMm = out.maximumAngleCorrectionRad = 0;
    out.maximumMultiplierCorrection = 0;
    out.increment = new Float64Array(batch.rows.length);
    out.responses = [constraint.innerBody, constraint.outerBody].map(body => ({
        body, start: body.activeStart, end: body.activeEnd,
        controlled: body.orientationControlCompliance === 0 ? body.orientationControlSegment : -1,
        mobility: Object.fromEntries(['inverseMass', 'inverseInertia1', 'inverseInertia2', 'inverseInertia3'].map(key => [key, body[key].slice()])),
        correction: new Float64Array(body.count * 6),
        before: Object.fromEntries(poseFields.map(key => [key, body[key].slice()])),
        after: Object.fromEntries(poseFields.map(key => [key, Float64Array.from(body[key])]))
    }));
    for (const entry of batch.entries) {
        const lambda = entry.surface.rows.map(row => row.lambda);
        const projected = projectKirchhoffSurfaceFriction(lambda, entry.contact.normalLambda, entry.surface.group.mu);
        out.changedContacts += Number(projected.distance > 0);
        for (let axis = 0; axis < 2; axis++) {
            const delta = projected.lambda[axis] - lambda[axis];
            out.increment[entry.rowStart + axis] = delta;
            out.maximumMultiplierCorrection = Math.max(out.maximumMultiplierCorrection, Math.abs(delta));
            for (const g of entry.surface.rows[axis].gradients) {
                const response = out.responses[g.side], node = Math.floor(g.dof / 6), component = g.dof % 6;
                if (node >= response.start && node <= response.end && (component < 3 || node < response.end))
                    response.correction[g.dof] += g.value * delta;
            }
        }
    }
    for (const response of out.responses) {
        const { body, correction, after, start, end } = response;
        const controlled = body.orientationControlCompliance === 0 ? body.orientationControlSegment : -1;
        for (let node = start; node <= end; node++) {
            const d = node * 6;
            for (let axis = 0; axis < 3; axis++) {
                correction[d + axis] *= body.inverseMass[node];
                after[poseFields[axis]][node] += correction[d + axis];
            }
            out.maximumPositionCorrectionMm = Math.max(out.maximumPositionCorrectionMm,
                Math.hypot(correction[d], correction[d + 1], correction[d + 2]));
            if (node === end) continue;
            for (let axis = 0; axis < 3; axis++) correction[d + 3 + axis] *=
                node === controlled ? 0 : body['inverseInertia' + (axis + 1)][node];
            out.maximumAngleCorrectionRad = Math.max(out.maximumAngleCorrectionRad,
                Math.hypot(correction[d + 3], correction[d + 4], correction[d + 5]));
            if (correction[d + 3] === 0 && correction[d + 4] === 0 && correction[d + 5] === 0) continue;
            const increment = quaternionExp({ x: correction[d + 3], y: correction[d + 4], z: correction[d + 5] });
            const q = { x: body.orientationX[node], y: body.orientationY[node], z: body.orientationZ[node], w: body.orientationW[node] };
            const next = normalizeQuaternion(multiplyQuaternions(q, increment));
            for (const axis of ['X', 'Y', 'Z', 'W']) after['orientation' + axis][node] = next[axis.toLowerCase()];
        }
        if (![...correction, ...Object.values(after).flatMap(values => [...values])].every(Number.isFinite)) throw new RangeError('Non-finite cone repair');
    }
    out.accepted = out.maximumPositionCorrectionMm <= maximumPositionCorrectionMm &&
        out.maximumAngleCorrectionRad <= maximumAngleCorrectionRad;
    out.status = !out.accepted ? 'correction-too-large' : out.changedContacts ? 'prepared' : 'already-feasible';
    return out;
}

/** Apply once while the plan's geometry and final Fn remain unchanged.
 * Does not certify refreshed cone/material/normal/boundary/length residuals.
 */
export function applyKirchhoffCoupledConeRepair(plan) {
    if (plan.applied) throw new Error('Cone repair already applied');
    if (!plan.accepted) throw new Error('Cone repair exceeds the permitted correction');
    for (const entry of plan.batch.entries) if (entry.contact.normalLambda !== entry.surface.group.normalLambda)
        throw new Error('Normal load changed before cone repair');
    for (const response of plan.responses) {
        const { body, before, start, end } = response;
        if (body.activeStart !== start || body.activeEnd !== end) throw new Error('Cone repair topology changed');
        if ((body.orientationControlCompliance === 0 ? body.orientationControlSegment : -1) !== response.controlled)
            throw new Error('Cone repair frame control changed');
        for (const [key, values] of Object.entries(response.mobility)) for (let i = start; i < Math.min(end + 1, body[key].length); i++)
            if (body[key][i] !== values[i]) throw new Error('Cone repair mobility changed');
        for (const key of poseFields) for (let i = start; i < Math.min(end + 1, body[key].length); i++)
            if (body[key][i] !== before[key][i]) throw new Error('Cone repair pose changed before application');
    }
    // Commit validates all manifold entries atomically before writing any of
    // them. Prepared poses are finite and there is no asynchronous gap here.
    if (plan.changedContacts) {
        commitKirchhoffCoupledFrictionMultipliers(plan.batch, plan.increment, 1);
        for (const { body, after, start, end } of plan.responses) {
            for (const key of poseFields) for (let i = start; i < Math.min(end + 1, body[key].length); i++) body[key][i] = after[key][i];
            if (body.kirchhoffScratch?.direct) body.kirchhoffScratch.direct.factorAge = Infinity;
        }
    }
    plan.applied = true; plan.status = plan.changedContacts ? 'applied-needs-residual-check' : 'already-feasible';
    return plan;
}
