import { rotateVectorByQuaternion } from './discreteKirchhoffRod.js';
import { evaluateKirchhoffSurfaceFrictionKKT } from './kirchhoffSurfaceFriction.js';

const XYZ = ['X', 'Y', 'Z'];
const bodiesOf = joint => [joint.innerBody, joint.outerBody];
const unequal = body => body.wallStaticFriction !== body.wallKineticFriction;
const clone = value => structuredClone(value);
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);
const length = vector => Math.hypot(...vector);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const materialArrays = ['restLength', 'inverseMass', 'inverseInertia1', 'inverseInertia2', 'inverseInertia3', 'controlEnabled'];
function materialSignature(body, first, last) {
    return { first, last, adaptationCompliance: body.adaptationCompliance,
        orientationControlSegment: body.orientationControlSegment,
        values: [...materialArrays, 'materialCoordinate', 'x', 'y', 'z',
            ...XYZ.flatMap(a => ['velocity' + a, 'angularVelocity' + a]),
            ...[...XYZ, 'W'].map(a => 'orientation' + a)].map(key => body[key] ? [...body[key].slice(first, last + 1)] : null) };
}
function frame(body, segment, inverse = false) {
    return Object.fromEntries([...XYZ, 'W'].map(a => [a.toLowerCase(), body['orientation' + a][segment] * (inverse && a !== 'W' ? -1 : 1)]));
}
function rotated(q, vector) {
    const result = rotateVectorByQuaternion(q, { x: vector[0], y: vector[1], z: vector[2] });
    return [result.x, result.y, result.z];
}
function issue(joint, reason, detail = {}) {
    const state = joint._splitMotion, controller = state?.wallFrictionModes;
    if (state && !state.diagnostics.unverifiedHistoryKinds.includes(reason)) state.diagnostics.unverifiedHistoryKinds.push(reason);
    if (controller && !controller.issues.some(item => item.reason === reason && item.key === detail.key))
        controller.issues.push({ reason, ...detail });
}
function profileValid(body) {
    return Number.isFinite(body.wallStaticFriction) && Number.isFinite(body.wallKineticFriction) &&
        body.wallKineticFriction >= 0 && body.wallStaticFriction >= body.wallKineticFriction;
}
function motionSignature(motion, identity) {
    const { segment, weights } = identity;
    return [...weights.flatMap((weight, i) => weight ? XYZ.map(a => motion['velocity' + a][segment + i]) : []),
        ...XYZ.map(a => motion['angularVelocity' + a][segment])];
}

/** Capture BEFORE force/damping prediction. Predicted velocity must not turn a
 * resting contact under a sub-static applied force into a sliding contact.
 * Equal-coefficient bodies need no controller and retain the original path. */
export function captureKirchhoffWallFrictionIncoming(joint, world) {
    if (!bodiesOf(joint).some(unequal)) return null;
    return { dt: world.fixedDt, step: world.stepCount, bodies: bodiesOf(joint).map(body => {
        const saved = { count: body.count, activeStart: body.activeStart, activeEnd: body.activeEnd,
            adaptationCompliance: body.adaptationCompliance, orientationControlSegment: body.orientationControlSegment };
        for (const key of ['x', 'y', 'z', 'materialCoordinate', 'nodeRadius',
            ...XYZ.flatMap(a => ['velocity' + a, 'angularVelocity' + a]), ...[...XYZ, 'W'].map(a => 'orientation' + a)])
            saved[key] = body[key].slice();
        for (const key of materialArrays) if (body[key]) saved[key] = body[key].slice();
        return saved;
    }) };
}

/** Root owns the physical-phase snapshot/restart. All mechanics/history below
 * is owned by the joint, so the existing trial and whole-step transactions
 * restore it. The caller must pass the existing linear solve tolerance in mm;
 * there is deliberately no contactActivation/stiction-speed default. */
export function initializeKirchhoffWallFrictionModes(joint, incoming, { displacementToleranceMm, coneTolerance = 1e-9 } = {}) {
    if (!bodiesOf(joint).some(unequal)) return null;
    const state = joint._splitMotion;
    if (!state || state.phase !== 'physical' || !incoming || incoming.dt !== state.dt ||
        incoming.bodies.some((body, side) => body.count !== bodiesOf(joint)[side].count))
        throw new Error('Wall friction modes require matching pre-prediction motion and a physical phase');
    if (!(Number.isFinite(displacementToleranceMm) && displacementToleranceMm > 0) ||
        !(Number.isFinite(coneTolerance) && coneTolerance >= 0)) throw new RangeError('Explicit finite friction tolerances required');
    const controller = state.wallFrictionModes = { incoming, dt: state.dt, step: state.step,
        displacementToleranceMm, coneTolerance, records: new Map(), overrides: new Map(), issues: [],
        attempt: 1, maximumAttempts: 1 + bodiesOf(joint).reduce((sum, body) => sum + body.segmentCount + 2 * body.count, 0),
        committed: false, certificate: null };
    for (const [side, body] of bodiesOf(joint).entries()) if (!profileValid(body)) issue(joint, 'wall-friction-invalid-profile', { side });
    return controller;
}

/** A wall slot alone is not material contact identity. A loaded change in the
 * foot, material labels, normal or collider feature needs mechanical release;
 * this bounded controller guards it rather than transferring the old force.
 * Tangential world-point motion on the SAME flat face is not a new identity. */
function witness(entry, body) {
    const record = entry.record, normal = entry.normalRow, segment = record._innerSegmentIndex;
    const wall = normal.kind === 'wall' ? { branchId: body.wallBranchId[normal.node], faceIndex: body.wallFaceIndex[normal.node],
        planeOffset: dot(record.normal, XYZ.map(a => body['wall' + a][normal.node])) } : normal.wallFrictionWitness;
    return { kind: normal.kind, side: entry.side, node: normal.node, segment,
        weights: [...record.innerWeights], material: [body.materialCoordinate[segment], body.materialCoordinate[segment + 1]],
        radius: Math.max(body.nodeRadius[segment], body.nodeRadius[segment + 1]),
        normal: [...record.normal], branch: wall?.branchId, face: wall?.faceIndex, plane: wall?.planeOffset };
}
function incomingSurfaceMotion(entry, incoming, body) {
    const record = entry.record, segment = record._innerSegmentIndex;
    const velocity = XYZ.map(a => record.innerWeights.reduce((sum, weight, i) => sum + weight * incoming['velocity' + a][segment + i], 0));
    // The same material surface point at t_n: R_n R_current^T r_current.
    const center = XYZ.map(a => record.innerWeights.reduce((sum, weight, i) => sum + weight * body[a.toLowerCase()][segment + i], 0));
    const lever = record.surfaceContactPoint.map((value, i) => value - center[i]);
    const startLever = rotated(frame(incoming, segment), rotated(frame(body, segment, true), lever));
    const spin = cross(XYZ.map(a => incoming['angularVelocity' + a][segment]), startLever);
    const result = velocity.map((value, i) => value + spin[i]), n = record.normal;
    const normalMotion = dot(result, n) / dot(n, n);
    return result.map((value, i) => value - normalMotion * n[i]);
}

/** Called only by the wall builder after constructing its shared-point
 * geometry. It selects an exact profile coefficient, never a blended value. */
export function prepareKirchhoffWallFrictionEntry(joint, entry) {
    const body = bodiesOf(joint)[entry.side], state = joint._splitMotion, controller = state?.wallFrictionModes;
    entry.modeRecord = null;
    if (!unequal(body)) return body.wallStaticFriction;
    if (!controller) { issue(joint, 'unequal-wall-static-kinetic'); return body.wallStaticFriction; }
    if (!profileValid(body)) { issue(joint, 'wall-friction-invalid-profile', { side: entry.side }); return body.wallStaticFriction; }
    const key = entry.contact.key, identity = witness(entry, body), incoming = controller.incoming.bodies[entry.side];
    if (![...identity.weights, ...identity.material, identity.radius, ...identity.normal, identity.plane, identity.branch, identity.face].every(Number.isFinite))
        issue(joint, 'wall-friction-witness-history-missing', { key });
    if (identity.segment < incoming.activeStart || identity.segment + 1 > incoming.activeEnd ||
        identity.material.some((value, i) => value !== incoming.materialCoordinate[identity.segment + i]) ||
        identity.radius !== Math.max(incoming.nodeRadius[identity.segment], incoming.nodeRadius[identity.segment + 1]))
        issue(joint, 'wall-friction-incoming-material-changed', { key });
    let record = controller.records.get(key);
    if (record && !same(record.identity, identity)) {
        if (record.maximumReaction > 0 || entry.contact.normalLambda !== 0 || entry.contact.tangentLambda.some(value => value !== 0))
            issue(joint, 'wall-friction-loaded-witness-changed', { key, before: record.identity, after: identity });
        else record = null;
    }
    if (!record) {
        const old = joint._wallFrictionHistory?.records.get(key), signature = motionSignature(incoming, identity);
        const materialHistory = old?.materialSignature;
        const priorStop = old?.stopped && same(old.identity, identity) && same(old.motionSignature, signature) &&
            (!materialHistory || same(materialHistory, materialSignature(incoming, materialHistory.first, materialHistory.last)));
        const startMotion = incomingSurfaceMotion(entry, incoming, body);
        let mode = priorStop || startMotion.every(value => value === 0) ? 'stick' : 'slide';
        const override = controller.overrides.get(key);
        if (override) {
            if (!same(override.identity, identity)) issue(joint, 'wall-friction-retry-witness-changed', { key });
            mode = 'slide';
        }
        record = { key, identity, mode, startMotion, incomingCertificate: priorStop ? 'committed-stop' :
            startMotion.every(value => value === 0) ? 'exact-zero' : 'nonzero-incoming',
            muStatic: body.wallStaticFriction, muKinetic: body.wallKineticFriction, maximumReaction: 0,
            appliedCount: 0, lastAppliedScale: 0, lastNormalIncrement: 0, lastTangentIncrement: [0, 0] };
        controller.records.set(key, record);
    }
    if (record.muStatic !== body.wallStaticFriction || record.muKinetic !== body.wallKineticFriction)
        issue(joint, 'wall-friction-profile-changed', { key });
    entry.modeRecord = record;
    return record.mode === 'stick' ? record.muStatic : record.muKinetic;
}

/** Applied history is recorded before any later collector can erase a load.
 * Rejected scales are restored by the same snapshot as lambda and velocity. */
export function recordKirchhoffWallFrictionApplication(entry, normalIncrement, tangentIncrement, scale) {
    const record = entry.modeRecord;
    if (!record) return;
    const values = [entry.baseNormalLambda, entry.contact.normalLambda, normalIncrement,
        ...entry.rows.map(row => row.lambda), ...entry.lambda, ...tangentIncrement];
    if (!values.every(Number.isFinite)) throw new RangeError('Nonfinite wall friction reaction history');
    record.maximumReaction = Math.max(record.maximumReaction, ...values.map(Math.abs));
    record.appliedCount++;
    record.lastAppliedScale = scale; record.lastNormalIncrement = normalIncrement; record.lastTangentIncrement = [...tangentIncrement];
}

function actualContact(joint, entry) {
    const motion = joint._splitMotion.bodies[entry.side], r = entry.record, segment = r._innerSegmentIndex;
    const velocity = XYZ.map(a => r.innerWeights.reduce((sum, weight, i) => sum + weight * motion['velocity' + a][segment + i], 0));
    const spin = cross(XYZ.map(a => motion['angularVelocity' + a][segment]), entry.surface.levers[0]);
    const u = velocity.map((value, i) => value + spin[i]);
    const worldLambda = XYZ.map((_, i) => entry.contact.tangentU[i] * entry.contact.tangentLambda[0] + entry.contact.tangentV[i] * entry.contact.tangentLambda[1]);
    const magnitude = XYZ.map((a, axis) => r.innerWeights.reduce((sum, weight, i) => sum + Math.abs(weight * motion['velocity' + a][segment + i]), 0) +
        Math.abs(motion['angularVelocity' + XYZ[(axis + 1) % 3]][segment] * entry.surface.levers[0][(axis + 2) % 3]) +
        Math.abs(motion['angularVelocity' + XYZ[(axis + 2) % 3]][segment] * entry.surface.levers[0][(axis + 1) % 3]));
    return { displacement: entry.surface.axes.map(axis => joint._splitMotion.dt * dot(axis, u)),
        lambda: entry.surface.axes.map(axis => dot(axis, worldLambda)), normalLambda: entry.contact.normalLambda,
        arithmeticMagnitudeMm: entry.surface.axes.map(axis => joint._splitMotion.dt * dot(axis.map(Math.abs), magnitude)) };
}

function basisCoefficients(axis, axes) {
    for (let i = 0; i < 2; i++) for (const sign of [1, -1])
        if (axis.every((value, j) => value === sign * axes[i][j])) return i ? [0, sign] : [sign, 0];
    return axes.map(source => dot(axis, source));
}
function rowCombination(entry, coefficients) {
    const result = new Map();
    for (let axis = 0; axis < 2; axis++) if (coefficients[axis] !== 0)
        for (const gradient of entry.surface.rows[axis].gradients) if (gradient.side === 0)
            result.set(gradient.dof, (result.get(gradient.dof) ?? 0) + coefficients[axis] * gradient.value);
    return result;
}
function fixedZeroMotion(body, motion, dof) {
    const node = Math.floor(dof / 6), axis = dof % 6;
    if (axis < 3) return body.inverseMass[node] === 0 && motion['velocity' + XYZ[axis]][node] === 0;
    const prescribed = body.orientationControlCompliance === 0 && node === body.orientationControlSegment;
    if (!prescribed && body['inverseInertia' + (axis - 2)][node] !== 0) return false;
    const local = rotated(frame(body, node, true), XYZ.map(a => motion['angularVelocity' + a][node]));
    return local[axis - 3] === 0;
}

/** A redundant zero-load witness may inherit a kinematic zero certificate,
 * never a force, from a CURRENT loaded stop. Its two tangent functionals must
 * be identical on every mobile DOF. Any unmatched prescribed term must have
 * exactly zero known physical rate. This is intentionally narrower than a
 * general material-row-space proof and uses no velocity epsilon. */
function equivalentStop(joint, target, support, targetEvidence, supportEvidence) {
    const a = targetEvidence.identity, b = supportEvidence.identity;
    if (a.side !== b.side || a.branch !== b.branch || a.face !== b.face || a.plane !== b.plane || !same(a.normal, b.normal)) return null;
    const body = bodiesOf(joint)[a.side], motion = joint._splitMotion.bodies[a.side], coefficients = [], roundoff = [];
    for (let axis = 0; axis < 2; axis++) {
        const combine = basisCoefficients(target.surface.axes[axis], support.surface.axes);
        const difference = rowCombination(target, axis ? [0, 1] : [1, 0]);
        for (const [dof, value] of rowCombination(support, combine)) difference.set(dof, (difference.get(dof) ?? 0) - value);
        for (const [dof, value] of difference) if (value !== 0 && !fixedZeroMotion(body, motion, dof)) return null;
        const expected = dot(combine, supportEvidence.displacement);
        // A roundoff bound from the actual dot-product term magnitudes, not a
        // free-standing small-speed threshold. The source is already stopped.
        const error = 64 * Number.EPSILON * (targetEvidence.arithmeticMagnitudeMm[axis] +
            dot(combine.map(Math.abs), supportEvidence.arithmeticMagnitudeMm));
        if (Math.abs(targetEvidence.displacement[axis] - expected) > error) return null;
        coefficients.push(combine); roundoff.push(error);
    }
    return { supportKey: supportEvidence.key, sourceCertificate: supportEvidence.stopCertificate,
        freeJacobianDifferenceIsZero: true, prescribedDifferenceRateIsZero: true,
        coefficients, roundoffBoundMm: roundoff };
}

/** A dual gauge certificate for one affine, native hard-material chain.
 * Tangent reactions can be redistributed and balanced by unconstrained hard
 * material multipliers without changing the primal correction. The witness
 * is owned data only: no published force or motion is changed. Unlike a
 * homogeneous-rate proof, this does NOT require material C to be zero:
 * alpha * deltaLambda = 0 preserves each ORIGINAL constitutive equation. */
function staticDualCertificate(joint, entries, contacts) {
    const side = contacts[0].identity.side, body = bodiesOf(joint)[side], state = joint._splitMotion;
    const controller = state.wallFrictionModes, incoming = controller.incoming.bodies[side], motion = state.bodies[side];
    const native = body.kirchhoffScratch?.direct, first = body.activeStart, last = body.activeEnd;
    if (!native || body.sleeping || body.adaptationCompliance !== 0 || incoming.adaptationCompliance !== 0 ||
        first !== incoming.activeStart || last !== incoming.activeEnd || native.start !== first || native.end !== last ||
        native.dt !== state.dt || !state.start?.[side] || !incoming.restLength) return null;
    const reference = contacts[0].identity, mu = body.wallStaticFriction;
    if (!contacts.every(c => c.mode === 'stick' && c.identity.side === side && c.identity.branch === reference.branch &&
        c.identity.face === reference.face && c.identity.plane === reference.plane && same(c.identity.normal, reference.normal) &&
        Number.isFinite(c.normalLambda) && c.normalLambda >= 0 &&
        length(c.displacement) <= controller.displacementToleranceMm)) return null;
    for (let node = first; node <= last; node++) {
        if (!(body.inverseMass[node] > 0) || body.inverseMass[node] !== incoming.inverseMass?.[node] ||
            body.controlEnabled?.[node] || incoming.controlEnabled?.[node] ||
            body.materialCoordinate[node] !== incoming.materialCoordinate[node] ||
            XYZ.some(axis => incoming[axis.toLowerCase()][node] !== state.start[side][axis][node])) return null;
        if (node === last) continue;
        if (body.orientationControlSegment === node || incoming.orientationControlSegment === node ||
            body.restLength[node] !== incoming.restLength[node] || !(body.restLength[node] > 0) || !Number.isFinite(body.restLength[node])) return null;
        for (const k of [1, 2, 3]) if (body['inverseInertia' + k][node] !== 0 || incoming['inverseInertia' + k]?.[node] !== 0) return null;
        if (XYZ.some(axis => motion['angularVelocity' + axis][node] !== 0 || incoming['angularVelocity' + axis][node] !== 0) ||
            [...XYZ, 'W'].some(axis => body['orientation' + axis][node] !== incoming['orientation' + axis][node] ||
                incoming['orientation' + axis][node] !== state.start[side]['orientation' + axis][node])) return null;
        if (dot(Object.values(frame(body, node)), Object.values(frame(body, node))) !== 1) return null;
        for (let axis = 0; axis < 3; axis++) if (native.alpha[(node - first) * 6 + 3 + axis] !== 0) return null;
    }
    const normal = contacts.reduce((sum, c) => sum + c.normalLambda, 0), total = [0, 0, 0];
    for (const c of contacts) {
        const entry = entries.get(c.key);
        for (let axis = 0; axis < 2; axis++) for (let k = 0; k < 3; k++) total[k] += c.lambda[axis] * entry.surface.axes[axis][k];
    }
    if (!(normal > 0) || !(length(total) < mu * normal * (1 - controller.coneTolerance))) return null;
    const lastLoaded = contacts.findLastIndex(c => c.normalLambda > 0), allocated = [0, 0, 0];
    const load = new Float64Array(body.count * 6), magnitude = new Float64Array(load.length);
    const alternate = [], material = [], delta = new Float64Array(native.rowCount), deltaMagnitude = new Float64Array(native.rowCount);
    let maximumTangentResidualMm = 0, maximumConeViolation = 0;
    for (let index = 0; index < contacts.length; index++) {
        const c = contacts[index], entry = entries.get(c.key);
        const world = total.map((v, k) => index === lastLoaded ? v - allocated[k] : v * (c.normalLambda / normal));
        world.forEach((v, k) => { allocated[k] += v; });
        const lambda = entry.surface.axes.map(axis => dot(axis, world));
        const residual = evaluateKirchhoffSurfaceFrictionKKT(lambda, c.displacement, c.normalLambda, [mu, mu]);
        if (residual.residualMm > controller.displacementToleranceMm || residual.coneViolation > controller.coneTolerance ||
            c.normalLambda > 0 && !(length(lambda) < mu * c.normalLambda * (1 - controller.coneTolerance))) return null;
        maximumTangentResidualMm = Math.max(maximumTangentResidualMm, length(c.displacement), residual.residualMm);
        maximumConeViolation = Math.max(maximumConeViolation, residual.coneViolation);
        for (let axis = 0; axis < 2; axis++) for (const g of entry.surface.rows[axis].gradients) if (g.side === 0) {
            const value = g.value * (lambda[axis] - c.lambda[axis]);
            load[g.dof] += value; magnitude[g.dof] += Math.abs(g.value * lambda[axis]) + Math.abs(g.value * c.lambda[axis]);
        }
        alternate.push({ key: c.key, normalLambda: c.normalLambda, lambda, worldTangent: world, originalLambda: [...c.lambda] });
    }
    // Construct the balancing hard-material dual by exact chain incidence.
    const prefix = [0, 0, 0], prefixMagnitude = [0, 0, 0];
    for (let node = first; node < last; node++) for (let axis = 0; axis < 3; axis++) {
        prefix[axis] += load[node * 6 + axis];
        prefixMagnitude[axis] += magnitude[node * 6 + axis];
        const row = (node - first) * 6 + 3 + axis;
        delta[row] = prefix[axis]; deltaMagnitude[row] = prefixMagnitude[axis];
        material.push({ row, segment: node, axis, deltaLambda: delta[row], alphaDeltaLambda: native.alpha[row] * delta[row] });
    }
    // Independently multiply the ACTUAL native sparse J^T by the proposed
    // material dual. Do not verify with the prefix construction itself.
    for (let dof = first * 6; dof <= last * 6 + 2; dof++) for (let slot = 0; slot < native.degree[dof]; slot++) {
        const k = dof * native.degreeCapacity + slot, value = native.gradients[k] * delta[native.rows[k]];
        load[dof] += value; magnitude[dof] += Math.abs(native.gradients[k]) * deltaMagnitude[native.rows[k]];
    }
    // The native mobile Jacobian omits prescribed rotations. Recover their
    // analytic adaptation torque only for the owned support-reaction witness:
    // dC/dtheta_local = [L*d2, -L*d1, 0]. Their known angular rate is zero.
    for (let segment = first; segment < last; segment++) {
        const q = frame(body, segment), d1 = rotated(q, [1, 0, 0]), d2 = rotated(q, [0, 1, 0]);
        for (let axis = 0; axis < 3; axis++) {
            const value = body.restLength[segment] * delta[(segment - first) * 6 + 3 + axis];
            load[segment * 6 + 3] += d2[axis] * value;
            load[segment * 6 + 4] -= d1[axis] * value;
        }
    }
    let maximumMobileBalance = 0, maximumCorrectionChangeMm = 0;
    const arithmeticFactor = 64 * Number.EPSILON * (contacts.length + last - first + 1);
    const supportTorque = [], mobileBalance = [];
    for (let dof = first * 6; dof <= last * 6 + 2; dof++) {
        const axis = dof % 6, node = Math.floor(dof / 6);
        if (axis >= 3) { if (load[dof] !== 0) supportTorque.push({ segment: node, materialAxis: axis - 3, deltaReaction: -load[dof] }); continue; }
        if (!Number.isFinite(load[dof]) || Math.abs(load[dof]) > arithmeticFactor * magnitude[dof]) return null;
        maximumMobileBalance = Math.max(maximumMobileBalance, Math.abs(load[dof]));
        mobileBalance.push({ dof, residual: load[dof], arithmeticBound: arithmeticFactor * magnitude[dof] });
        maximumCorrectionChangeMm = Math.max(maximumCorrectionChangeMm, body.inverseMass[node] * Math.abs(load[dof]));
    }
    if (maximumCorrectionChangeMm > controller.displacementToleranceMm || material.some(row => row.alphaDeltaLambda !== 0)) return null;
    return { kind: 'native-hard-material-static-dual', first, last, side, nativeGradientVerification: true,
        prerequisite: 'converged-whole-physical-closure',
        totalNormalLambda: normal, totalWorldTangent: total, staticCapacity: mu * normal,
        alternateContacts: alternate, materialDelta: material, fixedSupportTorqueDelta: supportTorque,
        maximumMobileBalance, mobileBalance, maximumCorrectionChangeMm, maximumTangentResidualMm, maximumConeViolation,
        normalMultipliersUnchanged: true, originalConstitutiveEquationsUnchanged: true,
        originalPrimalAndPublishedReactionsUnchanged: true };
}

function evaluate(joint, batch, converged, final) {
    const state = joint._splitMotion, controller = state?.wallFrictionModes;
    const base = { status: 'accepted', accepted: true, restart: false, step: state?.step, dt: state?.dt, contacts: [], issues: [] };
    if (!controller) {
        if (batch?.entries.some(entry => unequal(bodiesOf(joint)[entry.side]))) {
            issue(joint, 'unequal-wall-static-kinetic'); return { ...base, status: 'unsupported', accepted: false, issues: ['unequal-wall-static-kinetic'] };
        }
        return base;
    }
    Object.assign(base, { attempt: controller.attempt, maximumAttempts: controller.maximumAttempts });
    if (!converged) return { ...base, status: 'unconverged', accepted: false };
    if (!batch || batch.joint !== joint) return { ...base, status: 'unsupported', accepted: false, issues: ['wall-friction-fresh-batch-missing'] };
    if (controller.issues.length) return { ...base, status: 'unsupported', accepted: false, issues: clone(controller.issues) };
    const overrides = new Map(controller.overrides), seen = new Set(), entries = new Map();
    let ambiguous = false, unresolved = false;
    for (const entry of batch.entries) {
        if (!unequal(bodiesOf(joint)[entry.side])) continue;
        const record = entry.modeRecord;
        if (!record || controller.records.get(record.key) !== record) {
            issue(joint, 'wall-friction-mode-history-missing', { key: entry.contact.key }); continue;
        }
        seen.add(record.key);
        entries.set(record.key, entry);
        const { displacement, lambda, normalLambda, arithmeticMagnitudeMm } = actualContact(joint, entry);
        const mu = record.mode === 'stick' ? record.muStatic : record.muKinetic;
        const residual = evaluateKirchhoffSurfaceFrictionKKT(lambda, displacement, normalLambda, [mu, mu]);
        const radius = mu * normalLambda, slip = length(displacement), force = length(lambda);
        const interior = radius > 0 && force < radius * (1 - controller.coneTolerance);
        const valid = residual.residualMm <= controller.displacementToleranceMm && residual.coneViolation <= controller.coneTolerance;
        const stopped = valid && (slip === 0 || normalLambda > 0 && interior && slip <= controller.displacementToleranceMm);
        const evidence = { key: record.key, identity: clone(record.identity), mode: record.mode,
            muStatic: record.muStatic, muKinetic: record.muKinetic, muApplied: mu,
            normalLambda, lambda, displacement, arithmeticMagnitudeMm, residualMm: residual.residualMm, coneViolation: residual.coneViolation,
            stopped, stopCertificate: stopped ? (slip === 0 ? 'exact-zero' : 'interior-cone-and-solver-residual') : null,
            maximumReaction: record.maximumReaction };
        base.contacts.push(evidence);
        if (!valid) { unresolved = true; continue; }
        if (normalLambda === 0) continue;
    }
    // A full physical closure is supplied by the caller. This additional
    // witness verifies the changed dual blocks and explicit static tangent
    // equalities; all untouched rows retain their original full-KKT evidence.
    for (const side of [0, 1]) {
        const contacts = base.contacts.filter(c => c.identity.side === side);
        if (!contacts.length || !contacts.some(c => !c.stopped)) continue;
        const proof = staticDualCertificate(joint, entries, contacts);
        if (!proof) continue;
        (base.staticDualCertificates ??= []).push(proof);
        for (const contact of contacts) {
            contact.stopped = true; contact.stopCertificate = 'native-hard-material-static-dual';
            contact.staticDualProof = { side, first: proof.first, last: proof.last };
        }
    }
    const loadedStops = base.contacts.filter(contact => contact.normalLambda > 0 && contact.stopped);
    for (const contact of base.contacts) if (contact.normalLambda === 0 && !contact.stopped && contact.lambda.every(value => value === 0)) {
        for (const support of loadedStops) {
            const proof = equivalentStop(joint, entries.get(contact.key), entries.get(support.key), contact, support);
            if (!proof) continue;
            contact.stopped = true; contact.stopCertificate = 'kinematic-equivalence-to-loaded-stop';
            contact.kinematicStopProof = proof;
            break;
        }
    }
    for (const contact of base.contacts) if (contact.normalLambda > 0 && !contact.stopped) {
        const slip = length(contact.displacement), radius = contact.muApplied * contact.normalLambda;
        if (contact.mode === 'stick') {
            if (slip > controller.displacementToleranceMm)
                overrides.set(contact.key, { key: contact.key, identity: clone(contact.identity), mode: 'slide' });
            else ambiguous = true;
        } else if (radius > 0 && slip <= controller.displacementToleranceMm) ambiguous = true;
    }
    for (const record of controller.records.values()) if (!seen.has(record.key) && record.maximumReaction > 0)
        issue(joint, 'wall-friction-loaded-witness-missing', { key: record.key });
    if (controller.issues.length) return { ...base, status: 'unsupported', accepted: false, issues: clone(controller.issues) };
    if (unresolved) return { ...base, status: 'unconverged', accepted: false };
    if (ambiguous) return { ...base, status: 'ambiguous', accepted: false, issues: ['wall-friction-mode-ambiguous'] };
    if (overrides.size > controller.overrides.size) return { ...base, status: final ? 'unsupported' : 'restart',
        accepted: false, restart: !final, overrides: [...overrides.values()], attempt: controller.attempt + 1,
        issues: final ? ['wall-friction-static-slip-final'] : [] };
    return base;
}

/** Call only after a fresh, converged WHOLE physical closure. A failed linear
 * or nonlinear solve is never evidence of static breakaway. Returned restart
 * plans own all values and survive restoring the physical-phase snapshot. */
export function evaluateKirchhoffWallFrictionCandidate(joint, freshWallBatch, { converged = false } = {}) {
    return evaluate(joint, freshWallBatch, converged, false);
}

/** AFTER root restores the same phase base, reinstall all monotone demotions.
 * Trials never clip old static multipliers or apply a second force kick. */
export function prepareKirchhoffWallFrictionRetry(joint, decision) {
    const controller = joint._splitMotion?.wallFrictionModes;
    if (!controller || decision.status !== 'restart' || decision.step !== controller.step || decision.dt !== controller.dt ||
        !Number.isInteger(decision.attempt) || decision.attempt < 2 || decision.attempt > controller.maximumAttempts ||
        decision.overrides.length < decision.attempt - 1 || new Set(decision.overrides.map(item => item.key)).size !== decision.overrides.length)
        throw new Error('Invalid or exhausted wall-friction restart');
    controller.overrides = new Map(decision.overrides.map(item => [item.key, clone(item)]));
    controller.attempt = decision.attempt; controller.certificate = null;
    for (const record of controller.records.values()) {
        const override = controller.overrides.get(record.key);
        if (!override) continue;
        if (!same(override.identity, record.identity)) issue(joint, 'wall-friction-retry-witness-changed', { key: record.key });
        record.mode = 'slide';
    }
    return controller;
}

export function certifyKirchhoffWallFrictionModes(joint, freshWallBatch, { converged = false } = {}) {
    const result = evaluate(joint, freshWallBatch, converged, true), controller = joint._splitMotion?.wallFrictionModes;
    if (controller) controller.certificate = result;
    if (!result.accepted) for (const item of result.issues.length ? result.issues : ['wall-friction-final-unconverged'])
        issue(joint, typeof item === 'string' ? item : item.reason, typeof item === 'string' ? {} : item);
    return result;
}

/** Call inside the accepted final split commit. This persists mode evidence,
 * never a normal/tangent multiplier. Whole-step rollback owns this map too. */
export function commitKirchhoffWallFrictionHistory(joint, certificate) {
    const state = joint._splitMotion, controller = state?.wallFrictionModes;
    if (!controller) return false;
    if (state.phase !== 'complete' || !state.diagnostics.certified || controller.committed ||
        certificate !== controller.certificate || !certificate.accepted || certificate.step !== state.step)
        throw new Error('Wall-friction history requires one accepted fresh final certificate');
    const records = new Map();
    for (const contact of certificate.contacts) if (contact.normalLambda > 0 || contact.stopped) records.set(contact.key, {
        identity: clone(contact.identity), mode: contact.stopped ? 'stick' : 'slide', stopped: contact.stopped,
        stopCertificate: contact.stopCertificate, kinematicStopProof: clone(contact.kinematicStopProof ?? null),
        materialSignature: contact.staticDualProof ? materialSignature(bodiesOf(joint)[contact.identity.side],
            contact.staticDualProof.first, contact.staticDualProof.last) : null,
        motionSignature: motionSignature(bodiesOf(joint)[contact.identity.side], contact.identity)
    });
    joint._wallFrictionHistory = { step: state.step, dt: state.dt, records };
    controller.committed = true;
    return true;
}
