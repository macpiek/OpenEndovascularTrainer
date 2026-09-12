import { buildKirchhoffSurfaceFriction, measureKirchhoffSurfaceFrictionState, materializeKirchhoffSurfaceFrictionGradients, refreshKirchhoffSurfaceFrictionLoads, beginKirchhoffSurfaceEvaluation, evaluateKirchhoffSurfaceFriction, evaluateKirchhoffSurfaceFrictionKKT } from './kirchhoffSurfaceFriction.js';
import { updateKirchhoffFrictionInputStamp } from './kirchhoffFrictionInputStamp.js';

const EPSILON = 1e-12;
const XYZ = ['x', 'y', 'z'];
const KINDS = new Set(['side', 'material-side', 'distal-fillet', 'distal-rim', 'sliding-rim']);
const vector = () => new Float64Array(3);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function finite(value, label) {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return value;
}

function readVector(source, out, label) {
    for (let i = 0; i < 3; i++) out[i] = finite(source?.[i] ?? source?.[XYZ[i]], label);
    return out;
}

function normalize(v, label) {
    const length = Math.hypot(...v);
    if (length <= EPSILON) throw new RangeError(`${label} must have positive length`);
    for (let i = 0; i < 3; i++) v[i] /= length;
    return v;
}

function cross(a, b, out) {
    const x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2];
    const z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
    return out;
}

function segmentIndex(record, side) {
    return record[side === 0 ? '_innerSegmentIndex' : '_outerSegmentIndex'] ??
        record.manifoldContact?.[side === 0 ? 'innerSegmentIndex' : 'outerSegmentIndex'];
}

function pointAtStencil(body, record, side, out, stencil) {
    const prefix = side === 0 ? '_inner' : '_outer';
    const nodes = record[prefix + 'NodeIndices'];
    const weights = nodes ? record[prefix + 'NodeWeights'] : record[side === 0 ? 'innerWeights' : 'outerWeights'];
    const count = nodes ? record[prefix + 'NodeCount'] : 2;
    const segment = segmentIndex(record, side);
    if (!Number.isInteger(segment) || segment < 0 || segment + 1 >= body.count ||
        !Number.isInteger(count) || count < 1 || count > body.count || !weights || weights.length < count ||
        nodes && nodes.length < count) throw new RangeError('Invalid contact stencil');
    stencil.segment = segment; stencil.count = count;
    stencil.nodes.length = stencil.weights.length = count;
    out.fill(0);
    let sum = 0;
    for (let i = 0; i < count; i++) {
        const node = nodes ? nodes[i] : segment + i, weight = finite(weights[i], 'contact weight');
        if (!Number.isInteger(node) || node < 0 || node >= body.count) throw new RangeError('Invalid contact node');
        stencil.nodes[i] = node; stencil.weights[i] = weight;
        sum += weight;
        for (let axis = 0; axis < 3; axis++) out[axis] += weight * finite(body[XYZ[axis]][node], 'contact position');
    }
    if (Math.abs(sum - 1) > 1e-8) throw new RangeError('Contact stencil must preserve partition of unity');
    return out;
}

function tangentAtStencil(body, record, out) {
    const nodes = record._outerNodeIndices, count = record._outerNodeCount;
    const segment = segmentIndex(record, 1);
    if (nodes && count === 4) {
        // Exact derivative of the uniform cubic B-spline in the current world.
        // Generalized/custom stencils must supply their own physical tangent.
        const t = finite(record.outerT, 'outerT'), t2 = t * t, t3 = t2 * t;
        const expected = [(1 - 3 * t + 3 * t2 - t3) / 6,
            (4 - 6 * t2 + 3 * t3) / 6, (1 + 3 * t + 3 * t2 - 3 * t3) / 6, t3 / 6];
        const derivative = [(-3 + 6 * t - 3 * t2) / 6, (-12 * t + 9 * t2) / 6,
            (3 + 6 * t - 9 * t2) / 6, t2 / 2];
        if (t < 0 || t > 1 || expected.some((value, i) =>
            nodes[i] !== segment - 1 + i || Math.abs(value - record._outerNodeWeights[i]) > 1e-8)) {
            throw new RangeError('Custom smooth stencil requires surfaceAxialTangent');
        }
        out.fill(0);
        for (let i = 0; i < 4; i++) for (let axis = 0; axis < 3; axis++) out[axis] += body[XYZ[axis]][nodes[i]] * derivative[i];
    } else {
        if (nodes && count !== 2) throw new RangeError('Custom smooth stencil requires surfaceAxialTangent');
        for (let axis = 0; axis < 3; axis++) out[axis] = body[XYZ[axis]][segment + 1] - body[XYZ[axis]][segment];
    }
    return normalize(out, 'outer tangent');
}

function geometryStorage(out) {
    if (out.point) return out;
    for (const key of ['point', 'normal', 'axialTangent', 'outerAxis', 'radial', 'innerCenter',
        'outerCenter', 'innerWitness', 'outerWitness', 'normalMomentResidual']) out[key] = vector();
    out.stencils = [0, 1].map(() => ({ segment: 0, count: 0, nodes: [], weights: [] }));
    return out;
}

/** Reconstruct a shared witness for the EXISTING effective gap model.
 * Does not alter collision shape, normal, gap, stencil or material registration.
 * The inner witness is c_i+r*n. The zero-gap centre locus is c_i+gap*n;
 * offsetting that by r*n supplies the outer effective witness. Their midpoint
 * is common even during penetration. For distal-fillet this is an effective
 * centreline-model witness, NOT a claim to recover a manufactured torus.
 *
 * Fillet U is the continuous meridional tangent n_radial*a-n_axial*e, including
 * the pole where projecting the shaft axis alone is degenerate. At a spherical
 * sliding-aperture pole, the crossing wire tangent supplies a physical sliding
 * direction. Explicit surfaceContactPoint/surfaceAxialTangent take precedence.
 * Geometry is borrowed until the next call with this out; source is read-only.
 */
export function prepareKirchhoffCoupledSurfaceGeometry(constraint, record, out = {}) {
    geometryStorage(out);
    if (!KINDS.has(record.kind) && !record.surfaceContactPoint) throw new RangeError(`Unknown surface feature ${record.kind}`);
    const inner = constraint.innerBody, outer = constraint.outerBody;
    pointAtStencil(inner, record, 0, out.innerCenter, out.stencils[0]);
    pointAtStencil(outer, record, 1, out.outerCenter, out.stencils[1]);
    normalize(readVector(record.normal, out.normal, 'normal'), 'normal');
    const segment = segmentIndex(record, 0);
    const radius = Math.max(finite(inner.nodeRadius?.[segment] ?? inner.radius, 'wire radius'),
        finite(inner.nodeRadius?.[segment + 1] ?? inner.radius, 'wire radius'));
    if (radius < 0) throw new RangeError('Wire radius must be non-negative');
    const gap = finite(constraint.surfaceMotion ? record._splitActualGap ?? record.gap : record.gap, 'contact gap');
    out.kind = record.kind; out.gap = gap; out.wireRadius = radius;
    out.effectiveFillet = record.kind === 'distal-fillet';
    out.pointSource = record.surfaceContactPoint ? 'provided' : 'effective-gap-witness';
    out.physicalSurfaceVerified = record.surfaceGeometryVerified === true;
    for (let axis = 0; axis < 3; axis++) {
        out.innerWitness[axis] = out.innerCenter[axis] + radius * out.normal[axis];
        out.outerWitness[axis] = out.innerCenter[axis] + (radius + gap) * out.normal[axis];
        out.point[axis] = (out.innerWitness[axis] + out.outerWitness[axis]) * 0.5;
    }
    if (record.surfaceContactPoint) readVector(record.surfaceContactPoint, out.point, 'surfaceContactPoint');
    if (record.surfaceAxialTangent) {
        normalize(readVector(record.surfaceAxialTangent, out.axialTangent, 'surfaceAxialTangent'), 'surfaceAxialTangent');
        out.tangentSource = 'provided';
    } else {
        tangentAtStencil(outer, record, out.outerAxis);
        const axialNormal = dot(out.normal, out.outerAxis);
        if (record.kind === 'distal-fillet') {
            // Normal already stores the effective rounded-lip azimuth. This
            // remains defined at rho=0 whenever the emitted normal is radial.
            for (let i = 0; i < 3; i++) out.radial[i] = out.normal[i] - axialNormal * out.outerAxis[i];
            const radialNormal = Math.hypot(...out.radial);
            if (radialNormal > EPSILON) for (let i = 0; i < 3; i++) out.radial[i] /= radialNormal;
            else {
                let axial = 0;
                for (let i = 0; i < 3; i++) axial += (out.innerCenter[i] - out.outerCenter[i]) * out.outerAxis[i];
                for (let i = 0; i < 3; i++) out.radial[i] = out.innerCenter[i] - out.outerCenter[i] - axial * out.outerAxis[i];
                normalize(out.radial, 'fillet azimuth');
            }
            for (let i = 0; i < 3; i++) out.axialTangent[i] = radialNormal * out.outerAxis[i] - axialNormal * out.radial[i];
            normalize(out.axialTangent, 'fillet meridian');
            out.tangentSource = 'effective-fillet-meridian';
        } else {
            out.axialTangent.set(out.outerAxis);
            out.tangentSource = record._outerNodeCount === 4 ? 'uniform-bspline-derivative' : 'outer-segment';
            if (1 - axialNormal * axialNormal < 1e-12) {
                if (record.kind !== 'sliding-rim') throw new RangeError('Degenerate surface tangent requires explicit surfaceAxialTangent');
                for (let i = 0; i < 3; i++) out.axialTangent[i] = inner[XYZ[i]][segment + 1] - inner[XYZ[i]][segment];
                normalize(out.axialTangent, 'crossing wire tangent');
                out.tangentSource = 'wire-at-aperture-pole';
            }
        }
    }
    // Normal block uses -n on inner, +n on outer. A nonzero result flags the
    // missing normal lever/tangent-gradient independently of friction rows.
    for (let i = 0; i < 3; i++) out.radial[i] = out.outerCenter[i] - out.innerCenter[i];
    cross(out.radial, out.normal, out.normalMomentResidual);
    return out;
}

function batchStorage(out) {
    out.rows ??= []; out.groups ??= []; out.entries ??= []; out.skipped ??= [];
    out._pool ??= []; out._commitValues ??= [];
    return out;
}

function makeEntry() {
    return { geometry: {}, surface: {}, view: null, lambda: new Float64Array(2),
        increment: new Float64Array(2), force: vector(), innerMoment: vector(), outerMoment: vector(),
        appendedGroup: { rowIndices: new Uint32Array(2) }, residual: {} };
}

/** Build ALL existing lumen contact features into one additional-row batch.
 * Groups are fixed at each contact's current normalLambda; rebuild each outer
 * nonlinear iteration. Null-manifold records are listed in skipped; they have
 * no solver reaction. No source state is changed by build or append.
 */
export function buildKirchhoffCoupledFrictionRows(constraint, dt, out = {}) {
    return prepareCoupledFrictionBatch(constraint, dt, out, true);
}

/** Promote the current residual evaluation for an immediate cone repair.
 * No geometry/history edits are allowed between evaluation and this call.
 * This bank must not be shared with a frozen nonlinear solve batch. */
export function materializeKirchhoffCoupledFrictionRows(constraint, dt, batch) {
    if (batch.constraint !== constraint || batch.dt !== dt || !batch.kinematicsOnly || batch.committed || batch.appended)
        throw new Error('A fresh friction residual batch is required');
    for (const entry of batch.entries) {
        const muU = constraint.axialFriction,
            muV = constraint.circumferentialFriction ?? constraint.torsionalFriction ?? muU;
        if (entry.record.manifoldContact !== entry.contact || entry.record.id !== entry.recordId ||
            entry.record.kind !== entry.geometry.kind ||
            segmentIndex(entry.record, 0) !== entry.innerSegment || segmentIndex(entry.record, 1) !== entry.outerSegment ||
            entry.contact.normalLambda !== entry.surface.group.normalLambda ||
            entry.contact.tangentLambda[0] !== entry.sourceTangentU || entry.contact.tangentLambda[1] !== entry.sourceTangentV ||
            muU !== entry.surface.group.mu[0] || muV !== entry.surface.group.mu[1])
            throw new Error('Contact identity, load or friction changed after evaluation');
        for (let axis = 0; axis < 3; axis++) if (entry.contact.normal[axis] !== entry.manifoldNormal[axis])
            throw new Error('Contact normal changed after evaluation');
    }
    batch.rows.length = 0;
    for (const entry of batch.entries) {
        materializeKirchhoffSurfaceFrictionGradients(entry.surface);
        for (const row of entry.surface.rows) batch.rows.push(row);
    }
    batch.kinematicsOnly = false;
    batch.version++;
    return batch;
}

function prepareCoupledFrictionBatch(constraint, dt, out, buildRows) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Positive finite dt is required');
    batchStorage(out);
    out.rows.length = out.groups.length = out.entries.length = out.skipped.length = 0;
    out.constraint = constraint; out.dt = dt; out.rowOffset = 0;
    out.kinematicsOnly = !buildRows;
    out.committed = false; out.appended = false; out.version = (out.version ?? 0) + 1;
    out.maximumNormalMomentResidual = 0; out.effectiveFilletCount = 0;
    if (constraint.surfaceMotion?.phase === 'bias') return out;
    const epoch = beginKirchhoffSurfaceEvaluation();
    for (const record of constraint.kirchhoffContacts ?? []) {
        if (!record.manifoldContact) { out.skipped.push({ record, reason: 'no-manifold-contact' }); continue; }
        const index = out.entries.length, entry = out._pool[index] ??= makeEntry();
        entry.record = record; entry.contact = record.manifoldContact;
        entry.sourceTangentU = entry.contact.tangentLambda?.[0];
        entry.sourceTangentV = entry.contact.tangentLambda?.[1];
        (entry.manifoldNormal ??= vector()).set(entry.contact.normal);
        entry.innerSegment = segmentIndex(record, 0); entry.outerSegment = segmentIndex(record, 1);
        entry.recordId = record.id;
        prepareKirchhoffCoupledSurfaceGeometry(constraint, record, entry.geometry);
        if (!entry.view || Object.getPrototypeOf(entry.view) !== record) entry.view = Object.create(record);
        entry.view.surfaceContactPoint = entry.geometry.point;
        entry.view.surfaceAxialTangent = entry.geometry.axialTangent;
        const evaluation = entry.evaluation ??= { epoch: 0,
            currentCenters: [entry.geometry.innerCenter, entry.geometry.outerCenter], stencils: entry.geometry.stencils };
        evaluation.epoch = epoch;
        evaluation.cachePreviousNodes = constraint._reuseCandidateEvaluation !== false;
        const surface = (buildRows ? buildKirchhoffSurfaceFriction : measureKirchhoffSurfaceFrictionState)(constraint, entry.view, dt, entry.surface, evaluation);
        if (!surface.supported) throw new RangeError(`Surface feature ${record.kind}: ${surface.reason}`);
        entry.rowStart = out.entries.length * 2;
        if (buildRows) for (const row of surface.rows) out.rows.push(row);
        const group = surface.group;
        group.rowIndices[0] = entry.rowStart; group.rowIndices[1] = entry.rowStart + 1;
        out.groups.push(group);
        out.entries.push(entry);
        out.maximumNormalMomentResidual = Math.max(out.maximumNormalMomentResidual, Math.hypot(...entry.geometry.normalMomentResidual));
        out.effectiveFilletCount += Number(entry.geometry.effectiveFillet);
    }
    return out;
}

/** Append after boundary/tool rows. Indices are relative to ALL additionalRows.
 * Records offset for commit; append once per freshly built batch. */
export function appendKirchhoffCoupledFrictionRows(batch, additionalRows, groups) {
    if (batch.kinematicsOnly) throw new Error('Residual batch has no solver rows');
    if (batch.appended) throw new Error('Friction batch was already appended; rebuild before reuse');
    if (!Array.isArray(additionalRows) || !Array.isArray(groups)) throw new TypeError('Row and group collectors must be arrays');
    batch.rowOffset = additionalRows.length;
    for (const row of batch.rows) additionalRows.push(row);
    for (const entry of batch.entries) {
        const source = entry.surface.group, target = entry.appendedGroup;
        target.kind = source.kind; target.mu = source.mu; target.normalLambda = source.normalLambda;
        target.normalContact = source.normalContact;
        target.rowIndices[0] = batch.rowOffset + entry.rowStart;
        target.rowIndices[1] = batch.rowOffset + entry.rowStart + 1;
        groups.push(target);
    }
    batch.appended = true;
    return batch;
}

function forceAndMoments(entry, components) {
    const surface = entry.surface;
    for (let axis = 0; axis < 3; axis++) entry.force[axis] = surface.axes[0][axis] * components[0] + surface.axes[1][axis] * components[1];
    cross(surface.levers[0], entry.force, entry.innerMoment);
    cross(surface.levers[1], entry.force, entry.outerMoment);
    for (let axis = 0; axis < 3; axis++) entry.outerMoment[axis] = -entry.outerMoment[axis];
}

function diagnosticVector(record, name, value) { (record[name] ??= vector()).set(value); }

function writeReactionDiagnostics(entry, components, phase) {
    forceAndMoments(entry, components);
    const record = entry.record;
    diagnosticVector(record, 'innerSurfaceMomentImpulse', entry.innerMoment);
    diagnosticVector(record, 'outerSurfaceMomentImpulse', entry.outerMoment);
    diagnosticVector(record, 'surfaceTangentialImpulse', entry.force);
    diagnosticVector(record, 'surfaceFrictionPoint', entry.surface.point);
    record.surfaceFrictionDiagnosticPhase = phase;
}

/** Commit only the SAME scale of additionalIncrement used by full-system apply.
 * Normal multipliers belong to caller and may already have changed. No cone
 * projection occurs here, including on Fn shrink: residual then requests a new
 * fixed-load solve. No body positions, frames, velocities or normal state change.
 * Commit exactly once, BEFORE rebuilding geometry/batch. Validation is atomic.
 */
export function commitKirchhoffCoupledFrictionMultipliers(batch, additionalIncrement, scale = 1) {
    if (batch.kinematicsOnly) throw new Error('Residual batch cannot apply solver reactions');
    if (!Number.isFinite(scale) || scale < 0 || scale > 1) throw new RangeError('Shared scale must lie in [0,1]');
    if (batch.committed) throw new Error('Friction multipliers were already committed');
    if (!additionalIncrement || additionalIncrement.length < batch.rowOffset + batch.rows.length) throw new RangeError('Missing friction increments');
    for (const entry of batch.entries) {
        const record = entry.record, contact = entry.contact;
        if (record.manifoldContact !== contact || record.id !== entry.recordId ||
            segmentIndex(record, 0) !== entry.innerSegment || segmentIndex(record, 1) !== entry.outerSegment)
            throw new Error('Friction contact topology changed before commit');
        for (let axis = 0; axis < 2; axis++) {
            const increment = finite(additionalIncrement[batch.rowOffset + entry.rowStart + axis], 'friction increment');
            entry.increment[axis] = scale * increment;
            entry.lambda[axis] = finite(entry.surface.rows[axis].lambda + entry.increment[axis], 'committed friction multiplier');
        }
        for (let axis = 0; axis < 3; axis++) {
            finite(contact.tangentU[axis], 'current manifold U'); finite(contact.tangentV[axis], 'current manifold V');
            if (Math.abs(contact.normal[axis] - entry.manifoldNormal[axis]) > 1e-8)
                throw new Error('Contact normal changed before friction commit');
        }
        if (!contact.tangentLambda || contact.tangentLambda.length < 2) throw new RangeError('Missing manifold multipliers');
    }
    for (const entry of batch.entries) {
        const contact = entry.contact, record = entry.record;
        forceAndMoments(entry, entry.increment);
        diagnosticVector(record, 'innerSurfaceMomentIncrement', entry.innerMoment);
        diagnosticVector(record, 'outerSurfaceMomentIncrement', entry.outerMoment);
        diagnosticVector(record, 'surfaceTangentialIncrement', entry.force);
        writeReactionDiagnostics(entry, entry.lambda, 'committed-linearization');
        contact.tangentLambda[0] = dot(entry.force, contact.tangentU);
        contact.tangentLambda[1] = dot(entry.force, contact.tangentV);
        contact.twistLambda = contact.innerTwistImpulse = contact.outerTwistImpulse = 0;
        record.surfaceFrictionAppliedScale = scale;
    }
    batch.committed = true;
    batch.commitScale = scale;
    return batch;
}

/** Call AFTER actual geometry refresh, with a separate out from the solve batch.
 * Residual is the projected natural map at FINAL current Fn, in multiplier
 * units. inverseMobility is multiplier/mm. Does not clip lambda or apply any
 * mechanics. Optional diagnostic refresh expresses committed TOTAL reactions
 * at the new contact geometry; actual increment diagnostics stay untouched.
 * Optionally reuses exactly guarded kinematics; loads and KKT stay fresh.
 * Does not assemble solver gradient rows. The borrowed
 * _batch supports diagnostics/merit only and cannot be appended or committed.
 */
export function measureKirchhoffCoupledFrictionResidual(constraint, dt, out = {},
    { inverseMobility = 1, updateDiagnostics = true, cacheInputs = false, reuseInputs = false } = {}) {
    if (!Number.isFinite(inverseMobility) || inverseMobility <= 0) throw new RangeError('inverseMobility must be positive and finite');
    const stamp = out._inputStamp ??= {};
    const same = cacheInputs && updateKirchhoffFrictionInputStamp(constraint, dt, inverseMobility, stamp, reuseInputs);
    stamp.valid = false;
    out.reusedEvaluation = false;
    let batch;
    if (reuseInputs && same && out._batch) {
        // Cone preparation may have materialized Jacobians in this bank.
        // Restore the residual-only representation used by a fresh evaluation
        // so downstream merit follows exactly the same arithmetic path.
        out._batch.kinematicsOnly = true;
        out._batch.committed = false; out._batch.appended = false;
        out._batch.rowOffset = 0; out._batch.version++;
        out._batch.rows.length = 0;
        for (const entry of out._batch.entries) {
            entry.surface.kinematicsOnly = true;
            for (const row of entry.surface.rows) row.gradients.length = 0;
            refreshKirchhoffSurfaceFrictionLoads(constraint, entry.record, entry.surface);
            entry.manifoldNormal.set(entry.contact.normal);
            entry.sourceTangentU = entry.contact.tangentLambda[0];
            entry.sourceTangentV = entry.contact.tangentLambda[1];
        }
        out.reusedEvaluation = true;
        batch = out._batch;
    } else {
        stamp.valid = false;
        batch = prepareCoupledFrictionBatch(constraint, dt, out._batch ??= {}, false);
    }
    out.maximumResidual = out.maximumFeasibilityResidual = out.maximumStationarityResidual = 0;
    out.maximumDisplacementResidualMm = out.maximumConeViolation = 0;
    out.maximumNormalMomentResidual = batch.maximumNormalMomentResidual;
    out.contactCount = batch.entries.length;
    out.residualUnits = 'multiplier'; out.inverseMobility = inverseMobility;
    for (const entry of batch.entries) {
        entry.lambda[0] = entry.surface.rows[0].lambda;
        entry.lambda[1] = entry.surface.rows[1].lambda;
        entry.increment[0] = entry.surface.rows[0].strain;
        entry.increment[1] = entry.surface.rows[1].strain;
        const residual = evaluateKirchhoffSurfaceFriction(entry.lambda, entry.increment,
            entry.contact.normalLambda, entry.surface.group.mu, { inverseMobility }, entry.residual);
        const local = evaluateKirchhoffSurfaceFrictionKKT(entry.lambda, entry.increment,
            entry.contact.normalLambda, entry.surface.group.mu, entry.localResidual ??= {});
        out.maximumDisplacementResidualMm = Math.max(out.maximumDisplacementResidualMm, local.residualMm);
        out.maximumConeViolation = Math.max(out.maximumConeViolation, local.coneViolation);
        out.maximumResidual = Math.max(out.maximumResidual, residual.residual);
        out.maximumFeasibilityResidual = Math.max(out.maximumFeasibilityResidual, residual.feasibilityResidual);
        out.maximumStationarityResidual = Math.max(out.maximumStationarityResidual, residual.stationarityResidual);
        if (updateDiagnostics) writeReactionDiagnostics(entry, entry.lambda, 'refreshed-total-reaction');
    }
    stamp.valid = cacheInputs && !constraint.surfaceMotion && !constraint._splitMotion;
    return out;
}
