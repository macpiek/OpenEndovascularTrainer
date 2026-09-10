import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { buildKirchhoffSurfaceFriction, evaluateKirchhoffSurfaceFriction,
    evaluateKirchhoffSurfaceFrictionKKT } from './kirchhoffSurfaceFriction.js';
import { recordKirchhoffToolReaction, captureKirchhoffToolReaction } from './kirchhoffToolContactOwnership.js';

const XYZ = ['x', 'y', 'z'];
const EPSILON = 1e-12;
const vector = () => new Float64Array(3);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function finite(value, label) {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return value;
}

function cross(a, b, out) {
    const x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0]; out[0] = x; out[1] = y;
}

function stateFor(constraint) {
    const [inner, outer] = kirchhoffComponentBodies(constraint);
    if (!inner || !outer || inner === outer) throw new TypeError('Two distinct joint bodies are required');
    let state = constraint._coupledExternalFriction;
    if (!state || state.inner !== inner || state.outer !== outer) {
        // Owners already live in contacts. An enumerable Map lets nonlinear
        // rollback remove contacts created by a rejected trial as well.
        state = constraint._coupledExternalFriction = { inner, outer, owners: new Map(), contacts: [], step: 0 };
    }
    return state;
}

/** Once per physical dt. Clears tangent forces only; keeps contact identity and
 * working-set hints. Never touches owner.lambdas or body state. No-contact
 * joints allocate no contact/geometry storage, even at begin. */
export function beginKirchhoffExternalFrictionStep(constraint) {
    const state = constraint._coupledExternalFriction;
    if (!state) return null;
    for (const contact of state.contacts) contact.tangentLambda.fill(0);
    state.step++;
    return state;
}

function contactFor(state, row) {
    let contacts = state.owners.get(row.owner);
    if (!contacts) state.owners.set(row.owner, contacts = new Map());
    let contact = contacts.get(row.node);
    if (!contact) {
        const owner = row.owner, index = row.node;
        contact = { owner, index, tangentLambda: new Float64Array(2), tangentU: vector(), tangentV: vector(),
            get normalLambda() { return owner.lambdas[index]; } };
        contacts.set(index, contact); state.contacts.push(contact);
    }
    // A recycled normal slot is a different material contact. Never transfer
    // tangent history across a changed segment/body association.
    if (contact.bodyA !== row.bodyA || contact.bodyB !== row.bodyB ||
        contact.segmentA !== row.segmentA || contact.segmentB !== row.segmentB) {
        contact.tangentLambda.fill(0);
        contact.bodyA = row.bodyA; contact.bodyB = row.bodyB;
        contact.segmentA = row.segmentA; contact.segmentB = row.segmentB;
        contact.generation = (contact.generation ?? 0) + 1;
    }
    return contact;
}

function makeEntry(contact) {
    return { contact, surface: {}, normalSnapshot: vector(), lambda: new Float64Array(2),
        increment: new Float64Array(2), displacement: new Float64Array(2), residual: {}, localResidual: {},
        force: vector(), innerMoment: vector(), outerMoment: vector(),
        geometry: { centerA: vector(), centerB: vector(), point: vector(), normal: vector(),
            axialTangent: vector(), normalMomentResidual: vector() },
        record: { kind: 'external-capsule', manifoldContact: contact,
            innerWeights: new Float64Array(2), outerWeights: new Float64Array(2) },
        constraint: {}, appendedGroup: { rowIndices: new Uint32Array(2) } };
}

function centerAndRadius(body, segment, t, out) {
    if (!Number.isInteger(segment) || segment < 0 || segment + 1 >= body.count)
        throw new RangeError('Invalid external capsule segment');
    if (!Number.isFinite(t) || t < 0 || t > 1) throw new RangeError('External capsule foot must lie in [0,1]');
    for (let axis = 0; axis < 3; axis++) {
        const values = body[XYZ[axis]];
        out[axis] = (1 - t) * finite(values?.[segment], 'capsule endpoint') + t * finite(values?.[segment + 1], 'capsule endpoint');
    }
    const r0 = finite(body.nodeRadius?.[segment] ?? body.radius, 'capsule radius');
    const r1 = finite(body.nodeRadius?.[segment + 1] ?? body.radius, 'capsule radius');
    if (r0 < 0 || r1 < 0) throw new RangeError('Capsule radii must be nonnegative');
    return Math.max(r0, r1);
}

function projectTangent(tangent, normal) {
    const length = Math.hypot(...tangent);
    if (!(length > EPSILON)) return false;
    for (let i = 0; i < 3; i++) tangent[i] /= length;
    const axial = dot(tangent, normal);
    for (let i = 0; i < 3; i++) tangent[i] -= axial * normal[i];
    const projectedLength = Math.hypot(...tangent);
    if (projectedLength <= EPSILON) return false;
    for (let i = 0; i < 3; i++) tangent[i] /= projectedLength;
    return true;
}

function chooseTangent(body, segment, normal, out) {
    for (let i = 0; i < 3; i++) out[i] = body[XYZ[i]][segment + 1] - body[XYZ[i]][segment];
    if (projectTangent(out, normal)) return 'outer-segment';
    // At end-cap poles there is no distinguished capsule axial tangent. The
    // law is isotropic, so either material transverse axis spans the same disk.
    // A material basis, unlike a fixed world axis, rotates with the apparatus.
    let x = finite(body.orientationX?.[segment], 'orientationX');
    let y = finite(body.orientationY?.[segment], 'orientationY');
    let z = finite(body.orientationZ?.[segment], 'orientationZ');
    let w = finite(body.orientationW?.[segment], 'orientationW');
    const length = Math.hypot(x, y, z, w);
    if (length <= EPSILON) throw new RangeError('Material quaternion must have nonzero norm');
    x /= length; y /= length; z /= length; w /= length;
    out[0] = 1 - 2 * (y * y + z * z); out[1] = 2 * (x * y + w * z); out[2] = 2 * (x * z - w * y);
    if (projectTangent(out, normal)) return 'outer-material-d1';
    out[0] = 2 * (x * y - w * z); out[1] = 1 - 2 * (x * x + z * z); out[2] = 2 * (y * z + w * x);
    if (!projectTangent(out, normal)) throw new RangeError('Degenerate capsule material basis');
    return 'outer-material-d2';
}

function prepareGeometry(joint, row, entry, mu) {
    const [inner, outer] = kirchhoffComponentBodies(joint);
    const g = entry.geometry, record = entry.record, aIsInner = row.bodyA === inner;
    g.radiusA = centerAndRadius(row.bodyA, row.segmentA, row.tA, g.centerA);
    g.radiusB = centerAndRadius(row.bodyB, row.segmentB, row.tB, g.centerB);
    for (let i = 0; i < 3; i++) entry.normalSnapshot[i] = finite(row.normal?.[i], 'capsule normal A-B');
    const length = Math.hypot(...entry.normalSnapshot);
    if (length <= EPSILON) throw new RangeError('Capsule normal must have positive length');
    for (let i = 0; i < 3; i++) {
        const n = entry.normalSnapshot[i] / length;
        // Common midpoint of the two facing capsule witnesses, also while
        // penetrating: p = ((ca-ra*n) + (cb+rb*n))/2, n = normalized(ca-cb).
        g.point[i] = 0.5 * (g.centerA[i] + g.centerB[i] + (g.radiusB - g.radiusA) * n);
        g.normal[i] = aIsInner ? n : -n;
        g.normalMomentResidual[i] = g.centerA[i] - g.centerB[i];
    }
    cross(g.normalMomentResidual, g.normal, g.normalMomentResidual);
    record._innerSegmentIndex = aIsInner ? row.segmentA : row.segmentB;
    record._outerSegmentIndex = aIsInner ? row.segmentB : row.segmentA;
    const ti = aIsInner ? row.tA : row.tB, to = aIsInner ? row.tB : row.tA;
    record.innerWeights[0] = 1 - ti; record.innerWeights[1] = ti;
    record.outerWeights[0] = 1 - to; record.outerWeights[1] = to;
    record.normal = g.normal; record.surfaceContactPoint = g.point;
    record.surfaceAxialTangent = g.axialTangent;
    g.tangentSource = chooseTangent(outer, record._outerSegmentIndex, g.normal, g.axialTangent);
    entry.constraint.innerBody = inner; entry.constraint.outerBody = outer;
    entry.constraint.surfaceMotion = joint.surfaceMotion;
    entry.constraint.axialFriction = entry.constraint.circumferentialFriction = mu;
    entry.normalRow = row; entry.owner = row.owner; entry.index = row.node;
    entry.bodyA = row.bodyA; entry.bodyB = row.bodyB;
    entry.segmentA = row.segmentA; entry.segmentB = row.segmentB;
    entry.tA = row.tA; entry.tB = row.tB; entry.generation = entry.contact.generation;
}

/** Accepts a mixed boundary-row array; only enabled, positive-friction tool
 * rows for this body pair are included. Required cached normal-row metadata:
 * owner, node (lambda index), bodyA/B, segmentA/B, tA/B, normal (unit A-B).
 * Normal geometry, gradients, lambda and bounds remain completely unchanged.
 * No collision/profile sampling: feet are those of the existing exact capsule
 * normal collector. Call after that collector refreshes current geometry.
 *
 * U/V share ONE fixed-load isotropic Coulomb disk, mu = owner.friction. There
 * is no separate torsional row: both translations and local right rotations
 * contribute to surface slip through the two physical lever arms.
 * All arrays/contacts/rows are persistent borrowed views. Reuse out; use a
 * SEPARATE out for measurement, so it cannot overwrite a pending solve.
 */
export function buildKirchhoffExternalFrictionRows(constraint, normalRows, dt, out = {}) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Positive finite dt is required');
    out.rows ??= []; out.groups ??= []; out.entries ??= [];
    out.rows.length = out.groups.length = out.entries.length = 0;
    out.constraint = constraint; out.dt = dt; out.rowOffset = 0;
    out.committed = out.appended = false; out.version = (out.version ?? 0) + 1;
    out.maximumNormalMomentResidual = 0; out.state = null;
    if (constraint.surfaceMotion?.phase === 'bias') return out;
    for (const row of normalRows ?? []) {
        if (row.kind !== 'tool' || !row.owner || row.owner.enabled === false) continue;
        const mu = finite(row.owner.friction ?? 0, 'external friction');
        if (mu < 0) throw new RangeError('External friction must be nonnegative');
        if (mu === 0) continue;
        const [inner, outer] = kirchhoffComponentBodies(constraint);
        if (!((row.bodyA === inner && row.bodyB === outer) || (row.bodyA === outer && row.bodyB === inner))) continue;
        if (!Number.isInteger(row.node) || row.node < 0 || row.node >= row.owner.lambdas?.length)
            throw new RangeError('Invalid external normal multiplier index');
        const state = out.state ??= stateFor(constraint), contact = contactFor(state, row);
        out.step = state.step;
        const pool = out._entriesByContact ??= new WeakMap();
        let entry = pool.get(contact);
        if (!entry) pool.set(contact, entry = makeEntry(contact));
        if (entry.buildVersion === out.version) throw new Error('Duplicate external normal contact');
        entry.buildVersion = out.version;
        prepareGeometry(constraint, row, entry, mu);
        const surface = buildKirchhoffSurfaceFriction(entry.constraint, entry.record, dt, entry.surface);
        if (!surface.supported) throw new RangeError(`External capsule friction: ${surface.reason}`);
        entry.rowStart = out.rows.length;
        for (const tangentRow of surface.rows) {
            tangentRow.kind = 'external-friction';
            tangentRow.reactionWrenches = captureKirchhoffToolReaction(entry.constraint, tangentRow.gradients, tangentRow.reactionWrenches);
            out.rows.push(tangentRow);
        }
        surface.group.rowIndices[0] = entry.rowStart; surface.group.rowIndices[1] = entry.rowStart + 1;
        out.groups.push(surface.group); out.entries.push(entry);
        out.maximumNormalMomentResidual = Math.max(out.maximumNormalMomentResidual, Math.hypot(...entry.geometry.normalMomentResidual));
    }
    return out;
}

/** Group indices and commit offsets are relative to ALL additionalRows. */
export function appendKirchhoffExternalFrictionRows(batch, additionalRows, groups) {
    if (batch.appended) throw new Error('External friction batch already appended');
    if (!Array.isArray(additionalRows) || !Array.isArray(groups)) throw new TypeError('Collectors must be arrays');
    if (additionalRows === batch.rows || groups === batch.groups) throw new Error('Append to separate combined collectors');
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

function reaction(entry, components) {
    const surface = entry.surface;
    for (let i = 0; i < 3; i++) entry.force[i] = surface.axes[0][i] * components[0] + surface.axes[1][i] * components[1];
    cross(surface.levers[0], entry.force, entry.innerMoment);
    cross(surface.levers[1], entry.force, entry.outerMoment);
    for (let i = 0; i < 3; i++) entry.outerMoment[i] = -entry.outerMoment[i];
}

function diagnostics(entry, components, phase) {
    reaction(entry, components);
    const record = entry.record;
    (record.surfaceTangentialImpulse ??= vector()).set(entry.force);
    (record.innerSurfaceMomentImpulse ??= vector()).set(entry.innerMoment);
    (record.outerSurfaceMomentImpulse ??= vector()).set(entry.outerMoment);
    (record.surfaceFrictionPoint ??= vector()).set(entry.surface.point);
    record.surfaceFrictionDiagnosticPhase = phase;
}

/** Commit exactly once at the SAME scale as the joint geometry update, AFTER
 * the root normal helper updates owner.lambdas. A changed Fn is allowed and
 * never clips tangential multipliers here: fresh residual drives another QP.
 * No displacement, frame, normal force or unrelated row is modified.
 * Validation is atomic; refresh cached normal geometry only after commit. */
export function commitKirchhoffExternalFrictionMultipliers(batch, additionalIncrement, scale = 1) {
    if (!Number.isFinite(scale) || scale < 0 || scale > 1) throw new RangeError('Shared scale must lie in [0,1]');
    if (batch.committed) throw new Error('External friction multipliers already committed');
    if (batch.state && (batch.constraint._coupledExternalFriction !== batch.state || batch.step !== batch.state.step ||
        kirchhoffComponentBodies(batch.constraint)[0] !== batch.state.inner || kirchhoffComponentBodies(batch.constraint)[1] !== batch.state.outer))
        throw new Error('External friction step changed before commit');
    if (batch.rows.length && (!additionalIncrement || additionalIncrement.length < batch.rowOffset + batch.rows.length))
        throw new RangeError('Missing external friction increments');
    for (const entry of batch.entries) {
        const row = entry.normalRow;
        if (row.owner !== entry.owner || row.node !== entry.index || row.bodyA !== entry.bodyA || row.bodyB !== entry.bodyB ||
            row.segmentA !== entry.segmentA || row.segmentB !== entry.segmentB || row.tA !== entry.tA || row.tB !== entry.tB ||
            entry.contact.generation !== entry.generation) throw new Error('External contact topology changed before commit');
        for (let i = 0; i < 3; i++) if (row.normal[i] !== entry.normalSnapshot[i])
            throw new Error('External contact normal changed before commit');
        for (let i = 0; i < 2; i++) {
            entry.increment[i] = scale * finite(additionalIncrement[batch.rowOffset + entry.rowStart + i], 'external friction increment');
            entry.lambda[i] = finite(entry.surface.rows[i].lambda + entry.increment[i], 'external friction multiplier');
        }
    }
    for (const entry of batch.entries) {
        for (let axis = 0; axis < 2; axis++) recordKirchhoffToolReaction(entry.owner, entry.index,
            entry.surface.rows[axis].reactionWrenches, entry.increment[axis]);
        reaction(entry, entry.increment);
        const record = entry.record;
        (record.surfaceTangentialIncrement ??= vector()).set(entry.force);
        (record.innerSurfaceMomentIncrement ??= vector()).set(entry.innerMoment);
        (record.outerSurfaceMomentIncrement ??= vector()).set(entry.outerMoment);
        diagnostics(entry, entry.lambda, 'committed-linearization');
        entry.contact.tangentU.set(entry.surface.axes[0]); entry.contact.tangentV.set(entry.surface.axes[1]);
        entry.contact.tangentLambda.set(entry.lambda);
        record.surfaceFrictionAppliedScale = scale;
    }
    batch.committed = true; batch.commitScale = scale;
    return batch;
}

/** Recollect exact capsule feet/normals at final geometry BEFORE measurement.
 * Use separate out from the solve batch. This reads current Fn via the owner
 * alias and evaluates the full surface displacement without applying/clipping
 * forces. Acceptance uses the exact Coulomb KKT displacement residual in mm
 * AND maximumConeViolation (dimensionless disk excess; Infinity for nonzero
 * force at zero load), exactly as the lumen adapter. Zero load permits slip
 * but requires zero tangent force. The raw natural-map residual in multiplier
 * units is retained only as a diagnostic; do not compare it to a mm tolerance.
 * inverseMobility affects that diagnostic only, never the KKT acceptance. */
export function measureKirchhoffExternalFrictionResidual(constraint, freshNormalRows, dt, out = {},
    { inverseMobility = 1, updateDiagnostics = true } = {}) {
    if (!Number.isFinite(inverseMobility) || inverseMobility <= 0) throw new RangeError('Positive finite inverseMobility is required');
    const batch = buildKirchhoffExternalFrictionRows(constraint, freshNormalRows, dt, out._batch ??= {});
    out.maximumResidual = out.maximumFeasibilityResidual = out.maximumStationarityResidual = 0;
    out.maximumDisplacementResidualMm = out.maximumConeViolation = 0;
    out.maximumNormalMomentResidual = batch.maximumNormalMomentResidual;
    out.contactCount = batch.entries.length; out.residualUnits = 'multiplier'; out.inverseMobility = inverseMobility;
    const options = out._options ??= {};
    for (const entry of batch.entries) {
        for (let i = 0; i < 2; i++) { entry.lambda[i] = entry.surface.rows[i].lambda; entry.displacement[i] = entry.surface.rows[i].strain; }
        options.inverseMobility = inverseMobility;
        const residual = evaluateKirchhoffSurfaceFriction(entry.lambda, entry.displacement,
            entry.contact.normalLambda, entry.surface.group.mu, options, entry.residual);
        const local = evaluateKirchhoffSurfaceFrictionKKT(entry.lambda, entry.displacement,
            entry.contact.normalLambda, entry.surface.group.mu, entry.localResidual);
        out.maximumDisplacementResidualMm = Math.max(out.maximumDisplacementResidualMm, local.residualMm);
        out.maximumConeViolation = Math.max(out.maximumConeViolation, local.coneViolation);
        out.maximumResidual = Math.max(out.maximumResidual, residual.residual);
        out.maximumFeasibilityResidual = Math.max(out.maximumFeasibilityResidual, residual.feasibilityResidual);
        out.maximumStationarityResidual = Math.max(out.maximumStationarityResidual, residual.stationarityResidual);
        if (updateDiagnostics) diagnostics(entry, entry.lambda, 'refreshed-total-reaction');
    }
    return out;
}
