import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { evaluateBendTwistConstraint, inverseRotateVectorByQuaternion } from './discreteKirchhoffRod.js';

const XYZ = ['x', 'y', 'z'];
const XYZW = ['x', 'y', 'z', 'w'];
const ZERO = Object.freeze({ x: 0, y: 0, z: 0 });

function finite(value, label) {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return value;
}

function bodiesMatch(constraint, state) {
    const bodies = kirchhoffComponentBodies(constraint);
    return bodies.length === state.bodies.length && bodies.every((body, side) => body === state.bodies[side]);
}

function stateFor(constraint) {
    const bodies = kirchhoffComponentBodies(constraint);
    let state = constraint._coupledOrientationRows;
    if (!state || !bodiesMatch(constraint, state)) {
        state = constraint._coupledOrientationRows = { constraint,
            bodies, rows: [], entries: [],
            pool: [], measurePool: [], measurement: {}, step: 0, version: 0, begun: false, dt: 0 };
    }
    return state;
}

function compliantSegment(body) {
    const compliance = finite(body.orientationControlCompliance ?? 0, 'orientation control compliance');
    if (compliance < 0) throw new RangeError('Orientation control compliance must be nonnegative');
    const segment = body.orientationControlSegment ?? -1;
    if (compliance === 0 || segment < 0) return -1;
    if (!Number.isInteger(segment) || segment >= body.segmentCount) throw new RangeError('Invalid orientation control segment');
    return segment >= Math.max(0, body.activeStart) && segment < Math.min(body.segmentCount, body.activeEnd) ? segment : -1;
}

function readQuaternion(body, segment, target, out) {
    for (const axis of XYZW) {
        const suffix = axis.toUpperCase();
        out[axis] = finite(target ? body['orientationControl' + suffix] : body['orientation' + suffix]?.[segment],
            target ? 'orientation control target' : 'material orientation');
    }
    const norm = Math.hypot(out.x, out.y, out.z, out.w);
    if (!(norm > 0) || !Number.isFinite(norm)) throw new RangeError('Orientation quaternion must have positive finite norm');
    for (const axis of XYZW) out[axis] /= norm;
    return out;
}

function makeEntry(side) {
    return { side, target: {}, current: {}, evaluation: {}, worldGradient: {}, localGradient: {},
        lambda: new Float64Array(3), nextLambda: new Float64Array(3), targetSnapshot: new Float64Array(4),
        residual: new Float64Array(3), rows: Array.from({ length: 3 }, (_, component) => ({
            kind: 'orientation-control', side, component, lower: -Infinity, upper: Infinity,
            gradients: Array.from({ length: 3 }, () => Object.seal({ side, dof: 0, value: 0 }))
        })) };
}

function evaluate(body, segment, dt, entry) {
    entry.body = body; entry.segment = segment;
    entry.compliance = finite(body.orientationControlCompliance, 'orientation control compliance');
    entry.alpha = finite(entry.compliance / (dt * dt), 'orientation control alpha');
    if (!body.orientationControlLambda || body.orientationControlLambda.length !== 3)
        throw new RangeError('Three orientation control multipliers are required');
    readQuaternion(body, segment, true, entry.target);
    readQuaternion(body, segment, false, entry.current);
    evaluateBendTwistConstraint(entry.target, entry.current, ZERO, entry.evaluation);
    for (let component = 0; component < 3; component++) {
        entry.lambda[component] = finite(body.orientationControlLambda[component], 'orientation control multiplier');
        entry.residual[component] = entry.evaluation.strain[XYZ[component]] + entry.alpha * entry.lambda[component];
    }
    return entry;
}

/** Once per fixed physical dt; resets only compliant orientation-control
 * multipliers. No frame, position, velocity, material lambda or hard-control
 * state is changed. Existing row objects and optional working-set hints remain.
 * The world may already clear these same three multipliers at step begin. */
export function beginKirchhoffCoupledOrientationStep(constraint) {
    const state = stateFor(constraint);
    for (const body of state.bodies) if (body.orientationControlCompliance > 0) {
        if (!body.orientationControlLambda || body.orientationControlLambda.length !== 3)
            throw new RangeError('Three orientation control multipliers are required');
        body.orientationControlLambda.fill(0);
    }
    state.step++; state.begun = true; state.dt = 0;
    state.rows.length = state.entries.length = 0;
    state.committed = true; state.appended = false;
    return state;
}

/** Three equality rows per ACTIVE compliant (>0) prescribed material frame.
 * c = Log(q_target^-1 q_current), alpha = compliance / dt², lambda is the
 * existing body.orientationControlLambda in these same target-frame strain
 * coordinates. J_world comes from evaluateBendTwistConstraint; J_local is
 * J_world R_current for LOCAL RIGHT increments q_current exp(deltaTheta).
 * This is the existing isotropic orientation spring, with no new compliance,
 * mass scaling, relaxation strength or independent pre-application.
 * The native principal quaternion logarithm is used unchanged; exactly at pi
 * its branch is not classically differentiable. Relinearize finite rotations.
 *
 * Hard controls (compliance=0) are omitted: the root must prescribe their exact
 * q BEFORE building ANY contact/fold/friction rows, then eliminate their angular
 * dofs in the joint kernel. Disabled/out-of-range controls are also omitted.
 * No body state is changed by build. The returned batch is persistent/borrowed;
 * append after any earlier additional rows, solve/apply once at a common scale,
 * then commit these three multipliers before rebuilding. */
export function buildKirchhoffCoupledOrientationRows(constraint, dt) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Positive finite dt is required');
    const state = stateFor(constraint);
    if (!state.begun) throw new Error('beginKirchhoffCoupledOrientationStep is required');
    if (state.dt && state.dt !== dt) throw new Error('Orientation timestep changed within a physical step');
    state.dt = dt; state.version++;
    state.rows.length = state.entries.length = 0;
    state.rowOffset = 0; state.appended = state.committed = false; state.builtStep = state.step;
    for (let side = 0; side < state.bodies.length; side++) {
        const body = state.bodies[side], segment = compliantSegment(body);
        if (segment < 0) continue;
        const entry = evaluate(body, segment, dt, state.pool[side] ??= makeEntry(side));
        entry.lambdaSource = body.orientationControlLambda;
        entry.start = body.activeStart; entry.end = body.activeEnd;
        for (let i = 0; i < 4; i++) entry.targetSnapshot[i] = body['orientationControl' + XYZW[i].toUpperCase()];
        entry.rowStart = state.rows.length;
        for (let component = 0; component < 3; component++) {
            const row = entry.rows[component], gradient = entry.evaluation.gradient1;
            row.strain = entry.evaluation.strain[XYZ[component]];
            row.alpha = entry.alpha; row.lambda = entry.lambda[component]; row.segment = segment;
            for (let axis = 0; axis < 3; axis++) entry.worldGradient[XYZ[axis]] = gradient[component * 3 + axis];
            inverseRotateVectorByQuaternion(entry.current, entry.worldGradient, entry.localGradient);
            for (let axis = 0; axis < 3; axis++) {
                row.gradients[axis].dof = segment * 6 + 3 + axis;
                row.gradients[axis].value = finite(entry.localGradient[XYZ[axis]], 'local orientation gradient');
            }
            state.rows.push(row);
        }
        state.entries.push(entry);
    }
    return state;
}

/** Additional-row indices are absolute within the combined collector. */
export function appendKirchhoffCoupledOrientationRows(batch, additionalRows) {
    if (batch.appended) throw new Error('Orientation batch already appended');
    if (!Array.isArray(additionalRows) || additionalRows === batch.rows)
        throw new TypeError('Append to a separate combined row array');
    batch.rowOffset = additionalRows.length;
    for (const row of batch.rows) additionalRows.push(row);
    batch.appended = true;
    return batch;
}

/** Commit only the multiplier increments included in the SAME accepted joint
 * correction. No q/xyz writes and no projection. Validation is atomic; changed
 * control target, compliance, support or external lambda writes are rejected.
 * Optional buildVersion guards callers retaining borrowed batches. */
export function commitKirchhoffCoupledOrientationMultipliers(batch, increments, scale = 1, buildVersion = batch.version) {
    if (!Number.isFinite(scale) || scale < 0 || scale > 1) throw new RangeError('Shared scale must lie in [0,1]');
    if (batch.constraint._coupledOrientationRows !== batch || !bodiesMatch(batch.constraint, batch) ||
        batch.builtStep !== batch.step || buildVersion !== batch.version) throw new Error('Stale orientation batch');
    if (batch.committed) throw new Error('Orientation multipliers already committed');
    if (batch.rows.length && (!increments || increments.length < batch.rowOffset + batch.rows.length))
        throw new RangeError('Missing orientation increments');
    for (const entry of batch.entries) {
        const body = entry.body;
        if (body.orientationControlSegment !== entry.segment || body.orientationControlCompliance !== entry.compliance ||
            body.activeStart !== entry.start || body.activeEnd !== entry.end || body.orientationControlLambda !== entry.lambdaSource)
            throw new Error('Orientation control topology/compliance changed before commit');
        for (let i = 0; i < 4; i++) if (body['orientationControl' + XYZW[i].toUpperCase()] !== entry.targetSnapshot[i])
            throw new Error('Orientation target changed before commit');
        for (let i = 0; i < 3; i++) {
            if (body.orientationControlLambda[i] !== entry.lambda[i]) throw new Error('Orientation multiplier changed before commit');
            entry.nextLambda[i] = finite(entry.lambda[i] + scale * finite(increments[batch.rowOffset + entry.rowStart + i],
                'orientation increment'), 'committed orientation multiplier');
        }
    }
    for (const entry of batch.entries) entry.body.orientationControlLambda.set(entry.nextLambda);
    batch.committed = true; batch.commitScale = scale;
    return batch;
}

/** Fresh nonlinear equilibrium of each compliant control: norm(c+alpha*lambda)
 * in RADIANS. Raw strain is diagnostic only: a loaded compliant spring need not
 * reach its exact target. Separate scratch leaves any pending rows untouched.
 * No geometric/force/control state is modified. Never skips a sleeping body's
 * constraint; sleep must follow, and cannot substitute for, convergence. */
export function measureKirchhoffCoupledOrientationResidual(constraint, dt) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Positive finite dt is required');
    const state = stateFor(constraint), out = state.measurement;
    out.maximumResidualRad = out.maximumStrainRad = out.controlCount = out.rowCount = 0;
    out.worstSide = out.worstSegment = -1; out.residualUnits = 'radians';
    for (let side = 0; side < state.bodies.length; side++) {
        const body = state.bodies[side], segment = compliantSegment(body);
        if (segment < 0) continue;
        const entry = evaluate(body, segment, dt, state.measurePool[side] ??= makeEntry(side));
        const residual = Math.hypot(...entry.residual), strain = entry.evaluation.strain;
        if (residual > out.maximumResidualRad) {
            out.maximumResidualRad = residual; out.worstSide = side; out.worstSegment = segment;
        }
        out.maximumStrainRad = Math.max(out.maximumStrainRad, Math.hypot(strain.x, strain.y, strain.z));
        out.controlCount++; out.rowCount += 3;
    }
    return out;
}
