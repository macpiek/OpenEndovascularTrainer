import { kirchhoffRowNaturalMapMobility } from './kirchhoffCoupledBoundaryRows.js';
import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
/**
 * Joint unilateral director-angle limits, in the additionalRows convention.
 *
 * c = limit - acos(d3_previous . d3_next) >= 0; lambda >= 0.
 * For local RIGHT frame increments, gradients are +R_previous^T n and
 * -R_next^T n, n = normalized(d3_previous x d3_next). No xyz rows, no spin
 * constraint. atan2(|cross|,dot) evaluates the same angle stably at endpoints.
 *
 * Lifecycle: begin(constraint) once per physical dt; build -> joint solve and
 * apply its geometry -> apply multipliers (same scale) -> measure; relinearize
 * until converged. All helper results/row objects are borrowed and persistent.
 * begin clears forces, retaining only optional working-set/branch hints.
 *
 * Append to a separate combined additionalRows buffer if another row provider
 * owns its own rows. Its multiplier helper must NOT process these fold rows.
 * apply uses the absolute indices recorded by the latest build; apply once.
 *
 * Legacy fractional strength multiplies a projection, not a measured material
 * compliance. Default: any positive strength retains the hard equilibrium
 * limit; strength <= 0 disables. Alpha is always zero. No mass/iteration-based
 * compliance mapping is invented. A fractional strength changes no physical
 * target; the old iteration relaxation is not reproduced inside a joint solve.
 *
 * At parallel directors angle has no unique linear differential; the slack
 * row uses zero subgradient. At antiparallel directors choose a bending branch
 * anchored in the previous material frame (retaining the last regular axis).
 * This finite one-sided subgradient is NOT a classical derivative at the cusp.
 * These cases are reported. Relinearization and the joint trust region remain
 * necessary; an inactive linear row cannot bound arbitrary finite rotations.
 *
 * The old limiter measured POSITION tangents but rotated directors: its c and
 * J were inconsistent away from x'=d3. This director inequality matches the
 * original positional anti-fold intent ON the Kirchhoff adaptation manifold.
 * measure also reports positional violations and director/tangent mismatch;
 * a small director residual alone does not certify the geometric angle limit.
 * Angular residuals below are in RADIANS, not the world's millimetre tolerance.
 */

const DEGREES = Math.PI / 180;
const AXIS_EPSILON = 64 * Number.EPSILON;

function finite(value, label) {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return value;
}

function storageFor(body) {
    const count = body.count;
    if (!Number.isInteger(count) || count < 0) throw new RangeError('body.count must be nonnegative');
    return {
        body, count, lambda: new Float64Array(count), pool: [],
        frames: new Float64Array(count * 9),
        axisLocal: new Float64Array(count * 2), axisValid: new Uint8Array(count),
        limitDegrees: new Float64Array(count).fill(NaN), limitRadians: new Float64Array(count),
        included: new Uint8Array(count), first: 0, end: 0, strength: 0,
        builtFirst: 0, builtEnd: 0, builtStrength: 0
    };
}

function stateFor(constraint) {
    const bodies = kirchhoffComponentBodies(constraint);
    return constraint._coupledFoldRows ??= {
        storage: bodies.map(storageFor),
        rows: [], geometry: {}, begun: false, pending: false, dt: 0,
        step: 0, buildVersion: 0,
        measurement: {
            rowCount: 0, maximumResidual: 0, maximumNaturalMapResidual: 0, maximumViolation: 0, maximumAngle: 0,
            maximumPositionalViolation: 0, maximumDirectorTangentMismatch: 0,
            maximumPositionalViolationBound: 0,
            parallelPairs: 0, antiparallelPairs: 0, degeneratePositionPairs: 0,
            unavailablePositionPairs: 0, fractionalHardRows: 0
        }
    };
}

function checkBodies(constraint, state) {
    const bodies = kirchhoffComponentBodies(constraint);
    if (bodies.length !== state.storage.length) throw new Error('Fold topology changed; begin a new fold step');
    for (let side = 0; side < state.storage.length; side++) {
        const body = bodies[side];
        if (body !== state.storage[side].body || body.count !== state.storage[side].count) {
            throw new Error('Fold topology changed; begin a new fold step');
        }
    }
    if (!state.begun) throw new Error('beginKirchhoffCoupledFoldStep is required');
}

/** Reset per-step multipliers, without applying displacements or carrying forces. */
export function beginKirchhoffCoupledFoldStep(constraint) {
    const state = stateFor(constraint), bodies = kirchhoffComponentBodies(constraint);
    state.storage.length = bodies.length;
    for (let side = 0; side < bodies.length; side++) {
        const body = bodies[side];
        if (state.storage[side]?.body !== body || state.storage[side].count !== body.count) {
            state.storage[side] = storageFor(body);
        }
        const storage = state.storage[side];
        storage.lambda.fill(0);
        storage.included.fill(0);
        for (const row of storage.pool) if (row) row.lambda = 0;
    }
    state.rows.length = 0;
    state.begun = true; state.pending = false; state.dt = 0; state.step++;
    return state;
}

function updateRange(storage) {
    const body = storage.body;
    const strength = finite(body.foldLimitStrength ?? 0, 'foldLimitStrength');
    storage.strength = strength;
    if (body.sleeping || body.count < 3 || strength <= 0) {
        storage.first = storage.end = 0;
        return;
    }
    const start = body.activeStart ?? 0, end = body.activeEnd ?? body.count - 1;
    if (!Number.isInteger(start) || !Number.isInteger(end) || start < 0 || end < start || end >= body.count) {
        throw new RangeError('Invalid active fold range');
    }
    const unsupported = Number.isFinite(body.sheathMaterialEndNode)
        ? Math.floor(body.sheathMaterialEndNode) : start + 1;
    storage.first = Math.max(1, start + 1, unsupported);
    storage.end = Math.min(body.count - 1, end);
}

function limitAt(storage, joint) {
    const body = storage.body;
    const raw = body.maxBendAngleByNode?.[joint] ?? body.maxBendAngle;
    if (typeof raw !== 'number' || Number.isNaN(raw)) throw new TypeError('A bend-angle limit in degrees is required');
    const degrees = Math.max(1, Math.min(179, raw));
    if (storage.limitDegrees[joint] !== degrees) {
        storage.limitDegrees[joint] = degrees;
        storage.limitRadians[joint] = degrees * DEGREES;
    }
    return storage.limitRadians[joint];
}

function writeFrames(storage) {
    const body = storage.body, r = storage.frames;
    for (let node = storage.first - 1; node < storage.end; node++) {
        if (storage.first >= storage.end) break;
        let x = finite(body.orientationX[node], 'orientationX');
        let y = finite(body.orientationY[node], 'orientationY');
        let z = finite(body.orientationZ[node], 'orientationZ');
        let w = finite(body.orientationW[node], 'orientationW');
        const length = Math.hypot(x, y, z, w);
        if (!(length > 0) || !Number.isFinite(length)) throw new RangeError('A nonzero finite frame quaternion is required');
        x /= length; y /= length; z /= length; w /= length;
        const i = node * 9;
        r[i] = 1 - 2 * (y * y + z * z); r[i + 1] = 2 * (x * y + w * z); r[i + 2] = 2 * (x * z - w * y);
        r[i + 3] = 2 * (x * y - w * z); r[i + 4] = 1 - 2 * (x * x + z * z); r[i + 5] = 2 * (y * z + w * x);
        r[i + 6] = 2 * (x * z + w * y); r[i + 7] = 2 * (y * z - w * x); r[i + 8] = 1 - 2 * (x * x + y * y);
    }
}

function evaluateJoint(storage, joint, out, rememberAxis) {
    const r = storage.frames, a = (joint - 1) * 9, b = joint * 9;
    const px = r[a + 6], py = r[a + 7], pz = r[a + 8];
    const qx = r[b + 6], qy = r[b + 7], qz = r[b + 8];
    let nx = py * qz - pz * qy, ny = pz * qx - px * qz, nz = px * qy - py * qx;
    const sine = Math.hypot(nx, ny, nz);
    const cosine = Math.max(-1, Math.min(1, px * qx + py * qy + pz * qz));
    out.angle = Math.atan2(sine, cosine);
    out.strain = limitAt(storage, joint) - out.angle;
    out.singularity = null;
    if (sine > AXIS_EPSILON) {
        nx /= sine; ny /= sine; nz /= sine;
    } else if (cosine < 0) {
        out.singularity = 'antiparallel';
        let lx = storage.axisValid[joint] ? storage.axisLocal[joint * 2] : 1;
        let ly = storage.axisValid[joint] ? storage.axisLocal[joint * 2 + 1] : 0;
        const norm = Math.hypot(lx, ly);
        lx /= norm; ly /= norm;
        nx = r[a] * lx + r[a + 3] * ly;
        ny = r[a + 1] * lx + r[a + 4] * ly;
        nz = r[a + 2] * lx + r[a + 5] * ly;
    } else {
        out.singularity = 'parallel';
        nx = ny = nz = 0;
    }
    out.previousX = r[a] * nx + r[a + 1] * ny + r[a + 2] * nz;
    out.previousY = r[a + 3] * nx + r[a + 4] * ny + r[a + 5] * nz;
    out.nextX = -(r[b] * nx + r[b + 1] * ny + r[b + 2] * nz);
    out.nextY = -(r[b + 3] * nx + r[b + 4] * ny + r[b + 5] * nz);
    // n is perpendicular to both d3 axes. Set the analytic spin derivative
    // exactly to zero rather than introducing roundoff-sized twist stiffness.
    if (rememberAxis && out.singularity !== 'parallel') {
        storage.axisLocal[joint * 2] = out.previousX;
        storage.axisLocal[joint * 2 + 1] = out.previousY;
        storage.axisValid[joint] = 1;
    }
    return out;
}

function rowFor(storage, side, joint) {
    let row = storage.pool[joint];
    if (!row) {
        row = storage.pool[joint] = {
            kind: 'fold', side, joint, node: joint, lower: 0, upper: Infinity,
            lambda: 0, alpha: 0, strain: 0, activeHint: false, gradients: [],
            additionalIndex: -1, singularity: null, fractionalHard: false
        };
        for (let axis = 0; axis < 3; axis++) row.gradients.push(Object.seal({ side, dof: (joint - 1) * 6 + 3 + axis, value: 0 }));
        for (let axis = 0; axis < 3; axis++) row.gradients.push(Object.seal({ side, dof: joint * 6 + 3 + axis, value: 0 }));
    }
    return row;
}

/**
 * Rebuild all eligible rows, retaining current-step lambda. If additionalRows
 * is supplied, APPEND and record absolute indices for multiplier application;
 * otherwise return the provider's own rows. All inactive eligible inequalities
 * remain present; every positive strength is a hard limit (alpha=0).
 */
export function buildKirchhoffCoupledFoldRows(constraint, dt, additionalRows = null) {
    const state = stateFor(constraint);
    checkBodies(constraint, state);
    if (!(finite(dt, 'dt') > 0)) throw new RangeError('dt must be positive');
    if (state.dt && state.dt !== dt) throw new Error('A changed timestep requires beginKirchhoffCoupledFoldStep');
    if (additionalRows !== null && (!Array.isArray(additionalRows) || additionalRows === state.rows)) {
        throw new TypeError('Append target must be an independent array');
    }
    state.pending = false;
    state.dt = dt; state.rows.length = 0; state.buildVersion++;
    const bodies = state.storage.map(item => item.body);
    for (let side = 0; side < state.storage.length; side++) {
        const storage = state.storage[side];
        updateRange(storage);
        storage.included.fill(0);
        writeFrames(storage);
        storage.builtFirst = storage.first; storage.builtEnd = storage.end; storage.builtStrength = storage.strength;
        for (let joint = storage.first; joint < storage.end; joint++) {
            const g = evaluateJoint(storage, joint, state.geometry, true);
            const row = rowFor(storage, side, joint);
            const fractional = storage.strength > 0 && storage.strength < 1;
            row.alpha = 0;
            row.lambda = storage.lambda[joint]; row.strain = g.strain; row.angle = g.angle;
            row.limit = storage.limitRadians[joint];
            row.singularity = g.singularity; row.fractionalHard = fractional;
            row.gradients[0].value = g.previousX; row.gradients[1].value = g.previousY; row.gradients[2].value = 0;
            row.gradients[3].value = g.nextX; row.gradients[4].value = g.nextY; row.gradients[5].value = 0;
            row.naturalMapMobility = kirchhoffRowNaturalMapMobility(bodies, row);
            row.additionalIndex = additionalRows ? additionalRows.length : state.rows.length;
            state.rows.push(row);
            if (additionalRows) additionalRows.push(row);
            storage.included[joint] = 1;
        }
        for (let joint = 0; joint < storage.count; joint++) if (!storage.included[joint]) {
            storage.lambda[joint] = 0;
            const old = storage.pool[joint];
            if (old) { old.lambda = 0; old.activeHint = false; }
        }
    }
    state.pending = true;
    return additionalRows ?? state.rows;
}

/** Commit only fold multiplier increments; the joint solver already moved frames. */
export function applyKirchhoffCoupledFoldMultipliers(constraint, increments, scale = 1, buildVersion = undefined) {
    const state = stateFor(constraint);
    checkBodies(constraint, state);
    if (!state.pending || (buildVersion !== undefined && buildVersion !== state.buildVersion)) {
        throw new Error('Fold increments must belong to the latest unapplied build');
    }
    if (!Number.isFinite(scale) || scale < 0 || scale > 1) throw new RangeError('scale must be in [0,1]');
    for (const storage of state.storage) {
        updateRange(storage);
        if (storage.first !== storage.builtFirst || storage.end !== storage.builtEnd || storage.strength !== storage.builtStrength) {
            throw new Error('Fold support or strength changed before multiplier application');
        }
    }
    // Validate the complete update before committing any multiplier.
    for (const row of state.rows) {
        const delta = finite(increments?.[row.additionalIndex], 'fold increment') * scale;
        const next = row.lambda + delta;
        const roundoff = 32 * Number.EPSILON * Math.max(1, Math.abs(row.lambda), Math.abs(delta));
        if (!Number.isFinite(next) || next < -roundoff) throw new RangeError('Fold total multiplier must stay nonnegative');
        if (limitAt(state.storage[row.side], row.joint) !== row.limit) throw new Error('Fold limit changed before multiplier application');
    }
    for (const row of state.rows) {
        row.lambda = Math.max(0, row.lambda + scale * increments[row.additionalIndex]);
        state.storage[row.side].lambda[row.joint] = row.lambda;
    }
    state.pending = false;
}

function vectorAngle(ax, ay, az, bx, by, bz) {
    return Math.atan2(Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx), ax * bx + ay * by + az * bz);
}

/**
 * Fresh nonlinear diagnostics; does not overwrite frozen rows or their indices.
 * Call after build and after each accepted correction. The returned object is
 * borrowed. maximumResidual is angular complementarity in radians; diagnostics
 * include the original positional-angle intent independently of that residual.
 */
export function measureKirchhoffCoupledFoldResidual(constraint) {
    const state = stateFor(constraint);
    checkBodies(constraint, state);
    if (!state.dt) throw new Error('Build fold rows before measuring their residual');
    const m = state.measurement;
    for (const key in m) m[key] = 0;
    for (let side = 0; side < state.storage.length; side++) {
        const storage = state.storage[side], body = storage.body;
        updateRange(storage);
        if (storage.first !== storage.builtFirst || storage.end !== storage.builtEnd || storage.strength !== storage.builtStrength) {
            throw new Error('Fold support changed; rebuild rows before measuring');
        }
        writeFrames(storage);
        for (let joint = storage.first; joint < storage.end; joint++) {
            const g = evaluateJoint(storage, joint, state.geometry, false), row = storage.pool[joint];
            if (row.limit !== storage.limitRadians[joint]) throw new Error('Fold limit changed; rebuild rows before measuring');
            const lambda = storage.lambda[joint], residual = g.strain + row.alpha * lambda;
            m.rowCount++;
            m.maximumResidual = Math.max(m.maximumResidual, lambda > 0 ? Math.abs(residual) : Math.max(0, -residual));
            m.maximumNaturalMapResidual = Math.max(m.maximumNaturalMapResidual,
                Math.abs(Math.min(row.naturalMapMobility * lambda, residual)));
            m.maximumViolation = Math.max(m.maximumViolation, -g.strain);
            m.maximumAngle = Math.max(m.maximumAngle, g.angle);
            if (g.singularity === 'parallel') m.parallelPairs++;
            if (g.singularity === 'antiparallel') m.antiparallelPairs++;
            if (row.fractionalHard) m.fractionalHardRows++;
            if (!body.x || !body.y || !body.z) { m.unavailablePositionPairs++; continue; }
            let ax = body.x[joint] - body.x[joint - 1], ay = body.y[joint] - body.y[joint - 1], az = body.z[joint] - body.z[joint - 1];
            let bx = body.x[joint + 1] - body.x[joint], by = body.y[joint + 1] - body.y[joint], bz = body.z[joint + 1] - body.z[joint];
            const al = Math.hypot(ax, ay, az), bl = Math.hypot(bx, by, bz);
            if (!(al > 0) || !(bl > 0) || !Number.isFinite(al + bl)) {
                m.degeneratePositionPairs++;
                m.maximumPositionalViolation = m.maximumDirectorTangentMismatch = m.maximumPositionalViolationBound = Infinity;
                continue;
            }
            ax /= al; ay /= al; az /= al; bx /= bl; by /= bl; bz /= bl;
            const angle = vectorAngle(ax, ay, az, bx, by, bz);
            const r = storage.frames, prev = (joint - 1) * 9 + 6, next = joint * 9 + 6;
            const mismatchA = vectorAngle(ax, ay, az, r[prev], r[prev + 1], r[prev + 2]);
            const mismatchB = vectorAngle(bx, by, bz, r[next], r[next + 1], r[next + 2]);
            m.maximumPositionalViolation = Math.max(m.maximumPositionalViolation, angle - row.limit);
            m.maximumDirectorTangentMismatch = Math.max(m.maximumDirectorTangentMismatch, mismatchA, mismatchB);
            m.maximumPositionalViolationBound = Math.max(m.maximumPositionalViolationBound, g.angle + mismatchA + mismatchB - row.limit);
        }
    }
    return m;
}
