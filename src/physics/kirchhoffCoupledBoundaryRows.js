import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { recordKirchhoffToolReaction, commitKirchhoffToolRelease } from './kirchhoffToolContactOwnership.js';
import { captureKirchhoffSplitSheathGeometry } from './kirchhoffSplitMotion.js';
const EPSILON = 1e-12;

function stateFor(constraint, begin = false) {
    const bodies = kirchhoffComponentBodies(constraint);
    const previous = constraint._coupledBoundaries;
    if (previous && (previous.bodies.length !== bodies.length || bodies.some((body, side) =>
        body !== previous.bodies[side] || body.count * 3 !== previous.controls[side].length ||
        body.segmentCount !== previous.wallHints[side].length))) {
        if (!begin) throw new Error('Boundary topology changed; begin a new boundary step');
        delete constraint._coupledBoundaries;
    }
    return constraint._coupledBoundaries ??= {
        bodies, rows: [], pool: [],
        controls: bodies.map(body => new Float64Array(body.count * 3)),
        wallHints: bodies.map(body => new Uint8Array(body.segmentCount)),
        sheaths: new Map()
    };
}

function sheathState(state, sheath, bodies) {
    let result = state.sheaths.get(sheath);
    if (!result) {
        result = bodies.map(body => ({ lambda: new Float64Array(body.count), normal: new Float64Array(body.count * 3), active: new Uint8Array(body.count) }));
        state.sheaths.set(sheath, result);
    }
    return result;
}

/** Boundary multipliers belong to this fixed physical step. */
export function beginKirchhoffCoupledBoundaryStep(constraint) {
    const state = stateFor(constraint, true);
    for (const controls of state.controls) controls.fill(0);
    for (const sheaths of state.sheaths.values()) for (const body of sheaths) body.lambda.fill(0);
    state.rows.length = 0;
    return state;
}

function rowFor(state, kind, side, node, alpha, lambda, strain, owner, component = -1) {
    const index = state.rows.length;
    const row = state.pool[index] ??= { gradients: [], gradientPool: [] };
    row.kind = kind; row.side = side; row.node = node; row.owner = owner; row.component = component;
    row.sheathWitness = null;
    row.alpha = alpha; row.lambda = lambda; row.strain = strain;
    row.lower = kind === 'control' ? -Infinity : 0;
    row.upper = Infinity;
    row.gradients.length = 0;
    row.activeHint = kind === 'wall' ? !!state.wallHints[side][node]
        : kind === 'sheath' ? !!owner.active[node] : false;
    state.rows.push(row);
    return row;
}

function gradient(row, side, node, axis, value) {
    if (!value) return;
    const index = row.gradients.length;
    // Fixed layout lets trial snapshots copy values without rediscovering keys.
    const item = row.gradientPool[index] ??= Object.seal({ side: 0, dof: 0, value: 0 });
    item.side = side; item.dof = node * 6 + axis; item.value = value;
    row.gradients.push(item);
}

/** Frozen boundary equations for the same simultaneous material/contact step.
 * Wall geometry is supplied by the existing vessel query; no collision sample
 * is removed. Position controls use the equivalent isotropic vector spring,
 * keeping three multiplier components instead of a changing radial scalar.
 * Translational rows use world axes and obey the additionalRows contract.
 */
export function collectKirchhoffCoupledBoundaryRows(constraint, sheaths, dt, activation = 0.2, includeWalls = true) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('A positive finite timestep is required');
    const state = stateFor(constraint), bodies = kirchhoffComponentBodies(constraint);
    state.rows.length = 0;
    const sheathGeometry = constraint._splitMotion ? state.sheathGeometry ??= [] : null;
    if (sheathGeometry) sheathGeometry.length = 0;
    const inverseDtSquared = 1 / (dt * dt);
    for (let side = 0; side < bodies.length; side++) {
        const body = bodies[side];
        for (let node = body.activeStart; node <= body.activeEnd; node++) {
            if (!body.controlEnabled[node] || body.inverseMass[node] <= 0) continue;
            for (let axis = 0; axis < 3; axis++) {
                const key = axis === 0 ? 'X' : axis === 1 ? 'Y' : 'Z';
                const positions = axis === 0 ? body.x : axis === 1 ? body.y : body.z;
                const row = rowFor(state, 'control', side, node,
                    body.controlCompliance[node] * inverseDtSquared,
                    state.controls[side][node * 3 + axis], positions[node] - body[`control${key}`][node], body, axis);
                gradient(row, side, node, axis, 1);
            }
        }
        const first = Math.max(body.activeStart, body.collisionStartSegment, 0);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        for (let node = first; node < end; node++) {
            if (!includeWalls || !body.wallActive[node]) continue;
            const t = body.wallT[node], a = 1 - t;
            const nx = body.wallNormalX[node], ny = body.wallNormalY[node], nz = body.wallNormalZ[node];
            const gap = (a * body.x[node] + t * body.x[node + 1] - body.wallX[node]) * nx +
                (a * body.y[node] + t * body.y[node + 1] - body.wallY[node]) * ny +
                (a * body.z[node] + t * body.z[node + 1] - body.wallZ[node]) * nz -
                Math.max(body.nodeRadius[node], body.nodeRadius[node + 1]);
            const row = rowFor(state, 'wall', side, node, body.wallCompliance * inverseDtSquared,
                body.wallLambda[node], gap, body);
            gradient(row, side, node, 0, a * nx); gradient(row, side, node, 1, a * ny); gradient(row, side, node, 2, a * nz);
            gradient(row, side, node + 1, 0, t * nx); gradient(row, side, node + 1, 1, t * ny); gradient(row, side, node + 1, 2, t * nz);
        }
    }
    for (const sheath of sheaths) {
        const geometry = sheathGeometry ? captureKirchhoffSplitSheathGeometry(sheath, bodies) : null;
        if (geometry) sheathGeometry.push(geometry);
        const multipliers = sheathState(state, sheath, bodies);
        for (let side = 0; side < bodies.length; side++) {
            const body = bodies[side], storage = multipliers[side];
            if (sheath.bodies && !sheath.bodies.includes(body)) continue;
            const end = Math.min(body.activeEnd, body.sheathMaterialEndNode);
            for (let node = body.activeStart; node <= end; node++) {
                if (body.inverseMass[node] <= 0) continue;
                const px = body.x[node] - sheath.startX, py = body.y[node] - sheath.startY, pz = body.z[node] - sheath.startZ;
                const axial = px * sheath.axisX + py * sheath.axisY + pz * sheath.axisZ;
                if (axial < -sheath.proximalExtension - 1e-5 || axial > sheath.length + 1e-5) {
                    storage.lambda[node] = 0;
                    continue;
                }
                const rx = px - axial * sheath.axisX, ry = py - axial * sheath.axisY, rz = pz - axial * sheath.axisZ;
                const distance = Math.hypot(rx, ry, rz);
                const gap = Math.max(0, sheath.innerRadius - body.nodeRadius[node]) - distance;
                if (gap > activation && storage.lambda[node] <= 0) continue;
                if (distance <= EPSILON && storage.lambda[node] <= 0) continue;
                const offset = node * 3;
                if (distance > EPSILON) {
                    storage.normal[offset] = -rx / distance;
                    storage.normal[offset + 1] = -ry / distance;
                    storage.normal[offset + 2] = -rz / distance;
                }
                const row = rowFor(state, 'sheath', side, node, 0, storage.lambda[node], gap, storage);
                if (geometry) row.sheathWitness = {geometry,axial,radius:distance,clearance:Math.max(0,sheath.innerRadius-body.nodeRadius[node])};
                for (let axis = 0; axis < 3; axis++) gradient(row, side, node, axis, storage.normal[offset + axis]);
            }
        }
    }
    return state.rows;
}

/** Commit exactly the increments already included in the simultaneous update.
 * This function does not project or move either body a second time.
 */
export function applyKirchhoffCoupledBoundaryMultipliers(constraint, increments, scale) {
    const state = stateFor(constraint);
    for (let index = 0; index < state.rows.length; index++) {
        const row = state.rows[index], delta = scale * increments[index];
        const next = row.lambda + delta;
        if (row.kind === 'control') {
            state.controls[row.side][row.node * 3 + row.component] = next;
            const values = state.controls[row.side], start = row.node * 3;
            row.owner.controlLambda[row.node] = -Math.hypot(values[start], values[start + 1], values[start + 2]);
        } else if (row.kind === 'wall') {
            row.owner.wallLambda[row.node] = Math.max(0, next);
            state.wallHints[row.side][row.node] = Number(row.activeHint);
            for (const g of row.gradients) {
                const node = Math.floor(g.dof / 6), axis = g.dof % 6;
                const projection = axis === 0 ? row.owner.wallProjectionX : axis === 1 ? row.owner.wallProjectionY : row.owner.wallProjectionZ;
                projection[node] += row.owner.inverseMass[node] * g.value * delta;
            }
        } else if (row.kind === 'tool') {
            recordKirchhoffToolReaction(row.owner, row.node, row.reactionWrenches, delta, true);
            row.owner.lambdas[row.node] = Math.max(0, next);
            for (const g of row.gradients) {
                const body = state.bodies[g.side];
                const node = Math.floor(g.dof / 6), axis = g.dof % 6;
                const projection = axis === 0 ? body.toolProjectionX : axis === 1 ? body.toolProjectionY : body.toolProjectionZ;
                projection[node] += body.inverseMass[node] * g.value * delta;
            }
        } else if (row.kind === 'tool-release') {
            commitKirchhoffToolRelease(constraint, row, delta);
        } else if (row.kind === 'split-sweep' || row.kind === 'split-point-wall') {
            row.lambda = Math.max(0, next);
        } else if (row.kind === 'sheath') {
            row.owner.lambda[row.node] = Math.max(0, next);
            row.owner.active[row.node] = Number(row.activeHint);
        }
    }
}

/** Complementarity/equality residual of the currently collected boundaries. */
export function measureKirchhoffCoupledBoundaryResidual(rows, out = null) {
    if (out) {
        for (const key of ['kind', 'side', 'node', 'strain', 'alpha', 'lambda', 'residual',
            'wallT', 'wallBranchId', 'wallFaceIndex', 'wallNormalX', 'wallNormalY', 'wallNormalZ', 'wallGap',
            'ax', 'ay', 'az', 'bx', 'by', 'bz', 'radius']) out[key] = null;
        out.maximumResidual = 0;
    }
    let maximum = 0;
    for (const row of rows) {
        if (!['control', 'wall', 'tool', 'sheath', 'split-sweep', 'split-point-wall'].includes(row.kind)) continue;
        const residual = row.strain + row.alpha * row.lambda;
        const violation = row.lower === -Infinity || row.lambda > 1e-10
            ? Math.abs(residual) : Math.max(0, -residual);
        if (out && (out.kind === null || violation > maximum)) {
            out.kind = row.kind; out.side = row.side ?? null; out.node = row.node ?? null;
            out.strain = row.strain; out.alpha = row.alpha; out.lambda = row.lambda; out.residual = residual;
            for (const key of ['wallT', 'wallBranchId', 'wallFaceIndex', 'wallNormalX', 'wallNormalY', 'wallNormalZ', 'wallGap'])
                out[key] = row.kind === 'wall' ? row.owner?.[key]?.[row.node] ?? null : null;
            for (const axis of ['x', 'y', 'z']) {
                out['a' + axis] = row.kind === 'wall' ? row.owner?.[axis]?.[row.node] ?? null : null;
                out['b' + axis] = row.kind === 'wall' ? row.owner?.[axis]?.[row.node + 1] ?? null : null;
            }
            out.radius = row.kind === 'wall' ? Math.max(row.owner.nodeRadius[row.node], row.owner.nodeRadius[row.node + 1]) : null;
        }
        maximum = Math.max(maximum, violation);
    }
    if (out) out.maximumResidual = maximum;
    return maximum;
}

/** Diagonal J W J^T + alpha in the native physical metric. A zero-mobility
 * row uses a unit conversion scale only for its merit, never as compliance or
 * an applied reaction. This keeps violated immovable rows visible and retains
 * exactly the same complementarity zeros. No force threshold is used.
 */
export function kirchhoffRowNaturalMapMobility(bodies, row) {
    let mobility = row.alpha ?? 0;
    for (let i = 0; i < row.gradients.length; i++) {
        const g = row.gradients[i];
        let duplicate = false;
        for (let j = 0; j < i; j++) if (row.gradients[j].side === g.side && row.gradients[j].dof === g.dof) { duplicate = true; break; }
        if (duplicate) continue;
        let value = g.value;
        for (let j = i + 1; j < row.gradients.length; j++) {
            const other = row.gradients[j];
            if (other.side === g.side && other.dof === g.dof) value += other.value;
        }
        const body = bodies[g.side], node = Math.floor(g.dof / 6), axis = g.dof % 6;
        if (!body) throw new TypeError('Natural-map row references an absent body');
        const start = Math.max(0, body.activeStart), end = Math.min(body.segmentCount ?? body.activeEnd, body.activeEnd);
        if (node < start || node > end || axis >= 3 && node === end) continue;
        const controlled = body.orientationControlCompliance === 0 && node === body.orientationControlSegment;
        const weight = axis < 3 ? body.inverseMass?.[node] ?? 0 : controlled ? 0 : body[`inverseInertia${axis - 2}`]?.[node] ?? 0;
        if (!Number.isFinite(weight) || weight < 0 || !Number.isFinite(value)) throw new RangeError('Invalid natural-map mobility');
        mobility += value * value * weight;
    }
    if (!Number.isFinite(mobility) || mobility < 0) throw new RangeError('Invalid natural-map mobility');
    return mobility > 0 ? mobility : 1;
}

/** Continuous globalization metric; the final KKT test above stays unchanged.
 * Optional scales are in row order and may be frozen at the base linearization.
 */
export function measureKirchhoffCoupledBoundaryMerit(constraint, rows, scales = null) {
    const bodies = kirchhoffComponentBodies(constraint);
    let maximum = 0;
    for (let i = 0; i < rows.length; i++) {
        const row = rows[i];
        if (!['control', 'wall', 'tool', 'sheath', 'split-sweep', 'split-point-wall'].includes(row.kind)) continue;
        const residual = row.strain + row.alpha * row.lambda;
        const mobility = scales?.[i] ?? kirchhoffRowNaturalMapMobility(bodies, row);
        if (!(mobility > 0) || !Number.isFinite(mobility)) throw new RangeError('Natural-map scale must be positive and finite');
        // m*(lambda - max(0, lambda-r/m)), evaluated without cancellation.
        maximum = Math.max(maximum, Math.abs(row.lower === -Infinity ? residual : Math.min(mobility * row.lambda, residual)));
    }
    return maximum;
}
