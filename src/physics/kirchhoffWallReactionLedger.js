import { rotateVectorByQuaternion, inverseRotateVectorByQuaternion } from './discreteKirchhoffRod.js';

const axes = ['fx', 'fy', 'fz', 'mx', 'my', 'mz'];

export function createKirchhoffWallReactionLedger(body) {
    return { body, lambda: 0, tangentLambda: new Float64Array(2), wrenches: [], normalWrenches: [], retiring: false };
}

function entry(values, node) {
    let value = values.find(v => v.node === node);
    if (!value) values.push(value = { node, fx: 0, fy: 0, fz: 0, mx: 0, my: 0, mz: 0 });
    return value;
}

/** Capture the normal row BEFORE positions change. Wall rows apply only
 * translational forces. Their moments use one fixed world origin. */
export function captureKirchhoffWallReaction(body, gradients, side, out = []) {
    return captureReaction(body, gradients, side, out, false);
}

export function captureKirchhoffWallSurfaceReaction(body, gradients, side, out = []) {
    return captureReaction(body, gradients, side, out, true);
}

function captureReaction(body, gradients, side, out, couples) {
    for (const g of gradients) {
        if (g.side !== side || !Number.isInteger(g.dof) || g.dof < 0 || (!couples && g.dof % 6 > 2) ||
            (g.dof % 6 >= 3 && Math.floor(g.dof / 6) >= body.activeEnd) ||
            Math.floor(g.dof / 6) >= body.count || !Number.isFinite(g.value))
            throw new RangeError('Wall reaction requires finite translations on its actual body');
    }
    for (const value of out) for (const axis of axes) value[axis] = 0;
    for (const g of gradients) entry(out, Math.floor(g.dof / 6))[axes[g.dof % 6]] += g.value;
    for (const value of out) {
        const n = value.node;
        const spin = couples && n < body.activeEnd ? rotateVectorByQuaternion({
            x:body.orientationX[n], y:body.orientationY[n], z:body.orientationZ[n], w:body.orientationW[n]
        }, {x:value.mx,y:value.my,z:value.mz}) : {x:0,y:0,z:0};
        value.mx = spin.x + body.y[n] * value.fz - body.z[n] * value.fy;
        value.my = spin.y + body.z[n] * value.fx - body.x[n] * value.fz;
        value.mz = spin.z + body.x[n] * value.fy - body.y[n] * value.fx;
    }
    return out;
}

/** Record the actual common-scale multiplier increment, never the proposed
 * full correction. Each witness owns its ledger; a new face starts at zero. */
export function recordKirchhoffWallReaction(ledger, frozen, delta) {
    recordReaction(ledger, frozen, delta, true);
}

export function recordKirchhoffWallSurfaceReaction(ledger, frozen, delta) {
    recordReaction(ledger, frozen, delta, false);
}

function recordReaction(ledger, frozen, delta, normal) {
    if (!Number.isFinite(delta) || (normal && (!Number.isFinite(ledger.lambda + delta) || ledger.lambda + delta < 0)) ||
        frozen.some(v => !Number.isInteger(v.node) || v.node < 0 || v.node >= ledger.body.count ||
            axes.some(axis => !Number.isFinite(v[axis]))))
        throw new RangeError('Invalid applied wall reaction');
    if (normal) ledger.lambda += delta;
    for (const source of frozen) {
        const target = entry(ledger.wrenches, source.node);
        const normalTarget = normal ? entry(ledger.normalWrenches, source.node) : null;
        for (const axis of axes) {
            target[axis] += source[axis] * delta;
            if (normalTarget) normalTarget[axis] += source[axis] * delta;
        }
    }
}

/** The fixed carrier bound forces 1 -> 0 in the SAME material/contact solve.
 * This row does not move positions itself. It transports retained world
 * force/moment to current solver coordinates, as the tool release does. */
export function buildKirchhoffWallReactionRelease(ledger, side, out = {}) {
    const body = ledger.body;
    for (const value of ledger.wrenches) if (axes.some(axis => value[axis] !== 0) &&
        (value.node < body.activeStart || value.node > body.activeEnd))
        throw new RangeError('Release wall reaction before removing its material support');
    Object.assign(out, { kind: 'wall-release', owner: ledger, strain: 0, alpha: 0,
        lambda: 1, lower: 0, upper: 0 });
    const gradients = out.gradients ??= [], pool = out.gradientPool ??= [];
    gradients.length = 0;
    function add(dof, value) {
        if (!value) return;
        const g = pool[gradients.length] ??= {};
        Object.assign(g, {side, dof, value}); gradients.push(g);
    }
    const q = out.frame ??= {}, torque = out.torque ??= {}, local = out.localTorque ??= {};
    for (const value of ledger.wrenches) {
        if (!axes.some(axis => value[axis] !== 0)) continue;
        const n = value.node, segment = Math.min(n, body.segmentCount - 1, body.activeEnd - 1);
        if (segment < body.activeStart) throw new RangeError('Wall reaction has no retained material frame');
        Object.assign(q, {x:body.orientationX[segment], y:body.orientationY[segment],
            z:body.orientationZ[segment], w:body.orientationW[segment]});
        torque.x = value.mx - (body.y[n] * value.fz - body.z[n] * value.fy);
        torque.y = value.my - (body.z[n] * value.fx - body.x[n] * value.fz);
        torque.z = value.mz - (body.x[n] * value.fy - body.y[n] * value.fx);
        inverseRotateVectorByQuaternion(q, torque, local);
        add(n*6, value.fx); add(n*6+1, value.fy); add(n*6+2, value.fz);
        add(segment*6+3, local.x); add(segment*6+4, local.y); add(segment*6+5, local.z);
    }
    const normalGradients = out.normalGradients ??= [], normalPool = out.normalGradientPool ??= [];
    normalGradients.length = 0;
    for (const value of ledger.normalWrenches) for (let axis = 0; axis < 3; axis++) {
        if (!value[axes[axis]]) continue;
        const g = normalPool[normalGradients.length] ??= {};
        Object.assign(g, {side, dof:value.node*6+axis, value:value[axes[axis]]});
        normalGradients.push(g);
    }
    ledger.retiring = true;
    return out;
}

/** Commit only after the global correction has been applied with this same
 * scale. Preserve velocity reconstruction's wall-projection channel too. */
export function commitKirchhoffWallReactionRelease(row, scaledIncrement) {
    const fraction = row.lambda + scaledIncrement;
    if (!Number.isFinite(fraction) || fraction < 0 || fraction > 1)
        throw new RangeError('Invalid common-scale wall release');
    const ledger = row.owner, body = ledger.body;
    for (const g of row.normalGradients) {
        const node = Math.floor(g.dof / 6), axis = g.dof % 6;
        if (axis > 2) continue;
        const projection = axis === 0 ? body.wallProjectionX : axis === 1 ? body.wallProjectionY : body.wallProjectionZ;
        projection[node] += body.inverseMass[node] * g.value * scaledIncrement;
    }
    ledger.lambda *= fraction;
    for (const values of [ledger.wrenches, ledger.normalWrenches])
        for (const value of values) for (const axis of axes) value[axis] *= fraction;
    for (let axis = 0; axis < ledger.tangentLambda.length; axis++) ledger.tangentLambda[axis] *= fraction;
    ledger.retiring = fraction !== 0;
}
