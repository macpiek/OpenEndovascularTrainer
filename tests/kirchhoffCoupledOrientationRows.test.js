import assert from 'node:assert/strict';
import test from 'node:test';
import { existsSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { evaluateBendTwistConstraint } from '../src/physics/discreteKirchhoffRod.js';
import { beginKirchhoffCoupledOrientationStep as begin, buildKirchhoffCoupledOrientationRows as build,
    appendKirchhoffCoupledOrientationRows as append, commitKirchhoffCoupledOrientationMultipliers as commit,
    measureKirchhoffCoupledOrientationResidual as measure } from '../src/physics/kirchhoffCoupledOrientationRows.js';

const dt = 1 / 120;
const close = (a, b, tol = 2e-10) => assert.ok(Math.abs(a - b) <= tol, `${a} != ${b}`);
const closeVector = (a, b, tol) => a.forEach((v, i) => close(v, b[i], tol));
function mul(a, b) {
    const [x, y, z, w] = a, [u, v, t, s] = b;
    return [w * u + x * s + y * t - z * v, w * v - x * t + y * s + z * u,
        w * t + x * v - y * u + z * s, w * s - x * u - y * v - z * t];
}
function exp(v) {
    const length = Math.hypot(...v), factor = length ? Math.sin(length / 2) / length : 0.5;
    return [...v.map(x => x * factor), Math.cos(length / 2)];
}
function log(q) {
    const length = Math.hypot(...q), sign = q[3] < 0 ? -1 : 1, v = q.map(x => x * sign / length);
    const sine = Math.hypot(v[0], v[1], v[2]), f = sine ? 2 * Math.atan2(sine, v[3]) / sine : 2;
    return v.slice(0, 3).map(x => x * f);
}
const inverse = q => [-q[0], -q[1], -q[2], q[3]];
const obj = q => ({ x: q[0], y: q[1], z: q[2], w: q[3] });
function read(body, segment = body.orientationControlSegment) {
    return ['X', 'Y', 'Z', 'W'].map(axis => body['orientation' + axis][segment]);
}
function write(body, q, segment = body.orientationControlSegment) {
    ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { body['orientation' + axis][segment] = q[i]; });
}
function target(body, q) { ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { body['orientationControl' + axis] = q[i]; }); }
function body(q = exp([0.7, -0.3, 0.5]), desired = exp([-0.4, 0.2, 0.6])) {
    const b = { count: 4, segmentCount: 3, activeStart: 0, activeEnd: 3,
        orientationControlSegment: 1, orientationControlCompliance: 1e-4,
        orientationControlLambda: new Float64Array(3), sleeping: false };
    for (const axis of ['X', 'Y', 'Z', 'W']) b['orientation' + axis] = new Float64Array(3);
    for (let i = 0; i < 3; i++) write(b, q, i);
    target(b, desired);
    return b;
}
function fixture() {
    const c = { innerBody: body(), outerBody: body() };
    c.outerBody.orientationControlSegment = -1;
    begin(c);
    return c;
}

test('local RIGHT angular Jacobians match independent quaternion-log finite differences', () => {
    const c = fixture(), b = c.innerBody, desired = ['X', 'Y', 'Z', 'W'].map(axis => b['orientationControl' + axis]);
    const original = read(b), batch = build(c, dt), h = 1e-6;
    const expected = log(mul(inverse(desired), original));
    batch.rows.forEach((row, component) => {
        close(row.strain, expected[component]); close(row.alpha, 1.44);
        assert.equal(row.lower, -Infinity); assert.equal(row.upper, Infinity);
        row.gradients.forEach((g, axis) => {
            assert.equal(g.side, 0); assert.equal(g.dof, 1 * 6 + 3 + axis);
            const v = [0, 0, 0]; v[axis] = h;
            const plus = log(mul(inverse(desired), mul(original, exp(v))));
            v[axis] = -h;
            const minus = log(mul(inverse(desired), mul(original, exp(v))));
            close(g.value, (plus[component] - minus[component]) / (2 * h), 4e-9);
        });
    });
    const native = evaluateBendTwistConstraint(obj(desired), obj(original));
    closeVector(batch.rows.map(r => r.strain), [native.strain.x, native.strain.y, native.strain.z]);
});

test('common rigid rotation and quaternion sign changes preserve strain and local gradients', () => {
    const a = fixture(), b = fixture(), global = exp([0.4, 0.8, -0.6]);
    const desired = ['X', 'Y', 'Z', 'W'].map(axis => b.innerBody['orientationControl' + axis]);
    write(b.innerBody, mul(global, read(b.innerBody)).map(x => -x));
    target(b.innerBody, mul(global, desired));
    const ra = build(a, dt).rows, rb = build(b, dt).rows;
    ra.forEach((r, i) => {
        close(r.strain, rb[i].strain);
        closeVector(r.gradients.map(g => g.value), rb[i].gradients.map(g => g.value));
    });
});

test('zero error is regular; near pi rows stay finite on the selected principal-log branch', () => {
    for (const angle of [0, 1e-9, Math.PI - 1e-5]) {
        const c = fixture(); target(c.innerBody, [0, 0, 0, 1]); write(c.innerBody, exp([angle, 0, 0]));
        const rows = build(c, dt).rows;
        close(rows[0].strain, angle); close(rows[1].strain, 0); close(rows[2].strain, 0);
        assert.ok(rows.every(row => row.gradients.every(g => Number.isFinite(g.value))));
    }
});

test('hard, disabled and inactive controls contribute no rows and never read or prescribe geometry', () => {
    const c = fixture(); c.innerBody.orientationControlCompliance = 0;
    Object.defineProperty(c.innerBody, 'orientationControlX', { get() { throw new Error('hard target read'); } });
    assert.equal(build(c, dt).rows.length, 0); assert.equal(measure(c, dt).rowCount, 0);
    c.innerBody.orientationControlCompliance = 1e-4; c.innerBody.activeStart = 2;
    assert.equal(build(c, dt).rows.length, 0); assert.equal(measure(c, dt).rowCount, 0);
    c.innerBody.activeStart = 0; c.innerBody.orientationControlSegment = -1;
    assert.equal(build(c, dt).rows.length, 0);
});

test('append offset and common scale commit only orientationControlLambda, preserving force ownership', () => {
    const c = fixture(); c.outerBody.orientationControlSegment = 0;
    c.innerBody.orientationControlLambda.set([0.1, -0.2, 0.3]);
    c.outerBody.orientationControlLambda.set([-0.4, 0.5, -0.6]);
    const before = [read(c.innerBody), read(c.outerBody)], batch = build(c, dt), rows = [{ kind: 'tool' }, { kind: 'fold' }];
    append(batch, rows); assert.equal(rows.length, 8);
    const delta = [999, 999, 0.2, -0.4, 0.6, -0.8, 1, -1.2];
    commit(batch, delta, 0.25);
    closeVector([...c.innerBody.orientationControlLambda], [0.15, -0.3, 0.45]);
    closeVector([...c.outerBody.orientationControlLambda], [-0.6, 0.75, -0.9]);
    closeVector(read(c.innerBody), before[0]); closeVector(read(c.outerBody), before[1]);
    assert.equal(rows[0].lambda, undefined); assert.equal(rows[1].lambda, undefined);
    assert.throws(() => commit(batch, delta), /already committed/);
    assert.throws(() => append(batch, []), /already appended/);
});

test('fresh nonlinear residual includes compliance*lambda and leaves assembly snapshots unchanged', () => {
    const c = fixture(), batch = build(c, dt), row = batch.rows[0], oldStrain = row.strain;
    write(c.innerBody, exp([0.1, -0.4, 0.8]));
    const fresh = measure(c, dt), raw = fresh.maximumStrainRad;
    assert.ok(raw > 0.1); assert.equal(row.strain, oldStrain);
    const desired = ['X', 'Y', 'Z', 'W'].map(axis => c.innerBody['orientationControl' + axis]);
    const strain = log(mul(inverse(desired), read(c.innerBody)));
    c.innerBody.orientationControlLambda.set(strain.map(x => -x / 1.44));
    c.innerBody.sleeping = true;
    const equilibrium = measure(c, dt);
    close(equilibrium.maximumResidualRad, 0); close(equilibrium.maximumStrainRad, raw);
    assert.equal(equilibrium.controlCount, 1); assert.equal(equilibrium.residualUnits, 'radians');
    assert.equal(equilibrium, fresh); assert.equal(row.strain, oldStrain);
});

test('begin clears only compliant-control forces, retaining row and gradient buffers across physical steps', () => {
    const c = fixture(); c.outerBody.orientationControlCompliance = 0;
    c.outerBody.orientationControlLambda.set([4, 5, 6]);
    const first = build(c, dt), row = first.rows[0], gradients = [...row.gradients]; row.activeHint = true;
    commit(first, [0.2, -0.1, 0.3]);
    begin(c); closeVector([...c.innerBody.orientationControlLambda], [0, 0, 0]);
    closeVector([...c.outerBody.orientationControlLambda], [4, 5, 6]);
    const next = build(c, dt);
    assert.equal(next, first); assert.equal(next.rows[0], row); assert.equal(row.activeHint, true);
    row.gradients.forEach((g, i) => assert.equal(g, gradients[i])); close(row.lambda, 0);
});

test('atomic validation rejects stale target/support/compliance/multiplier state and invalid data', () => {
    const c = fixture(); c.outerBody.orientationControlSegment = 0;
    let batch = build(c, dt);
    assert.throws(() => commit(batch, [0, 0, 0, 0, NaN, 0]), /finite/);
    closeVector([...c.innerBody.orientationControlLambda], [0, 0, 0]);
    c.outerBody.orientationControlW += 0.1;
    assert.throws(() => commit(batch, [0, 0, 0, 0, 0, 0]), /target/);
    batch = build(c, dt); c.innerBody.orientationControlCompliance *= 2;
    assert.throws(() => commit(batch, [0, 0, 0, 0, 0, 0]), /compliance/);
    batch = build(c, dt); c.innerBody.activeEnd--;
    assert.throws(() => commit(batch, [0, 0, 0, 0, 0, 0]), /topology/);
    batch = build(c, dt); c.innerBody.orientationControlLambda[0] = 1;
    assert.throws(() => commit(batch, [0, 0, 0, 0, 0, 0]), /multiplier changed/);
    const version = batch.version; begin(c);
    assert.throws(() => commit(batch, [], 1, version), /Stale/);
    build(c, dt); assert.throws(() => build(c, dt * 2), /timestep/);
    target(c.innerBody, [0, 0, 0, 0]); assert.throws(() => build(c, dt), /positive finite norm/);
    target(c.innerBody, [NaN, 0, 0, 1]); assert.throws(() => measure(c, dt), /finite/);
});

const parentRoot = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? fileURLToPath(new URL('../', import.meta.url)));
const kernelPath = join(parentRoot, 'src/physics/kirchhoffCoupledSystem.js');
for (const fixedPositions of [true, false]) test(`actual joint kernel equilibrates compliant orientation with material, fixed xyz=${fixedPositions}`, {
    skip: !existsSync(kernelPath) && 'Set OET_BUNDLE_PARENT_PATH in frozen delegation baseline'
}, async () => {
    const { EndovascularPhysicsWorld } = await import(pathToFileURL(join(parentRoot, 'src/physics/endovascularPhysicsWorld.js')));
    const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(pathToFileURL(kernelPath));
    const { measureKirchhoffCoupledMaterialResidual } = await import(pathToFileURL(join(parentRoot, 'src/physics/kirchhoffCoupledResidual.js')));
    const world = new EndovascularPhysicsWorld(), profile = { radius: 0.4, foldLimitStrength: 0, sleepFrames: 1e6 };
    const inner = world.createRod('orientation-inner', 3, 2, profile), outer = world.createRod('orientation-outer', 3, 2, profile);
    if (fixedPositions) inner.inverseMass.fill(0);
    const desired = mul(exp(fixedPositions ? [0, 0, 0.2] : [0.35, 0.15, -0.2]), read(inner, 0));
    inner.setProximalOrientationControl(...desired, 1e-4);
    const c = { innerBody: inner, outerBody: outer, kirchhoffContacts: [] };
    begin(c);
    let converged = false, loaded = false;
    for (let pass = 0; pass < 24; pass++) {
        const batch = build(c, dt), rows = [];
        append(batch, rows);
        const frozenOrientation = read(inner, 0);
        const solved = solveKirchhoffCoupledSystem(c, dt, { additionalRows: rows, tolerance: 1e-9 });
        assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
        closeVector(read(inner, 0), frozenOrientation, 1e-14, 'no orientation pre-apply during assembly');
        applyKirchhoffCoupledCorrection(c, solved);
        commit(batch, solved.additionalIncrement, solved.scale);
        const control = measure(c, dt), material = measureKirchhoffCoupledMaterialResidual(c, dt);
        loaded ||= Math.hypot(...inner.orientationControlLambda) > 1e-5;
        if (control.maximumResidualRad < 1e-7 && material.adaptationMm < 1e-6 && material.bendTwistRad < 1e-7) {
            converged = true; break;
        }
    }
    assert.ok(loaded); assert.ok(converged, JSON.stringify(measure(c, dt)));
    if (fixedPositions) {
        close(measure(c, dt).maximumStrainRad, 0.2, 2e-7);
        close(Math.hypot(...inner.orientationControlLambda), 0.2 / 1.44, 2e-7);
    }
});

test('single component preserves compliant orientation rows and step lifecycle', () => {
    const pair=fixture(), solo={bodies:[pair.innerBody]}; begin(solo);
    const snapshot=rows=>rows.map(({strain,alpha,lower,upper,gradients})=>
        ({strain,alpha,lower,upper,gradients:structuredClone(gradients)}));
    const expected=snapshot(build(pair,dt).rows), batch=build(solo,dt);
    assert.equal(batch.rows.length,3); assert.deepEqual(snapshot(batch.rows),expected);
    const rows=[]; append(batch,rows); commit(batch,[0.2,-0.4,0.6],0.5);
    closeVector([...pair.innerBody.orientationControlLambda],[0.1,-0.2,0.3]);
    assert.equal(measure(solo,dt).controlCount,1);
    solo.bodies=[body()];
    assert.throws(()=>build(solo,dt),/begin/);
    begin(solo); assert.equal(build(solo,dt).rows.length,3);
    assert.throws(()=>begin({bodies:[body()],innerBody:body()}),/aliases/);
});
