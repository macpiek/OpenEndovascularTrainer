import assert from 'node:assert/strict';
import test from 'node:test';
import { existsSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import {
    beginKirchhoffCoupledFoldStep, buildKirchhoffCoupledFoldRows,
    applyKirchhoffCoupledFoldMultipliers, measureKirchhoffCoupledFoldResidual
} from '../src/physics/kirchhoffCoupledFoldRows.js';

const rad = Math.PI / 180;
const close = (a, b, tolerance = 2e-10) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
function mul(a, b) {
    const [x, y, z, w] = a, [u, v, t, s] = b;
    return [w * u + x * s + y * t - z * v, w * v - x * t + y * s + z * u,
        w * t + x * v - y * u + z * s, w * s - x * u - y * v - z * t];
}
function exp(v) {
    const length = Math.hypot(...v), factor = length ? Math.sin(length / 2) / length : 0.5;
    return [...v.map(x => x * factor), Math.cos(length / 2)];
}
function rotate(q, v) {
    const norm = Math.hypot(...q), unit = q.map(x => x / norm);
    return mul(mul(unit, [...v, 0]), [-unit[0], -unit[1], -unit[2], unit[3]]).slice(0, 3);
}
function read(body, i) { return [body.orientationX[i], body.orientationY[i], body.orientationZ[i], body.orientationW[i]]; }
function write(body, i, q) {
    [body.orientationX[i], body.orientationY[i], body.orientationZ[i], body.orientationW[i]] = q;
}
function body(frames = [[0, 0, 0, 1], exp([0.4, 0.1, -0.2])]) {
    const count = frames.length + 1;
    const result = { count, activeStart: 0, activeEnd: count - 1, sheathMaterialEndNode: Infinity,
        foldLimitStrength: 1, maxBendAngle: 30,
        maxBendAngleByNode: new Float64Array(count).fill(30),
        x: new Float64Array(count), y: new Float64Array(count), z: new Float64Array(count) };
    for (const axis of ['X', 'Y', 'Z', 'W']) result[`orientation${axis}`] = new Float64Array(frames.length);
    frames.forEach((q, i) => {
        write(result, i, q);
        const tangent = rotate(q, [0, 0, 1]);
        result.x[i + 1] = result.x[i] + tangent[0];
        result.y[i + 1] = result.y[i] + tangent[1];
        result.z[i + 1] = result.z[i] + tangent[2];
    });
    return result;
}
function fixture(inner = body()) {
    const outer = body(); outer.foldLimitStrength = 0;
    const constraint = { innerBody: inner, outerBody: outer };
    beginKirchhoffCoupledFoldStep(constraint);
    return constraint;
}
function angle(body, joint) {
    const a = rotate(read(body, joint - 1), [0, 0, 1]), b = rotate(read(body, joint), [0, 0, 1]);
    return Math.acos(Math.max(-1, Math.min(1, dot(a, b))));
}

test('local right-rotation gradients match independent finite differences of acos director angle', () => {
    const c = fixture(body([exp([0.31, -0.24, 0.13]), exp([-0.42, 0.51, 0.38])]));
    const row = buildKirchhoffCoupledFoldRows(c, 1 / 120)[0];
    close(row.strain, 30 * rad - angle(c.innerBody, 1));
    const gradients = row.gradients.map(g => ({ ...g }));
    const h = 1e-6;
    for (const g of gradients) {
        const node = Math.floor(g.dof / 6), axis = g.dof % 6 - 3, original = read(c.innerBody, node);
        const perturb = [0, 0, 0]; perturb[axis] = h;
        write(c.innerBody, node, mul(original, exp(perturb)));
        const plus = 30 * rad - angle(c.innerBody, 1);
        perturb[axis] = -h;
        write(c.innerBody, node, mul(original, exp(perturb)));
        const minus = 30 * rad - angle(c.innerBody, 1);
        write(c.innerBody, node, original);
        close(g.value, (plus - minus) / (2 * h), 4e-9);
    }
});

test('common world rotation leaves strain and local gradients unchanged, including antiparallel branch', () => {
    for (const frames of [[exp([0.2, -0.3, 0.4]), exp([-0.4, 0.7, 0.5])], [[0, 0, 0, 1], exp([Math.PI, 0, 0])]]) {
        const global = exp([0.7, -0.2, 1.1]);
        const first = fixture(body(frames)), second = fixture(body(frames.map(q => mul(global, q))));
        const a = buildKirchhoffCoupledFoldRows(first, 0.01)[0], b = buildKirchhoffCoupledFoldRows(second, 0.01)[0];
        close(a.strain, b.strain);
        a.gradients.forEach((g, i) => close(g.value, b.gradients[i].value));
        assert.equal(a.singularity, b.singularity);
    }
});

test('fold gradients are equal/opposite world moments, with no axial or twist lock', () => {
    const c = fixture(body([exp([0.2, -0.3, 0.4]), exp([-0.4, 0.7, 0.5])]));
    const row = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    const originalStrain = row.strain;
    assert.ok(row.gradients.every(g => g.dof % 6 >= 3));
    assert.equal(row.gradients[2].value, 0); assert.equal(row.gradients[5].value, 0);
    const previous = rotate(read(c.innerBody, 0), row.gradients.slice(0, 3).map(g => g.value));
    const next = rotate(read(c.innerBody, 1), row.gradients.slice(3, 6).map(g => g.value));
    previous.forEach((v, i) => close(v + next[i], 0));
    close(dot(previous, rotate(read(c.innerBody, 0), [0, 0, 1])), 0);
    for (let i = 0; i < 2; i++) write(c.innerBody, i, mul(read(c.innerBody, i), exp([0, 0, i ? -1.2 : 0.8])));
    c.innerBody.x.fill(1000); c.innerBody.y.fill(-50); c.innerBody.z.fill(13);
    const spun = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    close(spun.strain, originalStrain);
    assert.equal(spun.gradients[2].value, 0); assert.equal(spun.gradients[5].value, 0);
});

test('unsupportedStart uses floor, active range is preserved, limits clamp to 1..179 degrees', () => {
    const b = body(Array.from({ length: 6 }, (_, i) => exp([0.1 * i, 0, 0])));
    b.activeStart = 1; b.sheathMaterialEndNode = 3.8;
    b.maxBendAngleByNode[3] = -10; b.maxBendAngleByNode[4] = 200;
    const c = fixture(b), rows = buildKirchhoffCoupledFoldRows(c, 0.01);
    assert.deepEqual(rows.map(row => row.joint), [3, 4, 5]);
    close(rows[0].limit, rad); close(rows[1].limit, 179 * rad);
    assert.ok(rows.every(row => row.lower === 0 && row.upper === Infinity));
    assert.equal(rows[0].gradients[0].dof, 2 * 6 + 3);
    assert.equal(rows.at(-1).gradients[3].dof, 5 * 6 + 3);
    assert.ok(rows.some(row => row.strain > 0), 'inactive eligible rows remain present');
    b.sheathMaterialEndNode = 6;
    assert.equal(buildKirchhoffCoupledFoldRows(c, 0.01).length, 0);
});

test('all positive strengths retain a hard constraint; zero/negative and sleeping disable it', () => {
    const c = fixture();
    for (const strength of [1, 0.7, 0.01, 1.2]) {
        c.innerBody.foldLimitStrength = strength;
        const rows = buildKirchhoffCoupledFoldRows(c, 0.01);
        assert.equal(rows.length, 1); assert.equal(rows[0].alpha, 0);
    }
    for (const strength of [0, -0.3]) {
        c.innerBody.foldLimitStrength = strength;
        assert.equal(buildKirchhoffCoupledFoldRows(c, 0.01).length, 0);
    }
    c.innerBody.foldLimitStrength = 1; c.innerBody.sleeping = true;
    assert.equal(buildKirchhoffCoupledFoldRows(c, 0.01).length, 0);
});

test('parallel and tiny bends stay finite; the antiparallel branch has the correct one-sided opening derivative', () => {
    const straight = fixture(body([[0, 0, 0, 1], [0, 0, 0, 1]]));
    const row = buildKirchhoffCoupledFoldRows(straight, 0.01)[0];
    assert.equal(row.singularity, 'parallel');
    assert.ok(row.gradients.every(g => g.value === 0));
    close(row.strain, 30 * rad);
    assert.equal(measureKirchhoffCoupledFoldResidual(straight).parallelPairs, 1);
    const tiny = fixture(body([[0, 0, 0, 1], exp([1e-10, 0, 0])]));
    const small = buildKirchhoffCoupledFoldRows(tiny, 0.01)[0];
    close(small.angle, 1e-10, 1e-15); assert.ok(small.gradients.every(g => Number.isFinite(g.value)));
    const c = fixture(body([[0, 0, 0, 1], exp([Math.PI, 0, 0])]));
    const anti = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    assert.equal(anti.singularity, 'antiparallel'); close(anti.angle, Math.PI);
    const oldStrain = anti.strain, h = 1e-6;
    const branch = anti.gradients.slice(0, 3).map(g => g.value);
    write(c.innerBody, 0, exp(branch.map(v => v * h)));
    const opened = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    close((opened.strain - oldStrain) / h, 1, 2e-8);
});

test('antiparallel continuity uses the previous regular local bending axis', () => {
    const c = fixture(body([[0, 0, 0, 1], exp([0, Math.PI - 0.01, 0])]));
    const regular = buildKirchhoffCoupledFoldRows(c, 0.01)[0].gradients.slice(0, 3).map(g => g.value);
    write(c.innerBody, 1, exp([0, Math.PI, 0]));
    const anti = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    assert.equal(anti.singularity, 'antiparallel');
    anti.gradients.slice(0, 3).forEach((g, i) => close(g.value, regular[i]));
});

test('append indices, scaled multiplier commit and per-step reset preserve unrelated rows and geometry', () => {
    const c = fixture(), prefix = [{ kind: 'other' }, { kind: 'other' }], combined = [...prefix];
    const state = c._coupledFoldRows;
    const rows = buildKirchhoffCoupledFoldRows(c, 0.01, combined);
    assert.equal(rows, combined); assert.equal(rows[0], prefix[0]);
    const fold = rows[2], q = read(c.innerBody, 0), x = [...c.innerBody.x];
    assert.equal(fold.additionalIndex, 2);
    fold.activeHint = true;
    applyKirchhoffCoupledFoldMultipliers(c, [99, 99, 0.004], 0.25, state.buildVersion);
    close(fold.lambda, 0.001);
    assert.deepEqual(read(c.innerBody, 0), q); assert.deepEqual([...c.innerBody.x], x);
    assert.throws(() => applyKirchhoffCoupledFoldMultipliers(c, [99, 99, 0.004]), /unapplied/);
    const fresh = [...prefix]; buildKirchhoffCoupledFoldRows(c, 0.01, fresh);
    assert.equal(fresh[2], fold); close(fresh[2].lambda, 0.001);
    applyKirchhoffCoupledFoldMultipliers(c, [99, 99, -0.001], 1);
    close(fold.lambda, 0);
    beginKirchhoffCoupledFoldStep(c);
    const next = buildKirchhoffCoupledFoldRows(c, 1 / 120)[0];
    assert.equal(next, fold); assert.equal(next.lambda, 0); assert.equal(next.activeHint, true);
});

test('loaded nonlinear residual demands release even when the geometric inequality is slack', () => {
    const c = fixture(body([[0, 0, 0, 1], exp([0.8, 0, 0])]));
    const row = buildKirchhoffCoupledFoldRows(c, 0.01)[0];
    close(measureKirchhoffCoupledFoldResidual(c).maximumResidual, 0.8 - 30 * rad);
    applyKirchhoffCoupledFoldMultipliers(c, [0.001]);
    write(c.innerBody, 1, exp([0.2, 0, 0]));
    const frozenStrain = row.strain;
    const m = measureKirchhoffCoupledFoldResidual(c);
    assert.equal(m.maximumViolation, 0);
    close(m.maximumResidual, 30 * rad - 0.2);
    assert.equal(row.strain, frozenStrain, 'measurement must not rewrite the assembly snapshot');
    buildKirchhoffCoupledFoldRows(c, 0.01);
    applyKirchhoffCoupledFoldMultipliers(c, [-0.001]);
    assert.equal(measureKirchhoffCoupledFoldResidual(c).maximumResidual, 0);
});

test('director angle equals positional angle on adaptation manifold; positional mismatch is independently reported', () => {
    const c = fixture(body([[0, 0, 0, 1], exp([0.8, 0, 0])]));
    buildKirchhoffCoupledFoldRows(c, 0.01);
    const aligned = measureKirchhoffCoupledFoldResidual(c);
    close(aligned.maximumDirectorTangentMismatch, 0);
    close(aligned.maximumPositionalViolation, aligned.maximumViolation);
    const straight = fixture(body([[0, 0, 0, 1], [0, 0, 0, 1]]));
    straight.innerBody.z[2] = 0;
    buildKirchhoffCoupledFoldRows(straight, 0.01);
    const m = measureKirchhoffCoupledFoldResidual(straight);
    assert.equal(m.maximumResidual, 0);
    close(m.maximumPositionalViolation, Math.PI - 30 * rad);
    close(m.maximumDirectorTangentMismatch, Math.PI);
    assert.ok(m.maximumPositionalViolationBound >= m.maximumPositionalViolation);
    straight.innerBody.z[1] = 0;
    const degenerate = measureKirchhoffCoupledFoldResidual(straight);
    assert.equal(degenerate.degeneratePositionPairs, 1);
    assert.equal(degenerate.maximumPositionalViolation, Infinity);
});

test('row, gradient, storage and measurement buffers persist across builds and changing support retires lambda', () => {
    const c = fixture(body(Array.from({ length: 4 }, (_, i) => exp([i * 0.3, 0, 0]))));
    const first = buildKirchhoffCoupledFoldRows(c, 0.01);
    const row = first[0], gradients = row.gradients, entries = [...gradients];
    const storage = c._coupledFoldRows.storage[0], frames = storage.frames, lambda = storage.lambda;
    const measurement = measureKirchhoffCoupledFoldResidual(c);
    for (let i = 0; i < 10; i++) {
        buildKirchhoffCoupledFoldRows(c, 0.01);
        assert.equal(first[0], row); assert.equal(row.gradients, gradients);
        row.gradients.forEach((g, j) => assert.equal(g, entries[j]));
        assert.equal(storage.frames, frames); assert.equal(storage.lambda, lambda);
        assert.equal(measureKirchhoffCoupledFoldResidual(c), measurement);
    }
    applyKirchhoffCoupledFoldMultipliers(c, [0.001, 0, 0]);
    c.innerBody.sheathMaterialEndNode = 2;
    buildKirchhoffCoupledFoldRows(c, 0.01);
    assert.equal(storage.lambda[1], 0); assert.equal(row.activeHint, false);
});

test('invalid frames and stale/negative multiplier updates fail before partial mutation', () => {
    const c = fixture(body(Array.from({ length: 3 }, (_, i) => exp([i * 0.2, 0, 0]))));
    const rows = buildKirchhoffCoupledFoldRows(c, 0.01);
    assert.throws(() => applyKirchhoffCoupledFoldMultipliers(c, [0.01, -0.02]), /nonnegative/);
    assert.ok(rows.every(row => row.lambda === 0));
    assert.throws(() => applyKirchhoffCoupledFoldMultipliers(c, [0, 0], 1, -1), /latest/);
    c.innerBody.sheathMaterialEndNode = 2;
    assert.throws(() => applyKirchhoffCoupledFoldMultipliers(c, [0, 0]), /support/);
    c.innerBody.sheathMaterialEndNode = Infinity;
    assert.throws(() => buildKirchhoffCoupledFoldRows(c, 0.02), /timestep/);
    write(c.innerBody, 0, [0, 0, 0, 0]);
    assert.throws(() => buildKirchhoffCoupledFoldRows(c, 0.01), /nonzero/);
});

const parentRoot = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? fileURLToPath(new URL('../', import.meta.url)));
const coupledPath = join(parentRoot, 'src/physics/kirchhoffCoupledSystem.js');
test('current parent kernel solves unilateral director folds in the same material/contact step', {
    skip: !existsSync(coupledPath) && 'Set OET_BUNDLE_PARENT_PATH in the frozen baseline'
}, async () => {
    const { EndovascularPhysicsWorld } = await import(pathToFileURL(join(parentRoot, 'src/physics/endovascularPhysicsWorld.js')));
    const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(pathToFileURL(coupledPath));
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('fold-joint-inner', 5, 2, { mass: 0.03, foldLimitStrength: 1, maxBendAngle: 15, sleepFrames: 1e6 });
    const outer = world.createRod('fold-joint-outer', 4, 2.5, { mass: 0.05, foldLimitStrength: 0, sleepFrames: 1e6 });
    const base = read(inner, 0), naturalTurn = 20 * rad;
    for (let i = 0; i < inner.segmentCount; i++) {
        write(inner, i, mul(base, exp([0, i * naturalTurn, 0])));
        inner.restRotation2[i] = naturalTurn;
        const tangent = rotate(read(inner, i), [0, 0, 1]);
        inner.x[i + 1] = inner.x[i] + 2 * tangent[0];
        inner.y[i + 1] = inner.y[i] + 2 * tangent[1];
        inner.z[i + 1] = inner.z[i] + 2 * tangent[2];
    }
    const c = { innerBody: inner, outerBody: outer, kirchhoffContacts: [] };
    beginKirchhoffCoupledFoldStep(c);
    let converged = false, loaded = false;
    for (let pass = 0; pass < 24; pass++) {
        const rows = buildKirchhoffCoupledFoldRows(c, 1 / 120);
        const solved = solveKirchhoffCoupledSystem(c, 1 / 120, { additionalRows: rows, tolerance: 1e-8 });
        assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
        assert.equal(solved.additionalIncrement.length, 3);
        applyKirchhoffCoupledCorrection(c, solved);
        applyKirchhoffCoupledFoldMultipliers(c, solved.additionalIncrement, solved.scale);
        const m = measureKirchhoffCoupledFoldResidual(c);
        loaded ||= rows.some(row => row.lambda > 1e-12);
        if (m.maximumResidual < 1e-5 && m.maximumPositionalViolation < 1e-4) { converged = true; break; }
    }
    assert.ok(loaded, 'the preferred over-limit curvature should load at least one hard fold');
    assert.ok(converged, JSON.stringify(measureKirchhoffCoupledFoldResidual(c)));
});

test('single component retains fold geometry, commits and rejects a changed component until begin', () => {
    const b = body([[0,0,0,1],exp([0.8,0.1,0])]), pair = fixture(b), solo = {bodies:[b]};
    beginKirchhoffCoupledFoldStep(solo);
    const snapshot = rows => rows.map(({strain,alpha,lower,upper,gradients})=>
        ({strain,alpha,lower,upper,gradients:structuredClone(gradients)}));
    const expected = snapshot(buildKirchhoffCoupledFoldRows(pair,1/120));
    const rows = buildKirchhoffCoupledFoldRows(solo,1/120);
    assert.ok(rows.length>0); assert.deepEqual(snapshot(rows),expected);
    applyKirchhoffCoupledFoldMultipliers(solo,rows.map(()=>0.2),0.5);
    assert.ok(buildKirchhoffCoupledFoldRows(solo,1/120).every(row=>row.lambda===0.1));
    assert.ok(Number.isFinite(measureKirchhoffCoupledFoldResidual(solo).maximumResidual));
    solo.bodies.push(body());
    assert.throws(()=>buildKirchhoffCoupledFoldRows(solo,1/120),/topology/);
    beginKirchhoffCoupledFoldStep(solo);
    assert.doesNotThrow(()=>buildKirchhoffCoupledFoldRows(solo,1/120));
});

test('fold natural map uses frozen inertia diagonal and remains continuous for tiny positive loads', () => {
    const b=body(), c={bodies:[b]};
    b.inverseInertia1=new Float64Array(b.count).fill(2);
    b.inverseInertia2=new Float64Array(b.count).fill(3);
    b.inverseInertia3=new Float64Array(b.count).fill(4);
    beginKirchhoffCoupledFoldStep(c);
    const rows=buildKirchhoffCoupledFoldRows(c,1/120), row=rows[0];
    assert.ok(row.strain>0);
    const m=row.naturalMapMobility;
    close(m,row.gradients.reduce((sum,g)=>sum+g.value*g.value*b[`inverseInertia${g.dof%6-2}`][Math.floor(g.dof/6)],0));
    assert.equal(measureKirchhoffCoupledFoldResidual(c).maximumNaturalMapResidual,0);
    for(const lambda of [1e-9,1e-12,1e-18]) {
        c._coupledFoldRows.storage[0].lambda[row.joint]=lambda;
        const measured=measureKirchhoffCoupledFoldResidual(c);
        close(measured.maximumNaturalMapResidual,m*lambda,1e-25);
        assert.ok(measured.maximumResidual>0.01,'strict final KKT remains unchanged');
    }
    c._coupledFoldRows.storage[0].lambda[row.joint]=0;
    write(b,1,exp([.8,0,0]));
    buildKirchhoffCoupledFoldRows(c,1/120);
    const violated=measureKirchhoffCoupledFoldResidual(c);
    close(violated.maximumNaturalMapResidual,violated.maximumViolation);
    write(b,1,exp([30*rad,0,0]));
    buildKirchhoffCoupledFoldRows(c,1/120);
    c._coupledFoldRows.storage[0].lambda[row.joint]=1;
    assert.ok(measureKirchhoffCoupledFoldResidual(c).maximumNaturalMapResidual<1e-14);
    b.inverseInertia1.fill(0);b.inverseInertia2.fill(0);b.inverseInertia3.fill(0);
    assert.equal(buildKirchhoffCoupledFoldRows(c,1/120)[0].naturalMapMobility,1);
});

test('actual Kirchhoff apply differentiates director fold consistently while spatial fold has a distinct xyz derivative', async () => {
    const {EndovascularPhysicsWorld}=await import('../src/physics/endovascularPhysicsWorld.js');
    const {assembleKirchhoffDirect,applyKirchhoffDirectCorrection}=await import('../src/physics/kirchhoffDirectSolver.js');
    const w=new EndovascularPhysicsWorld(),b=w.createRod('fold-apply-audit',3,5,{});
    const frames=[exp([.31,-.24,.13]),exp([-.42,.51,.38])];
    frames.forEach((q,i)=>write(b,i,q));
    b.foldLimitStrength=1;b.maxBendAngle=30;b.maxBendAngleByNode.fill(30);b.sheathMaterialEndNode=Infinity;
    // Deliberately off the adaptation manifold: spatial tangents and directors
    // are independent state variables until the material equations close.
    b.x.set([0,4,5]);b.y.set([0,0,3]);b.z.set([0,0,1]);
    const c={bodies:[b]};beginKirchhoffCoupledFoldStep(c);
    const row=buildKirchhoffCoupledFoldRows(c,1/120)[0];
    const gradient=row.gradients.map(g=>({...g}));
    const source=assembleKirchhoffDirect(b,1/120),lambda=new Float64Array(source.rowCount);
    const position=[b.x.slice(),b.y.slice(),b.z.slice()],savedFrames=frames.map(q=>q.slice());
    const evaluate=(direction,h)=>{
        [b.x,b.y,b.z].forEach((array,axis)=>array.set(position[axis]));savedFrames.forEach((q,i)=>write(b,i,q));
        applyKirchhoffDirectCorrection(b,direction,lambda,h);
        const measured=measureKirchhoffCoupledFoldResidual(c);
        return {director:30*rad-angle(b,1),spatial:measured.maximumPositionalViolation};
    };
    const direction=Float64Array.from({length:18},(_,i)=>Math.sin(i+.4)),h=1e-6;
    const predicted=gradient.reduce((sum,g)=>sum+g.value*direction[g.dof],0),plus=evaluate(direction,h),minus=evaluate(direction,-h);
    close(predicted,(plus.director-minus.director)/(2*h),2e-9);
    const xyz=new Float64Array(18);xyz[6]=1;
    const p=evaluate(xyz,h),m=evaluate(xyz,-h);
    close((p.director-m.director)/(2*h),0,1e-12);
    assert.ok(Math.abs((p.spatial-m.spatial)/(2*h))>.1,'spatial fold is not the director row derivative');
});
