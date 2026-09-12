import assert from 'node:assert/strict';
import test, { after } from 'node:test';
import { existsSync, mkdtempSync, copyFileSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const localRoot = fileURLToPath(new URL('../', import.meta.url));
const parentRoot = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? localRoot);
const localModule = join(localRoot, 'src/physics/kirchhoffExternalFrictionRows.js');
let modulePath = localModule;
// The frozen delegation baseline predates SurfaceFriction. Test the delivered
// adapter against the parent's implementation in an isolated temporary ESM
// fixture; never copy dependencies into either source tree. Once integrated,
// ordinary node --test uses the local module directly.
if (!existsSync(join(localRoot, 'src/physics/kirchhoffSurfaceFriction.js'))) {
    const temporary = mkdtempSync(join(tmpdir(), 'oet-external-friction-'));
    after(() => rmSync(temporary, { recursive: true, force: true }));
    writeFileSync(join(temporary, 'package.json'), '{"type":"module"}');
    modulePath = join(temporary, 'kirchhoffExternalFrictionRows.js');
    copyFileSync(localModule, modulePath);
    copyFileSync(join(parentRoot, 'src/physics/kirchhoffSurfaceFriction.js'), join(temporary, 'kirchhoffSurfaceFriction.js'));
}
const { beginKirchhoffExternalFrictionStep: begin, buildKirchhoffExternalFrictionRows: build,
    appendKirchhoffExternalFrictionRows: append, commitKirchhoffExternalFrictionMultipliers: commit,
    measureKirchhoffExternalFrictionResidual: measure } = await import(pathToFileURL(modulePath));

const dt = 1 / 120;
const close = (a, b, tolerance = 2e-10) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const closeVector = (a, b, tolerance) => a.forEach((v, i) => close(v, b[i], tolerance));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const scale = (a, k) => a.map(v => v * k);
function mul(a, b) {
    const [x, y, z, w] = a, [u, v, t, s] = b;
    return [w * u + x * s + y * t - z * v, w * v - x * t + y * s + z * u,
        w * t + x * v - y * u + z * s, w * s - x * u - y * v - z * t];
}
function exp(v) {
    const l = Math.hypot(...v), f = l ? Math.sin(l / 2) / l : 0.5;
    return [...v.map(x => x * f), Math.cos(l / 2)];
}
function rotate(q, v) { return mul(mul(q, [...v, 0]), [-q[0], -q[1], -q[2], q[3]]).slice(0, 3); }
function read(body, segment = 0, previous = false) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    return ['X', 'Y', 'Z', 'W'].map(axis => body[prefix + axis][segment]);
}
function write(body, q, segment = 0, previous = false) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { body[prefix + axis][segment] = q[i]; });
}
function point(body, i) { return ['x', 'y', 'z'].map(axis => body[axis][i]); }
function body(center, direction, t, radius, q) {
    const points = [add(center, scale(direction, -t)), add(center, scale(direction, 1 - t)), add(center, scale(direction, 2 - t))];
    const b = { count: 3, segmentCount: 2, radius, nodeRadius: new Float64Array([radius * 0.9, radius, radius]),
        inverseMass: new Float64Array([1.2, 0.8, 1]), inverseInertia1: new Float64Array([0.4, 0.5]),
        inverseInertia2: new Float64Array([0.7, 0.6]), inverseInertia3: new Float64Array([1.1, 0.9]) };
    ['x', 'y', 'z'].forEach((axis, k) => {
        b[axis] = new Float64Array(points.map(p => p[k]));
        b['previous' + axis.toUpperCase()] = b[axis].slice();
    });
    for (const axis of ['X', 'Y', 'Z', 'W']) {
        b['orientation' + axis] = new Float64Array(2);
        b['previousOrientation' + axis] = new Float64Array(2);
    }
    for (let i = 0; i < 2; i++) { write(b, q, i); write(b, q, i, true); }
    return b;
}
function fixture(reverse = false) {
    const inner = body([1.4, 0, 0.3], [0, 2, 1], 0.25, 0.2, exp([0.2, -0.3, 0.4]));
    const outer = body([0, 0, 0.3], [0, -1, 2], 0.6, 1.1, exp([-0.4, 0.5, 0.2]));
    const c = { innerBody: inner, outerBody: outer, kirchhoffContacts: [] };
    const owner = { enabled: true, bodyA: reverse ? outer : inner, bodyB: reverse ? inner : outer,
        lambdas: new Float64Array([2, 3]), friction: 0.08 };
    const row = { kind: 'tool', owner, node: 0, bodyA: owner.bodyA, bodyB: owner.bodyB, segmentA: 0, segmentB: 0,
        tA: reverse ? 0.6 : 0.25, tB: reverse ? 0.25 : 0.6, normal: new Float64Array([reverse ? -1 : 1, 0, 0]),
        distance: 1.4, strain: 0.1, alpha: 0, lambda: 2, lower: 0, upper: Infinity, gradients: [] };
    for (let side = 0; side < 2; side++) for (let node = 0; node < 2; node++) {
        const t = side ? 0.6 : 0.25, weight = node ? t : 1 - t;
        row.gradients.push({ side, dof: node * 6, value: (side ? -1 : 1) * weight });
    }
    begin(c);
    return { c, inner, outer, owner, row, rows: [row] };
}
function force(entry, components = entry.lambda) {
    return add(scale([...entry.surface.axes[0]], components[0]), scale([...entry.surface.axes[1]], components[1]));
}
function reactionFromRows(c, rows, lambdas) {
    const forces = [[0, 0, 0], [0, 0, 0]], moments = [[0, 0, 0], [0, 0, 0]];
    rows.forEach((row, k) => row.gradients.forEach(g => {
        const b = g.side ? c.outerBody : c.innerBody, node = Math.floor(g.dof / 6), axis = g.dof % 6;
        const value = g.value * lambdas[k], v = [0, 0, 0];
        if (axis < 3) {
            v[axis] = value; forces[g.side] = add(forces[g.side], v);
            moments[g.side] = add(moments[g.side], cross(point(b, node), v));
        } else {
            v[axis - 3] = value; moments[g.side] = add(moments[g.side], rotate(read(b, node), v));
        }
    }));
    return { forces, moments };
}

test('exact facing capsule witness, max endpoint radii, and side identity survive reversed owner order', () => {
    const f = fixture(), reversed = fixture(true);
    const snapshot = structuredClone({ ...f.row, owner: null, bodyA: null, bodyB: null });
    const a = build(f.c, f.rows, dt), b = build(reversed.c, reversed.rows, dt);
    const e = a.entries[0];
    closeVector([...e.geometry.centerA], [1.4, 0, 0.3]);
    closeVector([...e.geometry.centerB], [0, 0, 0.3]);
    closeVector([...e.surface.point], [1.15, 0, 0.3]);
    close(e.geometry.radiusA, 0.2); close(e.geometry.radiusB, 1.1);
    closeVector([...e.surface.levers[0]], [-0.25, 0, 0]);
    closeVector([...e.surface.levers[1]], [1.15, 0, 0]);
    closeVector([...a.groups[0].mu], [0.08, 0.08]);
    assert.equal(a.groups[0].kind, 'coulomb-disk'); assert.equal(a.groups[0].normalLambda, 2);
    assert.equal(a.rows.length, 2); close(a.maximumNormalMomentResidual, 0);
    closeVector([...e.surface.point], [...b.entries[0].surface.point]);
    a.rows.forEach((r, i) => assert.deepEqual(r.gradients, b.rows[i].gradients));
    assert.deepEqual({ ...f.row, owner: null, bodyA: null, bodyB: null }, snapshot);
});

test('all translation and local RIGHT rotation derivatives agree with independently perturbed material surface points', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt), e = batch.entries[0], h = 1e-6;
    for (const row of batch.rows) {
        const direction = [...e.surface.axes[batch.rows.indexOf(row)]];
        const evaluate = () => {
            const samples = [f.inner, f.outer].map((b, side) => {
                const t = side ? 0.6 : 0.25;
                const center = add(scale(point(b, 0), 1 - t), scale(point(b, 1), t));
                return add(center, rotate(read(b), [...e.surface.localLevers[side]]));
            });
            return dot(direction, sub(samples[0], samples[1]));
        };
        for (let side = 0; side < 2; side++) for (let dof = 0; dof < 12; dof++) {
            if (dof >= 9) continue; // No material frame at the second interpolation node.
            const b = side ? f.outer : f.inner, node = Math.floor(dof / 6), axis = dof % 6;
            const gradient = row.gradients.find(g => g.side === side && g.dof === dof)?.value ?? 0;
            let plus, minus;
            if (axis < 3) {
                const values = b[['x', 'y', 'z'][axis]], original = values[node];
                values[node] = original + h; plus = evaluate(); values[node] = original - h; minus = evaluate(); values[node] = original;
            } else {
                const original = read(b), v = [0, 0, 0]; v[axis - 3] = h;
                write(b, mul(original, exp(v))); plus = evaluate(); v[axis - 3] = -h;
                write(b, mul(original, exp(v))); minus = evaluate(); write(b, original);
            }
            close(gradient, (plus - minus) / (2 * h), 5e-10);
        }
    }
});

test('row impulses conserve total force and orbital plus spin moment at unequal radii and arbitrary frames', () => {
    for (const reverse of [false, true]) {
        const f = fixture(reverse), batch = build(f.c, f.rows, dt), lambdas = [0.07, -0.11];
        const { forces, moments } = reactionFromRows(f.c, batch.rows, lambdas);
        closeVector(add(...forces), [0, 0, 0]); closeVector(add(...moments), [0, 0, 0]);
        closeVector(forces[0], force(batch.entries[0], lambdas));
        closeVector(moments[0], cross([...batch.entries[0].surface.point], forces[0]));
        assert.ok(batch.rows.some(r => r.gradients.some(g => g.dof % 6 >= 3 && Math.abs(g.value) > 0.1)));
    }
});

test('end-cap pole uses material tangent basis and remains covariant under a common rigid rotation', () => {
    const a = fixture(), b = fixture(), q = exp([0.6, -0.3, 1.2]), translation = [4, -2, 7];
    for (const f of [a, b]) {
        for (const body of [f.inner, f.outer]) for (let i = 0; i < body.count; i++) {
            body.x[i] = (body === f.inner ? 2 : 0) + i;
            body.y[i] = body.z[i] = 0;
            body.previousX[i] = body.x[i]; body.previousY[i] = body.previousZ[i] = 0;
        }
        f.row.tA = 0; f.row.tB = 1;
    }
    for (const body of [b.inner, b.outer]) {
        for (let i = 0; i < body.count; i++) {
            const p = add(rotate(q, point(body, i)), translation);
            ['x', 'y', 'z'].forEach((axis, k) => { body[axis][i] = body['previous' + axis.toUpperCase()][i] = p[k]; });
        }
        for (let i = 0; i < body.segmentCount; i++) { write(body, mul(q, read(body, i)), i); write(body, read(body, i), i, true); }
    }
    b.row.normal.set(rotate(q, [...b.row.normal]));
    const ba = build(a.c, a.rows, dt), bb = build(b.c, b.rows, dt), ea = ba.entries[0], eb = bb.entries[0];
    assert.equal(ea.geometry.tangentSource, 'outer-material-d1');
    assert.equal(eb.geometry.tangentSource, 'outer-material-d1');
    closeVector([...eb.surface.point], add(rotate(q, [...ea.surface.point]), translation));
    ea.surface.axes.forEach((axis, i) => closeVector([...eb.surface.axes[i]], rotate(q, [...axis])));
    const ra = reactionFromRows(a.c, ba.rows, [0.1, -0.03]), rb = reactionFromRows(b.c, bb.rows, [0.1, -0.03]);
    closeVector(rb.forces[0], rotate(q, ra.forces[0]));
    closeVector(rb.moments[0], add(rotate(q, ra.moments[0]), cross(translation, rb.forces[0])));
});

test('normal alias reads updated owner storage; scaled commit keeps force outside a shrunken disk for relinearization', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt), e = batch.entries[0];
    const additional = [{ kind: 'boundary' }, f.row], groups = [];
    append(batch, additional, groups);
    assert.deepEqual([...groups[0].rowIndices], [2, 3]);
    assert.equal(groups[0].normalContact, e.contact);
    f.owner.lambdas = new Float64Array([0.1, 3]);
    assert.equal(e.contact.normalLambda, 0.1); assert.equal(groups[0].normalLambda, 2, 'QP radius is a frozen snapshot');
    commit(batch, [999, 999, 0.2, -0.1], 0.5);
    closeVector([...e.contact.tangentLambda], [0.1, -0.05]); close(f.owner.lambdas[0], 0.1);
    const result = measure(f.c, additional, dt);
    close(result.maximumFeasibilityResidual, Math.hypot(0.1, -0.05) - 0.008);
    assert.ok(result.maximumResidual > 0.1); close(result.maximumDisplacementResidualMm, 0);
    close(result.maximumConeViolation, Math.hypot(0.1, -0.05) / 0.008 - 1);
    assert.equal(result._batch.groups[0].normalLambda, 0.1);
    closeVector([...e.contact.tangentLambda], [0.1, -0.05]);
    assert.throws(() => commit(batch, [0, 0, 0, 0]), /already committed/);
    assert.throws(() => append(batch, [], []), /already appended/);
});

test('begin clears only tangent forces and retains contact, row identity and working-set hints across permutations', () => {
    const f = fixture(), other = { ...f.row, node: 1 }, batch = build(f.c, [f.row, other], dt);
    const rows = [...batch.rows], contacts = batch.entries.map(e => e.contact), gradient = rows[0].gradients[0];
    rows[0].activeHint = true; commit(batch, [0.04, 0.02, 0.03, 0.01]);
    const normalBefore = f.owner.lambdas.slice(), initialBody = f.inner.x.slice();
    begin(f.c);
    closeVector([...contacts[0].tangentLambda], [0, 0]); closeVector([...contacts[1].tangentLambda], [0, 0]);
    assert.deepEqual(f.owner.lambdas, normalBefore); assert.deepEqual(f.inner.x, initialBody);
    build(f.c, [other, f.row], dt, batch);
    assert.equal(batch.entries[1].contact, contacts[0]); assert.equal(batch.rows[2], rows[0]);
    assert.equal(batch.rows[2].gradients[0], gradient); assert.equal(batch.rows[2].activeHint, true);
    assert.equal(batch.rows[2].lambda, 0);
});

test('zero-contact/zero-mu/unrelated rows do not read geometry, allocate contact state or touch other forces', () => {
    const c = new Proxy({}, { get(target, key) {
        if (key === 'innerBody' || key === 'outerBody') throw new Error('No body access on empty branch');
        return target[key];
    } });
    const noGeometry = { kind: 'tool', owner: { friction: 0 }, get normal() { throw new Error('Geometry was read'); } };
    assert.equal(begin(c), null);
    const batch = build(c, [noGeometry, { kind: 'fold' }, { kind: 'external-friction' }], dt);
    assert.equal(batch.rows.length, 0); assert.equal(batch._entriesByContact, undefined);
    assert.equal(c._coupledExternalFriction, undefined);
    append(batch, [], []); commit(batch, null);
    assert.equal(measure(c, [], dt).maximumResidual, 0);
    const f = fixture();
    const unrelated = { ...f.row, bodyB: {} };
    assert.equal(build(f.c, [unrelated], dt).rows.length, 0);
    assert.equal(f.c._coupledExternalFriction, undefined);
});

test('fresh residual detects changed surface motion and load without overwriting a pending linearization', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt), e = batch.entries[0], row = batch.rows[0];
    const initial = row.strain, tangent = [...e.surface.axes[0]];
    for (let node = 0; node < f.inner.count; node++) ['X', 'Y', 'Z'].forEach((axis, i) => {
        f.inner['previous' + axis][node] -= 0.03 * tangent[i];
    });
    const result = measure(f.c, f.rows, dt);
    assert.ok(result.maximumDisplacementResidualMm > 0.025);
    assert.equal(row.strain, initial); assert.equal(e.contact.tangentLambda[0], 0);
    commit(batch, [0, 0]);
    build(f.c, f.rows, dt, batch);
    close(batch.rows[0].strain, 0.03); close(batch.rows[1].strain, 0);
});

test('finite spin causes surface slip; exact common rigid motion and material-point rolling have zero slip', () => {
    const f = fixture(), base = build(f.c, f.rows, dt), q = read(f.inner);
    write(f.inner, mul(q, exp([0, 0, 0.2])), 0, true);
    const spin = build(f.c, f.rows, dt, base), e = spin.entries[0];
    assert.ok(Math.hypot(...e.surface.relativeSurfaceDisplacement) > 0.005);
    const displacement = [...e.surface.relativeSurfaceDisplacement];
    for (let node = 0; node < f.inner.count; node++) ['X', 'Y', 'Z'].forEach((axis, i) => {
        f.inner['previous' + axis][node] += displacement[i];
    });
    const rolling = build(f.c, f.rows, dt, base);
    closeVector([...rolling.entries[0].surface.relativeSurfaceDisplacement], [0, 0, 0]);
    const global = exp([0.1, -0.2, 0.3]);
    for (const b of [f.inner, f.outer]) {
        for (let i = 0; i < b.count; i++) {
            const previous = add(rotate(global, point(b, i)), [1, 2, -3]);
            ['X', 'Y', 'Z'].forEach((axis, k) => { b['previous' + axis][i] = previous[k]; });
        }
        for (let i = 0; i < b.segmentCount; i++) write(b, mul(global, read(b, i)), i, true);
    }
    closeVector([...build(f.c, f.rows, dt, base).entries[0].surface.relativeSurfaceDisplacement], [0, 0, 0]);
});

test('Coulomb maximum-dissipation residual vanishes for sliding at one shared disk limit', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt), e = batch.entries[0], components = [0.03, -0.04];
    const slip = force(e, components);
    for (let node = 0; node < f.inner.count; node++) ['X', 'Y', 'Z'].forEach((axis, i) => {
        f.inner['previous' + axis][node] -= slip[i];
    });
    build(f.c, f.rows, dt, batch);
    commit(batch, [-0.096, 0.128]); // -(mu*Fn) * [0.6,-0.8]
    const m = measure(f.c, f.rows, dt), r = m._batch.entries[0].residual;
    close(m.maximumResidual, 0); close(r.work, -0.008); close(r.dissipationGap, 0);
    close(Math.hypot(...e.contact.tangentLambda), 0.16);
});

test('commit rejects malformed/stale updates atomically and a recycled normal slot clears history', () => {
    const f = fixture(), other = { ...f.row, node: 1 }, batch = build(f.c, [f.row, other], dt);
    assert.throws(() => commit(batch, [0.1, 0.1, NaN, 0]), /finite/);
    batch.entries.forEach(e => closeVector([...e.contact.tangentLambda], [0, 0]));
    other.tA = 0.3;
    assert.throws(() => commit(batch, [0.1, 0.1, 0.1, 0.1]), /topology/);
    other.tA = 0.25; other.normal = new Float64Array([0, 1, 0]);
    assert.throws(() => commit(batch, [0.1, 0.1, 0.1, 0.1]), /normal/);
    other.normal = f.row.normal; begin(f.c);
    assert.throws(() => commit(batch, [0, 0, 0, 0]), /step/);
    build(f.c, [f.row], dt, batch); commit(batch, [0.1, 0.1]);
    f.row.segmentA = 1;
    build(f.c, [f.row], dt, batch);
    closeVector([...batch.entries[0].contact.tangentLambda], [0, 0]);
    assert.throws(() => build(f.c, [f.row, f.row], dt), /Duplicate/);
    f.row.tA = 1.01; assert.throws(() => build(f.c, f.rows, dt), /foot/);
    f.row.tA = 0.25; f.row.normal.fill(0); assert.throws(() => build(f.c, f.rows, dt), /normal/);
});

test('Fn zero keeps two rows so the same QP can release an existing tangent reaction', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt);
    commit(batch, [0.05, -0.03]); f.owner.lambdas[0] = 0;
    build(f.c, f.rows, dt, batch);
    assert.equal(batch.rows.length, 2); assert.equal(batch.groups[0].normalLambda, 0);
    closeVector(batch.rows.map(r => r.lambda), [0.05, -0.03]);
    const loaded = measure(f.c, f.rows, dt);
    assert.equal(loaded.maximumConeViolation, Infinity); close(loaded.maximumDisplacementResidualMm, 0);
    commit(batch, [-0.05, 0.03]);
    close(measure(f.c, f.rows, dt).maximumResidual, 0);
});

test('exact KKT mm and dimensionless cone acceptance are independent of masses and natural-map scaling', () => {
    const f = fixture(), batch = build(f.c, f.rows, dt), e = batch.entries[0];
    const slip = force(e, [0.03, 0.02]);
    for (let node = 0; node < f.inner.count; node++) ['X', 'Y', 'Z'].forEach((axis, i) => {
        f.inner['previous' + axis][node] -= slip[i];
    });
    build(f.c, f.rows, dt, batch); commit(batch, [0.01, 0.01]); // Interior disk: any slip violates sticking.
    const a = measure(f.c, f.rows, dt, {}, { inverseMobility: 1e-8 });
    for (const b of [f.inner, f.outer]) {
        delete b.inverseMass; delete b.inverseInertia1; delete b.inverseInertia2; delete b.inverseInertia3;
    }
    const b = measure(f.c, f.rows, dt, {}, { inverseMobility: 1e8 });
    const radius = e.surface.group.mu[0] * e.surface.group.normalLambda;
    const expected = 0.03 + Math.hypot(0.03, 0.02) * 0.01 / radius;
    close(a.maximumDisplacementResidualMm, expected); close(b.maximumDisplacementResidualMm, expected);
    close(a.maximumConeViolation, 0); close(b.maximumConeViolation, 0);
    assert.ok(b.maximumResidual > a.maximumResidual * 1e6, 'only the diagnostic natural map changes its scale');
});

const kernelPath = join(parentRoot, 'src/physics/kirchhoffCoupledSystem.js');
for (const normalLoad of [0.01, 0]) test(`joint kernel consumes capsule normal plus disk and conserves momentum, Fn=${normalLoad}`, {
    skip: !existsSync(kernelPath) && 'Set OET_BUNDLE_PARENT_PATH in the frozen baseline'
}, async () => {
    const { EndovascularPhysicsWorld } = await import(pathToFileURL(join(parentRoot, 'src/physics/endovascularPhysicsWorld.js')));
    const { solveKirchhoffCoupledSystem } = await import(pathToFileURL(kernelPath));
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('external-inner', 3, 2, { mass: 0.03, radius: 0.2 });
    const outer = world.createRod('external-outer', 3, 2, { mass: 0.05, radius: 0.8 });
    for (let i = 0; i < 3; i++) {
        inner.y[i] = inner.previousY[i] = 1;
        inner.previousX[i] = inner.x[i] - 0.03;
        inner.previousZ[i] = inner.z[i] - 0.04;
    }
    const c = { innerBody: inner, outerBody: outer, kirchhoffContacts: [] };
    const owner = { enabled: true, friction: 0.08, lambdas: new Float64Array([normalLoad]) };
    const normal = { kind: 'tool', owner, node: 0, bodyA: inner, bodyB: outer, segmentA: 0, segmentB: 0,
        tA: 0.5, tB: 0.5, distance: 1, normal: new Float64Array([0, 1, 0]),
        strain: -0.001, alpha: 0, lambda: normalLoad, lower: 0, upper: Infinity, gradients: [] };
    for (let side = 0; side < 2; side++) for (let node = 0; node < 2; node++)
        normal.gradients.push({ side, dof: node * 6 + 1, value: (side ? -1 : 1) * 0.5 });
    begin(c);
    const batch = build(c, [normal], dt), rows = [normal], groups = [];
    if (normalLoad === 0) {
        commit(batch, [0.0003, -0.0004]);
        build(c, [normal], dt, batch);
    }
    append(batch, rows, groups);
    const solved = solveKirchhoffCoupledSystem(c, dt, { additionalRows: rows, groups, tolerance: 1e-8 });
    assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
    assert.equal(solved.additionalIncrement.length, 3);
    const lambda = [...solved.additionalIncrement.slice(1)];
    const previous = batch.rows.map(r => r.lambda), total = add(previous, lambda);
    close(Math.hypot(...total), 0.08 * normalLoad, 1e-8);
    if (normalLoad > 0) assert.ok(dot(lambda, batch.rows.map(r => r.strain)) < 0);
    else closeVector(lambda, [-0.0003, 0.0004]);
    const reaction = reactionFromRows(c, rows, [...solved.additionalIncrement]);
    closeVector(add(...reaction.forces), [0, 0, 0]); closeVector(add(...reaction.moments), [0, 0, 0]);
    // Root owns the normal update. Tangents commit only the same trust scale.
    owner.lambdas[0] += solved.scale * solved.additionalIncrement[0];
    commit(batch, solved.additionalIncrement, solved.scale);
    closeVector([...batch.entries[0].contact.tangentLambda], add(previous, scale(lambda, solved.scale)));
});

test('explicit components preserve external pair friction and omit absent tool contacts for a singleton',()=>{
 const f=fixture(),component={bodies:[f.inner,f.outer],kirchhoffContacts:[]};
 const legacy=build(f.c,f.rows,dt),actual=build(component,f.rows,dt);
 assert.equal(actual.rows.length,2);
 actual.rows.forEach((row,i)=>{
  assert.deepEqual(row.gradients,legacy.rows[i].gradients);
  close(row.strain,legacy.rows[i].strain);
 });
 const solo={bodies:[f.inner],kirchhoffContacts:[]},empty=build(solo,f.rows,dt);
 assert.equal(empty.rows.length,0);assert.equal(empty.state,null);
 commit(empty,new Float64Array(0));
 component.bodies=[f.outer,f.inner];
 assert.throws(()=>commit(actual,new Float64Array(2)),/step changed/);
});
