import assert from 'node:assert/strict';
import test from 'node:test';
import { pathToFileURL } from 'node:url';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { condenseKirchhoffTwoChannelSystem } from '../src/physics/kirchhoffTwoChannelCondensation.js';

const source = process.env.OET_TWO_CHANNEL_SOURCE_ROOT ? pathToFileURL(process.env.OET_TWO_CHANNEL_SOURCE_ROOT + '/') : new URL('../', import.meta.url);
const [{ EndovascularPhysicsWorld }, { assembleKirchhoffTwoChannelSystem }, { solveCoulombNewton }, { measureCoupledLoadKKT }] = await Promise.all([
    import(new URL('src/physics/endovascularPhysicsWorld.js', source)), import(new URL('src/physics/kirchhoffTwoChannelSystem.js', source)),
    import(new URL('src/physics/kirchhoffCoulombNewtonSolver.js', source)), import(new URL('src/physics/kirchhoffCoupledLoadSolver.js', source))]);
const dt = 1 / 120, near = (a, b, tolerance = 1e-8) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const close = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((v, i) => near(v, b[i], tolerance)); };
function denseSolve(matrix, rhs) {
    const n = rhs.length, a = Array.from({ length: n }, (_, i) => [...matrix.slice(i * n, (i + 1) * n), rhs[i]]);
    for (let i = 0; i < n; i++) {
        let p = i; for (let j = i + 1; j < n; j++) if (Math.abs(a[j][i]) > Math.abs(a[p][i])) p = j;
        assert.ok(Math.abs(a[p][i]) > 1e-14, 'independent dense equality block must be nonsingular');
        [a[i], a[p]] = [a[p], a[i]];
        for (let j = i + 1; j < n; j++) { const f = a[j][i] / a[i][i]; for (let k = i; k <= n; k++) a[j][k] -= f * a[i][k]; }
    }
    const x = new Float64Array(n);
    for (let i = n - 1; i >= 0; i--) { let r = a[i][n]; for (let j = i + 1; j < n; j++) r -= a[i][j] * x[j]; x[i] = r / a[i][i]; }
    return x;
}
function scalarFixture() {
    const world = new EndovascularPhysicsWorld(), gap = .01632478;
    const a = world.createRod('condensed-a', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    const b = world.createRod('condensed-b', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    a.setPinned(0, true); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const axis of [1, 2, 3]) body['inverseInertia' + axis].fill(0);
    a.restLength[0] = 10 + 100 * gap / 99; a.setNodePosition(1, 10 - 200 * gap, 0, 0);
    const normal = { strain: 0, alpha: 0, lambda: 0, lower: 0, upper: Infinity, gradients: [{ side: 0, dof: 6, value: 1 }] };
    const constraint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    const options = { basis: 'individual', additionalRows: [normal], channels: native => native.rows.map(row => row.kind === 'material'
        ? { physical: 'pose', bias: { channel: 'bias-motion', strain: 0, alpha: row.alpha, lambda: 0, lower: -Infinity, upper: Infinity } }
        : { physical: 'physical-motion', bias: { channel: 'pose', strain: a.x[1] - 10, alpha: 0, lambda: 0, lower: 0, upper: Infinity } }) };
    return { a, b, constraint, options };
}
function mixedFixture(basis, release = false) {
    const world = new EndovascularPhysicsWorld(), bodies = [0, 1].map(side => {
        const b = world.createRod('mixed-' + side, 3, 1, { mass: side + 1, adaptationCompliance: 0 });
        b.setPinned(0, true);
        for (const k of ['kirchhoffBendCompliance1', 'kirchhoffBendCompliance2', 'kirchhoffTwistCompliance']) b[k].fill(dt * dt * .2);
        for (const k of [1, 2, 3]) b['inverseInertia' + k].fill(1);
        b.setProximalOrientationControl(b.orientationX[0], b.orientationY[0], b.orientationZ[0], b.orientationW[0], 0, 0);
        return b;
    });
    const g = (dof, value = 1) => [{ side: 0, dof, value }, { side: 1, dof, value: -value }];
    const normal = { kind: 'normal', coordinate: 10, strain: -.001, alpha: .01, lambda: .05, lower: 0, upper: Infinity, gradients: g(14) };
    const rows = [normal,
        { kind: 'friction', coordinate: -5, strain: .002, alpha: 0, lambda: .001, lower: -Infinity, upper: Infinity, gradients: g(13) },
        { kind: 'control', coordinate: .33, strain: .0002, alpha: 0, lambda: .003, lower: -Infinity, upper: Infinity, gradients: [{ side: 0, dof: 13, value: 1 }] },
        { kind: 'friction', coordinate: 40, strain: -.003, alpha: 0, lambda: -.002, lower: -Infinity, upper: Infinity, gradients: g(9) },
        { kind: 'fold', coordinate: 50, strain: .02, alpha: .1, lambda: 0, lower: 0, upper: .35, gradients: g(12) }];
    if (release) rows.push({ kind: 'bias-release', coordinate: 12, strain: 0, alpha: 0,
        lambda: 0, lower: 0, upper: 0, gradients: [...g(13, .7), ...g(9, -.4)] });
    const constraint = { innerBody: bodies[0], outerBody: bodies[1], kirchhoffContacts: [] };
    const options = { basis, additionalRows: rows, resolveNormalLoads: true,
        groups: [{ type: 'coulomb-ellipse', rows: [1, 3], mu: [.2, .35], normalLambda: .05, normalRow: normal }],
        channels: native => native.rows.map(row => {
            const source = row.kind === 'material' ? null : rows[row.local];
            if (!source || source.kind === 'control') return { physical: 'pose', bias: { channel: 'bias-motion',
                strain: source ? .0001 : 0, alpha: row.alpha, lambda: source ? -.004 : 0, lower: -Infinity, upper: Infinity } };
            if (source.kind === 'normal') return { physical: 'physical-motion', bias: { channel: 'pose', strain: -.002, alpha: .02, lambda: .03, lower: 0, upper: Infinity } };
            if (source.kind === 'bias-release') return { physical: 'physical-motion', bias: { channel: 'bias-motion',
                strain: 0, alpha: 0, lambda: .06, lower: 0, upper: 0 } };
            return { physical: source.kind === 'fold' ? 'pose' : 'physical-motion', bias: null };
        }) };
    return { constraint, options, rows, bodies };
}
function originalRetainedIndices(dense, condensed) {
    return condensed.retainedRows.map(r => (r.phase === 'physical' ? dense.physicalRows : dense.biasRows)[r.sorted]);
}
function verifySchur(dense, c) {
    assert.equal(c.status, 'condensed', JSON.stringify(c));
    const rr = originalRetainedIndices(dense, c), ee = [...c.equalityOriginalRows].flatMap(original => {
        const i = dense.native.inverseOrder[original]; return [dense.physicalRows[i], dense.biasRows[i]];
    });
    const ne = ee.length, A = Float64Array.from(ee.flatMap(i => ee.map(j => dense.matrix[i * dense.count + j])));
    const free = denseSolve(A, Float64Array.from(ee, i => dense.rhs[i]));
    const dot = (i, x) => ee.reduce((s, e, k) => s + dense.matrix[i * dense.count + e] * x[k], 0);
    rr.forEach((r, i) => near(c.rhs[i], dense.rhs[r] - dot(r, free), 2e-9));
    rr.forEach((r, j) => {
        const column = denseSolve(A, Float64Array.from(ee, e => dense.matrix[e * dense.count + r]));
        rr.forEach((s, i) => near(c.matrix[i * c.count + j], dense.matrix[s * dense.count + r] - dot(s, column), 2e-9));
    });
    assert.deepEqual(c.lower, Float64Array.from(rr, i => dense.lower[i]));
    assert.deepEqual(c.upper, Float64Array.from(rr, i => dense.upper[i]));
    const retained = Float64Array.from({ length: c.count }, (_, i) => (i + 1) * .00013), recovered = c.recover(retained);
    const residual = Float64Array.from(dense.rhs, (v, i) => v - recovered.fullIncrement.reduce((s, x, j) => s + dense.matrix[i * dense.count + j] * x, 0));
    ee.forEach(i => near(residual[i], 0, 2e-9));
    rr.forEach((r, i) => near(residual[r], c.rhs[i] - retained.reduce((s, v, j) => s + c.matrix[i * c.count + j] * v, 0), 2e-9));
    assert.equal(c.diagnostics.factorizations, 1); assert.equal(c.diagnostics.fullExpandedMatrixEntries, 0);
    assert.ok(c.diagnostics.schurEntries < dense.matrix.length);
}

test('native stiff rod condensed and dense Coulomb solves recover the same two-channel solution', () => {
    const f = scalarFixture(), dense = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    const c = condenseKirchhoffTwoChannelSystem(dense.native, dense.descriptors); verifySchur(dense, c);
    const opts = { matrixFormat: 'row-major', tolerance: 1e-9 };
    const full = solveCoulombNewton(dense.matrix, dense.rhs, dense.lower, dense.upper, dense.count, dense.count, dense.groups, opts);
    const small = solveCoulombNewton(c.matrix, c.rhs, c.lower, c.upper, c.count, c.count, c.groups, opts);
    assert.equal(full.diagnostics.converged, true, JSON.stringify(full.diagnostics));
    assert.equal(small.diagnostics.converged, true, JSON.stringify(small.diagnostics));
    const recovered = c.recover(small.increment); close(recovered.fullIncrement, full.increment, 2e-7);
});

test('fixed physical-zero and bias-release rows stay in the exact Schur with their full cross-channel reactions', () => {
    const f = mixedFixture('common-relative', true), dense = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    const c = condenseKirchhoffTwoChannelSystem(dense.native, dense.descriptors);
    verifySchur(dense, c);
    const released = c.retainedRows.map((row, i) => ({ ...row, i })).filter(row => row.original === dense.native.additionalOffset + 5);
    assert.equal(released.length, 2);
    for (const row of released) {
        const increment = row.phase === 'physical' ? 0 : -.06;
        assert.equal(c.lower[row.i], increment); assert.equal(c.upper[row.i], increment);
    }
    // The prescribed release is a load on the remaining equations, not a
    // removable zero-force row. Check its recovered response independently.
    const x = new Float64Array(c.count);
    x[released.find(r => r.phase === 'bias').i] = -.06;
    const r = c.recover(x), referenceRows = originalRetainedIndices(dense, c);
    referenceRows.forEach((row, i) => {
        let full = dense.rhs[row], reduced = c.rhs[i];
        r.fullIncrement.forEach((v, j) => { full -= dense.matrix[row * dense.count + j] * v; });
        x.forEach((v, j) => { reduced -= c.matrix[i * c.count + j] * v; });
        near(full, reduced, 2e-9);
    });
});

for (const basis of ['individual', 'common-relative']) test(`native hard material/controls + all normal/friction/fold rows match dense Schur (${basis})`, () => {
    const f = mixedFixture(basis), dense = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    assert.ok(dense.native.order.some((v, i) => v !== i), 'fixture must exercise original/sorted mappings');
    assert.ok(dense.native.rows.some(r => r.kind === 'material' && r.alpha === 0));
    const c = condenseKirchhoffTwoChannelSystem(dense.native, dense.descriptors); verifySchur(dense, c);
    assert.equal(c.count, 5, 'retain physical normal, bias normal, two friction rows and bounded fold');
    assert.equal(c.groups.length, 1); assert.equal(c.groups[0].rows.length, 2);
    const normal = c.groups[0].normalRow;
    assert.equal(c.retainedRows[normal].phase, 'physical');
    const fold = c.retainedRows.find(r => r.original === dense.native.additionalOffset + 4);
    assert.equal(fold.upper, .35); assert.equal(fold.phase, 'physical');
});

test('owned Schur, groups and recovery survive native scratch reuse and another condensation', () => {
    const f = mixedFixture('common-relative'), dense = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    const c = condenseKirchhoffTwoChannelSystem(dense.native, dense.descriptors); assert.equal(c.status, 'condensed');
    const x = new Float64Array(c.count).fill(.001), first = c.recover(x);
    const saved = structuredClone({ matrix: c.matrix, rhs: c.rhs, groups: c.groups, first });
    f.bodies[0].x[2] += .01; f.rows[0].lambda = .07;
    const next = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options);
    condenseKirchhoffTwoChannelSystem(next.native, next.descriptors);
    assert.deepEqual({ matrix: c.matrix, rhs: c.rhs, groups: c.groups, first: c.recover(x) }, saved);
    first.physicalIncrement.fill(999); assert.deepEqual(c.recover(x), saved.first);
});

test('unsupported equality pair/compliance return explicit fallback without editing input', () => {
    const f = scalarFixture(), dense = assembleKirchhoffTwoChannelSystem(f.constraint, dt, f.options), before = dense.native.matrix.slice();
    const changed = structuredClone(dense.descriptors); changed[0].bias.alpha += .1;
    assert.equal(condenseKirchhoffTwoChannelSystem(dense.native, changed).reason, 'unequal-bias-alpha');
    changed[0] = { physical: 'pose', bias: null };
    assert.equal(condenseKirchhoffTwoChannelSystem(dense.native, changed).reason, 'unsupported-equality-pair');
    assert.deepEqual(dense.native.matrix, before);
});

function synthetic(matrix, rhs, band) {
    const count = rhs.length, native = { count, band, matrix, rhs, lower: Array(count).fill(-Infinity), upper: Array(count).fill(Infinity),
        order: Array.from({ length: count }, (_, i) => i), rows: Array.from({ length: count }, () => ({ kind: 'material', alpha: 0 })), groups: [] };
    const channels = native.rows.map(() => ({ physical: 'pose', bias: { channel: 'bias-motion', alpha: 0, strain: 0, lambda: 0, lower: -Infinity, upper: Infinity } }));
    return { native, channels };
}
test('dependent nonzero equalities do not use the Wasm pivot floor as physical compliance', () => {
    const f = synthetic([1, 0, 1, 1], [0, 0], 2), c = condenseKirchhoffTwoChannelSystem(f.native, f.channels);
    assert.equal(c.status, 'fallback'); assert.equal(c.reason, 'unsafe-equality-factor');
});
test('structurally immovable equations retain their nonzero original RHS and full row identity', () => {
    const f = synthetic([1, 0], [.2, 3.102116925779441e-6], 1), c = condenseKirchhoffTwoChannelSystem(f.native, f.channels);
    assert.equal(c.status, 'condensed'); assert.equal(c.count, 2);
    assert.deepEqual(c.diagnostics.retainedZeroEqualityOriginalRows, [1]);
    near(c.rhs[0], f.native.rhs[1], 0); near(c.rhs[1], 0, 0);
    const result = c.recover(new Float64Array(2)); near(result.physicalIncrement[0], .2); near(result.physicalIncrement[1], 0);
});

test('saved 155-row runtime system condenses exactly and passes the original 298-row load KKT', () => {
    const f = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-two-channel-condensation.json.gz', import.meta.url))),
        (_k, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    const c = condenseKirchhoffTwoChannelSystem(f.native, f.channels); assert.equal(c.status, 'condensed', JSON.stringify(c));
    assert.equal(c.count, 62); assert.equal(c.diagnostics.equalityCount, 118);
    assert.equal(c.diagnostics.equalitySolves + c.diagnostics.zeroEqualityResponses + c.diagnostics.reusedEqualityResponses,
        2 * (c.count + 1), 'every equality response is solved, exactly zero, or shared by a native row pair');
    assert.ok(c.diagnostics.equalitySolves < c.count, 'paired responses avoid solving both channels independently');
    const solve = solveCoulombNewton(c.matrix, c.rhs, c.lower, c.upper, c.count, c.count, c.groups, { matrixFormat: 'row-major', tolerance: 2e-4 });
    assert.equal(solve.diagnostics.converged, true, JSON.stringify(solve.diagnostics));
    const recovered = c.recover(solve.increment), nf = recovered.fullIncrement.length;
    const residual = Float64Array.from(f.oracle.rhs, (v, i) => v - recovered.fullIncrement.reduce((s, x, j) => s + f.oracle.matrix[i * nf + j] * x, 0));
    const lower = [], upper = [], physical = [], bias = []; let fullIndex = 0;
    f.native.order.forEach((original, sorted) => {
        physical[sorted] = fullIndex++; lower.push(f.native.lower[sorted]); upper.push(f.native.upper[sorted]);
        const b = f.channels[original].bias;
        if (b) { bias[sorted] = fullIndex++; lower.push(b.lower - b.lambda); upper.push(b.upper - b.lambda); }
    });
    const groups = f.native.groups.map(g => ({ ...g, rows: g.rows.map(i => physical[i]), normalRow: g.normalRow == null ? undefined : physical[g.normalRow],
        radii: g.normalRow == null ? g.radii : g.mu.map(mu => mu * Math.max(0, g.normalLambda + recovered.fullIncrement[physical[g.normalRow]])) }));
    const kkt = measureCoupledLoadKKT(residual, recovered.fullIncrement, lower, upper, groups);
    assert.ok(kkt.maximumResidual <= 2e-4, JSON.stringify(kkt));
    recovered.fullIncrement.forEach((v, i) => assert.ok(v >= lower[i] - 1e-12 && v <= upper[i] + 1e-12));
});

test('Wasm arena reuses capacity across band changes and failed factors without borrowing published recovery',()=>{
    const make=(n,band=1)=>synthetic(Array.from({length:n*band},(_,i)=>i%band===0?2:0),Array.from({length:n},(_,i)=>i/10),band);
    const first=make(19),a=condenseKirchhoffTwoChannelSystem(first.native,first.channels),saved=a.recover(new Float64Array(a.count));
    const repeat=condenseKirchhoffTwoChannelSystem(first.native,first.channels);
    assert.equal(repeat.diagnostics.kernelArenaReused,true);assert.deepEqual(repeat.recover(new Float64Array(repeat.count)),saved);
    const wide=make(17,17),b=condenseKirchhoffTwoChannelSystem(wide.native,wide.channels);
    assert.equal(b.status,'condensed');
    const tall=make(35),c=condenseKirchhoffTwoChannelSystem(tall.native,tall.channels);assert.equal(c.status,'condensed');
    const wideAgain=condenseKirchhoffTwoChannelSystem(wide.native,wide.channels);assert.equal(wideAgain.diagnostics.kernelArenaReused,true);
    const singular=synthetic([1,0,1,1],[0,0],2);assert.equal(condenseKirchhoffTwoChannelSystem(singular.native,singular.channels).reason,'unsafe-equality-factor');
    const restored=condenseKirchhoffTwoChannelSystem(first.native,first.channels);assert.equal(restored.diagnostics.kernelArenaReused,true);
    assert.deepEqual(restored.recover(new Float64Array(restored.count)),saved);assert.deepEqual(a.recover(new Float64Array(a.count)),saved);
});
