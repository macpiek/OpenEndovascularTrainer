import assert from 'node:assert/strict';
import test from 'node:test';
import { existsSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import {
    createKirchhoffBundleRuntime, assembleKirchhoffBundleColumns, addKirchhoffBundleGram,
    encodeKirchhoffBundleDofs, decodeKirchhoffBundleDofs, recoverKirchhoffBundleCorrection, measureKirchhoffBundleMobility
} from '../src/physics/kirchhoffBundleRuntime.js';

const close = (a, b, tol = 2e-12) => assert.ok(Math.abs(a - b) <= tol * Math.max(1, Math.abs(b)), `${a} != ${b}`);
const closeArray = (a, b, tol) => {
    assert.equal(a.length, b.length);
    a.forEach((value, i) => close(value, b[i], tol));
};
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);

function fixture(innerArc = [0, 1, 2, 3], outerArc = [0, 1, 2, 3], rows = 9) {
    const coordinates = [innerArc, outerArc].map(x => Float64Array.from(x));
    const bodies = coordinates.map(arc => ({
        count: arc.length, restLength: Float64Array.from(arc.slice(1), (v, i) => v - arc[i])
    }));
    const material = bodies.map((body, side) => body.count === 0 ? null : ({
        start: 0, end: body.count - 1,
        weight: Float64Array.from({ length: body.count * 6 }, (_, dof) => dof % 6 < 3 ? side ? 0.3 : 1.7 : 0.1 + (dof % 3) * 0.04)
    }));
    const columns = bodies.map((body, side) => Array.from({ length: body.count * 6 }, (_, dof) => {
        const result = [];
        for (let row = 0; row < rows; row++) {
            if ((row * 7 + dof * 3 + side) % 5 > 1) continue;
            const value = Math.sin(0.2 + row * 0.31 + dof * 0.71 + side);
            if ((dof + row) % 3 === 0) result.push(row, value * 0.4, row, value * 0.6);
            else result.push(row, value);
        }
        return result;
    }));
    return { system: { bodies, material, columns, count: rows }, options: { axialCoordinates: coordinates } };
}

// Independent dense oracle: original node coordinates, original J, original W.
// It knows nothing of pairing, common/relative coefficients or transformed CSR.
function oracle(system) {
    const ni = system.bodies[0].count * 6, no = system.bodies[1].count * 6;
    const J = Array.from({ length: system.count }, () => new Float64Array(ni + no));
    const W = new Float64Array(ni + no);
    for (let side = 0; side < 2; side++) {
        const s = system.material[side], offset = side ? ni : 0;
        for (let dof = 0; dof < system.columns[side].length; dof++) {
            W[offset + dof] = s && dof >= s.start * 6 && dof < (s.end + 1) * 6 ? s.weight[dof] : 0;
            const entries = system.columns[side][dof];
            for (let k = 0; k < entries.length; k += 2) J[entries[k]][offset + dof] += entries[k + 1];
        }
    }
    const A = J.map(row => J.map(other => row.reduce((sum, value, i) => sum + value * W[i] * other[i], 0)));
    return { J, W, A };
}

function checkGram(w, system, tolerance = 3e-13) {
    const { A } = oracle(system);
    const matrix = new Float64Array(system.count * w.band);
    addKirchhoffBundleGram(w, matrix);
    for (let i = 0; i < system.count; i++) {
        for (let j = 0; j <= i; j++) {
            const actual = i - j < w.band ? matrix[i * w.band + i - j] : 0;
            const scale = Math.sqrt(A[i][i]) * Math.sqrt(A[j][j]);
            const error = Math.abs(actual - A[i][j]);
            assert.ok(error <= tolerance * scale || error === 0, `Gram [${i},${j}]: ${actual} vs ${A[i][j]}, scale=${scale}`);
        }
    }
    return matrix;
}

function originalCorrection(system, multipliers) {
    const { J, W } = oracle(system);
    return W.map((weight, dof) => weight * J.reduce((sum, row, i) => sum + row[dof] * multipliers[i], 0));
}

test('full transformed Gram equals independent dense J W Jᵀ and does not mutate source columns', () => {
    const { system, options } = fixture();
    const before = structuredClone(system.columns);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    checkGram(w, system);
    assert.equal(w.pairCount, 4);
    assert.equal(w.columnCount, 48);
    assert.equal(w.rowCount, system.count);
    assert.deepEqual(system.columns, before);
    const gram = new Float64Array(system.count * w.band);
    for (let i = 0; i < system.count; i++) gram[i * w.band] = 0.1 + i * 0.02;
    addKirchhoffBundleGram(w, gram);
    const { A } = oracle(system);
    for (let i = 0; i < system.count; i++) close(gram[i * w.band], A[i][i] + 0.1 + i * 0.02);
});

test('unequal node counts/spacing give distinct monotone partners and retain every unpaired/angular column', () => {
    const { system, options } = fixture([0, 0.7, 1.4, 2.3, 3.2, 4.2, 5.3], [0.2, 1.7, 3.1, 4.8]);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    let previous = -1;
    let paired = 0;
    const outerUsed = new Set();
    for (let i = 0; i < system.bodies[0].count; i++) {
        const j = w.partner[0][i];
        if (j < 0) continue;
        assert.ok(j > previous);
        assert.equal(w.partner[1][j], i);
        assert.ok(!outerUsed.has(j)); outerUsed.add(j);
        previous = j; paired++;
    }
    assert.equal(paired, 4);
    assert.equal(w.columnCount, (7 + 4) * 6);
    assert.equal([...w.kind.slice(0, w.columnCount)].filter(kind => kind < 2).length, 3 * (7 + 4) + 3 * (7 - 4));
    checkGram(w, system);
});

test('different arclength offsets change pairing but never the physical operator', () => {
    const { system, options } = fixture([0, 1, 2, 3, 4, 5], [0, 1.4, 2.8, 4.2]);
    const w = createKirchhoffBundleRuntime();
    assembleKirchhoffBundleColumns(w, system, options);
    const first = [...w.partner[0].slice(0, 6)];
    checkGram(w, system);
    assembleKirchhoffBundleColumns(w, system, { ...options, axialOffsets: [0, 2] });
    assert.notDeepEqual([...w.partner[0].slice(0, 6)], first);
    checkGram(w, system);
    assembleKirchhoffBundleColumns(w, system, { ...options, axialOffsets: [0, 100] });
    assert.equal(w.pairCount, 0);
    assert.equal(w.columnCount, 60);
    checkGram(w, system);
});

test('a supplied rest-arclength map and computed rest coordinates produce the same matching', () => {
    const data = fixture([0, 1, 2.2, 4.1], [0, 0.8, 1.9, 3.6]);
    const a = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), data.system, data.options);
    const b = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), data.system);
    assert.deepEqual(a.partner[0], b.partner[0]);
    assert.deepEqual(a.arc[1], b.arc[1]);
    checkGram(b, data.system);
});

test('full basis preserves increments, angular local coordinates, virtual work and kinetic energy', () => {
    const { system, options } = fixture([0, 0.6, 1.4, 2.5, 4], [0, 1.2, 2.6, 4]);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    const inputs = system.bodies.map((body, side) => Float64Array.from({ length: body.count * 6 }, (_, i) => Math.cos(i * 0.23 + side)));
    const outputs = inputs.map(v => new Float64Array(v.length));
    const q = encodeKirchhoffBundleDofs(w, ...inputs);
    decodeKirchhoffBundleDofs(w, q, ...outputs);
    closeArray(outputs[0], inputs[0]); closeArray(outputs[1], inputs[1]);
    const { J, W } = oracle(system);
    const x = [...inputs[0], ...inputs[1]];
    const multipliers = Float64Array.from({ length: system.count }, (_, i) => Math.sin(i + 1));
    const originalWork = J.reduce((sum, row, i) => sum + multipliers[i] * dot(row, x), 0);
    let transformedWork = 0;
    for (let k = 0; k < w.columnCount; k++) {
        let force = 0;
        for (let j = w.columnOffsets[k]; j < w.columnOffsets[k + 1]; j++) force += w.values[j] * multipliers[w.rowIndices[j]];
        transformedWork += force * q[k];
    }
    close(transformedWork, originalWork);
    const originalEnergy = x.reduce((sum, value, i) => sum + value * value / (2 * W[i]), 0);
    let transformedEnergy = 0;
    for (let k = 0; k < w.columnCount; k++) transformedEnergy += q[k] ** 2 / (2 * w.weights[k]);
    close(transformedEnergy, originalEnergy);
    for (let side = 0; side < 2; side++) for (let i = 3; i < inputs[side].length; i++) {
        if (i % 6 >= 3) assert.equal(outputs[side][i], inputs[side][i]);
    }
});

test('mass-centered common velocity carries total momentum for unequal masses', () => {
    const { system, options } = fixture([0, 2], [0, 2]);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    const inner = Float64Array.from({ length: 12 }, (_, i) => i / 7);
    const outer = Float64Array.from({ length: 12 }, (_, i) => 1 - i / 3);
    const q = encodeKirchhoffBundleDofs(w, inner, outer);
    for (let k = 0; k < w.columnCount; k++) {
        if (w.kind[k] !== 2) continue;
        const di = w.innerDof[k], dO = w.outerDof[k];
        const expectedMomentum = inner[di] / system.material[0].weight[di] + outer[dO] / system.material[1].weight[dO];
        close(q[k] / w.weights[k], expectedMomentum);
    }
});

test('recovered direction uses inverse common/relative map and equals original W Jᵀ λ', () => {
    const { system, options } = fixture([0, 0.7, 1.4, 2.8, 4], [0, 1.3, 2.6, 4]);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    const increment = Float64Array.from({ length: system.count }, (_, i) => Math.sin(i * 0.8));
    const inner = new Float64Array(30), outer = new Float64Array(24);
    recoverKirchhoffBundleCorrection(w, increment, inner, outer);
    closeArray([...inner, ...outer], [...originalCorrection(system, increment)]);
});

for (const [wi, wo] of [[0, 2], [3, 0], [0, 0], [1e-12, 1e12], [1e100, 1e-100], [1e-100, 1e100]]) {
    test(`zero/prescribed or strongly unequal inverse masses (${wi}, ${wo}) preserve the operator and direction`, () => {
        const { system, options } = fixture([0, 1, 2], [0, 1, 2], 5);
        for (let side = 0; side < 2; side++) for (let dof = 0; dof < 18; dof++) {
            system.material[side].weight[dof] = dof % 6 < 3 ? side ? wo : wi : 0;
        }
        const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
        checkGram(w, system);
        assert.equal(w.omittedFixedPairDofs, wi === 0 && wo === 0 ? 18 : 0);
        const increment = Float64Array.from([0.3, -0.1, 0.2, 0.6, -0.4]);
        const inner = new Float64Array(18), outer = new Float64Array(18);
        recoverKirchhoffBundleCorrection(w, increment, inner, outer);
        const actual = [...inner, ...outer], expected = originalCorrection(system, increment);
        for (let i = 0; i < actual.length; i++) {
            if (expected[i] === 0) assert.equal(actual[i], 0);
            else assert.ok(Math.abs(actual[i] - expected[i]) <= 3e-13 * Math.abs(expected[i]), `${i}: ${actual[i]} vs ${expected[i]}`);
        }
        if (wi === 0 && wo === 0) {
            inner[0] = 0.1;
            assert.throws(() => encodeKirchhoffBundleDofs(w, inner, outer), /Both-fixed/);
        }
    });
}

test('finite clearance and axial slide remain unconstrained by pairing; normal reactions conserve momentum', () => {
    const { system, options } = fixture([0, 1, 2], [0, 1, 2], 1);
    system.columns.forEach(columns => columns.forEach(column => { column.length = 0; }));
    system.columns[0][7] = [0, -1]; system.columns[1][7] = [0, 1];
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    const positions = [new Float64Array(18), new Float64Array(18)];
    for (let node = 0; node < 3; node++) {
        positions[0][node * 6] = node + 2.7;
        positions[1][node * 6] = node - 1.2;
        positions[0][node * 6 + 1] = 0.02;
        positions[0][node * 6 + 5] = 0.6;
        positions[1][node * 6 + 5] = -0.3;
    }
    const rebuilt = positions.map(v => new Float64Array(v.length));
    decodeKirchhoffBundleDofs(w, encodeKirchhoffBundleDofs(w, ...positions), ...rebuilt);
    closeArray(rebuilt[0], positions[0]); closeArray(rebuilt[1], positions[1]);
    close(0.0405 - Math.abs(rebuilt[0][7] - rebuilt[1][7]), 0.0205);
    const increment = Float64Array.of(0.0001);
    recoverKirchhoffBundleCorrection(w, increment, ...rebuilt);
    close(rebuilt[0][7] / system.material[0].weight[7] + rebuilt[1][7] / system.material[1].weight[7], 0);
    assert.equal(rebuilt[0][6], 0); assert.equal(rebuilt[1][6], 0);
    assert.equal(rebuilt[0][11], 0); assert.equal(rebuilt[1][11], 0);
});

test('transformed support expands the required band before Gram allocation', () => {
    const { system, options } = fixture([0], [0], 8);
    system.columns.forEach(columns => columns.forEach(column => { column.length = 0; }));
    system.columns[0][0] = [0, 1]; system.columns[1][0] = [7, 2];
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    assert.equal(w.band, 8);
    assert.throws(() => addKirchhoffBundleGram(w, new Float64Array(8), 1), /too small/);
    checkGram(w, system);
});

test('persistent buffers and pairing revision avoid allocations/rebuilds while refreshing masses and J', () => {
    const { system, options } = fixture();
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, { ...options, pairingRevision: 1 });
    const keys = ['columnOffsets', 'kind', 'innerDof', 'outerDof', 'decodeInner', 'decodeOuter',
        'encodeInner', 'encodeOuter', 'weights', 'generalized', 'rowIndices', 'values', 'omittedAxes'];
    const buffers = keys.map(key => w[key]);
    const arcBuffers = [...w.arc], partnerBuffers = [...w.partner];
    const oldStats = { ...w.stats };
    for (let repeat = 0; repeat < 12; repeat++) {
        system.material[0].weight[0] *= 1.01;
        system.columns[1][0][1] += 0.02;
        assembleKirchhoffBundleColumns(w, system, { ...options, pairingRevision: 1 });
        checkGram(w, system);
    }
    keys.forEach((key, i) => assert.equal(w[key], buffers[i]));
    w.arc.forEach((buffer, i) => assert.equal(buffer, arcBuffers[i]));
    w.partner.forEach((buffer, i) => assert.equal(buffer, partnerBuffers[i]));
    assert.equal(w.stats.bufferGrowths, oldStats.bufferGrowths);
    assert.equal(w.stats.pairingBuilds, oldStats.pairingBuilds);
    assert.equal(w.stats.coordinateScans, oldStats.coordinateScans);
    assert.equal(w.stats.assemblies, oldStats.assemblies + 12);
});

test('without a revision, in-place arclength edits are detected; active ranges invalidate even a retained revision', () => {
    const { system, options } = fixture([0, 1, 2, 3, 4], [0, 1, 2, 3, 4]);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), system, options);
    const builds = w.stats.pairingBuilds;
    options.axialCoordinates[1][2] = 2.1;
    assembleKirchhoffBundleColumns(w, system, options);
    assert.equal(w.stats.pairingBuilds, builds + 1);
    assembleKirchhoffBundleColumns(w, system, { ...options, pairingRevision: 7 });
    system.material[0].start = 2;
    assembleKirchhoffBundleColumns(w, system, { ...options, pairingRevision: 7 });
    assert.equal(w.partner[0][0], -1); assert.equal(w.partner[0][1], -1);
    assert.equal(w.pairCount, 3);
    checkGram(w, system);
});

test('cached pairing refreshes prescribed masses, including restoring formerly both-fixed DOFs', () => {
    const data = fixture([0, 1], [0, 1]);
    const options = { ...data.options, pairingRevision: 'fixed-topology' };
    for (const s of data.system.material) s.weight.fill(0, 0, 3);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), data.system, options);
    assert.equal(w.omittedFixedPairDofs, 6);
    checkGram(w, data.system);
    data.system.material[0].weight.fill(0.7, 0, 3);
    assembleKirchhoffBundleColumns(w, data.system, options);
    assert.equal(w.omittedFixedPairDofs, 0);
    assert.equal(w.columnCount, 24);
    assert.equal(w.stats.pairingBuilds, 1);
    checkGram(w, data.system);
});

test('empty rows/columns, a sleeping rod and topology growth/shrink do not leave stale entries', () => {
    const w = createKirchhoffBundleRuntime();
    for (const n of [4, 19, 3]) {
        const data = fixture(Array.from({ length: n }, (_, i) => i), [0, 1, 2]);
        if (n === 3) data.system.material[1] = null;
        assembleKirchhoffBundleColumns(w, data.system, data.options);
        checkGram(w, data.system);
        if (n === 3) assert.equal(w.pairCount, 0);
    }
    const empty = fixture([], [], 0);
    assembleKirchhoffBundleColumns(w, empty.system, empty.options);
    assert.equal(w.columnCount, 0); assert.equal(w.entryCount, 0); assert.equal(w.rowCount, 0);
    addKirchhoffBundleGram(w, new Float64Array(0));
});

test('invalid weights, nonmonotone arcs, unsorted rows, stale failed assembly and buffer aliasing fail visibly', () => {
    const data = fixture();
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), data.system, data.options);
    assert.throws(() => encodeKirchhoffBundleDofs(w, w.generalized, w.generalized, w.generalized), /alias/);
    assert.throws(() => decodeKirchhoffBundleDofs(w, w.generalized, w.generalized, w.generalized), /alias/);
    assert.throws(() => recoverKirchhoffBundleCorrection(w, w.generalized, new Float64Array(24), new Float64Array(24)), /alias/);
    data.system.material[0].weight[0] = -1;
    assert.throws(() => assembleKirchhoffBundleColumns(w, data.system, data.options), /nonnegative/);
    assert.throws(() => addKirchhoffBundleGram(w, new Float64Array(100)), /successful/);
    data.system.material[0].weight[0] = 1;
    data.options.axialCoordinates[0][2] = -1;
    assert.throws(() => assembleKirchhoffBundleColumns(w, data.system, data.options), /increasing/);
    data.options.axialCoordinates[0][2] = 2;
    data.system.columns[0][0] = [3, 1, 1, 2];
    assert.throws(() => assembleKirchhoffBundleColumns(w, data.system, data.options), /sorted/);
});

// The frozen baseline predates the parent module. Set this path for the first
// validation; after integration the same test automatically uses the local file.
const parentRoot = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? fileURLToPath(new URL('../', import.meta.url)));
const coupledPath = join(parentRoot, 'src/physics/kirchhoffCoupledSystem.js');
test('current parent assembly: curvature/frame increments, unequal grids, finite clearance and all contact rows agree', {
    skip: !existsSync(coupledPath) && 'Coupled module absent from frozen baseline; set OET_BUNDLE_PARENT_PATH'
}, async () => {
    const { assembleKirchhoffCoupledSystem } = await import(pathToFileURL(coupledPath));
    const { EndovascularPhysicsWorld } = await import(pathToFileURL(join(parentRoot, 'src/physics/endovascularPhysicsWorld.js')));
    const { applyKirchhoffMaterialProfile } = await import(pathToFileURL(join(parentRoot, 'src/physics/applyKirchhoffMaterialProfile.js')));
    const { multiplyQuaternions, quaternionExp } = await import(pathToFileURL(join(parentRoot, 'src/physics/discreteKirchhoffRod.js')));
    const world = new EndovascularPhysicsWorld();
    const bodies = [world.createRod('bundle-i', 8, 3, { mass: 0.02 }), world.createRod('bundle-o', 6, 4.5, { mass: 0.09 })];
    bodies.forEach((body, side) => {
        applyKirchhoffMaterialProfile(body, 'berenstein');
        body.setPinned(0, true);
        for (let node = 0; node < body.count; node++) body.y[node] = side ? 0 : 0.02;
    });
    const records = [0.2, 0.65, 0.9].map((t, index) => ({
        _innerSegmentIndex: index + 2, _outerSegmentIndex: index + 1,
        innerWeights: [1 - t, t], outerWeights: [0.6, 0.4],
        normal: [0, 1, 0], gap: index === 1 ? -0.0004 : 0.0205,
        _normalAlpha: 0.001, manifoldContact: { normalLambda: 0.0001 }
    }));
    const constraint = { innerBody: bodies[0], outerBody: bodies[1], kirchhoffContacts: records };
    const extra = [{ strain: 0.02, alpha: 0.001, lambda: 0,
        gradients: [{ side: 0, dof: 17, value: -1 }, { side: 1, dof: 17, value: 1 }] }];
    const w = createKirchhoffBundleRuntime();
    const inputPositions = bodies.map(body => [body.x.slice(), body.y.slice(), body.z.slice()]);
    for (let iteration = 0; iteration < 3; iteration++) {
        if (iteration > 0) {
            for (const [side, body] of bodies.entries()) {
                body.y[3] += 0.003 * (side ? -1 : 1);
                body.restRotation1[2] += 0.012;
                const node = 3;
                const q = multiplyQuaternions({ x: body.orientationX[node], y: body.orientationY[node], z: body.orientationZ[node], w: body.orientationW[node] },
                    quaternionExp({ x: 0.004, y: -0.008, z: side ? 0.015 : -0.02 }));
                body.orientationX[node] = q.x; body.orientationY[node] = q.y; body.orientationZ[node] = q.z; body.orientationW[node] = q.w;
            }
        }
        const system = assembleKirchhoffCoupledSystem(constraint, 1 / 120, { additionalRows: extra });
        assembleKirchhoffBundleColumns(w, system, { axialOffsets: [0, 0.7], pairingRevision: 1 });
        const matrix = new Float64Array(system.count * w.band);
        for (let i = 0; i < system.count; i++) matrix[i * w.band] = system.rows[system.order[i]].alpha;
        addKirchhoffBundleGram(w, matrix);
        for (let i = 0; i < system.count; i++) for (let j = 0; j <= i; j++) {
            const expected = i - j < system.band ? system.matrix[i * system.band + i - j] : 0;
            const actual = i - j < w.band ? matrix[i * w.band + i - j] : 0;
            const scale = Math.sqrt(system.matrix[i * system.band]) * Math.sqrt(system.matrix[j * system.band]);
            assert.ok(Math.abs(actual - expected) <= 3e-13 * scale || actual === expected, `parent Gram ${i},${j}`);
        }
        const lambda = Float64Array.from({ length: system.count }, (_, i) => 1e-5 * Math.sin(i));
        const outputs = bodies.map(body => new Float64Array(body.count * 6));
        recoverKirchhoffBundleCorrection(w, lambda, ...outputs);
        closeArray([...outputs[0], ...outputs[1]], [...originalCorrection(system, lambda)], 1e-11);
        assert.equal(w.rowCount, system.count);
        assert.equal(system.additionalOffset - system.contactOffset, records.length);
    }
    assert.equal(w.stats.pairingBuilds, 1, 'curvature changes refresh J, not topology pairing');
    assert.equal(records[0].gap, 0.0205);
    assert.equal(records[1].gap, -0.0004);
    close(bodies[0].y[1], inputPositions[0][1][1]);
    close(bodies[1].y[1], inputPositions[1][1][1]);
});


test('reduced common-axis response cannot pass only by satisfying its projected constraint', () => {
    const f = fixture([0], [0], 1);
    f.system.material.forEach(m => m.weight.fill(1));
    f.system.columns.forEach(body => body.forEach(column => { column.length = 0; }));
    f.system.columns[0][0].push(0, 1);
    const w = assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(), f.system, f.options);
    const lambda = Float64Array.of(1), inner = new Float64Array(6), outer = new Float64Array(6);
    inner[0] = outer[0] = .5;
    // Projected Gram=.5 and rhs=.5: J*dq matches rhs perfectly, but imposing
    // a shared displacement has suppressed the relative mechanical response.
    assert.equal(.5 - inner[0], 0);
    const options = {translationTolerance:1e-12, rotationTolerance:1e-12};
    const reduced = measureKirchhoffBundleMobility(w, lambda, inner, outer, options);
    assert.equal(reduced.passed, false);
    assert.equal(reduced.maximumTranslationError, .5);
    assert.deepEqual(reduced.violations.map(v=>[v.side,v.node,v.axis]), [[0,0,0],[1,0,0]]);
    recoverKirchhoffBundleCorrection(w, lambda, inner, outer);
    assert.equal(measureKirchhoffBundleMobility(w, lambda, inner, outer, options).passed, true);
    inner[3] = NaN;
    assert.equal(measureKirchhoffBundleMobility(w, lambda, inner, outer, options).passed, false);
    assert.throws(()=>measureKirchhoffBundleMobility(w,lambda,inner,outer), /Tolerance/);
});

test('original mobility certificate retains independent rotational response', () => {
    const f=fixture(), w=assembleKirchhoffBundleColumns(createKirchhoffBundleRuntime(),f.system,f.options);
    const lambda=Float64Array.from({length:f.system.count},(_,i)=>Math.sin(i));
    const inner=new Float64Array(24),outer=new Float64Array(24);
    recoverKirchhoffBundleCorrection(w,lambda,inner,outer);
    const options={translationTolerance:1e-12,rotationTolerance:1e-12};
    assert.equal(measureKirchhoffBundleMobility(w,lambda,inner,outer,options).passed,true);
    outer[5]+=.01;
    const r=measureKirchhoffBundleMobility(w,lambda,inner,outer,options);
    assert.equal(r.passed,false);
    assert.deepEqual(r.violations.map(v=>[v.side,v.node,v.axis]),[[1,0,5]]);
});
