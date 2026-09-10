import { decodeKirchhoffAxialLayout, pullbackKirchhoffAxialForces, applyKirchhoffAxialMobility } from '../src/physics/kirchhoffAxialLayout.js';
import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { assembleKirchhoffDirect, solveKirchhoffDirect } from '../src/physics/kirchhoffDirectSolver.js';
import { assembleKirchhoffCoupledSystem, solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { solveCoupledBandQP } from '../src/physics/kirchhoffCoupledLinearSolver.js';
import { quaternionExp, multiplyQuaternions } from '../src/physics/discreteKirchhoffRod.js';

const dt = 1 / 120;
function rod(world, id, count = 8, mass = 0.03) {
    const body = world.createRod(id, count, 5, { mass, sleepFrames: 1e6 });
    applyKirchhoffMaterialProfile(body, 'berenstein');
    body.restRotation1.fill(0); body.restRotation2.fill(0);
    return body;
}
function clamp(body) {
    body.setPinned(body.activeStart, true);
    const s = body.activeStart;
    body.setProximalOrientationControl(body.orientationX[s], body.orientationY[s], body.orientationZ[s], body.orientationW[s], 0, s);
}
function record(node, gap, normal = [0, 1, 0], t = 0.4, lambda = 0, alpha = 0) {
    return { _innerSegmentIndex: node, _outerSegmentIndex: node,
        innerWeights: [1 - t, t], outerWeights: [1 - t, t], normal, gap, _normalAlpha: alpha,
        manifoldContact: { normalLambda: lambda } };
}
function denseSolve(matrix, rhs) {
    const n = rhs.length, a = matrix.map((r, i) => [...r, rhs[i]]);
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) if (Math.abs(a[i][k]) > Math.abs(a[pivot][k])) pivot = i;
        if (Math.abs(a[pivot][k]) < 1e-12) return null;
        [a[k], a[pivot]] = [a[pivot], a[k]];
        for (let i = k + 1; i < n; i++) {
            const f = a[i][k] / a[k][k];
            for (let j = k + 1; j <= n; j++) a[i][j] -= f * a[k][j];
        }
    }
    const x = new Float64Array(n);
    for (let i = n - 1; i >= 0; i--) {
        let b = a[i][n];
        for (let j = i + 1; j < n; j++) b -= a[i][j] * x[j];
        x[i] = b / a[i][i];
    }
    return x;
}
// Assemble an independent dense J by body/material equation order. This does
// not use the coupled matrix, its permutation, band width or column merging.
function denseOracle(constraint) {
    const bodies = [constraint.innerBody, constraint.outerBody], ss = bodies.map(b => assembleKirchhoffDirect(b, dt));
    const nm = ss[0].rowCount + ss[1].rowCount, nr = constraint.kirchhoffContacts.length, n = nm + nr;
    const nd = bodies.reduce((n, b) => n + b.count * 6, 0);
    const J = Array.from({ length: n }, () => new Float64Array(nd)), alpha = new Float64Array(n), rhs = new Float64Array(n);
    const W = new Float64Array(nd);
    let ro = 0, co = 0;
    for (let side = 0; side < 2; side++) {
        const s = ss[side];
        for (let r = 0; r < s.rowCount; r++) { alpha[ro + r] = s.alpha[r]; rhs[ro + r] = s.rhs[r]; }
        for (let dof = s.start * 6; dof < (s.end + 1) * 6; dof++) {
            W[co + dof] = s.weight[dof];
            for (let k = 0; k < s.degree[dof]; k++) J[ro + s.rows[dof * 9 + k]][co + dof] = s.gradients[dof * 9 + k];
        }
        ro += s.rowCount; co += bodies[side].count * 6;
    }
    constraint.kirchhoffContacts.forEach((r, i) => {
        alpha[nm + i] = r._normalAlpha; rhs[nm + i] = -r.gap - r._normalAlpha * r.manifoldContact.normalLambda;
        for (let side = 0; side < 2; side++) for (let k = 0; k < 2; k++) for (let axis = 0; axis < 3; axis++) {
            const node = (side ? r._outerSegmentIndex : r._innerSegmentIndex) + k;
            const dof = (side ? bodies[0].count * 6 : 0) + node * 6 + axis;
            J[nm + i][dof] += (side ? 1 : -1) * (side ? r.outerWeights[k] : r.innerWeights[k]) * r.normal[axis];
        }
    });
    const A = J.map((row, i) => J.map((other, j) => row.reduce((sum, v, k) => sum + v * W[k] * other[k], i === j ? alpha[i] : 0)));
    for (let mask = 0; mask < 1 << nr; mask++) {
        const free = Array.from({ length: n }, (_, i) => i).filter(i => i < nm || mask & 1 << (i - nm));
        const x = new Float64Array(n);
        for (let i = 0; i < nr; i++) x[nm + i] = -constraint.kirchhoffContacts[i].manifoldContact.normalLambda;
        const solution = denseSolve(free.map(i => free.map(j => A[i][j])), free.map(i => rhs[i] - A[i].reduce((s, a, j) => s + (j >= nm && !(mask & 1 << (j - nm)) ? a * x[j] : 0), 0)));
        if (!solution) continue;
        free.forEach((i, k) => { x[i] = solution[k]; });
        if (constraint.kirchhoffContacts.some((r, i) => x[nm + i] + r.manifoldContact.normalLambda < -1e-9)) continue;
        const residual = rhs.map((b, i) => b - A[i].reduce((s, a, j) => s + a * x[j], 0));
        if (residual.some((r, i) => i < nm || mask & 1 << (i - nm) ? Math.abs(r) > 1e-7 : r > 1e-7)) continue;
        const correction = W.map((w, dof) => w * J.reduce((s, row, i) => s + row[dof] * x[i], 0));
        return { x, correction, ss, nm, J, W, A, rhs };
    }
    throw new Error('Dense exhaustive active-set oracle found no solution');
}

for (const warm of [false, true]) test(`full simultaneous direction agrees with independent dense exhaustive active set (warm=${warm})`, () => {
    const world = new EndovascularPhysicsWorld();
    const innerBody = rod(world, 'inner', 8, 0.02), outerBody = rod(world, 'outer', 8, 0.07);
    for (const [side, b] of [innerBody, outerBody].entries()) {
        clamp(b);
        b.y[4] = 0.002 * (side ? -1 : 1); b.z[5] = 0.001;
        b.restRotation1[3] = 0.015; b.restRotation2[4] = -0.01; b.restRotation3[5] = 0.02;
        b.bendTwistLambda1[3] = 2e-6; b.adaptationLambdaY[4] = -1e-6;
    }
    const records = [record(2, -0.001, [0, 1, 0], 0.3), record(3, 0.002, [0, 1, 0], 0.8, warm ? 0.001 : 0),
        record(4, -0.003, [0, 0, 1]), record(5, -0.001, [0, -1, 0]), record(3, 0.03, [0, 0, -1])];
    for (const r of records) r._normalAlpha = 0.01;
    const constraint = { innerBody, outerBody, kirchhoffContacts: records };
    const expected = denseOracle(constraint), old = [innerBody.y.slice(), outerBody.y.slice()], normal = records.map(r => r.manifoldContact.normalLambda);
    const actual = solveKirchhoffCoupledSystem(constraint, dt, { tolerance: 1e-9 });
    assert.equal(actual.diagnostics.converged, true, JSON.stringify(actual.diagnostics));
    const lambda = [...actual.inner.lambda, ...actual.outer.lambda, ...actual.contactIncrement];
    lambda.forEach((v, i) => assert.ok(Math.abs(v - expected.x[i]) < 1e-7, `lambda ${i}: ${v} vs ${expected.x[i]}`));
    const correction = [...actual.inner.correction, ...actual.outer.correction];
    correction.forEach((v, i) => assert.ok(Math.abs(v - expected.correction[i]) < 1e-8, `dof ${i}: ${v} vs ${expected.correction[i]}`));
    assert.deepEqual(innerBody.y, old[0]); assert.deepEqual(outerBody.y, old[1]);
    applyKirchhoffCoupledCorrection(constraint, actual, 0.5);
    assert.deepEqual(records.map(r => r.manifoldContact.normalLambda), normal, 'caller owns contact accumulation');
    assert.equal(innerBody.y[4], Math.fround(old[0][4] + 0.5 * actual.inner.correction[24 + 1]));
});

test('assembly leaves solo material result and complete active ranges unchanged', () => {
    const world = new EndovascularPhysicsWorld(), a = rod(world, 'a', 13), b = rod(world, 'b', 13);
    for (const body of [a, b]) {
        body.setActiveRange(2, 11); clamp(body); body.y[7] = 0.04; body.restRotation2[8] = 0.02;
        body.restRotation3[6] = -0.03;
    }
    assembleKirchhoffDirect(a, dt);
    solveKirchhoffDirect(a, dt); solveKirchhoffDirect(b, dt);
    for (const key of ['x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW', 'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3', 'adaptationLambdaX', 'adaptationLambdaY', 'adaptationLambdaZ']) assert.deepEqual(a[key], b[key], key);
});

test('a normal transfers equal and opposite reactions and preserves free slide/twist', () => {
    const world = new EndovascularPhysicsWorld(), innerBody = rod(world, 'i'), outerBody = rod(world, 'o');
    // Translate along the tangent and independently rotate each entire frame
    // field about d3. Neither enters the radial contact equation.
    for (const [side, body] of [innerBody, outerBody].entries()) for (let i = 0; i < body.count; i++) {
        body.x[i] += side ? -1.2 : 2.7;
        if (i === body.segmentCount) continue;
        const q = multiplyQuaternions({ x: body.orientationX[i], y: body.orientationY[i], z: body.orientationZ[i], w: body.orientationW[i] }, quaternionExp({ x: 0, y: 0, z: side ? -0.4 : 0.7 }));
        body.orientationX[i] = q.x; body.orientationY[i] = q.y; body.orientationZ[i] = q.z; body.orientationW[i] = q.w;
    }
    const c = { innerBody, outerBody, kirchhoffContacts: [record(4, -0.001, [0, 1, 0], 0)] };
    const r = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-9 });
    assert.ok(r.diagnostics.converged);
    assert.ok(r.inner.correction[25] < 0 && r.outer.correction[25] > 0);
    let sum = 0;
    for (const bodyResult of [r.inner, r.outer]) for (let node = 0; node < innerBody.count; node++) sum += bodyResult.correction[node * 6 + 1] / innerBody.inverseMass[node];
    assert.ok(Math.abs(sum) < 1e-12, `net transverse impulse ${sum}`);
    // Every node retains its unconstrained axial translation; local spin about
    // d3 is free even though bending corrections act on the other two axes.
    for (const result of [r.inner, r.outer]) for (let node = 0; node < innerBody.count; node++) {
        assert.ok(Math.abs(result.correction[node * 6]) < 2e-6);
        assert.ok(Math.abs(result.correction[node * 6 + 5]) < 1e-9);
    }
    c.kirchhoffContacts[0].gap = 0.01;
    c.kirchhoffContacts[0].manifoldContact.normalLambda = r.contactIncrement[0];
    const released = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-9 });
    assert.ok(released.diagnostics.converged);
    assert.ok(Math.abs(released.contactIncrement[0] + r.contactIncrement[0]) < 1e-12);
});

test('redundant inequalities retain the deepest contact and diagnose impossible rows', () => {
    // All rows describe the same scalar displacement with different bounds.
    const matrix = new Float64Array([1,0,0, 1,1,0, 1,1,1]);
    const solved = solveCoupledBandQP(matrix, new Float64Array([1,2,0.5]), new Float64Array(3), new Float64Array(3).fill(Infinity), 3, 3, { tolerance: 1e-9 });
    assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
    assert.ok(solved.diagnostics.nearNullPivots > 0);
    assert.ok(Math.abs(solved.increment[1] - 2) < 1e-9);
    assert.equal(solved.increment[0], 0); assert.equal(solved.increment[2], 0);
    const impossible = solveCoupledBandQP(new Float64Array([0]), new Float64Array([1]), new Float64Array([0]), new Float64Array([Infinity]), 1, 1, { maxActiveSetIterations: 5 });
    assert.equal(impossible.diagnostics.converged, false);
    assert.equal(impossible.diagnostics.maximumResidual, 1);
});

test('additional sticking tangent/twist rows use local angular dofs and total multiplier bounds', () => {
    const world = new EndovascularPhysicsWorld(), innerBody = rod(world, 'i'), outerBody = rod(world, 'o');
    const constraint = { innerBody, outerBody, kirchhoffContacts: [] };
    const additionalRows = [{ strain: -0.01, alpha: 0.001, lambda: 0.001, lower: -0.02, upper: 0.02,
        gradients: [{ side: 0, dof: 3 * 6 + 5, value: -1 }, { side: 1, dof: 3 * 6 + 5, value: 1 }] }];
    const r = solveKirchhoffCoupledSystem(constraint, dt, { additionalRows, tolerance: 1e-9 });
    assert.ok(r.diagnostics.converged);
    assert.ok(r.inner.correction[23] < 0 && r.outer.correction[23] > 0);
    const residual = -0.01 + r.outer.correction[23] - r.inner.correction[23] + 0.001 * (0.001 + r.additionalIncrement[0]);
    assert.ok(Math.abs(residual) < 1e-9);
    assert.throws(() => solveKirchhoffCoupledSystem(constraint, dt, { groups: [{ type: 'coulomb-disk' }] }), /two distinct additional-row indices/);
});

export function largeFixture() {
    const world = new EndovascularPhysicsWorld(), innerBody = rod(world, 'i', 201), outerBody = rod(world, 'o', 197, 0.07);
    clamp(innerBody); clamp(outerBody);
    const records = Array.from({ length: 479 }, (_, i) => {
        const u = 2 + i / 478 * 192, s = Math.floor(u), t = u - s;
        return record(s, -0.001 * (1 + Math.sin(u * 0.1)), [0, 1, 0], t);
    });
    return { innerBody, outerBody, kirchhoffContacts: records };
}

test('479 hard contact records and 201/197 nodes solve without dropping rod dofs', () => {
    const c = largeFixture();
    const r = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-7 });
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.diagnostics.dofCount, 2382);
    assert.ok(r.diagnostics.band < 40);
    assert.ok(r.diagnostics.materialResidual < 1e-7 && r.diagnostics.contactResidual < 1e-7);
    assert.ok(r.diagnostics.nearNullPivots > 0);
    applyKirchhoffCoupledCorrection(c, r);
    for (const body of [c.innerBody, c.outerBody]) {
        let total = 0;
        for (let i = body.activeStart; i < body.activeEnd; i++) {
            const length = Math.hypot(body.x[i+1]-body.x[i], body.y[i+1]-body.y[i], body.z[i+1]-body.z[i]);
            assert.ok(Math.abs(length - body.restLength[i]) < 1e-6, `length segment ${i}`);
            total += length;
        }
        assert.ok(Math.abs(total - body.segmentCount * 5) < 1e-5, `whole rod length ${total}`);
    }
});

test('contact identity hints survive multiplier resets, record reordering and release', () => {
    const c = largeFixture();
    const first = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-7 });
    assert.ok(first.diagnostics.converged);
    const before = c.kirchhoffContacts.map(r => r.manifoldContact._jointActive);
    assert.ok(before.some(Boolean) && before.some(v => !v));
    // The world resets all force multipliers each fixed step. No result was
    // applied here, so this precisely repeats the unsolved equations with
    // only a boolean hint, in the opposite detection order.
    c.kirchhoffContacts.reverse();
    const warm = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-7 });
    assert.ok(warm.diagnostics.converged);
    assert.ok(warm.diagnostics.factorizations <= 2, JSON.stringify(warm.diagnostics));
    for (let i = 0; i < first.contactIncrement.length; i++) assert.ok(Math.abs(first.contactIncrement[i] - warm.contactIncrement[first.contactIncrement.length - 1 - i]) < 1e-8);
    for (const r of c.kirchhoffContacts) r.gap = 0.02;
    const release = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-7 });
    assert.ok(release.diagnostics.converged);
    assert.equal(release.diagnostics.activeContacts, 0);
    assert.ok(c.kirchhoffContacts.every(r => !r.manifoldContact._jointActive));
});

test('nonlinear simultaneous iterations restore full rod lengths and material/contact equations', () => {
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i', 21), outerBody = rod(w, 'o', 19);
    for (const [side, body] of [innerBody, outerBody].entries()) {
        clamp(body);
        for (let i = 2; i < body.count; i++) body.y[i] = (side ? -0.1 : 0.2) * Math.sin((i - 1) / (body.count - 2) * Math.PI);
        body.restRotation2[body.segmentCount - 2] = side ? 0.005 : -0.01;
    }
    const records = Array.from({ length: 16 }, (_, i) => record(i + 2, 0, [0, 1, 0], 0));
    const c = { innerBody, outerBody, kirchhoffContacts: records };
    let result;
    for (let pass = 0; pass < 20; pass++) {
        for (const r of records) {
            const node = r._innerSegmentIndex, difference = innerBody.y[node] - outerBody.y[node];
            const sign = Math.sign(difference) || 1;
            if (sign !== r.normal[1]) r.manifoldContact.normalLambda = 0;
            r.normal[1] = sign; r.gap = 0.0405 - Math.abs(difference);
        }
        result = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-8 });
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        applyKirchhoffCoupledCorrection(c, result);
        records.forEach((r, i) => { r.manifoldContact.normalLambda += result.scale * result.contactIncrement[i]; });
    }
    for (const body of [innerBody, outerBody]) {
        let sum = 0;
        for (let i = 0; i < body.segmentCount; i++) {
            const length = Math.hypot(body.x[i+1]-body.x[i], body.y[i+1]-body.y[i], body.z[i+1]-body.z[i]);
            assert.ok(Math.abs(length - body.restLength[i]) < 1e-5);
            sum += length;
        }
        assert.ok(Math.abs(sum - body.segmentCount * 5) < 1e-5);
        const assembly = assembleKirchhoffDirect(body, dt);
        const materialError = Math.max(...assembly.rhs.subarray(0, assembly.rowCount).map(Math.abs));
        // World positions are Float32: at 100 mm their ULP is about 8e-6 mm.
        assert.ok(materialError < 1e-5, `material nonlinear residual ${materialError}`);
    }
    assert.ok(records.every(r => Math.abs(innerBody.y[r._innerSegmentIndex] - outerBody.y[r._outerSegmentIndex]) <= 0.040501));
});

test('zero-width multiplier bounds and failed application are explicit', () => {
    const fixed = solveCoupledBandQP(new Float64Array([1]), new Float64Array([2]), new Float64Array([0]), new Float64Array([0]), 1, 1);
    assert.ok(fixed.diagnostics.converged); assert.equal(fixed.increment[0], 0);
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i'), outerBody = rod(w, 'o');
    clamp(innerBody); clamp(outerBody);
    const c = { innerBody, outerBody, kirchhoffContacts: [record(1, -0.01, [0, 1, 0], 0)] };
    const r = solveKirchhoffCoupledSystem(c, dt, { maxActiveSetIterations: 8 });
    assert.equal(r.diagnostics.converged, false);
    assert.throws(() => applyKirchhoffCoupledCorrection(c, r), /failed.*KKT/);
});

test('dry assembly never clears material multipliers at redundant pinned boundaries', () => {
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i'), outerBody = rod(w, 'o');
    for (const body of [innerBody, outerBody]) {
        body.setPinned(0, true); body.setPinned(1, true);
        body.adaptationLambdaX[0] = 123;
    }
    const c = { innerBody, outerBody, kirchhoffContacts: [] };
    const r = solveKirchhoffCoupledSystem(c, dt);
    assert.ok(r.diagnostics.converged);
    assert.equal(innerBody.adaptationLambdaX[0], 123); assert.equal(outerBody.adaptationLambdaX[0], 123);
    applyKirchhoffCoupledCorrection(c, r);
    assert.equal(innerBody.adaptationLambdaX[0], 0); assert.equal(outerBody.adaptationLambdaX[0], 0);
});

test('active-set bound handling preserves forces below 1e-14', () => {
    for (const upper of [Infinity, 1e-16]) {
        const solved = solveCoupledBandQP(new Float64Array([1e10]), new Float64Array([1e-5]),
            new Float64Array([0]), new Float64Array([upper]), 1, 1, { tolerance: 1e-12 });
        assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
        assert.ok(solved.increment[0] > 0);
        assert.ok(Math.abs(solved.increment[0] - Math.min(1e-15, upper)) < 1e-26);
    }
});

for (const mu of [[0.2, 0.2], [0.1, 0.3], [0, 0.2], [0, 0]]) test(`joint rods retain material/normal equations with fixed-load surface friction ${mu}`, () => {
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i'), outerBody = rod(w, 'o');
    const c = { innerBody, outerBody, kirchhoffContacts: [record(4, -0.001, [0, 1, 0], 0)] };
    const normalOnly = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-9 });
    const fn = normalOnly.contactIncrement[0];
    const additionalRows = [
        { strain: 0.01, alpha: 0, lambda: 0, gradients: [{ side: 0, dof: 24, value: -1 }, { side: 1, dof: 24, value: 1 }] },
        { strain: -0.015, alpha: 0, lambda: 0, gradients: [{ side: 0, dof: 26, value: -1 }, { side: 1, dof: 26, value: 1 },
            { side: 0, dof: 29, value: -0.45 }, { side: 1, dof: 29, value: 0.49 }] },
    ];
    const options = { tolerance: 1e-9, additionalRows, groups: [{ kind: mu[0] === mu[1] ? 'coulomb-disk' : 'coulomb-ellipse', rowIndices: [0, 1], mu, normalLambda: fn }] };
    const result = solveKirchhoffCoupledSystem(c, dt, options);
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    assert.ok(result.diagnostics.materialResidual < 1e-9);
    assert.ok(result.diagnostics.contactResidual < 1e-9);
    const normalized = result.additionalIncrement.map((v, i) => mu[i] ? v / (mu[i] * fn) : 0);
    assert.ok(Math.hypot(...normalized) <= 1 + 1e-8);
    for (let i = 0; i < 2; i++) if (!mu[i]) assert.ok(result.additionalIncrement[i] === 0);
    // Straight rod radial and surface blocks are orthogonal. A friction cone
    // that increases Fn to buy friction would violate this independent check.
    assert.ok(Math.abs(result.contactIncrement[0] - fn) < 1e-9);
    let work = 0;
    additionalRows.forEach((row, i) => {
        const displacement = row.strain + row.gradients.reduce((sum, g) => sum + g.value * [result.inner, result.outer][g.side].correction[g.dof], 0);
        work += result.additionalIncrement[i] * displacement;
    });
    assert.ok(work <= 1e-12, `friction work ${work}`);
    assert.deepEqual(additionalRows.map(row => row.lambda), [0, 0]);
    assert.equal(c.kirchhoffContacts[0].manifoldContact.normalLambda, 0);
});

test('custom normal gradients override the default stencil and sum duplicate force/moment dofs', () => {
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i'), outerBody = rod(w, 'o');
    const normalGradients = [{ side: 0, dof: 26, value: -0.3 }, { side: 0, dof: 26, value: -0.7 },
        { side: 1, dof: 26, value: 1 }, { side: 1, dof: 29, value: 0.5 }];
    const contact = { ...record(4, -0.001, [0, 1, 0], 0), normalGradients };
    const c = { innerBody, outerBody, kirchhoffContacts: [contact] };
    const actual = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-9 });
    assert.ok(actual.diagnostics.converged, JSON.stringify(actual.diagnostics));
    assert.ok(actual.inner.correction[26] < 0 && actual.outer.correction[26] > 0 && actual.outer.correction[29] > 0);
    assert.ok(Math.abs(actual.inner.correction[25]) < 1e-10, 'default y normal must not be added a second time');
    c.kirchhoffContacts = [];
    const equivalent = solveKirchhoffCoupledSystem(c, dt, { tolerance: 1e-9, additionalRows: [{ strain: -0.001, alpha: 0, lambda: 0, lower: 0,
        gradients: [{ side: 0, dof: 26, value: -1 }, { side: 1, dof: 26, value: 1 }, { side: 1, dof: 29, value: 0.5 }] }] });
    assert.ok(equivalent.diagnostics.converged);
    assert.ok(Math.abs(actual.contactIncrement[0] - equivalent.additionalIncrement[0]) < 1e-10);
    for (const side of ['inner', 'outer']) actual[side].correction.forEach((v, i) => assert.ok(Math.abs(v - equivalent[side].correction[i]) < 1e-10));
});

test('typed Uint32 friction row indices retain fractional previous multipliers and bound total forces', () => {
    const w = new EndovascularPhysicsWorld(), innerBody = rod(w, 'i'), outerBody = rod(w, 'o');
    const c = { innerBody, outerBody, kirchhoffContacts: [] };
    const previous = [0.00006, -0.00003], radius = 0.0001;
    const additionalRows = [
        { strain: -0.02, alpha: 0, lambda: previous[0], gradients: [{ side: 0, dof: 24, value: -1 }, { side: 1, dof: 24, value: 1 }] },
        { strain: 0.01, alpha: 0, lambda: previous[1], gradients: [{ side: 0, dof: 26, value: -1 }, { side: 1, dof: 26, value: 1 }] },
    ];
    const group = { kind: 'coulomb-disk', rowIndices: new Uint32Array([0, 1]), mu: [radius, radius], normalLambda: 1 };
    const typed = solveKirchhoffCoupledSystem(c, dt, { additionalRows, groups: [group], tolerance: 1e-9, includeSystem: true });
    assert.ok(typed.diagnostics.converged, JSON.stringify(typed.diagnostics));
    assert.deepEqual(typed.system.groups[0].lambda, previous, 'integer row-index storage must not become integer force storage');
    const total = typed.additionalIncrement.map((delta, i) => delta + previous[i]);
    assert.ok(Math.hypot(...total) <= radius * (1 + 1e-9), `total friction ${Math.hypot(...total)} exceeds ${radius}`);
    group.rowIndices = [0, 1];
    const plain = solveKirchhoffCoupledSystem(c, dt, { additionalRows, groups: [group], tolerance: 1e-9 });
    for (let i = 0; i < 2; i++) assert.ok(Math.abs(plain.additionalIncrement[i] - typed.additionalIncrement[i]) < 1e-12);
    for (const side of ['inner', 'outer']) typed[side].correction.forEach((value, i) => assert.ok(Math.abs(value - plain[side].correction[i]) < 1e-12));
});

test('joint material/contact solve exposes an independent original-mobility certificate', () => {
    const world = new EndovascularPhysicsWorld();
    const inner = rod(world,'mobility-wire',8,.03), outer = rod(world,'mobility-catheter',8,.07);
    clamp(inner); clamp(outer);
    inner.y[6] += .01;
    const constraint = {innerBody:inner,outerBody:outer,kirchhoffContacts:[record(5,-.001)]};
    const result = solveKirchhoffCoupledSystem(constraint,dt,{tolerance:1e-7,
        mobilityAudit:{translationTolerance:1e-9,rotationTolerance:1e-9}});
    assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));
    assert.equal(result.diagnostics.mobility.passed,true);
    assert.deepEqual(result.diagnostics.mobility.violations,[]);
});


test('shared-axis layout binds original active material nodes without changing the existing equations', () => {
    const world=new EndovascularPhysicsWorld();
    const inner=rod(world,'layout-wire',8),outer=rod(world,'layout-catheter',6,.07);
    clamp(inner);clamp(outer);
    outer.y[2]+=.035;
    const c={innerBody:inner,outerBody:outer,kirchhoffContacts:[record(3,-.001)],
        startNode:1,outerStartNode:0,innerArcOffset:.75};
    const original=assembleKirchhoffCoupledSystem(c,dt);
    const matrix=original.matrix.slice(),rhs=original.rhs.slice();
    const bound=assembleKirchhoffCoupledSystem(c,dt,{includeAxialLayout:true});
    assert.deepEqual(bound.matrix,matrix); assert.deepEqual(bound.rhs,rhs);
    const layout=bound.axialLayout;
    assert.equal(layout.axialOffsets[1],4.25);
    const decoded=decodeKirchhoffAxialLayout(layout,layout.referenceCoordinates);
    for(const [body,xyz] of [[inner,decoded.wire],[outer,decoded.catheter]])
        for(let i=0;i<body.count;i++) for(let d=0;d<3;d++)
            assert.ok(Math.abs(xyz[3*i+d]-body[['x','y','z'][d]][i])<1e-12);
    assert.deepEqual(layout.wireNodes,Array.from({length:8},(_,i)=>i));
    c.innerArcOffset=1.25;
    const moved=assembleKirchhoffCoupledSystem(c,dt,{includeAxialLayout:true}).axialLayout;
    assert.equal(moved.axialOffsets[1],3.75);
    assert.deepEqual(moved.wireMaterialIds,layout.wireMaterialIds);
    assert.notDeepEqual(moved.sites[0].weights,layout.sites[0].weights);
});

test('axial layout refuses inferred spatial alignment and supports original active indices', () => {
    const world=new EndovascularPhysicsWorld();
    const inner=rod(world,'active-wire',8),outer=rod(world,'active-catheter',6,.07);
    inner.setActiveRange(2,7);outer.setActiveRange(1,5);
    const c={innerBody:inner,outerBody:outer,kirchhoffContacts:[]};
    assert.throws(()=>assembleKirchhoffCoupledSystem(c,dt,{includeAxialLayout:true}),/material containment window/);
    const axialCoordinates=[Float64Array.from({length:8},(_,i)=>i*5),Float64Array.from({length:6},(_,i)=>i*5)];
    const layout=assembleKirchhoffCoupledSystem(c,dt,{includeAxialLayout:true,axialCoordinates}).axialLayout;
    assert.deepEqual(layout.wireNodes,[2,3,4,5,6,7]);
    assert.equal(layout.sites[0].nodeIndex,1);
    assert.equal(layout.sites[0].wireSlots.length,0,'proximal catheter outside overlap remains independent');
    assert.equal(layout.sites[1].wireNodes[0],2);
});


test('interpolated axial metric reconstructs original native Jacobian impulses', () => {
    const world=new EndovascularPhysicsWorld();
    const inner=rod(world,'metric-wire',8,.03),outer=rod(world,'metric-catheter',6,.09);
    clamp(inner);clamp(outer);inner.y[3]+=.04;
    const c={innerBody:inner,outerBody:outer,kirchhoffContacts:[record(3,-.001)],
        startNode:1,outerStartNode:0,innerArcOffset:.75};
    const s=assembleKirchhoffCoupledSystem(c,dt,{includeAxialLayout:true});
    const lambda=Float64Array.from({length:s.count},(_,i)=>Math.sin(i)*.001);
    const forces=[],weights=[];
    for(let side=0;side<2;side++) {
        const n=s.bodies[side].count, f=new Float64Array(n*3),w=new Float64Array(n*3);
        for(let node=0;node<n;node++) for(let axis=0;axis<3;axis++) {
            const column=s.columns[side][node*6+axis];
            for(let k=0;k<column.length;k+=2) f[node*3+axis]+=column[k+1]*lambda[column[k]];
            w[node*3+axis]=s.material[side].weight[node*6+axis];
        }
        forces.push(f);weights.push(w);
    }
    const g=pullbackKirchhoffAxialForces(s.axialLayout,...forces);
    const q=applyKirchhoffAxialMobility(s.axialLayout,g,...weights);
    const original=decodeKirchhoffAxialLayout(s.axialLayout,q);
    for(const [side,values] of [[0,original.wire],[1,original.catheter]])
        for(let i=0;i<values.length;i++) assert.ok(Math.abs(values[i]-weights[side][i]*forces[side][i])<1e-10);
});

test('isolated component solves material and wall together without second-body degrees of freedom',()=>{
 const world=new EndovascularPhysicsWorld(),body=rod(world,'solo',9),other=rod(world,'other',9);
 for(const b of [body,other]){clamp(b);b.y[4]=.03;b.restRotation2[5]=.01;}
 const component={bodies:[body],kirchhoffContacts:[]};
 const wall={strain:-.01,alpha:0,lambda:0,lower:0,gradients:[{side:0,dof:4*6+1,value:1}],coordinate:20};
 const options={additionalRows:[wall],basis:'individual',tolerance:1e-8,includeSystem:true};
 const solo=solveKirchhoffCoupledSystem(component,dt,options);
 assert.ok(solo.diagnostics.converged,JSON.stringify(solo.diagnostics));
 assert.equal(solo.responses.length,1);assert.equal(solo.outer,undefined);
 assert.equal(solo.system.columns.length,1);assert.equal(solo.system.material.length,1);
 assert.equal(solo.system.count,solo.system.material[0].rowCount+1);
 const pair=solveKirchhoffCoupledSystem({innerBody:body,outerBody:other,kirchhoffContacts:[]},dt,options);
 assert.ok(pair.diagnostics.converged,JSON.stringify(pair.diagnostics));
 solo.inner.correction.forEach((v,i)=>assert.ok(Math.abs(v-pair.inner.correction[i])<1e-8,`dof ${i}`));
 assert.ok(Math.abs(solo.additionalIncrement[0]-pair.additionalIncrement[0])<1e-8);
 const y=body.y[4];applyKirchhoffCoupledCorrection(component,solo,.5);
 assert.equal(body.y[4],Math.fround(y+.5*solo.inner.correction[25]));
 assert.throws(()=>applyKirchhoffCoupledCorrection({bodies:[other]},solo),/bodies changed/);
 assert.throws(()=>assembleKirchhoffCoupledSystem({bodies:[body],kirchhoffContacts:[record(2,-.01)]}),/two bodies/);
});

test('isolated material residual equals its contribution to an uncoupled pair',async()=>{
 const {measureKirchhoffCoupledMaterialResidual:measure}=await import('../src/physics/kirchhoffCoupledResidual.js');
 const world=new EndovascularPhysicsWorld(),body=rod(world,'solo-residual',8),other=rod(world,'other-residual',8);
 body.y[4]=.1;body.restRotation2[5]=.02;
 const solo=measure({bodies:[body]},dt),pair=measure({innerBody:body,outerBody:other},dt);
 assert.equal(solo.adaptationMm,pair.adaptationMm);assert.equal(solo.bendTwistRad,pair.bendTwistRad);
 assert.equal(solo.worstAdaptationSide,0);assert.equal(solo.worstAdaptationSegment,pair.worstAdaptationSegment);
});
