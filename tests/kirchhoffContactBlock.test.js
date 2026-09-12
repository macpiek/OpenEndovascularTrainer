import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { solveKirchhoffDirect, kirchhoffDirectContactResponse, kirchhoffDirectBlockResponse } from '../src/physics/kirchhoffDirectSolver.js';
import { createKirchhoffLinearKernel } from '../src/physics/kirchhoffLinearKernel.js';
import { solveKirchhoffContactBlock } from '../src/physics/kirchhoffContactBlock.js';

function rod(world, id, mass) {
    const body = world.createRod(id, 13, 4, { mass, sleepFrames: 1e6 });
    applyKirchhoffMaterialProfile(body, 'berenstein');
    body.restRotation1.fill(0); body.restRotation2.fill(0);
    body.setPinned(0, true);
    body.setProximalOrientationControl(body.orientationX[0], body.orientationY[0], body.orientationZ[0], body.orientationW[0], 0, 0);
    solveKirchhoffDirect(body, 1 / 120, true);
    return body;
}

test('one accumulated full-rod response agrees with independent sparse impulses', () => {
    const body = rod(new EndovascularPhysicsWorld(), 'beam', 0.03);
    const impulse = new Float64Array(body.count * 6);
    const expected = new Float64Array(impulse.length);
    const expectedLambda = new Float64Array(body.segmentCount * 6);
    for (const [node, normal] of [[2, [0.1, -0.3, 0.2]], [7, [0, 0.7, -0.1]], [11, [-0.2, 0.1, 0.5]]]) {
        for (let axis = 0; axis < 3; axis++) impulse[node * 6 + axis] += normal[axis];
        const response = kirchhoffDirectContactResponse(body, [node], [1], 1, normal, 1);
        for (let i = 0; i < expected.length; i++) expected[i] += response.correction[i];
        for (let i = 0; i < expectedLambda.length; i++) expectedLambda[i] += response.lambda[i];
    }
    const actual = kirchhoffDirectBlockResponse(body, impulse);
    for (let i = 0; i < expected.length; i++) assert.ok(Math.abs(expected[i] - actual.correction[i]) < 1e-8);
    for (let i = 0; i < expectedLambda.length; i++) assert.ok(Math.abs(expectedLambda[i] - actual.lambda[i]) < 1e-8);
});

function denseSolve(matrix, rhs) {
    const a = matrix.map((row, i) => [...row, rhs[i]]), n = a.length;
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) if (Math.abs(a[i][k]) > Math.abs(a[pivot][k])) pivot = i;
        [a[k], a[pivot]] = [a[pivot], a[k]];
        if (Math.abs(a[k][k]) < 1e-14) return null;
        const scale = a[k][k];
        for (let j = k; j <= n; j++) a[k][j] /= scale;
        for (let i = 0; i < n; i++) if (i !== k) {
            const value = a[i][k];
            for (let j = k; j <= n; j++) a[i][j] -= value * a[k][j];
        }
    }
    return a.map(row => row[n]);
}

for (const direct of [true, false]) for (const staleHint of [false, true]) test(`the complete unilateral contact block agrees with a dense active-set oracle (full rod: ${direct}, stale active set: ${staleHint})`, () => {
    const world = new EndovascularPhysicsWorld();
    const innerBody = rod(world, 'wire', 0.02), outerBody = rod(world, 'catheter', 0.07);
    const bodies = [innerBody, outerBody];
    const records = Array.from({ length: 6 }, (_, i) => ({
        kind: 'material-side', normal: [0, i === 4 ? -1 : 1, 0],
        _innerNodeIndices: [i + 2, i + 3], _innerNodeWeights: [0.4, 0.6], _innerNodeCount: 2,
        _outerNodeIndices: [i + 2, i + 3], _outerNodeWeights: [0.7, 0.3], _outerNodeCount: 2,
        _innerSegmentIndex: i + 2, _outerSegmentIndex: i + 2,
        _normalAlpha: 0.01, gap: [0.001, -0.002, -0.001, 0.002, -0.001, -0.002][i],
        manifoldContact: { normalLambda: i === 3 ? 0.0001 : 0, _blockActive: staleHint ? i % 2 === 0 : undefined }
    }));
    const n = records.length, matrix = Array.from({ length: n }, () => new Array(n).fill(0));
    for (let j = 0; j < n; j++) {
        const r = records[j];
        for (let side = 0; side < 2; side++) {
            const prefix = side === 0 ? '_inner' : '_outer', sign = side === 0 ? -1 : 1;
            const response = direct ? kirchhoffDirectContactResponse(bodies[side], r[prefix + 'NodeIndices'], r[prefix + 'NodeWeights'], 2, r.normal, sign)
                : { correction: new Float64Array(bodies[side].count * 6) };
            if (!direct) for (let k = 0; k < 2; k++) for (let axis = 0; axis < 3; axis++) {
                const node = r[prefix + 'NodeIndices'][k];
                response.correction[node * 6 + axis] += sign * r[prefix + 'NodeWeights'][k] * r.normal[axis] * bodies[side].inverseMass[node];
            }
            for (let i = 0; i < n; i++) {
                const row = records[i];
                for (let k = 0; k < 2; k++) for (let axis = 0; axis < 3; axis++) {
                    matrix[i][j] += sign * row[prefix + 'NodeWeights'][k] * row.normal[axis] * response.correction[row[prefix + 'NodeIndices'][k] * 6 + axis];
                }
            }
        }
        matrix[j][j] += r._normalAlpha;
        if (!direct) matrix[j][j] *= 1.001;
    }
    const rhs = records.map(r => -r.gap - r._normalAlpha * r.manifoldContact.normalLambda);
    let expected;
    for (let mask = 0; mask < 1 << n; mask++) {
        const free = Array.from({ length: n }, (_, i) => i).filter(i => mask & (1 << i));
        const delta = records.map(r => -r.manifoldContact.normalLambda);
        const solution = denseSolve(free.map(i => free.map(j => matrix[i][j])), free.map(i => rhs[i] - delta.reduce((s, d, j) => s + (mask & (1 << j) ? 0 : matrix[i][j] * d), 0)));
        if (!solution) continue;
        free.forEach((i, j) => { delta[i] = solution[j]; });
        if (delta.some((v, i) => v + records[i].manifoldContact.normalLambda < -1e-10)) continue;
        const residual = rhs.map((v, i) => v - matrix[i].reduce((s, a, j) => s + a * delta[j], 0));
        if (residual.some((v, i) => mask & (1 << i) ? Math.abs(v) > 1e-8 : v > 1e-8)) continue;
        expected = delta; break;
    }
    assert.ok(expected, 'dense oracle must find a complementary solution');
    const actual = solveKirchhoffContactBlock({ innerBody, outerBody, kirchhoffContacts: records, lumenMaxCorrection: 0.25, _contactBlockDirect: direct }, 1e-8, direct ? 0 : 1e-3);
    for (let i = 0; i < n; i++) assert.ok(Math.abs(actual.increment[i] - expected[i]) < 2e-5,
        `contact ${i}: ${actual.increment[i]} != ${expected[i]}`);
    assert.equal(innerBody.y[0], 0);
    assert.equal(outerBody.y[0], 0);
});


test('masked WebAssembly band solves agree with dense elimination', () => {
    for (const band of [1, 2, 9, 16]) for (const masked of [false, true]) {
        const n = 31;
        const a = Array.from({ length: n }, () => new Array(n).fill(0));
        const mask = Array.from({ length: n }, (_, i) => !masked || i % 4 !== 1);
        const rhs = Array.from({ length: n }, (_, i) => mask[i] ? Math.cos(i) : 0);
        // Symmetric, strictly diagonally dominant SPD matrix; includes rows
        // at both ends of the band and isolated constrained coordinates.
        for (let row = 0; row < n; row++) for (let col = Math.max(0, row - band + 1); col < row; col++) {
            a[row][col] = a[col][row] = Math.sin(row * 3 + col) * 0.1;
        }
        for (let row = 0; row < n; row++) a[row][row] = 1 + a[row].reduce((sum, value) => sum + Math.abs(value), 0);
        const kernel = createKirchhoffLinearKernel(n * band * 8 + n * 40 + 64);
        const matrix = kernel.alloc(Float64Array, n * band);
        const vector = kernel.alloc(Float64Array, n);
        const free = kernel.alloc(Uint8Array, n);
        free.set(mask); vector.set(rhs);
        const product = kernel.alloc(Float64Array, n), diagonal = kernel.alloc(Float64Array, n);
        diagonal.fill(0.25);
        for (let row = 0; row < n; row++) for (let col = Math.max(0, row - band + 1); col <= row; col++) matrix[row * band + row - col] = a[row][col];
        kernel.multiplyBand(matrix.byteOffset, vector.byteOffset, product.byteOffset, n, band, diagonal.byteOffset, 0.1);
        for (let row = 0; row < n; row++) {
            const expectedProduct = a[row].reduce((sum, value, col) => sum + value * rhs[col], 0) + 0.025 * rhs[row];
            assert.ok(Math.abs(product[row] - expectedProduct) < 1e-12);
        }
        for (let row = 0; row < n; row++) for (let col = 0; col < n; col++) if (!mask[row] || !mask[col]) a[row][col] = row === col ? 1 : 0;
        const expected = denseSolve(a, rhs);
        kernel.factorBand(matrix.byteOffset, n, band, masked ? free.byteOffset : -1, 0);
        kernel.solveBand(matrix.byteOffset, vector.byteOffset, n, band);
        for (let i = 0; i < n; i++) assert.ok(Math.abs(vector[i] - expected[i]) < 1e-12);
    }
});
