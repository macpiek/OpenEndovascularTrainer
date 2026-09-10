import test from 'node:test';
import assert from 'node:assert/strict';
import { condenseKirchhoffAxialBlock as condense } from '../src/physics/kirchhoffAxialCondensation.js';

function oracle(matrix, rhs) {
    const n = rhs.length, A = Array.from(matrix), b = Array.from(rhs);
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) if (Math.abs(A[i * n + k]) > Math.abs(A[pivot * n + k])) pivot = i;
        for (let j = k; j < n; j++) [A[k * n + j], A[pivot * n + j]] = [A[pivot * n + j], A[k * n + j]];
        [b[k], b[pivot]] = [b[pivot], b[k]];
        assert.notEqual(A[k * n + k], 0);
        for (let i = k + 1; i < n; i++) {
            const factor = A[i * n + k] / A[k * n + k];
            for (let j = k + 1; j < n; j++) A[i * n + j] -= factor * A[k * n + j];
            b[i] -= factor * b[k];
        }
    }
    for (let i = n - 1; i >= 0; i--) {
        for (let j = i + 1; j < n; j++) b[i] -= A[i * n + j] * b[j];
        b[i] /= A[i * n + i];
    }
    return b;
}
const close = (a, b, tolerance = 1e-12) => {
    assert.equal(a.length, b.length);
    a.forEach((v, i) => assert.ok(Math.abs(v - b[i]) <= tolerance * Math.max(1, Math.abs(b[i])), `${i}: ${v} != ${b[i]}`));
};

test('scaled condensation matches independent dense solve with unequal stiffness and retained ordering', () => {
    const base = [4, -1, .2, 0, -1, 3, -.5, .1, .2, -.5, 5, -2, 0, .1, -2, 6];
    const scales = [1e-5, 2, 1e4, .07];
    const matrix = base.map((v, k) => v * (scales[Math.floor(k / 4)] * scales[k % 4]));
    const rhs = [.3, -.2, 2, .8];
    for (const retainedIndices of [[3, 0], [2], [2, 0, 3, 1], []]) {
        const block = condense({ matrix, rhs, count: 4, retainedIndices });
        const x = block.recover(oracle(block.matrix, block.rhs));
        close(x, oracle(matrix, rhs), 1e-11);
        assert.ok(block.certify(x).normalizedResidual < 1e-14);
        assert.deepEqual(block.retainedIndices, retainedIndices);
    }
});

test('eliminated loading transfers to both boundaries and preserves the offdiagonal reaction', () => {
    // Middle degree connects both retained boundaries, carrying load 12.
    const block = condense({ matrix: [4, -2, 0, -2, 6, -3, 0, -3, 5], rhs: [1, 12, -1], count: 3, retainedIndices: [0, 2] });
    close(block.matrix, [4 - 4 / 6, -1, -1, 5 - 9 / 6]);
    close(block.rhs, [5, 5]);
    const x = block.recover(oracle(block.matrix, block.rhs));
    assert.ok(block.certify(x).normalizedResidual < 1e-15);
    const wrong = x.slice(); wrong[0] += 1;
    assert.ok(block.certify(wrong).normalizedResidual > .01);
});

test('full and empty retained blocks, including zero-size input, own recovered results', () => {
    const zero = condense({ matrix: [], rhs: [], count: 0, retainedIndices: [] });
    assert.equal(zero.recover([]).length, 0); assert.equal(zero.certify([]).normalizedResidual, 0);
    const all = condense({ matrix: [2, .5, .5, 3], rhs: [1, 2], count: 2, retainedIndices: [1, 0] });
    close(all.matrix, [3, .5, .5, 2]); close(all.rhs, [2, 1]); close(all.recover([4, 5]), [5, 4]);
    const none = condense({ matrix: [2, .5, .5, 3], rhs: [1, 2], count: 2, retainedIndices: [] });
    assert.equal(none.matrix.length, 0); assert.ok(none.certify(none.recover([])).normalizedResidual < 1e-15);
});

test('prior recovery and original certificate survive subsequent calls and input/output mutation', () => {
    const matrix = [4, -2, 0, -2, 6, -3, 0, -3, 5], rhs = [1, 12, -1];
    const block = condense({ matrix, rhs, count: 3, retainedIndices: [0, 2] });
    const retained = oracle(block.matrix, block.rhs), original = block.recover(retained);
    matrix.fill(999); rhs.fill(999); block.matrix.fill(999); block.rhs.fill(999);
    block.retainedIndices.reverse(); block.eliminatedIndices.fill(0);
    condense({ matrix: [7], rhs: [9], count: 1, retainedIndices: [] }).recover([]);
    assert.deepEqual(block.recover(retained), original);
    const next = block.recover(retained); next.fill(0);
    assert.deepEqual(block.recover(retained), original);
    assert.ok(block.certify(original).normalizedResidual < 1e-15);
});

test('reject invalid, asymmetric, singular or indefinite eliminated blocks without shifting', () => {
    const good = { matrix: [2, 1, 1, 2], rhs: [1, 2], count: 2, retainedIndices: [0] };
    for (const patch of [{ count: -1 }, { matrix: [2] }, { rhs: [NaN, 1] }, { matrix: [2, 1, 1.000000000000001, 2] },
        { retainedIndices: [0, 0] }, { retainedIndices: [2] }, { retainedIndices: [.1] }, { retainedIndices: undefined },
        { matrix: [2, 1, 1, 0] }, { matrix: [2, 1, 1, -1] },
        { matrix: [1, 1, 1, 1], retainedIndices: [] }, { matrix: [1, 2, 2, 1], retainedIndices: [] }])
        assert.throws(() => condense({ ...good, ...patch }));
    const block = condense(good);
    assert.throws(() => block.recover([])); assert.throws(() => block.recover([Infinity]));
    assert.throws(() => block.certify([1]));
});

test('reuses eliminated factor for independent owned response columns and rejects invalid or overflow rhs', () => {
    const block = condense({ matrix: [4, -2, 0, -2, 6, -3, 0, -3, 5], rhs: [1, 12, -1], count: 3, retainedIndices: [2] });
    assert.deepEqual(block.eliminatedIndices, [0, 1]);
    const rhs = [2, -3], snapshot = rhs.slice();
    const a = block.solveEliminated(rhs), b = block.solveEliminated([-1, 8]);
    close(a, oracle([4, -2, -2, 6], rhs));
    close(b, oracle([4, -2, -2, 6], [-1, 8]));
    assert.deepEqual(rhs, snapshot);
    b.fill(99); close(a, oracle([4, -2, -2, 6], rhs));
    assert.throws(() => block.solveEliminated([1]));
    assert.throws(() => block.solveEliminated([NaN, 1]));
    const tiny = condense({ matrix: [1e-300], rhs: [0], count: 1, retainedIndices: [] });
    assert.throws(() => tiny.solveEliminated([1e300]), /overflow/);
    assert.equal(tiny.solveEliminated([1e-300])[0], 1);
});

test('independent offsets use division while retaining boundary coupling and reactions', () => {
    const matrix = [8, 2, -1, 2, 4, 0, -1, 0, 2], rhs = [3, 7, -5];
    const block = condense({ matrix, rhs, count: 3, retainedIndices: [0] });
    assert.equal(block.diagnostics.eliminatedSolver, 'diagonal');
    assert.equal(block.diagnostics.factorEntries, 0);
    const solution = block.recover(oracle(block.matrix, block.rhs));
    close(solution, oracle(matrix, rhs));
    assert.ok(block.certify(solution).normalizedResidual < 1e-15);
    close(block.solveEliminated([8, -6]), [2, -3]);
});

test('even a tiny offset coupling takes the full factorization without dropping it', () => {
    const matrix = [8, 2, -1, 2, 4, 1e-20, -1, 1e-20, 2], rhs = [3, 7, -5];
    const block = condense({ matrix, rhs, count: 3, retainedIndices: [0] });
    assert.equal(block.diagnostics.eliminatedSolver, 'cholesky');
    assert.equal(block.diagnostics.factorEntries, 4);
    close(block.recover(oracle(block.matrix, block.rhs)), oracle(matrix, rhs));
});
