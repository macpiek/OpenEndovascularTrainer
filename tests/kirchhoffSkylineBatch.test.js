import assert from 'node:assert/strict';
import test from 'node:test';
import { createKirchhoffLinearKernel } from '../src/physics/kirchhoffLinearKernel.js';

function bytes(array) {
    return new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
}

function compareBatch(matrix, count, band, rhsValues, residualTolerance = 1e-12) {
    const kernel = createKirchhoffLinearKernel(matrix.byteLength + count * 128 + 256);
    const factor = kernel.alloc(Float64Array, matrix.length);
    const starts = kernel.alloc(Int32Array, count);
    const batch = kernel.alloc(Float64Array, count * 4);
    const singles = Array.from({ length: 4 }, () => kernel.alloc(Float64Array, count));
    const guard = kernel.alloc(Float64Array, 4);
    guard.fill(123.25);
    factor.set(matrix);
    kernel.findBandStarts(factor.byteOffset, starts.byteOffset, count, band);
    kernel.factorSkyline(factor.byteOffset, count, band, starts.byteOffset);
    const factorBefore = factor.slice(), startsBefore = starts.slice();
    // Reusing the same allocation must not retain another batch's RHS.
    for (const rhsValue of rhsValues) {
        for (let row = 0; row < count; row++) for (let axis = 0; axis < 4; axis++) {
            batch[row * 4 + axis] = singles[axis][row] = rhsValue(row, axis);
        }
        kernel.solveSkyline4(factor.byteOffset, batch.byteOffset, count, band, starts.byteOffset);
        for (const single of singles) kernel.solveSkyline(factor.byteOffset, single.byteOffset, count, band, starts.byteOffset);
        const expected = new Float64Array(count * 4);
        for (let row = 0; row < count; row++) for (let axis = 0; axis < 4; axis++) expected[row * 4 + axis] = singles[axis][row];
        assert.deepEqual(bytes(batch), bytes(expected), 'all channels preserve scalar operation order, including signed zero');
        for (let row = 0; row < count; row++) for (let axis = 0; axis < 4; axis++) {
            let product = 0, magnitude = Math.abs(rhsValue(row, axis));
            for (let col = Math.max(0, row - band + 1); col < Math.min(count, row + band); col++) {
                const a = row >= col ? matrix[row * band + row - col] : matrix[col * band + col - row];
                const term = a * batch[col * 4 + axis];
                product += term; magnitude += Math.abs(term);
            }
            assert.ok(Number.isFinite(batch[row * 4 + axis]));
            assert.ok(Math.abs(product - rhsValue(row, axis)) <= residualTolerance * Math.max(1, magnitude), `original equations row ${row}, channel ${axis}`);
        }
        assert.deepEqual(factor, factorBefore);
        assert.deepEqual(starts, startsBefore);
        assert.deepEqual([...guard], [123.25, 123.25, 123.25, 123.25]);
    }
}

for (const [count, band] of [[1, 1], [7, 1], [17, 5], [63, 63], [113, 19], [185, 185]]) {
    test(`four-RHS skyline solve is bit identical for irregular SPD matrix ${count}x${band}`, () => {
        const matrix = new Float64Array(count * band);
        for (let row = 0; row < count; row++) matrix[row * band] = 1;
        for (let row = 1; row < count; row++) for (let col = Math.max(0, row - band + 1); col < row; col++) {
            if ((row * 13 + col * 17) % 7 !== 0 && col !== row - 1) continue;
            const value = Math.cos(row + col) * 0.2;
            matrix[row * band + row - col] = value;
            matrix[row * band] += Math.abs(value);
            matrix[col * band] += Math.abs(value);
        }
        if (band > 1) matrix[(count - 1) * band + band - 1] = 1e-30;
        compareBatch(matrix, count, band, [
            (row, axis) => Math.sin(row * 0.3 + axis) * (axis === 0 ? 1e8 : axis === 1 ? 1e-8 : 1),
            () => 0,
            (row, axis) => axis === 3 ? -0 : row === axis ? 1 : 0,
        ]);
    });
}

for (const shift of [1e-8, 1e-10, 1e-12]) {
    test(`four-RHS skyline preserves near-null positive shift ${shift}`, () => {
        // Independent rank-one 2x2 blocks with a strictly positive diagonal shift.
        const count = 12, band = 2, matrix = new Float64Array(count * band);
        for (let row = 0; row < count; row++) {
            matrix[row * band] = 1 + shift;
            if (row % 2) matrix[row * band + 1] = 1;
        }
        compareBatch(matrix, count, band, [
            (row, axis) => Math.sin(row + axis),
            (row, axis) => axis === 0 ? 0 : (row % 2 ? 1 : -1) * axis,
        ]);
    });
}

test('empty four-RHS skyline solve does not access factor or RHS memory', () => {
    const kernel = createKirchhoffLinearKernel(256);
    const guard = kernel.alloc(Float64Array, 16);
    guard.fill(0.25);
    // Invalid pointers are safe only if count zero really performs no memory access.
    kernel.solveSkyline4(0x7fffffff, 0x7fffffff, 0, 0, 0x7fffffff);
    assert.ok(guard.every(value => value === 0.25));
});
