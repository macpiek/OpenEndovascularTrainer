import assert from 'node:assert/strict';
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import test from 'node:test';
import { createCompositeChainLayout, createCompositeChainWorkspace, solveCompositeChainIncrement } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeColdMaterialFixture } from './helpers/compositeColdMaterialFixture.js';

const frozen = JSON.parse(fs.readFileSync(new URL('./fixtures/compositeColdMaterialReference.json', import.meta.url)));
const hashOriginal = w => createHash('sha256').update(Buffer.from(w.matrix.buffer, w.matrix.byteOffset, w.matrix.byteLength))
    .update(Buffer.from(w.rhs.buffer, w.rhs.byteOffset, w.rhs.byteLength)).digest('hex');
const bits = new DataView(new ArrayBuffer(8));
function dyadic(value) {
    assert.ok(Number.isFinite(value));
    bits.setFloat64(0, value, false);
    const word = bits.getBigUint64(0, false), exponent = Number((word >> 52n) & 2047n);
    const fraction = word & ((1n << 52n) - 1n), sign = word >> 63n ? -1n : 1n;
    return [sign * (exponent ? (1n << 52n) + fraction : fraction), exponent ? exponent - 1075 : -1074];
}
function numberFromDyadic(integer, exponent) {
    if (!integer) return 0;
    const sign = integer < 0n ? -1 : 1, magnitude = integer < 0n ? -integer : integer;
    const shift = Math.max(0, magnitude.toString(2).length - 53);
    let head = magnitude >> BigInt(shift);
    if (shift) {
        const remainder = magnitude - (head << BigInt(shift)), half = 1n << BigInt(shift - 1);
        if (remainder > half || remainder === half && (head & 1n)) head++;
    }
    const power = exponent + shift;
    return sign * Number(head) * 2 ** Math.max(-1022, power) * 2 ** Math.min(0, power + 1022);
}
/** Exact BigInt products and sum of the supplied BINARY64 inputs. Neither the
 * production compensator nor its Cholesky/linear kernel is reused here.
 */
function exactResidual(w, x) {
    const { dofCount: n, band } = w.layout;
    return Array.from({ length: n }, (_, i) => {
        const terms = [dyadic(w.rhs[i])];
        for (let j = Math.max(0, i - band + 1); j < Math.min(n, i + band); j++) {
            const [a, ae] = dyadic(w.matrix[Math.max(i, j) * band + Math.abs(i - j)]), [b, be] = dyadic(x[j]);
            if (a && b) terms.push([-a * b, ae + be]);
        }
        const relevant = terms.filter(([v]) => v !== 0n);
        if (!relevant.length) return 0;
        const exponent = Math.min(...relevant.map(([, e]) => e));
        return numberFromDyadic(relevant.reduce((sum, [v, e]) => sum + (v << BigInt(e - exponent)), 0n), exponent);
    });
}
function assertOriginalResidual(w, result, fixed) {
    const exact = exactResidual(w, result.increment);
    let maximum = 0;
    exact.forEach((r, i) => {
        const tolerance = Math.max(1e-25, 1e-13 * Math.abs(r));
        assert.ok(Math.abs(r - result.residual[i]) <= tolerance, `Original row ${i}: ${r} vs ${result.residual[i]}`);
        if (fixed?.[i]) {
            assert.equal(result.increment[i], 0);
            assert.ok(Math.abs(result.reactions[i] + r) <= tolerance);
        } else { maximum = Math.max(maximum, Math.abs(r)); assert.equal(result.reactions[i], 0); }
    });
    assert.ok(Math.abs(result.maximumResidual - maximum) <= Math.max(1e-25, 1e-13 * maximum));
}

for (const elementBackend of ['javascript', 'wasm']) {
    test(`cold real profile310.017 recovers by one original-matrix refinement (${elementBackend})`, () => {
        const { workspace: w, diagonal } = createCompositeColdMaterialFixture({ elementBackend });
        const hessian = w.hessian.slice(), gradient = w.gradient.slice();
        const unrefined = solveCompositeChainIncrement(w, { diagonal, tolerance: 1e-7, maxRefinementSteps: 0 });
        assert.equal(unrefined.converged, false); assert.equal(unrefined.refinementSteps, 0);
        assertOriginalResidual(w, unrefined);
        assert.ok(unrefined.maximumResidual > 1.2e-7);
        const factor = w.factor.slice(), matrix = w.matrix.slice(), rhs = w.rhs.slice();
        const result = solveCompositeChainIncrement(w, { diagonal, tolerance: 1e-7 });
        assert.equal(result.converged, true); assert.equal(result.refinementSteps, 1);
        assert.equal(result.factorizations, 1); assert.equal(result.linearSolves, 2);
        assert.ok(result.maximumResidual < 6e-8); assert.equal(result.initialMaximumResidual, unrefined.maximumResidual);
        assertOriginalResidual(w, result);
        assert.deepEqual(w.hessian, hessian); assert.deepEqual(w.gradient, gradient);
        assert.deepEqual(w.factor, factor); assert.deepEqual(w.matrix, matrix); assert.deepEqual(w.rhs, rhs);
    });

    test(`frozen exact A/RHS agree with independent80-digit LDL (${elementBackend})`, () => {
        const w = createCompositeChainWorkspace(createCompositeChainLayout(frozen.edgeToolIds), { elementBackend });
        w.hessian.set(frozen.hessian); w.gradient.set(frozen.gradient);
        const result = solveCompositeChainIncrement(w, { diagonal: frozen.diagonal, tolerance: 1e-7 });
        assert.equal(hashOriginal(w), frozen.matrixRhsHash);
        assert.equal(result.converged, true); assertOriginalResidual(w, result);
        result.increment.forEach((v, i) => assert.ok(Math.abs(v - frozen.roundedReferenceDirection[i]) <=
            5e-16 * Math.max(1, Math.abs(v)), `High precision direction ${i}`));
        const original = exactResidual(w, frozen.roundedReferenceDirection);
        assert.equal(Math.max(...original.map(Math.abs)), frozen.roundedHighPrecisionMaximumOriginalResidual);
    });

    test(`fixed zero increments and reactions retain ORIGINAL applied operator through refinement (${elementBackend})`, () => {
        const { workspace: w, diagonal } = createCompositeColdMaterialFixture({ elementBackend });
        const fixed = new Uint8Array(w.layout.dofCount), extraGradient = new Float64Array(w.layout.dofCount);
        for (let i = 0; i < 10; i++) fixed[i] = 1;
        for (let i = 0; i < extraGradient.length; i++) extraGradient[i] = .03 * Math.sin(i / 7);
        const saved = { fixed: fixed.slice(), load: extraGradient.slice(), diagonal: diagonal.slice(), gradient: w.gradient.slice() };
        const result = solveCompositeChainIncrement(w, { diagonal, fixed, extraGradient, tolerance: 1e-7 });
        assert.equal(result.converged, true); assertOriginalResidual(w, result, fixed);
        assert.ok(result.reactions.some(v => v !== 0));
        assert.deepEqual(fixed, saved.fixed); assert.deepEqual(extraGradient, saved.load);
        assert.deepEqual(diagonal, saved.diagonal); assert.deepEqual(w.gradient, saved.gradient);
    });
}

test('a finite representability floor remains a visible failure after the exact refinement budget', () => {
    const { workspace: w, diagonal } = createCompositeColdMaterialFixture();
    const result = solveCompositeChainIncrement(w, { diagonal, tolerance: 1e-12, maxRefinementSteps: 3 });
    assert.equal(result.converged, false); assert.ok(result.maximumResidual > 1e-12);
    assert.equal(result.refinementSteps, 3); assert.equal(result.factorizations, 1); assert.equal(result.linearSolves, 4);
    assertOriginalResidual(w, result);
    for (const value of [-1, 4, .5, NaN]) assert.throws(() => solveCompositeChainIncrement(w, { diagonal, maxRefinementSteps: value }), /maxRefinementSteps/);
});

test('very large finite entries do not overflow product splitting or hide nonfinite rows', () => {
    const w = createCompositeChainWorkspace(createCompositeChainLayout([['wire'], ['wire']]));
    const n = w.layout.dofCount, band = w.layout.band;
    for (let i = 0; i < n; i++) { w.hessian[i * band] = 1e300; w.gradient[i] = -1e300 * (.123456789 + .01 * i); }
    const result = solveCompositeChainIncrement(w, { tolerance: 1e285 });
    assert.equal(result.converged, true); assertOriginalResidual(w, result);
    w.gradient[0] = Infinity;
    assert.throws(() => solveCompositeChainIncrement(w, { fixed: new Uint8Array(n).fill(1) }), /Nonfinite original/);
});

test('a roundoff-scale original singular pivot is still rejected with any refinement allowance', () => {
    const w = createCompositeChainWorkspace(createCompositeChainLayout([['wire'], ['wire']]));
    const n = w.layout.dofCount, band = w.layout.band;
    for (let i = 0; i < n; i++) w.hessian[i * band] = 1;
    w.hessian[band + 1] = 1 - Number.EPSILON;
    for (const maxRefinementSteps of [0, 2, 3]) assert.throws(() => solveCompositeChainIncrement(w, { maxRefinementSteps }), /singular/);
});
