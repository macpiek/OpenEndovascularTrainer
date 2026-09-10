import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { auditFrozenCoupledSystem } from '../scripts/physics/audit-frozen-coupled-system.mjs';

// Unit normal load, unit tangential drive and mu=.5: sliding force is .5.
// The remaining drive is slip, not an unsatisfied tangential equality.
function sliding() {
    return { count: 3, band: 1, matrix: [1, 1, 1], rhs: [1, 1, 0],
        lower: [0, -Infinity, -Infinity], upper: [Infinity, Infinity, Infinity],
        increment: [1, .5, 0], rows: [{ alpha: 0 }, { alpha: 0 }, { alpha: 0 }],
        columns: [[[0, 1], [1, 1], [2, 1]]], weights: [[1, 1, 1]],
        groups: [{ rows: [1, 2], normalRow: 0, normalLambda: 0, mu: [.5, .5], lambda: [0, 0] }],
        options: { tolerance: 1e-8 } };
}

test('frozen audit certifies sliding using the solved normal load', () => {
    const result = auditFrozenCoupledSystem(sliding());
    assert.equal(result.passed, true);
    assert.equal(result.maximumResidual, 0);
});

test('frozen audit rejects an inconsistent J/W export even with a valid dual solution', () => {
    const data = sliding(); data.weights[0][0] = 2;
    const result = auditFrozenCoupledSystem(data);
    assert.equal(result.maximumResidual, 0);
    assert.equal(result.passed, false);
    assert.ok(result.maximumGramError > 0);
});

test('frozen audit rejects force outside the solved Coulomb cone', () => {
    const data = sliding(); data.increment[1] = .6;
    const result = auditFrozenCoupledSystem(data);
    assert.equal(result.passed, false);
    assert.ok(result.coneViolation > 1e-9);
});

test('frozen audit rejects a scalar-bound violation even with zero equation residual', () => {
    const data = sliding(); data.upper[0] = .9;
    const result = auditFrozenCoupledSystem(data);
    assert.equal(result.boundsValid, false);
    assert.equal(result.passed, false);
});

test('frozen audit cannot silently lose an off-band sparse coupling', () => {
    const data = sliding(); data.columns[0][0].push(1, 1);
    assert.throws(() => auditFrozenCoupledSystem(data), /band omits/);
});

test('actual 200 mm system preserves the full dual operator and final-load certificate', () => {
    const bytes = gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-coupled-full-200.json.gz', import.meta.url)));
    const data = JSON.parse(bytes, (_key, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const result = auditFrozenCoupledSystem(data);
    assert.equal(result.count, 2459);
    assert.equal(result.passed, true);
    assert.ok(result.maximumRelativeGramError < 1e-14);
    assert.equal(result.maximumResidual, data.diagnostics.maximumResidual);
});
