import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateKirchhoffContinuousFrictionKKT as evaluate } from '../src/physics/kirchhoffContinuousFrictionKKT.js';

test('circle maximum dissipation, sticking, reverse and transverse sliding', () => {
    assert.equal(evaluate([-3, -4], [6, 8], 10, [.5, .5]).residualMm, 0);
    assert.equal(evaluate([1, 2], [0, 0], 10, [.5, .5]).residualMm, 0);
    assert.equal(evaluate([3, 4], [6, 8], 10, [.5, .5]).residualMm, 16);
    assert.ok(evaluate([4, -3], [6, 8], 10, [.5, .5]).residualMm > 1);
    assert.ok(evaluate([-1.5, -2], [6, 8], 10, [.5, .5]).residualMm > 1);
});

test('anisotropic ellipse returns its analytical support point and preserves force scale', () => {
    // a=3,b=2, d=(2,-3); |D d|=sqrt(72).
    const slip = [2, -3], lambda = [-18 / Math.sqrt(72), 12 / Math.sqrt(72)];
    const result = evaluate(lambda, slip, 10, [.3, .2]);
    assert.ok(result.residualMm < 1e-14);
    assert.ok(result.coneViolation < 1e-14);
    for (const scale of [1e-12, 1e-4, 1e4, 1e12]) {
        const scaled = evaluate(lambda.map(v => v * scale), slip, 10 * scale, [.3, .2]);
        assert.ok(scaled.residualMm < 1e-14);
        assert.ok(scaled.coneViolation < 1e-14);
    }
});

test('near-boundary force error gives continuous displacement error without relaxing feasibility', () => {
    const slip = [.018, -.001], norm = Math.hypot(...slip);
    for (const deficit of [1e-3, 1e-6, 8e-9, 0]) {
        const lambda = slip.map(d => -.024 * (1 - deficit) * d / norm);
        const result = evaluate(lambda, slip, .12, [.2, .2]);
        assert.ok(Math.abs(result.residualMm - .018 * deficit) < 1e-16);
        assert.ok(result.coneViolation < 1e-15);
    }
    const invalid = evaluate([-.024 * (1 + 2e-8), 0], [.018, 0], .12, [.2, .2]);
    assert.ok(invalid.coneViolation > 1e-9, 'unchanged feasibility limit rejects an excessive force');
});

test('zero load and zero coefficient leave motion free but require zero force', () => {
    assert.equal(evaluate([0, 0], [100, 50], 0, [.2, .2]).residualMm, 0);
    assert.equal(evaluate([0, -2], [100, 50], 10, [0, .2]).residualMm, 0);
    assert.equal(evaluate([0, -1], [100, 50], 10, [0, .2]).residualMm, 25);
    assert.equal(evaluate([1e-20, -2], [100, 50], 10, [0, .2]).coneViolation, Infinity);
});

test('tiny radial force deficit is distinguished from a real slip-direction defect', () => {
    const radius = .024, slip = [.018, 0];
    const radial = evaluate([-radius * (1 - 8e-9), 0], slip, .12, [.2, .2]);
    assert.ok(radial.residualMm < 2e-10);
    const angle = .1;
    const wrongDirection = evaluate([-radius * Math.cos(angle), radius * Math.sin(angle)], slip, .12, [.2, .2]);
    assert.ok(wrongDirection.coneViolation < 1e-15, 'force is still feasible');
    assert.ok(wrongDirection.residualMm > .001, 'the existing displacement threshold rejects the direction defect');
});
