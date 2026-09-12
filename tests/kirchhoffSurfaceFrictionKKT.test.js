import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateKirchhoffSurfaceFrictionKKT as evaluate } from '../src/physics/kirchhoffSurfaceFriction.js';

test('surface KKT separates admissible sliding displacement from sticking error', () => {
    const slide = evaluate([-0.2, 0], [3, 0], 1, [0.2, 0.2]);
    assert.equal(slide.residualMm, 0);
    assert.equal(slide.coneViolation, 0);
    assert.equal(evaluate([-0.1, 0], [0.01, 0], 1, [0.2, 0.2]).residualMm, 0.005);
    assert.equal(evaluate([-0.2, 0], [-0.01, 0], 1, [0.2, 0.2]).residualMm, 0.02);
});

test('an anisotropic ellipse uses its force-domain normal and preserves millimetres under force scaling', () => {
    const angle = 0.4, a = 0.2, b = 0.05;
    const force = [a * Math.cos(angle), b * Math.sin(angle)];
    const displacement = [-force[0] / (a * a), -force[1] / (b * b)];
    for (const scale of [1e-12, 1, 1e12]) {
        const state = evaluate(force.map(v => v * scale), displacement, scale, [a, b]);
        assert.ok(state.residualMm < 1e-12);
        assert.ok(state.coneViolation < 1e-14);
        const wrong = evaluate(force.map(v => v * scale), [displacement[0] + 0.01, displacement[1]], scale, [a, b]);
        assert.ok(wrong.residualMm > 0.001);
    }
});

test('zero load and zero friction axes release motion while requiring zero force', () => {
    assert.equal(evaluate([0, 0], [100, -100], 0, [0.2, 0.1]).residualMm, 0);
    assert.equal(evaluate([0, -0.1], [100, 2], 1, [0, 0.1]).residualMm, 0);
    assert.equal(evaluate([1e-20, 0], [0, 0], 0, [0.2, 0.1]).coneViolation, Infinity);
    assert.ok(evaluate([0.201, 0], [-2, 0], 1, [0.2, 0.2]).coneViolation > 0.004);
});
