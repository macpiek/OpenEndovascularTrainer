import test from 'node:test';
import assert from 'node:assert/strict';
import { Quaternion, Vector3 } from 'three';
import { boundHermiteGeometry, evaluateHermiteElement } from '../scripts/physics/curved-element-geometry.mjs';

test('straight material field preserves position, endpoint tangents and unit speed', () => {
    const a = [3, -2, 5], b = [3, -2, 17], d = [0, 0, 1], length = 12;
    const bounds = boundHermiteGeometry({ a, middle: [3, -2, 9.44], b, da: d, db: d, length, middleFraction: .37 });
    assert.ok(bounds.position.upper < 1e-6);
    assert.ok(bounds.stretch.upper < 1e-6);
    for (const t of [0, .13, .37, .75, 1]) {
        assert.ok(Math.hypot(...evaluateHermiteElement(a, b, d, d, length, t).map((v, i) => v - [3, -2, 5 + 12 * t][i])) < 1e-13);
        assert.ok(Math.hypot(...evaluateHermiteElement(a, b, d, d, length, t, true).map((v, i) => v - d[i])) < 1e-13);
    }
});

test('continuous screen detects a cubic bow with zero error at all three old vertices', () => {
    // Independent polynomial: x=t, y=t(1-t)(1-2t). Its interior extrema are
    // at (3 ± sqrt(3))/6 with |y|=1/(6 sqrt(3)); endpoint speed=sqrt(2).
    const bounds = boundHermiteGeometry({ a: [0, 0, 0], middle: [.5, 0, 0], b: [1, 0, 0],
        da: [1, 1, 0], db: [1, 1, 0], length: 1, middleFraction: .5 });
    const maximumPosition = 1 / (6 * Math.sqrt(3)), maximumStretch = Math.sqrt(2) - 1;
    for (const [bound, exact] of [[bounds.position, maximumPosition], [bounds.stretch, maximumStretch]]) {
        assert.ok(bound.lower <= exact && bound.upper >= exact, JSON.stringify({ bound, exact }));
        assert.ok(Math.max(exact - bound.lower, bound.upper - exact) < 2e-6);
        assert.equal(bound.depthLimited, false);
    }
});

test('unequal material intervals retain their geometry bounds under a rigid transform', () => {
    const input = { a: [-2, 1, 4], middle: [-1, 2, 5], b: [4, 3, 9], da: [.6, 0, .8],
        db: [0, .8, .6], length: 9, middleFraction: .23 };
    const q = new Quaternion().setFromAxisAngle(new Vector3(2, -1, 3).normalize(), .87);
    const vector = v => new Vector3(...v).applyQuaternion(q).toArray();
    const position = v => new Vector3(...vector(v)).add(new Vector3(11, -7, 25)).toArray();
    const transformed = { ...input, a: position(input.a), middle: position(input.middle), b: position(input.b),
        da: vector(input.da), db: vector(input.db) };
    const original = boundHermiteGeometry(input), moved = boundHermiteGeometry(transformed);
    for (const field of ['position', 'stretch']) for (const bound of ['lower', 'upper']) {
        assert.ok(Math.abs(original[field][bound] - moved[field][bound]) < 1e-12);
    }
    let positionMaximum = 0, stretchMaximum = 0;
    for (let k = 0; k <= 10000; k++) {
        const t = k / 10000, edge = t <= input.middleFraction ? [input.a, input.middle] : [input.middle, input.b];
        const u = t <= input.middleFraction ? t / input.middleFraction : (t - input.middleFraction) / (1 - input.middleFraction);
        const p = evaluateHermiteElement(input.a, input.b, input.da, input.db, input.length, t);
        const fine = edge[0].map((v, i) => v * (1 - u) + edge[1][i] * u);
        positionMaximum = Math.max(positionMaximum, Math.hypot(...p.map((v, i) => v - fine[i])));
        stretchMaximum = Math.max(stretchMaximum, Math.abs(Math.hypot(...evaluateHermiteElement(
            input.a, input.b, input.da, input.db, input.length, t, true)) - 1));
    }
    assert.ok(positionMaximum <= original.position.upper);
    assert.ok(stretchMaximum <= original.stretch.upper);
    assert.ok(original.position.upper - positionMaximum < 1e-5);
    assert.ok(original.stretch.upper - stretchMaximum < 1e-5);
});
