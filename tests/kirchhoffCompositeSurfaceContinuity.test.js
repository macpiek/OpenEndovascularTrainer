import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateCompositeSurfaceMotion as evaluate, createCompositeSurfaceMotionWorkspace } from '../src/physics/kirchhoffCompositeSurfaceMotion.js';
import { captureCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';

const close = (a, b, tolerance = 1e-10) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}, tolerance ${tolerance}`);
const equalVector = (a, b, tolerance = 1e-10) => a.forEach((v, i) => close(v, b[i], tolerance));
const difference = (a, b) => Array.from(a, (v, i) => v - b[i]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function source({ quadratic = false, coordinates = [0, 1, 2, 3], feed = 2 } = {}) {
    const positions = coordinates.map(x => [x, .2 * (quadratic ? x * x : x * x * x), 0]);
    return { coordinates, positions, previousPositions: structuredClone(positions), reference: captureCompositeReferenceFrames(positions, [0, 0, 1]),
        angles: [0, 0, 0], previousAngles: [0, 0, 0], feed, dt: 1 / 120 };
}
function patch(s, start) {
    return { coordinates: s.coordinates.slice(start, start + 3), positions: s.positions.slice(start, start + 3), previousPositions: s.previousPositions.slice(start, start + 3),
        coordinate: (s.coordinates[1] + s.coordinates[2]) / 2, reference: s.reference.slice(start, start + 2), dt: s.dt,
        reconstruction: 'quadratic-hinge', axes: { frame: 'material', toolId: 'wire', vectors: [[0, 0, 1], [1, 0, 0]] },
        tools: [{ id: 'wire', angles: s.angles.slice(start, start + 2), previousAngles: s.previousAngles.slice(start, start + 2), referenceTwist: 0,
            materialMap: { sStart: 100 + 2 * s.coordinates[start], dsDx: 2, dsDt: -2 * s.feed }, lever: { frame: 'material', vector: [0, .2, 0] } }] };
}
function both(s) {
    const work = createCompositeSurfaceMotionWorkspace(1);
    return [evaluate(patch(s, 0), work), evaluate(patch(s, 1), work)];
}
function interpolate(x, points, at) {
    const weights = x.map((xi, i) => x.reduce((weight, xj, j) => i === j ? weight : weight * (at - xj) / (xi - xj), 1));
    return [0, 1, 2].map(axis => points.reduce((sum, p, i) => sum + weights[i] * p[axis], 0));
}
function secondDerivative(x, points) {
    return [0, 1, 2].map(axis => x.reduce((sum, xi, i) => sum + 2 * points[i][axis] /
        x.reduce((denominator, xj, j) => i === j ? denominator : denominator * (xi - xj), 1), 0));
}

test('adjacent quadratic supports have the exact third-difference position jump despite matching tangents and material frames', () => {
    const s = source(), saved = structuredClone(s), [left, right] = both(s);
    const expected = s.positions[0].map((_, axis) => (-s.positions[0][axis] + 3 * s.positions[1][axis] - 3 * s.positions[2][axis] + s.positions[3][axis]) / 8);
    equalVector(left.position, interpolate(s.coordinates.slice(0, 3), s.positions.slice(0, 3), 1.5));
    equalVector(right.position, interpolate(s.coordinates.slice(1), s.positions.slice(1), 1.5));
    equalVector(difference(left.position, right.position), expected); close(expected[1], .15);
    equalVector(left.positionDx, right.positionDx); equalVector(left.tangent, right.tangent);
    equalVector(left.tools[0].director1, right.tools[0].director1); equalVector(left.tools[0].director2, right.tools[0].director2);
    close(left.tools[0].materialLabel, right.tools[0].materialLabel);
    assert.deepEqual(s, saved); assert.equal(left.reconstruction.contactCertified, false); assert.equal(right.reconstruction.contactCertified, false);
});

test('stationary curved material feed has a bending-omega and finite-radius slip jump at the common midpoint', () => {
    const s = source(), [left, right] = both(s), qx = difference(s.positions[2], s.positions[1]);
    const seconds = [secondDerivative(s.coordinates.slice(0, 3), s.positions.slice(0, 3)), secondDerivative(s.coordinates.slice(1), s.positions.slice(1))];
    const omega = seconds.map(qxx => cross(qx, qxx).map(v => s.feed * v / dot(qx, qx)));
    equalVector(left.tools[0].omega, omega[0]); equalVector(right.tools[0].omega, omega[1]);
    equalVector(left.tools[0].velocity, right.tools[0].velocity); close(left.tools[0].spin, 0); close(right.tools[0].spin, 0);
    const velocityJump = cross(difference(omega[0], omega[1]), left.tools[0].lever);
    const expected = left.axes.map(axis => s.dt * dot(axis, velocityJump));
    equalVector(difference(left.slip, right.slip), expected); close(expected[0], -.0013513513513513514);
    assert.ok(Math.hypot(...difference(left.tangentDx, right.tangentDx)) > .1);
    assert.ok(Math.hypot(...difference(left.slip, right.slip)) > .001);
});

test('two tools with opposite feed retain the boundary jump in their relative material slip', () => {
    const s = source(), inputs = [patch(s, 0), patch(s, 1)];
    inputs.forEach(input => input.tools.push({ ...structuredClone(input.tools[0]), id: 'catheter',
        materialMap: { sStart: 200 + .5 * input.coordinates[0], dsDx: .5, dsDt: .5 } }));
    const work = createCompositeSurfaceMotionWorkspace(2), left = evaluate(inputs[0], work), right = evaluate(inputs[1], work);
    close(difference(left.slip, right.slip)[0], -.0020270270270270273);
    left.tools.forEach((t, i) => { equalVector(t.director1, right.tools[i].director1); close(t.materialLabel, right.tools[i].materialLabel); });
});

test('a straight continuous centerline still has a slip jump when independent edge-angle slopes differ', () => {
    const s = source(); s.positions = s.coordinates.map(x => [x, 0, 0]); s.previousPositions = structuredClone(s.positions);
    s.reference = captureCompositeReferenceFrames(s.positions, [0, 0, 1]); s.angles = [0, .2, .8]; s.previousAngles = [...s.angles];
    const [left, right] = both(s);
    equalVector(left.position, right.position); equalVector(left.tangent, right.tangent); equalVector(left.tangentDx, [0, 0, 0]); equalVector(right.tangentDx, [0, 0, 0]);
    equalVector(left.tools[0].director1, right.tools[0].director1); equalVector(left.tools[0].director2, right.tools[0].director2);
    close(left.tools[0].thetaDx, .2); close(right.tools[0].thetaDx, .6);
    close(left.tools[0].spin, .4); close(right.tools[0].spin, 1.2);
    close(difference(left.slip, right.slip)[1], .0013333333333333333);
});

test('BE centerline motion can jump at the interface even with zero feed and unchanged common-edge geometry', () => {
    const s = source({ feed: 0 }), displacement = [0, .08, 0];
    s.positions[3] = s.positions[3].map((v, i) => v + displacement[i]);
    const [left, right] = both(s);
    equalVector(left.tangentDt, right.tangentDt); equalVector(left.tools[0].omega, right.tools[0].omega);
    equalVector(left.positionDt, [0, 0, 0]);
    equalVector(difference(left.positionDt, right.positionDt), displacement.map(v => v / (8 * s.dt)));
    equalVector(difference(left.slip, right.slip), left.axes.map(axis => dot(axis, displacement) / 8));
    assert.ok(Math.hypot(...difference(left.slip, right.slip)) > .008);
});

test('the nonuniform-grid jump follows the cubic divided difference; a globally quadratic control is continuous', () => {
    const s = source({ coordinates: [0, 1, 2.4, 4] }), [left, right] = both(s), h = s.coordinates[2] - s.coordinates[1];
    let divided = s.positions.map(p => [...p]);
    for (let order = 1; order <= 3; order++) divided = divided.slice(0, -1).map((p, i) => p.map((v, axis) =>
        (divided[i + 1][axis] - v) / (s.coordinates[i + order] - s.coordinates[i])));
    const expected = divided[0].map(v => v * h * h * (s.coordinates[3] - s.coordinates[0]) / 4);
    equalVector(difference(left.position, right.position), expected); assert.ok(Math.hypot(...expected) > .3);
    equalVector(left.tangent, right.tangent); equalVector(left.tools[0].director1, right.tools[0].director1);
    const [a, b] = both(source({ quadratic: true }));
    for (const key of ['position', 'positionDx', 'tangent', 'tangentDx', 'slip']) equalVector(a[key], b[key]);
    equalVector(a.tools[0].omega, b.tools[0].omega);
});

test('the exact quadratic-versus-affine position envelope is attained at each patch midpoint', () => {
    const s = source({ coordinates: [0, 1, 2.4, 4] });
    for (const start of [0, 1]) {
        const x = s.coordinates.slice(start, start + 3), q = s.positions.slice(start, start + 3), qxx = secondDerivative(x, q);
        for (const edge of [0, 1]) {
            const h = x[edge + 1] - x[edge], midpoint = (x[edge] + x[edge + 1]) / 2, endpoint = x[1], bound = h * h * Math.hypot(...qxx) / 8;
            let maximum = 0;
            for (let i = 0; i <= 20; i++) {
                const at = midpoint + (endpoint - midpoint) * i / 20, f = (at - x[edge]) / h;
                const affine = q[edge].map((v, axis) => v + f * (q[edge + 1][axis] - v));
                const delta = difference(interpolate(x, q, at), affine);
                equalVector(delta, qxx.map(v => v * (at - x[edge]) * (at - x[edge + 1]) / 2));
                maximum = Math.max(maximum, Math.hypot(...delta)); assert.ok(Math.hypot(...delta) <= bound + 1e-12);
            }
            close(maximum, bound);
        }
    }
});
