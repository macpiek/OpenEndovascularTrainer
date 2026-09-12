import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateCompositeSurfaceMotion as evaluate, createCompositeSurfaceMotionWorkspace as workspace } from '../src/physics/kirchhoffCompositeSurfaceMotion.js';
import { captureCompositeReferenceFrames, transportCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';
import { evaluateCompositeKinematics } from '../src/physics/kirchhoffCompositeKinematics.js';

const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}, tolerance ${tolerance}`);
const closeVector = (a, b, tolerance = 1e-9) => a.forEach((v, i) => close(v, b[i], tolerance));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => a.map(v => v / Math.hypot(...a));
const turn = (v, axis, angle) => {
    const a = unit(axis), c = Math.cos(angle), s = Math.sin(angle), k = cross(a, v), d = dot(a, v);
    return v.map((x, i) => c * x + s * k[i] + (1 - c) * d * a[i]);
};
function fixture() {
    const previousPositions = [[0, 0, 0], [2, .3, -.1], [5, 1.1, .4]], positions = [[.02, -.01, .03], [2.12, .36, -.03], [5.13, 1.22, .32]];
    return { positions, previousPositions, coordinates: [0, 2, 5], coordinate: 2.3, reference: captureCompositeReferenceFrames(previousPositions), dt: 1 / 120,
        reconstruction: 'quadratic-hinge', axes: { frame: 'material', toolId: 'wire', vectors: [[0, 1, 0], [0, 0, 1]] },
        tools: [
            { id: 'wire', angles: [.25, .79], previousAngles: [.2, .7], referenceTwist: 0, materialMap: { sStart: 100, dsDx: 1.2, dsDt: -.9 }, lever: { frame: 'material', vector: [.2, 0, 0] } },
            { id: 'catheter', angles: [-.27, .03], previousAngles: [-.3, .1], referenceTwist: 0, materialMap: { sStart: 200, dsDx: .7, dsDt: .4 }, lever: { frame: 'world', vector: [.1, .07, -.03] } }
        ] };
}
function change(input, dof, delta) {
    if (dof < 9) input.positions[Math.floor(dof / 3)][dof % 3] += delta;
    else input.tools[Math.floor((dof - 9) / 2)].angles[(dof - 9) % 2] += delta;
}

test('straight stationary opposite feed/spin uses non-unit qx and retains theta convection', () => {
    const positions = [[0, 0, 0], [4, 0, 0], [8, 0, 0]], dt = .01;
    const input = { positions, previousPositions: structuredClone(positions), coordinates: [0, 2, 4], coordinate: 2,
        reference: captureCompositeReferenceFrames(positions), dt, reconstruction: 'quadratic-hinge',
        axes: { frame: 'world', vectors: [[1, 0, 0], [0, 0, 1]] }, tools: [
            { id: 'wire', angles: [.22, .62], previousAngles: [.2, .6], referenceTwist: 0, materialMap: { sStart: 10, dsDx: 2, dsDt: -3 }, lever: { frame: 'world', vector: [0, .3, 0] } },
            { id: 'catheter', angles: [-.11, -.31], previousAngles: [-.1, -.3], referenceTwist: 0, materialMap: { sStart: 20, dsDx: .5, dsDt: .4 }, lever: { frame: 'world', vector: [0, .3, 0] } }
        ] };
    const r = evaluate(input), spins = [2 + 1.5 * .2, -1 + (-.8) * (-.1)];
    closeVector(r.positionDx, [2, 0, 0]); closeVector(r.tools[0].velocity, [3, 0, 0]); closeVector(r.tools[1].velocity, [-1.6, 0, 0]);
    r.tools.forEach((t, i) => { close(t.spin, spins[i]); closeVector(t.omega, [spins[i], 0, 0]); close(t.referenceTimeConnection, 0); close(t.referenceSpaceConnection, 0); });
    closeVector(r.slip, [dt * 4.6, dt * .3 * (spins[0] - spins[1])]);
    close(r.tools[0].materialLabel, 14); close(r.tools[1].materialLabel, 21);
    assert.notEqual(r.tools[0].spin, r.tools[0].thetaDt);
    close(r.reconstruction.affineComparison.positionDifferenceNorm, 0);
    r.tools.forEach(t => closeVector(t.affineCenterlineVelocityDifference, [0, 0, 0]));
});

test('a curved stationary centerline advects bending omega and actual surface velocity', () => {
    const curvature = .2, coordinate = 1.7, positions = [0, 2, 4].map(x => [x, curvature * x * x, 0]);
    const input = { positions, previousPositions: structuredClone(positions), coordinates: [0, 2, 4], coordinate,
        reference: captureCompositeReferenceFrames(positions, [0, 0, 1]), dt: 1 / 120, reconstruction: 'quadratic-hinge',
        axes: { frame: 'material', toolId: 'wire', vectors: [[1, 0, 0], [0, 0, 1]] }, tools: [
            { id: 'wire', angles: [0, 0], previousAngles: [0, 0], referenceTwist: 0,
                materialMap: { sStart: 30, dsDx: 1, dsDt: -2 }, lever: { frame: 'material', vector: [0, .3, 0] } }
        ] };
    const r = evaluate(input), slope = 2 * curvature * coordinate, omega = 2 * 2 * curvature / (1 + slope * slope), t = unit([1, slope, 0]);
    closeVector(r.positionDt, [0, 0, 0]); closeVector(r.positionDx, [1, slope, 0]);
    closeVector(r.tools[0].velocity, [2, 2 * slope, 0]); closeVector(r.tools[0].omega, [0, 0, omega]); close(r.tools[0].spin, 0);
    const lever = [t[1] * .3, -t[0] * .3, 0], rotationVelocity = cross([0, 0, omega], lever);
    closeVector(r.tools[0].surfaceVelocity, [2 + rotationVelocity[0], 2 * slope + rotationVelocity[1], 0]);
    assert.ok(Math.hypot(...rotationVelocity) > .1); assert.equal(r.reconstruction.equalsAffineEdgeField, false);
    close(r.reconstruction.affineComparison.positionDifference[1], curvature * coordinate * coordinate - .8 * coordinate / 2);
    assert.ok(Math.hypot(...r.tools[0].affineCenterlineVelocityDifference) > .1);
});

test('unwrapped reference winding selects a continuous spatial material field rather than zero twist', () => {
    const positions = [[0, 0, 0], [2, 0, 0], [4, 0, 0]], input = fixture();
    Object.assign(input, { positions, previousPositions: structuredClone(positions), coordinates: [0, 2, 4], coordinate: 1.6,
        reference: captureCompositeReferenceFrames(positions), axes: { frame: 'world', vectors: [[1, 0, 0], [0, 0, 1]] } });
    input.tools.forEach((t, i) => { t.angles = [0, 0]; t.previousAngles = [0, 0]; t.referenceTwist = (i ? -1 : 1) * 2 * Math.PI; t.materialMap.dsDx = 1; t.materialMap.dsDt = -1; });
    const r = evaluate(input);
    close(r.tools[0].referenceSpaceConnection, Math.PI); close(r.tools[1].referenceSpaceConnection, -Math.PI);
    close(r.tools[0].spin, Math.PI); close(r.tools[1].spin, -Math.PI);
    closeVector(r.tools[0].director1, [0, Math.cos(.3 * 2 * Math.PI), Math.sin(.3 * 2 * Math.PI)]);
});

test('common finite rigid-body motion has no false relative slip at the same common surface foot', () => {
    const input = fixture(), axis = [.3, -.4, .8], angle = .42, translation = [.4, -.2, .1];
    input.positions = input.previousPositions.map(p => turn(p, axis, angle).map((v, i) => v + translation[i]));
    const carried = transportCompositeReferenceFrames(input.reference, input.positions);
    const thetaShift = input.reference.map((f, edge) => {
        const material = turn(f.director, axis, angle), ref = carried[edge];
        return Math.atan2(dot(ref.tangent, cross(ref.director, material)), dot(ref.director, material));
    });
    input.axes = { frame: 'world', vectors: [[1, 0, 0], [0, 1, 0]] };
    input.tools.forEach((t, i) => { t.previousAngles = [i ? -.4 : .2, i ? -.4 : .2]; t.angles = thetaShift.map((v, edge) => v + t.previousAngles[edge]);
        t.materialMap.dsDt = 0; t.lever = { frame: 'world', vector: [.13, .21, -.09] }; });
    const r = evaluate(input);
    assert.ok(Math.hypot(...r.tools[0].omega) > 1); assert.ok(Math.hypot(...r.tools[0].surfaceVelocity) > 1);
    closeVector(r.tools[0].omega, r.tools[1].omega, 1e-11); closeVector(r.tools[0].surfaceVelocity, r.tools[1].surfaceVelocity, 1e-11);
    closeVector(r.slip, [0, 0], 1e-12);
});

test('accepted time-frame gauges preserve material orientation, full omega, slip and every Jacobian column', () => {
    const input = fixture(), gauged = structuredClone(input), gauge = [2 * Math.PI + .6, -2 * Math.PI - .4], original = evaluate(input);
    gauged.reference.forEach((f, edge) => { f.director = turn(f.director, f.tangent, gauge[edge]); });
    gauged.tools.forEach(t => {
        t.angles = t.angles.map((v, edge) => v - gauge[edge]); t.previousAngles = t.previousAngles.map((v, edge) => v - gauge[edge]);
        t.referenceTwist += gauge[1] - gauge[0];
    });
    const r = evaluate(gauged);
    closeVector(original.slip, r.slip, 2e-12); closeVector(original.jacobian, r.jacobian, 2e-10);
    r.tools.forEach((t, i) => { closeVector(t.director1, original.tools[i].director1, 1e-12); closeVector(t.omega, original.tools[i].omega, 2e-10); });
    assert.ok(Math.abs(r.tools[0].thetaDx - original.tools[0].thetaDx) > 1);
    assert.ok(Math.abs(r.tools[0].referenceSpaceConnection - original.tools[0].referenceSpaceConnection) > 1);
});

test('world-frame rotation and translation preserve slip and transform angular/material velocity covariantly', () => {
    const input = fixture(); input.axes = { frame: 'world', vectors: [[1, 0, 0], [0, 1, 0]] };
    const original = evaluate(input), rotated = structuredClone(input), axis = [.2, .7, -.3], angle = .63, shift = [3, -1, .4];
    for (const key of ['positions', 'previousPositions']) rotated[key] = rotated[key].map(p => turn(p, axis, angle).map((v, i) => v + shift[i]));
    rotated.reference.forEach(f => { f.tangent = turn(f.tangent, axis, angle); f.director = turn(f.director, axis, angle); });
    rotated.axes.vectors = rotated.axes.vectors.map(v => turn(v, axis, angle));
    rotated.tools.forEach(t => { if (t.lever.frame === 'world') t.lever.vector = turn(t.lever.vector, axis, angle); });
    const r = evaluate(rotated);
    closeVector(r.slip, original.slip, 2e-12);
    r.tools.forEach((t, i) => { closeVector(t.omega, turn(original.tools[i].omega, axis, angle), 1e-10); closeVector(t.surfaceVelocity, turn(original.tools[i].surfaceVelocity, axis, angle), 1e-10); });
    for (let row = 0; row < 2; row++) for (let node = 0; node < 3; node++) for (let k = 0; k < 3; k++) {
        const direction = turn([0, 1, 2].map(i => i === k ? 1 : 0), axis, angle);
        close(direction.reduce((sum, v, i) => sum + v * r.jacobian[row * r.dofCount + node * 3 + i], 0), original.jacobian[row * r.dofCount + node * 3 + k], 1e-9);
    }
});

test('full analytic slip Jacobian matches independent differences including neighbor q, both spins, moving levers and axes', () => {
    for (const materialAxes of [false, true]) {
        const input = fixture(); if (!materialAxes) input.axes = { frame: 'world', vectors: [[1, 0, 0], [0, 1, 0]] };
        const work = workspace(2), r = evaluate(input, work), n = r.dofCount;
        for (let dof = 0; dof < n; dof++) {
            const h = 1e-6, plus = structuredClone(input), minus = structuredClone(input); change(plus, dof, h); change(minus, dof, -h);
            const a = evaluate(plus, work), b = evaluate(minus, work);
            for (let row = 0; row < 2; row++) close((a.slip[row] - b.slip[row]) / (2 * h), r.jacobian[row * n + dof], 5e-8);
            for (let tool = 0; tool < 2; tool++) for (let axis = 0; axis < 3; axis++)
                close((a.tools[tool].omega[axis] - b.tools[tool].omega[axis]) / (2 * h), r.tools[tool].omegaJacobian[axis * n + dof], 2e-6);
        }
        assert.ok(Math.abs(r.tools[0].referenceTimeConnection) > 1e-6); assert.ok(Math.abs(r.tools[0].referenceSpaceConnection) > 1e-6);
        assert.ok(r.jacobian.slice(0, 3).some(v => Math.abs(v) > 1e-5)); assert.ok(r.jacobian.slice(6, 9).some(v => Math.abs(v) > 1e-5));
        assert.ok(r.jacobian.slice(9, 13).some(v => Math.abs(v) > 1e-5));
        assert.equal(r.derivatives.finiteDifferences, false);
    }
});

test('independent material-path finite differences of the entire director triad recover full omega', () => {
    const input = fixture(), r = evaluate(input), h = 1e-7, work = workspace(2);
    for (let tool = 0; tool < 2; tool++) {
        const variants = [-1, 1].map(sign => {
            const p = structuredClone(input), epsilon = sign * h;
            p.coordinate += epsilon * r.tools[tool].u;
            p.positions.forEach((position, node) => position.forEach((_, axis) => { position[axis] += epsilon * (input.positions[node][axis] - input.previousPositions[node][axis]) / input.dt; }));
            p.tools.forEach((t, i) => t.angles.forEach((_, edge) => { t.angles[edge] += epsilon * (input.tools[i].angles[edge] - input.tools[i].previousAngles[edge]) / input.dt; }));
            return evaluate(p, work);
        });
        const omega = [0, 0, 0], axes = [r.tools[tool].director1, r.tools[tool].director2, r.tangent];
        const before = [variants[0].tools[tool].director1, variants[0].tools[tool].director2, variants[0].tangent], after = [variants[1].tools[tool].director1, variants[1].tools[tool].director2, variants[1].tangent];
        axes.forEach((axis, j) => { const rate = Array.from(after[j], (v, i) => (v - before[j][i]) / (2 * h)), spin = cross(axis, rate); spin.forEach((v, i) => { omega[i] += v / 2; }); });
        closeVector(omega, r.tools[tool].omega, 2e-7);
    }
});

test('quadratic midpoint tangent/material endpoints and BE translational Kinematics agree with their declared fields', () => {
    const input = fixture(), carried = transportCompositeReferenceFrames(input.reference, input.positions);
    for (const [side, coordinate] of [1, 3.5].entries()) {
        const r = evaluate({ ...input, coordinate }); closeVector(r.tangent, carried[side].tangent, 1e-12);
        r.tools.forEach((t, tool) => closeVector(t.director1, turn(carried[side].director, carried[side].tangent, input.tools[tool].angles[side]), 1e-12));
    }
    const r = evaluate(input), oracle = evaluateCompositeKinematics({ positionDt: Array.from(r.positionDt), positionDx: Array.from(r.positionDx),
        tools: input.tools.map(t => ({ id: t.id, dsDx: t.materialMap.dsDx, dsDt: t.materialMap.dsDt })) });
    r.tools.forEach((t, i) => closeVector(t.velocity, oracle.tools[i].velocity, 1e-12));
    assert.equal(r.reconstruction.contactCertified, false);
});

test('stationary wall is one side, outputs survive workspace reuse and accepted inputs remain untouched', () => {
    const input = fixture(); input.tools = [input.tools[0]]; const before = structuredClone(input), work = workspace(1), first = evaluate(input, work), copy = structuredClone(first);
    closeVector(first.relativeVelocity, first.tools[0].surfaceVelocity);
    const different = structuredClone(input); different.tools[0].angles[1] += .1; evaluate(different, work);
    assert.deepEqual(first, copy); assert.deepEqual(input, before);
    first.tools[0].director1[0] = 999; first.slip[0] = 999; assert.deepEqual(input, before);
});

test('missing angular history/reconstruction and stale or degenerate time charts fail explicitly', () => {
    const input = fixture();
    for (const alter of [p => { delete p.reconstruction; }, p => { delete p.tools[0].previousAngles; },
        p => { delete p.tools[0].referenceTwist; }, p => { delete p.tools[0].materialMap.dsDt; },
        p => { p.reference[0].tangent = [1, 0, 0]; }, p => { p.positions[1] = [...p.positions[0]]; },
        p => { p.coordinate = 0; }, p => { p.axes.vectors = [[1, 0, 0], [1, 0, 0]]; }]) {
        const p = structuredClone(input); alter(p); const saved = structuredClone(p); assert.throws(() => evaluate(p)); assert.deepEqual(p, saved);
    }
});
