import assert from 'node:assert/strict';
import test from 'node:test';
import {
    createCompositeLengthConstraintWorkspace,
    evaluateCompositeLengthConstraints,
    assembleCompositeLengthConstraints,
    measureCompositeLengthConstraints,
} from '../src/physics/kirchhoffCompositeLengthConstraints.js';

function layoutFor(count) {
    // Two independent spins between successive position triples.
    const positions = Int32Array.from({ length: count }, (_, i) => 5 * i);
    return { nodeCount: count, positions, dofCount: 5 * count - 2, band: 8 };
}
const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance * (1 + Math.max(Math.abs(a), Math.abs(b))), `${a} != ${b}`);
const maxError = (a, b) => Math.max(...Array.from(a, (v, i) => Math.abs(v - b[i])));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
function dense(band, layout) {
    const n = layout.dofCount, out = new Float64Array(n * n);
    for (let i = 0; i < n; i++) for (let j = Math.max(0, i - layout.band + 1); j <= i; j++)
        out[i * n + j] = out[j * n + i] = band[i * layout.band + i - j];
    return out;
}
function fixture() {
    const layout = layoutFor(3), workspace = createCompositeLengthConstraintWorkspace(layout);
    return { layout, workspace, args: {
        positions: [[.1, .2, .3], [1.5, .8, -.4], [3.2, -.2, .7]],
        coordinates: [0, 1.6, 3.3], multipliers: [2, -.7], penalty: [4, 11], tolerance: 1e-8,
    } };
}

test('original length and signed physical reactions are distinct from AL trial multipliers', () => {
    const layout = layoutFor(2), workspace = createCompositeLengthConstraintWorkspace(layout);
    const args = { positions: [[0, 0, 0], [3, 4, 0]], coordinates: [10, 14.5], multipliers: [2], tolerance: .01, penalty: 8 };
    const result = assembleCompositeLengthConstraints(args, workspace);
    close(result.evaluation.lengths[0], 5); close(result.evaluation.residuals[0], .5);
    close(result.energy, 2); close(result.trialMultipliers[0], 6);
    close(result.evaluation.constraintForces[0], 1.2); close(result.evaluation.constraintForces[1], 1.6);
    close(result.trialForces[0], 3.6); close(result.trialForces[1], 4.8);
    assert.equal(result.evaluation.withinLengthTolerance, false);
    assert.deepEqual(args.multipliers, [2]); assert.equal('converged' in result, false);
    assert.equal(result.hessianType, 'gauss-newton');
});

test('zero AL trial force cannot certify a nonzero original length error', () => {
    const workspace = createCompositeLengthConstraintWorkspace(layoutFor(2));
    const args = { positions: [[0, 0, 0], [1.1, 0, 0]], coordinates: [0, 1], multipliers: [-1], penalty: 10, tolerance: 1e-6 };
    const result = assembleCompositeLengthConstraints(args, workspace);
    close(result.trialMultipliers[0], 0); assert.ok(result.gradient.every(v => Math.abs(v) < 1e-14));
    assert.equal(result.evaluation.withinLengthTolerance, false);
    close(result.evaluation.maximumLengthResidual, .1);
    close(result.evaluation.constraintForces[0], -1);
});

test('physical reactions obey action-reaction, zero net torque, and exact virtual work', () => {
    const { layout, workspace, args } = fixture(), out = evaluateCompositeLengthConstraints(args, workspace);
    const delta = [[.2, -.4, .3], [-.1, .6, -.2], [.3, .1, -.5]], total = [0, 0, 0], torque = [0, 0, 0];
    let work = 0, constraintWork = 0;
    for (let i = 0; i < args.positions.length; i++) {
        const p = args.positions[i], force = Array.from(out.constraintForces.slice(layout.positions[i], layout.positions[i] + 3));
        force.forEach((v, axis) => { total[axis] += v; }); work += dot(force, delta[i]);
        torque[0] += p[1] * force[2] - p[2] * force[1]; torque[1] += p[2] * force[0] - p[0] * force[2]; torque[2] += p[0] * force[1] - p[1] * force[0];
    }
    for (let e = 0; e < 2; e++) constraintWork -= args.multipliers[e] * dot(Array.from(out.directions.slice(3 * e, 3 * e + 3)), delta[e + 1].map((v, a) => v - delta[e][a]));
    total.concat(torque).forEach(v => close(v, 0, 1e-12)); close(work, constraintWork, 1e-12);
    for (const i of [3, 4, 8, 9]) assert.equal(out.constraintForces[i], 0);
});

test('exact constraint Jacobian and nonlinear AL gradient match independent energy differences', () => {
    const { layout, workspace, args } = fixture();
    const base = assembleCompositeLengthConstraints(args, workspace), gradient = [...base.gradient], jacobian = [...base.evaluation.jacobian], eps = 1e-6;
    for (let node = 0; node < 3; node++) for (let axis = 0; axis < 3; axis++) {
        const original = args.positions[node][axis], values = [];
        for (const sign of [-1, 1]) {
            args.positions[node][axis] = original + sign * eps;
            const result = assembleCompositeLengthConstraints(args, workspace);
            values.push({ energy: result.energy, residuals: [...result.evaluation.residuals] });
        }
        args.positions[node][axis] = original;
        close(gradient[layout.positions[node] + axis], (values[1].energy - values[0].energy) / (2 * eps), 2e-9);
        for (let e = 0; e < 2; e++) {
            const expected = node === e ? jacobian[6 * e + axis] : node === e + 1 ? jacobian[6 * e + 3 + axis] : 0;
            close(expected, (values[1].residuals[e] - values[0].residuals[e]) / (2 * eps), 2e-9);
        }
    }
});

test('GN band is mu J transpose J and explicitly excludes the nonlinear geometric Hessian', () => {
    const { layout, workspace, args } = fixture(), n = layout.dofCount;
    const result = assembleCompositeLengthConstraints(args, workspace), gn = dense(result.hessian, layout), exact = Float64Array.from(gn);
    const jac = [...result.evaluation.jacobian], trial = [...result.trialMultipliers], lengths = [...result.evaluation.lengths];
    const independent = new Float64Array(n * n);
    for (let e = 0; e < 2; e++) for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
        const gi = layout.positions[e + (i >= 3 ? 1 : 0)] + i % 3, gj = layout.positions[e + (j >= 3 ? 1 : 0)] + j % 3;
        independent[gi * n + gj] += args.penalty[e] * jac[6 * e + i] * jac[6 * e + j];
        const sign = (i < 3 ? -1 : 1) * (j < 3 ? -1 : 1), t0 = jac[6 * e + 3 + i % 3], t1 = jac[6 * e + 3 + j % 3];
        exact[gi * n + gj] += trial[e] / lengths[e] * sign * ((i % 3 === j % 3 ? 1 : 0) - t0 * t1);
    }
    close(maxError(independent, gn), 0, 1e-12);
    const eps = 1e-6;
    for (let node = 0; node < 3; node++) for (let axis = 0; axis < 3; axis++) {
        const original = args.positions[node][axis];
        args.positions[node][axis] = original + eps; const plus = [...assembleCompositeLengthConstraints(args, workspace).gradient];
        args.positions[node][axis] = original - eps; const minus = [...assembleCompositeLengthConstraints(args, workspace).gradient];
        args.positions[node][axis] = original;
        for (let row = 0; row < n; row++) close(exact[row * n + layout.positions[node] + axis], (plus[row] - minus[row]) / (2 * eps), 3e-9);
    }
    assert.ok(maxError(exact, gn) > .1, 'GN must not be labelled the exact nonlinear Hessian');
    const direction = Array.from({ length: n }, (_, i) => Math.sin(i));
    let quadratic = 0; for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) quadratic += direction[i] * gn[i * n + j] * direction[j];
    assert.ok(quadratic >= -1e-12);
});

test('AL adds to the existing common band without modifying independent spins', () => {
    const { layout, workspace, args } = fixture(), n = layout.dofCount;
    const target = { layout, energy: 3, gradient: new Float64Array(n).fill(.25), hessian: new Float64Array(n * layout.band).fill(.125) };
    const out = assembleCompositeLengthConstraints(args, workspace, target);
    close(target.energy, 3 + out.energy);
    target.gradient.forEach((v, i) => close(v, .25 + out.gradient[i]));
    target.hessian.forEach((v, i) => close(v, .125 + out.hessian[i]));
    for (const i of [3, 4, 8, 9]) { assert.equal(out.gradient[i], 0); assert.equal(target.gradient[i], .25); }
    const scalar = assembleCompositeLengthConstraints({ ...args, penalty: 4 }, workspace);
    assert.deepEqual([...scalar.penalties], [4, 4]);
});

test('final measurement uses fresh geometry and physical multipliers, independent of prior AL state', () => {
    const { workspace, args } = fixture();
    assembleCompositeLengthConstraints(args, workspace);
    const positions = [[0, 0, 0], [1.6, 0, 0], [3.3, 0, 0]], multipliers = [7, -3];
    const measured = measureCompositeLengthConstraints({ ...args, positions, multipliers }, workspace);
    assert.equal(measured.withinLengthTolerance, true); close(measured.maximumLengthResidual, 0);
    assert.deepEqual([...measured.physicalMultipliers], multipliers);
    assert.equal(measured.constraintForces[0], 7); assert.equal(measured.constraintForces[5], -10); assert.equal(measured.constraintForces[10], 3);
    assert.equal('trialMultipliers' in measured, false); assert.equal('converged' in measured, false);
});

test('length and force unit changes preserve the physical and augmented operators', () => {
    const { layout, workspace, args } = fixture(), base = assembleCompositeLengthConstraints(args, workspace);
    const copy = { energy: base.energy, gradient: [...base.gradient], hessian: [...base.hessian], forces: [...base.evaluation.constraintForces] };
    const lengthScale = 1000, forceScale = .01;
    const scaled = assembleCompositeLengthConstraints({
        ...args, positions: args.positions.map(p => p.map(v => v * lengthScale)), coordinates: args.coordinates.map(v => v * lengthScale),
        multipliers: args.multipliers.map(v => v * forceScale), penalty: args.penalty.map(v => v * forceScale / lengthScale), tolerance: args.tolerance * lengthScale,
    }, createCompositeLengthConstraintWorkspace(layout));
    close(scaled.energy / (forceScale * lengthScale), copy.energy);
    scaled.gradient.forEach((v, i) => close(v / forceScale, copy.gradient[i]));
    scaled.hessian.forEach((v, i) => close(v * lengthScale / forceScale, copy.hessian[i]));
    scaled.evaluation.constraintForces.forEach((v, i) => close(v / forceScale, copy.forces[i]));
});

test('tolerance and penalty are explicit, and invalid or collapsed geometry is rejected without a floor', () => {
    const { workspace, args } = fixture();
    for (const tolerance of [undefined, 0, -1, NaN, Infinity])
        assert.throws(() => evaluateCompositeLengthConstraints({ ...args, tolerance }, workspace), /tolerance/);
    for (const penalty of [undefined, 0, -1, NaN, Infinity, [1], [1, 0]])
        assert.throws(() => assembleCompositeLengthConstraints({ ...args, penalty }, workspace), /penalt/i);
    assert.throws(() => evaluateCompositeLengthConstraints({ ...args, coordinates: [0, 2, 1] }, workspace), /strictly increase/);
    assert.throws(() => evaluateCompositeLengthConstraints({ ...args, positions: [[0, 0, 0], [0, 0, 0], [1, 0, 0]] }, workspace), /collapse/);
    assert.throws(() => evaluateCompositeLengthConstraints({ ...args, multipliers: [Infinity, 0] }, workspace), /finite/);
    const tiny = evaluateCompositeLengthConstraints({ positions: [[0, 0, 0], [1e-200, 0, 0]], coordinates: [0, 1e-200], multipliers: [0], tolerance: 1e-210 }, createCompositeLengthConstraintWorkspace(layoutFor(2)));
    assert.equal(tiny.lengths[0], 1e-200); assert.equal(tiny.directions[0], 1); assert.equal(tiny.withinLengthTolerance, true);
    assert.throws(() => createCompositeLengthConstraintWorkspace({ ...layoutFor(2), band: 7 }), /band/);
});

test('finite-input overflow cannot produce an accepted operator or partially change a target', () => {
    const { layout, workspace, args } = fixture();
    assert.throws(() => evaluateCompositeLengthConstraints({ ...args, positions: [[-1e308, 0, 0], [1e308, 0, 0], [0, 1, 0]] }, workspace), /nonfinite/);
    assert.equal(workspace.evaluation.withinLengthTolerance, false);
    assert.throws(() => assembleCompositeLengthConstraints({ ...args, positions: [[0, 0, 0], [10, 0, 0], [20, 0, 0]], penalty: 1e308 }, workspace), /Nonfinite/);
    const short = createCompositeLengthConstraintWorkspace(layoutFor(2)), huge = {
        positions: [[0, 0, 0], [1, 0, 0]], coordinates: [0, 1], multipliers: [1e308], penalty: 1, tolerance: 1e-8,
    };
    const target = { layout: short.layout, energy: 2, gradient: new Float64Array(8), hessian: new Float64Array(64) };
    target.gradient[5] = 1e308; const before = [...target.gradient];
    assert.throws(() => assembleCompositeLengthConstraints(huge, short, target), /Nonfinite scattered/);
    assert.equal(target.energy, 2); assert.deepEqual([...target.gradient], before); assert.ok(target.hessian.every(v => v === 0));
});
