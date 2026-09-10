import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeMaterialInertiaEdge } from '../src/physics/kirchhoffCompositeMaterialInertia.js';
import { assembleCompositeTranslationalInertia } from '../src/physics/kirchhoffCompositeKinematics.js';

const close = (a, b, tolerance = 2e-11) => assert.ok(Number.isFinite(a) && Number.isFinite(b) &&
    Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const vectorClose = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tolerance)); };
function fixture(id = 'wire') {
    const wire = id === 'wire';
    return { coordinates: [12, 14.7], previousPositions: wire ? [[1, 2, 3], [3.5, 2.1, 3.2]] : [[.9, 2.03, 3], [3.4, 2.2, 3.1]], dt: .04,
        tool: { id, massPerMaterialLength: wire ? .2 : .35,
            materialMap: { sStart: wire ? 30 : 20, dsDx: wire ? 1.2 : .8, dsDt: wire ? [-.7, 1.3] : [2.1, -.8] },
            oldMaterialVelocities: wire ? [[.3, -.1, .7], [-.8, .4, .2]] : [[-.2, .8, -.4], [.1, -.3, .5]] } };
}
const original = (input, positions) => assembleCompositeTranslationalInertia({ ...input, positions, tools: [input.tool] });
function compare(actual, expected, tolerance = 2e-11) {
    for (const field of ['energy', 'kineticEnergy', 'oldKineticEnergy', 'mass']) close(actual[field], expected[field], tolerance);
    for (const field of ['gradient', 'kineticGradient', 'hessian', 'momentumIncrement']) vectorClose(actual[field], expected[field], tolerance);
    for (const field of ['momentum', 'oldMomentum']) vectorClose(actual.tools[0][field], expected.tools[0][field], tolerance);
    actual.tools[0].samples.forEach((sample, i) => {
        for (const field of ['s', 'dsDt', 'u', 'massWeight']) close(sample[field], expected.tools[0].samples[i][field], tolerance);
        for (const field of ['coefficients', 'velocity', 'oldMaterialVelocity', 'velocityIncrement'])
            vectorClose(sample[field], expected.tools[0].samples[i][field], tolerance);
    });
}

test('prepared single-material E/g/H and momentum match the independent original operator for varying maps and geometry', () => {
    for (const id of ['wire', 'catheter']) {
        const input = fixture(id), compiled = createCompositeMaterialInertiaEdge(input);
        for (let iteration = 0; iteration < 6; iteration++) {
            const positions = input.previousPositions.map((p, n) => p.map((v, a) => v + .03 * Math.sin(iteration + 2 * n + a)));
            const actual = compiled.evaluate(positions); compare(actual, original(input, positions));
            assert.equal(actual.hessianValid, true); assert.equal(actual.certified, false); assert.equal(compiled.includesAngularInertia, false);
            assert.notEqual(actual.hessian[3], 0, 'the full physical off-diagonal mass must remain');
        }
    }
});

test('independent wire/catheter histories pull back once to common and full relative motion', () => {
    const wire = fixture(), catheter = fixture('catheter'), w = createCompositeMaterialInertiaEdge(wire), c = createCompositeMaterialInertiaEdge(catheter);
    const q = [.92, 2.05, 3.02, 3.42, 2.17, 3.13], rho = [.09, -.02, -.01, .12, -.07, .08], variables = q.concat(rho);
    const points = values => [values.slice(0, 3), values.slice(3, 6)];
    const evaluate = values => {
        const common = values.slice(0, 6), relative = values.slice(6), wirePoints = points(common.map((v, i) => v + relative[i]));
        return { wire: original(wire, wirePoints), catheter: original(catheter, points(common)) };
    };
    const actualW = structuredClone(w.evaluate(points(q.map((v, i) => v + rho[i])))), actualC = c.evaluate(points(q));
    const g = [...actualW.gradient].map((v, i) => v + actualC.gradient[i]).concat([...actualW.gradient]);
    const hessian = Array.from({ length: 12 }, (_, i) => Array.from({ length: 12 }, (_, j) =>
        actualW.hessian[6 * (i % 6) + j % 6] + (i < 6 && j < 6 ? actualC.hessian[6 * i + j] : 0)));
    const epsilon = 2e-6;
    for (let col = 0; col < 12; col++) {
        const plus = evaluate(variables.map((v, i) => v + (i === col ? epsilon : 0)));
        const minus = evaluate(variables.map((v, i) => v - (i === col ? epsilon : 0)));
        close(g[col], ((plus.wire.energy - minus.wire.energy) + (plus.catheter.energy - minus.catheter.energy)) / (2 * epsilon), 2e-8);
        for (let row = 0; row < 12; row++) {
            const gp = plus.wire.gradient[row % 6] + (row < 6 ? plus.catheter.gradient[row] : 0);
            const gm = minus.wire.gradient[row % 6] + (row < 6 ? minus.catheter.gradient[row] : 0);
            close(hessian[row][col], (gp - gm) / (2 * epsilon), 2e-8);
        }
    }
    const wrong = original({ ...wire, previousPositions: catheter.previousPositions }, points(q.map((v, i) => v + rho[i])));
    assert.ok(actualW.gradient.some((v, i) => Math.abs(v - wrong.gradient[i]) > 1), 'borrowing catheter history must measurably change wire momentum');
});

test('exact integrated momentum and kinetic energy retain the two independent material velocities', () => {
    const velocity = [3, -2, 1], input = fixture(), map = input.tool.materialMap;
    map.dsDt = 0; input.tool.oldMaterialVelocities = [[0, 0, 0], [0, 0, 0]];
    const compiled = createCompositeMaterialInertiaEdge(input), positions = input.previousPositions.map(p => p.map((v, i) => v + input.dt * velocity[i]));
    const result = compiled.evaluate(positions), mass = input.tool.massPerMaterialLength * map.dsDx * (input.coordinates[1] - input.coordinates[0]);
    close(result.kineticEnergy, .5 * mass * velocity.reduce((sum, v) => sum + v * v, 0));
    vectorClose(result.momentumIncrement, velocity.map(v => mass * v));
    for (let axis = 0; axis < 3; axis++) close(result.gradient[axis] + result.gradient[axis + 3], mass * velocity[axis] / input.dt);
    vectorClose(result.tools[0].samples[0].velocity, velocity); vectorClose(result.tools[0].samples[1].velocity, velocity);
});

test('prepared data owns both material maps and old positions/velocities while local output buffers are reused', () => {
    const input = fixture(), saved = structuredClone(input), positions = [[1.02, 2.04, 3.01], [3.51, 2.08, 3.18]];
    const compiled = createCompositeMaterialInertiaEdge(input), first = compiled.evaluate(positions), expected = original(saved, positions);
    input.coordinates.fill(999); input.previousPositions[0].fill(12); input.tool.materialMap.dsDt.fill(500);
    input.tool.massPerMaterialLength = 1e5; input.tool.oldMaterialVelocities[1].fill(-80);
    const again = compiled.evaluate(positions); assert.equal(again, first); compare(again, expected);
    assert.ok(Object.isFrozen(compiled));
});

test('gradient evaluation leaves poisoned Hessian bytes unused and a later full evaluation restores the original tangent', () => {
    const input = fixture(), compiled = createCompositeMaterialInertiaEdge(input), positions = [[1.02, 2.04, 3.01], [3.51, 2.08, 3.18]];
    const full = structuredClone(compiled.evaluate(positions)), buffer = compiled.evaluate(positions);
    buffer.hessian.fill(NaN); const gradient = compiled.evaluate(positions, { order: 'gradient' });
    assert.equal(gradient.hessianValid, false); assert.ok(gradient.hessian.every(Number.isNaN));
    assert.equal(gradient.energy, full.energy); assert.deepEqual(gradient.gradient, full.gradient); assert.deepEqual(gradient.tools, full.tools);
    const restored = compiled.evaluate(positions); assert.equal(restored.hessianValid, true); assert.deepEqual(restored.hessian, full.hessian);
    compare(restored, original(input, positions));
});

test('missing physical history, invalid parameters and nonfinite trials fail explicitly', () => {
    const input = fixture();
    for (const bad of [{ ...input, dt: 0 }, { ...input, tool: { ...input.tool, oldMaterialVelocities: undefined } },
        { ...input, tool: { ...input.tool, materialMap: { ...input.tool.materialMap, dsDx: 0 } } }])
        assert.throws(() => createCompositeMaterialInertiaEdge(bad));
    const compiled = createCompositeMaterialInertiaEdge(input);
    const previous = compiled.evaluate(input.previousPositions);
    assert.throws(() => compiled.evaluate([[NaN, 0, 0], [1, 0, 0]]), /Finite/);
    assert.equal(previous.hessianValid, false);
    assert.throws(() => compiled.evaluate([[1, 0, 0]]), /Two/);
    assert.throws(() => compiled.evaluate(input.previousPositions, { order: 'other' }), /order/);
    compare(compiled.evaluate(input.previousPositions), original(input, input.previousPositions));
});
