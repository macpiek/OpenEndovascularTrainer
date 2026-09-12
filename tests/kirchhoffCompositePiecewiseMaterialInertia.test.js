import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositePiecewiseMaterialInertiaEdge } from '../src/physics/kirchhoffCompositePiecewiseMaterialInertia.js';
import { createCompositeMaterialInertiaEdge } from '../src/physics/kirchhoffCompositeMaterialInertia.js';
import { createCompositeJointMaterialHistory } from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
const close = (a, b, tolerance = 2e-11) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const same = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tolerance)); };
const piece = (a, b, left, right = left) => ({ fractions: [a, b], oldMaterialVelocities: [left.slice(), right.slice()] });
function fixture() {
    return { coordinates: [12, 14.7], previousPositions: [[1, 2, 3], [3.5, 2.1, 3.2]], dt: .04,
        tool: { id: 'wire', massPerMaterialLength: .2, materialMap: { sStart: 30, dsDx: 1.2, dsDt: [-.7, 1.3] },
            oldVelocityPieces: [piece(0, .23, [.3, -.1, .7], [-.8, .4, .2]), piece(.23, .8, [1.1, .2, -.4], [.2, -.3, .1]), piece(.8, 1, [-.5, .3, 1], [.3, -.2, -.8])] } };
}
const positions = [[1.02, 2.04, 3.01], [3.51, 2.08, 3.18]];
// Independent integration of original-edge polynomials. Simpson is exact for
// every declared quadratic integrand on each history piece. No local operator
// or virtual-endpoint chain rule is used here.
function polynomial(input, current) {
    const { tool, dt } = input, L = input.coordinates[1] - input.coordinates[0], sx = tool.materialMap.dsDx;
    const rate = typeof tool.materialMap.dsDt === 'number' ? [tool.materialMap.dsDt, tool.materialMap.dsDt] : tool.materialMap.dsDt;
    const result = { energy: 0, kineticEnergy: 0, oldKineticEnergy: 0, mass: 0, gradient: new Float64Array(6), kineticGradient: new Float64Array(6), hessian: new Float64Array(36), momentumIncrement: new Float64Array(3), momentum: new Float64Array(3), oldMomentum: new Float64Array(3) };
    for (const piece of tool.oldVelocityPieces) {
        const [a, b] = piece.fractions;
        for (const [t, factor] of [[0, 1], [.5, 4], [1, 1]]) {
            const f = (1 - t) * a + t * b, st = (1 - f) * rate[0] + f * rate[1], u = -st / sx;
            const weight = tool.massPerMaterialLength * sx * L * (b - a) * factor / 6, J = [(1 - f) / dt - u / L, f / dt + u / L];
            result.mass += weight;
            for (let axis = 0; axis < 3; axis++) {
                const qt = ((1 - f) * (current[0][axis] - input.previousPositions[0][axis]) + f * (current[1][axis] - input.previousPositions[1][axis])) / dt;
                const v = qt + u * (current[1][axis] - current[0][axis]) / L;
                const old = (1 - t) * piece.oldMaterialVelocities[0][axis] + t * piece.oldMaterialVelocities[1][axis], increment = v - old;
                result.energy += .5 * weight * increment * increment; result.kineticEnergy += .5 * weight * v * v; result.oldKineticEnergy += .5 * weight * old * old;
                result.momentum[axis] += weight * v; result.oldMomentum[axis] += weight * old; result.momentumIncrement[axis] += weight * increment;
                for (let end = 0; end < 2; end++) {
                    result.gradient[3 * end + axis] += weight * J[end] * increment; result.kineticGradient[3 * end + axis] += weight * J[end] * v;
                    for (let other = 0; other < 2; other++) result.hessian[6 * (3 * end + axis) + 3 * other + axis] += weight * J[end] * J[other];
                }
            }
        }
    }
    return result;
}
function compare(actual, expected, tolerance = 2e-11) {
    for (const field of ['energy', 'kineticEnergy', 'oldKineticEnergy', 'mass']) close(actual[field], expected[field], tolerance);
    for (const field of ['gradient', 'kineticGradient', 'hessian', 'momentumIncrement']) same(actual[field], expected[field], tolerance);
    for (const field of ['momentum', 'oldMomentum']) same(actual.tools[0][field], expected[field], tolerance);
}

test('jump history integrates energy9.5 on one unchanged six-DOF edge while whole-edge Gauss gives7.5', () => {
    const history = createCompositeJointMaterialHistory({ materialVelocities: [
        { edge: 0, tools: [{ id: 'wire', sStart: 0, sEnd: 1, velocities: [[1, 0, 0], [1, 0, 0]], interpretation: 'physical-material-velocity' }] },
        { edge: 1, tools: [{ id: 'wire', sStart: 1, sEnd: 3, velocities: [[3, 0, 0], [3, 0, 0]], interpretation: 'physical-material-velocity' }] }] });
    const map = { sStart: 0, dsDx: 1, dsDt: 0 }, prepared = history.prepare({ coordinates: [0, 3], inertiaEdges: [{ tools: [{ id: 'wire', materialMap: map }] }] });
    assert.equal(prepared.requiresSubdivision, true);
    const input = { coordinates: [0, 3], previousPositions: [[0, 0, 0], [3, 0, 0]], dt: 1,
        tool: { id: 'wire', massPerMaterialLength: 1, materialMap: map, oldVelocityPieces: prepared.inertiaEdges[0].tools[0].pieces } };
    const op = createCompositePiecewiseMaterialInertiaEdge(input), r = op.evaluate(input.previousPositions);
    close(r.energy, 9.5); close(r.oldKineticEnergy, 9.5); close(r.kineticEnergy, 0); close(r.mass, 3); same(r.tools[0].oldMomentum, [7, 0, 0]); same(r.momentumIncrement, [-7, 0, 0]);
    assert.equal(op.dofCount, 6); assert.equal(r.gradient.length, 6); assert.equal(r.hessian.length, 36); assert.equal(r.tools[0].samples.length, 4);
    const wrong = [(1 - 1 / Math.sqrt(3)) * 1.5, (1 + 1 / Math.sqrt(3)) * 1.5].reduce((s, label) => s + .5 * 1.5 * history.sample('wire', label)[0] ** 2, 0);
    close(wrong, 7.5); assert.ok(Math.abs(wrong - r.energy) > 1); compare(r, polynomial(input, input.previousPositions));
});

test('piecewise E/g/fullH/momentum match independent original-edge polynomials with convection, nonunit metric and affine rates', () => {
    const input = fixture(), op = createCompositePiecewiseMaterialInertiaEdge(input), result = op.evaluate(positions);
    compare(result, polynomial(input, positions)); assert.equal(result.hessianValid, true); assert.equal(result.includesAngularInertia, false);
    assert.equal(result.quadrature.pieceCount, 3); assert.equal(result.tools[0].samples.length, 6); assert.notEqual(result.hessian[3], 0);
    const L = input.coordinates[1] - input.coordinates[0], { sStart, dsDx, dsDt } = input.tool.materialMap;
    for (const sample of result.tools[0].samples) {
        const f = sample.fraction, u = -((1 - f) * dsDt[0] + f * dsDt[1]) / dsDx;
        close(sample.s, sStart + dsDx * L * f); close(sample.u, u);
        same(sample.coefficients, [(1 - f) / input.dt - u / L, f / input.dt + u / L]);
        const oldPiece = input.tool.oldVelocityPieces[sample.pieceIndex], t = sample.localFraction;
        same(sample.oldMaterialVelocity, oldPiece.oldMaterialVelocities[0].map((v, k) => (1 - t) * v + t * oldPiece.oldMaterialVelocities[1][k]));
    }
    close(result.tools[0].samples.reduce((sum, s) => sum + s.massWeight, 0), result.mass);
});

test('independent energy and gradient finite differences recover all six force rows and full consistent Hessian', () => {
    const input = fixture(), op = createCompositePiecewiseMaterialInertiaEdge(input), r = structuredClone(op.evaluate(positions)), epsilon = 1e-6;
    for (let col = 0; col < 6; col++) {
        const plus = positions.map(p => p.slice()), minus = positions.map(p => p.slice()); plus[Math.floor(col / 3)][col % 3] += epsilon; minus[Math.floor(col / 3)][col % 3] -= epsilon;
        const a = polynomial(input, plus), b = polynomial(input, minus);
        close(r.gradient[col], (a.energy - b.energy) / (2 * epsilon), 2e-8); close(r.kineticGradient[col], (a.kineticEnergy - b.kineticEnergy) / (2 * epsilon), 2e-8);
        for (let row = 0; row < 6; row++) close(r.hessian[6 * row + col], (a.gradient[row] - b.gradient[row]) / (2 * epsilon), 2e-8);
    }
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) close(r.hessian[6 * i + j], r.hessian[6 * j + i], 1e-14);
});

test('one-piece response is unchanged from the existing original material inertia factory', () => {
    const input = fixture(), old = [[.3, -.1, .7], [-.8, .4, .2]]; input.tool.oldVelocityPieces = [piece(0, 1, ...old)];
    const { oldVelocityPieces, ...tool } = input.tool, original = createCompositeMaterialInertiaEdge({ ...input, tool: { ...tool, oldMaterialVelocities: old } });
    const op = createCompositePiecewiseMaterialInertiaEdge(input), a = op.evaluate(positions), b = original.evaluate(positions);
    for (const field of ['energy', 'kineticEnergy', 'oldKineticEnergy', 'mass', 'gradient', 'kineticGradient', 'hessian', 'momentumIncrement']) assert.deepEqual(a[field], b[field], field);
    for (const field of ['energy', 'kineticEnergy', 'oldKineticEnergy', 'mass', 'momentum', 'oldMomentum']) assert.deepEqual(a.tools[0][field], b.tools[0][field], field);
    a.tools[0].samples.forEach((s, i) => {
        for (const field of ['fraction', 's', 'dsDt', 'u', 'massWeight', 'coefficients', 'velocity', 'oldMaterialVelocity', 'velocityIncrement']) assert.deepEqual(s[field], b.tools[0].samples[i][field], field);
    });
});

test('constant rigid velocities and common frame translations preserve physical momentum and convective relative increments', () => {
    const input = fixture(), shift = [17, -9, 4], original = createCompositePiecewiseMaterialInertiaEdge(input), reference = structuredClone(original.evaluate(positions));
    const translated = structuredClone(input); translated.previousPositions = input.previousPositions.map(p => p.map((v, k) => v + shift[k]));
    compare(createCompositePiecewiseMaterialInertiaEdge(translated).evaluate(positions.map(p => p.map((v, k) => v + shift[k]))),
        { ...reference, momentum: reference.tools[0].momentum, oldMomentum: reference.tools[0].oldMomentum });
    const boost = [3, -2, 1], boosted = structuredClone(input);
    boosted.tool.oldVelocityPieces.forEach(p => p.oldMaterialVelocities.forEach(v => v.forEach((_, k) => v[k] += boost[k])));
    const r = createCompositePiecewiseMaterialInertiaEdge(boosted).evaluate(positions.map(p => p.map((v, k) => v + input.dt * boost[k])));
    close(r.energy, reference.energy); same(r.gradient, reference.gradient); same(r.hessian, reference.hessian); same(r.momentumIncrement, reference.momentumIncrement);
    same(r.tools[0].momentum, reference.tools[0].momentum.map((v, k) => v + r.mass * boost[k]));
    const rigid = fixture(); rigid.tool.materialMap.dsDt = 0; rigid.tool.oldVelocityPieces.forEach(p => p.oldMaterialVelocities.forEach(v => v.fill(0)));
    const motion = createCompositePiecewiseMaterialInertiaEdge(rigid).evaluate(rigid.previousPositions.map(p => p.map((v, k) => v + rigid.dt * boost[k])));
    same(motion.momentumIncrement, boost.map(v => v * motion.mass));
    for (let k = 0; k < 3; k++) close(motion.gradient[k] + motion.gradient[3 + k], motion.mass * boost[k] / rigid.dt);
});

test('prepared histories, maps and positions are owned; invalid trials and gradient-only calls cannot certify stale Hessians', () => {
    const input = fixture(), saved = structuredClone(input), op = createCompositePiecewiseMaterialInertiaEdge(input), expected = polynomial(saved, positions);
    const first = op.evaluate(positions), hessian = first.hessian.slice();
    input.coordinates.fill(999); input.previousPositions[0].fill(888); input.tool.materialMap.dsDt.fill(500); input.tool.massPerMaterialLength = 1e6;
    input.tool.oldVelocityPieces[0].fractions[1] = .99; input.tool.oldVelocityPieces[1].oldMaterialVelocities[0].fill(-444);
    const again = op.evaluate(positions); assert.equal(again, first); compare(again, expected);
    again.hessian.fill(NaN); const g = op.evaluate(positions, { order: 'gradient' }); assert.equal(g.hessianValid, false); assert.ok(g.hessian.every(Number.isNaN));
    same(g.gradient, expected.gradient); const full = op.evaluate(positions); assert.equal(full.hessianValid, true); assert.deepEqual(full.hessian, hessian);
    assert.throws(() => op.evaluate([[NaN, 0, 0], [1, 2, 3]]), /finite/); assert.equal(full.hessianValid, false);
    assert.throws(() => op.evaluate(positions, { order: 'unknown' }), /order/); assert.equal(full.hessianValid, false);
    compare(op.evaluate(positions), expected); assert.deepEqual(op.evaluate(positions), op.evaluate(positions)); assert.equal(Object.isFrozen(op), true);
});

test('coverage, density, map, history and finite guards reject without clamping or extrapolation', () => {
    for (const fractions of [[[.01, 1]], [[0, .4], [.5, 1]], [[0, .7], [.6, 1]], [[0, .9]], [[0, 1.01]], [[0, 0], [0, 1]], [[.5, 1], [0, .5]]]) {
        const input = fixture(); input.tool.oldVelocityPieces = fractions.map(([a, b]) => piece(a, b, [0, 0, 0])); assert.throws(() => createCompositePiecewiseMaterialInertiaEdge(input), /cover/);
    }
    for (const alter of [i => i.dt = 0, i => i.tool.massPerMaterialLength = 0, i => i.tool.materialMap.dsDx = -1,
        i => i.tool.materialMap.dsDt = null, i => i.tool.oldVelocityPieces = [], i => i.tool.oldVelocityPieces[0].oldMaterialVelocities[0][0] = NaN,
        i => i.tool.oldMaterialVelocities = [[0, 0, 0], [0, 0, 0]]]) {
        const input = fixture(); alter(input); assert.throws(() => createCompositePiecewiseMaterialInertiaEdge(input));
    }
    const input = fixture(), before = structuredClone(input); input.tool.oldVelocityPieces[1].fractions[0] = .25;
    assert.throws(() => createCompositePiecewiseMaterialInertiaEdge(input)); input.tool.oldVelocityPieces[1].fractions[0] = .23;
    assert.deepEqual(input, before); compare(createCompositePiecewiseMaterialInertiaEdge(input).evaluate(positions), polynomial(input, positions));
});
