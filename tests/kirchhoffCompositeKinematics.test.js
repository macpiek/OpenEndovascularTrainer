import assert from 'node:assert/strict';
import test from 'node:test';
import {
    evaluateCompositeKinematics as kinematics, createCompositeInertiaWorkspace,
    assembleCompositeTranslationalInertia as assemble, scatterCompositeTranslationalInertia as scatter
} from '../src/physics/kirchhoffCompositeKinematics.js';
import { createCompositeChainLayout, createCompositeChainWorkspace } from '../src/physics/kirchhoffCompositeChain.js';

const close = (a, b, tolerance = 1e-10) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const vectorClose = (a, b, tolerance) => a.forEach((v, i) => close(v, b[i], tolerance));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
function input() {
    return { coordinates: [2, 5], dt: .13,
        positions: [[.2, -.1, .4], [3.3, .7, -.2]], previousPositions: [[.16, -.08, .38], [3.25, .67, -.15]],
        tools: [
            { id: 'wire', materialMap: { sStart: 170, dsDx: 1.3, dsDt: [-.7, -2.1] },
                massPerMaterialLength: 2.4, oldMaterialVelocities: [[.3, -.2, .1], [.8, .4, -.3]] },
            { id: 'catheter', materialMap: { sStart: 82, dsDx: .8, dsDt: [.4, 1.2] },
                massPerMaterialLength: 4.1, oldMaterialVelocities: [[-.7, .1, .2], [-.4, -.1, .6]] }
        ] };
}
function move(data, dof, amount) { data.positions[Math.floor(dof / 3)][dof % 3] += amount; }
const rotate = v => [-v[1], v[2], -v[0]]; // proper orthogonal transform

test('feed on a stationary non-unit axis gives separate physical material velocities', () => {
    const result = kinematics({ positionDt: [0, 0, 0], positionDx: [2, .5, -1], tools: [
        { id: 'wire', dsDx: 2, dsDt: -6 }, { id: 'catheter', dsDx: .5, dsDt: 1 }
    ] });
    vectorClose(result.tools[0].velocity, [6, 1.5, -3]); vectorClose(result.tools[1].velocity, [-4, -1, 2]);
    assert.equal(result.tools[0].u, 3); assert.equal(result.tools[1].u, -2);
    assert.equal(result.tools[0].spin, null); assert.equal(result.tools[0].spinKnown, false);
});

test('opposite feed and spin preserve theta convection and each material-path frame spin', () => {
    const result = kinematics({ positionDt: [.1, .2, .3], positionDx: [1, 0, 0], tools: [
        { id: 'wire', dsDx: 2, dsDt: -6, thetaDt: .4, thetaDx: .2, frameSpin: { dt: .1, dx: -.03 } },
        { id: 'catheter', dsDx: .5, dsDt: 1, thetaDt: -.8, thetaDx: -.3, frameSpin: .25 }
    ] });
    close(result.tools[0].spin, .4 + 3 * .2 + .1 - 3 * .03);
    close(result.tools[1].spin, -.8 + (-2) * (-.3) + .25);
    close(result.tools[0].frameSpin, .01); close(result.tools[1].frameSpin, .25);
    assert.ok(result.tools.every(t => t.spinKnown));
    assert.throws(() => kinematics({ positionDt: [0, 0, 0], positionDx: [1, 0, 0], tools: [
        { id: 'wire', dsDx: 1, dsDt: 0, thetaDt: 0, thetaDx: 0 }
    ] }), /frameSpin/);
});

test('old MATERIAL velocity sustains steady opposite transport while kinetic energy remains positive', () => {
    const data = { coordinates: [0, 2], positions: [[0, 0, 0], [4, 1, 0]], previousPositions: [[0, 0, 0], [4, 1, 0]], dt: .1,
        tools: [
            { id: 'wire', massPerMaterialLength: 2, materialMap: { sStart: 10, dsDx: 2, dsDt: -6 },
                oldMaterialVelocities: [[6, 1.5, 0], [6, 1.5, 0]] },
            { id: 'catheter', massPerMaterialLength: 3, materialMap: { sStart: 20, dsDx: .5, dsDt: 1 },
                oldMaterialVelocities: [[-4, -1, 0], [-4, -1, 0]] }
        ] };
    const result = assemble(data);
    close(result.energy, 0); vectorClose(result.gradient, new Array(6).fill(0));
    close(result.kineticEnergy, .5 * 8 * (36 + 2.25) + .5 * 3 * 17);
    close(result.oldKineticEnergy, result.kineticEnergy);
    assert.equal(result.includesAngularInertia, false);
    assert.equal(result.energyKind, 'material-velocity-increment');
    const wrong = structuredClone(data); wrong.tools.forEach(tool => tool.oldMaterialVelocities = [[0, 0, 0], [0, 0, 0]]);
    assert.ok(assemble(wrong).energy > 100, 'old centerline time velocity is not the old material velocity');
});

test('nonzero linear s_t keeps the full convective gradient and exact Hessian', () => {
    const data = input(), result = assemble(data), gradient = result.gradient.slice(), hessian = result.hessian.slice();
    const kineticGradient = result.kineticGradient.slice(), h = 2e-6;
    for (let dof = 0; dof < 6; dof++) {
        move(data, dof, h); const plus = assemble(data);
        move(data, dof, -2 * h); const minus = assemble(data);
        move(data, dof, h);
        close(gradient[dof], (plus.energy - minus.energy) / (2 * h), 2e-8);
        close(kineticGradient[dof], (plus.kineticEnergy - minus.kineticEnergy) / (2 * h), 2e-8);
        for (let row = 0; row < 6; row++) {
            close(hessian[6 * row + dof], (plus.gradient[row] - minus.gradient[row]) / (2 * h), 2e-8);
            close(hessian[6 * row + dof], (plus.kineticGradient[row] - minus.kineticGradient[row]) / (2 * h), 2e-8);
        }
    }
    const noConvection = input(); noConvection.tools.forEach(tool => tool.materialMap.dsDt = 0);
    assert.ok(result.hessian.some((v, i) => Math.abs(v - assemble(noConvection).hessian[i]) > 1));
});

test('consistent edge mass blocks include both convection cross terms and negative off-diagonal entries', () => {
    const length = 3, dt = .2, u = 10, lambda = 1.4, rho = 2, mass = length * lambda * rho;
    const result = assemble({ coordinates: [0, length], dt, positions: [[0, 0, 0], [3, 0, 0]], previousPositions: [[0, 0, 0], [3, 0, 0]],
        tools: [{ id: 'wire', materialMap: { sStart: 4, dsDx: lambda, dsDt: -u * lambda }, massPerMaterialLength: rho,
            oldMaterialVelocities: [[0, 0, 0], [0, 0, 0]] }] });
    const expected = [mass * (1 / (3 * dt ** 2) - u / (length * dt) + u ** 2 / length ** 2),
        mass * (1 / (6 * dt ** 2) - u ** 2 / length ** 2),
        mass * (1 / (3 * dt ** 2) + u / (length * dt) + u ** 2 / length ** 2)];
    assert.ok(expected[1] < 0);
    for (let axis = 0; axis < 3; axis++) {
        close(result.hessian[6 * axis + axis], expected[0]);
        close(result.hessian[6 * axis + 3 + axis], expected[1]);
        close(result.hessian[6 * (3 + axis) + 3 + axis], expected[2]);
    }
    for (const v of [[1, 2, 3, -2, 1, .2], [1, 0, 0, 1, 0, 0], [1, 0, 0, -1, 0, 0]])
        assert.ok(dot(v, v.map((_, row) => dot(Array.from(result.hessian.subarray(6 * row, 6 * row + 6)), v))) > 0);
});

test('a cancelling affine transport map preserves its exact inertial null mode without a diagonal floor', () => {
    const data = { coordinates: [0, 2], dt: .2, positions: [[0, 0, 0], [2, 0, 0]], previousPositions: [[0, 0, 0], [2, 0, 0]],
        tools: [{ id: 'wire', materialMap: { sStart: 10, dsDx: 1, dsDt: [-5, 5] }, massPerMaterialLength: 2,
            oldMaterialVelocities: [[5, 0, 0], [-5, 0, 0]] }] };
    const result = assemble(data), nullMode = [-1, 0, 0, 1, 0, 0];
    for (let row = 0; row < 6; row++) close(dot(Array.from(result.hessian.subarray(6 * row, 6 * row + 6)), nullMode), 0);
    data.positions[0][0] -= .1; data.positions[1][0] += .1;
    close(assemble(data).energy, 0);
});

test('Gauss-2 matches independent exact Simpson integration for linear old velocity and linear material-map rate', () => {
    const data = input(), result = assemble(data), length = data.coordinates[1] - data.coordinates[0];
    let incremental = 0, kinetic = 0;
    for (const tool of data.tools) for (const [fraction, weight] of [[0, 1], [.5, 4], [1, 1]]) {
        const n0 = 1 - fraction, n1 = fraction;
        const rate = n0 * tool.materialMap.dsDt[0] + n1 * tool.materialMap.dsDt[1];
        const u = -rate / tool.materialMap.dsDx;
        const massWeight = tool.massPerMaterialLength * tool.materialMap.dsDx * length * weight / 6;
        for (let axis = 0; axis < 3; axis++) {
            const velocity = (n0 * (data.positions[0][axis] - data.previousPositions[0][axis]) +
                n1 * (data.positions[1][axis] - data.previousPositions[1][axis])) / data.dt +
                u * (data.positions[1][axis] - data.positions[0][axis]) / length;
            const old = n0 * tool.oldMaterialVelocities[0][axis] + n1 * tool.oldMaterialVelocities[1][axis];
            incremental += .5 * massWeight * (velocity - old) ** 2; kinetic += .5 * massWeight * velocity ** 2;
        }
    }
    close(result.energy, incremental); close(result.kineticEnergy, kinetic);
    for (const tool of result.tools) for (const sample of tool.samples) {
        const original = data.tools.find(t => t.id === tool.id);
        close(sample.s, original.materialMap.sStart + original.materialMap.dsDx * length * sample.fraction);
    }
});

test('separate tool contributions add on one position field without imposing a common velocity', () => {
    const data = input(), together = assemble(data);
    const individual = data.tools.map(tool => assemble({ ...data, tools: [tool] }));
    for (const key of ['energy', 'kineticEnergy', 'oldKineticEnergy', 'mass']) close(together[key], individual.reduce((sum, r) => sum + r[key], 0));
    for (const key of ['gradient', 'kineticGradient', 'hessian', 'momentumIncrement'])
        together[key].forEach((v, i) => close(v, individual.reduce((sum, r) => sum + r[key][i], 0)));
    assert.notDeepEqual(together.tools[0].samples[0].velocity, together.tools[1].samples[0].velocity);
});

test('rigid transforms preserve energy and rotate velocity and position forces; boosts preserve the inertial objective', () => {
    const data = input(), initial = assemble(data), transformed = structuredClone(data), shift = [12, -6, 20];
    transformed.positions = data.positions.map(p => rotate(p).map((v, i) => v + shift[i]));
    transformed.previousPositions = data.previousPositions.map(p => rotate(p).map((v, i) => v + shift[i]));
    transformed.tools.forEach(tool => tool.oldMaterialVelocities = tool.oldMaterialVelocities.map(rotate));
    const result = assemble(transformed);
    close(result.energy, initial.energy); close(result.kineticEnergy, initial.kineticEnergy);
    for (let node = 0; node < 2; node++) vectorClose(result.gradient.subarray(3 * node, 3 * node + 3), rotate(initial.gradient.subarray(3 * node, 3 * node + 3)));
    const boost = [.7, -.3, .2], boosted = structuredClone(data);
    boosted.positions = data.positions.map(p => p.map((v, i) => v + data.dt * boost[i]));
    boosted.tools.forEach(tool => tool.oldMaterialVelocities = tool.oldMaterialVelocities.map(v => v.map((component, i) => component + boost[i])));
    const afterBoost = assemble(boosted);
    close(afterBoost.energy, initial.energy); vectorClose(afterBoost.gradient, initial.gradient);
    const momentum = initial.tools.reduce((sum, tool) => sum.map((v, i) => v + tool.momentum[i]), [0, 0, 0]);
    close(afterBoost.kineticEnergy - initial.kineticEnergy, dot(boost, momentum) + .5 * initial.mass * dot(boost, boost));
});

test('reparameterizing the common coordinate preserves mass, velocities, energy and the complete operator', () => {
    const data = input(), a = assemble(data), changed = structuredClone(data), scale = 2.7;
    changed.coordinates = data.coordinates.map(x => 10 + scale * x);
    changed.tools.forEach(tool => { tool.materialMap.dsDx /= scale; });
    const b = assemble(changed);
    for (const key of ['energy', 'kineticEnergy', 'mass']) close(a[key], b[key]);
    for (const key of ['gradient', 'hessian']) vectorClose(a[key], b[key]);
    a.tools.forEach((tool, i) => tool.samples.forEach((sample, j) => {
        close(sample.s, b.tools[i].samples[j].s); vectorClose(sample.velocity, b.tools[i].samples[j].velocity);
    }));
});

test('virtual work and the translation resultant conserve the material momentum increment', () => {
    const data = input(), result = assemble(data), displacement = [.2, -.1, .4, -.3, .6, -.2];
    let sampleWork = 0;
    for (const tool of result.tools) for (const sample of tool.samples) {
        const velocityVariation = [0, 1, 2].map(axis => sample.coefficients[0] * displacement[axis] + sample.coefficients[1] * displacement[3 + axis]);
        sampleWork += sample.massWeight * dot(sample.velocityIncrement, velocityVariation);
    }
    close(dot(result.gradient, displacement), sampleWork);
    for (let axis = 0; axis < 3; axis++) close(result.gradient[axis] + result.gradient[3 + axis], result.momentumIncrement[axis] / data.dt);
    let oldVelocityWork = 0;
    for (const tool of result.tools) for (const sample of tool.samples)
        oldVelocityWork += sample.massWeight * dot(sample.oldMaterialVelocity, sample.velocityIncrement);
    close(result.kineticEnergy - result.oldKineticEnergy, oldVelocityWork + result.energy);
});

test('scatter adds the full consistent inertia to Chain positions and leaves independent spin rows unchanged', () => {
    const layout = createCompositeChainLayout([['wire', 'catheter'], ['wire']]);
    const chain = createCompositeChainWorkspace(layout, { elementBackend: 'javascript' });
    chain.gradient.fill(.2); chain.hessian.fill(.1); chain.energy = 3;
    const beforeG = chain.gradient.slice(), beforeH = chain.hessian.slice(), local = assemble(input());
    scatter(local, 0, chain);
    close(chain.energy, 3 + local.energy);
    const dofs = [layout.positions[0], layout.positions[1]].flatMap(start => [start, start + 1, start + 2]);
    for (let i = 0; i < layout.dofCount; i++) {
        const localRow = dofs.indexOf(i);
        close(chain.gradient[i], beforeG[i] + (localRow < 0 ? 0 : local.gradient[localRow]));
        for (let j = Math.max(0, i - layout.band + 1); j <= i; j++) {
            const localCol = dofs.indexOf(j);
            close(chain.hessian[i * layout.band + i - j], beforeH[i * layout.band + i - j] +
                (localRow < 0 || localCol < 0 ? 0 : local.hessian[6 * localRow + localCol]));
        }
    }
    for (const offsets of layout.spins.values()) for (const dof of offsets) if (dof >= 0) close(chain.gradient[dof], .2);
    assert.throws(() => scatter(local, 1, chain), /tools must match/);
    const narrow = createCompositeChainWorkspace(createCompositeChainLayout([['wire']]), { elementBackend: 'javascript' });
    const single = input(); single.tools = [single.tools[0]];
    assert.throws(() => scatter(assemble(single), 0, narrow), /band cannot contain/);
    assert.equal(narrow.energy, 0); assert.ok(narrow.gradient.every(v => v === 0));
});

test('local workspaces reuse numeric buffers and never alter input maps, old velocities or positions', () => {
    const data = input(), snapshot = structuredClone(data), workspace = createCompositeInertiaWorkspace(2);
    const gradient = workspace.gradient, hessian = workspace.hessian, samples = workspace.tools[0].samples;
    const a = assemble(data, workspace), expected = a.energy;
    assemble(data, workspace);
    assert.equal(a, workspace); assert.equal(workspace.gradient, gradient); assert.equal(workspace.hessian, hessian);
    assert.equal(workspace.tools[0].samples, samples); close(workspace.energy, expected); assert.deepEqual(data, snapshot);
});

test('missing rates, old material velocities, masses and nonfinite operators fail explicitly', () => {
    for (const mutation of [
        data => { delete data.tools[0].materialMap.dsDt; }, data => { delete data.tools[0].oldMaterialVelocities; },
        data => { data.tools[0].massPerMaterialLength = 0; }, data => { data.tools[0].materialMap.dsDx = -1; },
        data => { data.tools[1].id = 'wire'; }, data => { data.dt = 0; }, data => { data.coordinates[1] = data.coordinates[0]; },
        data => { data.tools[0].oldMaterialVelocities[0][0] = NaN; }
    ]) { const data = input(); mutation(data); assert.throws(() => assemble(data)); }
    const tooLarge = input(); tooLarge.dt = 1e-310;
    assert.throws(() => assemble(tooLarge), /Nonfinite/);
    assert.throws(() => kinematics({ positionDt: [0, 0, 0], positionDx: [1, 0, 0], tools: [{ id: 'wire', dsDx: 1 }] }), /dsDt/);
    assert.throws(() => assemble(input(), createCompositeInertiaWorkspace(1)), /count/);
});
