import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeInertiaCache } from '../src/physics/kirchhoffCompositeInertiaCache.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeInertiaWorkspace, assembleCompositeTranslationalInertia as assemble,
    scatterCompositeTranslationalInertia as scatter } from '../src/physics/kirchhoffCompositeKinematics.js';

const close = (a, b, tolerance = 1e-11) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const vectorClose = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tolerance)); };
const bytes = a => Buffer.from(a.buffer, a.byteOffset, a.byteLength);
function fixture() {
    const ids = [['wire'], ['wire', 'catheter'], ['catheter']];
    const layout = createCompositeChainLayout(ids), coordinates = [2, 5, 7.1, 11];
    const previousPositions = [[.16, -.08, .38], [3.25, .67, -.15], [5.5, .8, -.3], [8.7, 1.3, .2]];
    const positions = previousPositions.map((p, i) => p.map((v, j) => v + .04 * Math.cos(i + j)));
    const inertiaEdges = ids.map((edgeIds, edge) => ({ tools: edgeIds.map((id, i) => ({ id,
        materialMap: { sStart: 170 - 12 * edge + i, dsDx: id === 'wire' ? 1.3 : .8, dsDt: id === 'wire' ? [-.7, -2.1] : [.4, 1.2] },
        massPerMaterialLength: id === 'wire' ? 2.4 : 4.1,
        oldMaterialVelocities: id === 'wire' ? [[.3, -.2, .1], [.8, .4, -.3]] : [[-.7, .1, .2], [-.4, -.1, .6]]
    })) }));
    return { layout, coordinates, previousPositions, positions, inertiaEdges, dt: .13 };
}
function empty(layout, seeded = false) {
    return { layout, energy: seeded ? 2.3 : 0,
        gradient: new Float64Array(layout.dofCount).fill(seeded ? .17 : 0),
        hessian: new Float64Array(layout.dofCount * layout.band).fill(seeded ? .013 : 0) };
}
function original(data, positions = data.positions, seeded = false) {
    const chain = empty(data.layout, seeded), kineticGradient = new Float64Array(data.layout.dofCount), velocities = [], increments = [];
    let energy = 0, kineticEnergy = 0, mass = 0, oldKineticEnergy = 0;
    for (let edge = 0; edge < data.inertiaEdges.length; edge++) {
        const local = assemble({ coordinates: data.coordinates.slice(edge, edge + 2), positions: positions.slice(edge, edge + 2),
            previousPositions: data.previousPositions.slice(edge, edge + 2), dt: data.dt, tools: data.inertiaEdges[edge].tools });
        scatter(local, edge, chain);
        for (let row = 0; row < 6; row++) kineticGradient[data.layout.positions[edge + Math.floor(row / 3)] + row % 3] += local.kineticGradient[row];
        for (const tool of local.tools) for (const sample of tool.samples) { velocities.push(...sample.velocity); increments.push(...sample.velocityIncrement); }
        energy += local.energy; kineticEnergy += local.kineticEnergy; mass += local.mass; oldKineticEnergy += local.oldKineticEnergy;
    }
    return { chain, energy, kineticEnergy, kineticGradient, velocities, increments, mass, oldKineticEnergy };
}
function evaluated(cache, data, positions = data.positions, seeded = false) {
    const chain = empty(data.layout, seeded), response = cache.append(positions, chain);
    return { chain, ...structuredClone({ energy: response.energy, kineticEnergy: response.kineticEnergy,
        gradient: response.gradient, kineticGradient: response.kineticGradient, velocities: response.velocities,
        increments: response.velocityIncrements, mass: response.mass, oldKineticEnergy: response.oldKineticEnergy }) };
}
function compare(actual, expected) {
    for (const key of ['energy', 'kineticEnergy', 'mass', 'oldKineticEnergy']) close(actual[key], expected[key]);
    close(actual.chain.energy, expected.chain.energy);
    for (const key of ['kineticGradient', 'velocities', 'increments']) vectorClose(actual[key], expected[key]);
    vectorClose(actual.chain.gradient, expected.chain.gradient);
    assert.deepEqual(bytes(actual.chain.hessian), bytes(expected.chain.hessian), 'the complete original band must be byte identical');
}

test('compiled affine operator preserves original 3D mixed ownership E/g/full H and kinetic response on every trial', () => {
    const data = fixture(), cache = createCompositeInertiaCache(data), inputBefore = structuredClone(data);
    for (let trial = 0; trial < 5; trial++) {
        const positions = data.positions.map((p, node) => p.map((v, axis) => v + .01 * trial * Math.sin(node + axis)));
        compare(evaluated(cache, data, positions, true), original(data, positions, true));
    }
    assert.deepEqual(data, inputBefore);
    assert.deepEqual(cache.edgeTools, [['wire'], ['wire', 'catheter'], ['catheter']]);
    assert.equal(cache.sampleCount, 8); assert.equal(cache.includesAngularInertia, false);
    assert.equal(cache.energyKind, 'material-velocity-increment');
    for (const dofs of data.layout.spins.values()) for (const dof of dofs) if (dof >= 0) {
        close(cache.gradient[dof], 0); close(cache.kineticGradient[dof], 0);
    }
});

test('independent finite differences validate exact energy/kinetic gradients and every full Hessian row', () => {
    const data = fixture(), cache = createCompositeInertiaCache(data), base = evaluated(cache, data), h = 2e-6;
    for (let node = 0; node < data.positions.length; node++) for (let axis = 0; axis < 3; axis++) {
        const plus = structuredClone(data.positions), minus = structuredClone(data.positions), dof = data.layout.positions[node] + axis;
        plus[node][axis] += h; minus[node][axis] -= h;
        const a = evaluated(cache, data, plus), b = evaluated(cache, data, minus);
        close(base.gradient[dof], (a.energy - b.energy) / (2 * h), 2e-8);
        close(base.kineticGradient[dof], (a.kineticEnergy - b.kineticEnergy) / (2 * h), 2e-8);
        for (let row = 0; row < data.layout.dofCount; row++) {
            const separation = Math.abs(row - dof), value = separation < data.layout.band ? base.chain.hessian[Math.max(row, dof) * data.layout.band + separation] : 0;
            close(value, (a.gradient[row] - b.gradient[row]) / (2 * h), 2e-8);
            close(value, (a.kineticGradient[row] - b.kineticGradient[row]) / (2 * h), 2e-8);
        }
    }
});

test('rigid frame rotation/translation and a material-velocity boost preserve the inertial objective', () => {
    const data = fixture(), base = evaluated(createCompositeInertiaCache(data), data), transformed = structuredClone(data);
    const rotate = v => [-v[1], v[2], -v[0]], shift = [12, -6, 20];
    transformed.layout = data.layout;
    for (const key of ['positions', 'previousPositions']) transformed[key] = data[key].map(p => rotate(p).map((v, i) => v + shift[i]));
    transformed.inertiaEdges.forEach(edge => edge.tools.forEach(tool => tool.oldMaterialVelocities = tool.oldMaterialVelocities.map(rotate)));
    const rotated = evaluated(createCompositeInertiaCache(transformed), transformed);
    compare(rotated, original(transformed)); close(base.energy, rotated.energy); close(base.kineticEnergy, rotated.kineticEnergy);
    for (const dof of data.layout.positions) vectorClose(rotated.gradient.subarray(dof, dof + 3), rotate(base.gradient.subarray(dof, dof + 3)));
    const boost = [.7, -.3, .2], boosted = structuredClone(data); boosted.layout = data.layout;
    boosted.positions = data.positions.map(p => p.map((v, i) => v + data.dt * boost[i]));
    boosted.inertiaEdges.forEach(edge => edge.tools.forEach(tool => tool.oldMaterialVelocities = tool.oldMaterialVelocities.map(v => v.map((component, i) => component + boost[i]))));
    const afterBoost = evaluated(createCompositeInertiaCache(boosted), boosted);
    compare(afterBoost, original(boosted)); close(afterBoost.energy, base.energy); vectorClose(afterBoost.gradient, base.gradient);
});

function steadyFixture() {
    const layout = createCompositeChainLayout([['wire', 'catheter'], ['wire', 'catheter']]);
    const positions = [[0, 0, 0], [4, 1, 0], [8, 2, 0]], coordinates = [0, 2, 4];
    return { layout, positions, previousPositions: structuredClone(positions), coordinates, dt: .1,
        inertiaEdges: [0, 1].map(() => ({ tools: [
            { id: 'wire', massPerMaterialLength: 2, materialMap: { sStart: 10, dsDx: 2, dsDt: -6 }, oldMaterialVelocities: [[6, 1.5, 0], [6, 1.5, 0]] },
            { id: 'catheter', massPerMaterialLength: 3, materialMap: { sStart: 20, dsDx: .5, dsDt: 1 }, oldMaterialVelocities: [[-4, -1, 0], [-4, -1, 0]] }
        ] })) };
}

test('steady opposite feed has zero objective and different nonzero physical velocities for the two materials', () => {
    const data = steadyFixture(), cache = createCompositeInertiaCache(data), result = evaluated(cache, data);
    compare(result, original(data)); close(result.energy, 0); vectorClose(result.gradient, new Float64Array(data.layout.dofCount));
    assert.ok(result.kineticEnergy > 100); assert.notDeepEqual(result.velocities.slice(0, 3), result.velocities.slice(6, 9));
    const zero = steadyFixture(); zero.inertiaEdges.forEach(edge => edge.tools.forEach(tool => {
        tool.materialMap.dsDt = 0; tool.oldMaterialVelocities = [[0, 0, 0], [0, 0, 0]];
    }));
    const rest = evaluated(createCompositeInertiaCache(zero), zero);
    assert.equal(rest.energy, 0); assert.equal(rest.kineticEnergy, 0); compare(rest, original(zero));
});

test('full convection keeps negative off-diagonal entries and exact transport null modes without lumping or floors', () => {
    const data = steadyFixture(); data.dt = .2;
    data.inertiaEdges.forEach(edge => edge.tools.forEach(tool => { tool.materialMap.dsDt = -10 * tool.materialMap.dsDx; }));
    const result = evaluated(createCompositeInertiaCache(data), data);
    compare(result, original(data));
    const dof0 = data.layout.positions[0], dof1 = data.layout.positions[1];
    assert.ok(result.chain.hessian[dof1 * data.layout.band + dof1 - dof0] < 0);
    // Coefficients a0 == a1 give a true anti-symmetric endpoint null mode.
    const cancelling = steadyFixture(); cancelling.dt = .2;
    cancelling.inertiaEdges.forEach(edge => edge.tools.forEach(tool => {
        tool.materialMap.dsDt = [-5 * tool.materialMap.dsDx, 5 * tool.materialMap.dsDx];
        tool.oldMaterialVelocities = [[10, 2.5, 0], [-10, -2.5, 0]];
    }));
    const cache = createCompositeInertiaCache(cancelling);
    cancelling.positions[0][0] -= .1; cancelling.positions[1][0] += .1; cancelling.positions[2][0] -= .1;
    const nullResult = evaluated(cache, cancelling); compare(nullResult, original(cancelling)); close(nullResult.energy, 0);
});

test('small increments retain positive energy on a very large material transport velocity', () => {
    const data = steadyFixture(), speed = 1e10;
    data.inertiaEdges.forEach(edge => edge.tools.forEach(tool => {
        tool.materialMap.dsDt = -speed * tool.materialMap.dsDx;
        tool.oldMaterialVelocities = [[2 * speed, .5 * speed, 0], [2 * speed, .5 * speed, 0]];
    }));
    // Rigid z motion avoids losing the position perturbation in large axial coordinates.
    data.positions.forEach(p => p[2] += 1e-7);
    const result = evaluated(createCompositeInertiaCache(data), data);
    compare(result, original(data)); assert.ok(result.energy > 1e-13 && result.energy < 1e-9);
    assert.ok(result.kineticEnergy > 1e20);
    assert.equal(result.kineticEnergy - result.oldKineticEnergy, 0, 'subtracting large kinetic energies would destroy the objective');
});

test('prepared maps, mass, previous positions and material history are frozen; outputs reuse buffers', () => {
    const data = fixture(), cache = createCompositeInertiaCache(data), expected = evaluated(cache, data), inputPositions = structuredClone(data.positions);
    const buffers = [cache.gradient, cache.kineticGradient, cache.velocities, cache.velocityIncrements, cache.toolEnergy, cache.toolKineticEnergy];
    data.coordinates.fill(NaN); data.previousPositions.forEach(p => p.fill(NaN));
    data.inertiaEdges.forEach(edge => edge.tools.forEach(tool => {
        tool.materialMap.dsDx = NaN; tool.materialMap.dsDt = NaN; tool.massPerMaterialLength = NaN; tool.oldMaterialVelocities.forEach(v => v.fill(NaN));
    }));
    const response = cache.append(inputPositions, empty(data.layout));
    assert.equal(response, cache); buffers.forEach((buffer, i) => assert.equal(buffer, [cache.gradient, cache.kineticGradient, cache.velocities, cache.velocityIncrements, cache.toolEnergy, cache.toolKineticEnergy][i]));
    close(cache.energy, expected.energy); vectorClose(cache.gradient, expected.gradient);
    cache.gradient.fill(NaN); cache.velocities.fill(NaN); cache.toolEnergy.fill(NaN);
    const afterPoison = evaluated(cache, data, inputPositions);
    close(afterPoison.energy, expected.energy); vectorClose(afterPoison.gradient, expected.gradient); vectorClose(afterPoison.velocities, expected.velocities);
});

test('compilation and evaluation retain ownership, finite-input, and complete-band guards', () => {
    for (const change of [d => d.inertiaEdges[0].tools[0].materialMap.dsDx = 0,
        d => d.inertiaEdges[0].tools[0].massPerMaterialLength = NaN,
        d => d.inertiaEdges[0].tools[0].oldMaterialVelocities = null,
        d => d.inertiaEdges[0].tools[0].id = 'catheter', d => d.coordinates[1] = d.coordinates[0],
        d => d.previousPositions[1][0] = Infinity, d => d.layout.band = 1]) {
        const data = fixture(); change(data); assert.throws(() => createCompositeInertiaCache(data));
    }
    const data = fixture(), cache = createCompositeInertiaCache(data);
    assert.throws(() => cache.append(data.positions, empty(structuredClone(data.layout))), /original fixed Chain/);
    const invalid = structuredClone(data.positions); invalid[1][0] = NaN;
    assert.throws(() => cache.append(invalid, empty(data.layout)), /Nonfinite/);
    const chain = empty(data.layout); chain.hessian[0] = Infinity;
    assert.throws(() => cache.append(data.positions, chain), /Nonfinite/);
});
