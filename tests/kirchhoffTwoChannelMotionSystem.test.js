import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffSplitMotion } from '../src/physics/kirchhoffSplitMotion.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion,
    commitKirchhoffTwoChannelBiasMaterial, measureKirchhoffTwoChannelMaterial } from '../src/physics/kirchhoffTwoChannelMotion.js';

// Worker validation can read the independently owned frozen adapter in root.
// After integration the default is the ordinary repository-relative module.
const systemModule = process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT
    ? pathToFileURL(resolve(process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT, 'src/physics/kirchhoffTwoChannelSystem.js'))
    : new URL('../src/physics/kirchhoffTwoChannelSystem.js', import.meta.url);
const { solveKirchhoffTwoChannelSystem } = await import(systemModule);
const dt = 1 / 120, near = (a, b, message, tolerance = 1e-8) =>
    assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance, `${message}: ${a} vs ${b}`);

test('actual two-channel solve and motion staging close both finite-compliance material equations at the final poses', t => {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt }), gap = .01632478;
    const a = world.createRod('two-channel-motion-native-a', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    const b = world.createRod('two-channel-motion-native-b', 2, 1, { mass: 1, adaptationCompliance: dt * dt / 99 });
    a.setPinned(0, true); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const axis of [1, 2, 3]) body['inverseInertia' + axis].fill(0);
    a.restLength[0] = 10 + 100 * gap / 99; a.setNodePosition(1, 10 - 200 * gap, 0, 0);
    a.copyCurrentToPrevious(); b.copyCurrentToPrevious();
    const joint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    beginKirchhoffSplitMotion(joint, world); const state = beginKirchhoffTwoChannelMotion(joint);
    const initialP = state.physicalPose[0].x[1], compliance = a.adaptationCompliance, rest = a.restLength.slice();
    const normal = { kind: 'test-normal', strain: 0, alpha: 0, lambda: 0, lower: 0, upper: Infinity,
        gradients: [{ side: 0, dof: 6, value: 1 }] };
    const result = solveKirchhoffTwoChannelSystem(joint, dt, { basis: 'individual', tolerance: 1e-9,
        maximumPosition: 100, maximumAngle: 100, additionalRows: [normal],
        channels(native) {
            const keys = ['strain', 'alpha', 'lambda', 'weight', 'degree', 'rows', 'gradients', 'rotations', 'linearizationFrames', 'redundantAxes'];
            const before = native.material.map(m => Object.fromEntries(keys.map(key => [key, m[key].slice()])));
            const measured = measureKirchhoffTwoChannelMaterial(joint);
            native.material.forEach((m, side) => keys.forEach(key => assert.deepEqual(m[key], before[side][key], 'channel factory must restore qG scratch ' + key)));
            return native.rows.slice(0, native.count).map(row => row.kind === 'material'
                ? { physical: 'pose', bias: { channel: 'bias-motion', strain: measured.biasStrain[row.side][row.local],
                    alpha: row.alpha, lambda: measured.biasLambda[row.side][row.local], lower: -Infinity, upper: Infinity } }
                : { physical: 'physical-motion', bias: { channel: 'pose', strain: a.x[1] - 10, alpha: 0, lambda: 0, lower: 0, upper: Infinity } });
        } });
    assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics)); assert.equal(result.scale, 1);
    const expectedPhysical = 99 * (a.restLength[0] - 10), expectedBias = 10 - a.x[1] - expectedPhysical;
    applyKirchhoffTwoChannelPhysicalMotion(joint, result);
    near(a.x[1], initialP, 'helper did not apply qG');
    applyKirchhoffCoupledCorrection(joint, result); commitKirchhoffTwoChannelBiasMaterial(joint, result);
    const measured = measureKirchhoffTwoChannelMaterial(joint);
    near(a.x[1], 10, 'total final geometric point');
    near(state.physicalPose[0].x[1], initialP + expectedPhysical, 'physical pose has only physical response');
    near(result.physical[0].correction[6], expectedPhysical, 'native physical response matches the analytic impulse');
    near(joint._splitMotion.bodies[0].velocityX[1], result.physical[0].correction[6] / dt, 'actual physical response is published once; bias contributes no velocity');
    assert.equal(state.materialLambda[0][3], result.bias[0].lambda[3], 'native bias multiplier is committed once');
    near(measured.alpha[0][3] * state.materialLambda[0][3], -expectedBias, 'analytic bias reaction in displacement-equation units');
    near(measured.physicalResidual.adaptationMm, 0, 'fresh physical material closure');
    near(measured.biasResidual.adaptationMm, 0, 'fresh finite-compliance bias material closure');
    near(measured.alpha[0][3], 1 / 99, 'alpha was not hardened');
    assert.equal(a.adaptationCompliance, compliance); assert.deepEqual(a.restLength, rest);
    t.diagnostic(JSON.stringify({ physicalPoseX: state.physicalPose[0].x[1], geometryX: a.x[1],
        physicalVelocityX: joint._splitMotion.bodies[0].velocityX[1], betaMaterial: state.materialLambda[0][3],
        physicalResidual: measured.physicalResidual, biasResidual: measured.biasResidual, alpha: measured.alpha[0][3] }));
});
