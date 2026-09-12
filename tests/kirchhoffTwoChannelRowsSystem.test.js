import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffCoupledBoundaryStep, applyKirchhoffCoupledBoundaryMultipliers } from '../src/physics/kirchhoffCoupledBoundaryRows.js';
import { beginKirchhoffSplitMotion, kirchhoffSplitRowMotion } from '../src/physics/kirchhoffSplitMotion.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion,
    commitKirchhoffTwoChannelBiasMaterial, measureKirchhoffTwoChannelMaterial } from '../src/physics/kirchhoffTwoChannelMotion.js';
import { beginKirchhoffTwoChannelRows, prepareKirchhoffTwoChannelRows, commitKirchhoffTwoChannelRows,
    measureKirchhoffTwoChannelRows } from '../src/physics/kirchhoffTwoChannelRows.js';

// The worker reads the independently owned System without editing/copying it.
// After integration the ordinary repository-relative module is used.
const systemModule = process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT
    ? pathToFileURL(resolve(process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT, 'src/physics/kirchhoffTwoChannelSystem.js'))
    : new URL('../src/physics/kirchhoffTwoChannelSystem.js', import.meta.url);
const { solveKirchhoffTwoChannelSystem } = await import(systemModule);
const dt = 1 / 120, near = (a, b, reason, tol = 2e-8) =>
    assert.ok(Number.isFinite(a) && Math.abs(a - b) < tol, `${reason}: ${a} vs ${b}`);

test('native wall/material solve closes both channels and retains independent beta on the next outer pass', t => {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt }), gap = .01632478;
    const a = world.createRod('two-channel-rows-native-a', 2, 1, { mass: 1, adaptationCompliance: dt ** 2 / 99 });
    const b = world.createRod('two-channel-rows-native-b', 2, 1, { mass: 1, adaptationCompliance: dt ** 2 / 99 });
    a.setPinned(0, true); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const axis of [1, 2, 3]) body['inverseInertia' + axis].fill(0);
    a.restLength[0] = 10 + 100 * gap / 99; a.setNodePosition(1, 10 - 200 * gap, 0, 0);
    a.copyCurrentToPrevious(); b.copyCurrentToPrevious();
    const joint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    beginKirchhoffCoupledBoundaryStep(joint); beginKirchhoffSplitMotion(joint, world);
    beginKirchhoffTwoChannelMotion(joint); beginKirchhoffTwoChannelRows(joint, world);
    const initialP = a.x[1], beta = joint._splitMotion.biasBank.bodies[0].wallLambda;
    // An existing capsule endpoint wall constraint in one free dimension.
    const wall = { kind: 'wall', side: 0, node: 0, owner: a, strain: 0, _splitActualStrain: a.x[1] - 10,
        alpha: 0, lambda: 0, lower: 0, upper: Infinity, gradients: [{ side: 0, dof: 6, value: 1 }] };
    joint._coupledBoundaries.rows = [wall];
    let first, betaAfterFirst, velocityAfterFirst;
    for (let pass = 0; pass < 2; pass++) {
        wall.strain = kirchhoffSplitRowMotion(joint, wall.gradients); wall.lambda = a.wallLambda[0]; wall._splitActualStrain = a.x[1] - 10;
        const prepared = prepareKirchhoffTwoChannelRows(joint, [wall]); assert.equal(prepared.ready, true);
        const result = solveKirchhoffTwoChannelSystem(joint, dt, { basis: 'individual', tolerance: 1e-9,
            maximumPosition: 100, maximumAngle: 100, additionalRows: [wall], channels: prepared.channels, includeSystem: true });
        assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics)); assert.equal(result.scale, 1);
        const descriptor = result.system.descriptors[result.system.native.additionalOffset];
        assert.equal(descriptor.bias.lambda, beta[0]); assert.equal(descriptor.bias.alpha, wall.alpha);
        if (pass) {
            near(result.physical[0].correction[6], 0, 'closed physical direction');
            near(result.bias[0].correction[6], 0, 'closed bias direction');
            assert.equal(descriptor.bias.lambda, betaAfterFirst, 'persistent normal beta supplied to next solve');
        }
        applyKirchhoffTwoChannelPhysicalMotion(joint, result); applyKirchhoffCoupledCorrection(joint, result);
        applyKirchhoffCoupledBoundaryMultipliers(joint, result.additionalIncrement, result.scale);
        commitKirchhoffTwoChannelBiasMaterial(joint, result); commitKirchhoffTwoChannelRows(joint, result);
        wall._splitActualStrain = a.x[1] - 10;
        const material = measureKirchhoffTwoChannelMaterial(joint), rows = measureKirchhoffTwoChannelRows(joint, [wall]);
        near(material.physicalResidual.adaptationMm, 0, 'fresh physical material closure');
        near(material.biasResidual.adaptationMm, 0, 'fresh bias material closure');
        near(rows.normalResidualMm, 0, 'fresh geometric normal closure'); assert.equal(rows.supported, true);
        assert.ok(beta[0] > 0); near(a.wallLambda[0], 0, 'geometric reaction has no physical normal load');
        if (!pass) { first = result; betaAfterFirst = beta[0]; velocityAfterFirst = joint._splitMotion.bodies[0].velocityX[1]; }
    }
    const p = joint._splitMotion.twoChannel.physicalPose[0].x[1], v = joint._splitMotion.bodies[0].velocityX[1];
    near(a.x[1], 10, 'final geometry'); near(p, initialP + first.physical[0].correction[6], 'qP excludes bias');
    near(v, first.physical[0].correction[6] / dt, 'physical impulse only once'); near(v, velocityAfterFirst, 'next outer pass adds no repeated impulse');
    assert.ok(Math.abs(first.bias[0].correction[6]) > 1);
    t.diagnostic(JSON.stringify({ geometryX: a.x[1], physicalPoseX: p, physicalVelocityX: v, betaNormal: beta[0], physicalNormal: a.wallLambda[0] }));
});
