import assert from 'node:assert/strict';
import test from 'node:test';
import { createCoupledRuntimeFixture } from './helpers/coupledRuntimeFixture.js';
import { createCoupledSolverSelection } from '../src/physics/coupledSolverSelection.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { configureKirchhoffSplitBias } from '../src/physics/kirchhoffSplitMotion.js';
import { solveKirchhoffTwoChannelSystem } from '../src/physics/kirchhoffTwoChannelSystem.js';

test('application actuation passes both former friction stalls with unchanged final gates and one history commit', () => {
    const selection = createCoupledSolverSelection('joint-two-channel', {
        solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection, solveTwoChannel: solveKirchhoffTwoChannelSystem
    });
    const f = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: selection.jointMotionMode });
    configureKirchhoffSplitBias(f.containment, { materialMode: selection.biasMaterialMode });
    const original = { dt: f.world.fixedDt, mm: f.world.coupledContainmentTolerance,
        length: f.world.coupledLengthTolerance, rad: f.world.coupledAngularToleranceRad };
    let jointSteps = 0, refinements = 0, coneRepairs = 0;
    try {
        for (let i = 0; i < 33; i++) assert.notEqual(f.step({ guidewireAdvance: 1 }).accepted, false);
        // This interval includes the prior fresh-cone stalls at 7.3667 and 9.1 mm.
        // Longer insertion remains covered by the separate ongoing runtime probe.
        for (let i = 0; i < 23; i++) {
            const before = f.world.stepCount, step = f.step({ catheterAdvance: 1 });
            assert.notEqual(step.accepted, false, JSON.stringify(f.world.getStats().jointMotion));
            assert.equal(f.world.stepCount, before + 1);
            const d = f.world.getStats().jointMotion;
            if (!d?.twoChannel) continue;
            jointSteps++;
            assert.equal(d.certified, true); assert.equal(d.historyCommits, 1);
            assert.equal(d.biasPasses, 0); assert.ok(d.physicalConeViolation <= 1e-9);
            assert.ok(d.normalCertificate.maximumGeometryViolationMm <= original.mm);
            for (const r of d.linearRefinements ?? []) {
                refinements++;
                assert.ok(r.to > 0 && r.to < r.from && r.to >= r.structuralResidualFloor);
                assert.ok(r.from <= original.mm * .2);
            }
            for (const r of d.coneFilterAcceptances ?? []) {
                coneRepairs++;
                assert.ok(r.cone < r.previousCone);
                assert.ok(r.cone > 1e-9, 'internal repair still needs another iteration before publication');
                assert.ok(r.merit <= 1, 'other errors remain below their final gates');
            }
        }
        assert.equal(jointSteps, 14); assert.ok(coneRepairs > 0, 'exercise nonlinear friction recovery');
        assert.ok(refinements + coneRepairs > 0);
        assert.equal(f.world.stepCount, 56); assert.equal(f.snapshot().executedSteps, 56);
        assert.equal(f.world.droppedTime, 0);
        assert.deepEqual({ dt: f.world.fixedDt, mm: f.world.coupledContainmentTolerance,
            length: f.world.coupledLengthTolerance, rad: f.world.coupledAngularToleranceRad }, original);
        assert.equal(original.dt, 1 / 120);
        const report = selection.getReport(f.world);
        assert.equal(report.id, 'joint-two-channel'); assert.equal(report.installed, true);
        assert.ok(report.twoChannelResults >= jointSteps); assert.equal(report.lastResult.channels, 2);
    } finally { f.dispose(); }
});

test('unfinished cone repair rejects and rolls back the entire timestep despite successful internal iterations', () => {
    const selection = createCoupledSolverSelection('joint-two-channel', {
        solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection, solveTwoChannel: solveKirchhoffTwoChannelSystem
    });
    const f = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: selection.jointMotionMode });
    configureKirchhoffSplitBias(f.containment, { materialMode: selection.biasMaterialMode });
    try {
        for (let i = 0; i < 33; i++) assert.notEqual(f.step({ guidewireAdvance: 1 }).accepted, false);
        for (let i = 0; i < 18; i++) assert.notEqual(f.step({ catheterAdvance: 1 }).accepted, false);
        // This real feed needs thirteen iterations. Stop after the first
        // successful cone-only repair, while its cone still exceeds 1e-9.
        f.world.coupledClosureMaxPasses = 3;
        const stepFixed = f.world.stepFixed.bind(f.world);
        let prepared;
        f.world.stepFixed = () => { prepared = f.snapshot(); return stepFixed(); };
        const step = f.step({ catheterAdvance: 1 });
        const d = f.world.getStats().jointMotion;
        assert.equal(step.accepted, false);
        assert.equal(d.certified, false); assert.equal(d.historyCommits, 0);
        assert.ok(d.coneFilterAcceptances.length > 0, 'exercise accepted inner repair before rejection');
        assert.ok(d.physicalConeViolation > 1e-9, 'original final friction gate still rejects the step');
        assert.equal(f.world.stepCount, 51); assert.equal(f.snapshot().executedSteps, 51);
        assert.deepEqual(f.snapshot(), prepared, 'restore the prepared input and both physical states');
        f.world.coupledClosureMaxPasses = 32;
        const retry = stepFixed();
        assert.equal(retry.accepted, true, JSON.stringify(retry));
        assert.equal(f.world.stepCount, 52);
        assert.equal(retry.diagnostics.historyCommits, 1);
        assert.ok(retry.diagnostics.physicalConeViolation <= 1e-9);
        assert.equal(f.catheter.progress, prepared.catheterMm, 'retry does not repeat catheter feed');
    } finally { f.dispose(); }
});
