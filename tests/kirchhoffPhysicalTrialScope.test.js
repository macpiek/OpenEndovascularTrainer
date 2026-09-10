import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { captureKirchhoffCoupledTrialState as capture, restoreKirchhoffCoupledTrialState as restore } from '../src/physics/kirchhoffCoupledTrialState.js';

// Compare all retained bytes/properties by identity, not recursive equality:
// aliased child objects are checked in their own records, and cycles are legal.
function assertCompleteState(snapshot, derived) {
    for (const record of snapshot.records) {
        if (derived.has(record.object)) continue;
        if (record.kind === 'bytes') assert.deepEqual(record.view, record.copy);
        else if (record.kind === 'map') {
            assert.equal(record.object.size, record.entries.length);
            for (const [key, value] of record.entries) assert.equal(record.object.get(key), value);
        } else if (record.kind === 'set') {
            assert.equal(record.object.size, record.entries.length);
            for (const value of record.entries) assert.ok(record.object.has(value));
        } else {
            const names = Object.getOwnPropertyNames(record.object)
                .filter(key => !record.filter || record.filter(key, record.object[key]));
            assert.deepEqual(names.slice().sort(), record.keys.slice().sort());
            for (const key of record.keys) {
                const actual = Object.getOwnPropertyDescriptor(record.object, key), expected = record.descriptors[key];
                for (const attribute of Object.keys(expected)) assert.ok(Object.is(actual[attribute], expected[attribute]),
                    `Changed ${key}.${attribute} on ${record.object.id ?? record.object.kind ?? 'state'}`);
            }
        }
    }
}

test('physical rollback agrees with a complete snapshot after real coupled apply and nonlinear contact measurement', () => {
    let world, base, audited = 0, changedBytes = 0;
    const coupledSystem = {
        physicalTrialState: true,
        solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt, {
            ...options, activeCondensation: true, simultaneousCoulomb: true
        }),
        apply(c, result) {
            base = capture(c, { world, frozenFrictionBatches: true });
            applyKirchhoffCoupledCorrection(c, result);
        }
    };
    world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120, coupledSystem });
    const profile = { radius: .4445, linearDamping: 1, angularDamping: 1,
        projectionVelocityRetention: 1, maxBendAngle: 45, foldLimitStrength: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 8, 10, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', 8, 10, { ...profile, mass: 3 });
    for (let i = 0; i < 8; i++) {
        inner.setNodePosition(i, i * 10, .2, 0);
        outer.setNodePosition(i, i * 10, 0, 0);
        inner.velocityX[i] = 6;
    }
    inner.angularVelocityX.fill(2);
    world.addContainment(inner, outer, { innerRadius: .485, axialFriction: .2, torsionalFriction: .2,
        portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    world.debugJointTrial = (c, _measurement, pass) => {
        if (!pass) return;
        const trial = c._jointTrialState;
        assert.ok(trial.foldTrial?.state);
        for (const record of base.records) if (record.kind === 'bytes' &&
            record.view.some((byte, i) => byte !== record.copy[i])) changedBytes++;
        const candidate = capture(c, { world, frozenFrictionBatches: true });
        restore(trial);
        const fold = c._coupledFoldRows;
        const derived = new Set([fold.geometry, fold.measurement, ...fold.storage.map(storage => storage.frames)]);
        assertCompleteState(base, derived);
        // Continue the real solver with exactly the candidate it just measured.
        restore(candidate);
        audited++;
    };
    for (let step = 0; step < 3; step++) world.stepFixed();
    assert.ok(audited > 0, 'Must exercise repeated real apply/measure trials');
    assert.ok(changedBytes > 0, 'The trials must change actual mechanics');
});
