import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { KirchhoffContactManifold } from '../src/physics/kirchhoffContactManifold.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';
import { buildKirchhoffExternalFrictionRows } from '../src/physics/kirchhoffExternalFrictionRows.js';

function fixture(surfaceFrictionEnabled) {
    const calls = [];
    const world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120, coupledSystem: {
        physicalTrialState: true,
        solve(c, dt, options) {
            calls.push({ groups: options.groups.length, normals: c.kirchhoffContacts.length });
            return solveKirchhoffCoupledSystem(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true });
        }, apply: applyKirchhoffCoupledCorrection
    } });
    const profile = { radius: .4445, linearDamping: 1, angularDamping: 1,
        projectionVelocityRetention: 1, maxBendAngle: 45, foldLimitStrength: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 8, 10, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', 8, 10, { ...profile, mass: 3 });
    for (let i = 0; i < 8; i++) {
        inner.setNodePosition(i, i * 10, .2, 0);
        outer.setNodePosition(i, i * 10, 0, 0);
        inner.velocityX[i] = 6;
        inner.angularVelocityX[i] = 2;
    }
    const c = world.addContainment(inner, outer, { surfaceFrictionEnabled, innerRadius: .485,
        axialFriction: surfaceFrictionEnabled ? 0 : .9, torsionalFriction: surfaceFrictionEnabled ? 0 : .9, portalFilletRadius: 0,
        coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    return { world, inner, outer, c, calls };
}

test('frictionless normal contact never reads or transports tangent history while retaining normal loads', () => {
    const manifold = new KirchhoffContactManifold({ normalOnly: true });
    const contact = manifold.upsertContact({ innerMaterialSegmentId: 0, outerMaterialSegmentId: 0,
        normal: [0, 2, 0], tangentU: [NaN, NaN, NaN], frictionCoefficient: .9 });
    assert.equal(contact.frictionCoefficient, 0);
    assert.equal(manifold.setNormalLambda(contact, 3), 3);
    const tangent = contact.tangentLambda;
    Object.defineProperty(contact, 'tangentLambda', { configurable: true, get() { throw Error('tangent history read'); } });
    manifold.refreshKnownContact(contact, { normal: [2, 0, 0], innerSegmentIndex: 1, tangentU: [NaN, 0, 0] });
    assert.deepEqual([...contact.normal], [1, 0, 0]);
    assert.equal(contact.innerSegmentIndex, 1);
    assert.equal(contact.normalLambda, 3);
    assert.equal(manifold.accumulateKnownNormalLambda(contact, -1), -1);
    Object.defineProperty(contact, 'tangentLambda', { configurable: true, writable: true, value: tangent });
    assert.deepEqual([...tangent], [0, 0]);
    assert.throws(() => manifold.refreshKnownContact(contact, { normal: [NaN, 0, 0] }), /finite/);
});

test('normal-only coupled runtime omits friction rows, residuals, repair and history scratch', () => {
    const f = fixture(false);
    for (let i = 0; i < 3; i++) f.world.stepFixed();
    assert.ok(f.calls.length > 0 && f.calls.some(x => x.normals > 0));
    assert.ok(f.calls.every(x => x.groups === 0));
    for (const key of ['_jointFrictionBatch', '_jointFrictionResidual', '_jointFrictionMerit', '_jointConeRepair'])
        assert.equal(f.c[key], undefined, key);
    assert.equal(f.world.lastJointCosts.frictionBatchReuseCount, 0);
    assert.equal(f.c._jointMeritTerms.lumenFriction, 0);
    assert.equal(f.world.lastJointNonlinearFailure ?? null, null);
    for (const contact of f.c.manifold.contacts()) {
        assert.deepEqual([...contact.tangentLambda], [0, 0]);
        assert.equal(contact.twistLambda, 0);
    }
    assert.ok(f.inner.y[3] < .2, 'inner rod responds to normal containment');
    assert.ok(f.outer.y[3] > 0, 'outer rod receives the reciprocal normal reaction');
});

test('free axial and rotational motion agrees with the zero-friction full-row reference', () => {
    const fast = fixture(false), reference = fixture(true);
    for (let i = 0; i < 3; i++) { fast.world.stepFixed(); reference.world.stepFixed(); }
    for (const side of ['inner', 'outer']) for (const key of ['x', 'y', 'z',
        'orientationX', 'orientationY', 'orientationZ', 'orientationW', 'velocityX', 'velocityY', 'velocityZ']) {
        const a = fast[side][key], b = reference[side][key];
        for (let i = 0; i < a.length; i++) assert.ok(Math.abs(a[i] - b[i]) < 1e-5, `${side}.${key}[${i}]: ${a[i]} / ${b[i]}`);
    }
    assert.ok(reference.calls.some(x => x.groups > 0), 'reference must actually assemble zero-friction cone rows');
    assert.ok(fast.inner.x[3] > 30, 'wire can advance axially while loaded');
    assert.ok(Math.abs(fast.inner.orientationX[3]) > 0, 'wire can rotate independently');
});

test('normal-only contact loads survive trial rollback without resurrecting friction', () => {
    const f = fixture(false); f.world.stepFixed();
    const snapshot = captureKirchhoffCoupledTrialState(f.c, { world: f.world, frozenFrictionBatches: true,
        physicalStateOnly: true, reusePropertyLayout: true });
    const contact = [...f.c.manifold.contacts()][0], normal = contact.normal.slice(), load = contact.normalLambda;
    f.c.manifold.setNormalLambda(contact, load + 1);
    f.c.manifold.refreshKnownContact(contact, { normal: [0, 0, 1] });
    restoreKirchhoffCoupledTrialState(snapshot);
    assert.deepEqual(contact.normal, normal); assert.equal(contact.normalLambda, load);
    assert.equal(f.c.manifold.normalOnly, true);
    assert.deepEqual([...contact.tangentLambda], [0, 0]);
});

test('zero-friction external tool contacts allocate no tangential contact state', () => {
    const f = fixture(false);
    const batch = buildKirchhoffExternalFrictionRows(f.c, [{ kind: 'tool', owner: { friction: 0 } }], 1 / 120);
    assert.equal(batch.rows.length, 0); assert.equal(batch.groups.length, 0);
    assert.equal(f.c._coupledExternalFriction, undefined);
});

