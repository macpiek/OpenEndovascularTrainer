import assert from 'node:assert/strict';
import test from 'node:test';
import { fixture, DT } from './fixtures/splitMotionAnalyticWorld.js';

const near = (a, b, message) => assert.ok(Number.isFinite(a) && Math.abs(a - b) < 5e-4, `${message}: ${a} vs ${b}`);
function wallFixture(muStatic = .6, muKinetic = .2) {
    const f = fixture({ wall: true, y: -.5 });
    f.wire.wallStaticFriction = muStatic; f.wire.wallKineticFriction = muKinetic;
    // A prescribed orientation makes the translational oracle independent of
    // the angular response. Separate free-angular wall tests verify the wrench.
    for (const a of [1, 2, 3]) f.wire['inverseInertia' + a].fill(0);
    return f;
}
function drive(f, tangential, normal = 1) {
    f.wire.forceX.fill(tangential / DT); f.wire.forceY.fill(normal / DT);
}
function accepted(f, velocity, impulse) {
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true, JSON.stringify(result.diagnostics));
    const d = result.diagnostics;
    assert.equal(d.historyCommits, 1); assert.equal(d.wallFrictionCertificate.accepted, true);
    assert.deepEqual(d.unverifiedHistoryKinds, []);
    for (const v of f.wire.velocityX) near(v, velocity, 'native physical velocity');
    const tangentialImpulse = d.contacts.reduce((sum, c) => sum + c.tangentPhysical[0] / DT, 0);
    near(tangentialImpulse, 3 * impulse, 'accepted force accounts for all three masses');
    assert.ok(f.wire.forceX.every(v => v === 0) && f.wire.forceY.every(v => v === 0));
    assert.ok(d.normalCertificate.settled && d.finalPhysicalResidualSettled);
    return d;
}

test('World captures rest before the force kick and retains static mode across accepted dt', () => {
    const f = wallFixture();
    for (let i = 0; i < 3; i++) {
        drive(f, .4); const d = accepted(f, 0, -.4);
        assert.equal(d.wallFrictionRestarts, 0);
        assert.equal(d.wallFrictionAttempts.length, 1);
        assert.ok(d.wallFrictionCertificate.contacts.every(c => c.mode === 'stick' && c.muApplied === .6));
        assert.equal(f.world.stepCount, i + 1);
    }
});

test('World breakaway restores the whole static attempt and applies one kinetic impulse', () => {
    const f = wallFixture(); drive(f, .8);
    const d = accepted(f, .6, -.2);
    assert.equal(d.wallFrictionRestarts, 1);
    assert.deepEqual(d.wallFrictionAttempts.map(a => a.status), ['restart', 'accepted']);
    assert.equal(d.physicalPasses, d.wallFrictionAttempts.reduce((sum, a) => sum + a.passes, 0));
    assert.ok(d.wallFrictionCertificate.contacts.every(c => c.mode === 'slide' && c.muApplied === .2));
    assert.ok(d.contacts.every(c => c.mu.every(mu => mu === .2)), 'public statistics report the applied coefficient');
    assert.equal(f.world.stepCount, 1);
    assert.ok(f.world.lastJointFactorizations >= 2, 'discarded static solve remains in the work counters');
    assert.ok(f.constraint._splitMotion.wallFrictionModes.committed);
});

test('World sliding uses kinetic friction even when static friction could stop the rod', () => {
    const f = wallFixture(); f.wire.velocityX.fill(.1); drive(f, .3);
    const d = accepted(f, .2, -.2);
    assert.equal(d.wallFrictionRestarts, 0);
    assert.ok(d.wallFrictionCertificate.contacts.every(c => c.mode === 'slide'));
});

test('World zero kinetic coefficient preserves static holding and releases tangential force on breakaway', () => {
    const f = wallFixture(.6, 0); drive(f, .4); accepted(f, 0, -.4);
    drive(f, .8); const d = accepted(f, .8, 0);
    assert.equal(d.wallFrictionRestarts, 1);
    assert.ok(d.contacts.every(c => c.tangentPhysical.every(v => v === 0)));
});

test('World failed kinetic retry rolls back input, mode history and time; advance prepares the same dt once', () => {
    const f = wallFixture(); drive(f, .4); accepted(f, 0, -.4);
    const history = f.constraint._wallFrictionHistory;
    const oldVelocity = f.wire.velocityX.slice(), oldX = f.wire.x.slice(), oldStep = f.world.stepCount;
    const native = f.world.coupledSystem.solve;
    let fail = true, prepared = 0;
    f.world.coupledSystem.solve = (c, dt, options) => {
        const result = native(c, dt, options);
        if (fail && c._splitMotion?.phase === 'physical' && c._splitMotion.wallFrictionModes.attempt === 2)
            result.diagnostics = { ...result.diagnostics, converged: false, status: 'test-rejected-kinetic-direction' };
        return result;
    };
    const prepare = () => { prepared++; drive(f, .8); };
    assert.equal(f.world.advance(DT, prepare), 0);
    assert.equal(prepared, 1); assert.equal(f.world.stepCount, oldStep); assert.equal(f.world.accumulator, DT);
    assert.equal(f.constraint._wallFrictionHistory, history);
    assert.deepEqual(f.wire.velocityX, oldVelocity); assert.deepEqual(f.wire.x, oldX);
    assert.ok(f.wire.forceX.every(v => v === 96));
    const rejected = structuredClone(f.world.lastStepResult);
    assert.equal(rejected.diagnostics.historyCommits, 0);
    assert.deepEqual(rejected.diagnostics.wallFrictionAttempts.map(a => a.status), ['restart', 'unconverged']);
    fail = false;
    assert.equal(f.world.advance(0, prepare), 1);
    assert.equal(prepared, 1); assert.equal(f.world.stepCount, oldStep + 1); assert.equal(f.world.accumulator, 0);
    for (const v of f.wire.velocityX) near(v, .6, 'single force kick after successful retry');
    assert.notEqual(f.constraint._wallFrictionHistory, history);
    assert.equal(rejected.diagnostics.certified, false, 'owned prior rejection remains unchanged');
});

test('equal wall coefficients retain the single physical solve without mode controller or restart snapshot', () => {
    const f = wallFixture(.2, .2); drive(f, .8);
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true);
    assert.equal(f.constraint._splitMotion.wallFrictionModes, undefined);
    assert.equal(result.diagnostics.wallFrictionAttempts, undefined);
    assert.equal(result.diagnostics.wallFrictionCertificate, undefined);
    for (const v of f.wire.velocityX) near(v, .6, 'unchanged equal-coefficient dynamics');
});
