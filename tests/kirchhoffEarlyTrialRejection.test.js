import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { boundaryRejectsKirchhoffTrial as rejects } from '../src/physics/kirchhoffTrialRejection.js';
import { createLineSearchStats, recordLineSearchTrial } from '../src/physics/kirchhoffLineSearch.js';

test('boundary certificate excludes both convergence and merit acceptance, including equality edges', () => {
    assert.equal(rejects(.003, .001, 2), true);
    assert.equal(rejects(.002, .001, 2), false, 'equal merit is still admissible');
    assert.equal(rejects(.001, .001, .5), false, 'final convergence can bypass merit decrease');
    assert.equal(rejects(.0009, .001, .5), false);
    for (const threshold of [null, undefined, Infinity, NaN]) assert.equal(rejects(.003, .001, threshold), false);
    for (const boundary of [Infinity, NaN]) assert.equal(rejects(boundary, .001, 2), false);
    for (const tolerance of [0, -1, Infinity, NaN]) assert.equal(rejects(.003, tolerance, 2), false);
    // Even the best possible remaining residuals cannot reverse a certificate.
    for (const boundary of [.0005, .001, .0011, .003, .1]) for (const threshold of [.5, 1, 2, 10]) {
        if (!rejects(boundary, .001, threshold)) continue;
        for (const other of [0, .5, 1, 100]) {
            assert.ok(Math.max(boundary / .001, other) > threshold);
            assert.equal(boundary <= .001, false);
        }
    }
});

test('early-rejection telemetry never presents a partial certificate as a measured dominant error', () => {
    const stats = createLineSearchStats();
    recordLineSearchTrial(stats, 2, false, { earlyRejected: true, boundaryMeritLowerBound: 4 }, null, { kind: 'wall' });
    assert.equal(stats.earlyRejections, 1); assert.equal(stats.rejected[2], 1);
    assert.deepEqual(stats.rejectionTerms, {});
});

function run(mode, unproductiveDirection = false) {
    let certificates = 0, fastTrials = 0, trials = 0, full = 0;
    const field = {
        voxelSize: .5,
        querySphere(p, radius, out) {
            const floor = p.z <= p.y, distance = floor ? p.z : p.y, gap = distance - radius;
            Object.assign(out, { signedDistance: distance, signedGap: gap, penetration: Math.max(0, -gap),
                inside: distance >= 0, violation: gap < 0, branchId: floor ? 0 : 1, faceIndex: floor ? 0 : 1, source: 'test-corner', timeOfImpact: gap < 0 ? 0 : 1 });
            Object.assign(out.point, p); Object.assign(out.closestPoint, { x: p.x, y: floor ? p.y : 0, z: floor ? 0 : p.z });
            Object.assign(out.normal, { x: 0, y: floor ? 0 : 1, z: floor ? 1 : 0 }); Object.assign(out.inward, out.normal);
            Object.assign(out.target, { x: p.x, y: floor ? p.y : Math.max(p.y, radius), z: floor ? Math.max(p.z, radius) : p.z });
            return out;
        },
        queryCapsule(a, b, radius, out) {
            const p = Math.min(a.y, a.z) <= Math.min(b.y, b.z) ? a : b;
            this.querySphere(p, radius, out); out.segmentT = p === a ? 0 : 1; return out;
        },
        queryCapsuleSoA(x, y, z, radii, index, out) {
            return this.queryCapsule({ x: x[index], y: y[index], z: z[index] },
                { x: x[index + 1], y: y[index + 1], z: z[index + 1] }, Math.max(radii[index], radii[index + 1]), out);
        },
        sweepSphere(a, b, radius, out) { return this.querySphere(b, radius, out); }
    };
    const world = new EndovascularPhysicsWorld({ contactField: field, fixedDt: 1 / 120, coupledSystem: {
        physicalTrialState: true, earlyTrialRejection: mode !== 'reference',
        solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt, {
            ...options, activeCondensation: true, simultaneousCoulomb: true
        }), apply(c, result) {
            applyKirchhoffCoupledCorrection(c, result);
            // Deliberately poor common translation: preserve relative tool
            // geometry while driving both tools into the floor. This exercises
            // rollback and the complete final failure report deterministically.
            if (unproductiveDirection) for (const body of world.bodies)
                for (let n = body.activeStart; n <= body.activeEnd; n++) body.z[n] -= result.scale;
        }
    } });
    const profile = { radius: .4445, linearDamping: 1, angularDamping: 1,
        projectionVelocityRetention: 1, maxBendAngle: 45, foldLimitStrength: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 8, 10, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', 8, 10, { ...profile, mass: 3 });
    for (let i = 0; i < 8; i++) {
        inner.setNodePosition(i, i * 10, .5, .44);
        outer.setNodePosition(i, i * 10, .44, .44);
        inner.velocityX[i] = 6;
    }
    inner.angularVelocityX.fill(2);
    const c = world.addContainment(inner, outer, { innerRadius: .485, axialFriction: .2, torsionalFriction: .2,
        portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    if (mode === 'audit') world.debugJointEarlyRejection = (_c, certificate, state) => {
        assert.equal(state.settled, false);
        assert.ok(state.merit >= certificate.boundaryMeritLowerBound);
        assert.ok(state.merit > certificate.rejectionThreshold);
        certificates++;
    };
    world.debugJointTrial = (_c, state, pass, trial) => {
        if (state.earlyRejected) {
            assert.ok(pass > 0); assert.ok(trial < 7);
            assert.equal('merit' in state, false);
            fastTrials++;
        }
    };
    const steps = [];
    for (let i = 0; i < 4; i++) {
        world.stepFixed();
        trials += world.lastJointCosts.measureCalls;
        full += world.lastJointCosts.fullMeasureCalls;
        steps.push({
            bodies: world.bodies.map(body => Object.fromEntries([
                'x', 'y', 'z', 'previousX', 'previousY', 'previousZ', 'velocityX', 'velocityY', 'velocityZ',
                'orientationX', 'orientationY', 'orientationZ', 'orientationW',
                'adaptationLambdaX', 'adaptationLambdaY', 'adaptationLambdaZ',
                'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3', 'wallLambda'
            ].map(key => [key, Array.from(body[key])]))),
            contacts: [...c.manifold.contacts()].map(contact => ({ id: contact.id,
                normalLambda: contact.normalLambda, tangentLambda: Array.from(contact.tangentLambda),
                normal: Array.from(contact.normal), tangentU: Array.from(contact.tangentU), tangentV: Array.from(contact.tangentV) })),
            accepted: [...world.lastJointLineSearch.accepted], rejected: [...world.lastJointLineSearch.rejected],
            failure: structuredClone(world.lastJointNonlinearFailure)
        });
    }
    return { steps, certificates, fastTrials, trials, full };
}

test('unproductive trial rejection preserves coupled contact history and complete final failure diagnostics', () => {
    const reference = run('reference', true), audit = run('audit', true), fast = run('fast', true);
    assert.ok(audit.certificates > 0, JSON.stringify({trials:audit.trials, failures:audit.steps.map(s=>s.failure), accepted:audit.steps.map(s=>s.accepted), rejected:audit.steps.map(s=>s.rejected)}));
    assert.equal(fast.fastTrials, audit.certificates);
    assert.deepEqual(audit.steps, reference.steps);
    assert.deepEqual(fast.steps, reference.steps);
    assert.equal(fast.trials, reference.trials);
    assert.equal(fast.full, reference.full - fast.fastTrials);
});

test('converging coupled corrections still receive complete measurements', () => {
    const reference = run('reference'), fast = run('fast');
    assert.deepEqual(fast.steps, reference.steps);
    assert.ok(fast.steps.some(step => step.failure === null));
});
