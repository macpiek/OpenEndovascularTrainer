import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { isKirchhoffDistalLumenWitness as owns } from '../src/physics/kirchhoffToolContactOwnership.js';

function fixture(shift = 0) {
    const world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120 });
    const wire = world.createRod('wire', 4, 5, { radius: .4445, foldLimitStrength: 0 });
    const catheter = world.createRod('catheter', 3, 5, { radius: .8333, foldLimitStrength: 0 });
    for (let i = 0; i < wire.count; i++) wire.setNodePosition(i, i * 5 + shift, .04, 0);
    wire.captureKirchhoffRestConfiguration(); catheter.captureKirchhoffRestConfiguration();
    const joint = world.addContainment(wire, catheter, { innerRadius: .485, startNode: 0, endNode: 1,
        containedLength: 10, enforceDistalPortal: true, openDistal: true });
    const tool = world.addToolContact(wire, catheter, { startSegmentA: 2, endSegmentA: 2,
        startSegmentB: 1, endSegmentB: 1, openDistalB: true });
    joint._slidingPortalState = { segment: 1 };
    return { world, wire, catheter, joint, tool };
}

for (const shift of [-.001, 0, .001]) test(`the hollow mouth remains open as an exposed node crosses its plane at ${shift} mm`, () => {
    const f = fixture(shift), initial = f.wire.x.slice();
    let calls = 0;
    f.world.coupledSystem = { solve(c, dt, options) {
        calls++;
        assert.equal(options.additionalRows.filter(row => row.kind === 'tool').length, 0,
            'a solid capsule must not add a ~1.24 mm false penetration inside the bore');
        return solveKirchhoffCoupledSystem(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true });
    }, apply: applyKirchhoffCoupledCorrection };
    f.world.stepFixed();
    assert.ok(calls > 0); assert.ok(f.world.lastCoupledClosureConverged);
    for (let i = 0; i < f.wire.count; i++) assert.ok(Math.abs(f.wire.x[i] - initial[i]) < 1e-4);
    assert.ok(f.tool.lambdas.every(value => value === 0));
});

test('only the current lumen witness is reassigned; external returns, solid sides and other pairs remain', () => {
    const f = fixture(), { joint, tool } = f;
    assert.equal(owns(joint, tool, 2, 1, 0, 10, .04, 0), true);
    assert.equal(owns(joint, tool, 2, 1, 0, 10, .8, 0), false);
    assert.equal(owns(joint, tool, 2, 1, .5, 10, .04, 0), false, 'remote part of the next segment');
    assert.equal(owns(joint, tool, 2, 0, 0, 10, .04, 0), false, 'non-tip catheter surface');
    assert.equal(owns({ ...joint, innerBody: {} }, tool, 2, 1, 0, 10, .04, 0), false);
    assert.equal(owns({ ...joint, enforceDistalPortal: false }, tool, 2, 1, 0, 10, .04, 0), false);
});

test('geometric ownership does not mutate an applied normal or tangential reaction', () => {
    const f = fixture(), index = 2 * f.catheter.segmentCount + 1;
    f.tool.lambdas[index] = .02;
    assert.equal(owns(f.joint, f.tool, 2, 1, 0, 10, .04, 0), true);
    assert.equal(f.tool.lambdas[index], .02);
    f.tool.lambdas[index] = 0;
    const tangentLambda = new Float64Array([.001, 0]);
    f.joint._coupledExternalFriction = { owners: new Map([[f.tool, new Map([[index, { tangentLambda }]])]]) };
    assert.equal(owns(f.joint, f.tool, 2, 1, 0, 10, .04, 0), true);
    assert.deepEqual([...tangentLambda], [.001, 0]);
});

test('ownership uses a physical radius and is invariant to a common rigid rotation and translation', () => {
    const f = fixture();
    // Rotate z by pi/2 and translate, preserving the same aperture point.
    for (const body of [f.wire, f.catheter]) for (let i = 0; i < body.count; i++) {
        const x = body.x[i], y = body.y[i]; body.x[i] = 7 - y; body.y[i] = -3 + x; body.z[i] += 2;
    }
    assert.equal(owns(f.joint, f.tool, 2, 1, 0, 7 - .04, 7, 2), true);
    assert.equal(owns(f.joint, f.tool, 2, 1, 0, 7 - .6, 7, 2), false);
});
