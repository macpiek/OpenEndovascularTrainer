import assert from 'node:assert/strict';
import test from 'node:test';
import { fixture } from './fixtures/splitMotionAnalyticWorld.js';
import { beginKirchhoffSplitMotion, appendKirchhoffSplitPointWalls, captureKirchhoffSplitSweep } from '../src/physics/kirchhoffSplitMotion.js';

test('an empty vessel capsule interval cannot create a phantom endpoint contact', () => {
    const f = fixture({ wall: true, y: .25 });
    f.wire.setActiveRange(0, 1);
    f.wire.setCollisionRange(f.wire.activeEnd, f.wire.activeEnd - 1);
    assert.equal(f.wire.collisionStartSegment, 1);
    assert.equal(f.wire.collisionEndSegment, 0);
    // This wire lies outside the analytic vessel, but its explicit empty
    // collision interval exempts it (as for material owned by an introducer).
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true, JSON.stringify(result));
    assert.equal(result.diagnostics.normalCertificate.maximumRawPenetrationMm, 0);
    assert.deepEqual(result.diagnostics.contacts, []);
    assert.ok(f.wire.y.every(y => y === .25), 'no phantom wall force or bias correction');
});

function pointRows(activeStart, activeEnd, collisionStart, collisionEnd) {
    const f = fixture({ wall: true, y: -.25 });
    f.wire.setActiveRange(activeStart, activeEnd);
    f.wire.setCollisionRange(collisionStart, collisionEnd);
    f.catheter.setCollisionRange(1, 0);
    beginKirchhoffSplitMotion(f.constraint, f.world);
    const rows = [];
    appendKirchhoffSplitPointWalls(f.constraint, f.world, rows);
    return rows;
}

test('one active capsule includes both endpoints and no neighbouring endpoint', () => {
    const rows = pointRows(0, 2, 1, 1);
    assert.deepEqual(rows.map(row => [row.side, row.node]), [[0, 1], [0, 2]]);
    assert.ok(rows.every(row => row.strain < 0), 'genuine penetration remains detectable');
});

test('disjoint active and collision segment intervals do not create their shared boundary node', () => {
    assert.deepEqual(pointRows(1, 2, 0, 0), []);
});

test('endpoint friction identity retains the collider plane when a shared query object changes', () => {
    const f = fixture({ wall: true, y: -.25 });
    f.catheter.setCollisionRange(1, 0);
    beginKirchhoffSplitMotion(f.constraint, f.world);
    const rows = [];
    appendKirchhoffSplitPointWalls(f.constraint, f.world, rows);
    const identity = structuredClone(rows[0].wallFrictionWitness);
    assert.deepEqual(identity, { branchId: 0, faceIndex: 0, planeOffset: 0 });
    f.constraint._splitMotion.wallQuery.closestPoint.y = 99;
    f.constraint._splitMotion.wallQuery.faceIndex = 50;
    assert.deepEqual(rows[0].wallFrictionWitness, identity);
    f.wire.y.fill(-.3);
    appendKirchhoffSplitPointWalls(f.constraint, f.world, []);
    assert.deepEqual(rows[0].wallFrictionWitness, identity, 'normal rod motion does not move the collider plane');
});

test('sweep friction identity owns the plane of its retained normal constraint', () => {
    const f = fixture({ wall: true, y: -.5 });
    beginKirchhoffSplitMotion(f.constraint, f.world);
    f.wire.y[0] = -.25;
    const contact = { timeOfImpact: .4, inward: { x: 0, y: -1, z: 0 }, branchId: 7, faceIndex: 12 };
    captureKirchhoffSplitSweep(f.constraint, f.wire, 0, contact);
    const row = f.constraint._splitMotion.sweeps[0];
    const identity = structuredClone(row.wallFrictionWitness);
    assert.deepEqual(identity, { branchId: 7, faceIndex: 12, planeOffset: .4 });
    contact.faceIndex = 90; contact.inward.y = 0;
    assert.deepEqual(row.wallFrictionWitness, identity);
    assert.equal(row.n.reduce((sum, n, axis) => sum + n * row.point[axis], 0), identity.planeOffset);
});
