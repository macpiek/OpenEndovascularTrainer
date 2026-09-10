import assert from 'node:assert/strict';
import test from 'node:test';
import { transportCatheterThroughSheath } from '../src/physics/catheterSheathTransport.js';
import { DEFAULT_TOOL_PROFILES, EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import { RodState } from '../src/physics/rodState.js';
import { GuidewireTransport } from '../src/physics/guidewireTransport.js';
import { SHEATH_BOUNDARY_EPSILON } from '../src/physics/sheathBoundary.js';

const dt = 1 / 120;
const sheath = { start: { x: 7, y: -2, z: 3 }, end: { x: 7, y: 4, z: 11 } };
const axis = [0, 0.6, 0.8];
function fixture(progress = 26) {
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('catheter-feed', 21, 4, { ...DEFAULT_TOOL_PROFILES.catheter });
    const state = transportCatheterThroughSheath(body, sheath, progress, dt);
    return { body, state, world };
}
function near(actual, expected, message) {
    assert.ok(Math.abs(actual - expected) < 1e-10, `${message}: ${actual} != ${expected}`);
}

test('wire and catheter release the same material nodes at the sheath outlet', () => {
    const { body, state } = fixture(0);
    const rod = new RodState(body.count, body.segmentLength);
    const transport = new GuidewireTransport({ rod, segmentLength: 4,
        guidewireLength: 80, sheath, maxInsert: 76 });
    transport.initialize();
    for (const progress of [10 - SHEATH_BOUNDARY_EPSILON, 10,
        10 + SHEATH_BOUNDARY_EPSILON / 2, 10 + 2 * SHEATH_BOUNDARY_EPSILON,
        14 + 2 * SHEATH_BOUNDARY_EPSILON, 14, 10]) {
        transport.tailProgress = progress;
        transport.constrainSheath();
        transportCatheterThroughSheath(body, sheath, progress, dt, state);
        for (let i = 0; i < body.count; i++) {
            assert.equal(Boolean(body.pinned[i]), Boolean(rod.nodes[i].pinned),
                `pinning at progress ${progress}, node ${i}`);
        }
    }
});
const nodeFields = ['x', 'y', 'z', 'previousX', 'previousY', 'previousZ', 'velocityX', 'velocityY', 'velocityZ'];
const segmentFields = ['orientationX', 'orientationY', 'orientationZ', 'orientationW',
    'angularVelocityX', 'angularVelocityY', 'angularVelocityZ', 'wallLambda', 'wallFrictionLambda',
    'wallFrictionLoad', 'wallFaceIndex', 'lengthLambda', 'bendTwistLambda1'];

test('material chain, tip identity and rest lengths survive fractional feed and node-boundary crossings', () => {
    const { body, state } = fixture(0);
    const arrays = Object.fromEntries([...nodeFields, ...segmentFields, 'restLength', 'materialCoordinate'].map(key => [key, body[key]]));
    const labels = Array.from(body.materialCoordinate);
    const rest = Array.from(body.restLength);
    for (const progress of [0.25, 3.75, 4, 4.25, 27.5, 47.75, 25.25, 4, 0.25, 0]) {
        transportCatheterThroughSheath(body, sheath, progress, dt, state);
        assert.equal(body.count, 21);
        assert.equal(body.activeEnd, 20, 'tip remains the same material node');
        assert.deepEqual(Array.from(body.restLength), rest);
        for (const [key, array] of Object.entries(arrays)) assert.equal(body[key], array, `${key} storage must not be replaced`);
        for (let i = 0; i < body.count; i++) near(body.materialCoordinate[i] - progress, labels[i], `material label ${i}`);
    }
});

test('feed, withdrawal and idle preserve exposed mechanical and contact state before the world solve', () => {
    const { body, state } = fixture();
    for (const [k, key] of nodeFields.entries()) for (let i = 18; i <= 20; i++) body[key][i] = (k + 1) * 0.125 + i;
    for (const [k, key] of segmentFields.entries()) for (let i = 18; i < 20; i++) body[key][i] = (k + 1) * 0.125 + i;
    body.wallActive[18] = body.wallActive[19] = 1;
    const before = Object.fromEntries([...nodeFields, ...segmentFields, 'wallActive'].map(key => [key, Array.from(body[key].slice(18))]));
    for (const progress of [30.25, 24.25, 24.25, 27.75, 26]) {
        transportCatheterThroughSheath(body, sheath, progress, dt, state);
        for (const [key, expected] of Object.entries(before)) assert.deepEqual(Array.from(body[key].slice(18)), expected, `${key} changed at progress ${progress}`);
    }
});

test('only prescribed sheath nodes receive signed axial feed velocity, including zero-speed hold', () => {
    const { body, state } = fixture();
    let previous = state.progress;
    for (const progress of [26.125, 25.875, 25.875, 26.25]) {
        const speed = (progress - previous) / dt;
        transportCatheterThroughSheath(body, sheath, progress, dt, state);
        for (let i = 0; i < state.firstOutside; i++) {
            assert.equal(body.pinned[i], 1);
            assert.equal(body.inverseMass[i], 0);
            for (const [a, name] of ['X', 'Y', 'Z'].entries()) {
                const position = sheath.start[name.toLowerCase()] + axis[a] * body.materialCoordinate[i];
                assert.equal(body[name.toLowerCase()][i], Math.fround(position), `axis position ${i}/${name}`);
                assert.equal(body['velocity' + name][i], Math.fround(axis[a] * speed), `feed velocity ${i}/${name}`);
            }
        }
        for (let i = state.firstOutside; i < body.count; i++) assert.equal(body.pinned[i], 0);
        previous = progress;
    }
});

test('active support and collision windows follow material crossings in both directions', () => {
    const { body, state } = fixture(0);
    // [progress, proximal support node, first unpinned node, first collision segment]
    for (const [progress, start, outside, collision] of [
        [0, 19, 21, null], [4, 18, 21, null], [10, 17, 21, null],
        [10.25, 17, 20, 19], [14, 16, 20, 19], [14.25, 16, 19, 18],
        [10, 17, 21, null], [0, 19, 21, null]
    ]) {
        transportCatheterThroughSheath(body, sheath, progress, dt, state);
        assert.equal(body.activeStart, start, `active start at ${progress}`);
        assert.equal(state.firstOutside, outside, `outside at ${progress}`);
        assert.equal(state.lumenStart, start);
        assert.ok(state.lumenOrigin <= 0 && state.lumenOrigin >= -4);
        if (collision === null) assert.ok(body.collisionEndSegment < body.collisionStartSegment);
        else {
            assert.equal(body.collisionStartSegment, collision);
            assert.equal(body.collisionEndSegment, 19);
        }
    }
});

test('reset reconstructs the complete reservoir and next feed starts at the reset progress', () => {
    const { body, state } = fixture(35);
    body.x[20] = 901; body.velocityZ[20] = 71;
    transportCatheterThroughSheath(body, sheath, 0, dt, state, { reset: true });
    assert.equal(state.delta, 0);
    assert.equal(body.activeStart, 19);
    for (let i = 0; i < body.count; i++) {
        assert.equal(body.pinned[i], 1);
        for (const [a, name] of ['X', 'Y', 'Z'].entries()) {
            assert.equal(body[name.toLowerCase()][i], Math.fround(sheath.start[name.toLowerCase()] + axis[a] * body.materialCoordinate[i]), 'reset axis');
            assert.equal(body['velocity' + name][i], 0);
        }
    }
    transportCatheterThroughSheath(body, sheath, 0.25, dt, state);
    assert.equal(state.delta, 0.25);
    assert.equal(body.velocityZ[20], Math.fround(0.8 * 0.25 / dt), 'post-reset feed');
});

test('public sheath feed and sync never sample the guidewire route, with or without guidewire overlap', () => {
    const { body } = fixture(0);
    const catheter = new PigtailCatheter({ wire: new RodState(41, 2), segmentLength: 2, guidewireLength: 80,
        tailProgressRef: () => 60, vessel: { sheath, segments: [] }, maxLength: 60 });
    try {
        catheter.syncXpbdBody(body);
        catheter.wire = new Proxy({}, { get() { throw new Error('Guidewire geometry sampled during catheter feed'); } });
        catheter.tailProgressRef = () => { throw new Error('Guidewire route sampled during catheter feed'); };
        for (const overlap of [0, 60, 20]) for (const command of [1, 1, 0, -1]) {
            catheter.advance(command, 0.2, overlap);
            catheter.stepPhysics(0.2);
            catheter.syncXpbdBody(body);
        }
        assert.equal(catheter.pathSamples.length, 0);
        assert.equal(body.activeEnd, body.count - 1);
        assert.equal(catheter.physicsActiveCount, body.count);
    } finally { catheter.dispose(); }
});

test('idle transport in an oblique sheath preserves sleep despite Float32 position rounding', () => {
    const { body, state } = fixture(26.125);
    const before = Object.fromEntries(nodeFields.map(key => [key, Array.from(body[key])]));
    body.sleeping = true;
    for (let step = 0; step < 20; step++) {
        transportCatheterThroughSheath(body, sheath, 26.125, dt, state);
        assert.equal(body.sleeping, true, `idle transport woke the rod at repetition ${step}`);
        for (const [key, values] of Object.entries(before)) assert.deepEqual(Array.from(body[key]), values, `${key} moved during idle`);
    }
});

test('explicit reset at unchanged nonzero insertion clears stale contact and control state without relabelling material', () => {
    const { body, state } = fixture(26.125);
    const material = Array.from(body.materialCoordinate);
    const rest = Array.from(body.restLength);
    const range = [body.activeStart, body.activeEnd, body.collisionStartSegment, body.collisionEndSegment];
    const zeroFields = ['lengthLambda', 'controlEnabled', 'controlLambda', 'wallLambda',
        'wallFrictionLambda', 'wallFrictionLoad', 'wallActive', 'wallInsideClearance',
        'wallCapsuleSampleCount', 'wallProjectionX', 'wallProjectionY', 'wallProjectionZ',
        'toolProjectionX', 'toolProjectionY', 'toolProjectionZ'];
    // In particular poison contacts that remain INSIDE the unchanged collision
    // window: changing the collision mask cannot clear these for the reset.
    for (const key of zeroFields) body[key].fill(1);
    body.wallBranchId.fill(23); body.wallFaceIndex.fill(41); body.wallGap.fill(-0.125);
    transportCatheterThroughSheath(body, sheath, 26.125, dt, state, { reset: true });
    assert.equal(state.delta, 0);
    assert.deepEqual(Array.from(body.materialCoordinate), material);
    assert.deepEqual(Array.from(body.restLength), rest);
    assert.deepEqual([body.activeStart, body.activeEnd, body.collisionStartSegment, body.collisionEndSegment], range);
    for (const key of zeroFields) assert.ok(body[key].every(value => value === 0), `${key} survived explicit reset`);
    assert.ok(body.wallBranchId.every(value => value === -1));
    assert.ok(body.wallFaceIndex.every(value => value === -1));
    assert.ok(body.wallGap.every(value => value === Infinity));
});
