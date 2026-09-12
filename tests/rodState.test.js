import assert from 'node:assert/strict';
import { RodState } from '../src/physics/rodState.js';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';

const state = new RodState(5, 2, {mass: 3});
assert.equal(state.computeLength(), 8);
state.nodes[2].y = 3;
state.nodes[2].vy = 7;
state.nodes[0].pinned = true;
assert.equal(state.nodeStorage.y[2], 3);
assert.equal(state.nodeStorage.vy[2], 7);
assert.equal(state.nodeStorage.pinned[0], 1);
assert.equal(state.nodes[2].mass, 3);
assert.ok(state.bendAngleAt(2) > 90);

const world = new EndovascularPhysicsWorld();
const body = world.createRod('shared-storage', 5, 2);
body.syncFromRodState(state, { resetVelocity: false });
assert.equal(body.y[2], 3);
assert.equal(body.velocityY[2], 7);
assert.equal(body.pinned[0], 1);
body.y[2] = 4;
body.velocityY[2] = -2;
body.syncToRodState(state);
assert.equal(state.nodes[2].y, 4);
assert.equal(state.nodes[2].vy, -2);
assert.equal('step' in state, false, 'storage must not own a second dynamics solver');
console.log('Rod storage and solver synchronization tests passed');
