import assert from 'node:assert/strict';
import { RodState } from '../src/physics/rodState.js';
import { GuidewireTransport } from '../src/physics/guidewireTransport.js';

const rod = new RodState(51, 2);
const transport = new GuidewireTransport({
        rod,
        segmentLength: 2,
        guidewireLength: 100,
        sheath: { start: { x: 0, y: 0, z: 0 }, end: { x: 0, y: 20, z: 0 } },
        advanceRate: 44,
        minInsert: 0,
        maxInsert: 100
    });
transport.reset();
assert.equal(transport.progress, 0);
assert.equal(rod.computeLength(), 100);
transport.advance(1, 1);
assert.equal(transport.progress, 44);
const firstFree = transport.firstLumenNodeIndex();
assert.ok(firstFree > 0 && firstFree < rod.nodes.length - 2);
const probe = rod.nodes[firstFree + 2];
probe.x = 7;
probe.z = -3;
probe.vx = 11;
const before = [probe.x, probe.y, probe.z, probe.vx];
for (const command of [1, 0, -1]) {
    transport.advance(command, 1 / 120);
    assert.deepEqual([probe.x, probe.y, probe.z, probe.vx], before,
        'feeding controls the sheath boundary; only the direct world moves free material');
}
transport.advance(1, 100);
assert.equal(transport.progress, 100);
assert.equal(transport.advance(1, 1), 0);
transport.advance(-1, 100);
assert.equal(transport.progress, 0);
assert.equal(transport.advance(-1, 1), 0);
transport.reset();
assert.ok(rod.nodes.every(node => node.x === 0 && node.z === 0));
assert.equal('solve' in transport, false);

// The actual 44 mm/s application sequence reaches 110 mm a few ulps below
// its intended source-grid node. The native range must retain real material
// before the valve instead of only that numerically coincident endpoint.
const nativeRod = new RodState(201, 5), native = new GuidewireTransport({rod:nativeRod,
    segmentLength:5, guidewireLength:1000, sheath:transport.sheath, advanceRate:44});
native.reset();
assert.equal(native.firstProximalSupportNodeIndex(),199);
for(let step=0;step<300;step++)native.advance(1,1/120);
assert.equal(native.firstInsertedNodeIndex()-1,178);
assert.ok(Math.abs(native.insertedCoordinate(178))<1e-11);
assert.equal(native.firstProximalSupportNodeIndex(),177);
assert.ok(native.insertedCoordinate(177)<-4.99);
const originalProgress=native.progress;
for(const progress of [110,110-1e-12,110+1e-12]) {
    native.tailProgress=progress;
    assert.equal(native.firstProximalSupportNodeIndex(),177);
    assert.equal(native.progress,progress,'Support selection never changes the prescribed material feed');
}
native.tailProgress=110-1e-6;
assert.equal(native.firstProximalSupportNodeIndex(),178,'Resolved short support is retained without a geometric tolerance');
native.tailProgress=originalProgress;
native.advance(-1,1/120);
assert.equal(native.firstProximalSupportNodeIndex(),178);
native.tailProgress=1000;
assert.equal(native.firstProximalSupportNodeIndex(),0,'Support cannot invent material beyond the proximal tool endpoint');
console.log('Guidewire boundary transport tests passed');
