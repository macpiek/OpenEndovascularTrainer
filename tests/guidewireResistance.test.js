import assert from 'node:assert/strict';
import {
    GuidewireResistanceEstimator,
    sampleGuidewireResistance
} from '../src/physics/guidewireResistance.js';

function createBody({ lambda = 0, normal = [0, 1, 0], active = false } = {}) {
    return {
        x: new Float32Array([0, 1]),
        y: new Float32Array([0, 0]),
        z: new Float32Array([0, 0]),
        wallLambda: new Float32Array([lambda]),
        wallActive: new Uint8Array([active ? 1 : 0]),
        wallNormalX: new Float32Array([normal[0]]),
        wallNormalY: new Float32Array([normal[1]]),
        wallNormalZ: new Float32Array([normal[2]]),
        wallFriction: 0.002,
        wallKineticFriction: 0.002
    };
}

const free = sampleGuidewireResistance(createBody());
assert.equal(free.level, 0, 'a freely moving guidewire should report no resistance');
assert.equal(free.activeContacts, 0);

const grazing = sampleGuidewireResistance(createBody({
    lambda: 0.08,
    normal: [0, 1, 0],
    active: true
}));
const wedged = sampleGuidewireResistance(createBody({
    lambda: 0.08,
    normal: [1, 0, 0],
    active: true
}));
assert.ok(grazing.level > 0, 'wall friction should create a small insertion resistance');
assert.ok(grazing.level < 0.01,
    'lateral support at a grazing contact must not look like high handle resistance');
assert.ok(
    wedged.level > grazing.level * 2,
    'a guidewire directed into the wall should report more resistance than a grazing contact'
);

const shielded = createBody({
    lambda: 0.12,
    normal: [1, 0, 0],
    active: true
});
shielded.activeStart = 0;
shielded.activeEnd = 1;
shielded.collisionStartSegment = 1;
shielded.collisionEndSegment = 0;
const shieldedResistance = sampleGuidewireResistance(shielded);
assert.equal(shieldedResistance.level, 0,
    'a guidewire segment shielded by the catheter must not report vessel resistance');
assert.equal(shieldedResistance.activeContacts, 0);

const estimator = new GuidewireResistanceEstimator();
let result = estimator.update(createBody(), { dt: 1 / 120, command: 0 });
assert.equal(result.reason, 'Swobodne wsuwanie prowadnika');
for (let step = 0; step < 30; step++) {
    result = estimator.update(createBody({
        lambda: 0.12,
        normal: [1, 0, 0],
        active: true
    }), { dt: 1 / 120, command: 1 });
}
assert.ok(result.level > 0.7, 'sustained wedging should reach the high-resistance range');
assert.equal(result.reason, 'Wysoki opór — cofnij lub zmień kierunek');

result = estimator.update(createBody(), {
    dt: 0.5,
    command: 1,
    atMaximumInsertion: true
});
assert.ok(result.level > 0.9, 'continued insertion at the length limit should saturate the indicator');
assert.equal(result.reason, 'Osiągnięto maksymalną długość prowadnika');

console.log('guidewire resistance tests passed');
