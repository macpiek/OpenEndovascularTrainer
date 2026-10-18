import assert from 'node:assert/strict';
import test from 'node:test';
import { createSharedAxisNative, feedSharedAxisNative } from '../src/physics/kirchhoffSharedAxisNative.js';
import { defineKirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';

const fields = ['materialCoordinate', 'restRotation1', 'restRotation2', 'restRotation3',
    'kirchhoffBendCompliance1', 'kirchhoffBendCompliance2', 'kirchhoffTwistCompliance',
    'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3', 'restLength'];
const tools = () => [
    { id: 'wire', type: 'glidewire', insertion: 181.2, length: 1000, shaftStiffness: 13.15, tipStiffness: 19.4 },
    { id: 'catheter', type: 'pigtail', insertion: 160.6667, length: 1000, shaftStiffness: 40.65, tipStiffness: 100 }
];
function equalMaterial(actual, expected) {
    assert.deepEqual(actual.coordinates, expected.coordinates);
    for (let m = 0; m < actual.materials.length; m++) {
        const a = actual.materials[m].body, b = expected.materials[m].body;
        for (const field of fields) assert.deepEqual(a[field], b[field], `${m}/${field}`);
        for (const field of ['activeStart', 'activeEnd', 'rodModel', 'sleeping', 'sleepCounter'])
            assert.equal(a[field], b[field], `${m}/${field}`);
    }
}

test('unchanged feed preserves exact manufactured profiles for every built-in tool', () => {
    for (const type of ['glidewire', 'steel-j-035', 'pigtail', 'berenstein', 'sim1']) {
        const input = { tools: [{ ...tools()[type.includes('wire') || type === 'steel-j-035' ? 0 : 1], type }] };
        const expected = createSharedAxisNative(input);
        let state = createSharedAxisNative(input);
        for (let step = 0; step < 4; step++) {
            state = feedSharedAxisNative(state, {});
            equalMaterial(state, expected);
        }
    }
});

test('feed reevaluates changed material coordinates, stiffness, profile and mesh boundaries', () => {
    const previous = createSharedAxisNative({ tools: tools() });
    const changes = [
        { insertion: 160.8 }, { insertion: 170 }, { length: 1200 },
        { shaftStiffness: 17 }, { tipStiffness: 25 }, { type: 'berenstein' }
    ];
    for (const change of changes) {
        const input = { tools: [tools()[0], { ...tools()[1], ...change }] };
        equalMaterial(createSharedAxisNative({ ...input, previous }), createSharedAxisNative(input));
    }
    for (const mesh of [{ spacing: 4 }, { boundaryCoordinates: [77.5] }]) {
        const input = { tools: tools(), ...mesh };
        equalMaterial(createSharedAxisNative({ ...input, previous }), createSharedAxisNative(input));
    }
    const extended = createSharedAxisNative({ tools: tools(), startCoordinate: -5 });
    equalMaterial(createSharedAxisNative({ tools: tools(), previous: extended }),
        createSharedAxisNative({ tools: tools() }));
    // Failed input must still reach the ordinary validation on a cache miss.
    for (const scale of [NaN, 0, -1, Infinity]) assert.throws(() => createSharedAxisNative({
        previous, tools: [tools()[0], { ...tools()[1], shaftStiffness: scale }]
    }), /stiffness scale/);
});

test('live body edits and discarded feed candidates do not contaminate manufactured values', () => {
    const input = { tools: tools() }, previous = createSharedAxisNative(input);
    for (const { body } of previous.materials) {
        for (const field of fields) body[field].fill(7);
        body.sleeping = true;
        body.sleepCounter = 99;
    }
    const discarded = createSharedAxisNative({ previous, tools: [tools()[0], { ...tools()[1], tipStiffness: 23 }] });
    discarded.materials[0].body.restRotation1.fill(123);
    equalMaterial(createSharedAxisNative({ ...input, previous }), createSharedAxisNative(input));
});

test('configure callbacks run on freshly initialized constitutive values on every feed', () => {
    let calls = 0, adjustment = .25;
    const tool = { ...tools()[1], configure(body) {
        calls++;
        body.restRotation1[4] += adjustment;
        body.kirchhoffBendCompliance1[3] *= 2;
    } };
    const previous = createSharedAxisNative({ tools: [tool] });
    adjustment = .5;
    const next = feedSharedAxisNative(previous, {});
    assert.equal(calls, 2);
    equalMaterial(next, createSharedAxisNative({ tools: [tool] }));
    const plain = { ...tool }; delete plain.configure;
    equalMaterial(createSharedAxisNative({ tools: [plain], previous: next }), createSharedAxisNative({ tools: [plain] }));
});

test('mutable custom profile samplers and changing delivery exposure never reuse built-in values', () => {
    let rigidity = 1000, calls = 0;
    const profile = defineKirchhoffMaterialProfile({ id: 'pigtail',
        sampleEI1: () => { calls++; return rigidity; }, sampleGJ: () => rigidity / 1.4 });
    const custom = { ...tools()[1], type: profile, shaftStiffness: 1, tipStiffness: 1 };
    const previous = createSharedAxisNative({ tools: [custom] });
    calls = 0; rigidity = 5000;
    const next = feedSharedAxisNative(previous, {});
    assert.ok(calls > 0);
    equalMaterial(next, createSharedAxisNative({ tools: [custom] }));
    assert.notDeepEqual(next.materials[0].body.kirchhoffBendCompliance1, previous.materials[0].body.kirchhoffBendCompliance1);
    const delivery = { ...custom, type: 'stentgraft-delivery', deliveryExposureMm: 0 };
    const covered = createSharedAxisNative({ tools: [delivery] });
    const exposed = { ...delivery, deliveryExposureMm: 100 };
    const release = createSharedAxisNative({ tools: [exposed], previous: covered });
    equalMaterial(release, createSharedAxisNative({ tools: [exposed] }));
    assert.notDeepEqual(release.materials[0].body.kirchhoffBendCompliance1, covered.materials[0].body.kirchhoffBendCompliance1);
});
