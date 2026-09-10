import test from 'node:test';
import assert from 'node:assert/strict';
import { createFrozenCoupledMetadata } from '../scripts/physics/frozen-coupled-metadata.mjs';

test('run-local object identity cannot hide a material relabel behind an unchanged runtime id', () => {
    const tracker = createFrozenCoupledMetadata();
    const contact = { id: 'runtime:1:7', feature: 'rim', innerMaterialSegmentId: 10, outerMaterialSegmentId: 20,
        tangentLambda: Float64Array.of(.1, .2), normal: [0, 1, 0], normalLambda: 1 };
    const record = { id: 'same-record', kind: 'sliding-rim', innerT: .25, outerT: .75 };
    const first = tracker.contact(contact, record);
    contact.innerMaterialSegmentId = 11; contact.tangentLambda[0] = .3; record.innerT = .5;
    const second = tracker.contact(contact, record);
    assert.equal(first.objectId, second.objectId);
    assert.equal(first.id, second.id);
    assert.notEqual(first.innerMaterialSegmentId, second.innerMaterialSegmentId);
    assert.equal(first.innerT, .25);
    assert.deepEqual(first.tangentLambda, [.1, .2]);
    assert.notEqual(tracker.contact({ ...contact }, record).objectId, first.objectId);
});

test('friction export owns the actual surface basis and multiplier components before a pool rebuild', () => {
    const tracker = createFrozenCoupledMetadata();
    const axes = [[0, 0, 1], [1, 0, 0]], rows = [{ lambda: .1 }, { lambda: .2 }];
    const contact = { tangentU: [1, 0, 0], tangentV: [0, 0, 1], tangentLambda: [.2, .1] };
    const entry = { contact, record: { kind: 'side' }, rowStart: 2,
        surface: { axes, rows, group: { normalLambda: 1 } } };
    const joint = { manifold: {}, _jointFrictionBatch: { rowOffset: 10, entries: [entry] } };
    const [frozen] = tracker.friction(joint);
    axes[0][2] = 0; rows[0].lambda = 9; entry.rowStart = 4;
    assert.deepEqual(frozen.axes, [[0, 0, 1], [1, 0, 0]]);
    assert.deepEqual(frozen.lambda, [.1, .2]);
    assert.deepEqual(frozen.additionalRows, [12, 13]);
    assert.equal(frozen.forceSide, 0);
    assert.notDeepEqual(frozen.axes[0], frozen.contact.tangentU);
});
