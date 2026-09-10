import assert from 'node:assert/strict';
import test from 'node:test';
import { measureKirchhoffBiasReleases } from '../src/physics/kirchhoffTwoChannelRelease.js';
import { measureKirchhoffBiasReleasesReference } from './fixtures/twoChannelReleaseMeasureReference.js';
import { sparseReleaseMeasureFixture, releaseMeasureOwnedReferences, measureFloat64Allocations, releaseMeasureBytes } from './fixtures/sparseReleaseMeasureFixture.js';

function byteParity(joint) {
    const before = releaseMeasureBytes(joint), refs = releaseMeasureOwnedReferences(joint);
    const reference = measureKirchhoffBiasReleasesReference(joint), referenceBytes = releaseMeasureBytes(reference);
    assert.deepEqual(releaseMeasureBytes(joint), before, 'baseline changes no retired/body state');
    const actual = measureKirchhoffBiasReleases(joint);
    assert.deepEqual(releaseMeasureBytes(actual), referenceBytes, 'entire output including typed-array bytes, scalar order, modes and issues');
    assert.deepEqual(releaseMeasureBytes(joint), before, 'optimized measurement changes no retired/body state');
    releaseMeasureOwnedReferences(joint).forEach((r, i) => assert.equal(r, refs[i], 'owned reference retained'));
    actual.releaseCorrection.forEach((c, side) => assert.deepEqual(Buffer.from(c.buffer, c.byteOffset, c.byteLength),
        Buffer.from(reference.releaseCorrection[side].buffer), 'full shared correction bytes'));
    return actual;
}

test('sparse norms have complete A/B byte parity on long rods, rotated frames, active masks and duplicate DOFs', () => {
    for (const options of [
        { counts: [16, 21], activeStart: [3, 5], releases: 80 },
        { counts: [200, 200], releases: 400 },
        { counts: [201, 320], activeStart: [13, 17], releases: 400 },
        { counts: [503, 601], activeStart: [41, 53], releases: 137 },
        { counts: [200, 200], releases: 400, roundingSensitive: true }
    ]) {
        const measured = byteParity(sparseReleaseMeasureFixture(options));
        assert.equal(measured.finite, true); assert.ok(measured.releasePositionMm > 0 && measured.releaseAngleRad > 0);
    }
});

test('shared correction preserves exact release ordering and opposing-wrench cancellation before global norms', () => {
    const joint = sparseReleaseMeasureFixture({ paired: true });
    // With one contribution per DOF, each adjacent opposite pair cancels
    // exactly. The separate duplicate/large-value case checks the baseline's
    // non-associative rounding as well, without assuming it sums to zero.
    for (const r of joint._splitMotion.twoChannel.rows.releases) {
        const seen = new Set();
        r.frozenWorldGradients = r.frozenWorldGradients.filter(g => {
            const key = g.side + ':' + g.dof; if (seen.has(key)) return false; seen.add(key); return true;
        });
    }
    const result = byteParity(joint);
    assert.equal(result.pendingReleases.length, 400);
    assert.ok(result.pendingReleases.every(r => r.positionMm > 0 && r.angleRad > 0));
    assert.ok(result.releaseCorrection.every(c => c.every(v => v === 0)));
    assert.equal(result.releasePositionMm, 0); assert.equal(result.releaseAngleRad, 0);
});

test('empty, zero, failed-history, inactive, overflow and ignored out-of-range entries retain baseline output and guards', () => {
    byteParity(sparseReleaseMeasureFixture({ releases: 0 }));
    for (const failure of ['zero', 'negative', 'history', 'wrench', 'mobility', 'sleeping', 'overflow', 'out-of-range']) {
        const joint = sparseReleaseMeasureFixture({ releases: 4 }), r = joint._splitMotion.twoChannel.rows.releases[0];
        if (failure === 'zero') { r.bank.lambda = 0; delete r.historyGuards; }
        if (failure === 'negative') r.bank.lambda = -1;
        if (failure === 'history') delete r.historyGuards;
        if (failure === 'wrench') r.frozenWorldGradients[0].value = NaN;
        if (failure === 'mobility') r.historyGuards[0].value += .01;
        if (failure === 'sleeping') { joint.innerBody.sleeping = true; joint.outerBody.activeEnd = joint.outerBody.activeStart; }
        if (failure === 'overflow') { r.bank.lambda = 2; r.frozenWorldGradients = [{ side: 0, dof: 6 * 17 + 1, value: Number.MAX_VALUE }]; }
        if (failure === 'out-of-range') { r.bank.lambda = 2; r.frozenWorldGradients = [{ side: 0, dof: 6 * joint.innerBody.count + 1, value: Number.MAX_VALUE }]; }
        byteParity(joint);
    }
});

test('only two full correction vectors are allocated, independently of sparse release count', () => {
    const joint = sparseReleaseMeasureFixture(), baseline = measureFloat64Allocations(measureKirchhoffBiasReleasesReference, joint);
    const current = measureFloat64Allocations(measureKirchhoffBiasReleases, joint);
    assert.equal(baseline.arrays, 802); assert.equal(baseline.bytes, 7_699_200);
    assert.equal(current.arrays, 3); assert.deepEqual(current.lengths, [1200, 1200, 6]); assert.equal(current.bytes, 19_248);
    assert.equal(measureFloat64Allocations(measureKirchhoffBiasReleases,
        sparseReleaseMeasureFixture({ releases: 1 })).bytes, current.bytes);
});
