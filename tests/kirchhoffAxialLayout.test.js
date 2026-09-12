import test from 'node:test';
import assert from 'node:assert/strict';
import { buildKirchhoffAxialLayout as build, encodeKirchhoffAxialLayout as encode,
    decodeKirchhoffAxialLayout as decode, pullbackKirchhoffAxialForces as pullback,
    applyKirchhoffAxialMobility as mobility } from '../src/physics/kirchhoffAxialLayout.js';

const rod = (axialCoordinates, positions = axialCoordinates.flatMap(x => [x, 0, 0])) => ({ axialCoordinates, positions });
const close = (a, b) => {
    assert.equal(a.length, b.length);
    a.forEach((value, i) => assert.ok(Math.abs(value - b[i]) < 1e-13, `${i}: ${value} != ${b[i]}`));
};

test('unequal spacing uses aligned interpolation, preserving original identities', () => {
    const wire = { ...rod([0, 2, 5]), nodeIndices: [7, 9, 12], materialIds: ['w7', 'w9', 'w12'] };
    const catheter = { ...rod([0, 1, 3, 5]), nodeIndices: [20, 21, 25, 30], materialIds: ['c20', 'c21', 'c25', 'c30'] };
    const before = structuredClone({ wire, catheter });
    const layout = build({ wire, catheter });
    assert.deepEqual(layout.sites[1].wireNodes, [7, 9]);
    assert.deepEqual(layout.sites[1].weights, [.5, .5]);
    assert.deepEqual(layout.sites[2].wireNodes, [9, 12]);
    close(layout.sites[2].weights, [2 / 3, 1 / 3]);
    assert.deepEqual(layout.sites[3].wireSlots, [2]);
    assert.equal(layout.sites[2].nodeIndex, 25); assert.equal(layout.sites[2].materialId, 'c25');
    assert.deepEqual({ wire, catheter }, before);
});

test('feed changes interpolation and actual overlap without changing material labels', () => {
    const wire = rod([0, 2, 4]), catheter = rod([0, 1, 2, 3, 4]);
    const a = build({ wire, catheter }), b = build({ wire, catheter, axialOffsets: [10, 11] });
    assert.deepEqual(b.overlap, [11, 14]);
    assert.deepEqual(b.sites[0].weights, [.5, .5]);
    assert.deepEqual(b.sites[4].wireSlots, []);
    assert.deepEqual(a.sites.map(s => s.materialId), b.sites.map(s => s.materialId));
    assert.equal(b.sites[0].materialCoordinate, 0);
    close(decode(b, b.referenceCoordinates).catheter, catheter.positions);
});

test('curved initial offsets are retained and roundtrip positions and arbitrary increments', () => {
    const wire = rod([0, 2, 5], [0, 0, 0, 1, 2, 0, 3, 1, 2]);
    const catheter = rod([-1, 1, 3, 6], [-1, 4, 2, .6, 1.2, .3, 2, 1.8, .9, 7, 2, 1]);
    const layout = build({ wire, catheter });
    close(layout.referenceCoordinates.catheterCoordinates.slice(3, 6), [.1, .2, .3]);
    close(layout.referenceCoordinates.catheterCoordinates.slice(0, 3), [-1, 4, 2]);
    close(layout.referenceCoordinates.catheterCoordinates.slice(9), [7, 2, 1]);
    const restored = decode(layout, layout.referenceCoordinates);
    close(restored.wire, wire.positions); close(restored.catheter, catheter.positions);
    const dw = wire.positions.map((_, i) => Math.sin(i) * .03), dc = catheter.positions.map((_, i) => Math.cos(i) * .02);
    const increment = decode(layout, encode(layout, dw, dc));
    close(increment.wire, dw); close(increment.catheter, dc);
    const translated = { axis: Float64Array.from(layout.referenceCoordinates.axis, (v, i) => v + [2, -3, 4][i % 3]),
        catheterCoordinates: layout.referenceCoordinates.catheterCoordinates };
    const moved = decode(layout, translated);
    close(moved.catheter.slice(3, 6), [.6 + 2, 1.2 - 3, .3 + 4]);
    close(moved.catheter.slice(0, 3), catheter.positions.slice(0, 3));
    restored.wire.fill(99); restored.catheter.fill(99);
    close(decode(layout, layout.referenceCoordinates).catheter, catheter.positions);
});

test('disjoint, empty and single-point tool ranges retain independent coordinates', () => {
    for (const wire of [rod([]), rod([10, 11]), rod([10])]) {
        const catheter = rod([0, 1]); const layout = build({ wire, catheter });
        assert.equal(layout.overlap, null);
        assert.ok(layout.sites.every(s => !s.wireSlots.length));
        close(decode(layout, layout.referenceCoordinates).catheter, catheter.positions);
    }
    const layout = build({ wire: rod([1], [2, 3, 4]), catheter: rod([0, 1, 2]) });
    assert.deepEqual(layout.overlap, [1, 1]);
    assert.deepEqual(layout.sites[1].wireSlots, [0]);
    close(decode(layout, layout.referenceCoordinates).catheter, [0, 0, 0, 1, 0, 0, 2, 0, 0]);
    assert.equal(build({ wire: rod([0, 1]), catheter: rod([]) }).sites.length, 0);
});

test('reject invalid coordinates, identity collisions, strides and alignment overflow', () => {
    const good = rod([0, 1]);
    for (const bad of [rod([0, 0]), rod([1, 0]), rod([0, NaN]), rod([0, Infinity]),
        { ...good, positions: [0, 0] }, { ...good, nodeIndices: [1, 1] },
        { ...good, materialIds: ['same', 'same'] }, { ...good, nodeIndices: [-1, 2] }])
        assert.throws(() => build({ wire: bad, catheter: good }));
    assert.throws(() => build({ wire: good, catheter: good, axialOffsets: [Infinity, 0] }));
    assert.throws(() => build({ wire: good, catheter: good, axialOffsets: [1e30, 0] }));
    const layout = build({ wire: good, catheter: good });
    assert.throws(() => encode(layout, [0], good.positions));
    assert.throws(() => decode(layout, { axis: good.positions, catheterCoordinates: [NaN, 0, 0, 0, 0, 0] }));
});

const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const pairedDot = (a, b) => dot(a.axis, b.axis) + dot(a.catheterCoordinates, b.catheterCoordinates);

test('force pullback preserves virtual work for curved overlap and independent exterior nodes', () => {
    const layout = build({ wire: rod([0, 2, 5], [1, 0, 2, 3, 2, 1, 4, 1, 0]), catheter: rod([-1, 1, 3, 6]) });
    const fw = Array.from({ length: 9 }, (_, i) => Math.sin(i + .3));
    const fc = Array.from({ length: 12 }, (_, i) => Math.cos(i + .4));
    const dw = fw.map((_, i) => Math.cos(i + 2));
    const dc = fc.map((_, i) => Math.sin(i + 3));
    const generalizedForce = pullback(layout, fw, fc), generalizedMotion = encode(layout, dw, dc);
    close([pairedDot(generalizedForce, generalizedMotion)], [dot(fw, dw) + dot(fc, dc)]);
    assert.deepEqual(Array.from(generalizedForce.catheterCoordinates), fc);
});

test('exact mobility reconstructs original diagonal response with unequal masses and pinned axes', () => {
    const layout = build({ wire: rod([0, 2, 5]), catheter: rod([-1, 1, 3, 6]) });
    const fw = Array.from({ length: 9 }, (_, i) => (i - 2) / 7);
    const fc = Array.from({ length: 12 }, (_, i) => (3 - i) / 9);
    const wi = fw.map((_, i) => i % 3 ? (i + 1) / 8 : 0);
    const wc = fc.map((_, i) => i % 4 ? (i + 2) / 5 : 0);
    const g = pullback(layout, fw, fc), before = structuredClone({ g, wi, wc });
    const response = decode(layout, mobility(layout, g, wi, wc));
    close(response.wire, fw.map((v, i) => v * wi[i]));
    close(response.catheter, fc.map((v, i) => v * wc[i]));
    assert.deepEqual({ g, wi, wc }, before);
    const zero = mobility(layout, g, wi.map(() => 0), wc.map(() => 0));
    close(zero.axis, wi.map(() => 0)); close(zero.catheterCoordinates, wc.map(() => 0));
    assert.throws(() => mobility(layout, g, wi.map(() => -1), wc), /nonnegative/);
    assert.throws(() => mobility(layout, g, [1], wc));
});

test('shared interpolation node creates symmetric offdiagonal relative mobility', () => {
    const layout = build({ wire: rod([0, 2]), catheter: rod([.5, 1.5]) });
    const wi = [2, 0, 0, 4, 0, 0], wc = [3, 0, 0, 5, 0, 0];
    const a = { axis: new Float64Array(6), catheterCoordinates: Float64Array.of(1, 0, 0, 0, 0, 0) };
    const b = { axis: new Float64Array(6), catheterCoordinates: Float64Array.of(0, 0, 0, 1, 0, 0) };
    const ma = mobility(layout, a, wi, wc), mb = mobility(layout, b, wi, wc);
    // B1 Wwire B0^T = .25*2*.75 + .75*4*.25 = 1.125.
    close([ma.catheterCoordinates[3], mb.catheterCoordinates[0]], [1.125, 1.125]);
    close(ma.axis, [-1.5, 0, -0, -1, 0, -0]);
    close([pairedDot(a, mb)], [pairedDot(b, ma)]);
    assert.ok(pairedDot(a, ma) > 0);
    const full = decode(layout, ma);
    close(full.catheter, [3, 0, 0, 0, 0, 0]);
});
