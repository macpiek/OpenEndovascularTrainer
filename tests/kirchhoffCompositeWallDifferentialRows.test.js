import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { createCompositeChainLayout, createCompositeChainWorkspace } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeWallWorkspace, refreshCompositeWallContacts, measureCompositeWallConstraints,
    assembleCompositeWallAugmented, canonicalizeCompositeWallReactions } from '../src/physics/kirchhoffCompositeWallContacts.js';
import { createCompositeWallEnvelopeWorkspace, refreshCompositeWallEnvelope, canonicalizeCompositeWallEnvelope } from '../src/physics/kirchhoffCompositeWallEnvelope.js';
import { differentiateCompositeWallRow, requireCompositeWallDifferentialRows } from '../src/physics/kirchhoffCompositeWallDifferentialRows.js';
import { createCompositeWallGeometryWorkspace, differentiateCompositeWallContact } from '../src/physics/kirchhoffCompositeWallGeometry.js';
import { VesselContactField, createContactResult } from '../src/physics/collision/vesselContactField.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';

const close = (a, b, t = 1e-10) => assert.ok(Math.abs(a - b) <= t * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const vectorClose = (a, b, t) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], t)); };
const tolerance = { penalty: 10, gapTolerance: 1e-9, forceTolerance: 1e-9, workTolerance: 1e-9 };
const witness = [[65.00287246704102, -462.00980948623305, -79.56869888305664], [64.95548751831055, -461.7916869276393, -79.65612350463867]];
function ownership(layout, radius) { return { edges: layout.edgeToolIds.map((_, edge) => ({ edge, wall: { owner: 'wire', radius } })) }; }
function raw(contact) { return { signedDistance: contact.signedDistance, signedGap: contact.signedGap, source: contact.source,
    segmentT: contact.segmentT, capsuleSampleCount: contact.capsuleSampleCount, inward: { values: Array.from(contact.inward.values) } }; }
function plane(source = 'analytic-plane', height = 0) {
    return { calls: 0, queryCapsuleCoordinates(ax, ay, az, bx, by, bz, radius, out) {
        this.calls++; const t = ay === by ? .5 : ay < by ? 0 : 1;
        out.signedDistance = (1 - t) * ay + t * by - height; out.signedGap = out.signedDistance - radius;
        out.segmentT = t; out.inward.values.set([0, 1, 0]); out.closestPoint.values.set([(1 - t) * ax + t * bx, height, (1 - t) * az + t * bz]);
        out.source = source; out.faceIndex = 1; out.branchId = 0; out.capsuleSampleCount = 2; return out;
    } };
}
function planeEnvelope() {
    const layout = createCompositeChainLayout([['wire'], ['wire']]), positions = [[0, 1, 0], [2, 1, 0], [5, 1, 0]], contactOwners = ownership(layout, 1), field = plane();
    const w = refreshCompositeWallEnvelope({ positions, contactOwners, field }, createCompositeWallEnvelopeWorkspace(layout));
    return { w, layout, positions, contactOwners, field };
}
// At y=.125 the unsigned trilinear polynomial is constant along x, and all
// physical unit normals are identical. G and DB still vary with x, providing
// an actual nonlinear counterexample to normal/gap-only dependence tests.
function sparseField() {
    const value = ([x, y]) => 2 + y + .5 * (x - .125) * (y - .125);
    const gradient = ([x, y]) => [.5 * (y - .125), 1 + .5 * (x - .125), 0];
    const field = { sdfOrigin: [0, 0, 0], sdfDimensions: [1, 1, 1], brickSize: 2, voxelSize: .5,
        sdfQuantization: 1 / 1024, sdfBrickLookup: new Uint16Array([0]), sdfDistances: new Uint32Array(8), calls: 0 };
    for (let i = 0; i < 8; i++) field.sdfDistances[i] = value([.5 * (i & 1), .5 * ((i >> 1) & 1), .5 * ((i >> 2) & 1)]) * 1024;
    field.queryCapsuleCoordinates = function(ax, ay, az, bx, by, bz, radius, out) {
        this.calls++; const a = [ax, ay, az], b = [bx, by, bz]; let chosen = .5, best = Infinity;
        for (const f of [.5, 0, 1]) { const p = a.map((v, i) => v + f * (b[i] - v)), d = value(p); if (d < best) { best = d; chosen = f; } }
        const p = a.map((v, i) => v + chosen * (b[i] - v)), g = gradient(p), m = Math.hypot(...g);
        out.signedDistance = best; out.signedGap = best - radius; out.source = 'sparse-sdf'; out.segmentT = chosen; out.capsuleSampleCount = 2;
        out.inward.values.set(g.map(v => v / m)); out.closestPoint.values.set(p.map((v, i) => v - best * out.inward.values[i])); return out;
    };
    return field;
}
function sparseEnvelope() {
    const layout = createCompositeChainLayout([['wire']]), positions = [[.125, .125, .125], [.375, .125, .125]], contactOwners = ownership(layout, 2.2), field = sparseField();
    const w = refreshCompositeWallEnvelope({ positions, contactOwners, field }, createCompositeWallEnvelopeWorkspace(layout));
    return { w, layout, positions, contactOwners, field };
}

test('frozen anatomy P1 row retains raw query and true G != B / DB, with no extra detection and unchanged physical Fn', () => {
    const bytes = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
    const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)), field = new VesselContactField(asset), radius = .4445;
    const originalContact = field.queryCapsuleCoordinates(...witness[0], ...witness[1], radius, createContactResult()), expectedRaw = raw(originalContact);
    const expected = differentiateCompositeWallContact({ field, positions: witness, radius, contact: originalContact }, createCompositeWallGeometryWorkspace(2));
    assert.ok(expected.supported, expected.reason);
    const query = field.queryCapsuleCoordinates.bind(field); let calls = 0;
    field.queryCapsuleCoordinates = (...args) => { calls++; return query(...args); };
    const layout = createCompositeChainLayout([['wire']]), contactOwners = ownership(layout, radius), lambdas = new Float64Array([2]);
    const w = refreshCompositeWallContacts({ positions: witness, contactOwners, field, lambdas }, createCompositeWallWorkspace(layout)), row = w.rows[0];
    assert.equal(calls, 1); assert.equal(w.queries, 1); assert.deepEqual(raw(row.rawContact), expectedRaw);
    assert.equal(row.signedDistance, expectedRaw.signedDistance); assert.equal(row.signedGap, expectedRaw.signedGap);
    assert.equal(row.querySegmentT, expectedRaw.segmentT); assert.equal(row.sampleCount, expectedRaw.capsuleSampleCount); assert.equal(row.source, expectedRaw.source);
    vectorClose(row.gapJacobian, expected.gapGradient, 0); vectorClose(row.forceColumn, Array.from(expected.normalForceColumn, v => -v), 0);
    vectorClose(row.normalDerivative, expected.normalForceJacobian, 0);
    assert.ok(Math.abs(row.gapJacobian[3] + row.forceColumn[3]) > .4);
    assert.ok(row.normalDerivative.some((v, i) => Math.abs(v - row.normalDerivative[6 * (i % 6) + Math.floor(i / 6)]) > 1));
    field.querySphere = () => { throw new Error('No derivative query is authorized'); };
    differentiateCompositeWallRow({ field, positions: witness, row }, w.differentialScratch); assert.equal(calls, 1);
    const measured = measureCompositeWallConstraints(w, { ...tolerance, lambdas });
    const force = [0, 1, 2].map(axis => measured.physicalGradient[layout.positions[0] + axis] + measured.physicalGradient[layout.positions[1] + axis]);
    close(Math.hypot(...force), 2); vectorClose(force, row.normal.map(v => -2 * v)); assert.deepEqual(lambdas, new Float64Array([2]));
    const chain = createCompositeChainWorkspace(layout); const saved = [chain.energy, chain.gradient.slice(), chain.hessian.slice()];
    assert.throws(() => assembleCompositeWallAugmented(w, chain, { lambdas, penalty: 10 }), /Physical-normal AL.*inconsistent potential.*sparse-SDF/);
    assert.equal(chain.energy, saved[0]); assert.deepEqual(chain.gradient, saved[1]); assert.deepEqual(chain.hessian, saved[2]);
});

test('endpoint envelope uses point derivatives and scatters their 3x3 DB into the correct six-DOF block', () => {
    const f = sparseEnvelope(); assert.equal(f.field.calls, 3);
    const savedRaw = f.w.rows.map(row => raw(row.rawContact));
    for (const [end, row] of f.w.rows.slice(0, 2).entries()) {
        assert.equal(row.t, end); assert.equal(row.rawContact.segmentT, .5); assert.equal(row.querySegmentT, .5);
        const point = differentiateCompositeWallContact({ field: f.field, positions: [f.positions[end]], radius: row.radius, contact: row.rawContact }, createCompositeWallGeometryWorkspace(1));
        assert.ok(point.supported, point.reason); assert.equal(row.derivativeUnavailable, false);
        for (let i = 0; i < 6; i++) {
            const active = Math.floor(i / 3) === end;
            close(row.gapJacobian[i], active ? point.gapGradient[i % 3] : 0, 0);
            close(row.forceColumn[i], active ? -point.normalForceColumn[i % 3] : 0, 0);
            for (let j = 0; j < 6; j++) close(row.normalDerivative[6 * i + j], active && Math.floor(j / 3) === end ? point.normalForceJacobian[3 * (i % 3) + j % 3] : 0, 0);
        }
        assert.ok(row.normalDerivative.some(v => v !== 0));
    }
    requireCompositeWallDifferentialRows(f.w, [1, 2, 3]); assert.equal(f.field.calls, 3);
    assert.deepEqual(f.w.rows.map(row => raw(row.rawContact)), savedRaw);
});

test('endpoint six-DOF gap and physical-force derivatives match independent finite differences', () => {
    const f = sparseEnvelope(), epsilon = 1e-6, normalForce = 2;
    const baseline = f.w.rows.slice(0, 2).map(row => ({ G: row.gapJacobian.slice(), DB: row.normalDerivative.slice() }));
    const at = positions => {
        const w = refreshCompositeWallEnvelope({ ...f, positions }, createCompositeWallEnvelopeWorkspace(f.layout));
        return w.rows.slice(0, 2).map(row => ({ gap: row.gap, force: Float64Array.from(row.forceColumn, v => normalForce * v) }));
    };
    for (let col = 0; col < 6; col++) {
        const move = delta => f.positions.map((p, node) => p.map((v, axis) => v + (3 * node + axis === col ? delta : 0)));
        const plus = at(move(epsilon)), minus = at(move(-epsilon));
        for (let end = 0; end < 2; end++) {
            close(baseline[end].G[col], (plus[end].gap - minus[end].gap) / (2 * epsilon), 1e-8);
            for (let row = 0; row < 6; row++) close(-normalForce * baseline[end].DB[6 * row + col],
                (plus[end].force[row] - minus[end].force[row]) / (2 * epsilon), 1e-8);
        }
    }
});

test('equal endpoint normal/gap/closest witness does not merge differing globally scattered G, B, DB or source', () => {
    for (const alter of [row => { row.gapJacobian[0] += 1e-15; }, row => { row.forceColumn[0] += 1e-15; },
        row => { row.normalDerivative[0] += 1e-15; }, row => { row.source = row.derivativeSource = 'sparse-sdf'; }]) {
        const f = planeEnvelope(), lambda = Float64Array.from([0, 2, 0, 3, 0, 0]), original = lambda.slice();
        assert.deepEqual(f.w.rows[1].normal, f.w.rows[3].normal); assert.equal(f.w.rows[1].gap, f.w.rows[3].gap);
        assert.deepEqual(f.w.rows[1].closestPoint, f.w.rows[3].closestPoint); alter(f.w.rows[3]);
        const representatives = canonicalizeCompositeWallReactions(f.w, lambda, f.contactOwners);
        assert.ok(representatives.includes(1) && representatives.includes(3)); assert.deepEqual(lambda, original);
    }
});

test('exact analytic-plane dependence still transfers physical force and torque to the unique endpoints', () => {
    const f = planeEnvelope(), lambda = Float64Array.from([1, 2, 6, 3, 4, 10]), before = measureCompositeWallConstraints(f.w, { ...tolerance, lambdas: lambda }).physicalGradient.slice();
    const rows = f.w.rows.map(row => ({ gap: row.gap, raw: raw(row.rawContact), G: row.gapJacobian.slice(), B: row.forceColumn.slice(), DB: row.normalDerivative.slice() }));
    const reps = canonicalizeCompositeWallEnvelope(f.w, lambda, f.contactOwners);
    assert.deepEqual(reps, [0, 1, 4]); assert.deepEqual(Array.from(lambda), [4, 13, 0, 0, 9, 0]); close(lambda.reduce((a, b) => a + b, 0), 26);
    const after = measureCompositeWallConstraints(f.w, { ...tolerance, lambdas: lambda }).physicalGradient;
    assert.deepEqual(after, before);
    const moment = gradient => f.positions.reduce((sum, p, node) => {
        const force = Array.from(gradient.subarray(f.layout.positions[node], f.layout.positions[node] + 3));
        const torque = [p[1] * force[2] - p[2] * force[1], p[2] * force[0] - p[0] * force[2], p[0] * force[1] - p[1] * force[0]];
        return sum.map((v, i) => v + torque[i]);
    }, [0, 0, 0]);
    vectorClose(moment(after), moment(before), 0); assert.equal(f.field.calls, 5);
    f.w.rows.forEach((row, i) => { assert.equal(row.gap, rows[i].gap); assert.deepEqual(raw(row.rawContact), rows[i].raw); assert.deepEqual(row.gapJacobian, rows[i].G);
        assert.deepEqual(row.forceColumn, rows[i].B); assert.deepEqual(row.normalDerivative, rows[i].DB); });
});

test('a real nonlinear sparse polynomial with flat gap and identical unit normals is not reduced to endpoint rows', () => {
    const f = sparseEnvelope(), lambda = Float64Array.from([1, 2, 6]);
    assert.equal(f.w.rows[0].gap, f.w.rows[2].gap); assert.equal(f.w.rows[1].gap, f.w.rows[2].gap);
    assert.deepEqual(f.w.rows[0].normal, f.w.rows[2].normal); assert.deepEqual(f.w.rows[1].normal, f.w.rows[2].normal);
    const before = lambda.slice(), reps = canonicalizeCompositeWallEnvelope(f.w, lambda, f.contactOwners);
    assert.ok(reps.includes(2)); assert.deepEqual(lambda, before);
    assert.ok(f.w.rows[2].normalDerivative.slice(0, 18).some((v, i) => i % 6 >= 3 && v !== 0), 'capsule DB has genuine cross-endpoint entries');
});

test('DB mismatch alone prevents interior reduction even when g/G/B are exactly their endpoint combination', () => {
    const f = planeEnvelope(), lambda = Float64Array.from([1, 2, 6, 0, 0, 0]), c = f.w.rows[2];
    c.normalDerivative[3] = 1e-15;
    const reps = canonicalizeCompositeWallEnvelope(f.w, lambda, f.contactOwners);
    assert.ok(reps.includes(2)); assert.equal(lambda[2], 6);
});

test('unsupported active or loaded rows reject; open zero-force rows retain their original gap and unknown derivatives', () => {
    const layout = createCompositeChainLayout([['wire']]), contactOwners = ownership(layout, 1), positions = [[0, 2, 0], [2, 2, 0]], field = plane('centerline-safe-core');
    const w = refreshCompositeWallContacts({ positions, contactOwners, field, lambdas: [0] }, createCompositeWallWorkspace(layout)), row = w.rows[0];
    assert.equal(row.gap, 1); assert.equal(row.derivativeUnavailable, true); assert.equal(row.inactiveForSolve, true); assert.equal(row.deltaNormalForce, 0);
    assert.ok(row.gapJacobian.every(Number.isNaN)); assert.ok(row.forceColumn.every(Number.isNaN)); assert.ok(row.normalDerivative.every(Number.isNaN));
    const measured = measureCompositeWallConstraints(w, { ...tolerance, lambdas: [0] }); assert.equal(measured.converged, true); assert.ok(measured.physicalGradient.every(v => v === 0));
    assert.throws(() => requireCompositeWallDifferentialRows(w, [1]), /Unsupported active\/loaded/);
    assert.throws(() => canonicalizeCompositeWallReactions(w, new Float64Array([1]), contactOwners), /Unsupported active\/loaded/);
    for (const y of [1, .5]) {
        const q = positions.map(p => [p[0], y, p[2]]);
        assert.throws(() => refreshCompositeWallContacts({ positions: q, contactOwners, field, lambdas: [0] }, w), /Unsupported active\/loaded/);
    }
});

test('unknown unloaded endpoint derivatives never create false duplicate or interior dependencies', () => {
    const f = planeEnvelope(); f.positions.forEach(p => { p[1] = 2; }); const field = plane('fallback');
    refreshCompositeWallEnvelope({ ...f, field, lambdas: new Float64Array(6) }, f.w);
    const lambda = new Float64Array(6), reps = canonicalizeCompositeWallEnvelope(f.w, lambda, f.contactOwners);
    assert.deepEqual(reps, [0, 1, 2, 3, 4, 5]); assert.ok(f.w.rows.every(row => row.derivativeUnavailable && row.inactiveForSolve && row.deltaNormalForce === 0));
    assert.ok(f.w.rows.every(row => row.gap === 1)); assert.equal(field.calls, 5);
});

test('refresh clears previously supported derivative buffers when the query switches to an unsupported source', () => {
    const f = planeEnvelope(); assert.ok(f.w.rows.every(row => !row.derivativeUnavailable));
    const buffers = f.w.rows.map(row => [row.dofs, row.gapJacobian, row.forceColumn, row.normalDerivative]);
    f.positions.forEach(p => { p[1] = 2; });
    refreshCompositeWallEnvelope({ ...f, field: plane('fallback'), lambdas: new Float64Array(6) }, f.w);
    for (const [i, row] of f.w.rows.entries()) {
        assert.ok(row.gapJacobian.every(Number.isNaN)); assert.ok(row.forceColumn.every(Number.isNaN)); assert.ok(row.normalDerivative.every(Number.isNaN));
        assert.equal(row.rawContact.source, 'fallback'); assert.equal(row.signedGap, 1);
        [row.dofs, row.gapJacobian, row.forceColumn, row.normalDerivative].forEach((buffer, j) => assert.equal(buffer, buffers[i][j]));
    }
});
