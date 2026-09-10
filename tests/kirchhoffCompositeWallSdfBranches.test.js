import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { createCompositeWallSdfBranchesWorkspace, evaluateCompositeWallSdfBranches,
    measureCompositeWallSdfBranches, findCompositeWallSdfSeamCrossing } from '../src/physics/kirchhoffCompositeWallSdfBranches.js';
import { VesselContactField, createContactResult } from '../src/physics/collision/vesselContactField.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import {createCompositeAnatomyField} from './helpers/compositeAnatomyField.js';

const near = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const same = (a, b, eps) => { assert.equal(a.length, b.length); a.forEach((v, i) => near(v, b[i], eps)); };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);

test('captured step361 mixed-occupancy face admits a locally proved inside min intersection',()=>{
    const report=JSON.parse(fs.readFileSync(new URL('../reports/composite-material-point-rejected-step361.json',import.meta.url),'utf8')),
        origin=JSON.parse(fs.readFileSync(new URL('./fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
        p=new Map(report.result.diagnostics.rejectedConfiguration.positions).get('wire')[39].map((v,k)=>v+origin[k]),
        radius=JSON.parse(report.result.diagnostics.certificate.wall.samples.find(s=>s.edge===38).key)[2],anatomy=createCompositeAnatomyField();
    try {
        const field=anatomy.field,raw=field.queryCapsuleCoordinates(...p,...p,radius,createContactResult()),
            args={field,face:{axis:2,gridIndex:215},positions:[p],radius,contact:raw};
        assert.equal(evaluatePoint(args).reason,'nonuniform-or-exterior-sign-branch');
        const domainBox={lower:p.map(v=>v-.02),upper:p.map(v=>v+.02)},out=evaluatePoint({...args,domainBox});
        assert.equal(out.supported,true,out.reason);assert.equal(out.localInsideProof.supported,true);
        assert.equal(out.classification,'min-intersection');assert.ok(out.jumpLowerBound>.13);
        near(Math.min(...out.rows.map(r=>r.gap)),raw.signedGap,1e-12);
        const original=field.certifyInsideBallCoordinates;field.certifyInsideBallCoordinates=()=>({supported:false});
        assert.equal(evaluatePoint({...args,domainBox}).reason,'nonuniform-or-exterior-sign-branch');
        field.certifyInsideBallCoordinates=original;
    } finally {anatomy.dispose();}
});

// Independent analytic polynomials, sampled into quantized storage below.
function model(p, side, kind = 'min') {
    const [x, y, z] = p;
    let s, sy, sz, syz;
    if (kind === 'mixed') { s = side === 0 ? y - .5 : 0; sy = side === 0 ? 1 : 0; sz = 0; syz = 0; }
    else if (kind === 'zero') { s = side === 0 ? .5 : 0; sy = sz = syz = 0; }
    else if (kind === 'coincident') { s = side === 0 ? 1 : .5; sy = sz = syz = 0; }
    else if (kind === 'opposite') { s = side === 0 ? 1 : -1; sy = sz = syz = 0; }
    else {
        const lower = kind === 'smooth' || (kind === 'max' ? side === 1 : side === 0);
        s = lower ? .5 + .125 * y + .25 * z + .0625 * y * z : -.375 + .0625 * y - .125 * z - .03125 * y * z;
        sy = lower ? .125 + .0625 * z : .0625 - .03125 * z;
        sz = lower ? .25 + .0625 * y : -.125 - .03125 * y;
        syz = lower ? .0625 : -.03125;
    }
    const flat = ['zero', 'coincident', 'opposite'].includes(kind);
    const base = flat ? 4 : 4 + .25 * y + .5 * z + .125 * y * z;
    const G = [s, (flat ? 0 : .25 + .125 * z) + (x - 1) * sy, (flat ? 0 : .5 + .125 * y) + (x - 1) * sz];
    const H = [0, sy, sz, sy, 0, (flat ? 0 : .125) + (x - 1) * syz, sz, (flat ? 0 : .125) + (x - 1) * syz, 0];
    return { value: base + (x - 1) * s, G, H, n: G.map(v => v / norm(G)) };
}
function grid(kind = 'min', axis = 0) {
    const order = [axis, ...[0, 1, 2].filter(i => i !== axis)], size = 2, dimensions = [2, 2, 2], quantization = 1 / 1024;
    const distances = new Uint32Array(64), lookup = Uint16Array.from({ length: 8 }, (_, i) => i);
    for (let bz = 0; bz < 2; bz++) for (let by = 0; by < 2; by++) for (let bx = 0; bx < 2; bx++) {
        const brick = bx + 2 * (by + 2 * bz);
        for (let z = 0; z < size; z++) for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
            const world = [2 * bx + x, 2 * by + y, 2 * bz + z], p = order.map(i => world[i]);
            const encoded = model(p, p[0] < 1 ? 0 : 1, kind).value / quantization;
            assert.equal(encoded, Math.round(encoded)); assert.ok(encoded >= 0);
            distances[brick * 8 + x + 2 * (y + 2 * z)] = encoded;
        }
    }
    return { sdfOrigin: [0, 0, 0], sdfDimensions: dimensions, brickSize: size, voxelSize: 1,
        sdfQuantization: quantization, sdfBrickLookup: lookup, sdfDistances: distances, sdfInsideBits: new Uint8Array(8).fill(255),
        querySphere() { throw Error('Unexpected point query'); }, queryCapsuleCoordinates() { throw Error('Unexpected capsule query'); } };
}
function contact(p, radius, kind = 'min', t = 0, count = 0, axis = 0) {
    const order = [axis, ...[0, 1, 2].filter(i => i !== axis)], local = order.map(i => p[i]);
    const g = model(local, local[0] < 1 ? 0 : 1, kind), n = Array(3);
    order.forEach((i, k) => n[i] = g.n[k]);
    return { signedGap: g.value - radius, signedDistance: g.value, source: 'sparse-sdf', segmentT: t, capsuleSampleCount: count,
        inward: { values: Float64Array.from(n) } };
}
function pointArgs(p = [1, .375, .625], kind = 'min', radius = model(p, 1, kind).value, axis = 0) {
    return { field: grid(kind, axis), face: { axis, gridIndex: 1 }, positions: [p], radius, contact: contact(p, radius, kind, 0, 0, axis) };
}
const evaluatePoint = args => evaluateCompositeWallSdfBranches(args, createCompositeWallSdfBranchesWorkspace(1));
const tolerances = { penalty: 100, gapTolerance: 1e-8, forceTolerance: 1e-7, workTolerance: 1e-7 };

test('one shared face proves a min chart; exact limits match independent analytic polynomials on every axis', () => {
    for (const axis of [0, 1, 2]) {
        const order = [axis, ...[0, 1, 2].filter(i => i !== axis)], local = [1, .375, .625], p = Array(3);
        order.forEach((i, k) => p[i] = local[k]);
        const args = pointArgs(p, 'min', model(local, 1).value, axis), input = structuredClone({ p, raw: args.contact });
        const out = evaluatePoint(args);
        assert.ok(out.supported, out.reason); assert.equal(out.classification, 'min-intersection');
        assert.equal(out.continuity, true); assert.equal(out.onSeam, true); assert.equal(out.queryCount, 0);
        assert.ok(out.jumpLowerBound > 0); assert.equal(out.branchSign, 1);
        for (const row of out.rows) {
            const oracle = model(local, row.index);
            near(row.gap, 0, 1e-14); same(order.map(i => row.pointGapGradient[i]), oracle.G, 1e-14);
            same(order.flatMap(i => order.map(j => row.pointGapHessian[3 * i + j])), oracle.H, 1e-14);
            same(order.map(i => row.normal[i]), oracle.n, 1e-14);
            assert.equal(row.inOriginalCell, true); assert.equal(row.enabled, true);
        }
        assert.deepEqual({ p, raw: args.contact }, input);
    }
});

test('G and DB match independent one-sided finite differences for both capsule endpoints', () => {
    const positions = [[.75, .25, .5], [1.75, .75, 1]], t = .25, weights = [.75, .25];
    const p = positions[0].map((v, i) => v + t * (positions[1][i] - v)), radius = model(p, 1).value;
    const out = evaluateCompositeWallSdfBranches({ field: grid(), face: { axis: 0, gridIndex: 1 }, positions,
        radius, contact: contact(p, radius, 'min', t, 4), dofs: [4, 5, 6, 8, 9, 10] }, createCompositeWallSdfBranchesWorkspace());
    assert.ok(out.supported, out.reason); same(out.weights, weights); same(out.rows[0].dofs, [4, 5, 6, 8, 9, 10]);
    const epsilon = 2e-5;
    for (const row of out.rows) for (let j = 0; j < 6; j++) {
        const axis = j % 3, w = weights[Math.floor(j / 3)], side = row.index === 0 ? -1 : 1;
        function sample(s) { return model(p.map((v, i) => v + (i === axis ? s * w : 0)), row.index); }
        const base = sample(0), one = sample(side * epsilon), two = sample(side * 2 * epsilon);
        const fdGap = side * (-3 * base.value + 4 * one.value - two.value) / (2 * epsilon);
        near(row.gapJacobian[j], fdGap, 2e-8);
        for (let i = 0; i < 6; i++) {
            const a = i % 3, wi = weights[Math.floor(i / 3)];
            const fdB = wi * side * (-3 * base.n[a] + 4 * one.n[a] - two.n[a]) / (2 * epsilon);
            near(row.normalDerivative[6 * i + j], fdB, 3e-8);
            near(row.forceColumn[i], -row.normalForceColumn[i], 1e-14);
        }
    }
    assert.ok(out.rows.some(row => row.normalDerivative.some((v, i) => Math.abs(v - row.normalDerivative[6 * (i % 6) + Math.floor(i / 6)]) > 1e-4)));
    assert.ok(Math.abs(out.rows[0].gapJacobian[0] - out.rows[0].normalForceColumn[0]) > .01);
});

test('two physical reactions preserve the resultant, nodal support and moment about one origin', () => {
    const positions = [[.75, .25, .5], [1.75, .75, 1]], p = [1, .375, .625], radius = model(p, 1).value;
    const out = evaluateCompositeWallSdfBranches({ field: grid(), face: { axis: 0, gridIndex: 1 }, positions, radius,
        contact: contact(p, radius, 'min', .25, 4) }, createCompositeWallSdfBranchesWorkspace());
    const about = [.125, -.25, 1.5], forces = [1.2, .4], proof = measureCompositeWallSdfBranches(out, { ...tolerances, forces, about });
    assert.ok(proof.converged); assert.equal(proof.scope, 'discrete-contact-cone-only');
    const expected = [0, 1, 2].map(i => forces.reduce((sum, f, k) => sum + f * model(p, k).n[i], 0));
    same(proof.resultant, expected); same(proof.physicalGradient, [...expected.map(v => -.75 * v), ...expected.map(v => -.25 * v)]);
    const moments = positions.map((p, node) => cross(p.map((v, i) => v - about[i]), [...proof.nodalForces.slice(3 * node, 3 * node + 3)]));
    same(proof.moment, moments[0].map((v, i) => v + moments[1][i]));
    same(proof.moment, cross(p.map((v, i) => v - about[i]), expected));
    assert.ok(Math.abs(proof.scalarForceSum - proof.forceMagnitude) > .1);
    const negative = measureCompositeWallSdfBranches(out, { ...tolerances, forces: [-1e-14, .4] });
    assert.equal(negative.converged, false); near(negative.maximumNegativeForce, 1e-14, 1e-16);
});

test('wrong-domain force is rejected away from the exact tie, even with loose residual tolerances', () => {
    const p = [.8, .375, .625], out = evaluatePoint(pointArgs(p));
    assert.ok(out.supported); assert.equal(out.onSeam, false); assert.equal(out.selectedRow, 0);
    assert.equal(out.rows[0].inOriginalCell, true); assert.equal(out.rows[1].inOriginalCell, false);
    assert.ok(out.rows[1].gap > out.rows[0].gap); near(out.rawContact.signedGap, out.rows[0].gap);
    const params = { penalty: 1, gapTolerance: 100, forceTolerance: 100, workTolerance: 100 };
    assert.equal(measureCompositeWallSdfBranches(out, { ...params, forces: [1, 0] }).converged, true);
    const bad = measureCompositeWallSdfBranches(out, { ...params, forces: [1, 1e-15] });
    assert.equal(bad.converged, false); assert.equal(bad.domainAdmissible, false); assert.equal(bad.tieAdmissible, false);
});

test('penetration and open-gap complementarity remain explicit; grid seam alone is not an active wall', () => {
    const p = [1, .375, .625], value = model(p, 1).value;
    const open = evaluatePoint(pointArgs(p, 'min', value - .2));
    assert.ok(measureCompositeWallSdfBranches(open, { ...tolerances, forces: [0, 0] }).converged);
    const loaded = measureCompositeWallSdfBranches(open, { ...tolerances, forces: [1, 1] });
    assert.equal(loaded.converged, false); near(loaded.maximumComplementarity, .2);
    const penetrated = evaluatePoint(pointArgs(p, 'min', value + .2));
    const proof = measureCompositeWallSdfBranches(penetrated, { ...tolerances, forces: [1, 1] });
    assert.equal(proof.converged, false); near(proof.maximumPenetration, .2);
});

test('smooth, max/union, mixed order, zero gradient and coincident rays are distinct unsupported classes', () => {
    for (const [kind, reason] of [['smooth', 'smooth-seam-needs-one-branch'], ['max', 'max-union-needs-different-contact-law'],
        ['mixed', 'unresolved-min-max-order'], ['zero', 'zero-fallback-or-nonfinite-branch-gradient'],
        ['coincident', 'coincident-normal-rays-needs-one-force'], ['opposite', 'opposite-dependent-normal-rays']]) {
        const args = pointArgs([1, .375, .625], kind);
        // The zero-gradient provider may use a fallback normal; keep raw finite.
        if (kind === 'zero') args.contact.inward.values.set([0, 1, 0]);
        const out = evaluatePoint(args);
        assert.equal(out.supported, false, kind); assert.equal(out.reason, reason, kind);
        assert.ok(out.rows.every(row => !row.enabled && row.gapJacobian.every(Number.isNaN)));
    }
});

test('missing corners and unproved sign/source or sample provenance never become synthesized provider contacts', () => {
    const cases = [
        [args => args.contact.source = 'sdf-bvh', 'unsupported-source:sdf-bvh'],
        [args => args.contact.signedDistance = -args.contact.signedDistance, 'unresolved-or-exterior-sign-branch'],
        [args => args.contact.signedDistance = 0, 'unresolved-or-exterior-sign-branch'],
        [args => delete args.field.sdfInsideBits, 'missing-inside-sign-proof'],
        [args => args.field.sdfInsideBits[0] = 0, 'nonuniform-or-exterior-sign-branch'],
        [args => args.field.sdfBrickLookup[0] = 0xffff, 'missing-sdf-corner'],
        [args => args.contact.signedGap += .01, 'original-contact-does-not-match-cell-chart'],
        [args => args.contact.inward.values.set([1, 0, 0]), 'original-normal-does-not-match-selected-cell'],
    ];
    for (const [mutate, reason] of cases) {
        const args = pointArgs(); mutate(args); const out = evaluatePoint(args);
        assert.equal(out.supported, false); assert.equal(out.reason, reason);
        assert.throws(() => measureCompositeWallSdfBranches(out, { ...tolerances, forces: [0, 0] }), /unavailable/);
    }
    const args = pointArgs(); args.positions = [args.positions[0], args.positions[0]]; args.contact.segmentT = .3; args.contact.capsuleSampleCount = 2;
    assert.equal(evaluateCompositeWallSdfBranches(args, createCompositeWallSdfBranchesWorkspace()).reason, 'unrecognized-capsule-sampling-branch');
});

test('invalid input bounds throw; chart bounds and multi-axis intersections are explicitly unsupported', () => {
    for (const mutate of [args => args.face.axis = 3, args => args.face.gridIndex = -1, args => args.radius = -1,
        args => args.positions[0][0] = NaN, args => args.field.voxelSize = 0, args => args.dofs = [1, 1, 2]]) {
        const args = pointArgs(); mutate(args); assert.throws(() => evaluatePoint(args));
    }
    assert.throws(() => createCompositeWallSdfBranchesWorkspace(3));
    assert.equal(evaluatePoint(pointArgs([2.25, .375, .625])).reason, 'outside-adjacent-cell-domain');
    assert.equal(evaluatePoint(pointArgs([1, 1, .625])).reason, 'multi-axis-cell-intersection');
    assert.equal(evaluatePoint(pointArgs([0, .375, .625])).reason, 'outside-adjacent-cell-domain');
});

test('reused rows retain buffers, own the raw query, and invalidate previous success on unsupported refresh', () => {
    const ws = createCompositeWallSdfBranchesWorkspace(1), args = pointArgs();
    const refs = ws.rows.map(row => row.normalDerivative);
    evaluateCompositeWallSdfBranches(args, ws); const raw = ws.rawContact.inward.values.slice();
    args.contact.inward.values[0] += .25; same(ws.rawContact.inward.values, raw);
    args.contact.source = 'analytic-plane'; evaluateCompositeWallSdfBranches(args, ws);
    assert.equal(ws.supported, false); ws.rows.forEach((row, i) => { assert.equal(row.normalDerivative, refs[i]); assert.ok(row.normalDerivative.every(Number.isNaN)); });
    const independent = evaluatePoint(pointArgs()); assert.ok(independent.supported); assert.notEqual(independent.rawContact.inward.values, ws.rawContact.inward.values);
    assert.throws(() => measureCompositeWallSdfBranches(independent, { ...tolerances, forces: [1e308, 1e308] }), /Nonfinite/);
});

test('nearest trial crossing reports positive/negative/endpoint events and leaves the trial point unchanged', () => {
    const field = grid(), cases = [
        { position: [.25, .375, .625], delta: [2.25, 0, 0], alpha: 1 / 3, index: 1, direction: 1 },
        { position: [1.25, .375, .625], delta: [-1, 0, 0], alpha: .25, index: 1, direction: -1 },
        { position: [.25, .375, .625], delta: [.75, 0, 0], alpha: 1, index: 1, direction: 1 },
    ];
    for (const c of cases) {
        const original = structuredClone(c), out = findCompositeWallSdfSeamCrossing({ field, ...c });
        assert.ok(out.hit && out.supported); near(out.alpha, c.alpha); near(out.point[0], 1);
        assert.equal(out.events[0].gridIndex, c.index); assert.equal(out.events[0].direction, c.direction);
        assert.deepEqual(c, original); assert.equal(out.startsOnFace, false);
    }
    const none = findCompositeWallSdfSeamCrossing({ field, position: [.25, .375, .625], delta: [0, 0, 0] });
    assert.equal(none.hit, false); assert.equal(none.alpha, null);
});

test('starting-face and simultaneous-axis events cannot hide zero-alpha loops or higher-codimension seams', () => {
    const field = grid();
    const start = findCompositeWallSdfSeamCrossing({ field, position: [1, .375, .625], delta: [0, .1, 0] });
    assert.ok(start.hit && start.startsOnFace); assert.equal(start.alpha, 0); assert.equal(start.events[0].direction, 0);
    const two = findCompositeWallSdfSeamCrossing({ field, position: [.25, .25, .625], delta: [1.5, 1.5, 0] });
    assert.equal(two.hit, true); assert.equal(two.supported, false); assert.equal(two.events.length, 2); near(two.alpha, .5);
    const edge = findCompositeWallSdfSeamCrossing({ field, position: [1, 1, .625], delta: [.1, .2, 0] });
    assert.equal(edge.supported, false); assert.equal(edge.startsOnFace, true);
    assert.throws(() => findCompositeWallSdfSeamCrossing({ field, position: [NaN, 0, 0], delta: [1, 0, 0] }));
    assert.throws(() => findCompositeWallSdfSeamCrossing({ field, position: [-.25, .375, .625], delta: [1, 0, 0] }), /bounds/);
    const outside = findCompositeWallSdfSeamCrossing({ field, position: [.25, .375, .625], delta: [-1, 0, 0] });
    assert.equal(outside.hit, true); assert.equal(outside.supported, false); assert.equal(outside.reason, 'missing-adjacent-grid-domain');
});

test('original Aorta P1 two-face equilibrium retains the raw sparse query and passes the contact cone certificate', () => {
    const bytes = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
    const field = new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)));
    const positions = [[64.97836989399583, -462.02966905032116, -79.55471756170732],
        [65, -461.81336005090407, -79.65575420693476]], radius = .4445;
    const raw = field.queryCapsuleCoordinates(...positions[0], ...positions[1], radius, createContactResult());
    const snapshot = { gap: raw.signedGap, normal: [...raw.inward.values], t: raw.segmentT, source: raw.source };
    const out = evaluateCompositeWallSdfBranches({ field, face: { axis: 0, gridIndex: 434 }, positions, radius, contact: raw }, createCompositeWallSdfBranchesWorkspace());
    assert.ok(out.supported, out.reason); assert.equal(out.onSeam, true); assert.equal(out.sampleFraction, 1); assert.equal(out.sampleCount, 1);
    same(out.rows[0].cell, [433, 60, 48]); same(out.rows[1].cell, [434, 60, 48]);
    same(out.jumpCorners, [.8, .72, 1.16, .92], 1e-14);
    assert.deepEqual({ gap: raw.signedGap, normal: [...raw.inward.values], t: raw.segmentT, source: raw.source }, snapshot);
    same(out.rawContact.inward.values, snapshot.normal); assert.equal(out.rawContact.signedGap, snapshot.gap);
    const proof = measureCompositeWallSdfBranches(out, { ...tolerances, penalty: 1e4, forces: [2.192477870513528, .598007423867116] });
    assert.ok(proof.converged); assert.ok(proof.maximumPenetration < 2e-14); assert.ok(proof.maximumProjectedResidual < 2e-10);
    same(proof.resultant, [1.07679363763326, -1.63945340210970, .50651812538474], 1e-11);
    near(proof.forceMagnitude, 2.02579678326326, 1e-11);
    // The raw provider chose one cell normal; it is not the cone resultant.
    assert.ok(norm(cross([...proof.resultant], snapshot.normal)) > 1);
    const old = [64.95548751831055, -461.7916869276393, -79.65612350463867];
    const crossing = findCompositeWallSdfSeamCrossing({ field, position: old, delta: [.05591625654271809, -.01078068273281739, -.010463195422401641] });
    assert.ok(crossing.hit && crossing.supported); assert.equal(crossing.events[0].axis, 0); assert.equal(crossing.events[0].gridIndex, 434);
});

test('mixed-order cell face admits only a proved local min subdomain',()=>{
    const domainBox={lower:[.8,.6,.2],upper:[1.2,.9,.8]},args=pointArgs([1,.75,.5],'mixed');
    assert.equal(evaluatePoint(args).reason,'unresolved-min-max-order');
    const out=evaluatePoint({...args,domainBox});
    assert.equal(out.supported,true,out.reason);
    near(out.jumpLowerBound,.1);near(out.jumpUpperBound,.4);
    same(out.domain.lower,domainBox.lower);same(out.domain.upper,domainBox.upper);
    for(const x of [.85,1,1.15])for(const y of [.65,.75,.85]) {
        const p=[x,y,.5],a=pointArgs(p,'mixed'),r=evaluatePoint({...a,domainBox});
        assert.equal(r.supported,true,r.reason);
        near(Math.min(...r.rows.map(row=>row.gap)),a.contact.signedGap);
    }
});

test('local seam domain cannot extend the proof beyond its rectangle or cross mixed min/max order',()=>{
    const args=pointArgs([1,.75,.5],'mixed'),domainBox={lower:[.8,.6,.2],upper:[1.2,.9,.8]};
    assert.equal(evaluatePoint({...pointArgs([1,.6,.5],'mixed'),domainBox}).reason,'outside-local-seam-domain');
    assert.equal(evaluatePoint({...args,domainBox:{lower:[.8,.4,.2],upper:[1.2,.9,.8]}}).reason,'unresolved-min-max-order');
    assert.throws(()=>evaluatePoint({...args,domainBox:{lower:[-.1,.6,.2],upper:[1.2,.9,.8]}}),/within/);
    assert.throws(()=>evaluatePoint({...args,domainBox:{lower:[1,.6,.2],upper:[1.2,.9,.8]}}),/straddle/);
    const mixedSign=pointArgs([1,.75,.5],'mixed');mixedSign.field.sdfInsideBits.fill(0);
    assert.equal(evaluatePoint({...mixedSign,domainBox}).reason,'nonuniform-or-exterior-sign-branch');
});
