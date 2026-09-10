import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateKirchhoffLumenSegmentContact } from '../src/physics/kirchhoffLumenContact.js';
import { createCompositeLumenSideGeometryWorkspace, differentiateCompositeLumenSideContact } from '../src/physics/kirchhoffCompositeLumenSideGeometry.js';

const names = ['innerStart', 'innerEnd', 'outerStart', 'outerEnd'];
const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const same = (a, b, eps) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], eps)); };
const norm = a => Math.hypot(...a);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const flattenedGradient = contact => [...contact.gradients.inner.flat(), ...contact.gradients.outer.flat()];

test('needed derivative orders preserve exact original G/B and invalidate unavailable witness derivatives and Hessians',()=>{
    const input=fixture(),contact=evaluateKirchhoffLumenSegmentContact(input).side,ws=createCompositeLumenSideGeometryWorkspace();
    const full=structuredClone(differentiateCompositeLumenSideContact({input,contact},ws));
    assert.equal(full.hessianValid,true);assert.equal(full.witnessJacobianValid,true);
    for(const order of ['gradient','witness']) {
        const r=differentiateCompositeLumenSideContact({input,contact},ws,{order});
        assert.equal(r.supported,true);assert.equal(r.hessianValid,false);
        assert.equal(r.witnessJacobianValid,order==='witness');
        for(const key of ['gapJacobian','normalForceColumn','forceColumn','normal','innerPoint','outerPoint'])assert.deepEqual(r[key],full[key]);
        assert.ok(r.normalDerivative.every(Number.isNaN));assert.ok(r.gapHessian.every(Number.isNaN));
        for(const key of ['normalJacobian','outerTGradient'])if(order==='witness')assert.deepEqual(r[key],full[key]);else assert.ok(r[key].every(Number.isNaN));
    }
    const restored=differentiateCompositeLumenSideContact({input,contact},ws);assert.deepEqual(restored.normalDerivative,full.normalDerivative);
    const stale=structuredClone(contact);stale.outerT+=.01;
    for(const order of ['gradient','witness'])assert.equal(differentiateCompositeLumenSideContact({input,contact:stale},ws,{order}).supported,false);
});
function fixture() {
    return { innerStart: [-.3, 1.1, .5], innerEnd: [2.1, 2.8, .8], outerStart: [-1, .4, .2], outerEnd: [3, 1.7, -.5],
        lumenRadius: 1, innerRadius: .2, innerMaterialSegmentId: 'wire', outerMaterialSegmentId: 'cat',
        quadrature: [.1, .35, .7] };
}
function evaluate(input, workspace = createCompositeLumenSideGeometryWorkspace()) {
    const detection = evaluateKirchhoffLumenSegmentContact(input);
    const geometry = differentiateCompositeLumenSideContact({ input, contact: detection.side }, workspace);
    return { detection, geometry };
}
function perturbed(input, dof, delta) {
    const out = structuredClone(input); out[names[Math.floor(dof / 3)]][dof % 3] += delta; return out;
}
function assertStable(detection, selected) {
    assert.ok(detection.side); assert.equal(detection.side.kind, 'side'); assert.equal(detection.side.innerT, selected);
    assert.ok(detection.side.outerT > 0 && detection.side.outerT < 1);
    for (const sample of detection.samples) if (sample.innerT !== selected)
        assert.ok(sample.gap - detection.side.gap > .1, 'the FD fixture must have a unique separated quadrature winner');
}

test('selected original side data is preserved; the physical 12D column equals the gap gradient with inward wire signs', () => {
    const input = fixture(), detection = evaluateKirchhoffLumenSegmentContact(input);
    const before = structuredClone({ input, contact: detection.side });
    const out = differentiateCompositeLumenSideContact({ input, contact: detection.side }, createCompositeLumenSideGeometryWorkspace());
    assert.ok(out.supported, out.reason); assertStable(detection, .7);
    assert.equal(out.dofCount, 12); same(out.dofs, Array.from({ length: 12 }, (_, i) => i));
    assert.equal(out.gap, detection.side.gap); assert.equal(out.innerT, detection.side.innerT); assert.equal(out.outerT, detection.side.outerT);
    same(out.normal, detection.side.normal, 0); same(out.innerWeights, detection.side.innerWeights, 0); same(out.outerWeights, detection.side.outerWeights, 0);
    same(out.gapJacobian, flattenedGradient(detection.side), 0); same(out.normalForceColumn, out.gapJacobian, 0);
    same(out.forceColumn, Array.from(out.gapJacobian, v => -v), 0); same(out.normalDerivative, out.gapHessian, 0);
    assert.equal(out.rawContact.id, detection.side.id); assert.equal(out.rawContact.kind, 'side');
    assert.equal(out.selectionCertified, false); assert.equal(out.certified, false); assert.equal(out.queryCount, 0);
    assert.match(out.derivativeScope, /fixed-inner-quadrature/);
    assert.deepEqual({ input, contact: detection.side }, before);
});

test('all 12 columns G, outerT and DB match independent finite differences of the original detector', () => {
    const input = fixture(), { detection, geometry } = evaluate(input), epsilon = 2e-5;
    assert.ok(geometry.supported); assertStable(detection, geometry.innerT);
    for (let j = 0; j < 12; j++) {
        const plus = evaluateKirchhoffLumenSegmentContact(perturbed(input, j, epsilon));
        const minus = evaluateKirchhoffLumenSegmentContact(perturbed(input, j, -epsilon));
        assertStable(plus, geometry.innerT); assertStable(minus, geometry.innerT);
        close(geometry.gapJacobian[j], (plus.side.gap - minus.side.gap) / (2 * epsilon), 2e-8);
        close(geometry.outerTGradient[j], (plus.side.outerT - minus.side.outerT) / (2 * epsilon), 2e-8);
        const pG = flattenedGradient(plus.side), mG = flattenedGradient(minus.side);
        for (let i = 0; i < 12; i++) {
            close(geometry.normalDerivative[12 * i + j], (pG[i] - mG[i]) / (2 * epsilon), 3e-8);
            close(geometry.normalDerivative[12 * i + j], geometry.normalDerivative[12 * j + i], 1e-13);
        }
        for (let i = 0; i < 3; i++) close(geometry.normalJacobian[12 * i + j],
            (plus.side.normal[i] - minus.side.normal[i]) / (2 * epsilon), 3e-8);
    }
});

test('rotating only the outer segment differentiates the projection parameter and both outer force weights', () => {
    const input = fixture(), { geometry } = evaluate(input), epsilon = 1e-5;
    const center = input.outerStart.map((v, i) => .5 * (v + input.outerEnd[i]));
    const rotate = angle => {
        const next = structuredClone(input), c = Math.cos(angle), s = Math.sin(angle);
        for (const name of ['outerStart', 'outerEnd']) {
            const [x, y, z] = input[name].map((v, i) => v - center[i]);
            next[name] = [center[0] + c * x - s * y, center[1] + s * x + c * y, center[2] + z];
        }
        return next;
    };
    const velocity = [0, 0, 0, 0, 0, 0, ...['outerStart', 'outerEnd'].flatMap(name => {
        const p = input[name].map((v, i) => v - center[i]); return [-p[1], p[0], 0];
    })];
    const plus = evaluateKirchhoffLumenSegmentContact(rotate(epsilon)), minus = evaluateKirchhoffLumenSegmentContact(rotate(-epsilon));
    assertStable(plus, geometry.innerT); assertStable(minus, geometry.innerT);
    const dt = geometry.outerTGradient.reduce((sum, v, i) => sum + v * velocity[i], 0);
    assert.ok(Math.abs(dt) > .05); close(dt, (plus.side.outerT - minus.side.outerT) / (2 * epsilon), 2e-8);
    const pG = flattenedGradient(plus.side), mG = flattenedGradient(minus.side);
    for (let i = 0; i < 12; i++) {
        const product = geometry.normalDerivative.slice(12 * i, 12 * (i + 1)).reduce((sum, v, j) => sum + v * velocity[j], 0);
        close(product, (pG[i] - mG[i]) / (2 * epsilon), 3e-8);
    }
    // Omitting the changing outer weight loses exactly ±normal * dt.
    const dn = [0, 1, 2].map(i => geometry.normalJacobian.slice(12 * i, 12 * (i + 1)).reduce((sum, v, j) => sum + v * velocity[j], 0));
    const fixedWeight = dn.map(v => geometry.outerWeights[0] * v), correct = [6, 7, 8].map(i =>
        geometry.normalDerivative.slice(12 * i, 12 * (i + 1)).reduce((sum, v, j) => sum + v * velocity[j], 0));
    same(correct.map((v, i) => v - fixedWeight[i]), Array.from(geometry.normal, v => -v * dt), 1e-12);
    assert.ok(norm(correct.map((v, i) => v - fixedWeight[i])) > .05);
});

test('physical forces obey action/reaction, zero total moment and common translation null modes', () => {
    const input = fixture(), { geometry } = evaluate(input), Fn = 2.75, about = [7, -3, 5];
    const forces = names.map((_, i) => Array.from(geometry.normalForceColumn.slice(3 * i, 3 * i + 3), v => v * Fn));
    const inner = forces[0].map((v, i) => v + forces[1][i]), outer = forces[2].map((v, i) => v + forces[3][i]);
    same(inner, Array.from(geometry.normal, v => -Fn * v), 1e-13); same(outer, inner.map(v => -v), 1e-13);
    const moment = [0, 0, 0];
    names.forEach((name, node) => cross(input[name].map((v, i) => v - about[i]), forces[node]).forEach((v, i) => moment[i] += v));
    same(moment, [0, 0, 0], 1e-12);
    for (let axis = 0; axis < 3; axis++) {
        close([0, 1, 2, 3].reduce((sum, node) => sum + geometry.gapJacobian[3 * node + axis], 0), 0, 1e-14);
        for (let i = 0; i < 12; i++) close([0, 1, 2, 3].reduce((sum, node) => sum + geometry.normalDerivative[12 * i + 3 * node + axis], 0), 0, 1e-13);
    }
});

test('rigid world transforms and length-unit changes preserve the derivative and physical force contracts', () => {
    const input = fixture(), { geometry } = evaluate(input), angle = .63, c = Math.cos(angle), s = Math.sin(angle);
    const R = [[c, -s, 0], [s, c, 0], [0, 0, 1]], applyR = v => R.map(row => row.reduce((sum, x, i) => sum + x * v[i], 0));
    const transformed = structuredClone(input);
    for (const name of names) transformed[name] = applyR(input[name]).map((v, i) => v + [31, -47, 19][i]);
    const world = evaluate(transformed).geometry; assert.ok(world.supported, world.reason);
    close(world.gap, geometry.gap, 1e-12);
    for (let block = 0; block < 4; block++) same(world.gapJacobian.slice(3 * block, 3 * block + 3), applyR([...geometry.gapJacobian.slice(3 * block, 3 * block + 3)]), 1e-12);
    for (let i = 0; i < 12; i++) for (let j = 0; j < 12; j++) {
        let expected = 0;
        for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) expected += R[i % 3][a] * geometry.normalDerivative[12 * (3 * Math.floor(i / 3) + a) + 3 * Math.floor(j / 3) + b] * R[j % 3][b];
        close(world.normalDerivative[12 * i + j], expected, 1e-12);
    }
    const scaled = structuredClone(input), factor = 8;
    names.forEach(name => scaled[name] = input[name].map(v => factor * v)); scaled.lumenRadius *= factor; scaled.innerRadius *= factor;
    const units = evaluate(scaled).geometry;
    close(units.gap, factor * geometry.gap, 1e-13); same(units.normalForceColumn, geometry.normalForceColumn, 1e-13);
    same(units.normalDerivative, Array.from(geometry.normalDerivative, v => v / factor), 1e-13);
});

test('workspace reuse neither repeats detection/manifold updates nor aliases input, raw provenance or another workspace', () => {
    const input = fixture(); let detectorCalls = 0, manifoldCalls = 0;
    input.manifold = { upsertContact() { manifoldCalls++; return { id: 'external-manifold-record' }; } };
    const detect = args => { detectorCalls++; return evaluateKirchhoffLumenSegmentContact(args); };
    const original = detect(input).side, ws = createCompositeLumenSideGeometryWorkspace();
    const G = ws.gapJacobian, H = ws.normalDerivative;
    for (let i = 0; i < 4; i++) {
        differentiateCompositeLumenSideContact({ input, contact: original }, ws);
        assert.ok(ws.supported); assert.equal(ws.gapJacobian, G); assert.equal(ws.normalDerivative, H);
    }
    assert.equal(detectorCalls, 1); assert.equal(manifoldCalls, 1); assert.equal(ws.queryCount, 0);
    const independent = evaluate(fixture()).geometry, saved = independent.normalDerivative.slice();
    original.normal[0] += .1; assert.notEqual(ws.rawContact.normal[0], original.normal[0]);
    ws.normalDerivative[0] = 123; same(independent.normalDerivative, saved, 0);
    input.innerStart[0] += .1; assert.notEqual(ws.positions[0][0], input.innerStart[0]);
});

test('default quadrature and vector-object inputs match the original detector with the sample explicitly frozen', () => {
    const input = fixture(); delete input.quadrature;
    for (const name of names) { const [x, y, z] = input[name]; input[name] = { x, y, z }; }
    const { detection, geometry } = evaluate(input);
    assert.ok(geometry.supported, geometry.reason); assert.equal(geometry.innerT, 1); assert.equal(detection.side.innerT, 1);
    same(geometry.gapJacobian.slice(0, 3), [0, 0, 0], 0);
    assert.equal(geometry.selectionCertified, false, 'no claim about remaximization or ties is made');
});

test('a quadrature tie remains an explicit frozen-sample derivative, without inventing a unique winner', () => {
    const input = { ...fixture(), outerStart: [0, 0, 0], outerEnd: [2, 0, 0],
        innerStart: [.4, .5, 0], innerEnd: [1.5, .5, 0], quadrature: [.25, .75] };
    const ws = createCompositeLumenSideGeometryWorkspace(), first = evaluate(input, ws);
    assert.ok(first.geometry.supported); assert.equal(ws.innerT, .25); assert.equal(ws.selectionCertified, false);
    close(first.detection.samples[0].gap, first.detection.samples[1].gap, 0);
    const signature = ws.branchSignature;
    input.quadrature = [.75, .25];
    const second = evaluate(input, ws);
    assert.ok(second.geometry.supported); assert.equal(ws.innerT, .75); assert.equal(ws.selectionCertified, false);
    assert.notEqual(ws.branchSignature, signature); same(ws.innerWeights, [.25, .75], 0);
});

test('stale gap, geometry, parameters, material identity, normal and weights cannot masquerade as the original side', () => {
    const cases = [
        [(_, c) => c.gap += .001, 'original-contact-does-not-match-current-geometry'],
        [(_, c) => c.outerT += .01, 'original-contact-does-not-match-current-geometry'],
        [(_, c) => c.radialDistance += .01, 'original-contact-does-not-match-current-geometry'],
        [(_, c) => c.clearance += .01, 'original-contact-does-not-match-current-geometry'],
        [(i) => i.innerStart[1] += .01, 'original-contact-does-not-match-current-geometry'],
        [(i) => i.lumenRadius += .01, 'original-contact-does-not-match-current-geometry'],
        [(i) => i.innerMaterialSegmentId = 'other-wire', 'original-material-or-feature-mismatch'],
        [(_, c) => c.feature = 'different:side', 'original-material-or-feature-mismatch'],
        [(_, c) => c.normal[0] += .01, 'original-normal-does-not-match-radial-direction'],
        [(_, c) => c.innerWeights[0] += .01, 'original-contact-weights-mismatch'],
        [(_, c) => c.outerWeights[1] += .01, 'original-contact-weights-mismatch'],
        [(_, c) => c.gradients.outer[0][0] += .01, 'original-contact-gradients-mismatch'],
        [(i) => i.quadrature = [.1, .35], 'unrecognized-inner-quadrature-sample'],
    ];
    for (const [mutate, reason] of cases) {
        const input = fixture(), contact = evaluateKirchhoffLumenSegmentContact(input).side, ws = evaluate(input).geometry;
        mutate(input, contact); differentiateCompositeLumenSideContact({ input, contact }, ws);
        assert.equal(ws.supported, false); assert.equal(ws.reason, reason);
        for (const key of ['gapJacobian', 'normalForceColumn', 'forceColumn', 'normalDerivative', 'gapHessian', 'outerTGradient']) assert.ok(ws[key].every(Number.isNaN));
    }
});

test('outer endpoints, radial fallback, degenerate projection and portal/fillet ownership are explicit unsupported branches', () => {
    for (const x of [0, 2]) {
        const input = { ...fixture(), outerStart: [0, 0, 0], outerEnd: [2, 0, 0], innerStart: [x, .8, 0], innerEnd: [x, 1.1, 0] };
        assert.equal(evaluate(input).geometry.reason, 'outer-endpoint-or-clamped-projection');
    }
    const coaxial = { ...fixture(), outerStart: [0, 0, 0], outerEnd: [2, 0, 0], innerStart: [.3, 0, 0], innerEnd: [1.2, 0, 0] };
    assert.equal(evaluate(coaxial).geometry.reason, 'zero-or-fallback-radial-normal');
    const input = fixture(), contact = evaluateKirchhoffLumenSegmentContact(input).side;
    input.outerEnd = input.outerStart.map((v, i) => v + (i === 0 ? 1e-7 : 0));
    assert.equal(differentiateCompositeLumenSideContact({ input, contact }, createCompositeLumenSideGeometryWorkspace()).reason, 'degenerate-detector-projection-branch');
    const rimInput = { ...fixture(), innerStart: [9, .7, 0], innerEnd: [11, 1.1, 0], outerStart: [0, 0, 0], outerEnd: [10, 0, 0], openDistal: true };
    const rim = evaluateKirchhoffLumenSegmentContact(rimInput).portal.contact;
    assert.ok(rim); assert.equal(differentiateCompositeLumenSideContact({ input: rimInput, contact: rim }, createCompositeLumenSideGeometryWorkspace()).reason, 'unsupported-feature:distal-rim');
    const lipInput = { ...rimInput, innerStart: [9.85, .85, 0], innerEnd: [10.05, .85, 0], portalFilletRadius: .2 };
    const lip = evaluateKirchhoffLumenSegmentContact(lipInput).fillet;
    assert.ok(lip); assert.equal(differentiateCompositeLumenSideContact({ input: lipInput, contact: lip }, createCompositeLumenSideGeometryWorkspace()).reason, 'unsupported-feature:distal-fillet');
    const sideInput = { ...fixture(), innerStart: [9.85, .85, 0], innerEnd: [9.9, .9, 0], outerStart: [0, 0, 0], outerEnd: [10, 0, 0] };
    const side = evaluateKirchhoffLumenSegmentContact(sideInput).side;
    sideInput.openDistal = true; sideInput.portalFilletRadius = .2;
    assert.equal(differentiateCompositeLumenSideContact({ input: sideInput, contact: side }, createCompositeLumenSideGeometryWorkspace()).reason, 'distal-portal-or-fillet-owned-sample');
});

test('invalid input or missing original record invalidates a previously successful workspace', () => {
    for (const mutate of [i => i.innerEnd[0] = NaN, i => i.innerRadius = -1, i => i.quadrature = [], i => i.portalFilletRadius = -1,
        i => i.quadrature = null, i => i.portalFilletRadius = null, i => i.activationDistance = null, i => i.featurePrefix = null]) {
        const input = fixture(), { detection, geometry } = evaluate(input); mutate(input);
        assert.throws(() => differentiateCompositeLumenSideContact({ input, contact: detection.side }, geometry));
        assert.equal(geometry.supported, false); assert.ok(geometry.normalDerivative.every(Number.isNaN)); assert.ok(geometry.positions.every(p => p.every(Number.isNaN)));
    }
    const { geometry } = evaluate(fixture());
    differentiateCompositeLumenSideContact({ input: fixture(), contact: null }, geometry);
    assert.equal(geometry.reason, 'missing-original-contact'); assert.ok(geometry.gapJacobian.every(Number.isNaN));
});
