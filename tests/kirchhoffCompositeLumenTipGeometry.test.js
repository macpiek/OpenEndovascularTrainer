import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateKirchhoffLumenSegmentContact as detect } from '../src/physics/kirchhoffLumenContact.js';
import { createCompositeLumenTipGeometryWorkspace as create, differentiateCompositeLumenTipContact as differentiate } from '../src/physics/kirchhoffCompositeLumenTipGeometry.js';

const names = ['innerStart', 'innerEnd', 'outerStart', 'outerEnd'];
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const norm = v => Math.hypot(...v);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const close = (a, b, eps = 1e-10) => assert.ok(Math.abs(a - b) <= eps * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const same = (a, b, eps) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], eps)); };
const rawB = raw => [...raw.gradients.inner.flat(), ...raw.gradients.outer.flat()];
function fixture(kind = 'fillet') {
    return { innerStart: kind === 'fillet' ? [1.9, .38, 0] : [1.8, .4, .05],
        innerEnd: kind === 'fillet' ? [2.1, .38, 0] : [2.3, .7, .12], outerStart: [0, 0, 0], outerEnd: [2, 0, 0],
        lumenRadius: .5, innerRadius: .16, openDistal: true, portalFilletRadius: .15, quadrature: [.25],
        innerMaterialSegmentId: 'wire0', outerMaterialSegmentId: 'cat0' };
}
function original(input, kind) {
    const result = detect(input), raw = kind === 'fillet' ? result.fillet : result.portal.contact;
    assert.ok(raw, `Original ${kind} contact required`); return raw;
}
function evaluate(input, kind, ws = create()) {
    const raw = original(input, kind), out = differentiate({ input, contact: raw }, ws);
    assert.ok(out.supported, out.reason); return out;
}
function perturb(input, dof, value) {
    const next = structuredClone(input); next[names[Math.floor(dof / 3)]][dof % 3] += value; return next;
}
function wrench(input, column, scale = 1, origin = [0, 0, 0]) {
    const force = [0, 0, 0], moment = [0, 0, 0];
    names.forEach((name, block) => {
        const f = Array.from(column.slice(3 * block, 3 * block + 3), v => scale * v);
        f.forEach((v, i) => force[i] += v);
        cross(sub(input[name], origin), f).forEach((v, i) => moment[i] += v);
    });
    return { force, moment };
}

// Independent closed-form first derivative, used to differentiate G and B
// numerically. It does not share the production forward-derivative arena.
function analytic(input, kind) {
    const [A, W, C, D] = names.map(name => input[name]), u = sub(D, C), L = norm(u), e = u.map(v => v / L), d = sub(W, A);
    const s = kind === 'fillet' ? input.quadrature[0] : -dot(sub(A, D), e) / dot(d, e);
    const p = A.map((v, i) => v + s * d[i]), x = sub(p, D), z = dot(x, e), radial = x.map((v, i) => v - z * e[i]);
    const r = norm(radial), n = radial.map(v => v / r), f = input.portalFilletRadius, R = Math.max(0, input.lumenRadius - input.innerRadius) + f;
    let v, K;
    if (kind === 'fillet') {
        const h = Math.hypot(z + f, r - R);
        v = e.map((value, i) => ((z + f) * value + (r - R) * n[i]) / h);
        K = (f * r + R * z) / h;
    } else {
        const beta = dot(n, d) / dot(e, d);
        v = e.map((value, i) => -n[i] + beta * value); K = beta * r;
    }
    const G = [...v.map(value => (1 - s) * value), ...v.map(value => s * value),
        ...n.map(value => -K * value / L), ...v.map((value, i) => -value + K * n[i] / L)];
    const m = kind === 'fillet' ? 1 : norm(v);
    return { G, B: G.map(value => value / m), m, s };
}

test('fillet restores the missing catheter lever couple with the original torus gap and unit Fn', () => {
    const input = fixture(), raw = original(input, 'fillet'), out = differentiate({ input, contact: raw }, create());
    assert.ok(out.supported); close(out.gap, -.001339312526814962, 1e-15); assert.equal(out.gap, raw.gap);
    close(wrench(input, rawB(raw)).moment[2], -.21861865804880165, 1e-14);
    same(wrench(input, out.normalForceColumn).force, [0, 0, 0], 1e-14);
    same(wrench(input, out.normalForceColumn).moment, [0, 0, 0], 1e-14);
    same(out.normalForceColumn.slice(6, 9), [0, -.1093093290244007, 0], 1e-14);
    same(out.normalForceColumn.slice(9), [-.6726727939963122, .8492494024203444, 0], 1e-14);
    same(out.normalForceColumn, analytic(input, 'fillet').G, 1e-14);
    same(out.gapJacobian, out.normalForceColumn, 0); same(out.normalDerivative, out.gapHessian, 0);
    same(out.physicalNormal, raw.normal.map(v => -v), 1e-14); close(norm(out.physicalNormal), 1, 1e-14);
    assert.equal(out.forceScale, 1); assert.ok(out.forceScaleGradient.every(v => v === 0));
    assert.equal(out.rawContact.kind, 'distal-fillet'); same(out.rawContact.gradients.flat(), rawB(raw), 0);
});

test('rim differentiates the moving crossing, uses unit physical normal load, and does no work on tangent reparameterization', () => {
    const input = fixture('rim'), raw = original(input, 'rim'), out = evaluate(input, 'rim');
    close(out.innerT, .4); close(out.gap, -.03581745882007381, 1e-14);
    close(out.forceScale, 1.1735224804940634, 1e-14);
    const d = sub(input.innerEnd, input.innerStart), tangentMotion = [...d, ...d, 0, 0, 0, 0, 0, 0];
    close(dot(out.physicalNormal, d), 0, 1e-14); close(norm(out.physicalNormal), 1, 1e-14);
    close(dot(out.gapJacobian, tangentMotion), 0, 1e-14); close(dot(out.normalForceColumn, tangentMotion), 0, 1e-14);
    assert.ok(Math.abs(dot(rawB(raw), tangentMotion)) > .3, 'old radial load wrongly works along the inner tangent');
    close(dot(out.innerTGradient, tangentMotion), -1, 1e-14);
    same(out.gapJacobian, analytic(input, 'rim').G, 1e-14); same(out.normalForceColumn, analytic(input, 'rim').B, 1e-14);
    assert.ok(out.gapJacobian.some((v, i) => Math.abs(v - out.normalForceColumn[i]) > .05));
    assert.ok(out.normalForceColumn.some((v, i) => Math.abs(v - rawB(raw)[i]) > .2));
    const epsilon = 1e-5, shifted = sign => {
        const next = structuredClone(input); for (const name of names.slice(0, 2)) next[name] = input[name].map((v, i) => v + sign * epsilon * d[i]); return next;
    };
    close(original(shifted(1), 'rim').gap, original(shifted(-1), 'rim').gap, 1e-14);
    assert.equal(out.branchSignature, evaluate(shifted(1), 'rim').branchSignature, 's is moving, not a fixed rim branch identity');
    let skew = 0;
    for (let i = 0; i < 12; i++) for (let j = 0; j < 12; j++) skew = Math.max(skew, Math.abs(out.normalDerivative[12 * i + j] - out.normalDerivative[12 * j + i]));
    assert.ok(skew > .1, 'normalization has a required nonsymmetric derivative');
});

for (const kind of ['fillet', 'rim']) {
    test(`${kind}: every physical G/H/B/DB column matches original-gap and independent-analytic finite differences`, () => {
        const input = fixture(kind), out = evaluate(input, kind), h = 2e-6, ref = analytic(input, kind);
        same(out.gapJacobian, ref.G, 1e-13); same(out.normalForceColumn, ref.B, 1e-13);
        for (let j = 0; j < 12; j++) {
            const p = perturb(input, j, h), m = perturb(input, j, -h), pr = original(p, kind), mr = original(m, kind);
            const pa = analytic(p, kind), ma = analytic(m, kind);
            assert.equal(pr.kind, `distal-${kind}`); assert.equal(mr.kind, pr.kind);
            if (kind === 'fillet') { assert.equal(pr.innerT, .25); assert.equal(mr.innerT, .25); }
            else { assert.ok(pr.innerT > 0 && pr.innerT < 1); assert.ok(mr.innerT > 0 && mr.innerT < 1); }
            close(out.gapJacobian[j], (pr.gap - mr.gap) / (2 * h), 2e-8);
            close(out.innerTGradient[j], (pr.innerT - mr.innerT) / (2 * h), 2e-8);
            close(out.forceScaleGradient[j], (pa.m - ma.m) / (2 * h), 2e-8);
            for (let i = 0; i < 12; i++) {
                close(out.gapHessian[12 * i + j], (pa.G[i] - ma.G[i]) / (2 * h), 3e-8);
                close(out.normalDerivative[12 * i + j], (pa.B[i] - ma.B[i]) / (2 * h), 3e-8);
                close(out.gapHessian[12 * i + j], out.gapHessian[12 * j + i], 2e-13);
            }
            for (let i = 0; i < 3; i++) {
                close(out.normalJacobian[12 * i + j], (pr.normal[i] - mr.normal[i]) / (2 * h), 3e-8);
                close(out.physicalNormalJacobian[12 * i + j], (pa.B[i] + pa.B[i + 3] - ma.B[i] - ma.B[i + 3]) / (2 * h), 3e-8);
            }
        }
        same(out.forceColumn, Array.from(out.normalForceColumn, v => -v), 0);
    });

    test(`${kind}: all Hessian entries also match mixed finite differences of only the unchanged original detector gap`, () => {
        const input = fixture(kind), out = evaluate(input, kind), h = 3e-5;
        for (let i = 0; i < 12; i++) for (let j = 0; j <= i; j++) {
            const value = (di, dj) => original(perturb(perturb(input, i, di), j, dj), kind).gap;
            const fd = (value(h, h) - value(h, -h) - value(-h, h) + value(-h, -h)) / (4 * h * h);
            close(out.gapHessian[12 * i + j], fd, 4e-6);
        }
    });
}

test('both tip branches preserve signed forces, wrench, virtual power and differentiated rigid null modes', () => {
    for (const kind of ['fillet', 'rim']) {
        const input = fixture(kind), out = evaluate(input, kind), dx = Array.from({ length: 12 }, (_, i) => Math.sin(i + .4));
        for (const Fn of [-2.3, 0, 3.7]) {
            const result = wrench(input, out.normalForceColumn, Fn, [4, -7, 3]);
            same(result.force, [0, 0, 0], 1e-13); same(result.moment, [0, 0, 0], 1e-13);
            close(Fn * dot(out.normalForceColumn, dx), Fn / out.forceScale * dot(out.gapJacobian, dx), 1e-13);
        }
        for (let axis = 0; axis < 3; axis++) {
            const velocity = names.flatMap(() => [0, 1, 2].map(i => +(i === axis)));
            close(dot(out.gapJacobian, velocity), 0, 1e-13);
            for (const matrix of [out.gapHessian, out.normalDerivative]) for (let i = 0; i < 12; i++)
                close(dot(matrix.slice(12 * i, 12 * (i + 1)), velocity), 0, 2e-13);
        }
        const omega = [.3, -.7, 1.1], rotational = names.flatMap(name => cross(omega, input[name]));
        close(dot(out.gapJacobian, rotational), 0, 1e-13); close(dot(out.normalForceColumn, rotational), 0, 1e-13);
        for (const [matrix, column] of [[out.gapHessian, out.gapJacobian], [out.normalDerivative, out.normalForceColumn]]) {
            const expected = names.flatMap((_, block) => cross(omega, [...column.slice(3 * block, 3 * block + 3)]));
            for (let i = 0; i < 12; i++) close(dot(matrix.slice(12 * i, 12 * i + 12), rotational), expected[i], 3e-13);
        }
    }
});

test('full 3D rigid world covariance and length scaling include both halves of the normalized DB', () => {
    const axis = [2 / 3, -1 / 3, 2 / 3], angle = .71, c = Math.cos(angle), s = Math.sin(angle);
    const rotate = v => v.map((value, i) => c * value + s * cross(axis, v)[i] + (1 - c) * axis[i] * dot(axis, v));
    const R = [0, 1, 2].map(i => [0, 1, 2].map(j => rotate([+(j === 0), +(j === 1), +(j === 2)])[i]));
    for (const kind of ['fillet', 'rim']) {
        const input = fixture(kind), out = evaluate(input, kind), transformed = structuredClone(input), scaled = structuredClone(input);
        for (const name of names) {
            transformed[name] = rotate(input[name]).map((v, i) => v + [13, -17, 21][i]);
            scaled[name] = input[name].map(v => 8 * v);
        }
        for (const name of ['lumenRadius', 'innerRadius', 'portalFilletRadius']) scaled[name] *= 8;
        const world = evaluate(transformed, kind), units = evaluate(scaled, kind);
        close(world.gap, out.gap, 2e-13); close(world.forceScale, out.forceScale, 2e-13);
        same(world.physicalNormal, rotate([...out.physicalNormal]), 2e-13);
        for (const key of ['gapJacobian', 'normalForceColumn']) for (let block = 0; block < 4; block++)
            same(world[key].slice(3 * block, 3 * block + 3), rotate([...out[key].slice(3 * block, 3 * block + 3)]), 2e-13);
        for (const key of ['gapHessian', 'normalDerivative']) for (let i = 0; i < 12; i++) for (let j = 0; j < 12; j++) {
            let expected = 0;
            for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++)
                expected += R[i % 3][a] * out[key][12 * (3 * Math.floor(i / 3) + a) + 3 * Math.floor(j / 3) + b] * R[j % 3][b];
            close(world[key][12 * i + j], expected, 5e-12);
        }
        close(units.gap, 8 * out.gap, 1e-13); same(units.normalForceColumn, out.normalForceColumn, 1e-13);
        same(units.normalDerivative, Array.from(out.normalDerivative, v => v / 8), 1e-13);
    }
});

test('fresh raw geometry validates without another query; buffers, physical positions and raw snapshots are owned', () => {
    for (const kind of ['fillet', 'rim']) {
        const input = fixture(kind); let queries = 0, manifoldUpdates = 0;
        input.manifold = { upsertContact() { manifoldUpdates++; return {}; } };
        const detector = args => { queries++; return detect(args); }, result = detector(input);
        const contact = kind === 'fillet' ? result.fillet : result.portal.contact, updates = manifoldUpdates, out = create();
        const G = out.gapJacobian, H = out.gapHessian, B = out.normalForceColumn, DB = out.normalDerivative;
        const before = JSON.stringify({ input, contact });
        for (let i = 0; i < 4; i++) {
            differentiate({ input, contact }, out); assert.ok(out.supported, out.reason);
            assert.equal(out.gapJacobian, G); assert.equal(out.gapHessian, H); assert.equal(out.normalForceColumn, B); assert.equal(out.normalDerivative, DB);
        }
        assert.equal(queries, 1); assert.equal(manifoldUpdates, updates); assert.equal(out.queryCount, 0);
        assert.equal(out.certified, false); assert.equal(out.selectionCertified, false);
        assert.equal(JSON.stringify({ input, contact }), before);
        same(out.dofs, Array.from({ length: 12 }, (_, i) => i), 0); assert.deepEqual(out.pointOrder, names);
        names.forEach((name, i) => same(out.positions[i], input[name], 0)); same(out.outerPoint, input.outerEnd, 0);
        const independent = evaluate(fixture(kind), kind), saved = independent.normalDerivative.slice();
        out.normalDerivative[0] = 123; same(independent.normalDerivative, saved, 0);
        contact.normal[0] += .01; assert.notEqual(out.rawContact.normal[0], contact.normal[0]);
        input.innerStart[0] += .01; assert.notEqual(out.positions[0][0], input.innerStart[0]);
    }
});

test('stale original gap, geometry, radius, identity, normal, weights and simplified raw gradients are rejected', () => {
    const mutations = [
        (i, r) => { r.gap += .01; delete r.active; delete r.violation; },
        (i, r) => r.clearance += .01, (i, r) => r.radialDistance += .01, (i, r) => r.outerT = .99,
        i => i.innerStart[1] += .01, i => i.lumenRadius += .01, i => i.innerRadius += .01,
        i => i.portalFilletRadius += .01, i => i.innerMaterialSegmentId = 'different-wire',
        (i, r) => r.feature = 'other-feature', (i, r) => r.normal[0] += .01,
        (i, r) => r.innerWeights[0] += .01, (i, r) => r.outerWeights[0] += .01,
        (i, r) => r.gradients.outer[0][1] += .01, (i, r) => r.active = !r.active,
        (i, r) => r.violation += .01, i => i.openDistal = false
    ];
    for (const kind of ['fillet', 'rim']) for (const mutate of mutations) {
        const input = fixture(kind), raw = original(input, kind), out = evaluate(fixture(kind), kind);
        mutate(input, raw); differentiate({ input, contact: raw }, out);
        assert.equal(out.supported, false, `${kind}: ${mutate}`);
        assert.ok(out.normalDerivative.every(Number.isNaN)); assert.ok(out.gapJacobian.every(Number.isNaN));
        assert.ok(out.positions.every(p => p.every(Number.isNaN)));
    }
});

test('accepted raw source binds exact geometry/options across workspaces, including otherwise invisible rigid translation', () => {
    for (const kind of ['fillet', 'rim']) for (const mutate of [
        i => names.forEach(name => i[name][2] += 7), i => i.quadrature.push(.8),
        i => i.activationDistance = 1, (i, r) => { delete r.gradients; }
    ]) {
        const input = fixture(kind), raw = original(input, kind), out = create();
        differentiate({ input, contact: raw }, out); assert.ok(out.supported);
        mutate(input, raw);
        const next = differentiate({ input, contact: raw }, create());
        assert.equal(next.reason, 'original-source-mutated'); assert.ok(next.normalForceColumn.every(Number.isNaN));
    }
    // A freshly queried record is allowed after motion, with the same branch.
    const input = fixture('rim'), out = evaluate(input, 'rim');
    names.forEach(name => input[name][2] += 7); evaluate(input, 'rim', out); assert.ok(out.supported);
});

test('fillet branch excludes support boundaries, fallback radial direction and torus singularity explicitly', () => {
    function at(z, r) {
        return { ...fixture(), outerStart: [-2, 0, 0], outerEnd: [0, 0, 0], innerStart: [z, r, 0], innerEnd: [z, r, 0] };
    }
    for (const input of [at(-.15, .3), at(.15, .3), at(0, .49)]) {
        const raw = original(input, 'fillet'), out = differentiate({ input, contact: raw }, create());
        assert.equal(out.reason, 'fillet-feature-boundary-or-outside');
    }
    const zero = at(0, 0), zr = original(zero, 'fillet');
    assert.equal(differentiate({ input: zero, contact: zr }, create()).reason, 'zero-or-fallback-radial-normal');
    const missing = fixture(), mr = original(missing, 'fillet'); missing.portalFilletRadius = 0;
    assert.equal(differentiate({ input: missing, contact: mr }, create()).reason, 'missing-smooth-fillet');
    const missingSample = fixture(), msr = original(missingSample, 'fillet'); missingSample.quadrature = [.75];
    assert.equal(differentiate({ input: missingSample, contact: msr }, create()).reason, 'unrecognized-inner-quadrature-sample');
    const singular = at(-.15 + 1e-13, .49 - 1e-13);
    const copied = structuredClone(original(fixture(), 'fillet'));
    assert.equal(differentiate({ input: singular, contact: copied }, create()).reason, 'degenerate-fillet-circle');
});

test('rim only supports strict interior crossings: clamp/tolerance/endpoints, parallel and radial fallbacks reject', () => {
    for (const [a, b] of [[0, .3], [-.3, 0], [5e-10, .3], [-.3, -5e-10]]) {
        const input = { ...fixture('rim'), outerStart: [-2, 0, 0], outerEnd: [0, 0, 0],
            innerStart: [a, .6, 0], innerEnd: [b, .7, 0] };
        const raw = original(input, 'rim');
        assert.equal(differentiate({ input, contact: raw }, create()).reason, 'rim-endpoint-or-clamped-crossing');
    }
    const parallel = fixture('rim'), pr = original(parallel, 'rim'); parallel.innerEnd[0] = parallel.innerStart[0];
    assert.equal(differentiate({ input: parallel, contact: pr }, create()).reason, 'degenerate-rim-crossing');
    const zero = { ...fixture('rim'), innerStart: [1.8, 0, 0], innerEnd: [2.3, 0, 0], activationDistance: 1 };
    assert.equal(differentiate({ input: zero, contact: original(zero, 'rim') }, create()).reason, 'zero-or-fallback-radial-normal');
    const open = { ...fixture('rim'), innerStart: [1.8, .1, 0], innerEnd: [2.3, .2, 0], activationDistance: 1 };
    const contact = original(open, 'rim'); open.activationDistance = 0;
    assert.equal(differentiate({ input: open, contact }, create()).reason, 'original-rim-outside-activation');
});

test('invalid inputs, missing contacts, unsupported features and zero axes invalidate all previously valid outputs', () => {
    for (const mutate of [i => i.innerEnd[1] = NaN, i => i.lumenRadius = -1, i => i.portalFilletRadius = null,
        i => i.activationDistance = null, i => i.quadrature = [], i => i.quadrature = null, i => i.featurePrefix = null]) {
        const input = fixture(), contact = original(input, 'fillet'), out = evaluate(fixture(), 'fillet'); mutate(input);
        assert.throws(() => differentiate({ input, contact }, out));
        assert.equal(out.supported, false); assert.ok(out.normalDerivative.every(Number.isNaN));
    }
    const input = fixture(), contact = original(input, 'fillet'), out = evaluate(fixture(), 'fillet');
    input.outerStart = [...input.outerEnd];
    assert.equal(differentiate({ input, contact }, out).reason, 'degenerate-outer-axis');
    assert.equal(differentiate({ input: fixture(), contact: null }, out).reason, 'missing-original-contact');
    assert.equal(differentiate({ input: fixture(), contact: { kind: 'side' } }, out).reason, 'unsupported-feature:side');
    assert.equal(differentiate({ input: fixture(), contact: { kind: 'other' } }, out).reason, 'unsupported-feature:other');
    assert.ok(out.forceColumn.every(Number.isNaN)); assert.ok(Number.isNaN(out.gap));
});

test('default samples, vector-object inputs and reversed strict rim direction retain explicit branch semantics', () => {
    const fillet = fixture(); delete fillet.quadrature;
    for (const name of names) { const [x, y, z] = fillet[name]; fillet[name] = { x, y, z }; }
    const f = evaluate(fillet, 'fillet'); assert.match(f.derivativeScope, /fixed-inner-quadrature/);
    assert.equal(f.selectionCertified, false);
    const rim = fixture('rim'), before = evaluate(rim, 'rim');
    [rim.innerStart, rim.innerEnd] = [rim.innerEnd, rim.innerStart];
    const after = evaluate(rim, 'rim'); close(after.innerT, 1 - before.innerT, 1e-14);
    same(after.normalForceColumn, [...before.normalForceColumn.slice(3, 6), ...before.normalForceColumn.slice(0, 3), ...before.normalForceColumn.slice(6)], 1e-13);
    assert.match(after.derivativeScope, /strict-interior-moving/); assert.equal(after.selectionCertified, false);
});

test('fillet ties retain the selected fixed quadrature and do not invent a winner or transfer a reaction', () => {
    const input = { ...fixture(), innerStart: [1.95, .38, 0], innerEnd: [1.95, .38, 0], quadrature: [.25, .75] };
    const first = evaluate(input, 'fillet'); assert.equal(first.innerT, .25); assert.equal(first.selectionCertified, false);
    input.quadrature.reverse(); const second = evaluate(input, 'fillet'); assert.equal(second.innerT, .75);
    assert.equal(second.gap, first.gap); assert.notEqual(second.branchSignature, first.branchSignature);
    assert.ok(first.normalForceColumn.some((v, i) => Math.abs(v - second.normalForceColumn[i]) > .3));
});
