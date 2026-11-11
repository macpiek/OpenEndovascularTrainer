import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {columnProjectionFragmentShader} from '../src/contrast/anatomicalColumnProjection.js';
import {columnNearestReference} from './helpers/columnNearestReference.js';

const fixture = JSON.parse(readFileSync(new URL('./fixtures/contrast-renal-nearest.json', import.meta.url)));
const f = Math.fround;
const add = (a, b) => f(a + b), sub = (a, b) => f(a - b), mul = (a, b) => f(a * b);
const dot = (a, b) => add(add(mul(a[0], b[0]), mul(a[1], b[1])), mul(a[2], b[2]));
const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

// An explicit Float32 model of the shader's elementary operations. It is not
// a GPU compiler oracle (a driver may fuse operations); browser parity still
// compares the real shader against the frozen GLSL. This specifically checks
// that caching immutable texels changes no arithmetic, iteration or corner.
function project({range, atlas}, point, seed, cached) {
    let s = clamp(f(seed), 0, range.length), bestS = s, bestDistance = 1e30, bestRadius = 0;
    let cachedIndex = -1, a, b, derivative, fetches = 0, iterations = 0;
    const visited = [];
    for (let i = 0; i <= 6; i++) {
        const x = clamp(f(s / Math.max(range.step, 1e-8)), 0, range.count - 1);
        const index = Math.min(range.count - 2, Math.floor(x));
        if (!cached || index !== cachedIndex) {
            a = atlas.slice(index * 4, index * 4 + 4);
            b = atlas.slice(index * 4 + 4, index * 4 + 8);
            derivative = a.map((v, k) => f(sub(b[k], v) / Math.max(range.step, 1e-8)));
            cachedIndex = index; fetches += 2;
        }
        const fraction = sub(x, index), complement = sub(1, fraction);
        const local = a.map((v, k) => add(mul(v, complement), mul(b[k], fraction)));
        const delta = point.map((v, k) => sub(v, local[k]));
        const next = add(s, f(dot(delta, derivative) / Math.max(1e-8, dot(derivative, derivative))));
        const segment = Math.min(range.count - 2, Math.floor(f(s / Math.max(range.step, 1e-8))));
        const candidate = clamp(next, mul(segment, range.step), mul(add(segment, 1), range.step));
        const nearest = local.map((v, k) => add(v, mul(derivative[k], sub(candidate, s))));
        const offset = point.map((v, k) => sub(v, nearest[k])), distance = dot(offset, offset);
        if (distance < bestDistance || (distance === bestDistance && candidate < bestS)) {
            bestDistance = distance; bestS = candidate; bestRadius = nearest[3];
        }
        const bounded = clamp(next, 0, range.length);
        visited.push({s, bounded, index, candidate}); iterations++;
        if (i >= 6 || bounded === s) break;
        s = bounded;
    }
    return {s: bestS, score: sub(f(Math.sqrt(Math.max(0, bestDistance))), bestRadius), iterations, visited, fetches};
}

test('cached shader preserves the entire frozen Newton arithmetic and iteration sequence', () => {
    const start = columnProjectionFragmentShader.indexOf('float columnNearestRefined(');
    const end = columnProjectionFragmentShader.indexOf('\nfloat columnNearestCoordinate(', start);
    let actual = columnProjectionFragmentShader.slice(start, end).trim();
    const cachedStart = actual.indexOf('        // Repeated Newton iterates');
    const cachedEnd = actual.indexOf('\n        float next =', cachedStart);
    assert.ok(cachedStart > 0 && cachedEnd > cachedStart);
    const cache = actual.slice(cachedStart, cachedEnd);
    assert.ok(cache.includes('if (index != cachedIndex)'));
    assert.ok(cache.includes('derivative = (cachedB - cachedA) / max(range.w, 1e-8);'));
    assert.ok(cache.includes('vec4 local = mix(cachedA, cachedB, x - float(index));'));
    actual = actual.slice(0, cachedStart) + '        vec4 local = columnGeometryAt(range, s, derivative);' + actual.slice(cachedEnd);
    actual = actual.replace('    vec4 derivative, cachedA, cachedB;\n    int cachedIndex = -1;', '    vec4 derivative;');
    assert.equal(actual, columnNearestReference.trim(), 'Only the immutable atlas fetches may change');
});

test('real curved renal paths retain bit-identical Float32 coordinates, clearance and visits', () => {
    let probes = 0, referenceFetches = 0, cachedFetches = 0;
    for (const path of fixture.cases) for (const {point, seed} of path.probes) {
        const reference = project(path, point, seed, false), actual = project(path, point, seed, true);
        referenceFetches += reference.fetches; cachedFetches += actual.fetches; probes++;
        const {fetches: ignoredReference, ...expected} = reference;
        const {fetches: ignoredActual, ...result} = actual;
        assert.deepEqual(result, expected, `Renal path ${path.path}, seed ${seed}, point ${point}`);
    }
    assert.ok(probes > 100, 'Exercise both endpoints and radial positions on the frozen paths');
    assert.ok(cachedFetches < referenceFetches * .6, 'The cache must remove repeated texel reads, not merely rename variables');
});

test('near-corner renal projections still continue past misleading same-segment steps', () => {
    const path = fixture.cases.find(path => path.path === 7168);
    const {point, seed} = path.probes.find(probe => probe.cornerRegression);
    const result = project(path, point, seed, true);
    assert.equal(result.iterations, 7);
    assert.ok(result.visited.some((step, index, steps) => index && step.index === steps[index - 1].index));
    assert.ok(result.visited.slice(3).some(step => step.index === 0));
    assert.equal(result.s, 0.17147667706012726);
    assert.ok(Math.abs(result.s - .3234037756919861) > .15, 'The incorrect early-stop coordinate must not return');
});
