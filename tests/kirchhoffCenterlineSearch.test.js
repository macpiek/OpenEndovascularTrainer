import assert from 'node:assert/strict';
import { closestKirchhoffCenterlinePoint, prepareKirchhoffCenterlineSearch } from '../src/physics/kirchhoffCenterlineSearch.js';

// Independent basis-form reference: exhaustively evaluates every candidate,
// as the original lumen solver did, without polynomial or hull shortcuts.
function sample(body, segment, t) {
    const cubic = segment > body.activeStart && segment + 2 <= body.activeEnd;
    const t2 = t * t, t3 = t2 * t;
    const weights = cubic ? [(1 - 3*t + 3*t2 - t3)/6,
        (4 - 6*t2 + 3*t3)/6, (1 + 3*t + 3*t2 - 3*t3)/6, t3/6] : [1-t, t];
    const first = cubic ? segment - 1 : segment;
    const derivatives = cubic ? [(-3 + 6*t - 3*t2)/6,
        (-12*t + 9*t2)/6, (3 + 6*t - 9*t2)/6, t2/2] : [-1, 1];
    const second = cubic ? [1-t, -2+3*t, 1-3*t, t] : [0, 0];
    return [weights, derivatives, second].map(basis => ['x', 'y', 'z'].map(axis =>
        basis.reduce((sum, value, i) => sum + value * body[axis][first + i], 0)));
}

function reference(body, point, first, last) {
    let best = { distanceSquared: Infinity };
    for (let segment = first; segment <= last; segment++) {
        const chord = ['x', 'y', 'z'].map(axis => body[axis][segment+1] - body[axis][segment]);
        const squaredLength = chord.reduce((sum, value) => sum + value*value, 0);
        const projection = ['x', 'y', 'z'].reduce((sum, axis, i) =>
            sum + (point[i] - body[axis][segment]) * chord[i], 0);
        let t = squaredLength > 1e-8 ? Math.max(0, Math.min(1, projection/squaredLength)) : 0;
        for (let iteration = 0; iteration < 4; iteration++) {
            const [position, tangent, second] = sample(body, segment, t);
            const distance = point.map((value, axis) => value - position[axis]);
            const denominator = tangent.reduce((sum, value, i) =>
                sum + value*value - distance[i]*second[i], 0);
            if (Math.abs(denominator) <= 1e-8) break;
            const next = Math.max(0, Math.min(1, t + distance.reduce((sum, value, i) =>
                sum + value*tangent[i], 0)/denominator));
            const settled = Math.abs(next-t) <= 1e-5;
            t = next;
            if (settled) break;
        }
        const position = sample(body, segment, t)[0];
        const distanceSquared = point.reduce((sum, value, i) => sum + (value-position[i])**2, 0);
        if (distanceSquared < best.distanceSquared) best = { segment, t, distanceSquared };
    }
    return best;
}

let seed = 72809;
const random = () => ((seed = Math.imul(seed, 1664525) + 1013904223) >>> 0) / 2**32;
const body = { count: 31, activeStart: 3, activeEnd: 28,
    x: new Float64Array(31), y: new Float64Array(31), z: new Float64Array(31) };
const scratch = {}, cachedScratch = {}, cache = {};
let rejected = 0;
for (const shape of ['straight', 'curved', 'folded', 'coincident']) {
    for (let i = 0; i < body.count; i++) {
        body.x[i] = shape === 'coincident' ? 4 : shape === 'folded' ? 9*Math.sin(i) : 4*i;
        body.y[i] = shape === 'straight' || shape === 'coincident' ? 0 : 8*Math.sin(i*0.5);
        body.z[i] = shape === 'straight' || shape === 'coincident' ? 0 : 3*Math.cos(i*0.4);
    }
    prepareKirchhoffCenterlineSearch(body, cache);
    for (let n = 0; n < 400; n++) {
        const segment = 3 + Math.floor(random()*25);
        const point = sample(body, segment, random())[0].map(value => value + random()-0.5);
        const first = Math.max(body.activeStart, segment-2);
        const last = Math.min(body.activeEnd-1, segment+2);
        const expected = reference(body, point, first, last);
        const actual = closestKirchhoffCenterlinePoint(body, point, first, last, segment, scratch);
        const cached = closestKirchhoffCenterlinePoint(body, point, first, last, segment, cachedScratch, cache);
        assert.equal(cached.segment, actual.segment);
        assert.equal(cached.t, actual.t);
        assert.equal(cached.distanceSquared, actual.distanceSquared);
        assert.equal(actual, scratch);
        assert.ok(Math.abs(actual.distanceSquared - expected.distanceSquared) < 1e-8,
            `${shape}: ${actual.distanceSquared} vs ${expected.distanceSquared}`);
        // At an exact tie any coincident representation has the same physics.
        if (shape !== 'coincident') {
            const a = sample(body, actual.segment, actual.t)[0];
            const b = sample(body, expected.segment, expected.t)[0];
            assert.ok(Math.hypot(...a.map((value, i) => value-b[i])) < 1e-6);
        } else assert.equal(actual.segment, first, 'preserve the ascending segment tie break');
        rejected += actual.rejectedCandidates;
    }
}
assert.ok(rejected > 1000, 'the conservative bounds must actually avoid distant curve evaluations');
console.log(`Kirchhoff centerline search: 1600 exhaustive comparisons passed; ${rejected} candidates pruned`);
