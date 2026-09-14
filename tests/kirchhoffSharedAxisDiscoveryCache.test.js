import test from 'node:test';
import assert from 'node:assert/strict';
import { createSharedAxisDiscoveryCache } from '../src/physics/kirchhoffSharedAxisDiscoveryCache.js';
import { createSharedAxisVesselDiscovery } from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

const geometryToken = {};
const descriptor = (extra = {}) => ({ a: [0, 0, 0], b: [0, 0, 5], sampleCount: 5,
    geometryToken, gridToken: 'material/70/75/0/1', radius: 1, ...extra });
const contact = distance => ({ signedDistance: distance, inside: distance > 0, source: 'sparse-sdf-bvh', faceIndex: 2 });
function certify(cache, d, distance = () => 10, key = 'edge', options = { insideCertified: true }) {
    const capture = cache.begin(key, d, options);
    // The actual capsule visits both endpoints before its interior samples.
    for (const i of [0, d.sampleCount, ...Array.from({ length: d.sampleCount - 1 }, (_, j) => j + 1)]) {
        const t = i / d.sampleCount;
        capture.visit(contact(distance(d.a.map((v, k) => (1 - t) * v + t * d.b[k]), t)), t);
    }
    return capture.commit();
}

test('clearance proof retains the certified pose across repeated skips and distinguishes inside from clear', () => {
    const cache = createSharedAxisDiscoveryCache(), d = descriptor();
    assert.equal(certify(cache, d), true);
    for (const x of [1, 2, 4, 6, 8]) {
        const proof = cache.lookup('edge', descriptor({ a: [x, 0, 0], b: [x, 0, 5] }));
        assert.equal(proof.skip, true); assert.equal(proof.knownInside, true);
        assert.ok(proof.lowerBound <= 10 - x && proof.lowerBound > 10 - x - 1e-10);
    }
    assert.equal(cache.lookup('edge', descriptor({ a: [8.5, 0, 0], b: [8.5, 0, 5] })).skip, false, 'equality cannot skip discovery');
    const near = cache.lookup('edge', descriptor({ a: [9, 0, 0], b: [9, 0, 5] }));
    assert.equal(near.skip, false); assert.equal(near.knownInside, true);
    assert.equal(cache.lookup('edge', descriptor({ a: [10, 0, 0], b: [10, 0, 5] })).knownInside, false);
});

test('sample grid, spatial interval and surface identity must match; changed radius uses the new clearance threshold', () => {
    const cache = createSharedAxisDiscoveryCache(), d = descriptor(); certify(cache, d);
    for (const extra of [{ sampleCount: 6 }, { gridToken: 'material/71/76/0/1' }, { geometryToken: {} },
        { radius: NaN }, { margin: -1 }, { a: [Infinity, 0, 0] }]) {
        assert.equal(cache.lookup('edge', descriptor(extra)).knownInside, false);
        assert.equal(cache.lookup('edge', descriptor(extra)).skip, false);
    }
    assert.equal(cache.lookup('edge', descriptor({ radius: 9.6 })).skip, false);
    assert.equal(cache.lookup('edge', descriptor({ radius: .5 })).skip, true);
    d.a[0] = 50;
    assert.equal(cache.lookup('edge', descriptor()).skip, true, 'certificate snapshots endpoints rather than retaining mutable arrays');
    cache.invalidate('edge'); assert.equal(cache.lookup('edge', descriptor()).skip, false);
});

test('winner-only, duplicate, shifted grid, outside, non-BVH and failed queries never publish certificates', () => {
    const d = descriptor(), badCases = [
        capture => capture.visit(contact(10), 0),
        capture => { for (let i = 0; i <= 5; i++) capture.visit(contact(10), 0); },
        capture => { for (let i = 0; i <= 5; i++) capture.visit(contact(10), i / 5 + (i === 2 ? 1e-15 : 0)); },
        capture => { for (let i = 0; i <= 5; i++) capture.visit(contact(i === 3 ? -1 : 10), i / 5); },
        capture => { for (let i = 0; i <= 5; i++) capture.visit({ ...contact(10), source: 'sparse-sdf' }, i / 5); },
        capture => { for (let i = 0; i <= 5; i++) capture.visit(contact(NaN), i / 5); }
    ];
    for (const fill of badCases) {
        const cache = createSharedAxisDiscoveryCache(); certify(cache, d);
        const capture = cache.begin('edge', d, { insideCertified: true }); fill(capture);
        assert.equal(capture.commit(), false); assert.equal(cache.size, 0);
    }
    const cache = createSharedAxisDiscoveryCache(); certify(cache, d);
    const capture = cache.begin('edge', d, { insideCertified: true }); capture.visit(contact(10), 0); capture.abort();
    assert.equal(capture.commit(), false); assert.equal(cache.lookup('edge', d).skip, false);
    assert.equal(certify(cache, d, () => 10, 'edge', {}), false, 'positive BVH sign alone does not assert physical inside');
});

test('seeded independent endpoint motions preserve all affine-grid wall distances and contact discovery', () => {
    let seed = 981783;
    const random = () => ((seed = Math.imul(seed, 1664525) + 1013904223 | 0) >>> 0) / 2 ** 32;
    // Exact signed distance inside a closed rectangular mesh; all initial
    // points are inside. Closest wall/normal can switch after the motion.
    const distance = p => Math.min(...p.map(v => 20 - Math.abs(v)));
    let skips = 0, inside = 0;
    for (let trial = 0; trial < 1000; trial++) {
        const cache = createSharedAxisDiscoveryCache();
        const d = descriptor({ a: Array.from({ length: 3 }, () => 24 * (random() - .5)),
            b: Array.from({ length: 3 }, () => 24 * (random() - .5)), sampleCount: 1 + Math.floor(random() * 16), radius: .3 + random() * 2 });
        assert.equal(certify(cache, d, distance), true);
        const moved = { ...d, a: d.a.map(v => v + 16 * (random() - .5)), b: d.b.map(v => v + 16 * (random() - .5)) };
        const proof = cache.lookup('edge', moved);
        skips += Number(proof.skip); inside += Number(proof.knownInside);
        for (let i = 0; i <= d.sampleCount; i++) {
            const t = i / d.sampleCount, actual = distance(moved.a.map((v, k) => (1 - t) * v + t * moved.b[k]));
            assert.ok(actual >= proof.lowerBound - 1e-12);
            if (proof.skip) assert.ok(actual - moved.radius > .5);
            if (proof.knownInside) assert.ok(actual > 0);
        }
    }
    assert.ok(skips > 100); assert.ok(inside > skips);
});

test('distance error allowance and large-coordinate guard reduce skips; storage is bounded and resettable', () => {
    const cache = createSharedAxisDiscoveryCache({ capacity: 2 }), d = descriptor();
    certify(cache, d, () => 2, 'first', { insideCertified: true, distanceError: .6 });
    assert.equal(cache.lookup('first', d).skip, false);
    certify(cache, d, () => 10, 'second'); certify(cache, d, () => 10, 'third');
    assert.equal(cache.size, 2); assert.equal(cache.stats.evictions, 1);
    assert.equal(cache.lookup('first', d).knownInside, false);
    const distant = descriptor({ a: [1e12, 0, 0], b: [1e12, 0, 5] });
    certify(cache, distant, () => 1.501, 'far');
    assert.equal(cache.lookup('far', distant).skip, false, 'roundoff guard cannot create an optimistic skip');
    cache.clear(); assert.equal(cache.size, 0);
});

function discoveryFixture({ queryReuse = true, source = 'sparse-sdf-bvh', winnerOnly = false } = {}) {
    const field = { voxelSize: .25, fallbackGeometry: { boundsTree: {} }, calls: [], fail: false,
        queryCapsuleSoA(...args) {
            const [x, , , r] = args, knownInside = args[7], count = args[12], visitor = args[15];
            this.calls.push({ x: Array.from(x), knownInside, count });
            let result;
            for (const i of [0, count, ...Array.from({ length: count - 1 }, (_, j) => j + 1)]) {
                const t = i / count, distance = 10 - ((1 - t) * x[0] + t * x[1]);
                result = { ...contact(distance), source, signedGap: distance - r[0], segmentT: t };
                if (!winnerOnly) visitor?.(result, t);
                if (this.fail) throw new Error('geometry query interrupted');
            }
            return result;
        }
    };
    const discover = createSharedAxisVesselDiscovery(field, 0, { queryReuse });
    const state = { origin: [0, 0, 0], definitions: [], layout: { positions: [0, 3] } };
    const sample = x => discover({ state, edge: 0, a: [x, 0, 0], b: [x, 0, 5],
        coordinateA: 0, coordinateB: 5, radius: 1 });
    return { field, discover, state, sample };
}

test('discovery integration skips clear segments, queries again near the wall and preserves reference witness rows', () => {
    const fast = discoveryFixture(), reference = discoveryFixture({ queryReuse: false });
    for (const x of [0, 1, 2, 4, 6, 8]) {
        fast.sample(x); reference.sample(x);
        assert.deepEqual([...(fast.state.pendingVesselRows?.keys() ?? [])], [...(reference.state.pendingVesselRows?.keys() ?? [])]);
    }
    assert.equal(fast.field.calls.length, 1); assert.equal(reference.field.calls.length, 6);
    fast.sample(8.6); reference.sample(8.6);
    assert.equal(fast.field.calls.length, 2); assert.equal(fast.field.calls.at(-1).knownInside, true);
    assert.equal(fast.state.pendingVesselRows.size, 6);
    assert.deepEqual([...fast.state.pendingVesselRows.keys()], [...reference.state.pendingVesselRows.keys()]);
    // A large motion invalidates the inside proof and uses an ordinary exact
    // classification; an outside query must not leave a reusable certificate.
    assert.throws(() => fast.sample(10.1), /crossed the vessel surface/);
    assert.equal(fast.field.calls.at(-1).knownInside, false);
    assert.equal(fast.discover.discoveryCache.size, 0);
    fast.sample(0); assert.equal(fast.field.calls.length, 4);
    assert.equal(fast.field.calls.at(-1).knownInside, false);
});

test('discovery integration keeps cold, winner-only, non-mesh and interrupted queries uncached', () => {
    for (const options of [{ winnerOnly: true }, { source: 'sparse-sdf' }]) {
        const f = discoveryFixture(options); f.sample(0); f.sample(0);
        assert.equal(f.field.calls.length, 2); assert.equal(f.discover.discoveryCache.size, 0);
    }
    const f = discoveryFixture();
    f.field.fail = true; assert.throws(() => f.sample(0), /interrupted/);
    assert.equal(f.discover.discoveryCache.size, 0);
    f.field.fail = false; f.sample(0); assert.equal(f.field.calls.length, 2);
    f.sample(0); assert.equal(f.field.calls.length, 2);
    // The descriptor uses world endpoints; translating the origin must consume
    // the old clearance exactly as translating the local centerline would.
    f.state.origin[0] = 8.7; f.sample(0);
    assert.equal(f.field.calls.length, 3); assert.equal(f.state.pendingVesselRows.size, 6);
    f.field.fallbackGeometry.boundsTree = {}; f.sample(0);
    assert.equal(f.field.calls.length, 4); assert.equal(f.field.calls.at(-1).knownInside, false);
});
