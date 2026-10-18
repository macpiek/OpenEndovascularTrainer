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

test('private discovery forks isolate commits, aborts, evictions and exported snapshots',()=>{
    for(const retainOnAbort of [false,true]) {
        const original=createSharedAxisDiscoveryCache({capacity:2,retainOnAbort}),d=descriptor();
        certify(original,d);certify(original,descriptor({geometryToken:{}}),()=>10,'other-mesh');
        const incoming=original.capture(geometryToken),fork=original.fork(geometryToken);
        assert.equal(fork.size,1);assert.deepEqual(fork.capture(geometryToken),incoming);
        const snapshot=fork.capture(geometryToken);snapshot[0][1].a[0]=999;
        assert.equal(fork.lookup('edge',d).skip,true);
        fork.begin('edge',d,{insideCertified:true}).abort();
        assert.equal(fork.lookup('edge',d).skip,retainOnAbort);
        certify(fork,d,()=>3);certify(fork,d,()=>4,'new-a');certify(fork,d,()=>5,'new-b');
        assert.equal(fork.lookup('edge',d).skip,false,'private eviction');
        assert.deepEqual(original.capture(geometryToken),incoming);
        const child=fork.fork();fork.clear();
        assert.equal(child.size,2);assert.equal(original.size,2);
        const copied=child.capture(geometryToken);original.restore([],geometryToken);
        assert.deepEqual(child.capture(geometryToken),copied);
    }
});

test('clearance proof retains the certified pose across repeated skips and distinguishes inside from clear', () => {
    const cache = createSharedAxisDiscoveryCache({retainOnAbort:true}), d = descriptor();
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
    const cache = createSharedAxisDiscoveryCache({retainOnAbort:true}), d = descriptor(); certify(cache, d);
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

test('remeshed points inherit only discrete sample balls on the same surface', () => {
    const cache = createSharedAxisDiscoveryCache(), d = descriptor();
    certify(cache, d, () => .2);
    assert.equal(cache.containsInsidePoint([.1, 0, 5.1], geometryToken), true, 'a new tip can inherit the old endpoint ball');
    assert.equal(cache.containsInsidePoint([0, 0, 2.1], geometryToken), true, 'interior samples also certify nearby points');
    assert.equal(cache.containsInsidePoint([0, 0, 2.5], geometryToken), false, 'the line between samples is not certified');
    assert.equal(cache.containsInsidePoint([.2, 0, 2], geometryToken), false, 'touching the ball boundary cannot prove inside');
    assert.equal(cache.containsInsidePoint([.21, 0, 2], geometryToken), false, 'crossing a wall at x=.2 remains rejected');
    assert.equal(cache.containsInsidePoint([0, 0, 2], {}), false);
    for (const point of [null, [], [NaN, 0, 0], [Infinity, 0, 0]])
        assert.equal(cache.containsInsidePoint(point, geometryToken), false);
    const restored = createSharedAxisDiscoveryCache(), nextToken = {};
    restored.restore(cache.capture(geometryToken), nextToken);
    assert.equal(restored.containsInsidePoint([.1, 0, 5.1], nextToken), true);
    restored.clear(); assert.equal(restored.containsInsidePoint([0, 0, 2], nextToken), false);
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
        const cache = createSharedAxisDiscoveryCache({retainOnAbort:true}); certify(cache, d);
        const capture = cache.begin('edge', d, { insideCertified: true }); fill(capture);
        assert.equal(capture.commit(), false); assert.equal(cache.size, 1);
        assert.equal(cache.lookup('edge',d).skip,true,'Incomplete query preserves the earlier immutable proof');
    }
    const cache = createSharedAxisDiscoveryCache({retainOnAbort:true}); certify(cache, d);
    const capture = cache.begin('edge', d, { insideCertified: true }); capture.visit(contact(10), 0); capture.abort();
    assert.equal(capture.commit(), false); assert.equal(cache.lookup('edge', d).skip, true);
    assert.equal(cache.lookup('edge',descriptor({a:[11,0,0],b:[11,0,5]})).knownInside,false,'Retained proof never certifies the failed distant trial');
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
        const cache = createSharedAxisDiscoveryCache({retainOnAbort:true});
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
            const point = moved.a.map((v, k) => (1 - t) * v + t * moved.b[k]);
            if (cache.containsInsidePoint(point, geometryToken)) assert.ok(actual > 0, 'point evidence cannot cross any of the box walls');
        }
    }
    assert.ok(skips > 100); assert.ok(inside > skips);
});

test('distance error allowance and large-coordinate guard reduce skips; storage is bounded and resettable', () => {
    const cache = createSharedAxisDiscoveryCache({ capacity: 2,retainOnAbort:true }), d = descriptor();
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

function discoveryFixture({ queryReuse = true, retainDiscoveryCertificates=true, source = 'sparse-sdf-bvh', winnerOnly = false, candidateMargin=.5 } = {}) {
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
    const discover = createSharedAxisVesselDiscovery(field, 0, { queryReuse,retainDiscoveryCertificates,candidateMargin });
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
    // classification; a rejected query cannot certify its own pose, but the
    // previous pose remains certified and can be reused after rollback.
    assert.throws(() => fast.sample(10.1), /crossed the vessel surface/);
    assert.equal(fast.field.calls.at(-1).knownInside, false);
    assert.equal(fast.discover.discoveryCache.size, 1);
    fast.sample(8.6); assert.equal(fast.field.calls.length, 4);
    assert.equal(fast.field.calls.at(-1).knownInside, true, 'Rollback to the last certified pose retains its proof');
    fast.sample(0); assert.equal(fast.field.calls.length, 5);
    assert.equal(fast.field.calls.at(-1).knownInside, false, 'Large motion still needs fresh classification');
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

test('remeshing repairs a cold inside-sign reversal without suppressing penetrating capsule witnesses', () => {
    const field = {voxelSize: .25, fallbackGeometry: {boundsTree: {}}, reverse: false,
        queryCapsuleSoA(x, y, z, r, ...args) {
            const count = args[8], visitor = args[11];
            let result;
            for (let i = 0; i <= count; i++) {
                const t = i / count, actual = 10 - ((1 - t) * x[0] + t * x[1]);
                const distance = this.reverse ? -Math.abs(actual) : actual;
                result = {...contact(distance), signedGap: distance - r[0], segmentT: t};
                visitor(result, t);
            }
            return result;
        }
    };
    const discover = createSharedAxisVesselDiscovery(field, 0, {retainDiscoveryCertificates: true});
    const state = {origin: [4, 0, 0], definitions: [], layout: {positions: [0, 3]}};
    const input = {state, edge: 0, a: [5, 0, 0], b: [5, 0, 5], coordinateA: 0, coordinateB: 5, radius: 1};
    discover(input); field.reverse = true;
    state.pendingVesselRows.clear();
    assert.doesNotThrow(() => discover({...input, a: [5.1, 0, 0], b: [5.1, 0, 5.2], coordinateB: 5.2}));
    assert.ok(discover.discoveryCache.stats.pointInsideProofs > 0);
    assert.equal(state.pendingVesselRows.size, 7, 'every new near-wall sample still creates a contact constraint');
    assert.throws(() => discover({...input, a: [6.1, 0, 0], b: [6.1, 0, 5.2], coordinateB: 5.2}), /crossed/);
    field.fallbackGeometry.boundsTree = {};
    assert.throws(() => discover(input), /crossed/, 'a different surface invalidates all inside evidence');
});


test('reference solver retains its original cache invalidation policy',()=>{
    const cache=createSharedAxisDiscoveryCache(),d=descriptor();certify(cache,d);
    const capture=cache.begin('edge',d,{insideCertified:true});capture.visit(contact(10),0);capture.abort();
    assert.equal(cache.size,0);
    const f=discoveryFixture({retainDiscoveryCertificates:false});f.sample(8.6);
    assert.throws(()=>f.sample(10.1),/crossed the vessel surface/);
    assert.equal(f.discover.discoveryCache.size,0);
});

test('debug replay retains immutable clearance evidence, binds a new BVH and rejects different anatomy',()=>{
    const mesh=vertices=>({boundsTree:{},getAttribute:()=>({itemSize:3,array:Float32Array.from(vertices)})});
    const vertices=[0,0,0,1,0,0,0,1,0];
    const original=discoveryFixture();original.field.fallbackGeometry=mesh(vertices);original.sample(0);
    const saved=JSON.parse(JSON.stringify(original.discover.captureDiscoveryState()));
    assert.equal(saved.certificates.length,1);
    assert.equal('geometryToken' in saved.certificates[0][1],false);
    const replay=discoveryFixture();replay.field.fallbackGeometry=mesh(vertices);
    replay.discover.restoreDiscoveryState(saved);saved.certificates[0][1].a[0]=100;
    replay.sample(0);assert.equal(replay.field.calls.length,0,'Replay uses the live certificate instead of cold sign classification');
    replay.sample(8.6);assert.equal(replay.field.calls.at(-1).knownInside,true);
    const wrong=discoveryFixture();wrong.field.fallbackGeometry=mesh([0,0,0,2,0,0,0,1,0]);
    assert.throws(()=>wrong.discover.restoreDiscoveryState(original.discover.captureDiscoveryState()),/same vessel mesh/);
    assert.equal(wrong.discover.discoveryCache.size,0);
    replay.field.fallbackGeometry.boundsTree={};replay.sample(8.6);
    assert.equal(replay.field.calls.at(-1).knownInside,false,'A later BVH replacement still invalidates restored evidence');
});

test('restoring numeric certificates validates inputs before replacing the current cache and obeys capacity',()=>{
    const cache=createSharedAxisDiscoveryCache({capacity:2}),d=descriptor();certify(cache,d);
    const saved=cache.capture(geometryToken);assert.equal(saved.length,1);
    const invalid=structuredClone(saved);invalid[0][1].minimumDistance=Infinity;
    assert.throws(()=>cache.restore(invalid,geometryToken),/Invalid discovery/);
    assert.equal(cache.lookup('edge',d).skip,true);
    const restored=createSharedAxisDiscoveryCache({capacity:1}),nextToken={};
    restored.restore([...saved,['new',saved[0][1]]],nextToken);
    assert.equal(restored.size,1);assert.equal(restored.lookup('edge',{...d,geometryToken:nextToken}).skip,false);
    assert.equal(restored.lookup('new',{...d,geometryToken:nextToken}).skip,true);
    assert.equal(restored.lookup('new',d).skip,false);
});

test('private discovery history cannot erase the accepted inside certificates',()=>{
    const f=discoveryFixture({retainDiscoveryCertificates:false});f.sample(8.6);
    const before=f.discover.captureDiscoveryState();
    const candidate=f.discover.forkDiscovery();
    candidate.discoveryCache.clear();
    assert.deepEqual(f.discover.captureDiscoveryState(),before);
    assert.equal(f.discover.discoveryCache.size,1);
    const retry=f.discover.forkDiscovery();
    assert.equal(retry.discoveryCache.size,1);
    assert.notEqual(retry.discoveryCache,f.discover.discoveryCache);
});


test('narrow candidates skip only proven separation and rediscover approaching contacts after a private fork',()=>{
    const f=discoveryFixture({candidateMargin:.01});
    f.sample(0);f.sample(8.8);
    assert.equal(f.field.calls.length,1,'the full sample grid remains beyond the candidate margin');
    assert.equal(f.state.pendingVesselRows?.size??0,0);
    f.sample(8.995);
    assert.equal(f.field.calls.length,2);assert.equal(f.state.pendingVesselRows.size,6);
    const before=f.discover.discoveryCache.capture(f.field.fallbackGeometry.boundsTree);
    const broad=f.discover.forkDiscovery({candidateMargin:.5});
    const state={...f.state,pendingVesselRows:new Map()};
    broad({state,edge:0,a:[8.8,0,0],b:[8.8,0,5],coordinateA:0,coordinateB:5,radius:1});
    assert.equal(state.pendingVesselRows.size,6);
    assert.deepEqual(f.discover.discoveryCache.capture(f.field.fallbackGeometry.boundsTree),before);
    assert.throws(()=>f.sample(10.1),/crossed/);
    for(const margin of [-1,NaN,Infinity,.51])assert.throws(()=>f.discover.forkDiscovery({candidateMargin:margin}),/margin/);
});
