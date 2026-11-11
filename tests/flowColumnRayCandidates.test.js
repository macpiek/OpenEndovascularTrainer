import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createFlowColumnRayCandidates} from '../src/contrast/flowColumnRayCandidates.js';
import {createFlowColumnGeometry} from '../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';
import {partialOstiumFixture} from './helpers/partialOstiumFixture.js';

function geometry(paths) {
    const records = [], ranges = new Map();
    for (const [path, points] of paths.entries()) {
        ranges.set(path, {start: records.length / 4, count: points.length});
        for (const point of points) records.push(...point);
    }
    return {ranges, data: Float32Array.from(records)};
}

test('ray broad phase retains every sampled interior point of bent variable-radius atlas tubes', () => {
    const paths = Array.from({length: 40}, (_, path) => Array.from({length: 12}, (_, i) =>
        [path * 3 + Math.sin(i * .4), i * 1.7 - 255, Math.cos(i * .3) * 2, .05 + ((path + i) % 9) * .11]));
    const spatial = geometry(paths), candidates = createFlowColumnRayCandidates(spatial);
    for (const [path, range] of spatial.ranges) for (let i = 0; i < range.count - 1; i++) {
        const at = (range.start + i) * 4, data = spatial.data, fraction = .371;
        const center = new THREE.Vector3().fromArray(data, at).lerp(new THREE.Vector3().fromArray(data, at + 4), fraction);
        const radius = data[at + 3] * (1 - fraction) + data[at + 7] * fraction;
        for (let axis = 0; axis < 3; axis++) {
            const point = center.clone().setComponent(axis, center.getComponent(axis) + radius * .99999);
            const entry = point.clone().add(new THREE.Vector3(-.003, -.005, -.007));
            const exit = point.clone().add(new THREE.Vector3(.003, .005, .007));
            assert.ok(candidates.query(entry, exit).paths.has(path), `thin core ${path}:${i}:${axis}`);
            assert.ok(candidates.query(exit, entry).paths.has(path), 'ray reversal cannot remove a candidate');
        }
    }
});

test('global deformation padding retains a displaced ray while distant tubes are rejected', () => {
    const candidates = createFlowColumnRayCandidates(geometry([[[0, 0, 0, .1], [0, 10, 0, .1]], [[50, 0, 0, 1], [50, 10, 0, 1]]]), {leafSize: 1});
    const entry = new THREE.Vector3(.5, 5, -1), exit = new THREE.Vector3(.5, 5, 1);
    assert.equal(candidates.query(entry, exit).paths.has(0), false);
    assert.deepEqual([...candidates.query(entry, exit, {padding: .5}).paths], [0]);
});

test('a bounded ray cache reports overflow rather than silently dropping a late arm', () => {
    const candidates = createFlowColumnRayCandidates(geometry(Array.from({length: 41}, (_, path) =>
        [[0, 0, 0, 2], [Math.sin(path) * .1, 10, Math.cos(path) * .1, 2]])));
    const entry = new THREE.Vector3(0, 4, -1), exit = new THREE.Vector3(0, 4, 1);
    const full = candidates.query(entry, exit), bounded = candidates.query(entry, exit, {limit: 16});
    assert.equal(full.paths.size, 41); assert.equal(full.overflow, false);
    assert.equal(bounded.overflow, true);
    assert.ok(bounded.paths.size > 16, 'caller must use its uncapped reference path');
    assert.ok(candidates.nodes.every((node, index) => node.escape > index));
});

test('real ostial ray shortlists preserve every concentration, owner and support value of the uncapped field', () => {
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const f = partialOstiumFixture(data);
    const spatial = createFlowColumnGeometry(f.network, f.integral, f.built.territories, f.built.continuations);
    const junction = createFlowJunctionProjection(f.network, f.integral, f.built.territories, spatial);
    try {
        f.fill((point, edge, path) => .1 + .8 * Math.abs(Math.sin(path * .17 + point.y * .051)));
        let compared = 0, valid = 0;
        for (const angle of [0, 30, -30, 180]) for (const x of [-3, 0, 3]) for (const y of [-260, -255, -250]) {
            const ray = f.ray(angle, x, y), reference = f.sample(ray);
            for (const hit of reference.intervals) {
                const a = hit.entry, b = hit.exit, entry = ray.at(hit.range[0], new THREE.Vector3()), exit = ray.at(hit.range[1], new THREE.Vector3());
                const cache = junction.rayCache(entry, exit, a.path, b.path);
                if (cache.valid) valid++;
                for (let i = 0; i <= 24; i++) {
                    const point = entry.clone().lerp(exit, i / 24), expectedState = {}, actualState = {};
                    const expected = junction.pointValue(point, a.path, a.s, b.path, b.s, expectedState);
                    const actual = junction.pointValue(point, a.path, a.s, b.path, b.s, actualState, cache);
                    assert.equal(actual, expected);
                    assert.deepEqual(actualState, expectedState);
                    compared++;
                }
            }
        }
        assert.ok(valid > 30 && compared > 750, 'exercise real curved and oblique anatomical rays');
    } finally {junction.dispose(); spatial.dispose(); f.dispose();}
});
