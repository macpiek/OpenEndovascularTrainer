import test from 'node:test';
import assert from 'node:assert/strict';
import { ExactCoordinateMap } from '../src/contrast/exactCoordinateMap.js';

const formerKey = (x, y, z) => [x, y, z].map(value => Math.abs(value) < 1e-7 ? 0 : value).join(',');

test('coordinate welding retains the exact former zero threshold and Float64 distinctions', () => {
    const map = new ExactCoordinateMap();
    map.set(0, -0, 0, 'zero');
    assert.equal(map.get(-0, 1e-8, -1e-8), 'zero');
    for (const [i, point] of [[1e-7, 0, 0], [-1e-7, 0, 0],
        [1, 0, 0], [1 + Number.EPSILON, 0, 0], [1, Number.EPSILON + 1, 0],
        [1, 0, Number.EPSILON + 1]].entries()) map.set(...point, i);
    assert.equal(map.size, 7);
    assert.equal(map.get(1e-7, 0, 0), 0);
    assert.equal(map.get(-1e-7, 0, 0), 1);
    assert.equal(map.get(1, 0, 0), 2);
    assert.equal(map.get(1 + Number.EPSILON, 0, 0), 3);
    assert.equal(map.get(1, Number.EPSILON + 1, 0), 4);
    assert.equal(map.get(1, 0, Number.EPSILON + 1), 5);
    assert.equal(map.get(1, 1, 1), undefined);
});

test('colliding coordinate hashes remain distinct through growth and head replacement', () => {
    // Force every bucket collision to exercise full-coordinate comparison
    // and rehashing, independently of the chosen hash function.
    class CollidingMap extends ExactCoordinateMap { _hash() { return 0; } }
    const map = new CollidingMap(), points = [];
    for (let i = 0; i < 300; i++) {
        const point = [i * .125, i % 11, -i * .375];
        points.push(point); map.set(...point, {id: i});
    }
    for (let i = 299; i >= 0; i--) assert.equal(map.get(...points[i]).id, i);
    const replacement = {id: 'new territory head'};
    assert.equal(map.set(...points[27], replacement), map);
    assert.equal(map.size, 300);
    assert.equal(map.get(...points[27]), replacement);
    assert.deepEqual([...map.values()].map(value => value.id),
        Array.from({length: 300}, (_, i) => i === 27 ? replacement.id : i));
    assert.equal(map.get(1, 1, 999), undefined);
});

test('numeric keys reproduce string-key lookup and insertion order for repeated Float32 and Float64 positions', () => {
    const map = new ExactCoordinateMap(), reference = new Map(), points = [];
    let seed = 18789;
    const random = () => ((seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0) / 4294967296);
    for (let i = 0; i < 2000; i++) {
        const point = Array.from({length: 3}, () => (random() - .5) * 2000);
        points.push(i % 2 ? point : Array.from(Float32Array.from(point)));
    }
    for (let i = 0; i < 8000; i++) {
        const point = points[Math.floor(random() * points.length)];
        assert.equal(map.get(...point), reference.get(formerKey(...point)));
        const value = i % 7 ? {head: i} : undefined;
        map.set(...point, value); reference.set(formerKey(...point), value);
    }
    assert.equal(map.size, reference.size);
    assert.deepEqual([...map.values()], [...reference.values()]);
    for (const point of points) assert.equal(map.get(...point), reference.get(formerKey(...point)));
});

test('NaN payloads and infinities preserve former coordinate-key identity', () => {
    const payloads = new BigUint64Array([0x7ff8000000000001n, 0x7ff8000000001234n]);
    const nan = new Float64Array(payloads.buffer), map = new ExactCoordinateMap();
    map.set(nan[0], 1, 0, 'first');
    map.set(nan[1], 1, 0, 'second');
    assert.equal(map.size, 1);
    assert.equal(map.get(NaN, 1, 0), 'second');
    map.set(Infinity, -Infinity, 0, 'infinite');
    assert.equal(map.get(Infinity, -Infinity, -0), 'infinite');
    assert.equal(map.size, 2);
});
