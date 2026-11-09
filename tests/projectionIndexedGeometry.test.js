import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createProjectionIndexedGeometry} from '../src/contrast/projectionIndexedGeometry.js';

const canonical = value => value === 0x80000000 ? 0 : value;

function assertSameCorners(source, indexed) {
    const sourcePosition = source.getAttribute('position'), target = indexed.getAttribute('position');
    const scratch = new Float32Array(3), scratchBits = new Uint32Array(scratch.buffer);
    const targetBits = new Uint32Array(target.array.buffer);
    const cornerCount = source.index?.count ?? sourcePosition.count;
    assert.equal(indexed.index.count, cornerCount);
    for (let corner = 0; corner < cornerCount; corner++) {
        const from = source.index?.getX(corner) ?? corner, to = indexed.index.getX(corner);
        scratch[0] = sourcePosition.getX(from); scratch[1] = sourcePosition.getY(from); scratch[2] = sourcePosition.getZ(from);
        for (let axis = 0; axis < 3; axis++) assert.equal(targetBits[to * 3 + axis], canonical(scratchBits[axis]),
            `Triangle corner ${corner}, axis ${axis} changed during exact indexing`);
    }
}

test('exact indexing welds signed zero but keeps adjacent Float32 coordinates and triangle winding', () => {
    const next = new Float32Array(new Uint32Array([0x3f800001]).buffer)[0];
    const positions = new Float32Array([
        0, 0, 0, 1, 0, 0, 0, 1, 0,
        -0, -0, 0, 0, 1, -0, next, 0, 0
    ]);
    const source = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const before = positions.slice(), indexed = createProjectionIndexedGeometry(source);
    try {
        assert.equal(indexed.attributes.position.count, 4);
        assert.deepEqual(Array.from(indexed.index.array), [0, 1, 2, 0, 2, 3]);
        assertSameCorners(source, indexed);
        assert.deepEqual(positions, before);
        assert.equal(source.index, null);
    } finally {indexed.dispose(); source.dispose();}
});

test('indexed input preserves corner order, groups and bounds without touching seam attributes', () => {
    const source = new THREE.BufferGeometry();
    source.setAttribute('position', new THREE.Float32BufferAttribute([
        0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 2, 0, -2, 0, 0
    ], 3));
    source.setAttribute('normal', new THREE.Float32BufferAttribute([
        0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0, -1
    ], 3));
    source.setIndex([0, 1, 2, 3, 4, 5]);
    source.addGroup(0, 3, 2); source.addGroup(3, 3, 5); source.setDrawRange(3, 3);
    source.computeBoundingBox(); source.computeBoundingSphere();
    const positions = source.attributes.position.array.slice(), normals = source.attributes.normal.array.slice();
    const index = source.index.array.slice(), groups = structuredClone(source.groups);
    const indexed = createProjectionIndexedGeometry(source);
    try {
        assert.equal(indexed.attributes.position.count, 4);
        assertSameCorners(source, indexed);
        assert.deepEqual(indexed.groups, groups);
        assert.notEqual(indexed.groups[0], source.groups[0]);
        assert.deepEqual(indexed.drawRange, source.drawRange);
        assert.deepEqual(indexed.boundingBox, source.boundingBox);
        assert.deepEqual(indexed.boundingSphere, source.boundingSphere);
        assert.notEqual(indexed.boundingBox, source.boundingBox);
        assert.equal(indexed.attributes.normal, undefined, 'Only positions belong to the private projection BVH');
        assert.deepEqual(source.attributes.position.array, positions);
        assert.deepEqual(source.attributes.normal.array, normals);
        assert.deepEqual(source.index.array, index);
        assert.deepEqual(source.groups, groups);
    } finally {indexed.dispose(); source.dispose();}
});

test('typed hash resizing preserves all unique coordinates and duplicate ownership', () => {
    const unique = 70000, count = unique + 20000;
    const positions = new Float32Array(count * 3);
    for (let i = 0; i < count; i++) {
        const original = i < unique ? i : (i * 7919) % unique;
        positions[i * 3] = original;
        positions[i * 3 + 1] = (original % 97) / 8;
        positions[i * 3 + 2] = -(original % 131) / 16;
    }
    const source = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const indexed = createProjectionIndexedGeometry(source);
    try {
        assert.equal(indexed.attributes.position.count, unique);
        assert.ok(indexed.index.array instanceof Uint32Array);
        for (let i = 0; i < count; i++) assert.equal(indexed.index.array[i], i < unique ? i : (i * 7919) % unique);
        assertSameCorners(source, indexed);
    } finally {indexed.dispose(); source.dispose();}
});

test('interleaved Float32 positions are indexed without including unrelated components', () => {
    const interleaved = new THREE.InterleavedBuffer(new Float32Array([
        7, 0, 0, 0, 9, 8, 1, 0, 0, 10, 11, -0, 0, 0, 12
    ]), 5);
    const source = new THREE.BufferGeometry().setAttribute('position', new THREE.InterleavedBufferAttribute(interleaved, 3, 1));
    const before = interleaved.array.slice(), indexed = createProjectionIndexedGeometry(source);
    try {
        assert.equal(indexed.attributes.position.count, 2);
        assert.deepEqual(Array.from(indexed.index.array), [0, 1, 0]);
        assertSameCorners(source, indexed);
        assert.deepEqual(interleaved.array, before);
    } finally {indexed.dispose(); source.dispose();}
});

test('empty geometry is valid and unsupported position storage is rejected explicitly', () => {
    const empty = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    const indexed = createProjectionIndexedGeometry(empty);
    assert.equal(indexed.attributes.position.count, 0);
    assert.equal(indexed.index.count, 0);
    const doubles = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float64Array(9), 3));
    assert.throws(() => createProjectionIndexedGeometry(doubles), /Float32 position/);
    indexed.dispose(); empty.dispose(); doubles.dispose();
});
