import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {mergeGeometries} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {MeshBVH} from 'three-mesh-bvh';
import {closeLumenBoundaries} from '../src/contrast/lumenBoundaryCaps.js';

function inwardTube({closed = false, x = 0, radialSegments = 32} = {}) {
    const indexed = new THREE.CylinderGeometry(3, 3, 24, radialSegments, 4, !closed).translate(x, 0, 0);
    const geometry = indexed.toNonIndexed(); indexed.dispose();
    const p = geometry.attributes.position.array;
    for (let i = 0; i < p.length; i += 9) for (let k = 0; k < 3; k++)
        [p[i + 3 + k], p[i + 6 + k]] = [p[i + 6 + k], p[i + 3 + k]];
    geometry.computeVertexNormals();
    return geometry;
}

function assertAxialClosure(geometry, centerX = 0) {
    const tree = new MeshBVH(geometry, {indirect: true});
    for (const direction of [-1, 1]) for (const fraction of [0, .25, .75, .95])
        for (let k = 0; k < (fraction ? 32 : 1); k++) {
            const angle = k * Math.PI / 16;
            const ray = new THREE.Ray(new THREE.Vector3(centerX + 3 * fraction * Math.cos(angle), 0,
                3 * fraction * Math.sin(angle)), new THREE.Vector3(0, direction, 0));
            const hit = tree.raycastFirst(ray, THREE.DoubleSide);
            assert.ok(hit, `Unclosed axial ray at fraction ${fraction}, angle ${angle}`);
            assert.ok(Math.abs(hit.distance - 12) < 1e-6);
            assert.ok(hit.face.normal.dot(ray.direction) < -.999999, 'Cap must face into the blood volume');
        }
    return tree;
}

test('open inward lumen closes at its existing contour with inward cap winding', () => {
    const geometry = inwardTube(), count = geometry.attributes.position.count;
    const original = Object.fromEntries(Object.entries(geometry.attributes)
        .map(([name, a]) => [name, a.array.slice()]));
    const vertexKeys = new Set(Array.from({length: count}, (_, i) =>
        new THREE.Vector3().fromBufferAttribute(geometry.attributes.position, i).toArray().join(',')));
    try {
        const closed = closeLumenBoundaries(geometry);
        assert.deepEqual(closed, {triangles: 60, loops: 2, boundaryEdges: 64, skippedLoops: 0});
        for (const [name, a] of Object.entries(geometry.attributes))
            assert.deepEqual(a.array.slice(0, original[name].length), original[name]);
        const p = geometry.attributes.position;
        for (let i = count; i < p.count; i++) {
            assert.ok(vertexKeys.has(new THREE.Vector3().fromBufferAttribute(p, i).toArray().join(',')));
            assert.equal(Math.abs(p.getY(i)), 12);
        }
        assertAxialClosure(geometry);
        const cappedCount = p.count;
        const again = closeLumenBoundaries(geometry);
        assert.equal(again.triangles, 0); assert.equal(again.boundaryEdges, 0);
        assert.equal(geometry.attributes.position.count, cappedCount);
    } finally {geometry.dispose();}
});

test('an already closed original lumen receives no duplicate cap triangles', () => {
    const geometry = inwardTube({closed: true}), count = geometry.attributes.position.count;
    try {
        const result = closeLumenBoundaries(geometry);
        assert.equal(result.triangles, 0); assert.equal(result.boundaryEdges, 0);
        assert.equal(geometry.attributes.position.count, count);
        assertAxialClosure(geometry);
    } finally {geometry.dispose();}
});

test('separate terminal contours close individually without bridging neighbouring lumens', () => {
    const parts = [inwardTube({x: -5}), inwardTube({x: 5})], geometry = mergeGeometries(parts);
    parts.forEach(p => p.dispose());
    try {
        const result = closeLumenBoundaries(geometry);
        assert.equal(result.loops, 4); assert.equal(result.triangles, 120);
        const tree = assertAxialClosure(geometry, -5); assertAxialClosure(geometry, 5);
        for (const x of [-1.9, 0, 1.9]) assert.equal(tree.raycastFirst(
            new THREE.Ray(new THREE.Vector3(x, 40, 0), new THREE.Vector3(0, -1, 0)), THREE.DoubleSide), null);
    } finally {geometry.dispose();}
});

test('nonplanar terminal loops and broken side-wall cuts do not become invented caps', () => {
    const nonplanar = inwardTube(), p = nonplanar.attributes.position;
    for (let i = 0; i < p.count; i++) if (p.getY(i) === 12)
        p.setY(i, 12 + .4 * Math.sin(2 * Math.atan2(p.getZ(i), p.getX(i))));
    const broken = inwardTube({radialSegments: 8}), source = broken.attributes.position.array;
    // Remove a full angular wall strip. Its joined boundary follows the wall
    // along the axis and cannot be filled by an anatomical terminal polygon.
    const keep = [];
    for (let i = 0; i < source.length; i += 9) {
        const x = (source[i] + source[i + 3] + source[i + 6]) / 3;
        const z = (source[i + 2] + source[i + 5] + source[i + 8]) / 3;
        if (x > 0 && z > 2) continue;
        keep.push(...source.slice(i, i + 9));
    }
    broken.deleteAttribute('normal'); broken.deleteAttribute('uv');
    broken.setAttribute('position', new THREE.Float32BufferAttribute(keep, 3));
    try {
        const a = closeLumenBoundaries(nonplanar);
        assert.equal(a.loops, 1); assert.equal(a.skippedLoops, 1);
        const b = closeLumenBoundaries(broken);
        assert.equal(b.triangles, 0); assert.ok(b.skippedLoops > 0);
    } finally {nonplanar.dispose(); broken.dispose();}
});

test('numerical signed-zero seams do not split the terminal contour', () => {
    const geometry = inwardTube(), p = geometry.attributes.position;
    for (let i = 0; i < p.count; i++) for (const axis of ['X', 'Z'])
        if (Math.abs(p[`get${axis}`](i)) < 1e-7) p[`set${axis}`](i, i % 2 ? -5e-8 : 5e-8);
    try {
        const result = closeLumenBoundaries(geometry);
        assert.equal(result.loops, 2); assert.equal(result.skippedLoops, 0);
        assertAxialClosure(geometry);
    } finally {geometry.dispose();}
});
