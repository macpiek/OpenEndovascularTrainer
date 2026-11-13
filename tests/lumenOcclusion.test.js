import test from 'node:test';
import assert from 'node:assert/strict';
import {BoxGeometry, DoubleSide, Ray, TorusGeometry, Vector3} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createLumenOcclusionTest} from '../src/contrast/lumenOcclusion.js';

for (const indirect of [false, true]) for (const shape of ['box', 'torus'])
test(`boolean lumen occlusion matches nearest-hit visibility (${shape}, indirect=${indirect})`, () => {
    const geometry = shape === 'box' ? new BoxGeometry(10, 8, 6, 3, 3, 3) : new TorusGeometry(7, 2, 9, 19);
    const tree = new MeshBVH(geometry, {indirect, maxLeafTris: 4});
    const blocked = createLumenOcclusionTest(tree), ray = new Ray();
    try {
        for (let i = 0; i < 4000; i++) {
            ray.origin.set(Math.sin(i * .43) * 15, Math.cos(i * .37) * 12, Math.sin(i * .71) * 10);
            ray.direction.set(Math.cos(i * .31), Math.sin(i * .11), Math.cos(i * .73)).normalize();
            if (i % 4 === 0) ray.direction.set(1, 0, 0);
            if (i % 4 === 1) ray.direction.set(0, -1, 0);
            if (i % 5 === 0) ray.origin.set(0, 0, 0);
            if (i % 9 === 0) ray.origin.set(5, 4, 3);
            const near = i % 7 === 0 ? 0 : 1e-4;
            const far = i % 3 === 0 ? Infinity : (i % 19) * .9;
            assert.equal(blocked(ray, near, far), !!tree.raycastFirst(ray, DoubleSide, near, far),
                `Ray ${i}, near=${near}, far=${far}`);
        }
        for (const [origin, direction, near, far] of [
            [[0, 0, 0], [1, 0, 0], 5, 5],
            [[5, 0, 0], [1, 0, 0], 0, 0],
            [[5, 4, 3], [-1, 0, 0], 0, Infinity],
            [[5, 4, 3], [0, 0, -1], 1e-4, 1],
            [[0, 0, 0], [0, 0, 1], 2, 1]
        ]) {
            ray.origin.fromArray(origin); ray.direction.fromArray(direction);
            assert.equal(blocked(ray, near, far), !!tree.raycastFirst(ray, DoubleSide, near, far));
        }
    } finally {geometry.dispose();}
});

test('minimal wall adapters retain their existing raycast contract', () => {
    const ray = new Ray(new Vector3(), new Vector3(0, 1, 0));
    const calls = [];
    const blocked = createLumenOcclusionTest({raycastFirst(...args) {calls.push(args); return args[3] > 1 ? {} : null;}});
    assert.equal(blocked(ray, .01, 2), true);
    assert.equal(blocked(ray, .01, .5), false);
    assert.deepEqual(calls[0], [ray, DoubleSide, .01, 2]);
    assert.equal(createLumenOcclusionTest(null)(ray, 0, Infinity), false);
});
