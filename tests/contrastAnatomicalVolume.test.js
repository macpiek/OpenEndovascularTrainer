import test from 'node:test';
import assert from 'node:assert/strict';
import { anatomicalContrastFixture } from './helpers/anatomicalContrastFixture.js';
import { AnatomicalContrastVolume } from '../src/contrast/anatomicalContrastVolume.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { HybridContrastSystem } from '../src/contrast/hybridContrastSystem.js';
import * as THREE from 'three';

for (const sparse of [false, true])
    test(`anatomical atlas shares border concentrations and fills its interior (sparse=${sparse})`, () => {
        const f = anatomicalContrastFixture({ sparse });
        const v = new AnatomicalContrastVolume(f.network, f.offsets, f.field);
        try {
            const mass = f.network.edges.map((e) => Array.from(e.massMg));
            v.update(f.plume, 0.3, false);
            assert.ok(v.current.every((c) => Math.abs(c - 0.1) < 1e-6));
            const a = v.atlas,
                coordinates = new Map();
            for (let b = 0; b < a.brickCount; b++)
                for (let z = 0; z < 2; z++)
                    for (let y = 0; y < 2; y++)
                        for (let x = 0; x < 2; x++) {
                            const key = [
                                a.brickOrigins[b * 3] + x * a.step,
                                a.brickOrigins[b * 3 + 1] + y * a.step,
                                a.brickOrigins[b * 3 + 2] + z * a.step
                            ].join(',');
                            const id =
                                a.brickVertices[b * 8 + x + 2 * y + 4 * z];
                            if (coordinates.has(key))
                                assert.equal(
                                    id,
                                    coordinates.get(key),
                                    'neighbouring bricks must share the same physical sample'
                                );
                            coordinates.set(key, id);
                        }
            const b = Array.from({ length: a.brickCount }, (_, i) => i).find(
                (i) =>
                    a.brickOrigins[i * 3] === 0 &&
                    a.brickOrigins[i * 3 + 1] === 0 &&
                    a.brickOrigins[i * 3 + 2] === 0
            );
            assert.notEqual(
                b,
                undefined,
                'deep lumen interior must be retained'
            );
            const [ax, ay, az] = a.atlasOrigins.slice(b * 3, b * 3 + 3);
            const signal =
                a.data[
                    ax +
                        3 +
                        a.atlasDims[0] * (ay + 3 + a.atlasDims[1] * (az + 3))
                ];
            assert.ok(
                signal > 128,
                'interior must have a positive anatomical distance'
            );
            f.fill(0.06);
            v.update(f.plume, 0.3, true);
            assert.ok(v.previous.every((c) => Math.abs(c - 0.1) < 1e-6));
            assert.ok(v.current.every((c) => Math.abs(c - 0.2) < 1e-6));
            assert.deepEqual(
                f.network.edges.map((e) => Array.from(e.massMg, (m) => m / 2)),
                mass,
                'rendering must preserve iodine'
            );
            let disposed = 0;
            for (const texture of v.textures)
                texture.addEventListener('dispose', () => disposed++);
            v.resetHistory();
            assert.equal(v.mesh.visible, false);
            assert.ok(
                v.currentTexels.every((c) => c === 0) &&
                    v.previousTexels.every((c) => c === 0)
            );
            v.dispose();
            assert.equal(disposed, 3);
        } catch (error) {
            v.dispose();
            throw error;
        }
    });

test('renderer selects anatomy only when a signed anatomical asset is available', () => {
    const system = new HybridContrastSystem({
        centerlineSegments: [
            {
                id: 0,
                start: new THREE.Vector3(0, 16, 0),
                end: new THREE.Vector3(0, -16, 0),
                radiusStart: 1,
                radiusEnd: 1,
                safeRadius: 1,
                nodeStartId: 0,
                nodeEndId: 1
            }
        ],
        localOptions: { capacity: 8 }
    });
    const fallback = new ContrastVolumeRenderer(system);
    try {
        assert.equal(
            fallback._flowVolume instanceof AnatomicalContrastVolume,
            false
        );
    } finally {
        fallback.dispose();
    }
    system.anatomyContactField = anatomicalContrastFixture().field;
    const renderer = new ContrastVolumeRenderer(system);
    try {
        assert.ok(renderer._flowVolume instanceof AnatomicalContrastVolume);
        system.flowNetwork.edges[0].massMg.fill(0.1);
        renderer.update();
        assert.equal(renderer._flowVolume.mesh.visible, true);
        assert.equal(
            renderer.flowMesh.visible,
            false,
            'do not overlay a second proxy lumen'
        );
    } finally {
        renderer.dispose();
    }
});
