import test from 'node:test';
import assert from 'node:assert/strict';
import { FlowContrastVolume } from '../src/contrast/flowContrastVolume.js';
import {
    ostialVolumeFixture,
    bifurcationVolumeFixture
} from './helpers/contrastOstialVolumeFixture.js';

test('only a connected small daughter shares optical lumen with its continuing trunk', () => {
    for (const detached of [false, true]) {
        const f = ostialVolumeFixture({ detached });
        const volume = new FlowContrastVolume(f.network, f.offsets);
        try {
            const originalMass = f.network.edges.map((e) => e.massMg.slice());
            volume.update(f.plume, 0.3, true);
            const samples = volume.cells.flatMap((s, i) => (s.edge.index === 2 ? [i] : []));
            assert.ok(samples.length > 0);
            assert.equal(
                samples.some((i) => !!volume.owners[i]),
                !detached
            );
            assert.ok(volume.cells.every((s, i) => s.edge.index >= 2 || !volume.owners[i]));
            f.network.edges.forEach((e, i) => assert.deepEqual(e.massMg, originalMass[i]));
            const total = originalMass.reduce((s, a) => s + a.reduce((a, b) => a + b, 0), 0);
            assert.ok(
                Math.abs(volume.current.reduce((a, b) => a + b, 0) - total / 0.6) < 1e-4,
                'ownership clips duplicate geometry, never changes the transported iodine'
            );
        } finally {
            volume.dispose();
        }
    }
});

test('a hidden or graft-replaced owner cannot clip contrast in its daughter', () => {
    const f = ostialVolumeFixture(),
        volume = new FlowContrastVolume(f.network, f.offsets);
    try {
        const i = volume.owners.findIndex(Boolean),
            owner = volume.owners[i];
        assert.ok(i >= 0);
        volume.update(f.plume, 0.3, false);
        assert.ok(volume.mesh.geometry.attributes.lumenOwner.getW(i) > 0);
        owner.edge.graftSections = new Uint8Array(owner.edge.cellCount).fill(1);
        volume.update(f.plume, 0.3, true, true);
        assert.equal(volume.mesh.geometry.attributes.lumenOwner.getW(i), 0);
        volume.update(f.plume, 0.3, true, false);
        assert.ok(volume.mesh.geometry.attributes.lumenOwner.getW(i) > 0);
        owner.edge.transportExcluded = true;
        volume.update(f.plume, 0.3, true);
        assert.equal(volume.mesh.geometry.attributes.lumenOwner.getW(i), 0);
    } finally {
        volume.dispose();
    }
});

test('a balanced fork shares all three limbs without changing iodine or losing history', () => {
    const f = bifurcationVolumeFixture(),
        volume = new FlowContrastVolume(f.network, f.offsets);
    try {
        const masses = f.network.edges.map((e) => e.massMg.slice());
        volume.update(f.plume, 0.3, true);
        assert.equal(volume.junctions.regions.length, 1);
        const region = volume.junctions.regions[0];
        assert.equal(new Set(region.samples.flat().map((s) => s.edge.index)).size, 3);
        const current = region.concentration.map((c) => c.clone());
        f.network.edges[1].massMg.fill(0);
        volume.update(f.plume, 0.3, true);
        assert.deepEqual(region.previousConcentration, current);
        assert.ok(region.concentration.some((c) => c.x === 0 && c.y === 0));
        for (const i of [0, 2]) assert.deepEqual(f.network.edges[i].massMg, masses[i]);
        f.network.edges[0].graftSections = new Uint8Array(f.network.edges[0].cellCount).fill(1);
        volume.update(f.plume, 0.3, true, true);
        assert.equal(region.mesh.visible, false);
        for (let i = 0; i < volume.cellJunctions.length; i++)
            if (volume.cellJunctions[i])
                assert.equal(volume.mesh.geometry.attributes.junctionRegion.getW(i), 0);
        volume.junctions.resetHistory();
        assert.ok(region.previousConcentration.every((c) => c.lengthSq() === 0));
    } finally {
        volume.dispose();
    }
});

test('a short bolus between junction sample points remains visible', () => {
    const f = bifurcationVolumeFixture(),
        volume = new FlowContrastVolume(f.network, f.offsets);
    try {
        f.network.edges.forEach((e) => e.massMg.fill(0));
        // This cell lies between the coarser geometric samples, not on one.
        f.network.edges[1].massMg[2] = 1;
        volume.update(f.plume, 0.3, true);
        const region = volume.junctions.regions[0];
        assert.ok(region.mesh.visible);
        assert.ok(region.concentration.some((c) => c.x > 0 || c.y > 0));
        assert.equal(f.network.edges[1].massMg[2], 1);
    } finally {
        volume.dispose();
    }
});
