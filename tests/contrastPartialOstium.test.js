import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import * as THREE from 'three';
import {partialOstiumFixture, slabConcentration, spatialSlabIntegral, createOstiumBeforeProjection, ostialBolusConcentration} from './helpers/partialOstiumFixture.js';

test('physical slab oracle integrates world-space iodine without a wall or graph lookup', () => {
    const transverse = new THREE.Ray(new THREE.Vector3(3, -255, 100), new THREE.Vector3(0, 0, -1));
    assert.ok(Math.abs(spatialSlabIntegral(transverse, [[80, 90]]) - 10 * 5 / 6) < 1e-10);
    const axial = new THREE.Ray(new THREE.Vector3(0, -250, 0), new THREE.Vector3(0, -1, 0));
    // Empty for first 4.5mm, a .6mm linear ramp, then 4.9mm at C=1.
    assert.ok(Math.abs(spatialSlabIntegral(axial, [[0, 10]]) - 5.2) < 1e-10);
    assert.equal(spatialSlabIntegral(axial, [[0, 2], [8, 10]]), 2,
        'Gaps between blood intervals must not contribute iodine');
});

test('diagnostic fixture reproduces old angle-dependent iodine loss missed by uniform-C tests', () => {
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const f = partialOstiumFixture(data);
    try {
        f.fill(() => 1);
        const uniform = f.sample(f.ray());
        assert.ok(Math.abs(uniform.legacyIodine - uniform.chord) < 1e-8,
            'The old calculation passes a uniform concentration test on the very same problematic ray');
        f.fill((point, edge, path) => path === f.aortaTerritory ? 1 : 0);
        const anterior = f.sample(f.ray());
        assert.ok(anterior.chord > 20);
        const reverse = f.sample(f.ray(180));
        assert.ok(Math.abs(reverse.parentCoreIodine - anterior.parentCoreIodine) < 1e-8,
            'The independent physical iodine integral must be identical for a reversed detector ray');
        assert.ok(anterior.parentCoreIodine > 7,
            'Original geometry contains at least 7mm of independently defined filled aortic core');
        assert.ok(anterior.legacyIodine < .1 * anterior.parentCoreIodine,
            'The frozen pre-fix route-average diagnostic must reproduce the transparent hole');
        const crossed = anterior.intervals.find(hit => hit.range[1] - hit.range[0] > 19);
        assert.notEqual(crossed.entry.path, f.aortaTerritory);
        assert.notEqual(crossed.exit.path, f.aortaTerritory);
        assert.notEqual(crossed.entry.path, crossed.exit.path);
        for (const angle of [-60, -30, 30, 60]) {
            const oblique = f.sample(f.ray(angle));
            assert.ok(oblique.parentCoreIodine > 7);
            assert.ok(oblique.legacyIodine >= oblique.parentCoreIodine,
                `At ${angle} degrees the old method happens to recover the filled parent`);
        }
        f.fill(point => slabConcentration(point.y));
        for (const angle of [-60, -30, 0, 30, 60]) {
            const front = f.sample(f.ray(angle));
            assert.ok(Math.abs(front.spatialSlabIodine - front.chord) < 1e-8,
                'Every point on a horizontal detector ray below this front has C=1');
            assert.ok(front.legacyIodine < .8 * front.spatialSlabIodine,
                `The graph-route field must expose the axial-front mismatch at ${angle} degrees`);
        }
    } finally {f.dispose();}
});

test('production spatial column integration preserves the filled aortic core behind clear ostial branches', async () => {
    const {createFlowColumnGeometry} = await import('../src/contrast/flowColumnGeometry.js');
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const f = partialOstiumFixture(data);
    const spatial = createFlowColumnGeometry(f.network, f.integral, f.built.territories, f.built.continuations);
    try {
        f.fill((point, edge, path) => path === f.aortaTerritory ? 1 : 0);
        const measured = [];
        for (const angle of [0, -30, 30, -60, 60, 180]) {
            const ray = f.ray(angle), reference = f.sample(ray);
            let iodine = 0;
            for (const {entry: a, exit: b, range} of reference.intervals) {
                const mean = a.path === b.path ? f.integral.average(a.path, a.s, b.s) :
                    spatial.spatialAverage(a.path, a.s, b.path, b.s,
                        ray.at(range[0], new THREE.Vector3()), ray.at(range[1], new THREE.Vector3()));
                assert.ok(Number.isFinite(mean) && mean >= 0 && mean <= 1);
                iodine += (range[1] - range[0]) * mean;
            }
            assert.ok(iodine >= reference.parentCoreIodine * .9,
                `${angle} degrees: actual iodine ${iodine.toFixed(4)} must preserve the independently defined filled core ${reference.parentCoreIodine.toFixed(4)}`);
            measured.push(iodine);
        }
        assert.ok(Math.abs(measured[0] - measured.at(-1)) < .01,
            'Reversing the same physical X-ray line must preserve its iodine integral');
    } finally {spatial.dispose(); f.dispose();}
});


test('diagnostic reverse concentration recreates a dark rim beside a missing filled daughter core', () => {
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const f = partialOstiumFixture(data), before = createOstiumBeforeProjection(f);
    const daughter = f.network.edges.find(edge => edge.sourceIndex === 1566).index;
    const iodineLength = intervals => intervals.reduce((sum, [a, b]) => sum + b - a, 0);
    try {
        f.fill((point, edge, path) => path === daughter ? 1 : 0);
        const row = [3.4, 3.6, 3.8].map(x => {
            const ray = f.ray(0, x, -256.4), reference = f.sample(ray);
            return {current: before.iodine(ray, reference), core: iodineLength(f.branchCoreOutsideParent(ray, daughter)), reference};
        });
        assert.ok(row[0].core > .85 && row[1].core > .85 && row[2].core > .6);
        assert.ok(row[0].core > row[2].core,
            'The transparent center has MORE independently filled daughter core than the dark rim');
        assert.equal(row[0].current, 0); assert.equal(row[1].current, 0);
        assert.ok(row[2].current > 2, 'A 0.2mm detector shift exposes the artificial surface-label step');
        assert.ok(row[0].reference.intervals.every(hit => hit.entry.path === f.aortaTerritory && hit.exit.path === f.aortaTerritory),
            'Both surface labels can claim aorta while the ray traverses a separate filled daughter segment');
    } finally {before.dispose(); f.dispose();}
});


test('production junction projection preserves a filled daughter and a finite ostial bolus across surface label changes', async () => {
    const {createFlowColumnGeometry} = await import('../src/contrast/flowColumnGeometry.js');
    const {createFlowJunctionProjection} = await import('../src/contrast/flowJunctionProjection.js');
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const f = partialOstiumFixture(data);
    const geometry = createFlowColumnGeometry(f.network, f.integral, f.built.territories, f.built.continuations);
    const junction = createFlowJunctionProjection(f.network, f.integral, f.built.territories, geometry);
    const daughter = f.network.edges.find(edge => edge.sourceIndex === 1566).index;
    try {
        f.fill((point, edge, path) => path === f.aortaTerritory ? 1 : 0);
        for (const angle of [0, -30, 30, -60, 60, 180]) {
            const ray = f.ray(angle), reference = f.sample(ray);
            let iodine = 0;
            for (const {entry: a, exit: b, range} of reference.intervals) iodine += (range[1] - range[0]) *
                junction.average(a.path, a.s, b.path, b.s,
                    ray.at(range[0], new THREE.Vector3()), ray.at(range[1], new THREE.Vector3()));
            assert.ok(iodine >= .9 * reference.parentCoreIodine,
                'Adding hidden daughter candidates must retain the filled parent behind clear branches');
        }
        for (const partial of [false, true]) {
            f.fill((point, edge, path) => path !== daughter ? 0 : partial ?
                ostialBolusConcentration(f.integral.edgeStart[edge.index] + point.clone().sub(edge.start).dot(edge.axis)) : 1);
            const measured = [];
            for (const angle of [0, -30, 30, -60, 60, 180]) for (const x of [3.4, 3.6, 3.8]) {
                const ray = f.ray(angle, x, -256.4, 19.9), reference = f.sample(ray);
                const core = f.branchCoreOutsideParent(ray, daughter, .7, partial ? [11, 13] : null)
                    .reduce((sum, [a, b]) => sum + b - a, 0);
                let iodine = 0;
                for (const {entry: a, exit: b, range} of reference.intervals) iodine += (range[1] - range[0]) *
                    junction.average(a.path, a.s, b.path, b.s,
                        ray.at(range[0], new THREE.Vector3()), ray.at(range[1], new THREE.Vector3()));
                assert.ok(core > .6, 'Each ray crosses a guaranteed filled daughter core outside the clear aorta');
                assert.ok(iodine >= .9 * core,
                    `${partial ? 'Finite bolus' : 'Filled daughter'}, ${angle} degrees, x=${x}: iodine ${iodine} must preserve core ${core}`);
                measured.push(iodine);
            }
            for (let i = 0; i < 3; i++) assert.ok(Math.abs(measured[i] - measured[15 + i]) < .01,
                'The finite iodine integral is symmetric when the detector ray is reversed');
        }
        // Detector-edge regression: aorta-sized mixing/fade widths must
        // not suppress the core of a much narrower, independently filled arm.
        f.fill((point, edge, path) => path === daughter ? 1 : 0);
        for (const [pixelX, pixelY] of [[35, 58], [28, 66], [29, 65], [28, 67]]) {
            const x = -.5 + (pixelX + .5) / 12, y = -260 + (pixelY + .5) / 12;
            const ray = f.ray(0, x, y, 19.9), reference = f.sample(ray);
            const core = f.branchCoreOutsideParent(ray, daughter).reduce((sum, [a, b]) => sum + b - a, 0);
            let iodine = 0;
            for (const {entry: a, exit: b, range} of reference.intervals) iodine += (range[1] - range[0]) *
                junction.average(a.path, a.s, b.path, b.s,
                    ray.at(range[0], new THREE.Vector3()), ray.at(range[1], new THREE.Vector3()));
            assert.ok(core >= .3, 'The pixel crosses a finite daughter core outside the parent');
            assert.ok(iodine / core >= .65,
                `Detector pixel ${pixelX},${pixelY}: aorta-scale fading erases the narrow daughter core (${iodine / core})`);
        }
    } finally {junction.dispose(); geometry.dispose(); f.dispose();}
});
