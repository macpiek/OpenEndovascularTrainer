import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {junctionContinuityFixture} from './helpers/contrastJunctionContinuityFixture.js';
import {JunctionSurfaceReconstruction} from '../src/contrast/junctionSurfaceReconstruction.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';

function fixture(subdivide = false) {
    const f = junctionContinuityFixture(true, subdivide);
    const field = new JunctionSurfaceReconstruction(f.network, f.field, f.territories, f.continuations);
    field.finish();
    const projection = createFlowJunctionProjection(f.network, f.integral, f.territories, f.geometry, field);
    const originalFill = f.fill, originalDispose = f.dispose;
    return {...f, shared: projection, junction: field.junctions[0],
        fill(profile) {originalFill(profile); projection.shared.update(f.field.values);},
        dispose() {projection.dispose(); originalDispose();}};
}

// The bright daughter meets clear incoming parent blood. In the old column
// renderer its shared node was immediately clipped to twice the clear cell,
// producing a fixed cap instead of a finite common-lumen mixture.
test('a dense daughter mixes into clear parent blood over the physical junction, independently of wall labels', () => {
    const f = fixture();
    try {
        f.fill(edge => edge === f.daughter ? 1 : 0);
        const center = f.junction.center, nodeValue = f.field.values[f.junction.nodeSample];
        assert.ok(nodeValue > 0 && nodeValue < 1);
        const point = center.clone().addScaledVector(f.junction.axis, 1.5);
        const mixed = f.shared.pointValue(point, 0, 28.5, 0, 28.5);
        const otherWall = f.shared.pointValue(point, f.daughter, 0, f.daughter, 0);
        assert.ok(mixed > .01 && mixed < nodeValue, 'A dilute local plume must extend into the common parent lumen');
        assert.ok(Math.abs(mixed - otherWall) < 1e-7, 'Wall triangle ownership must not cut the mixture');
        let previous = f.shared.shared.sample(0, center).value, largestStep = 0;
        for (let d = .05; d <= 8; d += .05) {
            const value = f.shared.shared.sample(0, center.clone().addScaledVector(f.junction.axis, d)).value;
            largestStep = Math.max(largestStep, Math.abs(value - previous));
            assert.ok(value >= 0 && value <= nodeValue + 1e-7);
            assert.ok(value <= previous + 1e-7, 'The common mixture should fade toward the clear inflow');
            previous = value;
        }
        assert.ok(largestStep < .015, `A 50 µm step must not expose a transport-cell interface: ${largestStep}`);
        assert.equal(previous, 0, 'Mixing must remain local to the fork');
    } finally {f.dispose();}
});

test('common-lumen reconstruction preserves a uniform concentration, history, and complete washout', () => {
    const f = fixture();
    try {
        for (const concentration of [0, .37, 1, 0]) {
            f.fill(() => concentration);
            const before = f.field.values.slice();
            for (let x = -4; x <= 4; x += 2) for (let y = -4; y <= 4; y += 2) {
                const value = f.shared.pointValue(new THREE.Vector3(x, y, 0), 0, 30, f.daughter, 0);
                assert.ok(Math.abs(value - concentration) < 1e-6, `Uniform ${concentration} at ${x},${y}: ${value}`);
            }
            assert.deepEqual(f.field.values, before, 'Imaging must not mutate the transported iodine mass or concentration');
        }
        f.fill(() => .8);
        const previous = Float32Array.from(f.field.values, c => c * .25);
        f.shared.shared.update(f.field.values, previous);
        const data = f.shared.shared.history.image.data;
        assert.ok(Math.abs(data[0] - .8) < 1e-6 && Math.abs(data[1] - .2) < 1e-6);
    } finally {f.dispose();}
});

test('a remote bolus cannot illuminate an empty junction and a local moving front is retained', () => {
    const f = fixture();
    try {
        f.fill((edge, s) => edge === f.daughter && s > 12 ? 1 : 0);
        for (let x = -3; x <= 3; x++) for (let y = -3; y <= 3; y++)
            assert.equal(f.shared.pointValue(new THREE.Vector3(x, y, 0), 0, 30, 0, 30), 0);
        f.fill((edge, s) => edge === 0 && s > 23 && s < 25 ? 1 : 0);
        const inFront = f.shared.shared.sample(0, new THREE.Vector3(0, 6, 0));
        assert.ok(inFront.value > .9, 'The local parent bolus must survive; distant anchors must not replace its position');
        assert.equal(f.shared.shared.sample(0, new THREE.Vector3()).value, 0);
    } finally {f.dispose();}
});

test('splitting an upstream segment does not move the common mixing field', () => {
    const whole = fixture(), split = fixture(true);
    try {
        for (const f of [whole, split]) f.fill(edge => edge === f.daughter ? 1 : 0);
        for (let x = -5; x <= 5; x++) for (let y = -8; y <= 8; y++) {
            const point = new THREE.Vector3(x, y, 1);
            const a = whole.shared.pointValue(point, 0, 30, 0, 30);
            const b = split.shared.pointValue(point, 0, 30, 0, 30);
            assert.ok(Math.abs(a - b) < 2e-6, `Graph subdivision altered the spatial field at ${x},${y}: ${a},${b}`);
        }
        assert.equal(whole.shared.shared.hasPath(0, 99), false, 'Other paths in the same spatial box do not share this mixture');
    } finally {whole.dispose(); split.dispose();}
});

test('recorded aortoiliac reflux uses the common field in the production column projector', async () => {
    const {readFileSync} = await import('node:fs');
    const {gunzipSync} = await import('node:zlib');
    const {aortoiliacColumnFixture} = await import('./helpers/aortoiliacColumnFixture.js');
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/aortoiliac-column.json.gz', import.meta.url))));
    const f = aortoiliacColumnFixture(data);
    try {
        f.show(3);
        const {junctions} = f.cpu();
        const item = junctions.shared.items.find(item => Math.abs(item.junction.center.y + 291.300964) < .01);
        assert.ok(item, 'The regression must include the actual aortic bifurcation');
        const expected = f.surface.current[item.junction.nodeSample];
        const history = f.surface.projection.uniforms.columnSharedHistory.value.image.data;
        assert.equal(history[item.index * 4], expected);
        assert.ok(expected > .5 && expected < 1, 'The recording must include dense retrograde iliac contrast');
        const paths = [...new Set(item.arms.map(arm => arm.path))];
        assert.equal(paths.length, 3);
        let checked = 0;
        for (let x = -3; x <= 3; x++) for (let y = -3; y <= 3; y++) for (let z = -3; z <= 3; z++) {
            const point = item.junction.center.clone().add(new THREE.Vector3(x, y, z));
            const one = junctions.pointValue(point, paths[0], item.arms[0].s, paths[0], item.arms[0].s);
            for (const path of paths.slice(1)) {
                const two = junctions.pointValue(point, path, 0, path, 0);
                assert.ok(Math.abs(one - two) < 1e-7, 'The real common lumen must not depend on the entry triangle label');
            }
            assert.ok(Number.isFinite(one) && one >= 0 && one <= 1);
            checked++;
        }
        assert.equal(checked, 343);
    } finally {f.dispose();}
});
