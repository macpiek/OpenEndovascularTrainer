import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { HybridContrastSystem, CONTRAST_SOURCE_SHEATH } from '../src/contrast/hybridContrastSystem.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';

function network() {
    const net = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
        start: new THREE.Vector3(-100, 0, 0), end: new THREE.Vector3(100, 0, 0),
        radiusStart: 1, radiusEnd: 1}],
    {rootPoint: new THREE.Vector3(-100, 0, 0), cellLengthMm: 1, axialDispersionMm2PerS: 0});
    net.setFlowOverride([0], 1000);
    return net;
}
const accounted = net => net.getIodineMassMg() + net.outletIodineMassMg;

test('a continuous high-Courant source has no empty source neighbourhood or detached axial plugs', () => {
    const net = network(), source = 100, dt = 1 / 60, stock = .3, rate = 1000 * stock;
    for (let step = 0; step < 6; step++) {
        net.queueIodineSource(0, source, rate * dt);
        assert.ok(Math.abs(accounted(net) - rate * (step + 1) * dt) < 1e-10,
            'pending source iodine is counted before transport consumes it');
        net.update(dt);
    }
    const edge = net.edges[0];
    for (let cell = source; cell < source + 20; cell++)
        assert.ok(Math.abs(edge.massMg[cell] / edge.volumes[cell] - stock) < 1e-12,
            `the continuous column must stay filled at cell ${cell}`);
    assert.equal(net._advection.queuedSourceVertices.length, 0);
    const injected = accounted(net);
    net.update(.2);
    assert.ok(Math.abs(accounted(net) - injected) < 1e-10, 'consumed source must not be emitted again');
});

test('instantaneous bolus deposition remains instantaneous', () => {
    const net = network();
    net.depositIodine(0, 100, 5);
    assert.equal(net.edges[0].massMg[100], 5);
    net.update(1 / 60);
    assert.equal(net.edges[0].massMg[100], 0);
    assert.ok(Math.abs(accounted(net) - 5) < 1e-12);
});

test('queued source lives across all accuracy substeps with the same constant delivery rate', () => {
    const whole = network(), split = network(), source = 100;
    whole.queueIodineSource(0, source, 30);
    whole.update(0); // Paused/zero-length calls must retain pending delivery.
    assert.equal(whole.getIodineMassMg(), 30);
    whole.update(.1);
    for (let i = 0; i < 3; i++) {
        split.queueIodineSource(0, source, 10);
        split.update(1 / 30);
    }
    assert.equal(whole.lastTransportSubstepCount, 3);
    assert.ok(Math.abs(accounted(whole) - 30) < 1e-10);
    for (let cell = 0; cell < whole.edges[0].cellCount; cell++)
        assert.ok(Math.abs(whole.edges[0].massMg[cell] - split.edges[0].massMg[cell]) < 1e-11);
    assert.equal(whole._advection.queuedSourceVertices.length, 0);
});

test('a divergent source remains continuously filled and splits exactly by its resolved outflows', () => {
    const net = network(), edge = net.edges[0], source = 100, dt = 1 / 60;
    net.setFlowOverride([0], 0);
    net.addFaceFlowDelta(0, 0, source + 1, -500);
    net.addFaceFlowDelta(0, source + 1, edge.cellCount + 1, 1500);
    for (let frame = 0; frame < 6; frame++) {
        net.queueIodineSource(0, source, .3 * 2000 * dt);
        net.update(dt);
    }
    for (let cell = source - 10; cell <= source + 10; cell++)
        assert.ok(Math.abs(edge.massMg[cell] / edge.volumes[cell] - .3) < 1e-11);
    const upstream = edge.massMg.slice(0, source).reduce((a, b) => a + b, 0);
    const downstream = edge.massMg.slice(source + 1).reduce((a, b) => a + b, 0);
    assert.ok(Math.abs(upstream / (upstream + downstream) - .25) < 1e-10);
    assert.ok(Math.abs(accounted(net) - 60) < 1e-10);
});

test('the actual femoral sheath model has a filled source neighbourhood at production 60 Hz', () => {
    const bytes = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
    const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    const a = asset.arrays.centerlineSegments, nodes = asset.arrays.centerlineEdges;
    const stride = asset.metadata.centerline.stride, segments = [];
    for (let index = 0; index < a.length / stride; index++) {
        const o = index * stride;
        segments.push({id: index, nodeStartId: nodes[index * 2], nodeEndId: nodes[index * 2 + 1],
            start: new THREE.Vector3(a[o], a[o + 1], a[o + 2]),
            end: new THREE.Vector3(a[o + 3], a[o + 4], a[o + 5]),
            radiusStart: a[o + 6], radiusEnd: a[o + 7], safeRadius: a[o + 8]});
    }
    const system = new HybridContrastSystem({centerlineSegments: segments,
        sheath: {start: new THREE.Vector3(-89.014, -447.056, 28.020),
            end: new THREE.Vector3(-73, -383, 14), radius: 1}});
    system.startInjection({source: CONTRAST_SOURCE_SHEATH, volumeMl: 100, rateMlPerSec: 15});
    const profiles = [];
    for (let frame = 0; frame < 120; frame++) {
        system.advanceSynchronized(1 / 60);
        if (frame !== 29 && frame !== 59 && frame !== 119) continue;
        const edges = system.flowNetwork.edges, source = system.localSolver.lastSourceEdgeIndex;
        const chain = [source];
        let current = edges[source];
        for (let i = 0; i < 4; i++) {current = edges[current.parentEdgeIndex]; chain.unshift(current.index);}
        current = edges[source];
        for (let i = 0; i < 4; i++) {current = edges[current.childEdgeIndices[0]]; chain.push(current.index);}
        const concentration = chain.map(id => edges[id].massMg[0] / edges[id].volumes[0] / .3);
        profiles.push({time: system.flowNetwork.time, concentration});
        assert.ok(Math.min(...concentration) > .3, `source neighbourhood emptied: ${JSON.stringify(profiles.at(-1))}`);
        assert.ok(Math.max(...concentration) < 1.4, `batch concentration overshoot: ${JSON.stringify(profiles.at(-1))}`);
        const aroundSource = concentration.slice(2, 7);
        assert.ok(Math.max(...aroundSource) - Math.min(...aroundSource) < .4,
            `source-cell trough must not split the column: ${JSON.stringify(profiles.at(-1))}`);
    }
    assert.ok(Math.abs(system.getMetrics().relativeBalanceError) < 1e-10);
    console.log('actual sheath source profiles', JSON.stringify(profiles));
});
