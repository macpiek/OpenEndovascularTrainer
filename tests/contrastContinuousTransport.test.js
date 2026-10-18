import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { FlowConcentrationField } from '../src/contrast/flowConcentrationField.js';
import { ContrastVolumeRenderer } from '../src/contrast/contrastVolumeRenderer.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';

const area = Math.PI * 4;
const segment = (id, a, b, start, end) => ({
    id,
    nodeStartId: a,
    nodeEndId: b,
    start: new THREE.Vector3(...start),
    end: new THREE.Vector3(...end),
    radiusStart: 2,
    radiusEnd: 2
});
function chain(points, options = {}) {
    return new ContrastFlowNetwork(
        points.slice(1).map((x, i) => segment(i, i, i + 1, [points[i], 0, 0], [x, 0, 0])),
        { rootPoint: new THREE.Vector3(points[0], 0, 0), axialDispersionMm2PerS: 0, ...options }
    );
}
function sum(net) {
    return net.getIodineMassMg() + net.outletIodineMassMg;
}
function centroid(net) {
    let mass = 0,
        moment = 0;
    for (const edge of net.edges)
        for (let i = 0; i < edge.cellCount; i++) {
            const m = edge.massMg[i];
            mass += m;
            moment += m * (edge.start.x + (i + 0.5) * edge.cellLength);
        }
    return moment / mass;
}
for (const sign of [1, -1])
    test(`short fragments preserve travel distance at high Courant number (${sign})`, () => {
        const net = chain(Array.from({ length: 101 }, (_, i) => i * 0.01));
        net.setFlowOverride(
            net.edges.map((e) => e.index),
            sign * area * 20
        );
        const start = sign > 0 ? 0 : 90,
            mass = 0.3 * net.edges[start].volumes[0];
        net.depositIodine(start, 0, mass);
        net.update(0.02);
        assert.ok(Math.abs(centroid(net) - ((start + 0.5) * 0.01 + sign * 0.4)) < 1e-6);
        assert.ok(Math.abs(sum(net) - mass) < 1e-10);
        assert.equal(
            net.lastTransportSubstepCount,
            1,
            'tiny cells must not require hundreds of whole-tree steps'
        );
        for (const e of net.edges) assert.ok(e.massMg.every((m) => m >= 0));
    });

test('time profiles keep finite arrival, split by flow, reverse and wash out without junction storage', () => {
    const net = new ContrastFlowNetwork(
        [
            segment(0, 0, 1, [0, 0, 0], [1, 0, 0]),
            segment(1, 1, 2, [1, 0, 0], [2, 1, 0]),
            segment(2, 1, 3, [1, 0, 0], [2, -1, 0])
        ],
        { rootPoint: new THREE.Vector3(0, 0, 0), cellLengthMm: 0.1, axialDispersionMm2PerS: 0 }
    );
    net.setFlowOverride([0], area * 100);
    net.setFlowOverride([1, 2], area * 50);
    const mass = 0.3 * net.edges[0].volumes[0];
    net.depositIodine(0, 0, mass);
    net.update(0.002);
    assert.equal(
        net.edges[1].massMg.reduce((a, b) => a + b, 0),
        0,
        'no instantaneous daughter filling'
    );
    net.update(0.01);
    const daughters = net.edges.slice(1).map((e) => e.massMg.reduce((a, b) => a + b, 0));
    assert.ok(daughters.every((m) => m > 0));
    assert.ok(Math.abs(daughters[0] - daughters[1]) < 1e-12);
    assert.ok(Math.abs(sum(net) - mass) < 1e-10);
    net.setFlowOverride([0], -area * 100);
    net.setFlowOverride([1, 2], -area * 50);
    for (let i = 0; i < 10; i++) net.update(0.01);
    assert.ok(net.getIodineMassMg() < 1e-9);
    assert.ok(Math.abs(sum(net) - mass) < 1e-10);
});

test('closed internal junction retains its donors without losing iodine', () => {
    const net = chain([0, 1, 2]);
    net.setFlowOverride([0], area * 100);
    net.setFlowOverride([1], 0);
    net.depositIodine(0, 0, 1);
    net.update(0.02);
    assert.equal(net.edges[0].massMg[0], 1);
    assert.equal(net.outletIodineMassMg, 0);
});

test('long updates resolve the same arterial pulse as consecutive accuracy steps', () => {
    const a = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    const b = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    a.depositIodine(0, 0, 10);
    b.depositIodine(0, 0, 10);
    a.update(0.1);
    for (let i = 0; i < 3; i++) b.update(1 / 30);
    for (let e = 0; e < a.edges.length; e++)
        for (let i = 0; i < a.edges[e].cellCount; i++)
            assert.ok(Math.abs(a.edges[e].massMg[i] - b.edges[e].massMg[i]) < 1e-10);
    assert.ok(Math.abs(a.outletIodineMassMg - b.outletIodineMassMg) < 1e-10);
});

test('stiff diffusion remains positive and conservative without a tiny-cell timestep', () => {
    const net = chain([0, 0.1], { cellLengthMm: 0.05, axialDispersionMm2PerS: 100 });
    net.setFlowOverride([0], 0);
    net.depositIodine(0, 0, 1);
    net.update(1 / 30);
    assert.ok(Math.abs(net.edges[0].massMg[0] - 0.5) < 1e-9);
    assert.ok(Math.abs(net.edges[0].massMg[1] - 0.5) < 1e-9);
    assert.ok(Math.abs(sum(net) - 1) < 1e-12);
    assert.equal(net.lastTransportSubstepCount, 1);
});

test('connected samples are continuous across unequal one-cell segment boundaries', () => {
    const net = chain([0, 1, 5, 6], { cellLengthMm: 10 });
    const field = new FlowConcentrationField(net, new Uint32Array([0, 1, 2]), 3);
    field.update(new Float32Array([1.5, 4, 6.5])); // c(x)=1+x at cell centres
    for (const [a, b, x] of [
        [0, 1, 1],
        [1, 2, 5]
    ]) {
        assert.ok(Math.abs(field.sample(a, 1) - (1 + x)) < 1e-6);
        assert.ok(Math.abs(field.sample(a, 1) - field.sample(b, 0)) < 1e-7);
    }
    field.update(new Float32Array([1, 0, 0]));
    assert.equal(
        field.sample(0, 1),
        field.sample(1, 0),
        'a clear downstream cell must not cut off the bolus front'
    );
    assert.ok(field.sample(1, 0) > 0);
    assert.equal(field.sample(1, 0.5), 0, 'the interpolation must stop at the clear cell centre');
});

function display(net) {
    const r = new ContrastVolumeRenderer({
        flowNetwork: net,
        localSolver: new LocalContrastInjectionSolver({ flowNetwork: net, capacity: 16 }),
        medium: { iodineMgPerMl: 300 }
    });
    return r;
}
test('junction connectors follow the local arm instead of retaining the strongest daughter', () => {
    const net = new ContrastFlowNetwork(
        [
            segment(0, 0, 1, [0, 0, 0], [20, 0, 0]),
            segment(1, 1, 2, [20, 0, 0], [40, 15, 0]),
            segment(2, 1, 3, [20, 0, 0], [40, -15, 0])
        ],
        { rootPoint: new THREE.Vector3(0, 0, 0), cellLengthMm: 2 }
    );
    for (let i = 0; i < 3; i++) net.depositIodine(1, i, net.edges[1].volumes[i] * 0.1);
    const r = display(net);
    try {
        const vertices = [...r._flowTrueJunctionConnectorVertexIndices];
        assert.ok(vertices.length > 0);
        assert.ok(
            vertices.some((i) => r._flowVertexConcentration[i] > 0),
            'perfused arm stays visible'
        );
        for (const i of vertices)
            if (r._flowVertexConcentrationEdgeIndex[i] !== 1)
                assert.equal(
                    r._flowVertexConcentration[i],
                    0,
                    'clear arms must not acquire the maximum neighbour signal'
                );
    } finally {
        r.dispose();
    }
});

test('display does not fill long clear intervals between separate boluses', () => {
    const net = chain(Array.from({ length: 21 }, (_, i) => i * 2));
    for (const i of [1, 2, 17, 18]) net.depositIodine(i, 0, net.edges[i].volumes[0] * 0.1);
    const r = display(net);
    try {
        for (let i = 6; i < 14; i++)
            assert.equal(r._flowCellDisplayConcentration[r._flowCellOffset[i]], 0);
    } finally {
        r.dispose();
    }
});
