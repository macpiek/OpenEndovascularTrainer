import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { DilatedLumenMixing } from '../src/contrast/dilatedLumenMixing.js';
import { FlowContrastVolume } from '../src/contrast/flowContrastVolume.js';

// Identical physical cells, either one long edge or a fragmented centerline.
function dilation(fragmented = false, dispersion = 10) {
    const edges = [];
    function edge(radius, length, count) {
        const index = edges.length,
            x = edges.at(-1)?.end.x || 0;
        const area = Math.PI * radius * radius;
        const result = {
            index,
            parentEdgeIndex: index - 1,
            childEdgeIndices: [],
            radiusStart: radius,
            radiusEnd: radius,
            length,
            start: new THREE.Vector3(x, 0, 0),
            end: new THREE.Vector3(x + length, 0, 0),
            axis: new THREE.Vector3(1, 0, 0),
            cellCount: count,
            cellLength: length / count,
            areas: new Float64Array(count).fill(area),
            volumes: new Float64Array(count).fill((area * length) / count),
            massMg: new Float64Array(count),
            nextMassMg: new Float64Array(count),
            meanFlowMm3PerS: area * 20
        };
        if (index) edges[index - 1].childEdgeIndices.push(index);
        edges.push(result);
    }
    edge(5, 1, 1);
    if (fragmented) for (let i = 0; i < 7; i++) edge(12, 0.1, 1);
    else edge(12, 0.7, 7);
    const net = { edges, hemodynamics: { axialDispersionMm2PerS: dispersion } };
    net.dilatedLumenMixing = new DilatedLumenMixing(net);
    return net;
}
function mix(net, dt) {
    const touched = [],
        marks = new Uint8Array(net.edges.length);
    net.dilatedLumenMixing.update(dt, touched, marks);
    return net.dilatedLumenMixing.regions.flatMap(({ cells }) =>
        cells.map(({ edge, cell }) =>
            marks[edge.index] ? edge.nextMassMg[cell] : edge.massMg[cell]
        )
    );
}

test('stiff aneurysm mixing is positive, conservative and independent of centerline fragmentation', () => {
    const a = dilation(),
        b = dilation(true);
    a.edges[1].massMg[3] = 600;
    b.edges[4].massMg[0] = 600;
    const ma = mix(a, 1),
        mb = mix(b, 1);
    assert.ok(
        ma.every((m) => m > 0),
        'exchange must spread to both sides of the bolus'
    );
    assert.ok(Math.abs(ma.reduce((a, b) => a + b, 0) - 600) < 1e-7);
    ma.forEach((m, i) => assert.ok(Math.abs(m - mb[i]) < 1e-7));
    assert.equal(a.dilatedLumenMixing.weights[0], 0, 'normal inlet stays outside the dilation');
});

test('aneurysm mixing cannot diffuse across deployed graft fabric or an excluded cell', () => {
    for (const excluded of [false, true]) {
        const net = dilation(true);
        net.edges[2].massMg[0] = 1;
        if (excluded) net.edges[3].transportExcluded = true;
        else net.edges[3].graftSections = [1];
        const mass = mix(net, 1);
        assert.ok(mass[0] > 0);
        assert.ok(mass.slice(2).every((m) => m === 0));
        assert.ok(Math.abs(mass.reduce((a, b) => a + b, 0) - 1) < 1e-10);
    }
});

test('zero dispersion leaves the finite advection profile unchanged', () => {
    const net = dilation(false, 0);
    net.edges[1].massMg[3] = 1;
    assert.deepEqual(mix(net, 1), [0, 0, 0, 1, 0, 0, 0]);
});

test('subcell volume samples preserve every cell mass without widening the optical kernel', () => {
    const net = dilation();
    const edge = net.edges[1];
    edge.cellLength = 8;
    edge.length = edge.cellCount * 8;
    edge.end.copy(edge.start).addScaledVector(edge.axis, edge.length);
    edge.massMg.fill(3);
    const volume = new FlowContrastVolume(net, new Uint32Array([0, 1]));
    try {
        volume.update(new Float32Array(8), 0.3, false);
        for (let cell = 0; cell < edge.cellCount; cell++) {
            const indices = volume.cells.flatMap((entry, i) =>
                entry.edge === edge && entry.cell === cell ? [i] : []
            );
            assert.equal(indices.length, 4);
            assert.ok(Math.abs(indices.reduce((sum, i) => sum + volume.current[i], 0) - 5) < 1e-6);
            for (const i of indices)
                assert.ok(Math.abs(volume.mesh.geometry.attributes.sigma.getY(i) - 4.8) < 1e-6);
        }
    } finally {
        volume.dispose();
    }
});

test('dilated volume follows iodine mass and only yields to an actual replacement graft image', () => {
    const net = dilation(),
        volume = new FlowContrastVolume(net, new Uint32Array([0, 1]));
    const plume = new Float32Array(8);
    const sampleSum = (values) =>
        volume.cells.reduce(
            (sum, entry, i) => sum + (entry.edge.index === 1 && entry.cell === 3 ? values[i] : 0),
            0
        );
    try {
        net.edges[1].massMg[3] = 2;
        plume[4] = 1;
        volume.update(plume, 0.3, true);
        assert.equal(
            sampleSum(volume.current),
            5,
            'stock-equivalent volume uses both flow and local plume'
        );
        assert.ok(volume.mesh.visible);
        net.edges[1].graftSections = new Uint8Array(7).fill(1);
        volume.update(plume, 0.3, true, false);
        assert.equal(
            sampleSum(volume.current),
            5,
            'partial graft without a replacement image must not erase contrast'
        );
        volume.update(plume, 0.3, true, true);
        assert.equal(sampleSum(volume.current), 0);
        assert.equal(
            sampleSum(volume.previous),
            5,
            'retain one presentation frame for smooth fading'
        );
        volume.update(plume, 0.3, true, true);
        assert.equal(volume.mesh.visible, false);
    } finally {
        volume.dispose();
    }
});

test('native volume keeps iodine visible below the former 4–6 mm surface transition', () => {
    const net = dilation(true);
    const radii = [6.8, 6.2, 5.8, 5, 4.5, 4, 2, 0.8];
    net.edges.forEach((edge, i) => {
        edge.radiusStart = edge.radiusEnd = radii[i];
        edge.massMg[0] = Math.PI * radii[i] ** 2 * edge.cellLength * 0.03;
    });
    const volume = new FlowContrastVolume(
        net,
        Uint32Array.from(radii, (_, i) => i)
    );
    try {
        volume.update(new Float32Array(radii.length), 0.3, false);
        net.edges.forEach((edge) => {
            const opticalMass = volume.cells.reduce(
                (sum, sample, i) => sum + (sample.edge === edge ? volume.current[i] : 0),
                0
            );
            const expected = (0.5 * edge.massMg[0]) / 0.3;
            assert.ok(
                Math.abs(opticalMass / expected - 1) < 1e-6,
                `radius ${edge.radiusStart}: the complete lumen mass must remain visible`
            );
        });
    } finally {
        volume.dispose();
    }
});
