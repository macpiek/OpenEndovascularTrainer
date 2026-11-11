import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {ContrastFlowNetwork, arterialWaveform} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';

// Frozen previous algorithm: geometric mixture followed by an inflow override.
// It checks the optimized traversal against exactly the prior display field.
function reference(field, source) {
    const result = new Float32Array(field.values.length); result.set(source);
    const wave = field.network.hemodynamics ? arterialWaveform(field.network.time, field.network.hemodynamics.heartRateBpm) : 1;
    field.nodes.forEach((node, n) => {
        let value = 0, weight = 0;
        const geometric = (index, atEnd) => {
            const edge = field.edges[index], cell = atEnd ? edge.cellCount - 1 : 0;
            if (edge.transportExcluded) return;
            const w = edge.areas[cell] / edge.cellLength;
            value += source[field.offsets[index] + cell] * w; weight += w;
        };
        if (node.parentEdgeIndex >= 0) geometric(node.parentEdgeIndex, true);
        for (const child of node.childEdgeIndices) geometric(child, false);
        if (node.parentEdgeIndex >= 0 && node.childEdgeIndices.length > 1 && field.network.getFaceSignedFlowMm3PerS) {
            let inflow = 0, flux = 0;
            const incoming = (index, atEnd) => {
                const edge = field.edges[index], cell = atEnd ? edge.cellCount - 1 : 0;
                if (edge.transportExcluded) return;
                const q = (atEnd ? 1 : -1) * field.network.getFaceSignedFlowMm3PerS(index, atEnd ? edge.cellCount : 0, wave);
                if (q > 0) {inflow += q; flux += q * source[field.offsets[index] + cell];}
            };
            incoming(node.parentEdgeIndex, true);
            for (const child of node.childEdgeIndices) incoming(child, false);
            if (inflow > 0) {weight = inflow; value = flux;}
        }
        result[field.cellCount + n] = weight > 0 ? value / weight : 0;
    });
    return result;
}
function fixture() {
    const locations = [[0, 20, 0], [0, 0, 0], [-12, -10, 0], [12, -10, 0], [16, -20, 0]];
    const network = new ContrastFlowNetwork([[0, 1], [1, 2], [1, 3], [3, 4]].map(([a, b], id) => ({
        id, nodeStartId: a, nodeEndId: b, start: new Vector3(...locations[a]), end: new Vector3(...locations[b]),
        radiusStart: id ? 1.3 : 3, radiusEnd: id ? 1.1 : 3
    })), {rootPoint: new Vector3(...locations[0]), cellLengthMm: 1});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const result = count; count += edge.cellCount; return result;});
    return {network, field: new FlowConcentrationField(network, offsets, count), cells: new Float32Array(count)};
}

test('fork-first update exactly preserves forward flow, reflux, blocked and clear display fields', () => {
    const {network, field, cells} = fixture();
    for (const flows of [[400, 100, 300, 300], [-200, -300, 100, 100], [100, -200, -100, -100],
        [-100, 50, 50, 50], [0, 0, 0, 0]]) {
        for (const excluded of [-1, 0, 1, 2, 3]) for (let frame = 0; frame < 8; frame++) {
            network.time = frame / 60;
            network.edges.forEach((edge, i) => {
                network.setFlowOverride([edge.index], flows[i]); edge.transportExcluded = i === excluded;
                // Geometry can change under a device; do not cache these weights.
                edge.areas[0] = 1.1 + frame * .7 + i;
            });
            for (let i = 0; i < cells.length; i++) cells[i] = frame === 7 ? 0 : Math.max(0, Math.sin(i * .41 + frame * .7));
            const expected = reference(field, cells), mass = network.edges.map(edge => edge.massMg.slice());
            field.update(cells);
            assert.deepEqual(field.values, expected, `flows=${flows}, excluded=${excluded}, frame=${frame}`);
            network.edges.forEach((edge, i) => assert.deepEqual(edge.massMg, mass[i]));
        }
    }
});

test('geometric fallback remains exact without a signed-flow provider', () => {
    const {network, field, cells} = fixture(); network.getFaceSignedFlowMm3PerS = undefined;
    for (let frame = 0; frame < 12; frame++) {
        for (let i = 0; i < cells.length; i++) cells[i] = (i + frame) % 13 / 13;
        const expected = reference(field, cells); field.update(cells); assert.deepEqual(field.values, expected);
    }
});
