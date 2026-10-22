import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';

function fixture({exits = false, renal = false, scale = 1} = {}) {
    const point = (x, y) => new THREE.Vector3(x * scale, y * scale, 0);
    const segments = [
        [0, 1, point(0, 10), point(0, 0), 2.5, 2.5],
        [1, 2, point(0, 0), point(0, -20), 2.5, 2.5],
        [1, 3, point(0, 0), point(.2, -2), 2.5, 1],
        [3, 4, point(.2, -2), point(exits ? 4 : .4, -4), 1, .4]
    ].map(([nodeStartId,nodeEndId,start,end,radiusStart,radiusEnd],id) => ({
        id,nodeStartId,nodeEndId,start,end,radiusStart:radiusStart*scale,radiusEnd:radiusEnd*scale,
        ...(renal && id > 1 ? {renalSide:'left'} : {})
    }));
    return new ContrastFlowNetwork(segments);
}

test('a tapered terminal duplicate inside a continuing artery is not a second reservoir', () => {
    const net = fixture();
    assert.equal(net.containedTerminalSpurDiagnostics.roots.length, 1);
    const p = net.containedTerminalSpurDiagnostics.roots[0];
    assert.equal(p.edgeIndices.length, 2);
    assert.ok(p.edgeIndices.every(i => net.edges[i].transportExcluded));
    assert.equal(net.edges[p.mainContinuationEdgeIndex].meanFlowMm3PerS,
        net.hemodynamics.cardiacOutputMlPerMin * 1000 / 60);
});

test('a short branch that leaves the continuing lumen retains its flow and outlet', () => {
    const net = fixture({exits:true});
    assert.equal(net.containedTerminalSpurDiagnostics.roots.length, 0);
    assert.ok(net.edges.every(e => !e.transportExcluded && e.meanFlowMm3PerS > 0));
});

test('explicit renal anatomy remains catheterizable even close to a parent', () => {
    const net = fixture({renal:true});
    assert.equal(net.containedTerminalSpurDiagnostics.roots.length, 0);
    assert.ok(net.edges.every(e => !e.transportExcluded));
});

for (const scale of [.2, 10]) {
    test(`contained duplicates are removed across artery sizes (scale ${scale})`, () => {
        const net = fixture({scale});
        assert.equal(net.containedTerminalSpurDiagnostics.roots.length, 1);
        assert.equal(net.edges.filter(e => !e.transportExcluded).length, 2);
    });
}
