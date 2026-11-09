import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {createFlowColumnIntegral} from '../src/contrast/flowColumnIntegral.js';
import {createFlowColumnGeometry} from '../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';

function fixture(points, links, territoryIds, continuationIds) {
    const nodes = new Map();
    const node = id => {
        if (!nodes.has(id)) nodes.set(id, {id, point: new THREE.Vector3(...points[id]),
            parentEdgeIndex: -1, childEdgeIndices: []});
        return nodes.get(id);
    };
    const edges = links.map(([from, to, radius], index) => {
        const start = node(from).point, end = node(to).point;
        node(from).childEdgeIndices.push(index); node(to).parentEdgeIndex = index;
        const length = start.distanceTo(end), cellCount = 32;
        return {index, startNodeId: from, endNodeId: to, start, end, length,
            axis: end.clone().sub(start).divideScalar(length), radiusStart: radius, radiusEnd: radius,
            cellCount, cellLength: length / cellCount,
            areas: new Float64Array(cellCount).fill(Math.PI * radius * radius)};
    });
    for (const edge of edges) {
        edge.parentEdgeIndex = node(edge.startNodeId).parentEdgeIndex;
        edge.childEdgeIndices = node(edge.endNodeId).childEdgeIndices;
    }
    const network = {edges, nodes}, offsets = Uint32Array.from(edges, edge => edge.index * 32);
    const field = new FlowConcentrationField(network, offsets, edges.length * 32);
    const territories = Int32Array.from(territoryIds), continuations = Int32Array.from(continuationIds);
    const integral = createFlowColumnIntegral(network, field, territories, continuations);
    const spatial = createFlowColumnGeometry(network, integral, territories, continuations);
    const projection = createFlowJunctionProjection(network, integral, territories, spatial);
    return {projection, integral, spatial,
        fill(value) {
            const values = new Float32Array(field.cellCount);
            for (const edge of edges) values.fill(value(edge.index), offsets[edge.index], offsets[edge.index] + edge.cellCount);
            field.update(values); integral.update(field.values);
        },
        dispose() {projection.dispose(); spatial.dispose(); integral.dispose();}
    };
}

test('a clear disconnected lumen cannot borrow iodine from an overlapping junction search region', () => {
    const f = fixture([[0, 12, 0], [0, 0, 0], [0, -12, 0], [12, 0, 0], [4, 12, 4], [4, -12, 4]],
        [[0, 1, 3], [1, 2, 3], [1, 3, .5], [4, 5, .5]], [0, 0, 2, 3], [1, -1, -1, -1]);
    try {
        const entry = new THREE.Vector3(4, 0, 4.5), exit = new THREE.Vector3(4, 0, 3.5);
        const point = entry.clone().lerp(exit, .5), junction = f.projection.junctions[0];
        assert.ok([0, 1, 2].every(axis => point.getComponent(axis) >= junction.min[axis] &&
            point.getComponent(axis) <= junction.max[axis]), 'The independent vessel must lie inside broad-phase junction support');
        assert.ok(point.distanceTo(new THREE.Vector3(0, 0, 0)) > 3 && Math.abs(point.z) > .5,
            'It remains physically outside both T-junction tubes');
        f.fill(edge => edge === 3 ? 0 : 1);
        assert.equal(f.projection.hasSupport(entry, exit, 3), false,
            'A spatially overlapping junction in a different graph component is not this lumen’s junction');
        assert.equal(f.projection.pointValue(point, 3, 12, 3, 12), 0);
        assert.equal(f.projection.average(3, 12, 3, 12, entry, exit), 0);
        f.fill(edge => edge === 3 ? .4 : 0);
        assert.ok(Math.abs(f.projection.average(3, 12, 3, 12, entry, exit) - .4) < 1e-6,
            'The separate lumen retains its own signal when the surrounding component is clear');
    } finally {f.dispose();}
});

test('local junction integration retains a filled daughter beyond fixed 16- or 32-arm limits', () => {
    // A high-degree custom junction stresses candidate storage independently
    // of the production anatomy. Only the final, out-of-plane arm is filled.
    const points = [[0, 12, 0], [0, 0, 0], [0, -12, 0]], links = [[0, 1, 3], [1, 2, 3]];
    for (let i = 0; i < 39; i++) {
        const angle = Math.PI * 2 * (i + .37) / 39;
        points.push([12 * Math.cos(angle), 12 * Math.sin(angle), 0]);
        links.push([1, points.length - 1, .5]);
    }
    points.push([0, 0, 12]); links.push([1, points.length - 1, .5]);
    const last = links.length - 1, territories = links.map((_, i) => i === 1 ? 0 : i);
    const continuations = links.map((_, i) => i === 0 ? 1 : -1);
    const f = fixture(points, links, territories, continuations);
    try {
        f.fill(edge => edge === last ? 1 : 0);
        const entry = new THREE.Vector3(0, 0, 4), exit = new THREE.Vector3(0, 0, 6);
        assert.ok(f.projection.junctions[0].seeds.size > 32);
        assert.equal(f.projection.hasSupport(entry, exit, 0), true);
        assert.ok(f.projection.pointValue(entry.clone().lerp(exit, .5), 0, 12, 0, 12) > .99,
            'Late incident arms must participate even when both wall labels name the empty parent');
        const forward = f.projection.average(0, 12, 0, 12, entry, exit);
        const reverse = f.projection.average(0, 12, 0, 12, exit, entry);
        assert.ok(forward > .99, 'No fixed-capacity candidate list may silently drop the filled daughter');
        assert.ok(Math.abs(forward - reverse) < 1e-10);
    } finally {f.dispose();}
});

test('overlapping small junctions do not truncate the ray’s local perfusion candidates', () => {
    const junctionCount = 40;
    const points = Array.from({length: junctionCount + 2}, (_, i) => [0, 1 - 2 * i / (junctionCount + 1), 0]);
    const links = Array.from({length: junctionCount + 1}, (_, i) => [i, i + 1, 3]);
    for (let node = 1; node <= junctionCount; node++) {
        points.push(node === junctionCount ? [0, points[node][1], 12] : [12, points[node][1], 0]);
        links.push([node, points.length - 1, .5]);
    }
    const last = links.length - 1;
    const territories = links.map((_, i) => i <= junctionCount ? 0 : i);
    const continuations = links.map((_, i) => i < junctionCount ? i + 1 : -1);
    const f = fixture(points, links, territories, continuations);
    try {
        const y = points[junctionCount][1], s = 1 - y;
        const entry = new THREE.Vector3(0, y, 4), exit = new THREE.Vector3(0, y, 6);
        const center = entry.clone().lerp(exit, .5);
        const overlapping = f.projection.junctions.filter(junction => [0, 1, 2].every(axis =>
            center.getComponent(axis) >= junction.min[axis] && center.getComponent(axis) <= junction.max[axis]));
        assert.equal(overlapping.length, junctionCount, 'The sample must exercise more than 32 overlapping junction bounds');
        f.fill(edge => edge === last ? 1 : 0);
        assert.ok(f.projection.average(0, s, 0, s, entry, exit) > .99,
            'A late visible daughter cannot be omitted because earlier junctions consumed a fixed ray-candidate buffer');
    } finally {f.dispose();}
});

test('local projection preserves a corner minimum when unclamped Newton steps oscillate between its segments', () => {
    // A reduced version of the anterior-aortic atlas kink: projection onto
    // either infinite supporting line jumps past the corner onto the other
    // segment. Returning the final iterate would alternate with iteration
    // parity, and therefore sample opposite sides of a contrast front.
    const f = fixture([[-1, 0, 0], [0, 0, 0], [0, -1, 0]],
        [[0, 1, 2], [1, 2, 2]], [0, 0], [1, -1]);
    try {
        const point = new THREE.Vector3(1, 1, 0), range = f.spatial.ranges.get(0), data = f.spatial.data;
        let expected = {distance: Infinity};
        // Independent exhaustive projection on every finite atlas segment.
        for (let i = 0; i < range.count - 1; i++) {
            const at = (range.start + i) * 4;
            const a = new THREE.Vector3().fromArray(data, at), b = new THREE.Vector3().fromArray(data, at + 4);
            const line = new THREE.Line3(a, b), t = line.closestPointToPointParameter(point, true);
            const distance = line.at(t, new THREE.Vector3()).distanceTo(point);
            if (distance < expected.distance) expected = {distance, s: (i + t) * range.step};
        }
        assert.ok(Math.abs(expected.s - 1) < 1e-12, 'The independent minimum lies at the shared corner');
        for (const seed of [.25, .75, 1.25, 1.75]) {
            let best = Infinity;
            for (let iterations = 1; iterations <= 6; iterations++) {
                const actual = f.spatial.nearest(0, point, seed, iterations);
                assert.ok(actual.score <= best + 1e-12,
                    `More refinement cannot discard a closer finite-segment location (seed ${seed}, step ${iterations})`);
                assert.ok(Math.abs(actual.s - expected.s) < 1e-12,
                    'The selected concentration coordinate must be independent of iteration parity');
                assert.ok(Math.abs(actual.point.distanceTo(point) - expected.distance) < 1e-12);
                best = actual.score;
            }
        }
    } finally {f.dispose();}
});
