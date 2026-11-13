import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {LocalContrastInjectionSolver} from '../src/contrast/localInjectionSolver.js';

function fixture() {
    const make = (index, start, end, radius, parentEdgeIndex, childEdgeIndices) => {
        start = new THREE.Vector3(...start); end = new THREE.Vector3(...end);
        const length = start.distanceTo(end), cellCount = Math.ceil(length / 2);
        return {index, start, end, axis: end.clone().sub(start).normalize(), length,
            radiusStart: radius, radiusEnd: radius, cellCount, cellLength: length / cellCount,
            parentEdgeIndex, childEdgeIndices};
    };
    const edges = [make(0, [-20, 0, 0], [0, 0, 0], 3, -1, [1, 2]),
        make(1, [0, 0, 0], [20, 10, 0], 2, 0, []),
        make(2, [0, 0, 0], [20, -10, 0], 2, 0, []),
        // This physically closer artery has no connection to the source.
        make(3, [-12, .1, 0], [-4, .1, 0], 2, -1, [])];
    const deposited = new Map();
    const network = {edges, time: 0, hemodynamics: {heartRateBpm: 72},
        getFaceSignedFlowMm3PerS: () => 1, depositGraftSacAtCoordinates: () => false,
        queueIodineSource(edge, cell, mass) {
            const key = `${edge}:${cell}`; deposited.set(key, (deposited.get(key) ?? 0) + mass); return true;
        }};
    const solver = new LocalContrastInjectionSolver({flowNetwork: network, capacity: 1});
    solver.count = 1; solver.sourceEdgeIndex[0] = 1; solver.sourceCellIndex[0] = 0;
    return {edges, solver, deposited, locate(point, source = 1) {
        solver.sourceEdgeIndex[0] = source; solver.handoffEdgeIndex[0] = -1;
        [solver.positionX[0], solver.positionY[0], solver.positionZ[0]] = point;
        return solver.resolveSourceHandoffLocation(0, {});
    }};
}

test('a jet crosses a connected upstream fork without snapping to a nearby unrelated vessel', () => {
    const f = fixture(), location = f.locate([-6, .1, 0]);
    assert.equal(location.edgeIndex, 0);
    assert.ok(location.t > .6 && location.t < .8);
    assert.notEqual(location.edgeIndex, 3);
    assert.deepEqual(f.solver.resolveSourceHandoffLocation(0, {}), location,
        'Rendering the same parcel again must not alter its transport mapping');
});

test('a downstream jet enters only a containing daughter after leaving the common parent volume', () => {
    const f = fixture();
    assert.equal(f.locate([1, .5, 0], 0).edgeIndex, 0, 'The common volume cannot be captured by a smaller ostial axis');
    assert.equal(f.locate([8, 4, 0], 0).edgeIndex, 1);
    assert.equal(f.locate([8, -4, 0], 0).edgeIndex, 2);
    assert.equal(f.locate([8, 8, 0], 0).edgeIndex, 0, 'No daughter contains this point');
});

test('excluded and fabric-replaced parent paths cannot be crossed by local handoff', () => {
    for (const flag of ['transportExcluded', 'lumenMappingExcluded']) {
        const f = fixture(); f.edges[0][flag] = true;
        const location = f.locate([-1, 0, 0]);
        assert.equal(location.edgeIndex, 1);
        assert.equal(location.lowerEdgeIndex, 1);
        assert.equal(location.upperEdgeIndex, 1);
    }
});

test('iodine deposition is conservative and continuous across the true fork boundary', () => {
    const f = fixture(), distributions = [];
    for (const signedDistance of [-1e-5, 0, 1e-5]) {
        f.locate([signedDistance, signedDistance * .5, 0]); f.deposited.clear();
        assert.equal(f.solver._depositSourceEntrainment(0, 7), 7);
        assert.ok(Math.abs([...f.deposited.values()].reduce((sum, value) => sum + value, 0) - 7) < 1e-12);
        distributions.push(new Map(f.deposited));
        assert.ok([...f.deposited.keys()].every(key => !key.startsWith('2:') && !key.startsWith('3:')));
    }
    const keys = new Set(distributions.flatMap(value => [...value.keys()]));
    const difference = [...keys].reduce((sum, key) => sum + Math.abs(
        (distributions[0].get(key) ?? 0) - (distributions[2].get(key) ?? 0)), 0);
    assert.ok(difference < .001, 'A tiny position change cannot relocate a whole parcel at the atlas seam');
});
