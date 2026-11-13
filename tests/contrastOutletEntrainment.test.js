import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { ContrastFlowNetwork } from '../src/contrast/flowNetwork.js';
import { LocalContrastInjectionSolver } from '../src/contrast/localInjectionSolver.js';

const segment = (id, a, b, start, end) => ({
    id, nodeStartId: a, nodeEndId: b,
    start: new THREE.Vector3(...start), end: new THREE.Vector3(...end),
    radiusStart: 5, radiusEnd: 5
});
function chain(points = [-60, -20, 20, 60]) {
    const network = new ContrastFlowNetwork(points.slice(1).map((x, i) =>
        segment(i, i, i + 1, [points[i], 0, 0], [x, 0, 0])),
    {rootPoint: new THREE.Vector3(points[0], 0, 0), cellLengthMm: 2});
    network.setFlowOverride(network.edges.map(e => e.index), 0);
    return network;
}
function emit(network, direction = 1, x = 0) {
    const solver = new LocalContrastInjectionSolver({flowNetwork: network, randomSeed: 43});
    solver.emit({ports: [{position: new THREE.Vector3(x, 0, 0),
        direction: new THREE.Vector3(direction, 0, 0), radiusMm: .5, valid: true}],
    volumeMl: .1, rateMlPerSec: 1, allowPressureDrivenRetrograde: false});
    return solver;
}
function massCentroid(network) {
    let mass = 0, moment = 0;
    for (const edge of network.edges) for (let i = 0; i < edge.cellCount; i++) {
        mass += edge.massMg[i];
        moment += edge.massMg[i] * (edge.start.x + edge.axis.x * (i + .5) * edge.cellLength);
    }
    return moment / mass;
}

test('a coherent outlet core gradually entrains downstream of the actual port, conserving iodine', () => {
    const network = chain(), solver = emit(network);
    let previous = 0, positiveReleaseSteps = 0, largestRelease = 0;
    for (let step = 0; step < 160; step++) {
        solver.update(.001);
        network.update(.001);
        const transferred = network.getIodineMassMg() + network.outletIodineMassMg;
        const released = transferred - previous;
        if (released > 1e-6) positiveReleaseSteps++;
        largestRelease = Math.max(largestRelease, released);
        assert.ok(Math.abs(solver.getIodineMassMg() + transferred - 30) < 1e-10);
        if (step === 0) assert.ok(solver.getIodineMassMg() > 29.9,
            'a newly emitted core is not instantly painted across the source lumen');
        if (step === 54) assert.ok(solver.getIodineMassMg() > .01,
            'the old 55 ms source-cell reset must not retire the entire jet');
        previous = transferred;
    }
    assert.equal(solver.count, 0);
    assert.ok(positiveReleaseSteps > 60, 'entrainment releases mass continuously');
    assert.ok(largestRelease < 2, `no discrete source-cell dump: ${largestRelease}`);
    assert.ok(massCentroid(network) > 6, 'mixed iodine remains along the travelled jet, not at the source cell');
    assert.ok(Math.abs(solver.sourceHandoffIodineMassMg - 30) < 1e-10);
});

test('opposing nozzle momentum mixes locally upstream without modifying the hydraulic reflux solution', () => {
    const network = chain(), solver = emit(network, -1);
    const before = network.edges.map(e => Array.from(e.faceFlowDeltaMm3PerS));
    for (let i = 0; i < 160; i++) { solver.update(.001); network.update(.001); }
    assert.ok(massCentroid(network) < -6);
    assert.deepEqual(network.edges.map(e => Array.from(e.faceFlowDeltaMm3PerS)), before);
    assert.ok(Math.abs(network.getIodineMassMg() + network.outletIodineMassMg - 30) < 1e-10);
});

test('source-connected mapping follows a reached parent without jumping laterally into a neighbouring branch', () => {
    const network = new ContrastFlowNetwork([
        segment(0, 0, 1, [-10, 0, 0], [0, 0, 0]),
        segment(1, 1, 2, [0, 0, 0], [20, 0, 0]),
        segment(2, 2, 3, [20, 0, 0], [40, 0, 0]),
        segment(3, 1, 4, [0, 0, 0], [40, 1, 0])
    ], {rootPoint: new THREE.Vector3(-10, 0, 0), cellLengthMm: 2});
    const solver = emit(network, 1, 10);
    solver.positionX[0] = 30; solver.positionY[0] = .75; solver.positionZ[0] = 0;
    assert.equal(network.findNearestLocationCoordinates(30, .75, 0).edgeIndex, 2);
    const location = solver.resolveSourceHandoffLocation(0);
    assert.equal(location.edgeIndex, 3);
    assert.equal(location.lowerEdgeIndex, 3);
    solver._depositSourceEntrainment(0, .75);
    network.update(1e-6);
    assert.ok(network.edges[3].massMg.some(m => m > 0));
    assert.equal(network.edges[2].massMg.reduce((a, b) => a + b, 0), 0);
    solver.positionX[0] = -5;
    const upstream = solver.resolveSourceHandoffLocation(0);
    assert.equal(upstream.edgeIndex, 0, 'The parcel has physically crossed the fork into its containing parent');
    assert.equal(upstream.lowerEdgeIndex, 0);
    assert.equal(upstream.upperEdgeIndex, 0);
    assert.ok(Math.abs(upstream.t - .5) < 1e-7);
    solver._depositSourceEntrainment(0, .25);
    network.update(1e-6);
    assert.ok(network.edges[0].massMg.reduce((a, b) => a + b, 0) > .24,
        'Upstream entrainment remains at the reached position instead of returning to the source cell');
    assert.equal(network.edges[2].massMg.reduce((a, b) => a + b, 0), 0,
        'The closer parallel sibling never acquires source iodine through spatial proximity');
    assert.ok(Math.abs(network.getIodineMassMg() + network.outletIodineMassMg - 1) < 1e-12);
});

test('a divergent hydraulic source divides entrained iodine by its outgoing face flows', () => {
    const network = chain([-60, 60]);
    network.hemodynamics.axialDispersionMm2PerS = 0;
    const solver = emit(network, -1), source = solver.sourceCellIndex[0];
    const edge = network.edges[0];
    network.addFaceFlowDelta(0, 0, source + 1, -100);
    network.addFaceFlowDelta(0, source + 1, edge.cellCount + 1, 300);
    // A directional upstream core must not force all iodine upstream when
    // the volume source supplies one part reflux to three parts distal flow.
    for (let i = 0; i < 160; i++) solver.update(.001);
    assert.ok(Math.abs(network.getIodineMassMg() - 30) < 1e-10);
    assert.ok(edge.massMg.every(mass => mass === 0), 'entrainment is pending as a rate, not an initial bolus');
    network.update(.6);
    const upstream = edge.massMg.slice(0, source).reduce((a, b) => a + b, 0);
    const downstream = edge.massMg.slice(source + 1).reduce((a, b) => a + b, 0);
    assert.ok(upstream > 0 && downstream > 0);
    assert.ok(Math.abs(upstream / (upstream + downstream) - .25) < 1e-8);
    assert.ok(Math.abs(network.getIodineMassMg() - 30) < 1e-10);
});

test('handoff interpolation crosses an ordinary cell seam continuously', () => {
    const network = chain([-20, 0, 20]), solver = emit(network, 1, -2);
    let previousWeight = -1;
    for (const x of [-.9, -.5, 0, .5, .9]) {
        solver.positionX[0] = x; solver.positionY[0] = 0; solver.positionZ[0] = 0;
        const location = solver.resolveSourceHandoffLocation(0);
        assert.equal(location.lowerEdgeIndex, 0);
        assert.equal(location.upperEdgeIndex, 1);
        assert.ok(location.upperWeight > previousWeight);
        assert.ok(Math.abs(location.upperWeight - (x + 1) / 2) < 1e-7);
        previousWeight = location.upperWeight;
    }
});

test('swap removal preserves outlet geometry for kernels belonging to the surviving parcel', () => {
    const solver = emit(chain());
    const last = solver.count - 1;
    solver.outletPositionX[last] = 9; solver.outletPositionY[last] = 3;
    solver.outletPositionZ[last] = 2; solver.outletDirectionX[last] = -.5;
    solver.outletDirectionY[last] = .7; solver.outletDirectionZ[last] = .2;
    solver.outletRadiusMm[last] = .6; solver.handoffEdgeIndex[last] = 2;
    solver._removeParticle(0);
    for (const name of ['outletPositionX', 'outletPositionY', 'outletPositionZ',
        'outletDirectionX', 'outletDirectionY', 'outletDirectionZ', 'outletRadiusMm', 'handoffEdgeIndex'])
        assert.equal(solver[name][0], solver[name][last]);
});

test('capacity fallback and sub-parcel final doses conserve the exact delivered iodine', () => {
    for (const [volumeMl, capacity] of [[.1, 1], [1e-7, 16]]) {
        const network = chain();
        const solver = new LocalContrastInjectionSolver({flowNetwork: network, capacity});
        const emitted = solver.emit({ports: [{position: new THREE.Vector3(),
            direction: new THREE.Vector3(1, 0, 0), radiusMm: .5, valid: true}],
        volumeMl, rateMlPerSec: 100, allowPressureDrivenRetrograde: false});
        assert.ok(Math.abs(emitted.emittedIodineMassMg - volumeMl * 300) < 1e-10);
        for (let i = 0; i < 30; i++) {
            solver.update(1 / 120);
            network.update(1 / 120);
            assert.ok(Math.abs(network.getIodineMassMg() + network.outletIodineMassMg + solver.getIodineMassMg() - volumeMl * 300) < 1e-10);
        }
        assert.equal(solver.count, 0);
        assert.ok(Math.abs(network.getIodineMassMg() + network.outletIodineMassMg - volumeMl * 300) < 1e-10);
    }
});
