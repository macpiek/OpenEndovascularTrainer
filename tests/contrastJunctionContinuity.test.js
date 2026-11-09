import test, {before, after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import * as THREE from 'three';
import {partialOstiumFixture} from './helpers/partialOstiumFixture.js';
import {junctionContinuityFixture, smoothConcentrationRamp, anteriorSupportBoundaries} from './helpers/contrastJunctionContinuityFixture.js';
import {createFlowColumnGeometry} from '../src/contrast/flowColumnGeometry.js';
import {createFlowJunctionProjection} from '../src/contrast/flowJunctionProjection.js';

let real, geometry, projection;
before(() => {
    const data = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    real = partialOstiumFixture(data);
    geometry = createFlowColumnGeometry(real.network, real.integral, real.built.territories, real.built.continuations);
    projection = createFlowJunctionProjection(real.network, real.integral, real.built.territories, geometry);
});
after(() => {projection?.dispose(); geometry?.dispose(); real?.dispose();});

for (const oblique of [false, true]) test(`${oblique ? 'Y' : 'T'} junction concentration has no discontinuity where geometric ownership changes`, () => {
    const f = junctionContinuityFixture(oblique);
    try {
        const axis = f.network.edges[2].axis;
        // Parent clearance is Rparent - perpendicular distance. It equals
        // daughter clearance at x=Rparent-Rdaughter=3mm. This is geometry,
        // not a discontinuity in the prescribed concentration field.
        const crossing = 3 / Math.abs(axis.x), delta = .001;
        const left = axis.clone().multiplyScalar(crossing - delta), right = axis.clone().multiplyScalar(crossing + delta);
        const cLeft = f.projection.pointValue(left, 0, 30, 2, crossing - delta);
        const cRight = f.projection.pointValue(right, 0, 30, 2, crossing + delta);
        const inputDifference = Math.abs(f.integral.sample(2, crossing + delta) - f.integral.sample(2, crossing - delta));
        assert.ok(inputDifference < .001, 'The existing transport field itself is smooth across these points');
        assert.ok(Math.abs(cRight - cLeft) < .005,
            `A 0.002mm displacement must not create a concentration step: ${cLeft} -> ${cRight}`);
        assert.ok(cLeft >= .3 && cLeft <= 1 && cRight >= .3 && cRight <= 1,
            'Blending must not create a concentration beyond the local input range');
    } finally {f.dispose();}
});

test('real left renal ostium preserves smooth dilution across the parent-to-daughter spatial seam', () => {
    const daughter = real.network.edges.find(edge => edge.sourceIndex === 1628).index;
    const parent = real.built.territories[real.network.edges[daughter].parentEdgeIndex];
    const parentS = real.integral.edgeStart[real.network.edges[daughter].parentEdgeIndex] +
        real.network.edges[real.network.edges[daughter].parentEdgeIndex].length;
    real.fill((point, edge, path) => path === daughter ? 1 - .7 * smoothConcentrationRamp(
        (real.integral.edgeStart[edge.index] + point.clone().sub(edge.start).dot(edge.axis)) / 6) : 1);
    const startS = 4.066, endS = 4.068;
    const left = geometry.sample(daughter, startS).point.clone(), right = geometry.sample(daughter, endS).point.clone();
    const cLeft = projection.pointValue(left, parent, parentS, daughter, startS);
    const cRight = projection.pointValue(right, parent, parentS, daughter, endS);
    assert.ok(left.distanceTo(right) < .0021);
    assert.ok(Math.abs(real.integral.sample(daughter, endS) - real.integral.sample(daughter, startS)) < .001,
        'The graph concentration has no bolus front or node jump at this location');
    assert.ok(Math.abs(cRight - cLeft) < .005,
        `Real renal ostium must not introduce the previous 51% display-only jump: ${cLeft} -> ${cRight}`);
});

test('both historical and current outer support planes remain continuous in the original anatomical lumen', () => {
    for (const boundary of anteriorSupportBoundaries(real, projection)) {
        // Recenter the smooth nonzero input at each plane. A moved boundary
        // must not accidentally pass because the old test profile is clear.
        real.fill(point => smoothConcentrationRamp((point.y - boundary.y + 4) / 8));
        const means = [], analytical = [];
        for (const displacement of [-1e-5, 1e-5]) {
            const ray = real.ray(0, 0, boundary.y + displacement), sample = real.sample(ray);
            assert.equal(sample.intervals.length, 1, `${boundary.name}: one unchanged aortic blood interval`);
            const {entry, exit, range} = sample.intervals[0];
            assert.equal(entry.path, exit.path);
            const a = ray.at(range[0], new THREE.Vector3()), b = ray.at(range[1], new THREE.Vector3());
            assert.ok(range[1] - range[0] > 10, 'The boundary must cross the actual aortic lumen inside the STL crop');
            if (boundary.outer) assert.equal(a.y >= boundary.junction.min[1], displacement > 0,
                'This probe must cross the current outer leaf activation plane, not just the frozen historical plane');
            means.push(projection.average(entry.path, entry.s, exit.path, exit.s, a, b));
            analytical.push(real.integral.average(entry.path, entry.s, exit.s));
        }
        assert.ok(means.every(value => value > .1 && value < .9), 'The boundary profile must have nontrivial contrast');
        assert.ok(Math.abs(analytical[1] - analytical[0]) < 1e-5, 'Transport and wall coordinates remain continuous');
        assert.ok(Math.abs(means[1] - means[0]) < .0005,
            `${boundary.name}: acceleration support introduces a fixed-plane step ${means[0]} -> ${means[1]}`);
    }
});

test('an isolated junction activates continuously at its true outer support boundary', () => {
    const f = junctionContinuityFixture();
    try {
        const boundary = f.projection.junctions[0].min[1];
        f.fill((edge, s) => smoothConcentrationRamp((30 - f.integral.edgeStart[edge] - s - boundary + 4) / 8));
        const means = [], support = [], wallCoordinate = 30 - boundary + 1;
        for (const displacement of [-1e-5, 1e-5]) {
            const entry = new THREE.Vector3(0, boundary + displacement, 6), exit = new THREE.Vector3(0, boundary + displacement, -6);
            support.push(f.projection.hasSupport(entry, exit, 0));
            // Deliberately distinct wall-coordinate and spatial reconstructions
            // make a bare algorithm switch observable, as in the real crop.
            means.push(f.projection.average(0, wallCoordinate, 0, wallCoordinate, entry, exit));
        }
        assert.deepEqual(support, [false, true], 'No neighboring junction may hide the actual activation switch');
        assert.ok(means.every(value => value > .1 && value < .9));
        assert.ok(Math.abs(means[1] - means[0]) < .0005,
            `The true outer support switch must be continuous: ${means[0]} -> ${means[1]}`);
    } finally {f.dispose();}
});

test('overlapping junction reconstruction preserves uniform iodine and complete washout without an opacity floor', () => {
    const point = new THREE.Vector3(12.63, -189.44, -7.92);
    for (const concentration of [0, .2, 1]) {
        real.fill(() => concentration);
        const c = projection.pointValue(point, 162, 280.94, 2130, 4.067);
        assert.ok(Math.abs(c - concentration) < 1e-6, `Uniform ${concentration} must remain ${concentration}, got ${c}`);
        if (concentration === 0) assert.equal(c, 0, 'Cleared vessels must be exactly clear');
    }
});

test('continuous junction mixing stays within the transported concentration range and is ray-direction symmetric', () => {
    const f = junctionContinuityFixture();
    try {
        f.fill(edge => edge === f.daughter ? .2 : .8);
        for (let x = -2; x <= 10; x += .25) {
            const value = f.projection.pointValue(new THREE.Vector3(x, 0, .3), 0, 30, f.daughter, Math.max(0, x));
            assert.ok(value >= .2 - 1e-6 && value <= .8 + 1e-6);
        }
        const a = new THREE.Vector3(-3, .4, -2), b = new THREE.Vector3(8, -.5, 2);
        const forward = f.projection.average(0, 30, f.daughter, 8, a, b);
        const reverse = f.projection.average(f.daughter, 8, 0, 30, b, a);
        assert.ok(Math.abs(forward - reverse) < 1e-7, `Reversing one physical ray changes its concentration: ${forward} -> ${reverse}`);
    } finally {f.dispose();}
});

test('subdividing the parent centerline does not change the spatial mixing field', () => {
    const original = junctionContinuityFixture(), subdivided = junctionContinuityFixture(false, true);
    try {
        for (let x = 0; x <= 10; x += .05) {
            const point = new THREE.Vector3(x, .1, .2);
            const a = original.projection.pointValue(point, 0, 30, original.daughter, x);
            const b = subdivided.projection.pointValue(point, 0, 30, subdivided.daughter, x);
            assert.ok(Math.abs(a - b) < 1e-6, `Adding a degree-two parent node changes C at x=${x}: ${a} -> ${b}`);
        }
    } finally {original.dispose(); subdivided.dispose();}
});
