import assert from 'node:assert/strict';
import * as THREE from 'three';
import { generateVessel } from '../src/vesselGeometry.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';

function buildLumenSampler(vessel) {
    const path = [
        { point: new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z), radius: 2.8 },
        { point: new THREE.Vector3(-71, -374, 12), radius: 3.4 },
        { point: new THREE.Vector3(-68, -365, 10), radius: 4.6 },
        { point: new THREE.Vector3(-60, -355, 2), radius: 7.2 },
        { point: new THREE.Vector3(-28, -338, -8), radius: 10.8 },
        { point: new THREE.Vector3(-10, -315, 2), radius: 12.5 },
        { point: new THREE.Vector3(-14, -290, 8), radius: 13.8 },
        { point: new THREE.Vector3(0, -230, 0), radius: 17.5 },
        { point: new THREE.Vector3(0, -160, 0), radius: 18.0 },
        { point: new THREE.Vector3(-9, -125, -6), radius: 17.0 },
        { point: new THREE.Vector3(28, -100, -33), radius: 16.0 },
        { point: new THREE.Vector3(38, -60, -49), radius: 13.0 },
        { point: new THREE.Vector3(36, -25, -56), radius: 13.0 },
        { point: new THREE.Vector3(19, 0, -22), radius: 16.0 },
        { point: new THREE.Vector3(-8, 35, -18), radius: 18.0 },
        { point: new THREE.Vector3(3, 95, -18), radius: 18.0 },
        { point: new THREE.Vector3(10, 145, -18), radius: 18.0 },
        { point: new THREE.Vector3(20, 230, -18), radius: 18.0 },
        { point: new THREE.Vector3(32, 330, -18), radius: 18.0 }
    ];
    const radiusSegments = [];
    let radiusLength = 0;
    for (let i = 0; i < path.length - 1; i++) {
        const start = path[i].point;
        const end = path[i + 1].point;
        const length = start.distanceTo(end);
        radiusSegments.push({
            startRadius: path[i].radius,
            endRadius: path[i + 1].radius,
            length,
            offset: radiusLength
        });
        radiusLength += length;
    }

    const curve = new THREE.CatmullRomCurve3(
        path.map(entry => entry.point),
        false,
        'centripetal',
        0.35
    );
    curve.arcLengthDivisions = Math.max(200, path.length * 48);
    const curveLength = curve.getLength();
    const sampleSpacing = 2.5;
    const sampleCount = Math.max(2, Math.ceil(curveLength / sampleSpacing) + 1);
    const samples = [];

    const sampleRadius = distance => {
        const d = Math.max(0, distance);
        for (const seg of radiusSegments) {
            if (d <= seg.offset + seg.length) {
                const t = Math.max(0, Math.min(1, (d - seg.offset) / Math.max(1e-6, seg.length)));
                return seg.startRadius * (1 - t) + seg.endRadius * t;
            }
        }
        return radiusSegments[radiusSegments.length - 1].endRadius;
    };

    for (let i = 0; i < sampleCount; i++) {
        const u = i / (sampleCount - 1);
        const point = curve.getPointAt(u);
        const tangent = curve.getTangentAt(u).normalize();
        samples.push({
            distance: u * curveLength,
            x: point.x,
            y: point.y,
            z: point.z,
            tx: tangent.x,
            ty: tangent.y,
            tz: tangent.z,
            radius: sampleRadius(u * radiusLength)
        });
    }

    return distance => {
        const d = Math.max(0, distance);
        if (d <= curveLength) {
            const samplePosition = d / Math.max(1e-6, curveLength) * (samples.length - 1);
            const lowerIndex = Math.max(0, Math.min(samples.length - 2, Math.floor(samplePosition)));
            const upperIndex = lowerIndex + 1;
            const t = samplePosition - lowerIndex;
            const a = samples[lowerIndex];
            const b = samples[upperIndex];
            let tx = a.tx * (1 - t) + b.tx * t;
            let ty = a.ty * (1 - t) + b.ty * t;
            let tz = a.tz * (1 - t) + b.tz * t;
            const tangentLength = Math.hypot(tx, ty, tz) || 1;
            tx /= tangentLength;
            ty /= tangentLength;
            tz /= tangentLength;
            return {
                point: {
                    x: a.x * (1 - t) + b.x * t,
                    y: a.y * (1 - t) + b.y * t,
                    z: a.z * (1 - t) + b.z * t
                },
                tangent: { x: tx, y: ty, z: tz },
                radius: a.radius * (1 - t) + b.radius * t
            };
        }

        const last = samples[samples.length - 1];
        const overrun = d - curveLength;
        return {
            point: {
                x: last.x + last.tx * overrun,
                y: last.y + last.ty * overrun,
                z: last.z + last.tz * overrun
            },
            tangent: { x: last.tx, y: last.ty, z: last.tz },
            radius: radiusSegments[radiusSegments.length - 1].endRadius
        };
    };
}

function maxNodeDrift(a, b) {
    let max = 0;
    for (let i = 0; i < a.length; i++) {
        max = Math.max(max, Math.hypot(a[i].x - b[i].x, a[i].y - b[i].y, a[i].z - b[i].z));
    }
    return max;
}

const { vessel } = generateVessel(140, 0);
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const wire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
const solver = new GuidewireSolver({
    rod: wire,
    segmentLength,
    guidewireLength,
    sheath: vessel.sheath,
    lumenSampler: buildLumenSampler(vessel),
    advanceRate: 44,
    minInsert: 0,
    maxInsert: guidewireLength,
    lumenClearance: 0.78,
    straightening: 0.72,
    routeBlend: 0.018,
    relaxationIterations: 26,
    lengthIterations: 18,
    segmentSamples: [0.06, 0.12, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.9, 0.94]
});
solver.initialize();

const dt = 1 / 60;
for (let frame = 0; frame < 900; frame++) {
    const delta = solver.advance(1, dt);
    if (solver.progress < solver.maxInsert - 1e-6) {
        assert.ok(Math.abs(delta - 44 * dt) < 1e-9);
    }
    solver.solve(dt, null, { iterations: 22 });
}

for (let frame = 0; frame < 180; frame++) {
    solver.advance(0, dt);
    solver.solve(dt, null, { iterations: 18 });
}
const settledA = wire.nodes.map(node => ({ x: node.x, y: node.y, z: node.z }));
for (let frame = 0; frame < 180; frame++) {
    solver.advance(0, dt);
    solver.solve(dt, null, { iterations: 18 });
}
const settledB = wire.nodes.map(node => ({ x: node.x, y: node.y, z: node.z }));

const diagnostics = solver.collectContactSamples(null, 1.85);
const firstInsertedIndex = solver.firstInsertedNodeIndex();
const firstVisibleIndex = solver.firstLumenNodeIndex();
const maxSegmentError = wire.nodes.slice(1).reduce((max, node, i) => {
    return Math.max(max, Math.abs(Math.hypot(
        node.x - wire.nodes[i].x,
        node.y - wire.nodes[i].y,
        node.z - wire.nodes[i].z
    ) - segmentLength));
}, 0);

console.log('guidewire inserted cm', (solver.progress / 10).toFixed(1));
console.log('guidewire contacts', diagnostics.contacts.length);
console.log('guidewire breaches', diagnostics.breaches.length);
console.log('guidewire rest drift', maxNodeDrift(settledA, settledB).toFixed(5));
console.log('guidewire max segment error', maxSegmentError.toFixed(5));
console.log('guidewire first inserted visible', solver.insertedCoordinate(firstInsertedIndex).toFixed(2));
console.log('guidewire first visible inserted', solver.insertedCoordinate(firstVisibleIndex).toFixed(2));

assert.equal(diagnostics.breaches.length, 0, 'guidewire should remain inside the modeled lumen');
assert.ok(diagnostics.contacts.length > 0, 'straightened guidewire should touch the vessel wall somewhere');
assert.ok(maxNodeDrift(settledA, settledB) < 0.025, 'guidewire should be stable at rest');
assert.ok(maxSegmentError < 0.035, 'guidewire segment lengths should stay rigid');
assert.ok(
    firstInsertedIndex >= wire.nodes.length ||
        solver.insertedCoordinate(firstInsertedIndex) >= 0,
    'rendered guidewire should start at the sheath entrance, not at the vessel lumen'
);
assert.ok(
    firstVisibleIndex >= wire.nodes.length ||
        solver.insertedCoordinate(firstVisibleIndex) >= solver.sheathLength,
    'lumen-only diagnostics should still start at the vessel lumen'
);

const earlyWire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
const earlySolver = new GuidewireSolver({
    rod: earlyWire,
    segmentLength,
    guidewireLength,
    sheath: vessel.sheath,
    lumenSampler: buildLumenSampler(vessel),
    advanceRate: 44,
    minInsert: 0,
    maxInsert: guidewireLength,
    lumenClearance: 0.78,
    straightening: 0.72,
    routeBlend: 0.018,
    relaxationIterations: 26,
    lengthIterations: 18,
    segmentSamples: [0.06, 0.12, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.9, 0.94]
});
earlySolver.initialize();
for (let frame = 0; frame < 18; frame++) {
    earlySolver.advance(1, dt);
    earlySolver.solve(dt, null, { iterations: 8 });
}
assert.ok(earlySolver.progress > 12, 'guidewire should continue advancing immediately in the sheath');
assert.ok(
    earlySolver.firstInsertedNodeIndex() < earlyWire.nodes.length,
    'partially introduced guidewire should be visible before it reaches the vessel lumen'
);
assert.ok(
    earlySolver.firstLumenNodeIndex() >= earlyWire.nodes.length,
    'early guidewire should not be treated as a lumen segment before it leaves the sheath'
);
