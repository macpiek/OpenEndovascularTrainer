import assert from 'node:assert/strict';
import * as THREE from 'three';
import { generateVessel } from '../src/vesselGeometry.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';

function buildProceduralSampler(vessel) {
    const origin = new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z);
    const forward = new THREE.Vector3(
        vessel.sheath.end.x - vessel.sheath.start.x,
        vessel.sheath.end.y - vessel.sheath.start.y,
        vessel.sheath.end.z - vessel.sheath.start.z
    ).normalize();
    const side = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0));
    if (side.lengthSq() < 1e-8) side.set(1, 0, 0);
    side.normalize();
    const up = new THREE.Vector3().crossVectors(side, forward).normalize();

    const centerAt = distance => {
        const d = Math.max(0, distance);
        const fade = Math.min(1, d / 120);
        return origin.clone()
            .addScaledVector(forward, d)
            .addScaledVector(side, Math.sin(d * 0.034) * 12 * fade)
            .addScaledVector(up, (Math.sin(d * 0.021 + 0.5) - Math.sin(0.5)) * 10 * fade);
    };

    return distance => {
        const d = Math.max(0, distance);
        const point = centerAt(d);
        const ahead = centerAt(d + 0.5);
        const behind = centerAt(Math.max(0, d - 0.5));
        const tangent = ahead.sub(behind).normalize();
        return {
            point: { x: point.x, y: point.y, z: point.z },
            tangent: { x: tangent.x, y: tangent.y, z: tangent.z },
            radius: 9.5 + Math.sin(d * 0.017) * 1.2
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
    lumenSampler: buildProceduralSampler(vessel),
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
const insideLumenDiagnostics = solver.collectLumenDiagnostics({
    meshCollider: {
        pointContact() {
            return {
                signedDistance: 2,
                distance: 2,
                violation: false
            };
        }
    }
}, { clearance: 0.45, contactBand: 1.85 });
const outsideLumenDiagnostics = solver.collectLumenDiagnostics({
    meshCollider: {
        pointContact() {
            return {
                signedDistance: -0.25,
                distance: 0,
                violation: true
            };
        }
    }
}, { clearance: 0.45, contactBand: 1.85 });
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
console.log('guidewire lumen diagnostic samples', insideLumenDiagnostics.checkedCount);
console.log('guidewire first inserted visible', solver.insertedCoordinate(firstInsertedIndex).toFixed(2));
console.log('guidewire first visible inserted', solver.insertedCoordinate(firstVisibleIndex).toFixed(2));

assert.equal(diagnostics.breaches.length, 0, 'guidewire should remain inside the modeled lumen');
assert.ok(diagnostics.contacts.length > 0, 'straightened guidewire should touch the vessel wall somewhere');
assert.ok(insideLumenDiagnostics.checkedCount > 0, 'lumen diagnostics should sample guidewire points outside the sheath');
assert.equal(insideLumenDiagnostics.outsideCount, 0, 'positive signed distances should not count as outside lumen');
assert.equal(insideLumenDiagnostics.clearanceViolationCount, 0, 'positive signed distances above clearance should keep clearance clear');
assert.equal(
    outsideLumenDiagnostics.outsideCount,
    outsideLumenDiagnostics.checkedCount,
    'negative signed distances should count as outside lumen'
);
assert.equal(
    outsideLumenDiagnostics.clearanceViolationCount,
    outsideLumenDiagnostics.checkedCount,
    'negative signed distances should also count as clearance violations'
);
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
    lumenSampler: buildProceduralSampler(vessel),
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
