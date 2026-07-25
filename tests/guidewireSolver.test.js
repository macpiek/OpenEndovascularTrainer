import assert from 'node:assert/strict';
import * as THREE from 'three';
import { generateVessel } from '../src/vesselGeometry.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';
import {
    GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
    GUIDEWIRE_BODY_BENDING_STIFFNESS,
    GUIDEWIRE_SOFT_TIP_LENGTH_MM,
    GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES,
    GUIDEWIRE_TIP_BENDING_STIFFNESS,
    applyGuidewireMaterialProfile
} from '../src/physics/guidewireMaterialProfile.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';

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

function maxSolverBendAngle(wire, solver) {
    let max = 0;
    for (let i = 1; i < wire.nodes.length - 1; i++) {
        if (solver.insertedCoordinate(i) <= solver.sheathLength) continue;
        max = Math.max(max, wire.bendAngleAt(i) || 0);
    }
    return max;
}

function maxSegmentLengthError(wire, segmentLength) {
    return wire.nodes.slice(1).reduce((max, node, i) => {
        return Math.max(max, Math.abs(Math.hypot(
            node.x - wire.nodes[i].x,
            node.y - wire.nodes[i].y,
            node.z - wire.nodes[i].z
        ) - segmentLength));
    }, 0);
}

const { vessel } = generateVessel(140, 0);
const segmentLength = 5;
const nodeCount = 201;
const guidewireLength = segmentLength * (nodeCount - 1);
const wire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
applyGuidewireMaterialProfile(wire, { segmentLength });
const materialFirstSoftIndex = wire.nodes.findIndex((_, index) => {
    return (wire.nodes.length - 1 - index) * segmentLength < GUIDEWIRE_SOFT_TIP_LENGTH_MM;
});
assert.equal(
    materialFirstSoftIndex,
    wire.nodes.length - GUIDEWIRE_SOFT_TIP_LENGTH_MM / segmentLength,
    'the soft guidewire material should occupy exactly the final 20 mm'
);
assert.ok(
    wire.nodes.slice(0, materialFirstSoftIndex).every(
        node =>
            node.bendingStiffness === GUIDEWIRE_BODY_BENDING_STIFFNESS &&
            node.bendAngleLimit === GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES
    ),
    'the entire guidewire shaft before the final 20 mm should use the stiff material'
);
assert.ok(
    wire.nodes.slice(materialFirstSoftIndex).every(
        node =>
            node.bendingStiffness === GUIDEWIRE_TIP_BENDING_STIFFNESS &&
            node.bendAngleLimit === GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES
    ),
    'only nodes inside the final 20 mm should use the soft-tip material'
);
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
const maxSegmentError = maxSegmentLengthError(wire, segmentLength);

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
assert.ok(insideLumenDiagnostics.worstPoint, 'lumen diagnostics should expose the worst sampled point');
assert.equal(insideLumenDiagnostics.minSignedDistance, 2, 'lumen diagnostics should preserve signed-distance minima');
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

const withdrawalStartProgress = solver.progress;
let withdrawalLastProgress = solver.progress;
for (let frame = 0; frame < 180; frame++) {
    const delta = solver.advance(-1, dt);
    assert.ok(Math.abs(delta + 44 * dt) < 1e-9, 'withdrawal should move freely at commanded speed');
    solver.solve(dt, null, { iterations: 16 });
    assert.ok(
        solver.progress <= withdrawalLastProgress + 1e-9,
        'guidewire progress should decrease monotonically during withdrawal'
    );
    withdrawalLastProgress = solver.progress;
}
for (let frame = 0; frame < 60; frame++) {
    solver.advance(0, dt);
    solver.solve(dt, null, { iterations: 12 });
}
const withdrawalDiagnostics = solver.collectContactSamples(null, 1.85);
const withdrawalSegmentError = maxSegmentLengthError(wire, segmentLength);
const withdrawalMaxBend = maxSolverBendAngle(wire, solver);
console.log('guidewire withdrawn cm', ((withdrawalStartProgress - solver.progress) / 10).toFixed(1));
console.log('guidewire withdrawal breaches', withdrawalDiagnostics.breaches.length);
console.log('guidewire withdrawal max segment error', withdrawalSegmentError.toFixed(5));
console.log('guidewire withdrawal max bend', withdrawalMaxBend.toFixed(2));

assert.ok(
    withdrawalStartProgress - solver.progress > 120,
    'withdrawal test should retract a meaningful length of guidewire'
);
assert.equal(withdrawalDiagnostics.breaches.length, 0, 'withdrawn guidewire should remain inside the modeled lumen');
assert.ok(withdrawalSegmentError < 0.06, 'withdrawal should preserve guidewire segment lengths');
assert.ok(withdrawalMaxBend < 135, 'withdrawal should not create a sharp local fold');

solver.reset();
assert.equal(solver.progress, 0, 'reset should restore the guidewire insertion baseline');
assert.equal(solver.contactPoints.length, 0, 'reset should clear cached contacts');
assert.equal(solver.breachPoints.length, 0, 'reset should clear cached wall breaches');
assert.ok(
    wire.nodes.every(node => node.vx === 0 && node.vy === 0 && node.vz === 0),
    'reset should clear guidewire velocity'
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

const maxInsertWire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
const maxInsertSolver = new GuidewireSolver({
    rod: maxInsertWire,
    segmentLength,
    guidewireLength,
    sheath: vessel.sheath,
    lumenSampler: buildProceduralSampler(vessel),
    advanceRate: 220,
    minInsert: 0,
    maxInsert: guidewireLength,
    lumenClearance: 0.78,
    straightening: 0.72,
    routeBlend: 0.018,
    relaxationIterations: 26,
    lengthIterations: 18,
    segmentSamples: [0.06, 0.12, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.9, 0.94],
    foldGuardAngle: 118,
    foldGuardStrength: 0.85,
    foldGuardPasses: 4,
    foldGuardCenterPull: 1.25
});
maxInsertSolver.initialize();
let maxInsertLastProgress = maxInsertSolver.progress;
for (let frame = 0; frame < 420 && maxInsertSolver.progress < maxInsertSolver.maxInsert - 1e-6; frame++) {
    maxInsertSolver.advance(1, dt);
    maxInsertSolver.solve(dt, null, { iterations: 16 });
    assert.ok(
        maxInsertSolver.progress >= maxInsertLastProgress - 1e-9,
        'guidewire progress should increase monotonically toward max insert'
    );
    maxInsertLastProgress = maxInsertSolver.progress;
}
for (let frame = 0; frame < 120; frame++) {
    maxInsertSolver.advance(0, dt);
    maxInsertSolver.solve(dt, null, { iterations: 14 });
}
const maxInsertDiagnostics = maxInsertSolver.collectContactSamples(null, 1.85);
const maxInsertSegmentError = maxSegmentLengthError(maxInsertWire, segmentLength);
const maxInsertMaxBend = maxSolverBendAngle(maxInsertWire, maxInsertSolver);
console.log('guidewire max inserted cm', (maxInsertSolver.progress / 10).toFixed(1));
console.log('guidewire max insert breaches', maxInsertDiagnostics.breaches.length);
console.log('guidewire max insert segment error', maxInsertSegmentError.toFixed(5));
console.log('guidewire max insert bend', maxInsertMaxBend.toFixed(2));

assert.equal(maxInsertSolver.progress, maxInsertSolver.maxInsert, 'guidewire should reach maximum insertion');
assert.equal(maxInsertDiagnostics.breaches.length, 0, 'max inserted guidewire should remain inside the modeled lumen');
assert.ok(maxInsertDiagnostics.contacts.length > 0, 'max inserted guidewire should still report wall contacts');
assert.ok(maxInsertSegmentError < 0.08, 'max inserted guidewire should preserve guidewire segment lengths');
assert.ok(maxInsertMaxBend < 135, 'max inserted guidewire should not create a sharp local fold');

const slidingSegmentLength = 1;
const slidingNodeCount = 30;
const slidingGuidewireLength = slidingSegmentLength * (slidingNodeCount - 1);
const slidingSheath = {
    start: { x: 0, y: 0, z: 0 },
    end: { x: 0, y: 5, z: 0 },
    radius: 2
};
const slidingWire = new ElasticRod(slidingNodeCount, slidingSegmentLength, { constraintIterations: 8 });
const slidingSolver = new GuidewireSolver({
    rod: slidingWire,
    segmentLength: slidingSegmentLength,
    guidewireLength: slidingGuidewireLength,
    sheath: slidingSheath,
    advanceRate: 1,
    minInsert: 0,
    maxInsert: 20,
    straightening: 0,
    routeBlend: 0,
    relaxationIterations: 1,
    lengthIterations: 1,
    meshClearance: 0.45
});
slidingSolver.initialize();
slidingSolver.advance(1, 12);
for (let i = 0; i < slidingWire.nodes.length; i++) {
    const inserted = slidingSolver.insertedCoordinate(i);
    if (inserted <= slidingSolver.sheathLength) continue;
    slidingWire.nodes[i].x = (inserted - slidingSolver.sheathLength) * 0.5;
    slidingWire.nodes[i].y = inserted;
    slidingWire.nodes[i].z = 0;
}

const slidingIndex = 25;
const slidingBeforeX = slidingWire.nodes[slidingIndex].x;
const slidingBeforeY = slidingWire.nodes[slidingIndex].y;
const tangentPlaneCollision = {
    meshCollider: {
        pointContact(point) {
            return {
                signedDistance: 0.3,
                distance: 0.3,
                violation: false,
                normal: { x: 1, y: 0, z: 0 },
                target: { x: point.x, y: point.y, z: point.z }
            };
        }
    }
};
slidingSolver.advance(1, 1, tangentPlaneCollision);
assert.ok(
    slidingWire.nodes[slidingIndex].x <= slidingBeforeX + 1e-9,
    'feed convection near the STL wall should remove the outward normal component'
);
assert.ok(
    slidingWire.nodes[slidingIndex].y > slidingBeforeY,
    'feed convection near the STL wall should preserve tangential sliding'
);

const boundaryWire = new ElasticRod(slidingNodeCount, slidingSegmentLength, {
    constraintIterations: 8
});
applyGuidewireMaterialProfile(boundaryWire, { segmentLength: slidingSegmentLength });
const boundarySolver = new GuidewireSolver({
    rod: boundaryWire,
    segmentLength: slidingSegmentLength,
    guidewireLength: slidingGuidewireLength,
    sheath: slidingSheath,
    advanceRate: 1,
    minInsert: 0,
    maxInsert: 24,
    straightening: 0,
    routeBlend: 0,
    relaxationIterations: 1,
    lengthIterations: 1
});
boundarySolver.initialize();
boundarySolver.advance(1, 18);
const firstBoundaryFreeIndex = boundaryWire.nodes.findIndex((_, index) => {
    return boundarySolver.insertedCoordinate(index) > boundarySolver.sheathLength;
});
const boundaryBend = Math.PI / 6;
for (let index = firstBoundaryFreeIndex; index < boundaryWire.nodes.length; index++) {
    const local = index - firstBoundaryFreeIndex + 1;
    boundaryWire.nodes[index].x = Math.sin(boundaryBend) * local * slidingSegmentLength;
    boundaryWire.nodes[index].y = slidingSheath.end.y +
        Math.cos(boundaryBend) * local * slidingSegmentLength;
    boundaryWire.nodes[index].z = 0;
}
const boundaryInitialPose = boundaryWire.nodes.map(node => ({
    x: node.x,
    y: node.y,
    z: node.z
}));
const boundaryBendIndex = firstBoundaryFreeIndex - 1;
const boundaryInitialBend = boundaryWire.bendAngleAt(boundaryBendIndex);
const boundaryProbeIndex = firstBoundaryFreeIndex + 5;
const boundaryProbeBefore = {
    x: boundaryWire.nodes[boundaryProbeIndex].x,
    y: boundaryWire.nodes[boundaryProbeIndex].y,
    z: boundaryWire.nodes[boundaryProbeIndex].z
};
boundarySolver.advance(1, 0.5, null, {
    routeAssist: false,
    boundaryDriven: true
});
const boundaryProbeAfter = boundaryWire.nodes[boundaryProbeIndex];
assert.ok(
    Math.hypot(
        boundaryProbeAfter.x - boundaryProbeBefore.x,
        boundaryProbeAfter.y - boundaryProbeBefore.y,
        boundaryProbeAfter.z - boundaryProbeBefore.z
    ) < 1e-9,
    'boundary-driven feed should leave free material nodes for XPBD to move instead of replaying the old path'
);
assert.equal(
    boundarySolver.getPerformanceStats().boundaryDrivenFeed,
    true,
    'boundary-driven feed should be visible in solver diagnostics'
);

const boundaryWorld = new EndovascularPhysicsWorld({
    fixedDt: 1 / 120,
    iterations: 8,
    penetrationIterations: 8
});
const boundaryBody = boundaryWorld.createRod(
    'boundary-driven-guidewire',
    slidingNodeCount,
    slidingSegmentLength,
    {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        sleepFrames: 1000
    }
);
boundaryBody.syncFromElasticRod(boundaryWire, { resetVelocity: false });
boundaryBody.setActiveRange(
    Math.max(0, boundarySolver.firstInsertedNodeIndex() - 1),
    boundaryBody.count - 1
);
for (let step = 0; step < 240; step++) {
    boundarySolver.advance(1, 1 / 120, null, {
        routeAssist: false,
        boundaryDriven: true
    });
    boundaryBody.syncFromElasticRod(boundaryWire, { resetVelocity: false });
    boundaryBody.setActiveRange(
        Math.max(0, boundarySolver.firstInsertedNodeIndex() - 1),
        boundaryBody.count - 1
    );
    boundaryWorld.stepFixed();
    boundaryBody.syncToElasticRod(boundaryWire);
}
for (let step = 0; step < 300; step++) {
    boundarySolver.advance(-1, 1 / 120, null, {
        routeAssist: false,
        boundaryDriven: true
    });
    boundaryBody.syncFromElasticRod(boundaryWire, { resetVelocity: false });
    boundaryBody.setActiveRange(
        Math.max(0, boundarySolver.firstInsertedNodeIndex() - 1),
        boundaryBody.count - 1
    );
    boundaryWorld.stepFixed();
    boundaryBody.syncToElasticRod(boundaryWire);
}
const boundaryCycleBend = boundaryWire.bendAngleAt(boundaryBendIndex);
const boundaryCyclePathChange = maxNodeDrift(boundaryInitialPose, boundaryWire.nodes);
console.log('boundary-driven path change', boundaryCyclePathChange.toFixed(4));
console.log('boundary-driven bend', boundaryInitialBend.toFixed(2), '->', boundaryCycleBend.toFixed(2));
console.log(
    'boundary-driven max segment error',
    maxSegmentLengthError(boundaryWire, slidingSegmentLength).toFixed(5)
);
assert.ok(
    boundaryCyclePathChange > 0.25,
    'changing axial load should produce a new guidewire equilibrium instead of replaying the insertion path'
);
assert.ok(
    boundaryCycleBend < boundaryInitialBend * 0.75,
    'the straight-rest XPBD rod should release stored curvature through an insertion-withdrawal cycle'
);
assert.ok(
    maxSegmentLengthError(boundaryWire, slidingSegmentLength) < 0.02,
    'boundary-driven stress redistribution should preserve the rigid guidewire length'
);

const retractingWire = new ElasticRod(slidingNodeCount, slidingSegmentLength, { constraintIterations: 8 });
const retractingSolver = new GuidewireSolver({
    rod: retractingWire,
    segmentLength: slidingSegmentLength,
    guidewireLength: slidingGuidewireLength,
    sheath: slidingSheath,
    advanceRate: 1,
    minInsert: 0,
    maxInsert: 24,
    straightening: 0,
    routeBlend: 0,
    relaxationIterations: 2,
    lengthIterations: 3,
    meshClearance: 0.45,
    withdrawalStraightening: 0.8,
    withdrawalStraighteningPasses: 3
});
retractingSolver.initialize();
retractingSolver.advance(1, 18);
const firstRetractingFreeIndex = retractingWire.nodes.findIndex((_, index) => {
    return retractingSolver.insertedCoordinate(index) > retractingSolver.sheathLength;
});
const bendIndex = firstRetractingFreeIndex + 3;
const bendAngle = Math.PI / 4;
for (let i = firstRetractingFreeIndex; i < retractingWire.nodes.length; i++) {
    const local = i - firstRetractingFreeIndex;
    const node = retractingWire.nodes[i];
    if (local <= 3) {
        node.x = slidingSheath.end.x;
        node.y = slidingSheath.end.y + local * slidingSegmentLength;
    } else {
        const angled = local - 3;
        node.x = slidingSheath.end.x + Math.sin(bendAngle) * angled * slidingSegmentLength;
        node.y = slidingSheath.end.y + 3 * slidingSegmentLength +
            Math.cos(bendAngle) * angled * slidingSegmentLength;
    }
    node.z = 0;
}
const retractingBeforeBend = retractingWire.bendAngleAt(bendIndex);
retractingSolver.advance(-1, 1, tangentPlaneCollision);
retractingSolver.solve(dt, tangentPlaneCollision, { iterations: 2 });
const retractingAfterBend = retractingWire.bendAngleAt(bendIndex);
assert.ok(
    retractingAfterBend < retractingBeforeBend - 5,
    'withdrawal should relax a stored guidewire bend instead of preserving its frozen angle'
);

const freeEndWire = new ElasticRod(slidingNodeCount, slidingSegmentLength, { constraintIterations: 8 });
const freeEndSolver = new GuidewireSolver({
    rod: freeEndWire,
    segmentLength: slidingSegmentLength,
    guidewireLength: slidingGuidewireLength,
    sheath: slidingSheath,
    advanceRate: 1,
    minInsert: 0,
    maxInsert: 24,
    straightening: 0,
    routeBlend: 0,
    relaxationIterations: 2,
    lengthIterations: 3,
    meshClearance: 0.45,
    withdrawalStraightening: 0.8,
    withdrawalStraighteningPasses: 3
});
freeEndSolver.initialize();
freeEndSolver.advance(1, 18);
const firstFreeEndIndex = freeEndWire.nodes.findIndex((_, index) => {
    return freeEndSolver.insertedCoordinate(index) > freeEndSolver.sheathLength;
});
const freeEndBendIndex = firstFreeEndIndex + 3;
for (let i = firstFreeEndIndex; i < freeEndWire.nodes.length; i++) {
    const local = i - firstFreeEndIndex;
    const node = freeEndWire.nodes[i];
    if (local <= 3) {
        node.x = slidingSheath.end.x;
        node.y = slidingSheath.end.y + local * slidingSegmentLength;
    } else {
        const angled = local - 3;
        node.x = slidingSheath.end.x + Math.sin(bendAngle) * angled * slidingSegmentLength;
        node.y = slidingSheath.end.y + 3 * slidingSegmentLength +
            Math.cos(bendAngle) * angled * slidingSegmentLength;
    }
    node.z = 0;
}
freeEndSolver.lastAdvanceDelta = 0;
freeEndSolver.settleFramesRemaining = 0;
freeEndSolver.withdrawalRelaxFramesRemaining = 0;
const freeEndBeforeBend = freeEndWire.bendAngleAt(freeEndBendIndex);
freeEndSolver.advance(0, dt);
freeEndSolver.solve(dt, null, { iterations: 2 });
const freeEndAfterBend = freeEndWire.bendAngleAt(freeEndBendIndex);
assert.ok(
    freeEndAfterBend < freeEndBeforeBend - 5,
    'unsupported free guidewire end should self-straighten even after withdrawal input has stopped'
);

const advancingFreeEndWire = new ElasticRod(slidingNodeCount, slidingSegmentLength, { constraintIterations: 8 });
const advancingFreeEndSolver = new GuidewireSolver({
    rod: advancingFreeEndWire,
    segmentLength: slidingSegmentLength,
    guidewireLength: slidingGuidewireLength,
    sheath: slidingSheath,
    advanceRate: 1,
    minInsert: 0,
    maxInsert: 24,
    straightening: 0,
    routeBlend: 0,
    relaxationIterations: 2,
    lengthIterations: 3,
    meshClearance: 0.45,
    withdrawalStraightening: 0.8,
    withdrawalStraighteningPasses: 3
});
advancingFreeEndSolver.initialize();
advancingFreeEndSolver.advance(1, 18);
const firstAdvancingFreeEndIndex = advancingFreeEndWire.nodes.findIndex((_, index) => {
    return advancingFreeEndSolver.insertedCoordinate(index) > advancingFreeEndSolver.sheathLength;
});
for (let i = firstAdvancingFreeEndIndex; i < advancingFreeEndWire.nodes.length; i++) {
    const local = i - firstAdvancingFreeEndIndex;
    const node = advancingFreeEndWire.nodes[i];
    if (local <= 3) {
        node.x = slidingSheath.end.x;
        node.y = slidingSheath.end.y + local * slidingSegmentLength;
    } else {
        const angled = local - 3;
        node.x = slidingSheath.end.x + Math.sin(bendAngle) * angled * slidingSegmentLength;
        node.y = slidingSheath.end.y + 3 * slidingSegmentLength +
            Math.cos(bendAngle) * angled * slidingSegmentLength;
    }
    node.z = 0;
}
advancingFreeEndSolver.advance(1, dt);
advancingFreeEndSolver.solve(dt, null, { iterations: 2 });
assert.equal(
    advancingFreeEndSolver.getPerformanceStats().withdrawalRelaxed,
    false,
    'active guidewire advance should not trigger the withdrawal/free-end relaxation mode'
);

function assertHairpinGuardAtProgress(progress) {
    const foldWire = new ElasticRod(16, 5, { constraintIterations: 12 });
    const foldGuidewireLength = 5 * (foldWire.nodes.length - 1);
    const foldSolver = new GuidewireSolver({
        rod: foldWire,
        segmentLength: 5,
        guidewireLength: foldGuidewireLength,
        sheath: slidingSheath,
        advanceRate: progress,
        minInsert: 0,
        maxInsert: foldGuidewireLength,
        straightening: 0,
        routeBlend: 0,
        relaxationIterations: 1,
        lengthIterations: 8,
        foldGuardAngle: 118,
        foldGuardStrength: 0.85,
        foldGuardPasses: 4,
        foldGuardCenterPull: 1.25
    });
    foldSolver.initialize();
    foldSolver.advance(1, 1);
    for (let i = 0; i < foldWire.nodes.length; i++) {
        const inserted = foldSolver.insertedCoordinate(i);
        if (inserted <= foldSolver.sheathLength) continue;
        const local = inserted - foldSolver.sheathLength;
        foldWire.nodes[i].x = local < 20 ? local : 40 - local;
        foldWire.nodes[i].y = 12;
        foldWire.nodes[i].z = 0;
    }
    assert.ok(
        maxSolverBendAngle(foldWire, foldSolver) > 145,
        `synthetic setup should start with a sharp hairpin at ${progress} mm`
    );
    foldSolver.solve(dt, null, { iterations: 1, forceRelax: true });
    assert.ok(
        maxSolverBendAngle(foldWire, foldSolver) < 122,
        `hairpin guard should reduce sharp local folding at ${progress} mm`
    );
    assert.ok(
        maxSegmentLengthError(foldWire, 5) < 0.12,
        `hairpin guard should preserve rigid guidewire segment lengths at ${progress} mm`
    );
}

for (const progress of [44, 60, 72]) {
    assertHairpinGuardAtProgress(progress);
}
