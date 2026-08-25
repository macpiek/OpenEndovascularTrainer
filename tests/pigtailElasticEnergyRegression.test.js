import assert from 'node:assert/strict';
import {
    BERENSTEIN_NATURAL_BEND_ANGLE_RAD,
    BERENSTEIN_TIP_SHAPE_LENGTH_MM,
    catheterMaterialProfile,
    integrateBerensteinIntrinsicTurn,
    integratePigtailIntrinsicTurn,
    PIGTAIL_NATURAL_ARC_LENGTH_MM,
    PIGTAIL_NATURAL_TURNS,
    pigtailIntrinsicCurvature,
    pigtailTotalIntrinsicTurn,
    sampleBerensteinRestCenterline,
    samplePigtailRestCenterline,
    integrateSim1IntrinsicTurn,
    sampleSim1RestCenterline,
    SIM1_DISTAL_BEND_ANGLE_RAD,
    SIM1_MAIN_BEND_ANGLE_RAD,
    SIM1_TIP_SHAPE_LENGTH_MM,
    SIM1_TOTAL_TURN_RAD
} from '../src/physics/catheterMaterialProfile.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';

const EXPECTED_TOTAL_TURN = PIGTAIL_NATURAL_TURNS * Math.PI * 2;

assert.equal(
    PIGTAIL_NATURAL_TURNS,
    0.9,
    'the short Pigtail preform should use a sub-circular distal arc'
);

const pigtailProfile = catheterMaterialProfile('pigtail');
const berensteinProfile = catheterMaterialProfile('berenstein');
const sim1Profile = catheterMaterialProfile('sim1');
assert.equal(
    pigtailProfile.integrateIntrinsicTurn,
    integratePigtailIntrinsicTurn,
    'Pigtail must enter the common rod solver through its material profile'
);
assert.equal(
    berensteinProfile.integrateIntrinsicTurn,
    integrateBerensteinIntrinsicTurn,
    'Berenstein must enter the same rod solver through its material profile'
);
assert.equal(
    sim1Profile.integrateIntrinsicTurn,
    integrateSim1IntrinsicTurn,
    'SIM 1 must enter the common rod solver through its material profile'
);
assert.notEqual(
    pigtailProfile.frameNormalSign,
    berensteinProfile.frameNormalSign,
    'opposite manufactured bend orientation belongs to profile data, not a second solver'
);

assert.ok(
    Math.abs(pigtailTotalIntrinsicTurn() - EXPECTED_TOTAL_TURN) < 1e-10,
    'the smooth intrinsic-curvature profile must preserve the prescribed turn'
);
assert.ok(
    pigtailIntrinsicCurvature(PIGTAIL_NATURAL_ARC_LENGTH_MM) < 1e-12,
    'intrinsic curvature must join the straight shaft continuously'
);
const sampledPigtailTip = samplePigtailRestCenterline(
    PIGTAIL_NATURAL_ARC_LENGTH_MM,
    PIGTAIL_NATURAL_ARC_LENGTH_MM
);
assert.ok(
    Math.abs(sampledPigtailTip.turnAngle - EXPECTED_TOTAL_TURN) < 1e-6,
    'the Pigtail preview centerline must integrate the rod intrinsic curvature'
);

for (const spacing of [1, 2.5, 4, 5.75]) {
    let integratedTurn = 0;
    for (let start = 0; start < PIGTAIL_NATURAL_ARC_LENGTH_MM; start += spacing) {
        const length = Math.min(spacing, PIGTAIL_NATURAL_ARC_LENGTH_MM - start);
        integratedTurn += integratePigtailIntrinsicTurn(
            start + length * 0.5,
            length
        );
    }
    assert.ok(
        Math.abs(integratedTurn - EXPECTED_TOTAL_TURN) < 1e-9,
        `integrated Pigtail turn must not depend on ${spacing} mm discretization`
    );
}

for (const spacing of [1, 2.5, 4]) {
    let integratedTurn = 0;
    for (let start = 0; start < BERENSTEIN_TIP_SHAPE_LENGTH_MM; start += spacing) {
        const length = Math.min(spacing, BERENSTEIN_TIP_SHAPE_LENGTH_MM - start);
        integratedTurn += integrateBerensteinIntrinsicTurn(
            start + length * 0.5,
            length
        );
    }
    assert.ok(
        Math.abs(integratedTurn - BERENSTEIN_NATURAL_BEND_ANGLE_RAD) < 1e-9,
        `integrated Berenstein turn must not depend on ${spacing} mm discretization`
    );
}

const partiallyExposedBerenstein = sampleBerensteinRestCenterline(8, 8);
assert.ok(
    Math.abs(partiallyExposedBerenstein.tangentDistance - 8) < 1e-9 &&
        Math.abs(partiallyExposedBerenstein.normalDistance) < 1e-9,
    'the distal 8 mm Berenstein tip must remain straight'
);
const berensteinBendEnd = sampleBerensteinRestCenterline(18, 10);
const berensteinTip = sampleBerensteinRestCenterline(18, 18);
const distalTangentAngle = Math.atan2(
    berensteinTip.normalDistance - berensteinBendEnd.normalDistance,
    berensteinTip.tangentDistance - berensteinBendEnd.tangentDistance
);
assert.ok(
    Math.abs(berensteinTip.turnAngle - BERENSTEIN_NATURAL_BEND_ANGLE_RAD) < 1e-6,
    'the Berenstein seed centerline must integrate the same total intrinsic turn'
);
assert.ok(
    Math.abs(distalTangentAngle - BERENSTEIN_NATURAL_BEND_ANGLE_RAD) < 1e-6,
    'the Berenstein straight distal tip must follow the terminal bend tangent'
);

for (const spacing of [1, 2.5, 4, 5.75]) {
    let integratedTurn = 0;
    for (let start = 0; start < SIM1_TIP_SHAPE_LENGTH_MM; start += spacing) {
        const length = Math.min(spacing, SIM1_TIP_SHAPE_LENGTH_MM - start);
        integratedTurn += integrateSim1IntrinsicTurn(
            start + length * 0.5,
            length
        );
    }
    assert.ok(
        Math.abs(integratedTurn - SIM1_TOTAL_TURN_RAD) < 1e-9,
        `integrated SIM 1 turn must not depend on ${spacing} mm discretization`
    );
}
const sim1MainBendEnd = sampleSim1RestCenterline(
    SIM1_TIP_SHAPE_LENGTH_MM,
    30
);
const sim1BridgeEnd = sampleSim1RestCenterline(
    SIM1_TIP_SHAPE_LENGTH_MM,
    50
);
const sim1Tip = sampleSim1RestCenterline(
    SIM1_TIP_SHAPE_LENGTH_MM,
    SIM1_TIP_SHAPE_LENGTH_MM
);
assert.ok(
    Math.abs(sim1MainBendEnd.turnAngle - SIM1_MAIN_BEND_ANGLE_RAD) < 1e-6,
    'SIM 1 must complete its broad 180-degree return before the bridge'
);
assert.ok(
    Math.abs(sim1BridgeEnd.turnAngle - SIM1_MAIN_BEND_ANGLE_RAD) < 1e-6,
    'SIM 1 bridge must remain straight between its two bends'
);
assert.ok(
    Math.abs(sim1Tip.turnAngle - (
        SIM1_MAIN_BEND_ANGLE_RAD + SIM1_DISTAL_BEND_ANGLE_RAD
    )) < 1e-6,
    'SIM 1 distal leg must open outward through the opposite 30-degree bend'
);
assert.ok(
    sim1Tip.tangentDistance < sim1BridgeEnd.tangentDistance &&
        sim1Tip.normalDistance > sim1BridgeEnd.normalDistance,
    'SIM 1 terminal leg must descend and move outward like the reference shape'
);

// The Pigtail hinge owns one elastic bending energy. An impossible legacy
// chord target on the same joint must have no effect once intrinsic curvature
// is enabled; otherwise the two solvers fight and seed the familiar zig-zag.
const energyWorld = new EndovascularPhysicsWorld({ iterations: 6 });
const energyRod = energyWorld.createRod('single-bend-energy', 5, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter,
    bendCompliance: 0,
    stretchCompliance: 0,
    linearDamping: 1,
    maxBendAngle: 179,
    postStabilizationPasses: 0
});
for (let index = 0; index < energyRod.count; index++) {
    energyRod.setNodePosition(index, index * 4, 0, 0);
}
energyRod.captureRestConfiguration();
energyRod.copyCurrentToPrevious();
energyRod.restBendChord[1] = 0.1;
energyRod.setIntrinsicCurvatureTarget(1, 0, 0, 0, 1, 0, Infinity, 0, 4);
energyWorld.stepFixed();
const twoSegmentChord = Math.hypot(
    energyRod.x[2] - energyRod.x[0],
    energyRod.y[2] - energyRod.y[0],
    energyRod.z[2] - energyRod.z[0]
);
assert.ok(
    twoSegmentChord > 7.9,
    `intrinsic joint must ignore the duplicate unsigned bend energy (${twoSegmentChord} mm)`
);

function settledProfileTurn(profile, iterations) {
    const segmentCount = Math.ceil(profile.naturalArcLength / 2.5);
    const spacing = profile.naturalArcLength / segmentCount;
    const world = new EndovascularPhysicsWorld({ iterations });
    const body = world.createRod(
        `${profile.id}-${iterations}`,
        segmentCount + 1,
        spacing,
        {
            ...DEFAULT_TOOL_PROFILES.catheter,
            maxBendAngle: 179,
            linearDamping: 0.9,
            sleepVelocity: 0,
            sleepFrames: 1_000_000,
            postStabilizationPasses: 0
        }
    );
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, index * spacing, 0, 0);
    }
    body.captureRestConfiguration();
    body.setPinned(0, true);
    let targetTotal = 0;
    for (let joint = 1; joint < body.segmentCount; joint++) {
        const distanceFromTip = (body.segmentCount - joint) * spacing;
        const turn = profile.frameNormalSign *
            profile.integrateIntrinsicTurn(distanceFromTip, spacing);
        targetTotal += turn;
        body.setIntrinsicCurvatureTarget(
            joint,
            turn,
            0,
            0,
            1,
            1e-7,
            0.05,
            0,
            spacing
        );
    }
    for (let step = 0; step < 720; step++) {
        body.wake();
        world.stepFixed();
    }

    let total = 0;
    for (let joint = 1; joint < body.activeEnd; joint++) {
        const ax = body.x[joint] - body.x[joint - 1];
        const ay = body.y[joint] - body.y[joint - 1];
        const bx = body.x[joint + 1] - body.x[joint];
        const by = body.y[joint + 1] - body.y[joint];
        total += Math.atan2(ax * by - ay * bx, ax * bx + ay * by);
    }
    return { total, targetTotal };
}

for (const profile of [pigtailProfile, berensteinProfile]) {
    const results = [4, 8, 16].map(iterations =>
        settledProfileTurn(profile, iterations)
    );
    for (const { total, targetTotal } of results) {
        assert.ok(
            Math.abs(total - targetTotal) < 0.2,
            `${profile.id} free equilibrium must recover its discrete material turn (${total} vs ${targetTotal})`
        );
    }
    const turns = results.map(result => result.total);
    assert.ok(
        Math.max(...turns) - Math.min(...turns) < 0.2,
        `${profile.id} equilibrium must not depend on solver iteration count (${turns.join(', ')})`
    );
}

console.log('Pigtail elastic-energy regressions passed');
