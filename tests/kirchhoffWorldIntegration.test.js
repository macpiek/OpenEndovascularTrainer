import assert from 'node:assert/strict';
import {
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    conjugateQuaternion,
    materialFrameDirectors,
    multiplyQuaternions,
    quaternionExp,
    quaternionLog
} from '../src/physics/discreteKirchhoffRod.js';

const EPSILON = 1e-8;

function quaternionAt(body, segment) {
    return {
        x: body.orientationX[segment],
        y: body.orientationY[segment],
        z: body.orientationZ[segment],
        w: body.orientationW[segment]
    };
}

function relativeRotation(body, joint) {
    return quaternionLog(multiplyQuaternions(
        conjugateQuaternion(quaternionAt(body, joint - 1)),
        quaternionAt(body, joint)
    ));
}

function createKirchhoffWorldAndBody(id, count = 8, segmentLength = 2, iterations = 30) {
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations,
        penetrationIterations: iterations
    });
    const body = world.createRod(id, count, segmentLength, {
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        kirchhoffBendCompliance: 0,
        kirchhoffTwistCompliance: 0,
        maxBendAngle: 179,
        foldLimitStrength: 0,
        linearDamping: 0,
        angularDamping: 0,
        projectionVelocityRetention: 0,
        sleepVelocity: 0,
        sleepAngularVelocity: 0,
        sleepFrames: 1_000_000,
        postStabilizationPasses: 0
    });
    return { world, body };
}

function settle(world, body, steps = 180) {
    for (let step = 0; step < steps; step++) {
        body.wake();
        world.stepFixed();
    }
}

function maximumLengthError(body) {
    let maximum = 0;
    for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
        const length = Math.hypot(
            body.x[segment + 1] - body.x[segment],
            body.y[segment + 1] - body.y[segment],
            body.z[segment + 1] - body.z[segment]
        );
        maximum = Math.max(maximum, Math.abs(length - body.restLength[segment]));
    }
    return maximum;
}

function signedPlanarTurn(body) {
    let total = 0;
    for (let joint = body.activeStart + 1; joint < body.activeEnd; joint++) {
        const incomingX = body.x[joint] - body.x[joint - 1];
        const incomingY = body.y[joint] - body.y[joint - 1];
        const outgoingX = body.x[joint + 1] - body.x[joint];
        const outgoingY = body.y[joint + 1] - body.y[joint];
        total += Math.atan2(
            incomingX * outgoingY - incomingY * outgoingX,
            incomingX * outgoingX + incomingY * outgoingY
        );
    }
    return total;
}

// Legacy remains the default; merely loading the new module cannot change
// existing guidewire or catheter bodies.
const legacyWorld = new EndovascularPhysicsWorld();
const legacyBody = legacyWorld.createRod('legacy-default', 4, 2);
assert.equal(legacyBody.rodModel, 'legacy');

// A straight Kirchhoff rod starts with Bishop frames adapted to its x-directed
// centerline and remains an exact zero-energy state.
{
    const { world, body } = createKirchhoffWorldAndBody('straight');
    assert.equal(body.rodModel, 'kirchhoff');
    for (let index = 0; index < body.count; index++) {
        assert.equal(body.materialCoordinate[index], index * body.segmentLength);
    }
    settle(world, body, 40);
    for (let index = 0; index < body.count; index++) {
        assert.ok(Math.abs(body.x[index] - index * body.segmentLength) < EPSILON);
        assert.ok(Math.abs(body.y[index]) < EPSILON);
        assert.ok(Math.abs(body.z[index]) < EPSILON);
    }
    for (let segment = 0; segment < body.segmentCount; segment++) {
        const tangent = materialFrameDirectors(quaternionAt(body, segment)).d3;
        assert.ok(tangent.x > 1 - EPSILON);
        assert.ok(Math.abs(tangent.y) < EPSILON);
        assert.ok(Math.abs(tangent.z) < EPSILON);
    }
    assert.ok(maximumLengthError(body) < EPSILON);
}

// A distributed manufactured bend uses the same bend/twist operator on every
// joint. Adaptation transfers the material-frame curvature to the centerline.
{
    const { world, body } = createKirchhoffWorldAndBody('natural-arc', 9, 2, 40);
    body.setPinned(0, true);
    const proximalFrame = quaternionAt(body, 0);
    body.setProximalOrientationControl(
        proximalFrame.x,
        proximalFrame.y,
        proximalFrame.z,
        proximalFrame.w
    );
    const turnPerJoint = Math.PI / 18;
    for (let joint = 1; joint < body.segmentCount; joint++) {
        // For the initial x-directed Bishop frame, material d2 is world z, so
        // this bends the rod in the xy plane.
        body.setKirchhoffRestRotation(joint, 0, turnPerJoint, 0, 0, 0, 0);
    }
    settle(world, body, 220);
    const expectedTurn = (body.segmentCount - 1) * turnPerJoint;
    const centerlineTurn = signedPlanarTurn(body);
    assert.ok(
        Math.abs(centerlineTurn - expectedTurn) < 0.012,
        `natural centerline turn ${centerlineTurn} vs ${expectedTurn}`
    );
    assert.ok(maximumLengthError(body) < 0.003);
    assert.ok(
        Math.max(...Array.from(body.z, value => Math.abs(value))) < 1e-5,
        'planar intrinsic curvature must not become a helix'
    );
}

// Natural torsion rotates material cross-sections while the straight
// centerline and its lengths remain unchanged.
{
    const { world, body } = createKirchhoffWorldAndBody('natural-twist', 7, 2, 30);
    const proximalFrame = quaternionAt(body, 0);
    body.setProximalOrientationControl(
        proximalFrame.x,
        proximalFrame.y,
        proximalFrame.z,
        proximalFrame.w
    );
    const twistPerJoint = 0.12;
    for (let joint = 1; joint < body.segmentCount; joint++) {
        body.setKirchhoffRestRotation(joint, 0, 0, twistPerJoint, 0, 0, 0);
    }
    settle(world, body, 100);
    let representedTwist = 0;
    let representedBend = 0;
    for (let joint = 1; joint < body.segmentCount; joint++) {
        const rotation = relativeRotation(body, joint);
        representedTwist += rotation.z;
        representedBend += Math.hypot(rotation.x, rotation.y);
    }
    assert.ok(Math.abs(
        representedTwist - (body.segmentCount - 1) * twistPerJoint
    ) < 1e-7);
    assert.ok(representedBend < 1e-7);
    assert.ok(maximumLengthError(body) < EPSILON);
    assert.ok(Math.max(...Array.from(body.y, value => Math.abs(value))) < EPSILON);
    assert.ok(Math.max(...Array.from(body.z, value => Math.abs(value))) < EPSILON);
}

function settledArcWithLegacyPoison(poisonLegacyConstraints) {
    const { world, body } = createKirchhoffWorldAndBody(
        poisonLegacyConstraints ? 'single-energy-poisoned' : 'single-energy-reference',
        8,
        2,
        40
    );
    body.setPinned(0, true);
    const proximalFrame = quaternionAt(body, 0);
    body.setProximalOrientationControl(
        proximalFrame.x,
        proximalFrame.y,
        proximalFrame.z,
        proximalFrame.w
    );
    for (let joint = 1; joint < body.segmentCount; joint++) {
        body.setKirchhoffRestRotation(joint, 0, 0.14, 0, 0, 0, 0);
        if (!poisonLegacyConstraints) continue;
        body.restBendChord[joint] = 0.01;
        body.setRestDirectionTarget(joint, -2, 3, 4, 0);
        body.setRestShapeTarget(joint, -50, 25, 10, 0);
    }
    if (poisonLegacyConstraints) {
        body.curvatureVariationEnabled = true;
        body.longStraightSpan = 3;
        body.setShapeClosureTarget(1, body.activeEnd, 0.01, 0);
    }
    settle(world, body, 220);
    return body;
}

// Kirchhoff bodies must not also receive the legacy chord, rest-direction,
// positional rest-shape, straightness or closure energies.
{
    const reference = settledArcWithLegacyPoison(false);
    const poisoned = settledArcWithLegacyPoison(true);
    let maximumDifference = 0;
    for (let index = 0; index < reference.count; index++) {
        maximumDifference = Math.max(maximumDifference, Math.hypot(
            poisoned.x[index] - reference.x[index],
            poisoned.y[index] - reference.y[index],
            poisoned.z[index] - reference.z[index]
        ));
    }
    assert.ok(
        maximumDifference < 1e-6,
        `legacy energy leaked into Kirchhoff body (${maximumDifference} mm)`
    );
}

// Orientation corrections participate in the same integrate/project/velocity
// cycle as positions. A changed proximal rotation therefore produces a finite
// angular velocity instead of teleporting an untracked material frame.
{
    const world = new EndovascularPhysicsWorld({ iterations: 12 });
    const body = world.createRod('angular-velocity', 5, 2, {
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        kirchhoffBendCompliance: 0,
        kirchhoffTwistCompliance: 0,
        foldLimitStrength: 0,
        linearDamping: 1,
        angularDamping: 1,
        projectionVelocityRetention: 1,
        maxBendAngle: 179,
        sleepFrames: 1_000_000
    });
    const initial = quaternionAt(body, 0);
    const target = multiplyQuaternions(
        quaternionExp({ x: 0.08, y: 0, z: 0 }),
        initial
    );
    body.setProximalOrientationControl(
        target.x,
        target.y,
        target.z,
        target.w
    );
    world.stepFixed();
    const angularSpeed = Math.hypot(
        body.angularVelocityX[0],
        body.angularVelocityY[0],
        body.angularVelocityZ[0]
    );
    assert.ok(Number.isFinite(angularSpeed) && angularSpeed > 0.01);
}

console.log('Kirchhoff world integration tests passed');
