import assert from 'node:assert/strict';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    applyGuidewireMaterialProfile,
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
    STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD
} from '../src/physics/guidewireMaterialProfile.js';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { BERENSTEIN_NATURAL_BEND_ANGLE_RAD } from '../src/physics/catheterMaterialProfile.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';

function createBody(id, count, profile = DEFAULT_TOOL_PROFILES.guidewire) {
    const world = new EndovascularPhysicsWorld();
    return world.createRod(id, count, 3, { ...profile });
}

function sumRange(array, start, end) {
    let sum = 0;
    for (let index = start; index <= end; index++) sum += array[index];
    return sum;
}

function assertArrayNear(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let index = 0; index < actual.length; index++) {
        assert.ok(
            Math.abs(actual[index] - expected[index]) <= tolerance,
            `array value ${index}: ${actual[index]} vs ${expected[index]}`
        );
    }
}

// A complete guidewire range can migrate from legacy without capturing its
// deliberately deformed centerline as manufactured rest curvature.
{
    const body = createBody('steel-j-full', 9);
    const length = STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM;
    const materialCoordinates = Float64Array.from(
        [0, 0.05, 0.14, 0.28, 0.45, 0.63, 0.79, 0.91, 1],
        fraction => fraction * length
    );
    for (let index = 0; index < body.count; index++) {
        body.x[index] = index * 1.3;
        body.y[index] = Math.sin(index * 0.8) * 6;
        body.z[index] = Math.cos(index * 0.6) * 3;
    }
    const result = applyKirchhoffMaterialProfile(body, 'steel-j-035', {
        activeStart: 0,
        activeEnd: body.count - 1,
        materialCoordinates,
        tipCoordinate: length
    });
    assert.equal(body.rodModel, 'kirchhoff');
    assert.deepEqual(Array.from(body.materialCoordinate), Array.from(materialCoordinates));
    assert.ok(Math.abs(
        sumRange(body.restRotation1, 1, body.count - 2) -
        STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD
    ) < 1e-10);
    assert.equal(sumRange(body.restRotation2, 1, body.count - 2), 0);
    assert.equal(sumRange(body.restRotation3, 1, body.count - 2), 0);
    assert.equal(result.changedJoints.length, body.count - 2);
    for (const joint of result.changedJoints) {
        assert.ok(body.kirchhoffBendCompliance1[joint] > 0);
        assert.ok(body.kirchhoffBendCompliance2[joint] > 0);
        assert.ok(body.kirchhoffTwistCompliance[joint] > 0);
    }

    // Reapplying an identical constitutive state cannot wake the body or erase
    // the accumulated XPBD multipliers.
    for (let joint = 1; joint < body.segmentCount; joint++) {
        body.bendTwistLambda1[joint] = 10 + joint;
        body.bendTwistLambda2[joint] = 20 + joint;
        body.bendTwistLambda3[joint] = 30 + joint;
    }
    body.sleeping = true;
    body.sleepCounter = 73;
    const lambda1 = Float64Array.from(body.bendTwistLambda1);
    const lambda2 = Float64Array.from(body.bendTwistLambda2);
    const lambda3 = Float64Array.from(body.bendTwistLambda3);
    const unchanged = applyKirchhoffMaterialProfile(body, 'steel-j-035', {
        activeStart: 0,
        activeEnd: body.count - 1,
        materialCoordinates,
        tipCoordinate: length
    });
    assert.deepEqual(unchanged.changedJoints, []);
    assert.equal(body.sleeping, true);
    assert.equal(body.sleepCounter, 73);
    assert.deepEqual(body.bendTwistLambda1, lambda1);
    assert.deepEqual(body.bendTwistLambda2, lambda2);
    assert.deepEqual(body.bendTwistLambda3, lambda3);

    // Live spatial deformation is not an input to profile application. Kappa0
    // and warm-start state therefore remain bit-for-bit unchanged.
    const rest1 = Float64Array.from(body.restRotation1);
    const rest2 = Float64Array.from(body.restRotation2);
    const rest3 = Float64Array.from(body.restRotation3);
    for (let index = 0; index < body.count; index++) {
        body.x[index] += Math.sin(index * 1.7) * 20;
        body.y[index] -= index * index * 0.75;
        body.z[index] += Math.cos(index * 0.3) * 11;
    }
    const afterLivePoseChange = applyKirchhoffMaterialProfile(
        body,
        'steel-j-035',
        {
            activeStart: 0,
            activeEnd: body.count - 1,
            materialCoordinates,
            tipCoordinate: length
        }
    );
    assert.deepEqual(afterLivePoseChange.changedJoints, []);
    assert.deepEqual(body.restRotation1, rest1);
    assert.deepEqual(body.restRotation2, rest2);
    assert.deepEqual(body.restRotation3, rest3);
    assert.deepEqual(body.bendTwistLambda1, lambda1);
}

// A catheter subset changes only active material coordinates and active joints.
{
    const body = createBody('berenstein-subset', 11, DEFAULT_TOOL_PROFILES.catheter);
    body.enableKirchhoff(true);
    body.materialCoordinate.fill(-100);
    body.restRotation1.fill(7);
    body.restRotation2.fill(8);
    body.restRotation3.fill(9);
    body.kirchhoffBendCompliance1.fill(0.7);
    body.kirchhoffBendCompliance2.fill(0.8);
    body.kirchhoffTwistCompliance.fill(0.9);
    body.bendTwistLambda1.fill(101);
    body.bendTwistLambda2.fill(102);
    body.bendTwistLambda3.fill(103);
    const activeStart = 2;
    const activeEnd = 8;
    const fullCoordinates = Float64Array.from(
        { length: body.count },
        (_, index) => index * 3
    );
    const result = applyKirchhoffMaterialProfile(body, 'berenstein', {
        activeStart,
        activeEnd,
        materialCoordinates: fullCoordinates,
        tipCoordinate: fullCoordinates[activeEnd]
    });
    assert.equal(body.activeStart, activeStart);
    assert.equal(body.activeEnd, activeEnd);
    assert.ok(Math.abs(
        sumRange(body.restRotation1, activeStart + 1, activeEnd - 1) -
        BERENSTEIN_NATURAL_BEND_ANGLE_RAD
    ) < 1e-10);
    assert.deepEqual(
        result.changedJoints,
        Array.from(
            { length: activeEnd - activeStart - 1 },
            (_, index) => activeStart + 1 + index
        )
    );
    for (let node = 0; node < body.count; node++) {
        if (node >= activeStart && node <= activeEnd) {
            assert.equal(body.materialCoordinate[node], fullCoordinates[node]);
        } else {
            assert.equal(body.materialCoordinate[node], -100);
        }
    }
    for (const joint of [1, 2, 8, 9]) {
        assert.equal(body.restRotation1[joint], 7);
        assert.equal(body.bendTwistLambda1[joint], 101);
        assert.equal(body.bendTwistLambda2[joint], 102);
        assert.equal(body.bendTwistLambda3[joint], 103);
    }

    // A local active-coordinate array can infer its tip from its last entry;
    // callers do not need to translate the activeEnd index back to body space.
    body.sleeping = true;
    const activeCoordinates = fullCoordinates.slice(activeStart, activeEnd + 1);
    const inferredTip = applyKirchhoffMaterialProfile(body, 'berenstein', {
        activeStart,
        activeEnd,
        materialCoordinates: activeCoordinates
    });
    assert.deepEqual(inferredTip.changedJoints, []);
    assert.equal(body.sleeping, true);
}

// A local remesh resets only joints whose integrated rest strain or effective
// compliance actually changes; remote warm starts survive.
{
    const body = createBody('local-remesh', 10, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        rodModel: 'kirchhoff'
    });
    const tipCoordinate = STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM;
    const coordinates = Float64Array.from(
        [0, 2, 4, 6, 8, 10, 12, 15, 18, 20]
    );
    applyKirchhoffMaterialProfile(body, 'steel-j-035', {
        activeStart: 0,
        activeEnd: body.count - 1,
        materialCoordinates: coordinates,
        tipCoordinate
    });
    body.bendTwistLambda1.fill(41);
    body.bendTwistLambda2.fill(42);
    body.bendTwistLambda3.fill(43);
    const remeshedCoordinates = Float64Array.from(coordinates);
    remeshedCoordinates[5] += 0.35;
    const remeshed = applyKirchhoffMaterialProfile(body, 'steel-j-035', {
        activeStart: 0,
        activeEnd: body.count - 1,
        materialCoordinates: remeshedCoordinates,
        tipCoordinate
    });
    assert.ok(remeshed.changedJoints.length > 0);
    assert.ok(remeshed.changedJoints.length <= 3);
    for (let joint = 1; joint < body.segmentCount; joint++) {
        if (remeshed.changedJoints.includes(joint)) {
            assert.equal(body.bendTwistLambda1[joint], 0);
            assert.equal(body.bendTwistLambda2[joint], 0);
            assert.equal(body.bendTwistLambda3[joint], 0);
        } else {
            assert.equal(body.bendTwistLambda1[joint], 41);
            assert.equal(body.bendTwistLambda2[joint], 42);
            assert.equal(body.bendTwistLambda3[joint], 43);
        }
    }

    // Translating the material coordinate origin together with the tip changes
    // no s-from-tip sample and must not reset any joint.
    const shiftedCoordinates = Float64Array.from(
        remeshedCoordinates,
        coordinate => coordinate + 500
    );
    body.bendTwistLambda1.fill(51);
    body.bendTwistLambda2.fill(52);
    body.bendTwistLambda3.fill(53);
    body.sleeping = true;
    const shifted = applyKirchhoffMaterialProfile(body, 'steel-j-035', {
        activeStart: 0,
        activeEnd: body.count - 1,
        materialCoordinates: shiftedCoordinates,
        tipCoordinate: tipCoordinate + 500
    });
    assert.deepEqual(shifted.changedJoints, []);
    assert.equal(body.sleeping, true);
    assert.ok(body.bendTwistLambda1.every(value => value === 51));
}

// A live stiffness control scales EI/GJ without changing the manufactured
// rest shape or collapsing the soft-tip taper into a uniform material.
{
    const elasticRod = new ElasticRod(21, 5);
    applyGuidewireMaterialProfile(elasticRod, {
        type: 'glidewire',
        stiffnessScale: 1
    });
    const baseElasticStiffness = Float64Array.from(
        elasticRod.nodeStorage.bendingStiffness
    );
    applyGuidewireMaterialProfile(elasticRod, {
        type: 'glidewire',
        stiffnessScale: 4
    });
    for (let node = 0; node < elasticRod.nodes.length; node++) {
        assert.ok(Math.abs(
            elasticRod.nodeStorage.bendingStiffness[node] -
            baseElasticStiffness[node] * 4
        ) < 1e-9);
    }

    const body = createBody('scaled-glidewire', 21, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        rodModel: 'kirchhoff'
    });
    const coordinates = Float64Array.from(
        { length: body.count },
        (_, index) => index * 5
    );
    const base = applyKirchhoffMaterialProfile(body, 'glidewire', {
        materialCoordinates: coordinates,
        tipCoordinate: coordinates[coordinates.length - 1]
    });
    const baseRest = Float64Array.from(body.restRotation1);
    const baseCompliance = Float64Array.from(body.kirchhoffBendCompliance1);
    const scaled = applyKirchhoffMaterialProfile(body, 'glidewire', {
        materialCoordinates: coordinates,
        tipCoordinate: coordinates[coordinates.length - 1],
        stiffnessScale: 4
    });
    assert.equal(base.stiffnessScale, 1);
    assert.equal(scaled.stiffnessScale, 4);
    assert.deepEqual(body.restRotation1, baseRest);
    assert.ok(scaled.changedJoints.length > 0);
    for (let joint = 1; joint < body.segmentCount; joint++) {
        assert.ok(Math.abs(
            body.kirchhoffBendCompliance1[joint] - baseCompliance[joint] / 4
        ) < 1e-15);
    }

    applyGuidewireMaterialProfile(elasticRod, {
        type: 'glidewire',
        shaftStiffnessScale: 25,
        tipStiffnessScale: 0.5
    });
    assert.ok(Math.abs(
        elasticRod.nodeStorage.bendingStiffness[0] -
        baseElasticStiffness[0] * 25
    ) < 1e-9);
    assert.ok(Math.abs(
        elasticRod.nodeStorage.bendingStiffness.at(-1) -
        baseElasticStiffness.at(-1) * 0.5
    ) < 1e-9);

    const independentlyScaled = applyKirchhoffMaterialProfile(
        body,
        'glidewire',
        {
            materialCoordinates: coordinates,
            tipCoordinate: coordinates[coordinates.length - 1],
            shaftStiffnessScale: 25,
            tipStiffnessScale: 0.5
        }
    );
    assert.equal(independentlyScaled.stiffnessScale, null);
    assert.equal(independentlyScaled.shaftStiffnessScale, 25);
    assert.equal(independentlyScaled.tipStiffnessScale, 0.5);
    assert.deepEqual(body.restRotation1, baseRest);
    assert.ok(Math.abs(
        body.kirchhoffBendCompliance1[1] - baseCompliance[1] / 25
    ) < 1e-15);
    assert.ok(Math.abs(
        body.kirchhoffBendCompliance1[body.segmentCount - 1] -
        baseCompliance[body.segmentCount - 1] / 0.5
    ) < 1e-15);
    assert.throws(
        () => applyKirchhoffMaterialProfile(body, 'glidewire', {
            materialCoordinates: coordinates,
            stiffnessScale: 0
        }),
        /stiffness scale/
    );
}

// Catheters expose the same independent material controls. The preformed
// distal zone uses the tip scale, the straight proximal material uses the
// shaft scale, and neither control changes the manufactured rest curvature.
for (const catheterType of ['pigtail', 'berenstein', 'sim1']) {
    const body = createBody(`scaled-${catheterType}`, 33, {
        ...DEFAULT_TOOL_PROFILES.catheter,
        rodModel: 'kirchhoff'
    });
    const coordinates = Float64Array.from(
        { length: body.count },
        (_, index) => index * 5
    );
    applyKirchhoffMaterialProfile(body, catheterType, {
        materialCoordinates: coordinates,
        tipCoordinate: coordinates.at(-1)
    });
    const baseRest = Float64Array.from(body.restRotation1);
    const baseCompliance = Float64Array.from(body.kirchhoffBendCompliance1);
    const scaled = applyKirchhoffMaterialProfile(body, catheterType, {
        materialCoordinates: coordinates,
        tipCoordinate: coordinates.at(-1),
        shaftStiffnessScale: 9,
        tipStiffnessScale: 0.5
    });
    const shaftJoint = 1;
    const tipJoint = body.segmentCount - 1;
    assert.equal(scaled.stiffnessScale, null);
    assert.equal(scaled.shaftStiffnessScale, 9);
    assert.equal(scaled.tipStiffnessScale, 0.5);
    assert.deepEqual(body.restRotation1, baseRest);
    assert.ok(Math.abs(
        body.kirchhoffBendCompliance1[shaftJoint] -
            baseCompliance[shaftJoint] / 9
    ) < 1e-15, `${catheterType} shaft scale was not applied`);
    assert.ok(Math.abs(
        body.kirchhoffBendCompliance1[tipJoint] -
            baseCompliance[tipJoint] / 0.5
    ) < 1e-15, `${catheterType} tip scale was not applied`);
}

console.log('Kirchhoff material application tests passed');
