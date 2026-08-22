import assert from 'node:assert/strict';
import {
    adaptMaterialFrameToEdge,
    createBishopFrame,
    evaluateAdaptationConstraint,
    evaluateBendTwistConstraint,
    evaluateKirchhoffHingeEnergy,
    materialFrameDirectors,
    multiplyQuaternions,
    normalizeQuaternion,
    quaternionExp,
    quaternionLog,
    restRotationFromCurvature,
    slerpQuaternions,
    solveAdaptationXPBDArraySweep,
    solveAdaptationXPBD,
    solveBendTwistXPBDArraySweep,
    solveBendTwistXPBD,
    transportBishopFrame
} from '../src/physics/discreteKirchhoffRod.js';

const EPSILON = 1e-9;

function assertNear(actual, expected, tolerance, message) {
    assert.ok(
        Math.abs(actual - expected) <= tolerance,
        `${message}: ${actual} vs ${expected}`
    );
}

function assertVectorNear(actual, expected, tolerance, message) {
    assertNear(actual.x, expected.x, tolerance, `${message} x`);
    assertNear(actual.y, expected.y, tolerance, `${message} y`);
    assertNear(actual.z, expected.z, tolerance, `${message} z`);
}

function quaternionDistance(a, b) {
    const qa = normalizeQuaternion(a);
    const qb = normalizeQuaternion(b);
    const cosine = Math.abs(
        qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
    );
    return 2 * Math.acos(Math.max(-1, Math.min(1, cosine)));
}

function leftPerturb(quaternion, axis, amount) {
    const rotation = { x: 0, y: 0, z: 0 };
    rotation[axis] = amount;
    return normalizeQuaternion(
        multiplyQuaternions(quaternionExp(rotation), quaternion)
    );
}

// Exp/Log must be stable at zero and retain the principal rotation immediately
// below the unavoidable pi branch cut.
for (const rotation of [
    { x: 0, y: 0, z: 0 },
    { x: 1e-10, y: -2e-10, z: 3e-10 },
    { x: 0.2, y: -0.3, z: 0.4 },
    { x: Math.PI - 1e-7, y: 0, z: 0 }
]) {
    const roundTrip = quaternionLog(quaternionExp(rotation));
    assertVectorNear(roundTrip, rotation, 2e-9, 'quaternion Exp/Log round trip');
}

const signedQuaternion = quaternionExp({ x: 0.3, y: -0.2, z: 0.1 });
const negativeQuaternion = {
    x: -signedQuaternion.x,
    y: -signedQuaternion.y,
    z: -signedQuaternion.z,
    w: -signedQuaternion.w
};
assertVectorNear(
    quaternionLog(signedQuaternion),
    quaternionLog(negativeQuaternion),
    EPSILON,
    'quaternion log must be invariant to the double-cover sign'
);

const halfTurn = quaternionExp({ x: 0, y: 0, z: Math.PI });
const halfSlerp = slerpQuaternions(
    { x: 0, y: 0, z: 0, w: 1 },
    halfTurn,
    0.5
);
assertNear(
    quaternionLog(halfSlerp).z,
    Math.PI / 2,
    EPSILON,
    'slerp midpoint must bisect material rotation'
);
assert.ok(
    quaternionDistance(
        slerpQuaternions(signedQuaternion, negativeQuaternion, 0.37),
        signedQuaternion
    ) < 1e-7,
    'slerp must select the shortest representative when q1 = -q0'
);

// Bishop transport changes the tangent by the shortest rotation and adds no
// spin about it. The d2 director remains fixed for this rotation about y.
const bishop = createBishopFrame(
    { x: 0, y: 0, z: 1 },
    { x: 1, y: 0, z: 0 }
);
const diagonalTangent = {
    x: Math.SQRT1_2,
    y: 0,
    z: Math.SQRT1_2
};
const transported = transportBishopFrame(bishop, diagonalTangent);
const transportedDirectors = materialFrameDirectors(transported);
assertVectorNear(
    transportedDirectors.d3,
    diagonalTangent,
    EPSILON,
    'Bishop frame tangent'
);
assertVectorNear(
    transportedDirectors.d2,
    { x: 0, y: 1, z: 0 },
    EPSILON,
    'Bishop transport must not add twist'
);

const reversed = adaptMaterialFrameToEdge(bishop, { x: 0, y: 0, z: -2 });
const reversedDirectors = materialFrameDirectors(reversed);
assertVectorNear(
    reversedDirectors.d3,
    { x: 0, y: 0, z: -1 },
    EPSILON,
    'antiparallel adaptation tangent'
);
assertVectorNear(
    reversedDirectors.d1,
    { x: 1, y: 0, z: 0 },
    EPSILON,
    'antiparallel adaptation must use deterministic d1 axis'
);

// Pointwise material curvature must integrate additively under an edge split.
const fullRestRotation = restRotationFromCurvature(0.12, -0.04, 0.03, 5);
const splitRestA = restRotationFromCurvature(0.12, -0.04, 0.03, 2);
const splitRestB = restRotationFromCurvature(0.12, -0.04, 0.03, 3);
assertVectorNear(fullRestRotation, {
    x: splitRestA.x + splitRestB.x,
    y: splitRestA.y + splitRestB.y,
    z: splitRestA.z + splitRestB.z
}, EPSILON, 'rest strain integral must survive remeshing');

// Analytic adaptation gradients are checked against world-space left
// perturbations, the same convention used by the XPBD update.
const adaptationX0 = { x: -0.2, y: 0.1, z: 0.3 };
const adaptationX1 = { x: 2.1, y: -0.4, z: 1.2 };
const adaptationQ = quaternionExp({ x: 0.2, y: -0.3, z: 0.1 });
const adaptationState = evaluateAdaptationConstraint(
    adaptationX0,
    adaptationX1,
    adaptationQ,
    2.7
);
const componentKeys = ['x', 'y', 'z'];
const finiteDifferenceStep = 1e-6;
for (let angularAxis = 0; angularAxis < 3; angularAxis++) {
    const plus = evaluateAdaptationConstraint(
        adaptationX0,
        adaptationX1,
        leftPerturb(adaptationQ, componentKeys[angularAxis], finiteDifferenceStep),
        2.7
    ).constraint;
    const minus = evaluateAdaptationConstraint(
        adaptationX0,
        adaptationX1,
        leftPerturb(adaptationQ, componentKeys[angularAxis], -finiteDifferenceStep),
        2.7
    ).constraint;
    for (let constraintComponent = 0; constraintComponent < 3; constraintComponent++) {
        const numerical = (
            plus[componentKeys[constraintComponent]] -
            minus[componentKeys[constraintComponent]]
        ) / (2 * finiteDifferenceStep);
        const analytic = adaptationState.orientationGradient[
            constraintComponent * 3 + angularAxis
        ];
        assertNear(numerical, analytic, 5e-7, 'adaptation angular gradient');
    }
}

// Bend/twist Log gradients include the exact SO(3) logarithm Jacobian and
// arbitrary rest curvature, rather than a small-angle shortcut.
const bendQ0 = quaternionExp({ x: 0.2, y: -0.1, z: 0.3 });
const bendQ1 = quaternionExp({ x: -0.15, y: 0.25, z: 0.1 });
const restRotation = { x: 0.1, y: 0.05, z: -0.08 };
const bendState = evaluateBendTwistConstraint(bendQ0, bendQ1, restRotation);
for (const [orientationIndex, orientation] of [[0, bendQ0], [1, bendQ1]]) {
    for (let angularAxis = 0; angularAxis < 3; angularAxis++) {
        const plusQ = leftPerturb(
            orientation,
            componentKeys[angularAxis],
            finiteDifferenceStep
        );
        const minusQ = leftPerturb(
            orientation,
            componentKeys[angularAxis],
            -finiteDifferenceStep
        );
        const plus = evaluateBendTwistConstraint(
            orientationIndex === 0 ? plusQ : bendQ0,
            orientationIndex === 1 ? plusQ : bendQ1,
            restRotation
        ).strain;
        const minus = evaluateBendTwistConstraint(
            orientationIndex === 0 ? minusQ : bendQ0,
            orientationIndex === 1 ? minusQ : bendQ1,
            restRotation
        ).strain;
        const gradient = orientationIndex === 0
            ? bendState.gradient0
            : bendState.gradient1;
        for (let constraintComponent = 0; constraintComponent < 3; constraintComponent++) {
            const numerical = (
                plus[componentKeys[constraintComponent]] -
                minus[componentKeys[constraintComponent]]
            ) / (2 * finiteDifferenceStep);
            const analytic = gradient[constraintComponent * 3 + angularAxis];
            assertNear(numerical, analytic, 8e-7, 'bend/twist angular gradient');
        }
    }
}

// Manufactured relative rotation is the zero-energy configuration under any
// rigid world rotation of the entire pair of material frames.
const baseFrame = quaternionExp({ x: -0.25, y: 0.15, z: 0.4 });
const manufacturedRotation = { x: 0.3, y: -0.2, z: 0.12 };
const manufacturedFrame = multiplyQuaternions(
    baseFrame,
    quaternionExp(manufacturedRotation)
);
const zeroEnergy = evaluateKirchhoffHingeEnergy(
    baseFrame,
    manufacturedFrame,
    manufacturedRotation,
    {
        bendStiffness1: 3,
        bendStiffness2: 5,
        twistStiffness: 7,
        voronoiLength: 4
    }
);
assert.ok(zeroEnergy.totalEnergy < 1e-24, 'rest frame must have zero elastic energy');

const rigidRotation = quaternionExp({ x: 0.4, y: 0.2, z: -0.3 });
const rigidEnergy = evaluateKirchhoffHingeEnergy(
    multiplyQuaternions(rigidRotation, baseFrame),
    multiplyQuaternions(rigidRotation, manufacturedFrame),
    manufacturedRotation,
    {
        bendStiffness1: 3,
        bendStiffness2: 5,
        twistStiffness: 7,
        voronoiLength: 4
    }
);
assertNear(rigidEnergy.totalEnergy, zeroEnergy.totalEnergy, 1e-24,
    'hinge energy must be rigid-rotation invariant');

const knownEnergy = evaluateKirchhoffHingeEnergy(
    { x: 0, y: 0, z: 0, w: 1 },
    quaternionExp({ x: 0.2, y: -0.1, z: 0.3 }),
    { x: 0, y: 0, z: 0 },
    {
        bendStiffness1: 2,
        bendStiffness2: 4,
        twistStiffness: 6,
        voronoiLength: 5
    }
);
assertNear(
    knownEnergy.totalEnergy,
    0.5 / 5 * (2 * 0.2 ** 2 + 4 * 0.1 ** 2 + 6 * 0.3 ** 2),
    EPSILON,
    'hinge quadratic energy'
);

// Local hard XPBD adaptation can align a frame while both endpoint positions
// are fixed. This isolates the rotational denominator and correction sign.
const fixedX0 = { x: 0, y: 0, z: 0 };
const fixedX1 = { x: 2, y: 0, z: 0 };
const adaptedOrientation = { x: 0, y: 0, z: 0, w: 1 };
const adaptationLambda = { x: 0, y: 0, z: 0 };
let adaptationResidual = Infinity;
for (let iteration = 0; iteration < 6; iteration++) {
    adaptationResidual = solveAdaptationXPBD({
        x0: fixedX0,
        x1: fixedX1,
        orientation: adaptedOrientation,
        restLength: 2,
        inverseMass0: 0,
        inverseMass1: 0,
        inverseInertia: { x: 1, y: 2, z: 0.5 },
        compliance: 0,
        lambda: adaptationLambda
    }).residual;
}
assert.ok(adaptationResidual < 1e-9, `adaptation residual ${adaptationResidual}`);
assertVectorNear(
    materialFrameDirectors(adaptedOrientation).d3,
    { x: 1, y: 0, z: 0 },
    EPSILON,
    'XPBD-adapted material tangent'
);

// A local bend/twist solve recovers a finite, coupled manufactured rotation.
const solvedQ0 = { x: 0, y: 0, z: 0, w: 1 };
const solvedQ1 = { x: 0, y: 0, z: 0, w: 1 };
const solvedRestRotation = { x: 0.35, y: -0.25, z: 0.2 };
const bendLambda = { x: 0, y: 0, z: 0 };
let bendResidual = Infinity;
for (let iteration = 0; iteration < 6; iteration++) {
    bendResidual = solveBendTwistXPBD({
        orientation0: solvedQ0,
        orientation1: solvedQ1,
        restRotation: solvedRestRotation,
        inverseInertia0: 0,
        inverseInertia1: { x: 1, y: 0.8, z: 1.2 },
        compliance: 0,
        lambda: bendLambda
    }).residual;
}
assert.ok(bendResidual < 1e-9, `bend/twist residual ${bendResidual}`);
assertVectorNear(
    quaternionLog(solvedQ1),
    solvedRestRotation,
    2e-9,
    'XPBD manufactured relative rotation'
);

function createArraySweepFixture() {
    const count = 7;
    const segmentCount = count - 1;
    const body = {
        x: new Float32Array(count),
        y: new Float32Array(count),
        z: new Float32Array(count),
        orientationX: new Float64Array(segmentCount),
        orientationY: new Float64Array(segmentCount),
        orientationZ: new Float64Array(segmentCount),
        orientationW: new Float64Array(segmentCount),
        inverseMass: new Float32Array(count),
        inverseInertia1: new Float64Array(segmentCount),
        inverseInertia2: new Float64Array(segmentCount),
        inverseInertia3: new Float64Array(segmentCount),
        restLength: new Float32Array(segmentCount),
        adaptationLambdaX: new Float64Array(segmentCount),
        adaptationLambdaY: new Float64Array(segmentCount),
        adaptationLambdaZ: new Float64Array(segmentCount),
        restRotation1: new Float64Array(count),
        restRotation2: new Float64Array(count),
        restRotation3: new Float64Array(count),
        kirchhoffBendCompliance1: new Float64Array(count),
        kirchhoffBendCompliance2: new Float64Array(count),
        kirchhoffTwistCompliance: new Float64Array(count),
        bendTwistLambda1: new Float64Array(count),
        bendTwistLambda2: new Float64Array(count),
        bendTwistLambda3: new Float64Array(count),
        adaptationCompliance: 3e-8,
        kirchhoffScratch: {}
    };
    for (let node = 0; node < count; node++) {
        body.x[node] = node * 1.17 + Math.sin(node * 0.7) * 0.08;
        body.y[node] = Math.cos(node * 0.43) * 0.12;
        body.z[node] = Math.sin(node * 0.31) * 0.09;
        body.inverseMass[node] = node === 0 ? 0 : 0.6 + node * 0.07;
        body.restRotation1[node] = node < 3 ? 0 : 0.013 * node;
        body.restRotation2[node] = node < 3 ? 0 : -0.009 * node;
        body.restRotation3[node] = node < 3 ? 0 : 0.006 * node;
        body.kirchhoffBendCompliance1[node] = 2e-5 + node * 1e-7;
        body.kirchhoffBendCompliance2[node] = 3e-5 + node * 2e-7;
        body.kirchhoffTwistCompliance[node] = 4e-5 + node * 3e-7;
        body.bendTwistLambda1[node] = (node - 2) * 1e-4;
        body.bendTwistLambda2[node] = (3 - node) * 8e-5;
        body.bendTwistLambda3[node] = (node + 1) * -6e-5;
    }
    for (let segment = 0; segment < segmentCount; segment++) {
        const orientation = quaternionExp({
            x: 0.035 * segment,
            y: -0.021 * segment,
            z: 0.017 * segment
        });
        body.orientationX[segment] = orientation.x;
        body.orientationY[segment] = orientation.y;
        body.orientationZ[segment] = orientation.z;
        body.orientationW[segment] = orientation.w;
        body.inverseInertia1[segment] = 0.7 + segment * 0.03;
        body.inverseInertia2[segment] = 0.9 + segment * 0.02;
        body.inverseInertia3[segment] = 1.1 + segment * 0.01;
        body.restLength[segment] = 1.15 + segment * 0.005;
        body.adaptationLambdaX[segment] = segment * 7e-5;
        body.adaptationLambdaY[segment] = -segment * 5e-5;
        body.adaptationLambdaZ[segment] = segment * 3e-5;
    }
    return body;
}

function solveReferenceAdaptationSweep(body, start, end, reverse, dt) {
    const scratch = {};
    for (
        let segment = reverse ? end - 1 : start;
        reverse ? segment >= start : segment < end;
        segment += reverse ? -1 : 1
    ) {
        const x0 = {
            x: body.x[segment],
            y: body.y[segment],
            z: body.z[segment]
        };
        const x1 = {
            x: body.x[segment + 1],
            y: body.y[segment + 1],
            z: body.z[segment + 1]
        };
        const orientation = {
            x: body.orientationX[segment],
            y: body.orientationY[segment],
            z: body.orientationZ[segment],
            w: body.orientationW[segment]
        };
        const lambda = {
            x: body.adaptationLambdaX[segment],
            y: body.adaptationLambdaY[segment],
            z: body.adaptationLambdaZ[segment]
        };
        solveAdaptationXPBD({
            x0,
            x1,
            orientation,
            restLength: body.restLength[segment],
            inverseMass0: body.inverseMass[segment],
            inverseMass1: body.inverseMass[segment + 1],
            inverseInertia: {
                x: body.inverseInertia1[segment],
                y: body.inverseInertia2[segment],
                z: body.inverseInertia3[segment]
            },
            compliance: body.adaptationCompliance,
            dt,
            lambda,
            scratch,
            returnState: false,
            normalizedOrientations: true,
            objectVectors: true
        });
        body.x[segment] = x0.x;
        body.y[segment] = x0.y;
        body.z[segment] = x0.z;
        body.x[segment + 1] = x1.x;
        body.y[segment + 1] = x1.y;
        body.z[segment + 1] = x1.z;
        body.orientationX[segment] = orientation.x;
        body.orientationY[segment] = orientation.y;
        body.orientationZ[segment] = orientation.z;
        body.orientationW[segment] = orientation.w;
        body.adaptationLambdaX[segment] = lambda.x;
        body.adaptationLambdaY[segment] = lambda.y;
        body.adaptationLambdaZ[segment] = lambda.z;
    }
}

function solveReferenceBendSweep(body, start, end, reverse, dt) {
    const scratch = {};
    for (let offset = 0; offset < end - start; offset++) {
        const joint = reverse ? end - 1 - offset : start + offset;
        const previous = joint - 1;
        const next = joint;
        const orientation0 = {
            x: body.orientationX[previous],
            y: body.orientationY[previous],
            z: body.orientationZ[previous],
            w: body.orientationW[previous]
        };
        const orientation1 = {
            x: body.orientationX[next],
            y: body.orientationY[next],
            z: body.orientationZ[next],
            w: body.orientationW[next]
        };
        const lambda = {
            x: body.bendTwistLambda1[joint],
            y: body.bendTwistLambda2[joint],
            z: body.bendTwistLambda3[joint]
        };
        solveBendTwistXPBD({
            orientation0,
            orientation1,
            restRotation: {
                x: body.restRotation1[joint],
                y: body.restRotation2[joint],
                z: body.restRotation3[joint]
            },
            inverseInertia0: {
                x: body.inverseInertia1[previous],
                y: body.inverseInertia2[previous],
                z: body.inverseInertia3[previous]
            },
            inverseInertia1: {
                x: body.inverseInertia1[next],
                y: body.inverseInertia2[next],
                z: body.inverseInertia3[next]
            },
            compliance: {
                x: body.kirchhoffBendCompliance1[joint],
                y: body.kirchhoffBendCompliance2[joint],
                z: body.kirchhoffTwistCompliance[joint]
            },
            dt,
            lambda,
            scratch,
            returnState: false,
            normalizedOrientations: true,
            objectVectors: true
        });
        body.orientationX[previous] = orientation0.x;
        body.orientationY[previous] = orientation0.y;
        body.orientationZ[previous] = orientation0.z;
        body.orientationW[previous] = orientation0.w;
        body.orientationX[next] = orientation1.x;
        body.orientationY[next] = orientation1.y;
        body.orientationZ[next] = orientation1.z;
        body.orientationW[next] = orientation1.w;
        body.bendTwistLambda1[joint] = lambda.x;
        body.bendTwistLambda2[joint] = lambda.y;
        body.bendTwistLambda3[joint] = lambda.z;
    }
}

function assertNumericArrayNear(actual, expected, tolerance, message) {
    assert.equal(actual.length, expected.length, `${message} length`);
    for (let index = 0; index < actual.length; index++) {
        assertNear(actual[index], expected[index], tolerance, `${message}[${index}]`);
    }
}

const referenceSweepBody = createArraySweepFixture();
const fastSweepBody = createArraySweepFixture();
for (const reverse of [false, true]) {
    solveReferenceAdaptationSweep(referenceSweepBody, 0, 6, reverse, 1 / 120);
    solveAdaptationXPBDArraySweep(fastSweepBody, 0, 6, reverse, 1 / 120);
    solveReferenceBendSweep(referenceSweepBody, 1, 6, reverse, 1 / 120);
    solveBendTwistXPBDArraySweep(fastSweepBody, 1, 6, reverse, 1 / 120);
}
for (const key of ['x', 'y', 'z']) {
    assertNumericArrayNear(
        fastSweepBody[key],
        referenceSweepBody[key],
        2e-7,
        `array sweep ${key}`
    );
}
for (const key of [
    'orientationX',
    'orientationY',
    'orientationZ',
    'orientationW',
    'adaptationLambdaX',
    'adaptationLambdaY',
    'adaptationLambdaZ',
    'bendTwistLambda1',
    'bendTwistLambda2',
    'bendTwistLambda3'
]) {
    assertNumericArrayNear(
        fastSweepBody[key],
        referenceSweepBody[key],
        3e-15,
        `array sweep ${key}`
    );
}

console.log('Discrete Kirchhoff rod tests passed');
