import assert from 'node:assert/strict';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { GUIDEWIRE_TYPE_GLIDEWIRE } from '../src/physics/guidewireMaterialProfile.js';
import {
    DEFAULT_GUIDEWIRE_RELAXATION_RATE,
    MAX_GUIDEWIRE_RELAXATION_RATE,
    MIN_GUIDEWIRE_RELAXATION_RATE,
    clampGuidewireRelaxationRate,
    guidewireRelaxationPasses
} from '../src/physics/guidewireRelaxationRate.js';

assert.equal(clampGuidewireRelaxationRate(Number.NaN), DEFAULT_GUIDEWIRE_RELAXATION_RATE);
assert.equal(clampGuidewireRelaxationRate(-10), MIN_GUIDEWIRE_RELAXATION_RATE);
assert.equal(clampGuidewireRelaxationRate(100), MAX_GUIDEWIRE_RELAXATION_RATE);
assert.equal(
    guidewireRelaxationPasses(DEFAULT_GUIDEWIRE_RELAXATION_RATE),
    0,
    'the default relaxation rate must preserve the existing solver path exactly'
);
assert.ok(
    guidewireRelaxationPasses(MAX_GUIDEWIRE_RELAXATION_RATE) >
        guidewireRelaxationPasses(2),
    'higher relaxation rates must schedule more constitutive convergence passes'
);

function totalCenterlineBend(body) {
    let total = 0;
    for (let joint = body.activeStart + 1; joint < body.activeEnd; joint++) {
        const ax = body.x[joint] - body.x[joint - 1];
        const ay = body.y[joint] - body.y[joint - 1];
        const az = body.z[joint] - body.z[joint - 1];
        const bx = body.x[joint + 1] - body.x[joint];
        const by = body.y[joint + 1] - body.y[joint];
        const bz = body.z[joint + 1] - body.z[joint];
        total += Math.acos(Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) /
                (Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
        )));
    }
    return total;
}

function recoveredBend(relaxationRate) {
    const segmentLength = 4;
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod('relaxation-rate-fixture', 31, segmentLength, {
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        linearDamping: 0.98,
        angularDamping: 0.96,
        projectionVelocityRetention: 1,
        sleepVelocity: 0,
        sleepAngularVelocity: 0,
        sleepFrames: 10000,
        relaxationPasses: guidewireRelaxationPasses(relaxationRate)
    });

    body.x[0] = 0;
    body.y[0] = 0;
    body.z[0] = 0;
    for (let segment = 0; segment < body.segmentCount; segment++) {
        const phase = segment / (body.segmentCount - 1);
        const tangentAngle = 0.55 * Math.sin(phase * Math.PI * 2);
        body.x[segment + 1] = body.x[segment] +
            Math.cos(tangentAngle) * segmentLength;
        body.y[segment + 1] = body.y[segment] +
            Math.sin(tangentAngle) * segmentLength;
        body.z[segment + 1] = 0;
    }
    body.copyCurrentToPrevious();
    body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    applyKirchhoffMaterialProfile(body, GUIDEWIRE_TYPE_GLIDEWIRE, {
        materialCoordinates: body.materialCoordinate,
        tipCoordinate: 200
    });
    body.setPinned(0, true);
    body.setProximalOrientationControl(
        body.orientationX[0],
        body.orientationY[0],
        body.orientationZ[0],
        body.orientationW[0]
    );

    for (let step = 0; step < 60; step++) world.stepFixed();
    return {
        bend: totalCenterlineBend(body),
        passes: body.lastRelaxationPasses
    };
}

const standardRecovery = recoveredBend(DEFAULT_GUIDEWIRE_RELAXATION_RATE);
const fastRecovery = recoveredBend(MAX_GUIDEWIRE_RELAXATION_RATE);
assert.equal(standardRecovery.passes, 0);
assert.equal(
    fastRecovery.passes,
    guidewireRelaxationPasses(MAX_GUIDEWIRE_RELAXATION_RATE)
);
assert.ok(
    fastRecovery.bend < standardRecovery.bend,
    `higher relaxation rate must reduce the residual bend sooner (` +
        `${standardRecovery.bend} -> ${fastRecovery.bend})`
);

console.log('Guidewire relaxation-rate tests passed');
