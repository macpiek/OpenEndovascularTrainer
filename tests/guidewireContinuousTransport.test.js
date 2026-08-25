import assert from 'node:assert/strict';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { GUIDEWIRE_TYPE_GLIDEWIRE } from '../src/physics/guidewireMaterialProfile.js';

const FIXED_DT = 1 / 120;
const OPERATOR_SPEED = 44;

function totalBend(body) {
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
            Math.max(1e-9,
                Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
        )));
    }
    return total;
}

function maximumSpeed(body) {
    let maximum = 0;
    for (let node = body.activeStart; node <= body.activeEnd; node++) {
        maximum = Math.max(maximum, Math.hypot(
            body.velocityX[node],
            body.velocityY[node],
            body.velocityZ[node]
        ));
    }
    return maximum;
}

function continuousBoundaryMotion(stiffnessScale, direction) {
    const segmentLength = 4;
    const world = new EndovascularPhysicsWorld({
        fixedDt: FIXED_DT,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod(
        `continuous-${stiffnessScale}-${direction}`,
        31,
        segmentLength,
        {
            ...DEFAULT_TOOL_PROFILES.guidewire,
            rodModel: 'kirchhoff',
            adaptationCompliance: 0,
            foldLimitStrength: 0,
            maxBendAngle: 179,
            linearDamping: 0.98,
            angularDamping: 0.96,
            projectionVelocityRetention: 1,
            sleepFrames: 1000
        }
    );

    body.x[0] = 0;
    body.y[0] = 0;
    body.z[0] = 0;
    for (let segment = 0; segment < body.segmentCount; segment++) {
        const phase = segment / (body.segmentCount - 1);
        const tangentAngle = 0.45 * Math.sin(phase * Math.PI * 2);
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
        tipCoordinate: 200,
        stiffnessScale
    });
    body.setPinned(0, true);
    body.setPinned(1, true);
    body.setProximalOrientationControl(
        body.orientationX[0],
        body.orientationY[0],
        body.orientationZ[0],
        body.orientationW[0]
    );

    const initialBend = totalBend(body);
    let bendBeforeRelease = 0;
    let bendAfterRelease = 0;
    let speedBeforeRelease = 0;
    let speedAfterRelease = 0;
    let maximumDrivenSpeed = 0;
    let maximumLengthError = 0;
    for (let step = 0; step < 360; step++) {
        if (step < 180) {
            const transportDelta = direction * OPERATOR_SPEED * FIXED_DT;
            for (let node = 0; node <= 1; node++) {
                body.setNodePosition(
                    node,
                    body.x[node] + transportDelta,
                    body.y[node],
                    body.z[node],
                    false
                );
            }
        }
        world.stepFixed();
        const bodyStats = world.getStats().bodies[0];
        maximumLengthError = Math.max(
            maximumLengthError,
            bodyStats.maxLengthError
        );
        const speed = maximumSpeed(body);
        if (step < 180) maximumDrivenSpeed = Math.max(maximumDrivenSpeed, speed);
        if (step === 179) {
            bendBeforeRelease = totalBend(body);
            speedBeforeRelease = speed;
        } else if (step === 180) {
            bendAfterRelease = totalBend(body);
            speedAfterRelease = speed;
        }
    }

    return {
        stiffnessScale,
        direction,
        initialBend,
        bendBeforeRelease,
        bendAfterRelease,
        finalBend: totalBend(body),
        speedBeforeRelease,
        speedAfterRelease,
        maximumDrivenSpeed,
        maximumLengthError,
        finite: world.getStats().bodies[0].finite
    };
}

const results = [];
for (const stiffnessScale of [0.5, 1, 2]) {
    for (const direction of [-1, 1]) {
        const result = continuousBoundaryMotion(stiffnessScale, direction);
        results.push(result);
        assert.equal(result.finite, true);
        assert.ok(
            result.maximumLengthError <= 0.005,
            `continuous transport stretched the ${stiffnessScale}x wire (` +
            `${result.maximumLengthError})`
        );
        assert.ok(
            result.maximumDrivenSpeed > OPERATOR_SPEED + 5,
            `elastic motion was clipped into the ${OPERATOR_SPEED} mm/s ` +
            `transport budget (${result.maximumDrivenSpeed} mm/s)`
        );
        assert.ok(
            result.maximumDrivenSpeed < 80,
            `continuous transport generated an impulse (` +
            `${result.maximumDrivenSpeed} mm/s)`
        );
        assert.ok(
            Math.abs(result.bendAfterRelease - result.bendBeforeRelease) < 0.03,
            `releasing transport changed shape discontinuously (` +
            `${result.bendBeforeRelease} -> ${result.bendAfterRelease} rad)`
        );
        assert.ok(
            result.speedAfterRelease <= result.speedBeforeRelease + 1,
            `releasing transport created a velocity impulse (` +
            `${result.speedBeforeRelease} -> ${result.speedAfterRelease} mm/s)`
        );
    }
}

const withdrawals = results.filter(result => result.direction < 0);
for (const result of withdrawals) {
    assert.ok(
        result.bendBeforeRelease < result.initialBend * 0.1,
        `the ${result.stiffnessScale}x wire did not straighten while continuously ` +
        `withdrawing (${result.initialBend} -> ${result.bendBeforeRelease} rad)`
    );
}

const softAdvance = results.find(result =>
    result.direction > 0 && result.stiffnessScale === 0.5
);
const stiffAdvance = results.find(result =>
    result.direction > 0 && result.stiffnessScale === 2
);
assert.ok(
    stiffAdvance.bendBeforeRelease < softAdvance.bendBeforeRelease,
    'raising guidewire stiffness must reduce continuously driven curvature'
);
assert.ok(
    stiffAdvance.finalBend < softAdvance.finalBend * 0.75,
    'raising guidewire stiffness must accelerate post-drive equilibration'
);

console.log('Guidewire continuous-transport tests passed');
