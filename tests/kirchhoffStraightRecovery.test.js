import assert from 'node:assert/strict';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { GUIDEWIRE_TYPE_GLIDEWIRE } from '../src/physics/guidewireMaterialProfile.js';

function maximumCenterlineBend(body, endJoint = body.activeEnd) {
    let maximum = 0;
    for (let joint = body.activeStart + 1; joint < endJoint; joint++) {
        const ax = body.x[joint] - body.x[joint - 1];
        const ay = body.y[joint] - body.y[joint - 1];
        const az = body.z[joint] - body.z[joint - 1];
        const bx = body.x[joint + 1] - body.x[joint];
        const by = body.y[joint + 1] - body.y[joint];
        const bz = body.z[joint + 1] - body.z[joint];
        maximum = Math.max(maximum, Math.acos(Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) /
            (Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
        ))));
    }
    return maximum;
}

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

// Contact damping must not be implemented by globally discarding the
// Kirchhoff rod's projection velocity. A manufactured-straight Glidewire must
// keep its elastic recovery motion until the residual S-bend relaxes.
{
    const segmentLength = 4;
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod('idle-straight-recovery', 31, segmentLength, {
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        linearDamping: 0.98,
        angularDamping: 0.96,
        // Mimic a wire whose proximal span is locally damped by catheter
        // coupling. The free shaft below is explicitly allowed to retain its
        // elastic recovery velocity.
        projectionVelocityRetention: 0.005,
        sleepVelocity: 1,
        sleepAngularVelocity: 0.015,
        sleepFrames: 10,
        postStabilizationPasses: 0
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
        // Exercise a shaft span; the clinical Glidewire's distal 48 mm is
        // intentionally much softer than its body.
        tipCoordinate: 200
    });
    body.setPinned(0, true);
    body.setProximalOrientationControl(
        body.orientationX[0],
        body.orientationY[0],
        body.orientationZ[0],
        body.orientationW[0]
    );
    body.distalProjectionVelocityRetention = 1;
    // The material span inside the catheter may be locally damped, but the
    // first free node beyond its distal portal must immediately retain the
    // Kirchhoff rod's full elastic recovery velocity.
    body.distalProjectionVelocityRetentionStartNode = body.activeStart + 1;

    const initialBend = maximumCenterlineBend(body);
    const initialTotalBend = totalCenterlineBend(body);
    const recoveryTrace = [];
    for (let step = 0; step < 720; step++) {
        world.stepFixed();
        if (process.env.OET_TRACE_STRAIGHT_RECOVERY && (step + 1) % 120 === 0) {
            recoveryTrace.push({
                seconds: (step + 1) / 120,
                maximumBend: maximumCenterlineBend(body),
                totalBend: totalCenterlineBend(body),
                sleeping: body.sleeping
            });
        }
    }
    const finalBend = maximumCenterlineBend(body);
    const finalTotalBend = totalCenterlineBend(body);

    assert.ok(initialBend > 0.05, `fixture must start curved (${initialBend})`);
    assert.ok(
        finalTotalBend < initialTotalBend * 0.75,
        `idle Glidewire retained its bend (${initialBend} -> ${finalBend} rad; ` +
        `total=${initialTotalBend} -> ${finalTotalBend}; ` +
        `sleeping=${body.sleeping}, counter=${body.sleepCounter})`
    );
    assert.ok(
        finalBend < initialBend * 0.5,
        `tool-coupled Glidewire concentrated rather than released its shaft bend (` +
        `${initialBend} -> ${finalBend} rad)`
    );
    assert.equal(
        body.sleeping,
        false,
        'a visibly strained Kirchhoff rod must not be frozen by idle damping'
    );
    if (recoveryTrace.length) {
        console.log('Glidewire shaft recovery trace', {
            initialMaximumBend: initialBend,
            initialTotalBend,
            samples: recoveryTrace
        });
    }
}

// The distal Glidewire zone must be compliant under load without behaving
// like a plastically deformed hinge after that load disappears. Exercise the
// complete 5 cm taper, including its lowest-EI terminal core.
{
    const segmentLength = 4;
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod('glidewire-distal-recovery', 31, segmentLength, {
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        foldLimitStrength: 1,
        maxBendAngle: 30,
        linearDamping: 0.98,
        angularDamping: 0.96,
        projectionVelocityRetention: 1,
        sleepVelocity: 1,
        sleepAngularVelocity: 0.015,
        sleepFrames: 10,
        postStabilizationPasses: 0
    });

    const curvedLength = 40;
    body.x[0] = 0;
    body.y[0] = 0;
    body.z[0] = 0;
    for (let segment = 0; segment < body.segmentCount; segment++) {
        const distanceFromTip = (body.segmentCount - 1 - segment) * segmentLength;
        const tangentAngle = distanceFromTip < curvedLength
            ? 0.52 * (1 - distanceFromTip / curvedLength)
            : 0;
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
        tipCoordinate: body.materialCoordinate[body.count - 1]
    });
    body.setPinned(0, true);
    body.setProximalOrientationControl(
        body.orientationX[0],
        body.orientationY[0],
        body.orientationZ[0],
        body.orientationW[0]
    );

    const initialTotalBend = totalCenterlineBend(body);
    for (let step = 0; step < 720; step++) world.stepFixed();
    const finalTotalBend = totalCenterlineBend(body);
    assert.ok(
        finalTotalBend < initialTotalBend * 0.72,
        `released Glidewire tip retained its hook (` +
        `${initialTotalBend} -> ${finalTotalBend} rad)`
    );
}

console.log('Kirchhoff straight-recovery tests passed');
