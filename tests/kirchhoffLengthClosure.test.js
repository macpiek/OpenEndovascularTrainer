import assert from 'node:assert/strict';
import {
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';

function maximumSegmentError(body) {
    let maximum = 0;
    for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
        const length = Math.hypot(
            body.x[segment + 1] - body.x[segment],
            body.y[segment + 1] - body.y[segment],
            body.z[segment + 1] - body.z[segment]
        );
        maximum = Math.max(
            maximum,
            Math.abs(length - body.restLength[segment]) /
                body.restLength[segment]
        );
    }
    return maximum;
}

function profile(overrides = {}) {
    return {
        rodModel: 'kirchhoff',
        radius: 0.45,
        innerRadius: 0.75,
        mass: 1,
        adaptationCompliance: 0,
        kirchhoffBendCompliance: 2e-5,
        kirchhoffTwistCompliance: 2e-5,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        linearDamping: 0.98,
        angularDamping: 0.96,
        projectionVelocityRetention: 0.005,
        sleepFrames: 1_000_000,
        postStabilizationPasses: 0,
        ...overrides
    };
}

// The introducer is solved again after ordinary per-body stabilization. A
// long active wire with an oblique free span at its outlet therefore exposes
// whether the final solver closure includes Kirchhoff adaptation.
{
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const segmentLength = 5;
    const wire = world.createRod(
        'long-kirchhoff-guidewire-at-sheath-outlet',
        121,
        segmentLength,
        profile({ radius: 0.445 })
    );
    for (let index = 0; index < wire.count; index++) {
        wire.setNodePosition(index, index * segmentLength, 0, 0);
    }
    wire.captureRestConfiguration();
    wire.setPinned(0, true);
    wire.setSheathMaterialEndNode(12);
    for (let index = 1; index < wire.count; index++) {
        wire.setNodePosition(index, index * segmentLength, 3, 0);
    }
    wire.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    wire.copyCurrentToPrevious();
    world.addSheath({
        start: { x: 0, y: 0, z: 0 },
        end: { x: 60, y: 0, z: 0 },
        innerRadius: 1.2,
        bodies: [wire]
    });
    world.stepFixed();
    const maximumError = maximumSegmentError(wire);
    assert.ok(Number.isFinite(maximumError));
    assert.ok(
        maximumError < 0.002,
        `long guidewire outlet segment error ${(maximumError * 100).toFixed(4)}%`
    );
}

// A long guidewire is partially constrained by a catheter lumen. Radial
// contact at many differently loaded segments must not remain as axial stretch
// after the last contact pass of the fixed step.
{
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const segmentLength = 5;
    const wire = world.createRod(
        'long-kirchhoff-guidewire',
        121,
        segmentLength,
        profile({ radius: 0.445 })
    );
    const catheter = world.createRod(
        'contacting-kirchhoff-catheter',
        101,
        4,
        profile({
            radius: 0.85,
            innerRadius: 0.65,
            mass: 1000,
            kirchhoffBendCompliance: 1e-9,
            kirchhoffTwistCompliance: 1e-9
        })
    );
    for (let index = 0; index < catheter.count; index++) {
        catheter.setNodePosition(index, index * 4, 0, 0);
        catheter.setPinned(index, true);
    }
    catheter.captureRestConfiguration();
    for (let index = 0; index < wire.count; index++) {
        wire.setNodePosition(index, index * segmentLength, 0, 0);
    }
    wire.captureRestConfiguration();
    wire.setPinned(0, true);
    // Manufacture a smooth but lumen-violating initial pose without changing
    // the material rest lengths. The free distal span also bends outside the
    // open catheter tip, as in a deployed guidewire.
    for (let index = 1; index < wire.count; index++) {
        const x = index * segmentLength;
        const contained = x <= 400;
        wire.setNodePosition(
            index,
            x,
            contained ? 0.42 * Math.sin(index * 0.38) : (x - 400) * 0.012,
            contained ? 0.34 * Math.cos(index * 0.31) : 0
        );
    }
    wire.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    wire.copyCurrentToPrevious();
    const containment = world.addContainment(wire, catheter, {
        model: 'kirchhoff',
        innerRadius: catheter.innerRadius,
        friction: 0.04,
        openDistal: true,
        outerStartNode: 0,
        startNode: 0,
        endNode: 81,
        enabled: true
    });
    let maximumObservedError = 0;
    for (let step = 0; step < 90; step++) {
        wire.wake();
        catheter.wake();
        world.stepFixed();
        maximumObservedError = Math.max(
            maximumObservedError,
            maximumSegmentError(wire)
        );
    }
    const maximumError = maximumSegmentError(wire);
    assert.ok(containment.manifold.size > 0);
    assert.ok(Number.isFinite(maximumError));
    assert.ok(
        maximumObservedError < 0.002,
        `contacted guidewire peak segment error ` +
            `${(maximumObservedError * 100).toFixed(4)}%`
    );
    assert.ok(
        maximumError < 0.002,
        `long guidewire segment error ${(maximumError * 100).toFixed(4)}%`
    );
}

console.log('Kirchhoff length-closure tests passed');
