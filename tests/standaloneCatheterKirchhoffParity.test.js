import assert from 'node:assert/strict';
import test from 'node:test';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import {
    MAX_GUIDEWIRE_RELAXATION_RATE,
    guidewireRelaxationPasses
} from '../src/physics/guidewireRelaxationRate.js';

const DT = 1 / 120;

function createStandaloneCatheter(type = 'berenstein') {
    const guidewireLength = 240;
    const guidewireSpacing = 2;
    const sheathLength = 20;
    const wire = new ElasticRod(
        guidewireLength / guidewireSpacing + 1,
        guidewireSpacing
    );
    for (let index = 0; index < wire.nodes.length; index++) {
        const node = wire.nodes[index];
        node.x = index * guidewireSpacing - guidewireLength - sheathLength;
        node.y = 0;
        node.z = 0;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
    }
    const catheter = new PigtailCatheter({
        wire,
        segmentLength: guidewireSpacing,
        guidewireLength,
        tailProgressRef: () => 0,
        vessel: {
            sheath: {
                start: { x: -sheathLength, y: 0, z: 0 },
                end: { x: 0, y: 0, z: 0 }
            },
            segments: []
        },
        maxLength: 180
    });
    catheter.setType(type);
    catheter.setExternalCollisionSolver(true);
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('standalone-catheter', 160, 4, {
        ...DEFAULT_TOOL_PROFILES.catheter,
        rodModel: 'kirchhoff'
    });
    return { body, catheter, world };
}

function runtimeState(body) {
    return {
        postStabilizationPasses: body.postStabilizationPasses,
        finalStructuralClosurePasses: body.finalStructuralClosurePasses,
        intrinsicClosureCorrectionScale: body.intrinsicClosureCorrectionScale,
        postStabilizeBending: body.postStabilizeBending,
        restTurnPolishMaxAngle: body.restTurnPolishMaxAngle,
        projectionVelocityRetention: body.projectionVelocityRetention,
        distalProjectionVelocityRetention: body.distalProjectionVelocityRetention,
        distalProjectionVelocityRetentionStartNode:
            body.distalProjectionVelocityRetentionStartNode,
        maxFrameDisplacement: body.maxFrameDisplacement,
        wallProjectionVelocityRetention: body.wallProjectionVelocityRetention,
        sweptContactPreserveTangentialMotion:
            body.sweptContactPreserveTangentialMotion,
        wallFrictionUsesCurrentLoad: body.wallFrictionUsesCurrentLoad,
        wallFrictionUsesSmoothedLoad: body.wallFrictionUsesSmoothedLoad
    };
}

test('standalone catheter keeps one guidewire-equivalent Kirchhoff runtime during feed and rest', () => {
    const { body, catheter } = createStandaloneCatheter();
    try {
        for (let step = 0; step < 100; step++) {
            catheter.advance(1, DT, 0);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
        }
        const feeding = runtimeState(body);

        catheter.advance(0, DT, 0);
        catheter.stepPhysics(DT, { collisions: false });
        catheter.syncXpbdBody(body);
        const resting = runtimeState(body);

        assert.deepEqual(resting, feeding);
        assert.deepEqual(resting, {
            postStabilizationPasses: 0,
            finalStructuralClosurePasses: 8,
            intrinsicClosureCorrectionScale: 0,
            postStabilizeBending: false,
            restTurnPolishMaxAngle: 0,
            projectionVelocityRetention: 1,
            distalProjectionVelocityRetention: 1,
            distalProjectionVelocityRetentionStartNode: Infinity,
            maxFrameDisplacement: Infinity,
            wallProjectionVelocityRetention:
                DEFAULT_TOOL_PROFILES.guidewire.wallProjectionVelocityRetention,
            sweptContactPreserveTangentialMotion:
                DEFAULT_TOOL_PROFILES.guidewire.sweptContactPreserveTangentialMotion,
            wallFrictionUsesCurrentLoad: false,
            wallFrictionUsesSmoothedLoad: false
        });
    } finally {
        catheter.dispose();
    }
});

test('guidewire-supported catheter does not reconstruct idle projections as momentum', () => {
    const { body, catheter } = createStandaloneCatheter('pigtail');
    try {
        const guidewireInserted = 120;
        for (let step = 0; step < 100; step++) {
            catheter.advance(1, DT, guidewireInserted);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
        }
        assert.ok(catheter.progress > 18);
        assert.equal(body.projectionVelocityRetention, 1,
            'active feed must retain physical transport velocity');
        assert.equal(body.wallProjectionVelocityRetention, 0,
            'adding lumen support must not turn vessel projection into rebound');

        catheter.advance(0, DT, guidewireInserted);
        catheter.stepPhysics(DT, { collisions: false });
        catheter.syncXpbdBody(body);
        assert.equal(body.projectionVelocityRetention, 0.005,
            'idle coupled equilibrium projections must be quasi-static');
        assert.equal(body.wallProjectionVelocityRetention, 0,
            'idle coupled wall contact must remain zero-restitution');
    } finally {
        catheter.dispose();
    }
});

test('standalone catheter XPBD pose remains authoritative while it is advancing', () => {
    const { body, catheter } = createStandaloneCatheter('pigtail');
    try {
        for (let step = 0; step < 130; step++) {
            catheter.advance(1, DT, 0);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
        }
        assert.ok(catheter.freeNodes.length > 4);
        const tracked = catheter.freeNodes[Math.floor(catheter.freeNodes.length / 2)];
        const bodyIndex = tracked._xpbdIndex;
        assert.ok(bodyIndex > body.activeStart && bodyIndex <= body.activeEnd);
        body.y[bodyIndex] += 3;
        const expectedY = body.y[bodyIndex];

        catheter.advance(1, DT, 0);
        catheter.stepPhysics(DT, { collisions: false });

        const synchronized = catheter.freeNodes.find(
            node => node._xpbdIndex === bodyIndex
        );
        assert.ok(synchronized, 'the tracked catheter material node should remain active');
        assert.ok(
            Math.abs(synchronized.pos.y - expectedY) < 1e-6,
            `feed replaced the physical XPBD pose (${synchronized.pos.y} instead of ${expectedY})`
        );
    } finally {
        catheter.dispose();
    }
});

test('standalone catheter advances, withdraws and relaxes through one continuous rod solve', () => {
    const { body, catheter, world } = createStandaloneCatheter('berenstein');
    try {
        world.addSheath({
            start: { x: -20, y: 0, z: 0 },
            end: { x: 0, y: 0, z: 0 },
            innerRadius: 2,
            proximalExtension: 90,
            bodies: [body]
        });
        let previousTip = null;
        let maximumTipStep = 0;
        const sampleTipStep = () => {
            const tip = [
                body.x[body.activeEnd],
                body.y[body.activeEnd],
                body.z[body.activeEnd]
            ];
            if (previousTip && catheter.progress >= 40) {
                maximumTipStep = Math.max(maximumTipStep, Math.hypot(
                    tip[0] - previousTip[0],
                    tip[1] - previousTip[1],
                    tip[2] - previousTip[2]
                ));
            }
            previousTip = tip;
        };
        const step = command => {
            catheter.advance(command, DT, 0);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
            sampleTipStep();
        };

        for (let frame = 0; frame < 280; frame++) step(1);
        for (let frame = 0; frame < 90; frame++) step(-1);
        for (let frame = 0; frame < 180; frame++) step(0);

        let maximumLengthError = 0;
        for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
            maximumLengthError = Math.max(maximumLengthError, Math.abs(
                Math.hypot(
                    body.x[segment + 1] - body.x[segment],
                    body.y[segment + 1] - body.y[segment],
                    body.z[segment + 1] - body.z[segment]
                ) - body.restLength[segment]
            ));
        }
        assert.ok(Number.isFinite(maximumTipStep));
        assert.ok(maximumTipStep < 1.25,
            `standalone catheter tip jumped ${maximumTipStep} mm in one fixed step`);
        assert.ok(maximumLengthError < 0.05,
            `standalone catheter length error reached ${maximumLengthError} mm`);
        assert.equal(body.lastPostStabilizationPasses, 0);
    } finally {
        catheter.dispose();
    }
});

test('standalone catheter supports its own always-on relaxation pass rate', () => {
    const { body, catheter, world } = createStandaloneCatheter('pigtail');
    try {
        const expectedPasses = guidewireRelaxationPasses(
            MAX_GUIDEWIRE_RELAXATION_RATE
        );
        body.relaxationPasses = expectedPasses;

        for (const command of [1, -1, 0]) {
            catheter.advance(command, DT, 0);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
            assert.equal(
                body.lastRelaxationPasses,
                expectedPasses,
                `catheter relaxation should remain active for command ${command}`
            );
        }
    } finally {
        catheter.dispose();
    }
});
