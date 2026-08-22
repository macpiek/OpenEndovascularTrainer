import assert from 'node:assert/strict';
import test from 'node:test';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
    PigtailCatheter
} from '../src/pigtailCatheter.js';
import { INTRODUCER_SHEATH_INNER_RADIUS_MM } from '../src/toolDimensions.js';

const DT = 1 / 120;

function maximumBendDegrees(body) {
    let maximum = 0;
    let node = -1;
    for (let index = body.activeStart + 1; index < body.activeEnd; index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const az = body.z[index] - body.z[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const bz = body.z[index + 1] - body.z[index];
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator <= 1e-9) continue;
        const cosine = Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) / denominator
        ));
        const angle = Math.acos(cosine) * 180 / Math.PI;
        if (angle <= maximum) continue;
        maximum = angle;
        node = index;
    }
    return { maximum, node };
}

test('the loading hub radially supports a Berenstein while guidewire-only setup runs', () => {
    const guidewireLength = 1000;
    const guidewireSpacing = 5;
    const wire = new ElasticRod(
        guidewireLength / guidewireSpacing + 1,
        guidewireSpacing
    );
    for (let index = 0; index < wire.nodes.length; index++) {
        const node = wire.nodes[index];
        node.x = index * guidewireSpacing - guidewireLength - 100;
        node.y = 0;
        node.z = 0;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
    }

    const vessel = {
        sheath: {
            start: { x: -100, y: 0, z: 0 },
            end: { x: 0, y: 0, z: 0 }
        },
        segments: []
    };
    const catheter = new PigtailCatheter({
        wire,
        segmentLength: guidewireSpacing,
        guidewireLength,
        tailProgressRef: () => 0,
        vessel
    });
    catheter.setType('berenstein');
    catheter.setExternalCollisionSolver(true);

    const world = new EndovascularPhysicsWorld({
        fixedDt: DT,
        iterations: 6
    });
    const body = world.createRod('loading-hub-berenstein', 320, 4, {
        ...DEFAULT_TOOL_PROFILES.catheter,
        rodModel: 'kirchhoff'
    });
    catheter.syncXpbdBody(body);
    world.addSheath({
        start: vessel.sheath.start,
        end: vessel.sheath.end,
        innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM,
        proximalExtension: CATHETER_PROXIMAL_LOADING_SUPPORT_LENGTH_MM,
        bodies: [body]
    });

    try {
        // Keep the complete manufactured 45-degree rest bend. The support,
        // rather than a command-dependent kappa_0 scale, straightens it.
        const initialRestTurn = Array.from(body.restRotation1).reduce(
            (sum, value) => sum + value,
            0
        );
        assert.ok(
            Math.abs(initialRestTurn - Math.PI / 4) < 1e-9,
            `Berenstein constitutive turn was modified (${initialRestTurn})`
        );

        let peak = { maximum: 0, node: -1 };
        for (let step = 0; step < 180; step++) {
            catheter.advance(0, DT, 0);
            catheter.stepPhysics(DT, { collisions: false });
            catheter.syncXpbdBody(body);
            world.stepFixed();
            const bend = maximumBendDegrees(body);
            if (bend.maximum > peak.maximum) peak = bend;
        }

        assert.equal(catheter.physicsLumenStartNode, 15);
        assert.ok(
            peak.maximum < 5,
            `proximal Berenstein material kinked at node ${peak.node} (${peak.maximum} degrees)`
        );
    } finally {
        catheter.dispose();
    }
});

test('proximal sheath extension supplies radial contact without axial tracking', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, iterations: 4 });
    const body = world.createRod('axially-free-loading-hub', 4, 1, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.1,
        linearDamping: 1,
        stretchCompliance: 0
    });
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, -8 + index, 1.2, 0);
        body.velocityX[index] = 12;
    }
    world.addSheath({
        start: { x: 0, y: 0, z: 0 },
        end: { x: 10, y: 0, z: 0 },
        innerRadius: 0.9,
        proximalExtension: 10,
        bodies: [body]
    });

    const initialX = body.x[0];
    for (let step = 0; step < 8; step++) world.stepFixed();
    assert.ok(body.x[0] > initialX + 0.5, 'loading support applied an axial position track');
    for (let index = 0; index < body.count; index++) {
        assert.ok(
            Math.hypot(body.y[index], body.z[index]) <= 0.801,
            'loading support failed to apply the radial lumen clearance'
        );
    }
});
