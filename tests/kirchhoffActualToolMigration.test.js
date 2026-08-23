import assert from 'node:assert/strict';
import test from 'node:test';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { KIRCHHOFF_PROFILE_EXPECTED_TURNS } from '../src/physics/kirchhoffMaterialProfile.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';

const DT = 1 / 120;

function createDeployedCatheter(type, guidewireInserted = 0) {
    const guidewireLength = 200;
    const guidewireSpacing = 2;
    const wire = new ElasticRod(
        guidewireLength / guidewireSpacing + 1,
        guidewireSpacing
    );
    for (let index = 0; index < wire.nodes.length; index++) {
        const node = wire.nodes[index];
        node.x = index * guidewireSpacing - guidewireLength - 20 +
            guidewireInserted;
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
        tailProgressRef: () => guidewireInserted,
        vessel: {
            sheath: {
                start: { x: -20, y: 0, z: 0 },
                end: { x: 0, y: 0, z: 0 }
            },
            segments: []
        },
        maxLength: 160
    });
    catheter.setType(type);
    catheter.setExternalCollisionSolver(true);
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod(`${type}-catheter`, 128, 4, {
        ...DEFAULT_TOOL_PROFILES.catheter,
        rodModel: 'kirchhoff'
    });
    for (let step = 0; step < 210; step++) {
        catheter.advance(1, DT, guidewireInserted);
        catheter.stepPhysics(DT, { collisions: false });
        catheter.syncXpbdBody(body);
    }
    return { body, catheter };
}

for (const type of ['pigtail', 'berenstein']) {
    test(`${type} Kirchhoff material is independent of guidewire support`, () => {
        const standalone = createDeployedCatheter(type, 0);
        const supported = createDeployedCatheter(type, 140);
        try {
            const first = standalone.body.activeStart;
            const last = standalone.body.activeEnd + 1;
            assert.deepEqual(
                [supported.body.activeStart, supported.body.activeEnd],
                [standalone.body.activeStart, standalone.body.activeEnd]
            );
            for (const property of [
                'restRotation1',
                'restRotation2',
                'restRotation3',
                'kirchhoffBendCompliance1',
                'kirchhoffBendCompliance2',
                'kirchhoffTwistCompliance',
                'maxBendAngleByNode'
            ]) {
                assert.deepEqual(
                    Array.from(supported.body[property].slice(first, last)),
                    Array.from(standalone.body[property].slice(first, last)),
                    `${property} must not contain a hidden composite-beam override`
                );
            }
        } finally {
            standalone.catheter.dispose();
            supported.catheter.dispose();
        }
    });
}

for (const type of ['pigtail', 'berenstein']) {
    test(`${type} sync installs only its signed Kirchhoff material rest strain`, () => {
        const { body, catheter } = createDeployedCatheter(type);
        try {
            assert.equal(body.rodModel, 'kirchhoff');
            assert.ok(body.activeEnd - body.activeStart > 12);
            let totalRestTurn = 0;
            for (let joint = body.activeStart + 1; joint < body.activeEnd; joint++) {
                totalRestTurn += body.restRotation1[joint];
                assert.equal(body.restRotation2[joint], 0);
                assert.equal(body.restRotation3[joint], 0);
            }
            assert.ok(
                Math.abs(totalRestTurn - KIRCHHOFF_PROFILE_EXPECTED_TURNS[type]) < 1e-8,
                `${type} signed rest turn ${totalRestTurn}`
            );
            for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
                assert.equal(body.restDirectionEnabled[segment], 0);
            }
            for (let node = body.activeStart; node <= body.activeEnd; node++) {
                assert.equal(body.restShapeEnabled[node], 0);
            }
        } finally {
            catheter.dispose();
        }
    });
}

test('catheter rotation changes only the proximal material orientation boundary', () => {
    const { body, catheter } = createDeployedCatheter('pigtail');
    try {
        const arrays = [
            body.x, body.y, body.z,
            body.previousX, body.previousY, body.previousZ,
            body.velocityX, body.velocityY, body.velocityZ
        ];
        const snapshots = arrays.map(array => Array.from(array));
        const initialControl = [
            body.orientationControlX,
            body.orientationControlY,
            body.orientationControlZ,
            body.orientationControlW
        ];

        catheter.rotate(1, 0.1);
        catheter.syncXpbdBody(body);

        arrays.forEach((array, index) => {
            assert.deepEqual(Array.from(array), snapshots[index]);
        });
        const rotatedControl = [
            body.orientationControlX,
            body.orientationControlY,
            body.orientationControlZ,
            body.orientationControlW
        ];
        const absoluteDot = Math.abs(initialControl.reduce(
            (sum, value, index) => sum + value * rotatedControl[index],
            0
        ));
        assert.ok(absoluteDot < 0.999, `orientation boundary did not rotate (${absoluteDot})`);
        assert.equal(body.orientationControlSegment, body.activeStart);
    } finally {
        catheter.dispose();
    }
});
