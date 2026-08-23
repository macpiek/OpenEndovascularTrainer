import assert from 'node:assert/strict';
import test from 'node:test';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { guidewireRelaxationPasses } from '../src/physics/guidewireRelaxationRate.js';

const SEGMENT_LENGTH = 4;
const NODE_COUNT = 31;
const MATERIAL_LENGTH = (NODE_COUNT - 1) * SEGMENT_LENGTH;

const parameterSets = Object.freeze([
    Object.freeze({
        name: 'minimum',
        catheterShaft: 0.25,
        catheterTip: 0.25,
        guidewireShaft: 0.25,
        guidewireTip: 0.25,
        relaxationRate: 1
    }),
    Object.freeze({
        name: 'application-default',
        catheterShaft: 25,
        catheterTip: 5,
        guidewireShaft: 10,
        guidewireTip: 4.55,
        relaxationRate: 30
    }),
    Object.freeze({
        name: 'maximum',
        catheterShaft: 25,
        catheterTip: 10,
        guidewireShaft: 25,
        guidewireTip: 10,
        relaxationRate: 50
    })
]);

function createMaterialRod(world, id, profile) {
    const body = world.createRod(id, NODE_COUNT, SEGMENT_LENGTH, {
        ...profile,
        rodModel: 'kirchhoff',
        sleepFrames: 10000
    });
    for (let node = 0; node < NODE_COUNT; node++) {
        body.setNodePosition(node, node * SEGMENT_LENGTH, 0, 0);
        body.materialCoordinate[node] = node * SEGMENT_LENGTH;
    }
    body.copyCurrentToPrevious();
    body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    body.setPinned(0, true);
    return body;
}

function exercisePair(catheterType, guidewireType, parameters) {
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8,
        coupledClosureMaxPasses: 32
    });
    const guidewire = createMaterialRod(
        world,
        `${catheterType}-${guidewireType}-${parameters.name}-wire`,
        DEFAULT_TOOL_PROFILES.guidewire
    );
    const catheter = createMaterialRod(
        world,
        `${catheterType}-${guidewireType}-${parameters.name}-catheter`,
        DEFAULT_TOOL_PROFILES.catheter
    );
    applyKirchhoffMaterialProfile(guidewire, guidewireType, {
        materialCoordinates: guidewire.materialCoordinate,
        tipCoordinate: MATERIAL_LENGTH,
        shaftStiffnessScale: parameters.guidewireShaft,
        tipStiffnessScale: parameters.guidewireTip
    });
    applyKirchhoffMaterialProfile(catheter, catheterType, {
        materialCoordinates: catheter.materialCoordinate,
        tipCoordinate: MATERIAL_LENGTH,
        shaftStiffnessScale: parameters.catheterShaft,
        tipStiffnessScale: parameters.catheterTip
    });
    const relaxationPasses = guidewireRelaxationPasses(
        parameters.relaxationRate
    );
    guidewire.relaxationPasses = relaxationPasses;
    catheter.relaxationPasses = relaxationPasses;
    const containment = world.addContainment(guidewire, catheter, {
        model: 'kirchhoff',
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        friction: DEFAULT_TOOL_PROFILES.catheter.lumenFriction,
        axialFriction: DEFAULT_TOOL_PROFILES.catheter.lumenAxialFriction,
        torsionalFriction: DEFAULT_TOOL_PROFILES.catheter.lumenTorsionalFriction,
        openProximal: true,
        openDistal: true,
        outerStartNode: 0,
        startNode: 0,
        endNode: NODE_COUNT - 1,
        containedLength: MATERIAL_LENGTH
    });
    world.updateContainmentWindow(containment, {
        enabled: true,
        outerStartNode: 0,
        startNode: 0,
        endNode: NODE_COUNT - 1,
        innerArcOffset: 0,
        containedLength: MATERIAL_LENGTH,
        enforceDistalPortal: true
    });

    for (let step = 0; step < 36; step++) world.stepFixed();
    const stats = world.getStats();
    const guidewireStats = stats.bodies.find(body => body.id === guidewire.id);
    const catheterStats = stats.bodies.find(body => body.id === catheter.id);
    assert.equal(guidewireStats?.finite, true);
    assert.equal(catheterStats?.finite, true);
    assert.equal(guidewire.lastRelaxationPasses, relaxationPasses);
    assert.equal(catheter.lastRelaxationPasses, relaxationPasses);
    assert.ok(guidewireStats.maxLengthError < 0.08,
        `guidewire length residual ${guidewireStats.maxLengthError}`);
    assert.ok(catheterStats.maxLengthError < 0.08,
        `catheter length residual ${catheterStats.maxLengthError}`);
    assert.ok(containment.kirchhoffMaxViolation < 0.15,
        `lumen residual ${containment.kirchhoffMaxViolation}`);
    assert.ok(guidewireStats.maxBendAngleDegrees < 75,
        `guidewire fold ${guidewireStats.maxBendAngleDegrees}`);
    assert.ok(catheterStats.maxBendAngleDegrees < 45,
        `catheter fold ${catheterStats.maxBendAngleDegrees}`);
}

for (const catheterType of ['pigtail', 'berenstein']) {
    for (const guidewireType of ['glidewire', 'steel-j-035']) {
        for (const parameters of parameterSets) {
            test(
                `${catheterType} × ${guidewireType} remains finite at ` +
                    `${parameters.name} coupled controls`,
                () => exercisePair(catheterType, guidewireType, parameters)
            );
        }
    }
}
