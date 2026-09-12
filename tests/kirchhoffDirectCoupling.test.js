import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { CATHETER_REFERENCE_RIGIDITY, kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { evaluateBendTwistConstraint, multiplyQuaternions, quaternionExp } from '../src/physics/discreteKirchhoffRod.js';
import { evaluateKirchhoffSlidingPortal } from '../src/physics/kirchhoffSlidingPortal.js';
import { solveKirchhoffDirect, kirchhoffDirectContactResponse,
    applyKirchhoffDirectCorrection } from '../src/physics/kirchhoffDirectSolver.js';

const DT = 1 / 120;
const profile = {
     
    
    radius: 0.2, innerRadius: 0.6,
    linearDamping: 1, angularDamping: 1,
    foldLimitStrength: 0, projectionVelocityRetention: 1,
    sleepFrames: 1_000_000
};

test('catheter preform geometry does not implicitly multiply material rigidity', () => {
    for (const type of ['berenstein', 'pigtail', 'sim1']) {
        assert.equal(kirchhoffMaterialProfile(type).sample(0).EI1, CATHETER_REFERENCE_RIGIDITY);
        assert.equal(kirchhoffMaterialProfile(type).sample(100).EI1, CATHETER_REFERENCE_RIGIDITY);
    }
});

test('all rods use immutable direct Kirchhoff mechanics and nominal catheter rigidity', () => {
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('default', 21, 2);
    assert.equal(body.rodModel, 'kirchhoff');
    assert.equal(body.constitutiveSolver, 'direct');
    assert.throws(() => { body.constitutiveSolver = 'local'; }, TypeError);
    assert.throws(() => { body.rodModel = 'legacy'; }, TypeError);
    applyKirchhoffMaterialProfile(body, 'pigtail');
    assert.ok(Math.abs(body.kirchhoffBendCompliance1[10] - 2/CATHETER_REFERENCE_RIGIDITY) < 1e-12);
});

function frame(body, node) {
    return { x: body.orientationX[node], y: body.orientationY[node],
        z: body.orientationZ[node], w: body.orientationW[node] };
}

function maximumLengthError(body) {
    let result = 0;
    for (let i = body.activeStart; i < body.activeEnd; i++) {
        result = Math.max(result, Math.abs(Math.hypot(
            body.x[i + 1] - body.x[i], body.y[i + 1] - body.y[i],
            body.z[i + 1] - body.z[i]) - body.restLength[i]));
    }
    return result;
}

for (const velocity of [-6, 6]) {
    test(`open catheter permits frictionless axial sliding at ${velocity} mm/s`, () => {
        const world = new EndovascularPhysicsWorld({ fixedDt: DT });
        const inner = world.createRod('wire', 25, 5, profile);
        const outer = world.createRod('catheter', 11, 5, profile);
        for (let i = 0; i < inner.count; i++) {
            inner.setNodePosition(i, i * 5 - 20, 0.1, 0);
            inner.velocityX[i] = velocity;
        }
        // Exercise the idle-catheter path too: it must not impose its velocity
        // on the unsupported distal guidewire in the absence of contact.
        outer.projectionVelocityRetention = 0.005;
        const contact = world.addContainment(inner, outer, {
            model: 'kirchhoff', innerRadius: 0.6,
            startNode: 4, endNode: 13, containedLength: 47,
            enforceDistalPortal: true, axialFriction: 0, torsionalFriction: 0
        });
        // The material registration deliberately differs by 3 mm from the
        // spatial mouth. Registering equal arclengths is not an axial joint.
        assert.equal(contact.distalPortalModel, 'spatial');
        for (let step = 0; step < 60; step++) world.stepFixed();
        for (let i = 0; i < inner.count; i++) {
            assert.ok(Math.abs(inner.x[i] - (i * 5 - 20 + velocity * 0.5)) < 0.003,
                `material node ${i} acquired an artificial axial reaction`);
        }
        assert.ok(Math.abs(outer.x[outer.activeEnd] - 50) < 1e-5);
        assert.equal('materialPortalAxialLambda' in contact, false);
        assert.ok(maximumLengthError(inner) < 0.001);
    });
}

test('radial lumen reaction is reciprocal with unequal tool masses', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, iterations: 8 });
    const inner = world.createRod('wire', 2, 10, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', 2, 10, { ...profile, mass: 3 });
    inner.setNodePosition(0, 0, 0.8, 0);
    inner.setNodePosition(1, 10, 0.8, 0);
    // Keep both axes parallel; positions remain free, so only the internal
    // radial reaction determines their mass-weighted translation.
    for (const body of [inner, outer]) {
        const q = frame(body, 0);
        body.setProximalOrientationControl(q.x, q.y, q.z, q.w, 0, 0);
    }
    world.addContainment(inner, outer, {
        model: 'kirchhoff', innerRadius: 0.6, portalFilletRadius: 0,
        axialFriction: 0, torsionalFriction: 0,
        coupledBendingRateDamping: 0, radialVelocityDamping: 0
    });
    world.stepFixed();
    const wireY = (inner.y[0] + inner.y[1]) / 2;
    const catheterY = (outer.y[0] + outer.y[1]) / 2;
    assert.ok(wireY < 0.8 && catheterY > 0, 'both rods must react');
    assert.ok(Math.abs((wireY + 3 * catheterY) / 4 - 0.2) < 1e-5,
        'internal contact must preserve the mass-weighted center');
    assert.ok(Math.abs(wireY - catheterY - 0.4) < 1e-4,
        `contact must retain the finite radial clearance (${wireY - catheterY})`);
    assert.ok(maximumLengthError(inner) < 0.001);
    assert.ok(maximumLengthError(outer) < 0.001);
});

test('unloaded coaxial tools do not transmit torsion through the lumen', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT });
    const inner = world.createRod('wire', 9, 5, profile);
    const outer = world.createRod('catheter', 9, 5, profile);
    const original = frame(outer, 0);
    const rotated = multiplyQuaternions(original, quaternionExp({ x: 0, y: 0, z: 0.2 }));
    inner.setProximalOrientationControl(rotated.x, rotated.y, rotated.z, rotated.w, 0, 0);
    world.addContainment(inner, outer, {
        model: 'kirchhoff', innerRadius: 0.6, axialFriction: 1, torsionalFriction: 1,
        enforceDistalPortal: true, containedLength: 38
    });
    for (let step = 0; step < 30; step++) world.stepFixed();
    for (let i = 0; i < outer.segmentCount; i++) {
        const actual = frame(outer, i);
        assert.ok(Math.abs(actual.x * original.x + actual.y * original.y +
            actual.z * original.z + actual.w * original.w) > 1 - 1e-7,
        'torsional friction must vanish without normal load');
    }
});

test('a compliant contact converges without hiding its physical penetration', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT });
    const inner = world.createRod('wire', 2, 10, profile);
    const outer = world.createRod('catheter', 2, 10, profile);
    inner.y.fill(0.8);
    for (const body of [inner, outer]) {
        const q = frame(body, 0);
        body.setProximalOrientationControl(q.x, q.y, q.z, q.w, 0, 0);
    }
    world.addContainment(inner, outer, {
        model: 'kirchhoff', innerRadius: 0.6, compliance: 1e-4,
        openDistal: false, axialFriction: 0, torsionalFriction: 0,
        radialVelocityDamping: 0, coupledBendingRateDamping: 0
    });
    world.stepFixed();
    const stats = world.getStats();
    assert.ok(stats.containments[0].maximumViolation > 0.005,
        'raw compliant deformation must remain visible in diagnostics');
    assert.ok(stats.containments[0].solverResidual < 0.001);
    assert.equal(stats.coupledClosureConverged, true);
    assert.ok(stats.coupledClosurePasses < 64,
        'a settled compliant contact must not exhaust the closure budget');
});

test('the sliding mouth detects radial separation and releases a withdrawn wire tip', () => {
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('wire', 5, 5, profile);
    const outer = world.createRod('catheter', 3, 5, profile);
    outer.y[2] = 1;
    const constraint = world.addContainment(inner, outer, {
        model: 'kirchhoff', enforceDistalPortal: true, containedLength: 10, endNode: 2
    });
    const state = evaluateKirchhoffSlidingPortal(constraint);
    assert.ok(Math.abs(state.violation - 0.6) < 1e-7);
    assert.equal(state.x, 0, 'the reaction must have no axial component');
    inner.setActiveRange(0, 1);
    constraint.endNode = 1;
    assert.equal(evaluateKirchhoffSlidingPortal(constraint).segment, -1,
        'withdrawal must not create an artificial end cap');
});

test('contact response transmits bending along a clamped rod while preserving its boundary', () => {
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('beam', 21, 2, profile);
    applyKirchhoffMaterialProfile(body, 'berenstein');
    body.restRotation1.fill(0);
    body.restRotation2.fill(0);
    body.setPinned(0, true);
    const q = frame(body, 0);
    body.setProximalOrientationControl(q.x, q.y, q.z, q.w, 0, 0);
    solveKirchhoffDirect(body, DT, true);
    const response = kirchhoffDirectContactResponse(body, [20], [1], 1, [0, 1, 0], 1);
    assert.ok(response.mobility > 0 && response.mobility < body.inverseMass[20]);
    assert.ok(response.correction[18 * 6 + 1] > 0,
        'contact must transmit a reaction beyond its interpolation nodes');
    const scale = 0.01 / response.maximumPosition;
    applyKirchhoffDirectCorrection(body, response.correction, response.lambda, scale, true);
    assert.equal(body.x[0], 0);
    assert.equal(body.y[0], 0);
    assert.ok(maximumLengthError(body) < 1e-4);
    assert.ok(body.bendTwistLambda1.some(value => Math.abs(value) > 0) ||
        body.bendTwistLambda2.some(value => Math.abs(value) > 0),
    'material multipliers must include the contact reaction');
});

for (const type of ['berenstein', 'pigtail', 'sim1']) {
    test(`${type} recovers its manufactured curvature using the direct rod solver`, () => {
        const world = new EndovascularPhysicsWorld({ fixedDt: DT });
        const body = world.createRod(type, 41, 2, {
            ...profile, mass: 0.01, linearDamping: 0.9, angularDamping: 0.9
        });
        applyKirchhoffMaterialProfile(body, type);
        body.setPinned(0, true);
        const base = frame(body, 0);
        body.setProximalOrientationControl(base.x, base.y, base.z, base.w, 0, 0);
        const manufactured = Array.from(body.restRotation1);
        // Start straight: no path or shape target is supplied during recovery.
        for (let step = 0; step < 1200; step++) world.stepFixed();
        let maxStrain = 0;
        for (let i = 1; i < body.segmentCount; i++) {
            const state = evaluateBendTwistConstraint(frame(body, i - 1), frame(body, i), {
                x: body.restRotation1[i], y: body.restRotation2[i], z: body.restRotation3[i]
            });
            maxStrain = Math.max(maxStrain, Math.hypot(state.strain.x, state.strain.y, state.strain.z));
        }
        assert.ok(maxStrain < 0.005, `residual material strain: ${maxStrain} rad`);
        assert.ok(maximumLengthError(body) < 0.001);
        assert.deepEqual(Array.from(body.restRotation1), manufactured);
    });
}
