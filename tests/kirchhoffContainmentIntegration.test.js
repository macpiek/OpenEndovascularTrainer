import assert from 'node:assert/strict';
import {
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    multiplyQuaternions,
    quaternionExp
} from '../src/physics/discreteKirchhoffRod.js';

const TOLERANCE = 2e-5;

function rodProfile(overrides = {}) {
    return {

        radius: 0.2,
        innerRadius: 1,
        adaptationCompliance: 0,
        kirchhoffBendCompliance: 0,
        kirchhoffTwistCompliance: 0,
        stretchCompliance: 0,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        postStabilizationPasses: 0,
        linearDamping: 1,
        angularDamping: 1,
        projectionVelocityRetention: 1,
        sleepFrames: 1_000_000,
        ...overrides
    };
}

function twoRodWorld({
    innerStart = [0, 1.2, 0],
    innerEnd = [10, 1.2, 0],
    outerStart = [0, 0, 0],
    outerEnd = [10, 0, 0],
    iterations = 10
} = {}) {
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations,
        penetrationIterations: iterations,
        // This analytical fixture checks micrometre-scale geometry. Request
        // matching solve accuracy explicitly instead of relying on incidental
        // over-solving below the application's 0.001 mm default tolerance.
        coupledContainmentTolerance: TOLERANCE * 0.01
    });
    const outer = world.createRod('outer-kirchhoff', 2, 10, rodProfile());
    const innerLength = Math.hypot(
        innerEnd[0] - innerStart[0],
        innerEnd[1] - innerStart[1],
        innerEnd[2] - innerStart[2]
    );
    const inner = world.createRod(
        'inner-kirchhoff',
        2,
        innerLength,
        rodProfile()
    );
    outer.setNodePosition(0, ...outerStart);
    outer.setNodePosition(1, ...outerEnd);
    inner.setNodePosition(0, ...innerStart);
    inner.setNodePosition(1, ...innerEnd);
    outer.captureRestConfiguration();
    inner.captureRestConfiguration();
    return { world, inner, outer };
}

// Every containment uses the shared Kirchhoff model.
{
    const { world, inner, outer } = twoRodWorld();
    const containment = world.addContainment(inner, outer);
    assert.equal(containment.model, 'kirchhoff');
}

// Material overlap is published as one coherent window and normalized to the
// currently active rods. This is the only topology mutation path used by the
// runtime before a coupled fixed step.
{
    const { world, inner, outer } = twoRodWorld();
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        enabled: false
    });
    containment.kirchhoffOuterSegmentByInner.fill(7);
    world.updateContainmentWindow(containment, {
        enabled: true,
        outerStartNode: -4,
        startNode: -3,
        endNode: 12,
        innerArcOffset: 2.5,
        containedLength: 8,
        enforceDistalPortal: true
    });
    assert.equal(containment.enabled, true);
    assert.equal(containment.outerStartNode, outer.activeStart);
    assert.equal(containment.startNode, inner.activeStart);
    assert.equal(containment.endNode, inner.activeEnd);
    assert.equal(containment.innerArcOffset, 2.5);
    assert.equal(containment.containedLength, 8);
    assert.equal(containment.enforceDistalPortal, true);
    assert.equal(containment._kirchhoffMappingLocked, false);
}

// A hard radial constraint uses reciprocal barycentric gradients. Both rods
// opt into the reaction; their separate Kirchhoff energies determine the
// eventual shared equilibrium instead of snapping either centerline.
{
    const { world, inner, outer } = twoRodWorld();
    const centerBefore = (inner.y[0] + outer.y[0]) * 0.5;
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        innerResponse: 1,
        outerResponse: 1,
        openDistal: false,
        openProximal: false,
        enforceDistalPortal: false,
        portalFilletRadius: 0
    });
    world.stepFixed();
    const clearance = 1 - inner.radius;
    assert.ok(inner.y[0] < 1.2, 'inner rod must receive the inward gradient');
    assert.ok(outer.y[0] > 0, 'outer rod must receive the opposite reaction');
    assert.ok(Math.abs(
        (inner.y[0] - outer.y[0]) - clearance
    ) < TOLERANCE);
    assert.ok(Math.abs(
        (
            inner.y[0] + inner.y[1] + outer.y[0] + outer.y[1]
        ) * 0.25 - centerBefore
    ) < TOLERANCE);
}

// A crossing through the open distal aperture is not assigned a direction or
// end-cap constraint. This remains true even for an oblique crossing.
{
    const { world, inner, outer } = twoRodWorld({
        innerStart: [9, -0.7, 0],
        innerEnd: [11, 0.7, 0]
    });
    const before = [inner.x[0], inner.y[0], inner.x[1], inner.y[1]];
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        openDistal: true,
        enforceDistalPortal: true
    });
    world.stepFixed();
    assert.deepEqual(
        [inner.x[0], inner.y[0], inner.x[1], inner.y[1]],
        before
    );
    assert.ok(![...containment.manifold.contacts()].some(
        contact => contact.feature.endsWith(':distal-rim')
    ));
}

// A crossing outside the aperture is constrained only by the circular rim.
// Both rods react; no synthetic exit tangent or distal rotation is generated.
{
    const { world, inner, outer } = twoRodWorld({
        innerStart: [9, 0.9, 0],
        innerEnd: [11, 1.3, 0]
    });
    const innerBefore = inner.y[1];
    const outerBefore = outer.y[1];
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        openDistal: true,
        enforceDistalPortal: true,
        portalFilletRadius: 0,

    });
    world.stepFixed();
    const rim = [...containment.manifold.contacts()].find(
        contact => (
            contact.feature.endsWith(':distal-rim') ||
            contact.feature.endsWith(':distal-fillet')
        ) && contact.normalLambda > 0
    );
    assert.ok(rim);
    assert.ok(inner.y[1] < innerBefore);
    assert.ok(outer.y[1] > outerBefore);
}

// Sliding and torsional friction share the same normal load. Their persistent
// multipliers remain inside the translational Coulomb disk and torsional
// Coulomb interval, while the two material frames receive opposite impulses.
{
    const { world, inner, outer } = twoRodWorld({ iterations: 6 });
    inner.velocityX.fill(1);
    const initial = {
        x: inner.orientationX[0],
        y: inner.orientationY[0],
        z: inner.orientationZ[0],
        w: inner.orientationW[0]
    };
    const target = multiplyQuaternions(
        quaternionExp({ x: 0.2, y: 0, z: 0 }),
        initial
    );
    inner.setProximalOrientationControl(
        target.x,
        target.y,
        target.z,
        target.w,
        1e-3
    );
    const axialFriction = 0.5;
    const torsionalFriction = 0.08;
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: axialFriction,
        axialFriction,
        torsionalFriction,
        openDistal: false
    });
    world.stepFixed();
    const contact = [...containment.manifold.contacts()].find(
        entry => entry.feature.endsWith(':side') && entry.normalLambda > 0
    );
    assert.ok(contact);
    assert.ok(Math.hypot(...contact.tangentLambda) <=
        axialFriction * contact.normalLambda + 1e-10);
    assert.ok(Math.abs(contact.twistLambda) <=
        torsionalFriction * contact.effectiveTwistRadius *
            contact.normalLambda + 1e-10);
    assert.equal(contact.frictionCoefficient, axialFriction);
    assert.equal(contact.twistFrictionCoefficient, torsionalFriction);
    assert.ok(Math.abs(
        contact.innerTwistImpulse + contact.outerTwistImpulse
    ) < 1e-12);
    assert.ok(outer.velocityX[0] > 0, 'outer rod receives sliding support');
    assert.ok(inner.velocityX[0] < 1, 'inner sliding motion is opposed');
}

// A tight wet lumen damps unresolved transverse checkerboard motion without
// turning axial guidewire feed into drag. Both rods remain independent bodies;
// only their relative short-wave velocity is dissipated.
{
    const world = new EndovascularPhysicsWorld({
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 6
    });
    const outer = world.createRod(
        'outer-velocity-bearing',
        5,
        4,
        rodProfile()
    );
    const inner = world.createRod(
        'inner-velocity-bearing',
        5,
        4,
        rodProfile()
    );
    for (let node = 0; node < 5; node++) {
        outer.setNodePosition(node, node * 4, 0, 0);
        inner.setNodePosition(node, node * 4, 0, 0);
        inner.velocityX[node] = 10;
        inner.velocityY[node] = node === 1 ? 4 : node === 2 ? -4 : 0;
    }
    outer.captureRestConfiguration();
    inner.captureRestConfiguration();
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        axialFriction: 0,
        torsionalFriction: 0,
        radialVelocityDamping: 0.9,
        coupledBendingRateDamping: 0.5,
        coupledBendingRatePasses: 8,
        openDistal: false
    });
    world.stepFixed();
    const minimumAxialSpeed = Math.min(...inner.velocityX);
    const maximumTransverseSpeed = Math.max(
        ...inner.velocityY.map(Math.abs),
        ...outer.velocityY.map(Math.abs)
    );
    assert.ok(minimumAxialSpeed > 9.5,
        `axial sliding was incorrectly damped (${minimumAxialSpeed} mm/s)`);
    assert.ok(maximumTransverseSpeed < 1,
        `checkerboard velocity survived lumen damping (${maximumTransverseSpeed} mm/s)`);
    assert.equal(containment.radialVelocityDamping, 0.9);
}

console.log('Kirchhoff containment-integration tests passed');
