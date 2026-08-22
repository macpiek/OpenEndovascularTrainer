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
        rodModel: 'kirchhoff',
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
        penetrationIterations: iterations
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

// Existing callers stay on the legacy containment solver unless they opt in.
{
    const { world, inner, outer } = twoRodWorld();
    const containment = world.addContainment(inner, outer);
    assert.equal(containment.model, 'legacy');
}

// A hard radial constraint uses equal/opposite barycentric gradients. The
// legacy response switches are deliberately ignored on this path: momentum
// is shared by the two material rods instead of snapping one centerline.
{
    const { world, inner, outer } = twoRodWorld();
    const centerBefore = (inner.y[0] + outer.y[0]) * 0.5;
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        innerResponse: 1,
        outerResponse: 0,
        finalProjection: 'outer',
        outerFollowsInnerCenterline: true,
        enforceDistalPortal: true
    });
    world.stepFixed();
    const clearance = 1 - inner.radius;
    assert.equal(containment.finalProjection, 'none');
    assert.ok(inner.y[0] < 1.2, 'inner rod must receive the inward gradient');
    assert.ok(outer.y[0] > 0, 'outer rod must receive the opposite reaction');
    assert.ok(Math.abs(
        (inner.y[0] - outer.y[0]) - clearance
    ) < TOLERANCE);
    assert.ok(Math.abs(
        (inner.y[0] + outer.y[0]) * 0.5 - centerBefore
    ) < TOLERANCE);
    assert.equal(containment.portalDirectionLambda, 0);
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
    assert.equal(containment.portalDirectionLambda, 0);
    assert.ok(![...containment.manifold.contacts()].some(
        contact => contact.feature.endsWith(':distal-rim')
    ));
}

// A crossing outside the aperture is constrained only by the circular rim.
// Both rods react; no synthetic exit tangent or distal rotation is generated.
{
    const { world, inner, outer } = twoRodWorld({
        innerStart: [9, 0.7, 0],
        innerEnd: [11, 1.1, 0]
    });
    const innerBefore = inner.y[1];
    const outerBefore = outer.y[1];
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0,
        openDistal: true,
        enforceDistalPortal: true,
        portalInnerResponse: 0,
        portalOuterResponse: 0
    });
    world.stepFixed();
    const rim = [...containment.manifold.contacts()].find(
        contact => contact.feature.endsWith(':distal-rim')
    );
    assert.ok(rim && rim.normalLambda > 0);
    assert.ok(inner.y[1] < innerBefore);
    assert.ok(outer.y[1] > outerBefore);
    assert.equal(containment.portalDirectionLambda, 0);
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
    const frictionCoefficient = 0.5;
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: frictionCoefficient,
        openDistal: false
    });
    world.stepFixed();
    const contact = [...containment.manifold.contacts()].find(
        entry => entry.feature.endsWith(':side') && entry.normalLambda > 0
    );
    assert.ok(contact);
    assert.ok(Math.hypot(...contact.tangentLambda) <=
        frictionCoefficient * contact.normalLambda + 1e-10);
    assert.ok(Math.abs(contact.twistLambda) <=
        frictionCoefficient * contact.effectiveTwistRadius *
            contact.normalLambda + 1e-10);
    assert.ok(Math.abs(
        contact.innerTwistImpulse + contact.outerTwistImpulse
    ) < 1e-12);
    assert.ok(outer.velocityX[0] > 0, 'outer rod receives sliding support');
    assert.ok(inner.velocityX[0] < 1, 'inner sliding motion is opposed');
}

console.log('Kirchhoff containment-integration tests passed');
