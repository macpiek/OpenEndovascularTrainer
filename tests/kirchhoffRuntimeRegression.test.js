import assert from 'node:assert/strict';
import {
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';

function profile(overrides = {}) {
    return {
        rodModel: 'kirchhoff',
        radius: 0.2,
        innerRadius: 1,
        adaptationCompliance: 0,
        kirchhoffBendCompliance: 0,
        kirchhoffTwistCompliance: 0,
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

// A segment within the broad activation shell but inside the lumen has no
// normal load and therefore no Coulomb capacity. The optimized early-out must
// retain exactly that physical result: no synthetic drag or torsional impulse.
{
    const world = new EndovascularPhysicsWorld({
        iterations: 3,
        penetrationIterations: 3,
        contactActivation: 0.25
    });
    const outer = world.createRod('runtime-outer', 5, 2, profile());
    const inner = world.createRod('runtime-inner', 5, 2, profile());
    inner.y.fill(0.65);
    inner.previousY.fill(0.65);
    inner.velocityX.fill(1);
    outer.captureRestConfiguration();
    inner.captureRestConfiguration();
    const containment = world.addContainment(inner, outer, {
        model: 'kirchhoff',
        innerRadius: 1,
        friction: 0.8,
        openDistal: false
    });
    world.stepFixed();
    const contacts = [...containment.manifold.contacts()];
    assert.ok(contacts.length > 0, 'activation shell should retain manifold contacts');
    for (const contact of contacts) {
        assert.equal(contact.normalLambda, 0);
        assert.deepEqual(Array.from(contact.tangentLambda), [0, 0]);
        assert.equal(contact.twistLambda, 0);
    }
    assert.ok(
        Math.max(...outer.velocityX.map(Math.abs)) < 1e-10,
        'an unloaded lumen must not receive guidewire friction'
    );
}

console.log('Kirchhoff runtime regression tests passed');
