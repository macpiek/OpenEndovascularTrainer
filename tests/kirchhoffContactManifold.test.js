import assert from 'node:assert/strict';
import {
    KirchhoffContactManifold,
    materialSegmentContactId
} from '../src/physics/kirchhoffContactManifold.js';

const EPSILON = 1e-12;

function approximatelyEqual(actual, expected, tolerance = EPSILON) {
    assert.ok(
        Math.abs(actual - expected) <= tolerance,
        `expected ${actual} to be within ${tolerance} of ${expected}`
    );
}

// A remesh changes transient array slots, not material contact identity. Warm
// starts must survive that remap and the tangential impulse must be expressed
// in the refreshed local basis without rotating in world space.
const persistent = new KirchhoffContactManifold({
    frictionCoefficient: 0.5,
    retentionSteps: 1
});
persistent.beginStep();
const original = persistent.upsertContact({
    innerMaterialSegmentId: 'wire:s=42',
    outerMaterialSegmentId: 'catheter:s=37',
    feature: 'lumen',
    innerSegmentIndex: 42,
    outerSegmentIndex: 9,
    normal: [0, 0, 1],
    tangentU: [1, 0, 0],
    effectiveTwistRadius: 0.25
});
persistent.setNormalLambda(original, 4);
persistent.accumulateTangentialLambda(original, 0.2, 0.4);
persistent.accumulateTwistImpulse(original, 0.3);
persistent.endStep();

persistent.beginStep();
const remapped = persistent.upsertContact({
    innerMaterialSegmentId: 'wire:s=42',
    outerMaterialSegmentId: 'catheter:s=37',
    feature: 'lumen',
    innerSegmentIndex: 6,
    outerSegmentIndex: 21,
    normal: [0, 0, 1],
    tangentU: [0, 1, 0]
});
assert.strictEqual(remapped, original,
    'the same material segment pair should retain one persistent contact');
assert.equal(remapped.id, materialSegmentContactId(
    'wire:s=42',
    'catheter:s=37',
    'lumen'
));
assert.equal(remapped.innerSegmentIndex, 6);
assert.equal(remapped.outerSegmentIndex, 21);
approximatelyEqual(remapped.normalLambda, 4);
approximatelyEqual(remapped.tangentLambda[0], 0.4);
approximatelyEqual(remapped.tangentLambda[1], -0.2);
approximatelyEqual(remapped.twistLambda, 0.3);
approximatelyEqual(remapped.innerTwistImpulse, 0.3);
approximatelyEqual(remapped.outerTwistImpulse, -0.3);
assert.equal(persistent.size, 1);
persistent.endStep();

// A tip-anchored adaptive/remeshed lattice continuously changes the exact
// material coordinate represented by one runtime contact slot. Recycle the
// allocation, but reset the multipliers exactly as a newly created material
// contact would do.
persistent.beginStep();
const advected = persistent.rekeyKnownContact(remapped, {
    innerMaterialSegmentId: 'wire:s=42.25',
    outerMaterialSegmentId: 'catheter:s=37.25',
    feature: 'lumen',
    innerSegmentIndex: 6,
    outerSegmentIndex: 21,
    normal: [0, 0, 1],
    tangentU: [0, 1, 0]
});
assert.strictEqual(advected, original,
    'continuous material relabelling should recycle the existing allocation');
assert.equal(advected.id, materialSegmentContactId(
    'wire:s=42.25',
    'catheter:s=37.25',
    'lumen'
));
approximatelyEqual(advected.normalLambda, 0);
approximatelyEqual(advected.tangentLambda[0], 0);
approximatelyEqual(advected.tangentLambda[1], 0);
approximatelyEqual(advected.twistLambda, 0);
assert.equal(persistent.size, 1);
persistent.endStep();

// With mu=0 the Coulomb disk has zero radius, even under a non-zero normal
// load. No axial or circumferential friction impulse may survive.
const frictionless = new KirchhoffContactManifold({ frictionCoefficient: 0 });
frictionless.beginStep();
const frictionlessContact = frictionless.upsertContact({
    innerMaterialSegmentId: 3,
    outerMaterialSegmentId: 8,
    normal: [0, 1, 0],
    tangentU: [1, 0, 0],
    effectiveTwistRadius: 1
});
frictionless.setNormalLambda(frictionlessContact, 5);
const frictionlessProjection = frictionless.accumulateTangentialLambda(
    frictionlessContact,
    3,
    4
);
approximatelyEqual(frictionlessProjection.lambdaU, 0);
approximatelyEqual(frictionlessProjection.lambdaV, 0);
approximatelyEqual(frictionlessProjection.limit, 0);
assert.equal(frictionlessProjection.clamped, true);
const frictionlessTwist = frictionless.accumulateTwistImpulse(
    frictionlessContact,
    0.7
);
approximatelyEqual(frictionlessTwist.inner, 0);
approximatelyEqual(frictionlessTwist.outer, 0);
approximatelyEqual(frictionlessTwist.limit, 0);
assert.equal(frictionlessTwist.clamped, true);
frictionless.endStep();

// A requested 3-4 impulse under mu*lambda_n=1 must retain direction while its
// total magnitude is projected exactly onto the friction cone boundary.
const coulomb = new KirchhoffContactManifold({ frictionCoefficient: 0.5 });
coulomb.beginStep();
const coneContact = coulomb.upsertContact({
    innerMaterialSegmentId: 12,
    outerMaterialSegmentId: 4,
    normal: [0, 0, 1],
    tangentU: [1, 0, 0],
    effectiveTwistRadius: 0.4
});
coulomb.setNormalLambda(coneContact, 2);
const coneProjection = coulomb.accumulateTangentialLambda(
    coneContact,
    3,
    4
);
approximatelyEqual(coneProjection.limit, 1);
approximatelyEqual(coneProjection.lambdaU, 0.6);
approximatelyEqual(coneProjection.lambdaV, 0.8);
approximatelyEqual(Math.hypot(
    coneContact.tangentLambda[0],
    coneContact.tangentLambda[1]
), 1);
assert.equal(coneProjection.clamped, true);
const coneTwist = coulomb.accumulateTwistImpulse(coneContact, 1);
approximatelyEqual(coneTwist.limit, 0.4);
approximatelyEqual(coneTwist.inner, 0.4);
approximatelyEqual(coneTwist.outer, -0.4);
assert.equal(coneTwist.clamped, true);

// Unloading the normal contact must shrink the already accumulated tangent
// state as well; stale friction cannot outlive its supporting normal reaction.
coulomb.setNormalLambda(coneContact, 0.5);
approximatelyEqual(Math.hypot(
    coneContact.tangentLambda[0],
    coneContact.tangentLambda[1]
), 0.25);
approximatelyEqual(coneContact.twistLambda, 0.1);
approximatelyEqual(coneContact.innerTwistImpulse, 0.1);
approximatelyEqual(coneContact.outerTwistImpulse, -0.1);
coulomb.endStep();

// A non-zero coefficient and radius still cannot create torsional friction in
// the absence of a supporting normal reaction.
const unloaded = new KirchhoffContactManifold({ frictionCoefficient: 0.8 });
unloaded.beginStep();
const unloadedContact = unloaded.upsertContact({
    innerMaterialSegmentId: 1,
    outerMaterialSegmentId: 2,
    normal: [0, 0, 1],
    tangentU: [1, 0, 0],
    effectiveTwistRadius: 0.6
});
const unloadedTwist = unloaded.accumulateTwistImpulse(unloadedContact, 1);
approximatelyEqual(unloadedTwist.inner, 0);
approximatelyEqual(unloadedTwist.outer, 0);
approximatelyEqual(unloadedTwist.limit, 0);
unloaded.endStep();

// Generalized twist impulses are an internal contact action. Every accepted
// increment must therefore enter the two material frames with opposite signs.
const torque = new KirchhoffContactManifold();
torque.beginStep();
const torqueContact = torque.upsertContact({
    innerMaterialSegmentId: 'wire-tip',
    outerMaterialSegmentId: 'catheter-tip',
    feature: 'rim',
    normal: [0, 1, 0],
    tangentU: [1, 0, 0],
    frictionCoefficient: 0.5,
    effectiveTwistRadius: 1
});
torque.setNormalLambda(torqueContact, 4);
const firstTorque = torque.accumulateTwistImpulse(torqueContact, 0.7);
approximatelyEqual(firstTorque.appliedInner, 0.7);
approximatelyEqual(firstTorque.appliedOuter, -0.7);
const secondTorque = torque.accumulateTwistImpulse(torqueContact, -0.2);
approximatelyEqual(secondTorque.inner, 0.5);
approximatelyEqual(secondTorque.outer, -0.5);
approximatelyEqual(
    torqueContact.innerTwistImpulse + torqueContact.outerTwistImpulse,
    0
);
torque.endStep();

console.log('Kirchhoff contact-manifold tests passed');
