import assert from 'node:assert/strict';
import { KirchhoffContactManifold } from '../src/physics/kirchhoffContactManifold.js';
import {
    closestSegmentSegment,
    evaluateKirchhoffLumenSegmentContact
} from '../src/physics/kirchhoffLumenContact.js';

const EPSILON = 1e-10;

function approximatelyEqual(actual, expected, tolerance = EPSILON) {
    assert.ok(
        Math.abs(actual - expected) <= tolerance,
        `expected ${actual} to be within ${tolerance} of ${expected}`
    );
}

function sumVectors(vectors) {
    return vectors.reduce((sum, vector) => [
        sum[0] + vector[0],
        sum[1] + vector[1],
        sum[2] + vector[2]
    ], [0, 0, 0]);
}

const coaxialClosest = closestSegmentSegment(
    [2, 0, 0],
    [8, 0, 0],
    [0, 0, 0],
    [10, 0, 0]
);
approximatelyEqual(coaxialClosest.distance, 0);

// A coaxial guidewire has the full centerline clearance and must not receive a
// contact merely because it occupies the lumen.
const coaxial = evaluateKirchhoffLumenSegmentContact({
    innerStart: [2, 0, 0],
    innerEnd: [8, 0, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-1',
    outerMaterialSegmentId: 'catheter-1'
});
approximatelyEqual(coaxial.clearance, 0.8);
approximatelyEqual(coaxial.side.radialDistance, 0);
approximatelyEqual(coaxial.side.gap, 0.8);
assert.equal(coaxial.side.active, false);
assert.equal(coaxial.activeContacts.length, 0);

// Radial penetration produces one inward guidewire gradient and an equal,
// opposite catheter gradient. Material IDs, rather than transient indices,
// keep the manifold state alive across remapping.
const manifold = new KirchhoffContactManifold({ frictionCoefficient: 0.2 });
manifold.beginStep();
const penetrated = evaluateKirchhoffLumenSegmentContact({
    innerStart: [2, 1, 0],
    innerEnd: [8, 1.2, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-material-7',
    outerMaterialSegmentId: 'catheter-material-3',
    innerSegmentIndex: 17,
    outerSegmentIndex: 3,
    manifold
});
assert.ok(penetrated.side.gap < -0.39);
assert.equal(penetrated.side.active, true);
approximatelyEqual(penetrated.side.normal[0], 0);
approximatelyEqual(penetrated.side.normal[1], 1);
approximatelyEqual(penetrated.side.normal[2], 0);
approximatelyEqual(
    penetrated.side.innerWeights[0] + penetrated.side.innerWeights[1],
    1
);
approximatelyEqual(
    penetrated.side.outerWeights[0] + penetrated.side.outerWeights[1],
    1
);
const translationGradient = sumVectors([
    ...penetrated.side.gradients.inner,
    ...penetrated.side.gradients.outer
]);
approximatelyEqual(translationGradient[0], 0);
approximatelyEqual(translationGradient[1], 0);
approximatelyEqual(translationGradient[2], 0);
const persistentContact = penetrated.side.manifoldContact;
assert.ok(persistentContact);
manifold.setNormalLambda(persistentContact, 2);
manifold.endStep();

manifold.beginStep();
const remapped = evaluateKirchhoffLumenSegmentContact({
    innerStart: [2, 1, 0],
    innerEnd: [8, 1.2, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-material-7',
    outerMaterialSegmentId: 'catheter-material-3',
    innerSegmentIndex: 4,
    outerSegmentIndex: 19,
    manifold
});
assert.strictEqual(remapped.side.manifoldContact, persistentContact);
assert.equal(persistentContact.innerSegmentIndex, 4);
assert.equal(persistentContact.outerSegmentIndex, 19);
approximatelyEqual(persistentContact.normalLambda, 2);
manifold.endStep();

// The distal end is genuinely open. A steeply angled segment may cross the
// aperture when its plane intersection fits inside the lumen; no directional
// alignment constraint is generated.
const openCrossing = evaluateKirchhoffLumenSegmentContact({
    innerStart: [9, -0.7, 0],
    innerEnd: [11, 0.7, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-crossing',
    outerMaterialSegmentId: 'catheter-tip',
    openDistal: true
});
assert.equal(openCrossing.portal.crosses, true);
assert.equal(openCrossing.portal.valid, true);
approximatelyEqual(openCrossing.portal.radialDistance, 0);
assert.equal(openCrossing.portal.contact, null);
assert.equal(openCrossing.activeContacts.length, 0);

// A crossing outside the aperture is a rim violation. It cannot be accepted
// as an open-end exit, and it is corrected radially without prescribing the
// direction of the free distal guidewire.
const rimManifold = new KirchhoffContactManifold();
rimManifold.beginStep();
const blockedCrossing = evaluateKirchhoffLumenSegmentContact({
    innerStart: [9, 0.7, 0],
    innerEnd: [11, 1.1, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-blocked',
    outerMaterialSegmentId: 'catheter-tip',
    innerSegmentIndex: 9,
    outerSegmentIndex: 4,
    openDistal: true,
    manifold: rimManifold
});
assert.equal(blockedCrossing.portal.crosses, true);
assert.equal(blockedCrossing.portal.valid, false);
approximatelyEqual(blockedCrossing.portal.radialDistance, 0.9);
approximatelyEqual(blockedCrossing.portal.gap, -0.1);
assert.equal(blockedCrossing.portal.contact.kind, 'distal-rim');
assert.equal(blockedCrossing.portal.contact.active, true);
assert.ok(blockedCrossing.portal.contact.manifoldContact);
assert.match(blockedCrossing.portal.contact.id, /distal-rim/);
assert.equal(blockedCrossing.activeContacts.length, 1,
    'the distal plane must be owned by the rim rather than double-counted as side contact');
rimManifold.endStep();

// Even when both inner endpoints are individually on opposite sides of the
// distal plane, the exact plane intersection prevents a segment from cutting
// through the catheter side wall between quadrature nodes.
const sideEscape = evaluateKirchhoffLumenSegmentContact({
    innerStart: [8, 0, 0],
    innerEnd: [10.5, 2, 0],
    outerStart: [0, 0, 0],
    outerEnd: [10, 0, 0],
    lumenRadius: 1,
    innerRadius: 0.2,
    innerMaterialSegmentId: 'wire-side-escape',
    outerMaterialSegmentId: 'catheter-tip',
    openDistal: true
});
assert.equal(sideEscape.portal.crosses, true);
assert.equal(sideEscape.portal.valid, false);
assert.ok(sideEscape.portal.radialDistance > sideEscape.clearance);
assert.ok(sideEscape.portal.contact.violation > 0.79);

console.log('Kirchhoff lumen-contact tests passed');
