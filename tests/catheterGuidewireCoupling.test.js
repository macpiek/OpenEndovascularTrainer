import assert from 'node:assert/strict';
import { RodState } from '../src/physics/rodState.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    buildContainedGuidewireRenderPolyline,
    firstFreeGuidewireNodeAfterContainment,
    spatiallyCapturedContainmentEnd
} from '../src/physics/catheterGuidewireCoupling.js';

import { PigtailCatheter } from '../src/pigtailCatheter.js';

assert.equal(
    firstFreeGuidewireNodeAfterContainment({
        activeStart: 85,
        activeEnd: 180,
        containmentEndNode: 112
    }),
    113,
    'projection damping must end at the lumen portal, not at the end of a broader external-contact window'
);
assert.equal(
    firstFreeGuidewireNodeAfterContainment({
        activeStart: 85,
        activeEnd: 180,
        containmentEndNode: 180
    }),
    181,
    'a fully contained wire may keep its entire active span under lumen projection damping'
);

// A material boundary may advance past more than one guidewire node before
// the moving catheter has physically surrounded them. Spatial classification
// must stop at the first non-contained node instead of accepting a later node
// that happens to be close to the catheter again.
const captureOuter = {
    x: new Float32Array([0, 1, 2, 3]),
    y: new Float32Array([0, 0, 0, 0]),
    z: new Float32Array(4),
    activeStart: 0,
    activeEnd: 3,
    innerRadius: 0.5
};
const captureInner = {
    x: new Float32Array([0.25, 1.25, 2.25, 2.75]),
    y: new Float32Array([0, 0, 0.35, 0]),
    z: new Float32Array(4),
    activeStart: 0,
    activeEnd: 3,
    radius: 0.4
};
const captureEnd = spatiallyCapturedContainmentEnd({
    innerBody: captureInner,
    outerBody: captureOuter,
    firstContainedNode: 0,
    materialEndNode: 3,
    closestSegment: new Int32Array([0, 1, 2, 2])
});
assert.equal(captureEnd, 1,
    'spatial containment must remain contiguous at a moving catheter tip');
const degenerateCaptureEnd = spatiallyCapturedContainmentEnd({
    innerBody: captureInner,
    outerBody: {
        ...captureOuter,
        x: new Float32Array(4),
        activeEnd: 1
    },
    firstContainedNode: 0,
    materialEndNode: 1
});
assert.equal(degenerateCaptureEnd, -1,
    'a zero-length catheter tip must not classify an arbitrary captured node');

// The visible contained wire uses the catheter's own vertices and an axial
// guide point beyond the distal opening. It must not draw a sparse straight
// chord through the side wall of a curved catheter.
const renderOuter = {
    x: new Float32Array([0, 1, 2, 3]),
    y: new Float32Array([0, 0, 1, 1]),
    z: new Float32Array(4),
    activeStart: 0,
    activeEnd: 3
};
const renderContainment = {
    enabled: true,
    startNode: 1,
    endNode: 3,
    closestSegment: new Int32Array([-1, 0, 1, 2, -1]),
    closestT: new Float32Array([0, 0.25, 0.5, 0.75, 0])
};
const renderGuidewire = [
    { x: -1, y: 0, z: 0 },
    { x: 0.25, y: 0, z: 0 },
    { x: 1.5, y: 0.5, z: 0 },
    { x: 2.75, y: 1, z: 0 },
    { x: 4, y: 2, z: 0 }
];
const renderPath = buildContainedGuidewireRenderPolyline({
    guidewireNodes: renderGuidewire,
    outerBody: renderOuter,
    containment: renderContainment
});
assert.ok(renderPath.containedStartIndex >= 0);
assert.ok(renderPath.containedEndIndex > renderPath.containedStartIndex);
assert.deepEqual(renderPath[renderPath.containedEndIndex], { x: 3, y: 1, z: 0 },
    'the contained render path must reach the real distal opening');
assert.ok(renderPath[renderPath.containedEndIndex + 1].x > 3,
    'the render tangent must continue axially beyond the distal opening');

// When the catheter has advanced beyond the guidewire tip, spatial capture
// may intentionally leave renderEndNode one node behind endNode. That trailing
// render node is still inside the catheter; it must not be mistaken for a
// distal external span and stitched to the catheter opening and back.
const overtakenOuter = {
    x: new Float32Array([0, 1, 2, 3, 4, 5]),
    y: new Float32Array(6),
    z: new Float32Array(6),
    activeStart: 0,
    activeEnd: 5
};
const overtakenGuidewire = [
    { x: -1, y: 0, z: 0 },
    { x: 0.25, y: 0, z: 0 },
    { x: 1.5, y: 0, z: 0 },
    { x: 2.75, y: 0, z: 0 },
    { x: 3.25, y: 0, z: 0 }
];
const overtakenPath = buildContainedGuidewireRenderPolyline({
    guidewireNodes: overtakenGuidewire,
    outerBody: overtakenOuter,
    containment: {
        enabled: true,
        startNode: 1,
        endNode: 4,
        renderEndNode: 3,
        closestSegment: new Int32Array([-1, 0, 1, 2, 3]),
        closestT: new Float32Array([0, 0.25, 0.5, 0.75, 0.25])
    }
});
assert.deepEqual(overtakenPath.at(-1), overtakenGuidewire.at(-1),
    'a guidewire ending inside the catheter must end at its own physical tip');
assert.ok(
    overtakenPath.every(point => point.x <= overtakenGuidewire.at(-1).x),
    'an overtaken guidewire must not be rendered out to the catheter opening'
);

// Perturbing the supporting wire must never rewrite catheter rest strain.
const feedbackLength = 180;
const feedbackSpacing = 2;
const feedbackInserted = 120;
const feedbackSheath = {
    start: { x: -20, y: 0, z: 0 },
    end: { x: 0, y: 0, z: 0 }
};
const feedbackWire = new RodState(
    feedbackLength / feedbackSpacing + 1,
    feedbackSpacing
);
for (let index = 0; index < feedbackWire.nodes.length; index++) {
    const node = feedbackWire.nodes[index];
    node.x = index * feedbackSpacing - feedbackLength + feedbackInserted - 20;
    node.y = 0;
    node.z = 0;
}
const feedbackCatheter = new PigtailCatheter({
    wire: feedbackWire,
    segmentLength: feedbackSpacing,
    guidewireLength: feedbackLength,
    tailProgressRef: () => feedbackInserted,
    vessel: { sheath: feedbackSheath, segments: [] },
    maxLength: feedbackLength
});
feedbackCatheter.setType('berenstein');

const feedbackWorld = new EndovascularPhysicsWorld();
const feedbackBody = feedbackWorld.createRod(
    'feedback-catheter',
    128,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
for (let step = 0; step < 210; step++) {
    feedbackCatheter.advance(1, 1 / 120, feedbackInserted);
    feedbackCatheter.stepPhysics(1 / 120);
    feedbackCatheter.syncXpbdBody(feedbackBody);
}
for (let step = 0; step < 60; step++) {
    feedbackCatheter.advance(0, 1 / 120, feedbackInserted);
    feedbackCatheter.stepPhysics(1 / 120);
    feedbackCatheter.syncXpbdBody(feedbackBody);
}
const feedbackTargetX = feedbackBody.restRotation1.slice();
const feedbackTargetY = feedbackBody.restRotation2.slice();
const feedbackTargetZ = feedbackBody.restRotation3.slice();
for (let index = 0; index < feedbackWire.nodes.length; index++) {
    const node = feedbackWire.nodes[index];
    node.y += index & 1 ? 18 : -18;
    node.z += index % 3 ? 12 : -12;
}
feedbackCatheter.advance(0, 1 / 120, feedbackInserted);
feedbackCatheter.stepPhysics(1 / 120);
feedbackCatheter.syncXpbdBody(feedbackBody);
let maximumFeedbackTargetShift = 0;
let feedbackTargetCount = 0;
for (let index = feedbackBody.activeStart; index <= feedbackBody.activeEnd; index++) {
    if (index <= feedbackBody.activeStart || index >= feedbackBody.activeEnd) continue;
    feedbackTargetCount++;
    maximumFeedbackTargetShift = Math.max(
        maximumFeedbackTargetShift,
        Math.hypot(
            feedbackBody.restRotation1[index] - feedbackTargetX[index],
            feedbackBody.restRotation2[index] - feedbackTargetY[index],
            feedbackBody.restRotation3[index] - feedbackTargetZ[index]
        )
    );
}
console.log('recorded catheter target shift after live wire perturbation mm',
    maximumFeedbackTargetShift.toFixed(6));
assert.ok(feedbackTargetCount >= 8,
    'the feedback regression must inspect deployed catheter shape targets');
assert.ok(maximumFeedbackTargetShift <= 1e-4,
    `live guidewire deformation must not rewrite existing catheter material targets (${maximumFeedbackTargetShift} mm)`);
feedbackCatheter.dispose();

// With material-coordinate feeding, removing one active catheter node during
// withdrawal happens at the sheath entry. The physical distal tip must survive
// the topology change instead of being deleted from a recorded route.
const guidewireLength = 180;
const guidewireSpacing = 2;
let guidewireInserted = 120;
const interactionWire = new RodState(
    guidewireLength / guidewireSpacing + 1,
    guidewireSpacing
);
for (let index = 0; index < interactionWire.nodes.length; index++) {
    const node = interactionWire.nodes[index];
    node.x = index * guidewireSpacing - guidewireLength + guidewireInserted - 20;
    node.y = 0;
    node.z = 0;
    node.vx = 0;
    node.vy = 0;
    node.vz = 0;
}
const materialCatheter = new PigtailCatheter({
    wire: interactionWire,
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
    maxLength: guidewireLength
});

const materialBodyWorld = new EndovascularPhysicsWorld();
const materialBody = materialBodyWorld.createRod('material-catheter', 128, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
for (let step = 0; step < 220; step++) {
    materialCatheter.advance(1, 1 / 120, guidewireInserted);
    materialCatheter.stepPhysics(1 / 120);
    materialCatheter.syncXpbdBody(materialBody);
}
const insertedCount = materialCatheter.physicsActiveCount;
assert.ok(insertedCount > 20, 'the material-coordinate fixture should deploy a catheter');
materialBody.y[materialBody.activeEnd] = 6;
materialBody.previousY[materialBody.activeEnd] = 6;
let withdrawnCount = insertedCount;
for (let step = 0; step < 80 && withdrawnCount >= insertedCount; step++) {
    materialCatheter.advance(-1, 1 / 120, guidewireInserted);
    materialCatheter.stepPhysics(1 / 120);
    withdrawnCount = materialCatheter.syncXpbdBody(materialBody);
}
assert.equal(withdrawnCount, insertedCount - 1,
    'the withdrawal fixture should cross exactly one catheter topology boundary');
assert.ok(
    materialBody.y[materialBody.activeEnd] > 5.5,
    'withdrawal must preserve the physical distal tip state when a proximal node leaves the sheath'
);
materialCatheter.dispose();

console.log('catheter-guidewire coupling regression tests passed');
