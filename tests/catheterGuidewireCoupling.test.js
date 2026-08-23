import assert from 'node:assert/strict';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import {
    buildContainedGuidewireRenderPolyline,
    firstFreeGuidewireNodeAfterContainment,
    spatiallyCapturedContainmentEnd
} from '../src/physics/catheterGuidewireCoupling.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';
import { applyGuidewireMaterialProfile } from '../src/physics/guidewireMaterialProfile.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';

const EPSILON = 1e-8;

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

function setRodPoint(body, index, x, y, z = 0) {
    body.setNodePosition(index, x, y, z);
}

function portalCrossingState(inner, segment, outer) {
    segment = Math.max(inner.activeStart, Math.min(inner.activeEnd - 1, segment));
    const tipIndex = outer.activeEnd;
    const tipX = outer.x[tipIndex];
    const tipY = outer.y[tipIndex];
    const tipZ = outer.z[tipIndex];
    let tangentX = tipX - outer.x[tipIndex - 1];
    let tangentY = tipY - outer.y[tipIndex - 1];
    let tangentZ = tipZ - outer.z[tipIndex - 1];
    const tangentLength = Math.hypot(tangentX, tangentY, tangentZ);
    tangentX /= tangentLength;
    tangentY /= tangentLength;
    tangentZ /= tangentLength;
    const aX = inner.x[segment] - tipX;
    const aY = inner.y[segment] - tipY;
    const aZ = inner.z[segment] - tipZ;
    const bX = inner.x[segment + 1] - tipX;
    const bY = inner.y[segment + 1] - tipY;
    const bZ = inner.z[segment + 1] - tipZ;
    const aAxial = aX * tangentX + aY * tangentY + aZ * tangentZ;
    const bAxial = bX * tangentX + bY * tangentY + bZ * tangentZ;
    const t = Math.max(0, Math.min(1, -aAxial / Math.max(EPSILON, bAxial - aAxial)));
    const crossingX = aX + (bX - aX) * t;
    const crossingY = aY + (bY - aY) * t;
    const crossingZ = aZ + (bZ - aZ) * t;
    const axial = crossingX * tangentX + crossingY * tangentY + crossingZ * tangentZ;
    return {
        radius: Math.hypot(
            crossingX - tangentX * axial,
            crossingY - tangentY * axial,
            crossingZ - tangentZ * axial
        ),
        straddles: aAxial <= 0 && bAxial >= 0,
        axialDistance: Math.abs(axial)
    };
}

function portalCrossingRadius(inner, segment, outer) {
    return portalCrossingState(inner, segment, outer).radius;
}

function portalExitAngleDegrees(inner, segment, outer) {
    segment = Math.max(inner.activeStart, Math.min(inner.activeEnd - 1, segment));
    const tipIndex = outer.activeEnd;
    const tangentX = outer.x[tipIndex] - outer.x[tipIndex - 1];
    const tangentY = outer.y[tipIndex] - outer.y[tipIndex - 1];
    const tangentZ = outer.z[tipIndex] - outer.z[tipIndex - 1];
    const wireX = inner.x[segment + 1] - inner.x[segment];
    const wireY = inner.y[segment + 1] - inner.y[segment];
    const wireZ = inner.z[segment + 1] - inner.z[segment];
    const denominator = Math.max(
        EPSILON,
        Math.hypot(tangentX, tangentY, tangentZ) *
            Math.hypot(wireX, wireY, wireZ)
    );
    return Math.acos(Math.max(-1, Math.min(1,
        (tangentX * wireX + tangentY * wireY + tangentZ * wireZ) /
            denominator
    ))) * 180 / Math.PI;
}

function bodyShapeMetrics(body, segmentStart = body.activeStart) {
    let maximumSegmentError = 0;
    let maximumSegmentErrorNode = -1;
    let maximumBend = 0;
    let maximumBendNode = -1;
    const firstSegment = Math.max(body.activeStart, segmentStart);
    for (let index = firstSegment; index < body.activeEnd; index++) {
        const ax = body.x[index + 1] - body.x[index];
        const ay = body.y[index + 1] - body.y[index];
        const az = body.z[index + 1] - body.z[index];
        const segmentLength = Math.hypot(ax, ay, az);
        const segmentError = Math.abs(segmentLength - body.restLength[index]);
        if (segmentError > maximumSegmentError) {
            maximumSegmentError = segmentError;
            maximumSegmentErrorNode = index;
        }
        if (index <= firstSegment || index >= body.activeEnd) continue;
        const bx = body.x[index] - body.x[index - 1];
        const by = body.y[index] - body.y[index - 1];
        const bz = body.z[index] - body.z[index - 1];
        const cosine = Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) /
                Math.max(EPSILON, segmentLength * Math.hypot(bx, by, bz))
        ));
        const bend = Math.acos(cosine) * 180 / Math.PI;
        if (bend > maximumBend) {
            maximumBend = bend;
            maximumBendNode = index;
        }
    }
    return {
        maximumSegmentError,
        maximumSegmentErrorNode,
        maximumBend,
        maximumBendNode
    };
}

function distanceToBodySegments(pointBody, pointIndex, centerlineBody, startSegment, endSegment) {
    let minimumDistance = Infinity;
    const px = pointBody.x[pointIndex];
    const py = pointBody.y[pointIndex];
    const pz = pointBody.z[pointIndex];
    for (let segment = startSegment; segment <= endSegment; segment++) {
        const ax = centerlineBody.x[segment];
        const ay = centerlineBody.y[segment];
        const az = centerlineBody.z[segment];
        const dx = centerlineBody.x[segment + 1] - ax;
        const dy = centerlineBody.y[segment + 1] - ay;
        const dz = centerlineBody.z[segment + 1] - az;
        const lengthSquared = dx * dx + dy * dy + dz * dz;
        const t = Math.max(0, Math.min(1,
            ((px - ax) * dx + (py - ay) * dy + (pz - az) * dz) /
                Math.max(EPSILON, lengthSquared)
        ));
        minimumDistance = Math.min(
            minimumDistance,
            Math.hypot(px - (ax + dx * t), py - (ay + dy * t), pz - (az + dz * t))
        );
    }
    return minimumDistance;
}

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

// The default catheter must dominate the wire inside its lumen. The former
// application setup projected the catheter onto the guidewire instead.
const dominanceWorld = new EndovascularPhysicsWorld({ iterations: 8 });
const dominantCatheter = dominanceWorld.createRod('dominant-catheter', 13, 1, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
const shapedWire = dominanceWorld.createRod('shaped-wire', 13, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    bendCompliance: 5e-4
});
for (let index = 0; index < dominantCatheter.count; index++) {
    const x = index;
    const y = Math.sin(index / (dominantCatheter.count - 1) * Math.PI) * 1.6;
    setRodPoint(dominantCatheter, index, x, y);
    setRodPoint(shapedWire, index, x, 0);
}
dominantCatheter.captureRestConfiguration();
shapedWire.captureRestConfiguration();
for (let index = 0; index < dominantCatheter.count; index++) {
    dominantCatheter.setRestShapeTarget(
        index,
        dominantCatheter.x[index],
        dominantCatheter.y[index],
        dominantCatheter.z[index],
        1e-7
    );
}
const initialCatheterMidY = dominantCatheter.y[6];
dominanceWorld.addContainment(shapedWire, dominantCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    openProximal: false,
    openDistal: false,
    searchWindow: dominantCatheter.segmentCount,
    innerResponse: 1,
    outerResponse: 0.08,
    finalProjection: 'inner'
});
for (let step = 0; step < 48; step++) dominanceWorld.stepFixed();
assert.ok(
    Math.abs(shapedWire.y[6] - dominantCatheter.y[6]) < 0.06,
    'the guidewire should be shaped by the catheter lumen'
);
assert.ok(
    dominantCatheter.y[6] > initialCatheterMidY * 0.8,
    'the catheter should retain most of its shape instead of yielding to the guidewire'
);

// Once catheter material has entered, its rest route is material history, not
// a live sample of the guidewire that containment has just projected into it.
// Otherwise wire -> catheter target -> wire forms a hidden two-way feedback
// loop even when positional containment itself has outerResponse = 0.
const feedbackLength = 180;
const feedbackSpacing = 2;
const feedbackInserted = 120;
const feedbackSheath = {
    start: { x: -20, y: 0, z: 0 },
    end: { x: 0, y: 0, z: 0 }
};
const feedbackWire = new ElasticRod(
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
feedbackCatheter.setExternalCollisionSolver(true);
const feedbackWorld = new EndovascularPhysicsWorld();
const feedbackBody = feedbackWorld.createRod(
    'feedback-catheter',
    128,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
for (let step = 0; step < 210; step++) {
    feedbackCatheter.advance(1, 1 / 120, feedbackInserted);
    feedbackCatheter.stepPhysics(1 / 120, { collisions: false });
    feedbackCatheter.syncXpbdBody(feedbackBody);
}
for (let step = 0; step < 60; step++) {
    feedbackCatheter.advance(0, 1 / 120, feedbackInserted);
    feedbackCatheter.stepPhysics(1 / 120, { collisions: false });
    feedbackCatheter.syncXpbdBody(feedbackBody);
}
const feedbackTargetX = feedbackBody.restShapeX.slice();
const feedbackTargetY = feedbackBody.restShapeY.slice();
const feedbackTargetZ = feedbackBody.restShapeZ.slice();
for (let index = 0; index < feedbackWire.nodes.length; index++) {
    const node = feedbackWire.nodes[index];
    node.y += index & 1 ? 18 : -18;
    node.z += index % 3 ? 12 : -12;
}
feedbackCatheter.advance(0, 1 / 120, feedbackInserted);
feedbackCatheter.stepPhysics(1 / 120, { collisions: false });
feedbackCatheter.syncXpbdBody(feedbackBody);
let maximumFeedbackTargetShift = 0;
let feedbackTargetCount = 0;
for (let index = feedbackBody.activeStart; index <= feedbackBody.activeEnd; index++) {
    if (!feedbackBody.restShapeEnabled[index]) continue;
    feedbackTargetCount++;
    maximumFeedbackTargetShift = Math.max(
        maximumFeedbackTargetShift,
        Math.hypot(
            feedbackBody.restShapeX[index] - feedbackTargetX[index],
            feedbackBody.restShapeY[index] - feedbackTargetY[index],
            feedbackBody.restShapeZ[index] - feedbackTargetZ[index]
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

// Catheter and guidewire form a supported composite beam over their shared
// span. The catheter must be stiffer there than immediately distal to the
// guidewire, with a finite taper rather than a discontinuous hinge.
const compositeWireInserted = 90;
const compositeCatheter = new PigtailCatheter({
    wire: feedbackWire,
    segmentLength: feedbackSpacing,
    guidewireLength: feedbackLength,
    tailProgressRef: () => compositeWireInserted,
    vessel: { sheath: feedbackSheath, segments: [] },
    maxLength: feedbackLength
});
compositeCatheter.setType('berenstein');
compositeCatheter.setExternalCollisionSolver(true);
const compositeWorld = new EndovascularPhysicsWorld();
const compositeBody = compositeWorld.createRod(
    'composite-profile-catheter',
    128,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
for (let step = 0; step < 230; step++) {
    compositeCatheter.advance(1, 1 / 120, compositeWireInserted);
    compositeCatheter.stepPhysics(1 / 120, { collisions: false });
    compositeCatheter.syncXpbdBody(compositeBody);
}
let supportedCompliance = Infinity;
let unsupportedCompliance = 0;
for (let index = 0; index < compositeCatheter.physicsActiveCount; index++) {
    const distance = compositeCatheter._centerlineDistances[index];
    if (distance <= compositeWireInserted - 10) {
        supportedCompliance = Math.min(
            supportedCompliance,
            compositeBody.bendComplianceByNode[index]
        );
    }
    if (distance >= compositeWireInserted + 8) {
        unsupportedCompliance = Math.max(
            unsupportedCompliance,
            compositeBody.bendComplianceByNode[index]
        );
    }
}
assert.ok(Number.isFinite(supportedCompliance) && unsupportedCompliance > 0,
    'the composite stiffness regression must sample supported and unsupported catheter spans');
assert.ok(supportedCompliance <= DEFAULT_TOOL_PROFILES.catheter.bendCompliance / 1.95,
    `the guide-supported catheter must gain composite bending stiffness (${supportedCompliance})`);
assert.ok(unsupportedCompliance > supportedCompliance * 1.05,
    `the unsupported distal catheter must remain more compliant (${unsupportedCompliance})`);
compositeCatheter.dispose();

// The wire segment transitioning from the lumen to free space must cross the
// distal opening. Node-only containment lets this segment cut through the side.
const portalWorld = new EndovascularPhysicsWorld({ iterations: 10 });
const portalCatheter = portalWorld.createRod('portal-catheter', 5, 2, {
    ...DEFAULT_TOOL_PROFILES.catheter,
    stretchCompliance: 0,
    bendCompliance: 0
});
const portalWire = portalWorld.createRod('portal-wire', 6, 2, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    stretchCompliance: 0,
    bendCompliance: 1
});
for (let index = 0; index < portalCatheter.count; index++) {
    setRodPoint(portalCatheter, index, index * 2, 0);
    portalCatheter.setPinned(index, true);
}
setRodPoint(portalWire, 0, 5, 0);
setRodPoint(portalWire, 1, 7, 0);
setRodPoint(portalWire, 2, 9, 1.2);
setRodPoint(portalWire, 3, 11, 2.4);
setRodPoint(portalWire, 4, 13, 3.6);
setRodPoint(portalWire, 5, 15, 4.8);
portalWire.captureRestConfiguration();
const portalContainment = portalWorld.addContainment(portalWire, portalCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    openProximal: false,
    openDistal: true,
    searchWindow: portalCatheter.segmentCount,
    startNode: 0,
    endNode: 1,
    innerArcOffset: 5,
    innerResponse: 1,
    outerResponse: 0,
    finalProjection: 'inner',
    enforceDistalPortal: true,
    portalTransitionLength: 4
});
for (let step = 0; step < 24; step++) portalWorld.stepFixed();
const allowedPortalRadius =
    DEFAULT_TOOL_PROFILES.catheter.innerRadius - DEFAULT_TOOL_PROFILES.guidewire.radius;
const crossingRadius = portalCrossingRadius(portalWire, 1, portalCatheter);
assert.ok(
    crossingRadius <= allowedPortalRadius + 0.01,
    `the guidewire must emerge through the distal opening (${crossingRadius} mm radial crossing)`
);
const portalExitAngle = portalExitAngleDegrees(portalWire, 1, portalCatheter);
const physicalExitConeDegrees = Math.atan2(
    allowedPortalRadius * 2,
    portalContainment.portalTransitionLength
) * 180 / Math.PI;
assert.ok(
    portalExitAngle <= physicalExitConeDegrees + 0.3,
    `the crossing guidewire segment must inherit the catheter-tip direction ` +
        `(${portalExitAngle} degrees, physical cone ${physicalExitConeDegrees} degrees)`
);
const portalWireShape = bodyShapeMetrics(portalWire);
assert.ok(
    portalWireShape.maximumSegmentError <= 0.02,
    `the portal moment must preserve the continuous wire length ` +
        `(${portalWireShape.maximumSegmentError} mm at segment ` +
        `${portalWireShape.maximumSegmentErrorNode})`
);
assert.ok(
    portalWireShape.maximumBend <= 30.5,
    `the catheter opening must transmit its moment into the continuous wire ` +
        `without moving the kink one node downstream ` +
        `(${portalWireShape.maximumBend} degrees at node ` +
        `${portalWireShape.maximumBendNode})`
);

// Reproduce the clinical choreography instead of constructing only its final
// geometry: insert a short guidewire, stop it, then advance the catheter past
// the stationary wire tip. The distal lumen transition must not kick or fold
// the catheter as the guidewire changes from external contact to containment.
const catchUpGuidewireLength = 360;
const catchUpGuidewireSpacing = 2;
const catchUpGuidewireInserted = 189;
const catchUpSheath = {
    start: { x: -20, y: 0, z: 0 },
    end: { x: 0, y: 0, z: 0 }
};
const catchUpWire = new ElasticRod(
    catchUpGuidewireLength / catchUpGuidewireSpacing + 1,
    catchUpGuidewireSpacing
);
applyGuidewireMaterialProfile(catchUpWire, {
    segmentLength: catchUpGuidewireSpacing
});
for (let index = 0; index < catchUpWire.nodes.length; index++) {
    const node = catchUpWire.nodes[index];
    const distalWeight = Math.max(0, (index - (catchUpWire.nodes.length - 7)) / 6);
    node.x = index * catchUpGuidewireSpacing - catchUpGuidewireLength +
        catchUpGuidewireInserted - 20;
    node.y = distalWeight * distalWeight * 1.2;
    node.z = 0;
    node.vx = 0;
    node.vy = 0;
    node.vz = 0;
}
const catchUpCatheter = new PigtailCatheter({
    wire: catchUpWire,
    segmentLength: catchUpGuidewireSpacing,
    guidewireLength: catchUpGuidewireLength,
    tailProgressRef: () => catchUpGuidewireInserted,
    vessel: { sheath: catchUpSheath, segments: [] },
    maxLength: catchUpGuidewireLength
});
catchUpCatheter.setType('berenstein');
catchUpCatheter.setExternalCollisionSolver(true);
const catchUpWorld = new EndovascularPhysicsWorld({ iterations: 6 });
const catchUpWireBody = catchUpWorld.createRod(
    'short-stationary-guidewire',
    catchUpWire.nodes.length,
    catchUpGuidewireSpacing,
    { ...DEFAULT_TOOL_PROFILES.guidewire }
);
catchUpWireBody.syncFromElasticRod(catchUpWire);
catchUpWireBody.setActiveRange(
    Math.max(0, Math.ceil(
        (catchUpGuidewireLength - catchUpGuidewireInserted) / catchUpGuidewireSpacing
    ) - 1),
    catchUpWireBody.count - 1
);
const catchUpCatheterBody = catchUpWorld.createRod(
    'overtaking-catheter',
    128,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
catchUpWorld.addSheath({
    start: catchUpSheath.start,
    end: catchUpSheath.end,
    innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
    bodies: [catchUpWireBody, catchUpCatheterBody]
});
const catchUpContainment = catchUpWorld.addContainment(
    catchUpWireBody,
    catchUpCatheterBody,
    {
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        openProximal: true,
        openDistal: true,
        searchWindow: 2,
        innerResponse: 1,
        outerResponse: 0,
        portalInnerResponse: 1,
        portalOuterResponse: 0,
        portalCompliance: 1e-7,
        portalTransitionLength: 4,
        portalMaxCorrection: 0.15,
        finalProjection: 'inner',
        outerFollowsInnerCenterline: false,
        innerFollowsOuterCenterline: true,
        enforceDistalPortal: false,
        enabled: false
    }
);
const catchUpExternalContact = catchUpWorld.addToolContact(
    catchUpWireBody,
    catchUpCatheterBody,
    {
        friction: 0.08,
        openDistalB: true,
        enabled: false
    }
);
let catchUpPreviousTip = null;
let catchUpMaximumTipStep = 0;
let catchUpMaximumTipStepAt = -1;
let catchUpMaximumBend = 0;
let catchUpMaximumBendAt = -1;
let catchUpMaximumBendNode = -1;
let catchUpMaximumSegmentError = 0;
let catchUpMaximumSegmentErrorAt = -1;
let catchUpMaximumSegmentErrorNode = -1;
let catchUpMaximumSegmentErrorCollisionStart = -1;
let catchUpMaximumWireSegmentError = 0;
let catchUpMaximumWireSegmentErrorAt = -1;
let catchUpMaximumWireSegmentErrorNode = -1;
let catchUpMaximumWireBend = 0;
let catchUpMaximumWireBendAt = -1;
let catchUpMaximumWireBendNode = -1;
let catchUpSawExternalContact = false;
let catchUpSawContainedOnly = false;
for (let step = 0; step < 720; step++) {
    catchUpCatheter.advance(1, 1 / 120, catchUpGuidewireInserted);
    catchUpWireBody.syncFromElasticRod(catchUpWire, { preservePrevious: true });
    catchUpCatheter.stepPhysics(1 / 120, { collisions: false });
    const activeCount = catchUpCatheter.syncXpbdBody(catchUpCatheterBody);
    const firstContainedNode = Math.max(0, Math.ceil(
        (catchUpGuidewireLength - catchUpGuidewireInserted) / catchUpGuidewireSpacing
    ));
    const materialEndNode = Math.min(
        catchUpWireBody.count - 1,
        Math.floor(
            (catchUpGuidewireLength - catchUpGuidewireInserted + catchUpCatheter.progress) /
            catchUpGuidewireSpacing
        )
    );
    catchUpContainment.outerStartNode = catchUpCatheter.physicsLumenStartNode;
    const lastContainedNode = materialEndNode;
    catchUpContainment.enabled =
        catchUpCatheter.progress > 0.5 &&
        activeCount >= 2 &&
        lastContainedNode >= firstContainedNode;
    catchUpContainment.startNode = firstContainedNode;
    catchUpContainment.endNode = Math.max(firstContainedNode, lastContainedNode);
    catchUpContainment.innerArcOffset =
        firstContainedNode * catchUpGuidewireSpacing -
        catchUpGuidewireLength + catchUpGuidewireInserted;
    catchUpContainment.containedLength = Math.min(
        catchUpCatheter.progress,
        catchUpGuidewireInserted
    );
    catchUpContainment.enforceDistalPortal = true;
    catchUpContainment.portalInnerResponse = 0;
    catchUpContainment.portalOuterResponse = 0.2;
    catchUpContainment.limitDistalCorrection = true;
    catchUpContainment.preserveStationaryInnerLength = true;
    const catheterEndSegment = Math.max(0, activeCount - 2);
    const firstExternalSegment = Math.max(0, Math.min(
        catchUpWireBody.segmentCount - 1,
        lastContainedNode + 1
    ));
    catchUpExternalContact.enabled =
        catchUpCatheter.progress > 4 &&
        activeCount >= 2 &&
        catchUpGuidewireInserted > catchUpCatheter.progress + 0.5 &&
        firstExternalSegment <= catchUpWireBody.activeEnd - 1;
    catchUpExternalContact.startSegmentA = firstExternalSegment;
    catchUpExternalContact.endSegmentA = Math.min(
        catchUpWireBody.activeEnd - 1,
        firstExternalSegment + 16
    );
    catchUpExternalContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
    catchUpExternalContact.endSegmentB = catheterEndSegment;
    catchUpSawExternalContact ||= catchUpExternalContact.enabled;
    catchUpSawContainedOnly ||=
        catchUpCatheter.progress > catchUpGuidewireInserted + 1 &&
        catchUpContainment.enabled &&
        !catchUpExternalContact.enabled;
    catchUpWorld.stepFixed();
    catchUpWireBody.syncToElasticRod(catchUpWire);

    const tipIndex = catchUpCatheterBody.activeEnd;
    const tip = [
        catchUpCatheterBody.x[tipIndex],
        catchUpCatheterBody.y[tipIndex],
        catchUpCatheterBody.z[tipIndex]
    ];
    if (catchUpPreviousTip && catchUpCatheter.progress >= 60) {
        const tipStep = Math.hypot(
            tip[0] - catchUpPreviousTip[0],
            tip[1] - catchUpPreviousTip[1],
            tip[2] - catchUpPreviousTip[2]
        );
        if (tipStep > catchUpMaximumTipStep) {
            catchUpMaximumTipStep = tipStep;
            catchUpMaximumTipStepAt = step;
        }
    }
    catchUpPreviousTip = tip;
    if (catchUpCatheter.progress >= 60) {
        const stats = catchUpWorld.getStats().bodies.find(
            body => body.id === 'overtaking-catheter'
        );
        if (stats.maxBendAngleDegrees > catchUpMaximumBend) {
            catchUpMaximumBend = stats.maxBendAngleDegrees;
            catchUpMaximumBendAt = step;
            catchUpMaximumBendNode = bodyShapeMetrics(
                catchUpCatheterBody
            ).maximumBendNode;
        }
        for (let index = catchUpCatheterBody.activeStart; index < tipIndex; index++) {
            const segmentLength = Math.hypot(
                catchUpCatheterBody.x[index + 1] - catchUpCatheterBody.x[index],
                catchUpCatheterBody.y[index + 1] - catchUpCatheterBody.y[index],
                catchUpCatheterBody.z[index + 1] - catchUpCatheterBody.z[index]
            );
            const segmentError = Math.abs(
                segmentLength - catchUpCatheterBody.restLength[index]
            );
            if (segmentError > catchUpMaximumSegmentError) {
                catchUpMaximumSegmentError = segmentError;
                catchUpMaximumSegmentErrorAt = step;
                catchUpMaximumSegmentErrorNode = index;
                catchUpMaximumSegmentErrorCollisionStart =
                    catchUpCatheterBody.collisionStartSegment;
            }
        }
        for (let index = catchUpWireBody.activeStart; index < catchUpWireBody.activeEnd; index++) {
            const segmentLength = Math.hypot(
                catchUpWireBody.x[index + 1] - catchUpWireBody.x[index],
                catchUpWireBody.y[index + 1] - catchUpWireBody.y[index],
                catchUpWireBody.z[index + 1] - catchUpWireBody.z[index]
            );
            const wireSegmentError = Math.abs(
                segmentLength - catchUpWireBody.restLength[index]
            );
            if (wireSegmentError > catchUpMaximumWireSegmentError) {
                catchUpMaximumWireSegmentError = wireSegmentError;
                catchUpMaximumWireSegmentErrorAt = step;
                catchUpMaximumWireSegmentErrorNode = index;
            }
            if (index <= catchUpWireBody.activeStart || index >= catchUpWireBody.activeEnd) continue;
            const incomingX = catchUpWireBody.x[index] - catchUpWireBody.x[index - 1];
            const incomingY = catchUpWireBody.y[index] - catchUpWireBody.y[index - 1];
            const incomingZ = catchUpWireBody.z[index] - catchUpWireBody.z[index - 1];
            const outgoingX = catchUpWireBody.x[index + 1] - catchUpWireBody.x[index];
            const outgoingY = catchUpWireBody.y[index + 1] - catchUpWireBody.y[index];
            const outgoingZ = catchUpWireBody.z[index + 1] - catchUpWireBody.z[index];
            const denominator = Math.max(
                EPSILON,
                Math.hypot(incomingX, incomingY, incomingZ) *
                    Math.hypot(outgoingX, outgoingY, outgoingZ)
            );
            const cosine = Math.max(-1, Math.min(1,
                (incomingX * outgoingX + incomingY * outgoingY + incomingZ * outgoingZ) /
                    denominator
            ));
            const wireBend = Math.acos(cosine) * 180 / Math.PI;
            if (wireBend > catchUpMaximumWireBend) {
                catchUpMaximumWireBend = wireBend;
                catchUpMaximumWireBendAt = step;
                catchUpMaximumWireBendNode = index;
            }
        }
    }
}
console.log('18.9/31.2 catch-up max tip step mm', catchUpMaximumTipStep.toFixed(4),
    'at', catchUpMaximumTipStepAt);
console.log('18.9/31.2 catch-up max bend degrees', catchUpMaximumBend.toFixed(2),
    'at', catchUpMaximumBendAt, 'node', catchUpMaximumBendNode,
    'limit', catchUpCatheterBody.maxBendAngleByNode[catchUpMaximumBendNode],
    'intrinsic', catchUpCatheterBody.intrinsicBendEnabled[catchUpMaximumBendNode]);
console.log('18.9/31.2 catch-up max segment error mm', catchUpMaximumSegmentError.toFixed(4),
    'at', catchUpMaximumSegmentErrorAt, 'node', catchUpMaximumSegmentErrorNode,
    'collisionStart', catchUpMaximumSegmentErrorCollisionStart);
console.log('18.9/31.2 catch-up max guidewire segment error mm',
    catchUpMaximumWireSegmentError.toFixed(4), 'at', catchUpMaximumWireSegmentErrorAt,
    'node', catchUpMaximumWireSegmentErrorNode);
console.log('18.9/31.2 catch-up max guidewire bend degrees',
    catchUpMaximumWireBend.toFixed(2), 'at', catchUpMaximumWireBendAt,
    'node', catchUpMaximumWireBendNode);
assert.ok(Math.abs(catchUpGuidewireInserted - 189) <= 1e-9,
    'the regression must hold the guidewire at 18.9 cm');
assert.ok(Math.abs(catchUpCatheter.progress - 312) <= 1e-6,
    `the regression must advance the catheter to 31.2 cm (${catchUpCatheter.progress} mm)`);
assert.ok(catchUpSawExternalContact && catchUpSawContainedOnly,
    'the regression must cross from external guidewire contact to containment only');
assert.ok(catchUpMaximumTipStep <= 1.8,
    `overtaking a short stationary guidewire must not kick the catheter tip (${catchUpMaximumTipStep} mm)`);
assert.ok(catchUpMaximumBend <= 36,
    `overtaking a short stationary guidewire must not fold the catheter (${catchUpMaximumBend} degrees)`);
assert.ok(catchUpMaximumSegmentError <= 0.12,
    `the catheter must retain its material length during the transition (${catchUpMaximumSegmentError} mm)`);
assert.ok(catchUpMaximumWireSegmentError <= 0.14,
    `the contained guidewire must retain its material length (${catchUpMaximumWireSegmentError} mm)`);
assert.ok(catchUpMaximumWireBend <= 65,
    `the contained guidewire must not fold while the catheter overtakes it (${catchUpMaximumWireBend} degrees)`);
let catchUpIdleTip = catchUpPreviousTip;
let catchUpLateIdleTipStep = 0;
let catchUpIdleSegmentError = 0;
let catchUpIdleWireSegmentError = 0;
let catchUpIdleWireBend = 0;
let catchUpIdleContainmentEscape = 0;
let catchUpIdleContainmentEscapeAt = -1;
for (let step = 0; step < 360; step++) {
    catchUpCatheter.advance(0, 1 / 120, catchUpGuidewireInserted);
    catchUpWireBody.syncFromElasticRod(catchUpWire, { preservePrevious: true });
    catchUpCatheter.stepPhysics(1 / 120, { collisions: false });
    catchUpCatheter.syncXpbdBody(catchUpCatheterBody);
    catchUpContainment.outerStartNode = catchUpCatheter.physicsLumenStartNode;
    catchUpContainment.limitDistalCorrection = false;
    catchUpContainment.preserveStationaryInnerLength = false;
    catchUpWorld.stepFixed();
    catchUpWireBody.syncToElasticRod(catchUpWire);
    const tipIndex = catchUpCatheterBody.activeEnd;
    const tip = [
        catchUpCatheterBody.x[tipIndex],
        catchUpCatheterBody.y[tipIndex],
        catchUpCatheterBody.z[tipIndex]
    ];
    if (step >= 240) {
        catchUpLateIdleTipStep = Math.max(
            catchUpLateIdleTipStep,
            Math.hypot(
                tip[0] - catchUpIdleTip[0],
                tip[1] - catchUpIdleTip[1],
                tip[2] - catchUpIdleTip[2]
            )
        );
        for (let index = catchUpCatheterBody.activeStart; index < tipIndex; index++) {
            const segmentLength = Math.hypot(
                catchUpCatheterBody.x[index + 1] - catchUpCatheterBody.x[index],
                catchUpCatheterBody.y[index + 1] - catchUpCatheterBody.y[index],
                catchUpCatheterBody.z[index + 1] - catchUpCatheterBody.z[index]
            );
            catchUpIdleSegmentError = Math.max(
                catchUpIdleSegmentError,
                Math.abs(segmentLength - catchUpCatheterBody.restLength[index])
            );
        }
        for (let index = catchUpWireBody.activeStart; index < catchUpWireBody.activeEnd; index++) {
            const segmentLength = Math.hypot(
                catchUpWireBody.x[index + 1] - catchUpWireBody.x[index],
                catchUpWireBody.y[index + 1] - catchUpWireBody.y[index],
                catchUpWireBody.z[index + 1] - catchUpWireBody.z[index]
            );
            catchUpIdleWireSegmentError = Math.max(
                catchUpIdleWireSegmentError,
                Math.abs(segmentLength - catchUpWireBody.restLength[index])
            );
            if (index <= catchUpWireBody.activeStart) continue;
            const incomingX = catchUpWireBody.x[index] - catchUpWireBody.x[index - 1];
            const incomingY = catchUpWireBody.y[index] - catchUpWireBody.y[index - 1];
            const incomingZ = catchUpWireBody.z[index] - catchUpWireBody.z[index - 1];
            const outgoingX = catchUpWireBody.x[index + 1] - catchUpWireBody.x[index];
            const outgoingY = catchUpWireBody.y[index + 1] - catchUpWireBody.y[index];
            const outgoingZ = catchUpWireBody.z[index + 1] - catchUpWireBody.z[index];
            const denominator = Math.max(EPSILON,
                Math.hypot(incomingX, incomingY, incomingZ) *
                    Math.hypot(outgoingX, outgoingY, outgoingZ));
            const cosine = Math.max(-1, Math.min(1,
                (incomingX * outgoingX + incomingY * outgoingY + incomingZ * outgoingZ) /
                    denominator));
            catchUpIdleWireBend = Math.max(
                catchUpIdleWireBend,
                Math.acos(cosine) * 180 / Math.PI
            );
        }
        const allowedRadius = Math.max(
            0,
            DEFAULT_TOOL_PROFILES.catheter.innerRadius - catchUpWireBody.radius
        );
        const outerStart = catchUpContainment.outerStartNode;
        const outerEnd = Math.min(
            catchUpCatheterBody.activeEnd - 1,
            catchUpCatheterBody.segmentCount - 1
        );
        for (
            let index = catchUpContainment.startNode;
            index <= catchUpContainment.endNode;
            index++
        ) {
            const escape = distanceToBodySegments(
                catchUpWireBody,
                index,
                catchUpCatheterBody,
                outerStart,
                outerEnd
            ) - allowedRadius;
            if (escape > catchUpIdleContainmentEscape) {
                catchUpIdleContainmentEscape = escape;
                catchUpIdleContainmentEscapeAt = index;
            }
        }
    }
    catchUpIdleTip = tip;
}
console.log('18.9/31.2 catch-up late idle tip step mm', catchUpLateIdleTipStep.toFixed(4));
console.log('18.9/31.2 catch-up idle segment error mm', catchUpIdleSegmentError.toFixed(4));
console.log('18.9/31.2 catch-up idle guidewire segment error mm',
    catchUpIdleWireSegmentError.toFixed(4));
console.log('18.9/31.2 catch-up idle guidewire bend degrees',
    catchUpIdleWireBend.toFixed(2));
console.log('18.9/31.2 catch-up idle containment escape mm',
    catchUpIdleContainmentEscape.toFixed(6), 'at', catchUpIdleContainmentEscapeAt,
    'range', catchUpContainment.startNode, catchUpContainment.endNode);
assert.ok(catchUpLateIdleTipStep <= 0.35,
    `the overtaken catheter should settle without residual waving (${catchUpLateIdleTipStep} mm)`);
assert.ok(catchUpIdleSegmentError <= 0.12,
    `the settled catheter should retain its material length (${catchUpIdleSegmentError} mm)`);
assert.ok(catchUpIdleWireSegmentError <= 0.12,
    `the settled guidewire should retain its material length (${catchUpIdleWireSegmentError} mm)`);
assert.ok(catchUpIdleWireBend <= 60,
    `the settled contained guidewire should not remain folded (${catchUpIdleWireBend} degrees)`);
assert.ok(catchUpIdleContainmentEscape <= 0.001,
    `the settled guidewire must remain inside the catheter lumen (${catchUpIdleContainmentEscape} mm escape)`);
catchUpCatheter.dispose();

// Simultaneous feed is the highest-risk clinical interaction: both sheath
// boundaries inject material while the catheter catches the slower guidewire.
// Exercise the same boundary-driven ordering and relative portal mode as the
// application, then release both controls and require a quiet equilibrium.
const simultaneousLength = 360;
const simultaneousSpacing = 2;
const simultaneousSheath = {
    start: { x: -20, y: 0, z: 0 },
    end: { x: 0, y: 0, z: 0 }
};
const simultaneousWire = new ElasticRod(
    simultaneousLength / simultaneousSpacing + 1,
    simultaneousSpacing
);
applyGuidewireMaterialProfile(simultaneousWire, {
    segmentLength: simultaneousSpacing
});
const simultaneousSolver = new GuidewireSolver({
    rod: simultaneousWire,
    segmentLength: simultaneousSpacing,
    guidewireLength: simultaneousLength,
    sheath: simultaneousSheath,
    advanceRate: 44,
    minInsert: 0,
    maxInsert: simultaneousLength,
    straightening: 0.72,
    routeBlend: 0,
    relaxationIterations: 6,
    lengthIterations: 10,
    foldGuardAngle: 166,
    foldGuardStrength: 0.62,
    foldGuardPasses: 2,
    foldGuardCenterPull: 1.25,
    stabilityRepairSegmentError: 0.09,
    stabilityRepairBendAngle: 150,
    stabilityRepairTargetBendAngle: 112,
    stabilityRepairPasses: 3,
    stabilityRepairLengthIterations: 10,
    tipBacktrackAngle: 108,
    tipBacktrackStrength: 1,
    segmentProjectionBlend: 0.48,
    maxSegmentProjectionStep: 0.32,
    collisionProjectionRepeats: 1,
    segmentSamples: [0.1, 0.24, 0.38, 0.52, 0.66, 0.8, 0.93],
    finalCollisionPasses: 3,
    finalLengthPasses: 2,
    finalProjectionPasses: 2
});
simultaneousSolver.initialize();
const simultaneousCatheter = new PigtailCatheter({
    wire: simultaneousWire,
    segmentLength: simultaneousSpacing,
    guidewireLength: simultaneousLength,
    tailProgressRef: () => simultaneousSolver.progress,
    vessel: { sheath: simultaneousSheath, segments: [] },
    maxLength: simultaneousLength
});
simultaneousCatheter.setType('berenstein');
simultaneousCatheter.setExternalCollisionSolver(true);
const simultaneousWorld = new EndovascularPhysicsWorld({ iterations: 6 });
const simultaneousWireBody = simultaneousWorld.createRod(
    'simultaneous-guidewire',
    simultaneousWire.nodes.length,
    simultaneousSpacing,
    { ...DEFAULT_TOOL_PROFILES.guidewire }
);
simultaneousWireBody.syncFromElasticRod(simultaneousWire);
const simultaneousCatheterBody = simultaneousWorld.createRod(
    'simultaneous-catheter',
    128,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
simultaneousWorld.addSheath({
    start: simultaneousSheath.start,
    end: simultaneousSheath.end,
    innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
    bodies: [simultaneousWireBody, simultaneousCatheterBody]
});
const simultaneousContainment = simultaneousWorld.addContainment(
    simultaneousWireBody,
    simultaneousCatheterBody,
    {
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        openProximal: true,
        openDistal: true,
        searchWindow: 2,
        innerResponse: 1,
        outerResponse: 0,
        portalInnerResponse: 1,
        portalOuterResponse: 0,
        portalCompliance: 1e-7,
        portalTransitionLength: 4,
        portalMaxCorrection: 0.1,
        finalProjection: 'inner',
        outerFollowsInnerCenterline: false,
        innerFollowsOuterCenterline: true,
        enforceDistalPortal: true,
        enabled: false
    }
);
const simultaneousExternalContact = simultaneousWorld.addToolContact(
    simultaneousWireBody,
    simultaneousCatheterBody,
    { friction: 0.08, openDistalB: true, enabled: false }
);
let simultaneousPortalInnerDriven = true;
let simultaneousPreviousTip = null;
let simultaneousMaximumTipStep = 0;
let simultaneousMaximumTipStepAt = '';
let simultaneousMaximumCatheterSegmentError = 0;
let simultaneousMaximumCatheterBend = 0;
let simultaneousMaximumCatheterBendAt = '';
let simultaneousMaximumCatheterBendLimit = 0;
let simultaneousMaximumCatheterBendIntrinsic = 0;
let simultaneousMaximumCatheterBendWeights = '';
let simultaneousMaximumWireSegmentError = 0;
let simultaneousMaximumWireBend = 0;
let simultaneousMaximumFreeCatheterSegmentError = 0;
let simultaneousMaximumFreeCatheterBend = 0;
let simultaneousMaximumFreeWireSegmentError = 0;
let simultaneousMaximumFreeWireBend = 0;
let simultaneousMaximumFreeCatheterSegmentErrorAt = '';
let simultaneousMaximumFreeWireSegmentErrorAt = '';
let simultaneousMaximumCatheterSegmentErrorAt = '';
let simultaneousMaximumWireSegmentErrorAt = '';
let simultaneousMaximumWireBendAt = '';

function stepSimultaneousFixture(guidewireCommand, catheterCommand) {
    const dt = 1 / 120;
    const guidewireDelta = simultaneousSolver.advance(
        guidewireCommand,
        dt,
        null,
        { routeAssist: false, boundaryDriven: true }
    );
    const inserted = simultaneousSolver.progress;
    const catheterProgressBefore = simultaneousCatheter.progress;
    simultaneousCatheter.advance(catheterCommand, dt, inserted);
    const catheterDelta = simultaneousCatheter.progress - catheterProgressBefore;
    simultaneousWireBody.syncFromElasticRod(simultaneousWire, {
        preservePrevious: true
    });
    simultaneousWireBody.setActiveRange(
        Math.min(
            simultaneousWireBody.count - 2,
            Math.max(0, simultaneousSolver.firstInsertedNodeIndex() - 1)
        ),
        simultaneousWireBody.count - 1
    );
    simultaneousWireBody.setCollisionRange(
        Math.max(0, simultaneousSolver.firstLumenNodeIndex() - 1),
        simultaneousWireBody.segmentCount - 1
    );
    simultaneousCatheter.stepPhysics(dt, { collisions: false });
    const activeCount = simultaneousCatheter.syncXpbdBody(simultaneousCatheterBody);
    const firstContainedNode = Math.max(0, Math.ceil(
        (simultaneousLength - inserted) / simultaneousSpacing
    ));
    const materialEndNode = Math.min(
        simultaneousWireBody.count - 1,
        Math.floor(
            (simultaneousLength - inserted + simultaneousCatheter.progress) /
                simultaneousSpacing
        )
    );
    simultaneousContainment.outerStartNode = simultaneousCatheter.physicsLumenStartNode;
    const lastContainedNode = materialEndNode;
    simultaneousContainment.enabled =
        simultaneousCatheter.progress > 0.5 &&
        activeCount >= 2 &&
        lastContainedNode >= firstContainedNode;
    simultaneousContainment.startNode = firstContainedNode;
    simultaneousContainment.endNode = Math.max(firstContainedNode, lastContainedNode);
    simultaneousContainment.innerArcOffset =
        firstContainedNode * simultaneousSpacing - simultaneousLength + inserted;
    simultaneousContainment.containedLength = Math.min(
        simultaneousCatheter.progress,
        inserted
    );
    const relativePortalAdvance = guidewireDelta - catheterDelta;
    if (relativePortalAdvance > 1e-5) simultaneousPortalInnerDriven = true;
    else if (relativePortalAdvance < -1e-5) simultaneousPortalInnerDriven = false;
    simultaneousContainment.enforceDistalPortal = true;
    simultaneousContainment.portalInnerResponse = simultaneousPortalInnerDriven ? 1 : 0;
    simultaneousContainment.portalOuterResponse = simultaneousPortalInnerDriven ? 0 : 1;
    simultaneousContainment.limitDistalCorrection =
        Math.abs(guidewireDelta) > 1e-5 || Math.abs(catheterDelta) > 1e-5;
    simultaneousContainment.preserveStationaryInnerLength =
        Math.abs(catheterDelta) > 1e-5 && Math.abs(guidewireCommand) <= 1e-5;
    simultaneousContainment.reconcileMovingInnerStructure =
        Math.abs(catheterDelta) > 1e-5 && Math.abs(guidewireCommand) > 1e-5;
    simultaneousContainment.outerResponse = simultaneousContainment.preserveStationaryInnerLength
        ? 0.2
        : simultaneousContainment.reconcileMovingInnerStructure
            ? 0.04
            : 0;
    simultaneousWireBody.projectionVelocityRetention = Math.abs(guidewireCommand) > 0
        ? 1
        : 0.005;
    simultaneousWireBody.maxFrameDisplacement =
        simultaneousContainment.preserveStationaryInnerLength ? 1.5 : Infinity;
    simultaneousWireBody.frameDisplacementStartNode = Math.max(
        simultaneousWireBody.activeStart,
        simultaneousContainment.endNode
    );
    const catheterEndSegment = Math.max(0, activeCount - 2);
    const firstExternalSegment = Math.max(0, Math.min(
        simultaneousWireBody.segmentCount - 1,
        lastContainedNode + 1
    ));
    simultaneousExternalContact.enabled =
        simultaneousCatheter.progress > 4 &&
        activeCount >= 2 &&
        inserted > simultaneousCatheter.progress + 0.5 &&
        firstExternalSegment <= simultaneousWireBody.activeEnd - 1;
    simultaneousExternalContact.startSegmentA = firstExternalSegment;
    simultaneousExternalContact.endSegmentA = Math.min(
        simultaneousWireBody.activeEnd - 1,
        firstExternalSegment + 16
    );
    simultaneousExternalContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
    simultaneousExternalContact.endSegmentB = catheterEndSegment;
    simultaneousWorld.stepFixed();
    simultaneousWireBody.syncToElasticRod(simultaneousWire);
    return { firstExternalSegment, lastContainedNode };
}

// Establish only a small wire lead, then hold both controls continuously. The
// catheter's higher feed rate makes it catch and overtake the wire while both
// rods are actively remeshed at the sheath.
for (let step = 0; step < 55; step++) stepSimultaneousFixture(1, 0);
for (let step = 0; step < 720; step++) {
    const simultaneousRange = stepSimultaneousFixture(1, 1);
    const tipIndex = simultaneousCatheterBody.activeEnd;
    const tip = [
        simultaneousCatheterBody.x[tipIndex],
        simultaneousCatheterBody.y[tipIndex],
        simultaneousCatheterBody.z[tipIndex]
    ];
    if (simultaneousPreviousTip && simultaneousCatheter.progress >= 60) {
        const tipStep = Math.hypot(
            tip[0] - simultaneousPreviousTip[0],
            tip[1] - simultaneousPreviousTip[1],
            tip[2] - simultaneousPreviousTip[2]
        );
        if (tipStep > simultaneousMaximumTipStep) {
            simultaneousMaximumTipStep = tipStep;
            simultaneousMaximumTipStepAt = [
                step,
                simultaneousCatheter.progress.toFixed(2),
                simultaneousSolver.progress.toFixed(2),
                simultaneousCatheterBody.activeEnd,
                simultaneousCatheterBody.collisionStartSegment
            ].join('/');
        }
        const catheterMetrics = bodyShapeMetrics(simultaneousCatheterBody);
        const wireMetrics = bodyShapeMetrics(simultaneousWireBody);
        const freeCatheterMetrics = bodyShapeMetrics(
            simultaneousCatheterBody,
            simultaneousCatheterBody.collisionStartSegment + 4
        );
        const freeWireMetrics = bodyShapeMetrics(
            simultaneousWireBody,
            Math.min(
                simultaneousWireBody.activeEnd,
                simultaneousRange.firstExternalSegment + 4
            )
        );
        if (catheterMetrics.maximumSegmentError > simultaneousMaximumCatheterSegmentError) {
            simultaneousMaximumCatheterSegmentError = catheterMetrics.maximumSegmentError;
            simultaneousMaximumCatheterSegmentErrorAt =
                `${step}/${catheterMetrics.maximumSegmentErrorNode}`;
        }
        if (catheterMetrics.maximumBend > simultaneousMaximumCatheterBend) {
            simultaneousMaximumCatheterBend = catheterMetrics.maximumBend;
            simultaneousMaximumCatheterBendAt =
                `${step}/${catheterMetrics.maximumBendNode}/` +
                `${simultaneousCatheterBody.collisionStartSegment}/` +
                `${simultaneousCatheterBody.activeEnd}`;
            simultaneousMaximumCatheterBendLimit =
                simultaneousCatheterBody.maxBendAngleByNode[
                    catheterMetrics.maximumBendNode
                ];
            simultaneousMaximumCatheterBendIntrinsic =
                simultaneousCatheterBody.intrinsicBendEnabled[
                    catheterMetrics.maximumBendNode
                ];
            simultaneousMaximumCatheterBendWeights = [
                catheterMetrics.maximumBendNode - 1,
                catheterMetrics.maximumBendNode,
                catheterMetrics.maximumBendNode + 1
            ].map(node => simultaneousCatheterBody.inverseMass[node]).join('/');
        }
        if (wireMetrics.maximumSegmentError > simultaneousMaximumWireSegmentError) {
            simultaneousMaximumWireSegmentError = wireMetrics.maximumSegmentError;
            simultaneousMaximumWireSegmentErrorAt =
                `${step}/${wireMetrics.maximumSegmentErrorNode}/` +
                `${simultaneousWireBody.activeStart}/` +
                `${simultaneousWireBody.collisionStartSegment}`;
        }
        if (wireMetrics.maximumBend > simultaneousMaximumWireBend) {
            simultaneousMaximumWireBend = wireMetrics.maximumBend;
            simultaneousMaximumWireBendAt = `${step}/${wireMetrics.maximumBendNode}`;
        }
        if (freeCatheterMetrics.maximumSegmentError > simultaneousMaximumFreeCatheterSegmentError) {
            simultaneousMaximumFreeCatheterSegmentError = freeCatheterMetrics.maximumSegmentError;
            simultaneousMaximumFreeCatheterSegmentErrorAt =
                `${step}/${freeCatheterMetrics.maximumSegmentErrorNode}`;
        }
        simultaneousMaximumFreeCatheterBend = Math.max(
            simultaneousMaximumFreeCatheterBend,
            freeCatheterMetrics.maximumBend
        );
        if (freeWireMetrics.maximumSegmentError > simultaneousMaximumFreeWireSegmentError) {
            simultaneousMaximumFreeWireSegmentError = freeWireMetrics.maximumSegmentError;
            simultaneousMaximumFreeWireSegmentErrorAt =
                `${step}/${freeWireMetrics.maximumSegmentErrorNode}/` +
                `${simultaneousWireBody.collisionStartSegment}`;
        }
        simultaneousMaximumFreeWireBend = Math.max(
            simultaneousMaximumFreeWireBend,
            freeWireMetrics.maximumBend
        );
    }
    simultaneousPreviousTip = tip;
}
let simultaneousLateIdleTipStep = 0;
let simultaneousLateIdleTipStepAt = -1;
let simultaneousLateIdleCatheterSegmentError = 0;
let simultaneousLateIdleCatheterBend = 0;
let simultaneousIdleWireSegmentError = 0;
let simultaneousIdleWireBend = 0;
let simultaneousIdleWireSegmentErrorAt = -1;
let simultaneousIdleWireBendAt = -1;
for (let step = 0; step < 360; step++) {
    const previousTip = simultaneousPreviousTip;
    stepSimultaneousFixture(0, 0);
    const tipIndex = simultaneousCatheterBody.activeEnd;
    simultaneousPreviousTip = [
        simultaneousCatheterBody.x[tipIndex],
        simultaneousCatheterBody.y[tipIndex],
        simultaneousCatheterBody.z[tipIndex]
    ];
    if (step < 240) continue;
    const idleTipStep = Math.hypot(
        simultaneousPreviousTip[0] - previousTip[0],
        simultaneousPreviousTip[1] - previousTip[1],
        simultaneousPreviousTip[2] - previousTip[2]
    );
    if (idleTipStep > simultaneousLateIdleTipStep) {
        simultaneousLateIdleTipStep = idleTipStep;
        simultaneousLateIdleTipStepAt = step;
    }
    const idleCatheterMetrics = bodyShapeMetrics(simultaneousCatheterBody);
    simultaneousLateIdleCatheterSegmentError = Math.max(
        simultaneousLateIdleCatheterSegmentError,
        idleCatheterMetrics.maximumSegmentError
    );
    simultaneousLateIdleCatheterBend = Math.max(
        simultaneousLateIdleCatheterBend,
        idleCatheterMetrics.maximumBend
    );
    const wireMetrics = bodyShapeMetrics(simultaneousWireBody);
    if (wireMetrics.maximumSegmentError > simultaneousIdleWireSegmentError) {
        simultaneousIdleWireSegmentError = wireMetrics.maximumSegmentError;
        simultaneousIdleWireSegmentErrorAt = wireMetrics.maximumSegmentErrorNode;
    }
    if (wireMetrics.maximumBend > simultaneousIdleWireBend) {
        simultaneousIdleWireBend = wireMetrics.maximumBend;
        simultaneousIdleWireBendAt = wireMetrics.maximumBendNode;
    }
}
console.log('simultaneous feed final guidewire/catheter mm',
    simultaneousSolver.progress.toFixed(1), simultaneousCatheter.progress.toFixed(1));
console.log('simultaneous feed max tip step mm', simultaneousMaximumTipStep.toFixed(4),
    simultaneousMaximumTipStepAt);
console.log('simultaneous feed max catheter segment error mm',
    simultaneousMaximumCatheterSegmentError.toFixed(4), 'at',
    simultaneousMaximumCatheterSegmentErrorAt);
console.log('simultaneous feed max catheter bend degrees',
    simultaneousMaximumCatheterBend.toFixed(2), 'at',
    simultaneousMaximumCatheterBendAt, 'limit/intrinsic',
    simultaneousMaximumCatheterBendLimit,
    simultaneousMaximumCatheterBendIntrinsic, 'weights',
    simultaneousMaximumCatheterBendWeights);
console.log('simultaneous feed max guidewire segment error mm',
    simultaneousMaximumWireSegmentError.toFixed(4), 'at',
    simultaneousMaximumWireSegmentErrorAt);
console.log('simultaneous feed max guidewire bend degrees',
    simultaneousMaximumWireBend.toFixed(2), 'at', simultaneousMaximumWireBendAt);
console.log('simultaneous feed free catheter segment/bend',
    simultaneousMaximumFreeCatheterSegmentError.toFixed(4),
    simultaneousMaximumFreeCatheterBend.toFixed(2),
    simultaneousMaximumFreeCatheterSegmentErrorAt);
console.log('simultaneous feed free guidewire segment/bend',
    simultaneousMaximumFreeWireSegmentError.toFixed(4),
    simultaneousMaximumFreeWireBend.toFixed(2),
    simultaneousMaximumFreeWireSegmentErrorAt);
console.log('simultaneous feed late idle tip step mm',
    simultaneousLateIdleTipStep.toFixed(4), 'at', simultaneousLateIdleTipStepAt,
    'catheter segment/bend', simultaneousLateIdleCatheterSegmentError.toFixed(4),
    simultaneousLateIdleCatheterBend.toFixed(2));
console.log('simultaneous feed idle guidewire segment error mm',
    simultaneousIdleWireSegmentError.toFixed(4), 'at',
    simultaneousIdleWireSegmentErrorAt, 'range', simultaneousWireBody.activeStart,
    simultaneousWireBody.collisionStartSegment);
console.log('simultaneous feed idle guidewire bend degrees',
    simultaneousIdleWireBend.toFixed(2), 'at', simultaneousIdleWireBendAt);
assert.ok(simultaneousMaximumTipStep <= 3.5,
    `simultaneous feed must not kick the catheter tip (${simultaneousMaximumTipStep} mm)`);
assert.ok(simultaneousMaximumCatheterSegmentError <= 1,
    `simultaneous feed must bound catheter feed-segment remeshing (${simultaneousMaximumCatheterSegmentError} mm)`);
assert.ok(simultaneousMaximumCatheterBend <= 35,
    `simultaneous feed must not fold the catheter (${simultaneousMaximumCatheterBend} degrees)`);
assert.ok(simultaneousMaximumWireSegmentError <= 1,
    `simultaneous feed must bound guidewire feed-segment remeshing (${simultaneousMaximumWireSegmentError} mm)`);
assert.ok(simultaneousMaximumWireBend <= 85,
    `simultaneous feed must bound the guidewire sheath-transition bend (${simultaneousMaximumWireBend} degrees)`);
assert.ok(simultaneousMaximumFreeCatheterSegmentError <= 0.13,
    `the free catheter must retain material length (${simultaneousMaximumFreeCatheterSegmentError} mm)`);
assert.ok(simultaneousMaximumFreeCatheterBend <= 35,
    `the free catheter shaft must not fold (${simultaneousMaximumFreeCatheterBend} degrees)`);
assert.ok(simultaneousMaximumFreeWireSegmentError <= 0.12,
    `the free guidewire must retain material length (${simultaneousMaximumFreeWireSegmentError} mm)`);
assert.ok(simultaneousMaximumFreeWireBend <= 65,
    `the free guidewire shaft must not fold (${simultaneousMaximumFreeWireBend} degrees)`);
assert.ok(simultaneousLateIdleTipStep <= 0.35,
    `the simultaneously fed pair must settle without waving (${simultaneousLateIdleTipStep} mm)`);
assert.ok(simultaneousIdleWireSegmentError <= 0.12,
    `the settled simultaneous guidewire must retain length (${simultaneousIdleWireSegmentError} mm)`);
assert.ok(simultaneousIdleWireBend <= 60,
    `the settled simultaneous guidewire must not remain folded (${simultaneousIdleWireBend} degrees)`);
simultaneousCatheter.dispose();

// With material-coordinate feeding, removing one active catheter node during
// withdrawal happens at the sheath entry. The physical distal tip must survive
// the topology change instead of being deleted from a recorded route.
const guidewireLength = 180;
const guidewireSpacing = 2;
let guidewireInserted = 120;
const interactionWire = new ElasticRod(
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
materialCatheter.setExternalCollisionSolver(true);
const materialBodyWorld = new EndovascularPhysicsWorld();
const materialBody = materialBodyWorld.createRod('material-catheter', 128, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
for (let step = 0; step < 220; step++) {
    materialCatheter.advance(1, 1 / 120, guidewireInserted);
    materialCatheter.stepPhysics(1 / 120, { collisions: false });
    materialCatheter.syncXpbdBody(materialBody);
}
const insertedCount = materialCatheter.physicsActiveCount;
assert.ok(insertedCount > 20, 'the material-coordinate fixture should deploy a catheter');
materialBody.y[materialBody.activeEnd] = 6;
materialBody.previousY[materialBody.activeEnd] = 6;
let withdrawnCount = insertedCount;
for (let step = 0; step < 80 && withdrawnCount >= insertedCount; step++) {
    materialCatheter.advance(-1, 1 / 120, guidewireInserted);
    materialCatheter.stepPhysics(1 / 120, { collisions: false });
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
