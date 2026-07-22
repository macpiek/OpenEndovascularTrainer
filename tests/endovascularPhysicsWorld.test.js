import assert from 'node:assert/strict';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';

const EPSILON = 1e-8;

function setVector(target, x, y, z) {
    target.x = x;
    target.y = y;
    target.z = z;
}

function createContactResult() {
    return {
        inside: false,
        violation: false,
        signedDistance: -Infinity,
        signedGap: -Infinity,
        penetration: Infinity,
        branchId: -1,
        segmentT: 0,
        timeOfImpact: 1,
        source: 'analytic',
        point: { x: 0, y: 0, z: 0 },
        target: { x: 0, y: 0, z: 0 },
        closestPoint: { x: 0, y: 0, z: 0 },
        normal: { x: 1, y: 0, z: 0 },
        inward: { x: 1, y: 0, z: 0 }
    };
}

function copyContact(target, source) {
    target.inside = source.inside;
    target.violation = source.violation;
    target.signedDistance = source.signedDistance;
    target.signedGap = source.signedGap;
    target.penetration = source.penetration;
    target.branchId = source.branchId;
    target.segmentT = source.segmentT;
    target.timeOfImpact = source.timeOfImpact;
    target.source = source.source;
    for (const key of ['point', 'target', 'closestPoint', 'normal', 'inward']) {
        setVector(target[key], source[key].x, source[key].y, source[key].z);
    }
    return target;
}

class AnalyticContactField {
    constructor(distanceFunction) {
        this.distanceFunction = distanceFunction;
        this.voxelSize = 0.5;
        this._sample = createContactResult();
        this._point = { x: 0, y: 0, z: 0 };
    }

    querySphere(position, radius = 0, out = createContactResult()) {
        const state = this.distanceFunction(position.x, position.y, position.z);
        const gap = state.distance - radius;
        const penetration = Math.max(0, -gap);
        out.inside = state.distance >= 0;
        out.violation = gap < 0;
        out.signedDistance = state.distance;
        out.signedGap = gap;
        out.penetration = penetration;
        out.branchId = state.branchId ?? 0;
        out.segmentT = 0;
        out.timeOfImpact = out.violation ? 0 : 1;
        out.source = 'analytic';
        setVector(out.point, position.x, position.y, position.z);
        setVector(out.inward, state.inwardX, state.inwardY, state.inwardZ);
        setVector(out.normal, state.inwardX, state.inwardY, state.inwardZ);
        setVector(
            out.closestPoint,
            position.x - state.inwardX * state.distance,
            position.y - state.inwardY * state.distance,
            position.z - state.inwardZ * state.distance
        );
        setVector(
            out.target,
            position.x + state.inwardX * penetration,
            position.y + state.inwardY * penetration,
            position.z + state.inwardZ * penetration
        );
        return out;
    }

    queryCapsule(start, end, radius = 0, out = createContactResult()) {
        let bestGap = Infinity;
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const dz = end.z - start.z;
        const sampleCount = Math.max(2, Math.ceil(Math.hypot(dx, dy, dz) / 0.5));
        for (let index = 0; index <= sampleCount; index++) {
            const t = index / sampleCount;
            this._point.x = start.x + dx * t;
            this._point.y = start.y + dy * t;
            this._point.z = start.z + dz * t;
            const contact = this.querySphere(this._point, radius, this._sample);
            if (contact.signedGap >= bestGap) continue;
            bestGap = contact.signedGap;
            copyContact(out, contact);
            out.segmentT = t;
        }
        return out;
    }

    sweepSphere(previous, current, radius = 0, out = createContactResult()) {
        const dx = current.x - previous.x;
        const dy = current.y - previous.y;
        const dz = current.z - previous.z;
        const sampleCount = Math.max(2, Math.ceil(Math.hypot(dx, dy, dz) / Math.max(0.05, radius * 0.25)));
        for (let index = 0; index <= sampleCount; index++) {
            const t = index / sampleCount;
            this._point.x = previous.x + dx * t;
            this._point.y = previous.y + dy * t;
            this._point.z = previous.z + dz * t;
            const contact = this.querySphere(this._point, radius, this._sample);
            if (!contact.violation) continue;
            copyContact(out, contact);
            out.timeOfImpact = t;
            return out;
        }
        this.querySphere(current, radius, out);
        out.timeOfImpact = 1;
        return out;
    }
}

function radialState(radius, x, y, z, branchId = 0) {
    const radial = Math.hypot(y, z);
    const inverse = radial > EPSILON ? 1 / radial : 0;
    return {
        distance: radius - radial,
        inwardX: 0,
        inwardY: radial > EPSILON ? -y * inverse : 1,
        inwardZ: radial > EPSILON ? -z * inverse : 0,
        branchId
    };
}

function cylinderField(radius = 5) {
    return new AnalyticContactField((x, y, z) => radialState(radius, x, y, z));
}

function variableCylinderField(radiusAtX) {
    return new AnalyticContactField((x, y, z) => radialState(radiusAtX(x), x, y, z));
}

function bendField(centerlineRadius = 20, lumenRadius = 4) {
    return new AnalyticContactField((x, y, z) => {
        const angle = Math.atan2(y, x);
        const clampedAngle = Math.max(0, Math.min(Math.PI * 0.5, angle));
        const cx = Math.cos(clampedAngle) * centerlineRadius;
        const cy = Math.sin(clampedAngle) * centerlineRadius;
        const dx = cx - x;
        const dy = cy - y;
        const dz = -z;
        const radial = Math.hypot(dx, dy, dz);
        const inverse = radial > EPSILON ? 1 / radial : 1;
        return {
            distance: lumenRadius - radial,
            inwardX: dx * inverse,
            inwardY: dy * inverse,
            inwardZ: dz * inverse,
            branchId: 0
        };
    });
}

function bifurcationField(radius = 3) {
    return new AnalyticContactField((x, y, z) => {
        const trunk = radialState(radius, x, y, z, 0);
        const branchY = y - Math.max(0, x - 5) * 0.6;
        const upper = radialState(radius * 0.75, x, branchY, z, 1);
        const lower = radialState(radius * 0.75, x, y + Math.max(0, x - 5) * 0.6, z, 2);
        if (upper.distance > trunk.distance && upper.distance >= lower.distance) return upper;
        if (lower.distance > trunk.distance) return lower;
        return trunk;
    });
}

function seedStraightRod(body, startX, spacing, y = 0, z = 0) {
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, startX + index * spacing, y, z);
    }
    body.captureRestConfiguration();
    body.copyCurrentToPrevious();
}

function averageCoordinate(values, start = 0, end = values.length - 1) {
    let sum = 0;
    for (let index = start; index <= end; index++) sum += values[index];
    return sum / Math.max(1, end - start + 1);
}

function maximumPointBendDegrees(points, pointAt = index => points[index]) {
    let maximum = 0;
    for (let index = 1; index < points.length - 1; index++) {
        const previous = pointAt(index - 1);
        const current = pointAt(index);
        const next = pointAt(index + 1);
        const ax = current.x - previous.x;
        const ay = current.y - previous.y;
        const az = current.z - previous.z;
        const bx = next.x - current.x;
        const by = next.y - current.y;
        const bz = next.z - current.z;
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator <= EPSILON) continue;
        const cosine = Math.max(-1, Math.min(1, (ax * bx + ay * by + az * bz) / denominator));
        maximum = Math.max(maximum, Math.acos(cosine) * 180 / Math.PI);
    }
    return maximum;
}

function maximumGapViolation(field, body) {
    const result = createContactResult();
    let penetration = 0;
    for (let index = body.activeStart; index < body.activeEnd; index++) {
        const start = { x: body.x[index], y: body.y[index], z: body.z[index] };
        const end = { x: body.x[index + 1], y: body.y[index + 1], z: body.z[index + 1] };
        field.queryCapsule(start, end, body.radius, result);
        penetration = Math.max(penetration, result.penetration);
    }
    return penetration;
}

function exerciseWallFixture(name, field, initialize) {
    const world = new EndovascularPhysicsWorld({ contactField: field });
    const body = world.createRod(name, 12, 1, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.45,
        stretchCompliance: 0,
        bendCompliance: 3e-4
    });
    initialize(body);
    body.captureRestConfiguration();
    body.copyCurrentToPrevious();
    let maximumPostStepPenetration = 0;
    for (let step = 0; step < 24; step++) {
        world.stepFixed();
        maximumPostStepPenetration = Math.max(maximumPostStepPenetration, world.settledMaxPenetration);
    }
    const penetration = maximumGapViolation(field, body);
    const stats = world.getStats().bodies[0];
    assert.ok(penetration <= 0.05, `${name} settled penetration should stay <= 0.05 mm, got ${penetration}`);
    assert.ok(maximumPostStepPenetration <= 0.2,
        `${name} post-step transient penetration should stay <= 0.20 mm, got ${maximumPostStepPenetration}`);
    assert.ok(stats.maxLengthError <= 0.01, `${name} length error should stay <= 1%`);
    assert.equal(stats.finite, true, `${name} should stay finite`);
}

exerciseWallFixture('cylinder', cylinderField(5), body => seedStraightRod(body, -5, 1, 4.82, 0));
exerciseWallFixture('stenosis', variableCylinderField(x => 4.5 - 1.7 * Math.exp(-(x * x) / 5)), body => {
    seedStraightRod(body, -5.5, 1, 2.7, 0);
});
exerciseWallFixture('taper', variableCylinderField(x => 5 - Math.max(0, Math.min(2, (x + 5) * 0.12))), body => {
    seedStraightRod(body, -5, 1, 3.05, 0);
});
exerciseWallFixture('bend', bendField(), body => {
    for (let index = 0; index < body.count; index++) {
        const angle = index / (body.count - 1) * Math.PI * 0.5;
        const radius = 23.7;
        body.setNodePosition(index, Math.cos(angle) * radius, Math.sin(angle) * radius, 0);
    }
});
exerciseWallFixture('bifurcation', bifurcationField(), body => seedStraightRod(body, 0, 1, 2.45, 0));

const coupledWallWorld = new EndovascularPhysicsWorld({ contactField: cylinderField(5) });
const wallLoadedCatheter = coupledWallWorld.createRod('wall-loaded-catheter', 48, 1, {
    ...DEFAULT_TOOL_PROFILES.catheter,
    radius: 0.8,
    innerRadius: 0.7,
    stretchCompliance: 0,
    bendCompliance: 2e-4,
    shapeCompliance: 2e-4
});
const wallLoadedWire = coupledWallWorld.createRod('wall-loaded-wire', 48, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    radius: 0.45,
    stretchCompliance: 0
});
seedStraightRod(wallLoadedCatheter, -23.5, 1, 4.1, 0);
seedStraightRod(wallLoadedWire, -23.5, 1, 4.55, 0);
for (let index = 0; index < wallLoadedCatheter.count; index++) {
    wallLoadedCatheter.setRestShapeTarget(index, wallLoadedCatheter.x[index], 4.3, 0, 2e-4);
}
coupledWallWorld.addContainment(wallLoadedWire, wallLoadedCatheter, {
    innerRadius: wallLoadedCatheter.innerRadius,
    openProximal: false,
    openDistal: false,
    searchWindow: 2
});
let coupledMaximumPostStepPenetration = 0;
for (let step = 0; step < 120; step++) {
    coupledWallWorld.stepFixed();
    coupledMaximumPostStepPenetration = Math.max(
        coupledMaximumPostStepPenetration,
        coupledWallWorld.settledMaxPenetration
    );
}
assert.ok(coupledMaximumPostStepPenetration <= 0.2,
    `coupled wall and lumen constraints should stay below 0.20 mm after each step (got ${coupledMaximumPostStepPenetration})`);
assert.ok(maximumGapViolation(coupledWallWorld.contactField, wallLoadedCatheter) <= 0.05,
    'a catheter loaded toward the wall by its guidewire should settle below 0.05 mm penetration');
assert.ok(coupledWallWorld.getStats().bodies.every(body => body.maxLengthError <= 0.01),
    'final wall correction should retain both tools within one percent length error');

const sheathWorld = new EndovascularPhysicsWorld();
const sheathRod = sheathWorld.createRod('sheath-contained', 4, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    radius: 0.45,
    stretchCompliance: 0
});
seedStraightRod(sheathRod, 3, 1, 1.3, 0);
sheathWorld.addSheath({
    start: { x: 0, y: 0, z: 0 },
    end: { x: 10, y: 0, z: 0 },
    innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
    bodies: [sheathRod]
});
for (let step = 0; step < 8; step++) sheathWorld.stepFixed();
for (let index = 0; index < sheathRod.count; index++) {
    assert.ok(Math.hypot(sheathRod.y[index], sheathRod.z[index]) <= 0.451, 'sheath should enforce its analytic lumen');
}

assert.equal(DEFAULT_TOOL_PROFILES.catheter.innerDiameter, 0.97,
    'the default catheter profile should expose a configurable 0.97 mm ID');
assert.equal(DEFAULT_TOOL_PROFILES.sheath.innerDiameter, 1.80,
    'the default sheath profile should expose a configurable 1.80 mm ID');

const sharedSheathWorld = new EndovascularPhysicsWorld();
const sheathWire = sharedSheathWorld.createRod('sheath-wire', 4, 1, DEFAULT_TOOL_PROFILES.guidewire);
const sheathCatheter = sharedSheathWorld.createRod('sheath-catheter', 4, 1, DEFAULT_TOOL_PROFILES.catheter);
seedStraightRod(sheathWire, 3, 1, 1.2, 0);
seedStraightRod(sheathCatheter, 3, 1, -0.5, 0);
sharedSheathWorld.addSheath({
    start: { x: 0, y: 0, z: 0 },
    end: { x: 10, y: 0, z: 0 },
    innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
    bodies: [sheathWire, sheathCatheter]
});
for (let step = 0; step < 8; step++) sharedSheathWorld.stepFixed();
const wireSheathClearance = DEFAULT_TOOL_PROFILES.sheath.innerRadius - sheathWire.radius;
const catheterSheathClearance = DEFAULT_TOOL_PROFILES.sheath.innerRadius - sheathCatheter.radius;
for (let index = 0; index < sheathWire.count; index++) {
    assert.ok(Math.hypot(sheathWire.y[index], sheathWire.z[index]) <= wireSheathClearance + 0.002,
        'the guidewire should remain inside the shared sheath lumen');
    assert.ok(Math.hypot(sheathCatheter.y[index], sheathCatheter.z[index]) <= catheterSheathClearance + 0.002,
        'the catheter should remain inside the shared sheath lumen');
}

const openEndWorld = new EndovascularPhysicsWorld();
const openEndRod = openEndWorld.createRod('open-end', 3, 1, { radius: 0.45, stretchCompliance: 0 });
seedStraightRod(openEndRod, 11, 1, 1.3, 0);
openEndWorld.addSheath({
    start: { x: 0, y: 0, z: 0 },
    end: { x: 10, y: 0, z: 0 },
    innerRadius: 0.9,
    bodies: [openEndRod]
});
openEndWorld.stepFixed();
assert.ok(Math.abs(openEndRod.y[1] - 1.3) < 1e-4, 'the distal sheath end should remain open');

const containmentWorld = new EndovascularPhysicsWorld();
const catheter = containmentWorld.createRod('catheter', 16, 1, DEFAULT_TOOL_PROFILES.catheter);
const containedWire = containmentWorld.createRod('wire', 16, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(catheter, 0, 1, 0, 0);
seedStraightRod(containedWire, 0, 1, 1.2, 0);
for (let index = 0; index < catheter.count; index++) catheter.setPinned(index, true);
containmentWorld.addContainment(containedWire, catheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    searchWindow: catheter.segmentCount
});
for (let step = 0; step < 16; step++) containmentWorld.stepFixed();
const containmentClearance = DEFAULT_TOOL_PROFILES.catheter.innerRadius - DEFAULT_TOOL_PROFILES.guidewire.radius;
for (let index = 0; index < containedWire.count - 1; index++) {
    assert.ok(Math.hypot(containedWire.y[index], containedWire.z[index]) <= containmentClearance + 0.01,
        'guidewire should remain inside the catheter ID with two-way radial constraints');
}

const reactionWorld = new EndovascularPhysicsWorld();
const reactingCatheter = reactionWorld.createRod('reacting-catheter', 8, 1, DEFAULT_TOOL_PROFILES.catheter);
const reactingWire = reactionWorld.createRod('reacting-wire', 8, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(reactingCatheter, 0, 1, 0, 0);
seedStraightRod(reactingWire, 0, 1, 1.2, 0);
reactionWorld.addContainment(reactingWire, reactingCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    searchWindow: reactingCatheter.segmentCount
});
for (let step = 0; step < 24; step++) reactionWorld.stepFixed();
const reactedCatheterY = averageCoordinate(reactingCatheter.y);
const reactedWireY = averageCoordinate(reactingWire.y);
assert.ok(reactedCatheterY > 0.05,
    'catheter nodes should receive the opposite reaction from an off-axis contained guidewire');
assert.ok(reactedWireY < 1.15,
    'the contained guidewire should move toward the catheter lumen axis');
assert.ok(reactedWireY - reactedCatheterY <= containmentClearance + 0.02,
    'two-way containment should settle within the configured radial clearance');

function containmentSlidingSpeed(friction) {
    const world = new EndovascularPhysicsWorld();
    const outer = world.createRod('friction-catheter', 8, 1, DEFAULT_TOOL_PROFILES.catheter);
    const inner = world.createRod('friction-wire', 8, 1, DEFAULT_TOOL_PROFILES.guidewire);
    seedStraightRod(outer, 0, 1, 0, 0);
    seedStraightRod(inner, 0, 1, 0.35, 0);
    for (let index = 0; index < outer.count; index++) outer.setPinned(index, true);
    for (let index = 0; index < inner.count; index++) inner.velocityX[index] = 24;
    world.addContainment(inner, outer, {
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        friction,
        openProximal: false,
        openDistal: false,
        searchWindow: outer.segmentCount
    });
    world.stepFixed();
    return averageCoordinate(inner.velocityX, 1, inner.count - 2);
}

const freeContainmentSlidingSpeed = containmentSlidingSpeed(0);
const frictionContainmentSlidingSpeed = containmentSlidingSpeed(0.8);
assert.ok(frictionContainmentSlidingSpeed < freeContainmentSlidingSpeed * 0.5,
    'catheter-lumen Coulomb friction should resist guidewire sliding under radial load');

const changingRangeWorld = new EndovascularPhysicsWorld();
const changingRangeCatheter = changingRangeWorld.createRod(
    'changing-range-catheter', 8, 1, DEFAULT_TOOL_PROFILES.catheter
);
const changingRangeWire = changingRangeWorld.createRod(
    'changing-range-wire', 8, 1, DEFAULT_TOOL_PROFILES.guidewire
);
seedStraightRod(changingRangeCatheter, 0, 1, 0, 0);
seedStraightRod(changingRangeWire, 0, 1, 0, 0);
const changingRangeContainment = changingRangeWorld.addContainment(
    changingRangeWire,
    changingRangeCatheter,
    { innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius, endNode: 3 }
);
changingRangeContainment.lambdas.fill(1);
changingRangeContainment.closestSegment.fill(3);
changingRangeContainment.endNode = 5;
changingRangeWorld.stepFixed();
assert.ok(changingRangeContainment.lambdas.every(value => value === 0),
    'changing the contained guidewire range should invalidate radial warm-start state');
assert.ok(changingRangeContainment.closestSegment
    .subarray(changingRangeContainment.endNode + 1)
    .every(value => value === -1),
    'changing the contained guidewire range should clear cached segments outside the active range');

const partialContainmentWorld = new EndovascularPhysicsWorld();
const longCatheter = partialContainmentWorld.createRod('long-catheter', 24, 1, DEFAULT_TOOL_PROFILES.catheter);
const partialWire = partialContainmentWorld.createRod('partial-wire', 12, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(longCatheter, 0, 1, 0, 0);
seedStraightRod(partialWire, 0, 1, 1.2, 0);
for (let index = 0; index < longCatheter.count; index++) longCatheter.setPinned(index, true);
partialContainmentWorld.addContainment(partialWire, longCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    searchWindow: 2,
    endNode: partialWire.count - 1
});
for (let step = 0; step < 12; step++) partialContainmentWorld.stepFixed();
assert.ok(partialWire.x[partialWire.count - 1] < 12,
    'a partial guidewire should map by physical distance instead of the full catheter range');
assert.ok(Math.hypot(partialWire.y[partialWire.count - 1], partialWire.z[partialWire.count - 1]) <= containmentClearance + 0.01,
    'the partial guidewire tip should remain radially contained');

const offsetContainmentWorld = new EndovascularPhysicsWorld();
const offsetCatheter = offsetContainmentWorld.createRod('offset-catheter', 16, 1, DEFAULT_TOOL_PROFILES.catheter);
const offsetWire = offsetContainmentWorld.createRod('offset-wire', 6, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(offsetCatheter, -5, 1, 0, 0);
seedStraightRod(offsetWire, 0, 1, 1.2, 0);
for (let index = 0; index < offsetCatheter.count; index++) offsetCatheter.setPinned(index, true);
offsetContainmentWorld.addContainment(offsetWire, offsetCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    outerStartNode: 5,
    searchWindow: 0
});
for (let step = 0; step < 12; step++) offsetContainmentWorld.stepFixed();
assert.ok(Math.abs(offsetWire.x[0]) < 0.05,
    'containment should map the guidewire origin to the catheter lumen start, not its external tail');
assert.ok(Math.hypot(offsetWire.y[0], offsetWire.z[0]) <= containmentClearance + 0.01,
    'offset containment should still enforce radial clearance');

const arcContainmentWorld = new EndovascularPhysicsWorld();
const sparseCatheter = arcContainmentWorld.createRod('sparse-catheter', 6, 1, DEFAULT_TOOL_PROFILES.catheter);
const denseWire = arcContainmentWorld.createRod('dense-wire', 11, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(sparseCatheter, 0, 2, 0, 0);
seedStraightRod(denseWire, 0, 1, 1.2, 0);
for (let index = 0; index < sparseCatheter.count; index++) sparseCatheter.setPinned(index, true);
arcContainmentWorld.addContainment(denseWire, sparseCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    searchWindow: 0
});
for (let step = 0; step < 12; step++) arcContainmentWorld.stepFixed();
assert.ok(Math.abs(denseWire.x[5] - 5) < 0.05,
    'containment should map unequal sampling by accumulated arc length');
assert.ok(Math.hypot(denseWire.y[5], denseWire.z[5]) <= containmentClearance + 0.01,
    'arc-length containment should enforce radial clearance without axial folding');

const foldWorld = new EndovascularPhysicsWorld();
const foldedRod = foldWorld.createRod('fold-limit', 5, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    maxBendAngle: 120,
    foldLimitStrength: 1
});
seedStraightRod(foldedRod, 0, 1, 0, 0);
foldedRod.setNodePosition(2, 0.05, 0, 0);
foldedRod.setNodePosition(3, -0.95, 0, 0);
foldedRod.setNodePosition(4, -1.95, 0, 0);
for (let step = 0; step < 24; step++) foldWorld.stepFixed();
const foldedStats = foldWorld.getStats().bodies[0];
assert.ok(foldedStats.maxBendAngleDegrees <= 122,
    `the XPBD fold limiter should remove a near-180-degree hairpin (got ${foldedStats.maxBendAngleDegrees})`);
assert.ok(foldedStats.maxLengthError <= 0.01,
    'fold repair should retain segment lengths within one percent');

const recoveryWorld = new EndovascularPhysicsWorld();
const recoveringWire = recoveryWorld.createRod('recovering-guidewire', 12, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    sleepFrames: 1000
});
const recoveryRadius = 3;
const recoveryAngle = 2 * Math.asin(1 / (2 * recoveryRadius));
for (let index = 0; index < recoveringWire.count; index++) {
    const angle = index * recoveryAngle;
    recoveringWire.setNodePosition(
        index,
        Math.sin(angle) * recoveryRadius,
        (1 - Math.cos(angle)) * recoveryRadius,
        0
    );
}
recoveringWire.setPinned(0, true);
recoveringWire.copyCurrentToPrevious();
const initialRecoveryBend = recoveryWorld.getStats().bodies[0].maxBendAngleDegrees;
for (let step = 0; step < 480; step++) recoveryWorld.stepFixed();
const recoveredWireStats = recoveryWorld.getStats().bodies[0];
assert.ok(recoveredWireStats.maxBendAngleDegrees < initialRecoveryBend * 0.35,
    `an unloaded guidewire should recover toward straight (got ${recoveredWireStats.maxBendAngleDegrees} degrees)`);
assert.ok(recoveredWireStats.maxLengthError <= 0.01,
    'guidewire straightening should preserve segment length');

const activationWorld = new EndovascularPhysicsWorld();
const activationRod = activationWorld.createRod('activation', 5, 1, DEFAULT_TOOL_PROFILES.guidewire);
activationRod.setActiveRange(0, 1);
activationRod.velocityX[2] = 120;
activationRod.previousX[2] = -10;
activationRod.setActiveRange(0, 2);
assert.equal(activationRod.velocityX[2], 0, 'newly activated nodes should not inherit stale velocity');
assert.equal(activationRod.previousX[2], activationRod.x[2], 'newly activated nodes should reset swept history');

const syncWorld = new EndovascularPhysicsWorld();
const syncedRod = syncWorld.createRod('sync-history', 3, 1, DEFAULT_TOOL_PROFILES.guidewire);
seedStraightRod(syncedRod, 0, 1, 0, 0);
const elasticStorage = {
    x: new Float32Array([0.25, 1.25, 2.25]),
    y: new Float32Array(3),
    z: new Float32Array(3),
    vx: new Float32Array([20, 20, 20]),
    vy: new Float32Array(3),
    vz: new Float32Array(3),
    pinned: new Uint8Array(3),
    mass: new Float32Array([1, 1, 1]),
    bendingStiffness: new Float32Array([1, 1, 1])
};
syncedRod.syncFromElasticRod({ nodeStorage: elasticStorage, nodes: new Array(3) }, {
    resetVelocity: true,
    preservePrevious: true
});
assert.equal(syncedRod.previousX[2], 2, 'kinematic sync should retain the prior pose for swept collision');
assert.equal(syncedRod.x[2], 2.25, 'kinematic sync should install the new input pose');
assert.equal(syncedRod.velocityX[2], 0, 'kinematic sync should not integrate the source pose twice');

const toolWorld = new EndovascularPhysicsWorld();
const toolA = toolWorld.createRod('tool-a', 3, 2, { radius: 0.5, stretchCompliance: 0 });
const toolB = toolWorld.createRod('tool-b', 3, 2, { radius: 0.5, stretchCompliance: 0 });
seedStraightRod(toolA, 0, 2, 0, 0);
seedStraightRod(toolB, 0, 2, 0.2, 0);
for (let index = 0; index < toolB.count; index++) toolB.setPinned(index, true);
toolWorld.addToolContact(toolA, toolB);
for (let step = 0; step < 12; step++) toolWorld.stepFixed();
assert.ok(Math.abs(toolA.y[1] - toolB.y[1]) >= 0.95, 'external tool contact should separate overlapping capsules');

const openToolWorld = new EndovascularPhysicsWorld();
const emergingWire = openToolWorld.createRod('emerging-wire', 2, 2, { radius: 0.45, stretchCompliance: 0 });
const openCatheter = openToolWorld.createRod('open-catheter', 3, 2, { radius: 0.8, stretchCompliance: 0 });
seedStraightRod(openCatheter, 0, 2, 0, 0);
seedStraightRod(emergingWire, 4.2, 2, 0.2, 0);
for (let index = 0; index < openCatheter.count; index++) openCatheter.setPinned(index, true);
openToolWorld.addToolContact(emergingWire, openCatheter, { openDistalB: true });
openToolWorld.stepFixed();
assert.ok(Math.abs(emergingWire.y[0] - 0.2) < 1e-4,
    'wire emerging beyond the distal catheter plane should not collide with a closed capsule cap');

const transitionToolWorld = new EndovascularPhysicsWorld();
const crossingWire = transitionToolWorld.createRod('crossing-wire', 2, 2, {
    radius: 0.45,
    stretchCompliance: 0
});
const transitionCatheter = transitionToolWorld.createRod('transition-catheter', 3, 2, {
    radius: 0.8,
    stretchCompliance: 0
});
seedStraightRod(transitionCatheter, 0, 2, 0, 0);
seedStraightRod(crossingWire, 3.5, 2, 0.2, 0);
for (let index = 0; index < transitionCatheter.count; index++) transitionCatheter.setPinned(index, true);
transitionToolWorld.addToolContact(crossingWire, transitionCatheter, { openDistalB: true });
transitionToolWorld.stepFixed();
assert.ok(crossingWire.y[0] > 0.5,
    'the guidewire segment crossing an open distal tip should still contact the catheter side wall');

function slidingSpeedWithToolFriction(friction) {
    const world = new EndovascularPhysicsWorld();
    const moving = world.createRod('moving', 3, 2, { radius: 0.5, stretchCompliance: 0 });
    const fixed = world.createRod('fixed', 3, 2, { radius: 0.5, stretchCompliance: 0 });
    seedStraightRod(moving, 0, 2, 0.6, 0);
    seedStraightRod(fixed, 0, 2, 0, 0);
    for (let index = 0; index < fixed.count; index++) fixed.setPinned(index, true);
    for (let index = 0; index < moving.count; index++) moving.velocityX[index] = 24;
    world.addToolContact(moving, fixed, { friction });
    world.stepFixed();
    return Math.abs(moving.velocityX[1]);
}

const freeSlidingSpeed = slidingSpeedWithToolFriction(0);
const frictionSlidingSpeed = slidingSpeedWithToolFriction(0.8);
assert.ok(frictionSlidingSpeed < freeSlidingSpeed * 0.8,
    'Coulomb tool friction should reduce relative tangential sliding');

function slidingSpeedWithWallFriction(friction) {
    const world = new EndovascularPhysicsWorld({ contactField: cylinderField(5) });
    const body = world.createRod('wall-friction', 4, 1, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.45,
        stretchCompliance: 0,
        wallFriction: friction,
        linearDamping: 1
    });
    seedStraightRod(body, -1.5, 1, 4.8, 0);
    for (let index = 0; index < body.count; index++) body.velocityX[index] = 24;
    world.stepFixed();
    let speed = 0;
    for (let index = 0; index < body.count; index++) speed += Math.abs(body.velocityX[index]);
    return speed / body.count;
}

const freeWallSlidingSpeed = slidingSpeedWithWallFriction(0);
const frictionWallSlidingSpeed = slidingSpeedWithWallFriction(0.8);
assert.ok(frictionWallSlidingSpeed < freeWallSlidingSpeed * 0.8,
    'Coulomb wall friction should reduce tangential guidewire sliding under contact load');

const sweepField = cylinderField(5);
const sweepWorld = new EndovascularPhysicsWorld({ contactField: sweepField });
const fastRod = sweepWorld.createRod('fast-wire', 3, 1, { radius: 0.45, stretchCompliance: 0 });
seedStraightRod(fastRod, -1, 1, 0, 0);
for (let index = 0; index < fastRod.count; index++) fastRod.velocityY[index] = 2400;
sweepWorld.stepFixed();
assert.ok(maximumGapViolation(sweepField, fastRod) <= 0.2, 'swept contact should prevent tunnelling at 4x control speed');

const catheterGuidewireLength = 160;
const catheterGuidewireSpacing = 2;
const catheterGuidewire = new ElasticRod(
    catheterGuidewireLength / catheterGuidewireSpacing + 1,
    catheterGuidewireSpacing
);
let catheterGuidewireInserted = 0;
const catheterVessel = {
    sheath: {
        start: { x: -20, y: 0, z: 0 },
        end: { x: 0, y: 0, z: 0 },
        radius: DEFAULT_TOOL_PROFILES.sheath.innerRadius
    },
    segments: []
};
const catheterModel = new PigtailCatheter({
    wire: catheterGuidewire,
    segmentLength: catheterGuidewireSpacing,
    guidewireLength: catheterGuidewireLength,
    tailProgressRef: () => catheterGuidewireInserted,
    vessel: catheterVessel,
    maxLength: 120
});
catheterModel.setExternalCollisionSolver(true);

function alignInteractionGuidewire(inserted) {
    catheterGuidewireInserted = inserted;
    for (let index = 0; index < catheterGuidewire.nodes.length; index++) {
        const node = catheterGuidewire.nodes[index];
        node.x = index * catheterGuidewireSpacing - catheterGuidewireLength + inserted;
        node.y = 0;
        node.z = 0;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
    }
}

alignInteractionGuidewire(0);
for (let step = 0; step < 180; step++) {
    catheterModel.advance(1, 1 / 120, 0);
    catheterModel.stepPhysics(1 / 120, { collisions: false });
}
assert.ok(catheterModel.progress > 70,
    'a catheter should advance through the sheath without guidewire support');
assert.ok(catheterModel.freeNodes.length > 8,
    'a catheter inserted without a guidewire should form a physical unsupported shaft and tip');
assert.ok(catheterModel.freeNodes.every(node =>
    Number.isFinite(node.pos.x) && Number.isFinite(node.pos.y) && Number.isFinite(node.pos.z)
), 'a catheter inserted without a guidewire should remain finite');
assert.ok(maximumPointBendDegrees(catheterModel.freeNodes, index => catheterModel.freeNodes[index].pos) <= 74,
    'an unsupported catheter should not develop an acute local fold');
assert.ok(catheterModel.freeNodes.at(-1).pos.x > catheterVessel.sheath.end.x + 15,
    'an unsupported catheter tip should progress distally instead of bunching at the sheath exit');

catheterModel.reset();
alignInteractionGuidewire(80);
for (let step = 0; step < 165; step++) {
    catheterModel.advance(1, 1 / 120, catheterGuidewireInserted);
    catheterModel.stepPhysics(1 / 120, { collisions: false });
}
const supportedLateralOffset = Math.max(
    ...catheterModel.freeNodes.map(node => Math.hypot(node.pos.y, node.pos.z))
);
for (let step = 0; step < 120; step++) {
    alignInteractionGuidewire(80 - 70 * (step + 1) / 120);
    catheterModel.advance(0, 1 / 120, catheterGuidewireInserted);
    catheterModel.stepPhysics(1 / 120, { collisions: false });
}
for (let step = 0; step < 120; step++) {
    catheterModel.advance(0, 1 / 120, catheterGuidewireInserted);
    catheterModel.stepPhysics(1 / 120, { collisions: false });
}
const releasedLateralOffset = Math.max(
    ...catheterModel.freeNodes.map(node => Math.hypot(node.pos.y, node.pos.z))
);
assert.ok(supportedLateralOffset < 1,
    'a guidewire inside the catheter should straighten and support its preformed distal tip');
assert.ok(releasedLateralOffset > supportedLateralOffset + 3,
    'the preformed catheter tip should recover after the guidewire is withdrawn');
catheterModel.dispose();

const stabilityWorld = new EndovascularPhysicsWorld({ contactField: cylinderField(6) });
const stableRod = stabilityWorld.createRod('stability', 32, 0.5, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    stretchCompliance: 0,
    bendCompliance: 2e-4
});
seedStraightRod(stableRod, -7.75, 0.5, 4.8, 0);
stableRod.setPinned(0, true);
for (let step = 0; step < 10000; step++) {
    const controlY = 4.2 + Math.sin(step * 0.004) * 0.7;
    stableRod.setControlTarget(stableRod.count - 1, stableRod.x[stableRod.count - 1], controlY, 0, 2e-5);
    stabilityWorld.stepFixed();
}
const stabilityStats = stabilityWorld.getStats();
assert.equal(stabilityStats.bodies[0].finite, true, '10,000 XPBD steps should not produce NaN');
assert.ok(stabilityStats.bodies[0].maxLengthError <= 0.01, '10,000 XPBD steps should retain segment length within 1%');
assert.ok(stabilityStats.bodies[0].maxBendAngleDegrees < 150, '10,000 XPBD steps should not create acute folds');

const sleepWorld = new EndovascularPhysicsWorld();
const sleepRod = sleepWorld.createRod('sleep', 4, 1, {
    ...DEFAULT_TOOL_PROFILES.guidewire,
    sleepFrames: 3,
    sleepVelocity: 0.01
});
seedStraightRod(sleepRod, 0, 1, 0, 0);
for (let step = 0; step < 4; step++) sleepWorld.stepFixed();
assert.equal(sleepRod.sleeping, true, 'an unchanged stable rod should enter sleep');
sleepRod.setControlTarget(3, sleepRod.x[3], 0.5, 0, 0);
assert.equal(sleepRod.sleeping, false, 'changing a control target should wake a sleeping rod');
sleepWorld.stepFixed();
assert.ok(sleepRod.y[3] > 0, 'a woken rod should respond to its new control target');

function runRenderRateReplay(renderFps) {
    const world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120, maxSubsteps: 2 });
    const body = world.createRod(`render-${renderFps}`, 24, 0.75, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        stretchCompliance: 0,
        bendCompliance: 3e-4
    });
    seedStraightRod(body, -8.625, 0.75, 0, 0);
    body.setPinned(0, true);
    while (world.stepCount < 600) {
        world.advance(1 / renderFps, () => {
            const phase = world.stepCount * 0.011;
            const tip = body.count - 1;
            body.setControlTarget(
                tip,
                body.x[tip],
                Math.sin(phase) * 0.8,
                Math.cos(phase * 0.7) * 0.45,
                2e-5
            );
        });
    }
    return {
        x: body.x.slice(),
        y: body.y.slice(),
        z: body.z.slice(),
        stats: world.getStats()
    };
}

const replay30 = runRenderRateReplay(30);
const replay60 = runRenderRateReplay(60);
const replay120 = runRenderRateReplay(120);
for (let index = 0; index < replay30.x.length; index++) {
    for (const key of ['x', 'y', 'z']) {
        assert.ok(Math.abs(replay30[key][index] - replay60[key][index]) <= 1e-6,
            `30 and 60 FPS replay should match for ${key}[${index}]`);
        assert.ok(Math.abs(replay60[key][index] - replay120[key][index]) <= 1e-6,
            `60 and 120 FPS replay should match for ${key}[${index}]`);
    }
}
assert.equal(replay30.stats.lastSubsteps, 2, '30 FPS rendering should still cap physics at two substeps');
assert.equal(replay60.stats.lastSubsteps, 2, '60 FPS rendering should use two 1/120 s substeps');
assert.equal(replay120.stats.lastSubsteps, 1, '120 FPS rendering should use one 1/120 s substep');

console.log('xpbd stability total average ms', stabilityStats.phases.total.averageMs.toFixed(4));
console.log('xpbd stability total p95 ms', stabilityStats.phases.total.p95Ms.toFixed(4));
console.log('xpbd stability max length error %', (stabilityStats.bodies[0].maxLengthError * 100).toFixed(4));

stableRod.wallLambda.fill(1);
stabilityWorld.resetSimulationState();
assert.equal(stabilityWorld.stepCount, 0, 'simulation reset should restore the step counter');
assert.equal(stabilityWorld.droppedTime, 0, 'simulation reset should clear dropped time');
assert.ok(stableRod.wallLambda.every(value => value === 0), 'simulation reset should clear warm-started wall contacts');
