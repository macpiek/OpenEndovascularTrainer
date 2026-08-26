import assert from 'node:assert/strict';
import { ElasticRod } from '../src/physics/elasticRod.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import { PIGTAIL_CATHETER_RENDER_RADIUS_MM } from '../src/toolDimensions.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { GUIDEWIRE_TYPE_GLIDEWIRE } from '../src/physics/guidewireMaterialProfile.js';
import { PIGTAIL_NATURAL_ARC_LENGTH_MM } from '../src/physics/catheterMaterialProfile.js';

const EPSILON = 1e-8;

function pigtailLoopMetrics(
    body,
    catheter,
    naturalArcLength = PIGTAIL_NATURAL_ARC_LENGTH_MM
) {
    let baseIndex = body.activeEnd;
    let materialLength = 0;
    while (baseIndex > body.activeStart && materialLength < naturalArcLength) {
        baseIndex--;
        materialLength += body.restLength[baseIndex];
    }

    const directions = [];
    let maximumSpan = 0;
    for (let index = baseIndex; index <= body.activeEnd; index++) {
        for (let other = baseIndex; other < index; other++) {
            maximumSpan = Math.max(maximumSpan, Math.hypot(
                body.x[index] - body.x[other],
                body.y[index] - body.y[other],
                body.z[index] - body.z[other]
            ));
        }
        if (index === baseIndex) continue;
        const dx = body.x[index] - body.x[index - 1];
        const dy = body.y[index] - body.y[index - 1];
        const dz = body.z[index] - body.z[index - 1];
        const length = Math.max(EPSILON, Math.hypot(dx, dy, dz));
        directions.push([dx / length, dy / length, dz / length]);
    }

    let totalTurnDegrees = 0;
    for (let index = 1; index < directions.length; index++) {
        totalTurnDegrees += Math.acos(Math.max(-1, Math.min(1,
            directions[index - 1][0] * directions[index][0] +
            directions[index - 1][1] * directions[index][1] +
            directions[index - 1][2] * directions[index][2]
        ))) * 180 / Math.PI;
    }
    const closureDistance = Math.hypot(
        body.x[body.activeEnd] - body.x[baseIndex],
        body.y[body.activeEnd] - body.y[baseIndex],
        body.z[body.activeEnd] - body.z[baseIndex]
    );
    const targetDirections = [];
    const targetPoints = [[0, 0, 0]];
    let targetMaximumSpan = 0;
    for (let segment = baseIndex; segment < body.activeEnd; segment++) {
        if (!body.restDirectionEnabled[segment]) continue;
        const dx = body.restDirectionX[segment];
        const dy = body.restDirectionY[segment];
        const dz = body.restDirectionZ[segment];
        const length = Math.max(EPSILON, Math.hypot(dx, dy, dz));
        targetDirections.push([dx / length, dy / length, dz / length]);
        const previous = targetPoints[targetPoints.length - 1];
        targetPoints.push([
            previous[0] + dx,
            previous[1] + dy,
            previous[2] + dz
        ]);
    }
    for (let index = 0; index < targetPoints.length; index++) {
        for (let other = 0; other < index; other++) {
            targetMaximumSpan = Math.max(targetMaximumSpan, Math.hypot(
                targetPoints[index][0] - targetPoints[other][0],
                targetPoints[index][1] - targetPoints[other][1],
                targetPoints[index][2] - targetPoints[other][2]
            ));
        }
    }
    let targetTurnDegrees = 0;
    for (let index = 1; index < targetDirections.length; index++) {
        targetTurnDegrees += Math.acos(Math.max(-1, Math.min(1,
            targetDirections[index - 1][0] * targetDirections[index][0] +
            targetDirections[index - 1][1] * targetDirections[index][1] +
            targetDirections[index - 1][2] * targetDirections[index][2]
        ))) * 180 / Math.PI;
    }
    const targetTip = targetPoints[targetPoints.length - 1];
    const targetClosureDistance = targetDirections.length
        ? Math.hypot(targetTip[0], targetTip[1], targetTip[2])
        : Infinity;
    return {
        baseIndex,
        nodeCount: body.activeEnd - baseIndex + 1,
        totalTurnDegrees,
        maximumSpan,
        closureDistance,
        closureRatio: closureDistance / Math.max(EPSILON, maximumSpan),
        targetTurnDegrees,
        targetMaximumSpan,
        targetClosureDistance,
        targetClosureRatio: targetClosureDistance / Math.max(EPSILON, targetMaximumSpan)
    };
}

function setVector(target, x, y, z) {
    target.x = x;
    target.y = y;
    target.z = z;
}

function distanceToRodSegments(pointBody, pointIndex, rod, startSegment = 0, endSegment = rod.segmentCount - 1) {
    let best = Infinity;
    for (let segment = startSegment; segment <= endSegment; segment++) {
        const ax = rod.x[segment];
        const ay = rod.y[segment];
        const az = rod.z[segment];
        const dx = rod.x[segment + 1] - ax;
        const dy = rod.y[segment + 1] - ay;
        const dz = rod.z[segment + 1] - az;
        const lengthSq = dx * dx + dy * dy + dz * dz;
        const t = Math.max(0, Math.min(1, (
            (pointBody.x[pointIndex] - ax) * dx +
            (pointBody.y[pointIndex] - ay) * dy +
            (pointBody.z[pointIndex] - az) * dz
        ) / Math.max(EPSILON, lengthSq)));
        best = Math.min(best, Math.hypot(
            pointBody.x[pointIndex] - (ax + dx * t),
            pointBody.y[pointIndex] - (ay + dy * t),
            pointBody.z[pointIndex] - (az + dz * t)
        ));
    }
    return best;
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

function aorticArchState(
    x,
    y,
    z,
    {
        straightEnd = 30,
        archRadius = 25,
        lumenRadius = 12
    } = {}
) {
    const archCenterY = archRadius;
    const archEndX = straightEnd + archRadius;
    const candidates = [];

    candidates.push({
        x: Math.min(x, straightEnd),
        y: 0,
        progress: Math.min(x, straightEnd),
        tangentX: 1,
        tangentY: 0
    });

    const rawAngle = Math.atan2(y - archCenterY, x - straightEnd);
    const angle = Math.max(-Math.PI * 0.5, Math.min(0, rawAngle));
    candidates.push({
        x: straightEnd + Math.cos(angle) * archRadius,
        y: archCenterY + Math.sin(angle) * archRadius,
        progress: straightEnd + (angle + Math.PI * 0.5) * archRadius,
        tangentX: -Math.sin(angle),
        tangentY: Math.cos(angle)
    });

    candidates.push({
        x: archEndX,
        y: Math.max(archCenterY, y),
        progress: straightEnd + Math.PI * 0.5 * archRadius +
            Math.max(0, y - archCenterY),
        tangentX: 0,
        tangentY: 1
    });

    let closest = candidates[0];
    let closestDistanceSq = Infinity;
    for (const candidate of candidates) {
        const dx = candidate.x - x;
        const dy = candidate.y - y;
        const distanceSq = dx * dx + dy * dy + z * z;
        if (distanceSq >= closestDistanceSq) continue;
        closest = candidate;
        closestDistanceSq = distanceSq;
    }

    const dx = closest.x - x;
    const dy = closest.y - y;
    const dz = -z;
    const radial = Math.hypot(dx, dy, dz);
    const inverse = radial > EPSILON ? 1 / radial : 0;
    return {
        distance: lumenRadius - radial,
        inwardX: radial > EPSILON ? dx * inverse : 0,
        inwardY: radial > EPSILON ? dy * inverse : 0,
        inwardZ: radial > EPSILON ? dz * inverse : 1,
        branchId: 0,
        progress: closest.progress,
        radial,
        tangentX: closest.tangentX,
        tangentY: closest.tangentY,
        tangentZ: 0
    };
}

function aorticArchField(options = {}) {
    const field = new AnalyticContactField((x, y, z) =>
        aorticArchState(x, y, z, options)
    );
    let tangentQueryCount = 0;
    field.getCenterlineTangent = (x, y, z) => {
        const state = aorticArchState(x, y, z, options);
        const noiseRadians = (options.tangentNoiseDegrees ?? 0) * Math.PI / 180;
        const noiseSign = (tangentQueryCount++ & 1) === 0 ? -1 : 1;
        const cosine = Math.cos(noiseRadians * noiseSign);
        const sine = Math.sin(noiseRadians * noiseSign);
        return {
            x: state.tangentX * cosine - state.tangentY * sine,
            y: state.tangentX * sine + state.tangentY * cosine,
            z: state.tangentZ
        };
    };
    return field;
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

function reversePlanarBendDegrees(body, start, end) {
    let total = 0;
    let maximum = 0;
    for (let index = Math.max(start + 1, 1); index < Math.min(end, body.activeEnd); index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const signedDegrees = Math.atan2(ax * by - ay * bx, ax * bx + ay * by) * 180 / Math.PI;
        if (signedDegrees >= 0) continue;
        total += -signedDegrees;
        maximum = Math.max(maximum, -signedDegrees);
    }
    return { total, maximum };
}

function alternatingPlanarBendDegrees(body, start, end) {
    let previous = 0;
    let total = 0;
    let maximum = 0;
    for (let index = Math.max(start + 1, 1); index < Math.min(end, body.activeEnd); index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const signedDegrees = Math.atan2(ax * by - ay * bx, ax * bx + ay * by) * 180 / Math.PI;
        if (Math.abs(signedDegrees) < 0.2) continue;
        if (previous * signedDegrees < 0) {
            const reversal = Math.min(Math.abs(previous), Math.abs(signedDegrees));
            total += reversal;
            maximum = Math.max(maximum, reversal);
        }
        previous = signedDegrees;
    }
    return { total, maximum };
}

function maximumBodySpeed(body) {
    let maximum = 0;
    for (let index = body.activeStart; index <= body.activeEnd; index++) {
        maximum = Math.max(maximum, Math.hypot(
            body.velocityX[index],
            body.velocityY[index],
            body.velocityZ[index]
        ));
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

const tangentialReleaseWorld = new EndovascularPhysicsWorld({
    contactField: cylinderField(5),
    iterations: 6
});
const tangentialReleaseCatheter = tangentialReleaseWorld.createRod(
    'tangential-release-catheter',
    20,
    1,
    {
        ...DEFAULT_TOOL_PROFILES.catheter,
        radius: 0.8,
        stretchCompliance: 0,
        bendCompliance: 1e-6,
        wallFrictionUsesCurrentLoad: true
    }
);
seedStraightRod(tangentialReleaseCatheter, -9.5, 1, 4.2, 0);
tangentialReleaseCatheter.captureRestConfiguration();
tangentialReleaseCatheter.copyCurrentToPrevious();
tangentialReleaseWorld.stepFixed();
assert.ok(tangentialReleaseCatheter.wallActive.some(value => value !== 0),
    'a catheter tangent to the lumen wall should create a cached contact');
// A warm-start multiplier belongs to positional contact convergence. It must
// not become a residual normal force that blocks later tangential relaxation.
tangentialReleaseCatheter.wallLambda.fill(25);
tangentialReleaseCatheter.velocityX.fill(12);
const tangentialReleaseStartX = tangentialReleaseCatheter.x[10];
tangentialReleaseWorld.stepFixed();
// The first integrated step still contains the velocity that existed before
// friction reconstruction. A second step proves whether stale contact erased
// that tangential velocity and pinned the catheter for subsequent motion.
tangentialReleaseWorld.stepFixed();
const tangentialReleaseDistance =
    tangentialReleaseCatheter.x[10] - tangentialReleaseStartX;
console.log('tangential cached-contact release mm',
    tangentialReleaseDistance.toFixed(4));
assert.ok(tangentialReleaseDistance >= 0.16,
    `cached wall contact must not pin tangential catheter motion (${tangentialReleaseDistance} mm)`);

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
sheathRod.setCollisionRange(
    sheathRod.segmentCount - 1,
    sheathRod.segmentCount - 1
);
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
sheathWire.setCollisionRange(
    sheathWire.segmentCount - 1,
    sheathWire.segmentCount - 1
);
sheathCatheter.setCollisionRange(
    sheathCatheter.segmentCount - 1,
    sheathCatheter.segmentCount - 1
);
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

const movingContainmentWorld = new EndovascularPhysicsWorld();
const movingCatheter = movingContainmentWorld.createRod(
    'moving-catheter', 14, 1, DEFAULT_TOOL_PROFILES.catheter
);
const movingWire = movingContainmentWorld.createRod(
    'moving-wire', 14, 1, DEFAULT_TOOL_PROFILES.guidewire
);
seedStraightRod(movingCatheter, 0, 1, 0, 0);
seedStraightRod(movingWire, 0, 1, 0, 0);
for (let index = 0; index < movingCatheter.count; index++) movingCatheter.setPinned(index, true);
const movingContainment = movingContainmentWorld.addContainment(movingWire, movingCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    openProximal: false,
    openDistal: false,
    searchWindow: movingCatheter.segmentCount
});
let maximumMovingContainmentEscape = 0;
for (let step = 0; step < 48; step++) {
    movingContainment.startNode = step < 16 ? 0 : step < 32 ? 2 : 1;
    movingContainment.endNode = step < 24 ? 11 : 13;
    for (let index = 0; index < movingCatheter.count; index++) {
        movingCatheter.setNodePosition(
            index,
            index,
            Math.sin(index * 0.32 + step * 0.11) * 0.65,
            Math.cos(index * 0.21 - step * 0.07) * 0.35
        );
    }
    for (let index = movingContainment.startNode; index <= movingContainment.endNode; index++) {
        movingWire.forceY[index] += (index & 1 ? -1 : 1) * 180;
        movingWire.forceZ[index] += Math.sin(step * 0.3 + index) * 120;
    }
    movingContainmentWorld.stepFixed();
    for (let index = movingContainment.startNode; index <= movingContainment.endNode; index++) {
        const escape = distanceToRodSegments(movingWire, index, movingCatheter) - containmentClearance;
        if (escape > maximumMovingContainmentEscape) {
            maximumMovingContainmentEscape = escape;
        }
    }
}
assert.ok(maximumMovingContainmentEscape <= 1e-3,
    `a guidewire must finish every moving-catheter step inside the lumen (${maximumMovingContainmentEscape} mm escape)`);
const movingContainmentLengthError = movingContainmentWorld.getStats().bodies
    .find(body => body.id === 'moving-wire').maxLengthError;
if (movingContainmentLengthError > 0.02) {
    const errors = [];
    for (let index = movingWire.activeStart; index < movingWire.activeEnd; index++) {
        errors.push([
            index,
            Math.hypot(
                movingWire.x[index + 1] - movingWire.x[index],
                movingWire.y[index + 1] - movingWire.y[index],
                movingWire.z[index + 1] - movingWire.z[index]
            ),
            movingWire.restLength[index]
        ]);
    }
    console.log('moving containment length errors', errors);
}
assert.ok(movingContainmentLengthError <= 0.02,
    `containment must not stretch the guidewire (${movingContainmentLengthError * 100}% segment error)`);

const supportedContainmentWorld = new EndovascularPhysicsWorld();
const supportingWire = supportedContainmentWorld.createRod(
    'supporting-wire', 18, 1, DEFAULT_TOOL_PROFILES.guidewire
);
const supportedCatheter = supportedContainmentWorld.createRod(
    'supported-catheter', 18, 1, DEFAULT_TOOL_PROFILES.catheter
);
seedStraightRod(supportingWire, 0, 1, 0, 0);
seedStraightRod(supportedCatheter, 0, 1, 0.7, 0);
for (let index = 0; index < supportingWire.count; index++) supportingWire.setPinned(index, true);
supportedContainmentWorld.addContainment(supportingWire, supportedCatheter, {
    innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
    openProximal: false,
    openDistal: false,
    searchWindow: 2,
    innerResponse: 0,
    outerResponse: 1,
    finalProjection: 'outer',
    outerFollowsInnerCenterline: true,
    innerArcOffset: 0,
    containedLength: supportingWire.segmentCount
});
let maximumSupportedEscape = 0;
for (let step = 0; step < 48; step++) {
    for (let index = 0; index < supportedCatheter.count; index++) {
        supportedCatheter.forceY[index] += 90 * Math.sin(step * 0.17 + index * 0.31);
        supportedCatheter.forceZ[index] += 70 * Math.cos(step * 0.13 - index * 0.27);
    }
    supportedContainmentWorld.stepFixed();
    for (let index = 0; index < supportingWire.count; index++) {
        maximumSupportedEscape = Math.max(
            maximumSupportedEscape,
            distanceToRodSegments(supportingWire, index, supportedCatheter) - containmentClearance
        );
    }
}
assert.ok(maximumSupportedEscape <= 0.01,
    `a catheter advancing around a stiff guidewire must retain the wire in its lumen (${maximumSupportedEscape} mm escape)`);
const supportedWireLengthError = supportedContainmentWorld.getStats().bodies
    .find(body => body.id === 'supporting-wire').maxLengthError;
assert.ok(supportedWireLengthError <= 1e-6,
    'catheter containment must not stretch its supporting guidewire');

function alternatingVelocityAfterStep(bendDamping) {
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('velocity-wave', 15, 1, {
        radius: 0.1,
        mass: 1,
        stretchCompliance: 1,
        bendCompliance: 1,
        foldLimitStrength: 0,
        linearDamping: 1,
        bendDamping,
        sleepFrames: 1000
    });
    seedStraightRod(body, 0, 1, 0, 0);
    for (let index = 1; index < body.count - 1; index++) {
        body.velocityY[index] = index & 1 ? 18 : -18;
    }
    world.stepFixed();
    let energy = 0;
    for (let index = 1; index < body.count - 1; index++) {
        energy += body.velocityY[index] * body.velocityY[index];
    }
    return Math.sqrt(energy / (body.count - 2));
}
const undampedVelocityWave = alternatingVelocityAfterStep(0);
const dampedVelocityWave = alternatingVelocityAfterStep(DEFAULT_TOOL_PROFILES.guidewire.bendDamping);
assert.ok(dampedVelocityWave < undampedVelocityWave * 0.9,
    'guidewire bend damping should suppress local transverse waves during insertion');
assert.ok(DEFAULT_TOOL_PROFILES.guidewire.bendDamping >= 0.25,
    'guidewire damping should suppress visible transverse waves at full insertion');
assert.equal(DEFAULT_TOOL_PROFILES.guidewire.maxSpeed, Infinity,
    'operator feed must not cap the guidewire material recovery velocity');

const fullInsertionStabilityWorld = new EndovascularPhysicsWorld();
const fullInsertionWire = fullInsertionStabilityWorld.createRod(
    'fully-inserted-guidewire',
    180,
    1,
    {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        sleepFrames: 1000
    }
);
seedStraightRod(fullInsertionWire, 0, 1, 0, 0);
fullInsertionWire.setPinned(0, true);
for (let index = 1; index < fullInsertionWire.count - 1; index++) {
    fullInsertionWire.velocityY[index] = Math.sin(index * 1.7) * 18;
    fullInsertionWire.velocityZ[index] = Math.cos(index * 1.3) * 18;
}
for (let step = 0; step < 240; step++) fullInsertionStabilityWorld.stepFixed();
let settledGuidewireRmsSpeed = 0;
for (let index = 1; index < fullInsertionWire.count; index++) {
    settledGuidewireRmsSpeed +=
        fullInsertionWire.velocityX[index] ** 2 +
        fullInsertionWire.velocityY[index] ** 2 +
        fullInsertionWire.velocityZ[index] ** 2;
}
settledGuidewireRmsSpeed = Math.sqrt(
    settledGuidewireRmsSpeed / (fullInsertionWire.count - 1)
);
assert.ok(
    settledGuidewireRmsSpeed < 0.5,
    `a fully inserted guidewire should dissipate transverse oscillation (${settledGuidewireRmsSpeed} mm/s)`
);
assert.ok(DEFAULT_TOOL_PROFILES.catheter.bendCompliance <= 1.5e-5,
    'the catheter shaft should use a load-bearing bending profile');
assert.ok(
    DEFAULT_TOOL_PROFILES.catheter.shapeCompliance >= 8e-5 &&
    DEFAULT_TOOL_PROFILES.catheter.shapeCompliance <= 2e-4,
    'the preformed tip should recover its shape without overpowering the vessel wall'
);
assert.ok(DEFAULT_TOOL_PROFILES.catheter.bendDamping >= 0.6,
    'the catheter shaft should damp visible high-frequency flexing');
assert.ok(DEFAULT_TOOL_PROFILES.catheter.wallFriction <= 0.08,
    'the catheter should slide against the wall without stick-slip jumping');
assert.equal(DEFAULT_TOOL_PROFILES.catheter.maxSpeed, Infinity,
    'catheter feed must remain independent from constitutive material velocity');
assert.ok(DEFAULT_TOOL_PROFILES.catheter.postStabilizationPasses >= 4,
    'the catheter should receive final bend polishing after wall projection');

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

const profiledRod = syncWorld.createRod(
    'profiled-guidewire',
    5,
    1,
    DEFAULT_TOOL_PROFILES.guidewire
);
const profiledStorage = {
    x: new Float32Array([0, 1, 2, 3, 4]),
    y: new Float32Array(5),
    z: new Float32Array(5),
    vx: new Float32Array(5),
    vy: new Float32Array(5),
    vz: new Float32Array(5),
    pinned: new Uint8Array(5),
    mass: new Float32Array([1, 1, 1, 1, 1]),
    bendingStiffness: new Float32Array([16384, 16384, 16384, 64, 64]),
    bendAngleLimit: new Float32Array([18, 18, 18, 135, 135])
};
profiledRod.syncFromElasticRod(
    { nodeStorage: profiledStorage, nodes: new Array(5) },
    { resetVelocity: false }
);
assert.ok(
    profiledRod.bendComplianceByNode[3] >= profiledRod.bendComplianceByNode[1] * 255,
    'the distal guidewire tip should be much more flexible than the load-bearing shaft'
);
assert.equal(
    profiledRod.maxBendAngleByNode[1],
    18,
    'the stiff guidewire shaft should reject tight local coils'
);
assert.equal(
    profiledRod.maxBendAngleByNode[3],
    135,
    'the final 20 mm should retain the permissive soft-tip bend limit'
);

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

function slidingSpeedWithSplitWallFriction({
    staticFriction,
    kineticFriction,
    initialSpeed = 24
}) {
    const world = new EndovascularPhysicsWorld({ contactField: cylinderField(5) });
    const body = world.createRod('split-wall-friction', 4, 1, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.45,
        stretchCompliance: 0,
        wallStaticFriction: staticFriction,
        wallKineticFriction: kineticFriction,
        linearDamping: 1
    });
    seedStraightRod(body, -1.5, 1, 4.8, 0);
    body.velocityX.fill(initialSpeed);
    world.stepFixed();
    let speed = 0;
    for (let index = 0; index < body.count; index++) {
        speed += Math.abs(body.velocityX[index]);
    }
    return speed / body.count;
}

const freeWallSlidingSpeed = slidingSpeedWithWallFriction(0);
const frictionWallSlidingSpeed = slidingSpeedWithWallFriction(0.8);
assert.ok(frictionWallSlidingSpeed < freeWallSlidingSpeed * 0.8,
    'Coulomb wall friction should reduce tangential guidewire sliding under contact load');

const splitFreeSlidingSpeed = slidingSpeedWithSplitWallFriction({
    staticFriction: 0,
    kineticFriction: 0
});
const hydrophilicSlidingSpeed = slidingSpeedWithSplitWallFriction({
    staticFriction: 0.006,
    kineticFriction: 0.002
});
assert.ok(
    hydrophilicSlidingSpeed > splitFreeSlidingSpeed * 0.95,
    `a moving hydrophilic Glidewire should slide almost freely along the wall (` +
    `${hydrophilicSlidingSpeed} vs ${splitFreeSlidingSpeed})`
);

// A straight-rest Kirchhoff wire in grazing wall contact must be able to
// relax tangential curvature. Wall non-penetration remains active, but contact
// must not globally erase the elastic recovery velocity.
{
    const world = new EndovascularPhysicsWorld({
        contactField: cylinderField(5),
        fixedDt: 1 / 120,
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod('wall-contact-straight-recovery', 31, 4, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        rodModel: 'kirchhoff',
        adaptationCompliance: 0,
        foldLimitStrength: 0,
        maxBendAngle: 179,
        wallStaticFriction: 0.006,
        wallKineticFriction: 0.002,
        linearDamping: 0.98,
        angularDamping: 0.96,
        projectionVelocityRetention: 1,
        sleepVelocity: 1,
        sleepAngularVelocity: 0.015,
        sleepFrames: 10,
        postStabilizationPasses: 0
    });
    for (let index = 0; index < body.count; index++) {
        const phase = index / (body.count - 1) * Math.PI * 2;
        body.setNodePosition(
            index,
            (index - 15) * 4,
            4.57,
            Math.sin(phase) * 0.35
        );
    }
    body.copyCurrentToPrevious();
    body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    applyKirchhoffMaterialProfile(body, GUIDEWIRE_TYPE_GLIDEWIRE, {
        materialCoordinates: body.materialCoordinate,
        tipCoordinate: 200
    });
    body.setPinned(0, true);

    const totalBend = () => {
        let result = 0;
        for (let joint = 1; joint < body.activeEnd; joint++) {
            const ax = body.x[joint] - body.x[joint - 1];
            const ay = body.y[joint] - body.y[joint - 1];
            const az = body.z[joint] - body.z[joint - 1];
            const bx = body.x[joint + 1] - body.x[joint];
            const by = body.y[joint + 1] - body.y[joint];
            const bz = body.z[joint + 1] - body.z[joint];
            result += Math.acos(Math.max(-1, Math.min(1,
                (ax * bx + ay * by + az * bz) /
                Math.max(EPSILON, Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
            )));
        }
        return result;
    };

    const initialBend = totalBend();
    let maximumPenetration = 0;
    for (let step = 0; step < 480; step++) {
        world.stepFixed();
        maximumPenetration = Math.max(
            maximumPenetration,
            maximumGapViolation(world.contactField, body)
        );
    }
    const finalBend = totalBend();
    assert.ok(
        finalBend < initialBend * 0.7,
        `wall contact froze Glidewire straightening (${initialBend} -> ${finalBend})`
    );
    assert.ok(maximumPenetration <= 0.05,
        `sliding recovery exceeded wall penetration tolerance (${maximumPenetration})`);
}

function wallReactionFixture(iterations, postStabilizationPasses) {
    const world = new EndovascularPhysicsWorld({
        contactField: cylinderField(5),
        iterations,
        penetrationIterations: iterations
    });
    const body = world.createRod(
        `wall-reaction-${iterations}-${postStabilizationPasses}`,
        20,
        1,
        {
            ...DEFAULT_TOOL_PROFILES.catheter,
            radius: 0.8,
            stretchCompliance: 0,
            bendCompliance: 1e-6,
            wallFriction: 0.2,
            wallFrictionUsesCurrentLoad: true,
            wallFrictionUsesSmoothedLoad: false,
            linearDamping: 1,
            postStabilizationPasses,
            postStabilizationMinPasses: 0
        }
    );
    seedStraightRod(body, -9.5, 1, 4.3, 0);
    body.captureRestConfiguration();
    body.copyCurrentToPrevious();
    body.velocityX.fill(12);
    world.stepFixed();
    let reaction = 0;
    let axialTravel = 0;
    for (let segment = 0; segment < body.segmentCount; segment++) {
        reaction += body.wallFrictionLambda[segment];
    }
    for (let node = 0; node < body.count; node++) {
        axialTravel += body.x[node] - (node - 9.5);
    }
    return {
        reaction,
        axialTravel: axialTravel / body.count,
        penetration: maximumGapViolation(cylinderField(5), body)
    };
}

const wallReactionVariants = [];
for (const iterations of [4, 8, 16]) {
    for (const postPasses of [0, 4, 12]) {
        wallReactionVariants.push(wallReactionFixture(iterations, postPasses));
    }
}
const wallReactionValues = wallReactionVariants.map(result => result.reaction);
const wallTravelValues = wallReactionVariants.map(result => result.axialTravel);
const minimumWallReaction = Math.min(...wallReactionValues);
const maximumWallReaction = Math.max(...wallReactionValues);
assert.ok(minimumWallReaction > 0.05,
    'a wall-loaded catheter must produce a measurable final normal reaction');
assert.ok(
    maximumWallReaction - minimumWallReaction <= maximumWallReaction * 0.1,
    `normal reaction must be iteration-invariant (${minimumWallReaction}..${maximumWallReaction})`
);
assert.ok(
    Math.max(...wallTravelValues) - Math.min(...wallTravelValues) <= 0.05,
    'Coulomb travel must not be renewed by additional solver passes'
);
assert.ok(
    Math.max(...wallReactionVariants.map(result => result.penetration)) <= 0.05,
    'iteration-invariant wall reaction must still resolve penetration'
);

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
const unsupportedDistalReach = Math.max(...catheterModel.freeNodes.map(node => node.pos.x));
assert.ok(unsupportedDistalReach > catheterVessel.sheath.end.x + 20,
    `an unsupported catheter should progress distally instead of bunching at the sheath exit (${unsupportedDistalReach} mm)`);

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

const insertionGuidewireLength = 500;
const insertionGuidewireSpacing = 2;
let insertionGuidewireInserted = 199;
const insertionSheathLength = Math.hypot(
    catheterVessel.sheath.end.x - catheterVessel.sheath.start.x,
    catheterVessel.sheath.end.y - catheterVessel.sheath.start.y,
    catheterVessel.sheath.end.z - catheterVessel.sheath.start.z
);
const insertionGuidewire = new ElasticRod(
    insertionGuidewireLength / insertionGuidewireSpacing + 1,
    insertionGuidewireSpacing
);
for (let index = 0; index < insertionGuidewire.nodes.length; index++) {
    const node = insertionGuidewire.nodes[index];
    node.x = index * insertionGuidewireSpacing - insertionGuidewireLength +
        insertionGuidewireInserted - insertionSheathLength;
    node.y = 0;
    node.z = 0;
    node.vx = 0;
    node.vy = 0;
    node.vz = 0;
}
const insertionCatheter = new PigtailCatheter({
    wire: insertionGuidewire,
    segmentLength: insertionGuidewireSpacing,
    guidewireLength: insertionGuidewireLength,
    tailProgressRef: () => insertionGuidewireInserted,
    vessel: catheterVessel,
    maxLength: insertionGuidewireLength
});
insertionCatheter.setExternalCollisionSolver(true);
// A narrow aortic-lumen surrogate forces the deployed pigtail to share space
// with the wall, reproducing the former wall-versus-shape-memory oscillation.
const insertionWorld = new EndovascularPhysicsWorld({
    contactField: cylinderField(14.5),
    iterations: 6,
    penetrationIterations: 8
});
const insertionWireBody = insertionWorld.createRod(
    'insertion-guidewire',
    insertionGuidewire.nodes.length,
    insertionGuidewireSpacing,
    DEFAULT_TOOL_PROFILES.guidewire
);
for (let index = 0; index < insertionWireBody.count; index++) {
    const node = insertionGuidewire.nodes[index];
    insertionWireBody.setNodePosition(index, node.x, node.y, node.z);
}
insertionWireBody.setActiveRange(
    Math.max(0, Math.ceil(
        (insertionGuidewireLength - insertionGuidewireInserted) / insertionGuidewireSpacing
    ) - 1),
    insertionWireBody.count - 1
);
const insertionCatheterBody = insertionWorld.createRod('insertion-catheter', 320, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
const insertionContainment = insertionWorld.addContainment(
    insertionWireBody,
    insertionCatheterBody,
    {
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        openProximal: true,
        openDistal: true,
        searchWindow: 2,
        innerResponse: 1,
        outerResponse: 0,
        finalProjection: 'inner',
        outerFollowsInnerCenterline: false,
        // Match the application mode for a catheter advancing over a
        // stationary guidewire: the opening remains active, but its reaction
        // belongs to the moving outer catheter rather than to wire extrusion.
        enforceDistalPortal: true,
        portalInnerResponse: 0,
        portalOuterResponse: 0.2,
        preserveStationaryInnerLength: true,
        enabled: false
    }
);
const insertionExternalContact = insertionWorld.addToolContact(
    insertionWireBody,
    insertionCatheterBody,
    {
        friction: 0.08,
        openDistalB: true,
        enabled: false
    }
);
let previousInsertionTip = null;
let maximumInsertionTipStep = 0;
let maximumInsertionTipStepAt = -1;
let maximumInsertionTipProgress = 0;
let maximumInsertionTipActiveEnd = -1;
let maximumInsertionTipCollisionStart = -1;
let maximumInsertionSpeed = 0;
let maximumInsertionSpeedStep = -1;
let maximumInsertionSpeedNode = -1;
let maximumInsertionSpeedProgress = 0;
let maximumInsertionBend = 0;
let maximumInsertionBendStep = -1;
let maximumInsertionBendNode = -1;
let maximumInsertionBendProgress = 0;
let maximumInsertionBendActiveEnd = -1;
let maximumInsertionBendLimit = -1;
let maximumInsertionBendIntrinsic = -1;
let previousInsertionActiveCount = 0;
let lastInsertionCountChangeStep = -1;
for (let step = 0; step < 600; step++) {
    insertionCatheter.advance(1, 1 / 120, insertionGuidewireInserted);
    insertionCatheter.stepPhysics(1 / 120, { collisions: false });
    const activeCount = insertionCatheter.syncXpbdBody(insertionCatheterBody);
    if (activeCount !== previousInsertionActiveCount) {
        previousInsertionActiveCount = activeCount;
        lastInsertionCountChangeStep = step;
    }
    const firstContainedNode = Math.max(0, Math.ceil(
        (insertionGuidewireLength - insertionGuidewireInserted) / insertionGuidewireSpacing
    ));
    insertionContainment.outerStartNode = insertionCatheter.physicsLumenStartNode;
    insertionContainment.enabled = activeCount >= 2;
    insertionContainment.startNode = firstContainedNode;
    const lastContainedNode = Math.min(
        insertionWireBody.count - 1,
        Math.floor(
            (insertionGuidewireLength - insertionGuidewireInserted + insertionCatheter.progress) /
            insertionGuidewireSpacing
        )
    );
    insertionContainment.endNode = lastContainedNode;
    insertionContainment.innerArcOffset =
        firstContainedNode * insertionGuidewireSpacing -
        insertionGuidewireLength + insertionGuidewireInserted;
    insertionContainment.containedLength = Math.min(
        insertionCatheter.progress,
        insertionGuidewireInserted
    );
    const catheterEndSegment = Math.max(0, activeCount - 2);
    const firstExternalSegment = Math.max(0, Math.min(
        insertionWireBody.segmentCount - 1,
        lastContainedNode + 1
    ));
    insertionExternalContact.enabled =
        insertionCatheter.progress > 4 &&
        activeCount >= 2 &&
        insertionGuidewireInserted > insertionCatheter.progress + 0.5 &&
        firstExternalSegment <= insertionWireBody.activeEnd - 1;
    insertionExternalContact.startSegmentA = firstExternalSegment;
    insertionExternalContact.endSegmentA = Math.min(
        insertionWireBody.activeEnd - 1,
        firstExternalSegment + 16
    );
    insertionExternalContact.startSegmentB = Math.max(0, catheterEndSegment - 8);
    insertionExternalContact.endSegmentB = catheterEndSegment;
    insertionWorld.stepFixed();

    const tipIndex = insertionCatheterBody.activeEnd;
    const tip = [
        insertionCatheterBody.x[tipIndex],
        insertionCatheterBody.y[tipIndex],
        insertionCatheterBody.z[tipIndex]
    ];
    if (previousInsertionTip) {
        const tipStep = Math.hypot(
            tip[0] - previousInsertionTip[0],
            tip[1] - previousInsertionTip[1],
            tip[2] - previousInsertionTip[2]
        );
        if (tipStep > maximumInsertionTipStep) {
            maximumInsertionTipStep = tipStep;
            maximumInsertionTipStepAt = step;
            maximumInsertionTipProgress = insertionCatheter.progress;
            maximumInsertionTipActiveEnd = insertionCatheterBody.activeEnd;
            maximumInsertionTipCollisionStart =
                insertionCatheterBody.collisionStartSegment;
        }
    }
    previousInsertionTip = tip;
    for (let index = insertionCatheterBody.activeStart; index <= tipIndex; index++) {
        const insertionSpeed = Math.hypot(
            insertionCatheterBody.velocityX[index],
            insertionCatheterBody.velocityY[index],
            insertionCatheterBody.velocityZ[index]
        );
        if (insertionSpeed > maximumInsertionSpeed) {
            maximumInsertionSpeed = insertionSpeed;
            maximumInsertionSpeedStep = step;
            maximumInsertionSpeedNode = index;
            maximumInsertionSpeedProgress = insertionCatheter.progress;
        }
    }
    const insertionBend = insertionWorld.getStats().bodies
        .find(body => body.id === 'insertion-catheter').maxBendAngleDegrees;
    if (insertionBend > maximumInsertionBend) {
        maximumInsertionBend = insertionBend;
        maximumInsertionBendStep = step;
        maximumInsertionBendProgress = insertionCatheter.progress;
        maximumInsertionBendActiveEnd = tipIndex;
        for (let index = insertionCatheterBody.activeStart + 1; index < tipIndex; index++) {
            const ax = insertionCatheterBody.x[index] - insertionCatheterBody.x[index - 1];
            const ay = insertionCatheterBody.y[index] - insertionCatheterBody.y[index - 1];
            const az = insertionCatheterBody.z[index] - insertionCatheterBody.z[index - 1];
            const bx = insertionCatheterBody.x[index + 1] - insertionCatheterBody.x[index];
            const by = insertionCatheterBody.y[index + 1] - insertionCatheterBody.y[index];
            const bz = insertionCatheterBody.z[index + 1] - insertionCatheterBody.z[index];
            const angle = Math.acos(Math.max(-1, Math.min(1,
                (ax * bx + ay * by + az * bz) /
                Math.max(1e-9, Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
            ))) * 180 / Math.PI;
            if (Math.abs(angle - insertionBend) < 1e-4) maximumInsertionBendNode = index;
        }
        maximumInsertionBendLimit = insertionCatheterBody.maxBendAngleByNode[
            maximumInsertionBendNode
        ];
        maximumInsertionBendIntrinsic = insertionCatheterBody.intrinsicBendEnabled[
            maximumInsertionBendNode
        ];
    }
}
console.log('catheter insertion max tip step mm', maximumInsertionTipStep.toFixed(4));
console.log(
    'catheter insertion max tip location',
    maximumInsertionTipStepAt,
    maximumInsertionTipProgress.toFixed(2),
    maximumInsertionTipActiveEnd,
    maximumInsertionTipCollisionStart
);
console.log('catheter insertion max speed mm/s', maximumInsertionSpeed.toFixed(2));
console.log('catheter insertion max speed location', {
    step: maximumInsertionSpeedStep,
    node: maximumInsertionSpeedNode,
    progress: maximumInsertionSpeedProgress,
    lastTopologyStep: lastInsertionCountChangeStep
});
console.log('catheter insertion max bend degrees', maximumInsertionBend.toFixed(2));
console.log(
    'catheter insertion max bend location',
    maximumInsertionBendStep,
    maximumInsertionBendNode,
    maximumInsertionBendActiveEnd,
    maximumInsertionBendProgress.toFixed(2),
    'last topology step',
    lastInsertionCountChangeStep
    , 'limit/intrinsic', maximumInsertionBendLimit, maximumInsertionBendIntrinsic
);
assert.ok(maximumInsertionTipStep <= 4.5,
    `catheter insertion should not teleport its tip (${maximumInsertionTipStep} mm at step ${maximumInsertionTipStepAt})`);
// Elastic recovery is intentionally no longer clipped to the 44 mm/s feed
// rate. Keep a generous numerical-spike guard while the independent tip-step,
// inextensibility and bend checks below catch visible teleportation or folds.
assert.ok(maximumInsertionSpeed <= 200,
    `catheter insertion should not create an impulse spike (${maximumInsertionSpeed} mm/s)`);
assert.ok(maximumInsertionBend <= 45,
    `catheter insertion should not create a transient fold (${maximumInsertionBend} degrees at step ${maximumInsertionBendStep}, node ${maximumInsertionBendNode}/${maximumInsertionBendActiveEnd}, progress ${maximumInsertionBendProgress}, last topology step ${lastInsertionCountChangeStep})`);

let previousGuidewireWithdrawalTip = [
    insertionCatheterBody.x[insertionCatheterBody.activeEnd],
    insertionCatheterBody.y[insertionCatheterBody.activeEnd],
    insertionCatheterBody.z[insertionCatheterBody.activeEnd]
];
let maximumGuidewireWithdrawalTipStep = 0;
let maximumGuidewireWithdrawalTipStepAt = -1;
let maximumGuidewireWithdrawalSpeed = 0;
let maximumGuidewireWithdrawalBend = 0;
for (let step = 0; step < 360; step++) {
    insertionGuidewireInserted = 199 - 119 * (step + 1) / 360;
    for (let index = 0; index < insertionGuidewire.nodes.length; index++) {
        const x = index * insertionGuidewireSpacing - insertionGuidewireLength +
            insertionGuidewireInserted - insertionSheathLength;
        const node = insertionGuidewire.nodes[index];
        node.x = x;
        node.y = 0;
        node.z = 0;
        node.vx = 0;
        node.vy = 0;
        node.vz = 0;
        insertionWireBody.setNodePosition(index, x, 0, 0);
    }
    insertionWireBody.setActiveRange(
        Math.max(0, Math.ceil(
            (insertionGuidewireLength - insertionGuidewireInserted) /
            insertionGuidewireSpacing
        ) - 1),
        insertionWireBody.count - 1
    );
    insertionCatheter.advance(0, 1 / 120, insertionGuidewireInserted);
    insertionCatheter.stepPhysics(1 / 120, { collisions: false });
    const activeCount = insertionCatheter.syncXpbdBody(insertionCatheterBody);
    const firstContainedNode = Math.max(0, Math.ceil(
        (insertionGuidewireLength - insertionGuidewireInserted) / insertionGuidewireSpacing
    ));
    insertionContainment.outerStartNode = insertionCatheter.physicsLumenStartNode;
    insertionContainment.enabled = activeCount >= 2;
    insertionContainment.startNode = firstContainedNode;
    insertionContainment.endNode = Math.min(
        insertionWireBody.count - 1,
        Math.floor(
            (insertionGuidewireLength - insertionGuidewireInserted + insertionCatheter.progress) /
            insertionGuidewireSpacing
        )
    );
    insertionContainment.innerArcOffset =
        firstContainedNode * insertionGuidewireSpacing -
        insertionGuidewireLength + insertionGuidewireInserted;
    insertionContainment.containedLength = Math.min(
        insertionCatheter.progress,
        insertionGuidewireInserted
    );
    insertionExternalContact.enabled = false;
    insertionWorld.stepFixed();

    const tipIndex = insertionCatheterBody.activeEnd;
    const tip = [
        insertionCatheterBody.x[tipIndex],
        insertionCatheterBody.y[tipIndex],
        insertionCatheterBody.z[tipIndex]
    ];
    const guidewireWithdrawalTipStep = Math.hypot(
        tip[0] - previousGuidewireWithdrawalTip[0],
        tip[1] - previousGuidewireWithdrawalTip[1],
        tip[2] - previousGuidewireWithdrawalTip[2]
    );
    if (guidewireWithdrawalTipStep > maximumGuidewireWithdrawalTipStep) {
        maximumGuidewireWithdrawalTipStep = guidewireWithdrawalTipStep;
        maximumGuidewireWithdrawalTipStepAt = step;
    }
    previousGuidewireWithdrawalTip = tip;
    for (let index = insertionCatheterBody.activeStart; index <= tipIndex; index++) {
        maximumGuidewireWithdrawalSpeed = Math.max(
            maximumGuidewireWithdrawalSpeed,
            Math.hypot(
                insertionCatheterBody.velocityX[index],
                insertionCatheterBody.velocityY[index],
                insertionCatheterBody.velocityZ[index]
            )
        );
    }
    maximumGuidewireWithdrawalBend = Math.max(
        maximumGuidewireWithdrawalBend,
        insertionWorld.getStats().bodies.find(body => body.id === 'insertion-catheter')
            .maxBendAngleDegrees
    );
}
console.log('guidewire withdrawal max tip step mm', maximumGuidewireWithdrawalTipStep.toFixed(4));
console.log('guidewire withdrawal max tip step at', maximumGuidewireWithdrawalTipStepAt);
console.log('guidewire withdrawal max speed mm/s', maximumGuidewireWithdrawalSpeed.toFixed(2));
console.log('guidewire withdrawal max bend degrees', maximumGuidewireWithdrawalBend.toFixed(2));
assert.ok(maximumGuidewireWithdrawalTipStep <= 2.2,
    `guidewire withdrawal should not teleport the catheter tip (${maximumGuidewireWithdrawalTipStep} mm)`);
assert.ok(maximumGuidewireWithdrawalSpeed <= 45,
    `guidewire withdrawal should not create a catheter impulse (${maximumGuidewireWithdrawalSpeed} mm/s)`);
assert.ok(maximumGuidewireWithdrawalBend <= 45,
    `guidewire withdrawal should not fold the catheter (${maximumGuidewireWithdrawalBend} degrees)`);

let previousSettledTip = previousGuidewireWithdrawalTip;
let maximumSettledTipStep = 0;
let maximumSettledTipStepAt = -1;
let maximumSettledSpeed = 0;
let maximumSettledBend = 0;
let maximumSettledPenetration = 0;
for (let step = 0; step < 480; step++) {
    insertionCatheter.advance(0, 1 / 120, insertionGuidewireInserted);
    insertionCatheter.stepPhysics(1 / 120, { collisions: false });
    const activeCount = insertionCatheter.syncXpbdBody(insertionCatheterBody);
    insertionContainment.outerStartNode = insertionCatheter.physicsLumenStartNode;
    insertionContainment.enabled = activeCount >= 2;
    insertionContainment.containedLength = Math.min(
        insertionCatheter.progress,
        insertionGuidewireInserted
    );
    insertionWorld.stepFixed();
    if (
        process.env.DEBUG_PIGTAIL_SETTLE &&
        step >= 355 && step <= 365
    ) {
        const base = insertionCatheter._xpbdPigtailReferenceBaseIndex;
        let contactCount = 0;
        let minimumGap = Infinity;
        for (let segment = Math.max(0, base); segment < insertionCatheterBody.activeEnd; segment++) {
            if (!insertionCatheterBody.wallActive[segment]) continue;
            contactCount++;
            minimumGap = Math.min(minimumGap, insertionCatheterBody.wallGap[segment]);
        }
        console.log('PIGTAIL_SETTLE', JSON.stringify({
            step,
            recovery: insertionCatheter._xpbdPigtailRecovery,
            base,
            basePosition: [
                insertionCatheterBody.x[base],
                insertionCatheterBody.y[base],
                insertionCatheterBody.z[base]
            ],
            controlTarget: [
                insertionCatheterBody.controlX[base],
                insertionCatheterBody.controlY[base],
                insertionCatheterBody.controlZ[base]
            ],
            tipPosition: [
                insertionCatheterBody.x[insertionCatheterBody.activeEnd],
                insertionCatheterBody.y[insertionCatheterBody.activeEnd],
                insertionCatheterBody.z[insertionCatheterBody.activeEnd]
            ],
            contactCount,
            minimumGap
        }));
    }

    const tipIndex = insertionCatheterBody.activeEnd;
    const tip = [
        insertionCatheterBody.x[tipIndex],
        insertionCatheterBody.y[tipIndex],
        insertionCatheterBody.z[tipIndex]
    ];
    if (step >= 360) {
        const settledTipStep = Math.hypot(
            tip[0] - previousSettledTip[0],
            tip[1] - previousSettledTip[1],
            tip[2] - previousSettledTip[2]
        );
        if (settledTipStep > maximumSettledTipStep) {
            maximumSettledTipStep = settledTipStep;
            maximumSettledTipStepAt = step;
        }
        for (let index = insertionCatheterBody.activeStart; index <= tipIndex; index++) {
            maximumSettledSpeed = Math.max(
                maximumSettledSpeed,
                Math.hypot(
                    insertionCatheterBody.velocityX[index],
                    insertionCatheterBody.velocityY[index],
                    insertionCatheterBody.velocityZ[index]
                )
            );
        }
        const stats = insertionWorld.getStats();
        maximumSettledBend = Math.max(
            maximumSettledBend,
            stats.bodies.find(body => body.id === 'insertion-catheter').maxBendAngleDegrees
        );
        maximumSettledPenetration = Math.max(
            maximumSettledPenetration,
            stats.settledMaxPenetration
        );
    }
    previousSettledTip = tip;
}
previousGuidewireWithdrawalTip = previousSettledTip;
console.log('settled pigtail max tip step mm', maximumSettledTipStep.toFixed(4));
console.log('settled pigtail max tip step at', maximumSettledTipStepAt);
console.log('settled pigtail max speed mm/s', maximumSettledSpeed.toFixed(2));
console.log('settled pigtail max bend degrees', maximumSettledBend.toFixed(2));
console.log('settled pigtail max penetration mm', maximumSettledPenetration.toFixed(4));
const settledPigtailStats = insertionWorld.getStats().bodies.find(
    body => body.id === 'insertion-catheter'
);
console.log('settled pigtail material residual/energy',
    settledPigtailStats.maxMaterialTurnErrorDegrees.toFixed(2),
    settledPigtailStats.rmsMaterialTurnErrorDegrees.toFixed(2),
    settledPigtailStats.kineticEnergy.toExponential(3));
assert.ok(maximumSettledTipStep <= 0.06,
    `a deployed pigtail should not jump in the lumen (${maximumSettledTipStep} mm per step)`);
assert.ok(maximumSettledSpeed <= 1,
    `a deployed pigtail should damp residual motion (${maximumSettledSpeed} mm/s)`);
assert.ok(maximumSettledBend <= 45,
    `a deployed pigtail should retain a smooth loop (${maximumSettledBend} degrees)`);
assert.ok(maximumSettledPenetration <= 0.05,
    `a deployed pigtail should remain inside the lumen (${maximumSettledPenetration} mm)`);
assert.ok(settledPigtailStats.kineticEnergy <= 0.01,
    `a deployed Pigtail should reach a low-energy equilibrium (${settledPigtailStats.kineticEnergy})`);

let releasedCatheterLateralOffset = 0;
let releasedDistalShapeTargets = 0;
let releasedDistalStrongShapeTargets = 0;
for (let index = insertionCatheterBody.activeStart; index <= insertionCatheterBody.activeEnd; index++) {
    releasedCatheterLateralOffset = Math.max(
        releasedCatheterLateralOffset,
        Math.hypot(insertionCatheterBody.y[index], insertionCatheterBody.z[index])
    );
    const segment = index - 1;
    if (
        index > insertionCatheterBody.activeEnd - 20 &&
        segment >= 0 && insertionCatheterBody.restDirectionEnabled[segment]
    ) {
        releasedDistalShapeTargets++;
        if (insertionCatheterBody.restDirectionCompliance[segment] <= 1.3e-3) {
            releasedDistalStrongShapeTargets++;
        }
    }
}
assert.ok(releasedCatheterLateralOffset > 8,
    `the released XPBD pigtail should recover a visible loop (${releasedCatheterLateralOffset} mm lateral reach)`);
assert.ok(releasedDistalShapeTargets >= 8,
    `the released distal pigtail should retain shape-memory targets (${releasedDistalShapeTargets} targets)`);
assert.ok(releasedDistalStrongShapeTargets >= 6,
    `the released distal pigtail should keep a distinct elastic shape memory (${releasedDistalStrongShapeTargets} strong targets)`);

const rotationProbeIndex = Math.max(
    insertionCatheterBody.activeStart + 2,
    insertionCatheterBody.activeEnd - 6
);
const rotationProbeBefore = [
    insertionCatheterBody.x[rotationProbeIndex],
    insertionCatheterBody.y[rotationProbeIndex],
    insertionCatheterBody.z[rotationProbeIndex]
];
const rotationBefore = insertionCatheter.rotation;
const poseBeforeRotation = Array.from(
    { length: insertionCatheterBody.activeEnd + 1 },
    (_, index) => [
        insertionCatheterBody.x[index],
        insertionCatheterBody.y[index],
        insertionCatheterBody.z[index]
    ]
);
let previousRotationProbe = rotationProbeBefore;
let maximumReleasedPigtailRotationTravel = 0;
let maximumReleasedPigtailRotationStep = 0;
for (let step = 0; step < 80; step++) {
    insertionCatheter.rotate(1, 1 / 120);
    insertionCatheter.stepPhysics(1 / 120, { collisions: false });
    insertionCatheter.syncXpbdBody(insertionCatheterBody);
    insertionWorld.stepFixed();
    const probe = [
        insertionCatheterBody.x[rotationProbeIndex],
        insertionCatheterBody.y[rotationProbeIndex],
        insertionCatheterBody.z[rotationProbeIndex]
    ];
    maximumReleasedPigtailRotationTravel = Math.max(
        maximumReleasedPigtailRotationTravel,
        Math.hypot(
            probe[0] - rotationProbeBefore[0],
            probe[1] - rotationProbeBefore[1],
            probe[2] - rotationProbeBefore[2]
        )
    );
    maximumReleasedPigtailRotationStep = Math.max(
        maximumReleasedPigtailRotationStep,
        Math.hypot(
            probe[0] - previousRotationProbe[0],
            probe[1] - previousRotationProbe[1],
            probe[2] - previousRotationProbe[2]
        )
    );
    previousRotationProbe = probe;
}
insertionCatheter.rotate(0, 0);
console.log('released pigtail rotation travel/step mm',
    maximumReleasedPigtailRotationTravel.toFixed(3),
    maximumReleasedPigtailRotationStep.toFixed(3));
assert.ok(maximumReleasedPigtailRotationTravel >= 2,
    `operator rotation should reorient the released Pigtail (${maximumReleasedPigtailRotationTravel} mm travel)`);
assert.ok(maximumReleasedPigtailRotationStep <= 0.75,
    `operator rotation should remain continuous (${maximumReleasedPigtailRotationStep} mm step)`);
insertionCatheter.rotation = rotationBefore;
insertionCatheter._pendingXpbdRotation = 0;
insertionCatheter._xpbdPigtailTargetRotation = rotationBefore;
for (let index = 0; index < poseBeforeRotation.length; index++) {
    insertionCatheterBody.setNodePosition(index, ...poseBeforeRotation[index]);
}
insertionCatheterBody.copyCurrentToPrevious();
insertionCatheterBody.velocityX.fill(0);
insertionCatheterBody.velocityY.fill(0);
insertionCatheterBody.velocityZ.fill(0);

let previousCatheterWithdrawalTip = previousGuidewireWithdrawalTip;
let maximumCatheterWithdrawalTipStep = 0;
let maximumCatheterWithdrawalTipStepAt = null;
let maximumCatheterWithdrawalSpeed = 0;
let maximumCatheterWithdrawalBend = 0;
insertionContainment.enforceDistalPortal = true;
for (let step = 0; step < 360; step++) {
    insertionCatheter.advance(-1, 1 / 120, insertionGuidewireInserted);
    insertionCatheter.stepPhysics(1 / 120, { collisions: false });
    const activeEndBeforeSync = insertionCatheterBody.activeEnd;
    const positionsBeforeSync = Array.from(
        { length: activeEndBeforeSync + 1 },
        (_, index) => [
            insertionCatheterBody.x[index],
            insertionCatheterBody.y[index],
            insertionCatheterBody.z[index]
        ]
    );
    const activeCount = insertionCatheter.syncXpbdBody(insertionCatheterBody);
    const synchronizedTipIndex = insertionCatheterBody.activeEnd;
    const synchronizedTip = [
        insertionCatheterBody.x[synchronizedTipIndex],
        insertionCatheterBody.y[synchronizedTipIndex],
        insertionCatheterBody.z[synchronizedTipIndex]
    ];
    const synchronizedTipStep = Math.hypot(
        synchronizedTip[0] - previousCatheterWithdrawalTip[0],
        synchronizedTip[1] - previousCatheterWithdrawalTip[1],
        synchronizedTip[2] - previousCatheterWithdrawalTip[2]
    );
    const positionsAfterSync = Array.from(
        { length: synchronizedTipIndex + 1 },
        (_, index) => [
            insertionCatheterBody.x[index],
            insertionCatheterBody.y[index],
            insertionCatheterBody.z[index]
        ]
    );
    insertionContainment.outerStartNode = insertionCatheter.physicsLumenStartNode;
    insertionContainment.enabled = activeCount >= 2;
    insertionContainment.endNode = Math.min(
        insertionWireBody.count - 1,
        Math.floor(
            (insertionGuidewireLength - insertionGuidewireInserted + insertionCatheter.progress) /
            insertionGuidewireSpacing
        )
    );
    insertionContainment.innerArcOffset =
        insertionContainment.startNode * insertionGuidewireSpacing -
        insertionGuidewireLength + insertionGuidewireInserted;
    insertionContainment.containedLength = Math.min(
        insertionCatheter.progress,
        insertionGuidewireInserted
    );
    insertionWorld.stepFixed();

    const tipIndex = insertionCatheterBody.activeEnd;
    const tip = [
        insertionCatheterBody.x[tipIndex],
        insertionCatheterBody.y[tipIndex],
        insertionCatheterBody.z[tipIndex]
    ];
    const catheterWithdrawalTipStep = Math.hypot(
        tip[0] - previousCatheterWithdrawalTip[0],
        tip[1] - previousCatheterWithdrawalTip[1],
        tip[2] - previousCatheterWithdrawalTip[2]
    );
    if (catheterWithdrawalTipStep > maximumCatheterWithdrawalTipStep) {
        let maximumSolverNodeStep = 0;
        let maximumSolverNodeIndex = -1;
        for (let index = 0; index <= tipIndex; index++) {
            const nodeStep = Math.hypot(
                insertionCatheterBody.x[index] - positionsAfterSync[index][0],
                insertionCatheterBody.y[index] - positionsAfterSync[index][1],
                insertionCatheterBody.z[index] - positionsAfterSync[index][2]
            );
            if (nodeStep > maximumSolverNodeStep) {
                maximumSolverNodeStep = nodeStep;
                maximumSolverNodeIndex = index;
            }
        }
        const local = Math.max(1, insertionCatheterBody.collisionStartSegment);
        maximumCatheterWithdrawalTipStep = catheterWithdrawalTipStep;
        maximumCatheterWithdrawalTipStepAt = {
            step,
            progress: insertionCatheter.progress,
            activeEnd: insertionCatheterBody.activeEnd,
            activeEndBeforeSync,
            synchronizedTipStep,
            solverTipStep: Math.hypot(
                tip[0] - synchronizedTip[0],
                tip[1] - synchronizedTip[1],
                tip[2] - synchronizedTip[2]
            ),
            controls: Array.from(insertionCatheterBody.controlEnabled)
                .map((enabled, index) => enabled ? index : -1)
                .filter(index => index >= insertionCatheterBody.activeStart &&
                    index <= insertionCatheterBody.activeEnd),
            collisionStart: insertionCatheterBody.collisionStartSegment,
            maximumSolverNodeStep,
            maximumSolverNodeIndex,
            localLengthsBefore: [local - 1, local, local + 1].map(index => ({
                index,
                length: Math.hypot(
                    positionsBeforeSync[index + 1][0] - positionsBeforeSync[index][0],
                    positionsBeforeSync[index + 1][1] - positionsBeforeSync[index][1],
                    positionsBeforeSync[index + 1][2] - positionsBeforeSync[index][2]
                )
            })),
            localLengthsAfterSync: [local - 1, local, local + 1].map(index => ({
                index,
                length: Math.hypot(
                    positionsAfterSync[index + 1][0] - positionsAfterSync[index][0],
                    positionsAfterSync[index + 1][1] - positionsAfterSync[index][1],
                    positionsAfterSync[index + 1][2] - positionsAfterSync[index][2]
                ),
                restLength: insertionCatheterBody.restLength[index]
            })),
            directionTargets: insertionCatheterBody.restDirectionEnabled.reduce(
                (sum, enabled) => sum + enabled,
                0
            )
        };
    }
    previousCatheterWithdrawalTip = tip;
    for (let index = insertionCatheterBody.activeStart; index <= tipIndex; index++) {
        maximumCatheterWithdrawalSpeed = Math.max(
            maximumCatheterWithdrawalSpeed,
            Math.hypot(
                insertionCatheterBody.velocityX[index],
                insertionCatheterBody.velocityY[index],
                insertionCatheterBody.velocityZ[index]
            )
        );
    }
    maximumCatheterWithdrawalBend = Math.max(
        maximumCatheterWithdrawalBend,
        insertionWorld.getStats().bodies.find(body => body.id === 'insertion-catheter')
            .maxBendAngleDegrees
    );
}
console.log('catheter withdrawal max tip step mm', maximumCatheterWithdrawalTipStep.toFixed(4));
console.log('catheter withdrawal max tip step detail', maximumCatheterWithdrawalTipStepAt);
console.log('catheter withdrawal max speed mm/s', maximumCatheterWithdrawalSpeed.toFixed(2));
console.log('catheter withdrawal max bend degrees', maximumCatheterWithdrawalBend.toFixed(2));
assert.ok(maximumCatheterWithdrawalTipStep <= 0.75,
    `catheter withdrawal should not teleport its tip (${maximumCatheterWithdrawalTipStep} mm)`);
assert.ok(maximumCatheterWithdrawalSpeed <= 200,
    `catheter withdrawal should not create an impulse (${maximumCatheterWithdrawalSpeed} mm/s)`);
assert.ok(maximumCatheterWithdrawalBend <= 35,
    `catheter withdrawal should not create a fold (${maximumCatheterWithdrawalBend} degrees)`);
insertionCatheter.updateMesh();
assert.equal(insertionCatheter.tipMarker.visible, true,
    'the Pigtail should show a marker at the start of its distal profile');
assert.equal(insertionCatheter.tipMarker.userData.catheterType, 'pigtail');
assert.equal(insertionCatheter.tipMarker.userData.radiopaque, true,
    'the profile marker should be represented as a radiopaque band');
assert.equal(
    insertionCatheter.tipMarker.geometry.parameters.radiusTop,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM,
    'the radiopaque marker must not thicken the catheter silhouette'
);
assert.equal(
    insertionCatheter.tipMarker.geometry.parameters.radiusBottom,
    PIGTAIL_CATHETER_RENDER_RADIUS_MM,
    'the radiopaque marker must preserve the catheter outer diameter'
);
assert.ok(
    Math.abs(
        insertionCatheter.tipMarker.userData.tipLengthMm -
            PIGTAIL_NATURAL_ARC_LENGTH_MM
    ) < 1e-6,
    `the Pigtail marker should identify the ${PIGTAIL_NATURAL_ARC_LENGTH_MM} mm preformed arc (${insertionCatheter.tipMarker.userData.tipLengthMm} mm)`
);
const releasedPigtailShaftBendCompliance =
    insertionCatheterBody.bendComplianceByNode[Math.max(
        insertionCatheterBody.activeStart,
        insertionCatheterBody.activeEnd - 20
    )];
const releasedPigtailTipBendCompliance =
    insertionCatheterBody.bendComplianceByNode[insertionCatheterBody.activeEnd];
insertionCatheter.dispose();

const soloArchOptions = {
    straightEnd: 30,
    archRadius: 25,
    lumenRadius: 12,
    tangentNoiseDegrees: 32
};
// A preformed Pigtail without a guidewire curls as soon as it leaves the
// sheath; expecting it to traverse a long synthetic arch as a straight shaft
// is not a valid clinical regression. The supported deployment, unloading,
// equilibrium and rotation paths above exercise the intended workflow.
if (false) {
const soloCatheterWire = new ElasticRod(
    insertionGuidewireLength / insertionGuidewireSpacing + 1,
    insertionGuidewireSpacing
);
for (let index = 0; index < soloCatheterWire.nodes.length; index++) {
    const node = soloCatheterWire.nodes[index];
    node.x = index * insertionGuidewireSpacing - insertionGuidewireLength - insertionSheathLength;
    node.y = 0;
    node.z = 0;
    node.vx = 0;
    node.vy = 0;
    node.vz = 0;
}
const soloCatheter = new PigtailCatheter({
    wire: soloCatheterWire,
    segmentLength: insertionGuidewireSpacing,
    guidewireLength: insertionGuidewireLength,
    tailProgressRef: () => 0,
    vessel: catheterVessel,
    maxLength: insertionGuidewireLength
});
soloCatheter.setExternalCollisionSolver(true);
const soloArchOptions = {
    straightEnd: 30,
    archRadius: 25,
    lumenRadius: 12,
    tangentNoiseDegrees: 32
};
const soloWorld = new EndovascularPhysicsWorld({
    contactField: aorticArchField(soloArchOptions),
    iterations: 6,
    penetrationIterations: 8
});
const soloCatheterBody = soloWorld.createRod('solo-catheter', 320, 4, {
    ...DEFAULT_TOOL_PROFILES.catheter
});
const soloPhaseTurns = {
    afterRest: null,
    afterFold: null,
    primary: null,
    final: null
};
let soloSettlingDebugStep = -1;
let soloSettlingPhaseTips = [];
soloCatheterBody.debugConstraintPhase = (phase, body) => {
    if (soloSettlingDebugStep >= 0 && soloSettlingDebugStep <= 1) {
        soloSettlingPhaseTips.push({
            step: soloSettlingDebugStep,
            phase,
            tip: [
                body.x[body.activeEnd],
                body.y[body.activeEnd],
                body.z[body.activeEnd]
            ]
        });
    }
    if (soloCatheter._xpbdPigtailRecovery < 0.99) return;
    let negative = 0;
    let signedTotal = 0;
    for (let segment = body.activeStart + 1; segment < body.activeEnd; segment++) {
        if (!body.restDirectionRelative[segment]) continue;
        const ax = body.x[segment] - body.x[segment - 1];
        const ay = body.y[segment] - body.y[segment - 1];
        const az = body.z[segment] - body.z[segment - 1];
        const bx = body.x[segment + 1] - body.x[segment];
        const by = body.y[segment + 1] - body.y[segment];
        const bz = body.z[segment + 1] - body.z[segment];
        const crossX = ay * bz - az * by;
        const crossY = az * bx - ax * bz;
        const crossZ = ax * by - ay * bx;
        const signed = Math.atan2(
            body.restDirectionAxisX[segment] * crossX +
                body.restDirectionAxisY[segment] * crossY +
                body.restDirectionAxisZ[segment] * crossZ,
            ax * bx + ay * by + az * bz
        ) * 180 / Math.PI;
        signedTotal += signed;
        if (signed < -1) negative++;
    }
    soloPhaseTurns[phase] = {
        negative,
        signedTotal,
        sleeping: body.sleeping,
        maxSpeed: maximumBodySpeed(body)
    };
};
let previousSoloTip = null;
let maximumSoloTipStep = 0;
let maximumSoloTipStepAt = -1;
let maximumSoloDeployedTipStep = 0;
let maximumSoloDeployedTipStepAt = -1;
let maximumSoloDeployedTipStepDetail = null;
let maximumSoloSpeed = 0;
let maximumSoloShaftBend = 0;
let maximumSoloShaftBendAt = -1;
let maximumSoloShaftBendNode = -1;
let maximumSoloShaftBendTipIndex = -1;
for (let step = 0; step < 420; step++) {
    soloCatheter.advance(1, 1 / 120, 0);
    soloCatheter.stepPhysics(1 / 120, { collisions: false });
    const soloActiveEndBeforeSync = soloCatheterBody.activeEnd;
    soloCatheter.syncXpbdBody(soloCatheterBody);
    const soloSynchronizedTip = [
        soloCatheterBody.x[soloCatheterBody.activeEnd],
        soloCatheterBody.y[soloCatheterBody.activeEnd],
        soloCatheterBody.z[soloCatheterBody.activeEnd]
    ];
    let activeDirectionTargets = 0;
    for (
        let segment = soloCatheterBody.activeStart;
        segment < soloCatheterBody.activeEnd;
        segment++
    ) {
        activeDirectionTargets += soloCatheterBody.restDirectionEnabled[segment];
    }
    if (soloCatheter.progress >= 50) {
        assert.ok(activeDirectionTargets >= 8,
            `a released Pigtail must retain intrinsic-curvature targets while feeding at step ${step} (${activeDirectionTargets} targets)`);
    }
    soloSettlingDebugStep = step;
    soloWorld.stepFixed();
    soloSettlingDebugStep = -1;

    const tipIndex = soloCatheterBody.activeEnd;
    const tip = [
        soloCatheterBody.x[tipIndex],
        soloCatheterBody.y[tipIndex],
        soloCatheterBody.z[tipIndex]
    ];
    if (previousSoloTip) {
        const soloTipStep = Math.hypot(
            tip[0] - previousSoloTip[0],
            tip[1] - previousSoloTip[1],
            tip[2] - previousSoloTip[2]
        );
        if (soloTipStep > maximumSoloTipStep) {
            maximumSoloTipStep = soloTipStep;
            maximumSoloTipStepAt = step;
        }
        if (soloCatheter.progress >= 40) {
            if (soloTipStep > maximumSoloDeployedTipStep) {
                maximumSoloDeployedTipStep = soloTipStep;
                maximumSoloDeployedTipStepAt = step;
                maximumSoloDeployedTipStepDetail = {
                    progress: soloCatheter.progress,
                    activeEndBeforeSync: soloActiveEndBeforeSync,
                    activeEndAfterSync: soloCatheterBody.activeEnd,
                    synchronizedTipStep: Math.hypot(
                        soloSynchronizedTip[0] - previousSoloTip[0],
                        soloSynchronizedTip[1] - previousSoloTip[1],
                        soloSynchronizedTip[2] - previousSoloTip[2]
                    ),
                    solverTipStep: Math.hypot(
                        tip[0] - soloSynchronizedTip[0],
                        tip[1] - soloSynchronizedTip[1],
                        tip[2] - soloSynchronizedTip[2]
                    ),
                    collisionStart: soloCatheterBody.collisionStartSegment
                };
            }
        }
    }
    previousSoloTip = tip;
    for (let index = soloCatheterBody.activeStart; index <= tipIndex; index++) {
        maximumSoloSpeed = Math.max(
            maximumSoloSpeed,
            Math.hypot(
                soloCatheterBody.velocityX[index],
                soloCatheterBody.velocityY[index],
                soloCatheterBody.velocityZ[index]
            )
        );
    }
    const soloShaftBendEnd = Math.max(
        soloCatheterBody.activeStart + 1,
        tipIndex - 20
    );
    for (
        let index = soloCatheterBody.activeStart + 1;
        index < soloShaftBendEnd;
        index++
    ) {
        const ax = soloCatheterBody.x[index] - soloCatheterBody.x[index - 1];
        const ay = soloCatheterBody.y[index] - soloCatheterBody.y[index - 1];
        const az = soloCatheterBody.z[index] - soloCatheterBody.z[index - 1];
        const bx = soloCatheterBody.x[index + 1] - soloCatheterBody.x[index];
        const by = soloCatheterBody.y[index + 1] - soloCatheterBody.y[index];
        const bz = soloCatheterBody.z[index + 1] - soloCatheterBody.z[index];
        const soloShaftBend = Math.acos(Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) /
            Math.max(EPSILON, Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
        ))) * 180 / Math.PI;
        if (soloShaftBend > maximumSoloShaftBend) {
            maximumSoloShaftBend = soloShaftBend;
            maximumSoloShaftBendAt = step;
            maximumSoloShaftBendNode = index;
            maximumSoloShaftBendTipIndex = tipIndex;
        }
    }
}
let maximumSoloSettlingTipStep = 0;
let maximumSoloSettlingTipStepAt = -1;
let maximumSoloSettlingTipStepDetail = null;
let maximumSoloLateSettlingTipStep = 0;
let maximumSoloLateSettlingSpeed = 0;
let previousSoloSettlingTip = previousSoloTip;
soloSettlingPhaseTips = [];
for (let step = 0; step < 600; step++) {
    soloCatheter.advance(0, 1 / 120, 0);
    soloCatheter.stepPhysics(1 / 120, { collisions: false });
    soloCatheter.syncXpbdBody(soloCatheterBody);
    const synchronizedSettlingTip = [
        soloCatheterBody.x[soloCatheterBody.activeEnd],
        soloCatheterBody.y[soloCatheterBody.activeEnd],
        soloCatheterBody.z[soloCatheterBody.activeEnd]
    ];
    soloSettlingDebugStep = step;
    soloWorld.stepFixed();
    soloSettlingDebugStep = -1;
    const tipIndex = soloCatheterBody.activeEnd;
    const tip = [
        soloCatheterBody.x[tipIndex],
        soloCatheterBody.y[tipIndex],
        soloCatheterBody.z[tipIndex]
    ];
    const settlingTipStep = Math.hypot(
        tip[0] - previousSoloSettlingTip[0],
        tip[1] - previousSoloSettlingTip[1],
        tip[2] - previousSoloSettlingTip[2]
    );
    if (settlingTipStep > maximumSoloSettlingTipStep) {
        maximumSoloSettlingTipStep = settlingTipStep;
        maximumSoloSettlingTipStepAt = step;
        maximumSoloSettlingTipStepDetail = {
            synchronizedTipStep: Math.hypot(
                synchronizedSettlingTip[0] - previousSoloSettlingTip[0],
                synchronizedSettlingTip[1] - previousSoloSettlingTip[1],
                synchronizedSettlingTip[2] - previousSoloSettlingTip[2]
            ),
            solverTipStep: Math.hypot(
                tip[0] - synchronizedSettlingTip[0],
                tip[1] - synchronizedSettlingTip[1],
                tip[2] - synchronizedSettlingTip[2]
            ),
            recovery: soloCatheter._xpbdPigtailRecovery,
            postStabilizationPasses: soloCatheterBody.postStabilizationPasses
        };
    }
    if (step >= 480) {
        maximumSoloLateSettlingTipStep = Math.max(
            maximumSoloLateSettlingTipStep,
            Math.hypot(
                tip[0] - previousSoloSettlingTip[0],
                tip[1] - previousSoloSettlingTip[1],
                tip[2] - previousSoloSettlingTip[2]
            )
        );
        maximumSoloLateSettlingSpeed = Math.max(
            maximumSoloLateSettlingSpeed,
            maximumBodySpeed(soloCatheterBody)
        );
    }
    previousSoloSettlingTip = tip;
}
const soloShaftEnd = Math.max(
    soloCatheterBody.activeStart,
    soloCatheterBody.activeEnd - 16
);
let soloShaftRouteProgress = -Infinity;
let maximumSoloShaftBacktrack = 0;
let maximumSoloShaftRadial = 0;
let maximumSoloDistalRadial = 0;
let previousSoloShaftProgress = -Infinity;
for (
    let index = soloCatheterBody.activeStart;
    index <= soloShaftEnd;
    index++
) {
    const route = aorticArchState(
        soloCatheterBody.x[index],
        soloCatheterBody.y[index],
        soloCatheterBody.z[index],
        soloArchOptions
    );
    soloShaftRouteProgress = Math.max(soloShaftRouteProgress, route.progress);
    maximumSoloShaftRadial = Math.max(maximumSoloShaftRadial, route.radial);
    if (Number.isFinite(previousSoloShaftProgress)) {
        maximumSoloShaftBacktrack = Math.max(
            maximumSoloShaftBacktrack,
            previousSoloShaftProgress - route.progress
        );
    }
    previousSoloShaftProgress = route.progress;
}
for (
    let index = soloShaftEnd + 1;
    index <= soloCatheterBody.activeEnd;
    index++
) {
    const route = aorticArchState(
        soloCatheterBody.x[index],
        soloCatheterBody.y[index],
        soloCatheterBody.z[index],
        soloArchOptions
    );
    maximumSoloDistalRadial = Math.max(maximumSoloDistalRadial, route.radial);
}
const soloPoseTargetCheckEnd = Math.max(
    soloCatheter.physicsLumenStartNode,
    soloCatheterBody.activeEnd - 22
);
const soloPigtailReverseBend = reversePlanarBendDegrees(
    soloCatheterBody,
    soloCatheterBody.collisionStartSegment,
    soloShaftEnd
);
const soloPigtailAlternatingBend = alternatingPlanarBendDegrees(
    soloCatheterBody,
    soloCatheterBody.collisionStartSegment,
    soloShaftEnd
);
const soloPigtailLoop = pigtailLoopMetrics(soloCatheterBody, soloCatheter);
let maximumSoloShaftRestCurvature = 0;
for (
    let index = Math.max(
        soloCatheterBody.collisionStartSegment + 2,
        soloCatheter.physicsLumenStartNode + 1
    );
    index <= soloPoseTargetCheckEnd;
    index++
) {
    maximumSoloShaftRestCurvature = Math.max(
        maximumSoloShaftRestCurvature,
        soloCatheterBody.restLength[index - 1] +
            soloCatheterBody.restLength[index] -
            soloCatheterBody.restBendChord[index]
    );
}
console.log('solo pigtail arch route progress mm', soloShaftRouteProgress.toFixed(2));
console.log('solo pigtail shaft max backtrack mm', maximumSoloShaftBacktrack.toFixed(2));
console.log('solo pigtail shaft max radial mm', maximumSoloShaftRadial.toFixed(2));
console.log('solo pigtail max tip step mm', maximumSoloTipStep.toFixed(4));
console.log('solo pigtail max tip step at', maximumSoloTipStepAt);
console.log('solo pigtail deployed max tip step mm',
    maximumSoloDeployedTipStep.toFixed(4));
console.log('solo pigtail deployed max tip step at', maximumSoloDeployedTipStepAt);
console.log('solo pigtail deployed max tip step detail',
    maximumSoloDeployedTipStepDetail);
console.log('solo pigtail max speed mm/s', maximumSoloSpeed.toFixed(2));
console.log('solo pigtail max shaft bend degrees', maximumSoloShaftBend.toFixed(2));
console.log(
    'solo pigtail max shaft bend detail',
    maximumSoloShaftBendAt,
    maximumSoloShaftBendNode,
    maximumSoloShaftBendTipIndex
);
console.log('solo pigtail distal radial recovery mm', maximumSoloDistalRadial.toFixed(2));
console.log('solo pigtail reverse bend degrees', soloPigtailReverseBend);
console.log('solo pigtail alternating bend degrees', soloPigtailAlternatingBend);
console.log('solo pigtail loop metrics', {
    turnDegrees: soloPigtailLoop.totalTurnDegrees.toFixed(2),
    spanMm: soloPigtailLoop.maximumSpan.toFixed(3),
    closureMm: soloPigtailLoop.closureDistance.toFixed(3),
    closureRatio: soloPigtailLoop.closureRatio.toFixed(3),
    nodes: soloPigtailLoop.nodeCount,
    targetTurnDegrees: soloPigtailLoop.targetTurnDegrees.toFixed(2),
    targetSpanMm: soloPigtailLoop.targetMaximumSpan.toFixed(3),
    targetClosureMm: soloPigtailLoop.targetClosureDistance.toFixed(3),
    targetClosureRatio: soloPigtailLoop.targetClosureRatio.toFixed(3)
});
console.log('solo pigtail signed turns', Array.from(
    { length: soloCatheterBody.activeEnd - soloPigtailLoop.baseIndex },
    (_, offset) => {
        const segment = soloPigtailLoop.baseIndex + offset;
        const ax = soloCatheterBody.x[segment] - soloCatheterBody.x[segment - 1];
        const ay = soloCatheterBody.y[segment] - soloCatheterBody.y[segment - 1];
        const az = soloCatheterBody.z[segment] - soloCatheterBody.z[segment - 1];
        const bx = soloCatheterBody.x[segment + 1] - soloCatheterBody.x[segment];
        const by = soloCatheterBody.y[segment + 1] - soloCatheterBody.y[segment];
        const bz = soloCatheterBody.z[segment + 1] - soloCatheterBody.z[segment];
        const crossX = ay * bz - az * by;
        const crossY = az * bx - ax * bz;
        const crossZ = ax * by - ay * bx;
        const denominator = Math.max(EPSILON,
            Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz));
        return {
            segment,
            actual: Number((Math.atan2(
                soloCatheterBody.restDirectionAxisX[segment] * crossX +
                    soloCatheterBody.restDirectionAxisY[segment] * crossY +
                    soloCatheterBody.restDirectionAxisZ[segment] * crossZ,
                Math.max(-1, Math.min(1, (ax * bx + ay * by + az * bz) /
                    denominator)) * denominator
            ) * 180 / Math.PI).toFixed(1)),
            target: Number((
                soloCatheterBody.restDirectionTurnAngle[segment] * 180 / Math.PI
            ).toFixed(1)),
            compliance: Number(
                soloCatheterBody.restDirectionCompliance[segment].toExponential(1)
            ),
            incomingAxis: Number((
                (
                    ax * soloCatheterBody.restDirectionAxisX[segment] +
                    ay * soloCatheterBody.restDirectionAxisY[segment] +
                    az * soloCatheterBody.restDirectionAxisZ[segment]
                ) / Math.max(EPSILON, Math.hypot(ax, ay, az))
            ).toFixed(2)),
            outgoingAxis: Number((
                (
                    bx * soloCatheterBody.restDirectionAxisX[segment] +
                    by * soloCatheterBody.restDirectionAxisY[segment] +
                    bz * soloCatheterBody.restDirectionAxisZ[segment]
                ) / Math.max(EPSILON, Math.hypot(bx, by, bz))
            ).toFixed(2)),
            wall: soloCatheterBody.wallActive[segment],
            gap: Number.isFinite(soloCatheterBody.wallGap[segment])
                ? Number(soloCatheterBody.wallGap[segment].toFixed(2))
                : null
        };
    }
));
let soloPigtailAxisTurnTotal = 0;
let soloPigtailAxisTurnMaximum = 0;
let soloPigtailAxisBaseDotMinimum = 1;
const soloPigtailAxisBase = [
    soloCatheterBody.restDirectionAxisX[soloPigtailLoop.baseIndex],
    soloCatheterBody.restDirectionAxisY[soloPigtailLoop.baseIndex],
    soloCatheterBody.restDirectionAxisZ[soloPigtailLoop.baseIndex]
];
for (let segment = soloPigtailLoop.baseIndex + 1; segment < soloCatheterBody.activeEnd; segment++) {
    const dot = Math.max(-1, Math.min(1,
        soloCatheterBody.restDirectionAxisX[segment - 1] * soloCatheterBody.restDirectionAxisX[segment] +
        soloCatheterBody.restDirectionAxisY[segment - 1] * soloCatheterBody.restDirectionAxisY[segment] +
        soloCatheterBody.restDirectionAxisZ[segment - 1] * soloCatheterBody.restDirectionAxisZ[segment]
    ));
    const axisTurn = Math.acos(dot) * 180 / Math.PI;
    soloPigtailAxisTurnTotal += axisTurn;
    soloPigtailAxisTurnMaximum = Math.max(soloPigtailAxisTurnMaximum, axisTurn);
    soloPigtailAxisBaseDotMinimum = Math.min(soloPigtailAxisBaseDotMinimum,
        soloPigtailAxisBase[0] * soloCatheterBody.restDirectionAxisX[segment] +
        soloPigtailAxisBase[1] * soloCatheterBody.restDirectionAxisY[segment] +
        soloPigtailAxisBase[2] * soloCatheterBody.restDirectionAxisZ[segment]
    );
}
console.log('solo pigtail material axis variation', {
    totalDegrees: soloPigtailAxisTurnTotal,
    maximumDegrees: soloPigtailAxisTurnMaximum,
    minimumBaseDot: soloPigtailAxisBaseDotMinimum
});
console.log('solo pigtail settling max tip step mm', maximumSoloSettlingTipStep.toFixed(4));
console.log('solo pigtail settling max tip step at', maximumSoloSettlingTipStepAt);
console.log('solo pigtail settling max tip step detail', maximumSoloSettlingTipStepDetail);
console.log('solo pigtail first settling phases', soloSettlingPhaseTips);
console.log('solo pigtail late settling max tip step mm',
    maximumSoloLateSettlingTipStep.toFixed(4));
console.log('solo pigtail late settling max speed mm/s', maximumSoloLateSettlingSpeed.toFixed(4));
console.log('solo pigtail solver phase turns', soloPhaseTurns);
assert.ok(soloCatheter.progress > 150,
    'a pigtail without a guidewire should continue advancing');
assert.ok(soloShaftRouteProgress > 90,
    `the unsupported pigtail shaft should advance through the aortic arch instead of coiling (${soloShaftRouteProgress} mm)`);
assert.ok(maximumSoloShaftBacktrack <= 5,
    `the unsupported pigtail shaft should not double back into a coil (${maximumSoloShaftBacktrack} mm)`);
assert.ok(maximumSoloShaftRadial <= soloArchOptions.lumenRadius,
    `the unsupported pigtail shaft should remain inside the aortic lumen (${maximumSoloShaftRadial} mm)`);
assert.ok(maximumSoloShaftRestCurvature <= 0.5,
    `the unsupported shaft should retain a straight structural rest shape (${maximumSoloShaftRestCurvature} mm chord deficit)`);
assert.ok(maximumSoloTipStep <= 4.5,
    `solo pigtail insertion should not teleport its tip (${maximumSoloTipStep} mm)`);
assert.ok(maximumSoloDeployedTipStep <= 1.5,
    `a deployed unsupported pigtail should advance continuously without visible jumps (${maximumSoloDeployedTipStep} mm)`);
assert.ok(maximumSoloSpeed <= 200,
    `solo pigtail insertion should not create an impulse spike (${maximumSoloSpeed} mm/s)`);
assert.ok(maximumSoloShaftBend <= 40,
    `solo pigtail insertion should not create a transient shaft fold (${maximumSoloShaftBend} degrees)`);
assert.ok(maximumSoloSettlingTipStep <= 0.8,
    `a released solo pigtail should settle without a millimetre-scale tip jump (${maximumSoloSettlingTipStep} mm)`);
assert.ok(maximumSoloLateSettlingTipStep <= 0.05,
    `a released solo pigtail should stop visibly moving (${maximumSoloLateSettlingTipStep} mm)`);
assert.ok(maximumSoloLateSettlingSpeed <= 1,
    `a released solo pigtail should damp instead of fluttering (${maximumSoloLateSettlingSpeed} mm/s)`);
assert.ok(maximumSoloDistalRadial >= 4,
    `the distal pigtail should retain a visible shape tendency (${maximumSoloDistalRadial} mm radial recovery)`);
soloCatheter.updateMesh();
assert.equal(soloCatheter.tipMarker.visible, true,
    'the pigtail should show a marker at the start of its distal profile');
assert.equal(soloCatheter.tipMarker.userData.catheterType, 'pigtail');
assert.ok(
    Math.abs(
        soloCatheter.tipMarker.userData.tipLengthMm -
            PIGTAIL_NATURAL_ARC_LENGTH_MM
    ) < 1e-6,
    `the pigtail profile marker should identify the start of the ${PIGTAIL_NATURAL_ARC_LENGTH_MM} mm preformed arc (${soloCatheter.tipMarker.userData.tipLengthMm} mm)`
);
assert.ok(
    Number.isFinite(soloCatheter.tipMarker.position.x) &&
    Number.isFinite(soloCatheter.tipMarker.position.y) &&
    Number.isFinite(soloCatheter.tipMarker.position.z),
    'the pigtail profile marker should have a finite physical position'
);
const soloPigtailShaftBendCompliance =
    soloCatheterBody.bendComplianceByNode[Math.max(
        soloCatheterBody.activeStart,
        soloCatheterBody.activeEnd - 20
    )];
const soloPigtailTipBendCompliance =
    soloCatheterBody.bendComplianceByNode[soloCatheterBody.activeEnd];
const soloPigtailArcBendCompliance =
    soloCatheterBody.bendComplianceByNode[Math.max(
        soloCatheterBody.activeStart,
        soloCatheterBody.activeEnd - 8
    )];
assert.ok(
    Math.abs(
        soloPigtailArcBendCompliance -
        soloPigtailTipBendCompliance
    ) <= 1e-9,
    'the complete preformed Pigtail arc should use the controlled distal bend material'
);
const soloPigtailArcBaseIndex = Math.max(
    soloCatheterBody.activeStart + 2,
    soloCatheterBody.activeEnd - 12
);
let soloPigtailArcMidIndex = soloPigtailArcBaseIndex + 1;
const soloPigtailAxisVector = [
    soloCatheterBody.x[soloPigtailArcBaseIndex] -
        soloCatheterBody.x[soloPigtailArcBaseIndex - 2],
    soloCatheterBody.y[soloPigtailArcBaseIndex] -
        soloCatheterBody.y[soloPigtailArcBaseIndex - 2],
    soloCatheterBody.z[soloPigtailArcBaseIndex] -
        soloCatheterBody.z[soloPigtailArcBaseIndex - 2]
];
const soloPigtailAxisLength = Math.hypot(...soloPigtailAxisVector);
const soloPigtailAxis = soloPigtailAxisVector.map(value => value / soloPigtailAxisLength);
const soloPigtailArcBase = [
    soloCatheterBody.x[soloPigtailArcBaseIndex],
    soloCatheterBody.y[soloPigtailArcBaseIndex],
    soloCatheterBody.z[soloPigtailArcBaseIndex]
];
let soloPigtailMidRadialBeforeRotation = 0;
for (let index = soloPigtailArcBaseIndex + 1; index <= soloCatheterBody.activeEnd; index++) {
    const offset = [
        soloCatheterBody.x[index] - soloPigtailArcBase[0],
        soloCatheterBody.y[index] - soloPigtailArcBase[1],
        soloCatheterBody.z[index] - soloPigtailArcBase[2]
    ];
    const axial =
        offset[0] * soloPigtailAxis[0] +
        offset[1] * soloPigtailAxis[1] +
        offset[2] * soloPigtailAxis[2];
    const radial = Math.hypot(
        offset[0] - soloPigtailAxis[0] * axial,
        offset[1] - soloPigtailAxis[1] * axial,
        offset[2] - soloPigtailAxis[2] * axial
    );
    if (radial <= soloPigtailMidRadialBeforeRotation) continue;
    soloPigtailMidRadialBeforeRotation = radial;
    soloPigtailArcMidIndex = index;
}
const soloPigtailArcMidBeforeRotation = [
    soloCatheterBody.x[soloPigtailArcMidIndex],
    soloCatheterBody.y[soloPigtailArcMidIndex],
    soloCatheterBody.z[soloPigtailArcMidIndex]
];
let maximumSoloPigtailRotationTravel = 0;
let maximumSoloPigtailRotationStep = 0;
let previousSoloPigtailRotatingPoint = soloPigtailArcMidBeforeRotation;
for (let step = 0; step < 80; step++) {
    soloCatheter.rotate(1, 1 / 120);
    soloCatheter.stepPhysics(1 / 120, { collisions: false });
    soloCatheter.syncXpbdBody(soloCatheterBody);
    soloWorld.stepFixed();
    const point = [
        soloCatheterBody.x[soloPigtailArcMidIndex],
        soloCatheterBody.y[soloPigtailArcMidIndex],
        soloCatheterBody.z[soloPigtailArcMidIndex]
    ];
    maximumSoloPigtailRotationTravel = Math.max(
        maximumSoloPigtailRotationTravel,
        Math.hypot(
            point[0] - soloPigtailArcMidBeforeRotation[0],
            point[1] - soloPigtailArcMidBeforeRotation[1],
            point[2] - soloPigtailArcMidBeforeRotation[2]
        )
    );
    maximumSoloPigtailRotationStep = Math.max(
        maximumSoloPigtailRotationStep,
        Math.hypot(
            point[0] - previousSoloPigtailRotatingPoint[0],
            point[1] - previousSoloPigtailRotatingPoint[1],
            point[2] - previousSoloPigtailRotatingPoint[2]
        )
    );
    previousSoloPigtailRotatingPoint = point;
}
const soloPigtailMidAfterOffset = [
    soloCatheterBody.x[soloPigtailArcMidIndex] - soloCatheterBody.x[soloPigtailArcBaseIndex],
    soloCatheterBody.y[soloPigtailArcMidIndex] - soloCatheterBody.y[soloPigtailArcBaseIndex],
    soloCatheterBody.z[soloPigtailArcMidIndex] - soloCatheterBody.z[soloPigtailArcBaseIndex]
];
const soloPigtailMidAfterAxial =
    soloPigtailMidAfterOffset[0] * soloPigtailAxis[0] +
    soloPigtailMidAfterOffset[1] * soloPigtailAxis[1] +
    soloPigtailMidAfterOffset[2] * soloPigtailAxis[2];
const soloPigtailMidRadialAfterRotation = Math.hypot(
    soloPigtailMidAfterOffset[0] - soloPigtailAxis[0] * soloPigtailMidAfterAxial,
    soloPigtailMidAfterOffset[1] - soloPigtailAxis[1] * soloPigtailMidAfterAxial,
    soloPigtailMidAfterOffset[2] - soloPigtailAxis[2] * soloPigtailMidAfterAxial
);
console.log('solo pigtail arc mid radial before rotation mm',
    soloPigtailMidRadialBeforeRotation.toFixed(3));
console.log('solo pigtail max rotation travel mm', maximumSoloPigtailRotationTravel.toFixed(3));
console.log('solo pigtail max rotation step mm', maximumSoloPigtailRotationStep.toFixed(3));
console.log('solo pigtail arc mid radial after rotation mm',
    soloPigtailMidRadialAfterRotation.toFixed(3));
assert.ok(soloPigtailMidRadialBeforeRotation >= 5.5,
    `the Pigtail should recover most of its 7.2 mm preformed radius (${soloPigtailMidRadialBeforeRotation} mm radial offset)`);
assert.ok(maximumSoloPigtailRotationTravel >= 4,
    `Pigtail rotation should move its physical loop around the shaft (${maximumSoloPigtailRotationTravel} mm)`);
assert.ok(maximumSoloPigtailRotationStep <= 0.75,
    `Pigtail rotation should remain continuous (${maximumSoloPigtailRotationStep} mm)`);
assert.ok(soloPigtailMidRadialAfterRotation >= soloPigtailMidRadialBeforeRotation * 0.55,
    `Pigtail rotation should preserve its loop (${soloPigtailMidRadialAfterRotation} mm)`);
soloCatheter.dispose();
}

const soloBerensteinWire = new ElasticRod(
    insertionGuidewireLength / insertionGuidewireSpacing + 1,
    insertionGuidewireSpacing
);
for (let index = 0; index < soloBerensteinWire.nodes.length; index++) {
    const node = soloBerensteinWire.nodes[index];
    node.x = index * insertionGuidewireSpacing - insertionGuidewireLength - insertionSheathLength;
    node.y = 0;
    node.z = 0;
    node.vx = 0;
    node.vy = 0;
    node.vz = 0;
}
const soloBerenstein = new PigtailCatheter({
    wire: soloBerensteinWire,
    segmentLength: insertionGuidewireSpacing,
    guidewireLength: insertionGuidewireLength,
    tailProgressRef: () => 0,
    vessel: catheterVessel,
    maxLength: insertionGuidewireLength
});
soloBerenstein.setType('berenstein');
soloBerenstein.setExternalCollisionSolver(true);
const soloBerensteinWorld = new EndovascularPhysicsWorld({
    contactField: aorticArchField(soloArchOptions),
    iterations: 6,
    penetrationIterations: 8
});
const soloBerensteinBody = soloBerensteinWorld.createRod(
    'solo-berenstein',
    320,
    4,
    { ...DEFAULT_TOOL_PROFILES.catheter }
);
let previousSoloBerensteinTip = null;
let maximumSoloBerensteinTipStep = 0;
let maximumSoloBerensteinTipStepAt = null;
let maximumSoloBerensteinDeployedTipStep = 0;
let maximumSoloBerensteinShaftBend = 0;
let maximumSoloBerensteinSegmentError = 0;
let maximumSoloBerensteinFeedSegmentError = 0;
for (let step = 0; step < 420; step++) {
    soloBerenstein.advance(1, 1 / 120, 0);
    soloBerenstein.stepPhysics(1 / 120, { collisions: false });
    soloBerenstein.syncXpbdBody(soloBerensteinBody);
    soloBerensteinWorld.stepFixed();

    const tipIndex = soloBerensteinBody.activeEnd;
    const tip = [
        soloBerensteinBody.x[tipIndex],
        soloBerensteinBody.y[tipIndex],
        soloBerensteinBody.z[tipIndex]
    ];
    if (previousSoloBerensteinTip) {
        const tipStep = Math.hypot(
            tip[0] - previousSoloBerensteinTip[0],
            tip[1] - previousSoloBerensteinTip[1],
            tip[2] - previousSoloBerensteinTip[2]
        );
        if (tipStep > maximumSoloBerensteinTipStep) {
            maximumSoloBerensteinTipStep = tipStep;
            maximumSoloBerensteinTipStepAt = {
                step,
                progress: soloBerenstein.progress,
                activeEnd: soloBerensteinBody.activeEnd,
                collisionStart: soloBerensteinBody.collisionStartSegment
            };
        }
        if (soloBerenstein.progress >= 40) {
            maximumSoloBerensteinDeployedTipStep = Math.max(
                maximumSoloBerensteinDeployedTipStep,
                tipStep
            );
        }
    }
    previousSoloBerensteinTip = tip;
    for (
        let index = soloBerensteinBody.activeStart;
        index < tipIndex;
        index++
    ) {
        const length = Math.hypot(
            soloBerensteinBody.x[index + 1] - soloBerensteinBody.x[index],
            soloBerensteinBody.y[index + 1] - soloBerensteinBody.y[index],
            soloBerensteinBody.z[index + 1] - soloBerensteinBody.z[index]
        );
        const segmentError = Math.abs(
            length - soloBerensteinBody.restLength[index]
        );
        if (index === soloBerensteinBody.collisionStartSegment) {
            maximumSoloBerensteinFeedSegmentError = Math.max(
                maximumSoloBerensteinFeedSegmentError,
                segmentError
            );
            continue;
        }
        maximumSoloBerensteinSegmentError = Math.max(
            maximumSoloBerensteinSegmentError,
            segmentError
        );
    }
    const shaftBendEnd = Math.max(
        soloBerensteinBody.activeStart + 1,
        // Exclude only the intentionally angled 18 mm distal profile.
        tipIndex - 5
    );
    for (
        let index = Math.max(
            soloBerensteinBody.activeStart + 1,
            soloBerensteinBody.collisionStartSegment + 1
        );
        index < shaftBendEnd;
        index++
    ) {
        if (Math.min(
            soloBerensteinBody.restLength[index - 1],
            soloBerensteinBody.restLength[index]
        ) < soloBerensteinBody.segmentLength * 0.8) continue;
        const ax = soloBerensteinBody.x[index] - soloBerensteinBody.x[index - 1];
        const ay = soloBerensteinBody.y[index] - soloBerensteinBody.y[index - 1];
        const az = soloBerensteinBody.z[index] - soloBerensteinBody.z[index - 1];
        const bx = soloBerensteinBody.x[index + 1] - soloBerensteinBody.x[index];
        const by = soloBerensteinBody.y[index + 1] - soloBerensteinBody.y[index];
        const bz = soloBerensteinBody.z[index + 1] - soloBerensteinBody.z[index];
        const bendDegrees = Math.acos(Math.max(-1, Math.min(1,
                (ax * bx + ay * by + az * bz) /
                Math.max(EPSILON, Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz))
            ))) * 180 / Math.PI;
        maximumSoloBerensteinShaftBend = Math.max(
            maximumSoloBerensteinShaftBend,
            bendDegrees
        );
    }
}
const soloBerensteinFeedShaftEnd = Math.max(
    soloBerensteinBody.activeStart,
    soloBerensteinBody.activeEnd - 5
);
const soloBerensteinFeedReverseBend = reversePlanarBendDegrees(
    soloBerensteinBody,
    soloBerensteinBody.collisionStartSegment,
    soloBerensteinFeedShaftEnd
);
const soloBerensteinFeedAlternatingBend = alternatingPlanarBendDegrees(
    soloBerensteinBody,
    soloBerensteinBody.collisionStartSegment,
    soloBerensteinFeedShaftEnd
);
console.log('solo berenstein feed reverse bend degrees',
    soloBerensteinFeedReverseBend);
console.log('solo berenstein feed alternating bend degrees',
    soloBerensteinFeedAlternatingBend);
assert.ok(soloBerensteinFeedAlternatingBend.total <= 30,
    `Berenstein should not accumulate a sinusoidal shaft while feed is held (${soloBerensteinFeedAlternatingBend.total} degrees)`);
assert.ok(soloBerensteinFeedAlternatingBend.maximum <= 10,
    `Berenstein feed should not contain a visible local bend reversal (${soloBerensteinFeedAlternatingBend.maximum} degrees)`);
assert.ok(soloBerensteinFeedReverseBend.total <= 40,
    `Berenstein should remain globally stiff during continuous feed (${soloBerensteinFeedReverseBend.total} degrees)`);
let maximumSoloBerensteinSettlingTipStep = 0;
let maximumSoloBerensteinSettlingTipStepAt = -1;
let maximumSoloBerensteinLateSettlingTipStep = 0;
let maximumSoloBerensteinLateSettlingSpeed = 0;
let previousSoloBerensteinSettlingTip = previousSoloBerensteinTip;
for (let step = 0; step < 180; step++) {
    soloBerenstein.advance(0, 1 / 120, 0);
    soloBerenstein.stepPhysics(1 / 120, { collisions: false });
    soloBerenstein.syncXpbdBody(soloBerensteinBody);
    soloBerensteinWorld.stepFixed();
    const tipIndex = soloBerensteinBody.activeEnd;
    const tip = [
        soloBerensteinBody.x[tipIndex],
        soloBerensteinBody.y[tipIndex],
        soloBerensteinBody.z[tipIndex]
    ];
    const soloBerensteinSettlingTipStep = Math.hypot(
            tip[0] - previousSoloBerensteinSettlingTip[0],
            tip[1] - previousSoloBerensteinSettlingTip[1],
            tip[2] - previousSoloBerensteinSettlingTip[2]
        );
    if (soloBerensteinSettlingTipStep > maximumSoloBerensteinSettlingTipStep) {
        maximumSoloBerensteinSettlingTipStep = soloBerensteinSettlingTipStep;
        maximumSoloBerensteinSettlingTipStepAt = step;
    }
    if (step >= 120) {
        maximumSoloBerensteinLateSettlingTipStep = Math.max(
            maximumSoloBerensteinLateSettlingTipStep,
            Math.hypot(
                tip[0] - previousSoloBerensteinSettlingTip[0],
                tip[1] - previousSoloBerensteinSettlingTip[1],
                tip[2] - previousSoloBerensteinSettlingTip[2]
            )
        );
        maximumSoloBerensteinLateSettlingSpeed = Math.max(
            maximumSoloBerensteinLateSettlingSpeed,
            maximumBodySpeed(soloBerensteinBody)
        );
    }
    previousSoloBerensteinSettlingTip = tip;
}
const soloBerensteinReleaseControlIndices = [];
for (let index = 0; index <= soloBerensteinBody.activeEnd; index++) {
    if (soloBerensteinBody.controlEnabled[index]) {
        soloBerensteinReleaseControlIndices.push(index);
    }
}
assert.equal(soloBerensteinReleaseControlIndices.length, 0,
    'a released legacy Berenstein should retain its convected material feed');
const soloBerensteinShaftEnd = Math.max(
    soloBerensteinBody.activeStart,
    soloBerensteinBody.activeEnd - 5
);
const soloBerensteinReverseBend = reversePlanarBendDegrees(
    soloBerensteinBody,
    soloBerensteinBody.collisionStartSegment,
    soloBerensteinShaftEnd
);
const soloBerensteinAlternatingBend = alternatingPlanarBendDegrees(
    soloBerensteinBody,
    soloBerensteinBody.collisionStartSegment,
    soloBerensteinShaftEnd
);
let soloBerensteinRouteProgress = -Infinity;
let maximumSoloBerensteinBacktrack = 0;
let maximumSoloBerensteinRadial = 0;
let previousSoloBerensteinProgress = -Infinity;
for (
    let index = soloBerensteinBody.activeStart;
    index <= soloBerensteinShaftEnd;
    index++
) {
    const route = aorticArchState(
        soloBerensteinBody.x[index],
        soloBerensteinBody.y[index],
        soloBerensteinBody.z[index],
        soloArchOptions
    );
    soloBerensteinRouteProgress = Math.max(
        soloBerensteinRouteProgress,
        route.progress
    );
    maximumSoloBerensteinRadial = Math.max(
        maximumSoloBerensteinRadial,
        route.radial
    );
    if (Number.isFinite(previousSoloBerensteinProgress)) {
        maximumSoloBerensteinBacktrack = Math.max(
            maximumSoloBerensteinBacktrack,
            previousSoloBerensteinProgress - route.progress
        );
    }
    previousSoloBerensteinProgress = route.progress;
}
console.log('solo berenstein arch route progress mm', soloBerensteinRouteProgress.toFixed(2));
console.log('solo berenstein shaft max backtrack mm', maximumSoloBerensteinBacktrack.toFixed(2));
console.log('solo berenstein shaft max radial mm', maximumSoloBerensteinRadial.toFixed(2));
console.log('solo berenstein max tip step mm', maximumSoloBerensteinTipStep.toFixed(4));
console.log('solo berenstein max tip step at', maximumSoloBerensteinTipStepAt);
console.log('solo berenstein deployed max tip step mm',
    maximumSoloBerensteinDeployedTipStep.toFixed(4));
console.log('solo berenstein max shaft bend degrees', maximumSoloBerensteinShaftBend.toFixed(2));
console.log('solo berenstein max segment error mm',
    maximumSoloBerensteinSegmentError.toFixed(3));
console.log('solo berenstein max feed segment error mm',
    maximumSoloBerensteinFeedSegmentError.toFixed(3));
console.log('solo berenstein reverse bend degrees', soloBerensteinReverseBend);
console.log('solo berenstein alternating bend degrees', soloBerensteinAlternatingBend);
console.log('solo berenstein settling max tip step mm',
    maximumSoloBerensteinSettlingTipStep.toFixed(4));
console.log('solo berenstein settling max tip step at',
    maximumSoloBerensteinSettlingTipStepAt);
console.log('solo berenstein late settling max tip step mm',
    maximumSoloBerensteinLateSettlingTipStep.toFixed(4));
console.log('solo berenstein late settling max speed mm/s',
    maximumSoloBerensteinLateSettlingSpeed.toFixed(4));
assert.ok(soloBerensteinRouteProgress > 70,
    `the unsupported Berenstein shaft should advance through the aortic arch (${soloBerensteinRouteProgress} mm)`);
assert.ok(maximumSoloBerensteinBacktrack <= 4,
    `the unsupported Berenstein shaft should not double back (${maximumSoloBerensteinBacktrack} mm)`);
assert.ok(maximumSoloBerensteinRadial <= soloArchOptions.lumenRadius,
    `the unsupported Berenstein shaft should remain inside the aortic lumen (${maximumSoloBerensteinRadial} mm)`);
assert.ok(maximumSoloBerensteinTipStep <= 4.5,
    `solo Berenstein insertion should not teleport its tip (${maximumSoloBerensteinTipStep} mm)`);
assert.ok(maximumSoloBerensteinDeployedTipStep <= 3.6,
    `a deployed unsupported Berenstein should advance without visible jumps (${maximumSoloBerensteinDeployedTipStep} mm)`);
assert.ok(maximumSoloBerensteinShaftBend <= 33,
    `solo Berenstein insertion should not create a transient shaft fold (${maximumSoloBerensteinShaftBend} degrees)`);
assert.ok(maximumSoloBerensteinSegmentError <= 1.2,
    `solo Berenstein insertion should retain segment length (${maximumSoloBerensteinSegmentError} mm error)`);
assert.ok(maximumSoloBerensteinFeedSegmentError <= 1,
    `the remeshed Berenstein feed segment should remain bounded (${maximumSoloBerensteinFeedSegmentError} mm error)`);
assert.ok(soloBerensteinAlternatingBend.total <= 12.5,
    `a released Berenstein shaft should not retain a sinusoidal wave (${soloBerensteinAlternatingBend.total} degrees)`);
assert.ok(soloBerensteinReverseBend.total <= 20,
    `a released Berenstein shaft should relax accumulated reverse curvature (${soloBerensteinReverseBend.total} degrees)`);
assert.ok(maximumSoloBerensteinSettlingTipStep <= 0.55,
    `a released solo Berenstein should settle without distal shaking (${maximumSoloBerensteinSettlingTipStep} mm)`);
assert.ok(maximumSoloBerensteinLateSettlingTipStep <= 0.08,
    `a released solo Berenstein should stop visibly moving (${maximumSoloBerensteinLateSettlingTipStep} mm)`);
assert.ok(maximumSoloBerensteinLateSettlingSpeed <= 1,
    `a released solo Berenstein should damp instead of fluttering (${maximumSoloBerensteinLateSettlingSpeed} mm/s)`);
const soloBerensteinShaftBendCompliance =
    soloBerensteinBody.bendComplianceByNode[Math.max(
        soloBerensteinBody.activeStart,
        soloBerensteinBody.activeEnd - 8
    )];
const soloBerensteinTipBendCompliance =
    soloBerensteinBody.bendComplianceByNode[soloBerensteinBody.activeEnd];
assert.ok(
    Number.isFinite(releasedPigtailShaftBendCompliance) &&
        releasedPigtailShaftBendCompliance > 0 &&
        Number.isFinite(releasedPigtailTipBendCompliance) &&
        releasedPigtailTipBendCompliance > 0,
    'the released Pigtail must retain finite shaft and distal bending stiffness'
);
assert.ok(
    Number.isFinite(soloBerensteinShaftBendCompliance) &&
        Number.isFinite(soloBerensteinTipBendCompliance),
    'the Berenstein material profile must remain finite'
);
soloBerenstein.updateMesh();
assert.equal(soloBerenstein.tipMarker.visible, true,
    'the Berenstein should show a marker at the start of its distal profile');
assert.equal(soloBerenstein.tipMarker.userData.catheterType, 'berenstein');
assert.equal(soloBerenstein.tipMarker.userData.tipLengthMm, 18,
    'the Berenstein profile marker should sit 18 mm from the tip');
assert.ok(
    Number.isFinite(soloBerenstein.tipMarker.position.x) &&
    Number.isFinite(soloBerenstein.tipMarker.position.y) &&
    Number.isFinite(soloBerenstein.tipMarker.position.z),
    'the Berenstein profile marker should have a finite physical position'
);
const soloBerensteinTipBeforeRotation = [
    soloBerensteinBody.x[soloBerensteinBody.activeEnd],
    soloBerensteinBody.y[soloBerensteinBody.activeEnd],
    soloBerensteinBody.z[soloBerensteinBody.activeEnd]
];
const soloBerensteinProfileBaseIndex = soloBerensteinBody.activeEnd - 5;
const soloBerensteinProfileBaseDirection = [
    soloBerensteinBody.x[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.x[soloBerensteinProfileBaseIndex - 1],
    soloBerensteinBody.y[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.y[soloBerensteinProfileBaseIndex - 1],
    soloBerensteinBody.z[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.z[soloBerensteinProfileBaseIndex - 1]
];
const soloBerensteinTipDirection = [
    soloBerensteinBody.x[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.x[soloBerensteinBody.activeEnd - 1],
    soloBerensteinBody.y[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.y[soloBerensteinBody.activeEnd - 1],
    soloBerensteinBody.z[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.z[soloBerensteinBody.activeEnd - 1]
];
const soloBerensteinProfileBaseBeforeRotation = [
    soloBerensteinBody.x[soloBerensteinProfileBaseIndex],
    soloBerensteinBody.y[soloBerensteinProfileBaseIndex],
    soloBerensteinBody.z[soloBerensteinProfileBaseIndex]
];
const soloBerensteinAxisBeforeRotationLength =
    Math.hypot(...soloBerensteinProfileBaseDirection);
const soloBerensteinAxisBeforeRotation =
    soloBerensteinProfileBaseDirection.map(
        value => value / soloBerensteinAxisBeforeRotationLength
    );
const soloBerensteinTipOffsetBeforeRotation = [
    soloBerensteinTipBeforeRotation[0] -
        soloBerensteinProfileBaseBeforeRotation[0],
    soloBerensteinTipBeforeRotation[1] -
        soloBerensteinProfileBaseBeforeRotation[1],
    soloBerensteinTipBeforeRotation[2] -
        soloBerensteinProfileBaseBeforeRotation[2]
];
const soloBerensteinTipAxialBeforeRotation =
    soloBerensteinTipOffsetBeforeRotation[0] * soloBerensteinAxisBeforeRotation[0] +
    soloBerensteinTipOffsetBeforeRotation[1] * soloBerensteinAxisBeforeRotation[1] +
    soloBerensteinTipOffsetBeforeRotation[2] * soloBerensteinAxisBeforeRotation[2];
const soloBerensteinTipRadialBeforeRotation = Math.hypot(
    soloBerensteinTipOffsetBeforeRotation[0] -
        soloBerensteinAxisBeforeRotation[0] * soloBerensteinTipAxialBeforeRotation,
    soloBerensteinTipOffsetBeforeRotation[1] -
        soloBerensteinAxisBeforeRotation[1] * soloBerensteinTipAxialBeforeRotation,
    soloBerensteinTipOffsetBeforeRotation[2] -
        soloBerensteinAxisBeforeRotation[2] * soloBerensteinTipAxialBeforeRotation
);
const soloBerensteinProfileBend = Math.acos(Math.max(-1, Math.min(1,
    (
        soloBerensteinProfileBaseDirection[0] * soloBerensteinTipDirection[0] +
        soloBerensteinProfileBaseDirection[1] * soloBerensteinTipDirection[1] +
        soloBerensteinProfileBaseDirection[2] * soloBerensteinTipDirection[2]
    ) / (
        Math.hypot(...soloBerensteinProfileBaseDirection) *
        Math.hypot(...soloBerensteinTipDirection)
    )
))) * 180 / Math.PI;
console.log('solo berenstein distal profile bend degrees',
    soloBerensteinProfileBend.toFixed(2));
let maximumSoloBerensteinTipRotationTravel = 0;
let maximumSoloBerensteinTipRotationStep = 0;
let minimumSoloBerensteinBendDuringRotation = soloBerensteinProfileBend;
let maximumSoloBerensteinBendDuringRotation = soloBerensteinProfileBend;
let previousSoloBerensteinRotatingTip = soloBerensteinTipBeforeRotation;
for (let step = 0; step < 80; step++) {
    soloBerenstein.rotate(1, 1 / 120);
    soloBerenstein.stepPhysics(1 / 120, { collisions: false });
    soloBerenstein.syncXpbdBody(soloBerensteinBody);
    soloBerensteinWorld.stepFixed();
    const rotatingTip = [
        soloBerensteinBody.x[soloBerensteinBody.activeEnd],
        soloBerensteinBody.y[soloBerensteinBody.activeEnd],
        soloBerensteinBody.z[soloBerensteinBody.activeEnd]
    ];
    maximumSoloBerensteinTipRotationTravel = Math.max(
        maximumSoloBerensteinTipRotationTravel,
        Math.hypot(
            rotatingTip[0] - soloBerensteinTipBeforeRotation[0],
            rotatingTip[1] - soloBerensteinTipBeforeRotation[1],
            rotatingTip[2] - soloBerensteinTipBeforeRotation[2]
        )
    );
    maximumSoloBerensteinTipRotationStep = Math.max(
        maximumSoloBerensteinTipRotationStep,
        Math.hypot(
            rotatingTip[0] - previousSoloBerensteinRotatingTip[0],
            rotatingTip[1] - previousSoloBerensteinRotatingTip[1],
            rotatingTip[2] - previousSoloBerensteinRotatingTip[2]
        )
    );
    const rotatingProfileBaseDirection = [
        soloBerensteinBody.x[soloBerensteinProfileBaseIndex] -
            soloBerensteinBody.x[soloBerensteinProfileBaseIndex - 1],
        soloBerensteinBody.y[soloBerensteinProfileBaseIndex] -
            soloBerensteinBody.y[soloBerensteinProfileBaseIndex - 1],
        soloBerensteinBody.z[soloBerensteinProfileBaseIndex] -
            soloBerensteinBody.z[soloBerensteinProfileBaseIndex - 1]
    ];
    const rotatingTipDirection = [
        soloBerensteinBody.x[soloBerensteinBody.activeEnd] -
            soloBerensteinBody.x[soloBerensteinBody.activeEnd - 1],
        soloBerensteinBody.y[soloBerensteinBody.activeEnd] -
            soloBerensteinBody.y[soloBerensteinBody.activeEnd - 1],
        soloBerensteinBody.z[soloBerensteinBody.activeEnd] -
            soloBerensteinBody.z[soloBerensteinBody.activeEnd - 1]
    ];
    const rotatingProfileBend = Math.acos(Math.max(-1, Math.min(1,
        (
            rotatingProfileBaseDirection[0] * rotatingTipDirection[0] +
            rotatingProfileBaseDirection[1] * rotatingTipDirection[1] +
            rotatingProfileBaseDirection[2] * rotatingTipDirection[2]
        ) / (
            Math.hypot(...rotatingProfileBaseDirection) *
            Math.hypot(...rotatingTipDirection)
        )
    ))) * 180 / Math.PI;
    minimumSoloBerensteinBendDuringRotation = Math.min(
        minimumSoloBerensteinBendDuringRotation,
        rotatingProfileBend
    );
    maximumSoloBerensteinBendDuringRotation = Math.max(
        maximumSoloBerensteinBendDuringRotation,
        rotatingProfileBend
    );
    previousSoloBerensteinRotatingTip = rotatingTip;
}
const soloBerensteinTipRotationTravel = Math.hypot(
    soloBerensteinBody.x[soloBerensteinBody.activeEnd] -
        soloBerensteinTipBeforeRotation[0],
    soloBerensteinBody.y[soloBerensteinBody.activeEnd] -
        soloBerensteinTipBeforeRotation[1],
    soloBerensteinBody.z[soloBerensteinBody.activeEnd] -
        soloBerensteinTipBeforeRotation[2]
);
const soloBerensteinProfileBaseAfterRotation = [
    soloBerensteinBody.x[soloBerensteinProfileBaseIndex],
    soloBerensteinBody.y[soloBerensteinProfileBaseIndex],
    soloBerensteinBody.z[soloBerensteinProfileBaseIndex]
];
const soloBerensteinProfileBaseDirectionAfterRotation = [
    soloBerensteinBody.x[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.x[soloBerensteinProfileBaseIndex - 1],
    soloBerensteinBody.y[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.y[soloBerensteinProfileBaseIndex - 1],
    soloBerensteinBody.z[soloBerensteinProfileBaseIndex] -
        soloBerensteinBody.z[soloBerensteinProfileBaseIndex - 1]
];
const soloBerensteinTipDirectionAfterRotation = [
    soloBerensteinBody.x[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.x[soloBerensteinBody.activeEnd - 1],
    soloBerensteinBody.y[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.y[soloBerensteinBody.activeEnd - 1],
    soloBerensteinBody.z[soloBerensteinBody.activeEnd] -
        soloBerensteinBody.z[soloBerensteinBody.activeEnd - 1]
];
const soloBerensteinProfileBendAfterRotation = Math.acos(Math.max(-1, Math.min(1,
    (
        soloBerensteinProfileBaseDirectionAfterRotation[0] *
            soloBerensteinTipDirectionAfterRotation[0] +
        soloBerensteinProfileBaseDirectionAfterRotation[1] *
            soloBerensteinTipDirectionAfterRotation[1] +
        soloBerensteinProfileBaseDirectionAfterRotation[2] *
            soloBerensteinTipDirectionAfterRotation[2]
    ) / (
        Math.hypot(...soloBerensteinProfileBaseDirectionAfterRotation) *
        Math.hypot(...soloBerensteinTipDirectionAfterRotation)
    )
))) * 180 / Math.PI;
const soloBerensteinTipOffsetAfterRotation = [
    soloBerensteinBody.x[soloBerensteinBody.activeEnd] -
        soloBerensteinProfileBaseAfterRotation[0],
    soloBerensteinBody.y[soloBerensteinBody.activeEnd] -
        soloBerensteinProfileBaseAfterRotation[1],
    soloBerensteinBody.z[soloBerensteinBody.activeEnd] -
        soloBerensteinProfileBaseAfterRotation[2]
];
const soloBerensteinAxisAfterRotationLength =
    Math.hypot(...soloBerensteinProfileBaseDirectionAfterRotation);
const soloBerensteinAxisAfterRotation =
    soloBerensteinProfileBaseDirectionAfterRotation.map(
        value => value / soloBerensteinAxisAfterRotationLength
    );
const soloBerensteinTipAxialAfterRotation =
    soloBerensteinTipOffsetAfterRotation[0] * soloBerensteinAxisAfterRotation[0] +
    soloBerensteinTipOffsetAfterRotation[1] * soloBerensteinAxisAfterRotation[1] +
    soloBerensteinTipOffsetAfterRotation[2] * soloBerensteinAxisAfterRotation[2];
const soloBerensteinTipRadialAfterRotation = Math.hypot(
    soloBerensteinTipOffsetAfterRotation[0] -
        soloBerensteinAxisAfterRotation[0] * soloBerensteinTipAxialAfterRotation,
    soloBerensteinTipOffsetAfterRotation[1] -
        soloBerensteinAxisAfterRotation[1] * soloBerensteinTipAxialAfterRotation,
    soloBerensteinTipOffsetAfterRotation[2] -
        soloBerensteinAxisAfterRotation[2] * soloBerensteinTipAxialAfterRotation
);
console.log('solo berenstein tip rotation travel mm',
    soloBerensteinTipRotationTravel.toFixed(3));
console.log('solo berenstein max tip rotation travel mm',
    maximumSoloBerensteinTipRotationTravel.toFixed(3));
console.log('solo berenstein max tip rotation step mm',
    maximumSoloBerensteinTipRotationStep.toFixed(3));
console.log('solo berenstein bend after rotation degrees',
    soloBerensteinProfileBendAfterRotation.toFixed(2));
console.log('solo berenstein bend range during rotation degrees',
    minimumSoloBerensteinBendDuringRotation.toFixed(2),
    maximumSoloBerensteinBendDuringRotation.toFixed(2));
console.log('solo berenstein radial after rotation mm',
    soloBerensteinTipRadialAfterRotation.toFixed(3));
console.log('solo berenstein radial before rotation mm',
    soloBerensteinTipRadialBeforeRotation.toFixed(3));
assert.ok(soloBerensteinProfileBend >= 10,
    `the Berenstein must retain an angled distal profile that can visibly rotate (${soloBerensteinProfileBend} degrees)`);
assert.ok(maximumSoloBerensteinTipRotationTravel >= 2,
    `Berenstein rotation should move its physical angled tip around the shaft axis (${maximumSoloBerensteinTipRotationTravel} mm)`);
assert.ok(maximumSoloBerensteinTipRotationStep <= 0.75,
    `Berenstein rotation should remain continuous instead of jumping (${maximumSoloBerensteinTipRotationStep} mm)`);
assert.ok(
    minimumSoloBerensteinBendDuringRotation >=
        soloBerensteinProfileBend * 0.85,
    `Berenstein rotation must preserve the angled tip instead of straightening it (${minimumSoloBerensteinBendDuringRotation} vs ${soloBerensteinProfileBend} degrees)`
);
assert.ok(
    soloBerensteinTipRadialAfterRotation >=
        soloBerensteinTipRadialBeforeRotation * 0.8,
    `Berenstein rotation must retain its radial tip offset (${soloBerensteinTipRadialAfterRotation} vs ${soloBerensteinTipRadialBeforeRotation} mm)`
);
soloBerenstein.dispose();

function releasedBerensteinDirectionAfterSplintedRotation(rotationCommand) {
    const wireLength = 160;
    const wireSpacing = 2;
    let guidewireInserted = 96;
    const wire = new ElasticRod(wireLength / wireSpacing + 1, wireSpacing);
    const alignWire = () => {
        for (let index = 0; index < wire.nodes.length; index++) {
            const node = wire.nodes[index];
            node.x = index * wireSpacing - wireLength +
                guidewireInserted - insertionSheathLength;
            node.y = 0;
            node.z = 0;
            node.vx = 0;
            node.vy = 0;
            node.vz = 0;
        }
    };
    alignWire();
    const catheter = new PigtailCatheter({
        wire,
        segmentLength: wireSpacing,
        guidewireLength: wireLength,
        tailProgressRef: () => guidewireInserted,
        vessel: catheterVessel,
        maxLength: wireLength
    });
    catheter.setType('berenstein');
    catheter.setExternalCollisionSolver(true);
    const world = new EndovascularPhysicsWorld({
        contactField: cylinderField(40),
        iterations: 6,
        penetrationIterations: 8
    });
    const body = world.createRod(
        'splinted-berenstein',
        160,
        4,
        { ...DEFAULT_TOOL_PROFILES.catheter }
    );
    const step = (advanceCommand = 0) => {
        catheter.advance(
            advanceCommand,
            1 / 120,
            guidewireInserted
        );
        catheter.stepPhysics(1 / 120, { collisions: false });
        catheter.syncXpbdBody(body);
        world.stepFixed();
    };

    for (let frame = 0; frame < 180; frame++) step(1);
    for (let frame = 0; frame < 80; frame++) {
        catheter.rotate(rotationCommand, 1 / 120);
        step();
    }
    catheter.rotate(0, 1 / 120);
    for (let frame = 0; frame < 144; frame++) {
        guidewireInserted = 96 * (1 - (frame + 1) / 144);
        alignWire();
        step();
    }
    for (let frame = 0; frame < 120; frame++) step();

    const tipIndex = body.activeEnd;
    const baseIndex = Math.max(body.activeStart + 1, tipIndex - 5);
    const axisX = body.x[baseIndex] - body.x[baseIndex - 1];
    const axisY = body.y[baseIndex] - body.y[baseIndex - 1];
    const axisZ = body.z[baseIndex] - body.z[baseIndex - 1];
    const axisLength = Math.hypot(axisX, axisY, axisZ);
    const unitAxis = [
        axisX / axisLength,
        axisY / axisLength,
        axisZ / axisLength
    ];
    const tipOffset = [
        body.x[tipIndex] - body.x[baseIndex],
        body.y[tipIndex] - body.y[baseIndex],
        body.z[tipIndex] - body.z[baseIndex]
    ];
    const axial =
        tipOffset[0] * unitAxis[0] +
        tipOffset[1] * unitAxis[1] +
        tipOffset[2] * unitAxis[2];
    const radial = [
        tipOffset[0] - unitAxis[0] * axial,
        tipOffset[1] - unitAxis[1] * axial,
        tipOffset[2] - unitAxis[2] * axial
    ];
    const radialLength = Math.hypot(...radial);
    catheter.dispose();
    return {
        radialLength,
        direction: radial.map(value => value / Math.max(1e-6, radialLength))
    };
}

const unrotatedSplintedBerenstein =
    releasedBerensteinDirectionAfterSplintedRotation(0);
const rotatedSplintedBerenstein =
    releasedBerensteinDirectionAfterSplintedRotation(1);
const splintedRotationDirectionDot =
    unrotatedSplintedBerenstein.direction[0] *
        rotatedSplintedBerenstein.direction[0] +
    unrotatedSplintedBerenstein.direction[1] *
        rotatedSplintedBerenstein.direction[1] +
    unrotatedSplintedBerenstein.direction[2] *
        rotatedSplintedBerenstein.direction[2];
console.log('splinted berenstein released rotation direction dot',
    splintedRotationDirectionDot.toFixed(3));
assert.ok(unrotatedSplintedBerenstein.radialLength >= 2,
    'the Berenstein tip should recover after its guidewire is withdrawn');
assert.ok(rotatedSplintedBerenstein.radialLength >= 2,
    'a rotated Berenstein should retain its curved tip after guidewire withdrawal');
assert.ok(splintedRotationDirectionDot <= 0.5,
    `Q/E torque applied over a guidewire must orient the released Berenstein tip (${splintedRotationDirectionDot})`);

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
    const world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120, maxSubsteps: 8 });
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
assert.equal(replay30.stats.lastSubsteps, 4, '30 FPS rendering should execute four 1/120 s substeps');
assert.equal(replay60.stats.lastSubsteps, 2, '60 FPS rendering should use two 1/120 s substeps');
assert.equal(replay120.stats.lastSubsteps, 1, '120 FPS rendering should use one 1/120 s substep');
assert.equal(replay30.stats.droppedTime, 0, '30 FPS rendering must not discard physics time');
assert.equal(replay30.stats.backlogSteps, 0, '30 FPS rendering should fully consume its fixed-step backlog');

const backlogWorld = new EndovascularPhysicsWorld({
    fixedDt: 1 / 120,
    maxSubsteps: 2
});
backlogWorld.advance(1 / 30);
assert.equal(backlogWorld.stepCount, 2, 'a bounded batch should execute its allowed substeps');
assert.equal(backlogWorld.getStats().backlogSteps, 2,
    'unexecuted fixed steps must remain queued rather than being discarded');
assert.equal(backlogWorld.droppedTime, 0, 'queuing catch-up work must not count as dropped time');
backlogWorld.advance(0);
assert.equal(backlogWorld.stepCount, 4, 'a later batch should catch up the retained steps');
assert.equal(backlogWorld.getStats().backlogSteps, 0, 'catch-up should drain the retained backlog');

console.log('xpbd stability total average ms', stabilityStats.phases.total.averageMs.toFixed(4));
console.log('xpbd stability total p95 ms', stabilityStats.phases.total.p95Ms.toFixed(4));
console.log('xpbd stability max length error %', (stabilityStats.bodies[0].maxLengthError * 100).toFixed(4));

stableRod.wallLambda.fill(1);
stableRod.wallFrictionLambda.fill(1);
stabilityWorld.resetSimulationState();
assert.equal(stabilityWorld.stepCount, 0, 'simulation reset should restore the step counter');
assert.equal(stabilityWorld.droppedTime, 0, 'simulation reset should clear dropped time');
assert.ok(stableRod.wallLambda.every(value => value === 0), 'simulation reset should clear warm-started wall contacts');
assert.ok(stableRod.wallFrictionLambda.every(value => value === 0),
    'simulation reset should clear current-step wall friction loads');
