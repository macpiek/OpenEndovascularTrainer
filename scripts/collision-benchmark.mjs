import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import * as THREE from 'three';
import { ElasticRod } from '../src/physics/elasticRod.js';
import { GuidewireSolver } from '../src/physics/guidewireSolver.js';
import {
    DEFAULT_TOOL_PROFILES,
    EndovascularPhysicsWorld
} from '../src/physics/endovascularPhysicsWorld.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import { generateVessel } from '../src/vesselGeometry.js';

const args = new Map(process.argv.slice(2).map(value => {
    const [key, raw = 'true'] = value.replace(/^--/, '').split('=');
    return [key, raw];
}));
const requestedMode = args.get('mode') || 'all';
const disableToolContact = args.get('disable-tool-contact') === 'true';
const disableContainment = args.get('disable-containment') === 'true';
const outputDirectory = path.resolve(args.get('output') || 'reports');
const EPSILON = 1e-8;

function percentile(values, fraction) {
    if (!values.length) return 0;
    const ordered = [...values].sort((a, b) => a - b);
    return ordered[Math.min(ordered.length - 1, Math.floor((ordered.length - 1) * fraction))];
}

function summarize(values) {
    return {
        average: values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length),
        p95: percentile(values, 0.95),
        maximum: values.length ? Math.max(...values) : 0,
        samples: values.length
    };
}

function maxSegmentError(body, restLength) {
    let maximum = 0;
    const count = body.nodes?.length ?? body.count;
    const start = body.nodes ? 0 : body.activeStart;
    const end = body.nodes ? count - 1 : Math.min(body.activeEnd, body.segmentCount);
    for (let index = start; index < end; index++) {
        const ax = body.nodes ? body.nodes[index].x : body.x[index];
        const ay = body.nodes ? body.nodes[index].y : body.y[index];
        const az = body.nodes ? body.nodes[index].z : body.z[index];
        const bx = body.nodes ? body.nodes[index + 1].x : body.x[index + 1];
        const by = body.nodes ? body.nodes[index + 1].y : body.y[index + 1];
        const bz = body.nodes ? body.nodes[index + 1].z : body.z[index + 1];
        const expected = body.restLength?.[index] ?? restLength;
        maximum = Math.max(maximum, Math.abs(Math.hypot(bx - ax, by - ay, bz - az) - expected));
    }
    return maximum;
}

function maxRelativeSegmentError(body) {
    let maximum = 0;
    const start = body.activeStart;
    const end = Math.min(body.activeEnd, body.segmentCount);
    for (let index = start; index < end; index++) {
        const length = Math.hypot(
            body.x[index + 1] - body.x[index],
            body.y[index + 1] - body.y[index],
            body.z[index + 1] - body.z[index]
        );
        maximum = Math.max(
            maximum,
            Math.abs(length - body.restLength[index]) / Math.max(EPSILON, body.restLength[index])
        );
    }
    return maximum;
}

function maxRelativeSegmentErrorIndex(body) {
    let maximum = -1;
    let maximumIndex = -1;
    const start = body.activeStart;
    const end = Math.min(body.activeEnd, body.segmentCount);
    for (let index = start; index < end; index++) {
        const length = Math.hypot(
            body.x[index + 1] - body.x[index],
            body.y[index + 1] - body.y[index],
            body.z[index + 1] - body.z[index]
        );
        const relativeError = Math.abs(length - body.restLength[index]) /
            Math.max(EPSILON, body.restLength[index]);
        if (relativeError <= maximum) continue;
        maximum = relativeError;
        maximumIndex = index;
    }
    return maximumIndex;
}

function segmentErrorDetails(body, index) {
    if (index < 0 || index >= body.segmentCount) return null;
    return {
        currentLength: Math.hypot(
            body.x[index + 1] - body.x[index],
            body.y[index + 1] - body.y[index],
            body.z[index + 1] - body.z[index]
        ),
        restLength: body.restLength[index],
        start: [body.x[index], body.y[index], body.z[index]],
        end: [body.x[index + 1], body.y[index + 1], body.z[index + 1]],
        targetStart: [body.restShapeX[index], body.restShapeY[index], body.restShapeZ[index]],
        targetEnd: [body.restShapeX[index + 1], body.restShapeY[index + 1], body.restShapeZ[index + 1]],
        shapeEnabled: [body.restShapeEnabled[index], body.restShapeEnabled[index + 1]]
    };
}

function maxBodyBendAngleDegrees(body) {
    let maximum = 0;
    for (let index = Math.max(1, body.activeStart + 1); index < body.activeEnd; index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const az = body.z[index] - body.z[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const bz = body.z[index + 1] - body.z[index];
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator <= EPSILON) continue;
        const cosine = Math.max(-1, Math.min(1, (ax * bx + ay * by + az * bz) / denominator));
        maximum = Math.max(maximum, Math.acos(cosine) * 180 / Math.PI);
    }
    return maximum;
}

function bodyIsFinite(body) {
    for (let index = body.activeStart; index <= body.activeEnd; index++) {
        if (
            !Number.isFinite(body.x[index]) ||
            !Number.isFinite(body.y[index]) ||
            !Number.isFinite(body.z[index]) ||
            !Number.isFinite(body.velocityX[index]) ||
            !Number.isFinite(body.velocityY[index]) ||
            !Number.isFinite(body.velocityZ[index])
        ) return false;
    }
    return true;
}

function xpbdPhase(step) {
    if (step < 600) return 'full-insert';
    if (step < 1200) return 'branch-stenosis-taper-contact';
    if (step < 1600) return 'pigtail-deploy-rotate';
    if (step < 2000) return 'berenstein-deploy-rotate';
    if (step < 2280) return 'full-withdraw';
    return 'settle';
}

function heapUsed() {
    globalThis.gc?.();
    return process.memoryUsage().heapUsed;
}

function buildProceduralSampler(vessel) {
    const origin = new THREE.Vector3(vessel.sheath.end.x, vessel.sheath.end.y, vessel.sheath.end.z);
    const forward = new THREE.Vector3().subVectors(vessel.sheath.end, vessel.sheath.start).normalize();
    const side = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0));
    if (side.lengthSq() < EPSILON) side.set(1, 0, 0);
    side.normalize();
    const up = new THREE.Vector3().crossVectors(side, forward).normalize();
    const center = new THREE.Vector3();
    const ahead = new THREE.Vector3();
    const behind = new THREE.Vector3();
    const tangent = new THREE.Vector3();
    const point = { x: 0, y: 0, z: 0 };
    const tangentPoint = { x: 1, y: 0, z: 0 };
    const result = { point, tangent: tangentPoint, radius: 0 };

    const setCenter = (target, distance) => {
        const d = Math.max(0, distance);
        const fade = Math.min(1, d / 120);
        target.copy(origin)
            .addScaledVector(forward, d)
            .addScaledVector(side, Math.sin(d * 0.034) * 12 * fade)
            .addScaledVector(up, (Math.sin(d * 0.021 + 0.5) - Math.sin(0.5)) * 10 * fade);
        return target;
    };

    return distance => {
        const d = Math.max(0, distance);
        setCenter(center, d);
        setCenter(ahead, d + 0.5);
        setCenter(behind, Math.max(0, d - 0.5));
        tangent.subVectors(ahead, behind).normalize();
        point.x = center.x;
        point.y = center.y;
        point.z = center.z;
        tangentPoint.x = tangent.x;
        tangentPoint.y = tangent.y;
        tangentPoint.z = tangent.z;
        result.radius = 7.2 + Math.sin(d * 0.017) * 1.1 - Math.exp(-((d - 310) ** 2) / 520) * 2.2;
        return result;
    };
}

function benchmarkLegacy() {
    const dt = 1 / 60;
    const segmentLength = 5;
    const nodeCount = 201;
    const guidewireLength = segmentLength * (nodeCount - 1);
    const { vessel } = generateVessel(140, 0);
    const sampler = buildProceduralSampler(vessel);
    const wire = new ElasticRod(nodeCount, segmentLength, { constraintIterations: 28 });
    const solver = new GuidewireSolver({
        rod: wire,
        segmentLength,
        guidewireLength,
        sheath: vessel.sheath,
        lumenSampler: sampler,
        advanceRate: 44,
        minInsert: 0,
        maxInsert: guidewireLength,
        lumenClearance: DEFAULT_TOOL_PROFILES.guidewire.radius,
        meshClearance: DEFAULT_TOOL_PROFILES.guidewire.radius,
        straightening: 0.72,
        routeBlend: 0.018,
        relaxationIterations: 4,
        lengthIterations: 10,
        finalCollisionPasses: 3,
        finalLengthPasses: 2,
        finalProjectionPasses: 2
    });
    solver.initialize();
    const physicsTimes = [];
    const narrowTimes = [];
    let queryCount = 0;
    let maximumContacts = 0;
    let maximumPenetration = 0;
    let stabilityRepairs = 0;
    const heapStart = heapUsed();

    const step = command => {
        const start = performance.now();
        solver.advance(command, dt);
        solver.solve(dt, null, { iterations: command === 0 ? 3 : 4 });
        physicsTimes.push(performance.now() - start);
        const stats = solver.getPerformanceStats();
        narrowTimes.push(stats.projectMs);
        queryCount += stats.pointContactCount;
        stabilityRepairs += stats.stabilityRepaired ? 1 : 0;
        maximumContacts = Math.max(maximumContacts, stats.pointContactCount);
        for (let index = solver.firstLumenNodeIndex(); index < wire.nodes.length; index++) {
            const inserted = solver.insertedCoordinate(index);
            const route = solver.routeSample(inserted);
            const node = wire.nodes[index];
            const radial = Math.hypot(node.x - route.point.x, node.y - route.point.y, node.z - route.point.z);
            maximumPenetration = Math.max(
                maximumPenetration,
                radial + DEFAULT_TOOL_PROFILES.guidewire.radius - route.radius
            );
        }
    };

    while (solver.progress < solver.maxInsert - 1e-6) step(1);
    for (let index = 0; index < 120; index++) step(0);
    const fullyInserted = solver.progress;
    while (solver.progress > solver.minInsert + 1e-6) step(-1);
    for (let index = 0; index < 120; index++) step(0);

    const catheter = new PigtailCatheter({
        wire,
        segmentLength,
        guidewireLength,
        tailProgressRef: () => solver.progress,
        vessel,
        maxLength: 260
    });
    const catheterTimes = [];
    for (const type of ['pigtail', 'berenstein']) {
        catheter.setType(type);
        for (let frame = 0; frame < 240; frame++) {
            const start = performance.now();
            catheter.advance(frame < 180 ? 1 : 0, dt, fullyInserted);
            catheter.rotate(frame % 80 < 40 ? 1 : -1, dt);
            catheter.stepPhysics(dt);
            catheterTimes.push(performance.now() - start);
        }
    }
    catheter.dispose();
    const heapEnd = heapUsed();

    return {
        mode: 'legacy',
        scenarios: ['full-insert', 'full-withdraw', 'stenosis', 'pigtail-deploy-rotate', 'berenstein-deploy-rotate'],
        steps: physicsTimes.length,
        insertedMm: fullyInserted,
        physicsMs: summarize(physicsTimes),
        narrowPhaseMs: summarize(narrowTimes),
        catheterPhysicsMs: summarize(catheterTimes),
        contactQueries: queryCount,
        maxContactsPerStep: maximumContacts,
        maxPenetrationMm: Math.max(0, maximumPenetration),
        maxSegmentErrorMm: maxSegmentError(wire, segmentLength),
        maxSegmentErrorPercent: maxSegmentError(wire, segmentLength) / segmentLength * 100,
        stabilityRepairs,
        heapDeltaBytes: heapEnd - heapStart
    };
}

function setVector(target, x, y, z) {
    target.x = x;
    target.y = y;
    target.z = z;
}

class AnalyticBenchmarkField {
    constructor() {
        this.voxelSize = 0.5;
        this.point = { x: 0, y: 0, z: 0 };
        this.scratch = null;
        this.queryCount = 0;
    }

    state(x, y, z) {
        const stenosis = Math.exp(-((x - 210) ** 2) / 300) * 2.1;
        const taper = Math.max(0, Math.min(1.6, (x - 300) * 0.006));
        const trunkRadius = 6.5 - stenosis - taper;
        const trunkRadial = Math.hypot(y, z);
        let distance = trunkRadius - trunkRadial;
        let inwardX = 0;
        let inwardY = trunkRadial > EPSILON ? -y / trunkRadial : 1;
        let inwardZ = trunkRadial > EPSILON ? -z / trunkRadial : 0;
        let branchId = 0;
        if (x > 330) {
            const branchCenterY = (x - 330) * 0.16;
            const branchRadial = Math.hypot(y - branchCenterY, z);
            const branchDistance = 2.2 - branchRadial;
            if (branchDistance > distance) {
                distance = branchDistance;
                inwardY = branchRadial > EPSILON ? -(y - branchCenterY) / branchRadial : 1;
                inwardZ = branchRadial > EPSILON ? -z / branchRadial : 0;
                branchId = 1;
            }
        }
        return { distance, inwardX, inwardY, inwardZ, branchId };
    }

    querySphere(position, radius = 0, out) {
        this.queryCount++;
        const state = this.state(position.x, position.y, position.z);
        const gap = state.distance - radius;
        const penetration = Math.max(0, -gap);
        out.inside = state.distance >= 0;
        out.violation = gap < 0;
        out.signedDistance = state.distance;
        out.signedGap = gap;
        out.penetration = penetration;
        out.branchId = state.branchId;
        out.segmentT = 0;
        out.timeOfImpact = out.violation ? 0 : 1;
        out.source = 'analytic-benchmark';
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

    copyContact(target, source) {
        for (const key of [
            'inside', 'violation', 'signedDistance', 'signedGap', 'penetration',
            'branchId', 'segmentT', 'timeOfImpact', 'source'
        ]) target[key] = source[key];
        for (const key of ['point', 'target', 'closestPoint', 'normal', 'inward']) {
            setVector(target[key], source[key].x, source[key].y, source[key].z);
        }
    }

    queryCapsule(start, end, radius = 0, out) {
        if (!this.scratch) {
            this.scratch = {
                ...out,
                point: { x: 0, y: 0, z: 0 },
                target: { x: 0, y: 0, z: 0 },
                closestPoint: { x: 0, y: 0, z: 0 },
                normal: { x: 0, y: 0, z: 0 },
                inward: { x: 0, y: 0, z: 0 }
            };
        }
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const dz = end.z - start.z;
        const samples = Math.max(1, Math.ceil(Math.hypot(dx, dy, dz)));
        let bestGap = Infinity;
        for (let index = 0; index <= samples; index++) {
            const t = index / samples;
            this.point.x = start.x + dx * t;
            this.point.y = start.y + dy * t;
            this.point.z = start.z + dz * t;
            const contact = this.querySphere(this.point, radius, this.scratch);
            if (contact.signedGap >= bestGap) continue;
            bestGap = contact.signedGap;
            this.copyContact(out, contact);
            out.segmentT = t;
        }
        return out;
    }

    sweepSphere(previous, current, radius = 0, out) {
        const dx = current.x - previous.x;
        const dy = current.y - previous.y;
        const dz = current.z - previous.z;
        const samples = Math.max(2, Math.ceil(Math.hypot(dx, dy, dz) / Math.max(0.1, radius * 0.5)));
        for (let index = 0; index <= samples; index++) {
            const t = index / samples;
            this.point.x = previous.x + dx * t;
            this.point.y = previous.y + dy * t;
            this.point.z = previous.z + dz * t;
            const contact = this.querySphere(this.point, radius, out);
            if (!contact.violation) continue;
            out.timeOfImpact = t;
            return out;
        }
        this.querySphere(current, radius, out);
        out.timeOfImpact = 1;
        return out;
    }
}

function seedBody(body, yOffset = 0) {
    for (let index = 0; index < body.count; index++) {
        const x = index * body.segmentLength;
        const branchDeflection = x > 330 ? (x - 330) * 0.16 : 0;
        body.setNodePosition(index, x, yOffset + branchDeflection, 0);
    }
    body.captureRestConfiguration();
    body.copyCurrentToPrevious();
}

function benchmarkCenterlineY(x) {
    return x > 330 ? (x - 330) * 0.16 : 0;
}

function smoothRamp(value) {
    const t = Math.max(0, Math.min(1, value));
    return t * t * (3 - 2 * t);
}

function setArcRestShape(body, end, segmentCount, targetAngle, deployment, rotation, compliance) {
    const start = Math.max(0, end - segmentCount);
    const angleStep = targetAngle / Math.max(1, segmentCount);
    const curlFront = smoothRamp(deployment) * segmentCount;
    let x = start * body.segmentLength;
    let radial = 0;
    let direction = 0;
    for (let index = start; index <= end; index++) {
        const centerY = benchmarkCenterlineY(x);
        body.setRestShapeTarget(
            index,
            x,
            centerY + Math.cos(rotation) * radial,
            Math.sin(rotation) * radial,
            compliance
        );
        if (index === end) continue;
        const segment = index - start;
        const localCurl = smoothRamp(curlFront - (segmentCount - 1 - segment));
        direction += angleStep * localCurl;
        x += Math.cos(direction) * body.segmentLength;
        radial += Math.sin(direction) * body.segmentLength;
    }
    for (let index = start; index < end; index++) {
        body.restLength[index] = Math.max(0.5, Math.hypot(
            body.restShapeX[index + 1] - body.restShapeX[index],
            body.restShapeY[index + 1] - body.restShapeY[index],
            body.restShapeZ[index + 1] - body.restShapeZ[index]
        ));
    }
    for (let index = start + 1; index < end; index++) {
        const targetChord = Math.hypot(
            body.restShapeX[index + 1] - body.restShapeX[index - 1],
            body.restShapeY[index + 1] - body.restShapeY[index - 1],
            body.restShapeZ[index + 1] - body.restShapeZ[index - 1]
        );
        body.restBendChord[index] = targetChord;
    }
}

function clearBenchmarkRestShape(body, end, span = 24) {
    const start = Math.max(0, end - span);
    for (let index = start; index <= end; index++) body.clearRestShapeTarget(index);
    for (let index = Math.max(0, start - 1); index < Math.min(end, body.segmentCount); index++) {
        body.restLength[index] = body.segmentLength;
    }
    for (let index = Math.max(1, start); index < Math.min(end, body.count - 1); index++) {
        body.restBendChord[index] = body.segmentLength * 2;
    }
}

function configureXpbdScenario(step, wire, catheter, containment, externalContact) {
    const fullWireEnd = wire.count - 1;
    const shapeSupportEnd = 106;
    let wireEnd;
    if (step < 600) {
        wireEnd = 16 + Math.floor(step / 599 * (wire.count - 17));
    } else if (step < 1200) {
        wireEnd = fullWireEnd;
    } else if (step < 1280) {
        wireEnd = fullWireEnd - Math.floor((step - 1200) / 79 * (fullWireEnd - shapeSupportEnd));
    } else if (step < 1880) {
        wireEnd = shapeSupportEnd;
    } else if (step < 2000) {
        wireEnd = shapeSupportEnd + Math.floor((step - 1880) / 119 * (fullWireEnd - shapeSupportEnd));
    } else {
        wireEnd = fullWireEnd - Math.floor(Math.min(1, (step - 2000) / 279) * (wire.count - 17));
    }
    const catheterEnd = step < 600
        ? Math.max(2, Math.min(120, wireEnd - 8))
        : step < 2000
            ? 120
            : Math.max(2, Math.min(120, wireEnd - 8));
    wire.setActiveRange(0, wireEnd);
    catheter.setActiveRange(0, catheterEnd);
    wire.setCollisionRange(16, wireEnd - 1);
    catheter.setCollisionRange(16, catheterEnd - 1);

    containment.enabled = !disableContainment && catheterEnd >= 2;
    containment.startNode = 0;
    containment.endNode = Math.min(wireEnd, catheterEnd);
    externalContact.enabled = !disableToolContact && wireEnd > catheterEnd + 1;
    // The segment ending at the catheter tip is still governed by containment.
    // External capsule contact starts with the first segment beyond that plane.
    externalContact.startSegmentA = Math.max(0, catheterEnd + 1);
    externalContact.endSegmentA = Math.min(wireEnd - 1, catheterEnd + 8);
    externalContact.startSegmentB = Math.max(0, catheterEnd - 8);
    externalContact.endSegmentB = catheterEnd - 1;

    if (step === 2000) {
        clearBenchmarkRestShape(catheter, catheterEnd);
    }
    if (step === 1600) {
        clearBenchmarkRestShape(catheter, catheterEnd);
    }
    if (step >= 1280 && step < 1600) {
        const deployment = step < 1480
            ? smoothRamp((step - 1280) / 100)
            : smoothRamp((1600 - step) / 120);
        const rotation = Math.max(0, step - 1380) * 0.018;
        const pigtailRadius = 2.85;
        const pigtailSegments = 12;
        const pigtailAngleStep = 2 * Math.asin(catheter.segmentLength / (2 * pigtailRadius));
        setArcRestShape(
            catheter,
            catheterEnd,
            pigtailSegments,
            pigtailAngleStep * pigtailSegments,
            deployment,
            rotation,
            2e-4
        );
    } else if (step >= 1600 && step < 2000) {
        const deployment = step < 1880
            ? smoothRamp((step - 1600) / 120)
            : smoothRamp((2000 - step) / 120);
        const rotation = Math.max(0, step - 1720) * 0.018;
        setArcRestShape(
            catheter,
            catheterEnd,
            4,
            0.9,
            deployment,
            rotation,
            2e-4
        );
    }
    return { wireEnd, catheterEnd, settling: step >= 2280 };
}

function benchmarkXpbd() {
    const field = new AnalyticBenchmarkField();
    const world = new EndovascularPhysicsWorld({ contactField: field });
    const wire = world.createRod('guidewire', 201, 2.5, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        stretchCompliance: 0
    });
    const catheter = world.createRod('catheter', 201, 2.5, {
        ...DEFAULT_TOOL_PROFILES.catheter,
        stretchCompliance: 0,
        bendCompliance: 2e-4
    });
    seedBody(wire, 0.01);
    seedBody(catheter, 0);
    wire.setPinned(0, true);
    catheter.setPinned(0, true);
    wire.setCollisionRange(16, wire.segmentCount - 1);
    catheter.setCollisionRange(16, catheter.segmentCount - 1);
    world.addSheath({
        start: { x: 0, y: 0, z: 0 },
        end: { x: 40, y: 0, z: 0 },
        innerRadius: DEFAULT_TOOL_PROFILES.sheath.innerRadius,
        bodies: [wire, catheter]
    });
    const containment = world.addContainment(wire, catheter, {
        innerRadius: DEFAULT_TOOL_PROFILES.catheter.innerRadius,
        searchWindow: 8,
        enabled: false
    });
    const externalContact = world.addToolContact(wire, catheter, {
        friction: 0.08,
        openDistalB: true,
        enabled: false
    });
    const heapStart = heapUsed();
    for (let step = 0; step < 240; step++) world.stepFixed();
    field.queryCount = 0;
    const stepTimes = [];
    let maximumPenetration = 0;
    let maximumTransientPenetration = 0;
    let maximumContacts = 0;
    let maximumLengthErrorPercent = 0;
    let maximumLengthErrorStep = -1;
    let maximumLengthErrorBody = '';
    let maximumLengthErrorSegment = -1;
    let maximumLengthErrorDetails = null;
    let maximumBendAngleDegrees = 0;
    let finiteThroughout = true;
    let previousWireTip = -1;
    let previousCatheterTip = -1;
    for (let step = 0; step < 2400; step++) {
        const active = configureXpbdScenario(step, wire, catheter, containment, externalContact);
        const phase = step * 0.006;
        const wireTip = active.wireEnd;
        const catheterTip = active.catheterEnd;
        if (previousWireTip >= 0 && previousWireTip !== wireTip) wire.clearControlTarget(previousWireTip);
        if (previousCatheterTip >= 0 && previousCatheterTip !== catheterTip) {
            catheter.clearControlTarget(previousCatheterTip);
        }
        if (active.settling && step === 2280) {
            wire.clearControlTarget(wireTip);
            catheter.clearControlTarget(catheterTip);
        }
        if (!active.settling) {
            const wireTipX = wireTip * wire.segmentLength;
            wire.setControlTarget(
                wireTip,
                wireTipX,
                benchmarkCenterlineY(wireTipX) + Math.sin(phase) * 0.05,
                Math.cos(phase) * 0.1,
                2e-5
            );
            if (step < 1200 || step >= 2000) {
                const catheterTipX = catheterTip * catheter.segmentLength;
                catheter.setControlTarget(
                    catheterTip,
                    catheterTipX,
                    benchmarkCenterlineY(catheterTipX),
                    Math.sin(phase * 0.7) * 0.05,
                    4e-5
                );
            } else {
                catheter.clearControlTarget(catheterTip);
            }
        }
        previousWireTip = wireTip;
        previousCatheterTip = catheterTip;
        const start = performance.now();
        world.stepFixed();
        stepTimes.push(performance.now() - start);
        maximumPenetration = Math.max(maximumPenetration, world.settledMaxPenetration);
        maximumTransientPenetration = Math.max(maximumTransientPenetration, world.maxPenetration);
        maximumContacts = Math.max(maximumContacts, world.contactCount);
        const wireLengthErrorPercent = maxRelativeSegmentError(wire) * 100;
        const catheterLengthErrorPercent = maxRelativeSegmentError(catheter) * 100;
        if (wireLengthErrorPercent > maximumLengthErrorPercent) {
            maximumLengthErrorPercent = wireLengthErrorPercent;
            maximumLengthErrorStep = step;
            maximumLengthErrorBody = wire.id;
            maximumLengthErrorSegment = maxRelativeSegmentErrorIndex(wire);
            maximumLengthErrorDetails = segmentErrorDetails(wire, maximumLengthErrorSegment);
        }
        if (catheterLengthErrorPercent > maximumLengthErrorPercent) {
            maximumLengthErrorPercent = catheterLengthErrorPercent;
            maximumLengthErrorStep = step;
            maximumLengthErrorBody = catheter.id;
            maximumLengthErrorSegment = maxRelativeSegmentErrorIndex(catheter);
            maximumLengthErrorDetails = segmentErrorDetails(catheter, maximumLengthErrorSegment);
        }
        maximumBendAngleDegrees = Math.max(
            maximumBendAngleDegrees,
            maxBodyBendAngleDegrees(wire),
            maxBodyBendAngleDegrees(catheter)
        );
        finiteThroughout = finiteThroughout && bodyIsFinite(wire) && bodyIsFinite(catheter);
    }
    const heapEnd = heapUsed();
    const stats = world.getStats();
    const result = {
        mode: 'xpbd-contact-v1',
        scenarios: [
            'full-insert',
            'small-branch',
            'stenosis',
            'taper',
            'wire-inside-catheter',
            'external-wire-catheter',
            'pigtail-deploy-rotate',
            'berenstein-deploy-rotate',
            'full-withdraw',
            'sheath'
        ],
        steps: stepTimes.length,
        physicsMs: summarize(stepTimes),
        narrowPhaseMs: stats.phases.narrowPhase,
        constraintMs: stats.phases.constraints,
        contactQueries: field.queryCount,
        maxContactsPerStep: maximumContacts,
        maxPenetrationMm: maximumPenetration,
        maxTransientPenetrationMm: maximumTransientPenetration,
        settledPenetrationMm: stats.settledMaxPenetration,
        maxSegmentErrorMm: Math.max(maxSegmentError(wire), maxSegmentError(catheter)),
        maxSegmentErrorPercent: maximumLengthErrorPercent,
        maxSegmentErrorAt: {
            step: maximumLengthErrorStep,
            phase: xpbdPhase(maximumLengthErrorStep),
            body: maximumLengthErrorBody,
            segment: maximumLengthErrorSegment,
            details: maximumLengthErrorDetails
        },
        maxBendAngleDegrees: maximumBendAngleDegrees,
        bodyStats: stats.bodies,
        finite: finiteThroughout && stats.bodies.every(body => body.finite),
        heapDeltaBytes: heapEnd - heapStart
    };
    const checks = {
        physicsAverage: result.physicsMs.average <= 4,
        physicsP95: result.physicsMs.p95 <= 6,
        postStepPenetration: result.maxPenetrationMm <= 0.2,
        settledPenetration: result.settledPenetrationMm <= 0.05,
        segmentLength: result.maxSegmentErrorPercent <= 1,
        noSharpFold: result.maxBendAngleDegrees < 150,
        finite: result.finite
    };
    result.acceptance = disableToolContact || disableContainment
        ? { skipped: true, passed: null, checks }
        : { skipped: false, passed: Object.values(checks).every(Boolean), checks };
    return result;
}

function markdown(report) {
    const lines = [
        '# Collision benchmark',
        '',
        `Generated: ${report.generatedAt}`,
        '',
        `Host: ${report.environment.model}, ${report.environment.cpus} logical CPUs, ${report.environment.memoryGb.toFixed(1)} GB RAM`,
        '',
        '| Mode | Steps | Physics avg | Physics p95 | Narrow avg | Narrow p95 | Max penetration | Length error | Heap delta |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |'
    ];
    for (const result of report.results) {
        const narrowAverage = result.narrowPhaseMs.average ?? result.narrowPhaseMs.averageMs ?? 0;
        const narrowP95 = result.narrowPhaseMs.p95 ?? result.narrowPhaseMs.p95Ms ?? 0;
        lines.push(
            `| ${result.mode} | ${result.steps} | ${result.physicsMs.average.toFixed(3)} ms | ` +
            `${result.physicsMs.p95.toFixed(3)} ms | ${narrowAverage.toFixed(3)} ms | ` +
            `${narrowP95.toFixed(3)} ms | ${result.maxPenetrationMm.toFixed(3)} mm | ` +
            `${result.maxSegmentErrorPercent.toFixed(3)}% | ${(result.heapDeltaBytes / 1048576).toFixed(2)} MB |`
        );
    }
    lines.push('', 'Scenarios:');
    for (const result of report.results) lines.push(`- ${result.mode}: ${result.scenarios.join(', ')}`);
    for (const result of report.results) {
        if (!result.acceptance) continue;
        lines.push(
            `- ${result.mode} acceptance: ${result.acceptance.skipped ? 'SKIPPED (diagnostic variant)' : result.acceptance.passed ? 'PASS' : 'FAIL'}`
        );
    }
    lines.push(
        '',
        '> Node timings are deterministic engineering comparisons, not the final Chrome/Safari M3 acceptance run.',
        ''
    );
    return lines.join('\n');
}

const results = [];
if (requestedMode === 'all' || requestedMode === 'legacy') results.push(benchmarkLegacy());
if (requestedMode === 'all' || requestedMode === 'xpbd') results.push(benchmarkXpbd());
const report = {
    schemaVersion: 1,
    generatedAt: new Date().toISOString(),
    environment: {
        platform: `${os.platform()} ${os.release()} ${os.arch()}`,
        model: os.cpus()[0]?.model || 'unknown',
        cpus: os.cpus().length,
        memoryGb: os.totalmem() / 1073741824,
        node: process.version
    },
    results
};
fs.mkdirSync(outputDirectory, { recursive: true });
fs.writeFileSync(path.join(outputDirectory, 'collision-benchmark.json'), `${JSON.stringify(report, null, 2)}\n`);
fs.writeFileSync(path.join(outputDirectory, 'collision-benchmark.md'), markdown(report));
console.log(markdown(report));
const failedAcceptance = results.find(result => result.acceptance && !result.acceptance.skipped && !result.acceptance.passed);
if (failedAcceptance) {
    const failedChecks = Object.entries(failedAcceptance.acceptance.checks)
        .filter(([, passed]) => !passed)
        .map(([name]) => name)
        .join(', ');
    console.error(`XPBD acceptance failed: ${failedChecks}`);
    process.exitCode = 1;
}
