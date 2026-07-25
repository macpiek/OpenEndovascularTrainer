import { createContactResult } from './collision/vesselContactField.js';
import {
    GUIDEWIRE_RADIUS_MM,
    INTRODUCER_SHEATH_INNER_DIAMETER_MM,
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_DIAMETER_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM
} from '../toolDimensions.js';

const EPSILON = 1e-8;
const DEFAULT_FIXED_DT = 1 / 120;
const CONTACT_SIGNED_GAP = 1;
const CONTACT_PENETRATION = 3;
const CONTACT_BRANCH_ID = 4;
const CONTACT_SEGMENT_T = 5;
const MAX_WALL_CORRECTION_PASSES = 16;
const WALL_SETTLING_CLEARANCE = 0.01;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function magnitude3(x, y, z) {
    return Math.sqrt(x * x + y * y + z * z);
}

function now() {
    return globalThis.performance?.now?.() ?? Date.now();
}

function percentile(values, count, fraction) {
    if (!count) return 0;
    const copy = Array.from(values.subarray(0, count));
    copy.sort((a, b) => a - b);
    return copy[Math.min(copy.length - 1, Math.floor((copy.length - 1) * fraction))];
}

function createPhaseTimings(capacity = 512) {
    return {
        samples: new Float32Array(capacity),
        cursor: 0,
        count: 0,
        recordedCount: 0,
        total: 0,
        last: 0
    };
}

function recordTiming(timing, duration) {
    timing.last = duration;
    timing.total += duration;
    timing.recordedCount++;
    timing.samples[timing.cursor] = duration;
    timing.cursor = (timing.cursor + 1) % timing.samples.length;
    timing.count = Math.min(timing.samples.length, timing.count + 1);
}

function timingStats(timing) {
    return {
        lastMs: timing.last,
        averageMs: timing.recordedCount ? timing.total / timing.recordedCount : 0,
        p95Ms: percentile(timing.samples, timing.count, 0.95)
    };
}

export const DEFAULT_TOOL_PROFILES = Object.freeze({
    guidewire: Object.freeze({
        id: 'guidewire',
        radius: GUIDEWIRE_RADIUS_MM,
        mass: 1,
        stretchCompliance: 2e-7,
        bendCompliance: 2e-5,
        minBendComplianceScale: 0.001953125,
        maxBendAngle: 135,
        foldLimitStrength: 1,
        wallFriction: 0.006,
        linearDamping: 0.98,
        bendDamping: 0.06
    }),
    catheter: Object.freeze({
        id: 'catheter',
        outerRadius: PIGTAIL_CATHETER_RADIUS_MM,
        innerDiameter: PIGTAIL_CATHETER_INNER_DIAMETER_MM,
        innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        radius: PIGTAIL_CATHETER_RADIUS_MM,
        mass: 1.4,
        stretchCompliance: 1e-7,
        bendCompliance: 1e-5,
        shapeCompliance: 1e-4,
        maxBendAngle: 35,
        foldLimitStrength: 1,
        wallFriction: 0.06,
        lumenFriction: 0.04,
        linearDamping: 0.9,
        bendDamping: 0.68,
        maxSpeed: 40,
        postStabilizationPasses: 4
    }),
    sheath: Object.freeze({
        id: 'sheath',
        outerRadius: 1,
        innerDiameter: INTRODUCER_SHEATH_INNER_DIAMETER_MM,
        innerRadius: INTRODUCER_SHEATH_INNER_RADIUS_MM
    })
});

export class EndovascularRodBody {
    constructor(id, count, segmentLength, profile = {}) {
        if (!Number.isInteger(count) || count < 2) throw new RangeError('A rod requires at least two nodes');
        this.id = id;
        this.count = count;
        this.segmentCount = count - 1;
        this.segmentLength = segmentLength;
        this.radius = profile.radius ?? 0.5;
        this.innerRadius = profile.innerRadius ?? 0;
        this.mass = profile.mass ?? 1;
        this.stretchCompliance = profile.stretchCompliance ?? 2e-7;
        this.bendCompliance = profile.bendCompliance ?? 1e-3;
        this.minBendComplianceScale = profile.minBendComplianceScale ?? 0.125;
        this.shapeCompliance = profile.shapeCompliance ?? 5e-5;
        this.maxBendAngle = profile.maxBendAngle ?? 135;
        this.foldLimitStrength = profile.foldLimitStrength ?? 0.7;
        this.wallCompliance = profile.wallCompliance ?? 0;
        this.wallFriction = profile.wallFriction ?? 0.08;
        this.lumenFriction = profile.lumenFriction ?? 0.04;
        this.linearDamping = profile.linearDamping ?? 0.98;
        this.bendDamping = clamp(profile.bendDamping ?? 0, 0, 1);
        this.maxSpeed = profile.maxSpeed ?? Infinity;
        this.postStabilizationPasses = Math.max(
            0,
            Math.floor(profile.postStabilizationPasses ?? 0)
        );
        this.sleepVelocity = profile.sleepVelocity ?? 0.015;
        this.sleepFrames = profile.sleepFrames ?? 120;
        this.activeStart = 0;
        this.activeEnd = count - 1;
        this.collisionStartSegment = 0;
        this.collisionEndSegment = count - 2;
        this.sleepCounter = 0;
        this.sleeping = false;

        this.x = new Float32Array(count);
        this.y = new Float32Array(count);
        this.z = new Float32Array(count);
        this.previousX = new Float32Array(count);
        this.previousY = new Float32Array(count);
        this.previousZ = new Float32Array(count);
        this.velocityX = new Float32Array(count);
        this.velocityY = new Float32Array(count);
        this.velocityZ = new Float32Array(count);
        this.forceX = new Float32Array(count);
        this.forceY = new Float32Array(count);
        this.forceZ = new Float32Array(count);
        this.inverseMass = new Float32Array(count);
        this.nodeRadius = new Float32Array(count);
        this.pinned = new Uint8Array(count);
        this.controlEnabled = new Uint8Array(count);
        this.controlX = new Float32Array(count);
        this.controlY = new Float32Array(count);
        this.controlZ = new Float32Array(count);
        this.controlCompliance = new Float32Array(count);
        this.restShapeEnabled = new Uint8Array(count);
        this.restShapeX = new Float32Array(count);
        this.restShapeY = new Float32Array(count);
        this.restShapeZ = new Float32Array(count);
        this.restShapeCompliance = new Float32Array(count);
        this.restLength = new Float32Array(this.segmentCount);
        this.restBendChord = new Float32Array(count);
        this.lengthLambda = new Float32Array(this.segmentCount);
        this.lengthNormalX = new Float32Array(this.segmentCount);
        this.lengthNormalY = new Float32Array(this.segmentCount);
        this.lengthNormalZ = new Float32Array(this.segmentCount);
        this.lengthLower = new Float32Array(this.segmentCount);
        this.lengthUpper = new Float32Array(this.segmentCount);
        this.lengthRhs = new Float32Array(this.segmentCount);
        this.lengthSolution = new Float32Array(this.segmentCount);
        this.bendLambda = new Float32Array(count);
        this.bendComplianceByNode = new Float32Array(count);
        this.maxBendAngleByNode = new Float32Array(count);
        this.controlLambda = new Float32Array(count);
        this.shapeLambda = new Float32Array(count);
        this.wallLambda = new Float32Array(this.segmentCount);
        this.wallActive = new Uint8Array(this.segmentCount);
        this.wallT = new Float32Array(this.segmentCount);
        this.wallX = new Float32Array(this.segmentCount);
        this.wallY = new Float32Array(this.segmentCount);
        this.wallZ = new Float32Array(this.segmentCount);
        this.wallNormalX = new Float32Array(this.segmentCount);
        this.wallNormalY = new Float32Array(this.segmentCount);
        this.wallNormalZ = new Float32Array(this.segmentCount);
        this.wallBranchId = new Int32Array(this.segmentCount);
        this.wallGap = new Float32Array(this.segmentCount);
        this.wallQueryStartX = new Float32Array(this.segmentCount);
        this.wallQueryStartY = new Float32Array(this.segmentCount);
        this.wallQueryStartZ = new Float32Array(this.segmentCount);
        this.wallQueryEndX = new Float32Array(this.segmentCount);
        this.wallQueryEndY = new Float32Array(this.segmentCount);
        this.wallQueryEndZ = new Float32Array(this.segmentCount);
        this.wallCorrectionX = new Float32Array(count);
        this.wallCorrectionY = new Float32Array(count);
        this.wallCorrectionZ = new Float32Array(count);
        this.wallCorrectionWeight = new Float32Array(count);
        this.wallBranchId.fill(-1);
        this.wallGap.fill(Infinity);
        this.nodeRadius.fill(this.radius);
        this.inverseMass.fill(1 / Math.max(EPSILON, this.mass));
        this.restLength.fill(segmentLength);
        this.bendComplianceByNode.fill(this.bendCompliance);
        this.maxBendAngleByNode.fill(this.maxBendAngle);
        for (let index = 0; index < count; index++) this.x[index] = index * segmentLength;
        this.captureRestConfiguration();
        this.copyCurrentToPrevious();
    }

    setNodePosition(index, x, y, z, resetVelocity = true) {
        this.x[index] = x;
        this.y[index] = y;
        this.z[index] = z;
        this.previousX[index] = x;
        this.previousY[index] = y;
        this.previousZ[index] = z;
        if (resetVelocity) {
            this.velocityX[index] = 0;
            this.velocityY[index] = 0;
            this.velocityZ[index] = 0;
        }
        this.wake();
        return this;
    }

    setPinned(index, pinned = true) {
        this.pinned[index] = pinned ? 1 : 0;
        this.inverseMass[index] = pinned ? 0 : 1 / Math.max(EPSILON, this.mass);
        this.wake();
        return this;
    }

    setActiveRange(start, end) {
        const nextStart = clamp(Math.floor(start), 0, this.count - 1);
        const nextEnd = clamp(Math.ceil(end), nextStart, this.count - 1);
        if (nextStart < this.activeStart) {
            for (let index = nextStart; index < this.activeStart; index++) {
                this.previousX[index] = this.x[index];
                this.previousY[index] = this.y[index];
                this.previousZ[index] = this.z[index];
                this.velocityX[index] = 0;
                this.velocityY[index] = 0;
                this.velocityZ[index] = 0;
            }
        }
        if (nextEnd > this.activeEnd) {
            for (let index = this.activeEnd + 1; index <= nextEnd; index++) {
                this.previousX[index] = this.x[index];
                this.previousY[index] = this.y[index];
                this.previousZ[index] = this.z[index];
                this.velocityX[index] = 0;
                this.velocityY[index] = 0;
                this.velocityZ[index] = 0;
            }
        }
        if (nextStart !== this.activeStart || nextEnd !== this.activeEnd) this.wake();
        this.activeStart = nextStart;
        this.activeEnd = nextEnd;
        return this;
    }

    setCollisionRange(startSegment, endSegment) {
        const requestedStart = Math.floor(startSegment);
        const requestedEnd = Math.floor(endSegment);
        const nextStart = clamp(requestedStart, 0, this.segmentCount - 1);
        let nextEnd;
        if (requestedEnd < requestedStart || requestedStart >= this.segmentCount || requestedEnd < 0) {
            nextEnd = nextStart - 1;
        } else {
            nextEnd = clamp(requestedEnd, nextStart, this.segmentCount - 1);
        }
        if (nextStart !== this.collisionStartSegment || nextEnd !== this.collisionEndSegment) this.wake();
        this.collisionStartSegment = nextStart;
        this.collisionEndSegment = nextEnd;
        return this;
    }

    setControlTarget(index, x, y, z, compliance = 0) {
        const nextCompliance = Math.max(0, compliance);
        const changed = !this.controlEnabled[index] ||
            Math.abs(this.controlX[index] - x) > 1e-6 ||
            Math.abs(this.controlY[index] - y) > 1e-6 ||
            Math.abs(this.controlZ[index] - z) > 1e-6 ||
            Math.abs(this.controlCompliance[index] - nextCompliance) > 1e-10;
        this.controlEnabled[index] = 1;
        this.controlX[index] = x;
        this.controlY[index] = y;
        this.controlZ[index] = z;
        this.controlCompliance[index] = nextCompliance;
        if (changed) {
            this.controlLambda[index] = 0;
            this.wake();
        }
        return this;
    }

    clearControlTarget(index) {
        if (this.controlEnabled[index]) this.wake();
        this.controlEnabled[index] = 0;
        this.controlLambda[index] = 0;
        return this;
    }

    setRestShapeTarget(index, x, y, z, compliance = this.shapeCompliance) {
        const nextCompliance = Math.max(0, compliance);
        const changed = !this.restShapeEnabled[index] ||
            Math.abs(this.restShapeX[index] - x) > 1e-6 ||
            Math.abs(this.restShapeY[index] - y) > 1e-6 ||
            Math.abs(this.restShapeZ[index] - z) > 1e-6 ||
            Math.abs(this.restShapeCompliance[index] - nextCompliance) > 1e-10;
        this.restShapeEnabled[index] = 1;
        this.restShapeX[index] = x;
        this.restShapeY[index] = y;
        this.restShapeZ[index] = z;
        this.restShapeCompliance[index] = nextCompliance;
        if (changed) {
            this.shapeLambda[index] = 0;
            this.wake();
        }
        return this;
    }

    clearRestShapeTarget(index) {
        if (this.restShapeEnabled[index]) this.wake();
        this.restShapeEnabled[index] = 0;
        this.shapeLambda[index] = 0;
        return this;
    }

    captureRestConfiguration() {
        for (let index = 0; index < this.segmentCount; index++) {
            this.restLength[index] = magnitude3(
                this.x[index + 1] - this.x[index],
                this.y[index + 1] - this.y[index],
                this.z[index + 1] - this.z[index]
            ) || this.segmentLength;
        }
        for (let index = 1; index < this.count - 1; index++) {
            this.restBendChord[index] = magnitude3(
                this.x[index + 1] - this.x[index - 1],
                this.y[index + 1] - this.y[index - 1],
                this.z[index + 1] - this.z[index - 1]
            );
        }
        this.lengthLambda.fill(0);
        this.bendLambda.fill(0);
        return this;
    }

    copyCurrentToPrevious() {
        this.previousX.set(this.x);
        this.previousY.set(this.y);
        this.previousZ.set(this.z);
    }

    wake() {
        this.sleeping = false;
        this.sleepCounter = 0;
    }

    syncFromElasticRod(rod, { resetVelocity = false, preservePrevious = false } = {}) {
        const storage = rod.nodeStorage;
        const count = Math.min(this.count, rod.nodes.length);
        let changed = false;
        for (let index = 0; index < count; index++) {
            changed = changed ||
                Math.abs(this.x[index] - storage.x[index]) > 1e-6 ||
                Math.abs(this.y[index] - storage.y[index]) > 1e-6 ||
                Math.abs(this.z[index] - storage.z[index]) > 1e-6 ||
                Math.abs(this.velocityX[index] - storage.vx[index]) > 1e-5 ||
                Math.abs(this.velocityY[index] - storage.vy[index]) > 1e-5 ||
                Math.abs(this.velocityZ[index] - storage.vz[index]) > 1e-5;
            if (preservePrevious) {
                this.previousX[index] = this.x[index];
                this.previousY[index] = this.y[index];
                this.previousZ[index] = this.z[index];
            }
            this.x[index] = storage.x[index];
            this.y[index] = storage.y[index];
            this.z[index] = storage.z[index];
            this.velocityX[index] = resetVelocity ? 0 : storage.vx[index];
            this.velocityY[index] = resetVelocity ? 0 : storage.vy[index];
            this.velocityZ[index] = resetVelocity ? 0 : storage.vz[index];
            this.inverseMass[index] = storage.pinned[index] ? 0 : 1 / Math.max(EPSILON, storage.mass[index]);
            this.pinned[index] = storage.pinned[index];
            this.bendComplianceByNode[index] = clamp(
                this.bendCompliance * 32 / Math.max(0.1, storage.bendingStiffness[index]),
                this.bendCompliance * this.minBendComplianceScale,
                this.bendCompliance * 8
            );
            this.maxBendAngleByNode[index] = clamp(
                storage.bendAngleLimit?.[index] ?? this.maxBendAngle,
                1,
                179
            );
        }
        if (!preservePrevious) this.copyCurrentToPrevious();
        if (changed) this.wake();
        return this;
    }

    syncToElasticRod(rod) {
        const storage = rod.nodeStorage;
        const count = Math.min(this.count, rod.nodes.length);
        for (let index = 0; index < count; index++) {
            storage.x[index] = this.x[index];
            storage.y[index] = this.y[index];
            storage.z[index] = this.z[index];
            storage.vx[index] = this.velocityX[index];
            storage.vy[index] = this.velocityY[index];
            storage.vz[index] = this.velocityZ[index];
        }
        return this;
    }
}

export class EndovascularPhysicsWorld {
    constructor({
        contactField = null,
        fixedDt = DEFAULT_FIXED_DT,
        maxSubsteps = 2,
        iterations = 6,
        penetrationIterations = 8,
        highPenetration = 0.15,
        contactActivation = 0.25
    } = {}) {
        this.contactField = contactField;
        this.fixedDt = fixedDt;
        this.maxSubsteps = maxSubsteps;
        this.iterations = iterations;
        this.penetrationIterations = penetrationIterations;
        this.highPenetration = highPenetration;
        this.contactActivation = contactActivation;
        this.accumulator = 0;
        this.bodies = [];
        this.sheaths = [];
        this.containments = [];
        this.toolContacts = [];
        this.stepCount = 0;
        this.contactCount = 0;
        this.maxPenetration = 0;
        this.settledMaxPenetration = 0;
        this.settledContactBodyId = null;
        this.settledContactSegment = -1;
        this.settledContactT = 0;
        this.settledContactX = 0;
        this.settledContactY = 0;
        this.settledContactZ = 0;
        this.lastSubsteps = 0;
        this.droppedTime = 0;
        this._queryStart = { x: 0, y: 0, z: 0 };
        this._queryEnd = { x: 0, y: 0, z: 0 };
        this._segmentParameters = { s: 0, t: 0 };
        this._contact = createContactResult();
        this._sweep = createContactResult();
        this.timings = {
            total: createPhaseTimings(),
            integrate: createPhaseTimings(),
            narrowPhase: createPhaseTimings(),
            constraints: createPhaseTimings(),
            velocity: createPhaseTimings()
        };
    }

    createRod(id, count, segmentLength, profile = {}) {
        const body = new EndovascularRodBody(id, count, segmentLength, profile);
        body.contactField = this.contactField;
        this.bodies.push(body);
        return body;
    }

    addRod(body) {
        if (!(body instanceof EndovascularRodBody)) throw new TypeError('EndovascularRodBody is required');
        if (!this.bodies.includes(body)) this.bodies.push(body);
        return body;
    }

    addSheath({ id = 'sheath', start, end, innerRadius = DEFAULT_TOOL_PROFILES.sheath.innerRadius, bodies = null } = {}) {
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const dz = end.z - start.z;
        const length = magnitude3(dx, dy, dz);
        if (length < EPSILON) throw new RangeError('Sheath axis must have positive length');
        const constraint = {
            id,
            startX: start.x,
            startY: start.y,
            startZ: start.z,
            axisX: dx / length,
            axisY: dy / length,
            axisZ: dz / length,
            length,
            innerRadius,
            bodies,
            lambdas: new Map()
        };
        this.sheaths.push(constraint);
        return constraint;
    }

    addContainment(innerBody, outerBody, {
        innerRadius = outerBody.innerRadius,
        compliance = 0,
        friction = outerBody.lumenFriction,
        enabled = true,
        openProximal = true,
        openDistal = true,
        searchWindow = 10,
        outerStartNode = outerBody.activeStart,
        startNode = innerBody.activeStart,
        endNode = innerBody.activeEnd,
        innerResponse = 1,
        outerResponse = 1,
        finalProjection = 'inner',
        outerFollowsInnerCenterline = false,
        innerArcOffset = 0,
        containedLength = Infinity
    } = {}) {
        const constraint = {
            innerBody,
            outerBody,
            innerRadius,
            compliance,
            friction,
            enabled,
            openProximal,
            openDistal,
            searchWindow,
            outerStartNode,
            startNode,
            endNode,
            innerResponse: clamp(innerResponse, 0, 1),
            outerResponse: clamp(outerResponse, 0, 1),
            finalProjection,
            outerFollowsInnerCenterline,
            innerArcOffset,
            containedLength,
            lambdas: new Float32Array(innerBody.count),
            closestSegment: new Int32Array(innerBody.count),
            _lastEnabled: enabled,
            _lastOuterStartNode: outerStartNode,
            _lastStartNode: startNode,
            _lastEndNode: endNode,
            _lastInnerActiveStart: innerBody.activeStart,
            _lastInnerActiveEnd: innerBody.activeEnd,
            _lastOuterActiveStart: outerBody.activeStart,
            _lastOuterActiveEnd: outerBody.activeEnd
        };
        constraint.closestSegment.fill(-1);
        this.containments.push(constraint);
        return constraint;
    }

    addToolContact(bodyA, bodyB, {
        compliance = 0,
        friction = 0.06,
        enabled = true,
        openDistalB = false,
        startSegmentA = 0,
        endSegmentA = bodyA.segmentCount - 1,
        startSegmentB = 0,
        endSegmentB = bodyB.segmentCount - 1
    } = {}) {
        const pairCount = bodyA.segmentCount * bodyB.segmentCount;
        const constraint = {
            bodyA,
            bodyB,
            compliance,
            friction,
            enabled,
            openDistalB,
            startSegmentA,
            endSegmentA,
            startSegmentB,
            endSegmentB,
            lambdas: new Float32Array(pairCount),
            _lastEnabled: enabled,
            _lastStartSegmentA: startSegmentA,
            _lastEndSegmentA: endSegmentA,
            _lastStartSegmentB: startSegmentB,
            _lastEndSegmentB: endSegmentB
        };
        this.toolContacts.push(constraint);
        return constraint;
    }

    advance(frameDt, beforeSubstep = null) {
        const elapsed = Math.max(0, Math.min(0.25, frameDt));
        this.accumulator += elapsed;
        let substeps = 0;
        while (this.accumulator + EPSILON >= this.fixedDt && substeps < this.maxSubsteps) {
            beforeSubstep?.(this.fixedDt, substeps);
            this.stepFixed();
            this.accumulator -= this.fixedDt;
            substeps++;
        }
        if (this.accumulator >= this.fixedDt) {
            this.droppedTime += this.accumulator - (this.accumulator % this.fixedDt);
            this.accumulator %= this.fixedDt;
        }
        this.lastSubsteps = substeps;
        return substeps;
    }

    stepFixed() {
        const totalStart = now();
        this.contactCount = 0;
        this.maxPenetration = 0;

        let phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.contactField = this.contactField;
            body.lengthLambda.fill(0);
            body.bendLambda.fill(0);
            body.controlLambda.fill(0);
            body.shapeLambda.fill(0);
            this.#integrate(body);
        }
        recordTiming(this.timings.integrate, now() - phaseStart);

        phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) this.#applySweptCollision(this.bodies[index]);
        for (let index = 0; index < this.bodies.length; index++) this.#prepareWallContacts(this.bodies[index]);
        let narrowPhaseDuration = now() - phaseStart;

        phaseStart = now();
        const iterationCount = this.maxPenetration > this.highPenetration
            ? this.penetrationIterations
            : this.iterations;
        for (let iteration = 0; iteration < iterationCount; iteration++) {
            for (let index = 0; index < this.sheaths.length; index++) this.#solveSheath(this.sheaths[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveControls(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveLengths(this.bodies[index], (iteration & 1) === 1);
            }
            for (let index = 0; index < this.bodies.length; index++) this.#solveBending(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveRestShape(this.bodies[index]);
            // Shape memory is deliberately solved after the first control
            // projection, but an unsupported catheter tip must not receive the
            // entire shape correction as a single-frame impulse. Rebalance the
            // compliant controls before the wall gets the final say.
            for (let index = 0; index < this.bodies.length; index++) this.#solveControls(this.bodies[index]);
            for (let index = 0; index < this.containments.length; index++) this.#solveContainment(this.containments[index]);
            for (let index = 0; index < this.toolContacts.length; index++) this.#solveToolContact(this.toolContacts[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveWallContacts(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveFoldLimits(this.bodies[index]);
        }
        // Later bend, shape and contact projections can perturb segment lengths.
        // Finish the substep with inexpensive structural polishing so callers
        // never observe a transiently stretched rod between fixed steps.
        for (let pass = 0; pass < 8; pass++) {
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveWallContacts(this.bodies[index]);
            }
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveFoldLimits(this.bodies[index]);
            }
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveLengthsGlobal(this.bodies[index]);
            }
            let lengthsSettled = true;
            for (let index = 0; index < this.bodies.length; index++) {
                lengthsSettled = lengthsSettled && !this.#hasLengthErrorOver(this.bodies[index], 0.002);
            }
            if (lengthsSettled) break;
        }
        for (
            let correctionPass = 0;
            correctionPass < MAX_WALL_CORRECTION_PASSES;
            correctionPass++
        ) {
            let activePenetration = 0;
            for (let index = 0; index < this.bodies.length; index++) {
                activePenetration = Math.max(
                    activePenetration,
                    this.#refreshActiveWallContacts(this.bodies[index])
                );
            }
            if (activePenetration <= 0.02) break;
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveFoldLimits(this.bodies[index]);
                this.#solveDistributedWallContacts(this.bodies[index]);
                if (correctionPass + 1 < MAX_WALL_CORRECTION_PASSES) {
                    this.#solveLengthsGlobal(this.bodies[index]);
                }
            }
        }
        // Later wall and fold corrections can separate the two centerlines.
        // Finish with exactly one radial projection of the body selected by
        // the material coupling. Repeating structural projections here caused
        // the catheter to collapse at its open distal transition.
        const finalContainmentPasses = this.containments.some(constraint =>
            constraint.enabled &&
            constraint.finalProjection !== 'none' &&
            !constraint.outerFollowsInnerCenterline
        ) ? 2 : 1;
        for (let pass = 0; pass < finalContainmentPasses; pass++) {
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                if (!constraint.enabled || constraint.finalProjection === 'none') continue;
                this.#solveContainment(constraint, {
                    innerOnly: constraint.finalProjection !== 'outer',
                    outerOnly: constraint.finalProjection === 'outer',
                    applyFriction: false
                });
            }
        }
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            for (let pass = 0; pass < body.postStabilizationPasses; pass++) {
                this.#solveControls(body);
                this.#solveFoldLimits(body);
                this.#solveLengthsGlobal(body);
                this.#prepareWallContacts(body);
                this.#solveWallContacts(body);
            }
            this.#solveFoldLimits(body);
        }
        recordTiming(this.timings.constraints, now() - phaseStart);

        const transientMaxPenetration = this.maxPenetration;
        phaseStart = now();
        this.contactCount = 0;
        this.maxPenetration = 0;
        this.settledContactBodyId = null;
        this.settledContactSegment = -1;
        for (let index = 0; index < this.bodies.length; index++) this.#refreshActiveWallContacts(this.bodies[index]);
        this.settledMaxPenetration = this.maxPenetration;
        this.maxPenetration = Math.max(transientMaxPenetration, this.settledMaxPenetration);
        narrowPhaseDuration += now() - phaseStart;
        recordTiming(this.timings.narrowPhase, narrowPhaseDuration);

        phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) this.#updateVelocityAndFriction(this.bodies[index]);
        for (let index = 0; index < this.bodies.length; index++) {
            this.#stabilizeBendingVelocity(this.bodies[index]);
        }
        for (let index = 0; index < this.containments.length; index++) {
            this.#stabilizeContainmentVelocity(this.containments[index]);
        }
        for (let index = 0; index < this.toolContacts.length; index++) {
            this.#stabilizeToolContactVelocity(this.toolContacts[index]);
        }
        for (let index = 0; index < this.bodies.length; index++) {
            this.#limitVelocity(this.bodies[index]);
        }
        recordTiming(this.timings.velocity, now() - phaseStart);

        this.stepCount++;
        recordTiming(this.timings.total, now() - totalStart);
    }

    resetPerformanceStats() {
        this.contactCount = 0;
        this.maxPenetration = 0;
        this.settledMaxPenetration = 0;
        for (const timing of Object.values(this.timings)) {
            timing.samples.fill(0);
            timing.cursor = 0;
            timing.count = 0;
            timing.recordedCount = 0;
            timing.total = 0;
            timing.last = 0;
        }
    }

    resetSimulationState() {
        this.accumulator = 0;
        this.stepCount = 0;
        this.lastSubsteps = 0;
        this.droppedTime = 0;
        for (const body of this.bodies) {
            body.lengthLambda.fill(0);
            body.bendLambda.fill(0);
            body.controlLambda.fill(0);
            body.shapeLambda.fill(0);
            body.wallLambda.fill(0);
            body.wallActive.fill(0);
            body.wallBranchId.fill(-1);
            body.wallGap.fill(Infinity);
            body.copyCurrentToPrevious();
            body.wake();
        }
        for (const sheath of this.sheaths) sheath.lambdas.clear();
        for (const containment of this.containments) {
            containment.lambdas.fill(0);
            containment.closestSegment.fill(-1);
            containment._lastEnabled = containment.enabled;
            containment._lastOuterStartNode = containment.outerStartNode;
            containment._lastStartNode = containment.startNode;
            containment._lastEndNode = containment.endNode;
            containment._lastInnerActiveStart = containment.innerBody.activeStart;
            containment._lastInnerActiveEnd = containment.innerBody.activeEnd;
            containment._lastOuterActiveStart = containment.outerBody.activeStart;
            containment._lastOuterActiveEnd = containment.outerBody.activeEnd;
        }
        for (const contact of this.toolContacts) {
            contact.lambdas.fill(0);
            contact._lastEnabled = contact.enabled;
            contact._lastStartSegmentA = contact.startSegmentA;
            contact._lastEndSegmentA = contact.endSegmentA;
            contact._lastStartSegmentB = contact.startSegmentB;
            contact._lastEndSegmentB = contact.endSegmentB;
        }
        this.resetPerformanceStats();
        return this;
    }

    getStats() {
        const bodies = this.bodies.map(body => this.#bodyStats(body));
        return {
            mode: 'xpbd-contact-v1',
            fixedDt: this.fixedDt,
            steps: this.stepCount,
            lastSubsteps: this.lastSubsteps,
            droppedTime: this.droppedTime,
            contacts: this.contactCount,
            maxPenetration: this.maxPenetration,
            settledMaxPenetration: this.settledMaxPenetration,
            settledContact: {
                bodyId: this.settledContactBodyId,
                segment: this.settledContactSegment,
                t: this.settledContactT,
                x: this.settledContactX,
                y: this.settledContactY,
                z: this.settledContactZ
            },
            phases: {
                total: timingStats(this.timings.total),
                integrate: timingStats(this.timings.integrate),
                narrowPhase: timingStats(this.timings.narrowPhase),
                constraints: timingStats(this.timings.constraints),
                velocity: timingStats(this.timings.velocity)
            },
            bodies
        };
    }

    #integrate(body) {
        if (body.sleeping) return;
        const dt = this.fixedDt;
        const dtSquared = dt * dt;
        const start = body.activeStart;
        const end = body.activeEnd;
        for (let index = start; index <= end; index++) {
            body.previousX[index] = body.x[index];
            body.previousY[index] = body.y[index];
            body.previousZ[index] = body.z[index];
            if (body.inverseMass[index] <= 0) continue;
            body.velocityX[index] *= body.linearDamping;
            body.velocityY[index] *= body.linearDamping;
            body.velocityZ[index] *= body.linearDamping;
            body.x[index] += body.velocityX[index] * dt + body.forceX[index] * body.inverseMass[index] * dtSquared;
            body.y[index] += body.velocityY[index] * dt + body.forceY[index] * body.inverseMass[index] * dtSquared;
            body.z[index] += body.velocityZ[index] * dt + body.forceZ[index] * body.inverseMass[index] * dtSquared;
        }
        body.forceX.fill(0);
        body.forceY.fill(0);
        body.forceZ.fill(0);
    }

    #applySweptCollision(body) {
        if (!this.contactField || body.sleeping || body.collisionEndSegment < body.collisionStartSegment) return;
        const start = Math.max(body.activeStart, body.collisionStartSegment);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1);
        for (let index = start; index <= end; index++) {
            const dx = body.x[index] - body.previousX[index];
            const dy = body.y[index] - body.previousY[index];
            const dz = body.z[index] - body.previousZ[index];
            const radius = body.nodeRadius[index];
            if (dx * dx + dy * dy + dz * dz <= radius * radius * 0.25) continue;
            this._queryStart.x = body.previousX[index];
            this._queryStart.y = body.previousY[index];
            this._queryStart.z = body.previousZ[index];
            this._queryEnd.x = body.x[index];
            this._queryEnd.y = body.y[index];
            this._queryEnd.z = body.z[index];
            const contact = this.contactField.sweepSphere(this._queryStart, this._queryEnd, radius, this._sweep);
            if (!contact.violation || contact.timeOfImpact >= 1) continue;
            const safeT = Math.max(0, contact.timeOfImpact - 1e-3);
            body.x[index] = body.previousX[index] + dx * safeT + contact.inward.x * 1e-3;
            body.y[index] = body.previousY[index] + dy * safeT + contact.inward.y * 1e-3;
            body.z[index] = body.previousZ[index] + dz * safeT + contact.inward.z * 1e-3;
        }
    }

    #prepareWallContacts(body) {
        if (!this.contactField || body.sleeping || body.collisionEndSegment < body.collisionStartSegment) return;
        const start = Math.max(body.activeStart, body.collisionStartSegment, 0);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        for (let index = start; index < end; index++) {
            const wasActive = body.wallActive[index] !== 0;
            body.wallActive[index] = 0;
            if (wasActive) {
                const t = body.wallT[index];
                const px = body.x[index] + (body.x[index + 1] - body.x[index]) * t;
                const py = body.y[index] + (body.y[index + 1] - body.y[index]) * t;
                const pz = body.z[index] + (body.z[index + 1] - body.z[index]) * t;
                const radius = Math.max(body.nodeRadius[index], body.nodeRadius[index + 1]);
                const cachedPlaneGap =
                    (px - body.wallX[index]) * body.wallNormalX[index] +
                    (py - body.wallY[index]) * body.wallNormalY[index] +
                    (pz - body.wallZ[index]) * body.wallNormalZ[index] - radius;
                // The previous step ended with an exact SDF refresh. Reuse its
                // contact plane for the first projection while still inside a
                // small activation halo; the end-of-step refresh below remains
                // authoritative and updates curvature, branch and penetration.
                if (cachedPlaneGap <= this.contactActivation + 0.1) {
                    body.wallActive[index] = 1;
                    if (cachedPlaneGap < 0) {
                        this.contactCount++;
                        this.maxPenetration = Math.max(this.maxPenetration, -cachedPlaneGap);
                    }
                    continue;
                }
            }
            const cachedGap = body.wallGap[index];
            if (!wasActive && Number.isFinite(cachedGap)) {
                const startDx = body.x[index] - body.wallQueryStartX[index];
                const startDy = body.y[index] - body.wallQueryStartY[index];
                const startDz = body.z[index] - body.wallQueryStartZ[index];
                const endDx = body.x[index + 1] - body.wallQueryEndX[index];
                const endDy = body.y[index + 1] - body.wallQueryEndY[index];
                const endDz = body.z[index + 1] - body.wallQueryEndZ[index];
                const startMovement = Math.sqrt(
                    startDx * startDx + startDy * startDy + startDz * startDz
                );
                const endMovement = Math.sqrt(endDx * endDx + endDy * endDy + endDz * endDz);
                // Distance to a closed surface is 1-Lipschitz. A capsule whose
                // cached gap exceeds the largest endpoint displacement cannot
                // have reached the activation band since its last exact query.
                if (cachedGap - Math.max(startMovement, endMovement) > this.contactActivation) {
                    body.wallLambda[index] *= 0.5;
                    continue;
                }
            }
            let contact;
            if (this.contactField.queryCapsuleSoA) {
                contact = this.contactField.queryCapsuleSoA(
                    body.x, body.y, body.z, body.nodeRadius, index, this._contact
                );
            } else {
                const radius = Math.max(body.nodeRadius[index], body.nodeRadius[index + 1]);
                this._queryStart.x = body.x[index];
                this._queryStart.y = body.y[index];
                this._queryStart.z = body.z[index];
                this._queryEnd.x = body.x[index + 1];
                this._queryEnd.y = body.y[index + 1];
                this._queryEnd.z = body.z[index + 1];
                contact = this.contactField.queryCapsule(
                    this._queryStart, this._queryEnd, radius, this._contact
                );
            }
            const contactValues = contact.values;
            const signedGap = contactValues[CONTACT_SIGNED_GAP];
            const segmentT = contactValues[CONTACT_SEGMENT_T];
            const branchId = contactValues[CONTACT_BRANCH_ID];
            const closest = contact.closestPoint.values;
            const inward = contact.inward.values;
            body.wallGap[index] = signedGap;
            body.wallQueryStartX[index] = body.x[index];
            body.wallQueryStartY[index] = body.y[index];
            body.wallQueryStartZ[index] = body.z[index];
            body.wallQueryEndX[index] = body.x[index + 1];
            body.wallQueryEndY[index] = body.y[index + 1];
            body.wallQueryEndZ[index] = body.z[index + 1];
            if (signedGap > this.contactActivation) {
                body.wallLambda[index] *= 0.5;
                continue;
            }
            if (body.wallBranchId[index] !== branchId) body.wallLambda[index] = 0;
            body.wallActive[index] = 1;
            body.wallT[index] = segmentT;
            body.wallX[index] = closest[0];
            body.wallY[index] = closest[1];
            body.wallZ[index] = closest[2];
            body.wallNormalX[index] = inward[0];
            body.wallNormalY[index] = inward[1];
            body.wallNormalZ[index] = inward[2];
            body.wallBranchId[index] = branchId;
            if (contact.violation) {
                this.contactCount++;
                this.maxPenetration = Math.max(
                    this.maxPenetration,
                    contactValues[CONTACT_PENETRATION]
                );
            }
        }
    }

    #refreshActiveWallContacts(body) {
        if (!this.contactField || body.sleeping || body.collisionEndSegment < body.collisionStartSegment) return 0;
        const start = Math.max(body.activeStart, body.collisionStartSegment, 0);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        let maximumPenetration = 0;
        for (let index = start; index < end; index++) {
            if (!body.wallActive[index]) continue;
            const cachedGap = body.wallGap[index];
            if (Number.isFinite(cachedGap)) {
                const startDx = body.x[index] - body.wallQueryStartX[index];
                const startDy = body.y[index] - body.wallQueryStartY[index];
                const startDz = body.z[index] - body.wallQueryStartZ[index];
                const endDx = body.x[index + 1] - body.wallQueryEndX[index];
                const endDy = body.y[index + 1] - body.wallQueryEndY[index];
                const endDz = body.z[index + 1] - body.wallQueryEndZ[index];
                const startMovement = Math.sqrt(
                    startDx * startDx + startDy * startDy + startDz * startDz
                );
                const endMovement = Math.sqrt(endDx * endDx + endDy * endDy + endDz * endDz);
                const conservativeGap = cachedGap - Math.max(startMovement, endMovement);
                if (conservativeGap > 0.02) {
                    if (conservativeGap > this.contactActivation) {
                        body.wallActive[index] = 0;
                        body.wallLambda[index] *= 0.5;
                    }
                    continue;
                }
            }
            let contact;
            if (this.contactField.queryCapsuleSoA) {
                contact = this.contactField.queryCapsuleSoA(
                    body.x, body.y, body.z, body.nodeRadius, index, this._contact
                );
            } else {
                const radius = Math.max(body.nodeRadius[index], body.nodeRadius[index + 1]);
                this._queryStart.x = body.x[index];
                this._queryStart.y = body.y[index];
                this._queryStart.z = body.z[index];
                this._queryEnd.x = body.x[index + 1];
                this._queryEnd.y = body.y[index + 1];
                this._queryEnd.z = body.z[index + 1];
                contact = this.contactField.queryCapsule(
                    this._queryStart, this._queryEnd, radius, this._contact
                );
            }
            const contactValues = contact.values;
            const signedGap = contactValues[CONTACT_SIGNED_GAP];
            const segmentT = contactValues[CONTACT_SEGMENT_T];
            const branchId = contactValues[CONTACT_BRANCH_ID];
            const closest = contact.closestPoint.values;
            const inward = contact.inward.values;
            body.wallGap[index] = signedGap;
            body.wallQueryStartX[index] = body.x[index];
            body.wallQueryStartY[index] = body.y[index];
            body.wallQueryStartZ[index] = body.z[index];
            body.wallQueryEndX[index] = body.x[index + 1];
            body.wallQueryEndY[index] = body.y[index + 1];
            body.wallQueryEndZ[index] = body.z[index + 1];
            if (signedGap > this.contactActivation) {
                body.wallActive[index] = 0;
                body.wallLambda[index] *= 0.5;
                continue;
            }
            if (body.wallBranchId[index] !== branchId) body.wallLambda[index] = 0;
            body.wallT[index] = segmentT;
            body.wallX[index] = closest[0];
            body.wallY[index] = closest[1];
            body.wallZ[index] = closest[2];
            body.wallNormalX[index] = inward[0];
            body.wallNormalY[index] = inward[1];
            body.wallNormalZ[index] = inward[2];
            body.wallBranchId[index] = branchId;
            if (contact.violation) {
                this.contactCount++;
                if (contactValues[CONTACT_PENETRATION] > this.maxPenetration) {
                    this.settledContactBodyId = body.id;
                    this.settledContactSegment = index;
                    this.settledContactT = segmentT;
                    this.settledContactX = body.x[index] +
                        (body.x[index + 1] - body.x[index]) * segmentT;
                    this.settledContactY = body.y[index] +
                        (body.y[index + 1] - body.y[index]) * segmentT;
                    this.settledContactZ = body.z[index] +
                        (body.z[index + 1] - body.z[index]) * segmentT;
                }
                this.maxPenetration = Math.max(
                    this.maxPenetration,
                    contactValues[CONTACT_PENETRATION]
                );
                maximumPenetration = Math.max(
                    maximumPenetration,
                    contactValues[CONTACT_PENETRATION]
                );
            }
        }
        return maximumPenetration;
    }

    #hasLengthErrorOver(body, threshold) {
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        for (let index = start; index < end; index++) {
            const length = magnitude3(
                body.x[index + 1] - body.x[index],
                body.y[index + 1] - body.y[index],
                body.z[index + 1] - body.z[index]
            );
            if (Math.abs(length - body.restLength[index]) > body.restLength[index] * threshold) return true;
        }
        return false;
    }

    #solveControls(body) {
        if (body.sleeping) return;
        const dtSquared = this.fixedDt * this.fixedDt;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            if (!body.controlEnabled[index] || body.inverseMass[index] <= 0) continue;
            const dx = body.x[index] - body.controlX[index];
            const dy = body.y[index] - body.controlY[index];
            const dz = body.z[index] - body.controlZ[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            const alpha = body.controlCompliance[index] / dtSquared;
            const deltaLambda = (-distance - alpha * body.controlLambda[index]) / (body.inverseMass[index] + alpha);
            body.controlLambda[index] += deltaLambda;
            const scale = deltaLambda / distance * body.inverseMass[index];
            body.x[index] += dx * scale;
            body.y[index] += dy * scale;
            body.z[index] += dz * scale;
        }
    }

    #solveLengths(body, reverse = false) {
        if (body.sleeping) return;
        const alpha = body.stretchCompliance / (this.fixedDt * this.fixedDt);
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        for (
            let index = reverse ? end - 1 : start;
            reverse ? index >= start : index < end;
            index += reverse ? -1 : 1
        ) {
            const dx = body.x[index + 1] - body.x[index];
            const dy = body.y[index + 1] - body.y[index];
            const dz = body.z[index + 1] - body.z[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            const w0 = body.inverseMass[index];
            const w1 = body.inverseMass[index + 1];
            const denominator = w0 + w1 + alpha;
            if (denominator < EPSILON) continue;
            const constraint = distance - body.restLength[index];
            const deltaLambda = (-constraint - alpha * body.lengthLambda[index]) / denominator;
            body.lengthLambda[index] += deltaLambda;
            const nx = dx / distance;
            const ny = dy / distance;
            const nz = dz / distance;
            body.x[index] -= nx * deltaLambda * w0;
            body.y[index] -= ny * deltaLambda * w0;
            body.z[index] -= nz * deltaLambda * w0;
            body.x[index + 1] += nx * deltaLambda * w1;
            body.y[index + 1] += ny * deltaLambda * w1;
            body.z[index + 1] += nz * deltaLambda * w1;
        }
    }

    #solveLengthsGlobal(body) {
        if (body.sleeping) return;
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        const count = end - start;
        if (count <= 0) return;

        for (let local = 0; local < count; local++) {
            const segment = start + local;
            const dx = body.x[segment + 1] - body.x[segment];
            const dy = body.y[segment + 1] - body.y[segment];
            const dz = body.z[segment + 1] - body.z[segment];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) {
                body.lengthNormalX[local] = 1;
                body.lengthNormalY[local] = 0;
                body.lengthNormalZ[local] = 0;
                body.lengthRhs[local] = 0;
            } else {
                body.lengthNormalX[local] = dx / distance;
                body.lengthNormalY[local] = dy / distance;
                body.lengthNormalZ[local] = dz / distance;
                body.lengthRhs[local] = -(distance - body.restLength[segment]);
            }
        }

        for (let local = 0; local < count; local++) {
            const segment = start + local;
            let lower = 0;
            let upper = 0;
            if (local > 0) {
                lower = -body.inverseMass[segment] * (
                    body.lengthNormalX[local] * body.lengthNormalX[local - 1] +
                    body.lengthNormalY[local] * body.lengthNormalY[local - 1] +
                    body.lengthNormalZ[local] * body.lengthNormalZ[local - 1]
                );
            }
            if (local + 1 < count) {
                upper = -body.inverseMass[segment + 1] * (
                    body.lengthNormalX[local] * body.lengthNormalX[local + 1] +
                    body.lengthNormalY[local] * body.lengthNormalY[local + 1] +
                    body.lengthNormalZ[local] * body.lengthNormalZ[local + 1]
                );
            }
            body.lengthLower[local] = lower;
            body.lengthUpper[local] = upper;
            body.lengthSolution[local] = body.inverseMass[segment] + body.inverseMass[segment + 1];
        }

        let denominator = Math.max(EPSILON, body.lengthSolution[0]);
        body.lengthUpper[0] /= denominator;
        body.lengthRhs[0] /= denominator;
        for (let local = 1; local < count; local++) {
            denominator = Math.max(
                EPSILON,
                body.lengthSolution[local] - body.lengthLower[local] * body.lengthUpper[local - 1]
            );
            body.lengthUpper[local] = local + 1 < count
                ? body.lengthUpper[local] / denominator
                : 0;
            body.lengthRhs[local] = (
                body.lengthRhs[local] - body.lengthLower[local] * body.lengthRhs[local - 1]
            ) / denominator;
        }
        body.lengthSolution[count - 1] = body.lengthRhs[count - 1];
        for (let local = count - 2; local >= 0; local--) {
            body.lengthSolution[local] = body.lengthRhs[local] -
                body.lengthUpper[local] * body.lengthSolution[local + 1];
        }

        for (let local = 0; local < count; local++) {
            const segment = start + local;
            const lambda = body.lengthSolution[local];
            const nx = body.lengthNormalX[local];
            const ny = body.lengthNormalY[local];
            const nz = body.lengthNormalZ[local];
            body.x[segment] -= nx * lambda * body.inverseMass[segment];
            body.y[segment] -= ny * lambda * body.inverseMass[segment];
            body.z[segment] -= nz * lambda * body.inverseMass[segment];
            body.x[segment + 1] += nx * lambda * body.inverseMass[segment + 1];
            body.y[segment + 1] += ny * lambda * body.inverseMass[segment + 1];
            body.z[segment + 1] += nz * lambda * body.inverseMass[segment + 1];
        }
    }

    #solveBending(body) {
        if (body.sleeping || body.count < 3) return;
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.count - 1, body.activeEnd);
        for (let index = start; index < end; index++) {
            const previous = index - 1;
            const next = index + 1;
            const dx = body.x[next] - body.x[previous];
            const dy = body.y[next] - body.y[previous];
            const dz = body.z[next] - body.z[previous];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            const w0 = body.inverseMass[previous];
            const w1 = body.inverseMass[next];
            const alpha = body.bendComplianceByNode[index] / (this.fixedDt * this.fixedDt);
            const denominator = w0 + w1 + alpha;
            if (denominator < EPSILON) continue;
            const constraint = distance - body.restBendChord[index];
            const deltaLambda = (-constraint - alpha * body.bendLambda[index]) / denominator;
            body.bendLambda[index] += deltaLambda;
            const nx = dx / distance;
            const ny = dy / distance;
            const nz = dz / distance;
            body.x[previous] -= nx * deltaLambda * w0;
            body.y[previous] -= ny * deltaLambda * w0;
            body.z[previous] -= nz * deltaLambda * w0;
            body.x[next] += nx * deltaLambda * w1;
            body.y[next] += ny * deltaLambda * w1;
            body.z[next] += nz * deltaLambda * w1;
        }
    }

    #solveRestShape(body) {
        if (body.sleeping) return;
        const dtSquared = this.fixedDt * this.fixedDt;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            if (!body.restShapeEnabled[index] || body.inverseMass[index] <= 0) continue;
            const dx = body.x[index] - body.restShapeX[index];
            const dy = body.y[index] - body.restShapeY[index];
            const dz = body.z[index] - body.restShapeZ[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            const alpha = body.restShapeCompliance[index] / dtSquared;
            const deltaLambda = (-distance - alpha * body.shapeLambda[index]) / (body.inverseMass[index] + alpha);
            body.shapeLambda[index] += deltaLambda;
            const scale = deltaLambda / distance * body.inverseMass[index];
            body.x[index] += dx * scale;
            body.y[index] += dy * scale;
            body.z[index] += dz * scale;
        }
    }

    #solveFoldLimits(body) {
        if (body.sleeping || body.count < 3 || body.foldLimitStrength <= 0) return;
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.count - 1, body.activeEnd);
        for (let index = start; index < end; index++) {
            const limit = clamp(body.maxBendAngleByNode[index], 1, 179) * Math.PI / 180;
            const minDot = Math.cos(limit);
            const previous = index - 1;
            const next = index + 1;
            const incomingX = body.x[index] - body.x[previous];
            const incomingY = body.y[index] - body.y[previous];
            const incomingZ = body.z[index] - body.z[previous];
            const outgoingX = body.x[next] - body.x[index];
            const outgoingY = body.y[next] - body.y[index];
            const outgoingZ = body.z[next] - body.z[index];
            const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
            const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
            if (incomingLength < EPSILON || outgoingLength < EPSILON) continue;
            const dot = (
                incomingX * outgoingX + incomingY * outgoingY + incomingZ * outgoingZ
            ) / (incomingLength * outgoingLength);
            if (dot >= minDot) continue;
            const wallConstrained = (
                (previous > 0 && body.wallActive[previous - 1]) ||
                body.wallActive[previous] ||
                body.wallActive[index]
            );
            if (wallConstrained && body.inverseMass[index] > 0) {
                // Expanding the neighbouring endpoints can push them back
                // through the vessel wall and make the next contact pass snap
                // the rod in the opposite direction. At a wall, smooth the
                // hinge by moving its centre toward the local chord instead.
                const centerStrength = Math.min(0.72, body.foldLimitStrength * 0.62);
                body.x[index] += (
                    (body.x[previous] + body.x[next]) * 0.5 - body.x[index]
                ) * centerStrength;
                body.y[index] += (
                    (body.y[previous] + body.y[next]) * 0.5 - body.y[index]
                ) * centerStrength;
                body.z[index] += (
                    (body.z[previous] + body.z[next]) * 0.5 - body.z[index]
                ) * centerStrength;
                continue;
            }

            let chordX = body.x[next] - body.x[previous];
            let chordY = body.y[next] - body.y[previous];
            let chordZ = body.z[next] - body.z[previous];
            const chordLength = magnitude3(chordX, chordY, chordZ);
            if (chordLength < EPSILON) {
                chordX = incomingX / incomingLength;
                chordY = incomingY / incomingLength;
                chordZ = incomingZ / incomingLength;
            } else {
                chordX /= chordLength;
                chordY /= chordLength;
                chordZ /= chordLength;
            }
            const targetChord = Math.sqrt(Math.max(0,
                incomingLength * incomingLength + outgoingLength * outgoingLength +
                2 * incomingLength * outgoingLength * minDot
            ));
            const deficit = targetChord - chordLength;
            if (deficit <= 0) continue;
            const previousWeight = body.inverseMass[previous];
            const nextWeight = body.inverseMass[next];
            const totalWeight = previousWeight + nextWeight;
            if (totalWeight < EPSILON) continue;
            const correction = Math.min(
                deficit * body.foldLimitStrength,
                Math.min(incomingLength, outgoingLength) * 0.35
            );
            const previousScale = correction * previousWeight / totalWeight;
            const nextScale = correction * nextWeight / totalWeight;
            body.x[previous] -= chordX * previousScale;
            body.y[previous] -= chordY * previousScale;
            body.z[previous] -= chordZ * previousScale;
            body.x[next] += chordX * nextScale;
            body.y[next] += chordY * nextScale;
            body.z[next] += chordZ * nextScale;
            if (body.inverseMass[index] > 0) {
                const centerStrength = body.foldLimitStrength * 0.45;
                body.x[index] += ((body.x[previous] + body.x[next]) * 0.5 - body.x[index]) * centerStrength;
                body.y[index] += ((body.y[previous] + body.y[next]) * 0.5 - body.y[index]) * centerStrength;
                body.z[index] += ((body.z[previous] + body.z[next]) * 0.5 - body.z[index]) * centerStrength;
                if (dot < -0.999 && chordLength < Math.min(incomingLength, outgoingLength) * 0.1) {
                    const nx = incomingX / incomingLength;
                    const ny = incomingY / incomingLength;
                    const nz = incomingZ / incomingLength;
                    let bendX;
                    let bendY;
                    let bendZ;
                    if (Math.abs(nx) < 0.8) {
                        bendX = 0;
                        bendY = nz;
                        bendZ = -ny;
                    } else {
                        bendX = -nz;
                        bendY = 0;
                        bendZ = nx;
                    }
                    const bendLength = magnitude3(bendX, bendY, bendZ) || 1;
                    const nudge = Math.min(incomingLength, outgoingLength) * body.foldLimitStrength * 0.05;
                    body.x[index] += bendX / bendLength * nudge;
                    body.y[index] += bendY / bendLength * nudge;
                    body.z[index] += bendZ / bendLength * nudge;
                }
            }
        }
    }

    #solveContainment(constraint, {
        innerOnly = false,
        outerOnly = false,
        applyFriction = true
    } = {}) {
        if (
            constraint.enabled !== constraint._lastEnabled ||
            constraint.outerStartNode !== constraint._lastOuterStartNode ||
            constraint.startNode !== constraint._lastStartNode ||
            constraint.endNode !== constraint._lastEndNode ||
            constraint.innerBody.activeStart !== constraint._lastInnerActiveStart ||
            constraint.innerBody.activeEnd !== constraint._lastInnerActiveEnd ||
            constraint.outerBody.activeStart !== constraint._lastOuterActiveStart ||
            constraint.outerBody.activeEnd !== constraint._lastOuterActiveEnd
        ) {
            constraint.lambdas.fill(0);
            constraint.closestSegment.fill(-1);
            constraint._lastEnabled = constraint.enabled;
            constraint._lastOuterStartNode = constraint.outerStartNode;
            constraint._lastStartNode = constraint.startNode;
            constraint._lastInnerActiveStart = constraint.innerBody.activeStart;
            constraint._lastInnerActiveEnd = constraint.innerBody.activeEnd;
            constraint._lastOuterActiveStart = constraint.outerBody.activeStart;
            constraint._lastOuterActiveEnd = constraint.outerBody.activeEnd;
        }
        constraint._lastEndNode = constraint.endNode;
        if (!constraint.enabled) return;
        if (constraint.outerFollowsInnerCenterline) {
            this.#projectOuterAlongInnerCenterline(constraint);
            return;
        }
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const allowedRadius = Math.max(0, constraint.innerRadius - inner.radius);
        const alpha = constraint.compliance / (this.fixedDt * this.fixedDt);
        const outerStart = clamp(constraint.outerStartNode, outer.activeStart, outer.activeEnd);
        const outerEnd = Math.min(outer.activeEnd, outer.segmentCount);
        if (outerEnd <= outerStart) return;

        const innerStart = clamp(constraint.startNode, inner.activeStart, inner.activeEnd);
        const innerEnd = clamp(constraint.endNode, innerStart, inner.activeEnd);
        let expected = outerStart;
        let previousBestSegment = outerStart;
        let innerArcLength = 0;
        let outerArcEnd = outer.restLength[outerStart];
        for (let innerIndex = innerStart; innerIndex <= innerEnd; innerIndex++) {
            if (innerIndex > innerStart) innerArcLength += inner.restLength[innerIndex - 1];
            while (expected < outerEnd - 1 && outerArcEnd < innerArcLength) {
                expected++;
                outerArcEnd += outer.restLength[expected];
            }
            let searchStart = Math.max(
                outerStart,
                previousBestSegment,
                expected - constraint.searchWindow
            );
            let searchEnd = Math.min(outerEnd - 1, expected + constraint.searchWindow);
            let bestDistanceSq = Infinity;
            let bestSegment = -1;
            let bestT = 0;
            let bestX = 0;
            let bestY = 0;
            let bestZ = 0;
            for (let segment = searchStart; segment <= searchEnd; segment++) {
                const ax = outer.x[segment];
                const ay = outer.y[segment];
                const az = outer.z[segment];
                const dx = outer.x[segment + 1] - ax;
                const dy = outer.y[segment + 1] - ay;
                const dz = outer.z[segment + 1] - az;
                const lengthSq = dx * dx + dy * dy + dz * dz;
                const t = clamp(
                    ((inner.x[innerIndex] - ax) * dx + (inner.y[innerIndex] - ay) * dy + (inner.z[innerIndex] - az) * dz) /
                    Math.max(EPSILON, lengthSq),
                    0,
                    1
                );
                const cx = ax + dx * t;
                const cy = ay + dy * t;
                const cz = az + dz * t;
                const rx = inner.x[innerIndex] - cx;
                const ry = inner.y[innerIndex] - cy;
                const rz = inner.z[innerIndex] - cz;
                const distanceSq = rx * rx + ry * ry + rz * rz;
                if (distanceSq < bestDistanceSq) {
                    bestDistanceSq = distanceSq;
                    bestSegment = segment;
                    bestT = t;
                    bestX = cx;
                    bestY = cy;
                    bestZ = cz;
                }
            }
            if (bestSegment < 0) continue;
            previousBestSegment = bestSegment;
            constraint.closestSegment[innerIndex] = bestSegment;
            if (constraint.openProximal && bestSegment === outerStart && bestT <= 1e-5) {
                const dx = outer.x[outerStart + 1] - outer.x[outerStart];
                const dy = outer.y[outerStart + 1] - outer.y[outerStart];
                const dz = outer.z[outerStart + 1] - outer.z[outerStart];
                const before =
                    (inner.x[innerIndex] - outer.x[outerStart]) * dx +
                    (inner.y[innerIndex] - outer.y[outerStart]) * dy +
                    (inner.z[innerIndex] - outer.z[outerStart]) * dz;
                if (before < 0) continue;
            }
            if (constraint.openDistal && bestSegment === outerEnd - 1 && bestT >= 1 - 1e-5) {
                const dx = outer.x[outerEnd] - outer.x[outerEnd - 1];
                const dy = outer.y[outerEnd] - outer.y[outerEnd - 1];
                const dz = outer.z[outerEnd] - outer.z[outerEnd - 1];
                const beyond =
                    (inner.x[innerIndex] - outer.x[outerEnd]) * dx +
                    (inner.y[innerIndex] - outer.y[outerEnd]) * dy +
                    (inner.z[innerIndex] - outer.z[outerEnd]) * dz;
                if (beyond > 0) continue;
            }
            const distance = Math.sqrt(bestDistanceSq);
            if (distance <= allowedRadius || distance < EPSILON) {
                constraint.lambdas[innerIndex] *= 0.8;
                continue;
            }
            const radialX = (inner.x[innerIndex] - bestX) / distance;
            const radialY = (inner.y[innerIndex] - bestY) / distance;
            const radialZ = (inner.z[innerIndex] - bestZ) / distance;
            const innerResponse = outerOnly
                ? 0
                : innerOnly ? 1 : constraint.innerResponse;
            const outerResponse = innerOnly ? 0 : constraint.outerResponse;
            const innerWeight = inner.inverseMass[innerIndex] * innerResponse;
            const w0Factor = 1 - bestT;
            const w1Factor = bestT;
            const outerWeight0 =
                outer.inverseMass[bestSegment] * outerResponse * w0Factor * w0Factor;
            const outerWeight1 =
                outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor * w1Factor;
            const denominator = innerWeight + outerWeight0 + outerWeight1 + alpha;
            if (denominator < EPSILON) continue;
            const c = allowedRadius - distance;
            const deltaLambda = (-c - alpha * constraint.lambdas[innerIndex]) / denominator;
            constraint.lambdas[innerIndex] += deltaLambda;
            inner.x[innerIndex] -= radialX * deltaLambda * innerWeight;
            inner.y[innerIndex] -= radialY * deltaLambda * innerWeight;
            inner.z[innerIndex] -= radialZ * deltaLambda * innerWeight;
            outer.x[bestSegment] +=
                radialX * deltaLambda * outer.inverseMass[bestSegment] * outerResponse * w0Factor;
            outer.y[bestSegment] +=
                radialY * deltaLambda * outer.inverseMass[bestSegment] * outerResponse * w0Factor;
            outer.z[bestSegment] +=
                radialZ * deltaLambda * outer.inverseMass[bestSegment] * outerResponse * w0Factor;
            outer.x[bestSegment + 1] +=
                radialX * deltaLambda * outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;
            outer.y[bestSegment + 1] +=
                radialY * deltaLambda * outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;
            outer.z[bestSegment + 1] +=
                radialZ * deltaLambda * outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;

            const relativeX =
                inner.x[innerIndex] - inner.previousX[innerIndex] -
                (outer.x[bestSegment] - outer.previousX[bestSegment]) * w0Factor -
                (outer.x[bestSegment + 1] - outer.previousX[bestSegment + 1]) * w1Factor;
            const relativeY =
                inner.y[innerIndex] - inner.previousY[innerIndex] -
                (outer.y[bestSegment] - outer.previousY[bestSegment]) * w0Factor -
                (outer.y[bestSegment + 1] - outer.previousY[bestSegment + 1]) * w1Factor;
            const relativeZ =
                inner.z[innerIndex] - inner.previousZ[innerIndex] -
                (outer.z[bestSegment] - outer.previousZ[bestSegment]) * w0Factor -
                (outer.z[bestSegment + 1] - outer.previousZ[bestSegment + 1]) * w1Factor;
            const normalMotion = relativeX * radialX + relativeY * radialY + relativeZ * radialZ;
            let tangentX = relativeX - radialX * normalMotion;
            let tangentY = relativeY - radialY * normalMotion;
            let tangentZ = relativeZ - radialZ * normalMotion;
            const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
            const frictionWeight = innerWeight + outerWeight0 + outerWeight1;
            if (
                tangentLength > EPSILON && frictionWeight > EPSILON &&
                applyFriction && constraint.friction > 0
            ) {
                tangentX /= tangentLength;
                tangentY /= tangentLength;
                tangentZ /= tangentLength;
                const tangentLambda = -Math.min(
                    tangentLength / frictionWeight,
                    constraint.friction * constraint.lambdas[innerIndex]
                );
                inner.x[innerIndex] += tangentX * tangentLambda * innerWeight;
                inner.y[innerIndex] += tangentY * tangentLambda * innerWeight;
                inner.z[innerIndex] += tangentZ * tangentLambda * innerWeight;
                outer.x[bestSegment] -= tangentX * tangentLambda *
                    outer.inverseMass[bestSegment] * outerResponse * w0Factor;
                outer.y[bestSegment] -= tangentY * tangentLambda *
                    outer.inverseMass[bestSegment] * outerResponse * w0Factor;
                outer.z[bestSegment] -= tangentZ * tangentLambda *
                    outer.inverseMass[bestSegment] * outerResponse * w0Factor;
                outer.x[bestSegment + 1] -= tangentX * tangentLambda *
                    outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;
                outer.y[bestSegment + 1] -= tangentY * tangentLambda *
                    outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;
                outer.z[bestSegment + 1] -= tangentZ * tangentLambda *
                    outer.inverseMass[bestSegment + 1] * outerResponse * w1Factor;
            }
        }
    }

    #projectOuterAlongInnerCenterline(constraint) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerStart = clamp(constraint.startNode, inner.activeStart, inner.activeEnd);
        const outerStart = clamp(constraint.outerStartNode, outer.activeStart, outer.activeEnd);
        if (innerStart >= inner.activeEnd || outerStart > outer.activeEnd) return;

        let innerSegment = innerStart;
        let innerSegmentArc = Math.max(0, constraint.innerArcOffset);
        if (innerStart > inner.activeStart) {
            innerSegment = innerStart - 1;
            innerSegmentArc -= inner.restLength[innerSegment];
        }
        let outerArc = 0;
        const containedLength = Math.max(0, constraint.containedLength);
        for (let outerIndex = outerStart; outerIndex <= outer.activeEnd; outerIndex++) {
            if (outerArc > containedLength + 1e-5) break;
            while (
                innerSegment < inner.activeEnd - 1 &&
                innerSegmentArc + inner.restLength[innerSegment] < outerArc
            ) {
                innerSegmentArc += inner.restLength[innerSegment];
                innerSegment++;
            }
            const segmentLength = Math.max(EPSILON, inner.restLength[innerSegment]);
            const t = clamp((outerArc - innerSegmentArc) / segmentLength, 0, 1);
            const targetX =
                inner.x[innerSegment] +
                (inner.x[innerSegment + 1] - inner.x[innerSegment]) * t;
            const targetY =
                inner.y[innerSegment] +
                (inner.y[innerSegment + 1] - inner.y[innerSegment]) * t;
            const targetZ =
                inner.z[innerSegment] +
                (inner.z[innerSegment + 1] - inner.z[innerSegment]) * t;
            outer.x[outerIndex] = targetX;
            outer.y[outerIndex] = targetY;
            outer.z[outerIndex] = targetZ;
            if (outerIndex < outer.activeEnd) outerArc += outer.restLength[outerIndex];
        }
    }

    #solveToolContact(constraint) {
        if (
            constraint.enabled !== constraint._lastEnabled ||
            constraint.startSegmentA !== constraint._lastStartSegmentA ||
            constraint.endSegmentA !== constraint._lastEndSegmentA ||
            constraint.startSegmentB !== constraint._lastStartSegmentB ||
            constraint.endSegmentB !== constraint._lastEndSegmentB
        ) {
            constraint.lambdas.fill(0);
            constraint._lastEnabled = constraint.enabled;
            constraint._lastStartSegmentA = constraint.startSegmentA;
            constraint._lastEndSegmentA = constraint.endSegmentA;
            constraint._lastStartSegmentB = constraint.startSegmentB;
            constraint._lastEndSegmentB = constraint.endSegmentB;
        }
        if (!constraint.enabled) return;
        const a = constraint.bodyA;
        const b = constraint.bodyB;
        const alpha = constraint.compliance / (this.fixedDt * this.fixedDt);
        const aStart = clamp(constraint.startSegmentA, a.activeStart, a.segmentCount - 1);
        const aEnd = clamp(constraint.endSegmentA, aStart, Math.min(a.activeEnd - 1, a.segmentCount - 1));
        const bStart = clamp(constraint.startSegmentB, b.activeStart, b.segmentCount - 1);
        const bEnd = clamp(constraint.endSegmentB, bStart, Math.min(b.activeEnd - 1, b.segmentCount - 1));
        for (let ia = aStart; ia <= aEnd; ia++) {
            for (let ib = bStart; ib <= bEnd; ib++) {
                const closest = this.#closestSegmentParameters(a, ia, b, ib, this._segmentParameters);
                const ax = a.x[ia] + (a.x[ia + 1] - a.x[ia]) * closest.s;
                const ay = a.y[ia] + (a.y[ia + 1] - a.y[ia]) * closest.s;
                const az = a.z[ia] + (a.z[ia + 1] - a.z[ia]) * closest.s;
                const bx = b.x[ib] + (b.x[ib + 1] - b.x[ib]) * closest.t;
                const by = b.y[ib] + (b.y[ib + 1] - b.y[ib]) * closest.t;
                const bz = b.z[ib] + (b.z[ib + 1] - b.z[ib]) * closest.t;
                if (constraint.openDistalB && ib === bEnd && closest.t >= 1 - 1e-5) {
                    const endDx = b.x[bEnd + 1] - b.x[bEnd];
                    const endDy = b.y[bEnd + 1] - b.y[bEnd];
                    const endDz = b.z[bEnd + 1] - b.z[bEnd];
                    const beyond =
                        (ax - b.x[bEnd + 1]) * endDx +
                        (ay - b.y[bEnd + 1]) * endDy +
                        (az - b.z[bEnd + 1]) * endDz;
                    if (beyond > 0) continue;
                }
                let nx = ax - bx;
                let ny = ay - by;
                let nz = az - bz;
                const distance = magnitude3(nx, ny, nz);
                const minimum = Math.max(a.nodeRadius[ia], a.nodeRadius[ia + 1]) +
                    Math.max(b.nodeRadius[ib], b.nodeRadius[ib + 1]);
                if (distance >= minimum || distance < EPSILON) continue;
                nx /= distance;
                ny /= distance;
                nz /= distance;
                const aw0 = 1 - closest.s;
                const aw1 = closest.s;
                const bw0 = 1 - closest.t;
                const bw1 = closest.t;
                const wa0 = a.inverseMass[ia] * aw0 * aw0;
                const wa1 = a.inverseMass[ia + 1] * aw1 * aw1;
                const wb0 = b.inverseMass[ib] * bw0 * bw0;
                const wb1 = b.inverseMass[ib + 1] * bw1 * bw1;
                const denominator = wa0 + wa1 + wb0 + wb1 + alpha;
                if (denominator < EPSILON) continue;
                const lambdaIndex = ia * b.segmentCount + ib;
                const c = distance - minimum;
                let deltaLambda = (-c - alpha * constraint.lambdas[lambdaIndex]) / denominator;
                const nextLambda = Math.max(0, constraint.lambdas[lambdaIndex] + deltaLambda);
                deltaLambda = nextLambda - constraint.lambdas[lambdaIndex];
                constraint.lambdas[lambdaIndex] = nextLambda;
                a.x[ia] += nx * deltaLambda * a.inverseMass[ia] * aw0;
                a.y[ia] += ny * deltaLambda * a.inverseMass[ia] * aw0;
                a.z[ia] += nz * deltaLambda * a.inverseMass[ia] * aw0;
                a.x[ia + 1] += nx * deltaLambda * a.inverseMass[ia + 1] * aw1;
                a.y[ia + 1] += ny * deltaLambda * a.inverseMass[ia + 1] * aw1;
                a.z[ia + 1] += nz * deltaLambda * a.inverseMass[ia + 1] * aw1;
                b.x[ib] -= nx * deltaLambda * b.inverseMass[ib] * bw0;
                b.y[ib] -= ny * deltaLambda * b.inverseMass[ib] * bw0;
                b.z[ib] -= nz * deltaLambda * b.inverseMass[ib] * bw0;
                b.x[ib + 1] -= nx * deltaLambda * b.inverseMass[ib + 1] * bw1;
                b.y[ib + 1] -= ny * deltaLambda * b.inverseMass[ib + 1] * bw1;
                b.z[ib + 1] -= nz * deltaLambda * b.inverseMass[ib + 1] * bw1;

                const relativeX =
                    (a.x[ia] - a.previousX[ia]) * aw0 +
                    (a.x[ia + 1] - a.previousX[ia + 1]) * aw1 -
                    (b.x[ib] - b.previousX[ib]) * bw0 -
                    (b.x[ib + 1] - b.previousX[ib + 1]) * bw1;
                const relativeY =
                    (a.y[ia] - a.previousY[ia]) * aw0 +
                    (a.y[ia + 1] - a.previousY[ia + 1]) * aw1 -
                    (b.y[ib] - b.previousY[ib]) * bw0 -
                    (b.y[ib + 1] - b.previousY[ib + 1]) * bw1;
                const relativeZ =
                    (a.z[ia] - a.previousZ[ia]) * aw0 +
                    (a.z[ia + 1] - a.previousZ[ia + 1]) * aw1 -
                    (b.z[ib] - b.previousZ[ib]) * bw0 -
                    (b.z[ib + 1] - b.previousZ[ib + 1]) * bw1;
                const normalMotion = relativeX * nx + relativeY * ny + relativeZ * nz;
                let tangentX = relativeX - nx * normalMotion;
                let tangentY = relativeY - ny * normalMotion;
                let tangentZ = relativeZ - nz * normalMotion;
                const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
                const frictionWeight = wa0 + wa1 + wb0 + wb1;
                if (tangentLength > EPSILON && frictionWeight > EPSILON && constraint.friction > 0) {
                    tangentX /= tangentLength;
                    tangentY /= tangentLength;
                    tangentZ /= tangentLength;
                    const tangentLambda = -Math.min(
                        tangentLength / frictionWeight,
                        constraint.friction * nextLambda
                    );
                    a.x[ia] += tangentX * tangentLambda * a.inverseMass[ia] * aw0;
                    a.y[ia] += tangentY * tangentLambda * a.inverseMass[ia] * aw0;
                    a.z[ia] += tangentZ * tangentLambda * a.inverseMass[ia] * aw0;
                    a.x[ia + 1] += tangentX * tangentLambda * a.inverseMass[ia + 1] * aw1;
                    a.y[ia + 1] += tangentY * tangentLambda * a.inverseMass[ia + 1] * aw1;
                    a.z[ia + 1] += tangentZ * tangentLambda * a.inverseMass[ia + 1] * aw1;
                    b.x[ib] -= tangentX * tangentLambda * b.inverseMass[ib] * bw0;
                    b.y[ib] -= tangentY * tangentLambda * b.inverseMass[ib] * bw0;
                    b.z[ib] -= tangentZ * tangentLambda * b.inverseMass[ib] * bw0;
                    b.x[ib + 1] -= tangentX * tangentLambda * b.inverseMass[ib + 1] * bw1;
                    b.y[ib + 1] -= tangentY * tangentLambda * b.inverseMass[ib + 1] * bw1;
                    b.z[ib + 1] -= tangentZ * tangentLambda * b.inverseMass[ib + 1] * bw1;
                }
            }
        }
    }

    #closestSegmentParameters(a, ia, b, ib, out) {
        const ux = a.x[ia + 1] - a.x[ia];
        const uy = a.y[ia + 1] - a.y[ia];
        const uz = a.z[ia + 1] - a.z[ia];
        const vx = b.x[ib + 1] - b.x[ib];
        const vy = b.y[ib + 1] - b.y[ib];
        const vz = b.z[ib + 1] - b.z[ib];
        const wx = a.x[ia] - b.x[ib];
        const wy = a.y[ia] - b.y[ib];
        const wz = a.z[ia] - b.z[ib];
        const aa = ux * ux + uy * uy + uz * uz;
        const bb = ux * vx + uy * vy + uz * vz;
        const cc = vx * vx + vy * vy + vz * vz;
        const dd = ux * wx + uy * wy + uz * wz;
        const ee = vx * wx + vy * wy + vz * wz;
        const denominator = aa * cc - bb * bb;
        let s = denominator > EPSILON ? clamp((bb * ee - cc * dd) / denominator, 0, 1) : 0;
        let t = cc > EPSILON ? clamp((bb * s + ee) / cc, 0, 1) : 0;
        if (aa > EPSILON) s = clamp((bb * t - dd) / aa, 0, 1);
        out.s = s;
        out.t = t;
        return out;
    }

    #solveSheath(sheath) {
        for (let bodyIndex = 0; bodyIndex < this.bodies.length; bodyIndex++) {
            const body = this.bodies[bodyIndex];
            if (sheath.bodies && !sheath.bodies.includes(body)) continue;
            let lambdas = sheath.lambdas.get(body);
            if (!lambdas) {
                lambdas = new Float32Array(body.count);
                sheath.lambdas.set(body, lambdas);
            }
            for (let index = body.activeStart; index <= body.activeEnd; index++) {
                const px = body.x[index] - sheath.startX;
                const py = body.y[index] - sheath.startY;
                const pz = body.z[index] - sheath.startZ;
                const axial = px * sheath.axisX + py * sheath.axisY + pz * sheath.axisZ;
                if (axial <= 0 || axial >= sheath.length) {
                    lambdas[index] *= 0.8;
                    continue;
                }
                const centerX = sheath.startX + sheath.axisX * axial;
                const centerY = sheath.startY + sheath.axisY * axial;
                const centerZ = sheath.startZ + sheath.axisZ * axial;
                const radialX = body.x[index] - centerX;
                const radialY = body.y[index] - centerY;
                const radialZ = body.z[index] - centerZ;
                const distance = magnitude3(radialX, radialY, radialZ);
                const allowed = Math.max(0, sheath.innerRadius - body.nodeRadius[index]);
                if (distance <= allowed || distance < EPSILON) {
                    lambdas[index] *= 0.8;
                    continue;
                }
                const weight = body.inverseMass[index];
                if (weight <= 0) continue;
                const c = allowed - distance;
                const deltaLambda = -c / weight;
                lambdas[index] += deltaLambda;
                body.x[index] -= radialX / distance * deltaLambda * weight;
                body.y[index] -= radialY / distance * deltaLambda * weight;
                body.z[index] -= radialZ / distance * deltaLambda * weight;
            }
        }
    }

    #solveWallContacts(body) {
        if (body.sleeping) return;
        const alpha = body.wallCompliance / (this.fixedDt * this.fixedDt);
        const start = Math.max(0, body.activeStart, body.collisionStartSegment);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        for (let index = start; index < end; index++) {
            if (!body.wallActive[index]) continue;
            const t = body.wallT[index];
            const w0Factor = 1 - t;
            const w1Factor = t;
            const px = body.x[index] * w0Factor + body.x[index + 1] * w1Factor;
            const py = body.y[index] * w0Factor + body.y[index + 1] * w1Factor;
            const pz = body.z[index] * w0Factor + body.z[index + 1] * w1Factor;
            const nx = body.wallNormalX[index];
            const ny = body.wallNormalY[index];
            const nz = body.wallNormalZ[index];
            const radius = Math.max(body.nodeRadius[index], body.nodeRadius[index + 1]);
            const c =
                (px - body.wallX[index]) * nx +
                (py - body.wallY[index]) * ny +
                (pz - body.wallZ[index]) * nz - radius;
            if (c >= 0) {
                body.wallLambda[index] *= 0.85;
                continue;
            }
            const w0 = body.inverseMass[index] * w0Factor * w0Factor;
            const w1 = body.inverseMass[index + 1] * w1Factor * w1Factor;
            const denominator = w0 + w1 + alpha;
            if (denominator < EPSILON) continue;
            let deltaLambda = (-c - alpha * body.wallLambda[index]) / denominator;
            const nextLambda = Math.max(0, body.wallLambda[index] + deltaLambda);
            deltaLambda = nextLambda - body.wallLambda[index];
            body.wallLambda[index] = nextLambda;
            body.x[index] += nx * deltaLambda * body.inverseMass[index] * w0Factor;
            body.y[index] += ny * deltaLambda * body.inverseMass[index] * w0Factor;
            body.z[index] += nz * deltaLambda * body.inverseMass[index] * w0Factor;
            body.x[index + 1] += nx * deltaLambda * body.inverseMass[index + 1] * w1Factor;
            body.y[index + 1] += ny * deltaLambda * body.inverseMass[index + 1] * w1Factor;
            body.z[index + 1] += nz * deltaLambda * body.inverseMass[index + 1] * w1Factor;
        }
    }

    #solveDistributedWallContacts(body) {
        if (body.sleeping) return;
        const correctionX = body.wallCorrectionX;
        const correctionY = body.wallCorrectionY;
        const correctionZ = body.wallCorrectionZ;
        const correctionWeight = body.wallCorrectionWeight;
        correctionX.fill(0);
        correctionY.fill(0);
        correctionZ.fill(0);
        correctionWeight.fill(0);
        const start = Math.max(0, body.activeStart, body.collisionStartSegment);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);

        for (let index = start; index < end; index++) {
            if (!body.wallActive[index]) continue;
            const t = body.wallT[index];
            const px = body.x[index] + (body.x[index + 1] - body.x[index]) * t;
            const py = body.y[index] + (body.y[index + 1] - body.y[index]) * t;
            const pz = body.z[index] + (body.z[index + 1] - body.z[index]) * t;
            const nx = body.wallNormalX[index];
            const ny = body.wallNormalY[index];
            const nz = body.wallNormalZ[index];
            const radius = Math.max(body.nodeRadius[index], body.nodeRadius[index + 1]);
            const penetration = Math.max(0, WALL_SETTLING_CLEARANCE + radius - (
                (px - body.wallX[index]) * nx +
                (py - body.wallY[index]) * ny +
                (pz - body.wallZ[index]) * nz
            ));
            if (penetration <= 0.02) continue;

            const nominalLength = Math.max(0.5, body.segmentLength);
            const span = clamp(Math.ceil(penetration / (nominalLength * 0.02)), 4, 32);
            const contactNode = index + t;
            const firstNode = Math.max(body.activeStart, Math.floor(contactNode - span));
            const lastNode = Math.min(body.activeEnd, Math.ceil(contactNode + span));
            for (let node = firstNode; node <= lastNode; node++) {
                if (body.inverseMass[node] <= 0) continue;
                const weight = Math.max(0, 1 - Math.abs(node - contactNode) / (span + 0.5));
                correctionX[node] += nx * penetration * weight;
                correctionY[node] += ny * penetration * weight;
                correctionZ[node] += nz * penetration * weight;
                correctionWeight[node] += weight;
            }
        }

        for (let node = body.activeStart; node <= body.activeEnd; node++) {
            const weight = correctionWeight[node];
            if (weight > EPSILON) {
                correctionX[node] /= weight;
                correctionY[node] /= weight;
                correctionZ[node] /= weight;
            }
        }

        // Project the correction field itself before applying it. This spreads
        // incompatible normals around bends while bounding the extra strain
        // introduced by each wall pass.
        for (let sweep = 0; sweep < 28; sweep++) {
            let changed = false;
            const reverse = (sweep & 1) === 1;
            for (
                let segment = reverse ? body.activeEnd - 1 : body.activeStart;
                reverse ? segment >= body.activeStart : segment < body.activeEnd;
                segment += reverse ? -1 : 1
            ) {
                const next = segment + 1;
                const dx = correctionX[next] - correctionX[segment];
                const dy = correctionY[next] - correctionY[segment];
                const dz = correctionZ[next] - correctionZ[segment];
                const distanceSq = dx * dx + dy * dy + dz * dz;
                const limit = Math.max(1e-5, body.restLength[segment] * 0.02);
                if (distanceSq <= limit * limit) continue;
                const distance = Math.sqrt(distanceSq);
                const w0 = body.inverseMass[segment];
                const w1 = body.inverseMass[next];
                const totalWeight = w0 + w1;
                if (totalWeight <= EPSILON) continue;
                const excessScale = (distance - limit) / distance;
                const scale0 = excessScale * w0 / totalWeight;
                const scale1 = excessScale * w1 / totalWeight;
                correctionX[segment] += dx * scale0;
                correctionY[segment] += dy * scale0;
                correctionZ[segment] += dz * scale0;
                correctionX[next] -= dx * scale1;
                correctionY[next] -= dy * scale1;
                correctionZ[next] -= dz * scale1;
                changed = true;
            }
            if (!changed) break;
        }

        for (let node = body.activeStart; node <= body.activeEnd; node++) {
            body.x[node] += correctionX[node];
            body.y[node] += correctionY[node];
            body.z[node] += correctionZ[node];
        }

        // A capsule contact exactly at the free distal endpoint cannot be
        // resolved by translating the whole stiff shaft. Finish that one
        // degree of freedom directly; the following substep redistributes the
        // tiny length change through the deliberately soft terminal section.
        const terminalSegment = Math.min(end - 1, body.activeEnd - 1);
        if (
            terminalSegment >= start &&
            body.wallActive[terminalSegment] &&
            body.wallT[terminalSegment] > 0.75 &&
            body.inverseMass[terminalSegment + 1] > 0
        ) {
            const t = body.wallT[terminalSegment];
            const px = body.x[terminalSegment] +
                (body.x[terminalSegment + 1] - body.x[terminalSegment]) * t;
            const py = body.y[terminalSegment] +
                (body.y[terminalSegment + 1] - body.y[terminalSegment]) * t;
            const pz = body.z[terminalSegment] +
                (body.z[terminalSegment + 1] - body.z[terminalSegment]) * t;
            const nx = body.wallNormalX[terminalSegment];
            const ny = body.wallNormalY[terminalSegment];
            const nz = body.wallNormalZ[terminalSegment];
            const radius = Math.max(
                body.nodeRadius[terminalSegment],
                body.nodeRadius[terminalSegment + 1]
            );
            const penetration = Math.max(0, WALL_SETTLING_CLEARANCE + radius - (
                (px - body.wallX[terminalSegment]) * nx +
                (py - body.wallY[terminalSegment]) * ny +
                (pz - body.wallZ[terminalSegment]) * nz
            ));
            if (penetration > 0) {
                const correction = penetration / t;
                body.x[terminalSegment + 1] += nx * correction;
                body.y[terminalSegment + 1] += ny * correction;
                body.z[terminalSegment + 1] += nz * correction;
            }
        }
    }

    #updateVelocityAndFriction(body) {
        if (body.sleeping) return;
        const inverseDt = 1 / this.fixedDt;
        let maxSpeed = 0;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            let dx = body.x[index] - body.previousX[index];
            let dy = body.y[index] - body.previousY[index];
            let dz = body.z[index] - body.previousZ[index];
            let frictionBudget = 0;
            let nx = 0;
            let ny = 0;
            let nz = 0;
            if (index > 0 && body.wallActive[index - 1]) {
                frictionBudget += body.wallLambda[index - 1] * body.wallFriction;
                nx += body.wallNormalX[index - 1];
                ny += body.wallNormalY[index - 1];
                nz += body.wallNormalZ[index - 1];
            }
            if (index < body.segmentCount && body.wallActive[index]) {
                frictionBudget += body.wallLambda[index] * body.wallFriction;
                nx += body.wallNormalX[index];
                ny += body.wallNormalY[index];
                nz += body.wallNormalZ[index];
            }
            const normalLength = magnitude3(nx, ny, nz);
            if (normalLength > EPSILON) {
                nx /= normalLength;
                ny /= normalLength;
                nz /= normalLength;
                let normalMotion = dx * nx + dy * ny + dz * nz;
                // Position projection must not turn into a new outward
                // velocity on the next frame. Remove only the component that
                // points back through the wall; inward release remains free.
                if (normalMotion < 0) {
                    dx -= nx * normalMotion;
                    dy -= ny * normalMotion;
                    dz -= nz * normalMotion;
                    normalMotion = 0;
                }
                const tangentX = dx - nx * normalMotion;
                const tangentY = dy - ny * normalMotion;
                const tangentZ = dz - nz * normalMotion;
                const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
                if (frictionBudget > 0 && tangentLength > EPSILON) {
                    const reduction = Math.min(tangentLength, frictionBudget) / tangentLength;
                    dx -= tangentX * reduction;
                    dy -= tangentY * reduction;
                    dz -= tangentZ * reduction;
                }
            }
            body.velocityX[index] = dx * inverseDt;
            body.velocityY[index] = dy * inverseDt;
            body.velocityZ[index] = dz * inverseDt;
            maxSpeed = Math.max(maxSpeed, magnitude3(body.velocityX[index], body.velocityY[index], body.velocityZ[index]));
        }
        if (maxSpeed < body.sleepVelocity && this.settledMaxPenetration < 0.01) body.sleepCounter++;
        else body.sleepCounter = 0;
        if (body.sleepCounter >= body.sleepFrames) {
            body.sleeping = true;
            body.velocityX.fill(0);
            body.velocityY.fill(0);
            body.velocityZ.fill(0);
        }
    }

    #limitVelocity(body) {
        if (!Number.isFinite(body.maxSpeed) || body.maxSpeed <= 0) return;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            const speed = magnitude3(
                body.velocityX[index],
                body.velocityY[index],
                body.velocityZ[index]
            );
            if (speed <= body.maxSpeed || speed < EPSILON) continue;
            const scale = body.maxSpeed / speed;
            body.velocityX[index] *= scale;
            body.velocityY[index] *= scale;
            body.velocityZ[index] *= scale;
        }
    }

    #stabilizeContainmentVelocity(constraint) {
        if (!constraint.enabled || constraint.outerFollowsInnerCenterline) return;
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const allowedRadius = Math.max(0, constraint.innerRadius - inner.radius);
        const innerStart = clamp(constraint.startNode, inner.activeStart, inner.activeEnd);
        const innerEnd = clamp(constraint.endNode, innerStart, inner.activeEnd);
        const outerStart = clamp(constraint.outerStartNode, outer.activeStart, outer.activeEnd);
        const outerEnd = Math.min(outer.activeEnd, outer.segmentCount);
        for (let innerIndex = innerStart; innerIndex <= innerEnd; innerIndex++) {
            const segment = constraint.closestSegment[innerIndex];
            if (segment < outerStart || segment >= outerEnd) continue;
            const ax = outer.x[segment];
            const ay = outer.y[segment];
            const az = outer.z[segment];
            const dx = outer.x[segment + 1] - ax;
            const dy = outer.y[segment + 1] - ay;
            const dz = outer.z[segment + 1] - az;
            const lengthSq = dx * dx + dy * dy + dz * dz;
            const t = clamp(
                ((inner.x[innerIndex] - ax) * dx +
                    (inner.y[innerIndex] - ay) * dy +
                    (inner.z[innerIndex] - az) * dz) /
                    Math.max(EPSILON, lengthSq),
                0,
                1
            );
            const radialX = inner.x[innerIndex] - (ax + dx * t);
            const radialY = inner.y[innerIndex] - (ay + dy * t);
            const radialZ = inner.z[innerIndex] - (az + dz * t);
            const distance = magnitude3(radialX, radialY, radialZ);
            if (
                distance < EPSILON ||
                (distance < allowedRadius - 0.01 && constraint.lambdas[innerIndex] <= EPSILON)
            ) {
                continue;
            }
            const nx = radialX / distance;
            const ny = radialY / distance;
            const nz = radialZ / distance;
            const w0 = 1 - t;
            const w1 = t;
            const relativeX = inner.velocityX[innerIndex] -
                outer.velocityX[segment] * w0 - outer.velocityX[segment + 1] * w1;
            const relativeY = inner.velocityY[innerIndex] -
                outer.velocityY[segment] * w0 - outer.velocityY[segment + 1] * w1;
            const relativeZ = inner.velocityZ[innerIndex] -
                outer.velocityZ[segment] * w0 - outer.velocityZ[segment + 1] * w1;
            const outwardVelocity = relativeX * nx + relativeY * ny + relativeZ * nz;
            if (outwardVelocity <= 0) continue;
            const innerWeight = inner.inverseMass[innerIndex] * constraint.innerResponse;
            const outerWeight0 =
                outer.inverseMass[segment] * constraint.outerResponse * w0 * w0;
            const outerWeight1 =
                outer.inverseMass[segment + 1] * constraint.outerResponse * w1 * w1;
            const denominator = innerWeight + outerWeight0 + outerWeight1;
            if (denominator < EPSILON) continue;
            const impulse = outwardVelocity / denominator;
            inner.velocityX[innerIndex] -= nx * impulse * innerWeight;
            inner.velocityY[innerIndex] -= ny * impulse * innerWeight;
            inner.velocityZ[innerIndex] -= nz * impulse * innerWeight;
            outer.velocityX[segment] +=
                nx * impulse * outer.inverseMass[segment] * constraint.outerResponse * w0;
            outer.velocityY[segment] +=
                ny * impulse * outer.inverseMass[segment] * constraint.outerResponse * w0;
            outer.velocityZ[segment] +=
                nz * impulse * outer.inverseMass[segment] * constraint.outerResponse * w0;
            outer.velocityX[segment + 1] +=
                nx * impulse * outer.inverseMass[segment + 1] * constraint.outerResponse * w1;
            outer.velocityY[segment + 1] +=
                ny * impulse * outer.inverseMass[segment + 1] * constraint.outerResponse * w1;
            outer.velocityZ[segment + 1] +=
                nz * impulse * outer.inverseMass[segment + 1] * constraint.outerResponse * w1;
        }
    }

    #stabilizeBendingVelocity(body) {
        if (body.sleeping || body.bendDamping <= 0 || body.count < 3) return;
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.count - 1, body.activeEnd);
        for (let index = start; index < end; index++) {
            if (body.inverseMass[index] <= 0) continue;
            const tangentX = body.x[index + 1] - body.x[index - 1];
            const tangentY = body.y[index + 1] - body.y[index - 1];
            const tangentZ = body.z[index + 1] - body.z[index - 1];
            const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
            if (tangentLength < EPSILON) continue;
            const tx = tangentX / tangentLength;
            const ty = tangentY / tangentLength;
            const tz = tangentZ / tangentLength;
            const averageX = (body.velocityX[index - 1] + body.velocityX[index + 1]) * 0.5;
            const averageY = (body.velocityY[index - 1] + body.velocityY[index + 1]) * 0.5;
            const averageZ = (body.velocityZ[index - 1] + body.velocityZ[index + 1]) * 0.5;
            const relativeX = body.velocityX[index] - averageX;
            const relativeY = body.velocityY[index] - averageY;
            const relativeZ = body.velocityZ[index] - averageZ;
            const axial = relativeX * tx + relativeY * ty + relativeZ * tz;
            body.velocityX[index] -= (relativeX - tx * axial) * body.bendDamping;
            body.velocityY[index] -= (relativeY - ty * axial) * body.bendDamping;
            body.velocityZ[index] -= (relativeZ - tz * axial) * body.bendDamping;
        }
    }

    #stabilizeToolContactVelocity(constraint) {
        if (!constraint.enabled) return;
        const a = constraint.bodyA;
        const b = constraint.bodyB;
        const startA = clamp(constraint.startSegmentA, a.activeStart, a.segmentCount - 1);
        const endA = clamp(constraint.endSegmentA, startA, Math.min(a.activeEnd - 1, a.segmentCount - 1));
        const startB = clamp(constraint.startSegmentB, b.activeStart, b.segmentCount - 1);
        const endB = clamp(constraint.endSegmentB, startB, Math.min(b.activeEnd - 1, b.segmentCount - 1));
        for (let ia = startA; ia <= endA; ia++) {
            for (let ib = startB; ib <= endB; ib++) {
                const lambdaIndex = ia * b.segmentCount + ib;
                if (constraint.lambdas[lambdaIndex] <= EPSILON) continue;
                this.#closestSegmentParameters(a, ia, b, ib, this._segmentParameters);
                const s = this._segmentParameters.s;
                const t = this._segmentParameters.t;
                const aw0 = 1 - s;
                const aw1 = s;
                const bw0 = 1 - t;
                const bw1 = t;
                const ax = a.x[ia] * aw0 + a.x[ia + 1] * aw1;
                const ay = a.y[ia] * aw0 + a.y[ia + 1] * aw1;
                const az = a.z[ia] * aw0 + a.z[ia + 1] * aw1;
                const bx = b.x[ib] * bw0 + b.x[ib + 1] * bw1;
                const by = b.y[ib] * bw0 + b.y[ib + 1] * bw1;
                const bz = b.z[ib] * bw0 + b.z[ib + 1] * bw1;
                const dx = ax - bx;
                const dy = ay - by;
                const dz = az - bz;
                const distance = magnitude3(dx, dy, dz);
                if (distance < EPSILON) continue;
                const nx = dx / distance;
                const ny = dy / distance;
                const nz = dz / distance;
                const relativeX =
                    a.velocityX[ia] * aw0 + a.velocityX[ia + 1] * aw1 -
                    b.velocityX[ib] * bw0 - b.velocityX[ib + 1] * bw1;
                const relativeY =
                    a.velocityY[ia] * aw0 + a.velocityY[ia + 1] * aw1 -
                    b.velocityY[ib] * bw0 - b.velocityY[ib + 1] * bw1;
                const relativeZ =
                    a.velocityZ[ia] * aw0 + a.velocityZ[ia + 1] * aw1 -
                    b.velocityZ[ib] * bw0 - b.velocityZ[ib + 1] * bw1;
                const closingVelocity = relativeX * nx + relativeY * ny + relativeZ * nz;
                if (closingVelocity >= 0) continue;
                const wa0 = a.inverseMass[ia] * aw0 * aw0;
                const wa1 = a.inverseMass[ia + 1] * aw1 * aw1;
                const wb0 = b.inverseMass[ib] * bw0 * bw0;
                const wb1 = b.inverseMass[ib + 1] * bw1 * bw1;
                const denominator = wa0 + wa1 + wb0 + wb1;
                if (denominator < EPSILON) continue;
                const impulse = -closingVelocity / denominator;
                a.velocityX[ia] += nx * impulse * a.inverseMass[ia] * aw0;
                a.velocityY[ia] += ny * impulse * a.inverseMass[ia] * aw0;
                a.velocityZ[ia] += nz * impulse * a.inverseMass[ia] * aw0;
                a.velocityX[ia + 1] += nx * impulse * a.inverseMass[ia + 1] * aw1;
                a.velocityY[ia + 1] += ny * impulse * a.inverseMass[ia + 1] * aw1;
                a.velocityZ[ia + 1] += nz * impulse * a.inverseMass[ia + 1] * aw1;
                b.velocityX[ib] -= nx * impulse * b.inverseMass[ib] * bw0;
                b.velocityY[ib] -= ny * impulse * b.inverseMass[ib] * bw0;
                b.velocityZ[ib] -= nz * impulse * b.inverseMass[ib] * bw0;
                b.velocityX[ib + 1] -= nx * impulse * b.inverseMass[ib + 1] * bw1;
                b.velocityY[ib + 1] -= ny * impulse * b.inverseMass[ib + 1] * bw1;
                b.velocityZ[ib + 1] -= nz * impulse * b.inverseMass[ib + 1] * bw1;
            }
        }
    }

    #bodyStats(body) {
        let maxLengthError = 0;
        let maxBendAngle = 0;
        let finite = true;
        for (let index = body.activeStart; index < Math.min(body.activeEnd, body.segmentCount); index++) {
            const dx = body.x[index + 1] - body.x[index];
            const dy = body.y[index + 1] - body.y[index];
            const dz = body.z[index + 1] - body.z[index];
            const length = magnitude3(dx, dy, dz);
            maxLengthError = Math.max(maxLengthError, Math.abs(length - body.restLength[index]) / Math.max(EPSILON, body.restLength[index]));
            finite = finite && Number.isFinite(length);
            if (index <= body.activeStart || index >= body.activeEnd - 1) continue;
            const ax = body.x[index] - body.x[index - 1];
            const ay = body.y[index] - body.y[index - 1];
            const az = body.z[index] - body.z[index - 1];
            const bx = body.x[index + 1] - body.x[index];
            const by = body.y[index + 1] - body.y[index];
            const bz = body.z[index + 1] - body.z[index];
            const denominator = magnitude3(ax, ay, az) * magnitude3(bx, by, bz);
            if (denominator > EPSILON) {
                maxBendAngle = Math.max(maxBendAngle, Math.acos(clamp((ax * bx + ay * by + az * bz) / denominator, -1, 1)));
            }
        }
        return {
            id: body.id,
            sleeping: body.sleeping,
            finite,
            maxLengthError,
            maxBendAngleDegrees: maxBendAngle * 180 / Math.PI
        };
    }
}
