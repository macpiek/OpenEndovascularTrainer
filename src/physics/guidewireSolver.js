import * as THREE from 'three';

const DEFAULT_CONTACT_BAND = 1.35;
const DEFAULT_LUMEN_CLEARANCE = 0.72;
const DEFAULT_AXIAL_WINDOW_SCALE = 2.4;
const DEFAULT_STRAIGHTENING = 0.42;
const DEFAULT_ROUTE_BLEND = 0.035;
const DEFAULT_RELAXATION_ITERATIONS = 18;
const DEFAULT_LENGTH_ITERATIONS = 5;
const DEFAULT_SEGMENT_SAMPLES = [0.06, 0.12, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.9, 0.94];
const DEFAULT_MAX_BEND_ANGLE = 64;
const DEFAULT_BEND_LIMIT_STRENGTH = 0.32;
const DEFAULT_BEND_LIMIT_ITERATIONS = 0;
const DEFAULT_SEGMENT_PROJECTION_BLEND = 1;
const DEFAULT_MAX_SEGMENT_PROJECTION_STEP = 0.55;
const DEFAULT_FOLD_ANGLE = 142;
const DEFAULT_FOLD_UNTANGLE_STRENGTH = 0;
const DEFAULT_FOLD_UNTANGLE_WINDOW = 5;
const DEFAULT_FINAL_COLLISION_PASSES = 10;
const DEFAULT_FINAL_LENGTH_PASSES = 6;
const DEFAULT_FINAL_PROJECTION_PASSES = 2;
const SHEATH_BOUNDARY_EPSILON = 1e-3;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function smoothstep(edge0, edge1, value) {
    const t = clamp((value - edge0) / Math.max(1e-6, edge1 - edge0), 0, 1);
    return t * t * (3 - 2 * t);
}

function copyNodePosition(node) {
    return { x: node.x, y: node.y, z: node.z };
}

function setNode(node, point) {
    node.x = point.x;
    node.y = point.y;
    node.z = point.z;
}

function addScaled(node, vector, scale) {
    node.x += vector.x * scale;
    node.y += vector.y * scale;
    node.z += vector.z * scale;
}

function nodeDistance(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function normalizeVector(vector, fallback) {
    const length = Math.hypot(vector.x, vector.y, vector.z);
    if (length < 1e-8) return { ...fallback };
    return { x: vector.x / length, y: vector.y / length, z: vector.z / length };
}

function interpolatePosition(a, b, t) {
    return {
        x: a.x * (1 - t) + b.x * t,
        y: a.y * (1 - t) + b.y * t,
        z: a.z * (1 - t) + b.z * t
    };
}

export class GuidewireSolver {
    constructor({
        rod,
        segmentLength,
        guidewireLength,
        sheath,
        lumenSampler,
        advanceRate = 44,
        minInsert = 0,
        maxInsert = guidewireLength,
        lumenClearance = DEFAULT_LUMEN_CLEARANCE,
        axialWindowScale = DEFAULT_AXIAL_WINDOW_SCALE,
        straightening = DEFAULT_STRAIGHTENING,
        routeBlend = DEFAULT_ROUTE_BLEND,
        relaxationIterations = DEFAULT_RELAXATION_ITERATIONS,
        lengthIterations = DEFAULT_LENGTH_ITERATIONS,
        segmentSamples = DEFAULT_SEGMENT_SAMPLES,
        maxBendAngle = DEFAULT_MAX_BEND_ANGLE,
        bendLimitStrength = DEFAULT_BEND_LIMIT_STRENGTH,
        bendLimitIterations = DEFAULT_BEND_LIMIT_ITERATIONS,
        segmentProjectionBlend = DEFAULT_SEGMENT_PROJECTION_BLEND,
        maxSegmentProjectionStep = DEFAULT_MAX_SEGMENT_PROJECTION_STEP,
        foldAngle = DEFAULT_FOLD_ANGLE,
        foldUntangleStrength = DEFAULT_FOLD_UNTANGLE_STRENGTH,
        foldUntangleWindow = DEFAULT_FOLD_UNTANGLE_WINDOW,
        finalCollisionPasses = DEFAULT_FINAL_COLLISION_PASSES,
        finalLengthPasses = DEFAULT_FINAL_LENGTH_PASSES,
        finalProjectionPasses = DEFAULT_FINAL_PROJECTION_PASSES
    }) {
        this.rod = rod;
        this.segmentLength = segmentLength;
        this.guidewireLength = guidewireLength;
        this.sheath = sheath;
        this.lumenSampler = lumenSampler;
        this.advanceRate = advanceRate;
        this.minInsert = minInsert;
        this.maxInsert = maxInsert;
        this.lumenClearance = lumenClearance;
        this.axialWindowScale = axialWindowScale;
        this.straightening = straightening;
        this.routeBlend = routeBlend;
        this.relaxationIterations = relaxationIterations;
        this.lengthIterations = lengthIterations;
        this.segmentSamples = segmentSamples;
        this.maxBendAngle = maxBendAngle;
        this.bendLimitStrength = bendLimitStrength;
        this.bendLimitIterations = bendLimitIterations;
        this.segmentProjectionBlend = segmentProjectionBlend;
        this.maxSegmentProjectionStep = maxSegmentProjectionStep;
        this.foldAngle = foldAngle;
        this.foldUntangleStrength = foldUntangleStrength;
        this.foldUntangleWindow = foldUntangleWindow;
        this.finalCollisionPasses = finalCollisionPasses;
        this.finalLengthPasses = finalLengthPasses;
        this.finalProjectionPasses = finalProjectionPasses;
        this.tailProgress = 0;
        this.lastAdvanceDelta = 0;
        this.settleFramesRemaining = 0;
        this.contactPoints = [];
        this.breachPoints = [];
        this.previousPositions = null;

        const sheathAxis = {
            x: sheath.end.x - sheath.start.x,
            y: sheath.end.y - sheath.start.y,
            z: sheath.end.z - sheath.start.z
        };
        this.sheathLength = Math.hypot(sheathAxis.x, sheathAxis.y, sheathAxis.z) || 1;
        this.sheathDir = normalizeVector(sheathAxis, { x: 1, y: 0, z: 0 });
        this.externalTailStart = {
            x: sheath.start.x - this.sheathDir.x * guidewireLength,
            y: sheath.start.y - this.sheathDir.y * guidewireLength,
            z: sheath.start.z - this.sheathDir.z * guidewireLength
        };
    }

    get progress() {
        return this.tailProgress;
    }

    initialize() {
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const distance = this.segmentLength * i;
            const node = this.rod.nodes[i];
            node.x = this.externalTailStart.x + this.sheathDir.x * distance;
            node.y = this.externalTailStart.y + this.sheathDir.y * distance;
            node.z = this.externalTailStart.z + this.sheathDir.z * distance;
            node.vx = node.vy = node.vz = 0;
            node.pinned = true;
        }
        this.constrainSheath();
        this.previousPositions = this.rod.nodes.map(copyNodePosition);
    }

    insertedCoordinate(indexOrFloat) {
        return this.segmentLength * indexOrFloat - this.guidewireLength + this.tailProgress;
    }

    firstLumenNodeIndex() {
        return clamp(
            Math.ceil((this.sheathLength + this.guidewireLength - this.tailProgress) / this.segmentLength),
            0,
            this.rod.nodes.length
        );
    }

    firstInsertedNodeIndex() {
        return clamp(
            Math.ceil((this.guidewireLength - this.tailProgress) / this.segmentLength),
            0,
            this.rod.nodes.length
        );
    }

    sheathAxisPoint(inserted) {
        return {
            x: this.sheath.start.x + this.sheathDir.x * inserted,
            y: this.sheath.start.y + this.sheathDir.y * inserted,
            z: this.sheath.start.z + this.sheathDir.z * inserted
        };
    }

    routeSample(inserted) {
        if (this.#isInSheath(inserted)) {
            return {
                point: this.sheathAxisPoint(inserted),
                tangent: { ...this.sheathDir },
                radius: this.sheath.radius || 2
            };
        }
        return this.lumenSampler(Math.max(0, inserted - this.sheathLength));
    }

    constrainSheath(feedSpeed = 0) {
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const inserted = this.insertedCoordinate(i);
            const node = this.rod.nodes[i];
            const inSheath = this.#isInSheath(inserted);
            node.pinned = inSheath;
            if (!inSheath) continue;

            const target = this.sheathAxisPoint(inserted);
            setNode(node, target);
            node.vx = this.sheathDir.x * feedSpeed;
            node.vy = this.sheathDir.y * feedSpeed;
            node.vz = this.sheathDir.z * feedSpeed;
        }
    }

    advance(command, dt) {
        const previous = this.rod.nodes.map(copyNodePosition);
        const nextProgress = clamp(
            this.tailProgress + command * this.advanceRate * dt,
            this.minInsert,
            this.maxInsert
        );
        const delta = nextProgress - this.tailProgress;
        this.tailProgress = nextProgress;
        this.lastAdvanceDelta = delta;
        if (Math.abs(delta) > 1e-6) this.requestSettle();
        const feedSpeed = delta / Math.max(dt, 1e-6);

        this.constrainSheath(feedSpeed);
        if (Math.abs(delta) > 1e-6) {
            this.#convectMaterial(delta, previous, dt);
        }

        this.previousPositions = previous;
        return delta;
    }

    requestSettle(frames = 48) {
        this.settleFramesRemaining = Math.max(this.settleFramesRemaining, frames);
    }

    solve(dt, collisionTarget = null, { iterations = this.relaxationIterations, forceRelax = false } = {}) {
        const before = this.rod.nodes.map(copyNodePosition);
        this.contactPoints.length = 0;
        this.breachPoints.length = 0;

        this.constrainSheath();
        const shouldRelax = forceRelax || Math.abs(this.lastAdvanceDelta) > 1e-6 || this.settleFramesRemaining > 0;
        if (!shouldRelax) {
            this.#zeroVelocities(before, dt);
            return;
        }

        if (Math.abs(this.lastAdvanceDelta) > 1e-6) {
            this.#routeNudge();
        }

        const passCount = Math.max(1, iterations);
        for (let pass = 0; pass < passCount; pass++) {
            this.#straightenInsideVessel(pass / passCount);
            this.#untangleFoldedSections();
            this.#limitBendsInsideVessel();
            this.#solveLengths(this.lengthIterations);
            this.#projectNodesInside(collisionTarget, true);
            this.#projectSegmentsInside(collisionTarget, true);
            this.#limitBendsInsideVessel();
            this.#projectNodesInside(collisionTarget, true);
            this.#projectSegmentsInside(collisionTarget, true);
            this.#solveLengths(2);
        }

        this.constrainSheath();
        for (let pass = 0; pass < this.finalCollisionPasses; pass++) {
            this.#untangleFoldedSections();
            this.#limitBendsInsideVessel();
            this.#projectNodesInside(collisionTarget, true);
            this.#projectSegmentsInside(collisionTarget, true);
            this.#solveLengths(this.lengthIterations + 2);
        }
        this.#solveLengths(this.lengthIterations + 4);
        for (let pass = 0; pass < this.finalLengthPasses; pass++) {
            this.#projectNodesInside(collisionTarget, true);
            this.#projectSegmentsInside(collisionTarget, true);
            this.#solveLengths(5);
        }
        for (let pass = 0; pass < this.finalProjectionPasses; pass++) {
            this.#projectNodesInside(collisionTarget, true);
            this.#projectSegmentsInside(collisionTarget, true);
        }
        this.#zeroVelocities(before, dt);
        if (Math.abs(this.lastAdvanceDelta) <= 1e-6 && this.settleFramesRemaining > 0) {
            this.settleFramesRemaining--;
        }
    }

    collectContactSamples(collisionTarget = null, contactBand = DEFAULT_CONTACT_BAND) {
        const contacts = [...this.contactPoints];
        const breaches = [...this.breachPoints];
        const samples = [0, 0.15, 0.35, 0.55, 0.75, 0.9, 1];
        const addDiagnosticPoint = (point, inserted) => {
            if (this.#isInSheath(inserted)) return;
            const state = this.diagnosePoint(point, inserted, collisionTarget, contactBand);
            if (state.breach) this.#pushLimited(breaches, point);
            else if (state.contact) this.#pushLimited(contacts, point);
        };

        for (let i = 0; i < this.rod.nodes.length - 1; i++) {
            const n0 = this.rod.nodes[i];
            const n1 = this.rod.nodes[i + 1];
            for (const t of samples) {
                const inserted = this.insertedCoordinate(i + t);
                const point = interpolatePosition(n0, n1, t);
                addDiagnosticPoint(point, inserted);
            }
        }

        const tip = this.rod.nodes[this.rod.nodes.length - 1];
        addDiagnosticPoint(tip, this.insertedCoordinate(this.rod.nodes.length - 1));

        return { contacts, breaches };
    }

    diagnosePoint(point, inserted, collisionTarget = null, contactBand = DEFAULT_CONTACT_BAND) {
        const lumen = this.#lumenConstraint(point, inserted);
        let contact = lumen.radialMargin <= contactBand || Math.abs(lumen.axialOffset) >= lumen.axialWindow - contactBand;
        let breach = lumen.breach;
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        if (collider?.pointContact && !this.#isInSheath(inserted)) {
            const meshContact = collider.pointContact(point, 0);
            breach = breach || !!meshContact?.violation;
            contact = contact || (
                !meshContact?.violation &&
                Number.isFinite(meshContact?.distance) &&
                meshContact.distance <= contactBand
            );
        }

        return { contact: !breach && contact, breach };
    }

    #convectMaterial(delta, previous, dt) {
        const sourceShift = delta / this.segmentLength;
        const invDt = 1 / Math.max(dt, 1e-6);

        for (let i = 0; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            const inserted = this.insertedCoordinate(i);
            if (this.#isInSheath(inserted)) continue;

            const source = this.#samplePreviousPosition(previous, i + sourceShift);
            const route = this.routeSample(inserted).point;
            const justExited = inserted < this.sheathLength + this.segmentLength * 2;
            const blend = justExited ? 0.64 : 0.12;
            const target = {
                x: source.x * (1 - blend) + route.x * blend,
                y: source.y * (1 - blend) + route.y * blend,
                z: source.z * (1 - blend) + route.z * blend
            };

            const old = previous[i];
            setNode(node, target);
            node.vx = (target.x - old.x) * invDt * 0.2;
            node.vy = (target.y - old.y) * invDt * 0.2;
            node.vz = (target.z - old.z) * invDt * 0.2;
        }
    }

    #samplePreviousPosition(previous, sourceIndex) {
        const lastIndex = previous.length - 1;
        if (sourceIndex <= 0) {
            const head = previous[0];
            return {
                x: head.x + this.sheathDir.x * sourceIndex * this.segmentLength,
                y: head.y + this.sheathDir.y * sourceIndex * this.segmentLength,
                z: head.z + this.sheathDir.z * sourceIndex * this.segmentLength
            };
        }

        if (sourceIndex < lastIndex) {
            const lower = Math.floor(sourceIndex);
            const upper = Math.min(lastIndex, lower + 1);
            return interpolatePosition(previous[lower], previous[upper], sourceIndex - lower);
        }

        const tip = previous[lastIndex];
        const prev = previous[Math.max(0, lastIndex - 1)];
        const direction = normalizeVector({
            x: tip.x - prev.x,
            y: tip.y - prev.y,
            z: tip.z - prev.z
        }, this.routeSample(this.tailProgress).tangent);
        const distance = (sourceIndex - lastIndex) * this.segmentLength;
        return {
            x: tip.x + direction.x * distance,
            y: tip.y + direction.y * distance,
            z: tip.z + direction.z * distance
        };
    }

    #routeNudge(multiplier = 1) {
        if (this.routeBlend <= 0) return;
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (this.#isInSheath(inserted)) continue;

            const sample = this.routeSample(inserted);
            const fade = smoothstep(this.sheathLength, this.sheathLength + this.segmentLength * 8, inserted);
            const blend = this.routeBlend * multiplier * (0.35 + 0.65 * fade);
            node.x += (sample.point.x - node.x) * blend;
            node.y += (sample.point.y - node.y) * blend;
            node.z += (sample.point.z - node.z) * blend;
        }
    }

    #straightenInsideVessel(progress) {
        const corrections = new Array(this.rod.nodes.length);
        const endpointBias = 0.35 + 0.65 * smoothstep(0, 1, progress);

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;

            const prev = this.rod.nodes[i - 1];
            const next = this.rod.nodes[i + 1];
            const midpoint = {
                x: (prev.x + next.x) * 0.5,
                y: (prev.y + next.y) * 0.5,
                z: (prev.z + next.z) * 0.5
            };
            const inserted = this.insertedCoordinate(i);
            const nearExit = 1 - smoothstep(this.sheathLength, this.sheathLength + this.segmentLength * 5, inserted);
            const strength = this.straightening * endpointBias * (1 - nearExit * 0.45);
            corrections[i] = {
                x: (midpoint.x - node.x) * strength,
                y: (midpoint.y - node.y) * strength,
                z: (midpoint.z - node.z) * strength
            };
        }

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            const correction = corrections[i];
            if (!correction || node.pinned) continue;
            addScaled(node, correction, 1);
        }

        const spans = [2, 4, 8, 12];
        for (const span of spans) {
            const spanCorrections = new Array(this.rod.nodes.length);
            const spanStrength = this.straightening * 0.13 / Math.sqrt(span);
            for (let i = span; i < this.rod.nodes.length - span; i++) {
                const node = this.rod.nodes[i];
                if (node.pinned) continue;
                const prev = this.rod.nodes[i - span];
                const next = this.rod.nodes[i + span];
                const target = {
                    x: (prev.x + next.x) * 0.5,
                    y: (prev.y + next.y) * 0.5,
                    z: (prev.z + next.z) * 0.5
                };
                spanCorrections[i] = {
                    x: (target.x - node.x) * spanStrength,
                    y: (target.y - node.y) * spanStrength,
                    z: (target.z - node.z) * spanStrength
                };
            }
            for (let i = span; i < this.rod.nodes.length - span; i++) {
                const node = this.rod.nodes[i];
                const correction = spanCorrections[i];
                if (!correction || node.pinned) continue;
                addScaled(node, correction, 1);
            }
        }
    }

    #untangleFoldedSections() {
        if (this.foldUntangleStrength <= 0 || this.foldUntangleWindow <= 0) return;
        const corrections = new Array(this.rod.nodes.length);
        const threshold = clamp(this.foldAngle, 1, 179);
        const window = Math.max(1, Math.floor(this.foldUntangleWindow));
        const baseStrength = clamp(this.foldUntangleStrength, 0, 1);

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;
            const angle = this.rod.bendAngleAt?.(i) ?? 0;
            if (angle <= threshold) continue;

            const severity = clamp((angle - threshold) / Math.max(1, 180 - threshold), 0, 1);
            for (let offset = -window; offset <= window; offset++) {
                const index = i + offset;
                const targetNode = this.rod.nodes[index];
                if (!targetNode || targetNode.pinned) continue;
                const targetInserted = this.insertedCoordinate(index);
                if (this.#isInSheath(targetInserted)) continue;
                const falloff = 1 - Math.abs(offset) / (window + 1);
                const strength = baseStrength * severity * falloff;
                if (strength <= 0) continue;
                const route = this.routeSample(targetInserted).point;
                corrections[index] ??= { x: 0, y: 0, z: 0 };
                corrections[index].x += (route.x - targetNode.x) * strength;
                corrections[index].y += (route.y - targetNode.y) * strength;
                corrections[index].z += (route.z - targetNode.z) * strength;
            }
        }

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            const correction = corrections[i];
            if (!correction || node.pinned) continue;
            addScaled(node, correction, 1);
        }
    }

    #limitBendsInsideVessel() {
        if (this.bendLimitIterations <= 0 || this.bendLimitStrength <= 0) return;
        const limit = clamp(this.maxBendAngle, 1, 179) * Math.PI / 180;
        const minChord = 2 * this.segmentLength * Math.cos(limit * 0.5);
        const baseStrength = clamp(this.bendLimitStrength, 0, 1);

        for (let iter = 0; iter < this.bendLimitIterations; iter++) {
            const corrections = new Array(this.rod.nodes.length);
            for (let i = 1; i < this.rod.nodes.length - 1; i++) {
                const node = this.rod.nodes[i];
                if (node.pinned) continue;
                const inserted = this.insertedCoordinate(i);
                if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;

                const angleDegrees = this.rod.bendAngleAt?.(i) ?? 0;
                if (angleDegrees <= this.maxBendAngle) continue;

                const prev = this.rod.nodes[i - 1];
                const next = this.rod.nodes[i + 1];
                const chord = {
                    x: next.x - prev.x,
                    y: next.y - prev.y,
                    z: next.z - prev.z
                };
                const chordLength = Math.hypot(chord.x, chord.y, chord.z);
                const routeTangent = this.routeSample(inserted).tangent;
                const direction = normalizeVector(chord, routeTangent);
                const severity = clamp((angleDegrees - this.maxBendAngle) / Math.max(1, 180 - this.maxBendAngle), 0, 1);
                const strength = baseStrength * (0.35 + 0.65 * severity);

                if (chordLength < minChord) {
                    const spread = (minChord - chordLength) * 0.5 * strength;
                    if (!prev.pinned) {
                        corrections[i - 1] ??= { x: 0, y: 0, z: 0 };
                        corrections[i - 1].x -= direction.x * spread;
                        corrections[i - 1].y -= direction.y * spread;
                        corrections[i - 1].z -= direction.z * spread;
                    }
                    if (!next.pinned) {
                        corrections[i + 1] ??= { x: 0, y: 0, z: 0 };
                        corrections[i + 1].x += direction.x * spread;
                        corrections[i + 1].y += direction.y * spread;
                        corrections[i + 1].z += direction.z * spread;
                    }
                }

                corrections[i] ??= { x: 0, y: 0, z: 0 };
                corrections[i].x += ((prev.x + next.x) * 0.5 - node.x) * strength * 0.45;
                corrections[i].y += ((prev.y + next.y) * 0.5 - node.y) * strength * 0.45;
                corrections[i].z += ((prev.z + next.z) * 0.5 - node.z) * strength * 0.45;
            }

            for (let i = 1; i < this.rod.nodes.length - 1; i++) {
                const node = this.rod.nodes[i];
                const correction = corrections[i];
                if (!correction || node.pinned) continue;
                addScaled(node, correction, 1);
            }
        }
    }

    #solveLengths(iterations) {
        const rest = this.segmentLength;
        for (let iter = 0; iter < iterations; iter++) {
            for (let i = 0; i < this.rod.nodes.length - 1; i++) {
                const n0 = this.rod.nodes[i];
                const n1 = this.rod.nodes[i + 1];
                const dx = n1.x - n0.x;
                const dy = n1.y - n0.y;
                const dz = n1.z - n0.z;
                const dist = Math.hypot(dx, dy, dz);
                if (dist < 1e-8) continue;

                const correction = (dist - rest) / dist;
                const w0 = n0.pinned ? 0 : 1;
                const w1 = n1.pinned ? 0 : 1;
                const total = w0 + w1;
                if (total <= 0) continue;

                const c0 = w0 / total;
                const c1 = w1 / total;
                if (w0) {
                    n0.x += dx * correction * c0;
                    n0.y += dy * correction * c0;
                    n0.z += dz * correction * c0;
                }
                if (w1) {
                    n1.x -= dx * correction * c1;
                    n1.y -= dy * correction * c1;
                    n1.z -= dz * correction * c1;
                }
            }
            this.constrainSheath();
        }
    }

    #projectNodesInside(collisionTarget, recordContacts) {
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (this.#isInSheath(inserted)) continue;
            const projected = this.#projectPointInside(node, inserted, collisionTarget, recordContacts);
            setNode(node, projected);
        }
    }

    #projectSegmentsInside(collisionTarget, recordContacts) {
        for (let i = 0; i < this.rod.nodes.length - 1; i++) {
            const n0 = this.rod.nodes[i];
            const n1 = this.rod.nodes[i + 1];
            if (n0.pinned && n1.pinned) continue;

            for (const t of this.segmentSamples) {
                const inserted = this.insertedCoordinate(i + t);
                if (this.#isInSheath(inserted)) continue;
                const point = interpolatePosition(n0, n1, t);
                const target = this.#projectPointInside(point, inserted, collisionTarget, recordContacts);
                const correction = {
                    x: (target.x - point.x) * this.segmentProjectionBlend,
                    y: (target.y - point.y) * this.segmentProjectionBlend,
                    z: (target.z - point.z) * this.segmentProjectionBlend
                };
                const correctionLength = Math.hypot(correction.x, correction.y, correction.z);
                const maxCorrection = this.segmentLength * this.maxSegmentProjectionStep;
                if (correctionLength > maxCorrection) {
                    const scale = maxCorrection / correctionLength;
                    correction.x *= scale;
                    correction.y *= scale;
                    correction.z *= scale;
                }
                const w0 = n0.pinned ? 0 : 1 - t;
                const w1 = n1.pinned ? 0 : t;
                const denom = w0 * w0 + w1 * w1;
                if (denom <= 1e-8) continue;
                if (w0) addScaled(n0, correction, w0 / denom);
                if (w1) addScaled(n1, correction, w1 / denom);
            }
        }
    }

    #projectPointInside(point, inserted, collisionTarget, recordContacts) {
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        if (collider?.pointContact && !this.#isInSheath(inserted)) {
            const lumenState = this.#lumenConstraint(point, inserted);
            const lumenProjected = lumenState.projected;
            if (recordContacts) {
                if (lumenState.breach) this.#pushLimited(this.breachPoints, point);
                else if (lumenState.radialMargin <= DEFAULT_CONTACT_BAND) this.#pushLimited(this.contactPoints, point);
            }
            const contact = collider.pointContact(lumenProjected, 0);
            if (contact?.violation && contact.target) {
                if (recordContacts) this.#pushLimited(this.breachPoints, lumenProjected);
                return { x: contact.target.x, y: contact.target.y, z: contact.target.z };
            } else if (recordContacts && Number.isFinite(contact?.distance) && contact.distance <= DEFAULT_CONTACT_BAND) {
                this.#pushLimited(this.contactPoints, lumenProjected);
            }
            return lumenProjected;
        }

        return this.#projectToLumen(point, inserted, recordContacts);
    }

    #projectToLumen(point, inserted, recordContacts) {
        const state = this.#lumenConstraint(point, inserted);
        if (recordContacts) {
            if (state.breach) this.#pushLimited(this.breachPoints, point);
            else if (state.radialMargin <= DEFAULT_CONTACT_BAND) this.#pushLimited(this.contactPoints, point);
        }
        return state.projected;
    }

    #lumenConstraint(point, inserted) {
        const sample = this.routeSample(inserted);
        const radius = Math.max(0.5, (sample.radius || 1) - this.lumenClearance);
        const tangent = normalizeVector(sample.tangent, this.sheathDir);
        const dx = point.x - sample.point.x;
        const dy = point.y - sample.point.y;
        const dz = point.z - sample.point.z;
        let axialOffset = dx * tangent.x + dy * tangent.y + dz * tangent.z;
        let lateralX = dx - tangent.x * axialOffset;
        let lateralY = dy - tangent.y * axialOffset;
        let lateralZ = dz - tangent.z * axialOffset;
        let lateralLength = Math.hypot(lateralX, lateralY, lateralZ);
        const axialWindow = Math.max(this.segmentLength * 0.5, this.segmentLength * this.axialWindowScale);
        const breach = lateralLength > radius + 1e-4;

        if (lateralLength > radius) {
            const scale = radius / Math.max(1e-8, lateralLength);
            lateralX *= scale;
            lateralY *= scale;
            lateralZ *= scale;
            lateralLength = radius;
        }
        axialOffset = clamp(axialOffset, -axialWindow, axialWindow);

        return {
            projected: {
                x: sample.point.x + tangent.x * axialOffset + lateralX,
                y: sample.point.y + tangent.y * axialOffset + lateralY,
                z: sample.point.z + tangent.z * axialOffset + lateralZ
            },
            radialMargin: radius - lateralLength,
            axialOffset,
            axialWindow,
            breach
        };
    }

    #zeroVelocities(before, dt) {
        const invDt = 1 / Math.max(dt, 1e-6);
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            const moved = nodeDistance(node, before[i]);
            const scale = moved > 0.02 ? 0.08 : 0;
            node.vx = (node.x - before[i].x) * invDt * scale;
            node.vy = (node.y - before[i].y) * invDt * scale;
            node.vz = (node.z - before[i].z) * invDt * scale;
        }
    }

    #pushLimited(points, point, limit = 420) {
        if (points.length >= limit) return;
        points.push({ x: point.x, y: point.y, z: point.z });
    }

    #isInSheath(inserted) {
        return inserted <= this.sheathLength + SHEATH_BOUNDARY_EPSILON;
    }
}
