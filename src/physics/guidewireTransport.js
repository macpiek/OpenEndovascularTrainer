import { clamp } from '../mathUtils.js';
import { snapshotNodePositions } from './pointBuffer.js';
import { SHEATH_BOUNDARY_EPSILON } from './sheathBoundary.js';

const DEFAULT_CONTACT_BAND = 1.35;

const DEFAULT_DIAGNOSTIC_SAMPLES = [0, 0.2, 0.4, 0.6, 0.8, 1];

function createContactScratch() {
    return {
        query: {
            inward: { x: 0, y: 0, z: 0 },
            normal: { x: 0, y: 0, z: 0 },
            closestPoint: { x: 0, y: 0, z: 0 }
        },
        target: { x: 0, y: 0, z: 0 },
        closestPoint: { x: 0, y: 0, z: 0 },
        inward: { x: 0, y: 0, z: 0 },
        normal: { x: 0, y: 0, z: 0 }
    };
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

function nowMs() {
    return globalThis.performance?.now?.() ?? Date.now();
}

function createPerformanceStats() {
    return {
        advanceMs: 0,
        diagnosticMs: 0,
        diagnosticPointContactCount: 0,
        moving: false,
        boundaryDrivenFeed: false,
        transportDeltaMm: 0,
        transportSpeedMmPerSecond: 0,
    };
}

function resetPerformanceStats(stats) {
    stats.advanceMs = 0;
    stats.diagnosticMs = 0;
    stats.diagnosticPointContactCount = 0;
    stats.moving = false;
    stats.boundaryDrivenFeed = false;
    stats.transportDeltaMm = 0;
    stats.transportSpeedMmPerSecond = 0;
    return stats;
}

/** Prescribed guidewire feed and read-only lumen diagnostics; no rod solver. */
export class GuidewireTransport {
    constructor({
        rod, segmentLength, guidewireLength, sheath, lumenSampler = null,
        advanceRate = 44, minInsert = 0, maxInsert = guidewireLength,
        lumenClearance = 0.72, axialWindowScale = 2.4, meshClearance = 0.45
    }) {
        Object.assign(this, {
            rod, segmentLength, guidewireLength, sheath, advanceRate,
            minInsert, maxInsert, lumenClearance, axialWindowScale, meshClearance
        });
        this.lumenSampler = typeof lumenSampler === 'function' ? lumenSampler : null;
        this.tailProgress = 0;
        this.lastAdvanceDelta = 0;
        this.contactPoints = [];
        this.breachPoints = [];
        this.previousPositions = null;
        this._advancePreviousPositions = null;
        this.performanceStats = createPerformanceStats();
        this._diagnosticContact = createContactScratch();
        const axis = {
            x: sheath.end.x - sheath.start.x,
            y: sheath.end.y - sheath.start.y,
            z: sheath.end.z - sheath.start.z
        };
        this.sheathLength = Math.hypot(axis.x, axis.y, axis.z) || 1;
        this.sheathDir = normalizeVector(axis, { x: 1, y: 0, z: 0 });
        this.externalTailStart = {
            x: sheath.start.x - this.sheathDir.x * guidewireLength,
            y: sheath.start.y - this.sheathDir.y * guidewireLength,
            z: sheath.start.z - this.sheathDir.z * guidewireLength
        };
        this._lumenConstraintState = {
            projected: { x: 0, y: 0, z: 0 },
            radialMargin: 0, axialOffset: 0, axialWindow: 0, breach: false
        };
    }

    reset() {
        this.tailProgress = this.minInsert;
        this.lastAdvanceDelta = 0;
        this.contactPoints.length = 0;
        this.breachPoints.length = 0;
        this.initialize();
        return this;
    }

    // Only the prescribed proximal/sheath boundary receives operator feed.
    // The shared direct Kirchhoff world owns all free-material dynamics.
    advance(command, dt) {
        resetPerformanceStats(this.performanceStats);
        const startedAt = nowMs();
        const previous = snapshotNodePositions(this.rod.nodes, this._advancePreviousPositions);
        this._advancePreviousPositions = previous;
        const next = clamp(this.tailProgress + command * this.advanceRate * dt, this.minInsert, this.maxInsert);
        const delta = next - this.tailProgress;
        this.tailProgress = next;
        this.lastAdvanceDelta = delta;
        const feedSpeed = delta / Math.max(dt, 1e-6);
        this.constrainSheath(feedSpeed);
        this.previousPositions = previous;
        this.performanceStats.advanceMs = nowMs() - startedAt;
        this.performanceStats.moving = Math.abs(delta) > 1e-6;
        this.performanceStats.boundaryDrivenFeed = Math.abs(delta) > 1e-6;
        this.performanceStats.transportDeltaMm = delta;
        this.performanceStats.transportSpeedMmPerSecond = feedSpeed;
        return delta;
    }
    get progress() {
        return this.tailProgress;
    }

    getPerformanceStats() {
        return { ...this.performanceStats };
    }

    initialize() {
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, vx, vy, vz, pinned } = storage;
            for (let i = 0; i < this.rod.nodes.length; i++) {
                const distance = this.segmentLength * i;
                x[i] = this.externalTailStart.x + this.sheathDir.x * distance;
                y[i] = this.externalTailStart.y + this.sheathDir.y * distance;
                z[i] = this.externalTailStart.z + this.sheathDir.z * distance;
                vx[i] = 0;
                vy[i] = 0;
                vz[i] = 0;
                pinned[i] = 1;
            }
        } else {
            for (let i = 0; i < this.rod.nodes.length; i++) {
                const distance = this.segmentLength * i;
                const node = this.rod.nodes[i];
                node.x = this.externalTailStart.x + this.sheathDir.x * distance;
                node.y = this.externalTailStart.y + this.sheathDir.y * distance;
                node.z = this.externalTailStart.z + this.sheathDir.z * distance;
                node.vx = node.vy = node.vz = 0;
                node.pinned = true;
            }
        }
        this.constrainSheath();
        this.previousPositions = snapshotNodePositions(this.rod.nodes, this._advancePreviousPositions);
        this._advancePreviousPositions = this.previousPositions;
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

    // The native coupled chart needs material strictly before the valve. A
    // rounded ceil can leave its candidate only an ulp before coordinate 0;
    // union-grid reconciliation would then erase that entire overlap. Retain
    // the preceding source segment in that case. This only selects support;
    // it does not round material labels, move nodes, or change feed distance.
    firstProximalSupportNodeIndex() {
        let index = clamp(this.firstInsertedNodeIndex() - 1, 0, this.rod.nodes.length - 2);
        const resolution = 64 * Number.EPSILON * Math.max(1,
            Math.abs(this.guidewireLength), Math.abs(this.tailProgress),
            Math.abs(this.segmentLength * index));
        while (index > 0 && this.insertedCoordinate(index) >= -resolution) index--;
        return index;
    }

    #firstNodeOutsideSheathIndex() {
        return clamp(
            Math.floor(
                (this.sheathLength + SHEATH_BOUNDARY_EPSILON + this.guidewireLength - this.tailProgress) /
                this.segmentLength
            ) + 1,
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
        if (!this.lumenSampler) {
            return {
                point: this.sheathAxisPoint(inserted),
                tangent: { ...this.sheathDir },
                radius: Infinity
            };
        }
        return this.lumenSampler(Math.max(0, inserted - this.sheathLength));
    }

    constrainSheath(feedSpeed = 0) {
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, vx, vy, vz, pinned } = storage;
            const firstOutside = this.#firstNodeOutsideSheathIndex();
            for (let i = 0; i < firstOutside; i++) {
                const inserted = this.insertedCoordinate(i);
                pinned[i] = 1;
                x[i] = this.sheath.start.x + this.sheathDir.x * inserted;
                y[i] = this.sheath.start.y + this.sheathDir.y * inserted;
                z[i] = this.sheath.start.z + this.sheathDir.z * inserted;
                vx[i] = this.sheathDir.x * feedSpeed;
                vy[i] = this.sheathDir.y * feedSpeed;
                vz[i] = this.sheathDir.z * feedSpeed;
            }
            pinned.fill(0, firstOutside);
            return;
        }

        for (let i = 0; i < this.rod.nodes.length; i++) {
            const inserted = this.insertedCoordinate(i);
            const node = this.rod.nodes[i];
            const inSheath = this.#isInSheath(inserted);
            node.pinned = inSheath;
            if (!inSheath) continue;

            node.x = this.sheath.start.x + this.sheathDir.x * inserted;
            node.y = this.sheath.start.y + this.sheathDir.y * inserted;
            node.z = this.sheath.start.z + this.sheathDir.z * inserted;
            node.vx = this.sheathDir.x * feedSpeed;
            node.vy = this.sheathDir.y * feedSpeed;
            node.vz = this.sheathDir.z * feedSpeed;
        }
    }

    collectContactSamples(collisionTarget = null, contactBand = DEFAULT_CONTACT_BAND) {
        const contacts = [];
        const breaches = [];
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

    collectLumenDiagnostics(
        collisionTarget = null,
        {
            clearance = this.meshClearance,
            contactBand = DEFAULT_CONTACT_BAND,
            samples = DEFAULT_DIAGNOSTIC_SAMPLES,
            collectMarkers = false,
            markerLimit = 420
        } = {}
    ) {
        const diagnosticStart = nowMs();
        this.performanceStats.diagnosticMs = 0;
        this.performanceStats.diagnosticPointContactCount = 0;
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        const result = {
            checkedCount: 0,
            contactCount: 0,
            outsideCount: 0,
            clearanceViolationCount: 0,
            minSignedDistance: null,
            minClearanceMargin: null,
            worstPoint: null,
            worstInserted: null,
            maxSegmentError: 0,
            maxBendAngle: 0,
            clearance,
            contactBand,
            contacts: collectMarkers ? [] : null,
            breaches: collectMarkers ? [] : null
        };

        for (let i = 0; i < this.rod.nodes.length - 1; i++) {
            const n0 = this.rod.nodes[i];
            const n1 = this.rod.nodes[i + 1];
            result.maxSegmentError = Math.max(
                result.maxSegmentError,
                Math.abs(nodeDistance(n0, n1) - this.segmentLength)
            );

            if (collider?.pointContact) {
                for (const t of samples) {
                    const inserted = this.insertedCoordinate(i + t);
                    if (this.#isInSheath(inserted)) continue;

                    const point = interpolatePosition(n0, n1, t);
                    const contact = this.#pointContact(collider, point, clearance, this._diagnosticContact);
                    const signedDistance = Number.isFinite(contact?.signedDistance)
                        ? contact.signedDistance
                        : null;
                    if (!Number.isFinite(signedDistance)) continue;

                    result.checkedCount++;
                    if (
                        result.minSignedDistance === null ||
                        signedDistance < result.minSignedDistance
                    ) {
                        result.minSignedDistance = signedDistance;
                        result.worstPoint = { x: point.x, y: point.y, z: point.z };
                        result.worstInserted = inserted;
                    }

                    const clearanceMargin = signedDistance - clearance;
                    if (
                        result.minClearanceMargin === null ||
                        clearanceMargin < result.minClearanceMargin
                    ) {
                        result.minClearanceMargin = clearanceMargin;
                    }

                    if (signedDistance < 0) {
                        result.outsideCount++;
                        if (collectMarkers) this.#pushLimited(result.breaches, point, markerLimit);
                    } else if (signedDistance <= contactBand) {
                        result.contactCount++;
                        if (collectMarkers) this.#pushLimited(result.contacts, point, markerLimit);
                    }
                    if (signedDistance < clearance) {
                        result.clearanceViolationCount++;
                    }
                }
            }
        }

        if (typeof this.rod.bendAngleAt === 'function') {
            for (let i = 1; i < this.rod.nodes.length - 1; i++) {
                const inserted = this.insertedCoordinate(i);
                if (this.#isInSheath(inserted)) continue;
                result.maxBendAngle = Math.max(result.maxBendAngle, this.rod.bendAngleAt(i) || 0);
            }
        }

        this.performanceStats.diagnosticMs = nowMs() - diagnosticStart;
        return result;
    }

    diagnosePoint(point, inserted, collisionTarget = null, contactBand = DEFAULT_CONTACT_BAND) {
        const lumen = this.lumenSampler ? this.#lumenConstraint(point, inserted) : null;
        let contact = lumen
            ? lumen.radialMargin <= contactBand || Math.abs(lumen.axialOffset) >= lumen.axialWindow - contactBand
            : false;
        let breach = lumen?.breach || false;
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        if (collider?.pointContact && !this.#isInSheath(inserted)) {
            const meshContact = this.#pointContact(collider, point, 0, this._diagnosticContact);
            breach = breach || !!meshContact?.violation;
            contact = contact || (
                !meshContact?.violation &&
                Number.isFinite(meshContact?.distance) &&
                meshContact.distance <= contactBand
            );
        }

        return { contact: !breach && contact, breach };
    }

    #pointContact(collider, point, clearance, out = null) {
        this.performanceStats.diagnosticPointContactCount++;
        return collider.pointContact(point, clearance, out);
    }

    #lumenConstraint(point, inserted) {
        const sample = this.routeSample(inserted);
        const radius = Math.max(0.5, (sample.radius || 1) - this.lumenClearance);
        const tangentSource = sample.tangent || this.sheathDir;
        const tangentLength = Math.hypot(tangentSource.x, tangentSource.y, tangentSource.z);
        const tangentX = tangentLength < 1e-8 ? this.sheathDir.x : tangentSource.x / tangentLength;
        const tangentY = tangentLength < 1e-8 ? this.sheathDir.y : tangentSource.y / tangentLength;
        const tangentZ = tangentLength < 1e-8 ? this.sheathDir.z : tangentSource.z / tangentLength;
        const dx = point.x - sample.point.x;
        const dy = point.y - sample.point.y;
        const dz = point.z - sample.point.z;
        let axialOffset = dx * tangentX + dy * tangentY + dz * tangentZ;
        let lateralX = dx - tangentX * axialOffset;
        let lateralY = dy - tangentY * axialOffset;
        let lateralZ = dz - tangentZ * axialOffset;
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

        const state = this._lumenConstraintState;
        state.projected.x = sample.point.x + tangentX * axialOffset + lateralX;
        state.projected.y = sample.point.y + tangentY * axialOffset + lateralY;
        state.projected.z = sample.point.z + tangentZ * axialOffset + lateralZ;
        state.radialMargin = radius - lateralLength;
        state.axialOffset = axialOffset;
        state.axialWindow = axialWindow;
        state.breach = breach;
        return state;
    }

    #pushLimited(points, point, limit = 420) {
        if (points.length >= limit) return;
        points.push({ x: point.x, y: point.y, z: point.z });
    }

    #isInSheath(inserted) {
        return inserted <= this.sheathLength + SHEATH_BOUNDARY_EPSILON;
    }
}
