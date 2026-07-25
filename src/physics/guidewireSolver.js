import * as THREE from 'three';
import { clamp, smoothstep } from '../mathUtils.js';
import {
    addPointCorrection,
    clearPointBuffer,
    ensurePointBuffer,
    snapshotNodePositions
} from './pointBuffer.js';

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
const DEFAULT_MESH_CLEARANCE = 0.45;
const DEFAULT_COLLISION_PROJECTION_REPEATS = 2;
const DEFAULT_DIAGNOSTIC_SAMPLES = [0, 0.2, 0.4, 0.6, 0.8, 1];
const DEFAULT_FOLD_ANGLE = 142;
const DEFAULT_FOLD_UNTANGLE_STRENGTH = 0;
const DEFAULT_FOLD_UNTANGLE_WINDOW = 5;
const DEFAULT_FINAL_COLLISION_PASSES = 10;
const DEFAULT_FINAL_LENGTH_PASSES = 6;
const DEFAULT_FINAL_PROJECTION_PASSES = 2;
const DEFAULT_FOLD_GUARD_ANGLE = 128;
const DEFAULT_FOLD_GUARD_STRENGTH = 0.38;
const DEFAULT_FOLD_GUARD_PASSES = 2;
const DEFAULT_FOLD_GUARD_CENTER_PULL = 1.1;
const DEFAULT_STABILITY_REPAIR_SEGMENT_ERROR = 0.1;
const DEFAULT_STABILITY_REPAIR_BEND_ANGLE = 170;
const DEFAULT_STABILITY_REPAIR_TARGET_BEND_ANGLE = 140;
const DEFAULT_STABILITY_REPAIR_PASSES = 3;
const DEFAULT_STABILITY_REPAIR_LENGTH_ITERATIONS = 10;
const DEFAULT_TIP_BACKTRACK_ANGLE = 108;
const DEFAULT_TIP_BACKTRACK_STRENGTH = 1;
const DEFAULT_WITHDRAWAL_STRAIGHTENING = 0.34;
const DEFAULT_WITHDRAWAL_STRAIGHTENING_PASSES = 2;
const DEFAULT_WITHDRAWAL_RELAX_FRAMES = 96;
const DEFAULT_UNSUPPORTED_BEND_RELAX_ANGLE = 10;
const DEFAULT_UNSUPPORTED_BEND_SUPPORT_BAND = 0.35;
const DEFAULT_UNSUPPORTED_BEND_RELAX_FRAMES = 18;
const SHEATH_BOUNDARY_EPSILON = 1e-3;

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

function nowMs() {
    return globalThis.performance?.now?.() ?? Date.now();
}

function createPerformanceStats() {
    return {
        advanceMs: 0,
        solveMs: 0,
        projectMs: 0,
        diagnosticMs: 0,
        pointContactCount: 0,
        diagnosticPointContactCount: 0,
        projectGuidewireCalls: 0,
        nodeProjectionCount: 0,
        segmentProjectionCount: 0,
        segmentSampleCount: 0,
        solveIterations: 0,
        moving: false,
        boundaryDrivenFeed: false,
        forceRelax: false,
        foldGuarded: false,
        stabilityRepaired: false,
        withdrawalRelaxed: false
    };
}

function resetPerformanceStats(stats) {
    stats.advanceMs = 0;
    stats.solveMs = 0;
    stats.projectMs = 0;
    stats.diagnosticMs = 0;
    stats.pointContactCount = 0;
    stats.diagnosticPointContactCount = 0;
    stats.projectGuidewireCalls = 0;
    stats.nodeProjectionCount = 0;
    stats.segmentProjectionCount = 0;
    stats.segmentSampleCount = 0;
    stats.solveIterations = 0;
    stats.moving = false;
    stats.boundaryDrivenFeed = false;
    stats.forceRelax = false;
    stats.foldGuarded = false;
    stats.stabilityRepaired = false;
    stats.withdrawalRelaxed = false;
    return stats;
}

export class GuidewireSolver {
    constructor({
        rod,
        segmentLength,
        guidewireLength,
        sheath,
        lumenSampler = null,
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
        meshClearance = DEFAULT_MESH_CLEARANCE,
        collisionProjectionRepeats = DEFAULT_COLLISION_PROJECTION_REPEATS,
        foldAngle = DEFAULT_FOLD_ANGLE,
        foldUntangleStrength = DEFAULT_FOLD_UNTANGLE_STRENGTH,
        foldUntangleWindow = DEFAULT_FOLD_UNTANGLE_WINDOW,
        finalCollisionPasses = DEFAULT_FINAL_COLLISION_PASSES,
        finalLengthPasses = DEFAULT_FINAL_LENGTH_PASSES,
        finalProjectionPasses = DEFAULT_FINAL_PROJECTION_PASSES,
        foldGuardAngle = DEFAULT_FOLD_GUARD_ANGLE,
        foldGuardStrength = DEFAULT_FOLD_GUARD_STRENGTH,
        foldGuardPasses = DEFAULT_FOLD_GUARD_PASSES,
        foldGuardCenterPull = DEFAULT_FOLD_GUARD_CENTER_PULL,
        stabilityRepairSegmentError = DEFAULT_STABILITY_REPAIR_SEGMENT_ERROR,
        stabilityRepairBendAngle = DEFAULT_STABILITY_REPAIR_BEND_ANGLE,
        stabilityRepairTargetBendAngle = DEFAULT_STABILITY_REPAIR_TARGET_BEND_ANGLE,
        stabilityRepairPasses = DEFAULT_STABILITY_REPAIR_PASSES,
        stabilityRepairLengthIterations = DEFAULT_STABILITY_REPAIR_LENGTH_ITERATIONS,
        tipBacktrackAngle = DEFAULT_TIP_BACKTRACK_ANGLE,
        tipBacktrackStrength = DEFAULT_TIP_BACKTRACK_STRENGTH,
        withdrawalStraightening = DEFAULT_WITHDRAWAL_STRAIGHTENING,
        withdrawalStraighteningPasses = DEFAULT_WITHDRAWAL_STRAIGHTENING_PASSES,
        withdrawalRelaxFrames = DEFAULT_WITHDRAWAL_RELAX_FRAMES,
        unsupportedBendRelaxAngle = DEFAULT_UNSUPPORTED_BEND_RELAX_ANGLE,
        unsupportedBendSupportBand = DEFAULT_UNSUPPORTED_BEND_SUPPORT_BAND,
        unsupportedBendRelaxFrames = DEFAULT_UNSUPPORTED_BEND_RELAX_FRAMES
    }) {
        this.rod = rod;
        this.segmentLength = segmentLength;
        this.guidewireLength = guidewireLength;
        this.sheath = sheath;
        this.lumenSampler = typeof lumenSampler === 'function' ? lumenSampler : null;
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
        this.meshClearance = meshClearance;
        this.collisionProjectionRepeats = Math.max(1, Math.floor(collisionProjectionRepeats));
        this.foldAngle = foldAngle;
        this.foldUntangleStrength = foldUntangleStrength;
        this.foldUntangleWindow = foldUntangleWindow;
        this.finalCollisionPasses = finalCollisionPasses;
        this.finalLengthPasses = finalLengthPasses;
        this.finalProjectionPasses = finalProjectionPasses;
        this.foldGuardAngle = foldGuardAngle;
        this.foldGuardStrength = foldGuardStrength;
        this.foldGuardPasses = foldGuardPasses;
        this.foldGuardCenterPull = foldGuardCenterPull;
        this.stabilityRepairSegmentError = stabilityRepairSegmentError;
        this.stabilityRepairBendAngle = stabilityRepairBendAngle;
        this.stabilityRepairTargetBendAngle = stabilityRepairTargetBendAngle;
        this.stabilityRepairPasses = stabilityRepairPasses;
        this.stabilityRepairLengthIterations = stabilityRepairLengthIterations;
        this.tipBacktrackAngle = tipBacktrackAngle;
        this.tipBacktrackStrength = tipBacktrackStrength;
        this.withdrawalStraightening = withdrawalStraightening;
        this.withdrawalStraighteningPasses = withdrawalStraighteningPasses;
        this.withdrawalRelaxFrames = withdrawalRelaxFrames;
        this.unsupportedBendRelaxAngle = unsupportedBendRelaxAngle;
        this.unsupportedBendSupportBand = unsupportedBendSupportBand;
        this.unsupportedBendRelaxFrames = unsupportedBendRelaxFrames;
        this.tailProgress = 0;
        this.lastAdvanceDelta = 0;
        this.settleFramesRemaining = 0;
        this.withdrawalRelaxFramesRemaining = 0;
        this.unsupportedBendRelaxFramesRemaining = 0;
        this.unsupportedBendRelaxArmed = true;
        this.contactPoints = [];
        this.breachPoints = [];
        this.previousPositions = null;
        this.performanceStats = createPerformanceStats();
        this._advancePreviousPositions = null;
        this._solvePreviousPositions = null;
        this._straightenCorrections = null;
        this._spanCorrections = null;
        this._untangleCorrections = null;
        this._bendLimitCorrections = null;
        this._diagnosticContact = createContactScratch();
        this._supportContact = createContactScratch();
        this._projectContact = createContactScratch();
        this._slidePointContact = createContactScratch();
        this._slideTargetContact = createContactScratch();
        this._zeroVelocityContact = createContactScratch();
        this._projectNodePoint = { x: 0, y: 0, z: 0 };
        this._convectSource = { x: 0, y: 0, z: 0 };
        this._lumenConstraintState = {
            projected: { x: 0, y: 0, z: 0 },
            radialMargin: 0,
            axialOffset: 0,
            axialWindow: 0,
            breach: false
        };

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

    getPerformanceStats() {
        return { ...this.performanceStats };
    }

    reset() {
        this.tailProgress = this.minInsert;
        this.lastAdvanceDelta = 0;
        this.settleFramesRemaining = 0;
        this.withdrawalRelaxFramesRemaining = 0;
        this.unsupportedBendRelaxFramesRemaining = 0;
        this.unsupportedBendRelaxArmed = true;
        this.contactPoints.length = 0;
        this.breachPoints.length = 0;
        this.initialize();
        return this;
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

    #firstSegmentOutsideSheathIndex() {
        return Math.max(0, this.#firstNodeOutsideSheathIndex() - 1);
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

    advance(
        command,
        dt,
        collisionTarget = null,
        { routeAssist = true, boundaryDriven = false } = {}
    ) {
        resetPerformanceStats(this.performanceStats);
        const perfStart = nowMs();
        const previous = snapshotNodePositions(this.rod.nodes, this._advancePreviousPositions);
        this._advancePreviousPositions = previous;
        const nextProgress = clamp(
            this.tailProgress + command * this.advanceRate * dt,
            this.minInsert,
            this.maxInsert
        );
        const delta = nextProgress - this.tailProgress;
        this.tailProgress = nextProgress;
        this.lastAdvanceDelta = delta;
        if (Math.abs(delta) > 1e-6) {
            this.requestSettle();
            this.unsupportedBendRelaxArmed = true;
            this.unsupportedBendRelaxFramesRemaining = 0;
        }
        if (delta < -1e-6) {
            this.withdrawalRelaxFramesRemaining = Math.max(
                this.withdrawalRelaxFramesRemaining,
                Math.max(0, Math.floor(this.withdrawalRelaxFrames))
            );
        }
        const feedSpeed = delta / Math.max(dt, 1e-6);

        this.constrainSheath(feedSpeed);
        if (Math.abs(delta) > 1e-6 && !boundaryDriven) {
            this.#convectMaterial(delta, previous, dt, collisionTarget, routeAssist);
        }

        this.previousPositions = previous;
        this.performanceStats.advanceMs += nowMs() - perfStart;
        this.performanceStats.moving = Math.abs(delta) > 1e-6;
        this.performanceStats.boundaryDrivenFeed =
            boundaryDriven && Math.abs(delta) > 1e-6;
        return delta;
    }

    requestSettle(frames = 48) {
        this.settleFramesRemaining = Math.max(this.settleFramesRemaining, frames);
    }

    solve(dt, collisionTarget = null, { iterations = this.relaxationIterations, forceRelax = false } = {}) {
        const solveStart = nowMs();
        this.performanceStats.forceRelax = this.performanceStats.forceRelax || !!forceRelax;
        const before = snapshotNodePositions(this.rod.nodes, this._solvePreviousPositions);
        this._solvePreviousPositions = before;
        this.contactPoints.length = 0;
        this.breachPoints.length = 0;

        this.constrainSheath();
        const advancing = this.lastAdvanceDelta > 1e-6;
        const recentlyWithdrawing = this.lastAdvanceDelta < -1e-6 || this.withdrawalRelaxFramesRemaining > 0;
        let unsupportedFreeBend = false;
        if (
            !advancing &&
            (this.unsupportedBendRelaxArmed || this.unsupportedBendRelaxFramesRemaining > 0)
        ) {
            unsupportedFreeBend = this.#hasUnsupportedFreeBend(collisionTarget);
            if (unsupportedFreeBend && this.unsupportedBendRelaxArmed) {
                this.unsupportedBendRelaxFramesRemaining = Math.max(
                    this.unsupportedBendRelaxFramesRemaining,
                    Math.max(1, Math.floor(this.unsupportedBendRelaxFrames))
                );
                this.unsupportedBendRelaxArmed = false;
            } else if (!unsupportedFreeBend) {
                this.unsupportedBendRelaxFramesRemaining = 0;
                this.unsupportedBendRelaxArmed = true;
            }
        }
        const unsupportedFreeBendRelaxing = unsupportedFreeBend &&
            this.unsupportedBendRelaxFramesRemaining > 0;
        const shouldRelax = forceRelax ||
            Math.abs(this.lastAdvanceDelta) > 1e-6 ||
            this.settleFramesRemaining > 0 ||
            recentlyWithdrawing ||
            unsupportedFreeBendRelaxing;
        if (!shouldRelax) {
            this.#zeroVelocities(before, dt, collisionTarget);
            this.performanceStats.solveMs += nowMs() - solveStart;
            return;
        }

        if (Math.abs(this.lastAdvanceDelta) > 1e-6) {
            this.#routeNudge();
            this.#limitTipBacktracking(collisionTarget);
        }
        const shapeRelaxing = recentlyWithdrawing || unsupportedFreeBendRelaxing;
        if (shapeRelaxing) {
            this.performanceStats.withdrawalRelaxed = this.#relaxWithdrawalShape(
                collisionTarget,
                recentlyWithdrawing ? 1 : 0.72
            ) ||
                this.performanceStats.withdrawalRelaxed;
        }

        const passCount = Math.max(1, iterations);
        this.performanceStats.solveIterations += passCount;
        for (let pass = 0; pass < passCount; pass++) {
            this.#straightenInsideVessel(pass / passCount, collisionTarget);
            if (shapeRelaxing && pass < 2) {
                this.performanceStats.withdrawalRelaxed = this.#relaxWithdrawalShape(
                    collisionTarget,
                    recentlyWithdrawing ? 1 : 0.72
                ) ||
                    this.performanceStats.withdrawalRelaxed;
            }
            this.#untangleFoldedSections();
            for (let repeat = 0; repeat < this.collisionProjectionRepeats; repeat++) {
                this.#limitBendsInsideVessel();
                this.#solveLengths(this.lengthIterations);
                this.#projectGuidewireInside(collisionTarget, false);
            }
            this.#solveLengths(2);
            this.#projectGuidewireInside(collisionTarget, false);
        }

        this.constrainSheath();
        for (let pass = 0; pass < this.finalCollisionPasses; pass++) {
            this.#untangleFoldedSections();
            this.#limitBendsInsideVessel();
            this.#projectGuidewireInside(collisionTarget, false);
            this.#solveLengths(this.lengthIterations + 2);
        }
        this.#solveLengths(this.lengthIterations + 4);
        for (let pass = 0; pass < this.finalLengthPasses; pass++) {
            this.#projectGuidewireInside(collisionTarget, false);
            this.#solveLengths(5);
        }
        for (let pass = 0; pass < this.finalProjectionPasses; pass++) {
            this.#projectGuidewireInside(collisionTarget, false);
        }
        this.#limitTipBacktracking(collisionTarget);
        this.#solveLengths(Math.max(2, Math.ceil(this.lengthIterations * 0.4)));
        let foldGuarded = this.#guardAgainstHairpinFolds();
        this.performanceStats.foldGuarded = this.performanceStats.foldGuarded || foldGuarded;
        this.#projectGuidewireInside(collisionTarget, false);
        if (foldGuarded || this.#hasBendOver(this.foldGuardAngle)) {
            foldGuarded = this.#guardAgainstHairpinFolds() || foldGuarded;
            this.performanceStats.foldGuarded = this.performanceStats.foldGuarded || foldGuarded;
            this.#solveLengths(this.lengthIterations + 4);
            this.#projectGuidewireInside(collisionTarget, false);
            this.#solveLengths(4);
            this.#projectGuidewireInside(collisionTarget, false);
        }
        if (this.#repairGuidewireStability(collisionTarget)) {
            this.performanceStats.stabilityRepaired = true;
            this.performanceStats.foldGuarded = true;
        }
        this.#zeroVelocities(before, dt, collisionTarget);
        if (Math.abs(this.lastAdvanceDelta) <= 1e-6 && this.settleFramesRemaining > 0) {
            this.settleFramesRemaining--;
        }
        if (this.lastAdvanceDelta >= -1e-6 && this.withdrawalRelaxFramesRemaining > 0) {
            this.withdrawalRelaxFramesRemaining--;
        }
        if (this.unsupportedBendRelaxFramesRemaining > 0) {
            this.unsupportedBendRelaxFramesRemaining--;
            if (this.unsupportedBendRelaxFramesRemaining <= 0 && !unsupportedFreeBend) {
                this.unsupportedBendRelaxArmed = true;
            }
        }
        this.performanceStats.solveMs += nowMs() - solveStart;
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
                    const contact = this.#pointContact(collider, point, clearance, true, this._diagnosticContact);
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
            const meshContact = this.#pointContact(collider, point, 0, true, this._diagnosticContact);
            breach = breach || !!meshContact?.violation;
            contact = contact || (
                !meshContact?.violation &&
                Number.isFinite(meshContact?.distance) &&
                meshContact.distance <= contactBand
            );
        }

        return { contact: !breach && contact, breach };
    }

    #convectMaterial(delta, previous, dt, collisionTarget = null, routeAssist = true) {
        const sourceShift = delta / this.segmentLength;
        const invDt = 1 / Math.max(dt, 1e-6);
        const collider = routeAssist ? this.#collisionCollider(collisionTarget) : null;

        const startIndex = this.#firstNodeOutsideSheathIndex();
        for (let i = startIndex; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            const inserted = this.insertedCoordinate(i);
            if (this.#isInSheath(inserted)) continue;

            const source = this.#samplePreviousPosition(
                previous,
                i + sourceShift,
                routeAssist ? collisionTarget : null,
                this._convectSource,
                routeAssist
            );
            if (!routeAssist) {
                const old = previous[i];
                setNode(node, source);
                node.vx = (source.x - old.x) * invDt * 0.2;
                node.vy = (source.y - old.y) * invDt * 0.2;
                node.vz = (source.z - old.z) * invDt * 0.2;
                continue;
            }
            const route = this.lumenSampler ? this.routeSample(inserted).point : source;
            const justExited = inserted < this.sheathLength + this.segmentLength * 2;
            const blend = this.lumenSampler
                ? (justExited ? 0.64 : 0.12)
                : 0;
            const target = {
                x: source.x * (1 - blend) + route.x * blend,
                y: source.y * (1 - blend) + route.y * blend,
                z: source.z * (1 - blend) + route.z * blend
            };

            const old = previous[i];
            const slide = this.#slideVectorAlongCollider(old, {
                x: target.x - old.x,
                y: target.y - old.y,
                z: target.z - old.z
            }, collider);
            target.x = old.x + slide.x;
            target.y = old.y + slide.y;
            target.z = old.z + slide.z;
            const constrained = this.#projectPointInside(target, inserted, collisionTarget, false);
            setNode(node, constrained);
            node.vx = (constrained.x - old.x) * invDt * 0.2;
            node.vy = (constrained.y - old.y) * invDt * 0.2;
            node.vz = (constrained.z - old.z) * invDt * 0.2;
        }
    }

    #samplePreviousPosition(
        previous,
        sourceIndex,
        collisionTarget = null,
        out = { x: 0, y: 0, z: 0 },
        routeAssist = true
    ) {
        const lastIndex = previous.length - 1;
        if (sourceIndex <= 0) {
            const head = previous[0];
            out.x = head.x + this.sheathDir.x * sourceIndex * this.segmentLength;
            out.y = head.y + this.sheathDir.y * sourceIndex * this.segmentLength;
            out.z = head.z + this.sheathDir.z * sourceIndex * this.segmentLength;
            return out;
        }

        if (sourceIndex < lastIndex) {
            const lower = Math.floor(sourceIndex);
            const upper = Math.min(lastIndex, lower + 1);
            const t = sourceIndex - lower;
            const a = previous[lower];
            const b = previous[upper];
            out.x = a.x + (b.x - a.x) * t;
            out.y = a.y + (b.y - a.y) * t;
            out.z = a.z + (b.z - a.z) * t;
            return out;
        }

        const tip = previous[lastIndex];
        const prev = previous[Math.max(0, lastIndex - 1)];
        let directionX = tip.x - prev.x;
        let directionY = tip.y - prev.y;
        let directionZ = tip.z - prev.z;
        const directionLength = Math.sqrt(
            directionX * directionX + directionY * directionY + directionZ * directionZ
        );
        if (directionLength > 1e-8) {
            directionX /= directionLength;
            directionY /= directionLength;
            directionZ /= directionLength;
        } else if (routeAssist) {
            const fallback = this.routeSample(this.tailProgress).tangent;
            directionX = fallback.x;
            directionY = fallback.y;
            directionZ = fallback.z;
        } else {
            directionX = this.sheathDir.x;
            directionY = this.sheathDir.y;
            directionZ = this.sheathDir.z;
        }
        let tangentX = directionX;
        let tangentY = directionY;
        let tangentZ = directionZ;
        const collider = routeAssist ? this.#collisionCollider(collisionTarget) : null;
        if (collider) {
            const tangent = normalizeVector(
                this.#slideVectorAlongCollider(tip, {
                    x: directionX,
                    y: directionY,
                    z: directionZ
                }, collider),
                { x: directionX, y: directionY, z: directionZ }
            );
            tangentX = tangent.x;
            tangentY = tangent.y;
            tangentZ = tangent.z;
        }
        const distance = (sourceIndex - lastIndex) * this.segmentLength;
        out.x = tip.x + tangentX * distance;
        out.y = tip.y + tangentY * distance;
        out.z = tip.z + tangentZ * distance;
        return out;
    }

    #routeNudge(multiplier = 1) {
        if (this.routeBlend <= 0 || !this.lumenSampler) return;
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, pinned } = storage;
            const startIndex = this.#firstNodeOutsideSheathIndex();
            for (let i = startIndex; i < this.rod.nodes.length; i++) {
                const inserted = this.insertedCoordinate(i);
                if (this.#isInSheath(inserted)) continue;

                const sample = this.routeSample(inserted);
                const fade = smoothstep(this.sheathLength, this.sheathLength + this.segmentLength * 8, inserted);
                const blend = this.routeBlend * multiplier * (0.35 + 0.65 * fade);
                x[i] += (sample.point.x - x[i]) * blend;
                y[i] += (sample.point.y - y[i]) * blend;
                z[i] += (sample.point.z - z[i]) * blend;
            }
            return;
        }
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

    #isNodeWallSupported(index, collisionTarget = null) {
        const node = this.rod.nodes[index];
        if (!node) return true;
        const inserted = this.insertedCoordinate(index);
        if (this.#isInSheath(inserted)) return true;

        const collider = this.#collisionCollider(collisionTarget);
        if (!collider?.pointContact) return false;

        const contact = this.#pointContact(collider, node, this.meshClearance, false, this._supportContact);
        return !!contact?.violation || (
            Number.isFinite(contact?.signedDistance) &&
            contact.signedDistance <= this.meshClearance + this.unsupportedBendSupportBand
        );
    }

    #hasUnsupportedFreeBend(collisionTarget = null) {
        const threshold = Math.max(0, this.unsupportedBendRelaxAngle);
        if (threshold <= 0 || typeof this.rod.bendAngleAt !== 'function') return false;

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;

            const angle = this.rod.bendAngleAt(i) || 0;
            if (angle <= threshold) continue;

            const distalUnsupported = !this.#isNodeWallSupported(i + 1, collisionTarget);
            const bendUnsupported = !this.#isNodeWallSupported(i, collisionTarget);
            if (distalUnsupported || bendUnsupported) return true;
        }

        return false;
    }

    #applyRelaxedMove(node, inserted, move, collisionTarget, collider, maxStep) {
        let relaxedMove = move;
        if (collider?.pointContact && !this.#isInSheath(inserted)) {
            relaxedMove = this.#slideVectorAlongCollider(node, relaxedMove, collider);
        }

        const moveLength = Math.hypot(relaxedMove.x, relaxedMove.y, relaxedMove.z);
        if (moveLength <= this.segmentLength * 0.002) return false;
        if (moveLength > maxStep) {
            const scale = maxStep / moveLength;
            relaxedMove = {
                x: relaxedMove.x * scale,
                y: relaxedMove.y * scale,
                z: relaxedMove.z * scale
            };
        }

        const constrained = this.#projectPointInside({
            x: node.x + relaxedMove.x,
            y: node.y + relaxedMove.y,
            z: node.z + relaxedMove.z
        }, inserted, collisionTarget, false);
        setNode(node, constrained);
        return true;
    }

    #relaxWithdrawalShape(collisionTarget = null, strengthScale = 1) {
        const passes = Math.max(0, Math.floor(this.withdrawalStraighteningPasses));
        const baseStrength = clamp(this.withdrawalStraightening * strengthScale, 0, 1);
        if (passes <= 0 || baseStrength <= 0) return false;

        const collider = this.#collisionCollider(collisionTarget);
        const residualWithdrawalScale = this.withdrawalRelaxFramesRemaining > 0 ? 0.55 : 0.25;
        const speedScale = clamp(
            Math.abs(this.lastAdvanceDelta) / Math.max(1e-6, this.segmentLength * 0.25),
            residualWithdrawalScale,
            1
        );
        const maxStep = this.segmentLength * 0.16;
        const propagationWindow = Math.max(1, Math.ceil(56 / Math.max(1e-6, this.segmentLength)));
        let applied = false;

        for (let pass = 0; pass < passes; pass++) {
            for (let i = 1; i < this.rod.nodes.length; i++) {
                const node = this.rod.nodes[i];
                if (node.pinned) continue;

                const inserted = this.insertedCoordinate(i);
                if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;

                const prev = this.rod.nodes[i - 1];
                const prevPrev = this.rod.nodes[i - 2];
                const incoming = prevPrev
                    ? normalizeVector({
                        x: prev.x - prevPrev.x,
                        y: prev.y - prevPrev.y,
                        z: prev.z - prevPrev.z
                    }, this.sheathDir)
                    : this.sheathDir;
                const current = normalizeVector({
                    x: node.x - prev.x,
                    y: node.y - prev.y,
                    z: node.z - prev.z
                }, incoming);
                const dot = clamp(
                    incoming.x * current.x + incoming.y * current.y + incoming.z * current.z,
                    -1,
                    1
                );
                const bendWeight = clamp((1 - dot) / 0.28, 0, 1);
                if (bendWeight <= 1e-3) continue;

                const target = {
                    x: prev.x + incoming.x * this.segmentLength,
                    y: prev.y + incoming.y * this.segmentLength,
                    z: prev.z + incoming.z * this.segmentLength
                };
                const strength = baseStrength * speedScale * (0.25 + 0.75 * bendWeight);
                let move = {
                    x: (target.x - node.x) * strength,
                    y: (target.y - node.y) * strength,
                    z: (target.z - node.z) * strength
                };
                const rawLength = Math.hypot(move.x, move.y, move.z);
                if (rawLength <= 1e-8) continue;

                if (collider?.pointContact) {
                    const slidMove = this.#slideVectorAlongCollider(node, move, collider);
                    const slidLength = Math.hypot(slidMove.x, slidMove.y, slidMove.z);
                    if (slidLength < rawLength * 0.08) {
                        const pullDirection = normalizeVector({
                            x: prev.x - node.x,
                            y: prev.y - node.y,
                            z: prev.z - node.z
                        }, {
                            x: -incoming.x,
                            y: -incoming.y,
                            z: -incoming.z
                        });
                        const pullStep = Math.min(maxStep, Math.max(rawLength * 0.45, this.segmentLength * 0.035));
                        const pullMove = this.#slideVectorAlongCollider(node, {
                            x: pullDirection.x * pullStep,
                            y: pullDirection.y * pullStep,
                            z: pullDirection.z * pullStep
                        }, collider);
                        move = Math.hypot(pullMove.x, pullMove.y, pullMove.z) > slidLength
                            ? pullMove
                            : slidMove;
                    } else {
                        move = slidMove;
                    }
                }

                for (let j = i; j < this.rod.nodes.length && j < i + propagationWindow; j++) {
                    const targetNode = this.rod.nodes[j];
                    if (!targetNode || targetNode.pinned) break;
                    const targetInserted = this.insertedCoordinate(j);
                    if (this.#isInSheath(targetInserted)) break;
                    if (j > i && this.#isNodeWallSupported(j, collisionTarget)) break;

                    const moved = this.#applyRelaxedMove(
                        targetNode,
                        targetInserted,
                        move,
                        collisionTarget,
                        collider,
                        maxStep
                    );
                    applied = applied || moved;
                }
            }
        }

        return applied;
    }

    #preparePointBuffer(slot) {
        const buffer = ensurePointBuffer(this[slot], this.rod.nodes.length);
        this[slot] = buffer;
        clearPointBuffer(buffer);
        return buffer;
    }

    #straightenInsideVessel(progress, collisionTarget = null) {
        const corrections = this.#preparePointBuffer('_straightenCorrections');
        const endpointBias = 0.35 + 0.65 * smoothstep(0, 1, progress);
        const collider = this.#collisionCollider(collisionTarget);
        const maxStep = this.segmentLength * 0.18;
        const correctionTarget = { x: 0, y: 0, z: 0 };
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, pinned } = storage;
            const startIndex = this.#firstNodeOutsideSheathIndex();
            const applyCorrection = (index, inserted, correction) => {
                let moveX = correction.x;
                let moveY = correction.y;
                let moveZ = correction.z;
                if (collider?.pointContact && !this.#isInSheath(inserted)) {
                    const slidMove = this.#slideVectorAlongCollider(this.rod.nodes[index], correction, collider);
                    moveX = slidMove.x;
                    moveY = slidMove.y;
                    moveZ = slidMove.z;
                }
                const length = Math.hypot(moveX, moveY, moveZ);
                if (length > maxStep) {
                    const scale = maxStep / length;
                    moveX *= scale;
                    moveY *= scale;
                    moveZ *= scale;
                }
                correctionTarget.x = x[index] + moveX;
                correctionTarget.y = y[index] + moveY;
                correctionTarget.z = z[index] + moveZ;
                return this.#projectPointInside(correctionTarget, inserted, collisionTarget, false);
            };

            for (let i = Math.max(1, startIndex); i < this.rod.nodes.length - 1; i++) {
                if (pinned[i]) continue;

                const inserted = this.insertedCoordinate(i);
                const nearExit = 1 - smoothstep(this.sheathLength, this.sheathLength + this.segmentLength * 5, inserted);
                const strength = this.straightening * endpointBias * (1 - nearExit * 0.45);
                const correction = corrections[i];
                correction.x = ((x[i - 1] + x[i + 1]) * 0.5 - x[i]) * strength;
                correction.y = ((y[i - 1] + y[i + 1]) * 0.5 - y[i]) * strength;
                correction.z = ((z[i - 1] + z[i + 1]) * 0.5 - z[i]) * strength;
                correction.active = true;
            }

            for (let i = Math.max(1, startIndex); i < this.rod.nodes.length - 1; i++) {
                const correction = corrections[i];
                if (!correction.active || pinned[i]) continue;
                const inserted = this.insertedCoordinate(i);
                const constrained = applyCorrection(i, inserted, correction);
                x[i] = constrained.x;
                y[i] = constrained.y;
                z[i] = constrained.z;
            }

            const spans = [2, 4, 8, 12];
            for (const span of spans) {
                const spanCorrections = this.#preparePointBuffer('_spanCorrections');
                const spanStrength = this.straightening * 0.13 / Math.sqrt(span);
                for (let i = Math.max(span, startIndex); i < this.rod.nodes.length - span; i++) {
                    if (pinned[i]) continue;
                    const correction = spanCorrections[i];
                    correction.x = ((x[i - span] + x[i + span]) * 0.5 - x[i]) * spanStrength;
                    correction.y = ((y[i - span] + y[i + span]) * 0.5 - y[i]) * spanStrength;
                    correction.z = ((z[i - span] + z[i + span]) * 0.5 - z[i]) * spanStrength;
                    correction.active = true;
                }
                for (let i = Math.max(span, startIndex); i < this.rod.nodes.length - span; i++) {
                    const correction = spanCorrections[i];
                    if (!correction.active || pinned[i]) continue;
                    const inserted = this.insertedCoordinate(i);
                    const constrained = applyCorrection(i, inserted, correction);
                    x[i] = constrained.x;
                    y[i] = constrained.y;
                    z[i] = constrained.z;
                }
            }
            return;
        }
        const applyCorrection = (node, inserted, correction) => {
            let moveX = correction.x;
            let moveY = correction.y;
            let moveZ = correction.z;
            if (collider?.pointContact && !this.#isInSheath(inserted)) {
                const slidMove = this.#slideVectorAlongCollider(node, correction, collider);
                moveX = slidMove.x;
                moveY = slidMove.y;
                moveZ = slidMove.z;
            }
            const length = Math.hypot(moveX, moveY, moveZ);
            if (length > maxStep) {
                const scale = maxStep / length;
                moveX *= scale;
                moveY *= scale;
                moveZ *= scale;
            }
            correctionTarget.x = node.x + moveX;
            correctionTarget.y = node.y + moveY;
            correctionTarget.z = node.z + moveZ;
            return this.#projectPointInside(correctionTarget, inserted, collisionTarget, false);
        };

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;

            const prev = this.rod.nodes[i - 1];
            const next = this.rod.nodes[i + 1];
            const inserted = this.insertedCoordinate(i);
            const nearExit = 1 - smoothstep(this.sheathLength, this.sheathLength + this.segmentLength * 5, inserted);
            const strength = this.straightening * endpointBias * (1 - nearExit * 0.45);
            const correction = corrections[i];
            correction.x = ((prev.x + next.x) * 0.5 - node.x) * strength;
            correction.y = ((prev.y + next.y) * 0.5 - node.y) * strength;
            correction.z = ((prev.z + next.z) * 0.5 - node.z) * strength;
            correction.active = true;
        }

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            const correction = corrections[i];
            if (!correction.active || node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            const constrained = applyCorrection(node, inserted, correction);
            setNode(node, constrained);
        }

        const spans = [2, 4, 8, 12];
        for (const span of spans) {
            const spanCorrections = this.#preparePointBuffer('_spanCorrections');
            const spanStrength = this.straightening * 0.13 / Math.sqrt(span);
            for (let i = span; i < this.rod.nodes.length - span; i++) {
                const node = this.rod.nodes[i];
                if (node.pinned) continue;
                const prev = this.rod.nodes[i - span];
                const next = this.rod.nodes[i + span];
                const correction = spanCorrections[i];
                correction.x = ((prev.x + next.x) * 0.5 - node.x) * spanStrength;
                correction.y = ((prev.y + next.y) * 0.5 - node.y) * spanStrength;
                correction.z = ((prev.z + next.z) * 0.5 - node.z) * spanStrength;
                correction.active = true;
            }
            for (let i = span; i < this.rod.nodes.length - span; i++) {
                const node = this.rod.nodes[i];
                const correction = spanCorrections[i];
                if (!correction.active || node.pinned) continue;
                const inserted = this.insertedCoordinate(i);
                const constrained = applyCorrection(node, inserted, correction);
                setNode(node, constrained);
            }
        }
    }

    #untangleFoldedSections() {
        if (this.foldUntangleStrength <= 0 || this.foldUntangleWindow <= 0) return;
        const corrections = this.#preparePointBuffer('_untangleCorrections');
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
                addPointCorrection(
                    corrections,
                    index,
                    (route.x - targetNode.x) * strength,
                    (route.y - targetNode.y) * strength,
                    (route.z - targetNode.z) * strength
                );
            }
        }

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            const correction = corrections[i];
            if (!correction.active || node.pinned) continue;
            addScaled(node, correction, 1);
        }
    }

    #limitBendsInsideVessel(
        maxBendAngle = this.maxBendAngle,
        bendLimitStrength = this.bendLimitStrength,
        bendLimitIterations = this.bendLimitIterations,
        centerPull = 0.45
    ) {
        if (bendLimitIterations <= 0 || bendLimitStrength <= 0) return;
        const clampedMaxAngle = clamp(maxBendAngle, 1, 179);
        const limit = clampedMaxAngle * Math.PI / 180;
        const minChord = 2 * this.segmentLength * Math.cos(limit * 0.5);
        const baseStrength = clamp(bendLimitStrength, 0, 1);

        for (let iter = 0; iter < bendLimitIterations; iter++) {
            const corrections = this.#preparePointBuffer('_bendLimitCorrections');
            for (let i = 1; i < this.rod.nodes.length - 1; i++) {
                const node = this.rod.nodes[i];
                if (node.pinned) continue;
                const inserted = this.insertedCoordinate(i);
                if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;

                const angleDegrees = this.rod.bendAngleAt?.(i) ?? 0;
                if (angleDegrees <= clampedMaxAngle) continue;

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
                const severity = clamp((angleDegrees - clampedMaxAngle) / Math.max(1, 180 - clampedMaxAngle), 0, 1);
                const strength = baseStrength * (0.35 + 0.65 * severity);

                if (chordLength < minChord) {
                    const spread = (minChord - chordLength) * 0.5 * strength;
                    if (!prev.pinned) {
                        addPointCorrection(
                            corrections,
                            i - 1,
                            -direction.x * spread,
                            -direction.y * spread,
                            -direction.z * spread
                        );
                    }
                    if (!next.pinned) {
                        addPointCorrection(
                            corrections,
                            i + 1,
                            direction.x * spread,
                            direction.y * spread,
                            direction.z * spread
                        );
                    }
                }

                addPointCorrection(
                    corrections,
                    i,
                    ((prev.x + next.x) * 0.5 - node.x) * strength * centerPull,
                    ((prev.y + next.y) * 0.5 - node.y) * strength * centerPull,
                    ((prev.z + next.z) * 0.5 - node.z) * strength * centerPull
                );
            }

            for (let i = 1; i < this.rod.nodes.length - 1; i++) {
                const node = this.rod.nodes[i];
                const correction = corrections[i];
                if (!correction.active || node.pinned) continue;
                addScaled(node, correction, 1);
            }
        }
    }

    #hasBendOver(angleThreshold) {
        const threshold = clamp(angleThreshold, 1, 179);
        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;
            const angle = this.rod.bendAngleAt?.(i) ?? 0;
            if (angle > threshold) return true;
        }
        return false;
    }

    #maxSegmentError() {
        let maxError = 0;
        for (let i = 0; i < this.rod.nodes.length - 1; i++) {
            const n0 = this.rod.nodes[i];
            const n1 = this.rod.nodes[i + 1];
            maxError = Math.max(maxError, Math.abs(nodeDistance(n0, n1) - this.segmentLength));
        }
        return maxError;
    }

    #maxBendAngleOutsideSheath() {
        let maxAngle = 0;
        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;
            maxAngle = Math.max(maxAngle, this.rod.bendAngleAt?.(i) ?? 0);
        }
        return maxAngle;
    }

    #orthogonalFallback(direction) {
        const ax = Math.abs(direction.x);
        const ay = Math.abs(direction.y);
        const basis = ax < 0.7
            ? { x: 1, y: 0, z: 0 }
            : ay < 0.7
                ? { x: 0, y: 1, z: 0 }
                : { x: 0, y: 0, z: 1 };
        return normalizeVector({
            x: direction.y * basis.z - direction.z * basis.y,
            y: direction.z * basis.x - direction.x * basis.z,
            z: direction.x * basis.y - direction.y * basis.x
        }, { x: 1, y: 0, z: 0 });
    }

    #limitHairpinDirections(maxBendAngle, strength) {
        const angle = clamp(maxBendAngle, 1, 179) * Math.PI / 180;
        const minDot = Math.cos(angle);
        const sinLimit = Math.sin(angle);
        const amount = clamp(strength, 0, 1);
        if (amount <= 0) return;

        for (let i = 1; i < this.rod.nodes.length - 1; i++) {
            const prev = this.rod.nodes[i - 1];
            const curr = this.rod.nodes[i];
            const next = this.rod.nodes[i + 1];
            if (curr.pinned || next.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) continue;

            const incoming = normalizeVector({
                x: curr.x - prev.x,
                y: curr.y - prev.y,
                z: curr.z - prev.z
            }, this.sheathDir);
            const outgoingRaw = {
                x: next.x - curr.x,
                y: next.y - curr.y,
                z: next.z - curr.z
            };
            const outgoing = normalizeVector(outgoingRaw, incoming);
            const dot = incoming.x * outgoing.x + incoming.y * outgoing.y + incoming.z * outgoing.z;
            if (dot >= minDot) continue;

            let lateral = {
                x: outgoing.x - incoming.x * dot,
                y: outgoing.y - incoming.y * dot,
                z: outgoing.z - incoming.z * dot
            };
            const lateralLength = Math.hypot(lateral.x, lateral.y, lateral.z);
            if (lateralLength < 1e-8) {
                lateral = this.#orthogonalFallback(incoming);
            } else {
                lateral.x /= lateralLength;
                lateral.y /= lateralLength;
                lateral.z /= lateralLength;
            }

            const desired = normalizeVector({
                x: incoming.x * minDot + lateral.x * sinLimit,
                y: incoming.y * minDot + lateral.y * sinLimit,
                z: incoming.z * minDot + lateral.z * sinLimit
            }, incoming);
            const target = {
                x: curr.x + desired.x * this.segmentLength,
                y: curr.y + desired.y * this.segmentLength,
                z: curr.z + desired.z * this.segmentLength
            };
            next.x += (target.x - next.x) * amount;
            next.y += (target.y - next.y) * amount;
            next.z += (target.z - next.z) * amount;
        }
    }

    #limitTipBacktracking(collisionTarget = null) {
        const amount = clamp(this.tipBacktrackStrength, 0, 1);
        if (amount <= 0 || this.rod.nodes.length < 3) return false;

        const tipIndex = this.rod.nodes.length - 1;
        const pivotIndex = tipIndex - 1;
        const prevIndex = tipIndex - 2;
        const tip = this.rod.nodes[tipIndex];
        const pivot = this.rod.nodes[pivotIndex];
        const prev = this.rod.nodes[prevIndex];
        if (tip.pinned || pivot.pinned) return false;

        const inserted = this.insertedCoordinate(pivotIndex);
        if (inserted <= this.sheathLength + this.segmentLength + SHEATH_BOUNDARY_EPSILON) return false;

        const maxAngle = clamp(this.tipBacktrackAngle, 1, 179) * Math.PI / 180;
        const minDot = Math.cos(maxAngle);
        const sinLimit = Math.sin(maxAngle);
        const incoming = normalizeVector({
            x: pivot.x - prev.x,
            y: pivot.y - prev.y,
            z: pivot.z - prev.z
        }, this.sheathDir);
        const outgoing = normalizeVector({
            x: tip.x - pivot.x,
            y: tip.y - pivot.y,
            z: tip.z - pivot.z
        }, incoming);
        const dot = incoming.x * outgoing.x + incoming.y * outgoing.y + incoming.z * outgoing.z;
        if (dot >= minDot) return false;

        let lateral = {
            x: outgoing.x - incoming.x * dot,
            y: outgoing.y - incoming.y * dot,
            z: outgoing.z - incoming.z * dot
        };
        const lateralLength = Math.hypot(lateral.x, lateral.y, lateral.z);
        if (lateralLength < 1e-8) {
            lateral = this.#orthogonalFallback(incoming);
        } else {
            lateral.x /= lateralLength;
            lateral.y /= lateralLength;
            lateral.z /= lateralLength;
        }

        const desired = normalizeVector({
            x: incoming.x * minDot + lateral.x * sinLimit,
            y: incoming.y * minDot + lateral.y * sinLimit,
            z: incoming.z * minDot + lateral.z * sinLimit
        }, incoming);
        const collider = this.#collisionCollider(collisionTarget);
        let targetPoint = {
            x: pivot.x + desired.x * this.segmentLength,
            y: pivot.y + desired.y * this.segmentLength,
            z: pivot.z + desired.z * this.segmentLength
        };
        if (collider?.pointContact) {
            const slide = normalizeVector(this.#slideVectorAlongCollider(tip, incoming, collider), desired);
            const advanced = this.#projectPointInside({
                x: tip.x + slide.x * this.segmentLength * 0.8,
                y: tip.y + slide.y * this.segmentLength * 0.8,
                z: tip.z + slide.z * this.segmentLength * 0.8
            }, this.insertedCoordinate(tipIndex), collisionTarget, false);
            const advancedDirection = normalizeVector({
                x: advanced.x - pivot.x,
                y: advanced.y - pivot.y,
                z: advanced.z - pivot.z
            }, desired);
            const advancedDot = incoming.x * advancedDirection.x +
                incoming.y * advancedDirection.y +
                incoming.z * advancedDirection.z;
            if (advancedDot > dot) {
                targetPoint = advanced;
            }
        }
        const target = this.#projectPointInside(targetPoint, this.insertedCoordinate(tipIndex), collisionTarget, false);

        tip.x += (target.x - tip.x) * amount;
        tip.y += (target.y - tip.y) * amount;
        tip.z += (target.z - tip.z) * amount;
        tip.vx *= 0.2;
        tip.vy *= 0.2;
        tip.vz *= 0.2;
        return true;
    }

    #guardAgainstHairpinFolds() {
        if (this.foldGuardPasses <= 0 || this.foldGuardStrength <= 0) return false;
        if (!this.#hasBendOver(this.foldGuardAngle)) return false;

        const passes = Math.max(1, Math.floor(this.foldGuardPasses));
        let applied = false;
        for (let pass = 0; pass < passes; pass++) {
            applied = true;
            this.#limitHairpinDirections(
                this.foldGuardAngle,
                this.foldGuardStrength
            );
            this.#limitBendsInsideVessel(
                this.foldGuardAngle,
                this.foldGuardStrength,
                1,
                this.foldGuardCenterPull
            );
            this.#solveLengths(Math.max(3, Math.ceil(this.lengthIterations * 0.5)));
            this.#solveLengths(2);
            if (!this.#hasBendOver(this.foldGuardAngle)) break;
        }
        return applied;
    }

    #repairGuidewireStability(collisionTarget) {
        const passes = Math.max(0, Math.floor(this.stabilityRepairPasses));
        if (passes <= 0) return false;

        const segmentThreshold = Math.max(1e-4, this.stabilityRepairSegmentError);
        const bendThreshold = clamp(this.stabilityRepairBendAngle, 1, 179);
        const extraLengthIterations = Math.max(0, Math.floor(this.stabilityRepairLengthIterations));
        let applied = false;

        for (let pass = 0; pass < passes; pass++) {
            const segmentError = this.#maxSegmentError();
            const bendAngle = this.#maxBendAngleOutsideSheath();
            if (segmentError <= segmentThreshold && bendAngle <= bendThreshold) break;

            applied = true;
            const segmentSeverity = clamp(segmentError / segmentThreshold - 1, 0, 1);
            const bendSeverity = clamp((bendAngle - bendThreshold) / Math.max(1, 180 - bendThreshold), 0, 1);
            const severity = Math.max(segmentSeverity, bendSeverity);
            const guardAngle = clamp(
                Math.min(this.stabilityRepairTargetBendAngle, bendThreshold),
                1,
                179
            );
            const guardStrength = clamp(
                Math.max(this.foldGuardStrength, 0.72) * (0.75 + 0.25 * severity),
                0,
                1
            );
            const centerPull = Math.max(this.foldGuardCenterPull, 1.1);

            this.#limitHairpinDirections(guardAngle, guardStrength);
            this.#limitBendsInsideVessel(guardAngle, guardStrength, 2, centerPull);
            this.#solveLengths(this.lengthIterations + extraLengthIterations, collisionTarget, true);
            this.#projectGuidewireInside(collisionTarget, false);
            this.#limitBendsInsideVessel(guardAngle, guardStrength, 1, centerPull);
            this.#solveLengths(
                this.lengthIterations + Math.ceil(extraLengthIterations * 0.5),
                collisionTarget,
                true
            );
            this.#projectGuidewireInside(collisionTarget, false);
            this.#solveLengths(Math.max(4, Math.ceil(this.lengthIterations * 0.5)), collisionTarget, true);
        }

        return applied;
    }

    #solveLengths(iterations, collisionTarget = null, slideAgainstCollider = false) {
        const rest = this.segmentLength;
        const collider = slideAgainstCollider ? this.#collisionCollider(collisionTarget) : null;
        const shouldSlide = !!collider?.pointContact;
        const startIndex = this.#firstSegmentOutsideSheathIndex();
        const storage = this.rod.nodes.nodeStorage;
        if (!shouldSlide && storage) {
            const { x, y, z, pinned } = storage;
            for (let iter = 0; iter < iterations; iter++) {
                for (let i = startIndex; i < this.rod.nodes.length - 1; i++) {
                    const dx = x[i + 1] - x[i];
                    const dy = y[i + 1] - y[i];
                    const dz = z[i + 1] - z[i];
                    const dist = Math.hypot(dx, dy, dz);
                    if (dist < 1e-8) continue;

                    const correction = (dist - rest) / dist;
                    const w0 = pinned[i] ? 0 : 1;
                    const w1 = pinned[i + 1] ? 0 : 1;
                    const total = w0 + w1;
                    if (total <= 0) continue;

                    const c0 = w0 / total;
                    const c1 = w1 / total;
                    if (w0) {
                        const scale = correction * c0;
                        x[i] += dx * scale;
                        y[i] += dy * scale;
                        z[i] += dz * scale;
                    }
                    if (w1) {
                        const scale = -correction * c1;
                        x[i + 1] += dx * scale;
                        y[i + 1] += dy * scale;
                        z[i + 1] += dz * scale;
                    }
                }
                this.constrainSheath();
            }
            return;
        }
        for (let iter = 0; iter < iterations; iter++) {
            for (let i = startIndex; i < this.rod.nodes.length - 1; i++) {
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
                if (!shouldSlide) {
                    if (w0) {
                        const scale = correction * c0;
                        n0.x += dx * scale;
                        n0.y += dy * scale;
                        n0.z += dz * scale;
                    }
                    if (w1) {
                        const scale = -correction * c1;
                        n1.x += dx * scale;
                        n1.y += dy * scale;
                        n1.z += dz * scale;
                    }
                    continue;
                }

                if (w0) {
                    let move = {
                        x: dx * correction * c0,
                        y: dy * correction * c0,
                        z: dz * correction * c0
                    };
                    if (!this.#isInSheath(this.insertedCoordinate(i))) {
                        move = this.#slideVectorAlongCollider(n0, move, collider);
                    }
                    n0.x += move.x;
                    n0.y += move.y;
                    n0.z += move.z;
                }
                if (w1) {
                    let move = {
                        x: -dx * correction * c1,
                        y: -dy * correction * c1,
                        z: -dz * correction * c1
                    };
                    if (!this.#isInSheath(this.insertedCoordinate(i + 1))) {
                        move = this.#slideVectorAlongCollider(n1, move, collider);
                    }
                    n1.x += move.x;
                    n1.y += move.y;
                    n1.z += move.z;
                }
            }
            this.constrainSheath();
        }
    }

    #projectGuidewireInside(collisionTarget, recordContacts) {
        const projectStart = nowMs();
        this.performanceStats.projectGuidewireCalls++;
        this.#projectNodesInside(collisionTarget, recordContacts);
        this.#projectSegmentsInside(collisionTarget, recordContacts);
        this.#projectNodesInside(collisionTarget, recordContacts);
        this.performanceStats.projectMs += nowMs() - projectStart;
    }

    #projectNodesInside(collisionTarget, recordContacts) {
        const startIndex = this.#firstNodeOutsideSheathIndex();
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, pinned } = storage;
            const point = this._projectNodePoint;
            for (let i = startIndex; i < this.rod.nodes.length; i++) {
                if (pinned[i]) continue;
                const inserted = this.insertedCoordinate(i);
                if (this.#isInSheath(inserted)) continue;
                this.performanceStats.nodeProjectionCount++;
                point.x = x[i];
                point.y = y[i];
                point.z = z[i];
                const projected = this.#projectPointInside(point, inserted, collisionTarget, recordContacts);
                x[i] = projected.x;
                y[i] = projected.y;
                z[i] = projected.z;
            }
            return;
        }
        for (let i = startIndex; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            if (node.pinned) continue;
            const inserted = this.insertedCoordinate(i);
            if (this.#isInSheath(inserted)) continue;
            this.performanceStats.nodeProjectionCount++;
            const projected = this.#projectPointInside(node, inserted, collisionTarget, recordContacts);
            setNode(node, projected);
        }
    }

    #projectSegmentsInside(collisionTarget, recordContacts) {
        const point = { x: 0, y: 0, z: 0 };
        const maxCorrection = this.segmentLength * this.maxSegmentProjectionStep;
        const startIndex = this.#firstSegmentOutsideSheathIndex();
        const storage = this.rod.nodes.nodeStorage;
        if (storage) {
            const { x, y, z, pinned } = storage;
            for (let i = startIndex; i < this.rod.nodes.length - 1; i++) {
                if (pinned[i] && pinned[i + 1]) continue;

                for (const t of this.segmentSamples) {
                    const inserted = this.insertedCoordinate(i + t);
                    if (this.#isInSheath(inserted)) continue;
                    this.performanceStats.segmentSampleCount++;
                    const w0 = pinned[i] ? 0 : 1 - t;
                    const w1 = pinned[i + 1] ? 0 : t;
                    point.x = x[i] * (1 - t) + x[i + 1] * t;
                    point.y = y[i] * (1 - t) + y[i + 1] * t;
                    point.z = z[i] * (1 - t) + z[i + 1] * t;
                    const target = this.#projectPointInside(point, inserted, collisionTarget, recordContacts);
                    let correctionX = (target.x - point.x) * this.segmentProjectionBlend;
                    let correctionY = (target.y - point.y) * this.segmentProjectionBlend;
                    let correctionZ = (target.z - point.z) * this.segmentProjectionBlend;
                    const correctionLength = Math.hypot(correctionX, correctionY, correctionZ);
                    if (correctionLength > maxCorrection) {
                        const scale = maxCorrection / correctionLength;
                        correctionX *= scale;
                        correctionY *= scale;
                        correctionZ *= scale;
                    }
                    const denom = w0 * w0 + w1 * w1;
                    if (denom <= 1e-8) continue;
                    this.performanceStats.segmentProjectionCount++;
                    if (w0) {
                        const scale = w0 / denom;
                        x[i] += correctionX * scale;
                        y[i] += correctionY * scale;
                        z[i] += correctionZ * scale;
                    }
                    if (w1) {
                        const scale = w1 / denom;
                        x[i + 1] += correctionX * scale;
                        y[i + 1] += correctionY * scale;
                        z[i + 1] += correctionZ * scale;
                    }
                }
            }
            return;
        }
        for (let i = startIndex; i < this.rod.nodes.length - 1; i++) {
            const n0 = this.rod.nodes[i];
            const n1 = this.rod.nodes[i + 1];
            if (n0.pinned && n1.pinned) continue;

            for (const t of this.segmentSamples) {
                const inserted = this.insertedCoordinate(i + t);
                if (this.#isInSheath(inserted)) continue;
                this.performanceStats.segmentSampleCount++;
                const w0 = n0.pinned ? 0 : 1 - t;
                const w1 = n1.pinned ? 0 : t;
                point.x = n0.x * (1 - t) + n1.x * t;
                point.y = n0.y * (1 - t) + n1.y * t;
                point.z = n0.z * (1 - t) + n1.z * t;
                const target = this.#projectPointInside(point, inserted, collisionTarget, recordContacts);
                let correctionX = (target.x - point.x) * this.segmentProjectionBlend;
                let correctionY = (target.y - point.y) * this.segmentProjectionBlend;
                let correctionZ = (target.z - point.z) * this.segmentProjectionBlend;
                const correctionLength = Math.hypot(correctionX, correctionY, correctionZ);
                if (correctionLength > maxCorrection) {
                    const scale = maxCorrection / correctionLength;
                    correctionX *= scale;
                    correctionY *= scale;
                    correctionZ *= scale;
                }
                const denom = w0 * w0 + w1 * w1;
                if (denom <= 1e-8) continue;
                this.performanceStats.segmentProjectionCount++;
                if (w0) {
                    const scale = w0 / denom;
                    n0.x += correctionX * scale;
                    n0.y += correctionY * scale;
                    n0.z += correctionZ * scale;
                }
                if (w1) {
                    const scale = w1 / denom;
                    n1.x += correctionX * scale;
                    n1.y += correctionY * scale;
                    n1.z += correctionZ * scale;
                }
            }
        }
    }

    #projectPointInside(point, inserted, collisionTarget, recordContacts) {
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        if (collider?.pointContact && !this.#isInSheath(inserted)) {
            let projected = point;
            if (this.lumenSampler) {
                const lumenState = this.#lumenConstraint(point, inserted);
                projected = lumenState.projected;
                if (recordContacts) {
                    if (lumenState.breach) this.#pushLimited(this.breachPoints, point);
                    else if (lumenState.radialMargin <= DEFAULT_CONTACT_BAND) this.#pushLimited(this.contactPoints, point);
                }
            }
            const contact = this.#pointContact(collider, projected, this.meshClearance, false, this._projectContact);
            if (contact?.violation && contact.target) {
                if (recordContacts) {
                    if (Number.isFinite(contact.signedDistance) && contact.signedDistance < 0) {
                        this.#pushLimited(this.breachPoints, projected);
                    } else {
                        this.#pushLimited(this.contactPoints, projected);
                    }
                }
                return { x: contact.target.x, y: contact.target.y, z: contact.target.z };
            } else if (recordContacts && Number.isFinite(contact?.distance) && contact.distance <= DEFAULT_CONTACT_BAND) {
                this.#pushLimited(this.contactPoints, projected);
            }
            return projected;
        }

        return this.#projectToLumen(point, inserted, recordContacts);
    }

    #collisionCollider(collisionTarget) {
        return collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
    }

    #pointContact(collider, point, clearance, diagnostic = false, out = null) {
        if (diagnostic) {
            this.performanceStats.diagnosticPointContactCount++;
        } else {
            this.performanceStats.pointContactCount++;
        }
        return collider.pointContact(point, clearance, out);
    }

    #slideVectorAlongCollider(point, vector, collider) {
        if (!collider?.pointContact) return { x: vector.x, y: vector.y, z: vector.z };

        const target = {
            x: point.x + vector.x,
            y: point.y + vector.y,
            z: point.z + vector.z
        };
        const pointContact = this.#pointContact(collider, point, this.meshClearance, false, this._slidePointContact);
        const targetContact = this.#pointContact(collider, target, this.meshClearance, false, this._slideTargetContact);
        const pointNearWall = pointContact?.violation || (
            Number.isFinite(pointContact?.signedDistance) &&
            pointContact.signedDistance <= this.meshClearance + DEFAULT_CONTACT_BAND
        );
        const targetNearWall = targetContact?.violation || (
            Number.isFinite(targetContact?.signedDistance) &&
            targetContact.signedDistance <= this.meshClearance + DEFAULT_CONTACT_BAND
        );
        if (!pointNearWall && !targetNearWall) {
            return { x: vector.x, y: vector.y, z: vector.z };
        }

        const contact = targetContact?.violation ? targetContact : (targetNearWall ? targetContact : pointContact);
        const normal = contact?.normal || pointContact?.normal || targetContact?.normal;
        const normalLength = normal ? Math.hypot(normal.x, normal.y, normal.z) : 0;
        if (normalLength < 1e-8) {
            return { x: vector.x, y: vector.y, z: vector.z };
        }

        const nx = normal.x / normalLength;
        const ny = normal.y / normalLength;
        const nz = normal.z / normalLength;
        const outward = vector.x * nx + vector.y * ny + vector.z * nz;
        if (outward <= 0) {
            return { x: vector.x, y: vector.y, z: vector.z };
        }

        return {
            x: vector.x - nx * outward,
            y: vector.y - ny * outward,
            z: vector.z - nz * outward
        };
    }

    #projectToLumen(point, inserted, recordContacts) {
        if (!this.lumenSampler) return point;
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

    #zeroVelocities(before, dt, collisionTarget = null) {
        const invDt = 1 / Math.max(dt, 1e-6);
        const collider = collisionTarget?.meshCollider || collisionTarget?.lumenMeshCollider || null;
        const storage = this.rod.nodes.nodeStorage;
        if (storage && !collider?.pointContact) {
            const { x, y, z, vx, vy, vz } = storage;
            for (let i = 0; i < this.rod.nodes.length; i++) {
                const dx = x[i] - before[i].x;
                const dy = y[i] - before[i].y;
                const dz = z[i] - before[i].z;
                const scale = dx * dx + dy * dy + dz * dz > 0.0004 ? 0.08 : 0;
                vx[i] = dx * invDt * scale;
                vy[i] = dy * invDt * scale;
                vz[i] = dz * invDt * scale;
            }
            return;
        }
        for (let i = 0; i < this.rod.nodes.length; i++) {
            const node = this.rod.nodes[i];
            const dx = node.x - before[i].x;
            const dy = node.y - before[i].y;
            const dz = node.z - before[i].z;
            const scale = dx * dx + dy * dy + dz * dz > 0.0004 ? 0.08 : 0;
            let vx = dx * invDt * scale;
            let vy = dy * invDt * scale;
            let vz = dz * invDt * scale;

            const inserted = this.insertedCoordinate(i);
            if (collider?.pointContact && !this.#isInSheath(inserted)) {
                const contact = this.#pointContact(collider, node, this.meshClearance, false, this._zeroVelocityContact);
                const normal = contact?.normal;
                const nearWall = contact?.violation || (
                    Number.isFinite(contact?.signedDistance) &&
                    contact.signedDistance <= this.meshClearance + DEFAULT_CONTACT_BAND
                );
                if (nearWall && normal) {
                    const outwardSpeed = vx * normal.x + vy * normal.y + vz * normal.z;
                    if (outwardSpeed > 0) {
                        vx -= normal.x * outwardSpeed;
                        vy -= normal.y * outwardSpeed;
                        vz -= normal.z * outwardSpeed;
                    }
                }
            }

            node.vx = vx;
            node.vy = vy;
            node.vz = vz;
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
