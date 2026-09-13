import { FRICTIONLESS_LUMEN_RESIDUAL } from './kirchhoffFrictionlessLumen.js';
import { TOOL_MAX_BEND_ANGLE_DEGREES } from './kirchhoffToolRuntime.js';
import { lineSearchLevel, createLineSearchStats, recordLineSearchTrial } from './kirchhoffLineSearch.js';
import { beginKirchhoffWallWitnessFrictionModes, evaluateKirchhoffWallWitnessFrictionCandidate, prepareKirchhoffWallWitnessFrictionRetry, commitKirchhoffWallWitnessFrictionModes } from './kirchhoffWallWitnessFrictionMode.js';
import { buildKirchhoffWallWitnessFriction, appendKirchhoffWallWitnessFriction, commitKirchhoffWallWitnessFriction, measureKirchhoffWallWitnessFriction } from './kirchhoffWallWitnessFriction.js';
import { captureKirchhoffWallDiscoveries, retainKirchhoffWallDiscoveries, beginKirchhoffWallWitnessStep, collectKirchhoffWallWitnessRows, commitKirchhoffWallWitnessMultipliers, measureKirchhoffWallWitnessResidual } from './kirchhoffWallWitnessRows.js';
import { selectKirchhoffMechanicalComponents } from './kirchhoffMechanicalComponents.js';
import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { boundaryRejectsKirchhoffTrial } from './kirchhoffTrialRejection.js';
import { buildKirchhoffPortalSideSamples } from './kirchhoffPortalSideSamples.js';
import { captureKirchhoffSplitStep, restoreKirchhoffSplitStep,
    kirchhoffSplitStepTopologyUnchanged } from './kirchhoffSplitStepTransaction.js';
import { buildKirchhoffSplitWallFriction, appendKirchhoffSplitWallFriction,
    commitKirchhoffSplitWallFriction, measureKirchhoffSplitWallFriction } from './kirchhoffSplitWallFriction.js';
import { captureKirchhoffWallFrictionIncoming, initializeKirchhoffWallFrictionModes,
    evaluateKirchhoffWallFrictionCandidate, prepareKirchhoffWallFrictionRetry } from './kirchhoffWallFrictionMode.js';
import { beginKirchhoffSplitMotion, captureKirchhoffSplitSweep, appendKirchhoffSplitSweeps, appendKirchhoffSplitPointWalls,
    prepareKirchhoffSplitLumenRows, prepareKirchhoffSplitBoundaryRows, applyKirchhoffSplitPhysicalIncrement,
    beginKirchhoffSplitBias, finishKirchhoffSplitBias, measureKirchhoffSplitMaterial,
    copyKirchhoffSplitVelocity, prescribeKirchhoffSplitOrientation, syncKirchhoffSplitVelocity,
    commitKirchhoffSplitHistory, getKirchhoffSplitMotionStats } from './kirchhoffSplitMotion.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion,
    commitKirchhoffTwoChannelBiasMaterial, measureKirchhoffTwoChannelMaterial } from './kirchhoffTwoChannelMotion.js';
import { beginKirchhoffTwoChannelRows, prepareKirchhoffTwoChannelRows,
    commitKirchhoffTwoChannelRows, measureKirchhoffTwoChannelRows } from './kirchhoffTwoChannelRows.js';
import { solveKirchhoffTwoChannelSystem, nextKirchhoffTwoChannelTolerance } from './kirchhoffTwoChannelSystem.js';
import { measureKirchhoffFrictionMerit } from './kirchhoffFrictionMerit.js';
import { beginKirchhoffCoupledBoundaryStep, collectKirchhoffCoupledBoundaryRows, applyKirchhoffCoupledBoundaryMultipliers, measureKirchhoffCoupledBoundaryResidual } from './kirchhoffCoupledBoundaryRows.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from './kirchhoffCoupledTrialState.js';
import { isKirchhoffDistalLumenWitness, locateKirchhoffDistalLumenBranch,
    beginKirchhoffToolReactionStep, appendKirchhoffToolRelease, hasKirchhoffToolReaction, captureKirchhoffToolReaction,
    measureKirchhoffToolReleaseRows, evaluateKirchhoffOwnedSlidingPortal } from './kirchhoffToolContactOwnership.js';
import { measureKirchhoffCoupledMaterialResidual } from './kirchhoffCoupledResidual.js';
import { prepareKirchhoffCoupledConeRepair, applyKirchhoffCoupledConeRepair } from './kirchhoffCoupledConeRepair.js';
import { beginKirchhoffCoupledOrientationStep, buildKirchhoffCoupledOrientationRows,
    appendKirchhoffCoupledOrientationRows, commitKirchhoffCoupledOrientationMultipliers,
    measureKirchhoffCoupledOrientationResidual } from './kirchhoffCoupledOrientationRows.js';
import { beginKirchhoffCoupledFoldStep, buildKirchhoffCoupledFoldRows,
    applyKirchhoffCoupledFoldMultipliers, measureKirchhoffCoupledFoldResidual } from './kirchhoffCoupledFoldRows.js';
import { buildKirchhoffCoupledFrictionRows, appendKirchhoffCoupledFrictionRows,
    commitKirchhoffCoupledFrictionMultipliers, measureKirchhoffCoupledFrictionResidual } from './kirchhoffCoupledFrictionRows.js';
import { buildKirchhoffContactNormalGradients } from './kirchhoffContactNormalRows.js';
import { beginKirchhoffExternalFrictionStep, buildKirchhoffExternalFrictionRows,
    appendKirchhoffExternalFrictionRows, commitKirchhoffExternalFrictionMultipliers,
    measureKirchhoffExternalFrictionResidual } from './kirchhoffExternalFrictionRows.js';
import { solveKirchhoffContactBlock } from './kirchhoffContactBlock.js';
import { createContactResult } from './collision/vesselContactField.js';
import {
    GUIDEWIRE_RADIUS_MM,
    INTRODUCER_SHEATH_INNER_DIAMETER_MM,
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_DIAMETER_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM
} from '../toolDimensions.js';
import { conjugateQuaternion, createBishopFrame, inverseRotateVectorByQuaternion, multiplyQuaternions, normalizeQuaternion, quaternionExp, quaternionLog, solveAdaptationXPBDArraySweep, solveBendTwistXPBD, transportBishopFrame } from './discreteKirchhoffRod.js';
import { KirchhoffContactManifold } from './kirchhoffContactManifold.js';
import {
    solveKirchhoffDirect
} from './kirchhoffDirectSolver.js';
import { evaluateKirchhoffSlidingPortal } from './kirchhoffSlidingPortal.js';
import { closestKirchhoffCenterlinePoint, prepareKirchhoffCenterlineSearch } from './kirchhoffCenterlineSearch.js';
import { measureKirchhoffContactMotion } from './kirchhoffCoupledConvergence.js';

const EPSILON = 1e-8;
const TRIG_SERIES_ANGLE_SQUARED = 0.0625;
const DEFAULT_FIXED_DT = 1 / 120;
const CONTACT_SIGNED_GAP = 1;
const CONTACT_PENETRATION = 3;
const CONTACT_BRANCH_ID = 4;
const CONTACT_SEGMENT_T = 5;
const MAX_WALL_CORRECTION_PASSES = 16;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function magnitude3(x, y, z) {
    return Math.sqrt(x * x + y * y + z * z);
}

function hasKirchhoffContactImpulse(contact, manifold) {
    return contact?._manifold === manifold && (contact.normalLambda !== 0 ||
        contact.tangentLambda[0] !== 0 || contact.tangentLambda[1] !== 0 || contact.twistLambda !== 0);
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
        last: 0,
        maximum: 0,
        maximumRecord: -1
    };
}

function recordTiming(timing, duration) {
    timing.last = duration;
    timing.total += duration;
    timing.recordedCount++;
    if (duration > timing.maximum) {
        timing.maximum = duration;
        timing.maximumRecord = timing.recordedCount - 1;
    }
    timing.samples[timing.cursor] = duration;
    timing.cursor = (timing.cursor + 1) % timing.samples.length;
    timing.count = Math.min(timing.samples.length, timing.count + 1);
}

function timingStats(timing) {
    return {
        lastMs: timing.last,
        averageMs: timing.recordedCount ? timing.total / timing.recordedCount : 0,
        p95Ms: percentile(timing.samples, timing.count, 0.95),
        maximumMs: timing.maximum,
        maximumRecord: timing.maximumRecord
    };
}

function validateWholeStepSystem(system) {
    if (system === null) return;
    if (!system || typeof system.id !== 'string' || !system.id.trim() ||
        typeof system.step !== 'function' || typeof system.reset !== 'function')
        throw new TypeError('wholeStepSystem requires a nonempty id and synchronous step(world, dt), reset(world) methods');
}

export const DEFAULT_TOOL_PROFILES = Object.freeze({
    guidewire: Object.freeze({
        id: 'guidewire',
        radius: GUIDEWIRE_RADIUS_MM,
        mass: 1,
        stretchCompliance: 2e-7,
        bendCompliance: 2e-5,
        minBendComplianceScale: 0.001953125,
        // A continuous metallic wire can flex, but it cannot form the nearly
        // reversed one-node hinge that a permissive numerical cap allowed at
        // a moving lumen boundary.
        maxBendAngle: TOOL_MAX_BEND_ANGLE_DEGREES,
        // RodState's historical 10-degree shaft cap is a positional solver
        // parameter, not a calibrated curvature/yield limit. Kirchhoff EI/GJ
        // controls bending; retain this profile's existing anti-fold guard.
        inheritRodStateBendLimit: false,
        foldLimitStrength: 1,
        wallFriction: 0.006,
        wallMaxCorrection: 0.2,
        // Vessel contact is a non-penetration projection, not an elastic
        // actuator. Keep the corrected position, but do not feed the direct
        // projection displacement back into the next step as material
        // momentum. The velocity solve below still removes forbidden outward
        // motion and preserves inward release plus tangential sliding.
        wallProjectionVelocityRetention: 0,
        sweptContactPreserveTangentialMotion: true,
        linearDamping: 0.98,

        // Operator feed is bounded independently by GuidewireTransport. A limit
        // on the complete nodal velocity also clips elastic recovery and wall
        // sliding, especially when the 44 mm/s feed already consumes almost
        // the whole former 45 mm/s budget. Swept contact and the unilateral
        // wall correction remain responsible for numerical safety.
        maxSpeed: Infinity,
        sleepVelocity: 1,
        sleepFrames: 10,
        sleepPenetration: 0.08
    }),
    catheter: Object.freeze({
        id: 'catheter',
        outerRadius: PIGTAIL_CATHETER_RADIUS_MM,
        innerDiameter: PIGTAIL_CATHETER_INNER_DIAMETER_MM,
        innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        radius: PIGTAIL_CATHETER_RADIUS_MM,
        mass: 1.4,
        stretchCompliance: 1e-7,
        // A 5 Fr catheter shaft must dominate the default 0.035 in wire. The
        // distal material profile selectively raises compliance again where a
        // preformed tip needs to flex and recover.
        bendCompliance: 1e-9,

        // Elastic bending stiffness is responsible for straightening the
        // shaft. This inequality is only an anti-fold safety guard. Keeping
        // it above an ordinary aortic turn avoids an over-constrained
        // length/fold cycle while still rejecting a one-node kink.
        maxBendAngle: TOOL_MAX_BEND_ANGLE_DEGREES,
        foldLimitStrength: 1,
        // The wall is a hard unilateral non-penetration constraint. Shape
        // memory and bending remain compliant, so a loaded catheter reaches
        // equilibrium by elastic deformation rather than crossing the wall.
        wallCompliance: 0,
        // Applied only to normal corrections generated in the current fixed
        // step. Tangential cached contact therefore remains free while a
        // genuinely wall-loaded catheter retains modest wet-contact damping.
        wallFriction: 0.002,
        wallMaxCorrection: 0.12,
        lumenFriction: 0.04,
        lumenAxialFriction: 0.015,
        lumenTorsionalFriction: 0.006,
        linearDamping: 0.9,

        // Feeding is already bounded by the physical inlet control. A second
        // per-node velocity clamp below the commanded feed rate compresses an
        // inextensible catheter at the introducer and creates a numerical
        // buckle, so it must not participate in the constitutive solve.
        maxSpeed: Infinity,
        sleepVelocity: 1,
        sleepFrames: 10,
        sleepPenetration: 0.08,
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
    constructor(id, count, segmentLength, profile = {}, motionMode = 'position-history') {
        // Split physical/bias motion has a separate numerical history contract;
        // its experimental path keeps its existing storage in this change.
        const CoordinateArray = motionMode === 'split-physical-bias' ? Float32Array : Float64Array;
        if (!Number.isInteger(count) || count < 2) throw new RangeError('A rod requires at least two nodes');
        this.id = id;
        // Material calibration is selected explicitly, independently of the solver.

        this.count = count;
        this.segmentCount = count - 1;
        this.segmentLength = segmentLength;
        this.radius = profile.radius ?? 0.5;
        this.innerRadius = profile.innerRadius ?? 0;
        this.mass = profile.mass ?? 1;
        this.stretchCompliance = profile.stretchCompliance ?? 2e-7;
        this.bendCompliance = profile.bendCompliance ?? 1e-3;
        this.minBendComplianceScale = profile.minBendComplianceScale ?? 0.125;

        this.maxBendAngle = profile.maxBendAngle ?? 135;
        this.inheritRodStateBendLimit = profile.inheritRodStateBendLimit ?? true;
        this.foldLimitStrength = profile.foldLimitStrength ?? 0.7;
        this.wallCompliance = profile.wallCompliance ?? 0;
        this.wallMaxCorrection = profile.wallMaxCorrection ?? Infinity;
        this.wallFriction = profile.wallFriction ?? 0.08;
        this.wallStaticFriction = profile.wallStaticFriction ?? this.wallFriction;
        this.wallKineticFriction = profile.wallKineticFriction ?? this.wallFriction;
        this.wallFrictionUsesCurrentLoad = profile.wallFrictionUsesCurrentLoad ?? false;
        this.wallFrictionUsesSmoothedLoad =
            profile.wallFrictionUsesSmoothedLoad ?? false;
        this.wallProjectionVelocityRetention = clamp(
            profile.wallProjectionVelocityRetention ?? 1,
            0,
            1
        );
        // Lumen and tool-tool non-penetration are wet, effectively
        // zero-restitution contacts. Keep their positional action-reaction
        // in the equilibrium solve, but do not reinterpret the projection as
        // a fresh launch velocity on the next fixed step.
        this.toolProjectionVelocityRetention = clamp(
            profile.toolProjectionVelocityRetention ??
                (0),
            0,
            1
        );
        this.sweptContactPreserveTangentialMotion =
            profile.sweptContactPreserveTangentialMotion === true;
        this.lumenFriction = profile.lumenFriction ?? 0.04;
        this.lumenAxialFriction = profile.lumenAxialFriction ??
            this.lumenFriction;
        this.lumenTorsionalFriction = profile.lumenTorsionalFriction ??
            this.lumenFriction;
        this.linearDamping = profile.linearDamping ?? 0.98;

        this.angularDamping = clamp(profile.angularDamping ?? 0.96, 0, 1);
        this.adaptationCompliance = Math.max(
            0,
            profile.adaptationCompliance ?? 0
        );
        this.projectionVelocityRetention = clamp(
            profile.projectionVelocityRetention ?? 1,
            0,
            1
        );
        // Some positional coupling constraints intentionally suppress the
        // velocity reconstructed inside their projected span. A separate
        // distal value prevents that numerical damping from freezing the
        // unsupported remainder of the same continuous rod.
        this.distalProjectionVelocityRetention =
            this.projectionVelocityRetention;
        this.distalProjectionVelocityRetentionStartNode = Infinity;
        this.maxSpeed = profile.maxSpeed ?? Infinity;
        this.maxAngularSpeed = profile.maxAngularSpeed ?? Infinity;
        this.maxFrameDisplacement = profile.maxFrameDisplacement ?? Infinity;
        this.frameDisplacementStartNode = 0;
        this.postStabilizationPasses = Math.max(
            0,
            Math.floor(profile.postStabilizationPasses ?? 0)
        );
        this.postStabilizationMinPasses = Math.max(
            0,
            Math.floor(profile.postStabilizationMinPasses ?? 2)
        );
        this.postStabilizationTolerance = Math.max(
            0,
            profile.postStabilizationTolerance ?? 0.01
        );
        this.postStabilizationSettledPasses = Math.max(
            1,
            Math.floor(profile.postStabilizationSettledPasses ?? 2)
        );
        // Optional body-local constitutive sweeps. Unlike stiffness or
        // damping, these change only how quickly the discrete solve converges
        // toward its existing equilibrium. The simulator may enable them
        // independently for each rod body.
        this.relaxationPasses = Math.max(
            0,
            Math.floor(profile.relaxationPasses ?? 0)
        );
        this.lastRelaxationPasses = 0;
        this.lastPostStabilizationPasses = 0;
        this.lastPostStabilizationResidual = Infinity;

        this.distalLengthTransportMaxCorrection = 1.25;
        this.postStabilizeBending = false;
        this.debugConstraintPhase = null;
        this.sleepVelocity = profile.sleepVelocity ?? 0.015;
        this.sleepAngularVelocity = profile.sleepAngularVelocity ?? 0.015;
        this.sleepFrames = profile.sleepFrames ?? 120;
        this.sleepPenetration = profile.sleepPenetration ?? 0.01;
        this.kirchhoffSleepAdaptationResidual = Math.max(
            0,
            profile.kirchhoffSleepAdaptationResidual ?? 0.01
        );
        this.kirchhoffSleepBendTwistResidual = Math.max(
            0,
            profile.kirchhoffSleepBendTwistResidual ?? 0.002
        );
        this.activeStart = 0;
        this.activeEnd = count - 1;
        this.collisionStartSegment = 0;
        this.collisionEndSegment = count - 2;
        // Wall exposure and introducer ownership are different material
        // classifications.  A device can be shielded by another catheter
        // after it has left the introducer, so deriving both ranges from the
        // same collision index makes the first released node jump back onto
        // the sheath axis.  When unset, a standalone rod is classified by its
        // spatial position in the sheath's axial slab.
        this.sheathMaterialEndNode = Infinity;
        this.sleepCounter = 0;
        this.sleeping = false;
        this.settledMaxPenetration = 0;

        // The global solver produces Float64 corrections. Rounding positions
        // after each trial can erase their small normal component near a wall
        // while still committing the full multiplier. Keep the applied motion
        // and its velocity history at the same precision as the solved step.
        this.x = new CoordinateArray(count);
        this.y = new CoordinateArray(count);
        this.z = new CoordinateArray(count);
        this.previousX = new CoordinateArray(count);
        this.previousY = new CoordinateArray(count);
        this.previousZ = new CoordinateArray(count);
        this.portalSegmentX = new Float32Array(count);
        this.portalSegmentY = new Float32Array(count);
        this.portalSegmentZ = new Float32Array(count);
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
        // A Kirchhoff body owns one material frame per edge. The third
        // director is constrained to the edge tangent; relative frame
        // rotations store the two bending strains and one torsional strain.
        this.orientationX = new Float64Array(this.segmentCount);
        this.orientationY = new Float64Array(this.segmentCount);
        this.orientationZ = new Float64Array(this.segmentCount);
        this.orientationW = new Float64Array(this.segmentCount);
        this.previousOrientationX = new Float64Array(this.segmentCount);
        this.previousOrientationY = new Float64Array(this.segmentCount);
        this.previousOrientationZ = new Float64Array(this.segmentCount);
        this.previousOrientationW = new Float64Array(this.segmentCount);
        this.angularVelocityX = new Float64Array(this.segmentCount);
        this.angularVelocityY = new Float64Array(this.segmentCount);
        this.angularVelocityZ = new Float64Array(this.segmentCount);
        this.inverseInertia1 = new Float64Array(this.segmentCount);
        this.inverseInertia2 = new Float64Array(this.segmentCount);
        this.inverseInertia3 = new Float64Array(this.segmentCount);
        this.materialCoordinate = new Float64Array(count);
        this.restRotation1 = new Float64Array(count);
        this.restRotation2 = new Float64Array(count);
        this.restRotation3 = new Float64Array(count);
        this.kirchhoffBendCompliance1 = new Float64Array(count);
        this.kirchhoffBendCompliance2 = new Float64Array(count);
        this.kirchhoffTwistCompliance = new Float64Array(count);
        this.adaptationLambdaX = new Float64Array(this.segmentCount);
        this.adaptationLambdaY = new Float64Array(this.segmentCount);
        this.adaptationLambdaZ = new Float64Array(this.segmentCount);
        this.bendTwistLambda1 = new Float64Array(count);
        this.bendTwistLambda2 = new Float64Array(count);
        this.bendTwistLambda3 = new Float64Array(count);
        this.orientationControlSegment = -1;
        this.orientationControlX = 0;
        this.orientationControlY = 0;
        this.orientationControlZ = 0;
        this.orientationControlW = 1;
        this.orientationControlCompliance = 0;
        this.orientationControlLambda = new Float64Array(3);
        // Kirchhoff constraints run many times per fixed step. Keep their
        // temporary views on the body so the hot path does not allocate and
        // collect thousands of short-lived objects. These buffers contain no
        // physical state; every numeric field used by a solve is overwritten
        // before the solve, preserving the exact equations and sweep order.
        this.kirchhoffScratch = {
            integrate: {
                angularIncrement: {},
                increment: {},
                current: {},
                multiplied: {},
                normalized: {}
            },
            velocity: {
                current: {},
                previous: {},
                previousInverse: {},
                relative: {},
                delta: {}
            },
            contactFrame: {
                current: {},
                previous: {},
                previousInverse: {},
                relative: {},
                delta: {}
            },
            contactTwist: {
                angularIncrement: {},
                increment: {},
                current: {},
                multiplied: {},
                normalized: {}
            },
            orientationControl: {
                target: {},
                orientation: {},
                restRotation: { x: 0, y: 0, z: 0 },
                inverseInertia: {},
                lambda: {},
                solver: {},
                options: null
            },
            adaptation: {
                x0: {},
                x1: {},
                orientation: {},
                lambda: {},
                inverseInertia: {},
                solver: {},
                options: null
            },
            bendTwist: {
                orientation0: {},
                orientation1: {},
                restRotation: {},
                inverseInertia0: {},
                inverseInertia1: {},
                compliance: {},
                lambda: {},
                solver: {},
                options: null
            }
        };
        this.restLength = new Float32Array(this.segmentCount);
        this.lengthLambda = new Float32Array(this.segmentCount);
        this.lengthNormalX = new Float32Array(this.segmentCount);
        this.lengthNormalY = new Float32Array(this.segmentCount);
        this.lengthNormalZ = new Float32Array(this.segmentCount);
        this.lengthLower = new Float32Array(this.segmentCount);
        this.lengthUpper = new Float32Array(this.segmentCount);
        this.lengthRhs = new Float32Array(this.segmentCount);
        this.lengthSolution = new Float32Array(this.segmentCount);
        this.foldCorrectionX = new Float32Array(count);
        this.foldCorrectionY = new Float32Array(count);
        this.foldCorrectionZ = new Float32Array(count);
        this.foldCorrectionWeight = new Float32Array(count);
        this.maxBendAngleByNode = new Float32Array(count);
        this.controlLambda = new Float32Array(count);
        // Retain the solved load at the same precision as the applied impulse
        // and tangential reactions; rounding Fn alone can break their cone.
        this.wallLambda = new Float64Array(this.segmentCount);
        // wallLambda is warm-started across frames for positional convergence.
        // Friction must use only normal corrections generated in the current
        // fixed step; otherwise a historic contact becomes residual static
        // friction after the catheter is merely tangent to the wall.
        this.wallFrictionLambda = new Float32Array(this.segmentCount);
        // A short-lived, bounded normal-load state provides Coulomb friction
        // hysteresis without reusing the positional warm-start multiplier as
        // an unlimited static-friction budget.
        this.wallFrictionLoad = new Float32Array(this.segmentCount);
        this.wallActive = new Uint8Array(this.segmentCount);
        // Rebuilding a contact plane must not quantize its point/normal before
        // measuring the applied equations (notably far from the world origin).
        this.wallT = new CoordinateArray(this.segmentCount);
        this.wallX = new CoordinateArray(this.segmentCount);
        this.wallY = new CoordinateArray(this.segmentCount);
        this.wallZ = new CoordinateArray(this.segmentCount);
        this.wallNormalX = new CoordinateArray(this.segmentCount);
        this.wallNormalY = new CoordinateArray(this.segmentCount);
        this.wallNormalZ = new CoordinateArray(this.segmentCount);
        this.wallBranchId = new Int32Array(this.segmentCount);
        this.wallFaceIndex = new Int32Array(this.segmentCount);
        this.wallGap = new CoordinateArray(this.segmentCount);
        this.wallInsideClearance = new Float32Array(this.segmentCount);
        this.wallCapsuleSampleCount = new Uint16Array(this.segmentCount);
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
        // Direct wall/sweep projections are accumulated separately from the
        // total nodal displacement. This lets velocity reconstruction retain
        // constitutive recovery while treating non-penetration as a
        // zero-restitution constraint instead of an inward launch impulse.
        this.wallProjectionX = new Float64Array(count);
        this.wallProjectionY = new Float64Array(count);
        this.wallProjectionZ = new Float64Array(count);
        this.toolProjectionX = new Float64Array(count);
        this.toolProjectionY = new Float64Array(count);
        this.toolProjectionZ = new Float64Array(count);
        this.lastMaximumRawSpeed = 0;
        this.lastMaximumWallProjectionSpeed = 0;
        this.lastMaximumWallProjectionNode = -1;
        this.lastMaximumRejectedWallProjectionSpeed = 0;
        this.lastMaximumToolProjectionSpeed = 0;
        this.lastMaximumRejectedToolProjectionSpeed = 0;
        this.lastMaximumReconstructedSpeed = 0;
        this.postPassStartX = new Float64Array(count);
        this.postPassStartY = new Float64Array(count);
        this.postPassStartZ = new Float64Array(count);
        this.coupledClosureStartX = new Float64Array(count);
        this.coupledClosureStartY = new Float64Array(count);
        this.coupledClosureStartZ = new Float64Array(count);
        this.wallBranchId.fill(-1);
        this.wallFaceIndex.fill(-1);
        this.wallGap.fill(Infinity);
        this.nodeRadius.fill(this.radius);
        this.inverseMass.fill(1 / Math.max(EPSILON, this.mass));
        this.restLength.fill(segmentLength);
        this.maxBendAngleByNode.fill(this.maxBendAngle);
        this.orientationW.fill(1);
        this.previousOrientationW.fill(1);
        const defaultInverseInertia = Math.max(
            0,
            profile.inverseAngularInertia ?? 1 / Math.max(EPSILON, this.mass)
        );
        this.inverseInertia1.fill(Math.max(
            0,
            profile.inverseInertia1 ?? defaultInverseInertia
        ));
        this.inverseInertia2.fill(Math.max(
            0,
            profile.inverseInertia2 ?? defaultInverseInertia
        ));
        this.inverseInertia3.fill(Math.max(
            0,
            profile.inverseInertia3 ?? defaultInverseInertia
        ));
        this.kirchhoffBendCompliance1.fill(Math.max(
            0,
            profile.kirchhoffBendCompliance1 ??
                profile.kirchhoffBendCompliance ??
                this.bendCompliance
        ));
        this.kirchhoffBendCompliance2.fill(Math.max(
            0,
            profile.kirchhoffBendCompliance2 ??
                profile.kirchhoffBendCompliance ??
                this.bendCompliance
        ));
        this.kirchhoffTwistCompliance.fill(Math.max(
            0,
            profile.kirchhoffTwistCompliance ??
                profile.twistCompliance ??
                this.bendCompliance
        ));
        for (let index = 0; index < count; index++) {
            this.materialCoordinate[index] = index * segmentLength;
        }
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
        if (
            nextStart !== this.collisionStartSegment ||
            nextEnd !== this.collisionEndSegment
        ) {
            this.wake();
            // A catheter shields material that has become contained. Contact
            // flags from the previous, wider collision range must not survive
            // that ownership change: they would still add wall friction and
            // guidewire resistance even though no vessel constraint is solved
            // for those segments anymore.
            for (let segment = 0; segment < this.segmentCount; segment++) {
                if (segment >= nextStart && segment <= nextEnd) continue;
                this.wallActive[segment] = 0;
                this.wallLambda[segment] = 0;
                this.wallFrictionLambda[segment] = 0;
                this.wallFrictionLoad[segment] = 0;
            }
        }
        this.collisionStartSegment = nextStart;
        this.collisionEndSegment = nextEnd;
        return this;
    }

    setSheathMaterialEndNode(endNode = Infinity) {
        const nextEnd = Number.isFinite(endNode)
            ? clamp(Math.floor(endNode), 0, this.count - 1)
            : Infinity;
        if (nextEnd !== this.sheathMaterialEndNode) this.wake();
        this.sheathMaterialEndNode = nextEnd;
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

    get rodModel() { return 'kirchhoff'; }

    get constitutiveSolver() { return 'direct'; }

    setMaterialFrame(
        segment,
        x,
        y,
        z,
        w,
        { preservePrevious = false, resetAngularVelocity = true } = {}
    ) {
        if (segment < 0 || segment >= this.segmentCount) return this;
        const orientation = normalizeQuaternion({ x, y, z, w }, {});
        this.orientationX[segment] = orientation.x;
        this.orientationY[segment] = orientation.y;
        this.orientationZ[segment] = orientation.z;
        this.orientationW[segment] = orientation.w;
        if (!preservePrevious) {
            this.previousOrientationX[segment] = orientation.x;
            this.previousOrientationY[segment] = orientation.y;
            this.previousOrientationZ[segment] = orientation.z;
            this.previousOrientationW[segment] = orientation.w;
        }
        if (resetAngularVelocity) {
            this.angularVelocityX[segment] = 0;
            this.angularVelocityY[segment] = 0;
            this.angularVelocityZ[segment] = 0;
        }
        this.adaptationLambdaX[segment] = 0;
        this.adaptationLambdaY[segment] = 0;
        this.adaptationLambdaZ[segment] = 0;
        if (segment > 0) {
            this.bendTwistLambda1[segment] = 0;
            this.bendTwistLambda2[segment] = 0;
            this.bendTwistLambda3[segment] = 0;
        }
        if (segment + 1 < this.segmentCount) {
            this.bendTwistLambda1[segment + 1] = 0;
            this.bendTwistLambda2[segment + 1] = 0;
            this.bendTwistLambda3[segment + 1] = 0;
        }
        this.wake();
        return this;
    }

    setKirchhoffRestRotation(
        joint,
        bend1,
        bend2,
        twist = 0,
        bendCompliance1 = this.kirchhoffBendCompliance1[joint] ?? this.bendCompliance,
        bendCompliance2 = this.kirchhoffBendCompliance2[joint] ?? bendCompliance1,
        twistCompliance = this.kirchhoffTwistCompliance[joint] ?? bendCompliance1
    ) {
        if (joint <= 0 || joint >= this.segmentCount) return this;
        this.restRotation1[joint] = bend1;
        this.restRotation2[joint] = bend2;
        this.restRotation3[joint] = twist;
        this.kirchhoffBendCompliance1[joint] = Math.max(0, bendCompliance1);
        this.kirchhoffBendCompliance2[joint] = Math.max(0, bendCompliance2);
        this.kirchhoffTwistCompliance[joint] = Math.max(0, twistCompliance);
        this.bendTwistLambda1[joint] = 0;
        this.bendTwistLambda2[joint] = 0;
        this.bendTwistLambda3[joint] = 0;
        this.wake();
        return this;
    }

    setProximalOrientationControl(
        x,
        y,
        z,
        w,
        compliance = 0,
        segment = this.activeStart
    ) {
        const targetSegment = clamp(
            Math.floor(segment),
            0,
            this.segmentCount - 1
        );
        const target = normalizeQuaternion({ x, y, z, w }, {});
        const changed = this.orientationControlSegment !== targetSegment ||
            Math.abs(this.orientationControlX - target.x) > 1e-8 ||
            Math.abs(this.orientationControlY - target.y) > 1e-8 ||
            Math.abs(this.orientationControlZ - target.z) > 1e-8 ||
            Math.abs(this.orientationControlW - target.w) > 1e-8 ||
            Math.abs(this.orientationControlCompliance - compliance) > 1e-12;
        this.orientationControlSegment = targetSegment;
        this.orientationControlX = target.x;
        this.orientationControlY = target.y;
        this.orientationControlZ = target.z;
        this.orientationControlW = target.w;
        this.orientationControlCompliance = Math.max(0, compliance);
        if (changed) {
            this.orientationControlLambda.fill(0);
            this.wake();
        }
        return this;
    }

    clearProximalOrientationControl() {
        if (this.orientationControlSegment >= 0) this.wake();
        this.orientationControlSegment = -1;
        this.orientationControlLambda.fill(0);
        return this;
    }

    captureKirchhoffRestConfiguration({ captureRestRotation = true } = {}) {
        if (this.segmentCount <= 0) return this;
        let previousFrame = null;
        for (let segment = 0; segment < this.segmentCount; segment++) {
            const edge = {
                x: this.x[segment + 1] - this.x[segment],
                y: this.y[segment + 1] - this.y[segment],
                z: this.z[segment + 1] - this.z[segment]
            };
            const frame = previousFrame
                ? transportBishopFrame(previousFrame, edge, {})
                : createBishopFrame(edge, null, {});
            this.orientationX[segment] = frame.x;
            this.orientationY[segment] = frame.y;
            this.orientationZ[segment] = frame.z;
            this.orientationW[segment] = frame.w;
            this.previousOrientationX[segment] = frame.x;
            this.previousOrientationY[segment] = frame.y;
            this.previousOrientationZ[segment] = frame.z;
            this.previousOrientationW[segment] = frame.w;
            this.angularVelocityX[segment] = 0;
            this.angularVelocityY[segment] = 0;
            this.angularVelocityZ[segment] = 0;
            if (captureRestRotation && segment > 0) {
                const relative = multiplyQuaternions(
                    conjugateQuaternion(previousFrame, {}),
                    frame,
                    {}
                );
                const restRotation = quaternionLog(relative, {});
                this.restRotation1[segment] = restRotation.x;
                this.restRotation2[segment] = restRotation.y;
                this.restRotation3[segment] = restRotation.z;
            }
            previousFrame = frame;
        }
        this.adaptationLambdaX.fill(0);
        this.adaptationLambdaY.fill(0);
        this.adaptationLambdaZ.fill(0);
        this.bendTwistLambda1.fill(0);
        this.bendTwistLambda2.fill(0);
        this.bendTwistLambda3.fill(0);
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
        this.lengthLambda.fill(0);
        {
            this.captureKirchhoffRestConfiguration();
        }
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

    syncFromRodState(rod, { resetVelocity = false, preservePrevious = false } = {}) {
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
            this.maxBendAngleByNode[index] = clamp(
                this.inheritRodStateBendLimit
                    ? storage.bendAngleLimit?.[index] ?? this.maxBendAngle
                    : this.maxBendAngle,
                1,
                179
            );
        }
        if (!preservePrevious) this.copyCurrentToPrevious();
        if (changed) this.wake();
        return this;
    }

    syncToRodState(rod) {
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
        contactActivation = 0.25,
        coupledClosureMaxPasses = 32,
        coupledContactMaxPasses = 32,
        coupledContainmentTolerance = 0.001,
        coupledLengthTolerance = 0.002,
        coupledAngularToleranceRad = 0.001,
        reuseDirectLinearization = true,
        coupledSystem = null,
        wholeStepSystem = null,
        jointMotionMode = 'position-history',
        adaptiveLineSearch = true
    } = {}) {
        this.contactField = contactField;
        this.adaptiveLineSearch = adaptiveLineSearch;
        this.reuseDirectLinearization = reuseDirectLinearization;
        this.coupledSystem = coupledSystem;
        validateWholeStepSystem(wholeStepSystem);
        this.wholeStepSystem = wholeStepSystem;
        this._pendingWholeSubstep = null;
        if (!['position-history', 'split-physical-bias'].includes(jointMotionMode)) throw new RangeError('Unknown joint motion mode');
        this.jointMotionMode = jointMotionMode;
        this.fixedDt = fixedDt;
        this.maxSubsteps = maxSubsteps;
        this.iterations = iterations;
        this.penetrationIterations = penetrationIterations;
        this.highPenetration = highPenetration;
        this.contactActivation = contactActivation;
        this.coupledClosureMaxPasses = Math.max(
            8,
            Math.floor(coupledClosureMaxPasses)
        );
        this.coupledContactMaxPasses = Math.max(1, Math.floor(coupledContactMaxPasses));
        this.coupledContainmentTolerance = Math.max(
            0,
            coupledContainmentTolerance
        );
        this.coupledLengthTolerance = Math.max(
            0,
            coupledLengthTolerance
        );
        this.coupledAngularToleranceRad = Math.max(0, coupledAngularToleranceRad);
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
        this.captureCoupledClosureTrace = false;
        this.coupledClosureTrace = [];
        this.lastCoupledRelaxationPasses = 0;
        this.lastCoupledClosureConverged = true;
        this.lastCoupledContainmentResidual = 0;
        this.lastCoupledContactPasses = 0;
        this._queryStart = { x: 0, y: 0, z: 0 };
        this._queryEnd = { x: 0, y: 0, z: 0 };
        this._segmentParameters = { s: 0, t: 0 };
        this._contact = createContactResult();
        this._sweep = createContactResult();
        this._wallRepairPenetration = new Float32Array(0);
        this._wallRepairEligible = new Uint8Array(0);
        this.wallRepairResiduals = new Float32Array(
            MAX_WALL_CORRECTION_PASSES
        );
        this.wallRepairWorstSegments = new Int32Array(
            MAX_WALL_CORRECTION_PASSES
        );
        this.wallRepairWorstBodies = new Int16Array(
            MAX_WALL_CORRECTION_PASSES
        );
        this.timings = {
            total: createPhaseTimings(),
            integrate: createPhaseTimings(),
            narrowPhase: createPhaseTimings(),
            constraints: createPhaseTimings(),
            constraintPrimary: createPhaseTimings(),
            constraintBodyClosure: createPhaseTimings(),
            constraintBodyLengthPolish: createPhaseTimings(),
            constraintBodyWallRepair: createPhaseTimings(),
            constraintBodyPrePost: createPhaseTimings(),
            constraintBodyPostStabilization: createPhaseTimings(),
            constraintCoupledClosure: createPhaseTimings(),
            constraintMovingClosure: createPhaseTimings(),
            velocity: createPhaseTimings()
        };
    }

    createRod(id, count, segmentLength, profile = {}) {
        const body = new EndovascularRodBody(id, count, segmentLength, profile, this.jointMotionMode);
        body.contactField = this.contactField;
        this.bodies.push(body);
        return body;
    }

    addRod(body) {
        if (!(body instanceof EndovascularRodBody)) throw new TypeError('EndovascularRodBody is required');
        if (!this.bodies.includes(body)) this.bodies.push(body);
        return body;
    }

    addSheath({
        id = 'sheath',
        start,
        end,
        innerRadius = DEFAULT_TOOL_PROFILES.sheath.innerRadius,
        proximalExtension = 0,
        bodies = null
    } = {}) {
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
            // A haemostatic valve/loading hub may support material proximal
            // to the anatomical sheath start. This extends the same open,
            // radial lumen constraint; it does not impose an axial target or
            // change the distal sheath length used by feed/portal logic.
            proximalExtension: Math.max(
                0,
                Number.isFinite(proximalExtension) ? proximalExtension : 0
            ),
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
        surfaceFrictionEnabled = true,
        axialFriction = friction,
        torsionalFriction = friction,
        radialVelocityDamping = 0.9,
        coupledBendingRateDamping = 0.5,
        coupledBendingRatePasses = 8,
        lumenMaxCorrection = Infinity,
        enabled = true,
        openProximal = true,
        openDistal = true,
        searchWindow = 10,
        outerStartNode = outerBody.activeStart,
        startNode = innerBody.activeStart,
        endNode = innerBody.activeEnd,
        innerResponse = 1,
        outerResponse = 1,

        enforceDistalPortal = false,

        portalFilletRadius = 0.15,

        innerArcOffset = 0,
        containedLength = Infinity
    } = {}) {
        const constraint = {
            model: 'kirchhoff',
            innerBody,
            outerBody,
            innerRadius,
            compliance,
            surfaceFrictionEnabled,
            friction: surfaceFrictionEnabled ? friction : 0,
            axialFriction: surfaceFrictionEnabled ? Math.max(0, axialFriction) : 0,
            torsionalFriction: surfaceFrictionEnabled ? Math.max(0, torsionalFriction) : 0,
            // Blood and the lubricious coatings in the narrow annular gap
            // oppose transverse relative motion much more strongly than
            // axial sliding. This is a velocity-level, momentum-conserving
            // coupling; it does not prescribe a common centreline or shape.
            radialVelocityDamping: clamp(radialVelocityDamping, 0, 1),
            coupledBendingRateDamping: clamp(
                coupledBendingRateDamping,
                0,
                1
            ),
            coupledBendingRatePasses: Math.max(
                0,
                Math.floor(coupledBendingRatePasses)
            ),
            lumenMaxCorrection: Math.max(
                EPSILON,
                Number.isFinite(lumenMaxCorrection)
                    ? lumenMaxCorrection
                    : Infinity
            ),
            enabled,
            openProximal,
            openDistal,
            searchWindow,
            outerStartNode,
            startNode,
            endNode,
            innerResponse: clamp(innerResponse, 0, 1),
            outerResponse: clamp(outerResponse, 0, 1),
            enforceDistalPortal,
            // The open aperture constrains the geometric crossing of the rods.
            distalPortalModel: 'spatial',

            // Effective centreline fillet of the physical distal lumen edge.
            // It is local contact geometry, not an exit-direction target.
            portalFilletRadius: Math.max(0, portalFilletRadius),

            innerArcOffset,
            containedLength,
            manifold: new KirchhoffContactManifold({
                    normalOnly: !surfaceFrictionEnabled,
                    frictionCoefficient: surfaceFrictionEnabled ? Math.max(0, axialFriction) : 0,
                    retentionSteps: 1
                }),
            kirchhoffOuterSegmentByInner: new Int32Array(innerBody.segmentCount),
            kirchhoffContacts: [],
            kirchhoffContactActivation: Math.max(0.01, this.contactActivation),
            kirchhoffMaxViolation: 0,
            _kirchhoffStepOpen: false,
            lambdas: new Float32Array(innerBody.count),
            closestSegment: new Int32Array(innerBody.count),
            closestT: new Float32Array(innerBody.count),
            outerPostX: new Float32Array(outerBody.count),
            outerPostY: new Float32Array(outerBody.count),
            outerPostZ: new Float32Array(outerBody.count),
            _lastEnabled: enabled,
            _lastOuterStartNode: outerStartNode,
            _lastStartNode: startNode,
            _lastEndNode: endNode,
            _lastInnerActiveStart: innerBody.activeStart,
            _lastInnerActiveEnd: innerBody.activeEnd,
            _lastOuterActiveStart: outerBody.activeStart,
            _lastOuterActiveEnd: outerBody.activeEnd
        };
        constraint.kirchhoffOuterSegmentByInner?.fill(-1);
        constraint.closestSegment.fill(-1);
        this.containments.push(constraint);
        return constraint;
    }

    updateContainmentWindow(constraint, {
        enabled = constraint?.enabled,
        outerStartNode = constraint?.outerStartNode,
        startNode = constraint?.startNode,
        endNode = constraint?.endNode,
        innerArcOffset = constraint?.innerArcOffset,
        containedLength = constraint?.containedLength,
        enforceDistalPortal = constraint?.enforceDistalPortal
    } = {}) {
        if (!this.containments.includes(constraint)) {
            throw new TypeError('Containment must belong to this physics world');
        }
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const nextOuterStart = clamp(
            Math.floor(outerStartNode),
            outer.activeStart,
            outer.activeEnd
        );
        const nextStart = clamp(
            Math.floor(startNode),
            inner.activeStart,
            inner.activeEnd
        );
        const nextEnd = clamp(
            Math.floor(endNode),
            nextStart,
            inner.activeEnd
        );
        const nextArcOffset = Math.max(
            0,
            Number.isFinite(innerArcOffset) ? innerArcOffset : 0
        );
        const nextContainedLength = Math.max(
            0,
            Number.isFinite(containedLength) ? containedLength : 0
        );
        const nextEnabled = enabled === true &&
            nextEnd >= nextStart &&
            outer.activeEnd > nextOuterStart;
        const topologyChanged =
            constraint.enabled !== nextEnabled ||
            constraint.outerStartNode !== nextOuterStart ||
            constraint.startNode !== nextStart ||
            constraint.endNode !== nextEnd ||
            constraint.innerArcOffset !== nextArcOffset ||
            constraint.containedLength !== nextContainedLength;

        // Publish one coherent material window. The world has no asynchronous
        // step, but centralizing this transaction prevents a solver call or a
        // future substep hook from observing a mixture of old indices and new
        // arc coordinates.
        constraint.outerStartNode = nextOuterStart;
        constraint.startNode = nextStart;
        constraint.endNode = nextEnd;
        constraint.innerArcOffset = nextArcOffset;
        constraint.containedLength = nextContainedLength;
        constraint.enforceDistalPortal = enforceDistalPortal !== false;
        constraint.enabled = nextEnabled;

        if (topologyChanged && true) {
            constraint._kirchhoffMappingLocked = false;
            // Cached mappings are material-local. Retain the overlap, but
            // invalidate indices which left the contiguous lumen window so a
            // later re-entry cannot resurrect a stale remote segment.
            for (let segment = 0; segment < constraint.startNode; segment++) {
                constraint.kirchhoffOuterSegmentByInner[segment] = -1;
            }
            for (
                let segment = Math.max(0, constraint.endNode + 1);
                segment < constraint.kirchhoffOuterSegmentByInner.length;
                segment++
            ) {
                constraint.kirchhoffOuterSegmentByInner[segment] = -1;
            }
        }
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
            lambdas: new Float64Array(pairCount),
            _lastEnabled: enabled,
            _lastStartSegmentA: startSegmentA,
            _lastEndSegmentA: endSegmentA,
            _lastStartSegmentB: startSegmentB,
            _lastEndSegmentB: endSegmentB
        };
        this.toolContacts.push(constraint);
        return constraint;
    }

    // A rejected split leaves one prepared dt pending. Its callback inputs
    // survive rollback, and only an accepted retry consumes that elapsed time.
    advance(frameDt, beforeSubstep = null) {
        if (this._pendingWholeSubstep || this.wholeStepSystem !== null)
            return this.#advanceWholeStep(frameDt, beforeSubstep);
        const elapsed = Number.isFinite(frameDt) ? Math.max(0, frameDt) : 0;
        this.accumulator += elapsed;
        let substeps = 0;
        while (this.accumulator + EPSILON >= this.fixedDt && substeps < this.maxSubsteps) {
            if (this._pendingSplitSubstep && (this.jointMotionMode !== 'split-physical-bias' ||
                this._pendingSplitSubstep.dt !== this.fixedDt))
                throw new Error('A pending split timestep must retain its dt and motion mode');
            if (!this._pendingSplitSubstep) {
                beforeSubstep?.(this.fixedDt, substeps);
                if (this.wholeStepSystem !== null)
                    this._pendingWholeSubstep = this.#newWholePending(true);
                else if (this.jointMotionMode === 'split-physical-bias' && this.#jointCoupledConstraint())
                    this._pendingSplitSubstep = { dt: this.fixedDt };
            }
            const result = this.stepFixed();
            if (result?.accepted === false) break;
            if (!result?.consumedPendingDt) {
                this._pendingSplitSubstep = null;
                this.accumulator -= this.fixedDt;
            }
            substeps++;
        }
        this.lastSubsteps = substeps;
        return substeps;
    }

    // Legacy position-history keeps its void return. Optional split returns
    // an owned acceptance result; false means no committed physical timestep.
    stepFixed() {
        // The whole-dt provider owns prediction, all constraints/friction,
        // physical histories and atomic body publication. It cannot enter
        // any legacy pass or fall back after a rejection or exception.
        if (this._pendingWholeSubstep || this.wholeStepSystem !== null)
            return this.#stepWholeFixed();
        if (this._pendingSplitSubstep && (this.jointMotionMode !== 'split-physical-bias' ||
            this._pendingSplitSubstep.dt !== this.fixedDt))
            throw new Error('A pending split timestep must retain its dt and motion mode');
        if (this.jointMotionMode !== 'split-physical-bias') return this.#stepFixedImpl();
        const joint = this.#jointCoupledConstraint();
        if (!joint) {
            if (!this._pendingSplitSubstep) {
                // The selected law applies to an eligible pair. Before the
                // catheter is deployed the existing independent wire path
                // still prepares it; this is not retrying a failed split.
                this.lastStepResult = null;
                return this.#stepFixedImpl();
            }
            this.lastCoupledClosureConverged = false;
            return this.lastStepResult = { accepted: false, dt: this.fixedDt,
                status: 'split-joint-unavailable', diagnostics: null };
        }
        const transactionStart = now();
        const transaction = captureKirchhoffSplitStep(this);
        try {
            this.#stepFixedImpl(true);
            const topologyUnchanged = kirchhoffSplitStepTopologyUnchanged(this, transaction);
            const accepted = this.lastCoupledClosureConverged && topologyUnchanged;
            const diagnostics = structuredClone(getKirchhoffSplitMotionStats(this));
            const result = { accepted, dt: transaction.dt,
                status: accepted ? 'accepted' : topologyUnchanged ? 'split-uncertified' : 'split-topology-changed', diagnostics };
            if (!accepted) {
                if (diagnostics) diagnostics.candidateMotion = this.bodies.map(body => Object.fromEntries(
                    ['velocityX', 'velocityY', 'velocityZ', 'angularVelocityX', 'angularVelocityY', 'angularVelocityZ']
                        .map(key => [key, Array.from(body[key])])));
                if (diagnostics) diagnostics.candidatePose = this.bodies.map(body => ({
                    id: body.id, activeStart: body.activeStart, activeEnd: body.activeEnd,
                    ...Object.fromEntries(['x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW']
                        .map(key => [key, Array.from(body[key])]))
                }));
                restoreKirchhoffSplitStep(this, transaction);
                this.lastCoupledClosureConverged = false;
                if (diagnostics) { diagnostics.certified = false; diagnostics.historyCommits = 0; }
            } else if (this._pendingSplitSubstep) {
                // Also consume the queued dt when a caller retries stepFixed
                // directly after a failed advance. It must not execute again
                // when advance next visits its accumulator.
                this.accumulator -= this._pendingSplitSubstep.dt;
                this._pendingSplitSubstep = null;
                result.consumedPendingDt = true;
            }
            this.lastStepResult = result;
            return result;
        } catch (error) {
            let diagnostics = null;
            try { diagnostics = structuredClone(getKirchhoffSplitMotionStats(this)); } catch { /* Incomplete failed phase. */ }
            if (!transaction.restored) restoreKirchhoffSplitStep(this, transaction);
            this.lastCoupledClosureConverged = false;
            if (diagnostics) { diagnostics.certified = false; diagnostics.historyCommits = 0; }
            this.lastStepResult = { accepted: false, dt: transaction.dt, status: 'split-error',
                message: error.message, diagnostics };
            throw error;
        } finally {
            // One sample includes snapshot, failed work and rollback, rather
            // than reporting only the inner physics solve as the whole dt.
            recordTiming(this.timings.total, now() - transactionStart);
        }
    }

    #assertWholeConfiguration() {
        const pending = this._pendingWholeSubstep;
        if (this._pendingSplitSubstep)
            throw new Error('A pending split timestep cannot switch to a wholeStepSystem');
        if (pending && (this.wholeStepSystem !== pending.system || this.fixedDt !== pending.dt ||
            pending.system.id !== pending.id || pending.system.step !== pending.step || pending.system.reset !== pending.reset))
            throw new Error('A pending whole timestep must retain its dt and wholeStepSystem identity and methods');
        if (pending?.running) throw new Error('A whole timestep is already running');
        validateWholeStepSystem(this.wholeStepSystem);
        if (this.wholeStepSystem === null) throw new Error('A whole timestep requires its wholeStepSystem');
        if (!(Number.isFinite(this.fixedDt) && this.fixedDt > 0))
            throw new RangeError('A whole timestep requires a positive finite fixedDt');
    }

    #newWholePending(consumesAccumulator) {
        this.#assertWholeConfiguration();
        const system = this.wholeStepSystem;
        return { system, id: system.id, step: system.step, reset: system.reset,
            dt: this.fixedDt, consumesAccumulator, running: false, preparationFailed: false, preparationError: null, preparationMs: 0 };
    }

    #advanceWholeStep(frameDt, beforeSubstep) {
        // Check the pending identity even with no available elapsed time;
        // rejected mode changes must not add new elapsed time to the queue.
        this.#assertWholeConfiguration();
        this.accumulator += Number.isFinite(frameDt) ? Math.max(0, frameDt) : 0;
        let substeps = 0;
        try {
            while (this.accumulator + EPSILON >= this.fixedDt && substeps < this.maxSubsteps) {
                this.#assertWholeConfiguration();
                if (!this._pendingWholeSubstep) {
                    const pending = this._pendingWholeSubstep = this.#newWholePending(true), started = now();
                    pending.running = true;
                    try {
                        beforeSubstep?.(pending.dt, substeps);
                    } catch (error) {
                        // A partially executed preparation cannot safely be
                        // repeated or passed to the provider as valid input.
                        // Reset is the explicit way to abandon this dt.
                        pending.preparationFailed = true;
                        pending.preparationError = error;
                        this.lastStepResult = { accepted: false, dt: pending.dt, systemId: pending.id,
                            status: 'whole-step-preparation-error', message: error?.message ?? String(error), diagnostics: null };
                        recordTiming(this.timings.total, now() - started);
                        throw error;
                    } finally {
                        pending.running = false;
                        if (!pending.preparationFailed) pending.preparationMs = now() - started;
                    }
                }
                // A direct stepFixed rejection has no elapsed time attached.
                // An advance retry now funds exactly that prepared dt from
                // its available queue without preparing the inputs again.
                this._pendingWholeSubstep.consumesAccumulator = true;
                const result = this.stepFixed();
                if (!result.accepted) break;
                substeps++;
            }
        } finally { this.lastSubsteps = substeps; }
        return substeps;
    }

    #stepWholeFixed() {
        this.#assertWholeConfiguration();
        const pending = this._pendingWholeSubstep ??= this.#newWholePending(false);
        if (pending.preparationFailed) throw pending.preparationError;
        const started = now(), stepCount = this.stepCount, accumulator = this.accumulator;
        pending.running = true;
        try {
            const supplied = pending.step.call(pending.system, this, pending.dt);
            if (!supplied || typeof supplied.then === 'function' || typeof supplied.accepted !== 'boolean' ||
                supplied.dt !== pending.dt || typeof supplied.status !== 'string' || !supplied.status.trim())
                throw new TypeError('wholeStepSystem.step must return a synchronous { accepted: boolean, dt: requested dt, status: nonempty string }');
            if (this.wholeStepSystem !== pending.system || this.fixedDt !== pending.dt || pending.system.id !== pending.id ||
                pending.system.step !== pending.step || pending.system.reset !== pending.reset)
                throw new Error('wholeStepSystem.step changed its prepared dt or system identity');
            if (this.stepCount !== stepCount || this.accumulator !== accumulator)
                throw new Error('wholeStepSystem.step must leave World clocks to World');
            // The provider must return owned/stable diagnostics and must
            // publish its body/state changes atomically only on success.
            // World does not run the old split snapshot or mutate its own
            // clocks on a rejected provider transaction.
            const result = { ...supplied, systemId: pending.id, consumedPendingDt: false };
            if (result.accepted) {
                this.stepCount = stepCount + 1;
                if (pending.consumesAccumulator) {
                    this.accumulator = accumulator - pending.dt;
                    result.consumedPendingDt = true;
                }
                this._pendingWholeSubstep = null;
            }
            this.lastCoupledSolver = pending.id;
            this.lastCoupledClosureConverged = result.accepted;
            return this.lastStepResult = result;
        } catch (error) {
            this.stepCount = stepCount;
            this.accumulator = accumulator;
            this.lastCoupledSolver = pending.id;
            this.lastCoupledClosureConverged = false;
            this.lastStepResult = { accepted: false, dt: pending.dt, systemId: pending.id,
                status: 'whole-step-error', message: error?.message ?? String(error), diagnostics: null };
            throw error;
        } finally {
            pending.running = false;
            // Includes provider preparation, failed work and its rollback;
            // beforeSubstep time belongs only to its first attempted dt.
            recordTiming(this.timings.total, now() - started + pending.preparationMs);
            pending.preparationMs = 0;
        }
    }

    #stepFixedImpl(transactional = false) {
        const totalStart = now();
        this.lastCoupledSolver = 'independent';
        this.lastJointNonlinearFailure = null;
        this.lastJointLineSearch = createLineSearchStats();
        this.lastJointFactorizations = this.lastJointLinearIterations = 0;
        this.lastJointTrialEvaluations = this.lastJointBacktracks = 0;
        this.lastJointMaximumBand = this.lastJointMaximumRows = 0;
        this.lastJointCosts = { assemblyMs:0, solveMs:0, applyMs:0, snapshotMs:0, restoreMs:0, measureMs:0,
            geometryReuseCount:0, frictionBatchReuseCount:0,
            acceptedFrictionReuseCount:0,
            solveCalls:0, applyCalls:0, measureCalls:0, fullMeasureCalls:0,
            snapshotObjects:0, snapshotBytes:0, snapshots:0, restores:0,
            condensedSetupMs:0,schurMs:0,contactSolveMs:0,reconstructionMs:0,seedMs:0 };
        this.contactCount = 0;
        this.maxPenetration = 0;
        const connected = this.#jointCoupledConstraint();
        for (const body of this.bodies) body._splitPhysicalMotion = null;
        for (const constraint of this.containments) {
            if (constraint !== connected || this.jointMotionMode !== 'split-physical-bias') {
                // Phase-local motion belongs only to the selected pair. In
                // particular, a withdrawn/disabled pair must not publish an
                // old certificate or alter the independent velocity path.
                delete constraint._splitMotion;
                delete constraint.surfaceMotion;
            }
        }
        const mechanicalComponents = this.coupledSystem?.independentComponents &&
            this.jointMotionMode === 'position-history' ? selectKirchhoffMechanicalComponents(this) : null;
        const independentComponents = connected ? null : mechanicalComponents;
        for (const component of mechanicalComponents ?? []) {
            if (component._jointClosureConverged !== true)
                for (const body of kirchhoffComponentBodies(component)) if (body.sleeping) body.wake();
        }
        let everyBodySleeping = this.bodies.length > 0;
        for (let index = 0; index < this.bodies.length; index++) {
            if (!this.bodies[index].sleeping) {
                everyBodySleeping = false;
                break;
            }
        }
        // Sleep is valid only for an already solved mechanical component.
        // It must never turn a failed closure into a successful zero-work step.
        if (everyBodySleeping && this.#jointCoupledConstraint() && !this.lastCoupledClosureConverged) {
            for (const body of this.bodies) body.wake();
            everyBodySleeping = false;
        }
        if (everyBodySleeping) {
            this.lastCoupledSolver = 'sleeping';
            this.lastLengthPolishPasses = 0;
            this.lastWallRepairPasses = 0;
            this.lastCoupledClosurePasses = 0;
            this.lastCoupledRelaxationPasses = 0;
            this.lastCoupledClosureConverged = true;
            this.lastCoupledContainmentResidual = 0;
            this.lastCoupledContactPasses = 0;
            for (const [name, timing] of Object.entries(this.timings)) {
                if (name !== 'total') recordTiming(timing, 0);
            }
            this.stepCount++;
            if (!transactional) recordTiming(this.timings.total, now() - totalStart);
            return;
        }
        // A load on either tool wakes their connected mechanical component
        // before prediction captures positions and material frames.
        const wallFrictionIncoming = connected && this.jointMotionMode === 'split-physical-bias'
            ? captureKirchhoffWallFrictionIncoming(connected, this) : null;
        if (connected) {
            if (connected.innerBody.sleeping) connected.innerBody.wake();
            if (connected.outerBody.sleeping) connected.outerBody.wake();
            if (this.jointMotionMode === 'split-physical-bias') {
                // Configured damping acts once on physical incoming motion.
                // The subsequent joint velocity solve owns normal/friction
                // impulses; legacy post-projection velocity filters do not.
                this.#dampKirchhoffContainedRadialVelocity(connected, connected.innerBody, connected.outerBody);
                this.#dampKirchhoffCoupledBendingRates(connected, connected.innerBody, connected.outerBody);
            }
        }
        for (
            let constraintIndex = 0;
            constraintIndex < this.containments.length;
            constraintIndex++
        ) {
            const constraint = this.containments[constraintIndex];
            if (constraint._kirchhoffStepOpen) {
                constraint.manifold.endStep({ prune: false });
            }
            constraint.manifold.beginStep();
            // These positional multipliers are accumulated inside one fixed
            // XPBD solve. Keeping them across frames without applying the
            // matching warm-start displacement makes the manifold disagree
            // with the generalized coordinates and creates an alternating
            // lumen/length cycle. Preserve material contact identities and
            // tangent bases, but begin each new step with zero impulse.
            for (const contact of constraint.manifold.contacts()) {
                constraint.manifold.clearLambdas(contact);
            }
            constraint._kirchhoffStepOpen = true;

            constraint._contactBlockSweeps = 0;
            constraint._contactBlockIterations = 0;
            constraint.kirchhoffSolverResidual = null;
            constraint.kirchhoffContactMotion = 0;

            constraint._kirchhoffMappingLocked = false;
            const assemblyStarted = now();
            constraint.kirchhoffContacts.length = 0;
            constraint.kirchhoffMaxViolation = 0;
        }
        this._inCoupledClosure = false;
        let phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.contactField = this.contactField;
            const direct = body.kirchhoffScratch.direct;
            if (direct) {
                direct.factorAge = Infinity;
                direct.factorizationCount = 0;
                direct.factorReuseCount = 0;
            }
            const activeNodeStart = Math.max(0, body.activeStart);
            const activeNodeEnd = Math.min(body.count, body.activeEnd + 1);
            const activeSegmentStart = Math.min(
                body.segmentCount,
                activeNodeStart
            );
            const activeSegmentEnd = Math.min(
                body.segmentCount,
                body.activeEnd
            );
            // Inactive storage can be much larger than the inserted material
            // (notably the 1.28 m catheter allocation in the 24 cm scenario).
            // Multipliers are consumed only inside the active range and are
            // reset as soon as a node becomes active, so clearing dormant
            // capacity every 1/120 s performs no physical work.
            body.lengthLambda.fill(0, activeSegmentStart, activeSegmentEnd);
            body.controlLambda.fill(0, activeNodeStart, activeNodeEnd);
            body.adaptationLambdaX.fill(0, activeSegmentStart, activeSegmentEnd);
            body.adaptationLambdaY.fill(0, activeSegmentStart, activeSegmentEnd);
            body.adaptationLambdaZ.fill(0, activeSegmentStart, activeSegmentEnd);
            body.bendTwistLambda1.fill(0, activeNodeStart, activeNodeEnd);
            body.bendTwistLambda2.fill(0, activeNodeStart, activeNodeEnd);
            body.bendTwistLambda3.fill(0, activeNodeStart, activeNodeEnd);
            body.orientationControlLambda.fill(0);
            // Contact multipliers belong to this fixed-step solve. Reusing a
            // positional lambda across frames without applying a matching
            // warm-start impulse makes both normal reaction and Coulomb
            // friction depend on how many projections happened previously.
            body._wallWitnessFrictionSolved = false;
            body.wallLambda.fill(0, activeSegmentStart, activeSegmentEnd);
            body.wallFrictionLambda.fill(0, activeSegmentStart, activeSegmentEnd);
            body.wallProjectionX.fill(0, activeNodeStart, activeNodeEnd);
            body.wallProjectionY.fill(0, activeNodeStart, activeNodeEnd);
            body.wallProjectionZ.fill(0, activeNodeStart, activeNodeEnd);
            body.toolProjectionX.fill(0, activeNodeStart, activeNodeEnd);
            body.toolProjectionY.fill(0, activeNodeStart, activeNodeEnd);
            body.toolProjectionZ.fill(0, activeNodeStart, activeNodeEnd);
            body.lastMaximumRawSpeed = 0;
            body.lastMaximumWallProjectionSpeed = 0;
            body.lastMaximumWallProjectionNode = -1;
            body.lastMaximumRejectedWallProjectionSpeed = 0;
            body.lastMaximumToolProjectionSpeed = 0;
            body.lastMaximumRejectedToolProjectionSpeed = 0;
            body.lastMaximumReconstructedSpeed = 0;
            this.#integrate(body);
        }
        if (connected && this.jointMotionMode === 'split-physical-bias') {
            beginKirchhoffSplitMotion(connected, this);
            if (connected._splitMotion.biasMaterialMode === 'coupled-compliance') {
                beginKirchhoffTwoChannelMotion(connected);
                beginKirchhoffTwoChannelRows(connected, this);
                connected._splitMotion.diagnostics.twoChannel = true;
                delete connected._splitMotion.diagnostics.biasElasticEnergyDelta;
            }
            initializeKirchhoffWallFrictionModes(connected, wallFrictionIncoming,
                { displacementToleranceMm: this.coupledContainmentTolerance * 0.2 });
        }
        recordTiming(this.timings.integrate, now() - phaseStart);
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.debugConstraintPhase?.('afterIntegrate', body);
        }

        phaseStart = now();
        let constraintSectionStart = phaseStart;
        for (let index = 0; index < this.bodies.length; index++) this.#applySweptCollision(this.bodies[index]);
        for (let index = 0; index < this.bodies.length; index++) this.#prepareWallContacts(this.bodies[index]);
        let narrowPhaseDuration = now() - phaseStart;

        phaseStart = now();
        const jointConstraint = this.#jointCoupledConstraint();
        if (jointConstraint) {
            this.lastCoupledSolver = 'joint';
            this.#solveJointPhysicalWithWallModes(jointConstraint);
            if (jointConstraint._splitMotion) {
                if (jointConstraint._jointLinearFailure || jointConstraint._jointTrialFailure)
                    jointConstraint._splitMotion.diagnostics.physicalFailure = structuredClone({
                        linear: jointConstraint._jointLinearFailure, trial: jointConstraint._jointTrialFailure
                    });
                if (jointConstraint._splitMotion.twoChannel) {
                    const split = jointConstraint._splitMotion;
                    // Both equations have just been solved in the same outer
                    // iterations. No second integration or material-bank swap.
                    split.physicalMaterialResidual = { ...measureKirchhoffTwoChannelMaterial(jointConstraint).physicalResidual };
                    split.diagnostics.physicalAccepted = split.diagnostics.biasAccepted = this.lastCoupledClosureConverged;
                    split.diagnostics.jointPasses = split.diagnostics.physicalPasses;
                    split.phase = 'complete';
                } else {
                    beginKirchhoffSplitBias(jointConstraint, this, this.lastCoupledClosureConverged);
                    this.#solveJointCoupledConstraints(jointConstraint);
                    if (jointConstraint._jointLinearFailure || jointConstraint._jointTrialFailure)
                        jointConstraint._splitMotion.diagnostics.biasFailure = structuredClone({
                            linear: jointConstraint._jointLinearFailure, trial: jointConstraint._jointTrialFailure
                        });
                    finishKirchhoffSplitBias(jointConstraint, this.lastCoupledClosureConverged);
                }
                const physical = this.#measureJointCoupledConstraints(jointConstraint, false);
                const split = jointConstraint._splitMotion;
                split.diagnostics.finalPhysicalResidualSettled = physical.settled;
                split.diagnostics.certified = split.diagnostics.physicalAccepted && split.diagnostics.biasAccepted &&
                    physical.settled && split.diagnostics.unverifiedHistoryKinds.length === 0;
                this.lastCoupledClosureConverged = split.diagnostics.certified;
            }
            jointConstraint._jointClosureConverged = this.lastCoupledClosureConverged;
        } else if (this.coupledSystem?.independentComponents && this.jointMotionMode === 'position-history') {
            const components = independentComponents;
            let allConverged = true;
            this.lastCoupledSolver = 'joint-components';
            for (const component of components) {
                if (component._jointClosureConverged === true && kirchhoffComponentBodies(component).every(body => body.sleeping)) continue;
                // Compatibility aliases name actual tools only. A singleton
                // has no invented outer body or artificial lumen constraint.
                if (component.bodies?.length === 2) {
                    component.innerBody = component.bodies[0];component.outerBody = component.bodies[1];
                }
                component._contactBlockSweeps = component._contactBlockIterations = 0;
                this.#solveJointPhysicalWithWallModes(component);
                component._jointClosureConverged = this.lastCoupledClosureConverged;
                allConverged &&= this.lastCoupledClosureConverged;
            }
            this.lastCoupledClosureConverged = allConverged;
        } else {
            if (this.containments.some(item => item.enabled)) this.lastCoupledSolver = 'partitioned';
            const iterationCount = this.maxPenetration > this.highPenetration
                ? this.penetrationIterations
                : this.iterations;
            for (let iteration = 0; iteration < iterationCount; iteration++) {
                for (let index = 0; index < this.sheaths.length; index++) this.#solveSheath(this.sheaths[index]);
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        body.debugConstraintPhase?.('afterSheath', body);
                    }
                }
                for (let index = 0; index < this.bodies.length; index++) this.#solveControls(this.bodies[index]);
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        body.debugConstraintPhase?.('afterControls', body);
                    }
                }
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        body.debugConstraintPhase?.('afterLengths', body);
                    }
                }
                for (let index = 0; index < this.bodies.length; index++) this.#solveBending(this.bodies[index]);
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        body.debugConstraintPhase?.('afterBending', body);
                    }
                }
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        body.debugConstraintPhase?.('afterDirections', body);
                    }
                }
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        this.bodies[index].debugConstraintPhase?.(
                            'afterRest',
                            this.bodies[index]
                        );
                    }
                }
                // Shape memory is deliberately solved after the first control
                // projection, but an unsupported catheter tip must not receive the
                // entire shape correction as a single-frame impulse. Rebalance the
                // compliant controls before the wall gets the final say.
                for (let index = 0; index < this.bodies.length; index++) this.#solveControls(this.bodies[index]);
                for (let index = 0; index < this.containments.length; index++) this.#solveContainment(this.containments[index]);
                for (let index = 0; index < this.toolContacts.length; index++) this.#solveToolContact(this.toolContacts[index]);
                for (let index = 0; index < this.bodies.length; index++) this.#solveWallContacts(this.bodies[index]);
                for (let index = 0; index < this.bodies.length; index++) this.#solveFoldLimits(this.bodies[index]);
                if (iteration + 1 === iterationCount) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        this.bodies[index].debugConstraintPhase?.(
                            'afterFold',
                            this.bodies[index]
                        );
                    }
                }
            }
            for (let index = 0; index < this.bodies.length; index++) {
                this.bodies[index].debugConstraintPhase?.('primary', this.bodies[index]);
            }
            // Let selected rods converge more quickly without advancing physical
            // time or modifying their constitutive parameters. A rod which does
            // not share an active Kirchhoff lumen keeps the original body-local
            // schedule exactly. Once two rods share a lumen, however, their
            // constitutive sweeps and the unilateral lumen contact are one
            // mechanical system: solving all wire sweeps and then all catheter
            // sweeps lets each member approach an incompatible free equilibrium
            // before contact reacts, which produces the alternating lateral wave
            // seen during over-the-wire feed.
            let maximumCoupledRelaxationPasses = 0;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body._coupledRelaxationActive = false;
                body.lastRelaxationPasses = 0;
            }
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                if (!constraint.enabled) {
                    continue;
                }
                const inner = constraint.innerBody;
                const outer = constraint.outerBody;
                inner._coupledRelaxationActive = true;
                outer._coupledRelaxationActive = true;
                maximumCoupledRelaxationPasses = Math.max(
                    maximumCoupledRelaxationPasses,
                    Math.max(0, Math.floor(inner.relaxationPasses ?? 0)),
                    Math.max(0, Math.floor(outer.relaxationPasses ?? 0))
                );
            }
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                if (body._coupledRelaxationActive) continue;
                const relaxationPasses = Math.max(
                    0,
                    Math.floor(body.relaxationPasses ?? 0)
                );
                for (let pass = 0; pass < relaxationPasses; pass++) {
                    if (body.sleeping) break;
                    this.#solveRelaxationPass(body, pass);
                    body.lastRelaxationPasses = pass + 1;
                }
            }
            this.lastCoupledRelaxationPasses = 0;
            for (
                let pass = 0;
                pass < maximumCoupledRelaxationPasses;
                pass++
            ) {
                let solvedBody = false;
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    if (
                        !body._coupledRelaxationActive ||
                        body.sleeping ||
                        pass >= Math.max(0, Math.floor(body.relaxationPasses ?? 0))
                    ) continue;
                    this.#solveRelaxationPass(body, pass);
                    body.lastRelaxationPasses = pass + 1;
                    // Project the shared lumen immediately after each member's
                    // constitutive update. Waiting until both free-rod energies
                    // have run creates an avoidable Jacobi-like oscillation; this
                    // is the block Gauss-Seidel ordering of the coupled system.
                    this.#solveKirchhoffContainmentsForBody(body, false);
                    solvedBody = true;
                }
                if (!solvedBody) break;
                this.lastCoupledRelaxationPasses = pass + 1;
            }
            let constraintSectionEnd = now();
            recordTiming(
                this.timings.constraintPrimary,
                constraintSectionEnd - constraintSectionStart
            );
            constraintSectionStart = constraintSectionEnd;
            let bodyClosureStageStart = constraintSectionStart;
            // Later bend, shape and contact projections can perturb segment lengths.
            // Finish the substep with inexpensive structural polishing so callers
            // never observe a transiently stretched rod between fixed steps.
            this.lastLengthPolishPasses = 0;
            for (let pass = 0; pass < 16; pass++) {
                this.lastLengthPolishPasses = pass + 1;
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
            let bodyClosureStageEnd = now();
            recordTiming(
                this.timings.constraintBodyLengthPolish,
                bodyClosureStageEnd - bodyClosureStageStart
            );
            bodyClosureStageStart = bodyClosureStageEnd;
            if (this._wallRepairPenetration.length < this.bodies.length) {
                this._wallRepairPenetration = new Float32Array(
                    this.bodies.length
                );
                this._wallRepairEligible = new Uint8Array(this.bodies.length);
            }
            const wallRepairPenetration = this._wallRepairPenetration;
            const wallRepairEligible = this._wallRepairEligible;
            this.wallRepairResiduals.fill(0);
            this.wallRepairWorstSegments.fill(-1);
            this.wallRepairWorstBodies.fill(-1);
            this.lastWallRepairPasses = 0;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                let movingLumenOwnsInner = false;
                wallRepairEligible[index] = movingLumenOwnsInner ? 0 : 1;
            }
            for (
                let correctionPass = 0;
                correctionPass < MAX_WALL_CORRECTION_PASSES;
                correctionPass++
            ) {
                this.lastWallRepairPasses = correctionPass + 1;
                let activePenetration = 0;
                let repairablePenetration = 0;
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    // Bodies owned by the later moving-lumen closure are never
                    // written in this phase. Their exact first-pass penetration
                    // therefore remains exact for every subsequent repair pass.
                    const bodyPenetration =
                        correctionPass > 0 && !wallRepairEligible[index]
                            ? wallRepairPenetration[index]
                            : this.#refreshActiveWallContacts(body);
                    wallRepairPenetration[index] = bodyPenetration;
                    if (wallRepairEligible[index]) {
                        repairablePenetration = Math.max(
                            repairablePenetration,
                            bodyPenetration
                        );
                    }
                    if (bodyPenetration > activePenetration) {
                        activePenetration = bodyPenetration;
                        this.wallRepairWorstBodies[correctionPass] = index;
                        this.wallRepairWorstSegments[correctionPass] =
                            body._wallRefreshWorstSegment ?? -1;
                    }
                }
                this.wallRepairResiduals[correctionPass] = activePenetration;
                // A moving lumen owns the contained rod's wall response later in
                // the coupled closure. Re-querying the same skipped body for all
                // 16 repair passes cannot change any position or multiplier, so
                // stop as soon as no body eligible in this phase needs repair.
                if (repairablePenetration <= 0.02) break;
                for (let index = 0; index < this.bodies.length; index++) {
                    if (
                        !wallRepairEligible[index] ||
                        wallRepairPenetration[index] <= 0.02
                    ) continue;
                    const body = this.bodies[index];
                    this.#solveFoldLimits(body);
                    this.#prepareWallContacts(body);
                    this.#solveWallContacts(body);
                    if (correctionPass + 1 < MAX_WALL_CORRECTION_PASSES) {
                        this.#solveLengthsGlobal(body);
                    }
                }
            }
            bodyClosureStageEnd = now();
            recordTiming(
                this.timings.constraintBodyWallRepair,
                bodyClosureStageEnd - bodyClosureStageStart
            );
            bodyClosureStageStart = bodyClosureStageEnd;
            // Later wall and fold corrections can separate the two centerlines.
            // Finish with exactly one radial projection of the body selected by
            // the material coupling. Repeating structural projections here caused
            // the catheter to collapse at its open distal transition.
            let needsSecondFinalContainmentPass = false;
            const finalContainmentPasses = needsSecondFinalContainmentPass ? 2 : 1;
            for (let pass = 0; pass < finalContainmentPasses; pass++) {
                for (let index = 0; index < this.containments.length; index++) {
                    const constraint = this.containments[index];
                    continue;

                }
            }
            // A hard radial projection can leave the contained rod with a large
            // length error or an almost reversed hinge. Alternate one-way lumen
            // projection with the inner rod's structure before body-local wall
            // polishing. This protects the distal capture transition from a single
            // unrestricted correction.
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                continue;

            }
            bodyClosureStageEnd = now();
            recordTiming(
                this.timings.constraintBodyPrePost,
                bodyClosureStageEnd - bodyClosureStageStart
            );
            bodyClosureStageStart = bodyClosureStageEnd;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                let settledPostPasses = 0;
                body.lastPostStabilizationPasses = 0;
                body.lastPostStabilizationResidual = Infinity;
                for (let pass = 0; pass < body.postStabilizationPasses; pass++) {
                    for (let node = body.activeStart; node <= body.activeEnd; node++) {
                        body.postPassStartX[node] = body.x[node];
                        body.postPassStartY[node] = body.y[node];
                        body.postPassStartZ[node] = body.z[node];
                    }
                    this.#solveControls(body);
                    // Direction memory shares nodes with positional controls. A
                    // direction pass can therefore reopen the material anchor;
                    // rebalance controls before global length/contact polishing,
                    // matching the ordering used by the primary XPBD iterations.
                    this.#solveControls(body);
                    this.#solveLengthsGlobal(body);
                    if (body.postStabilizeBending) {
                        this.#solveBending(body);
                        this.#solveLengthsGlobal(body);
                    }
                    // Restore the signed material side once after unsigned
                    // structural bending; applying it both before and after the
                    // same pass double-counts the intrinsic moment.
                    this.#prepareWallContacts(body);
                    this.#solveWallContacts(body);
                    // Length and wall projection can recreate a sharp hinge at a
                    // material transition. Keep the bend limit as the last
                    // angular operation of every stabilization pass, then restore
                    // material length so the correction cannot become axial
                    // energy on the next frame.
                    this.#solveFoldLimits(body);
                    this.#solveLengthsGlobal(body);
                    this.#solveFoldLimits(body);
                    let residual = 0;
                    for (let node = body.activeStart; node <= body.activeEnd; node++) {
                        residual = Math.max(residual, magnitude3(
                            body.x[node] - body.postPassStartX[node],
                            body.y[node] - body.postPassStartY[node],
                            body.z[node] - body.postPassStartZ[node]
                        ));
                    }
                    body.lastPostStabilizationPasses = pass + 1;
                    body.lastPostStabilizationResidual = residual;
                    if (
                        pass + 1 >= body.postStabilizationMinPasses &&
                        residual <= body.postStabilizationTolerance &&
                        !this.#hasLengthErrorOver(body, 0.002)
                    ) {
                        settledPostPasses++;
                        if (settledPostPasses >= body.postStabilizationSettledPasses) break;
                    } else {
                        settledPostPasses = 0;
                    }
                }
                this.#solveFoldLimits(body);
                this.#solveLengthsGlobal(body);
                this.#solveFoldLimits(body);
                // Do not expose a frame after structural/contact polishing has
                // reopened a positional material anchor. Otherwise its displaced
                // pose becomes the next frame's refreshed target and shape memory
                // ratchets the catheter along the vessel despite zero user input.
                this.#solveControls(body);
                body.debugConstraintPhase?.('final', body);
            }
            bodyClosureStageEnd = now();
            recordTiming(
                this.timings.constraintBodyPostStabilization,
                bodyClosureStageEnd - bodyClosureStageStart
            );
            constraintSectionEnd = now();
            recordTiming(
                this.timings.constraintBodyClosure,
                constraintSectionEnd - constraintSectionStart
            );
            constraintSectionStart = constraintSectionEnd;
            for (let index = 0; index < this.sheaths.length; index++) {
                this.#solveSheath(this.sheaths[index]);
            }
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body.debugConstraintPhase?.('closureAfterSheath', body);
            }
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                if (body.postStabilizationPasses <= 0) continue;
                this.#transportDistalLengthError(
                    body,
                    body.collisionStartSegment,
                    body.distalLengthTransportMaxCorrection
                );
            }
            // The dominant catheter can still move during its final wall solve.
            // Advect the contained wire by that same local centerline displacement
            // before refreshing the lumen projection. Otherwise the next frame
            // converts the mismatch into an alternating radial kick.
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                continue;

            }
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body.debugConstraintPhase?.('closureAfterCarry', body);
            }
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                continue;

            }
            for (let index = 0; index < this.bodies.length; index++) {
                this.#limitFrameDisplacement(this.bodies[index]);
            }
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body.debugConstraintPhase?.('closureAfterLimit', body);
            }
            // Settle each rod against its wall before the final lumen closure. No
            // outer-catheter projection may run after that closure, otherwise the
            // lumen can move away from an already settled guidewire.
            let finalStructuralClosurePasses = 8;
            const hasActiveKirchhoffContainment = this.containments.some(constraint => constraint.enabled);
            let activeRodNodes = 0;
            for (const body of this.bodies) {
                activeRodNodes += body.activeEnd - body.activeStart + 1;
                finalStructuralClosurePasses = Math.max(
                    finalStructuralClosurePasses,
                    body.finalStructuralClosurePasses ?? 8
                );
            }
            if (hasActiveKirchhoffContainment) {
                finalStructuralClosurePasses = Math.max(
                    finalStructuralClosurePasses, this.coupledClosureMaxPasses * 2
                );
            }
            this.lastCoupledClosurePasses = 0;
            this.lastCoupledClosureConverged = false;
            this.lastCoupledContainmentResidual = 0;
            this.lastCoupledContactPasses = 0;
            if (this.captureCoupledClosureTrace) {
                this.coupledClosureTrace.length = 0;
            }
            for (let bodyIndex = 0; bodyIndex < this.bodies.length; bodyIndex++) {
                const body = this.bodies[bodyIndex];
                if (hasActiveKirchhoffContainment) {
                    for (
                        let node = body.activeStart;
                        node <= body.activeEnd;
                        node++
                    ) {
                        body.coupledClosureStartX[node] = body.x[node];
                        body.coupledClosureStartY[node] = body.y[node];
                        body.coupledClosureStartZ[node] = body.z[node];
                    }
                }
            }
            this._inCoupledClosure = true;
            for (let pass = 0; pass < finalStructuralClosurePasses; pass++) {
                this.lastCoupledClosurePasses = pass + 1;
                if (this.captureCoupledClosureTrace || hasActiveKirchhoffContainment) {
                    for (let index = 0; index < this.bodies.length; index++) {
                        const body = this.bodies[index];
                        for (
                            let node = body.activeStart;
                            node <= body.activeEnd;
                            node++
                        ) {
                            body.postPassStartX[node] = body.x[node];
                            body.postPassStartY[node] = body.y[node];
                            body.postPassStartZ[node] = body.z[node];
                        }
                    }
                }
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    // The material adaptation constraint is the Kirchhoff rod's
                    // inextensibility constraint. It must participate in the last
                    // coupled closure even when the body does not use the extra
                    // post-stabilization passes (the guidewire normally does not).
                    // Otherwise the outlet, wall and lumen projections below are
                    // the final writers of its positions and expose an axially
                    // stretched segment until the next fixed step.
                    {
                        this.#solveBending(body);
                        // Constitutive bend first, unilateral safety bound second,
                        // then adaptation.  This leaves one coherent orientation
                        // state for the centerline instead of letting material
                        // energy immediately undo the fold projection.
                        this.#solveFoldLimits(body);
                        this.#solveLengthsGlobal(body);
                    }
                    // Length/adaptation is allowed to redistribute the inlet
                    // reaction, but the Eulerian introducer sample is itself a
                    // member of this coupled closure. Leaving its control solve
                    // outside the loop lets every later length pass pull the
                    // catheter backwards and accumulate axial compression.
                    this.#solveControls(body);
                    this.#prepareWallContacts(body);
                    this.#solveWallContacts(body);
                    // The final Newton-like closure updates both constitutive rods
                    // before evaluating their shared material contact once below.
                    // Projecting the same symmetric contact after each individual
                    // body made the result depend on body array order and counted
                    // one physical constraint three times per pass.
                }
                // A contact residual needs another contact sweep, not necessarily
                // another free-rod Newton solve. Polish the coupled contact block
                // at the current material iterate; lengths of BOTH complete rods
                // are checked below and reopen the global solve when necessary.
                // Each outer iterate includes the complete direct solves for both
                // rods, so contact reactions propagate beyond the overlap.
                let contactPassLimit = this.coupledContactMaxPasses;
                let previousContactResidual = Infinity;
                for (let contactPass = 0; contactPass < contactPassLimit; contactPass++) {
                    let contactCount = 0;
                    for (const constraint of this.containments) {
                        if (!constraint.enabled) continue;
                        this.#solveKirchhoffContainment(constraint, true, false);
                        contactCount += constraint.kirchhoffContacts.length;
                    }
                    if (!hasActiveKirchhoffContainment) break;
                    this.lastCoupledContactPasses++;
                    // Length error already requires a global rod update. Avoid
                    // traversing (or polishing) a contact network it will reopen.
                    if (this.bodies.some(body => this.#hasLengthErrorOver(body, this.coupledLengthTolerance))) break;
                    let contactMotionSettled = true;
                    for (const constraint of this.containments) {
                        if (!constraint.enabled) continue;
                        constraint.kirchhoffContactMotion = measureKirchhoffContactMotion(constraint);
                        if (constraint.kirchhoffContactMotion > this.coupledContainmentTolerance) {
                            contactMotionSettled = false;
                        }
                    }
                    // Large motion of the coupled coordinates also requires the
                    // next material iterate. Only the remaining contact residual
                    // benefits from additional inexpensive inner sweeps.
                    if (!contactMotionSettled) break;
                    // Bound inner work by the size of the global rod block (six
                    // rows per node). Long overlaps return to the material solve
                    // sooner; small overlaps can finish cheaply in this loop.
                    contactPassLimit = Math.min(contactPassLimit, Math.max(
                        2, Math.ceil(6 * activeRodNodes / Math.max(1, contactCount))
                    ));
                    let contactsSettled = true;
                    let contactResidual = 0;
                    for (const constraint of this.containments) {
                        if (!constraint.enabled) continue;
                        constraint.kirchhoffSolverResidual = this.#kirchhoffContactSolverResidual(constraint);
                        contactResidual = Math.max(contactResidual, constraint.kirchhoffSolverResidual);
                        if (constraint.kirchhoffSolverResidual > this.coupledContainmentTolerance) {
                            contactsSettled = false;
                        }
                    }
                    if (contactsSettled) break;
                    // Contact polishing cannot repair material length. Return to
                    // the full rod response as soon as it is required, or when
                    // the contact block stops making progress on its own.
                    if (contactResidual >= previousContactResidual * 0.95) break;
                    previousContactResidual = contactResidual;
                }
                let coupledResidualSettled = true;
                const tracedBodies = this.captureCoupledClosureTrace ? [] : null;
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    const lengthError = this.#hasLengthErrorOver(
                        body,
                        hasActiveKirchhoffContainment
                            ? this.coupledLengthTolerance
                            : 0.002
                    );
                    let maximumPositionDelta = 0;
                    let maximumRelativeLengthError = 0;
                    if (this.captureCoupledClosureTrace) {
                        for (
                            let node = body.activeStart;
                            node <= body.activeEnd;
                            node++
                        ) {
                            maximumPositionDelta = Math.max(
                                maximumPositionDelta,
                                magnitude3(
                                    body.x[node] - body.postPassStartX[node],
                                    body.y[node] - body.postPassStartY[node],
                                    body.z[node] - body.postPassStartZ[node]
                                )
                            );
                        }
                        for (
                            let segment = body.activeStart;
                            segment < body.activeEnd;
                            segment++
                        ) {
                            const restLength = Math.max(
                                EPSILON,
                                body.restLength[segment]
                            );
                            maximumRelativeLengthError = Math.max(
                                maximumRelativeLengthError,
                                Math.abs(
                                    magnitude3(
                                        body.x[segment + 1] - body.x[segment],
                                        body.y[segment + 1] - body.y[segment],
                                        body.z[segment + 1] - body.z[segment]
                                    ) - restLength
                                ) / restLength
                            );
                        }
                    }
                    coupledResidualSettled = coupledResidualSettled && !lengthError;
                    tracedBodies?.push({
                        id: body.id,
                        lengthError,
                        maximumPositionDelta,
                        maximumRelativeLengthError
                    });
                }
                let tracedContainmentViolation = 0;
                let tracedSideViolation = 0;
                let tracedPortalViolation = 0;
                // The exact post-projection containment scan cannot change any
                // generalized coordinate. If material length already requires
                // another global sweep, measuring every lumen segment cannot affect
                // the convergence decision and only repeats the contact geometry
                // traversal. Defer it until containment is the remaining gate (or
                // tracing explicitly requests the value).
                const measureContainmentResidual =
                    coupledResidualSettled || this.captureCoupledClosureTrace;
                for (
                    let constraintIndex = 0;
                    constraintIndex < this.containments.length;
                    constraintIndex++
                ) {
                    const constraint = this.containments[constraintIndex];
                    if (
                        constraint.enabled &&
                        measureContainmentResidual
                    ) {
                        constraint.kirchhoffMaxViolation =
                            this.#measureKirchhoffCoupledContainmentViolation(
                                constraint
                            );
                        constraint.kirchhoffSolverResidual = this.#kirchhoffContactSolverResidual(constraint);
                        constraint.kirchhoffContactMotion = measureKirchhoffContactMotion(constraint);
                        tracedContainmentViolation = Math.max(
                            tracedContainmentViolation,
                            constraint.kirchhoffMaxViolation
                        );
                        tracedSideViolation = Math.max(
                            tracedSideViolation,
                            constraint.kirchhoffMeasuredSideViolation ?? 0
                        );
                        tracedPortalViolation = Math.max(
                            tracedPortalViolation,
                            constraint.kirchhoffMeasuredPortalViolation ?? 0
                        );
                    }
                    if (
                        constraint.enabled &&
                        measureContainmentResidual &&
                        Math.max(constraint.kirchhoffSolverResidual, constraint.kirchhoffContactMotion) >
                            this.coupledContainmentTolerance
                    ) {
                        coupledResidualSettled = false;
                    }
                }
                if (this.captureCoupledClosureTrace) {
                    this.coupledClosureTrace.push({
                        pass: pass + 1,
                        settled: coupledResidualSettled,
                        containmentViolation: tracedContainmentViolation,
                        contactPasses: this.lastCoupledContactPasses,
                        contactMotion: Math.max(0, ...this.containments.filter(c => c.enabled).map(c => c.kirchhoffContactMotion ?? 0)),
                        solverResidual: Math.max(0, ...this.containments.filter(c => c.enabled).map(c => c.kirchhoffSolverResidual ?? 0)),
                        sideViolation: tracedSideViolation,
                        portalViolation: tracedPortalViolation,
                        spatialPortalViolation: this.containments.find(
                            (constraint) =>
                                constraint.enabled
                        )?.kirchhoffMeasuredSpatialPortalViolation ?? 0,

                        worstSide: this.containments.find(
                            (constraint) =>
                                constraint.enabled
                        )?.kirchhoffMeasuredWorstSide ?? null,
                        bodies: tracedBodies
                    });
                }
                if (measureContainmentResidual) {
                    this.lastCoupledContainmentResidual =
                        tracedContainmentViolation;
                }
                if (coupledResidualSettled) {
                    this.lastCoupledClosureConverged = true;
                    break;
                }
            }
            this._inCoupledClosure = false;
            if (hasActiveKirchhoffContainment) {
                // The final coupled closure is a quasi-static nonlinear solve, not
                // an impulse integrator. Once operator transport stops, carry its
                // net projection into the previous pose so velocity reconstruction
                // cannot turn repeated equilibrium corrections into fresh kinetic
                // energy. During feed the catheter publishes retention=1 and this
                // blend becomes zero, preserving the real material transport.
                for (let bodyIndex = 0; bodyIndex < this.bodies.length; bodyIndex++) {
                    const body = this.bodies[bodyIndex];
                    let transportRetention = 0;
                    for (
                        let constraintIndex = 0;
                        constraintIndex < this.containments.length;
                        constraintIndex++
                    ) {
                        const constraint = this.containments[constraintIndex];
                        if (
                            !constraint.enabled ||
                            (
                                constraint.innerBody !== body &&
                                constraint.outerBody !== body
                            )
                        ) continue;
                        transportRetention = Math.max(
                            transportRetention,
                            constraint.outerBody.projectionVelocityRetention
                        );
                    }
                    const quasiStaticBlend = transportRetention < 0.5 ? 1 : 0;
                    if (quasiStaticBlend <= EPSILON) continue;
                    for (
                        let node = body.activeStart;
                        node <= body.activeEnd;
                        node++
                    ) {
                        body.previousX[node] += (
                            body.x[node] - body.coupledClosureStartX[node]
                        ) * quasiStaticBlend;
                        body.previousY[node] += (
                            body.y[node] - body.coupledClosureStartY[node]
                        ) * quasiStaticBlend;
                        body.previousZ[node] += (
                            body.z[node] - body.coupledClosureStartZ[node]
                        ) * quasiStaticBlend;
                    }
                }
            }
            constraintSectionEnd = now();
            recordTiming(
                this.timings.constraintCoupledClosure,
                constraintSectionEnd - constraintSectionStart
            );
            constraintSectionStart = constraintSectionEnd;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body.debugConstraintPhase?.('closureAfterWall', body);
            }
            // A moving lumen boundary and its material-length constraint form one
            // coupled system. Close that system per inner rod, without re-solving
            // the outer catheter: convergence of a guidewire must not multiply the
            // catheter's intrinsic-bend or wall passes. Each sweep first repairs
            // inner structure, then applies unilateral containment. Convergence is
            // measured after containment, so no unverified projection follows it.
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                continue;

            }
            constraintSectionEnd = now();
            recordTiming(
                this.timings.constraintMovingClosure,
                constraintSectionEnd - constraintSectionStart
            );
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                body.debugConstraintPhase?.('closureEnd', body);
            }
        }
        for (
            let constraintIndex = 0;
            constraintIndex < this.containments.length;
            constraintIndex++
        ) {
            const constraint = this.containments[constraintIndex];
            if (
                !constraint._kirchhoffStepOpen
            ) continue;
            constraint.manifold.endStep();
            constraint._kirchhoffStepOpen = false;
        }
        recordTiming(this.timings.constraints, now() - phaseStart);

        const transientMaxPenetration = this.maxPenetration;
        phaseStart = now();
        this.contactCount = 0;
        this.maxPenetration = 0;
        this.settledContactBodyId = null;
        this.settledContactSegment = -1;
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.settledMaxPenetration = this.#refreshActiveWallContacts(body);
            for (let segment = 0; segment < body.segmentCount; segment++) {
                body.wallFrictionLambda[segment] = body.wallActive[segment]
                    ? body.wallLambda[segment]
                    : 0;
            }
        }
        this.settledMaxPenetration = this.maxPenetration;
        this.maxPenetration = Math.max(transientMaxPenetration, this.settledMaxPenetration);
        narrowPhaseDuration += now() - phaseStart;
        recordTiming(this.timings.narrowPhase, narrowPhaseDuration);

        phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) this.#updateVelocityAndFriction(this.bodies[index]);
        for (let index = 0; index < this.containments.length; index++) {
            if (this.containments[index]._splitMotion) continue;
            this.#stabilizeContainmentVelocity(this.containments[index]);
        }
        for (let index = 0; index < this.toolContacts.length; index++) {
            if (this.toolContacts[index].bodyA._splitPhysicalMotion && this.toolContacts[index].bodyB._splitPhysicalMotion) continue;
            this.#stabilizeToolContactVelocity(this.toolContacts[index]);
        }
        for (let index = 0; index < this.bodies.length; index++) {
            this.#limitVelocity(this.bodies[index]);
        }
        if (jointConstraint?._splitMotion) {
            syncKirchhoffSplitVelocity(jointConstraint);
            const finalPhysical = this.#measureJointCoupledConstraints(jointConstraint, false);
            this.lastCoupledClosureConverged = commitKirchhoffSplitHistory(jointConstraint, this, finalPhysical);
        }
        // The connected pair sleeps atomically. Sleeping its members on
        // different frames made each wake the other at the next step, resetting
        // their counters forever despite an already settled component.
        if (this.lastCoupledSolver === 'joint' && this.lastCoupledClosureConverged &&
            this.bodies.every(body => body.sleepCounter >= body.sleepFrames)) {
            for (const body of this.bodies) this.#sleepBody(body);
        }
        recordTiming(this.timings.velocity, now() - phaseStart);

        this.stepCount++;
        if (!transactional) recordTiming(this.timings.total, now() - totalStart);
    }

    #jointCoupledConstraint() {
        if (!this.coupledSystem || this.bodies.length !== 2) return null;
        let selected = null;
        for (const constraint of this.containments) {
            if (!constraint.enabled) continue;
            if (selected) return null;
            selected = constraint;
        }
        return selected && this.bodies.includes(selected.innerBody) && this.bodies.includes(selected.outerBody)
            ? selected : null;
    }

    // One material/contact Newton direction per iteration. Boundary rows take
    // part in that same solve; no separate free-rod update follows it.
    #solveJointPhysicalWithWallModes(constraint) {
        if (this.coupledSystem.wallWitnesses && kirchhoffComponentBodies(constraint).some(
            body => body.wallStaticFriction !== body.wallKineticFriction)) {
            beginKirchhoffWallWitnessStep(constraint);
            beginKirchhoffWallWitnessFrictionModes(constraint, { dt: this.fixedDt, step: this.stepCount,
                displacementToleranceMm: this.coupledContainmentTolerance });
            // Prediction and operator feed have already run exactly once.
            // Retry only this component's complete material/contact solve from
            // its unchanged predicted pose, with zero applied wall reactions.
            const base = this.#captureJointTrial(constraint, { world: this });
            const attempts = [];
            for (;;) {
                this.#solveJointCoupledConstraints(constraint);
                const measurement = this.#measureJointCoupledConstraints(constraint, false);
                const decision = evaluateKirchhoffWallWitnessFrictionCandidate(constraint,
                    measurement.wallWitnessFriction._batch, { converged: this.lastCoupledClosureConverged && measurement.settled });
                attempts.push({ status: decision.status, attempt: decision.attempt,
                    contacts: decision.contacts, passes: this.lastCoupledClosurePasses });
                if (!decision.restart) {
                    this.lastCoupledClosureConverged &&= decision.accepted;
                    if (decision.accepted) commitKirchhoffWallWitnessFrictionModes(constraint, decision);
                    constraint._wallWitnessFrictionAttempts = attempts;
                    return;
                }
                this.#restoreJointTrial(base);
                prepareKirchhoffWallWitnessFrictionRetry(constraint, decision);
            }
        }
        if (!constraint._splitMotion?.wallFrictionModes) {
            this.#solveJointCoupledConstraints(constraint);
            return;
        }
        // All mode candidates start from the same post-integration mechanics.
        // This transaction also restores material/contact multipliers, pose,
        // physical velocity and reaction history. It deliberately retains
        // actual work counters; a discarded static attempt still costs time.
        let base = captureKirchhoffSplitStep(this);
        const attempts = [];
        let physicalPasses = 0;
        for (;;) {
            this.#solveJointCoupledConstraints(constraint);
            physicalPasses += constraint._splitMotion.diagnostics.physicalPasses;
            const state = this.lastCoupledClosureConverged
                ? this.#measureJointCoupledConstraints(constraint, false) : null;
            const decision = evaluateKirchhoffWallFrictionCandidate(constraint, state?.wallPhysicalFriction?._batch,
                { converged: this.lastCoupledClosureConverged && state?.settled === true });
            attempts.push({ attempt: decision.attempt, status: decision.status,
                passes: constraint._splitMotion.diagnostics.physicalPasses,
                contacts: decision.contacts.length,
                staticContacts: decision.contacts.filter(contact => contact.mode === 'stick').length,
                slidingContacts: decision.contacts.filter(contact => contact.mode === 'slide').length,
                issues: structuredClone(decision.issues) });
            if (!decision.restart) {
                const diagnostics = constraint._splitMotion.diagnostics;
                diagnostics.physicalPasses = physicalPasses;
                diagnostics.wallFrictionAttempts = attempts;
                diagnostics.wallFrictionRestarts = attempts.length - 1;
                this.lastCoupledClosureConverged = this.lastCoupledClosureConverged && decision.accepted;
                return;
            }
            restoreKirchhoffSplitStep(this, base);
            prepareKirchhoffWallFrictionRetry(constraint, decision);
            base = captureKirchhoffSplitStep(this);
        }
    }

    #solveJointCoupledConstraints(constraint) {
        const began = now();
        constraint._reuseCandidateEvaluation = this.coupledSystem.reuseCandidateEvaluation !== false;
        const bodies = kirchhoffComponentBodies(constraint);
        const hasLumen = !constraint.bodies || constraint.containment === constraint;
        for (const body of bodies) {
            if (body.sleeping) body.wake();
            for (let node = body.activeStart; node <= body.activeEnd; node++) {
                body.coupledClosureStartX[node] = body.x[node];
                body.coupledClosureStartY[node] = body.y[node];
                body.coupledClosureStartZ[node] = body.z[node];
            }
            body.lastRelaxationPasses = 0;
        }
        beginKirchhoffCoupledBoundaryStep(constraint);
        constraint._usesWallWitnesses = this.coupledSystem.wallWitnesses === true;
        if (constraint._usesWallWitnesses) {
            if (constraint._splitMotion) throw new Error('Wall witnesses require the position-history component solve');
            for (const body of bodies) body._wallWitnessFrictionSolved = true;
            beginKirchhoffWallWitnessStep(constraint);
        }
        beginKirchhoffCoupledFoldStep(constraint);
        beginKirchhoffCoupledOrientationStep(constraint);
        beginKirchhoffExternalFrictionStep(constraint);
        for (const contact of this.toolContacts) {
            if (!bodies.includes(contact.bodyA) || !bodies.includes(contact.bodyB)) continue;
            contact.lambdas.fill(0);
            beginKirchhoffToolReactionStep(contact);
        }
        this.lastLengthPolishPasses = this.lastWallRepairPasses = this.lastCoupledRelaxationPasses = 0;
        this.lastCoupledClosurePasses = this.lastCoupledContactPasses = 0;
        this.lastCoupledClosureConverged = false;
        this.lastCoupledContainmentResidual = 0;
        constraint._jointLinearFailure = null;
        if (this.captureCoupledClosureTrace) this.coupledClosureTrace.length = 0;
        this._inCoupledClosure = true;
        const options = constraint._jointOptions ??= {};
        options.tolerance = this.coupledContainmentTolerance * 0.2;
        options.resolveNormalLoads = true;
        // Proximal factorization shift only: every QP/refinement and applied
        // correction is checked against the original unshifted equations.
        options.numericalShift = 1e-8;
        options.groups ??= [];
        let previousMerit = Infinity;
        // Local to this closure: a new timestep/component/input does not
        // inherit an old contact state's preferred scale.
        let acceptedLevel = 0;
        let previousNonConeSettled = false, previousMaxCone = Infinity;
        constraint._jointTrialFailure = null;
        for (let pass = 0; pass < this.coupledClosureMaxPasses * 2; pass++) {
            this.lastCoupledClosurePasses = pass + 1;
            if (constraint._splitMotion) constraint._splitMotion.diagnostics[constraint._splitMotion.phase + 'Passes'] = pass + 1;
            for (const body of bodies) {
                for (let node = body.activeStart; node <= body.activeEnd; node++) {
                    body.postPassStartX[node] = body.x[node];
                    body.postPassStartY[node] = body.y[node];
                    body.postPassStartZ[node] = body.z[node];
                }
                const controlled = body.orientationControlSegment;
                if (body.orientationControlCompliance === 0 && controlled >= body.activeStart &&
                    controlled < Math.min(body.segmentCount, body.activeEnd)) {
                    // Prescribe before any frame-dependent contact Jacobians.
                    prescribeKirchhoffSplitOrientation(constraint, body, controlled);
                    body.orientationX[controlled] = body.orientationControlX;
                    body.orientationY[controlled] = body.orientationControlY;
                    body.orientationZ[controlled] = body.orientationControlZ;
                    body.orientationW[controlled] = body.orientationControlW;
                }
                this.#prepareWallContacts(body);
            }
            if (pass === 0 && constraint._splitMotion?.phase === 'bias') {
                // The physical solve can already leave geometry and strain
                // within every nonlinear acceptance gate. Certify that fresh
                // state before requesting a tighter, unnecessary bias solve.
                buildKirchhoffCoupledFoldRows(constraint, this.fixedDt);
                const initial = this.#measureJointCoupledConstraints(constraint, false);
                constraint._splitMotion.diagnostics.biasInitialStateSettled = initial.settled;
                constraint._splitMotion.diagnostics.biasInitialMerit = initial.merit;
                if (initial.settled) {
                    this.lastCoupledClosureConverged = true;
                    break;
                }
            }
            // Contact feet/features can migrate between linearizations. The
            // trial must decrease the merit of THIS base state, evaluated
            // with the same fresh geometry, not a stale previous feature set.
            if (pass > 0) {
                const previous = this.#measureJointCoupledConstraints(constraint, false, null, true);
                // Measurement scratch is borrowed: retain scalar values before
                // evaluating candidates, which overwrite that same object.
                previousMerit = previous.merit;
                Object.assign(constraint._jointBaseMeritTerms ??= {}, previous.meritTerms);
                Object.assign(constraint._jointBaseBoundaryWorst ??= {}, constraint._jointBoundaryWorst);
                previousNonConeSettled = previous.nonConeSettled;
                previousMaxCone = previous.maximumConeViolation;
            }
            const assemblyStarted = now();
            constraint.kirchhoffContacts.length = 0;
            constraint.kirchhoffMaxViolation = 0;
            // An empty lumen batch does not remove either rod's material and
            // boundary equations (e.g. while the catheter is outside entry).
            if (hasLumen) this.#collectKirchhoffContainmentGeometry(constraint, true);
            for (const record of constraint.kirchhoffContacts) buildKirchhoffContactNormalGradients(constraint, record);
            options.additionalRows = collectKirchhoffCoupledBoundaryRows(
                constraint, this.sheaths, this.fixedDt, this.contactActivation, !constraint._usesWallWitnesses
            );
            if (constraint._usesWallWitnesses) collectKirchhoffWallWitnessRows(constraint, this.contactField, options.additionalRows, this.fixedDt);
            for (const contact of this.toolContacts) if (bodies.includes(contact.bodyA) && bodies.includes(contact.bodyB))
                this.#solveToolContact(contact, options.additionalRows, constraint);
            appendKirchhoffSplitSweeps(constraint, options.additionalRows);
            appendKirchhoffSplitPointWalls(constraint, this, options.additionalRows);
            buildKirchhoffCoupledFoldRows(constraint, this.fixedDt, options.additionalRows);
            const orientationBatch = buildKirchhoffCoupledOrientationRows(constraint, this.fixedDt);
            appendKirchhoffCoupledOrientationRows(orientationBatch, options.additionalRows);
            prepareKirchhoffSplitLumenRows(constraint);
            prepareKirchhoffSplitBoundaryRows(constraint, options.additionalRows);
            options.materialStrainOffsets = constraint._splitMotion?.materialStrainOffsets ?? undefined;
            options.groups.length = 0;
            const frictionBatch = constraint.surfaceFrictionEnabled === false ? null
                : buildKirchhoffCoupledFrictionRows(constraint, this.fixedDt, constraint._jointFrictionBatch ??= {});
            if (frictionBatch) appendKirchhoffCoupledFrictionRows(frictionBatch, options.additionalRows, options.groups);
            const externalBatch = buildKirchhoffExternalFrictionRows(constraint, options.additionalRows,
                this.fixedDt, constraint._jointExternalFrictionBatch ??= {});
            appendKirchhoffExternalFrictionRows(externalBatch, options.additionalRows, options.groups);
            const wallBatch = constraint._splitMotion ? buildKirchhoffSplitWallFriction(constraint, options.additionalRows,
                this.fixedDt, constraint._jointSplitWallFrictionBatch ??= {}) : null;
            if (wallBatch) appendKirchhoffSplitWallFriction(wallBatch, options.additionalRows, options.groups);
            const witnessFrictionBatch = constraint._usesWallWitnesses ? buildKirchhoffWallWitnessFriction(constraint,
                options.additionalRows, this.fixedDt, constraint._wallWitnessFrictionSolve ??= {}) : null;
            if (witnessFrictionBatch) appendKirchhoffWallWitnessFriction(witnessFrictionBatch, options.additionalRows, options.groups);
            const channelRows = constraint._splitMotion?.twoChannel
                ? prepareKirchhoffTwoChannelRows(constraint, options.additionalRows, options.groups) : null;
            if (channelRows && !channelRows.ready) {
                constraint._jointLinearFailure = { converged: false, status: 'two-channel-contact-history', issues: channelRows.issues };
                break;
            }
            this.lastJointCosts.assemblyMs += now() - assemblyStarted;
            const solveStarted = now();
            const result = channelRows
                ? (this.coupledSystem.solveTwoChannel ?? solveKirchhoffTwoChannelSystem)(constraint, this.fixedDt,
                    { ...options, channels: channelRows.channels })
                : this.coupledSystem.solve(constraint, this.fixedDt, options);
            this.lastJointCosts.solveMs += now() - solveStarted;
            this.lastJointCosts.solveCalls++;
            const coreCosts=result.diagnostics.condensedCosts;
            if(coreCosts) {
                this.lastJointCosts.condensedSetupMs+=coreCosts.setupMs;
                for(const key of ['schurMs','contactSolveMs','reconstructionMs','seedMs'])this.lastJointCosts[key]+=coreCosts[key];
            }
            constraint._jointDiagnostics = result.diagnostics;
            this.lastJointFactorizations += result.diagnostics.factorizations ?? 0;
            this.lastJointLinearIterations += result.diagnostics.iterations ?? 0;
            this.lastJointMaximumBand = Math.max(this.lastJointMaximumBand, result.diagnostics.band ?? 0);
            this.lastJointMaximumRows = Math.max(this.lastJointMaximumRows, result.diagnostics.rowCount ?? 0);
            if (!result.diagnostics.converged) {
                // Keep the last accepted coordinates when controls/contact
                // produce an infeasible local system. This step remains failed.
                constraint._jointLinearFailure = { ...result.diagnostics };
                break;
            }
            constraint._contactBlockSweeps++;
            constraint._contactBlockIterations += result.diagnostics.iterations ?? 0;
            this.lastCoupledContactPasses++;
            const proposedScale = result.scale;
            // Near equilibrium, skip at most the full-scale candidate after
            // a strongly damped success. Larger starts are revisited below.
            // Far from equilibrium, retain full steps so shape recovery does
            // not trade fewer trials for more global material solves.
            const startLevel = this.adaptiveLineSearch && !constraint._splitMotion &&
                pass > 1 && previousMerit <= 4
                ? Math.min(1, Math.max(0, acceptedLevel - 2)) : 0;
            if (startLevel) this.lastJointLineSearch.predictedStarts++;
            const snapshot = pass === 0 ? null : this.#captureJointTrial(constraint,
                { world: this, reusePropertyLayout: true, frozenFrictionBatches: true,
                    compactContactState: this.coupledSystem.compactContactTrial !== false,
                    physicalStateOnly: this.coupledSystem.physicalTrialState === true && !constraint._splitMotion },
                constraint._jointTrialState ??= {});
            const knownWallWitnesses = constraint._usesWallWitnesses ? new Set(constraint._wallWitnessRows.witnesses) : null;
            let wallDiscoveries = [];
            let state, accepted = false, trialCount = 0;
            for (; trialCount < 8; trialCount++) {
                if (trialCount) {
                    this.#restoreJointTrial(snapshot);
                    this.lastJointBacktracks++;
                }
                const level = lineSearchLevel(trialCount, startLevel);
                if (startLevel && level < startLevel) this.lastJointLineSearch.largerFallbacks++;
                result.scale = proposedScale * 2 ** -level;
                const applyStarted = now();
                if (channelRows) applyKirchhoffTwoChannelPhysicalMotion(constraint, result);
                else applyKirchhoffSplitPhysicalIncrement(constraint, result);
                this.coupledSystem.apply(constraint, result);
                if (channelRows) {
                    commitKirchhoffTwoChannelBiasMaterial(constraint, result);
                    commitKirchhoffTwoChannelRows(constraint, result);
                }
                applyKirchhoffCoupledBoundaryMultipliers(constraint, result.additionalIncrement, result.scale);
                if (constraint._usesWallWitnesses) commitKirchhoffWallWitnessMultipliers(constraint, result.additionalIncrement, result.scale);
                applyKirchhoffCoupledFoldMultipliers(constraint, result.additionalIncrement, result.scale);
                commitKirchhoffCoupledOrientationMultipliers(orientationBatch, result.additionalIncrement, result.scale);
                for (let index = 0; index < constraint.kirchhoffContacts.length; index++) {
                    const contact = constraint.kirchhoffContacts[index].manifoldContact;
                    contact.normalLambda = Math.max(0, contact.normalLambda + result.scale * result.contactIncrement[index]);
                }
                if (frictionBatch) commitKirchhoffCoupledFrictionMultipliers(frictionBatch, result.additionalIncrement, result.scale);
                commitKirchhoffExternalFrictionMultipliers(externalBatch, result.additionalIncrement, result.scale);
                if (wallBatch) commitKirchhoffSplitWallFriction(wallBatch, result.additionalIncrement, result.scale);
                if (witnessFrictionBatch) commitKirchhoffWallWitnessFriction(witnessFrictionBatch, result.additionalIncrement, result.scale);
                constraint._kirchhoffMappingLocked = true;
                this.lastJointCosts.applyMs += now() - applyStarted;
                this.lastJointCosts.applyCalls++;
                // Keep the automatic first pass and the final failure report
                // complete. Split-motion cone filters and witness discovery
                // have additional acceptance/lifecycle rules and stay full.
                const rejectionThreshold = this.coupledSystem.earlyTrialRejection === true && pass > 0 &&
                    trialCount < 7 && !constraint._splitMotion && !constraint._usesWallWitnesses
                    ? previousMerit * (1 - 1e-4 * result.scale) : null;
                state = this.#measureJointCoupledConstraints(constraint, true, rejectionThreshold);
                this.debugJointTrial?.(constraint, state, pass, trialCount, result.scale);
                this.lastJointTrialEvaluations++;
                // The load-continuous natural map guides line search only;
                // final physical acceptance uses the independent KKT gates.
                // Once every other equation meets its final gate, allow
                // further cone repair even when an already-small material
                // residual dominates the merit. Final acceptance still
                // requires the original cone tolerance for all contacts.
                const coneFilterAccepted = constraint._splitMotion?.twoChannel && previousNonConeSettled && state.nonConeSettled &&
                    state.maximumConeViolation <= previousMaxCone * (1 - 1e-4 * result.scale);
                const meritAccepted = !state.earlyRejected && state.merit <= previousMerit * (1 - 1e-4 * result.scale);
                accepted = !state.earlyRejected && (pass === 0 || state.settled || meritAccepted || coneFilterAccepted);
                recordLineSearchTrial(this.lastJointLineSearch, level, accepted, state,
                    constraint._jointBaseBoundaryWorst, constraint._jointBoundaryWorst);
                if (accepted) acceptedLevel = level;
                if (coneFilterAccepted && pass > 0 && !state.settled && !meritAccepted) {
                    (constraint._splitMotion.diagnostics.coneFilterAcceptances ??= []).push({
                        pass: pass + 1, scale: result.scale, previousMerit, merit: state.merit,
                        previousCone: previousMaxCone, cone: state.maximumConeViolation
                    });
                }
                if (accepted) break;
                if (knownWallWitnesses) {
                    wallDiscoveries = captureKirchhoffWallDiscoveries(constraint, knownWallWitnesses);
                    if (wallDiscoveries.length) break;
                }
            }
            if (!accepted) {
                const rejectedMerit = state.merit;
                const rejectedTerms = {...state.meritTerms};
                const rejectedBoundary = {...constraint._jointBoundaryWorst};
                this.#restoreJointTrial(snapshot);
                if (wallDiscoveries.length && retainKirchhoffWallDiscoveries(constraint, wallDiscoveries)) {
                    // Re-linearize the unchanged base with newly discovered rows.
                    // This does not accept the rejected trial or weaken its gates.
                    continue;
                }
                const restoredMerit = this.#measureJointCoupledConstraints(constraint, false).merit;
                const tighter = channelRows ? nextKirchhoffTwoChannelTolerance(result.diagnostics, options.tolerance) : null;
                if (tighter !== null && pass + 1 < this.coupledClosureMaxPasses * 2) {
                    // Retry the restored mechanical state through the same
                    // full block. No force prediction, operator input, history
                    // commit or final acceptance threshold is repeated/changed.
                    (constraint._splitMotion.diagnostics.linearRefinements ??= []).push({
                        pass: pass + 1, from: options.tolerance, to: tighter,
                        structuralResidualFloor: result.diagnostics.structuralResidualFloor,
                        linearResidual: result.diagnostics.maximumResidual, previousMerit, rejectedMerit
                    });
                    options.tolerance = tighter;
                    continue;
                }
                constraint._jointTrialFailure = { previousMerit, rejectedMerit, restoredMerit, proposedScale, trials: trialCount,
                    baseTerms: {...constraint._jointBaseMeritTerms}, rejectedTerms,
                    baseBoundary: {...constraint._jointBaseBoundaryWorst}, rejectedBoundary,
                    restoredBoundary: {...constraint._jointBoundaryWorst},
                    restoredTerms: {...constraint._jointStateMeasurement.meritTerms} };
                break;
            }
            previousMerit = state.merit;
            const { materialResidual, foldResidual, orientationResidual, frictionResidual,
                externalFrictionResidual, coneRepair } = state;
            const settled = result.diagnostics.converged && state.settled;
            if (this.captureCoupledClosureTrace) this.coupledClosureTrace.push({
                pass: pass + 1, settled, merit: state.merit, trials: trialCount + 1, scale: result.scale, coneRepair, solver: 'joint', contactPasses: this.lastCoupledContactPasses,
                contactMotion: constraint.kirchhoffContactMotion, solverResidual: constraint.kirchhoffSolverResidual,
                boundaryResidual: constraint._jointBoundaryResidual, containmentViolation: constraint.kirchhoffMaxViolation,
                materialResidual: { ...materialResidual },
                foldResidual: { ...foldResidual },
                orientationResidual: { ...orientationResidual },
                frictionResidual: frictionResidual.maximumResidual,
                frictionDisplacementResidualMm: frictionResidual.maximumDisplacementResidualMm,
                frictionFeasibilityResidual: frictionResidual.maximumFeasibilityResidual,
                frictionConeViolation: frictionResidual.maximumConeViolation,
                externalFrictionDisplacementResidualMm: externalFrictionResidual.maximumDisplacementResidualMm,
                externalFrictionConeViolation: externalFrictionResidual.maximumConeViolation,
                linear: { ...result.diagnostics }
            });
            if (settled) { this.lastCoupledClosureConverged = true; break; }
        }
        if (!this.lastCoupledClosureConverged) {
            const measurement = constraint._jointStateMeasurement;
            this.lastJointNonlinearFailure = {
                bodyIds: bodies.map(body => body.id), passes: this.lastCoupledClosurePasses,
                linearStatus: constraint._jointLinearFailure?.status ?? null,
                trial: constraint._jointTrialFailure ? {...constraint._jointTrialFailure} : null,
                merit: measurement?.merit ?? null, lengthResidual: measurement?.lengthResidual ?? null,
                material: measurement?.materialResidual ? {...measurement.materialResidual} : null,
                boundaryResidual: constraint._jointBoundaryResidual ?? null,
                foldResidual: measurement?.foldResidual?.maximumResidual ?? null,
                positionalFoldViolation: measurement?.foldResidual?.maximumPositionalViolation ?? null,
                orientationResidual: measurement?.orientationResidual?.maximumResidualRad ?? null
            };
        }
        this._inCoupledClosure = false;
        // Keep one position-history convention during feed and hold. Do not
        // rewrite both bodies' histories based on the last body's damping.
        for (const name of ['constraintPrimary', 'constraintBodyClosure', 'constraintBodyLengthPolish',
            'constraintBodyWallRepair', 'constraintBodyPrePost', 'constraintBodyPostStabilization', 'constraintMovingClosure']) {
            recordTiming(this.timings[name], 0);
        }
        recordTiming(this.timings.constraintCoupledClosure, now() - began);
        for (const body of bodies) body.debugConstraintPhase?.('closureEnd', body);
    }

    // Rebuild actual contact geometry and evaluate all APPLIED equations.
    // Scratch is borrowed until the next call. Trial acceptance uses these
    // physical units; the linear QP residual alone is never sufficient.
    #captureJointTrial(constraint, options, out = {}) {
        const started = now();
        const snapshot = captureKirchhoffCoupledTrialState(constraint, options, out);
        const costs = this.lastJointCosts;
        costs.snapshotMs += now() - started; costs.snapshots++;
        costs.snapshotObjects = Math.max(costs.snapshotObjects, snapshot.objectCount);
        costs.snapshotBytes = Math.max(costs.snapshotBytes, snapshot.bytes);
        return snapshot;
    }

    #restoreJointTrial(snapshot) {
        const started = now();
        restoreKirchhoffCoupledTrialState(snapshot);
        this.lastJointCosts.restoreMs += now() - started;
        this.lastJointCosts.restores++;
    }

    #measureJointCoupledConstraints(constraint, repairCone = true, rejectionThreshold = null, reuseAccepted = false) {
        const started = now();
        this.lastJointCosts.measureCalls++;
        try {
            const result = this.#measureJointCoupledConstraintsImpl(constraint, repairCone, rejectionThreshold, reuseAccepted);
            if (!result.earlyRejected) this.lastJointCosts.fullMeasureCalls++;
            return result;
        }
        finally { this.lastJointCosts.measureMs += now() - started; }
    }

    #measureJointCoupledConstraintsImpl(constraint, repairCone = true, rejectionThreshold = null, reuseAccepted = false) {
        const bodies = kirchhoffComponentBodies(constraint);
        const hasLumen = !constraint.bodies || constraint.containment === constraint;
        // This bank lives only across the synchronous collect/measure pair.
        // Never carry a geometry certificate across apply, rollback or a new
        // measurement. A cone repair below recollects into a fresh bank too.
        const sideMeasurements = constraint._reuseCandidateEvaluation && !constraint._splitMotion
            ? (this._jointCandidateSideMeasurements ??= []) : null;
        if (sideMeasurements) sideMeasurements.length = 0;
        // Refresh the actual nonlinear gap and surface kinematics before
        // testing normal complementarity and the final-load friction cone.
        constraint.kirchhoffContacts.length = 0;
        if (hasLumen) this.#collectKirchhoffContainmentGeometry(constraint, true, sideMeasurements);
        for (const record of constraint.kirchhoffContacts) buildKirchhoffContactNormalGradients(constraint, record);
        prepareKirchhoffSplitLumenRows(constraint);
        const cacheFriction = this.coupledSystem.reuseAcceptedEvaluation !== false && hasLumen &&
            constraint.kirchhoffContacts.length > 0 && !constraint._splitMotion && !constraint._usesWallWitnesses;
        const frictionResidual = constraint.surfaceFrictionEnabled === false ? FRICTIONLESS_LUMEN_RESIDUAL
            : measureKirchhoffCoupledFrictionResidual(constraint, this.fixedDt,
                constraint._jointFrictionResidual ??= {}, { cacheInputs: cacheFriction, reuseInputs: reuseAccepted });
        if (frictionResidual.reusedEvaluation) this.lastJointCosts.acceptedFrictionReuseCount++;
        let coneRepair = null;
        if (!constraint._splitMotion && repairCone && frictionResidual.maximumConeViolation > 1e-9) {
            if (constraint._reuseCandidateEvaluation) this.lastJointCosts.frictionBatchReuseCount++;
            const plan = prepareKirchhoffCoupledConeRepair(constraint, this.fixedDt, {
                preparedBatch: constraint._reuseCandidateEvaluation ? frictionResidual._batch : null,
                maximumPositionCorrectionMm: this.coupledContainmentTolerance,
                maximumAngleCorrectionRad: this.coupledAngularToleranceRad
            }, constraint._jointConeRepair ??= {});
            if (plan.accepted && plan.changedContacts) {
                // This also applies W J^T deltaLambda to both tools. A
                // force-only clip would silently discard their reaction.
                applyKirchhoffCoupledConeRepair(plan);
                coneRepair = { contacts: plan.changedContacts, positionMm: plan.maximumPositionCorrectionMm,
                    angleRad: plan.maximumAngleCorrectionRad, multiplier: plan.maximumMultiplierCorrection };
                constraint.kirchhoffContacts.length = 0;
                if (sideMeasurements) sideMeasurements.length = 0;
                if (hasLumen) this.#collectKirchhoffContainmentGeometry(constraint, true, sideMeasurements);
                for (const record of constraint.kirchhoffContacts) buildKirchhoffContactNormalGradients(constraint, record);
                measureKirchhoffCoupledFrictionResidual(constraint, this.fixedDt, frictionResidual, { cacheInputs: cacheFriction });
            }
        }
        const lengthsSettled = bodies.every(body => !this.#hasLengthErrorOver(body, this.coupledLengthTolerance));
        constraint.kirchhoffContactMotion = measureKirchhoffContactMotion(constraint, false);
        constraint.kirchhoffMaxViolation = hasLumen ? this.#measureKirchhoffCoupledContainmentViolation(constraint, sideMeasurements) : 0;
        constraint.kirchhoffSolverResidual = hasLumen ? this.#kirchhoffContactSolverResidual(constraint) : 0;
        this.lastCoupledContainmentResidual = constraint.kirchhoffMaxViolation;
        for (const body of bodies) this.#prepareWallContacts(body, true);
        const boundaryRows = collectKirchhoffCoupledBoundaryRows(constraint, this.sheaths,
            this.fixedDt, this.contactActivation, !constraint._usesWallWitnesses);
        if (constraint._usesWallWitnesses) collectKirchhoffWallWitnessRows(constraint, this.contactField, boundaryRows, this.fixedDt);
        for (const contact of this.toolContacts) if (bodies.includes(contact.bodyA) && bodies.includes(contact.bodyB))
            this.#solveToolContact(contact, boundaryRows, constraint);
        appendKirchhoffSplitSweeps(constraint, boundaryRows);
        appendKirchhoffSplitPointWalls(constraint, this, boundaryRows);
        prepareKirchhoffSplitBoundaryRows(constraint, boundaryRows);
        constraint._jointBoundaryResidual = measureKirchhoffCoupledBoundaryResidual(boundaryRows, constraint._jointBoundaryWorst ??= {});
        const wallWitnessResidual = constraint._usesWallWitnesses ? measureKirchhoffWallWitnessResidual(constraint) : null;
        if (wallWitnessResidual) constraint._jointBoundaryResidual = Math.max(constraint._jointBoundaryResidual, wallWitnessResidual.maximumResidual);
        const witnessesSettled = !wallWitnessResidual || wallWitnessResidual.finite && wallWitnessResidual.pending === 0;
        const earlyRejected = boundaryRejectsKirchhoffTrial(constraint._jointBoundaryResidual,
            this.coupledContainmentTolerance, rejectionThreshold);
        if (earlyRejected && !this.debugJointEarlyRejection) {
            // This is a rejection certificate, not a complete residual. The
            // caller restores the trial snapshot before applying another scale.
            return Object.assign(this._jointEarlyRejection ??= {}, {
                earlyRejected: true, settled: false, nonConeSettled: false,
                boundaryMeritLowerBound: constraint._jointBoundaryResidual / this.coupledContainmentTolerance,
                rejectionThreshold
            });
        }
        const materialResidual = constraint._splitMotion?.twoChannel
            ? measureKirchhoffTwoChannelMaterial(constraint, constraint._jointMaterialResidual ??= {})
            : (constraint._splitMotion ? measureKirchhoffSplitMaterial : measureKirchhoffCoupledMaterialResidual)(constraint, this.fixedDt,
                constraint._jointMaterialResidual ??= {});
        const channelResidual = constraint._splitMotion?.twoChannel
            ? measureKirchhoffTwoChannelRows(constraint, boundaryRows) : null;
        const foldResidual = measureKirchhoffCoupledFoldResidual(constraint);
        const orientationResidual = measureKirchhoffCoupledOrientationResidual(constraint, this.fixedDt);
        const externalFrictionResidual = measureKirchhoffExternalFrictionResidual(constraint, boundaryRows,
            this.fixedDt, constraint._jointExternalFrictionResidual ??= {});
        const wallPhysicalFriction = constraint._splitMotion ? measureKirchhoffSplitWallFriction(constraint, boundaryRows,
            this.fixedDt, constraint._jointSplitWallFrictionResidual ??= {}) : null;
        // measure creates its own row bank; do not assemble the same geometry
        // twice just to pass the component, normal rows and timestep through.
        const witnessFrictionBatch = constraint._usesWallWitnesses ? Object.assign(
            constraint._wallWitnessFrictionMeasure ??= {},
            {component:constraint, normalRows:boundaryRows, dt:this.fixedDt}) : null;
        const wallWitnessFriction = witnessFrictionBatch ? measureKirchhoffWallWitnessFriction(constraint, witnessFrictionBatch) : null;
        const witnessFrictionSettled = !wallWitnessFriction || wallWitnessFriction.finite &&
            wallWitnessFriction.maximumTransportPositionMm <= this.coupledContainmentTolerance &&
            wallWitnessFriction.maximumTransportAngleRad <= this.coupledAngularToleranceRad &&
            wallWitnessFriction.maximumDisplacementResidualMm <= this.coupledContainmentTolerance;
        const toolReleaseResidual = measureKirchhoffToolReleaseRows(constraint, boundaryRows,
            constraint._jointToolReleaseResidual ??= {});
        const channelsSettled = !channelResidual || channelResidual.finite && channelResidual.supported &&
            channelResidual.missingLoadedRows.length === 0 &&
            Math.max(channelResidual.normalResidualMm, channelResidual.controlResidualMm,
                channelResidual.releasePositionMm ?? 0) <= this.coupledContainmentTolerance &&
            Math.max(channelResidual.orientationResidualRad, channelResidual.releaseAngleRad ?? 0) <= this.coupledAngularToleranceRad;
        const settled = witnessFrictionSettled && (!wallWitnessFriction || wallWitnessFriction.maximumConeViolation <= 1e-9) && witnessesSettled && channelsSettled && lengthsSettled && toolReleaseResidual.pending === 0 &&
            materialResidual.adaptationMm <= this.coupledContainmentTolerance &&
            materialResidual.bendTwistRad <= this.coupledAngularToleranceRad &&
            foldResidual.maximumResidual <= this.coupledAngularToleranceRad &&
            foldResidual.maximumPositionalViolation <= this.coupledAngularToleranceRad &&
            orientationResidual.maximumResidualRad <= this.coupledAngularToleranceRad &&
            frictionResidual.maximumDisplacementResidualMm <= this.coupledContainmentTolerance &&
            frictionResidual.maximumConeViolation <= 1e-9 &&
            externalFrictionResidual.maximumDisplacementResidualMm <= this.coupledContainmentTolerance &&
            externalFrictionResidual.maximumConeViolation <= 1e-9 &&
            (!wallPhysicalFriction || wallPhysicalFriction.maximumDisplacementResidualMm <= this.coupledContainmentTolerance &&
                wallPhysicalFriction.maximumConeViolation <= 1e-9) &&
            Math.max(constraint.kirchhoffContactMotion, constraint.kirchhoffSolverResidual,
                constraint._jointBoundaryResidual) <= this.coupledContainmentTolerance;
        const nonConeSettled = witnessFrictionSettled && witnessesSettled && channelsSettled && lengthsSettled && toolReleaseResidual.pending === 0 &&
            materialResidual.adaptationMm <= this.coupledContainmentTolerance &&
            materialResidual.bendTwistRad <= this.coupledAngularToleranceRad &&
            foldResidual.maximumResidual <= this.coupledAngularToleranceRad &&
            foldResidual.maximumPositionalViolation <= this.coupledAngularToleranceRad &&
            orientationResidual.maximumResidualRad <= this.coupledAngularToleranceRad &&
            frictionResidual.maximumDisplacementResidualMm <= this.coupledContainmentTolerance &&
            externalFrictionResidual.maximumDisplacementResidualMm <= this.coupledContainmentTolerance &&
            (!wallPhysicalFriction || wallPhysicalFriction.maximumDisplacementResidualMm <= this.coupledContainmentTolerance) &&
            Math.max(constraint.kirchhoffContactMotion, constraint.kirchhoffSolverResidual,
                constraint._jointBoundaryResidual) <= this.coupledContainmentTolerance;
        const maximumConeViolation = Math.max(frictionResidual.maximumConeViolation,
            externalFrictionResidual.maximumConeViolation, wallPhysicalFriction?.maximumConeViolation ?? 0, wallWitnessFriction?.maximumConeViolation ?? 0);
        let lengthResidual = 0;
        for (const body of bodies) for (let segment = body.activeStart; segment < body.activeEnd; segment++) {
            const length = magnitude3(body.x[segment + 1] - body.x[segment],
                body.y[segment + 1] - body.y[segment], body.z[segment + 1] - body.z[segment]);
            lengthResidual = Math.max(lengthResidual,
                Math.abs(length - body.restLength[segment]) / body.restLength[segment]);
        }
        const mm = this.coupledContainmentTolerance, rad = this.coupledAngularToleranceRad;
        const channelMerit = !channelResidual ? 0 : !channelResidual.finite || !channelResidual.supported || channelResidual.missingLoadedRows.length
            ? Infinity : Math.max(channelResidual.normalResidualMm / mm, channelResidual.controlResidualMm / mm,
                channelResidual.orientationResidualRad / rad, (channelResidual.releasePositionMm ?? 0) / mm,
                (channelResidual.releaseAngleRad ?? 0) / rad);
        const meritTerms = Object.assign(constraint._jointMeritTerms ??= {}, {
            witnessFriction: wallWitnessFriction ? Math.max(wallWitnessFriction.maximumMeritMm / mm,
                wallWitnessFriction.maximumTransportPositionMm / mm, wallWitnessFriction.maximumTransportAngleRad / rad) : 0,
            channels: channelMerit, length: lengthResidual / this.coupledLengthTolerance,
            releasePosition: toolReleaseResidual.positionMm / mm, releaseAngle: toolReleaseResidual.angleRad / rad,
            adaptation: materialResidual.adaptationMm / mm, bendTwist: materialResidual.bendTwistRad / rad,
            fold: foldResidual.maximumResidual / rad, positionalFold: foldResidual.maximumPositionalViolation / rad,
            orientation: orientationResidual.maximumResidualRad / rad,
            lumenFriction: constraint.surfaceFrictionEnabled === false ? 0 : measureKirchhoffFrictionMerit(frictionResidual._batch, constraint._jointFrictionMerit ??= {}).maximumMm / mm,
            externalFriction: measureKirchhoffFrictionMerit(externalFrictionResidual._batch, constraint._jointExternalFrictionMerit ??= {}).maximumMm / mm,
            wallFriction: wallPhysicalFriction ? measureKirchhoffFrictionMerit(wallPhysicalFriction._batch, constraint._jointSplitWallFrictionMerit ??= {}).maximumMm / mm : 0,
            lumenNormal: constraint.kirchhoffSolverResidual / mm,
            boundary: constraint._jointBoundaryResidual / mm
        });
        let merit = 0;
        for (const key in meritTerms) merit = Math.max(merit, meritTerms[key]);
        const measurement = Object.assign(constraint._jointStateMeasurement ??= {}, {
            motionPhase: constraint._splitMotion?.phase ?? 'position-history',
            settled, nonConeSettled, maximumConeViolation, merit, meritTerms, lengthResidual, materialResidual, channelResidual, foldResidual, orientationResidual,
            frictionResidual, externalFrictionResidual, wallPhysicalFriction, wallWitnessResidual, wallWitnessFriction, toolReleaseResidual, coneRepair
        });
        // Diagnostic shadow mode finishes the same measurement (no repeated
        // geometry query) so tests can check the certificate against all gates.
        if (earlyRejected) this.debugJointEarlyRejection(constraint, {
            boundaryMeritLowerBound: constraint._jointBoundaryResidual / this.coupledContainmentTolerance,
            rejectionThreshold
        }, measurement);
        return measurement;
    }

    resetPerformanceStats() {
        this.contactCount = 0;
        this.maxPenetration = 0;
        this.settledMaxPenetration = 0;
        for (const constraint of this.containments) {
            constraint.kirchhoffOpenCacheHits = 0;
        }
        for (const timing of Object.values(this.timings)) {
            timing.samples.fill(0);
            timing.cursor = 0;
            timing.count = 0;
            timing.recordedCount = 0;
            timing.total = 0;
            timing.last = 0;
            timing.maximum = 0;
            timing.maximumRecord = -1;
        }
    }

    resetSimulationState() {
        if (this._pendingWholeSubstep?.running) throw new Error('Cannot reset a running whole timestep');
        // Reset the saved pending owner too if configuration was changed
        // before explicitly abandoning that transaction via reset.
        const wholeSystems = new Set([this._pendingWholeSubstep?.system, this.wholeStepSystem].filter(system => system != null));
        for (const system of wholeSystems) validateWholeStepSystem(system);
        this._pendingWholeSubstep = null;
        this._pendingSplitSubstep = null;
        this.lastStepResult = null;
        this.accumulator = 0;
        this.stepCount = 0;
        this.lastSubsteps = 0;
        this.droppedTime = 0;
        for (const body of this.bodies) {
            body.lengthLambda.fill(0);
            body.controlLambda.fill(0);
            body.adaptationLambdaX.fill(0);
            body.adaptationLambdaY.fill(0);
            body.adaptationLambdaZ.fill(0);
            body.bendTwistLambda1.fill(0);
            body.bendTwistLambda2.fill(0);
            body.bendTwistLambda3.fill(0);
            body.orientationControlLambda.fill(0);
            body.wallLambda.fill(0);
            body.wallFrictionLambda.fill(0);
            body.wallFrictionLoad.fill(0);
            body.wallActive.fill(0);
            body.wallBranchId.fill(-1);
            body.wallFaceIndex.fill(-1);
            body.wallGap.fill(Infinity);
            body.wallInsideClearance.fill(0);
            body.wallCapsuleSampleCount.fill(0);
            body.wallProjectionX.fill(0);
            body.wallProjectionY.fill(0);
            body.wallProjectionZ.fill(0);
            body.toolProjectionX.fill(0);
            body.toolProjectionY.fill(0);
            body.toolProjectionZ.fill(0);
            body.lastMaximumRawSpeed = 0;
            body.lastMaximumWallProjectionSpeed = 0;
            body.lastMaximumWallProjectionNode = -1;
            body.lastMaximumRejectedWallProjectionSpeed = 0;
            body.lastMaximumToolProjectionSpeed = 0;
            body.lastMaximumRejectedToolProjectionSpeed = 0;
            body.lastMaximumReconstructedSpeed = 0;
            body.copyCurrentToPrevious();
            {
                body.previousOrientationX.set(body.orientationX);
                body.previousOrientationY.set(body.orientationY);
                body.previousOrientationZ.set(body.orientationZ);
                body.previousOrientationW.set(body.orientationW);
                body.angularVelocityX.fill(0);
                body.angularVelocityY.fill(0);
                body.angularVelocityZ.fill(0);
            }
            body.wake();
        }
        for (const sheath of this.sheaths) sheath.lambdas.clear();
        for (const containment of this.containments) {
            {
                if (containment._kirchhoffStepOpen) {
                    containment.manifold.endStep({ prune: false });
                    containment._kirchhoffStepOpen = false;
                }
                containment.manifold.clear();
                containment.kirchhoffOuterSegmentByInner.fill(-1);
                containment.kirchhoffContacts.length = 0;
                containment.kirchhoffMaxViolation = 0;
            }
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
        for (const system of wholeSystems) system.reset(this);
        this.resetPerformanceStats();
        return this;
    }

    getStats() {
        const bodies = this.bodies.map(body => {
            const native=this.#bodyStats(body);
            return this.wholeStepSystem?.id === 'shared-axis' && body.sharedAxisDiagnostics
                ? {...native,...body.sharedAxisDiagnostics,id:body.id,constitutiveSolver:'shared-axis'} : native;
        });
        return {
            mode: this.wholeStepSystem ? 'whole-step' : 'kirchhoff-direct',
            coupledSolver: this.lastCoupledSolver ?? 'independent',
            ...(this.wholeStepSystem ? { wholeStepSystem: this.wholeStepSystem.id, wholeStep: this.lastStepResult } : {}),
            jointMotion: this.wholeStepSystem ? null : this.jointMotionMode === 'split-physical-bias' && this.lastStepResult?.accepted === false
                ? this.lastStepResult.diagnostics : getKirchhoffSplitMotionStats(this),
            jointFactorizations: this.lastJointFactorizations ?? 0,
            jointTrialEvaluations: this.lastJointTrialEvaluations ?? 0,
            jointBacktracks: this.lastJointBacktracks ?? 0,
            jointLinearIterations: this.lastJointLinearIterations ?? 0,
            jointMaximumBand: this.lastJointMaximumBand ?? 0,
            jointMaximumRows: this.lastJointMaximumRows ?? 0,
            jointCosts: this.lastJointCosts ? { ...this.lastJointCosts } : null,
            fixedDt: this.fixedDt,
            steps: this.stepCount,
            lastSubsteps: this.lastSubsteps,
            lastLengthPolishPasses: this.lastLengthPolishPasses ?? 0,
            lastWallRepairPasses: this.lastWallRepairPasses ?? 0,
            wallRepairResiduals: Array.from(
                this.wallRepairResiduals.subarray(
                    0,
                    this.lastWallRepairPasses ?? 0
                )
            ),
            wallRepairWorstSegments: Array.from(
                this.wallRepairWorstSegments.subarray(
                    0,
                    this.lastWallRepairPasses ?? 0
                )
            ),
            wallRepairWorstBodies: Array.from(
                this.wallRepairWorstBodies.subarray(
                    0,
                    this.lastWallRepairPasses ?? 0
                )
            ),
            coupledClosurePasses: this.lastCoupledClosurePasses ?? 0,
            coupledContactPasses: this.lastCoupledContactPasses ?? 0,
            coupledClosureConverged:
                this.lastCoupledClosureConverged ?? true,
            coupledContainmentResidual:
                this.lastCoupledContainmentResidual ?? 0,
            coupledRelaxationPasses:
                this.lastCoupledRelaxationPasses ?? 0,
            backlogTime: this.accumulator,
            backlogSteps: Math.floor(
                (this.accumulator + EPSILON) / this.fixedDt
            ),
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
                constraintPrimary: timingStats(
                    this.timings.constraintPrimary
                ),
                constraintBodyClosure: timingStats(
                    this.timings.constraintBodyClosure
                ),
                constraintBodyLengthPolish: timingStats(
                    this.timings.constraintBodyLengthPolish
                ),
                constraintBodyWallRepair: timingStats(
                    this.timings.constraintBodyWallRepair
                ),
                constraintBodyPrePost: timingStats(
                    this.timings.constraintBodyPrePost
                ),
                constraintBodyPostStabilization: timingStats(
                    this.timings.constraintBodyPostStabilization
                ),
                constraintCoupledClosure: timingStats(
                    this.timings.constraintCoupledClosure
                ),
                constraintMovingClosure: timingStats(
                    this.timings.constraintMovingClosure
                ),
                velocity: timingStats(this.timings.velocity)
            },
            containments: this.containments.map(constraint => ({
                model: constraint.model,
                distalPortalModel: constraint.distalPortalModel,
                contactBlockSweeps: constraint._contactBlockSweeps ?? 0,
                contactBlockIterations: constraint._contactBlockIterations ?? 0,
                solverResidual: constraint.kirchhoffSolverResidual ?? null,
                jointTrialFailure: constraint._jointTrialFailure ?? null,
                jointLinearFailure: constraint._jointLinearFailure ?? null,
                closureContactMotion: constraint.kirchhoffContactMotion ?? 0,
                enabled: constraint.enabled,
                axialFriction: constraint.axialFriction,
                torsionalFriction: constraint.torsionalFriction,
                radialVelocityDamping:
                    constraint.radialVelocityDamping,
                coupledBendingRateDamping:
                    constraint.coupledBendingRateDamping,
                contacts: constraint.kirchhoffContacts?.length ?? 0,
                maximumViolation:
                    constraint.kirchhoffMaxViolation ?? 0,
                openCacheHits:
                    constraint.kirchhoffOpenCacheHits ?? 0
            })),
            bodies
        };
    }

    #solveRelaxationPass(body, pass) {
        this.#solveControls(body);
        this.#solveBending(body);
        this.#solveControls(body);
        this.#prepareWallContacts(body);
        this.#solveWallContacts(body);
        this.#solveFoldLimits(body);
    }

    #solveKirchhoffContainmentsForBody(body, applyFriction = false) {
        for (let index = 0; index < this.containments.length; index++) {
            const constraint = this.containments[index];
            if (
                !constraint.enabled ||
                (
                    constraint.innerBody !== body &&
                    constraint.outerBody !== body
                )
            ) continue;
            this.#solveKirchhoffContainment(
                constraint,
                applyFriction,
                false
            );
        }
    }

    #integrate(body) {
        if (body.sleeping) return;
        const dt = this.fixedDt;
        const dtSquared = dt * dt;
        const start = body.activeStart;
        const end = body.activeEnd;
        {
            const scratch = body.kirchhoffScratch.integrate;
            const segmentStart = Math.max(0, start);
            const segmentEnd = Math.min(body.segmentCount, end);
            for (let segment = segmentStart; segment < segmentEnd; segment++) {
                body.previousOrientationX[segment] = body.orientationX[segment];
                body.previousOrientationY[segment] = body.orientationY[segment];
                body.previousOrientationZ[segment] = body.orientationZ[segment];
                body.previousOrientationW[segment] = body.orientationW[segment];
                body.angularVelocityX[segment] *= body.angularDamping;
                body.angularVelocityY[segment] *= body.angularDamping;
                body.angularVelocityZ[segment] *= body.angularDamping;
                scratch.angularIncrement.x = body.angularVelocityX[segment] * dt;
                scratch.angularIncrement.y = body.angularVelocityY[segment] * dt;
                scratch.angularIncrement.z = body.angularVelocityZ[segment] * dt;
                quaternionExp(scratch.angularIncrement, scratch.increment);
                scratch.current.x = body.orientationX[segment];
                scratch.current.y = body.orientationY[segment];
                scratch.current.z = body.orientationZ[segment];
                scratch.current.w = body.orientationW[segment];
                multiplyQuaternions(
                    scratch.increment,
                    scratch.current,
                    scratch.multiplied
                );
                const orientation = normalizeQuaternion(
                    scratch.multiplied,
                    scratch.normalized
                );
                body.orientationX[segment] = orientation.x;
                body.orientationY[segment] = orientation.y;
                body.orientationZ[segment] = orientation.z;
                body.orientationW[segment] = orientation.w;
            }
        }
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
            const splitJoint = body._splitPhysicalMotion ? this.#jointCoupledConstraint() : null;
            if (splitJoint) {
                // Keep the existing continuous witness in the joint solve.
                // The physical prediction has not already received a separate
                // TOI position projection, so its normal impulse is applied once.
                captureKirchhoffSplitSweep(splitJoint, body, index, contact);
                continue;
            }
            const safeT = Math.max(0, contact.timeOfImpact - 1e-3);
            const inwardX = contact.inward.x;
            const inwardY = contact.inward.y;
            const inwardZ = contact.inward.z;
            if (!body.sweptContactPreserveTangentialMotion) {
                // Preserve the established bit-for-bit path for catheters and
                // other tools outside this guidewire-only change.
                body.x[index] = body.previousX[index] + dx * safeT +
                    inwardX * 1e-3;
                body.y[index] = body.previousY[index] + dy * safeT +
                    inwardY * 1e-3;
                body.z[index] = body.previousZ[index] + dz * safeT +
                    inwardZ * 1e-3;
                continue;
            }
            let remainingX = 0;
            let remainingY = 0;
            let remainingZ = 0;
            if (body.sweptContactPreserveTangentialMotion) {
                const remainingScale = 1 - safeT;
                remainingX = dx * remainingScale;
                remainingY = dy * remainingScale;
                remainingZ = dz * remainingScale;
                const remainingNormal =
                    remainingX * inwardX +
                    remainingY * inwardY +
                    remainingZ * inwardZ;
                // Continuous collision detection owns only the forbidden
                // outward component. Preserve the remaining tangent
                // displacement so a hydrophilic wire hitting the wall
                // obliquely slides instead of losing its entire step at time
                // of impact. Other tools retain their established swept
                // response; this goal deliberately does not alter catheter
                // physics.
                if (remainingNormal < 0) {
                    remainingX -= inwardX * remainingNormal;
                    remainingY -= inwardY * remainingNormal;
                    remainingZ -= inwardZ * remainingNormal;
                }
            }
            const correctedX = body.previousX[index] + dx * safeT +
                remainingX + inwardX * 1e-3;
            const correctedY = body.previousY[index] + dy * safeT +
                remainingY + inwardY * 1e-3;
            const correctedZ = body.previousZ[index] + dz * safeT +
                remainingZ + inwardZ * 1e-3;
            body.wallProjectionX[index] += correctedX - body.x[index];
            body.wallProjectionY[index] += correctedY - body.y[index];
            body.wallProjectionZ[index] += correctedZ - body.z[index];
            body.x[index] = correctedX;
            body.y[index] = correctedY;
            body.z[index] = correctedZ;
        }
    }

    #prepareWallContacts(body, exact = false) {
        if (!this.contactField || body.sleeping || body.collisionEndSegment < body.collisionStartSegment) return;
        const start = Math.max(body.activeStart, body.collisionStartSegment, 0);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        for (let index = start; index < end; index++) {
            const wasActive = body.wallActive[index] !== 0;
            body.wallActive[index] = 0;
            if (wasActive && !exact) {
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
                const maximumMovementSquared = Math.max(
                    startDx * startDx + startDy * startDy + startDz * startDz,
                    endDx * endDx + endDy * endDy + endDz * endDz
                );
                const safeMovement = cachedGap - this.contactActivation;
                if (safeMovement > 0) {
                    const safeMovementSquared = safeMovement * safeMovement;
                    // sqrt(max(a,b)) < clearance is algebraically equivalent.
                    // Keep a wide roundoff guard so a query near the exact
                    // activation boundary follows the original calculation.
                    if (
                        safeMovementSquared - maximumMovementSquared >
                        safeMovementSquared * 1e-12
                    ) {
                        body.wallLambda[index] = 0;
                        continue;
                    }
                }
                const maximumMovement = Math.sqrt(maximumMovementSquared);
                // Distance to a closed surface is 1-Lipschitz. A capsule whose
                // cached gap exceeds the largest endpoint displacement cannot
                // have reached the activation band since its last exact query.
                if (cachedGap - maximumMovement > this.contactActivation) {
                    body.wallLambda[index] = 0;
                    continue;
                }
            }
            let contact;
            if (this.contactField.queryCapsuleSoA) {
                contact = this.contactField.queryCapsuleSoA(
                    body.x,
                    body.y,
                    body.z,
                    body.nodeRadius,
                    index,
                    this._contact,
                    body.wallFaceIndex[index],
                    false,
                    false,
                    wasActive ? body.wallBranchId[index] : -1,
                    false, -1, 0,
                    this.coupledSystem?.wallWitnesses === true || ['joint', 'joint-components'].includes(this.lastCoupledSolver),
                    this.coupledSystem?.wallWitnesses === true
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
            body.wallInsideClearance[index] = Math.max(
                0,
                contact.insideClearance || 0
            );
            body.wallCapsuleSampleCount[index] = Math.max(
                0,
                contact.capsuleSampleCount || 0
            );
            if (signedGap > this.contactActivation && !(['joint', 'joint-components'].includes(this.lastCoupledSolver) && body.wallLambda[index] > 0)) {
                body.wallLambda[index] = 0;
                continue;
            }
            if (body.wallBranchId[index] !== branchId && !['joint', 'joint-components'].includes(this.lastCoupledSolver)) body.wallLambda[index] = 0;
            body.wallActive[index] = 1;
            body.wallT[index] = segmentT;
            body.wallX[index] = closest[0];
            body.wallY[index] = closest[1];
            body.wallZ[index] = closest[2];
            body.wallNormalX[index] = inward[0];
            body.wallNormalY[index] = inward[1];
            body.wallNormalZ[index] = inward[2];
            body.wallBranchId[index] = branchId;
            body.wallFaceIndex[index] = contact.faceIndex;
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
        if (
            !this.contactField ||
            body.sleeping ||
            body.collisionEndSegment < body.collisionStartSegment
        ) {
            return 0;
        }
        const start = Math.max(body.activeStart, body.collisionStartSegment, 0);
        const end = Math.min(body.activeEnd, body.collisionEndSegment + 1, body.segmentCount);
        let maximumPenetration = 0;
        body._wallRefreshWorstSegment = -1;
        for (let index = start; index < end; index++) {
            if (!body.wallActive[index]) continue;
            const cachedGap = body.wallGap[index];
            let maximumMovement = Infinity;
            if (Number.isFinite(cachedGap)) {
                const startDx = body.x[index] - body.wallQueryStartX[index];
                const startDy = body.y[index] - body.wallQueryStartY[index];
                const startDz = body.z[index] - body.wallQueryStartZ[index];
                const endDx = body.x[index + 1] - body.wallQueryEndX[index];
                const endDy = body.y[index + 1] - body.wallQueryEndY[index];
                const endDz = body.z[index + 1] - body.wallQueryEndZ[index];
                const maximumMovementSquared = Math.max(
                    startDx * startDx + startDy * startDy + startDz * startDz,
                    endDx * endDx + endDy * endDy + endDz * endDz
                );
                const activationMovement = cachedGap - this.contactActivation;
                if (activationMovement > 0) {
                    const activationMovementSquared =
                        activationMovement * activationMovement;
                    if (
                        activationMovementSquared - maximumMovementSquared >
                        activationMovementSquared * 1e-12
                    ) {
                        body.wallActive[index] = 0;
                        body.wallLambda[index] = 0;
                        continue;
                    }
                }
                const repairMovement = cachedGap - 0.02;
                if (repairMovement > 0) {
                    const repairMovementSquared =
                        repairMovement * repairMovement;
                    if (
                        repairMovementSquared - maximumMovementSquared >
                        repairMovementSquared * 1e-12
                    ) continue;
                }
                maximumMovement = Math.sqrt(maximumMovementSquared);
                const conservativeGap = cachedGap - maximumMovement;
                if (conservativeGap > 0.02) {
                    if (conservativeGap > this.contactActivation) {
                        body.wallActive[index] = 0;
                        body.wallLambda[index] = 0;
                    }
                    continue;
                }
            }
            let contact;
            let knownInside = false;
            if (this.contactField.queryCapsuleSoA) {
                const segmentDx = body.x[index + 1] - body.x[index];
                const segmentDy = body.y[index + 1] - body.y[index];
                const segmentDz = body.z[index + 1] - body.z[index];
                const segmentLength = Math.sqrt(
                    segmentDx * segmentDx +
                    segmentDy * segmentDy +
                    segmentDz * segmentDz
                );
                const radius = Math.max(
                    body.nodeRadius[index],
                    body.nodeRadius[index + 1]
                );
                const spacing = Math.max(
                    this.contactField.voxelSize * 4,
                    Math.max(0.5, radius)
                );
                const sampleCount = Math.max(
                    1,
                    Math.ceil(segmentLength / spacing)
                );
                knownInside =
                    body.wallCapsuleSampleCount[index] === sampleCount &&
                    body.wallInsideClearance[index] - maximumMovement > 0;
                const measureInsideClearance = !knownInside;
                const knownNearWall =
                    cachedGap - maximumMovement <=
                        this.contactField.capsuleBvhValidationGap;
                contact = this.contactField.queryCapsuleSoA(
                    body.x,
                    body.y,
                    body.z,
                    body.nodeRadius,
                    index,
                    this._contact,
                    body.wallFaceIndex[index],
                    knownInside,
                    measureInsideClearance,
                    body.wallBranchId[index],
                    knownNearWall,
                    segmentLength,
                    sampleCount,
                    this.coupledSystem?.wallWitnesses === true,
                    this.coupledSystem?.wallWitnesses === true
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
            body.wallInsideClearance[index] = knownInside
                ? Math.max(
                    0,
                    body.wallInsideClearance[index] - maximumMovement
                )
                : Math.max(0, contact.insideClearance || 0);
            body.wallCapsuleSampleCount[index] = Math.max(
                0,
                contact.capsuleSampleCount || 0
            );
            if (signedGap > this.contactActivation) {
                body.wallActive[index] = 0;
                body.wallLambda[index] = 0;
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
            body.wallFaceIndex[index] = contact.faceIndex;
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
                if (contactValues[CONTACT_PENETRATION] > maximumPenetration) {
                    maximumPenetration = contactValues[CONTACT_PENETRATION];
                    body._wallRefreshWorstSegment = index;
                }
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
        {
            this.#solveKirchhoffOrientationControl(body);
        }
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

    #solveKirchhoffOrientationControl(body) {
        const segment = body.orientationControlSegment;
        if (
            segment < Math.max(0, body.activeStart) ||
            segment >= Math.min(body.segmentCount, body.activeEnd)
        ) return;
        const scratch = body.kirchhoffScratch.orientationControl;
        const target = scratch.target;
        const orientation = scratch.orientation;
        const lambda = scratch.lambda;
        const inverseInertia = scratch.inverseInertia;
        target.x = body.orientationControlX;
        target.y = body.orientationControlY;
        target.z = body.orientationControlZ;
        target.w = body.orientationControlW;
        orientation.x = body.orientationX[segment];
        orientation.y = body.orientationY[segment];
        orientation.z = body.orientationZ[segment];
        orientation.w = body.orientationW[segment];
        lambda.x = body.orientationControlLambda[0];
        lambda.y = body.orientationControlLambda[1];
        lambda.z = body.orientationControlLambda[2];
        inverseInertia.x = body.inverseInertia1[segment];
        inverseInertia.y = body.inverseInertia2[segment];
        inverseInertia.z = body.inverseInertia3[segment];
        const solveOptions = scratch.options ??= {
            orientation0: target,
            orientation1: orientation,
            restRotation: scratch.restRotation,
            inverseInertia0: 0,
            inverseInertia1: inverseInertia,
            compliance: 0,
            dt: this.fixedDt,
            lambda,
            scratch: scratch.solver,
            returnState: false,
            normalizedOrientations: true,
            objectVectors: false
        };
        solveOptions.compliance = body.orientationControlCompliance;
        solveOptions.dt = this.fixedDt;
        solveBendTwistXPBD(solveOptions);
        body.orientationX[segment] = orientation.x;
        body.orientationY[segment] = orientation.y;
        body.orientationZ[segment] = orientation.z;
        body.orientationW[segment] = orientation.w;
        body.orientationControlLambda[0] = lambda.x;
        body.orientationControlLambda[1] = lambda.y;
        body.orientationControlLambda[2] = lambda.z;
    }

    #solveKirchhoffAdaptation(body, reverse = false) {
        if (body.sleeping || body.segmentCount <= 0) return;
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        solveAdaptationXPBDArraySweep(
            body,
            start,
            end,
            reverse,
            this.fixedDt
        );
    }

    #solveKirchhoffBendTwist(body) {
        if (body.sleeping) return;
        {
            solveKirchhoffDirect(body, this.fixedDt, false, this.reuseDirectLinearization);
            return;
        }

    }

    #solveLengthsGlobal(body) {
        {
            const reverse = body.kirchhoffLengthSweepReverse === true;
            body.kirchhoffLengthSweepReverse = !reverse;
            this.#solveKirchhoffAdaptation(body, reverse);
            if (body.adaptationCompliance > EPSILON) return;
            // The tridiagonal length block below is a numerical
            // preconditioner for the same inextensible adaptation constraint,
            // not a second constitutive energy. It propagates a local wall or
            // outlet correction through a long rod in one solve; subsequent
            // adaptation sweeps keep each material d3 aligned with its edge.
        }
        if (body.sleeping) return;
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        const count = end - start;
        if (count <= 0) return;

        for (let newtonPass = 0; newtonPass < 1; newtonPass++) {
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
                    body.lengthRhs[local] = -(
                        distance - body.restLength[segment]
                    );
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
                body.lengthSolution[local] = body.inverseMass[segment] +
                    body.inverseMass[segment + 1];
            }

            let denominator = Math.max(EPSILON, body.lengthSolution[0]);
            body.lengthUpper[0] /= denominator;
            body.lengthRhs[0] /= denominator;
            for (let local = 1; local < count; local++) {
                denominator = Math.max(
                    EPSILON,
                    body.lengthSolution[local] -
                        body.lengthLower[local] * body.lengthUpper[local - 1]
                );
                body.lengthUpper[local] = local + 1 < count
                    ? body.lengthUpper[local] / denominator
                    : 0;
                body.lengthRhs[local] = (
                    body.lengthRhs[local] -
                    body.lengthLower[local] * body.lengthRhs[local - 1]
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
                body.x[segment + 1] +=
                    nx * lambda * body.inverseMass[segment + 1];
                body.y[segment + 1] +=
                    ny * lambda * body.inverseMass[segment + 1];
                body.z[segment + 1] +=
                    nz * lambda * body.inverseMass[segment + 1];
            }
        }
    }

    #solveBending(body) {
        this.#solveKirchhoffBendTwist(body);
    }

    #solveFoldLimits(body) {
        if (body.sleeping || body.count < 3 || body.foldLimitStrength <= 0) return;
        {
            this.#solveKirchhoffFoldLimits(body);
            return;
        }
        // A supporting sheath is allowed to impose its own curvature. The
        // anti-fold inequality belongs to the unsupported rod; solving it in
        // the curved supported section displaces the shared outlet node and
        // creates exactly the hinge it is intended to prevent.

    }

    #solveKirchhoffFoldLimits(body) {
        // A Kirchhoff rod stores bending in the relative orientation of its
        // material frames.  Moving the three position nodes here (the legacy
        // implementation) creates a second, incompatible bend operator:
        // adaptation restores x' = d3, the positional limiter moves x again,
        // and the two projections can cycle indefinitely.  Project the same
        // unilateral tangent-angle inequality onto the adjacent material
        // frames instead.  The ordinary no-shear adaptation constraint then
        // transports that bounded turn back to the centerline.
        const unsupportedStart = Number.isFinite(body.sheathMaterialEndNode)
            ? Math.floor(body.sheathMaterialEndNode)
            : body.activeStart + 1;
        const start = Math.max(1, body.activeStart + 1, unsupportedStart);
        const end = Math.min(body.count - 1, body.activeEnd);
        if (start >= end) return;
        const x = body.x;
        const y = body.y;
        const z = body.z;
        const orientationX = body.orientationX;
        const orientationY = body.orientationY;
        const orientationZ = body.orientationZ;
        const orientationW = body.orientationW;
        const inverseInertia1 = body.inverseInertia1;
        const inverseInertia2 = body.inverseInertia2;
        const inverseInertia3 = body.inverseInertia3;
        const maximumBendAngle = body.maxBendAngleByNode;
        const scratch = body.kirchhoffScratch.foldLimit ??= {
            axis: {},
            localAxis: {},
            orientation: {},
            increment: {},
            corrected: {},
            rotation: {},
            limitDegrees: new Float64Array(body.count),
            limitRadians: new Float64Array(body.count),
            limitCosine: new Float64Array(body.count),
            limitCosineSquared: new Float64Array(body.count),
            violationJoints: new Int32Array(body.count),
            violationAxisX: new Float64Array(body.count),
            violationAxisY: new Float64Array(body.count),
            violationAxisZ: new Float64Array(body.count),
            violationCorrection: new Float64Array(body.count)
        };
        if (!scratch.limitCacheInitialized) {
            scratch.limitDegrees.fill(Number.NaN);
            scratch.limitCacheInitialized = true;
        }
        // Frame rotations do not change x/y/z, so the positional tangent
        // angle tested by the fold inequality is identical in both sweeps.
        // Keep a compact ascending list of the joints that fail the forward
        // sweep and visit that list backwards. This is the same reverse order
        // and correction arithmetic without scanning every inactive joint a
        // second time.
        let violationCount = 0;
        for (let sweep = 0; sweep < 2; sweep++) {
            const reverse = sweep === 1;
            const sweepCount = reverse ? violationCount : end - start;
            for (let offset = 0; offset < sweepCount; offset++) {
                const joint = reverse
                    ? scratch.violationJoints[violationCount - 1 - offset]
                    : start + offset;
                let axisX;
                let axisY;
                let axisZ;
                let correction;
                if (reverse) {
                    axisX = scratch.violationAxisX[joint];
                    axisY = scratch.violationAxisY[joint];
                    axisZ = scratch.violationAxisZ[joint];
                    correction = scratch.violationCorrection[joint];
                } else {
                const previousNode = joint - 1;
                const nextNode = joint + 1;
                let incomingX = x[joint] - x[previousNode];
                let incomingY = y[joint] - y[previousNode];
                let incomingZ = z[joint] - z[previousNode];
                let outgoingX = x[nextNode] - x[joint];
                let outgoingY = y[nextNode] - y[joint];
                let outgoingZ = z[nextNode] - z[joint];
                const incomingLengthSquared =
                    incomingX * incomingX + incomingY * incomingY +
                    incomingZ * incomingZ;
                const outgoingLengthSquared =
                    outgoingX * outgoingX + outgoingY * outgoingY +
                    outgoingZ * outgoingZ;
                if (
                    incomingLengthSquared < EPSILON * EPSILON ||
                    outgoingLengthSquared < EPSILON * EPSILON
                ) continue;
                const limitDegrees = clamp(
                    maximumBendAngle[joint],
                    1,
                    179
                );
                if (scratch.limitDegrees[joint] !== limitDegrees) {
                    scratch.limitDegrees[joint] = limitDegrees;
                    const limitRadians = limitDegrees * Math.PI / 180;
                    scratch.limitRadians[joint] = limitRadians;
                    scratch.limitCosine[joint] = Math.cos(limitRadians);
                    scratch.limitCosineSquared[joint] =
                        scratch.limitCosine[joint] *
                        scratch.limitCosine[joint];
                }
                const unnormalizedDot =
                    incomingX * outgoingX +
                    incomingY * outgoingY +
                    incomingZ * outgoingZ;
                const limitCosine = scratch.limitCosine[joint];
                // For the ordinary (< 90 degree) anti-fold bounds, squaring
                // dot >= cos(limit)|u||v| is algebraically equivalent while
                // avoiding both square roots. Keep a deliberately wide
                // roundoff margin; every joint near the unilateral boundary
                // and every violation continues through the original exact
                // normalization/projection below.
                if (limitCosine > 0 && unnormalizedDot > 0) {
                    const lengthProductSquared =
                        incomingLengthSquared * outgoingLengthSquared;
                    const squaredMargin =
                        unnormalizedDot * unnormalizedDot -
                        scratch.limitCosineSquared[joint] *
                            lengthProductSquared;
                    if (squaredMargin > lengthProductSquared * 1e-10) {
                        continue;
                    }
                }
                const incomingLength = Math.sqrt(
                    incomingLengthSquared
                );
                const outgoingLength = Math.sqrt(
                    outgoingLengthSquared
                );
                const lengthProduct = incomingLength * outgoingLength;
                // Far from the unilateral boundary, compare the equivalent
                // unnormalized inequality and skip six divisions. The margin
                // is orders of magnitude above IEEE-754 roundoff for these
                // segment scales; joints near the boundary retain the exact
                // normalized calculation below.
                if (
                    unnormalizedDot -
                        limitCosine * lengthProduct >
                    lengthProduct * 1e-12
                ) continue;
                incomingX /= incomingLength;
                incomingY /= incomingLength;
                incomingZ /= incomingLength;
                outgoingX /= outgoingLength;
                outgoingY /= outgoingLength;
                outgoingZ /= outgoingLength;
                const cosine = clamp(
                    incomingX * outgoingX +
                        incomingY * outgoingY +
                        incomingZ * outgoingZ,
                    -1,
                    1
                );
                if (cosine >= limitCosine) continue;
                scratch.violationJoints[violationCount++] = joint;
                const angle = Math.acos(cosine);
                const limit = scratch.limitRadians[joint];
                axisX = incomingY * outgoingZ -
                    incomingZ * outgoingY;
                axisY = incomingZ * outgoingX -
                    incomingX * outgoingZ;
                axisZ = incomingX * outgoingY -
                    incomingY * outgoingX;
                let axisLength = Math.sqrt(
                    axisX * axisX + axisY * axisY + axisZ * axisZ
                );
                if (axisLength < EPSILON) {
                    if (Math.abs(incomingX) < 0.8) {
                        axisX = 0;
                        axisY = incomingZ;
                        axisZ = -incomingY;
                    } else {
                        axisX = -incomingZ;
                        axisY = 0;
                        axisZ = incomingX;
                    }
                    axisLength = Math.sqrt(
                        axisX * axisX + axisY * axisY + axisZ * axisZ
                    );
                }
                axisX /= axisLength;
                axisY /= axisLength;
                axisZ /= axisLength;
                correction = (angle - limit) * body.foldLimitStrength;
                scratch.violationAxisX[joint] = axisX;
                scratch.violationAxisY[joint] = axisY;
                scratch.violationAxisZ[joint] = axisZ;
                scratch.violationCorrection[joint] = correction;
                }
                const previousSegment = joint - 1;
                const nextSegment = joint;
                let previousWeight;
                if (inverseInertia1[previousSegment] ===
                    inverseInertia2[previousSegment]) {
                    const qx = orientationX[previousSegment];
                    const qy = orientationY[previousSegment];
                    const qz = orientationZ[previousSegment];
                    const qw = orientationW[previousSegment];
                    const d3x = 2 * (qx * qz + qw * qy);
                    const d3y = 2 * (qy * qz - qw * qx);
                    const d3z = 1 - 2 * (qx * qx + qy * qy);
                    const localZ = axisX * d3x + axisY * d3y + axisZ * d3z;
                    const localZSquared = localZ * localZ;
                    previousWeight = Math.max(
                        0,
                        inverseInertia1[previousSegment] *
                            (1 - localZSquared) +
                        inverseInertia3[previousSegment] * localZSquared
                    );
                } else {
                    previousWeight = this.#kirchhoffFoldAngularWeight(
                        body,
                        previousSegment,
                        axisX,
                        axisY,
                        axisZ,
                        scratch
                    );
                }
                let nextWeight;
                if (inverseInertia1[nextSegment] ===
                    inverseInertia2[nextSegment]) {
                    const qx = orientationX[nextSegment];
                    const qy = orientationY[nextSegment];
                    const qz = orientationZ[nextSegment];
                    const qw = orientationW[nextSegment];
                    const d3x = 2 * (qx * qz + qw * qy);
                    const d3y = 2 * (qy * qz - qw * qx);
                    const d3z = 1 - 2 * (qx * qx + qy * qy);
                    const localZ = axisX * d3x + axisY * d3y + axisZ * d3z;
                    const localZSquared = localZ * localZ;
                    nextWeight = Math.max(
                        0,
                        inverseInertia1[nextSegment] *
                            (1 - localZSquared) +
                        inverseInertia3[nextSegment] * localZSquared
                    );
                } else {
                    nextWeight = this.#kirchhoffFoldAngularWeight(
                        body,
                        nextSegment,
                        axisX,
                        axisY,
                        axisZ,
                        scratch
                    );
                }
                const totalWeight = previousWeight + nextWeight;
                if (totalWeight < EPSILON) continue;
                this.#rotateKirchhoffFoldFrame(
                    body,
                    previousSegment,
                    axisX,
                    axisY,
                    axisZ,
                    correction * previousWeight / totalWeight,
                    scratch
                );
                this.#rotateKirchhoffFoldFrame(
                    body,
                    nextSegment,
                    axisX,
                    axisY,
                    axisZ,
                    -correction * nextWeight / totalWeight,
                    scratch
                );
            }
        }
    }

    #kirchhoffFoldAngularWeight(
        body,
        segment,
        axisX,
        axisY,
        axisZ,
        scratch
    ) {
        const inverseInertia1 = body.inverseInertia1[segment];
        const inverseInertia2 = body.inverseInertia2[segment];
        const inverseInertia3 = body.inverseInertia3[segment];
        if (inverseInertia1 === inverseInertia2) {
            // For a circular section the two bending eigenvalues are equal.
            // Only the component along material d3 can see the distinct
            // torsional inertia, so a full inverse quaternion rotation is
            // algebraically unnecessary.
            const qx = body.orientationX[segment];
            const qy = body.orientationY[segment];
            const qz = body.orientationZ[segment];
            const qw = body.orientationW[segment];
            const d3x = 2 * (qx * qz + qw * qy);
            const d3y = 2 * (qy * qz - qw * qx);
            const d3z = 1 - 2 * (qx * qx + qy * qy);
            const localZ = axisX * d3x + axisY * d3y + axisZ * d3z;
            const localZSquared = localZ * localZ;
            return Math.max(
                0,
                inverseInertia1 * (1 - localZSquared) +
                    inverseInertia3 * localZSquared
            );
        }
        scratch.orientation.x = body.orientationX[segment];
        scratch.orientation.y = body.orientationY[segment];
        scratch.orientation.z = body.orientationZ[segment];
        scratch.orientation.w = body.orientationW[segment];
        scratch.axis.x = axisX;
        scratch.axis.y = axisY;
        scratch.axis.z = axisZ;
        inverseRotateVectorByQuaternion(
            scratch.orientation,
            scratch.axis,
            scratch.localAxis
        );
        return Math.max(0,
            scratch.localAxis.x * scratch.localAxis.x *
                inverseInertia1 +
            scratch.localAxis.y * scratch.localAxis.y *
                inverseInertia2 +
            scratch.localAxis.z * scratch.localAxis.z *
                inverseInertia3
        );
    }

    #rotateKirchhoffFoldFrame(
        body,
        segment,
        axisX,
        axisY,
        axisZ,
        angle,
        scratch
    ) {
        if (Math.abs(angle) < EPSILON) return;
        const qx = body.orientationX[segment];
        const qy = body.orientationY[segment];
        const qz = body.orientationZ[segment];
        const qw = body.orientationW[segment];
        const rotationX = axisX * angle;
        const rotationY = axisY * angle;
        const rotationZ = axisZ * angle;
        const angleSquared = rotationX * rotationX +
            rotationY * rotationY + rotationZ * rotationZ;
        let vectorScale;
        let scalar;
        if (angleSquared < TRIG_SERIES_ANGLE_SQUARED) {
            const angleFourth = angleSquared * angleSquared;
            const angleSixth = angleFourth * angleSquared;
            const angleEighth = angleFourth * angleFourth;
            vectorScale = 0.5 - angleSquared / 48 +
                angleFourth / 3840 - angleSixth / 645120 +
                angleEighth / 185794560;
            scalar = 1 - angleSquared / 8 + angleFourth / 384 -
                angleSixth / 46080 + angleEighth / 10321920;
        } else {
            const rotationAngle = Math.sqrt(angleSquared);
            const halfAngle = rotationAngle * 0.5;
            vectorScale = Math.sin(halfAngle) / rotationAngle;
            scalar = Math.cos(halfAngle);
        }
        const ix = rotationX * vectorScale;
        const iy = rotationY * vectorScale;
        const iz = rotationZ * vectorScale;
        const x = scalar * qx + ix * qw + iy * qz - iz * qy;
        const y = scalar * qy - ix * qz + iy * qw + iz * qx;
        const z = scalar * qz + ix * qy - iy * qx + iz * qw;
        const w = scalar * qw - ix * qx - iy * qy - iz * qz;
        const length = Math.sqrt(x * x + y * y + z * z + w * w);
        if (length < EPSILON) {
            body.orientationX[segment] = 0;
            body.orientationY[segment] = 0;
            body.orientationZ[segment] = 0;
            body.orientationW[segment] = 1;
            return;
        }
        const inverseLength = 1 / length;
        body.orientationX[segment] = x * inverseLength;
        body.orientationY[segment] = y * inverseLength;
        body.orientationZ[segment] = z * inverseLength;
        body.orientationW[segment] = w * inverseLength;
    }

    #kirchhoffRuntimeContactRecord(
        constraint,
        innerSegment,
        outerSegment,
        slot,
        kind,
        feature,
        innerT,
        outerT,
        radialDistance,
        clearance,
        normalX,
        normalY,
        normalZ,
        gapOverride = null,
        effectiveTwistRadius = radialDistance
    ) {
        constraint._kirchhoffRuntimeRecordPool ??= [];
        const segmentPool = constraint._kirchhoffRuntimeRecordPool[
            innerSegment
        ] ??= [];
        const record = segmentPool[slot] ??= {
            id: '',
            kind,
            feature,
            gap: 0,
            violation: 0,
            active: false,
            clearance: 0,
            radialDistance: 0,
            normal: new Float64Array(3),
            innerT: 0,
            outerT: 0,
            innerWeights: new Float64Array(2),
            outerWeights: new Float64Array(2),
            contactScratch: {
                innerAxis: new Float64Array(3),
                outerAxis: new Float64Array(3),
                sharedAxis: new Float64Array(3),
                innerIncrement: new Float64Array(3),
                outerIncrement: new Float64Array(3),
                tangentResult: {},
                twistResult: {},
                tangentOptions: { frictionCoefficient: 0, out: null },
                twistOptions: {
                    frictionCoefficient: 0,
                    effectiveRadius: 0,
                    out: null
                }
            },
            manifoldContact: null,
            cachedManifoldContact: null,
            _innerSegmentIndex: -1,
            _outerSegmentIndex: -1,
            _innerMaterialSegmentId: null,
            _outerMaterialSegmentId: null
        };
        const gap = Number.isFinite(gapOverride)
            ? gapOverride
            : clearance - radialDistance;
        const innerWeight0 = 1 - innerT;
        const outerWeight0 = 1 - outerT;
        record.kind = kind;
        // The Jacobian must use the same radial cusp branch as this collector.
        record.normalRadialEpsilon = EPSILON;
        record.feature = feature;
        record.gap = gap;
        record.violation = Math.max(0, -gap);
        record.active = gap < 0;
        record.clearance = clearance;
        record.radialDistance = radialDistance;
        record.normal[0] = normalX;
        record.normal[1] = normalY;
        record.normal[2] = normalZ;
        record.innerT = innerT;
        record.outerT = outerT;
        record.innerWeights[0] = innerWeight0;
        record.innerWeights[1] = innerT;
        record.outerWeights[0] = outerWeight0;
        record.outerWeights[1] = outerT;
        // Ordinary segment contacts use the two endpoint weights above.  A
        // smooth material-coordinate lumen sample may replace them with cubic
        // centreline weights after this pooled record has been initialized.
        // Clear the optional stencil here so portal/rim records can never
        // inherit a stencil from an earlier side contact in the same slot.
        record._innerNodeIndices = null;
        record._innerNodeWeights = null;
        record._outerNodeIndices = null;
        record._outerNodeWeights = null;
        record.normalGradients = null;
        const sameMaterialSegments = constraint._kirchhoffMappingLocked &&
            record._innerSegmentIndex === innerSegment &&
            record._outerSegmentIndex === outerSegment;
        const innerMaterialSegmentId = sameMaterialSegments
            ? record._innerMaterialSegmentId
            : constraint.innerBody.materialCoordinate?.[innerSegment] ??
                innerSegment;
        const outerMaterialSegmentId = sameMaterialSegments
            ? record._outerMaterialSegmentId
            : constraint.outerBody.materialCoordinate?.[outerSegment] ??
                outerSegment;
        const materialChanged =
            record._innerMaterialSegmentId !== innerMaterialSegmentId ||
            record._outerMaterialSegmentId !== outerMaterialSegmentId;
        record.id ||= `${feature}|runtime:${innerSegment}:${slot}`;
        record._innerMaterialSegmentId = innerMaterialSegmentId;
        record._outerMaterialSegmentId = outerMaterialSegmentId;
        record._innerSegmentIndex = innerSegment;
        record._outerSegmentIndex = outerSegment;
        record._normalReference = this.#kirchhoffContactNormalOffset(
            record, constraint.innerBody, constraint.outerBody
        );
        const cached = record.cachedManifoldContact;
        const cachedBelongsToManifold =
            cached?._manifold === constraint.manifold;
        const hasStoredImpulse = hasKirchhoffContactImpulse(cached, constraint.manifold);
        // An open unilateral constraint with positive gap and no stored
        // impulse has no physical state to project. Keep evaluating its exact
        // geometry every sweep, but avoid rebuilding a tangent basis and
        // touching the persistent manifold until it closes or must unload a
        // previous impulse.
        if (
            gap > 0 &&
            !hasStoredImpulse &&
            cachedBelongsToManifold &&
            !materialChanged &&
            cached.innerSegmentIndex === innerSegment &&
            cached.outerSegmentIndex === outerSegment
        ) {
            record.manifoldContact = constraint.manifold
                .touchKnownOpenContact(cached);
            return record;
        }
        if (gap <= constraint.kirchhoffContactActivation || hasStoredImpulse) {
            const upsertOptions = constraint._kirchhoffUpsertOptions ??= {};
            upsertOptions.id = record.id;
            upsertOptions.preserveLambdas = this.lastCoupledSolver === 'joint' &&
                constraint._kirchhoffMappingLocked && cachedBelongsToManifold &&
                cached.innerMaterialSegmentId === innerMaterialSegmentId && cached.feature === feature;
            upsertOptions.innerMaterialSegmentId = innerMaterialSegmentId;
            upsertOptions.outerMaterialSegmentId = outerMaterialSegmentId;
            upsertOptions.feature = feature;
            upsertOptions.innerSegmentIndex = innerSegment;
            upsertOptions.outerSegmentIndex = outerSegment;
            upsertOptions.normal = record.normal;
            upsertOptions.tangentU = constraint._kirchhoffRuntimeAxis;
            upsertOptions.frictionCoefficient =
                constraint.axialFriction;
            upsertOptions.projectFriction = this.lastCoupledSolver !== 'joint';
            upsertOptions.twistFrictionCoefficient =
                constraint.torsionalFriction;
            upsertOptions.effectiveTwistRadius = effectiveTwistRadius;
            if (cachedBelongsToManifold) {
                record.manifoldContact = !materialChanged
                    ? constraint.manifold.refreshKnownContact(
                        cached,
                        upsertOptions
                    )
                    : constraint.manifold.rekeyKnownContact(
                        cached,
                        upsertOptions
                    );
            } else {
                record.manifoldContact = constraint.manifold.upsertContact(
                    upsertOptions
                );
            }
            record.cachedManifoldContact = record.manifoldContact;
        } else {
            record.manifoldContact = null;
        }
        return record;
    }

    #kirchhoffRuntimeLumenRecords(
        constraint,
        innerSegment,
        outerSegment,
        openDistal,
        includeSide = true
    ) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        constraint._kirchhoffRuntimeAxis ??= new Float64Array(3);
        const innerStartX = inner.x[innerSegment];
        const innerStartY = inner.y[innerSegment];
        const innerStartZ = inner.z[innerSegment];
        const innerDirectionX = inner.x[innerSegment + 1] - innerStartX;
        const innerDirectionY = inner.y[innerSegment + 1] - innerStartY;
        const innerDirectionZ = inner.z[innerSegment + 1] - innerStartZ;
        const innerRadius = Math.max(
            inner.nodeRadius[innerSegment],
            inner.nodeRadius[innerSegment + 1]
        );
        const clearance = Math.max(0, constraint.innerRadius - innerRadius);
        constraint._kirchhoffFeaturePrefix ??=
            `containment:${inner.id}:${outer.id}`;
        constraint._kirchhoffRuntimeAxis ??= new Float64Array(3);
        constraint._kirchhoffSideFeature ??=
            `${constraint._kirchhoffFeaturePrefix}:side`;
        constraint._kirchhoffRimFeature ??=
            `${constraint._kirchhoffFeaturePrefix}:distal-rim`;
        constraint._kirchhoffFilletFeature ??=
            `${constraint._kirchhoffFeaturePrefix}:distal-fillet`;
        const records = constraint._kirchhoffRuntimeRecords ??=
            new Array(7);
        const separateSideSamples = this.lastCoupledSolver === 'joint';
        records.fill(null);
        const lastOuter = Math.min(
            outer.segmentCount - 1,
            outer.activeEnd - 1
        );
        const outerStartX = outer.x[outerSegment];
        const outerStartY = outer.y[outerSegment];
        const outerStartZ = outer.z[outerSegment];
        const outerEndX = outer.x[outerSegment + 1];
        const outerEndY = outer.y[outerSegment + 1];
        const outerEndZ = outer.z[outerSegment + 1];
        const outerDirectionX = outerEndX - outerStartX;
        const outerDirectionY = outerEndY - outerStartY;
        const outerDirectionZ = outerEndZ - outerStartZ;
        const outerLengthSquared =
            outerDirectionX * outerDirectionX +
            outerDirectionY * outerDirectionY +
            outerDirectionZ * outerDirectionZ;
        const inverseOuterLength = outerLengthSquared > EPSILON
            ? 1 / Math.sqrt(outerLengthSquared)
            : 0;
        const axisX = outerDirectionX * inverseOuterLength;
        const axisY = outerDirectionY * inverseOuterLength;
        const axisZ = outerDirectionZ * inverseOuterLength;
        const rawOuterTBySample =
            constraint._kirchhoffRawOuterTBySample ??=
                new Float64Array(5);
        let includedSampleMask = 0;
        if (outerLengthSquared > EPSILON) {
            for (let sampleIndex = 0; sampleIndex < 5; sampleIndex++) {
                const innerT = sampleIndex * 0.25;
                const pointX = innerStartX + innerDirectionX * innerT;
                const pointY = innerStartY + innerDirectionY * innerT;
                const pointZ = innerStartZ + innerDirectionZ * innerT;
                const rawOuterT = (
                    (pointX - outerStartX) * outerDirectionX +
                    (pointY - outerStartY) * outerDirectionY +
                    (pointZ - outerStartZ) * outerDirectionZ
                ) / outerLengthSquared;
                rawOuterTBySample[sampleIndex] = rawOuterT;
                if (rawOuterT >= -1e-9 && rawOuterT <= 1 + 1e-9) {
                    includedSampleMask |= 1 << sampleIndex;
                }
            }
        } else {
            rawOuterTBySample.fill(Infinity);
        }
        // An open side constraint with a positive clearance has no multiplier
        // or coordinate correction. Distances between two moving segments are
        // Lipschitz with the sum of their maximum endpoint displacements. If
        // that conservative motion bound cannot consume the cached clearance,
        // the exact five-sample evaluation is guaranteed to stay open. Portal
        // segments are excluded because their plane classification is a
        // separate feature.
        const openCacheStride = 14;
        const openCacheLength = inner.segmentCount * openCacheStride;
        if (
            !constraint._kirchhoffOpenGapCache ||
            constraint._kirchhoffOpenGapCache.length < openCacheLength
        ) {
            constraint._kirchhoffOpenGapCache = new Float64Array(
                openCacheLength
            );
            constraint._kirchhoffOpenGapCache.fill(Number.NaN);
            constraint._kirchhoffOpenOuterSegment = new Int32Array(
                inner.segmentCount
            );
            constraint._kirchhoffOpenOuterSegment.fill(-1);
            constraint._kirchhoffOpenSampleMask = new Uint8Array(
                inner.segmentCount
            );
        }
        const openCache = constraint._kirchhoffOpenGapCache;
        const openCacheOffset = innerSegment * openCacheStride;
        if (
            !openDistal &&
            constraint._kirchhoffOpenOuterSegment[innerSegment] ===
                outerSegment &&
            constraint._kirchhoffOpenSampleMask[innerSegment] ===
                includedSampleMask &&
            openCache[openCacheOffset + 1] === clearance &&
            Number.isFinite(openCache[openCacheOffset])
        ) {
            const innerStartMovement = magnitude3(
                innerStartX - openCache[openCacheOffset + 2],
                innerStartY - openCache[openCacheOffset + 3],
                innerStartZ - openCache[openCacheOffset + 4]
            );
            const innerEndMovement = magnitude3(
                innerStartX + innerDirectionX -
                    openCache[openCacheOffset + 5],
                innerStartY + innerDirectionY -
                    openCache[openCacheOffset + 6],
                innerStartZ + innerDirectionZ -
                    openCache[openCacheOffset + 7]
            );
            const outerStartMovement = magnitude3(
                outerStartX - openCache[openCacheOffset + 8],
                outerStartY - openCache[openCacheOffset + 9],
                outerStartZ - openCache[openCacheOffset + 10]
            );
            const outerEndMovement = magnitude3(
                outerEndX - openCache[openCacheOffset + 11],
                outerEndY - openCache[openCacheOffset + 12],
                outerEndZ - openCache[openCacheOffset + 13]
            );
            const maximumRelativeMovement = Math.max(
                innerStartMovement,
                innerEndMovement
            ) + Math.max(outerStartMovement, outerEndMovement);
            if (
                openCache[openCacheOffset] - maximumRelativeMovement >
                constraint.kirchhoffContactActivation
            ) {
                constraint.kirchhoffOpenCacheHits =
                    (constraint.kirchhoffOpenCacheHits ?? 0) + 1;
                return records;
            }
        }
        // A portal may have no valid side sample to supply its azimuth. Its
        // fallback must still be radial: a world-X vector is not perpendicular
        // to an oblique catheter and gives a non-unit fillet normal on axis.
        let fallbackNormalX = 1 - axisX * axisX;
        let fallbackNormalY = -axisX * axisY;
        let fallbackNormalZ = -axisX * axisZ;
        let fallbackLength = magnitude3(fallbackNormalX, fallbackNormalY, fallbackNormalZ);
        if (fallbackLength <= EPSILON) {
            fallbackNormalX = -axisY * axisX;
            fallbackNormalY = 1 - axisY * axisY;
            fallbackNormalZ = -axisY * axisZ;
            fallbackLength = magnitude3(fallbackNormalX, fallbackNormalY, fallbackNormalZ);
        }
        fallbackNormalX /= fallbackLength;
        fallbackNormalY /= fallbackLength;
        fallbackNormalZ /= fallbackLength;
        let worstGap = Infinity;
        let worstInnerT = 0;
        let worstOuterT = 0;
        let worstOuterSegment = outerSegment;
        let worstRadius = 0;
        let worstAxisX = 1;
        let worstAxisY = 0;
        let worstAxisZ = 0;
        let worstNormalX = 1;
        let worstNormalY = 0;
        let worstNormalZ = 0;
        for (let sampleIndex = 0; sampleIndex < 5; sampleIndex++) {
            const innerT = sampleIndex * 0.25;
            const pointX = innerStartX + innerDirectionX * innerT;
            const pointY = innerStartY + innerDirectionY * innerT;
            const pointZ = innerStartZ + innerDirectionZ * innerT;
            if (outerLengthSquared <= EPSILON) continue;
            const rawOuterT = rawOuterTBySample[sampleIndex];
            if (rawOuterT < -1e-9 || rawOuterT > 1 + 1e-9) continue;
            const outerT = clamp(rawOuterT, 0, 1);
            const offsetX = pointX -
                (outerStartX + outerDirectionX * outerT);
            const offsetY = pointY -
                (outerStartY + outerDirectionY * outerT);
            const offsetZ = pointZ -
                (outerStartZ + outerDirectionZ * outerT);
            const radius = magnitude3(offsetX, offsetY, offsetZ);
            if (
                openDistal &&
                outerSegment === lastOuter &&
                (pointX - outerEndX) * axisX +
                    (pointY - outerEndY) * axisY +
                    (pointZ - outerEndZ) * axisZ >=
                        -Math.max(0, constraint.portalFilletRadius)
            ) continue;
            let normalX;
            let normalY;
            let normalZ;
            if (radius > EPSILON) {
                normalX = offsetX / radius;
                normalY = offsetY / radius;
                normalZ = offsetZ / radius;
            } else {
                let referenceX = 0;
                let referenceY = 0;
                let referenceZ = 0;
                if (
                    Math.abs(axisX) <= Math.abs(axisY) &&
                    Math.abs(axisX) <= Math.abs(axisZ)
                ) referenceX = 1;
                else if (Math.abs(axisY) <= Math.abs(axisZ)) referenceY = 1;
                else referenceZ = 1;
                const axial = referenceX * axisX +
                    referenceY * axisY + referenceZ * axisZ;
                normalX = referenceX - axisX * axial;
                normalY = referenceY - axisY * axial;
                normalZ = referenceZ - axisZ * axial;
                const normalLength = magnitude3(normalX, normalY, normalZ);
                normalX /= normalLength;
                normalY /= normalLength;
                normalZ /= normalLength;
            }
            fallbackNormalX = normalX;
            fallbackNormalY = normalY;
            fallbackNormalZ = normalZ;
            const gap = clearance - radius;
            if (gap >= worstGap - EPSILON) continue;
            worstGap = gap;
            worstInnerT = innerT;
            worstOuterT = outerT;
            worstOuterSegment = outerSegment;
            worstRadius = radius;
            worstAxisX = axisX;
            worstAxisY = axisY;
            worstAxisZ = axisZ;
            worstNormalX = normalX;
            worstNormalY = normalY;
            worstNormalZ = normalZ;
        }
        // The five regular samples are sufficient on an ordinary lumen
        // segment, but the distal side interval ends at a plane which moves
        // continuously through the crossing guidewire cell. Evaluate that
        // clipped endpoint analytically. Without it, a new 1.25 mm sample is
        // captured at once and a strongly preformed Pigtail receives a visible
        // impulse each time the material end node advances.
        let portalBoundaryInnerT = Number.NaN;
        if (
            openDistal &&
            outerSegment === lastOuter &&
            outerLengthSquared > EPSILON
        ) {
            const sideBoundaryAxial = -Math.max(
                0,
                constraint.portalFilletRadius
            );
            const startAxial =
                (innerStartX - outerEndX) * axisX +
                (innerStartY - outerEndY) * axisY +
                (innerStartZ - outerEndZ) * axisZ;
            const axialDelta = innerDirectionX * axisX +
                innerDirectionY * axisY +
                innerDirectionZ * axisZ;
            if (Math.abs(axialDelta) > EPSILON) {
                const candidateT =
                    (sideBoundaryAxial - startAxial) / axialDelta;
                if (candidateT >= 0 && candidateT <= 1) {
                    portalBoundaryInnerT = candidateT;
                    const pointX = innerStartX +
                        innerDirectionX * candidateT;
                    const pointY = innerStartY +
                        innerDirectionY * candidateT;
                    const pointZ = innerStartZ +
                        innerDirectionZ * candidateT;
                    const rawOuterT = (
                        (pointX - outerStartX) * outerDirectionX +
                        (pointY - outerStartY) * outerDirectionY +
                        (pointZ - outerStartZ) * outerDirectionZ
                    ) / outerLengthSquared;
                    if (rawOuterT >= -1e-9 && rawOuterT <= 1 + 1e-9) {
                        const outerT = clamp(rawOuterT, 0, 1);
                        const offsetX = pointX - (
                            outerStartX + outerDirectionX * outerT
                        );
                        const offsetY = pointY - (
                            outerStartY + outerDirectionY * outerT
                        );
                        const offsetZ = pointZ - (
                            outerStartZ + outerDirectionZ * outerT
                        );
                        const radius = magnitude3(
                            offsetX,
                            offsetY,
                            offsetZ
                        );
                        let normalX = fallbackNormalX;
                        let normalY = fallbackNormalY;
                        let normalZ = fallbackNormalZ;
                        if (radius > EPSILON) {
                            normalX = offsetX / radius;
                            normalY = offsetY / radius;
                            normalZ = offsetZ / radius;
                        }
                        const gap = clearance - radius;
                        if (gap < worstGap - EPSILON) {
                            worstGap = gap;
                            worstInnerT = candidateT;
                            worstOuterT = outerT;
                            worstOuterSegment = outerSegment;
                            worstRadius = radius;
                            worstAxisX = axisX;
                            worstAxisY = axisY;
                            worstAxisZ = axisZ;
                            worstNormalX = normalX;
                            worstNormalY = normalY;
                            worstNormalZ = normalZ;
                        }
                    }
                }
            }
        }
        if (
            !openDistal &&
            Number.isFinite(worstGap) &&
            worstGap > constraint.kirchhoffContactActivation
        ) {
            openCache[openCacheOffset] = worstGap;
            openCache[openCacheOffset + 1] = clearance;
            openCache[openCacheOffset + 2] = innerStartX;
            openCache[openCacheOffset + 3] = innerStartY;
            openCache[openCacheOffset + 4] = innerStartZ;
            openCache[openCacheOffset + 5] =
                innerStartX + innerDirectionX;
            openCache[openCacheOffset + 6] =
                innerStartY + innerDirectionY;
            openCache[openCacheOffset + 7] =
                innerStartZ + innerDirectionZ;
            openCache[openCacheOffset + 8] = outerStartX;
            openCache[openCacheOffset + 9] = outerStartY;
            openCache[openCacheOffset + 10] = outerStartZ;
            openCache[openCacheOffset + 11] = outerEndX;
            openCache[openCacheOffset + 12] = outerEndY;
            openCache[openCacheOffset + 13] = outerEndZ;
            constraint._kirchhoffOpenOuterSegment[innerSegment] =
                outerSegment;
            constraint._kirchhoffOpenSampleMask[innerSegment] =
                includedSampleMask;
        } else {
            openCache[openCacheOffset] = Number.NaN;
        }
        if (
            includeSide && !separateSideSamples &&
            worstGap <= constraint.kirchhoffContactActivation
        ) {
            constraint._kirchhoffRuntimeAxis[0] = worstAxisX;
            constraint._kirchhoffRuntimeAxis[1] = worstAxisY;
            constraint._kirchhoffRuntimeAxis[2] = worstAxisZ;
            records[0] = this.#kirchhoffRuntimeContactRecord(
                constraint,
                innerSegment,
                worstOuterSegment,
                4, // Slots 0..3 belong to the smooth side quadrature.
                'side',
                constraint._kirchhoffSideFeature,
                worstInnerT,
                worstOuterT,
                worstRadius,
                clearance,
                worstNormalX,
                worstNormalY,
                worstNormalZ
            );
        }

        if (openDistal) {
            const outerLength = magnitude3(
                outerDirectionX,
                outerDirectionY,
                outerDirectionZ
            );
            if (outerLength <= EPSILON) return records;
            const portalAxisX = outerDirectionX / outerLength;
            const portalAxisY = outerDirectionY / outerLength;
            const portalAxisZ = outerDirectionZ / outerLength;
            constraint._kirchhoffRuntimeAxis[0] = portalAxisX;
            constraint._kirchhoffRuntimeAxis[1] = portalAxisY;
            constraint._kirchhoffRuntimeAxis[2] = portalAxisZ;
            const filletRadius = Math.max(
                0,
                constraint.portalFilletRadius
            );
            if (filletRadius > EPSILON) {
                let worstFilletGap = Infinity;
                let worstFilletInnerT = 0;
                let worstFilletRadius = 0;
                let worstFilletNormalX = 0;
                let worstFilletNormalY = 0;
                let worstFilletNormalZ = 0;
                const filletMajorRadius = clearance + filletRadius;
                const filletSampleCount = Number.isFinite(
                    portalBoundaryInnerT
                ) ? 6 : 5;
                for (
                    let sampleIndex = 0;
                    sampleIndex < filletSampleCount;
                    sampleIndex++
                ) {
                    const innerT = sampleIndex < 5
                        ? sampleIndex * 0.25
                        : portalBoundaryInnerT;
                    const pointX = innerStartX + innerDirectionX * innerT;
                    const pointY = innerStartY + innerDirectionY * innerT;
                    const pointZ = innerStartZ + innerDirectionZ * innerT;
                    const offsetX = pointX - outerEndX;
                    const offsetY = pointY - outerEndY;
                    const offsetZ = pointZ - outerEndZ;
                    const axial = offsetX * portalAxisX +
                        offsetY * portalAxisY +
                        offsetZ * portalAxisZ;
                    if (axial < -filletRadius || axial > filletRadius) {
                        continue;
                    }
                    const radialX = offsetX - portalAxisX * axial;
                    const radialY = offsetY - portalAxisY * axial;
                    const radialZ = offsetZ - portalAxisZ * axial;
                    const radialRadius = magnitude3(
                        radialX,
                        radialY,
                        radialZ
                    );
                    // This is the inner quadrant of the rounded lip. The
                    // external catheter-wall contact owns points beyond its
                    // major radius.
                    if (radialRadius > filletMajorRadius) continue;
                    const radialInverse = radialRadius > EPSILON
                        ? 1 / radialRadius
                        : 0;
                    const radialUnitX = radialRadius > EPSILON
                        ? radialX * radialInverse
                        : fallbackNormalX;
                    const radialUnitY = radialRadius > EPSILON
                        ? radialY * radialInverse
                        : fallbackNormalY;
                    const radialUnitZ = radialRadius > EPSILON
                        ? radialZ * radialInverse
                        : fallbackNormalZ;
                    const circleAxial = axial + filletRadius;
                    const circleRadial = radialRadius - filletMajorRadius;
                    const circleDistance = Math.hypot(
                        circleAxial,
                        circleRadial
                    );
                    if (circleDistance <= EPSILON) continue;
                    const gap = circleDistance - filletRadius;
                    // The shared contact block applies -normal to the
                    // inner rod. Negating the signed-distance gradient moves
                    // the wire centre away from the rounded solid lip.
                    const inverseCircleDistance = 1 / circleDistance;
                    const gradientAxial =
                        circleAxial * inverseCircleDistance;
                    const gradientRadial =
                        circleRadial * inverseCircleDistance;
                    if (separateSideSamples) {
                        const slot = 16 + sampleIndex;
                        const record = this.#kirchhoffRuntimeContactRecord(constraint, innerSegment, outerSegment,
                            slot, 'distal-fillet', constraint._kirchhoffFilletFeature, innerT, 1,
                            filletRadius - gap, filletRadius,
                            -(portalAxisX * gradientAxial + radialUnitX * gradientRadial),
                            -(portalAxisY * gradientAxial + radialUnitY * gradientRadial),
                            -(portalAxisZ * gradientAxial + radialUnitZ * gradientRadial), gap, radialRadius);
                        record.normalInnerParameterMode = sampleIndex < 5 ? 'fixed' : 'portal-side-boundary';
                        records[slot] = record;
                    }
                    if (gap >= worstFilletGap) continue;
                    worstFilletGap = gap;
                    worstFilletInnerT = innerT;
                    worstFilletRadius = radialRadius;
                    worstFilletNormalX = -(
                        portalAxisX * gradientAxial +
                        radialUnitX * gradientRadial
                    );
                    worstFilletNormalY = -(
                        portalAxisY * gradientAxial +
                        radialUnitY * gradientRadial
                    );
                    worstFilletNormalZ = -(
                        portalAxisZ * gradientAxial +
                        radialUnitZ * gradientRadial
                    );
                }
                if (
                    !separateSideSamples && worstFilletGap <=
                        constraint.kirchhoffContactActivation
                ) {
                    records[6] = this.#kirchhoffRuntimeContactRecord(
                        constraint,
                        innerSegment,
                        outerSegment,
                        6,
                        'distal-fillet',
                        constraint._kirchhoffFilletFeature,
                        worstFilletInnerT,
                        1,
                        filletRadius - worstFilletGap,
                        filletRadius,
                        worstFilletNormalX,
                        worstFilletNormalY,
                        worstFilletNormalZ,
                        worstFilletGap,
                        worstFilletRadius
                    );
                }
            }
            const startAxial =
                (innerStartX - outerEndX) * portalAxisX +
                (innerStartY - outerEndY) * portalAxisY +
                (innerStartZ - outerEndZ) * portalAxisZ;
            const endAxial =
                (innerStartX + innerDirectionX - outerEndX) * portalAxisX +
                (innerStartY + innerDirectionY - outerEndY) * portalAxisY +
                (innerStartZ + innerDirectionZ - outerEndZ) * portalAxisZ;
            const axialDelta = endAxial - startAxial;
            if (
                Math.abs(axialDelta) > EPSILON &&
                Math.min(startAxial, endAxial) <= 1e-9 &&
                Math.max(startAxial, endAxial) >= -1e-9
            ) {
                const innerT = clamp(-startAxial / axialDelta, 0, 1);
                const crossingX = innerStartX + innerDirectionX * innerT;
                const crossingY = innerStartY + innerDirectionY * innerT;
                const crossingZ = innerStartZ + innerDirectionZ * innerT;
                const portalOffsetX = crossingX - outerEndX;
                const portalOffsetY = crossingY - outerEndY;
                const portalOffsetZ = crossingZ - outerEndZ;
                const residualAxial = portalOffsetX * portalAxisX +
                    portalOffsetY * portalAxisY +
                    portalOffsetZ * portalAxisZ;
                const radialX = portalOffsetX - portalAxisX * residualAxial;
                const radialY = portalOffsetY - portalAxisY * residualAxial;
                const radialZ = portalOffsetZ - portalAxisZ * residualAxial;
                const radius = magnitude3(radialX, radialY, radialZ);
                const portalClearance = clearance + filletRadius;
                const gap = portalClearance - radius;
                if (gap <= constraint.kirchhoffContactActivation) {
                    const inverseRadius = radius > EPSILON ? 1 / radius : 0;
                    records[5] = this.#kirchhoffRuntimeContactRecord(
                        constraint,
                        innerSegment,
                        outerSegment,
                        5,
                        'distal-rim',
                        constraint._kirchhoffRimFeature,
                        innerT,
                        1,
                        radius,
                        portalClearance,
                        radius > EPSILON ? radialX * inverseRadius : fallbackNormalX,
                        radius > EPSILON ? radialY * inverseRadius : fallbackNormalY,
                        radius > EPSILON ? radialZ * inverseRadius : fallbackNormalZ
                    );
                }
            }
        }
        if (separateSideSamples && includeSide) {
            const pool = constraint._jointPortalSideSamples ??= [];
            const samples = buildKirchhoffPortalSideSamples(inner, outer, innerSegment, outerSegment,
                clearance, openDistal ? constraint.portalFilletRadius : 0, pool[innerSegment] ??= {}).samples;
            constraint._kirchhoffRuntimeAxis[0] = axisX;
            constraint._kirchhoffRuntimeAxis[1] = axisY;
            constraint._kirchhoffRuntimeAxis[2] = axisZ;
            for (const sample of samples) {
                // The high clipping point belongs to the fillet boundary
                // sample, or to distal-rim for a sharp opening. Emit it once.
                if (openDistal && sample.plane === 1) continue;
                const slot = 8 + sample.endpoint;
                const record = this.#kirchhoffRuntimeContactRecord(constraint, innerSegment, outerSegment,
                    slot, 'side', constraint._kirchhoffSideFeature, sample.innerT, sample.outerT,
                    sample.radius, clearance, ...sample.normal);
                record.portalSideGradients = sample.gradients;
                records[slot] = record;
            }
        }
        return records;
    }

    #kirchhoffSmoothCenterlineSample(
        body,
        segment,
        t,
        sample,
        forceLinear = false
    ) {
        const firstNode = Math.max(0, body.activeStart);
        const lastNode = Math.min(body.count - 1, body.activeEnd);
        const useCubic = !forceLinear &&
            segment - 1 >= firstNode && segment + 2 <= lastNode;
        const nodes = sample.nodes;
        const weights = sample.weights;
        const derivativeWeights = sample.derivativeWeights;
        const secondDerivativeWeights = sample.secondDerivativeWeights;
        if (useCubic) {
            const t2 = t * t;
            const t3 = t2 * t;
            nodes[0] = segment - 1;
            nodes[1] = segment;
            nodes[2] = segment + 1;
            nodes[3] = segment + 2;
            // Uniform cubic B-spline basis. Unlike interpolating Catmull-Rom,
            // these non-negative weights cannot overshoot a sharply bent
            // control polygon and invent a lumen excursion between valid rod
            // nodes. The same smooth basis on both meshes is the continuum
            // centreline used by the nonconforming contact quadrature.
            weights[0] = (1 - 3 * t + 3 * t2 - t3) / 6;
            weights[1] = (4 - 6 * t2 + 3 * t3) / 6;
            weights[2] = (1 + 3 * t + 3 * t2 - 3 * t3) / 6;
            weights[3] = t3 / 6;
            derivativeWeights[0] = (-3 + 6 * t - 3 * t2) / 6;
            derivativeWeights[1] = (-12 * t + 9 * t2) / 6;
            derivativeWeights[2] = (3 + 6 * t - 9 * t2) / 6;
            derivativeWeights[3] = 3 * t2 / 6;
            secondDerivativeWeights[0] = 1 - t;
            secondDerivativeWeights[1] = -2 + 3 * t;
            secondDerivativeWeights[2] = 1 - 3 * t;
            secondDerivativeWeights[3] = t;
            sample.count = 4;
        } else {
            nodes[0] = segment;
            nodes[1] = segment + 1;
            weights[0] = 1 - t;
            weights[1] = t;
            derivativeWeights[0] = -1;
            derivativeWeights[1] = 1;
            secondDerivativeWeights[0] = 0;
            secondDerivativeWeights[1] = 0;
            sample.count = 2;
        }
        let pointX = 0;
        let pointY = 0;
        let pointZ = 0;
        let tangentX = 0;
        let tangentY = 0;
        let tangentZ = 0;
        let secondX = 0;
        let secondY = 0;
        let secondZ = 0;
        for (let index = 0; index < sample.count; index++) {
            const node = nodes[index];
            const weight = weights[index];
            const derivativeWeight = derivativeWeights[index];
            const secondDerivativeWeight = secondDerivativeWeights[index];
            pointX += body.x[node] * weight;
            pointY += body.y[node] * weight;
            pointZ += body.z[node] * weight;
            tangentX += body.x[node] * derivativeWeight;
            tangentY += body.y[node] * derivativeWeight;
            tangentZ += body.z[node] * derivativeWeight;
            secondX += body.x[node] * secondDerivativeWeight;
            secondY += body.y[node] * secondDerivativeWeight;
            secondZ += body.z[node] * secondDerivativeWeight;
        }
        sample.derivative[0] = tangentX;
        sample.derivative[1] = tangentY;
        sample.derivative[2] = tangentZ;
        sample.secondDerivative[0] = secondX;
        sample.secondDerivative[1] = secondY;
        sample.secondDerivative[2] = secondZ;
        const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
        if (tangentLength > EPSILON) {
            tangentX /= tangentLength;
            tangentY /= tangentLength;
            tangentZ /= tangentLength;
        } else {
            const dx = body.x[segment + 1] - body.x[segment];
            const dy = body.y[segment + 1] - body.y[segment];
            const dz = body.z[segment + 1] - body.z[segment];
            const length = Math.max(EPSILON, magnitude3(dx, dy, dz));
            tangentX = dx / length;
            tangentY = dy / length;
            tangentZ = dz / length;
        }
        sample.point[0] = pointX;
        sample.point[1] = pointY;
        sample.point[2] = pointZ;
        sample.tangent[0] = tangentX;
        sample.tangent[1] = tangentY;
        sample.tangent[2] = tangentZ;
        return sample;
    }

    #prepareKirchhoffOuterMaterialArc(constraint, outerStart, outerLast) {
        const outer = constraint.outerBody;
        const requiredLength = outer.segmentCount + 1;
        if (
            !constraint._kirchhoffOuterArcAtNode ||
            constraint._kirchhoffOuterArcAtNode.length < requiredLength
        ) {
            constraint._kirchhoffOuterArcAtNode = new Float64Array(
                requiredLength
            );
        }
        const arc = constraint._kirchhoffOuterArcAtNode;
        arc[outerStart] = 0;
        for (let segment = outerStart; segment <= outerLast; segment++) {
            arc[segment + 1] = arc[segment] + Math.max(
                EPSILON,
                outer.restLength[segment]
            );
        }
        constraint._kirchhoffOuterMaterialLength = arc[outerLast + 1];
        return arc;
    }

    #kirchhoffOuterSegmentAtMaterialArc(
        constraint,
        materialArc,
        outerStart,
        outerLast,
        expectedOuterSegment
    ) {
        const arc = constraint._kirchhoffOuterArcAtNode;
        let segment = clamp(
            expectedOuterSegment,
            outerStart,
            outerLast
        );
        while (segment > outerStart && materialArc < arc[segment]) segment--;
        while (
            segment < outerLast &&
            materialArc > arc[segment + 1]
        ) segment++;
        return segment;
    }

    #kirchhoffSmoothMaterialSideRecord(
        constraint,
        innerSegment,
        innerArcStart,
        containedSpanFraction,
        outerStart,
        outerLast,
        expectedOuterSegment,
        emitRecord = true,
        applyFriction = false,
        sideMeasurements = null
    ) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        constraint._kirchhoffFeaturePrefix ??=
            `containment:${inner.id}:${outer.id}`;
        constraint._kirchhoffRuntimeAxis ??= new Float64Array(3);
        const emittedRecords = emitRecord
            ? (constraint._kirchhoffSmoothRuntimeRecords ??= new Array(4))
            : null;
        emittedRecords?.fill(null);
        const scratch = constraint._kirchhoffSmoothContactScratch ??= {
            inner: {
                nodes: new Int32Array(4),
                weights: new Float64Array(4),
                derivativeWeights: new Float64Array(4),
                secondDerivativeWeights: new Float64Array(4),
                point: new Float64Array(3),
                tangent: new Float64Array(3),
                derivative: new Float64Array(3),
                secondDerivative: new Float64Array(3),
                count: 0
            },
            outer: {
                nodes: new Int32Array(4),
                weights: new Float64Array(4),
                derivativeWeights: new Float64Array(4),
                secondDerivativeWeights: new Float64Array(4),
                point: new Float64Array(3),
                tangent: new Float64Array(3),
                derivative: new Float64Array(3),
                secondDerivative: new Float64Array(3),
                count: 0
            },
            innerNodes: new Int32Array(4),
            innerWeights: new Float64Array(4),
            outerNodes: new Int32Array(4),
            outerWeights: new Float64Array(4)
        };
        const innerRestLength = Math.max(
            EPSILON,
            inner.restLength[innerSegment]
        );
        const containedLength = Math.max(0, constraint.containedLength);
        const outerMaterialLength = constraint._kirchhoffOuterMaterialLength;
        const arc = constraint._kirchhoffOuterArcAtNode;
        let worstGap = Infinity;
        let worstInnerT = 0;
        let worstOuterT = 0;
        let worstOuterSegment = expectedOuterSegment;
        let worstRadius = 0;
        let worstNormalX = 1;
        let worstNormalY = 0;
        let worstNormalZ = 0;
        let worstAxisX = 1;
        let worstAxisY = 0;
        let worstAxisZ = 0;
        let worstInnerCount = 0;
        let worstOuterCount = 0;
        let searchSegment = expectedOuterSegment;
        // Integrate the nonconforming lumen constraint in the interior of
        // each material cell. Sampling both cell endpoints made adjacent
        // segments impose two independently mapped contact constraints on
        // the same material point. Those duplicate constraints fought after
        // every length/pre-shape projection and produced the observed
        // two-cycle and lateral jumps while feeding the wire.
        const quadratureCount = 4;
        for (let sampleIndex = 0; sampleIndex < quadratureCount; sampleIndex++) {
            const innerT = (sampleIndex + 0.5) / quadratureCount;
            const materialArc = innerArcStart + innerRestLength * innerT;
            if (
                materialArc > containedLength + EPSILON ||
                materialArc > outerMaterialLength + EPSILON
            ) continue;
            searchSegment = this.#kirchhoffOuterSegmentAtMaterialArc(
                constraint,
                materialArc,
                outerStart,
                outerLast,
                searchSegment
            );
            const innerSample = this.#kirchhoffSmoothCenterlineSample(
                inner,
                innerSegment,
                innerT,
                scratch.inner,
                containedSpanFraction < 1 - EPSILON ||
                    innerArcStart - innerRestLength < -EPSILON ||
                    innerArcStart + 2 * innerRestLength >
                        containedLength + EPSILON
            );
            const materialOuterSegment = searchSegment;
            const localSearchWindow = Math.max(
                1,
                Math.floor(constraint.searchWindow)
            );
            const candidateStart = Math.max(
                outerStart,
                materialOuterSegment - localSearchWindow
            );
            const candidateEnd = Math.min(
                outerLast,
                materialOuterSegment + localSearchWindow
            );
            const closest = closestKirchhoffCenterlinePoint(
                outer, innerSample.point, candidateStart, candidateEnd,
                materialOuterSegment, constraint._kirchhoffClosestCurve ??= {},
                constraint._kirchhoffCurveCache
            );
            searchSegment = closest.segment;
            const outerT = closest.t;
            const outerSample = this.#kirchhoffSmoothCenterlineSample(
                outer,
                searchSegment,
                outerT,
                scratch.outer
            );
            const axisX = outerSample.tangent[0];
            const axisY = outerSample.tangent[1];
            const axisZ = outerSample.tangent[2];
            if (constraint.openDistal && searchSegment === outerLast) {
                // The final cell is cut by the open portal plane. Its side,
                // rounded lip and aperture must be classified together by the
                // exact clipped portal evaluator below; a smooth side sample
                // here would double-constrain the same crossing.
                continue;
            }
            const offsetX = innerSample.point[0] - outerSample.point[0];
            const offsetY = innerSample.point[1] - outerSample.point[1];
            const offsetZ = innerSample.point[2] - outerSample.point[2];
            const axial = offsetX * axisX + offsetY * axisY + offsetZ * axisZ;
            const radialX = offsetX - axisX * axial;
            const radialY = offsetY - axisY * axial;
            const radialZ = offsetZ - axisZ * axial;
            const radius = magnitude3(radialX, radialY, radialZ);
            const innerRadius = Math.max(
                inner.nodeRadius[innerSegment],
                inner.nodeRadius[innerSegment + 1]
            );
            const clearance = Math.max(0, constraint.innerRadius - innerRadius);
            const gap = clearance - radius;
            let normalX;
            let normalY;
            let normalZ;
            if (radius > EPSILON) {
                normalX = radialX / radius;
                normalY = radialY / radius;
                normalZ = radialZ / radius;
            } else {
                let referenceX = Math.abs(axisX) < 0.8 ? 1 : 0;
                let referenceY = referenceX === 0 ? 1 : 0;
                let referenceZ = 0;
                const projection = referenceX * axisX +
                    referenceY * axisY + referenceZ * axisZ;
                referenceX -= axisX * projection;
                referenceY -= axisY * projection;
                referenceZ -= axisZ * projection;
                const length = Math.max(
                    EPSILON,
                    magnitude3(referenceX, referenceY, referenceZ)
                );
                normalX = referenceX / length;
                normalY = referenceY / length;
                normalZ = referenceZ / length;
            }
            if (
                emitRecord &&
                (gap <= constraint.kirchhoffContactActivation || hasKirchhoffContactImpulse(
                    constraint._kirchhoffRuntimeRecordPool?.[innerSegment]?.[sampleIndex]?.cachedManifoldContact,
                    constraint.manifold))
            ) {
                constraint._kirchhoffRuntimeAxis[0] = axisX;
                constraint._kirchhoffRuntimeAxis[1] = axisY;
                constraint._kirchhoffRuntimeAxis[2] = axisZ;
                const record = this.#kirchhoffRuntimeContactRecord(
                    constraint,
                    innerSegment,
                    searchSegment,
                    sampleIndex,
                    'material-side',
                    `${constraint._kirchhoffFeaturePrefix}:side`,
                    innerT,
                    outerT,
                    radius,
                    clearance,
                    normalX,
                    normalY,
                    normalZ
                );
                record._smoothInnerNodeIndices ??= new Int32Array(4);
                record._smoothInnerNodeWeights ??= new Float64Array(4);
                record._smoothOuterNodeIndices ??= new Int32Array(4);
                record._smoothOuterNodeWeights ??= new Float64Array(4);
                for (let index = 0; index < innerSample.count; index++) {
                    record._smoothInnerNodeIndices[index] =
                        innerSample.nodes[index];
                    record._smoothInnerNodeWeights[index] =
                        innerSample.weights[index];
                }
                for (let index = 0; index < outerSample.count; index++) {
                    record._smoothOuterNodeIndices[index] =
                        outerSample.nodes[index];
                    record._smoothOuterNodeWeights[index] =
                        outerSample.weights[index];
                }
                record._innerNodeIndices = record._smoothInnerNodeIndices;
                record._innerNodeWeights = record._smoothInnerNodeWeights;
                record._innerNodeCount = innerSample.count;
                record._outerNodeIndices = record._smoothOuterNodeIndices;
                record._outerNodeWeights = record._smoothOuterNodeWeights;
                record._outerNodeCount = outerSample.count;
                record._normalReference = this.#kirchhoffContactNormalOffset(record, inner, outer);
                record._containedSpanFraction = containedSpanFraction;
                emittedRecords[sampleIndex] = record;
                constraint.kirchhoffContacts.push(record);
                constraint.kirchhoffMaxViolation = Math.max(
                    constraint.kirchhoffMaxViolation,
                    record.violation
                );
                this.#solveKirchhoffLumenRecord(
                    constraint,
                    record,
                    applyFriction
                );
            }
            if (gap >= worstGap - EPSILON) continue;
            worstGap = gap;
            worstInnerT = innerT;
            worstOuterT = outerT;
            worstOuterSegment = searchSegment;
            worstRadius = radius;
            worstAxisX = axisX;
            worstAxisY = axisY;
            worstAxisZ = axisZ;
            worstNormalX = normalX;
            worstNormalY = normalY;
            worstNormalZ = normalZ;
            worstInnerCount = innerSample.count;
            worstOuterCount = outerSample.count;
            for (let index = 0; index < innerSample.count; index++) {
                scratch.innerNodes[index] = innerSample.nodes[index];
                scratch.innerWeights[index] = innerSample.weights[index];
            }
            for (let index = 0; index < outerSample.count; index++) {
                scratch.outerNodes[index] = outerSample.nodes[index];
                scratch.outerWeights[index] = outerSample.weights[index];
            }
        }
        if (sideMeasurements) sideMeasurements[innerSegment] = Number.isFinite(worstGap) ? {
            violation: Math.max(0, -worstGap), innerSegment, outerSegment: worstOuterSegment,
            innerT: worstInnerT, outerT: worstOuterT, radialDistance: worstRadius
        } : { violation: 0 };
        if (!Number.isFinite(worstGap)) {
            return emitRecord ? emittedRecords : { violation: 0 };
        }
        if (!emitRecord) {
            return {
                violation: Math.max(0, -worstGap),
                innerSegment,
                outerSegment: worstOuterSegment,
                innerT: worstInnerT,
                outerT: worstOuterT,
                radialDistance: worstRadius
            };
        }
        return emittedRecords;
    }

    #kirchhoffContactWeight(record, inner, outer) {
        const innerSegment = record.manifoldContact.innerSegmentIndex;
        const outerSegment = record.manifoldContact.outerSegmentIndex;
        if (record._innerNodeIndices && record._outerNodeIndices) {
            let weight = 0;
            for (let index = 0; index < record._innerNodeCount; index++) {
                const node = record._innerNodeIndices[index];
                weight += inner.inverseMass[node] *
                    record._innerNodeWeights[index] ** 2;
            }
            for (let index = 0; index < record._outerNodeCount; index++) {
                const node = record._outerNodeIndices[index];
                weight += outer.inverseMass[node] *
                    record._outerNodeWeights[index] ** 2;
            }
            return weight;
        }
        return inner.inverseMass[innerSegment] * record.innerWeights[0] ** 2 +
            inner.inverseMass[innerSegment + 1] * record.innerWeights[1] ** 2 +
            outer.inverseMass[outerSegment] * record.outerWeights[0] ** 2 +
            outer.inverseMass[outerSegment + 1] * record.outerWeights[1] ** 2;
    }

    #applyKirchhoffContactVector(record, vector, lambda, innerSign = 1) {
        if (Math.abs(lambda) < EPSILON) return;
        const contact = record.manifoldContact;
        const inner = record._innerBody;
        const outer = record._outerBody;
        const innerSegment = contact.innerSegmentIndex;
        const outerSegment = contact.outerSegmentIndex;
        const innerNodes = record._innerNodeIndices;
        const outerNodes = record._outerNodeIndices;
        const innerWeights = record._innerNodeWeights;
        const outerWeights = record._outerNodeWeights;
        const innerCount = innerNodes ? record._innerNodeCount : 2;
        const outerCount = outerNodes ? record._outerNodeCount : 2;
        for (let index = 0; index < innerCount; index++) {
            const innerNode = innerNodes?.[index] ?? innerSegment + index;
            const innerWeight = innerWeights?.[index] ?? record.innerWeights[index];
            const innerScale = inner.inverseMass[innerNode] *
                innerWeight * lambda * innerSign;
            inner.x[innerNode] += vector[0] * innerScale;
            inner.y[innerNode] += vector[1] * innerScale;
            inner.z[innerNode] += vector[2] * innerScale;
        }
        for (let index = 0; index < outerCount; index++) {
            const outerNode = outerNodes?.[index] ?? outerSegment + index;
            const outerWeight = outerWeights?.[index] ?? record.outerWeights[index];
            const outerScale = outer.inverseMass[outerNode] *
                outerWeight * lambda * -innerSign;
            outer.x[outerNode] += vector[0] * outerScale;
            outer.y[outerNode] += vector[1] * outerScale;
            outer.z[outerNode] += vector[2] * outerScale;
        }
        // A constraint correction may wake a sleeping coupled body, but must
        // not reset the sleep counter of an already-awake body on every
        // Gauss-Seidel sweep. Doing so made an equilibrated catheter/wire pair
        // permanently consume the full solver budget at zero user input.
        if (inner.sleeping) inner.wake();
        if (outer.sleeping) outer.wake();
    }

    #kirchhoffFrameIncrement(body, segment, out) {
        const scratch = body.kirchhoffScratch.contactFrame;
        scratch.current.x = body.orientationX[segment];
        scratch.current.y = body.orientationY[segment];
        scratch.current.z = body.orientationZ[segment];
        scratch.current.w = body.orientationW[segment];
        scratch.previous.x = body.previousOrientationX[segment];
        scratch.previous.y = body.previousOrientationY[segment];
        scratch.previous.z = body.previousOrientationZ[segment];
        scratch.previous.w = body.previousOrientationW[segment];
        conjugateQuaternion(scratch.previous, scratch.previousInverse);
        multiplyQuaternions(
            scratch.current,
            scratch.previousInverse,
            scratch.relative
        );
        quaternionLog(scratch.relative, scratch.delta, true);
        out[0] = scratch.delta.x;
        out[1] = scratch.delta.y;
        out[2] = scratch.delta.z;
        return out;
    }

    #kirchhoffSegmentAxis(body, segment, out) {
        const dx = body.x[segment + 1] - body.x[segment];
        const dy = body.y[segment + 1] - body.y[segment];
        const dz = body.z[segment + 1] - body.z[segment];
        const length = magnitude3(dx, dy, dz);
        if (length > EPSILON) {
            out[0] = dx / length;
            out[1] = dy / length;
            out[2] = dz / length;
        } else {
            out[0] = 1;
            out[1] = 0;
            out[2] = 0;
        }
        return out;
    }

    #applyKirchhoffFrameTwist(body, segment, angularImpulse, sharedAxis) {
        if (
            Math.abs(angularImpulse) < EPSILON
        ) return;
        const inverseInertia = body.inverseInertia3[segment];
        if (inverseInertia <= 0) return;
        const scratch = body.kirchhoffScratch.contactTwist;
        const axis = sharedAxis ?? this.#kirchhoffSegmentAxis(
            body,
            segment,
            scratch.axis ??= new Float64Array(3)
        );
        const angle = angularImpulse * inverseInertia;
        scratch.angularIncrement.x = axis[0] * angle;
        scratch.angularIncrement.y = axis[1] * angle;
        scratch.angularIncrement.z = axis[2] * angle;
        quaternionExp(scratch.angularIncrement, scratch.increment);
        scratch.current.x = body.orientationX[segment];
        scratch.current.y = body.orientationY[segment];
        scratch.current.z = body.orientationZ[segment];
        scratch.current.w = body.orientationW[segment];
        multiplyQuaternions(
            scratch.increment,
            scratch.current,
            scratch.multiplied
        );
        const orientation = normalizeQuaternion(
            scratch.multiplied,
            scratch.normalized
        );
        body.orientationX[segment] = orientation.x;
        body.orientationY[segment] = orientation.y;
        body.orientationZ[segment] = orientation.z;
        body.orientationW[segment] = orientation.w;
        if (body.sleeping) body.wake();
    }

    #applyKirchhoffTwistCorrection(record, innerImpulse) {
        if (Math.abs(innerImpulse) < EPSILON) return;
        const contact = record.manifoldContact;
        const scratch = record.contactScratch;
        const innerAxis = this.#kirchhoffSegmentAxis(
            record._innerBody,
            contact.innerSegmentIndex,
            scratch.innerAxis
        );
        const outerAxisValue = this.#kirchhoffSegmentAxis(
            record._outerBody,
            contact.outerSegmentIndex,
            scratch.outerAxis
        );
        const direction =
            innerAxis[0] * outerAxisValue[0] +
            innerAxis[1] * outerAxisValue[1] +
            innerAxis[2] * outerAxisValue[2] < 0
                ? -1
                : 1;
        const axisX = innerAxis[0] + outerAxisValue[0] * direction;
        const axisY = innerAxis[1] + outerAxisValue[1] * direction;
        const axisZ = innerAxis[2] + outerAxisValue[2] * direction;
        const axisLength = magnitude3(axisX, axisY, axisZ);
        const sharedAxis = scratch.sharedAxis;
        if (axisLength > EPSILON) {
            sharedAxis[0] = axisX / axisLength;
            sharedAxis[1] = axisY / axisLength;
            sharedAxis[2] = axisZ / axisLength;
        } else {
            sharedAxis[0] = innerAxis[0];
            sharedAxis[1] = innerAxis[1];
            sharedAxis[2] = innerAxis[2];
        }
        this.#applyKirchhoffFrameTwist(
            record._innerBody,
            contact.innerSegmentIndex,
            innerImpulse,
            sharedAxis
        );
        this.#applyKirchhoffFrameTwist(
            record._outerBody,
            contact.outerSegmentIndex,
            -innerImpulse,
            sharedAxis
        );
    }

    #solveKirchhoffLumenRecord(constraint, record, applyFriction, normalIncrement = null) {
        const contact = record.manifoldContact;
        if (!contact) return;
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        record._innerBody = inner;
        record._outerBody = outer;
        {
            // A global reaction moves other samples in the cached manifold.
            const offset = this.#kirchhoffContactNormalOffset(record, inner, outer);
            record.gap -= offset - record._normalReference;
            record._normalReference = offset;
        }
        const weight = this.#kirchhoffContactWeight(record, inner, outer);
        const baseAlpha = Math.max(0, constraint.compliance) /
            (this.fixedDt * this.fixedDt);
        const containedSpanFraction = clamp(
            record._containedSpanFraction ?? 1,
            0,
            1
        );
        const containedResponseFraction = Math.max(
            1e-4,
            containedSpanFraction * containedSpanFraction * (
                3 - 2 * containedSpanFraction
            )
        );
        // One runtime contact represents a finite material cell. At the
        // distal opening only a continuously growing fraction of that cell is
        // inside the catheter. This overlap compliance fades the new contact
        // in by physical length instead of activating a complete 5 mm cell at
        // an integer end-node transition.
        const alpha = baseAlpha + weight * (
            (1 - containedResponseFraction) / containedResponseFraction
        );
        record._normalAlpha = alpha;
        if (constraint._collectContactBlock) return;
        const previousU = contact.tangentLambda[0];
        const previousV = contact.tangentLambda[1];
        const previousTwist = contact.twistLambda;
        constraint.manifold.accumulateKnownNormalLambda(contact, normalIncrement);

        // Normal unloading can shrink both Coulomb bounds. Apply that
        // projection to the rods too, otherwise the manifold and generalized
        // coordinates would disagree about the stored warm-start impulse.
        this.#applyKirchhoffContactVector(
            record,
            contact.tangentU,
            contact.tangentLambda[0] - previousU
        );
        this.#applyKirchhoffContactVector(
            record,
            contact.tangentV,
            contact.tangentLambda[1] - previousV
        );
        this.#applyKirchhoffTwistCorrection(
            record,
            contact.twistLambda - previousTwist
        );
        // Coulomb friction has zero admissible impulse without normal load.
        // The unloading projection above has already removed any stale
        // tangential/twist lambda, so computing relative displacement and two
        // quaternion logarithms for a merely nearby lumen segment is wasted.
        if (
            contact.normalLambda <= EPSILON ||
            !applyFriction ||
            (
                constraint.axialFriction <= 0 &&
                constraint.torsionalFriction <= 0
            ) ||
            weight <= EPSILON
        ) {
            return;
        }

        const innerSegment = contact.innerSegmentIndex;
        const outerSegment = contact.outerSegmentIndex;
        let displacementX = 0;
        let displacementY = 0;
        let displacementZ = 0;
        const innerNodes = record._innerNodeIndices;
        const outerNodes = record._outerNodeIndices;
        const innerWeights = record._innerNodeWeights;
        const outerWeights = record._outerNodeWeights;
        const innerCount = innerNodes ? record._innerNodeCount : 2;
        const outerCount = outerNodes ? record._outerNodeCount : 2;
        for (let index = 0; index < innerCount; index++) {
            const innerNode = innerNodes?.[index] ?? innerSegment + index;
            const innerWeight = innerWeights?.[index] ?? record.innerWeights[index];
            displacementX += (
                inner.x[innerNode] - inner.previousX[innerNode]
            ) * innerWeight;
            displacementY += (
                inner.y[innerNode] - inner.previousY[innerNode]
            ) * innerWeight;
            displacementZ += (
                inner.z[innerNode] - inner.previousZ[innerNode]
            ) * innerWeight;
        }
        for (let index = 0; index < outerCount; index++) {
            const outerNode = outerNodes?.[index] ?? outerSegment + index;
            const outerWeight = outerWeights?.[index] ?? record.outerWeights[index];
            displacementX -= (
                outer.x[outerNode] - outer.previousX[outerNode]
            ) * outerWeight;
            displacementY -= (
                outer.y[outerNode] - outer.previousY[outerNode]
            ) * outerWeight;
            displacementZ -= (
                outer.z[outerNode] - outer.previousZ[outerNode]
            ) * outerWeight;
        }
        const tangentUDisplacement =
            displacementX * contact.tangentU[0] +
            displacementY * contact.tangentU[1] +
            displacementZ * contact.tangentU[2];
        const tangentVDisplacement =
            displacementX * contact.tangentV[0] +
            displacementY * contact.tangentV[1] +
            displacementZ * contact.tangentV[2];
        const contactScratch = record.contactScratch;
        if (constraint.axialFriction > 0) {
            contactScratch.tangentOptions.frictionCoefficient =
                constraint.axialFriction;
            contactScratch.tangentOptions.out = contactScratch.tangentResult;
            const friction = constraint.manifold
                .accumulateKnownTangentialLambda(
                    contact,
                    -tangentUDisplacement / weight,
                    -tangentVDisplacement / weight,
                    contactScratch.tangentOptions
                );
            this.#applyKirchhoffContactVector(
                record,
                contact.tangentU,
                friction.appliedU
            );
            this.#applyKirchhoffContactVector(
                record,
                contact.tangentV,
                friction.appliedV
            );
        }

        const innerInverseInertia = inner.inverseInertia3[innerSegment];
        const outerInverseInertia = outer.inverseInertia3[outerSegment];
        const angularWeight = innerInverseInertia + outerInverseInertia;
        if (
            angularWeight <= EPSILON ||
            constraint.torsionalFriction <= 0
        ) return;
        const innerAxis = this.#kirchhoffSegmentAxis(
            inner,
            innerSegment,
            contactScratch.innerAxis
        );
        const outerAxisValue = this.#kirchhoffSegmentAxis(
            outer,
            outerSegment,
            contactScratch.outerAxis
        );
        const outerDirection =
            innerAxis[0] * outerAxisValue[0] +
            innerAxis[1] * outerAxisValue[1] +
            innerAxis[2] * outerAxisValue[2] < 0
                ? -1
                : 1;
        const sharedAxisX = innerAxis[0] +
            outerAxisValue[0] * outerDirection;
        const sharedAxisY = innerAxis[1] +
            outerAxisValue[1] * outerDirection;
        const sharedAxisZ = innerAxis[2] +
            outerAxisValue[2] * outerDirection;
        const sharedAxisLength = magnitude3(
            sharedAxisX,
            sharedAxisY,
            sharedAxisZ
        );
        const sharedAxis = contactScratch.sharedAxis;
        if (sharedAxisLength > EPSILON) {
            sharedAxis[0] = sharedAxisX / sharedAxisLength;
            sharedAxis[1] = sharedAxisY / sharedAxisLength;
            sharedAxis[2] = sharedAxisZ / sharedAxisLength;
        } else {
            sharedAxis[0] = innerAxis[0];
            sharedAxis[1] = innerAxis[1];
            sharedAxis[2] = innerAxis[2];
        }
        const innerIncrement = this.#kirchhoffFrameIncrement(
            inner,
            innerSegment,
            contactScratch.innerIncrement
        );
        const outerIncrement = this.#kirchhoffFrameIncrement(
            outer,
            outerSegment,
            contactScratch.outerIncrement
        );
        const relativeTwist =
            innerIncrement[0] * sharedAxis[0] +
            innerIncrement[1] * sharedAxis[1] +
            innerIncrement[2] * sharedAxis[2] -
            outerIncrement[0] * sharedAxis[0] -
            outerIncrement[1] * sharedAxis[1] -
            outerIncrement[2] * sharedAxis[2];
        contactScratch.twistOptions.frictionCoefficient =
            constraint.torsionalFriction;
        contactScratch.twistOptions.effectiveRadius = Math.max(
            inner.nodeRadius[innerSegment],
            inner.nodeRadius[innerSegment + 1]
        );
        contactScratch.twistOptions.out = contactScratch.twistResult;
        const twist = constraint.manifold.accumulateKnownTwistImpulse(
            contact,
            -relativeTwist / angularWeight,
            contactScratch.twistOptions
        );
        this.#applyKirchhoffTwistCorrection(record, twist.appliedInner);
    }

    #kirchhoffContactNormalOffset(record, inner, outer) {
        let result = 0;
        for (let side = 0; side < 2; side++) {
            const body = side === 0 ? inner : outer;
            const nodes = side === 0 ? record._innerNodeIndices : record._outerNodeIndices;
            const weights = side === 0 ? record._innerNodeWeights : record._outerNodeWeights;
            const segment = side === 0 ? record._innerSegmentIndex : record._outerSegmentIndex;
            const count = nodes ? (side === 0 ? record._innerNodeCount : record._outerNodeCount) : 2;
            const fallback = side === 0 ? record.innerWeights : record.outerWeights;
            for (let i = 0; i < count; i++) {
                const node = nodes ? nodes[i] : segment + i;
                result += (side === 0 ? 1 : -1) * (weights ? weights[i] : fallback[i]) * (
                    body.x[node] * record.normal[0] + body.y[node] * record.normal[1] +
                    body.z[node] * record.normal[2]);
            }
        }
        return result;
    }

    #kirchhoffContactSolverResidual(constraint) {
        let maximum = 0;
        for (const record of constraint.kirchhoffContacts) {
            const contact = record.manifoldContact;
            if (!contact) continue;
            const gap = record.gap - (this.#kirchhoffContactNormalOffset(
                record, constraint.innerBody, constraint.outerBody
            ) - record._normalReference);
            // XPBD equilibrium is g + alpha*lambda = 0 on a loaded compliant
            // cell. Demanding g = 0 makes a fractional entering cell consume
            // every closure pass even after it has reached that equilibrium.
            // Raw geometric penetration remains separately reported above.
            const residual = gap + (record._normalAlpha ?? 0) * contact.normalLambda;
            maximum = Math.max(maximum, contact.normalLambda > EPSILON
                ? Math.abs(residual) : Math.max(0, -gap));
        }
        return maximum;
    }

    #solveKirchhoffSlidingPortal(constraint, applyFriction) {

        const evaluatePortal = this.lastCoupledSolver === 'joint' ? evaluateKirchhoffOwnedSlidingPortal : evaluateKirchhoffSlidingPortal;
        const state = evaluatePortal(
            constraint, constraint._slidingPortalState ??= {}
        );
        if (state.segment < 0 || state.distance <= EPSILON) return;
        this.#kirchhoffSegmentAxis(constraint.innerBody, state.segment,
            constraint._kirchhoffRuntimeAxis);
        const record = this.#kirchhoffRuntimeContactRecord(
            constraint, state.segment, constraint.outerBody.activeEnd - 1,
            7, 'sliding-rim', 'lumen:sliding-rim', state.t, 1,
            state.distance, state.clearance, state.x / state.distance,
            state.y / state.distance, state.z / state.distance
        );
        if (!record.manifoldContact) return;
        record._containedSpanFraction = 1;
        constraint.kirchhoffContacts.push(record);
        // The same rigid lumen clearance and Coulomb law apply at the mouth
        // and along its side. A separate soft portal spring permits large
        // penetration when a stiff catheter preform presses against the wire.
        this.#solveKirchhoffLumenRecord(constraint, record, applyFriction);
    }

    #measureKirchhoffContainmentViolation(
        constraint,
        includeSide = true,
        distalSideOnly = false,
        portalTransitionOnly = false
    ) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            Math.max(inner.activeStart, inner.activeEnd - 1)
        );
        const innerEnd = Math.min(
            inner.segmentCount - 1,
            inner.activeEnd - 1,
            // endNode is the last fully captured material node. Its outgoing
            // segment is the continuously moving lumen/free-space crossing
            // and must remain in the portal solve; omitting it captures a
            // complete guidewire cell at once whenever endNode advances.
            Math.max(innerStart, constraint.endNode)
        );
        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            Math.max(outer.activeStart, outer.activeEnd - 1)
        );
        const outerLast = Math.min(
            outer.segmentCount - 1,
            outer.activeEnd - 1
        );
        if (innerEnd < innerStart || outerLast < outerStart) return 0;
        let maximumViolation = 0;
        for (
            let innerSegment = innerStart;
            innerSegment <= innerEnd;
            innerSegment++
        ) {
            // The distal aperture is a material boundary: endNode is the last
            // captured node and its outgoing segment is the unique
            // lumen/free-space transition. Other wire segments may cross the
            // same spatial plane after looping in the vessel; those are rim
            // contacts, not additional exits from the catheter. Runtime rim
            // contact is intentionally still solved for every nearby segment.
            if (portalTransitionOnly && innerSegment !== innerEnd) continue;
            const outerSegment =
                constraint.kirchhoffOuterSegmentByInner[innerSegment];
            if (outerSegment < outerStart || outerSegment > outerLast) continue;
            const outerStartX = outer.x[outerSegment];
            const outerStartY = outer.y[outerSegment];
            const outerStartZ = outer.z[outerSegment];
            const outerDirectionX = outer.x[outerSegment + 1] - outerStartX;
            const outerDirectionY = outer.y[outerSegment + 1] - outerStartY;
            const outerDirectionZ = outer.z[outerSegment + 1] - outerStartZ;
            const outerLengthSquared =
                outerDirectionX * outerDirectionX +
                outerDirectionY * outerDirectionY +
                outerDirectionZ * outerDirectionZ;
            if (outerLengthSquared <= EPSILON) continue;
            const inverseOuterLength = 1 / Math.sqrt(outerLengthSquared);
            const axisX = outerDirectionX * inverseOuterLength;
            const axisY = outerDirectionY * inverseOuterLength;
            const axisZ = outerDirectionZ * inverseOuterLength;
            const innerStartX = inner.x[innerSegment];
            const innerStartY = inner.y[innerSegment];
            const innerStartZ = inner.z[innerSegment];
            const innerDirectionX = inner.x[innerSegment + 1] - innerStartX;
            const innerDirectionY = inner.y[innerSegment + 1] - innerStartY;
            const innerDirectionZ = inner.z[innerSegment + 1] - innerStartZ;
            const innerRadius = Math.max(
                inner.nodeRadius[innerSegment],
                inner.nodeRadius[innerSegment + 1]
            );
            const clearance = Math.max(
                0,
                constraint.innerRadius - innerRadius
            );
            const openDistal = constraint.openDistal &&
                outerSegment === outerLast;
            const outerEndX = outer.x[outerSegment + 1];
            const outerEndY = outer.y[outerSegment + 1];
            const outerEndZ = outer.z[outerSegment + 1];
            const filletRadius = openDistal
                ? Math.max(0, constraint.portalFilletRadius)
                : 0;
            for (let sampleIndex = 0; sampleIndex < 5; sampleIndex++) {
                const innerT = sampleIndex * 0.25;
                const pointX = innerStartX + innerDirectionX * innerT;
                const pointY = innerStartY + innerDirectionY * innerT;
                const pointZ = innerStartZ + innerDirectionZ * innerT;
                let distalAxial = -Infinity;
                if (openDistal) {
                    const offsetX = pointX - outerEndX;
                    const offsetY = pointY - outerEndY;
                    const offsetZ = pointZ - outerEndZ;
                    distalAxial = offsetX * axisX +
                        offsetY * axisY + offsetZ * axisZ;
                    if (
                        filletRadius > EPSILON &&
                        distalAxial >= -filletRadius &&
                        distalAxial <= filletRadius
                    ) {
                        const radialX = offsetX - axisX * distalAxial;
                        const radialY = offsetY - axisY * distalAxial;
                        const radialZ = offsetZ - axisZ * distalAxial;
                        const radialRadius = magnitude3(
                            radialX,
                            radialY,
                            radialZ
                        );
                        const majorRadius = clearance + filletRadius;
                        if (radialRadius <= majorRadius) {
                            const circleDistance = Math.hypot(
                                distalAxial + filletRadius,
                                radialRadius - majorRadius
                            );
                            maximumViolation = Math.max(
                                maximumViolation,
                                filletRadius - circleDistance
                            );
                        }
                    }
                }
                const rawOuterT = (
                    (pointX - outerStartX) * outerDirectionX +
                    (pointY - outerStartY) * outerDirectionY +
                    (pointZ - outerStartZ) * outerDirectionZ
                ) / outerLengthSquared;
                if (rawOuterT < -1e-9 || rawOuterT > 1 + 1e-9) continue;
                if (openDistal && distalAxial >= -filletRadius) continue;
                const outerT = clamp(rawOuterT, 0, 1);
                const radialX = pointX -
                    (outerStartX + outerDirectionX * outerT);
                const radialY = pointY -
                    (outerStartY + outerDirectionY * outerT);
                const radialZ = pointZ -
                    (outerStartZ + outerDirectionZ * outerT);
                if (includeSide && (!distalSideOnly || openDistal)) {
                    maximumViolation = Math.max(
                        maximumViolation,
                        magnitude3(radialX, radialY, radialZ) - clearance
                    );
                }
            }
            if (!openDistal) continue;
            const startAxial =
                (innerStartX - outerEndX) * axisX +
                (innerStartY - outerEndY) * axisY +
                (innerStartZ - outerEndZ) * axisZ;
            const endAxial =
                (innerStartX + innerDirectionX - outerEndX) * axisX +
                (innerStartY + innerDirectionY - outerEndY) * axisY +
                (innerStartZ + innerDirectionZ - outerEndZ) * axisZ;
            const axialDelta = endAxial - startAxial;
            if (
                Math.abs(axialDelta) <= EPSILON ||
                Math.min(startAxial, endAxial) > 1e-9 ||
                Math.max(startAxial, endAxial) < -1e-9
            ) continue;
            const innerT = clamp(-startAxial / axialDelta, 0, 1);
            const radialX = innerStartX + innerDirectionX * innerT - outerEndX;
            const radialY = innerStartY + innerDirectionY * innerT - outerEndY;
            const radialZ = innerStartZ + innerDirectionZ * innerT - outerEndZ;
            const residualAxial = radialX * axisX +
                radialY * axisY + radialZ * axisZ;
            maximumViolation = Math.max(
                maximumViolation,
                magnitude3(
                    radialX - axisX * residualAxial,
                    radialY - axisY * residualAxial,
                    radialZ - axisZ * residualAxial
                ) - (clearance + filletRadius)
            );
        }
        return Math.max(0, maximumViolation);
    }

    #measureKirchhoffCoupledContainmentViolation(constraint, sideMeasurements = null) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            Math.max(inner.activeStart, inner.activeEnd - 1)
        );
        const innerEnd = Math.min(
            inner.segmentCount - 1,
            inner.activeEnd - 1,
            Math.max(innerStart, constraint.endNode)
        );
        const outerLast = Math.min(
            outer.segmentCount - 1,
            outer.activeEnd - 1
        );
        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            outerLast
        );
        if (innerEnd < innerStart || outerLast < outerStart) return 0;
        constraint._kirchhoffCurveCache = prepareKirchhoffCenterlineSearch(
            outer, constraint._kirchhoffCurveCache ??= {}
        );
        this.#prepareKirchhoffOuterMaterialArc(
            constraint,
            outerStart,
            outerLast
        );
        let maximumViolation = 0;
        let expectedOuterSegment = outerStart;
        let outerArcEnd = Math.max(
            EPSILON,
            outer.restLength[expectedOuterSegment]
        );
        let innerArcStart = Math.max(0, constraint.innerArcOffset);
        let transitionArcStart = innerArcStart;
        for (
            let innerSegment = innerStart;
            innerSegment <= innerEnd;
            innerSegment++
        ) {
            const innerRestLength = Math.max(
                EPSILON,
                inner.restLength[innerSegment]
            );
            const innerArcMidpoint = innerArcStart + innerRestLength * 0.5;
            while (
                expectedOuterSegment < outerLast &&
                outerArcEnd < innerArcMidpoint
            ) {
                expectedOuterSegment++;
                outerArcEnd += Math.max(
                    EPSILON,
                    outer.restLength[expectedOuterSegment]
                );
            }
            const containedSpanFraction = Number.isFinite(
                constraint.containedLength
            )
                ? clamp(
                    (
                        constraint.containedLength - innerArcStart
                    ) / innerRestLength,
                    0,
                    1
                )
                : 1;
            if (sideMeasurements?.[innerSegment]) this.lastJointCosts.geometryReuseCount++;
            const measurement = sideMeasurements?.[innerSegment] ?? this.#kirchhoffSmoothMaterialSideRecord(
                constraint,
                innerSegment,
                innerArcStart,
                containedSpanFraction,
                outerStart,
                outerLast,
                expectedOuterSegment,
                false
            );
            maximumViolation = Math.max(
                maximumViolation,
                measurement?.violation ?? 0
            );
            if (
                (measurement?.violation ?? 0) >= maximumViolation - EPSILON
            ) {
                constraint.kirchhoffMeasuredWorstSide = measurement;
            }
            if (innerSegment === innerEnd) {
                transitionArcStart = innerArcStart;
            }
            innerArcStart += innerRestLength;
        }
        // Reuse the exact spatial distal-rim/fillet measurement while excluding
        // its old piecewise-linear side test.  The side residual must be measured
        // with the same smooth material geometry that generated its gradients.
        if (constraint.openDistal) {
            const spatialPortalViolation =
                this.#measureKirchhoffContainmentViolation(
                    constraint,
                    true,
                    true,
                    true
                );
            constraint.kirchhoffMeasuredSideViolation = maximumViolation;
            constraint.kirchhoffMeasuredSpatialPortalViolation =
                spatialPortalViolation;
            {
                const evaluatePortal = this.lastCoupledSolver === 'joint' ? evaluateKirchhoffOwnedSlidingPortal : evaluateKirchhoffSlidingPortal;
                const sliding = evaluatePortal(
                    constraint, constraint._slidingPortalState ??= {}
                );
                const portalViolation = Math.max(spatialPortalViolation, sliding.violation);
                constraint.kirchhoffMeasuredPortalViolation = portalViolation;

                return Math.max(maximumViolation, portalViolation);
            }

        } else {
            constraint.kirchhoffMeasuredSideViolation = maximumViolation;
            constraint.kirchhoffMeasuredPortalViolation = 0;
            constraint.kirchhoffMeasuredSpatialPortalViolation = 0;

        }
        return maximumViolation;
    }

    #solveKirchhoffContainment(
        constraint,
        applyFriction = true,
        measureResidual = false
    ) {
        constraint.kirchhoffContacts.length = 0;
        constraint.kirchhoffMaxViolation = 0;
        if (!constraint.enabled) return;
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        if (inner.sleeping && outer.sleeping) return;
        if (inner.sleeping) inner.wake();
        if (outer.sleeping) outer.wake();
        // Aggregate the complete rods' responses once per step. Subsequent
        // local contact blocks alternate with complete outer material solves.
        constraint._contactBlockDirect = constraint._contactBlockSweeps === 0;
        if (constraint._contactBlockDirect) {
            solveKirchhoffDirect(inner, this.fixedDt, true, this.reuseDirectLinearization);
            solveKirchhoffDirect(outer, this.fixedDt, true, this.reuseDirectLinearization);
        }
        if (!this.#collectKirchhoffContainmentGeometry(constraint, applyFriction)) return;
        // Simultaneous reciprocal normals, followed by Coulomb projection.
        // The outer closure alternates this block with BOTH complete rods.
        // The diagonal shift damps only the increment, never the stored load;
        // final acceptance uses the original physical contact residual.
        // Predictor geometry will change again. Reserve tight linear accuracy
        // for the final closure; its physical acceptance gate is unchanged.
        const linearTolerance = this.coupledContainmentTolerance *
            (this._inCoupledClosure ? 0.2 : 20);
        const block = solveKirchhoffContactBlock(constraint, linearTolerance, 1e-3);
        constraint._contactBlockSweeps++;
        constraint._contactBlockIterations += block.iterations;
        for (let i = 0; i < constraint.kirchhoffContacts.length; i++) {
            this.#solveKirchhoffLumenRecord(constraint, constraint.kirchhoffContacts[i], applyFriction, block.increment[i]);
        }
        constraint._kirchhoffMappingLocked = true;
        if (measureResidual) {
            constraint.kirchhoffMaxViolation =
                this.#measureKirchhoffCoupledContainmentViolation(constraint);
        }
    }

    // Freeze every lumen/portal sample before computing a mechanical update.
    // The collector is shared by the contact-only reference and the upcoming
    // simultaneous material/contact solve; it applies no tool correction.
    #collectKirchhoffContainmentGeometry(constraint, applyFriction, sideMeasurements = null) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        constraint._collectContactBlock = true;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            Math.max(inner.activeStart, inner.activeEnd - 1)
        );
        const innerEnd = Math.min(
            inner.segmentCount - 1,
            inner.activeEnd - 1,
            // Include the fractional distal crossing segment. Runtime side,
            // fillet and rim sampling already reject the part beyond the
            // physical opening, so this changes a discrete node toggle into
            // continuous contact as the catheter advances.
            Math.max(innerStart, constraint.endNode)
        );
        const outerLast = Math.min(outer.segmentCount - 1, outer.activeEnd - 1);
        if (innerEnd < innerStart || outerLast < outer.activeStart) {
            constraint._collectContactBlock = false;
            return false;
        }

        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            outerLast
        );
        constraint._kirchhoffCurveCache = prepareKirchhoffCenterlineSearch(
            outer, constraint._kirchhoffCurveCache ??= {}
        );
        this.#prepareKirchhoffOuterMaterialArc(
            constraint,
            outerStart,
            outerLast
        );
        let expectedOuterSegment = outerStart;
        let outerArcEnd = Math.max(
            EPSILON,
            outer.restLength[expectedOuterSegment]
        );
        let innerArcStart = Math.max(0, constraint.innerArcOffset);

        for (
            let innerSegment = innerStart;
            innerSegment <= innerEnd;
            innerSegment++
        ) {
            const innerRestLength = Math.max(
                EPSILON,
                inner.restLength[innerSegment]
            );
            const segmentContainedFraction = Number.isFinite(
                constraint.containedLength
            )
                ? clamp(
                    (
                        constraint.containedLength - innerArcStart
                    ) / innerRestLength,
                    0,
                    1
                )
                : 1;

            const innerArcMidpoint = innerArcStart + innerRestLength * 0.5;
            while (
                expectedOuterSegment < outerLast &&
                outerArcEnd < innerArcMidpoint
            ) {
                expectedOuterSegment++;
                outerArcEnd += Math.max(
                    EPSILON,
                    outer.restLength[expectedOuterSegment]
                );
            }
            const materialSideRecords =
                this.#kirchhoffSmoothMaterialSideRecord(
                    constraint,
                    innerSegment,
                    innerArcStart,
                    segmentContainedFraction,
                    outerStart,
                    outerLast,
                    expectedOuterSegment,
                    true,
                    applyFriction,
                    sideMeasurements
                );
            innerArcStart += innerRestLength;
            constraint.kirchhoffOuterSegmentByInner[innerSegment] =
                expectedOuterSegment;
            constraint.closestSegment[innerSegment] = expectedOuterSegment;
            constraint.closestT[innerSegment] = clamp(
                (
                    innerArcMidpoint -
                    constraint._kirchhoffOuterArcAtNode[expectedOuterSegment]
                ) / Math.max(
                    EPSILON,
                    outer.restLength[expectedOuterSegment]
                ),
                0,
                1
            );
            const spatialMappingRecord = materialSideRecords?.[2] ??
                materialSideRecords?.find(record => record?.manifoldContact);
            if (spatialMappingRecord?.manifoldContact) {
                constraint.kirchhoffOuterSegmentByInner[innerSegment] =
                    spatialMappingRecord.manifoldContact.outerSegmentIndex;
                constraint.closestSegment[innerSegment] =
                    spatialMappingRecord.manifoldContact.outerSegmentIndex;
                constraint.closestT[innerSegment] = spatialMappingRecord.outerT;
            }
            if (
                !constraint.openDistal ||
                expectedOuterSegment !== outerLast
            ) continue;
            const records = this.#kirchhoffRuntimeLumenRecords(
                constraint,
                innerSegment,
                expectedOuterSegment,
                true,
                true
            );
            for (const record of records) {
                if (!record?.manifoldContact) continue;
                record._containedSpanFraction = segmentContainedFraction;
                constraint.kirchhoffContacts.push(record);
                constraint.kirchhoffMaxViolation = Math.max(
                    constraint.kirchhoffMaxViolation,
                    record.violation
                );
                this.#solveKirchhoffLumenRecord(
                    constraint,
                    record,
                    applyFriction
                );
            }
        }
        this.#solveKirchhoffSlidingPortal(constraint, applyFriction);
        constraint._collectContactBlock = false;
        return true;
    }

    #solveContainment(
        constraint,
        innerOnly = false,
        outerOnly = false,
        applyFriction = true
    ) {
        {
            this.#solveKirchhoffContainment(
                constraint,
                applyFriction,
                false
            );
            return;
        }

    }

    #solveToolContact(constraint, collectRows = null, jointConstraint = null) {
        if (
            constraint.enabled !== constraint._lastEnabled ||
            constraint.startSegmentA !== constraint._lastStartSegmentA ||
            constraint.endSegmentA !== constraint._lastEndSegmentA ||
            constraint.startSegmentB !== constraint._lastStartSegmentB ||
            constraint.endSegmentB !== constraint._lastEndSegmentB
        ) {
            // In a joint trial, range/enable changes retire the old reaction
            // through the common solve. Only the independent reference path
            // retains its separate per-range reset convention.
            if (!collectRows) constraint.lambdas.fill(0);
            constraint._lastEnabled = constraint.enabled;
            constraint._lastStartSegmentA = constraint.startSegmentA;
            constraint._lastEndSegmentA = constraint.endSegmentA;
            constraint._lastStartSegmentB = constraint.startSegmentB;
            constraint._lastEndSegmentB = constraint.endSegmentB;
        }
        const retireUnlisted = collectRows ? seen => {
            for (const [index] of constraint._jointReactions ?? []) if (!seen?.has(index))
                appendKirchhoffToolRelease(jointConstraint, constraint, index, collectRows, 'outside-tool-window');
        } : null;
        if (!constraint.enabled) { retireUnlisted?.(); return; }
        const a = constraint.bodyA;
        const b = constraint.bodyB;
        if (collectRows && (constraint.endSegmentA < constraint.startSegmentA || constraint.endSegmentB < constraint.startSegmentB)) {
            retireUnlisted(); return;
        }
        if (a.sleeping && b.sleeping && !collectRows) return;
        if (a.sleeping) a.wake();
        if (b.sleeping) b.wake();
        const alpha = constraint.compliance / (this.fixedDt * this.fixedDt);
        const aStart = clamp(constraint.startSegmentA, a.activeStart, a.segmentCount - 1);
        const aEnd = clamp(constraint.endSegmentA, aStart, Math.min(a.activeEnd - 1, a.segmentCount - 1));
        const bStart = clamp(constraint.startSegmentB, b.activeStart, b.segmentCount - 1);
        const bEnd = clamp(constraint.endSegmentB, bStart, Math.min(b.activeEnd - 1, b.segmentCount - 1));
        const seen = collectRows ? new Set() : null;
        const branch = collectRows ? locateKirchhoffDistalLumenBranch(jointConstraint) : null;
        for (let ia = aStart; ia <= aEnd; ia++) {
            for (let ib = bStart; ib <= bEnd; ib++) {
                const closest = this.#closestSegmentParameters(a, ia, b, ib, this._segmentParameters);
                const ax = a.x[ia] + (a.x[ia + 1] - a.x[ia]) * closest.s;
                const ay = a.y[ia] + (a.y[ia + 1] - a.y[ia]) * closest.s;
                const az = a.z[ia] + (a.z[ia + 1] - a.z[ia]) * closest.s;
                const bx = b.x[ib] + (b.x[ib + 1] - b.x[ib]) * closest.t;
                const by = b.y[ib] + (b.y[ib + 1] - b.y[ib]) * closest.t;
                const bz = b.z[ib] + (b.z[ib + 1] - b.z[ib]) * closest.t;
                const lambdaIndex = ia * b.segmentCount + ib;
                seen?.add(lambdaIndex);
                let releaseReason = collectRows && constraint._jointReactions?.get(lambdaIndex)?.retiring ? 'pending-release' : null;
                if (collectRows && isKirchhoffDistalLumenWitness(jointConstraint, constraint,
                    ia, ib, closest.s, ax, ay, az, branch)) releaseReason = 'lumen-ownership';
                if (constraint.openDistalB && ib === (collectRows ? b.activeEnd - 1 : bEnd) && closest.t >= 1 - 1e-5) {
                    const endDx = b.x[ib + 1] - b.x[ib];
                    const endDy = b.y[ib + 1] - b.y[ib];
                    const endDz = b.z[ib + 1] - b.z[ib];
                    const beyond =
                        (ax - b.x[ib + 1]) * endDx +
                        (ay - b.y[ib + 1]) * endDy +
                        (az - b.z[ib + 1]) * endDz;
                    if (beyond > 0) releaseReason ??= 'open-distal';
                }
                if (releaseReason) {
                    if (collectRows) appendKirchhoffToolRelease(jointConstraint, constraint, lambdaIndex, collectRows, releaseReason);
                    continue;
                }
                let nx = ax - bx;
                let ny = ay - by;
                let nz = az - bz;
                const distance = magnitude3(nx, ny, nz);
                const minimum = Math.max(a.nodeRadius[ia], a.nodeRadius[ia + 1]) +
                    Math.max(b.nodeRadius[ib], b.nodeRadius[ib + 1]);
                let normalLength = distance;
                if (distance < EPSILON) {
                    if (!collectRows || !hasKirchhoffToolReaction(jointConstraint, constraint, lambdaIndex)) continue;
                    // Coincident axes do not retire a real external contact.
                    // Its previous normal is an admissible one-sided branch.
                    const previous = constraint._jointBoundaryRows?.get(lambdaIndex)?.normal;
                    if (!previous || !(Math.hypot(...previous) > 0))
                        throw new Error('Loaded coincident tool contact has no retained normal');
                    [nx, ny, nz] = previous; normalLength = Math.hypot(nx, ny, nz);
                }
                if (distance >= minimum && (!collectRows ||
                    (distance > minimum + this.contactActivation && !hasKirchhoffToolReaction(jointConstraint, constraint, lambdaIndex)))) continue;
                nx /= normalLength;
                ny /= normalLength;
                nz /= normalLength;
                const aw0 = 1 - closest.s;
                const aw1 = closest.s;
                const bw0 = 1 - closest.t;
                const bw1 = closest.t;
                const wa0 = a.inverseMass[ia] * aw0 * aw0;
                const wa1 = a.inverseMass[ia + 1] * aw1 * aw1;
                const wb0 = b.inverseMass[ib] * bw0 * bw0;
                const wb1 = b.inverseMass[ib + 1] * bw1 * bw1;
                const denominator = wa0 + wa1 + wb0 + wb1 + alpha;
                if (denominator < EPSILON && !collectRows) continue;
                const c = distance - minimum;
                if (collectRows) {
                    const cache = constraint._jointBoundaryRows ??= new Map();
                    let row = cache.get(lambdaIndex);
                    if (!row) {
                        row = { kind: 'tool', gradients: Array.from({ length: 12 }, () => ({})),
                            owner: constraint, node: lambdaIndex, lower: 0, upper: Infinity };
                        cache.set(lambdaIndex, row);
                    }
                    row.strain = c; row.alpha = alpha; row.lambda = constraint.lambdas[lambdaIndex];
                    row.segmentA = ia; row.segmentB = ib;
                    row.bodyA = a; row.bodyB = b;
                    row.tA = closest.s; row.tB = closest.t;
                    row.distance = distance;
                    row.normal ??= new Float64Array(3);
                    row.normal[0] = nx; row.normal[1] = ny; row.normal[2] = nz;
                    row.bodyA = a; row.bodyB = b;
                    let cursor = 0;
                    for (let sideIndex = 0; sideIndex < 2; sideIndex++) {
                        const body = sideIndex === 0 ? a : b;
                        const side = body === jointConstraint.innerBody ? 0 : 1;
                        const segment = sideIndex === 0 ? ia : ib;
                        const weight0 = sideIndex === 0 ? aw0 : bw0, weight1 = sideIndex === 0 ? aw1 : bw1;
                        const sign = sideIndex === 0 ? 1 : -1;
                        for (let endpoint = 0; endpoint < 2; endpoint++) for (let axis = 0; axis < 3; axis++) {
                            const g = row.gradients[cursor++];
                            g.side = side; g.dof = (segment + endpoint) * 6 + axis;
                            g.value = sign * (endpoint ? weight1 : weight0) * (axis === 0 ? nx : axis === 1 ? ny : nz);
                        }
                    }
                    row.reactionWrenches = captureKirchhoffToolReaction(jointConstraint, row.gradients, row.reactionWrenches);
                    collectRows.push(row);
                    continue;
                }

                let deltaLambda = (-c - alpha * constraint.lambdas[lambdaIndex]) / denominator;
                const nextLambda = Math.max(0, constraint.lambdas[lambdaIndex] + deltaLambda);
                deltaLambda = nextLambda - constraint.lambdas[lambdaIndex];
                constraint.lambdas[lambdaIndex] = nextLambda;
                const correctionA0 = deltaLambda * a.inverseMass[ia] * aw0;
                const correctionA1 = deltaLambda * a.inverseMass[ia + 1] * aw1;
                const correctionB0 = -deltaLambda * b.inverseMass[ib] * bw0;
                const correctionB1 = -deltaLambda * b.inverseMass[ib + 1] * bw1;
                a.x[ia] += nx * correctionA0;
                a.y[ia] += ny * correctionA0;
                a.z[ia] += nz * correctionA0;
                a.x[ia + 1] += nx * correctionA1;
                a.y[ia + 1] += ny * correctionA1;
                a.z[ia + 1] += nz * correctionA1;
                b.x[ib] += nx * correctionB0;
                b.y[ib] += ny * correctionB0;
                b.z[ib] += nz * correctionB0;
                b.x[ib + 1] += nx * correctionB1;
                b.y[ib + 1] += ny * correctionB1;
                b.z[ib + 1] += nz * correctionB1;
                a.toolProjectionX[ia] += nx * correctionA0;
                a.toolProjectionY[ia] += ny * correctionA0;
                a.toolProjectionZ[ia] += nz * correctionA0;
                a.toolProjectionX[ia + 1] += nx * correctionA1;
                a.toolProjectionY[ia + 1] += ny * correctionA1;
                a.toolProjectionZ[ia + 1] += nz * correctionA1;
                b.toolProjectionX[ib] += nx * correctionB0;
                b.toolProjectionY[ib] += ny * correctionB0;
                b.toolProjectionZ[ib] += nz * correctionB0;
                b.toolProjectionX[ib + 1] += nx * correctionB1;
                b.toolProjectionY[ib + 1] += ny * correctionB1;
                b.toolProjectionZ[ib + 1] += nz * correctionB1;

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
        retireUnlisted?.(seen);
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
            if (body.sleeping) continue;
            if (sheath.bodies && !sheath.bodies.includes(body)) continue;
            let lambdas = sheath.lambdas.get(body);
            if (!lambdas) {
                lambdas = new Float32Array(body.count);
                sheath.lambdas.set(body, lambdas);
            }
            // Sheath ownership is material, not wall-contact ownership. A
            // distal loop can geometrically pass back through the introducer's
            // axial slab; constraining it would teleport free material onto
            // the sheath axis and amplify the bend. Standalone rods may leave
            // the explicit range unset and are then classified spatially.
            const materialEnd = Math.min(
                body.activeEnd,
                body.sheathMaterialEndNode
            );
            for (let index = body.activeStart; index <= materialEnd; index++) {
                let px = body.x[index] - sheath.startX;
                let py = body.y[index] - sheath.startY;
                let pz = body.z[index] - sheath.startZ;
                let axial = px * sheath.axisX + py * sheath.axisY + pz * sheath.axisZ;
                // The introducer is an open lumen: its contact reaction is
                // radial. Axial feed is supplied by the proximal material
                // boundary/control, not by snapping every contained node to a
                // moving rest-shape coordinate. The latter changes node
                // identity during remeshing and teleports the distal chain.
                if (
                    axial < -sheath.proximalExtension - 1e-5 ||
                    axial > sheath.length + 1e-5
                ) {
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

    #transportDistalLengthError(body, segmentIndex, maximumCorrection = 1.25) {
        const segment = clamp(
            Math.floor(segmentIndex),
            body.activeStart,
            Math.min(body.activeEnd - 1, body.segmentCount - 1)
        );
        const dx = body.x[segment + 1] - body.x[segment];
        const dy = body.y[segment + 1] - body.y[segment];
        const dz = body.z[segment + 1] - body.z[segment];
        const length = magnitude3(dx, dy, dz);
        if (length < EPSILON) return;
        const error = body.restLength[segment] - length;
        if (Math.abs(error) <= 0.01) return;
        // The segment at collisionStart is the material transition just past
        // the sheath. Translate the entire distal sub-chain rigidly so the
        // fractional feed segment closes without changing any downstream
        // length or injecting an angular fold at the outlet.
        const correction = clamp(
            error,
            -maximumCorrection,
            maximumCorrection
        );
        const tx = dx / length * correction;
        const ty = dy / length * correction;
        const tz = dz / length * correction;
        for (let index = segment + 1; index <= body.activeEnd; index++) {
            if (body.inverseMass[index] <= 0) continue;
            body.x[index] += tx;
            body.y[index] += ty;
            body.z[index] += tz;
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
            const w0 = body.inverseMass[index] * w0Factor * w0Factor;
            const w1 = body.inverseMass[index + 1] * w1Factor * w1Factor;
            const denominator = w0 + w1 + alpha;
            if (denominator < EPSILON) continue;
            // Unilateral XPBD reaction for this fixed step. Importantly this
            // permits a negative delta when another constraint unloads the
            // contact; summing raw overlap on every solver pass counted the
            // same normal force repeatedly and made friction iteration-bound.
            let deltaLambda = (-c - alpha * body.wallLambda[index]) / denominator;
            const nextLambda = Math.max(0, body.wallLambda[index] + deltaLambda);
            deltaLambda = nextLambda - body.wallLambda[index];
            const maximumResponseWeight = Math.max(
                body.inverseMass[index] * w0Factor,
                body.inverseMass[index + 1] * w1Factor
            );
            const displacement = maximumResponseWeight * Math.abs(deltaLambda);
            if (
                Number.isFinite(body.wallMaxCorrection) &&
                displacement > body.wallMaxCorrection
            ) {
                deltaLambda *= body.wallMaxCorrection / displacement;
            }
            body.wallLambda[index] += deltaLambda;
            if (body.wallProjectionVelocityRetention >= 1) {
                // Catheter physics intentionally retains its original direct
                // XPBD update. Guidewire-only diagnostics must not alter its
                // arithmetic path or floating-point rounding.
                body.x[index] += nx * deltaLambda *
                    body.inverseMass[index] * w0Factor;
                body.y[index] += ny * deltaLambda *
                    body.inverseMass[index] * w0Factor;
                body.z[index] += nz * deltaLambda *
                    body.inverseMass[index] * w0Factor;
                body.x[index + 1] += nx * deltaLambda *
                    body.inverseMass[index + 1] * w1Factor;
                body.y[index + 1] += ny * deltaLambda *
                    body.inverseMass[index + 1] * w1Factor;
                body.z[index + 1] += nz * deltaLambda *
                    body.inverseMass[index + 1] * w1Factor;
                continue;
            }
            const correction0X = nx * deltaLambda * body.inverseMass[index] * w0Factor;
            const correction0Y = ny * deltaLambda * body.inverseMass[index] * w0Factor;
            const correction0Z = nz * deltaLambda * body.inverseMass[index] * w0Factor;
            const correction1X = nx * deltaLambda * body.inverseMass[index + 1] * w1Factor;
            const correction1Y = ny * deltaLambda * body.inverseMass[index + 1] * w1Factor;
            const correction1Z = nz * deltaLambda * body.inverseMass[index + 1] * w1Factor;
            body.x[index] += correction0X;
            body.y[index] += correction0Y;
            body.z[index] += correction0Z;
            body.x[index + 1] += correction1X;
            body.y[index + 1] += correction1Y;
            body.z[index + 1] += correction1Z;
            body.wallProjectionX[index] += correction0X;
            body.wallProjectionY[index] += correction0Y;
            body.wallProjectionZ[index] += correction0Z;
            body.wallProjectionX[index + 1] += correction1X;
            body.wallProjectionY[index + 1] += correction1Y;
            body.wallProjectionZ[index + 1] += correction1Z;
        }
    }

    #updateVelocityAndFriction(body) {
        if (body.sleeping) return;
        if (copyKirchhoffSplitVelocity(body)) {
            body.lastMaximumReconstructedSpeed = 0;
            let angular = 0;
            for (let i = body.activeStart; i <= body.activeEnd; i++) {
                body.lastMaximumReconstructedSpeed = Math.max(body.lastMaximumReconstructedSpeed,
                    Math.hypot(body.velocityX[i], body.velocityY[i], body.velocityZ[i]));
                if (i < body.activeEnd) angular = Math.max(angular,
                    Math.hypot(body.angularVelocityX[i], body.angularVelocityY[i], body.angularVelocityZ[i]));
            }
            if (body.lastMaximumReconstructedSpeed < body.sleepVelocity && angular < body.sleepAngularVelocity &&
                body.settledMaxPenetration < body.sleepPenetration && this.lastCoupledClosureConverged) body.sleepCounter++;
            else body.sleepCounter = 0;
            return;
        }
        const inverseDt = 1 / this.fixedDt;
        let maxSpeed = 0;
        let maxAngularSpeed = 0;
        const frictionStart = Math.max(
            0,
            body.activeStart,
            body.collisionStartSegment
        );
        const frictionEnd = Math.min(
            body.segmentCount - 1,
            body.activeEnd - 1,
            body.collisionEndSegment
        );
        for (let segment = frictionStart; segment <= frictionEnd; segment++) {
            const currentNormalCorrection = body.wallActive[segment]
                ? Math.min(
                    body.wallFrictionLambda[segment],
                    Math.max(0.25, body.restLength[segment] * 0.5)
                )
                : 0;
            const retainedLoad = body.wallFrictionLoad[segment];
            const blend = currentNormalCorrection > retainedLoad ? 0.65 : 0.22;
            const nextLoad = retainedLoad +
                (currentNormalCorrection - retainedLoad) * blend;
            body.wallFrictionLoad[segment] = nextLoad > 1e-5 ? nextLoad : 0;
        }
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            let dx = body.x[index] - body.previousX[index];
            let dy = body.y[index] - body.previousY[index];
            let dz = body.z[index] - body.previousZ[index];
            const rawSpeed = magnitude3(dx, dy, dz) * inverseDt;
            body.lastMaximumRawSpeed = Math.max(
                body.lastMaximumRawSpeed,
                rawSpeed
            );
            const wallProjectionX = body.wallProjectionX[index];
            const wallProjectionY = body.wallProjectionY[index];
            const wallProjectionZ = body.wallProjectionZ[index];
            const wallProjectionSpeed = magnitude3(
                wallProjectionX,
                wallProjectionY,
                wallProjectionZ
            ) * inverseDt;
            if (wallProjectionSpeed > body.lastMaximumWallProjectionSpeed) {
                body.lastMaximumWallProjectionSpeed = wallProjectionSpeed;
                body.lastMaximumWallProjectionNode = index;
            }
            const rejectedWallProjection =
                1 - body.wallProjectionVelocityRetention;
            if (
                rejectedWallProjection > 0 &&
                wallProjectionSpeed > EPSILON
            ) {
                const wallProjectionLength =
                    wallProjectionSpeed / inverseDt;
                const wallDirectionX =
                    wallProjectionX / wallProjectionLength;
                const wallDirectionY =
                    wallProjectionY / wallProjectionLength;
                const wallDirectionZ =
                    wallProjectionZ / wallProjectionLength;
                const alignedMotion = Math.max(
                    0,
                    dx * wallDirectionX +
                        dy * wallDirectionY +
                        dz * wallDirectionZ
                );
                // Iterated wall and structural constraints can apply large,
                // opposing positional corrections during the same solve. A
                // raw subtraction of the accumulated wall vector would then
                // invert their small net displacement and manufacture an even
                // larger rebound. Reject only the part of the *observed* net
                // motion aligned with contact, bounded by both vectors. This
                // is dissipative by construction: it can shorten velocity but
                // can never reverse or increase it.
                const rejectedMotion = Math.min(
                    alignedMotion,
                    wallProjectionLength * rejectedWallProjection
                );
                dx -= wallDirectionX * rejectedMotion;
                dy -= wallDirectionY * rejectedMotion;
                dz -= wallDirectionZ * rejectedMotion;
                body.lastMaximumRejectedWallProjectionSpeed = Math.max(
                    body.lastMaximumRejectedWallProjectionSpeed,
                    rejectedMotion * inverseDt
                );
            }
            const toolProjectionX = body.toolProjectionX[index];
            const toolProjectionY = body.toolProjectionY[index];
            const toolProjectionZ = body.toolProjectionZ[index];
            const toolProjectionLength = magnitude3(
                toolProjectionX,
                toolProjectionY,
                toolProjectionZ
            );
            const toolProjectionSpeed = toolProjectionLength * inverseDt;
            body.lastMaximumToolProjectionSpeed = Math.max(
                body.lastMaximumToolProjectionSpeed,
                toolProjectionSpeed
            );
            const rejectedToolProjection =
                1 - body.toolProjectionVelocityRetention;
            if (
                rejectedToolProjection > 0 &&
                toolProjectionLength > EPSILON
            ) {
                const toolDirectionX = toolProjectionX / toolProjectionLength;
                const toolDirectionY = toolProjectionY / toolProjectionLength;
                const toolDirectionZ = toolProjectionZ / toolProjectionLength;
                const alignedMotion = Math.max(
                    0,
                    dx * toolDirectionX +
                        dy * toolDirectionY +
                        dz * toolDirectionZ
                );
                // Lumen and tool-tool constraints are non-penetrating
                // bearings, not springs. Remove only the observed motion in
                // the direction created by their projection. This preserves
                // axial sliding and constitutive recovery, and cannot reverse
                // or amplify the physical velocity already present.
                const rejectedMotion = Math.min(
                    alignedMotion,
                    toolProjectionLength * rejectedToolProjection
                );
                dx -= toolDirectionX * rejectedMotion;
                dy -= toolDirectionY * rejectedMotion;
                dz -= toolDirectionZ * rejectedMotion;
                body.lastMaximumRejectedToolProjectionSpeed = Math.max(
                    body.lastMaximumRejectedToolProjectionSpeed,
                    rejectedMotion * inverseDt
                );
            }
            let staticFrictionBudget = 0;
            let kineticFrictionBudget = 0;
            let nx = 0;
            let ny = 0;
            let nz = 0;
            if (index > 0 && body.wallActive[index - 1]) {
                const segment = index - 1;
                const currentLoad = body.wallFrictionLambda[segment];
                const frictionLoad = body.wallFrictionUsesCurrentLoad
                    ? currentLoad
                    : body.wallFrictionUsesSmoothedLoad
                        ? body.wallFrictionLoad[segment]
                        : body.wallLambda[segment];
                staticFrictionBudget += frictionLoad * body.wallStaticFriction;
                kineticFrictionBudget += frictionLoad * body.wallKineticFriction;
                nx += body.wallNormalX[index - 1];
                ny += body.wallNormalY[index - 1];
                nz += body.wallNormalZ[index - 1];
            }
            if (index < body.segmentCount && body.wallActive[index]) {
                const currentLoad = body.wallFrictionLambda[index];
                const frictionLoad = body.wallFrictionUsesCurrentLoad
                    ? currentLoad
                    : body.wallFrictionUsesSmoothedLoad
                        ? body.wallFrictionLoad[index]
                        : body.wallLambda[index];
                staticFrictionBudget += frictionLoad * body.wallStaticFriction;
                kineticFrictionBudget += frictionLoad * body.wallKineticFriction;
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
                if (!body._wallWitnessFrictionSolved && staticFrictionBudget > 0 && tangentLength > EPSILON) {
                    // Coulomb stick/slip: static friction may cancel the full
                    // tangential trial motion. Once that cone is exceeded,
                    // only the lower kinetic budget opposes sliding.
                    const frictionBudget = tangentLength <= staticFrictionBudget
                        ? tangentLength
                        : kineticFrictionBudget;
                    const reduction = Math.min(tangentLength, frictionBudget) /
                        tangentLength;
                    dx -= tangentX * reduction;
                    dy -= tangentY * reduction;
                    dz -= tangentZ * reduction;
                }
            }
            const projectionVelocityRetention =
                index >= body.distalProjectionVelocityRetentionStartNode
                    ? body.distalProjectionVelocityRetention
                    : body.projectionVelocityRetention;
            body.velocityX[index] = dx * inverseDt * projectionVelocityRetention;
            body.velocityY[index] = dy * inverseDt * projectionVelocityRetention;
            body.velocityZ[index] = dz * inverseDt * projectionVelocityRetention;
            const reconstructedSpeed = magnitude3(
                body.velocityX[index],
                body.velocityY[index],
                body.velocityZ[index]
            );
            body.lastMaximumReconstructedSpeed = Math.max(
                body.lastMaximumReconstructedSpeed,
                reconstructedSpeed
            );
            maxSpeed = Math.max(maxSpeed, reconstructedSpeed);
        }
        {
            const scratch = body.kirchhoffScratch.velocity;
            const segmentStart = Math.max(0, body.activeStart);
            const segmentEnd = Math.min(body.segmentCount, body.activeEnd);
            for (let segment = segmentStart; segment < segmentEnd; segment++) {
                scratch.current.x = body.orientationX[segment];
                scratch.current.y = body.orientationY[segment];
                scratch.current.z = body.orientationZ[segment];
                scratch.current.w = body.orientationW[segment];
                scratch.previous.x = body.previousOrientationX[segment];
                scratch.previous.y = body.previousOrientationY[segment];
                scratch.previous.z = body.previousOrientationZ[segment];
                scratch.previous.w = body.previousOrientationW[segment];
                conjugateQuaternion(
                    scratch.previous,
                    scratch.previousInverse
                );
                multiplyQuaternions(
                    scratch.current,
                    scratch.previousInverse,
                    scratch.relative
                );
                const delta = quaternionLog(scratch.relative, scratch.delta);
                const projectionVelocityRetention =
                    segment >= body.distalProjectionVelocityRetentionStartNode
                        ? body.distalProjectionVelocityRetention
                        : body.projectionVelocityRetention;
                let angularX = delta.x * inverseDt * projectionVelocityRetention;
                let angularY = delta.y * inverseDt * projectionVelocityRetention;
                let angularZ = delta.z * inverseDt * projectionVelocityRetention;
                const angularSpeed = magnitude3(angularX, angularY, angularZ);
                if (
                    Number.isFinite(body.maxAngularSpeed) &&
                    body.maxAngularSpeed > 0 &&
                    angularSpeed > body.maxAngularSpeed
                ) {
                    const scale = body.maxAngularSpeed / angularSpeed;
                    angularX *= scale;
                    angularY *= scale;
                    angularZ *= scale;
                }
                body.angularVelocityX[segment] = angularX;
                body.angularVelocityY[segment] = angularY;
                body.angularVelocityZ[segment] = angularZ;
                maxAngularSpeed = Math.max(
                    maxAngularSpeed,
                    magnitude3(angularX, angularY, angularZ)
                );
            }
        }
        if (
            maxSpeed < body.sleepVelocity &&
            maxAngularSpeed < body.sleepAngularVelocity &&
            body.settledMaxPenetration < body.sleepPenetration &&
            (!['joint', 'joint-components'].includes(this.lastCoupledSolver) || this.lastCoupledClosureConverged)
        ) body.sleepCounter++;
        else body.sleepCounter = 0;
        if (body.sleepCounter >= body.sleepFrames && this.lastCoupledSolver !== 'joint') this.#sleepBody(body);
    }

    #sleepBody(body) {
        body.sleeping = true;
        body.velocityX.fill(0); body.velocityY.fill(0); body.velocityZ.fill(0);
        body.angularVelocityX.fill(0); body.angularVelocityY.fill(0); body.angularVelocityZ.fill(0);
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

    #limitFrameDisplacement(body) {
        const maximum = body.maxFrameDisplacement;
        if (!Number.isFinite(maximum) || maximum <= 0) return;
        const start = clamp(
            body.frameDisplacementStartNode,
            body.activeStart,
            body.activeEnd
        );
        for (let index = start; index <= body.activeEnd; index++) {
            const dx = body.x[index] - body.previousX[index];
            const dy = body.y[index] - body.previousY[index];
            const dz = body.z[index] - body.previousZ[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance <= maximum || distance < EPSILON) continue;
            const scale = maximum / distance;
            body.x[index] = body.previousX[index] + dx * scale;
            body.y[index] = body.previousY[index] + dy * scale;
            body.z[index] = body.previousZ[index] + dz * scale;
        }
    }

    #stabilizeContainmentVelocity(constraint) {
        {
            this.#stabilizeKirchhoffContainmentVelocity(constraint);
            return;
        }

    }

    #stabilizeKirchhoffContainmentVelocity(constraint) {
        if (!constraint.enabled) return;
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        this.#dampKirchhoffContainedRadialVelocity(
            constraint,
            inner,
            outer
        );

        for (const record of constraint.kirchhoffContacts) {
            const contact = record?.manifoldContact;
            if (!contact) continue;
            const innerSegment = contact.innerSegmentIndex;
            const outerSegment = contact.outerSegmentIndex;
            if (
                innerSegment < inner.activeStart ||
                innerSegment >= inner.activeEnd ||
                outerSegment < outer.activeStart ||
                outerSegment >= outer.activeEnd
            ) continue;
            const innerNodes = record._innerNodeIndices;
            const outerNodes = record._outerNodeIndices;
            const innerWeights = record._innerNodeWeights;
            const outerWeights = record._outerNodeWeights;
            const innerCount = innerNodes ? record._innerNodeCount : 2;
            const outerCount = outerNodes ? record._outerNodeCount : 2;
            let relativeX = 0;
            let relativeY = 0;
            let relativeZ = 0;
            let denominator = 0;
            for (let index = 0; index < innerCount; index++) {
                const node = innerNodes?.[index] ?? innerSegment + index;
                const weight = innerWeights?.[index] ?? record.innerWeights[index];
                relativeX += inner.velocityX[node] * weight;
                relativeY += inner.velocityY[node] * weight;
                relativeZ += inner.velocityZ[node] * weight;
                denominator += inner.inverseMass[node] * weight * weight;
            }
            for (let index = 0; index < outerCount; index++) {
                const node = outerNodes?.[index] ?? outerSegment + index;
                const weight = outerWeights?.[index] ?? record.outerWeights[index];
                relativeX -= outer.velocityX[node] * weight;
                relativeY -= outer.velocityY[node] * weight;
                relativeZ -= outer.velocityZ[node] * weight;
                denominator += outer.inverseMass[node] * weight * weight;
            }
            const outwardVelocity =
                relativeX * record.normal[0] +
                relativeY * record.normal[1] +
                relativeZ * record.normal[2];
            if (outwardVelocity <= EPSILON) continue;
            if (denominator <= EPSILON) continue;
            const impulse = outwardVelocity / denominator;
            const impulseX = record.normal[0] * impulse;
            const impulseY = record.normal[1] * impulse;
            const impulseZ = record.normal[2] * impulse;
            for (let index = 0; index < innerCount; index++) {
                const node = innerNodes?.[index] ?? innerSegment + index;
                const weight = innerWeights?.[index] ?? record.innerWeights[index];
                const massWeight = inner.inverseMass[node] * weight;
                inner.velocityX[node] -= impulseX * massWeight;
                inner.velocityY[node] -= impulseY * massWeight;
                inner.velocityZ[node] -= impulseZ * massWeight;
            }
            for (let index = 0; index < outerCount; index++) {
                const node = outerNodes?.[index] ?? outerSegment + index;
                const weight = outerWeights?.[index] ?? record.outerWeights[index];
                const massWeight = outer.inverseMass[node] * weight;
                outer.velocityX[node] += impulseX * massWeight;
                outer.velocityY[node] += impulseY * massWeight;
                outer.velocityZ[node] += impulseZ * massWeight;
            }
        }
        this.#dampKirchhoffCoupledBendingRates(constraint, inner, outer);
    }

    #dampKirchhoffCoupledBendingRates(constraint, inner, outer) {
        // Active feed owns the material transport velocity and must not be
        // spatially filtered. Engage this high-frequency equilibrium damper
        // only as projection reconstruction becomes quasi-static after the
        // operator releases the catheter control.
        const quasiStaticBlend = 1 - clamp(
            outer.projectionVelocityRetention,
            0,
            1
        );
        const damping = constraint.coupledBendingRateDamping *
            quasiStaticBlend;
        const passes = constraint.coupledBendingRatePasses;
        if (damping <= EPSILON || passes <= 0) return;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            inner.activeEnd
        );
        const containedInnerEnd = clamp(
            constraint.endNode,
            innerStart,
            inner.activeEnd
        );
        if (containedInnerEnd - innerStart < 2) return;

        let outerStart = outer.activeEnd;
        let outerEnd = outer.activeStart;
        for (
            let segment = innerStart;
            segment <= containedInnerEnd;
            segment++
        ) {
            const mapped = constraint.closestSegment[segment];
            if (
                mapped < outer.activeStart ||
                mapped >= outer.activeEnd
            ) continue;
            outerStart = Math.min(outerStart, mapped);
            outerEnd = Math.max(outerEnd, mapped + 1);
        }
        this.#smoothCoupledVelocityRange(
            inner,
            innerStart,
            // Absorb the short wave over a finite transition beyond the
            // aperture. Extending the filter over the complete unsupported
            // wire would overdamp its physical long-wave recovery.
            Math.min(inner.activeEnd, containedInnerEnd + 8),
            damping,
            passes
        );
        if (outerEnd - outerStart >= 2) {
            this.#smoothCoupledVelocityRange(
                outer,
                outerStart,
                outerEnd,
                damping,
                passes
            );
        }
    }

    #smoothCoupledVelocityRange(body, start, end, damping, passes) {
        const rangeStart = clamp(start, body.activeStart, body.activeEnd);
        const rangeEnd = clamp(end, rangeStart, body.activeEnd);
        if (rangeEnd - rangeStart < 2) return;
        for (let pass = 0; pass < passes; pass++) {
            for (let node = rangeStart; node <= rangeEnd; node++) {
                body.postPassStartX[node] = body.velocityX[node];
                body.postPassStartY[node] = body.velocityY[node];
                body.postPassStartZ[node] = body.velocityZ[node];
            }
            for (let node = rangeStart + 1; node < rangeEnd; node++) {
                // This discrete bending-rate term is zero for rigid
                // translation and for a linear axial velocity field. It
                // therefore removes only unresolved short-wavelength motion,
                // not operator feed or the pair's common motion.
                body.velocityX[node] += (
                    (
                        body.postPassStartX[node - 1] +
                        body.postPassStartX[node + 1]
                    ) * 0.5 - body.postPassStartX[node]
                ) * damping;
                body.velocityY[node] += (
                    (
                        body.postPassStartY[node - 1] +
                        body.postPassStartY[node + 1]
                    ) * 0.5 - body.postPassStartY[node]
                ) * damping;
                body.velocityZ[node] += (
                    (
                        body.postPassStartZ[node - 1] +
                        body.postPassStartZ[node + 1]
                    ) * 0.5 - body.postPassStartZ[node]
                ) * damping;
            }
        }
    }

    #dampKirchhoffContainedRadialVelocity(constraint, inner, outer) {
        const damping = constraint.radialVelocityDamping;
        if (damping <= EPSILON) return;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            Math.max(inner.activeStart, inner.activeEnd - 1)
        );
        const innerEnd = Math.min(
            inner.segmentCount - 1,
            inner.activeEnd - 1,
            Math.max(innerStart, constraint.endNode)
        );
        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            Math.max(outer.activeStart, outer.activeEnd - 1)
        );
        const outerEnd = Math.min(
            outer.segmentCount - 1,
            outer.activeEnd - 1
        );
        if (innerEnd < innerStart || outerEnd < outerStart) return;

        let innerArcStart = Math.max(0, constraint.innerArcOffset);
        for (
            let innerSegment = innerStart;
            innerSegment <= innerEnd;
            innerSegment++
        ) {
            const innerRestLength = Math.max(
                EPSILON,
                inner.restLength[innerSegment]
            );
            const containedFraction = Number.isFinite(
                constraint.containedLength
            )
                ? clamp(
                    (
                        constraint.containedLength - innerArcStart
                    ) / innerRestLength,
                    0,
                    1
                )
                : 1;
            innerArcStart += innerRestLength;
            if (containedFraction <= EPSILON) continue;

            const outerSegment =
                constraint.closestSegment[innerSegment];
            if (
                outerSegment < outerStart ||
                outerSegment > outerEnd
            ) continue;
            const axisX = outer.x[outerSegment + 1] -
                outer.x[outerSegment];
            const axisY = outer.y[outerSegment + 1] -
                outer.y[outerSegment];
            const axisZ = outer.z[outerSegment + 1] -
                outer.z[outerSegment];
            const axisLength = magnitude3(axisX, axisY, axisZ);
            if (axisLength <= EPSILON) continue;
            const tangentX = axisX / axisLength;
            const tangentY = axisY / axisLength;
            const tangentZ = axisZ / axisLength;

            // Dampen the proximal material node of every mapped segment. A
            // midpoint-only operator misses the alternating endpoint mode
            // (equal and opposite node velocities have a stationary
            // midpoint), which is precisely the numerical wave seen in a
            // tightly coupled wire/catheter pair.
            const innerNode = innerSegment;
            const radialPositionX = inner.x[innerNode] -
                outer.x[outerSegment];
            const radialPositionY = inner.y[innerNode] -
                outer.y[outerSegment];
            const radialPositionZ = inner.z[innerNode] -
                outer.z[outerSegment];
            const outerT = clamp(
                (
                    radialPositionX * tangentX +
                    radialPositionY * tangentY +
                    radialPositionZ * tangentZ
                ) / axisLength,
                0,
                1
            );
            const outerWeight0 = 1 - outerT;
            const outerWeight1 = outerT;
            const relativeX =
                inner.velocityX[innerNode] -
                outer.velocityX[outerSegment] * outerWeight0 -
                outer.velocityX[outerSegment + 1] * outerWeight1;
            const relativeY =
                inner.velocityY[innerNode] -
                outer.velocityY[outerSegment] * outerWeight0 -
                outer.velocityY[outerSegment + 1] * outerWeight1;
            const relativeZ =
                inner.velocityZ[innerNode] -
                outer.velocityZ[outerSegment] * outerWeight0 -
                outer.velocityZ[outerSegment + 1] * outerWeight1;
            const axialVelocity =
                relativeX * tangentX +
                relativeY * tangentY +
                relativeZ * tangentZ;
            const radialX = relativeX - tangentX * axialVelocity;
            const radialY = relativeY - tangentY * axialVelocity;
            const radialZ = relativeZ - tangentZ * axialVelocity;
            const radialSpeedSquared =
                radialX * radialX +
                radialY * radialY +
                radialZ * radialZ;
            if (radialSpeedSquared <= EPSILON * EPSILON) continue;

            const innerMassWeight = inner.inverseMass[innerNode];
            const outerMassWeight0 =
                outer.inverseMass[outerSegment] * outerWeight0;
            const outerMassWeight1 =
                outer.inverseMass[outerSegment + 1] * outerWeight1;
            const denominator =
                innerMassWeight +
                outerMassWeight0 * outerWeight0 +
                outerMassWeight1 * outerWeight1;
            if (denominator <= EPSILON) continue;

            // Smoothly engage the last partially captured segment so moving
            // the portal across a node cannot introduce a damping impulse.
            const fraction = containedFraction * containedFraction *
                (3 - 2 * containedFraction);
            const impulseScale = damping * fraction / denominator;
            const impulseX = radialX * impulseScale;
            const impulseY = radialY * impulseScale;
            const impulseZ = radialZ * impulseScale;
            inner.velocityX[innerNode] -= impulseX * innerMassWeight;
            inner.velocityY[innerNode] -= impulseY * innerMassWeight;
            inner.velocityZ[innerNode] -= impulseZ * innerMassWeight;
            outer.velocityX[outerSegment] +=
                impulseX * outerMassWeight0;
            outer.velocityY[outerSegment] +=
                impulseY * outerMassWeight0;
            outer.velocityZ[outerSegment] +=
                impulseZ * outerMassWeight0;
            outer.velocityX[outerSegment + 1] +=
                impulseX * outerMassWeight1;
            outer.velocityY[outerSegment + 1] +=
                impulseY * outerMassWeight1;
            outer.velocityZ[outerSegment + 1] +=
                impulseZ * outerMassWeight1;
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
        let maxBendNode = -1;
        let maxBendLimitDegrees = 0;
        let maxSpeed = 0;
        let kineticEnergy = 0;
        let activeWallContacts = 0;
        let currentNormalLoad = 0;
        let retainedFrictionLoad = 0;
        let finite = true;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            const speed = magnitude3(
                body.velocityX[index],
                body.velocityY[index],
                body.velocityZ[index]
            );
            maxSpeed = Math.max(maxSpeed, speed);
            kineticEnergy += 0.5 * body.mass * speed * speed;
            finite = finite && Number.isFinite(speed);
        }
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
                const bendAngle = Math.acos(clamp(
                    (ax * bx + ay * by + az * bz) / denominator,
                    -1,
                    1
                ));
                if (bendAngle > maxBendAngle) {
                    maxBendAngle = bendAngle;
                    maxBendNode = index;
                    maxBendLimitDegrees = body.maxBendAngleByNode[index];
                }
            }
            if (body.wallActive[index]) {
                activeWallContacts++;
                currentNormalLoad += body.wallFrictionLambda[index];
                retainedFrictionLoad += body.wallFrictionLoad[index];
            }
        }
        return {
            id: body.id,
            sleeping: body.sleeping,
            constitutiveSolver: body.constitutiveSolver,
            directFactorizations: body.kirchhoffScratch.direct?.factorizationCount ?? 0,
            directFactorReuses: body.kirchhoffScratch.direct?.factorReuseCount ?? 0,

            finite,
            maxLengthError,
            maxBendAngleDegrees: maxBendAngle * 180 / Math.PI,
            maxBendNode,
            maxBendLimitDegrees,
            maxSpeed,
            maximumRawSpeed: body.lastMaximumRawSpeed,
            maximumWallProjectionSpeed:
                body.lastMaximumWallProjectionSpeed,
            maximumWallProjectionNode:
                body.lastMaximumWallProjectionNode,
            maximumRejectedWallProjectionSpeed:
                body.lastMaximumRejectedWallProjectionSpeed,
            wallProjectionVelocityRetention:
                body.wallProjectionVelocityRetention,
            maximumToolProjectionSpeed:
                body.lastMaximumToolProjectionSpeed,
            maximumRejectedToolProjectionSpeed:
                body.lastMaximumRejectedToolProjectionSpeed,
            toolProjectionVelocityRetention:
                body.toolProjectionVelocityRetention,
            maximumReconstructedSpeed:
                body.lastMaximumReconstructedSpeed,
            relaxationPasses: body.relaxationPasses,
            lastRelaxationPasses: body.lastRelaxationPasses,
            kineticEnergy,
            activeWallContacts,
            currentNormalLoad,
            retainedFrictionLoad,

        };
    }
}
