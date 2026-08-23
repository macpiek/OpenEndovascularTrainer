import { createContactResult } from './collision/vesselContactField.js';
import {
    GUIDEWIRE_RADIUS_MM,
    INTRODUCER_SHEATH_INNER_DIAMETER_MM,
    INTRODUCER_SHEATH_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_INNER_DIAMETER_MM,
    PIGTAIL_CATHETER_INNER_RADIUS_MM,
    PIGTAIL_CATHETER_RADIUS_MM
} from '../toolDimensions.js';
import {
    conjugateQuaternion,
    createBishopFrame,
    inverseRotateVectorByQuaternion,
    multiplyQuaternions,
    normalizeQuaternion,
    quaternionExp,
    quaternionLog,
    solveAdaptationXPBDArraySweep,
    solveBendTwistXPBDBlockArraySweep,
    solveBendTwistXPBD,
    transportBishopFrame
} from './discreteKirchhoffRod.js';
import { KirchhoffContactManifold } from './kirchhoffContactManifold.js';

const EPSILON = 1e-8;
const TRIG_SERIES_ANGLE_SQUARED = 0.0625;
const DEFAULT_FIXED_DT = 1 / 120;
const CONTACT_SIGNED_GAP = 1;
const CONTACT_PENETRATION = 3;
const CONTACT_BRANCH_ID = 4;
const CONTACT_SEGMENT_T = 5;
const MAX_WALL_CORRECTION_PASSES = 16;
const CHORD_TO_ANGULAR_BEND_COMPLIANCE_SCALE = 100;
const WALL_SETTLING_CLEARANCE = 0.01;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function magnitude3(x, y, z) {
    return Math.sqrt(x * x + y * y + z * z);
}

function closestRodSegmentParameters(
    firstBody,
    firstSegment,
    secondBody,
    secondSegment,
    out
) {
    const firstStartX = firstBody.x[firstSegment];
    const firstStartY = firstBody.y[firstSegment];
    const firstStartZ = firstBody.z[firstSegment];
    const firstDirectionX = firstBody.x[firstSegment + 1] - firstStartX;
    const firstDirectionY = firstBody.y[firstSegment + 1] - firstStartY;
    const firstDirectionZ = firstBody.z[firstSegment + 1] - firstStartZ;
    const secondStartX = secondBody.x[secondSegment];
    const secondStartY = secondBody.y[secondSegment];
    const secondStartZ = secondBody.z[secondSegment];
    const secondDirectionX = secondBody.x[secondSegment + 1] - secondStartX;
    const secondDirectionY = secondBody.y[secondSegment + 1] - secondStartY;
    const secondDirectionZ = secondBody.z[secondSegment + 1] - secondStartZ;
    const offsetX = firstStartX - secondStartX;
    const offsetY = firstStartY - secondStartY;
    const offsetZ = firstStartZ - secondStartZ;
    const firstLengthSquared = firstDirectionX * firstDirectionX +
        firstDirectionY * firstDirectionY +
        firstDirectionZ * firstDirectionZ;
    const secondLengthSquared = secondDirectionX * secondDirectionX +
        secondDirectionY * secondDirectionY +
        secondDirectionZ * secondDirectionZ;
    const secondProjection = secondDirectionX * offsetX +
        secondDirectionY * offsetY + secondDirectionZ * offsetZ;
    let firstT;
    let secondT;
    if (firstLengthSquared <= EPSILON && secondLengthSquared <= EPSILON) {
        firstT = 0;
        secondT = 0;
    } else if (firstLengthSquared <= EPSILON) {
        firstT = 0;
        secondT = clamp(secondProjection / secondLengthSquared, 0, 1);
    } else {
        const firstProjection = firstDirectionX * offsetX +
            firstDirectionY * offsetY + firstDirectionZ * offsetZ;
        if (secondLengthSquared <= EPSILON) {
            secondT = 0;
            firstT = clamp(-firstProjection / firstLengthSquared, 0, 1);
        } else {
            const directionsDot = firstDirectionX * secondDirectionX +
                firstDirectionY * secondDirectionY +
                firstDirectionZ * secondDirectionZ;
            const denominator = firstLengthSquared * secondLengthSquared -
                directionsDot * directionsDot;
            firstT = denominator > EPSILON
                ? clamp(
                    (directionsDot * secondProjection -
                        firstProjection * secondLengthSquared) / denominator,
                    0,
                    1
                )
                : 0;
            secondT = (
                directionsDot * firstT + secondProjection
            ) / secondLengthSquared;
            if (secondT < 0) {
                secondT = 0;
                firstT = clamp(-firstProjection / firstLengthSquared, 0, 1);
            } else if (secondT > 1) {
                secondT = 1;
                firstT = clamp(
                    (directionsDot - firstProjection) / firstLengthSquared,
                    0,
                    1
                );
            }
        }
    }
    const deltaX = firstStartX + firstDirectionX * firstT -
        secondStartX - secondDirectionX * secondT;
    const deltaY = firstStartY + firstDirectionY * firstT -
        secondStartY - secondDirectionY * secondT;
    const deltaZ = firstStartZ + firstDirectionZ * firstT -
        secondStartZ - secondDirectionZ * secondT;
    out.firstT = firstT;
    out.secondT = secondT;
    // Candidate selection only needs an ordering. Comparing squared distances
    // avoids one square root for every segment in the temporal-coherence
    // window; the actual Euclidean distance is evaluated once for the winner.
    out.distanceSquared = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
    return out;
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
        maxBendAngle: 60,
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
        bendDamping: 0.3,
        // Operator feed is bounded independently by GuidewireSolver. A limit
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
        shapeCompliance: 1e-4,
        // Elastic bending stiffness is responsible for straightening the
        // shaft. This inequality is only an anti-fold safety guard. Keeping
        // it above an ordinary aortic turn avoids an over-constrained
        // length/fold cycle while still rejecting a one-node kink.
        maxBendAngle: 24,
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
        linearDamping: 0.9,
        bendDamping: 0.68,
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
    constructor(id, count, segmentLength, profile = {}) {
        if (!Number.isInteger(count) || count < 2) throw new RangeError('A rod requires at least two nodes');
        this.id = id;
        this.rodModel = profile.rodModel === 'kirchhoff'
            ? 'kirchhoff'
            : 'legacy';
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
        this.sweptContactPreserveTangentialMotion =
            profile.sweptContactPreserveTangentialMotion === true;
        this.lumenFriction = profile.lumenFriction ?? 0.04;
        this.linearDamping = profile.linearDamping ?? 0.98;
        this.bendDamping = clamp(profile.bendDamping ?? 0, 0, 1);
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
        this.postStabilizeShape = false;
        this.distalLengthTransportMaxCorrection = 1.25;
        this.postStabilizeBending = false;
        this.restTurnPolishMaxAngle = 0;
        this.restDirectionSubiterations = 1;
        this.restDirectionContactPasses = 0;
        this.restDirectionContactCorrectionScale = 1;
        this.debugConstraintPhase = null;
        this.curvatureVariationEnabled = false;
        this.curvatureVariationCompliance = profile.curvatureVariationCompliance ?? 2e-4;
        this.curvatureVariationStartNode = 0;
        this.curvatureVariationEndNode = count - 1;
        this.longStraightSpan = 0;
        this.longStraightCompliance = profile.longStraightCompliance ?? 5e-5;
        this.longStraightStartNode = 0;
        this.longStraightEndNode = count - 1;
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

        this.x = new Float32Array(count);
        this.y = new Float32Array(count);
        this.z = new Float32Array(count);
        this.previousX = new Float32Array(count);
        this.previousY = new Float32Array(count);
        this.previousZ = new Float32Array(count);
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
        this.restShapeEnabled = new Uint8Array(count);
        this.restShapeX = new Float32Array(count);
        this.restShapeY = new Float32Array(count);
        this.restShapeZ = new Float32Array(count);
        this.restShapeCompliance = new Float32Array(count);
        this.restShapeMaxCorrection = new Float32Array(count);
        this.restShapeMaxCorrection.fill(Infinity);
        this.restShapeCorrectionX = new Float32Array(count);
        this.restShapeCorrectionY = new Float32Array(count);
        this.restShapeCorrectionZ = new Float32Array(count);
        this.restShapeTranslationNeutralStart = -1;
        this.restShapeTranslationNeutralEnd = -1;
        this.shapeClosureEnabled = false;
        this.shapeClosureStart = 0;
        this.shapeClosureEnd = 0;
        this.shapeClosureDistance = 0;
        this.shapeClosureCompliance = 0;
        this.shapeClosureMaxCorrection = Infinity;
        this.shapeClosureLambda = 0;
        // Signed material-direction constraints complement the scalar bend
        // chord. They encode which way a preformed segment turns without
        // pinning any node to an absolute world-space point.
        this.restDirectionEnabled = new Uint8Array(this.segmentCount);
        this.restDirectionX = new Float32Array(this.segmentCount);
        this.restDirectionY = new Float32Array(this.segmentCount);
        this.restDirectionZ = new Float32Array(this.segmentCount);
        this.restDirectionCompliance = new Float32Array(this.segmentCount);
        this.restDirectionMaxCorrection = new Float32Array(this.segmentCount);
        this.restDirectionMaxCorrection.fill(Infinity);
        this.restDirectionDistalBias = new Float32Array(this.segmentCount);
        this.restDirectionRelative = new Uint8Array(this.segmentCount);
        this.restDirectionTurnAngle = new Float32Array(this.segmentCount);
        this.restDirectionAxisX = new Float32Array(this.segmentCount);
        this.restDirectionAxisY = new Float32Array(this.segmentCount);
        this.restDirectionAxisZ = new Float32Array(this.segmentCount);
        this.restDirectionLambdaX = new Float32Array(this.segmentCount);
        this.restDirectionLambdaY = new Float32Array(this.segmentCount);
        this.restDirectionLambdaZ = new Float32Array(this.segmentCount);
        // A joint with intrinsic curvature is governed by the signed material
        // turn below. The legacy unsigned chord constraint is disabled there,
        // so the same bend is not counted as two independent elastic energies.
        this.intrinsicBendEnabled = new Uint8Array(this.segmentCount);
        this.intrinsicCurvature = new Float32Array(this.segmentCount);
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
        this.foldCorrectionX = new Float32Array(count);
        this.foldCorrectionY = new Float32Array(count);
        this.foldCorrectionZ = new Float32Array(count);
        this.foldCorrectionWeight = new Float32Array(count);
        this.curvatureVariationLambdaX = new Float32Array(count);
        this.curvatureVariationLambdaY = new Float32Array(count);
        this.curvatureVariationLambdaZ = new Float32Array(count);
        this.longStraightLambda = new Float32Array(count);
        this.bendComplianceByNode = new Float32Array(count);
        this.maxBendAngleByNode = new Float32Array(count);
        this.controlLambda = new Float32Array(count);
        this.shapeLambda = new Float32Array(count);
        this.wallLambda = new Float32Array(this.segmentCount);
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
        this.wallT = new Float32Array(this.segmentCount);
        this.wallX = new Float32Array(this.segmentCount);
        this.wallY = new Float32Array(this.segmentCount);
        this.wallZ = new Float32Array(this.segmentCount);
        this.wallNormalX = new Float32Array(this.segmentCount);
        this.wallNormalY = new Float32Array(this.segmentCount);
        this.wallNormalZ = new Float32Array(this.segmentCount);
        this.wallBranchId = new Int32Array(this.segmentCount);
        this.wallFaceIndex = new Int32Array(this.segmentCount);
        this.wallGap = new Float32Array(this.segmentCount);
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
        this.lastMaximumRawSpeed = 0;
        this.lastMaximumWallProjectionSpeed = 0;
        this.lastMaximumWallProjectionNode = -1;
        this.lastMaximumRejectedWallProjectionSpeed = 0;
        this.lastMaximumReconstructedSpeed = 0;
        this.postPassStartX = new Float64Array(count);
        this.postPassStartY = new Float64Array(count);
        this.postPassStartZ = new Float64Array(count);
        this.wallBranchId.fill(-1);
        this.wallFaceIndex.fill(-1);
        this.wallGap.fill(Infinity);
        this.nodeRadius.fill(this.radius);
        this.inverseMass.fill(1 / Math.max(EPSILON, this.mass));
        this.restLength.fill(segmentLength);
        this.bendComplianceByNode.fill(this.bendCompliance);
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

    setRestShapeTarget(
        index,
        x,
        y,
        z,
        compliance = this.shapeCompliance,
        maxCorrection = Infinity
    ) {
        const nextCompliance = Math.max(0, compliance);
        const nextMaxCorrection = Math.max(0, maxCorrection);
        const changed = !this.restShapeEnabled[index] ||
            Math.abs(this.restShapeX[index] - x) > 0.01 ||
            Math.abs(this.restShapeY[index] - y) > 0.01 ||
            Math.abs(this.restShapeZ[index] - z) > 0.01 ||
            Math.abs(this.restShapeCompliance[index] - nextCompliance) > 1e-10 ||
            this.restShapeMaxCorrection[index] !== nextMaxCorrection;
        this.restShapeEnabled[index] = 1;
        this.restShapeX[index] = x;
        this.restShapeY[index] = y;
        this.restShapeZ[index] = z;
        this.restShapeCompliance[index] = nextCompliance;
        this.restShapeMaxCorrection[index] = nextMaxCorrection;
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

    setShapeClosureTarget(
        start,
        end,
        distance,
        compliance = this.shapeCompliance,
        maxCorrection = Infinity
    ) {
        const nextStart = Math.max(0, Math.min(this.count - 1, start));
        const nextEnd = Math.max(0, Math.min(this.count - 1, end));
        const nextDistance = Math.max(0, distance);
        const nextCompliance = Math.max(0, compliance);
        const nextMaxCorrection = Math.max(0, maxCorrection);
        const changed = !this.shapeClosureEnabled ||
            this.shapeClosureStart !== nextStart ||
            this.shapeClosureEnd !== nextEnd ||
            Math.abs(this.shapeClosureDistance - nextDistance) > 1e-4 ||
            Math.abs(this.shapeClosureCompliance - nextCompliance) > 1e-10 ||
            this.shapeClosureMaxCorrection !== nextMaxCorrection;
        this.shapeClosureEnabled = nextStart !== nextEnd;
        this.shapeClosureStart = nextStart;
        this.shapeClosureEnd = nextEnd;
        this.shapeClosureDistance = nextDistance;
        this.shapeClosureCompliance = nextCompliance;
        this.shapeClosureMaxCorrection = nextMaxCorrection;
        if (changed) {
            this.shapeClosureLambda = 0;
            this.wake();
        }
        return this;
    }

    clearShapeClosureTarget() {
        if (this.shapeClosureEnabled) this.wake();
        this.shapeClosureEnabled = false;
        this.shapeClosureLambda = 0;
        return this;
    }

    setRestDirectionTarget(
        segment,
        x,
        y,
        z,
        compliance = this.shapeCompliance,
        maxCorrection = Infinity
    ) {
        if (segment < 0 || segment >= this.segmentCount) return this;
        const nextCompliance = Math.max(0, compliance);
        const nextMaxCorrection = Math.max(0, maxCorrection);
        const changed = !this.restDirectionEnabled[segment] ||
            Math.abs(this.restDirectionX[segment] - x) > 1e-4 ||
            Math.abs(this.restDirectionY[segment] - y) > 1e-4 ||
            Math.abs(this.restDirectionZ[segment] - z) > 1e-4 ||
            Math.abs(this.restDirectionCompliance[segment] - nextCompliance) > 1e-10 ||
            this.restDirectionMaxCorrection[segment] !== nextMaxCorrection;
        this.restDirectionEnabled[segment] = 1;
        this.restDirectionRelative[segment] = 0;
        this.restDirectionX[segment] = x;
        this.restDirectionY[segment] = y;
        this.restDirectionZ[segment] = z;
        this.restDirectionCompliance[segment] = nextCompliance;
        this.restDirectionMaxCorrection[segment] = nextMaxCorrection;
        if (changed) {
            this.restDirectionLambdaX[segment] = 0;
            this.restDirectionLambdaY[segment] = 0;
            this.restDirectionLambdaZ[segment] = 0;
            this.wake();
        }
        return this;
    }

    setRestTurnTarget(
        segment,
        angle,
        axisX,
        axisY,
        axisZ,
        compliance = this.shapeCompliance,
        maxCorrection = Infinity,
        distalBias = 0
    ) {
        if (segment <= 0 || segment >= this.segmentCount) return this;
        const axisLength = magnitude3(axisX, axisY, axisZ);
        if (axisLength < EPSILON) return this.clearRestDirectionTarget(segment);
        const nextCompliance = Math.max(0, compliance);
        const nextMaxCorrection = Math.max(0, maxCorrection);
        const nextDistalBias = clamp(distalBias, 0, 1);
        axisX /= axisLength;
        axisY /= axisLength;
        axisZ /= axisLength;
        const constitutiveChanged = !this.restDirectionEnabled[segment] ||
            !this.restDirectionRelative[segment] ||
            Math.abs(this.restDirectionTurnAngle[segment] - angle) > 1e-4 ||
            Math.abs(this.restDirectionCompliance[segment] - nextCompliance) > 1e-10 ||
            this.restDirectionMaxCorrection[segment] !== nextMaxCorrection ||
            Math.abs(this.restDirectionDistalBias[segment] - nextDistalBias) > 1e-4;
        const frameChanged =
            Math.abs(this.restDirectionAxisX[segment] - axisX) > 1e-4 ||
            Math.abs(this.restDirectionAxisY[segment] - axisY) > 1e-4 ||
            Math.abs(this.restDirectionAxisZ[segment] - axisZ) > 1e-4;
        this.restDirectionEnabled[segment] = 1;
        this.restDirectionRelative[segment] = 1;
        this.restDirectionTurnAngle[segment] = angle;
        this.restDirectionAxisX[segment] = axisX;
        this.restDirectionAxisY[segment] = axisY;
        this.restDirectionAxisZ[segment] = axisZ;
        this.restDirectionCompliance[segment] = nextCompliance;
        this.restDirectionMaxCorrection[segment] = nextMaxCorrection;
        this.restDirectionDistalBias[segment] = nextDistalBias;
        if (constitutiveChanged) {
            this.restDirectionLambdaX[segment] = 0;
            this.restDirectionLambdaY[segment] = 0;
            this.restDirectionLambdaZ[segment] = 0;
        }
        if (constitutiveChanged || frameChanged) this.wake();
        return this;
    }

    setIntrinsicCurvatureTarget(
        segment,
        angle,
        axisX,
        axisY,
        axisZ,
        compliance = this.shapeCompliance,
        maxCorrection = Infinity,
        distalBias = 0,
        voronoiLength = this.segmentLength
    ) {
        if (segment <= 0 || segment >= this.segmentCount) return this;
        this.intrinsicBendEnabled[segment] = 1;
        this.intrinsicCurvature[segment] = angle /
            Math.max(EPSILON, voronoiLength);
        return this.setRestTurnTarget(
            segment,
            angle,
            axisX,
            axisY,
            axisZ,
            compliance,
            maxCorrection,
            distalBias
        );
    }

    clearRestDirectionTarget(segment) {
        if (segment < 0 || segment >= this.segmentCount) return this;
        if (this.restDirectionEnabled[segment]) this.wake();
        this.restDirectionEnabled[segment] = 0;
        this.restDirectionRelative[segment] = 0;
        this.restDirectionDistalBias[segment] = 0;
        this.intrinsicBendEnabled[segment] = 0;
        this.intrinsicCurvature[segment] = 0;
        this.restDirectionLambdaX[segment] = 0;
        this.restDirectionLambdaY[segment] = 0;
        this.restDirectionLambdaZ[segment] = 0;
        return this;
    }

    enableKirchhoff(enabled = true, { captureRest = true } = {}) {
        const nextModel = enabled ? 'kirchhoff' : 'legacy';
        if (this.rodModel === nextModel) return this;
        this.rodModel = nextModel;
        if (enabled && captureRest) this.captureKirchhoffRestConfiguration();
        this.adaptationLambdaX.fill(0);
        this.adaptationLambdaY.fill(0);
        this.adaptationLambdaZ.fill(0);
        this.bendTwistLambda1.fill(0);
        this.bendTwistLambda2.fill(0);
        this.bendTwistLambda3.fill(0);
        this.wake();
        return this;
    }

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
        for (let index = 1; index < this.count - 1; index++) {
            this.restBendChord[index] = magnitude3(
                this.x[index + 1] - this.x[index - 1],
                this.y[index + 1] - this.y[index - 1],
                this.z[index + 1] - this.z[index - 1]
            );
        }
        this.lengthLambda.fill(0);
        this.bendLambda.fill(0);
        if (this.rodModel === 'kirchhoff') {
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
        this.captureCoupledClosureTrace = false;
        this.coupledClosureTrace = [];
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
        model = 'legacy',
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
        innerFollowsOuterCenterline = false,
        enforceDistalPortal = false,
        limitDistalCorrection = false,
        preserveStationaryInnerLength = false,
        reconcileMovingInnerStructure = false,
        portalInnerResponse = 1,
        portalOuterResponse = outerResponse,
        portalCompliance = 1e-7,
        portalTransitionLength = Math.max(innerBody.segmentLength, outerBody.segmentLength),
        portalMaxCorrection = 0.1,
        portalSmoothingLength = portalTransitionLength * 4,
        portalRetractionDistance = null,
        innerArcOffset = 0,
        containedLength = Infinity
    } = {}) {
        const containmentModel = model === 'kirchhoff'
            ? 'kirchhoff'
            : 'legacy';
        const constraint = {
            model: containmentModel,
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
            // Kirchhoff contact is always solved symmetrically from its
            // unilateral gradients. Legacy one-way final projection and its
            // command-specific response modes must never run on this path.
            finalProjection: containmentModel === 'kirchhoff'
                ? 'none'
                : finalProjection,
            outerFollowsInnerCenterline,
            innerFollowsOuterCenterline,
            enforceDistalPortal,
            limitDistalCorrection,
            preserveStationaryInnerLength,
            reconcileMovingInnerStructure,
            portalInnerResponse: clamp(portalInnerResponse, 0, 1),
            portalOuterResponse: clamp(portalOuterResponse, 0, 1),
            portalCompliance: Math.max(0, portalCompliance),
            portalTransitionLength: Math.max(EPSILON, portalTransitionLength),
            portalMaxCorrection: Math.max(EPSILON, portalMaxCorrection),
            portalSmoothingLength: Math.max(
                innerBody.segmentLength,
                portalSmoothingLength
            ),
            portalRetractionDistance,
            portalLambda: 0,
            portalDirectionLambda: 0,
            innerArcOffset,
            containedLength,
            manifold: containmentModel === 'kirchhoff'
                ? new KirchhoffContactManifold({
                    frictionCoefficient: Math.max(0, friction),
                    retentionSteps: 1
                })
                : null,
            kirchhoffOuterSegmentByInner: containmentModel === 'kirchhoff'
                ? new Int32Array(innerBody.segmentCount)
                : null,
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
        const elapsed = Number.isFinite(frameDt) ? Math.max(0, frameDt) : 0;
        this.accumulator += elapsed;
        let substeps = 0;
        while (this.accumulator + EPSILON >= this.fixedDt && substeps < this.maxSubsteps) {
            beforeSubstep?.(this.fixedDt, substeps);
            this.stepFixed();
            this.accumulator -= this.fixedDt;
            substeps++;
        }
        this.lastSubsteps = substeps;
        return substeps;
    }

    stepFixed() {
        const totalStart = now();
        this.contactCount = 0;
        this.maxPenetration = 0;
        let everyBodySleeping = this.bodies.length > 0;
        for (let index = 0; index < this.bodies.length; index++) {
            if (!this.bodies[index].sleeping) {
                everyBodySleeping = false;
                break;
            }
        }
        if (everyBodySleeping) {
            this.lastLengthPolishPasses = 0;
            this.lastWallRepairPasses = 0;
            this.lastCoupledClosurePasses = 0;
            for (const [name, timing] of Object.entries(this.timings)) {
                if (name !== 'total') recordTiming(timing, 0);
            }
            this.stepCount++;
            recordTiming(this.timings.total, now() - totalStart);
            return;
        }
        for (
            let constraintIndex = 0;
            constraintIndex < this.containments.length;
            constraintIndex++
        ) {
            const constraint = this.containments[constraintIndex];
            if (constraint.model !== 'kirchhoff') continue;
            if (constraint._kirchhoffStepOpen) {
                constraint.manifold.endStep({ prune: false });
            }
            constraint.manifold.beginStep();
            constraint._kirchhoffStepOpen = true;
            constraint._kirchhoffMappingLocked = false;
            constraint.kirchhoffContacts.length = 0;
            constraint.kirchhoffMaxViolation = 0;
        }
        let phaseStart = now();
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.contactField = this.contactField;
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
            body.bendLambda.fill(0, activeNodeStart, activeNodeEnd);
            body.curvatureVariationLambdaX.fill(0, activeNodeStart, activeNodeEnd);
            body.curvatureVariationLambdaY.fill(0, activeNodeStart, activeNodeEnd);
            body.curvatureVariationLambdaZ.fill(0, activeNodeStart, activeNodeEnd);
            body.longStraightLambda.fill(0, activeNodeStart, activeNodeEnd);
            body.controlLambda.fill(0, activeNodeStart, activeNodeEnd);
            body.shapeLambda.fill(0, activeNodeStart, activeNodeEnd);
            body.shapeClosureLambda = 0;
            body.restDirectionLambdaX.fill(0, activeSegmentStart, activeSegmentEnd);
            body.restDirectionLambdaY.fill(0, activeSegmentStart, activeSegmentEnd);
            body.restDirectionLambdaZ.fill(0, activeSegmentStart, activeSegmentEnd);
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
            body.wallLambda.fill(0, activeSegmentStart, activeSegmentEnd);
            body.wallFrictionLambda.fill(0, activeSegmentStart, activeSegmentEnd);
            body.wallProjectionX.fill(0, activeNodeStart, activeNodeEnd);
            body.wallProjectionY.fill(0, activeNodeStart, activeNodeEnd);
            body.wallProjectionZ.fill(0, activeNodeStart, activeNodeEnd);
            body.lastMaximumRawSpeed = 0;
            body.lastMaximumWallProjectionSpeed = 0;
            body.lastMaximumWallProjectionNode = -1;
            body.lastMaximumRejectedWallProjectionSpeed = 0;
            body.lastMaximumReconstructedSpeed = 0;
            this.#integrate(body);
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
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveLengths(this.bodies[index], (iteration & 1) === 1);
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
            for (let index = 0; index < this.bodies.length; index++) this.#solveCurvatureVariation(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveLongStraightness(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) this.#solveRestShape(this.bodies[index]);
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                for (let pass = 0; pass < body.restDirectionSubiterations; pass++) {
                    this.#solveRestDirections(body);
                }
            }
            if (iteration + 1 === iterationCount) {
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    body.debugConstraintPhase?.('afterDirections', body);
                }
            }
            for (let index = 0; index < this.bodies.length; index++) {
                this.#solveShapeClosure(this.bodies[index]);
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
            let maximumContactDirectionPasses = 0;
            for (let index = 0; index < this.bodies.length; index++) {
                maximumContactDirectionPasses = Math.max(
                    maximumContactDirectionPasses,
                    this.bodies[index].restDirectionContactPasses
                );
            }
            for (let pass = 0; pass < maximumContactDirectionPasses; pass++) {
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    if (pass >= body.restDirectionContactPasses) continue;
                    this.#solveRestDirections(
                        body,
                        body.restDirectionContactCorrectionScale
                    );
                    this.#solveControls(body);
                    this.#prepareWallContacts(body);
                }
                for (let index = 0; index < this.containments.length; index++) {
                    this.#solveContainment(this.containments[index]);
                }
                for (let index = 0; index < this.toolContacts.length; index++) {
                    this.#solveToolContact(this.toolContacts[index]);
                }
                for (let index = 0; index < this.bodies.length; index++) {
                    const body = this.bodies[index];
                    if (pass >= body.restDirectionContactPasses) continue;
                    this.#solveWallContacts(body);
                }
            }
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
        // Let selected free rods converge more quickly without advancing
        // physical time or modifying their constitutive parameters. This is a
        // body-local pass by design: tool-tool/containment constraints are not
        // solved here, so increasing one tool's relaxation cannot pull or
        // numerically relax the other tool along with it.
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            const relaxationPasses = Math.max(
                0,
                Math.floor(body.relaxationPasses ?? 0)
            );
            body.lastRelaxationPasses = 0;
            for (let pass = 0; pass < relaxationPasses; pass++) {
                if (body.sleeping) break;
                this.#solveControls(body);
                this.#solveLengths(body, (pass & 1) === 1);
                this.#solveBending(body);
                this.#solveCurvatureVariation(body);
                this.#solveLongStraightness(body);
                this.#solveRestShape(body);
                this.#solveRestDirections(body);
                this.#solveShapeClosure(body);
                this.#solveControls(body);
                this.#prepareWallContacts(body);
                this.#solveWallContacts(body);
                this.#solveFoldLimits(body);
                body.lastRelaxationPasses = pass + 1;
            }
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
            for (
                let constraintIndex = 0;
                constraintIndex < this.containments.length;
                constraintIndex++
            ) {
                const constraint = this.containments[constraintIndex];
                if (
                    constraint.enabled &&
                    constraint.limitDistalCorrection &&
                    constraint.innerBody === body
                ) {
                    movingLumenOwnsInner = true;
                    break;
                }
            }
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
        for (
            let constraintIndex = 0;
            constraintIndex < this.containments.length;
            constraintIndex++
        ) {
            const constraint = this.containments[constraintIndex];
            if (
                constraint.enabled &&
                constraint.finalProjection !== 'none' &&
                !constraint.outerFollowsInnerCenterline
            ) {
                needsSecondFinalContainmentPass = true;
                break;
            }
        }
        const finalContainmentPasses = needsSecondFinalContainmentPass ? 2 : 1;
        for (let pass = 0; pass < finalContainmentPasses; pass++) {
            for (let index = 0; index < this.containments.length; index++) {
                const constraint = this.containments[index];
                if (!constraint.enabled || constraint.finalProjection === 'none') continue;
                if (
                    constraint.finalProjection !== 'outer' &&
                    !constraint.outerFollowsInnerCenterline
                ) continue;
                this.#solveContainment(
                    constraint,
                    constraint.finalProjection !== 'outer',
                    constraint.finalProjection === 'outer',
                    false
                );
            }
        }
        // A hard radial projection can leave the contained rod with a large
        // length error or an almost reversed hinge. Alternate one-way lumen
        // projection with the inner rod's structure before body-local wall
        // polishing. This protects the distal capture transition from a single
        // unrestricted correction.
        for (let index = 0; index < this.containments.length; index++) {
            const constraint = this.containments[index];
            if (
                !constraint.enabled ||
                constraint.model === 'kirchhoff' ||
                constraint.finalProjection === 'none' ||
                constraint.finalProjection === 'outer' ||
                constraint.outerFollowsInnerCenterline ||
                constraint.limitDistalCorrection
            ) continue;
            const inner = constraint.innerBody;
            // Aggressive whole-rod polishing is reserved for a fixed topology.
            // While either tool is being fed, the capped containment solve is
            // the only safe owner of the moving material boundary; global
            // relaxation here otherwise turns the first covered node into a
            // large distal jump.
            const innerStructurePasses = 8;
            for (let pass = 0; pass < innerStructurePasses; pass++) {
                this.#solveLengthsGlobal(inner);
                this.#solveBending(inner);
                this.#solveFoldLimits(inner);
                this.#solveLengthsGlobal(inner);
                this.#solveFoldLimits(inner);
                this.#solveLengthsGlobal(inner);
                if (pass + 1 < innerStructurePasses) {
                    this.#solveContainment(constraint, true, false, false);
                }
            }
            this.#solveContainment(constraint, true, false, false);
            this.#captureContainmentOuterPose(constraint);
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
                if (body.postStabilizeShape) {
                    this.#solveRestShape(body);
                }
                this.#solveControls(body);
                this.#solveRestDirections(body);
                this.#solveShapeClosure(body);
                // Direction memory shares nodes with positional controls. A
                // direction pass can therefore reopen the material anchor;
                // rebalance controls before global length/contact polishing,
                // matching the ordering used by the primary XPBD iterations.
                this.#solveControls(body);
                this.#solveLengthsGlobal(body);
                if (body.postStabilizeBending) {
                    this.#solveBending(body);
                    this.#solveCurvatureVariation(body);
                    this.#solveLongStraightness(body);
                    this.#solveLengthsGlobal(body);
                }
                // Restore the signed material side once after unsigned
                // structural bending; applying it both before and after the
                // same pass double-counts the intrinsic moment.
                this.#solveRestDirections(body);
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
            this.#polishRestTurns(body);
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
            if (
                !constraint.enabled ||
                constraint.finalProjection === 'none' ||
                constraint.finalProjection === 'outer' ||
                constraint.outerFollowsInnerCenterline ||
                (
                    !constraint.limitDistalCorrection &&
                    !constraint.innerFollowsOuterCenterline
                )
            ) continue;
            if (!constraint.limitDistalCorrection) {
                this.#carryContainedInnerWithOuter(constraint);
            }
            this.#solveContainment(constraint, true, false, false);
            const inner = constraint.innerBody;
            if (
                constraint.limitDistalCorrection &&
                constraint.preserveStationaryInnerLength
            ) {
                // Advancing an outer catheter over a stationary inner wire
                // changes only the lumen classification. Reconcile locally so
                // that capture cannot stretch the held wire. This is disabled
                // while both material boundaries advance; their portal solve
                // already owns that simultaneous remeshing transition.
                inner.wake();
                for (let pass = 0; pass < 24; pass++) {
                    this.#solveLengths(inner, (pass & 1) === 1);
                    this.#solveBending(inner);
                    this.#solveContainment(constraint, true, false, false);
                }
            } else if (
                constraint.limitDistalCorrection &&
                constraint.reconcileMovingInnerStructure
            ) {
                // With both feeds active the sheath and distal portal move in
                // the same step. Use a small number of global material-length
                // projections to suppress the long-wave fold, but keep this
                // separate from the stronger stationary-wire reconciliation.
                for (let pass = 0; pass < 6; pass++) {
                    this.#solveLengthsGlobal(inner);
                    this.#solveBending(inner);
                    this.#solveFoldLimits(inner);
                    this.#solveContainment(constraint, true, false, false);
                }
            }
        }
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            body.debugConstraintPhase?.('closureAfterCarry', body);
        }
        for (let index = 0; index < this.containments.length; index++) {
            const constraint = this.containments[index];
            if (
                !constraint.enabled ||
                (
                    !constraint.preserveStationaryInnerLength &&
                    !constraint.reconcileMovingInnerStructure
                )
            ) continue;
            const inner = constraint.innerBody;
            for (let sheathIndex = 0; sheathIndex < this.sheaths.length; sheathIndex++) {
                const sheath = this.sheaths[sheathIndex];
                if (sheath.bodies && !sheath.bodies.includes(inner)) continue;
                const materialToOutlet = Math.max(
                    0,
                    sheath.length - Math.max(0, constraint.innerArcOffset)
                );
                const outletSegment = constraint.startNode + Math.floor(
                    materialToOutlet /
                    Math.max(EPSILON, inner.segmentLength)
                );
                this.#transportDistalLengthError(
                    inner,
                    outletSegment,
                    1.1
                );
            }
            this.#solveContainment(constraint, true, false, false);
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
        for (let index = 0; index < this.bodies.length; index++) {
            const body = this.bodies[index];
            finalStructuralClosurePasses = Math.max(
                finalStructuralClosurePasses,
                body.finalStructuralClosurePasses ?? 8
            );
        }
        this.lastCoupledClosurePasses = 0;
        if (this.captureCoupledClosureTrace) {
            this.coupledClosureTrace.length = 0;
        }
        if (
            !this._coupledHasIntrinsicBend ||
            this._coupledHasIntrinsicBend.length < this.bodies.length
        ) {
            this._coupledHasIntrinsicBend = new Uint8Array(
                this.bodies.length
            );
        }
        const coupledHasIntrinsicBend = this._coupledHasIntrinsicBend;
        for (let bodyIndex = 0; bodyIndex < this.bodies.length; bodyIndex++) {
            const body = this.bodies[bodyIndex];
            let hasIntrinsicBend = false;
            for (
                let segment = body.activeStart;
                segment < body.activeEnd;
                segment++
            ) {
                if (!body.intrinsicBendEnabled[segment]) continue;
                hasIntrinsicBend = true;
                break;
            }
            coupledHasIntrinsicBend[bodyIndex] = hasIntrinsicBend ? 1 : 0;
        }
        for (let pass = 0; pass < finalStructuralClosurePasses; pass++) {
            this.lastCoupledClosurePasses = pass + 1;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                const kirchhoffBody = body.rodModel === 'kirchhoff';
                const hasIntrinsicBend = coupledHasIntrinsicBend[index] !== 0;
                if (hasIntrinsicBend) {
                    if (body.intrinsicClosureCorrectionScale > 0) {
                        this.#solveRestDirections(
                            body,
                            body.intrinsicClosureCorrectionScale
                        );
                    }
                    if (!kirchhoffBody) this.#solveFoldLimits(body);
                }
                // The material adaptation constraint is the Kirchhoff rod's
                // inextensibility constraint. It must participate in the last
                // coupled closure even when the body does not use the legacy
                // post-stabilization passes (the guidewire normally does not).
                // Otherwise the outlet, wall and lumen projections below are
                // the final writers of its positions and expose an axially
                // stretched segment until the next fixed step.
                if (kirchhoffBody) {
                    this.#solveBending(body);
                    // Constitutive bend first, unilateral safety bound second,
                    // then adaptation.  This leaves one coherent orientation
                    // state for the centerline instead of letting material
                    // energy immediately undo the fold projection.
                    this.#solveFoldLimits(body);
                    this.#solveLengthsGlobal(body);
                } else if (body.postStabilizationPasses > 0) {
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
            }
            // Close both material rods and their lumen contacts as one
            // symmetric system. This uses the same compliant unilateral
            // constraint as the primary iterations; it is not a centerline
            // snap and never invokes the legacy distal direction projection.
            for (
                let constraintIndex = 0;
                constraintIndex < this.containments.length;
                constraintIndex++
            ) {
                const constraint = this.containments[constraintIndex];
                if (
                    constraint.model !== 'kirchhoff' ||
                    !constraint.enabled
                ) continue;
                this.#solveKirchhoffContainment(constraint, true, false);
            }
            let coupledResidualSettled = true;
            const tracedBodies = this.captureCoupledClosureTrace ? [] : null;
            for (let index = 0; index < this.bodies.length; index++) {
                const body = this.bodies[index];
                const hasIntrinsicBend = coupledHasIntrinsicBend[index] !== 0;
                const kirchhoffBody = body.rodModel === 'kirchhoff';
                if (
                    !kirchhoffBody &&
                    !hasIntrinsicBend &&
                    body.postStabilizationPasses <= 0
                ) continue;
                const materialResidual = !kirchhoffBody && hasIntrinsicBend &&
                    body.intrinsicClosureCorrectionScale > 0
                    ? this.#bodyStats(body).maxMaterialTurnResidualDegrees
                    : 0;
                const foldError = hasIntrinsicBend &&
                    this.#hasFoldLimitErrorOver(body, 0.02);
                const lengthError = !(
                    !kirchhoffBody && body.postStabilizationPasses <= 0
                ) && this.#hasLengthErrorOver(body, 0.002);
                coupledResidualSettled = coupledResidualSettled &&
                    !foldError &&
                    (
                        body.intrinsicClosureCorrectionScale <= 0 ||
                        materialResidual <= 1
                    ) && !lengthError;
                tracedBodies?.push({
                    id: body.id,
                    foldError,
                    lengthError,
                    materialResidual
                });
            }
            let tracedContainmentViolation = 0;
            // The exact post-projection containment scan cannot change any
            // generalized coordinate. If a material residual already requires
            // another sweep, measuring every lumen segment here cannot affect
            // the convergence decision and only repeats the contact geometry
            // traversal. Defer it until containment is the remaining gate (or
            // tracing explicitly requests the value). This preserves every
            // projection, their order, the pass count and the final state.
            const measureContainmentResidual =
                coupledResidualSettled || this.captureCoupledClosureTrace;
            for (
                let constraintIndex = 0;
                constraintIndex < this.containments.length;
                constraintIndex++
            ) {
                const constraint = this.containments[constraintIndex];
                if (
                    constraint.model === 'kirchhoff' &&
                    constraint.enabled &&
                    measureContainmentResidual
                ) {
                    constraint.kirchhoffMaxViolation =
                        this.#measureKirchhoffContainmentViolation(constraint);
                    tracedContainmentViolation = Math.max(
                        tracedContainmentViolation,
                        constraint.kirchhoffMaxViolation
                    );
                }
                if (
                    constraint.model === 'kirchhoff' &&
                    constraint.enabled &&
                    measureContainmentResidual &&
                    constraint.kirchhoffMaxViolation > 0.001
                ) {
                    coupledResidualSettled = false;
                }
            }
            if (this.captureCoupledClosureTrace) {
                this.coupledClosureTrace.push({
                    pass: pass + 1,
                    settled: coupledResidualSettled,
                    containmentViolation: tracedContainmentViolation,
                    bodies: tracedBodies
                });
            }
            if (coupledResidualSettled) break;
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
            if (
                !constraint.enabled ||
                constraint.finalProjection === 'none' ||
                constraint.finalProjection === 'outer' ||
                constraint.outerFollowsInnerCenterline ||
                (
                    !constraint.limitDistalCorrection &&
                    !constraint.innerFollowsOuterCenterline
                )
            ) continue;
            const inner = constraint.innerBody;
            const residualStart = clamp(
                constraint.startNode - 1,
                inner.activeStart,
                inner.activeEnd
            );
            const residualEnd = clamp(
                constraint.endNode + 1,
                residualStart,
                inner.activeEnd
            );
            const closurePasses = 64;
            for (let pass = 0; pass < closurePasses; pass++) {
                this.#solveFoldLimits(inner);
                this.#solveLengthsGlobal(inner);
                for (let node = residualStart; node <= residualEnd; node++) {
                    inner.postPassStartX[node] = inner.x[node];
                    inner.postPassStartY[node] = inner.y[node];
                    inner.postPassStartZ[node] = inner.z[node];
                }
                this.#solveContainment(constraint, true, false, false);
                let maximumContainmentCorrection = 0;
                for (let node = residualStart; node <= residualEnd; node++) {
                    maximumContainmentCorrection = Math.max(
                        maximumContainmentCorrection,
                        magnitude3(
                            inner.x[node] - inner.postPassStartX[node],
                            inner.y[node] - inner.postPassStartY[node],
                            inner.z[node] - inner.postPassStartZ[node]
                        )
                    );
                }
                if (
                    maximumContainmentCorrection <= (
                        constraint.limitDistalCorrection ? 0.002 : 0.00025
                    ) &&
                    !this.#hasFoldLimitErrorOver(inner, 0.02) &&
                    !this.#hasLengthErrorOver(inner, 0.002)
                ) break;
            }
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
        for (
            let constraintIndex = 0;
            constraintIndex < this.containments.length;
            constraintIndex++
        ) {
            const constraint = this.containments[constraintIndex];
            if (
                constraint.model !== 'kirchhoff' ||
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
        this.accumulator = 0;
        this.stepCount = 0;
        this.lastSubsteps = 0;
        this.droppedTime = 0;
        for (const body of this.bodies) {
            body.lengthLambda.fill(0);
            body.bendLambda.fill(0);
            body.curvatureVariationLambdaX.fill(0);
            body.curvatureVariationLambdaY.fill(0);
            body.curvatureVariationLambdaZ.fill(0);
            body.longStraightLambda.fill(0);
            body.controlLambda.fill(0);
            body.shapeLambda.fill(0);
            body.shapeClosureLambda = 0;
            body.restDirectionLambdaX.fill(0);
            body.restDirectionLambdaY.fill(0);
            body.restDirectionLambdaZ.fill(0);
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
            body.lastMaximumRawSpeed = 0;
            body.lastMaximumWallProjectionSpeed = 0;
            body.lastMaximumWallProjectionNode = -1;
            body.lastMaximumRejectedWallProjectionSpeed = 0;
            body.lastMaximumReconstructedSpeed = 0;
            body.copyCurrentToPrevious();
            if (body.rodModel === 'kirchhoff') {
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
            if (containment.model === 'kirchhoff') {
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
            containment.portalLambda = 0;
            containment.portalDirectionLambda = 0;
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
                enabled: constraint.enabled,
                contacts: constraint.kirchhoffContacts?.length ?? 0,
                maximumViolation:
                    constraint.kirchhoffMaxViolation ?? 0,
                openCacheHits:
                    constraint.kirchhoffOpenCacheHits ?? 0
            })),
            bodies
        };
    }

    #integrate(body) {
        if (body.sleeping) return;
        const dt = this.fixedDt;
        const dtSquared = dt * dt;
        const start = body.activeStart;
        const end = body.activeEnd;
        if (body.rodModel === 'kirchhoff') {
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
                    wasActive ? body.wallBranchId[index] : -1
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
            if (signedGap > this.contactActivation) {
                body.wallLambda[index] = 0;
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
                    sampleCount
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

    #hasFoldLimitErrorOver(body, toleranceDegrees = 0) {
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.count - 1, body.activeEnd);
        const kirchhoffScratch = body.kirchhoffScratch ??= {};
        const cache = kirchhoffScratch.foldResidual ??= {
            limitDegrees: new Float64Array(body.count),
            limitCosine: new Float64Array(body.count)
        };
        if (!cache.initialized) {
            cache.limitDegrees.fill(Number.NaN);
            cache.initialized = true;
        }
        for (let index = start; index < end; index++) {
            const incomingX = body.x[index] - body.x[index - 1];
            const incomingY = body.y[index] - body.y[index - 1];
            const incomingZ = body.z[index] - body.z[index - 1];
            const outgoingX = body.x[index + 1] - body.x[index];
            const outgoingY = body.y[index + 1] - body.y[index];
            const outgoingZ = body.z[index + 1] - body.z[index];
            const denominator = magnitude3(incomingX, incomingY, incomingZ) *
                magnitude3(outgoingX, outgoingY, outgoingZ);
            if (denominator < EPSILON) return true;
            const thresholdDegrees =
                body.maxBendAngleByNode[index] + toleranceDegrees;
            if (thresholdDegrees >= 180) continue;
            if (thresholdDegrees < 0) return true;
            if (cache.limitDegrees[index] !== thresholdDegrees) {
                cache.limitDegrees[index] = thresholdDegrees;
                cache.limitCosine[index] = Math.cos(
                    thresholdDegrees * Math.PI / 180
                );
            }
            const cosine = clamp(
                (
                    incomingX * outgoingX +
                    incomingY * outgoingY +
                    incomingZ * outgoingZ
                ) / denominator,
                -1,
                1
            );
            if (cosine < cache.limitCosine[index]) return true;
        }
        return false;
    }

    #hasRestDirectionErrorOver(body, threshold) {
        if (body.rodModel === 'kirchhoff') return false;
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount, body.activeEnd);
        for (let segment = start; segment < end; segment++) {
            if (!body.restDirectionEnabled[segment]) continue;
            const error = magnitude3(
                body.x[segment + 1] - body.x[segment] -
                    body.restDirectionX[segment],
                body.y[segment + 1] - body.y[segment] -
                    body.restDirectionY[segment],
                body.z[segment + 1] - body.z[segment] -
                    body.restDirectionZ[segment]
            );
            if (error > threshold) return true;
        }
        return false;
    }

    #solveControls(body) {
        if (body.sleeping) return;
        if (body.rodModel === 'kirchhoff') {
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
        if (body.sleeping || body.segmentCount < 2) return;
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.segmentCount, body.activeEnd);
        // One alternating Gauss-Seidel sweep per world iteration transmits the
        // same material moment in both directions over successive iterations.
        // Doing a forward and backward sweep on every call doubled the most
        // expensive SO(3) constraint without adding a new physical equation.
        const reverse = body.kirchhoffBendSweepReverse === true;
        body.kirchhoffBendSweepReverse = !reverse;
        solveBendTwistXPBDBlockArraySweep(
            body,
            start,
            end,
            reverse,
            this.fixedDt
        );
    }

    #solveLengths(body, reverse = false) {
        if (body.rodModel === 'kirchhoff') {
            this.#solveKirchhoffAdaptation(body, reverse);
            return;
        }
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
        if (body.rodModel === 'kirchhoff') {
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
        if (body.rodModel === 'kirchhoff') {
            this.#solveKirchhoffBendTwist(body);
            return;
        }
        if (body.sleeping || body.count < 3) return;
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.count - 1, body.activeEnd);
        // A symmetric sweep is the discrete rod equivalent of transmitting a
        // bending moment in both material directions. A one-way sweep leaves
        // a fed shaft much softer distally than proximally and lets contact
        // accumulate a travelling sinusoidal buckle.
        for (let sweep = 0; sweep < 2; sweep++) {
        for (let offset = 0; offset < end - start; offset++) {
            const index = sweep === 0
                ? start + offset
                : end - 1 - offset;
            if (body.intrinsicBendEnabled[index]) continue;
            const previous = index - 1;
            const next = index + 1;
            let incomingX = body.x[index] - body.x[previous];
            let incomingY = body.y[index] - body.y[previous];
            let incomingZ = body.z[index] - body.z[previous];
            let outgoingX = body.x[next] - body.x[index];
            let outgoingY = body.y[next] - body.y[index];
            let outgoingZ = body.z[next] - body.z[index];
            const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
            const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
            if (incomingLength < EPSILON || outgoingLength < EPSILON) continue;
            incomingX /= incomingLength;
            incomingY /= incomingLength;
            incomingZ /= incomingLength;
            outgoingX /= outgoingLength;
            outgoingY /= outgoingLength;
            outgoingZ /= outgoingLength;
            const dot = clamp(
                incomingX * outgoingX +
                    incomingY * outgoingY +
                    incomingZ * outgoingZ,
                -1,
                1
            );
            const angle = Math.acos(dot);
            if (angle < 1e-7) continue;
            let axisX = incomingY * outgoingZ - incomingZ * outgoingY;
            let axisY = incomingZ * outgoingX - incomingX * outgoingZ;
            let axisZ = incomingX * outgoingY - incomingY * outgoingX;
            const axisLength = magnitude3(axisX, axisY, axisZ);
            if (axisLength < EPSILON) continue;
            axisX /= axisLength;
            axisY /= axisLength;
            axisZ /= axisLength;
            const gradientPreviousX =
                (axisY * incomingZ - axisZ * incomingY) / incomingLength;
            const gradientPreviousY =
                (axisZ * incomingX - axisX * incomingZ) / incomingLength;
            const gradientPreviousZ =
                (axisX * incomingY - axisY * incomingX) / incomingLength;
            const gradientNextX =
                (axisY * outgoingZ - axisZ * outgoingY) / outgoingLength;
            const gradientNextY =
                (axisZ * outgoingX - axisX * outgoingZ) / outgoingLength;
            const gradientNextZ =
                (axisX * outgoingY - axisY * outgoingX) / outgoingLength;
            const gradientJointX = -gradientPreviousX - gradientNextX;
            const gradientJointY = -gradientPreviousY - gradientNextY;
            const gradientJointZ = -gradientPreviousZ - gradientNextZ;
            const previousWeight = body.inverseMass[previous];
            const jointWeight = body.inverseMass[index];
            const nextWeight = body.inverseMass[next];
            const alpha = (
                body.bendComplianceByNode[index] *
                CHORD_TO_ANGULAR_BEND_COMPLIANCE_SCALE
            ) / (this.fixedDt * this.fixedDt);
            const denominator = alpha +
                previousWeight * magnitude3(
                    gradientPreviousX,
                    gradientPreviousY,
                    gradientPreviousZ
                ) ** 2 +
                jointWeight * magnitude3(
                    gradientJointX,
                    gradientJointY,
                    gradientJointZ
                ) ** 2 +
                nextWeight * magnitude3(
                    gradientNextX,
                    gradientNextY,
                    gradientNextZ
                ) ** 2;
            if (denominator < EPSILON) continue;
            let deltaLambda = (
                -angle - alpha * body.bendLambda[index]
            ) / denominator;
            const maximumDisplacement = Math.max(
                previousWeight * magnitude3(
                    gradientPreviousX,
                    gradientPreviousY,
                    gradientPreviousZ
                ),
                jointWeight * magnitude3(
                    gradientJointX,
                    gradientJointY,
                    gradientJointZ
                ),
                nextWeight * magnitude3(
                    gradientNextX,
                    gradientNextY,
                    gradientNextZ
                )
            ) * Math.abs(deltaLambda);
            // Bound each angular-energy projection so a short material
            // interval cannot turn one local correction into a visible kick.
            const displacementLimit = Math.min(incomingLength, outgoingLength) * 0.2;
            if (maximumDisplacement > displacementLimit) {
                deltaLambda *= displacementLimit / maximumDisplacement;
            }
            body.bendLambda[index] += deltaLambda;
            body.x[previous] += gradientPreviousX * deltaLambda * previousWeight;
            body.y[previous] += gradientPreviousY * deltaLambda * previousWeight;
            body.z[previous] += gradientPreviousZ * deltaLambda * previousWeight;
            body.x[index] += gradientJointX * deltaLambda * jointWeight;
            body.y[index] += gradientJointY * deltaLambda * jointWeight;
            body.z[index] += gradientJointZ * deltaLambda * jointWeight;
            body.x[next] += gradientNextX * deltaLambda * nextWeight;
            body.y[next] += gradientNextY * deltaLambda * nextWeight;
            body.z[next] += gradientNextZ * deltaLambda * nextWeight;
        }
        }
    }

    #solveCurvatureVariation(body) {
        if (body.rodModel === 'kirchhoff') return;
        if (body.sleeping || !body.curvatureVariationEnabled || body.count < 4) return;
        const start = Math.max(body.activeStart, body.curvatureVariationStartNode);
        const end = Math.min(body.activeEnd, body.curvatureVariationEndNode);
        if (end - start < 3) return;
        const alpha = body.curvatureVariationCompliance /
            (this.fixedDt * this.fixedDt);
        for (let index = start; index + 3 <= end; index++) {
            const index1 = index + 1;
            const index2 = index + 2;
            const index3 = index + 3;
            const w0 = body.inverseMass[index];
            const w1 = body.inverseMass[index1];
            const w2 = body.inverseMass[index2];
            const w3 = body.inverseMass[index3];
            const denominator = alpha + w0 + 9 * w1 + 9 * w2 + w3;
            if (denominator < EPSILON) continue;
            const constraintX = body.x[index] - 3 * body.x[index1] +
                3 * body.x[index2] - body.x[index3];
            const deltaX = (
                -constraintX - alpha * body.curvatureVariationLambdaX[index]
            ) / denominator;
            body.curvatureVariationLambdaX[index] += deltaX;
            body.x[index] += w0 * deltaX;
            body.x[index1] -= 3 * w1 * deltaX;
            body.x[index2] += 3 * w2 * deltaX;
            body.x[index3] -= w3 * deltaX;

            const constraintY = body.y[index] - 3 * body.y[index1] +
                3 * body.y[index2] - body.y[index3];
            const deltaY = (
                -constraintY - alpha * body.curvatureVariationLambdaY[index]
            ) / denominator;
            body.curvatureVariationLambdaY[index] += deltaY;
            body.y[index] += w0 * deltaY;
            body.y[index1] -= 3 * w1 * deltaY;
            body.y[index2] += 3 * w2 * deltaY;
            body.y[index3] -= w3 * deltaY;

            const constraintZ = body.z[index] - 3 * body.z[index1] +
                3 * body.z[index2] - body.z[index3];
            const deltaZ = (
                -constraintZ - alpha * body.curvatureVariationLambdaZ[index]
            ) / denominator;
            body.curvatureVariationLambdaZ[index] += deltaZ;
            body.z[index] += w0 * deltaZ;
            body.z[index1] -= 3 * w1 * deltaZ;
            body.z[index2] += 3 * w2 * deltaZ;
            body.z[index3] -= w3 * deltaZ;
        }
    }

    #solveLongStraightness(body) {
        if (body.rodModel === 'kirchhoff') return;
        const span = body.longStraightSpan;
        if (body.sleeping || span < 3 || body.count <= span) return;
        const start = Math.max(body.activeStart, body.longStraightStartNode);
        const end = Math.min(body.activeEnd, body.longStraightEndNode);
        if (end - start < span) return;
        const alpha = body.longStraightCompliance /
            (this.fixedDt * this.fixedDt);
        for (let index = start; index + span <= end; index++) {
            const other = index + span;
            const dx = body.x[other] - body.x[index];
            const dy = body.y[other] - body.y[index];
            const dz = body.z[other] - body.z[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            let restChord = 0;
            for (let segment = index; segment < other; segment++) {
                restChord += body.restLength[segment];
            }
            const w0 = body.inverseMass[index];
            const w1 = body.inverseMass[other];
            const denominator = w0 + w1 + alpha;
            if (denominator < EPSILON) continue;
            const constraint = distance - restChord;
            const deltaLambda = (
                -constraint - alpha * body.longStraightLambda[index]
            ) / denominator;
            body.longStraightLambda[index] += deltaLambda;
            const nx = dx / distance;
            const ny = dy / distance;
            const nz = dz / distance;
            body.x[index] -= nx * deltaLambda * w0;
            body.y[index] -= ny * deltaLambda * w0;
            body.z[index] -= nz * deltaLambda * w0;
            body.x[other] += nx * deltaLambda * w1;
            body.y[other] += ny * deltaLambda * w1;
            body.z[other] += nz * deltaLambda * w1;
        }
    }

    #solveRestShape(body) {
        if (body.rodModel === 'kirchhoff') return;
        if (body.sleeping) return;
        const dtSquared = this.fixedDt * this.fixedDt;
        const neutralStart = Math.max(
            body.activeStart,
            body.restShapeTranslationNeutralStart
        );
        const neutralEnd = Math.min(
            body.activeEnd,
            body.restShapeTranslationNeutralEnd
        );
        const translationNeutral = neutralStart >= body.activeStart &&
            neutralEnd >= neutralStart;
        let neutralCorrectionX = 0;
        let neutralCorrectionY = 0;
        let neutralCorrectionZ = 0;
        let neutralCount = 0;
        for (let index = body.activeStart; index <= body.activeEnd; index++) {
            if (!body.restShapeEnabled[index] || body.inverseMass[index] <= 0) continue;
            const dx = body.x[index] - body.restShapeX[index];
            const dy = body.y[index] - body.restShapeY[index];
            const dz = body.z[index] - body.restShapeZ[index];
            const distance = magnitude3(dx, dy, dz);
            if (distance < EPSILON) continue;
            const alpha = body.restShapeCompliance[index] / dtSquared;
            let deltaLambda = (-distance - alpha * body.shapeLambda[index]) /
                (body.inverseMass[index] + alpha);
            const displacement = Math.abs(deltaLambda) * body.inverseMass[index];
            const maxCorrection = body.restShapeMaxCorrection[index];
            if (displacement > maxCorrection) {
                deltaLambda *= maxCorrection / displacement;
            }
            body.shapeLambda[index] += deltaLambda;
            const scale = deltaLambda / distance * body.inverseMass[index];
            const correctionX = dx * scale;
            const correctionY = dy * scale;
            const correctionZ = dz * scale;
            if (
                translationNeutral &&
                index >= neutralStart &&
                index <= neutralEnd
            ) {
                body.restShapeCorrectionX[index] = correctionX;
                body.restShapeCorrectionY[index] = correctionY;
                body.restShapeCorrectionZ[index] = correctionZ;
                neutralCorrectionX += correctionX;
                neutralCorrectionY += correctionY;
                neutralCorrectionZ += correctionZ;
                neutralCount++;
                continue;
            }
            body.x[index] += correctionX;
            body.y[index] += correctionY;
            body.z[index] += correctionZ;
        }
        if (neutralCount <= 0) return;
        neutralCorrectionX /= neutralCount;
        neutralCorrectionY /= neutralCount;
        neutralCorrectionZ /= neutralCount;
        for (let index = neutralStart; index <= neutralEnd; index++) {
            if (!body.restShapeEnabled[index] || body.inverseMass[index] <= 0) continue;
            body.x[index] += body.restShapeCorrectionX[index] - neutralCorrectionX;
            body.y[index] += body.restShapeCorrectionY[index] - neutralCorrectionY;
            body.z[index] += body.restShapeCorrectionZ[index] - neutralCorrectionZ;
        }
    }

    #solveShapeClosure(body) {
        if (body.rodModel === 'kirchhoff') return;
        if (body.sleeping || !body.shapeClosureEnabled) return;
        const start = body.shapeClosureStart;
        const end = body.shapeClosureEnd;
        if (
            start < body.activeStart ||
            end > body.activeEnd ||
            start === end
        ) return;
        const dx = body.x[end] - body.x[start];
        const dy = body.y[end] - body.y[start];
        const dz = body.z[end] - body.z[start];
        const distance = magnitude3(dx, dy, dz);
        if (distance < EPSILON) return;
        const w0 = body.inverseMass[start];
        const w1 = body.inverseMass[end];
        const alpha = body.shapeClosureCompliance /
            (this.fixedDt * this.fixedDt);
        const denominator = w0 + w1 + alpha;
        if (denominator < EPSILON) return;
        let deltaLambda = (
            -(distance - body.shapeClosureDistance) -
            alpha * body.shapeClosureLambda
        ) / denominator;
        const displacement = Math.max(w0, w1) * Math.abs(deltaLambda);
        if (displacement > body.shapeClosureMaxCorrection) {
            deltaLambda *= body.shapeClosureMaxCorrection / displacement;
        }
        body.shapeClosureLambda += deltaLambda;
        const nx = dx / distance;
        const ny = dy / distance;
        const nz = dz / distance;
        body.x[start] -= nx * deltaLambda * w0;
        body.y[start] -= ny * deltaLambda * w0;
        body.z[start] -= nz * deltaLambda * w0;
        body.x[end] += nx * deltaLambda * w1;
        body.y[end] += ny * deltaLambda * w1;
        body.z[end] += nz * deltaLambda * w1;
    }

    #solveRestDirections(body, correctionScale = 1) {
        if (body.rodModel === 'kirchhoff') return;
        if (body.sleeping) return;
        const dtSquared = this.fixedDt * this.fixedDt;
        const start = Math.max(0, body.activeStart);
        const end = Math.min(body.segmentCount - 1, body.activeEnd - 1);
        const segmentCount = Math.max(0, end - start + 1);
        // Adjacent direction constraints share a node. A one-way sweep leaves
        // the last constraints satisfied and continually reopens the first
        // ones. Symmetric Gauss-Seidel propagates the signed curvature back
        // to the material anchor within the same solver iteration.
        const intrinsicRod = body.intrinsicBendEnabled.some(
            (enabled, segment) => enabled &&
                segment >= body.activeStart &&
                segment < body.activeEnd
        );
        for (let sweep = 0; sweep < (intrinsicRod ? 2 : 1); sweep++) {
        for (let offset = 0; offset < segmentCount; offset++) {
            const segment = sweep === 0
                ? start + offset
                : end - offset;
            if (!body.restDirectionEnabled[segment]) continue;
            const next = segment + 1;
            let targetX = body.restDirectionX[segment];
            let targetY = body.restDirectionY[segment];
            let targetZ = body.restDirectionZ[segment];
            let w0 = body.inverseMass[segment];
            const w1 = body.inverseMass[next];
            if (body.restDirectionRelative[segment]) {
                const previous = segment - 1;
                if (previous < body.activeStart) continue;
                const axisX = body.restDirectionAxisX[segment];
                const axisY = body.restDirectionAxisY[segment];
                const axisZ = body.restDirectionAxisZ[segment];
                const compliance = body.restDirectionCompliance[segment];
                const maximumCorrection =
                    body.restDirectionMaxCorrection[segment] * correctionScale;
                let outgoingX = body.x[next] - body.x[segment];
                let outgoingY = body.y[next] - body.y[segment];
                let outgoingZ = body.z[next] - body.z[segment];

                let incomingX = body.x[segment] - body.x[previous];
                let incomingY = body.y[segment] - body.y[previous];
                let incomingZ = body.z[segment] - body.z[previous];
                outgoingX = body.x[next] - body.x[segment];
                outgoingY = body.y[next] - body.y[segment];
                outgoingZ = body.z[next] - body.z[segment];
                const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
                const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
                if (incomingLength < EPSILON || outgoingLength < EPSILON) continue;
                incomingX /= incomingLength;
                incomingY /= incomingLength;
                incomingZ /= incomingLength;
                outgoingX /= outgoingLength;
                outgoingY /= outgoingLength;
                outgoingZ /= outgoingLength;

                // Evaluate the signed hinge angle in the common material
                // plane. Unlike the former target-vector projection, this
                // three-node constraint carries reaction both proximally and
                // distally. A wall-blocked tip can therefore reposition the
                // loop instead of reversing curvature into a zig-zag.
                const incomingAxial =
                    axisX * incomingX + axisY * incomingY + axisZ * incomingZ;
                const outgoingAxial =
                    axisX * outgoingX + axisY * outgoingY + axisZ * outgoingZ;
                incomingX -= axisX * incomingAxial;
                incomingY -= axisY * incomingAxial;
                incomingZ -= axisZ * incomingAxial;
                outgoingX -= axisX * outgoingAxial;
                outgoingY -= axisY * outgoingAxial;
                outgoingZ -= axisZ * outgoingAxial;
                const incomingPlanarLength = magnitude3(incomingX, incomingY, incomingZ);
                const outgoingPlanarLength = magnitude3(outgoingX, outgoingY, outgoingZ);
                if (incomingPlanarLength < EPSILON || outgoingPlanarLength < EPSILON) continue;
                incomingX /= incomingPlanarLength;
                incomingY /= incomingPlanarLength;
                incomingZ /= incomingPlanarLength;
                outgoingX /= outgoingPlanarLength;
                outgoingY /= outgoingPlanarLength;
                outgoingZ /= outgoingPlanarLength;
                const crossX = incomingY * outgoingZ - incomingZ * outgoingY;
                const crossY = incomingZ * outgoingX - incomingX * outgoingZ;
                const crossZ = incomingX * outgoingY - incomingY * outgoingX;
                const sine = axisX * crossX + axisY * crossY + axisZ * crossZ;
                const cosine = clamp(
                    incomingX * outgoingX +
                    incomingY * outgoingY +
                    incomingZ * outgoingZ,
                    -1,
                    1
                );
                let angleError = Math.atan2(sine, cosine) -
                    body.restDirectionTurnAngle[segment];
                if (angleError > Math.PI) angleError -= Math.PI * 2;
                else if (angleError < -Math.PI) angleError += Math.PI * 2;

                const gradientPreviousX = (axisY * incomingZ - axisZ * incomingY) /
                    incomingLength;
                const gradientPreviousY = (axisZ * incomingX - axisX * incomingZ) /
                    incomingLength;
                const gradientPreviousZ = (axisX * incomingY - axisY * incomingX) /
                    incomingLength;
                const gradientNextX = (axisY * outgoingZ - axisZ * outgoingY) /
                    outgoingLength;
                const gradientNextY = (axisZ * outgoingX - axisX * outgoingZ) /
                    outgoingLength;
                const gradientNextZ = (axisX * outgoingY - axisY * outgoingX) /
                    outgoingLength;
                const gradientJointX = -gradientPreviousX - gradientNextX;
                const gradientJointY = -gradientPreviousY - gradientNextY;
                const gradientJointZ = -gradientPreviousZ - gradientNextZ;
                const distalBias = body.restDirectionDistalBias[segment];
                const reactionScale = 1 - distalBias;
                const previousWeight = body.inverseMass[previous] * reactionScale;
                const jointWeight = w0 * reactionScale;
                const nextWeight = w1;
                const angularWeight =
                    previousWeight * (
                        gradientPreviousX * gradientPreviousX +
                        gradientPreviousY * gradientPreviousY +
                        gradientPreviousZ * gradientPreviousZ
                    ) +
                    jointWeight * (
                        gradientJointX * gradientJointX +
                        gradientJointY * gradientJointY +
                        gradientJointZ * gradientJointZ
                    ) +
                    nextWeight * (
                        gradientNextX * gradientNextX +
                        gradientNextY * gradientNextY +
                        gradientNextZ * gradientNextZ
                    );
                const meanLength = Math.max(
                    EPSILON,
                    (incomingLength + outgoingLength) * 0.5
                );
                const angularAlpha = compliance / (dtSquared * meanLength * meanLength);
                if (angularWeight + angularAlpha < EPSILON) continue;
                let angularDelta = (
                    -angleError -
                    angularAlpha * body.restDirectionLambdaX[segment]
                ) / (angularWeight + angularAlpha);
                const maximumAngularDisplacement = Math.max(
                    previousWeight * magnitude3(
                        gradientPreviousX,
                        gradientPreviousY,
                        gradientPreviousZ
                    ),
                    jointWeight * magnitude3(
                        gradientJointX,
                        gradientJointY,
                        gradientJointZ
                    ),
                    nextWeight * magnitude3(
                        gradientNextX,
                        gradientNextY,
                        gradientNextZ
                    )
                ) * Math.abs(angularDelta);
                if (maximumAngularDisplacement > maximumCorrection) {
                    angularDelta *= maximumCorrection /
                        maximumAngularDisplacement;
                }
                body.restDirectionLambdaX[segment] += angularDelta;
                body.x[previous] += gradientPreviousX * angularDelta * previousWeight;
                body.y[previous] += gradientPreviousY * angularDelta * previousWeight;
                body.z[previous] += gradientPreviousZ * angularDelta * previousWeight;
                body.x[segment] += gradientJointX * angularDelta * jointWeight;
                body.y[segment] += gradientJointY * angularDelta * jointWeight;
                body.z[segment] += gradientJointZ * angularDelta * jointWeight;
                body.x[next] += gradientNextX * angularDelta * nextWeight;
                body.y[next] += gradientNextY * angularDelta * nextWeight;
                body.z[next] += gradientNextZ * angularDelta * nextWeight;

                // Keep a world-space diagnostic target for sleep/error
                // reporting and test introspection.
                const targetAngle = body.restDirectionTurnAngle[segment];
                const targetCosine = Math.cos(targetAngle);
                const targetSine = Math.sin(targetAngle);
                const targetCrossX = axisY * incomingZ - axisZ * incomingY;
                const targetCrossY = axisZ * incomingX - axisX * incomingZ;
                const targetCrossZ = axisX * incomingY - axisY * incomingX;
                const restLength = body.restLength[segment];
                body.restDirectionX[segment] = (
                    incomingX * targetCosine + targetCrossX * targetSine
                ) * restLength;
                body.restDirectionY[segment] = (
                    incomingY * targetCosine + targetCrossY * targetSine
                ) * restLength;
                body.restDirectionZ[segment] = (
                    incomingZ * targetCosine + targetCrossZ * targetSine
                ) * restLength;
                continue;
            }
            const weight = w0 + w1;
            if (weight < EPSILON) continue;
            const alpha = body.restDirectionCompliance[segment] / dtSquared;
            const denominator = weight + alpha;
            const errorX =
                body.x[next] - body.x[segment] - targetX;
            const errorY =
                body.y[next] - body.y[segment] - targetY;
            const errorZ =
                body.z[next] - body.z[segment] - targetZ;
            let deltaX = (
                -errorX - alpha * body.restDirectionLambdaX[segment]
            ) / denominator;
            let deltaY = (
                -errorY - alpha * body.restDirectionLambdaY[segment]
            ) / denominator;
            let deltaZ = (
                -errorZ - alpha * body.restDirectionLambdaZ[segment]
            ) / denominator;
            const correctionLength = magnitude3(deltaX, deltaY, deltaZ);
            const maximumCorrection =
                body.restDirectionMaxCorrection[segment] * correctionScale;
            if (correctionLength > maximumCorrection) {
                const correctionScale = maximumCorrection / correctionLength;
                deltaX *= correctionScale;
                deltaY *= correctionScale;
                deltaZ *= correctionScale;
            }
            body.restDirectionLambdaX[segment] += deltaX;
            body.restDirectionLambdaY[segment] += deltaY;
            body.restDirectionLambdaZ[segment] += deltaZ;
            body.x[segment] -= deltaX * w0;
            body.y[segment] -= deltaY * w0;
            body.z[segment] -= deltaZ * w0;
            body.x[next] += deltaX * w1;
            body.y[next] += deltaY * w1;
            body.z[next] += deltaZ * w1;
        }
        }
    }

    #polishRestTurns(body) {
        if (body.rodModel === 'kirchhoff') return;
        const maximumAngle = body.restTurnPolishMaxAngle;
        if (body.sleeping || maximumAngle <= 0 || body.activeEnd < body.activeStart + 2) {
            return;
        }
        const start = Math.max(1, body.activeStart + 1);
        const end = Math.min(body.segmentCount - 1, body.activeEnd - 1);
        for (let segment = start; segment <= end; segment++) {
            if (
                !body.restDirectionEnabled[segment] ||
                !body.restDirectionRelative[segment]
            ) continue;
            const previous = segment - 1;
            const next = segment + 1;
            const axisX = body.restDirectionAxisX[segment];
            const axisY = body.restDirectionAxisY[segment];
            const axisZ = body.restDirectionAxisZ[segment];
            let incomingX = body.x[segment] - body.x[previous];
            let incomingY = body.y[segment] - body.y[previous];
            let incomingZ = body.z[segment] - body.z[previous];
            let outgoingX = body.x[next] - body.x[segment];
            let outgoingY = body.y[next] - body.y[segment];
            let outgoingZ = body.z[next] - body.z[segment];
            const incomingAxial =
                incomingX * axisX + incomingY * axisY + incomingZ * axisZ;
            const outgoingAxial =
                outgoingX * axisX + outgoingY * axisY + outgoingZ * axisZ;
            incomingX -= axisX * incomingAxial;
            incomingY -= axisY * incomingAxial;
            incomingZ -= axisZ * incomingAxial;
            outgoingX -= axisX * outgoingAxial;
            outgoingY -= axisY * outgoingAxial;
            outgoingZ -= axisZ * outgoingAxial;
            const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
            const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
            if (incomingLength < EPSILON || outgoingLength < EPSILON) continue;
            incomingX /= incomingLength;
            incomingY /= incomingLength;
            incomingZ /= incomingLength;
            outgoingX /= outgoingLength;
            outgoingY /= outgoingLength;
            outgoingZ /= outgoingLength;
            const crossX = incomingY * outgoingZ - incomingZ * outgoingY;
            const crossY = incomingZ * outgoingX - incomingX * outgoingZ;
            const crossZ = incomingX * outgoingY - incomingY * outgoingX;
            const sine = axisX * crossX + axisY * crossY + axisZ * crossZ;
            const cosine = clamp(
                incomingX * outgoingX +
                    incomingY * outgoingY +
                    incomingZ * outgoingZ,
                -1,
                1
            );
            let correction = body.restDirectionTurnAngle[segment] -
                Math.atan2(sine, cosine);
            if (correction > Math.PI) correction -= Math.PI * 2;
            else if (correction < -Math.PI) correction += Math.PI * 2;
            correction = clamp(correction, -maximumAngle, maximumAngle);
            if (Math.abs(correction) < 1e-6) continue;

            const anchorX = body.x[segment];
            const anchorY = body.y[segment];
            const anchorZ = body.z[segment];
            if (
                this.contactField &&
                body.collisionEndSegment >= body.collisionStartSegment
            ) {
                let accepted = false;
                let trialCorrection = correction;
                for (let attempt = 0; attempt < 6; attempt++) {
                    const trialCosine = Math.cos(trialCorrection);
                    const trialSine = Math.sin(trialCorrection);
                    const trialOneMinusCosine = 1 - trialCosine;
                    let fitsLumen = true;
                    for (
                        let contactSegment = Math.max(
                            segment,
                            body.collisionStartSegment
                        );
                        contactSegment <= Math.min(
                            body.activeEnd - 1,
                            body.collisionEndSegment
                        );
                        contactSegment++
                    ) {
                        for (let endpoint = 0; endpoint < 2; endpoint++) {
                            const node = contactSegment + endpoint;
                            const output = endpoint === 0
                                ? this._queryStart
                                : this._queryEnd;
                            if (node === segment) {
                                output.x = anchorX;
                                output.y = anchorY;
                                output.z = anchorZ;
                                continue;
                            }
                            const relativeX = body.x[node] - anchorX;
                            const relativeY = body.y[node] - anchorY;
                            const relativeZ = body.z[node] - anchorZ;
                            const axisDot =
                                relativeX * axisX +
                                relativeY * axisY +
                                relativeZ * axisZ;
                            output.x = anchorX +
                                relativeX * trialCosine +
                                (axisY * relativeZ - axisZ * relativeY) *
                                    trialSine +
                                axisX * axisDot * trialOneMinusCosine;
                            output.y = anchorY +
                                relativeY * trialCosine +
                                (axisZ * relativeX - axisX * relativeZ) *
                                    trialSine +
                                axisY * axisDot * trialOneMinusCosine;
                            output.z = anchorZ +
                                relativeZ * trialCosine +
                                (axisX * relativeY - axisY * relativeX) *
                                    trialSine +
                                axisZ * axisDot * trialOneMinusCosine;
                        }
                        const radius = Math.max(
                            body.nodeRadius[contactSegment],
                            body.nodeRadius[contactSegment + 1]
                        );
                        const contact = this.contactField.queryCapsule(
                            this._queryStart,
                            this._queryEnd,
                            radius,
                            this._contact
                        );
                        const allowedPenetration = Math.max(
                            0.01,
                            Number.isFinite(body.wallGap[contactSegment])
                                ? -body.wallGap[contactSegment] + 0.002
                                : 0
                        );
                        if (
                            contact.violation &&
                            contact.penetration > allowedPenetration
                        ) {
                            fitsLumen = false;
                            break;
                        }
                    }
                    if (fitsLumen) {
                        correction = trialCorrection;
                        accepted = true;
                        break;
                    }
                    trialCorrection *= 0.5;
                }
                if (!accepted) continue;
            }

            const rotationCosine = Math.cos(correction);
            const rotationSine = Math.sin(correction);
            const oneMinusCosine = 1 - rotationCosine;
            for (let node = next; node <= body.activeEnd; node++) {
                const relativeX = body.x[node] - anchorX;
                const relativeY = body.y[node] - anchorY;
                const relativeZ = body.z[node] - anchorZ;
                const axisDot =
                    relativeX * axisX +
                    relativeY * axisY +
                    relativeZ * axisZ;
                body.x[node] = anchorX +
                    relativeX * rotationCosine +
                    (axisY * relativeZ - axisZ * relativeY) * rotationSine +
                    axisX * axisDot * oneMinusCosine;
                body.y[node] = anchorY +
                    relativeY * rotationCosine +
                    (axisZ * relativeX - axisX * relativeZ) * rotationSine +
                    axisY * axisDot * oneMinusCosine;
                body.z[node] = anchorZ +
                    relativeZ * rotationCosine +
                    (axisX * relativeY - axisY * relativeX) * rotationSine +
                    axisZ * axisDot * oneMinusCosine;
            }
        }
    }

    #solveFoldLimits(body) {
        if (body.sleeping || body.count < 3 || body.foldLimitStrength <= 0) return;
        if (body.rodModel === 'kirchhoff') {
            this.#solveKirchhoffFoldLimits(body);
            return;
        }
        // A supporting sheath is allowed to impose its own curvature. The
        // anti-fold inequality belongs to the unsupported rod; solving it in
        // the curved supported section displaces the shared outlet node and
        // creates exactly the hinge it is intended to prevent.
        const unsupportedStart = Number.isFinite(body.sheathMaterialEndNode)
            ? Math.floor(body.sheathMaterialEndNode)
            : body.activeStart + 1;
        const start = Math.max(1, body.activeStart + 1, unsupportedStart);
        const end = Math.min(body.count - 1, body.activeEnd);
        const correctionX = body.foldCorrectionX;
        const correctionY = body.foldCorrectionY;
        const correctionZ = body.foldCorrectionZ;
        const correctionWeight = body.foldCorrectionWeight;
        for (let sweep = 0; sweep < 2; sweep++) {
            correctionX.fill(0, body.activeStart, body.activeEnd + 1);
            correctionY.fill(0, body.activeStart, body.activeEnd + 1);
            correctionZ.fill(0, body.activeStart, body.activeEnd + 1);
            correctionWeight.fill(0, body.activeStart, body.activeEnd + 1);
            let correctionCount = 0;
        for (let index = start; index < end; index++) {
            const limit = clamp(body.maxBendAngleByNode[index], 1, 179) * Math.PI / 180;
            const previous = index - 1;
            const next = index + 1;
            let incomingX = body.x[index] - body.x[previous];
            let incomingY = body.y[index] - body.y[previous];
            let incomingZ = body.z[index] - body.z[previous];
            let outgoingX = body.x[next] - body.x[index];
            let outgoingY = body.y[next] - body.y[index];
            let outgoingZ = body.z[next] - body.z[index];
            const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
            const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
            if (incomingLength < EPSILON || outgoingLength < EPSILON) continue;
            incomingX /= incomingLength;
            incomingY /= incomingLength;
            incomingZ /= incomingLength;
            outgoingX /= outgoingLength;
            outgoingY /= outgoingLength;
            outgoingZ /= outgoingLength;
            const dot = clamp(
                incomingX * outgoingX + incomingY * outgoingY + incomingZ * outgoingZ
            , -1, 1);
            const angle = Math.acos(dot);

            // A newly exposed material interval can be shorter than the
            // nominal discretisation length. Applying the regular fold limit
            // to that temporary joint magnifies a tiny excess by 1 / length
            // and can kick the neighbouring full segment. The elastic rod
            // energy remains active there; only the emergency inequality is
            // relaxed by a small, bounded angular guard until the interval
            // grows to a regular material segment.
            const fractionalFeedJoint = Math.min(
                body.restLength[previous],
                body.restLength[index]
            ) < body.segmentLength * 0.8;
            const effectiveLimit = limit;
            if (angle <= effectiveLimit) continue;
            correctionCount++;

            let axisX = incomingY * outgoingZ - incomingZ * outgoingY;
            let axisY = incomingZ * outgoingX - incomingX * outgoingZ;
            let axisZ = incomingX * outgoingY - incomingY * outgoingX;
            let axisLength = magnitude3(axisX, axisY, axisZ);
            if (axisLength < EPSILON && body.intrinsicBendEnabled[index]) {
                axisX = body.restDirectionAxisX[index];
                axisY = body.restDirectionAxisY[index];
                axisZ = body.restDirectionAxisZ[index];
                axisLength = magnitude3(axisX, axisY, axisZ);
            }
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
                axisLength = magnitude3(axisX, axisY, axisZ);
            }
            axisX /= axisLength;
            axisY /= axisLength;
            axisZ /= axisLength;

            const gradientPreviousX = (axisY * incomingZ - axisZ * incomingY) /
                incomingLength;
            const gradientPreviousY = (axisZ * incomingX - axisX * incomingZ) /
                incomingLength;
            const gradientPreviousZ = (axisX * incomingY - axisY * incomingX) /
                incomingLength;
            const gradientNextX = (axisY * outgoingZ - axisZ * outgoingY) /
                outgoingLength;
            const gradientNextY = (axisZ * outgoingX - axisX * outgoingZ) /
                outgoingLength;
            const gradientNextZ = (axisX * outgoingY - axisY * outgoingX) /
                outgoingLength;
            const gradientJointX = -gradientPreviousX - gradientNextX;
            const gradientJointY = -gradientPreviousY - gradientNextY;
            const gradientJointZ = -gradientPreviousZ - gradientNextZ;
            const previousWeight = body.inverseMass[previous];
            const jointWeight = body.inverseMass[index];
            const nextWeight = body.inverseMass[next];
            const denominator =
                previousWeight * magnitude3(
                    gradientPreviousX,
                    gradientPreviousY,
                    gradientPreviousZ
                ) ** 2 +
                jointWeight * magnitude3(
                    gradientJointX,
                    gradientJointY,
                    gradientJointZ
                ) ** 2 +
                nextWeight * magnitude3(
                    gradientNextX,
                    gradientNextY,
                    gradientNextZ
                ) ** 2;
            if (denominator < EPSILON) continue;
            let delta = -(angle - effectiveLimit) * body.foldLimitStrength / denominator;
            const maximumDisplacement = Math.max(
                previousWeight * magnitude3(
                    gradientPreviousX,
                    gradientPreviousY,
                    gradientPreviousZ
                ),
                jointWeight * magnitude3(
                    gradientJointX,
                    gradientJointY,
                    gradientJointZ
                ),
                nextWeight * magnitude3(
                    gradientNextX,
                    gradientNextY,
                    gradientNextZ
                )
            ) * Math.abs(delta);
            const displacementLimit = Math.min(incomingLength, outgoingLength) *
                (fractionalFeedJoint ? 0.16 : 0.35);
            if (maximumDisplacement > displacementLimit) {
                delta *= displacementLimit / maximumDisplacement;
            }
            if (previousWeight > 0) {
                correctionX[previous] += gradientPreviousX * delta * previousWeight;
                correctionY[previous] += gradientPreviousY * delta * previousWeight;
                correctionZ[previous] += gradientPreviousZ * delta * previousWeight;
                correctionWeight[previous]++;
            }
            if (jointWeight > 0) {
                correctionX[index] += gradientJointX * delta * jointWeight;
                correctionY[index] += gradientJointY * delta * jointWeight;
                correctionZ[index] += gradientJointZ * delta * jointWeight;
                correctionWeight[index]++;
            }
            if (nextWeight > 0) {
                correctionX[next] += gradientNextX * delta * nextWeight;
                correctionY[next] += gradientNextY * delta * nextWeight;
                correctionZ[next] += gradientNextZ * delta * nextWeight;
                correctionWeight[next]++;
            }
        }
            if (correctionCount === 0) break;
            for (let index = body.activeStart; index <= body.activeEnd; index++) {
                const weight = correctionWeight[index];
                if (weight <= 0) continue;
                body.x[index] += correctionX[index] / weight;
                body.y[index] += correctionY[index] / weight;
                body.z[index] += correctionZ[index] / weight;
            }
        }
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

    #solveStableFoldLimits(body) {
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
                const strength = Math.min(0.72, body.foldLimitStrength * 0.62);
                body.x[index] += ((body.x[previous] + body.x[next]) * 0.5 - body.x[index]) * strength;
                body.y[index] += ((body.y[previous] + body.y[next]) * 0.5 - body.y[index]) * strength;
                body.z[index] += ((body.z[previous] + body.z[next]) * 0.5 - body.z[index]) * strength;
                continue;
            }
            let chordX = body.x[next] - body.x[previous];
            let chordY = body.y[next] - body.y[previous];
            let chordZ = body.z[next] - body.z[previous];
            const chordLength = magnitude3(chordX, chordY, chordZ);
            if (chordLength < EPSILON) continue;
            chordX /= chordLength;
            chordY /= chordLength;
            chordZ /= chordLength;
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
                const strength = body.foldLimitStrength * 0.45;
                body.x[index] += ((body.x[previous] + body.x[next]) * 0.5 - body.x[index]) * strength;
                body.y[index] += ((body.y[previous] + body.y[next]) * 0.5 - body.y[index]) * strength;
                body.z[index] += ((body.z[previous] + body.z[next]) * 0.5 - body.z[index]) * strength;
                if (dot < -0.999 && chordLength < Math.min(incomingLength, outgoingLength) * 0.1) {
                    const nx = incomingX / incomingLength;
                    const ny = incomingY / incomingLength;
                    const nz = incomingZ / incomingLength;
                    let bendX = Math.abs(nx) < 0.8 ? 0 : -nz;
                    let bendY = Math.abs(nx) < 0.8 ? nz : 0;
                    let bendZ = Math.abs(nx) < 0.8 ? -ny : nx;
                    const bendLength = magnitude3(bendX, bendY, bendZ) || 1;
                    const nudge = Math.min(incomingLength, outgoingLength) * body.foldLimitStrength * 0.05;
                    body.x[index] += bendX / bendLength * nudge;
                    body.y[index] += bendY / bendLength * nudge;
                    body.z[index] += bendZ / bendLength * nudge;
                }
            }
        }
    }

    #captureContainmentOuterPose(constraint) {
        constraint.outerPostX.set(constraint.outerBody.x);
        constraint.outerPostY.set(constraint.outerBody.y);
        constraint.outerPostZ.set(constraint.outerBody.z);
    }

    #carryContainedInnerWithOuter(constraint) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerStart = clamp(constraint.startNode, inner.activeStart, inner.activeEnd);
        const innerEnd = clamp(constraint.endNode, innerStart, inner.activeEnd);
        const outerStart = clamp(constraint.outerStartNode, outer.activeStart, outer.activeEnd);
        const outerEnd = Math.min(outer.activeEnd, outer.segmentCount);
        const retainedProjection = clamp(outer.projectionVelocityRetention, 0, 1);
        for (let innerIndex = innerStart; innerIndex <= innerEnd; innerIndex++) {
            if (inner.inverseMass[innerIndex] <= 0) continue;
            const segment = constraint.closestSegment[innerIndex];
            if (segment < outerStart || segment >= outerEnd) continue;
            const t = clamp(constraint.closestT[innerIndex], 0, 1);
            const w0 = 1 - t;
            const w1 = t;
            const dx =
                (outer.x[segment] - constraint.outerPostX[segment]) * w0 +
                (outer.x[segment + 1] - constraint.outerPostX[segment + 1]) * w1;
            const dy =
                (outer.y[segment] - constraint.outerPostY[segment]) * w0 +
                (outer.y[segment + 1] - constraint.outerPostY[segment + 1]) * w1;
            const dz =
                (outer.z[segment] - constraint.outerPostZ[segment]) * w0 +
                (outer.z[segment + 1] - constraint.outerPostZ[segment + 1]) * w1;
            if (Math.abs(dx) + Math.abs(dy) + Math.abs(dz) < EPSILON) continue;
            inner.x[innerIndex] += dx;
            inner.y[innerIndex] += dy;
            inner.z[innerIndex] += dz;
            // Match the outer body's projection retention. A wall correction
            // that is intentionally overdamped must carry the wire without
            // becoming a full-strength inertial impulse in the next step.
            const previousBlend = 1 - retainedProjection;
            inner.previousX[innerIndex] += dx * previousBlend;
            inner.previousY[innerIndex] += dy * previousBlend;
            inner.previousZ[innerIndex] += dz * previousBlend;
        }
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
        normalZ
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
        const gap = clearance - radialDistance;
        const innerWeight0 = 1 - innerT;
        const outerWeight0 = 1 - outerT;
        record.kind = kind;
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
        const cached = record.cachedManifoldContact;
        const cachedBelongsToManifold =
            cached?._manifold === constraint.manifold;
        const hasStoredImpulse = cachedBelongsToManifold && (
            cached.normalLambda !== 0 ||
            cached.tangentLambda[0] !== 0 ||
            cached.tangentLambda[1] !== 0 ||
            cached.twistLambda !== 0
        );
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
        if (gap <= constraint.kirchhoffContactActivation) {
            const upsertOptions = constraint._kirchhoffUpsertOptions ??= {};
            upsertOptions.id = record.id;
            upsertOptions.innerMaterialSegmentId = innerMaterialSegmentId;
            upsertOptions.outerMaterialSegmentId = outerMaterialSegmentId;
            upsertOptions.feature = feature;
            upsertOptions.innerSegmentIndex = innerSegment;
            upsertOptions.outerSegmentIndex = outerSegment;
            upsertOptions.normal = record.normal;
            upsertOptions.tangentU = constraint._kirchhoffRuntimeAxis;
            upsertOptions.effectiveTwistRadius = radialDistance;
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
        openDistal
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
        constraint._kirchhoffSideFeature ??=
            `${constraint._kirchhoffFeaturePrefix}:side`;
        constraint._kirchhoffRimFeature ??=
            `${constraint._kirchhoffFeaturePrefix}:distal-rim`;
        const records = constraint._kirchhoffRuntimeRecords ??=
            new Array(5);
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
        let fallbackNormalX = 1;
        let fallbackNormalY = 0;
        let fallbackNormalZ = 0;
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
                    (pointZ - outerEndZ) * axisZ >= -1e-9
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
        if (worstGap <= constraint.kirchhoffContactActivation) {
            constraint._kirchhoffRuntimeAxis[0] = worstAxisX;
            constraint._kirchhoffRuntimeAxis[1] = worstAxisY;
            constraint._kirchhoffRuntimeAxis[2] = worstAxisZ;
            records[0] = this.#kirchhoffRuntimeContactRecord(
                constraint,
                innerSegment,
                worstOuterSegment,
                0,
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
                const gap = clearance - radius;
                if (gap <= constraint.kirchhoffContactActivation) {
                    const inverseRadius = radius > EPSILON ? 1 / radius : 0;
                    records[1] = this.#kirchhoffRuntimeContactRecord(
                        constraint,
                        innerSegment,
                        outerSegment,
                        1,
                        'distal-rim',
                        constraint._kirchhoffRimFeature,
                        innerT,
                        1,
                        radius,
                        clearance,
                        radius > EPSILON ? radialX * inverseRadius : fallbackNormalX,
                        radius > EPSILON ? radialY * inverseRadius : fallbackNormalY,
                        radius > EPSILON ? radialZ * inverseRadius : fallbackNormalZ
                    );
                }
            }
        }
        return records;
    }

    #kirchhoffClosestOuterSegment(
        constraint,
        innerSegment,
        expectedOuterSegment,
        minimumOuterSegment,
        maximumOuterSegment
    ) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const firstOuter = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            Math.max(outer.activeStart, outer.activeEnd - 1)
        );
        const lastOuter = Math.min(outer.segmentCount - 1, outer.activeEnd - 1);
        if (lastOuter < firstOuter) return null;

        const topologicalFirst = clamp(
            minimumOuterSegment,
            firstOuter,
            lastOuter
        );
        const topologicalLast = clamp(
            maximumOuterSegment,
            topologicalFirst,
            lastOuter
        );
        const expected = clamp(
            expectedOuterSegment,
            topologicalFirst,
            topologicalLast
        );
        const cached = constraint.kirchhoffOuterSegmentByInner[innerSegment];
        const hasCache = cached >= firstOuter && cached <= lastOuter;
        if (
            constraint._kirchhoffMappingLocked &&
            hasCache &&
            cached >= topologicalFirst &&
            cached <= topologicalLast
        ) {
            const closest = constraint._kirchhoffClosestResult ??= {};
            closestRodSegmentParameters(
                inner,
                innerSegment,
                outer,
                cached,
                closest
            );
            const result = constraint._kirchhoffClosestMapping ??= {
                outerSegment: -1,
                closest
            };
            result.outerSegment = cached;
            result.closest = closest;
            return result;
        }
        const window = Math.max(1, Math.floor(constraint.searchWindow));
        // A lumen is a material tube, not an unordered bag of line segments.
        // Once the centerline bends back near itself, an unrestricted nearest
        // search can jump to a remote catheter segment and create a fictitious
        // cross-link.  Continue from the material arc-length prediction and a
        // persistent local cache, while preserving the monotonic order of the
        // two rods.  Axial sliding remains free inside this local band; only a
        // topologically impossible branch jump is excluded.
        const cacheCenter = hasCache
            ? clamp(cached, expected - window, expected + window)
            : expected;
        const searchStart = Math.max(
            topologicalFirst,
            Math.min(expected, cacheCenter) - window
        );
        const searchEnd = Math.min(
            topologicalLast,
            Math.max(expected, cacheCenter) + window
        );
        let bestOuterSegment = -1;
        let bestDistanceSquared = Infinity;
        let bestFirstT = 0;
        let bestSecondT = 0;
        const candidate = constraint._kirchhoffClosestCandidate ??= {};
        const scan = (start, end) => {
            for (let outerSegment = start; outerSegment <= end; outerSegment++) {
                closestRodSegmentParameters(
                    inner,
                    innerSegment,
                    outer,
                    outerSegment,
                    candidate
                );
                if (
                    candidate.distanceSquared < bestDistanceSquared ||
                    (
                        candidate.distanceSquared === bestDistanceSquared &&
                        outerSegment < bestOuterSegment
                    )
                ) {
                    bestOuterSegment = outerSegment;
                    bestDistanceSquared = candidate.distanceSquared;
                    bestFirstT = candidate.firstT;
                    bestSecondT = candidate.secondT;
                }
            }
        };
        scan(searchStart, searchEnd);
        if (bestOuterSegment >= 0) {
            constraint.kirchhoffOuterSegmentByInner[innerSegment] =
                bestOuterSegment;
            const closest = constraint._kirchhoffClosestResult ??= {};
            closest.firstT = bestFirstT;
            closest.secondT = bestSecondT;
            closest.distance = Math.sqrt(bestDistanceSquared);
            const result = constraint._kirchhoffClosestMapping ??= {
                outerSegment: -1,
                closest
            };
            result.outerSegment = bestOuterSegment;
            return result;
        }
        return null;
    }

    #kirchhoffContactWeight(record, inner, outer) {
        const innerSegment = record.manifoldContact.innerSegmentIndex;
        const outerSegment = record.manifoldContact.outerSegmentIndex;
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
        for (let endpoint = 0; endpoint < 2; endpoint++) {
            const innerNode = innerSegment + endpoint;
            const outerNode = outerSegment + endpoint;
            const innerScale = inner.inverseMass[innerNode] *
                record.innerWeights[endpoint] * lambda * innerSign;
            const outerScale = outer.inverseMass[outerNode] *
                record.outerWeights[endpoint] * lambda * -innerSign;
            inner.x[innerNode] += vector[0] * innerScale;
            inner.y[innerNode] += vector[1] * innerScale;
            inner.z[innerNode] += vector[2] * innerScale;
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

    #applyKirchhoffNormalCorrection(record, lambda) {
        if (Math.abs(lambda) < EPSILON) return;
        const contact = record.manifoldContact;
        const inner = record._innerBody;
        const outer = record._outerBody;
        const innerSegment = contact.innerSegmentIndex;
        const outerSegment = contact.outerSegmentIndex;
        for (let endpoint = 0; endpoint < 2; endpoint++) {
            const innerNode = innerSegment + endpoint;
            const outerNode = outerSegment + endpoint;
            const innerWeight = -record.innerWeights[endpoint];
            const outerWeight = record.outerWeights[endpoint];
            const innerGradientX = record.normal[0] * innerWeight;
            const innerGradientY = record.normal[1] * innerWeight;
            const innerGradientZ = record.normal[2] * innerWeight;
            const outerGradientX = record.normal[0] * outerWeight;
            const outerGradientY = record.normal[1] * outerWeight;
            const outerGradientZ = record.normal[2] * outerWeight;
            const innerScale = inner.inverseMass[innerNode] * lambda;
            const outerScale = outer.inverseMass[outerNode] * lambda;
            inner.x[innerNode] += innerGradientX * innerScale;
            inner.y[innerNode] += innerGradientY * innerScale;
            inner.z[innerNode] += innerGradientZ * innerScale;
            outer.x[outerNode] += outerGradientX * outerScale;
            outer.y[outerNode] += outerGradientY * outerScale;
            outer.z[outerNode] += outerGradientZ * outerScale;
        }
        if (inner.sleeping) inner.wake();
        if (outer.sleeping) outer.wake();
    }

    #kirchhoffFrameIncrement(body, segment, out) {
        if (body.rodModel !== 'kirchhoff') {
            out[0] = 0;
            out[1] = 0;
            out[2] = 0;
            return out;
        }
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
            body.rodModel !== 'kirchhoff' ||
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

    #solveKirchhoffLumenRecord(constraint, record, applyFriction) {
        const contact = record.manifoldContact;
        if (!contact) return;
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        record._innerBody = inner;
        record._outerBody = outer;
        const weight = this.#kirchhoffContactWeight(record, inner, outer);
        const alpha = Math.max(0, constraint.compliance) /
            (this.fixedDt * this.fixedDt);
        const previousU = contact.tangentLambda[0];
        const previousV = contact.tangentLambda[1];
        const previousTwist = contact.twistLambda;
        if (weight + alpha > EPSILON) {
            const requested = (-record.gap - alpha * contact.normalLambda) /
                (weight + alpha);
            const applied = constraint.manifold.accumulateKnownNormalLambda(
                contact,
                requested
            );
            this.#applyKirchhoffNormalCorrection(record, applied);
        }

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
            constraint.friction <= 0 ||
            weight <= EPSILON
        ) {
            return;
        }

        const innerSegment = contact.innerSegmentIndex;
        const outerSegment = contact.outerSegmentIndex;
        let displacementX = 0;
        let displacementY = 0;
        let displacementZ = 0;
        for (let endpoint = 0; endpoint < 2; endpoint++) {
            const innerNode = innerSegment + endpoint;
            const outerNode = outerSegment + endpoint;
            const innerWeight = record.innerWeights[endpoint];
            const outerWeight = record.outerWeights[endpoint];
            displacementX += (
                inner.x[innerNode] - inner.previousX[innerNode]
            ) * innerWeight - (
                outer.x[outerNode] - outer.previousX[outerNode]
            ) * outerWeight;
            displacementY += (
                inner.y[innerNode] - inner.previousY[innerNode]
            ) * innerWeight - (
                outer.y[outerNode] - outer.previousY[outerNode]
            ) * outerWeight;
            displacementZ += (
                inner.z[innerNode] - inner.previousZ[innerNode]
            ) * innerWeight - (
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
        contactScratch.tangentOptions.frictionCoefficient = constraint.friction;
        contactScratch.tangentOptions.out = contactScratch.tangentResult;
        const friction = constraint.manifold.accumulateKnownTangentialLambda(
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

        const innerInverseInertia = inner.rodModel === 'kirchhoff'
            ? inner.inverseInertia3[innerSegment]
            : 0;
        const outerInverseInertia = outer.rodModel === 'kirchhoff'
            ? outer.inverseInertia3[outerSegment]
            : 0;
        const angularWeight = innerInverseInertia + outerInverseInertia;
        if (angularWeight <= EPSILON) return;
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
        contactScratch.twistOptions.frictionCoefficient = constraint.friction;
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

    #measureKirchhoffContainmentViolation(constraint) {
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
            Math.max(innerStart, constraint.endNode - 1)
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
                if (rawOuterT < -1e-9 || rawOuterT > 1 + 1e-9) continue;
                const outerT = clamp(rawOuterT, 0, 1);
                const radialX = pointX -
                    (outerStartX + outerDirectionX * outerT);
                const radialY = pointY -
                    (outerStartY + outerDirectionY * outerT);
                const radialZ = pointZ -
                    (outerStartZ + outerDirectionZ * outerT);
                maximumViolation = Math.max(
                    maximumViolation,
                    magnitude3(radialX, radialY, radialZ) - clearance
                );
            }
            if (!openDistal) continue;
            const outerEndX = outer.x[outerSegment + 1];
            const outerEndY = outer.y[outerSegment + 1];
            const outerEndZ = outer.z[outerSegment + 1];
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
                ) - clearance
            );
        }
        return Math.max(0, maximumViolation);
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
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            Math.max(inner.activeStart, inner.activeEnd - 1)
        );
        const innerEnd = Math.min(
            inner.segmentCount - 1,
            inner.activeEnd - 1,
            Math.max(innerStart, constraint.endNode - 1)
        );
        const outerLast = Math.min(outer.segmentCount - 1, outer.activeEnd - 1);
        if (innerEnd < innerStart || outerLast < outer.activeStart) return;

        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            outerLast
        );
        let expectedOuterSegment = outerStart;
        let outerArcEnd = Math.max(
            EPSILON,
            outer.restLength[expectedOuterSegment]
        );
        let innerArcStart = Math.max(0, constraint.innerArcOffset);
        let previousOuterSegment = outerStart;

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
            let minimumOuterRestLength = Infinity;
            const localOuterStart = Math.max(
                outerStart,
                expectedOuterSegment - 1
            );
            const localOuterEnd = Math.min(
                outerLast,
                expectedOuterSegment + 1
            );
            for (
                let outerSegment = localOuterStart;
                outerSegment <= localOuterEnd;
                outerSegment++
            ) {
                minimumOuterRestLength = Math.min(
                    minimumOuterRestLength,
                    Math.max(EPSILON, outer.restLength[outerSegment])
                );
            }
            const maximumAdvance = Math.max(
                1,
                Math.ceil(innerRestLength / minimumOuterRestLength) + 1
            );
            const mapping = this.#kirchhoffClosestOuterSegment(
                constraint,
                innerSegment,
                expectedOuterSegment,
                previousOuterSegment,
                previousOuterSegment + maximumAdvance
            );
            innerArcStart += innerRestLength;
            if (!mapping) continue;
            const outerSegment = mapping.outerSegment;
            previousOuterSegment = outerSegment;
            constraint.closestSegment[innerSegment] = outerSegment;
            constraint.closestT[innerSegment] = mapping.closest.secondT;
            const records = this.#kirchhoffRuntimeLumenRecords(
                constraint,
                innerSegment,
                outerSegment,
                constraint.openDistal && outerSegment === outerLast
            );
            if (!records) continue;
            for (const record of records) {
                if (!record?.manifoldContact) continue;
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
        constraint._kirchhoffMappingLocked = true;
        if (measureResidual) {
            constraint.kirchhoffMaxViolation =
                this.#measureKirchhoffContainmentViolation(constraint);
        }
    }

    #solveContainment(
        constraint,
        innerOnly = false,
        outerOnly = false,
        applyFriction = true
    ) {
        if (constraint.model === 'kirchhoff') {
            this.#solveKirchhoffContainment(
                constraint,
                applyFriction,
                false
            );
            return;
        }
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
            constraint.closestT.fill(0);
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
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
        const settledContainmentGuard = constraint.limitDistalCorrection ? 0 : 0.004;
        const allowedRadius = Math.max(
            0,
            constraint.innerRadius - inner.radius - settledContainmentGuard
        );
        // When a catheter advances over a held guidewire, lumen contact and
        // wire length are a compliant force balance. Treating both as exact
        // projections produces an unsatisfiable two-cycle at coarse polyline
        // joints: length restores the metal wire, then a hard capsule snap
        // stretches it again. A finite XPBD compliance is active only for
        // this moving stationary-rail state; ordinary settled containment
        // remains exact.
        const containmentCompliance =
            constraint.limitDistalCorrection &&
            constraint.preserveStationaryInnerLength
                ? Math.max(constraint.compliance, 5e-5)
                : constraint.compliance;
        const alpha = containmentCompliance / (this.fixedDt * this.fixedDt);
        const outerStart = clamp(constraint.outerStartNode, outer.activeStart, outer.activeEnd);
        const outerEnd = Math.min(outer.activeEnd, outer.segmentCount);
        if (outerEnd <= outerStart) return;
        const nearDistalOpening =
            constraint.openDistal &&
            this.#containmentPortalRetraction(constraint) <=
                constraint.portalTransitionLength;
        const innerStart = clamp(constraint.startNode, inner.activeStart, inner.activeEnd);
        const configuredInnerEnd = clamp(
            constraint.endNode,
            innerStart,
            inner.activeEnd
        );
        const hasExternalDistalMaterial = configuredInnerEnd < inner.activeEnd;
        const portalNearOpening =
            constraint.enforceDistalPortal &&
            nearDistalOpening &&
            hasExternalDistalMaterial;
        // The first node nominally outside an open catheter is still part of
        // the lumen-to-free-space transition. Constrain it while it remains
        // proximal to the distal plane, then let it pass through the opening.
        // Without this extra node, the crossing segment can leave through the
        // side wall even though both cached endpoint classifications are valid.
        const innerEnd = portalNearOpening
            ? Math.min(inner.activeEnd, configuredInnerEnd + 1)
            : configuredInnerEnd;
        let expected = outerStart;
        let previousBestSegment = -1;
        let previousBestT = 0;
        // The first discrete wire node is generally a fractional segment past
        // the catheter's lumen entrance. Honour that material offset so the
        // expected outer segment advances continuously as proximal nodes enter
        // through the sheath; ignoring it made the nearest-segment window jump
        // backward on every guidewire remesh.
        let innerArcLength = Math.max(0, constraint.innerArcOffset);
        let outerArcEnd = outer.restLength[outerStart];
        for (let innerIndex = innerStart; innerIndex <= innerEnd; innerIndex++) {
            if (innerIndex > innerStart) innerArcLength += inner.restLength[innerIndex - 1];
            while (expected < outerEnd - 1 && outerArcEnd < innerArcLength) {
                expected++;
                outerArcEnd += outer.restLength[expected];
            }
            const portalSmoothingNodes = Math.max(1, Math.ceil(
                constraint.portalSmoothingLength /
                Math.max(EPSILON, inner.segmentLength)
            ));
            const distalMaterialMapping =
                constraint.limitDistalCorrection &&
                hasExternalDistalMaterial &&
                innerIndex >= configuredInnerEnd - portalSmoothingNodes;
            const materialMappedNode = distalMaterialMapping;
            const searchStart = materialMappedNode
                ? expected
                : Math.max(
                    outerStart,
                    previousBestSegment,
                    expected - constraint.searchWindow
                );
            const searchEnd = materialMappedNode
                ? expected
                : Math.min(
                    outerEnd - 1,
                    Math.max(searchStart, expected + constraint.searchWindow)
                );
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
                const segmentRestLength = Math.max(
                    EPSILON,
                    outer.restLength[segment]
                );
                const materialT = clamp(
                    (innerArcLength - (outerArcEnd - segmentRestLength)) /
                        segmentRestLength,
                    0,
                    1
                );
                const nearestT = clamp(
                    ((inner.x[innerIndex] - ax) * dx +
                        (inner.y[innerIndex] - ay) * dy +
                        (inner.z[innerIndex] - az) * dz) /
                        Math.max(EPSILON, lengthSq),
                    0,
                    1
                );
                let t = materialMappedNode
                    ? clamp(
                        nearestT,
                        Math.max(0, materialT - 0.2),
                        Math.min(1, materialT + 0.2)
                    )
                    : nearestT;
                // The wire is free to slide longitudinally, but its material
                // order cannot reverse inside one catheter segment. Clamp the
                // candidate only when it would cross the previous wire node;
                // all ordinary nearest-point motion remains purely radial.
                if (segment === previousBestSegment) {
                    t = Math.max(t, Math.min(1, previousBestT + 1e-3));
                }
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
            previousBestT = bestT;
            constraint.closestSegment[innerIndex] = bestSegment;
            constraint.closestT[innerIndex] = bestT;
            // The dedicated portal solver owns the crossing segment while the
            // wire is at the opening. Correcting its endpoint here as an
            // independent lumen node creates a large hinge one node proximal
            // before the distributed portal correction gets a chance to act.
            if (portalNearOpening && innerIndex >= configuredInnerEnd) continue;
            if (constraint.openProximal && bestSegment === outerStart) {
                const dx = outer.x[outerStart + 1] - outer.x[outerStart];
                const dy = outer.y[outerStart + 1] - outer.y[outerStart];
                const dz = outer.z[outerStart + 1] - outer.z[outerStart];
                const before =
                    (inner.x[innerIndex] - outer.x[outerStart]) * dx +
                    (inner.y[innerIndex] - outer.y[outerStart]) * dy +
                    (inner.z[innerIndex] - outer.z[outerStart]) * dz;
                // Geometry may lag the material feed by one solver sweep.
                // Only the actual boundary node is allowed to remain outside
                // an open proximal end, and only while its material offset
                // has not crossed that end yet. Once innerArcOffset is
                // positive the node is already inside the lumen and must not
                // escape merely because a bend placed it behind the plane.
                if (
                    before < 0 &&
                    innerIndex === innerStart &&
                    constraint.innerArcOffset <= EPSILON
                ) continue;
            }
            if (constraint.openDistal && bestSegment === outerEnd - 1) {
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
            const stationaryRail =
                !innerOnly &&
                !outerOnly &&
                constraint.limitDistalCorrection &&
                constraint.preserveStationaryInnerLength;
            const innerResponse = outerOnly
                ? 0
                : innerOnly
                    ? 1
                    : stationaryRail
                        ? 0.2
                        : constraint.innerResponse;
            const outerResponse = innerOnly
                ? 0
                : stationaryRail
                    ? 0.8
                    : constraint.outerResponse;
            const innerWeight = inner.inverseMass[innerIndex] * innerResponse;
            const w0Factor = 1 - bestT;
            const w1Factor = bestT;
            const outerInverseMass0 =
                (
                    !Number.isFinite(outer.sheathMaterialEndNode) ||
                    bestSegment > outer.sheathMaterialEndNode
                )
                    ? outer.inverseMass[bestSegment]
                    : 0;
            const outerInverseMass1 =
                (
                    !Number.isFinite(outer.sheathMaterialEndNode) ||
                    bestSegment + 1 > outer.sheathMaterialEndNode
                )
                    ? outer.inverseMass[bestSegment + 1]
                    : 0;
            const outerWeight0 =
                outerInverseMass0 * outerResponse * w0Factor * w0Factor;
            const outerWeight1 =
                outerInverseMass1 * outerResponse * w1Factor * w1Factor;
            const denominator = innerWeight + outerWeight0 + outerWeight1 + alpha;
            if (denominator < EPSILON) continue;
            const c = allowedRadius - distance;
            let deltaLambda = (-c - alpha * constraint.lambdas[innerIndex]) / denominator;
            if (
                constraint.outerResponse <= EPSILON &&
                outerResponse <= EPSILON &&
                innerResponse > EPSILON &&
                (
                    materialMappedNode ||
                    (portalNearOpening && innerIndex >= configuredInnerEnd - 1)
                ) &&
                innerWeight > EPSILON
            ) {
                const maximumDeltaLambda =
                    constraint.portalMaxCorrection / innerWeight;
                deltaLambda = clamp(
                    deltaLambda,
                    -maximumDeltaLambda,
                    maximumDeltaLambda
                );
            }
            constraint.lambdas[innerIndex] += deltaLambda;
            const innerCorrectionX = -radialX * deltaLambda * innerWeight;
            const innerCorrectionY = -radialY * deltaLambda * innerWeight;
            const innerCorrectionZ = -radialZ * deltaLambda * innerWeight;
            inner.x[innerIndex] += innerCorrectionX;
            inner.y[innerIndex] += innerCorrectionY;
            inner.z[innerIndex] += innerCorrectionZ;
            if (
                innerWeight > EPSILON &&
                Math.abs(innerCorrectionX) +
                    Math.abs(innerCorrectionY) +
                    Math.abs(innerCorrectionZ) > 1e-5
            ) {
                inner.wake();
            }
            if (
                constraint.outerResponse <= EPSILON &&
                outerResponse <= EPSILON &&
                innerResponse > EPSILON &&
                (distalMaterialMapping || portalNearOpening) &&
                innerIndex >= configuredInnerEnd - 1
            ) {
                const smoothingNodes = Math.max(1, Math.ceil(
                    constraint.portalSmoothingLength /
                    Math.max(EPSILON, inner.segmentLength)
                ));
                const firstSmoothedNode = Math.max(
                    innerStart,
                    innerIndex - smoothingNodes
                );
                for (let node = firstSmoothedNode; node < innerIndex; node++) {
                    if (inner.inverseMass[node] <= 0) continue;
                    const proximalOffset = innerIndex - node;
                    const taper = 1 - proximalOffset / (smoothingNodes + 1);
                    inner.x[node] += innerCorrectionX * taper;
                    inner.y[node] += innerCorrectionY * taper;
                    inner.z[node] += innerCorrectionZ * taper;
                }
                // When the catheter has not yet covered the whole guidewire,
                // the next wire node is outside the distal opening. Moving
                // only the final contained node makes that crossing segment
                // reverse direction. Carry the correction through the first
                // external segment, then fade it over the same material span.
                // When the outer catheter is advancing over a stationary
                // guidewire, transmit only a small share: full carry made a
                // newly covered node drag the entire free tip by more than one
                // 5 mm discretization interval in a single fixed step.
                if (
                    inner.activeEnd - innerIndex <= smoothingNodes
                ) {
                    const distalCarry =
                        constraint.preserveStationaryInnerLength &&
                        constraint.portalInnerResponse <= EPSILON
                            ? 0.2
                            : 1;
                    const lastSmoothedNode = Math.min(
                        inner.activeEnd,
                        innerIndex + smoothingNodes
                    );
                    for (let node = innerIndex + 1; node <= lastSmoothedNode; node++) {
                        if (inner.inverseMass[node] <= 0) continue;
                        const distalOffset = node - innerIndex;
                        const taper = 1 - Math.max(0, distalOffset - 1) / smoothingNodes;
                        inner.x[node] += innerCorrectionX * taper * distalCarry;
                        inner.y[node] += innerCorrectionY * taper * distalCarry;
                        inner.z[node] += innerCorrectionZ * taper * distalCarry;
                    }
                }
            }
            outer.x[bestSegment] +=
                radialX * deltaLambda * outerInverseMass0 * outerResponse * w0Factor;
            outer.y[bestSegment] +=
                radialY * deltaLambda * outerInverseMass0 * outerResponse * w0Factor;
            outer.z[bestSegment] +=
                radialZ * deltaLambda * outerInverseMass0 * outerResponse * w0Factor;
            outer.x[bestSegment + 1] +=
                radialX * deltaLambda * outerInverseMass1 * outerResponse * w1Factor;
            outer.y[bestSegment + 1] +=
                radialY * deltaLambda * outerInverseMass1 * outerResponse * w1Factor;
            outer.z[bestSegment + 1] +=
                radialZ * deltaLambda * outerInverseMass1 * outerResponse * w1Factor;
            if (outerResponse > EPSILON && Math.abs(deltaLambda) > 1e-5) {
                outer.wake();
            }
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
                    outerInverseMass0 * outerResponse * w0Factor;
                outer.y[bestSegment] -= tangentY * tangentLambda *
                    outerInverseMass0 * outerResponse * w0Factor;
                outer.z[bestSegment] -= tangentZ * tangentLambda *
                    outerInverseMass0 * outerResponse * w0Factor;
                outer.x[bestSegment + 1] -= tangentX * tangentLambda *
                    outerInverseMass1 * outerResponse * w1Factor;
                outer.y[bestSegment + 1] -= tangentY * tangentLambda *
                    outerInverseMass1 * outerResponse * w1Factor;
                outer.z[bestSegment + 1] -= tangentZ * tangentLambda *
                    outerInverseMass1 * outerResponse * w1Factor;
            }
        }
        if (constraint.enforceDistalPortal) {
            this.#solveDistalPortal(constraint, innerOnly, outerOnly);
        }
    }

    #solveDistalPortal(
        constraint,
        innerOnly = false,
        outerOnly = false
    ) {
        if (!constraint.enabled || !constraint.openDistal) {
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
            return;
        }
        if (
            innerOnly &&
            constraint.portalInnerResponse <= EPSILON &&
            constraint.portalOuterResponse > EPSILON
        ) {
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
            return;
        }
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerSegment = clamp(
            constraint.endNode,
            inner.activeStart,
            Math.min(inner.activeEnd - 1, inner.segmentCount - 1)
        );
        const outerTip = outer.activeEnd;
        if (innerSegment < inner.activeStart || outerTip <= outer.activeStart) {
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
            return;
        }

        let tipX = outer.x[outerTip];
        let tipY = outer.y[outerTip];
        let tipZ = outer.z[outerTip];
        let tangentX = tipX - outer.x[outerTip - 1];
        let tangentY = tipY - outer.y[outerTip - 1];
        let tangentZ = tipZ - outer.z[outerTip - 1];
        const tangentLength = magnitude3(tangentX, tangentY, tangentZ);
        if (tangentLength < EPSILON) {
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
            return;
        }
        tangentX /= tangentLength;
        tangentY /= tangentLength;
        tangentZ /= tangentLength;

        let aRelativeX = inner.x[innerSegment] - tipX;
        let aRelativeY = inner.y[innerSegment] - tipY;
        let aRelativeZ = inner.z[innerSegment] - tipZ;
        let bRelativeX = inner.x[innerSegment + 1] - tipX;
        let bRelativeY = inner.y[innerSegment + 1] - tipY;
        let bRelativeZ = inner.z[innerSegment + 1] - tipZ;
        let aAxial =
            aRelativeX * tangentX +
            aRelativeY * tangentY +
            aRelativeZ * tangentZ;
        let bAxial =
            bRelativeX * tangentX +
            bRelativeY * tangentY +
            bRelativeZ * tangentZ;
        let axialDelta = bAxial - aAxial;
        let segmentT = Math.abs(axialDelta) > EPSILON
            ? clamp(-aAxial / axialDelta, 0, 1)
            : Math.abs(aAxial) <= Math.abs(bAxial) ? 0 : 1;
        let crossingX =
            aRelativeX + (bRelativeX - aRelativeX) * segmentT;
        let crossingY =
            aRelativeY + (bRelativeY - aRelativeY) * segmentT;
        let crossingZ =
            aRelativeZ + (bRelativeZ - aRelativeZ) * segmentT;
        let crossingAxial =
            crossingX * tangentX +
            crossingY * tangentY +
            crossingZ * tangentZ;
        const axialDistance = Math.abs(crossingAxial);
        const spatialTransitionRatio = clamp(
            1 - axialDistance / constraint.portalTransitionLength,
            0,
            1
        );
        const materialRetraction = this.#containmentPortalRetraction(constraint);
        const materialTransitionRatio = clamp(
            1 - Math.max(0, materialRetraction) / constraint.portalTransitionLength,
            0,
            1
        );
        const transitionRatio = Math.min(
            spatialTransitionRatio,
            materialTransitionRatio
        );
        const activation = transitionRatio * transitionRatio * (3 - 2 * transitionRatio);
        if (activation <= EPSILON) {
            constraint.portalLambda = 0;
            constraint.portalDirectionLambda = 0;
            return;
        }
        const allowedRadius = Math.max(0, constraint.innerRadius - inner.radius);

        // The lumen behind the distal aperture acts as a short physical
        // bearing. It transfers a bending moment to the material segment that
        // is crossing the opening, but it does not prescribe any position for
        // the free guidewire beyond that segment. Solving this angular contact
        // before the radial aperture contact prevents a centered crossing from
        // retaining an arbitrary, visibly separated direction.
        this.#solveDistalPortalDirection(
            constraint,
            innerSegment,
            tangentX,
            tangentY,
            tangentZ,
            activation,
            allowedRadius,
            { innerOnly, outerOnly }
        );

        // The angular solve can move either the free wire or the catheter's
        // distal bearing, depending on which material boundary is driving.
        // Refresh both the opening frame and its intersection before applying
        // radial aperture contact.
        tipX = outer.x[outerTip];
        tipY = outer.y[outerTip];
        tipZ = outer.z[outerTip];
        tangentX = tipX - outer.x[outerTip - 1];
        tangentY = tipY - outer.y[outerTip - 1];
        tangentZ = tipZ - outer.z[outerTip - 1];
        const refreshedTangentLength = magnitude3(
            tangentX,
            tangentY,
            tangentZ
        );
        if (refreshedTangentLength < EPSILON) {
            constraint.portalLambda = 0;
            return;
        }
        tangentX /= refreshedTangentLength;
        tangentY /= refreshedTangentLength;
        tangentZ /= refreshedTangentLength;
        aRelativeX = inner.x[innerSegment] - tipX;
        aRelativeY = inner.y[innerSegment] - tipY;
        aRelativeZ = inner.z[innerSegment] - tipZ;
        bRelativeX = inner.x[innerSegment + 1] - tipX;
        bRelativeY = inner.y[innerSegment + 1] - tipY;
        bRelativeZ = inner.z[innerSegment + 1] - tipZ;
        aAxial =
            aRelativeX * tangentX +
            aRelativeY * tangentY +
            aRelativeZ * tangentZ;
        bAxial =
            bRelativeX * tangentX +
            bRelativeY * tangentY +
            bRelativeZ * tangentZ;
        axialDelta = bAxial - aAxial;
        if (Math.abs(axialDelta) < EPSILON) {
            constraint.portalLambda = 0;
            return;
        }
        segmentT = clamp(-aAxial / axialDelta, 0, 1);
        const aWeight = 1 - segmentT;
        const bWeight = segmentT;
        crossingX = aRelativeX + (bRelativeX - aRelativeX) * segmentT;
        crossingY = aRelativeY + (bRelativeY - aRelativeY) * segmentT;
        crossingZ = aRelativeZ + (bRelativeZ - aRelativeZ) * segmentT;
        crossingAxial =
            crossingX * tangentX +
            crossingY * tangentY +
            crossingZ * tangentZ;
        let radialX = crossingX - tangentX * crossingAxial;
        let radialY = crossingY - tangentY * crossingAxial;
        let radialZ = crossingZ - tangentZ * crossingAxial;
        const radialDistance = magnitude3(radialX, radialY, radialZ);
        if (radialDistance <= allowedRadius || radialDistance < EPSILON) {
            constraint.portalLambda *= 0.5;
            return;
        }
        radialX /= radialDistance;
        radialY /= radialDistance;
        radialZ /= radialDistance;

        // In catheter-dominant coupling, translating only the crossing
        // segment makes the guidewire form a hinge immediately proximal to the
        // catheter tip. Move the crossing segment as a rigid pair and taper
        // that translation over a short proximal span. This preserves segment
        // length at the opening and distributes the required bend instead of
        // creating a one-node fold.
        if (
            !outerOnly &&
            constraint.portalInnerResponse > EPSILON &&
            constraint.portalOuterResponse <= EPSILON
        ) {
            const correctionDistance = Math.min(
                constraint.portalMaxCorrection,
                Math.max(0, radialDistance - allowedRadius) * activation
            );
            const smoothingNodes = Math.max(1, Math.ceil(
                constraint.portalSmoothingLength /
                Math.max(EPSILON, inner.segmentLength)
            ));
            const firstNode = Math.max(
                inner.activeStart,
                innerSegment - smoothingNodes
            );
            for (let node = firstNode; node <= inner.activeEnd; node++) {
                if (inner.inverseMass[node] <= 0) continue;
                const proximalOffset = Math.max(0, innerSegment - node);
                const taper = proximalOffset <= 0
                    ? 1
                    : 1 - proximalOffset / (smoothingNodes + 1);
                inner.x[node] -= radialX * correctionDistance * taper;
                inner.y[node] -= radialY * correctionDistance * taper;
                inner.z[node] -= radialZ * correctionDistance * taper;
            }
            if (correctionDistance > 1e-5) inner.wake();
            constraint.portalLambda = 0;
            return;
        }
        if (
            !innerOnly &&
            constraint.portalOuterResponse > EPSILON &&
            constraint.portalInnerResponse <= EPSILON
        ) {
            const correctionDistance = Math.min(
                constraint.portalMaxCorrection,
                Math.max(0, radialDistance - allowedRadius) * activation
            ) * constraint.portalOuterResponse;
            const smoothingNodes = Math.max(1, Math.ceil(
                constraint.portalSmoothingLength /
                Math.max(EPSILON, outer.segmentLength)
            ));
            const firstNode = Math.max(
                outer.activeStart,
                outerTip - smoothingNodes
            );
            const previousBlend = constraint.limitDistalCorrection
                ? constraint.preserveStationaryInnerLength
                    ? Math.min(
                        0.05,
                        Math.max(
                            0,
                            constraint.portalRetractionDistance ?? 0
                        ) * 0.05
                    )
                    : 1 - clamp(outer.projectionVelocityRetention, 0, 1)
                : 1;
            for (let node = firstNode; node <= outerTip; node++) {
                if (outer.inverseMass[node] <= 0) continue;
                const proximalOffset = outerTip - node;
                const taper = proximalOffset <= 0
                    ? 1
                    : 1 - proximalOffset / (smoothingNodes + 1);
                const correctionX = radialX * correctionDistance * taper;
                const correctionY = radialY * correctionDistance * taper;
                const correctionZ = radialZ * correctionDistance * taper;
                outer.x[node] += correctionX;
                outer.y[node] += correctionY;
                outer.z[node] += correctionZ;
                outer.previousX[node] += correctionX * previousBlend;
                outer.previousY[node] += correctionY * previousBlend;
                outer.previousZ[node] += correctionZ * previousBlend;
            }
            if (correctionDistance > 1e-5) outer.wake();
            constraint.portalLambda = 0;
            return;
        }

        const innerResponse = outerOnly
            ? 0
            : innerOnly ? 1 : constraint.portalInnerResponse;
        const outerResponse = innerOnly ? 0 : constraint.portalOuterResponse;
        const gradientScale = activation;
        const innerWeightA =
            inner.inverseMass[innerSegment] * innerResponse *
            aWeight * aWeight * gradientScale * gradientScale;
        const innerWeightB =
            inner.inverseMass[innerSegment + 1] * innerResponse *
            bWeight * bWeight * gradientScale * gradientScale;
        const outerWeight = outer.inverseMass[outerTip] * outerResponse *
            gradientScale * gradientScale;
        const alpha = constraint.portalCompliance / (this.fixedDt * this.fixedDt);
        const denominator = innerWeightA + innerWeightB + outerWeight + alpha;
        if (denominator < EPSILON) return;
        const constraintValue = (radialDistance - allowedRadius) * activation;
        const unconstrainedDelta = (
            constraintValue - alpha * constraint.portalLambda
        ) / denominator;
        const maximumGradientWeight = Math.max(
            inner.inverseMass[innerSegment] * innerResponse * aWeight * gradientScale,
            inner.inverseMass[innerSegment + 1] * innerResponse * bWeight * gradientScale,
            outer.inverseMass[outerTip] * outerResponse * gradientScale
        );
        const maximumDelta = constraint.portalMaxCorrection /
            Math.max(EPSILON, maximumGradientWeight);
        const nextLambda = Math.max(
            0,
            constraint.portalLambda + clamp(unconstrainedDelta, -maximumDelta, maximumDelta)
        );
        const correction = nextLambda - constraint.portalLambda;
        constraint.portalLambda = nextLambda;

        inner.x[innerSegment] -=
            radialX * correction * inner.inverseMass[innerSegment] * innerResponse *
            aWeight * gradientScale;
        inner.y[innerSegment] -=
            radialY * correction * inner.inverseMass[innerSegment] * innerResponse *
            aWeight * gradientScale;
        inner.z[innerSegment] -=
            radialZ * correction * inner.inverseMass[innerSegment] * innerResponse *
            aWeight * gradientScale;
        inner.x[innerSegment + 1] -=
            radialX * correction * inner.inverseMass[innerSegment + 1] * innerResponse *
            bWeight * gradientScale;
        inner.y[innerSegment + 1] -=
            radialY * correction * inner.inverseMass[innerSegment + 1] * innerResponse *
            bWeight * gradientScale;
        inner.z[innerSegment + 1] -=
            radialZ * correction * inner.inverseMass[innerSegment + 1] * innerResponse *
            bWeight * gradientScale;
        outer.x[outerTip] +=
            radialX * correction * outer.inverseMass[outerTip] * outerResponse * gradientScale;
        outer.y[outerTip] +=
            radialY * correction * outer.inverseMass[outerTip] * outerResponse * gradientScale;
        outer.z[outerTip] +=
            radialZ * correction * outer.inverseMass[outerTip] * outerResponse * gradientScale;
        if (Math.abs(correction) > 1e-5) {
            if (innerResponse > EPSILON) inner.wake();
            if (outerResponse > EPSILON) outer.wake();
        }
    }

    #solveDistalPortalDirection(
        constraint,
        innerSegment,
        tangentX,
        tangentY,
        tangentZ,
        activation,
        allowedRadius,
        { innerOnly = false, outerOnly = false } = {}
    ) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerResponse = outerOnly ? 0 : constraint.portalInnerResponse;
        const outerResponse = innerOnly ? 0 : constraint.portalOuterResponse;
        if (
            (innerResponse <= EPSILON && outerResponse <= EPSILON) ||
            activation <= EPSILON
        ) {
            constraint.portalDirectionLambda = 0;
            return;
        }

        const next = innerSegment + 1;
        let segmentX = inner.x[next] - inner.x[innerSegment];
        let segmentY = inner.y[next] - inner.y[innerSegment];
        let segmentZ = inner.z[next] - inner.z[innerSegment];
        const segmentLength = magnitude3(segmentX, segmentY, segmentZ);
        if (segmentLength < EPSILON) {
            constraint.portalDirectionLambda = 0;
            return;
        }
        segmentX /= segmentLength;
        segmentY /= segmentLength;
        segmentZ /= segmentLength;

        const cosine = clamp(
            segmentX * tangentX +
                segmentY * tangentY +
                segmentZ * tangentZ,
            -1,
            1
        );
        const angle = Math.acos(cosine);
        // A wire can touch opposite lumen walls across the supported distal
        // length. That clearance defines a small physical exit cone; inside
        // it no angular correction is applied.
        const supportLength = Math.max(
            EPSILON,
            constraint.portalTransitionLength
        );
        const maximumAngle = Math.atan2(allowedRadius * 2, supportLength);
        const angleError = angle - maximumAngle;
        if (angleError <= 0) {
            constraint.portalDirectionLambda *= 0.5;
            return;
        }

        let axisX = segmentY * tangentZ - segmentZ * tangentY;
        let axisY = segmentZ * tangentX - segmentX * tangentZ;
        let axisZ = segmentX * tangentY - segmentY * tangentX;
        const axisLength = magnitude3(axisX, axisY, axisZ);
        if (axisLength < EPSILON) {
            constraint.portalDirectionLambda = 0;
            return;
        }
        axisX /= axisLength;
        axisY /= axisLength;
        axisZ /= axisLength;

        // The guidewire is one continuous elastic rod, not two independent
        // segments joined at the catheter opening. Rotating only the crossing
        // segment satisfies the aperture direction but creates an artificial
        // hinge at the next node and also changes both adjacent segment
        // lengths. The lumen bearing instead applies a boundary moment to the
        // whole free distal material. In the absence of another support its
        // lowest-energy response is a rigid rotation; vessel contacts and the
        // regular rod bending constraints subsequently distribute any
        // reaction-supported curvature along the metal wire.
        const gradientScale = activation;
        const angularWeight = (innerResponse + outerResponse) *
            gradientScale * gradientScale;
        const alpha = constraint.portalCompliance /
            (this.fixedDt * this.fixedDt);
        const denominator = angularWeight + alpha;
        if (denominator < EPSILON) return;
        let innerLeverLength = segmentLength;
        if (innerResponse > EPSILON) {
            innerLeverLength = 0;
            for (
                let segment = innerSegment;
                segment < inner.activeEnd;
                segment++
            ) {
                innerLeverLength += inner.restLength[segment];
            }
        }
        const outerTip = outer.activeEnd;
        const outerSupportSegments = Math.max(2, Math.ceil(
            constraint.portalSmoothingLength /
                Math.max(EPSILON, outer.segmentLength)
        ));
        const outerAnchor = Math.max(
            outer.activeStart,
            outerTip - outerSupportSegments
        );
        let outerLeverLength = outer.segmentLength;
        if (outerResponse > EPSILON) {
            outerLeverLength = 0;
            for (let segment = outerAnchor; segment < outerTip; segment++) {
                outerLeverLength += outer.restLength[segment];
            }
        }
        const maximumRotation = Math.atan2(
            constraint.portalMaxCorrection,
            Math.max(segmentLength, innerLeverLength, outerLeverLength)
        );
        const constraintValue = angleError * activation;
        const unconstrainedDelta = (
            -constraintValue - alpha * constraint.portalDirectionLambda
        ) / denominator;
        const maximumDelta = maximumRotation /
            Math.max(
                EPSILON,
                (innerResponse + outerResponse) * gradientScale
            );
        const nextLambda = Math.min(
            0,
            constraint.portalDirectionLambda +
                clamp(unconstrainedDelta, -maximumDelta, maximumDelta)
        );
        const lambdaCorrection = nextLambda - constraint.portalDirectionLambda;
        constraint.portalDirectionLambda = nextLambda;
        const innerCorrectionAngle = -lambdaCorrection *
            innerResponse * gradientScale;
        const outerCorrectionAngle = -lambdaCorrection *
            outerResponse * gradientScale;
        if (
            innerCorrectionAngle <= EPSILON &&
            outerCorrectionAngle <= EPSILON
        ) return;

        if (innerCorrectionAngle > EPSILON) {
            const anchorX = inner.x[innerSegment];
            const anchorY = inner.y[innerSegment];
            const anchorZ = inner.z[innerSegment];
            const rotationCosine = Math.cos(innerCorrectionAngle);
            const rotationSine = Math.sin(innerCorrectionAngle);
            const oneMinusCosine = 1 - rotationCosine;
            const previousBlend = 1 - clamp(
                inner.projectionVelocityRetention,
                0,
                1
            );
            for (let node = next; node <= inner.activeEnd; node++) {
                if (inner.inverseMass[node] <= 0) continue;
                const originalX = inner.x[node];
                const originalY = inner.y[node];
                const originalZ = inner.z[node];
                const relativeX = originalX - anchorX;
                const relativeY = originalY - anchorY;
                const relativeZ = originalZ - anchorZ;
                const axisDot =
                    relativeX * axisX +
                    relativeY * axisY +
                    relativeZ * axisZ;
                inner.x[node] = anchorX +
                    relativeX * rotationCosine +
                    (axisY * relativeZ - axisZ * relativeY) * rotationSine +
                    axisX * axisDot * oneMinusCosine;
                inner.y[node] = anchorY +
                    relativeY * rotationCosine +
                    (axisZ * relativeX - axisX * relativeZ) * rotationSine +
                    axisY * axisDot * oneMinusCosine;
                inner.z[node] = anchorZ +
                    relativeZ * rotationCosine +
                    (axisX * relativeY - axisY * relativeX) * rotationSine +
                    axisZ * axisDot * oneMinusCosine;
                inner.previousX[node] +=
                    (inner.x[node] - originalX) * previousBlend;
                inner.previousY[node] +=
                    (inner.y[node] - originalY) * previousBlend;
                inner.previousZ[node] +=
                    (inner.z[node] - originalZ) * previousBlend;
            }
            inner.wake();
        }

        if (outerCorrectionAngle > EPSILON && outerTip > outerAnchor) {
            for (let segment = outerAnchor; segment < outerTip; segment++) {
                outer.portalSegmentX[segment] =
                    outer.x[segment + 1] - outer.x[segment];
                outer.portalSegmentY[segment] =
                    outer.y[segment + 1] - outer.y[segment];
                outer.portalSegmentZ[segment] =
                    outer.z[segment + 1] - outer.z[segment];
            }
            // Once operator feed stops this is a quasi-static elastic
            // reaction, not a new command velocity. During active material
            // transport retain the body's configured projection response;
            // afterwards carry the previous pose by the full correction so
            // the bearing moment cannot pump an idle pair into oscillation.
            const previousBlend = constraint.limitDistalCorrection
                ? constraint.preserveStationaryInnerLength
                    ? Math.min(
                        0.05,
                        Math.max(
                            0,
                            constraint.portalRetractionDistance ?? 0
                        ) * 0.05
                    )
                    : 1 - clamp(outer.projectionVelocityRetention, 0, 1)
                : 1;
            const supportCount = outerTip - outerAnchor;
            for (let segment = outerAnchor; segment < outerTip; segment++) {
                const originalX = outer.x[segment + 1];
                const originalY = outer.y[segment + 1];
                const originalZ = outer.z[segment + 1];
                const ratio = (segment - outerAnchor + 1) / supportCount;
                const distributedRatio = ratio * ratio * (3 - 2 * ratio);
                const angle = -outerCorrectionAngle * distributedRatio;
                const rotationCosine = Math.cos(angle);
                const rotationSine = Math.sin(angle);
                const oneMinusCosine = 1 - rotationCosine;
                const directionX = outer.portalSegmentX[segment];
                const directionY = outer.portalSegmentY[segment];
                const directionZ = outer.portalSegmentZ[segment];
                const axisDot =
                    directionX * axisX +
                    directionY * axisY +
                    directionZ * axisZ;
                const rotatedX =
                    directionX * rotationCosine +
                    (axisY * directionZ - axisZ * directionY) * rotationSine +
                    axisX * axisDot * oneMinusCosine;
                const rotatedY =
                    directionY * rotationCosine +
                    (axisZ * directionX - axisX * directionZ) * rotationSine +
                    axisY * axisDot * oneMinusCosine;
                const rotatedZ =
                    directionZ * rotationCosine +
                    (axisX * directionY - axisY * directionX) * rotationSine +
                    axisZ * axisDot * oneMinusCosine;
                outer.x[segment + 1] = outer.x[segment] + rotatedX;
                outer.y[segment + 1] = outer.y[segment] + rotatedY;
                outer.z[segment + 1] = outer.z[segment] + rotatedZ;
                outer.previousX[segment + 1] +=
                    (outer.x[segment + 1] - originalX) * previousBlend;
                outer.previousY[segment + 1] +=
                    (outer.y[segment + 1] - originalY) * previousBlend;
                outer.previousZ[segment + 1] +=
                    (outer.z[segment + 1] - originalZ) * previousBlend;
            }
            outer.wake();
        }
    }

    #containmentPortalRetraction(constraint) {
        if (Number.isFinite(constraint.portalRetractionDistance)) {
            return Math.max(0, constraint.portalRetractionDistance);
        }
        if (!Number.isFinite(constraint.containedLength)) return 0;
        const outer = constraint.outerBody;
        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            outer.activeEnd
        );
        let outerLumenLength = 0;
        for (let segment = outerStart; segment < outer.activeEnd; segment++) {
            outerLumenLength += outer.restLength[segment];
        }
        return Math.max(
            0,
            outerLumenLength - Math.max(0, constraint.containedLength)
        );
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

    #projectInnerAlongOuterCenterline(constraint) {
        const inner = constraint.innerBody;
        const outer = constraint.outerBody;
        const innerStart = clamp(
            constraint.startNode,
            inner.activeStart,
            inner.activeEnd
        );
        const innerEnd = clamp(
            constraint.endNode,
            innerStart,
            inner.activeEnd
        );
        const projectionEnd = innerEnd;
        const outerStart = clamp(
            constraint.outerStartNode,
            outer.activeStart,
            outer.activeEnd
        );
        const outerEnd = Math.min(outer.activeEnd, outer.segmentCount);
        if (innerStart > innerEnd || outerStart >= outerEnd) return;
        if (innerEnd < inner.activeEnd) return;

        let outerSegment = outerStart;
        let segmentOffset = Math.max(0, constraint.innerArcOffset);
        while (outerSegment < outerEnd) {
            const materialLength = Math.max(
                EPSILON,
                outer.restLength[outerSegment]
            );
            if (
                segmentOffset <= materialLength ||
                outerSegment >= outerEnd - 1
            ) break;
            segmentOffset -= materialLength;
            outerSegment++;
        }
        const useSettledArcLength = !constraint.limitDistalCorrection;
        if (useSettledArcLength) {
            const dx = outer.x[outerSegment + 1] - outer.x[outerSegment];
            const dy = outer.y[outerSegment + 1] - outer.y[outerSegment];
            const dz = outer.z[outerSegment + 1] - outer.z[outerSegment];
            const materialLength = Math.max(
                EPSILON,
                outer.restLength[outerSegment]
            );
            segmentOffset = clamp(segmentOffset / materialLength, 0, 1) *
                Math.max(EPSILON, magnitude3(dx, dy, dz));
        }

        let moved = false;
        for (let innerIndex = innerStart; innerIndex <= projectionEnd; innerIndex++) {
            while (outerSegment < outerEnd) {
                const materialLength = Math.max(
                    EPSILON,
                    outer.restLength[outerSegment]
                );
                const mappingLength = useSettledArcLength
                    ? Math.max(EPSILON, magnitude3(
                        outer.x[outerSegment + 1] - outer.x[outerSegment],
                        outer.y[outerSegment + 1] - outer.y[outerSegment],
                        outer.z[outerSegment + 1] - outer.z[outerSegment]
                    ))
                    : materialLength;
                if (
                    segmentOffset <= mappingLength ||
                    outerSegment >= outerEnd - 1
                ) break;
                segmentOffset -= mappingLength;
                outerSegment++;
            }
            const dx = outer.x[outerSegment + 1] - outer.x[outerSegment];
            const dy = outer.y[outerSegment + 1] - outer.y[outerSegment];
            const dz = outer.z[outerSegment + 1] - outer.z[outerSegment];
            const materialLength = Math.max(
                EPSILON,
                outer.restLength[outerSegment]
            );
            const mappingLength = useSettledArcLength
                ? Math.max(EPSILON, magnitude3(dx, dy, dz))
                : materialLength;
            const t = clamp(segmentOffset / mappingLength, 0, 1);
            const targetX = outer.x[outerSegment] + dx * t;
            const targetY = outer.y[outerSegment] + dy * t;
            const targetZ = outer.z[outerSegment] + dz * t;
            const correctionX = targetX - inner.x[innerIndex];
            const correctionY = targetY - inner.y[innerIndex];
            const correctionZ = targetZ - inner.z[innerIndex];
            if (inner.inverseMass[innerIndex] > 0) {
                inner.x[innerIndex] = targetX;
                inner.y[innerIndex] = targetY;
                inner.z[innerIndex] = targetZ;
                moved ||= Math.abs(correctionX) + Math.abs(correctionY) +
                    Math.abs(correctionZ) > 0.01;
            }
            constraint.closestSegment[innerIndex] = outerSegment;
            constraint.closestT[innerIndex] = t;
            if (innerIndex < projectionEnd) {
                segmentOffset += inner.restLength[innerIndex];
            }
        }
        if (moved) inner.wake();
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
        if (a.sleeping && b.sleeping) return;
        if (a.sleeping) a.wake();
        if (b.sleeping) b.wake();
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
                if (staticFrictionBudget > 0 && tangentLength > EPSILON) {
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
        if (body.rodModel === 'kirchhoff') {
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
            !this.#hasRestDirectionErrorOver(body, 0.05)
        ) body.sleepCounter++;
        else body.sleepCounter = 0;
        if (body.sleepCounter >= body.sleepFrames) {
            body.sleeping = true;
            body.velocityX.fill(0);
            body.velocityY.fill(0);
            body.velocityZ.fill(0);
            body.angularVelocityX.fill(0);
            body.angularVelocityY.fill(0);
            body.angularVelocityZ.fill(0);
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
        if (constraint.model === 'kirchhoff') return;
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
        if (body.rodModel === 'kirchhoff') return;
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
        let maxBendNode = -1;
        let maxBendLimitDegrees = 0;
        let maxSpeed = 0;
        let kineticEnergy = 0;
        let maxMaterialTurnError = 0;
        let materialTurnErrorSquared = 0;
        let maxMaterialTurnResidual = 0;
        let maxMaterialTurnResidualNode = -1;
        let maxMaterialActualTurn = 0;
        let maxMaterialTargetTurn = 0;
        let materialTurnResidualSquared = 0;
        let materialTurnCount = 0;
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
            if (body.restDirectionEnabled[index] && body.restDirectionRelative[index]) {
                let incomingX = ax;
                let incomingY = ay;
                let incomingZ = az;
                let outgoingX = bx;
                let outgoingY = by;
                let outgoingZ = bz;
                const incomingLength = magnitude3(incomingX, incomingY, incomingZ);
                const outgoingLength = magnitude3(outgoingX, outgoingY, outgoingZ);
                if (incomingLength > EPSILON && outgoingLength > EPSILON) {
                    incomingX /= incomingLength;
                    incomingY /= incomingLength;
                    incomingZ /= incomingLength;
                    outgoingX /= outgoingLength;
                    outgoingY /= outgoingLength;
                    outgoingZ /= outgoingLength;
                    const axisX = body.restDirectionAxisX[index];
                    const axisY = body.restDirectionAxisY[index];
                    const axisZ = body.restDirectionAxisZ[index];
                    const incomingAxial =
                        incomingX * axisX + incomingY * axisY + incomingZ * axisZ;
                    const outgoingAxial =
                        outgoingX * axisX + outgoingY * axisY + outgoingZ * axisZ;
                    incomingX -= axisX * incomingAxial;
                    incomingY -= axisY * incomingAxial;
                    incomingZ -= axisZ * incomingAxial;
                    outgoingX -= axisX * outgoingAxial;
                    outgoingY -= axisY * outgoingAxial;
                    outgoingZ -= axisZ * outgoingAxial;
                    const incomingPlanarLength = magnitude3(
                        incomingX,
                        incomingY,
                        incomingZ
                    );
                    const outgoingPlanarLength = magnitude3(
                        outgoingX,
                        outgoingY,
                        outgoingZ
                    );
                    if (incomingPlanarLength > EPSILON && outgoingPlanarLength > EPSILON) {
                        incomingX /= incomingPlanarLength;
                        incomingY /= incomingPlanarLength;
                        incomingZ /= incomingPlanarLength;
                        outgoingX /= outgoingPlanarLength;
                        outgoingY /= outgoingPlanarLength;
                        outgoingZ /= outgoingPlanarLength;
                        const crossX = incomingY * outgoingZ - incomingZ * outgoingY;
                        const crossY = incomingZ * outgoingX - incomingX * outgoingZ;
                        const crossZ = incomingX * outgoingY - incomingY * outgoingX;
                        const actualTurn = Math.atan2(
                            axisX * crossX + axisY * crossY + axisZ * crossZ,
                            clamp(
                                incomingX * outgoingX +
                                    incomingY * outgoingY +
                                    incomingZ * outgoingZ,
                                -1,
                                1
                            )
                        );
                        let signedTurnError = actualTurn -
                            body.restDirectionTurnAngle[index];
                        if (signedTurnError > Math.PI) signedTurnError -= Math.PI * 2;
                        else if (signedTurnError < -Math.PI) signedTurnError += Math.PI * 2;
                        const turnError = Math.abs(signedTurnError);
                        const meanLength = Math.max(
                            EPSILON,
                            (incomingLength + outgoingLength) * 0.5
                        );
                        const angularAlpha = body.restDirectionCompliance[index] /
                            (
                                this.fixedDt * this.fixedDt *
                                meanLength * meanLength
                            );
                        const turnResidual = Math.abs(
                            signedTurnError +
                            angularAlpha * body.restDirectionLambdaX[index]
                        );
                        maxMaterialTurnError = Math.max(
                            maxMaterialTurnError,
                            turnError
                        );
                        materialTurnErrorSquared += turnError * turnError;
                        if (turnResidual > maxMaterialTurnResidual) {
                            maxMaterialTurnResidual = turnResidual;
                            maxMaterialTurnResidualNode = index;
                            maxMaterialActualTurn = actualTurn;
                            maxMaterialTargetTurn =
                                body.restDirectionTurnAngle[index];
                        }
                        materialTurnResidualSquared +=
                            turnResidual * turnResidual;
                        materialTurnCount++;
                    }
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
            maximumReconstructedSpeed:
                body.lastMaximumReconstructedSpeed,
            relaxationPasses: body.relaxationPasses,
            lastRelaxationPasses: body.lastRelaxationPasses,
            kineticEnergy,
            activeWallContacts,
            currentNormalLoad,
            retainedFrictionLoad,
            maxMaterialTurnErrorDegrees: maxMaterialTurnError * 180 / Math.PI,
            rmsMaterialTurnErrorDegrees: materialTurnCount > 0
                ? Math.sqrt(materialTurnErrorSquared / materialTurnCount) *
                    180 / Math.PI
                : 0,
            maxMaterialTurnResidualDegrees:
                maxMaterialTurnResidual * 180 / Math.PI,
            maxMaterialTurnResidualNode,
            maxMaterialActualTurnDegrees:
                maxMaterialActualTurn * 180 / Math.PI,
            maxMaterialTargetTurnDegrees:
                maxMaterialTargetTurn * 180 / Math.PI,
            rmsMaterialTurnResidualDegrees: materialTurnCount > 0
                ? Math.sqrt(materialTurnResidualSquared / materialTurnCount) *
                    180 / Math.PI
                : 0
        };
    }
}
