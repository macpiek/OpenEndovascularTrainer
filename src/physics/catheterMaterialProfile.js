const TWO_PI = Math.PI * 2;

export const PIGTAIL_NATURAL_RADIUS_MM = 7.2;
// A short pigtail: the distal preform keeps its 7.2 mm loop radius but ends
// before completing a full circle. This shortens the curved material without
// changing its local manufactured curvature.
export const PIGTAIL_NATURAL_TURNS = 0.9;
export const PIGTAIL_NATURAL_ARC_LENGTH_MM =
    PIGTAIL_NATURAL_RADIUS_MM * PIGTAIL_NATURAL_TURNS * TWO_PI;
export const PIGTAIL_CURVATURE_TRANSITION_MM = 4;
export const BERENSTEIN_STRAIGHT_TIP_LENGTH_MM = 8;
export const BERENSTEIN_BEND_LENGTH_MM = 10;
export const BERENSTEIN_TIP_SHAPE_LENGTH_MM =
    BERENSTEIN_STRAIGHT_TIP_LENGTH_MM + BERENSTEIN_BEND_LENGTH_MM;
export const BERENSTEIN_NATURAL_BEND_ANGLE_RAD = Math.PI / 4;
export const BERENSTEIN_CURVATURE_TRANSITION_MM = 2;

const PIGTAIL_TOTAL_TURN = PIGTAIL_NATURAL_TURNS * TWO_PI;
const PIGTAIL_FULL_CURVATURE_LENGTH =
    PIGTAIL_NATURAL_ARC_LENGTH_MM - PIGTAIL_CURVATURE_TRANSITION_MM;
// The quintic transition integrates to exactly half of its length. Scaling
// the plateau curvature preserves the prescribed total turn while the
// shaft-to-loop moment rises continuously instead of creating a hard hinge.
const PIGTAIL_PEAK_CURVATURE = PIGTAIL_TOTAL_TURN / (
    PIGTAIL_NATURAL_ARC_LENGTH_MM - PIGTAIL_CURVATURE_TRANSITION_MM * 0.5
);

function smootherstep01(value) {
    const t = Math.max(0, Math.min(1, value));
    return t * t * t * (t * (t * 6 - 15) + 10);
}

/**
 * Intrinsic centerline curvature at a material distance measured proximally
 * from the distal catheter tip. The rest curvature is a material property: it
 * does not depend on insertion direction, guidewire progress or wall contact.
 */
export function pigtailIntrinsicCurvature(distanceFromTipMm) {
    if (
        distanceFromTipMm < 0 ||
        distanceFromTipMm > PIGTAIL_NATURAL_ARC_LENGTH_MM
    ) return 0;
    if (distanceFromTipMm <= PIGTAIL_FULL_CURVATURE_LENGTH) {
        return PIGTAIL_PEAK_CURVATURE;
    }
    const transitionProgress = (
        distanceFromTipMm - PIGTAIL_FULL_CURVATURE_LENGTH
    ) / PIGTAIL_CURVATURE_TRANSITION_MM;
    return PIGTAIL_PEAK_CURVATURE * (1 - smootherstep01(transitionProgress));
}

const GAUSS_ABSCISSA = Math.sqrt(3 / 5);
const GAUSS_SAMPLES = Object.freeze([
    Object.freeze([-GAUSS_ABSCISSA, 5 / 9]),
    Object.freeze([0, 8 / 9]),
    Object.freeze([GAUSS_ABSCISSA, 5 / 9])
]);

function integrateSmoothInterval(start, end) {
    if (end <= start) return 0;
    const midpoint = (start + end) * 0.5;
    const halfWidth = (end - start) * 0.5;
    let result = 0;
    for (const [sample, weight] of GAUSS_SAMPLES) {
        result += weight * pigtailIntrinsicCurvature(
            midpoint + sample * halfWidth
        );
    }
    return result * halfWidth;
}

/**
 * Integrates the intrinsic turn owned by one discrete hinge/Voronoi cell.
 * Splitting at the material-profile boundaries keeps the result invariant to
 * catheter node spacing (the transition itself is a quintic polynomial).
 */
export function integratePigtailIntrinsicTurn(
    centerDistanceFromTipMm,
    voronoiLengthMm
) {
    const halfLength = Math.max(0, voronoiLengthMm) * 0.5;
    const start = Math.max(0, centerDistanceFromTipMm - halfLength);
    const end = Math.min(
        PIGTAIL_NATURAL_ARC_LENGTH_MM,
        centerDistanceFromTipMm + halfLength
    );
    if (end <= start) return 0;
    const firstEnd = Math.min(end, PIGTAIL_FULL_CURVATURE_LENGTH);
    let turn = integrateSmoothInterval(start, firstEnd);
    if (end > PIGTAIL_FULL_CURVATURE_LENGTH) {
        turn += integrateSmoothInterval(
            Math.max(start, PIGTAIL_FULL_CURVATURE_LENGTH),
            end
        );
    }
    return turn;
}

export function pigtailTotalIntrinsicTurn() {
    return integrateSmoothInterval(0, PIGTAIL_FULL_CURVATURE_LENGTH) +
        integrateSmoothInterval(
            PIGTAIL_FULL_CURVATURE_LENGTH,
            PIGTAIL_NATURAL_ARC_LENGTH_MM
        );
}

function samplePlanarIntrinsicRestCenterline(
    exposedLengthMm,
    distanceFromBaseMm,
    naturalShapeLengthMm,
    intrinsicCurvature,
    curvatureScale = 1,
    out = {}
) {
    const exposedLength = Math.max(0, exposedLengthMm);
    const deployedShapeLength = Math.min(
        exposedLength,
        naturalShapeLengthMm
    );
    const proximalStraightLength = Math.max(
        0,
        exposedLength - deployedShapeLength
    );
    const distance = Math.max(0, Math.min(exposedLength, distanceFromBaseMm));
    out.tangentDistance = distance;
    out.normalDistance = 0;
    out.turnAngle = 0;
    if (distance <= proximalStraightLength || deployedShapeLength <= 0) {
        return out;
    }

    const localDistance = Math.min(
        deployedShapeLength,
        distance - proximalStraightLength
    );
    const stepCount = Math.max(1, Math.ceil(localDistance / 0.25));
    const stepLength = localDistance / stepCount;
    const scale = Math.max(0, Math.min(1, curvatureScale));
    let tangentDistance = proximalStraightLength;
    let normalDistance = 0;
    let angle = 0;
    for (let step = 0; step < stepCount; step++) {
        const midpointFromBase = (step + 0.5) * stepLength;
        const distanceFromTip = deployedShapeLength - midpointFromBase;
        const curvature = intrinsicCurvature(distanceFromTip) * scale;
        const midpointAngle = angle + curvature * stepLength * 0.5;
        tangentDistance += Math.cos(midpointAngle) * stepLength;
        normalDistance += Math.sin(midpointAngle) * stepLength;
        angle += curvature * stepLength;
    }
    out.tangentDistance = tangentDistance;
    out.normalDistance = normalDistance;
    out.turnAngle = angle;
    return out;
}

/**
 * Samples the manufactured Pigtail centerline from the same intrinsic
 * curvature profile consumed by the rod solver.
 */
export function samplePigtailRestCenterline(
    exposedLengthMm,
    distanceFromBaseMm,
    curvatureScale = 1,
    out = {}
) {
    return samplePlanarIntrinsicRestCenterline(
        exposedLengthMm,
        distanceFromBaseMm,
        PIGTAIL_NATURAL_ARC_LENGTH_MM,
        pigtailIntrinsicCurvature,
        curvatureScale,
        out
    );
}

const BERENSTEIN_BEND_START = BERENSTEIN_STRAIGHT_TIP_LENGTH_MM;
const BERENSTEIN_BEND_END = BERENSTEIN_TIP_SHAPE_LENGTH_MM;
const BERENSTEIN_PLATEAU_START =
    BERENSTEIN_BEND_START + BERENSTEIN_CURVATURE_TRANSITION_MM;
const BERENSTEIN_PLATEAU_END =
    BERENSTEIN_BEND_END - BERENSTEIN_CURVATURE_TRANSITION_MM;
const BERENSTEIN_PEAK_CURVATURE = BERENSTEIN_NATURAL_BEND_ANGLE_RAD / (
    BERENSTEIN_BEND_LENGTH_MM - BERENSTEIN_CURVATURE_TRANSITION_MM
);

export function berensteinIntrinsicCurvature(distanceFromTipMm) {
    if (
        distanceFromTipMm <= BERENSTEIN_BEND_START ||
        distanceFromTipMm >= BERENSTEIN_BEND_END
    ) return 0;
    if (distanceFromTipMm < BERENSTEIN_PLATEAU_START) {
        return BERENSTEIN_PEAK_CURVATURE * smootherstep01(
            (distanceFromTipMm - BERENSTEIN_BEND_START) /
                BERENSTEIN_CURVATURE_TRANSITION_MM
        );
    }
    if (distanceFromTipMm <= BERENSTEIN_PLATEAU_END) {
        return BERENSTEIN_PEAK_CURVATURE;
    }
    return BERENSTEIN_PEAK_CURVATURE * (1 - smootherstep01(
        (distanceFromTipMm - BERENSTEIN_PLATEAU_END) /
            BERENSTEIN_CURVATURE_TRANSITION_MM
    ));
}

function integrateBerensteinSmoothInterval(start, end) {
    if (end <= start) return 0;
    const midpoint = (start + end) * 0.5;
    const halfWidth = (end - start) * 0.5;
    let result = 0;
    for (const [sample, weight] of GAUSS_SAMPLES) {
        result += weight * berensteinIntrinsicCurvature(
            midpoint + sample * halfWidth
        );
    }
    return result * halfWidth;
}

export function integrateBerensteinIntrinsicTurn(
    centerDistanceFromTipMm,
    voronoiLengthMm
) {
    const halfLength = Math.max(0, voronoiLengthMm) * 0.5;
    const start = Math.max(0, centerDistanceFromTipMm - halfLength);
    const end = Math.min(
        BERENSTEIN_TIP_SHAPE_LENGTH_MM,
        centerDistanceFromTipMm + halfLength
    );
    if (end <= start) return 0;
    const boundaries = [
        BERENSTEIN_BEND_START,
        BERENSTEIN_PLATEAU_START,
        BERENSTEIN_PLATEAU_END,
        BERENSTEIN_BEND_END
    ];
    let turn = 0;
    let intervalStart = start;
    for (const boundary of boundaries) {
        if (boundary <= intervalStart || boundary >= end) continue;
        turn += integrateBerensteinSmoothInterval(intervalStart, boundary);
        intervalStart = boundary;
    }
    return turn + integrateBerensteinSmoothInterval(intervalStart, end);
}

/**
 * Integrates the planar Berenstein rest centerline from the proximal end of
 * the currently exposed distal material. This is intentionally derived from
 * the same kappa_0 profile used by the XPBD hinge energy, so remeshing cannot
 * seed a curve whose bend lives in a different material interval.
 */
export function sampleBerensteinRestCenterline(
    exposedLengthMm,
    distanceFromBaseMm,
    curvatureScale = 1,
    out = {}
) {
    return samplePlanarIntrinsicRestCenterline(
        exposedLengthMm,
        distanceFromBaseMm,
        BERENSTEIN_TIP_SHAPE_LENGTH_MM,
        berensteinIntrinsicCurvature,
        curvatureScale,
        out
    );
}

/**
 * Constitutive data consumed by the common preformed-catheter rod solver.
 * Catheter types differ only by this material profile; frame transport,
 * stiffness, guidewire support, contact and XPBD equilibrium are shared.
 */
export const CATHETER_MATERIAL_PROFILES = Object.freeze({
    pigtail: Object.freeze({
        id: 'pigtail',
        naturalArcLength: PIGTAIL_NATURAL_ARC_LENGTH_MM,
        frameNormalSign: -1,
        intrinsicBendCompliance: 1e-7,
        intrinsicBendMaxCorrection: 0.012,
        shaftFoldLimitDegrees: 21,
        integrateIntrinsicTurn: integratePigtailIntrinsicTurn,
        sampleRestCenterline: samplePigtailRestCenterline
    }),
    berenstein: Object.freeze({
        id: 'berenstein',
        naturalArcLength: BERENSTEIN_TIP_SHAPE_LENGTH_MM,
        frameNormalSign: 1,
        intrinsicBendCompliance: 2e-5,
        intrinsicBendMaxCorrection: 0.012,
        shaftFoldLimitDegrees: 24,
        integrateIntrinsicTurn: integrateBerensteinIntrinsicTurn,
        sampleRestCenterline: sampleBerensteinRestCenterline
    })
});

export function catheterMaterialProfile(type) {
    return type === 'berenstein' || type === 'bernstein'
        ? CATHETER_MATERIAL_PROFILES.berenstein
        : CATHETER_MATERIAL_PROFILES.pigtail;
}
