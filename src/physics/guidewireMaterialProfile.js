export const GUIDEWIRE_BODY_BENDING_STIFFNESS = 16384;
// The Glidewire shaft remains pushable, while the distal 5 cm forms an
// atraumatic flexible zone. The final 24 mm has the lowest EI and the next
// 26 mm blends geometrically into the shaft so there is no stiffness hinge.
export const GUIDEWIRE_TIP_BENDING_STIFFNESS = 128;
export const GUIDEWIRE_TIP_CORE_LENGTH_MM = 24;
export const GUIDEWIRE_TIP_TRANSITION_LENGTH_MM = 26;
export const GUIDEWIRE_SOFT_TIP_LENGTH_MM =
    GUIDEWIRE_TIP_CORE_LENGTH_MM + GUIDEWIRE_TIP_TRANSITION_LENGTH_MM;
export const GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES = 10;
export const GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES = 30;
export const GUIDEWIRE_TYPE_GLIDEWIRE = 'glidewire';
export const GUIDEWIRE_TYPE_GLIDEWIRE_ANGLED = 'glidewire-angled';
export const GLIDEWIRE_ANGLED_TIP_LENGTH_MM = 15;
export const GLIDEWIRE_ANGLED_TURN_RAD = Math.PI / 4;
// A smooth manufactured bend, integrated exactly over each material cell.
export function angledGlidewireIntrinsicCurvature(distance) {
    if (distance < 0 || distance > GLIDEWIRE_ANGLED_TIP_LENGTH_MM) return 0;
    return GLIDEWIRE_ANGLED_TURN_RAD / GLIDEWIRE_ANGLED_TIP_LENGTH_MM *
        (1 - Math.cos(2 * Math.PI * distance / GLIDEWIRE_ANGLED_TIP_LENGTH_MM));
}
export function integrateAngledGlidewireIntrinsicTurn(center, width) {
    const length = GLIDEWIRE_ANGLED_TIP_LENGTH_MM;
    const a = Math.max(0, Math.min(length, center - Math.max(0, width) / 2));
    const b = Math.max(0, Math.min(length, center + Math.max(0, width) / 2));
    const primitive = x => GLIDEWIRE_ANGLED_TURN_RAD * (x / length - Math.sin(2 * Math.PI * x / length) / (2 * Math.PI));
    return primitive(b) - primitive(a);
}
export const GUIDEWIRE_TYPE_STEEL_J_035 = 'steel-j-035';
export const GUIDEWIRE_TYPE_LUNDERQUIST = 'lunderquist-double-curved';
// Cook G45208 / TSCMG-35-260-LESDC: 0.035", 260 cm, 4 cm floppy
// tip, 11 cm taper, primary/secondary radii 75/15 mm.
// https://www.cookmedical.com/products/ndo_tsmg_webds/
// The two half-turns approximate the manufacturer's product photograph;
// arc angles and EI are simulator presets, not manufacturer measurements.
export const LUNDERQUIST_LENGTH_MM = 2600;
export const LUNDERQUIST_PRIMARY_RADIUS_MM = 75;
export const LUNDERQUIST_SECONDARY_RADIUS_MM = 15;
const LUNDERQUIST_SECONDARY_ARC_MM = Math.PI * LUNDERQUIST_SECONDARY_RADIUS_MM;
const LUNDERQUIST_CURVATURE_BLEND_MM = 8;
const LUNDERQUIST_SHAFT_BLEND_MM = 10;
export const LUNDERQUIST_CURVED_LENGTH_MM = Math.PI *
    (LUNDERQUIST_PRIMARY_RADIUS_MM + LUNDERQUIST_SECONDARY_RADIUS_MM) +
    LUNDERQUIST_SHAFT_BLEND_MM / 2;
export const LUNDERQUIST_NATURAL_TURN_RAD = 2 * Math.PI;

export function lunderquistIntrinsicCurvature(distance) {
    if (distance < 0 || distance >= LUNDERQUIST_CURVED_LENGTH_MM) return 0;
    const junctionStart = LUNDERQUIST_SECONDARY_ARC_MM - LUNDERQUIST_CURVATURE_BLEND_MM / 2;
    const junction = smoothstep((distance - junctionStart) / LUNDERQUIST_CURVATURE_BLEND_MM);
    const curvature = (1 - junction) / LUNDERQUIST_SECONDARY_RADIUS_MM +
        junction / LUNDERQUIST_PRIMARY_RADIUS_MM;
    return curvature * (1 - smoothstep((distance -
        (LUNDERQUIST_CURVED_LENGTH_MM - LUNDERQUIST_SHAFT_BLEND_MM)) / LUNDERQUIST_SHAFT_BLEND_MM));
}

export function integrateLunderquistIntrinsicTurn(center, width) {
    const start = Math.max(0, center - Math.max(0, width) / 2);
    const end = Math.min(LUNDERQUIST_CURVED_LENGTH_MM, center + Math.max(0, width) / 2);
    if (end <= start) return 0;
    // Split at each polynomial boundary: exact on coarse and adaptive meshes.
    const boundaries = [start,
        LUNDERQUIST_SECONDARY_ARC_MM - LUNDERQUIST_CURVATURE_BLEND_MM / 2,
        LUNDERQUIST_SECONDARY_ARC_MM + LUNDERQUIST_CURVATURE_BLEND_MM / 2,
        LUNDERQUIST_CURVED_LENGTH_MM - LUNDERQUIST_SHAFT_BLEND_MM,
        end].filter(x => x >= start && x <= end);
    let integral = 0;
    for (let i = 1; i < boundaries.length; i++) {
        const half = (boundaries[i] - boundaries[i - 1]) / 2;
        const mid = (boundaries[i] + boundaries[i - 1]) / 2;
        for (const [x, weight] of GAUSS_SAMPLES)
            integral += half * weight * lunderquistIntrinsicCurvature(mid + half * x);
    }
    return integral;
}
export const STEEL_J_GUIDEWIRE_BODY_BENDING_STIFFNESS = 32768;
export const STEEL_J_GUIDEWIRE_TIP_BENDING_STIFFNESS = 128;
export const STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM = 20;
export const STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM = 30;
export const STEEL_J_GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES = 8;
export const STEEL_J_GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES = 58;
export const STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM = 20;
export const STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD = Math.PI;
export const STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM = 5;

const STEEL_J_CURVATURE_PLATEAU_LENGTH_MM =
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM -
    STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM;
const STEEL_J_PEAK_CURVATURE = STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD / (
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM -
    STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM * 0.5
);

function smoothstep(value) {
    const t = Math.max(0, Math.min(1, value));
    return t * t * (3 - 2 * t);
}

function smootherstep(value) {
    const t = Math.max(0, Math.min(1, value));
    return t * t * t * (t * (t * 6 - 15) + 10);
}

export function steelJGuidewireIntrinsicCurvature(distanceFromTipMm) {
    if (
        distanceFromTipMm < 0 ||
        distanceFromTipMm > STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM
    ) return 0;
    if (distanceFromTipMm <= STEEL_J_CURVATURE_PLATEAU_LENGTH_MM) {
        return STEEL_J_PEAK_CURVATURE;
    }
    const transition = (
        distanceFromTipMm - STEEL_J_CURVATURE_PLATEAU_LENGTH_MM
    ) / STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM;
    return STEEL_J_PEAK_CURVATURE * (1 - smootherstep(transition));
}

const GAUSS_ABSCISSA = Math.sqrt(3 / 5);
const GAUSS_SAMPLES = Object.freeze([
    Object.freeze([-GAUSS_ABSCISSA, 5 / 9]),
    Object.freeze([0, 8 / 9]),
    Object.freeze([GAUSS_ABSCISSA, 5 / 9])
]);

function integrateSteelJCurvature(start, end) {
    if (end <= start) return 0;
    const midpoint = (start + end) * 0.5;
    const halfWidth = (end - start) * 0.5;
    let result = 0;
    for (const [sample, weight] of GAUSS_SAMPLES) {
        result += weight * steelJGuidewireIntrinsicCurvature(
            midpoint + sample * halfWidth
        );
    }
    return result * halfWidth;
}

export function integrateSteelJGuidewireIntrinsicTurn(
    centerDistanceFromTipMm,
    voronoiLengthMm
) {
    const halfLength = Math.max(0, voronoiLengthMm) * 0.5;
    const start = Math.max(0, centerDistanceFromTipMm - halfLength);
    const end = Math.min(
        STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
        centerDistanceFromTipMm + halfLength
    );
    if (end <= start) return 0;
    const firstEnd = Math.min(end, STEEL_J_CURVATURE_PLATEAU_LENGTH_MM);
    let turn = integrateSteelJCurvature(start, firstEnd);
    if (end > STEEL_J_CURVATURE_PLATEAU_LENGTH_MM) {
        turn += integrateSteelJCurvature(
            Math.max(start, STEEL_J_CURVATURE_PLATEAU_LENGTH_MM),
            end
        );
    }
    return turn;
}

const ZERO_INTRINSIC_TURN = () => 0;
const ZERO_INTRINSIC_CURVATURE = () => 0;

export const GUIDEWIRE_MATERIAL_PROFILES = Object.freeze({
    [GUIDEWIRE_TYPE_LUNDERQUIST]: Object.freeze({
        id: GUIDEWIRE_TYPE_LUNDERQUIST,
        bodyBendingStiffness: 131072,
        tipBendingStiffness: 128,
        tipCoreLength: 40,
        tipTransitionLength: 110,
        bodyMaxBendAngle: 5,
        tipMaxBendAngle: 30,
        naturalArcLength: LUNDERQUIST_CURVED_LENGTH_MM,
        naturalTurn: LUNDERQUIST_NATURAL_TURN_RAD,
        frameNormalSign: 1,
        intrinsicCurvature: lunderquistIntrinsicCurvature,
        integrateIntrinsicTurn: integrateLunderquistIntrinsicTurn
    }),
    [GUIDEWIRE_TYPE_GLIDEWIRE]: Object.freeze({
        id: GUIDEWIRE_TYPE_GLIDEWIRE,
        bodyBendingStiffness: GUIDEWIRE_BODY_BENDING_STIFFNESS,
        tipBendingStiffness: GUIDEWIRE_TIP_BENDING_STIFFNESS,
        tipCoreLength: GUIDEWIRE_TIP_CORE_LENGTH_MM,
        tipTransitionLength: GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
        bodyMaxBendAngle: GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
        tipMaxBendAngle: GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES,
        naturalArcLength: 0,
        naturalTurn: 0,
        frameNormalSign: 1,
        intrinsicCurvature: ZERO_INTRINSIC_CURVATURE,
        integrateIntrinsicTurn: ZERO_INTRINSIC_TURN
    }),
    [GUIDEWIRE_TYPE_GLIDEWIRE_ANGLED]: Object.freeze({
        id: GUIDEWIRE_TYPE_GLIDEWIRE_ANGLED,
        bodyBendingStiffness: GUIDEWIRE_BODY_BENDING_STIFFNESS,
        tipBendingStiffness: GUIDEWIRE_TIP_BENDING_STIFFNESS,
        tipCoreLength: GUIDEWIRE_TIP_CORE_LENGTH_MM,
        tipTransitionLength: GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
        bodyMaxBendAngle: GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
        tipMaxBendAngle: GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES,
        naturalArcLength: GLIDEWIRE_ANGLED_TIP_LENGTH_MM,
        naturalTurn: GLIDEWIRE_ANGLED_TURN_RAD,
        frameNormalSign: 1,
        intrinsicCurvature: angledGlidewireIntrinsicCurvature,
        integrateIntrinsicTurn: integrateAngledGlidewireIntrinsicTurn
    }),
    [GUIDEWIRE_TYPE_STEEL_J_035]: Object.freeze({
        id: GUIDEWIRE_TYPE_STEEL_J_035,
        bodyBendingStiffness: STEEL_J_GUIDEWIRE_BODY_BENDING_STIFFNESS,
        tipBendingStiffness: STEEL_J_GUIDEWIRE_TIP_BENDING_STIFFNESS,
        tipCoreLength: STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM,
        tipTransitionLength: STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
        bodyMaxBendAngle: STEEL_J_GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
        tipMaxBendAngle: STEEL_J_GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES,
        naturalArcLength: STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM,
        naturalTurn: STEEL_J_GUIDEWIRE_NATURAL_TURN_RAD,
        frameNormalSign: 1,
        intrinsicCurvature: steelJGuidewireIntrinsicCurvature,
        integrateIntrinsicTurn: integrateSteelJGuidewireIntrinsicTurn
    })
});

export function normalizeGuidewireType(type) {
    return isGuidewireType(type) ? type : GUIDEWIRE_TYPE_GLIDEWIRE;
}

export function isGuidewireType(type) {
    return Object.hasOwn(GUIDEWIRE_MATERIAL_PROFILES, type);
}

export function guidewireMaterialProfile(type) {
    return GUIDEWIRE_MATERIAL_PROFILES[normalizeGuidewireType(type)];
}

export function sampleGuidewireRestCenterline(
    type,
    exposedLengthMm,
    distanceFromBaseMm,
    curvatureScale = 1,
    out = {}
) {
    const profile = guidewireMaterialProfile(type);
    const exposedLength = Math.max(0, exposedLengthMm);
    const deployedShapeLength = Math.min(
        exposedLength,
        profile.naturalArcLength
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
        const curvature = profile.intrinsicCurvature(distanceFromTip) * scale;
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

export function applyGuidewireMaterialProfile(
    rod,
    options = {}
) {
    const profile = guidewireMaterialProfile(options.type);
    const stiffnessScale = options.stiffnessScale ?? 1;
    if (!Number.isFinite(stiffnessScale) || stiffnessScale <= 0) {
        throw new RangeError('Guidewire stiffness scale must be finite and positive');
    }
    const shaftStiffnessScale = options.shaftStiffnessScale ?? stiffnessScale;
    const tipStiffnessScale = options.tipStiffnessScale ?? stiffnessScale;
    if (!Number.isFinite(shaftStiffnessScale) || shaftStiffnessScale <= 0) {
        throw new RangeError('Guidewire shaft stiffness scale must be finite and positive');
    }
    if (!Number.isFinite(tipStiffnessScale) || tipStiffnessScale <= 0) {
        throw new RangeError('Guidewire tip stiffness scale must be finite and positive');
    }
    const segmentLength = options.segmentLength ?? rod.segmentLength;
    const bodyBendingStiffness = (
        options.bodyBendingStiffness ?? profile.bodyBendingStiffness
    ) * shaftStiffnessScale;
    const tipBendingStiffness = (
        options.tipBendingStiffness ?? profile.tipBendingStiffness
    ) * tipStiffnessScale;
    const tipCoreLength = options.tipCoreLength ?? profile.tipCoreLength;
    const tipTransitionLength = options.tipTransitionLength ??
        profile.tipTransitionLength;
    const bodyMaxBendAngle = options.bodyMaxBendAngle ??
        profile.bodyMaxBendAngle;
    const tipMaxBendAngle = options.tipMaxBendAngle ??
        profile.tipMaxBendAngle;
    const lastIndex = rod.nodes.length - 1;
    for (let index = 0; index < rod.nodes.length; index++) {
        const distanceFromTip = (lastIndex - index) * segmentLength;
        const transition = smoothstep(
            (distanceFromTip - tipCoreLength) / Math.max(1e-6, tipTransitionLength)
        );
        // Interpolate stiffness geometrically. A linear blend would place
        // almost the entire compliance change in the first transition node.
        rod.nodes[index].bendingStiffness =
            tipBendingStiffness * Math.pow(bodyBendingStiffness / tipBendingStiffness, transition);
        rod.nodes[index].bendAngleLimit =
            tipMaxBendAngle + (bodyMaxBendAngle - tipMaxBendAngle) * transition;
    }
    return rod;
}
