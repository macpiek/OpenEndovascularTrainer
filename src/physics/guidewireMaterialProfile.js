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
export const GUIDEWIRE_TYPE_STEEL_J_035 = 'steel-j-035';
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
        intrinsicBendCompliance: 2e-5,
        intrinsicBendMaxCorrection: 0.08,
        frameNormalSign: 1,
        intrinsicCurvature: ZERO_INTRINSIC_CURVATURE,
        integrateIntrinsicTurn: ZERO_INTRINSIC_TURN
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
        intrinsicBendCompliance: 2e-5,
        intrinsicBendMaxCorrection: 0.08,
        frameNormalSign: 1,
        intrinsicCurvature: steelJGuidewireIntrinsicCurvature,
        integrateIntrinsicTurn: integrateSteelJGuidewireIntrinsicTurn
    })
});

export function normalizeGuidewireType(type) {
    return type === GUIDEWIRE_TYPE_STEEL_J_035
        ? GUIDEWIRE_TYPE_STEEL_J_035
        : GUIDEWIRE_TYPE_GLIDEWIRE;
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

export function applyGuidewireIntrinsicCurvatureProfile(
    body,
    {
        type = GUIDEWIRE_TYPE_GLIDEWIRE,
        axisX = 0,
        axisY = 0,
        axisZ = 1
    } = {}
) {
    const profile = guidewireMaterialProfile(type);
    const axisLength = Math.hypot(axisX, axisY, axisZ) || 1;
    const normalizedAxisX = axisX / axisLength * profile.frameNormalSign;
    const normalizedAxisY = axisY / axisLength * profile.frameNormalSign;
    const normalizedAxisZ = axisZ / axisLength * profile.frameNormalSign;
    const naturalTurns = new Float64Array(body.segmentCount);
    let representedTurn = 0;

    for (let segment = 1; segment < body.segmentCount; segment++) {
        const outgoingLength = Math.max(
            0.5,
            body.restLength?.[segment] ?? body.segmentLength
        );
        const incomingLength = Math.max(
            0.5,
            body.restLength?.[segment - 1] ?? body.segmentLength
        );
        const voronoiLength = (incomingLength + outgoingLength) * 0.5;
        const distanceFromTip = Math.max(
            0,
            (body.count - 1 - segment) * body.segmentLength
        );
        naturalTurns[segment] = profile.integrateIntrinsicTurn(
            distanceFromTip,
            voronoiLength
        );
        representedTurn += naturalTurns[segment];
    }
    const turnScale = Math.abs(representedTurn) > 1e-8
        ? profile.naturalTurn / representedTurn
        : 1;

    for (let segment = 1; segment < body.segmentCount; segment++) {
        const outgoingLength = Math.max(
            0.5,
            body.restLength?.[segment] ?? body.segmentLength
        );
        const incomingLength = Math.max(
            0.5,
            body.restLength?.[segment - 1] ?? body.segmentLength
        );
        const voronoiLength = (incomingLength + outgoingLength) * 0.5;
        const naturalTurn = naturalTurns[segment] * turnScale;
        if (Math.abs(naturalTurn) <= 1e-8) {
            body.clearRestDirectionTarget(segment);
            continue;
        }
        body.maxBendAngleByNode[segment] = Math.max(
            body.maxBendAngleByNode[segment],
            Math.abs(naturalTurn) * 180 / Math.PI + 0.5
        );
        body.setIntrinsicCurvatureTarget(
            segment,
            naturalTurn,
            normalizedAxisX,
            normalizedAxisY,
            normalizedAxisZ,
            profile.intrinsicBendCompliance,
            profile.intrinsicBendMaxCorrection,
            0,
            voronoiLength
        );
    }
    return body;
}
