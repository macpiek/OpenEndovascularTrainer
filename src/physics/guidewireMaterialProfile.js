export const GUIDEWIRE_BODY_BENDING_STIFFNESS = 16384;
export const GUIDEWIRE_TIP_BENDING_STIFFNESS = 64;
export const GUIDEWIRE_TIP_CORE_LENGTH_MM = 12;
export const GUIDEWIRE_TIP_TRANSITION_LENGTH_MM = 36;
export const GUIDEWIRE_SOFT_TIP_LENGTH_MM =
    GUIDEWIRE_TIP_CORE_LENGTH_MM + GUIDEWIRE_TIP_TRANSITION_LENGTH_MM;
export const GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES = 10;
export const GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES = 30;

function smoothstep(value) {
    const t = Math.max(0, Math.min(1, value));
    return t * t * (3 - 2 * t);
}

export function applyGuidewireMaterialProfile(
    rod,
    {
        segmentLength = rod.segmentLength,
        bodyBendingStiffness = GUIDEWIRE_BODY_BENDING_STIFFNESS,
        tipBendingStiffness = GUIDEWIRE_TIP_BENDING_STIFFNESS,
        tipCoreLength = GUIDEWIRE_TIP_CORE_LENGTH_MM,
        tipTransitionLength = GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
        bodyMaxBendAngle = GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
        tipMaxBendAngle = GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES
    } = {}
) {
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
