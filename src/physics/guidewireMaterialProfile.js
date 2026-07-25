export const GUIDEWIRE_BODY_BENDING_STIFFNESS = 16384;
export const GUIDEWIRE_TIP_BENDING_STIFFNESS = 64;
export const GUIDEWIRE_SOFT_TIP_LENGTH_MM = 20;
export const GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES = 10;
export const GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES = 30;

export function applyGuidewireMaterialProfile(
    rod,
    {
        segmentLength = rod.segmentLength,
        bodyBendingStiffness = GUIDEWIRE_BODY_BENDING_STIFFNESS,
        tipBendingStiffness = GUIDEWIRE_TIP_BENDING_STIFFNESS,
        softTipLength = GUIDEWIRE_SOFT_TIP_LENGTH_MM,
        bodyMaxBendAngle = GUIDEWIRE_BODY_MAX_BEND_ANGLE_DEGREES,
        tipMaxBendAngle = GUIDEWIRE_TIP_MAX_BEND_ANGLE_DEGREES
    } = {}
) {
    const lastIndex = rod.nodes.length - 1;
    for (let index = 0; index < rod.nodes.length; index++) {
        const distanceFromTip = (lastIndex - index) * segmentLength;
        const softTip = distanceFromTip < softTipLength;
        rod.nodes[index].bendingStiffness = softTip
            ? tipBendingStiffness
            : bodyBendingStiffness;
        rod.nodes[index].bendAngleLimit = softTip
            ? tipMaxBendAngle
            : bodyMaxBendAngle;
    }
    return rod;
}
