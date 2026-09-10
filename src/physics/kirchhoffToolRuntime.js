export const TOOL_MAX_BEND_ANGLE_DEGREES = 45;

/** Shared numerical motion policy for every instrument.
 * Material/geometric coefficients remain on the body profile. Commands and
 * tool identity do not change the solver or its position-history convention.
 */
export function configureKirchhoffToolRuntime(body) {
    body.maxBendAngle = TOOL_MAX_BEND_ANGLE_DEGREES;
    body.maxBendAngleByNode.fill(TOOL_MAX_BEND_ANGLE_DEGREES);
    body.postStabilizationPasses = 0;
    body.finalStructuralClosurePasses = 8;
    body.postStabilizeBending = false;
    body.projectionVelocityRetention = 1;
    body.distalProjectionVelocityRetention = 1;
    body.distalProjectionVelocityRetentionStartNode = Infinity;
    body.maxFrameDisplacement = Infinity;
    body.wallProjectionVelocityRetention = 0;
    // Contact projection is not an independent source of kinetic energy.
    // The same rule applies on both sides of an instrument contact.
    body.toolProjectionVelocityRetention = 0;
    body.sweptContactPreserveTangentialMotion = true;
    body.wallFrictionUsesCurrentLoad = false;
    body.wallFrictionUsesSmoothedLoad = false;
    return body;
}
