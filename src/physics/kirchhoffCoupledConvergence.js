const EPSILON = 1e-8;

/**
 * Change of the relative contact coordinates during one outer rod solve.
 * Use the actual linear/cubic contact stencil, including the sliding mouth.
 * Motion of unsupported material and common translation are not contact error.
 * This is only a stopping criterion; it never limits the rod's response range.
 */
export function measureKirchhoffContactMotion(constraint, includeSticking = true) {
    let maximum = 0;
    let normalMotion = 0, stickingTangentialMotion = 0;
    for (const record of constraint.kirchhoffContacts) {
        const contact = record.manifoldContact;
        if (!contact || (contact.normalLambda <= EPSILON && record.gap > 0)) continue;
        let dx = 0;
        let dy = 0;
        let dz = 0;
        for (let side = 0; side < 2; side++) {
            const body = side === 0 ? constraint.innerBody : constraint.outerBody;
            const nodes = side === 0 ? record._innerNodeIndices : record._outerNodeIndices;
            const weights = side === 0 ? record._innerNodeWeights : record._outerNodeWeights;
            const segment = side === 0 ? record._innerSegmentIndex : record._outerSegmentIndex;
            const count = nodes ? (side === 0 ? record._innerNodeCount : record._outerNodeCount) : 2;
            const fallback = side === 0 ? record.innerWeights : record.outerWeights;
            for (let i = 0; i < count; i++) {
                const node = nodes ? nodes[i] : segment + i;
                const weight = (side === 0 ? 1 : -1) * (weights ? weights[i] : fallback[i]);
                dx += weight * (body.x[node] - body.postPassStartX[node]);
                dy += weight * (body.y[node] - body.postPassStartY[node]);
                dz += weight * (body.z[node] - body.postPassStartZ[node]);
            }
        }
        // Sticking constrains both tangents. At the Coulomb limit, axial
        // sliding is admissible and must not be mistaken for nonconvergence.
        const frictionLimit = constraint.axialFriction * contact.normalLambda;
        // The joint surface solver checks sticking/sliding with the full
        // anisotropic Coulomb KKT, including spin and lever-arm velocities.
        // Its caller can request normal motion alone here.
        const sticking = includeSticking && frictionLimit > EPSILON && Math.hypot(
            contact.tangentLambda[0], contact.tangentLambda[1]
        ) < frictionLimit - EPSILON;
        const normal = Math.abs(dx * record.normal[0] + dy * record.normal[1] + dz * record.normal[2]);
        const motion = sticking ? Math.hypot(dx, dy, dz) : normal;
        normalMotion = Math.max(normalMotion, normal);
        if (sticking) stickingTangentialMotion = Math.max(stickingTangentialMotion, Math.sqrt(Math.max(0, motion * motion - normal * normal)));
        maximum = Math.max(maximum, motion);
    }
    constraint.kirchhoffNormalMotion = normalMotion;
    constraint.kirchhoffStickingTangentialMotion = stickingTangentialMotion;
    return maximum;
}
