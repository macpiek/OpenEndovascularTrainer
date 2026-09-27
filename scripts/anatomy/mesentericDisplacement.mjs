// This atlas branch passes anterior to the infrarenal sac. Move only its
// connected surface/centerline selection; the aorta and renal vessels stay put.
export const MESENTERIC_DISPLACEMENT = Object.freeze({
    // Select above the first branch junction, but begin bending below it.
    // Cutting selection at the bend start splits the surface and centerline
    // differently around a wide bifurcation.
    proximalY: -180,
    displacementStartY: -190,
    distalY: -305,
    referenceY: -230,
    seed: [-10, -230, 33],
    anteriorOffsetMm: 16,
    proximalRampMm: 30,
    distalRampMm: 40
});

const smoothstep = value => {
    const t = Math.max(0, Math.min(1, value));
    return t * t * (3 - 2 * t);
};

export function createMesentericDisplacement(config = MESENTERIC_DISPLACEMENT) {
    return {
        centerAt: y => [config.seed[0], y, config.seed[2]],
        move: (x, y, z) => [x, y, z + config.anteriorOffsetMm *
            smoothstep((config.displacementStartY - y) / config.proximalRampMm) *
            smoothstep((y - config.distalY) / config.distalRampMm)]
    };
}
