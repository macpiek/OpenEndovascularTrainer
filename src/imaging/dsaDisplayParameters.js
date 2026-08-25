// Shared DSA presentation constants. Zero mask/live difference is displayed
// near the middle of the grayscale range so both darker iodine signal and
// brighter motion/mask-misalignment residuals retain display headroom.
export const DSA_NEUTRAL_LUMA = 0.52;
export const DSA_DARK_FLOOR_LUMA = 0.025;
export const DSA_BRIGHT_CEILING_LUMA = 0.975;
export const DSA_SUBTRACTION_DEADBAND = 0.004;
export const DSA_SUBTRACTION_SCALE = 4;

const DISPLAY_GRAY_RED = 0xeb / 0xff;
const DETECTOR_TINT_RED = 0.992;

export const DSA_NEUTRAL_RED =
    DSA_NEUTRAL_LUMA * DISPLAY_GRAY_RED * DETECTOR_TINT_RED * 255;
