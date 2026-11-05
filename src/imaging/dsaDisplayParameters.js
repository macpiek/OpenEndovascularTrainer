// Shared DSA presentation constants. Zero mask/live difference is displayed
// near the middle of the grayscale range so both darker iodine signal and
// brighter motion/mask-misalignment residuals retain display headroom.
export const DSA_NEUTRAL_LUMA = 0.52;
export const DSA_DARK_FLOOR_LUMA = 0.025;
export const DSA_BRIGHT_CEILING_LUMA = 0.975;
export const DSA_SUBTRACTION_DEADBAND = 0.004;
export const DSA_SUBTRACTION_SCALE = 4;
// Projected iodine is already a path integral. Convert it to attenuation once;
// applying 1-exp(-x) here and Beer–Lambert again later erased dilution gradients.
export const IODINE_ATTENUATION_SCALE = 0.65;
export const DSA_OPTICAL_SUBTRACTION_SCALE = 1;
export const DETECTOR_OPTICAL_NOISE_GAIN = 0.035;
export const DETECTOR_OPTICAL_NOISE_MAX_SIGMA = 0.12;

// Relative photon-count noise (high-count Poisson approximation). The bound
// prevents the synthetic noise control from making nonpositive photon counts.
export function detectorOpticalNoiseSigma(attenuation, {
    noiseLevel = .1, pulseRate = 15, zoomNoiseScale = 1
} = {}) {
    return Math.min(DETECTOR_OPTICAL_NOISE_MAX_SIGMA,
        Math.max(0, noiseLevel) * DETECTOR_OPTICAL_NOISE_GAIN *
        Math.sqrt(30 / Math.max(4, Math.min(30, pulseRate))) * Math.max(0, zoomNoiseScale) *
        Math.exp(Math.min(6, Math.max(0, attenuation)) * .5));
}

export function measuredDetectorOpticalDepth(attenuation, normalSample, parameters) {
    const sigma = detectorOpticalNoiseSigma(attenuation, parameters);
    return attenuation - Math.log(1 + sigma * Math.max(-3, Math.min(3, normalSample)));
}

export function iodineOpticalDepth(signal, gain = 5, opacity = 1) {
    return Math.max(0, signal) * Math.max(0, gain) *
        Math.max(0, Math.min(1, opacity)) * IODINE_ATTENUATION_SCALE;
}

export function dsaLumaFromOpticalDepth(difference, gain = 2.2) {
    const centered = Math.sign(difference) * Math.max(0, Math.abs(difference) - DSA_SUBTRACTION_DEADBAND);
    const scaled = centered * Math.max(0, gain) * DSA_OPTICAL_SUBTRACTION_SCALE;
    const signal = scaled / (1 + Math.abs(scaled));
    return DSA_NEUTRAL_LUMA - Math.max(signal, 0) * (DSA_NEUTRAL_LUMA - DSA_DARK_FLOOR_LUMA) +
        Math.max(-signal, 0) * (DSA_BRIGHT_CEILING_LUMA - DSA_NEUTRAL_LUMA);
}

const DISPLAY_GRAY_RED = 0xeb / 0xff;
const DETECTOR_TINT_RED = 0.992;

export const DSA_NEUTRAL_RED =
    DSA_NEUTRAL_LUMA * DISPLAY_GRAY_RED * DETECTOR_TINT_RED * 255;
