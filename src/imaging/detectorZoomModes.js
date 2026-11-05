// Physical side lengths of the square detectors, before electronic field zoom.
export const DETECTOR_SIDES_MM = Object.freeze([210, 300]);
export const DEFAULT_DETECTOR_SIDE_MM = 300;

export function normalizeDetectorSideMm(value = DEFAULT_DETECTOR_SIDE_MM) {
    const sideMm = Number(value);
    return DETECTOR_SIDES_MM.includes(sideMm) ? sideMm : DEFAULT_DETECTOR_SIDE_MM;
}

const ZOOM_PRESETS = Object.freeze([
    Object.freeze({
        id: 'none',
        label: 'No zoom',
        displayLabel: 'Full field',
        zoomFactor: 1,
        fieldOfViewPercent: 100,
        kvOffset: 0,
        maMultiplier: 1,
        pulseWidthMultiplier: 1,
        referenceDoseRateMultiplier: 1,
        quantumNoiseScale: 1,
        scatterScale: 1
    }),
    Object.freeze({
        id: 'medium',
        label: 'Zoom 1',
        displayLabel: 'Medium',
        zoomFactor: 1.4,
        fieldOfViewPercent: 71,
        kvOffset: 5,
        maMultiplier: 1.48,
        pulseWidthMultiplier: 1.1,
        referenceDoseRateMultiplier: 1.84,
        quantumNoiseScale: 0.74,
        scatterScale: 0.84
    }),
    Object.freeze({
        id: 'maximum',
        label: 'Zoom 2',
        displayLabel: 'Maximum',
        zoomFactor: 2.1,
        fieldOfViewPercent: 48,
        kvOffset: 10,
        maMultiplier: 1.69,
        pulseWidthMultiplier: 1.18,
        referenceDoseRateMultiplier: 2.54,
        quantumNoiseScale: 0.63,
        scatterScale: 0.72
    })
]);

// Preserve the existing electronic magnification and exposure presets when
// switching hardware; the physical field is derived from the selected panel.
const MODES_BY_DETECTOR = new Map(DETECTOR_SIDES_MM.map(detectorSideMm => [
    detectorSideMm,
    Object.freeze(ZOOM_PRESETS.map(preset => Object.freeze({
        ...preset,
        detectorSideMm,
        inputFieldCm: detectorSideMm / (10 * preset.zoomFactor)
    })))
]));

export const DETECTOR_ZOOM_MODES = MODES_BY_DETECTOR.get(DEFAULT_DETECTOR_SIDE_MM);

export function getDetectorZoomMode(modeId = 'none', detectorSideMm = DEFAULT_DETECTOR_SIDE_MM) {
    const modes = MODES_BY_DETECTOR.get(normalizeDetectorSideMm(detectorSideMm));
    return modes.find(mode => mode.id === modeId) || modes[0];
}

export function nextDetectorZoomMode(modeId = 'none', detectorSideMm = DEFAULT_DETECTOR_SIDE_MM) {
    const modes = MODES_BY_DETECTOR.get(normalizeDetectorSideMm(detectorSideMm));
    const index = modes.findIndex(mode => mode.id === modeId);
    return modes[(Math.max(0, index) + 1) % modes.length];
}

export function applyDetectorZoomTechnique(
    { kv = 0, ma = 0, pulseWidthMs = 8 } = {},
    modeOrId = 'none'
) {
    const mode = typeof modeOrId === 'string'
        ? getDetectorZoomMode(modeOrId)
        : getDetectorZoomMode(modeOrId?.id, modeOrId?.detectorSideMm);
    return {
        kv: Math.max(0, Number(kv) || 0) + mode.kvOffset,
        ma: Math.max(0, Number(ma) || 0) * mode.maMultiplier,
        pulseWidthMs: Math.max(0, Number(pulseWidthMs) || 0) *
            mode.pulseWidthMultiplier
    };
}
