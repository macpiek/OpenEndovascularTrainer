export const DETECTOR_ZOOM_MODES = Object.freeze([
    Object.freeze({
        id: 'none',
        label: 'No zoom',
        displayLabel: 'Full field',
        inputFieldCm: 21,
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
        inputFieldCm: 15,
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
        inputFieldCm: 10,
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

export function getDetectorZoomMode(modeId = 'none') {
    return DETECTOR_ZOOM_MODES.find(mode => mode.id === modeId) ||
        DETECTOR_ZOOM_MODES[0];
}

export function nextDetectorZoomMode(modeId = 'none') {
    const index = DETECTOR_ZOOM_MODES.findIndex(mode => mode.id === modeId);
    return DETECTOR_ZOOM_MODES[(Math.max(0, index) + 1) % DETECTOR_ZOOM_MODES.length];
}

export function applyDetectorZoomTechnique(
    { kv = 0, ma = 0, pulseWidthMs = 8 } = {},
    modeOrId = 'none'
) {
    const mode = typeof modeOrId === 'string'
        ? getDetectorZoomMode(modeOrId)
        : getDetectorZoomMode(modeOrId?.id);
    return {
        kv: Math.max(0, Number(kv) || 0) + mode.kvOffset,
        ma: Math.max(0, Number(ma) || 0) * mode.maMultiplier,
        pulseWidthMs: Math.max(0, Number(pulseWidthMs) || 0) *
            mode.pulseWidthMultiplier
    };
}
