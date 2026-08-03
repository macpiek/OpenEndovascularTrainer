export const AORTA_SETUP_GUIDEWIRE_TARGET_MM = 400;
export const AORTA_SETUP_CATHETER_TARGET_MM = 240;
export const ILIAC_BUG_GUIDEWIRE_TARGET_MM = 223;
export const ILIAC_BUG_CATHETER_TARGET_MM = 241;
// Reproduces the short ipsilateral iliac position used to validate a
// momentum-dominant retrograde catheter jet and its aortic handoff.
export const RETROGRADE_GAP_GUIDEWIRE_TARGET_MM = 77;
export const RETROGRADE_GAP_CATHETER_TARGET_MM = 133;
// Reproduces the clinician-validated arch position where the catheter tip is
// in the aorta beside overlapping supra-aortic centreline prefixes.
export const ARCH_BOLUS_GUIDEWIRE_TARGET_MM = 559;
export const ARCH_BOLUS_CATHETER_TARGET_MM = 583;

export function createCatheterAortaSetupState() {
    return {
        running: false,
        phase: 'idle',
        guidewireTargetMm: AORTA_SETUP_GUIDEWIRE_TARGET_MM,
        catheterTargetMm: AORTA_SETUP_CATHETER_TARGET_MM,
        finalGuidewireTargetMm: null
    };
}

export function startCatheterAortaSetup(state, {
    guidewireTargetMm = AORTA_SETUP_GUIDEWIRE_TARGET_MM,
    catheterTargetMm = AORTA_SETUP_CATHETER_TARGET_MM,
    finalGuidewireTargetMm = null
} = {}) {
    state.guidewireTargetMm = guidewireTargetMm;
    state.catheterTargetMm = catheterTargetMm;
    state.finalGuidewireTargetMm = Number.isFinite(finalGuidewireTargetMm)
        ? finalGuidewireTargetMm
        : null;
    state.running = true;
    state.phase = 'guidewire';
    return state;
}

export function stopCatheterAortaSetup(state, phase = 'idle') {
    state.running = false;
    state.phase = phase;
    return state;
}

export function sampleCatheterAortaSetup(
    state,
    {
        guidewireProgressMm = 0,
        catheterProgressMm = 0
    } = {},
    out = {
        guidewireAdvance: 0,
        catheterAdvance: 0,
        catheterRotation: 0,
        catheterType: 'berenstein'
    }
) {
    if (!state.running) return null;

    out.guidewireAdvance = 0;
    out.catheterAdvance = 0;
    out.catheterRotation = 0;
    out.catheterType = 'berenstein';

    // Advance phases monotonically. In particular, once the optional final
    // guidewire withdrawal starts, do not re-enter the initial 400 mm advance
    // merely because the wire is now shorter than that intermediate target.
    if (state.phase === 'guidewire') {
        if (guidewireProgressMm < state.guidewireTargetMm - 0.25) {
            out.guidewireAdvance = 1;
            return out;
        }
        state.phase = 'catheter';
    }

    if (state.phase === 'catheter') {
        if (catheterProgressMm < state.catheterTargetMm - 0.25) {
            out.catheterAdvance = 1;
            return out;
        }
        state.phase = state.finalGuidewireTargetMm === null
            ? 'ready'
            : 'guidewire-withdraw';
    }

    if (state.phase === 'guidewire-withdraw') {
        if (guidewireProgressMm > state.finalGuidewireTargetMm + 0.25) {
            out.guidewireAdvance = -1;
            return out;
        }
        state.phase = 'ready';
    }

    state.running = false;
    return out;
}
