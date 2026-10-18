import {CONTACT_PROTECTED_MESH} from './kirchhoffSharedAxisAdaptiveMesh.js';

// Research preset for the recorded long-tool crossover. Local mesh budgets
// are not a global trajectory guarantee: compare every accepted step against
// the full model using benchmark-crossover-reduced-model.mjs (0.1 mm gate).
// Full vessel collision discovery and solver tolerances remain unchanged.
export const CROSSOVER_REDUCED_MODEL = Object.freeze({
    ...CONTACT_PROTECTED_MESH,
    maxSpacing:20, contactMaxSpacing:10, shapeTolerance:.02,
    maxArcLoss:.0001, maxTurn:.06, tipLength:80, tipRefinementAhead:80
});

// Wider mechanical elements in contact regions; collision queries still cover
// each complete capsule. Keep this separate from the narrower reference preset
// so browser measurements and paired trajectory reports remain reproducible.
export const CROSSOVER_WIDE_CONTACT_MODEL = Object.freeze({
    ...CROSSOVER_REDUCED_MODEL, contactMaxSpacing:20
});

// Opt-in manual PD preset from the complete 023 crossover replay. Keep the
// fine mesh and geometric closure; the polar and coarse-mesh trials failed.
// This is a functional candidate, not a Newton equilibrium certificate.
export const CROSSOVER_PROJECTIVE_MODEL = Object.freeze({
    adaptiveMesh:null,
    contactCandidateMargin:.01,
    projectiveDynamics:true,
    physicsOptions:Object.freeze({
        omitInactiveSampleRows:true,
        projective:Object.freeze({iterations:16,hardClosure:true,
            maxLengthError:1e-5,maxPenetration:1e-5,maxShear:1e-5,
            reuseGeometry:true,scratchRotations:true,fastRotations:false})
    })
});
