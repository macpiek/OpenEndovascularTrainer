import {createSharedAxisAppSystem} from './kirchhoffSharedAxisAppSystem.js';

/** Separate experiment: the adaptive Kirchhoff reference keeps its existing
 * physics and settings. 60 Hz is this variant's target, not a performance claim.
 * New numerical strategies belong behind this factory until validated. */
export function createSharedAxisRealtimeSystem(options={}) {
    const system=createSharedAxisAppSystem({continuousSegmentContacts:'axis',coupledFrictionNewton:true,predictiveNewton:true,reuseFrictionAssembly:true,wasmLinearAssembly:true,lazyBasisCoefficients:true,deferActiveBasis:true,earlyPredictorFallback:false,...options,adaptiveMesh:options.adaptiveMesh||true,
        projectiveDynamics:false,retainDiscoveryCertificates:true,continuousDiscoverySign:true,certifiedDiscoverySamples:true});
    system.id=system.diagnostics.solver='shared-axis-realtime';
    return system;
}
