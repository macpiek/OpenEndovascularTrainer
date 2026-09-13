import {captureSharedAxisNative,restoreSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,completeSharedAxisDynamicStep} from './kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction,refreshSharedAxisWallFriction,commitSharedAxisWallFriction} from './kirchhoffSharedAxisWallFriction.js';
import {iterateSharedAxisWithContacts} from './kirchhoffSharedAxisVesselWitnesses.js';

/** A full atomic dynamic step, including the external wall friction law.
 * Inter-tool constraints remain absent. Frozen friction modes are certified
 * against the final normal reactions before publishing either material. */
function* iterateTimeStepAttempt(s,dt,{feedById={},maxFrictionIterations=8,...options}={}) {
    if(!Number.isInteger(maxFrictionIterations)||maxFrictionIterations<1)throw new RangeError('Positive friction iteration limit required');
    const incoming=captureSharedAxisNative(s),acceptedSolves=s.acceptedSolves,started=performance.now();
    const history=s.wallFrictionHistory,gaps=s.acceptedWallGaps,velocities=s.velocities,angularVelocities=s.angularVelocities;
    let committed=false;
    try {
    const tolerance=options.forceTolerance??1e-6;
    if(!(Number.isFinite(tolerance)&&tolerance>0))throw new RangeError('Positive force tolerance required');
    s.cacheMechanicalAssembly=true;
    prepareSharedAxisDynamicStep(s,dt);prepareSharedAxisWallFriction(s,{feedById,liveNormalLoad:options.liveWallNormalLoad===true});
    let result,friction,iterations=0,factorizations=0,workingSetReuses=0,backtracks=0,geometryRestarts=0,frictionIterations=0;
    const timings={assemblyMs:0,linearMs:0,frictionMs:0};
    for(let outer=0;outer<maxFrictionIterations;outer++) {
        frictionIterations=outer+1;
        result=yield* iterateSharedAxisWithContacts(s,{...options,forceTolerance:tolerance});
        iterations+=result.iterations;factorizations+=result.factorizations;workingSetReuses+=result.workingSetReuses??0;backtracks+=result.backtracks;geometryRestarts+=result.geometryRestarts;
        for(const key of ['assemblyMs','linearMs'])timings[key]+=result.timings[key];
        if(!result.converged)break;
        const start=performance.now();friction=refreshSharedAxisWallFriction(s,{forceTolerance:Math.max(0,tolerance-Math.max(result.residual.force,result.residual.torque))});timings.frictionMs+=performance.now()-start;
        if(friction.converged) {
            commitSharedAxisWallFriction(s);completeSharedAxisDynamicStep(s);
            for(const b of result.quality.bodies) {
                const last=s.materials.find(m=>m.spec.id===b.id).last;
                b.maxSpeed=Math.max(...s.velocities.slice(0,last+1).map(v=>Math.hypot(...v)));
            }
            committed=true;
            return {...result,certificateBound:Math.max(result.residual.force,result.residual.torque)+friction.forceChange,iterations,factorizations,workingSetReuses,backtracks,geometryRestarts,timings,friction,frictionIterations:outer+1,ms:performance.now()-started};
        }
        result={...result,converged:false,status:'wall-friction-iteration-limit'};
    }
    return {...result,converged:false,frictionIterations,iterations,factorizations,workingSetReuses,backtracks,geometryRestarts,timings,friction,ms:performance.now()-started};
    } finally {
        s.cacheMechanicalAssembly=false;s.mechanicalAssemblyCache=null;s.wallGeometryCache=null;
        if(!committed){restoreSharedAxisNative(s,incoming);s.acceptedSolves=acceptedSolves;s.acceptedWallGaps=gaps;
            s.wallFrictionHistory=history;s.velocities=velocities;s.angularVelocities=angularVelocities;s.wallFrictionStep=null;s.dynamicStep=null;}
    }
}
/** Live load coupling is a faster direction strategy for the same wall law.
 * If its nonsymmetric active set fails, the attempt above has already restored
 * the original physical state/history. Retry the complete dt with frozen-load
 * outer corrections, retaining only discovered static geometry. Cancellation
 * cannot enter this fallback: generator.return unwinds the attempt's finally. */
export function* iterateSharedAxisTimeStep(s,dt,options={}) {
    const first=yield* iterateTimeStepAttempt(s,dt,options);
    if(first.converged||options.liveWallNormalLoad!==true||options.wallNormalFallback===false)return first;
    const fallback=yield* iterateTimeStepAttempt(s,dt,{...options,liveWallNormalLoad:false});
    const result={...fallback,wallNormalFallback:{attempted:true,liveFailure:first.status,converged:fallback.converged}};
    for(const key of ['iterations','factorizations','workingSetReuses','backtracks','geometryRestarts','frictionIterations','ms'])result[key]=(first[key]??0)+(fallback[key]??0);
    result.timings=Object.fromEntries(Object.keys(fallback.timings).map(key=>[key,(first.timings[key]??0)+fallback.timings[key]]));
    return result;
}
export function stepSharedAxis(s,dt,options={}) {
    const iterator=iterateSharedAxisTimeStep(s,dt,options);let next;
    do{next=iterator.next();}while(!next.done);return next.value;
}
