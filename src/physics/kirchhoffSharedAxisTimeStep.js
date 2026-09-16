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
    let result,friction,iterations=0,factorizations=0,workingSetReuses=0,backtracks=0,geometryRestarts=0,frictionIterations=0,fullAssemblies=0,residualAssemblies=0,promotedAssemblies=0;
    const timings={assemblyMs:0,linearMs:0,frictionMs:0,tangentAssemblyMs:0,residualAssemblyMs:0,projectionMs:0};
    for(let outer=0;outer<maxFrictionIterations;outer++) {
        frictionIterations=outer+1;
        result=yield* iterateSharedAxisWithContacts(s,{...options,forceTolerance:tolerance});
        iterations+=result.iterations;factorizations+=result.factorizations;workingSetReuses+=result.workingSetReuses??0;backtracks+=result.backtracks;geometryRestarts+=result.geometryRestarts;
        for(const key of ['assemblyMs','linearMs','tangentAssemblyMs','residualAssemblyMs','projectionMs'])timings[key]+=result.timings[key]??0;
        fullAssemblies+=result.fullAssemblies??0;residualAssemblies+=result.residualAssemblies??0;promotedAssemblies+=result.promotedAssemblies??0;
        if(!result.converged)break;
        const start=performance.now();friction=refreshSharedAxisWallFriction(s,{forceTolerance:Math.max(0,tolerance-Math.max(result.residual.force,result.residual.torque))});timings.frictionMs+=performance.now()-start;
        if(friction.converged) {
            commitSharedAxisWallFriction(s);completeSharedAxisDynamicStep(s);
            for(const b of result.quality.bodies) {
                const last=s.materials.find(m=>m.spec.id===b.id).last;
                b.maxSpeed=Math.max(...s.velocities.slice(0,last+1).map(v=>Math.hypot(...v)));
            }
            committed=true;
            return {...result,certificateBound:Math.max(result.residual.force,result.residual.torque)+friction.forceChange,iterations,factorizations,workingSetReuses,backtracks,geometryRestarts,timings,fullAssemblies,residualAssemblies,promotedAssemblies,friction,frictionIterations:outer+1,ms:performance.now()-started};
        }
        result={...result,converged:false,status:'wall-friction-iteration-limit'};
    }
    return {...result,converged:false,frictionIterations,iterations,factorizations,workingSetReuses,backtracks,geometryRestarts,timings,fullAssemblies,residualAssemblies,promotedAssemblies,friction,ms:performance.now()-started};
    } finally {
        s.cacheMechanicalAssembly=false;s.mechanicalAssemblyCache=null;s.wallGeometryCache=null;s.bufferedWallGeometryCache=null;
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
    const first=yield* iterateTimeStepAttempt(s,dt,{...options,
        earlyLiveFallback:options.earlyLiveFallback!==false&&options.liveWallNormalLoad===true&&options.wallNormalFallback!==false});
    if(first.converged||options.liveWallNormalLoad!==true||options.wallNormalFallback===false)return first;
    const fallback=yield* iterateTimeStepAttempt(s,dt,{...options,liveWallNormalLoad:false});
    const result={...fallback,wallNormalFallbacks:1,wallNormalFallback:{attempted:true,liveFailure:first.status,converged:fallback.converged,
        ...(first.detectedCycle?{detectedCycle:first.detectedCycle}:{}),...(first.detectedStagnation?{detectedStagnation:first.detectedStagnation}:{})}};
    for(const key of ['iterations','factorizations','workingSetReuses','backtracks','geometryRestarts','frictionIterations','fullAssemblies','residualAssemblies','promotedAssemblies','ms'])result[key]=(first[key]??0)+(fallback[key]??0);
    result.timings=Object.fromEntries(Object.keys(fallback.timings).map(key=>[key,(first.timings[key]??0)+fallback.timings[key]]));
    if(first.status==='live-contact-stagnation'&&!fallback.converged) {
        // A heuristic must not turn a solvable full step into a rejection or
        // force a smaller dt. Restore/retry the original strategy if its early
        // alternative failed. The guard is disabled for this one recovery.
        const recovered=yield* iterateSharedAxisTimeStep(s,dt,{...options,stagnationFallback:false});
        const merged={...recovered,wallNormalFallbacks:1+(recovered.wallNormalFallbacks??0),
            stagnationRecovery:{attempted:true,detectedStagnation:first.detectedStagnation,earlyFallbackStatus:fallback.status,converged:recovered.converged}};
        for(const key of ['iterations','factorizations','workingSetReuses','backtracks','geometryRestarts','frictionIterations','fullAssemblies','residualAssemblies','promotedAssemblies','ms'])merged[key]=(result[key]??0)+(recovered[key]??0);
        merged.timings=Object.fromEntries(Object.keys(result.timings).map(key=>[key,result.timings[key]+(recovered.timings[key]??0)]));
        return merged;
    }
    return result;
}
export function stepSharedAxis(s,dt,options={}) {
    const iterator=iterateSharedAxisTimeStep(s,dt,options);let next;
    do{next=iterator.next();}while(!next.done);return next.value;
}
