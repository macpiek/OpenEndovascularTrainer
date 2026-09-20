import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
// Discovery certificates are intentionally retained across private attempts;
// they are geometry caches, not committed pose, reactions or rate history.
const physicalReplay=(s,sheath)=>{
    const {discoveryState,insideContinuation,...physical}=captureSharedAxisReplay(s,sheath);
    return physical;
};
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const load=name=>JSON.parse(gunzipSync(readFileSync(new URL(`./fixtures/shared-axis/${name}.json.gz`,import.meta.url))));

for(const name of ['anatomy-wire-569.80-poor-prediction','anatomy-catheter-withdraw-180.80-sign-proof'])
test(`Reusing assembly after friction refresh preserve every direction and complete physical output: ${name}`,()=>{
    const fixture=load(name),req=fixture.stepRequest,outputs=[];
    for(const reuseFrictionAssembly of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=physicalReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true,wasmLinearAssembly:true,earlyPredictorFallback:true,reuseFrictionAssembly,
            observeTrial:e=>{
                if(e.kind==='direction') {
                    assert.equal(e.state.chain.hessianValid,true,'Each Newton/GN solve needs a valid full matrix');
                    trace.push({kind:e.kind,iteration:e.iteration,method:e.method,converged:e.direction.converged,failure:e.direction.failure,
                        increment:Array.from(e.direction.increment??[])});
                } else if(e.kind==='trial')trace.push({kind:e.kind,iteration:e.iteration,method:e.method,trial:e.trial,scale:e.scale,accept:e.accept,
                    energy:e.candidate.energy,force:e.candidate.force,torque:e.candidate.torque,constraint:e.candidate.constraint});
            }});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;assert.ok(state,JSON.stringify(result));
        assert.deepEqual(physicalReplay(input,fixture.sheath),before);
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        assert.ok(Math.abs(result.timings.assemblyMs-result.timings.tangentAssemblyMs-result.timings.residualAssemblyMs)<1e-6,
            'Assembly timing categories must survive discovery and friction/substep wrappers');
        outputs.push({trace,state:physicalReplay(state,fixture.sheath),result});
    }
    const [reference,lazy]=outputs;
    assert.deepEqual(lazy.trace,reference.trace);
    assert.deepEqual(lazy.state,reference.state,'Pose, reactions, frames, velocities and friction history must match exactly');
    for(const key of ['quality','residual','iterations','factorizations','backtracks','geometryRestarts','substepAttempts'])
        assert.deepEqual(lazy.result[key],reference.result[key],key);
    assert.ok(lazy.result.fullAssemblies<reference.result.fullAssemblies);
    assert.ok(lazy.result.residualAssemblies>reference.result.residualAssemblies);
});
