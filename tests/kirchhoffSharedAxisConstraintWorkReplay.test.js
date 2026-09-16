import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {assembleSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const load=name=>JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url)));

for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`constraint work reuse preserves every direction/acceptance decision and complete physical output: ${name}`,()=>{
    const fixture=load(name),req=fixture.stepRequest,outputs=[];
    for(const reuseConstraintWork of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,reuseConstraintWork,
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
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        assert.ok(Math.abs(result.timings.assemblyMs-result.timings.tangentAssemblyMs-result.timings.residualAssemblyMs)<1e-6,
            'Assembly timing categories must survive discovery and friction/substep wrappers');
        outputs.push({trace,state:captureSharedAxisReplay(state,fixture.sheath),result});
    }
    const [reference,lazy]=outputs;
    assert.deepEqual(lazy.trace,reference.trace);
    assert.deepEqual(lazy.state,reference.state,'Pose, reactions, frames, velocities and friction history must match exactly');
    for(const key of ['quality','residual','iterations','factorizations','backtracks','geometryRestarts','substepAttempts'])
        assert.deepEqual(lazy.result[key],reference.result[key],key);
    assert.equal(lazy.result.fullAssemblies,reference.result.fullAssemblies);
    assert.equal(lazy.result.promotedAssemblies,reference.result.promotedAssemblies);
    assert.equal(lazy.result.residualAssemblies,reference.result.residualAssemblies);
});

