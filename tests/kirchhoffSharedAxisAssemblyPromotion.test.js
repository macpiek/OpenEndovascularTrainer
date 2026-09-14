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
test(`promoted trial assembly preserves every direction/acceptance decision and complete physical output: ${name}`,()=>{
    const fixture=load(name),req=fixture.stepRequest,outputs=[];
    for(const promoteTrialAssembly of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly,
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
    assert.ok(lazy.result.fullAssemblies<reference.result.fullAssemblies);
    assert.ok(lazy.result.promotedAssemblies>0);assert.equal(reference.result.promotedAssemblies,0);
    assert.ok(lazy.result.residualAssemblies>0);assert.equal(reference.result.residualAssemblies,0);
});

test('promotion rebuilds exactly the full matrix and refreshes reaction forces at the same pose',()=>{
    const fixture=load('anatomy-pigtail-wire-withdraw-200.27-incoming');
    const current=restoreSharedAxisReplay(fixture,anatomy.field),reference=restoreSharedAxisReplay(fixture,anatomy.field),promotion={hits:0};
    const light=assembleSharedAxisNative(current,{withTangent:false,promotion});
    const full=assembleSharedAxisNative(reference);
    for(const k of ['energy','force','torque','constraint'])assert.equal(light[k],full[k]);
    assembleSharedAxisNative(current,{promotion});
    assert.equal(promotion.hits,1);
    for(const k of ['gradient','tangent','hessian'])assert.deepEqual(current.chain[k],reference.chain[k],k);
    // Reaction-only changes cannot reuse cached physical force sums.
    current.multipliers[0]+=.1;reference.multipliers[0]+=.1;
    const a=assembleSharedAxisNative(current,{promotion}),b=assembleSharedAxisNative(reference);
    for(const k of ['energy','force','torque','constraint'])assert.equal(a[k],b[k]);
    for(const k of ['gradient','tangent','hessian'])assert.deepEqual(current.chain[k],reference.chain[k],k);
    assert.deepEqual(a.rows.map(r=>r.geometricHessian),b.rows.map(r=>r.geometricHessian));
    // Reuse cannot survive a changed pose token, including restored poses
    // whose old local data have already been replaced by another candidate.
    current.geometryKey=Symbol('moved');reference.geometryKey=Symbol('moved');
    current.positions[3][0]+=.0001;reference.positions[3][0]+=.0001;
    const hits=promotion.hits;assembleSharedAxisNative(current,{promotion});assembleSharedAxisNative(reference);
    assert.equal(promotion.hits,hits);
    for(const k of ['gradient','tangent','hessian'])assert.deepEqual(current.chain[k],reference.chain[k],k);
});
