import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {assembleSharedAxisNative,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const load=name=>JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url)));

for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`lazy trial evaluation preserves every direction/acceptance decision and complete physical output: ${name}`,()=>{
    const fixture=load(name),req=fixture.stepRequest,outputs=[];
    for(const lazyTrialTangent of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,lazyTrialTangent,
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
    assert.ok(lazy.result.residualAssemblies>0);assert.equal(reference.result.residualAssemblies,0);
});

test('light evaluation keeps forces and row geometry; a subsequent full assembly restores all matrix coefficients',()=>{
    const fixture=load('anatomy-pigtail-wire-withdraw-200.27-incoming');
    const s=restoreSharedAxisReplay(fixture,anatomy.field),pose=captureSharedAxisNative(s);
    s.cacheMechanicalAssembly=true;
    const full=assembleSharedAxisNative(s),gradient=s.chain.gradient.slice(),matrix=s.chain.tangent.slice(),H=s.chain.hessian.slice();
    const light=assembleSharedAxisNative(s,{withTangent:false});
    assert.equal(s.chain.hessianValid,false);assert.deepEqual(s.chain.gradient,gradient);
    for(const k of ['energy','force','torque','constraint'])assert.equal(light[k],full[k]);
    const values=rows=>rows.map(r=>({gap:r.gap,jacobian:r.jacobian,multiplier:r.multiplier}));
    assert.deepEqual(values(light.rows),values(full.rows));
    restoreSharedAxisNative(s,pose);assembleSharedAxisNative(s);
    assert.equal(s.chain.hessianValid,true);assert.deepEqual(s.chain.tangent,matrix);assert.deepEqual(s.chain.hessian,H);
    assert.deepEqual(s.chain.gradient,gradient);
});
