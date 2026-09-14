import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const load=name=>JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url)));

for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming','anatomy-berenstein-feed-312.87-projection'])
test(`specialized projection preserves every direction/acceptance decision and complete physical output: ${name}`,()=>{
    const fixture=load(name),req=fixture.stepRequest,outputs=[];
    for(const projectionMode of [false,'reduced']) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode,
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
    if(name.endsWith('projection'))assert.ok(lazy.result.timings.projectionMs>0,'This fixture must exercise nonlinear trial corrections');

    assert.equal(lazy.result.residualAssemblies,reference.result.residualAssemblies);
});


test('projection refreshes gaps and fixed masks, keeps coupled global reactions, and removes only uncoupled spins',async()=>{
    const {iterateSharedAxisProjection}=await import('../src/physics/kirchhoffSharedAxisProjection.js');
    const {createSharedAxisLayout}=await import('../src/physics/kirchhoffSharedAxisLinear.js');
    const layout=createSharedAxisLayout([['wire','catheter'],['wire','catheter']]);
    const p=Array.from(layout.positions),fixed=new Uint8Array(layout.dofCount);fixed[p[0]]=1;
    const definitions=[{kind:'length',dofs:[p[0],p[1]],jacobian:[-1,1],gap:-.03,multiplier:7},
        {kind:'length',dofs:[p[1],p[2]],jacobian:[-1,1],gap:.01,multiplier:-2},
        {kind:'wall',id:'floor',dofs:[p[2]+1],jacobian:[1],gap:-.02,multiplier:3}];
    const s={layout,definitions,fixed};
    const solve=mode=>{const it=iterateSharedAxisProjection(s,{rows:definitions},{mode});let next;do{next=it.next();}while(!next.done);return next.value;};
    for(const change of [false,true]) {
        if(change){definitions[0].gap=-.06;fixed[p[0]]=0;}
        const reference=solve('reference'),expected=Array.from(reference.increment,v=>v+0),dual=Array.from(reference.multiplierIncrement,v=>v+0);
        assert.ok(reference.converged);
        for(const mode of ['band','reduced']) {
            const result=solve(mode);assert.ok(result.converged);assert.ok(result.residual<=1e-10);
            assert.deepEqual(Array.from(result.increment,v=>v+0),expected);
            assert.deepEqual(Array.from(result.multiplierIncrement,v=>v+0),dual);
            for(const indices of layout.spins.values())for(const i of indices)assert.equal(result.increment[i],0);
        }
    }
    const reused=s.projectionWorkspace;definitions[2].gap=-.04;
    const refreshed=solve('reduced');assert.ok(refreshed.converged);assert.equal(s.projectionWorkspace,reused);
    assert.ok(Math.abs(refreshed.increment[p[2]+1]-.04)<1e-10);
    const before=s.projectionWorkspace;definitions.push({kind:'wall',id:'other',dofs:[p[1]+2],jacobian:[1],gap:-.01,multiplier:0});
    const result=solve('reduced');assert.ok(result.converged);
    assert.notEqual(s.projectionWorkspace,before);assert.equal(result.zeroReactions.length,definitions.length);
});

test('borrowed basis storage survives smaller projection indexing and return to large physical indexing',async()=>{
    const {prepareSharedAxisActiveBasis}=await import('../src/physics/kirchhoffSharedAxisActiveBasis.js');
    const owner=new Uint8Array(12),small=new Uint8Array(3);
    const row=dofs=>({kind:'length',dofs,jacobian:dofs.map(()=>1),gap:0,multiplier:0});
    const prepare=(fixed,rows,activeSet)=>prepareSharedAxisActiveBasis({fixed,rows,activeSet,dual:new Float64Array(3),basisWorkspaceKey:owner,basisCache:Symbol('fresh-linearization')});
    assert.ok(prepare(owner,[row([11]),row([0]),row([1])],new Uint8Array([1,0,0])).converged);
    assert.ok(prepare(small,[row([0]),row([1]),row([2])],new Uint8Array([1,1,1])).converged);
    const result=prepare(owner,[row([8]),row([9]),row([8,9])],new Uint8Array([1,1,1]));
    assert.equal(result.converged,false);assert.equal(result.failure,'incompatible-active-constraints');
});
