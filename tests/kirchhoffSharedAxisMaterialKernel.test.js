import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createSharedAxisNative,assembleSharedAxisNative,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {sharedAxisMaterialKernelWorkspace} from '../src/physics/kirchhoffSharedAxisMaterialKernel.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('batched Float64 material derivatives match every matrix entry for all tool types and scales',()=>{
    for(const type of ['berenstein','pigtail','sim1'])for(const insertion of [0,15,65.2])for(const scale of [1e-7,.001,.05]) {
        const s=createSharedAxisNative({startCoordinate:-10,tools:[{id:'wire',insertion:80.1,shaftStiffness:11.9,tipStiffness:14.45},
            {id:'catheter',type,insertion,shaftStiffness:40.65,tipStiffness:59.5}]});
        applySharedAxisNativeIncrement(s,Float64Array.from({length:s.layout.dofCount},(_,i)=>scale*Math.sin(i)),new Float64Array(s.multipliers.length));
        const a=assembleSharedAxisNative(s),expected={H:s.chain.tangent.slice(),g:s.chain.gradient.slice()};
        const b=assembleSharedAxisNative(s,{wasmMaterial:true});
        assert.equal(a.energy,b.energy);assert.deepEqual(s.chain.tangent,expected.H);assert.deepEqual(s.chain.gradient,expected.g);
        const w=sharedAxisMaterialKernelWorkspace(s.chain,s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0));
        assert.equal(s.chain.tangent,w.tangent,'The solver reads the WASM output without copying it');
        assembleSharedAxisNative(s,{wasmMaterial:true});
        assert.deepEqual(s.chain.tangent,expected.H,'Repeated assembly clears the previous output');
        assert.equal(sharedAxisMaterialKernelWorkspace(s.chain,w.hinges),w,'Repeated calls reuse fixed views');
        assert.throws(()=>w.memory.grow(1),RangeError,'Views cannot be detached by memory growth');
    }
});
test('independent chains and changed layouts never share scratch or overwrite another tangent',()=>{
    const make=insertion=>createSharedAxisNative({tools:[{id:'wire',insertion},{id:'catheter',insertion:insertion/2}]});
    const a=make(30),b=make(70),run=s=>assembleSharedAxisNative(s,{wasmMaterial:true});
    run(a);const saved=a.chain.tangent.slice(),memory=a.chain.tangent.buffer;run(b);
    assert.deepEqual(a.chain.tangent,saved);assert.notEqual(b.chain.tangent.buffer,memory);
    const small=sharedAxisMaterialKernelWorkspace(a.chain,1),large=sharedAxisMaterialKernelWorkspace(a.chain,4);
    assert.notEqual(small,large);assert.equal(small.records[0].data.length,91,'Reallocation leaves old views valid');
});

const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
for(const name of ['anatomy-berenstein-feed-492.27-stagnation','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`WASM preserves all trial decisions and the complete certified physical step: ${name}`,()=>{
    const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest,outputs=[];
    for(const wasmMaterial of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath),trace=[];
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback:true,wasmMaterial,
            observeTrial:e=>{
                if(e.kind==='direction')trace.push({kind:e.kind,iteration:e.iteration,method:e.method,converged:e.direction.converged,
                    failure:e.direction.failure,increment:Array.from(e.direction.increment??[])});
                else if(e.kind==='trial')trace.push({kind:e.kind,iteration:e.iteration,method:e.method,trial:e.trial,scale:e.scale,accept:e.accept,
                    energy:e.candidate.energy,force:e.candidate.force,torque:e.candidate.torque,constraint:e.candidate.constraint});
            }});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;assert.ok(state,JSON.stringify(result));
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        outputs.push({trace,state:captureSharedAxisReplay(state,fixture.sheath),result});
    }
    const [a,b]=outputs;assert.deepEqual(b.trace,a.trace);assert.deepEqual(b.state,a.state);
    for(const key of ['quality','residual','iterations','factorizations','backtracks','geometryRestarts','substepAttempts','fullAssemblies','residualAssemblies','promotedAssemblies'])
        assert.deepEqual(b.result[key],a.result[key],key);
});
test('WASM augments promoted geometry and refreshes changed material/reaction/pose inputs',()=>{
    const fixture=JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-pigtail-wire-withdraw-200.27-incoming.json',import.meta.url)));
    const s=restoreSharedAxisReplay(fixture,anatomy.field),promotion={hits:0};
    for(const change of ['none','reaction','pose','material']) {
        if(change==='reaction')s.multipliers[0]+=.1;
        if(change==='pose'){s.positions[3][0]+=.0001;s.geometryKey=Symbol('moved');}
        if(change==='material'){s.materials[0].body.kirchhoffBendCompliance1[2]*=2;s.geometryKey=Symbol('new-material');}
        assembleSharedAxisNative(s,{withTangent:false,promotion,wasmMaterial:true});
        const a=assembleSharedAxisNative(s,{wasmMaterial:false}),expected={g:s.chain.gradient.slice(),H:s.chain.tangent.slice()};
        const b=assembleSharedAxisNative(s,{promotion,wasmMaterial:true});
        assert.equal(a.energy,b.energy);assert.deepEqual(s.chain.gradient,expected.g);assert.deepEqual(s.chain.tangent,expected.H);
    }
    assert.equal(promotion.hits,4);
});
