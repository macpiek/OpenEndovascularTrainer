import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';

const sheath={start:[0,0,0],end:[20,0,0],innerRadius:2,proximalExtension:40};
const field={voxelSize:1,queryCapsuleSoA(x,y,z,r,edge,out){
    Object.assign(out,{signedDistance:100,signedGap:100-r[edge],segmentT:.5,faceIndex:0});return out;
}};
const tools=[{id:'wire',type:'glidewire',insertion:70},{id:'catheter',type:'straight',insertion:18}];
const rowKey=r=>`${r.kind}/${r.edge}/${r.sample}/${r.sampleT}/${r.id}`;
const make=omitInactiveSampleRows=>createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field}),tools,omitInactiveSampleRows});

test('sample domains remove only constant inactive rows; exposed and straddling edges remain',()=>{
    const full=make(false),compact=make(true);
    const keys=new Set(compact.definitions.map(rowKey));let removed=0;
    for(const row of full.definitions){
        if(keys.has(rowKey(row)))continue;
        removed++;assert.equal(row.kind,'wall');
        const sample=full.wallSamples[row.sample],e=row.edge;
        const contact=sample({state:full,a:full.positions[e],b:full.positions[e+1],edge:e,radius:.8,
            coordinateA:full.coordinates[e],coordinateB:full.coordinates[e+1],sampleT:row.sampleT,needHessian:true});
        assert.equal(contact.gap,1);assert.ok(contact.jacobian.every(v=>v===0));assert.equal(contact.hessian,undefined);
    }
    assert.ok(removed>10);assert.deepEqual(compact.coordinates,full.coordinates);
    const nearTips=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field}),
        tools:tools.map(t=>({...t,insertion:t.id==='wire'?20:19.9995})),omitInactiveSampleRows:true});
    const fractional=nearTips.definitions.filter(row=>row.sampleT!==undefined);
    assert.ok(fractional.length>0,'Retain the fractional catheter endpoint within the introducer');
    // A replacement/custom sampler without the contract must never be omitted.
    const custom=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools,omitInactiveSampleRows:true,
        wallSamples:[()=>({gap:1,jacobian:[0,0,0,0,0,0]})]});
    assert.equal(custom.definitions.filter(r=>r.kind==='wall').length,custom.coordinates.length-1);
});

test('compact topology roundtrips and feed restores newly exposed domains',()=>{
    const compact=make(true),saved=captureSharedAxisReplay(compact,sheath);
    assert.equal(saved.omitInactiveSampleRows,true);
    const restored=restoreSharedAxisReplay(JSON.parse(JSON.stringify(saved)),field);
    assert.deepEqual(captureSharedAxisReplay(restored,sheath),saved);
    const extended=feedSharedAxisNative(restored,{wire:95,catheter:35});
    assert.equal(extended.omitInactiveSampleRows,true);
    for(const r of extended.definitions.filter(r=>r.sample===0))assert.ok(extended.coordinates[r.edge]<20);
    for(const e of extended.coordinates.keys())if(extended.coordinates[e+1]>20)
        assert.ok(extended.definitions.some(r=>r.edge===e&&r.sample===1),'Every exposed edge keeps vessel discovery');
    const legacy=make(false),old=captureSharedAxisReplay(legacy,sheath);
    assert.equal(old.omitInactiveSampleRows,undefined);
    assert.deepEqual(captureSharedAxisReplay(restoreSharedAxisReplay(old,field),sheath),old);
});

test('forward and reverse feed through the sheath preserve accepted positions and physical checks',()=>{
    const contexts=[false,true].map(omitInactiveSampleRows=>({state:make(omitInactiveSampleRows),rotations:{wire:0,catheter:0},omitInactiveSampleRows}));
    for(const insertion of [19,21,25,18,35,70]){
        for(const context of contexts){
            const targets=tools.map(t=>({...t,rotation:0,insertion:t.id==='wire'?insertion+10:insertion}));
            const iterator=advanceSharedAxis(context.state,context.rotations,1/60,targets,
                {omitInactiveSampleRows:context.omitInactiveSampleRows,forceTolerance:1e-6,lengthTolerance:1e-5});
            let next;do{next=iterator.next();}while(!next.done);
            const {state,result,rotations}=next.value;
            assert.ok(state&&result.converged,result.status);assert.ok(result.quality.finite);
            assert.ok(result.certificateBound<=1e-6&&result.quality.maxPenetration<=1e-5);
            context.state=state;context.rotations=rotations;
        }
        assert.deepEqual(contexts[1].state.coordinates,contexts[0].state.coordinates);
        for(let i=0;i<contexts[0].state.positions.length;i++)
            assert.ok(Math.hypot(...contexts[1].state.positions[i].map((v,k)=>v-contexts[0].state.positions[i][k]))<1e-9);
    }
});
