import test from 'node:test';
import assert from 'node:assert/strict';
import { createSharedAxisContacts } from '../src/physics/kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, feedSharedAxisNative, relaxSharedAxisNative, rotateSharedAxisNative, assembleSharedAxisNative } from '../src/physics/kirchhoffSharedAxisNative.js';
const sheath={start:[0,0,0],end:[40,0,0],innerRadius:1.5,proximalExtension:40};

test('rounded sheath outlet does not introduce an almost zero length cell',()=>{
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath:{...sheath,end:[69.99999999999996,0,0]}}),
        tools:[{id:'wire',insertion:100},{id:'catheter',insertion:70}]});
    for(let i=1;i<s.coordinates.length;i++)assert.ok(s.coordinates[i]-s.coordinates[i-1]>1e-9);
    assert.ok(relaxSharedAxisNative(s).converged);
});

test('zero insertion starts inside the introducer and both materials can emerge, retreat and rotate independently',()=>{
    let s=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:[{id:'wire',insertion:0},{id:'catheter',insertion:0}]});
    const check=()=>{const r=relaxSharedAxisNative(s);assert.ok(r.converged,JSON.stringify(r));
        for(const row of assembleSharedAxisNative(s).rows.filter(r=>r.kind==='wall'))assert.ok(row.gap>=-1e-5);};
    check();
    for(const wire of [10,20,30,40,50,60]){s=feedSharedAxisNative(s,{wire});check();}
    for(const catheter of [10,20,30,40,45,50,55,60,40,20,0]){s=feedSharedAxisNative(s,{catheter});check();}
    rotateSharedAxisNative(s,'wire',.05);check();
});

test('native capsule geometry is queried only beyond the sheath and sees the outer owner radius',()=>{
    const calls=[],field={queryCapsuleSoA(x,y,z,r,index,out,...flags){calls.push({r:r[0],a:x[0],b:x[1],flags});out.segmentT=.25;out.signedGap=3;out.signedDistance=3+r[0];out.inward.x=0;out.inward.y=1;out.inward.z=0;return out;}};
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field}),tools:[{id:'wire',insertion:70,radius:.4},{id:'catheter',insertion:50,radius:.8}]});
    assembleSharedAxisNative(s);
    assert.equal(calls.length,6);assert.deepEqual(calls.map(c=>c.r),[.8,.8,.4,.4,.4,.4]);
    assert.ok(calls.every(c=>c.a>=40));
});
