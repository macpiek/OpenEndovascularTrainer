import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const makeTools = () => [
    {id:'wire',type:'lunderquist-double-curved',length:2600,insertion:710},
    {id:'catheter',type:'stentgraft-delivery',length:1000,insertion:127.5},
    {id:'sheath',type:'introducer-sheath',length:450,insertion:128}
];
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-10,`${a} != ${b}`);
function comparePhysical(a,b) {
    assert.deepEqual(a.coordinates,b.coordinates);
    a.positions.forEach((p,i)=>p.forEach((v,k)=>close(v+a.origin[k],b.positions[i][k]+b.origin[k])));
    assert.deepEqual(a.velocities,b.velocities);
    assert.deepEqual(a.angularVelocities,b.angularVelocities);
    for(let i=0;i<a.materials.length;i++) {
        assert.deepEqual(a.materials[i].spec,b.materials[i].spec);
        for(const key of ['orientationX','orientationY','orientationZ','orientationW'])
            a.materials[i].body[key].forEach((v,n)=>close(v,b.materials[i].body[key][n]));
        for(const key of ['materialCoordinate','restRotation1','restLength','kirchhoffBendCompliance1'])
            assert.deepEqual(a.materials[i].body[key],b.materials[i].body[key]);
    }
}

test('three coaxial tools rebase at the interior sheath/delivery outlet without changing mechanics',()=>{
    for(const order of [[0,1,2],[2,0,1]]) {
        const tools=makeTools(),original=createSharedAxisNative({tools:order.map(i=>tools[i]),rebaseNearTips:true,
            origin:[-73,-383,14],samplePosition:s=>[.2*s,s,-.1*s]});
        const saved=JSON.stringify({positions:original.positions,origin:original.origin});
        const feed={catheter:127.916666666667};
        const reference=feedSharedAxisNative({...original,rebaseNearTips:false},feed);
        const next=feedSharedAxisNative(original,feed);
        comparePhysical(next,reference);
        sampleSharedAxisPosition(original,128).forEach((v,k)=>close(next.origin[k],v));
        assert.ok(next.coordinates.includes(feed.catheter),'the short material cell is not snapped away');
        assert.equal(saved,JSON.stringify({positions:original.positions,origin:original.origin}),'accepted state remains immutable');
        const withdrawn=feedSharedAxisNative(next,{catheter:127.7});
        comparePhysical(withdrawn,feedSharedAxisNative({...next,rebaseNearTips:false},{catheter:127.7}));
    }
});

test('separated outlets do not recenter, while two nearby distal tools still do',()=>{
    const far=createSharedAxisNative({tools:makeTools().map(t=>({...t,insertion:t.id==='catheter'?120:t.insertion})),rebaseNearTips:true});
    assert.deepEqual(feedSharedAxisNative(far,{}).origin,far.origin);
    const pair=createSharedAxisNative({tools:makeTools().slice(0,2).map(t=>({...t,insertion:t.id==='wire'?128:127.5})),rebaseNearTips:true});
    const next=feedSharedAxisNative(pair,{catheter:127.9});
    comparePhysical(next,feedSharedAxisNative({...pair,rebaseNearTips:false},{catheter:127.9}));
    sampleSharedAxisPosition(pair,128).forEach((v,k)=>close(next.origin[k],v));
});
