import test from 'node:test';
import assert from 'node:assert/strict';
import { BufferGeometry, Float32BufferAttribute } from 'three';
import { defineKirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { createSharedAxisNative, captureSharedAxisNative, feedSharedAxisNative } from '../src/physics/kirchhoffSharedAxisNative.js';
import { createSharedAxisVesselDiscovery, relaxSharedAxisWithContacts } from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import { createSharedAxisLayout, createSharedAxisLinear, solveSharedAxisLinear } from '../src/physics/kirchhoffSharedAxisLinear.js';

test('a fourth normal at one point replaces the active basis without singular KKT rows',()=>{
    const layout=createSharedAxisLayout([['wire']]),p=layout.positions[1];
    const dofs=[0,1,2,p,p+1,p+2];
    const rows=[[1,0,0],[0,1,0],[0,0,1],[1,1,1]].map((n,i)=>({
        kind:'wall',dofs,jacobian:[0,0,0,...n],gap:i===3?-1:0,multiplier:i===3?0:1
    }));
    const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
    for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=1;
    const fixed=new Uint8Array(layout.dofCount).fill(1);fixed.fill(0,p,p+3);
    // Objective 1/2 |x + (1,1,1)|², incoming reactions (1,1,1).
    const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{
        rows,gradient:new Float64Array(layout.dofCount),fixed,tolerance:1e-9
    });
    assert.ok(result.converged,JSON.stringify(result));
    for(let j=0;j<3;j++)assert.ok(Math.abs(result.increment[p+j]-1/3)<1e-9);
    rows.forEach((r,i)=>{
        const gap=r.gap+r.dofs.reduce((sum,d,k)=>sum+r.jacobian[k]*result.increment[d],0);
        const reaction=r.multiplier+result.multiplierIncrement[i];
        assert.ok(gap>=-1e-9&&reaction>=-1e-9&&Math.abs(gap*reaction)<1e-9);
    });
});

function corner() {
    const geometry=new BufferGeometry();geometry.setAttribute('position',new Float32BufferAttribute([
        -1000,1,-1000,1000,1,-1000,0,1,1000,
        -1000,-1000,1,1000,-1000,1,0,1000,1],3));
    return {fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,i,out){
        const face=y[1]>z[1]?0:1,distance=1-(face===0?y[1]:z[1]);
        out.segmentT=1;out.faceIndex=face;out.signedDistance=distance;out.signedGap=distance-r[0];
        out.inward.x=0;out.inward.y=face===0?-1:0;out.inward.z=face===1?-1:0;return out;
    }};
}
const beam=defineKirchhoffMaterialProfile({id:'corner-beam',sampleEI1:()=>1e5,sampleGJ:()=>1e5});

test('a two-face corner retains both normal reactions; release solves the new global direction',()=>{
    const field=corner(),s=createSharedAxisNative({tools:[{id:'wire',insertion:30,type:beam,radius:.4}],
        wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
    s.loads[s.layout.positions.at(-1)+1]=15;s.loads[s.layout.positions.at(-1)+2]=20;
    const r=relaxSharedAxisWithContacts(s);assert.ok(r.converged,JSON.stringify(r));
    assert.ok(r.geometryRestarts>=2);assert.ok(Math.abs(s.positions.at(-1)[1]-.6)<1e-5&&Math.abs(s.positions.at(-1)[2]-.6)<1e-5);
    assert.ok(s.definitions.filter((d,i)=>d.id&&s.multipliers[i]>0).length>=2);
    s.loads.fill(0);const released=relaxSharedAxisWithContacts(s);assert.ok(released.converged,JSON.stringify(released));
    assert.ok(Math.hypot(s.positions.at(-1)[1],s.positions.at(-1)[2])<1e-5);
    const saved=captureSharedAxisNative(s);feedSharedAxisNative(s,{wire:31});assert.deepEqual(captureSharedAxisNative(s),saved);
    field.fallbackGeometry.dispose();
});


test('local discovery restarts preserve the converged corner response and final forces',()=>{
    const results=[];
    for(const localContactRestarts of [false,true]) {
        const field=corner(),s=createSharedAxisNative({tools:[{id:'wire',insertion:30,type:beam,radius:.4}],wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
        s.loads[s.layout.positions.at(-1)+1]=15;s.loads[s.layout.positions.at(-1)+2]=20;
        const result=relaxSharedAxisWithContacts(s,{localContactRestarts});
        assert.ok(result.converged,JSON.stringify(result));assert.ok(result.geometryRestarts>=2);
        assert.ok(result.residual.force<=1e-6&&result.residual.torque<=1e-6&&result.residual.length<=1e-5);
        results.push({positions:structuredClone(s.positions),result});field.fallbackGeometry.dispose();
    }
    results[0].positions.forEach((p,i)=>p.forEach((v,k)=>assert.ok(Math.abs(v-results[1].positions[i][k])<1e-5)));
});
