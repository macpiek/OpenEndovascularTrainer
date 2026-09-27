import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {compliantGraftSurface} from '../src/devices/stentGraftCompliance.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';

const type=defineKirchhoffMaterialProfile({id:'apposition-wire',sampleEI1:()=>1000,sampleGJ:()=>1000});
function fixture() {
    // A graft wall 0.7 mm off a rigid vessel wall; the 0.889 mm wire cannot
    // fit in the original gap. The cloth faces outward towards z=0.
    const geometry=new THREE.PlaneGeometry(60,30,60,20).rotateY(Math.PI).translate(45,0,.7);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    const surface={geometry,bounds:geometry.boundingBox.clone(),revision:1,contains:p=>p.x>=15&&p.x<=75&&p.z>.7};
    const vessel=({b,radius})=>({gap:b[2]-radius,jacobian:[0,0,0,0,0,1]});
    vessel.sharedAxisGeometryOnly=true;
    vessel.vesselField={querySphere:p=>({inside:p.z>=0,signedDistance:p.z,normal:new THREE.Vector3(0,0,1)})};
    return {surface,vessel};
}
for(const radius of [.4445,.8333])test(`apposed cloth opens locally for radius ${radius}, supports it and allows withdrawal`,()=>{
    const {surface,vessel}=fixture();
    let state=createSharedAxisNative({tools:[{id:'wire',insertion:12,type,radius}],spacing:1,wallSamples:[vessel],samplePosition:x=>[x,0,radius]});
    let lifted=false,force=false;
    const solve=insertion=>{
        const sample=createStentGraftContacts(surface,state),source={...state,wallSamples:[vessel,sample],graftRevision:1,graftRecovery:sample.recovery};
        lifted||=sample.contactPatches.length>0;
        const tools=[{...state.materials[0].spec,insertion,rotation:0}];
        const iterator=advanceSharedAxis(source,{wire:0},1/60,tools,{forceTolerance:1e-5,lengthTolerance:1e-5});let next;
        do{next=iterator.next();}while(!next.done);
        assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({insertion,result:next.value.result}));
        state=next.value.state;
        assert.ok(state.positions.every(p=>p[2]>=radius-1e-5),'wire cannot gain space by crossing the vessel');
        force||=state.multipliers.some(x=>Math.abs(x)>1e-5);
        const ray=new THREE.Ray();
        for(let i=1;i<state.positions.length;i++) {
            const a=new THREE.Vector3(...state.positions[i-1]),b=new THREE.Vector3(...state.positions[i]);
            ray.origin.copy(a);ray.direction.copy(b).sub(a).normalize();
            assert.equal(sample.surface.geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,a.distanceTo(b)),null,'no piercing of displaced fabric');
        }
    };
    try {
        for(let x=12.25;x<=25;x+=.25)solve(x);
        for(let x=24.75;x>=12;x-=.25)solve(x);
        assert.ok(lifted&&force,'the opening has an elastic reaction, not disabled collisions');
        assert.equal(state.wallSamples.find(s=>s.graftSurface).contactPatches.length,0,'cloth returns after withdrawal');
    }finally{surface.geometry.dispose();}
});
test('intraluminal wire cannot request an exterior apposition opening',()=>{
    const {surface,vessel}=fixture();
    const state=createSharedAxisNative({tools:[{id:'wire',insertion:25,type,radius:.4445}],spacing:1,wallSamples:[vessel],samplePosition:x=>[x+20,0,1]});
    try{assert.equal(compliantGraftSurface(surface,state).patches.length,0);}finally{surface.geometry.dispose();}
});
