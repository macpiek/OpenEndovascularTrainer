import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {ADAPTIVE_SOLVE_OPTIONS} from '../src/physics/kirchhoffSharedAxisAdaptiveMesh.js';
import {createSharedAxisNative,feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';

function bentSurface() {
 const positions=[],indices=[],sides=24;
 for(const [x,y] of [[25,0],[30.25,.12],[35,0]])for(let j=0;j<sides;j++)positions.push(x,y+3*Math.cos(j*2*Math.PI/sides),3*Math.sin(j*2*Math.PI/sides));
 for(let r=0;r<2;r++)for(let j=0;j<sides;j++){const a=r*sides+j,b=r*sides+(j+1)%sides;indices.push(a,a+sides,b,b,a+sides,b+sides);}
 const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));geometry.setIndex(indices);geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 return {geometry,bounds:geometry.boundingBox,revision:1};
}
test('adaptive remeshing retains an accepted bend at graft cloth, including a former moving tip',()=>{
 const surface=bentSurface();
 let state=createSharedAxisNative({tools:[{id:'wire',insertion:100,radius:.4445}],spacing:5,adaptiveMesh:true,spatialKnots:[0,5,25,30.25,35,40,100],samplePosition:x=>[x,-2.99+(x===30.25?.12:0),0]});
 try {
  const sampler=createStentGraftContacts(surface,state);
  state={...state,wallSamples:[sampler],graftRevision:1,graftRecovery:[]};
  const next=feedSharedAxisNative(state,{wire:99.8});
  assert.ok(next.coordinates.includes(30.25),'contact bend must survive fine resampling and coarsening');
  for(let e=0;e<next.coordinates.length-1;e++)assert.doesNotThrow(()=>sampler({state:next,a:next.positions[e],b:next.positions[e+1],radius:.4445,coordinateA:next.coordinates[e],coordinateB:next.coordinates[e+1]}));
 }finally{surface.geometry.dispose();}
});

test('an accepted incoming overlap survives branch-recovery handover without freezing at zero feed',()=>{
 const surface=bentSurface();
 const previous=createSharedAxisNative({tools:[{id:'wire',insertion:100}],spacing:5,samplePosition:x=>[x,-2.99,0]});
 previous.graftRevision=1;previous.graftRecovery=[];
 try {
  const sampler=createStentGraftContacts(surface,previous);
  assert.ok(sampler.recovery.length>0,'actual existing crossings remain recoverable');
  for(let e=0;e<previous.coordinates.length-1;e++)assert.doesNotThrow(()=>sampler({state:previous,a:previous.positions[e],b:previous.positions[e+1],radius:.4445,coordinateA:previous.coordinates[e],coordinateB:previous.coordinates[e+1]}));
 }finally{surface.geometry.dispose();}
});

test('a new fabric crossing is still rejected and reports its exact segment and face',()=>{
 const surface=bentSurface();
 const previous=createSharedAxisNative({tools:[{id:'wire',insertion:100}],spacing:5});
 try {
  const sampler=createStentGraftContacts(surface,previous);
  assert.throws(()=>sampler({state:previous,a:[28,-5,0],b:[32,0,0],radius:.4445,coordinateA:28,coordinateB:32}),e=>e.code==='trial-outside-vessel'&&e.contact.face>=0&&e.contact.coordinateA===28);
 }finally{surface.geometry.dispose();}
});

for(const side of ['right','left'])test(`${side}: delivery system slides both ways inside the released ipsilateral limb with an adaptive mesh`,async()=>{
 const {previewFixture}=await import('./helpers/stentGraftPreviewFixture.js');
 const {advanceSharedAxis}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
 const {graftBranchContactAt}=await import('../src/devices/stentGraftBranchContact.js');
 const {DevicePath}=await import('../src/devices/stentGraftPaths.js');
 const {system,device:d}=previewFixture(side);
 try {
  for(let i=0;i<700&&d.phase!=='deployed';i++)system.updateAccess(side,1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  const surface=system.mechanicalSurfaceForAccess(side),part=d.parts[1];
  const centers=surface.ownedBranches[0].centers.map(p=>new THREE.Vector3(...p)).reverse();
  const axis=centers[1].clone().sub(centers[0]).normalize(),origin=centers[0].clone().addScaledVector(axis,-35);
  const route=new DevicePath([origin,...centers,...d.parts[0].points.slice().reverse()]);
  let state=createSharedAxisNative({startCoordinate:-20,tools:[{id:'wire',insertion:100,radius:.4445},{id:'catheter',insertion:65,type:'stentgraft-delivery',radius:3,deliveryExposureMm:40}],adaptiveMesh:true,spacing:2,samplePosition:s=>s<0?origin.clone().addScaledVector(axis,s).toArray():route.sample(s).toArray()});
  let contacts=0,maxNodes=0;
  for(let i=0;i<420;i++) {
   const sampler=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
   const insertion=i<180?65:i<300?65-(i-179)*.4:17+(i-299)*.4;
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.id==='catheter'?insertion:100}));
   const iterator=advanceSharedAxis(source,{wire:0,catheter:0},1/60,tools,{maxIterations:160,...ADAPTIVE_SOLVE_OPTIONS});
   let next;do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({side,i,result:next.value.result}));
   state=next.value.state;
   if(i>=180)for(let e=0;e<state.positions.length-1;e++) {
    const a=new THREE.Vector3(...state.positions[e]),b=new THREE.Vector3(...state.positions[e+1]);
    const ray=new THREE.Ray(a,b.clone().sub(a).normalize());
    assert.equal(surface.geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,a.distanceTo(b)),null,`accepted delivery shaft crosses cloth at step ${i}, edge ${e}`);
   }
   contacts+=state.graftContactStats?.contacts??0;maxNodes=Math.max(maxNodes,state.coordinates.length);
  }
  assert.ok(contacts>0,'contact forces are exercised, not disabled');
  assert.equal(state.materials.find(m=>m.spec.id==='catheter').spec.insertion,65);
  const mid=d.parts[1].path.sample(d.parts[1].path.length*.65);
  const candidates=state.positions.filter(p=>Math.abs(new THREE.Vector3(...p).sub(mid).dot(axis))<3);
  assert.ok(candidates.length>0);
  for(const p of candidates)assert.ok(!graftBranchContactAt(surface.ownedBranches,p,.4445)?.outside,'wire remains on the ipsilateral side');
  assert.ok(maxNodes<100,`contact knots must not accumulate: ${maxNodes}`);
 }finally{system.dispose();}
});
