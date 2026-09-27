import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {graftBranchContactAt,branchRecoveryPotential} from '../src/devices/stentGraftBranchContact.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {createSharedAxisNative,feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
const beam=defineKirchhoffMaterialProfile({id:'ipsilateral-test',sampleEI1:()=>1e3,sampleGJ:()=>1e3});
function tube(offset=0) {
 const geometry=new THREE.CylinderGeometry(3,3,40,24,1,true).rotateZ(-Math.PI/2).translate(50,offset,0);
 // CylinderGeometry duplicates cap seam vertices: reindex it as two rings.
 const positions=[],indices=[];
 for(const x of [30,70])for(let j=0;j<24;j++)positions.push(x,offset+3*Math.cos(j*Math.PI/12),3*Math.sin(j*Math.PI/12));
 for(let j=0;j<24;j++){const a=j,b=(j+1)%24;indices.push(a,a+24,b,b,a+24,b+24);}
 geometry.dispose();
 return {positions,indices,centers:[[30,offset,0],[70,offset,0]]};
}
function surface(branch) {
 const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(branch.positions,3));geometry.setIndex(branch.indices);geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 return {geometry,bounds:geometry.boundingBox,revision:1,ownedBranches:[branch]};
}

test('branch contact uses the real spatial lumen, remains free inside, and has two open ends',()=>{
 const branches=[tube()];
 for(const x of [30.1,40,60,69.9]) {
  assert.ok(graftBranchContactAt(branches,[x,1,0],.4445).gap>0);
  const q=graftBranchContactAt(branches,[x,5,0],.4445);assert.ok(q.outside&&q.gap<0&&q.normal[1]<-.99);
 }
 for(const x of [-20,29,71,120])assert.equal(graftBranchContactAt(branches,[x,5,0],.4445),null,'no remote tether or end cap');
});

for(const side of ['right','left'])test(`${side}: ownership follows the long leg, survives release, and is absent on the other access`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  for(let i=0;i<700&&d.phase!=='deployed';i++)system.updateAccess(side,1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  assert.equal(d.phase,'deployed');
  const own=system.mechanicalSurfaceForAccess(side),other=system.mechanicalSurfaceForAccess(side==='right'?'left':'right');
  assert.equal(own.ownedBranches.length,1);assert.equal(other.ownedBranches.length,0);assert.equal(own.lumenSections.length,0);
  const ipsi=d.parts[1],contra=d.parts[2],q=ipsi.path.sample(ipsi.path.length*.75);
  assert.ok(graftBranchContactAt(own.ownedBranches,q.toArray(),.4445).gap>0);
  const wrong=contra.path.sample(contra.path.length*.85);
  assert.ok(graftBranchContactAt(own.ownedBranches,wrong.toArray(),.4445)?.outside,'incoming divider cannot select the short leg for its own wire');
  assert.equal(graftBranchContactAt(other.ownedBranches,wrong.toArray(),.4445),null,'contralateral cannulation is not rerouted');
  const top=d.parts[0].points[0];assert.equal(graftBranchContactAt(own.ownedBranches,top.toArray(),.4445),null);
 }finally{system.dispose();}
});

test('dynamic release of a divider across the delivery wire recovers into the ipsilateral lumen and permits sliding',()=>{
 const branch=tube(-3),s=surface(branch);
 let state=createSharedAxisNative({startCoordinate:-20,tools:[{id:'wire',insertion:100,type:beam,radius:.4445}],spacing:2,samplePosition:x=>[x,1,0]});
 try {
  for(let step=0;step<100;step++) {
   const sampler=createStentGraftContacts(s,state);
   state={...state,wallSamples:[sampler],graftRevision:s.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0}));
   const iterator=advanceSharedAxis(state,{wire:0},1/60,tools,{maxIterations:160,forceTolerance:1e-5,lengthTolerance:1e-5});
   let next;do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({step,result:next.value.result}));
   state=next.value.state;
  }
  for(let i=0;i<state.positions.length;i++)if(state.coordinates[i]>=36&&state.coordinates[i]<=64) {
   const q=graftBranchContactAt([branch],state.positions[i],.4445);
   assert.ok(q&&!q.outside&&q.gap>-.03,JSON.stringify({s:state.coordinates[i],q,p:state.positions[i]}));
  }
  for(let i=0;i<30;i++) {
   const sampler=createStentGraftContacts(s,state);
   state={...state,wallSamples:[sampler],graftRevision:s.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.insertion+(i<15?.2:-.2)}));
   const iterator=advanceSharedAxis(state,{wire:0},1/60,tools,{maxIterations:160,forceTolerance:1e-5,lengthTolerance:1e-5});
   let next;do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({i,result:next.value.result}));
   state=next.value.state;
  }
 }finally{s.geometry.dispose();}
});


test('branch recovery has consistent energy, forces and tangent near both open ports',()=>{
 const branches=[tube()],h=1e-5;
 for(const x of [30.7,50,69.3]) {
  const p=[x,4,.2],contact=graftBranchContactAt(branches,p,.4445),r=branchRecoveryPotential(contact,p,.4945);
  for(let i=0;i<3;i++) {
   const a=p.slice(),b=p.slice();a[i]-=h;b[i]+=h;
   const ra=branchRecoveryPotential(contact,a,.4945),rb=branchRecoveryPotential(contact,b,.4945);
   assert.ok(Math.abs((rb.energy-ra.energy)/(2*h)-r.gradient[i])<1e-6);
   for(let j=0;j<3;j++)assert.ok(Math.abs((rb.gradient[j]-ra.gradient[j])/(2*h)-r.hessian[j][i])<1e-6);
  }
 }
});

test('withdrawing the original wire releases branch ownership and does not reassign a later wire',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let i=0;i<700&&d.phase!=='deployed';i++)system.updateAccess('right',1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  const old=system.mechanicalSurfaceForAccess('right');assert.equal(old.ownedBranches.length,1);
  const entry=d.implantPosition-d.parts[1].releaseOffset-d.parts[1].path.length;
  const source=length=>({nodes:[new THREE.Vector3(),new THREE.Vector3(0,length,0)],coordinate:i=>i*length,catheterMm:0});
  system.updateAccess('right',1/60,source(Math.max(0,entry-3)));
  const cleared=system.mechanicalSurfaceForAccess('right');assert.equal(cleared.ownedBranches.length,0);
  assert.notEqual(cleared.revision,old.revision,'the solver must wake for a changed ownership constraint');
  assert.equal(old.ownedBranches.length,1,'an in-flight Newton snapshot is immutable');
  system.updateAccess('right',1/60,source(d.implantPosition+50));
  assert.equal(system.mechanicalSurfaceForAccess('right').ownedBranches.length,0);
 }finally{system.dispose();}
});

for(const side of ['right','left'])test(`${side}: a wire placed in the short outlet recovers through the actual bifurcated fabric`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  for(let i=0;i<700&&d.phase!=='deployed';i++)system.updateAccess(side,1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  const own=system.mechanicalSurfaceForAccess(side),contra=d.parts[2];
  const axis=contra.points[0].clone().sub(contra.points.at(-1)).normalize(),origin=contra.points.at(-1).clone().addScaledVector(axis,-35);
  let state=createSharedAxisNative({startCoordinate:-20,tools:[{id:'wire',insertion:100,type:beam,radius:.4445}],spacing:2,samplePosition:s=>origin.clone().addScaledVector(axis,s).toArray()});
  for(let i=0;i<180;i++) {
   const sampler=createStentGraftContacts(own,state);
   state={...state,wallSamples:[sampler],graftRevision:own.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0}));
   const iterator=advanceSharedAxis(state,{wire:0},1/60,tools,{maxIterations:160,forceTolerance:1e-5,lengthTolerance:1e-5});
   let next;do{next=iterator.next();}while(!next.done);
   const {result}=next.value;assert.ok(next.value.state&&result.converged,JSON.stringify({side,i,result}));
   state=next.value.state;
  }
  const mid=d.parts[1].path.sample(d.parts[1].path.length*.65);
  const candidates=state.positions.filter(p=>Math.abs(new THREE.Vector3(...p).sub(mid).dot(axis))<5);
  assert.ok(candidates.length>0);
  for(const p of candidates) {
   const q=graftBranchContactAt(own.ownedBranches,p,.4445);
   assert.ok(q&&!q.outside&&q.gap>-.03,JSON.stringify({p,q}));
  }
  for(let i=0;i<30;i++) {
   const sampler=createStentGraftContacts(own,state);
   const source={...state,wallSamples:[sampler],graftRevision:own.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.insertion+(i<15?.2:-.2)}));
   const iterator=advanceSharedAxis(source,{wire:0},1/60,tools,{maxIterations:160,forceTolerance:1e-5,lengthTolerance:1e-5});
   let next;do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({side,i,result:next.value.result}));
   state=next.value.state;
  }
 }finally{system.dispose();}
});

test('the ipsilateral contact appears during release and covers only the exposed part of the leg',()=>{
 const {system,device:d}=previewFixture();
 try {
  let found=false;
  for(let i=0;i<600;i++) {
   system.updateAccess('right',1/30,null,{deviceId:d.id,release:'sheath'});
   const branches=system.mechanicalSurfaceForAccess('right')?.ownedBranches??[];
   if(branches.length) {
    assert.equal(d.phase,'deploying');
    assert.ok(branches[0].centers.length<d.parts[1].rows);
    found=true;break;
   }
  }
  assert.ok(found,'ownership must be established before full deployment');
 }finally{system.dispose();}
});

test('rejection replay retains the owning branch and reconstructs identical recovery forces',async()=>{
 const {captureSharedAxisReplay,restoreSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
 const {createSharedAxisContacts}=await import('../src/physics/kirchhoffSharedAxisContacts.js');
 const branch=tube(-3),s=surface(branch),sheath={start:[0,1,0],end:[10,1,0],innerRadius:2,proximalExtension:40};
 let state=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:[{id:'wire',insertion:100,type:'glidewire',radius:.4445}],spacing:2});
 let restored;
 try {
  state.wallSamples.push(createStentGraftContacts(s,state));state.graftRevision=s.revision;
  state=feedSharedAxisNative(state,{});
  const report=JSON.parse(JSON.stringify(captureSharedAxisReplay(state,sheath)));
  restored=restoreSharedAxisReplay(report,null);
  const sample=restored.wallSamples.find(s=>s.graftSurface);
  assert.deepEqual(sample.surface.ownedBranches,[branch]);
  const p=[50,1,0],a=graftBranchContactAt([branch],p,.4445),b=graftBranchContactAt(sample.surface.ownedBranches,p,.4445);
  assert.deepEqual(branchRecoveryPotential(a,p,.4945),branchRecoveryPotential(b,p,.4945));
 }finally{restored?.graftReplayGeometry?.dispose();s.geometry.dispose();}
});
