import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {buildGraftPart} from '../src/devices/stentGraftPart.js';
import {StentGraftSurface} from '../src/devices/stentGraftSurface.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {ContrastVolumeRenderer} from '../src/contrast/contrastVolumeRenderer.js';
import {applyStentGraftFlow,graftFluidContactField} from '../src/contrast/stentGraftFlow.js';

function tree(){
 const points=[[0,0,0],[20,0,0],[50,0,0],[80,0,0],[100,0,0],[50,25,0],[10,25,0],[-20,0,0]];
 return [[7,0],[0,1],[1,2],[2,3],[3,4],[2,5],[0,6]].map(([a,b],id)=>({id,nodeStartId:a,nodeEndId:b,start:new THREE.Vector3(...points[a]),end:new THREE.Vector3(...points[b]),radiusStart:b<5?10:3,radiusEnd:b<5?10:3}));
}
function graft({offset=0,radius=5,revision=1,sealed=true}={}) {
 const material=new THREE.MeshBasicMaterial(),part=buildGraftPart({points:[20,40,60,80].map(x=>new THREE.Vector3(x,offset,0)),radius,wallFit:{fit:p=>p},fabricMaterial:material});
 const surface=new StentGraftSurface([{id:1,type:'limb',parts:[part]}],revision);surface.sealed=sealed;
 return {surface,dispose(){surface.dispose();part.mesh.geometry.dispose();material.dispose();}};
}
const mass=n=>n.getIodineMassMg()+n.outletIodineMassMg+(n.stentGraftRemodeling?.trappedIodineMassMg??0);

test('offset graft uses its actual lumen, covers a lateral ostium but preserves proximal and distal branches',()=>{
 const n=new ContrastFlowNetwork(tree(),{rootPoint:new THREE.Vector3()}),g=graft({offset:4});
 try {
  assert.ok(g.surface.sectionAt(new THREE.Vector3(50,0,0)),'old 2.5 mm centreline cutoff must not reject an offset lumen');
  n.depositIodine(0,0,100);assert.ok(applyStentGraftFlow(n,g.surface));
  const blocked=n.edges.find(e=>e.end.y===25&&e.end.x===50),proximal=n.edges.find(e=>e.end.y===25&&e.end.x===10),distal=n.edges.find(e=>e.end.x===100);
  assert.ok(blocked.transportExcluded);assert.equal(blocked.meanFlowMm3PerS,0);
  assert.ok(proximal.meanFlowMm3PerS>0&&distal.meanFlowMm3PerS>0);
  assert.ok(n.stentGraftRemodeling.coveredEdges>=2);
  for(let i=0;i<90;i++)n.update(1/30);
  assert.ok(Math.abs(mass(n)-100)<1e-8);assert.ok(blocked.massMg.every(v=>v===0));
 }finally{g.dispose();}
});

test('open gate keeps bypass branches patent; only locally apposed fabric occludes an ostium before seal',()=>{
 for(const radius of [5,9.5]) {
  const n=new ContrastFlowNetwork(tree(),{rootPoint:new THREE.Vector3()}),g=graft({radius,sealed:false});
  try {assert.ok(applyStentGraftFlow(n,g.surface));const branch=n.edges.find(e=>e.end.y===25&&e.end.x===50);
   assert.equal(!!branch.transportExcluded,radius>9);
   assert.ok(n.edges.filter(e=>e.graftCovered).every(e=>e.totalVolume===n._preGraftGeometry[e.index].totalVolume),'open gate does not prematurely exclude sac volume');
  }finally{g.dispose();}
 }
});

test('new surface revisions conserve trapped iodine and are idempotent without repeatedly shrinking vessels',()=>{
 const n=new ContrastFlowNetwork(tree(),{rootPoint:new THREE.Vector3()}),g=graft();
 try {
  for(const e of n.edges)n.depositIodine(e.index,0,10);
  const before=mass(n);applyStentGraftFlow(n,g.surface);
  const held=n.stentGraftRemodeling.trappedIodineMassMg,volumes=n.edges.map(e=>e.totalVolume);
  for(let i=0;i<5;i++) {g.surface.revision++;assert.ok(applyStentGraftFlow(n,g.surface));assert.equal(applyStentGraftFlow(n,g.surface),false);}
  assert.deepEqual(n.edges.map(e=>e.totalVolume),volumes);assert.equal(n.stentGraftRemodeling.trappedIodineMassMg,held);assert.ok(Math.abs(mass(n)-before)<1e-10);
  const blocked=n.edges.find(e=>e.transportExcluded);
  n.setFlowOverride([blocked.index],1000);n.addFaceFlowDelta(blocked.index,0,1,1000);
  assert.equal(n.getSignedFlowMm3PerS(blocked.index),0);assert.equal(n.getFaceSignedFlowMm3PerS(blocked.index,0),0);
  n.depositIodine(blocked.index,0,2);n.update(.1);
  assert.ok(!n._activeEdgeIndices.has(blocked.index));assert.ok(Math.abs(mass(n)-before-2)<1e-8);
 }finally{g.dispose();}
});

test('GPU samples match CPU concentrations and unchanged simulation frames only change interpolation uniforms',()=>{
 const system=new HybridContrastSystem({centerlineSegments:tree(),flowOptions:{rootPoint:new THREE.Vector3()}}),n=system.flowNetwork;
 n.depositIodine(0,0,100);n.update(.01);const r=new ContrastVolumeRenderer(system);
 try {
  const expected=Float32Array.from(r._flowVertexConcentration),mask=r._flowVertexIsJunctionConnector;
  r.update({reuseUnchanged:true});
  for(let i=0;i<expected.length;i++) {
   const slot=r._flowVertexConcentrationSampleSlot[i],value=Math.max(r._flowSampleData[slot*4],mask[i]?r._flowVertexConcentration[i]:0);
   assert.ok(Math.abs(value-expected[i])<1e-6);
  }
  const version=r._flowSampleTexture.version,old=r._flowSampleData.slice();
  system._solverAccumulator=1/60;r.update({reuseUnchanged:true});
  assert.equal(r._flowSampleTexture.version,version);assert.deepEqual(r._flowSampleData,old);assert.equal(r.flowTubeMaterial.uniforms.displayAlpha.value,.5);
 }finally{r.dispose();}
});

test('deployed contrast uses actual graft mesh rather than clipping the artery into irregular facets',()=>{
 const system=new HybridContrastSystem({centerlineSegments:tree(),flowOptions:{rootPoint:new THREE.Vector3()}}),g=graft({offset:4}),r=new ContrastVolumeRenderer(system);
 try {
  system.flowNetwork.depositIodine(0,0,10);system.setStentGraftSurface(g.surface);r.update();
  assert.equal(r._graftMeshes.length,1);assert.deepEqual(r._graftMeshes[0].geometry.attributes.position.array,g.surface.parts[0].target);
  for(let i=0;i<r._flowVertexConcentration.length;i++)if(system.flowNetwork.edges[r._flowVertexConcentrationEdgeIndex[i]].transportExcluded)assert.equal(r._flowVertexConcentration[i],0);
  g.surface.revision++;system.setStentGraftSurface(g.surface);r.update();
  assert.equal(r.group.children.filter(m=>m.name==='contrast-graft-lumen').length,1);
 }finally{r.dispose();g.dispose();}
});

test('local contrast parcels cannot cross either side of fabric, while open portals remain passable',()=>{
 const g=graft({sealed:false});
 const anatomy={querySphere(p,r,out){return Object.assign(out,{signedDistance:100,signedGap:100-r,inside:true,violation:false,penetration:0,inward:{x:0,y:0,z:0},normal:{x:0,y:0,z:0},closestPoint:{x:0,y:0,z:0},target:{...p}});}};
 try {
  const field=graftFluidContactField(anatomy,g.surface);
  const inside=field.querySphereFrom(new THREE.Vector3(50,5.2,0),.05,{},new THREE.Vector3(50,0,0));
  assert.equal(inside.violation,true);assert.ok(inside.target.y<5);
  const outside=field.querySphereFrom(new THREE.Vector3(50,4.8,0),.05,{},new THREE.Vector3(50,7,0));
  assert.equal(outside.violation,true);assert.ok(outside.target.y>4.9);
  assert.equal(field.querySphereFrom(new THREE.Vector3(85,0,0),.05,{},new THREE.Vector3(79,0,0)).violation,false);
  assert.equal(field.querySphereFrom(new THREE.Vector3(21,0,0),.05,{},new THREE.Vector3(19,0,0)).violation,false,'a proximal pigtail bolus can enter through the open inlet');
 }finally{g.dispose();}
});

test('plume-to-column display deposition is continuous and preserves parcel mass between cells',()=>{
 const system=new HybridContrastSystem({centerlineSegments:tree(),flowOptions:{rootPoint:new THREE.Vector3()}}),r=new ContrastVolumeRenderer(system),local=system.localSolver;
 try {
  local.count=1;local.iodineMassMg[0]=1;local.positionX[0]=-15;local.positionY[0]=0;local.positionZ[0]=0;
  r.update();const before=r._flowCellLocalPlumeMassMg.slice();
  local.positionX[0]+=.01;r.update();const after=r._flowCellLocalPlumeMassMg;
  assert.ok(Math.abs(after.reduce((a,b)=>a+b,0)-1)<1e-7);
  assert.ok(after.reduce((a,b,i)=>a+Math.abs(b-before[i]),0)<.02,'a moving parcel cannot jump entirely to the next cell');
  assert.equal(r.flowTubeMaterial.premultipliedAlpha,true,'optical density is not multiplied by opacity a second time');
 }finally{r.dispose();}
});

function bifurcatedOpenGraft() {
 const nodes=[[-20,0,0],[20,0,0],[50,0,0],[80,-6,0],[110,-12,0],[70,6,0],[110,12,0]];
 const segments=[[0,1],[1,2],[2,3],[3,4],[2,5],[5,6]].map(([a,b],id)=>({id,nodeStartId:a,nodeEndId:b,start:new THREE.Vector3(...nodes[a]),end:new THREE.Vector3(...nodes[b]),radiusStart:10,radiusEnd:10}));
 const material=new THREE.MeshBasicMaterial();
 const make=(points,radius)=>buildGraftPart({points:points.map(p=>new THREE.Vector3(...p)),radius,wallFit:{fit:p=>p},fabricMaterial:material});
 const parts=[make([[20,0,0],[35,0,0],[50,0,0]],7),make([[48,-2,0],[65,-4,0],[80,-6,0]],4),make([[48,2,0],[60,4,0],[70,6,0]],4)];
 const surface=new StentGraftSurface([{id:1,type:'body',parts}],1);
 const system=new HybridContrastSystem({centerlineSegments:segments,flowOptions:{rootPoint:new THREE.Vector3(-20,0,0),cardiacOutputMlPerMin:600,axialDispersionMm2PerS:0}});
 system.setStentGraftSurface(surface);
 return {system,surface,dispose(){surface.dispose();for(const part of parts)part.mesh.geometry.dispose();material.dispose();}};
}

test('proximal bolus reaches open gate through graft before entering the aneurysm sac; sealing stops the leak',()=>{
 const f=bifurcatedOpenGraft(),n=f.system.flowNetwork,r=new ContrastVolumeRenderer(f.system);
 try {
  const sac=n.stentGraftRemodeling.sac;assert.ok(sac,'open main-body gate has a separate external transport compartment');
  n.depositIodine(0,0,100);
  for(let i=0;i<3;i++)n.update(.01);
  assert.equal(sac.receivedMassMg,0,'no shortcut from proximal inlet to aneurysm');
  assert.equal(n.stentGraftRemodeling.trappedIodineMassMg,0);
  r.update();assert.equal(r._trappedMesh.visible,false);
  let firstGraft=-1,firstSac=-1;
  for(let i=0;i<1200;i++) {
   n.update(.01);
   if(firstGraft<0&&n.edges.some(e=>e.graftCovered&&e.massMg.some(m=>m>1e-4)))firstGraft=i;
   if(firstSac<0&&sac.receivedMassMg>1e-4)firstSac=i;
  }
  assert.ok(firstGraft>=0&&firstSac>firstGraft+10,`${firstGraft} graft then ${firstSac} sac`);
  assert.ok(sac.receivedMassMg>1);assert.ok(sac.drainedMassMg>0,'sac has a distal washout route');
  assert.ok(Math.abs(mass(n)-100)<1e-7);assert.ok(sac.cells.every(c=>c.entry.mass[c.cellIndex]>=0));
  r.update();assert.equal(r._trappedMesh.visible,true,'new sac filling appears without a geometry revision');
  for(let slot=0;slot<r._flowConcentrationSampleValue.length;slot++) {
   const ei=r._flowConcentrationSampleEdgeIndex[slot],e=n.edges[ei],t=r._flowConcentrationSampleEdgeT[slot];
   if(e.graftSections?.[Math.min(e.cellCount-1,Math.floor(t*e.cellCount))]) {
    f.system.simulationTimeSeconds++;r.update({reuseUnchanged:true});
    assert.equal(r._flowSampleData[slot*4],0,'native tube cannot reuse intragraft concentration');break;
   }
  }
  f.surface.sealed=true;f.surface.revision++;f.system.setStentGraftSurface(f.surface);
  assert.equal(n.stentGraftRemodeling.sac,null);
  const held=n.stentGraftRemodeling.trappedIodineMassMg;
  n.depositIodine(0,0,10);for(let i=0;i<120;i++)n.update(1/30);
  assert.equal(n.stentGraftRemodeling.trappedIodineMassMg,held);
  assert.ok(Math.abs(mass(n)-110)<1e-7);
 }finally{r.dispose();f.dispose();}
});

test('sac exchange stays nonnegative and conservative with a tiny annular cell and a long step',async()=>{
 const {createGraftSacTransport}=await import('../src/contrast/graftSacTransport.js');
 const f=bifurcatedOpenGraft(),n=f.system.flowNetwork,m=n.stentGraftRemodeling;
 try {
  const inlet=m.sac.cells[m.sac.inlet];inlet.entry.volumes[inlet.cellIndex]=1e-5;
  m.sac=createGraftSacTransport(n,m);const cell=m.sac.cells[m.sac.inlet];
  cell.entry.mass[cell.cellIndex]=10;m.trappedIodineMassMg=10;
  m.sac.update(1);
  assert.ok(m.sac.cells.every(c=>Number.isFinite(c.entry.mass[c.cellIndex])&&c.entry.mass[c.cellIndex]>=0));
  assert.ok(Math.abs(mass(n)-10)<1e-8);
 }finally{f.dispose();}
});
