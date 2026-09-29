import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createContrastReleaseSurface} from '../src/devices/stentGraftContrastSurface.js';
import {partialFlowFixture} from './helpers/partialContrastFixture.js';
import {ContrastVolumeRenderer} from '../src/contrast/contrastVolumeRenderer.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

const mass=n=>n.getIodineMassMg()+n.outletIodineMassMg+(n.stentGraftRemodeling?.trappedIodineMassMg??0);

test('partially released trunk separates contrast before any gate is released; exit is the actual cloth frontier',()=>{
 const f=partialFlowFixture(),surface=createContrastReleaseSurface([f.device],1),n=f.system.flowNetwork;
 const renderer=new ContrastVolumeRenderer(f.system);
 try {
  assert.equal(surface.parts.length,1);assert.equal(surface.parts[0].rows,3);
  assert.equal(surface.openGate.center.x,36);assert.equal(surface.sectionAt(new THREE.Vector3(45,0,0)),null);
  f.system.setStentGraftSurface(surface);assert.ok(n.stentGraftRemodeling.sac);
  n.depositIodine(0,0,100);
  let firstLumen=-1,firstSac=-1;
  for(let step=0;step<700;step++) {
   n.update(.01);
   if(firstLumen<0&&n.edges.some(e=>e.graftCovered&&e.massMg.some(v=>v>1e-4)))firstLumen=step;
   if(firstSac<0&&n.stentGraftRemodeling.sac.receivedMassMg>1e-4)firstSac=step;
   if(step===firstLumen){assert.equal(n.stentGraftRemodeling.trappedIodineMassMg,0);renderer.update();assert.equal(renderer._trappedMesh.visible,false);}
  }
  assert.ok(firstLumen>=0&&firstSac>firstLumen+3,`${firstLumen} lumen, ${firstSac} sac`);
  assert.ok(Math.abs(mass(n)-100)<1e-7);
  assert.ok(n.stentGraftRemodeling.sac.drainedMassMg>0,'a frontier inside an atlas edge still has downstream washout');
  renderer.update();assert.equal(renderer._graftMeshes[0].geometry.attributes.position.count,3*24);
 }finally{renderer.dispose();surface.dispose();f.dispose();}
});

test('release fronts and recapture preserve iodine and remove covered fabric from contrast',()=>{
 const f=partialFlowFixture(),n=f.system.flowNetwork;let surface;
 try {
  surface=createContrastReleaseSurface([f.device],1);f.system.setStentGraftSurface(surface);
  n.depositIodine(0,0,100);for(let i=0;i<40;i++)n.update(.02);
  f.parts[0].exposure.fill(1);f.parts[1].exposure.set([1,1,0]);f.parts[2].exposure.set([1,1,0]);
  const next=createContrastReleaseSurface([f.device],2);f.system.setStentGraftSurface(next);surface.dispose();surface=next;
  assert.equal(surface.openGates.length,2);assert.ok(n.stentGraftRemodeling.sac.feeds.length>=1);
  assert.ok(Math.abs(mass(n)-100)<1e-7);
  for(const p of f.parts)p.exposure.fill(0);
  const empty=createContrastReleaseSurface([f.device],3);f.system.setStentGraftSurface(empty);surface.dispose();surface=empty;
  assert.equal(n.stentGraftRemodeling.coveredEdges,0);assert.equal(n.stentGraftRemodeling.sac,null);
  assert.ok(Math.abs(mass(n)-100)<1e-7);assert.equal(f.system.localSolver.graftSurface,null);
 }finally{surface?.dispose();f.dispose();}
});

test('live deployment exposes a cached contrast snapshot before full release, using current cloth positions',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let i=0;i<15;i++)system.updateAccess('right',d.sheathTravel*.45/12/15,null,{deviceId:d.id,release:'sheath'});
  assert.equal(d.phase,'deploying');assert.equal(system.surface,null);
  const s=system.getContrastSurface(.1);assert.ok(s.parts.length>0);assert.ok(s.openGate);
  assert.ok(s.parts[0].rows<d.parts[0].rows);
  assert.deepEqual(s.parts[0].target,d.parts[0].mesh.geometry.attributes.position.array.slice(0,s.parts[0].target.length));
  assert.equal(system.getContrastSurface(.1),s,'unchanged cloth does not rebuild the Boolean mesh');
  system.updateAccess('right',.02,null,{deviceId:d.id,release:'sheath'});
  assert.equal(system.getContrastSurface(0,false),s,'no rebuild when contrast is inactive');
  assert.notEqual(system.getContrastSurface(.1),s);
 }finally{system.dispose();}
});
