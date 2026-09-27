import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {calibrateStentGrafts} from '../src/devices/stentGraftCalibration.js';
import {AORTIC_NECK,DevicePath} from '../src/devices/stentGraftPaths.js';
import {worldBodyDimensions,nominalPartRadius,deliveryRadiusMm} from '../src/devices/stentGraftModels.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {fixture} from './helpers/stentGraftFixture.js';

test('22 mm reference section calibrates scale without dependence on centerline eccentricity',()=>{
 const geometry=new THREE.CylinderGeometry(8.8,8.8,40,128).toNonIndexed();geometry.translate(...AORTIC_NECK.toArray());geometry.boundsTree=new MeshBVH(geometry);
 try {
  for(const x of [0,2]) {
   const route=new DevicePath([-15,15].map(y=>AORTIC_NECK.clone().add(new THREE.Vector3(x,y,0))));
   const c=calibrateStentGrafts({geometry},route);
   assert.ok(Math.abs(c.worldUnitsPerMm-.8)<.0002);
  }
 }finally{geometry.dispose();}
});

test('both actual anatomies map the same infrarenal neck to 22 mm',()=>{
 const measured=[];
 for(const variant of ['Aorta_plain','Aorta_infrarenal_aneurysm']) {
  const f=fixture(variant);
  try {
   f.system.ensureRoutes();const c=f.system.calibration;
   assert.ok(Math.abs(c.measuredDiameter-17.138)<.02);
   assert.equal(c.referenceDiameterMm,22);
   assert.ok(Math.abs(c.measuredDiameter/c.worldUnitsPerMm-22)<1e-10);
   f.system.load('right','body');const d=f.system.accesses.right.device;
   assert.equal(d.diameter,28);assert.equal(d.length,103,'catalogue selection stays in nominal mm');
   assert.ok(Math.abs(nominalPartRadius(d,0,0)*2/c.measuredDiameter-28/22)<1e-10);
   assert.ok(Math.abs(worldBodyDimensions(d).length-103*c.worldUnitsPerMm)<1e-10);
   measured.push(c.worldUnitsPerMm);
  }finally{f.dispose();}
 }
 assert.ok(Math.abs(measured[0]-measured[1])<1e-6);
});

for(const type of ['body','limb'])test(`${type}: calibration agrees in folded preview, released geometry and delivery diameter`,()=>{
 const {system,device:d}=previewFixture('right',type,false,type==='limb'?'ETLW1624C124EE':undefined);
 try {
  d.dimensionScale=.78;system.refreshDelivery('right');
  const before=d.foldedPreview.slice(0,type==='body'?3:1).map(p=>p.scaffoldRings.map(r=>r.restLength));
  assert.ok(Math.abs(deliveryRadiusMm(d)/deliveryRadiusMm({...d,dimensionScale:1})-.78)<1e-10);
  system.deploy('right');
  assert.deepEqual(d.parts.map(p=>p.scaffoldRings.map(r=>r.restLength)),before);
  if(type==='body')assert.ok(d.crownCaptured.distanceTo(d.deliveryPath.sample(d.position+12*.78))<1e-10);
  assert.ok(Math.abs(d.parts[0].rowRadii[0]-d.diameter/2*.78)<1e-10);
  assert.ok(Math.abs(Math.max(...d.parts.map(p=>p.releaseOffset+p.path.length))-d.length*.78)<1e-10);
  system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});
  if(type==='body')system.updateAccess('right',2,null,{deviceId:d.id,release:'tip'});
  assert.equal(d.phase,'deployed');
  for(const part of d.parts) {
   assert.ok(Array.from(part.target).every(Number.isFinite));
   assert.ok(part.scaffoldRings.every(r=>Math.abs(r.lengthError)<.0001));
  }
 }finally{system.dispose();}
});

for(const scale of [1,.7789879466966056])test(`IIs 23 mm: bifurcation span is oval, not an oversized round tube (scale ${scale})`,()=>{
 const {system,device:d}=previewFixture('right','body',false);
 try {
  system.setDiameter('right',23);d.dimensionScale=scale;system.deploy('right');
  const [trunk,ipsi,contra]=d.parts,frame=trunk.ringFrames.at(-1),center=trunk.points.at(-1);
  const wide=contra.points[0].clone().sub(ipsi.points[0]);wide.addScaledVector(frame.tangent,-wide.dot(frame.tangent)).normalize();
  const depth=frame.tangent.clone().cross(wide).normalize(),xs=[],ys=[];
  for(let j=0;j<trunk.sides;j++) {
   const q=new THREE.Vector3().fromArray(trunk.target,((trunk.rows-1)*trunk.sides+j)*3).sub(center);
   xs.push(q.dot(wide));ys.push(q.dot(depth));
  }
  const width=Math.max(...xs)-Math.min(...xs),height=Math.max(...ys)-Math.min(...ys);
  assert.ok(width<=28*scale+.01);assert.ok(height<=14*scale+.01);
  assert.ok(height<width*.7,'the crotch must not inflate into a circular 28 mm tube');
  assert.equal(ipsi.rowRadii.at(-1),7*scale);assert.equal(contra.rowRadii.at(-1),7*scale);
  assert.ok(Math.abs(2*contra.rowRadii.at(-1)/(22*scale)-14/22)<1e-10);
 }finally{system.dispose();}
});
