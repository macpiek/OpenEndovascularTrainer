import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
const displacement=(a,b)=>{
 let max=0;for(let i=0;i<a.length;i+=3)max=Math.max(max,Math.hypot(a[i]-b[i],a[i+1]-b[i+1],a[i+2]-b[i+2]));return max;
};
const snapshot=d=>({cloth:d.sewnMesh.geometry.attributes.position.array.slice(),
 metal:d.parts.map(p=>p.rings.geometry.attributes.instanceStart.array.slice())});
for(const side of ['right','left'])test(`${side}: small cover steps cannot create multi-millimetre jumps in committed cloth or sewn stents`,()=>{
 const {system,device:d}=previewFixture(side);let previous=snapshot(d),cloth=0,metal=0;
 try {
  for(let step=0;step<850;step++) {
   system.updateAccess(side,1/60,null,{deviceId:d.id,release:'sheath'});
   const next=snapshot(d);cloth=Math.max(cloth,displacement(next.cloth,previous.cloth));
   for(let k=0;k<3;k++)metal=Math.max(metal,displacement(next.metal[k],previous.metal[k]));
   previous=next;
  }
  assert.ok(cloth<.405,`committed cloth jumps ${cloth} mm`);
  assert.ok(metal<.85,`metal jumps ${metal} mm`);
  // The cover can stop while the tissue/metal continues settling in simulation
  // time. Completion must wait for that same mechanical surface.
  system.updateAccess(side,1,null,{deviceId:d.id,release:'tip'});
  for(let i=0;i<300&&d.phase!=='deployed';i++)system.updateAccess(side,1/60);
  assert.equal(d.phase,'deployed');
 }finally{system.dispose();}
});
test('paused simulation does not relax geometry and recapture returns to a covered state',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let i=0;i<360;i++)system.updateAccess('right',1/60,null,{deviceId:d.id,release:'sheath'});
  const before=snapshot(d);system.updateAccess('right',0,null,{deviceId:d.id,release:'sheath'});
  assert.deepEqual(snapshot(d),before);
  for(let i=0;i<360;i++)system.updateAccess('right',1/60,null,{deviceId:d.id,release:'resheath'});
  for(let i=0;i<90;i++)system.updateAccess('right',1/60);
  assert.ok(d.parts.every(p=>p.exposure.every(v=>v===0)));
  assert.ok(d.parts.every(p=>p.scaffoldRings.every(r=>Math.abs(r.lengthError)<1e-4)));
 }finally{system.dispose();}
});
