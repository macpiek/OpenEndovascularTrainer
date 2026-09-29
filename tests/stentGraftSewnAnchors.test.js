import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';
import {ringWave} from '../src/devices/stentGraftRingKinematics.js';

for(const side of ['right','left'])test(`${side}: exposed crowns remain at their sewn material coordinates through release and rotation`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  let exposed=0;
  const material=d.parts.map(p=>p.scaffoldRings.map(r=>[r.center,r.height]));
  for(let step=0;step<45;step++) {
   system.updateAccess(side,d.sheathTravel/12/45,null,{deviceId:d.id,release:'sheath',mechanicalRotation:step/45*.4});
   for(const [pi,p]of d.parts.entries()) {
    let offset=0;
    for(const [ri,r]of p.scaffoldRings.entries()) {
     assert.deepEqual([r.center,r.height],material[pi][ri]);
     if(!r.packed) {
      exposed++;assert.equal(r.currentHeight,r.height,'no 8 → 4 → 8 mm material-height pumping');
      assert.ok(Math.abs(r.lengthError)<1e-4);
      for(let arm=0;arm<p.sides;arm++) {
       const sewn=fabricPoint(p,r.center+r.height*ringWave(arm,r.proximal),arm/p.sides*2*Math.PI);
       const rendered=new Vector3().fromBufferAttribute(p.rings.geometry.attributes.instanceStart,offset+arm*12);
       assert.ok(Math.abs(rendered.distanceTo(sewn)-(p.sewnParent ? .2 : .15))<5e-5,'crowns are attached just inside the same cloth point');
      }
     }
     offset+=r.samples;
    }
   }
  }
  assert.ok(exposed>100);
 }finally{system.dispose();}
});

test('gate marker is a thin closed ring attached to the distal fabric in packed, partial and released states',()=>{
 const {system,device:d}=previewFixture('right','body',false);
 const verify=p=>{
  const marker=p.gateMarker,a=marker.geometry.attributes.instanceStart,b=marker.geometry.attributes.instanceEnd;
  assert.equal(marker.count,p.sides*4);assert.ok(marker.userData.wireRadius<=.12);assert.ok(marker.visible);
  for(let i=0;i<marker.count;i++) {
   const x=new Vector3().fromBufferAttribute(a,i),y=new Vector3().fromBufferAttribute(b,i);
   assert.ok(x.distanceTo(fabricPoint(p,p.path.length,i/marker.count*2*Math.PI))<3e-5);
   assert.ok(y.distanceTo(new Vector3().fromBufferAttribute(a,(i+1)%marker.count))<3e-5,'closed continuous rim');
  }
 };
 try {
  system.refreshDelivery('right');verify(d.foldedPreview[2]);system.deploy('right');
  for(let step=0;step<25;step++){system.updateAccess('right',d.sheathTravel/12/25,null,{deviceId:d.id,release:'sheath'});verify(d.parts[2]);}
 }finally{system.dispose();}
});
