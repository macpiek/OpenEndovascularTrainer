import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {createStentGraftContacts} from '../../src/devices/stentGraftContacts.js';
const geometry=new THREE.CylinderGeometry(4,4,30,16,1,true).rotateZ(-Math.PI/2).translate(40,0,0);
geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
const surface={geometry,bounds:geometry.boundingBox,revision:1};
function prepare() {
 const refs=[];
 let state=createSharedAxisNative({tools:[{id:'wire',insertion:80}],spacing:5});
 for(let i=0;i<80;i++) {
  refs.push(new WeakRef(state));
  const sample=createStentGraftContacts(surface,state);
  sample.adaptiveContactKnots({contactMargin:1,shapeTolerance:.15});
  state={...createSharedAxisNative({tools:[{id:'wire',insertion:80}],spacing:5}),wallSamples:[sample],graftRevision:1,graftRecovery:sample.recovery};
 }
 return {refs,state};
}
const {refs,state}=prepare();
for(let i=0;i<8;i++){await new Promise(resolve=>setImmediate(resolve));global.gc();}
assert.equal(refs.filter(r=>r.deref()).length,0,'current cloth sampler must not retain preceding solver states / WASM arenas');
assert.ok(state.wallSamples[0].adaptiveContactKnots({contactMargin:1,shapeTolerance:.15}) instanceof Set);
assert.doesNotThrow(()=>state.wallSamples[0]({state,a:[30,0,0],b:[40,0,0],radius:.4445,coordinateA:30,coordinateB:40}));
geometry.dispose();
console.log('80 previous solver states collected; current contacts remain usable');
