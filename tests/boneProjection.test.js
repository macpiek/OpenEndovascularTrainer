import test from 'node:test';
import assert from 'node:assert/strict';
import * as T from 'three';
import {createBoneProjectionMaterial} from '../src/imaging/boneProjection.js';
import {applyCArmProjection} from '../src/imaging/carmProjection.js';

test('bone boundary integration follows the physical detector on every draw without changing projection',()=>{
 const material=createBoneProjectionMaterial(), camera=new T.PerspectiveCamera(45,1,.1,1600);
 try{
  for(const detectorDistance of [200,350,600]) for(const zoom of [1,1.4,2.1]){
   applyCArmProjection(camera,350,detectorDistance,zoom,300);
   const projection=camera.projectionMatrix.toArray();
   material.onBeforeRender(null,null,camera);
   assert.equal(material.uniforms.rayNear.value,.1);
   assert.equal(material.uniforms.rayFar.value,350+detectorDistance);
   assert.deepEqual(camera.projectionMatrix.toArray(),projection);
   assert.ok(new T.Vector3(0,0,-camera.far-1).project(camera).z>1,'ordinary scene geometry stays clipped beyond detector');
  }
 }finally{material.dispose();}
});

test('bone integration retains all boundary intersections once, regardless of overlapping bones',()=>{
 const m=createBoneProjectionMaterial();
 try{assert.equal(m.side,T.DoubleSide);assert.equal(m.forceSinglePass,true);assert.equal(m.depthTest,false);assert.equal(m.depthWrite,false);
 assert.equal(m.blendEquation,T.AddEquation);assert.equal(m.blendSrc,T.OneFactor);assert.equal(m.blendDst,T.OneFactor);
 }finally{m.dispose();}
});
