import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {BufferGeometry,Float32BufferAttribute,Vector3,Triangle} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {createKirchhoffWallWitnessGeometryWorkspace as workspace,evaluateKirchhoffWallWitnessGeometry as evaluate} from '../src/physics/kirchhoffWallWitnessGeometry.js';
const fixture=()=>new BufferGeometry().setAttribute('position',new Float32BufferAttribute([0,0,0,1,0,0,0,1,0],3));
const close=(a,b,tol=1e-10)=>assert.ok(Math.abs(a-b)<tol,`${a} vs ${b}`);
for(const [point,foot,bary,feature] of [
 [[.2,.3,2],[.2,.3,0],[.5,.2,.3],'face'],
 [[2,2,.5],[.5,.5,0],[0,.5,.5],'edge'],
 [[-2,-1,.5],[0,0,0],[1,0,0],'vertex']])test(`retained finite triangle ${feature}`,()=>{
 const r=evaluate({geometry:fixture(),faceIndex:0,point});
 r.closestPoint.forEach((v,i)=>close(v,foot[i]));r.barycentric.forEach((v,i)=>close(v,bary[i]));
 assert.equal(r.feature,feature);close(Math.hypot(...r.direction),1);
 close(r.distance,Math.hypot(...point.map((v,i)=>v-foot[i])));
 if(feature!=='face')assert.ok(r.distance>point[2],'finite feature must not become an infinite plane');
 const h=1e-6;
 for(let axis=0;axis<3;axis++){
  const a=point.slice(),b=point.slice();a[axis]+=h;b[axis]-=h;
  close((evaluate({geometry:fixture(),faceIndex:0,point:a}).distance-evaluate({geometry:fixture(),faceIndex:0,point:b}).distance)/(2*h),r.direction[axis],1e-8);
 }
});
test('zero distance is explicitly undefined; scratch arrays and owned keys are reused safely',()=>{
 const geometry=fixture(),out=workspace();evaluate({geometry,faceIndex:0,point:[.2,.3,1]},out);
 const array=out.closestPoint,key=out.triangleKey;
 evaluate({geometry,faceIndex:0,point:[.2,.3,0]},out);
 assert.equal(out.normalDefined,false);assert.ok(out.direction.every(Number.isNaN));assert.equal(out.closestPoint,array);assert.equal(out.triangleKey,key);
 geometry.attributes.position.setZ(0,.1);assert.equal(out.triangleKey,key);
 evaluate({geometry,faceIndex:0,point:[.2,.3,1]},out);assert.notEqual(out.triangleKey,key);
 assert.throws(()=>evaluate({geometry,faceIndex:99,point:[0,0,0]}),/face/);
 geometry.attributes.position.setXYZ(2,1,0,0);assert.throws(()=>evaluate({geometry,faceIndex:0,point:[0,0,0]}),/Degenerate/);
});
test('actual aorta retained faces evaluate captured near-wall point with finite-feature gradients',()=>{
 const bytes=fs.readFileSync('res/Aorta_plain.stl'),geometry=new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
 transformAortaGeometry(geometry,generateVessel(140,0).vessel);geometry.boundsTree=new MeshBVH(geometry);
 const point=[-4.901269912719727,-236.12643432617188,11.50005054473877];
 for(const faceIndex of [325178,325433]){
  const r=evaluate({geometry,faceIndex,point}),h=1e-5;
  assert.ok(r.normalDefined);assert.ok(r.barycentric.every(v=>v>=-1e-12&&v<=1+1e-12));
  const tri=new Triangle(...[0,1,2].map(i=>new Vector3().fromBufferAttribute(geometry.attributes.position,geometry.index.getX(3*faceIndex+i))));
  const expected=tri.closestPointToPoint(new Vector3(...point),new Vector3());r.closestPoint.forEach((v,i)=>close(v,expected.getComponent(i)));
  for(let axis=0;axis<3;axis++){const a=point.slice(),b=point.slice();a[axis]+=h;b[axis]-=h;
   close((evaluate({geometry,faceIndex,point:a}).distance-evaluate({geometry,faceIndex,point:b}).distance)/(2*h),r.direction[axis],2e-8);}
 }
});
