import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';
const root=resolve(process.env.OET_CONE_STORAGE_SOURCE_ROOT??fileURLToPath(new URL('../',import.meta.url)));
const source=p=>pathToFileURL(resolve(root,p));
const {KirchhoffContactManifold}=await import(source('src/physics/kirchhoffContactManifold.js'));
const {buildKirchhoffSurfaceFriction,evaluateKirchhoffSurfaceFrictionKKT}=await import(source('src/physics/kirchhoffSurfaceFriction.js'));
const fixture=JSON.parse(fs.readFileSync(new URL('./fixtures/kirchhoff-cone-transport-storage.json',import.meta.url),'utf8'));
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cone=(lambda,n,mu)=>evaluateKirchhoffSurfaceFrictionKKT(lambda,[0,0],n,mu).coneViolation;
function body(){
 const b={count:2,nodeRadius:new Float64Array([.5,.5])};
 for(const axis of ['X','Y','Z']){b[axis.toLowerCase()]=new Float64Array([0,axis==='Z'?1:0]);b['previous'+axis]=b[axis.toLowerCase()].slice();}
 for(const axis of ['X','Y','Z','W'])for(const prefix of ['orientation','previousOrientation'])b[prefix+axis]=new Float64Array([axis==='W'?1:0]);
 return b;
}
function nativeReplay(){
 const manifold=new KirchhoffContactManifold({frictionCoefficient:.015});
 const contact=manifold.upsertContact({innerMaterialSegmentId:0,outerMaterialSegmentId:0,feature:'sliding-rim',innerSegmentIndex:0,outerSegmentIndex:0,normal:fixture.before.normal,tangentU:fixture.before.axes[0]});
 contact.normalLambda=fixture.normalLambda;
 for(let i=0;i<3;i++){contact.tangentU[i]=fixture.before.axes[0][i];contact.tangentV[i]=fixture.before.axes[1][i];}contact.tangentLambda.set(fixture.lambda);
 const innerBody=body(),outerBody=body();
 const joint={innerBody,outerBody,axialFriction:fixture.mu[0],torsionalFriction:fixture.mu[1]};
 const record={kind:'sliding-rim',manifoldContact:contact,_innerSegmentIndex:0,_outerSegmentIndex:0,innerWeights:[...fixture.before.weights],outerWeights:[1,0],normal:fixture.before.normal,surfaceContactPoint:fixture.before.point,surfaceAxialTangent:fixture.before.axes[0]};
 const before=buildKirchhoffSurfaceFriction(joint,record,1/120,{});
 const beforeLambda=before.rows.map(r=>r.lambda),normalBefore=contact.normalLambda;
 manifold.remapContact(contact,{normal:fixture.after.normal});
 record.normal=fixture.after.normal;record.innerWeights=[...fixture.after.weights];record.surfaceContactPoint=fixture.after.point;record.surfaceAxialTangent=fixture.after.axes[0];
 const after=buildKirchhoffSurfaceFriction(joint,record,1/120,{});
 return {contact,beforeLambda,afterLambda:after.rows.map(r=>r.lambda),normalBefore};
}
test('native Float64 manifold remap and surface refresh reproduce the lumen ellipse drift with unchanged normal load',()=>{
 const r=nativeReplay();
 assert.equal(typeof r.contact.normalLambda,'number');assert.equal(r.contact.tangentLambda.constructor,Float64Array);
 assert.equal(r.contact.normalLambda,r.normalBefore);assert.equal(r.normalBefore,fixture.normalLambda);
 assert.ok(cone(r.beforeLambda,r.normalBefore,fixture.mu)<=1e-14);
 const actual=cone(r.afterLambda,r.normalBefore,fixture.mu);
 assert.ok(actual>1e-9);assert.ok(Math.abs(actual-fixture.expectedFreshCone)<2e-14);
});
test('independent world-force projection explains the drift; a circular cone does not exhibit it',()=>{
 const force=[0,1,2].map(k=>fixture.lambda.reduce((s,v,i)=>s+v*fixture.before.axes[i][k],0));
 const transformed=fixture.after.axes.map(axis=>dot(axis,force));
 assert.ok(Math.abs(cone(transformed,fixture.normalLambda,fixture.mu)-fixture.expectedFreshCone)<2e-14);
 assert.equal(cone(fixture.lambda,fixture.normalLambda,[.015,.015]),0);
 assert.equal(cone(transformed,fixture.normalLambda,[.015,.015]),0);
 assert.ok(Math.fround(fixture.normalLambda)>fixture.normalLambda,'the actual Fn would round UP, not down');
 assert.equal(cone(fixture.lambda,Math.fround(fixture.normalLambda),fixture.mu),0);
});
