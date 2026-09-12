import assert from 'node:assert/strict';
import test from 'node:test';
import {resolve} from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';
const root=resolve(process.env.OET_CONE_STORAGE_SOURCE_ROOT??fileURLToPath(new URL('../',import.meta.url)));
const source=p=>pathToFileURL(resolve(root,p));
const {EndovascularPhysicsWorld}=await import(source('src/physics/endovascularPhysicsWorld.js'));
const {beginKirchhoffCoupledBoundaryStep,applyKirchhoffCoupledBoundaryMultipliers}=await import(source('src/physics/kirchhoffCoupledBoundaryRows.js'));
const {evaluateKirchhoffSurfaceFrictionKKT}=await import(source('src/physics/kirchhoffSurfaceFriction.js'));
const NORMAL=0.016447099738834046;
for(const kind of ['wall','tool'])for(const widened of [false,true])test((widened?'owned Float64 counterfactual ':'native ')+kind+' normal bank preserves the solved multiplier and cone',()=>{
 const world=new EndovascularPhysicsWorld(),inner=world.createRod('storage-inner',2,1),outer=world.createRod('storage-outer',2,1);
 const joint={innerBody:inner,outerBody:outer};
 const owner=kind==='wall'?inner:world.addToolContact(inner,outer,{friction:.08});
 if(widened){if(kind==='wall')owner.wallLambda=Float64Array.from(owner.wallLambda);else owner.lambdas=Float64Array.from(owner.lambdas);}
 const state=beginKirchhoffCoupledBoundaryStep(joint);
 const row={kind,side:0,node:0,owner,lambda:0,alpha:0,lower:0,upper:Infinity,activeHint:true,gradients:[],reactionWrenches:[{side:0,node:0,fx:1,fy:0,fz:0,mx:0,my:0,mz:0},{side:1,node:0,fx:-1,fy:0,fz:0,mx:0,my:0,mz:0}]};
 state.rows.push(row);
 const mu=kind==='tool'?.08:.2,tangent=new Float64Array([mu*NORMAL,0]);
 applyKirchhoffCoupledBoundaryMultipliers(joint,new Float64Array([NORMAL]),1);
 const stored=(kind==='wall'?owner.wallLambda:owner.lambdas)[0];
 if(kind==='tool')assert.equal(owner._jointReactions.get(0).normalWrenches[0].fx,NORMAL,'applied reaction journal keeps the Float64 solved impulse');
 const cone=evaluateKirchhoffSurfaceFrictionKKT(tangent,[0,0],stored,[mu,mu]).coneViolation;
 assert.equal(stored,NORMAL,'normal-only rounding changes the cone after the solver has applied its full Float64 impulse');
 assert.ok(cone<=1e-9,'stored normal and final tangent must retain the solved cone');
});
