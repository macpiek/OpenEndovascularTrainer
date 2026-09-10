import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';
import {measureKirchhoffCoupledMaterialResidual} from '../src/physics/kirchhoffCoupledResidual.js';

test('world executes the common nonlinear loop for one actual rod',()=>{
 let calls=0;
 const world=new EndovascularPhysicsWorld({coupledSystem:{independentComponents:true,
  solve(c,dt,options){calls++;assert.equal(c.bodies.length,1);return solveKirchhoffCoupledSystem(c,dt,{...options,activeCondensation:true,simultaneousCoulomb:true});},
  apply:applyKirchhoffCoupledCorrection}});
 const body=world.createRod('catheter',8,5,{sleepFrames:1e6});
 body.setPinned(0,true);body.y[4]=.01;body.previousY[4]=.01;
 world.stepFixed();
 assert.equal(world.lastCoupledSolver,'joint-components');assert.ok(calls>0);
 assert.ok(world.lastCoupledClosureConverged);
 const residual=measureKirchhoffCoupledMaterialResidual({bodies:[body]},world.fixedDt);
 assert.ok(residual.adaptationMm<=world.coupledContainmentTolerance);
 assert.ok(residual.bendTwistRad<=world.coupledAngularToleranceRad);
});

class PlaneWall {
 voxelSize=.5;
 write(p,r,out){
  const gap=-p.y-r,penetration=Math.max(0,-gap);
  Object.assign(out,{signedDistance:-p.y,signedGap:gap,penetration,inside:p.y<=0,violation:gap<0,branchId:0,faceIndex:0,source:'test-plane',timeOfImpact:gap<0?0:1});
  Object.assign(out.point,p);Object.assign(out.closestPoint,{x:p.x,y:0,z:p.z});
  Object.assign(out.normal,{x:0,y:-1,z:0});Object.assign(out.inward,out.normal);
  Object.assign(out.target,{x:p.x,y:p.y-penetration,z:p.z});return out;
 }
 querySphere(p,r,out){return this.write(p,r,out);}
 queryCapsule(a,b,r,out){const p=a.y>=b.y?a:b;this.write(p,r,out);out.segmentT=p===a?0:1;return out;}
 sweepSphere(a,b,r,out){this.write(b,r,out);const ga=-a.y-r,gb=-b.y-r;out.timeOfImpact=gb>=0?1:ga<=0?0:ga/(ga-gb);return out;}
}

test('single rod shares material and wall rows in full nonlinear world closure',()=>{
 let wallRows=0,remoteResponse=0;
 const world=new EndovascularPhysicsWorld({contactField:new PlaneWall(),coupledSystem:{independentComponents:true,
  solve(c,dt,o){wallRows+=o.additionalRows.filter(r=>r.kind==='wall').length;
   const result=solveKirchhoffCoupledSystem(c,dt,{...o,activeCondensation:true,simultaneousCoulomb:true});
   remoteResponse=Math.max(remoteResponse,Math.abs(result.inner.correction[6+1]));return result;},apply:applyKirchhoffCoupledCorrection}});
 const body=world.createRod('wall-catheter',8,5,{radius:.5,sleepFrames:1e6});
 for(let i=0;i<body.count;i++)body.setNodePosition(i,5*i,-.55,0);
 body.setPinned(0,true);body.y[5]=-.49;body.previousY[5]=-.49;
 for(let i=0;i<3;i++)world.stepFixed();
 assert.ok(wallRows>0);assert.ok(remoteResponse>0);
 assert.ok(world.lastCoupledClosureConverged);
 for(let i=0;i<body.count;i++)assert.ok(body.y[i]+body.nodeRadius[i]<=world.coupledContainmentTolerance,`wall ${i}`);
 const residual=measureKirchhoffCoupledMaterialResidual({bodies:[body]},world.fixedDt);
 assert.ok(residual.adaptationMm<=world.coupledContainmentTolerance);
 assert.ok(residual.bendTwistRad<=world.coupledAngularToleranceRad);
});

test('world switches isolated components to an existing lumen owner and back',()=>{
 const owners=[];
 const world=new EndovascularPhysicsWorld({coupledSystem:{independentComponents:true,
  solve(c,dt,o){owners.push(c);return solveKirchhoffCoupledSystem(c,dt,{...o,activeCondensation:true,simultaneousCoulomb:true});},apply:applyKirchhoffCoupledCorrection}});
 const wire=world.createRod('wire',5,5,{radius:.4445,sleepFrames:1e6}),catheter=world.createRod('catheter',5,5,{radius:.8,sleepFrames:1e6});
 const lumen=world.addContainment(wire,catheter,{innerRadius:.485,enabled:false,axialFriction:0,torsionalFriction:0});
 world.stepFixed();assert.equal(world.lastCoupledSolver,'joint-components');
 const isolated=owners.slice();assert.equal(new Set(isolated).size,2);
 owners.length=0;lumen.enabled=true;world.stepFixed();
 assert.equal(world.lastCoupledSolver,'joint');assert.ok(owners.length>0);assert.ok(owners.every(owner=>owner===lumen));
 owners.length=0;lumen.enabled=false;world.stepFixed();
 assert.equal(world.lastCoupledSolver,'joint-components');assert.ok(world.lastCoupledClosureConverged);
 assert.equal(new Set(owners).size,2);assert.ok(owners.every(owner=>isolated.includes(owner)));
 for(const body of [wire,catheter])for(const axis of ['x','y','z'])assert.ok(body[axis].every(Number.isFinite));
});

test('a certified sleeping component stays asleep while an independent tool moves',()=>{
 const solved=[];
 const world=new EndovascularPhysicsWorld({coupledSystem:{independentComponents:true,
  solve(c,dt,o){solved.push(c.bodies[0]);return solveKirchhoffCoupledSystem(c,dt,o);},apply:applyKirchhoffCoupledCorrection}});
 const idle=world.createRod('idle-wire',5,5,{sleepFrames:1e6}),moving=world.createRod('moving-catheter',5,5,{sleepFrames:1e6});
 world.stepFixed();assert.ok(world.lastCoupledClosureConverged);
 idle.sleeping=true;const before=idle.x.slice();solved.length=0;
 moving.y[2]+=.01;moving.previousY[2]=moving.y[2];
 world.stepFixed();
 assert.ok(solved.includes(moving));assert.ok(!solved.includes(idle));
 assert.equal(idle.sleeping,true);assert.deepEqual(idle.x,before);
});

test('sleep cannot hide an uncertified component failure',()=>{
 let calls=0,fail=true;
 const world=new EndovascularPhysicsWorld({coupledSystem:{independentComponents:true,
  solve(c,dt,o){calls++;const r=solveKirchhoffCoupledSystem(c,dt,o);if(fail)r.diagnostics.converged=false;return r;},apply:applyKirchhoffCoupledCorrection}});
 const body=world.createRod('retry-catheter',5,5,{sleepFrames:1});
 world.stepFixed();assert.equal(world.lastCoupledClosureConverged,false);
 body.sleeping=true;fail=false;const before=calls;world.stepFixed();
 assert.ok(calls>before);assert.ok(world.lastCoupledClosureConverged);
});

test('merging and splitting sleeping tools requires fresh equilibrium despite cached owners',()=>{
 let calls=0;
 const world=new EndovascularPhysicsWorld({coupledSystem:{independentComponents:true,
  solve(c,dt,o){calls++;return solveKirchhoffCoupledSystem(c,dt,o);},apply:applyKirchhoffCoupledCorrection}});
 const a=world.createRod('sleep-a',4,5,{sleepFrames:1e6}),b=world.createRod('sleep-b',4,5,{sleepFrames:1e6});
 const lumen=world.addContainment(a,b,{enabled:false,innerRadius:.485,axialFriction:0,torsionalFriction:0});
 world.stepFixed();a.sleeping=b.sleeping=true;let before=calls;
 lumen.enabled=true;world.stepFixed();assert.ok(calls>before);
 a.sleeping=b.sleeping=true;before=calls;lumen.enabled=false;
 world.stepFixed();assert.ok(calls>before);assert.ok(world.lastCoupledClosureConverged);
});
