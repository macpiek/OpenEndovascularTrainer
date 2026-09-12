import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';
import {solveKirchhoffAxialCoupledSystem} from '../src/physics/kirchhoffAxialCoupledSolver.js';
function pair(solve,offset,friction=0,velocity=0,spin=0){
 const calls=[];
 const world=new EndovascularPhysicsWorld({fixedDt:1/120,coupledSystem:{solve:(c,dt,o)=>{const result=solve(c,dt,{...o,sectionSpan:20,sectionScope:'all',tolerance:1e-10,activeCondensation:true,simultaneousCoulomb:true});calls.push(result.diagnostics);return result;},apply:applyKirchhoffCoupledCorrection}});
 const bodies=[0,1].map(side=>world.createRod(side?'catheter':'wire',5,5,{mass:side?3:1,radius:.4445,innerRadius:.485,
  linearDamping:1,angularDamping:1,foldLimitStrength:0,projectionVelocityRetention:1,sleepFrames:1e6}));
 for(let i=0;i<5;i++)bodies[0].setNodePosition(i,i*5,offset,0);
 bodies[0].velocityX.fill(velocity);bodies[0].angularVelocityX.fill(spin);
 world.addContainment(...bodies,{innerRadius:.485,axialFriction:friction,torsionalFriction:friction,portalFilletRadius:0,coupledBendingRateDamping:0,radialVelocityDamping:0});
 return {world,bodies,calls};
}
for(const offset of [.02,.2])test(`axial direction uses existing world integration and reciprocal contact, offset=${offset}`,()=>{
 const reference=pair(solveKirchhoffCoupledSystem,offset),axial=pair(solveKirchhoffAxialCoupledSystem,offset);
 for(let step=0;step<3;step++) {
  reference.world.stepFixed();axial.world.stepFixed();
  for(let side=0;side<2;side++)for(const key of ['x','y','z','velocityX','velocityY','velocityZ'])
   reference.bodies[side][key].forEach((v,i)=>assert.ok(Math.abs(v-axial.bodies[side][key][i])<1e-5,`${step}/${side}/${key}/${i}: ${v} vs ${axial.bodies[side][key][i]}`));
 }
 if(offset>.1){
  assert.ok(axial.calls.length>0);assert.ok(axial.calls.every(d=>d.axialReduction===true&&d.converged),JSON.stringify(axial.calls));
  assert.ok(axial.bodies[0].y[0]<offset);assert.ok(axial.bodies[1].y[0]>0);}
});

for(const velocity of [-4,4])test(`full step retains loaded axial and torsional friction, velocity=${velocity}`,()=>{
 const reference=pair(solveKirchhoffCoupledSystem,.2,.015,velocity,2);
 const axial=pair(solveKirchhoffAxialCoupledSystem,.2,.015,velocity,2);
 for(let step=0;step<3;step++) {
  reference.world.stepFixed();axial.world.stepFixed();
  for(let side=0;side<2;side++)for(const key of ['x','y','z','velocityX','velocityY','velocityZ','orientationX','orientationY','orientationZ','orientationW'])
   reference.bodies[side][key].forEach((v,i)=>{
    // Compare velocity through its displacement over the actual 120 Hz step;
    // position and velocity cannot share a numerical tolerance with different units.
    const error=Math.abs(v-axial.bodies[side][key][i])*(key.startsWith('velocity')?1/120:1);
    assert.ok(error<1e-5,`${step}/${side}/${key}/${i}: ${v} vs ${axial.bodies[side][key][i]}`);
   });
 }
 assert.ok(axial.calls.some(d=>d.groupCount>0),'actual friction rows must enter the solve');
 assert.ok(axial.calls.every(d=>d.converged),JSON.stringify(axial.calls));
});
