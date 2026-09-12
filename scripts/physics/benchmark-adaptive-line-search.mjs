import { sampleCatheterBrowserBenchmarkCommands, createBrowserBenchmarkCommands } from '../../src/benchmark/browserBenchmarkScenario.js';
import fs from 'node:fs';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { ConstraintStageProfile } from '../../src/physics/constraintStageProfile.js';
const steps = Number(process.argv[2] ?? 1000);
const anatomy = await loadCoupledRuntimeAnatomy();
const results=[];
try {
for (const adaptive of (process.argv[4] ?? '0,1').split(',').map(value => value === '1')) {
 const profile=new ConstraintStageProfile();
 class World extends EndovascularPhysicsWorld {
  stepFixed() {
   this.adaptiveLineSearch=adaptive;
   for(const body of this.bodies) configureKirchhoffToolRuntime(body);
   const result=super.stepFixed();profile.record(this);return result;
  }
 }
 const fixture=createCoupledRuntimeFixture({...anatomy,World,coupledSystem:{independentComponents:true,
  solve:(c,dt,o)=>solveKirchhoffCoupledSystem(c,dt,{...o,activeCondensation:true,simultaneousCoulomb:true}),apply:applyKirchhoffCoupledCorrection}});
 const commands=createBrowserBenchmarkCommands();
 let maxLengthError=0,maxPenetration=0,finite=true;
 for(let i=0;i<steps;i++) {
  fixture.step(sampleCatheterBrowserBenchmarkCommands(i * 1000 / 120, commands));
  maxPenetration=Math.max(maxPenetration,fixture.world.settledMaxPenetration);
  const b=fixture.catheterBody;
  for(let j=b.activeStart;j<b.activeEnd;j++) {
   const length=Math.hypot(b.x[j+1]-b.x[j],b.y[j+1]-b.y[j],b.z[j+1]-b.z[j]);
   finite &&= Number.isFinite(length);maxLengthError=Math.max(maxLengthError,Math.abs(length-b.restLength[j]));
  }
 }
 results.push({adaptive,steps,maxLengthError,maxPenetration,finite,profile:profile.report(),final:fixture.world.getStats()});
 console.log(JSON.stringify({adaptive,steps,maxLengthError,maxPenetration,finite,profile:profile.report()}));
 fixture.dispose();
}
}finally{anatomy.dispose();}
if(process.argv[3])fs.writeFileSync(process.argv[3],JSON.stringify(results,null,2)+'\n');
