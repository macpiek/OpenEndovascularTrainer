import assert from 'node:assert/strict';
import fs from 'node:fs';
import {StageCapture,profileCall,profileGenerator} from './stage-capture.mjs';
let now=0;const capture=new StageCapture(()=>now);
capture.run(()=>profileCall('outer',()=>{now+=2;profileCall('inner',()=>now+=3);now+=5;}));
now+=100;capture.run(()=>now++);
assert.equal(capture.stages.outer.selfMs,7);assert.equal(capture.stages.inner.selfMs,3);assert.equal(capture.stages.unattributed.inclusiveMs,11);
let cleaned=false;const g=profileGenerator('cancel',(function*(){try{yield 1;}finally{cleaned=true;}})());g.next();g.return();assert.ok(cleaned);
const root=new URL('../../',import.meta.url);const orig=await import(new URL('src/physics/kirchhoffSharedAxisAppSystem.js',root));
const inst=await import('file:///private/tmp/oet-solver-stages-20260915/src/physics/kirchhoffSharedAxisAppSystem.js');
const {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES}=await import(new URL('src/physics/endovascularPhysicsWorld.js',root));
function run(api){const dt=1/60,world=new EndovascularPhysicsWorld({fixedDt:dt});
 const tools=['wire','catheter'].map(id=>({id,body:world.createRod(id,13,5,DEFAULT_TOOL_PROFILES[id==='wire'?'guidewire':id]),insertion:0,rotation:0,type:id==='wire'?'glidewire':'straight',shaftStiffness:1,tipStiffness:1,nodeCoordinates:Array.from({length:13},(_,i)=>-40+5*i)}));
 world.contactField={voxelSize:1,queryCapsuleSoA(x,y,z,r,edge,out){Object.assign(out,{signedDistance:100,signedGap:100-r[edge],segmentT:.5,faceIndex:0});return out;}};
 const system=api.createSharedAxisAppSystem({workSliceMs:0,readTools:()=>tools.map(t=>({...t,nodeCoordinates:t.nodeCoordinates.slice()})),readSheath:()=>({start:[127,-83,29],end:[137,-83,29],innerRadius:2,proximalExtension:40})});world.wholeStepSystem=system;const outputs=[];
 for(const [wire,catheter] of [[3,1],[4,2],[3.5,2],[3,1.5]]){tools[0].insertion=wire;tools[1].insertion=catheter;let result;for(let i=0;i<10000;i++){result=system.step(world,dt);if(result.accepted)break;assert.equal(result.status,'shared-axis-pending');}assert.ok(result.accepted);outputs.push({status:result.status,iterations:result.diagnostics.last.iterations,factorizations:result.diagnostics.last.factorizations,points:tools.map(t=>[Array.from(t.body.x),Array.from(t.body.y),Array.from(t.body.z)])});}
 return outputs;
}
assert.deepEqual(run(inst),run(orig));
const result={exclusiveAccounting:'PASS',consumerWaitExcluded:'PASS',generatorCancellation:'PASS',fourForwardAndWithdrawalStepsExact:'PASS'};console.log(result);fs.writeFileSync(new URL('validation.json',import.meta.url),JSON.stringify(result,null,2));
