import test from 'node:test';import assert from 'node:assert/strict';
import {beginKirchhoffWallWitnessFrictionModes as begin,selectKirchhoffWallWitnessFrictionCoefficient as select,evaluateKirchhoffWallWitnessFrictionCandidate as evaluate,prepareKirchhoffWallWitnessFrictionRetry as retry,commitKirchhoffWallWitnessFrictionModes as commit} from '../src/physics/kirchhoffWallWitnessFrictionMode.js';
const dt=1/120,tolerance=1e-8;
function fixture(kinetic=.2){
 const body={wallStaticFriction:.6,wallKineticFriction:kinetic};const c={bodies:[body]};
 const w={body,side:0,node:0,materialA:0,materialB:1,t:.5,face:42,radius:.5,triangleKey:'triangle42',ledger:{lambda:0,tangentLambda:new Float64Array(2),wrenches:[]}};
 c._wallWitnessRows={witnesses:[w]};begin(c,{dt,step:1,displacementToleranceMm:tolerance});return {c,w,body};
}
function candidate(c,w,{normal=1,force=-.4,slip=0}={}){
 const selected=select(c,w,dt);return {component:c,dt,committed:false,entries:[{witness:w,modeKey:selected.key,mode:selected.mode,contact:{normalLambda:normal},surface:{group:{mu:[selected.mu,selected.mu]},rows:[{lambda:force,strain:slip},{lambda:0,strain:0}]}}]};
}
test('position-history begins static every step and holds between kinetic and static budgets',()=>{
 const {c,w}=fixture();assert.equal(select(c,w,dt).mu,.6);
 const result=evaluate(c,candidate(c,w),{converged:true});assert.equal(result.status,'accepted');assert.equal(result.contacts[0].stopped,true);commit(c,result);
 begin(c,{dt,step:2,displacementToleranceMm:tolerance});assert.equal(select(c,w,dt).mode,'stick');
});
test('only converged static slip proposes a kinetic retry and never modifies force history',()=>{
 const {c,w}=fixture(),batch=candidate(c,w,{force:-.6,slip:.2});
 w.ledger.lambda=1;w.ledger.tangentLambda[0]=-.6;w.ledger.wrenches.push({node:0,fx:-.6,fy:0,fz:1,mx:0,my:0,mz:0});
 const before=JSON.stringify(w.ledger);assert.equal(evaluate(c,batch,{converged:false}).status,'unconverged');
 const decision=evaluate(c,batch,{converged:true});assert.equal(decision.status,'restart');assert.equal(JSON.stringify(w.ledger),before);
 assert.throws(()=>retry(c,decision),/Restore whole step/);
 w.ledger.lambda=0;w.ledger.tangentLambda.fill(0);w.ledger.wrenches.length=0;
 retry(c,decision);assert.equal(select(c,w,dt).mu,.2);
 const accepted=evaluate(c,candidate(c,w,{force:-.2,slip:.6}),{converged:true});assert.equal(accepted.status,'accepted');commit(c,accepted);
 assert.equal(w.ledger.lambda,0);assert.equal(w.ledger.wrenches.length,0);
});
test('zero kinetic coefficient retains static holding then permits exactly zero-force sliding',()=>{
 const {c,w}=fixture(0);assert.equal(evaluate(c,candidate(c,w),{converged:true}).accepted,true);
 const d=evaluate(c,candidate(c,w,{force:-.6,slip:.2}),{converged:true});retry(c,d);
 assert.equal(select(c,w,dt).mu,0);assert.equal(evaluate(c,candidate(c,w,{force:0,slip:.8}),{converged:true}).accepted,true);
});
test('small cone-boundary slip remains ambiguous and invalid KKT never proves breakaway',()=>{
 const {c,w}=fixture();assert.equal(evaluate(c,candidate(c,w,{force:-.6,slip:tolerance/2}),{converged:true}).status,'ambiguous');
 assert.equal(evaluate(c,candidate(c,w,{force:-.9,slip:.2}),{converged:true}).status,'unconverged');
});
test('retries own stable material/finite-face identities and cannot demote another witness',()=>{
 const {c,w}=fixture(),d=evaluate(c,candidate(c,w,{force:-.6,slip:.2}),{converged:true});retry(c,d);
 assert.equal(select(c,w,dt).mode,'slide');assert.equal(select(c,{...w,face:43},dt).mode,'stick');assert.equal(select(c,{...w,materialA:2},dt).mode,'stick');assert.equal(select(c,{...w,t:.6},dt).mode,'stick');
 const stale=candidate(c,w,{force:-.2,slip:.6});stale.entries[0].mode='stick';assert.throws(()=>evaluate(c,stale,{converged:true}),/Stale/);
});
test('profiles, timestep, attempts and fresh measurement are explicit guards',()=>{
 const {c,w,body}=fixture();const batch=candidate(c,w);batch.committed=true;assert.throws(()=>evaluate(c,batch,{converged:true}),/Fresh/);
 body.wallKineticFriction=.8;assert.throws(()=>select(c,w,dt),/static >= kinetic/);body.wallKineticFriction=.2;
 assert.throws(()=>select(c,w,dt*2),/matching/);
 begin(c,{dt,step:1,displacementToleranceMm:tolerance,maximumAttempts:1});assert.equal(evaluate(c,candidate(c,w,{force:-.6,slip:.2}),{converged:true}).status,'exhausted');
});
