import assert from 'node:assert/strict';
import {createCompositeChainLayout} from '/tmp/oet-composite-joint-tip-final-review-8996/stage/src/physics/kirchhoffCompositeChain.js';
import {createCompositeJointLumenRows} from '/tmp/oet-composite-joint-tip-final-review-8996/stage/src/physics/kirchhoffCompositeJointLumenRows.js';
import {evaluateKirchhoffLumenSegmentContact as detect} from '/tmp/oet-composite-joint-tip-final-review-8996/stage/src/physics/kirchhoffLumenContact.js';
const layout=createCompositeChainLayout([['wire','catheter'],['wire']]);
const modes=[0,1].map(node=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
const pair={id:'review-rim',feature:'distal-rim',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,
 innerMaterialSegmentId:'review-wire-0',outerMaterialSegmentId:'review-cat-tip',lumenRadius:.5,innerRadius:.16,openDistal:true,portalFilletRadius:.15};
const contacts={mode:'lumen-normal',friction:'none',chartId:'independent-review-rim',forcePerLength:1,pairs:[pair]};
const tolerances={force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9,linearConstraint:5e-11};
const physical=()=>new Map([['catheter',[[0,0,0],[2,0,0],[4,0,0]]],['wire',[[1.8,.45,0],[2.3,.55,0],[2.8,.65,0]]]]);
const original=p=>detect({...pair,quadrature:[.5],activationDistance:Number.MAX_VALUE,
 innerStart:p.get('wire')[0],innerEnd:p.get('wire')[1],outerStart:p.get('catheter')[0],outerEnd:p.get('catheter')[1]}).portal.contact;
const wireNorm=rows=>Math.hypot(...rows.nodalForces.get('wire').reduce((sum,f)=>sum.map((v,i)=>v+f[i]),[0,0,0]));
function prepared(){
 const rows=createCompositeJointLumenRows({layout,coordinates:[0,2,4],modes,contacts,tolerances}),p=physical();
 let queries=0;const consumeQuery=()=>queries++;
 const refresh=()=>rows.refresh({toolPositions:p,commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(6),order:'full',consumeQuery});
 rows.prepareGauge({toolPositions:p,consumeQuery});rows.normalForces[0]=1;
 const certificate=refresh();assert.equal(certificate.converged,true);assert.ok(Math.abs(original(p).gap)<1e-15);
 return{rows,p,certificate,refresh,queries:()=>queries};
}
const result={fixture:{pair,toolPositions:[...physical()]},cases:[]};
for(const trialForce of [-1e-30,NaN,2]){
 const w=prepared(),before=w.rows.commit(),forceBefore=wireNorm(w.rows);w.rows.normalForces[0]=trialForce;
 let rejection;
 assert.throws(()=>w.rows.commit(),error=>{rejection=error.message;return /changed after their last certified refresh/.test(rejection);});
 assert.equal(w.queries(),1);assert.equal(wireNorm(w.rows),forceBefore);assert.equal(before.normalForces[0],1);
 w.rows.normalForces[0]=1;assert.deepEqual(w.rows.commit(),before);assert.equal(w.queries(),1);
 let refreshedForce=null;
 if(trialForce===2){w.rows.normalForces[0]=2;assert.equal(w.refresh().converged,true);refreshedForce=w.rows.commit().normalForces[0];
  assert.equal(refreshedForce,2);assert.ok(Math.abs(wireNorm(w.rows)-2)<1e-14);assert.equal(w.queries(),2);}
 result.cases.push({trialForce:Number.isNaN(trialForce)?'NaN':trialForce,rejected:true,rejection,queriesAtRejectedCommit:1,
  restoredExactHistory:true,refreshedForce,originalHistoryUntouched:before.normalForces[0]===1});
}
{
 const w=prepared(),history=w.rows.commit(),saved=structuredClone(w.p);w.p.get('wire').forEach(p=>p[1]+=.01);
 const currentGap=original(w.p).gap;let rejection;
 assert.throws(()=>w.rows.commit(),error=>{rejection=error.message;return /changed after their last certified refresh/.test(rejection);});
 assert.ok(currentGap<-.009);assert.equal(w.queries(),1);
 for(const [id,pts] of saved)w.p.set(id,structuredClone(pts));assert.deepEqual(w.rows.commit(),history);assert.equal(w.queries(),1);
 w.p.get('wire').forEach(p=>p[1]+=.01);const fresh=w.refresh();assert.equal(fresh.converged,false);
 assert.throws(()=>w.rows.commit(),/uncertified/);
 for(const [id,pts] of saved)w.p.set(id,structuredClone(pts));
 assert.throws(()=>w.rows.commit(),/uncertified/);assert.equal(w.refresh().converged,true);assert.deepEqual(w.rows.commit(),history);
 result.cases.push({mutation:'wire y += .01 after certified refresh',currentGap,rejected:true,rejection,
  queriesAtRejectedCommit:1,restoredExactHistory:true,freshPenetrationRejected:true,restoreAfterFailedRefreshNeedsNewQuery:true});
}
result.resolution='All original independent witness cases are blocked at commit without a query; exact rollback restores the old proof, while a failed/new refresh requires a new converged certificate.';
result.scope='Frozen 41-file baseline plus exactly the two hash-verified fix files; no live RelativeDirection, variable ownership, friction or World claim.';
console.log(JSON.stringify(result,null,2));
