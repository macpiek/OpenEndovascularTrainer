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
const w=prepared(),before=w.rows.commit();w.p.get('wire').forEach(p=>p[1]+=.01);
const rejected=w.refresh();assert.equal(rejected.converged,false);assert.ok(rejected.penetration>.009);
assert.throws(()=>w.rows.commit(),/uncertified/);
const measured={converged:rejected.converged,minGap:rejected.minGap,Fn:rejected.samples[0].Fn,ncp:rejected.ncp,work:rejected.complementarity};
rejected.converged=true;let rejection;
assert.throws(()=>w.rows.commit(),error=>{rejection=error.message;return /uncertified/.test(rejection);});
assert.equal(w.queries(),2);assert.equal(before.normalForces[0],1);
// Public diagnostics have no acceptance authority in either direction.
const valid=prepared(),validHistory=valid.rows.commit();valid.certificate.converged=false;
assert.deepEqual(valid.rows.commit(),validHistory);assert.equal(valid.queries(),1);
// Beginning a failed refresh also revokes the earlier private proof.
assert.throws(()=>valid.rows.refresh({toolPositions:valid.p,commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(6),
 order:'full',consumeQuery(){throw new Error('independent-late-query-failure');}}),/independent-late-query-failure/);
valid.certificate.converged=true;assert.throws(()=>valid.rows.commit(),/uncertified/);
assert.equal(valid.refresh().converged,true);assert.deepEqual(valid.rows.commit(),validHistory);
console.log(JSON.stringify({measured,mutation:'returnedCertificate.converged=true',rejected:true,rejection,
 currentOriginalGap:original(w.p).gap,queriesAtRejectedCommit:w.queries(),originalHistoryUntouched:before.normalForces[0]===1,
 publicFalseCannotRevokeActualPrivateProof:true,failedRefreshRevokesPrivateProof:true,freshRetryRestoresHistory:true,
 resolution:'Private commitReady authority prevents diagnostic-field changes from manufacturing an accepted history.',
 scope:'Exactly frozen baseline plus final two-file fix; no source edits, live solver imports or World claim.'},null,2));
