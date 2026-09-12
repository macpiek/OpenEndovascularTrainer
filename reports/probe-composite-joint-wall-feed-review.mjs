import assert from 'node:assert/strict';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';

// Read-only row transaction probe against the frozen runtime supplied by caller.
const root=process.argv[2];
assert.ok(root,'Supply the frozen review runtime path');
const url=p=>pathToFileURL(`${root}/${p}`);
const {createCompositeChainLayout}=await import(url('src/physics/kirchhoffCompositeChain.js'));
const {createCompositeJointWallRows:create}=await import(url('src/physics/kirchhoffCompositeJointWallRows.js'));
const {VesselContactField}=await import(url('src/physics/collision/vesselContactField.js'));
const {decodeCollisionAsset}=await import(url('src/physics/collision/collisionAssetFormat.js'));
const bytes=fs.readFileSync(url('res/Aorta_plain.collision.bin'));
const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)));
const initial=[[65.00287246704102,-462.00980948623305,-79.56869888305664],[64.95548751831055,-461.7916869276393,-79.65612350463867]];
const accepted=[[64.97836989399555,-462.0296690503204,-79.55471756171002],[65,-461.81336005090367,-79.6557542069334]];
const acceptedFn=[2.1924778704896255,.5980074238507306];
const layout=createCompositeChainLayout(Array.from({length:3},()=>['wire','catheter']));
const modes=Array.from({length:4},(_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
const config={layout,coordinates:[0,2,4,6],modes,relativeToolId:'wire',
  wall:{mode:'wall-normal',friction:'none',chartId:'review-partial-refresh-two-P1-supports',field,forcePerLength:10,
    contactOwners:{edges:[{edge:0,wall:{owner:'catheter',radius:.4445}},{edge:1,wall:null},{edge:2,wall:{owner:'catheter',radius:.4445}}]}},
  tolerances:{force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-7,linearConstraint:5e-11}};
const physical=cat=>new Map([['catheter',structuredClone(cat)],['wire',cat.map(p=>p.map((v,k)=>v+(k===2?.1:0)))]]);
const baseline=physical([...initial,...initial]);
const failedTrial=physical([...accepted,initial[1],initial[0]]);
const acceptedTrial=physical([...accepted,...accepted]);
let queries=0;
const refresh=(a,p,{common=new Float64Array(layout.dofCount),relative=new Float64Array(12),query=true}={})=>({
  certificate:a.refresh({toolPositions:p,commonResidual:common,relativeResidual:relative,order:'full',query,consumeQuery:()=>queries++}),common,relative});
const operator=a=>a.rows.map(r=>({forceIndex:r.forceIndex,owner:r.owner,edge:r.edge,sdfBranch:r.sdfBranch,
  residual:r.residual,multiplierDerivative:r.multiplierDerivative,geometricTangentValid:r.geometricTangentValid,
  commonDofs:Array.from(r.commonDofs),relativeDofs:Array.from(r.relativeDofs),jacobian:Array.from(r.jacobian),
  forceColumn:Array.from(r.forceColumn),geometricTangent:Array.from(r.geometricTangent)}));
const proof=c=>{const {structureVersion,queryGeneration,...rest}=c;return rest;};
const digest=v=>createHash('sha256').update(JSON.stringify(v)).digest('hex');
const a=create(config),cold=create(config);
for(const adapter of [a,cold])adapter.normalForces.set([2,0,0,3,0,0]);
const base=refresh(a,baseline),coldBase=refresh(cold,baseline);
assert.deepEqual(proof(base.certificate),proof(coldBase.certificate));
assert.deepEqual(a.rowForceIndices,[0,3]);
const checkpoint=a.checkpoint(),baseOperator=operator(a),baseForces=Array.from(a.normalForces);
const common=new Float64Array(layout.dofCount).fill(7),relative=new Float64Array(12).fill(9);
let rejection;
assert.throws(()=>refresh(a,failedTrial,{common,relative}),error=>{
  rejection=error.message;return /Loaded wall source\/sample\/feature changed/.test(error.message);
});
const failedForces=Array.from(a.normalForces),discoveriesAfterFailure=a.diagnostics.discoveries;
assert.ok(discoveriesAfterFailure>0,'Failure must occur after actual chart mutation');
assert.equal(failedForces[0],0,'First ordinary reaction already moved into the SDF cone');
assert.equal(failedForces[1]+failedForces[2],2);
assert.deepEqual(failedForces.slice(3),[3,0,0]);
assert.ok(common.every(v=>v===7)&&relative.every(v=>v===9),'Rejected refresh must not add partial residuals');
assert.ok(a.rows.every(r=>!r.geometricTangentValid&&r.jacobian.every(Number.isNaN)&&r.forceColumn.every(Number.isNaN)));
assert.ok([...a.nodalForces.values()].flat(2).every(v=>v===0));
assert.throws(()=>a.commit(),/fresh original query/);
a.restore(checkpoint);
assert.deepEqual(Array.from(a.normalForces),baseForces);
assert.deepEqual(a.rowForceIndices,[0,3]);
assert.throws(()=>refresh(a,baseline,{query:false}),/matching original/);
const retryBase=refresh(a,baseline);
assert.deepEqual(operator(a),baseOperator);
assert.deepEqual(proof(retryBase.certificate),proof(coldBase.certificate));
assert.deepEqual(retryBase.common,coldBase.common);assert.deepEqual(retryBase.relative,coldBase.relative);
// Both retries now receive the same valid chart geometry and certified physical
// reactions. This compares the entire accepted history, not only row indices.
for(const adapter of [a,cold]) {
  refresh(adapter,acceptedTrial);
  assert.deepEqual(adapter.rowForceIndices,[1,2,4,5]);
  adapter.normalForces.set([0,...acceptedFn,0,...acceptedFn]);
}
const retry=refresh(a,acceptedTrial),fresh=refresh(cold,acceptedTrial);
assert.equal(retry.certificate.converged,true);assert.equal(fresh.certificate.converged,true);
assert.deepEqual(proof(retry.certificate),proof(fresh.certificate));
assert.deepEqual(operator(a),operator(cold));assert.deepEqual(a.nodalForces,cold.nodalForces);
assert.deepEqual(retry.common,fresh.common);assert.deepEqual(retry.relative,fresh.relative);
const history=a.commit(),cleanHistory=cold.commit();assert.deepEqual(history,cleanHistory);
assert.ok(history.records.every(r=>r.seam!==null&&r.representative===null));
const owned=structuredClone(history);a.normalForces.fill(0);assert.deepEqual(history,owned);
console.log(JSON.stringify({status:'PASS',scope:'original wall rows partial refresh failure, restore and clean retry; not a whole-dt benchmark',
  anatomySha256:createHash('sha256').update(bytes).digest('hex'),originalQueries:queries,rejection,
  discoveriesAfterFailure,baseForces,failedForces,restoredForces:baseForces,acceptedForces:Array.from(history.normalForces),
  acceptedRecords:history.records,certificate:proof(retry.certificate),baselineOperatorSha256:digest(baseOperator),
  acceptedOperatorSha256:digest(operator(cold)),
  checks:{partialMutationObserved:true,noPartialResidual:true,failedOperatorsInvalidated:true,failedCommitRejected:true,
    restoreRequiresFreshQuery:true,baselineOperatorsBitExact:true,retryOperatorsAndForcesBitExact:true,retryHistoryMetadataBitExact:true,historyOwned:true},
  diagnostics:{retry:a.diagnostics,cold:cold.diagnostics}},null,2));
