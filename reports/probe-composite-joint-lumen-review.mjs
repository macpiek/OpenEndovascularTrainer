import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const root=process.argv[2],imp=name=>import(pathToFileURL(`${root}/src/physics/${name}.js`));
const [step,chain,el,lumen]=await Promise.all(['kirchhoffCompositeJointTimeStep','kirchhoffCompositeChain','kirchhoffCompositeElement','kirchhoffCompositeJointLumenRows'].map(imp));
const hash=v=>createHash('sha256').update(JSON.stringify(v,(_,v)=>v instanceof Map?[...v]:ArrayBuffer.isView(v)?Array.from(v):v)).digest('hex');
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),sub=(a,b)=>a.map((v,i)=>v-b[i]),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const close=(a,b,t=1e-10)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]),wire=coordinates.map(x=>[2.25+1.125*x,.25,0]);
const layout=chain.createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),angles=new Map(),restLengths=new Map();
const tools=['wire','catheter'].map(id=>{
 const dsDx=id==='wire'?1.125:1;angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
 return {id,dsDx,reference:el.captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(1),material:el.compileCompositeMaterial({EI1:id==='wire'?2:8,EI2:id==='wire'?3:11,GJ:id==='wire'?1:4})};
});
const state=step.createCompositeJointTimeStepState({layout,coordinates,positions,relative:wire.flatMap((p,i)=>sub(p,positions[i])),modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
const contacts={mode:'lumen-normal',friction:'none',chartId:'independent-shifted-pair',forcePerLength:1,pairs:[{id:'wire0-cat1',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:1,
 innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:1',lumenRadius:.5,innerRadius:.25,quadrature:[.5,.75,.25,.625],openDistal:false,portalFilletRadius:0}]};
const tolerances={force:1e-7,torque:1e-8,length:1e-8,boundary:1e-9,linearForce:5e-10,linearTorque:5e-10,linearConstraint:5e-11,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9};
function rows(history=null){return lumen.createCompositeJointLumenRows({...state,contacts,history,tolerances});}
const first=rows(),initialFn=[2,3,1,4];
state.lumenContactState={signature:first.signature,sampleIds:first.samples.map(s=>s.sampleId),normalForces:Float64Array.from(initialFn),friction:'none'};
const before=hash(state);
// Independent Euclidean point-to-line algebra, with no detector or production
// derivative/pullback helper. Pairs are wire[0,1] and catheter[1,2].
function geometry(p,s) {
 const A=p.get('wire')[0],W=p.get('wire')[1],C=p.get('catheter')[1],D=p.get('catheter')[2];
 const P=A.map((v,k)=>v+s*(W[k]-v)),d=sub(D,C),t=dot(sub(P,C),d)/dot(d,d);assert.ok(t>0&&t<1);
 const r=P.map((v,k)=>v-C[k]-t*d[k]),distance=Math.hypot(...r),n=r.map(v=>v/distance);
 return {gap:.25-distance,t,B:[-(1-s),-s,1-t,t].flatMap(w=>n.map(v=>w*v))};
}
function physical(p,Fn) {return Array.from({length:12},(_,j)=>contacts.pairs[0].quadrature.reduce((sum,s,i)=>sum+Fn[i]*geometry(p,s).B[j],0));}
const prepared=rows(state.lumenContactState);let queryCount=0;prepared.prepareGauge({toolPositions:state.toolPositions,consumeQuery(){queryCount++;}});
assert.deepEqual(Array.from(prepared.normalForces),[0,7,3,0]);assert.equal(queryCount,4);
const F0=physical(state.toolPositions,initialFn),F1=physical(state.toolPositions,prepared.normalForces);assert.deepEqual(F0,F1);
const endpoints=[wire[0],wire[1],positions[1],positions[2]],origin=[.2,-.3,.7],virtual=Array.from({length:12},(_,i)=>Math.sin(i+.4));
function wrench(F){return [0,1].map(tool=>{const force=[0,0,0],moment=[0,0,0];for(let node=2*tool;node<2*tool+2;node++){const f=F.slice(3*node,3*node+3),m=cross(sub(endpoints[node],origin),f);for(let k=0;k<3;k++){force[k]+=f[k];moment[k]+=m[k];}}return {force,moment};});}
assert.deepEqual(wrench(F0),wrench(F1));assert.equal(dot(F0,virtual),dot(F1,virtual));
// New prescribed catheter endpoint changes normals, so doing the same old
// history transfer on the prepared candidate would be invalid.
const moved=structuredClone(state.toolPositions);moved.get('catheter')[2][2]=.001;moved.get('wire')[2][2]=.001;
assert.throws(()=>rows(state.lumenContactState).prepareGauge({toolPositions:moved,consumeQuery(){}}),/lacks an exact affine force-column transfer/);
const inertia={previousPositions:structuredClone(state.toolPositions),inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?.13:.24,
 materialMap:{sStart:20+coordinates[e]*tools.find(t=>t.id===id).dsDx,dsDx:tools.find(t=>t.id===id).dsDx,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
const loads={forces:[...F0.reduce((a,v,i)=>{const node=Math.floor(i/3);a[node][i%3]=-v;return a;},Array.from({length:4},()=>[0,0,0])).map((value,i)=>({toolId:i<2?'wire':'catheter',node:i<2?i:i-1,value}))],torques:[]};
loads.forces[0].value[1]+=.01;
const input={dt:.1,torsionMode:'quasi-static',contacts,tolerances,inertia,loads,boundaries:{positions:[{toolId:'wire',node:2,value:wire[2]},{toolId:'catheter',node:2,value:[4,0,.001]}],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]}};
const cold=step.advanceCompositeJointTimeStep(state,input),workspace=step.createCompositeJointTimeStepWorkspace(state),warm=step.advanceCompositeJointTimeStep(state,{...input,workspace});
assert.equal(cold.accepted,true,JSON.stringify({status:cold.status,error:cold.error,diagnostics:cold.diagnostics}));assert.equal(warm.accepted,true);
for(const key of ['state','contactForces','boundaryForces','balances','perTool','spinReactions'])assert.deepEqual(warm[key],cold[key],key);
assert.deepEqual(warm.diagnostics.certificate,cold.diagnostics.certificate);assert.equal(hash(state),before);
assert.equal(cold.diagnostics.lumenGauge.preparationQueries,4);assert.equal(cold.diagnostics.lumenGauge.transfers.length,2);
const Fn=cold.state.lumenContactState.normalForces,proof=cold.diagnostics.certificate.contact;
contacts.pairs[0].quadrature.forEach((s,i)=>{const g=geometry(cold.state.toolPositions,s);close(proof.samples[i].gap,g.gap,1e-14);close(proof.samples[i].outerT,g.t,1e-14);assert.ok(g.gap>=-1e-8&&Fn[i]>=0&&Math.abs(g.gap*Fn[i])<=1e-9);});
assert.equal(Fn[0],0);assert.equal(Fn[3],0);
const expected=physical(cold.state.toolPositions,Fn),actual=[...cold.contactForces.get('wire').slice(0,2),...cold.contactForces.get('catheter').slice(1,3)].flat();expected.forEach((v,i)=>close(v,actual[i],1e-13));
for(const b of cold.balances.values())assert.ok(Math.hypot(...b.residual)<2e-7);
// Directional DB and work from fresh current contact forces. The production
// row must reflect the changed geometry after the old-gauge transfer.
const cg=new Float64Array(layout.dofCount),rg=new Float64Array(state.relative.length);
prepared.refresh({toolPositions:state.toolPositions,commonResidual:cg,relativeResidual:rg,order:'full',consumeQuery(){}});
const oldH=prepared.samples.map(s=>s.geometry.normalDerivative.slice());
prepared.normalForces.set(Fn);prepared.refresh({toolPositions:cold.state.toolPositions,commonResidual:cg.fill(0),relativeResidual:rg.fill(0),order:'full',consumeQuery(){}});
const freshH=prepared.samples.map(s=>s.geometry.normalDerivative.slice()),h=1e-6;
function displace(eps){const p=structuredClone(cold.state.toolPositions);for(let i=0;i<4;i++){const id=i<2?'wire':'catheter',node=i<2?i:i-1;p.get(id)[node]=p.get(id)[node].map((v,k)=>v+eps*virtual[3*i+k]);}return p;}
const plus=physical(displace(h),Fn),minus=physical(displace(-h),Fn);
let maxDBError=0,maxOldFreshDifference=0;for(let i=0;i<12;i++){
 const analytic=Fn.reduce((sum,f,s)=>sum+f*freshH[s].slice(12*i,12*i+12).reduce((v,H,j)=>v+H*virtual[j],0),0);
 maxDBError=Math.max(maxDBError,Math.abs(analytic-(plus[i]-minus[i])/(2*h)));
 for(let j=0;j<12;j++)maxOldFreshDifference=Math.max(maxOldFreshDifference,Math.abs(freshH[0][12*i+j]-oldH[0][12*i+j]));
}assert.ok(maxDBError<1e-7);assert.ok(maxOldFreshDifference>1e-5);
const work=eps=>contacts.pairs[0].quadrature.reduce((sum,s,i)=>sum+Fn[i]*geometry(displace(eps),s).gap,0);const workError=Math.abs(dot(expected,virtual)-(work(h)-work(-h))/(2*h));assert.ok(workError<1e-8);
for(const budget of [{contactQueries:2},{evaluations:cold.diagnostics.evaluations-1}]) {
 const rejected=step.advanceCompositeJointTimeStep(state,{...input,workspace,budget});assert.equal(rejected.accepted,false);assert.equal(rejected.state,state);assert.equal(hash(state),before);
 const retry=step.advanceCompositeJointTimeStep(state,{...input,workspace});assert.equal(retry.accepted,true);assert.deepEqual(retry.state,cold.state);assert.deepEqual(retry.contactForces,cold.contactForces);
}
console.log(JSON.stringify({status:'PASS',scope:'one shifted innerEdge0/outerEdge1 contact; four unsorted samples, two loaded interior gauges; out-of-plane prescribed motion',stateInputSha256:before,acceptedStateSha256:hash(cold.state),
 incomingFn:initialFn,gaugeFn:Array.from(prepared.gauge.transfers),acceptedFn:Array.from(Fn),incomingTransferQueries:queryCount,originalQueries:cold.diagnostics.contactQueries,
 directions:cold.diagnostics.directions,evaluations:cold.diagnostics.evaluations,contactProof:proof,maxDBError,maxOldFreshDifference,workError,coldReuseExact:true,rollbackRetryExact:true},null,2));
