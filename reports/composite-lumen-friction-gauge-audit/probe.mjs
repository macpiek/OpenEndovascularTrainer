import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
const base=process.env.OET_GAUGE_SOURCE??'/tmp/oet-lumen-friction-gauge-audit-8996/stage';
const module=async name=>import(pathToFileURL(`${base}/src/physics/${name}.js`));
const {evaluateKirchhoffLumenSegmentContact:detect}=await module('kirchhoffLumenContact');
const {differentiateCompositeLumenSideContact:side,createCompositeLumenSideGeometryWorkspace:sideWorkspace}=await module('kirchhoffCompositeLumenSideGeometry');
const {createCompositeJointLumenRows}=await module('kirchhoffCompositeJointLumenRows');
const {evaluateCompositeJointLumenSurface:surface}=await module('kirchhoffCompositeJointLumenSurface');
const {createCompositeJointSurfacePullback,pullbackCompositeJointSurface,evaluateCompositeJointSurfaceLoads}=await module('kirchhoffCompositeJointSurfacePullback');
const {captureCompositeReferenceFrames}=await module('kirchhoffCompositeElement');
const {createCompositeChainLayout}=await module('kirchhoffCompositeChain');
const {measureCompositeFriction,evaluateCompositeFrictionEquation,createCompositeFrictionEquationWorkspace}=await module('kirchhoffCompositeFriction');
const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0),norm=a=>Math.hypot(...a),sub=(a,b)=>a.map((v,i)=>v-b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const close=(a,b,t=1e-11)=>assert.ok(Math.abs(a-b)<=t,`${a} != ${b}`),same=(a,b,t)=>a.forEach((v,i)=>close(v,b[i],t));
const ss=[.25,.5,.75],oldFn=[0,1,0],newFn=[.5,0,.5],mu=[.015,.006],dt=.01;
const current={innerStart:[.25,.25,0],innerEnd:[2.25,.25,0],outerStart:[0,0,0],outerEnd:[2,0,0],
 innerRadius:.1,lumenRadius:.35,innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'cat:0',openDistal:false,portalFilletRadius:0};
const names=['innerStart','innerEnd','outerStart','outerEnd'];
const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
const modes=[0,1,2].map(node=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
function raw(state,s){const input={...structuredClone(state),quadrature:[s]};return{input,contact:detect(input).side};}
const normalGeometry=ss.map(s=>{const r=raw(current,s),g=side(r,sideWorkspace());assert.ok(g.supported,g.reason);return g;});
const normalLoad=forces=>Array.from({length:12},(_,i)=>normalGeometry.reduce((sum,g,s)=>sum+forces[s]*g.normalForceColumn[i],0));
function normalWrench(load){return [0,1].map(body=>{const F=[0,0,0],M=[0,0,0];for(let end=0;end<2;end++){
 const at=6*body+3*end,f=load.slice(at,at+3);f.forEach((v,k)=>F[k]+=v);cross(current[names[2*body+end]],f).forEach((v,k)=>M[k]+=v);}
 return {force:F,moment:M};});}
const normalBefore=normalLoad(oldFn),normalAfter=normalLoad(newFn);same(normalBefore,normalAfter,0);
const physical=new Map([['wire',[current.innerStart,current.innerEnd,[4.25,.25,0]]],['catheter',[current.outerStart,current.outerEnd,[4,0,0]]]]);
const contacts={mode:'lumen-normal',friction:'none',chartId:'gauge-audit',forcePerLength:1,pairs:[{id:'side',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,
 innerMaterialSegmentId:current.innerMaterialSegmentId,outerMaterialSegmentId:current.outerMaterialSegmentId,quadrature:ss,lumenRadius:current.lumenRadius,innerRadius:current.innerRadius,openDistal:false,portalFilletRadius:0}]};
const rows=createCompositeJointLumenRows({layout,coordinates:[0,2,4],modes,contacts,tolerances:{force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9,linearConstraint:5e-11}});
rows.normalForces.set(oldFn);const gauge=rows.prepareGauge({toolPositions:physical,consumeQuery(){}});same(rows.normalForces,newFn,0);
assert.equal(rows.refresh({toolPositions:physical,commonResidual:new Float64Array(layout.dofCount),relativeResidual:new Float64Array(9),order:'full',consumeQuery(){}}).converged,true);
function buildSurface(previous,s){
 const tools=['wire','catheter'].map((id,i)=>{const previousPositions=names.slice(2*i,2*i+2).map(n=>previous[n]),positions=names.slice(2*i,2*i+2).map(n=>current[n]);
  return{id,edge:0,edgeId:`${id}:edge0`,materialSegmentId:i?current.outerMaterialSegmentId:current.innerMaterialSegmentId,coordinates:[0,2],positions,previousPositions,
   reference:captureCompositeReferenceFrames(previousPositions)[0],angle:0,previousAngle:0,materialMap:{sStart:0,dsDx:1,dsDt:0},
   materialPath:{kind:'linear-affine-maps',previousEdgeId:`${id}:edge0`,previousMap:{sStart:0,dsDx:1}}};});
 const r=surface({dt,tools,current:raw(current,s),previous:raw(previous,s)});assert.ok(r.operatorReady&&r.DforceMapValid);assert.equal(r.queryCount,0);
 const pulled=createCompositeJointSurfacePullback({layout,modes,tools:r.tools});pullbackCompositeJointSurface(r,pulled);
 return{r,pulled};
}
function FtFor(Fn,slip,coeff){const d=Math.hypot(coeff[0]*slip[0],coeff[1]*slip[1]);return d?coeff.map((v,i)=>-Fn*v*v*slip[i]/d):[0,0];}
const tol={slipTolerance:1e-10,coneTolerance:1e-10,workTolerance:1e-10};
function evaluateDistribution(Fn,slips,surfaces,coeffs=ss.map(()=>mu)){
 const loads=new Float64Array(14),common=new Float64Array(surfaces[0].pulled.commonDofs.length),relative=new Float64Array(surfaces[0].pulled.relativeDofs.length);
 const tractions=[],kkt=[],equations=[];let work=0;
 for(let i=0;i<3;i++){
  const Ft=FtFor(Fn[i],slips[i],coeffs[i]);tractions.push(Ft);
  const args={normalForce:Fn[i],traction:Ft,slip:slips[i],mu:coeffs[i]};const proof=measureCompositeFriction({...args,...tol});assert.ok(proof.converged);kkt.push(proof);work+=proof.work;
  const eq=evaluateCompositeFrictionEquation({...args,penalty:7},createCompositeFrictionEquationWorkspace());assert.ok(norm(eq.residual)<1e-11);equations.push([...eq.residual]);
  const f=evaluateCompositeJointSurfaceLoads(Ft,surfaces[i].pulled);assert.ok(f.valid);
  f.physical.forEach((v,j)=>loads[j]+=v);f.common.forEach((v,j)=>common[j]+=v);f.relative.forEach((v,j)=>relative[j]+=v);
 }
 const wrench=[0,1].map(body=>{const F=[0,0,0],M=[loads[7*body+6],0,0];for(let end=0;end<2;end++){
  const at=7*body+3*end,f=Array.from(loads.slice(at,at+3));f.forEach((v,k)=>F[k]+=v);cross(current[names[2*body+end]],f).forEach((v,k)=>M[k]+=v);}
  return {force:F,moment:M,spinTorque:loads[7*body+6]};});
 same(wrench[0].force.map((v,k)=>v+wrench[1].force[k]),[0,0,0],1e-12);
 same(wrench[0].moment.map((v,k)=>v+wrench[1].moment[k]),[0,0,0],1e-12);
 return{Fn,slips,tractions,loads:[...loads],common:[...common],relative:[...relative],wrench,work,kkt,equations};
}
const fixed=ss.map(s=>buildSurface(current,s));
const velocity=[0,0,-1,0,0,1,0,0,0,0,0,0,0,0]; // wire rotates at omega_y=-1 about x=1.25
const rateSlips=fixed.map(({r})=>[0,1].map(c=>velocity.reduce((sum,v,i)=>sum+v*r.forceMap[2*i+c],0)));
same(rateSlips.flat(),[0,-.5,0,0,0,.5],1e-14);
const before=evaluateDistribution(oldFn,rateSlips,fixed),after=evaluateDistribution(newFn,rateSlips,fixed);
close(before.work,0);close(after.work,-.003);close(after.wrench[0].moment[1],.003);close(dot(after.loads,velocity),after.work);
const keepingFt=ss.map((_,i)=>measureCompositeFriction({normalForce:newFn[i],traction:[0,0],slip:rateSlips[i],mu,...tol}));
assert.equal(keepingFt[0].converged,false);assert.equal(keepingFt[2].converged,false);
const angle=.02,previous={...structuredClone(current),innerStart:[1.25-Math.cos(angle),.24,Math.sin(angle)],innerEnd:[1.25+Math.cos(angle),.24,-Math.sin(angle)]};
const finite=ss.map(s=>buildSurface(previous,s)),finiteSlips=finite.map(({r})=>[...r.increment]);
const oldGaps=ss.map(s=>raw(previous,s).contact.gap);assert.ok(oldGaps.every(g=>g>0));same(finiteSlips[1],[0,0],1e-13);
const finiteBefore=evaluateDistribution(oldFn,finiteSlips,finite),finiteAfter=evaluateDistribution(newFn,finiteSlips,finite);
assert.ok(finiteAfter.work<-1e-6);assert.ok(Math.abs(finiteAfter.wrench[0].moment[1])>.002);
const variedMu=[[.01,.006],[.04,.006],[.01,.006]],uniformSlip=ss.map(()=>[1,0]);
const muBefore=evaluateDistribution(oldFn,uniformSlip,fixed,variedMu),muAfter=evaluateDistribution(newFn,uniformSlip,fixed,variedMu);
close(muBefore.work,-.04);close(muAfter.work,-.01);
console.log(JSON.stringify({scope:'Counterexample to exact Coulomb equivalence of normal-only affine pressure gauge; no whole-step solve',
 source:base,geometry:current,samples:ss,normal:{gaps:normalGeometry.map(g=>g.gap),before:normalBefore,after:normalAfter,exactEquality:normalBefore.every((v,i)=>v===normalAfter[i]),wrenchBefore:normalWrench(normalBefore),wrenchAfter:normalWrench(normalAfter),actualGauge:gauge},
 instantaneous:{mu,velocity,before,after,keepingZeroFtAfterNormalTransfer:keepingFt},
 finite:{previous,oldGaps,currentLength:norm(sub(current.innerEnd,current.innerStart)),previousLength:norm(sub(previous.innerEnd,previous.innerStart)),before:finiteBefore,after:finiteAfter,
  currentSurfacePoints:finite.map(({r})=>[...r.currentWitness.point]),currentWitnessSeparation:finite.map(({r})=>r.currentWitness.separationNorm),allSurfaceOperatorsReady:true},
 coefficientVariation:{note:'Separate per-sample material coefficient thought experiment; the uniform-mu counterexample above already suffices',mu:variedMu,before:muBefore,after:muAfter}},null,2));
