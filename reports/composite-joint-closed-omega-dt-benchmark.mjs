import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeElement.js';
import {evaluateKirchhoffLumenSegmentContact} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffLumenContact.js';
import {measureCompositeFriction} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeFriction.js';
import {createCompositeJointMaterialHistory} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from 'file:///Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointTimeStep.js';

const dt=1/120;
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));

function fixture() {
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]),slope=-.002;
    const wire=coordinates.map(x=>[x+.1,.341+slope*x,0]);
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const p=id==='wire'?wire:positions,dsDx=id==='wire'?Math.hypot(1,slope):1;
        angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(p),referenceTwists:new Float64Array(1),
            material:compileCompositeMaterial({stiffness:(id==='wire'?[2,3,1]:[8,11,4]).map((v,i)=>Array.from({length:3},(_,j)=>i===j?v:0)),intrinsic:[0,0,0]})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,
        relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const contacts={mode:'lumen-coulomb',chartId:'full-friction-dt',forcePerLength:1,
        friction:{law:'coulomb',mu:[.015,.006],forcePerLength:50,materialPath:'linear-affine-maps'},
        pairs:[{id:'own-side-0',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,
            innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:0',lumenRadius:.5,innerRadius:.16,
            quadrature:[.25],openDistal:false,portalFilletRadius:0}]};
    return {state,contacts};
}
function options(f,state,{feed={wire:-.3,catheter:.1},force=.4,k=50,workspace,budget}={}) {
    const mappedEdges=state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>{
        const dsDx=state.tools.find(t=>t.id===id).dsDx,dsDt=feed[id];
        return {id,massPerMaterialLength:id==='wire'?.13:.24,
            materialMap:{sStart:20+state.coordinates[edge]*dsDx+(state.time+dt)*dsDt,dsDx,dsDt}};
    })}));
    // The fixture explicitly starts its external material reservoir at rest.
    // Previously accepted internal velocity fields are sampled at CURRENT
    // labels, preserving every old edge boundary during the second dt.
    const history=createCompositeJointMaterialHistory({materialVelocities:state.materialVelocities,
        reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})});
    const sampled=history.prepare({coordinates:state.coordinates,inertiaEdges:mappedEdges});
    const inertiaEdges=mappedEdges.map((entry,edge)=>({tools:entry.tools.map((t,index)=>{
        const h=sampled.inertiaEdges[edge].tools[index];
        return {...t,...(h.pieces.length===1?{oldMaterialVelocities:h.oldMaterialVelocities}:
            {oldVelocityPieces:h.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})};
    })}));
    return {dt,torsionMode:'quasi-static',contacts:{...f.contacts,friction:{...f.contacts.friction,forcePerLength:k}},
        workspace,budget,
        boundaries:{positions:['wire','catheter'].map(toolId=>({toolId,node:2,value:f.state.toolPositions.get(toolId)[2]})),
            spins:[{toolId:'wire',edge:0,value:.03*(state.time+dt)},{toolId:'catheter',edge:0,value:-.01*(state.time+dt)}]},
        loads:{forces:[{toolId:'wire',node:0,value:[0,force,0]}],torques:[{toolId:'wire',edge:1,value:.001}]},
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges}};
}
function advance(f,state,extra) {return advanceCompositeJointTimeStep(state,options(f,state,extra));}
function verifyPhysical(f,r,externalContactResultant=[0,0,0]) {
    const pair=f.contacts.pairs[0],p=r.state.toolPositions;
    const raw=evaluateKirchhoffLumenSegmentContact({...pair,innerStart:p.get('wire')[0],innerEnd:p.get('wire')[1],
        outerStart:p.get('catheter')[0],outerEnd:p.get('catheter')[1]});
    const Fn=r.state.lumenContactState.normalForces[0],Ft=r.state.lumenFrictionState.tractions;
    assert.ok(Fn>=0);assert.ok(raw.side.gap>=-1e-8);assert.ok(Math.abs(Fn*raw.side.gap)<=1e-9);
    const proof=r.diagnostics.certificate.friction;assert.equal(proof.converged,true);
    assert.equal(proof.samples.length,1);const s=proof.samples[0];
    same(s.traction,Array.from(Ft),1e-14);close(s.Fn,Fn,1e-14);
    const original=measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:Fn,mu:pair.mu??f.contacts.friction.mu,
        slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9});
    assert.equal(original.converged,true,JSON.stringify(original));
    for(const [id,points] of p)for(let edge=0;edge<2;edge++)close(Math.hypot(...points[edge+1].map((v,k)=>v-points[edge][k])),r.state.restLengths.get(id)[edge],1e-8);
    for(const b of r.balances.values())same(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),1e-7);
    same(r.balances.get('wire').contactForce.map((v,k)=>v+r.balances.get('catheter').contactForce[k]),externalContactResultant,1e-12);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations+1+(r.diagnostics.wallQueries??0));
    assert.equal(r.diagnostics.frictionPreparationQueries,1);
    return s;
}

const baseline=await import('file:///tmp/oet-joint-closed-omega-dt-baseline/kirchhoffCompositeJointTimeStep.js');
const f=fixture(),first=advance(f,f.state);accepted(first);
const states=[f.state,first.state],summary=[];
const quantiles=a=>{const s=a.slice().sort((x,y)=>x-y);return {meanMs:s.reduce((v,x)=>v+x,0)/s.length,medianMs:s[Math.floor(s.length*.5)],p95Ms:s[Math.floor(s.length*.95)],minMs:s[0],maxMs:s.at(-1)};};
let maxNumericDifference=0;
function compare(a,b,path='') {
 if(typeof a==='number'){assert.equal(typeof b,'number',path);if(Object.is(a,b))return;assert.ok(Number.isFinite(a)&&Number.isFinite(b),path);const d=Math.abs(a-b);maxNumericDifference=Math.max(maxNumericDifference,d);assert.ok(d<=1e-9,path+': '+d);return;}
 if(a instanceof Map){assert.ok(b instanceof Map,path);assert.deepEqual([...a.keys()],[...b.keys()]);for(const [k,v]of a)compare(v,b.get(k),path+'/'+k);return;}
 if(a&&typeof a==='object'){assert.deepEqual(Object.keys(a),Object.keys(b),path);for(const k of Object.keys(a))compare(a[k],b[k],path+'/'+k);return;}
 assert.equal(a,b,path);
}
for(let stage=0;stage<states.length;stage++){
 const state=states[stage],oldWorkspace=baseline.createCompositeJointTimeStepWorkspace(state),newWorkspace=createCompositeJointTimeStepWorkspace(state);
 const runOld=()=>baseline.advanceCompositeJointTimeStep(state,options(f,state,{workspace:oldWorkspace}));
 const runNew=()=>advanceCompositeJointTimeStep(state,options(f,state,{workspace:newWorkspace}));
 for(let i=0;i<40;i++){accepted(runOld());accepted(runNew());}
 const oldTimes=[],newTimes=[];let last;
 for(let i=0;i<100;i++){
   let a,b;
   for(const key of i%2?['new','old']:['old','new']){const start=performance.now();const r=key==='old'?runOld():runNew();const elapsed=performance.now()-start;accepted(r);(key==='old'?oldTimes:newTimes).push(elapsed);if(key==='old')a=r;else b=r;}
   for(const key of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])compare(a[key],b[key],key);
   verifyPhysical(f,a);verifyPhysical(f,b);
   assert.equal(a.diagnostics.directions,b.diagnostics.directions);assert.equal(a.diagnostics.evaluations,b.diagnostics.evaluations);assert.deepEqual(a.diagnostics.acceptedAlphas,b.diagnostics.acceptedAlphas);last=b;
 }
 summary.push({stage:stage===0?'first-loaded-dt':'second-loaded-dt',pairs:100,warmupPairs:40,triadHessian:quantiles(oldTimes),closedOmega:quantiles(newTimes),directions:last.diagnostics.directions,evaluations:last.diagnostics.evaluations,fullAssemblies:last.diagnostics.fullAssemblies,gradientAssemblies:last.diagnostics.gradientAssemblies});
}
console.log(JSON.stringify({scope:'whole 3-node 2-tool one-side-contact physical dt including material-history preparation and commit; fixed small fixture, no World/render/FPS claim',dt,nodeCount:3,frictionMu:f.contacts.friction.mu,maxNumericDifference,summary},null,2));
