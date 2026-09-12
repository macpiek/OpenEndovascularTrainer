// Historical pre-reuse probe protocol. Results depend on the current source revision.
// See reports/composite-joint-timestep-pre-reuse-120hz.md for captured source-state limits.
import assert from 'node:assert/strict';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { createCompositeJointTimeStepState, advanceCompositeJointTimeStep } from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const options={dt:.06,torsionMode:'quasi-static',contacts:'none',
    tolerances:{force:1e-7,torque:1e-8,length:1e-8,boundary:1e-9,linearForce:5e-10,linearTorque:5e-10,linearConstraint:5e-11}};
const zero=[0,0,0];
function fixture({n=5,wireSlope=1.02,catheterSlope=1,angle=.07,wireVelocity=zero,catheterVelocity=zero,wireFeed=0}={}) {
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>['wire','catheter'])), coordinates=Array.from({length:n},(_,i)=>2*i);
    const positions=coordinates.map(x=>[x*catheterSlope,0,0]),wire=coordinates.map(x=>[x*wireSlope*Math.cos(angle),x*wireSlope*Math.sin(angle),.03]);
    const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const relative=Float64Array.from(wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k]))),angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const dsDx=id==='wire'?wireSlope:catheterSlope;angles.set(id,new Float64Array(n-1));restLengths.set(id,new Float64Array(n-1).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(n-2),
            // Declared test material parameters in consistent N/mm/s units,
            // not measured guidewire/catheter values or runtime tuning.
            material:compileCompositeMaterial({stiffness:(id==='wire'?[2,3,1]:[8,11,4]).map((v,i)=>Array.from({length:3},(_,j)=>i===j?v:0)),intrinsic:[0,0,0]})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const inputs={...options,inertia:inertiaFor(state,{wireVelocity,catheterVelocity,wireFeed}),boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]}};
    return {state,inputs};
}
function inertiaFor(state,{wireVelocity,catheterVelocity,wireFeed=0}={}) {
    const byId=new Map(state.tools.map(t=>[t.id,t]));
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        massPerMaterialLength:id==='wire'?.13:.24,
        materialMap:{sStart:20+state.coordinates[e]*byId.get(id).dsDx,dsDx:byId.get(id).dsDx,dsDt:id==='wire'?wireFeed:0},
        oldMaterialVelocities:(id==='wire'?wireVelocity:catheterVelocity)!==undefined
            ?[Array.from(id==='wire'?wireVelocity:catheterVelocity),Array.from(id==='wire'?wireVelocity:catheterVelocity)]
            :(state.materialVelocities?.[e].tools.find(t=>t.id===id)?.velocities??[zero,zero]).map(v=>Array.from(v))}))}))};
}
function run(f,extra={}) {return advanceCompositeJointTimeStep(f.state,{...f.inputs,...extra});}
function accepted(result) {assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error,diagnostics:result.diagnostics}));}
function originalLengths(result) {
    for(const [id,p] of result.state.toolPositions)for(let e=0;e<p.length-1;e++)
        close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),result.state.restLengths.get(id)[e],1e-8);
}
function balance(result,t=5e-7) {for(const b of result.balances.values())vectorClose(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]),t);}


const f=fixture({n:33}),loads={forces:[{toolId:'wire',node:32,value:[.01,.025,-.012]}]};f.inputs.dt=1/120;
function pair(){const first=run(f,{loads});accepted(first);const second=advanceCompositeJointTimeStep(first.state,{...f.inputs,loads,inertia:inertiaFor(first.state)});accepted(second);return [first,second];}
for(let i=0;i<30;i++)pair();
const records=[];
for(let i=0;i<50;i++)records.push(pair().map(r=>({accepted:r.accepted,totalMs:r.diagnostics.totalMs,preparationMs:r.diagnostics.preparationMs,iterationMs:r.diagnostics.iterationMs,commitMs:r.diagnostics.commitMs,directions:r.diagnostics.directions,evaluations:r.diagnostics.evaluations,fullAssemblies:r.diagnostics.fullAssemblies,gradientAssemblies:r.diagnostics.gradientAssemblies})));
const median=v=>[...v].sort((a,b)=>a-b)[Math.floor(v.length/2)],p95=v=>[...v].sort((a,b)=>a-b)[Math.ceil(.95*v.length)-1];
console.log(JSON.stringify({scope:'33 nodes, synthetic declared material parameters, no contacts, dt1/120, fresh physical workspace each dt, 30 pair warmups and50 measured pairs; Node only, not runtime/FPS',statistics:[0,1].map(j=>Object.fromEntries(['totalMs','preparationMs','iterationMs','commitMs','directions','evaluations','fullAssemblies','gradientAssemblies'].map(k=>[k,{median:median(records.map(r=>r[j][k])),p95:p95(records.map(r=>r[j][k]))}]))),records},null,2));
