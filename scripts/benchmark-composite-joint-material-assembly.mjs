import { performance } from 'node:perf_hooks';
import { pathToFileURL } from 'node:url';
const root=process.cwd(),get=path=>import(pathToFileURL(`${root}/${path}`));
const {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain}=await get('src/physics/kirchhoffCompositeChain.js');
const {createCompositeJointAssembly}=await get('src/physics/kirchhoffCompositeJointAssembly.js');
const {captureCompositeReferenceFrames,compileCompositeMaterial}=await get('src/physics/kirchhoffCompositeElement.js');
const percentile=(values,p)=>[...values].sort((a,b)=>a-b)[Math.ceil(p*values.length)-1];
const results=[];
for(const count of [65,201]) {
  const coordinates=Array.from({length:count},(_,i)=>5*i),positions=coordinates.map((x,i)=>[x,.2*Math.sin(i*.03),.1*Math.cos(i*.04)]);
  const reference=captureCompositeReferenceFrames(positions),material=compileCompositeMaterial({EI1:10,EI2:10,GJ:4.55});
  const tools=[{id:'wire',reference,referenceTwists:new Float64Array(count-2),dsDx:1,material,angles:new Float64Array(count-1)},
    {id:'catheter',reference,referenceTwists:new Float64Array(count-2),dsDx:1,material:compileCompositeMaterial({EI1:25,EI2:25,GJ:5}),angles:new Float64Array(count-1)}];
  const modes=Array.from({length:count-2},(_,i)=>({node:i+1,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
  const joint=createCompositeJointAssembly({layout:createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter'])),coordinates,modes,tools,inertia:null});
  const state={positions,relative:Float64Array.from({length:3*(count-2)},(_,i)=>.02*Math.sin(i*.07)),angles:new Map(tools.map(t=>[t.id,t.angles]))};
  const localModes=modes.slice(Math.floor(modes.length/2),Math.floor(modes.length/2)+3);
  const localJoint=createCompositeJointAssembly({layout:createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter'])),coordinates,modes:localModes,tools,inertia:null});
  const localState={...state,relative:state.relative.slice(0,3*localModes.length)};
  const one=createCompositeChainWorkspace(createCompositeChainLayout(Array.from({length:count-1},()=>['wire'])),{elementBackend:'wasm-exact'});
  const common=createCompositeChainWorkspace(createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter'])),{elementBackend:'wasm-exact'});
  const runs={single:()=>assembleCompositeChain({positions,coordinates,reference,tools:[tools[0]]},one),common:()=>assembleCompositeChain({positions,coordinates,reference,tools},common),
    fullRelative:()=>joint.evaluate(state),localRelative:()=>localJoint.evaluate(localState)};
  for(let i=0;i<20;i++)for(const run of Object.values(runs))run();
  const times=Object.fromEntries(Object.keys(runs).map(k=>[k,[]]));
  for(let i=0;i<70;i++)for(const key of (i%2?Object.keys(runs).reverse():Object.keys(runs))){const start=performance.now();runs[key]();times[key].push(performance.now()-start);}
  results.push({count,primalDOFs:{single:one.layout.dofCount,common:common.layout.dofCount,fullRelative:common.layout.dofCount+state.relative.length,localRelative:common.layout.dofCount+localState.relative.length},
    localReductionIsCertified:false,localRelativeStatistics:localJoint.evaluate(localState).statistics,
    timing:Object.fromEntries(Object.entries(times).map(([key,values])=>[key,{median:percentile(values,.5),p95:percentile(values,.95),mean:values.reduce((s,v)=>s+v,0)/values.length,samples:values}]))});
}
console.log(JSON.stringify({scope:'elastic assembly only, no solve/inertia/contacts/render/step acceptance; reference model full rho, not certified reduction; isolated Node alternating order, same chart/material profiles',results},null,2));
