import assert from 'node:assert/strict';
import { pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
const root=process.argv[2];
const mod=async (arm,file)=>import(pathToFileURL(`${root}/${arm}/src/physics/${file}.js`));
const [current,previous,direction,chain,element]=await Promise.all([
 mod('reuse','kirchhoffCompositeJointTimeStep'),mod('reuse/before','kirchhoffCompositeJointTimeStep'),mod('direction','kirchhoffCompositeJointTimeStep'),
 mod('reuse','kirchhoffCompositeChain'),mod('reuse','kirchhoffCompositeElement')]);
const hash=x=>createHash('sha256').update(JSON.stringify(x,(_,v)=>v instanceof Map?[...v]:ArrayBuffer.isView(v)?Array.from(v):v)).digest('hex');
const options={dt:1/120,torsionMode:'quasi-static',contacts:'none'};
const n=7,coordinates=Array.from({length:n},(_,i)=>2*i);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
function fixture(k=0,mixed=true) {
 const layout=chain.createCompositeChainLayout(Array.from({length:n-1},(_,e)=>mixed&&e===0?['wire']:mixed&&e===n-2?['catheter']:['wire','catheter']));
 const ws=1.015+.001*(k%3),cs=1+.001*(k%2),angle=.04+.001*k;
 const cat=coordinates.map(x=>[x*cs,0,0]),wire=coordinates.map(x=>[x*ws*Math.cos(angle),x*ws*Math.sin(angle),.035]);
 const positions=cat.map(p=>p.slice());if(mixed)positions[0]=wire[0].slice();
 const active=node=>[...new Set([...(layout.edgeToolIds[node-1]??[]),...(layout.edgeToolIds[node]??[])])];
 const modes=coordinates.flatMap((_,node)=>active(node).length===2?[{node,basis:[[Math.cos(.23+.02*node),Math.sin(.23+.02*node),0],[-Math.sin(.23+.02*node),Math.cos(.23+.02*node),0],[0,0,1]]}]:[]);
 const relative=Float64Array.from(modes.flatMap(m=>m.basis.map(b=>dot(b,wire[m.node].map((v,j)=>v-positions[m.node][j])))));
 const physicalWire=positions.map(p=>p.slice());modes.forEach((m,i)=>m.basis.forEach((b,a)=>b.forEach((v,j)=>physicalWire[m.node][j]+=v*relative[3*i+a])));
 const angles=new Map(),restLengths=new Map();
 const tools=[...layout.spins].map(([id,spins])=>{
  const slope=id==='wire'?ws:cs,p=id==='wire'?physicalWire:positions;
  angles.set(id,Float64Array.from(spins,d=>d<0?NaN:0));restLengths.set(id,Float64Array.from(spins,d=>d<0?NaN:2*slope));
  return{id,dsDx:slope,reference:element.captureCompositeReferenceFrames(p),referenceTwists:Float64Array.from({length:n-2},()=>0),
   material:element.compileCompositeMaterial({stiffness:id==='wire'?[[2+.01*k,.1,.02],[.1,3,.03],[.02,.03,1]]:[[9,.2,.1],[.2,12,.1],[.1,.1,4]],intrinsic:[.0001*k,-.00005*k,.00003*k],energyOffset:.0002*k})};
 });
 const state=current.createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles,restLengths,tools,materialCoordinate:'reference-arclength'});
 state.lengthMultipliers.forEach((_,i)=>state.lengthMultipliers[i]=.00003*Math.sin(i+k));
 return state;
}
function input(state,k=0) {
 const byId=new Map(state.tools.map(t=>[t.id,t]));
 const inertia={previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:(id==='wire'?.12:.27)*(1+.013*k),
 materialMap:{sStart:40+.01*k+state.coordinates[e]*byId.get(id).dsDx,dsDx:byId.get(id).dsDx,dsDt:[.0002*Math.sin(k),-.0002*Math.cos(k)]},
 oldMaterialVelocities:state.materialVelocities?.[e].tools.find(t=>t.id===id).velocities.map(v=>v.slice())??[[.0001*k,0,0],[0,-.0001*k,.0002*k]]}))}))};
 const first=id=>state.layout.spins.get(id).findIndex(d=>d>=0);
 const pattern=k%4,wireNode=first('wire')+(pattern===1?1:0),catNode=first('catheter')+(pattern===2?1:0);
 const pins=[['wire',wireNode],['catheter',catNode]];if(pattern===3)pins.push(['catheter',catNode+1]);
 const shift=[.000005*Math.sin(k),-.000004*Math.cos(k),.000003*Math.sin(2*k)];
 const positions=pins.map(([toolId,node])=>({toolId,node,value:state.toolPositions.get(toolId)[node].map((v,j)=>v+shift[j])}));
 return {...options,dt:(1+(k%3)*.07)/120,inertia,boundaries:{positions,spins:['wire','catheter'].map(toolId=>({toolId,edge:first(toolId)+(k%2),value:.002*Math.sin(k+Number(toolId==='wire'))}))},
 loads:{forces:[{toolId:'wire',node:state.layout.nodeCount-2,value:[.002,.004+.0001*k,-.001]}],torques:[{toolId:'catheter',edge:n-2,value:.00002*Math.cos(k)}]}};
}
const physicalKeys=['accepted','status','error','state','perTool','boundaryForces','spinReactions','balances'];
const diagKeys=['certificate','directions','evaluations','fullAssemblies','gradientAssemblies','linearSolves','factorizations','acceptedAlphas','retainedLengthRows','suppressedPrescribedLengthRows','invalidTrials'];
function equal(a,b,label) {for(const key of physicalKeys)assert.deepEqual(a[key],b[key],`${label}.${key}`);for(const key of diagKeys)assert.deepEqual(a.diagnostics[key],b.diagnostics[key],`${label}.diagnostics.${key}`);}
const records=[];let rejects=0,errors=0,accepted=0;
for(const mixed of [false,true]) {
 const first=fixture(0,mixed),warm=current.createCompositeJointTimeStepWorkspace({...first,rowCacheCapacity:1}),newWarm=direction.createCompositeJointTimeStepWorkspace({...first,rowCacheCapacity:1});
 const arms=[['before',previous,null],['cold',current,null],['reuse',current,warm],['direction',direction,newWarm]];
 const saved=[];
 for(let k=0;k<12;k++) {
  let state=fixture(k,mixed);const inputs=input(state,k),frozen=hash(state);
  // Accepted-history branches are supplied by the caller. No previous
  // workspace result is a permitted replacement for these values.
  for(const b of inputs.boundaries.positions)if(b.toolId==='wire')state.boundaryMultipliers.set(JSON.stringify([b.toolId,b.node]),[.0002,-.0001,.0003]);
  const beforeState=hash(state),results=arms.map(([name,api,workspace])=>api.advanceCompositeJointTimeStep(state,{...inputs,workspace}));
  for(let a=1;a<results.length;a++)equal(results[a],results[0],`${mixed}/${k}/${arms[a][0]}`);
  const r=results[0];assert.equal(r.accepted,true,`${mixed}/${k}: ${r.status} ${r.error}`);accepted+=results.length;
  assert.equal(hash(state),beforeState,'input state remains owned');
  for(const result of results)for(const b of result.balances.values())assert.ok(Math.hypot(...b.residual)<2e-6,'per-tool momentum balance');
  saved.push(...results.map(result=>[result,hash(result)]));
  // A deformed final-certificate rejection must not poison the next call.
  const budget={evaluations:Math.max(1,r.diagnostics.evaluations-1)};
  const rejected=arms.map(([name,api,workspace])=>api.advanceCompositeJointTimeStep(state,{...inputs,budget,workspace}));
  assert.equal(rejected[0].accepted,false);
  rejected.forEach((result,a)=>{equal(result,rejected[0],`reject/${a}`);assert.equal(result.state,state);rejects++;});
  const retried=arms.map(([name,api,workspace])=>api.advanceCompositeJointTimeStep(state,{...inputs,workspace}));
  retried.forEach((result,a)=>{equal(result,r,`retry/${a}`);accepted++;});
  // Continue one physical step with the accepted per-edge old velocities.
  state=results[2].state;const nextInput=input(state,k);
  const continued=arms.map(([name,api,workspace])=>api.advanceCompositeJointTimeStep(state,{...nextInput,workspace}));
  assert.equal(continued[0].accepted,true,`continued ${mixed}/${k}`);
  continued.forEach((result,a)=>{equal(result,continued[0],`continued/${a}`);accepted++;});
  if(k%3===0)for(const [,api,workspace] of arms.slice(2)) {
   const bad=structuredClone(inputs);bad.boundaries.positions.push(structuredClone(bad.boundaries.positions[0]));
   assert.throws(()=>api.advanceCompositeJointTimeStep(fixture(k,mixed),{...bad,workspace}),/distinct/);errors++;
  }
  for(const [old,digest] of saved)assert.equal(hash(old),digest,'previous returned state/reactions/diagnostics cannot alias scratch');
  records.push({mixed,k,initialHash:frozen,stateHash:hash(r.state),continuedHash:hash(continued[0].state),directions:r.diagnostics.directions,evaluations:r.diagnostics.evaluations,rejection:rejected[0].status});
 }
 assert.ok(warm.diagnostics.directionBuilds>1);assert.equal(warm.diagnostics.retainedRowPatterns,1);
 records.push({mixed,workspace:warm.diagnostics,directionWorkspace:newWarm.diagnostics});
}
console.log(JSON.stringify({status:'PASS',scope:'independent mixed/full fixed-chart sequence; all four arms exact physical equality; no contacts',accepted,rejects,preparationErrors:errors,records},null,2));
