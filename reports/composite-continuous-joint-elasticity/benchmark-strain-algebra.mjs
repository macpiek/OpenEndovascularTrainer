import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
import {performance} from 'node:perf_hooks';
const root=process.cwd(),url=name=>pathToFileURL(root+'/src/physics/'+name).href;
const before=await import('./before-frame.mjs'),after=await import(url('kirchhoffCompositeContinuousFrame.js'));
const {createCompositeContinuousGeometry}=await import(url('kirchhoffCompositeContinuousGeometry.js'));
const {captureCompositeReferenceFrames}=await import(url('kirchhoffCompositeElement.js'));
const median=v=>{const s=v.slice().sort((a,b)=>a-b);return s[Math.floor(s.length/2)];};
const results=[];
for(const count of [5,12]) {
 const edge=count===5?1:5,coordinates=Array.from({length:count},(_,j)=>j),positions=coordinates.map(x=>[x,.003*x*x,.0001*x*x*x]),angles=coordinates.slice(1).map((_,j)=>.04*j);
 const source={geometry:createCompositeContinuousGeometry({coordinates}),edge,toolId:'wire',previousPositions:positions,previousAngles:angles,reference:captureCompositeReferenceFrames(positions,[0,0,1]),referenceTwists:new Array(count-2).fill(0)};
 const plans=[before,after].map(api=>({api,frame:api.createCompositeContinuousFrame(source),workspace:api.createCompositeContinuousFrameWorkspace()}));
 for(const plan of plans)plan.input={positions:plan.frame.positionNodeIndices.map(j=>positions[j]),angles:plan.frame.angleEdgeIndices.map(j=>angles[j]),coordinates:Array.from({length:12},(_,j)=>edge+(j+.5)/12),dsDx:1.3};
 const run=side=>{const p=plans[side];return p.api.evaluateCompositeContinuousStrains(p.input,p.frame,p.workspace);};
 const values=[run(0),run(1)],error={strain:0,jacobian:0,hessian:0};
 for(let j=0;j<12;j++)for(const key of Object.keys(error))values[0][j][key].forEach((v,k)=>error[key]=Math.max(error[key],Math.abs(v-values[1][j][key][k])));
 for(let j=0;j<40;j++){run(j%2);run(1-j%2);}
 const times=[[],[]];
 for(let j=0;j<60;j++)for(const side of [j%2,1-j%2]){const start=performance.now();run(side);times[side].push(performance.now()-start);}
 results.push({configurationDofs:plans[0].frame.configurationDofs,batchSize:12,maxDifference:error,operationsPerSample:values.map(v=>v[0].operations),medianBatchMs:times.map(median),samples:times});
}
fs.writeFileSync('/tmp/oet-continuous-strain-algebra-benchmark.json',JSON.stringify({scope:'paired synthetic continuous strain batch only; no timestep or FPS claim',warmupPairs:40,measurementPairs:60,results},null,2)+'\n');
console.log(JSON.stringify(results.map(({samples,...r})=>r),null,2));
