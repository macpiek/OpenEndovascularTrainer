// Three archived tool-only steps; run from any directory.
// Outputs three repetitions plus independent, uncached containment checks.
import fs from 'node:fs';import {gunzipSync}from 'node:zlib';
import {loadCoupledRuntimeAnatomy}from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay}from '../tests/helpers/sharedAxisReplay.js';
import {createSharedAxisSegmentContact}from '../src/physics/kirchhoffSharedAxisSegmentContact.js';
import {sharedAxisOuterIntervals}from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis}from '../src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy();
for(let round=0;round<3;round++)for(const name of ['anatomy-berenstein-feed-138.67-live-cycle.json','anatomy-pigtail-wire-withdraw-200.27-incoming.json','anatomy-wire-569.80-poor-prediction.json.gz'])for(const mode of [true]){
const data=fs.readFileSync(new URL('../tests/fixtures/shared-axis/'+name,import.meta.url)),f=JSON.parse(name.endsWith('.gz')?gunzipSync(data):data);f.continuousSegmentContacts=mode;
// The archived samples predate this mesh revision; reconstruct certificates from exact queries.
delete f.discoveryState;delete f.insideContinuation;
const s=restoreSharedAxisReplay(f,anatomy.field),q=s.wallSamples.find(x=>x.segmentContact)?.segmentContact;let queryMs=0,maxRows=s.definitions.length;
if(q){const query=q.query.bind(q);q.query=(...args)=>{const t=performance.now();try{return query(...args)}finally{queryMs+=performance.now()-t}};}
const req=f.stepRequest; req.options={...req.options,forceTolerance:1e-4,lengthTolerance:1e-3}; const t=performance.now(),it=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,observeIteration:({state})=>maxRows=Math.max(maxRows,state.definitions.length)});let n;
do{n=it.next();if(performance.now()-t>20000){it.return();break;}}while(!n.done);
if(!n.done||!n.value?.state||!n.value.result.converged)throw new Error(JSON.stringify({name,result:n.value?.result,timeout:!n.done}));
const r=n.value.result,elapsed=performance.now()-t;
let maximumCapsuleOverlap=0,crossings=0;
if(n.value?.state){const accepted=n.value.state,exact=createSharedAxisSegmentContact(anatomy.geometry),sheathLength=s.wallSamples.find(x=>x.segmentContact).sheathLength;
 for(let e=0;e<accepted.positions.length-1;e++){
 const start=accepted.coordinates[e],end=accepted.coordinates[e+1];if(end<=sheathLength)continue;
 for(const interval of sharedAxisOuterIntervals(accepted,e)){
 const ta=Math.max(interval.start,(sheathLength-start)/(end-start),0),tb=interval.end;if(tb<=ta)continue;
 const point=t=>accepted.positions[e].map((v,k)=>v*(1-t)+accepted.positions[e+1][k]*t+accepted.origin[k]);
 const radius=interval.material.body.radius,hit=exact.query(point(ta),point(tb),radius+.01);
 maximumCapsuleOverlap=Math.max(maximumCapsuleOverlap,radius-hit.distance);if(hit.crossing)crossings++;
 }}
 if(crossings||maximumCapsuleOverlap>1.0001e-4)throw Error(JSON.stringify({name,crossings,maximumCapsuleOverlap}));
}
console.log(JSON.stringify({round,name,mode,ms:elapsed,maximumCapsuleOverlap,crossings,queryMs,stats:q?.stats,maxRows,converged:r.converged,status:r.status,error:r.error,iterations:r.iterations,factorizations:r.factorizations,assemblies:r.fullAssemblies,quality:r.quality,geometryRestarts:r.geometryRestarts,timings:r.timings}));
}
anatomy.dispose();
