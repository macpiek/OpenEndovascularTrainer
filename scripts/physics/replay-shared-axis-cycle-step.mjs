/** Replay a failed-incoming/slow-step full-cycle snapshot, including the
 * certified geometry state. OPTIONS is an explicit experimental override. */
import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {dirname,join} from 'node:path';
import {installExperimentalCollisionGeometry} from './experimental-collision-geometry.mjs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const [inputPath,outputPath]=process.argv.slice(2);
if(!inputPath||!outputPath)throw new Error('Usage: node scripts/physics/replay-shared-axis-cycle-step.mjs input.json output.json');
const input=JSON.parse(readFileSync(inputPath)),anatomy=await loadCoupledRuntimeAnatomy();
// Older research captures stored provenance only beside the snapshot.
const metadataPath=join(dirname(inputPath),'metadata.json');
const collision=input.collisionExperiment??(existsSync(metadataPath)?JSON.parse(readFileSync(metadataPath)).collisionExperiment:null);
if(process.env.COLLISION_WORLD_STL&&!collision)throw new Error('Cannot change triangle identities in a reference replay');
const experiment=collision?installExperimentalCollisionGeometry(anatomy,process.env.COLLISION_WORLD_STL??collision.path,collision):null;
try {
 const s=restoreSharedAxisReplay(input,anatomy.field),events=[],request=input.stepRequest;
 const options={...request.options,...JSON.parse(process.env.OPTIONS??'{}')};
 const started=performance.now();
 const it=advanceSharedAxis(s,request.rotations,request.dt,request.tools,{...options,observeTrial:e=>{
  if(e.kind==='outside-vessel'||e.kind==='projection'||e.kind==='residual-search-activation')events.push(e);
  if(e.kind==='direction')events.push({kind:e.kind,iteration:e.iteration,method:e.method,force:e.base.force,torque:e.base.torque,constraint:e.base.constraint,
   liveNormalLoad:e.state.wallFrictionStep?.liveNormalLoad===true,
   frictionModes:e.state.wallFrictionStep?.records.reduce((counts,r)=>{counts[r.mode]=(counts[r.mode]??0)+1;return counts;},{}),
   worstRows:e.base.rows.map((r,i)=>({index:i,id:r.id,kind:r.kind,subtype:r.subtype,edge:r.edge,gap:r.gap,multiplier:r.multiplier,coordinate:e.state.coordinates[r.edge],violation:Math.abs(r.kind==='length'?r.gap:Math.max(0,r.multiplier-r.gap)-r.multiplier)})).sort((a,b)=>b.violation-a.violation).slice(0,3),
   converged:e.direction.converged,failure:e.direction.failure,factorizations:e.direction.factorizations,
   ...(e.direction.condensationStats?{condensationStats:e.direction.condensationStats}:{}),
   maxPosition:e.direction.increment?Math.max(...Array.from(e.state.layout.positions,p=>Math.hypot(...e.direction.increment.slice(p,p+3)))):null,
   maxIncrement:e.direction.increment?Math.max(...e.direction.increment.map(Math.abs)):null});
  if(e.kind==='trial')events.push({kind:e.kind,iteration:e.iteration,method:e.method,trial:e.trial,rootSearch:e.rootSearch??false,scale:e.scale,accept:e.accept,
   force:e.candidate.force,torque:e.candidate.torque,constraint:e.candidate.constraint});
 }});let next;do{next=it.next();}while(!next.done);
 const elapsedMs=performance.now()-started;
 writeFileSync(outputPath,JSON.stringify({inputPath,collisionExperiment:experiment?.metadata??null,options,elapsedMs,result:next.value.result,events},null,2));
 console.log(JSON.stringify({status:next.value.result.status,error:next.value.result.error,iterations:next.value.result.iterations,
  factorizations:next.value.result.factorizations,events:events.length}));
}finally{experiment?.dispose();anatomy.dispose();}
