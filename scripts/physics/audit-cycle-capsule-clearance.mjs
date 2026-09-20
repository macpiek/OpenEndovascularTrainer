/** Independent spatial capsule audit of saved trajectories. Uses continuous
 * segment/triangle distance, including intersections; no grid sample assumption.
 * This does not audit swept motion between snapshots or prove lumen membership.
 */
import {readFileSync,writeFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {createSharedAxisSegmentContact} from '../../src/physics/kirchhoffSharedAxisSegmentContact.js';
const [directory,output]=process.argv.slice(2);
if(!directory||!output)throw Error('Usage: audit-cycle-capsule-clearance.mjs CYCLE_DIRECTORY OUTPUT.json');
const shapes=JSON.parse(readFileSync(directory+'/shapes.json')),metadata=JSON.parse(readFileSync(directory+'/metadata.json'));
const radii=Object.fromEntries(metadata.tools.map(t=>[t.id,t.radius]));
const anatomy=await loadCoupledRuntimeAnatomy(),query=createSharedAxisSegmentContact(anatomy.geometry),frames=[];
try {
 for(const shape of shapes){let worst=null,total=0,overlap=0;
  for(let i=0;i+1<shape.coordinates.length;i++){
   const sa=shape.coordinates[i],sb=shape.coordinates[i+1],start=Math.max(sa,anatomy.vessel.sheath.length);
   if(sb<=start)continue;
   const cuts=[start,...(shape.catheter>start&&shape.catheter<sb?[shape.catheter]:[]),sb];
   const at=x=>shape.positions[i].map((v,k)=>v+(shape.positions[i+1][k]-v)*(x-sa)/(sb-sa)+(shape.origin?.[k]??0));
   for(let j=0;j+1<cuts.length;j++){
    const lo=cuts[j],hi=cuts[j+1],owner=(lo+hi)/2<shape.catheter?'catheter':'wire',radius=radii[owner];
    const hit=query.query(at(lo),at(hi),radius+.001),penetration=Math.max(0,radius-hit.distance);total++;
    if(penetration>1e-6)overlap++;
    if(!worst||penetration>worst.penetration)worst={edge:i,owner,radius,site:lo+hit.t*(hi-lo),distance:hit.distance,penetration,face:hit.face,crossing:hit.crossing};
   }
  }
  frames.push({index:shape.index,phase:shape.phase,wire:shape.wire,catheter:shape.catheter,total,overlap,worst});
 }
 const report={source:directory,scope:'Continuous spatial capsule clearance for saved snapshots; not temporal CCD or a lumen-membership proof.',frames:frames.length,
  intervals:frames.reduce((s,f)=>s+f.total,0),overlappingIntervals:frames.reduce((s,f)=>s+f.overlap,0),worst:frames.reduce((w,f)=>!w||(f.worst?.penetration??-Infinity)>(w.worst?.penetration??-Infinity)?f:w,null),details:frames};
 writeFileSync(output,JSON.stringify(report,null,2));console.log(JSON.stringify({...report,details:undefined},null,2));
}finally{anatomy.dispose();}
