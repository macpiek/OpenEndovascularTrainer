/** Exact BVH ray audit of saved rod centerline segments. A hit proves a
 * centerline/surface intersection; no hit does not certify capsule clearance
 * or temporal swept collision. Sheath interior is excluded. */
import fs from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {Ray,Vector3,DoubleSide} from 'three';
const [input,output]=process.argv.slice(2);
if(!input||!output)throw Error('Usage: audit-cycle-surface-crossings.mjs shapes.json output.json');
const anatomy=await loadCoupledRuntimeAnatomy(),shapes=JSON.parse(fs.readFileSync(input)),results=[],ray=new Ray();
for(const s of shapes){const hits=[];for(let i=0;i+1<s.positions.length;i++){
 if(s.coordinates[i+1]<=anatomy.vessel.sheath.length)continue;
 const a=new Vector3(...s.positions[i]).add(new Vector3(...s.origin)),b=new Vector3(...s.positions[i+1]).add(new Vector3(...s.origin)),d=b.clone().sub(a),length=d.length();ray.origin.copy(a);ray.direction.copy(d).normalize();
 const hit=anatomy.geometry.boundsTree.raycastFirst(ray,DoubleSide,0,length);
 if(hit)hits.push({edge:i,site:s.coordinates[i]+hit.distance/length*(s.coordinates[i+1]-s.coordinates[i]),point:hit.point.toArray(),face:hit.faceIndex,edgeLength:length});
 }if(hits.length)results.push({index:s.index,phase:s.phase,wire:s.wire,catheter:s.catheter,hits});}
fs.writeFileSync(output,JSON.stringify(results,null,2));console.log(JSON.stringify({frames:results.length,first:results[0]},null,2));anatomy.dispose();
