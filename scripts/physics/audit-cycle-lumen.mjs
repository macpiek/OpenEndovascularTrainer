/** Independent diagnostic: query the packed anatomical lumen on saved world
 * trajectories without inheriting solver inside-continuation certificates.
 * The sliced field is approximate: disagreement is a warning, not by itself
 * a proof of penetration. Outside the mesh AABB is reported separately.
 */
import {readFileSync,writeFileSync} from 'node:fs';
import {Vector3} from 'three';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
const [input,output]=process.argv.slice(2);
if(!input||!output)throw Error('Usage: node audit-cycle-lumen.mjs shapes.json report.json');
const shapes=JSON.parse(readFileSync(input)),anatomy=await loadCoupledRuntimeAnatomy();
try {
 const {geometry,field}=anatomy,point=new Vector3(),closest={},bounds=geometry.boundingBox;
 const frames=[];
 for(const shape of shapes) {
  let samples=0,outside=0,beyondBounds=0,farOutside=0,worst=null;
  const examples=[];
  for(let i=0;i+1<shape.positions.length;i++) {
   const a=shape.positions[i],b=shape.positions[i+1],sa=shape.coordinates[i],sb=shape.coordinates[i+1];
   if(sb<=anatomy.vessel.sheath.length)continue;
   const count=Math.max(1,Math.ceil(Math.hypot(...a.map((v,k)=>v-b[k]))/2));
   for(let j=0;j<=count;j++) {
    const t=j/count,site=sa+(sb-sa)*t;if(site<=anatomy.vessel.sheath.length)continue;
    point.fromArray(a.map((v,k)=>(1-t)*v+t*b[k]+(shape.origin?.[k]??0)));
    const q=field.packedLumenField.queryCoordinates(point.x,point.y,point.z);samples++;
    const outsideBounds=!bounds.containsPoint(point);if(outsideBounds)beyondBounds++;
    if(q.signedDistance<0) {
     outside++;
     geometry.boundsTree.closestPointToPoint(point,closest);
     const record={site,point:point.toArray(),packedSignedDistance:q.signedDistance,
      exactSurfaceDistance:closest.distance,face:closest.faceIndex,outsideBounds};
     if(q.signedDistance < -2 && closest.distance>2){farOutside++;if(examples.length<3)examples.push(record);}
     if(!worst||q.signedDistance<worst.packedSignedDistance)worst=record;
    }
   }
  }
  frames.push({index:shape.index,phase:shape.phase,wire:shape.wire,catheter:shape.catheter,samples,outside,farOutside,beyondBounds,worst,examples});
 }
 const report={source:input,scope:'Saved trajectory samples at <=2 mm spacing; sheath interval excluded. Packed lumen disagreement is approximate, not an independently validated anatomical containment certificate.',bounds:{min:bounds.min.toArray(),max:bounds.max.toArray()},
  frames:frames.length,samples:frames.reduce((n,f)=>n+f.samples,0),framesWithFarOutside:frames.filter(f=>f.farOutside).length,
  firstFarOutside:frames.find(f=>f.farOutside)??null,framesWithBeyondBounds:frames.filter(f=>f.beyondBounds).length,details:frames};
 writeFileSync(output,JSON.stringify(report,null,2));console.log(JSON.stringify({...report,details:undefined},null,2));
} finally {anatomy.dispose();}
