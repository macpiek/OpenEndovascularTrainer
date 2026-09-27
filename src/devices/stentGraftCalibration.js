import * as THREE from 'three';
import {AORTIC_NECK} from './stentGraftPaths.js';

export const REFERENCE_NECK_DIAMETER_MM=22;
// One orthogonal lumen section, below the lowest renal ostium and above the
// aneurysm. Area-equivalent diameter avoids treating an oblique axial cut or
// an eccentric centerline as a change of scale. Calculated once per anatomy.
export function calibrateStentGrafts(anatomy,route) {
    const wall=anatomy?.geometry?.boundsTree;
    if(!wall||!route)return {worldUnitsPerMm:1,referenceDiameterMm:REFERENCE_NECK_DIAMETER_MM,measuredDiameter:null};
    const near=route.nearest(AORTIC_NECK),center=near.point;
    const normal=route.sample(near.s+2).sub(route.sample(near.s-2)).normalize();
    const axis=Math.abs(normal.x)<.9?new THREE.Vector3(1,0,0):new THREE.Vector3(0,0,1);
    const u=axis.addScaledVector(normal,-axis.dot(normal)).normalize(),v=new THREE.Vector3().crossVectors(normal,u);
    const count=360,radii=[];
    for(let i=0;i<count;i++) {
        const angle=i*2*Math.PI/count,direction=u.clone().multiplyScalar(Math.cos(angle)).addScaledVector(v,Math.sin(angle));
        const hit=wall.raycastFirst(new THREE.Ray(center,direction),THREE.DoubleSide,0,40);
        if(!hit||!Number.isFinite(hit.distance)||hit.distance<=0)throw Error('Nie można skalibrować stentgraftów: niepełny przekrój szyi aorty.');
        radii.push(hit.distance);
    }
    let area=0;
    for(let i=0;i<count;i++)area+=.5*radii[i]*radii[(i+1)%count]*Math.sin(2*Math.PI/count);
    const measuredDiameter=2*Math.sqrt(area/Math.PI);
    return {worldUnitsPerMm:measuredDiameter/REFERENCE_NECK_DIAMETER_MM,referenceDiameterMm:REFERENCE_NECK_DIAMETER_MM,
        measuredDiameter,center:center.toArray(),normal:normal.toArray()};
}
