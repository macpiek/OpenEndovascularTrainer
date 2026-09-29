import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';
import {relaxGraftAxis,fitExpandedGraft} from './stentGraftExpansion.js';

// Shared surface construction for the implant and its dimensioned catalogue view.
export function buildGraftPart({points,radius,endRadius=radius,distalStraight=null,radiusProfile=null,dimensionScale=1,oppositeBranch=null,wallFit,fabricMaterial}) {
        const rows=points.length,sides=24,positions=new Float32Array(rows*sides*3),target=new Float32Array(positions.length),indices=[];
        relaxGraftAxis(points,wallFit,oppositeBranch);
        for(let i=0;i<rows-1;i++)for(let j=0;j<sides;j++) {
            const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,e=b+sides;indices.push(a,c,b,b,c,e);
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,fabricMaterial);mesh.frustumCulled=false;
        const part={points,rows,sides,radius,target,mesh,path:new DevicePath(points),exposure:new Float64Array(rows).fill(-1)};
        part.rowRadii=Float64Array.from(points,(_,i)=>{
            if(radiusProfile)return radiusProfile(i/(rows-1));
            const t=distalStraight===null?i/(rows-1)
                :THREE.MathUtils.clamp((part.path.coordinates[i]-(part.path.length-distalStraight-10*dimensionScale))/(10*dimensionScale),0,1);
            const smooth=t*t*t*(10-15*t+6*t*t);
            return distalStraight===null?radius+(endRadius-radius)*smooth:endRadius+(radius-endRadius)*smooth;
        });
        fitExpandedGraft(part,wallFit);
        part.path=new DevicePath(points);
        return part;
}
