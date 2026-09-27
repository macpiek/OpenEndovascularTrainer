import * as THREE from 'three';

export const CROWN_SUBDIVISIONS=12;
export function crownMaterial(diameter,scale=1) {
    // Fixed straight metal arms; opening changes only the angle at the apex.
    const radius=diameter*scale/2,height=12*scale;
    const armLength=Math.hypot(height,2*radius*Math.sin(Math.PI/24));
    return {edge:armLength/CROWN_SUBDIVISIONS,armLength,hookLength:1.5*scale};
}

/** Both sewn roots are fixed. The shared apex lies on the intersection of two
 * equal-radius spheres, so both straight arms always retain their length.
 * Wall fitting selects an orientation of that rigid V, never clips its vertices.
 */
export function inextensibleCrown(left,right,material,fit,heldApex=null) {
    const n=left.length-1,a=left[0],b=right[0],mid=a.clone().lerp(b,.5);
    if(heldApex) {
        const lines=[a,b].map(root=>Array.from({length:n+1},(_,j)=>root.clone().lerp(heldApex,j/n)));
        return {left:lines[0],right:lines[1],error:Math.max(...[a,b].map(root=>Math.abs(root.distanceTo(heldApex)/material.armLength-1))),wallError:0};
    }
    const across=b.clone().sub(a),separation=across.length();
    if(separation>2*material.armLength)throw new Error('Crown roots exceed the fixed metal span');
    across.normalize();
    const preferred=left.at(-1).clone().sub(mid).addScaledVector(across,-left.at(-1).clone().sub(mid).dot(across));
    if(preferred.lengthSq()<1e-12)preferred.crossVectors(across,Math.abs(across.y)<.9?new THREE.Vector3(0,1,0):new THREE.Vector3(1,0,0));
    preferred.normalize();
    const height=Math.sqrt(Math.max(0,material.armLength**2-separation**2/4));
    const candidate=angle=>{
        const apex=mid.clone().addScaledVector(preferred.clone().applyAxisAngle(across,angle),height);
        return [a,b].map(root=>Array.from({length:n+1},(_,j)=>root.clone().lerp(apex,j/n)));
    };
    let best=candidate(0),bestError=Infinity;
    for(let k=0;k<=128;k++) {
        const angle=k===0?0:Math.ceil(k/2)*Math.PI/64*(k%2?1:-1),lines=k?candidate(angle):best;
        let error=0;
        if(fit)for(const line of lines)for(let j=1;j<=n;j++) {
            const fitted=fit(line[j].clone(),j);
            error=Math.max(error,fitted.distanceToSquared(line[j]));
        }
        if(error<bestError){best=lines;bestError=error;}
        if(error<1e-12)break;
    }
    return {left:best[0],right:best[1],error:0,wallError:Math.sqrt(bestError)};
}
