import * as THREE from 'three';

/** Reduced self-expansion shape: a smooth axis and coupled radial springs.
 * This is a geometric equilibrium approximation, not a nitinol shell solver.
 * End rows retain the landing/branch connections; wall constraints remain hard.
 */
export function relaxGraftAxis(points,wallFit) {
    const rest=points.map(p=>p.clone());
    for(let pass=0;pass<80;pass++) {
        const next=points.map(p=>p.clone());
        for(let i=1;i<points.length-1;i++) {
            next[i].multiplyScalar(.5).addScaledVector(points[i-1],.25).addScaledVector(points[i+1],.25).lerp(rest[i],.01);
            wallFit.fit(next[i],points[i],.7);
        }
        for(let i=1;i<points.length-1;i++)points[i].copy(next[i]);
    }
    return points;
}

export function graftRingFrames(points) {
    const frames=[];
    for(let i=0;i<points.length;i++) {
        const tangent=points[Math.min(points.length-1,i+1)].clone().sub(points[Math.max(0,i-1)]).normalize();
        if(tangent.lengthSq()<.5)tangent.copy(frames.at(-1)?.tangent??new THREE.Vector3(0,1,0));
        const previous=frames.at(-1);
        const u=previous?previous.u.clone().applyQuaternion(new THREE.Quaternion().setFromUnitVectors(previous.tangent,tangent))
            :new THREE.Vector3(Math.abs(tangent.z)<.9?0:1,0,Math.abs(tangent.z)<.9?1:0).cross(tangent).normalize();
        frames.push({u,v:tangent.clone().cross(u).normalize(),tangent});
    }
    return frames;
}

/** Rebuild from the nominal round rest shape, never from a previously clipped
 * surface. Neighbouring radii share hoop/longitudinal stiffness, so a wall hit
 * creates a smooth indentation rather than an isolated folded vertex.
 */
export function fitExpandedGraft(part,wallFit,frames=null) {
    const {rows,sides,radius,points,target}=part,count=rows*sides;
    for(const p of points)wallFit.fit(p,p,.7);
    frames??=graftRingFrames(points);
    const caps=new Float64Array(count),directions=[];
    for(let i=0;i<rows;i++)for(let j=0;j<sides;j++) {
        const a=j/sides*Math.PI*2,dir=frames[i].u.clone().multiplyScalar(Math.cos(a)).addScaledVector(frames[i].v,Math.sin(a));
        const p=wallFit.fit(points[i].clone().addScaledVector(dir,radius),points[i],.7);
        const k=i*sides+j;directions.push(dir);caps[k]=Math.max(0,Math.min(radius,p.clone().sub(points[i]).dot(dir)));
    }
    let radii=caps.slice(),next=caps.slice();
    for(let pass=0;pass<64;pass++) {
        for(let i=0;i<rows;i++)for(let j=0;j<sides;j++) {
            const k=i*sides+j,left=i*sides+(j+sides-1)%sides,right=i*sides+(j+1)%sides;
            // Nominal expansion pressure competes with curvature of the fabric.
            let sum=.15*radius+2*(radii[left]+radii[right]),weight=4.15;
            if(i){sum+=3*radii[k-sides];weight+=3;}
            if(i+1<rows){sum+=3*radii[k+sides];weight+=3;}
            next[k]=Math.min(caps[k],sum/weight);
        }
        [radii,next]=[next,radii];
    }
    for(let k=0;k<count;k++) {
        const center=points[Math.floor(k/sides)];
        wallFit.fit(center.clone().addScaledVector(directions[k],radii[k]),center,.7).toArray(target,k*3);
    }
    part.ringFrames=frames;
}
