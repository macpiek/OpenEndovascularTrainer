import * as THREE from 'three';

/** A sewn bifurcation is part of the implant, not a fixed point in the atlas.
 * Its free backbone spans the proximal landing and distal delivery-side end.
 * Keep both branch offsets, but let the unsupported crotch and gate move with
 * that backbone instead of forcing three separately straightened tubes into S.
 */
export function graftRestAxes(trunk,ipsi,gate,dimensions,overlap,wallFit=null) {
    const origin=trunk[0].clone(),fork=trunk.at(-1).clone();
    const offsets=[new THREE.Vector3(),ipsi[0].clone().sub(fork).sub(overlap),gate[0].clone().sub(fork).sub(overlap)];
    const end=ipsi.at(-1).clone().sub(offsets[1]);
    const total=dimensions.trunkLength+dimensions.ipsiLength;
    for(const [index,points] of [trunk,ipsi,gate].entries())for(let i=0;i<points.length;i++) {
        const length=[dimensions.trunkLength,dimensions.ipsiLength,dimensions.contraLength][index];
        const distance=(index?dimensions.trunkLength:0)+length*i/(points.length-1);
        const target=origin.clone().lerp(end,distance/total).add(offsets[index]);
        if(index&&i===0)target.add(overlap);
        if(wallFit)wallFit.fit(target,points[i],.7);
        points[i].copy(target);
    }
}

/** Reduced self-expansion shape: a smooth axis and coupled radial springs.
 * This is a geometric equilibrium approximation, not a nitinol shell solver.
 * End rows retain the landing/branch connections; wall constraints remain hard.
 */
export function relaxGraftAxis(points,wallFit,oppositeBranch=null) {
    if(points.length<3)return points;
    // The anatomy is an initial route, not an intrinsic curvature. Start from
    // the zero-curvature shape between the connected ends, then solve the
    // wall-constrained axis. A local smoothing pass tethered to the old route
    // retained large S-bends even in a completely unconstrained aneurysm.
    const coordinates=[0];
    for(let i=1;i<points.length;i++)coordinates.push(coordinates[i-1]+points[i].distanceTo(points[i-1]));
    const length=coordinates.at(-1);
    if(length<1e-8)return points;
    const start=points[0].clone(),end=points.at(-1).clone();
    const fit=(point,anchor)=>{
        wallFit.fit(point,anchor,.7);
        if(oppositeBranch) {
            const {path,radius}=oppositeBranch,nearest=path.nearest(point);
            const tangent=path.sample(Math.min(path.length,nearest.s+.5)).sub(path.sample(Math.max(0,nearest.s-.5))).normalize();
            // Beyond an open end there is no branch wall to push against.
            const delta=point.clone().sub(nearest.point),axial=delta.dot(tangent);
            if((nearest.s<=1e-6&&axial<0)||(nearest.s>=path.length-1e-6&&axial>0))return point;
            delta.addScaledVector(tangent,-axial);
            if(delta.length()>1e-8&&delta.length()<radius) {
                point.copy(nearest.point).addScaledVector(tangent,axial).addScaledVector(delta,radius/delta.length());
                wallFit.fit(point,anchor,.7);
            }
        }
        return point;
    };
    for(let i=1;i<points.length-1;i++) {
        const straight=start.clone().lerp(end,coordinates[i]/length);
        points[i].copy(fit(straight,points[i]));
    }
    for(let pass=0;pass<240;pass++) {
        let movement=0;
        for(let i=1;i<points.length-1;i++) {
            const span=coordinates[i+1]-coordinates[i-1];
            const fraction=span>1e-8?(coordinates[i]-coordinates[i-1])/span:.5;
            const next=points[i-1].clone().lerp(points[i+1],fraction);
            fit(next,points[i]);
            movement=Math.max(movement,next.distanceToSquared(points[i]));
            points[i].copy(next);
        }
        if(movement<1e-8)break;
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
    const restRadius=(i,j)=>part.restRadii?.[i*sides+j]??part.rowRadii?.[i]??radius;
    for(let i=0;i<rows;i++)for(let j=0;j<sides;j++) {
        const a=j/sides*Math.PI*2,dir=frames[i].u.clone().multiplyScalar(Math.cos(a)).addScaledVector(frames[i].v,Math.sin(a));
        const p=wallFit.fit(points[i].clone().addScaledVector(dir,restRadius(i,j)),points[i],.7);
        const k=i*sides+j;directions.push(dir);caps[k]=Math.max(0,Math.min(restRadius(i,j),p.clone().sub(points[i]).dot(dir)))/(part.restRadii?restRadius(i,j):1);
    }
    // For an oval, smooth expansion fractions rather than raw radii;
    // otherwise circumferential diffusion rounds and shrinks the rest shape.
    let radii=caps.slice(),next=caps.slice();
    for(let pass=0;pass<64;pass++) {
        for(let i=0;i<rows;i++)for(let j=0;j<sides;j++) {
            const k=i*sides+j,left=i*sides+(j+sides-1)%sides,right=i*sides+(j+1)%sides;
            // Nominal expansion pressure competes with curvature of the fabric.
            let sum=.15*(part.restRadii?1:restRadius(i,j))+2*(radii[left]+radii[right]),weight=4.15;
            if(i){sum+=3*radii[k-sides];weight+=3;}
            if(i+1<rows){sum+=3*radii[k+sides];weight+=3;}
            next[k]=Math.min(caps[k],sum/weight);
        }
        [radii,next]=[next,radii];
    }
    for(let k=0;k<count;k++) {
        const center=points[Math.floor(k/sides)];
        wallFit.fit(center.clone().addScaledVector(directions[k],radii[k]*(part.restRadii?restRadius(Math.floor(k/sides),k%sides):1)),center,.7).toArray(target,k*3);
    }
    part.ringFrames=frames;
}
