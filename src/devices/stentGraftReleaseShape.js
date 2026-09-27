import * as THREE from 'three';
import {graftRingFrames} from './stentGraftExpansion.js';

export function packedLayout(type,index,scale=1,side='right') {
    // The delivery lumen is in the long limb, not between two folded limbs.
    if(type==='body'&&index===1)return {radius:1.1*scale,lateral:0};
    if(type==='body'&&index===2)return {radius:.7*scale,lateral:(side==='left'?-1:1)*2.2*scale};
    return {radius:1.7*scale,lateral:0};
}
export function packedFrames(points,rotation=0) {
    return graftRingFrames(points).map(frame=>{
        frame.u.applyAxisAngle(frame.tangent,-rotation);
        frame.v.copy(frame.tangent).cross(frame.u).normalize();
        return frame;
    });
}
const orientation=frame=>new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(frame.u,frame.v,frame.tangent));

/** Interpolate section pose and radius separately. Linear interpolation of
 * opposite vertices collapsed and twisted rings when their frames differed.
 * The entire circumference now shares one rotation and opening coordinate. */
export function releaseSection(part,row,opening,frame,layout) {
    const start=part.folded[row].clone().addScaledVector(frame.u,layout.lateral);
    const end=part.points[row],targetFrame=part.ringFrames[row];
    const from=orientation(frame),to=orientation(targetFrame),inverse=to.clone().invert();
    const rotation=from.slerp(to,opening),center=start.lerp(end,opening);
    return {center,point(j){
        const target=new THREE.Vector3().fromArray(part.target,(row*part.sides+j)*3);
        if(opening===1)return target;
        const angle=j/part.sides*2*Math.PI;
        const radial=new THREE.Vector3(layout.radius*Math.cos(angle),layout.radius*Math.sin(angle),0);
        radial.lerp(target.sub(end).applyQuaternion(inverse),opening);
        return radial.applyQuaternion(rotation).add(center);
    }};
}

// The fabric's longitudinal material coordinates do not stretch when adjacent
// rings have different opening fractions. Project the preferred section poses
// onto those fixed link lengths; covered rows remain on the delivery shaft.
export function constrainReleaseCenters(part,sections,openings) {
    if(openings.every(value=>value===1)||openings.every(value=>value===0))return sections;
    const centers=sections.map(section=>section.center.clone());
    const weights=openings.map(value=>value*value);
    if(part.releaseOffset===0)weights[0]=0;
    for(let pass=0;pass<80;pass++) {
        let error=0;
        for(let j=0;j<centers.length-1;j++) {
            const i=pass%2?centers.length-2-j:j,next=i+1;
            const w=weights[i]+weights[next];if(!w)continue;
            const delta=centers[next].clone().sub(centers[i]),length=delta.length();
            const rest=Math.max(part.path.coordinates[next]-part.path.coordinates[i],part.points[next].distanceTo(part.points[i]),part.folded[next].distanceTo(part.folded[i]));
            if(length<1e-10)continue;
            const correction=Math.max(0,length-rest);error=Math.max(error,correction);
            delta.multiplyScalar(correction/(length*w));
            centers[i].addScaledVector(delta,weights[i]);centers[next].addScaledVector(delta,-weights[next]);
        }
        if(error<1e-5)break;
    }
    return sections.map((section,i)=>{
        const shift=centers[i].clone().sub(section.center);
        return {center:centers[i],point:j=>section.point(j).add(shift)};
    });
}


// Both branches share a sewn septum. Their independent radial springs may
// approach it, but cannot expand through the neighbouring branch.
export function separateReleaseBranches(device) {
    if(device.type!=='body')return;
    const parts=device.parts.slice(1);
    if(!parts.some(part=>part.releaseShapeChanged))return;
    if(parts.every(part=>Array.from(part.exposure).every(value=>value===1))) {
        for(const part of parts) {
            const positions=part.mesh.geometry.attributes.position;
            if(positions.array.some((v,i)=>v!==part.target[i])) {positions.array.set(part.target);positions.needsUpdate=true;}
            if(part.contactBasePositions)part.contactBasePositions.set(part.target);
        }
        return;
    }
    const key=parts.map(part=>part.mesh.geometry.attributes.position.version).join('/');
    if(device.septumPoseKey===key)return;
    for(const part of parts)if(part.contactBasePositions) {
        const positions=part.mesh.geometry.attributes.position;
        if(positions.array.some((v,i)=>v!==part.contactBasePositions[i])) {
            positions.array.set(part.contactBasePositions);positions.needsUpdate=true;
        }
    }
    const centers=parts.map(part=>part.path.coordinates.map((_,row)=>{
        const c=new THREE.Vector3(),p=part.mesh.geometry.attributes.position;
        for(let j=0;j<part.sides;j++)c.add(new THREE.Vector3().fromBufferAttribute(p,row*part.sides+j));
        return c.divideScalar(part.sides);
    }));
    const tangent=centers[0].at(-1).clone().sub(centers[0][0]).normalize();
    const normal=parts[1].points[0].clone().sub(parts[0].points[0]);
    normal.addScaledVector(tangent,-normal.dot(tangent)).normalize();
    const paths=centers.map(points=>points.map(p=>({s:p.dot(tangent),x:p.dot(normal)})).sort((a,b)=>a.s-b.s));
    const low=Math.max(...paths.map(p=>p[0].s)),high=Math.min(...paths.map(p=>p.at(-1).s));
    const lateral=(path,s)=>{
        let i=1;while(i<path.length-1&&path[i].s<s)i++;
        return THREE.MathUtils.lerp(path[i-1].x,path[i].x,THREE.MathUtils.clamp((s-path[i-1].s)/Math.max(1e-9,path[i].s-path[i-1].s),0,1));
    };
    for(let k=0;k<2;k++) {
        const part=parts[k],positions=part.mesh.geometry.attributes.position,sign=k===0?-1:1;
        let changed=false;
        for(let index=0;index<positions.count;index++) {
            const p=new THREE.Vector3().fromBufferAttribute(positions,index),axial=p.dot(tangent);
            if(axial<low||axial>high)continue;
            const mid=(lateral(paths[0],axial)+lateral(paths[1],axial))/2;
            const distance=(p.dot(normal)-mid)*sign;
            if(distance<0){p.addScaledVector(normal,-distance*sign);positions.setXYZ(index,p.x,p.y,p.z);changed=true;}
        }
        if(changed)positions.needsUpdate=true;
        if(part.contactBasePositions)part.contactBasePositions.set(positions.array);
    }
    device.septumPoseKey=parts.map(part=>part.mesh.geometry.attributes.position.version).join('/');
}
