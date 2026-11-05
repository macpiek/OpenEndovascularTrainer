import * as THREE from 'three';
import {DEFAULT_VESSEL_ELASTICITY} from '../physics/vesselElasticity.js';
import {getCompositeJointRenderPath} from '../compositeJointRenderPath.js';

import {RELIANT,balloonFreeRadiusAt} from './balloonProfile.js';
export {RELIANT,RELIANT_INFLATION_TABLE,balloonFreeDiameter} from './balloonProfile.js';

export function balloonInflationBlock({catheter,sheathInsertion,sheathFrench,dilatorInsertion=0,hasWall=false}) {
    if(catheter.type!==RELIANT.type)return 'Wybierz balon Reliant.';
    if(sheathFrench<RELIANT.minSheathFrench)return 'Reliant wymaga koszulki co najmniej 12F.';
    if(dilatorInsertion>0)return 'Wycofaj rozszerzacz.';
    if(catheter.progress-sheathInsertion<RELIANT.sectionLength+RELIANT.distalOffset+2)return 'Wysuń cały balon poza koszulkę.';
    if(!hasWall)return 'Oczekiwanie na model naczynia.';
    return '';
}
export function balloonMotionLocked(catheter,target=0) {
    return catheter.type===RELIANT.type&&((catheter.balloonDriveMl??0)>1e-6||target>0);
}
export function advanceBalloonVolume(current,target,dt) {
    if(![current,target,dt].every(Number.isFinite)||dt<0)throw new RangeError('Invalid balloon input');
    target=Math.max(0,Math.min(RELIANT.maxVolumeMl,target));
    // Deflation is faster than inflation, but still advances only on simulation ticks.
    const step=RELIANT.fillRateMlS*(target<current?2:1)*dt;
    return Math.max(0,Math.min(60,current+Math.max(-step,Math.min(step,target-current))));
}

/** Render the accepted mechanical membrane: its nested free envelope is clipped by
 * first radial wall intersections, never by a neighbouring centerline radius.
 * the accepted deformed wall. Contact forces and material tension are solved
 * separately by the shared-axis equilibrium before this geometry is published.
 */
export function sampleCompliantBalloon({path,coordinates,volumeMl,wallDistance,axialSegments=80,radialSegments=32}) {
    const shaft=RELIANT.shaftRadius;
    const polygonFactor=radialSegments*Math.sin(2*Math.PI/radialSegments)/2;
    const length=RELIANT.sectionLength;
    const first=coordinates[0],end=coordinates.at(-1),span=end-first;
    if(!(span>RELIANT.sectionLength+RELIANT.distalOffset)||!path)return null;
    const centerCoordinate=end-RELIANT.distalOffset-RELIANT.sectionLength/2;
    const points=new Float32Array((axialSegments+1)*radialSegments*3),indices=[],areas=[],centers=new Float32Array((axialSegments+1)*3);
    const center=new THREE.Vector3(),tangent=new THREE.Vector3(),normal=new THREE.Vector3(),binormal=new THREE.Vector3(),direction=new THREE.Vector3();
    let maxDiameter=0,limited=false,invalid=false,previousNormal=null;
    for(let i=0;i<=axialSegments;i++) {
        const t=i/axialSegments,u=(centerCoordinate+(t-.5)*length-first)/span;
        if(u<0||u>1)return null;
        path.getPointAt(u,center);path.getTangentAt(u,tangent);center.toArray(centers,i*3);
        if(previousNormal)normal.copy(previousNormal).addScaledVector(tangent,-previousNormal.dot(tangent));
        else normal.set(Math.abs(tangent.z)<.9?0:1,0,Math.abs(tangent.z)<.9?1:0).addScaledVector(tangent,-(Math.abs(tangent.z)<.9?tangent.z:tangent.x));
        normal.normalize();binormal.crossVectors(tangent,normal).normalize();previousNormal=normal.clone();
        const free=balloonFreeRadiusAt(RELIANT.distalOffset+(1-t)*length,volumeMl);
        const radii=[];
        for(let j=0;j<radialSegments;j++) {
            const angle=j*2*Math.PI/radialSegments+.173;
            direction.copy(normal).multiplyScalar(Math.cos(angle)).addScaledVector(binormal,Math.sin(angle));
            const clearance=wallDistance(center,direction,free);
            if(!Number.isFinite(clearance)||clearance<shaft-1e-3)invalid=true;
            const radius=Math.max(shaft,Math.min(free,Math.max(0,clearance-.03)));
            radii.push(radius);
            limited ||= radius<free-1e-4;
            center.clone().addScaledVector(direction,radius).toArray(points,(i*radialSegments+j)*3);
            if(i<axialSegments){const a=i*radialSegments+j,b=i*radialSegments+(j+1)%radialSegments,c=a+radialSegments,d=b+radialSegments;indices.push(a,b,c,b,d,c);}
        }
        for(let j=0;j<radialSegments/2;j++)maxDiameter=Math.max(maxDiameter,radii[j]+radii[j+radialSegments/2]);
        let area=0;for(let j=0;j<radialSegments;j++)area+=radii[j]*radii[(j+1)%radialSegments]*Math.sin(2*Math.PI/radialSegments)/2;
        areas.push(Math.max(0,area-polygonFactor*shaft*shaft));
    }
    let contained=0;for(let i=1;i<areas.length;i++)contained+=(areas[i-1]+areas[i])*.5*length/axialSegments/1000;
    contained*=Math.PI/polygonFactor; // radial polygon quadrature correction
    return {points,indices,centers,radialSegments,volumeMl:invalid?0:Math.min(volumeMl,contained),maxDiameter,limited,invalid,length};
}

export function sampleBalloonOnCatheter(catheter,tree,volumeMl,deformation=null) {
    const view=catheter.physicsBody?.jointStateView,path=getCompositeJointRenderPath(view);
    const coordinates=view?.continuousCurve?.coordinates??view?.coordinates;
    if(!tree||!path||!coordinates)return null;
    const ray=new THREE.Ray(),displacement=new THREE.Vector3(),nearest={point:new THREE.Vector3()};
    let ringX=NaN,ringY=NaN,ringZ=NaN,ringRadius=NaN,ringClear=false;
    return sampleCompliantBalloon({path,coordinates,volumeMl,wallDistance:(center,direction,freeRadius)=>{
        ray.origin.copy(center);ray.direction.copy(direction);
        // Geometry beyond the free membrane cannot limit this ray. A bounded
        // miss certifies that its entire membrane segment is wall-free; it is
        // not a missing anatomical query. Retain the same .03 mm clearance.
        const cutoff=freeRadius+.03;
        if(center.x!==ringX||center.y!==ringY||center.z!==ringZ||freeRadius!==ringRadius) {
            ringX=center.x;ringY=center.y;ringZ=center.z;ringRadius=freeRadius;
            // One exact empty-ball query certifies every direction of a free
            // ring. Loaded rings still test all 32 individual wall rays.
            const closest=tree.closestPointToPoint(center,nearest,0,cutoff);
            ringClear=!closest||closest.distance>=cutoff;
        }
        if(ringClear)return cutoff;
        const hit=tree.raycastFirst(ray,THREE.DoubleSide,0,cutoff);
        if(!hit)return cutoff;
        if(!deformation)return hit.distance;
        deformation.displacementAt(hit.point,displacement);
        const clearance=hit.distance+Math.max(0,displacement.dot(direction));
        // The accepted elastic rod can occupy a shallow indentation not fully
        // recovered by the smoothed display field. Keep its certified shaft
        // envelope there; never allow extra balloon expansion into that gap.
        return displacement.lengthSq()>1e-12&&clearance>=RELIANT.shaftRadius-DEFAULT_VESSEL_ELASTICITY.maxDisplacement-1e-3
            ?Math.max(RELIANT.shaftRadius,clearance):clearance;
    }});
}
export function updateBalloonGeometry(geometry,sample) {
    let attribute=geometry.getAttribute('position');
    if(!attribute||attribute.count*3!==sample.points.length){attribute=new THREE.BufferAttribute(sample.points,3);geometry.setAttribute('position',attribute);geometry.setIndex(sample.indices);}
    else {attribute.array.set(sample.points);attribute.needsUpdate=true;}
    geometry.computeVertexNormals();geometry.computeBoundingSphere();return geometry;
}

/** Remove fluid from the accepted membrane rather than rebuilding a larger
 * unconstrained shape during deflation. Cross-sectional annular areas scale
 * with volume; every vertex moves toward the shaft, never through the wall. */
export function deflateBalloonSample(sample,volumeMl) {
    const ratio=Math.max(0,Math.min(1,volumeMl/sample.volumeMl)),points=sample.points.slice(),r2=RELIANT.shaftRadius**2;
    let maxDiameter=0;
    for(let i=0;i<points.length;i+=3) {
        const c=Math.floor(i/3/sample.radialSegments)*3;
        const dx=points[i]-sample.centers[c],dy=points[i+1]-sample.centers[c+1],dz=points[i+2]-sample.centers[c+2],oldR2=dx*dx+dy*dy+dz*dz;
        const scale=Math.sqrt((r2+Math.max(0,oldR2-r2)*ratio)/Math.max(r2,oldR2));
        points[i]=sample.centers[c]+dx*scale;points[i+1]=sample.centers[c+1]+dy*scale;points[i+2]=sample.centers[c+2]+dz*scale;
    }
    for(let i=0;i<points.length;i+=sample.radialSegments*3)for(let j=0;j<sample.radialSegments/2;j++) {
        const a=i+j*3,b=a+sample.radialSegments/2*3;
        maxDiameter=Math.max(maxDiameter,Math.hypot(points[a]-points[b],points[a+1]-points[b+1],points[a+2]-points[b+2]));
    }
    return {...sample,points,volumeMl:sample.volumeMl*ratio,maxDiameter};
}

export function stepCompliantBalloon(state,target,dt,sampleAtVolume) {
    if(!Number.isFinite(target)||!Number.isFinite(dt)||dt<0)throw new RangeError('Invalid balloon step');
    target=Math.max(0,Math.min(60,target));
    if(!dt)return state;
    const actual=state.sample?.volumeMl??0;
    if(target<actual) {
        const volume=Math.max(target,actual-RELIANT.fillRateMlS*2*dt);
        state.driveMl=(state.driveMl??0)*(volume/actual);
        state.sample=volume>1e-6?deflateBalloonSample(state.sample,volume):null;
        if(!state.sample)state.driveMl=0;
        return state;
    }
    if(!target){state.driveMl=0;state.sample=null;return state;}
    state.driveMl=advanceBalloonVolume(state.driveMl??0,Math.max(state.driveMl??0,target),dt);
    state.elapsed=(state.elapsed??0)+dt;
    if(state.elapsed<.1&&state.sample)return state;
    state.elapsed=0;
    const sample=sampleAtVolume(state.driveMl);
    if(!sample||sample.invalid){
        if(state.sample){state.sample={...state.sample,limited:true};return state;}
        state.sample=null;state.driveMl=0;state.invalid=true;return state;
    }
    state.invalid=false;
    // A changing axial free envelope must not make fluid disappear while
    // pumping. Refuse such a trial and retain the last admissible filled shape.
    if(sample.volumeMl+1e-7>=actual)state.sample=sample.volumeMl>target?deflateBalloonSample(sample,target):sample;
    else if(state.sample)state.sample={...state.sample,limited:true};
    return state;
}
