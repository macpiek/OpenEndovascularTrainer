import {fitExpandedGraft} from './stentGraftExpansion.js';
import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';

// Immutable reference pose: repeated rotations must never accumulate wall-fit
// shrinkage. All geometry, ports and contact faces share this same deformation.
export function captureGraftPose(device,axis) {
    device.rotationAxis=axis;device.referencePosition=device.implantPosition;
    for(const part of device.parts) {
        part.referencePoints=part.points.map(p=>p.clone());
        part.referenceFrames=part.ringFrames.map(f=>({u:f.u.clone(),v:f.v.clone(),tangent:f.tangent.clone()}));
        part.axisCoordinates=part.points.map(p=>axis.nearest(p).s);
    }
    device.referenceCrown=device.crownPath?.points.map(p=>p.clone());
}
export function rotateGraftPose(device,angle,position=device.implantPosition) {
    const axis=device.rotationAxis;
    if(!axis)return;
    const shift=position-device.referencePosition;
    const sample=s=>{
        if(s<0)return axis.points[0].clone().addScaledVector(axis.points[1].clone().sub(axis.points[0]).normalize(),s);
        if(s>axis.length)return axis.points.at(-1).clone().addScaledVector(axis.points.at(-1).clone().sub(axis.points.at(-2)).normalize(),s-axis.length);
        return axis.sample(s);
    };
    const tangent=s=>sample(s-.5).sub(sample(s+.5)).normalize();
    const transform=(point,s=axis.nearest(point).s)=>{
        const origin=sample(s);
        const from=tangent(s),to=tangent(s-shift);
        return point.clone().sub(origin).applyAxisAngle(from,angle)
            .applyQuaternion(new THREE.Quaternion().setFromUnitVectors(from,to)).add(sample(s-shift));
    };
    for(const part of device.parts) {
        // All vertices of a section use its material-axis coordinate. Finding
        // the nearest axis point separately for every vertex sheared rings at
        // bends during roll.
        part.points=part.referencePoints.map((p,i)=>device.wallFit.fit(transform(p,part.axisCoordinates[i]),p,.7));
        const frames=part.referenceFrames.map((frame,i)=>{
            const s=part.axisCoordinates[i],from=tangent(s),to=tangent(s-shift);
            const q=new THREE.Quaternion().setFromUnitVectors(from,to);
            const rotate=v=>v.clone().applyAxisAngle(from,angle).applyQuaternion(q);
            return {u:rotate(frame.u),v:rotate(frame.v),tangent:rotate(frame.tangent)};
        });
        fitExpandedGraft(part,device.wallFit,frames);
        part.path=new DevicePath(part.points,part.path.coordinates);
        part.exposure.fill(-1);
    }
    if(device.referenceCrown)device.crownPath=new DevicePath(device.referenceCrown.map(p=>transform(p)));
    if(device.gate) {
        const path=device.parts[2].path;
        device.gate.entry=path.sample(path.length);
        device.gate.docking=path.sample(Math.max(0,path.length-10));
        device.gate.marker.position.copy(device.gate.entry);
        device.gate.marker.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1),path.sample(path.length-2).sub(device.gate.entry).normalize());
    }
    device.crownOpening=null;device.expandedCrown=null;
    device.graftRotation=angle;device.implantPosition=position;
    device.poseRevision=(device.poseRevision??0)+1;
}
