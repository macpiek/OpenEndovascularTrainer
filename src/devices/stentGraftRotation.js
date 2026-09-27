import {graftScale} from './stentGraftModels.js';
import {fitGraftJunction} from './stentGraftJunction.js';
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
    // A delivery motion changes the pose of the assembled implant. Mapping
    // every section along a polyline that turns into the ipsilateral branch
    // folded the body at that artificial elbow when translating it a few mm.
    // Transport one frame at the proximal attachment; then fit the moved
    // implant to the vessel, retaining its material shape and sewn junction.
    const origin=sample(0),destination=sample(-shift);
    const from=tangent(0),to=tangent(-shift);
    const transport=new THREE.Quaternion().setFromUnitVectors(from,to);
    const rotate=v=>v.clone().applyAxisAngle(from,angle).applyQuaternion(transport);
    const transform=point=>rotate(point.clone().sub(origin)).add(destination);
    const pose=new THREE.Matrix4().compose(destination.clone().sub(rotate(origin)),
        new THREE.Quaternion().setFromAxisAngle(from,angle).premultiply(transport),new THREE.Vector3(1,1,1));
    const delta=pose.clone().multiply((device.releasePoseMatrix??new THREE.Matrix4()).clone().invert());
    const deltaRotation=new THREE.Quaternion().setFromRotationMatrix(delta);
    device.releasePoseMatrix=pose;
    for(const part of device.parts) {
        // Commanded translation/roll moves released cloth as one assembly
        // while captured. Rod bending alone cannot rewrite its release pose.
        part.folded?.forEach(p=>p.applyMatrix4(delta));
        part.foldedFrames?.forEach(f=>{for(const key of ['u','v','tangent'])f[key].applyQuaternion(deltaRotation);});
        // All sections share the same rigid transport before wall fitting.
        part.points=part.referencePoints.map(p=>device.wallFit.fit(transform(p),p,.7));
        const frames=part.referenceFrames.map(frame=>({
            u:rotate(frame.u),v:rotate(frame.v),tangent:rotate(frame.tangent)
        }));
        fitExpandedGraft(part,device.wallFit,frames);
        part.path=new DevicePath(part.points,part.path.coordinates);
        part.releasePoseDirty=true;
    }
    if(device.type==='body')fitGraftJunction(device.parts,device.wallFit);
    if(device.referenceCrown)device.crownPath=new DevicePath(device.referenceCrown.map(p=>transform(p)));
    if(device.gate) {
        const path=device.parts[2].path;
        device.gate.entry=path.sample(path.length);
        device.gate.docking=path.sample(Math.max(0,path.length-10*graftScale(device)));
    }
    device.crownOpening=null;device.expandedCrown=null;
    device.graftRotation=angle;device.implantPosition=position;
    device.poseRevision=(device.poseRevision??0)+1;
}
