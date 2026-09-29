import {graftMarkerLayout} from './stentGraftMarkers.js';
import {capturedRoot} from './stentGraftCapture.js';
import {CROWN_SUBDIVISIONS,crownMaterial,inextensibleCrown} from './stentGraftCrownKinematics.js';
import {wireSegment,updateWire} from './stentGraftWire.js';
import {packedFrames,packedLayout} from './stentGraftReleaseShape.js';
import {disposeWire} from './stentGraftWire.js';
import {worldBodyDimensions,graftScale,nominalPartRadius} from './stentGraftModels.js';
import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';
import {createScaffold,updateScaffold,createGateMarker,createOrientationMarker,createSuprarenalCrown,fabricPoint} from './stentGraftScaffold.js';

// A packed delivery preview only. It creates no implant, contacts or flow seal;
// deployment replaces it with the anatomy-fitted parts in a single transaction.
export function createFoldedGraftPreview(device,{fabric,metal,fabricMaterial,metalMaterial,markerMaterial}) {
    const model=worldBodyDimensions(device),scale=graftScale(device);
    const layouts=device.type==='body'
        ?[{offset:0,length:model.trunkLength,radius:1.7,lateral:0},{offset:model.trunkLength,length:model.ipsiLength,...packedLayout('body',1,1,device.side)},
          {offset:model.trunkLength,length:model.contraLength,...packedLayout('body',2,1,device.side),gate:true},{offset:-12*scale,length:12*scale,radius:1.2,lateral:0,crown:true}]
        :[{offset:0,length:device.length*scale,radius:1.7,lateral:0}];
    return layouts.map((layout,index)=>{
        layout.radius*=scale;layout.lateral*=scale;
        const rows=Math.ceil(layout.length/2)+1,sides=24,coordinates=Array.from({length:rows},(_,i)=>layout.length*i/(rows-1));
        const points=coordinates.map(s=>new THREE.Vector3(0,s,0)),positions=new Float32Array(rows*sides*3),indices=[];
        for(let i=0;i<rows-1;i++)for(let j=0;j<sides;j++){
            const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,d=b+sides;indices.push(a,c,b,b,c,d);
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,fabricMaterial);mesh.frustumCulled=false;mesh.name='folded-graft-fabric';mesh.visible=!layout.crown;fabric.add(mesh);
        const nominalRadius=s=>nominalPartRadius(device,index,s);
        const part={...layout,dimensionScale:scale,nominalRadius,rows,sides,points,path:new DevicePath(points,coordinates),mesh};
        part.markerLayout=layout.crown?[]:graftMarkerLayout(device,index,part.path.length);
        if(device.type==='body'&&index===0)metal.add(createOrientationMarker(part,markerMaterial));
        part.scaffoldInset=device.type==='body'&&index>0&&!layout.crown?.25*scale:0;
        for(const object of createScaffold(part,metalMaterial,markerMaterial))metal.add(object);
        if(layout.crown) {
            part.rings.removeFromParent();part.rings.geometry.dispose();disposeWire(part.rings);
            part.rings=createSuprarenalCrown(metalMaterial);metal.add(part.rings);
            part.scaffoldRings=[];part.crownMaterial=crownMaterial(device.diameter,scale);part.markers.visible=false;
        }
        if(layout.gate)metal.add(createGateMarker(part,markerMaterial));
        return part;
    });
}
export function updateFoldedGraftPreview(parts,wire,position,rotation=0) {
    const first=wire.points[0],last=wire.points.at(-1);
    const inlet=wire.points[1].clone().sub(first).normalize(),outlet=last.clone().sub(wire.points.at(-2)).normalize();
    const sample=s=>s<wire.coordinates[0]?first.clone().addScaledVector(inlet,s-wire.coordinates[0]):
        s>wire.length?last.clone().addScaledVector(outlet,s-wire.length):wire.sample(s);
    const crown=parts.find(part=>part.crown),latch=crown?sample(position+crown.length):null;
    for(const part of parts){
        const positions=part.mesh.geometry.attributes.position;
        const centers=part.path.coordinates.map(s=>sample(position-part.offset-s));
        const frames=packedFrames(centers,rotation);
        for(let i=0;i<part.rows;i++){
            const {u,v}=frames[i],center=centers[i].clone().addScaledVector(u,part.lateral);
            for(let j=0;j<part.sides;j++){
                const angle=j/part.sides*Math.PI*2,q=center.clone().addScaledVector(u,part.radius*Math.cos(angle)).addScaledVector(v,part.radius*Math.sin(angle));
                if(crown&&part===parts[0]) {
                    const root=centers[0].clone().addScaledVector(frames[0].u,part.radius*Math.cos(angle)).addScaledVector(frames[0].v,part.radius*Math.sin(angle));
                    const delta=capturedRoot(root,latch,crown.crownMaterial.armLength).sub(root);
                    const t=Math.max(0,1-part.path.coordinates[i]/(20*part.dimensionScale));
                    q.addScaledVector(delta,t*t*(3-2*t));
                }
                positions.setXYZ(i*part.sides+j,q.x,q.y,q.z);
            }
        }
        positions.needsUpdate=true;
        if(!part.crown)updateScaffold(part);
        else {
            const captured=sample(position+part.length);let index=0;part.crownPolylines=[];
            for(let i=0;i<12;i++) {
                const targets=[i,i+1].map(k=>{
                    const root=fabricPoint(parts[0],0,k/12*Math.PI*2);
                    return Array.from({length:CROWN_SUBDIVISIONS+1},(_,j)=>root.clone().lerp(captured,j/CROWN_SUBDIVISIONS));
                });
                const fitted=inextensibleCrown(...targets,part.crownMaterial,null,captured);
                for(const line of [fitted.left,fitted.right]) {
                    for(let j=1;j<line.length;j++)wireSegment(part.rings,index++,line[j-1],line[j]);
                    part.crownPolylines.push(line);
                }
                const peak=fitted.left.at(-1),hook=peak.clone().addScaledVector(fitted.left.at(-2).clone().sub(peak).normalize(),part.crownMaterial.hookLength);
                wireSegment(part.rings,index++,peak,hook);part.crownPolylines.push([peak,hook]);
            }
            updateWire(part.rings);
        }
    }
}
export function disposeFoldedGraftPreview(device) {
    for(const part of device.foldedPreview??[])for(const object of [part.mesh,part.rings,part.markers,part.gateMarker,part.orientationMarker].filter(Boolean)){
        object.removeFromParent();object.geometry.dispose();disposeWire(object);if(object.isInstancedMesh)object.dispose();
    }
    device.foldedPreview=null;
}
export function setFoldedPreviewVisible(parts,visible) {
    for(const part of parts??[]){part.mesh.visible=visible&&!part.crown;part.rings.visible=visible;part.markers.visible=visible&&!part.crown;if(part.gateMarker)part.gateMarker.visible=visible;if(part.orientationMarker)part.orientationMarker.visible=visible;}
}
