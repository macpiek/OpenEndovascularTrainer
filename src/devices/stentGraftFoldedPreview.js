import {disposeWire} from './stentGraftWire.js';
import {bodyDimensions} from './stentGraftModels.js';
import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';
import {createScaffold,updateScaffold} from './stentGraftScaffold.js';

// A packed delivery preview only. It creates no implant, contacts or flow seal;
// deployment replaces it with the anatomy-fitted parts in a single transaction.
export function createFoldedGraftPreview(device,{fabric,metal,fabricMaterial,metalMaterial,markerMaterial}) {
    const model=bodyDimensions(device);
    const layouts=device.type==='body'
        ?[{offset:0,length:model.trunkLength,radius:1.7,lateral:0},{offset:model.trunkLength,length:model.ipsiLength,radius:.7,lateral:-.8},
          {offset:model.trunkLength,length:model.contraLength,radius:.7,lateral:.8},{offset:-12,length:12,radius:1.2,lateral:0,crown:true}]
        :[{offset:0,length:device.length,radius:1.7,lateral:0}];
    return layouts.map(layout=>{
        const rows=Math.ceil(layout.length/2)+1,sides=24,coordinates=Array.from({length:rows},(_,i)=>layout.length*i/(rows-1));
        const points=coordinates.map(s=>new THREE.Vector3(0,s,0)),positions=new Float32Array(rows*sides*3),indices=[];
        for(let i=0;i<rows-1;i++)for(let j=0;j<sides;j++){
            const a=i*sides+j,b=i*sides+(j+1)%sides,c=a+sides,d=b+sides;indices.push(a,c,b,b,c,d);
        }
        const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setIndex(indices);
        const mesh=new THREE.Mesh(geometry,fabricMaterial);mesh.frustumCulled=false;mesh.name='folded-graft-fabric';mesh.visible=!layout.crown;fabric.add(mesh);
        const part={...layout,rows,sides,points,path:new DevicePath(points,coordinates),mesh};
        for(const object of createScaffold(part,metalMaterial,markerMaterial))metal.add(object);
        if(layout.crown)part.markers.visible=false;
        return part;
    });
}
export function updateFoldedGraftPreview(parts,wire,position,rotation=0) {
    const first=wire.points[0],last=wire.points.at(-1);
    const inlet=wire.points[1].clone().sub(first).normalize(),outlet=last.clone().sub(wire.points.at(-2)).normalize();
    const sample=s=>s<wire.coordinates[0]?first.clone().addScaledVector(inlet,s-wire.coordinates[0]):
        s>wire.length?last.clone().addScaledVector(outlet,s-wire.length):wire.sample(s);
    for(const part of parts){
        const positions=part.mesh.geometry.attributes.position;
        let previous=null,u=null;
        for(let i=0;i<part.rows;i++){
            const s=position-part.offset-part.path.coordinates[i],center=sample(s),tangent=sample(s+.5).sub(sample(s-.5)).normalize();
            if(tangent.lengthSq()<1e-10)tangent.set(0,1,0);
            if(u)u.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(previous,tangent));
            else u=new THREE.Vector3(Math.abs(tangent.z)<.9?0:1,0,Math.abs(tangent.z)<.9?1:0).cross(tangent).normalize();
            const rolled=u.clone().applyAxisAngle(tangent,rotation),v=tangent.clone().cross(rolled).normalize();center.addScaledVector(rolled,part.lateral);
            for(let j=0;j<part.sides;j++){
                const angle=j/part.sides*Math.PI*2,q=center.clone().addScaledVector(rolled,part.radius*Math.cos(angle)).addScaledVector(v,part.radius*Math.sin(angle));
                positions.setXYZ(i*part.sides+j,q.x,q.y,q.z);
            }
            previous=tangent;
        }
        positions.needsUpdate=true;updateScaffold(part);
    }
}
export function disposeFoldedGraftPreview(device) {
    for(const part of device.foldedPreview??[])for(const object of [part.mesh,part.rings,part.markers]){
        object.removeFromParent();object.geometry.dispose();disposeWire(object);if(object.isInstancedMesh)object.dispose();
    }
    device.foldedPreview=null;
}
export function setFoldedPreviewVisible(parts,visible) {
    for(const part of parts??[]){part.mesh.visible=visible&&!part.crown;part.rings.visible=visible;part.markers.visible=visible&&!part.crown;}
}
