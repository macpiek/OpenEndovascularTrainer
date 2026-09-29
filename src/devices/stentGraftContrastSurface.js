import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';
import {StentGraftSurface} from './stentGraftSurface.js';

// A snapshot of the actual exposed cloth, not its fully expanded target.
// Covered rows do not displace native flow or create a fictitious distal portal.
function exposedPart(part,complete) {
    let rows=complete?part.rows:part.exposure.findIndex(value=>value<=0);
    if(rows<0)rows=part.rows;
    if(rows<2)return null;
    const positions=(complete?part.target:part.contactBasePositions??part.mesh.geometry.attributes.position.array).slice(0,rows*part.sides*3);
    const points=Array.from({length:rows},(_,row)=>{
        const center=new THREE.Vector3();
        for(let j=0;j<part.sides;j++)center.add(new THREE.Vector3().fromArray(positions,(row*part.sides+j)*3));
        return center.divideScalar(part.sides);
    });
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
    geometry.setIndex(Array.from(part.mesh.geometry.index.array.slice(0,(rows-1)*part.sides*6)));
    return {...part,rows,points,target:positions,path:new DevicePath(points),mesh:{geometry},sourcePart:part,fullyExposed:rows===part.rows};
}

export function createContrastReleaseSurface(implants,revision) {
    const devices=[],ports=[],gateParts=[];
    const body=implants.find(d=>d.type==='body');
    const connected=implants.find(d=>d.id===body?.connectedLimbId&&d.phase==='deployed');
    for(const device of implants) {
        const parts=device.parts.map(p=>exposedPart(p,device.phase==='deployed'));
        if(!parts[0])continue;
        devices.push({...device,parts:parts.filter(Boolean)});
        if(device.type==='body') {
            ports.push([parts[0],0]);
            const branches=parts.slice(1).filter(Boolean);
            if(!parts[0].fullyExposed||!branches.length) {
                ports.push([parts[0],-1]);gateParts.push(parts[0]);
            } else for(let i=1;i<parts.length;i++)if(parts[i]) {
                if(i===2&&connected)continue;
                ports.push([parts[i],-1]);
                if(i===2||!parts[i].fullyExposed)gateParts.push(parts[i]);
            }
        } else {
            ports.push([parts[0],-1]);
            if(device!==connected)ports.push([parts[0],0]);
            if(!parts[0].fullyExposed)gateParts.push(parts[0]);
        }
    }
    if(!devices.length)return {revision,empty:true,parts:[],paths:[],sealed:false,sectionAt:()=>null,dispose(){}};
    const surface=new StentGraftSurface(devices,revision,{ports,gateParts,
        sealed:!!(body?.phase==='deployed'&&connected)});
    surface.partial=true;
    const dispose=surface.dispose.bind(surface);
    surface.dispose=()=>{dispose();for(const part of surface.parts)part.mesh.geometry.dispose();};
    return surface;
}
