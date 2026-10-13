import * as THREE from 'three';
import {DevicePath} from './stentGraftPaths.js';
import {StentGraftSurface} from './stentGraftSurface.js';
import {acceptedPartPositions,connectedGraftLimb,openGraftBranches} from './stentGraftConnections.js';

// A snapshot of the actual exposed cloth, not its fully expanded target.
// Covered rows do not displace native flow or create a fictitious distal portal.
function exposedPart(part,complete) {
    let rows=complete?part.rows:part.exposure.findIndex(value=>value<=0);
    if(rows<0)rows=part.rows;
    if(rows<2)return null;
    const positions=acceptedPartPositions(part).slice(0,rows*part.sides*3);
    const points=Array.from({length:rows},(_,row)=>{
        const center=new THREE.Vector3();
        for(let j=0;j<part.sides;j++)center.add(new THREE.Vector3().fromArray(positions,(row*part.sides+j)*3));
        return center.divideScalar(part.sides);
    });
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
    geometry.setIndex(Array.from(part.mesh.geometry.index.array.slice(0,(rows-1)*part.sides*6)));
    return {...part,rows,points,target:positions,path:new DevicePath(points),mesh:{geometry},sourcePart:part,fullyExposed:rows===part.rows};
}

// Trim the already sewn Y at the exposed rows. Reusing its topology avoids
// three Boolean unions per contrast refresh and retains the real divider.
function releaseSewnTopology(device,parts) {
    const source=device.sewnTopology;
    if(!source||!parts[0]?.fullyExposed||parts.some(p=>!p))return null;
    const map=new Map(device.parts.map((p,i)=>[p,parts[i]])),vertices=[],ids=new Map();
    const add=id=>{
        if(!ids.has(id)) {
            ids.set(id,vertices.length);const v=source.vertices[id];
            vertices.push({...v,part:map.get(v.part)});
        }
        return ids.get(id);
    };
    const valid=id=>{const v=source.vertices[id];return v.indices.every(i=>i<map.get(v.part).rows*v.part.sides);};
    const faces=source.faces.filter(f=>f.ids.every(valid)).map(f=>({...f,part:map.get(f.part),ids:f.ids.map(add)}));
    const single=new Map();source.vertices.forEach((v,id)=>{if(v.indices.length===1&&v.weights[0]===1)single.set(`${device.parts.indexOf(v.part)}:${v.indices[0]}`,id);});
    const ports=parts.map((part,k)=>({part,proximal:k===0,ids:Array.from({length:part.sides},(_,j)=>
        add(single.get(`${k}:${(k===0?0:part.rows-1)*part.sides+j}`)))}));
    return {parts,vertices,faces,ports};
}

export function createContrastReleaseSurface(implants,revision) {
    const devices=[],ports=[],gateParts=[];
    const body=implants.find(d=>d.type==='body');
    const connected=body?[1,2].map(i=>connectedGraftLimb(implants,body,i)).filter(Boolean):[];
    const openBranches=openGraftBranches(implants,body);
    for(const device of implants) {
        const parts=device.parts.map(p=>exposedPart(p,device.phase==='deployed'));
        if(!parts[0])continue;
        const topology=releaseSewnTopology(device,parts);
        devices.push({...device,parts:topology?.parts??parts.filter(Boolean),sewnTopology:topology});
        if(device.type==='body') {
            ports.push([parts[0],0]);
            const branches=parts.slice(1).filter(Boolean);
            if(!parts[0].fullyExposed||!branches.length) {
                ports.push([parts[0],-1]);gateParts.push(parts[0]);
            } else for(let i=1;i<parts.length;i++)if(parts[i]) {
                if(connectedGraftLimb(implants,body,i))continue;
                ports.push([parts[i],-1]);
                if(openBranches.includes(i)||!parts[i].fullyExposed)gateParts.push(parts[i]);
            }
        } else {
            ports.push([parts[0],-1]);
            if(!connected.includes(device))ports.push([parts[0],0]);
            if(!parts[0].fullyExposed)gateParts.push(parts[0]);
        }
    }
    if(!devices.length)return {revision,empty:true,parts:[],paths:[],sealed:false,sectionAt:()=>null,dispose(){}};
    const surface=new StentGraftSurface(devices,revision,{ports,gateParts,
        sealed:!!(body?.phase==='deployed'&&!openBranches.length)});
    surface.partial=true;
    const dispose=surface.dispose.bind(surface);
    surface.dispose=()=>{dispose();for(const part of surface.parts)part.mesh.geometry.dispose();};
    return surface;
}
