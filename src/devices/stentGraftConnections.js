import {Vector3} from 'three';
import {DevicePath} from './stentGraftPaths.js';

// The two IIs outlets are docking ports. The longer II ipsilateral branch can
// also accept an extension, but does not require one to complete the conduit.
export function connectedGraftLimb(implants,body,index) {
    return implants.find(d=>d.type==='limb'&&d.phase==='deployed'&&
        (d.parentId===body?.id&&(d.parentPartIndex??2)===index||
         index===2&&d.id===body?.connectedLimbId));
}
export function openGraftBranches(implants,body) {
    if(!body)return [];
    return [1,2].filter(index=>!connectedGraftLimb(implants,body,index)&&
        (index===2||body.modelId==='iis-103'));
}
export function acceptedPartPositions(part) {
    return part.contactBasePositions??part.mesh.geometry.attributes.position.array;
}
export function acceptedPartPath(part) {
    const a=acceptedPartPositions(part);
    return new DevicePath(Array.from({length:part.rows},(_,row)=>{
        const center=new Vector3();
        for(let j=0;j<part.sides;j++)center.add(new Vector3().fromArray(a,(row*part.sides+j)*3));
        return center.divideScalar(part.sides);
    }));
}
export function graftDockingPort(body,index,route) {
    const part=body.parts[index],path=acceptedPartPath(part),scale=body.dimensionScale??1;
    const docking=path.sample(Math.max(0,path.length-10*scale)),entry=path.sample(path.length);
    return {parent:body,part,index,path,docking,entry,side:index===1?body.side:body.side==='right'?'left':'right',
        routeS:route.nearest(docking).s,freePlacement:body.gate?.freePlacement??false};
}
