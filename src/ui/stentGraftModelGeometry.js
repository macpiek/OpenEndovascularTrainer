import * as THREE from 'three';
import {bodyDimensions,nominalPartRadius} from '../devices/stentGraftModels.js';
import {buildGraftPart} from '../devices/stentGraftPart.js';
import {fitIIsTrunkSection,fitGraftJunction} from '../devices/stentGraftJunction.js';
import {createScaffold,updateScaffold,createGateMarker,createOrientationMarker,fabricPoint} from '../devices/stentGraftScaffold.js';
import {graftMarkerLayout} from '../devices/stentGraftMarkers.js';
import {disposeWire} from '../devices/stentGraftWire.js';
import {crownMaterial,inextensibleCrown,CROWN_SUBDIVISIONS} from '../devices/stentGraftCrownKinematics.js';

// Nominal, unloaded implant in millimetres. No patient anatomy, contacts or
// delivery pose: the same surface, sewn scaffold and markers as the simulator.
export function catalogueGeometry(device) {
    const wallFit={fit:p=>p},material=new THREE.MeshBasicMaterial();
    const parts=[];
    const axis=(x,start,length)=>{const rows=Math.ceil(length/2)+1;return Array.from({length:rows},(_,i)=>new THREE.Vector3(x,start+length*i/(rows-1),0));};
    const part=(points,radius,options={})=>buildGraftPart({points,radius,wallFit,fabricMaterial:material,...options});
    if(device.type==='body') {
        const d=bodyDimensions(device);
        parts.push(part(axis(0,0,d.trunkLength),device.diameter/2,{endRadius:d.crotchDiameter/2}));
        parts.push(part(axis(-d.gateDiameter/2,d.trunkLength,d.ipsiLength),device.distalDiameter/2,
            {endRadius:d.ipsiRootDiameter/2,distalStraight:d.distalStraight}));
        parts.push(part(axis(d.ipsiRootDiameter/2,d.trunkLength,d.contraLength),d.gateDiameter/2));
        if(device.modelId==='iis-103')fitIIsTrunkSection(parts,wallFit,d.gateDiameter);
        fitGraftJunction(parts,wallFit);
    } else parts.push(part(axis(0,0,device.length),device.diameter/2,{radiusProfile:t=>nominalPartRadius(device,0,t*device.length)}));
    const wires=[],markers=[],silhouettes=[];
    const segments=mesh=>{
        const a=mesh.geometry.attributes.instanceStart,b=mesh.geometry.attributes.instanceEnd;
        return Array.from({length:mesh.count},(_,i)=>[new THREE.Vector3().fromBufferAttribute(a,i),new THREE.Vector3().fromBufferAttribute(b,i)]);
    };
    for(const [index,p] of parts.entries()) {
        p.dimensionScale=1;p.nominalRadius=s=>nominalPartRadius(device,index,s);
        p.scaffoldInset=device.type==='body'&&index>0?.25:0;
        p.markerLayout=graftMarkerLayout(device,index,p.path.length);
        p.mesh.geometry.attributes.position.array.set(p.target);
        const objects=createScaffold(p,material,material);
        if(device.type==='body'&&index===0)objects.push(createOrientationMarker(p,material));
        if(device.type==='body'&&index===2)objects.push(createGateMarker(p,material));
        updateScaffold(p);wires.push(...segments(p.rings));
        for(const m of [p.markers,p.orientationMarker,p.gateMarker].filter(Boolean))markers.push(...segments(m));
        const left=[],right=[];
        for(let row=0;row<p.rows;row++) {
            const points=Array.from({length:p.sides},(_,j)=>new THREE.Vector3().fromArray(p.target,(row*p.sides+j)*3));
            left.push(points.reduce((a,b)=>a.x<b.x?a:b));right.push(points.reduce((a,b)=>a.x>b.x?a:b));
        }
        silhouettes.push([...left,...right.reverse()]);
        for(const o of objects){o.geometry.dispose();disposeWire(o);}
    }
    if(device.type==='body') {
        const p=parts[0],cm=crownMaterial(device.diameter);
        for(let i=0;i<12;i++) {
            const roots=[i,i+1].map(j=>fabricPoint(p,0,j/12*2*Math.PI));
            const peak=fabricPoint(p,0,(i+.5)/12*2*Math.PI);peak.y-=12;
            const targets=roots.map(root=>Array.from({length:CROWN_SUBDIVISIONS+1},(_,j)=>root.clone().lerp(peak,j/CROWN_SUBDIVISIONS)));
            const fitted=inextensibleCrown(...targets,cm);
            for(const line of [fitted.left,fitted.right])for(let j=1;j<line.length;j++)wires.push([line[j-1],line[j]]);
            const tip=fitted.left.at(-1);wires.push([tip,tip.clone().addScaledVector(fitted.left.at(-2).clone().sub(tip).normalize(),cm.hookLength)]);
        }
    }
    for(const p of parts)p.mesh.geometry.dispose();material.dispose();
    return {silhouettes,wires,markers};
}
