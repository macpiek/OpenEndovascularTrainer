import {BufferGeometry,Float32BufferAttribute,Mesh} from 'three';
import {sewnTopology} from './stentGraftSewnTopology.js';

export function sewnPositions(topology,target=false,base=false) {
    const positions=new Float32Array(topology.vertices.length*3);
    topology.vertices.forEach(({part,indices,weights},j)=>{
        const source=target?part.target:base?(part.contactBasePositions??part.mesh.geometry.attributes.position.array):part.mesh.geometry.attributes.position.array;
        for(let c=0;c<3;c++){let value=0;for(let i=0;i<indices.length;i++)value+=source[indices[i]*3+c]*weights[i];positions[j*3+c]=value;}
    });
    return positions;
}
export function sewnGeometry(device,{target=false,closed=false}={}) {
    const topology=sewnTopology(device),positions=Array.from(sewnPositions(topology,target)),indices=topology.faces.flatMap(f=>f.ids);
    if(closed)for(const port of topology.ports){
        const center=[0,0,0];for(const i of port.ids)for(let c=0;c<3;c++)center[c]+=positions[i*3+c]/port.ids.length;
        const id=positions.length/3;positions.push(...center);
        for(let j=0;j<port.ids.length;j++){const a=port.ids[j],b=port.ids[(j+1)%port.ids.length];indices.push(id,...(port.proximal?[a,b]:[b,a]));}
    }
    const geometry=new BufferGeometry();geometry.setAttribute('position',new Float32BufferAttribute(positions,3));geometry.setIndex(indices);return geometry;
}
export function updateSewnSurface(device,group,material) {
    if(device.type!=='body')return;
    if(!device.sewnMesh){device.sewnMesh=new Mesh(sewnGeometry(device),material);device.sewnMesh.name='continuous-sewn-graft';device.sewnMesh.frustumCulled=false;group.add(device.sewnMesh);device.parts.forEach(p=>p.mesh.visible=false);}
    const key=device.parts.map(p=>p.mesh.geometry.attributes.position.version).join('/');
    if(device.sewnMeshVersion===key)return;
    device.sewnMesh.geometry.attributes.position.array.set(sewnPositions(sewnTopology(device)));
    device.sewnMesh.geometry.attributes.position.needsUpdate=true;device.sewnMeshVersion=key;
}
export function sewnContactFaces(device) {
    const topology=sewnTopology(device),p=sewnPositions(topology,true);
    return topology.faces.map(({part,row,ids})=>({distance:part.releaseOffset+part.path.coordinates[row],gate:part===device.parts[2],ipsilateral:part===device.parts[1],
        positions:ids.flatMap(i=>Array.from(p.subarray(i*3,i*3+3))),bindings:ids.map(i=>topology.vertices[i])}));
}
