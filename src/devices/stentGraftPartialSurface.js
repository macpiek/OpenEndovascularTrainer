import {graftCoverWithdrawal} from './stentGraftDeployment.js';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {StentGraftSurface} from './stentGraftSurface.js';

/** Map the union's exterior to release distances once. Using the union removes
 * the internal membranes where the trunk and its two limbs overlap. The cover
 * carries folded/transitioning rows; fabric contacts start at fully open rows. */
export function preparePartialSurface(device) {
    const surface=new StentGraftSurface([device],0),p=surface.geometry.attributes.position,ix=surface.geometry.index;
    const parts=device.parts.map(part=>{
        const geometry=part.mesh.geometry.clone();
        geometry.setAttribute('position',new THREE.Float32BufferAttribute(part.target,3));
        geometry.boundsTree=new MeshBVH(geometry);
        return {part,geometry};
    });
    const faces=[],vertices=[new THREE.Vector3(),new THREE.Vector3(),new THREE.Vector3()],center=new THREE.Vector3();
    for(let i=0;i<(ix?.count??p.count);i+=3) {
        vertices.forEach((v,k)=>v.fromBufferAttribute(p,ix?ix.getX(i+k):i+k));
        center.copy(vertices[0]).add(vertices[1]).add(vertices[2]).multiplyScalar(1/3);
        let owner=null,best=Infinity;
        for(const candidate of parts){const hit=candidate.geometry.boundsTree.closestPointToPoint(center,{});
            if(hit.distance<best){best=hit.distance;owner=candidate.part;}}
        const distance=owner.releaseOffset+Math.max(...vertices.map(v=>owner.path.nearest(v).s));
        // Bind the union boundary to fabric triangles once. A commanded roll
        // then updates vertices without repeating expensive Boolean operations.
        const bindings=vertices.map(vertex=>{
            let best=null;
            for(const candidate of parts) {
                const hit=candidate.geometry.boundsTree.closestPointToPoint(vertex,{});
                if(!best||hit.distance<best.hit.distance)best={...candidate,hit};
            }
            const {geometry,hit,part}=best,indices=Array.from({length:3},(_,k)=>geometry.index.getX(hit.faceIndex*3+k));
            const points=indices.map(i=>new THREE.Vector3().fromBufferAttribute(geometry.attributes.position,i));
            const barycentric=THREE.Triangle.getBarycoord(hit.point,...points,new THREE.Vector3());
            let weights=barycentric?.toArray();
            if(!weights) {
                // Off-target release may collapse a row against a closed wall.
                // Bind a degenerate face to its longest surviving edge.
                const edges=[[0,1],[1,2],[2,0]].sort((a,b)=>points[b[0]].distanceToSquared(points[b[1]])-points[a[0]].distanceToSquared(points[a[1]]));
                const [a,b]=edges[0],edge=points[b].clone().sub(points[a]);
                const t=THREE.MathUtils.clamp(hit.point.clone().sub(points[a]).dot(edge)/Math.max(1e-12,edge.lengthSq()),0,1);
                weights=[0,0,0];weights[a]=1-t;weights[b]=t;
            }
            return {part,indices,weights};
        });
        faces.push({distance,positions:vertices.flatMap(v=>v.toArray()),bindings});
    }
    for(const {geometry} of parts)geometry.dispose();surface.dispose();
    return faces;
}

export function partialSurfaceSnapshot(devices,complete,revision) {
    const wall=[];
    if(complete){const p=complete.geometry.attributes.position,ix=complete.geometry.index;
        for(let i=0;i<(ix?.count??p.count);i++){const n=ix?ix.getX(i):i;wall.push(p.getX(n),p.getY(n),p.getZ(n));}}
    for(const device of devices) {
        const exposed=graftCoverWithdrawal(device)-device.coverLead-2;
        for(const face of device.contactFaces)if(face.distance<=exposed)wall.push(...face.positions);
    }
    if(!wall.length)return null;
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(wall,3));
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    return {geometry,bounds:geometry.boundingBox.clone(),revision};
}

export function updatePartialSurfacePose(device) {
    for(const face of device.contactFaces)for(let v=0;v<3;v++) {
        const {part,indices,weights}=face.bindings[v];
        for(let axis=0;axis<3;axis++)face.positions[v*3+axis]=indices.reduce((sum,index,k)=>sum+part.target[index*3+axis]*weights[k],0);
    }
}
