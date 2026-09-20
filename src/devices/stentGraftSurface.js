import * as THREE from 'three';
import {Brush, Evaluator, ADDITION} from 'three-bvh-csg';
import {MeshBVH} from 'three-mesh-bvh';
import {DevicePath} from './stentGraftPaths.js';

function closedPart(part) {
    const {rows,sides,target}=part, positions=Array.from(target), indices=Array.from(part.mesh.geometry.index.array);
    for(const [row,reverse] of [[0,false],[rows-1,true]]) {
        const center=part.points[row], index=positions.length/3;positions.push(center.x,center.y,center.z);
        for(let j=0;j<sides;j++) {
            const a=row*sides+j,b=row*sides+(j+1)%sides;
            indices.push(index,...(reverse?[b,a]:[a,b]));
        }
    }
    let volume=0;
    const a=new THREE.Vector3(),b=a.clone(),c=a.clone();
    for(let i=0;i<indices.length;i+=3)volume+=a.fromArray(positions,indices[i]*3).dot(b.fromArray(positions,indices[i+1]*3).cross(c.fromArray(positions,indices[i+2]*3)));
    if(volume<0)for(let i=0;i<indices.length;i+=3)[indices[i+1],indices[i+2]]=[indices[i+2],indices[i+1]];
    const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));g.setIndex(indices);g.computeVertexNormals();return g;
}

/** A union of the deployed fabric volumes. Caps belong to volume queries only;
 * the mechanical surface has open portals and no internal overlap membranes. */
export class StentGraftSurface {
    constructor(implants,revision) {
        this.revision=revision;
        const body=implants.find(d=>d.type==='body'),limb=implants.find(d=>d.type==='limb'&&d.id===body?.connectedLimbId);
        this.sealed=!!(body&&limb&&body.connectedLimbId===limb.id);
        this.parts=implants.flatMap(d=>d.parts);
        this.paths=this.parts.map(part=>({path:new DevicePath(part.points),part}));
        const evaluator=new Evaluator();evaluator.attributes=['position','normal'];evaluator.useGroups=false;
        let brush=null;
        for(const part of this.parts) {
            const next=new Brush(closedPart(part));next.updateMatrixWorld();
            if(!brush){brush=next;continue;}
            const result=evaluator.evaluate(brush,next,ADDITION);brush.geometry.dispose();next.geometry.dispose();brush=result;
        }
        this.solid=brush.geometry;this.solid.boundsTree=new MeshBVH(this.solid);
        this.solid.computeBoundingBox();this.bounds=this.solid.boundingBox.clone();
        const ports=[];
        if(body)ports.push([body.parts[0],0],[body.parts[1],-1]);
        if(body&&!limb)ports.push([body.parts[2],-1]);
        for(const device of implants.filter(d=>d.type==='limb')) {
            ports.push([device.parts[0],-1]);
            if(device!==limb)ports.push([device.parts[0],0]);
        }
        this.portals=ports.map(([part,end])=>{
            const index=end===0?0:part.rows-1,center=part.points[index];
            const normal=center.clone().sub(part.points[end===0?1:index-1]).normalize();
            return {center,normal};
        });
        const p=this.solid.attributes.position,ix=this.solid.index,wall=[];
        const vertices=[new THREE.Vector3(),new THREE.Vector3(),new THREE.Vector3()];
        for(let i=0;i<(ix?.count??p.count);i+=3) {
            vertices.forEach((v,k)=>v.fromBufferAttribute(p,ix?ix.getX(i+k):i+k));
            const isCap=this.portals.some(port=>vertices.every(v=>Math.abs(v.clone().sub(port.center).dot(port.normal))<.002));
            if(!isCap)vertices.forEach(v=>wall.push(v.x,v.y,v.z));
        }
        this.geometry=new THREE.BufferGeometry();this.geometry.setAttribute('position',new THREE.Float32BufferAttribute(wall,3));
        this.geometry.computeVertexNormals();this.geometry.boundsTree=new MeshBVH(this.geometry);
        this._ray=new THREE.Ray(new THREE.Vector3(),new THREE.Vector3(.371,.529,.763).normalize());
    }
    contains(point) {
        if(!this.bounds.containsPoint(point))return false;
        this._ray.origin.copy(point);
        const hit=this.solid.boundsTree.raycastFirst(this._ray,THREE.DoubleSide);
        return !!hit&&hit.face.normal.dot(this._ray.direction)>0;
    }
    /** Radius of the fabric section containing an original arterial centreline.
     * Branches merely inside the aneurysm are not graft transport paths. */
    sectionAt(point) {
        let best=null;
        for(const {path,part} of this.paths) {
            const near=path.nearest(point);
            if(near.distance>2.5||best&&near.distance>=best.distance)continue;
            const index=Math.min(part.rows-1,Math.round(near.s/Math.max(1e-6,path.length)*(part.rows-1)));
            const center=part.points[index];let area=0;
            for(let j=0;j<part.sides;j++) {
                const a=new THREE.Vector3().fromArray(part.target,(index*part.sides+j)*3).sub(center);
                const b=new THREE.Vector3().fromArray(part.target,(index*part.sides+(j+1)%part.sides)*3).sub(center);
                area+=a.cross(b).length()/2;
            }
            best={distance:near.distance,radius:Math.sqrt(area/Math.PI)};
        }return best;
    }
    nearest(point) {return this.geometry.boundsTree.closestPointToPoint(point,{});}
    dispose(){this.geometry.dispose();this.solid.dispose();}
}
