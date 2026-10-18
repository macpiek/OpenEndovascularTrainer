import {sewnGeometry,sewnPositions} from './stentGraftSewnSurface.js';
import * as THREE from 'three';
import {Brush, Evaluator, ADDITION} from 'three-bvh-csg';
import {MeshBVH} from 'three-mesh-bvh';
import {DevicePath} from './stentGraftPaths.js';
import {acceptedPartPositions,connectedGraftLimb,openGraftBranches} from './stentGraftConnections.js';

function closedPart(part,source=part.target,points=part.points) {
    const {rows,sides}=part, positions=Array.from(source), indices=Array.from(part.mesh.geometry.index.array);
    for(const [row,reverse] of [[0,false],[rows-1,true]]) {
        const center=points[row], index=positions.length/3;positions.push(center.x,center.y,center.z);
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
    constructor(implants,revision,flowTopology=null) {
        this.revision=revision;
        const body=implants.find(d=>d.type==='body'),openBranches=openGraftBranches(implants,body);
        const connections=body?[1,2].map(i=>connectedGraftLimb(implants,body,i)).filter(Boolean):[];
        this.sealed=flowTopology?.sealed??!!(body&&!openBranches.length);
        this.parts=implants.flatMap(d=>d.parts);
        // Distal transport territories remain attached to the access that
        // delivered each component, even when the implant axis is off-centre.
        this.flowOutlets=implants.map(d=>d.deliveryPath?.points[0]?.clone()).filter(Boolean);
        // Section areas are immutable for a deployed surface. Compute once,
        // not for every contrast cell or particle query.
        this.paths=this.parts.map(part=>{
            const deployed=implants.some(d=>d.phase==='deployed'&&d.parts.includes(part));
            const positions=deployed?acceptedPartPositions(part):part.target;
            const points=part.points.map((_,row)=>{
                const center=new THREE.Vector3();
                for(let j=0;j<part.sides;j++)center.add(new THREE.Vector3().fromArray(positions,(row*part.sides+j)*3));
                return center.divideScalar(part.sides);
            });
            const radii=points.map((center,row)=>{
                let area=0;
                const a=new THREE.Vector3(),b=new THREE.Vector3();
                for(let j=0;j<part.sides;j++) {
                    a.fromArray(positions,(row*part.sides+j)*3).sub(center);
                    b.fromArray(positions,(row*part.sides+(j+1)%part.sides)*3).sub(center);
                    area+=a.cross(b).length()/2;
                }
                return Math.sqrt(area/Math.PI);
            });
            return {path:new DevicePath(points),part,radii,positions:positions.slice()};
        });
        const evaluator=new Evaluator();evaluator.attributes=['position','normal'];evaluator.useGroups=false;
        let brush=null;
        const solids=implants.flatMap(device=>{
            if(device.type==='body'&&device.sewnTopology?.parts===device.parts){
                const geometry=sewnGeometry(device,{closed:true,target:device.phase!=='deployed'});
                // The closed Y replaces three overlapping capped solids.
                const p=geometry.attributes.position,ix=geometry.index,a=new THREE.Vector3(),b=a.clone(),c=a.clone();let volume=0;
                for(let i=0;i<ix.count;i+=3)volume+=a.fromBufferAttribute(p,ix.getX(i)).dot(b.fromBufferAttribute(p,ix.getX(i+1)).cross(c.fromBufferAttribute(p,ix.getX(i+2))));
                if(volume<0)for(let i=0;i<ix.count;i+=3){const a=ix.getX(i+1);ix.setX(i+1,ix.getX(i+2));ix.setX(i+2,a);}
                geometry.computeVertexNormals();return [geometry];
            }
            return device.parts.map(part=>{const snapshot=this.paths.find(p=>p.part===part);return closedPart(part,snapshot.positions,snapshot.path.points);});
        });
        for(const geometry of solids) {
            const next=new Brush(geometry);next.updateMatrixWorld();
            if(!brush){brush=next;continue;}
            const result=evaluator.evaluate(brush,next,ADDITION);brush.geometry.dispose();next.geometry.dispose();brush=result;
        }
        this.solid=brush.geometry;this.solid.boundsTree=new MeshBVH(this.solid);
        this.solid.computeBoundingBox();this.bounds=this.solid.boundingBox.clone();
        const ports=flowTopology?.ports??[];
        if(!flowTopology) {
            if(body) {
                ports.push([body.parts[0],0]);
                for(const i of [1,2])if(!connectedGraftLimb(implants,body,i))ports.push([body.parts[i],-1]);
            }
            for(const device of implants.filter(d=>d.type==='limb')) {
                ports.push([device.parts[0],-1]);
                if(!connections.includes(device))ports.push([device.parts[0],0]);
            }
        }
        const gateParts=flowTopology?.gateParts??openBranches.map(i=>body.parts[i]);
        this.openGates=gateParts.map(part=>{const points=this.paths.find(p=>p.part===part).path.points;return {part,center:points.at(-1).clone(),
            normal:points.at(-1).clone().sub(points.at(-2)).normalize()};});
        this.openGate=this.openGates[0]??null;
        const sewnPorts=new Map();
        for(const device of implants)if(device.type==='body'&&device.sewnTopology?.parts===device.parts){
            const topology=device.sewnTopology,positions=sewnPositions(topology,device.phase!=='deployed');
            for(const port of topology.ports){
                const boundary=port.ids.map(i=>new THREE.Vector3().fromArray(positions,i*3));
                const center=boundary.reduce((sum,p)=>sum.add(p),new THREE.Vector3()).divideScalar(boundary.length);
                sewnPorts.set(port.part,boundary.map((p,j)=>new THREE.Triangle(center,p,boundary[(j+1)%boundary.length])));
            }
        }
        this.portals=ports.map(([part,end])=>{
            const snapshot=this.paths.find(entry=>entry.part===part),points=snapshot.path.points;
            const index=end===0?0:part.rows-1,center=points[index];
            const normal=center.clone().sub(points[end===0?1:index-1]).normalize();
            const radii=snapshot.radii;
            const boundary=Array.from({length:part.sides},(_,j)=>new THREE.Vector3().fromArray(snapshot.positions,(index*part.sides+j)*3));
            const triangles=sewnPorts.get(part)??boundary.map((p,j)=>new THREE.Triangle(center,p,boundary[(j+1)%boundary.length]));
            const bounds=new THREE.Box3().setFromPoints(triangles.flatMap(t=>[t.a,t.b,t.c])).expandByScalar(.002);
            return {center,normal,radius:radii[index],triangles,bounds};
        });
        const p=this.solid.attributes.position,ix=this.solid.index,wall=[];
        const vertices=[new THREE.Vector3(),new THREE.Vector3(),new THREE.Vector3()];
        for(let i=0;i<(ix?.count??p.count);i+=3) {
            vertices.forEach((v,k)=>v.fromBufferAttribute(p,ix?ix.getX(i+k):i+k));
            // Shared cloth can have a slightly nonplanar port after wire
            // projection. Remove its actual cap triangles, not a nominal plane.
            const scratch=new THREE.Vector3();
            const isCap=this.portals.some(port=>vertices.every(v=>port.bounds.containsPoint(v))&&(port.triangles
                ?port.triangles.some(triangle=>vertices.every(v=>triangle.closestPointToPoint(v,scratch).distanceToSquared(v)<1e-8))
                :vertices.every(v=>Math.abs(v.clone().sub(port.center).dot(port.normal))<.002)));
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
        for(const {path,part,radii} of this.paths) {
            const near=path.nearest(point);
            if(best&&near.distance>=best.distance)continue;
            let row=0;
            while(row<path.points.length-2&&path.coordinates[row+1]<near.s)row++;
            const tangent=path.points[row+1].clone().sub(path.points[row]).normalize();
            const startDistance=point.clone().sub(path.points[0]).dot(path.points[1].clone().sub(path.points[0]).normalize());
            const endDistance=point.clone().sub(path.points.at(-1)).dot(path.points.at(-1).clone().sub(path.points.at(-2)).normalize());
            if(startDistance<-.01||endDistance>.01)continue; // open portals are not fabric
            const t=(near.s-path.coordinates[row])/Math.max(1e-9,path.coordinates[row+1]-path.coordinates[row]);
            best={distance:near.distance,radius:THREE.MathUtils.lerp(radii[row],radii[row+1],t),
                center:path.sample(near.s),tangent,part,s:near.s};
        }
        return best;
    }

    nearest(point) {return this.geometry.boundsTree.closestPointToPoint(point,{});}
    dispose(){this.geometry.dispose();this.solid.dispose();}
}
