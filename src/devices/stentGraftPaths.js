import * as THREE from 'three';

// Landmarks in the atlas physics coordinates (shared by both anatomy variants).
export const AORTIC_NECK = new THREE.Vector3(8, -196, -7);
export const AORTIC_BIFURCATION = new THREE.Vector3(-1.63, -253.78, 9.94);

export class DevicePath {
    constructor(points, coordinates = null) {
        this.points = points.map(p => new THREE.Vector3(p.x, p.y, p.z));
        this.coordinates = coordinates ? [...coordinates] : [0];
        if (!coordinates) for (let i=1;i<points.length;i++)
            this.coordinates.push(this.coordinates[i-1]+this.points[i].distanceTo(this.points[i-1]));
        this.length = this.coordinates.at(-1) ?? 0;
    }
    sample(s) {
        if (!this.points.length) return null;
        if (s<=this.coordinates[0]) return this.points[0].clone();
        for(let i=1;i<this.points.length;i++)if(s<=this.coordinates[i]) {
            const t=(s-this.coordinates[i-1])/Math.max(1e-9,this.coordinates[i]-this.coordinates[i-1]);
            return this.points[i-1].clone().lerp(this.points[i],t);
        }
        return this.points.at(-1).clone();
    }
    nearest(point) {
        let best={distance:Infinity,s:0,point:this.points[0]?.clone()};
        for(let i=1;i<this.points.length;i++) {
            const a=this.points[i-1],delta=this.points[i].clone().sub(a);
            const t=THREE.MathUtils.clamp(point.clone().sub(a).dot(delta)/Math.max(1e-9,delta.lengthSq()),0,1);
            const p=a.clone().addScaledVector(delta,t),distance=p.distanceTo(point);
            if(distance<best.distance)best={distance,point:p,s:this.coordinates[i-1]+t*(this.coordinates[i]-this.coordinates[i-1])};
        }
        return best;
    }
    section(start,end,spacing=3) {
        const count=Math.max(1,Math.ceil(Math.abs(end-start)/spacing));
        return Array.from({length:count+1},(_,i)=>this.sample(start+(end-start)*i/count));
    }
}

export function wireDevicePath({nodes,coordinate}) {
    if(!nodes?.length)return new DevicePath([]);
    const points=[],coordinates=[];
    for(let i=0;i<nodes.length;i++) {
        const s=coordinate(i);
        if(s<0 && i<nodes.length-1 && coordinate(i+1)>0) {
            const t=-s/(coordinate(i+1)-s);
            points.push(new THREE.Vector3(nodes[i].x,nodes[i].y,nodes[i].z)
                .lerp(new THREE.Vector3(nodes[i+1].x,nodes[i+1].y,nodes[i+1].z),t));coordinates.push(0);
        }
        if(s>=0){points.push(nodes[i]);coordinates.push(s);}
    }
    return new DevicePath(points,coordinates);
}

export function createAorticRoutes(segments,sheaths) {
    const nodes=new Map();
    const key=p=>`${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}`;
    for(const segment of segments) {
        const a=key(segment.start),b=key(segment.end);
        for(const [id,p] of [[a,segment.start],[b,segment.end]])if(!nodes.has(id))nodes.set(id,{point:new THREE.Vector3(p.x,p.y,p.z),neighbors:[]});
        nodes.get(a).neighbors.push(b);nodes.get(b).neighbors.push(a);
    }
    const nearest=p=>{
        let id=null,d=Infinity;for(const [candidate,node]of nodes) {
            const distance=node.point.distanceToSquared(p);if(distance<d){d=distance;id=candidate;}
        }return id;
    };
    const target=nearest(new THREE.Vector3(8,-175,-10)),routes={};
    for(const [side,sheath] of Object.entries(sheaths)) {
        const root=nearest(new THREE.Vector3(sheath.end.x,sheath.end.y,sheath.end.z));
        const queue=[root],parents=new Map([[root,null]]);
        for(let i=0;i<queue.length&&!parents.has(target);i++)for(const next of nodes.get(queue[i]).neighbors)
            if(!parents.has(next)){parents.set(next,queue[i]);queue.push(next);}
        if(!parents.has(target))throw Error('Brak ciągłej drogi od koszulki do aorty');
        const points=[];for(let id=target;id!==null;id=parents.get(id))points.push(nodes.get(id).point);
        routes[side]=new DevicePath(points.reverse());
    }
    return routes;
}
