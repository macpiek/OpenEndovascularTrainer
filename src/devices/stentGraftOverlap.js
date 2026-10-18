import {Vector3,Ray,DoubleSide,BufferGeometry,Float32BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {acceptedPartPositions} from './stentGraftConnections.js';

/** Parent cloth is a radial stop in the overlap, with an open distal portal.
 * Snapshot once at docking: the child's expansion must never refit the parent
 * or treat the native aneurysm wall as the only limit on its diameter. */
export function graftOverlapFit(vessel,port) {
    const geometry=new BufferGeometry();
    geometry.setAttribute('position',new Float32BufferAttribute(acceptedPartPositions(port.part),3));
    geometry.setIndex(port.part.mesh.geometry.index.clone());
    geometry.boundsTree=new MeshBVH(geometry);
    const path=port.path,start=path.points[0],end=path.points.at(-1);
    const first=path.points[1].clone().sub(start).normalize(),last=end.clone().sub(path.points.at(-2)).normalize();
    const ray=new Ray(),delta=new Vector3();
    const inSpan=p=>delta.copy(p).sub(start).dot(first)>=0&&delta.copy(p).sub(end).dot(last)<=0;
    const fitOverlap=point=>{
        if(!inSpan(point))return point;
        const center=path.nearest(point).point;
        ray.origin.copy(center);ray.direction.copy(point).sub(center);
        const length=ray.direction.length();if(length<1e-9)return point;
        ray.direction.divideScalar(length);
        const hit=geometry.boundsTree.raycastFirst(ray,DoubleSide,0,length+.15);
        if(hit)point.copy(center).addScaledVector(ray.direction,Math.min(length,Math.max(0,hit.distance-.08)));
        return point;
    };
    return {
        field:vessel.field,wall:vessel.wall,query:(p,r)=>vessel.query(p,r),connected:(a,b)=>vessel.connected(a,b),
        fitOverlap,
        inward(point) {return inSpan(point)?path.nearest(point).point.sub(point).normalize():null;},
        fitAxis(point,anchor) {
            vessel.fit(point,anchor,.7);
            if(inSpan(point))point.copy(path.nearest(point).point);
            return point;
        },
        fit(point,anchor,clearance=.35) {
            vessel.fit(point,anchor,clearance);
            // Fabric has a small thickness; anatomy's 0.7 mm clearance must
            // not be applied again inside a 14 mm parent outlet.
            return fitOverlap(point);
        },
        dispose(){geometry.dispose();}
    };
}

export function constrainGraftOverlap(device) {
    if(!device.wallFit.fitOverlap)return;
    const point=new Vector3();
    for(const part of device.parts) {
        const p=part.mesh.geometry.attributes.position;let changed=false;
        for(let row=0;row<part.rows;row++)if(part.exposure[row]>0)for(let j=0;j<part.sides;j++) {
            const i=row*part.sides+j;point.fromBufferAttribute(p,i);
            device.wallFit.fitOverlap(point);
            if(p.getX(i)!==Math.fround(point.x)||p.getY(i)!==Math.fround(point.y)||p.getZ(i)!==Math.fround(point.z)) {
                p.setXYZ(i,point.x,point.y,point.z);changed=true;
            }
        }
        if(changed){p.needsUpdate=true;part.contactBasePositions?.set(p.array);}
    }
}

// Expand through the parent's actual outlet before turning into the iliac
// route. Do not start from the atlas axis, which can be outside the parent.
export function graftLimbRoute(port,route,length,nose=port.docking) {
    const near=port.path.nearest(nose),overlap=port.path.length-near.s;
    const points=port.path.section(near.s,port.path.length,2);
    const distal=route.nearest(port.entry).s;
    const continuation=route.section(distal,Math.max(0,distal-Math.max(0,length-overlap)),2);
    if(continuation.length>1)points.push(...continuation.slice(1));
    return points;
}
