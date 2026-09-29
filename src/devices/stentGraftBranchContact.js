import {sewnRingExposure} from './stentGraftDeployment.js';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';

// The delivery wire is already threaded through the long body branch or
// the separately delivered limb when its cloth opens. Preserve that side of
// the moving cloth instead of selecting whichever outlet is currently closest.
// These are actual fabric triangles, not a circular centreline tether.
export function graftOwnedBranches(device) {
    if(device.deliveryWireReleased)return [];
    const part=device.parts[device.type==='body'?1:0];
    if(!part)return [];
    const complete=device.phase==='deployed'||part.exposure?.every(value=>value===1);
    // Free rings contact the tools while still expanding. A partly uncovered
    // ring is carried by the delivery cover until its trailing struts clear.
    // A still-captured ring surrounds the delivery cover, whose outer radius
    // exceeds the crimped graft lumen. Do not collide the cover with that lumen.
    const firstCaptured=part.path.coordinates.findIndex((s,i)=>(sewnRingExposure(part,s,true)??part.exposure?.[i]??0)<=0);
    const exposedRows=firstCaptured<0?part.rows:firstCaptured;
    const rows=complete?part.rows:exposedRows;
    const positions=part.contactBasePositions??part.mesh.geometry.attributes.position.array;
    if(rows<2)return [];
    const count=rows*part.sides;
    return [{transitioning:!complete,positions:Array.from(positions.slice(0,count*3)),indices:Array.from(part.mesh.geometry.index.array).slice(0,(rows-1)*part.sides*6),
        centers:part.points.slice(0,rows).map((_,row)=>{
            const center=[0,0,0];
            for(let j=0;j<part.sides;j++)for(let k=0;k<3;k++)center[k]+=positions[(row*part.sides+j)*3+k]/part.sides;
            return center;
        })}];
}
const compiled=new WeakMap();
function compile(branch) {
    if(compiled.has(branch))return compiled.get(branch);
    const centers=branch.centers.map(p=>new THREE.Vector3(...p));
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.Float32BufferAttribute(branch.positions,3));
    geometry.setIndex(branch.indices);
    const p=geometry.attributes.position,ix=geometry.index,a=new THREE.Vector3(),b=a.clone(),c=a.clone();
    const sides=p.count/centers.length;
    // Preserve the mesh's coherent winding. Flipping individual triangles
    // toward an averaged centre reverses folded/concave faces independently.
    // Temporary portal caps are used only to determine the volume orientation.
    let volume=0;
    for(let f=0;f<ix.count;f+=3) {
        a.fromBufferAttribute(p,ix.getX(f));b.fromBufferAttribute(p,ix.getX(f+1));c.fromBufferAttribute(p,ix.getX(f+2));
        volume+=a.dot(b.cross(c));
    }
    for(const [row,reverse] of [[0,false],[centers.length-1,true]])for(let j=0;j<sides;j++) {
        a.copy(centers[row]);b.fromBufferAttribute(p,row*sides+j);c.fromBufferAttribute(p,row*sides+(j+1)%sides);
        volume+=a.dot(b.cross(c))*(reverse?-1:1);
    }
    if(volume<0)for(let f=0;f<ix.count;f+=3) {
        const second=ix.getX(f+1);ix.setX(f+1,ix.getX(f+2));ix.setX(f+2,second);
    }
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    let maxRadius=0;
    const radii=centers.map(()=>0);
    for(let i=0;i<p.count;i++) {
        const row=Math.floor(i/sides),radius=a.fromBufferAttribute(p,i).distanceTo(centers[row]);
        radii[row]=Math.max(radii[row],radius);maxRadius=Math.max(maxRadius,radius);
    }
    const recoveryBounds=geometry.boundingBox.clone().expandByScalar(2*maxRadius);
    const data={geometry,centers,radii,recoveryBounds};compiled.set(branch,data);return data;
}

/** Signed clearance from the actual ipsilateral cloth. Open ports are not
 * capped: beyond either end there is no reaction. Within the lumen there is
 * no force until the finite tool radius touches the wall. */
export function graftBranchContactAt(branches,point,radius=0) {
    if(!branches?.length)return null;
    const p=new THREE.Vector3(...point);
    for(const branch of branches) {
        const {geometry,centers,radii,recoveryBounds}=compile(branch);
        // Tilted portals of a bent, partially released limb must not turn
        // ownership into an unbounded slab acting on remote wire segments.
        if(recoveryBounds.distanceToPoint(p)>radius)continue;
        const first=centers[1].clone().sub(centers[0]),last=centers.at(-1).clone().sub(centers.at(-2));
        if(p.clone().sub(centers[0]).dot(first)<0||p.clone().sub(centers.at(-1)).dot(last)>0)continue;
        // A local tubular bound also rejects remote points in the large box
        // of a bent limb; endpoint planes alone do not describe its interior.
        let nearby=false;
        for(let i=1;i<centers.length&&!nearby;i++) {
            const axis=centers[i].clone().sub(centers[i-1]);
            const t=Math.max(0,Math.min(1,p.clone().sub(centers[i-1]).dot(axis)/Math.max(1e-12,axis.lengthSq())));
            const center=centers[i-1].clone().addScaledVector(axis,t);
            nearby=p.distanceTo(center)<=2*Math.max(radii[i-1],radii[i])+radius;
        }
        if(!nearby)continue;
        const hit=geometry.boundsTree.closestPointToPoint(p,{});if(!hit)continue;
        const ix=geometry.index,pos=geometry.attributes.position;
        const vertices=[0,1,2].map(k=>new THREE.Vector3().fromBufferAttribute(pos,ix.getX(hit.faceIndex*3+k)));
        const outward=vertices[1].clone().sub(vertices[0]).cross(vertices[2].clone().sub(vertices[0])).normalize();
        const delta=p.clone().sub(hit.point),outside=delta.dot(outward)>0;
        const normal=hit.distance>1e-8?delta.divideScalar(hit.distance).multiplyScalar(outside?-1:1):outward.clone().negate();
        const gap=(outside?-hit.distance:hit.distance)-radius;
        return Object.defineProperty({gap,outside,transitioning:branch.transitioning===true,normal:normal.toArray(),planeNormal:outward.clone().negate().toArray(),point:hit.point.toArray(),
            start:centers[0].toArray(),end:centers.at(-1).toArray(),axisStart:first.normalize().toArray(),axisEnd:last.normalize().toArray()},'geometry',{value:geometry});
    }
    return null;
}


// Freeze the contacted face for a Newton step. Re-selecting nearest faces in
// a polygonal lumen creates a nonsmooth medial-axis force during recovery.
// A C2 fade at the real portals leaves them open without a jump in energy.
export function branchRecoveryPotential(contact,point,radius) {
    const dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),sub=(a,b)=>a.map((v,k)=>v-b[k]);
    const n=contact.planeNormal,gap=dot(sub(point,contact.point),n)-radius+(contact.recoveryOffset??0);
    const zero={energy:0,gradient:[0,0,0],hessian:Array.from({length:3},()=>[0,0,0])};
    if(gap>=0)return zero;
    const fade=(d,axis)=>{
        const width=2,t=Math.max(0,Math.min(1,d/width));
        return {w:t*t*t*(10-15*t+6*t*t),g:axis.map(v=>v*30*t*t*(1-t)*(1-t)/width),
            h:axis.map(v=>axis.map(u=>v*u*60*t*(1-t)*(1-2*t)/(width*width)))};
    };
    const a=fade(dot(sub(point,contact.start),contact.axisStart),contact.axisStart);
    const reverse=contact.axisEnd.map(v=>-v),b=fade(dot(sub(contact.end,point),contact.axisEnd),reverse);
    const w=a.w*b.w,g=a.g.map((v,i)=>v*b.w+b.g[i]*a.w);
    const h=a.h.map((row,i)=>row.map((v,j)=>v*b.w+b.h[i][j]*a.w+a.g[i]*b.g[j]+b.g[i]*a.g[j]));
    return {energy:.5*w*gap*gap,gradient:n.map((v,i)=>w*gap*v+.5*gap*gap*g[i]),
        hessian:n.map((v,i)=>n.map((u,j)=>w*v*u+gap*(v*g[j]+g[i]*u)+.5*gap*gap*h[i][j]))};
}
