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
    // Ownership starts with the flaring mouth, before the whole metal ring
    // clears the cover. Otherwise the thin core first sees a two-sided sheet
    // and can be expelled through the wrong outlet. Covered delivery intervals
    // exclude their own implant separately; they do not collide with this funnel.
    // A standalone limb has no sewn divider; its contact mesh still starts
    // at the first completely uncovered band.
    const firstCaptured=part.path.coordinates.findIndex((s,i)=>(device.type==='body'?
        part.exposure?.[i]??0:sewnRingExposure(part,s,true)??part.exposure?.[i]??0)<=0);
    const exposedRows=firstCaptured<0?part.rows:firstCaptured;
    const rows=complete?part.rows:exposedRows;
    const positions=part.contactBasePositions??part.mesh.geometry.attributes.position.array;
    if(rows<2)return [];
    const count=rows*part.sides;
    // The sewn body divider can pinch the threaded tool between both walls.
    // A separately delivered limb has no divider: retain its ordinary recovery.
    return [{junctionRecovery:device.type==='body',transitioning:!complete,positions:Array.from(positions.slice(0,count*3)),indices:Array.from(part.mesh.geometry.index.array).slice(0,(rows-1)*part.sides*6),
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
        // Older bifurcation replays predate the explicit limb opt-out.
        return Object.defineProperty({gap,outside,junctionRecovery:branch.junctionRecovery!==false,transitioning:branch.transitioning===true,normal:normal.toArray(),planeNormal:outward.clone().negate().toArray(),point:hit.point.toArray(),
            start:centers[0].toArray(),end:centers.at(-1).toArray(),axisStart:first.normalize().toArray(),axisEnd:last.normalize().toArray()},'geometry',{value:geometry});
    }
    return null;
}


// Include the opposite wall when a folded lumen is narrower than the tool.
// A nearest-face-only recovery alternates sides on successive time steps.
// Signed inward supports continue across the wall; axis entry cannot switch
// off the opposing support while the finite-radius tool still overlaps it.
export function graftBranchRecoveryPlanes(contact,point,radius) {
    if(contact.junctionRecovery===false)return [contact];
    // A triangle is a quadrature sample of one wall, not an independent
    // spring. Normalize the local support by projected area: refining or
    // wrinkling the same wall must not multiply its contact stiffness. Opposite
    // walls retain independent support, so a narrow lumen cannot alternate
    // between two nearest-face reactions.
    const p=new THREE.Vector3(...point),closest=new THREE.Vector3(),reach=radius+.55;
    const supports=[];
    contact.geometry.boundsTree.shapecast({
        intersectsBounds:box=>box.distanceToPoint(p)<reach,
        intersectsTriangle:triangle=>{
            const area=triangle.getArea();if(area<1e-10)return false;
            const n=triangle.getNormal(new THREE.Vector3()).negate();
            triangle.closestPointToPoint(p,closest);
            const distance=closest.distanceToSquared(p);
            if(distance<reach*reach) {
                const influence=area*(1-distance/(reach*reach))**2;
                supports.push({...contact,point:closest.toArray(),planeNormal:n.toArray(),influence});
            }
            return false;
        }
    });
    if(!supports.length)return [contact];
    const planes=[];
    for(const support of supports) {
        const n=support.planeNormal;
        const area=supports.reduce((sum,q)=>sum+q.influence*Math.max(0,
            n.reduce((dot,v,k)=>dot+v*q.planeNormal[k],0)),0);
        const w=support.influence/area;
        const same=planes.find(q=>n.reduce((dot,v,k)=>dot+v*q.planeNormal[k],0)>1-1e-8&&
            Math.abs(n.reduce((sum,v,k)=>sum+v*(support.point[k]-q.point[k]),0))<1e-6);
        if(same)same.weight+=w;
        else planes.push({...support,weight:w});
    }
    return planes;
}

// Freeze the contacted face for a Newton step. Re-selecting nearest faces in
// a polygonal lumen creates a nonsmooth medial-axis force during recovery.
// A C2 fade at the real portals leaves them open without a jump in energy.
export function createBranchRecoveryWorkspace() {
    return {energy:0,gradient:[0,0,0],hessian:[[0,0,0],[0,0,0],[0,0,0]],fadeGradient:[0,0,0],fadeA:[0,0,0],fadeB:[0,0,0]};
}

// A workspace belongs to one sampler, never to a state retained between steps.
// Residual/line-search evaluations need energy and force only; avoid forming
// the 3x3 tangent (and its temporary arrays) in that much more frequent path.
export function branchRecoveryPotential(contact,point,radius,tangentMode,out=createBranchRecoveryWorkspace(),withTangent=true) {
    const n=contact.planeNormal;
    let gap=0,da=0,db=0;
    for(let i=0;i<3;i++) {
        gap+=(point[i]-contact.point[i])*n[i];
        da+=(point[i]-contact.start[i])*contact.axisStart[i];
        db+=(contact.end[i]-point[i])*contact.axisEnd[i];
    }
    gap=gap-radius+(contact.recoveryOffset??0);
    const gradient=out.gradient,H=out.hessian;
    if(gap>=0) {
        out.energy=0;gradient.fill(0);
        if(withTangent)for(const row of H)row.fill(0);
        return out;
    }
    const ta=Math.max(0,Math.min(1,da/2)),tb=Math.max(0,Math.min(1,db/2));
    const wa=ta*ta*ta*(10-15*ta+6*ta*ta),wb=tb*tb*tb*(10-15*tb+6*tb*tb);
    const w=wa*wb,g=out.fadeGradient,ga=out.fadeA,gb=out.fadeB;
    out.energy=.5*w*gap*gap;
    for(let i=0;i<3;i++) {
        ga[i]=contact.axisStart[i]*30*ta*ta*(1-ta)*(1-ta)/2;
        gb[i]=-contact.axisEnd[i]*30*tb*tb*(1-tb)*(1-tb)/2;
        g[i]=ga[i]*wb+gb[i]*wa;
        gradient[i]=w*gap*n[i]+.5*gap*gap*g[i];
    }
    if(!withTangent)return out;
    // E = r²/2, r = sqrt(w) * gap. JᵀJ is positive semidefinite even in
    // the portal fade; keep the exact energy/gradient for trial acceptance.
    if(tangentMode==='gauss-newton') {
        for(let i=0;i<3;i++)for(let j=0;j<3;j++)
            H[i][j]=out.energy>0?gradient[i]*gradient[j]/(2*out.energy):0;
    } else {
        for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
            const ai=contact.axisStart[i],aj=contact.axisStart[j],bi=contact.axisEnd[i],bj=contact.axisEnd[j];
            const ha=ai*aj*60*ta*(1-ta)*(1-2*ta)/4,hb=(-bi)*(-bj)*60*tb*(1-tb)*(1-2*tb)/4;
            const h=ha*wb+hb*wa+ga[i]*gb[j]+gb[i]*ga[j];
            H[i][j]=w*n[i]*n[j]+gap*(n[i]*g[j]+g[i]*n[j])+.5*gap*gap*h;
        }
    }
    return out;
}
