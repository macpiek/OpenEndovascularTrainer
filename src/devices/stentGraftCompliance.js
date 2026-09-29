import {graftBranchContactAt} from './stentGraftBranchContact.js';
import {createSharedAxisSegmentContact} from '../physics/kirchhoffSharedAxisSegmentContact.js';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {sharedAxisOuterIntervals} from '../physics/kirchhoffSharedAxisNative.js';

// Reduced local indentation of apposed cloth, not a constitutive nitinol model.
// The vessel stays rigid. Only tools already on the exterior side can lift the
// graft away from the wall; an intraluminal wire cannot use this to pierce it.
export function displaceGraftPoint(point,patches) {
    let best=0,normal=null;
    for(const patch of patches) {
        const dx=point.x-patch.point[0],dy=point.y-patch.point[1],dz=point.z-patch.point[2],n=patch.normal;
        const axial=dx*n[0]+dy*n[1]+dz*n[2],tangent2=Math.max(0,dx*dx+dy*dy+dz*dz-axial*axial);
        if(Math.abs(axial)>1.5||tangent2>=patch.width**2)continue;
        const weight=(1-tangent2/patch.width**2)**2,depth=patch.depth*weight;
        if(depth>best){best=depth;normal=n;}
    }
    if(normal){point.x-=normal[0]*best;point.y-=normal[1]*best;point.z-=normal[2]*best;}
    return point;
}
// Local support lookup for one immutable indentation snapshot. Avoid testing
// every patch against every fabric vertex during each trial refit.
export function graftDisplacementSampler(patches) {
    const cells=new Map(),size=4;
    for(const patch of patches) {
        const extent=patch.normal.map(n=>patch.width*Math.sqrt(Math.max(0,1-n*n))+1.5*Math.abs(n));
        const low=patch.point.map((v,k)=>Math.floor((v-extent[k])/size)),high=patch.point.map((v,k)=>Math.floor((v+extent[k])/size));
        for(let x=low[0];x<=high[0];x++)for(let y=low[1];y<=high[1];y++)for(let z=low[2];z<=high[2];z++) {
            const key=`${x}/${y}/${z}`;let list=cells.get(key);if(!list)cells.set(key,list=[]);list.push(patch);
        }
    }
    return point=>displaceGraftPoint(point,cells.get(`${Math.floor(point.x/size)}/${Math.floor(point.y/size)}/${Math.floor(point.z/size)}`)??[]);
}
export function compliantGraftSurface(surface,previous) {
    if(surface.complianceApplied)return {surface,patches:surface.contactPatches??[]};
    const field=previous.wallSamples?.find(s=>s.vesselField)?.vesselField;
    if(!field)return {surface,patches:[]};
    const inherited=previous.wallSamples?.find(s=>s.graftSurface)?.contactPatches??[];
    const patches=[],bounds=surface.bounds.clone().expandByScalar(3);
    const p=new THREE.Vector3(),normal=new THREE.Vector3(),a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3();
    const positions=surface.geometry.attributes.position,ix=surface.geometry.index;
    for(let e=0;e<previous.positions.length-1;e++)for(const interval of sharedAxisOuterIntervals(previous,e)) {
        const radius=interval.material.body.radius;
        if(radius>1.1)continue; // Introducers/delivery systems do not lift the graft.
        const count=Math.max(1,Math.ceil((previous.coordinates[e+1]-previous.coordinates[e])*(interval.end-interval.start)/.6));
        for(let j=0;j<=count;j++) {
            const t=interval.start+(interval.end-interval.start)*j/count;
            p.fromArray(previous.positions[e]).lerp(new THREE.Vector3(...previous.positions[e+1]),t).add(new THREE.Vector3(...previous.origin));
            if(!bounds.containsPoint(p))continue;
            // A still-threaded delivery wire inside the graft cannot be an
            // exterior tool lifting the fabric away from the vessel wall.
            if(surface.ownedBranches?.length) {
                if(surface.contains?.(p))continue;
                // During extension release, contains() can describe only the
                // already deployed parent. Check the moving extension as well.
                const branch=graftBranchContactAt(surface.ownedBranches,p.toArray(),radius);
                if(branch&&!branch.outside)continue;
            }
            const hit=surface.geometry.boundsTree.closestPointToPoint(p,{},0,radius+.6);
            if(!hit||hit.distance>radius+.6)continue;
            const f=hit.faceIndex;
            [a,b,c].forEach((v,k)=>v.fromBufferAttribute(positions,ix?ix.getX(f*3+k):f*3+k));
            normal.crossVectors(b.clone().sub(a),c.clone().sub(a)).normalize();
            const signed=p.clone().sub(hit.point).dot(normal);
            const existingExterior=inherited.some(q=>new THREE.Vector3(...q.point).distanceToSquared(hit.point)<q.width**2);
            if(normal.lengthSq()<.5||signed < -2*radius-.2 || signed<0&&!existingExterior&&(surface.contains?.(p)??true))continue;
            const wall=field.querySphere(p,0);
            if(!wall.inside||wall.signedDistance>radius+1.2)continue;
            const apposition=field.querySphere(hit.point,0);
            if(!apposition.inside||apposition.signedDistance>2*radius+.15)continue;
            // Start lifting ahead of contact; bounded by the tool diameter.
            const depth=Math.min(radius*2+.2,Math.max(0,radius+.18-signed));
            if(depth<1e-4)continue;
            if(patches.some(q=>new THREE.Vector3(...q.point).distanceToSquared(hit.point)<.25&&q.depth>=depth))continue;
            patches.push({coordinate:previous.coordinates[e]+t*(previous.coordinates[e+1]-previous.coordinates[e]),radius,point:hit.point.toArray(),normal:normal.toArray(),depth,width:Math.max(3,4*radius)});
        }
    }
    const all=[...(surface.otherContactPatches??[]),...patches];
    if(!all.length&&!inherited.length)return {surface,patches};
    const geometry=surface.geometry.clone(),attribute=geometry.attributes.position,rest=surface.geometry.attributes.position;
    // The last accepted rod may be outside the REST cloth because it lifted it.
    // Validate against that accepted indentation, not just the undeformed sheet.
    const acceptedDisplacement=graftDisplacementSampler([...(surface.otherContactPatches??[]),...inherited]);
    for(let i=0;i<attribute.count;i++) {
        p.fromBufferAttribute(rest,i);acceptedDisplacement(p);
        attribute.setXYZ(i,p.x,p.y,p.z);
    }
    geometry.boundsTree=new MeshBVH(geometry);
    const originalQuery=createSharedAxisSegmentContact(surface.geometry),acceptedQuery=createSharedAxisSegmentContact(geometry),segments=[];
    for(let e=0;e<previous.positions.length-1;e++)for(const interval of sharedAxisOuterIntervals(previous,e)) {
        const a=previous.positions[e].map((v,k)=>v+(previous.positions[e+1][k]-v)*interval.start+previous.origin[k]);
        const b=previous.positions[e].map((v,k)=>v+(previous.positions[e+1][k]-v)*interval.end+previous.origin[k]);
        if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([new THREE.Vector3(...a),new THREE.Vector3(...b)])))continue;
        const radius=interval.material.body.radius;
        const restHit=originalQuery.query(a,b,radius),oldHit=acceptedQuery.query(a,b,radius);
        // Preserve the physical envelope, not just the mathematical axis.
        // Existing elastic penetration may remain, but relaxation must not
        // deepen it below the better of the rest and accepted configurations.
        const clearance=Math.min(radius,Math.max(restHit.distance,oldHit.distance));
        segments.push({a,b,radius,clearance,protectAxis:!restHit.crossing||!oldHit.crossing});
    }
    const clearsTools=query=>!segments.some(({a,b,radius,clearance,protectAxis})=>{
        const hit=query.query(a,b,radius);
        return (protectAxis&&hit.crossing)||hit.distance<clearance-1e-5;
    });
    let accepted=null,own=[];
    for(let attempt=0;attempt<=11;attempt++) {
        const scale=attempt===11?0:2**(-Math.max(0,attempt-1));
        // Try relaxing to the newly estimated cloth first. If that closes over
        // the accepted tool, retain its indentation until the tool is clear.
        own=[...(attempt?inherited:[]),...patches.map(p=>({...p,depth:p.depth*scale})).filter(p=>p.depth>1e-5)];
        own=own.filter((p,i)=>!own.some((q,j)=>j<i&&q.depth>=p.depth&&q.width===p.width&&q.point.every((v,k)=>Math.abs(v-p.point[k])<1e-6)));
        const candidate=[...(surface.otherContactPatches??[]),...own],displace=graftDisplacementSampler(candidate);
        for(let i=0;i<attribute.count;i++) {
            p.fromBufferAttribute(rest,i);displace(p);attribute.setXYZ(i,p.x,p.y,p.z);
        }
        if(geometry.boundsTree)geometry.boundsTree.refit();else geometry.boundsTree=new MeshBVH(geometry);
        const query=createSharedAxisSegmentContact(geometry);
        if(clearsTools(query)){accepted=candidate;break;}
    }
    if(!accepted) {
        // Reverting to rest is safe only when it also preserves tool clearance.
        // Otherwise retain the accepted indentation; a failed relaxation must
        // not close the graft over an exterior wire or flip its contact side.
        if(clearsTools(originalQuery)){geometry.dispose();return {surface,patches:[]};}
        own=inherited;
        accepted=[...(surface.otherContactPatches??[]),...own];
        for(let i=0;i<attribute.count;i++) {
            p.fromBufferAttribute(rest,i);acceptedDisplacement(p);attribute.setXYZ(i,p.x,p.y,p.z);
        }
        geometry.boundsTree.refit();
    }
    geometry.computeBoundingBox();
    return {surface:{...surface,geometry,bounds:geometry.boundingBox.clone(),complianceApplied:true,contactPatches:accepted,restSurface:surface},patches:own};
}
