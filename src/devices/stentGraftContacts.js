import {graftLumenAt} from './stentGraftLumenContact.js';
import * as THREE from 'three';
import {sharedAxisOuterIntervals} from '../physics/kirchhoffSharedAxisNative.js';
import {createKirchhoffWallWitnessGeometryWorkspace,evaluateKirchhoffWallWitnessGeometry} from '../physics/kirchhoffWallWitnessGeometry.js';
import {createSharedAxisSegmentContact} from '../physics/kirchhoffSharedAxisSegmentContact.js';

// A stiff normal response arrests motion before the centreline reaches the
// fabric. The logarithmic term steepens near the sheet; CCD remains a final
// guard, rather than the first mechanism that notices contact.
export function graftContactResponse(distance,radius) {
    const floor=radius*.01,d=Math.max(distance,floor),gap=d-radius;
    if(gap>=0)return {energy:0,slope:0,curvature:0};
    const log=Math.log(d/radius),stiffness=1e5,barrier=1e3;
    const energy=.5*stiffness*gap*gap-barrier*gap*gap*log;
    const slope=stiffness*gap-barrier*(2*gap*log+gap*gap/d);
    const curvature=stiffness-barrier*(2*log+4*gap/d-gap*gap/(d*d));
    const delta=distance-d;
    return {energy:energy+slope*delta+.5*curvature*delta*delta,slope:slope+curvature*delta,curvature};
}

const inactive=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
function referencePoint(state,coordinate) {
    let e=0;while(e<state.coordinates.length-2&&state.coordinates[e+1]<coordinate)e++;
    const t=THREE.MathUtils.clamp((coordinate-state.coordinates[e])/(state.coordinates[e+1]-state.coordinates[e]),0,1);
    return new THREE.Vector3(...state.positions[e].map((v,k)=>v*(1-t)+state.positions[e+1][k]*t+state.origin[k]));
}

/** Elastic, frictionless fabric contact integrated into Newton's potential.
 * A thin two-sided sheet needs a finite normal stiffness: activating a rigid
 * KKT wall around an already present rod makes the initial pose infeasible.
 * Sweeps separately reject new crossings, including jumps beyond the spring.
 */
export function createStentGraftContacts(surface,previous) {
    // Never capture the previous native state: its wall sampler captures its
    // predecessor, retaining every band matrix and WASM arena indefinitely.
    const reference={coordinates:[...previous.coordinates],positions:previous.positions.map(p=>[...p]),origin:[...previous.origin]};
    const bounds=surface.bounds.clone().expandByScalar(3),point=new THREE.Vector3(),ray=new THREE.Ray();
    const scratch=createKirchhoffWallWitnessGeometryWorkspace(),closest=new THREE.Vector3();
    const segmentContact=createSharedAxisSegmentContact(surface.geometry);
    const inherited=previous.graftRevision===surface.revision?previous.graftRecovery??[]:null;
    const recovery=[];
    if(previous.materials)for(let e=0;e<reference.positions.length-1;e++) {
        const start=reference.coordinates[e],end=reference.coordinates[e+1];
        if(inherited&&!inherited.some(([a,b])=>a<=end&&b>=start))continue;
        const a=referencePoint(reference,start),b=referencePoint(reference,end);
        if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([a,b])))continue;
        const radius=Math.max(...sharedAxisOuterIntervals(previous,e).map(i=>i.material.body.radius));
        if(segmentContact.query(a.toArray(),b.toArray(),radius).face>=0) {
            if(inherited)recovery.push(...inherited.filter(([a,b])=>a<=end&&b>=start));
            else recovery.push([start-1,end+1]);
        }
    }
    function crossing(a,b) {
        const length=a.distanceTo(b);if(length<1e-8)return null;
        ray.origin.copy(a);ray.direction.copy(b).sub(a).divideScalar(length);
        return surface.geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,length);
    }
    const sample=({state,a,b,radius,coordinateA,coordinateB})=>{
        const wa=new THREE.Vector3(...a).add(new THREE.Vector3(...state.origin)),wb=new THREE.Vector3(...b).add(new THREE.Vector3(...state.origin));
        const oldA=referencePoint(reference,coordinateA),oldB=referencePoint(reference,coordinateB);
        // The owning wire may initially lie outside a newly opened target ring.
        // Let the one-sided lumen potential pull it inward through that incoming
        // surface; keep CCD once it has recovered to the lumen.
        const mid=(coordinateA+coordinateB)/2;
        if([coordinateA,mid,coordinateB].some(s=>graftLumenAt(surface.lumenSections,s,referencePoint(reference,s).toArray(),radius)?.penetration>0))return inactive();
        if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([wa,wb,oldA,oldB])))return inactive();
        // A kinematic release can initially overlap an existing tool. Let the
        // elastic potential resolve that incoming overlap; CCD must not freeze
        // its withdrawal. As soon as clear, subsequent crossings are forbidden.
        if(recovery.some(([a,b])=>a<=coordinateB&&b>=coordinateA)&&
            segmentContact.query(oldA.toArray(),oldB.toArray(),radius).face>=0)return inactive();
        if(crossing(wa,wb))throw Object.assign(new Error('Tool segment crossed stent-graft fabric'),{code:'trial-outside-vessel'});
        const count=Math.max(1,Math.ceil(wa.distanceTo(wb)/.8));
        for(let i=0;i<=count;i++) {
            const t=i/count;point.copy(wa).lerp(wb,t);
            const old=referencePoint(reference,coordinateA+(coordinateB-coordinateA)*t);
            if(crossing(old,point))throw Object.assign(new Error('Tool trial crossed stent-graft fabric'),{code:'trial-outside-vessel'});
        }
        return inactive();
    };
    sample.addPotential=(state,withTangent)=>{
        const {chain,layout}=state,half=layout.band-1,width=2*half+1,H=chain.tangent??chain.hessian;
        const add=(i,j,value)=>{if(!withTangent)return;if(chain.tangent)H[i*width+j-i+half]+=value;else if(i>=j)H[i*layout.band+i-j]+=value;};
        let energy=0,contacts=0,maxPenetration=0;
        for(let e=0;e<state.positions.length-1;e++) {
            const a=new THREE.Vector3(...state.positions[e]).add(new THREE.Vector3(...state.origin));
            const b=new THREE.Vector3(...state.positions[e+1]).add(new THREE.Vector3(...state.origin));
            if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([a,b]))&&!surface.lumenSections?.some(s=>s.start<=state.coordinates[e+1]&&s.end>=state.coordinates[e]))continue;
            const intervals=sharedAxisOuterIntervals(state,e);
            const dofs=[layout.positions[e],layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]);
            for(const interval of intervals) {
                const radius=interval.material.body.radius,length=(state.coordinates[e+1]-state.coordinates[e])*(interval.end-interval.start);
                const count=Math.max(1,Math.ceil(length/.8));
                for(let i=0;i<=count;i++) {
                    const t=interval.start+(interval.end-interval.start)*i/count;point.copy(a).lerp(b,t);
                    const lumen=graftLumenAt(surface.lumenSections,state.coordinates[e]+t*(state.coordinates[e+1]-state.coordinates[e]),point.toArray(),radius);
                    if(lumen) {
                        const penetration=lumen.penetration;
                        if(penetration>0) {
                            const k=1000*length/count*((i===0||i===count)?.5:1),n=lumen.normal;
                            energy+=.5*k*penetration*penetration;contacts++;maxPenetration=Math.max(maxPenetration,penetration);
                            for(let j=0;j<6;j++)for(let l=0;l<6;l++) {
                                const wj=j<3?1-t:t,wl=l<3?1-t:t,jj=j%3,ll=l%3;
                                const curvature=((jj===ll?1:0)-lumen.axis[jj]*lumen.axis[ll]-n[jj]*n[ll])/Math.max(1e-12,lumen.distance);
                                add(dofs[j],dofs[l],k*wj*wl*(n[jj]*n[ll]+penetration*curvature));
                            }
                            for(let j=0;j<6;j++)chain.gradient[dofs[j]]+=k*penetration*(j<3?1-t:t)*n[j%3];
                        }
                        // The circular owning-lumen guide is only a recovery
                        // aid, not a replacement for the actual cloth mesh.
                        // Once the incoming point is inside it, retain ordinary
                        // fabric forces so CCD does not become the only contact.
                        const old=referencePoint(reference,state.coordinates[e]+t*(state.coordinates[e+1]-state.coordinates[e]));
                        if(graftLumenAt(surface.lumenSections,state.coordinates[e]+t*(state.coordinates[e+1]-state.coordinates[e]),old.toArray(),radius)?.penetration>0)continue;
                    }
                    const faces=[];
                    surface.geometry.boundsTree.shapecast({
                        intersectsBounds:box=>box.distanceToPoint(point)<radius,
                        intersectsTriangle:(triangle,index)=>{
                            triangle.closestPointToPoint(point,closest);
                            if(closest.distanceToSquared(point)<radius*radius&&triangle.getArea()>1e-10)faces.push(index);
                            return false;
                        }
                    });
                    // Sum finite-face springs, rather than switching one nearest
                    // face. At a corner both normals must support the tool.
                    for(const faceIndex of faces) {
                        const g=evaluateKirchhoffWallWitnessGeometry({geometry:surface.geometry,faceIndex,point:point.toArray()},scratch);
                        let n=g.direction;
                        if(g.distance<1e-8) {
                            const old=referencePoint(reference,state.coordinates[e]+t*(state.coordinates[e+1]-state.coordinates[e]));
                            n=old.sub(new THREE.Vector3(...g.closestPoint)).normalize().toArray();
                            if(Math.hypot(...n)<1e-8)n=[1,0,0];
                        }
                        const gap=g.distance-radius,weight=length/count*((i===0||i===count) ? .5 : 1);
                        const response=graftContactResponse(g.distance,radius);
                        energy+=weight*response.energy;contacts++;maxPenetration=Math.max(maxPenetration,-gap);
                        const J=[...n.map(v=>v*(1-t)),...n.map(v=>v*t)];
                        const edge=[0,0,0];
                        if(g.feature==='edge') {
                            const first=g.featureMask&1?0:1,second=g.featureMask&4?2:1;
                            for(let j=0;j<3;j++)edge[j]=g.triangleVertices[second*3+j]-g.triangleVertices[first*3+j];
                            const length=Math.hypot(...edge);for(let j=0;j<3;j++)edge[j]/=length;
                        }
                        for(let j=0;j<6;j++) {
                            chain.gradient[dofs[j]]+=weight*response.slope*J[j];
                            for(let l=0;l<6;l++) {
                                const curvature=g.feature==='face'||g.distance<1e-8?0:
                                    (j<3?1-t:t)*(l<3?1-t:t)*((j%3===l%3?1:0)-edge[j%3]*edge[l%3]-n[j%3]*n[l%3])/g.distance;
                                add(dofs[j],dofs[l],weight*(response.curvature*J[j]*J[l]+response.slope*curvature));
                            }
                        }
                    }
                }
            }
        }
        state.graftContactStats={contacts,maxPenetration};return energy;
    };
    sample.sharedAxisGeometryOnly=true;sample.sharedAxisDiscovery=true;sample.graftSurface=true;
    sample.surface=surface;
    sample.recovery=[...new Map(recovery.map(range=>[range.join('/'),range])).values()];
    return sample;
}
