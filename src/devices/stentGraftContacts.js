import {graftBranchContactAt,branchRecoveryPotential} from './stentGraftBranchContact.js';
import {compliantGraftSurface} from './stentGraftCompliance.js';
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
    const compliant=compliantGraftSurface(surface,previous);surface=compliant.surface;
    // Never capture the previous native state: its wall sampler captures its
    // predecessor, retaining every band matrix and WASM arena indefinitely.
    const reference={coordinates:[...previous.coordinates],positions:previous.positions.map(p=>[...p]),origin:[...previous.origin]};
    const branchCache=new Map();
    const branchReference=(coordinate,radius)=>{
        if(!surface.ownedBranches?.length)return null;
        const key=`${coordinate}/${radius}`;
        if(!branchCache.has(key)) {
            const old=referencePoint(reference,coordinate).toArray();
            const contact=graftBranchContactAt(surface.ownedBranches,old,radius);
            if(contact) {
                const planeGap=old.reduce((sum,v,k)=>sum+(v-contact.point[k])*contact.planeNormal[k],0)-radius-.05;
                // Opening is kinematic. Recover a newly displaced septum in
                // bounded increments rather than asking one Newton solve to
                // move the wire across the full distance between both limbs.
                contact.recoveryOffset=Math.max(0,-planeGap-.5);
            }
            branchCache.set(key,contact);
        }
        return branchCache.get(key);
    };
    const bounds=surface.bounds.clone().expandByScalar(3),point=new THREE.Vector3(),ray=new THREE.Ray();
    for(const branch of surface.ownedBranches??[])for(let i=0;i<branch.positions.length;i+=3)
        bounds.expandByPoint(new THREE.Vector3().fromArray(branch.positions,i));
    const scratch=createKirchhoffWallWitnessGeometryWorkspace(),closest=new THREE.Vector3();
    const segmentContact=createSharedAxisSegmentContact(surface.geometry);
    const inherited=previous.graftRevision===surface.revision?previous.graftRecovery??[]:null;
    const recovery=[];
    if(previous.materials)for(let e=0;e<reference.positions.length-1;e++) {
        const start=reference.coordinates[e],end=reference.coordinates[e+1];
        const a=referencePoint(reference,start),b=referencePoint(reference,end);
        if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([a,b])))continue;
        const radius=Math.max(...sharedAxisOuterIntervals(previous,e).map(i=>i.material.body.radius));
        const hit=segmentContact.query(a.toArray(),b.toArray(),radius);
        // The last accepted state may still intersect incoming cloth while
        // one-sided branch recovery hands over to ordinary contact. Retain
        // that overlap even if its old recovery interval had expired.
        if(hit.face>=0) {
            if(hit.crossing)recovery.push([start,end]);
            if(inherited)recovery.push(...inherited.filter(([a,b])=>a<=end&&b>=start));
            else recovery.push([start-1,end+1]);
        }
    }
    function crossing(a,b,geometry=surface.geometry) {
        const length=a.distanceTo(b);if(length<1e-8)return null;
        ray.origin.copy(a);ray.direction.copy(b).sub(a).divideScalar(length);
        return geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,length);
    }
    const sample=({state,a,b,radius,coordinateA,coordinateB})=>{
        const wa=new THREE.Vector3(...a).add(new THREE.Vector3(...state.origin)),wb=new THREE.Vector3(...b).add(new THREE.Vector3(...state.origin));
        const oldA=referencePoint(reference,coordinateA),oldB=referencePoint(reference,coordinateB);
        // The owning wire may initially lie outside a newly opened target ring.
        // Let the one-sided lumen potential pull it inward through that incoming
        // surface. Within a transitioning branch, use its actual triangles
        // instead of the mapped Boolean cut. Deployed cloth retains normal CCD.
        const mid=(coordinateA+coordinateB)/2;
        const branches=[coordinateA,mid,coordinateB].map(s=>branchReference(s,radius));
        if(branches.some(contact=>contact?.outside))return inactive();
        const cloth=branches.find(contact=>contact)?.geometry??surface.geometry;
        if([coordinateA,mid,coordinateB].some(s=>graftLumenAt(surface.lumenSections,s,referencePoint(reference,s).toArray(),radius)?.penetration>0))return inactive();
        if(!bounds.intersectsBox(new THREE.Box3().setFromPoints([wa,wb,oldA,oldB])))return inactive();
        // A kinematic release can initially overlap an existing tool. Let the
        // elastic potential resolve that incoming overlap; CCD must not freeze
        // its withdrawal. As soon as clear, subsequent crossings are forbidden.
        if(recovery.some(([a,b])=>a<=coordinateB&&b>=coordinateA)&&
            segmentContact.query(oldA.toArray(),oldB.toArray(),radius).face>=0)return inactive();
        const reject=(message,hit)=>Object.assign(new Error(message),{code:'trial-outside-vessel',contact:{
            kind:'stent-graft',face:hit.faceIndex,point:hit.point.toArray(),radius,coordinateA,coordinateB,
            a:wa.toArray(),b:wb.toArray(),referenceA:oldA.toArray(),referenceB:oldB.toArray()}});
        // A newly revealed branch can already overlap an accepted segment.
        // Apply the same recovery rule to the actual branch triangles used
        // by the force law, not only to the mapped union boundary.
        if(cloth!==surface.geometry&&crossing(oldA,oldB,cloth))return inactive();
        const hit=crossing(wa,wb,cloth);
        if(hit)throw reject('Tool segment crossed stent-graft fabric',hit);
        const count=Math.max(1,Math.ceil(wa.distanceTo(wb)/.8));
        for(let i=0;i<=count;i++) {
            const t=i/count;point.copy(wa).lerp(wb,t);
            const old=referencePoint(reference,coordinateA+(coordinateB-coordinateA)*t);
            const hit=crossing(old,point,cloth);
            if(hit)throw reject('Tool trial crossed stent-graft fabric',hit);
        }
        return inactive();
    };
    // Do not reference `previous` from even a temporary callback here.
    // V8 shares this function's closure context with the returned sampler;
    // capturing it in Array.map retained every previous state and WASM arena.
    const referenceRadii=[];
    if(previous.materials)for(let e=0;e<reference.positions.length-1;e++)
        referenceRadii.push(Math.max(...sharedAxisOuterIntervals(previous,e).map(i=>i.material.body.radius)));
    // Used only for near-cloth bends below the 0.01 mm retention budget.
    // A tiny simplification must still never cut through a thin sheet.
    sample.meshChordCrosses=(a,b)=>!!crossing(
        new THREE.Vector3(...a).add(new THREE.Vector3(...reference.origin)),
        new THREE.Vector3(...b).add(new THREE.Vector3(...reference.origin)));
    const knotCache=new Map();
    sample.adaptiveContactKnots=options=>{
        const margin=Math.max(options.contactMargin,options.shapeTolerance),key=margin;
        if(knotCache.has(key))return knotCache.get(key);
        const knots=new Set();
        for(let e=0;e<referenceRadii.length;e++) {
            const a=referencePoint(reference,reference.coordinates[e]),b=referencePoint(reference,reference.coordinates[e+1]);
            const radius=referenceRadii[e];
            if(segmentContact.query(a.toArray(),b.toArray(),radius+margin).face<0)continue;
            for(let n=Math.max(0,e-1);n<=Math.min(reference.coordinates.length-1,e+2);n++)knots.add(reference.coordinates[n]);
        }
        knotCache.set(key,knots);return knots;
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
                const count=Math.max(1,Math.ceil(length/(surface.lumenSections?.length ? .8 : .15)));
                // A segment straddling an open portal must use the same cloth
                // for its rim forces and CCD, including samples just beyond
                // the portal plane. Otherwise CCD sees a wall with no force.
                const span=state.coordinates[e+1]-state.coordinates[e];
                const branchGeometry=[0,.5,1]
                    .map(t=>branchReference(state.coordinates[e]+t*span,radius)).find(Boolean)?.geometry;
                for(let i=0;i<=count;i++) {
                    const t=interval.start+(interval.end-interval.start)*i/count;point.copy(a).lerp(b,t);
                    const coordinate=state.coordinates[e]+t*(state.coordinates[e+1]-state.coordinates[e]);
                    const oldBranch=branchReference(coordinate,radius);
                    if(oldBranch?.outside) {
                        const response=branchRecoveryPotential(oldBranch,point.toArray(),radius+.05);
                        // Match the normal cloth stiffness: the old weak
                        // recovery spring let the stiff delivery shaft remain
                        // embedded in the divider even after full deployment.
                        const k=1e5*length/count*((i===0||i===count)?.5:1);
                        energy+=k*response.energy;if(response.energy>0)contacts++;
                        for(let j=0;j<6;j++) {
                            const wj=j<3?1-t:t;
                            chain.gradient[dofs[j]]+=k*wj*response.gradient[j%3];
                            for(let l=0;l<6;l++)add(dofs[j],dofs[l],k*wj*(l<3?1-t:t)*response.hessian[j%3][l%3]);
                        }
                        // This branch uses its actual exposed triangles. Do
                        // not also apply the mapped Boolean junction here: its
                        // cut faces can cross the partially folded lumen.
                        // Inside clearance the potential is zero, with no
                        // centreline tether or axial friction constraint.
                        continue;
                    }
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
                    const contactGeometry=oldBranch?.geometry??branchGeometry??surface.geometry;
                    const faces=[];
                    contactGeometry.boundsTree.shapecast({
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
                        const g=evaluateKirchhoffWallWitnessGeometry({geometry:contactGeometry,faceIndex,point:point.toArray()},scratch);
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
        // Restoring force of the lifted cloth; the indented surface above still
        // supplies finite-radius contact and CCD. This reduced spring prevents
        // the local opening from being a force-free hole in the graft.
        for(const patch of compliant.patches) {
            const s=patch.coordinate;if(!Number.isFinite(s)||s>state.coordinates.at(-1))continue;
            let e=0;while(e<state.coordinates.length-2&&state.coordinates[e+1]<s)e++;
            const t=(s-state.coordinates[e])/(state.coordinates[e+1]-state.coordinates[e]);
            const n=patch.normal,p=state.positions[e].map((v,k)=>v*(1-t)+state.positions[e+1][k]*t+state.origin[k]);
            const gap=p.reduce((sum,v,k)=>sum+(v-patch.point[k])*n[k],0)-patch.radius;
            if(gap>=0)continue;
            const k=180,J=[...n.map(v=>v*(1-t)),...n.map(v=>v*t)];
            const dofs=[layout.positions[e],layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]);
            energy+=.5*k*gap*gap;
            for(let j=0;j<6;j++){chain.gradient[dofs[j]]+=k*gap*J[j];for(let l=0;l<6;l++)add(dofs[j],dofs[l],k*J[j]*J[l]);}
        }
        state.graftContactStats={contacts,maxPenetration};return energy;
    };
    sample.sharedAxisGeometryOnly=true;sample.sharedAxisDiscovery=true;sample.graftSurface=true;
    sample.surface=surface;
    sample.contactPatches=compliant.patches;
    sample.recovery=[...new Map(recovery.map(range=>[range.join('/'),range])).values()];
    return sample;
}
