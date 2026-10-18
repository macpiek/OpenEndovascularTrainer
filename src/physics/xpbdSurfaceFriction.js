import {Quaternion,Vector3} from 'three';
import {sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';
import {sharedAxisFrictionPotential} from './kirchhoffSharedAxisWallFriction.js';

const dot=(a,b)=>a.reduce((v,x,k)=>v+x*b[k],0);
const scale=(a,s)=>a.map(v=>v*s);
const project=(a,n)=>a.map((v,k)=>v-dot(a,n)*n[k]);
const rotate=(a,q)=>new Vector3(...a).applyQuaternion(q).toArray();
const point=(positions,e,t)=>positions[e].map((v,k)=>(1-t)*v+t*positions[e+1][k]);

// Research adapter for the reference regularized Coulomb law. Unlike the
// original local XPBD disk, this preserves elastic slip, the contact radius,
// material rotation and the feed direction of the incoming configuration.
// Charts, modes and effective stiffness are frozen during each global trial.
export function createXpbdSurfaceFriction(s,frames,initial,dt,feedById,{stiffness=1e4}={}) {
    const history=s.wallFrictionHistory??[],byId=new Map(history.map(h=>[h.id,h])),modes=new Map();
    let records=[];
    const kinematics=r=>{
        const rho=rotate(r.localRho,frames[r.materialIndex][r.edge].q),p=point(s.positions,r.edge,r.t);
        const slip=project(r.elastic.map((v,k)=>v+r.feed[k]+p[k]-r.origin[k]+rho[k]-r.rho0[k]),r.n);
        return {rho,slip};
    };
    const refresh=(rows,normalLambda)=>{
        records=[];
        rows.forEach((row,index)=>{
            const def=s.definitions[index];
            if(row.kind!=='wall'||!def?.witness||!def.evaluate||!(normalLambda[index]>0))return;
            const {edge}=row,{t,face,owner}=def.witness;
            const material=sharedAxisOuterMaterialAt(s,edge,t,owner),{spec,body}=material;
            const muStatic=spec.wallStaticFriction??0,muKinetic=spec.wallKineticFriction??0;
            if(!muStatic)return;
            const materialIndex=s.materials.findIndex(m=>m.spec.id===spec.id);
            const normal=row.jacobian.slice(0,3).map((v,k)=>v+row.jacobian[3+k]),length=Math.hypot(...normal);
            if(!(length>1e-10))throw new Error('XPBD friction requires a surface normal');
            const n=scale(normal,1/length),id=`${spec.id}/${def.id}`,siteCoordinate=(1-t)*s.coordinates[edge]+t*s.coordinates[edge+1];
            const old=byId.get(id)??history.find(h=>h.owner===spec.id&&h.face===face&&Math.abs(h.siteCoordinate-siteCoordinate)<=1e-6);
            const delta=initial.positions[edge+1].map((v,k)=>v-initial.positions[edge][k]),tangent=scale(delta,1/Math.hypot(...delta));
            const rho0=scale(n,-body.radius),q0=new Quaternion(...initial.frames[materialIndex][edge].old);
            let u=project(tangent,n);
            if(Math.hypot(...u)<1e-8)u=project(Math.abs(n[0])<.8?[1,0,0]:[0,1,0],n);
            u=scale(u,1/Math.hypot(...u));const v=new Vector3(...n).cross(new Vector3(...u)).toArray();
            const r={id,owner:spec.id,face,siteCoordinate,materialIndex,edge,t,n,basis:[u,v],rho0,
                localRho:rotate(rho0,q0.invert()),origin:point(initial.positions,edge,t),
                feed:scale(tangent,feedById[spec.id]??0),elastic:project(old?.elastic??[0,0,0],n),
                stiffness,muStatic,muKinetic,normalLoad:normalLambda[index]/(dt*dt),mode:modes.get(id)??old?.mode??'stick'};
            const demand=stiffness*Math.hypot(...kinematics(r).slip);
            if(r.mode==='stick'&&demand>muStatic*r.normalLoad)r.mode='slide';
            else if(r.mode==='slide'&&demand<=muKinetic*r.normalLoad)r.mode='stick';
            modes.set(id,r.mode);
            // Iteratively reweighted compliance has the same fixed-point
            // traction as the reference law. It is not a Newton certificate.
            r.effectiveStiffness=r.mode==='slide'&&demand>muKinetic*r.normalLoad?
                stiffness*muKinetic*r.normalLoad/demand:stiffness;
            if(r.effectiveStiffness>0)records.push(r);
        });
    };
    return {
        refresh,
        constraints(frameDofs,impulses) {
            return records.flatMap(r=>{
                const {rho,slip}=kinematics(r),d=frameDofs[r.materialIndex][r.edge];
                const dofs=[3*r.edge,3*r.edge+1,3*r.edge+2,3*(r.edge+1),3*(r.edge+1)+1,3*(r.edge+1)+2,d,d+1,d+2];
                return r.basis.map((tangent,k)=>({id:`friction/${r.id}/${k}`,frictionId:r.id,tangent,
                    edge:r.edge,dofs,jacobian:[...scale(tangent,1-r.t),...scale(tangent,r.t),...new Vector3(...rho).cross(new Vector3(...tangent)).toArray()],
                    gap:dot(slip,tangent),alpha:1/(r.effectiveStiffness*dt*dt),lambda:dot(impulses.get(r.id)??[0,0,0],tangent)}));
            });
        },
        history() {
            return records.map(r=>({id:r.id,siteCoordinate:r.siteCoordinate,owner:r.owner,face:r.face,mode:r.mode,
                elastic:scale(sharedAxisFrictionPotential(kinematics(r).slip,r,false).traction,1/stiffness)}));
        }
    };
}
