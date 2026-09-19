import {Quaternion,Vector3} from 'three';
import {sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';

const dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const add=(a,b)=>a.map((v,i)=>v+b[i]);
const scale=(a,k)=>a.map(v=>v*k);
const project=(a,n)=>add(a,scale(n,-dot(a,n)));
const quat=(body,e)=>new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
const rotate=(a,q)=>new Vector3(...a).applyQuaternion(q).toArray();
const point=(positions,e,t)=>positions[e].map((v,k)=>(1-t)*v+t*positions[e+1][k]);
const certificate=s=>[...s.positions.flat(),...s.multipliers,...s.materials.flatMap(({body,last})=>Array.from({length:last},(_,e)=>quat(body,e).toArray()).flat())];

/** Elastic sticking followed by a bounded kinetic return. By default a branch and normal
 * chart stay frozen throughout a global Newton solve. The fast Newton option
 * may refresh them between accepted iterations; each direction and its line
 * search still use one frozen chart. Live normal loads remain explicit duals.
 * This is a regularized Coulomb law: sticking admits traction / stiffness mm
 * of elastic motion. No additional contact unknowns or inter-tool rows exist.
 * A force-only evaluation returns hessian:null. Optional out is caller-owned
 * scratch; consumers must finish using its arrays before the next evaluation. */
export function sharedAxisFrictionPotential(slip,{stiffness,normalLoad,muStatic,muKinetic,mode='stick'},withTangent=true,out=null) {
    if(!(stiffness>0)||![stiffness,normalLoad,muStatic,muKinetic,...slip].every(Number.isFinite)||normalLoad<0||muKinetic<0||muStatic<muKinetic||!['stick','slide'].includes(mode))
        throw new RangeError('Invalid shared-axis wall friction law');
    const length=Math.hypot(...slip),limit=muKinetic*normalLoad;
    const result=out??{},traction=result.traction??=new Array(3);
    const hessian=withTangent?(result.hessian??=new Float64Array(9)):null;
    if(hessian)hessian.fill(0);
    result.hessian=hessian;result.mode=mode;
    let energy;
    if(normalLoad===0||muStatic===0){traction.fill(0);result.energy=0;return result;}
    if(mode==='stick'||stiffness*length<=limit) {
        energy=.5*stiffness*length**2;for(let i=0;i<3;i++)traction[i]=slip[i]*stiffness;
        if(hessian)for(let i=0;i<3;i++)hessian[i*3+i]=stiffness;
    } else {
        energy=limit*(length-.5*limit/stiffness);for(let i=0;i<3;i++)traction[i]=slip[i]*(limit/length);
        if(hessian)for(let i=0;i<3;i++)for(let j=0;j<3;j++)hessian[i*3+j]=limit/length*((i===j?1:0)-slip[i]*slip[j]/length**2);
    }
    result.energy=energy;return result;
}

/** Derivative of the frozen branch's traction with respect to normal load.
 * At zero load a sliding contact uses the positive-load one-sided derivative.
 * A frozen sticking branch with nonzero slip is discontinuous at zero load;
 * it needs the existing outer mode refresh, not an invented finite derivative.
 */
export function sharedAxisFrictionNormalDerivative(slip,{stiffness,normalLoad,muKinetic,mode}) {
    const length=Math.hypot(...slip);
    if(mode!=='slide'||!length||stiffness*length<=muKinetic*normalLoad)return [0,0,0];
    return scale(slip,muKinetic/length);
}

export function prepareSharedAxisWallFriction(s,{feedById={},stiffness=1e4,history=s.wallFrictionHistory??[],liveNormalLoad=false,lightweightFriction=false}={}) {
    if(!s.dynamicStep)throw new Error('Wall friction requires the prepared dynamic step');
    if(!(stiffness>0&&Number.isFinite(stiffness))||Object.values(feedById).some(v=>!Number.isFinite(v)))throw new RangeError('Invalid wall friction step');
    s.wallFrictionStep={lightweightFriction,dynamicStep:s.dynamicStep,feedById:{...feedById},stiffness,...(liveNormalLoad?{liveNormalLoad:true}:{}),
        history:new Map(history.map(r=>[r.id,{...r,elastic:r.elastic.slice()}])),records:[],certified:false};
    refreshSharedAxisWallFriction(s);
}

function makeRecord(s,def,index,old) {
    const step=s.wallFrictionStep,e=def.edge;
    // Only actual vessel witnesses. In particular: no sheath, fold, discovery,
    // or wire/catheter interaction rows can consume a friction budget.
    if(def.kind!=='wall'||!def.witness||!def.evaluate)return null;
    const material=sharedAxisOuterMaterialAt(s,e,def.witness.t,def.witness.owner);
    const muStatic=material.spec.wallStaticFriction??0,muKinetic=material.spec.wallKineticFriction??0;
    if(![muStatic,muKinetic].every(Number.isFinite)||muKinetic<0||muStatic<muKinetic)throw new RangeError('Static wall friction must contain the kinetic cone');
    if(!muStatic)return null;
    const normalLoad=Math.max(0,s.multipliers[index]);
    // Newly loaded witnesses enter through the certified outer refresh. Keeping
    // every dormant witness here would multiply the expensive surface work.
    if(!normalLoad)return null;
    const radius=material.body.radius,t=def.witness.t;
    const g=def.evaluate({state:s,a:s.positions[e],b:s.positions[e+1],edge:e,radius,owner:material.spec.id,coordinateA:s.coordinates[e],coordinateB:s.coordinates[e+1]});
    const n=[0,1,2].map(k=>g.jacobian[k]+g.jacobian[3+k]),norm=Math.hypot(...n);
    if(!(norm>1e-10))throw new RangeError('Loaded wall friction needs a surface normal');
    for(let k=0;k<3;k++)n[k]/=norm;
    const reference=step.dynamicStep.positions,edge=reference[e+1].map((v,k)=>v-reference[e][k]),length=Math.hypot(...edge),tangent=scale(edge,1/length);
    const tool=s.materials.indexOf(material),q0=new Quaternion(...step.dynamicStep.frames[tool][e].old),rho0=scale(n,-radius);
    const id=`${material.spec.id}/${def.id}`,siteCoordinate=(1-t)*s.coordinates[e]+t*s.coordinates[e+1];
    // A changed mesh interval must not erase an unchanged physical wall site.
    // Do not extrapolate a loaded history to an unrelated contact point.
    const history=step.history.get(id)??[...step.history.values()].find(h=>h.owner===material.spec.id&&h.face===def.witness.face&&Math.abs(h.siteCoordinate-siteCoordinate)<=1e-6);
    const record={id,siteCoordinate,face:def.witness.face,owner:material.spec.id,e,t,n,normalLoad,muStatic,muKinetic,stiffness:step.stiffness,material,
        ...(step.liveNormalLoad?{rowIndex:index}:{}),
        origin:point(reference,e,t),rho0,localRho:rotate(rho0,q0.clone().invert()),
        feed:scale(tangent,step.feedById[material.spec.id]??0),elastic:project(history?.elastic??[0,0,0],n),
        mode:old?.mode??history?.mode??'stick'};
    const trial=kinematics(s,record),demand=step.stiffness*Math.hypot(...trial.slip);
    if(record.mode==='stick'&&demand>muStatic*normalLoad)record.mode='slide';
    else if(record.mode==='slide'&&demand<=muKinetic*normalLoad)record.mode='stick';
    return record;
}

function kinematics(s,r,lightweight=s.wallFrictionStep?.lightweightFriction??false) {
    if(lightweight)return bufferedKinematics(s,r);

    const {e,t,n,material}=r,p=point(s.positions,e,t),rho=rotate(r.localRho,quat(material.body,e));
    const slip=project(add(r.elastic,add(r.feed,add(p.map((v,k)=>v-r.origin[k]),rho.map((v,k)=>v-r.rho0[k])))),n);
    const edge=s.positions[e+1].map((v,k)=>v-s.positions[e][k]),length=Math.hypot(...edge),tangent=scale(edge,1/length);
    const dofs=[0,1,2].map(k=>s.layout.positions[e]+k).concat([0,1,2].map(k=>s.layout.positions[e+1]+k),s.layout.spins.get(material.spec.id)[e]);
    const de=Array.from({length:7},(_,i)=>[0,1,2].map(k=>i<6&&i%3===k?(i<3?-1:1):0));
    const omega=de.map((v,i)=>add(scale(cross(tangent,v),1/length),scale(tangent,i===6?1:0)));
    const J=omega.map((w,i)=>project(add(cross(w,rho),[0,1,2].map(k=>i<6&&i%3===k?(i<3?1-t:t):0)),n));
    return {slip,rho,dofs,de,omega,J,length,tangent};
}
// Private per-state scratch, never included in replay/history. Each contact is
// scattered before evaluating the next one, so a single 7 x 7 block suffices.
const evaluationScratch=new WeakMap();
function workspace(s) {
    let scratch=evaluationScratch.get(s);
    if(!scratch) {
        const vectors=()=>Array.from({length:7},()=>[0,0,0]);
        scratch={HJ:new Float64Array(21),dRho:new Float64Array(21),dT:new Float64Array(21),tCrossDe:new Float64Array(21),lengthFactor:new Float64Array(7),H:new Float64Array(49),
            q:new Quaternion(),v:new Vector3(),point:[0,0,0],edge:[0,0,0],law:{},residualLaw:{},gradient:new Array(7),
            k:{slip:[0,0,0],rho:[0,0,0],dofs:new Array(7),tangent:[0,0,0],omega:vectors(),J:vectors(),
                de:Array.from({length:7},(_,i)=>[0,1,2].map(k=>i<6&&i%3===k?(i<3?-1:1):0))}};
        evaluationScratch.set(s,scratch);
    }
    return scratch;
}
function projectInPlace(a,n) {
    const along=-dot(a,n);
    for(let i=0;i<3;i++)a[i]=a[i]+n[i]*along;
}
// Preserve the reference expression order, including projection's initial
// zero in dot(). Buffers contain only numbers, never references to a contact,
// body or prior state, and every pose-dependent entry is overwritten.
function bufferedKinematics(s,r) {
    const scratch=workspace(s),k=scratch.k,{e,t,n,material}=r,{body}=material;
    const {point:p,edge}=scratch,{rho,slip,tangent,de,omega,J,dofs}=k;
    scratch.q.set(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
    scratch.v.set(...r.localRho).applyQuaternion(scratch.q).toArray(rho);
    for(let a=0;a<3;a++) {
        p[a]=(1-t)*s.positions[e][a]+t*s.positions[e+1][a];
        slip[a]=r.elastic[a]+(r.feed[a]+((p[a]-r.origin[a])+(rho[a]-r.rho0[a])));
        edge[a]=s.positions[e+1][a]-s.positions[e][a];
        dofs[a]=s.layout.positions[e]+a;dofs[3+a]=s.layout.positions[e+1]+a;
    }
    projectInPlace(slip,n);
    const length=k.length=Math.hypot(...edge),inverse=1/length;
    for(let a=0;a<3;a++)tangent[a]=edge[a]*inverse;
    dofs[6]=s.layout.spins.get(material.spec.id)[e];
    for(let i=0;i<7;i++) {
        const d=de[i],w=omega[i],j=J[i],spin=i===6?1:0;
        for(let a=0;a<3;a++) {
            const b=(a+1)%3,c=(a+2)%3;
            w[a]=(tangent[b]*d[c]-tangent[c]*d[b])*inverse+tangent[a]*spin;
        }
        for(let a=0;a<3;a++) {
            const b=(a+1)%3,c=(a+2)%3;
            j[a]=(w[b]*rho[c]-w[c]*rho[b])+(i<6&&i%3===a?(i<3?1-t:t):0);
        }
        projectInPlace(j,n);
    }
    return k;
}

function evaluate(s,r,full=false,lightweight=s.wallFrictionStep?.lightweightFriction??false) {
    const scratch=lightweight?workspace(s):null;
    const k=kinematics(s,r,lightweight),live=s.wallFrictionStep?.liveNormalLoad,
        rawLoad=live?s.multipliers[r.rowIndex]:r.normalLoad,
        parameters=live?{stiffness:r.stiffness,normalLoad:Math.max(0,rawLoad),muStatic:r.muStatic,muKinetic:r.muKinetic,mode:r.mode}:r,
        law=sharedAxisFrictionPotential(k.slip,parameters,full||!lightweight,scratch?.[full?'law':'residualLaw']),gradient=scratch?.gradient??new Array(7);
    for(let i=0;i<7;i++)gradient[i]=dot(k.J[i],law.traction);
    if(!full)return {...k,...law,gradient};
    const {HJ,dRho,dT,tCrossDe,lengthFactor,H}=scratch??workspace(s),{length,tangent,rho,de,omega,J}=k;
    const [tx,ty,tz]=tangent,[rx,ry,rz]=rho,[nx,ny,nz]=r.n,[fx,fy,fz]=law.traction,L2=length**2;
    const geometric=!!s.chain.tangent;
    for(let col=0;col<7;col++) {
        const j=J[col],offset=3*col;
        for(let a=0;a<3;a++)HJ[offset+a]=law.hessian[a*3]*j[0]+law.hessian[a*3+1]*j[1]+law.hessian[a*3+2]*j[2];
        if(!geometric)continue;
        const [dx,dy,dz]=de[col],[wx,wy,wz]=omega[col],dl=tx*dx+ty*dy+tz*dz;
        lengthFactor[col]=-dl/L2;
        dT[offset]=(dx-tx*dl)/length;dT[offset+1]=(dy-ty*dl)/length;dT[offset+2]=(dz-tz*dl)/length;
        tCrossDe[offset]=ty*dz-tz*dy;tCrossDe[offset+1]=tz*dx-tx*dz;tCrossDe[offset+2]=tx*dy-ty*dx;
        dRho[offset]=wy*rz-wz*ry;dRho[offset+1]=wz*rx-wx*rz;dRho[offset+2]=wx*ry-wy*rx;
    }
    for(let row=0;row<7;row++) {
        const j=J[row],dr=de[row],w=omega[row],at=3*row,spin=row===6?1:0;
        for(let col=0;col<7;col++) {
            const offset=3*col;
            let value=j[0]*HJ[offset]+j[1]*HJ[offset+1]+j[2]*HJ[offset+2];
            if(geometric) {
                const ax=dT[offset],ay=dT[offset+1],az=dT[offset+2],factor=lengthFactor[col];
                const ux=(ay*dr[2]-az*dr[1])/length+tCrossDe[at]*factor+ax*spin;
                const uy=(az*dr[0]-ax*dr[2])/length+tCrossDe[at+1]*factor+ay*spin;
                const uz=(ax*dr[1]-ay*dr[0])/length+tCrossDe[at+2]*factor+az*spin;
                const bx=dRho[offset],by=dRho[offset+1],bz=dRho[offset+2];
                const vx=uy*rz-uz*ry+w[1]*bz-w[2]*by;
                const vy=uz*rx-ux*rz+w[2]*bx-w[0]*bz;
                const vz=ux*ry-uy*rx+w[0]*by-w[1]*bx;
                const along=vx*nx+vy*ny+vz*nz;
                value+=fx*(vx-nx*along)+fy*(vy-ny*along)+fz*(vz-nz*along);
            }
            H[row*7+col]=value;
        }
    }
    const normalForceDerivative=live&&rawLoad>=0?sharedAxisFrictionNormalDerivative(k.slip,parameters):null;
    return {...k,...law,gradient,H,...(live?{normalForceDerivative:normalForceDerivative?k.J.map(j=>dot(j,normalForceDerivative)):new Array(7).fill(0)}:{})};
}

/** Refresh after a converged global solve, or between accepted iterations
 * in fast Newton. Never refresh during a direction or its line search. Re-solve
 * globally until the change of assembled friction force is below tolerance.
 * This certifies the lagged normal load and stick/slide decisions together.
 * With liveNormalLoad, both old and refreshed charts use the CURRENT lambda:
 * forceChange then measures normal/chart/mode changes only. The global force
 * certificate must already include the live-load friction and its dual column.
 */
export function refreshSharedAxisWallFriction(s,{forceTolerance=1e-6}={}) {
    const step=s.wallFrictionStep;if(!step)return {converged:true,forceChange:0,contacts:0};
    if(step.dynamicStep!==s.dynamicStep)throw new Error('Stale wall friction step');
    const old=new Map(step.records.map(r=>[r.id,r])),before=new Float64Array(s.layout.dofCount),after=before.slice();
    for(const r of step.records){const v=evaluate(s,r);v.dofs.forEach((d,i)=>before[d]+=v.gradient[i]);}
    const records=[];
    s.definitions.forEach((def,i)=>{const owner=sharedAxisOuterMaterialAt(s,def.edge,def.witness?.t??1,def.witness?.owner);
        const r=makeRecord(s,def,i,old.get(`${owner?.spec.id}/${def.id}`));if(r)records.push(r);});
    for(const r of records){const v=evaluate(s,r);v.dofs.forEach((d,i)=>after[d]+=v.gradient[i]);}
    let forceChange=0;for(let i=0;i<after.length;i++)if(!s.fixed[i])forceChange=Math.max(forceChange,Math.abs(after[i]-before[i]));
    step.records=records;step.certified=forceChange<=forceTolerance;
    step.certificate=step.certified?certificate(s):null;
    return {converged:step.certified,forceChange,contacts:records.length,sliding:records.filter(r=>r.mode==='slide').length};
}

export function assembleSharedAxisWallFriction(s,withTangent=true,lightweight=s.wallFrictionStep?.lightweightFriction??false) {
    const step=s.wallFrictionStep;if(!step)return 0;
    if(step.dynamicStep!==s.dynamicStep)throw new Error('Stale wall friction step');
    step.certified=false;step.certificate=null;
    // Derivative of generalized friction force with respect to the existing
    // normal reaction. These columns are not transposed into the gap equation:
    // Coulomb friction is a nonsymmetric force law, not a dual potential.
    if(step.liveNormalLoad)step.normalForceColumns=[];
    const {layout,chain}=s,half=layout.band-1,width=2*half+1;let energy=0;
    for(const r of step.records) {
        const v=evaluate(s,r,withTangent,lightweight);energy+=v.energy;
        if(step.liveNormalLoad&&withTangent&&v.normalForceDerivative.some(v=>v!==0))
            step.normalForceColumns.push({rowIndex:r.rowIndex,dofs:v.dofs.slice(),values:v.normalForceDerivative});
        for(let i=0;i<7;i++) {
            const a=v.dofs[i];chain.gradient[a]+=v.gradient[i];
            if(withTangent)for(let j=0;j<7;j++){const b=v.dofs[j];
                if(chain.tangent)chain.tangent[a*width+b-a+half]+=v.H[i*7+j];
                else if(a>=b)chain.hessian[a*layout.band+a-b]+=v.H[i*7+j];}
        }
    }
    return energy;
}

export function commitSharedAxisWallFriction(s) {
    const step=s.wallFrictionStep;if(!step)return;
    if(!step.certified||certificate(s).some((v,i)=>v!==step.certificate[i]))throw new Error('Wall friction requires a converged force refresh before commit');
    s.wallFrictionHistory=step.records.map(r=>{
        const v=evaluate(s,r);return {id:r.id,siteCoordinate:r.siteCoordinate,owner:r.owner,face:r.face,elastic:scale(v.traction,1/r.stiffness),mode:r.mode};
    });
    s.wallFrictionStep=null;
}


/** JSON-safe replay of accepted history and the frozen trial law. Restore the
 * dynamic step first: the friction chart is bound to that exact preparation. */
export function captureSharedAxisWallFriction(s) {
    const step=s.wallFrictionStep;
    return JSON.parse(JSON.stringify({history:s.wallFrictionHistory??[],step:step?{
        feedById:step.feedById,stiffness:step.stiffness,history:[...step.history.values()],...(step.liveNormalLoad?{liveNormalLoad:true}:{}),
        records:step.records.map(({material,...r})=>({...r,materialId:material.spec.id})),
        certified:step.certified,certificate:step.certificate??null}:null}));
}
export function restoreSharedAxisWallFriction(s,saved) {
    const copy=JSON.parse(JSON.stringify(saved));s.wallFrictionHistory=copy.history??[];
    if(!copy.step){s.wallFrictionStep=null;return;}
    if(!s.dynamicStep)throw new Error('Restore the dynamic step before wall friction');
    const step=copy.step;s.wallFrictionStep={...step,lightweightFriction:s.wallFrictionStep?.lightweightFriction??false,dynamicStep:s.dynamicStep,
        history:new Map(step.history.map(r=>[r.id,r])),records:step.records.map(({materialId,...r})=>{
            const material=s.materials.find(m=>m.spec.id===materialId);
            if(!material)throw new Error('Friction replay material is absent');return {...r,material};
        })};
}
