// Independent pre-optimization reference for numerical equivalence tests.
import { Quaternion, Vector3 } from 'three';
import { quaternionExp } from '../../src/physics/discreteKirchhoffRod.js';


const q=(b,e)=>new Quaternion(b.orientationX[e],b.orientationY[e],b.orientationZ[e],b.orientationW[e]);
const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const rotate=(a,q)=>new Vector3(...a).applyQuaternion(q).toArray();

/** Exact isotropic rotational kinetic potential. Only the current frame is
 * variable, so the general two-frame constitutive Hessian is unnecessary. */
export function sharedAxisRotationalInertia(previous,current,weight,withTangent=true) {
    const r=previous.clone().invert().multiply(current).normalize();
    if(r.w<0){r.x*=-1;r.y*=-1;r.z*=-1;r.w*=-1;}
    const n=Math.hypot(r.x,r.y,r.z),theta=2*Math.atan2(n,r.w),f=n>1e-12?theta/n:2;
    const phi=[r.x*f,r.y*f,r.z*f],x=theta*theta;
    if(!withTangent)return {energy:.5*weight*x,torque:phi.map(v=>weight*v)};
    const a=x<.0625?1/12+x/720+x*x/30240+x**3/1209600+x**4/47900160:(1-theta/2/Math.tan(theta/2))/x;
    const skew=[0,-phi[2],phi[1],phi[2],0,-phi[0],-phi[1],phi[0],0],jacobian=new Float64Array(9);
    for(let i=0;i<3;i++)for(let j=0;j<3;j++)jacobian[i*3+j]=weight*((i===j?1:0)+.5*skew[i*3+j]+a*(phi[i]*phi[j]-(i===j?x:0)));
    return {energy:.5*weight*x,torque:phi.map(v=>weight*v),jacobian};
}

export function prepareSharedAxisDynamicStep(s,dt) {
    if(!(dt>0&&Number.isFinite(dt)))throw new RangeError('Positive dynamic timestep required');
    const decay=.98**(dt*120),positions=s.positions.map(p=>p.slice());
    const predicted=positions.map((p,i)=>p.map((v,k)=>v+dt*decay*(s.velocities?.[i]?.[k]??0)));
    const frames=s.materials.map(({body,spec,last})=>Array.from({length:last},(_,e)=>{
        const old=q(body,e),omega=s.angularVelocities?.[spec.id]?.[e]??[0,0,0];
        const increment=quaternionExp({x:omega[0]*dt*decay,y:omega[1]*dt*decay,z:omega[2]*dt*decay});
        return {old:old.toArray(),predicted:new Quaternion(increment.x,increment.y,increment.z,increment.w).multiply(old).toArray()};
    }));
    const masses=new Float64Array(s.positions.length);
    for(const {spec,last,body} of s.materials)for(let e=0;e<last;e++) {
        const mass=(spec.mass??(spec.id==='catheter'?1.4:1))*body.restLength[e]/5;
        const fraction=body.restLength[e]/(s.coordinates[e+1]-s.coordinates[e]);
        masses[e]+=(1-.5*fraction)*mass;masses[e+1]+=.5*fraction*mass;
    }
    s.dynamicStep={dt,positions,predicted,frames,masses};
}

/** Implicit kinetic potential, on the SAME shared positions and independent
 * material frames. Its gradient participates in the physical residual. */
export function assembleSharedAxisInertia(s,withTangent=true) {
    if(!s.dynamicStep)return 0;
    const {dt,predicted,frames,masses}=s.dynamicStep,{layout,chain}=s,half=layout.band-1,width=2*half+1;
    let energy=0;
    const add=(i,j,value)=>{
        if(!withTangent)return;
        if(chain.tangent)chain.tangent[i*width+j-i+half]+=value;
        else if(i>=j)chain.hessian[i*layout.band+i-j]+=value;
    };
    s.positions.forEach((p,n)=>p.forEach((v,k)=>{
        const w=masses[n]/dt**2,d=v-predicted[n][k],i=layout.positions[n]+k;
        energy+=.5*w*d*d;chain.gradient[i]+=w*d;add(i,i,w);
    }));
    s.materials.forEach(({body,spec,last},tool)=>{
        for(let e=0;e<last;e++) {
            const orientation=q(body,e),previous=new Quaternion(...frames[tool][e].predicted);
            const inertia=(spec.mass??(spec.id==='catheter'?1.4:1))*body.restLength[e]/5;
            const potential=sharedAxisRotationalInertia(previous,orientation,inertia/dt**2,withTangent);
            const torque=potential.torque;energy+=potential.energy;
            const d1=rotate([1,0,0],orientation),d2=rotate([0,1,0],orientation);
            const edge=s.positions[e+1].map((v,k)=>v-s.positions[e][k]),length=Math.hypot(...edge),t=edge.map(v=>v/length);
            const force=d1.map((v,k)=>(v*torque[1]-d2[k]*torque[0])/length);
            const dofs=[...Array.from({length:2},(_,j)=>[0,1,2].map(k=>layout.positions[e+j]+k)).flat(),layout.spins.get(spec.id)[e]];
            force.forEach((v,k)=>{chain.gradient[dofs[k]]-=v;chain.gradient[dofs[3+k]]+=v;});chain.gradient[dofs[6]]+=torque[2];
            if(!withTangent)continue;
            for(let col=0;col<7;col++) {
                const de=[0,0,0];if(col<6)de[col%3]=col<3?-1:1;
                const angular=[-dot(d2,de)/length,dot(d1,de)/length,col===6?1:0];
                const dTorque=torque.map((_,i)=>angular.reduce((v,a,j)=>v+potential.jacobian[i*3+j]*a,0));
                const omega=rotate(angular,orientation),dd1=cross(omega,d1),dd2=cross(omega,d2),dl=dot(t,de);
                for(let k=0;k<3;k++) {
                    const df=(dd1[k]*torque[1]-dd2[k]*torque[0]+d1[k]*dTorque[1]-d2[k]*dTorque[0])/length-force[k]*dl/length;
                    add(dofs[k],dofs[col],-df);add(dofs[3+k],dofs[col],df);
                }
                add(dofs[6],dofs[col],dTorque[2]);
            }
        }
    });
    return energy;
}

export function completeSharedAxisDynamicStep(s) {
    const {positions,frames,dt}=s.dynamicStep;
    s.velocities=s.positions.map((p,i)=>p.map((v,k)=>(v-positions[i][k])/dt));
    s.angularVelocities=Object.fromEntries(s.materials.map(({body,spec,last},tool)=>[spec.id,Array.from({length:last},(_,e)=>{
        const relative=q(body,e).multiply(new Quaternion(...frames[tool][e].old).invert()).normalize();
        if(relative.w<0){relative.x*=-1;relative.y*=-1;relative.z*=-1;relative.w*=-1;}
        const length=Math.hypot(relative.x,relative.y,relative.z),scale=length>1e-12?2*Math.atan2(length,relative.w)/(dt*length):2/dt;
        return [relative.x*scale,relative.y*scale,relative.z*scale];
    })]));
    s.dynamicStep=null;
}
