import { Quaternion } from 'three';
import { quaternionExp } from './discreteKirchhoffRod.js';


const q=(b,e)=>new Quaternion(b.orientationX[e],b.orientationY[e],b.orientationZ[e],b.orientationW[e]);
// Preserve THREE.Vector3.applyQuaternion's arithmetic, without allocating a
// vector/array for each derivative column. The supplied frame is unit length.
function rotate3(vx,vy,vz,q,out) {
    const qx=q.x,qy=q.y,qz=q.z,qw=q.w;
    const tx=2*(qy*vz-qz*vy),ty=2*(qz*vx-qx*vz),tz=2*(qx*vy-qy*vx);
    out[0]=vx+qw*tx+qy*tz-qz*ty;out[1]=vy+qw*ty+qz*tx-qx*tz;out[2]=vz+qw*tz+qx*ty-qy*tx;
}
function rotationalInertiaInto(previous,current,weight,withTangent,out) {
    const r=out.relative.copy(previous).invert().multiply(current).normalize();
    if(r.w<0){r.x*=-1;r.y*=-1;r.z*=-1;r.w*=-1;}
    const n=Math.hypot(r.x,r.y,r.z),theta=2*Math.atan2(n,r.w),f=n>1e-12?theta/n:2,x=theta*theta;
    const {phi,torque,jacobian}=out;phi[0]=r.x*f;phi[1]=r.y*f;phi[2]=r.z*f;
    out.energy=.5*weight*x;for(let i=0;i<3;i++)torque[i]=weight*phi[i];
    if(!withTangent)return out;
    const a=x<.0625?1/12+x/720+x*x/30240+x**3/1209600+x**4/47900160:(1-theta/2/Math.tan(theta/2))/x;
    for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
        const skew=i===j?0:i===0?(j===1?-phi[2]:phi[1]):i===1?(j===0?phi[2]:-phi[0]):j===0?-phi[1]:phi[0];
        jacobian[i*3+j]=weight*((i===j?1:0)+.5*skew+a*(phi[i]*phi[j]-(i===j?x:0)));
    }
    return out;
}

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
    const H=chain.tangent??chain.hessian,g=chain.gradient,full=!!chain.tangent;
    const add=(i,j,value)=>{
        if(!withTangent)return;
        if(full)H[i*width+j-i+half]+=value;
        else if(i>=j)H[i*layout.band+i-j]+=value;
    };
    let energy=0;
    for(let node=0;node<s.positions.length;node++)for(let k=0;k<3;k++) {
        const w=masses[node]/dt**2,d=s.positions[node][k]-predicted[node][k],i=layout.positions[node]+k;
        energy+=.5*w*d*d;g[i]+=w*d;add(i,i,w);
    }
    // Scratch is local to one synchronous assembly and reused across all
    // material edges. It never becomes part of the physical checkpoint.
    const orientation=new Quaternion(),previous=new Quaternion(),potential={relative:new Quaternion(),
        phi:new Float64Array(3),torque:new Float64Array(3),jacobian:new Float64Array(9),energy:0};
    const dofs=new Int32Array(7),d1=new Float64Array(3),d2=new Float64Array(3),force=new Float64Array(3),omega=new Float64Array(3),dTorque=new Float64Array(3);
    for(let tool=0;tool<s.materials.length;tool++) {
        const {body,spec,last}=s.materials[tool],spins=layout.spins.get(spec.id);
        for(let e=0;e<last;e++) {
            orientation.set(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
            previous.fromArray(frames[tool][e].predicted);
            const inertia=(spec.mass??(spec.id==='catheter'?1.4:1))*body.restLength[e]/5;
            rotationalInertiaInto(previous,orientation,inertia/dt**2,withTangent,potential);
            const {torque,jacobian}=potential;energy+=potential.energy;
            rotate3(1,0,0,orientation,d1);rotate3(0,1,0,orientation,d2);
            const a=s.positions[e],b=s.positions[e+1],ex=b[0]-a[0],ey=b[1]-a[1],ez=b[2]-a[2],length=Math.hypot(ex,ey,ez);
            const tx=ex/length,ty=ey/length,tz=ez/length;
            for(let k=0;k<3;k++) {
                dofs[k]=layout.positions[e]+k;dofs[3+k]=layout.positions[e+1]+k;
                force[k]=(d1[k]*torque[1]-d2[k]*torque[0])/length;
                g[dofs[k]]-=force[k];g[dofs[3+k]]+=force[k];
            }
            dofs[6]=spins[e];g[dofs[6]]+=torque[2];
            if(!withTangent)continue;
            for(let col=0;col<7;col++) {
                const sign=col<3?-1:col<6?1:0,dx=col%3===0?sign:0,dy=col%3===1?sign:0,dz=col%3===2?sign:0;
                const a0=-(d2[0]*dx+d2[1]*dy+d2[2]*dz)/length,a1=(d1[0]*dx+d1[1]*dy+d1[2]*dz)/length,a2=col===6?1:0;
                for(let i=0;i<3;i++)dTorque[i]=jacobian[i*3]*a0+jacobian[i*3+1]*a1+jacobian[i*3+2]*a2;
                rotate3(a0,a1,a2,orientation,omega);
                const wx=omega[0],wy=omega[1],wz=omega[2],dl=tx*dx+ty*dy+tz*dz;
                for(let k=0;k<3;k++) {
                    const dd1=k===0?wy*d1[2]-wz*d1[1]:k===1?wz*d1[0]-wx*d1[2]:wx*d1[1]-wy*d1[0];
                    const dd2=k===0?wy*d2[2]-wz*d2[1]:k===1?wz*d2[0]-wx*d2[2]:wx*d2[1]-wy*d2[0];
                    const df=(dd1*torque[1]-dd2*torque[0]+d1[k]*dTorque[1]-d2[k]*dTorque[0])/length-force[k]*dl/length;
                    add(dofs[k],dofs[col],-df);add(dofs[3+k],dofs[col],df);
                }
                add(dofs[6],dofs[col],dTorque[2]);
            }
        }
    }
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
