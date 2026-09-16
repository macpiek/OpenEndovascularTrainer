import { evaluateBendTwistLocalConstraintNormalized, quaternionExp } from './discreteKirchhoffRod.js';
import {sharedAxisMaterialKernelWorkspace} from './kirchhoffSharedAxisMaterialKernel.js';

const dot = (a,b) => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const tmul = (A,v) => [A[0]*v[0]+A[3]*v[1]+A[6]*v[2],A[1]*v[0]+A[4]*v[1]+A[7]*v[2],A[2]*v[0]+A[5]*v[1]+A[8]*v[2]];
function matrix(q,out=new Float64Array(9),offset=0) {
    const {x,y,z,w}=q;
    out[offset]=1-2*(y*y+z*z);out[offset+1]=2*(x*y-w*z);out[offset+2]=2*(x*z+w*y);
    out[offset+3]=2*(x*y+w*z);out[offset+4]=1-2*(x*x+z*z);out[offset+5]=2*(y*z-w*x);
    out[offset+6]=2*(x*z-w*y);out[offset+7]=2*(y*z+w*x);out[offset+8]=1-2*(x*x+y*y);
    return out;
}
const readQ=(b,e)=>({x:b.orientationX[e],y:b.orientationY[e],z:b.orientationZ[e],w:b.orientationW[e]});
function coefficient(x) {
    if(x<.0625)return [1/12+x/720+x*x/30240+x**3/1209600+x**4/47900160,
        1/720+2*x/30240+3*x*x/1209600+4*x**3/47900160];
    const theta=Math.sqrt(x),h=theta/2,cot=Math.cos(h)/Math.sin(h),n=1-h*cot;
    return [n/x,(-cot/(4*theta)+1/(8*Math.sin(h)**2))/x-n/(x*x)];
}

/** Analytic derivative of native local material torques. Unlike J^T K J,
 * includes the derivative of the logarithm Jacobian and moving local frames.
 * This differentiates the existing native constitutive law, not a new energy.
 */
export function nativeHingeTorqueTangent(q0,q1,rest,compliance,withTangent=true,preparation=null,kernelRecord=null) {
    const cached=preparation?.reuse;
    const state=cached?null:evaluateBendTwistLocalConstraintNormalized(q0,q1,rest,{});
    const phi=cached?.phi??[state.strain.x,state.strain.y,state.strain.z],A=cached?.A??state.localGradient,R=cached?.R??matrix(state.relative);
    const p=cached?.p??phi.map((v,i)=>v/compliance[i]),u=cached?.u??tmul(A,p);
    const torque=cached?.torque??[-u[0],-u[1],-u[2],...tmul(R,u)];
    if(preparation?.store)preparation.value={phi,A,R,p,u,torque};
    if(!withTangent)return {energy:.5*dot(phi,p),torque};
    const restR=matrix(quaternionExp(rest));
    const x=dot(phi,phi),[a,ap]=coefficient(x);
    if(kernelRecord) {
        kernelRecord.set(phi,44);kernelRecord.set(A,47);kernelRecord.set(R,56);
        kernelRecord.set(p,65);kernelRecord.set(u,68);kernelRecord.set(torque,71);
        kernelRecord.set(restR,77);kernelRecord.set(compliance,86);kernelRecord[89]=a;kernelRecord[90]=ap;
        return {energy:.5*dot(phi,p),torque};
    }
    const jacobian=new Float64Array(36);
    const dphi=new Float64Array(3),dp=new Float64Array(3),dJ=new Float64Array(9),dA=new Float64Array(9),du=new Float64Array(3),spun=new Float64Array(3);
    for(let col=0;col<6;col++) {
        const axis=col%3,dx=col<3?(axis===0?-1:0):R[axis],dy=col<3?(axis===1?-1:0):R[3+axis],dz=col<3?(axis===2?-1:0):R[6+axis];
        for(let i=0;i<3;i++){dphi[i]=A[i*3]*dx+A[i*3+1]*dy+A[i*3+2]*dz;dp[i]=dphi[i]/compliance[i];}
        const dphiDot=dot(phi,dphi);
        for(let i=0;i<3;i++)for(let j=0;j<3;j++) {
            const skew=i===j?0:i===0?(j===1?-dphi[2]:dphi[1]):i===1?(j===0?dphi[2]:-dphi[0]):j===0?-dphi[1]:dphi[0];
            dJ[i*3+j]=-.5*skew+2*ap*dphiDot*(phi[i]*phi[j]-(i===j?x:0))+
                a*(dphi[i]*phi[j]+phi[i]*dphi[j]-(i===j?2*dphiDot:0));
        }
        dA.fill(0);
        for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)dA[i*3+j]+=dJ[i*3+k]*restR[j*3+k];
        for(let i=0;i<3;i++)du[i]=(dA[i]*p[0]+dA[3+i]*p[1]+dA[6+i]*p[2])+(A[i]*dp[0]+A[3+i]*dp[1]+A[6+i]*dp[2]);
        // d(R^T u) = R^T(du + v0 x u) - v1 x (R^T u).
        for(let i=0;i<3;i++) {
            const j=(i+1)%3,k=(i+2)%3;
            spun[i]=du[i]+(col<3?((axis===j?u[k]:0)-(axis===k?u[j]:0)):0);
        }
        for(let i=0;i<3;i++) {
            const j=(i+1)%3,k=(i+2)%3,spin=col>=3?((axis===j?torque[3+k]:0)-(axis===k?torque[3+j]:0)):0;
            jacobian[i*6+col]=-du[i];jacobian[(i+3)*6+col]=R[i]*spun[0]+R[3+i]*spun[1]+R[6+i]*spun[2]-spin;
        }
    }
    return {energy:.5*dot(phi,p),torque,jacobian};
}

/** Scatter the exact material residual derivative into the shared spatial
 * band. Each hinge touches three positions and two material spins only.
 * Off equilibrium this derivative need not be symmetric: spin is measured in
 * a moving material frame. Preserve both triangles for the Newton solve.
 */
export function assembleSharedAxisMaterialTangent(s,withTangent=true,promotion=null,wasmMaterial=false) {
    const {layout,chain}=s,n=layout.dofCount,width=2*layout.band-1,half=layout.band-1;
    const kernel=withTangent&&wasmMaterial?sharedAxisMaterialKernelWorkspace(chain,s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0)):null;
    const H=kernel?(chain.tangent=kernel.tangent):(chain.tangent??=new Float64Array(n*width)),g=chain.gradient;
    H.fill(0);for(let i=0;i<n;i++)g[i]=-s.loads[i];
    // Scratch belongs to this synchronous assembly call. Every material hinge
    // reuses it; no temporary vectors or columns are created in the 11-column
    // derivative loop. Both triangles of the moving-frame tangent are kept.
    const dofs=new Int32Array(11),frames=new Float64Array(18),d1=new Float64Array(6),d2=new Float64Array(6);
    const lengths=new Float64Array(2),tangents=new Float64Array(6),forces=new Float64Array(6);
    const angular=withTangent&&!kernel?new Float64Array(66):null,dLengths=withTangent&&!kernel?new Float64Array(22):null;
    const dTorque=withTangent&&!kernel?new Float64Array(6):null,column=withTangent&&!kernel?new Float64Array(11):null;
    let energy=0,hinge=0;
    for(const {body,spec,last} of s.materials) {
        const spins=layout.spins.get(spec.id);
        for(let e=1;e<last;e++) {
            for(let node=0;node<3;node++)for(let a=0;a<3;a++)dofs[node*3+a]=layout.positions[e-1+node]+a;
            dofs[9]=spins[e-1];dofs[10]=spins[e];
            const saved=promotion?.reuse?.[hinge],q0=readQ(body,e-1),q1=readQ(body,e);
            if(saved) {
                frames.set(saved.geometry.subarray(0,18));d1.set(saved.geometry.subarray(18,24));d2.set(saved.geometry.subarray(24,30));
                lengths.set(saved.geometry.subarray(30,32));tangents.set(saved.geometry.subarray(32,38));forces.set(saved.geometry.subarray(38,44));
            } else {
                matrix(q0,frames,0);matrix(q1,frames,9);
                for(let j=0;j<2;j++) {
                    const at=3*j,m=9*j,a=s.positions[e-1+j],b=s.positions[e+j];
                    const dx=b[0]-a[0],dy=b[1]-a[1],dz=b[2]-a[2],length=Math.sqrt(dx*dx+dy*dy+dz*dz);
                    lengths[j]=length;tangents[at]=dx/length;tangents[at+1]=dy/length;tangents[at+2]=dz/length;
                    for(let k=0;k<3;k++){d1[at+k]=frames[m+3*k];d2[at+k]=frames[m+3*k+1];}
                }
            }
            const preparation=saved?{reuse:saved.material}:promotion?.capture?{store:true}:null;
            const packed=kernel?.records[hinge];
            const material=nativeHingeTorqueTangent(q0,q1,{x:body.restRotation1[e],y:body.restRotation2[e],z:body.restRotation3[e]},
                [body.kirchhoffBendCompliance1[e],body.kirchhoffBendCompliance2[e],body.kirchhoffTwistCompliance[e]],withTangent,preparation,packed?.data);
            energy+=material.energy;
            for(let j=0;j<2;j++) {
                const at=3*j;
                for(let a=0;a<3;a++) {
                    const force=saved?forces[at+a]:(-d2[at+a]*material.torque[at]+d1[at+a]*material.torque[at+1])/lengths[j];
                    forces[at+a]=force;g[dofs[at+a]]-=force;g[dofs[at+3+a]]+=force;
                }
                g[dofs[9+j]]+=material.torque[at+2];
            }
            if(promotion?.capture) {
                const entry=promotion.capture[hinge]??={geometry:new Float64Array(44)};
                entry.geometry.set(frames,0);entry.geometry.set(d1,18);entry.geometry.set(d2,24);
                entry.geometry.set(lengths,30);entry.geometry.set(tangents,32);entry.geometry.set(forces,38);entry.material=preparation.value;
            }
            hinge++;
            if(!withTangent)continue;
            if(packed) {
                packed.dofs.set(dofs);packed.data.set(frames,0);packed.data.set(d1,18);packed.data.set(d2,24);
                packed.data.set(lengths,30);packed.data.set(tangents,32);packed.data.set(forces,38);
                continue;
            }
            for(let col=0;col<11;col++)for(let j=0;j<2;j++) {
                const at=3*j,sign=col>=at&&col<at+3?-1:col>=at+3&&col<at+6?1:0;
                const dx=col%3===0?sign:0,dy=col%3===1?sign:0,dz=col%3===2?sign:0;
                dLengths[col*2+j]=tangents[at]*dx+tangents[at+1]*dy+tangents[at+2]*dz;
                angular[col*6+at]=-(d2[at]*dx+d2[at+1]*dy+d2[at+2]*dz)/lengths[j];
                angular[col*6+at+1]=(d1[at]*dx+d1[at+1]*dy+d1[at+2]*dz)/lengths[j];
                angular[col*6+at+2]=col===9+j?1:0;
            }
            for(let col=0;col<11;col++) {
                const base=col*6;
                for(let i=0;i<6;i++) {
                    let value=0;for(let j=0;j<6;j++)value+=material.jacobian[i*6+j]*angular[base+j];
                    dTorque[i]=value;
                }
                column.fill(0);
                for(let j=0;j<2;j++) {
                    const at=3*j,m=9*j,a0=angular[base+at],a1=angular[base+at+1],a2=angular[base+at+2];
                    const wx=frames[m]*a0+frames[m+1]*a1+frames[m+2]*a2;
                    const wy=frames[m+3]*a0+frames[m+4]*a1+frames[m+5]*a2;
                    const wz=frames[m+6]*a0+frames[m+7]*a1+frames[m+8]*a2;
                    for(let a=0;a<3;a++) {
                        const dd1=a===0?wy*d1[at+2]-wz*d1[at+1]:a===1?wz*d1[at]-wx*d1[at+2]:wx*d1[at+1]-wy*d1[at];
                        const dd2=a===0?wy*d2[at+2]-wz*d2[at+1]:a===1?wz*d2[at]-wx*d2[at+2]:wx*d2[at+1]-wy*d2[at];
                        const df=(-dd2*material.torque[at]+dd1*material.torque[at+1]-d2[at+a]*dTorque[at]+d1[at+a]*dTorque[at+1])/lengths[j]-forces[at+a]*dLengths[col*2+j]/lengths[j];
                        column[at+a]-=df;column[at+3+a]+=df;
                    }
                    column[9+j]=dTorque[at+2];
                }
                for(let row=0;row<11;row++)H[dofs[row]*width+dofs[col]-dofs[row]+half]+=column[row];
            }
        }
    }
    kernel?.assemble();
    return energy;
}
