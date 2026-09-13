import {evaluateBendTwistLocalConstraintNormalized,normalizeQuaternion} from './discreteKirchhoffRod.js';

// Column-major rotation, matching the native material row convention.
function rotation(q,out) {
    const scale=2/Math.max(1e-24,q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    const xx=q.x*q.x*scale,yy=q.y*q.y*scale,zz=q.z*q.z*scale;
    const xy=q.x*q.y*scale,xz=q.x*q.z*scale,yz=q.y*q.z*scale;
    const wx=q.w*q.x*scale,wy=q.w*q.y*scale,wz=q.w*q.z*scale;
    out[0]=1-yy-zz;out[1]=xy+wz;out[2]=xz-wy;
    out[3]=xy-wz;out[4]=1-xx-zz;out[5]=yz+wx;
    out[6]=xz+wy;out[7]=yz-wx;out[8]=1-xx-yy;
}
function readFrame(body,e,q) {
    q.x=body.orientationX[e];q.y=body.orientationY[e];q.z=body.orientationZ[e];q.w=body.orientationW[e];
    return normalizeQuaternion(q,q);
}

/** The native three bend/twist rows pulled back directly onto three shared
 * positions and two material spins. It assembles exactly J^T K J; the full
 * Newton tangent remains a separate path. No adaptation rows or native KKT
 * workspace are needed because the shared geometry enforces adaptation.
 * Fractional final material cells retain their own profile/compliance; their
 * angular pullback uses the complete spatial cell, as in the exact tangent.
 */
export function assembleSharedAxisGaussNewton(s,withTangent=true) {
    const {layout,chain}=s,{gradient:g,hessian:H}=chain,band=layout.band;
    chain.tangent=null;H.fill(0);for(let i=0;i<g.length;i++)g[i]=-s.loads[i];
    const dofs=new Int32Array(11),row=new Float64Array(11),pull=new Float64Array(12);
    const relative=new Float64Array(9),R=new Float64Array(9),q0={},q1={},rest={},local={};
    let energy=0;
    for(const {body,spec,last} of s.materials) {
        const spins=layout.spins.get(spec.id);
        for(let e=1;e<last;e++) {
            for(let node=0;node<3;node++)for(let a=0;a<3;a++)dofs[node*3+a]=layout.positions[e-1+node]+a;
            dofs[9]=spins[e-1];dofs[10]=spins[e];
            readFrame(body,e-1,q0);readFrame(body,e,q1);
            rest.x=body.restRotation1[e];rest.y=body.restRotation2[e];rest.z=body.restRotation3[e];
            const state=evaluateBendTwistLocalConstraintNormalized(q0,q1,rest,local),A=state.localGradient;
            rotation(state.relative,relative);
            for(let side=0;side<2;side++) {
                rotation(side===0?q0:q1,R);
                const a=s.positions[e-1+side],b=s.positions[e+side];
                const dx=b[0]-a[0],dy=b[1]-a[1],dz=b[2]-a[2],inverseLength=1/Math.sqrt(dx*dx+dy*dy+dz*dz);
                for(let axis=0;axis<3;axis++){pull[side*6+axis]=-R[3+axis]*inverseLength;pull[side*6+3+axis]=R[axis]*inverseLength;}
            }
            for(let axis=0;axis<3;axis++) {
                const compliance=axis===0?body.kirchhoffBendCompliance1[e]:axis===1?body.kirchhoffBendCompliance2[e]:body.kirchhoffTwistCompliance[e];
                if(!(compliance>0))throw new RangeError('Shared material bend/twist compliance must be positive');
                const strain=axis===0?state.strain.x:axis===1?state.strain.y:state.strain.z,k=1/compliance;
                energy+=.5*k*strain*strain;row.fill(0);
                for(let side=0;side<2;side++) {
                    const at=axis*3;
                    const u=side===0?-A[at]:A[at]*relative[0]+A[at+1]*relative[1]+A[at+2]*relative[2];
                    const v=side===0?-A[at+1]:A[at]*relative[3]+A[at+1]*relative[4]+A[at+2]*relative[5];
                    const w=side===0?-A[at+2]:A[at]*relative[6]+A[at+1]*relative[7]+A[at+2]*relative[8];
                    for(let a=0;a<3;a++) {
                        const value=u*pull[side*6+a]+v*pull[side*6+3+a];
                        row[side*3+a]-=value;row[side*3+3+a]+=value;
                    }
                    row[9+side]=w;
                }
                for(let i=0;i<11;i++)g[dofs[i]]+=row[i]*k*strain;
                if(withTangent)for(let i=0;i<11;i++)for(let j=0;j<=i;j++) {
                    const a=dofs[i],b=dofs[j],hi=Math.max(a,b),lo=Math.min(a,b);
                    if(hi-lo>=band)throw new RangeError('Native pullback exceeds shared band');
                    H[hi*band+hi-lo]+=k*row[i]*row[j];
                }
            }
        }
    }
    return energy;
}
