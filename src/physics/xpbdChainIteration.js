import {Quaternion,Vector3} from 'three';
import {evaluateBendTwistConstraint,quaternionExp} from './discreteKirchhoffRod.js';
import {quaternionColumns} from './projectiveRodMath.js';
import {solveXpbdChainBlock,xpbdConstraintResidual} from './xpbdChainBlock.js';
import {createXpbdSurfaceFriction} from './xpbdSurfaceFriction.js';

const axes=['x','y','z'];
const vector=p=>({x:p[0],y:p[1],z:p[2]});
const triples=i=>[i,i+1,i+2];

/** Global alternative to the local XPBD bend/adaptation sweep. The shared
 * axis has ONE length constraint per edge; each material adds only two shear
 * constraints and its independent bend/twist energy. Duplicating a hard
 * length constraint for both rods makes the dual matrix singular.
 */
export function createXpbdChainIteration(s,frames,initial,weights,dt,{normalCompliance=1e-12,balancePrimal=true,surfaceFriction=false,feedById={}}={}) {
    let count=s.positions.length*3;const frameDofs=frames.map(a=>a.map(()=>{const d=count;count+=3;return d;}));
    const inverseMass=new Float64Array(count);
    for(let i=0;i<weights.length;i++)inverseMass.fill(weights[i],3*i,3*i+3);
    frames.forEach((a,m)=>a.forEach((f,e)=>inverseMass.fill(f.w,frameDofs[m][e],frameDofs[m][e]+3)));
    const target=initial.predicted.map((p,i)=>p.map((v,k)=>v+dt*dt*weights[i]*s.loads[s.layout.positions[i]+k]));
    const nativeToPacked=new Int32Array(s.layout.dofCount).fill(-1);
    s.layout.positions.forEach((d,n)=>{for(let k=0;k<3;k++)nativeToPacked[d+k]=3*n+k;});
    const lambda=new Map(),frictionImpulses=new Map();
    const friction=surfaceFriction?createXpbdSurfaceFriction(s,frames,initial,dt,feedById):null;
    const build=(evaluate,contactLambda,contactSlip)=>{
            const constraints=[],native=evaluate();
            const add=(id,edge,dofs,jacobian,gap,alpha=0,extra={})=>constraints.push({id,edge,dofs,jacobian,gap,alpha,lambda:lambda.get(id)??0,...extra});
            for(let e=0;e+1<s.positions.length;e++) {
                const delta=s.positions[e+1].map((v,k)=>v-s.positions[e][k]),length=Math.hypot(...delta),rest=s.coordinates[e+1]-s.coordinates[e];
                const tangent=delta.map(v=>v/length),dofs=[...triples(3*e),...triples(3*(e+1))];
                add(`length/${e}`,e,dofs,[...tangent.map(v=>-v),...tangent],length-rest);
                for(let m=0;m<frames.length;m++) {
                    const f=frames[m][e];if(!f)continue;
                    const columns=quaternionColumns(f.q);
                    for(let k=0;k<2;k++) {
                        const d=columns[k],angular=new Vector3(...d).cross(new Vector3(...delta)).multiplyScalar(1/rest).toArray();
                        add(`shear/${m}/${e}/${k}`,e,[...dofs,...triples(frameDofs[m][e])],
                            [...d.map(v=>-v/rest),...d.map(v=>v/rest),...angular],d.reduce((n,v,j)=>n+v*delta[j],0)/rest);
                    }
                    if(e>0) {
                        const r=evaluateBendTwistConstraint(frames[m][e-1].q,f.q,f.rest,f.bendMeasure);
                        for(let k=0;k<3;k++)add(`bend/${m}/${e}/${k}`,e,[...triples(frameDofs[m][e-1]),...triples(frameDofs[m][e])],
                            [...r.gradient0.slice(k*3,k*3+3),...r.gradient1.slice(k*3,k*3+3)],r.strain[axes[k]],f.compliance[k]/(dt*dt));
                    }
                }
            }
            native.forEach((r,i)=>{
                if(r.kind!=='wall')return;
                if(r.dofs.some(d=>nativeToPacked[d]<0))throw new Error('Angular contact is unsupported by the chain prototype');
                add(`wall/${i}`,r.edge,r.dofs.map(d=>nativeToPacked[d]),r.jacobian,r.gap,normalCompliance/(dt*dt),
                    {unilateral:true,lambda:contactLambda[i]??0,nativeIndex:i});
            });
            if(friction)constraints.push(...friction.constraints(frameDofs,frictionImpulses));
            constraints.sort((a,b)=>a.edge-b.edge);
            const displacement=new Float64Array(count);
            for(let i=0;i<s.positions.length;i++)if(weights[i])for(let k=0;k<3;k++)displacement[3*i+k]=s.positions[i][k]-target[i][k];
            frames.forEach((a,m)=>a.forEach((f,e)=>{
                if(!f.w)return;
                const q=f.q.clone().multiply(new Quaternion(...initial.frames[m][e].predicted).invert()).normalize();
                if(q.w<0){q.x*=-1;q.y*=-1;q.z*=-1;q.w*=-1;}
                const n=Math.hypot(q.x,q.y,q.z),scale=n>1e-12?2*Math.atan2(n,q.w)/n:2;
                displacement.set([q.x*scale,q.y*scale,q.z*scale],frameDofs[m][e]);
            }));
            // Include accumulated Coulomb impulses in the primal balance.
            native.forEach((r,i)=>{
                if(!r.witness||!contactSlip[i])return;
                const ws=[1-r.witness.t,r.witness.t];
                for(let j=0;j<2;j++)for(let k=0;k<3;k++)displacement[3*(r.edge+j)+k]-=weights[r.edge+j]*ws[j]*contactSlip[i][k];
            });
            return {constraints,displacement};
    };
    const merit=({constraints,displacement})=>{
        const primal=displacement.slice();let value=0;
        for(const r of constraints) {
            const diagonal=r.alpha+r.dofs.reduce((v,d,j)=>v+inverseMass[d]*r.jacobian[j]**2,0);
            const c=xpbdConstraintResidual(r,diagonal);
            if(diagonal>0)value+=c*c/diagonal;
            for(let j=0;j<r.dofs.length;j++)primal[r.dofs[j]]-=inverseMass[r.dofs[j]]*r.jacobian[j]*r.lambda;
        }
        if(balancePrimal)for(let i=0;i<primal.length;i++)if(inverseMass[i])value+=primal[i]**2/inverseMass[i];
        return value;
    };
    const residuals=({constraints,displacement})=>{
        const primal=displacement.slice();
        let maxConstraintResidual=0,maxCompliantForceResidual=0,maxLoadedContactGap=0;
        for(const r of constraints) {
            const diagonal=r.alpha+r.dofs.reduce((v,d,j)=>v+inverseMass[d]*r.jacobian[j]**2,0);
            const c=xpbdConstraintResidual(r,diagonal);
            maxConstraintResidual=Math.max(maxConstraintResidual,Math.abs(c));
            if(r.alpha>0)maxCompliantForceResidual=Math.max(maxCompliantForceResidual,Math.abs(c)/(r.alpha*dt*dt));
            if(r.unilateral&&r.lambda>0)maxLoadedContactGap=Math.max(maxLoadedContactGap,Math.abs(r.gap));
            for(let j=0;j<r.dofs.length;j++)primal[r.dofs[j]]-=inverseMass[r.dofs[j]]*r.jacobian[j]*r.lambda;
        }
        let maxPositionForceResidual=0,maxFrameTorqueResidual=0;
        for(let i=0;i<primal.length;i++)if(inverseMass[i]) {
            const force=Math.abs(primal[i])/(inverseMass[i]*dt*dt);
            if(i<s.positions.length*3)maxPositionForceResidual=Math.max(maxPositionForceResidual,force);
            else maxFrameTorqueResidual=Math.max(maxFrameTorqueResidual,force);
        }
        return {maxPositionForceResidual,maxFrameTorqueResidual,maxConstraintResidual,maxCompliantForceResidual,maxLoadedContactGap,
            chart:'frozen-iteration-chart',equilibriumCertified:false};
    };
    return {
        step(evaluate,contactLambda,contactSlip,{trace=false}={}) {
            friction?.refresh(evaluate(),contactLambda);
            const assembly=build(evaluate,contactLambda,contactSlip),{constraints,displacement}=assembly;
            const beforeMerit=merit(assembly),result=solveXpbdChainBlock(constraints,inverseMass,displacement,{balancePrimal});
            const oldPositions=s.positions.map(p=>p.slice()),oldFrames=frames.map(a=>a.map(f=>f.q.clone()));
            let applied=false,scale,after;
            for(let trial=0;trial<16;trial++) {
                scale=2**(-trial);
                for(let i=0;i<s.positions.length;i++)for(let k=0;k<3;k++)s.positions[i][k]=oldPositions[i][k]+scale*result.increment[3*i+k];
                frames.forEach((a,m)=>a.forEach((f,e)=>{
                    const d=frameDofs[m][e],v=Array.from(result.increment.slice(d,d+3),x=>scale*x),q=quaternionExp(vector(v));
                    f.q.copy(oldFrames[m][e]).premultiply(new Quaternion(q.x,q.y,q.z,q.w)).normalize();
                }));
                frictionImpulses.clear();
                constraints.forEach((r,i)=>{
                    const next=r.lambda+scale*(result.lambda[i]-r.lambda);lambda.set(r.id,next);
                    if(r.nativeIndex!==undefined)contactLambda[r.nativeIndex]=next;
                    if(r.frictionId!==undefined){
                        const impulse=frictionImpulses.get(r.frictionId)??[0,0,0];
                        for(let k=0;k<3;k++)impulse[k]+=r.tangent[k]*next;
                        frictionImpulses.set(r.frictionId,impulse);
                    }
                });
                try {
                    after=build(evaluate,contactLambda,contactSlip);const afterMerit=merit(after);
                    if(afterMerit<=beforeMerit*(1-1e-4*scale)||afterMerit<1e-18){applied=true;break;}
                }catch(error){if(error.code!=='trial-outside-vessel')throw error;}
            }
            if(!applied)throw new Error('XPBD chain trial failed residual decrease');
            return {factorizations:result.factorizations,band:result.band,rows:result.rows,scale,beforeMerit,
                ...(trace?{residuals:residuals(after)}:{})};
        },
        frictionHistory(evaluate,contactLambda) {
            if(!friction)return [];
            friction.refresh(evaluate(),contactLambda);return friction.history();
        }
    };
}
