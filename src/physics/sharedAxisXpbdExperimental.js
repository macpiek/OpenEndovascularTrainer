import {Quaternion} from 'three';
import {evaluateAdaptationConstraint,evaluateBendTwistConstraint,quaternionExp} from './discreteKirchhoffRod.js';
import {captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,completeSharedAxisDynamicStep} from './kirchhoffSharedAxisDynamics.js';
import {assembleSharedAxisConstraintRows} from './kirchhoffSharedAxisConstraintRows.js';
import {measureSharedAxisQuality} from './kirchhoffSharedAxisDiagnostics.js';
import {iterateSharedAxisProjection} from './kirchhoffSharedAxisProjection.js';
import {createXpbdChainIteration} from './xpbdChainIteration.js';

// Research only. No production selection or automatic fallback imports this
// solver. XPBD multipliers belong to one timestep, not to Newton's force duals.
// Equation 18: https://matthias-research.github.io/pages/publications/XPBD.pdf
export const XPBD_RESEARCH_DEFAULTS=Object.freeze({iterations:32,maxLengthError:1e-5,maxPenetration:1e-5,maxShear:1e-5});
const axes=['x','y','z'];
const xyz=p=>({x:p[0],y:p[1],z:p[2]});
const identity=Float64Array.from([1,0,0,0,1,0,0,0,1]);
const negativeIdentity=identity.map(x=>-x);

// Three-component compliant constraint. Each gradient is row-major and each
// variable has an isotropic inverse mass (angular inertia for orientations).
export function xpbdVectorIncrement(value,lambda,compliance,dt,gradients,weights) {
    const a=new Float64Array(9),b=new Float64Array(3);
    for(let i=0;i<3;i++) {
        const alpha=compliance[i]/(dt*dt);a[i*3+i]=alpha;b[i]=-value[i]-alpha*lambda[i];
        for(let j=0;j<3;j++)for(let v=0;v<gradients.length;v++)for(let k=0;k<3;k++)
            a[i*3+j]+=weights[v]*gradients[v][i*3+k]*gradients[v][j*3+k];
    }
    // Cholesky of the local SPD block. Fully prescribed constraints need no
    // update; their violation remains visible in the final quality measure.
    if(!(a[0]>0))return new Float64Array(3);
    const l00=Math.sqrt(a[0]),l10=a[3]/l00,l20=a[6]/l00;
    const d11=a[4]-l10*l10;if(!(d11>0))return new Float64Array(3);
    const l11=Math.sqrt(d11),l21=(a[7]-l20*l10)/l11,d22=a[8]-l20*l20-l21*l21;
    if(!(d22>0))return new Float64Array(3);
    const l22=Math.sqrt(d22),y0=b[0]/l00,y1=(b[1]-l10*y0)/l11,y2=(b[2]-l20*y0-l21*y1)/l22;
    const z2=y2/l22,z1=(y1-l21*z2)/l11,z0=(y0-l10*z1-l20*z2)/l00;
    return Float64Array.from([z0,z1,z2]);
}
function correction(j,delta,w) {
    return [0,1,2].map(k=>w*(j[k]*delta[0]+j[3+k]*delta[1]+j[6+k]*delta[2]));
}
function rotate(q,v) {
    const d=quaternionExp(xyz(v));q.premultiply(new Quaternion(d.x,d.y,d.z,d.w)).normalize();
}

/** Independent Cosserat orientations, exact manufactured Log bend/twist
 * potential, hard adaptation and unilateral contacts, mass-weighted XPBD.
 * Shares mesh/feed and collision discovery with the reference solver. A finite
 * iteration count is not a force-equilibrium certificate. A failed candidate
 * rolls back its pose/history; callers must fork discovery before attempting it.
 */
export function* iterateSharedAxisXpbd(s,dt,{xpbd={},bloodFlow=null,feedById={}}={}) {
    const {hardClosure=false,globalChain=false,chainPrimalBalance=true,chainSurfaceFriction=false,chainTrace=false,...settings}=xpbd;
    const o={...XPBD_RESEARCH_DEFAULTS,...settings};
    if(!(dt>0&&Number.isFinite(dt))||!Number.isInteger(o.iterations)||o.iterations<1||
        Object.values(o).some(x=>!Number.isFinite(x)||x<=0))throw new RangeError('Invalid XPBD research options');
    if(chainSurfaceFriction&&(!globalChain||!chainPrimalBalance||hardClosure))throw new RangeError('Surface friction requires the primal-balanced chain without post-projection');
    const saved=captureSharedAxisNative(s),history={velocities:s.velocities,angularVelocities:s.angularVelocities,
        acceptedWallGaps:s.acceptedWallGaps,wallFrictionHistory:s.wallFrictionHistory,acceptedSolves:s.acceptedSolves};
    const started=performance.now(),chainIterations=[];let committed=false,iterations=0,rows=[],frictionCorrections=0,closureIterations=0,factorizations=0;
    try {
        prepareSharedAxisDynamicStep(s,dt,bloodFlow);
        const initial=s.dynamicStep,weights=initial.masses.map((m,i)=>s.fixed[s.layout.positions[i]]?0:1/m);
        if([...weights].some(x=>!Number.isFinite(x)))throw new Error('XPBD requires finite positive free-node masses');
        const frames=s.materials.map(({body,spec,last},m)=>Array.from({length:last},(_,e)=>({
            q:new Quaternion(...(e===0?initial.frames[m][e].old:initial.frames[m][e].predicted)),
            w:e===0?0:5/((spec.mass??(spec.id==='catheter'?1.4:1))*body.restLength[e]),
            adaptation:new Float64Array(3),bend:new Float64Array(3),adaptationMeasure:{},bendMeasure:{},
            compliance:[body.kirchhoffBendCompliance1[e],body.kirchhoffBendCompliance2[e],body.kirchhoffTwistCompliance[e]],
            rest:{x:body.restRotation1[e],y:body.restRotation2[e],z:body.restRotation3[e]}
        })));
        for(let i=0;i<s.positions.length;i++)if(weights[i])for(let k=0;k<3;k++)
            s.positions[i][k]=initial.predicted[i][k]+dt*dt*weights[i]*s.loads[s.layout.positions[i]+k];
        const dofNode=new Int32Array(s.layout.dofCount).fill(-1);
        s.layout.positions.forEach((p,n)=>{for(let k=0;k<3;k++)dofNode[p+k]=n;});
        const contactLambda=[],contactSlip=[];
        const chainIteration=globalChain?createXpbdChainIteration(s,frames,initial,weights,dt,
            {balancePrimal:chainPrimalBalance,surfaceFriction:chainSurfaceFriction,feedById}):null;
        const evaluate=()=>{
            s.geometryKey=Symbol('xpbd-pose');s.chain.gradient.fill(0);
            for(let retries=0;;retries++) {
                try {const r=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});
                    if(!s.pendingVesselRows?.size)return r;
                } catch(error) {if(error.message!=='shared-axis-wall-discovery'||!s.pendingVesselRows?.size)throw error;}
                if(retries>=64)throw new Error('XPBD contact discovery did not settle');
                extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
            }
        };
        const adaptation=(m,e)=>{
            const f=frames[m][e],a=s.positions[e],b=s.positions[e+1],length=s.coordinates[e+1]-s.coordinates[e];
            // A fractional material tip shares the same geometric tangent.
            const r=evaluateAdaptationConstraint(xyz(a),xyz(b),f.q,length,f.adaptationMeasure);
            const gradients=[negativeIdentity,identity,r.orientationGradient],w=[weights[e],weights[e+1],f.w];
            const delta=xpbdVectorIncrement(axes.map(k=>r.constraint[k]),f.adaptation,[0,0,0],dt,gradients,w);
            for(let k=0;k<3;k++){f.adaptation[k]+=delta[k];a[k]-=w[0]*delta[k];b[k]+=w[1]*delta[k];}
            rotate(f.q,correction(r.orientationGradient,delta,f.w));
        };
        const bend=(m,e)=>{
            const a=frames[m][e-1],b=frames[m][e],r=evaluateBendTwistConstraint(a.q,b.q,b.rest,b.bendMeasure);
            const delta=xpbdVectorIncrement(axes.map(k=>r.strain[k]),b.bend,b.compliance,dt,[r.gradient0,r.gradient1],[a.w,b.w]);
            for(let k=0;k<3;k++)b.bend[k]+=delta[k];
            rotate(a.q,correction(r.gradient0,delta,a.w));rotate(b.q,correction(r.gradient1,delta,b.w));
        };
        for(let iteration=0;iteration<o.iterations;iteration++) {
            if(chainIteration){
                const result=chainIteration.step(evaluate,contactLambda,contactSlip,{trace:chainTrace});
                factorizations+=result.factorizations;if(chainTrace)chainIterations.push(result);
            }
            // Symmetric sweeps reduce dependence on insertion direction.
            else for(let m=0;m<frames.length;m++) {
                if(iteration%2===0){for(let e=1;e<frames[m].length;e++)bend(m,e);for(let e=0;e<frames[m].length;e++)adaptation(m,e);}
                else {for(let e=frames[m].length-1;e>0;e--)bend(m,e);for(let e=frames[m].length-1;e>=0;e--)adaptation(m,e);}
            }
            rows=evaluate();
            // Linearized contact rows all refer to this same pose. Account
            // for earlier corrections in this sweep before computing each gap.
            const base=s.positions.map(p=>p.slice());
            for(let i=0;i<rows.length;i++) {
                const r=rows[i];if(r.kind!=='wall')continue;
                let gap=r.gap,denom=0;
                for(let j=0;j<r.dofs.length;j++) {
                    const n=dofNode[r.dofs[j]],k=j%3;if(n<0)throw new Error('Unexpected angular wall constraint');
                    gap+=r.jacobian[j]*(s.positions[n][k]-base[n][k]);denom+=weights[n]*r.jacobian[j]**2;
                }
                if(!(denom>0))continue;
                const old=contactLambda[i]??0,next=globalChain?old:Math.max(0,old-gap/denom),delta=next-old;contactLambda[i]=next;
                for(let j=0;j<r.dofs.length;j++)s.positions[dofNode[r.dofs[j]]][j%3]+=weights[dofNode[r.dofs[j]]]*r.jacobian[j]*delta;
                // Material slip includes commanded feed, not only mesh motion.
                // Accumulated tangential impulse is projected onto the Coulomb
                // disk; it must not be reapplied in full on every iteration.
                if(!chainSurfaceFriction&&r.witness&&(next>0||contactSlip[i])&&r.dofs.length===6) {
                    const t=r.witness.t,ws=[1-t,t],n=[0,1,2].map(k=>r.jacobian[k]+r.jacobian[k+3]),norm=Math.hypot(...n);
                    if(norm===0)continue;for(let k=0;k<3;k++)n[k]/=norm;
                    const spec=sharedAxisOuterMaterialAt(s,r.edge,t,r.witness.owner).spec;
                    const tangent=s.positions[r.edge+1].map((v,k)=>v-s.positions[r.edge][k]),length=Math.hypot(...tangent);
                    const slip=[0,1,2].map(k=>ws.reduce((v,w,j)=>v+w*(s.positions[r.edge+j][k]-initial.positions[r.edge+j][k]),0)+(feedById[spec.id]??0)*tangent[k]/length);
                    const normal=slip.reduce((v,x,k)=>v+x*n[k],0),oldT=contactSlip[i]??[0,0,0];
                    const trial=oldT.map((v,k)=>v-(slip[k]-normal*n[k])/denom),magnitude=Math.hypot(...trial);
                    const limit=magnitude<=(spec.wallStaticFriction??0)*next?magnitude:(spec.wallKineticFriction??0)*next;
                    const newT=trial.map(v=>v*Math.min(1,limit/Math.max(magnitude,1e-30)));contactSlip[i]=newT;
                    for(let j=0;j<2;j++)for(let k=0;k<3;k++)s.positions[r.edge+j][k]+=weights[r.edge+j]*ws[j]*(newT[k]-oldT[k]);
                    frictionCorrections++;
                }
            }
            iterations++;yield {iteration,method:'xpbd',factorizations:0};
        }
        let maxShear=0;
        for(let m=0;m<frames.length;m++)for(let e=0;e<frames[m].length;e++) {
            const f=frames[m][e],length=s.coordinates[e+1]-s.coordinates[e],body=s.materials[m].body;
            const r=evaluateAdaptationConstraint(xyz(s.positions[e]),xyz(s.positions[e+1]),f.q,length,f.adaptationMeasure);
            maxShear=Math.max(maxShear,r.residual/length);
            for(const k of ['X','Y','Z','W'])body['orientation'+k][e]=f.q[k.toLowerCase()];
        }
        const preClosureShear=maxShear,preClosurePositions=hardClosure?s.positions.map(p=>p.slice()):null;
        const absoluteLengthError=()=>Math.max(...s.positions.slice(1).map((p,i)=>
            Math.abs(Math.hypot(...p.map((v,k)=>v-s.positions[i][k]))-(s.coordinates[i+1]-s.coordinates[i]))));
        if(hardClosure) {
            // Research hybrid: impose the Kirchhoff tangent and globally close
            // position constraints after local XPBD. This is an operator split,
            // not a converged XPBD or Newton equilibrium. Record its correction
            // and test its complete trajectory independently.
            applySharedAxisNativeIncrement(s,new Float64Array(s.layout.dofCount),new Float64Array(s.multipliers.length));
            for(let i=0;i<32;i++) {
                rows=evaluate();
                const q=measureSharedAxisQuality(s,rows);
                const bendViolation=rows.some(r=>r.subtype==='bend-limit'&&r.gap< -1e-9);
                if(q.maxPenetration<=o.maxPenetration*.5&&absoluteLengthError()<=o.maxLengthError*.5&&
                    q.bodies.every(b=>b.maxLengthError<=o.maxLengthError*.5)&&!bendViolation)break;
                const direction=yield* iterateSharedAxisProjection(s,{rows});
                factorizations+=direction.factorizations??0;closureIterations++;
                if(!direction.converged)throw new Error('XPBD hard closure could not solve the contact projection');
                const pose=captureSharedAxisNative(s);let valid=false;
                for(let trial=0;trial<14;trial++) {
                    restoreSharedAxisNative(s,pose);
                    try {
                        applySharedAxisNativeIncrement(s,direction.increment,direction.zeroReactions,2**(-trial));
                        evaluate();valid=true;break;
                    } catch(error) {if(error.code!=='trial-outside-vessel')throw error;}
                }
                if(!valid)throw new Error('XPBD hard closure crossed the vessel surface');
            }
            // Measure the published frames, including any remaining stretch.
            // The mismatch before correction is retained separately.
            maxShear=0;
            for(const {body,last} of s.materials)for(let e=0;e<last;e++) {
                const q=new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]);
                const length=s.coordinates[e+1]-s.coordinates[e];
                maxShear=Math.max(maxShear,evaluateAdaptationConstraint(xyz(s.positions[e]),xyz(s.positions[e+1]),q,length).residual/length);
            }
        }
        rows=evaluate();s.multipliers.fill(0);
        rows.forEach((r,i)=>{r.multiplier=r.kind==='wall'?(contactLambda[i]??0)/(dt*dt):0;s.multipliers[i]=r.multiplier;});
        const quality=measureSharedAxisQuality(s,rows),maxLengthError=Math.max(...quality.bodies.map(b=>b.maxLengthError));
        const maxLengthErrorMm=absoluteLengthError();
        const closureCorrectionMm=preClosurePositions?Math.max(...s.positions.map((p,i)=>Math.hypot(...p.map((v,k)=>v-preClosurePositions[i][k])))):0;
        const finiteFrames=frames.every(a=>a.every(f=>f.q.toArray().every(Number.isFinite)));
        const admissible=quality.finite&&finiteFrames&&maxShear<=o.maxShear&&maxLengthError<=o.maxLengthError&&quality.maxPenetration<=o.maxPenetration&&
            maxLengthErrorMm<=o.maxLengthError&&
            quality.bodies.every(b=>b.maxBendLimitDegrees===null||b.maxBendAngleDegrees<=b.maxBendLimitDegrees+1e-5);
        if(admissible) {
            // Native publication requires frames aligned to the common axis.
            // Raw mismatch is gated above, never hidden by this conversion.
            applySharedAxisNativeIncrement(s,new Float64Array(s.layout.dofCount),new Float64Array(s.multipliers.length));
            const frictionHistory=chainIteration?.frictionHistory(evaluate,contactLambda)??[];
            completeSharedAxisDynamicStep(s);s.acceptedSolves++;
            s.acceptedWallGaps=new Map(rows.filter(r=>r.id).map(r=>[r.id,r.gap]));s.wallFrictionHistory=frictionHistory;committed=true;
        }
        return {converged:admissible,status:admissible?'xpbd-geometry-admissible':'xpbd-quality-limit',quality,
            xpbd:{parameters:{...o,hardClosure,globalChain,chainPrimalBalance,chainSurfaceFriction,chainTrace},maxShear,preClosureShear,maxLengthError,maxLengthErrorMm,closureCorrectionMm,frictionCorrections,closureIterations,
                model:globalChain?'global-chain-xpbd':hardClosure?'xpbd-with-hard-kirchhoff-closure':'cosserat-xpbd',equilibriumCertified:false},iterations,factorizations,
            ...(chainTrace?{chainIterations}:{}),timings:{},ms:performance.now()-started};
    } catch(error) {return {converged:false,status:'xpbd-error',error:error.message,iterations,factorizations:0,
        ...(chainTrace?{chainIterations}:{}),timings:{},ms:performance.now()-started};}
    finally {if(!committed){restoreSharedAxisNative(s,saved);Object.assign(s,history);s.dynamicStep=null;}}
}
