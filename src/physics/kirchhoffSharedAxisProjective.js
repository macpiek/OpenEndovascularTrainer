import {Quaternion,Vector3} from 'three';
import {quaternionExp} from './discreteKirchhoffRod.js';
import {captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,sharedAxisOuterMaterialAt} from './kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,completeSharedAxisDynamicStep} from './kirchhoffSharedAxisDynamics.js';
import {assembleSharedAxisConstraintRows} from './kirchhoffSharedAxisConstraintRows.js';
import {measureSharedAxisQuality} from './kirchhoffSharedAxisDiagnostics.js';
import {factorProjectiveBand,projectRotation,quaternionColumns} from './projectiveRodMath.js';

export const PROJECTIVE_ROD_DEFAULTS=Object.freeze({iterations:16,tolerance:.002,axialRigidity:2e6,
    rotationPenalty:8,contactWeight:2e6,maxLengthError:.03,maxPenetration:.1,maxShear:.1});
const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const readQ=(b,e)=>({x:b.orientationX[e],y:b.orientationY[e],z:b.orientationZ[e],w:b.orientationW[e]});

/** Experimental PD Cosserat penalty rod. Generalized vector unknowns are
 * shared positions and three independent directors per material edge. Local
 * SO(3) projections + one constant SPD band solve replace Newton/KKT. Chordal
 * bend/twist, finite stretch/shear and penalty contacts deliberately change the
 * constitutive model. Contact geometry discovery can rebuild the factor.
 * Coulomb slip uses a bounded positional return, not the Newton certificate.
 */
export function* iterateSharedAxisProjective(s,dt,{projective={},feedById={}}={}) {
    const o={...PROJECTIVE_ROD_DEFAULTS,...projective};
    if(!Number.isInteger(o.iterations)||o.iterations<1||Object.values(o).some(v=>!Number.isFinite(v)||v<=0))throw new RangeError('Invalid PD options');
    const incoming=captureSharedAxisNative(s),previous={velocities:s.velocities,angularVelocities:s.angularVelocities,
        acceptedWallGaps:s.acceptedWallGaps,wallFrictionHistory:s.wallFrictionHistory,acceptedSolves:s.acceptedSolves};
    const started=performance.now(),timings={assemblyMs:0,linearMs:0,frictionMs:0,projectionMs:0};
    let committed=false,iterations=0,factorizations=0,backtracks=0,rows=[],change=Infinity,maxShear=0;
    try {
        prepareSharedAxisDynamicStep(s,dt);
        const initial=s.dynamicStep,variables=[],nodes=[],frames=[],stretches=[],terms=[],fixed=new Map(),contacts=new Map();
        const term=(ids,coefficients,weight,target=[0,0,0])=>{const t={ids,coefficients,weight,target};terms.push(t);return t;};
        const frameIds=s.materials.map(()=>[]);
        // Spatial ordering keeps the global matrix band narrow even for two rods.
        for(let e=0;e<s.positions.length;e++) {
            nodes[e]=variables.length;variables.push(s.positions[e].slice());
            if(s.fixed[s.layout.positions[e]])fixed.set(nodes[e],s.positions[e].slice());
            s.materials.forEach(({body,last,spec},m)=>{
                if(e>=last)return;
                const columns=quaternionColumns(readQ(body,e)),ids=columns.map(c=>{const id=variables.length;variables.push(c);return id;});
                frameIds[m][e]=ids;
                if(e===0)ids.forEach((id,k)=>fixed.set(id,columns[k].slice()));
                const length=body.restLength[e],inertia=(spec.mass??(spec.id==='catheter'?1.4:1))*length/5;
                const adjacent=Math.max(1/(body.kirchhoffBendCompliance1[e]||1),1/(body.kirchhoffBendCompliance1[e+1]||1));
                const weight=o.rotationPenalty*adjacent+inertia/(dt*dt);
                const projected=ids.map((id,k)=>term([id],[1],weight,columns[k]));
                const predicted=quaternionColumns(new Quaternion(...initial.frames[m][e].predicted));
                ids.forEach((id,k)=>term([id],[1],inertia/(2*dt*dt),predicted[k]));
                frames.push({ids,projected,m,e,length});
            });
        }
        s.positions.forEach((p,i)=>term([nodes[i]],[1],initial.masses[i]/(dt*dt),initial.predicted[i]));
        for(let e=0;e+1<nodes.length;e++) {
            const length=s.coordinates[e+1]-s.coordinates[e];
            stretches.push({e,length,term:term([nodes[e],nodes[e+1]],[-1,1],2*o.axialRigidity/length)});
        }
        for(let m=0;m<s.materials.length;m++) {
            const {body,last}=s.materials[m];
            for(let e=0;e<last;e++) {
                const length=body.restLength[e],fraction=length/(s.coordinates[e+1]-s.coordinates[e]);
                // A moving tip can create a tiny cell. Keep at least one fine-cell
                // shear penalty so this short director does not become a soft hinge.
                term([nodes[e],nodes[e+1],frameIds[m][e][2]],[-fraction,fraction,-length],o.axialRigidity*Math.max(length,s.spacing)/(length*length));
                if(e===0)continue;
                const d=quaternionColumns(quaternionExp({x:body.restRotation1[e],y:body.restRotation2[e],z:body.restRotation3[e]}));
                const k=[body.kirchhoffBendCompliance1[e],body.kirchhoffBendCompliance2[e],body.kirchhoffTwistCompliance[e]].map(c=>1/c);
                const weights=k.map((v,i)=>(k[(i+1)%3]+k[(i+2)%3]-v)/2);
                if(weights.some(w=>!(w>=0&&Number.isFinite(w))))throw new Error('PD chordal model requires compatible positive bend/twist rigidities');
                for(let c=0;c<3;c++)term([...frameIds[m][e-1],frameIds[m][e][c]],[...d[c].map(v=>-v),1],weights[c]);
            }
        }
        let x=variables,factor=null;
        const writePositions=()=>{
            for(let i=0;i<nodes.length;i++)for(let k=0;k<3;k++)s.positions[i][k]=x[nodes[i]][k];
            s.geometryKey=Symbol('pd-pose');
        };
        const evaluate=()=>{
            s.chain.gradient.fill(0);
            let r=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});
            if(s.pendingVesselRows?.size){extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
                r=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});}
            return r;
        };
        for(let iteration=0;iteration<o.iterations;iteration++) {
            iterations=iteration+1;const assemblyStart=performance.now();
            rows=evaluate();
            for(let i=0;i<rows.length;i++) {
                const r=rows[i];
                if(r.kind!=='wall'||r.gap>.1||!r.jacobian.some(v=>Math.abs(v)>1e-12)||contacts.has(i))continue;
                const ids=Array.from({length:r.dofs.length/3},(_,j)=>nodes[r.edge+j]);
                const projections=ids.map(id=>term([id],[1],o.contactWeight,x[id].slice()));
                contacts.set(i,{ids,projections});factor=null;
            }
            for(const stretch of stretches) {
                const edge=x[nodes[stretch.e+1]].map((v,k)=>v-x[nodes[stretch.e]][k]),norm=Math.hypot(...edge);
                if(!(norm>1e-12))throw new Error('PD collapsed segment');
                stretch.term.target=edge.map(v=>v*stretch.length/norm);
            }
            for(const f of frames) {
                const projected=projectRotation(f.ids.map(id=>x[id]));
                f.projected.forEach((t,k)=>{t.target=projected.columns[k];});
            }
            for(const [index,c] of contacts) {
                const r=rows[index],denom=r.jacobian.reduce((v,j,i)=>v+(fixed.has(c.ids[Math.floor(i/3)])?0:j*j),0);
                const depth=Math.max(0,-r.gap),lambda=denom>0?depth/denom:0;
                c.projections.forEach((t,j)=>{t.target=x[c.ids[j]].map((v,k)=>v+lambda*r.jacobian[j*3+k]);});
                // Approximate Coulomb return uses full material slip (including
                // feed), bounded by this projector's normal correction.
                if(r.witness&&depth>0&&denom>0) {
                    const t=r.witness.t,weights=[1-t,t],n=[0,1,2].map(k=>r.jacobian[k]+r.jacobian[k+3]),norm=Math.hypot(...n);
                    if(norm>0) {
                        for(let k=0;k<3;k++)n[k]/=norm;
                        const material=sharedAxisOuterMaterialAt(s,r.edge,t,r.witness.owner),spec=material.spec;
                        const tangent=s.positions[r.edge+1].map((v,k)=>v-s.positions[r.edge][k]),length=Math.hypot(...tangent);
                        const slip=[0,1,2].map(k=>weights.reduce((v,w,j)=>v+w*(s.positions[r.edge+j][k]-initial.positions[r.edge+j][k]),0)+(feedById[spec.id]??0)*tangent[k]/length);
                        const normal=dot(slip,n),tangential=slip.map((v,k)=>v-normal*n[k]),distance=Math.hypot(...tangential);
                        const limit=distance<=(spec.wallStaticFriction??0)*depth?distance:Math.min(distance,(spec.wallKineticFriction??0)*depth);
                        if(distance>0)c.projections.forEach((p,j)=>{for(let k=0;k<3;k++)p.target[k]-=weights[j]*limit*tangential[k]/(distance*(weights[0]**2+weights[1]**2));});
                    }
                }
            }
            const rhs=x.map(()=>[0,0,0]);
            for(const t of terms)for(let j=0;j<t.ids.length;j++)for(let k=0;k<3;k++)rhs[t.ids[j]][k]+=t.weight*t.coefficients[j]*t.target[k];
            for(let i=0;i<nodes.length;i++)for(let k=0;k<3;k++)rhs[nodes[i]][k]+=s.loads[s.layout.positions[i]+k];
            timings.assemblyMs+=performance.now()-assemblyStart;
            const solveStart=performance.now();
            if(!factor){factor=factorProjectiveBand(terms,x.length,fixed);factorizations++;}
            const proposed=factor.solve(rhs),old=x;
            timings.linearMs+=performance.now()-solveStart;
            // Limit trial movement to retain the signed vessel side. A failed
            // query must roll back before another trial or publication.
            let accepted=false;
            for(let trial=0;trial<14;trial++) {
                const alpha=2**(-trial);x=old.map((p,i)=>p.map((v,k)=>v+alpha*(proposed[i][k]-v)));writePositions();
                try {rows=evaluate();accepted=true;break;}catch(error) {
                    if(error.code!=='trial-outside-vessel')throw error;backtracks++;
                }
            }
            if(!accepted){x=old;writePositions();throw new Error('PD contact trial crossed the vessel surface');}
            change=0;
            for(let i=0;i<x.length;i++)change=Math.max(change,Math.hypot(...x[i].map((v,k)=>v-old[i][k])));
            yield {iteration,method:'projective-dynamics',change,factorizations};
            for(const f of frames)for(const id of f.ids)change=Math.max(change,f.length*Math.hypot(...x[id].map((v,k)=>v-old[id][k])));
            if(change<o.tolerance&&iteration>=7)break;
        }
        // Publish orthonormal material frames aligned with the shared tangent;
        // the raw director/tangent mismatch remains visible as a quality limit.
        for(const f of frames) {
            const projection=projectRotation(f.ids.map(id=>x[id])),{body}=s.materials[f.m];
            const tangent=new Vector3(...s.positions[f.e+1]).sub(new Vector3(...s.positions[f.e])).normalize();
            const z=new Vector3(...projection.columns[2]);maxShear=Math.max(maxShear,z.distanceTo(tangent));
            const q=new Quaternion(projection.quaternion.x,projection.quaternion.y,projection.quaternion.z,projection.quaternion.w);
            q.premultiply(new Quaternion().setFromUnitVectors(z,tangent));
            for(const component of ['X','Y','Z','W'])body['orientation'+component][f.e]=q[component.toLowerCase()];
        }
        // Synchronize native body positions without changing the accepted pose.
        applySharedAxisNativeIncrement(s,new Float64Array(s.layout.dofCount),new Float64Array(s.multipliers.length));
        rows=evaluate();s.multipliers.fill(0);
        for(const row of rows)row.multiplier=0;
        for(const [index,c] of contacts) {
            const r=rows[index],norm=r.jacobian.reduce((sum,j)=>sum+j*j,0);
            s.multipliers[index]=r.multiplier=norm>0?o.contactWeight*Math.max(0,-r.gap)/norm:0;
        }
        const quality=measureSharedAxisQuality(s,rows),maxLengthError=Math.max(...quality.bodies.map(b=>b.maxLengthError));
        const admissible=quality.finite&&Number.isFinite(change)&&maxLengthError<=o.maxLengthError&&quality.maxPenetration<=o.maxPenetration&&maxShear<=o.maxShear&&
            quality.bodies.every(b=>b.maxBendLimitDegrees===null||b.maxBendAngleDegrees<=b.maxBendLimitDegrees+2);
        const pd={model:'projective-cosserat-penalty',parameters:o,iterations,factorizations,change,maxShear,maxLengthError,dofs:3*x.length,
            localGlobalConverged:change<o.tolerance,limits:{lengthError:o.maxLengthError,penetration:o.maxPenetration,shear:o.maxShear},friction:'approximate-positional-coulomb'};
        if(admissible) {
            completeSharedAxisDynamicStep(s);s.acceptedSolves++;
            s.acceptedWallGaps=new Map(rows.filter(r=>r.id).map(r=>[r.id,r.gap]));s.wallFrictionHistory=[];committed=true;
            for(const b of quality.bodies){const m=s.materials.find(m=>m.spec.id===b.id);b.maxSpeed=Math.max(...s.velocities.slice(0,m.last+1).map(v=>Math.hypot(...v)));}
        }
        return {converged:admissible,status:admissible?(pd.localGlobalConverged?'pd-converged':'pd-iteration-budget'):'pd-quality-limit',
            pd,quality,iterations,factorizations,backtracks,geometryRestarts:0,frictionIterations:0,timings,ms:performance.now()-started};
    } catch(error) {
        return {converged:false,status:'pd-error',error:error.message,iterations,factorizations,backtracks,geometryRestarts:0,frictionIterations:0,timings,ms:performance.now()-started};
    } finally {
        if(!committed){restoreSharedAxisNative(s,incoming);Object.assign(s,previous);s.dynamicStep=null;}
    }
}
