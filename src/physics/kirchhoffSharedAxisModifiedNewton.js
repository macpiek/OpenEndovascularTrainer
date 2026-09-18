import {createIncrementalContactLU} from './kirchhoffIncrementalContactLU.js';

/** Private to one nonlinear solve, including its cooperative yields. Frozen
 * matrices are a chord-Newton approximation, never a current Jacobian cache.
 * Physical residuals, contact discovery and trial acceptance remain current. */
export function createSharedAxisModifiedNewton() {
    let packed=null,lu=null,indices=null,saved=null,armed=false,age=0;
    const diagnostics={modifiedAttempts:0,modifiedAccepted:0,modifiedFallbacks:0};
    const invalidate=()=>{armed=false;saved=null;age=0;};
    return {diagnostics,invalidate,
        getLU(w) {
            if(packed!==w) {lu?.dispose();lu=createIncrementalContactLU(w.band,w.count,{maxRank:0});packed=w;indices=null;invalidate();}
            return lu;
        },
        record(w,activeIndices) {if(w===packed)indices=activeIndices.slice();},
        seal(rows,fixed) {
            if(!packed||!indices)return invalidate();
            const mask=new Uint8Array(rows.length);for(const i of indices)mask[i]=1;
            saved={matrix:packed.matrix.slice(),scales:packed.scales.slice(),fixed:fixed.slice(),mask,
                rows:rows.map(r=>({id:r.id,kind:r.kind,dofs:r.dofs.slice(),jacobian:r.jacobian.slice(),
                    extraForceDofs:r.extraForceDofs?.slice(),extraForceJacobian:r.extraForceJacobian?.slice()}))};
            age=0;armed=false;
        },
        arm(ok) {armed=!!saved&&ok&&age<2;},
        canReuse(rows,fixed,tolerance) {
            if(!armed||!saved||rows.length!==saved.rows.length||fixed.length!==saved.fixed.length)return false;
            for(let i=0;i<fixed.length;i++)if(fixed[i]!==saved.fixed[i])return false;
            for(let i=0;i<rows.length;i++) {
                const a=rows[i],b=saved.rows[i];
                if(a.id!==b.id||a.kind!==b.kind||a.dofs.length!==b.dofs.length||
                    a.dofs.some((p,k)=>p!==b.dofs[k])||Number(a.kind==='length'||a.multiplier>tolerance)!==saved.mask[i])return false;
            }
            return true;
        },
        solve(rows,gradient,tolerance) {
            diagnostics.modifiedAttempts++;age++;armed=false;
            const w=packed,{matrix:A,scales,fixed,mask}=saved,F=new Float64Array(w.count);
            const g=gradient.slice();
            for(let i=0;i<rows.length;i++)if(!mask[i]) {
                const r=saved.rows[i],multiplier=rows[i].multiplier;
                r.dofs.forEach((p,k)=>{g[p]-=(r.kind==='wall'?-1:1)*multiplier*r.jacobian[k];});
                r.extraForceDofs?.forEach((p,k)=>{g[p]-=multiplier*r.extraForceJacobian[k];});
            }
            w.primal.forEach((p,i)=>{F[p]=fixed[i]?0:g[i];});
            for(let i=0;i<indices.length;i++) {
                const index=indices[i],r=saved.rows[index],current=rows[index];
                const movable=r.dofs.some((p,k)=>!fixed[p]&&r.jacobian[k]!==0);
                if(!movable&&(r.kind==='length'?Math.abs(current.gap)>tolerance:current.gap < -tolerance))
                    return {converged:false,factorizations:0,workingSetReuses:0};
                F[w.dual[i]]=movable?current.gap:current.multiplier;
            }
            const x=new Float64Array(w.count),before=lu.diagnostics.factorizations;
            let ok=lu.solve(A,F,scales,0,x);
            for(let i=0;i<x.length;i++)x[i]*=scales[i];
            let residual=Infinity;
            if(ok) {
                const {starts,ends,offsets}=w.band;
                residual=0;
                for(let i=0;i<w.count;i++) {let v=F[i];for(let j=starts[i];j<=ends[i];j++)v+=A[offsets[i]+j]*x[j];residual=Math.max(residual,Math.abs(v));}
                ok=Number.isFinite(residual)&&residual<=tolerance;
            }
            const increment=Float64Array.from(w.primal,p=>x[p]);
            const multiplierIncrement=Float64Array.from(rows,r=>-r.multiplier);
            indices.forEach((index,i)=>{multiplierIncrement[index]=x[w.dual[i]];});
            // A frozen working set may not release a contact or create a new
            // penetration in its current linear prediction. Refresh instead.
            if(ok)for(let i=0;i<rows.length;i++)if(rows[i].kind==='wall') {
                const r=rows[i];
                if(mask[i]&&r.multiplier+multiplierIncrement[i]<-tolerance ||
                    !mask[i]&&r.gap+r.dofs.reduce((sum,p,k)=>sum+r.jacobian[k]*increment[p],0)<-tolerance){ok=false;break;}
            }
            return {converged:ok,increment,multiplierIncrement,residual,
                factorizations:lu.diagnostics.factorizations-before,workingSetReuses:0};
        },
        dispose(){lu?.dispose();lu=null;packed=null;indices=null;invalidate();}
    };
}
