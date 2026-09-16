import {createCoulombBandLU,createCoulombBandLUArena} from '../../src/physics/kirchhoffCoulombBandLU.js';

const arena=createCoulombBandLUArena();

// Experimental elimination for identity-primal corrections only:
// x + J^T y = 0, J x + gap = 0 => J J^T y = gap.
// y includes the wall sign. Preserve the caller's active-set pivots and
// certify the original equations after recovery, not only the Gram system.
export function solveSharedAxisProjectionSchur(workspace,rows,fixed,tolerance) {
    const n=fixed.length;
    const indices=[];
    for(let i=0;i<rows.length;i++) {
        const r=rows[i];
        if(r.multiplier!==0||r.geometricHessian||r.extraForceDofs)return null;
        if(r.dofs.some((p,k)=>!fixed[p]&&r.jacobian[k]!==0))indices.push(i);
        else if(r.kind==='length'?Math.abs(r.gap)>tolerance:r.gap<-tolerance)return null;
    }
    indices.sort((a,b)=>Math.max(...rows[a].dofs)-Math.max(...rows[b].dofs)||a-b);
    const key=indices.map(i=>i+':'+rows[i].dofs.join('.')).join('/')+'/'+n;
    const caches=workspace.projectionSchurCaches??=new Map();
    let c=caches.get(key);
    if(!c) {
        const count=indices.length,incident=Array.from({length:n},()=>[]);
        indices.forEach((r,i)=>rows[r].dofs.forEach((p,k)=>incident[p].push([i,k])));
        let width=0;
        for(const list of incident)for(const [i] of list)for(const [j] of list)width=Math.max(width,Math.abs(i-j));
        const starts=Int32Array.from({length:count},(_,i)=>Math.max(0,i-width));
        const ends=Int32Array.from({length:count},(_,i)=>Math.min(count-1,i+width));
        let entries=0;const offsets=Int32Array.from(starts,(v,i)=>{const offset=entries-v;entries+=ends[i]-v+1;return offset;});
        const band={starts,ends,offsets,entries,kl:width,ku:width};
        c={incident,band,A:new Float64Array(entries),F:new Float64Array(count),scales:new Float64Array(count),y:new Float64Array(count),
            increment:new Float64Array(n),multipliers:new Float64Array(rows.length),lu:count?createCoulombBandLU(band,count,{arena}):null};
        if(caches.size>=8)caches.delete(caches.keys().next().value);
        caches.set(key,c);
    }
    const {A,F,scales,y,band,increment,multipliers}=c;
    A.fill(0);increment.fill(0);multipliers.fill(0);
    for(let p=0;p<n;p++)if(!fixed[p])for(const [i,k] of c.incident[p])for(const [j,l] of c.incident[p])
        A[band.offsets[i]+j]+=rows[indices[i]].jacobian[k]*rows[indices[j]].jacobian[l];
    for(let i=0;i<indices.length;i++) {
        F[i]=-rows[indices[i]].gap;
        let maximum=0;for(let j=band.starts[i];j<=band.ends[i];j++)maximum=Math.max(maximum,Math.abs(A[band.offsets[i]+j]));
        scales[i]=1/Math.sqrt(Math.max(maximum,1e-30));
    }
    if(c.lu&&!c.lu.solve(A,F,scales,0,y))return null;
    for(let i=0;i<indices.length;i++) {
        const r=rows[indices[i]],value=y[i]*scales[i];
        multipliers[indices[i]]=(r.kind==='wall'?-1:1)*value;
        for(let k=0;k<r.dofs.length;k++)if(!fixed[r.dofs[k]])increment[r.dofs[k]]-=r.jacobian[k]*value;
    }
    let residual=0;
    for(const i of indices) {
        const r=rows[i];let v=r.gap;
        for(let k=0;k<r.dofs.length;k++)v+=r.jacobian[k]*increment[r.dofs[k]];
        residual=Math.max(residual,Math.abs(v));
    }
    for(let p=0;p<n;p++)if(!fixed[p]) {
        let v=increment[p];
        for(const [i,k] of c.incident[p])v+=(rows[indices[i]].kind==='wall'?-1:1)*rows[indices[i]].jacobian[k]*multipliers[indices[i]];
        residual=Math.max(residual,Math.abs(v));
    }
    if(!Number.isFinite(residual)||residual>tolerance)return null;
    return {increment,multiplierIncrement:multipliers,residual,converged:true,factorizations:indices.length?1:0};
}
