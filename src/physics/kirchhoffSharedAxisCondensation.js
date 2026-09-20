import {sharedAxisEffectiveGap} from './kirchhoffSharedAxisCompliance.js';

/** Eliminate an active compliant normal reaction from a fixed working set.
 * J dx + C dλ = -(g + C λ), stationarity column B = -Jᵀ + friction.
 * The primal matrix gains -B J/C and its residual gains -B(g+Cλ)/C.
 * This is exact block elimination, not a new contact law or stopping rule.
 */
export function canCondenseSharedAxisContact(row,layout) {
    if(!(row.compliance>0)||row.kind!=='wall')return false;
    const forceDofs=[...row.dofs,...(row.extraForceDofs??[])];
    return forceDofs.every(p=>row.dofs.every(q=>Math.abs(p-q)<layout.band));
}

export function assembleCondensedSharedAxisContacts(w,rows) {
    const {matrix:A,residual:F,primal,band:{starts,ends,offsets}}=w;
    for(const r of rows) {
        const inverse=1/r.compliance,gap=sharedAxisEffectiveGap(r);
        const scatter=(p,b)=>{
            if(!b)return;
            const i=primal[p],scale=-b*inverse;
            F[i]+=scale*gap;
            for(let k=0;k<r.dofs.length;k++) {
                const value=scale*r.jacobian[k];if(!value)continue;
                const j=primal[r.dofs[k]];
                if(j<starts[i]||j>ends[i])throw new RangeError('Condensed contact exceeds primal band');
                A[offsets[i]+j]+=value;
            }
        };
        for(let k=0;k<r.dofs.length;k++)scatter(r.dofs[k],-r.jacobian[k]);
        for(let k=0;k<(r.extraForceDofs?.length??0);k++)scatter(r.extraForceDofs[k],r.extraForceJacobian[k]);
    }
}

export function recoverCondensedSharedAxisReaction(row,dx) {
    let gap=sharedAxisEffectiveGap(row);
    for(let k=0;k<row.dofs.length;k++)gap+=row.jacobian[k]*dx[row.dofs[k]];
    return -gap/row.compliance;
}

/** Check the ORIGINAL uncondensed equations, including nonsymmetric friction
 * columns, all geometric Hessians, fixed support and inactive dual equations.
 * Condensation can amplify cancellation; a failed check uses the original LU.
 */
export function measureUncondensedSharedAxisResidual(chain,{rows,gradient,fixed},activeSet,dx,dl) {
    const n=gradient.length,values=Float64Array.from(gradient),errors=new Float64Array(n);
    const add=(i,x)=>{
        const old=values[i],sum=old+x;
        errors[i]+=Math.abs(old)>=Math.abs(x)?(old-sum)+x:(x-sum)+old;
        values[i]=sum;
    };
    const half=chain.layout.band-1,width=2*half+1;
    for(let i=0;i<n;i++)for(let j=Math.max(0,i-half);j<=Math.min(n-1,i+half);j++) {
        const h=chain.tangent?chain.tangent[i*width+j-i+half]:
            chain.hessian[Math.max(i,j)*chain.layout.band+Math.abs(i-j)];
        add(i,h*dx[j]);
    }
    let residual=0;
    for(let index=0;index<rows.length;index++) {
        const r=rows[index],m=r.dofs.length,sign=r.kind==='wall'?-1:1;
        for(let k=0;k<m;k++)add(r.dofs[k],sign*r.jacobian[k]*dl[index]);
        for(let k=0;k<(r.extraForceDofs?.length??0);k++)add(r.extraForceDofs[k],r.extraForceJacobian[k]*dl[index]);
        if(r.geometricHessian)for(let i=0;i<m;i++)for(let j=0;j<m;j++)
            add(r.dofs[i],r.geometricHessian[i*m+j]*dx[r.dofs[j]]);
        const movable=r.dofs.some((p,k)=>!fixed[p]&&r.jacobian[k]!==0);
        let constraint=dl[index]+r.multiplier;
        if((r.kind==='length'||activeSet[index])&&(movable||r.compliance)) {
            constraint=sharedAxisEffectiveGap(r)+(r.compliance??0)*dl[index];
            for(let k=0;k<m;k++)constraint+=r.jacobian[k]*dx[r.dofs[k]];
        }
        residual=Math.max(residual,Math.abs(constraint));
    }
    for(let i=0;i<n;i++)residual=Math.max(residual,Math.abs(fixed[i]?dx[i]:values[i]+errors[i]));
    return Number.isFinite(residual)?residual:Infinity;
}
