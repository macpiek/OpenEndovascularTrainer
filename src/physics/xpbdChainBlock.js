/** Natural-map complementarity residual, in constraint units. A releasing
 * contact must approach zero continuously as its reaction vanishes, even when
 * its gap is already positive. Testing lambda > 0 as an exact active flag in
 * a line-search merit makes an arbitrarily tiny reaction close a large gap.
 */
export function xpbdConstraintResidual(row,diagonal) {
    const residual=row.gap+row.alpha*(row.lambda??0);
    return row.unilateral?Math.min(diagonal*(row.lambda??0),residual):residual;
}

/** Research global XPBD step. Rows are scalar constraints in spatial order;
 * each owns gap, alpha=compliance/dt^2, accumulated lambda and sparse J.
 * Solve the entire chain, including active unilateral rows, at once. The
 * nonlinear primal residual is retained (local XPBD normally omits it).
 */
export function solveXpbdChainBlock(rows,weights,displacement,{maxActiveSetIterations=32,balancePrimal=true}={}) {
    const active=rows.map(r=>!r.unilateral||r.lambda>0||r.gap<0);
    const lambda=Float64Array.from(rows,r=>r.lambda??0);let factorizations=0;
    let base=displacement;
    if(!balancePrimal) {
        // Original XPBD ignores the nonlinear primal residual. Still retain
        // impulses here so removing an active contact undoes its old impulse.
        base=new Float64Array(weights.length);
        for(const r of rows)for(let j=0;j<r.dofs.length;j++)base[r.dofs[j]]+=weights[r.dofs[j]]*r.jacobian[j]*(r.lambda??0);
    }
    for(let iteration=0;iteration<maxActiveSetIterations;iteration++) {
        const residual=Float64Array.from(base);
        for(let i=0;i<rows.length;i++) {
            if(!active[i])lambda[i]=0;
            const r=rows[i];for(let j=0;j<r.dofs.length;j++)residual[r.dofs[j]]-=weights[r.dofs[j]]*r.jacobian[j]*lambda[i];
        }
        const included=[],scales=[];
        for(let i=0;i<rows.length;i++)if(active[i]) {
            const r=rows[i],diagonal=r.alpha+r.dofs.reduce((v,d,j)=>v+weights[d]*r.jacobian[j]**2,0);
            if(diagonal>0){included.push(i);scales.push(1/Math.sqrt(diagonal));}
            else if(Math.abs(r.gap)>1e-9)throw new Error('Inconsistent prescribed XPBD constraint');
        }
        const supports=Array.from({length:weights.length},()=>[]),count=included.length;
        for(let i=0;i<count;i++) {
            const r=rows[included[i]];
            for(let j=0;j<r.dofs.length;j++)if(weights[r.dofs[j]]&&r.jacobian[j])supports[r.dofs[j]].push([i,r.jacobian[j]*scales[i]]);
        }
        let band=1;for(const support of supports)if(support.length)band=Math.max(band,1+support.at(-1)[0]-support[0][0]);
        const a=new Float64Array(count*band),rhs=new Float64Array(count);
        for(let d=0;d<supports.length;d++)for(const [i,ji] of supports[d])for(const [j,jj] of supports[d])
            if(i>=j)a[i*band+i-j]+=weights[d]*ji*jj;
        for(let i=0;i<count;i++) {
            const r=rows[included[i]];a[i*band]+=r.alpha*scales[i]**2;
            rhs[i]=scales[i]*(-r.gap-r.alpha*lambda[included[i]]+r.dofs.reduce((v,d,j)=>v+r.jacobian[j]*residual[d],0));
        }
        for(let i=0;i<count;i++)for(let j=Math.max(0,i-band+1);j<=i;j++) {
            let v=a[i*band+i-j];
            for(let k=Math.max(0,i-band+1);k<j;k++)v-=a[i*band+i-k]*a[j*band+j-k];
            if(i===j){if(!(v>0&&Number.isFinite(v)))throw new Error('XPBD chain block lost positive definiteness');a[i*band]=Math.sqrt(v);}
            else a[i*band+i-j]=v/a[j*band];
        }
        factorizations++;
        for(let i=0;i<count;i++){for(let j=Math.max(0,i-band+1);j<i;j++)rhs[i]-=a[i*band+i-j]*rhs[j];rhs[i]/=a[i*band];}
        for(let i=count-1;i>=0;i--){for(let j=i+1;j<Math.min(count,i+band);j++)rhs[i]-=a[j*band+j-i]*rhs[j];rhs[i]/=a[i*band];}
        const increment=Float64Array.from(residual,v=>-v),nextLambda=lambda.slice();
        for(let i=0;i<count;i++) {
            const index=included[i],r=rows[index],delta=rhs[i]*scales[i];nextLambda[index]+=delta;
            for(let j=0;j<r.dofs.length;j++)increment[r.dofs[j]]+=weights[r.dofs[j]]*r.jacobian[j]*delta;
        }
        // A simultaneous release/re-add can cycle even for this convex dual
        // problem. Walk to the first nonnegative-reaction boundary instead,
        // preserving a feasible dual iterate, then release that single row.
        let blocking=-1,fraction=1;
        for(let i=0;i<rows.length;i++)if(rows[i].unilateral&&active[i]&&nextLambda[i]<-1e-12) {
            const bound=lambda[i]/(lambda[i]-nextLambda[i]);
            if(bound<fraction){fraction=bound;blocking=i;}
        }
        if(blocking>=0) {
            for(let i=0;i<rows.length;i++)lambda[i]+=fraction*(nextLambda[i]-lambda[i]);
            active[blocking]=false;lambda[blocking]=0;continue;
        }
        let entering=-1,worst=-1e-10;
        for(let i=0;i<rows.length;i++)if(rows[i].unilateral&&!active[i]) {
            const r=rows[i],gap=r.gap+r.dofs.reduce((v,d,j)=>v+r.jacobian[j]*increment[d],0);
            if(gap<worst){worst=gap;entering=i;}
        }
        if(entering>=0){lambda.set(nextLambda);active[entering]=true;continue;}
        for(let i=0;i<rows.length;i++)if(rows[i].unilateral)nextLambda[i]=Math.max(0,nextLambda[i]);
        return {increment,lambda:nextLambda,factorizations,band,rows:count,activeSetIterations:iteration+1};
    }
    throw new Error('XPBD chain active set did not converge');
}
