/** Diagnostic dense solve of A x = b. No runtime integration or physical approval.
 * Complete row/column pivoting chooses a basic solution: nonpivot coordinates
 * of the scaled unknown are zero. This is NOT a minimum-norm solution or a
 * proof that the nullspace contains only force gauges. Callers must certify
 * every original physical equation before using a returned direction.
 */
export function solveCompositeRankRevealing({matrix,rhs,size, rowScales=null,columnScales=null,
    rankTolerance=64*Number.EPSILON*Math.max(1,size),backwardTolerance=1e-10,maxRefinements=2,maxSize=512}) {
    if(!Number.isInteger(size)||size<1||!Number.isInteger(maxSize)||maxSize<1||size>maxSize)throw new RangeError('Invalid or excessive dense size');
    const own=(v,n,name)=>{
        if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw new RangeError(`${name} requires finite entries`);
        return Float64Array.from(v);
    };
    const A=own(matrix,size*size,'matrix'),b=own(rhs,size,'rhs'),n=size;
    if(!(rankTolerance>0&&rankTolerance<1)||!(backwardTolerance>=0&&Number.isFinite(backwardTolerance))||
        !Number.isInteger(maxRefinements)||maxRefinements<0||maxRefinements>8)throw new RangeError('Invalid rank, residual or refinement limits');
    const positiveScales=(v,name)=>{const s=own(v,n,name);if(s.some(x=>x<=0))throw new RangeError(`${name} must be positive`);return s;};
    const R=rowScales===null?Float64Array.from({length:n},(_,i)=>{
        let m=0;for(let j=0;j<n;j++)m=Math.max(m,Math.abs(A[i*n+j]));return m===0?1:1/m;
    }):positiveScales(rowScales,'rowScales');
    const C=columnScales===null?Float64Array.from({length:n},(_,j)=>{
        let m=0;for(let i=0;i<n;i++)m=Math.max(m,Math.abs(A[i*n+j]*R[i]));return m===0?1:1/m;
    }):positiveScales(columnScales,'columnScales');
    const lu=Float64Array.from(A,(v,k)=>v*R[Math.floor(k/n)]*C[k%n]),rp=Int32Array.from({length:n},(_,i)=>i),cp=rp.slice();
    if(!lu.every(Number.isFinite)||!R.every(Number.isFinite)||!C.every(Number.isFinite))throw new RangeError('Nonfinite equilibrated matrix');
    let maximum=0;for(const v of lu)maximum=Math.max(maximum,Math.abs(v));
    const rankThreshold=maximum*rankTolerance,pivots=[];let rank=0;
    for(let k=0;k<n;k++) {
        let largest=0,pr=k,pc=k;
        for(let i=k;i<n;i++)for(let j=k;j<n;j++)if(Math.abs(lu[i*n+j])>largest){largest=Math.abs(lu[i*n+j]);pr=i;pc=j;}
        if(largest<=rankThreshold)break;
        if(pr!==k){for(let j=0;j<n;j++)[lu[k*n+j],lu[pr*n+j]]=[lu[pr*n+j],lu[k*n+j]];[rp[k],rp[pr]]=[rp[pr],rp[k]];}
        if(pc!==k){for(let i=0;i<n;i++)[lu[i*n+k],lu[i*n+pc]]=[lu[i*n+pc],lu[i*n+k]];[cp[k],cp[pc]]=[cp[pc],cp[k]];}
        const pivot=lu[k*n+k];pivots.push(Math.abs(pivot));rank++;
        for(let i=k+1;i<n;i++) {
            const factor=lu[i*n+k]/pivot;lu[i*n+k]=factor;
            for(let j=k+1;j<n;j++)lu[i*n+j]-=factor*lu[k*n+j];
        }
    }
    if(!lu.every(Number.isFinite))throw new RangeError('Nonfinite complete-pivot factors');
    const factoredSolve=v=>{
        const y=Float64Array.from(rp,i=>R[i]*v[i]),z=new Float64Array(n),x=new Float64Array(n);
        for(let i=0;i<n;i++)for(let j=0;j<Math.min(i,rank);j++)y[i]-=lu[i*n+j]*y[j];
        for(let i=rank-1;i>=0;i--){let value=y[i];for(let j=i+1;j<rank;j++)value-=lu[i*n+j]*z[j];z[i]=value/lu[i*n+i];}
        for(let i=0;i<n;i++)x[cp[i]]=C[cp[i]]*z[i];return x;
    };
    const measure=x=>{
        const residual=new Float64Array(n);let maximumResidual=0,componentwiseBackwardError=0;
        for(let i=0;i<n;i++) {
            let sum=-b[i],correction=0,denominator=Math.abs(b[i]);
            for(let j=0;j<n;j++) {
                const product=A[i*n+j]*x[j],term=product-correction,next=sum+term;
                correction=(next-sum)-term;sum=next;denominator+=Math.abs(product);
            }
            if(!Number.isFinite(denominator))throw new RangeError('Nonfinite original residual scale');
            residual[i]=sum;maximumResidual=Math.max(maximumResidual,Math.abs(sum));
            componentwiseBackwardError=Math.max(componentwiseBackwardError,denominator===0?(sum===0?0:Infinity):Math.abs(sum)/denominator);
        }
        if(!x.every(Number.isFinite)||!residual.every(Number.isFinite)||!Number.isFinite(componentwiseBackwardError))throw new RangeError('Nonfinite original dense residual');
        return {residual,maximumResidual,componentwiseBackwardError};
    };
    let solution=factoredSolve(b),proof=measure(solution),refinements=0;
    for(let i=0;i<maxRefinements&&proof.componentwiseBackwardError>0;i++) {
        const correction=factoredSolve(Float64Array.from(proof.residual,v=>-v)),trial=Float64Array.from(solution,(v,j)=>v+correction[j]),next=measure(trial);
        refinements++;
        if(next.componentwiseBackwardError>=proof.componentwiseBackwardError)break;
        solution=trial;proof=next;
    }
    return {solution,...proof,rank,nullity:n-rank,rankThreshold,pivots:Float64Array.from(pivots),rowPermutation:rp,columnPermutation:cp,
        rowScales:R,columnScales:C,refinements,compatible:proof.componentwiseBackwardError<=backwardTolerance,
        solutionKind:'complete-pivot-basic',certified:false,nonlinearStepAccepted:false};
}
