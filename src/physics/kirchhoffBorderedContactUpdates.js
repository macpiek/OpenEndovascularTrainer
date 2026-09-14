// Keep the original compact KKT factor. Released base contacts replace a dual
// row; new contacts border that system. Only the small Schur system is dense.
// All support, coefficients and multipliers are immutable for this owner.
export function createBorderedContactUpdates(w,lu,indices,rows) {
    indices=indices.slice();
    const indexSet=new Set(indices),baseF=w.residual.slice(),matrix=w.matrix.slice(),{primal,dual,count,band}=w;
    const {starts,ends,offsets}=band,invCache=new Map(),fixed=w.fixed.slice();
    const diagnostics={updates:0,rejections:0,rankResets:0,ranks:{}};
    const dot=(a,b)=>{let v=0;for(let i=0;i<count;i++)v+=a[i]*b[i];return v;};
    function inverseColumn(key,column) {
        let result=invCache.get(key);
        if(!result){result=new Float64Array(count);if(!lu.inverse(column,result))return null;if(invCache.size<32)invCache.set(key,result);}
        return result;
    }
    function smallSolve(A,b,n) {
        for(let k=0;k<n;k++) {
            let p=k;for(let i=k+1;i<n;i++)if(Math.abs(A[i*n+k])>Math.abs(A[p*n+k]))p=i;
            if(!Number.isFinite(A[p*n+k])||Math.abs(A[p*n+k])<1e-14)return false;
            if(p!==k){for(let j=k;j<n;j++){const v=A[k*n+j];A[k*n+j]=A[p*n+j];A[p*n+j]=v;}const v=b[k];b[k]=b[p];b[p]=v;}
            for(let i=k+1;i<n;i++){const q=A[i*n+k]/A[k*n+k];for(let j=k+1;j<n;j++)A[i*n+j]-=q*A[k*n+j];b[i]-=q*b[k];}
        }
        for(let i=n-1;i>=0;i--){for(let j=i+1;j<n;j++)b[i]-=A[i*n+j]*b[j];b[i]/=A[i*n+i];}
        return b.every(Number.isFinite);
    }
    return {diagnostics,solve(activeSet,tolerance,out,maxRank=8) {
        const removed=indices.flatMap((index,i)=>activeSet[index]?[]:[i]),added=[];
        for(let i=0;i<rows.length;i++)if(activeSet[i]&&!indexSet.has(i))added.push(i);
        const rank=removed.length+added.length;
        diagnostics.ranks[rank]=(diagnostics.ranks[rank]??0)+1;
        if(rank>maxRank){diagnostics.rankResets++;return null;}
        const reject=()=>{diagnostics.rejections++;return null;};
        const F=baseF.slice(),columns=[],equations=[],responses=[];
        for(const i of removed) {
            const d=dual[i],column=new Float64Array(count),equation=new Float64Array(count);column[d]=1;
            for(let j=starts[d];j<=ends[d];j++)equation[j]=-matrix[offsets[d]+j];equation[d]+=1;
            F[d]=rows[indices[i]].multiplier;
            columns.push(column);equations.push(equation);responses.push(inverseColumn(`remove/${i}`,column));
        }
        for(const index of added) {
            const r=rows[index],column=new Float64Array(count),equation=new Float64Array(count),sign=r.kind==='wall'?-1:1;
            for(let k=0;k<r.dofs.length;k++){const p=primal[r.dofs[k]];if(!fixed[p]){column[p]+=sign*r.jacobian[k];equation[p]+=r.jacobian[k];}}
            for(let k=0;k<(r.extraForceDofs?.length??0);k++){const p=primal[r.extraForceDofs[k]];if(!fixed[p])column[p]+=r.extraForceJacobian[k];}
            // This reaction was eliminated from the base's primal RHS.
            for(let i=0;i<count;i++)F[i]+=r.multiplier*column[i];
            columns.push(column);equations.push(equation);responses.push(inverseColumn(`add/${index}`,column));
        }
        if(responses.some(v=>v===null))return reject();
        const small=new Float64Array(rank*rank);
        for(let i=0;i<rank;i++)for(let j=0;j<rank;j++)small[i*rank+j]=dot(equations[i],responses[j])+(i===j&&i<removed.length?1:0);
        const solveFor=(top,bottom)=>{
            const x=new Float64Array(count),rhs=Float64Array.from(top,v=>-v),c=new Float64Array(rank);
            if(!lu.inverse(rhs,x))return null;
            for(let i=0;i<rank;i++)c[i]=dot(equations[i],x)+(i<removed.length?0:bottom[i-removed.length]);
            if(!smallSolve(small.slice(),c,rank))return null;
            for(let i=0;i<count;i++)for(let j=0;j<rank;j++)x[i]-=responses[j][i]*c[j];
            return {x,c};
        };
        const solved=solveFor(F,added.map(i=>rows[i].gap));if(!solved)return reject();
        const {x,c}=solved;
        // Certify every original equation, including new border equations.
        // Refinement uses the same retained factors and small border system.
        let residual=Infinity;
        for(let attempt=0;attempt<3;attempt++) {
            const error=new Float64Array(count),bottom=new Float64Array(added.length);residual=0;
            for(let i=0;i<count;i++) {
                let sum=F[i],correction=0;
                const add=v=>{const next=sum+v;correction+=Math.abs(sum)>=Math.abs(v)?(sum-next)+v:(v-next)+sum;sum=next;};
                for(let j=starts[i];j<=ends[i];j++)add(matrix[offsets[i]+j]*x[j]);
                for(let j=0;j<removed.length;j++)if(i===dual[removed[j]])add(dot(equations[j],x));
                for(let j=removed.length;j<rank;j++)add(columns[j][i]*c[j]);
                error[i]=sum+correction;residual=Math.max(residual,Math.abs(error[i]));
            }
            for(let j=0;j<added.length;j++){bottom[j]=dot(equations[j+removed.length],x)+rows[added[j]].gap;residual=Math.max(residual,Math.abs(bottom[j]));}
            if(!Number.isFinite(residual))return reject();
            if(residual<=tolerance)break;
            if(attempt===2)return reject();
            const correction=solveFor(error,bottom);if(!correction)return reject();
            for(let i=0;i<count;i++)x[i]+=correction.x[i];
            for(let i=0;i<rank;i++)c[i]+=correction.c[i];
        }
        for(let i=0;i<primal.length;i++)out.increment[i]=x[primal[i]];
        for(let i=0;i<rows.length;i++)out.multiplierIncrement[i]=-rows[i].multiplier;
        for(let i=0;i<indices.length;i++)out.multiplierIncrement[indices[i]]=x[dual[i]];
        for(let i=0;i<added.length;i++)out.multiplierIncrement[added[i]]=c[removed.length+i];
        diagnostics.updates++;
        return {increment:out.increment,multiplierIncrement:out.multiplierIncrement,factorizations:0,residual,converged:true};
    }};
}
