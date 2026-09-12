import assert from 'node:assert/strict';
import test from 'node:test';
import { solveCoupledFrictionQP } from '../src/physics/kirchhoffCoupledFrictionSolver.js';

function banded(A) {
    const n = A.length, out = new Float64Array(n*n);
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) out[i*n+i-j] = A[i][j];
    return out;
}
function denseSolve(A, b) {
    const n = b.length, a = A.map((row, i) => [...row, b[i]]);
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k+1; i < n; i++) if (Math.abs(a[i][k]) > Math.abs(a[pivot][k])) pivot = i;
        [a[k], a[pivot]] = [a[pivot], a[k]];
        for (let i = k+1; i < n; i++) {
            const f = a[i][k]/a[k][k];
            for (let j = k+1; j <= n; j++) a[i][j] -= f*a[k][j];
        }
    }
    const x = new Float64Array(n);
    for (let i = n-1; i >= 0; i--) {
        let v = a[i][n]; for (let j = i+1; j<n; j++) v-=a[i][j]*x[j]; x[i]=v/a[i][i];
    }
    return x;
}
// Independent dense oracle: enumerate normal activity, then bracket the ONE
// ellipse multiplier in (A + eta D)x = b - eta D lambdaOld.
function oracle(A, b, normalRows, group, lower) {
    const n = b.length, [u,v] = group.rows, [a,c] = group.radii;
    for (let mask=0; mask<1<<normalRows.length; mask++) {
        const fixed = normalRows.filter((_,k)=>!(mask&1<<k));
        const free = Array.from({length:n},(_,i)=>i).filter(i=>!fixed.includes(i));
        const evaluate = eta => {
            const x = new Float64Array(n); for (const i of fixed) x[i]=lower[i];
            const D = new Float64Array(n); D[u]=eta/(a*a); D[v]=eta/(c*c);
            const rhs=b.map((r,i)=>r-D[i]*(i===u?group.lambda[0]:i===v?group.lambda[1]:0)-fixed.reduce((s,j)=>s+A[i][j]*x[j],0));
            const sol=denseSolve(free.map(i=>free.map(j=>A[i][j]+(i===j?D[i]:0))),free.map(i=>rhs[i]));
            free.forEach((i,k)=>x[i]=sol[k]);
            return x;
        };
        const norm=x=>Math.hypot((x[u]+group.lambda[0])/a,(x[v]+group.lambda[1])/c);
        let x=evaluate(0);
        if (norm(x)>1) {
            let hi=1; while(norm(evaluate(hi))>1) hi*=2;
            let lo=0; for(let k=0;k<100;k++){const mid=(lo+hi)/2;if(norm(evaluate(mid))>1)lo=mid;else hi=mid;}
            x=evaluate(hi);
        }
        if(normalRows.some(i=>x[i]<lower[i]-1e-10))continue;
        const r=b.map((v,i)=>v-A[i].reduce((s,a,j)=>s+a*x[j],0));
        if(fixed.some(i=>r[i]>1e-9))continue;
        return x;
    }
    throw new Error('No dense disk solution');
}

for(const radius of [2,1e-15]) test(`isotropic disk keeps simultaneous U/V within radius ${radius}`,()=>{
    const A=[[1,0],[0,1]], b=new Float64Array([3,4]);
    const r=solveCoupledFrictionQP(banded(A),b,new Float64Array(2).fill(-Infinity),new Float64Array(2).fill(Infinity),2,2,
        [{rows:[0,1],radii:[radius,radius],lambda:[0,0]}],{tolerance:1e-9});
    assert.ok(r.diagnostics.converged,JSON.stringify(r.diagnostics));
    assert.ok(Math.abs(r.increment[0]/radius-0.6)<1e-8);
    assert.ok(Math.abs(r.increment[1]/radius-0.8)<1e-8);
});

for(const warm of [false,true]) for(const radii of [[0.4,0.4],[0.05,0.8]]) test(`normal+material+ellipse matches dense multiplier oracle (${warm},${radii})`,()=>{
    const n=7,A=Array.from({length:n},()=>new Array(n).fill(0));
    for(let i=0;i<n;i++)for(let j=0;j<n;j++)A[i][j]=(i===j?2:0)+0.25*Math.cos(i-j)+0.1*Math.sin(i+2)*Math.sin(j+2);
    const b=new Float64Array([0.5,-0.2,-0.3,1.2,0.6,3,-2]);
    const lower=new Float64Array(n).fill(-Infinity),upper=new Float64Array(n).fill(Infinity);
    lower[3]=warm?-0.1:0;lower[4]=0;
    const group={rows:[5,6],radii,lambda:warm?[0.08,-0.1]:[0,0]};
    const expected=oracle(A,b,[3,4],group,lower);
    const actual=solveCoupledFrictionQP(banded(A),b,lower,upper,n,n,[group],{tolerance:1e-9});
    assert.ok(actual.diagnostics.converged,JSON.stringify(actual.diagnostics));
    actual.increment.forEach((v,i)=>assert.ok(Math.abs(v-expected[i])<1e-8,`${i}: ${v} vs ${expected[i]}`));
});

for (let sample=0; sample<12; sample++) test(`multiple disjoint groups satisfy a manufactured full KKT solution ${sample}`,()=>{
    const n=10, A=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>(i===j?2:0)+0.2*Math.cos(i-j)+0.1*Math.sin(i+1)*Math.sin(j+1)));
    const groups=[{rows:[1,8],radii:[0.3,0.3],lambda:[0.1,-0.1]}, {rows:[3,5],radii:[0.05,0.7],lambda:[-0.08,0.15]}];
    const x=new Float64Array(n), r=new Float64Array(n), lower=new Float64Array(n).fill(-Infinity), upper=new Float64Array(n).fill(Infinity);
    for(let i=0;i<n;i++)x[i]=0.1*Math.sin(i+sample);
    for(const [k,g] of groups.entries()) {
        const theta=sample*0.4+k, active=(sample+k)%3!==0, scale=active?1:0.4;
        const u=g.radii[0]*Math.cos(theta)*scale,v=g.radii[1]*Math.sin(theta)*scale;
        x[g.rows[0]]=u-g.lambda[0];x[g.rows[1]]=v-g.lambda[1];
        if(active){const eta=0.05;r[g.rows[0]]=eta*u/g.radii[0]**2;r[g.rows[1]]=eta*v/g.radii[1]**2;}
    }
    for(const i of [2,6]){lower[i]=0;x[i]=(sample+i)%2?0:0.1;r[i]=x[i]===0?-0.2:0;}
    const b=new Float64Array(A.map((row,i)=>row.reduce((s,a,j)=>s+a*x[j],r[i])));
    const actual=solveCoupledFrictionQP(banded(A),b,lower,upper,n,n,groups,{tolerance:1e-9});
    assert.ok(actual.diagnostics.converged,JSON.stringify(actual.diagnostics));
    actual.increment.forEach((v,i)=>assert.ok(Math.abs(v-x[i])<2e-8,`${i}: ${v} vs ${x[i]}`));
});
