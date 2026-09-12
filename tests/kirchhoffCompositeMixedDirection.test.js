import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout,createCompositeChainWorkspace} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeMixedWorkspace,solveCompositeMixedDirection} from '../src/physics/kirchhoffCompositeMixedDirection.js';
const tol={force:1e-10,torque:1e-10,constraint:1e-11};
const close=(a,b,t=1e-10)=>assert.ok(Math.abs(a-b)<t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function fixture(n=3,wall=false){
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>['wire','catheter'])),chain=createCompositeChainWorkspace(layout);
    const definitions=Array.from({length:n-1},(_,edge)=>({kind:'length',edge,dofs:[layout.positions[edge],layout.positions[edge+1]].flatMap(s=>[s,s+1,s+2])}));
    if(wall)definitions.push({...definitions[0],kind:'wall'});
    const w=createCompositeMixedWorkspace(layout,definitions),fixed=new Uint8Array(layout.dofCount).fill(1);
    for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=i+1;
    const rows=definitions.map(d=>({...d,gap:0,multiplier:0,jacobian:[-1,0,0,1,0,0]}));
    return {layout,chain,definitions,w,fixed,rows};
}

test('length and original material forces solve together in one local band with the analytic axial reaction',()=>{
    const f=fixture(),{layout,chain,fixed,rows}=f,p=layout.positions;
    fixed[p[1]]=fixed[p[2]]=0;chain.hessian[p[1]*layout.band]=2;chain.hessian[p[2]*layout.band]=3;
    chain.gradient[p[2]]=-1.4;rows[0].gap=.1;rows[1].gap=.2;
    const before={h:chain.hessian.slice(),g:chain.gradient.slice(),rows:structuredClone(rows)};
    const r=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(r.converged,JSON.stringify(r.proof));close(r.increment[p[1]],-.1);close(r.increment[p[2]],-.3);
    close(r.multiplierIncrement[0],2.5);close(r.multiplierIncrement[1],2.3);assert.equal(r.factorizations,1);
    assert.deepEqual(chain.hessian,before.h);assert.deepEqual(chain.gradient,before.g);assert.deepEqual(rows,before.rows);
    for(let i=0;i<fixed.length;i++)if(fixed[i])assert.equal(r.increment[i],0);
});

test('active wall normal is a common physical reaction and an open row releases its stale load',()=>{
    const f=fixture(3,true),{layout,chain,rows,fixed}=f,q=layout.positions[1]+1;
    fixed[q]=0;chain.hessian[q*layout.band]=3;chain.gradient[q]=1;
    Object.assign(rows[2],{gap:-.1,jacobian:[0,0,0,0,1,0],penalty:10});
    const r=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(r.converged);close(r.increment[q],.1);close(r.multiplierIncrement[2],1.3);
    Object.assign(rows[2],{gap:.5,multiplier:.2});chain.gradient[q]=.8;
    const open=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(open.converged);close(open.multiplierIncrement[2],-.2);close(open.increment[q],-1/3);
});

test('a scaled SDF gap derivative does not rescale the physical normal force or its release',()=>{
    const f=fixture(3,true),{layout,chain,rows,fixed}=f,q=layout.positions[1]+1;
    fixed[q]=0;chain.hessian[q*layout.band]=3;chain.gradient[q]=1;
    Object.assign(rows[2],{gap:-.1,jacobian:[0,0,0,0,2,0],forceColumn:[0,0,0,0,-1,0],penalty:10});
    const r=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(r.converged);close(r.increment[q],.05);close(r.multiplierIncrement[2],1.15);
    Object.assign(rows[2],{gap:.5,multiplier:.2});chain.gradient[q]=.8;
    const open=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(open.converged);close(open.multiplierIncrement[2],-.2);close(open.increment[q],-1/3);
    rows[2].forceColumn=[0];
    assert.throws(()=>solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol}),/force column/);
});

test('a signed private SDF cone iterate solves the same inactive NCP equation and exact mechanical residual',()=>{
    const f=fixture(3,true),{layout,chain,rows,fixed}=f,q=layout.positions[1]+1;
    f.definitions[2].sdfBranch=0;const w=createCompositeMixedWorkspace(layout,f.definitions);
    fixed[q]=0;chain.hessian[q*layout.band]=3;
    // Base force -1, signed trial contact force -.2: Rq=-1-(-.2)=-.8.
    // Inactive NCP corrects deltaFn=+.2 and dq=+1/3, releasing into g>0.
    chain.gradient[q]=-.8;
    Object.assign(rows[2],{gap:0,multiplier:-.2,jacobian:[0,0,0,0,1,0],forceColumn:[0,0,0,0,-1,0],penalty:10});
    assert.throws(()=>solveCompositeMixedDirection(w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol}),/nonnegative/);
    rows[2].allowSignedWallIterate=true;
    const r=solveCompositeMixedDirection(w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});
    assert.ok(r.converged);close(r.increment[q],1/3);close(r.multiplierIncrement[2],.2);
    close(rows[2].multiplier+r.multiplierIncrement[2],0);
    close(3*r.increment[q]-r.multiplierIncrement[2]+chain.gradient[q],0);
    assert.equal(rows[2].multiplier,-.2,'linear solve never commits or clips the caller trial');
});

test('ordinary wall rows cannot opt into the private SDF cone signed-iterate contract',()=>{
    const f=fixture(3,true),q=f.layout.positions[1]+1;f.fixed[q]=0;
    Object.assign(f.rows[2],{gap:0,multiplier:-.2,jacobian:[0,0,0,0,1,0],penalty:10});
    assert.throws(()=>solveCompositeMixedDirection(f.w,f.chain,{rows:f.rows,gradient:f.chain.gradient,fixed:f.fixed,tolerances:tol}),/nonnegative/);
    f.rows[2].allowSignedWallIterate=true;
    assert.throws(()=>solveCompositeMixedDirection(f.w,f.chain,{rows:f.rows,gradient:f.chain.gradient,fixed:f.fixed,tolerances:tol}),/explicit SDF cone/);
});

test('indefinite signed stress tangent is kept instead of an SPD floor',()=>{
    const f=fixture(),q=f.layout.positions[1]+1;f.fixed[q]=0;f.chain.gradient[q]=1;
    f.rows[0].geometricHessian=new Float64Array(36);f.rows[0].geometricHessian[4*6+4]=-20;
    // Give the local constraint a free axial coordinate too, so its exact
    // stress tangent is assembled even though only transverse force is loaded.
    f.fixed[f.layout.positions[1]]=f.fixed[f.layout.positions[2]]=0;
    const r=solveCompositeMixedDirection(f.w,f.chain,{rows:f.rows,gradient:f.chain.gradient,fixed:f.fixed,tolerances:tol});
    assert.ok(r.converged);close(r.increment[q],-1/(q+1-20));
});

test('fixed incompatible original geometry fails even when every free mechanical residual is zero',()=>{
    const f=fixture();f.rows[0].gap=.01;
    const r=solveCompositeMixedDirection(f.w,f.chain,{rows:f.rows,gradient:f.chain.gradient,fixed:f.fixed,tolerances:tol});
    assert.equal(r.converged,false);assert.equal(r.proof.constraint,.01);
});

test('an invalid Hessian is rejected before solving even when its stale bytes are finite',()=>{
    const f=fixture();f.chain.hessianValid=false;
    assert.throws(()=>solveCompositeMixedDirection(f.w,f.chain,{rows:f.rows,gradient:f.chain.gradient,fixed:f.fixed,tolerances:tol}),/fresh full Hessian/);
});

test('interleaving bounds storage linearly as the remote guidewire gets longer',()=>{
    const samples=[5,65,201].map(n=>fixture(n,true).w);
    for(const w of samples){assert.ok(w.packedLayout.kl<=16);assert.ok(w.matrix.length<35*w.count);}
    assert.ok(samples[2].matrix.length/samples[1].matrix.length<3.3);
});

test('an independently formed dense original KKT system matches every primal and multiplier coordinate',()=>{
    const f=fixture(5),{layout,chain,rows,fixed}=f,n=layout.dofCount;
    fixed.fill(0);fixed[0]=fixed[1]=fixed[2]=1;
    rows.forEach((r,i)=>{r.jacobian=[-.8,-.6,0,.8,.6,0];r.gap=.001*(i+1);});
    for(let i=0;i<n;i++)chain.gradient[i]=.01*Math.sin(i+.3);
    const m=n+rows.length,A=Array.from({length:m},()=>new Float64Array(m)),b=new Float64Array(m);
    for(let i=0;i<n;i++){A[i][i]=fixed[i]?1:i+1;b[i]=fixed[i]?0:-chain.gradient[i];}
    rows.forEach((row,r)=>{b[n+r]=-row.gap;row.dofs.forEach((d,j)=>{if(!fixed[d])A[d][n+r]=A[n+r][d]=row.jacobian[j];});});
    // Independent dense partial-pivot elimination, no production packed map,
    // scales, LU or residual calculator used for this oracle.
    for(let k=0;k<m;k++){
        let pivot=k;for(let i=k+1;i<m;i++)if(Math.abs(A[i][k])>Math.abs(A[pivot][k]))pivot=i;
        [A[k],A[pivot]]=[A[pivot],A[k]];[b[k],b[pivot]]=[b[pivot],b[k]];
        assert.ok(Math.abs(A[k][k])>1e-14);
        for(let i=k+1;i<m;i++){const v=A[i][k]/A[k][k];for(let j=k+1;j<m;j++)A[i][j]-=v*A[k][j];b[i]-=v*b[k];}
    }
    const expected=new Float64Array(m);for(let i=m-1;i>=0;i--){let v=b[i];for(let j=i+1;j<m;j++)v-=A[i][j]*expected[j];expected[i]=v/A[i][i];}
    const r=solveCompositeMixedDirection(f.w,chain,{rows,gradient:chain.gradient,fixed,tolerances:tol});assert.ok(r.converged);
    r.increment.forEach((v,i)=>close(v,expected[i]));r.multiplierIncrement.forEach((v,i)=>close(v,expected[n+i]));
});
