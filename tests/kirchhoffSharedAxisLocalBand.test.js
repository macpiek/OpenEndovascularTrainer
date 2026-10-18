import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function fixture() {
    const n=64,band=5,width=2*band-1,layout={dofCount:n,band};
    const tangent=new Float64Array(n*width);
    for(let i=0;i<n;i++)for(let j=Math.max(0,i-band+1);j<=Math.min(n-1,i+band-1);j++)
        tangent[i*width+j-i+band-1]=i===j?15+i:.02*Math.sin(i*3+j);
    const rows=Array.from({length:10},(_,i)=>({id:`row-${i}`,kind:i%3?'wall':'length',dofs:[20+i,21+i],
        jacobian:[.7,.3],gap:0,multiplier:.4,geometricHessian:Float64Array.of(.01,.003,.002,.02),
        extraForceDofs:[18+i],extraForceJacobian:[.001*(i+1)]}));
    const fixed=new Uint8Array(n);fixed[0]=fixed[1]=1;
    const expected=Float64Array.from({length:n},(_,i)=>fixed[i]?0:.001*(1+.1*Math.sin(i)));
    const chain={layout,tangent,hessian:new Float64Array(n*band)},gradient=new Float64Array(n);
    for(let i=0;i<n;i++)for(let j=Math.max(0,i-band+1);j<=Math.min(n-1,i+band-1);j++)gradient[i]-=tangent[i*width+j-i+band-1]*expected[j];
    for(const r of rows) {
        r.gap=-r.dofs.reduce((v,p,k)=>v+r.jacobian[k]*expected[p],0);
        for(let i=0;i<r.dofs.length;i++) {
            gradient[r.dofs[i]]-=(r.kind==='wall'?-1:1)*r.jacobian[i]*.05;
            for(let j=0;j<r.dofs.length;j++)gradient[r.dofs[i]]-=r.geometricHessian[i*2+j]*expected[r.dofs[j]];
        }
        gradient[r.extraForceDofs[0]]-=r.extraForceJacobian[0]*.05;
    }
    return {layout,chain,rows,fixed,gradient,expected};
}
function dense(w,A) {
    const result=new Float64Array(w.count*w.count);
    for(let i=0;i<w.count;i++)for(let j=w.band.starts[i];j<=w.band.ends[i];j++)result[i*w.count+j]=A[w.band.offsets[i]+j];
    return result;
}
const physical=r=>({converged:r.converged,failure:r.failure,increment:r.increment.slice(),reaction:r.multiplierIncrement.slice(),
    residual:r.residual,factorizations:r.factorizations,attempts:r.activeSetAttempts});
for(const wasmLinearAssembly of [false,true])for(const reuseMatrixAssembly of [false,true])
test(`local row envelopes retain original KKT coefficients, pivot solution and nonsymmetric force coupling (${wasmLinearAssembly}/${reuseMatrixAssembly})`,()=>{
    const f=fixture(),out=[],matrices=[];
    for(const compactBand of [false,true]) {
        const w=createSharedAxisLinear(f.layout,f.rows,{compactBand}),original=w.lu.solve;
        if(!wasmLinearAssembly)w.lu.solve=function(...args){matrices.push(dense(w,args[0]));return original.apply(this,args);};
        const r=solveSharedAxisLinear(w,f.chain,{...f,wasmLinearAssembly,reuseMatrixAssembly,compactWorkingSet:false,tolerance:1e-10});
        assert.ok(r.converged,JSON.stringify(r));
        r.increment.forEach((v,i)=>assert.ok(Math.abs(v-f.expected[i])<1e-12));
        out.push(physical(r));
    }
    assert.deepEqual(out[1],out[0]);
    if(!wasmLinearAssembly){assert.equal(matrices.length,2);assert.deepEqual(matrices[1],matrices[0]);}
});

test('a crowded local contact site does not widen unrelated matrix rows',()=>{
    const f=fixture(),full=createSharedAxisLinear(f.layout,f.rows,{compactBand:false}),local=createSharedAxisLinear(f.layout,f.rows);
    assert.deepEqual(local.primal,full.primal);assert.deepEqual(local.dual,full.dual);assert.equal(local.count,full.count);
    assert.ok(local.band.entries<.7*full.band.entries,`${local.band.entries} versus ${full.band.entries}`);
    // No numerical threshold: every location in the physical Hessian stencil
    // remains writable even if its value is zero at the time of construction.
    for(let i=0;i<f.layout.dofCount;i++)for(let j=Math.max(0,i-f.layout.band+1);j<=Math.min(f.layout.dofCount-1,i+f.layout.band-1);j++) {
        assert.ok(local.band.starts[local.primal[i]]<=local.primal[j]);assert.ok(local.band.ends[local.primal[i]]>=local.primal[j]);
    }
});

test('changed public row supports rebuild local envelopes before assembly',()=>{
    const layout={dofCount:16,band:2},chain={layout,hessian:new Float64Array(32)};
    for(let i=0;i<16;i++)chain.hessian[i*2]=5;
    const rows=[{kind:'length',dofs:[2,3],jacobian:[.7,.3],gap:-.01,multiplier:0}],w=createSharedAxisLinear(layout,rows);
    const options={rows,gradient:new Float64Array(16),fixed:new Uint8Array(16),compactWorkingSet:false,reuseMatrixAssembly:true};
    assert.ok(solveSharedAxisLinear(w,chain,options).converged);
    rows[0].dofs=[10,11];rows[0].extraForceDofs=[14];rows[0].extraForceJacobian=[.02];rows[0].geometricHessian=Float64Array.of(.1,0,0,.2);
    const result=solveSharedAxisLinear(w,chain,options),fresh=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,options);
    assert.ok(w.fullReference);assert.ok(result.converged);assert.deepEqual(physical(result),physical(fresh));
});
