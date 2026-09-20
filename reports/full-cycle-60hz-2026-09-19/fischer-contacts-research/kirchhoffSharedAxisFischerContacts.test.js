import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function solve(chain,options,c) {
 return solveSharedAxisLinear(createSharedAxisLinear(chain.layout,options.rows,{lazy:true}),chain,{...options,fischerContacts:c,wasmLinearAssembly:true});
}
function physicalResidual(chain,options,result) {
 const {rows,gradient,fixed}=options,n=chain.layout.dofCount,b=chain.layout.band,x=result.increment,q=result.multiplierIncrement;
 const force=Float64Array.from(gradient);
 for(let i=0;i<n;i++)for(let j=Math.max(0,i-b+1);j<=i;j++) {
  const h=chain.hessian[i*b+i-j];force[i]+=h*x[j];if(i!==j)force[j]+=h*x[i];
 }
 for(let i=0;i<rows.length;i++) {
  const r=rows[i];let gap=r.gap;
  r.dofs.forEach((p,k)=>{gap+=r.jacobian[k]*x[p];force[p]+=(r.kind==='wall'?-1:1)*r.jacobian[k]*q[i];});
  r.extraForceDofs?.forEach((p,k)=>force[p]+=r.extraForceJacobian[k]*q[i]);
  if(r.kind==='length')assert.ok(Math.abs(gap)<1e-7);
  else {
   const load=r.multiplier+q[i];
   assert.ok(gap>=-1e-7,`gap ${gap}`);assert.ok(load>=-1e-7,`load ${load}`);
   assert.ok(Math.abs(Math.max(0,load-gap)-load)<1e-7,`complementarity ${gap}/${load}`);
  }
 }
 for(let i=0;i<n;i++)if(!fixed[i])assert.ok(Math.abs(force[i])<1e-7,`force ${force[i]}`);
}
test('Fischer direction solves unilateral reactions without changing the input model',()=>{
 const n=12,chain={layout:{dofCount:n,band:1},hessian:new Float64Array(n).fill(2)};
 const options={rows:Array.from({length:n},(_,i)=>({kind:'wall',dofs:[i],jacobian:[1],gap:-1-i/10,multiplier:0})),gradient:new Float64Array(n),fixed:new Uint8Array(n)};
 const before=structuredClone({chain,options});
 for(const c of [.0001,.01,1]) {
  const r=solve(chain,options,c);assert.ok(r.converged);assert.ok(r.fischerIterations>0);assert.equal(r.fischerFallback,undefined);
  physicalResidual(chain,options,r);
 }
 assert.deepEqual({chain,options},before);
});
test('Fischer contacts retain physical force balance for coupled rows, loaded release and nonsymmetric friction columns',()=>{
 for(let seed=1;seed<=30;seed++) {
  let random=seed;const rng=()=>{random=(Math.imul(random,1664525)+1013904223)>>>0;return random/2**32;};
  const n=6,layout={dofCount:n,band:n},dofs=Array.from({length:n},(_,i)=>i),target=dofs.map(()=>rng()*2-1);
  const rows=Array.from({length:10},(_,id)=>{
   const jacobian=dofs.map(()=>rng()*2-1);
   return {id,kind:'wall',dofs,jacobian,multiplier:rng()*.1,gap:-jacobian.reduce((v,x,i)=>v+x*target[i],0)+rng()*.3,
    extraForceDofs:[id%n],extraForceJacobian:[.001]};
  });
  const hessian=new Float64Array(n*n);for(let i=0;i<n;i++)hessian[i*n]=2;
  const chain={layout,hessian},options={rows,gradient:Float64Array.from(dofs,()=>rng()),fixed:new Uint8Array(n)};
  const ref=solve(chain,options,0),r=solve(chain,options,.01);
  assert.equal(r.converged,ref.converged,`seed ${seed}`);
  if(r.converged)physicalResidual(chain,options,r);
 }
});
