import test from 'node:test';import assert from 'node:assert/strict';import fs from 'node:fs';import {gunzipSync} from 'node:zlib';
import {solveCoupledBandQP} from '../src/physics/kirchhoffCoupledLinearSolver.js';
import {solveSeededCoulombNewton} from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import {measureCoupledLoadKKT} from '../src/physics/kirchhoffCoupledLoadSolver.js';
function originalResidual(matrix,rhs,count,band,x){const r=Float64Array.from(rhs);for(let i=0;i<count;i++)for(let j=Math.max(0,i-band+1);j<=i;j++){const a=matrix[i*band+i-j];r[i]-=a*x[j];if(i!==j)r[j]-=a*x[i];}return r;}
test('actual 185-row browser fixture retains full original KKT while deleting principal Cholesky rows',()=>{
 const p=JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-wall-witness-browser.json.gz',import.meta.url))),(_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v),before=structuredClone(p);
 const args=['matrix','rhs','lower','upper'].map(k=>Float64Array.from(p[k])),options={...p.options};
 const baseline=solveSeededCoulombNewton(...args,p.count,p.band,p.groups,options);
 const updated=solveSeededCoulombNewton(...args,p.count,p.band,p.groups,{...options,choleskyDeletes:true});
 for(const result of [baseline,updated]){
  assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));
  const residual=originalResidual(args[0],args[1],p.count,p.band,result.increment);
  const groups=p.groups.map(g=>({...g,radii:g.normalRow==null?g.radii:g.mu.map(mu=>mu*Math.max(0,g.normalLambda+result.increment[g.normalRow]))}));
  const kkt=measureCoupledLoadKKT(residual,result.increment,args[2],args[3],groups);
  assert.ok(kkt.maximumResidual<=p.options.tolerance,JSON.stringify(kkt));assert.ok(kkt.coneViolation<=1e-9);
  result.increment.forEach((x,i)=>assert.ok(Number.isFinite(x)&&x>=args[2][i]&&x<=args[3][i]));
 }
 assert.ok(updated.diagnostics.factorizations<baseline.diagnostics.factorizations/2);
 assert.deepEqual(p,before);args.forEach((a,i)=>assert.deepEqual(a,Float64Array.from(before[['matrix','rhs','lower','upper'][i]])));
});
function qp(n,band,negative){
 const A=new Float64Array(n*band),rhs=new Float64Array(n),expected=Float64Array.from({length:n},(_,i)=>i===negative?-.2:1);
 for(let i=0;i<n;i++){A[i*band]=2;if(i>0)A[i*band+1]=.1;rhs[i]=2*expected[i]+(i>0?.1*expected[i-1]:0)+(i<n-1?.1*expected[i+1]:0);}
 return [A,rhs,new Float64Array(n),new Float64Array(n).fill(Infinity),n,band];
}
for(const factorization of ['skyline','band'])test(`delete integration preserves ${factorization} solve through changed sizes, matrices and working sets`,()=>{
 const workspace={};
 for(const [n,index] of [[6,2],[9,0],[3,2],[9,4]]){
  const dense=qp(n,n,index),compact=qp(n,2,index),options={tolerance:1e-10,numericalShift:1e-8,initialFree:new Uint8Array(n).fill(1),factorization};
  const reference=solveCoupledBandQP(...dense,options),expected=reference.increment.slice();
  const updated=solveCoupledBandQP(...dense,{...options,choleskyDeletes:true,workspace});
  assert.ok(updated.diagnostics.converged);assert.ok(updated.diagnostics.factorUpdates>0);
  updated.increment.forEach((x,i)=>assert.ok(Math.abs(x-expected[i])<1e-10));
  const banded=solveCoupledBandQP(...compact,{...options,choleskyDeletes:true,workspace});
  assert.ok(banded.diagnostics.converged);assert.equal(banded.diagnostics.factorUpdates,0,'nonfull source band uses original refactor');
  banded.increment.forEach((x,i)=>assert.ok(Math.abs(x-expected[i])<1e-10));
 }
});
