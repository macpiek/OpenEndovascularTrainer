import test from 'node:test';import assert from 'node:assert/strict';import fs from 'node:fs';import {gunzipSync} from 'node:zlib';
import {solveActiveCondensedCoupledQP as solve} from '../src/physics/kirchhoffActiveCondensedSolver.js';
import {measureCoupledLoadKKT} from '../src/physics/kirchhoffCoupledLoadSolver.js';
function read(name){return JSON.parse(gunzipSync(fs.readFileSync(new URL(`./fixtures/${name}.json.gz`,import.meta.url))),(_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);}
function audit(p,result,tolerance){
 const residual=Float64Array.from(p.rhs),x=result.increment;
 for(let i=0;i<p.count;i++)for(let j=Math.max(0,i-p.band+1);j<=i;j++){
  const value=p.matrix[i*p.band+i-j];residual[i]-=value*x[j];if(i!==j)residual[j]-=value*x[i];
 }
 const groups=p.groups.map(group=>({...group,radii:group.normalRow==null?group.radii:group.mu.map(mu=>mu*Math.max(0,group.normalLambda+x[group.normalRow]))}));
 const kkt=measureCoupledLoadKKT(residual,x,p.lower,p.upper,groups);
 assert.ok(kkt.maximumResidual<=tolerance,JSON.stringify(kkt));assert.ok(kkt.coneViolation<=1e-9);
 x.forEach((v,i)=>assert.ok(Number.isFinite(v)&&v>=p.lower[i]&&v<=p.upper[i]));
 residual.forEach((v,i)=>assert.ok(Math.abs(v-result.residual[i])<1e-9*Math.max(1,Math.abs(v))));return kkt;
}
for(const name of ['kirchhoff-friction-boundary-cycle','kirchhoff-normal-load-cycle'])test(`short seed retains original full KKT on loaded ${name}`,()=>{
 const p=read(name),before=structuredClone(p),args=['matrix','rhs','lower','upper'].map(key=>Float64Array.from(p[key]));
 assert.ok(p.groups.some(group=>group.normalLambda>0),'exercise positive-load fixed friction cones');
 const options={...p.options,simultaneousCoulomb:true},seedIterations=[];
 const standard=solve(...args,p.count,p.band,p.groups,options);
 const short=solve(...args,p.count,p.band,p.groups,{...options,maxSeedFrictionIterations:4,
  debugLoadIteration({result,groups}){if(groups.length)seedIterations.push(result.diagnostics.iterations);}});
 assert.ok(standard.diagnostics.converged,JSON.stringify(standard.diagnostics));assert.ok(short.diagnostics.converged,JSON.stringify(short.diagnostics));
 audit(p,standard,options.tolerance??1e-8);audit(p,short,options.tolerance??1e-8);
 assert.ok(seedIterations.length>0);assert.ok(seedIterations.every(n=>n<=4));
 const boundedIterations=[];
 const bounded=solve(...args,p.count,p.band,p.groups,{...options,maxSeedFrictionIterations:4,maxSeedActiveSetIterations:16,
  debugLoadIteration({result,groups}){boundedIterations.push({outer:result.diagnostics.iterations,inner:result.diagnostics.innerIterations??result.diagnostics.iterations,grouped:groups.length>0});}});
 assert.ok(bounded.diagnostics.converged,JSON.stringify(bounded.diagnostics));audit(p,bounded,options.tolerance??1e-8);
 assert.ok(boundedIterations.length>0);
 for(const iteration of boundedIterations){
  assert.ok(iteration.outer<=(iteration.grouped?4:16));
  assert.ok(iteration.inner<=(iteration.grouped?4*16:16),'seed inner active-set iterations stay bounded');
 }
 assert.deepEqual(p,before);args.forEach((array,i)=>assert.deepEqual(array,Float64Array.from(before[['matrix','rhs','lower','upper'][i]])));
});
