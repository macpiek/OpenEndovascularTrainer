import test from 'node:test';import assert from 'node:assert/strict';import fs from 'node:fs';import {gunzipSync} from 'node:zlib';
import {solveSeededCoulombNewton as solve} from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import {measureCoupledLoadKKT} from '../src/physics/kirchhoffCoupledLoadSolver.js';
function problem(){return {matrix:Float64Array.from([2,0,0,2,.4,0,2,0,0]),rhs:Float64Array.from([1,-4,0]),lower:Float64Array.from([0,-Infinity,-Infinity]),upper:new Float64Array(3).fill(Infinity),count:3,band:3,groups:[{rows:[1,2],lambda:[0,0],normalLambda:0,normalRow:0,mu:[.2,.2]}]};}
const inputs=p=>[p.matrix,p.rhs,p.lower,p.upper,p.count,p.band,p.groups];
function audit(p,result,tolerance){
 const residual=Float64Array.from(p.rhs),x=result.increment;
 for(let i=0;i<p.count;i++)for(let j=Math.max(0,i-p.band+1);j<=i;j++){
  const a=p.matrix[i*p.band+i-j];residual[i]-=a*x[j];if(i!==j)residual[j]-=a*x[i];
 }
 const groups=p.groups.map(g=>({...g,radii:g.normalRow==null?g.radii:g.mu.map(mu=>mu*Math.max(0,g.normalLambda+x[g.normalRow]))}));
 const kkt=measureCoupledLoadKKT(residual,x,p.lower,p.upper,groups);
 assert.ok(kkt.maximumResidual<=tolerance,JSON.stringify(kkt));assert.ok(kkt.coneViolation<=1e-9);
 x.forEach((value,i)=>assert.ok(Number.isFinite(value)&&value>=p.lower[i]&&value<=p.upper[i]));
 residual.forEach((value,i)=>assert.ok(Math.abs(value-result.residual[i])<1e-10*Math.max(1,Math.abs(value))));
}
test('a successful zero-increment Newton probe skips the fixed-load seed',()=>{
 const p=problem(),before=structuredClone(p);let seeds=0;
 const result=solve(...inputs(p),{tolerance:1e-10,tryNewtonBeforeSeed:true,debugLoadIteration(){seeds++;}});
 assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));assert.equal(seeds,0);audit(p,result,1e-10);assert.deepEqual(p,before);
});
test('a rejected probe discards its iterate and falls back to exact original seeded outputs',()=>{
 const p=problem(),before=structuredClone(p),options={tolerance:1e-10},standard=solve(...inputs(p),options);
 let seeds=0;const probes=[];
 const result=solve(...inputs(p),{...options,tryNewtonBeforeSeed:true,maxUnseededNewtonIterations:1,
  acceptCandidate(){return seeds>0;},debugLoadIteration(){seeds++;},debugCoulombResult({result}){probes.push(result.diagnostics.converged);}});
 assert.ok(seeds>0);assert.equal(probes[0],false);assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));
 for(const key of ['increment','residual','free','lower','upper','allGroups'])assert.deepEqual(result[key],standard[key],key);
 audit(p,result,1e-10);assert.deepEqual(p,before);
});
test('probe-first saved operator still satisfies independently reconstructed original KKT',()=>{
 const p=JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-condensed-load-cycle.json.gz',import.meta.url))),(_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);
 for(const k of ['matrix','rhs','lower','upper'])p[k]=Float64Array.from(p[k]);const before=structuredClone(p);
 const options={...p.options},standard=solve(...inputs(p),options),candidate=solve(...inputs(p),{...options,tryNewtonBeforeSeed:true});
 assert.ok(standard.diagnostics.converged);assert.ok(candidate.diagnostics.converged,JSON.stringify(candidate.diagnostics));
 audit(p,standard,options.tolerance);audit(p,candidate,options.tolerance);assert.deepEqual(p,before);
});
