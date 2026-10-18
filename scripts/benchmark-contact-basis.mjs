// Isolated dependency-repair benchmark; this is not full physics throughput.
// Alternating versions, two warmups, exact reaction/trace comparison each run.
import{performance}from'node:perf_hooks';
import{prepareSharedAxisActiveBasis as after}from'../src/physics/kirchhoffSharedAxisActiveBasis.js';
import{pathToFileURL}from'node:url';
import{resolve}from'node:path';
if(!process.argv[2])throw Error('Pass the baseline checkout path');
const{prepareSharedAxisActiveBasis:before}=await import(pathToFileURL(resolve(process.argv[2],'src/physics/kirchhoffSharedAxisActiveBasis.js')));
import assert from'node:assert/strict';
const rows=[];for(let i=0;i<210;i++)rows.push({id:i,kind:'length',dofs:[i,i+1],jacobian:[-1,1],gap:0});
for(let i=0;i<50;i++)for(let j=0;j<3;j++)rows.push({id:`wall/${i}/${j}`,kind:'wall',dofs:[220+i],jacobian:[1+j],gap:0});
const fixed=new Uint8Array(310),basisCache=Symbol('fixed');let costs={before:[],after:[]};
for(let k=0;k<22;k++){
 let reference;
 for(const[name,fn]of k%2?[['after',after],['before',before]]:[['before',before],['after',after]]){
  const activeSet=new Uint8Array(rows.length).fill(1),dual=Float64Array.from(rows,r=>r.kind==='length'?-.1:1+.01*k),trace=[];
  const t=performance.now(),r=fn({rows,fixed,activeSet,dual,trace,lazyBasisCoefficients:true,basisCache});
  if(k>1)costs[name].push(performance.now()-t);
  let out={r,activeSet,dual,trace};if(reference)assert.deepEqual(out,reference);else reference=out;
 }
}
console.log(JSON.stringify(Object.fromEntries(Object.entries(costs).map(([k,a])=>[k,{total:a.reduce((s,v)=>s+v,0),median:a.sort((a,b)=>a-b)[10]}]))));
