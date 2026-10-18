import assert from 'node:assert/strict';
import fs from 'node:fs';
const root=new URL('../',import.meta.url).href;
const {createSharedAxisNative,applySharedAxisNativeIncrement}=await import(root+'src/physics/kirchhoffSharedAxisNative.js');
const {prepareSharedAxisDynamicStep,assembleSharedAxisInertia}=await import(root+'src/physics/kirchhoffSharedAxisDynamics.js');
const {sharedAxisMaterialKernelWorkspace}=await import(root+'src/physics/kirchhoffSharedAxisMaterialKernel.js');
const s=createSharedAxisNative({tools:[{id:'wire',insertion:981.2,mass:1},{id:'catheter',insertion:860.6667,mass:1.75,type:'pigtail'}],samplePosition:x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),.01*x]});
s.velocities=s.positions.map((_,i)=>[Math.sin(i),Math.cos(i),.1*i]);
s.angularVelocities=Object.fromEntries(s.materials.map(m=>[m.spec.id,Array.from({length:m.last},(_,i)=>[8*Math.sin(i),-3*Math.cos(i),4*Math.sin(i*.7)])]));
prepareSharedAxisDynamicStep(s,1/60);
applySharedAxisNativeIncrement(s,Float64Array.from({length:s.layout.dofCount},(_,i)=>.03*Math.sin(i*1.23)),new Float64Array(s.multipliers.length));
const w=sharedAxisMaterialKernelWorkspace(s.chain,s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0));
s.chain.tangent=w.tangent;
function run(wasm){s.chain.gradient.fill(0);w.tangent.fill(.2);return assembleSharedAxisInertia(s,true,wasm);}
const energy=run(false),H=w.tangent.slice(),g=s.chain.gradient.slice();
assert.equal(run(true),energy);assert.deepEqual(w.tangent,H);assert.deepEqual(s.chain.gradient,g);
for(let i=0;i<500;i++){run(false);run(true);}
const runs=[],iterations=1000;
for(let repeat=0;repeat<10;repeat++)for(const wasm of repeat%2?[true,false]:[false,true]){
 const start=performance.now();for(let i=0;i<iterations;i++)run(wasm);
 runs.push({repeat,wasm,ms:performance.now()-start});
}
const totals=Object.fromEntries([false,true].map(w=>[w,runs.filter(r=>r.wasm===w).reduce((a,r)=>a+r.ms,0)]));
const report={passed:true,scope:'Rotational inertia assembly on a synthetic curved two-tool shaft. Includes gradient/matrix clearing; excludes solver, contacts and new-chain allocation.',iterationsPerMethod:10000,node:process.version,nodes:s.positions.length,totals,reductionPercent:100*(1-totals.true/totals.false),runs};
fs.writeFileSync(new URL('reports/performance-scenarios/001-inertia-kernel-microbenchmark.json',root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report));
