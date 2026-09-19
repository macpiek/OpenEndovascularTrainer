import {readFileSync,readdirSync,writeFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy(),folder=new URL('./adaptive-pilot/',import.meta.url),samples=[];
const variants=JSON.parse(process.env.VARIANTS??'{"base":{},"incremental":{"incrementalContacts":true},"limit4":{"newtonActiveSetLimit":4,"reuseNewtonFallback":true},"limit1":{"newtonActiveSetLimit":1,"reuseNewtonFallback":true}}');
try{for(const name of readdirSync(folder).filter(n=>n.startsWith('hot-'))){const f=JSON.parse(readFileSync(new URL(name,folder))),req=f.stepRequest;let ref;
for(const [variant,opts]of Object.entries(variants)){
const s=restoreSharedAxisReplay(f,anatomy.field),it=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,...opts});let n;const start=performance.now();do{n=it.next();}while(!n.done);const ms=performance.now()-start,{state,result}=n.value;if(variant==='base')ref=state;
const diff=state&&ref?Math.max(...state.coordinates.map(x=>{const a=sampleSharedAxisPosition(state,x),b=sampleSharedAxisPosition(ref,x);return Math.hypot(...a.map((v,k)=>v-b[k]));})):null;
const row={name,variant,ms,iterations:result.iterations,factors:result.factorizations,assemblies:result.fullAssemblies+result.residualAssemblies,ok:result.converged,certificate:result.certificateBound,shape:diff};samples.push(row);console.log(JSON.stringify(row));
}}}finally{anatomy.dispose();writeFileSync(new URL('./probe-results.json',import.meta.url),JSON.stringify(samples,null,2));}
