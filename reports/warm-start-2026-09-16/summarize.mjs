import {readFileSync,writeFileSync} from 'node:fs';
const root=new URL('./',import.meta.url),reports={};
const stats=a=>{const sorted=a.toSorted((x,y)=>x-y);return {mean:a.reduce((a,b)=>a+b)/a.length,p95:sorted[Math.floor(.95*(a.length-1))],p99:sorted[Math.floor(.99*(a.length-1))],max:sorted.at(-1)}};
for(const variant of ['duplicate-guard','cost-gated']) {
 const r=JSON.parse(readFileSync(new URL(variant+'/profile.json',root))),samples=r.samples;
 reports[variant]={pairs:samples.length,failed:r.failed,exactStates:samples.filter(s=>s.paired.exactState).length,
  drift:Object.fromEntries(Object.keys(samples[0].paired.predictionComparison).map(k=>[k,Math.max(...samples.map(s=>s.paired.predictionComparison[k]))])),phases:{}};
 for(const phase of new Set(samples.map(s=>s.phase))) {
  const ss=samples.filter(s=>s.phase===phase),sum=f=>ss.reduce((a,s)=>a+f(s),0),reference=stats(ss.map(s=>s.paired.referenceMs)),optimized=stats(ss.map(s=>s.paired.optimizedMs));
  reports[variant].phases[phase]={n:ss.length,referenceMs:reference,optimizedMs:optimized,changePercent:100*(optimized.mean/reference.mean-1),
   referenceFactors:sum(s=>s.factorizations),optimizedFactors:sum(s=>s.paired.optimizedResult.factorizations),
   referenceIterations:sum(s=>s.iterations),optimizedIterations:sum(s=>s.paired.optimizedResult.iterations)};
 }
}
writeFileSync(new URL('summary.json',root),JSON.stringify(reports,null,2));
console.log(Object.fromEntries(Object.entries(reports).map(([v,r])=>[v,{pairs:r.pairs,exact:r.exactStates,catheter:r.phases.catheter}])));
