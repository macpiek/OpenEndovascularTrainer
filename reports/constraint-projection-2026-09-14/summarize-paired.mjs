import {readFileSync,writeFileSync} from 'node:fs';
const root=new URL('./',import.meta.url),name=process.argv[2]??'paired-final';
const p=JSON.parse(readFileSync(new URL(name+'/profile.json',root))),summary={scope:'Paired complete advanceSharedAxis per identical incoming state, alternating execution order; serialization excluded',sourceTreeHash:p.sourceTreeHash,failed:p.failed,samples:p.samples.length,allExact:p.samples.every(s=>s.paired.exactState),phases:{}};
for(const phase of ['wire','catheter']){
 const rows=p.samples.filter(s=>s.phase===phase),mean=k=>rows.reduce((a,s)=>a+s.paired[k],0)/rows.length,median=v=>v.toSorted((a,b)=>a-b)[Math.floor(v.length/2)];
 let a=mean('referenceMs'),b=mean('optimizedMs'),pa=mean('referenceProjectionMs'),pb=mean('optimizedProjectionMs');
 summary.phases[phase]={samples:rows.length,referenceMs:a,optimizedMs:b,reductionPercent:100*(1-b/a),referenceProjectionMs:pa,optimizedProjectionMs:pb,projectionReductionPercent:100*(1-pb/pa),medianSavedMs:median(rows.map(s=>s.paired.referenceMs-s.paired.optimizedMs)),optimizedFasterSteps:rows.filter(s=>s.paired.optimizedMs<s.paired.referenceMs).length};
}
writeFileSync(new URL(name+'-summary.json',root),JSON.stringify(summary,null,2));console.log(JSON.stringify(summary,null,2));
