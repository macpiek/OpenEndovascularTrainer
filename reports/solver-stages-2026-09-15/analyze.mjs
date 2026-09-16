import fs from 'node:fs';
const dir=new URL('./',import.meta.url);
const mean=x=>x.reduce((a,b)=>a+b,0)/(x.length||1),sum=x=>x.reduce((a,b)=>a+b,0);
const stats=x=>{const a=x.filter(Number.isFinite).sort((a,b)=>a-b);return {mean:mean(a),p50:a[Math.floor((a.length-1)*.5)],p95:a[Math.floor((a.length-1)*.95)],p99:a[Math.floor((a.length-1)*.99)],max:a.at(-1)}};
function group(steps){
 const names=[...new Set(steps.flatMap(s=>Object.keys(s.stageProfile?.stages??{})))];
 return {n:steps.length,cpu:stats(steps.map(s=>s.cpuMs)),provider:stats(steps.map(s=>s.providerMs)),iterations:stats(steps.map(s=>s.iterations)),factorizations:stats(steps.map(s=>s.factorizations)),attempts:sum(steps.map(s=>s.substepAttempts)),geometryRestarts:sum(steps.map(s=>s.geometryRestarts)),timings:Object.fromEntries(Object.keys(steps[0]?.timings??{}).map(k=>[k,stats(steps.map(s=>s.timings[k]))])),stages:Object.fromEntries(names.map(k=>[k,{self:stats(steps.map(s=>s.stageProfile?.stages[k]?.selfMs??0)),inclusive:stats(steps.map(s=>s.stageProfile?.stages[k]?.inclusiveMs??0)),calls:stats(steps.map(s=>s.stageProfile?.stages[k]?.calls??0))}]))};
}
for(const name of ['baseline','instrumented']){
 const path=new URL(name+'-browser.json',dir);if(!fs.existsSync(path))continue;
 const r=JSON.parse(fs.readFileSync(path)),steps=r.wire60Profile.steps,frames=r.wire60Profile.frames;
 const wire=steps.filter(s=>s.catheter<1e-7),cat=steps.filter(s=>s.catheter>=1e-7),wireEnd=wire.at(-1).wallMs;
 const threshold=stats(cat.map(s=>s.cpuMs)).p95,slow=cat.filter(s=>s.cpuMs>=threshold);
 const out={name,durationMs:r.durationMs,frame:{fps:r.averageFps,low1:r.onePercentLowFps,p99:r.p99FrameMs,max:r.maxFrameMs,long33:r.longFrame33Count,long50:r.longFrame50Count,cpu:r.frameCpu},page:r.pageState,scenario:r.scenario,all:group(steps),wire:{...group(wire),wallMs:wireEnd,hz:1000*wire.length/wireEnd},catheter:{...group(cat),wallMs:cat.at(-1).wallMs-wireEnd,hz:1000*cat.length/(cat.at(-1).wallMs-wireEnd)},slowCatheter:group(slow),topSteps:[...steps].sort((a,b)=>b.cpuMs-a.cpuMs).slice(0,12),bins:Array.from({length:6},(_,i)=>{const ss=cat.filter(s=>s.catheter>i*100&&s.catheter<=(i+1)*100+1e-7);return {rangeCm:[i*10,(i+1)*10],...group(ss)}}),topFrames:[...frames].sort((a,b)=>b.frameMs-a.frameMs).slice(0,15)};
 if(name==='instrumented'){
  out.accountingErrorMaxMs=Math.max(...steps.map(s=>Math.abs(sum(Object.values(s.stageProfile.stages).map(r=>r.selfMs))-s.stageProfile.stages.unattributed.inclusiveMs)));
  out.topSlices=steps.flatMap(s=>s.stageProfile.slowestSlices.map(sl=>({...sl,relativeMs:sl.startMs-r.profileTimeOrigin,wire:s.wire,catheter:s.catheter,stepCpu:s.cpuMs}))).sort((a,b)=>b.ms-a.ms).slice(0,15);
 }
 fs.writeFileSync(new URL(name+'-summary.json',dir),JSON.stringify(out,null,2));
 console.log(JSON.stringify({name,durationMs:out.durationMs,frame:out.frame,wire:{n:out.wire.n,cpu:out.wire.cpu,hz:out.wire.hz},catheter:{n:out.catheter.n,cpu:out.catheter.cpu,hz:out.catheter.hz},slow:{n:slow.length,cpu:out.slowCatheter.cpu},stages:Object.entries(out.catheter.stages).sort((a,b)=>b[1].self.mean-a[1].self.mean).map(([k,v])=>[k,v.self.mean,v.inclusive.mean,v.calls.mean]),top:out.topSteps.slice(0,3).map(s=>({wire:s.wire,cat:s.catheter,cpu:s.cpuMs,iterations:s.iterations,LU:s.factorizations,attempts:s.substepAttempts,t:s.timings})),accounting:out.accountingErrorMaxMs},null,2));
}
