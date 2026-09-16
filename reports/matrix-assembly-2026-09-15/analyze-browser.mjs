import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const root=new URL('./',import.meta.url),read=name=>JSON.parse(readFileSync(new URL(name,root)));
const reports=['reference','optimized'].map(name=>read(name+'-browser.json'));
const mean=a=>a.reduce((x,y)=>x+y,0)/a.length;
const stats=a=>{const s=a.toSorted((x,y)=>x-y);return {mean:mean(a),p95:s[Math.floor(.95*(s.length-1))],p99:s[Math.floor(.99*(s.length-1))],max:s.at(-1)}};
const fields=['accepted','status','wire','catheter','iterations','factorizations','substepAttempts','geometryRestarts','wallNormalFallbacks'];
assert.equal(reports[0].wire60Profile.steps.length,reports[1].wire60Profile.steps.length);
reports[0].wire60Profile.steps.forEach((s,i)=>{for(const key of fields)assert.deepEqual(s[key],reports[1].wire60Profile.steps[i][key],`${i}/${key}`)});
const summary=reports.map((r,i)=>{
 const s=r.wire60Profile.steps,w=s.filter(x=>x.catheter<1e-7),c=s.filter(x=>x.catheter>=1e-7);
 return {variant:i?'optimized':'reference',durationMs:r.durationMs,wire:{n:w.length,cpu:stats(w.map(s=>s.cpuMs)),hz:1000*w.length/w.at(-1).wallMs},
  catheter:{n:c.length,cpu:stats(c.map(s=>s.cpuMs)),hz:1000*c.length/(c.at(-1).wallMs-w.at(-1).wallMs),timings:Object.fromEntries(Object.keys(c[0].timings).map(k=>[k,stats(c.map(s=>s.timings[k]))]))},
  render:{fps:r.averageFps,low1:r.onePercentLowFps,p99FrameMs:r.p99FrameMs,maxFrameMs:r.maxFrameMs,long33:r.longFrame33Count,long50:r.longFrame50Count},
  quality:r.physicsEnvelope,scenario:r.scenario,page:r.pageState,heap:r.heap,
  counters:Object.fromEntries(['iterations','factorizations','backtracks','geometryRestarts','substepAttempts'].map(k=>[k,r.constraintStageProfile.fields[k].sum]))};
});
assert.deepEqual(summary[0].quality,summary[1].quality);
assert.deepEqual(summary[0].counters,summary[1].counters);
writeFileSync(new URL('browser-comparison.json',root),JSON.stringify({sameStepInputsAndCounters:true,sameQualityEnvelope:true,conditions:'Both reloaded normal 5173 tab, contrast0, no ROADMAP, same defaults and benchmark; sequential reference then optimized, no overlapping CPU tests/benchmarks. One run per variant; not a confidence interval.',runs:summary},null,2));
console.log(summary.map(s=>({variant:s.variant,wall:s.durationMs,catheter:s.catheter.cpu,hz:s.catheter.hz,linear:s.catheter.timings.linearMs.mean,render:s.render})));
