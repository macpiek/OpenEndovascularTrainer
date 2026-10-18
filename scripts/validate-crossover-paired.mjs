// Audit two independently evolved full trajectories measured in alternating
// order in one browser. CPU ratios do not establish full-application FPS.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
const [input,prefix]=process.argv.slice(2);
assert.ok(input&&prefix,'Provide paired report and output prefix');
const bytes=fs.readFileSync(input),pair=JSON.parse(input.endsWith('.gz')?gunzipSync(bytes):bytes);
assert.equal(pair.rows.length,7507);assert.equal(pair.controlRows.length,7507);
assert.equal(pair.dt,1/60);
assert.ok(pair.pairedScope?.includes('order alternates'));
const sourceDigest=createHash('sha256').update(bytes).digest('hex');
const variants=[['candidate',pair.rows,pair.options,pair.summary],['control',pair.controlRows,pair.controlOptions,pair.controlSummary]];
const mean=xs=>xs.reduce((a,b)=>a+b,0)/xs.length;
const summarize=rows=>{
    const moving=rows.filter(r=>r.moving),long=moving.filter(r=>r.stage===4);
    const stats=rs=>({steps:rs.length,meanCpuMs:mean(rs.map(r=>r.cpuMs)),cpuCapacityHz:1000/mean(rs.map(r=>r.cpuMs)),
        timings:Object.fromEntries(['constraintMs','linearMs','assemblyMs','rotationMs','projectionMs','closureSolveMs'].map(k=>[k,
            mean(rs.map(r=>r.attempts.reduce((sum,a)=>sum+(a.timings?.[k]??0),0)))]))});
    return {moving:stats(moving),longMoving:stats(long),stages:[0,1,2,3,4].map(stage=>stats(moving.filter(r=>r.stage===stage)))};
};
const profiles={};
for(const [name,rows,options,summary] of variants) {
    assert.ok(rows.every(r=>r.accepted));
    for(const row of rows)for(const [key,value] of Object.entries(options))assert.equal(row.pd.parameters[key],value,`${name} step ${row.step}: ${key}`);
    const {controlRows,controlOptions,controlSummary,...common}=pair;
    const filename=`${prefix}-${name}.json.gz`;
    fs.writeFileSync(filename,gzipSync(JSON.stringify({...common,rows,options,summary,
        pairedSource:{path:input,sha256:sourceDigest,variant:name}})));
    const result=spawnSync(process.execPath,[path.resolve('scripts/validate-crossover-alternative.mjs'),filename,`${prefix}-${name}-validation.json`],{encoding:'utf8'});
    assert.equal(result.status,0,result.stderr||result.stdout);
    const audit=JSON.parse(fs.readFileSync(`${prefix}-${name}-validation.json`));
    assert.equal(audit.summary.complete,true);assert.equal(audit.functionalRoute.routeChecksPassed,true);
    profiles[name]=summarize(rows);
}
for(let i=0;i<pair.rows.length;i++) {
    assert.deepEqual(pair.rows[i].shape,pair.controlRows[i].shape,`Shape changed at step ${i+1}`);
    assert.deepEqual(pair.rows[i].quality,pair.controlRows[i].quality,`Quality changed at step ${i+1}`);
}
const report={input,sha256:sourceDigest,steps:7507,allPosesAndQualityIdentical:true,profiles,
    longStageCostReductionPercent:100*(1-profiles.candidate.longMoving.meanCpuMs/profiles.control.longMoving.meanCpuMs),
    orderGroups:[0,1].map(parity=>{
        const indices=pair.rows.flatMap((r,i)=>r.moving&&r.stage===4&&i%2===parity?[i]:[]);
        return {first:parity===0?'control':'candidate',steps:indices.length,
            candidateMs:mean(indices.map(i=>pair.rows[i].cpuMs)),controlMs:mean(indices.map(i=>pair.controlRows[i].cpuMs))};
    }),
    scope:pair.pairedScope,applicationFpsVerified:false,goalAchieved:false,
    caveat:'Both independent trajectories run on one thread. Alternating order reduces time/order bias but shared browser caches and scheduling remain. Full app and two-access/common-clock validation are still required.'};
fs.writeFileSync(`${prefix}-comparison.json`,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
