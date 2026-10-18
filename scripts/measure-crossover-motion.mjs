// Diagnostic for temporal jumps; finite accepted states alone do not prove
// smooth motion. Tip displacement is measured at each tool's own moving end,
// interpolated on the shared axis, rather than comparing array node indices.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import assert from 'node:assert/strict';
const [output,...paths]=process.argv.slice(2);assert.ok(output&&paths.length);
function tip(step,id){
    const {coordinates:c,positions:p}=step.shape,x=step.tools[id];
    assert.ok(x>=c[0]&&x<=c.at(-1)+1e-8);
    let i=0;while(i<c.length-2&&c[i+1]<x)i++;
    const t=(x-c[i])/(c[i+1]-c[i]);
    return [0,1,2].map(k=>p[3*i+k]+t*(p[3*(i+1)+k]-p[3*i+k]));
}
const reports=[];
for(const path of paths){
    const bytes=fs.readFileSync(path),r=JSON.parse(path.endsWith('.gz')?gunzipSync(bytes):bytes),rows=[];
    const steps=r.steps??r.rows?.filter(row=>row.accepted);
    assert.ok(Array.isArray(steps)&&steps.every(step=>step.shape),'Accepted poses are required');
    let previous=null;
    for(const [i,s] of steps.entries()){
        const current={wire:tip(s,'wire'),catheter:tip(s,'catheter')};
        if(previous)rows.push({step:i+1,stage:s.stage,moving:s.moving,
            stationary:!s.moving&&!steps[i-1].moving,
            ...Object.fromEntries(['wire','catheter'].map(id=>[id,Math.hypot(...current[id].map((v,k)=>v-previous[id][k]))]))});
        previous=current;
    }
    const metrics=selected=>Object.fromEntries(['wire','catheter'].map(id=>{
        const sorted=selected.map(r=>r[id]).sort((a,b)=>a-b),max=selected.reduce((a,b)=>!a||b[id]>a[id]?b:a,null);
        return [id,{p95MmPerStep:sorted[Math.floor((sorted.length-1)*.95)]??null,
            maxMmPerStep:max?.[id]??null,maxAtStep:max?.step??null}];
    }));
    reports.push({path,complete:r.complete??r.summary?.complete??false,dt:r.dt,steps:steps.length,
        moving:metrics(rows.filter(r=>r.moving)),stationary:metrics(rows.filter(r=>r.stationary)),
        stages:Array.from({length:5},(_,stage)=>({stage:stage+1,...metrics(rows.filter(r=>r.stage===stage))})),
        largestJumps:rows.sort((a,b)=>Math.max(b.wire,b.catheter)-Math.max(a.wire,a.catheter)).slice(0,12)});
}
fs.writeFileSync(output,JSON.stringify({reports,scope:'Temporal tip-motion diagnostics, not a standalone stability or performance certificate.'},null,2)+'\n');
console.log(JSON.stringify(reports.map(({path,moving,stationary})=>({path,moving,stationary}))));
