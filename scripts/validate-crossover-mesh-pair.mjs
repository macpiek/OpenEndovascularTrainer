// Independent audit for a paired PD mesh experiment. A changed trajectory is
// compared diagnostically; both complete paths still need their own gates.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';
const [input,prefix]=process.argv.slice(2);
assert.ok(input&&prefix,'Provide paired report and output prefix');
const bytes=fs.readFileSync(input),pair=JSON.parse(input.endsWith('.gz')?gunzipSync(bytes):bytes);
assert.equal(pair.pairedComparison,'fine-mesh');assert.equal(pair.controlMesh,null);
assert.ok(pair.mesh&&pair.mesh.finePrefixLength===250);assert.deepEqual(pair.options,pair.controlOptions);
assert.equal(pair.options.iterations,16);assert.equal(pair.dt,1/60);
assert.equal(pair.rows.length,pair.controlRows.length);assert.ok(pair.pairedScope?.includes('order alternates'));
const mean=xs=>xs.length?xs.reduce((a,b)=>a+b,0)/xs.length:null;
const summarize=rows=>{
    const stats=rs=>({steps:rs.length,meanCpuMs:mean(rs.map(r=>r.cpuMs)),meanNodes:mean(rs.map(r=>r.shape.coordinates.length)),
        meanDofs:mean(rs.map(r=>r.pd.dofs)),meanClosureIterations:mean(rs.map(r=>r.pd.closureIterations)),
        timings:Object.fromEntries(['constraintMs','linearMs','assemblyMs','rotationMs','projectionMs','closureSolveMs'].map(k=>[k,
            mean(rs.map(r=>r.attempts.reduce((sum,a)=>sum+(a.timings?.[k]??0),0)))]))});
    return {moving:stats(rows.filter(r=>r.accepted&&r.moving)),longMoving:stats(rows.filter(r=>r.accepted&&r.moving&&r.stage===4))};
};
const profiles={},audits={};
for(const [name,rows,mesh] of [['candidate',pair.rows,pair.mesh],['control',pair.controlRows,pair.controlMesh]]) {
    for(const row of rows)if(row.accepted) {
        assert.equal(row.pd.equilibriumCertified,false);assert.equal(row.pd.model,'projective-with-hard-kirchhoff-closure');
        for(const [key,value] of Object.entries(pair.options))assert.equal(row.pd.parameters[key],value,`${name} ${row.step}: ${key}`);
        for(const body of row.quality.bodies)assert.ok(Number.isFinite(body.maxBendAngleDegrees)&&
            (body.maxBendLimitDegrees===null||body.maxBendAngleDegrees<=body.maxBendLimitDegrees+1e-5));
    }
    const {controlRows,controlOptions,controlMesh,controlSummary,...common}=pair;
    const filename=`${prefix}-${name}.json.gz`;
    fs.writeFileSync(filename,gzipSync(JSON.stringify({...common,rows,mesh,
        pairedSource:{path:input,sha256:createHash('sha256').update(bytes).digest('hex'),variant:name}})));
    const result=spawnSync(process.execPath,[path.resolve('scripts/validate-crossover-alternative.mjs'),filename,`${prefix}-${name}-validation.json`],{encoding:'utf8'});
    assert.equal(result.status,0,result.stderr||result.stdout);
    audits[name]=JSON.parse(fs.readFileSync(`${prefix}-${name}-validation.json`));profiles[name]=summarize(rows);
}
let maxShapeDifference=0,firstChangedStep=null;
for(let i=0;i<pair.rows.length;i++)if(pair.rows[i].accepted&&pair.controlRows[i].accepted) {
    const difference=crossoverShapeDifference(pair.rows[i].shape,pair.controlRows[i].shape);
    maxShapeDifference=Math.max(maxShapeDifference,difference);if(difference>0)firstChangedStep??=i+1;
}
const qualified=['candidate','control'].every(k=>audits[k].summary.complete&&audits[k].functionalRoute.routeChecksPassed);
const pairedLongIndices=pair.rows.flatMap((r,i)=>r.accepted&&pair.controlRows[i].accepted&&r.moving&&r.stage===4?[i]:[]);
const matchedLong={steps:pairedLongIndices.length,
    candidateMs:mean(pairedLongIndices.map(i=>pair.rows[i].cpuMs)),
    controlMs:mean(pairedLongIndices.map(i=>pair.controlRows[i].cpuMs))};
const report={input,stepsPerVariant:pair.rows.length,bothFullTrajectoriesAndCoarseRoutesPassed:qualified,
    firstChangedStep,maximumCandidateControlShapeDifferenceMm:maxShapeDifference,shapeComparisonRole:'diagnostic',
    profiles,matchedLong,longStageCostReductionPercent:matchedLong.controlMs?
        100*(1-matchedLong.candidateMs/matchedLong.controlMs):null,
    orderGroups:[0,1].map(parity=>{
        const indices=pair.rows.flatMap((r,i)=>r.accepted&&pair.controlRows[i].accepted&&r.moving&&r.stage===4&&i%2===parity?[i]:[]);
        return {first:parity===0?'control':'candidate',steps:indices.length,
            candidateMs:mean(indices.map(i=>pair.rows[i].cpuMs)),controlMs:mean(indices.map(i=>pair.controlRows[i].cpuMs))};
    }),applicationFpsVerified:false,goalAchieved:false,
    caveat:'Independent full trajectories use alternating order on one browser thread. CPU ratios are not full-application FPS; route checks are coarse and require visual/functional review.'};
fs.writeFileSync(`${prefix}-comparison.json`,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
