import assert from 'node:assert/strict';
import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';

const roots=process.argv.slice(2,4);assert.equal(roots.length,2,'Pass baseline and candidate roots');
const load=(root,path)=>import(pathToFileURL(resolve(root,path)));
const sides=await Promise.all(roots.map(async root=>({root,
    step:await load(root,'src/physics/kirchhoffCompositeJointTimeStep.js'),
    fixture:await load(root,'tests/fixtures/compositeRelativeSearch.js')})));
const summarize=xs=>{const s=xs.slice().sort((a,b)=>a-b);return {mean:xs.reduce((a,b)=>a+b,0)/xs.length,median:s[Math.floor(xs.length/2)],p95:s[Math.ceil(xs.length*.95)-1]};};
const cases=[{name:'open-lumen-65',geometry:{n:65,lumen:true},input:{force:[.01,0,0]}},
    {name:'loaded-lumen-17',geometry:{n:17,lumen:true,offset:.04075,wireSlope:-.0005},input:{force:[.1,.4,0],forceNode:0}},
    {name:'loaded-lumen-65',geometry:{n:65,lumen:true,offset:.0406,wireSlope:-.0002},input:{force:[.1,.4,0],forceNode:0}}];
const files=['kirchhoffCompositeJointTimeStep.js','kirchhoffCompositeJointLumenRows.js','kirchhoffCompositeLumenSideGeometry.js',
    'kirchhoffCompositeContactPullback.js','kirchhoffCompositeJointLumenSurface.js','kirchhoffCompositeJointLumenFrictionRows.js'];
const report={scope:'Frozen original vs needed contact derivatives; identical whole-step mechanics, input and acceptance; Node synthetic shafts, not anatomy or browser FPS',
    warmupPairs:10,measuredPairs:24,sources:roots.map(root=>({root,sha256:Object.fromEntries(files.map(name=>[name,createHash('sha256').update(readFileSync(resolve(root,'src/physics',name))).digest('hex')]))})),cases:[]};
assert.equal(report.sources[0].sha256['kirchhoffCompositeJointTimeStep.js'],report.sources[1].sha256['kirchhoffCompositeJointTimeStep.js']);
for(const spec of cases) {
    const contexts=sides.map(s=>{const f=s.fixture.relativeSearchFixture(spec.geometry);return {s,f,workspace:s.step.createCompositeJointTimeStepWorkspace(f.state)};});
    const records=[[],[]];let maxShapeError=0,maxReactionError=0;
    for(let repeat=-report.warmupPairs;repeat<report.measuredPairs;repeat++) {
        const results=[];
        for(const side of repeat%2===0?[0,1]:[1,0]) {
            const c=contexts[side],r=c.s.step.advanceCompositeJointTimeStep(c.f.state,c.s.fixture.relativeSearchOptions(c.f,c.f.state,{...spec.input,workspace:c.workspace}));
            assert.equal(r.accepted,true,JSON.stringify({scenario:spec.name,side,status:r.status,error:r.error,diagnostics:r.diagnostics}));results[side]=r;
            const d=r.diagnostics;if(repeat>=0)records[side].push({totalMs:d.totalMs,directionMs:d.directionMs,preparationMs:d.preparationMs,iterationMs:d.iterationMs,
                directions:d.directions,evaluations:d.evaluations,contactQueries:d.contactQueries,linearSolves:d.linearSolves});
        }
        for(const [id,p] of results[0].state.toolPositions)p.forEach((v,i)=>v.forEach((x,k)=>maxShapeError=Math.max(maxShapeError,Math.abs(x-results[1].state.toolPositions.get(id)[i][k]))));
        for(const [key,field] of [['lumenContactState','normalForces'],['lumenFrictionState','tractions']])
            results[0].state[key][field].forEach((x,i)=>maxReactionError=Math.max(maxReactionError,Math.abs(x-results[1].state[key][field][i])));
        for(const key of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(results[1][key],results[0][key],key);
        for(const key of ['certificate','directions','evaluations','contactQueries','linearSolves'])assert.deepEqual(results[1].diagnostics[key],results[0].diagnostics[key],key);
    }
    const entry={...spec,maxShapeError,maxReactionError,results:records.map((r,side)=>({root:roots[side],
        statistics:Object.fromEntries(Object.keys(r[0]).map(key=>[key,summarize(r.map(x=>x[key]))])),records:r}))};report.cases.push(entry);
    console.log(JSON.stringify({...entry,results:entry.results.map(({records,...r})=>r)}));
    if(process.argv[4])writeFileSync(process.argv[4],JSON.stringify(report,null,2)+'\n');
}
if(process.argv[4])writeFileSync(process.argv[4],JSON.stringify(report,null,2)+'\n');
