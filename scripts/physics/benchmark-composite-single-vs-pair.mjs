import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL,fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {prepareSingleVsPair,createSingleVsPairOperator} from './helpers/composite-single-vs-pair.js';

const snapshot=process.argv[3]??'/tmp/oet-composite-exact-snapshot-iBYhkf',output=process.argv[2]??'/tmp/oet-composite-single-vs-pair.json';
const sha=value=>createHash('sha256').update(value).digest('hex'),manifest=JSON.parse(fs.readFileSync(path.join(snapshot,'manifest.json')));
const intact=()=>Object.entries(manifest.files).every(([p,m])=>sha(fs.readFileSync(path.join(snapshot,p)))===m.sha256);
assert.ok(intact(),'The selected frozen dependency graph must match its manifest');
const load=relative=>import(pathToFileURL(path.join(snapshot,relative)).href);
const modules={fixture:await load('scripts/physics/helpers/compositeTimeStepBenchmark.js'),chain:await load('src/physics/kirchhoffCompositeChain.js'),
    time:await load('src/physics/kirchhoffCompositeTimeStep.js'),inertia:await load('src/physics/kirchhoffCompositeInertiaCache.js'),
    length:await load('src/physics/kirchhoffCompositeLengthConstraints.js'),mixed:await load('src/physics/kirchhoffCompositeMixedDirection.js')};
const serial=value=>typeof value==='function'?'[frozen-function]':ArrayBuffer.isView(value)?Array.from(value,serial):Array.isArray(value)?value.map(serial):value instanceof Map?[...value].map(serial):value&&typeof value==='object'?Object.fromEntries(Object.entries(value).map(([k,v])=>[k,serial(v)])):value;
const hash=value=>sha(JSON.stringify(serial(value)));
const median=values=>{const a=values.toSorted((a,b)=>a-b),i=a.length>>1;return a.length%2?a[i]:(a[i-1]+a[i])/2;};
const stats=values=>({median:median(values),mean:values.reduce((a,b)=>a+b)/values.length,max:Math.max(...values),samples:values});
const ownPaths=[fileURLToPath(import.meta.url),fileURLToPath(new URL('./helpers/composite-single-vs-pair.js',import.meta.url))];
const ownHashes=()=>Object.fromEntries(ownPaths.map(p=>[p,sha(fs.readFileSync(p))]));
const report={snapshot,manifestHash:sha(fs.readFileSync(path.join(snapshot,'manifest.json'))),sourceBefore:ownHashes(),
    scope:'Single wire vs wire+catheter on IDENTICAL spatial grids, wire profiles/maps/history, translational load/BC, wire spin BC, dt and strict tolerances. Only catheter material/spins/mass arm is removed. Fixed initial straight state repeated unchanged to separate cold/warm machine behavior from different physical trajectories. No anatomy/lumen/feed/remesh/FPS claim.',
    conventions:modules.fixture.conventions,cases:[]};
for(const insertion of [9,160,310]) {
    const prepared=prepareSingleVsPair(modules,insertion),commonInputHash=hash(prepared.commonInputs),arms=[];
    for(const population of ['single','pair'])for(const elementBackend of ['wasm','wasm-exact']) {
        const fixture=prepared.cases[population],operator=createSingleVsPairOperator(modules,fixture,elementBackend);
        arms.push({population,elementBackend,fixture,operator,operatorRows:[],coldOperatorRows:[],steps:[]});
    }
    // First FULL call in each arm is recorded before operator warmups. Later
    // calls reuse the identical original physical state, not another arm's
    // different accepted trajectory. The TimeStep creates its own workspaces.
    for(let repeat=0;repeat<3;repeat++)for(const arm of repeat%2?[...arms].reverse():arms) {
        const state=arm.fixture.state,before=hash(state),base=arm.fixture.options,preparedHash=hash(base),start=performance.now();
        const result=modules.time.advanceCompositeTimeStep(state,{...base,elementBackend:arm.elementBackend});const fullDtMs=performance.now()-start;
        assert.equal(hash(state),before);assert.equal(hash(base),preparedHash);
        assert.ok(result.accepted?result.state.step===state.step+1&&result.state.time===state.time+base.dt&&result.diagnostics.historyCommits===1:
            result.state===state&&result.diagnostics.historyCommits===0);
        arm.steps.push({repeat,phase:repeat===0?'first-full-call':'repeated-identical-input',fullDtMs,accepted:result.accepted,status:result.status,
            originalStateHash:before,preparedHash,resultHash:hash(result.state),originalUnchanged:true,diagnostics:serial(result.diagnostics)});
    }
    for(let pair=-3;pair<6;pair++)for(const arm of pair%2?[...arms].reverse():arms) {
        const row={pair,...arm.operator.run()};(pair<0?arm.coldOperatorRows:arm.operatorRows).push(row);
    }
    const result={insertion,commonInputHash,commonInputsIdentical:true,pairProfileAndMeshSetupMs:prepared.pairSetupMs,
        singleProjectionStateSetupMs:prepared.singleStateSetupMs,arms:arms.map(arm=>({population:arm.population,elementBackend:arm.elementBackend,
            metadata:arm.operator.metadata,operatorSetup:arm.operator.setup,operatorRows:arm.operatorRows,coldOperatorRows:arm.coldOperatorRows,steps:arm.steps,
            operatorSummary:Object.fromEntries(['elasticMs','inertiaMs','lengthRowsMs','operatorMs','directionMs','totalMs'].map(key=>[key,stats(arm.operatorRows.map(row=>row[key]))])),
            fullSummary:{accepted:arm.steps.filter(r=>r.accepted).length,rejected:arm.steps.filter(r=>!r.accepted).length,firstCallMs:arm.steps[0].fullDtMs,
                warmMs:stats(arm.steps.slice(1).map(r=>r.fullDtMs)),evaluations:arm.steps.map(r=>r.diagnostics.evaluations),directions:arm.steps.map(r=>r.diagnostics.directions),
                backsolves:arm.steps.map(r=>r.diagnostics.linearSolves)}}))};
    report.cases.push(result);fs.writeFileSync(output+'.partial',JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify({insertion,arms:result.arms.map(a=>({population:a.population,backend:a.elementBackend,metadata:a.metadata,
        elasticMs:a.operatorSummary.elasticMs.median,operatorMs:a.operatorSummary.operatorMs.median,directionMs:a.operatorSummary.directionMs.median,full:a.fullSummary}))}));
}
report.snapshotIntact=intact();report.sourceAfter=ownHashes();report.ownSourcesStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);
assert.ok(report.snapshotIntact&&report.ownSourcesStable);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,snapshot,ownSourcesStable:report.ownSourcesStable,snapshotIntact:report.snapshotIntact}));
