import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {prepareSingleVsPair} from './helpers/composite-single-vs-pair.js';

const snapshot=process.argv[3]??'/tmp/oet-composite-exact-snapshot-iBYhkf',output=process.argv[2]??'/tmp/oet-composite-single-vs-pair-setup.json';
const load=relative=>import(pathToFileURL(path.join(snapshot,relative)).href);
const modules={fixture:await load('scripts/physics/helpers/compositeTimeStepBenchmark.js'),chain:await load('src/physics/kirchhoffCompositeChain.js'),time:await load('src/physics/kirchhoffCompositeTimeStep.js')};
const serial=value=>typeof value==='function'?'[frozen-function]':ArrayBuffer.isView(value)?Array.from(value,serial):Array.isArray(value)?value.map(serial):value instanceof Map?[...value].map(serial):value&&typeof value==='object'?Object.fromEntries(Object.entries(value).map(([k,v])=>[k,serial(v)])):value;
const hash=value=>createHash('sha256').update(JSON.stringify(serial(value))).digest('hex');
const manifest=JSON.parse(fs.readFileSync(path.join(snapshot,'manifest.json')));
const report={snapshot,scope:'Separate SETUP CONTROL: same TimeStep and strict tolerances, but explicit evaluations:0 budget. Every attempt must reject before any evaluation/direction/commit. It measures call setup/freezing/compilation plus fail-path return, not an accepted physics dt and not an exact partition of another timed call.',rows:[]};
for(const insertion of [9,160,310]){
    const {cases,commonInputs}=prepareSingleVsPair(modules,insertion);
    const variants=['single','pair'].flatMap(population=>['wasm','wasm-exact'].map(elementBackend=>({population,elementBackend})));
    for(let repeat=0;repeat<3;repeat++)for(const variant of repeat%2?[...variants].reverse():variants){
        const f=cases[variant.population],before=hash(f.state),options={...f.options,elementBackend:variant.elementBackend,budget:{...f.options.budget,evaluations:0}};
        const start=performance.now(),result=modules.time.advanceCompositeTimeStep(f.state,options),elapsedMs=performance.now()-start,d=result.diagnostics;
        assert.equal(result.accepted,false);assert.equal(result.state,f.state);assert.equal(hash(f.state),before);
        assert.equal(d.evaluations,0);assert.equal(d.directions,0);assert.equal(d.linearSolves,0);assert.equal(d.historyCommits,0);
        report.rows.push({insertion,...variant,repeat,elapsedMs,status:result.status,commonInputHash:hash(commonInputs),originalUnchanged:true,diagnostics:serial(d)});
    }
}
report.snapshotIntact=Object.entries(manifest.files).every(([p,m])=>createHash('sha256').update(fs.readFileSync(path.join(snapshot,p))).digest('hex')===m.sha256);
assert.ok(report.snapshotIntact);fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,snapshot,attempts:report.rows.length,accepted:0,snapshotIntact:report.snapshotIntact,
    rows:report.rows.map(({diagnostics,...row})=>row)},null,2));
