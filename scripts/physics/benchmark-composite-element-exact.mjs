import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import assert from 'node:assert/strict';

// Every executed dependency is copied before import. Concurrent root changes
// cannot alter this paired GN/exact experiment or its prepared fixtures.
const root=fileURLToPath(new URL('../../',import.meta.url)),output=path.resolve(process.argv[2]??'/tmp/oet-composite-element-exact.json');
const snapshot=fs.mkdtempSync('/tmp/oet-composite-exact-snapshot-'),files=new Map();
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const entry='scripts/physics/helpers/compositeTimeStepBenchmark.js';
function capture(relative,recurse=true) {
    if(files.has(relative))return;
    const bytes=fs.readFileSync(path.join(root,relative));files.set(relative,{sha256:sha(bytes),bytes:bytes.length});
    const target=path.join(snapshot,relative);fs.mkdirSync(path.dirname(target),{recursive:true});fs.writeFileSync(target,bytes);
    if(!recurse||!/\.(js|mjs)$/.test(relative))return;
    for(const match of bytes.toString('utf8').matchAll(/(?:\bfrom\s*|\bimport\s*\(?\s*)['"]([^'"]+)['"]/g)) {
        const specifier=match[1];
        if(specifier.startsWith('.'))capture(path.normalize(path.join(path.dirname(relative),specifier)));
        else if(specifier==='three'){capture('node_modules/three/package.json',false);capture('node_modules/three/build/three.module.js');}
        else if(!specifier.startsWith('node:'))throw new Error('Uncaptured import '+specifier);
    }
}
capture(entry);capture('scripts/physics/benchmark-composite-element-exact.mjs',false);
fs.writeFileSync(path.join(snapshot,'package.json'),'{"type":"module"}\n');
assert.ok([...files].every(([p,m])=>sha(fs.readFileSync(path.join(root,p)))===m.sha256),'source changed while freezing snapshot');
fs.writeFileSync(path.join(snapshot,'manifest.json'),JSON.stringify({root,snapshot,files:Object.fromEntries(files)},null,2)+'\n');
for(const p of files.keys())fs.chmodSync(path.join(snapshot,p),0o444);
fs.chmodSync(path.join(snapshot,'package.json'),0o444);fs.chmodSync(path.join(snapshot,'manifest.json'),0o444);
const seal=dir=>{for(const e of fs.readdirSync(dir,{withFileTypes:true}))if(e.isDirectory())seal(path.join(dir,e.name));fs.chmodSync(dir,0o555);};seal(snapshot);
const load=relative=>import(pathToFileURL(path.join(snapshot,relative)).href);
const chain=await load('src/physics/kirchhoffCompositeChain.js'),element=await load('src/physics/kirchhoffCompositeElement.js');
const {makeFixture,preparedOptions,advance,conventions}=await load(entry);
const serial=value=>typeof value==='function'?'[frozen-function]':ArrayBuffer.isView(value)?Array.from(value,serial):Array.isArray(value)?value.map(serial):value instanceof Map?[...value].map(serial):value&&typeof value==='object'?Object.fromEntries(Object.entries(value).map(([k,v])=>[k,serial(v)])):value;
const hash=value=>sha(JSON.stringify(serial(value)));
const median=values=>{const a=values.toSorted((a,b)=>a-b),i=a.length>>1;return a.length%2?a[i]:(a[i-1]+a[i])/2;};
const stats=values=>({median:median(values),mean:values.reduce((a,b)=>a+b)/values.length,max:Math.max(...values),samples:values});
const report={snapshot,sourceStableAtCapture:true,snapshotManifestHash:sha(fs.readFileSync(path.join(snapshot,'manifest.json'))),
    scope:'Exact elastic energy Hessian vs unchanged GN. Assembly65 microfixture plus real Glidewire/Berenstein wire318/cat310 full mixed dt on identical canonical prepared states. Fixed mesh, no feed/remesh/history transfer/anatomy/FPS claim.',
    conventions,assembly:null,steps:[],comparison:[]};
let assemblyState;
// Assembly-only control, including every local element and full band scatter.
{
    const count=65,coordinates=Array.from({length:count},(_,i)=>i*3),positions=coordinates.map(x=>[x,.2*Math.sin(x/40),.1*Math.cos(x/37)]);
    const layout=chain.createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter']));
    const data={positions,coordinates,reference:element.captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:Float64Array.from(coordinates.slice(1),x=>.1*Math.sin(x/50)),material:element.compileCompositeMaterial({EI1:2,EI2:3,GJ:1.5,kappa0:[.001,0]})},
        {id:'catheter',angles:Float64Array.from(coordinates.slice(1),x=>.3+.02*x),material:element.compileCompositeMaterial({EI1:7,EI2:5,GJ:3,kappa0:[0,.003]})}]};
    const workspaces={},creation={},times={wasm:[],'wasm-exact':[]},coldTimes={wasm:[],'wasm-exact':[]};
    for(const backend of ['wasm','wasm-exact']){const start=performance.now();workspaces[backend]=chain.createCompositeChainWorkspace(layout,{elementBackend:backend});creation[backend]=performance.now()-start;}
    for(let pair=-6;pair<8;pair++) {
        for(const backend of pair%2?['wasm-exact','wasm']:['wasm','wasm-exact']){
            const start=performance.now();chain.assembleCompositeChain(data,workspaces[backend]);const elapsed=performance.now()-start;
            if(pair>=0)times[backend].push(elapsed);else coldTimes[backend].push(elapsed);
        }
        const a=workspaces.wasm,b=workspaces['wasm-exact'];assert.equal(a.energy,b.energy);
        assert.ok(a.gradient.every((v,i)=>Math.abs(v-b.gradient[i])<=2e-12*(1+Math.abs(v))));
    }
    assemblyState={data,workspaces};
    report.assembly={coldAssemblyWarmupsMs:coldTimes,nodes:count,dofCount:layout.dofCount,band:layout.band,inputHash:hash(data),workspaceCreationMs:creation,
        gnMs:stats(times.wasm),exactMs:stats(times['wasm-exact']),hessianDifferent:workspaces.wasm.hessian.some((v,i)=>Math.abs(v-workspaces['wasm-exact'].hessian[i])>1e-8)};
}
// Exact and GN always solve the SAME canonical state/input in a given pair.
// A converged GN result becomes the next common state (exact result only if GN
// failed). This prevents different trajectories from masquerading as parity.
for(const scenario of ['contact-free','analytic-plane'])for(let repeat=0;repeat<2;repeat++) {
    const setupStart=performance.now(),fixture=makeFixture(310,scenario),setupMs=performance.now()-setupStart;
    let state=fixture.state;
    for(let attempt=0;attempt<2;attempt++) {
        const pairResults={},before=hash(state),base=preparedOptions(fixture,state,'strict-tests');
        base.constraintSolver='mixed';base.inertiaBackend='compiled';
        const inputHash=hash(base),caseRows={};
        for(const elementBackend of repeat%2?['wasm-exact','wasm']:['wasm','wasm-exact']) {
            const options={...base,elementBackend},start=performance.now(),result=advance(state,options),fullDtMs=performance.now()-start;
            assert.equal(hash(state),before);assert.equal(hash(base),inputHash);
            assert.ok(result.accepted?result.state.step===state.step+1&&result.state.time===state.time+base.dt&&result.diagnostics.historyCommits===1:
                result.state===state&&result.diagnostics.historyCommits===0);
            const row={scenario,repeat,attempt,elementBackend,stage:state.step===0?'initial-or-retry':'consecutive-loaded-dt',nodes:state.layout.nodeCount,
                dofCount:state.layout.dofCount,setupMs:attempt?0:setupMs,fullDtMs,accepted:result.accepted,status:result.status,
                preparedInputHash:inputHash,originalStateHash:before,resultStateHash:hash(result.state),originalUnchanged:true,
                diagnostics:serial(result.diagnostics)};
            report.steps.push(row);caseRows[elementBackend]=row;pairResults[elementBackend]=result;
            console.log(JSON.stringify({scenario,repeat,attempt,elementBackend,fullDtMs,accepted:result.accepted,status:result.status,
                evaluations:result.diagnostics.evaluations,directions:result.diagnostics.directions,backsolves:result.diagnostics.linearSolves}));
        }
        const gn=pairResults.wasm,ex=pairResults['wasm-exact'];let positionDifference=null,spinDifference=null;
        if(gn.accepted&&ex.accepted){positionDifference=Math.max(...gn.state.data.positions.flatMap((p,i)=>p.map((v,j)=>Math.abs(v-ex.state.data.positions[i][j]))));
            spinDifference=Math.max(...gn.state.data.tools.flatMap((tool,i)=>Array.from(state.layout.spins.get(tool.id),(dof,edge)=>dof<0?0:Math.abs(tool.angles[edge]-ex.state.data.tools[i].angles[edge]))));
            assert.ok(Number.isFinite(positionDifference)&&Number.isFinite(spinDifference));}
        report.comparison.push({scenario,repeat,attempt,samePreparedInput:true,sameOriginalState:true,bothAccepted:gn.accepted&&ex.accepted,
            positionDifference,spinDifference,canonicalNextState:gn.accepted?'wasm':ex.accepted?'wasm-exact':'original-retry'});
        state=gn.accepted?gn.state:ex.accepted?ex.state:state;
        fs.writeFileSync(output+'.partial',JSON.stringify(report,null,2)+'\n');
    }
}
{
    const times={wasm:[],'wasm-exact':[]};
    for(let pair=0;pair<8;pair++)for(const backend of pair%2?['wasm-exact','wasm']:['wasm','wasm-exact']) {
        const start=performance.now();chain.assembleCompositeChain(assemblyState.data,assemblyState.workspaces[backend]);times[backend].push(performance.now()-start);
    }
    report.assembly.postStepWarm={gnMs:stats(times.wasm),exactMs:stats(times['wasm-exact'])};
}
report.summary=['contact-free','analytic-plane'].map(scenario=>({scenario,arms:Object.fromEntries(['wasm','wasm-exact'].map(backend=>{
    const rows=report.steps.filter(r=>r.scenario===scenario&&r.elementBackend===backend),accepted=rows.filter(r=>r.accepted),rejected=rows.filter(r=>!r.accepted);
    return [backend,{accepted:accepted.length,rejected:rejected.length,acceptedDtMs:accepted.length?stats(accepted.map(r=>r.fullDtMs)):null,
        rejectedDtMs:rejected.map(r=>r.fullDtMs),evaluations:rows.map(r=>r.diagnostics.evaluations),directions:rows.map(r=>r.diagnostics.directions),
        backsolves:rows.map(r=>r.diagnostics.linearSolves),statuses:rows.map(r=>r.status)}];}))}));
report.snapshotIntact=[...files].every(([p,m])=>sha(fs.readFileSync(path.join(snapshot,p)))===m.sha256);
report.rootChangedSinceSnapshot=[...files].filter(([p,m])=>sha(fs.readFileSync(path.join(root,p)))!==m.sha256).map(([p])=>p);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,snapshot,snapshotIntact:report.snapshotIntact,rootChangedSinceSnapshot:report.rootChangedSinceSnapshot,
    assembly:report.assembly,summary:report.summary},null,2));
