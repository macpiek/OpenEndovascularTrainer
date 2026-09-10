import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {performance} from 'node:perf_hooks';

const root=process.cwd(),report=path.resolve(root,process.argv.find(v=>v.startsWith('--baseline-report='))?.slice(18)??'reports/composite-continuous-reverse-tape'),temporary=fs.mkdtempSync(path.join(os.tmpdir(),'oet-tape-comparison-')),
    argument=(key,fallback)=>Number(process.argv.find(v=>v.startsWith('--'+key+'='))?.split('=')[1]??fallback),
    warmup=argument('warmup',12),samples=argument('samples',30),cases=[{count:3,fixedNodes:[0,1]},{count:6,fixedNodes:[0]},{count:6,fixedNodes:[0,1]}],
    output=process.argv.find(v=>v.startsWith('--output='))?.slice(9)??'/tmp/oet-reverse-tape-joint-benchmark.json';
assert.ok(Number.isInteger(warmup)&&warmup>=0&&Number.isInteger(samples)&&samples>0);
const stats=values=>{const s=values.slice().sort((a,b)=>a-b);return {mean:values.reduce((a,b)=>a+b,0)/values.length,median:s[Math.floor(s.length/2)],p95:s[Math.min(s.length-1,Math.ceil(.95*s.length)-1)]};};
const result={scope:'paired full continuous joint steps of two synthetic loaded rods, no contact/anatomy/render, fixed initial state replay; not FPS or an advancing insertion trajectory',warmupPairs:warmup,measurementPairs:samples,cases:[]};

async function api(directory) {
    const names=['Chain','Element','JointTimeStep','JointMaterialHistory'],all={};
    for(const name of names)Object.assign(all,await import(pathToFileURL(path.join(directory,'src/physics/kirchhoffComposite'+name+'.js')).href));
    return all;
}
const dt=1/120,ids=['wire','catheter'];
function setup(a,count,fixedNodes) {
    const height=count===3?.12:0,arc=height===0?1:.5*Math.sqrt(1+4*height*height)+Math.asinh(2*height)/(4*height),
        coordinates=Array.from({length:count},(_,j)=>arc*j),positions=coordinates.map((_,j)=>[j,height*j*(2-j),0]),
        toolPositions=id=>positions.map(p=>p.map((v,k)=>v+(id==='wire'&&k===1?.03:0))),
        tools=ids.map(id=>{const material=a.compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4});
            return {id,dsDx:1,reference:a.captureCompositeReferenceFrames(toolPositions(id),[0,0,1]),referenceTwists:new Float64Array(count-2),material,
                ...(height===0?{}:{materialAt:({coordinate})=>({...material,intrinsic:[-2*height/(1+4*height*height*(1-coordinate/arc)**2)/arc,0,0]})})};}),
        layout=a.createCompositeChainLayout(coordinates.slice(1).map(()=>ids)),modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        state=a.createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,.03,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(count-1)])),restLengths:new Map(ids.map(id=>[id,new Float64Array(count-1).fill(arc)])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength',materialCoordinate:'reference-arclength'}),
        workspace=a.createCompositeJointTimeStepWorkspace(state);
    function inertiaFor(current) {
        const inertia={dt,previousPositions:structuredClone(current.toolPositions),inertiaEdges:current.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
            massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+current.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
        if(current.step>0) {
            const h=a.createCompositeJointMaterialHistory({materialVelocities:current.materialVelocities}),prepared=h.prepare({coordinates:current.coordinates,inertiaEdges:inertia.inertiaEdges});
            for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
        }
        return inertia;
    }
    const input={dt,torsionMode:'quasi-static',contacts:'none',workspace,inertia:inertiaFor(state),
        boundaries:{positions:ids.flatMap(toolId=>fixedNodes.map(node=>({toolId,node,value:state.toolPositions.get(toolId)[node]}))),spins:ids.map(toolId=>({toolId,edge:0,value:0}))},
        loads:{forces:[{toolId:'wire',node:count-1,value:[0,.004,.002]},{toolId:'catheter',node:count-1,value:[0,-.003,-.002]}]}};
    return {state,input,workspace,inertiaFor,run:(current=state,own=input)=>a.advanceCompositeJointTimeStep(current,own)};
}
function physics(r) {
    return {positions:r.state.toolPositions,angles:r.state.angles,relative:r.state.relative,
        lengthMultipliers:r.state.lengthMultipliers,boundaryMultipliers:r.state.boundaryMultipliers,tautLengthState:r.state.tautLengthState,materialVelocities:r.state.materialVelocities,
        frames:r.state.tools.map(t=>({id:t.id,reference:t.reference,referenceTwists:t.referenceTwists})),
        boundaryForces:r.boundaryForces,spinReactions:r.spinReactions,perTool:r.perTool,balances:r.balances,
        certificate:r.diagnostics.certificate};
}
function numericDifference(left,right) {
    let maximum=0,count=0;
    function walk(a,b) {
        if(typeof a==='number') {assert.equal(typeof b,'number');if(Number.isNaN(a)){assert.ok(Number.isNaN(b));return;}assert.ok(Number.isFinite(a)&&Number.isFinite(b));maximum=Math.max(maximum,Math.abs(a-b));count++;return;}
        if(a instanceof Map){assert.ok(b instanceof Map);assert.deepEqual([...a.keys()],[...b.keys()]);for(const [k,v] of a)walk(v,b.get(k));return;}
        if(a&&typeof a==='object'){assert.deepEqual(Object.keys(a),Object.keys(b));for(const key of Object.keys(a))walk(a[key],b[key]);return;}
        assert.equal(a,b);
    }
    walk(left,right);return {maximum,scalarCount:count};
}
function counters(r) {const d=r.diagnostics;return Object.fromEntries(['directions','evaluations','fullAssemblies','gradientAssemblies','factorizations','linearSolves','lineSearchTrials','acceptedAlphas'].map(k=>[k,d[k]]));}
function requireAccepted(r) {assert.equal(r.accepted,true,JSON.stringify({status:r.status,diagnostics:r.diagnostics}));}
try {
    fs.cpSync(path.join(root,'src/physics'),path.join(temporary,'src/physics'),{recursive:true});
    fs.writeFileSync(path.join(temporary,'package.json'),'{"type":"module"}');
    fs.symlinkSync(path.join(root,'node_modules'),path.join(temporary,'node_modules'),'dir');
    fs.cpSync(path.join(report,'before-frame.js'),path.join(temporary,'src/physics/kirchhoffCompositeContinuousFrame.js'));
    fs.cpSync(path.join(report,'before-elasticity.js'),path.join(temporary,'src/physics/kirchhoffCompositeContinuousElasticity.js'));
    if(fs.existsSync(path.join(report,'before-tape.js')))fs.cpSync(path.join(report,'before-tape.js'),path.join(temporary,'src/physics/kirchhoffCompositeStrainDifferentialTape.js'));
    const apis=[await api(temporary),await api(root)];
    for(const {count,fixedNodes} of cases) {
        const plans=apis.map(a=>setup(a,count,fixedNodes)),initial=plans.map(p=>p.run());
        if(initial.some(r=>!r.accepted)) {
            const row={nodesPerTool:count,fixedNodes,benchmarkable:false,reason:'original or candidate step failed its physical acceptance',
                outcomes:initial.map(r=>({accepted:r.accepted,status:r.status,diagnostics:r.diagnostics}))};
            result.cases.push(row);console.log(JSON.stringify({nodesPerTool:count,fixedNodes,benchmarkable:false,outcomes:initial.map(r=>({accepted:r.accepted,status:r.status,lastInvalidTrial:r.diagnostics.lastInvalidTrial}))}));continue;
        }
        const difference=numericDifference(physics(initial[0]),physics(initial[1]));assert.ok(difference.maximum<1e-9);
        assert.deepEqual(counters(initial[0]),counters(initial[1]));
        const nextInputs=plans.map((p,j)=>({...p.input,inertia:p.inertiaFor(initial[j].state)})),second=plans.map((p,j)=>p.run(initial[j].state,nextInputs[j]));second.forEach(requireAccepted);
        const secondDifference=numericDifference(physics(second[0]),physics(second[1]));assert.ok(secondDifference.maximum<1e-9);
        assert.deepEqual(counters(second[0]),counters(second[1]));
        const times=[[],[]],phaseNames=['preparationMs','iterationMs','directionMs','commitMs'],phases=[{},{}];
        for(const p of phases)for(const key of phaseNames)p[key]=[];
        for(let pair=0;pair<warmup+samples;pair++)for(const side of [pair%2,1-pair%2]) {
            const start=performance.now(),r=plans[side].run(),elapsed=performance.now()-start;requireAccepted(r);
            if(pair>=warmup){times[side].push(elapsed);for(const key of phaseNames)phases[side][key].push(r.diagnostics[key]);}
        }
        const row={nodesPerTool:count,fixedNodes,benchmarkable:true,case:count===3?'curved-clamped-bending':'wider-continuous-loaded-bending',firstStepDifference:difference,secondStepDifference:secondDifference,
            counters:initial.map(counters),secondCounters:second.map(counters),scratchBytes:plans.map(p=>p.workspace.diagnostics.assembly.continuousFrames.retainedBytes),
            stepMs:times.map(stats),phaseMs:phases.map(p=>Object.fromEntries(phaseNames.map(key=>[key,stats(p[key])]))),samples:times};
        result.cases.push(row);console.log(JSON.stringify({...row,samples:undefined}));
    }
} finally {
    fs.writeFileSync(output,JSON.stringify(result,null,2)+'\n');fs.rmSync(temporary,{recursive:true,force:true});
}
