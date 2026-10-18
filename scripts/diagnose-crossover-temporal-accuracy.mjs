// Compare contact-recovery ordering with temporal refinement of the SAME
// captured incoming state. Diagnostic only: does not change runtime defaults.
// node scripts/diagnose-crossover-temporal-accuracy.mjs [output.json] [maxLevel]
import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const output=process.argv[2]??new URL('001-temporal-accuracy.json',directory);
const maxLevel=Number(process.argv[3]??16);
assert.ok([4,8,16,32,64].includes(maxLevel));
const bytes=fs.readFileSync(new URL('001-stage-05-replay.json.gz',directory));
const fixture=JSON.parse(gunzipSync(bytes)),request=fixture.stepRequest;
const options={...request.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true};
const targetSteps=new Set([99,152,173,203,239,250,288]);
const sourceFiles=['src/physics/kirchhoffSharedAxisAppSystem.js','src/physics/kirchhoffSharedAxisTimeStep.js',
    'src/physics/kirchhoffSharedAxisNative.js','src/physics/kirchhoffSharedAxisLinear.js',
    'src/physics/kirchhoffSharedAxisConstraintRows.js','src/physics/kirchhoffSharedAxisDynamics.js',
    'src/physics/kirchhoffSharedAxisWallFriction.js','src/physics/bloodFlowCoupling.js',
    'tests/helpers/sharedAxisReplay.js','scripts/diagnose-crossover-temporal-accuracy.mjs'];
const report={recordedAt:new Date().toISOString(),node:process.version,dt:request.dt,maxLevel,
    fixtureSha256:createHash('sha256').update(bytes).digest('hex'),
    sourceHashes:Object.fromEntries(sourceFiles.map(file=>[file,createHash('sha256').update(fs.readFileSync(new URL('../'+file,import.meta.url))).digest('hex')])),
    method:'Regenerate each incoming state using the current production path. Freeze its outer-step blood-flow samples and controls. Restart from that identical state for ordinary recovery, early recovery, and 2/4/... manually divided ordinary steps. Every accepted substep retains the original physical certificate.',
    limitations:['Temporal refinement estimates consistency, not ground truth; contact/friction can make convergence non-monotone.',
        'Frozen flow samples match production subdivision semantics; this does not test a fully coupled fluid substep.',
        'Only one outer timestep per case is compared; this cannot establish long-cycle stability.',
        'Diagnostic elapsed times are not paired performance benchmarks.'],cases:[],complete:false,productionDefaultChanged:false};
const write=()=>fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
const drain=iterator=>{let next;do{next=iterator.next();}while(!next.done);return next.value;};
function certify({state,result:r},label) {
    assert.ok(state&&r.converged&&r.quality?.finite,`${label}: ${r.status}`);
    assert.ok(r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5&&
        r.quality.bodies.every(b=>b.maxLengthError<1e-5),`${label}: physical certificate`);
}
function difference(a,b) {
    assert.deepEqual(a.coordinates,b.coordinates,'Compare the same physical grid');
    let maxPositionMm=0,positionSquared=0,maxVelocityMmPerSecond=0;
    for(let i=0;i<a.positions.length;i++) {
        const distance=Math.hypot(...a.positions[i].map((v,k)=>v+a.origin[k]-b.positions[i][k]-b.origin[k]));
        maxPositionMm=Math.max(maxPositionMm,distance);positionSquared+=distance*distance;
        maxVelocityMmPerSecond=Math.max(maxVelocityMmPerSecond,Math.hypot(...a.velocities[i].map((v,k)=>v-b.velocities[i][k])));
    }
    const tipPositionMm=Object.fromEntries(a.materials.map(m=>{
        const other=b.materials.find(n=>n.spec.id===m.spec.id);
        return [m.spec.id,Math.hypot(...a.positions[m.last].map((v,k)=>v+a.origin[k]-b.positions[other.last][k]-b.origin[k]))];
    }));
    return {maxPositionMm,rmsPositionMm:Math.sqrt(positionSquared/a.positions.length),maxVelocityMmPerSecond,tipPositionMm};
}
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),flow=createCrossoverFlow();
flow.time=options.bloodFlow.time;
try {
    let state=restoreSharedAxisReplay({...fixture,contactCandidateMargin:.01},anatomy.field),rotations=request.rotations;
    for(let step=0;step<=Math.max(...targetSteps);step++) {
        const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const bloodFlow=captureBloodFlow(state,flow,options.bloodFlow.sheathLength,Math.max(...tools.map(t=>t.insertion)));
        const ordinary=drain(advanceSharedAxis(state,rotations,request.dt,tools,{...options,bloodFlow}));
        certify(ordinary,`production step ${step}`);
        if(targetSteps.has(step)) {
            // All selected cases are in the stationary-control settling phase.
            for(const tool of tools) {
                assert.equal(tool.insertion,state.materials.find(m=>m.spec.id===tool.id).spec.insertion);
                assert.equal(tool.rotation,rotations[tool.id]);
            }
            const saved={...captureSharedAxisReplay(state,fixture.sheath),anatomy:fixture.anatomy,
                stepRequest:{dt:request.dt,rotations,tools,options:{...options,bloodFlow}}};
            const replayName=`001-temporal-step-${step}-replay.json.gz`;
            fs.writeFileSync(new URL(replayName,directory),gzipSync(JSON.stringify(saved)));
            const entry={step,replay:replayName,runs:[],comparisons:[]};report.cases.push(entry);
            const states=new Map([['ordinary/1',ordinary.state]]);
            entry.runs.push({name:'ordinary/1',passed:true,substeps:[{subdivisions:ordinary.result.subdivisions,
                certificate:ordinary.result.certificateBound,factorizations:ordinary.result.factorizations,attempts:ordinary.result.attempts}]});
            const variants=[{name:'early/1',count:1,extra:{recoverNegativeContact:true}}];
            for(let count=2;count<=maxLevel;count*=2)variants.push({name:`ordinary/${count}`,count,extra:{}});
            for(const {name,count,extra} of variants) {
                const run={name,passed:false,substeps:[]},start=performance.now();entry.runs.push(run);
                let current=restoreSharedAxisReplay(saved,anatomy.field),currentRotations=rotations;
                try {
                    for(let index=0;index<count;index++) {
                        const next=drain(advanceSharedAxis(current,currentRotations,request.dt/count,tools,{...options,...extra,bloodFlow}));
                        const r=next.result;
                        run.substeps.push({index,converged:r.converged,status:r.status,subdivisions:r.subdivisions,
                            certificate:r.certificateBound,maxPenetrationMm:r.quality?.maxPenetration,
                            factorizations:r.factorizations,attempts:r.attempts});
                        certify(next,`step ${step}, ${name}/${index}`);
                        current=next.state;currentRotations=next.rotations;
                    }
                    states.set(name,current);run.passed=true;
                }catch(error){run.error=error.stack;}
                run.elapsedMs=performance.now()-start;
                console.log(JSON.stringify({step,name,passed:run.passed,elapsedMs:run.elapsedMs,error:run.error}));write();
            }
            for(const [a,b] of [['ordinary/1','early/1'],...Array.from({length:Math.log2(maxLevel)},(_,i)=>
                [`ordinary/${2**i}`,`ordinary/${2**(i+1)}`]),['ordinary/1',`ordinary/${maxLevel}`],['early/1',`ordinary/${maxLevel}`]])
                if(states.has(a)&&states.has(b))entry.comparisons.push({a,b,...difference(states.get(a),states.get(b))});
            console.log(JSON.stringify({step,comparisons:entry.comparisons}));write();
        }
        state=ordinary.state;rotations=ordinary.rotations;flow.update(request.dt);
    }
    report.complete=true;
}catch(error){report.error=error.stack;process.exitCode=1;}
finally{anatomy.dispose();write();}
