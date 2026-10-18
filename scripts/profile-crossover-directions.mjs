// Diagnose direction-search work on the captured long-tool continuation.
// Requires the direction-profile instrumented checkout. Scalar counters preserve
// fallback caches; observeTrial would disable them and distort this workload.
// This continuation is diagnostic, never a full-maneuver performance result.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';

const root=pathToFileURL(resolve(process.argv[2]??'.')+'/');
const output=process.argv[3];
assert.ok(output,'Usage: node scripts/profile-crossover-directions.mjs checkout output.json');
assert.ok(fs.existsSync(new URL('src/physics/kirchhoffSharedAxisDirectionProfile.js',root)),
    'Checkout needs the nonintrusive direction-profile patch before this diagnostic can run');
const fixture=JSON.parse(gunzipSync(fs.readFileSync(new URL('reports/performance-scenarios/001-stage-05-replay.json.gz',root))));
const {loadCoupledRuntimeAnatomy}=await import(new URL('tests/helpers/coupledRuntimeFixture.js',root));
const {restoreSharedAxisReplay}=await import(new URL('tests/helpers/sharedAxisReplay.js',root));
const {advanceSharedAxis,SHARED_AXIS_APP_PHYSICS_OPTIONS}=await import(new URL('src/physics/kirchhoffSharedAxisAppSystem.js',root));
const {captureBloodFlow}=await import(new URL('src/physics/bloodFlowCoupling.js',root));
const {createCrossoverFlow}=await import(new URL('tests/helpers/crossoverFlow.js',root));
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),request=fixture.stepRequest;
const options={...request.options,...SHARED_AXIS_APP_PHYSICS_OPTIONS,contactCandidateMargin:.01,
    lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,reuseFrictionAssembly:true,profileDirections:true};
const flow=createCrossoverFlow();flow.time=request.options.bloodFlow.time;
// Restore the recorded topology first. The current row policy takes effect
// on the next feed, just as in the paired continuation benchmark.
const restoreOptions={...options};delete restoreOptions.omitInactiveSampleRows;
let state=restoreSharedAxisReplay({...fixture,...restoreOptions},anatomy.field),rotations=request.rotations;
const report={scope:'100 diagnostic continuation steps, not the complete human maneuver; no performance claim',
    methods:{},projectionFactorizations:0,steps:[],goalAchieved:false};
try {
    for(let step=0;step<100;step++) {
        const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
        const targets=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...targets.map(t=>t.insertion)));
        const iterator=advanceSharedAxis(state,rotations,request.dt,targets,{...options,bloodFlow});
        let next;do{next=iterator.next();}while(!next.done);
        const r=next.value.result;
        assert.ok(next.value.state&&r.converged&&r.quality.finite);
        assert.ok(r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5);
        state=next.value.state;rotations=next.value.rotations;flow.update(request.dt);
        assert.ok(r.directionProfile,'Checkout lacks nonintrusive direction profiling');
        let directionFactorizations=0;
        for(const [key,d] of Object.entries(r.directionProfile)) {
            const item=report.methods[key]??={calls:0,attempts:0,factorizations:0,maximumAttempts:0};
            item.calls+=d.calls;item.attempts+=d.attempts;item.factorizations+=d.factorizations;
            item.maximumAttempts=Math.max(item.maximumAttempts,d.maximumAttempts);
            directionFactorizations+=d.factorizations;
        }
        // Native adds these separately for correctTrialConstraints. They are
        // not Newton/GN direction attempts, but must remain in total work.
        const projectionFactorizations=r.factorizations-directionFactorizations;
        assert.ok(projectionFactorizations>=0,'Direction counts exceed step work');
        report.projectionFactorizations+=projectionFactorizations;
        report.steps.push({step,iterations:r.iterations,factorizations:r.factorizations,
            projectionFactorizations,directions:r.directionProfile});
    }
} finally {
    anatomy.dispose();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
}
console.log(JSON.stringify(report.methods,null,2));
