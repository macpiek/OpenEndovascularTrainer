// Capture and diagnose the costly final step of the measured 100-step replay.
// This deliberately records extra observations; use the throughput benchmark
// for performance numbers, not this diagnostic's elapsed time.
import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';
const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const fixture=JSON.parse(gunzipSync(fs.readFileSync(new URL('001-stage-05-replay.json.gz',directory))));
const request=fixture.stepRequest,options={...request.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true};
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),flow=createCrossoverFlow();flow.time=options.bloodFlow.time;
const report={recordedAt:new Date().toISOString(),directions:[],reusedDirections:[],passed:false};
try {
    let state=restoreSharedAxisReplay({...fixture,contactCandidateMargin:.01},anatomy.field),rotations=request.rotations;
    for(let step=0;step<100;step++) {
        const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const bloodFlow=captureBloodFlow(state,flow,options.bloodFlow.sheathLength,Math.max(...tools.map(t=>t.insertion)));
        if(step===99) {
            const saved={...captureSharedAxisReplay(state,fixture.sheath),anatomy:fixture.anatomy,
                stepRequest:{dt:request.dt,rotations,tools,options:{...options,bloodFlow}}};
            fs.writeFileSync(new URL('001-contact-tail-replay.json.gz',directory),gzipSync(JSON.stringify(saved)));
        }
        const observeTrial=step===99?event=>{
            if(event.kind==='reused-rejected-direction'){report.reusedDirections.push({iteration:event.iteration,method:event.method});return;}
            if(event.kind!=='direction')return;
            const d=event.direction;
            report.directions.push({iteration:event.iteration,method:event.method,base:{energy:event.base.energy,force:event.base.force,torque:event.base.torque,constraint:event.base.constraint,rows:event.base.rows.length},
                converged:d.converged,failure:d.failure,residual:d.residual,factorizations:d.factorizations,
                activeSetAttempts:d.activeSetAttempts,batchFallback:d.batchFallback,batchFailure:d.batchFailure,
                batchFactorizations:d.batchFactorizations,referenceFactorizations:d.referenceFactorizations,
                batchAttempts:d.batchAttempts,referenceAttempts:d.referenceAttempts});
        }:null;
        const iterator=advanceSharedAxis(state,rotations,request.dt,tools,{...options,bloodFlow,observeTrial});
        let next;do{next=iterator.next();}while(!next.done);
        const r=next.value.result;
        assert.ok(next.value.state&&r.converged&&r.quality.finite&&r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5);
        state=next.value.state;rotations=next.value.rotations;flow.update(request.dt);
        if(step===99){report.result={status:r.status,iterations:r.iterations,factorizations:r.factorizations,subdivisions:r.subdivisions,certificateBound:r.certificateBound,residual:r.residual,attempts:r.attempts};report.passed=true;}
    }
} finally {
    anatomy.dispose();fs.writeFileSync(new URL('001-contact-tail-diagnostic.json',directory),JSON.stringify(report,null,2)+'\n');
}
console.log(JSON.stringify({passed:report.passed,result:report.result,directions:report.directions.length}));
