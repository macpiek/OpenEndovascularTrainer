import fs from 'node:fs';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {readBalloonRuntimeReplay,runBalloonRuntimeReplay,balloonReplayView,referenceBalloonProjection} from '../tests/helpers/balloonRuntimeReplay.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {sampleBalloonOnCatheter} from '../src/devices/compliantBalloon.js';
import {VesselDeformation} from '../src/vesselDeformation.js';
const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline'),fixture=readBalloonRuntimeReplay();
const statistics=values=>{const sorted=values.slice().sort((a,b)=>a-b);return {samples:sorted.length,meanMs:values.reduce((a,b)=>a+b,0)/values.length,
    medianMs:sorted[sorted.length>>1],p95Ms:sorted[Math.ceil(sorted.length*.95)-1],maxMs:sorted.at(-1),within60HzBudget:values.filter(ms=>ms<=1000/60).length};};
const report={scope:'Node CPU wall time for complete accepted 1/60 s steps; excludes browser render/flow/UI. Surface sampling is measured separately. No browser FPS claim.',
    dt:fixture.stepRequest.dt,node:process.version,runs:[],identicalMechanics:true,identicalSurface:true};
const deformation=new VesselDeformation();
try {
    let baseline;
    for(const [i,optimized] of [false,true,true,false].entries()) {
        const run=runBalloonRuntimeReplay(fixture,anatomy.field,{optimized});
        baseline??=run.poses;assert.deepEqual(run.poses,baseline);
        report.runs.push({optimized,warmup:i<2,steps:run.steps,summary:statistics(run.steps.map(s=>s.wallMs))});
        console.log(JSON.stringify({optimized,warmup:i<2,...report.runs.at(-1).summary}));
    }
    const state=restoreSharedAxisReplay(fixture,anatomy.field),view=balloonReplayView(state),catheter={physicsBody:{jointStateView:view}};
    const samplers=[()=>referenceBalloonProjection(view,anatomy.geometry.boundsTree,4,deformation),
        ()=>sampleBalloonOnCatheter(catheter,anatomy.geometry.boundsTree,4,deformation)];
    assert.deepEqual(samplers[0](),samplers[1]());
    const samples=[[],[]];
    for(let i=0;i<45;i++)for(const j of i%2?[1,0]:[0,1]) {
        const start=performance.now();samplers[j]();const ms=performance.now()-start;if(i>=5)samples[j].push(ms);
    }
    report.projection={reference:statistics(samples[0]),optimized:statistics(samples[1])};
    console.log(JSON.stringify(report.projection));
    fs.writeFileSync(process.argv[2]??'/tmp/balloon-runtime-report.json',JSON.stringify(report,null,2)+'\n');
}finally{deformation.dispose();anatomy.dispose();}
