import fs from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
const root=resolve(process.env.OET_SHEATH_TRANSITION_SOURCE_ROOT??'/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer');
const source=p=>pathToFileURL(resolve(root,p));
const {createCoupledRuntimeFixture}=await import(source('tests/helpers/coupledRuntimeFixture.js'));
const {createCoupledSolverSelection}=await import(source('src/physics/coupledSolverSelection.js'));
const {solveKirchhoffCoupledSystem:solve,applyKirchhoffCoupledCorrection:apply}=await import(source('src/physics/kirchhoffCoupledSystem.js'));
const {configureKirchhoffSplitBias}=await import(source('src/physics/kirchhoffSplitMotion.js'));
const output=process.argv[2]??'/tmp/oet-sheath-unloaded-boundary-proof-8996.json';
const paths=fs.readdirSync(resolve(root,'src/physics')).filter(p=>p.endsWith('.js')).sort();
const hashes=()=>Object.fromEntries(paths.map(p=>[p,createHash('sha256').update(fs.readFileSync(resolve(root,'src/physics',p))).digest('hex')]));
const report={sourceRoot:root,sourceBefore:hashes(),events:[],steps:[],target:{sheath:0,side:1,node:0}};
let f,attempt=0,pending;
function inspect(stage,extra={}) {
    if(!f?.containment._splitMotion)return;
    const c=f.containment,s=c._splitMotion,b=c.outerBody,sh=f.world.sheaths[0],h=s.sheathHistory.get(sh);
    const p=[b.x[0],b.y[0],b.z[0]],origin=[sh.startX,sh.startY,sh.startZ],axis=[sh.axisX,sh.axisY,sh.axisZ];
    const v=p.map((x,i)=>x-origin[i]),axial=v.reduce((a,x,i)=>a+x*axis[i],0),radius=Math.hypot(...v.map((x,i)=>x-axial*axis[i]));
    const bank=state=>state?.sheaths?.get(sh)?.[1].lambda[0]??0;
    report.events.push({attempt,step:f.world.stepCount,phase:s.phase,stage,axial,radialGap:Math.max(0,sh.innerRadius-b.nodeRadius[0])-radius,
        material:b.materialCoordinate[0],startMaterial:h.materialCoordinates[1][0],
        currentLambda:bank(c._coupledBoundaries),physicalBank:bank(s.bank?.joints?._coupledBoundaries),
        biasBank:bank(s.biasBank?.joints?._coupledBoundaries),legacyLambda:sh.lambdas.get(b)?.[0]??0,
        controlLambda:b.controlLambda[0],...extra});
}
const selection=createCoupledSolverSelection('joint-active-coulomb',{
    solve(c,dt,o) {
        pending=o.additionalRows.map((r,index)=>({kind:r.kind,side:r.side,node:r.node,lambda:r.lambda,index})).filter(r=>r.kind==='sheath'&&r.side===1&&r.node===0);
        inspect('before-solve',{rows:pending});
        return solve(c,dt,o);
    },
    apply(c,result) {
        inspect('actual-apply',{scale:result.scale,applied:pending.map(r=>({...r,delta:result.scale*result.additionalIncrement[r.index],
            next:Math.max(0,r.lambda+result.scale*result.additionalIncrement[r.index])}))});
        apply(c,result);
    }
});
f=createCoupledRuntimeFixture({coupledSystem:selection.coupledSystem,jointMotionMode:'split-physical-bias'});
configureKirchhoffSplitBias(f.containment,{materialMode:'preserve-strain'});
f.world.debugJointTrial=(c,s,pass,trial)=>inspect('after-trial',{pass,trial,settled:s.settled});
f.containment.outerBody.debugConstraintPhase=phase=>{if(phase==='closureEnd')inspect('phase-end');};
try {
    for(let i=0;i<33;i++)f.step({guidewireAdvance:1});
    for(attempt=1;attempt<=14;attempt++) {
        const r=f.step({catheterAdvance:1});
        if(f.world.getStats().jointMotion)report.steps.push({attempt,state:f.snapshot(),accepted:r.accepted!==false,diagnostics:structuredClone(f.world.getStats().jointMotion)});
        if(r.accepted===false)break;
    }
    report.sourceAfter=hashes();report.sourceStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);
    const target=report.events.filter(e=>e.attempt===14);
    report.targetSummary={events:target.length,phases:[...new Set(target.map(e=>e.phase))],
        maxPhysicalOrBiasBank:Math.max(0,...target.flatMap(e=>[Math.abs(e.currentLambda),Math.abs(e.physicalBank),Math.abs(e.biasBank),Math.abs(e.legacyLambda)])),
        maxAppliedLambda:Math.max(0,...target.flatMap(e=>(e.applied??[]).flatMap(r=>[Math.abs(r.lambda),Math.abs(r.delta),Math.abs(r.next)]))),
        minimumObservedRadialGap:Math.min(...target.map(e=>e.radialGap)),identityStable:target.every(e=>e.material===e.startMaterial)};
    fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify({output,sourceStable:report.sourceStable,targetSummary:report.targetSummary,
        candidateIssues:report.steps.at(-1)?.diagnostics.sheathHistoryIssues},null,2));
} finally {f.dispose();}
