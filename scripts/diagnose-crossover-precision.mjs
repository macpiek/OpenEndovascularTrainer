// Reconstruct a failing temporal-refinement substep, without changing solver
// tolerances. These observer-heavy timings are not performance measurements.
// node scripts/diagnose-crossover-precision.mjs [outerStep=99] [division=16] [index=8]
import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const outerStep=Number(process.argv[2]??99),division=Number(process.argv[3]??16),index=Number(process.argv[4]??8);
assert.ok(Number.isInteger(outerStep)&&outerStep>=0&&Number.isInteger(division)&&division>0&&Number.isInteger(index)&&index>=0&&index<division);
const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const sourceName=`001-temporal-step-${outerStep}-replay.json.gz`;
const source=fs.readFileSync(new URL(sourceName,directory)),fixture=JSON.parse(gunzipSync(source));
const request=fixture.stepRequest,dt=request.dt/division;
const name=`001-precision-${outerStep}-${division}-${index}`;
const hash=file=>createHash('sha256').update(fs.readFileSync(new URL('../'+file,import.meta.url))).digest('hex');
const report={recordedAt:new Date().toISOString(),outerStep,division,index,dt,sourceName,
    sourceSha256:createHash('sha256').update(source).digest('hex'),
    sourceHashes:Object.fromEntries(['src/physics/kirchhoffSharedAxisNative.js','src/physics/kirchhoffSharedAxisDynamics.js',
        'src/physics/kirchhoffSharedAxisTimeStep.js','src/physics/kirchhoffSharedAxisAppSystem.js',
        'scripts/diagnose-crossover-precision.mjs'].map(f=>[f,hash(f)])),
    warmup:[],solves:[],complete:false,
    limitations:['Frozen blood-flow samples match production subdivision semantics.',
        'Inertia force quantum is an estimate for one coordinate ulp, not a bound on the full nonlinear residual.',
        'The observer reports unscaled Newton corrections; line search and projection can change the final correction.',
        'Observer timings must not be used for performance claims.']};
const drain=iterator=>{let next;do{next=iterator.next();}while(!next.done);return next.value;};
const summary=r=>({converged:r.converged,status:r.status,subdivisions:r.subdivisions,
    certificateBound:r.certificateBound,residual:r.residual,iterations:r.iterations,
    factorizations:r.factorizations,attempts:r.attempts});
const ulp=value=>Math.max(Number.MIN_VALUE,2**(Math.floor(Math.log2(Math.abs(value)))-52));
let solve;
function observeIteration({state:s,iteration,base}) {
    if(iteration===0||!solve) {
        solve={dt:s.dynamicStep.dt,liveNormalLoad:s.wallFrictionStep?.liveNormalLoad,iterations:[],directions:[]};
        report.solves.push(solve);
    }
    let worst=null;
    for(let node=0;node<s.positions.length;node++)for(let axis=0;axis<3;axis++) {
        const dof=s.layout.positions[node]+axis,value=s.chain.gradient[dof];
        if(s.fixed[dof]||(worst&&Math.abs(value)<=Math.abs(worst.residual)))continue;
        const position=s.positions[node][axis],weight=s.dynamicStep.masses[node]/s.dynamicStep.dt**2;
        worst={node,axis,dof,position,residual:value,coordinateUlpMm:ulp(position),
            inertiaForcePerUlp:weight*ulp(position),inertiaWeight:weight,
            inertiaResidual:weight*(position-s.dynamicStep.predicted[node][axis])};
    }
    solve.iterations.push({iteration,force:base.force,torque:base.torque,constraint:base.constraint,
        rows:base.rows.length,worst});
}
function observeTrial(event) {
    if(event.kind!=='direction'||!solve)return;
    const {state:s,direction:d,iteration,method}=event;
    const item={iteration,method,converged:d.converged,failure:d.failure,factorizations:d.factorizations};
    if(d.converged) {
        let maxPositionCorrectionMm=0,roundedAway=0,nonzero=0;
        for(let node=0;node<s.positions.length;node++)for(let axis=0;axis<3;axis++) {
            const dof=s.layout.positions[node]+axis,delta=d.increment[dof];
            if(s.fixed[dof]||delta===0)continue;
            nonzero++;maxPositionCorrectionMm=Math.max(maxPositionCorrectionMm,Math.abs(delta));
            if(s.positions[node][axis]+delta===s.positions[node][axis])roundedAway++;
        }
        const worst=solve.iterations.at(-1).worst,delta=d.increment[worst.dof];
        Object.assign(item,{maxPositionCorrectionMm,nonzero,roundedAway,
            worstForceCorrectionMm:delta,worstForceCorrectionUlps:delta/worst.coordinateUlpMm,
            worstForcePositionUnchanged:worst.position+delta===worst.position});
    }
    solve.directions.push(item);
}
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
try {
    let state=restoreSharedAxisReplay(fixture,anatomy.field),rotations=request.rotations;
    for(let substep=0;substep<index;substep++) {
        const next=drain(advanceSharedAxis(state,rotations,dt,request.tools,request.options));
        report.warmup.push(summary(next.result));
        assert.ok(next.state&&next.result.converged&&next.result.certificateBound<=1e-6&&
            next.result.residual.length<=1e-5&&next.result.quality.maxPenetration<=1e-5);
        state=next.state;rotations=next.rotations;
    }
    const saved={...captureSharedAxisReplay(state,fixture.sheath),anatomy:fixture.anatomy,
        stepRequest:{...request,dt,rotations}};
    report.replay=`${name}-replay.json.gz`;
    fs.writeFileSync(new URL(report.replay,directory),gzipSync(JSON.stringify(saved)));
    const next=drain(advanceSharedAxis(state,rotations,dt,request.tools,{...request.options,observeIteration,observeTrial}));
    report.result=summary(next.result);report.complete=true;
    console.log(JSON.stringify({name,result:report.result,solves:report.solves.map(s=>({dt:s.dt,
        live:s.liveNormalLoad,iterations:s.iterations.length,last:s.iterations.at(-1),
        lastDirection:s.directions.filter(d=>d.converged).at(-1)}))}));
} catch(error) {report.error=error.stack;process.exitCode=1;}
finally {anatomy.dispose();fs.writeFileSync(new URL(`${name}.json`,directory),JSON.stringify(report,null,2)+'\n');}
