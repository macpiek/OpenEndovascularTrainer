// Paired runs from identical accepted poses in the actual aneurysm STL.
// Run: node scripts/benchmark-blood-flow-aneurysm.mjs [output.json]
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {Vector3} from 'three';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

const saved=JSON.parse(gunzipSync(fs.readFileSync(new URL('../tests/fixtures/shared-axis/aneurysm-wire-272.80-sparse-interior.json.gz',import.meta.url))));
const anatomy=await loadCoupledRuntimeAnatomy(undefined,'infrarenal-aneurysm');
const bytes=fs.readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),data=asset.arrays.centerlineSegments,segments=[];
for(let i=0;i<data.length;i+=9)segments.push({id:i/9,start:new Vector3().fromArray(data,i),end:new Vector3().fromArray(data,i+3),radiusStart:data[i+6],radiusEnd:data[i+7],safeRadius:data[i+8],nodeStartId:asset.arrays.centerlineEdges[i/9*2],nodeEndId:asset.arrays.centerlineEdges[i/9*2+1]});
const duration=20,steps=Math.round(duration*60),dt=1/60,options=saved.stepRequest.options,sheath=saved.sheath;
const dist=(a,b)=>Math.hypot(...a.map((v,k)=>v-b[k]));
function solve(state,rotations,tools,bloodFlow=null){
    const it=advanceSharedAxis(state,rotations,dt,tools,{...options,bloodFlow});let n;
    do{n=it.next();}while(!n.done);return n.value;
}
let state=restoreSharedAxisReplay(saved,anatomy.field),rotations=saved.stepRequest.rotations;
let tools=saved.tools.map(t=>({...t,rotation:rotations[t.id],shaftStiffness:t.id==='wire'?13.15:40.65,tipStiffness:t.id==='wire'?19.4:100}));
const report={anatomy:'infrarenal-aneurysm',dt,secondsPerRun:duration,comparisonSamplingHz:4,options,preparation:[],cases:[],limitations:['Reduced-order, one-way blood drag; no 3-D aneurysm recirculation or calibrated device forces.','CPU timings are Node replay timings, not browser FPS; the no-flow case also solves every step (no provider sleeping).']};
function prepare(name,targets){
    for(let step=0;step<700;step++){
        let moving=false;
        tools=tools.map(t=>{const delta=targets[t.id]-t.insertion,travel=Math.sign(delta)*Math.min(Math.abs(delta),(t.id==='wire'?44:52)*dt);moving||=Math.abs(delta)>1e-8;return {...t,insertion:t.insertion+travel};});
        const next=solve(state,rotations,tools);
        if(!next.state)throw new Error(`Preparation ${name} ${step}: ${JSON.stringify(next.result)}`);
        state=next.state;rotations=next.rotations;
        if(!moving)break;
    }
    for(let i=0;i<480;i++){const next=solve(state,rotations,tools);if(!next.state)throw new Error(`Settle ${name}: ${next.result.status}`);state=next.state;rotations=next.rotations;}
    const checkpoint=captureSharedAxisReplay(state,sheath);
    const positions=Object.fromEntries(tools.map(t=>[t.id,sampleSharedAxisPosition(state,t.insertion)]));
    report.preparation.push({name,tools,tipWorldMm:positions});console.log('Prepared',name,JSON.stringify(positions));
    return checkpoint;
}
function run(checkpoint,name,enabled,output=5000,heartRate=72){
    let s=restoreSharedAxisReplay(checkpoint,anatomy.field),r={...rotations};
    const flow=new ContrastFlowNetwork(segments,{cardiacOutputMlPerMin:output,heartRateBpm:heartRate});
    const actualTools=checkpoint.tools.map(t=>({...t,rotation:r[t.id]}));
    const samples=actualTools.map(t=>Array.from({length:Math.ceil(t.insertion/5)+1},(_,i)=>Math.min(i*5,t.insertion)));
    const initial=samples.map(xs=>xs.map(x=>sampleSharedAxisPosition(s,x)));
    const result={name,enabled,cardiacOutputMlPerMin:output,heartRateBpm:heartRate,steps:0,terminalFailure:null,recoveredSteps:0,subdivisionsMax:1,maxPenetrationMm:0,maxRelativeLengthError:0,maxToolSpeedMmS:0,maxLateToolSpeedMmS:0,maxStepDisplacementMm:0,maxLateStepDisplacementMm:0,maxFlowSpeedMmS:0,speedBySecondMmS:[],finite:true,tipDisplacementMm:{},cpuMs:[],trajectory:[]};
    for(let step=0;step<steps;step++){
        const old=s.positions.map(p=>p.slice());const start=performance.now();
        const blood=enabled?captureBloodFlow(s,flow,sheath.length):null;
        const next=solve(s,r,actualTools,blood);result.cpuMs.push(performance.now()-start);
        if(!next.state){result.terminalFailure={step,status:next.result.status,error:next.result.error,attempts:next.result.attempts};break;}
        s=next.state;r=next.rotations;flow.update(dt);result.steps++;
        const q=next.result.quality,speed=Math.max(...q.bodies.map(b=>b.maxSpeed)),movement=Math.max(...s.positions.map((p,i)=>dist(p,old[i])));
        result.recoveredSteps+=Number(next.result.attempts.some(a=>!a.converged));
        result.subdivisionsMax=Math.max(result.subdivisionsMax,next.result.subdivisions);
        result.maxPenetrationMm=Math.max(result.maxPenetrationMm,q.maxPenetration);
        result.maxRelativeLengthError=Math.max(result.maxRelativeLengthError,...q.bodies.map(b=>b.maxLengthError));
        result.maxToolSpeedMmS=Math.max(result.maxToolSpeedMmS,speed);result.maxStepDisplacementMm=Math.max(result.maxStepDisplacementMm,movement);
        result.finite&&=q.finite&&s.positions.every(p=>p.every(Number.isFinite));
        const second=Math.floor(step*dt);result.speedBySecondMmS[second]=Math.max(result.speedBySecondMmS[second]??0,speed);
        if(step>=steps/2){result.maxLateToolSpeedMmS=Math.max(result.maxLateToolSpeedMmS,speed);result.maxLateStepDisplacementMm=Math.max(result.maxLateStepDisplacementMm,movement);}
        result.maxFlowSpeedMmS=Math.max(result.maxFlowSpeedMmS,s.bloodFlowDiagnostics?.maxSpeed??0);
        if(step%15===14)result.trajectory.push(samples.map(xs=>xs.map(x=>sampleSharedAxisPosition(s,x))));
    }
    for(const [i,t] of actualTools.entries())result.tipDisplacementMm[t.id]=dist(sampleSharedAxisPosition(s,t.insertion),initial[i].at(-1));
    const sorted=result.cpuMs.slice().sort((a,b)=>a-b);result.meanStepCpuMs=result.cpuMs.reduce((a,b)=>a+b,0)/sorted.length;result.p95StepCpuMs=sorted[Math.floor(sorted.length*.95)];delete result.cpuMs;
    report.cases.push(result);console.log('Finished',name,enabled,JSON.stringify({...result,trajectory:undefined}));return result;
}
function compare(a,b){
    b.maxDifferenceFromNoFlowMm={};
    for(let j=0;j<2;j++){let max=0;for(let i=0;i<Math.min(a.trajectory.length,b.trajectory.length);i++)for(let n=0;n<a.trajectory[i][j].length;n++)max=Math.max(max,dist(a.trajectory[i][j][n],b.trajectory[i][j][n]));b.maxDifferenceFromNoFlowMm[j===0?'wire':'catheter']=max;}
}
try{
    for(const [name,targets] of [['wire-in-sac',{wire:270,catheter:0}],['berenstein-in-sac',{wire:245,catheter:270}],['pigtail-in-sac',{wire:245,catheter:270}]]){
        if(name==='pigtail-in-sac'){prepare('retract-for-pigtail',{wire:270,catheter:0});tools=tools.map(t=>t.id==='catheter'?{...t,type:'pigtail'}:t);}
        const checkpoint=prepare(name,targets),baseline=run(checkpoint,name,false),normal=run(checkpoint,name,true),stress=run(checkpoint,name,true,9000,140);
        compare(baseline,normal);compare(baseline,stress);
    }
}finally{
    report.completed=report.cases.length===9&&report.cases.every(c=>c.steps===steps);
    report.checks={noTerminalRejections:report.completed&&report.cases.every(c=>!c.terminalFailure),
        finite:report.completed&&report.cases.every(c=>c.finite),
        wallPenetrationBelow001mm:report.completed&&report.cases.every(c=>c.maxPenetrationMm<.01),
        relativeLengthErrorBelow1e4:report.completed&&report.cases.every(c=>c.maxRelativeLengthError<1e-4)};
    for(const c of report.cases)delete c.trajectory;
    fs.writeFileSync(process.argv[2]??'reports/blood-flow-aneurysm-stability-2026-10-11.json',JSON.stringify(report,null,2)+'\n');
    if(!Object.values(report.checks).every(Boolean))process.exitCode=1;
    anatomy.dispose();
}
