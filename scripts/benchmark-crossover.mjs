// CPU benchmark of the recorded crossover. Timings exclude anatomy loading.
// `checkpoint` restores the exact long-tool step; `continuation` exercises it
// with fresh pulsatile flow. `reconstruct` uses explicit estimated intermediate
// commands: the demonstration captured endpoints, not a keyboard time series.
import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const mode=process.argv[2]??'checkpoint';
if(!['checkpoint','continuation','reconstruct'].includes(mode))throw new Error('Mode: checkpoint | continuation | reconstruct');
const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const output=process.argv[3]??new URL(`001-${mode}-result.json`,directory);
const source=fs.readFileSync(new URL('001-stage-05-replay.json.gz',directory));
const fixture=JSON.parse(gunzipSync(source)),request=fixture.stepRequest,dt=request.dt;
const scenario=JSON.parse(fs.readFileSync(new URL('001-demonstration.json',directory)));
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
const report={mode,recordedAt:new Date().toISOString(),anatomy:fixture.anatomy,node:process.version,fixtureSha256:createHash('sha256').update(source).digest('hex'),dt,
    sourceRevision:scenario.sourceRevision,rendering:false,runs:[],passed:false,
    limitations:['Node CPU timings are not browser FPS.','Only the captured stage-05 step is an exact input replay; earlier stages have recorded endpoints and descriptions.']};
function solve(state,rotations,tools,options) {
    const iterator=advanceSharedAxis(state,rotations,dt,tools,options),start=performance.now();
    let next;do{next=iterator.next();}while(!next.done);
    return {...next.value,cpuMs:performance.now()-start};
}
function sample(next) {
    const r=next.result;
    return {cpuMs:next.cpuMs,status:r.status,iterations:r.iterations,factorizations:r.factorizations,
        subdivisions:r.subdivisions,certificateBound:r.certificateBound,residual:r.residual,quality:r.quality,attempts:r.attempts};
}
function worldPoints(state) {
    return state.positions.map(p=>p.map((v,k)=>v+state.origin[k]));
}
function maxDistance(a,b) {
    if(a.length!==b.length)throw new Error('Replay topology differs');
    return Math.max(...a.map((p,i)=>Math.hypot(...p.map((v,k)=>v-b[i][k]))));
}
function check(next) {
    const r=next.result;
    return !!next.state&&r.converged&&r.quality.finite&&r.certificateBound<=1e-6&&
        r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5&&r.quality.bodies.every(b=>b.maxLengthError<1e-5);
}
try {
    if(mode==='checkpoint') {
        const outputs=[];
        for(let repeat=0;repeat<4;repeat++)for(const enabled of repeat%2?[true,false]:[false,true]) {
            const input=restoreSharedAxisReplay(fixture,anatomy.field);
            const next=solve(input,request.rotations,request.tools,{...request.options,roundoffStagnation:enabled});
            report.runs.push({enabled,repeat,warmup:repeat===0,...sample(next)});
            if(!check(next))throw new Error(`Uncertified ${enabled}: ${next.result.status}`);
            outputs.push({enabled,points:worldPoints(next.state)});
            console.log(JSON.stringify({repeat,enabled,ms:next.cpuMs,iterations:next.result.iterations}));
        }
        const reference=outputs.find(r=>!r.enabled).points;
        report.maxWorldPositionDifferenceMm=Math.max(...outputs.map(r=>maxDistance(reference,r.points)));
        report.positionToleranceMm=.01;
        report.limitations.push('The optimized solve can accept a full step instead of two half steps; the time discretization and rates then differ slightly.');
        report.summary=[false,true].map(enabled=>{
            const samples=report.runs.filter(r=>r.enabled===enabled&&!r.warmup).map(r=>r.cpuMs).sort((a,b)=>a-b);
            return {enabled,medianCpuMs:samples[1],minCpuMs:samples[0],maxCpuMs:samples.at(-1)};
        });
        report.passed=report.maxWorldPositionDifferenceMm<report.positionToleranceMm;
    } else {
        const flow=createCrossoverFlow();
        let state,rotations,tools,simulationTime;
        if(mode==='continuation') {
            state=restoreSharedAxisReplay(fixture,anatomy.field);rotations={...request.rotations};tools=structuredClone(request.tools);
            simulationTime=request.options.bloodFlow.time;
        } else {
            tools=request.tools.map(t=>({...t,insertion:0,rotation:0}));rotations={wire:0,catheter:0};
            state=createSharedAxisNative({...createSharedAxisContacts({sheath:fixture.sheath,contactField:anatomy.field,
                localCoordinates:true,retainDiscoveryCertificates:true}),maxBendAngle:Math.PI/4,tools});
            simulationTime=scenario.initialObservation.displayedSimulationSeconds;
        }
        flow.time=simulationTime;
        const longWire=request.tools.find(t=>t.id==='wire').insertion,longCatheter=request.tools.find(t=>t.id==='catheter').insertion;
        const phases=mode==='continuation'?
            [{name:'hold',steps:30,wire:longWire,catheter:longCatheter},
                {name:'withdraw-both',steps:20,wire:longWire-10,catheter:longCatheter-10},
                {name:'advance-both',steps:20,wire:longWire,catheter:longCatheter},
                {name:'settle',steps:30,wire:longWire,catheter:longCatheter}]:
            [{name:'wire-to-aorta',wire:300,catheter:0},
                {name:'pigtail-over-wire',wire:300,catheter:276},
                {name:'expose-pigtail',wire:226,catheter:276},
                {name:'aim-pigtail',wire:226,catheter:276,rotation:-94,checkpoint:'checkpoint-01'},
                {name:'seat-at-bifurcation',wire:172,catheter:212,rotation:-119,checkpoint:'checkpoint-02'},
                {name:'wire-crossover',wire:269,catheter:212,checkpoint:'checkpoint-03'},
                {name:'follow-wire',wire:269,catheter:250},
                {name:'withdraw-from-internal',wire:235,catheter:250},
                {name:'aim-external',wire:235,catheter:260,rotation:-125},
                {name:'wire-to-femoral',wire:364,catheter:260},
                {name:'follow-to-iliac',wire:364,catheter:290,checkpoint:'checkpoint-04'},
                {name:'deep-insertion',wire:longWire,catheter:longCatheter,checkpoint:'checkpoint-05'}];
        report.phases=phases;report.trajectory=[];
        if(mode==='reconstruct')report.limitations.push('Initial wire overshoot, simultaneous commands and intermediate corrections are estimates; checkpoint lengths alone cannot guarantee the demonstrated branch.');
        let step=0;
        for(const phase of phases) {
            const entry={name:phase.name,steps:[],checkpoint:phase.checkpoint??null};report.runs.push(entry);
            for(let i=0;i<(phase.steps??3000);i++) {
                let moving=false;
                const targets=tools.map(t=>{
                    const difference=phase[t.id]-t.insertion,rate=t.id==='wire'?44:difference>=0?52:32;
                    const travel=Math.sign(difference)*Math.min(Math.abs(difference),rate*dt);
                    const targetRotation=t.id==='catheter'&&phase.rotation!==undefined?phase.rotation*Math.PI/180:t.rotation;
                    const delta=Math.atan2(Math.sin(targetRotation-t.rotation),Math.cos(targetRotation-t.rotation));
                    const spin=Math.sign(delta)*Math.min(Math.abs(delta),Math.PI*.9*dt);
                    moving ||= Math.abs(difference)>1e-8||Math.abs(delta)>1e-8;
                    return {...t,insertion:t.insertion+travel,rotation:t.rotation+spin};
                });
                const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...targets.map(t=>t.insertion)));
                const next=solve(state,rotations,targets,{...request.options,bloodFlow});
                entry.steps.push(sample(next));
                if(!check(next)) {
                    const failed={...captureSharedAxisReplay(state,fixture.sheath),anatomy:fixture.anatomy,
                        stepRequest:{...request,rotations,tools:targets,options:{...request.options,bloodFlow}}};
                    fs.writeFileSync(new URL(`001-${mode}-failed.json.gz`,directory),gzipSync(JSON.stringify(failed)));
                    throw new Error(`${phase.name}, step ${i}: ${next.result.status}`);
                }
                state=next.state;rotations=next.rotations;tools=targets;simulationTime+=dt;flow.update(dt);step++;
                if(mode==='continuation'||step%30===0||!moving&&!phase.steps||i===phase.steps-1)report.trajectory.push({phase:phase.name,time:simulationTime,
                    tools:tools.map(t=>({id:t.id,insertion:t.insertion,rotation:t.rotation})),
                    coordinates:state.coordinates,points:worldPoints(state)});
                if(step%100===0)console.log(JSON.stringify({phase:phase.name,step,wire:tools[0].insertion,catheter:tools[1].insertion}));
                if(!moving&&!phase.steps)break;
                if(i===2999&&!phase.steps)throw new Error(`Unfinished phase: ${phase.name}`);
            }
            entry.tipPositions=Object.fromEntries(tools.map(t=>[t.id,sampleSharedAxisPosition(state,t.insertion)]));
            const pose={...captureSharedAxisReplay(state,fixture.sheath),anatomy:fixture.anatomy,stepRequest:{...request,rotations,tools,options:{...request.options,bloodFlow:captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength)}}};
            fs.writeFileSync(new URL(`001-${mode}-${phase.name}.json.gz`,directory),gzipSync(JSON.stringify(pose)));
            console.log(JSON.stringify({phase:phase.name,steps:entry.steps.length,tips:entry.tipPositions}));
        }
        const costs=report.runs.flatMap(r=>r.steps.map(s=>s.cpuMs)).sort((a,b)=>a-b);
        report.summary={steps:costs.length,simulationSeconds:costs.length*dt,
            medianCpuMs:costs[Math.floor(costs.length*.5)],p95CpuMs:costs[Math.ceil(costs.length*.95)-1],
            maxCpuMs:costs.at(-1),totalCpuMs:costs.reduce((a,b)=>a+b,0)};
        report.passed=true;
    }
} catch(error) {report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {
    anatomy.dispose();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
}
if(!report.passed)process.exitCode=1;
