import test from 'node:test';import assert from 'node:assert/strict';
import {fullCyclePhases,fullCycleDurationMs,sampleFullCycleBenchmark,summarizeFullCycle} from '../src/benchmark/fullCycleBenchmark.js';
for(const dt of [1/60,1/120])test(`complete 100 cm cycle respects runtime feed rates and returns both tools in order at dt=${dt}`,()=>{
    const phases=fullCyclePhases(dt);let wire=0,catheter=0;
    for(const phase of phases){
        for(let i=0;i<phase.steps;i++){
            const cmd=sampleFullCycleBenchmark((phase.start+i)*dt*1000,dt,{},'pigtail');
            assert.equal(cmd.catheterType,'pigtail');assert.ok(Math.abs(cmd.guidewireAdvance)<=1&&Math.abs(cmd.catheterAdvance)<=1);
            if(phase.tool==='wire')assert.equal(cmd.catheterAdvance,0);else assert.equal(cmd.guidewireAdvance,0);
            wire+=cmd.guidewireAdvance*44*dt;catheter+=cmd.catheterAdvance*(cmd.catheterAdvance>0?52:32)*dt;
            assert.ok(wire>=-1e-7&&wire<=1000+1e-7&&catheter>=-1e-7&&catheter<=1000+1e-7);
        }
        if(phase.name==='wire-in')assert.ok(Math.abs(wire-1000)<1e-7&&catheter===0);
        if(phase.name==='catheter-in')assert.ok(Math.abs(wire-1000)<1e-7&&Math.abs(catheter-1000)<1e-7);
        if(phase.name==='catheter-out')assert.ok(Math.abs(wire-1000)<1e-7&&Math.abs(catheter)<1e-7);
    }
    assert.ok(Math.abs(wire)<1e-7&&Math.abs(catheter)<1e-7);
    const end=sampleFullCycleBenchmark(fullCycleDurationMs(dt),dt,{});assert.equal(end.guidewireAdvance,0);assert.equal(end.catheterAdvance,0);
});
test('physics throughput exposes delayed accepted steps despite cheap CPU slices, and incomplete cycles cannot pass',()=>{
    const samples=Array.from({length:181},(_,i)=>({accepted:true,cpuMs:2,wallMs:i*1000/60,wire:i===0?1000:0,catheter:i===60?1000:0}));
    const a=summarizeFullCycle(samples,{completed:true});assert.equal(a.completedCycle,true);assert.ok(Math.abs(a.minimumOneSecondPhysicsHz-60)<1e-10);
    for(let i=90;i<samples.length;i++)samples[i].wallMs+=500;
    const b=summarizeFullCycle(samples,{completed:true});assert.ok(Math.abs(b.minimumOneSecondPhysicsHz-40)<1e-10);assert.equal(b.maxStepMs,2);assert.ok(b.maxAcceptedIntervalMs>500);
    assert.equal(summarizeFullCycle(samples).completedCycle,false);
    assert.equal(summarizeFullCycle([...samples,{accepted:false}],{completed:true}).completedCycle,false);
    assert.equal(summarizeFullCycle(samples.filter(s=>s.catheter!==1000),{completed:true}).completedCycle,false);
});
