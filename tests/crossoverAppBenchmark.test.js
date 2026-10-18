import test from 'node:test';
import assert from 'node:assert/strict';
import {crossoverContinuationCommands,crossoverPhysicalGate} from '../src/benchmark/crossoverAppBenchmark.js';

test('full-app continuation reaches exactly the paired benchmark targets using real feed rates',()=>{
    const dt=1/60,tools=[{id:'wire',insertion:981.2},{id:'catheter',insertion:860.6666666667}];
    const current=Object.fromEntries(tools.map(t=>[t.id,t.insertion]));
    for(let step=0;step<600;step++) {
        const c=crossoverContinuationCommands(step,dt,tools,current);
        current.wire+=c.guidewireAdvance*44*dt;
        current.catheter+=c.catheterAdvance*(c.catheterAdvance>=0?52:32)*dt;
        const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
        for(const t of tools)assert.ok(Math.abs(current[t.id]-t.insertion-offset)<1e-10);
        assert.equal(c.catheterType,'pigtail');assert.equal(c.catheterRotation,0);
    }
});

test('browser acceptance rejects missing, nonfinite and out-of-tolerance diagnostics',()=>{
    const valid={converged:true,certificateBound:1e-7,residual:{length:1e-8},
        quality:{finite:true,maxPenetration:0,bodies:[{maxLengthError:1e-8},{maxLengthError:1e-8}]}};
    assert.equal(crossoverPhysicalGate(1/60,valid),true);
    assert.equal(crossoverPhysicalGate(1/30,valid),false);
    for(const path of [['certificateBound'],['residual','length'],['quality','maxPenetration'],['quality','bodies',1,'maxLengthError']])
        for(const value of [undefined,NaN,Infinity,-1,1]){
            const result=structuredClone(valid),parent=path.slice(0,-1).reduce((object,key)=>object[key],result);
            parent[path.at(-1)]=value;
            assert.equal(crossoverPhysicalGate(1/60,result),false,`${path.join('.')}: ${value}`);
        }
    assert.equal(crossoverPhysicalGate(1/60,{...valid,converged:false}),false);
    assert.equal(crossoverPhysicalGate(1/60,{...valid,quality:{...valid.quality,bodies:[]}}),false);
    assert.equal(crossoverPhysicalGate(1/60,null),false);
});
