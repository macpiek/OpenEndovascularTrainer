import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync(new URL('../src/simulator.js',import.meta.url),'utf8');
function pulseHarness(rate=15) {
    const state={displayMaterial:{uniforms:{pulseRate:{value:rate}}},completedSimulationTime:0,
        lastFluoroPulseTime:-Infinity,lastFluoroViewRevision:-1,revision:0};
    state.ui={getCArmRevision:()=>state.revision};
    vm.createContext(state);
    const body=source.slice(source.indexOf('        const pulseRate = Math.max'),source.indexOf('        if (!shouldAcquireFluoroFrame)'));
    vm.runInContext('function acquire(){'+body+'; if(shouldAcquireFluoroFrame){lastFluoroPulseTime=completedSimulationTime*1000;lastFluoroViewRevision=revision;}return shouldAcquireFluoroFrame;}',state);
    return state;
}

test('60 Hz accumulated simulation time produces the requested 15 Hz pulses without floating-point skips',()=>{
    const s=pulseHarness();let count=0;
    for(let tick=0;tick<3600;tick++){if(s.acquire())count++;s.completedSimulationTime+=1/60;}
    assert.equal(count,900);
});

test('moving the C-arm redraws the projection even when physics is stalled',()=>{
    const s=pulseHarness();assert.equal(s.acquire(),true);assert.equal(s.acquire(),false);
    s.revision++;assert.equal(s.acquire(),true);
    assert.equal(s.completedSimulationTime,0);assert.equal(s.acquire(),false);
});
