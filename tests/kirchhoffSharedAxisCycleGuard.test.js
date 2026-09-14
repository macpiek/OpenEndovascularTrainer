import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisCycleGuard} from '../src/physics/kirchhoffSharedAxisCycleGuard.js';

function sample(phase=0) {
    const s={positions:[[phase,0,0]],materials:[{body:{orientationX:[0],orientationY:[0],orientationZ:[0],orientationW:[1]}}]};
    const base={force:10+phase,torque:2,constraint:1e-6,energy:100,rows:[{kind:'wall',multiplier:phase+1}]};
    return [s,base];
}

for(const period of [1,2,3,4])test(`repeated period ${period} requires multiple complete comparisons`,()=>{
    const guard=createSharedAxisCycleGuard();
    const required=Math.max(3,2*period);
    for(let i=0;i<period+required-1;i++)assert.equal(guard.observe(...sample(i%period)),null);
    assert.deepEqual(guard.observe(...sample((period+required-1)%period)),{period,repeatedComparisons:required});
});

test('equal contact counts do not identify a cycle while pose, frames, residuals or active rows change',()=>{
    const changes=[
        (s,b,i)=>s.positions[0][1]=i*1e-5,
        (s,b,i)=>s.materials[0].body.orientationZ[0]=i*1e-6,
        (s,b,i)=>b.force=10/(i+1),
        (s,b,i)=>b.torque=2/(i+1),
        (s,b,i)=>b.constraint=1e-6/(i+1),
        (s,b,i)=>b.energy=100-i*.01,
        (s,b,i)=>{b.rows=Array.from({length:32},(_,j)=>({kind:'wall',multiplier:j===i?1:0}));},
    ];
    for(const change of changes){
        const guard=createSharedAxisCycleGuard();
        for(let i=0;i<32;i++){const [s,b]=sample();change(s,b,i);assert.equal(guard.observe(s,b),null);}
    }
});

test('new row topology clears repetition history',()=>{
    const guard=createSharedAxisCycleGuard(),[s,b]=sample();
    for(let i=0;i<3;i++)assert.equal(guard.observe(s,b),null);
    b.rows.push({kind:'wall',multiplier:1});
    for(let i=0;i<3;i++)assert.equal(guard.observe(s,b),null);
    assert.equal(guard.observe(s,b).period,1);
});

test('one recurrence or a nonfinite state is insufficient',()=>{
    const guard=createSharedAxisCycleGuard();
    for(const phase of [0,1,0,3,4,5,6,7])assert.equal(guard.observe(...sample(phase)),null);
    for(const invalid of [NaN,Infinity,-Infinity]) {
        const g=createSharedAxisCycleGuard(),[s,b]=sample();b.force=invalid;
        for(let i=0;i<16;i++)assert.equal(g.observe(s,b),null);
    }
});
