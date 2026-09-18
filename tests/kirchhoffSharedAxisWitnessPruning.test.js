import test from 'node:test';
import assert from 'node:assert/strict';
import {prunableSharedAxisWitnesses} from '../src/physics/kirchhoffSharedAxisWitnessPruning.js';
import {createSharedAxisNative,feedSharedAxisNative,extendSharedAxisNativeRows} from '../src/physics/kirchhoffSharedAxisNative.js';

function fixture() {
    const sample=Object.assign(()=>({gap:1,jacobian:[0,0,0,0,0,0]}),{sharedAxisCompleteDiscovery:true});
    const s=createSharedAxisNative({startCoordinate:-10,tools:[{id:'wire',insertion:40}],wallSamples:[sample]});
    const dofs=[s.layout.positions[3],s.layout.positions[4]].flatMap(i=>[i,i+1,i+2]);
    const rows=Array.from({length:7},(_,i)=>({kind:'wall',id:`vessel/test/${i}`,edge:3,dofs,
        witness:{t:.5,face:i},evaluate:()=>({gap:.1*i,jacobian:[0,0,.5,0,0,.5]})}));
    extendSharedAxisNativeRows(s,rows);
    s.acceptedWallGaps=new Map(rows.map((r,i)=>[r.id,i===1?1e-12:.1*i]));
    return {s,rows};
}

test('pruning keeps nearest/tied faces, loaded rows, friction memory and missing measurements',()=>{
    const {s,rows}=fixture();s.multipliers[s.definitions.indexOf(rows[3])]=2;
    s.wallFrictionHistory=[{id:`wire/${rows[4].id}`,elastic:[1,0,0]}];
    s.acceptedWallGaps.delete(rows[5].id);s.acceptedWallGaps.set(rows[6].id,NaN);
    assert.deepEqual([...prunableSharedAxisWitnesses(s)],[rows[2]]);
});

test('a different physical site or owner cannot dominate a witness; incomplete discovery disables pruning',()=>{
    const {s,rows}=fixture();rows[2].witness={t:.6,face:2};rows[3].witness={t:.5,face:3,owner:'catheter'};
    const drops=prunableSharedAxisWitnesses(s);assert.ok(!drops.has(rows[2]));assert.ok(!drops.has(rows[3]));
    s.wallSamples=[()=>({gap:1,jacobian:[0,0,0,0,0,0]})];assert.equal(prunableSharedAxisWitnesses(s).size,0);
});

test('pruning is confined to the private feed candidate; reactions and friction history survive',()=>{
    const {s,rows}=fixture();s.multipliers[s.definitions.indexOf(rows[3])]=2;
    s.wallFrictionHistory=[{id:`wire/${rows[4].id}`,elastic:[1,0,0]}];
    const definitions=s.definitions.slice(),multipliers=s.multipliers.slice(),positions=structuredClone(s.positions);
    const reference=feedSharedAxisNative(s,{wire:40});
    const next=feedSharedAxisNative(s,{wire:40},{pruneInactiveWitnesses:true});
    assert.ok(next.definitions.length<reference.definitions.length);
    for(const i of [0,1,3,4])assert.ok(next.definitions.some(r=>r.id===rows[i].id));
    assert.equal(next.multipliers[next.definitions.findIndex(r=>r.id===rows[3].id)],2);
    assert.equal(next.wallFrictionHistory,s.wallFrictionHistory);
    assert.deepEqual(s.definitions,definitions);assert.deepEqual(s.multipliers,multipliers);assert.deepEqual(s.positions,positions);
    assert.deepEqual(next.coordinates,reference.coordinates);
});
