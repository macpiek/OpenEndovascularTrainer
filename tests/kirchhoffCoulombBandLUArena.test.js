import test from 'node:test';
import assert from 'node:assert/strict';
import {createCoulombBandLU,createCoulombBandLUArena} from '../src/physics/kirchhoffCoulombBandLU.js';

function system(count,kl,ku,seed=1) {
    const starts=Int32Array.from({length:count},(_,i)=>Math.max(0,i-kl));
    const ends=Int32Array.from({length:count},(_,i)=>Math.min(count-1,i+ku));
    let entries=0;
    const offsets=Int32Array.from(starts,(start,i)=>{const offset=entries-start;entries+=ends[i]-start+1;return offset;});
    const layout={starts,ends,offsets,entries,kl,ku},J=new Float64Array(entries);
    for(let i=0;i<count;i++)for(let j=starts[i];j<=ends[i];j++)
        J[offsets[i]+j]=i===j?2+kl+ku:Math.sin(seed+i*7+j*11)*.4;
    return {layout,count,J,F:Float64Array.from({length:count},(_,i)=>Math.cos(i+seed)),
        scales:Float64Array.from({length:count},(_,i)=>.5+(i%7)/7)};
}
function compare(s,shared,owned,shift=0) {
    const actual=new Float64Array(s.count).fill(99),expected=actual.slice();
    const accepted=shared.solve(s.J,s.F,s.scales,shift,actual),reference=owned.solve(s.J,s.F,s.scales,shift,expected);
    assert.equal(accepted,reference);assert.deepEqual(actual,expected);
    assert.deepEqual(shared.diagnostics,owned.diagnostics);
    return accepted;
}

test('arena is lazy and many differently shaped LU objects share one current power-of-two allocation',()=>{
    const arena=createCoulombBandLUArena();
    const fixtures=Array.from({length:48},(_,i)=>system(i+1,i%4,i%5,i+1));
    const solvers=fixtures.map(s=>createCoulombBandLU(s.layout,s.count,{arena}));
    assert.deepEqual({...arena.diagnostics},{allocations:0,capacityBytes:0,grows:0,viewBuilds:0,requests:0});
    fixtures.forEach((s,i)=>assert.equal(compare(s,solvers[i],createCoulombBandLU(s.layout,s.count)),true));
    assert.equal(arena.diagnostics.allocations,1);assert.equal(arena.diagnostics.capacityBytes,65536);
    assert.equal(arena.diagnostics.requests,48);assert.equal(arena.diagnostics.viewBuilds,48);
    assert.equal(arena.diagnostics.grows,0);
    assert.throws(()=>{arena.diagnostics.allocations=0;},TypeError);
});

test('interleaved shapes preserve bitwise results and independent diagnostics before and after growth',()=>{
    const arena=createCoulombBandLUArena();
    const fixtures=[[0,0,0],[1,0,0],[4,1,1],[23,3,2],[51,7,4],[19,0,3],[17,4,0],[600,30,20],[5,1,2],[800,45,18]]
        .map(([n,kl,ku],i)=>system(n,kl,ku,i+1));
    const shared=fixtures.map(s=>createCoulombBandLU(s.layout,s.count,{arena}));
    const owned=fixtures.map(s=>createCoulombBandLU(s.layout,s.count));
    let calls=0;
    for(const order of [fixtures.map((_,i)=>i),fixtures.map((_,i)=>fixtures.length-1-i),[0,7,2,9,1,8,3,6,4,5]]) {
        for(const i of order) { assert.equal(compare(fixtures[i],shared[i],owned[i],calls%3===0?.01:0),true);calls++; }
    }
    assert.equal(arena.diagnostics.requests,calls);assert.equal(arena.diagnostics.allocations,3);
    assert.equal(arena.diagnostics.grows,2);assert.equal(arena.diagnostics.capacityBytes,1048576);
});

test('growth invalidates cached shape views and old LU instances reacquire the new kernel buffer',()=>{
    const arena=createCoulombBandLUArena(),first=arena.getViews(4,4);
    assert.equal(arena.getViews(4,4),first);assert.equal(first.factor.buffer,first.rhs.buffer);assert.equal(first.rhs.buffer,first.right.buffer);
    const small=system(4,1,1),oldLU=createCoulombBandLU(small.layout,small.count,{arena}),reference=createCoulombBandLU(small.layout,small.count);
    assert.equal(compare(small,oldLU,reference),true);
    const large=arena.getViews(1000,256),again=arena.getViews(4,4);
    assert.notEqual(again,first);assert.notEqual(again.factor.buffer,first.factor.buffer);
    assert.equal(again.factor.buffer,large.factor.buffer);
    assert.equal(arena.getViews(4,4),again);
    assert.equal(compare(small,oldLU,reference),true,'An old solver may not retain an obsolete arena view');
    const bytes=8*1000*256+16*1000+128;
    assert.ok(arena.diagnostics.capacityBytes>=bytes&&arena.diagnostics.capacityBytes<2*bytes);
    assert.ok(Number.isInteger(Math.log2(arena.diagnostics.capacityBytes)));
    assert.equal(arena.diagnostics.allocations,2);
});

test('pivoting and failed solves do not leak factor or RHS state between shared owners',()=>{
    const arena=createCoulombBandLUArena(),pivot=system(2,1,1),singular=system(2,1,1),other=system(7,2,3);
    pivot.J.set([.001,2,3,4]);singular.J.set([0,1,0,2]);
    const systems=[pivot,singular,other],shared=systems.map(s=>createCoulombBandLU(s.layout,s.count,{arena})),owned=systems.map(s=>createCoulombBandLU(s.layout,s.count));
    for(const i of [0,1,2,1,0,2,0])assert.equal(compare(systems[i],shared[i],owned[i]),i!==1);
    assert.ok(shared[0].diagnostics.rowSwaps>0);assert.equal(shared[1].diagnostics.rowSwaps,0);
    assert.equal(arena.diagnostics.allocations,1);
});

test('invalid arena shapes cannot allocate or corrupt an already usable arena',()=>{
    const arena=createCoulombBandLUArena(),views=arena.getViews(4,4),before={...arena.diagnostics};
    for(const shape of [[-1,4],[1,0],[1,1.5],[NaN,1],[1,Infinity],[Number.MAX_SAFE_INTEGER,4]])
        assert.throws(()=>arena.getViews(...shape),/arena shape/);
    assert.deepEqual({...arena.diagnostics},before);assert.equal(arena.getViews(4,4),views);
});

test('WASM packing and certification preserve bitwise solutions and diagnostics across shared arena growth',()=>{
    const arena=createCoulombBandLUArena();
    const fixtures=[[0,0,0],[1,0,0],[4,1,2],[37,5,3],[600,30,20],[13,2,0],[800,45,18]]
        .map(([n,kl,ku],i)=>system(n,kl,ku,i+1));
    const fast=fixtures.map(s=>createCoulombBandLU(s.layout,s.count,{arena,wasmAssembly:true}));
    const reference=fixtures.map(s=>createCoulombBandLU(s.layout,s.count));
    for(const order of [fixtures.map((_,i)=>i),fixtures.map((_,i)=>fixtures.length-1-i)]) {
        for(const i of order) {
            const before=structuredClone(fixtures[i]);
            assert.equal(compare(fixtures[i],fast[i],reference[i],i%2?.01:0),true);
            assert.deepEqual(fixtures[i],before);
        }
    }
    assert.ok(arena.diagnostics.grows>0);
});

test('WASM certificate retains rejection, pivot diagnostics, signed zero and independent owned storage',()=>{
    const arena=createCoulombBandLUArena();
    for(const values of [[.001,2,3,4],[0,1,0,2],[1,-0,0,1],[1e-200,2e-200,3e-200,4e-200]]) {
        const s=system(2,1,1);s.J.set(values);
        for(const shared of [false,true]) {
            const fast=createCoulombBandLU(s.layout,s.count,{wasmAssembly:true,...(shared?{arena}:{})});
            const reference=createCoulombBandLU(s.layout,s.count);
            compare(s,fast,reference);s.F[0]*=2;compare(s,fast,reference,.0001);
        }
    }
});

test('WASM unscaled certificate uses the original RHS after a refinement solve',()=>{
    const s=system(29,4,3),lu=createCoulombBandLU(s.layout,s.count,{arena:createCoulombBandLUArena(),wasmAssembly:true});
    const solution=new Float64Array(s.count),errors=new Float64Array(s.count);
    assert.ok(lu.solve(s.J,s.F,s.scales,0,solution));
    for(let i=0;i<s.count;i++)solution[i]*=s.scales[i];
    const expected=Float64Array.from(s.F);
    for(let i=0;i<s.count;i++)for(let j=s.layout.starts[i];j<=s.layout.ends[i];j++)
        expected[i]+=s.J[s.layout.offsets[i]+j]*solution[j];
    // A refinement overwrites the arena's RHS, but not the original equation.
    assert.ok(lu.solve(s.J,expected,s.scales,0,new Float64Array(s.count)));
    const residual=lu.measureOriginalResidual(solution,errors,s.F);
    assert.deepEqual(errors,expected);
    assert.equal(residual,Math.max(...expected.map(Math.abs)));
});

test('WASM row equilibration preserves the reference scales, direction and backward error',()=>{
    const s=system(31,5,3),expectedScales=new Float64Array(s.count);
    for(let i=0;i<s.count;i++) {
        let maximum=0;
        for(let j=s.layout.starts[i];j<=s.layout.ends[i];j++)maximum=Math.max(maximum,Math.abs(s.J[s.layout.offsets[i]+j]));
        expectedScales[i]=1/Math.sqrt(Math.max(maximum,1e-30));
    }
    const reference=createCoulombBandLU(s.layout,s.count),fast=createCoulombBandLU(s.layout,s.count,{wasmAssembly:true});
    const expected=new Float64Array(s.count),actual=expected.slice(),scales=new Float64Array(s.count).fill(NaN);
    assert.equal(reference.solve(s.J,s.F,expectedScales,.002,expected),true);
    assert.equal(fast.solve(s.J,s.F,scales,.002,actual,true),true);
    assert.deepEqual(scales,expectedScales);assert.deepEqual(actual,expected);
    assert.deepEqual(fast.diagnostics,reference.diagnostics);
});
