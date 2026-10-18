import test from 'node:test';
import assert from 'node:assert/strict';
import {createBandRefinement} from '../src/physics/kirchhoffBandRefinement.js';
import {createCoulombBandLU,createCoulombBandLUArena} from '../src/physics/kirchhoffCoulombBandLU.js';

function fixture(n,kl,ku,seed=1) {
    const starts=Int32Array.from({length:n},(_,i)=>Math.max(0,i-kl));
    const ends=Int32Array.from({length:n},(_,i)=>Math.min(n-1,i+ku));
    let entries=0;
    const offsets=Int32Array.from(starts,(first,i)=>{const offset=entries-first;entries+=ends[i]-first+1;return offset;});
    const layout={starts,ends,offsets,kl,ku,entries},matrix=new Float64Array(entries);
    for(let i=0;i<n;i++)for(let j=starts[i];j<=ends[i];j++)
        matrix[offsets[i]+j]=i===j?.17:Math.sin(seed+i*7+j*11)+.1;
    return {n,layout,matrix,scales:Float64Array.from({length:n},(_,i)=>.4+(i%7)/7)};
}
function compare(f,lease,reference,seed) {
    const rhs=Float64Array.from({length:f.n},(_,i)=>Math.cos(seed+i));
    const expected=new Float64Array(f.n).fill(123),actual=expected.slice();
    const accepted=reference.solve(f.matrix,rhs,f.scales,0,expected);
    assert.equal(lease.solve(rhs,actual),accepted);assert.deepEqual(actual,expected);
    return accepted;
}

test('refinement reuses pivoted nonsymmetric factors with bitwise identical RHS solutions',()=>{
    for(const shape of [[0,0,0],[1,0,0],[9,0,3],[9,3,0],[19,3,2],[37,7,4],[80,13,10]]) {
        const f=fixture(...shape),lease=createBandRefinement(f.layout,f.n,f.matrix,f.scales);
        const reference=createCoulombBandLU(f.layout,f.n,{wasmAssembly:true});
        try {
            for(let rhs=0;rhs<12;rhs++)assert.ok(compare(f,lease,reference,rhs));
            assert.equal(lease.diagnostics.factorizations,1);assert.equal(lease.diagnostics.backsolves,11);
            if(f.layout.kl&&f.layout.ku)assert.ok(reference.diagnostics.rowSwaps>0);
        } finally {lease.dispose();}
    }
});

test('ordinary LU arena work and original-equation measurement cannot overwrite refinement factors',()=>{
    const f=fixture(31,5,4),lease=createBandRefinement(f.layout,f.n,f.matrix,f.scales);
    const reference=createCoulombBandLU(f.layout,f.n,{arena:createCoulombBandLUArena(),wasmAssembly:true});
    try {
        assert.ok(compare(f,lease,reference,1));
        const rhs=new Float64Array(f.n),solution=rhs.slice(),error=rhs.slice();
        reference.solve(f.matrix,rhs,f.scales,0,solution);
        reference.measureOriginalResidual(solution,error,rhs);
        assert.ok(compare(f,lease,reference,2));assert.equal(lease.diagnostics.factorizations,1);
    } finally {lease.dispose();}
});

test('leases snapshot coefficients, scaling and row metadata before callers reuse their scratch',()=>{
    const f=fixture(23,4,3),saved=structuredClone(f),lease=createBandRefinement(f.layout,f.n,f.matrix,f.scales);
    f.matrix.fill(0);f.scales.fill(NaN);f.layout.starts.fill(0);f.layout.ends.fill(0);f.layout.offsets.fill(0);
    const reference=createCoulombBandLU(saved.layout,saved.n);
    try {for(let i=0;i<3;i++)assert.ok(compare(saved,lease,reference,i));}
    finally {lease.dispose();}
});

test('shape changes, arena growth and disposal invalidate old leases without corrupting the new owner',()=>{
    const small=fixture(5,1,2),large=fixture(700,40,27);
    let old=createBandRefinement(small.layout,small.n,small.matrix,small.scales);
    for(const f of [small,large,small]) {
        const lease=createBandRefinement(f.layout,f.n,f.matrix,f.scales);
        assert.throws(()=>old.solve(new Float64Array(small.n),new Float64Array(small.n)),/Expired/);
        old.dispose();
        assert.ok(compare(f,lease,createCoulombBandLU(f.layout,f.n),1));old=lease;
    }
    old.dispose();assert.throws(()=>old.solve(new Float64Array(small.n),new Float64Array(small.n)),/Expired/);
});

test('singular, signed-zero and extreme-scale systems preserve the ordinary acceptance and output contract',()=>{
    for(const values of [[0,1,0,2],[1,-0,0,1],[.001,2,3,4],[1e-200,2e-200,3e-200,4e-200]]) {
        const f=fixture(2,1,1);f.matrix.set(values);
        const lease=createBandRefinement(f.layout,f.n,f.matrix,f.scales),reference=createCoulombBandLU(f.layout,f.n);
        try {for(let i=0;i<3;i++)compare(f,lease,reference,i);}
        finally {lease.dispose();}
    }
});
