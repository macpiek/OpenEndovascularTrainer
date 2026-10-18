import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';

function fixture() {
    const epsilon=1e-7,layout={dofCount:2,band:2};
    const rows=[{kind:'wall',id:'loose',dofs:[0,1],jacobian:[.64,.768],gap:-1,multiplier:1},
        {kind:'wall',id:'tight',dofs:[0,1],jacobian:[.64+epsilon,.768-epsilon],gap:-1.0001,multiplier:1}];
    return {layout,rows,chain:{layout,hessian:Float64Array.from([1,0,1,0])},
        gradient:Float64Array.from([-(1.28+epsilon),-(1.536-epsilon)]),fixed:new Uint8Array(2)};
}
const workspace=f=>createSharedAxisLinear(f.layout,f.rows,{lazy:true});
const options={tolerance:1e-8,reuseConstraintWork:true,reuseMatrixAssembly:true,deferActiveBasis:true};
const output=r=>({converged:r.converged,failure:r.failure,residual:r.residual,
    increment:Array.from(r.increment),multiplierIncrement:Array.from(r.multiplierIncrement)});
const run=(f,extra={},w=workspace(f))=>solveSharedAxisLinear(w,f.chain,{...f,...options,...extra});

for(const wasmLinearAssembly of [false,true])for(const recoverNegativeContact of [false,true])
test(`finite rejected equations retain their exact result and pivot path across retries (WASM=${wasmLinearAssembly}, recovery=${recoverNegativeContact})`,()=>{
    const f=fixture(),before=structuredClone(f),results=[];
    for(const reuseRejectedWorkingSet of [false,true]) {
        const trace=[],r=run(f,{wasmLinearAssembly,recoverNegativeContact,reuseRejectedWorkingSet,trace});
        assert.equal(r.converged,recoverNegativeContact);
        assert.ok(recoverNegativeContact?r.residual<=options.tolerance:r.residual>options.tolerance);
        results.push({r,physical:output(r),trace});
    }
    assert.deepEqual(results[1].physical,results[0].physical);
    assert.deepEqual(results[1].trace,results[0].trace,'No change in basis repair, wall release or batch fallback');
    assert.ok(results[1].r.factorizations<results[0].r.factorizations);
    assert.ok(results[1].r.workingSetReuses>results[0].r.workingSetReuses);
    assert.deepEqual(f,before);
});

test('a cancelled failure memo cannot survive changed equations or another solver',()=>{
    const f=fixture(),before=structuredClone(f),w=workspace(f);
    const iterator=iterateSharedAxisLinear(w,f.chain,{...f,...options});
    assert.equal(iterator.next().value.attempt,0);
    assert.equal(iterator.next().value.attempt,0,'Failure is saved before retrying with the prepared basis');
    iterator.return();assert.deepEqual(f,before);
    const other=fixture();other.rows[1].jacobian[0]=.9;other.chain.hessian[0]=3;
    assert.equal(run(other).converged,true);
    f.rows[1].jacobian[0]=1;f.rows[1].gap=-1.2;
    const actual=run(f,{},w),expected=run(f,{reuseRejectedWorkingSet:false});
    assert.equal(actual.converged,true);assert.deepEqual(output(actual),output(expected));
    assert.equal(actual.factorizations,expected.factorizations);
});

test('condensation and disabled working-set caching keep their original rejection retries',()=>{
    for(const extra of [{condenseCompliantContacts:true},{reuseWorkingSet:false}]) {
        const f=fixture(),a=run(f,{...extra,reuseRejectedWorkingSet:false}),b=run(f,{...extra,reuseRejectedWorkingSet:true});
        assert.equal(a.converged,false);assert.deepEqual(output(b),output(a));
        assert.equal(b.factorizations,a.factorizations);
        assert.equal(b.workingSetReuses,a.workingSetReuses);
    }
});

test('private singular compact factors retain rejection while public LU failures keep their retries',()=>{
    const f=fixture();f.chain.hessian.fill(0);f.rows=[];
    for(const compactWorkingSet of [true,false]) {
        const before=run(f,{compactWorkingSet,reuseRejectedWorkingSet:false}),after=run(f,{compactWorkingSet});
        assert.equal(after.failure,'band-lu-rejected');assert.equal(after.converged,false);
        assert.deepEqual(output(after),output(before));
        if(compactWorkingSet) {
            assert.ok(after.workingSetReuses>0);assert.ok(after.factorizations<before.factorizations);
        } else {
            assert.equal(after.workingSetReuses,0);assert.equal(after.factorizations,before.factorizations);
        }
    }
});

test('late crossover recovery preserves the whole physical checkpoint and subdivision path',async()=>{
    const fixture=JSON.parse(gunzipSync(fs.readFileSync(new URL('../reports/performance-scenarios/001-long-hold-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),request=fixture.stepRequest;
    try {
        const outputs=[];
        for(const reuseRejectedWorkingSet of [false,true]) {
            const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
            const iterator=advanceSharedAxis(input,request.rotations,request.dt,request.tools,
                {...request.options,wasmBasis:true,reuseRejectedWorkingSet});
            let next;do{next=iterator.next();}while(!next.done);
            const {state,result}=next.value;
            assert.ok(state&&result.converged&&result.quality.finite);
            assert.ok(result.certificateBound<=1e-6&&result.residual.length<=1e-5&&result.quality.maxPenetration<=1e-5);
            assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
            outputs.push({state:captureSharedAxisReplay(state,fixture.sheath),result});
        }
        const [before,after]=outputs;
        assert.deepEqual(after.state,before.state,'Geometry, reactions, velocities, friction and topology');
        for(const key of ['subdivisions','iterations','residual','certificateBound','negativeContactRecovery'])
            assert.deepEqual(after.result[key],before.result[key],key);
        const attempts=r=>r.attempts.map(({factorizations,...attempt})=>attempt);
        assert.deepEqual(attempts(after.result),attempts(before.result));
        assert.ok(after.result.factorizations<before.result.factorizations);
    } finally {anatomy.dispose();}
});
