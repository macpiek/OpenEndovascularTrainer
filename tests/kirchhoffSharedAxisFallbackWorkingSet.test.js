import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createSharedAxisFallbackWorkingSetCache,fallbackWorkingSetLease} from '../src/physics/kirchhoffSharedAxisRejectedLinearization.js';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

function problem() {
    return {chain:{layout:{dofCount:2,band:2},hessian:new Float64Array([1,0,1,0])},
        options:{rows:[{kind:'wall',dofs:[0,1],jacobian:[1,1],gap:-1,multiplier:2}],
            gradient:new Float64Array([3,2]),fixed:new Uint8Array(2),tolerance:1e-8,
            maxActiveSetAttempts:16,batchRelease:true,reuseWorkingSet:true},
        result:{converged:false,residual:1e-5,increment:new Float64Array([1,2]),
            multiplierIncrement:new Float64Array([-1]),factorizations:3,workingSetReuses:2,cpuMs:7}};
}
test('fallback working-set cache invalidates changed equations while allowing a different pivot search',()=>{
    const p=problem(),cache=createSharedAxisFallbackWorkingSetCache();
    const solutions=new Map([['1',structuredClone(p.result)]]);
    cache.remember(p.chain,p.options,p.result,solutions);
    p.result.increment[0]=100;
    assert.equal(cache.find(p.chain,p.options).get('1').increment[0],1);
    for(const change of [
        (c,o)=>c.hessian[0]+=1e-12,(c,o)=>o.gradient[0]+=1e-12,(c,o)=>o.fixed[0]=1,
        (c,o)=>o.rows[0].gap+=1e-12,(c,o)=>o.rows[0].multiplier+=1e-12,
        (c,o)=>o.rows[0].jacobian[0]+=1e-12,(c,o)=>o.rows[0].dofs[0]=1,
        (c,o)=>o.rows[0].compliance=0,(c,o)=>o.rows[0].extraForceDofs=[0],
        (c,o)=>o.tolerance*=2,(c,o)=>o.wasmLinearAssembly=true,
        (c,o)=>o.reuseMatrixAssembly=false,(c,o)=>o.futureFlag=true
    ]) {
        const q=problem();q.chain.layout=p.chain.layout;change(q.chain,q.options);
        assert.equal(cache.find(q.chain,q.options),null);
    }
    for(const extra of [{maxActiveSetAttempts:4},{batchRelease:false},{recoverNegativeContact:true},
        {zeroDualStart:true},{batchActivationSize:1},{deferActiveBasis:false}])
        assert.equal(cache.find(p.chain,{...p.options,...extra}),solutions);
    const other=problem();assert.equal(cache.find(other.chain,other.options),null,'Separate layout/solver');
    cache.clear();assert.equal(cache.find(p.chain,p.options),null);
});

test('successful, limited, retained-factor, observed and unknown solver strategies retain their full retries',()=>{
    const p=problem();
    for(const result of [{...p.result,converged:true},{...p.result,failure:'active-set-limit'},
        {...p.result,batchFailure:'active-set-limit'}]) {
        const cache=createSharedAxisFallbackWorkingSetCache();cache.remember(p.chain,p.options,result,new Map([['1',structuredClone(result)]]));
        assert.equal(cache.find(p.chain,p.options),null);
    }
    for(const extra of [{observeLinearSystem(){}},{trace:[]},{incrementalContacts:true},
        {modifiedNewtonContext:{}},{compactWorkingSet:false},{condenseCompliantContacts:true},
        {primalCompliantContacts:true},{unknownCallback(){}},{unknownState:{}}]) {
        const cache=createSharedAxisFallbackWorkingSetCache(),options={...p.options,...extra};
        cache.remember(p.chain,options,p.result,new Map([['1',structuredClone(p.result)]]));assert.equal(cache.find(p.chain,options),null);
    }
});

test('only four rejected equation sets are retained within a timestep',()=>{
    const p=problem(),cache=createSharedAxisFallbackWorkingSetCache();
    for(let n=0;n<5;n++){p.options.gradient[0]=n;cache.remember(p.chain,p.options,p.result,new Map([['1',structuredClone(p.result)]]));}
    p.options.gradient[0]=0;assert.equal(cache.find(p.chain,p.options),null);
    p.options.gradient[0]=1;assert.ok(cache.find(p.chain,p.options));
    p.options.gradient[0]=4;assert.ok(cache.find(p.chain,p.options));
});

for(const wasmLinearAssembly of [false,true])test(`a different search can recover using rejected KKT work without accepting its old failure (WASM=${wasmLinearAssembly})`,()=>{
    const epsilon=1e-7,layout={dofCount:2,band:2};
    const rows=[{kind:'wall',id:'loose',dofs:[0,1],jacobian:[.64,.768],gap:-1,multiplier:1},
        {kind:'wall',id:'tight',dofs:[0,1],jacobian:[.64+epsilon,.768-epsilon],gap:-1.0001,multiplier:1}];
    const chain={layout,hessian:Float64Array.from([1,0,1,0])},cache=createSharedAxisFallbackWorkingSetCache();
    const options={rows,gradient:Float64Array.from([-(1.28+epsilon),-(1.536-epsilon)]),fixed:new Uint8Array(2),
        tolerance:1e-8,reuseConstraintWork:true,reuseMatrixAssembly:true,deferActiveBasis:true,wasmLinearAssembly};
    const run=extra=>solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{...options,...extra});
    const rejected=run({[fallbackWorkingSetLease]:cache,zeroDualStart:true,batchRelease:true});
    assert.equal(rejected.converged,false);
    const actual=run({[fallbackWorkingSetLease]:cache,recoverNegativeContact:true});
    const expected=run({recoverNegativeContact:true});
    assert.equal(actual.converged,true);assert.equal(expected.converged,true);
    for(const key of ['increment','multiplierIncrement','residual','activeSetAttempts','negativeReactionRecoveries'])
        assert.deepEqual(actual[key],expected[key],key);
    assert.ok(cache.hits>0);assert.ok(actual.factorizations<expected.factorizations);
});

test('repeated crossover fallbacks preserve the complete physical checkpoint and cancellation',async()=>{
    const fixture=JSON.parse(gunzipSync(fs.readFileSync(new URL('../reports/performance-scenarios/001-temporal-step-99-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),request=fixture.stepRequest;
    const drain=iterator=>{let n;do{n=iterator.next();}while(!n.done);return n.value;};
    try {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
        const interrupted=advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...request.options,reuseFallbackWorkingSets:true});
        let event;
        do{event=interrupted.next();assert.equal(event.done,false);}while(event.value.kind!=='direction');
        interrupted.return();assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        const reference=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...request.options,reuseFallbackWorkingSets:false}));
        const cached=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...request.options,reuseFallbackWorkingSets:true}));
        for(const value of [reference,cached]) {
            assert.ok(value.state&&value.result.converged&&value.result.certificateBound<=1e-6);
            assert.ok(value.result.residual.length<=1e-5&&value.result.quality.maxPenetration<=1e-5);
        }
        assert.deepEqual(captureSharedAxisReplay(cached.state,fixture.sheath),captureSharedAxisReplay(reference.state,fixture.sheath));
        for(const key of ['subdivisions','residual','certificateBound','iterations'])assert.deepEqual(cached.result[key],reference.result[key],key);
        assert.ok(cached.result.fallbackLinearizationReuses>0);
        assert.ok(cached.result.factorizations<reference.result.factorizations);
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        // This experiment switches strategies based on factor count. A cache
        // must not silently move that trigger and change its integration path.
        const earlyOptions={...request.options,earlyPredictorFallback:true};
        const earlyReference=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...earlyOptions,reuseFallbackWorkingSets:false}));
        const earlyDefault=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,earlyOptions));
        assert.ok(earlyReference.state&&earlyDefault.state);
        assert.equal(earlyDefault.result.fallbackLinearizationReuses,0);
        assert.equal(earlyDefault.result.factorizations,earlyReference.result.factorizations);
        assert.deepEqual(captureSharedAxisReplay(earlyDefault.state,fixture.sheath),
            captureSharedAxisReplay(earlyReference.state,fixture.sheath));
    } finally {anatomy.dispose();}
});
