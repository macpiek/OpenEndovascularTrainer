import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {deferSharedAxisContact} from '../src/physics/kirchhoffSharedAxisInactiveContacts.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';

const physical=r=>({increment:r.increment?.slice(),multiplierIncrement:r.multiplierIncrement?.slice(),
    residual:r.residual,converged:r.converged,failure:r.failure,factorizations:r.factorizations,
    activeSetAttempts:r.activeSetAttempts,batchFallback:r.batchFallback});
function fixture(n=12) {
    const layout={dofCount:n,band:1},rows=[];
    for(let i=0;i<n;i++) {
        rows.push({id:`dummy-${i}`,kind:'wall',dofs:[i],jacobian:[i%2?-0:0],gap:i%3,multiplier:i%2?-0:0});
        rows.push({id:`contact-${i}`,kind:'wall',dofs:[i],jacobian:[1],gap:-1-i/10,multiplier:i%3?.2:0});
    }
    return {layout,rows,chain:{layout,hessian:new Float64Array(n).fill(1)},
        fixed:new Uint8Array(n),gradient:new Float64Array(n).fill(.1)};
}
const options={reuseConstraintWork:true,reuseMatrixAssembly:true,lazyBasisCoefficients:true,deferActiveBasis:true};
const workspace=f=>createSharedAxisLinear(f.layout,f.rows,{lazy:true});
const run=(f,extra={},w=workspace(f))=>solveSharedAxisLinear(w,f.chain,{...f,...options,...extra});

for(const wasmLinearAssembly of [false,true])test(`exact constant inequalities leave directions, signed reactions and active-set decisions unchanged (WASM=${wasmLinearAssembly})`,()=>{
    for(const n of [1,12,91]) {
        const f=fixture(n),before=structuredClone(f);
        const expected=run(f,{wasmLinearAssembly,eliminateConstantRows:false});
        const actual=run(f,{wasmLinearAssembly});
        assert.ok(actual.converged);assert.equal(actual.constantRowsEliminated,n);
        assert.deepEqual(physical(actual),physical(expected));assert.deepEqual(f,before);
    }
});

test('changing coefficients, reactions, compliance, Hessians and feasibility never reuse an old elimination decision',()=>{
    const f=fixture(3),w=workspace(f),row=f.rows[0];
    const changes=[{}, {jacobian:[1e-12]}, {multiplier:1e-12}, {compliance:.1},
        {geometricHessian:new Float64Array([.1])}, {extraForceDofs:[0],extraForceJacobian:[.1]}, {gap:-1}, {}];
    for(const change of changes) {
        Object.assign(row,{jacobian:[0],multiplier:0,compliance:undefined,geometricHessian:undefined,
            extraForceDofs:undefined,extraForceJacobian:undefined,gap:1},change);
        if(change.gap<0) {
            assert.throws(()=>run(f,{eliminateConstantRows:false}),/Incompatible fixed shared axis constraint/);
            assert.throws(()=>run(f,{},w),/Incompatible fixed shared axis constraint/);
            continue;
        }
        const expected=run(f,{eliminateConstantRows:false}),actual=run(f,{},w);
        assert.deepEqual(physical(actual),physical(expected));
        if(actual.increment)assert.equal(actual.constantRowsEliminated,Object.keys(change).length?2:3);
    }
});

test('the captured long crossover preserves the complete accepted state and solver path',async()=>{
    const f=JSON.parse(gunzipSync(readFileSync(new URL('../reports/performance-scenarios/001-stage-05-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,f.anatomy),outputs=[];
    try {
        for(const eliminateConstantRows of [false,true]) {
            const input=restoreSharedAxisReplay({...f,contactCandidateMargin:.01},anatomy.field),before=captureSharedAxisReplay(input,f.sheath),req=f.stepRequest;
            const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,
                contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,eliminateConstantRows});
            let next;do{next=iterator.next();}while(!next.done);
            const {state,result}=next.value;assert.ok(state,JSON.stringify(result));assert.ok(result.converged);
            assert.deepEqual(captureSharedAxisReplay(input,f.sheath),before);
            outputs.push({state:captureSharedAxisReplay(state,f.sheath),result});
        }
        assert.deepEqual(outputs[1].state,outputs[0].state,'Positions, frames, rates, reactions and friction history remain exact');
        for(const key of ['quality','residual','certificateBound','iterations','factorizations','backtracks','geometryRestarts','frictionIterations','substepAttempts'])
            assert.deepEqual(outputs[1].result[key],outputs[0].result[key],key);
    } finally {anatomy.dispose();}
});

test('a deferred zero Jacobian remains available for exact activation by a large direction',()=>{
    const results=[];
    for(const eliminateConstantRows of [false,true]) {
        const layout={dofCount:6,band:6},row={id:'deferred',kind:'wall',dofs:[0,1,2,3,4,5],jacobian:[0,0,0,1,0,0],gap:1,multiplier:0};
        let evaluations=0;
        deferSharedAxisContact(row,()=>{evaluations++;return {gap:1,jacobian:[0,0,0,1,0,0]};},
            {state:{origin:[0,0,0]},a:[2,0,0],b:[2,1,0],radius:.5},1);
        const rows=[{id:'dummy',kind:'wall',dofs:[3],jacobian:[0],gap:1,multiplier:0},row];
        const hessian=new Float64Array(36);for(let i=0;i<6;i++)hessian[i*6]=1;
        const fixed=new Uint8Array(6).fill(1);fixed[3]=0;
        const gradient=new Float64Array(6);gradient[3]=3;
        const f={layout,rows,chain:{layout,hessian},fixed,gradient},result=run(f,{eliminateConstantRows});
        assert.ok(result.converged);assert.equal(evaluations,1);
        if(eliminateConstantRows)assert.equal(result.constantRowsEliminated,1);
        results.push(physical(result));
    }
    assert.deepEqual(results[1],results[0]);
});

test('an incompatible active basis still returns its original failure even before a first linear direction',()=>{
    const f=fixture(1);
    f.rows.push({id:'opposing',kind:'wall',dofs:[0],jacobian:[-1],gap:-1,multiplier:1});
    f.rows[1].multiplier=1;
    const expected=run(f,{eliminateConstantRows:false,deferActiveBasis:false});
    const actual=run(f,{deferActiveBasis:false});
    assert.equal(actual.converged,false);assert.deepEqual(physical(actual),physical(expected));
});

test('interleaved and cancelled reduced calls own their output and refresh membership on reuse',()=>{
    const a=fixture(12),b=fixture(18),wa=workspace(a),wb=workspace(b);
    const expectedA=physical(run(a,{eliminateConstantRows:false})),expectedB=physical(run(b,{eliminateConstantRows:false}));
    const ia=iterateSharedAxisLinear(wa,a.chain,{...a,...options}),ib=iterateSharedAxisLinear(wb,b.chain,{...b,...options});
    let na,nb;
    do{if(!na?.done)na=ia.next();if(!nb?.done)nb=ib.next();}while(!na.done||!nb.done);
    assert.deepEqual(physical(na.value),expectedA);assert.deepEqual(physical(nb.value),expectedB);
    const saved=physical(na.value);run(b,{},wb);assert.deepEqual(physical(na.value),saved);
    const cancelled=iterateSharedAxisLinear(wa,a.chain,{...a,...options});cancelled.next();cancelled.return();
    a.rows[0].jacobian[0]=1;a.rows[0].gap=-2;a.gradient.fill(.2);
    assert.deepEqual(physical(run(a,{},wa)),physical(run(a,{eliminateConstantRows:false})));
});
