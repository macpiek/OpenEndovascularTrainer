import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createSharedAxisStagnationGuard} from '../src/physics/kirchhoffSharedAxisStagnationGuard.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const sample=(i,extra={})=>({force:(i%2?100:30)+i*1e-8,torque:i%2?20:10,constraint:0,rows:[],...extra});
test('stagnation requires two complete windows and tolerates drifting alternating residuals',()=>{
    const guard=createSharedAxisStagnationGuard(1,1);
    for(let i=0;i<15;i++)assert.equal(guard.observe(sample(i)),null);
    assert.equal(guard.observe(sample(15)).window,16);
});
test('progress of any unsatisfied channel and near-tolerance settling keep running',()=>{
    for(const channel of ['force','torque','constraint']) {
        const guard=createSharedAxisStagnationGuard(1,1);
        for(let i=0;i<32;i++)assert.equal(guard.observe(sample(i,{[channel]:100*.9**i})),null);
    }
    const guard=createSharedAxisStagnationGuard(1,1);
    for(let i=0;i<32;i++)assert.equal(guard.observe(sample(i,{force:9,torque:2})),null);
});
test('new rows and invalid residuals discard the old stagnation window',()=>{
    for(const extra of [{rows:[{}]},{force:NaN},{torque:Infinity},{constraint:-1}]) {
        const guard=createSharedAxisStagnationGuard(1,1);
        for(let i=0;i<15;i++)guard.observe(sample(i));
        assert.equal(guard.observe(sample(15,extra)),null);
        for(let i=0;i<15;i++)assert.equal(guard.observe(sample(i)),null);
        assert.ok(guard.observe(sample(15)));
    }
});
test('the strategy decision scales with the unchanged physical tolerances',()=>{
    const a=createSharedAxisStagnationGuard(1,1),b=createSharedAxisStagnationGuard(1e-6,1e-5);
    for(let i=0;i<24;i++) {
        const x=sample(i);
        assert.equal(Boolean(a.observe(x)),Boolean(b.observe({...x,force:x.force*1e-6,torque:x.torque*1e-6})));
    }
});

// Exercise the actual orchestration independently of numerical convergence:
// replace only the atomic attempt, preserving its yield/cancellation contract.
const source=readFileSync(new URL('../src/physics/kirchhoffSharedAxisTimeStep.js',import.meta.url),'utf8');
const scheduler=attempt=>new Function('iterateTimeStepAttempt',source.slice(source.indexOf('export function* iterateSharedAxisTimeStep'),source.indexOf('export function stepSharedAxis')).replace('export ','')+'\nreturn iterateSharedAxisTimeStep;')(attempt);
const finish=iterator=>{let next;do{next=iterator.next();}while(!next.done);return next.value;};
test('failed early alternative retries the original full dt and accounts for every attempt',()=>{
    for(const recoveryConverged of [true,false]) {
        const calls=[],responses=[
            {status:'live-contact-stagnation',converged:false},
            {status:'wall-friction-iteration-limit',converged:false},
            {status:recoveryConverged?'converged':'iteration-limit',converged:recoveryConverged},
            {status:'wall-friction-iteration-limit',converged:false},
        ];
        const iterate=scheduler(function*(state,dt,options){calls.push({dt,options});yield 'attempt';return {...responses[calls.length-1],iterations:3,factorizations:7,backtracks:2,ms:4,timings:{linearMs:2,assemblyMs:1}};});
        const result=finish(iterate({},1/60,{liveWallNormalLoad:true,stagnationFallback:true}));
        assert.equal(calls.length,recoveryConverged?3:4);
        assert.ok(calls.every(c=>c.dt===1/60),'Recovery must not shrink the physical timestep');
        assert.equal(calls[2].options.stagnationFallback,false);
        assert.equal(calls[2].options.liveWallNormalLoad,true);
        assert.equal(result.converged,recoveryConverged);
        assert.equal(result.stagnationRecovery.converged,recoveryConverged);
        assert.equal(result.wallNormalFallbacks,recoveryConverged?1:2);
        assert.equal(result.iterations,3*calls.length);
        assert.equal(result.factorizations,7*calls.length);
        assert.equal(result.backtracks,2*calls.length);
        assert.equal(result.ms,4*calls.length);
        assert.deepEqual(result.timings,{linearMs:2*calls.length,assemblyMs:calls.length});
    }
});
test('cancellation unwinds the current attempt without starting another strategy',()=>{
    let attempts=0,restored=0;
    const iterate=scheduler(function*(){attempts++;try{yield 'working';return {converged:false,status:'live-contact-stagnation'};}finally{restored++;}});
    const iterator=iterate({},1/60,{liveWallNormalLoad:true,stagnationFallback:true});
    iterator.next();iterator.return();assert.equal(attempts,1);assert.equal(restored,1);
});

const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
for(const name of ['anatomy-berenstein-feed-492.27-stagnation','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`early stagnation fallback preserves certified physical output: ${name}`,()=>{
    const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest,outputs=[];
    for(const stagnationFallback of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
        const {state,result}=finish(advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback}));
        assert.ok(state,JSON.stringify(result));assert.equal(result.converged,true);
        assert.equal(result.subdivisions,1);assert.equal(result.interToolRows,0);
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        assert.ok(result.residual.length<=req.options.lengthTolerance);
        assert.ok(result.quality.maxPenetration<=req.options.lengthTolerance);
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        outputs.push({result,state:captureSharedAxisReplay(state,fixture.sheath)});
    }
    const [reference,guard]=outputs;
    assert.deepEqual(guard.state,reference.state,'Pose, reactions, velocities and friction history must match');
    assert.deepEqual(guard.result.residual,reference.result.residual);
    assert.deepEqual(guard.result.quality,reference.result.quality);
    if(name.endsWith('stagnation')) {
        assert.equal(guard.result.wallNormalFallback.liveFailure,'live-contact-stagnation');
        assert.ok(guard.result.iterations<reference.result.iterations/2);
        assert.ok(guard.result.factorizations<reference.result.factorizations/2);
        assert.ok(guard.result.backtracks<reference.result.backtracks/2);
    }else {
        assert.equal(guard.result.wallNormalFallback,undefined);
        assert.equal(guard.result.iterations,reference.result.iterations);
    }
});
