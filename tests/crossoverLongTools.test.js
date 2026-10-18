import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from './helpers/crossoverFlow.js';

const fixture=JSON.parse(gunzipSync(readFileSync(new URL('../reports/performance-scenarios/001-stage-05-replay.json.gz',import.meta.url))));
const request=fixture.stepRequest;
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
after(()=>anatomy.dispose());
function drain(iterator) {let next;do{next=iterator.next();}while(!next.done);return next.value;}

test('981 mm crossover avoids roundoff retries without relaxing certificates or changing the accepted pose',t=>{
    const outputs=[];
    for(const roundoffStagnation of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
        const {state,result}=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...request.options,roundoffStagnation,roundoffRecenter:false}));
        assert.ok(state&&result.converged,JSON.stringify(result));
        assert.ok(result.certificateBound<=1e-6);
        assert.ok(result.residual.length<=1e-5);
        assert.ok(result.quality.finite&&result.quality.maxPenetration<=1e-5);
        assert.ok(result.quality.bodies.every(b=>b.maxLengthError<1e-5));
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before,'rejected private trials preserve all accepted history');
        outputs.push({state:captureSharedAxisReplay(state,fixture.sheath),result});
    }
    const [reference,optimized]=outputs;
    assert.equal(reference.result.attempts[0].status,'iteration-limit');
    assert.equal(optimized.result.attempts[0].status,'roundoff-stagnation');
    assert.equal(optimized.result.subdivisions,reference.result.subdivisions);
    assert.ok(optimized.result.iterations<reference.result.iterations/10);
    for(const property of ['positions','frames','velocities','angularVelocities','wallFriction'])
        assert.deepEqual(optimized.state[property],reference.state[property],property);
    assert.deepEqual(optimized.result.residual,reference.result.residual);
    t.diagnostic(JSON.stringify({before:reference.result.iterations,after:optimized.result.iterations}));
});

test('cancelling subdivision after a roundoff stall keeps the complete incoming state',()=>{
    const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
    let subdividing=false;
    const iterator=advanceSharedAxis(input,request.rotations,request.dt,request.tools,{...request.options,
        roundoffRecenter:false,
        observeIteration:({state})=>{if(state.dynamicStep.dt<request.dt)subdividing=true;}});
    for(let yields=0;yields<10000&&!subdividing;yields++)assert.equal(iterator.next().done,false);
    assert.ok(subdividing);iterator.return();
    assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
});

test('recentered crossover preserves world geometry, history and continued motion with pulsatile flow',()=>{
    let state=restoreSharedAxisReplay(fixture,anatomy.field),rotations=request.rotations;
    const before=captureSharedAxisReplay(state,fixture.sheath),initial=state;
    const first=drain(advanceSharedAxis(state,rotations,request.dt,request.tools,request.options));
    assert.ok(first.state&&first.result.recenteredRoundoff);
    assert.ok(first.result.iterations<30);
    assert.deepEqual(captureSharedAxisReplay(initial,fixture.sheath),before);

    // An unguarded full step in the same translated chart is the numerical
    // reference. Subdivision changes the time discretization and thus rates.
    const origin=first.state.origin,originalOrigin=state.origin;
    state={...state,origin,positions:state.positions.map(p=>p.map((v,k)=>v+originalOrigin[k]-origin[k]))};
    const reference=drain(advanceSharedAxis(state,rotations,request.dt,request.tools,
        {...request.options,roundoffStagnation:false}));
    assert.ok(reference.state);
    for(let i=0;i<first.state.positions.length;i++)for(let k=0;k<3;k++)
        assert.ok(Math.abs(first.state.positions[i][k]-reference.state.positions[i][k])<1e-7);

    const flow=createCrossoverFlow();flow.time=request.options.bloodFlow.time+request.dt;
    state=first.state;rotations=first.rotations;
    // Holding previously failed after eight steps. Then cross mesh boundaries
    // in both directions; checks cover the state after every accepted step.
    for(let i=0;i<60;i++) {
        const offset=i<20?0:i<40?-(i-19)*.5:-(59-i)*.5;
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...tools.map(t=>t.insertion)));
        const next=drain(advanceSharedAxis(state,rotations,request.dt,tools,{...request.options,bloodFlow}));
        assert.ok(next.state&&next.result.converged,`step ${i}: ${next.result.status}`);
        assert.ok(next.result.certificateBound<=1e-6);
        assert.ok(next.result.quality.finite&&next.result.quality.maxPenetration<=1e-5);
        assert.ok(next.result.quality.bodies.every(b=>b.maxLengthError<1e-5));
        assert.ok(next.result.iterations<100,`step ${i} repeated an expensive stalled solve`);
        state=next.state;rotations=next.rotations;flow.update(request.dt);
    }
    assert.deepEqual(captureSharedAxisReplay(initial,fixture.sheath),before);
});

test('cancelling recentered recovery does not publish its translated origin or history',()=>{
    const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
    let recentered=false;
    const iterator=advanceSharedAxis(input,request.rotations,request.dt,request.tools,{...request.options,
        observeIteration:({state})=>{recentered ||= state.origin.some((v,k)=>v!==input.origin[k]);}});
    for(let yields=0;yields<10000&&!recentered;yields++)assert.equal(iterator.next().done,false);
    assert.ok(recentered);iterator.return();
    assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
});
