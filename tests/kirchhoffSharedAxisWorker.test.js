import test from 'node:test';
import assert from 'node:assert/strict';
import {Worker} from 'node:worker_threads';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createSharedAxisWorkerRpc} from '../src/physics/kirchhoffSharedAxisWorkerRpc.js';
import {createSharedAxisWorkerSystem} from '../src/physics/kirchhoffSharedAxisWorkerSystem.js';
import {createSharedAxisAppSystem} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {SHARED_AXIS_BODY_FIELDS,publishSharedAxisWorkerBodies} from '../src/physics/kirchhoffSharedAxisWorkerData.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {createCrossoverFlow} from './helpers/crossoverFlow.js';

function workerPort() {
    const worker=new Worker(new URL('./helpers/sharedAxisPhysicsNodeWorker.mjs',import.meta.url));
    const port={postMessage:message=>worker.postMessage(message),terminate:()=>worker.terminate()};
    worker.on('message',data=>port.onmessage?.({data}));worker.on('error',error=>port.onerror?.(error));return port;
}
test('epoch reset rejects queued work and ignores late, duplicate and foreign replies',async()=>{
    const messages=[],worker={postMessage:m=>messages.push(m),terminate(){this.terminated=true;}};
    const rpc=createSharedAxisWorkerRpc(worker);
    const first=rpc.request('step',{}),old=messages.at(-1),cancelled=assert.rejects(first,{name:'AbortError'});
    const reset=rpc.reset(),current=messages.at(-1);await cancelled;
    worker.onmessage({data:{...old,value:'late'}});
    worker.onmessage({data:{...current,value:'reset'}});assert.equal(await reset,'reset');
    const next=rpc.request('step',{}),step=messages.at(-1);
    worker.onmessage({data:{...current,value:'duplicate'}});
    worker.onmessage({data:{...step,epoch:step.epoch-1,value:'wrong epoch'}});
    worker.onmessage({data:{...step,value:'accepted'}});assert.equal(await next,'accepted');
    rpc.dispose();assert.equal(worker.terminated,true);await assert.rejects(rpc.request('step',{}),/disposed/);
});
test('publication validates both tools before either visible buffer changes',()=>{
    const tools=['wire','catheter'].map(id=>({id,body:{x:new Float64Array([1,2])}}));
    assert.throws(()=>publishSharedAxisWorkerBodies(tools,[{id:'wire',x:new Float64Array([3,4])},{id:'catheter',x:new Float64Array(1)}]),/layout changed/);
    assert.deepEqual(Array.from(tools[0].body.x),[1,2]);assert.deepEqual(Array.from(tools[1].body.x),[1,2]);
});

test('real worker preserves crossover bodies, frames, flow samples and strict acceptance; reset cannot publish stale work',async()=>{
    const fixture=JSON.parse(gunzipSync(readFileSync(new URL('../reports/performance-scenarios/001-stage-05-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),world={contactField:anatomy.field};
    const makeTools=()=>fixture.tools.map(t=>{
        const count=208,body={count,segmentCount:count-1,mass:t.mass,radius:t.radius,wallStaticFriction:t.wallStaticFriction,wallKineticFriction:t.wallKineticFriction};
        for(const k of SHARED_AXIS_BODY_FIELDS)body[k]=new Float64Array(k.startsWith('orientation')?count-1:count);
        return {...t,rotation:fixture.stepRequest.rotations[t.id],body,nodeCoordinates:Array.from({length:count},(_,i)=>-40+5*i)};
    });
    const referenceTools=makeTools(),remoteTools=makeTools(),flow=createCrossoverFlow();flow.time=fixture.stepRequest.options.bloodFlow.time;
    const options={adaptiveMesh:{maxSpacing:20,contactMaxSpacing:20,shapeTolerance:.02,maxArcLoss:.0001,maxTurn:.06,contactMargin:.01,tipLength:80,tipRefinementAhead:80},
        contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,coupledFrictionNewton:true,predictiveNewton:true,
        readSheath:()=>fixture.sheath,readBloodFlow:()=>flow,physicsOptions:{...fixture.stepRequest.options,forceTolerance:1e-6,lengthTolerance:1e-5,omitInactiveSampleRows:true,recoverNegativeContact:true}};
    const reference=createSharedAxisAppSystem({...options,readTools:()=>referenceTools,workSliceMs:Infinity});
    let wake;
    const replaySteps=Number(process.env.OET_WORKER_CROSSOVER_STEPS??12);
    assert.ok(Number.isInteger(replaySteps)&&replaySteps>=12&&replaySteps<=1200);
    const remote=createSharedAxisWorkerSystem({...options,readTools:()=>remoteTools,createWorker:workerPort,onReady:()=>wake?.()});
    const visible=tools=>tools.map(({body})=>({joint:structuredClone(body.jointStateView),...Object.fromEntries(SHARED_AXIS_BODY_FIELDS.map(k=>[k,Array.from(body[k])]))}));
    const finish=async()=>{
        let ready=new Promise(r=>{wake=r;}),result=remote.step(world,1/60),before=visible(remoteTools);
        while(result.pending){await ready;assert.deepEqual(visible(remoteTools),before,'Messages never publish outside step()');ready=new Promise(r=>{wake=r;});result=remote.step(world,1/60);}
        assert.ok(result.accepted,JSON.stringify(result));return result;
    };
    try {
        reference.restoreReplay(fixture,world);remote.restoreReplay(fixture,world);
        assert.deepEqual(visible(remoteTools),visible(referenceTools));
        const bytes=anatomy.geometry.attributes.position.array.byteLength;
        for(let step=0;step<replaySteps;step++) {
            const offset=replaySteps===12?(step<6?-step*.25:(step-12)*.25):step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
            for(const tools of [referenceTools,remoteTools])for(const t of tools)t.insertion=fixture.tools.find(f=>f.id===t.id).insertion+offset;
            const expected=reference.step(world,1/60);assert.ok(expected.accepted,JSON.stringify(expected));
            const actual=await finish();assert.ok(actual.diagnostics.last.certificateBound<=1e-6,JSON.stringify(actual.diagnostics.last));
            assert.deepEqual(visible(remoteTools),visible(referenceTools));
            assert.deepEqual(remote.captureFlowSamplingState(),reference.captureFlowSamplingState());
            for(const key of ['iterations','factorizations','quality','residual','certificateBound'])
                assert.deepEqual(actual.diagnostics.last[key],expected.diagnostics.last[key],key);
            flow.update(1/60);
        }
        if(replaySteps>12)console.log(JSON.stringify({workerCrossoverReplay:{steps:replaySteps,identicalBodies:true,identicalFlowSamples:true,identicalSolverAcceptance:true}}));
        assert.equal(anatomy.geometry.attributes.position.array.byteLength,bytes,'Worker transfer cannot detach rendering geometry');
        const detached=remote.captureFlowSamplingState();detached.positions[0][0]+=999;assert.notDeepEqual(detached,remote.captureFlowSamplingState());
        assert.equal(remote.step(world,1/60).pending,true);remote.reset();
        for(const tools of [referenceTools,remoteTools])for(const t of tools)t.insertion=fixture.tools.find(f=>f.id===t.id).insertion;
        reference.restoreReplay(fixture,world);remote.restoreReplay(fixture,world);
        const expected=reference.step(world,1/60);assert.ok(expected.accepted);await finish();
        assert.deepEqual(visible(remoteTools),visible(referenceTools),'Reset did not apply the old pending reply');
        assert.throws(()=>remote.step({...world,readStentGraftSurface:()=>({})},1/60),/does not yet support/);
    }finally{remote.dispose();reference.reset();anatomy.dispose();}
});
