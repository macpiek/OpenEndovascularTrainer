import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import vm from 'node:vm';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';
import {createCoupledRuntimeFixture} from './helpers/coupledRuntimeFixture.js';
import {createCoupledSolverSelection} from '../src/physics/coupledSolverSelection.js';
import {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';
import {solveKirchhoffTwoChannelSystem} from '../src/physics/kirchhoffTwoChannelSystem.js';
import {configureKirchhoffSplitBias} from '../src/physics/kirchhoffSplitMotion.js';

const dt=1/120;
function stubWorld(outcomes=[false,false,true]) {
    return {fixedDt:dt,accumulator:0,stepCount:0,pending:false,attempts:0,admitted:[],
        advance(elapsed,prepare) {
            this.admitted.push(elapsed);this.accumulator+=elapsed;
            if(!this.pending){prepare?.();this.pending=true;}
            const outcome=outcomes[this.attempts++]??true;
            if(outcome instanceof Error)throw outcome;
            if(outcome===false){this.lastStepResult={accepted:false,status:'rejected'};return 0;}
            this.stepCount++;this.accumulator-=dt;this.pending=false;
            this.lastStepResult=outcome==='void'?undefined:{accepted:true};return 1;
        },reset(){this.accumulator=0;this.pending=false;this.stepCount=0;}};
}

test('two rejections retry one prepared dt, defer mutable settings, and preserve the next command for a new dt',()=>{
    const world=stubWorld(),prepared=[];let command=1,stiffness=3,boundaries=0;
    const owner=createFixedStepTransaction({world,beforePrepare:()=>boundaries++,prepare:()=>{
        const context=Object.freeze({command,stiffness});prepared.push(context);return context;}});
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);
    command=-1;owner.change('stiffness',()=>stiffness=7);owner.change('stiffness',()=>stiffness=9);
    assert.equal(stiffness,3);assert.equal(owner.attempt(dt).attempted,false);
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);
    assert.equal(prepared.length,1);assert.equal(boundaries,1);
    owner.beginFrame();const committed=owner.attempt(dt);
    assert.equal(committed.accepted,true);assert.equal(committed.context,prepared[0]);
    assert.deepEqual(committed.context,{command:1,stiffness:3});assert.equal(stiffness,3);
    assert.equal(world.stepCount,1);assert.ok(Math.abs(world.accumulator)<1e-12);
    assert.deepEqual(world.admitted,[dt,0,0]);
    assert.equal(owner.attempt(dt).accepted,true);assert.deepEqual(prepared[1],{command:-1,stiffness:9});
});

test('reset invalidates pending input without discarding the caller backlog; void World steps count as accepted',()=>{
    const world=stubWorld([false,'void']);let prepares=0,backlog=4*dt;
    const owner=createFixedStepTransaction({world,prepare:()=>++prepares});
    owner.beginFrame();owner.attempt(dt);const epoch=owner.epoch;
    owner.reset();world.reset();assert.notEqual(owner.epoch,epoch);assert.equal(backlog,4*dt);
    owner.beginFrame();const result=owner.attempt(dt);assert.equal(result.accepted,true);backlog-=dt;
    assert.equal(result.context,2);assert.equal(world.stepCount,1);assert.equal(backlog,3*dt);
    assert.deepEqual(world.admitted,[dt,dt]);
    owner.dispose();assert.equal(owner.attempt(dt).attempted,false);
});

test('solver exceptions keep a prepared dt and expose their cost; mode/dt errors never re-prepare input',()=>{
    const error=new Error('solver failed'),world=stubWorld([error,true]);let prepares=0,clock=0;
    const owner=createFixedStepTransaction({world,prepare:()=>++prepares,now:()=>clock++});
    owner.beginFrame();const failed=owner.attempt(dt);
    assert.equal(failed.error,error);assert.equal(failed.firstError,error);assert.equal(failed.accepted,false);assert.equal(failed.durationMs,1);
    assert.equal(world.stepCount,0);assert.equal(owner.pending,true);
    owner.beginFrame();const secondary=owner.attempt(dt/2);assert.equal(secondary.accepted,false);assert.equal(secondary.firstError,error);assert.notEqual(secondary.error,error);assert.equal(prepares,1);
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,true);assert.equal(prepares,1);
    assert.deepEqual(world.admitted,[dt,0]);
});

test('a partial preparation exception cannot apply input again before explicit reset',()=>{
    const world=stubWorld([true]);let feed=0,shouldThrow=true;
    const owner=createFixedStepTransaction({world,prepare:()=>{feed++;if(shouldThrow)throw new Error('preparation failed');return feed;}});
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);assert.equal(feed,1);
    owner.beginFrame();assert.match(owner.attempt(dt).error.message,/reset is required/);assert.equal(feed,1);
    owner.reset();world.reset();shouldThrow=false;owner.beginFrame();
    assert.equal(owner.attempt(dt).accepted,true);assert.equal(feed,2);assert.equal(world.stepCount,1);
});

const simulatorSource=fs.readFileSync(new URL('../src/simulator.js',import.meta.url),'utf8');
function schedulerHarness(outcomes) {
    const world=stubWorld(outcomes),queue=[],errors=[];let clock=0;
    const originalAdvance=world.advance.bind(world);world.advance=(...args)=>{clock+=1;return originalAdvance(...args);};
    const state={world,queue,errors,performance:{now:()=>clock},console:{error:(...args)=>errors.push(args)},
        fixedDt:dt,MAX_PHYSICS_STEPS_PER_FRAME:2,MAX_IDLE_PHYSICS_STEPS:6,
        TARGET_RENDER_FRAME_MS:1000/60,PHYSICS_IDLE_GUARD_MS:.75,PHYSICS_RENDER_RESERVE_MS:3.5,
        lastRenderTime:null,simulationAccumulator:0,simulationPeakBacklog:0,simulationPeakBacklogScenarioMs:0,
        simulationPeakBacklogElapsedMs:0,simulationPeakBacklogGuidewireMm:0,
        simulationExecutedSteps:0,simulationIdleExecutedSteps:0,simulationAcceptedTime:0,
        simulationStepEstimateMs:1,simulationCatchupPending:false,simulationLastAttempt:null,
        compositeAppSystem:null,compositeStatus:null,compositePhysicsClockStarted:false,endovascularWorld:world,
        browserBenchmarkScenario:{running:false},shortCatheterBenchmarkMetrics:null,guidewireTransport:{progress:0},
        recordBrowserFrame:()=>{},document:{visibilityState:'visible'},runtime:{timeout:cb=>queue.push(cb)},
        prepares:0,commits:0,contrastTime:0,benchmarkTime:0,throwPresentation:false};
    state.simulationStepTransaction=createFixedStepTransaction({world,prepare:()=>++state.prepares,now:()=>clock});
    state.stepSimulation=stepDt=>state.simulationStepTransaction.attempt(stepDt);
    state.commitSimulationStep=()=>{state.commits++;state.contrastTime+=dt;state.benchmarkTime+=dt;
        if(state.throwPresentation)throw new Error('UI failed');};
    const scheduler=simulatorSource.slice(simulatorSource.indexOf('function executeAccumulatedPhysicsStep('),simulatorSource.indexOf('function animate(time)'));
    const frame=simulatorSource.slice(simulatorSource.indexOf('function animate(time)'),simulatorSource.indexOf('    const frameSimulationEndedAt',simulatorSource.indexOf('function animate(time)')))+'\n}';
    vm.createContext(state);vm.runInContext(scheduler+'\n'+frame,state);
    state.frame=time=>{clock=time;state.animate(time);};
    return state;
}

test('Joint physics starts after vessel loading without preparing an empty field or admitting loading time',()=>{
    const s=schedulerHarness([true]);s.compositeAppSystem={};s.assetsReady=false;s.loadingAssetsReady=()=>s.assetsReady;
    s.frame(0);s.frame(5000);
    assert.equal(s.world.attempts,0);assert.equal(s.prepares,0);assert.equal(s.simulationAccumulator,0);
    s.assetsReady=true;s.world.contactField={};s.frame(5016);
    assert.equal(s.world.attempts,0);assert.equal(s.simulationAccumulator,0);
    s.frame(5032);
    assert.ok(s.world.attempts>0);assert.equal(s.simulationAcceptedTime,.016);
    assert.ok(Math.abs(s.simulationAcceptedTime-s.simulationExecutedSteps*dt-s.simulationAccumulator)<1e-12);
});

test('actual rAF and already queued idle callback share one rejected-attempt limit and do not consume backlog',()=>{
    const s=schedulerHarness([false,false,true]);
    s.frame(0);const oldEpoch=s.simulationStepTransaction.epoch;
    s.queue.push(()=>s.runIdlePhysicsCatchup(null,oldEpoch));
    s.frame(25);assert.equal(s.world.attempts,1);assert.equal(s.simulationExecutedSteps,0);
    while(s.queue.length)s.queue.shift()();
    assert.equal(s.world.attempts,1);assert.equal(s.simulationAccumulator,.025);
    assert.equal(s.simulationIdleExecutedSteps,0);assert.equal(s.contrastTime,0);assert.equal(s.benchmarkTime,0);
    s.frame(50);assert.equal(s.world.attempts,2);assert.equal(s.prepares,1);assert.equal(s.simulationAccumulator,.05);
    s.frame(75);assert.equal(s.simulationExecutedSteps,2);assert.equal(s.commits,2);
    assert.equal(s.world.stepCount,s.simulationExecutedSteps);
    assert.ok(Math.abs(s.simulationAcceptedTime-s.simulationExecutedSteps*dt-s.simulationAccumulator)<1e-12);
});

test('first failure in idle blocks another idle callback; reset/dispose invalidates stale callbacks',()=>{
    const s=schedulerHarness([true,true,false,true]);s.frame(0);s.frame(25);
    assert.equal(s.simulationExecutedSteps,2);assert.equal(s.queue.length,1);
    s.queue.shift()();assert.equal(s.world.attempts,3);assert.equal(s.simulationIdleExecutedSteps,0);
    s.runIdlePhysicsCatchup({timeRemaining:()=>100});assert.equal(s.world.attempts,3);
    const epoch=s.simulationStepTransaction.epoch;s.simulationStepTransaction.reset();s.world.reset();
    s.runIdlePhysicsCatchup({timeRemaining:()=>100},epoch);assert.equal(s.world.attempts,3);
    s.simulationStepTransaction.dispose();s.runIdlePhysicsCatchup({timeRemaining:()=>100});assert.equal(s.world.attempts,3);
});

test('actual executor commits counters once even when presentation throws after World success',()=>{
    const s=schedulerHarness([true]);s.throwPresentation=true;s.frame(0);s.frame(25);
    assert.equal(s.world.stepCount,1);assert.equal(s.simulationExecutedSteps,1);assert.equal(s.commits,1);
    assert.equal(s.simulationStepTransaction.pending,false);assert.equal(s.errors.length,1);
    assert.ok(Math.abs(s.simulationAccumulator-(.025-dt))<1e-12);
    s.runIdlePhysicsCatchup({timeRemaining:()=>100});assert.equal(s.world.attempts,1);
});

test('actual benchmark sampler advances no clock; lifecycle reset runs before World receives its next dt',()=>{
    const world=stubWorld([false,true,true]);let resets=0,prepares=0,clock=25;
    const state={performance:{now:()=>clock},browserBenchmarkScenario:{running:true,warmingUp:true,
        warmupStartedAt:0,memorySettling:false,simulationElapsedMs:7,startedAt:0,durationMs:1000},
        BROWSER_BENCHMARK_CHOREOGRAPHY_WARMUP_MS:20,BROWSER_BENCHMARK_WARMUP_MS:100,
        browserBenchmarkCommands:{},shortCatheterBenchmarkMetrics:{},
        resetBrowserBenchmark:()=>{},stopBrowserBenchmarkScenario:()=>{},
        sampleActiveBrowserBenchmarkCommands:()=>({catheterAdvance:1}),
        resetBrowserBenchmarkSimulation:()=>{assert.equal(world.accumulator,0);world.reset();resets++;}};
    vm.createContext(state);vm.runInContext(simulatorSource.slice(simulatorSource.indexOf('function prepareBrowserBenchmarkBoundary()'),
        simulatorSource.indexOf('function sampleActiveBrowserBenchmarkCommands(')),state);
    const owner=createFixedStepTransaction({world,beforePrepare:state.prepareBrowserBenchmarkBoundary,prepare:()=>{
        prepares++;return state.sampleBrowserBenchmarkScenario();}});
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);assert.equal(resets,1);
    assert.equal(state.browserBenchmarkScenario.simulationElapsedMs,0);
    clock=150;owner.beginFrame();assert.equal(owner.attempt(dt).accepted,true);assert.equal(resets,1);assert.equal(prepares,1);
    assert.equal(state.browserBenchmarkScenario.warmingUp,true,'pending retry cannot reset the warmup epoch');
    owner.attempt(dt);assert.equal(state.browserBenchmarkScenario.warmingUp,false);assert.equal(prepares,2);
});

test('actual post-commit credits only the originating benchmark epoch while always advancing committed contrast once',()=>{
    const calls={sync:0,metrics:0,envelope:0,resistance:0,contrast:0,dose:0};
    const state={xpbdWireBody:{syncToRodState:()=>calls.sync++},xpbdCatheterBody:{},wire:{},
        xpbdContainment:{outerStartNode:0,innerRadius:1,closestSegment:null},spatiallyCapturedContainmentEnd:()=>0,
        browserBenchmarkEpoch:2,browserBenchmarkScenario:{running:true,simulationElapsedMs:0},
        simulationAccumulator:3*dt,shortCatheterBenchmarkMetrics:{recordStep:()=>calls.metrics++},
        endovascularWorld:{},recordBrowserPhysicsEnvelope:()=>calls.envelope++,
        contrastSystem:{update:time=>calls.contrast+=time,totalDeliveredVolumeMl:1},displayedContrastDoseMl:0,
        updateGuidewireResistance:()=>calls.resistance++,guidewireRotation:.1,
        pigtailCatheter:{progress:4,rotation:.2},ui:{updateInsertedLength:()=>{},updateCatheterLength:()=>{},updateDose:()=>calls.dose++}};
    vm.createContext(state);vm.runInContext(simulatorSource.slice(simulatorSource.indexOf('function commitSimulationStep('),
        simulatorSource.indexOf('const renderHiddenObjects')),state);
    const context={dt,automatedCommands:{benchmarkPhase:4},inserted:12,firstContainedNode:0,materialEndNode:0,
        benchmarkEpoch:1,benchmarkRunning:true,benchmarkClockAdvances:true};
    state.commitSimulationStep(context);
    assert.equal(calls.sync,1);assert.equal(calls.contrast,dt);assert.equal(calls.metrics,0);
    assert.equal(state.browserBenchmarkScenario.simulationElapsedMs,0);
    state.commitSimulationStep({...context,benchmarkEpoch:2});
    assert.equal(calls.contrast,2*dt);assert.equal(calls.metrics,1);assert.equal(calls.envelope,1);
    assert.equal(state.browserBenchmarkScenario.simulationElapsedMs,dt*1000);
});

test('actual physics-setting UI callbacks leave pending material and friction untouched',()=>{
    const world=stubWorld([false,true,true]),owner=createFixedStepTransaction({world,prepare:()=>({})});
    const calls={catheter:0,wire:0,friction:0,wake:0};
    const state={changePhysicsSetting:(key,apply)=>owner.change(key,apply),
        THREE:{MathUtils:{clamp:(v,lo,hi)=>Math.max(lo,Math.min(hi,v))}},
        MIN_CATHETER_STIFFNESS_SCALE:0,MAX_CATHETER_SHAFT_STIFFNESS_SCALE:100,MAX_CATHETER_TIP_STIFFNESS_SCALE:100,
        MIN_GUIDEWIRE_STIFFNESS_SCALE:0,MAX_GUIDEWIRE_SHAFT_STIFFNESS_SCALE:100,MAX_GUIDEWIRE_TIP_STIFFNESS_SCALE:100,
        catheterShaftStiffnessScale:1,catheterTipStiffnessScale:1,guidewireShaftStiffnessScale:1,guidewireTipStiffnessScale:1,
        guidewireStaticWallFriction:.1,guidewireKineticWallFriction:.1,
        pigtailCatheter:{setStiffnessScales:()=>calls.catheter++},xpbdWireBody:{wake:()=>calls.wake++},xpbdCatheterBody:{wake:()=>calls.wake++},
        applyActiveGuidewireElasticProfile:()=>{},
        applyActiveGuidewireKirchhoffProfile:()=>calls.wire++,applyActiveGuidewireWallFriction:()=>calls.friction++};
    const callbacks=simulatorSource.slice(simulatorSource.indexOf('    onCatheterStiffnessChange:'),simulatorSource.indexOf('    onContrastHemodynamicsChange:'));
    vm.createContext(state);vm.runInContext('globalThis.callbacks={'+callbacks+'};',state);
    owner.beginFrame();owner.attempt(dt);
    state.callbacks.onCatheterStiffnessChange({shaftStiffnessScale:20,tipStiffnessScale:4});
    state.callbacks.onGuidewireStiffnessChange({shaftStiffnessScale:8,tipStiffnessScale:3});
    state.callbacks.onGuidewireFrictionChange({staticFriction:.03,kineticFriction:.01});
    assert.deepEqual(calls,{catheter:0,wire:0,friction:0,wake:0});assert.equal(state.guidewireStaticWallFriction,.1);
    owner.beginFrame();assert.equal(owner.attempt(dt).accepted,true);assert.equal(calls.friction,0);
    owner.attempt(dt);assert.deepEqual(calls,{catheter:1,wire:1,friction:1,wake:0});
    assert.equal(state.guidewireStaticWallFriction,.03);assert.equal(state.catheterShaftStiffnessScale,20);
});

test('real World.advance accepts void independent/legacy steps and prepares the first eligible split once',()=>{
    for(const id of ['reference','joint-two-channel']) {
        const selection=createCoupledSolverSelection(id,{solve:solveKirchhoffCoupledSystem,apply:applyKirchhoffCoupledCorrection,
            solveTwoChannel:solveKirchhoffTwoChannelSystem});
        const f=createCoupledRuntimeFixture({coupledSystem:selection.coupledSystem,jointMotionMode:selection.jointMotionMode});
        if(selection.biasMaterialMode)configureKirchhoffSplitBias(f.containment,{materialMode:selection.biasMaterialMode});
        try {
            for(let i=0;i<33;i++)f.step({guidewireAdvance:1});
            const before=f.world.stepCount;let prepares=0;const returns=[],stepFixed=f.world.stepFixed;
            f.world.stepFixed=function(){const result=stepFixed.call(this);returns.push(result);return result;};
            const owner=createFixedStepTransaction({world:f.world,prepare:()=>{
                prepares++;const original=f.world.stepFixed;f.world.stepFixed=()=>({accepted:false,status:'prepared-only'});
                try{f.step({catheterAdvance:1});}finally{f.world.stepFixed=original;}
                return f.snapshot();}});
            owner.beginFrame();
            for(let i=0;i<10;i++) {
                const result=owner.attempt(dt);assert.equal(result.accepted,true,result.error?.message);
                f.wireBody.syncToRodState(f.wire);
            }
            assert.equal(prepares,10);assert.equal(f.world.stepCount,before+10);assert.equal(returns[0],undefined);
            assert.equal(f.containment.enabled,true);assert.ok(Math.abs(f.world.accumulator)<1e-12);
            if(id==='joint-two-channel'){assert.equal(returns.at(-1).accepted,true);assert.equal(returns.at(-1).diagnostics.historyCommits,1);}
        } finally{f.dispose();}
    }
});

test('real two-channel World and real tool transports retry without repeating feed, rotation, sync or path history',()=>{
    const selection=createCoupledSolverSelection('joint-two-channel',{solve:solveKirchhoffCoupledSystem,
        apply:applyKirchhoffCoupledCorrection,solveTwoChannel:solveKirchhoffTwoChannelSystem});
    const f=createCoupledRuntimeFixture({coupledSystem:selection.coupledSystem,jointMotionMode:selection.jointMotionMode});
    configureKirchhoffSplitBias(f.containment,{materialMode:selection.biasMaterialMode});
    try {
        for(let i=0;i<33;i++)assert.notEqual(f.step({guidewireAdvance:1}).accepted,false);
        for(let i=0;i<18;i++)assert.notEqual(f.step({catheterAdvance:1}).accepted,false);
        f.world.coupledClosureMaxPasses=3;
        const beforeSteps=f.world.stepCount,beforeMm=f.catheter.progress,beforePathStep=f.catheter._physicsStepIndex;
        const beforeRotations={wire:f.snapshot().wireRotation,catheter:f.catheter.rotation};
        let prepares=0,preparedSnapshot;
        const owner=createFixedStepTransaction({world:f.world,prepare:()=>{
            prepares++;const original=f.world.stepFixed;
            // Use the real adapter's complete actuation path. Its solver is
            // intercepted only here so the outer World.advance owns this dt.
            f.world.stepFixed=()=>({accepted:false,status:'prepared-only'});
            try {f.step({catheterAdvance:1,guidewireRotation:1,catheterRotation:-1});} finally {f.world.stepFixed=original;}
            preparedSnapshot=f.snapshot();return {snapshot:preparedSnapshot};
        }});
        owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);
        const path=structuredClone(f.catheter.pathSamples),layout=f.catheter._xpbdProgress;
        assert.equal(f.catheter._physicsStepIndex,beforePathStep+1);
        assert.deepEqual(f.snapshot(),preparedSnapshot);assert.equal(f.world.stepCount,beforeSteps);
        owner.beginFrame();assert.equal(owner.attempt(dt).accepted,false);
        assert.equal(prepares,1);assert.deepEqual(f.catheter.pathSamples,path);assert.equal(f.catheter._xpbdProgress,layout);
        assert.equal(f.catheter._physicsStepIndex,beforePathStep+1);assert.deepEqual(f.snapshot(),preparedSnapshot);
        assert.ok(Math.abs(f.catheter.progress-beforeMm-52/120)<1e-12);
        assert.ok(Math.abs(f.snapshot().wireRotation-beforeRotations.wire-Math.PI*.9*dt)<1e-12);
        assert.ok(Math.abs(f.catheter.rotation-beforeRotations.catheter)>0,'the real catheter rotation command was prepared');
        f.world.coupledClosureMaxPasses=32;
        owner.beginFrame();const result=owner.attempt(dt);assert.equal(result.accepted,true,result.error?.message);
        assert.equal(prepares,1);assert.equal(f.world.stepCount,beforeSteps+1);
        assert.equal(f.world.getStats().jointMotion.historyCommits,1);assert.ok(Math.abs(f.world.accumulator)<1e-12);
        assert.equal(f.catheter.progress,preparedSnapshot.catheterMm);
    } finally {f.dispose();}
});
