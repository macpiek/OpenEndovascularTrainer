import {captureBloodFlow} from './bloodFlowCoupling.js';
import {createCapturePotential} from '../devices/stentGraftCapture.js';
import {captureSharedAxisReplay} from './kirchhoffSharedAxisReplay.js';
import {restoreSharedAxisReplay} from './kirchhoffSharedAxisRestore.js';
import {createSharedAxisContacts} from './kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,feedSharedAxisNative,rotateSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {iterateSharedAxisProjective} from './kirchhoffSharedAxisProjective.js';
import {iterateSharedAxisTimeStep} from './kirchhoffSharedAxisTimeStep.js';
import {adaptiveMeshOptions,ADAPTIVE_SOLVE_OPTIONS} from './kirchhoffSharedAxisAdaptiveMesh.js';
import {createStentGraftContacts} from '../devices/stentGraftContacts.js';

const angleDifference=(a,b)=>Math.atan2(Math.sin(a-b),Math.cos(a-b));
export const SHARED_AXIS_APP_PHYSICS_OPTIONS=Object.freeze({liveWallNormalLoad:true,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback:true,wasmMaterial:true,reuseMaterialScratch:true,lightweightFriction:true,reuseTriangleKernel:true,earlyContactPreflight:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true,omitInactiveSampleRows:true});
const profile=t=>({id:t.id,type:t.type,...(t.type==='stentgraft-delivery'?{deliveryExposureMm:t.deliveryExposureMm??0}:{}),shaftStiffness:t.shaftStiffness,tipStiffness:t.tipStiffness,length:t.length??1000,mass:t.mass??t.body?.mass,radius:t.radius??t.body?.radius,
    wallStaticFriction:t.wallStaticFriction??t.body?.wallStaticFriction??0,wallKineticFriction:t.wallKineticFriction??t.body?.wallKineticFriction??0});
function edgeAt(coordinates,x) {
    let low=0,high=coordinates.length-1;
    while(low+1<high){const mid=(low+high)>>>1;if(coordinates[mid]<=x)low=mid;else high=mid;}
    return low;
}
export function sampleSharedAxisPosition(s,x,out=[0,0,0]) {
    const edge=edgeAt(s.coordinates,x),t=(x-s.coordinates[edge])/(s.coordinates[edge+1]-s.coordinates[edge]);
    for(let k=0;k<3;k++)out[k]=s.origin[k]+(1-t)*s.positions[edge][k]+t*s.positions[edge+1][k];
    return out;
}

/** Independent shared-axis provider for World's existing whole-step contract.
 * A coroutine yields between global solves. Native render/measurement buffers
 * are published only after the complete requested timestep has been accepted.
 */
export function createSharedAxisAppSystem({readTools,readSheath,readBloodFlow=()=>null,readFrozenBloodFlow=()=>undefined,workSliceMs=4,adaptiveMesh=null,reuseFrictionAssembly=null,wasmLinearAssembly=true,lazyBasisCoefficients=false,wasmBasis=false,deferActiveBasis=false,earlyPredictorFallback=false,modifiedNewton=false,coupledFrictionNewton=false,predictiveNewton=true,onRejectedStep=null,retainDiscoveryCertificates=true,continuousDiscoverySign=false,certifiedDiscoverySamples=false,continuousSegmentContacts=false,projectiveDynamics=false,pruneInactiveWitnesses=false,contactCandidateMargin,physicsOptions=SHARED_AXIS_APP_PHYSICS_OPTIONS}) {
    adaptiveMesh=adaptiveMeshOptions(adaptiveMesh);
    modifiedNewton=modifiedNewton&&!projectiveDynamics;
    coupledFrictionNewton=coupledFrictionNewton&&!projectiveDynamics;
    // A friction chart refresh changes the wall law, not the accepted pose.
    // Reuse its owned mechanical residual; callers can still opt out.
    reuseFrictionAssembly??=coupledFrictionNewton;
    predictiveNewton=predictiveNewton&&coupledFrictionNewton;
    pruneInactiveWitnesses=pruneInactiveWitnesses&&!projectiveDynamics;
    physicsOptions={velocityPredictor:predictiveNewton?1:0,zeroDualStart:predictiveNewton,...physicsOptions,reuseFrictionAssembly,wasmLinearAssembly,lazyBasisCoefficients,wasmBasis,deferActiveBasis,earlyPredictorFallback,modifiedNewton,coupledFrictionNewton,projectiveDynamics,pruneInactiveWitnesses,contactCandidateMargin};
    if(adaptiveMesh)physicsOptions={...ADAPTIVE_SOLVE_OPTIONS,...physicsOptions};
    let state=null,pending=null,rotations={},sleepFrames=0,lastKey=null,failedKey=null,failedResult=null;
    const publication=new Map();
    let lastFailure=null;
    let nextWorkSliceMs=null,recoveryMode=false;
    function recordFailure(entry,result,recovered=false) {
        const captureStarted=performance.now();
        const failure={id:(globalThis.crypto?.randomUUID?.()??`${Date.now()}-${Math.random().toString(36).slice(2)}`),capturedAt:new Date().toISOString(),acceptedSteps:diagnostics.acceptedSteps,recovered,solver:diagnostics.solver,result};
        // The accepted state is never solved in-place: feed creates private candidates.
        // Serialize once at rejection, not on every successful frame or cooperative yield.
        const captureTimes={};
        try {
            lastFailure={...captureSharedAxisReplay({...state,captureOverride:entry.capture,adaptiveMesh:entry.adaptiveMesh},entry.sheath,entry.graftSurface),
                stepRequest:structuredClone({dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools,
                    options:Object.fromEntries(Object.entries(entry.physicsOptions??physicsOptions).filter(([,v])=>typeof v!=='function'))}),
                failure:structuredClone(failure)};
            captureTimes.snapshot=performance.now()-captureStarted;
        } catch(error) {
            lastFailure={version:1,failure:{...failure,
                captureError:error.message},stepRequest:{dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools}};
        }
        // Archive observers cannot reject an otherwise valid physical step.
        try{const started=performance.now(),event=structuredClone(lastFailure);captureTimes.eventClone=performance.now()-started;
            const observerStarted=performance.now();onRejectedStep?.(event);captureTimes.observer=performance.now()-observerStarted;
        }catch{/* The panel reports storage errors separately. */}
        result.replayCaptureMs=(result.replayCaptureMs??0)+performance.now()-captureStarted;
        result.replayCaptureTimes=captureTimes;
    }
    const diagnostics={continuousSegmentContacts,reuseFrictionAssembly,wasmLinearAssembly,certifiedDiscoverySamples,lazyBasisCoefficients,wasmBasis,deferActiveBasis,earlyPredictorFallback,modifiedNewton,coupledFrictionNewton,predictiveNewton,projectiveDynamics,pruneInactiveWitnesses,contactCandidateMargin,initializations:0,acceptedSteps:0,pendingSlices:0,failedSteps:0,last:null,solver:projectiveDynamics?'shared-axis-projective':adaptiveMesh?'shared-axis-adaptive':'shared-axis',mesh:null};
    function publish(tools,dt) {
        diagnostics.mesh={nodes:state.coordinates.length,dofs:diagnostics.last?.pd?.dofs??state.layout.dofCount,constraintRows:state.definitions.length,
            minSpacing:Math.min(...state.coordinates.slice(1).map((x,i)=>x-state.coordinates[i])),
            maxSpacing:Math.max(...state.coordinates.slice(1).map((x,i)=>x-state.coordinates[i])),
            adaptive:!!state.adaptiveMesh,shapeTolerance:state.adaptiveMesh?.shapeTolerance??0,contactMargin:state.adaptiveMesh?.contactMargin??0,
            maxArcLoss:state.adaptiveMesh?.maxArcLoss??0,maxAllowedSpacing:state.adaptiveMesh?.maxSpacing??state.spacing};
        for(const input of tools) {
            const body=input.body,material=state.materials.find(m=>m.spec.id===input.id),coordinates=material.coordinates;
            const positions=new Float64Array(coordinates.length*3);
            coordinates.forEach((x,i)=>positions.set(sampleSharedAxisPosition(state,x),3*i));
            body.jointStateView={coordinates:Float64Array.from(coordinates),positions};
            const previous=publication.get(body),saved=new Float64Array(body.count*3),point=[0,0,0];
            for(let i=0;i<body.count;i++) {
                const x=input.nodeCoordinates[i];sampleSharedAxisPosition(state,x,point);saved.set(point,3*i);
                for(let k=0;k<3;k++) {
                    const axis=['x','y','z'][k],velocity=['velocityX','velocityY','velocityZ'][k];
                    body[axis][i]=point[k];if(body[velocity])body[velocity][i]=previous?(point[k]-previous[i*3+k])/dt:0;
                }
                if(i<body.segmentCount) {
                    const e=Math.min(material.last-1,edgeAt(state.coordinates,x));
                    for(const component of ['X','Y','Z','W'])body['orientation'+component][i]=material.body['orientation'+component][e];
                }
            }
            publication.set(body,saved);
        }
    }
    function* solve(world,dt,tools) {
        if(!state) {
            state=createSharedAxisNative({...createSharedAxisContacts({sheath:pending.sheath,contactField:world.contactField,localCoordinates:true,retainDiscoveryCertificates,continuousDiscoverySign,certifiedDiscoverySamples,continuousSegmentContacts,contactCandidateMargin}),
                maxBendAngle:Math.PI/4,adaptiveMesh:pending.adaptiveMesh, tools:tools.map(t=>({...profile(t),insertion:0}))});
            rotations=Object.fromEntries(tools.map(t=>[t.id,0]));pending.rotations={...rotations};diagnostics.initializations++;
        }
        // Capture before the first yield. Neither a UI change nor a network
        // advance can change the forcing of an in-flight/replayed timestep.
        if(pending.flowNetwork)pending.physicsOptions={...pending.physicsOptions,
            bloodFlow:captureBloodFlow(state,pending.flowNetwork,pending.sheathLength,Math.max(...tools.map(t=>t.insertion)))};
        pending.flowNetwork=null;
        // Snapshot the budget for this entire cooperative step. A UI change
        // may arrive between yields and must only affect the following step.
        const surface=pending.graftSurface;
        const wallSamples=state.wallSamples.filter(sample=>!sample.graftSurface&&!sample.graftCapture);
        // A selected large-bore delivery system uses its matching introducer.
        // Replace only the inlet sampler; preserve the accepted rod and wall history.
        const inlet=createSharedAxisContacts({sheath:pending.sheath,localCoordinates:true}).wallSamples[0];
        const inletIndex=wallSamples.findIndex(sample=>sample.sharedAxisSheath);
        if(inletIndex>=0)wallSamples[inletIndex]=inlet;
        const graft=surface?createStentGraftContacts(surface,state):null;
        if(graft)wallSamples.push(graft);
        if(pending.capture)wallSamples.push(createCapturePotential(pending.capture));
        const source = {...state,wallSamples,graftRevision:surface?.revision??0,graftRecovery:graft?.recovery,adaptiveMesh:pending.adaptiveMesh};
        return yield* advanceSharedAxis(source,{...rotations},dt,tools,pending.physicsOptions);
    }
    const system={id:diagnostics.solver,diagnostics,
        // A small owned view for sampling the next shared flow boundary. It
        // contains no matrices, contact history or live solver references.
        captureFlowSamplingState:()=>state?{coordinates:state.coordinates.slice(),
            positions:state.positions.map(p=>p.slice()),origin:state.origin.slice()}:null,
        restoreReplay(fixture,world) {
            if(pending)throw new Error('Cannot restore a replay during a prepared timestep');
            const tools=readTools(),request=fixture.stepRequest;
            if(!request||!(request.dt>0)||!Number.isFinite(request.dt)||!world.contactField)
                throw new RangeError('Replay requires its timestep and loaded contact field');
            for(const tool of tools) {
                const recorded=fixture.tools.find(t=>t.id===tool.id),current=profile(tool);
                if(!recorded||recorded.type!==tool.type||!Number.isFinite(tool.insertion)||Math.abs(recorded.insertion-tool.insertion)>1e-8||
                    !Number.isFinite(request.rotations[tool.id])||
                    ['shaftStiffness','tipStiffness','length','mass','radius','wallStaticFriction','wallKineticFriction'].some(key=>
                        recorded[key]!==undefined&&(!Number.isFinite(current[key])||Math.abs(recorded[key]-current[key])>1e-10)))
                    throw new RangeError('Replay controls do not match the recorded tools');
            }
            // Build and validate privately before discarding the current state.
            const restored=restoreSharedAxisReplay(fixture,world.contactField);
            system.reset();state={...restored,adaptiveMesh};rotations={...request.rotations};
            diagnostics.initializations=1;publish(tools,request.dt);
            return diagnostics.mesh;
        },
        getLastFailure:()=>lastFailure?structuredClone(lastFailure):null,
        retryFromLastAccepted() {
            if(pending||!state||!failedResult)return false;
            // Rejected candidates were private. Keep accepted positions, frames,
            // contacts and feed; discard momentum that would repeat the collision.
            state={...state,velocities:state.velocities.map(v=>v.map(()=>0)),
                angularVelocities:Object.fromEntries(Object.entries(state.angularVelocities).map(([id,values])=>[id,values.map(v=>v.map(()=>0))]))};
            failedKey=null;failedResult=null;lastKey=null;sleepFrames=0;recoveryMode=true;
            diagnostics.recoveryPending=true;
            return true;
        },
        setAdaptiveShapeTolerance(shapeTolerance) {
            if (!adaptiveMesh) return false;
            adaptiveMesh=adaptiveMeshOptions({...adaptiveMesh,shapeTolerance});
            return true;
        },
        setAdaptiveContactMargin(contactMargin) {
            if (!adaptiveMesh) return false;
            adaptiveMesh=adaptiveMeshOptions({...adaptiveMesh,contactMargin});
            return true;
        },
        setAdaptiveMaxArcLoss(maxArcLoss) {
            if (!adaptiveMesh) return false;
            adaptiveMesh=adaptiveMeshOptions({...adaptiveMesh,maxArcLoss});
            return true;
        },
        setAdaptiveMaxSpacing(maxSpacing) {
            if (!adaptiveMesh) return false;
            if (maxSpacing < (state?.spacing ?? 5)) throw new RangeError('Adaptive max spacing must cover the fine spacing');
            adaptiveMesh=adaptiveMeshOptions({...adaptiveMesh,maxSpacing});
            return true;
        },
        // A scheduling hint for one invocation, never a change to physical dt
        // or solver tolerances. An indivisible generator operation may overrun it.
        setWorkSliceBudget(milliseconds) {
            if(!Number.isFinite(milliseconds)||milliseconds<0)throw new RangeError('Invalid work slice budget');
            nextWorkSliceMs=milliseconds;
        },
        step(world,dt) {
            const sliceMs=nextWorkSliceMs??workSliceMs;nextWorkSliceMs=null;
            if(!Number.isFinite(dt)||dt<=0)throw new RangeError('Positive shared-axis timestep required');
            if(pending&&pending.dt!==dt)throw new RangeError('Pending shared-axis timestep cannot change');
            if(!world.contactField)return {accepted:false,dt,status:'geometry-not-ready'};
            if(!pending) {
                const capture=structuredClone(world.readStentGraftCapture?.()??null);
                const tools=readTools(),graftSurface=world.readStentGraftSurface?.();
                const sheath=structuredClone(readSheath()),xyz=p=>Array.isArray(p)?p:[p.x,p.y,p.z];
                const sheathLength=Math.hypot(...xyz(sheath.end).map((v,k)=>v-xyz(sheath.start)[k]));
                const flowNetwork=tools.some(t=>t.insertion>sheathLength)?readBloodFlow():null;
                const frozenBloodFlow=readFrozenBloodFlow();
                const key=JSON.stringify({bloodFlow:frozenBloodFlow?.time??flowNetwork?.time??null,capture,graftRevision:graftSurface?.revision??0,tools:tools.map(t=>({...profile(t),insertion:t.insertion,rotation:t.rotation})),adaptiveMesh});
                if(key===failedKey)return failedResult;
                if(state&&key===lastKey&&sleepFrames>=10)return {accepted:true,dt,status:'sleeping',diagnostics:{...diagnostics}};
                pending={iterator:solve(world,dt,tools),tools,dt,key,graftSurface,capture,adaptiveMesh,physicsOptions:recoveryMode?{...physicsOptions,velocityPredictor:0,zeroDualStart:true}:physicsOptions,started:performance.now(),cpuMs:0,
                    sheath,sheathLength,flowNetwork,rotations:{...rotations},
                    requestTools:tools.map(t=>({...profile(t),insertion:t.insertion,rotation:t.rotation}))};
                if(frozenBloodFlow!==undefined)pending.physicsOptions={...pending.physicsOptions,bloodFlow:structuredClone(frozenBloodFlow)};
            }
            const start=performance.now();
            do {
                let next;
                try { next=pending.iterator.next(); } catch(error) {
                    const entry=pending;failedKey=pending.key;pending=null;diagnostics.failedSteps++;
                    diagnostics.last={status:'shared-axis-error',error:error.message,stack:error.stack};
                    recordFailure(entry,diagnostics.last);
                    return failedResult={accepted:false,terminal:true,dt,status:'shared-axis-error',diagnostics:{...diagnostics}};
                }
                if(next.done) {
                    const entry=pending,{tools,key,started}=pending,cpuMs=pending.cpuMs+performance.now()-start;pending=null;diagnostics.last={...next.value.result,cpuMs,wallMs:performance.now()-started};
                    if(!next.value.state){failedKey=key;diagnostics.failedSteps++;recordFailure(entry,diagnostics.last);return failedResult={accepted:false,terminal:true,dt,status:diagnostics.last.status,diagnostics:{...diagnostics}};}
                    if(diagnostics.last.attempts?.some(attempt=>attempt.converged===false))recordFailure(entry,diagnostics.last,true);
                    // A graft publication failure must not leave a newer
                    // native state behind bodies/input rolled back by the app.
                    try {
                        entry.graftSurface?.commitContactPatches?.(next.value.state.wallSamples.find(s=>s.graftSurface)?.contactPatches??[]);
                    } catch(error) {
                        failedKey=key;diagnostics.failedSteps++;
                        diagnostics.last={...diagnostics.last,converged:false,status:'shared-axis-commit-error',error:error.message,stack:error.stack};
                        recordFailure(entry,diagnostics.last);
                        return failedResult={accepted:false,terminal:true,dt,status:diagnostics.last.status,diagnostics:{...diagnostics}};
                    }
                    failedKey=null;failedResult=null;
                    if(recoveryMode)diagnostics.recoveries=(diagnostics.recoveries??0)+1;
                    diagnostics.recoveryPending=false;
                    state=next.value.state;rotations=next.value.rotations;recoveryMode=false;
                    diagnostics.last.bloodFlow=state.bloodFlowDiagnostics??null;
                    diagnostics.last.graftCapture=structuredClone(state.graftCaptureStats??null);
                    const speed=Math.max(0,...state.velocities.flat().map(Math.abs),...Object.values(state.angularVelocities).flat(2).map(v=>Math.abs(v)*60));
                    sleepFrames=key===lastKey&&speed<1&&(!projectiveDynamics||diagnostics.last.pd?.localGlobalConverged)?sleepFrames+1:0;lastKey=key;
                    publish(tools,dt);diagnostics.acceptedSteps++;
                    const quality=diagnostics.last.quality;
                    if(quality){world.contactCount=quality.contacts;world.maxPenetration=world.settledMaxPenetration=quality.maxPenetration;
                        for(const t of tools)t.body.sharedAxisDiagnostics=quality.bodies.find(b=>b.id===t.id);}
                    return {accepted:true,dt,status:'converged',diagnostics:{...diagnostics}};
                }
            }while(performance.now()-start<sliceMs);
            pending.cpuMs+=performance.now()-start;diagnostics.pendingSlices++;return {accepted:false,pending:true,dt,status:'shared-axis-pending',diagnostics:{...diagnostics}};
        },
        reset() {
            nextWorkSliceMs=null;recoveryMode=false;diagnostics.recoveries=0;diagnostics.recoveryPending=false;
            pending?.iterator.return();pending=null;state=null;rotations={};sleepFrames=0;lastKey=null;failedKey=null;failedResult=null;
            for(const body of publication.keys()){body.jointStateView=null;body.sharedAxisDiagnostics=null;}publication.clear();
            diagnostics.initializations=diagnostics.acceptedSteps=diagnostics.pendingSlices=diagnostics.failedSteps=0;diagnostics.last=null;diagnostics.mesh=null;
        }
    };
    return system;
}

/** Feed and rotation subdivision is identical in the UI and anatomy replay. */
export function* advanceSharedAxis(starting,startingRotations,dt,tools,physicsOptions={}) {
    let last;const attempts=[];const totals={fallbackLinearizationReuses:0,predictorFallbacks:0,retainedDiscoveryTrials:0,coupledFrictionRefreshes:0,coupledFrictionFallbacks:0,iterations:0,factorizations:0,workingSetReuses:0,backtracks:0,geometryRestarts:0,frictionIterations:0,substepAttempts:0,wallNormalFallbacks:0,fullAssemblies:0,residualAssemblies:0,promotedAssemblies:0,modifiedAttempts:0,modifiedAccepted:0,modifiedFallbacks:0};
    const timings={assemblyMs:0,linearMs:0,frictionMs:0,tangentAssemblyMs:0,residualAssemblyMs:0,projectionMs:0};
    const result=()=>({...last,...totals,timings,attempts});
    let roundoffDetected=false,recenterAttempted=false;
    const detectRoundoff=physicsOptions.roundoffStagnation??(physicsOptions.coupledFrictionNewton===true);
    // Change the numerical origin or subdivide when Newton corrections hit
    // the arithmetic floor. If both fail, retain the complete original search.
    // A heuristic may save work, but may not remove a previously solvable case.
    for(const roundoffStagnation of detectRoundoff?[true,false]:[false]) {
        if(!roundoffStagnation&&detectRoundoff&&!roundoffDetected)break;
        for(const subdivisions of [1,2,4,8]) {
            // Feed copies the rod, but used to share discovery samplers. Give
            // every attempt private history too: rejection/cancellation and
            // even a failed publication leave the accepted checkpoint intact.
            let current={...starting,wallSamples:(starting.wallSamples??[]).map(sample=>sample.forkDiscovery?.({candidateMargin:physicsOptions.contactCandidateMargin})??sample)},
                currentRotations=startingRotations,failed=false;
            for(let index=1;index<=subdivisions;index++) {
                const fraction=index/subdivisions;
                const source={...current,materials:current.materials.map(m=>({...m,spec:{...m.spec,...profile(tools.find(t=>t.id===m.spec.id))}}))};
                const feeds=Object.fromEntries(tools.map(t=>{
                    const old=starting.materials.find(m=>m.spec.id===t.id).spec.insertion;
                    return [t.id,old+fraction*(t.insertion-old)];
                }));
                const nextRotations=Object.fromEntries(tools.map(t=>[t.id,startingRotations[t.id]+fraction*angleDifference(t.rotation,startingRotations[t.id])]));
                const candidate=feedSharedAxisNative(source,feeds,{pruneInactiveWitnesses:physicsOptions.pruneInactiveWitnesses===true,contactCandidateMargin:physicsOptions.contactCandidateMargin,omitInactiveSampleRows:physicsOptions.omitInactiveSampleRows});
                for(const t of tools)rotateSharedAxisNative(candidate,t.id,angleDifference(nextRotations[t.id],currentRotations[t.id]));
                const feedById=Object.fromEntries(current.materials.map(m=>[m.spec.id,feeds[m.spec.id]-m.spec.insertion]));
                last=yield* (physicsOptions.projectiveDynamics?iterateSharedAxisProjective:iterateSharedAxisTimeStep)(candidate,dt/subdivisions,{...physicsOptions,roundoffStagnation,feedById});
                roundoffDetected ||= last.status==='roundoff-stagnation';
                attempts.push({subdivisions,index,roundoffStagnation,dt:dt/subdivisions,status:last.status,error:last.error,converged:last.converged,
                    residual:last.residual,friction:last.friction,wallNormalFallback:last.wallNormalFallback,
                    ...(last.detectedRoundoffStagnation?{detectedRoundoffStagnation:last.detectedRoundoffStagnation}:{}),
                    iterations:last.iterations,factorizations:last.factorizations});
                totals.substepAttempts++;for(const key of Object.keys(totals))if(key!=='substepAttempts')
                    totals[key]+=last[key]??(key==='wallNormalFallbacks'?Number(last.wallNormalFallback?.attempted===true):0);
                for(const key of Object.keys(timings))timings[key]+=last.timings[key]??0;
                if(last.status==='roundoff-stagnation'&&!recenterAttempted&&physicsOptions.roundoffRecenter!==false) {
                    recenterAttempted=true;
                    const local=recenterShortInterval(starting,candidate);
                    if(local) {
                        // Retry from the SAME accepted world-space pose with a
                        // better numerical origin near the short material cell.
                        // Translation changes neither feed, frames nor forces.
                        const recovered=yield* advanceSharedAxis(local,startingRotations,dt,tools,
                            {...physicsOptions,roundoffRecenter:false,contactRecoveryFallback:false});
                        last=recovered.result;attempts.push(...last.attempts);
                        for(const key of Object.keys(totals))totals[key]+=last[key]??0;
                        for(const key of Object.keys(timings))timings[key]+=last.timings[key]??0;
                        if(recovered.state)return {...recovered,result:{...result(),recenteredRoundoff:true}};
                    }
                }
                if(!last.converged){failed=true;break;}
                current=candidate;currentRotations=nextRotations;
            }
            if(!failed)return {state:current,rotations:currentRotations,result:{...result(),subdivisions}};
        }
    }
    if(last.status==='linear-solve'&&!physicsOptions.projectiveDynamics&&
        physicsOptions.recoverNegativeContact===undefined&&physicsOptions.contactRecoveryFallback!==false) {
        // Retain the ordinary complete-step/subdivision path exactly. Earlier
        // contact release can change which substeps succeed and thus change
        // time integration even when both results meet the force certificate.
        // Recover only a step that would otherwise be rejected in its entirety,
        // always from the original accepted checkpoint and full requested dt.
        const originalFailure=last.status;
        const recovered=yield* advanceSharedAxis(starting,startingRotations,dt,tools,
            {...physicsOptions,recoverNegativeContact:true});
        last=recovered.result;
        attempts.push(...last.attempts.map(a=>({...a,negativeContactRecovery:true})));
        for(const key of Object.keys(totals))totals[key]+=last[key]??0;
        for(const key of Object.keys(timings))timings[key]+=last.timings[key]??0;
        return {...recovered,result:{...result(),negativeContactRecovery:{attempted:true,originalFailure,converged:!!recovered.state}}};
    }
    return {result:result()};
}

function recenterShortInterval(starting,candidate) {
    // An unmatched material endpoint can leave a short cell far from the
    // sheath origin even when the two tips are nowhere near each other.
    // Recenter only a stalled private attempt, without snapping the mesh.
    if(!starting.rebaseNearTips)return null;
    let edge=-1,length=starting.spacing*.5;
    for(let i=0;i+1<candidate.coordinates.length;i++) {
        const span=candidate.coordinates[i+1]-candidate.coordinates[i];
        if(span<length){length=span;edge=i;}
    }
    if(edge<0)return null;
    const shift=candidate.positions[edge].map((v,k)=>(v+candidate.positions[edge+1][k])*.5+
        candidate.origin[k]-starting.origin[k]);
    if(Math.hypot(...shift)<starting.spacing)return null;
    return {...starting,origin:starting.origin.map((v,k)=>v+shift[k]),
        positions:starting.positions.map(p=>p.map((v,k)=>v-shift[k]))};
}
