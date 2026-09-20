import {captureSharedAxisReplay} from './kirchhoffSharedAxisReplay.js';
import {createSharedAxisContacts} from './kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,feedSharedAxisNative,rotateSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {iterateSharedAxisProjective} from './kirchhoffSharedAxisProjective.js';
import {iterateSharedAxisTimeStep} from './kirchhoffSharedAxisTimeStep.js';
import {adaptiveMeshOptions,ADAPTIVE_SOLVE_OPTIONS} from './kirchhoffSharedAxisAdaptiveMesh.js';

const angleDifference=(a,b)=>Math.atan2(Math.sin(a-b),Math.cos(a-b));
const profile=t=>({id:t.id,type:t.type,shaftStiffness:t.shaftStiffness,tipStiffness:t.tipStiffness,length:t.length??1000,mass:t.mass??t.body?.mass,radius:t.radius??t.body?.radius,
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
export function createSharedAxisAppSystem({readTools,readSheath,workSliceMs=4,adaptiveMesh=null,reuseFrictionAssembly=false,wasmLinearAssembly=false,lazyBasisCoefficients=false,deferActiveBasis=false,earlyPredictorFallback=false,modifiedNewton=false,coupledFrictionNewton=false,predictiveNewton=true,onRejectedStep=null,retainDiscoveryCertificates=false,continuousDiscoverySign=false,certifiedDiscoverySamples=false,continuousSegmentContacts=false,projectiveDynamics=false,pruneInactiveWitnesses=false,physicsOptions={liveWallNormalLoad:true,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback:true,wasmMaterial:true,reuseMaterialScratch:true,lightweightFriction:true,reuseTriangleKernel:true,earlyContactPreflight:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true}}) {
    adaptiveMesh=adaptiveMeshOptions(adaptiveMesh);
    modifiedNewton=modifiedNewton&&!projectiveDynamics;
    coupledFrictionNewton=coupledFrictionNewton&&!projectiveDynamics;
    predictiveNewton=predictiveNewton&&coupledFrictionNewton;
    pruneInactiveWitnesses=pruneInactiveWitnesses&&!projectiveDynamics;
    physicsOptions={velocityPredictor:predictiveNewton?1:0,zeroDualStart:predictiveNewton,...physicsOptions,reuseFrictionAssembly,wasmLinearAssembly,lazyBasisCoefficients,deferActiveBasis,earlyPredictorFallback,modifiedNewton,coupledFrictionNewton,projectiveDynamics,pruneInactiveWitnesses};
    if(adaptiveMesh)physicsOptions={...ADAPTIVE_SOLVE_OPTIONS,...physicsOptions};
    let state=null,pending=null,rotations={},sleepFrames=0,lastKey=null,failedKey=null,failedResult=null;
    const publication=new Map();
    let lastFailure=null;
    let nextWorkSliceMs=null;
    function recordFailure(entry,result,recovered=false) {
        const failure={id:(globalThis.crypto?.randomUUID?.()??`${Date.now()}-${Math.random().toString(36).slice(2)}`),capturedAt:new Date().toISOString(),acceptedSteps:diagnostics.acceptedSteps,recovered,solver:diagnostics.solver,result};
        // The accepted state is never solved in-place: feed creates private candidates.
        // Serialize once at rejection, not on every successful frame or cooperative yield.
        try {
            lastFailure={...captureSharedAxisReplay({...state,adaptiveMesh:entry.adaptiveMesh},entry.sheath),
                stepRequest:structuredClone({dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools,
                    options:Object.fromEntries(Object.entries(physicsOptions).filter(([,v])=>typeof v!=='function'))}),
                failure:structuredClone(failure)};
        } catch(error) {
            lastFailure={version:1,failure:{...failure,
                captureError:error.message},stepRequest:{dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools}};
        }
        // Archive observers cannot reject an otherwise valid physical step.
        try{onRejectedStep?.(structuredClone(lastFailure));}catch{/* The panel reports storage errors separately. */}
    }
    const diagnostics={continuousSegmentContacts,reuseFrictionAssembly,wasmLinearAssembly,certifiedDiscoverySamples,lazyBasisCoefficients,deferActiveBasis,earlyPredictorFallback,modifiedNewton,coupledFrictionNewton,predictiveNewton,projectiveDynamics,pruneInactiveWitnesses,initializations:0,acceptedSteps:0,pendingSlices:0,failedSteps:0,last:null,solver:projectiveDynamics?'shared-axis-projective':adaptiveMesh?'shared-axis-adaptive':'shared-axis',mesh:null};
    function publish(tools,dt) {
        diagnostics.mesh={nodes:state.coordinates.length,dofs:diagnostics.last?.pd?.dofs??state.layout.dofCount,
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
            state=createSharedAxisNative({...createSharedAxisContacts({sheath:pending.sheath,contactField:world.contactField,localCoordinates:true,retainDiscoveryCertificates,continuousDiscoverySign,certifiedDiscoverySamples,continuousSegmentContacts}),
                maxBendAngle:Math.PI/4,adaptiveMesh:pending.adaptiveMesh, tools:tools.map(t=>({...profile(t),insertion:0}))});
            rotations=Object.fromEntries(tools.map(t=>[t.id,0]));pending.rotations={...rotations};diagnostics.initializations++;
        }
        // Snapshot the budget for this entire cooperative step. A UI change
        // may arrive between yields and must only affect the following step.
        const source = {...state,adaptiveMesh:pending.adaptiveMesh};
        return yield* advanceSharedAxis(source,{...rotations},dt,tools,physicsOptions);
    }
    const system={id:diagnostics.solver,diagnostics,
        getLastFailure:()=>lastFailure?structuredClone(lastFailure):null,
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
                const tools=readTools(),key=JSON.stringify({tools:tools.map(t=>({...profile(t),insertion:t.insertion,rotation:t.rotation})),adaptiveMesh});
                if(key===failedKey)return failedResult;
                if(state&&key===lastKey&&sleepFrames>=10)return {accepted:true,dt,status:'sleeping',diagnostics:{...diagnostics}};
                pending={iterator:solve(world,dt,tools),tools,dt,key,adaptiveMesh,started:performance.now(),cpuMs:0,
                    sheath:structuredClone(readSheath()),rotations:{...rotations},
                    requestTools:tools.map(t=>({...profile(t),insertion:t.insertion,rotation:t.rotation}))};
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
                    failedKey=null;failedResult=null;
                    state=next.value.state;rotations=next.value.rotations;
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
            nextWorkSliceMs=null;
            pending?.iterator.return();pending=null;state=null;rotations={};sleepFrames=0;lastKey=null;failedKey=null;failedResult=null;
            for(const body of publication.keys()){body.jointStateView=null;body.sharedAxisDiagnostics=null;}publication.clear();
            diagnostics.initializations=diagnostics.acceptedSteps=diagnostics.pendingSlices=diagnostics.failedSteps=0;diagnostics.last=null;diagnostics.mesh=null;
        }
    };
    return system;
}

/** Feed and rotation subdivision is identical in the UI and anatomy replay. */
export function* advanceSharedAxis(starting,startingRotations,dt,tools,physicsOptions={}) {
    let last;const attempts=[];const totals={predictorFallbacks:0,retainedDiscoveryTrials:0,coupledFrictionRefreshes:0,coupledFrictionFallbacks:0,iterations:0,factorizations:0,workingSetReuses:0,backtracks:0,geometryRestarts:0,frictionIterations:0,substepAttempts:0,wallNormalFallbacks:0,fullAssemblies:0,residualAssemblies:0,promotedAssemblies:0,modifiedAttempts:0,modifiedAccepted:0,modifiedFallbacks:0};
    const timings={assemblyMs:0,linearMs:0,frictionMs:0,tangentAssemblyMs:0,residualAssemblyMs:0,projectionMs:0};
    const result=()=>({...last,...totals,timings,attempts});
        for(const subdivisions of [1,2,4,8]) {
            let current=starting,currentRotations=startingRotations,failed=false;
            for(let index=1;index<=subdivisions;index++) {
                const fraction=index/subdivisions;
                const source={...current,materials:current.materials.map(m=>({...m,spec:{...m.spec,...profile(tools.find(t=>t.id===m.spec.id))}}))};
                const feeds=Object.fromEntries(tools.map(t=>{
                    const old=starting.materials.find(m=>m.spec.id===t.id).spec.insertion;
                    return [t.id,old+fraction*(t.insertion-old)];
                }));
                const nextRotations=Object.fromEntries(tools.map(t=>[t.id,startingRotations[t.id]+fraction*angleDifference(t.rotation,startingRotations[t.id])]));
                const candidate=feedSharedAxisNative(source,feeds,{pruneInactiveWitnesses:physicsOptions.pruneInactiveWitnesses===true});
                for(const t of tools)rotateSharedAxisNative(candidate,t.id,angleDifference(nextRotations[t.id],currentRotations[t.id]));
                const feedById=Object.fromEntries(current.materials.map(m=>[m.spec.id,feeds[m.spec.id]-m.spec.insertion]));
                last=yield* (physicsOptions.projectiveDynamics?iterateSharedAxisProjective:iterateSharedAxisTimeStep)(candidate,dt/subdivisions,{...physicsOptions,feedById});
                attempts.push({subdivisions,index,dt:dt/subdivisions,status:last.status,error:last.error,converged:last.converged,
                    residual:last.residual,friction:last.friction,wallNormalFallback:last.wallNormalFallback,
                    iterations:last.iterations,factorizations:last.factorizations});
                totals.substepAttempts++;for(const key of Object.keys(totals))if(key!=='substepAttempts')
                    totals[key]+=last[key]??(key==='wallNormalFallbacks'?Number(last.wallNormalFallback?.attempted===true):0);
                for(const key of Object.keys(timings))timings[key]+=last.timings[key]??0;
                if(!last.converged){failed=true;break;}
                current=candidate;currentRotations=nextRotations;
            }
            if(!failed)return {state:current,rotations:currentRotations,result:{...result(),subdivisions}};
        }
        return {result:result()};
}
