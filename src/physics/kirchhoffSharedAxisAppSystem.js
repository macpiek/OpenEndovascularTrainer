import {captureSharedAxisReplay} from './kirchhoffSharedAxisReplay.js';
import {createSharedAxisContacts} from './kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,feedSharedAxisNative,rotateSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {iterateSharedAxisTimeStep} from './kirchhoffSharedAxisTimeStep.js';

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
export function createSharedAxisAppSystem({readTools,readSheath,workSliceMs=4,physicsOptions={liveWallNormalLoad:true,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback:true}}) {
    let state=null,pending=null,rotations={},sleepFrames=0,lastKey=null,failedKey=null,failedResult=null;
    const publication=new Map();
    let lastFailure=null;
    function recordFailure(entry,result) {
        // The accepted state is never solved in-place: feed creates private candidates.
        // Serialize once at rejection, not on every successful frame or cooperative yield.
        try {
            lastFailure={...captureSharedAxisReplay(state,entry.sheath),
                stepRequest:structuredClone({dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools,
                    options:Object.fromEntries(Object.entries(physicsOptions).filter(([,v])=>typeof v!=='function'))}),
                failure:structuredClone({capturedAt:new Date().toISOString(),acceptedSteps:diagnostics.acceptedSteps,result})};
        } catch(error) {
            lastFailure={version:1,failure:{capturedAt:new Date().toISOString(),result,
                captureError:error.message},stepRequest:{dt:entry.dt,rotations:entry.rotations,tools:entry.requestTools}};
        }
    }
    const diagnostics={initializations:0,acceptedSteps:0,pendingSlices:0,failedSteps:0,last:null,solver:'shared-axis'};
    function publish(tools,dt) {
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
            state=createSharedAxisNative({...createSharedAxisContacts({sheath:pending.sheath,contactField:world.contactField,localCoordinates:true}),
                maxBendAngle:Math.PI/4, tools:tools.map(t=>({...profile(t),insertion:0}))});
            rotations=Object.fromEntries(tools.map(t=>[t.id,0]));pending.rotations={...rotations};diagnostics.initializations++;
        }
        return yield* advanceSharedAxis(state,{...rotations},dt,tools,physicsOptions);
    }
    const system={id:'shared-axis',diagnostics,
        getLastFailure:()=>lastFailure?structuredClone(lastFailure):null,
        step(world,dt) {
            if(!Number.isFinite(dt)||dt<=0)throw new RangeError('Positive shared-axis timestep required');
            if(pending&&pending.dt!==dt)throw new RangeError('Pending shared-axis timestep cannot change');
            if(!world.contactField)return {accepted:false,dt,status:'geometry-not-ready'};
            if(!pending) {
                const tools=readTools(),key=JSON.stringify(tools.map(t=>({...profile(t),insertion:t.insertion,rotation:t.rotation})));
                if(key===failedKey)return failedResult;
                if(state&&key===lastKey&&sleepFrames>=10)return {accepted:true,dt,status:'sleeping',diagnostics:{...diagnostics}};
                pending={iterator:solve(world,dt,tools),tools,dt,key,started:performance.now(),cpuMs:0,
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
                    failedKey=null;failedResult=null;
                    state=next.value.state;rotations=next.value.rotations;
                    const speed=Math.max(0,...state.velocities.flat().map(Math.abs),...Object.values(state.angularVelocities).flat(2).map(v=>Math.abs(v)*60));
                    sleepFrames=key===lastKey&&speed<1?sleepFrames+1:0;lastKey=key;
                    publish(tools,dt);diagnostics.acceptedSteps++;
                    const quality=diagnostics.last.quality;
                    if(quality){world.contactCount=quality.contacts;world.maxPenetration=world.settledMaxPenetration=quality.maxPenetration;
                        for(const t of tools)t.body.sharedAxisDiagnostics=quality.bodies.find(b=>b.id===t.id);}
                    return {accepted:true,dt,status:'converged',diagnostics:{...diagnostics}};
                }
            }while(performance.now()-start<workSliceMs);
            pending.cpuMs+=performance.now()-start;diagnostics.pendingSlices++;return {accepted:false,pending:true,dt,status:'shared-axis-pending',diagnostics:{...diagnostics}};
        },
        reset() {
            pending?.iterator.return();pending=null;state=null;rotations={};sleepFrames=0;lastKey=null;failedKey=null;failedResult=null;
            for(const body of publication.keys()){body.jointStateView=null;body.sharedAxisDiagnostics=null;}publication.clear();
            diagnostics.initializations=diagnostics.acceptedSteps=diagnostics.pendingSlices=diagnostics.failedSteps=0;diagnostics.last=null;
        }
    };
    return system;
}

/** Feed and rotation subdivision is identical in the UI and anatomy replay. */
export function* advanceSharedAxis(starting,startingRotations,dt,tools,physicsOptions={}) {
    let last;const attempts=[];const totals={iterations:0,factorizations:0,workingSetReuses:0,backtracks:0,geometryRestarts:0,frictionIterations:0,substepAttempts:0,wallNormalFallbacks:0,fullAssemblies:0,residualAssemblies:0,promotedAssemblies:0};
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
                const candidate=feedSharedAxisNative(source,feeds);
                for(const t of tools)rotateSharedAxisNative(candidate,t.id,angleDifference(nextRotations[t.id],currentRotations[t.id]));
                const feedById=Object.fromEntries(current.materials.map(m=>[m.spec.id,feeds[m.spec.id]-m.spec.insertion]));
                last=yield* iterateSharedAxisTimeStep(candidate,dt/subdivisions,{...physicsOptions,feedById});
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
