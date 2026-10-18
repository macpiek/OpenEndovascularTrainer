import {createSharedAxisAppSystem} from './kirchhoffSharedAxisAppSystem.js';
import {createSharedAxisNative} from './kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from './kirchhoffSharedAxisContacts.js';
import {captureBloodFlow} from './bloodFlowCoupling.js';
import {createSharedAxisWorkerRpc} from './kirchhoffSharedAxisWorkerRpc.js';
import {sharedAxisFieldMessage,sharedAxisToolMessage,publishSharedAxisWorkerBodies} from './kirchhoffSharedAxisWorkerData.js';

/** Opt-in worker prototype. The ordinary simulator keeps its existing provider.
 * Only step() publishes a ready reply, under the same World/common-clock barrier.
 * Message handlers merely signal readiness; they never mutate visible tools.
 * Graft publication needs a two-phase commit and is deliberately unsupported
 * until that transaction is implemented and tested, rather than ignored. */
export function createSharedAxisWorkerSystem({readTools,readSheath,readBloodFlow=()=>null,onRejectedStep,onReady=()=>{},
    createWorker=()=>new Worker(new URL('../workers/sharedAxisPhysics.worker.js',import.meta.url),{type:'module'}),...options}) {
    const diagnostics={solver:options.adaptiveMesh?'shared-axis-adaptive':'shared-axis',worker:true,initializations:0,
        acceptedSteps:0,pendingSlices:0,failedSteps:0,last:null,mesh:null};
    let rpc,fieldRef,ready=Promise.resolve(),pending=null,flowState=null,disposed=false,lastFailure=null,
        sleepingKey=null,sleepingResult=null,fatal=null,readySettled=false;
    const published=new Set();
    const setReady=promise=>{
        ready=promise;readySettled=false;
        promise.then(()=>{if(ready===promise)readySettled=true;},()=>{});
    };
    function ensure(world) {
        if(disposed)throw new Error('Worker provider is disposed');
        if(rpc&&fieldRef===world.contactField)return;
        rpc?.dispose();pending=null;flowState=null;sleepingKey=null;sleepingResult=null;
        rpc=createSharedAxisWorkerRpc(createWorker());fieldRef=world.contactField;
        setReady(rpc.request('initialize',{field:sharedAxisFieldMessage(fieldRef),options}));
    }
    function requireSupported(world,fixture=null) {
        if(world.readStentGraftSurface?.()||world.readStentGraftCapture?.()||fixture?.graftContactSurface||fixture?.graftCapture)
            throw new Error('Worker prototype does not yet support transactional stent-graft publication');
    }
    function publication(world,tools,packet) {
        publishSharedAxisWorkerBodies(tools,packet.bodies);
        Object.assign(diagnostics,packet.diagnostics,{worker:true});flowState=packet.flowState;
        world.contactCount=packet.contactCount;world.maxPenetration=packet.maxPenetration;
        world.settledMaxPenetration=packet.settledMaxPenetration;
        for(const tool of tools)published.add(tool.body);
        for(const failure of packet.failures??[]) {
            lastFailure=failure;
            try{onRejectedStep?.(structuredClone(failure));}catch{/* Observers cannot reject a physical step. */}
        }
    }
    function sendStep(world,dt) {
        requireSupported(world);
        const tools=readTools(),sheath=structuredClone(readSheath()),messages=tools.map(sharedAxisToolMessage);
        const xyz=p=>Array.isArray(p)?p:[p.x,p.y,p.z];
        const length=Math.hypot(...xyz(sheath.end).map((v,k)=>v-xyz(sheath.start)[k]));
        if(!flowState) {
            const empty=createSharedAxisNative({...createSharedAxisContacts({sheath,localCoordinates:true}),
                tools:messages.map(t=>({...t,...t.body,body:undefined,insertion:0})),adaptiveMesh:options.adaptiveMesh});
            flowState={coordinates:empty.coordinates,positions:empty.positions,origin:empty.origin};
        }
        const network=tools.some(t=>t.insertion>length)?readBloodFlow():null;
        const flow=captureBloodFlow(flowState,network,length,Math.max(...tools.map(t=>t.insertion)));
        const key=JSON.stringify({tools:messages,sheath,flowTime:flow?.time??null});
        if(sleepingResult&&key===sleepingKey)return {...sleepingResult,dt,diagnostics:{...diagnostics}};
        const entry={dt,tools,key,started:performance.now(),packet:null,error:null,epoch:rpc.epoch};pending=entry;
        const dispatch=()=>{
            if(pending!==entry||entry.epoch!==rpc.epoch)throw Object.assign(new Error('Cancelled'),{name:'AbortError'});
            entry.dispatched=performance.now();
            return rpc.request('step',{dt,tools:messages,sheath,flow});
        };
        // Enqueue before this task renders: the worker can solve while the
        // main thread paints, rather than starting in its later microtask.
        (readySettled?dispatch():ready.then(dispatch)).then(packet=>{
            if(pending!==entry||entry.epoch!==rpc.epoch)return;
            entry.packet=packet;entry.readyAt=performance.now();onReady();
        },error=>{
            if(pending!==entry||error.name==='AbortError')return;
            entry.error=error;onReady();
        });
        return null;
    }
    const system={id:diagnostics.solver,diagnostics,
        whenReady:()=>ready,
        get waitingForWorker(){return !!pending&&!pending.packet&&!pending.error;},
        captureFlowSamplingState:()=>flowState?structuredClone(flowState):null,
        getLastFailure:()=>lastFailure?structuredClone(lastFailure):null,
        setWorkSliceBudget(milliseconds){if(!Number.isFinite(milliseconds)||milliseconds<0)throw new RangeError('Invalid work slice budget');},
        step(world,dt) {
            if(!Number.isFinite(dt)||dt<=0)throw new RangeError('Positive shared-axis timestep required');
            if(pending&&pending.dt!==dt)throw new RangeError('Pending shared-axis timestep cannot change');
            if(!world.contactField)return {accepted:false,dt,status:'geometry-not-ready'};
            ensure(world);
            if(fatal)return {accepted:false,terminal:true,dt,status:'shared-axis-worker-error',error:fatal};
            if(!pending){const asleep=sendStep(world,dt);if(asleep)return asleep;}
            const entry=pending;
            if(entry.error){fatal=entry.error;pending=null;diagnostics.failedSteps++;return {accepted:false,terminal:true,dt,status:'shared-axis-worker-error',error:fatal};}
            if(!entry.packet){diagnostics.pendingSlices++;return {accepted:false,pending:true,dt,status:'shared-axis-pending',diagnostics:{...diagnostics}};}
            pending=null;const packet=entry.packet,result=packet.result;
            if(result.accepted) {
                publication(world,entry.tools,packet);
                if(diagnostics.last)Object.assign(diagnostics.last,{
                    workerWallMs:performance.now()-entry.started,
                    workerDispatchDelayMs:entry.dispatched-entry.started,
                    workerResponseMs:entry.readyAt-entry.dispatched,
                    workerPublicationDelayMs:performance.now()-entry.readyAt
                });
                sleepingKey=entry.key;sleepingResult=result.status==='sleeping'?result:null;
            }else {
                Object.assign(diagnostics,packet.diagnostics,{worker:true});
                for(const failure of packet.failures??[]){lastFailure=failure;try{onRejectedStep?.(structuredClone(failure));}catch{}}
            }
            return {...result,diagnostics:{...diagnostics}};
        },
        restoreReplay(fixture,world) {
            if(pending)throw new Error('Cannot restore a replay during a prepared timestep');
            requireSupported(world,fixture);ensure(world);
            // Publish the recorded pose synchronously, exactly like the local
            // provider. Its private solver is discarded after this one restore.
            const local=createSharedAxisAppSystem({readTools,readSheath,...options});
            const mesh=local.restoreReplay(fixture,world);flowState=local.captureFlowSamplingState();
            Object.assign(diagnostics,local.diagnostics,{worker:true});
            const tools=readTools();for(const t of tools)published.add(t.body);
            const data={fixture,tools:tools.map(sharedAxisToolMessage),sheath:structuredClone(readSheath())};
            sleepingKey=null;sleepingResult=null;fatal=null;
            setReady(ready.then(()=>rpc.request('restore',data)));
            return mesh;
        },
        retryFromLastAccepted() {
            if(fatal||pending||!rpc||!diagnostics.failedSteps)return false;
            fatal=null;sleepingKey=null;sleepingResult=null;
            setReady(ready.then(()=>rpc.request('retry',{})));return true;
        },
        reset() {
            pending=null;flowState=null;sleepingKey=null;sleepingResult=null;fatal=null;
            if(rpc)setReady(rpc.reset());
            for(const body of published){body.jointStateView=null;body.sharedAxisDiagnostics=null;}published.clear();
            diagnostics.initializations=diagnostics.acceptedSteps=diagnostics.pendingSlices=diagnostics.failedSteps=0;
            diagnostics.last=null;diagnostics.mesh=null;
        },
        dispose(){if(disposed)return;system.reset();disposed=true;rpc?.dispose();rpc=null;}
    };
    for(const method of ['setAdaptiveShapeTolerance','setAdaptiveContactMargin','setAdaptiveMaxArcLoss','setAdaptiveMaxSpacing'])
        system[method]=value=>{
            if(!options.adaptiveMesh)return false;
            if(!Number.isFinite(value)||value<0)throw new RangeError('Invalid adaptive mesh setting');
            const key={setAdaptiveShapeTolerance:'shapeTolerance',setAdaptiveContactMargin:'contactMargin',setAdaptiveMaxArcLoss:'maxArcLoss',setAdaptiveMaxSpacing:'maxSpacing'}[method];
            options.adaptiveMesh={...options.adaptiveMesh,[key]:value};
            if(rpc)setReady(ready.then(()=>rpc.request('setting',{method,value})));return true;
        };
    return system;
}
