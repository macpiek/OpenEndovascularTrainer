/** Application-side owner of one prepared physical dt. The application
 * owns wall-time backlog; World owns only this current dt. Never add those
 * two accumulators together or feed the entire application backlog to World.
 * This adapter does not change any solver acceptance or numerical limit. */
export function createFixedStepTransaction({world,prepare,beforePrepare=()=>{},now=()=>performance.now(),recovery=null}) {
    let pending=null,frame=0,lastRejectedFrame=-1,epoch=0,disposed=false;
    let blocked=null,changeRevision=0;
    const deferred=new Map();
    const recoveryKey=()=>JSON.stringify([recovery?.readKey(),changeRevision]);
    const canAttempt=()=>!disposed&&lastRejectedFrame!==frame&&(!blocked||blocked.key!==recoveryKey());
    function flushChanges() {
        if(pending)throw new Error('Prepared physics inputs cannot be replaced');
        for(const [key,apply] of deferred) {deferred.delete(key);apply();}
    }
    return {
        get pending(){return pending!==null;},get epoch(){return epoch;},get frame(){return frame;},
        get disposed(){return disposed;},canAttempt,
        get blocked(){return blocked!==null;},
        beginFrame(){if(!disposed)frame++;},
        blockCurrentFrame(){lastRejectedFrame=frame;},
        change(key,apply) {
            if(disposed)return;
            changeRevision++;
            if(pending)deferred.set(key,apply);else apply();
        },
        flushChanges,
        // Caller resets World's physical state/debt as part of the same
        // explicit lifecycle action. Retaining application backlog is allowed.
        reset(){pending=null;blocked=null;lastRejectedFrame=-1;epoch++;},
        dispose(){disposed=true;pending=null;blocked=null;deferred.clear();epoch++;},
        attempt(dt) {
            if(!canAttempt())return{accepted:false,pending:false,attempted:false,status:disposed?'disposed':blocked?'awaiting-input':'frame-blocked',durationMs:0};
            const start=now();let accepted=false,cooperativePending=false,context=null,error=null,status,terminal=false;
            try {
                if(!Number.isFinite(dt)||dt<=0||world.fixedDt!==dt)throw new Error('A prepared timestep must retain the World fixed dt');
                if(!pending) {
                    // Lifecycle transitions must precede advance: resetting
                    // World from its callback would erase the newly added dt.
                    beforePrepare();flushChanges();
                    if(Math.abs(world.accumulator)>1e-9)throw new Error('World must own only the application current timestep');
                    blocked=null;
                    pending={dt,context:null,queued:false,prepared:false,preparationFailed:false,recoveryKey:recovery?recoveryKey():null};
                }
                const entry=pending;
                if(entry.dt!==dt)throw new Error('A pending timestep must retain its dt');
                if(entry.preparationFailed)throw new Error('Input preparation failed; reset is required before further physical work');
                const elapsed=entry.queued?0:dt;
                entry.queued=true;
                const before=world.stepCount;
                const committed=world.advance(elapsed,()=>{
                    if(entry.prepared)throw new Error('World repeated preparation of a pending timestep');
                    try {entry.context=prepare(dt);entry.prepared=true;}
                    catch(caught){entry.preparationFailed=true;throw caught;}
                });
                if(committed!==0&&committed!==1)throw new Error('A single application attempt must execute at most one physical timestep');
                if(world.stepCount-before!==committed)throw new Error('World execution and acceptance counters disagree');
                accepted=committed===1;
                status=accepted?'accepted':world.lastStepResult?.status??'rejected';
                if(accepted) {
                    context=entry.context;
                    // Consume BEFORE any application accounting/presentation.
                    // A fallible UI update must never replay this solved dt.
                    pending=null;
                } else {
                    // An explicit cooperative yield retains the prepared dt
                    // without declaring a numerical rejection. The scheduler
                    // may continue it within the same frame's idle budget.
                    cooperativePending=world.lastStepResult?.accepted===false&&world.lastStepResult.pending===true&&!world.lastStepResult.error;
                    if(!cooperativePending&&world.lastStepResult?.terminal===true&&recovery) {
                        recovery.rollback(entry.context,world.lastStepResult);
                        world.abandonFailedWholeStep();
                        blocked={key:entry.recoveryKey};pending=null;epoch++;terminal=true;
                    }
                    if(!cooperativePending)lastRejectedFrame=frame;
                }
            } catch(caught) {
                error=caught;status='error';cooperativePending=false;lastRejectedFrame=frame;
                // Keep the initiating failure when retries hit secondary guards.
                if(pending)pending.firstError??=caught;
            }
            return {accepted,pending:cooperativePending,terminal,attempted:true,status,context,error,firstError:pending?.firstError??null,durationMs:now()-start};
        }
    };
}
