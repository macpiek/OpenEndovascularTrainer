/** Ordered messages with an epoch barrier. A reset invalidates every in-flight
 * response immediately, even if the worker is still finishing its old solve. */
export function createSharedAxisWorkerRpc(worker) {
    let epoch=0,sequence=0,disposed=false;
    const pending=new Map();
    const fail=error=>{for(const entry of pending.values())entry.reject(error);pending.clear();};
    worker.onmessage=({data})=>{
        if(disposed||data.epoch!==epoch)return;
        const entry=pending.get(data.id);if(!entry)return;pending.delete(data.id);
        if(data.error)entry.reject(Object.assign(new Error(data.error.message),{stack:data.error.stack}));else entry.resolve(data.value);
    };
    worker.onerror=event=>fail(new Error(event.message||'Physics worker failed'));
    worker.onmessageerror=()=>fail(new Error('Physics worker returned unreadable data'));
    const rpc={
        request(operation,data) {
            if(disposed)return Promise.reject(new Error('Physics worker is disposed'));
            const id=++sequence;
            return new Promise((resolve,reject)=>{
                pending.set(id,{resolve,reject});
                try{worker.postMessage({epoch,id,operation,data});}catch(error){pending.delete(id);reject(error);}
            });
        },
        reset() {
            epoch++;fail(Object.assign(new Error('Physics worker epoch was cancelled'),{name:'AbortError'}));
            return rpc.request('reset',{});
        },
        dispose() {if(disposed)return;disposed=true;epoch++;fail(Object.assign(new Error('Physics worker disposed'),{name:'AbortError'}));
            worker.onmessage=worker.onerror=worker.onmessageerror=null;worker.terminate();},
        get epoch(){return epoch;}
    };
    return rpc;
}
