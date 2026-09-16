// Diagnostic-only module copied into an isolated app. Never imported by the
// production simulator. Timers are synchronous and unwind at every yield.
let active=null;
export class StageCapture {
    constructor(clock=()=>performance.now()){this.clock=clock;this.stages={};this.stack=[];this.slices=0;this.slowestSlices=[];}
    measure(name,action) {
        const start=this.clock(),entry={child:0};this.stack.push(entry);
        try{return action();}finally {
            const ms=this.clock()-start,selfMs=Math.max(0,ms-entry.child);this.stack.pop();
            if(this.stack.length)this.stack.at(-1).child+=ms;
            const r=this.stages[name]??={calls:0,inclusiveMs:0,selfMs:0,maxMs:0};
            r.calls++;r.inclusiveMs+=ms;r.selfMs+=selfMs;r.maxMs=Math.max(r.maxMs,ms);
            if(this.currentSlice)this.currentSlice.stages[name]=(this.currentSlice.stages[name]??0)+selfMs;
        }
    }
    run(action) {
        const previous=active;active=this;
        const slice={startMs:this.clock(),stages:{}};this.currentSlice=slice;
        try{return this.measure('unattributed',action);}finally {
            slice.ms=this.clock()-slice.startMs;this.currentSlice=null;this.slices++;active=previous;
            if(this.slowestSlices.length<5||slice.ms>this.slowestSlices.at(-1).ms) {
                this.slowestSlices.push(slice);this.slowestSlices.sort((a,b)=>b.ms-a.ms);this.slowestSlices.length=Math.min(5,this.slowestSlices.length);
            }
        }
    }
    report(){return {stages:this.stages,slices:this.slices,slowestSlices:this.slowestSlices};}
}
export function profileCall(name,action){return active?active.measure(name,action):action();}
export function* profileGenerator(name,iterator) {
    let done=false;
    try{while(true){const next=profileCall(name,()=>iterator.next());if(next.done){done=true;return next.value;}yield next.value;}}
    finally{if(!done)profileCall(name,()=>iterator.return());}
}
export function* profileStep(iterator) {
    const capture=new StageCapture();let done=false;
    try {
        while(true) {
            const next=capture.run(()=>iterator.next());
            if(next.done){done=true;return {...next.value,result:{...next.value.result,stageProfile:capture.report()}};}
            yield next.value;
        }
    }finally{if(!done)capture.run(()=>iterator.return());}
}
