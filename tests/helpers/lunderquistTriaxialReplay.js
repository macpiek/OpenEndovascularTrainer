import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {restoreSharedAxisReplay} from '../../src/physics/kirchhoffSharedAxisRestore.js';
import {advanceSharedAxis,SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';

export function readLunderquistTriaxialReplay() {
    return JSON.parse(gunzipSync(fs.readFileSync(new URL('../fixtures/shared-axis/lunderquist-triaxial-near-outlet.json.gz',import.meta.url))));
}

// This continuation isolates the captured failure. Blood flow is frozen at
// capture time; it is not a full application/pulsatile-flow benchmark.
export function runLunderquistTriaxialReplay(fixture,field,{patch={},count=30,onStep=()=>{},deadlineMs=10000}={}) {
    const request=fixture.stepRequest;
    let state=restoreSharedAxisReplay(fixture,field),rotations=request.rotations;
    const steps=[],poses=[];
    for(let i=0;i<count;i++) {
        const offset=i<10?i*request.dt*25:i<20?(19-i)*request.dt*25:0;
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+(t.id==='catheter'?offset:0)}));
        const start=performance.now(),iterator=advanceSharedAxis(state,rotations,request.dt,tools,
            {...request.options,newtonActiveSetLimit:SHARED_AXIS_APP_PHYSICS_OPTIONS.newtonActiveSetLimit,...patch});
        let next,cancelled=false;
        do {
            next=iterator.next();
            if(!next.done&&performance.now()-start>deadlineMs){iterator.return();cancelled=true;break;}
        } while(!next.done);
        const r=next.value?.result??{},ms=performance.now()-start;
        const accepted=!cancelled&&!!next.value?.state;
        const record={i,phase:i<10?'advance':i<20?'withdraw':'hold',ms,accepted,cancelled,
            ...Object.fromEntries(['status','iterations','factorizations','subdivisions','certificateBound','residual','quality','timings'].map(k=>[k,r[k]]))};
        steps.push(record);onStep(record);
        if(!accepted)break;
        state=next.value.state;rotations=next.value.rotations;
        poses.push({coordinates:state.coordinates.slice(),positions:state.positions.map(p=>p.map((v,k)=>v+state.origin[k])),
            frames:state.materials.map(m=>['X','Y','Z','W'].map(k=>Array.from(m.body['orientation'+k]))),
            velocities:structuredClone(state.velocities),insertions:state.materials.map(m=>m.spec.insertion)});
    }
    const times=steps.map(s=>s.ms).sort((a,b)=>a-b);
    return {steps,poses,summary:{steps:times.length,allAccepted:steps.length===count&&steps.every(s=>s.accepted),
        medianMs:times[times.length>>1],p95Ms:times[Math.ceil(times.length*.95)-1],maxMs:times.at(-1),
        stepsWithin60HzBudget:times.filter(ms=>ms<=1000/60).length}};
}

export function compareTriaxialPoses(a,b) {
    if(a.length!==b.length)return {sameTopology:false,maxPositionMm:Infinity,maxFrameComponent:Infinity,maxVelocity:Infinity};
    let maxPositionMm=0,maxFrameComponent=0,maxVelocity=0;
    for(let i=0;i<a.length;i++) {
        if(JSON.stringify(a[i].coordinates)!==JSON.stringify(b[i].coordinates)||JSON.stringify(a[i].insertions)!==JSON.stringify(b[i].insertions))
            return {sameTopology:false,maxPositionMm:Infinity,maxFrameComponent:Infinity,maxVelocity:Infinity};
        for(let n=0;n<a[i].positions.length;n++) {
            maxPositionMm=Math.max(maxPositionMm,Math.hypot(...a[i].positions[n].map((v,k)=>v-b[i].positions[n][k])));
            for(let k=0;k<3;k++)maxVelocity=Math.max(maxVelocity,Math.abs(a[i].velocities[n][k]-b[i].velocities[n][k]));
        }
        a[i].frames.forEach((f,m)=>f.forEach((axis,k)=>axis.forEach((v,n)=>{maxFrameComponent=Math.max(maxFrameComponent,Math.abs(v-b[i].frames[m][k][n]));})));
    }
    return {sameTopology:true,maxPositionMm,maxFrameComponent,maxVelocity};
}
