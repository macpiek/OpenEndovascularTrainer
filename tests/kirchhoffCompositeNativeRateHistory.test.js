import assert from 'node:assert/strict';
import test from 'node:test';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {captureCompositeNativeRateHistory as capture,sampleCompositeNativeRateHistory as sample} from '../src/physics/kirchhoffCompositeNativeRateHistory.js';

const model='endpoint-derivative-of-linear-grid-pose-path',ids=['wire','catheter'],
    dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    unit=v=>v.map(x=>x/Math.hypot(...v)),close=(a,b,t=2e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`),
    same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
function fixture(straight=false) {
    const coordinates=[0,1,2,3],dt=.04,layout={edgeToolIds:coordinates.slice(1).map(()=>ids)},
        previous={coordinates,layout,elasticityGeometry:'native-discrete-rod',inertiaGeometryByTool:null,toolPositions:new Map(),angles:new Map(),tools:[]},
        current={...previous,toolPositions:new Map(),angles:new Map()},grid={wire:[.04,-.03,.02],catheter:[-.05,.02,-.01]},spin={wire:3,catheter:-2},metric={wire:1.25,catheter:.75},feed={wire:-.3,catheter:.2};
    for(const [i,id] of ids.entries()) {
        const old=coordinates.map(x=>[x,(straight?0:.05*x*x)+i*.4,straight?0:.013*x*x*x]),oldAngles=coordinates.slice(1).map((_,j)=>4*Math.PI+(straight?0:.07*j));
        previous.toolPositions.set(id,old);previous.angles.set(id,oldAngles);
        previous.tools.push({id,reference:captureCompositeReferenceFrames(old,i?[0,0,1]:[0,1,0])});
        current.toolPositions.set(id,old.map((p,j)=>p.map((v,k)=>v+dt*(grid[id][k]+(straight?0:.08*j*(k+1))))));
        current.angles.set(id,oldAngles.map((v,j)=>v+dt*(spin[id]+(straight?.0:.04*j))));
    }
    const prepared={inertiaEdges:coordinates.slice(1).map((_,edge)=>({tools:ids.map(id=>({id,materialMap:{sStart:(id==='wire'?20:40)+metric[id]*coordinates[edge],dsDx:metric[id],
        dsDt:straight?feed[id]:[feed[id]+.01*edge,feed[id]+.01*(edge+1)]}}))}))};
    return {previous,current,prepared,dt,rateModel:model,grid,spin,metric,feed};
}
function phasePose(entry,dt,alpha,label) {
    const positions=entry.previousPositions.map((p,j)=>p.map((v,k)=>v+alpha*(entry.positions[j][k]-v))),chord=positions[1].map((v,k)=>v-positions[0][k]),t=unit(chord),
        axis=cross(entry.reference.tangent,t),first=cross(axis,entry.reference.director),second=cross(axis,first),den=1+dot(entry.reference.tangent,t),
        ref=entry.reference.director.map((v,k)=>v+first[k]+second[k]/den),side=cross(t,ref),angle=entry.previousAngle+alpha*(entry.angle-entry.previousAngle),
        d1=ref.map((v,k)=>Math.cos(angle)*v+Math.sin(angle)*side[k]),d2=cross(t,d1),
        labels=entry.labels.map((s,j)=>s+(alpha-1)*dt*entry.materialMap.dsDt[j]),fraction=(label-labels[0])/(labels[1]-labels[0]),
        position=positions[0].map((v,k)=>v+fraction*chord[k]);
    return {position,directors:[d1,d2,t]};
}

test('explicit native endpoint path preserves independent tool translation, feed, full unwrapped spin and constant open-edge angular rate',()=>{
    const f=fixture(true),history=capture(f);
    for(const id of ids)for(const e of history.tools.find(t=>t.id===id).edges)for(const fraction of [0,.37,1]) {
        const label=e.labels[0]+fraction*(e.labels[1]-e.labels[0]),r=sample({history,toolId:id,label,trace:fraction===1?'left':'right'});
        same(r.velocity,f.grid[id].map((v,k)=>v-(k===0?f.feed[id]/f.metric[id]:0)));same(r.angularVelocity,[f.spin[id],0,0]);
        assert.equal(r.rateModel,model);assert.equal(r.interpretation,'accepted-native-endpoint-path-material-rate');assert.equal(r.finiteStepSlipKnown,false);assert.equal(r.includesHingeTransport,false);
    }
    f.current.angles.get('wire').forEach((_,j)=>f.current.angles.get('wire')[j]=f.previous.angles.get('wire')[j]+4*Math.PI);
    const winding=capture(f),r=sample({history:winding,toolId:'wire',label:21});same(r.angularVelocity,[4*Math.PI/f.dt,0,0],2e-12);
});

test('changing native tangents and feed match independent derivatives of the declared full endpoint pose path',context=>{
    const f=fixture(),history=capture(f),h=1e-6;let maxVelocity=0,maxOmega=0;
    for(const tool of history.tools)for(const e of tool.edges) {
        const label=e.labels[0]+.37*(e.labels[1]-e.labels[0]),r=sample({history,toolId:tool.id,label}),pose=phasePose(e,f.dt,1,label),
            plus=phasePose(e,f.dt,1+h,label),minus=phasePose(e,f.dt,1-h,label),velocity=plus.position.map((v,k)=>(v-minus.position[k])/(2*h*f.dt)),omega=[0,0,0];
        for(let j=0;j<3;j++) {
            const derivative=plus.directors[j].map((v,k)=>(v-minus.directors[j][k])/(2*h*f.dt)),part=cross(pose.directors[j],derivative);
            part.forEach((v,k)=>omega[k]+=.5*v);
        }
        same(r.position,pose.position,2e-14);same(r.velocity,velocity,2e-7);same(r.angularVelocity,omega,2e-7);
        r.velocity.forEach((v,k)=>maxVelocity=Math.max(maxVelocity,Math.abs(v-velocity[k])));r.angularVelocity.forEach((v,k)=>maxOmega=Math.max(maxOmega,Math.abs(v-omega[k])));
    }
    context.diagnostic(JSON.stringify({maxVelocity,maxOmega}));
});

test('native rate history owns poses, frames, maps and returned samples and retries after an out-of-support lookup',()=>{
    const f=fixture(),history=capture(f),saved=structuredClone(history),args={history,toolId:'wire',label:21},before=sample(args);
    for(const state of [f.previous,f.current])for(const id of ids){state.toolPositions.get(id).forEach(p=>p.fill(NaN));state.angles.get(id).fill(NaN);}
    f.previous.tools.forEach(t=>t.reference.forEach(r=>{r.tangent.fill(NaN);r.director.fill(NaN);}));
    f.prepared.inertiaEdges.forEach(e=>e.tools.forEach(t=>{t.materialMap.sStart=-999;t.materialMap.dsDt.fill(NaN);}));f.current.coordinates[0]=-999;
    assert.deepEqual(history,saved);assert.deepEqual(sample(args),before);
    assert.throws(()=>sample({...args,label:-1}),/explicit reservoir/);assert.deepEqual(sample(args),before);
    before.position.fill(999);before.velocity.fill(999);before.angularVelocity.fill(999);
    assert.deepEqual(history,saved);assert.notDeepEqual(sample(args).angularVelocity,before.angularVelocity);assert.deepEqual(sample({...args,history:structuredClone(history)}),sample(args));
});

test('native accepted labels select explicit one-sided rates at hinges without claiming a finite crossing path or extrapolated reservoir',()=>{
    const f=fixture(),history=capture(f),tool=history.tools.find(t=>t.id==='wire'),label=tool.edges[1].labels[0],
        left=sample({history,toolId:'wire',label,trace:'left'}),right=sample({history,toolId:'wire',label,trace:'right'});
    assert.equal(left.edge,0);assert.equal(left.trace,'left');assert.equal(right.edge,1);assert.equal(right.trace,'right');assert.notDeepEqual(left.angularVelocity,right.angularVelocity);
    same(left.position,right.position,1e-14);assert.equal(left.includesHingeTransport,false);assert.equal(right.finiteStepSlipKnown,false);
    // A material label can have entered this edge during the preceding dt.
    // Only its current one-sided endpoint rate is returned, never finite slip.
    const near=tool.edges[1].labels[0]+1e-5,entered=sample({history,toolId:'wire',label:near});assert.equal(entered.edge,1);assert.equal(entered.finiteStepSlipKnown,false);
    for(const label of [tool.edges[0].labels[0]-1e-8,tool.edges.at(-1).labels[1]+1e-8])assert.throws(()=>sample({history,toolId:'wire',label}),/explicit reservoir/);
});

test('native rate source requires exact declared model, body, affine chart and valid own frame metadata',()=>{
    const f=fixture(),history=capture(f),args={history,toolId:'wire',label:21};
    assert.throws(()=>capture({...f,rateModel:undefined}),/Explicit native endpoint/);
    assert.throws(()=>capture({...f,current:{...f.current,elasticityGeometry:'continuous-material-frame'}}),/native discrete rod/);
    assert.throws(()=>capture({...f,current:{...f.current,inertiaGeometryByTool:new Map()}}),/native discrete rod/);
    assert.throws(()=>sample({...args,toolId:'missing'}),/another physical body/);
    assert.throws(()=>sample({...args,history:{...history,rateModel:'instantaneous'}}),/Explicit owned native/);
    assert.throws(()=>sample({...args,history:{...history,finiteStepSlipKnown:true}}),/Explicit owned native/);
    assert.throws(()=>sample({...args,trace:'nearest'}),/Explicit owned native/);
    f.previous.tools[0].reference[0].tangent[0]+=.1;assert.throws(()=>capture(f),/reference frame/);
    const bad=fixture();bad.current.layout={edgeToolIds:[['catheter'],ids,ids]};assert.throws(()=>capture(bad),/another body/);
    const flipped=fixture(true);flipped.current.toolPositions.get('wire')[1]=flipped.current.toolPositions.get('wire')[0].map((v,k)=>v-(k===0?1:0));assert.throws(()=>capture(flipped),/Antiparallel/);
});
