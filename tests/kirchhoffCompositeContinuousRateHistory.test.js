import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeContinuousFrame} from '../src/physics/kirchhoffCompositeContinuousFrame.js';
import {evaluateCompositeContinuousSurfaceMotion} from '../src/physics/kirchhoffCompositeContinuousSurfaceMotion.js';
import {captureCompositeContinuousRateHistory as capture,sampleCompositeContinuousRateHistory as sample} from '../src/physics/kirchhoffCompositeContinuousRateHistory.js';

const ids=['wire','catheter'],close=(a,b,t=2e-11)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`),
    same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
function fixture({straight=false,discontinuousFeed=false}={}) {
    const coordinates=[0,1,2,3,4,5],dt=.02,geometryByTool=new Map(ids.map(id=>[id,createCompositeContinuousGeometry({coordinates})])),
        grid={wire:[.04,-.02,.01],catheter:[-.03,.01,-.02]},spin={wire:2,catheter:-3},feed={wire:-.3,catheter:.2},metric={wire:1.25,catheter:.75},
        previous={coordinates,toolPositions:new Map(),angles:new Map(),tools:[]},current={coordinates,inertiaGeometryByTool:geometryByTool,toolPositions:new Map(),angles:new Map(),tools:[]};
    for(const [i,id] of ids.entries()) {
        const old=coordinates.map(x=>[x,(straight?0:.03*x*x)+i*.4,straight?0:.003*x*x*x]),
            oldAngles=coordinates.slice(1).map((_,j)=>(id==='wire'?4:-2)*Math.PI+(straight?0:.04*j)),
            reference=captureCompositeReferenceFrames(old,id==='wire'?[0,1,0]:[0,0,1]);
        previous.toolPositions.set(id,old);previous.angles.set(id,oldAngles);previous.tools.push({id,reference,referenceTwists:new Array(coordinates.length-2).fill(0)});
        current.toolPositions.set(id,old.map((p,j)=>p.map((v,k)=>v+dt*(grid[id][k]+(straight?0:.002*j*(k+1))))));
        current.angles.set(id,oldAngles.map((v,j)=>v+dt*(spin[id]+(straight?0:.03*j))));
        current.tools.push({id,reference:captureCompositeReferenceFrames(current.toolPositions.get(id),[0,0,1]),referenceTwists:new Array(coordinates.length-2).fill(0)});
    }
    const prepared={inertiaEdges:coordinates.slice(1).map((_,edge)=>({tools:ids.map(id=>({id,materialMap:{sStart:(id==='wire'?20:40)+metric[id]*coordinates[edge],dsDx:metric[id],
        dsDt:discontinuousFeed?feed[id]+.2*edge:[feed[id]+.01*edge,feed[id]+.01*(edge+1)]}}))}))};
    if(straight&&!discontinuousFeed)for(const entry of prepared.inertiaEdges)for(const t of entry.tools)t.materialMap.dsDt=feed[t.id];
    return {previous,current,prepared,dt,geometryByTool,grid,spin,feed,metric};
}
function direct(f,toolId,edge,fraction) {
    const tool=f.previous.tools.find(t=>t.id===toolId),geometry=f.geometryByTool.get(toolId),
        frame=createCompositeContinuousFrame({geometry,edge,toolId,previousPositions:f.previous.toolPositions.get(toolId),previousAngles:f.previous.angles.get(toolId),reference:tool.reference,referenceTwists:tool.referenceTwists}),
        map=f.prepared.inertiaEdges[edge].tools.find(t=>t.id===toolId).materialMap,
        coordinate=f.previous.coordinates[edge]+fraction*(f.previous.coordinates[edge+1]-f.previous.coordinates[edge]);
    return evaluateCompositeContinuousSurfaceMotion({positions:frame.positionNodeIndices.map(n=>f.current.toolPositions.get(toolId)[n]),
        angles:frame.angleEdgeIndices.map(e=>f.current.angles.get(toolId)[e]),coordinate,materialMap:map,dt:f.dt,rateMode:'backward-euler-grid',order:'value',
        contact:{point:[0,0,0],axes:[[1,0,0],[0,1,0]]},wall:{velocity:[0,0,0]}},frame);
}
function labelAt(f,id,edge,fraction){const map=f.prepared.inertiaEdges[edge].tools.find(t=>t.id===id).materialMap;return map.sStart+fraction*map.dsDx*(f.previous.coordinates[edge+1]-f.previous.coordinates[edge]);}

test('accepted history retains independent tool translation, feed and unwrapped spin instead of sharing one rate',()=>{
    const f=fixture({straight:true}),history=capture(f),responses=[];
    for(const id of ids)for(const [edge,fraction] of [[0,0],[1,.37],[3,.82],[4,1]]) {
        const label=labelAt(f,id,edge,fraction),r=sample({history,geometryByTool:f.geometryByTool,toolId:id,label}),expected=direct(f,id,edge,fraction);
        same(r.velocity,f.grid[id].map((v,k)=>v-(k===0?f.feed[id]/f.metric[id]:0)));same(r.angularVelocity,[f.spin[id],0,0]);
        same(r.velocity,expected.velocity);same(r.angularVelocity,expected.omega);same(r.position,expected.position);
        assert.equal(r.toolId,id);assert.equal(r.label,label);assert.equal(r.finiteStepSlipKnown,false);
        assert.equal(r.interpretation,'accepted-continuous-backward-euler-material-rate');responses.push(r);
    }
    assert.notDeepEqual(responses[0].velocity,responses.at(-1).velocity);assert.notDeepEqual(responses[0].angularVelocity,responses.at(-1).angularVelocity);
});

test('curved history samples the original accepted frame gauge and full prior-step surface material rate at matching labels',()=>{
    const f=fixture(),history=capture(f);
    for(const id of ids)for(let edge=0;edge<5;edge++)for(const fraction of [.07,.37,.81]) {
        const label=labelAt(f,id,edge,fraction),r=sample({history,geometryByTool:f.geometryByTool,toolId:id,label}),expected=direct(f,id,edge,fraction);
        same(r.velocity,expected.velocity,2e-12);same(r.angularVelocity,expected.omega,2e-12);same(r.position,expected.position,2e-12);
    }
});

test('capture owns both poses, maps, unwrapped angles and prior gauge; rejected lookups and returned arrays cannot corrupt retry',()=>{
    const f=fixture(),sourceBefore=structuredClone({previous:f.previous,current:f.current,prepared:f.prepared}),history=capture(f),saved=structuredClone(history),
        args={history,geometryByTool:f.geometryByTool,toolId:'wire',label:labelAt(f,'wire',2,.37)},reference=sample(args);
    assert.deepEqual({previous:f.previous,current:f.current,prepared:f.prepared},sourceBefore);
    f.previous.coordinates[0]=-999;
    for(const source of [f.previous,f.current])for(const id of ids) {
        source.toolPositions.get(id).forEach(p=>p.fill(999));source.angles.get(id).fill(-999);
        const t=source.tools.find(t=>t.id===id);t.reference.forEach(r=>{r.director.fill(NaN);r.tangent.fill(NaN);});t.referenceTwists.fill(NaN);
    }
    f.prepared.inertiaEdges.forEach(e=>e.tools.forEach(t=>{t.materialMap.sStart=-999;t.materialMap.dsDt.fill(NaN);}));
    assert.deepEqual(history,saved);assert.deepEqual(sample(args),reference);
    assert.throws(()=>sample({...args,label:-999}),/explicit reservoir outside accepted labels/);
    assert.deepEqual(history,saved);assert.deepEqual(sample(args),reference);
    reference.velocity.fill(999);reference.angularVelocity.fill(999);reference.position.fill(999);
    const retried=sample(args);assert.notDeepEqual(retried.velocity,reference.velocity);assert.deepEqual(history,saved);
    assert.deepEqual(sample({...args,history:structuredClone(history)}),retried);
});

test('left and right traces select their own edge material rates at shared labels, with explicit outer endpoint and reservoir behavior',()=>{
    const f=fixture({straight:true,discontinuousFeed:true}),history=capture(f);
    for(const id of ids)for(let edge=1;edge<5;edge++) {
        const label=labelAt(f,id,edge,0),args={history,geometryByTool:f.geometryByTool,toolId:id,label},left=sample({...args,trace:'left'}),right=sample({...args,trace:'right'}),
            expectedLeft=direct(f,id,edge-1,1),expectedRight=direct(f,id,edge,0);
        same(left.velocity,expectedLeft.velocity);same(right.velocity,expectedRight.velocity);same(left.angularVelocity,expectedLeft.omega);same(right.angularVelocity,expectedRight.omega);
        assert.notDeepEqual(left.velocity,right.velocity);assert.deepEqual(sample(args),right);
    }
    for(const id of ids) {
        const args={history,geometryByTool:f.geometryByTool,toolId:id},start=labelAt(f,id,0,0),end=labelAt(f,id,4,1);
        assert.deepEqual(sample({...args,label:start,trace:'left'}),sample({...args,label:start,trace:'right'}));
        assert.deepEqual(sample({...args,label:end,trace:'left'}),sample({...args,label:end,trace:'right'}));
        for(const label of [start-1e-8,end+1e-8])assert.throws(()=>sample({...args,label}),/explicit reservoir outside accepted labels/);
    }
});

test('history rejects missing material ownership, a changed geometry coordinate chart, invalid traces and nonpositive capture dt',()=>{
    const f=fixture(),history=capture(f),args={history,geometryByTool:f.geometryByTool,toolId:'wire',label:labelAt(f,'wire',1,.37)};
    assert.throws(()=>sample({...args,toolId:'missing'}),/another material geometry/);
    const geometryByTool=new Map(f.geometryByTool);geometryByTool.set('wire',createCompositeContinuousGeometry({coordinates:[0,1,2,3,4,5.1]}));
    assert.throws(()=>sample({...args,geometryByTool}),/another material geometry/);
    assert.throws(()=>sample({...args,trace:'nearest'}),/Explicit accepted/);assert.throws(()=>sample({...args,label:NaN}),/Explicit accepted/);
    for(const dt of [0,-1,Infinity,NaN])assert.throws(()=>capture({...f,dt}),/positive accepted dt/);
    assert.throws(()=>capture({...f,current:{...f.current,inertiaGeometryByTool:null}}),/positive accepted dt/);
});

test('history owns the active geometry chart, accepts identical recompilation and rejects changed interfaces or support with identical coordinates',()=>{
    const f=fixture(),original=f.geometryByTool.get('wire'),mutable={...original,interfaces:original.interfaces.slice(),edges:original.edges.slice()};
    f.geometryByTool.set('wire',mutable);
    const history=capture(f),record=history.tools.find(t=>t.id==='wire'),signature=structuredClone(record.geometrySignature),
        args={history,geometryByTool:f.geometryByTool,toolId:'wire',label:labelAt(f,'wire',1,.37)},before=sample(args),
        recompiled=createCompositeContinuousGeometry({coordinates:f.current.coordinates,interfaces:[]});
    mutable.interfaces.push(2);
    assert.deepEqual(record.geometrySignature,signature);assert.throws(()=>sample(args),/another material geometry/);
    f.geometryByTool.set('wire',recompiled);assert.deepEqual(sample(args),before);
    f.geometryByTool.set('wire',createCompositeContinuousGeometry({coordinates:f.current.coordinates,interfaces:[2]}));
    assert.throws(()=>sample(args),/another material geometry/);
    for(const change of ['region','nodeIndices']) {
        const edges=recompiled.edges.slice(),replacement={...edges[1],[change]:edges[1][change].slice()};replacement[change][0]+=1;edges[1]=replacement;
        f.geometryByTool.set('wire',{...recompiled,edges});assert.throws(()=>sample(args),/another material geometry/);
    }
    f.geometryByTool.set('wire',{...recompiled,edges:recompiled.edges.map((e,j)=>j===1?null:e)});
    assert.throws(()=>sample(args),/another material geometry/);
    f.geometryByTool.set('wire',recompiled);assert.deepEqual(sample(args),before);assert.deepEqual(record.geometrySignature,signature);
});

test('geometry signatures mark inactive tool edges null while preserving all active one-sided regions and copied supports',()=>{
    const f=fixture(),toolId='catheter';f.prepared.inertiaEdges[4].tools=f.prepared.inertiaEdges[4].tools.filter(t=>t.id!==toolId);
    const geometry=createCompositeContinuousGeometry({coordinates:f.current.coordinates,interfaces:[4]}),mutable={...geometry,interfaces:geometry.interfaces.slice(),edges:geometry.edges.slice()};
    f.geometryByTool.set(toolId,mutable);
    const history=capture(f),record=history.tools.find(t=>t.id===toolId),args={history,geometryByTool:f.geometryByTool,toolId,label:labelAt(f,toolId,2,.37)},before=sample(args);
    assert.equal(record.geometrySignature.edges.length,5);assert.equal(record.geometrySignature.edges[4],null);
    assert.deepEqual(record.geometrySignature.interfaces,[4]);
    record.geometrySignature.edges.slice(0,4).forEach((e,j)=>{
        assert.deepEqual(e.nodeIndices,geometry.edges[j].nodeIndices);assert.deepEqual(e.region,[0,4]);
        assert.notEqual(e.nodeIndices,geometry.edges[j].nodeIndices);assert.notEqual(e.region,geometry.edges[j].region);
    });
    mutable.edges[4]=null;assert.deepEqual(sample(args),before);
    f.geometryByTool.set(toolId,createCompositeContinuousGeometry({coordinates:f.current.coordinates,interfaces:[4]}));assert.deepEqual(sample(args),before);
    f.geometryByTool.set(toolId,createCompositeContinuousGeometry({coordinates:f.current.coordinates,interfaces:[]}));assert.throws(()=>sample(args),/another material geometry/);
});
