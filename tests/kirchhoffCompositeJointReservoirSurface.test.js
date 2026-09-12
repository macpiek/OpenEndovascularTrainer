import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeJointSurfacePoseHistory as history, prepareCompositeJointSurfacePosePath as prepare } from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';
import { createCompositeJointReservoirSurfaceWorkspace as workspace, evaluateCompositeJointReservoirSurface as evaluate } from '../src/physics/kirchhoffCompositeJointReservoirSurface.js';
import { evaluateCompositeJointSurfaceIncrement as sameEdge, evaluateCompositeJointSurfaceForceMap as physicalB } from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(a,s)=>a.map(v=>v*s);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>scale(a,1/Math.hypot(...a));
const rotate=(v,a,angle)=>add(add(scale(v,Math.cos(angle)),scale(cross(a,v),Math.sin(angle))),scale(a,dot(a,v)*(1-Math.cos(angle))));
const pt=(d,a,t)=>{const v=cross(a,t),w=cross(v,d);return add(add(d,w),scale(cross(v,w),1/(1+dot(a,t))));};
const phase=(a,b,t)=>Math.atan2(dot(t,cross(a,b)),dot(a,b));
const close=(a,b,eps=2e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=eps,`${a} != ${b}`);
const same=(a,b,eps)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],eps));};

function fixture({dt=.1,feed=.3,adjacent=false,attached=false,reservoir=true,coordinate=0}={}){
    const offset=reservoir?1:0,ownCount=adjacent?3:2,positions=Array.from({length:ownCount+offset},(_,i)=>[i-offset,0,0]);
    const source={toolId:'wire',reservoirIdentity:reservoir?17:null,nodes:positions.map((p,i)=>({id:i===0&&reservoir?'external':i-offset,position:p,...(i<offset?{}:{node:i-offset})})),
        edges:Array.from({length:positions.length-1},(_,i)=>({edgeId:i<offset?'reservoir:wire':`wire:${i-offset}`,materialSegmentId:i<offset?'42':42,source:i<offset?'reservoir':'accepted',
            ...(i<offset?{}:{edge:i-offset}),nodeIds:[i===0&&reservoir?'external':i-offset,i+1-offset],coordinates:[i-offset,i+1-offset],labels:[i-offset,i+1-offset],reference:{tangent:[1,0,0],director:[0,1,0]},angle:0})),hinges:[]};
    for(let i=0;i<source.edges.length-1;i++)source.hinges.push({leftEdgeId:source.edges[i].edgeId,rightEdgeId:source.edges[i+1].edgeId,referenceTwist:0});
    const columns=[...Array.from({length:ownCount},(_,node)=>[0,1,2].map(component=>({kind:'position',toolId:'wire',node,component}))).flat(),
        ...Array.from({length:ownCount-1},(_,edge)=>({kind:'angle',toolId:'wire',edge}))];
    const configuration=Float64Array.from([...positions.slice(offset).flat(),...new Array(ownCount-1).fill(0)]);
    const nodeBindings=source.nodes.map(n=>n.node===undefined?{nodeId:n.id,offset:positions[0].slice(),terms:[]}:{nodeId:n.id,offset:[0,0,0],terms:[0,1,2].map(k=>({column:3*n.node+k,weights:[0,1,2].map(j=>j===k?1:0)}))});
    const angleBindings=source.edges.map(e=>e.source==='reservoir'?{edgeId:e.edgeId,offset:0,terms:[]}:{edgeId:e.edgeId,offset:0,terms:[{column:3*ownCount+e.edge,weight:1}]});
    if(attached){
        nodeBindings[0]={nodeId:'external',offset:[0,0,0],terms:[0,1,2].flatMap(k=>adjacent?[{column:k,weights:[0,1,2].map(j=>j===k?1:0)},{column:3+k,weights:[0,1,2].map(j=>j===k?1:0)},{column:6+k,weights:[0,1,2].map(j=>j===k?-1:0)}]:[{column:k,weights:[0,1,2].map(j=>j===k?2:0)},{column:3+k,weights:[0,1,2].map(j=>j===k?-1:0)}])};
        angleBindings[0]={edgeId:'reservoir:wire',offset:0,terms:[{column:3*ownCount+(adjacent?1:0),weight:1}]};
    }
    const preparation={targetEdgeId:'wire:0',dt,currentMaps:source.edges.map(e=>({edgeId:e.edgeId,labels:e.labels.map(s=>s-feed*dt),dsDt:-feed})),configurationColumns:columns,nodeBindings,angleBindings,reservoirIdentity:source.reservoirIdentity};
    const query={point:[0,.2,0],normal:[0,1,0],tangent:[1,0,0]},input={configuration,query:{coordinate,...(coordinate===0?{trace:'right'}:{})},finiteGeometry:{kind:'explicit-affine-side-queries',current:structuredClone(query),previous:structuredClone(query)}};
    const h=history(source),path=prepare({history:h,...preparation}),out=workspace(path);
    return{source,preparation,h,path,out,input,ownCount};
}
function rebuild(f){f.h=history(f.source);f.path=prepare({history:f.h,...f.preparation});f.out=workspace(f.path);return f;}
function changeQuery(input,j,h){if(j===0)input.query.coordinate+=h;else{const local=j-1,q=input.finiteGeometry[local<9?'current':'previous'];q[['point','normal','tangent'][Math.floor((local%9)/3)]][local%3]+=h;}}
function directInput(f){const e=f.source.edges.find(e=>e.edgeId===f.preparation.targetEdgeId),map=f.preparation.currentMaps.find(m=>m.edgeId===e.edgeId),col=f.input.configuration,x=f.input.query.coordinate,dx=e.coordinates[1]-e.coordinates[0],sx=(map.labels[1]-map.labels[0])/dx,oldSx=(e.labels[1]-e.labels[0])/dx,frac=(x-e.coordinates[0])/dx;
    return{tools:[{id:'wire',edgeId:e.edgeId,coordinates:e.coordinates,coordinate:x,trace:f.input.query.trace,positions:[0,1].map(i=>Array.from(col.slice(3*(e.edge+i),3*(e.edge+i)+3))),previousPositions:e.nodeIds.map(id=>f.source.nodes.find(n=>n.id===id).position),reference:e.reference,
        angle:col[3*f.ownCount+e.edge],previousAngle:e.angle,materialMap:{sStart:map.labels[0],dsDx:sx,dsDt:((map.labels[0]-e.labels[0])+(sx-oldSx)*dx*frac)/f.preparation.dt},
        materialPath:{kind:'linear-affine-maps',previousEdgeId:e.edgeId,previousMap:{sStart:e.labels[0],dsDx:oldSx},previousTrace:f.input.query.previousTrace}}],
        dt:f.preparation.dt,wall:{velocity:[0,0,0]},rotationPath:'short-contact-frame-own-unwrapped-spins',finiteGeometry:f.input.finiteGeometry,forceGeometry:{kind:'explicit-affine-side-query',...f.input.finiteGeometry.current}};
}

test('real proximal dsDt=-.3 resolves explicit reservoir pose at tau=1; reverse feed remains on accepted own material',()=>{
    for(const dt of [.1,.01,1e-4]){const f=fixture({dt}),r=evaluate(f.input,f.out);same(r.increment,[.3*dt,0],3e-14);assert.equal(r.events[0].terminal,true);assert.equal(r.materialLabel,-.3*dt);assert.equal(r.physicalForceMapValid,true);assert.equal(r.forceMapValid,true);assert.equal(r.physicalForceMap.length,14);assert.equal(r.forceMap.length,14);
        assert.equal(r.currentTools[0].materialSegmentId,42);assert.deepEqual(r.currentTools[0].nodes,[0,1]);assert.throws(()=>sameEdge(directInput(f)),{code:'surface-material-transport-required'});}
    const back=fixture({feed:-.3});same(evaluate(back.input,back.out).increment,[-.03,0],3e-14);assert.equal(back.out.events.length,0);
});

test('owned pose history preserves primitive material IDs, all lifts/maps/angles and exact shared-node identity',()=>{
    const f=fixture(),before=structuredClone(f.source),initial=structuredClone(evaluate(f.input,f.out));
    assert.equal(f.h.edges[0].materialSegmentId,'42');assert.equal(f.h.edges[1].materialSegmentId,42);assert.equal(f.h.nodes[1].id,0);
    f.source.nodes[0].position[0]=99;f.source.edges[1].angle=NaN;f.preparation.nodeBindings[0].offset[0]=99;f.preparation.currentMaps[0].labels[0]=NaN;
    assert.deepEqual(evaluate(f.input,f.out).increment,initial.increment);assert.equal(f.h.nodes[0].position[0],before.nodes[0].position[0]);assert.ok(Object.isFrozen(f.h.edges[0].reference.director));
});

test('actual body nodes/spin retain identity columns and reject prescribed/fake free reservoir reaction DOFs',()=>{
    for(const corrupt of [
        f=>{f.preparation.nodeBindings[1].terms=[];},f=>{f.preparation.angleBindings[1].terms=[];},
        f=>{f.preparation.nodeBindings[1].offset[0]=.2;},f=>{f.preparation.angleBindings[1].terms[0].column=0;},
        f=>{f.preparation.configurationColumns[0].node=90;},f=>{f.preparation.configurationColumns.push({...f.preparation.configurationColumns[0]});},
        f=>{f.source.nodes[0].node=2;},f=>{f.source.edges[0].edge=1;},
        f=>{f.source.nodes.push({id:'duplicate-prox',node:0,position:[0,0,0]});f.source.edges[0].nodeIds[1]='duplicate-prox';}
    ]){const f=fixture();corrupt(f);assert.throws(()=>rebuild(f));}
});

test('expanded physical N packing keeps extra actual adjacent G columns and zero B on upstream reservoir attachment dependencies',()=>{
    const f=fixture({adjacent:true,attached:true});f.input.configuration[1]=.02;f.input.configuration[4]=-.01;f.input.configuration[7]=.03;f.input.configuration[9]=.08;f.input.configuration[10]=-.04;
    const r=evaluate(f.input,f.out);assert.equal(r.configurationDofs,11);assert.equal(r.forceMap.length,22);assert.equal(r.configurationJacobian.length,22);
    for(const column of [6,7,8,10])same(Array.from(r.forceMap.slice(column*2,column*2+2)),[0,0],0);
    assert.ok(Math.max(...[6,7,8,10].flatMap(j=>[r.configurationJacobian[j],r.configurationJacobian[11+j]]).map(Math.abs))>1e-5);
    const force=physicalB(directInput(f));const mapping=[0,1,2,3,4,5,9];mapping.forEach((j,p)=>same(Array.from(r.forceMap.slice(j*2,j*2+2)),Array.from(force.forceMap.slice(p*2,p*2+2)),0));
    const rates=Array.from({length:11},(_,j)=>.13*Math.cos(j+1)),traction=[.3,-.8],work=dot(rates,Array.from({length:11},(_,j)=>r.forceMap[2*j]*traction[0]+r.forceMap[2*j+1]*traction[1]));
    close(work,mapping.reduce((s,j,p)=>s+rates[j]*(force.forceMap[p*2]*traction[0]+force.forceMap[p*2+1]*traction[1]),0),2e-14);
});

test('full G and expanded B/DB differentiate every physical column and every current/previous query column',context=>{
    const f=fixture({adjacent:true,attached:true,coordinate:.015});f.input.configuration[1]=.01;f.input.configuration[4]=-.02;f.input.configuration[7]=.03;f.input.configuration[9]=.09;f.input.configuration[10]=-.05;
    const r=structuredClone(evaluate(f.input,f.out)),N=r.configurationDofs,h=1e-6;let maxG=0,maxDB=0;
    for(let j=0;j<N+19;j++){const plus=structuredClone(f.input),minus=structuredClone(f.input);if(j<N){plus.configuration[j]+=h;minus.configuration[j]-=h;}else{changeQuery(plus,j-N,h);changeQuery(minus,j-N,-h);}
        const a=structuredClone(evaluate(plus,f.out)),b=evaluate(minus,f.out);
        for(let k=0;k<2;k++){const fd=(a.increment[k]-b.increment[k])/(2*h),ad=j<N?r.configurationJacobian[k*N+j]:r.queryJacobian[k*19+j-N];maxG=Math.max(maxG,Math.abs(fd-ad));close(fd,ad,4e-8);}
        for(let entry=0;entry<N*2;entry++){const fd=(a.forceMap[entry]-b.forceMap[entry])/(2*h),ad=j<N?r.DforceMap[entry*N+j]:r.forceMapQueryDerivative[entry*19+j-N];maxDB=Math.max(maxDB,Math.abs(fd-ad));close(fd,ad,4e-8);}
    }context.diagnostic(JSON.stringify({columns:N+19,maxG,maxDB}));
});

test('terminal entry preserves outgoing one-sided foot G and zero-duration endpoint derivatives',context=>{
    const f=fixture({attached:true});f.input.configuration[1]=.02;f.input.configuration[4]=-.01;f.input.configuration[6]=.1;
    const r=structuredClone(evaluate(f.input,f.out)),plus=structuredClone(f.input),h=1e-6;plus.query.coordinate+=h;const p=evaluate(plus,f.out);let maxError=0;
    for(let k=0;k<2;k++){const error=Math.abs((p.increment[k]-r.increment[k])/h-r.queryJacobian[k*19]);maxError=Math.max(maxError,error);assert.ok(error<2e-5);}
    assert.equal(r.events[0].tau,1);close(r.events[0].queryDerivative[0],-1/.03,2e-13);context.diagnostic(JSON.stringify({maxOneSidedGError:maxError}));
});

test('same-edge specialization matches frozen finite values/G and B when history has no reservoir',()=>{
    const f=fixture({reservoir:false,coordinate:.4});f.input.configuration[1]=.02;f.input.configuration[4]=-.03;f.input.configuration[6]=.09;
    const r=evaluate(f.input,f.out),old=sameEdge(directInput(f)),force=physicalB(directInput(f));same(r.increment,Array.from(old.increment),3e-14);same(r.configurationJacobian,Array.from(old.configurationJacobian),3e-13);same(r.queryJacobian,Array.from(old.queryJacobian),3e-13);same(r.forceMap,Array.from(force.forceMap),0);
});

test('compatible entry keeps nonzero own bending/spin, complete turns and B/feed power limit',context=>{
    const limits=[];const rates=[[.04,-.03,.02],[-.02,.05,.01]],spin=.7,traction=[.3,-.8];
    for(const dt of [1e-3,1e-4,1e-5]){const f=fixture({dt});rates.forEach((v,i)=>v.forEach((x,k)=>f.input.configuration[3*i+k]+=x*dt));f.input.configuration[6]=spin*dt;const r=evaluate(f.input,f.out),force=r.physicalForce,ownRate=rates.flat().concat(spin),feed=scale(sub(Array.from(f.input.configuration.slice(3,6)),Array.from(f.input.configuration.slice(0,3))),.3),
        expected=force.axes.map((axis,k)=>dot(axis,feed)+ownRate.reduce((s,v,j)=>s+force.forceMap[j*2+k]*v,0)),velocity=Array.from(r.increment,x=>x/dt),error=Math.max(...sub(velocity,expected).map(Math.abs));
        close(dot(traction,velocity),dot(traction,expected),dt*.1);limits.push({dt,error,powerError:Math.abs(dot(traction,velocity)-dot(traction,expected))});}
    assert.ok(limits[2].error<.011*limits[0].error);const f=fixture();f.input.configuration[6]=2*Math.PI;const r=evaluate(f.input,f.out);same(r.increment,[.03,.4*Math.PI],3e-14);context.diagnostic(JSON.stringify(limits));
});

test('independent reference gauges and full angle lifts retain physical reservoir entry and G',()=>{
    const f=fixture({attached:true}),r=structuredClone(evaluate(f.input,f.out)),gauges=[2*Math.PI+.7,-4*Math.PI+.4];
    f.source.edges.forEach((e,j)=>{e.reference.director=rotate(e.reference.director,e.reference.tangent,gauges[j]);e.angle-=gauges[j];});f.source.hinges[0].referenceTwist+=gauges[1]-gauges[0];
    // Preserve the explicit external angle dependency after changing both own gauges.
    f.input.configuration[6]-=gauges[1];f.preparation.angleBindings[0].offset+=gauges[1]-gauges[0];rebuild(f);
    const g=evaluate(f.input,f.out);same(g.increment,r.increment,4e-14);same(g.configurationJacobian,r.configurationJacobian,3e-13);same(g.forceMap,r.forceMap,0);
});

test('observer rotation and common finite body/query motion retain objective compatible entry',()=>{
    const f=fixture(),r=structuredClone(evaluate(f.input,f.out)),axis=unit([.3,-.4,.8]),angle=.83,shift=[.3,-.1,.7],R=v=>rotate(v,axis,angle),point=v=>add(R(v),shift);
    const observer=fixture();observer.source.nodes.forEach(n=>n.position=point(n.position));observer.source.edges.forEach(e=>{e.reference.tangent=R(e.reference.tangent);e.reference.director=R(e.reference.director);});
    observer.preparation.nodeBindings[0].offset=point(observer.preparation.nodeBindings[0].offset);for(let j=0;j<2;j++)observer.input.configuration.set(point(Array.from(observer.input.configuration.slice(j*3,j*3+3))),j*3);
    for(const q of [observer.input.finiteGeometry.current,observer.input.finiteGeometry.previous]){q.point=point(q.point);q.normal=R(q.normal);q.tangent=R(q.tangent);}rebuild(observer);same(evaluate(observer.input,observer.out).increment,r.increment,4e-14);
    const moving=fixture();moving.preparation.nodeBindings[0].offset=point(moving.preparation.nodeBindings[0].offset);for(let j=0;j<2;j++)moving.input.configuration.set(point(Array.from(moving.input.configuration.slice(j*3,j*3+3))),j*3);
    const t=R([1,0,0]),d=pt([0,1,0],[1,0,0],t),correction=phase(d,R([0,1,0]),t);moving.input.configuration[6]=correction;moving.preparation.angleBindings[0].offset=correction;
    const q=moving.input.finiteGeometry.current;q.point=point(q.point);q.normal=R(q.normal);q.tangent=R(q.tangent);rebuild(moving);same(evaluate(moving.input,moving.out).increment,r.increment,4e-14);
});

test('full/value flags, owned preparation and buffer invalidation recover after errors without mutating candidate inputs',()=>{
    const f=fixture({attached:true}),input=structuredClone(f.input),r=structuredClone(evaluate(f.input,f.out)),buffers=[f.out.increment,f.out.configurationJacobian,f.out.DforceMap];
    evaluate({...f.input,order:'value'},f.out);assert.deepEqual(f.out.increment,r.increment);assert.deepEqual(f.out.forceMap,r.forceMap);assert.deepEqual(f.out.physicalForceMap,r.physicalForceMap);
    for(const key of ['incrementValid','forceMapValid','physicalForceMapValid'])assert.equal(f.out[key],true);
    for(const key of ['operatorReady','configurationJacobianValid','queryJacobianValid','GValid','DforceMapValid','forceMapQueryDerivativeValid','physicalDforceMapValid'])assert.equal(f.out[key],false);
    assert.ok(f.out.configurationJacobian.every(Number.isNaN));assert.ok(f.out.DforceMap.every(Number.isNaN));assert.equal(f.out.physicalForce.configurationDerivative,null);
    const bad=structuredClone(f.input);bad.configuration[0]=NaN;assert.throws(()=>evaluate(bad,f.out));assert.equal(f.out.forceMapValid,false);assert.equal(f.out.physicalForceMapValid,false);assert.equal(f.out.physicalForce,null);assert.ok(f.out.increment.every(Number.isNaN));
    evaluate(f.input,f.out);assert.deepEqual(f.out.increment,r.increment);assert.deepEqual(f.out.configurationJacobian,r.configurationJacobian);assert.deepEqual(f.input,input);assert.equal(f.out.increment,buffers[0]);assert.equal(f.out.configurationJacobian,buffers[1]);assert.equal(f.out.DforceMap,buffers[2]);
});

test('sharp internal hinge and incompatible accepted reservoir twist reject with explicit unsupported metadata',()=>{
    const internal=fixture({reservoir:false,adjacent:true,coordinate:1.015});internal.preparation.targetEdgeId='wire:1';rebuild(internal);
    let error;try{evaluate(internal.input,internal.out);}catch(e){error=e;}assert.equal(error.code,'surface-material-transport-required');assert.equal(error.requiredTransport.reason,'internal-hinge-surface-reconstruction-required');assert.equal(internal.out.supported,false);assert.equal(internal.out.forceMapValid,false);
    const f=fixture();f.source.edges[0].angle=2*Math.PI;rebuild(f);assert.throws(()=>evaluate(f.input,f.out),e=>e.requiredTransport?.reason==='incompatible-accepted-reservoir-orientation');assert.equal(f.out.physicalForceMapValid,false);
});

test('value/full equality spans prescribed/attached reservoir, expanded support, nonzero bending and unwrapped turns',()=>{
    for(const options of [{},{attached:true},{adjacent:true,attached:true},{reservoir:false,coordinate:.4}]){
        const f=fixture(options);f.input.configuration[1]=.01;f.input.configuration[4]=-.03;f.input.configuration[3*f.ownCount]=4*Math.PI+.2;
        const full=structuredClone(evaluate(f.input,f.out)),value=evaluate({...f.input,order:'value'},f.out);
        for(const key of ['increment','relativeIncrement','forceMap','physicalForceMap'])assert.deepEqual(value[key],full[key]);
        assert.deepEqual(value.segments,full.segments);assert.deepEqual(value.jumps,full.jumps);
        for(const e of value.events){assert.equal(e.queryDerivative,null);assert.equal(e.configurationDerivative,null);}
        assert.equal(value.physicalForce.derivativeValid,false);assert.equal(value.DforceMapValid,false);assert.ok(value.queryJacobian.every(Number.isNaN));
        assert.deepEqual(evaluate(f.input,f.out).DforceMap,full.DforceMap);
    }
});

test('every evaluation failure revokes finite, physical7 and contractedN operators before a successful retry',()=>{
    const invalid=[
        x=>{x.configuration[0]=Infinity;},x=>{x.configuration=new Float64Array(1);},
        x=>{x.configuration.set(x.configuration.slice(0,3),3);},x=>{x.order='gradient';},
        x=>{delete x.query;},x=>{x.query.coordinate=NaN;},x=>{x.query.coordinate=-2;},x=>{delete x.query.trace;},
        x=>{x.query.coordinate=1;x.query.trace='right';},x=>{x.query.previousTrace='unknown';},
        x=>{x.finiteGeometry.kind='unknown';},x=>{x.finiteGeometry.previous.point[0]=NaN;},
        x=>{x.finiteGeometry.previous.tangent=[0,0,0];},x=>{x.finiteGeometry.current.normal=x.finiteGeometry.current.tangent.slice();},
        x=>{x.finiteGeometry.previous.normal=[0,1e-30,0];}
    ];
    const f=fixture(),saved=structuredClone(evaluate(f.input,f.out));
    for(const order of ['full','value'])for(const mutate of invalid){const input={...structuredClone(f.input),order};mutate(input);assert.throws(()=>evaluate(input,f.out));
        for(const flag of ['supported','incrementValid','GValid','forceMapValid','DforceMapValid','physicalForceMapValid','physicalDforceMapValid'])assert.equal(f.out[flag],false);
        for(const key of ['increment','configurationJacobian','queryJacobian','forceMap','physicalForceMap','DforceMap','forceMapQueryDerivative'])assert.ok(f.out[key].every(Number.isNaN));
        assert.equal(f.out.physicalForce,null);assert.deepEqual(evaluate(f.input,f.out).increment,saved.increment);
    }
    assert.throws(()=>{f.out.configurationColumns=[];},TypeError);assert.throws(()=>{f.out.forceMap=new Float64Array(14);},TypeError);
});

test('accepted frame/phase/map provenance, reservoir identity and explicit dependencies validate before preparing a callable path',()=>{
    const corruptions=[
        f=>{f.source.edges[1].reference.tangent=[0,1,0];},f=>{f.source.edges[1].angle=NaN;},
        f=>{f.source.hinges[0].referenceTwist=.2;},f=>{f.source.edges[0].materialSegmentId={id:42};},
        f=>{f.source.edges[0].labels=[0,0];},f=>{f.source.edges[0].labels[1]=.01;},
        f=>{f.preparation.currentMaps[0].labels[1]=.02;},f=>{f.preparation.currentMaps[1].dsDt=.3;},
        f=>{f.preparation.nodeBindings[0].terms=undefined;},f=>{f.preparation.angleBindings[0].terms=[{column:99,weight:1}];},
        f=>{f.preparation.nodeBindings[0].terms=[{column:0,weights:[1,0,0]},{column:0,weights:[0,1,0]}];},
        f=>{f.preparation.reservoirIdentity='17';},f=>{f.preparation.dt=0;},f=>{f.preparation.targetEdgeId='reservoir:wire';}
    ];
    for(const change of corruptions){const f=fixture();change(f);assert.throws(()=>rebuild(f));}
    const f=fixture();assert.throws(()=>workspace({...f.path}),/prepared own surface pose path/);assert.throws(()=>prepare({...f.preparation,history:{...f.h}}),/owned surface pose history/);
});

test('same-edge stationary material at either physical endpoint retains explicit accepted/current traces and finite G',()=>{
    for(const reservoir of [false,true])for(const coordinate of [0,1]){
        const f=fixture({reservoir,feed:0,coordinate});f.input.query.trace=coordinate===0?'right':'left';f.input.query.previousTrace=f.input.query.trace;
        f.input.configuration[1]=.01;f.input.configuration[4]=-.03;f.input.configuration[6]=.07;
        const r=evaluate(f.input,f.out),old=sameEdge(directInput(f));same(r.increment,Array.from(old.increment),3e-14);same(r.configurationJacobian,Array.from(old.configurationJacobian),3e-13);same(r.queryJacobian,Array.from(old.queryJacobian),3e-13);
        const bad=structuredClone(f.input);delete bad.query.previousTrace;assert.throws(()=>evaluate(bad,f.out),e=>e.requiredTransport?.reason==='explicit-accepted-material-trace-required');
        same(evaluate(f.input,f.out).increment,Array.from(old.increment),3e-14);
    }
});
