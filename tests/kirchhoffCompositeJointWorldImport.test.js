import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld, EndovascularRodBody } from '../src/physics/endovascularPhysicsWorld.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { createBishopFrame, quaternionExp, multiplyQuaternions, materialFrameDirectors } from '../src/physics/discreteKirchhoffRod.js';
import { createCompositeJointAssembly } from '../src/physics/kirchhoffCompositeJointAssembly.js';
import { importCompositeJointWorld } from '../src/physics/kirchhoffCompositeJointWorldImport.js';

const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,k)=>close(v,b[k],t));};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const obj=v=>({x:v[0],y:v[1],z:v[2]});
const read=(b,i,keys=['x','y','z'])=>keys.map(k=>b[k][i]);
const velocities=['velocityX','velocityY','velocityZ'];
function orient(record){
    record.reference.forEach((f,i)=>{
        const q=multiplyQuaternions(quaternionExp(obj(f.tangent.map(v=>v*record.angles[i]))),createBishopFrame(obj(f.tangent),obj(f.director)));
        for(const axis of ['x','y','z','w'])record.body[`orientation${axis.toUpperCase()}`][record.body.activeStart+i]=q[axis];
    });
}
function fixture({wireX=[0,5,10,15,20],catheterX=[2,6,10,14,18],dsWire=1.2,dsCatheter=.85}={}){
    const world=new EndovascularPhysicsWorld(),calls=[],profile={factor:1};
    const tools=[['wire','guidewire',wireX,5,dsWire],['catheter','catheter',catheterX,4,dsCatheter]].filter(([, ,x])=>x).map(([toolId,id,x,spacing,dsDx])=>{
        const body=world.createRod(id,x.length+2,spacing),isWire=toolId==='wire',sign=isWire?1:-1;
        body.activeStart=1;body.activeEnd=x.length;
        for(const key of ['x','y','z',...velocities,'restLength','orientationX','orientationY','orientationZ','orientationW'])body[key].fill(NaN);
        x.forEach((coordinate,i)=>{
            const node=i+body.activeStart,p=[coordinate,sign*(.25+.015*coordinate+.003*coordinate**2),sign*(.125+.007*coordinate**2)];
            ['x','y','z'].forEach((key,k)=>body[key][node]=p[k]);
            velocities.forEach((key,k)=>body[key][node]=sign*(.03*(k+1)+.004*(k+1)*coordinate));
            // The importer must use explicit labels/density, never these legacy values.
            body.materialCoordinate[node]=-999;body.inverseMass[node]=7;
            if(i<x.length-1)body.restLength[node]=dsDx*(x[i+1]-coordinate);
        });
        body.angularVelocityX.fill(7);body.angularVelocityY.fill(-8);body.angularVelocityZ.fill(9);
        const reference=captureCompositeReferenceFrames(x.map((_,i)=>read(body,i+1)));
        reference.forEach((f,i)=>{const angle=.07*i*sign,other=cross(f.tangent,f.director);f.director=f.director.map((v,k)=>v*Math.cos(angle)+other[k]*Math.sin(angle));});
        const record={body,toolId,nodeCoordinates:x.slice(),reference,angles:x.slice(1).map((_,i)=>sign*(.11+.03*i+2*Math.PI*(i+1))),
            referenceTwists:x.slice(2).map((_,i)=>sign*(.07+2*Math.PI*(i+1))),winding:'explicit-unwrapped',
            materialLabels:x.map(v=>(isWire?50:100)+dsDx*(v-x[0])),velocityInterpretation:'physical-material-velocity',
            material:{dsDx,massPerMaterialLength:isWire?.13:.24,materialAt:request=>{
                calls.push(request);const k=profile.factor*(isWire?2:7)+.001*request.materialCoordinate;
                return compileCompositeMaterial({stiffness:[[k,.03,0],[.03,k+1,.02],[0,.02,k+2]],intrinsic:[.002*sign,.001*request.materialStart,.0003],energyOffset:.004});
            }}};
        orient(record);return record;
    });
    return{world,tools,calls,profile};
}
const run=f=>importCompositeJointWorld({tools:f.tools,time:.125,step:7});
function bodySnapshot(body){return Object.fromEntries(Object.entries(body).filter(([,v])=>ArrayBuffer.isView(v)||typeof v==='number'||typeof v==='string').map(([k,v])=>[k,ArrayBuffer.isView(v)?v.slice():v]));}
function affine(record,x,keys=['x','y','z']){
    const exact=record.nodeCoordinates.indexOf(x);if(exact>=0)return read(record.body,record.body.activeStart+exact,keys);
    const e=record.nodeCoordinates.findIndex((a,i)=>a<x&&x<record.nodeCoordinates[i+1]),u=(x-record.nodeCoordinates[e])/(record.nodeCoordinates[e+1]-record.nodeCoordinates[e]);
    const a=read(record.body,record.body.activeStart+e,keys),b=read(record.body,record.body.activeStart+e+1,keys);return a.map((v,k)=>v+u*(b[k]-v));
}

test('actual World spacing 5/4 imports the exact union with independent physical curves and original node roundtrip',()=>{
    const f=fixture(),before=f.tools.map(t=>bodySnapshot(t.body)),r=run(f),s=r.state;
    assert.ok(f.world.bodies.every(b=>b instanceof EndovascularRodBody));
    assert.deepEqual(Array.from(s.coordinates),[0,2,5,6,10,14,15,18,20]);
    assert.deepEqual(s.layout.edgeToolIds,[['wire'],...Array.from({length:6},()=>['wire','catheter']),['wire']]);
    assert.equal(s.relativeToolId,'wire');assert.equal(s.time,.125);assert.equal(s.step,7);
    assert.equal(s.modes.length,7);assert.equal(s.relative.length,21);
    assert.deepEqual(s.modes.map(m=>m.node),[1,2,3,4,5,6,7]);
    s.modes.forEach(m=>assert.deepEqual(m.basis,[[1,0,0],[0,1,0],[0,0,1]]));
    for(const record of f.tools){
        const p=s.toolPositions.get(record.toolId),m=r.mappings.get(record.toolId);
        m.nodes.forEach(({node,jointNode,coordinate})=>{assert.equal(coordinate,record.nodeCoordinates[node-record.body.activeStart]);assert.deepEqual(p[jointNode],read(record.body,node));});
        m.jointNodes.forEach(({jointNode})=>vectorClose(p[jointNode],affine(record,s.coordinates[jointNode])));
    }
    s.positions.forEach((p,j)=>assert.deepEqual(p,affine(f.tools[j===0||j===8?0:1],s.coordinates[j])));
    assert.notDeepEqual(s.toolPositions.get('wire')[4],s.toolPositions.get('catheter')[4]);
    assert.equal(r.evidence.originalNodeCount,10);assert.equal(r.evidence.unionNodeCount,9);assert.equal(r.evidence.maxOriginalPositionError,0);
    assert.ok(r.evidence.maxAffinePositionError<2e-12);assert.equal(r.evidence.everyNewOwnEdgeInsideOneSourceEdge,true);
    assert.deepEqual(f.tools.map(t=>bodySnapshot(t.body)),before);assert.equal(f.world.stepCount,0);
});

test('each refined own edge stays in one original edge, with copied physical spin, frame and retained hinge winding',()=>{
    const f=fixture(),r=run(f),s=r.state;
    for(const record of f.tools){
        const tool=s.tools.find(t=>t.id===record.toolId),m=r.mappings.get(record.toolId),a=s.angles.get(record.toolId),rest=s.restLengths.get(record.toolId);
        for(const edge of m.edges){
            assert.equal(edge.fractions[0][0],0);assert.equal(edge.fractions.at(-1)[1],1);
            edge.fractions.slice(1).forEach((v,i)=>assert.equal(v[0],edge.fractions[i][1]));
            close(edge.jointEdges.reduce((sum,e)=>sum+rest[e],0),record.material.dsDx*(edge.coordinateInterval[1]-edge.coordinateInterval[0]));
        }
        for(const {jointEdge:e,sourceEdge:i,u0,u1} of m.jointEdges){
            assert.ok(u0>=0&&u1<=1&&u1>u0);assert.deepEqual(tool.reference[e],record.reference[i]);assert.equal(a[e],record.angles[i]);
            assert.equal(rest[e],record.material.dsDx*(s.coordinates[e+1]-s.coordinates[e]));
            const q=materialFrameDirectors(Object.fromEntries(['x','y','z','w'].map(k=>[k,record.body[`orientation${k.toUpperCase()}`][record.body.activeStart+i]])));
            const f=tool.reference[e],d1=f.director.map((v,k)=>v*Math.cos(a[e])+cross(f.tangent,f.director)[k]*Math.sin(a[e]));
            vectorClose(d1,['x','y','z'].map(k=>q.d1[k]));
        }
        for(const h of s.layout.hinges)if(h.tools.includes(record.toolId)){
            const old=record.nodeCoordinates.indexOf(s.coordinates[h.vertex]);
            assert.equal(tool.referenceTwists[h.vertex-1],old>0&&old<record.nodeCoordinates.length-1?record.referenceTwists[old-1]:0);
        }
        tool.reference.forEach((frame,e)=>{if(!s.layout.edgeToolIds[e].includes(record.toolId)){assert.equal(frame,null);assert.ok(Number.isNaN(a[e]));assert.ok(Number.isNaN(rest[e]));}});
    }
    assert.ok(r.evidence.maxSourceRestStorageError>0);assert.ok(r.evidence.maxSourceQuaternionDirectorError<1e-12);
    assert.equal(r.evidence.knownWindingsPreserved,true);
});

test('publication bindings cover each active source node and edge once; midpoint cuts choose the outgoing child',()=>{
    const f=fixture({catheterX:[2.5,6.5,10.5,14.5,18.5]}),r=run(f);
    assert.equal(r.bindings.length,2);
    for(const b of r.bindings){
        const record=f.tools.find(t=>t.toolId===b.toolId),m=r.mappings.get(b.toolId);
        assert.equal(b.body,record.body);assert.deepEqual(b.nodes.map(n=>n.node),[1,2,3,4,5]);assert.deepEqual(b.edges.map(e=>e.edge),[1,2,3,4]);
        assert.deepEqual(b.nodes.map(n=>n.trace),['right','right','right','right','left']);
        b.edges.forEach(e=>assert.equal(e.jointEdge,m.edges.find(x=>x.edge===e.edge).jointEdge));
        m.edges.forEach(e=>{assert.ok(r.state.coordinates[e.jointEdge]<=e.midpoint);assert.ok(r.state.coordinates[e.jointEdge+1]>e.midpoint);});
    }
    const wireFirst=r.bindings[0].edges[0];assert.equal(r.state.coordinates[wireFirst.jointEdge],2.5);
    assert.equal(r.bindings[0].toolId,'wire');assert.equal(r.bindings[0].body.id,'guidewire');
    assert.equal(r.solverAdvanced,false);assert.equal(r.certified,false);assert.equal(r.remapReady,false);assert.equal(r.fullFeedLifecycleReady,false);
});

test('physical velocity and material label history is preserved and owned, with no angular or force-history inference',()=>{
    const f=fixture(),r=run(f);
    for(const record of f.tools){
        const snapshot=r.history.sourceSnapshots.find(t=>t.toolId===record.toolId);
        assert.deepEqual(snapshot.materialLabels,record.materialLabels);assert.deepEqual(snapshot.angles,record.angles);assert.deepEqual(snapshot.referenceTwists,record.referenceTwists);
        assert.deepEqual(snapshot.physicalVelocities,record.nodeCoordinates.map((_,j)=>read(record.body,j+1,velocities)));
        assert.equal(snapshot.massPerMaterialLength,record.material.massPerMaterialLength);
        assert.equal(snapshot.angularVelocity,null);assert.equal(snapshot.materialSpin,null);assert.equal(snapshot.frameSpin,null);
    }
    for(const edge of r.history.materialVelocities)for(const t of edge.tools){
        const record=f.tools.find(s=>s.toolId===t.id),x=r.state.coordinates;
        t.velocities.forEach((v,k)=>assert.deepEqual(v,affine(record,x[edge.edge+k],velocities)));
        for(const [label,node] of [[t.sStart,edge.edge],[t.sEnd,edge.edge+1]])close(label,record.materialLabels[0]+record.material.dsDx*(x[node]-record.nodeCoordinates[0]));
        assert.equal(t.interpretation,'physical-material-velocity');assert.equal(t.angularVelocity,null);assert.equal(t.materialSpin,null);assert.equal(t.frameSpin,null);
    }
    assert.equal(r.history.includesAngularVelocity,false);assert.equal(r.history.includesKnownOrientation,true);
    assert.ok(r.state.lengthMultipliers.every(v=>v===0));assert.equal(r.state.boundaryMultipliers.size,0);
    for(const key of ['lumenContactState','wallContactState','lumenFrictionState','wallFrictionState'])assert.equal(r.state[key],undefined);
    const old=r.history.materialVelocities[0].tools[0].velocities[0][0];r.state.materialVelocities[0].tools[0].velocities[0][0]=123;
    assert.equal(r.history.materialVelocities[0].tools[0].velocities[0][0],old);
    assert.throws(()=>r.history.sourceSnapshots[0].physicalVelocities[0][0]=123,TypeError);
});

test('compiled profile samples use own material coordinates/supports and remain valid in authoritative Joint assembly',()=>{
    const f=fixture(),r=run(f),s=r.state;
    assert.equal(f.calls.length,[...r.history.materialSamples.values()].reduce((sum,a)=>sum+a.length,0));
    for(const record of f.tools){
        const tool=s.tools.find(t=>t.id===record.toolId),samples=r.history.materialSamples.get(record.toolId);
        assert.equal(tool.massPerMaterialLength,record.material.massPerMaterialLength);assert.equal(tool.dsDx,record.material.dsDx);
        for(const sample of samples){
            assert.equal(sample.bodyId,record.body.id);assert.equal(sample.toolId,record.toolId);
            close(sample.materialCoordinate,record.materialLabels[0]+record.material.dsDx*(sample.coordinate-record.nodeCoordinates[0]));
            close(sample.materialEnd-sample.materialStart,record.material.dsDx*(sample.end-sample.start));
            assert.deepEqual(sample.sourceEdges,record.nodeCoordinates.slice(0,-1).flatMap((a,i)=>a<sample.end&&record.nodeCoordinates[i+1]>sample.start?[record.body.activeStart+i]:[]));
            assert.equal(tool.materialAt(sample),sample.material);assert.ok(Object.isFrozen(sample.material.stiffness));
            assert.throws(()=>tool.materialAt({...sample,start:sample.start-.01}),/support-changed/);
        }
    }
    f.profile.factor=900;
    const callsBefore=f.calls.length;
    assert.doesNotThrow(()=>createCompositeJointAssembly({...s,inertia:null,elementBackend:'javascript'}));
    assert.equal(f.calls.length,callsBefore,'Joint assembly must use the owned material snapshot, not re-evaluate an external closure');
});

test('explicit material angular rates retain each original edge across the union grid, without inferring separate spin rates',()=>{
    const f=fixture();
    for(const [j,t] of f.tools.entries()) {
        t.angularVelocityInterpretation='physical-material-angular-velocity';
        for(const key of ['angularVelocityX','angularVelocityY','angularVelocityZ'])t.body[key].fill(NaN);
        for(let edge=t.body.activeStart;edge<t.body.activeEnd;edge++)
            ['angularVelocityX','angularVelocityY','angularVelocityZ'].forEach((key,k)=>t.body[key][edge]=(j?1:-1)*(edge+1)*(k+1));
    }
    const before=f.tools.map(t=>bodySnapshot(t.body)),r=run(f);
    assert.equal(r.history.includesAngularVelocity,true);assert.deepEqual(r.history.angularVelocityToolIds,['wire','catheter']);
    for(const source of f.tools)for(const {jointEdge,bodyEdge} of r.mappings.get(source.toolId).jointEdges) {
        const expected=read(source.body,bodyEdge,['angularVelocityX','angularVelocityY','angularVelocityZ']),record=r.state.materialVelocities[jointEdge].tools.find(t=>t.id===source.toolId);
        assert.deepEqual(record.angularVelocity,expected);assert.equal(record.angularVelocityInterpretation,'physical-material-angular-velocity');
        assert.equal(record.materialSpin,null);assert.equal(record.frameSpin,null);
    }
    assert.deepEqual(f.tools.map(t=>bodySnapshot(t.body)),before);
    const old=r.state.materialVelocities[0].tools[0].angularVelocity.slice();
    f.tools[0].body.angularVelocityX[1]=999;r.history.materialVelocities[0].tools[0].angularVelocity[0]=888;
    assert.deepEqual(r.state.materialVelocities[0].tools[0].angularVelocity,old);
    assert.deepEqual(r.history.sourceSnapshots[0].angularVelocity[0],old);
    assert.throws(()=>r.history.sourceSnapshots[0].angularVelocity[0][0]=777,TypeError);
    delete f.tools[1].angularVelocityInterpretation;
    const partial=run(f);assert.equal(partial.history.includesAngularVelocity,false);assert.deepEqual(partial.history.angularVelocityToolIds,['wire']);
    assert.ok(partial.state.materialVelocities.flatMap(e=>e.tools).filter(t=>t.id==='catheter').every(t=>t.angularVelocity===null));
});

test('declared angular rates reject nonphysical interpretations, missing storage and nonfinite active values before import',()=>{
    for(const change of [t=>t.angularVelocityInterpretation='quaternion-difference',t=>t.body.angularVelocityX=new Float64Array(1),
        t=>t.body.angularVelocityY[t.body.activeEnd-1]=NaN,t=>t.body.angularVelocityZ[t.body.activeStart]=Infinity]) {
        const f=fixture();f.tools[0].angularVelocityInterpretation='physical-material-angular-velocity';change(f.tools[0]);
        const before=f.tools.map(t=>bodySnapshot(t.body));assert.throws(()=>run(f),/angular-velocity-interpretation|body edges|finite/);
        assert.deepEqual(f.tools.map(t=>bodySnapshot(t.body)),before);
    }
});

test('source descriptors, frames, world buffers and output histories have separate ownership',()=>{
    const f=fixture(),r=run(f),saved=r.history.sourceSnapshots[0],old=saved.positions[0][0],angle=r.state.angles.get('wire')[0],dir=r.state.tools[0].reference[0].director[0];
    f.tools[0].body.x[1]=900;f.tools[0].body.velocityX[1]=901;f.tools[0].reference[0].director[0]=902;f.tools[0].angles[0]=903;f.tools[0].materialLabels[0]=904;f.tools[0].nodeCoordinates[0]=905;
    assert.equal(r.state.toolPositions.get('wire')[0][0],old);assert.equal(saved.positions[0][0],old);assert.equal(r.state.angles.get('wire')[0],angle);assert.equal(r.state.tools[0].reference[0].director[0],dir);
    r.state.positions[0][0]=-9;r.state.tools[0].reference[0].director[0]=-9;r.state.angles.get('wire')[0]=-9;
    assert.equal(f.tools[0].body.x[1],900);assert.equal(saved.angles[0],angle);assert.equal(saved.reference[0].director[0],dir);
});

test('single actual tool and staggered exposed ends retain own ranges without invented nodes',()=>{
    for(const single of ['wire','catheter']){
        const f=fixture(single==='wire'?{catheterX:null}:{wireX:null}),r=run(f);
        assert.deepEqual(Array.from(r.state.coordinates),f.tools[0].nodeCoordinates);assert.equal(r.state.modes.length,0);assert.equal(r.state.relative.length,0);assert.equal(r.state.relativeToolId,single);
    }
    const r=run(fixture({catheterX:[6,10,14,18,22]}));
    assert.deepEqual(r.state.layout.edgeToolIds[0],['wire']);assert.deepEqual(r.state.layout.edgeToolIds.at(-1),['catheter']);
    assert.equal(r.state.coordinates.at(-1),22);assert.equal(r.state.modes.at(-1).node,r.state.coordinates.length-2);
});

test('exact coordinate union never silently merges nearby source nodes',()=>{
    const nearby=10+2**-13,f=fixture({wireX:[0,5,10,15,20],catheterX:[2,6,nearby,14,18]});
    for(const record of f.tools){
        record.nodeCoordinates.forEach((x,i)=>{record.body.x[i+1]=x;record.body.y[i+1]=record.toolId==='wire'?.25:0;record.body.z[i+1]=0;});
        record.reference=captureCompositeReferenceFrames(record.nodeCoordinates.map((_,i)=>read(record.body,i+1)));
        record.referenceTwists=record.referenceTwists.map((_,i)=>2*Math.PI*(i+1));orient(record);
    }
    const r=run(f);
    assert.ok(r.state.coordinates.includes(10));assert.ok(r.state.coordinates.includes(nearby));assert.equal(r.state.coordinates.length,10);
    assert.equal(r.evidence.maxOriginalPositionError,0);
});

test('Float64 coordinate-change roundoff stays inside its arithmetic bound and preserves original Float32 publication',t=>{
    const f=fixture({catheterX:[2,6,10.0001,14,18]}),before=f.tools.map(t=>bodySnapshot(t.body));
    const r=run(f);assert.ok(r.evidence.maxOriginalNodeRoundoff>0);assert.equal(r.evidence.bitExactOriginalNodeReconstruction,false);
    assert.equal(r.evidence.exactOriginalNodeCoordinateCoverage,true);assert.equal(r.evidence.maxFloat32PublicationError,0);assert.equal(r.evidence.float32PublicationMismatchNodeCount,0);
    assert.equal(r.evidence.originalNodeRoundoff.length,10);
    for(const node of r.evidence.originalNodeRoundoff)node.errors.forEach((error,k)=>{assert.ok(error<=node.bounds[k]);assert.equal(node.float32PublicationErrors[k],0);});
    assert.deepEqual(f.tools.map(t=>bodySnapshot(t.body)),before);
    t.diagnostic(JSON.stringify({maxOriginalNodeRoundoff:r.evidence.maxOriginalNodeRoundoff,maxFloat32PublicationError:r.evidence.maxFloat32PublicationError,float32PublicationMismatchNodeCount:r.evidence.float32PublicationMismatchNodeCount}));
});

const invalid=[
    ['missing known winding',f=>delete f.tools[0].winding,/known-own-unwrapped-winding/],
    ['missing velocity interpretation',f=>delete f.tools[0].velocityInterpretation,/physical-material-velocity-interpretation/],
    ['nonfinite own spin',f=>f.tools[0].angles[0]=NaN,/finite/],
    ['missing own reference twist',f=>f.tools[0].referenceTwists.pop(),/reference twists/],
    ['wrong reference winding phase',f=>f.tools[0].referenceTwists[0]+=.2,/source-reference-winding/],
    ['stale own frame',f=>f.tools[0].reference[0].tangent=[0,0,1],/reference-does-not-belong/],
    ['wrong physical quaternion',f=>f.tools[0].body.orientationW[1]=.2,/body-orientation-disagrees/],
    ['missing compiled profile provider',f=>delete f.tools[0].material.materialAt,/profile-provider/],
    ['noncompiled provider output',f=>f.tools[0].material.materialAt=()=>({stiffness:[[1,0,0],[0,1,0],[0,0,1]],intrinsic:[0,0,0]}),/stiffness needs 9/],
    ['non-SPD profile',f=>f.tools[0].material.materialAt=()=>({stiffness:[-1,0,0,0,1,0,0,0,1],intrinsic:[0,0,0]}),/positive definite/],
    ['missing physical density',f=>delete f.tools[0].material.massPerMaterialLength,/finite/],
    ['zero material metric',f=>f.tools[0].material.dsDx=0,/positive/],
    ['nonaffine material labels',f=>f.tools[0].materialLabels[2]+=.001,/labels-disagree/],
    ['wrong actual rest metric',f=>f.tools[0].body.restLength[1]+=.001,/rest-length-disagrees/],
    ['unordered own coordinates',f=>f.tools[0].nodeCoordinates[2]=4,/strictly-increase/],
    ['full rather than active coordinate array',f=>f.tools[0].nodeCoordinates.unshift(-5),/coordinates needs 5/],
    ['implicit guidewire alias',f=>f.tools[0].toolId='guidewire',/body-to-physical-tool-mapping/],
    ['unknown actual body id',f=>f.tools[0].body.id='other',/body-to-physical-tool-mapping/],
    ['nonfinite active velocity',f=>f.tools[0].body.velocityY[2]=NaN,/finite/],
    ['empty active edge range',f=>f.tools[0].body.activeEnd=1,/active-physical-range/],
    ['duplicate body',f=>f.tools[1]=f.tools[0],/duplicate-physical/],
];
for(const [name,change,pattern] of invalid)test(`invalid import rejects ${name} without a World mutation`,()=>{
    const f=fixture();change(f);const before=f.tools.map(t=>bodySnapshot(t.body));assert.throws(()=>run(f),pattern);assert.deepEqual(f.tools.map(t=>bodySnapshot(t.body)),before);
});

test('disjoint or single touching intervals and insufficient union charts reject without synthesizing overlap',()=>{
    assert.throws(()=>run(fixture({catheterX:[21,25,29]})),/disconnected-active/);
    assert.throws(()=>run(fixture({catheterX:[20,24,28]})),/disconnected-active/);
    assert.throws(()=>run(fixture({wireX:[0,5],catheterX:null})),/at-least-three/);
});

test('a source rest difference smaller than Float32 storage resolution is accepted only at the exact stored rounding',()=>{
    const f=fixture(),source=f.tools[0],expected=source.material.dsDx*5;
    source.material.dsDx=1.20000000001;source.materialLabels=source.nodeCoordinates.map(x=>50+source.material.dsDx*x);
    assert.equal(Math.fround(source.material.dsDx*5),Math.fround(expected));assert.doesNotThrow(()=>run(f));
    source.body.restLength[1]=Math.fround(expected+1e-5);assert.throws(()=>run(f),/rest-length-disagrees/);
});
