import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeTimeStepState, advanceCompositeTimeStep } from '../src/physics/kirchhoffCompositeTimeStep.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';

const dt=1/120,close=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function setup({height=.82,velocity=[0,-10,0],count=5}={}) {
    const positions=Array.from({length:count},(_,i)=>[2*i,height,0]);
    const layout=createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter']));
    const data={positions,coordinates:positions.map(p=>p[0]),reference:captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:new Float64Array(count-1),material:compileCompositeMaterial({EI1:1,GJ:2})},
        {id:'catheter',angles:new Float64Array(count-1),material:compileCompositeMaterial({EI1:3,GJ:5})}]};
    const state=createCompositeTimeStepState({data,layout}),calls=[];
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        calls.push(radius);const t=ay===by?.5:ay<by?0:1;
        out.signedGap=(1-t)*ay+t*by-radius;out.segmentT=t;out.inward.values.set([0,1,0]);
        out.closestPoint.values.set([(1-t)*ax+t*bx,0,(1-t)*az+t*bz]);
        out.faceIndex=1;out.branchId=0;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const options={dt,torsionMode:'quasi-static',prescribed:[...layout.spins.values()].map(s=>({dof:s[0],value:0})),
        inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?.01:.02,
            materialMap:{sStart:100+2*e,dsDx:1,dsDt:0},oldMaterialVelocities:[[...velocity],[...velocity]]}))})),
        tolerances:{force:1e-7,torque:1e-8,length:1e-8,linear:1e-10},initialPenalty:1e4,
        budget:{directions:150,outerIterations:32,evaluations:2000,lineSearchTrials:30},
        wall:{field,friction:'frictionless',initialPenalty:1e5,
            tolerances:{gap:1e-8,force:1e-7,work:1e-7},
            contactOwners:{edges:Array.from({length:count-1},(_,edge)=>({edge,wall:{owner:'catheter',radius:.8}}))}}};
    return {state,options,calls,velocity};
}
const accept=r=>assert.ok(r.accepted,JSON.stringify({status:r.status,...r.diagnostics}));

test('one common chain stops at the outer catheter wall and balances the sum of material momentum changes',()=>{
    const f=setup(),before=structuredClone(f.state),r=advanceCompositeTimeStep(f.state,f.options);accept(r);
    assert.deepEqual(f.state,before);assert.ok(f.calls.every(radius=>radius===.8));
    assert.equal(r.diagnostics.contactScope,'discrete-frictionless-vessel-contact');
    assert.ok(r.diagnostics.certificate.wall.converged);
    for(const p of r.state.data.positions)close(p[1],.8,1e-8);
    const force=r.state.wallContactState.multipliers.reduce((a,b)=>a+b,0);
    let rate=0;
    r.state.materialVelocities.forEach((edge,i)=>edge.tools.forEach(tool=>{
        const input=f.options.inertiaEdges[i].tools.find(t=>t.id===tool.id);
        rate+=input.massPerMaterialLength*2/dt*(.5*(tool.velocities[0][1]+tool.velocities[1][1])-f.velocity[1]);
    }));
    close(force,rate,1e-8);close(force,.03*8*(10-.02/dt)/dt,1e-8);
    assert.ok(r.diagnostics.wallDualUpdates>0);assert.equal(r.diagnostics.historyCommits,1);
});

test('free tangential translation has no artificial friction and an unloaded gap has no reaction',()=>{
    const f=setup({height:1,velocity:[3,0,-2]}),r=advanceCompositeTimeStep(f.state,f.options);accept(r);
    r.state.data.positions.forEach((p,i)=>{close(p[0],2*i+3*dt);close(p[1],1);close(p[2],-2*dt);});
    assert.ok(r.state.wallContactState.multipliers.every(x=>x===0));
});

test('rejected wall solves commit neither staged reaction nor geometry or accepted time',()=>{
    const f=setup(),before=structuredClone(f.state);
    const r=advanceCompositeTimeStep(f.state,{...f.options,budget:{directions:30,outerIterations:1,evaluations:100,lineSearchTrials:15}});
    assert.equal(r.accepted,false);assert.equal(r.state,f.state);assert.deepEqual(f.state,before);
    assert.ok(r.diagnostics.wallDualUpdates>0);assert.equal(r.diagnostics.historyCommits,0);
});

test('a second physical step keeps wall history and can release its load when moving away',()=>{
    const f=setup(),first=advanceCompositeTimeStep(f.state,f.options);accept(first);
    const inertiaEdges=structuredClone(f.options.inertiaEdges);
    for(const edge of inertiaEdges)for(const tool of edge.tools)tool.oldMaterialVelocities=[[0,1,0],[0,1,0]];
    const second=advanceCompositeTimeStep(first.state,{...f.options,inertiaEdges});accept(second);
    assert.equal(second.state.step,2);close(second.state.time,2*dt);
    assert.ok(second.state.wallContactState.multipliers.every(x=>x===0));
    for(const p of second.state.data.positions)close(p[1],.8+dt,1e-8);
    assert.throws(()=>advanceCompositeTimeStep(first.state,{...f.options,wall:null}),/wall history/);
    const changed={...f.options.wall,contactOwners:structuredClone(f.options.wall.contactOwners)};
    changed.contactOwners.edges[0].wall.owner='wire';
    assert.throws(()=>advanceCompositeTimeStep(first.state,{...f.options,wall:changed}),/history transfer/);
});

test('zero complementarity and fixed positions do not hide penetration; missing friction choice is explicit',()=>{
    const f=setup({height:.79,velocity:[0,0,0]});
    for(let i=0;i<f.state.layout.nodeCount;i++)for(let j=0;j<3;j++)f.options.prescribed.push({dof:f.state.layout.positions[i]+j,value:f.state.data.positions[i][j]});
    const r=advanceCompositeTimeStep(f.state,{...f.options,budget:{outerIterations:2}});
    assert.equal(r.accepted,false);assert.ok(r.diagnostics.certificate.wall.maximumPenetration>.009);
    assert.throws(()=>advanceCompositeTimeStep(f.state,{...f.options,wall:{...f.options.wall,friction:undefined}}),/frictionless/);
});
