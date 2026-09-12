import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES} from '../src/physics/endovascularPhysicsWorld.js';
import {applyKirchhoffMaterialProfile} from '../src/physics/applyKirchhoffMaterialProfile.js';
import {createCompositeAppToolSource} from '../src/physics/kirchhoffCompositeAppInputs.js';
import {createCompositeJointAppSystem} from '../src/physics/kirchhoffCompositeAppSystem.js';
import {createCompositeAppInletReservoir} from '../src/physics/kirchhoffCompositeAppReservoir.js';
import {transferCompositeNativeState} from '../src/physics/kirchhoffCompositeNativeTransfer.js';
import {sampleCompositeNativeRateHistory} from '../src/physics/kirchhoffCompositeNativeRateHistory.js';

const dt=1/120;
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,message:r.message,diagnostics:r.diagnostics}));
function fixture() {
    const controls={wireProgress:4,catProgress:4,wireStart:2,catCount:4,layouts:0,preparations:0,initializations:0,lastRead:null};
    const bodies=[];
    function layout() {
        return bodies.map(({body,toolId,type})=>{
            const wire=toolId==='wire',progress=wire?controls.wireProgress:controls.catProgress;
            body.setActiveRange(wire?controls.wireStart:0,wire?4:controls.catCount-1);
            const x=wire?Array.from({length:5-controls.wireStart},(_,i)=>(controls.wireStart+i-4)*4+progress):
                controls.catCount===4?[-8,-4,0,progress]:[-8,-4,0,progress/2,progress];
            x.slice(0,-1).forEach((v,i)=>body.restLength[body.activeStart+i]=x[i+1]-v);
            return {body,toolId,nodeCoordinates:x,materialLabels:x.map(v=>v-progress),profile:{type,tipMaterialCoordinate:0}};
        });
    }
    const system=createCompositeJointAppSystem({geometry:'native-discrete-rod',worldWall:{rateMode:'backward-euler-grid'},
        readToolSources() {
            controls.initializations++;
            return layout().map(s=>createCompositeAppToolSource({...s,massPerMaterialLength:s.body.mass/4,
                unwrappedAngles:Array(s.nodeCoordinates.length-1).fill(s.toolId==='wire'?4*Math.PI:-6*Math.PI),
                referenceWindingTurns:Array(s.nodeCoordinates.length-2).fill(0)}));
        },
        readNativeLayout({state}) {controls.layouts++;const tools=layout();return {tools,reservoir:createCompositeAppInletReservoir({state,tools})};},
        readControls({state,bindings}) {
            controls.preparations++;controls.lastRead=state;
            const positionBoundaries=bindings.flatMap(b=>b.nodes.filter(r=>b.toolId==='wire'||state.coordinates[r.jointNode]===0).map(r=>({toolId:b.toolId,node:r.jointNode,
                value:b.toolId==='wire'?[state.coordinates[r.jointNode],.03,0]:[0,0,0]})));
            return {commands:state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:t.id==='wire'?.0001:-.0002})),positionBoundaries};
        }
    });
    const world=new EndovascularPhysicsWorld({fixedDt:dt,wholeStepSystem:system});
    for(const [toolId,id,type] of [['wire','guidewire','glidewire'],['catheter','catheter','berenstein']]) {
        const body=world.createRod(id,5,4,DEFAULT_TOOL_PROFILES[id]),wire=toolId==='wire';
        Array.from({length:5},(_,i)=>i).forEach(i=>{body.setNodePosition(i,wire?(i-4)*4+4:i*4-8,wire?.03:0,0);body.materialCoordinate[i]=i*4-16;});
        body.setActiveRange(wire?2:0,wire?4:3);body.captureKirchhoffRestConfiguration({captureRestRotation:false});
        applyKirchhoffMaterialProfile(body,type,{tipCoordinate:0});bodies.push({body,toolId,type});
    }
    world.addSheath({start:{x:-10,y:0,z:0},end:{x:8,y:0,z:0},innerRadius:2,bodies:bodies.map(t=>t.body)});
    return {controls,bodies,world,system,layout};
}

test('native feed remaps own material history for fixed count, active range, hold and one pending budget retry',()=>{
    const f=fixture();accepted(f.system.step(f.world,dt));
    const initial=f.system.snapshot();assert.equal(initial.step,1);
    f.controls.wireProgress+=.2;f.controls.catProgress+=.1;
    accepted(f.system.step(f.world,dt));
    const fed=f.system.snapshot();assert.equal(fed.step,2);
    assert.equal(f.controls.lastRead.tools.find(t=>t.id==='catheter').appMaterialProfile.materialOrigin,-12.1);
    const inlet=Array.from(fed.coordinates).indexOf(0);assert.deepEqual(fed.toolPositions.get('catheter')[inlet],[0,0,0]);
    assert.ok(Math.abs(fed.toolPositions.get('wire').at(-1)[0]-4.2)<1e-10);
    assert.ok(fed.angles.get('wire')[fed.layout.edgeToolIds.findIndex(ids=>ids.includes('wire'))]>4*Math.PI);
    assert.ok(fed.angles.get('catheter')[0]<-6*Math.PI);
    f.controls.wireStart=1;f.controls.catCount=5;f.system.setBudget({directions:0});
    const retry=f.system.step(f.world,dt);assert.equal(retry.status,'direction-budget',JSON.stringify(retry));
    assert.equal(f.system.snapshot().step,2);const sampled=f.controls.layouts,prepared=f.controls.preparations;
    f.system.setBudget(null);accepted(f.system.step(f.world,dt));
    assert.equal(f.controls.layouts,sampled);assert.equal(f.controls.preparations,prepared);
    const expanded=f.system.snapshot();assert.equal(expanded.step,3);assert.ok(expanded.layout.nodeCount>fed.layout.nodeCount);
    accepted(f.system.step(f.world,dt));assert.equal(f.system.snapshot().step,4);
    assert.equal(f.controls.initializations,1);assert.equal(f.controls.preparations,4);
});

test('native transfer retains incoming rates and unwrapped material pose independently of mutated Float32 views',()=>{
    const f=fixture();accepted(f.system.step(f.world,dt));const state=f.system.snapshot();
    f.controls.wireStart=1;f.controls.catProgress+=.1;const tools=f.layout(),reservoir=createCompositeAppInletReservoir({state,tools});
    for(const {body} of f.bodies){body.x.fill(999);body.velocityY.fill(888);body.orientationW.fill(.3);}
    const result=transferCompositeNativeState({state,tools,reservoir,contactRateMode:'backward-euler-grid'});
    assert.equal(result.state.step,state.step);assert.equal(result.state.time,state.time);
    assert.equal(result.diagnostics.warmStartReset,true);assert.ok(result.diagnostics.reservoirSamples>0);
    assert.equal(result.state.sheathContactState,undefined);
    for(const tool of state.tools) {
        const old=sampleCompositeNativeRateHistory({history:state.nativeRateHistory,toolId:tool.id,label:-2}),
            own=sampleCompositeNativeRateHistory({history:result.state.nativeRateHistory,toolId:tool.id,label:-2});
        assert.deepEqual(own.position,old.position);assert.deepEqual(own.velocity,old.velocity);assert.deepEqual(own.angularVelocity,old.angularVelocity);
    }
    const supply=sampleCompositeNativeRateHistory({history:result.state.nativeRateHistory,toolId:'wire',label:-10});
    assert.ok(supply.position[0]<0);assert.ok(result.state.positions.every(p=>p[0]!==999));
    const frame=result.state.tools.find(t=>t.id==='wire').reference.find(Boolean);
    assert.ok(Math.abs(Math.hypot(...frame.director)-1)<1e-12);
    assert.throws(()=>transferCompositeNativeState({state:{...state,wallFrictionState:{rateMode:'finite-displacement'}},tools,reservoir}),/Finite-displacement/);
});

test('cross-tool grid coincidence is reconciled only at arithmetic resolution, retaining material labels and genuine short intervals',()=>{
    const f=fixture();accepted(f.system.step(f.world,dt));const state=f.system.snapshot();
    for(const [shift,merge] of [[32*Number.EPSILON*8,true],[1e-7,false]]) {
        f.controls.wireProgress=4+shift;
        const tools=f.layout(),result=transferCompositeNativeState({state,tools,reservoir:createCompositeAppInletReservoir({state,tools}),contactRateMode:'backward-euler-grid'});
        assert.ok(result);
        const wire=result.bindings.find(b=>b.toolId==='wire'),cat=result.bindings.find(b=>b.toolId==='catheter');
        assert.equal(wire.nodes[1].jointNode===cat.nodes[2].jointNode,merge);
        assert.deepEqual(wire.nodes.map(r=>r.materialLabel),[-8,-4,0]);
        assert.equal(result.state.time,state.time);assert.equal(result.state.step,state.step);
        if(merge) {
            assert.ok(result.diagnostics.coordinateRoundoff.mergedPoints>0);
            assert.ok(result.diagnostics.coordinateRoundoff.maximumAdjustment<=result.diagnostics.coordinateRoundoff.resolution);
            assert.ok(result.state.coordinates.slice(1).every((x,i)=>x-result.state.coordinates[i]>=4));
        } else {
            assert.equal(result.diagnostics.coordinateRoundoff.mergedPoints,0);
            assert.ok(result.state.coordinates.some((x,i)=>i&&x-result.state.coordinates[i]<2e-7));
        }
    }
});
