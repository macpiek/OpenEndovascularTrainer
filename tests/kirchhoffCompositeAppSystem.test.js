import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES} from '../src/physics/endovascularPhysicsWorld.js';
import {applyKirchhoffMaterialProfile} from '../src/physics/applyKirchhoffMaterialProfile.js';
import {createCompositeAppToolSource} from '../src/physics/kirchhoffCompositeAppInputs.js';
import {createCompositeJointAppSystem} from '../src/physics/kirchhoffCompositeAppSystem.js';

const dt=1/120;
function fixture({sheath=false,rotate=false,geometry='continuous-material-frame',cooperative=false}={}) {
    const controls={initializations:0,preparations:0,labelShift:0,rotate,guard:false,readState:null},bodies=[];
    const system=createCompositeJointAppSystem({geometry,cooperative,
        readToolSources(world) {
            controls.initializations++;
            return bodies.map(({body,toolId,type,tip})=>createCompositeAppToolSource({body,toolId,
                nodeCoordinates:[-8,-4,0],materialLabels:[-8,-4,0],unwrappedAngles:[0,0],referenceWindingTurns:[0],
                massPerMaterialLength:body.mass/4,profile:{type,tipMaterialCoordinate:tip}}));
        },
        readControls({state,bindings,mappings}) {
            controls.preparations++;controls.readState=state;
            assert.equal(bindings.length,2);assert.ok(Object.isFrozen(bindings));assert.ok(Object.isFrozen(bindings[0].nodes[0]));
            assert.equal(mappings.get('wire').nodes[0].node,0);
            mappings.clear(); // The next callback must receive its intact own snapshot.
            const result={commands:state.tools.map(t=>({toolId:t.id,labelShift:controls.labelShift,
                feedVelocity:-controls.labelShift/dt,spinIncrement:controls.rotate?(t.id==='wire'?.001:-.002):0})),
                positionBoundaries:state.tools.map(t=>({toolId:t.id,node:0,value:state.toolPositions.get(t.id)[0].slice()})),
                reservoir:({toolId})=>({id:toolId,sStart:-10,sEnd:-8,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})};
            if(controls.guard)result.state=state;
            return result;
        }
    });
    const world=new EndovascularPhysicsWorld({fixedDt:dt,wholeStepSystem:system});
    for(const [toolId,id,type,offset,tip] of [['wire','guidewire','glidewire',.03,0],['catheter','catheter','pigtail',0,80]]) {
        const body=world.createRod(id,3,4,DEFAULT_TOOL_PROFILES[id]);
        [-8,-4,0].forEach((x,i)=>{body.setNodePosition(i,x,offset,0);body.materialCoordinate[i]=x;});
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});applyKirchhoffMaterialProfile(body,type,{tipCoordinate:tip});
        bodies.push({body,toolId,type,tip});
    }
    if(sheath)world.addSheath({start:{x:-12,y:0,z:0},end:{x:4,y:0,z:0},innerRadius:2,bodies:bodies.map(t=>t.body)});
    return {system,world,controls,bodies};
}
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,message:r.message,diagnostics:r.diagnostics}));

test('application factory runs two actual World dt, independent handle rotation and complete newest curve publication',()=>{
    const f=fixture({sheath:true,rotate:true});
    for(let i=0;i<2;i++) {
        assert.equal(f.world.advance(dt),1,JSON.stringify(f.world.lastStepResult));accepted(f.world.lastStepResult);
        const state=f.system.snapshot();assert.equal(state.step,i+1);assert.equal(state.time,(i+1)*dt);
        assert.equal(state.elasticityGeometry,'continuous-material-frame');assert.equal(state.lengthGeometry,'continuous-arclength');
        assert.ok(Math.abs(state.angles.get('wire')[0]-(i+1)*.001)<1e-12);
        assert.ok(Math.abs(state.angles.get('catheter')[0]+(i+1)*.002)<1e-12);
        for(const {body} of f.bodies){assert.equal(body.jointStateView.continuousCurve.kind,'quintic-bernstein');assert.equal(body.jointStateView.step,i+1);}
        assert.ok(f.world.lastStepResult.diagnostics.certificate.sheath.converged);
    }
    assert.equal(f.controls.initializations,1);assert.equal(f.controls.preparations,2);
    assert.equal(f.system.diagnostics.publications,2);assert.equal(f.world.accumulator,0);
    f.controls.readState.positions[0][0]=999;assert.notEqual(f.system.snapshot().positions[0][0],999);
});

test('accepted state and labels survive budget retry without resampling controls or source bodies',()=>{
    const f=fixture({rotate:true});f.controls.labelShift=-.01;f.system.setBudget({directions:0});
    assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'direction-budget');
    assert.equal(f.system.snapshot().step,0);assert.equal(f.system.snapshot().tools[0].appMaterialProfile.materialOrigin,-8);
    f.controls.labelShift=-2;f.system.setBudget(null);
    const r=f.world.stepFixed();accepted(r);
    assert.equal(f.controls.initializations,1);assert.equal(f.controls.preparations,1);
    const state=f.system.snapshot();for(const t of state.tools)assert.equal(t.appMaterialProfile.materialOrigin,-8.01);
    assert.equal(state.step,1);
});

test('source range changes and active unsupported containment reject; resets clear imported mapping and publication',()=>{
    const f=fixture();accepted(f.system.step(f.world,dt));
    f.bodies[0].body.setActiveRange(1,2);
    const changed=f.system.step(f.world,dt);assert.equal(changed.accepted,false);assert.equal(changed.status,'joint-world-remap-required');
    f.system.reset(f.world);assert.equal(f.system.snapshot(),null);assert.equal(f.system.diagnostics.appSourceInitialized,false);
    assert.ok(f.bodies.every(({body})=>body.jointStateView===undefined));
    f.bodies[0].body.setActiveRange(0,2);
    // Source winding must be declared by the source callback after reset;
    // this fixture has no spin and keeps its known initial zero epoch.
    f.world.addContainment(f.bodies[0].body,f.bodies[1].body,{innerRadius:1});
    const unsupported=f.system.step(f.world,dt);assert.equal(unsupported.accepted,false);assert.equal(unsupported.status,'joint-world-containment-adapter-required');
    assert.equal(f.controls.initializations,2);assert.equal(f.system.snapshot().step,0);
});

test('control callback cannot replace accepted physical state or pending dt',()=>{
    const f=fixture();f.controls.guard=true;
    const r=f.system.step(f.world,dt);assert.equal(r.accepted,false);assert.match(r.message,/cannot replace/);
    assert.equal(f.system.snapshot().step,0);
});

test('native geometry selects the same common/relative Joint core with native inertia and lengths for successive independent controls',()=>{
    const f=fixture({sheath:true,rotate:true,geometry:'native-discrete-rod',cooperative:true});
    for(let step=1;step<=2;step++) {
        let result=f.system.step(f.world,dt),attempts=1;
        while(result.status==='computing'&&attempts++<100)result=f.system.step(f.world,dt);
        accepted(result);
        const state=f.system.snapshot();assert.equal(state.step,step);assert.equal(state.inertiaGeometryByTool,null);
        assert.equal(state.elasticityGeometry,'native-discrete-rod');assert.equal(state.lengthGeometry,'native-chords');
        assert.equal(result.diagnostics.scope,'fixed-topology-full-relative-nonlinear-dt');
        assert.equal(result.diagnostics.inertiaGeometry,'affine');assert.ok(result.diagnostics.relativeDofs>0);
        assert.ok(Math.abs(state.angles.get('wire')[0]-step*.001)<1e-12);
        assert.ok(Math.abs(state.angles.get('catheter')[0]+step*.002)<1e-12);
        assert.deepEqual(state.materialVelocities.map(e=>e.tools.map(t=>t.interpretation)),
            Array.from({length:2},()=>['physical-material-velocity','physical-material-velocity']));
    }
    assert.equal(f.controls.initializations,1);assert.equal(f.controls.preparations,2);
    assert.equal(f.system.diagnostics.elasticityGeometry,'native-discrete-rod');
    assert.equal(f.system.diagnostics.lengthGeometry,'native-chords');
});
