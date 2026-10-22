import assert from 'node:assert/strict';
import test from 'node:test';
import {DEFAULT_TOOL_PROFILES,EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis,SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {initializeElasticIntroducer,introducerAccessBoundary} from '../src/devices/introducerMechanics.js';
import {BERENSTEIN_NATURAL_BEND_ANGLE_RAD} from '../src/physics/catheterMaterialProfile.js';
import {GUIDEWIRE_RADIUS_MM,PIGTAIL_CATHETER_RADIUS_MM} from '../src/toolDimensions.js';

const DT = 1 / 120;

function maximumBendDegrees(body) {
    let maximum = 0;
    let node = -1;
    for (let index = body.activeStart + 1; index < body.activeEnd; index++) {
        const ax = body.x[index] - body.x[index - 1];
        const ay = body.y[index] - body.y[index - 1];
        const az = body.z[index] - body.z[index - 1];
        const bx = body.x[index + 1] - body.x[index];
        const by = body.y[index + 1] - body.y[index];
        const bz = body.z[index + 1] - body.z[index];
        const denominator = Math.hypot(ax, ay, az) * Math.hypot(bx, by, bz);
        if (denominator <= 1e-9) continue;
        const cosine = Math.max(-1, Math.min(1,
            (ax * bx + ay * by + az * bz) / denominator
        ));
        const angle = Math.acos(cosine) * 180 / Math.PI;
        if (angle <= maximum) continue;
        maximum = angle;
        node = index;
    }
    return { maximum, node };
}

test('the shared-axis loading hub supports the full Berenstein preform during wire feed and withdrawal', () => {
    const sheath={start:{x:-100,y:0,z:0},end:{x:-30,y:0,z:0}};
    initializeElasticIntroducer(sheath);
    const boundary=introducerAccessBoundary(sheath);
    const tools=[
        {id:'wire',type:'glidewire',insertion:0,rotation:0,radius:GUIDEWIRE_RADIUS_MM,shaftStiffness:13.15,tipStiffness:19.4},
        {id:'catheter',type:'berenstein',insertion:0,rotation:0,radius:PIGTAIL_CATHETER_RADIUS_MM,shaftStiffness:40.65,tipStiffness:100},
        {id:'sheath',type:'introducer-sheath',insertion:70,rotation:0,radius:sheath.radius,length:70,shaftStiffness:1,tipStiffness:1}
    ];
    let state=createSharedAxisNative({...createSharedAxisContacts({sheath:boundary,localCoordinates:true}),tools});
    let rotations=Object.fromEntries(tools.map(t=>[t.id,0]));
    const initialCoordinates=state.materials.find(m=>m.spec.id==='catheter').coordinates.slice();
    let peak={maximum:0,node:-1};
    // The zero-insertion material still extends into the 40 mm loading hub.
    // Inspect the actual Kirchhoff material, not the app's two-node transport
    // view of a fully withdrawn catheter (which has no internal bending joint).
    for(const insertion of [0,0,...Array.from({length:100},(_,i)=>i+1),...Array.from({length:100},(_,i)=>99-i)]) {
        tools[0].insertion=insertion;
        const step=advanceSharedAxis(state,rotations,DT,tools,SHARED_AXIS_APP_PHYSICS_OPTIONS);
        let result;do{result=step.next();}while(!result.done);
        assert.ok(result.value.state,JSON.stringify(result.value.result));
        ({state,rotations}=result.value);
        const material=state.materials.find(m=>m.spec.id==='catheter'),body=material.body;
        const restTurn=Array.from(body.restRotation1).reduce((sum,value)=>sum+value,0);
        assert.ok(Math.abs(restTurn-BERENSTEIN_NATURAL_BEND_ANGLE_RAD)<1e-9,
            `the manufactured preform was changed: ${restTurn}`);
        assert.deepEqual(material.coordinates,initialCoordinates,'wire feed cannot move the catheter material');
        assert.equal(material.spec.insertion,0);
        for(let i=0;i<body.count;i++) {
            assert.ok(Number.isFinite(body.x[i])&&Number.isFinite(body.y[i])&&Number.isFinite(body.z[i]));
            assert.ok(Math.hypot(body.y[i],body.z[i])<=boundary.innerRadius-sheath.radius+1e-5,
                'the common shaft must remain inside the loading sleeve');
        }
        const bend=maximumBendDegrees(body);if(bend.maximum>peak.maximum)peak=bend;
    }
    assert.ok(peak.maximum<5,`Berenstein kinked at node ${peak.node} (${peak.maximum} degrees)`);
});

test('proximal sheath extension supplies radial contact without axial tracking', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, iterations: 4 });
    const body = world.createRod('axially-free-loading-hub', 4, 1, {
        ...DEFAULT_TOOL_PROFILES.guidewire,
        radius: 0.1,
        linearDamping: 1,
        stretchCompliance: 0
    });
    for (let index = 0; index < body.count; index++) {
        body.setNodePosition(index, -8 + index, 1.2, 0);
        body.velocityX[index] = 12;
    }
    world.addSheath({
        start: { x: 0, y: 0, z: 0 },
        end: { x: 10, y: 0, z: 0 },
        innerRadius: 0.9,
        proximalExtension: 10,
        bodies: [body]
    });

    const initialX = body.x[0];
    for (let step = 0; step < 8; step++) world.stepFixed();
    assert.ok(body.x[0] > initialX + 0.5, 'loading support applied an axial position track');
    for (let index = 0; index < body.count; index++) {
        assert.ok(
            Math.hypot(body.y[index], body.z[index]) <= 0.801,
            'loading support failed to apply the radial lumen clearance'
        );
    }
});
