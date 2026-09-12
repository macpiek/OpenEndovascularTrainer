import test from 'node:test';
import assert from 'node:assert/strict';
import {RodState} from '../src/physics/rodState.js';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {PigtailCatheter} from '../src/pigtailCatheter.js';

test('Joint catheter layout retains the physical tip and material lengths through fractional feed and withdrawal',()=>{
    const wire=new RodState(51,5),vessel={sheath:{start:{x:0,y:0,z:0},end:{x:20,y:0,z:0}},segments:[]};
    wire.nodes.forEach((node,i)=>{node.x=i*5-200;node.y=0;node.z=0;});
    const catheter=new PigtailCatheter({wire,segmentLength:5,guidewireLength:250,tailProgressRef:()=>50,vessel,retainMaterialTip:true}),
        world=new EndovascularPhysicsWorld(),body=world.createRod('catheter',320,4);
    catheter.setType('berenstein');
    for(const progress of [.13333333333333333,.4,3.9,4,4.133333333333333,19.9,20.1,21.9,22,.2,0]) {
        catheter.progress=progress;catheter.guidewireInserted=50;
        catheter.stepPhysics(1/120);catheter.syncXpbdBody(body);
        const end=body.activeEnd,tip=body.materialCoordinate[end];
        assert.ok(Math.abs(tip-progress)<1e-12,`Physical tip ${progress} was replaced by ${tip}`);
        for(let e=body.activeStart;e<end;e++) {
            const length=body.materialCoordinate[e+1]-body.materialCoordinate[e];
            assert.ok(length>0,`Collapsed material edge at ${progress}:${e}`);
            assert.equal(body.restLength[e],Math.fround(length),`Changed material length at ${progress}:${e}`);
        }
    }
    catheter.dispose();
});
