import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeAppInletReservoir} from '../src/physics/kirchhoffCompositeAppReservoir.js';

function fixture() {
    const ids=['wire','catheter'],state={step:0,coordinates:[0,1],layout:{edgeToolIds:[ids]},
        tools:ids.map(id=>({id,appMaterialProfile:{materialOrigin:-4,coordinateOrigin:0},reference:[{tangent:[0,1,0],director:[1,0,0]}]})),
        toolPositions:new Map(ids.map((id,j)=>[id,[[j,2,3],[j,3,3]]])),
        angles:new Map([['wire',[4*Math.PI]],['catheter',[-2*Math.PI]]]),
        materialVelocities:[{tools:ids.map((id,j)=>({id,sStart:-4,sEnd:-3,velocities:[[0,j+1,0],[0,j+2,0]],
            interpretation:'physical-material-velocity',angularVelocity:[0,j+.5,0],angularVelocityInterpretation:'physical-material-angular-velocity'}))}]};
    return {state,tools:ids.map(toolId=>({toolId,materialLabels:[-6,-3]}))};
}
test('explicit inlet continues each accepted native tangent, winding and rate without reading source body poses',()=>{
    const f=fixture(),before=structuredClone(f.state),reservoir=createCompositeAppInletReservoir(f);
    const wire=reservoir({toolId:'wire',s:-5}),cat=reservoir({toolId:'catheter',s:-5});
    assert.deepEqual(wire.positions,[[0,0,3],[0,2,3]]);assert.deepEqual(cat.positions,[[1,0,3],[1,2,3]]);
    assert.equal(wire.angle,4*Math.PI);assert.equal(cat.angle,-2*Math.PI);
    assert.deepEqual(wire.velocities,[[0,1,0],[0,1,0]]);assert.deepEqual(cat.angularVelocity,[0,1.5,0]);
    assert.equal(reservoir({toolId:'wire',s:-7}),null);assert.equal(reservoir({toolId:'wire',s:-3}),null);
    wire.positions[1][0]=123;wire.reference.director[0]=0;assert.deepEqual(f.state,before);
});
test('accepted steps cannot manufacture missing incoming angular history as zero motion',()=>{
    const f=fixture();f.state.step=1;
    assert.throws(()=>createCompositeAppInletReservoir(f),/angular-rate history/);
});
