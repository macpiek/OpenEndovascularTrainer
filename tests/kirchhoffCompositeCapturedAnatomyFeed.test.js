import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeAnatomyField} from './helpers/compositeAnatomyField.js';
import {createCapturedCompositeAppFixture} from './helpers/capturedCompositeAppFixture.js';

test('actual exposed anatomy feed crosses the loaded SDF cell and source witness changes through 130mm',()=>{
    const anatomy=createCompositeAnatomyField(),f=createCapturedCompositeAppFixture({wireRate:44,coordinateOrigin:'sheath-start',
        worldWall:{source:'original-field',contactMode:'capsule',rateMode:'backward-euler-grid'}});
    f.world.contactField=anatomy.field;let loadedSteps=0,frictionSteps=0,nonmonotoneTrials=0,duplicateHolds=0;
    try {
        for(let step=0;step<357;step++) {
            f.prepare(1,0);const result=f.system.step(f.world,f.dt),c=result.diagnostics?.certificate;
            assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate:c}));
            nonmonotoneTrials+=result.diagnostics.nonmonotoneAcceptedTrials;
            duplicateHolds+=result.diagnostics.duplicateConstraintHolds??0;
            assert.equal(c.converged,true);assert.ok(c.force<=1e-7);
            if(c.wall?.samples.some(s=>s.Fn>0)){loadedSteps++;assert.equal(c.wall.converged,true);}
            if(c.wallFriction){frictionSteps++;assert.equal(c.wallFriction.converged,true);}
            f.bodies.get('wire').syncToRodState(f.rod);
        }
        assert.ok(loadedSteps>5,'The replay must solve loaded vessel contact, not only the empty sheath');
        assert.ok(duplicateHolds>0,'Actual coupled contact must exercise the exact repeated-equation solve');
        assert.ok(nonmonotoneTrials>0,'The real SDF seam must exercise a private merit increase before final convergence');
        assert.ok(frictionSteps>5);assert.equal(f.system.snapshot().step,357);
        assert.ok(f.transport.progress>130);
    } finally {f.catheter.dispose();anatomy.dispose();}
});
