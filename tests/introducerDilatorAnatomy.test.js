import test from 'node:test';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createSharedAxisAppSystem,SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {DEFAULT_VESSEL_ELASTICITY} from '../src/physics/vesselElasticity.js';
import {initializeElasticIntroducer,elasticIntroducerTool,introducerAccessBoundary,publishElasticIntroducer} from '../src/devices/introducerMechanics.js';
import {initializeIntroducerDilator,elasticDilatorTool,prepareDilatorMotion,publishElasticDilator} from '../src/devices/introducerDilator.js';

test('matching dilator seats and withdraws over a wire in the actual femoral anatomy',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        for(const side of ['right','left']) {
            const sheath=anatomy.vessel.sheaths[side],world=new EndovascularPhysicsWorld();
            initializeElasticIntroducer(sheath);initializeIntroducerDilator(sheath);world.contactField=anatomy.field;
            const wire=world.createRod('wire',201,5,{radius:.4445,mass:1}),catheter=world.createRod('catheter',201,5,{radius:5/6,mass:1.4}),
                sleeve=world.createRod('sheath',191,5,{radius:1.1,mass:1.6}),core=world.createRod('dilator',201,5,{radius:.96,mass:1.4});
            let wireInsertion=0;
            const readTools=()=>[
                {id:'wire',type:'glidewire',body:wire,insertion:wireInsertion,rotation:0,shaftStiffness:13.15,tipStiffness:19.4,
                    nodeCoordinates:Array.from({length:wire.count},(_,i)=>-40+i/(wire.count-1)*(wireInsertion+40))},
                {id:'catheter',type:'berenstein',body:catheter,insertion:0,rotation:0,shaftStiffness:40.65,tipStiffness:100,
                    nodeCoordinates:Array.from({length:catheter.count},(_,i)=>-40+i/(catheter.count-1)*40)},
                elasticIntroducerTool(sheath,sleeve),elasticDilatorTool(sheath,core)
            ];
            const app=createSharedAxisAppSystem({readTools,readSheath:()=>introducerAccessBoundary(sheath),workSliceMs:Infinity,
                coupledFrictionNewton:true,reuseFrictionAssembly:true,contactCandidateMargin:.01,lazyBasisCoefficients:true,wasmBasis:true,deferActiveBasis:true,
                physicsOptions:{...SHARED_AXIS_APP_PHYSICS_OPTIONS,vesselElasticity:DEFAULT_VESSEL_ELASTICITY}});
            const step=()=>{
                const r=app.step(world,1/60);assert.ok(r.accepted,JSON.stringify({side,wireInsertion,dilator:sheath.dilatorInsertion,result:r}));
                const q=app.diagnostics.last.quality;assert.ok(q.finite&&q.maxPenetration<1e-5);
                assert.ok(app.diagnostics.last.certificateBound<=1e-6);
                publishElasticIntroducer(sheath,sleeve);publishElasticDilator(sheath,core);
            };
            step();for(wireInsertion=2;wireInsertion<=120;wireInsertion+=2)step();wireInsertion=120;
            while(!sheath.dilatorSeated){prepareDilatorMotion(sheath,2,{wireProgress:wireInsertion});step();}
            assert.ok(Math.abs(sheath.dilatorInsertion-95)<1e-9);
            assert.ok(Math.abs(core.jointStateView.coordinates.at(-1)-95)<1e-9);
            while(sheath.dilatorInsertion>0){prepareDilatorMotion(sheath,-2);step();}
            assert.equal(core.jointStateView.coordinates.at(-1),0);
            assert.ok(Math.abs(sheath.insertion-70)<1e-9);assert.equal(wireInsertion,120);
        }
    }finally{anatomy.dispose();}
});
