import test from 'node:test';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {readBalloonRuntimeReplay,runBalloonRuntimeReplay,balloonReplayView,referenceBalloonProjection} from './helpers/balloonRuntimeReplay.js';
import {sampleBalloonOnCatheter} from '../src/devices/compliantBalloon.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {VesselDeformation} from '../src/vesselDeformation.js';

test('runtime caches preserve exact accepted rods, rates, reactions and wall deformation through inflation/deflation',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const fixture=readBalloonRuntimeReplay();
        const reference=runBalloonRuntimeReplay(fixture,anatomy.field,{optimized:false});
        const optimized=runBalloonRuntimeReplay(fixture,anatomy.field,{optimized:true});
        assert.deepEqual(optimized.poses,reference.poses);
        for(let i=0;i<optimized.steps.length;i++) {
            const {wallMs,...step}=optimized.steps[i],{wallMs:unused,...expected}=reference.steps[i];
            assert.deepEqual(step,expected);
            assert.ok(step.certificateBound<=1e-6&&step.quality.maxPenetration<=1e-5);
        }
    }finally{anatomy.dispose();}
});

test('bounded balloon projection preserves every surface vertex and volume on actual anatomy with wall deformation',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline'),deformation=new VesselDeformation();
    try {
        const state=restoreSharedAxisReplay(readBalloonRuntimeReplay(),anatomy.field),view=balloonReplayView(state);
        const catheter={physicsBody:{jointStateView:view}};
        for(const deformed of [false,true]) {
            if(deformed)deformation.update([{center:[...view.positions.slice(30,33)],normal:[1,0,0],width:10,displacement:.2}]);
            for(const volume of [.1,1,4,12.5,41,60]) {
                const expected=referenceBalloonProjection(view,anatomy.geometry.boundsTree,volume,deformation);
                const actual=sampleBalloonOnCatheter(catheter,anatomy.geometry.boundsTree,volume,deformation);
                assert.deepEqual(actual,expected,`entire membrane at ${volume} ml, deformed=${deformed}`);
            }
        }
    }finally{deformation.dispose();anatomy.dispose();}
});
