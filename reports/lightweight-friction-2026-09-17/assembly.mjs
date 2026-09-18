import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {prepareSharedAxisDynamicStep} from '../../src/physics/kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction,assembleSharedAxisWallFriction} from '../../src/physics/kirchhoffSharedAxisWallFriction.js';

const anatomy=await loadCoupledRuntimeAnatomy();
try {
    const results=[];
    for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming']) {
        const fixture=JSON.parse(readFileSync(new URL('../../tests/fixtures/shared-axis/'+name+'.json',import.meta.url)));
        const s=restoreSharedAxisReplay(fixture,anatomy.field);
        prepareSharedAxisDynamicStep(s,fixture.stepRequest.dt);
        prepareSharedAxisWallFriction(s,{feedById:{wire:.01,catheter:.005},liveNormalLoad:true});
        s.chain.tangent=new Float64Array(s.layout.dofCount*(2*s.layout.band-1));
        assert.ok(s.wallFrictionStep.records.length>0);
        const repetitions=500;
        for(const full of [false,true]) {
            const once=lightweight=>{
                s.chain.gradient.fill(0);s.chain.tangent.fill(0);
                const energy=assembleSharedAxisWallFriction(s,full,lightweight);
                return {energy,g:s.chain.gradient.slice(),H:s.chain.tangent.slice(),columns:structuredClone(s.wallFrictionStep.normalForceColumns)};
            };
            assert.deepEqual(once(true),once(false));
            const run=lightweight=>{
                const start=performance.now();let checksum=0;
                for(let i=0;i<repetitions;i++) {
                    s.chain.gradient.fill(0);s.chain.tangent.fill(0);
                    checksum+=assembleSharedAxisWallFriction(s,full,lightweight);
                }
                return {ms:performance.now()-start,checksum};
            };
            for(let i=0;i<3;i++){run(false);run(true);}
            const pairs=[];
            for(let i=0;i<10;i++) {
                const pair={};for(const light of i%2?[true,false]:[false,true])pair[light?'optimized':'reference']=run(light);
                assert.equal(pair.optimized.checksum,pair.reference.checksum);pairs.push(pair);
            }
            const mean=key=>pairs.reduce((sum,p)=>sum+p[key].ms,0)/pairs.length;
            results.push({name,full,contacts:s.wallFrictionStep.records.length,repetitions,referenceMs:mean('reference'),optimizedMs:mean('optimized'),
                reductionPercent:100*(1-mean('optimized')/mean('reference')),pairs});
        }
    }
    console.log(JSON.stringify({scope:'Repeated friction assembly on two saved anatomical poses with prepared slip; includes output zeroing, excludes remaining physics and rendering',
        node:process.version,sourceHash:createHash('sha256').update(readFileSync(new URL('../../src/physics/kirchhoffSharedAxisWallFriction.js',import.meta.url))).digest('hex'),results},null,2));
} finally {anatomy.dispose();}
