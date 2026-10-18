import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createCrossoverFlow} from './helpers/crossoverFlow.js';
import {createCrossoverRouteCheck,summarizeCrossoverRoute} from '../src/benchmark/crossoverFunctionalRoute.js';

const flow=createCrossoverFlow(),check=createCrossoverRouteCheck(flow);
const recording=JSON.parse(gunzipSync(readFileSync(new URL('../reports/performance-scenarios/003-crossover-full-user-inputs.json.gz',import.meta.url))));
const stages=recording.checkpoints.map(c=>recording.steps[c.afterStep-1]);

test('original human maneuver reaches the required coarse branches at all five checkpoints',()=>{
    const checkpoints=stages.map((s,i)=>check.checkpoint(i,s.shape,s.tools));
    assert.deepEqual(checkpoints.map(c=>c.passed),[true,true,true,true,true]);
    assert.equal(checkpoints[0].catheter.region,'aorta');
    assert.equal(checkpoints[1].catheter.region,'contralateral-common');
    assert.equal(checkpoints[2].wire.region,'contralateral-internal');
    assert.equal(checkpoints[3].wire.region,'contralateral-external');
    const summary=summarizeCrossoverRoute(checkpoints,{complete:true});
    assert.equal(summary.routeChecksPassed,true);
    assert.equal(summary.fullManeuverVerified,false); // coarse routing is not clinical/visual validation
});

test('shape deviations larger than 1 mm within the same aortic region are allowed',()=>{
    const s=stages[0],altered={...s.shape,positions:s.shape.positions.map((x,i)=>i%3===0?x+2:x)};
    assert.equal(check.checkpoint(0,altered,s.tools).passed,true);
});

test('ipsilateral routing and internal iliac entry cannot pass the external iliac stage',()=>{
    const s=stages[1];
    assert.equal(check.checkpoint(1,s.shape,{...s.tools,catheter:s.tools.wire}).passed,false);
    assert.equal(check.checkpoint(3,stages[2].shape,stages[2].tools).passed,false);
});

test('missing or distant tips remain unknown and incomplete/repeated checkpoints do not pass',()=>{
    const s=stages[4],distant={...s.shape,positions:s.shape.positions.map(x=>x+10000)};
    for(const shape of [null,distant])assert.equal(check.checkpoint(4,shape,s.tools).passed,false);
    assert.equal(check.locate(s.shape,NaN).region,'unknown');
    assert.equal(check.locate(s.shape,s.shape.coordinates.at(-1)+10).region,'unknown');
    const checkpoints=stages.map((s,i)=>check.checkpoint(i,s.shape,s.tools));
    assert.equal(summarizeCrossoverRoute(checkpoints).routeChecksPassed,false);
    assert.equal(summarizeCrossoverRoute(checkpoints.slice(1),{complete:true}).routeChecksPassed,false);
    assert.equal(summarizeCrossoverRoute(Array(5).fill(checkpoints[4]),{complete:true}).routeChecksPassed,false);
});
