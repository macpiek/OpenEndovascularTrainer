import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createSharedAxisNative} from '../../src/physics/kirchhoffSharedAxisNative.js';
import {evaluateBendTwistLocalConstraintNormalized as evaluate} from '../../src/physics/discreteKirchhoffRod.js';

// Microbenchmark only: the current production caller passes {} every time.
// This does not measure complete material assembly or a physical timestep.
const fixture=JSON.parse(gunzipSync(readFileSync(new URL('../inactive-contacts-2026-09-17/final-full/terminal.json.gz',import.meta.url))));
const s=createSharedAxisNative({tools:fixture.tools,spacing:fixture.spacing,startCoordinate:fixture.coordinates[0],
    spatialKnots:fixture.coordinates,adaptiveMesh:fixture.adaptiveMesh,minimumEdgeLength:fixture.minimumEdgeLength});
assert.deepEqual(s.coordinates,fixture.coordinates);
const inputs=[];
for(let tool=0;tool<s.materials.length;tool++) {
    const {body,last}=s.materials[tool],frames=fixture.frames[tool];
    const q=e=>Object.fromEntries(['x','y','z','w'].map((k,i)=>[k,frames[i][e]]));
    for(let e=1;e<last;e++)inputs.push({q0:q(e-1),q1:q(e),alternate:q(e===last-1?e-1:e+1),rest:{x:body.restRotation1[e],y:body.restRotation2[e],z:body.restRotation3[e]},out:{}});
}
const values=o=>({strain:{...o.strain},relative:{...o.relative},gradient:Array.from(o.localGradient)});
for(const input of inputs) {
    // Include invalidation by a changed rest shape, then restore the original.
    for(const rest of [input.rest,{...input.rest,x:input.rest.x+.03},input.rest])for(const q1 of [input.q1,input.alternate,input.q1])
        assert.deepEqual(values(evaluate(input.q0,q1,rest,input.out)),values(evaluate(input.q0,q1,rest,{})));
}
const repetitions=200;
function run(reuse) {
    let checksum=0;const start=performance.now();
    for(let repeat=0;repeat<repetitions;repeat++)for(const i of inputs) {
        const out=evaluate(i.q0,repeat%2?i.alternate:i.q1,i.rest,reuse?i.out:{});
        checksum+=out.strain.x+out.localGradient[0];
    }
    return {ms:performance.now()-start,checksum};
}
for(let i=0;i<3;i++){run(false);run(true);}
const pairs=[];
for(let i=0;i<8;i++) {
    const results={};for(const reuse of i%2?[true,false]:[false,true])results[reuse?'reused':'fresh']=run(reuse);
    assert.equal(results.reused.checksum,results.fresh.checksum);pairs.push(results);
}
const mean=key=>pairs.reduce((sum,p)=>sum+p[key].ms,0)/pairs.length;
console.log(JSON.stringify({scope:'Repeated local material evaluator on terminal rotations; excludes assembly, promotion, nonlinear solve and rendering',
    sourceHashes:Object.fromEntries(['src/physics/discreteKirchhoffRod.js','src/physics/kirchhoffSharedAxisMaterialTangent.js'].map(path=>[path,createHash('sha256').update(readFileSync(new URL('../../'+path,import.meta.url))).digest('hex')])),
    node:process.version,changingOrientations:true,hinges:inputs.length,evaluationsPerBatch:inputs.length*repetitions,restChangeParity:true,
    freshMeanMs:mean('fresh'),reusedMeanMs:mean('reused'),reductionPercent:100*(1-mean('reused')/mean('fresh')),pairs},null,2));
