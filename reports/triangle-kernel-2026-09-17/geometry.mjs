import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {createKirchhoffWallWitnessGeometryWorkspace as workspace,evaluateKirchhoffWallWitnessGeometry as evaluate} from '../../src/physics/kirchhoffWallWitnessGeometry.js';

const fixture=JSON.parse(gunzipSync(readFileSync(new URL('../lightweight-friction-2026-09-17/long/terminal.json.gz',import.meta.url))));
const anatomy=await loadCoupledRuntimeAnatomy();
try {
    const geometry=anatomy.field.fallbackGeometry;
    const queries=fixture.definitions.filter(d=>d.kind==='wall'&&d.witness).map(d=>{
        const {face,t}=d.witness,e=d.edge;
        const point=fixture.positions[e].map((v,i)=>(1-t)*v+t*fixture.positions[e+1][i]+(fixture.origin?.[i]??0));
        return {geometry,faceIndex:face,point,reuseTriangle:true,out:workspace(),alternate:point.map((v,i)=>v+(i===0?1e-4:0))};
    });
    assert.ok(queries.length>0);
    const values=g=>[g.distance,g.feature,g.featureMask,g.normalDefined,g.triangleKey,...g.closestPoint,...g.direction,...g.barycentric];
    for(const q of queries)for(const point of [q.point,q.alternate])
        assert.deepEqual(values(evaluate({...q,point,reuseTriangleKernel:true},q.out)),values(evaluate({...q,point})));
    const repetitions=30;
    function run(reuseTriangleKernel) {
        const start=performance.now();let checksum=0;
        for(let i=0;i<repetitions;i++)for(const q of queries) {
            const original=q.point;if(i%2)q.point=q.alternate;
            q.reuseTriangleKernel=reuseTriangleKernel;checksum+=evaluate(q,q.out).distance;q.point=original;
        }
        return {ms:performance.now()-start,checksum};
    }
    for(let i=0;i<3;i++){run(false);run(true);}
    const pairs=[];
    for(let i=0;i<10;i++) {
        const pair={};for(const mode of i%2?[true,false]:[false,true])pair[mode?'optimized':'reference']=run(mode);
        assert.equal(pair.reference.checksum,pair.optimized.checksum);pairs.push(pair);
    }
    const mean=key=>pairs.reduce((n,p)=>n+p[key].ms,0)/pairs.length;
    console.log(JSON.stringify({scope:'Retained finite triangle geometry on saved anatomical contacts, alternating nearby points; excludes discovery, global assembly, solving and rendering',
        queries:queries.length,uniqueFaces:new Set(queries.map(q=>q.faceIndex)).size,repetitions,node:process.version,
        sourceHashes:Object.fromEntries(['kirchhoffWallWitnessGeometry.js','kirchhoffWallTriangleKernel.js'].map(f=>[f,createHash('sha256').update(readFileSync(new URL('../../src/physics/'+f,import.meta.url))).digest('hex')])),
        referenceMs:mean('reference'),optimizedMs:mean('optimized'),reductionPercent:100*(1-mean('optimized')/mean('reference')),pairs},null,2));
} finally {anatomy.dispose();}
