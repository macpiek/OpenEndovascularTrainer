import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {assembleSharedAxisMaterialTangent as assemble} from '../../src/physics/kirchhoffSharedAxisMaterialTangent.js';

const path=process.argv[2],raw=readFileSync(path),fixture=JSON.parse(path.endsWith('.gz')?gunzipSync(raw):raw);
const anatomy=await loadCoupledRuntimeAnatomy();
try {
    const s=restoreSharedAxisReplay(fixture,anatomy.field),repetitions=200,results={};
    for(const mode of ['residual-capture','full-wasm','promoted-wasm']) {
        const preparation=[];
        assemble(s,false,{capture:preparation},true,false);
        const run=reuse=>{
            const start=performance.now();let checksum=0;
            for(let i=0;i<repetitions;i++)checksum+=assemble(s,mode!=='residual-capture',
                mode==='residual-capture'?{capture:preparation}:mode==='promoted-wasm'?{reuse:preparation}:null,true,reuse);
            return {ms:performance.now()-start,checksum};
        };
        for(let i=0;i<3;i++){run(false);run(true);}
        const pairs=[];
        for(let i=0;i<10;i++) {
            const pair={};
            for(const reuse of i%2?[true,false]:[false,true])pair[reuse?'reused':'fresh']=run(reuse);
            assert.equal(pair.fresh.checksum,pair.reused.checksum);
            pairs.push(pair);
        }
        const mean=key=>pairs.reduce((n,p)=>n+p[key].ms,0)/pairs.length;
        results[mode]={freshMs:mean('fresh'),reusedMs:mean('reused'),reductionPercent:100*(1-mean('reused')/mean('fresh')),pairs};
    }
    console.log(JSON.stringify({scope:'Repeated complete material assembly at one saved pose; excludes contact, inertia, remeshing, solving and rendering',
        node:process.version,hinges:s.materials.reduce((n,m)=>n+Math.max(0,m.last-1),0),repetitions,
        sourceHash:createHash('sha256').update(readFileSync(new URL('../../src/physics/kirchhoffSharedAxisMaterialTangent.js',import.meta.url))).digest('hex'),results},null,2));
} finally {anatomy.dispose();}
