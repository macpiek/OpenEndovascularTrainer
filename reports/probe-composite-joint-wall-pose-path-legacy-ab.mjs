import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
// Operates only on an isolated runtime snapshot, never on the root sources.
const [runtime,baseline]=process.argv.slice(2);
if(!runtime||!baseline)throw Error('Usage: node probe.mjs ISOLATED_RUNTIME BASELINE_WALL_SURFACE');
const source='src/physics/kirchhoffCompositeJointWallSurface.js',test='tests/kirchhoffCompositeJointWallSurface.test.js';
const fixtureFile=path.join(runtime,'tests/wall-pose-ab-fixture.mjs'),oldFile=path.join(runtime,'src/physics/wall-pose-ab-baseline.js');
fs.writeFileSync(fixtureFile,fs.readFileSync(path.join(runtime,test),'utf8').split("for(const kind of ['plane','sdf','bvh'])test")[0]+'\nexport {fixture,refresh,createCompositeWallEnvelopeWorkspace,layout};\n');
fs.copyFileSync(baseline,oldFile);
const {fixture,refresh,createCompositeWallEnvelopeWorkspace,layout}=await import(pathToFileURL(fixtureFile));
const old=await import(pathToFileURL(oldFile)),now=await import(pathToFileURL(path.join(runtime,source))),cases=[];
try{
    for(const kind of ['plane','sdf','bvh'])for(const basis of ['projected-own-tangent','projected-own-reference-director'])for(const envelope of [false,true])for(const spin of [0,.19,2*Math.PI]){
        const input=fixture(kind);input.wall.tangentBasis=basis;input.tool.angle=input.tool.previousAngle+spin;
        if(envelope){input.current.envelope=true;input.current.collector=createCompositeWallEnvelopeWorkspace(layout);refresh(input);}
        const oldWs=old.createCompositeJointWallSurfaceWorkspace(),newWs=now.createCompositeJointWallSurfaceWorkspace();
        assert.deepEqual(newWs,oldWs);
        for(const order of ['full','value']){
            const a=structuredClone(old.evaluateCompositeJointWallSurface({...input,order},oldWs)),b=now.evaluateCompositeJointWallSurface({...input,order},newWs);
            assert.deepEqual(b,a);
            cases.push({kind,basis,envelope,spin,order,exact:true,sha256:createHash('sha256').update(JSON.stringify(a)).digest('hex')});
        }
        input.current.field.fallbackGeometry?.dispose();
    }
    console.log(JSON.stringify({pass:true,cases:cases.length,baselineSha256:createHash('sha256').update(fs.readFileSync(baseline)).digest('hex'),candidateSha256:createHash('sha256').update(fs.readFileSync(path.join(runtime,source))).digest('hex'),results:cases},null,2));
}finally{fs.rmSync(fixtureFile);fs.rmSync(oldFile);}
