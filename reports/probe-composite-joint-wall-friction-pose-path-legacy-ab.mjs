import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
const [runtime,baseline]=process.argv.slice(2);if(!runtime||!baseline)throw Error('Usage: node probe.mjs ISOLATED_RUNTIME BASELINE_MANAGER');
const source='src/physics/kirchhoffCompositeJointWallFrictionRows.js',test='tests/kirchhoffCompositeJointWallFrictionRows.test.js';
const fixtureFile=path.join(runtime,'tests/wall-friction-pose-ab-fixture.mjs'),oldFile=path.join(runtime,'src/physics/wall-friction-pose-ab-baseline.js'),contents=fs.readFileSync(path.join(runtime,test),'utf8');
const nodal=contents.slice(contents.indexOf('function nodalFixture(')).split("test('explicit nodal pressure")[0];
fs.writeFileSync(fixtureFile,contents.split("test('original wall records")[0]+nodal+'\nexport {fixture,nodalFixture,normalRefresh};\n');fs.copyFileSync(baseline,oldFile);
const helpers=await import(pathToFileURL(fixtureFile)),old=await import(pathToFileURL(oldFile)),now=await import(pathToFileURL(path.join(runtime,source))),cases=[];
try{
    const inputs=[];
    for(const single of [false,true])for(const two of [false,true])for(const rate of [0,.2])for(const penalty of [5,50,500])inputs.push({description:{mode:'capsule',single,two,rate,penalty},f:helpers.fixture({single,two,rate,penalty})});
    for(const single of [false,true])for(const sharedTrace of ['left','right'])for(const penalty of [5,50,500]){const f=helpers.nodalFixture({single,sharedTrace});f.wall.friction.forcePerLength=penalty;inputs.push({description:{mode:'nodal-endpoints',single,sharedTrace,penalty},f});}
    for(const {description,f} of inputs){
        if(cases.length%2===0)f.wall.surfacePosePaths=[];
        const a=old.createCompositeJointWallFrictionRows(f),b=now.createCompositeJointWallFrictionRows(f);
        assert.equal(a.signature,b.signature);helpers.normalRefresh(f);a.prepare({toolPositions:f.candidate.toolPositions});b.prepare({toolPositions:f.candidate.toolPositions});
        f.candidate.toolPositions.forEach(p=>p.forEach(v=>v[0]+=.02));f.candidate.angles.forEach(v=>v[0]=.2);helpers.normalRefresh(f);
        const evaluate=(m,order)=>{const common=new Float64Array(f.state.layout.dofCount),relative=new Float64Array(f.candidate.relative.length),certificate=m.refresh({toolPositions:f.candidate.toolPositions,commonResidual:common,relativeResidual:relative,order});return structuredClone({certificate,common,relative,rows:m.rows,nodalForces:m.nodalForces,spinTorques:m.spinTorques});};
        assert.deepEqual(evaluate(a,'full'),evaluate(b,'full'));const first=evaluate(a,'full').certificate;evaluate(b,'full');
        first.samples.forEach((s,j)=>{const d=Math.hypot(...s.slip.map((v,k)=>v*s.mu[k]));for(let k=0;k<2;k++)a.tractions[2*j+k]=b.tractions[2*j+k]=d===0?0:-s.Fn*s.mu[k]**2*s.slip[k]/d;});
        for(const order of ['full','gradient','full']){const x=evaluate(a,order),y=evaluate(b,order);assert.deepEqual(y,x);assert.equal(y.certificate.converged,true);}
        assert.deepEqual(a.commit(),b.commit());assert.deepEqual(a.diagnostics,b.diagnostics);assert.deepEqual(a.workspace.diagnostics,b.workspace.diagnostics);cases.push({...description,exact:true});
    }
    console.log(JSON.stringify({pass:true,count:cases.length,baselineSHA256:createHash('sha256').update(fs.readFileSync(baseline)).digest('hex'),candidateSHA256:createHash('sha256').update(fs.readFileSync(path.join(runtime,source))).digest('hex'),cases},null,2));
}finally{fs.rmSync(fixtureFile);fs.rmSync(oldFile);}
