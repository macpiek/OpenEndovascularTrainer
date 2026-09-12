import assert from 'node:assert/strict';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const root=process.argv[2],read=p=>import(pathToFileURL(`${root}/${p}`));
const {fixture,normalRefresh}=await read('tests/capsule-ab-fixture.mjs');
const before=await read('src/physics/kirchhoffCompositeJointWallFrictionRowsBaseline.js'),after=await read('src/physics/kirchhoffCompositeJointWallFrictionRows.js');
const results=[];
for(const single of [false,true])for(const two of [false,true])for(const rate of [0,.2])for(const penalty of [5,50,500]) {
    const f=fixture({single,two,rate,penalty}),a=before.createCompositeJointWallFrictionRows(f),b=after.createCompositeJointWallFrictionRows(f);
    normalRefresh(f);a.prepare({toolPositions:f.candidate.toolPositions});b.prepare({toolPositions:f.candidate.toolPositions});
    assert.equal(a.signature,b.signature);f.candidate.toolPositions.forEach(p=>p.forEach(v=>v[0]+=.02));f.candidate.angles.forEach(v=>v[0]=.2);normalRefresh(f);
    function evaluate(m,order){const common=new Float64Array(f.state.layout.dofCount),relative=new Float64Array(f.candidate.relative.length),certificate=m.refresh({toolPositions:f.candidate.toolPositions,commonResidual:common,relativeResidual:relative,order});
        return structuredClone({certificate,common,relative,rows:m.rows,nodalForces:m.nodalForces,spinTorques:m.spinTorques});}
    assert.deepEqual(evaluate(a,'full'),evaluate(b,'full'));
    const c=evaluate(a,'full').certificate;evaluate(b,'full');
    for(const s of c.samples){const d=Math.hypot(...s.slip.map((v,i)=>s.mu[i]*v)),index=c.samples.indexOf(s);for(let i=0;i<2;i++)a.tractions[2*index+i]=b.tractions[2*index+i]=d===0?0:-s.Fn*s.mu[i]**2*s.slip[i]/d;}
    for(const order of ['full','gradient','full']){const old=evaluate(a,order),current=evaluate(b,order);assert.deepEqual(current,old);assert.equal(current.certificate.converged,true);}
    assert.deepEqual(a.commit(),b.commit());assert.deepEqual(a.diagnostics,b.diagnostics);
    results.push({single,two,rate,penalty,signatureIdentical:true,fullAndValueBitIdentical:true,acceptedHistoryIdentical:true});
}
const evidence={status:'PASS',baselineSourceSha256:'6d4b779c39510a68273b0d83b181756030ff2f33486a4374f54df3ee5c07bbc9',cases:results};
fs.writeFileSync(process.argv[3],JSON.stringify(evidence,null,2)+'\n');console.log(JSON.stringify({status:evidence.status,cases:results.length}));
