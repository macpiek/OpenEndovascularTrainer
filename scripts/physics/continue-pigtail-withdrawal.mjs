import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const fixture=JSON.parse(readFileSync(process.argv[2],'utf8')),output=process.argv[3],target=Number(process.argv[4]??190);
const anatomy=await loadCoupledRuntimeAnatomy(),req=fixture.stepRequest;
mkdirSync(output,{recursive:true});let s=restoreSharedAxisReplay(fixture,anatomy.field),rotations=req.rotations;const samples=[];
try {
    while(s.materials.find(m=>m.spec.id==='wire').spec.insertion>target) {
        const wire=Math.max(target,s.materials.find(m=>m.spec.id==='wire').spec.insertion-32*req.dt);
        const tools=req.tools.map(t=>t.id==='wire'?{...t,insertion:wire}:t),started=performance.now();
        const iterator=advanceSharedAxis(s,rotations,req.dt,tools,req.options);let next;do{next=iterator.next();}while(!next.done);
        samples.push({wire,totalMs:performance.now()-started,nodes:next.value.state?.positions.length,...next.value.result});
        if(!next.value.state){writeFileSync(`${output}/incoming.json`,JSON.stringify({...captureSharedAxisReplay(s,fixture.sheath),stepRequest:{...req,rotations,tools}}));process.exitCode=1;break;}
        s=next.value.state;rotations=next.value.rotations;
        if(samples.length%20===0)console.log(wire,s.positions.length,samples.at(-1).totalMs);
    }
}finally {
    writeFileSync(`${output}/profile.json`,JSON.stringify(samples,null,2));
    writeFileSync(`${output}/terminal.json`,JSON.stringify(captureSharedAxisReplay(s,fixture.sheath)));anatomy.dispose();
}
console.log(samples.at(-1));
