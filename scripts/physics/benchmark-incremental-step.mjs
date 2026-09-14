import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const fixture=JSON.parse(readFileSync(process.argv[2],'utf8')),out=process.argv[3];mkdirSync(out,{recursive:true});
const anatomy=await loadCoupledRuntimeAnatomy(),samples=[],terminals={};
try {
    for(let repeat=0;repeat<6;repeat++)for(const incrementalContacts of repeat%2?[true,false]:[false,true]) {
        const state=restoreSharedAxisReplay(fixture,anatomy.field),req=fixture.stepRequest,start=performance.now();
        const mode=incrementalContacts?(process.env.OET_INCREMENTAL_MODE??true):false;
        const iterator=advanceSharedAxis(state,req.rotations,req.dt,req.tools,{...req.options,incrementalContacts:mode});
        let next;do{next=iterator.next();}while(!next.done);
        const {state:terminal,result}=next.value;
        samples.push({repeat,incrementalContacts,mode,totalMs:performance.now()-start,result});
        if(terminal)terminals[incrementalContacts?'incremental':'reference']=captureSharedAxisReplay(terminal,fixture.sheath);
        console.log(JSON.stringify(samples.at(-1)));
    }
    writeFileSync(`${out}/comparison.json`,JSON.stringify({scope:'Complete captured prepared physical step, repeated in alternating order; repeat 0 is warmup; no rendering/FPS claim.',fixture:process.argv[2],samples},null,2));
    for(const [k,v] of Object.entries(terminals))writeFileSync(`${out}/${k}-terminal.json`,JSON.stringify(v));
}finally{anatomy.dispose();}
