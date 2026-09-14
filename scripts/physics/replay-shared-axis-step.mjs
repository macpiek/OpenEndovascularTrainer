import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';

const fixture=JSON.parse(readFileSync(process.argv[2],'utf8'));
const output=process.argv[3];mkdirSync(output,{recursive:true});
const anatomy=await loadCoupledRuntimeAnatomy();
try {
    for(const earlyLiveFallback of [false,true]) {
        const s=restoreSharedAxisReplay(fixture,anatomy.field),req=fixture.stepRequest;
        const start=performance.now();
        const directions={};
        const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,earlyLiveFallback,
            observeTrial:e=>{if(e.kind==='direction'){
                const key=`${e.state.wallFrictionStep?.liveNormalLoad?'live':'frozen'}/${e.method}/${e.direction.failure??'success'}`;
                const d=directions[key]??={count:0,factorizations:0};d.count++;d.factorizations+=e.direction.factorizations;
            }}});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value,report={earlyLiveFallback,totalMs:performance.now()-start,result,directions};
        const label=earlyLiveFallback?'guard':'reference';
        writeFileSync(`${output}/${label}-result.json`,JSON.stringify(report,null,2));
        if(state)writeFileSync(`${output}/${label}-terminal.json`,JSON.stringify(captureSharedAxisReplay(state,fixture.sheath)));
        console.log(JSON.stringify(report));
        if(!state)process.exitCode=1;
    }
}finally{anatomy.dispose();}
