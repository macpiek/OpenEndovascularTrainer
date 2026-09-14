import {readFileSync,writeFileSync,mkdirSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';

const fixture=JSON.parse(readFileSync(process.argv[2],'utf8'));
const output=process.argv[3];mkdirSync(output,{recursive:true});
if(fixture.failure?.captureError)throw new Error(`Incomplete capture: ${fixture.failure.captureError}`);
const incremental=process.argv.includes('--incremental-contacts');
const compare=process.argv.includes('--compare-fallback');
const anatomy=await loadCoupledRuntimeAnatomy();
try {
    for(const earlyLiveFallback of (compare?[false,true]:[fixture.stepRequest.options?.earlyLiveFallback])) {
        const s=restoreSharedAxisReplay(fixture,anatomy.field),req=fixture.stepRequest;
        const start=performance.now();
        const directions={};
        const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,earlyLiveFallback,...(incremental?{incrementalContacts:true}:{}),
            observeTrial:e=>{if(e.kind==='direction'){
                const key=`${e.state.wallFrictionStep?.liveNormalLoad?'live':'frozen'}/${e.method}/${e.direction.failure??'success'}`;
                const d=directions[key]??={count:0,factorizations:0};d.count++;d.factorizations+=e.direction.factorizations;
            }}});
        let next;
        try {do{next=iterator.next();}while(!next.done);}
        catch(error) {next={value:{result:{status:'shared-axis-error',error:error.message,stack:error.stack}}};}
        const {state,result}=next.value,report={earlyLiveFallback,incrementalContacts:incremental||req.options?.incrementalContacts===true,totalMs:performance.now()-start,result,directions};
        const label=compare?(earlyLiveFallback?'guard':'reference'):'captured';
        writeFileSync(`${output}/${label}-result.json`,JSON.stringify(report,null,2));
        if(state)writeFileSync(`${output}/${label}-terminal.json`,JSON.stringify(captureSharedAxisReplay(state,fixture.sheath)));
        console.log(JSON.stringify(report));
        if(!state)process.exitCode=1;
    }
}finally{anatomy.dispose();}
