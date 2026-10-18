import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';
import {compareCrossoverAlternative,summarizeAlternative} from '../src/benchmark/crossoverAlternativeSolvers.js';

const input='reports/performance-scenarios/003-crossover-full-user-inputs.json.gz';
const solver=process.argv[2]??'xpbd',options=JSON.parse(process.argv[3]??'{}');
const output=process.argv[4]??`reports/performance-scenarios/012-${solver}-screen.json`;
const screening=JSON.parse(process.argv[5]??'{}');
if(screening.maxSteps!==undefined&&(!Number.isInteger(screening.maxSteps)||screening.maxSteps<1))throw new RangeError('Invalid diagnostic prefix length');
const bytes=fs.readFileSync(input),record=JSON.parse(gunzipSync(bytes));
const anatomy=await loadCoupledRuntimeAnatomy(undefined,record.configuration.anatomy),rows=[];
const report={solver,options,screening,input,sourceSha256:createHash('sha256').update(bytes).digest('hex'),startedAt:new Date().toISOString(),
    sourceHashes:Object.fromEntries(['src/physics/sharedAxisXpbdExperimental.js','src/physics/xpbdChainBlock.js','src/physics/xpbdChainIteration.js','src/physics/xpbdSurfaceFriction.js','src/physics/kirchhoffSharedAxisProjective.js',
        'src/benchmark/crossoverAlternativeSolvers.js'].map(p=>[p,createHash('sha256').update(fs.readFileSync(p)).digest('hex')])),
    scope:'CPU screening of original recorded targets from empty tools, right access only; not full-app FPS. Stops on geometry rejection.'+
        (screening.stopOnShapeLimit?' Also rejects at the first shape-budget violation.':' Continues through shape-budget failures for diagnosis.')+
        (screening.maxSteps?` Diagnostic prefix limited to ${screening.maxSteps} steps; never a full-maneuver pass.`:''),
    defaultSolverChanged:false,runtimeVersions:process.versions,crossRuntimeReferenceCalibrated:false,
    shapeFailureAttribution:'Node/browser arithmetic differs (017-runtime-audit-validation.json). Shape differences include runtime effects; use a same-runtime control before attributing a failure to the solver. Full browser replay remains authoritative.',rows};
try {
    for(const row of compareCrossoverAlternative({record,field:anatomy.field,flow:createCrossoverFlow(),solver,options})) {
        rows.push(row);
        if(row.step%300===0||!row.accepted)console.log(JSON.stringify({step:row.step,status:row.status,cpuMs:row.cpuMs,
            differenceMm:row.shapeDifferenceMm,attempts:!row.accepted?row.attempts:undefined}));
        if(screening.stopOnShapeLimit&&row.accepted&&!row.shapeWithinLimit){report.stopReason='shape-budget-failure';break;}
        if(screening.maxSteps&&rows.length>=screening.maxSteps){report.stopReason='diagnostic-prefix-limit';break;}
    }
}catch(error){report.error=error.stack;process.exitCode=1;}
finally {
    report.summary=summarizeAlternative(rows,record.steps.length);report.finishedAt=new Date().toISOString();
    const serialized=JSON.stringify(report);fs.writeFileSync(output,output.endsWith('.gz')?gzipSync(serialized):serialized+'\n');anatomy.dispose();console.log(JSON.stringify(report.summary));
}
