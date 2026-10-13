// Deterministic CPU replay of long inserted tools. No rendering or FPS claims.
import fs from 'node:fs';
import {createHash} from 'node:crypto';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const fixtures=[
    'anatomy-berenstein-feed-492.27-stagnation.json',
    'anatomy-berenstein-feed-312.87-projection.json',
    'anatomy-pigtail-friction-settling-incoming.json'
];
const hash=value=>createHash('sha256').update(typeof value==='string'?value:JSON.stringify(value)).digest('hex');
const hold=process.argv.includes('--hold');
const report={hold,node:process.version,warmupRepeats:3,measuredRepeats:7,stepsPerRepeat:8,
    protocol:'Restore each fixture; repeat the recorded insertion delta each step (or hold the first target with --hold). Exclude anatomy loading, restoring and hashing from CPU step timings. Warm up separately for each fixture. Compare full physical replay hashes and per-step convergence, quality and residuals.',
    cases:[]};
const anatomy=await loadCoupledRuntimeAnatomy();
let failed=false;
try {
    for(const name of fixtures) {
        const source=fs.readFileSync(new URL('../tests/fixtures/shared-axis/'+name,import.meta.url),'utf8');
        const saved=JSON.parse(source),req=saved.stepRequest;
        const options={...req.options,velocityPredictor:1,zeroDualStart:true,coupledFrictionNewton:true,
            wasmMaterial:true,wasmLinearAssembly:true,reuseMaterialScratch:true,promoteTrialAssembly:true,reuseConstraintWork:true,
            reuseMatrixAssembly:true,reuseRowBuffers:true,lightweightFriction:true,reuseTriangleKernel:true,
            earlyContactPreflight:true,projectionMode:'reduced'};
        for(let repeat=0;repeat<report.warmupRepeats+report.measuredRepeats;repeat++) {
            let state=restoreSharedAxisReplay(saved,anatomy.field),rotations=req.rotations;
            const record={name,fixtureHash:hash(source),repeat,warmup:repeat<report.warmupRepeats,
                tools:saved.tools.map(t=>({id:t.id,insertion:t.insertion})),nodes:state.coordinates.length,
                dt:req.dt,options,steps:[],physicalHash:null};
            for(let step=0;step<report.stepsPerRepeat;step++) {
                const targets=hold?req.tools:req.tools.map(t=>({...t,insertion:t.insertion+step*(t.insertion-saved.tools.find(old=>old.id===t.id).insertion)}));
                const start=performance.now(),iterator=advanceSharedAxis(state,rotations,req.dt,targets,options);
                let next;do{next=iterator.next();}while(!next.done);
                const ms=performance.now()-start,{result}=next.value;
                record.steps.push({ms,...Object.fromEntries(['converged','status','iterations','factorizations',
                    'fullAssemblies','residualAssemblies','subdivisions','quality','residual','certificateBound','timings'].map(k=>[k,result[k]]))});
                if(!next.value.state||!result.converged||result.quality?.finite===false){failed=true;break;}
                state=next.value.state;rotations=next.value.rotations;
            }
            record.physicalHash=hash(captureSharedAxisReplay(state,saved.sheath));
            report.cases.push(record);
            console.log(name,repeat,record.steps.map(r=>`${r.ms.toFixed(1)}ms/${r.iterations}/${r.status}`).join(' '));
        }
    }
    report.summary=fixtures.map(name=>{
        const cases=report.cases.filter(c=>c.name===name&&!c.warmup);
        const samples=cases.flatMap(c=>c.steps.map(s=>s.ms)).sort((a,b)=>a-b);
        const hashes=new Set(cases.map(c=>c.physicalHash));
        if(hashes.size!==1)failed=true;
        return {name,steps:samples.length,meanMs:samples.reduce((a,b)=>a+b,0)/samples.length,
            medianMs:(samples[(samples.length-1)>>1]+samples[samples.length>>1])/2,
            p95Ms:samples[Math.ceil(samples.length*.95)-1],deterministic:hashes.size===1,physicalHash:cases[0].physicalHash};
    });
    report.passed=!failed;
} finally {
    anatomy.dispose();
    fs.writeFileSync(process.argv[2]??'/tmp/oet-long-tool-physics.json',JSON.stringify(report,null,2)+'\n');
}
if(failed)process.exitCode=1;
