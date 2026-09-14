import { writeFileSync, mkdirSync } from 'node:fs';
import { loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { relaxSharedAxisWithContacts } from '../../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import { createSharedAxisContacts } from '../../src/physics/kirchhoffSharedAxisContacts.js';
import { createSharedAxisNative, feedSharedAxisNative, relaxSharedAxisNative, assembleSharedAxisNative } from '../../src/physics/kirchhoffSharedAxisNative.js';
import { INTRODUCER_SHEATH_INNER_RADIUS_MM } from '../../src/toolDimensions.js';
import { captureSharedAxisReplay } from '../../tests/helpers/sharedAxisReplay.js';
const anatomy=await loadCoupledRuntimeAnatomy(),samples=[];
const outputDirectory=process.argv[2]??'reports/shared-axis-native-2026-09-12';
mkdirSync(outputDirectory,{recursive:true});
const sheath={...anatomy.vessel.sheath,innerRadius:INTRODUCER_SHEATH_INNER_RADIUS_MM,proximalExtension:40};
let s=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:anatomy.field}),tools:[
    {id:'wire',insertion:0,shaftStiffness:39,tipStiffness:30.7},{id:'catheter',insertion:0,shaftStiffness:58.1,tipStiffness:87}]});
let failed=false;
const record=(phase,depth,candidate)=>{
    const incoming=captureSharedAxisReplay(candidate,sheath),trace=[],trials=[];
    let terminal;
    if(depth>=110)candidate.linearTrace=[];
    const measure=r=>({force:r.force,torque:r.torque,constraint:r.constraint,energy:r.energy});
    const result=relaxSharedAxisWithContacts(candidate,{
        observeIteration:({state,iteration,base})=>{if(depth>=104)trace.push({iteration,...measure(base),contacts:base.rows.filter(r=>r.kind==='wall'&&(r.multiplier>0||r.gap<.001)).map(r=>({id:r.id,edge:r.edge,sample:r.sample,gap:r.gap,lambda:r.multiplier,jacobian:r.jacobian}))});},
        observeTrial:event=>{
            if(depth<145)return;
            const {kind,state,iteration,method}=event;
            if(kind==='direction') {
                terminal=captureSharedAxisReplay(state,sheath);
                trials.push({kind,iteration,method,base:measure(event.base),linearConverged:event.direction.converged,
                    linearResidual:event.direction.residual,factorizations:event.direction.factorizations});
            } else if(kind==='trial') {
                const {trial,scale,basePotential,candidatePotential,predictedSlope,accept}=event;
                trials.push({kind,iteration,method,trial,scale,basePotential,candidatePotential,predictedSlope,accept,candidate:measure(event.candidate)});
            } else trials.push({kind,iteration,method,predictedSlope:event.predictedSlope});
        }
    });
    const rows=result.converged?assembleSharedAxisNative(candidate).rows:[];
    samples.push({phase,depth,...result,minimumGap:rows.length?Math.min(...rows.filter(r=>r.kind==='wall').map(r=>r.gap)):null});
    if(!result.converged){
        for(const [name,data] of Object.entries({incoming,terminal,contacts:trace,linear:candidate.linearTrace,trials}))
            if(data)writeFileSync(`${outputDirectory}/${name}.json`,JSON.stringify(data,null,2)+'\n');
        failed=true;console.log(JSON.stringify(samples.at(-1)));return false;
    }
    s=candidate;if(depth%25===0)console.log(phase,depth,result.ms.toFixed(2));return true;
};
try {
    if(record('initial',0,s))for(const [tool,target] of [['wire',309],['catheter',100]]){
        for(let depth=.25;depth<=target;depth+=.25)if(!record(tool,depth,feedSharedAxisNative(s,{[tool]:depth})))break;
        if(failed)break;
    }
}finally {
    writeFileSync(`${outputDirectory}/anatomy.json`,JSON.stringify({scope:'quasi-static native collision geometry; not dynamic application parity',failed,samples,positions:s.positions,coordinates:s.coordinates},null,2)+'\n');
    anatomy.dispose();
}
process.exitCode=failed?1:0;
