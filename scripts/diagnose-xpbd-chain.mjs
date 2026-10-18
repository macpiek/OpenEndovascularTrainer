// Constitutive calibration only, not a crossover throughput benchmark.
import fs from 'node:fs';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {iterateSharedAxisXpbd} from '../src/physics/sharedAxisXpbdExperimental.js';
const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};
const cases=[];
for(const type of ['straight','berenstein'])for(const chainPrimalBalance of [true,false]) {
    const create=()=>createSharedAxisNative({tools:[{id:'catheter',type,insertion:60,shaftStiffness:2,tipStiffness:2}]});
    const reference=create(),candidate=create(),item={type,chainPrimalBalance,steps:[],complete:false};cases.push(item);
    reference.loads[reference.layout.positions.at(-1)+1]=candidate.loads[candidate.layout.positions.at(-1)+1]=1;
    for(let step=0;step<30;step++) {
        const before=performance.now(),a=finish(iterateSharedAxisTimeStep(reference,1/60)),referenceMs=performance.now()-before;
        const start=performance.now(),b=finish(iterateSharedAxisXpbd(candidate,1/60,{xpbd:{iterations:32,globalChain:true,chainPrimalBalance}})),candidateMs=performance.now()-start;
        const difference=Math.max(...reference.positions.map((p,i)=>Math.hypot(...p.map((v,k)=>v-candidate.positions[i][k]))));
        item.steps.push({step:step+1,referenceMs,candidateMs,referenceConverged:a.converged,candidateConverged:b.converged,
            candidateStatus:b.status,candidateError:b.error,referenceStatus:a.status,differenceMm:difference,
            candidateQuality:b.xpbd,referenceTip:reference.positions.at(-1).slice(),candidateTip:candidate.positions.at(-1).slice()});
        if(!a.converged||!b.converged)break;
    }
    item.complete=item.steps.length===30&&item.steps.at(-1).candidateConverged;
    console.log(JSON.stringify({type,chainPrimalBalance,complete:item.complete,last:item.steps.at(-1)}));
}
fs.writeFileSync('reports/performance-scenarios/013-constitutive-calibration.json',JSON.stringify({scope:'Free 60 mm rod under unit tip load for 30 steps. Mechanical calibration; not scenario performance.',cases},null,2)+'\n');
