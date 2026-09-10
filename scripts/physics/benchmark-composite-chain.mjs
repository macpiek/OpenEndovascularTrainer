import fs from 'node:fs';
import {createHash} from 'node:crypto';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain,
    solveCompositeChainIncrement} from '../../src/physics/kirchhoffCompositeChain.js';

const root=new URL('../../',import.meta.url),output=process.argv[2]??'/tmp/oet-composite-chain-benchmark.json';
const files=['src/physics/kirchhoffCompositeElement.js','src/physics/kirchhoffCompositeChain.js',
    'src/physics/kirchhoffLinearKernel.js','src/physics/kirchhoffLinearKernelBytes.js','scripts/physics/benchmark-composite-chain.mjs'];
const hashes=()=>Object.fromEntries(files.map(path=>[path,createHash('sha256').update(fs.readFileSync(new URL(path,root))).digest('hex')]));
const report={scope:'Common-axis constitutive assembly plus one primal direction only; NOT a full physics step, contact solve, admission certificate, anatomy run or FPS measurement.',
    sourceBefore:hashes(),node:process.version,measurements:[]};
for(const count of [32,65,128,201]) {
    const coordinates=Array.from({length:count},(_,i)=>i*3),positions=coordinates.map(x=>[x,.2*Math.sin(x/40),.1*Math.cos(x/37)]);
    const layout=createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter'])),workspace=createCompositeChainWorkspace(layout);
    const data={positions,coordinates,reference:captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:Float64Array.from(coordinates.slice(1),x=>.1*Math.sin(x/50)),material:compileCompositeMaterial({EI1:2,EI2:3,GJ:1.5,kappa0:[.001,0]})},
        {id:'catheter',angles:Float64Array.from(coordinates.slice(1),x=>.3+.02*x),material:compileCompositeMaterial({EI1:7,EI2:5,GJ:3,kappa0:[0,.003]})}
    ]};
    const diagonal=new Float64Array(layout.dofCount).fill(3),times=[];
    let last;
    for(let iteration=0;iteration<20;iteration++) {
        const began=performance.now();
        assembleCompositeChain(data,workspace);
        last=solveCompositeChainIncrement(workspace,{diagonal,tolerance:1e-10});
        const elapsed=performance.now()-began;
        if(!last.converged) throw new Error('The primal direction did not satisfy its original equations');
        if(iteration>=10)times.push(elapsed);
    }
    const sorted=times.toSorted((a,b)=>a-b);
    report.measurements.push({nodes:count,dofCount:layout.dofCount,independentPositionFrameDofs:2*(6*count-3),
        band:layout.band,matrixEntries:layout.dofCount*layout.band,factorizations:last.factorizations,
        maximumResidual:last.maximumResidual,meanMs:times.reduce((a,b)=>a+b)/times.length,medianMs:(sorted[4]+sorted[5])/2,
        maximumMs:sorted.at(-1),samplesMs:times});
}
report.sourceAfter=hashes();report.sourceStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,sourceStable:report.sourceStable,measurements:report.measurements.map(({samplesMs,...v})=>v)},null,2));
