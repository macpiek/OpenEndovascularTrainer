import fs from 'node:fs';
import {createHash} from 'node:crypto';
import {buildKirchhoffCompositeTopology,compositeToolFromTipProfile} from '../../src/physics/kirchhoffCompositeTopology.js';
import {buildKirchhoffCompositeMesh} from '../../src/physics/kirchhoffCompositeMesh.js';
import {kirchhoffMaterialProfile} from '../../src/physics/kirchhoffMaterialProfile.js';
import {GUIDEWIRE_TIP_CORE_LENGTH_MM,GUIDEWIRE_SOFT_TIP_LENGTH_MM} from '../../src/physics/guidewireMaterialProfile.js';
import {BERENSTEIN_TIP_SHAPE_LENGTH_MM} from '../../src/physics/catheterMaterialProfile.js';
import {createCompositeChainWorkspace,assembleCompositeChain,solveCompositeChainIncrement} from '../../src/physics/kirchhoffCompositeChain.js';

const root=new URL('../../',import.meta.url),output=process.argv[2]??'/tmp/oet-composite-material-feed.json';
const paths=['src/physics/kirchhoffCompositeTopology.js','src/physics/kirchhoffCompositeMesh.js',
    'src/physics/kirchhoffCompositeChain.js','src/physics/kirchhoffCompositeElementFast.js','src/physics/kirchhoffCompositeElementFastKernelBytes.js',
    'src/physics/kirchhoffMaterialProfile.js','scripts/physics/benchmark-composite-material-feed.mjs'];
const hashes=()=>Object.fromEntries(paths.map(p=>[p,createHash('sha256').update(fs.readFileSync(new URL(p,root))).digest('hex')]));
const report={scope:'Rebuilding topology/material mesh during feed, then one constitutive assembly and direction. Real Glidewire/Berenstein profiles, synthetic smooth geometry. No dynamics, contact, admission, anatomy or FPS claim.',
    sourceBefore:hashes(),date:new Date().toISOString(),warmups:3,runs:8,cases:[]};
for(const insertion of [9,160,310]) {
    const rows=[];
    for(let iteration=0;iteration<report.warmups+report.runs;iteration++) {
        const catheterInsertion=insertion+.01*iteration,start=performance.now();
        const topology=buildKirchhoffCompositeTopology({
            wire:compositeToolFromTipProfile({profile:kirchhoffMaterialProfile('glidewire'),materialInterval:[0,500],insertion:318,
                radius:.4445,tipBreakpoints:[GUIDEWIRE_TIP_CORE_LENGTH_MM,GUIDEWIRE_SOFT_TIP_LENGTH_MM]}),
            catheter:compositeToolFromTipProfile({profile:kirchhoffMaterialProfile('berenstein'),materialInterval:[0,500],insertion:catheterInsertion,
                radius:.8,innerRadius:.485,tipBreakpoints:[BERENSTEIN_TIP_SHAPE_LENGTH_MM]})});
        // Even subdivisions within each required section avoid accidentally
        // placing an optional uniform-grid point arbitrarily close to a tip.
        const coordinates=[topology.interval[0]];
        for(const section of topology.sections) {
            const count=Math.ceil((section.end-section.start)/5);
            for(let i=1;i<=count;i++)coordinates.push(i===count?section.end:section.start+(section.end-section.start)*i/count);
        }
        const partitioned=performance.now();
        const mesh=buildKirchhoffCompositeMesh({topology,meshCoordinates:coordinates,
            sampleCenterline:x=>[x,.2*Math.sin(x/40),.1*Math.cos(x/37)],spinFields:{wire:0,catheter:.1}});
        const compiled=performance.now(),workspace=createCompositeChainWorkspace(mesh.layout),ready=performance.now();
        assembleCompositeChain(mesh.data,workspace);const assembled=performance.now();
        const result=solveCompositeChainIncrement(workspace,{diagonal:new Float64Array(mesh.layout.dofCount).fill(3),tolerance:1e-7});
        const done=performance.now();if(!result.converged)throw new Error('Original direction residual failed');
        if(iteration>=report.warmups)rows.push({catheterInsertion,nodes:mesh.layout.nodeCount,dofCount:mesh.layout.dofCount,
            topologyMs:partitioned-start,materialMeshMs:compiled-partitioned,workspaceMs:ready-compiled,assemblyMs:assembled-ready,
            solveMs:done-assembled,totalMs:done-start,materialEvaluations:mesh.materialCells.concat(mesh.boundaryCells)
                .reduce((sum,c)=>sum+c.quadrature.evaluations,0),residual:result.maximumResidual});
    }
    const median=key=>{const s=rows.map(r=>r[key]).sort((a,b)=>a-b);return(s[3]+s[4])/2;};
    report.cases.push({insertion,medians:Object.fromEntries(['topologyMs','materialMeshMs','workspaceMs','assemblyMs','solveMs','totalMs'].map(k=>[k,median(k)])),rows});
}
report.sourceAfter=hashes();report.sourceStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,sourceStable:report.sourceStable,cases:report.cases.map(({rows,...summary})=>summary)},null,2));
