import fs from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout,createCompositeChainWorkspace,assembleCompositeChain} from '../../src/physics/kirchhoffCompositeChain.js';
const root=new URL('../../',import.meta.url),output=process.argv[2]??'/tmp/oet-composite-gradient.json';
const files=['src/physics/kirchhoffCompositeChain.js','src/physics/kirchhoffCompositeElement.js','src/physics/kirchhoffCompositeElementExact.js',
    'src/physics/kirchhoffCompositeElementExactKernelBytes.js','src/physics/kirchhoffCompositeElementFast.js','src/physics/kirchhoffCompositeElementFastKernelBytes.js',
    'src/physics/kirchhoffLinearKernel.js','src/physics/kirchhoffLinearKernelBytes.js','scripts/physics/build-composite-element-exact.mjs',
    'scripts/physics/benchmark-composite-gradient.mjs'];
const hash=v=>createHash('sha256').update(v).digest('hex'),hashes=()=>Object.fromEntries(files.map(p=>[p,hash(fs.readFileSync(new URL(p,root)))]));
const bytes=a=>Buffer.from(a.buffer,a.byteOffset,a.byteLength);
const statistics=values=>{const sorted=values.toSorted((a,b)=>a-b),i=sorted.length>>1;return{mean:values.reduce((a,b)=>a+b)/values.length,
    median:sorted.length%2?sorted[i]:(sorted[i-1]+sorted[i])/2,max:sorted.at(-1),samples:values};};
const report={scope:'Repeated whole Chain constitutive assembly, full Exact E/g/H versus same exact E/g with explicit invalid/stale H. Declared constant anisotropic materials on one3D centerline, independent spins/dsDx/winding. No timestep/direction/physics-certificate/FPS claim. Preparation and original JS oracle parity are outside timed assembly.',
    sourceBefore:hashes(),node:process.version,cases:[]};
for(const count of [65,128]) {
    const coordinates=Array.from({length:count},(_,i)=>3*i),rest=coordinates.map(x=>[x,.2*Math.sin(x/40),.1*Math.cos(x/37)]);
    const positions=rest.map((p,i)=>p.map((v,axis)=>v+.03*Math.sin(.1*i+axis)));
    const layout=createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter']));
    const data={coordinates,positions,reference:captureCompositeReferenceFrames(rest),tools:[0,1].map(i=>({id:i?'catheter':'wire',
        angles:Float64Array.from(coordinates.slice(1),x=>.3*i+.02*x),referenceTwists:new Float64Array(count-2).fill(i?2*Math.PI:0),dsDx:.7+.4*i,
        material:compileCompositeMaterial({stiffness:[[3+4*i,.3,.2],[.3,5,-.1],[.2,-.1,2+i]],intrinsic:[.01,-.02,.03],energyOffset:.2})}))};
    const createStart=performance.now(),full=createCompositeChainWorkspace(layout,{elementBackend:'wasm-exact'}),gradient=createCompositeChainWorkspace(layout,{elementBackend:'wasm-exact'});
    const workspaceCreationMs=performance.now()-createStart,oracle=createCompositeChainWorkspace(layout,{elementBackend:'javascript'});
    assembleCompositeChain(data,oracle);const originalEnergy=oracle.energy,originalGradient=oracle.gradient.slice();
    // A NaN sentinel proves that gradient mode never clears, reads, checks or
    // scatters a hidden Hessian, while its valid first-order result is finite.
    gradient.hessian.fill(NaN);const staleHash=hash(bytes(gradient.hessian)),rows=[],warmups=[];
    for(let pair=-4;pair<8;pair++) {
        const row={pair};
        for(const order of pair%2?['gradient','full']:['full','gradient']){
            const workspace=order==='full'?full:gradient,start=performance.now();assembleCompositeChain(data,workspace,{order});row[order]=performance.now()-start;
        }
        assert.equal(full.hessianValid,true);assert.equal(gradient.hessianValid,false);assert.equal(hash(bytes(gradient.hessian)),staleHash);
        let maximumGradientDifference=0,maximumOracleDifference=0;
        full.gradient.forEach((v,i)=>{maximumGradientDifference=Math.max(maximumGradientDifference,Math.abs(v-gradient.gradient[i]));
            maximumOracleDifference=Math.max(maximumOracleDifference,Math.abs(originalGradient[i]-gradient.gradient[i]));
            assert.ok(Math.abs(v-gradient.gradient[i])<=2e-12*(1+Math.abs(v)));
            assert.ok(Math.abs(originalGradient[i]-gradient.gradient[i])<=2e-12*(1+Math.abs(originalGradient[i])));});
        assert.ok(Math.abs(gradient.energy-originalEnergy)<=2e-12*(1+Math.abs(originalEnergy)));assert.equal(full.energy,gradient.energy);
        row.maximumGradientDifference=maximumGradientDifference;row.maximumOracleDifference=maximumOracleDifference;
        row.gradientByteIdentical=bytes(full.gradient).equals(bytes(gradient.gradient));
        (pair<0?warmups:rows).push(row);
    }
    const fullMs=statistics(rows.map(row=>row.full)),gradientMs=statistics(rows.map(row=>row.gradient));
    report.cases.push({nodes:count,dofs:layout.dofCount,band:layout.band,workspaceCreationMs,fullMs,gradientMs,medianSpeedup:fullMs.median/gradientMs.median,
        maximumGradientDifference:Math.max(...rows.map(row=>row.maximumGradientDifference)),maximumOracleDifference:Math.max(...rows.map(row=>row.maximumOracleDifference)),
        gradientByteIdentical:rows.every(row=>row.gradientByteIdentical),staleHessianNeverTouched:true,warmups,rows});
}
report.sourceAfter=hashes();report.sourceStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);assert.ok(report.sourceStable);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({output,sourceStable:report.sourceStable,cases:report.cases.map(({rows,warmups,...row})=>row)},null,2));
