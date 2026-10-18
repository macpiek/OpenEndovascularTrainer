// Isolated linearized geometry diagnosis, not a crossover performance run.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {extendSharedAxisNativeRows,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisConstraintRows} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {iterateSharedAxisProjection} from '../src/physics/kirchhoffSharedAxisProjection.js';
const [input,output]=process.argv.slice(2);
if(!input||!output)throw new Error('Provide captured failures and output path');
const record=JSON.parse(gunzipSync(fs.readFileSync(input))),anatomy=await loadCoupledRuntimeAnatomy(undefined,record.anatomy),results=[];
try {
    for(const captured of record.failures) {
        const s=restoreSharedAxisReplay(captured.fixture,anatomy.field);let rows;
        for(let i=0;i<64;i++) {
            s.chain.gradient.fill(0);rows=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});
            if(!s.pendingVesselRows?.size)break;
            extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
        }
        const trace=[];trace.captureConflicts=true;
        const it=iterateSharedAxisProjection(s,{rows},{trace});let n;do{n=it.next();}while(!n.done);
        const {converged,failure,residual,factorizations,activeSetAttempts}=n.value;
        const result={subdivisions:captured.subdivisions,part:captured.part,browser:captured.linear,
            node:{converged,failure,residual,factorizations,activeSetAttempts},trace};results.push(result);
        console.log(JSON.stringify({...result,trace:trace.filter(x=>x.kind==='incompatible-active-constraints')}));
    }
}finally{anatomy.dispose();}
fs.writeFileSync(output,JSON.stringify({input,scope:'Isolated projection replay; not whole-scenario performance or cross-runtime trajectory parity.',results},null,2)+'\n');
