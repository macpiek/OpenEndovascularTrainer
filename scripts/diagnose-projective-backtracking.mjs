// Does reducing a failed nonlinear geometry displacement restore feasibility?
// This isolated probe does not advance dynamics or validate a whole maneuver.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisConstraintRows} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {measureSharedAxisQuality} from '../src/physics/kirchhoffSharedAxisDiagnostics.js';
import {iterateSharedAxisProjection} from '../src/physics/kirchhoffSharedAxisProjection.js';
const [input,output]=process.argv.slice(2);
if(!input||!output)throw new Error('Provide captured failures and output path');
const record=JSON.parse(gunzipSync(fs.readFileSync(input))),anatomy=await loadCoupledRuntimeAnatomy(undefined,record.anatomy),results=[];
const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};
try {
    for(const captured of record.failures)for(const scale of [1,.5,.25,.125,.0625,0]) {
        const s=restoreSharedAxisReplay(captured.fixture,anatomy.field),out={subdivisions:captured.subdivisions,part:captured.part,scale,admissible:false,iterations:0};results.push(out);
        const evaluate=()=>{for(let i=0;i<64;i++) {
            s.chain.gradient.fill(0);const rows=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});
            if(!s.pendingVesselRows?.size)return rows;
            extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
        }throw new Error('Discovery did not settle');};
        try {
            if(scale!==1){s.positions.forEach((p,i)=>p.forEach((v,k)=>{p[k]=s.dynamicStep.positions[i][k]+scale*(v-s.dynamicStep.positions[i][k]);}));
                applySharedAxisNativeIncrement(s,new Float64Array(s.layout.dofCount),new Float64Array(s.multipliers.length));}
            for(let iteration=0;iteration<32;iteration++) {
                const rows=evaluate(),quality=measureSharedAxisQuality(s,rows);
                const lengthMm=Math.max(...s.positions.slice(1).map((p,i)=>Math.abs(Math.hypot(...p.map((v,k)=>v-s.positions[i][k]))-(s.coordinates[i+1]-s.coordinates[i]))));
                out.iterations=iteration;out.penetrationMm=quality.maxPenetration;out.lengthErrorMm=lengthMm;
                if(quality.finite&&quality.maxPenetration<=5e-6&&lengthMm<=5e-6&&quality.bodies.every(b=>b.maxLengthError<=5e-6)&&!rows.some(r=>r.subtype==='bend-limit'&&r.gap< -1e-9)){out.admissible=true;break;}
                const direction=finish(iterateSharedAxisProjection(s,{rows}));
                if(!direction.converged){out.failure=direction.failure??'linear-residual';out.linearResidual=direction.residual;break;}
                const pose=captureSharedAxisNative(s);let accepted=false;
                for(let trial=0;trial<14;trial++) {restoreSharedAxisNative(s,pose);try {
                    applySharedAxisNativeIncrement(s,direction.increment,direction.zeroReactions,2**(-trial));evaluate();accepted=true;break;
                }catch(error){if(error.code!=='trial-outside-vessel')throw error;}}
                if(!accepted)throw new Error('Projection crossed surface');
            }
        }catch(error){out.failure=error.message;}
        console.log(JSON.stringify(out));
    }
}finally{anatomy.dispose();}
fs.writeFileSync(output,JSON.stringify({input,scope:'Isolated feasibility of geometry backtracking, not whole-trajectory stability or performance.',results},null,2)+'\n');
