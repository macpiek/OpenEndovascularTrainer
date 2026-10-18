// Isolated contact-feasibility experiment. No dynamics or production defaults
// are changed, and no reference trajectory is injected into the candidate.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {createSharedAxisNative,captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisConstraintRows} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {measureSharedAxisQuality} from '../src/physics/kirchhoffSharedAxisDiagnostics.js';
import {iterateSharedAxisProjection} from '../src/physics/kirchhoffSharedAxisProjection.js';

const [input,diagnosis,output]=process.argv.slice(2);
if(!input||!diagnosis||!output)throw new Error('Provide captured failures, projection diagnosis and output path');
const record=JSON.parse(gunzipSync(fs.readFileSync(input))),traces=JSON.parse(fs.readFileSync(diagnosis));
const anatomy=await loadCoupledRuntimeAnatomy(undefined,record.anatomy),results=[];
const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};
function evaluate(s) {
    for(let i=0;i<64;i++) {
        s.chain.gradient.fill(0);const rows=assembleSharedAxisConstraintRows(s,{withTangent:false,outerMaterialAt:sharedAxisOuterMaterialAt});
        if(!s.pendingVesselRows?.size)return rows;
        extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
    }
    throw new Error('Discovery did not settle');
}
try {
    for(const [index,captured] of record.failures.entries())for(const pieces of [1,2,4,8]) {
        const old=restoreSharedAxisReplay(captured.fixture,anatomy.field);
        // Refresh the original grid's valid inside certificates before asking
        // about any new subdivision samples. No sign is forced or overridden.
        evaluate(old);
        const conflicts=traces.results[index].trace.filter(t=>t.kind==='incompatible-active-constraints');
        // Use the diagnosed contact patch, not an anatomical branch or a
        // recording step. Trace formats contain nested row descriptions.
        const edges=new Set();
        const collect=value=>{if(!value||typeof value!=='object')return;
            if(Number.isInteger(value.edge))for(let e=value.edge-1;e<=value.edge+2;e++)if(e>=2&&e<old.coordinates.length-1)edges.add(e);
            for(const child of Object.values(value))if(typeof child==='object')collect(child);
        };
        conflicts.forEach(collect);
        if(!edges.size)throw new Error('No diagnosed conflict edges');
        let s=old;
        if(pieces>1) {
            const spatialKnots=old.coordinates.slice();
            for(const e of edges)for(let part=1;part<pieces;part++)spatialKnots.push(old.coordinates[e]+part/pieces*(old.coordinates[e+1]-old.coordinates[e]));
            s=createSharedAxisNative({tools:old.materials.map(m=>m.spec),spacing:old.spacing,wallSamples:old.wallSamples,
                startCoordinate:old.startCoordinate,boundaryCoordinates:old.boundaryCoordinates,previous:old,minimumEdgeLength:old.minimumEdgeLength,
                origin:old.origin,maxBendAngle:old.maxBendAngle,rebaseNearTips:old.rebaseNearTips,fractionalTipThreshold:old.fractionalTipThreshold,
                adaptiveMesh:old.adaptiveMesh,omitInactiveSampleRows:old.omitInactiveSampleRows,spatialKnots});
            // These fixtures use only the native prescribed proximal DOFs.
            const defaults=createSharedAxisNative({tools:old.materials.map(m=>m.spec),spacing:old.spacing,spatialKnots:old.coordinates,
                startCoordinate:old.startCoordinate,boundaryCoordinates:old.boundaryCoordinates,minimumEdgeLength:old.minimumEdgeLength});
            if(old.fixed.some((v,i)=>v!==defaults.fixed[i]))throw new Error('Custom fixed DOFs require explicit remapping');
        }
        const out={subdivisions:captured.subdivisions,part:captured.part,pieces,refinedEdges:[...edges].sort((a,b)=>a-b),
            oldNodes:old.positions.length,nodes:s.positions.length,admissible:false,iterations:0};results.push(out);
        out.initialPolylineDifferenceMm=Math.max(...s.coordinates.map((x,i)=>{
            let e=0;while(e+2<old.coordinates.length&&old.coordinates[e+1]<x)e++;
            const t=(x-old.coordinates[e])/(old.coordinates[e+1]-old.coordinates[e]);
            return Math.hypot(...s.positions[i].map((v,k)=>v-((1-t)*old.positions[e][k]+t*old.positions[e+1][k])));
        }));
        const start=performance.now();
        try {
            for(let iteration=0;iteration<64;iteration++) {
                const rows=evaluate(s),quality=measureSharedAxisQuality(s,rows);
                const lengthMm=Math.max(...s.positions.slice(1).map((p,i)=>Math.abs(Math.hypot(...p.map((v,k)=>v-s.positions[i][k]))-(s.coordinates[i+1]-s.coordinates[i]))));
                out.iterations=iteration;out.penetrationMm=quality.maxPenetration;out.lengthErrorMm=lengthMm;
                out.minBendGap=Math.min(0,...rows.filter(r=>r.subtype==='bend-limit').map(r=>r.gap));
                if(quality.finite&&quality.maxPenetration<=5e-6&&lengthMm<=5e-6&&quality.bodies.every(b=>b.maxLengthError<=5e-6)&&out.minBendGap>=-1e-9){out.admissible=true;break;}
                const direction=finish(iterateSharedAxisProjection(s,{rows}));
                if(!direction.converged){out.failure=direction.failure??'linear-residual';out.linearResidual=direction.residual;break;}
                const pose=captureSharedAxisNative(s);let accepted=false;
                for(let trial=0;trial<14;trial++) {restoreSharedAxisNative(s,pose);try {
                    applySharedAxisNativeIncrement(s,direction.increment,direction.zeroReactions,2**(-trial));evaluate(s);accepted=true;break;
                }catch(error){if(error.code!=='trial-outside-vessel')throw error;}}
                if(!accepted)throw new Error('Projection crossed surface');
            }
        }catch(error){out.failure=error.message;if(error.contact)out.contact=error.contact;}
        out.cpuMs=performance.now()-start;console.log(JSON.stringify(out));
    }
}finally{anatomy.dispose();}
fs.writeFileSync(output,JSON.stringify({input,diagnosis,scope:'Isolated geometry projection after local refinement; not a dynamic replay, performance benchmark or realism validation.',results},null,2)+'\n');
