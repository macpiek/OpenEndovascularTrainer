// Research screen only: can a few PD iterations produce a useful, cheap
// initial pose for the original Newton timestep? No runtime switch is added.
// Each guess uses a fresh copy; its velocities, friction history and reactions
// are discarded before measuring the ORIGINAL implicit-step residual.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay,assembleDiscoveredSharedAxis} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {feedSharedAxisNative,captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisProjective} from '../src/physics/kirchhoffSharedAxisProjective.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const root=new URL('../',import.meta.url),source=fs.readFileSync(new URL('reports/performance-scenarios/001-stage-05-replay.json.gz',root));
const fixture=JSON.parse(gunzipSync(source)),request=fixture.stepRequest,dt=request.dt;
const output=process.argv[2]??new URL('reports/performance-scenarios/001-projective-predictor-screen.json',root);
const selected=new Set([1,29,40,60,150,203,288]);
const options={...request.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,reuseFrictionAssembly:true};
const report={recordedAt:new Date().toISOString(),researchOnly:true,productionDefaultsChanged:false,
    fixtureSha256:createHash('sha256').update(source).digest('hex'),scriptSha256:createHash('sha256').update(fs.readFileSync(new URL(import.meta.url))).digest('hex'),dt,selectedSteps:[...selected],cases:[],complete:false,
    sourceHashes:Object.fromEntries(['kirchhoffSharedAxisProjective.js','kirchhoffSharedAxisTimeStep.js','kirchhoffSharedAxisNative.js'].map(file=>
        [file,createHash('sha256').update(fs.readFileSync(new URL('src/physics/'+file,root))).digest('hex')])),
    limitations:['Single-pass screening timings include JIT and are not a speedup benchmark.',
        'PD penalty contacts and chordal material forces are used only to propose a pose, never to certify a physical step.',
        'Raw force residual is measured with zero reactions; it is not an equilibrium certificate.',
        'This screen does not run Newton from the guess and cannot establish final trajectory accuracy.']};
const finish=iterator=>{let n;do{n=iterator.next();}while(!n.done);return n.value;};
const offset=step=>step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
const difference=(s,t)=>Math.max(...s.positions.map((p,i)=>Math.hypot(...p.map((v,k)=>v+s.origin[k]-t.positions[i][k]-t.origin[k]))));
function velocityGuess(s){
    const increment=new Float64Array(s.layout.dofCount),decay=.98**(dt*120);
    for(let n=0;n<s.positions.length;n++)for(let a=0;a<3;a++){
        const dof=s.layout.positions[n]+a;
        if(!s.fixed[dof])increment[dof]=dt*decay*(s.velocities?.[n]?.[a]??0);
    }
    for(const {spec,last} of s.materials)for(let e=0;e<last;e++){
        const dof=s.layout.spins.get(spec.id)[e],p=s.positions[e],v=s.positions[e+1].map((x,a)=>x-p[a]);
        if(!s.fixed[dof])increment[dof]=dt*decay*v.reduce((sum,x,a)=>sum+x*(s.angularVelocities?.[spec.id]?.[e]?.[a]??0),0)/Math.hypot(...v);
    }
    applySharedAxisNativeIncrement(s,increment,new Float64Array(s.multipliers.length));
}
const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
const guesses=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
try {
    let state=restoreSharedAxisReplay({...fixture,...options},anatomy.field),rotations=request.rotations;
    const flow=createCrossoverFlow();flow.time=request.options.bloodFlow.time;
    for(let step=0;step<=Math.max(...selected);step++){
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+offset(step)}));
        const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...tools.map(t=>t.insertion)));
        const saved=selected.has(step)?captureSharedAxisReplay(state,fixture.sheath):null;
        const start=performance.now(),next=finish(advanceSharedAxis(state,rotations,dt,tools,{...options,bloodFlow}));
        assert.ok(next.state&&next.result.converged&&next.result.certificateBound<=1e-6&&next.result.residual.length<=1e-5&&next.result.quality.maxPenetration<=1e-5);
        if(saved){
            const item={step,reference:{ms:performance.now()-start,iterations:next.result.iterations,factorizations:next.result.factorizations,
                certificateBound:next.result.certificateBound,subdivisions:next.result.subdivisions},guesses:[]};
            report.cases.push(item);
            for(const method of ['incoming','velocity',1,2,4,8]){
                // Fork discovery and physical histories via replay. Never run
                // PD on the authoritative reference continuation.
                const restored=restoreSharedAxisReplay(saved,guesses.field);
                const s=feedSharedAxisNative(restored,Object.fromEntries(tools.map(t=>[t.id,t.insertion])),{contactCandidateMargin:.01});
                const incoming=captureSharedAxisNative(s),history={velocities:s.velocities,angularVelocities:s.angularVelocities,
                    acceptedWallGaps:s.acceptedWallGaps,wallFrictionHistory:s.wallFrictionHistory,acceptedSolves:s.acceptedSolves};
                const feedById=Object.fromEntries(tools.map(t=>[t.id,t.insertion-restored.materials.find(m=>m.spec.id===t.id).spec.insertion]));
                const started=performance.now();let pd,pose;
                if(typeof method==='number'){
                    pd=finish(iterateSharedAxisProjective(s,dt,{projective:{iterations:method},feedById,bloodFlow}));
                    if(pd.converged)pose=captureSharedAxisNative(s);
                }else if(method==='velocity'){
                    velocityGuess(s);pose=captureSharedAxisNative(s);
                }
                const guess={method:typeof method==='number'?`pd-${method}`:method,guessMs:performance.now()-started,
                    ...(pd?{status:pd.status,converged:pd.converged,error:pd.error,quality:pd.quality,pd:pd.pd,backtracks:pd.backtracks}: {})};
                item.guesses.push(guess);
                if(pd&&!pd.converged)continue;
                // Preserve the original time target and friction reference.
                // Copy only the guessed pose back, not PD's step/history.
                restoreSharedAxisNative(s,incoming);Object.assign(s,history);
                s.dynamicStep=null;s.wallFrictionStep=null;
                prepareSharedAxisDynamicStep(s,dt,bloodFlow);
                prepareSharedAxisWallFriction(s,{feedById,liveNormalLoad:true,lightweightFriction:true});
                if(pose)restoreSharedAxisNative(s,{...pose,multipliers:incoming.multipliers});
                s.multipliers.fill(0);
                guess.shapeDifferenceToFullStepMm=difference(s,next.state);
                try {
                    const measure=assembleDiscoveredSharedAxis(s,{withTangent:false,wasmMaterial:true,reuseMaterialScratch:true});
                    guess.residual={force:measure.force,torque:measure.torque,constraint:measure.constraint};
                }catch(error){guess.residualError=error.message;}
            }
            console.log(JSON.stringify({step,reference:item.reference,guesses:item.guesses.map(({quality,pd,...g})=>g)}));
        }
        state=next.state;rotations=next.rotations;flow.update(dt);
    }
    report.complete=true;
}catch(error){report.error=error.stack;process.exitCode=1;console.error(error.stack);}
finally {anatomy.dispose();guesses.dispose();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');}
