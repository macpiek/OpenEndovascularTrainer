import assert from 'node:assert/strict';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const runtime=process.argv[2];
const read=rel=>import(pathToFileURL(`${runtime}/${rel}`));
const {createCompositeFrictionEquationWorkspace,evaluateCompositeFrictionEquation,measureCompositeFriction}=await read('src/physics/kirchhoffCompositeFriction.js');
const {createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep}=await read('src/physics/kirchhoffCompositeJointTimeStep.js');
const fixtures={wall:await read('tests/audit-wall-fixture.mjs'),lumen:await read('tests/audit-lumen-fixture.mjs')};
const tol={slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9};
const equation=input=>structuredClone(evaluateCompositeFrictionEquation(input,createCompositeFrictionEquationWorkspace()));
const normalResidual=(Fn,g,k)=>Fn-k*g>0?g:Fn/k;
const linearResidual=(e,dFt,dFn,dSlip)=>[0,1].map(i=>e.residual[i]+e.normalDerivative[i]*dFn+[0,1].reduce((s,j)=>s+e.tractionJacobian[2*i+j]*dFt[j]+e.slipJacobian[2*i+j]*dSlip[j],0));
const assertTiny=v=>assert.ok(Math.abs(v)<3e-16,`linear residual ${v}`);
const algebra=[];
for(const Fn of [-.25,-1e-30,0]) {
    const input={normalForce:Fn,traction:[.125,-.075],slip:[.17,-.09],mu:[.3,.6],penalty:7},e=equation(input),g=.02,normalPenalty=5;
    assert.equal(Fn-normalPenalty*g>0,false);assert.ok(e.slipJacobian.every(v=>v===0));
    if(Fn<0)assert.ok(e.normalDerivative.every(v=>v===0));
    const dFn=-Fn,dFt=input.traction.map(v=>-v),linear=linearResidual(e,dFt,dFn,[1.23,-.87]);linear.forEach(assertTiny);
    const trials=[1,.5,.125].map(alpha=>({alpha,Fn:(1-alpha)*Fn,Ft:input.traction.map(v=>(1-alpha)*v),
        normalNcp:normalResidual((1-alpha)*Fn,g,normalPenalty)}));
    assert.ok(trials[0].Ft.every(v=>v===0));assert.ok(trials[0].Fn===0);
    algebra.push({Fn,normalInactive:true,projectionBranch:e.branch,DPz:Array.from(e.slipJacobian),equationDNormal:Array.from(e.normalDerivative),dFn,dFt,linearResidual:linear,trials});
}
const input={normalForce:0,traction:[0,0],slip:[1,0],mu:[.3,.6],penalty:50},e=equation(input),dFn=.4;
const activeCounterexample={normalForce:0,gap:-.1,normalActive:true,dFn,
    correctDFt:[-.12,0],correctLinearResidual:linearResidual(e,[-.12,0],dFn,[0,0]),
    incorrectlyZeroedLinearResidual:linearResidual(e,[0,0],dFn,[0,0])};
activeCounterexample.correctLinearResidual.forEach(assertTiny);assertTiny(activeCounterexample.incorrectlyZeroedLinearResidual[0]-.0024);
const disabledAxes=[];
for(const mu of [[0,.6],[.3,0],[0,0]]) {
    const e=equation({normalForce:2,traction:[.125,-.075],slip:[.17,-.09],mu,penalty:7});
    for(let i=0;i<2;i++)if(mu[i]===0){assert.equal(e.residual[i],[.125,-.075][i]/7);assert.equal(e.tractionJacobian[2*i+i],1/7);
        assert.ok(e.tractionJacobian[2*i+1-i]===0);assert.ok(e.normalDerivative[i]===0);assert.ok(e.slipJacobian[2*i]===0);assert.ok(e.slipJacobian[2*i+1]===0);}
    disabledAxes.push({mu,residual:Array.from(e.residual),G:Array.from(e.slipJacobian),tractionJacobian:Array.from(e.tractionJacobian),DNormal:Array.from(e.normalDerivative)});
}
let cancellation;
for(const Ft of [.1,.2,.3,.7,Math.PI,1/3])for(const k of [3,7,11,50,500]){
    const delta=-(Ft/k)/(1/k),target=Ft+delta;
    if(target!==0&&!cancellation){const physical=measureCompositeFriction({normalForce:0,traction:[target,0],slip:[1,0],mu:[.3,.6],...tol});
        assert.equal(physical.converged,false);assert.equal(physical.coneViolation,Infinity);cancellation={Ft,k,delta,naiveTarget:target,exactBacksubTarget:(1-1)*Ft,originalConeViolation:'Infinity'};}
}
assert.ok(cancellation);
const wholeSteps=[];let acceptedSteps=0,rejectedSteps=0;
const physicalResult=r=>({state:r.state,perTool:r.perTool,contactForces:r.contactForces,boundaryForces:r.boundaryForces,spinReactions:r.spinReactions,balances:r.balances});
for(const kind of ['wall','lumen'])for(const mu of [[0,.006],[.006,0],[0,0]])for(const k of [5,50,500]) {
    const api=fixtures[kind],f=api.fixture(),policy=kind==='wall'?f.wall:f.contacts;policy.friction.mu=mu;
    const workspace=createCompositeJointTimeStepWorkspace(f.state),steps=[];let state=f.state;
    for(let stage=0;stage<2;stage++) {
        const before=structuredClone(state),cold=advanceCompositeJointTimeStep(state,api.options(f,state,{k})),warm=advanceCompositeJointTimeStep(state,api.options(f,state,{k,workspace}));
        assert.equal(cold.accepted,true,`${kind} ${mu} k${k}: ${cold.status} ${cold.error}`);assert.equal(warm.accepted,true);acceptedSteps+=2;
        assert.deepEqual(state,before);assert.deepEqual(physicalResult(warm),physicalResult(cold));
        const history=kind==='wall'?warm.state.wallFrictionState:warm.state.lumenFrictionState,proof=warm.diagnostics.certificate[kind==='wall'?'wallFriction':'friction'].samples[0];
        for(let i=0;i<2;i++)if(mu[i]===0)assert.ok(history.tractions[i]===0);
        assert.equal(measureCompositeFriction({normalForce:proof.Fn,traction:Array.from(history.tractions),slip:proof.slip,mu,...tol}).converged,true);
        const failed=advanceCompositeJointTimeStep(state,api.options(f,state,{k,workspace,budget:{evaluations:cold.diagnostics.evaluations-1}}));
        rejectedSteps++;assert.equal(failed.accepted,false);assert.equal(failed.state,state);assert.deepEqual(state,before);
        const retry=advanceCompositeJointTimeStep(state,api.options(f,state,{k,workspace}));acceptedSteps++;assert.equal(retry.accepted,true);assert.deepEqual(physicalResult(retry),physicalResult(cold));
        steps.push({stage,Fn:proof.Fn,Ft:Array.from(history.tractions),slip:proof.slip,work:proof.work,fullEvaluations:cold.diagnostics.evaluations,rejection:failed.status});state=warm.state;
    }
    wholeSteps.push({kind,mu,k,steps});
}
const evidence={status:'PASS',algebra,activeCounterexample,disabledAxes,cancellation,wholeSteps,acceptedSteps,rejectedSteps,
    findings:[],notes:['Disabled axes satisfy exact Ft_i/k equations, but no whole-step failure reproduced; no speculative fix requested.',
        'Root source unchanged; probes use copied fixtures and frozen runtime sources.']};
fs.writeFileSync(process.argv[3],JSON.stringify(evidence,null,2)+'\n');
console.log(JSON.stringify({status:evidence.status,acceptedSteps,rejectedSteps,algebraCases:algebra.length,activeCounterexample,cancellation,wholeStepCases:wholeSteps.length},null,2));
