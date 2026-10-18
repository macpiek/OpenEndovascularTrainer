import test from 'node:test';
import assert from 'node:assert/strict';
import {solveXpbdChainBlock,xpbdConstraintResidual} from '../src/physics/xpbdChainBlock.js';
import {createSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisXpbd} from '../src/physics/sharedAxisXpbdExperimental.js';
import {iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
const close=(a,b,e=1e-8)=>assert.ok(Math.abs(a-b)<=e,`${a} vs ${b}`);
const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};

test('unilateral merit is continuous through contact release and zero only at complementarity',()=>{
    const residual=(gap,lambda)=>xpbdConstraintResidual({gap,lambda,alpha:.01,unilateral:true},2);
    close(residual(.3,0),0);close(residual(.3,1e-15),2e-15,1e-20);
    close(residual(-.1,0),-.1);close(residual(-.1,10),0);
    assert.notEqual(residual(.3,1),0);assert.ok(residual(.3,-.1)<0);
    close(xpbdConstraintResidual({gap:.3,lambda:0,alpha:.01},2),.3);
});

test('global block propagates a tip displacement through the whole inextensible chain in one solve',()=>{
    const n=120,weights=new Float64Array(n).fill(1);weights[0]=0;
    const displacement=new Float64Array(n);displacement[n-1]=1;
    const rows=Array.from({length:n-1},(_,i)=>({dofs:[i,i+1],jacobian:[-1,1],gap:i===n-2?1:0,alpha:0,lambda:0}));
    const result=solveXpbdChainBlock(rows,weights,displacement);
    assert.equal(result.factorizations,1);assert.equal(result.band,2);
    result.increment.forEach((x,i)=>close(x+displacement[i],0));
});

test('global compliant block satisfies both primal balance and compliant constraints',()=>{
    const rows=[{dofs:[0,1],jacobian:[-1,1],gap:.4,alpha:.01,lambda:.3},
        {dofs:[1,2],jacobian:[-1,1],gap:-.1,alpha:.2,lambda:-.2}];
    const w=[.5,1,2],d=[.2,-.1,.3],r=solveXpbdChainBlock(rows,w,d);
    for(let i=0;i<rows.length;i++)close(rows[i].gap+rows[i].dofs.reduce((s,k,j)=>s+rows[i].jacobian[j]*r.increment[k],0)+rows[i].alpha*r.lambda[i],0);
    for(let k=0;k<3;k++)close(d[k]+r.increment[k],w[k]*rows.reduce((s,row,i)=>s+row.dofs.reduce((t,d,j)=>t+(d===k?row.jacobian[j]*r.lambda[i]:0),0),0));
});

test('unilateral reactions release tensile contacts and activate violated contacts',()=>{
    for(const initialLambda of [0,10]) {
        const r=solveXpbdChainBlock([{dofs:[0],jacobian:[1],gap:.5,alpha:1e-6,lambda:initialLambda,unilateral:true}], [1],[0]);
        close(r.lambda[0],0);close(r.increment[0],0);
    }
    const r=solveXpbdChainBlock([{dofs:[0],jacobian:[1],gap:-.5,alpha:1e-6,lambda:0,unilateral:true}], [1],[0]);
    assert.ok(r.lambda[0]>0);close(-.5+r.increment[0]+1e-6*r.lambda[0],0);
});

test('feasible dual pivots converge on conflicting contact normals without release/reactivation cycles',()=>{
    let seed=718;const random=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/2**32);
    // Case 53 cycles with simultaneous active-set toggles. Check independent
    // KKT conditions, rather than hard-coding this implementation's solution.
    for(let sample=0;sample<200;sample++) {
        const displacement=Array.from({length:3},()=>random()*2-1);
        const rows=Array.from({length:6},()=>({dofs:[0,1,2],jacobian:Array.from({length:3},()=>random()*2-1),
            gap:random()*2-1,alpha:.01,lambda:0,unilateral:true}));
        const result=solveXpbdChainBlock(rows,[1,1,1],displacement);
        rows.forEach((r,i)=>{
            const gap=r.gap+r.jacobian.reduce((v,x,k)=>v+x*result.increment[k],0)+r.alpha*result.lambda[i];
            assert.ok(result.lambda[i]>=0);assert.ok(gap>=-1e-9);close(gap*result.lambda[i],0,1e-8);
        });
        for(let k=0;k<3;k++)close(displacement[k]+result.increment[k],rows.reduce((v,r,i)=>v+r.jacobian[k]*result.lambda[i],0));
    }
});

test('global two-material rod has no duplicate hard length equation and retains exact rest',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',type:'straight',insertion:80},{id:'catheter',type:'straight',insertion:60}]}),before=captureSharedAxisNative(s);
    const r=finish(iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:4,globalChain:true}}));
    assert.equal(r.converged,true,JSON.stringify(r));assert.equal(r.xpbd.model,'global-chain-xpbd');
    s.positions.forEach((p,i)=>p.forEach((x,k)=>close(x,before.positions[i][k])));
});

test('global chain cancellation restores poses and histories',()=>{
    const s=createSharedAxisNative({tools:[{id:'catheter',type:'berenstein',insertion:60}]}),before=captureSharedAxisNative(s);
    s.velocities=s.positions.map(()=>[0,.2,0]);const velocities=s.velocities;
    const it=iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:8,globalChain:true}});assert.equal(it.next().done,false);it.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.velocities,velocities);assert.equal(s.dynamicStep,null);
});

test('classic XPBD releases the previously applied unilateral impulse without inventing a new force balance',()=>{
    const r=solveXpbdChainBlock([{dofs:[0],jacobian:[1],gap:.5,alpha:1e-6,lambda:.1,unilateral:true}], [1],[99],{balancePrimal:false});
    close(r.lambda[0],0);close(r.increment[0],-.1);
});

test('global bending dynamics matches the reference curved-rod potential without contact',()=>{
    const create=()=>createSharedAxisNative({tools:[{id:'catheter',type:'berenstein',insertion:60,shaftStiffness:2,tipStiffness:2}]});
    const a=create(),b=create();a.loads[a.layout.positions.at(-1)+1]=b.loads[b.layout.positions.at(-1)+1]=1;
    for(let i=0;i<20;i++) {
        assert.equal(finish(iterateSharedAxisTimeStep(a,1/60)).converged,true);
        const result=finish(iterateSharedAxisXpbd(b,1/60,{xpbd:{iterations:32,globalChain:true}}));
        assert.equal(result.converged,true,JSON.stringify(result));
        const error=Math.max(...a.positions.map((p,i)=>Math.hypot(...p.map((x,k)=>x-b.positions[i][k]))));
        assert.ok(error<1e-6,`Free rod discrepancy: ${error}`);
    }
});

test('iteration residual tracing is passive and reports the free-rod force balance separately from geometry',()=>{
    const create=()=>{
        const s=createSharedAxisNative({tools:[{id:'catheter',type:'berenstein',insertion:60,shaftStiffness:2,tipStiffness:2}]});
        s.loads[s.layout.positions.at(-1)+1]=1;return s;
    };
    const a=create(),b=create(),settings={iterations:32,globalChain:true,chainSurfaceFriction:true};
    const plain=finish(iterateSharedAxisXpbd(a,1/60,{xpbd:settings}));
    const traced=finish(iterateSharedAxisXpbd(b,1/60,{xpbd:{...settings,chainTrace:true}}));
    assert.equal(plain.converged,true);assert.equal(traced.converged,true);
    assert.deepEqual(a.positions,b.positions);assert.deepEqual(captureSharedAxisNative(a).frames,captureSharedAxisNative(b).frames);
    assert.equal(plain.chainIterations,undefined);assert.equal(traced.chainIterations.length,32);
    const last=traced.chainIterations.at(-1).residuals;
    assert.ok(last.maxPositionForceResidual<1e-6);assert.ok(last.maxFrameTorqueResidual<1e-6);
    assert.equal(last.equilibriumCertified,false);
});
