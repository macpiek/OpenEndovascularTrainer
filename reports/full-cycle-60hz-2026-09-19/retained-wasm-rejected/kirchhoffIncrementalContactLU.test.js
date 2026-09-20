import test from 'node:test';
import assert from 'node:assert/strict';
import {createCoulombBandLU} from '../src/physics/kirchhoffCoulombBandLU.js';
import {createIncrementalContactLU} from '../src/physics/kirchhoffIncrementalContactLU.js';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function fixture(n=24,kl=3,ku=2) {
    const starts=Int32Array.from({length:n},(_,i)=>Math.max(0,i-kl)),ends=Int32Array.from({length:n},(_,i)=>Math.min(n-1,i+ku));
    let entries=0;const offsets=Int32Array.from(starts,(v,i)=>{const o=entries-v;entries+=ends[i]-v+1;return o;});
    const layout={starts,ends,offsets,entries,kl,ku},J=new Float64Array(entries);
    for(let i=0;i<n;i++)for(let j=starts[i];j<=ends[i];j++)J[offsets[i]+j]=i===j?.15:Math.sin(i*7+j*5)+.1;
    return {n,layout,J,F:Float64Array.from({length:n},(_,i)=>Math.cos(i)),scales:Float64Array.from({length:n},(_,i)=>.4+i/n)};
}
function compare(f,solver) {
    const a=new Float64Array(f.n),b=a.slice(),ref=createCoulombBandLU(f.layout,f.n);
    assert.equal(ref.solve(f.J,f.F,f.scales,0,b),true);assert.equal(solver.solve(f.J,f.F,f.scales,0,a),true);
    for(let i=0;i<f.n;i++)assert.ok(Math.abs(a[i]-b[i])<1e-8*Math.max(1,Math.abs(b[i])),`${i}: ${a[i]} / ${b[i]}`);
}
test('retained pivoted nonsymmetric LU handles new RHS and added/removed row updates',()=>{
    const f=fixture(),solver=createIncrementalContactLU(f.layout,f.n),base=f.J.slice();
    compare(f,solver);
    for(let k=0;k<20;k++) {
        f.F[k%f.n]+=.1;f.scales.fill(.7+k/10);compare(f,solver);
        const r=k%4;f.J[f.layout.offsets[r]+r]+=.05;compare(f,solver);
    }
    f.J.set(base);compare(f,solver);
    assert.equal(solver.diagnostics.factorizations,1);assert.ok(solver.diagnostics.updates>0);assert.ok(solver.diagnostics.reuses>0);
});
test('rank budget refactors, singular updates reject without overwriting output, then recover',()=>{
    const f=fixture(10,1,1),solver=createIncrementalContactLU(f.layout,f.n,{maxRank:1});compare(f,solver);
    f.J[f.layout.offsets[0]]+=.1;f.J[f.layout.offsets[1]+1]+=.1;compare(f,solver);
    assert.equal(solver.diagnostics.rankResets,1);
    for(let j=f.layout.starts[3];j<=f.layout.ends[3];j++)f.J[f.layout.offsets[3]+j]=0;
    const output=new Float64Array(f.n).fill(123);
    assert.equal(solver.solve(f.J,f.F,f.scales,0,output),false);assert.ok(output.every(x=>x===123));
    f.J[f.layout.offsets[3]+3]=1;compare(f,solver);assert.ok(solver.diagnostics.updateFailures>0);
});
test('WASM retained packing and certification preserve updates, changed scaling, and original residuals',()=>{
    const f=fixture(35,4,3),js=createIncrementalContactLU(f.layout,f.n),wasm=createIncrementalContactLU(f.layout,f.n,{wasmAssembly:true});
    const a=new Float64Array(f.n),b=a.slice(),error=a.slice();
    try {
        for(let step=0;step<24;step++) {
            f.F[step%f.n]+=.017;f.scales.fill(.3+step/10);
            if(step%3===0)f.J[f.layout.offsets[step%5]+step%5]+=.05;
            assert.equal(wasm.solve(f.J,f.F,f.scales,0,b),js.solve(f.J,f.F,f.scales,0,a));
            assert.deepEqual(b,a);
            const x=Float64Array.from(b,(v,i)=>v*f.scales[i]);
            const measured=wasm.measureOriginalResidual(x,error,f.F);
            let maximum=0;
            for(let i=0;i<f.n;i++) {
                let expected=f.F[i];for(let j=f.layout.starts[i];j<=f.layout.ends[i];j++)expected+=f.J[f.layout.offsets[i]+j]*x[j];
                assert.equal(error[i],expected);maximum=Math.max(maximum,Math.abs(expected));
            }
            assert.equal(measured,maximum);
            assert.deepEqual(wasm.diagnostics,js.diagnostics);
        }
        const saved=b.slice();f.J.fill(0);
        assert.equal(wasm.solve(f.J,f.F,f.scales,0,b),false);assert.deepEqual(b,saved);
    } finally {js.dispose();wasm.dispose();}
});
test('active-set updates retain unilateral certificates and discard factors after cancellation',()=>{
    const n=12,layout={dofCount:n,band:1},chain={layout,hessian:new Float64Array(n).fill(2)},rows=Array.from({length:n},(_,i)=>({id:i,kind:'wall',dofs:[i],jacobian:[1],gap:-1-i/10,multiplier:0}));
    const options={rows,gradient:new Float64Array(n),fixed:new Uint8Array(n),batchActivation:false},w=createSharedAxisLinear(layout,rows,{lazy:true});
    const a=solveSharedAxisLinear(w,chain,{...options,incrementalContacts:true}),b=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,options);
    assert.ok(a.converged);assert.ok(a.factorizations<b.factorizations);assert.equal(a.activeSetAttempts,b.activeSetAttempts);
    for(const key of ['increment','multiplierIncrement'])for(let i=0;i<a[key].length;i++)assert.ok(Math.abs(a[key][i]-b[key][i])<1e-10);
    const iterator=iterateSharedAxisLinear(w,chain,{...options,incrementalContacts:true});iterator.next();iterator.next();iterator.return();
    chain.hessian.fill(3);
    const c=solveSharedAxisLinear(w,chain,{...options,incrementalContacts:true});assert.ok(c.converged);assert.ok(c.incrementalStats[0].factorizations>0);
    for(let i=0;i<n;i++)assert.ok(Math.abs(c.multiplierIncrement[i]-3*(1+i/10))<1e-9);
});

test('bordered updates match general active sets with nonsymmetric force columns and physical reaction offsets',()=>{
    for(let seed=1;seed<=40;seed++) {
        let random=seed;const rng=()=>{random=(Math.imul(random,1664525)+1013904223)>>>0;return random/2**32;};
        const n=6,layout={dofCount:n,band:n},dofs=Array.from({length:n},(_,i)=>i),target=dofs.map(()=>rng()*2-1);
        const rows=Array.from({length:14},(_,id)=>{
            const jacobian=dofs.map(()=>rng()*2-1);
            return {id,kind:'wall',dofs,jacobian,multiplier:rng()*.1,gap:-jacobian.reduce((v,x,i)=>v+x*target[i],0)+rng()*.2,
                extraForceDofs:[id%n],extraForceJacobian:[.001]};
        });
        const hessian=new Float64Array(n*n);for(let i=0;i<n;i++)hessian[i*n]=2;
        const chain={layout,hessian},options={rows,gradient:Float64Array.from(dofs,()=>rng()),fixed:new Uint8Array(n)};
        const before=structuredClone({chain,options});
        const reference=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,options);
        const actual=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{...options,incrementalContacts:true});
        assert.equal(actual.converged,reference.converged,`seed ${seed}`);
        assert.equal(actual.failure,reference.failure,`seed ${seed}`);
        if(actual.converged) {
            assert.ok(actual.residual<=1e-8);
            for(const k of ['increment','multiplierIncrement'])for(let i=0;i<actual[k].length;i++)assert.ok(Math.abs(actual[k][i]-reference[k][i])<1e-6,`seed ${seed}/${k}/${i}`);
        }
        assert.deepEqual({chain,options},before);
    }
});

test('retained leases are isolated from other solvers and invalid after release',()=>{
    const f=fixture(),a=createIncrementalContactLU(f.layout,f.n),b=createIncrementalContactLU(f.layout,f.n);
    compare(f,a);f.J[f.layout.offsets[1]+1]+=.05;compare(f,b);compare(f,a);
    b.dispose();compare(f,a);a.dispose();
    assert.throws(()=>a.solve(f.J,f.F,f.scales,0,new Float64Array(f.n)),/Released/);
    const c=createIncrementalContactLU(f.layout,f.n);compare(f,c);c.dispose();
});

test('ROLLOUT GATE: captured pigtail withdrawal retains strict state parity',
    {skip:process.env.OET_INCREMENTAL_ROLLOUT!=='1'&&'Experimental path is disabled: strict Pigtail parity has not passed; set OET_INCREMENTAL_ROLLOUT=1 to run the gate'},async()=>{
    const {readFileSync}=await import('node:fs');
    const {loadCoupledRuntimeAnatomy}=await import('./helpers/coupledRuntimeFixture.js');
    const {restoreSharedAxisReplay,captureSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
    const {advanceSharedAxis}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
    const fixture=JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-pigtail-wire-withdraw-200.27-incoming.json',import.meta.url),'utf8'));
    const anatomy=await loadCoupledRuntimeAnatomy(),outputs=[];
    try {
        for(const incrementalContacts of [false,true]) {
            const req=fixture.stepRequest,s=restoreSharedAxisReplay(fixture,anatomy.field);
            const it=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,incrementalContacts});
            let next;do{next=it.next();}while(!next.done);
            assert.ok(next.value.state);assert.ok(next.value.result.converged);assert.ok(next.value.result.friction.converged);
            assert.ok(next.value.result.quality.maxPenetration<=1e-5);
            for(const body of next.value.result.quality.bodies){assert.ok(body.maxLengthError<=1e-5);assert.ok(body.maxBendAngleDegrees<=45+1e-6);}
            outputs.push(captureSharedAxisReplay(next.value.state,fixture.sheath));
        }
        const numbers=v=>Array.isArray(v)?v.flatMap(numbers):typeof v==='object'&&v!==null?Object.values(v).flatMap(numbers):typeof v==='number'?[v]:[];
        for(const key of ['positions','frames','velocities']) {
            const a=numbers(outputs[0][key]),b=numbers(outputs[1][key]);assert.equal(a.length,b.length);
            for(let i=0;i<a.length;i++)assert.ok(Math.abs(a[i]-b[i])<1e-7,`${key}/${i}`);
        }
    }finally{anatomy.dispose();}
});
