import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {stepSharedAxis,iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {captureSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function rod(overlap=false) {
    const tools=[{id:'wire',insertion:20,wallStaticFriction:.2,wallKineticFriction:.1}];
    if(overlap)tools.push({id:'catheter',insertion:20,wallStaticFriction:.2,wallKineticFriction:.1});
    const s=createSharedAxisNative({tools});
    for(let e=1;e+1<s.positions.length;e++) {
        extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:`floor/${e}`,witness:{face:0,t:1},
            dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(p=>[p,p+1,p+2]),
            evaluate:({b})=>({gap:b[1],jacobian:[0,0,0,0,1,0]})}]);
        s.multipliers[s.multipliers.length-1]=100;s.loads[s.layout.positions[e+1]+1]=-20;
    }
    return s;
}
const options={liveWallNormalLoad:true,promoteTrialAssembly:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true};
const close=(a,b,tolerance)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<=tolerance,`${i}: ${v} != ${b[i]}`));
const chart=s=>JSON.stringify(s.wallFrictionStep.records.map(r=>({id:r.id,n:r.n,mode:r.mode,normalLoad:r.normalLoad,elastic:r.elastic})));

test('coupled friction updates only between accepted Newton steps and retains the full final force certificate',()=>{
    for(const overlap of [false,true]) {
        const reference=rod(overlap),actual=rod(overlap),feedById={wire:.1,...(overlap?{catheter:.1}:{})};
        const a=stepSharedAxis(reference,1/60,{...options,feedById});let directions=0,refreshes=0,frozen;
        const b=stepSharedAxis(actual,1/60,{...options,feedById,coupledFrictionNewton:true,observeTrial:e=>{
            if(e.kind==='direction'){frozen=chart(e.state);directions++;}
            if(e.kind==='trial')assert.equal(chart(e.state),frozen,'a line search must use one fixed friction chart');
            if(e.kind==='friction-refresh')refreshes++;
        }});
        assert.ok(a.converged);assert.ok(b.converged,JSON.stringify(b));assert.ok(directions>0&&refreshes>0);
        assert.equal(b.coupledFrictionRefreshes,refreshes);
        assert.ok(b.certificateBound<=1e-6);assert.ok(b.residual.length<=1e-5);assert.equal(b.interToolRows,0);
        close(actual.positions.flat(),reference.positions.flat(),1e-5);
        assert.ok(actual.wallFrictionHistory.length>0);assert.equal(actual.wallFrictionStep,null);assert.equal(actual.dynamicStep,null);
    }
});

test('cancelling after an early friction refresh restores pose, velocities and committed history',()=>{
    const s=rod(),before=captureSharedAxisNative(s),friction=captureSharedAxisWallFriction(s),velocities=s.velocities;let refreshed=false;
    const iterator=iterateSharedAxisTimeStep(s,1/60,{...options,feedById:{wire:.1},coupledFrictionNewton:true,
        observeTrial:e=>{if(e.kind==='friction-refresh')refreshed=true;}});
    for(let i=0;i<500&&!refreshed;i++)assert.equal(iterator.next().done,false);
    assert.ok(refreshed);iterator.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.deepEqual(captureSharedAxisWallFriction(s),friction);
    assert.equal(s.velocities,velocities);assert.equal(s.dynamicStep,null);assert.equal(s.wallFrictionStep,null);
    assert.ok(stepSharedAxis(s,1/60,{...options,feedById:{wire:.1},coupledFrictionNewton:true}).converged);
});

test('a failed early update retries the reference method from the original physical state and counts failed work',()=>{
    const reference=rod(),actual=rod(),feedById={wire:.1};
    const a=stepSharedAxis(reference,1/60,{...options,feedById});let thrown=0;
    const b=stepSharedAxis(actual,1/60,{...options,feedById,coupledFrictionNewton:true,observeTrial:e=>{
        if(e.kind==='friction-refresh'){thrown++;throw new Error('injected early-friction failure');}
    }});
    assert.equal(thrown,1);assert.ok(b.converged);assert.equal(b.coupledFrictionRecovery.converged,true);
    assert.equal(b.coupledFrictionFallbacks,1);assert.ok(b.factorizations>a.factorizations);assert.ok(b.fullAssemblies>a.fullAssemblies);
    assert.ok(b.certificateBound<=1e-6);close(actual.positions.flat(),reference.positions.flat(),1e-10);
    assert.deepEqual(actual.wallFrictionHistory,reference.wallFrictionHistory);
});

test('simultaneously release zero-reaction blocking contacts without changing the certified linear solution',()=>{
    const layout={dofCount:3,band:3},chain={layout,hessian:Float64Array.from([1,0,0,1,0,0,1,0,0])};
    const rows=[[1,0,0],[1,1,0],[1,0,1]].map((jacobian,i)=>({id:`wall/${i}`,kind:'wall',dofs:[0,1,2],jacobian,gap:0,multiplier:0}));
    const common={rows,gradient:Float64Array.from([1,-.5,-.5]),fixed:new Uint8Array(3)};
    const a=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,common);
    const b=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{...common,batchRelease:true});
    assert.ok(a.converged&&b.converged);assert.ok(b.factorizations<a.factorizations,`${a.factorizations} -> ${b.factorizations}`);
    close(b.increment,a.increment,1e-10);close(b.multiplierIncrement,a.multiplierIncrement,1e-10);
    for(const r of rows)assert.ok(r.gap+r.jacobian.reduce((v,j,k)=>v+j*b.increment[k],0)>=-1e-8);
    assert.ok(b.multiplierIncrement.every(v=>v>=-1e-8));
});

function discoveryRod(blocking=false) {
    const sample=({state:s,edge,b})=>{
        const id=`discovered/${edge}`;
        if(edge===s.positions.length-2&&b[1]>1e-8&&!s.definitionIds.has(id)) {
            const sign=blocking?-1:1,limit=blocking?1e-5:1;
            const row={kind:'wall',edge,id,dofs:[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]),
                evaluate:({b})=>({gap:limit+sign*b[1],jacobian:[0,0,0,0,sign,0]})};
            (s.pendingVesselRows??=new Map()).set(id,row);
        }
        return {gap:1,jacobian:[0,0,0,0,0,0]};
    };
    const s=createSharedAxisNative({tools:[{id:'wire',type:'straight',insertion:25}],wallSamples:[sample]});
    s.loads[s.layout.positions.at(-1)+1]=100;return s;
}

test('a feasible newly discovered contact can keep an accepted trial without resolving its direction',()=>{
    const reference=discoveryRod(),actual=discoveryRod();
    const a=stepSharedAxis(reference,1/60,options);
    const b=stepSharedAxis(actual,1/60,{...options,coupledFrictionNewton:true});
    assert.ok(a.converged&&b.converged,JSON.stringify({a,b}));assert.ok(b.retainedDiscoveryTrials>0);
    assert.ok(b.factorizations<a.factorizations);assert.ok(b.iterations<a.iterations);
    close(actual.positions.flat(),reference.positions.flat(),1e-7);assert.ok(b.certificateBound<=1e-6);
});

test('a newly discovered violated contact always rebuilds the direction instead of retaining the trial',()=>{
    const s=discoveryRod(true),r=stepSharedAxis(s,1/60,{...options,coupledFrictionNewton:true});
    assert.ok(r.converged,JSON.stringify(r));assert.equal(r.retainedDiscoveryTrials,0);
    assert.ok(r.geometryRestarts>0);assert.ok(s.positions.at(-1)[1]<=1e-5+1e-8);
    assert.ok(r.certificateBound<=1e-6);assert.ok(r.residual.length<=1e-5);
});

test('a non-geometric failure before the first factorization still retries the original method',()=>{
    const s=rod();let calls=0;
    const result=stepSharedAxis(s,1/60,{...options,coupledFrictionNewton:true,feedById:{wire:.1},observeIteration:()=>{
        if(calls++===0)throw new Error('injected failure before any LU');
    }});
    assert.ok(result.converged,JSON.stringify(result));assert.equal(result.coupledFrictionFallbacks,1);
    assert.equal(result.coupledFrictionRecovery.experimentStatus,'unsupported-direction');assert.ok(result.certificateBound<=1e-6);
});
