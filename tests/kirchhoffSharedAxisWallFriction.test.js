import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {sharedAxisFrictionPotential,sharedAxisFrictionNormalDerivative,prepareSharedAxisWallFriction,assembleSharedAxisWallFriction,refreshSharedAxisWallFriction,commitSharedAxisWallFriction,captureSharedAxisWallFriction,restoreSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';

function fixture({overlap=false,feed=.002}={}) {
    const tools=[{id:'wire',insertion:20,wallStaticFriction:.5,wallKineticFriction:.3}];
    if(overlap)tools.push({id:'catheter',insertion:20,wallStaticFriction:.8,wallKineticFriction:.6});
    const s=createSharedAxisNative({tools}),e=2,t=.4;
    extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'wall-site',witness:{face:1,t},
        dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
        evaluate:()=>({gap:0,jacobian:[0,1-t,0,0,t,0]})}]);
    s.multipliers[s.multipliers.length-1]=100;prepareSharedAxisDynamicStep(s,1/120);
    prepareSharedAxisWallFriction(s,{feedById:{wire:feed,catheter:0}});return s;
}
function sample(s) {
    const half=s.layout.band-1;s.chain.gradient.fill(0);s.chain.tangent=new Float64Array(s.layout.dofCount*(2*half+1));
    return {energy:assembleSharedAxisWallFriction(s),gradient:s.chain.gradient.slice(),H:s.chain.tangent.slice()};
}

test('optional live normal force columns match multiplier finite differences including surface torque',()=>{
    for(const feed of [.002,.1]) {
        const s=fixture({feed});prepareSharedAxisWallFriction(s,{feedById:{wire:feed},liveNormalLoad:true});
        const delta=new Float64Array(s.layout.dofCount);delta[s.layout.spins.get('wire')[2]]=.0004;
        applySharedAxisNativeIncrement(s,delta,new Float64Array(s.multipliers.length));
        sample(s);const columns=s.wallFrictionStep.normalForceColumns,index=s.multipliers.length-1;
        const expected=new Float64Array(s.layout.dofCount);
        for(const c of columns){assert.equal(c.rowIndex,index);c.dofs.forEach((d,i)=>expected[d]+=c.values[i]);}
        const h=1e-4;s.multipliers[index]+=h;const plus=sample(s).gradient;
        s.multipliers[index]-=2*h;const minus=sample(s).gradient;s.multipliers[index]+=h;
        for(let i=0;i<expected.length;i++)assert.ok(Math.abs((plus[i]-minus[i])/(2*h)-expected[i])<1e-9);
        if(feed===.002)assert.equal(columns.length,0);else assert.ok(Math.abs(expected[s.layout.spins.get('wire')[2]])>0);
    }
});

test('normal derivative selects a valid radial return branch and a sliding one-sided derivative at zero',()=>{
    const law={stiffness:1000,normalLoad:10,muStatic:.5,muKinetic:.25,mode:'slide'},slip=[.01,0,0];
    assert.deepEqual(sharedAxisFrictionNormalDerivative(slip,law),[.25,0,0]);
    assert.deepEqual(sharedAxisFrictionNormalDerivative(slip,{...law,normalLoad:40}),[0,0,0]);
    assert.deepEqual(sharedAxisFrictionNormalDerivative(slip,{...law,normalLoad:50}),[0,0,0]);
    const h=1e-6,zero=sharedAxisFrictionPotential(slip,{...law,normalLoad:0}),plus=sharedAxisFrictionPotential(slip,{...law,normalLoad:h});
    assert.equal((plus.traction[0]-zero.traction[0])/h,sharedAxisFrictionNormalDerivative(slip,{...law,normalLoad:0})[0]);
    // Existing frozen stick is discontinuous when loaded elastic slip meets N=0.
    // Its zero formal column cannot certify that corner without the mode refresh.
    assert.equal(sharedAxisFrictionPotential(slip,{...law,mode:'stick',normalLoad:h}).traction[0],10);
    assert.deepEqual(sharedAxisFrictionNormalDerivative(slip,{...law,mode:'stick',normalLoad:0}),[0,0,0]);
});

test('live loads remove load-only outer changes but retain mode changes and new-contact certification',()=>{
    const s=fixture({feed:.1});prepareSharedAxisWallFriction(s,{feedById:{wire:.1},liveNormalLoad:true});
    const index=s.multipliers.length-1,first=sample(s);s.multipliers[index]=200;
    const next=sample(s);assert.equal(next.gradient[s.layout.positions[2]],2*first.gradient[s.layout.positions[2]]);
    // Live force and spatial tangent are exactly the original law evaluated at
    // the same current load, even though the prepared record stored N=100.
    s.wallFrictionStep.liveNormalLoad=false;s.wallFrictionStep.records[0].normalLoad=200;
    assert.deepEqual(sample(s),next);s.wallFrictionStep.liveNormalLoad=true;s.wallFrictionStep.records[0].normalLoad=100;
    assert.equal(refreshSharedAxisWallFriction(s).forceChange,0);
    s.multipliers[index]=0;prepareSharedAxisWallFriction(s,{feedById:{wire:.1},liveNormalLoad:true});
    assert.equal(s.wallFrictionStep.records.length,0);s.multipliers[index]=100;assert.equal(sample(s).energy,0);
    assert.ok(refreshSharedAxisWallFriction(s).forceChange>0);assert.equal(s.wallFrictionStep.records.length,1);
    prepareSharedAxisWallFriction(s,{feedById:{wire:.002},liveNormalLoad:true});assert.equal(s.wallFrictionStep.records[0].mode,'stick');
    s.multipliers[index]=10;sample(s);const update=refreshSharedAxisWallFriction(s);
    assert.ok(update.forceChange>0);assert.equal(s.wallFrictionStep.records[0].mode,'slide');
});

test('live friction remains exterior-only and replay/rollback reconstructs columns without stale trial output',()=>{
    const overlap=fixture({overlap:true,feed:5});prepareSharedAxisWallFriction(overlap,{feedById:{wire:5},liveNormalLoad:true});
    assert.equal(sample(overlap).energy,0);assert.equal(overlap.wallFrictionStep.normalForceColumns.length,0);
    assert.equal(overlap.wallFrictionStep.records[0].owner,'catheter');assert.equal(overlap.interToolRows,0);
    const s=fixture({feed:.1});prepareSharedAxisWallFriction(s,{feedById:{wire:.1},liveNormalLoad:true});
    const pose=captureSharedAxisNative(s),expected=sample(s),columns=structuredClone(s.wallFrictionStep.normalForceColumns),saved=captureSharedAxisWallFriction(s);
    s.multipliers[s.multipliers.length-1]=0;assert.equal(sample(s).energy,0);assert.equal(s.wallFrictionStep.normalForceColumns.length,1);
    s.multipliers[s.multipliers.length-1]=-1;assert.equal(sample(s).energy,0);assert.equal(s.wallFrictionStep.normalForceColumns.length,0);
    restoreSharedAxisNative(s,pose);restoreSharedAxisWallFriction(s,saved);assert.deepEqual(sample(s),expected);
    assert.deepEqual(s.wallFrictionStep.normalForceColumns,columns);
    assembleSharedAxisWallFriction(s,false);assert.deepEqual(s.wallFrictionStep.normalForceColumns,[]);
    assert.throws(()=>commitSharedAxisWallFriction(s),/converged/);
});

test('wall friction has a static elastic cone, bounded kinetic traction and no zero-load resistance',()=>{
    const law={stiffness:1000,normalLoad:10,muStatic:.5,muKinetic:.3};
    assert.equal(sharedAxisFrictionPotential([.004,0,0],law).traction[0],4);
    const kinetic=sharedAxisFrictionPotential([.4,0,0],{...law,mode:'slide'});assert.equal(kinetic.traction[0],3);
    assert.deepEqual(sharedAxisFrictionPotential([1,2,3],{...law,normalLoad:0}).traction,[0,0,0]);
    assert.throws(()=>sharedAxisFrictionPotential([0,0,0],{...law,muKinetic:.6}),RangeError);
});

test('feed friction belongs only to the exterior material; guidewire can slide freely inside a catheter',()=>{
    const wire=fixture({feed:.1});assert.equal(wire.wallFrictionStep.records[0].mode,'slide');
    assert.ok(sample(wire).energy>0);
    const overlap=fixture({overlap:true,feed:5});assert.equal(overlap.wallFrictionStep.records.length,1);
    assert.equal(overlap.wallFrictionStep.records[0].material.spec.id,'catheter');assert.equal(sample(overlap).energy,0);
    assert.equal(overlap.interToolRows,0);
});

test('wall surface friction includes spin and has the exact global gradient and tangent',()=>{
    for(const feed of [.002,.1]) {
    const s=fixture({feed}),zero=new Float64Array(s.multipliers.length),dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.0002*Math.sin(i));
    applySharedAxisNativeIncrement(s,dx,zero);
    const reference=sample(s),saved=captureSharedAxisNative(s),half=s.layout.band-1,width=2*half+1,h=1e-7;
    assert.ok(Math.abs(reference.gradient[s.layout.spins.get('wire')[2]])>0,'Surface rotation must create a friction torque');
    for(let col=0;col<dx.length;col++) {
        dx.fill(0);dx[col]=1;
        const side=sign=>{restoreSharedAxisNative(s,saved);applySharedAxisNativeIncrement(s,dx,zero,sign*h);return sample(s);};
        const a=side(1),b=side(-1);
        assert.ok(Math.abs((a.energy-b.energy)/(2*h)-reference.gradient[col])<1e-5,`gradient ${col}`);
        for(let row=0;row<dx.length;row++) {
            const expected=Math.abs(row-col)<=half?reference.H[row*width+col-row+half]:0,actual=(a.gradient[row]-b.gradient[row])/(2*h);
            assert.ok(Math.abs(actual-expected)<2e-5*Math.max(1,Math.abs(expected)),`H ${row}/${col}: ${actual} vs ${expected}`);
        }
    }
    }
});

test('normal-load changes require global relaxation and rejected trials cannot commit friction history',()=>{
    const s=fixture({feed:.1});assert.throws(()=>commitSharedAxisWallFriction(s),/converged/);
    assert.ok(refreshSharedAxisWallFriction(s).converged);
    s.multipliers[s.multipliers.length-1]=200;
    const update=refreshSharedAxisWallFriction(s);assert.ok(!update.converged);assert.ok(update.forceChange>0);
    assert.ok(refreshSharedAxisWallFriction(s).converged);
    s.positions[2][0]+=.001;assert.throws(()=>commitSharedAxisWallFriction(s),/converged/);
    assert.ok(refreshSharedAxisWallFriction(s).converged);commitSharedAxisWallFriction(s);
    assert.equal(s.wallFrictionHistory.length,1);assert.ok(Math.abs(Math.hypot(...s.wallFrictionHistory[0].elastic)-.006)<1e-12);
    assert.equal(s.wallFrictionStep,null);
});

test('global dynamic solve releases static wall sticking into kinetic sliding under independent feed',async()=>{
    const {relaxSharedAxisWithContacts}=await import('../src/physics/kirchhoffSharedAxisVesselWitnesses.js');
    const {completeSharedAxisDynamicStep}=await import('../src/physics/kirchhoffSharedAxisDynamics.js');
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20,wallStaticFriction:.5,wallKineticFriction:.3,
        configure:body=>{body.restRotation1.fill(0);body.restRotation2.fill(0);body.restRotation3.fill(0);}}]});
    const e=2;extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'feed-wall',witness:{face:2,t:1},
        dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
        evaluate:({b})=>({gap:b[1],jacobian:[0,0,0,0,1,0]})}]);
    s.loads[s.layout.positions[e+1]+1]=-100;
    const run=feed=>{
        prepareSharedAxisDynamicStep(s,1/120);prepareSharedAxisWallFriction(s,{feedById:{wire:feed}});
        let accepted=false,last,refresh;
        for(let outer=0;outer<8;outer++) {
            last=relaxSharedAxisWithContacts(s,{forceTolerance:5e-7});assert.ok(last.converged,JSON.stringify(last));
            refresh=refreshSharedAxisWallFriction(s,{forceTolerance:5e-7});if(refresh.converged){accepted=true;break;}
        }
        assert.ok(accepted,JSON.stringify({last,refresh}));
        const record=s.wallFrictionStep.records[0],normal=record.normalLoad,mode=record.mode;
        commitSharedAxisWallFriction(s);completeSharedAxisDynamicStep(s);
        return {normal,mode,traction:Math.hypot(...s.wallFrictionHistory[0].elastic)*1e4};
    };
    const stick=run(.0002);assert.equal(stick.mode,'stick');assert.ok(stick.traction<.5*stick.normal);
    const slide=run(.1);assert.equal(slide.mode,'slide');assert.ok(Math.abs(slide.traction-.3*slide.normal)<1e-8);
    assert.equal(s.interToolRows,0);
});


test('prepared wall friction replays exactly and gradient-only trials leave the Hessian untouched',()=>{
    const s=fixture({feed:.1}),expected=sample(s),saved=JSON.parse(JSON.stringify(captureSharedAxisWallFriction(s)));
    s.wallFrictionStep=null;restoreSharedAxisWallFriction(s,saved);
    assert.deepEqual(sample(s),expected);
    const tangent=s.chain.tangent;s.chain.gradient.fill(0);tangent.fill(17);
    const energy=assembleSharedAxisWallFriction(s,false);
    assert.equal(energy,expected.energy);assert.deepEqual(s.chain.gradient,expected.gradient);
    assert.ok(tangent.every(v=>v===17));
});

test('an identical vessel site retains its elastic history when only the containing edge ID changes',()=>{
    const s=fixture({feed:.001});refreshSharedAxisWallFriction(s);commitSharedAxisWallFriction(s);
    const elastic=s.wallFrictionHistory[0].elastic.slice();
    s.definitions.at(-1).id='remeshed-site';
    prepareSharedAxisWallFriction(s);assert.deepEqual(s.wallFrictionStep.records[0].elastic,elastic);
});

test('cancelling a prepared global step restores the incoming pose and accepted friction history',async()=>{
    const {iterateSharedAxisTimeStep}=await import('../src/physics/kirchhoffSharedAxisTimeStep.js');
    const s=fixture({feed:.1});s.dynamicStep=null;s.wallFrictionStep=null;
    s.wallFrictionHistory=[{id:'accepted',elastic:[.01,0,0],mode:'stick'}];
    const before=captureSharedAxisNative(s),history=s.wallFrictionHistory;
    const iterator=iterateSharedAxisTimeStep(s,1/120,{feedById:{wire:.01}});
    assert.equal(iterator.next().done,false);assert.equal(iterator.next().done,false);iterator.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.wallFrictionHistory,history);
    assert.equal(s.dynamicStep,null);assert.equal(s.wallFrictionStep,null);
});

test('reused friction tangent scratch scatters every contact and switches safely between exact and positive tangents',()=>{
    const s=fixture({feed:.01}),e=3,t=.7;
    extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'second-wall',witness:{face:3,t},
        dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
        evaluate:()=>({gap:0,jacobian:[0,0,1-t,0,0,t]})}]);
    s.multipliers[s.multipliers.length-1]=400;prepareSharedAxisWallFriction(s,{feedById:{wire:.01}});
    const records=s.wallFrictionStep.records;assert.equal(records.length,2);
    assert.deepEqual(records.map(r=>r.mode),['slide','stick']);
    const assemble=exact=>{
        s.chain.gradient.fill(0);s.chain.hessian.fill(0);
        s.chain.tangent=exact?new Float64Array(s.layout.dofCount*(2*s.layout.band-1)):null;
        const energy=assembleSharedAxisWallFriction(s);
        return {energy,g:s.chain.gradient.slice(),H:(s.chain.tangent??s.chain.hessian).slice()};
    };
    for(const exact of [true,false,true]) {
        s.wallFrictionStep.records=records;const whole=assemble(exact);
        const parts=records.map(r=>{s.wallFrictionStep.records=[r];return assemble(exact);});
        assert.equal(whole.energy,parts[0].energy+parts[1].energy);
        for(const key of ['g','H'])for(let i=0;i<whole[key].length;i++)
            assert.ok(Math.abs(whole[key][i]-parts[0][key][i]-parts[1][key][i])<1e-11);
    }
});
