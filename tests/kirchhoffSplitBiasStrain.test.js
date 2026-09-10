import test from 'node:test';
import assert from 'node:assert/strict';
import {fixture,axes,frame,source,close,POSITION,VELOCITY,ANGULAR} from './fixtures/splitMotionAnalyticWorld.js';
const {configureKirchhoffSplitBias}=await import(source('src/physics/kirchhoffSplitMotion.js'));
const modelKeys=['restLength','restRotation1','restRotation2','restRotation3',
    'kirchhoffBendCompliance1','kirchhoffBendCompliance2','kirchhoffTwistCompliance'];

function tilted(materialMode) {
    const f=fixture({wall:true,y:-.5});
    configureKirchhoffSplitBias(f.constraint,{materialMode});
    for(let i=0;i<f.wire.count;i++)f.wire.setNodePosition(i,i-1,-.5+.2*(i-1),0);
    f.wire.captureRestConfiguration();f.wire.copyCurrentToPrevious();
    const saved=modelKeys.map(k=>f.wire[k].slice());
    f.world.stepFixed();
    modelKeys.forEach((k,i)=>assert.deepEqual(f.wire[k],saved[i],k+' must retain the physical model'));
    return f;
}

test('hard strain-preserving bias repairs a tilted rest rod without elastic or kinetic bias',()=>{
    const compliant=tilted('physical-compliance'),hard=tilted('preserve-strain');
    const old=compliant.world.getStats().jointMotion,d=hard.world.getStats().jointMotion;
    assert.equal(old.certified,false);assert.ok(old.biasElasticEnergyDelta>.1);
    assert.equal(d.biasMaterialMode,'preserve-strain');assert.equal(d.certified,true,JSON.stringify(d));
    assert.equal(d.historyCommits,1);assert.equal(d.physicalAccepted,true);assert.equal(d.biasAccepted,true);
    assert.ok(d.finalMaterialResidual.adaptationMm<=hard.world.coupledContainmentTolerance);
    assert.ok(d.finalMaterialResidual.bendTwistRad<=hard.world.coupledAngularToleranceRad);
    close(d.biasElasticEnergyDelta,0,1e-12,'no artificial elastic energy');
    // Independent kinematics: a strain-preserving initially straight rod has
    // equal adjacent material frames and unchanged geometric segment lengths.
    close(frame(hard.wire,0).angleTo(frame(hard.wire,1)),0,1e-7,'no relative frame bend/twist');
    for(let i=0;i<hard.wire.count;i++)assert.ok(-hard.wire.y[i]-hard.wire.nodeRadius[i]>=-POSITION,'raw plane gap');
    for(let i=0;i<hard.wire.segmentCount;i++) {
        const length=Math.hypot(...['x','y','z'].map(a=>hard.wire[a][i+1]-hard.wire[a][i]));
        close(length,hard.wire.restLength[i],POSITION,'geometric rest length');
    }
    for(const a of axes) {
        assert.deepEqual(Array.from(hard.wire['velocity'+a]),old.candidateMotion[0]['velocity'+a],'bias cannot change candidate physical velocity');
        assert.deepEqual(Array.from(hard.wire['angularVelocity'+a]),old.candidateMotion[0]['angularVelocity'+a],'bias cannot change candidate physical spin');
        for(const v of hard.wire['velocity'+a])close(v,0,VELOCITY,'initially stationary translation');
        for(const w of hard.wire['angularVelocity'+a])close(w,0,ANGULAR,'initially stationary spin');
    }
    assert.ok(d.contacts.some(c=>c.normalBias>0),'actual wall repair must carry bias load');
    for(const c of d.contacts) {
        close(c.normalPhysical,0,1e-10,'bias cannot create physical normal force');
        for(const t of c.tangentPhysical)close(t,0,1e-10,'bias cannot create physical friction');
    }
});

test('an immobile rod penetrating the plane cannot certify an infeasible strain-preserving bias',()=>{
    const f=fixture({wall:true,y:-.25});configureKirchhoffSplitBias(f.constraint,{materialMode:'preserve-strain'});
    f.wire.inverseMass.fill(0);
    for(let a=1;a<=3;a++)f.wire['inverseInertia'+a].fill(0);
    f.world.stepFixed();
    const d=f.world.getStats().jointMotion;
    assert.ok(f.wire.y.every(y=>-y-.5<-.2),'fixture must remain geometrically infeasible');
    assert.equal(d.biasAccepted,false);assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
    assert.equal(d.biasFailure.linear.status,'immovable-constraints');
    assert.equal(d.biasFailure.linear.converged,false);
    assert.equal(f.constraint._jointLinearFailure,undefined,'failed candidate diagnostics are separate from restored mechanics');
    for(const a of axes)assert.ok(f.wire['velocity'+a].every(v=>v===0));
});

function observeBiasSolves(penetration) {
    const f = fixture({ wall: true, y: -.5 + penetration });
    configureKirchhoffSplitBias(f.constraint, { materialMode: 'preserve-strain' });
    const solve = f.world.coupledSystem.solve, phases = [];
    f.world.coupledSystem.solve = (joint, ...args) => {
        phases.push(joint._splitMotion.phase);
        return solve(joint, ...args);
    };
    return { ...f, phases };
}

test('bias within the original geometry tolerance certifies without a linear solve or artificial motion', () => {
    const f = observeBiasSolves(.0005), before = f.wire.y.slice();
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true);
    assert.deepEqual(f.phases, ['physical']);
    assert.equal(result.diagnostics.biasInitialStateSettled, true);
    assert.ok(result.diagnostics.biasInitialMerit > .4 && result.diagnostics.biasInitialMerit < 1);
    assert.deepEqual(f.wire.y, before);
    assert.ok(f.wire.y.every(y => -y - .5 >= -f.world.coupledContainmentTolerance));
    assert.equal(result.diagnostics.biasElasticEnergyDelta, 0);
    assert.equal(result.diagnostics.historyCommits, 1);
    for (const axis of axes) assert.ok(f.wire['velocity' + axis].every(v => v === 0));
});

test('bias outside the original geometry tolerance still solves and repairs the raw gap', () => {
    const f = observeBiasSolves(.002);
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true);
    assert.equal(result.diagnostics.biasInitialStateSettled, false);
    assert.ok(result.diagnostics.biasInitialMerit > 1);
    assert.ok(f.phases.includes('bias'));
    for (const y of f.wire.y) close(-y - .5, 0, POSITION, 'repaired raw gap');
    for (const axis of axes) assert.ok(f.wire['velocity' + axis].every(v => v === 0));
});

test('an initially settled bias cannot override a rejected physical phase', () => {
    const f = observeBiasSolves(.0005), before = f.wire.y.slice();
    f.world.coupledClosureMaxPasses = 1;
    f.world.debugJointTrial = (_joint, state) => {
        if (state.motionPhase === 'physical') state.settled = false;
    };
    const result = f.world.stepFixed();
    assert.equal(result.accepted, false);
    assert.equal(result.diagnostics.physicalAccepted, false);
    assert.equal(result.diagnostics.biasInitialStateSettled, true);
    assert.equal(result.diagnostics.biasAccepted, true);
    assert.equal(result.diagnostics.historyCommits, 0);
    assert.equal(f.world.stepCount, 0);
    assert.deepEqual(f.wire.y, before);
    assert.ok(f.phases.every(phase => phase === 'physical'));
});
