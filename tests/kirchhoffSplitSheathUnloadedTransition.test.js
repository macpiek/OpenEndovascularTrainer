import assert from 'node:assert/strict';
import test from 'node:test';
import {fixture,DT,source,axes,close,VELOCITY} from './fixtures/splitMotionAnalyticWorld.js';
const {configureKirchhoffSplitBias,beginKirchhoffSplitMotion,prepareKirchhoffSplitBoundaryRows,applyKirchhoffSplitPhysicalIncrement}=
    await import(source('src/physics/kirchhoffSplitMotion.js'));
const {collectKirchhoffCoupledBoundaryRows,applyKirchhoffCoupledBoundaryMultipliers}=await import(source('src/physics/kirchhoffCoupledBoundaryRows.js'));
const {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection}=await import(source('src/physics/kirchhoffCoupledSystem.js'));
const {captureKirchhoffCoupledTrialState,restoreKirchhoffCoupledTrialState}=await import(source('src/physics/kirchhoffCoupledTrialState.js'));
function setup({y=.45,end=5,extension=2}={}) {
    const f=fixture({y});configureKirchhoffSplitBias(f.constraint,{materialMode:'preserve-strain'});
    f.sheath=f.world.addSheath({start:{x:0,y:0,z:0},end:{x:end,y:0,z:0},innerRadius:.75,proximalExtension:extension,bodies:[f.wire]});
    return f;
}
function free(f,vx) {
    const d=f.world.getStats().jointMotion,h=f.constraint._splitMotion.sheathHistory.get(f.sheath);
    assert.equal(d.certified,true,JSON.stringify(d));assert.equal(d.historyCommits,1);assert.deepEqual(d.limitations,[]);
    assert.ok(d.sheathUnloadedTransitions.length>0);
    for(const t of d.sheathUnloadedTransitions)assert.ok(t.startRadialGap>0&&t.currentRadialGap>0&&t.reactionHistoryValid);
    for(const phase of ['physical','bias','legacy'])assert.ok(h.reactionHistory[phase][0].every(v=>v===0));
    for(const v of f.wire.velocityX)close(v,vx,VELOCITY,'unloaded axial velocity');
    for(const a of ['Y','Z'])for(const v of f.wire['velocity'+a])close(v,0,VELOCITY,'no invented radial impulse');
    for(const c of d.contacts.filter(c=>c.kind==='sheath'))assert.equal(c.normalPhysical+c.normalBias,0);
}
for(const [name,options,vx]of [
    ['distal entry',{end:.99},-4],['distal exit',{end:1.01},4],
    ['proximal entry',{extension:.99},4],['proximal exit',{extension:1.01},-4]
])test(`strictly clear unloaded sheath ${name} certifies without an impulse`,()=>{
    const f=setup(options);f.wire.velocityX.fill(vx);f.world.stepFixed();free(f,vx);
});

test('touching an end with zero reaction is outside the strictly-clear transition proof',()=>{
    const f=setup({y:.5,end:1.01});f.wire.velocityX.fill(4);f.world.stepFixed();
    const d=f.world.getStats().jointMotion;
    assert.ok(d.limitations.includes('sheath-axial-feature-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

for(const phase of ['physical','bias'])test(`positive endpoint gaps cannot erase a ${phase} reaction applied earlier in the dt`,()=>{
    const f=setup({end:1.01});let injected=false,exited=false,candidate;
    f.wire.debugConstraintPhase=stage=>{
        const s=f.constraint._splitMotion;
        if(stage==='closureEnd'&&s?.phase==='bias') {
            const h=s.sheathHistory.get(f.sheath);
            candidate={load:h.reactionHistory[phase][0][2],
                before:Math.max(0,h.innerRadius-h.nodeRadii[0][2])-Math.hypot(s.start[0].Y[2],s.start[0].Z[2]),
                now:.5-Math.hypot(f.wire.y[2],f.wire.z[2])};
        }
    };
    if(phase==='physical')f.wire.velocityY.fill(20);
    f.world.debugJointTrial=(c,state,pass)=>{
        if(phase==='bias'&&c._splitMotion.phase==='physical'&&pass===0&&!injected) {
            f.wire.y.fill(.75);injected=true;
        }
        if(c._splitMotion.phase===phase&&pass===0&&!exited) {
            f.wire.y.fill(.4);for(let i=0;i<f.wire.count;i++)f.wire.x[i]+=.1;exited=true;
        }
    };
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    assert.ok(exited);assert.ok(candidate.load>0,'actual apply hook records the load before the collector drops the row');
    assert.ok(candidate.before>0&&candidate.now>0);
    assert.ok(d.limitations.includes('sheath-axial-feature-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('missing reaction history cannot certify an otherwise clear unloaded transition',()=>{
    const f=setup({end:1.01});f.wire.velocityX.fill(4);let erased=false;
    f.world.debugJointTrial=c=>{if(!erased){delete c._splitMotion.sheathHistory.get(f.sheath).reactionHistory;erased=true;}};
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    assert.ok(d.limitations.includes('sheath-reaction-history-missing'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('relabelled material cannot use a zero journal to certify an unloaded crossing',()=>{
    const f=setup({end:1.01});f.wire.velocityX.fill(4);let changed=false;
    f.world.debugJointTrial=()=>{if(!changed){f.wire.materialCoordinate[2]+=.1;changed=true;}};
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    assert.ok(d.limitations.includes('sheath-material-support-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('a rejected actual scaled trial restores its sheath reaction journal with mechanics',()=>{
    const f=setup();f.wire.y.fill(.7);beginKirchhoffSplitMotion(f.constraint,f.world);
    const rows=collectKirchhoffCoupledBoundaryRows(f.constraint,[f.sheath],DT,f.world.contactActivation);
    prepareKirchhoffSplitBoundaryRows(f.constraint,rows);
    const result=solveKirchhoffCoupledSystem(f.constraint,DT,{additionalRows:rows,activeCondensation:true,simultaneousCoulomb:true});
    assert.equal(result.diagnostics.converged,true);result.scale*=.5;
    const s=f.constraint._splitMotion,h=s.sheathHistory.get(f.sheath),journal=h.reactionHistory.physical[0];
    const before={journal:journal.slice(),y:f.wire.y.slice(),v:s.bodies[0].velocityY.slice()};
    const snapshot=captureKirchhoffCoupledTrialState(f.constraint,{world:f.world,reusePropertyLayout:true,frozenFrictionBatches:true});
    applyKirchhoffSplitPhysicalIncrement(f.constraint,result);
    applyKirchhoffCoupledCorrection(f.constraint,result);
    applyKirchhoffCoupledBoundaryMultipliers(f.constraint,result.additionalIncrement,result.scale);
    assert.ok(journal.some(v=>v>0));
    for(const [i,row]of rows.entries())if(row.kind==='sheath')
        close(journal[row.node],Math.abs(result.scale*result.additionalIncrement[i]),1e-12,'journal uses actual common apply scale');
    restoreKirchhoffCoupledTrialState(snapshot);
    assert.equal(h.reactionHistory.physical[0],journal,'journal identity survives rollback');
    assert.deepEqual(journal,before.journal);assert.deepEqual(f.wire.y,before.y);assert.deepEqual(s.bodies[0].velocityY,before.v);
});
