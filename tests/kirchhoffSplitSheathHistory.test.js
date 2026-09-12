import assert from 'node:assert/strict';
import test,{after} from 'node:test';
import {writeFileSync} from 'node:fs';
import { fixture,DT,axes,source,close,POSITION,VELOCITY,ANGULAR } from './fixtures/splitMotionAnalyticWorld.js';
const {configureKirchhoffSplitBias,beginKirchhoffSplitMotion,prepareKirchhoffSplitBoundaryRows}=
    await import(source('src/physics/kirchhoffSplitMotion.js'));
const {collectKirchhoffCoupledBoundaryRows}=await import(source('src/physics/kirchhoffCoupledBoundaryRows.js'));
const observations=[];
after(()=>{if(process.env.OET_SHEATH_HISTORY_REPORT)writeFileSync(process.env.OET_SHEATH_HISTORY_REPORT,JSON.stringify(observations,null,2));});
function record(name,f) {
    const sh=f.sheath,axis=[sh.axisX,sh.axisY,sh.axisZ],origin=[sh.startX,sh.startY,sh.startZ];
    observations.push({name,diagnostics:f.world.getStats().jointMotion,
        geometryState:'published after step transaction; rejected candidate pose is in diagnostics.candidatePose',
        rawGaps:[...f.wire.y].map((_,i)=>{
            const p=axes.map((a,k)=>f.wire[a.toLowerCase()][i]-origin[k]);
            const cross=[p[1]*axis[2]-p[2]*axis[1],p[2]*axis[0]-p[0]*axis[2],p[0]*axis[1]-p[1]*axis[0]];
            return Math.max(0,sh.innerRadius-f.wire.nodeRadius[i])-Math.hypot(...cross)/Math.hypot(...axis);
        }),
        velocity:axes.map(a=>[...f.wire['velocity'+a]]),omega:axes.map(a=>[...f.wire['angularVelocity'+a]]),
        history:(f.constraint._coupledBoundaries?.rows??[]).filter(r=>r.kind==='sheath').map(r=>({node:r.node,
            actualGap:r._splitActualStrain,startGap:r._splitStartGap,startAxial:r._splitSheathStartAxial}))});
}

function sheathFixture({y=.5,end=5}={}) {
    const f=fixture({y});configureKirchhoffSplitBias(f.constraint,{materialMode:'preserve-strain'});
    f.sheath=f.world.addSheath({start:{x:0,y:0,z:0},end:{x:end,y:0,z:0},innerRadius:.75,
        proximalExtension:2,bodies:[f.wire]});
    return f;
}
function check(f,expectedV) {
    const d=f.world.getStats().jointMotion;
    assert.equal(d.certified,true,JSON.stringify(d));assert.equal(d.historyCommits,1);
    assert.equal(d.physicalAccepted,true);assert.equal(d.biasAccepted,true);assert.deepEqual(d.limitations,[]);
    assert.ok(d.physicalKKTResidualMm<=f.world.coupledContainmentTolerance);
    assert.ok(d.maximumOutwardContactVelocity*DT<=f.world.coupledContainmentTolerance);
    assert.ok(d.finalMaterialResidual.adaptationMm<=f.world.coupledContainmentTolerance);
    assert.ok(d.finalMaterialResidual.bendTwistRad<=f.world.coupledAngularToleranceRad);
    for(let i=0;i<f.wire.count;i++) {
        assert.ok(.5-Math.hypot(f.wire.y[i],f.wire.z[i])>=-POSITION,'independent raw cylindrical gap');
        for(let a=0;a<3;a++)close(f.wire['velocity'+axes[a]][i],expectedV[a],VELOCITY,'physical velocity');
    }
    for(const a of axes)for(const w of f.wire['angularVelocity'+a])close(w,0,ANGULAR,'no radial-force couple');
    close(d.biasElasticEnergyDelta,0,1e-10,'no bias elastic work');
    return d;
}
function loads(d) {
    const rows=d.contacts.filter(c=>c.kind==='sheath');
    assert.ok(rows.every(c=>c.normalPhysical>=-1e-10&&c.normalBias>=-1e-10));
    assert.ok(rows.every(c=>c.tangentPhysical.every(v=>v===0)),'existing sheath model has no tangential row');
    return {physical:rows.reduce((s,c)=>s+c.normalPhysical,0),bias:rows.reduce((s,c)=>s+c.normalBias,0)};
}

test('stable sheath interior bias repairs overlap without physical pressure or axial drag',()=>{
    const f=sheathFixture({y:.75});f.wire.velocityX.fill(4);f.world.stepFixed();
    const d=check(f,[4,0,0]),r=loads(d);
    record('bias-only',f);
    assert.ok(r.bias>.1);close(r.physical,0,1e-10,'bias-only pressure');
    for(const row of f.constraint._coupledBoundaries.rows.filter(r=>r.kind==='sheath')) {
        close(row._splitStartGap,-.25,1e-12,'exact initial radius clearance');
        assert.equal(row.sheathWitness.geometry.minimumAxial,-2.00001,'same proximal slab extension');
    }
});

test('stable sheath radial input produces the independent physical momentum impulse',()=>{
    const f=sheathFixture();f.wire.velocityX.fill(4);f.wire.velocityY.fill(1);f.world.stepFixed();
    const d=check(f,[4,0,0]),r=loads(d);assert.ok(r.physical>0);
    record('incoming-radial',f);
    const momentumLoss=f.wire.count-[...f.wire.velocityY].reduce((s,v)=>s+v,0);
    close(r.physical/DT,momentumLoss,VELOCITY,'sum(lambda)/dt = radial momentum loss');
    close(r.bias,0,1e-6,'true pressure is not moved into bias bank');
});

test('away and axial motion unload sheath pressure on the next physical step',()=>{
    const f=sheathFixture();f.wire.velocityY.fill(1);f.world.stepFixed();
    assert.ok(loads(check(f,[0,0,0])).physical>0,'first step carries true physical pressure');
    f.wire.velocityX.fill(3);f.wire.velocityY.fill(-2);f.world.stepFixed();
    const r=loads(check(f,[3,-2,0]));close(r.physical,0,1e-10,'away releases physical pressure');
    record('next-step-away',f);
    close(r.bias,0,1e-10,'away needs no pseudo pressure');
});

test('historical sheath gap uses exact distance to the frozen oblique axis rather than a tangent plane',()=>{
    const f=fixture();
    const origin=[2,-3,4],axis=[.6,.8,0],radial=[-.8,.6,0];
    const sh=f.world.addSheath({start:{x:2,y:-3,z:4},end:{x:8,y:5,z:4},innerRadius:.75,bodies:[f.wire]});
    for(let i=0;i<f.wire.count;i++)f.wire.setNodePosition(i,...origin.map((v,a)=>v+(i+2)*axis[a]+.3*radial[a]+(a===2?.4:0)));
    f.wire.copyCurrentToPrevious();
    for(let i=0;i<f.wire.count;i++)for(let a=0;a<3;a++)
        f.wire[axes[a].toLowerCase()][i]=origin[a]+(i+2)*axis[a]+.4*radial[a]+(a===2?.3:0);
    beginKirchhoffSplitMotion(f.constraint,f.world);
    const rows=collectKirchhoffCoupledBoundaryRows(f.constraint,[sh],DT);
    const oldPlane=rows.map(r=>r.strain+r.gradients.reduce((sum,g)=>{
        const node=Math.floor(g.dof/6),a=g.dof%6;
        return sum+g.value*(f.wire['previous'+axes[a]][node]-f.wire[axes[a].toLowerCase()][node]);
    },0));
    prepareKirchhoffSplitBoundaryRows(f.constraint,rows);assert.equal(rows.length,f.wire.count);
    for(const [index,row]of rows.entries()) {
        const p=axes.map((a,i)=>f.wire['previous'+a][row.node]-origin[i]);
        const cross=[p[1]*axis[2]-p[2]*axis[1],p[2]*axis[0]-p[0]*axis[2],p[0]*axis[1]-p[1]*axis[0]];
        const exact=.5-Math.hypot(...cross)/Math.hypot(...axis);
        close(row._splitStartGap,exact,1e-12,'independent cross-product distance');
        assert.ok(Math.abs(oldPlane[index]-exact)>.019,'fixture exposes tangent-plane history error');
    }
    assert.deepEqual(f.constraint._splitMotion.diagnostics.unverifiedHistoryKinds,[]);
    observations.push({name:'oblique-history',exactStartGap:rows.map(r=>r._splitStartGap),oldTangentPlane:oldPlane});
});

test('an entering distal-end feature is explicitly unsupported despite zero geometric penetration',()=>{
    const f=sheathFixture({end:.99});f.wire.velocityX.fill(-4);f.world.stepFixed();
    const d=f.world.getStats().jointMotion;
    record('distal-entry-unsupported',f);
    assert.ok(d.candidatePose[0].x.at(-1)<.99,'rejected candidate tip crosses into sheath slab');
    assert.equal(f.wire.x.at(-1),1,'transaction restores the pre-step tip');
    assert.ok(d.limitations.includes('sheath-axial-feature-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('a loaded row disappearing beyond the distal end cannot masquerade as a certified release',()=>{
    const f=sheathFixture({end:1.01});f.wire.velocityY.fill(1);let moved=false,loaded=0;
    f.world.debugJointTrial=(c,state,pass)=>{
        if(c._splitMotion.phase==='physical'&&pass===0&&!moved) {
            loaded=c._coupledBoundaries.sheaths.get(f.sheath)[0].lambda.at(-1);
            for(let i=0;i<f.wire.count;i++)f.wire.x[i]+=.1;
            moved=true;
        }
    };
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    record('loaded-distal-exit-unsupported',f);
    assert.ok(loaded>0,'tip had a physical reaction before its row disappeared');
    assert.ok(d.candidatePose[0].x.at(-1)>1.01,'candidate leaves the supported slab');
    assert.ok(!d.contacts.some(c=>c.id==='sheath:0:0:2'),'candidate has no retained reaction at the disappeared row');
    assert.equal(f.wire.x.at(-1),1,'published pose is restored instead of committing the unsupported release');
    assert.ok(d.limitations.includes('sheath-axial-feature-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('changing sheath geometry during the physical step invalidates the frozen history',()=>{
    const f=sheathFixture();let changed=false,originalOrigin;
    f.world.debugJointTrial=c=>{
        if(c._splitMotion.phase==='physical'&&!changed){originalOrigin=c._splitMotion.sheathHistory.get(f.sheath).origin[1];f.sheath.startY=.05;changed=true;}
    };
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    record('geometry-change-unsupported',f);
    assert.equal(originalOrigin,0,'candidate used the immutable initial origin');
    assert.equal(f.sheath.startY,0,'transaction restores the original sheath geometry');
    assert.ok(d.limitations.includes('sheath-geometry-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});

test('changing material labels at the same runtime node invalidates sheath history',()=>{
    const f=sheathFixture();let changed=false;
    f.world.debugJointTrial=c=>{
        if(c._splitMotion.phase==='physical'&&!changed){f.wire.materialCoordinate[2]+=.1;changed=true;}
    };
    f.world.stepFixed();const d=f.world.getStats().jointMotion;
    record('material-change-unsupported',f);
    assert.ok(d.limitations.includes('sheath-material-support-changed'));
    assert.equal(d.certified,false);assert.equal(d.historyCommits,0);
});
