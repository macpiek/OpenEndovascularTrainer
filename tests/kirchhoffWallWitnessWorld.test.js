import test from 'node:test';
import assert from 'node:assert/strict';
import { BufferGeometry, Float32BufferAttribute } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';

// Two finite orthogonal triangles. The query exposes only the nearer wall;
// the common contact provider must retain the other loaded witness itself.
class CornerField {
    voxelSize=.5;
    constructor() {
        this.fallbackGeometry=new BufferGeometry();
        this.fallbackGeometry.setAttribute('position',new Float32BufferAttribute([
            -20,-20,0, 20,-20,0, 0,20,0,
            -20,0,-20, 0,0,20, 20,0,-20
        ],3));
    }
    write(p,r,out) {
        const face=p.z<=p.y?0:1, distance=face===0?p.z:p.y, gap=distance-r;
        const n=face===0?{x:0,y:0,z:1}:{x:0,y:1,z:0};
        Object.assign(out,{signedDistance:distance,signedGap:gap,penetration:Math.max(0,-gap),inside:distance>=0,
            violation:gap<0,branchId:face,faceIndex:face,source:'sparse-sdf-bvh',timeOfImpact:gap<0?0:1});
        Object.assign(out.point,p);Object.assign(out.closestPoint,{x:p.x,y:face===1?0:p.y,z:face===0?0:p.z});
        Object.assign(out.inward,n);Object.assign(out.normal,n);
        Object.assign(out.target,{x:p.x,y:p.y+n.y*Math.max(0,-gap),z:p.z+n.z*Math.max(0,-gap)});
        return out;
    }
    queryCapsuleSoA(x,y,z,radii,index,out,face,inside,clearance,branch,near,length,samples,physical,finite){
        assert.equal(physical,true,'witness queries and refreshes need physical distance');
        assert.equal(finite,true,'contact refresh must preserve finite mesh identity');
        return this.queryCapsule({x:x[index],y:y[index],z:z[index]},
            {x:x[index+1],y:y[index+1],z:z[index+1]},Math.max(radii[index],radii[index+1]),out);
    }
    querySphere(p,r,out){return this.write(p,r,out);}
    queryCapsule(a,b,r,out){const p=Math.min(a.y,a.z)<=Math.min(b.y,b.z)?a:b;
        this.write(p,r,out);out.segmentT=p===a?0:1;return out;}
    sweepSphere(a,b,r,out){return this.write(b,r,out);}
}

for (const physicalTrialState of [false, true]) {
test('one common rod solve retains simultaneous finite wall witnesses at a corner' + (physicalTrialState ? ' (physical snapshot)' : ' (complete snapshot)'),()=>{
    let maximumRows=0, sawRelease=false;
    const world=new EndovascularPhysicsWorld({contactField:new CornerField(),coupledSystem:{
        independentComponents:true,wallWitnesses:true,physicalTrialState,earlyTrialRejection:true,
        solve(c,dt,options){
            maximumRows=Math.max(maximumRows,options.additionalRows.filter(r=>r.kind==='wall-witness').length);
            sawRelease ||= options.additionalRows.some(r=>r.kind==='wall-release');
            return solveKirchhoffCoupledSystem(c,dt,{...options,activeCondensation:true});
        },apply:applyKirchhoffCoupledCorrection
    }});
    const body=world.createRod('corner-catheter',3,2,{radius:.5,wallFriction:0,sleepFrames:1e6});
    for(let n=0;n<body.count;n++)body.setNodePosition(n,2*n,.49,.49);
    world.stepFixed();
    assert.equal(world.lastJointLineSearch.earlyRejections,0,'witness discovery requires complete measurements');
    assert.ok(maximumRows>body.segmentCount,'both walls must have independent rows on the same rod');
    assert.ok(world.lastCoupledClosureConverged,JSON.stringify(world.lastJointNonlinearFailure));
    for(let n=0;n<body.count-1;n++) {
        assert.ok(body.y[n]>=.5-world.coupledContainmentTolerance,`side wall ${n}`);
        assert.ok(body.z[n]>=.5-world.coupledContainmentTolerance,`floor ${n}`);
    }
    assert.equal(sawRelease,false,'both static finite walls remain valid during this solve');
});

for (const legacyPositionStorage of [false, true]) test('world solves catheter wall slip and radius torque in the global normal/friction block' + (physicalTrialState ? ' (physical snapshot)' : ' (complete snapshot)') + (legacyPositionStorage ? ' Float32 discovery regression' : ' Float64'),()=>{
    let maximumGroups=0, maximumTorque=0, discoveredAtRestoredBase=0;
    const seen=new Set();
    const world=new EndovascularPhysicsWorld({contactField:new CornerField(),coupledSystem:{
        independentComponents:true,wallWitnesses:true,physicalTrialState,earlyTrialRejection:true,
        solve(c,dt,options){
            for(const w of c._wallWitnessRows.witnesses)if(w.discoveredForStep&&!seen.has(w)){
                seen.add(w);discoveredAtRestoredBase++;
                assert.equal(w.ledger.lambda,0,'rejected trial normal force must not survive');
                assert.deepEqual(Array.from(w.ledger.tangentLambda),[0,0]);
                assert.equal(w.ledger.wrenches.length,0,'rejected pose must not transfer any reaction');
            }
            const friction=options.additionalRows.filter(r=>r.kind==='wall-witness-friction');
            maximumGroups=Math.max(maximumGroups,options.groups.filter(g=>g.normalRow?.kind==='wall-witness').length);
            for(const row of friction)for(const gradient of row.gradients)
                if(gradient.dof%6>=3)maximumTorque=Math.max(maximumTorque,Math.abs(gradient.value));
            return solveKirchhoffCoupledSystem(c,dt,{...options,activeCondensation:true,simultaneousCoulomb:true});
        },apply:applyKirchhoffCoupledCorrection
    }});
    const body=world.createRod('sliding-catheter',3,2,{radius:.5,wallFriction:.2,sleepFrames:1e6});
    // Preserve the captured quantization-triggered discovery/rollback case
    // alongside the production precision, which converges without that retry.
    if (legacyPositionStorage) for (const key of ['x','y','z','previousX','previousY','previousZ','wallX','wallY','wallZ','wallNormalX','wallNormalY','wallNormalZ','wallT','wallGap'])
        body[key] = Float32Array.from(body[key]);
    for(let n=0;n<body.count;n++){
        body.setNodePosition(n,2*n,.49,.49);
        body.velocityX[n]=.1;
    }
    world.stepFixed();
    assert.equal(world.lastJointLineSearch.earlyRejections,0,'witness discovery requires complete measurements');
    if (legacyPositionStorage) assert.ok(discoveredAtRestoredBase>0,'new contact geometry must survive rollback to be solved at the restored base');
    assert.ok(maximumGroups>0,'wall normal and friction must share Coulomb groups');
    assert.ok(maximumTorque>0,'surface friction must include radius torque');
    assert.equal(body._wallWitnessFrictionSolved,true,'post-step friction must not apply the same load again');
    assert.ok(world.lastCoupledClosureConverged,JSON.stringify(world.lastJointNonlinearFailure));
});

test('world retries converged static sliding from the predicted state with kinetic friction' + (physicalTrialState ? ' (physical snapshot)' : ' (complete snapshot)'),()=>{
    let maximumGroups=0, maximumTorque=0, discoveredAtRestoredBase=0, component;
    const seen=new Set(),attemptBases=new Map();
    const world=new EndovascularPhysicsWorld({contactField:new CornerField(),coupledSystem:{
        independentComponents:true,wallWitnesses:true,physicalTrialState,earlyTrialRejection:true,
        solve(c,dt,options){
            component=c;
            const attempt=c._wallWitnessFrictionModes.attempt;
            if(!attemptBases.has(attempt)) {
                const b=c.bodies[0];
                const pose=Object.fromEntries(['x','y','z','previousX','previousY','previousZ',
                    'velocityX','velocityY','velocityZ','orientationX','orientationY','orientationZ','orientationW']
                    .map(key=>[key,Array.from(b[key])]));
                if(attempt>1)assert.deepEqual(pose,attemptBases.get(1),'kinetic retry must start at the same predicted pose and history');
                attemptBases.set(attempt,pose);
                for(const w of c._wallWitnessRows.witnesses) {
                    assert.equal(w.ledger.lambda,0);
                    assert.equal(w.ledger.wrenches.length,0);
                }
            }
            for(const w of c._wallWitnessRows.witnesses)if(w.discoveredForStep&&!seen.has(w)){
                seen.add(w);discoveredAtRestoredBase++;
                assert.equal(w.ledger.lambda,0,'rejected trial normal force must not survive');
                assert.deepEqual(Array.from(w.ledger.tangentLambda),[0,0]);
                assert.equal(w.ledger.wrenches.length,0,'rejected pose must not transfer any reaction');
            }
            const friction=options.additionalRows.filter(r=>r.kind==='wall-witness-friction');
            maximumGroups=Math.max(maximumGroups,options.groups.filter(g=>g.normalRow?.kind==='wall-witness').length);
            for(const row of friction)for(const gradient of row.gradients)
                if(gradient.dof%6>=3)maximumTorque=Math.max(maximumTorque,Math.abs(gradient.value));
            return solveKirchhoffCoupledSystem(c,dt,{...options,activeCondensation:true,simultaneousCoulomb:true});
        },apply:applyKirchhoffCoupledCorrection
    }});
    const body=world.createRod('sliding-catheter',3,2,{radius:.5,wallFriction:.002,wallStaticFriction:.006,wallKineticFriction:.002,sleepFrames:1e6});
    for(let n=0;n<body.count;n++){
        body.setNodePosition(n,2*n,.49,.49);
        body.velocityX[n]=10;
    }
    world.stepFixed();
    assert.equal(world.lastJointLineSearch.earlyRejections,0,'witness discovery requires complete measurements');

    assert.ok(maximumGroups>0,'wall normal and friction must share Coulomb groups');
    assert.ok(maximumTorque>0,'surface friction must include radius torque');
    assert.equal(body._wallWitnessFrictionSolved,true,'post-step friction must not apply the same load again');
    assert.ok(world.lastCoupledClosureConverged,JSON.stringify({failure:world.lastJointNonlinearFailure,attempts:component._wallWitnessFrictionAttempts}));
    assert.ok(component._wallWitnessFrictionAttempts.some(a=>a.status==='restart'));
    assert.equal(component._wallWitnessFrictionAttempts.at(-1).status,'accepted');
    assert.ok(attemptBases.size>=2);
    assert.equal(world.stepCount,1,'kinetic retries must not advance simulated time');
    assert.equal(body.wallStaticFriction,.006);
    assert.equal(body.wallKineticFriction,.002);
});

}
