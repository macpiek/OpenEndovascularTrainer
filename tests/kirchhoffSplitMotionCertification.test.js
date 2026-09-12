import assert from 'node:assert/strict';
import test from 'node:test';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

// Small native-World regression cases for split-motion certification.
// Fixture adapted from the independent 0827 analytic oracle.
const sourceRoot = resolve(process.env.OET_SPLIT_MOTION_SOURCE_ROOT ?? fileURLToPath(new URL('../', import.meta.url)));
const source = path => pathToFileURL(resolve(sourceRoot, path));
const { EndovascularPhysicsWorld } = await import(source('src/physics/endovascularPhysicsWorld.js'));
const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(source('src/physics/kirchhoffCoupledSystem.js'));
const { Quaternion, Vector3 } = createRequire(source('package.json'))('three');

const DT = 1 / 120, MODE = 'split-physical-bias';
// Float32 positions/velocities at coordinates of order 1 mm, reconstructed
// over 1/120 s. These are fixture precision bounds, not relaxed World gates.
const POSITION = 2e-6, VELOCITY = 5e-4, ANGULAR = 2e-5, LAMBDA = 1e-10;
const axes = ['X', 'Y', 'Z'], xyz = ['x', 'y', 'z'];
const close = (actual, expected, tolerance, message) => assert.ok(Number.isFinite(actual) &&
    Math.abs(actual - expected) <= tolerance, `${message}: ${actual} vs ${expected}, tolerance ${tolerance}`);
const frame = (body, i) => new Quaternion(...['X', 'Y', 'Z', 'W'].map(a => body['orientation' + a][i])).normalize();
const writeFrame = (body, i, q) => ['X', 'Y', 'Z', 'W'].forEach((a, j) => { body['orientation' + a][i] = q.toArray()[j]; });
const spin = angle => new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), angle);

// Analytic lumen y <= 0: finite segments attain their largest wall violation
// at an endpoint. No production collision/constraint helper supplies the oracle.
class AffineWall {
    voxelSize = .5;
    write(p, radius, out) {
        const gap = -p.y - radius, penetration = Math.max(0, -gap);
        Object.assign(out, { signedDistance: -p.y, signedGap: gap, penetration,
            inside: p.y <= 0, violation: gap < 0, branchId: 0, faceIndex: 0,
            source: 'split-motion-affine-oracle', timeOfImpact: gap < 0 ? 0 : 1 });
        Object.assign(out.point, p); Object.assign(out.closestPoint, { x: p.x, y: 0, z: p.z });
        Object.assign(out.inward, { x: 0, y: -1, z: 0 }); Object.assign(out.normal, out.inward);
        Object.assign(out.target, { x: p.x, y: p.y - penetration, z: p.z });
        return out;
    }
    querySphere(p, radius, out) { return this.write(p, radius, out); }
    queryCapsule(a, b, radius, out) {
        const p = a.y >= b.y ? a : b;
        this.write(p, radius, out); out.segmentT = p === a ? 0 : 1; return out;
    }
    sweepSphere(a, b, radius, out) {
        this.write(b, radius, out);
        const ga = -a.y - radius, gb = -b.y - radius;
        out.timeOfImpact = gb >= 0 ? 1 : ga <= 0 ? 0 : ga / (ga - gb);
        return out;
    }
}

const profile = { radius: .25, mass: 1, inverseAngularInertia: 1,
    adaptationCompliance: 0, kirchhoffBendCompliance: .02, kirchhoffTwistCompliance: .02,
    maxBendAngle: 179, foldLimitStrength: 0, linearDamping: 1, angularDamping: 1,
    projectionVelocityRetention: 0, wallProjectionVelocityRetention: 0, toolProjectionVelocityRetention: 0,
    wallStaticFriction: 0, wallKineticFriction: 0, wallCompliance: 0,
    sleepVelocity: 0, sleepAngularVelocity: 0, sleepFrames: 1e6, postStabilizationPasses: 0 };

function fixture({ wall = false, nested = false, y = 0, mu = 0, dt = DT } = {}) {
    const calls = { solve: 0, apply: 0 };
    const world = new EndovascularPhysicsWorld({ fixedDt: dt, jointMotionMode: MODE,
        contactField: wall ? new AffineWall() : null,
        coupledSystem: {
            solve(c, physicalDt, options) {
                calls.solve++;
                return solveKirchhoffCoupledSystem(c, physicalDt, { ...options,
                    activeCondensation: true, simultaneousCoulomb: true });
            },
            apply(c, result) { calls.apply++; applyKirchhoffCoupledCorrection(c, result); }
        } });
    const wire = world.createRod('proof-wire', 3, 1, { ...profile, radius: wall ? .5 : .25 });
    const catheter = world.createRod('proof-catheter', 5, 1, { ...profile, radius: 1 });
    for (let i = 0; i < wire.count; i++) wire.setNodePosition(i, i - 1, y, 0);
    for (let i = 0; i < catheter.count; i++) catheter.setNodePosition(i, i - 2, wall ? -4 : 0, 0);
    // An exact, static external support. Reaction/momentum checks below refer
    // to the free wire; the supported catheter may exchange momentum with it.
    catheter.inverseMass.fill(0);
    for (const key of ['inverseInertia1', 'inverseInertia2', 'inverseInertia3']) catheter[key].fill(0);
    const constraint = world.addContainment(wire, catheter, { innerRadius: nested ? .75 : 100,
        openDistal: false, openProximal: false, enforceDistalPortal: false,
        axialFriction: mu, torsionalFriction: mu, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    wire.copyCurrentToPrevious(); catheter.copyCurrentToPrevious();
    return { world, wire, catheter, constraint, calls, wall, nested };
}

const { beginKirchhoffSplitMotion, beginKirchhoffSplitBias, finishKirchhoffSplitBias,
    commitKirchhoffSplitHistory } = await import(source('src/physics/kirchhoffSplitMotion.js'));
const { assembleKirchhoffDirect } = await import(source('src/physics/kirchhoffDirectSolver.js'));

test('stationary wall bias does not become velocity on the next physical timestep', () => {
    const f = fixture({wall:true,y:-.25});
    for (let step=0;step<2;step++) {
        f.world.stepFixed();
        const d=f.world.getStats().jointMotion;
        assert.equal(d.certified,true,JSON.stringify(d));
        assert.equal(d.historyCommits,1);
        for(const a of axes) for(const v of f.wire['velocity'+a]) close(v,0,VELOCITY,'next-step physical velocity');
        for(const a of axes) for(const w of f.wire['angularVelocity'+a]) close(w,0,ANGULAR,'next-step physical spin');
        for(let i=0;i<f.wire.count;i++)close(f.wire.y[i],-.5,POSITION,'repaired wall pose');
        assert.ok(Math.abs(d.biasElasticEnergyDelta)<1e-9);
    }
});

test('nonuniform bias that changes elastic strain cannot reuse the old phase certificate', () => {
    const f=fixture({wall:true,y:-.5});
    for(let i=0;i<f.wire.count;i++)f.wire.setNodePosition(i,i-1,-.5+.2*(i-1),0);
    f.wire.captureRestConfiguration();f.wire.copyCurrentToPrevious();
    const before=f.wire.y.slice(),result=f.world.stepFixed();
    assert.equal(result.accepted,false);
    const d=f.world.getStats().jointMotion,m=assembleKirchhoffDirect(f.wire,DT);
    let actual=0;
    for(let i=0;i<m.rowCount;i+=6)actual=Math.max(actual,Math.hypot(...[0,1,2].map(k=>m.strain[i+k]+m.alpha[i+k]*m.lambda[i+k])));
    assert.ok(d.finalMaterialResidual.bendTwistRad>f.world.coupledAngularToleranceRad,'rejected candidate must have a real final material residual');
    assert.ok(actual<f.world.coupledAngularToleranceRad,'published state is restored before the rejected bias');
    assert.deepEqual(f.wire.y,before);
    assert.ok(d.biasElasticEnergyDelta>.1,'fixture must change elastic energy');
    assert.equal(d.certified,false);
    assert.equal(d.historyCommits,0);
    assert.equal(d.finalPhysicalResidualSettled,false);
    assert.ok(d.physicalPhaseMaterialResidual.bendTwistRad<f.world.coupledAngularToleranceRad);
});

test('a stable runtime id cannot restore physical load to new material labels during bias', () => {
    const f=fixture({nested:true,y:.5,mu:.3});f.world.stepFixed();
    const c=[...f.constraint.manifold.contacts()][0];assert.ok(c);
    c.normalLambda=2;c.tangentLambda[0]=.3;c.tangentLambda[1]=.1;
    const id=c.id,oldInner=c.innerMaterialSegmentId;
    beginKirchhoffSplitMotion(f.constraint,f.world);
    beginKirchhoffSplitBias(f.constraint,f.world,true);
    f.constraint.manifold.rekeyKnownContact(c,{id,
        innerMaterialSegmentId:String(oldInner)+':migrated',
        outerMaterialSegmentId:String(c.outerMaterialSegmentId)+':migrated',feature:c.feature});
    assert.equal(c.id,id);assert.notEqual(c.innerMaterialSegmentId,oldInner);
    finishKirchhoffSplitBias(f.constraint,true);
    assert.equal(c.normalLambda,0);assert.deepEqual([...c.tangentLambda],[0,0]);
    const s=f.constraint._splitMotion;
    assert.ok(s.diagnostics.unverifiedHistoryKinds.includes('contact-identity-changed-during-bias'));
    const zero={maximumDisplacementResidualMm:0,maximumConeViolation:0};
    const accepted=commitKirchhoffSplitHistory(f.constraint,f.world,{settled:true,
        materialResidual:{adaptationMm:0,bendTwistRad:0},frictionResidual:zero,externalFrictionResidual:zero});
    assert.equal(accepted,false,'even otherwise settled equations cannot certify lost impulse ownership');
    assert.equal(s.diagnostics.historyCommits,0);
});
