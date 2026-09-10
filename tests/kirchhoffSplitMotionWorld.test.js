import assert from 'node:assert/strict';
import test from 'node:test';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

// Run the same independent tests against a frozen implementation or root.
// No skip/expected-failure route: an absent or ignored mode must fail.
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

function executed(f) {
    const stats = f.world.getStats(), d = stats.jointMotion;
    assert.ok(d && typeof d === 'object', 'World must publish diagnostics from the executed split-motion path');
    assert.equal(d.mode, MODE); assert.equal(stats.coupledSolver, 'joint');
    assert.ok(f.calls.solve > 0 && f.calls.apply > 0, 'actual World solve/apply must run');
    assert.ok(d.physicalPasses > 0 && d.biasPasses > 0, 'both phases must actually execute');
    close(d.physicalDt, f.world.fixedDt, 1e-15, 'one physical dt');
    assert.equal(d.historyCommits, 1, 'accepted physical step commits history exactly once');
    assert.equal(d.reactionUnits, 'xpbd-multiplier');
    close(d.impulseScale, 1 / d.physicalDt, 1e-12, 'lambda to physical impulse conversion');
    assert.equal(d.certified, true, JSON.stringify(d));
    assert.equal(stats.coupledClosureConverged, true);
    assert.ok(Number.isFinite(d.physicalKKTResidualMm) && d.physicalKKTResidualMm <= f.world.coupledContainmentTolerance,
        `physical KKT ${d.physicalKKTResidualMm}`);
    assert.ok(Number.isFinite(d.physicalConeViolation) && d.physicalConeViolation <= 1e-9,
        `physical cone ${d.physicalConeViolation}`);
    assert.ok(Array.isArray(d.contacts), 'final phase must expose physical/bias reaction samples');
    return d;
}

function geometry(f) {
    for (const body of [f.wire, f.catheter]) for (let i = 0; i < body.segmentCount; i++) {
        const length = Math.hypot(...xyz.map(a => body[a][i + 1] - body[a][i]));
        close(length, body.restLength[i], POSITION, `${body.id} segment ${i} exact length`);
    }
    for (let i = 0; i < f.wire.count; i++) {
        if (f.wall) assert.ok(f.wire.y[i] + f.wire.nodeRadius[i] <= POSITION, 'analytic wall nonpenetration');
        if (f.nested) {
            assert.ok(Math.hypot(f.wire.y[i], f.wire.z[i]) <= .5 + POSITION, 'analytic cylindrical clearance');
            assert.ok(f.wire.x[i] >= f.catheter.x[0] && f.wire.x[i] <= f.catheter.x.at(-1), 'witness remains in straight lumen span');
        }
    }
}

function motion(body, expectedV, expectedW = [0, 0, 0]) {
    for (let i = 0; i < body.count; i++) axes.forEach((a, k) =>
        close(body['velocity' + a][i], expectedV[k], VELOCITY, `node ${i} v${a}`));
    for (let i = 0; i < body.segmentCount; i++) axes.forEach((a, k) =>
        close(body['angularVelocity' + a][i], expectedW[k], ANGULAR, `segment ${i} omega${a}`));
}

function kinetic(body) {
    let energy = 0;
    for (let i = 0; i < body.count; i++) if (body.inverseMass[i] > 0)
        energy += axes.reduce((sum, a) => sum + body['velocity' + a][i] ** 2, 0) / (2 * body.inverseMass[i]);
    for (let i = 0; i < body.segmentCount; i++) {
        const w = new Vector3(...axes.map(a => body['angularVelocity' + a][i])).applyQuaternion(frame(body, i).invert()).toArray();
        for (let a = 0; a < 3; a++) if (body['inverseInertia' + (a + 1)][i] > 0)
            energy += w[a] ** 2 / (2 * body['inverseInertia' + (a + 1)][i]);
    }
    return energy;
}

function reactions(d, kind) {
    const rows = d.contacts.filter(c => c.kind === kind);
    assert.ok(rows.length > 0, `fixture must expose ${kind} contacts`);
    for (const r of rows) {
        assert.ok(Number.isFinite(r.normalPhysical) && r.normalPhysical >= -LAMBDA);
        assert.ok(Number.isFinite(r.normalBias) && r.normalBias >= -LAMBDA);
        assert.equal(r.tangentPhysical.length, 2); assert.equal(r.mu.length, 2);
        let ellipse = 0;
        for (let a = 0; a < 2; a++) {
            assert.ok(Number.isFinite(r.mu[a]) && r.mu[a] >= 0 && Number.isFinite(r.tangentPhysical[a]));
            const radius = r.mu[a] * r.normalPhysical;
            if (radius === 0) close(r.tangentPhysical[a], 0, LAMBDA, 'zero physical cone radius');
            else ellipse += (r.tangentPhysical[a] / radius) ** 2;
        }
        assert.ok(ellipse <= (1 + 1e-9) ** 2, `physical-load ellipse ${ellipse}`);
    }
    return { rows, normal: rows.reduce((s, r) => s + r.normalPhysical, 0),
        bias: rows.reduce((s, r) => s + r.normalBias, 0),
        tangent: rows.reduce((s, r) => s + Math.hypot(...r.tangentPhysical), 0) };
}

test('initial affine-wall overlap is repaired without physical velocity, spin or load', () => {
    const f = fixture({ wall: true, y: -.25 });
    const initialY = f.wire.y.slice();
    f.world.stepFixed(); const d = executed(f); geometry(f);
    assert.ok(initialY.some((v, i) => Math.abs(v - f.wire.y[i]) > .2), 'large actual geometry repair');
    motion(f.wire, [0, 0, 0]);
    const r = reactions(d, 'wall'); assert.ok(r.bias > .01);
    close(r.normal, 0, LAMBDA, 'bias cannot create physical normal reaction');
    close(r.tangent, 0, LAMBDA, 'bias cannot create physical friction');
    assert.ok(kinetic(f.wire) <= 1e-9, 'no translation or material-spin kinetic energy from bias');
});

test('nested bias-only contact with positive friction preserves axial slip and has zero physical budget', () => {
    const f = fixture({ nested: true, y: .75, mu: .3 });
    f.wire.velocityX.fill(4);
    f.world.stepFixed(); const d = executed(f); geometry(f); motion(f.wire, [4, 0, 0]);
    const r = reactions(d, 'lumen'); assert.ok(r.bias > .01);
    assert.ok(r.rows.every(c => c.mu.every(mu => mu === .3)), 'exercise a positive configured friction coefficient');
    close(r.normal, 0, LAMBDA, 'bias-only normal physical load');
    close(r.tangent, 0, LAMBDA, 'bias-only tangential physical load');
    close(kinetic(f.wire), 24, .01, 'axial kinetic energy retained');
});

for (const input of ['incoming velocity', 'external force']) test(`nested ${input} creates physical load and a bounded friction impulse`, () => {
    const f = fixture({ nested: true, y: .5, mu: .3 });
    f.wire.velocityX.fill(4);
    if (input === 'incoming velocity') f.wire.velocityY.fill(1);
    else f.wire.forceY.fill(120); // m=1, dt=1/120: predictor vy=1 in either case.
    f.world.stepFixed(); const d = executed(f); geometry(f);
    const r = reactions(d, 'lumen');
    assert.ok(r.normal > 1e-4 && r.tangent > 1e-5, 'both physical normal and friction reactions must occur');
    const py = Array.from(f.wire.velocityY).reduce((s, v, i) => s + v / f.wire.inverseMass[i], 0);
    close(r.normal * d.impulseScale, 3 - py, .002, 'normal impulse independently matches wire momentum change');
    const px = Array.from(f.wire.velocityX).reduce((s, v, i) => s + v / f.wire.inverseMass[i], 0);
    assert.ok(px < 12 - .01, 'physical axial slip experiences nonzero friction');
    assert.ok(12 - px <= r.tangent * d.impulseScale + .002, 'axial impulse cannot exceed summed physical surface friction');
    assert.ok(r.tangent <= .3 * r.normal + LAMBDA, 'only physical normal load supplies the friction budget');
    assert.ok(kinetic(f.wire) <= 25.5 + .002, 'zero-restitution contact cannot create kinetic energy');
});

test('wall bias preserves incoming away/tangent motion and independent wire-only axial spin', () => {
    const f = fixture({ wall: true, y: -.25 });
    const before = Array.from({ length: f.wire.segmentCount }, (_, i) => frame(f.wire, i));
    f.wire.velocityX.fill(3); f.wire.velocityY.fill(-2); f.wire.angularVelocityX.fill(2);
    f.world.stepFixed(); const d = executed(f); geometry(f);
    motion(f.wire, [3, -2, 0], [2, 0, 0]); motion(f.catheter, [0, 0, 0]);
    for (let i = 0; i < f.wire.segmentCount; i++)
        close(frame(f.wire, i).angleTo(spin(2 * DT).multiply(before[i])), 0, 2e-7, 'physical wire-only frame spin');
    const r = reactions(d, 'wall'); assert.ok(r.bias > .01); close(r.normal, 0, LAMBDA, 'separating body has no physical wall load');
});

test('free torsional prestrain produces analytic physical recoil and dissipative total energy', () => {
    const f = fixture(), theta = .2, compliance = f.wire.kirchhoffTwistCompliance[1];
    writeFrame(f.wire, 1, spin(theta).multiply(frame(f.wire, 1)));
    f.wire.copyCurrentToPrevious();
    const initialEnergy = theta * theta / (2 * compliance);
    // Two equal free inertias I=1 and one C-compliant torsional spring.
    // Backward Euler: delta = theta*dt^2/(C+2*dt^2), omega=delta/dt.
    const delta = theta * DT * DT / (compliance + 2 * DT * DT), expectedOmega = delta / DT;
    f.world.stepFixed(); executed(f); geometry(f);
    close(f.wire.angularVelocityX[0], expectedOmega, ANGULAR, 'first segment recoil');
    close(f.wire.angularVelocityX[1], -expectedOmega, ANGULAR, 'opposite segment recoil');
    const remaining = frame(f.wire, 0).angleTo(frame(f.wire, 1));
    close(remaining, theta - 2 * delta, 2e-7, 'remaining torsional prestrain');
    const kineticEnergy = kinetic(f.wire), finalEnergy = remaining * remaining / (2 * compliance) + kineticEnergy;
    assert.ok(kineticEnergy > 1e-4, 'physical elastic recoil cannot be classified as bias');
    assert.ok(finalEnergy <= initialEnergy + 2e-6, 'unforced recoil cannot create total energy');
    close(f.wire.angularVelocityX[0] + f.wire.angularVelocityX[1], 0, ANGULAR, 'internal torque balances');
});

test('hard positional and orientation controls remain exact while physical history commits once per dt', () => {
    const f = fixture({ wall: true, y: -.5 }), initial = frame(f.wire, 0);
    f.wire.kirchhoffTwistCompliance.fill(0);
    const pinned = xyz.map(a => f.catheter[a].slice());
    for (let step = 1; step <= 2; step++) {
        for (let i = 0; i < f.wire.count; i++) f.wire.setControlTarget(i, i - 1 + step * 2 * DT, -.5, 0, 0);
        const target = spin(step * DT).multiply(initial.clone());
        f.wire.setProximalOrientationControl(target.x, target.y, target.z, target.w, 0);
        f.world.stepFixed(); executed(f); geometry(f);
        for (let i = 0; i < f.wire.count; i++) {
            close(f.wire.x[i], i - 1 + step * 2 * DT, POSITION, 'hard axial control');
            close(f.wire.y[i], -.5, POSITION, 'hard wall-tangent support');
            close(f.wire.velocityX[i], 2, VELOCITY, 'control movement is physical');
        }
        close(frame(f.wire, 0).angleTo(target), 0, 2e-7, 'hard material-frame control');
        close(f.wire.angularVelocityX[0], 1, ANGULAR, 'controlled spin advances over one dt');
        xyz.forEach((a, i) => assert.deepEqual(f.catheter[a], pinned[i], 'supported catheter is unchanged'));
        assert.equal(f.world.stepCount, step);
    }
});

test('a rejected physical trial rolls back and commits the accepted physical history once', () => {
    function run(reject) {
        const f = fixture();
        writeFrame(f.wire, 1, spin(.2).multiply(frame(f.wire, 1))); f.wire.copyCurrentToPrevious();
        let rejected = 0, measuredCandidate = 0;
        f.world.debugJointTrial = (c, state, pass, trial) => {
            if (state.motionPhase !== 'physical') return;
            // Keep the first converged candidate open for one additional real
            // World iteration so the existing pass>=1 rollback path is tested.
            if (pass === 0) { state.settled = false; state.merit = 1; }
            if (reject && pass === 1 && trial === 0) {
                measuredCandidate++; state.settled = false; state.merit = Infinity; rejected++;
            }
        };
        f.world.stepFixed(); const d = executed(f); geometry(f);
        if (reject) {
            assert.equal(rejected, 1); assert.equal(measuredCandidate, 1);
            assert.ok(d.rejectedTrials >= 1 && d.rollbackCount >= 1, 'actual rejection and restoration must occur');
        } else { assert.equal(d.rejectedTrials, 0); assert.equal(d.rollbackCount, 0); }
        return f;
    }
    const clean = run(false), retried = run(true);
    for (const name of ['wire', 'catheter']) for (const key of [
        ...xyz, ...axes.map(a => 'velocity' + a), ...axes.map(a => 'angularVelocity' + a),
        ...['X', 'Y', 'Z', 'W'].map(a => 'orientation' + a),
        ...axes.map(a => 'previous' + a), ...['X', 'Y', 'Z', 'W'].map(a => 'previousOrientation' + a)]) {
        const tolerance = key.startsWith('angularVelocity') ? ANGULAR : key.startsWith('velocity') ? VELOCITY : POSITION;
        clean[name][key].forEach((v, i) => close(retried[name][key][i], v, tolerance, `${name}.${key}[${i}] after retry`));
    }
    close(kinetic(retried.wire), kinetic(clean.wire), 2e-6, 'retry cannot add a second physical impulse');
});
