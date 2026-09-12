import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { COUPLED_VALIDATION_LIMITS as limits, maximumRelativeLengthError } from './helpers/coupledValidationMetrics.js';
import { measureRodSystemMomentum, measureRodSystemCorrection } from './helpers/rodSystemMomentum.js';

// Always exercise the joint kernel explicitly. An external root permits an
// independent validation worktree without copying or modifying the engine.
const sourceRoot = process.env.OET_REALISM_SOURCE_ROOT
    ? pathToFileURL(resolve(process.env.OET_REALISM_SOURCE_ROOT) + '/') : new URL('../', import.meta.url);
const source = name => import(new URL(`src/physics/${name}.js`, sourceRoot));
const [{ EndovascularPhysicsWorld }, { applyKirchhoffMaterialProfile }, { evaluateBendTwistConstraint },
    { evaluateKirchhoffSlidingPortal }, { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection }] = await Promise.all([
    source('endovascularPhysicsWorld'), source('applyKirchhoffMaterialProfile'), source('discreteKirchhoffRod'),
    source('kirchhoffSlidingPortal'), source('kirchhoffCoupledSystem')
]);
const coupledSystem = { solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection };

const dt = 1 / 120;
const wireRadius = 0.889 / 2, lumenRadius = 0.97 / 2;
const clearance = lumenRadius - wireRadius;
const profile = { radius: wireRadius, innerRadius: lumenRadius,
    linearDamping: 1, angularDamping: 1, foldLimitStrength: 0,
    projectionVelocityRetention: 1, sleepFrames: 1e6 };
const sum = values => values.reduce((a, b) => a + b, 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => sum(a.map((v, i) => v * b[i]));
const frame = (body, i) => ({ x: body.orientationX[i], y: body.orientationY[i], z: body.orientationZ[i], w: body.orientationW[i] });

function pair({ offset = 0.02, velocity = 0, radialVelocity = 0, spin = 0, friction = 0,
    count = 2, spacing = 10, fixedDt = dt, kernel = coupledSystem } = {}) {
    const world = new EndovascularPhysicsWorld({ fixedDt, coupledSystem: kernel });
    const inner = world.createRod('wire', count, spacing, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', count, spacing, { ...profile, mass: 3 });
    for (let i = 0; i < count; i++) {
        inner.setNodePosition(i, i * spacing, offset, 0);
        inner.velocityX[i] = velocity;
        inner.velocityY[i] = radialVelocity;
    }
    inner.angularVelocityX.fill(spin);
    const contact = world.addContainment(inner, outer, {
        innerRadius: lumenRadius, axialFriction: friction, torsionalFriction: friction,
        portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0
    });
    return { world, inner, outer, contact };
}

test('finite-clearance normal reaction preserves center of mass with unequal masses', () => {
    const { world, inner, outer } = pair({ offset: 0.2 });
    const before = (sum([...inner.y]) + 3 * sum([...outer.y])) / 8;
    world.stepFixed();
    const after = (sum([...inner.y]) + 3 * sum([...outer.y])) / 8;
    assert.ok(Math.abs(after - before) < limits.reciprocalTranslationMm, `COM change ${after - before} mm`);
    assert.ok(inner.y[0] < 0.2 && outer.y[0] > 0, 'both rods participate');
    assert.ok(Math.abs((sum([...inner.y]) - sum([...outer.y])) / 2 - clearance) < 0.001);
});

for (const velocity of [-6, 6]) test(`free axial slide and twist at ${velocity} mm/s preserve independent tool motion`, () => {
    const { world, inner, outer, contact } = pair({ velocity, spin: 2, friction: 0.5, count: 9, spacing: 5 });
    const outerFrame = frame(outer, 0);
    for (let step = 0; step < 60; step++) world.stepFixed();
    for (let i = 0; i < inner.count; i++) {
        assert.ok(Math.abs(inner.x[i] - (i * 5 + velocity * 0.5)) < limits.freeSlideErrorMm);
        assert.ok(Math.abs(outer.x[i] - i * 5) < limits.freeSlideErrorMm);
    }
    for (let i = 0; i < outer.segmentCount; i++) {
        const q = frame(outer, i);
        assert.ok(1 - Math.abs(q.x * outerFrame.x + q.y * outerFrame.y + q.z * outerFrame.z + q.w * outerFrame.w) < limits.noLoadQuaternionDotError);
    }
    for (const record of contact.manifold.contacts()) {
        assert.equal(record.normalLambda, 0);
        assert.equal(Math.hypot(...record.tangentLambda), 0);
        assert.equal(record.twistLambda, 0);
    }
});

test('penetration repair activates bounded friction and must converge for both overlap depths', t => {
    const loaded = [];
    for (const offset of [0.02, 0.1, 0.2]) {
        const { world, inner, outer, contact } = pair({ offset, velocity: 6, spin: 2, friction: 0.2 });
        const before = measureRodSystemMomentum([inner, outer]);
        world.stepFixed();
        const after = measureRodSystemMomentum([inner, outer]);
        // These initial offsets already violate finite clearance. Repairing
        // their positions changes orbital L even for zero friction. They are
        // convergence stress cases, not admissible momentum initial states.
        t.diagnostic(JSON.stringify({ offset, before, after, deltaL: sub(after.total, before.total),
            converged: world.lastCoupledClosureConverged, passes: world.lastCoupledClosurePasses,
            frictionDisplacementResidualMm: contact._jointFrictionResidual?.maximumDisplacementResidualMm }));
        let normal = 0, tangent = 0;
        for (const record of contact.manifold.contacts()) {
            normal += record.normalLambda;
            tangent += Math.hypot(...record.tangentLambda);
            assert.ok(Math.hypot(...record.tangentLambda) <= 0.2 * record.normalLambda + 1e-10);
            assert.ok(Math.abs(record.twistLambda) <= 0.2 * wireRadius * record.normalLambda + 1e-9);
            assert.ok(Math.abs(record.innerTwistImpulse + record.outerTwistImpulse) < 1e-12);
        }
        const momentum = sum([...inner.velocityX]) + 3 * sum([...outer.velocityX]);
        assert.ok(Math.abs(momentum - 12) < 1e-3, `axial momentum ${momentum}`);
        const axialEnergy = 0.5 * sum([...inner.velocityX].map(v => v * v)) + 1.5 * sum([...outer.velocityX].map(v => v * v));
        assert.ok(axialEnergy <= 36 + 1e-3, `friction injected axial kinetic energy: ${axialEnergy}`);
        loaded.push({ normal, tangent });
        assertJointClosure(world, contact);
    }
    assert.equal(loaded[0].normal, 0);
    assert.equal(loaded[0].tangent, 0);
    assert.ok(loaded[1].normal > 0 && loaded[1].tangent > 0);
    assert.ok(loaded[2].normal > loaded[1].normal && loaded[2].tangent > loaded[1].tangent);
});

function assertJointClosure(world, contact) {
    assert.equal(world.lastCoupledSolver, 'joint', 'the intended kernel must actually run');
    assert.equal(world.lastCoupledClosureConverged, true, `joint closure exhausted ${world.lastCoupledClosurePasses} passes`);
    assert.ok(contact._jointFrictionResidual.maximumDisplacementResidualMm <= world.coupledContainmentTolerance);
    assert.ok(contact._jointFrictionResidual.maximumConeViolation <= 1e-9, 'final normal load must bound the complete friction cone');
    assert.ok(contact._jointMaterialResidual.adaptationMm <= world.coupledContainmentTolerance);
    assert.ok(contact._jointMaterialResidual.bendTwistRad <= world.coupledAngularToleranceRad);
    assert.ok(contact.kirchhoffMaxViolation <= world.coupledContainmentTolerance);
    assert.ok(contact.kirchhoffSolverResidual <= world.coupledContainmentTolerance);
    assert.ok(contact.kirchhoffContactMotion <= world.coupledContainmentTolerance);
    assert.ok(contact._jointBoundaryResidual <= world.coupledContainmentTolerance);
    for (const body of [contact.innerBody, contact.outerBody]) for (let i = body.activeStart; i < body.activeEnd; i++) {
        const length = Math.hypot(body.x[i + 1] - body.x[i], body.y[i + 1] - body.y[i], body.z[i + 1] - body.z[i]);
        assert.ok(Math.abs(length - body.restLength[i]) <= world.coupledLengthTolerance);
    }
}

test('complete momentum oracle rotates anisotropic material inertia and includes orbital momentum', () => {
    // Rz(pi/2), I_local=diag(2,3,5), omega_world=(1,2,3):
    // I_world=diag(3,2,5), L_spin=(3,4,15). Two mass-2 nodes
    // at (1,0,0),(2,0,0), velocity=(0,3,0) add L_orb=(0,0,18).
    const body = { activeStart: 0, activeEnd: 1, segmentCount: 1,
        x: [1, 2], y: [0, 0], z: [0, 0], inverseMass: [0.5, 0.5],
        velocityX: [0, 0], velocityY: [3, 3], velocityZ: [0, 0],
        orientationX: [0], orientationY: [0], orientationZ: [Math.SQRT1_2], orientationW: [Math.SQRT1_2],
        angularVelocityX: [1], angularVelocityY: [2], angularVelocityZ: [3],
        inverseInertia1: [0.5], inverseInertia2: [1 / 3], inverseInertia3: [0.2] };
    const measured = measureRodSystemMomentum([body]);
    assert.deepEqual(measured.linear, [0, 12, 0]);
    assert.deepEqual(measured.orbital, [0, 0, 18]);
    assert.ok(Math.hypot(...sub(measured.total, [3, 4, 33])) < 1e-12);
    assert.ok(Math.abs(measured.kineticEnergy - 46) < 1e-12);
    const shifted = measureRodSystemMomentum([body], { origin: [1, 0, 0] });
    assert.ok(Math.hypot(...sub(shifted.total, [3, 4, 21])) < 1e-12);
});

for (const friction of [0, 0.2]) test(`valid loaded contact: complete angular momentum has first-order timestep error at friction ${friction}`, t => {
    const results = [];
    for (const fixedDt of [2 * dt, dt, dt / 2, dt / 4]) {
        const increments = [];
        const kernel = { solve: solveKirchhoffCoupledSystem, apply(constraint, response) {
            const bodies = [constraint.innerBody, constraint.outerBody];
            const before = measureRodSystemMomentum(bodies, { reconstructDt: fixedDt });
            const wrench = measureRodSystemCorrection(bodies, [response.inner, response.outer], response.scale, fixedDt);
            // Preserve the strict instantaneous wrench criterion. A full-step
            // truncation allowance must not excuse a missing surface moment.
            assert.ok(Math.hypot(...wrench.linearImpulse) < 1e-10, `unbalanced correction force ${wrench.linearImpulse}`);
            assert.ok(Math.hypot(...wrench.momentImpulse) < 1e-8, `unbalanced correction moment ${wrench.momentImpulse}`);
            applyKirchhoffCoupledCorrection(constraint, response);
            const after = measureRodSystemMomentum(bodies, { reconstructDt: fixedDt });
            increments.push({ ...wrench, finiteSpin: sub(sub(after.spin, before.spin), wrench.angularImpulse.map(v => v / fixedDt)) });
        } };
        const { world, inner, outer, contact } = pair({ offset: clearance * (1 - 2 ** -23), velocity: 6,
            radialVelocity: 2, spin: 2, friction, fixedDt, kernel });
        // Round inside the admissible Float32 clearance. Contact develops
        // from outward velocity, not from a prescribed overlap.
        assert.ok(inner.y[0] - outer.y[0] <= clearance);
        const bodies = [inner, outer], before = measureRodSystemMomentum(bodies);
        let closure;
        outer.debugConstraintPhase = phase => {
            if (phase === 'closureEnd') closure = measureRodSystemMomentum(bodies, { reconstructDt: fixedDt });
        };
        world.stepFixed();
        assertJointClosure(world, contact);
        assert.ok(increments.length > 0);
        const records = [...contact.manifold.contacts()];
        const normalLoad = sum(records.map(record => record.normalLambda));
        const tangentLoad = sum(records.map(record => Math.hypot(...record.tangentLambda)));
        assert.ok(normalLoad > 1e-6, 'outward velocity must load the contact');
        if (friction > 0) assert.ok(tangentLoad > 1e-6, 'exercise nonzero sliding friction');
        else assert.equal(tangentLoad, 0);
        for (const record of records) assert.equal(record.twistLambda, 0, 'surface friction replaces the legacy independent twist pair');
        const after = measureRodSystemMomentum(bodies);
        const deltaL = sub(after.total, before.total), absoluteError = Math.hypot(...deltaL);
        const relativeError = absoluteError / Math.hypot(...before.total);
        const finiteOrbital = increments.reduce((v, entry) => add(v, entry.finiteOrbital), [0, 0, 0]);
        const finiteSpin = increments.reduce((v, entry) => add(v, entry.finiteSpin), [0, 0, 0]);
        const velocityStageDelta = sub(after.total, closure.total);
        results.push({ fixedDt, before, after, deltaL, absoluteError, relativeError,
            finiteOrbital, finiteSpin, velocityStageDelta,
            maximumIncrementMoment: Math.max(...increments.map(entry => Math.hypot(...entry.momentImpulse))),
            normalLoad, tangentLoad, coneViolation: contact._jointFrictionResidual.maximumConeViolation,
            frictionDisplacementResidualMm: contact._jointFrictionResidual.maximumDisplacementResidualMm,
            passes: world.lastCoupledClosurePasses });
        // Float32 storage at x~10 mm produces ~1e-4 mm/s reconstruction
        // roundoff. The existing linear-momentum and energy budgets remain.
        assert.ok(Math.hypot(...sub(after.linear, before.linear)) < 1e-3);
        assert.ok(after.kineticEnergy <= before.kineticEnergy + 1e-3);
        assert.ok(Math.hypot(...sub(after.spin, before.spin)) > 1e-6, 'contact must exchange spin with orbital motion');
    }
    t.diagnostic(JSON.stringify({ friction, results }));
    for (let i = 1; i < results.length; i++) {
        // The position/quaternion projection integrator is first order in
        // full angular momentum: dx x m v is O(dt) for a valid impact.
        // This asserts a measured convergence rate, not exact conservation
        // under a widened absolute tolerance. 0.95 allows Float32 roundoff
        // and higher-order terms around theoretical order 1; faster is fine.
        const order = Math.log2(results[i - 1].absoluteError / results[i].absoluteError);
        assert.ok(order >= 0.95, `full-vector momentum convergence order ${order}`);
    }
});

test('surface-contact oracle conserves the complete moment and satisfies power conjugacy', () => {
    // One physical contact point supplies r x F and v + omega x r on each
    // surface. Independent axial twist alone does not provide these moments.
    const centers = [[5, clearance, 0], [5, 0, 0]], point = [5, lumenRadius, 0];
    const force = [2, -3, 0.7], opposite = force.map(v => -v);
    const offsets = centers.map(center => sub(point, center));
    const moments = [cross(offsets[0], force), cross(offsets[1], opposite)];
    const totalMoment = add(add(cross(centers[0], force), moments[0]), add(cross(centers[1], opposite), moments[1]));
    assert.ok(Math.hypot(...totalMoment) < 1e-12);
    const velocity = [6, -0.1, 0.2], omega = [0.3, 0.2, 1];
    const surfaceVelocity = add(velocity, cross(omega, offsets[0]));
    assert.ok(Math.abs(dot(force, surfaceVelocity) - dot(force, velocity) - dot(moments[0], omega)) < 1e-12);
    const axisOnlyMoment = add(cross(centers[0], force), cross(centers[1], opposite));
    assert.ok(Math.hypot(...axisOnlyMoment) > 0.01, 'oracle must detect separated-axis force application');
});

test('physical oracle: separated-axis lumen friction must balance its transverse moment', t => {
    const { world, inner, outer, contact } = pair({ offset: 0.1, velocity: 6, spin: 2, friction: 0.2 });
    world.stepFixed();
    let missingMoment = [0, 0, 0], tangentLoad = 0;
    for (const record of contact.kirchhoffContacts) {
        const c = record.manifoldContact;
        if (!c || c.normalLambda <= 1e-12) continue;
        const point = (body, indices, weights, count, segment) => {
            const result = [0, 0, 0];
            assert.ok(Number.isInteger(count) && count > 0, 'contact interpolation count must be available');
            for (let i = 0; i < count; i++) for (let axis = 0; axis < 3; axis++) result[axis] += body[['x', 'y', 'z'][axis]][indices?.[i] ?? segment + i] * weights[i];
            return result;
        };
        const pi = point(inner, record._innerNodeIndices, record._innerNodeWeights ?? record.innerWeights, record._innerNodeCount ?? 2, c.innerSegmentIndex);
        const po = point(outer, record._outerNodeIndices, record._outerNodeWeights ?? record.outerWeights, record._outerNodeCount ?? 2, c.outerSegmentIndex);
        const tangentialImpulse = c.tangentU.map((v, i) => v * c.tangentLambda[0] + c.tangentV[i] * c.tangentLambda[1]);
        tangentLoad += Math.hypot(...tangentialImpulse);
        // Baseline exposes only an axial twist pair. Future surface contact may
        // expose the additional world-space lever-arm moments in these arrays.
        const surfaceMoments = add(record.innerSurfaceMomentImpulse ?? [0, 0, 0], record.outerSurfaceMomentImpulse ?? [0, 0, 0]);
        missingMoment = add(missingMoment, add(cross(sub(pi, po), tangentialImpulse), surfaceMoments));
    }
    assert.ok(tangentLoad > 1e-6, 'exercise loaded sliding contact');
    t.diagnostic(JSON.stringify({ tangentLoad, missingMoment }));
    assert.ok(Math.hypot(...missingMoment) < 1e-8,
        `surface-wrench moment residual ${JSON.stringify(missingMoment)}; axial twist reciprocity alone is insufficient`);
});

test('moving portal crosses material nodes continuously and releases the withdrawn wire tip', () => {
    const { inner, outer, contact } = pair({ count: 9, spacing: 5 });
    outer.setActiveRange(0, 2);
    contact.enforceDistalPortal = true;
    contact.containedLength = 10;
    contact.endNode = 2;
    let previousDistance = null;
    for (let step = 0; step <= 100; step++) {
        const mouthX = 9.5 + step / 100;
        outer.setNodePosition(2, mouthX, 0.1, 0);
        const state = evaluateKirchhoffSlidingPortal(contact);
        assert.ok(state.segment >= 0);
        assert.ok(Math.abs(state.x) < 1e-12, 'portal supplies no axial end cap');
        if (previousDistance != null) assert.ok(Math.abs(state.distance - previousDistance) < 1e-12);
        previousDistance = state.distance;
    }
    inner.setActiveRange(0, 1);
    contact.endNode = 1;
    assert.equal(evaluateKirchhoffSlidingPortal(contact).segment, -1);
});

test('a 0.1 mm catheter overhang beyond the wire tip leaves the fractional portal open', () => {
    const { outer, contact } = pair({ count: 5, spacing: 5 });
    contact.enforceDistalPortal = true;
    contact.containedLength = 20;
    contact.endNode = 4;
    outer.setNodePosition(4, 19.9, 0, 0);
    assert.equal(evaluateKirchhoffSlidingPortal(contact).segment, 3);
    outer.setNodePosition(4, 20.1, 0, 0);
    assert.equal(evaluateKirchhoffSlidingPortal(contact).segment, -1,
        'maximum 1000 mm catheter over a 999.9 mm wire must not create a spherical wire end cap');
});

const recoveryResults = new Map();
function recover(spacing) {
    if (recoveryResults.has(spacing)) return recoveryResults.get(spacing);
    const world = new EndovascularPhysicsWorld({ fixedDt: dt, coupledSystem });
    const body = world.createRod('exposed-berenstein', Math.round(80 / spacing) + 1, spacing, {
        ...profile, mass: 0.005 * spacing, linearDamping: 0.9, angularDamping: 0.9
    });
    applyKirchhoffMaterialProfile(body, 'berenstein', { shaftStiffnessScale: 25, tipStiffnessScale: 5 });
    body.setPinned(0, true);
    const q = frame(body, 0);
    body.setProximalOrientationControl(q.x, q.y, q.z, q.w, 0, 0);
    const manufactured = [...body.restRotation1];
    // Represents a tip just released from a straight sleeve: no shape targets
    // or contacts remain, only its own immutable constitutive rest strain.
    for (let step = 0; step < 1200; step++) world.stepFixed();
    let strain = 0;
    for (let i = 1; i < body.segmentCount; i++) {
        const value = evaluateBendTwistConstraint(frame(body, i - 1), frame(body, i), {
            x: body.restRotation1[i], y: body.restRotation2[i], z: body.restRotation3[i]
        }).strain;
        strain = Math.max(strain, Math.hypot(value.x, value.y, value.z));
    }
    const result = { body, strain, manufactured };
    recoveryResults.set(spacing, result);
    return result;
}

test('an exposed Berenstein tip recovers immutable material shape at runtime stiffness 25/5', () => {
    const { body, strain, manufactured } = recover(2);
    assert.ok(strain < limits.materialStrainRad, `material strain ${strain} rad`);
    assert.ok(maximumRelativeLengthError(body) * 2 < limits.isolatedLengthErrorMm);
    assert.deepEqual([...body.restRotation1], manufactured);
    assert.ok(Math.hypot(body.y[body.activeEnd], body.z[body.activeEnd]) > 1, 'tip must actually bend');
});

test('material-coordinate tip shape converges under 4/2/1 mm mechanical mesh refinement', t => {
    const coarse = recover(4).body, medium = recover(2).body, fine = recover(1).body;
    function distance(a, aSpacing, b, bSpacing) {
        let max = 0;
        for (let s = 0; s <= 80; s += aSpacing) {
            const i = Math.round(s / aSpacing), j = Math.round(s / bSpacing);
            max = Math.max(max, Math.hypot(a.x[i] - b.x[j], a.y[i] - b.y[j], a.z[i] - b.z[j]));
        }
        return max;
    }
    const coarseError = distance(coarse, 4, medium, 2), fineError = distance(medium, 2, fine, 1);
    t.diagnostic(JSON.stringify({ coarseError, fineError }));
    // 0.2 mm is the existing geometric envelope, not a loosened closure residual.
    assert.ok(fineError <= 0.2, `2→1 mm mesh shape difference ${fineError} mm (4→2: ${coarseError})`);
    assert.ok(fineError <= coarseError + 0.001, `refinement must reduce error: ${coarseError} → ${fineError}`);
});
