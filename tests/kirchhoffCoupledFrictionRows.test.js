import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { KirchhoffContactManifold } from '../src/physics/kirchhoffContactManifold.js';
import { evaluateKirchhoffLumenSegmentContact } from '../src/physics/kirchhoffLumenContact.js';
import { prepareKirchhoffCoupledSurfaceGeometry, buildKirchhoffCoupledFrictionRows,
    appendKirchhoffCoupledFrictionRows, commitKirchhoffCoupledFrictionMultipliers,
    measureKirchhoffCoupledFrictionResidual } from '../src/physics/kirchhoffCoupledFrictionRows.js';

const dt = 1 / 120, radius = 0.4445, lumen = 0.485;
const norm = v => Math.hypot(...v);
const sub = (a, b) => a.map((value, i) => value - b[i]);
const add = (a, b) => a.map((value, i) => value + b[i]);
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

function fixture(count = 6) {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const inner = world.createRod('wire', count, 5, { radius });
    const outer = world.createRod('catheter', count, 5, { radius: 0.8 });
    for (let i = 0; i < count; i++) inner.setNodePosition(i, i * 5, 0.1, 0);
    const constraint = { innerBody: inner, outerBody: outer, innerRadius: lumen,
        axialFriction: 0.2, torsionalFriction: 0.4, portalFilletRadius: 0.15,
        kirchhoffContacts: [] };
    function record(kind = 'side') {
        const index = constraint.kirchhoffContacts.length;
        const r = { id: `contact-${index}`, kind, gap: -0.0595, normal: [0, 1, 0],
            _innerSegmentIndex: 0, _outerSegmentIndex: 0, innerT: 0.5, outerT: 0.5,
            innerWeights: [0.5, 0.5], outerWeights: [0.5, 0.5], manifoldContact: {
                normalLambda: 2, tangentLambda: new Float64Array([0.2, 0.1]), twistLambda: 0.03,
                innerTwistImpulse: 0.03, outerTwistImpulse: -0.03,
                tangentU: [1, 0, 0], tangentV: [0, 0, -1], normal: [0, 1, 0],
                innerSegmentIndex: 0, outerSegmentIndex: 0
            } };
        constraint.kirchhoffContacts.push(r);
        return r;
    }
    return { world, inner, outer, constraint, record };
}

function momentumResidual(entry, force) {
    const s = entry.surface;
    return add(add(cross([...s.centers[0]], force), cross([...s.levers[0]], force)),
        add(cross([...s.centers[1]], force.map(value => -value)), cross([...s.levers[1]], force.map(value => -value))));
}

test('all current record kinds produce finite complete U/V rows and fixed-normal groups', () => {
    const f = fixture();
    for (const kind of ['side', 'material-side', 'distal-fillet', 'distal-rim', 'sliding-rim']) f.record(kind);
    const batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    assert.equal(batch.rows.length, 10); assert.equal(batch.groups.length, 5); assert.equal(batch.entries.length, 5);
    assert.equal(batch.skipped.length, 0);
    for (let i = 0; i < 5; i++) {
        assert.deepEqual([...batch.groups[i].rowIndices], [i * 2, i * 2 + 1]);
        assert.equal(batch.groups[i].normalLambda, 2);
        assert.deepEqual([...batch.groups[i].mu], [0.2, 0.4]);
        assert.ok([...batch.entries[i].geometry.point].every(Number.isFinite));
        for (const row of batch.entries[i].surface.rows) {
            assert.ok(Number.isFinite(row.strain) && Number.isFinite(row.lambda));
            assert.ok(row.gradients.every(g => Number.isFinite(g.value)));
        }
        assert.ok(norm(momentumResidual(batch.entries[i], [0.1, 0.2, -0.3])) < 1e-12);
    }
    assert.equal(batch.effectiveFilletCount, 1);
});

test('effective fillet witness agrees with its existing torus gap and remains finite at the meridian pole', () => {
    const f = fixture(2), manifold = new KirchhoffContactManifold({ frictionCoefficient: 0.2 });
    f.inner.setNodePosition(0, 9.9, 0.18, 0); f.inner.setNodePosition(1, 10.1, 0.18, 0);
    f.outer.setNodePosition(0, 0, 0, 0); f.outer.setNodePosition(1, 10, 0, 0);
    const point = (b, i) => [b.x[i], b.y[i], b.z[i]];
    const evaluated = evaluateKirchhoffLumenSegmentContact({
        innerStart: point(f.inner, 0), innerEnd: point(f.inner, 1),
        outerStart: point(f.outer, 0), outerEnd: point(f.outer, 1), lumenRadius: lumen,
        innerRadius: f.inner.nodeRadius[0], innerMaterialSegmentId: 1, outerMaterialSegmentId: 2,
        innerSegmentIndex: 0, outerSegmentIndex: 0, openDistal: true,
        portalFilletRadius: 0.15, activationDistance: 1, manifold
    });
    const record = evaluated.fillet;
    assert.ok(record && record.gap < 0);
    record.manifoldContact.normalLambda = 1;
    f.constraint.kirchhoffContacts = [record];
    const batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt), geometry = batch.entries[0].geometry;
    const ci = [...geometry.innerCenter], r = geometry.wireRadius;
    const center = [9.85, lumen - r + 0.15, 0];
    const delta = sub(ci, center), distance = norm(delta);
    const boundary = add(center, delta.map(v => v * 0.15 / distance));
    const outerWitness = add(boundary, [...geometry.normal].map(v => r * v));
    assert.ok(norm(sub(outerWitness, [...geometry.outerWitness])) < 1e-12);
    assert.equal(geometry.effectiveFillet, true);
    assert.equal(geometry.physicalSurfaceVerified, false);
    assert.ok(norm(geometry.normalMomentResidual) > 0.1, 'nonradial normal itself needs a torque-aware gradient');

    // At the rounded lip pole n=-axis, projection of shaft tangent vanishes;
    // the physical meridional tangent continues radially instead of becoming NaN.
    record.normal = [-1, 0, 0]; record.manifoldContact.normal = [-1, 0, 0];
    record.manifoldContact.tangentU = [0, 1, 0]; record.manifoldContact.tangentV = [0, 0, -1];
    const pole = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    assert.equal(pole.entries[0].surface.supported, true);
    assert.ok(Math.abs(dot([...pole.entries[0].surface.axes[0]], [0, 1, 0])) > 1 - 1e-12);
});

test('smooth material-side gets the exact B-spline tangent instead of a segment chord', () => {
    const f = fixture(), record = f.record('material-side'), t = 0.3;
    for (let i = 0; i < f.outer.count; i++) f.outer.setNodePosition(i, i * 5, Math.sin(i) * 2, 0);
    const weights = value => [(1 - value) ** 3 / 6, (4 - 6 * value ** 2 + 3 * value ** 3) / 6,
        (1 + 3 * value + 3 * value ** 2 - 3 * value ** 3) / 6, value ** 3 / 6];
    record._outerSegmentIndex = record.manifoldContact.outerSegmentIndex = 2;
    record._outerNodeIndices = [1, 2, 3, 4]; record._outerNodeCount = 4;
    record._outerNodeWeights = weights(t); record.outerT = t;
    const sample = value => ['x', 'y', 'z'].map(key => weights(value).reduce((s, w, i) => s + w * f.outer[key][i + 1], 0));
    const derivative = sub(sample(t + 1e-6), sample(t - 1e-6)), length = norm(derivative);
    const geometry = prepareKirchhoffCoupledSurfaceGeometry(f.constraint, record);
    assert.ok(norm(sub([...geometry.axialTangent], derivative.map(v => v / length))) < 1e-9);
    assert.equal(geometry.tangentSource, 'uniform-bspline-derivative');
    record._outerNodeWeights[0] += 0.001; record._outerNodeWeights[1] -= 0.001;
    assert.throws(() => prepareKirchhoffCoupledSurfaceGeometry(f.constraint, record), /surfaceAxialTangent/);
    record.surfaceAxialTangent = [1, 0, 0];
    assert.equal(prepareKirchhoffCoupledSurfaceGeometry(f.constraint, record).tangentSource, 'provided');
});

test('append offsets groups after boundary rows and reuses buffers on the next build', () => {
    const f = fixture(); f.record(); f.record('distal-rim');
    const batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt), firstRow = batch.rows[0], firstEntry = batch.entries[0];
    const rows = [{ boundary: 1 }, { boundary: 2 }], groups = [];
    appendKirchhoffCoupledFrictionRows(batch, rows, groups);
    assert.equal(batch.rowOffset, 2); assert.equal(rows.length, 6);
    assert.deepEqual(groups.map(g => [...g.rowIndices]), [[2, 3], [4, 5]]);
    assert.throws(() => appendKirchhoffCoupledFrictionRows(batch, rows, groups), /already appended/);
    f.constraint.kirchhoffContacts[0].manifoldContact.normalLambda = 0.1;
    buildKirchhoffCoupledFrictionRows(f.constraint, dt, batch);
    assert.equal(batch.rows[0], firstRow); assert.equal(batch.entries[0], firstEntry);
    assert.equal(batch.groups[0].normalLambda, 0.1); assert.equal(batch.appended, false);
});

test('commit shares scale, changes basis, clears legacy twist and reports physically applied moments without moving bodies', () => {
    const f = fixture(), record = f.record(), batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    appendKirchhoffCoupledFrictionRows(batch, [{ boundary: 1 }], []);
    const before = [f.inner, f.outer].map(b => Array.from(b.x));
    record.manifoldContact.tangentU = [0, 0, -1];
    record.manifoldContact.tangentV = [-1, 0, 0];
    record.manifoldContact.normalLambda = 0.05; // caller's normal commit may shrink Fn; do not project.
    commitKirchhoffCoupledFrictionMultipliers(batch, [99, -0.3, 0.2], 0.5);
    assert.ok(Math.abs(record.manifoldContact.tangentLambda[0] - 0.2) < 1e-12);
    assert.ok(Math.abs(record.manifoldContact.tangentLambda[1] + 0.05) < 1e-12);
    assert.equal(record.manifoldContact.normalLambda, 0.05);
    assert.equal(record.manifoldContact.twistLambda, 0);
    assert.equal(record.manifoldContact.innerTwistImpulse, 0); assert.equal(record.manifoldContact.outerTwistImpulse, 0);
    assert.deepEqual([f.inner, f.outer].map(b => Array.from(b.x)), before);
    const force = [...record.surfaceTangentialImpulse];
    assert.ok(norm(sub(force, [0.05, 0, -0.2])) < 1e-12);
    const centers = batch.entries[0].surface.centers;
    const moment = add(add(cross([...centers[0]], force), cross([...centers[1]], force.map(v => -v))),
        add([...record.innerSurfaceMomentImpulse], [...record.outerSurfaceMomentImpulse]));
    assert.ok(norm(moment) < 1e-12);
    assert.ok(norm(sub([...record.surfaceTangentialIncrement], [-0.15, 0, -0.1])) < 1e-12);
    assert.throws(() => commitKirchhoffCoupledFrictionMultipliers(batch, [0, 0, 0], 1), /already committed/);
});

test('rebuild residual uses refreshed final Fn and basis without clipping or recommitting any state', () => {
    const f = fixture(), record = f.record(), batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    commitKirchhoffCoupledFrictionMultipliers(batch, [0, 0], 1);
    const increment = [...record.innerSurfaceMomentIncrement];
    record.manifoldContact.normalLambda = 0.01;
    const lambda = [...record.manifoldContact.tangentLambda], position = [...f.inner.x];
    // Geometry refresh changes the gap/point, but only diagnostics will be refreshed.
    record.gap = -0.02;
    const result = measureKirchhoffCoupledFrictionResidual(f.constraint, dt, {}, { inverseMobility: 0.7 });
    assert.ok(result.maximumFeasibilityResidual > 0.1);
    assert.equal(result.residualUnits, 'multiplier'); assert.equal(result.inverseMobility, 0.7);
    assert.deepEqual([...record.manifoldContact.tangentLambda], lambda);
    assert.deepEqual([...f.inner.x], position); assert.equal(record.manifoldContact.normalLambda, 0.01);
    assert.deepEqual([...record.innerSurfaceMomentIncrement], increment);
    assert.equal(record.surfaceFrictionDiagnosticPhase, 'refreshed-total-reaction');
    assert.equal(result._batch.groups[0].normalLambda, 0.01);
});

test('invalid increments are rejected atomically and stale contact topology cannot commit', () => {
    const f = fixture(), a = f.record(), b = f.record(), batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    const before = [a, b].map(r => [...r.manifoldContact.tangentLambda, r.manifoldContact.twistLambda]);
    assert.throws(() => commitKirchhoffCoupledFrictionMultipliers(batch, [1, 2, 3, NaN], 1), /finite/);
    assert.deepEqual([a, b].map(r => [...r.manifoldContact.tangentLambda, r.manifoldContact.twistLambda]), before);
    b._innerSegmentIndex = 1;
    assert.throws(() => commitKirchhoffCoupledFrictionMultipliers(batch, [1, 2, 3, 4], 1), /topology/);
    assert.deepEqual([a, b].map(r => [...r.manifoldContact.tangentLambda, r.manifoldContact.twistLambda]), before);
});

test('zero-load records stay in the batch and null-manifold records are explicitly accounted for', () => {
    const f = fixture(), unloaded = f.record('sliding-rim'), inactive = f.record('side');
    unloaded.manifoldContact.normalLambda = 0; inactive.manifoldContact = null;
    const batch = buildKirchhoffCoupledFrictionRows(f.constraint, dt);
    assert.equal(batch.rows.length, 2); assert.equal(batch.groups[0].normalLambda, 0);
    assert.equal(batch.skipped.length, 1); assert.equal(batch.skipped[0].reason, 'no-manifold-contact');
    assert.ok(measureKirchhoffCoupledFrictionResidual(f.constraint, dt).maximumFeasibilityResidual > 0);
    assert.throws(() => buildKirchhoffCoupledFrictionRows({ ...f.constraint, kirchhoffContacts: [] }, 0), /dt/);
});

test('actual baseline runtime records are all included and publishing diagnostics never adds a position correction', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const p = { radius, mass: 1, linearDamping: 1, angularDamping: 1, foldLimitStrength: 0,
        projectionVelocityRetention: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 2, 10, p), outer = world.createRod('catheter', 2, 10, { ...p, mass: 3 });
    for (let i = 0; i < 2; i++) { inner.setNodePosition(i, i * 10, 0.1, 0); inner.velocityX[i] = 6; }
    inner.angularVelocityX[0] = 2;
    const c = world.addContainment(inner, outer, { innerRadius: lumen, axialFriction: 0.2,
        torsionalFriction: 0.2, portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    world.stepFixed();
    const batch = buildKirchhoffCoupledFrictionRows(c, dt);
    assert.equal(batch.entries.length, c.kirchhoffContacts.filter(r => r.manifoldContact).length);
    assert.ok(batch.entries.some(e => e.record.kind === 'distal-rim'));
    const positions = [inner, outer].map(b => [...b.x, ...b.y, ...b.z]);
    commitKirchhoffCoupledFrictionMultipliers(batch, new Float64Array(batch.rows.length), 1);
    assert.deepEqual([inner, outer].map(b => [...b.x, ...b.y, ...b.z]), positions);
    for (const entry of batch.entries) assert.ok(norm(momentumResidual(entry, [...entry.record.surfaceTangentialImpulse])) < 1e-12);
});
