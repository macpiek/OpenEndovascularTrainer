import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import { buildKirchhoffContactNormalGradients } from '../src/physics/kirchhoffContactNormalRows.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';

const radius = 0.4445, lumen = 0.485, fillet = 0.15;
const v = values => new THREE.Vector3(...values);
function body(points) {
    return { count: points.length, radius, nodeRadius: new Float64Array(points.length).fill(radius),
        x: Float64Array.from(points.map(p => p[0])), y: Float64Array.from(points.map(p => p[1])), z: Float64Array.from(points.map(p => p[2])) };
}
const point = (b, node) => new THREE.Vector3(b.x[node], b.y[node], b.z[node]);

function fixture(kind = 'distal-fillet', mode = 'fixed') {
    const inner = body([[9.7, 0.16, 0.02], [10.2, 0.2, 0.02]]);
    const outer = body([[0, 0, 0], [10, 0, 0]]);
    const constraint = { innerBody: inner, outerBody: outer, innerRadius: lumen, portalFilletRadius: fillet };
    const record = { kind, innerT: 0.5, outerT: 1, _innerSegmentIndex: 0, _outerSegmentIndex: 0,
        normal: [], clearance: lumen - radius + fillet, manifoldContact: { normalLambda: 2 },
        innerWeights: [0.5, 0.5], outerWeights: [0, 1] };
    const f = { inner, outer, constraint, record, mode };
    updateRecord(f);
    return f;
}

// Independent scalar geometry using vector operations, not the analytic
// Jacobian. Recompute implicit t on every perturbation for boundary/rim tests.
function geometry(f) {
    const a0 = point(f.inner, 0), a1 = point(f.inner, 1), o = point(f.outer, 1);
    const axis = o.clone().sub(point(f.outer, 0)).normalize(), direction = a1.clone().sub(a0);
    let t = f.record.innerT;
    if (f.mode === 'portal-side-boundary' || f.record.kind === 'distal-rim') {
        const offset = f.mode === 'portal-side-boundary' ? -fillet : 0;
        t = (offset - a0.clone().sub(o).dot(axis)) / direction.dot(axis);
    } else if (f.record.kind === 'sliding-rim') {
        t = THREE.MathUtils.clamp(o.clone().sub(a0).dot(direction) / direction.lengthSq(), 0, 1);
    }
    const c = a0.addScaledVector(direction, t), offset = c.clone().sub(o);
    const z = offset.dot(axis), radial = offset.clone().addScaledVector(axis, -z), rho = radial.length();
    if (f.record.kind === 'distal-fillet') {
        const circle = new THREE.Vector2(z + fillet, rho - (lumen - radius + fillet));
        const unit = circle.clone().normalize();
        const normal = axis.clone().multiplyScalar(-unit.x).addScaledVector(radial.normalize(), -unit.y);
        return { gap: circle.length() - fillet, normal: normal.toArray(), t };
    }
    if (f.record.kind === 'sliding-rim') return { gap: lumen - radius - offset.length(), normal: offset.normalize().toArray(), t };
    return { gap: f.record.clearance - rho, normal: radial.normalize().toArray(), t };
}

function updateRecord(f) {
    const g = geometry(f);
    f.record.gap = g.gap; f.record.normal = g.normal; f.record.innerT = g.t;
    f.record.innerWeights = [1 - g.t, g.t];
}

test('compact rollback rebuilds exactly the original fillet Jacobian after changed candidate geometry', () => {
    const f = fixture();
    f.constraint._kirchhoffRuntimeRecordPool = [[f.record]];
    f.constraint.kirchhoffContacts = [f.record];
    const expected = structuredClone(buildKirchhoffContactNormalGradients(f.constraint, f.record));
    const snapshot = captureKirchhoffCoupledTrialState(f.constraint,
        { physicalStateOnly: true, frozenFrictionBatches: true, reusePropertyLayout: true });
    assert.ok(snapshot.contactTrial.derived.size > 0);
    for (const shift of [.02, -.03, .05]) {
        f.inner.y[0] += shift;
        updateRecord(f);
        const trial = buildKirchhoffContactNormalGradients(f.constraint, f.record);
        assert.notDeepEqual(trial, expected);
        f.record.manifoldContact.normalLambda = 99;
        restoreKirchhoffCoupledTrialState(snapshot);
        assert.equal(f.record.manifoldContact.normalLambda, 2);
        assert.deepEqual(buildKirchhoffContactNormalGradients(f.constraint, f.record), expected);
    }
});

test('near-axis fillet Jacobian retains the radial branch selected by the runtime collector', () => {
    const f = fixture();
    f.inner.y.fill(5e-10); f.inner.z.fill(0);
    // Runtime uses a fallback azimuth below 1e-8 mm, while the generic
    // derivative's 1e-12 threshold would select the tiny y offset instead.
    const axial = (f.inner.x[0] + f.inner.x[1]) / 2 - f.outer.x[1];
    const u = axial + fillet, v = 5e-10 - (lumen - radius + fillet), distance = Math.hypot(u, v);
    f.record.normal = [-u / distance, 0, -v / distance];
    f.record.normalRadialEpsilon = 1e-8;
    const normal = [...f.record.normal], lambda = f.record.manifoldContact.normalLambda;
    const rows = buildKirchhoffContactNormalGradients(f.constraint, f.record);
    assert.ok(rows.length > 0 && rows.every(row => Number.isFinite(row.value)));
    assert.ok(f.record.normalGradientDiagnostics.normalMismatch < 1e-12);
    assert.equal(f.record.normalGradientDiagnostics.directionalAtAxis, true);
    assert.deepEqual(f.record.normal, normal);
    assert.equal(f.record.manifoldContact.normalLambda, lambda);
    delete f.record.normalRadialEpsilon;
    assert.throws(() => buildKirchhoffContactNormalGradients(f.constraint, f.record), /geometry changed/);
    f.record.normalRadialEpsilon = -1;
    assert.throws(() => buildKirchhoffContactNormalGradients(f.constraint, f.record), /radial epsilon/);
});

function dense(rows) {
    const result = Array.from({ length: 2 }, () => new Float64Array(12));
    for (const row of rows) result[row.side][row.dof] += row.value;
    return result;
}

function balances(f, rows) {
    const force = new THREE.Vector3(), moment = new THREE.Vector3();
    for (const row of rows) {
        const body = row.side === 0 ? f.inner : f.outer, node = Math.floor(row.dof / 6), axis = row.dof % 6;
        assert.ok(axis < 3, 'endpoint derivative must not double-count an angular lever');
        const gradient = new THREE.Vector3().setComponent(axis, row.value);
        force.add(gradient); moment.add(point(body, node).cross(gradient));
    }
    return { force: force.length(), moment: moment.length() };
}

function finiteDifference(f, rows, tolerance = 2e-7) {
    const gradients = dense(rows), step = 1e-6;
    for (let side = 0; side < 2; side++) for (let node = 0; node < 2; node++) for (let axis = 0; axis < 3; axis++) {
        const values = [f.inner, f.outer][side][['x', 'y', 'z'][axis]], original = values[node];
        values[node] = original + step; const plus = geometry(f).gap;
        values[node] = original - step; const minus = geometry(f).gap;
        values[node] = original;
        const numeric = (plus - minus) / (2 * step), expected = gradients[side][node * 6 + axis];
        assert.ok(Math.abs(numeric - expected) < tolerance,
            `side ${side} node ${node} axis ${axis}: ${numeric} != ${expected}`);
    }
}

test('fillet exact endpoint gradient passes finite differences and closes its missing normal moment', () => {
    const f = fixture(), before = JSON.stringify({ gap: f.record.gap, normal: f.record.normal,
        lambda: f.record.manifoldContact.normalLambda, positions: [f.inner, f.outer] });
    const defaultForce = v(f.record.normal).negate();
    const c = point(f.inner, 0).multiplyScalar(1 - f.record.innerT).addScaledVector(point(f.inner, 1), f.record.innerT);
    const oldMoment = c.sub(point(f.outer, 1)).cross(defaultForce).length();
    assert.ok(oldMoment > 0.05, 'fixture must expose the old missing moment');
    const rows = buildKirchhoffContactNormalGradients(f.constraint, f.record);
    finiteDifference(f, rows);
    const balance = balances(f, rows);
    assert.ok(balance.force < 1e-12 && balance.moment < 1e-12);
    assert.equal(f.record.normalGradientDiagnostics.parameterMode, 'fixed');
    assert.equal(JSON.stringify({ gap: f.record.gap, normal: f.record.normal,
        lambda: f.record.manifoldContact.normalLambda, positions: [f.inner, f.outer] }), before);
});

test('the clipped moving fillet sample includes the implicit crossing derivative', () => {
    const f = fixture('distal-fillet', 'portal-side-boundary');
    assert.ok(Math.abs(f.record.innerT * 4 - Math.round(f.record.innerT * 4)) > 0.01);
    const rows = buildKirchhoffContactNormalGradients(f.constraint, f.record);
    finiteDifference(f, rows);
    assert.equal(f.record.normalGradientDiagnostics.parameterMode, 'inferred-portal-side-boundary');
    assert.ok(balances(f, rows).moment < 1e-12);
    f.record.normalInnerParameterMode = 'portal-side-boundary';
    finiteDifference(f, buildKirchhoffContactNormalGradients(f.constraint, f.record));
});

test('fillet normal response is objective under a rigid rotation and translation', () => {
    const f = fixture(), original = dense(buildKirchhoffContactNormalGradients(f.constraint, f.record));
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.5, -0.3, 0.8));
    const shift = new THREE.Vector3(10, -2, 3);
    for (const b of [f.inner, f.outer]) for (let node = 0; node < 2; node++) {
        const p = point(b, node).applyQuaternion(q).add(shift);
        [b.x[node], b.y[node], b.z[node]] = p.toArray();
    }
    updateRecord(f);
    const rows = buildKirchhoffContactNormalGradients(f.constraint, f.record), actual = dense(rows);
    for (let side = 0; side < 2; side++) for (let node = 0; node < 2; node++) {
        const expected = new THREE.Vector3(...original[side].slice(node * 6, node * 6 + 3)).applyQuaternion(q);
        assert.ok(expected.distanceTo(new THREE.Vector3(...actual[side].slice(node * 6, node * 6 + 3))) < 1e-12);
    }
    finiteDifference(f, rows);
    assert.ok(balances(f, rows).moment < 1e-12);
});

test('optional rim gradient differentiates the oblique plane crossing and permits axial wire slide', () => {
    const f = fixture('distal-rim', 'portal-plane');
    assert.equal(buildKirchhoffContactNormalGradients(f.constraint, f.record), null, 'rim migration is opt-in');
    const rows = buildKirchhoffContactNormalGradients(f.constraint, f.record, { includeRim: true });
    finiteDifference(f, rows);
    assert.ok(balances(f, rows).moment < 1e-12);
    const gradients = dense(rows), direction = point(f.inner, 1).sub(point(f.inner, 0));
    const totalWire = new THREE.Vector3(gradients[0][0] + gradients[0][6], gradients[0][1] + gradients[0][7], gradients[0][2] + gradients[0][8]);
    assert.ok(Math.abs(totalWire.dot(direction)) < 1e-12, 'normal-only rim must not oppose frictionless material slide');
    assert.ok(f.record.normalGradientDiagnostics.innerGradientMagnitude >= 1);
});

test('sliding-rim closest-point default already has the exact branch gradient', () => {
    const f = fixture('sliding-rim');
    assert.equal(buildKirchhoffContactNormalGradients(f.constraint, f.record), null);
    const rows = [];
    for (let axis = 0; axis < 3; axis++) {
        rows.push({ side: 0, dof: axis, value: -(1 - f.record.innerT) * f.record.normal[axis] },
            { side: 0, dof: 6 + axis, value: -f.record.innerT * f.record.normal[axis] },
            { side: 1, dof: 6 + axis, value: f.record.normal[axis] });
    }
    finiteDifference(f, rows);
    assert.ok(balances(f, rows).moment < 1e-12);
});

test('pooled rows are reused and cleared when a record changes feature', () => {
    const f = fixture();
    const a = buildKirchhoffContactNormalGradients(f.constraint, f.record), first = a[0];
    const b = buildKirchhoffContactNormalGradients(f.constraint, f.record);
    assert.equal(a, b); assert.equal(b[0], first);
    f.record.kind = 'side';
    assert.equal(buildKirchhoffContactNormalGradients(f.constraint, f.record), null);
    assert.equal(f.record.normalGradients, null);
});

test('invalid or stale contact geometry fails visibly instead of changing the collision gap', () => {
    const f = fixture();
    f.record.normal = [0, 1, 0];
    assert.throws(() => buildKirchhoffContactNormalGradients(f.constraint, f.record), /geometry changed/);
    updateRecord(f); f.constraint.portalFilletRadius = 0;
    assert.throws(() => buildKirchhoffContactNormalGradients(f.constraint, f.record), /positive/);
    f.constraint.portalFilletRadius = fillet; f.outer.x[0] = f.outer.x[1];
    assert.throws(() => buildKirchhoffContactNormalGradients(f.constraint, f.record), /degenerate/);
});
