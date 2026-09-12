import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import { buildKirchhoffSurfaceFriction, measureKirchhoffSurfaceFrictionState, projectKirchhoffSurfaceFriction,
    evaluateKirchhoffSurfaceFriction } from '../src/physics/kirchhoffSurfaceFriction.js';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';

const dt = 1 / 120, radius = 0.4445, lumen = 0.485, clearance = lumen - radius;
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const cross = (a, b) => new THREE.Vector3(...a).cross(new THREE.Vector3(...b)).toArray();
const add = (a, b) => a.map((value, i) => value + b[i]);
const subtract = (a, b) => a.map((value, i) => value - b[i]);
const norm = vector => Math.hypot(...vector);
const rotate = (q, vector) => new THREE.Vector3(...vector).applyQuaternion(q).toArray();
const aligned = () => new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, 0, 0));

test('gradient objects survive an intervening residual-only evaluation', () => {
    const f = fixture(), out = {};
    const full = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out);
    const entries = full.rows.map(row => [...row.gradients]);
    const values = structuredClone(entries);
    measureKirchhoffSurfaceFrictionState(f.constraint, f.record, dt, out);
    assert.ok(out.rows.every(row => row.gradients.length === 0));
    buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out);
    for (let axis = 0; axis < 2; axis++) {
        assert.deepEqual(out.rows[axis].gradients, values[axis]);
        for (let i = 0; i < entries[axis].length; i++) assert.equal(out.rows[axis].gradients[i], entries[axis][i]);
    }
});

function getQ(body, segment = 0, previous = false) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    return new THREE.Quaternion(...['X', 'Y', 'Z', 'W'].map(axis => body[prefix + axis][segment]));
}

function setQ(body, q, segment = 0, previous = false) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { body[prefix + axis][segment] = q.toArray()[i]; });
}

function body(y, count = 4) {
    const rod = { count, segmentCount: count - 1, radius, nodeRadius: new Float64Array(count).fill(radius) };
    for (const key of ['x', 'y', 'z', 'previousX', 'previousY', 'previousZ']) rod[key] = new Float64Array(count);
    for (const prefix of ['orientation', 'previousOrientation']) for (const axis of ['X', 'Y', 'Z', 'W']) rod[prefix + axis] = new Float64Array(count - 1);
    for (let i = 0; i < count; i++) {
        rod.x[i] = rod.previousX[i] = i * 5;
        rod.y[i] = rod.previousY[i] = y;
    }
    for (let i = 0; i < count - 1; i++) { setQ(rod, aligned(), i); setQ(rod, aligned(), i, true); }
    return rod;
}

function fixture({ muU = 0.2, muV = 0.5, normalLambda = 2, kind = 'side' } = {}) {
    const inner = body(clearance), outer = body(0);
    const contact = { normalLambda, tangentLambda: new Float64Array(2), twistLambda: 0,
        tangentU: [1, 0, 0], tangentV: [0, 0, -1], normal: [0, 1, 0],
        innerSegmentIndex: 0, outerSegmentIndex: 0 };
    const record = { kind, normal: [0, 1, 0], innerWeights: [0.3, 0.7], outerWeights: [0.3, 0.7],
        _innerSegmentIndex: 0, _outerSegmentIndex: 0, manifoldContact: contact };
    const constraint = { innerBody: inner, outerBody: outer, innerRadius: lumen,
        axialFriction: muU, torsionalFriction: muV };
    return { inner, outer, contact, record, constraint };
}

function rowWrench(row, bodies) {
    const force = [0, 0, 0], moment = [0, 0, 0];
    const localTorques = bodies.map(body => Array.from({ length: body.segmentCount }, () => [0, 0, 0]));
    for (const g of row.gradients) {
        const body = bodies[g.side], node = Math.floor(g.dof / 6), axis = g.dof % 6;
        if (axis < 3) {
            const f = [0, 0, 0]; f[axis] = g.value;
            const r = [body.x[node], body.y[node], body.z[node]], m = cross(r, f);
            for (let i = 0; i < 3; i++) { force[i] += f[i]; moment[i] += m[i]; }
        } else localTorques[g.side][node][axis - 3] += g.value;
    }
    for (let side = 0; side < 2; side++) for (let segment = 0; segment < bodies[side].segmentCount; segment++) {
        const m = rotate(getQ(bodies[side], segment), localTorques[side][segment]);
        for (let i = 0; i < 3; i++) moment[i] += m[i];
    }
    return { force, moment };
}

test('one common surface point closes total force and orbital plus material moment for both friction axes', () => {
    const f = fixture();
    // Noncoincident centers; rotations make world and material angular dofs distinct.
    setQ(f.inner, new THREE.Quaternion().setFromEuler(new THREE.Euler(0.31, 0.72, -0.18)));
    setQ(f.outer, new THREE.Quaternion().setFromEuler(new THREE.Euler(-0.21, 0.52, 0.38)));
    f.record.surfaceContactPoint = [3.5, lumen, 0.1];
    const result = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.equal(result.supported, true);
    assert.equal(result.rows.length, 2);
    for (const row of result.rows) {
        assert.equal(row.alpha, 0); assert.equal(row.lower, -Infinity); assert.equal(row.upper, Infinity);
        assert.equal('group' in row, false);
        const balance = rowWrench(row, [f.inner, f.outer]);
        assert.ok(norm(balance.force) < 1e-12, `net force ${balance.force}`);
        assert.ok(norm(balance.moment) < 1e-12, `net moment ${balance.moment}`);
        assert.ok(row.gradients.some(g => g.side === 0 && g.dof % 6 >= 3));
        assert.ok(row.gradients.some(g => g.side === 1 && g.dof % 6 >= 3));
    }
    assert.ok(Math.abs(norm(result.levers[0]) - norm(result.levers[1])) > 0.01,
        'moments cannot be replaced by one equal-and-opposite axial twist pair');
});

test('finite differences verify every world translation and LOCAL right-sided angular gradient', () => {
    const f = fixture();
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.31, 0.72, -0.18));
    setQ(f.inner, q); setQ(f.outer, q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), 0.5)));
    f.record.surfaceContactPoint = [3.5, lumen, 0.1];
    const built = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    const bodies = [f.inner, f.outer], locals = built.localLevers.map(value => [...value]);
    // Independent surface potential, with the same frozen local material anchors.
    const potential = axis => {
        const points = bodies.map((b, side) => {
            const weights = side === 0 ? f.record.innerWeights : f.record.outerWeights;
            const center = ['x', 'y', 'z'].map(key => b[key][0] * weights[0] + b[key][1] * weights[1]);
            return add(center, rotate(getQ(b), locals[side]));
        });
        return dot([...built.axes[axis]], subtract(points[0], points[1]));
    };
    const epsilon = 1e-6;
    for (let axis = 0; axis < 2; axis++) {
        const gradient = new Map();
        for (const g of built.rows[axis].gradients) gradient.set(`${g.side}:${g.dof}`, (gradient.get(`${g.side}:${g.dof}`) ?? 0) + g.value);
        for (let side = 0; side < 2; side++) for (let dof = 0; dof < 12; dof++) {
            const b = bodies[side], node = Math.floor(dof / 6), component = dof % 6;
            let plus, minus;
            if (component < 3) {
                const values = b[['x', 'y', 'z'][component]], original = values[node];
                values[node] = original + epsilon; plus = potential(axis);
                values[node] = original - epsilon; minus = potential(axis);
                values[node] = original;
            } else {
                const original = getQ(b, node), unit = new THREE.Vector3(); unit.setComponent(component - 3, 1);
                setQ(b, original.clone().multiply(new THREE.Quaternion().setFromAxisAngle(unit, epsilon)), node); plus = potential(axis);
                setQ(b, original.clone().multiply(new THREE.Quaternion().setFromAxisAngle(unit, -epsilon)), node); minus = potential(axis);
                setQ(b, original, node);
            }
            assert.ok(Math.abs((plus - minus) / (2 * epsilon) - (gradient.get(`${side}:${dof}`) ?? 0)) < 2e-9,
                `row ${axis}, side ${side}, dof ${dof}: finite difference ${(plus - minus) / (2 * epsilon)}`);
        }
    }
});

test('negative cubic interpolation preserves partition of unity, rigid translation and moment reciprocity', () => {
    const f = fixture();
    f.record._innerNodeIndices = [0, 1, 2, 3];
    f.record._innerNodeWeights = [-0.0625, 0.5625, 0.5625, -0.0625];
    f.record._innerNodeCount = 4;
    f.record.surfaceContactPoint = [5, lumen, 0];
    for (let i = 0; i < f.inner.count; i++) f.inner.x[i] += 0.02;
    const result = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.ok(Math.abs(result.rows[0].strain - 0.02) < 1e-12);
    for (const row of result.rows) assert.ok(norm(rowWrench(row, [f.inner, f.outer]).moment) < 1e-12);
    f.record._innerNodeWeights[0] -= 0.01;
    assert.throws(() => buildKirchhoffSurfaceFriction(f.constraint, f.record, dt), /sum to one/);
});

test('a shared finite rigid motion has zero relative surface displacement', () => {
    const f = fixture(), rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.4, -0.3, 0.2));
    const translation = [0.4, -0.1, 0.2];
    for (const b of [f.inner, f.outer]) {
        for (let i = 0; i < b.count; i++) {
            const point = add(rotate(rotation, [b.x[i], b.y[i], b.z[i]]), translation);
            [b.x[i], b.y[i], b.z[i]] = point;
        }
        for (let i = 0; i < b.segmentCount; i++) setQ(b, rotation.clone().multiply(getQ(b, i, true)), i);
    }
    f.record.normal = rotate(rotation, [0, 1, 0]);
    const result = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.equal(result.supported, true);
    assert.ok(norm(result.relativeSurfaceDisplacement) < 1e-12);
    assert.ok(norm(result.rows.map(row => row.strain)) < 1e-12);
});

test('finite relative twist and axial translation appear as separate surface slip components', () => {
    const f = fixture(), angle = 0.3;
    setQ(f.inner, new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), angle).multiply(aligned()));
    const twist = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.ok(Math.abs(twist.rows[0].strain) < 1e-12);
    assert.ok(Math.abs(twist.rows[1].strain + radius * Math.sin(angle)) < 1e-12);
    assert.ok(Math.abs(twist.averageRelativeSurfaceVelocity[2] - radius * Math.sin(angle) / dt) < 1e-12);
    const g = fixture();
    for (let i = 0; i < g.inner.count; i++) g.inner.x[i] += 0.2;
    const slide = buildKirchhoffSurfaceFriction(g.constraint, g.record, dt);
    assert.ok(Math.abs(slide.rows[0].strain - 0.2) < 1e-12);
    assert.ok(Math.abs(slide.rows[1].strain) < 1e-12);
});

test('virtual work uses v + omega cross r and admits surface rolling with zero slip', () => {
    const f = fixture(), built = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    const omega = [0.8, -0.3, 0.5];
    const linear = cross(omega, [...built.levers[0]]).map(value => -value);
    const localOmega = rotate(getQ(f.inner).invert(), omega);
    for (const row of built.rows) {
        let power = 0;
        for (const g of row.gradients) if (g.side === 0) power += g.value *
            (g.dof % 6 < 3 ? linear[g.dof % 6] : localOmega[g.dof % 6 - 3]);
        assert.ok(Math.abs(power) < 1e-12, `rolling slip ${power}`);
    }
    const velocity = [2, -0.4, 0.1], forceComponents = [-0.12, 0.21];
    const surfaceVelocity = add(velocity, cross(omega, [...built.levers[0]]));
    let generalizedPower = 0, surfacePower = 0;
    for (let i = 0; i < 2; i++) {
        surfacePower += forceComponents[i] * dot([...built.axes[i]], surfaceVelocity);
        for (const g of built.rows[i].gradients) if (g.side === 0) generalizedPower += forceComponents[i] * g.value *
            (g.dof % 6 < 3 ? velocity[g.dof % 6] : localOmega[g.dof % 6 - 3]);
    }
    assert.ok(Math.abs(generalizedPower - surfacePower) < 1e-12);
});

test('anisotropic U/V follows physical axial/circumferential directions, with force-preserving warm start', () => {
    const f = fixture({ muU: 0.1, muV: 0.8 });
    const c = Math.SQRT1_2;
    f.contact.tangentU = [c, 0, -c]; f.contact.tangentV = [-c, 0, -c];
    f.contact.tangentLambda.set([0.2, -0.3]);
    const built = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.deepEqual([...built.axes[0]], [1, 0, 0]);
    assert.deepEqual([...built.axes[1]], [0, 0, -1]);
    assert.deepEqual([...built.group.mu], [0.1, 0.8]);
    const oldForce = add(f.contact.tangentU.map(v => v * 0.2), f.contact.tangentV.map(v => v * -0.3));
    const newForce = add([...built.axes[0]].map(v => v * built.rows[0].lambda), [...built.axes[1]].map(v => v * built.rows[1].lambda));
    assert.ok(norm(subtract(oldForce, newForce)) < 1e-12);
    f.constraint.circumferentialFriction = 0.4;
    assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt).group.mu[1], 0.4);
    delete f.constraint.circumferentialFriction; delete f.constraint.torsionalFriction;
    assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt).group.mu[1], 0.1);
});

test('legacy twist cannot add a second friction budget, and zero normal load permits free motion', () => {
    const f = fixture({ normalLambda: 0 });
    for (let i = 0; i < f.inner.count; i++) f.inner.x[i] += 1;
    const before = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    const rows = structuredClone(before.rows);
    f.contact.twistLambda = 200;
    const after = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt);
    assert.deepEqual(after.rows, rows);
    assert.equal(after.diagnostics.requiresLegacyTwistRetirement, true);
    assert.deepEqual([...projectKirchhoffSurfaceFriction([5, -3], 0, after.group.mu).lambda], [0, 0]);
    assert.equal(evaluateKirchhoffSurfaceFriction([0, 0], [100, -100], 0, after.group.mu).residual, 0);
    assert.equal(f.contact.twistLambda, 200, 'building rows must not mutate legacy state');
});

test('fillet and portal require an explicit physical surface point; output storage is reused', () => {
    const f = fixture(), out = {};
    const built = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out);
    const firstRow = built.rows[0], firstGradient = firstRow.gradients[0], point = built.point;
    const again = buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out);
    assert.equal(again.rows[0], firstRow); assert.equal(again.rows[0].gradients[0], firstGradient); assert.equal(again.point, point);
    for (const kind of ['distal-fillet', 'distal-rim', 'sliding-rim']) {
        f.record.kind = kind;
        assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out).supported, false);
        assert.equal(out.rows.length, 0);
        f.record.surfaceContactPoint = [3.5, lumen, 0];
        assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out).supported, true);
        assert.equal(out.rows[0], firstRow);
        delete f.record.surfaceContactPoint;
    }
    f.record.normal = [1, 0, 0];
    assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out).reason, 'axial-tangent-parallel-to-normal');
    f.record.surfaceAxialTangent = [0, 1, 0]; f.record.surfaceContactPoint = [5, lumen, 0];
    assert.equal(buildKirchhoffSurfaceFriction(f.constraint, f.record, dt, out).supported, true);
});

test('shared Coulomb disk spends one budget on combined axial and circumferential slip', () => {
    const candidate = Object.freeze([4, 4]), mu = Object.freeze([0.5, 0.5]);
    const result = projectKirchhoffSurfaceFriction(candidate, 10, mu);
    assert.ok(Math.abs(norm([...result.lambda]) - 5) < 1e-12);
    assert.ok(Math.abs(result.lambda[0] - 5 / Math.sqrt(2)) < 1e-12);
    assert.equal(result.normalLambda, 10);
    assert.deepEqual(candidate, [4, 4]);
    const residual = evaluateKirchhoffSurfaceFriction([4, 4], [0, 0], 10, mu);
    assert.ok(residual.feasibilityResidual > 0.6, 'independent component bounds are not a shared disk');
});

test('ellipse projection satisfies geometric KKT and an independent boundary-search oracle', () => {
    for (const [candidate, normal, mu] of [ [[3, 3], 4, [0.2, 0.8]], [[-5, 0.2], 2, [0.7, 0.1]], [[0.01, -20], 5, [0.003, 1]] ]) {
        const p = projectKirchhoffSurfaceFriction(candidate, normal, mu), a = normal * mu[0], b = normal * mu[1];
        assert.ok(Math.abs(Math.hypot(p.lambda[0] / a, p.lambda[1] / b) - 1) < 1e-11);
        const normalDirection = [p.lambda[0] / (a * a), p.lambda[1] / (b * b)];
        const delta = subtract(candidate, [...p.lambda]);
        assert.ok(Math.abs(delta[0] * normalDirection[1] - delta[1] * normalDirection[0]) < 1e-8);
        assert.ok(dot(delta, normalDirection) >= 0);
        let best = Infinity;
        for (let i = 0; i < 20000; i++) {
            const theta = i * Math.PI * 2 / 20000;
            best = Math.min(best, Math.hypot(candidate[0] - a * Math.cos(theta), candidate[1] - b * Math.sin(theta)));
        }
        assert.ok(p.distance <= best + 1e-10 && best - p.distance < 1e-5);
    }
});

test('fixed-normal cone residual recognizes sticking, maximum-dissipation slip and wrong-direction friction', () => {
    const mu = [0.2, 0.8], normal = 2, displacement = [2, -3];
    const denominator = Math.hypot(mu[0] * displacement[0], mu[1] * displacement[1]);
    const lambda = displacement.map((d, i) => -normal * mu[i] * mu[i] * d / denominator);
    const result = evaluateKirchhoffSurfaceFriction(lambda, displacement, normal, mu, { inverseMobility: 0.7 });
    assert.ok(result.residual < 1e-12);
    assert.ok(Math.abs(result.dissipationGap) < 1e-12);
    assert.ok(result.work < 0);
    assert.equal(evaluateKirchhoffSurfaceFriction([0.1, -0.2], [0, 0], normal, mu).residual, 0);
    assert.ok(evaluateKirchhoffSurfaceFriction(lambda.map(v => -v), displacement, normal, mu).residual > 0.1);
    assert.equal(result.normalLambda, normal);
});

test('zero coefficients and scale changes preserve admissibility without changing normal reaction', () => {
    assert.deepEqual([...projectKirchhoffSurfaceFriction([2, -3], 2, [0, 0.5]).lambda], [0, -1]);
    assert.deepEqual([...projectKirchhoffSurfaceFriction([2, -3], 2, [0, 0]).lambda], [0, 0]);
    assert.equal(evaluateKirchhoffSurfaceFriction([0, 0.5], [100, 0], 2, [0, 0.5]).residual, 0);
    const reference = projectKirchhoffSurfaceFriction([3, -7], 2, [0.4, 0.8]);
    for (const scale of [1e-9, 1, 1e9]) {
        const actual = projectKirchhoffSurfaceFriction([3 * scale, -7 * scale], 2 * scale, [0.4, 0.8]);
        assert.ok(norm(subtract([...actual.lambda].map(v => v / scale), [...reference.lambda])) < 1e-12);
        assert.equal(actual.normalLambda, 2 * scale);
    }
});

test('input validation rejects malformed constraints without renormalizing or modifying physical state', () => {
    const f = fixture();
    assert.throws(() => buildKirchhoffSurfaceFriction(f.constraint, f.record, 0), /dt/);
    f.record.innerWeights = [0.3, 0.6];
    assert.throws(() => buildKirchhoffSurfaceFriction(f.constraint, f.record, dt), /sum to one/);
    f.record.innerWeights = [0.3, 0.7]; f.inner.previousOrientationW[0] = NaN;
    assert.throws(() => buildKirchhoffSurfaceFriction(f.constraint, f.record, dt), /finite/);
    assert.throws(() => projectKirchhoffSurfaceFriction([1, 2], -1, [0.2, 0.3]), /non-negative/);
    assert.throws(() => projectKirchhoffSurfaceFriction([1, NaN], 1, [0.2, 0.3]), /finite/);
    assert.throws(() => projectKirchhoffSurfaceFriction([1, 2], 1, [-0.2, 0.3]), /non-negative/);
});

test('new surface rows close the moment of an actual loaded baseline side contact without modifying the world', () => {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const profile = { radius, mass: 1, linearDamping: 1, angularDamping: 1,
        foldLimitStrength: 0, projectionVelocityRetention: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 2, 10, profile), outer = world.createRod('catheter', 2, 10, { ...profile, mass: 3 });
    for (let i = 0; i < 2; i++) { inner.setNodePosition(i, i * 10, 0.1, 0); inner.velocityX[i] = 6; }
    inner.angularVelocityX[0] = 2;
    const constraint = world.addContainment(inner, outer, { innerRadius: lumen, axialFriction: 0.2,
        torsionalFriction: 0.2, portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    world.stepFixed();
    const record = constraint.kirchhoffContacts.find(r => r.kind === 'side');
    assert.ok(record?.manifoldContact.normalLambda > 0);
    const before = [...record.manifoldContact.tangentLambda, record.manifoldContact.twistLambda, record.manifoldContact.normalLambda];
    const built = buildKirchhoffSurfaceFriction(constraint, record, dt);
    assert.equal(built.supported, true);
    assert.ok(Math.abs(built.rows[0].lambda) > 1e-6);
    for (const row of built.rows) {
        const balance = rowWrench(row, [inner, outer]);
        assert.ok(norm(balance.force) < 1e-12);
        assert.ok(norm(balance.moment) < 1e-12);
    }
    assert.deepEqual([...record.manifoldContact.tangentLambda, record.manifoldContact.twistLambda, record.manifoldContact.normalLambda], before);
});
