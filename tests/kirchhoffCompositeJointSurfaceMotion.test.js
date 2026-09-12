import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeJointSurfaceForceMapWorkspace, evaluateCompositeJointSurfaceForceMap as physicalMap, createCompositeJointSurfaceIncrementWorkspace, evaluateCompositeJointSurfaceIncrement as finiteGeneral, createCompositeJointSurfaceMotionWorkspace, evaluateCompositeJointSurfaceMotion as evaluate, evaluateCompositeJointParallelSurfaceIncrement as finiteStep } from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import { captureCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';
const dot = (a, b) => a.reduce((s, v, k) => s + v * b[k], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => a.map((v, k) => v + b[k]), sub = (a, b) => a.map((v, k) => v - b[k]);
const scale = (a, f) => a.map(v => v * f), unit = a => scale(a, 1 / Math.hypot(...a));
const close = (a, b, eps = 2e-10) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= eps, `${a} != ${b}`);
const same = (a, b, eps) => { assert.equal(a.length, b.length); a.forEach((v, k) => close(v, b[k], eps)); };
function rotate(v, axis, angle) { const n = unit(axis), c = Math.cos(angle), s = Math.sin(angle); return add(add(scale(v, c), scale(cross(n, v), s)), scale(n, dot(n, v) * (1 - c))); }
function tool(id, previous, dt, { st = 0, sx = 1, label = 20, coordinate = 1 } = {}) {
    return { id, edgeId: `${id}:edge`, coordinates: [0, 2], coordinate, positions: structuredClone(previous), previousPositions: structuredClone(previous),
        reference: captureCompositeReferenceFrames(previous)[0], angle: .2, previousAngle: .2, angleRate: 0,
        positionRates: [[0, 0, 0], [0, 0, 0]], materialMap: { sStart: label, dsDx: sx, dsDt: st },
        materialPath: { kind: 'linear-affine-maps', previousEdgeId: `${id}:edge`, previousMap: { sStart: label - dt * st, dsDx: sx } } };
}
function fixture() {
    const dt = .02, w = tool('wire', [[0, 0, 0], [2, .3, .1]], dt, { st: -.5, sx: 1.3, coordinate: .8 }),
        c = tool('catheter', [[0, -.4, .2], [2, -.15, -.1]], dt, { st: .3, sx: .8, label: 100, coordinate: 1.1 });
    w.positions = [[.03, -.01, .02], [2.01, .32, .09]]; c.positions = [[-.02, -.39, .19], [2.04, -.17, -.08]];
    w.positionRates = [[.2, .1, -.05], [-.1, .2, .1]]; c.positionRates = [[-.1, .2, .05], [.3, -.1, .2]];
    w.angle = .5; w.angleRate = 3; c.angle = -.4; c.angleRate = -2;
    return { dt, rateMode: 'instantaneous', contact: { point: [.7, .2, .1], axes: [[1, 0, 0], [0, 0, 1]] }, tools: [w, c] };
}
function change(input, column, amount) {
    const n = 7 * input.tools.length, rates = column >= n, index = column % n, t = input.tools[Math.floor(index / 7)], local = index % 7;
    if (local === 6) t[rates ? 'angleRate' : 'angle'] += amount;
    else t[rates ? 'positionRates' : 'positions'][Math.floor(local / 3)][local % 3] += amount;
}
function straight(dt = .02) {
    const tools = [tool('wire', [[-1, -.25, 0], [1, -.25, 0]], dt), tool('catheter', [[-1, -.5, 0], [1, -.5, 0]], dt, { label: 100 })];
    return { dt, rateMode: 'instantaneous', rotationPath: 'short-contact-frame-own-unwrapped-spins', radii: { inner: .25, outer: .5 },
        contact: { point: [0, 0, 0], axes: [[1, 0, 0], [0, 0, 1]] }, tools };
}

test('each own affine axis retains translation, opposite feed/spin, all configuration/rate Jacobian columns and finite differences', () => {
    const input = fixture(), workspace = createCompositeJointSurfaceMotionWorkspace(2), result = evaluate(input, workspace);
    assert.notDeepEqual(result.tools[0].center, result.tools[1].center); assert.equal(result.finiteStepSlipKnown, false);
    for (let column = 0; column < result.dofCount; column++) {
        const h = 1e-6, plus = structuredClone(input), minus = structuredClone(input); change(plus, column, h); change(minus, column, -h);
        const a = evaluate(plus, workspace), b = evaluate(minus, workspace);
        for (let row = 0; row < 2; row++) close((a.slipRate[row] - b.slipRate[row]) / (2 * h), result.jacobian[row * result.dofCount + column], 2e-7);
        for (let t = 0; t < 2; t++) for (let axis = 0; axis < 3; axis++) close((a.tools[t].omega[axis] - b.tools[t].omega[axis]) / (2 * h), result.tools[t].omegaJacobian[axis * result.dofCount + column], 3e-7);
    }
    result.tools.forEach((r, i) => {
        const t = input.tools[i], f = t.coordinate / 2, qt = t.positionRates[0].map((v, k) => (1 - f) * v + f * t.positionRates[1][k]);
        same(r.velocity, add(qt, scale(sub(t.positions[1], t.positions[0]), r.u / 2)));
    });
});

test('instantaneous force map is exactly work conjugate to rates, with prescribed feed and wall power explicit', () => {
    for (const count of [1, 2]) {
        const input = fixture(); input.tools = input.tools.slice(0, count); if (count === 1) input.wall = { velocity: [.1, -.2, .05] };
        const r = evaluate(input), rates = input.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate)), force = [.3, -.8];
        const generalized = rates.map((_, j) => r.forceMap[2 * j] * force[0] + r.forceMap[2 * j + 1] * force[1]);
        close(dot(generalized, rates) + dot(force, r.prescribedSlipRate), dot(force, r.slipRate));
        const delta = rates.map((_, i) => .13 * Math.sin(i + 1)), h = 1e-6, plus = structuredClone(input), minus = structuredClone(input);
        delta.forEach((v, i) => { change(plus, rates.length + i, h * v); change(minus, rates.length + i, -h * v); });
        close(dot(generalized, delta), dot(force, sub(Array.from(evaluate(plus).slipRate), Array.from(evaluate(minus).slipRate))) / (2 * h), 2e-8);
    }
});

test('common instantaneous rigid motion cancels at the same contact point with different physical lever arms', () => {
    const input = straight(), omega = [.4, -.3, .7], translation = [.2, -.1, .05];
    input.tools.forEach(t => { t.positionRates = t.positions.map(q => add(translation, cross(omega, q))); t.angleRate = dot(omega, t.reference.tangent); });
    const r = evaluate(input); same(r.tools[0].omega, omega); same(r.tools[1].omega, omega);
    same(r.tools[0].surfaceVelocity, translation); same(r.tools[1].surfaceVelocity, translation); same(r.slipRate, [0, 0]);
    assert.notDeepEqual(r.tools[0].lever, r.tools[1].lever);
});

test('own accepted frame gauges and unwrapped angle lifts preserve physical rate maps and independent winding', () => {
    const input = fixture(), original = evaluate(input), changed = structuredClone(input), gauge = [2 * Math.PI + .6, -2 * Math.PI - .4];
    changed.tools.forEach((t, i) => { t.reference.director = rotate(t.reference.director, t.reference.tangent, gauge[i]); t.angle -= gauge[i]; t.previousAngle -= gauge[i]; });
    const r = evaluate(changed); same(r.slipRate, original.slipRate); same(r.jacobian, original.jacobian, 5e-9);
    r.tools.forEach((t, i) => { same(t.omega, original.tools[i].omega); t.directors.forEach((d, j) => same(d, original.tools[i].directors[j])); });
    const spin = straight(1); spin.tools.forEach(t => { t.angle += 2 * Math.PI; t.angleRate = 2 * Math.PI; });
    const full = evaluate(spin); close(full.slipRate[1], -.5 * Math.PI); close(full.tools[0].unwrappedAngleDifference, 2 * Math.PI);
});

test('zero-feed bending follows the actual affine triad derivative rather than a quadratic reconstruction', () => {
    const input = fixture(); input.tools.forEach(t => { t.materialMap.dsDt = 0; t.materialPath.previousMap.sStart = t.materialMap.sStart; });
    const r = evaluate(input), h = 1e-7;
    for (let i = 0; i < 2; i++) {
        const [minus, plus] = [-1, 1].map(sign => { const p = structuredClone(input), t = p.tools[i];
            t.positions.forEach((v, end) => v.forEach((_, k) => v[k] += sign * h * t.positionRates[end][k])); t.angle += sign * h * t.angleRate; return evaluate(p); });
        let omega = [0, 0, 0]; r.tools[i].directors.forEach((axis, j) => omega = add(omega, scale(cross(Array.from(axis), scale(sub(Array.from(plus.tools[i].directors[j]), Array.from(minus.tools[i].directors[j])), 1 / (2 * h))), .5)));
        same(r.tools[i].omega, omega, 1e-7); assert.equal(r.tools[i].spatialOrientationField, 'constant-on-own-open-DER-edge');
    }
});

test('midpoints inside one affine edge are not support seams; real hinge crossings and ambiguous traces require transport', () => {
    const input = straight(), left = structuredClone(input), right = structuredClone(input); left.tools[0].coordinate = 1 - 1e-7; right.tools[0].coordinate = 1 + 1e-7;
    same(evaluate(left).tools[0].directors[0], evaluate(right).tools[0].directors[0], 0);
    const cross = structuredClone(input); cross.tools[0].materialMap.dsDt = -100; cross.tools[0].materialPath.previousMap.sStart += 2;
    assert.throws(() => evaluate(cross), { code: 'surface-material-transport-required' });
    const endpoint = structuredClone(input); endpoint.tools[0].coordinate = 0; assert.throws(() => evaluate(endpoint), { code: 'surface-material-transport-required' });
    endpoint.tools[0].trace = 'right'; endpoint.tools[0].materialPath.previousTrace = 'right'; assert.equal(evaluate(endpoint).tools[0].fraction, 0);
    const wrong = structuredClone(input); wrong.tools[0].materialPath.previousEdgeId = 'other'; assert.throws(() => evaluate(wrong), { code: 'surface-material-transport-required' });
    const missing = structuredClone(input); delete missing.tools[0].angleRate; assert.throws(() => evaluate(missing), /instantaneous angle rate/);
});

test('source/returned histories are owned across workspace reuse and stale own-frame or nonfinite inputs reject', () => {
    const input = fixture(), saved = structuredClone(input), workspace = createCompositeJointSurfaceMotionWorkspace(2), first = evaluate(input, workspace), copy = structuredClone(first);
    const other = structuredClone(input); other.tools[0].angleRate += 5; evaluate(other, workspace); assert.deepEqual(first, copy); assert.deepEqual(input, saved);
    first.tools[0].directors[0][0] = 999; assert.deepEqual(input, saved);
    const wrong = structuredClone(input); wrong.tools[0].reference = structuredClone(wrong.tools[1].reference); assert.throws(() => evaluate(wrong), /own previous physical edge/);
    const noWall = fixture(); noWall.tools.pop(); assert.throws(() => evaluate(noWall), /wall velocity/);
    const badRate = structuredClone(input); badRate.tools[0].materialMap.dsDt += .1; assert.throws(() => evaluate(badRate), /linear affine-map path/);
    assert.throws(() => evaluate({ ...input, rateMode: 'backward-euler' }), /instantaneous rates/);
    const huge = structuredClone(input); huge.tools[0].materialMap.dsDx = 1e308; huge.tools[0].materialPath.previousMap.dsDx = 1e308;
    assert.throws(() => evaluate(huge), /material (label|span)/);
});

function rigidFinite(angle = .7, axis = [.3, -.4, .8]) {
    const input = straight(1), translation = [.4, -.2, .1];
    input.tools.forEach(t => {
        t.positions = t.previousPositions.map(p => add(rotate(p, axis, angle), translation));
        const newT = unit(sub(t.positions[1], t.positions[0])), from = t.reference.tangent, d = t.reference.director, k = cross(from, newT), first = cross(k, d), second = cross(k, first);
        const ref = add(add(d, first), scale(second, 1 / (1 + dot(from, newT)))), physical = rotate(d, axis, angle);
        const beta = Math.atan2(dot(newT, cross(ref, physical)), dot(ref, physical)); t.angle = t.previousAngle + beta;
    });
    input.contact.point = translation; input.contact.axes = input.contact.axes.map(v => rotate(v, axis, angle)); return input;
}

test('finite contact-frame rule cancels common short finite rigid motion with different axes origins/levers and frame gauges', () => {
    const input = rigidFinite(), saved = structuredClone(input), result = finiteStep(input); same(result.increment, [0, 0], 3e-15); assert.ok(Math.abs(result.contactFrameSpinIncrement) > .01);
    input.tools.forEach((t, i) => { const gamma = i ? -1.1 : .6; t.reference.director = rotate(t.reference.director, t.reference.tangent, gamma); t.angle -= gamma; t.previousAngle -= gamma; });
    same(finiteStep(input).increment, result.increment, 5e-15); assert.notDeepEqual(result.currentCenters[0], result.currentCenters[1]);
    assert.equal(result.nonlinearReady, false); assert.equal(result.finiteJacobian, null); assert.deepEqual(saved.tools[0].previousPositions, input.tools[0].previousPositions);
});

test('finite opposite feeds and each own winding match the instantaneous limit, including both+2pi and stationary-wall control', () => {
    const input = straight(.02), feeds = [1.2, -.7], rates = [3, -2];
    input.tools.forEach((t, i) => { const st = -feeds[i]; t.materialMap.dsDt = st; t.materialPath.previousMap.sStart = t.materialMap.sStart - input.dt * st;
        t.angle = t.previousAngle + input.dt * rates[i]; t.angleRate = rates[i]; });
    const r = finiteStep(input), instant = evaluate(input); same(Array.from(r.increment, v => v / input.dt), instant.slipRate, 1e-12);
    close(r.increment[0], input.dt * 1.9); close(r.increment[1], input.dt * (.25 * 3 - .5 * -2));
    const winding = straight(1); winding.tools.forEach(t => { t.angle += 2 * Math.PI; t.angleRate = 2 * Math.PI; });
    close(finiteStep(winding).increment[1], -.5 * Math.PI); // relative winding is zero, actual slip is not
    const wall = structuredClone(input); wall.tools.pop(); wall.wall = { velocity: [0, 0, 0] }; wall.finiteWall = { mode: 'stationary-plane', normal: [0, 1, 0], tangent: [1, 0, 0], point: [0, 0, 0] };
    same(Array.from(finiteStep(wall).increment, v => v / wall.dt), evaluate(wall).slipRate, 1e-12);
});

test('finite BE-center-plus-omega counterexample is nonobjective while the stated contact-frame rule cancels it', () => {
    const input = rigidFinite(Math.PI / 2, [1, 0, 0]);
    input.tools.forEach(t => { t.positionRates = t.positions.map((p, e) => scale(sub(p, t.previousPositions[e]), 1 / input.dt)); t.angleRate = (t.angle - t.previousAngle) / input.dt; });
    const falseSlip = evaluate(input); close(falseSlip.slipRate[1], -.14269908169872414);
    same(finiteStep(input).increment, [0, 0], 2e-15);
    // A principal relative pose increment and surface-position chord also
    // miss a full turn of both distinct material surfaces about fixed axes.
    const both = straight(1); both.tools.forEach(t => { t.angle += 2 * Math.PI; t.angleRate = 2 * Math.PI; });
    const before = straight(1); same(sub(Array.from(evaluate(both).tools[0].directors[0]), Array.from(evaluate(before).tools[0].directors[0])), [0, 0, 0], 1e-14);
    close(both.tools[0].angle - both.tools[0].previousAngle - both.tools[1].angle + both.tools[1].previousAngle, 0);
    close(finiteStep(both).increment[1], -.5 * Math.PI);
});

test('finite control rejects nonparallel/unknown rotation paths and the ambiguous short-contact-normal branch', () => {
    const generic = fixture(); assert.throws(() => finiteStep({ ...generic, rotationPath: 'short-contact-frame-own-unwrapped-spins', radii: { inner: .25, outer: .5 } }), /parallel/);
    assert.throws(() => finiteStep({ ...straight(), rotationPath: undefined }), /path/);
    const turn = rigidFinite(Math.PI, [1, 0, 0]); assert.throws(() => finiteStep(turn), /ambiguous/);
});

test('finite control needs own unwrapped pose history and the contact path, not invented instantaneous angular rates', () => {
    const input = rigidFinite(), expected = finiteStep(input); delete input.rateMode;
    input.tools.forEach(t => { delete t.positionRates; delete t.angleRate; });
    same(finiteStep(input).increment, expected.increment, 0); assert.equal(finiteStep(input).instantaneousRatesRequired, false);
    assert.throws(() => evaluate({ ...input, rateMode: 'instantaneous' }), /instantaneous endpoint rates/);
});

test('finite parallel contact-frame increments have the instantaneous virtual-power limit under common bending with independent feeds and spins', () => {
    const omega = [.4, -.3, .7], translationRate = [.2, -.1, .05], speeds = [.3, -.2], feeds = [.4, -.1], spins = [1.1, -.6], traction = [.7, -.4];
    function prepared(dt, finite) {
        const input = straight(dt), rotation = finite ? Math.hypot(...omega) * dt : 0, shift = finite ? scale(translationRate, dt) : [0, 0, 0];
        input.tools.forEach((t, i) => {
            t.materialMap.dsDx = i ? .8 : 1.2; t.materialMap.dsDt = -t.materialMap.dsDx * feeds[i];
            t.materialPath.previousMap = { sStart: t.materialMap.sStart - dt * t.materialMap.dsDt, dsDx: t.materialMap.dsDx };
            if (finite) {
                t.coordinate = 1 - speeds[i] * dt;
                t.positions = t.previousPositions.map(p => add(rotate(add(p, [speeds[i] * dt, 0, 0]), omega, rotation), shift));
                const newT = unit(sub(t.positions[1], t.positions[0])), k = cross(t.reference.tangent, newT), first = cross(k, t.reference.director), second = cross(k, first);
                const transported = add(add(t.reference.director, first), scale(second, 1 / (1 + dot(t.reference.tangent, newT))));
                const rotated = rotate(t.reference.director, omega, rotation);
                const beta = Math.atan2(dot(newT, cross(transported, rotated)), dot(transported, rotated));
                t.angle = t.previousAngle + beta + spins[i] * dt;
            } else {
                t.positionRates = t.positions.map(p => add(add(translationRate, cross(omega, p)), [speeds[i], 0, 0]));
                t.angleRate = dot(omega, t.reference.tangent) + spins[i];
            }
        });
        input.contact.point = shift; input.contact.axes = input.contact.axes.map(v => rotate(v, omega, rotation)); return input;
    }
    const instantaneousInput = prepared(.01, false), instantaneous = evaluate(instantaneousInput);
    const rates = instantaneousInput.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate));
    const generalizedForce = rates.map((_, j) => traction[0] * instantaneous.forceMap[2 * j] + traction[1] * instantaneous.forceMap[2 * j + 1]);
    const originalPower = dot(generalizedForce, rates) + dot(traction, instantaneous.prescribedSlipRate);
    for (const dt of [.01, .001, .0001]) {
        const finite = finiteStep(prepared(dt, true)), rate = Array.from(finite.increment, v => v / dt);
        same(rate, instantaneous.slipRate, 2e-9); close(dot(traction, rate), originalPower, 2e-9);
    }
});

function sideQuery(tools, previous = false) {
    const centers = tools.map(t => {
        const q = previous ? t.previousPositions : t.positions, f = (t.coordinate - t.coordinates[0]) / (t.coordinates[1] - t.coordinates[0]);
        return add(q[0], scale(sub(q[1], q[0]), f));
    });
    const q = previous ? tools[1].previousPositions : tools[1].positions, tangent = unit(sub(q[1], q[0])), delta = sub(centers[0], centers[1]);
    const normal = unit(sub(delta, scale(tangent, dot(delta, tangent))));
    return { point: add(centers[1], scale(normal, .5)), normal, tangent };
}
function generalFixture() {
    const input = fixture(); input.rotationPath = 'short-contact-frame-own-unwrapped-spins';
    input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: sideQuery(input.tools), previous: sideQuery(input.tools, true) }; return input;
}
function changeFinite(input, column, amount) {
    const n = input.tools.length * 7, count = input.tools.length;
    if (column < n) change(input, column, amount);
    else if (column < n + count) {
        const t = input.tools[column - n]; t.coordinate += amount;
        const map = t.materialMap, old = t.materialPath.previousMap;
        map.dsDt = ((map.sStart - old.sStart) + (map.dsDx - old.dsDx) * (t.coordinate - t.coordinates[0])) / input.dt;
    } else {
        const local = column - n - count, q = input.finiteGeometry[local < 9 ? 'current' : 'previous'];
        q[['point', 'normal', 'tangent'][Math.floor((local % 9) / 3)]][local % 3] += amount;
    }
}
function pt(d, from, to) { const k = cross(from, to), first = cross(k, d); return add(add(d, first), scale(cross(k, first), 1 / (1 + dot(from, to)))); }

test('finite nonparallel chart exposes exact AD for every configuration, moving foot and both geometric query arguments', () => {
    const input = generalFixture(), workspace = createCompositeJointSurfaceIncrementWorkspace(2), r = finiteGeneral(input, workspace);
    assert.equal(r.dofCount, 34); assert.equal(r.nonlinearReady, true); assert.equal(r.derivatives.callerOwnsQueryChainRule, true);
    const saved = structuredClone(r), source = structuredClone(input);
    for (let column = 0; column < r.dofCount; column++) {
        const h = 1e-6, plus = structuredClone(input), minus = structuredClone(input); changeFinite(plus, column, h); changeFinite(minus, column, -h);
        const a = finiteGeneral(plus, workspace), b = finiteGeneral(minus, workspace);
        for (let row = 0; row < 2; row++) close((a.increment[row] - b.increment[row]) / (2 * h), r.jacobian[row * r.dofCount + column], 2e-7);
        for (let row = 0; row < 3; row++) close((a.relativeIncrement[row] - b.relativeIncrement[row]) / (2 * h), r.relativeIncrementJacobian[row * r.dofCount + column], 2e-7);
    }
    assert.deepEqual(r, saved); assert.deepEqual(input, source);
    assert.ok(Array.from(r.queryJacobian).some(x => Math.abs(x) > .01));
    const rate = evaluate(input); assert.ok(Array.from(r.configurationJacobian).some((x, k) => Math.abs(x - input.dt * rate.rateJacobian[k]) > 1e-3));
    assert.equal(r.derivatives.forceMap, 'use-separate-instantaneous-rate-jacobian');
});

test('finite nonparallel contact-frame rule cancels arbitrary common short rigid motion with unequal physical axes/levers', () => {
    const input = generalFixture(), axis = [.3, -.4, .8], angle = .9, shift = [.5, -.3, .1];
    input.tools.forEach(t => {
        t.positions = t.previousPositions.map(q => add(rotate(q, axis, angle), shift)); t.materialMap.dsDt = 0;
        t.materialPath.previousMap = { sStart: t.materialMap.sStart, dsDx: t.materialMap.dsDx };
        const newT = unit(sub(t.positions[1], t.positions[0])), carried = pt(t.reference.director, t.reference.tangent, newT), physical = rotate(t.reference.director, axis, angle);
        t.angle = t.previousAngle + Math.atan2(dot(newT, cross(carried, physical)), dot(carried, physical));
    });
    input.finiteGeometry.previous = sideQuery(input.tools, true);
    const old = input.finiteGeometry.previous;
    input.finiteGeometry.current = { point: add(rotate(old.point, axis, angle), shift), tangent: rotate(old.tangent, axis, angle), normal: rotate(old.normal, axis, angle) };
    const r = finiteGeneral(input); same(r.increment, [0, 0], 3e-14); same(r.relativeIncrement, [0, 0, 0], 3e-14);
    assert.ok(Math.hypot(...cross(input.tools[0].reference.tangent, input.tools[1].reference.tangent)) > .1);
    assert.notDeepEqual(r.tools[0].meanLever, r.tools[1].meanLever);
    input.tools.forEach((t, i) => { const gauge = i ? -2 * Math.PI - .7 : 2 * Math.PI + .3; t.reference.director = rotate(t.reference.director, t.reference.tangent, gauge); t.angle -= gauge; t.previousAngle -= gauge; });
    const gauged = finiteGeneral(input); same(gauged.increment, r.increment, 4e-15); same(gauged.jacobian, r.jacobian, 4e-14);
});

test('finite nonparallel reversal swaps the same material labels and negates each winding, swing and resulting slip', () => {
    const input = generalFixture(); input.tools[0].angle += 2 * Math.PI; input.tools[1].angle -= 4 * Math.PI;
    const forward = finiteGeneral(input), reverse = structuredClone(input);
    reverse.finiteGeometry.current = structuredClone(input.finiteGeometry.previous); reverse.finiteGeometry.previous = structuredClone(input.finiteGeometry.current);
    reverse.tools.forEach((t, i) => {
        const source = input.tools[i], currentT = unit(sub(source.positions[1], source.positions[0]));
        t.reference = { tangent: currentT, director: pt(source.reference.director, source.reference.tangent, currentT) };
        t.positions = structuredClone(source.previousPositions); t.previousPositions = structuredClone(source.positions);
        t.angle = source.previousAngle; t.previousAngle = source.angle;
        t.materialMap = { ...source.materialPath.previousMap, dsDt: -source.materialMap.dsDt };
        t.materialPath.previousMap = { sStart: source.materialMap.sStart, dsDx: source.materialMap.dsDx };
        t.coordinate = t.coordinates[0] + forward.tools[i].previousFraction * (t.coordinates[1] - t.coordinates[0]);
    });
    const backward = finiteGeneral(reverse);
    same(backward.increment, scale(Array.from(forward.increment), -1), 3e-14);
    backward.tools.forEach((r, i) => { close(r.ownRelativeSpinIncrement, -forward.tools[i].ownRelativeSpinIncrement, 3e-14);
        same(r.tangentSwing, scale(Array.from(forward.tools[i].tangentSwing), -1), 3e-14); same(r.meanLever, forward.tools[i].meanLever, 3e-14); });
});

test('general finite rule preserves both own full turns, agrees with the parallel control and differentiates departure from parallelism', () => {
    for (const spins of [[2 * Math.PI, 2 * Math.PI], [2 * Math.PI, -2 * Math.PI], [-2 * Math.PI, 0]]) {
        const input = straight(1); input.tools.forEach((t, i) => { t.angle += spins[i]; });
        input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: { point: [0, 0, 0], normal: [0, 1, 0], tangent: [1, 0, 0] }, previous: { point: [0, 0, 0], normal: [0, 1, 0], tangent: [1, 0, 0] } };
        const r = finiteGeneral(input); same(r.increment, finiteStep(input).increment, 2e-15);
        r.tools.forEach((t, i) => close(t.ownRelativeSpinIncrement, spins[i], 1e-15));
        const h = 1e-6, plus = structuredClone(input), minus = structuredClone(input); plus.tools[0].positions[1][2] += h; minus.tools[0].positions[1][2] -= h;
        for (let row = 0; row < 2; row++) close((finiteGeneral(plus).increment[row] - finiteGeneral(minus).increment[row]) / (2 * h), r.configurationJacobian[row * 14 + 5], 1e-8);
    }
});

function nonparallelLimit(dt) {
    const input = generalFixture(), rateTools = structuredClone(input.tools), footRate = [.17, -.11], slopeRates = [.03, -.02]; input.dt = dt;
    input.tools.forEach((t, i) => {
        t.positions = t.previousPositions.map((q, end) => add(q, scale(t.positionRates[end], dt)));
        t.angle = t.previousAngle + t.angleRate * dt; t.coordinate += footRate[i] * dt;
        const start = t.materialMap.sStart, slope = t.materialMap.dsDx, startRate = i ? .3 : -.5;
        t.materialPath.previousMap = { sStart: start, dsDx: slope };
        t.materialMap = { sStart: start + dt * startRate, dsDx: slope + dt * slopeRates[i], dsDt: startRate + slopeRates[i] * (t.coordinate - t.coordinates[0]) };
        const r = rateTools[i]; r.positions = structuredClone(r.previousPositions); r.angle = r.previousAngle;
        r.materialMap.dsDt = startRate + slopeRates[i] * (r.coordinate - r.coordinates[0]);
        r.materialPath.previousMap = { sStart: start - dt * r.materialMap.dsDt, dsDx: slope };
    });
    const previousTools = structuredClone(input.tools); previousTools.forEach((t, i) => { t.coordinate = rateTools[i].coordinate; });
    input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: sideQuery(input.tools), previous: sideQuery(previousTools, true) };
    const q = input.finiteGeometry.previous;
    const rateInput = { ...input, tools: rateTools, contact: { point: q.point, axes: [q.tangent, cross(q.tangent, q.normal)] } };
    return { input, instantaneous: evaluate(rateInput), rateInput };
}

test('finite nonparallel moving-query increment and work converge to the independent instantaneous rate map with opposite feed and changing metrics', () => {
    const traction = [.7, -.4], errors = [], powerErrors = [];
    for (const dt of [.01, .001, .0001, .00001]) {
        const { input, instantaneous, rateInput } = nonparallelLimit(dt), result = finiteGeneral(input), rates = rateInput.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate));
        const generalized = rates.map((_, j) => dot(traction, Array.from(instantaneous.forceMap.slice(2 * j, 2 * j + 2))));
        const power = dot(generalized, rates) + dot(traction, instantaneous.prescribedSlipRate), finiteRate = Array.from(result.increment, x => x / dt);
        errors.push(Math.max(...finiteRate.map((x, k) => Math.abs(x - instantaneous.slipRate[k])))); powerErrors.push(Math.abs(dot(traction, finiteRate) - power));
    }
    for (let i = 1; i < errors.length; i++) assert.ok(errors[i] < .11 * errors[i - 1], JSON.stringify(errors));
    assert.ok(errors.at(-1) < 1e-5, JSON.stringify(errors)); assert.ok(powerErrors.at(-1) < 1e-5, JSON.stringify(powerErrors));
});

function actualSideQuery(input) {
    const [wire, catheter] = input.tools, wf = (wire.coordinate - wire.coordinates[0]) / (wire.coordinates[1] - wire.coordinates[0]);
    const inner = add(wire.positions[0], scale(sub(wire.positions[1], wire.positions[0]), wf)), edge = sub(catheter.positions[1], catheter.positions[0]);
    const fraction = dot(sub(inner, catheter.positions[0]), edge) / dot(edge, edge);
    assert.ok(fraction > 0 && fraction < 1); catheter.coordinate = catheter.coordinates[0] + fraction * (catheter.coordinates[1] - catheter.coordinates[0]);
    const outer = add(catheter.positions[0], scale(edge, fraction)), normal = unit(sub(inner, outer)), tangent = unit(edge);
    return { point: add(outer, scale(normal, .5)), normal, tangent };
}

test('finite configuration Jacobian includes actual strict-side projection foot, point, normal and tangent chain rules', () => {
    const input = generalFixture(); input.finiteGeometry.current = actualSideQuery(input);
    const result = finiteGeneral(input), delta = input.tools.flatMap((t, i) => Array.from({ length: 7 }, (_, j) => .11 * Math.sin(j + 3 * i + 1))), h = 1e-6;
    const [minus, plus] = [-1, 1].map(sign => { const p = structuredClone(input); delta.forEach((x, j) => change(p, j, sign * h * x)); p.finiteGeometry.current = actualSideQuery(p); return p; });
    const queryDelta = ['point', 'normal', 'tangent'].flatMap(key => scale(sub(plus.finiteGeometry.current[key], minus.finiteGeometry.current[key]), 1 / (2 * h)));
    const direction = [...delta, 0, (plus.tools[1].coordinate - minus.tools[1].coordinate) / (2 * h), ...queryDelta, ...Array(9).fill(0)];
    const a = finiteGeneral(plus), b = finiteGeneral(minus);
    for (let row = 0; row < 2; row++) close(dot(Array.from(result.jacobian.slice(row * 34, (row + 1) * 34)), direction), (a.increment[row] - b.increment[row]) / (2 * h), 2e-8);
});

test('stationary-wall finite increment allows own nonparallel bending, but rejects missing material transport or a moving wall history', () => {
    const input = generalFixture(); input.tools.pop(); input.wall = { velocity: [0, 0, 0] };
    input.finiteGeometry.current = { point: [.7, .2, .1], normal: [0, 1, 0], tangent: [1, 0, 0] }; input.finiteGeometry.previous = structuredClone(input.finiteGeometry.current);
    const result = finiteGeneral(input); assert.equal(result.dofCount, 26); assert.ok(Math.abs(result.increment[0]) > 0);
    const wrong = structuredClone(input); wrong.finiteGeometry.previous.point[0] += .01; assert.throws(() => finiteGeneral(wrong), /stationary identical/);
    const crossed = structuredClone(input); crossed.tools[0].materialMap.dsDt = 100; crossed.tools[0].materialPath.previousMap.sStart -= input.dt * 100; assert.throws(() => finiteGeneral(crossed), { code: 'surface-material-transport-required' });
    const turn = generalFixture(); turn.finiteGeometry.current.normal = scale(turn.finiteGeometry.previous.normal, -1); turn.finiteGeometry.current.tangent = turn.finiteGeometry.previous.tangent; assert.throws(() => finiteGeneral(turn), /short chart/);
    const missing = generalFixture(); delete missing.rotationPath; assert.throws(() => finiteGeneral(missing), /path/);
});

test('finite swing integral matches independent path quadrature and is differentiable at the removable parallel singularity and series join', () => {
    for (const angle of [0, 1e-6, Math.acos(.999) - 1e-8, Math.acos(.999) + 1e-8, .4]) {
        const input = straight(.1), axis = [0, 0, 1]; input.tools[0].positions = input.tools[0].previousPositions.map(p => rotate(p, axis, angle));
        input.tools[0].angle += .8;
        input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: { point: [0, 0, 0], normal: [0, 1, 0], tangent: [1, 0, 0] }, previous: { point: [0, 0, 0], normal: [0, 1, 0], tangent: [1, 0, 0] } };
        const r = finiteGeneral(input), wire = r.tools[0], steps = 1000;
        let average = [0, 0, 0];
        // Independent Simpson integral of Rodrigues' actual tangent path in
        // world coordinates; C=(y,z,x) simply permutes the result.
        for (let j = 0; j <= steps; j++) average = add(average, scale(rotate([1, 0, 0], axis, angle * j / steps), (j === 0 || j === steps ? 1 : j % 2 ? 4 : 2) / (3 * steps)));
        same(wire.meanTangent, [average[1], average[2], average[0]], 4e-14);
        const h = 1e-7, plus = structuredClone(input), minus = structuredClone(input); plus.tools[0].positions[1][1] += h; minus.tools[0].positions[1][1] -= h;
        for (let row = 0; row < 2; row++) close((finiteGeneral(plus).increment[row] - finiteGeneral(minus).increment[row]) / (2 * h), r.configurationJacobian[row * 14 + 4], 2e-8);
    }
});

test('zero-feed nonparallel bending has the same virtual-power limit, including one tool against its stationary wall', () => {
    for (const count of [1, 2]) {
        const errors = [];
        for (const dt of [.001, .0001, .00001]) {
            const { input, rateInput } = nonparallelLimit(dt); input.tools = input.tools.slice(0, count); rateInput.tools = rateInput.tools.slice(0, count);
            for (const t of [...input.tools, ...rateInput.tools]) { t.materialMap.dsDx = t.materialPath.previousMap.dsDx; t.materialMap.sStart = t.materialPath.previousMap.sStart; t.materialMap.dsDt = 0; }
            if (count === 1) {
                input.wall = { velocity: [0, 0, 0] }; rateInput.wall = { velocity: [0, 0, 0] };
                input.finiteGeometry.current = structuredClone(input.finiteGeometry.previous);
            }
            const rate = evaluate(rateInput), r = finiteGeneral(input);
            errors.push(Math.max(...Array.from(r.increment, (x, j) => Math.abs(x / dt - rate.slipRate[j]))));
        }
        assert.ok(errors[1] < .11 * errors[0] && errors[2] < .11 * errors[1], JSON.stringify(errors)); assert.ok(errors[2] < 1e-5);
    }
});

function forceFixture(count = 2) {
    const input = generalFixture(); input.forceGeometry = { kind: 'explicit-affine-side-query', ...input.finiteGeometry.current };
    input.tools = input.tools.slice(0, count); return input;
}
function changeForce(input, column, amount) {
    const n = input.tools.length * 7, count = input.tools.length;
    if (column < n) change(input, column, amount);
    else if (column < n + count) input.tools[column - n].coordinate += amount;
    else { const local = column - n - count; input.forceGeometry[['point', 'normal', 'tangent'][Math.floor(local / 3)]][local % 3] += amount; }
}
function matchingInstantaneous(input, result) {
    const prepared = structuredClone(input); prepared.contact = { point: Array.from(result.point), axes: result.axes.map(x => Array.from(x)) };
    if (prepared.tools.length === 1) prepared.wall = { velocity: [.1, -.2, .05] };
    return { input: prepared, result: evaluate(prepared) };
}

test('physical force map agrees with the original instantaneous rate transpose for one and two actual axes without inventing rates', () => {
    for (const count of [1, 2]) {
        const input = forceFixture(count), r = physicalMap(input), existing = matchingInstantaneous(input, r);
        same(r.forceMap, existing.result.forceMap, 2e-14); assert.equal(r.dofCount, 8 * count + 9);
        const noRates = structuredClone(input); delete noRates.dt; delete noRates.wall; delete noRates.rateMode;
        noRates.tools.forEach(t => { delete t.positionRates; delete t.angleRate; delete t.previousAngle; delete t.materialMap; delete t.materialPath; });
        const bare = physicalMap(noRates); same(bare.forceMap, r.forceMap, 0); same(bare.derivative, r.derivative, 0);
        assert.equal(bare.velocityKnown, false); assert.equal(bare.prescribedFeedAndWallPowerKnown, false); assert.equal(bare.instantaneousRatesRequired, false);
        const traction = [.3, -.8], rates = existing.input.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate));
        const generalized = rates.map((_, j) => dot(Array.from(r.forceMap.slice(2 * j, 2 * j + 2)), traction));
        close(dot(generalized, rates) + dot(traction, existing.result.prescribedSlipRate), dot(traction, existing.result.slipRate), 2e-14);
    }
});

test('physical force-map AD differentiates every configuration and current foot/point/normal/tangent column on the open nonparallel chart', () => {
    for (const count of [1, 2]) {
        const input = forceFixture(count), workspace = createCompositeJointSurfaceForceMapWorkspace(count), r = physicalMap(input, workspace);
        const n = 7 * count, m = r.dofCount, saved = structuredClone(r), source = structuredClone(input);
        for (let column = 0; column < m; column++) {
            const h = 1e-6, plus = structuredClone(input), minus = structuredClone(input); changeForce(plus, column, h); changeForce(minus, column, -h);
            const a = physicalMap(plus, workspace), b = physicalMap(minus, workspace);
            for (let row = 0; row < 2 * n; row++) {
                close((a.forceMap[row] - b.forceMap[row]) / (2 * h), r.derivative[row * m + column], 2e-8);
                close(r.derivative[row * m + column], column < n ? r.configurationDerivative[row * n + column] : r.queryDerivative[row * r.queryDofs + column - n], 0);
            }
        }
        assert.deepEqual(r, saved); assert.deepEqual(input, source);
        const bad = structuredClone(input); bad.tools[0].positions[1] = bad.tools[0].positions[0].slice(); assert.throws(() => physicalMap(bad, workspace), /nonzero tangents/);
        assert.deepEqual(physicalMap(input, workspace), r); assert.deepEqual(input, source);
    }
});

test('physical map preserves each tool net force and complete moment including bending endpoint couples and own spin torque', () => {
    const input = forceFixture(), r = physicalMap(input), traction = [.7, -.4], worldForce = add(scale(Array.from(r.axes[0]), traction[0]), scale(Array.from(r.axes[1]), traction[1]));
    const omega = [.4, -.3, .7], translation = [.2, -.1, .05], rigidInput = structuredClone(input);
    let connectionWitness = 0;
    input.tools.forEach((t, i) => {
        const result = r.tools[i], sign = i === 0 ? 1 : -1, applied = scale(worldForce, sign);
        const generalized = Array.from({ length: 7 }, (_, j) => dot(Array.from(r.forceMap.slice((i * 7 + j) * 2, (i * 7 + j + 1) * 2)), traction));
        const forces = [generalized.slice(0, 3), generalized.slice(3, 6)], spinTorque = generalized[6], tangent = Array.from(result.tangent);
        same(add(...forces), applied, 2e-14);
        const connection = Array.from({ length: 6 }, (_, j) => dot(tangent, [result.omegaMap[j], result.omegaMap[7 + j], result.omegaMap[14 + j]]));
        // A q variation at fixed time-PT theta also changes physical axial
        // orientation. Remove its work from q before decoding spatial forces.
        const physicalForces = forces.map((f, end) => f.map((x, k) => x - spinTorque * connection[3 * end + k]));
        const moment = add(add(cross(t.positions[0], physicalForces[0]), cross(t.positions[1], physicalForces[1])), scale(tangent, spinTorque));
        same(moment, cross(Array.from(r.point), applied), 4e-14);
        close(spinTorque, dot(tangent, cross(Array.from(result.lever), applied)), 2e-14);
        const naive = add(add(cross(t.positions[0], forces[0]), cross(t.positions[1], forces[1])), scale(tangent, spinTorque));
        connectionWitness = Math.max(connectionWitness, Math.hypot(...sub(naive, moment)));
        const rates = t.positions.map(q => add(translation, cross(omega, q))), qRates = rates.flat();
        const axialReferenceRate = dot(connection, qRates), thetaRate = dot(omega, tangent) - axialReferenceRate;
        rigidInput.tools[i].positionRates = rates; rigidInput.tools[i].angleRate = thetaRate;
        close(dot(generalized, [...qRates, thetaRate]), dot(applied, add(translation, cross(omega, Array.from(r.point)))), 4e-14);
        const centerOnly = applied.map(x => x * (1 - result.fraction));
        assert.ok(Math.hypot(...sub(physicalForces[0], centerOnly)) > .01); // bending lever couple retained
    });
    assert.ok(connectionWitness > .001); // naive generalized-force wrench is not the physical spatial wrench
    const instant = matchingInstantaneous(rigidInput, r).result;
    same(sub(Array.from(instant.slipRate), Array.from(instant.prescribedSlipRate)), [0, 0], 2e-14);
});

test('physical-map derivative chains actual strict-side feet/point/normal/tangent into generalized friction-force derivatives', () => {
    const input = forceFixture(); input.forceGeometry = { kind: 'explicit-affine-side-query', ...actualSideQuery(input) };
    const r = physicalMap(input), direction = input.tools.flatMap((t, i) => Array.from({ length: 7 }, (_, j) => .13 * Math.cos(j + 3 * i + 1))), h = 1e-6, traction = [.3, -.8];
    const [minus, plus] = [-1, 1].map(sign => { const p = structuredClone(input); direction.forEach((x, j) => change(p, j, sign * h * x)); p.forceGeometry = { kind: 'explicit-affine-side-query', ...actualSideQuery(p) }; return p; });
    const footDelta = input.tools.map((_, i) => (plus.tools[i].coordinate - minus.tools[i].coordinate) / (2 * h));
    const queryDelta = ['point', 'normal', 'tangent'].flatMap(key => scale(sub(plus.forceGeometry[key], minus.forceGeometry[key]), 1 / (2 * h)));
    const total = [...direction, ...footDelta, ...queryDelta], a = physicalMap(plus), b = physicalMap(minus);
    let frozenQueryError = 0;
    for (let j = 0; j < 14; j++) {
        const forceDifference = dot(traction, sub(Array.from(a.forceMap.slice(j * 2, j * 2 + 2)), Array.from(b.forceMap.slice(j * 2, j * 2 + 2)))) / (2 * h);
        const analytical = traction.reduce((sum, f, k) => sum + f * dot(Array.from(r.derivative.slice((j * 2 + k) * 25, (j * 2 + k + 1) * 25)), total), 0);
        const frozen = traction.reduce((sum, f, k) => sum + f * dot(Array.from(r.configurationDerivative.slice((j * 2 + k) * 14, (j * 2 + k + 1) * 14)), direction), 0);
        close(forceDifference, analytical, 2e-8); frozenQueryError = Math.max(frozenQueryError, Math.abs(frozen - analytical));
    }
    assert.ok(frozenQueryError > .005);
});

test('physical-map derivatives respect independent frame gauges, explicit endpoint traces and owned current query validation', () => {
    const input = forceFixture(), r = physicalMap(input), gauge = [2 * Math.PI + .6, -2 * Math.PI - .4];
    input.tools.forEach((t, i) => { t.reference.director = rotate(t.reference.director, t.reference.tangent, gauge[i]); t.angle -= gauge[i]; });
    const changed = physicalMap(input); same(changed.forceMap, r.forceMap, 2e-14); same(changed.derivative, r.derivative, 5e-14);
    const stale = forceFixture(); stale.tools[0].reference = structuredClone(stale.tools[1].reference); assert.throws(() => physicalMap(stale), /own previous physical edge/);
    const endpoint = forceFixture(); endpoint.tools[0].coordinate = 0; assert.throws(() => physicalMap(endpoint), { code: 'surface-material-transport-required' });
    endpoint.tools[0].trace = 'right'; assert.equal(physicalMap(endpoint).tools[0].fraction, 0);
    const missing = forceFixture(); delete missing.forceGeometry; assert.throws(() => physicalMap(missing), /explicit current/);
    const degenerate = forceFixture(); degenerate.forceGeometry.normal = degenerate.forceGeometry.tangent.slice(); assert.throws(() => physicalMap(degenerate), /nondegenerate/);
    const overflow = forceFixture(); overflow.tools[0].previousPositions = [[-1e308, 0, 0], [1e308, 0, 0]]; assert.throws(() => physicalMap(overflow), /force chord length must be finite/);
});

test('finite value mode shares exact scalar algebra with full mode, keeps owned values and publishes no fabricated Jacobians', () => {
    const generic = generalFixture(), spin = straight(1), rigid = rigidFinite(.8, [.3, -.4, .8]);
    spin.tools.forEach((t, i) => { t.angle += i ? -4 * Math.PI : 2 * Math.PI; });
    for (const input of [spin, rigid]) {
        const old = { point: [0, 0, 0], normal: [0, 1, 0], tangent: [1, 0, 0] };
        input.finiteGeometry = { kind: 'explicit-affine-side-queries', current: { point: input.contact.point, tangent: input.contact.axes[0], normal: cross(input.contact.axes[1], input.contact.axes[0]) }, previous: old };
    }
    const wall = generalFixture(); wall.tools.pop(); wall.wall = { velocity: [0, 0, 0] }; wall.finiteGeometry.current = structuredClone(wall.finiteGeometry.previous);
    for (const input of [generic, spin, rigid, wall]) {
        const workspace = createCompositeJointSurfaceIncrementWorkspace(input.tools.length), original = finiteGeneral(input, workspace), before = structuredClone(original), source = structuredClone(input);
        const arenaStorage = workspace.value.data, fullStorage = workspace.first.data;
        const value = finiteGeneral({ ...input, order: 'value' }, workspace);
        assert.deepEqual(value.increment, original.increment); assert.deepEqual(value.relativeIncrement, original.relativeIncrement); assert.deepEqual(value.tools, original.tools);
        for (const key of ['configurationJacobian', 'queryJacobian', 'jacobian', 'relativeIncrementJacobian']) assert.equal(value[key], null);
        assert.equal(value.incrementValid, true); assert.equal(value.jacobianValid, false); assert.equal(value.nonlinearReady, false); assert.equal(value.dofCount, original.dofCount);
        const bad = structuredClone(input); bad.rotationPath = 'unknown'; assert.throws(() => finiteGeneral({ ...bad, order: 'value' }, workspace), /path/);
        assert.deepEqual(finiteGeneral(input, workspace), original); assert.deepEqual(original, before); assert.deepEqual(input, source);
        assert.equal(workspace.value.data, arenaStorage); assert.equal(workspace.first.data, fullStorage);
    }
    assert.throws(() => finiteGeneral({ ...generic, order: 'gradient' }), /full or value/);
});

test('physical B value mode uses scalar explicit omega, is bit-identical for one/two tools and omits DB', () => {
    for (const count of [1, 2]) {
        const input = forceFixture(count), workspace = createCompositeJointSurfaceForceMapWorkspace(count), original = physicalMap(input, workspace), before = structuredClone(original);
        const inputBefore = structuredClone(input), firstStorage = workspace.first.data, valueStorage = workspace.valueOutput.data;
        const value = physicalMap({ ...input, order: 'value' }, workspace);
        assert.deepEqual(value.forceMap, original.forceMap); assert.deepEqual(value.tools, original.tools); assert.deepEqual(value.axes, original.axes);
        assert.equal(value.forceMapValid, true); assert.equal(value.derivativeValid, false); assert.equal(value.configurationDerivativeValid, false); assert.equal(value.queryDerivativeValid, false);
        for (const key of ['configurationDerivative', 'queryDerivative', 'derivative']) assert.equal(value[key], null);
        const bad = structuredClone(input); bad.tools[0].positions[1] = bad.tools[0].positions[0].slice(); assert.throws(() => physicalMap({ ...bad, order: 'value' }, workspace), /nonzero tangents/);
        assert.deepEqual(physicalMap(input, workspace), original); assert.deepEqual(original, before); assert.deepEqual(input, inputBefore);
        assert.equal(workspace.first.data, firstStorage); assert.equal(workspace.valueOutput.data, valueStorage);
    }
    assert.throws(() => physicalMap({ ...forceFixture(), order: 'gradient' }), /full or value/);
});

// Frozen triad/Hessian oracle, source SHA-256:
// ec36b610557e1f47938be9f107bde40710a46f9dbce0a8dcb5bc368510b48c9f
// Keep this old algebra independent of the production explicit-omega path.
const frozenForceMapOracle = (() => {
const finite = (v, name) => { if (!Number.isFinite(v)) throw new TypeError(`${name} must be finite`); return v; };
const vector = (v, n, name) => {
    if (!v || v.length !== n) throw new TypeError(`${name} requires ${n} entries`);
    return Array.from(v, x => finite(x, name));
};
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);

// Local first/second forward derivatives of the AFFINE physical edge frame.
// Copied arena infrastructure only; no quadratic reconstruction is used.
// Fixed arenas can be reused; no numerical differentiation enters evaluation.
function arena(dimension, second, capacity = 4096) {
    const stride = 1 + dimension + (second ? dimension * dimension : 0), data = new Float64Array(stride * capacity);
    let cursor = 0;
    const alloc = () => {
        const at = cursor; cursor += stride;
        if (cursor > data.length) throw new RangeError('Surface derivative arena capacity exceeded');
        data.fill(0, at, at + stride); return at;
    };
    const constant = value => { const r = alloc(); data[r] = value; return r; };
    const unary = (a, value, first, secondDerivative = 0) => {
        const r = constant(value);
        for (let i = 0; i < dimension; i++) data[r + 1 + i] = first * data[a + 1 + i];
        if (second) for (let i = 0; i < dimension; i++) for (let j = 0; j < dimension; j++) {
            const h = 1 + dimension + i * dimension + j;
            data[r + h] = first * data[a + h] + secondDerivative * data[a + 1 + i] * data[a + 1 + j];
        }
        return r;
    };
    const binary = (a, b, value, da, db, daa = 0, dab = 0, dbb = 0) => {
        const r = constant(value);
        for (let i = 0; i < dimension; i++) data[r + 1 + i] = da * data[a + 1 + i] + db * data[b + 1 + i];
        if (second) for (let i = 0; i < dimension; i++) for (let j = 0; j < dimension; j++) {
            const h = 1 + dimension + i * dimension + j, ai = data[a + 1 + i], aj = data[a + 1 + j], bi = data[b + 1 + i], bj = data[b + 1 + j];
            data[r + h] = da * data[a + h] + db * data[b + h] + daa * ai * aj + dab * (ai * bj + bi * aj) + dbb * bi * bj;
        }
        return r;
    };
    const api = { data, dimension, stride, reset() { cursor = 0; }, used: () => cursor / stride, constant,
        variable(value, index) { const r = constant(value); data[r + 1 + index] = 1; return r; },
        add: (a, b) => binary(a, b, data[a] + data[b], 1, 1),
        sub: (a, b) => binary(a, b, data[a] - data[b], 1, -1),
        mul: (a, b) => binary(a, b, data[a] * data[b], data[b], data[a], 0, 1),
        scale: (a, scale) => unary(a, data[a] * scale, scale),
        reciprocal(a) { const v = data[a]; if (v === 0) throw new RangeError('Zero surface derivative denominator'); return unary(a, 1 / v, -1 / (v * v), 2 / (v * v * v)); },
        sqrt(a) { const v = Math.sqrt(data[a]); if (!(v > 0)) throw new RangeError('Surface reconstruction requires nonzero tangents'); return unary(a, v, .5 / v, -.25 / (v * v * v)); },
        sin: a => unary(a, Math.sin(data[a]), Math.cos(data[a]), -Math.sin(data[a])),
        cos: a => unary(a, Math.cos(data[a]), -Math.sin(data[a]), -Math.cos(data[a])),
        atan2(y, x) {
            const xv = data[x], yv = data[y], d = xv * xv + yv * yv;
            if (!(d > 0)) throw new RangeError('Unresolved surface reference connection');
            return binary(y, x, Math.atan2(yv, xv), xv / d, -yv / d, -2 * xv * yv / (d * d), (yv * yv - xv * xv) / (d * d), 2 * xv * yv / (d * d));
        },
        finite() { for (let i = 0; i < cursor; i++) if (!Number.isFinite(data[i])) return false; return true; }
    };
    return api;
}
function constantArena(capacity = 4096) {
    const out = arena(0, false, capacity);
    // The ordinary variable writer cannot be used with dimension zero:
    // writing its derivative entry would overwrite an adjacent scalar node.
    out.variable = value => out.constant(value);
    return out;
}
function evaluationOrder(input) {
    const order = input.order ?? 'full';
    if (order !== 'full' && order !== 'value') throw new RangeError('Surface order must be full or value');
    return order;
}
function vectors(a) {
    const add = (u, v) => u.map((x, i) => a.add(x, v[i]));
    const sub = (u, v) => u.map((x, i) => a.sub(x, v[i]));
    const times = (u, scalar) => u.map(x => a.mul(x, scalar));
    const dot = (u, v) => u.reduce((sum, x, i) => a.add(sum, a.mul(x, v[i])), a.constant(0));
    const cross = (u, v) => [a.sub(a.mul(u[1], v[2]), a.mul(u[2], v[1])), a.sub(a.mul(u[2], v[0]), a.mul(u[0], v[2])), a.sub(a.mul(u[0], v[1]), a.mul(u[1], v[0]))];
    const unit = v => times(v, a.reciprocal(a.sqrt(dot(v, v))));
    const transport = (v, from, to) => {
        const denominator = a.add(a.constant(1), dot(from, to));
        if (!(a.data[denominator] > 1e-10)) throw new RangeError('Antiparallel surface tangents require another reconstruction chart');
        const axis = cross(from, to), first = cross(axis, v), second = cross(axis, first);
        return add(add(v, first), times(second, a.reciprocal(denominator)));
    };
    return { add, sub, times, dot, cross, unit, transport };
}

function scaleUnit(value) {
    const length = norm(value); if (!(length > 1e-12)) throw new RangeError('A nondegenerate affine tangent is required');
    return value.map(v => v / length);
}
function unsupported(reason, details) {
    const error = new RangeError(reason); error.code = 'surface-material-transport-required'; error.requiredTransport = details; throw error;
}

function createCompositeJointSurfaceForceMapWorkspace(toolCount = 2) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One or two actual surface tools are required');
    const configurationDofs = 7 * toolCount, queryDofs = toolCount + 9;
    return { toolCount, configurationDofs, queryDofs, dofCount: configurationDofs + queryDofs,
        second: arena(configurationDofs, true, 2048), first: arena(configurationDofs + queryDofs, false, 4096),
        firstOnly: arena(configurationDofs, false, 2048), valueOutput: constantArena(4096) };
}

/** Physical instantaneous force map B and its configuration/query derivative.
 * No endpoint/angular/wall rates are inputs: B maps a tangential traction to
 * the work-conjugate generalized forces for rates at fixed own coordinates.
 * Prescribed feed and wall velocity supply a SEPARATE power contribution;
 * this operator neither guesses them nor returns a total surface velocity.
 * No material-map, dt or previous angular rate is needed to determine B.
 *
 * forceGeometry={kind:'explicit-affine-side-query',point,normal,tangent} is
 * the caller-owned current objective query. Its normal/tangent are normalized
 * and orthogonalized inside AD, just as in the finite increment operator.
 * Derivative columns: [7T current configurations,T own current coordinates,
 * point.xyz,normal.xyz,tangent.xyz]. Entries are ordered [dof,component,column]
 * with component 0 axial, 1 circumferential. The caller chains ALL moving
 * foot/point/normal/tangent arguments to its detector geometry.
 *
 * A single second-order triad evaluation gives omega_j=.5 sum d cross d_j
 * and D_k omega_j=.5 sum (d_k cross d_j+d cross d_jk). No unit-rate repeated
 * evaluation, numerical differences, finite-slip Jacobian or Hessian of the
 * friction law is substituted for this physical virtual-power derivative.
 * order:'value' retains the triad's required FIRST configuration derivatives
 * to compute physical B, but does not form its Hessian or differentiate B.
 * The same algebra feeds a scalar-only output arena; returned DB is null.
 */
function evaluateCompositeJointSurfaceForceMap(input, workspace = createCompositeJointSurfaceForceMapWorkspace(input.tools?.length)) {
    const order = evaluationOrder(input), full = order === 'full';
    const count = workspace.toolCount, tools = input.tools, geometry = input.forceGeometry;
    if (!Array.isArray(tools) || tools.length !== count || new Set(tools.map(t => t.id)).size !== count)
        throw new TypeError('Distinct physical tools must match the force-map workspace');
    if (geometry?.kind !== 'explicit-affine-side-query') throw new TypeError('An explicit current objective affine-side force query is required');
    const point = vector(geometry.point, 3, 'Force query point'), rawNormal = vector(geometry.normal, 3, 'Force query normal'), rawTangent = vector(geometry.tangent, 3, 'Force query tangent');
    const numericTangent = scaleUnit(rawTangent);
    scaleUnit(rawNormal.map((x, k) => x - dot(rawNormal, numericTangent) * numericTangent[k]));
    const prepared = tools.map(t => {
        if (typeof t.id !== 'string' || !t.id || typeof t.edgeId !== 'string' || !t.edgeId) throw new TypeError('A named physical tool and own edge are required');
        const coordinates = vector(t.coordinates, 2, 'Own force coordinates'), coordinate = finite(t.coordinate, 'Own force contact foot'), length = finite(coordinates[1] - coordinates[0], 'Own force coordinate length');
        if (!(length > 0)) throw new RangeError('Own force coordinates must increase');
        const fraction = (coordinate - coordinates[0]) / length;
        if (fraction < 0 || fraction > 1 || fraction === 0 && t.trace !== 'right' || fraction === 1 && t.trace !== 'left')
            unsupported('Force-map affine endpoint needs its explicit one-sided trace', { toolId: t.id, fraction });
        if (t.positions?.length !== 2 || t.previousPositions?.length !== 2) throw new RangeError('Own current and accepted previous physical endpoints are required');
        const positions = t.positions.map(p => vector(p, 3, 'Own current force position')), previous = t.previousPositions.map(p => vector(p, 3, 'Own accepted force position'));
        const reference = { tangent: vector(t.reference?.tangent, 3, 'Own accepted force tangent'), director: vector(t.reference?.director, 3, 'Own accepted force director') };
        const chord = previous[1].map((x, k) => x - previous[0][k]), oldLength = finite(norm(chord), 'Own accepted force chord length');
        if (!(oldLength > 1e-12)) throw new RangeError('A nondegenerate accepted force tangent is required');
        const oldTangent = chord.map(x => x / oldLength);
        if (Math.abs(dot(reference.tangent, reference.tangent) - 1) > 1e-10 || Math.abs(dot(reference.director, reference.director) - 1) > 1e-10 ||
            Math.abs(dot(reference.tangent, reference.director)) > 1e-10 || norm(reference.tangent.map((x, k) => x - oldTangent[k])) > 1e-10)
            throw new RangeError('Each force-map accepted frame must belong to its own previous physical edge');
        return { id: t.id, edgeId: t.edgeId, coordinates, coordinate, length, fraction, positions, reference, angle: finite(t.angle, 'Own current unwrapped force angle') };
    });
    const a = full ? workspace.second : workspace.firstOnly, b = full ? workspace.first : workspace.valueOutput,
        av = vectors(a), bv = vectors(b), n = workspace.configurationDofs, m = workspace.dofCount;
    a.reset(); b.reset();
    const lift = at => {
        const out = b.constant(a.data[at]);
        if (full) for (let j = 0; j < n; j++) b.data[out + 1 + j] = a.data[at + 1 + j];
        return out;
    };
    const partial = (at, index) => {
        const out = b.constant(a.data[at + 1 + index]);
        if (full) for (let j = 0; j < n; j++) b.data[out + 1 + j] = a.data[at + 1 + n + index * n + j];
        return out;
    };
    const queryStart = n + count, contactPoint = point.map((x, k) => b.variable(x, queryStart + k)),
        normalInput = rawNormal.map((x, k) => b.variable(x, queryStart + 3 + k)),
        tangent = bv.unit(rawTangent.map((x, k) => b.variable(x, queryStart + 6 + k))),
        normal = bv.unit(bv.sub(normalInput, bv.times(tangent, bv.dot(normalInput, tangent)))), axes = [tangent, bv.cross(tangent, normal)];
    const entries = [], responses = prepared.map((tool, index) => {
        const start = 7 * index, q = tool.positions.map((p, end) => p.map((x, k) => a.variable(x, start + 3 * end + k))), theta = a.variable(tool.angle, start + 6);
        const toolTangent = av.unit(av.sub(q[1], q[0])), reference = av.transport(tool.reference.director.map(a.constant), tool.reference.tangent.map(a.constant), toolTangent);
        const perpendicular = av.cross(toolTangent, reference), d1 = av.add(av.times(reference, a.cos(theta)), av.times(perpendicular, a.sin(theta))), triad = [d1, av.cross(toolTangent, d1), toolTangent];
        const lifted = triad.map(d => d.map(lift)), fraction = b.scale(b.sub(b.variable(tool.coordinate, n + index), b.constant(tool.coordinates[0])), 1 / tool.length);
        const center = bv.add(q[0].map(lift), bv.times(av.sub(q[1], q[0]).map(lift), fraction)), lever = bv.sub(contactPoint, center);
        const omegaColumns = [];
        for (let local = 0; local < 7; local++) {
            let omega = [0, 0, 0].map(b.constant);
            for (let d = 0; d < 3; d++) omega = bv.add(omega, bv.cross(lifted[d], triad[d].map(at => partial(at, start + local))).map(at => b.scale(at, .5)));
            const centerRate = [0, 0, 0].map(b.constant);
            if (local < 6) centerRate[local % 3] = local < 3 ? b.sub(b.constant(1), fraction) : fraction;
            const field = bv.add(centerRate, bv.cross(omega, lever)), sign = index === 0 ? 1 : -1;
            for (const axis of axes) entries.push(b.scale(bv.dot(axis, field), sign));
            omegaColumns.push(omega);
        }
        return { tool, center, lever, tangent: lifted[2], omegaColumns };
    });
    if (!a.finite() || !b.finite()) throw new RangeError('Nonfinite physical surface force map/derivative');
    const values = vector => Float64Array.from(vector, at => b.data[at]);
    const derivative = (start, length) => full ? Float64Array.from(entries.flatMap(at => Array.from(b.data.subarray(at + 1 + start, at + 1 + start + length)))) : null;
    return { scope: 'instantaneous-own-affine-surface-force-map', forceMap: values(entries), configurationDofs: n, queryDofs: workspace.queryDofs, dofCount: m,
        configurationDerivative: derivative(0, n), queryDerivative: derivative(n, workspace.queryDofs), derivative: derivative(0, m),
        point: values(contactPoint), axes: axes.map(values), normal: values(normal),
        tools: responses.map(r => ({ id: r.tool.id, edgeId: r.tool.edgeId, fraction: r.tool.fraction, center: values(r.center), lever: values(r.lever), tangent: values(r.tangent),
            omegaMap: Float64Array.from([0, 1, 2].flatMap(k => r.omegaColumns.map(column => b.data[column[k]]))) })),
        instantaneousRatesRequired: false, velocityKnown: false, prescribedFeedAndWallPowerKnown: false, certified: false,
        order, forceMapValid: true, configurationDerivativeValid: full, queryDerivativeValid: full, derivativeValid: full,
        derivatives: { finiteDifferences: false, repeatedUnitRateEvaluations: false, callerOwnsQueryChainRule: true,
            columns: 'per-tool(q0.xyz,q1.xyz,theta);per-tool(coordinate);current(point,normal,tangent)',
            storage: 'dof,tangent-component,derivative-column', forceMap: 'transpose-of-instantaneous-rate-jacobian' } };
}

    return { create: createCompositeJointSurfaceForceMapWorkspace, evaluate: evaluateCompositeJointSurfaceForceMap };
})();

function variedForceFixture(count, sample) {
    let state = 0x4a31 + sample * 127 + count * 947;
    const random = () => { state = (Math.imul(1664525, state) + 1013904223) >>> 0; return state / 0x100000000; };
    const vec = () => Array.from({ length: 3 }, () => 2 * random() - 1);
    const tools = Array.from({ length: count }, (_, i) => {
        const oldTangent = unit(vec()), oldStart = vec(), previous = [oldStart, add(oldStart, scale(oldTangent, .8 + 2 * random()))];
        const t = tool(`body-${i}`, previous, .01, { coordinate: .05 + 1.9 * random() });
        let currentTangent = unit(vec());
        if (dot(oldTangent, currentTangent) < -.93) currentTangent = scale(currentTangent, -1);
        if (sample === 0) currentTangent = oldTangent;
        if (sample === 1) currentTangent = unit(add(oldTangent, scale(t.reference.director, 1e-7)));
        if (sample === 2) currentTangent = unit(add(scale(oldTangent, -1), scale(t.reference.director, .1)));
        const start = vec(); t.positions = [start, add(start, scale(currentTangent, .7 + 2 * random()))];
        t.reference.director = rotate(t.reference.director, oldTangent, (i + 1) * .7 + 4 * Math.PI * (sample - 4));
        t.angle = 6 * Math.PI * (sample - 5) + .3 + i;
        return t;
    });
    const tangent = vec(), normal = add(vec(), scale(unit(cross(tangent, [1, .2, -.4])), 2));
    return { tools, forceGeometry: { kind: 'explicit-affine-side-query', point: vec(), normal, tangent } };
}

// Independent dense rotation of a physical triad by the shortest-arc unit
// quaternion; no AD, omega formula or production transport helper is used.
function densePhysicalTriad(tool) {
    const tangent = unit(sub(tool.positions[1], tool.positions[0])), old = tool.reference.tangent,
        quaternion = [1 + dot(old, tangent), ...cross(old, tangent)], qLength = Math.hypot(...quaternion),
        scalar = quaternion[0] / qLength, axis = quaternion.slice(1).map(x => x / qLength);
    const rotateQuaternion = v => add(add(scale(v, scalar * scalar - dot(axis, axis)), scale(axis, 2 * dot(axis, v))), scale(cross(axis, v), 2 * scalar));
    const oldNormal = tool.reference.director, oldBinormal = cross(old, oldNormal),
        c = Math.cos(tool.angle), s = Math.sin(tool.angle);
    return [rotateQuaternion(add(scale(oldNormal, c), scale(oldBinormal, s))),
        rotateQuaternion(add(scale(oldNormal, -s), scale(oldBinormal, c))), rotateQuaternion(old)];
}

test('explicit omega retains the negative axial time-PT connection on an independent azimuthal rotation witness', () => {
    const input = variedForceFixture(1, 4), t = input.tools[0], tangent = unit([1, 2, 3]);
    t.previousPositions = [[0, 0, 0], [0, 0, 1]]; t.reference = { tangent: [0, 0, 1], director: [1, 0, 0] };
    t.positions = [[0, 0, 0], tangent]; t.angle = 14 * Math.PI + .3;
    const rate = [...[0, 0, 0], ...cross([0, 0, 1], tangent), 0], expected = sub([0, 0, 1], tangent);
    for (const evaluate of [physicalMap, frozenForceMapOracle.evaluate]) {
        const map = evaluate(input).tools[0].omegaMap;
        const omega = [0, 1, 2].map(k => dot(Array.from(map.slice(k * 7, (k + 1) * 7)), rate));
        same(omega, expected, 2e-14); close(dot(omega, tangent), tangent[2] - 1, 2e-14);
        assert.ok(dot(omega, tangent) < -.1); // plus-sign connection would have the opposite sign
    }
});

test('first-order physical B/DB matches frozen triad Hessians across nonparallel frames, gauges, windings and both tool packings', context => {
    let maxB = 0, maxDB = 0, maxOmega = 0;
    for (const count of [1, 2]) {
        const workspace = createCompositeJointSurfaceForceMapWorkspace(count), oracleWorkspace = frozenForceMapOracle.create(count);
        for (let sample = 0; sample < 24; sample++) {
            const input = variedForceFixture(count, sample), expected = frozenForceMapOracle.evaluate(input, oracleWorkspace), actual = physicalMap(input, workspace);
            for (const key of ['forceMap', 'configurationDerivative', 'queryDerivative', 'derivative']) {
                assert.equal(actual[key].length, expected[key].length);
                actual[key].forEach((v, j) => {
                    const error = Math.abs(v - expected[key][j]);
                    assert.ok(error <= 3e-11 * (1 + Math.abs(expected[key][j])), `${count}/${sample}/${key}/${j}: ${v} != ${expected[key][j]}`);
                    if (key === 'forceMap') maxB = Math.max(maxB, error); else maxDB = Math.max(maxDB, error);
                });
            }
            actual.tools.forEach((t, i) => t.omegaMap.forEach((v, j) => {
                maxOmega = Math.max(maxOmega, Math.abs(v - expected.tools[i].omegaMap[j]));
                close(v, expected.tools[i].omegaMap[j], 3e-11 * (1 + Math.abs(v)));
            }));
            const scalar = physicalMap({ ...input, order: 'value' }, workspace);
            assert.deepEqual(scalar.forceMap, actual.forceMap); assert.deepEqual(scalar.tools, actual.tools);
            // Circular material surface: current theta and independent director
            // gauge cancel analytically, including every derivative column.
            const changed = structuredClone(input);
            changed.tools.forEach((t, i) => { t.angle += (i + 1) * 38 * Math.PI + .31; t.reference.director = rotate(t.reference.director, t.reference.tangent, 2.7 - i); });
            const gauged = physicalMap(changed, workspace);
            assert.deepEqual(gauged.forceMap, actual.forceMap); assert.deepEqual(gauged.derivative, actual.derivative);
            for (let i = 0; i < count; i++) for (let entry = 0; entry < actual.forceMap.length; entry++)
                assert.ok(actual.derivative[entry * actual.dofCount + i * 7 + 6] === 0);
        }
    }
    context.diagnostic(JSON.stringify({ samples: 48, maxB, maxDB, maxOmega }));
});

test('explicit omega and physical power match dense infinitesimal rotations of every actual triad column', context => {
    let maxOmega = 0, maxPower = 0;
    for (let sample = 3; sample < 15; sample++) {
        const input = variedForceFixture(2, sample), result = physicalMap(input), h = 1e-6, traction = [.3, -.8];
        input.tools.forEach((t, i) => {
            const triad = densePhysicalTriad(t), lever = Array.from(result.tools[i].lever), materialLever = triad.map(d => dot(d, lever)),
                force = scale(add(scale(Array.from(result.axes[0]), traction[0]), scale(Array.from(result.axes[1]), traction[1])), i === 0 ? 1 : -1);
            for (let local = 0; local < 7; local++) {
                const plus = structuredClone(input), minus = structuredClone(input);
                change(plus, 7 * i + local, h); change(minus, 7 * i + local, -h);
                const [p, m] = [plus, minus].map(v => densePhysicalTriad(v.tools[i]));
                const omega = triad.reduce((sum, d, j) => add(sum, scale(cross(d, scale(sub(p[j], m[j]), 1 / (2 * h))), .5)), [0, 0, 0]);
                const expected = [0, 1, 2].map(k => result.tools[i].omegaMap[k * 7 + local]);
                omega.forEach((x, k) => { maxOmega = Math.max(maxOmega, Math.abs(x - expected[k])); close(x, expected[k], 4e-8); });
                const surfacePoint = (v, frame) => {
                    const positions = v.tools[i].positions, fraction = result.tools[i].fraction;
                    return frame.reduce((point, d, k) => add(point, scale(d, materialLever[k])), add(scale(positions[0], 1 - fraction), scale(positions[1], fraction)));
                };
                const densePower = dot(force, scale(sub(surfacePoint(plus, p), surfacePoint(minus, m)), 1 / (2 * h))),
                    physicalPower = dot(traction, Array.from(result.forceMap.slice((7 * i + local) * 2, (7 * i + local) * 2 + 2)));
                maxPower = Math.max(maxPower, Math.abs(densePower - physicalPower)); close(densePower, physicalPower, 7e-8);
            }
        });
    }
    context.diagnostic(JSON.stringify({ samples: 12, columns: 168, maxOmega, maxPower }));
});

test('optimized full/value force map preserves frozen input validation and workspace recovery', () => {
    const invalid = [
        x => { x.tools[0].angle = Infinity; }, x => { delete x.tools[0].angle; },
        x => { delete x.tools[0].reference.director; }, x => { x.tools[0].reference.director = [1, 2, 3]; },
        x => { x.tools[0].reference.tangent = scale(x.tools[0].reference.tangent, 1.1); },
        x => { x.tools[0].reference = structuredClone(x.tools[1].reference); },
        x => { delete x.tools[0].previousPositions; }, x => { x.tools[0].previousPositions[1] = x.tools[0].previousPositions[0].slice(); },
        x => { x.tools[0].previousPositions[1][0] = NaN; }, x => { x.tools[0].positions[1] = x.tools[0].positions[0].slice(); },
        x => { const t = x.tools[0]; t.positions[1] = sub(t.positions[0], t.reference.tangent); },
        x => { x.tools[0].positions[1][0] = Infinity; }, x => { delete x.tools[0].edgeId; },
        x => { x.tools[0].coordinates = [1, 1]; }, x => { x.tools[0].coordinate = NaN; },
        x => { x.tools[0].coordinate = -1; }, x => { x.tools[0].coordinate = 0; },
        x => { x.tools[0].coordinate = 2; x.tools[0].trace = 'right'; },
        x => { x.tools[0].id = x.tools[1].id; }, x => { x.tools.pop(); },
        x => { delete x.forceGeometry; }, x => { x.forceGeometry.kind = 'unknown'; },
        x => { x.forceGeometry.point[0] = NaN; }, x => { x.forceGeometry.normal = [0, 0, 0]; },
        x => { x.forceGeometry.tangent = [0, 0, 0]; }, x => { x.order = 'unknown'; }
    ];
    const captureError = fn => {
        try { fn(); } catch (error) { return [error.constructor.name, error.message, error.code, error.requiredTransport]; }
        assert.fail('Expected input rejection');
    };
    const workspace = createCompositeJointSurfaceForceMapWorkspace(2), oracleWorkspace = frozenForceMapOracle.create(2), original = physicalMap(forceFixture(), workspace);
    for (const order of ['full', 'value']) for (const mutate of invalid) {
        const input = { ...forceFixture(), order }; mutate(input);
        assert.deepEqual(captureError(() => physicalMap(input, workspace)), captureError(() => frozenForceMapOracle.evaluate(input, oracleWorkspace)));
        assert.deepEqual(physicalMap(forceFixture(), workspace), original);
    }
    for (const [coordinate, trace] of [[0, 'right'], [2, 'left']]) {
        const input = forceFixture(); input.tools[0].coordinate = coordinate; input.tools[0].trace = trace;
        same(physicalMap(input).forceMap, frozenForceMapOracle.evaluate(input).forceMap, 2e-14);
    }
    assert.equal(workspace.second, undefined); // No quadratic derivative storage remains in this provider.
    assert.equal(workspace.firstOnly, undefined);
});
