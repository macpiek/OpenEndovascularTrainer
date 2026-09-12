import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateKirchhoffLumenSegmentContact } from '../src/physics/kirchhoffLumenContact.js';
import { captureCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';
import { evaluateCompositeJointSurfaceMotion } from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import { createCompositeJointLumenSurfaceWorkspace, evaluateCompositeJointLumenSurface } from '../src/physics/kirchhoffCompositeJointLumenSurface.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeJointSurfacePullback, pullbackCompositeJointSurface, evaluateCompositeJointSurfaceLoads } from '../src/physics/kirchhoffCompositeJointSurfacePullback.js';

const dot = (a, b) => a.reduce((s, x, k) => s + x * b[k], 0), norm = a => Math.hypot(...a);
const add = (a, b) => a.map((x, k) => x + b[k]), sub = (a, b) => a.map((x, k) => x - b[k]), scale = (a, s) => Array.from(a, x => x * s);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = a => scale(a, 1 / norm(a));
const close = (a, b, tolerance = 3e-10) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const same = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((x, k) => close(x, b[k], tolerance)); };
function rotate(p, axis, angle) { const t = unit(axis), c = Math.cos(angle), s = Math.sin(angle); return add(add(scale(p, c), scale(cross(t, p), s)), scale(t, dot(t, p) * (1 - c))); }
function pt(d, a, b) { const axis = cross(a, b), first = cross(axis, d); return add(add(d, first), scale(cross(axis, first), 1 / (1 + dot(a, b)))); }
const names = ['innerStart', 'innerEnd', 'outerStart', 'outerEnd'];
function refresh(input, previous = false) {
    const state = previous ? input.previous : input.current;
    state.contact = evaluateKirchhoffLumenSegmentContact(state.input).side;
    assert.ok(state.contact, 'fixture requires actual original side record');
    if (!previous) input.tools.forEach((t, i) => {
        const f = i === 0 ? state.contact.innerT : state.contact.outerT, L = t.coordinates[1] - t.coordinates[0];
        const map = t.materialMap, old = t.materialPath.previousMap;
        map.dsDt = ((map.sStart - old.sStart) + (map.dsDx - old.dsDx) * L * f) / input.dt;
    });
    return input;
}
function fixture(dt = .02) {
    const old = { innerStart: [-.8, .32, .04], innerEnd: [1.2, .35, .06], outerStart: [-1, -.04, -.03], outerEnd: [1.5, .03, .08],
        innerRadius: .16, lumenRadius: .5, innerMaterialSegmentId: 'wire-segment', outerMaterialSegmentId: 'cat-segment', quadrature: [.43] };
    const current = { ...structuredClone(old), innerStart: [-.78, .34, .05], innerEnd: [1.23, .31, .1], outerStart: [-1.02, -.03, -.02], outerEnd: [1.51, .06, .07] };
    const tools = [0, 1].map(i => {
        const id = i === 0 ? 'wire' : 'catheter', previousPositions = names.slice(i * 2, i * 2 + 2).map(name => old[name].slice());
        const sStart = i ? 100 : 20, dsDx = i ? .8 : 1.3, dsDt = i ? .3 : -.5;
        return { id, edgeId: `${id}:edge0`, edge: 0, materialSegmentId: i ? old.outerMaterialSegmentId : old.innerMaterialSegmentId,
            coordinates: i ? [-3, .2] : [10, 12], positions: names.slice(i * 2, i * 2 + 2).map(name => current[name].slice()), previousPositions,
            reference: captureCompositeReferenceFrames(previousPositions)[0], previousAngle: i ? -.1 : .2, angle: i ? -.5 : .5,
            materialMap: { sStart, dsDx, dsDt }, materialPath: { kind: 'linear-affine-maps', previousEdgeId: `${id}:edge0`, previousMap: { sStart: sStart - dt * dsDt, dsDx } },
            positionRates: i ? [[-.1, .2, .05], [.3, -.1, .2]] : [[.2, .1, -.05], [-.1, .2, .1]], angleRate: i ? -2 : 3 };
    });
    const input = { dt, current: { input: current }, previous: { input: old }, tools };
    refresh(input, true); refresh(input); return input;
}
function perturb(input, column, amount) {
    const t = Math.floor(column / 7), local = column % 7;
    if (local === 6) input.tools[t].angle += amount;
    else {
        const end = Math.floor(local / 3), axis = local % 3;
        input.tools[t].positions[end][axis] += amount; input.current.input[names[t * 2 + end]][axis] += amount;
    }
    return refresh(input);
}
function instantaneous(input, result) {
    const tools = input.tools.map((t, i) => ({ ...t, coordinate: t.coordinates[0] + result.physicalForce.tools[i].fraction * (t.coordinates[1] - t.coordinates[0]) }));
    return { input: { tools, dt: input.dt, rateMode: 'instantaneous', contact: { point: result.currentWitness.point, axes: result.physicalForce.axes } },
        result: evaluateCompositeJointSurfaceMotion({ tools, dt: input.dt, rateMode: 'instantaneous', contact: { point: result.currentWitness.point, axes: result.physicalForce.axes } }) };
}

test('composition preserves actual original records, derives both own feet and exposes the declared virtual midpoint witness', () => {
    const input = fixture(), before = structuredClone(input), out = evaluateCompositeJointLumenSurface(input);
    assert.ok(out.supported); assert.equal(out.queryCount, 0); assert.equal(out.physicalDofCount, 14); assert.equal(out.virtualCommonPoint, true); assert.equal(out.exactCylinderIntersection, false);
    for (const [state, witness] of [[input.current, out.currentWitness], [input.previous, out.previousWitness]]) {
        const t = unit(sub(state.input.innerEnd, state.input.innerStart)), n = Array.from(state.contact.normal), radial = unit(sub(n, scale(t, dot(n, t))));
        const inner = add(state.input.innerStart, scale(sub(state.input.innerEnd, state.input.innerStart), state.contact.innerT));
        const outer = add(state.input.outerStart, scale(sub(state.input.outerEnd, state.input.outerStart), state.contact.outerT));
        same(witness.innerSurface, add(inner, scale(radial, .16)), 2e-15); same(witness.outerSurface, add(outer, scale(n, .5)), 2e-15);
        same(witness.point, scale(add(Array.from(witness.innerSurface), Array.from(witness.outerSurface)), .5), 0);
        close(dot(sub(Array.from(witness.innerSurface), inner), t), 0, 2e-15); close(norm(sub(Array.from(witness.innerSurface), inner)), .16, 2e-15);
        assert.ok(witness.separationNorm > 1e-6);
    }
    same(out.forceMap, instantaneous(input, out).result.forceMap, 2e-14); assert.deepEqual(input, before);
    assert.notEqual(out.motion.tools[1].previousFraction, input.previous.contact.outerT); // current label, not frozen old geometric foot
});

test('complete physical14 G and DB match FD of the actual detector plus surface evaluation on a nonparallel unequal-map/spin fixture', () => {
    const input = fixture(), workspace = createCompositeJointLumenSurfaceWorkspace(), out = evaluateCompositeJointLumenSurface(input, workspace), original = structuredClone(out);
    for (let column = 0; column < 14; column++) {
        const h = 1e-6, plus = perturb(structuredClone(input), column, h), minus = perturb(structuredClone(input), column, -h);
        const a = structuredClone(evaluateCompositeJointLumenSurface(plus, workspace)), b = evaluateCompositeJointLumenSurface(minus, workspace);
        for (let row = 0; row < 2; row++) close((a.increment[row] - b.increment[row]) / (2 * h), original.slipJacobian[row * 14 + column], 3e-8);
        for (let row = 0; row < 28; row++) close((a.forceMap[row] - b.forceMap[row]) / (2 * h), original.DforceMap[row * 14 + column], 3e-8);
    }
    assert.ok(norm(Array.from(original.slipJacobian, (x, k) => x - original.motion.configurationJacobian[k])) > .01);
});

test('actual old/current virtual surface policy is objective under common finite R with unequal axes and reference gauges', () => {
    const input = fixture(.1), axis = [.3, -.4, .8], angle = .8, shift = [.4, -.2, .1];
    input.tools.forEach((t, i) => {
        t.positions = t.previousPositions.map(q => add(rotate(q, axis, angle), shift));
        names.slice(i * 2, i * 2 + 2).forEach((name, end) => { input.current.input[name] = t.positions[end].slice(); });
        const tangent = unit(sub(t.positions[1], t.positions[0])), carried = pt(t.reference.director, t.reference.tangent, tangent), physical = rotate(t.reference.director, axis, angle);
        t.angle = t.previousAngle + Math.atan2(dot(tangent, cross(carried, physical)), dot(carried, physical));
        t.materialPath.previousMap = { sStart: t.materialMap.sStart, dsDx: t.materialMap.dsDx };
    });
    refresh(input); const out = evaluateCompositeJointLumenSurface(input); same(out.increment, [0, 0], 3e-14);
    same(out.currentWitness.point, add(rotate(Array.from(out.previousWitness.point), axis, angle), shift), 3e-15);
    input.tools.forEach((t, i) => { const gamma = i ? -2 * Math.PI - .7 : 2 * Math.PI + .6; t.reference.director = rotate(t.reference.director, t.reference.tangent, gamma); t.angle -= gamma; t.previousAngle -= gamma; });
    same(evaluateCompositeJointLumenSurface(input).increment, out.increment, 4e-14);
});

test('independent material feeds and full own spins survive composition without freezing the current outer label', () => {
    const input = fixture(.01), feeds = [.4, -.2], spins = [3, -2];
    const positions = [[[-1, .34, 0], [1, .34, 0]], [[-1.4, 0, 0], [1.4, 0, 0]]];
    input.tools.forEach((t, i) => {
        t.positions = structuredClone(positions[i]); t.previousPositions = structuredClone(positions[i]); t.reference = captureCompositeReferenceFrames(t.previousPositions)[0];
        names.slice(i * 2, i * 2 + 2).forEach((name, end) => { input.current.input[name] = positions[i][end].slice(); input.previous.input[name] = positions[i][end].slice(); });
        t.angle = t.previousAngle + input.dt * spins[i]; t.angleRate = spins[i]; t.positionRates = [[0, 0, 0], [0, 0, 0]];
        const dxPhysical = norm(sub(t.positions[1], t.positions[0])) / (t.coordinates[1] - t.coordinates[0]);
        const dsDt = -feeds[i] * t.materialMap.dsDx / dxPhysical;
        t.materialPath.previousMap = { sStart: t.materialMap.sStart - input.dt * dsDt, dsDx: t.materialMap.dsDx };
    });
    refresh(input, true); refresh(input); const out = evaluateCompositeJointLumenSurface(input), rate = instantaneous(input, out).result;
    same(Array.from(out.increment, x => x / input.dt), [.6, .16 * 3 - .5 * -2], 4e-12); same(Array.from(out.increment, x => x / input.dt), rate.slipRate, 4e-12);
    input.tools.forEach(t => { t.materialPath.previousMap.sStart = t.materialMap.sStart; t.angle = t.previousAngle + 2 * Math.PI; });
    refresh(input); close(evaluateCompositeJointLumenSurface(input).increment[1], (.16 - .5) * 2 * Math.PI, 3e-15);
});

test('physical virtual power, per-tool net force and correct physical wrench survive common-point geometry composition and joint pullback', () => {
    const input = fixture(), out = evaluateCompositeJointLumenSurface(input), rate = instantaneous(input, out), traction = [.7, -.4];
    const rates = rate.input.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate)), generalized = rates.map((_, j) => dot(Array.from(out.forceMap.slice(2 * j, 2 * j + 2)), traction));
    close(dot(generalized, rates) + dot(traction, rate.result.prescribedSlipRate), dot(traction, rate.result.slipRate), 3e-14);
    const world = add(scale(out.physicalForce.axes[0], traction[0]), scale(out.physicalForce.axes[1], traction[1]));
    input.tools.forEach((t, i) => {
        const force = generalized.slice(i * 7, i * 7 + 7), data = out.physicalForce.tools[i], tangent = Array.from(data.tangent), applied = scale(world, i === 0 ? 1 : -1);
        same(add(force.slice(0, 3), force.slice(3, 6)), applied, 3e-14);
        const connection = Array.from({ length: 6 }, (_, j) => dot(tangent, [data.omegaMap[j], data.omegaMap[7 + j], data.omegaMap[14 + j]]));
        const physical = force.slice(0, 6).map((x, j) => x - force[6] * connection[j]);
        const moment = add(add(cross(t.positions[0], physical.slice(0, 3)), cross(t.positions[1], physical.slice(3, 6))), scale(tangent, force[6]));
        same(moment, cross(Array.from(out.currentWitness.point), applied), 3e-14);
    });
    const layout = createCompositeChainLayout([['wire', 'catheter'], ['wire', 'catheter']]);
    const modes = [0, 1, 2].map(node => ({ node, basis: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], relativeDofs: [node * 3, node * 3 + 1, node * 3 + 2] }));
    const mapping = createCompositeJointSurfacePullback({ layout, modes, tools: out.tools });
    pullbackCompositeJointSurface(out, mapping); assert.ok(mapping.operatorReady); assert.ok(mapping.DforceMapValid);
    same(evaluateCompositeJointSurfaceLoads(traction, mapping).physical, generalized, 0);
});

test('full composed finite rule approaches its instantaneous B plus feed power at dt→0 with a moving actual query', () => {
    const errors = [], powerErrors = [], traction = [.7, -.4];
    for (const dt of [.001, .0001, .00001]) {
        const input = fixture(dt);
        input.tools.forEach((t, i) => {
            t.positions = t.previousPositions.map((q, end) => add(q, scale(t.positionRates[end], dt))); t.angle = t.previousAngle + t.angleRate * dt;
            names.slice(i * 2, i * 2 + 2).forEach((name, end) => { input.current.input[name] = t.positions[end].slice(); });
        });
        refresh(input); const out = evaluateCompositeJointLumenSurface(input);
        const oldRateInput = structuredClone(input); oldRateInput.current = structuredClone(oldRateInput.previous);
        oldRateInput.tools.forEach(t => { t.positions = structuredClone(t.previousPositions); t.angle = t.previousAngle; });
        refresh(oldRateInput); const oldOut = evaluateCompositeJointLumenSurface(oldRateInput), original = instantaneous(oldRateInput, oldOut);
        const rates = original.input.tools.flatMap(t => t.positionRates.flat().concat(t.angleRate));
        const generalized = rates.map((_, j) => dot(Array.from(oldOut.forceMap.slice(2 * j, 2 * j + 2)), traction));
        const originalPower = dot(generalized, rates) + dot(traction, original.result.prescribedSlipRate), finiteRate = Array.from(out.increment, x => x / dt);
        errors.push(Math.max(...sub(finiteRate, Array.from(original.result.slipRate)).map(Math.abs))); powerErrors.push(Math.abs(dot(traction, finiteRate) - originalPower));
    }
    assert.ok(errors[1] < .11 * errors[0] && errors[2] < .11 * errors[1], JSON.stringify(errors)); assert.ok(errors[2] < 1e-5);
    assert.ok(powerErrors[1] < .11 * powerErrors[0] && powerErrors[2] < .11 * powerErrors[1], JSON.stringify(powerErrors));
});

test('stale identities, unsupported previous/hinge geometry and invalid inputs revoke every map before retry without queries or incoming mutations', () => {
    const source = fixture(), before = structuredClone(source), workspace = createCompositeJointLumenSurfaceWorkspace();
    const expected = structuredClone(evaluateCompositeJointLumenSurface(source, workspace));
    const cases = [
        x => { x.current.contact.gap += .01; }, x => { x.previous.contact.kind = 'tip'; },
        x => { x.previous.contact.normal.fill(0); }, x => { x.previous.contact.radialDistance = 0; }, x => { x.tools[0].positions[0][0] += .01; },
        x => { x.tools[1].materialSegmentId = 'wrong'; }, x => { x.previous.input.lumenRadius += .1; },
        x => { x.previous.contact.innerT += .01; }, x => { x.tools[1].coordinate = 999; },
        x => { x.tools[0].materialPath.previousEdgeId = 'different-edge'; },
        x => { x.tools[0].materialPath.previousMap.sStart += 10; refresh(x); },
    ];
    for (const mutate of cases) {
        const bad = structuredClone(source); mutate(bad); assert.throws(() => evaluateCompositeJointLumenSurface(bad, workspace));
        assert.equal(workspace.operatorReady, false); assert.equal(workspace.forceMapValid, false); assert.equal(workspace.slipJacobianValid, false); assert.equal(workspace.DforceMapValid, false);
        for (const name of ['increment', 'forceMap', 'slipJacobian', 'DforceMap', 'currentQueryJacobian']) assert.ok(workspace[name].every(Number.isNaN));
        assert.deepEqual(evaluateCompositeJointLumenSurface(source, workspace), expected);
    }
    let queries = 0; source.current.input.manifold = { upsertContact() { queries++; } }; source.previous.input.manifold = source.current.input.manifold;
    for (let i = 0; i < 3; i++) { evaluateCompositeJointLumenSurface(source, workspace); assert.equal(workspace.queryCount, 0); }
    assert.equal(queries, 0); delete source.current.input.manifold; delete source.previous.input.manifold; assert.deepEqual(source, before);
});

test('zero original gap does not imply coincident cylinder witnesses; undefined projected inner surface direction explicitly rejects', () => {
    const input = fixture(.01), angle = .3, tangent = [Math.cos(angle), Math.sin(angle), 0], center = [0, .34, 0], s = input.current.input.quadrature[0];
    const positions = [[sub(center, scale(tangent, 2 * s)), add(center, scale(tangent, 2 * (1 - s)))], [[-1.4, 0, 0], [1.4, 0, 0]]];
    input.tools.forEach((t, i) => {
        t.positions = structuredClone(positions[i]); t.previousPositions = structuredClone(positions[i]); t.reference = captureCompositeReferenceFrames(t.previousPositions)[0]; t.angle = t.previousAngle;
        t.materialPath.previousMap = { sStart: t.materialMap.sStart, dsDx: t.materialMap.dsDx };
        names.slice(i * 2, i * 2 + 2).forEach((name, end) => { input.current.input[name] = positions[i][end].slice(); input.previous.input[name] = positions[i][end].slice(); });
    });
    refresh(input, true); refresh(input); const workspace = createCompositeJointLumenSurfaceWorkspace(), out = evaluateCompositeJointLumenSurface(input, workspace);
    close(input.current.contact.gap, 0, 2e-16); close(out.currentWitness.separationNorm, .32 * Math.sin(.15), 2e-15);
    assert.ok(out.currentWitness.separationNorm > .047); assert.ok(Math.abs(norm(out.currentWitness.point.slice(1)) - .5) > .003);
    // A tangent parallel to the original normal has no unique inner cylinder
    // radial witness. Both original records remain valid radial detector rows.
    const bad = structuredClone(input), direct = [0, 1, 0];
    bad.tools[0].positions = [sub(center, scale(direct, 2 * s)), add(center, scale(direct, 2 * (1 - s)))];
    bad.tools[0].previousPositions = structuredClone(bad.tools[0].positions); bad.tools[0].reference = { tangent: [0, 1, 0], director: [0, 0, 1] };
    for (const state of [bad.current, bad.previous]) { state.input.innerStart = bad.tools[0].positions[0].slice(); state.input.innerEnd = bad.tools[0].positions[1].slice(); }
    refresh(bad, true); refresh(bad); assert.throws(() => evaluateCompositeJointLumenSurface(bad, workspace), /undefined-projected-inner-radial-witness/);
    assert.equal(workspace.operatorReady, false); assert.ok(workspace.forceMap.every(Number.isNaN));
});

test('frozen endpoint dsDt rates are evaluated at the moving derived foot, preserving current-label G and rejecting stale scalar rates', () => {
    const input = fixture(), i = 1, tool = input.tools[i], L = tool.coordinates[1] - tool.coordinates[0];
    tool.materialPath.previousMap.dsDx -= .003;
    const map = tool.materialMap, old = tool.materialPath.previousMap;
    map.dsDtEnds = [0, 1].map(f => ((map.sStart - old.sStart) + (map.dsDx - old.dsDx) * L * f) / input.dt); delete map.dsDt;
    const out = evaluateCompositeJointLumenSurface(input), f = input.current.contact.outerT;
    close(out.motion.tools[1].ownRelativeSpinIncrement, evaluateCompositeJointLumenSurface(input).motion.tools[1].ownRelativeSpinIncrement, 0);
    const scalar = structuredClone(input); scalar.tools[1].materialMap.dsDt = (1 - f) * map.dsDtEnds[0] + f * map.dsDtEnds[1];
    same(evaluateCompositeJointLumenSurface(scalar).increment, out.increment, 0);
    const h = 1e-6, column = 10;
    const [minus, plus] = [-1, 1].map(sign => {
        const next = structuredClone(input), local = column % 7, end = Math.floor(local / 3), axis = local % 3;
        next.tools[1].positions[end][axis] += sign * h; next.current.input[names[2 + end]][axis] += sign * h;
        next.current.contact = evaluateKirchhoffLumenSegmentContact(next.current.input).side; return next;
    });
    const a = evaluateCompositeJointLumenSurface(plus), b = evaluateCompositeJointLumenSurface(minus);
    for (let row = 0; row < 2; row++) close((a.increment[row] - b.increment[row]) / (2 * h), out.slipJacobian[row * 14 + column], 3e-8);
    scalar.tools[1].materialMap.dsDt += .01; assert.throws(() => evaluateCompositeJointLumenSurface(scalar), /scalar-label-rate-is-stale/);
});

test('value composition is bit-identical for increment/B while full-value-invalid-full reuse revokes all absent derivatives', () => {
    const input = fixture(), before = structuredClone(input), workspace = createCompositeJointLumenSurfaceWorkspace();
    const full = structuredClone(evaluateCompositeJointLumenSurface(input, workspace)), storage = Object.fromEntries(['increment', 'forceMap', 'slipJacobian', 'DforceMap', 'currentQueryJacobian'].map(key => [key, workspace[key]]));
    const value = evaluateCompositeJointLumenSurface({ ...input, order: 'value' }, workspace);
    assert.deepEqual(value.increment, full.increment); assert.deepEqual(value.forceMap, full.forceMap); assert.deepEqual(value.currentWitness, full.currentWitness); assert.deepEqual(value.previousWitness, full.previousWitness);
    assert.equal(value.supported, true); assert.equal(value.incrementValid, true); assert.equal(value.forceMapValid, true); assert.equal(value.operatorReady, false);
    assert.equal(value.slipJacobianValid, false); assert.equal(value.DforceMapValid, false);
    for (const name of ['slipJacobian', 'DforceMap', 'currentQueryJacobian']) assert.ok(value[name].every(Number.isNaN));
    assert.equal(value.motion.jacobian, null); assert.equal(value.physicalForce.derivative, null);
    const layout = createCompositeChainLayout([['wire', 'catheter'], ['wire', 'catheter']]);
    const modes = [0, 1, 2].map(node => ({ node, basis: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], relativeDofs: [node * 3, node * 3 + 1, node * 3 + 2] }));
    const mapping = createCompositeJointSurfacePullback({ layout, modes, tools: value.tools });
    assert.throws(() => pullbackCompositeJointSurface(value, mapping), /slipJacobian validity/);
    pullbackCompositeJointSurface({ tools: value.tools, forceMap: value.forceMap, forceMapValid: value.forceMapValid }, mapping);
    assert.equal(mapping.operatorReady, false); assert.equal(mapping.forceMapValid, true); assert.equal(mapping.DforceMapValid, false);
    assert.ok(evaluateCompositeJointSurfaceLoads([.3, -.8], mapping).valid);
    const bad = structuredClone(input); bad.previous.contact.normal.fill(0);
    assert.throws(() => evaluateCompositeJointLumenSurface({ ...bad, order: 'value' }, workspace));
    assert.equal(workspace.incrementValid, false); assert.equal(workspace.forceMapValid, false); assert.ok(workspace.forceMap.every(Number.isNaN));
    assert.deepEqual(evaluateCompositeJointLumenSurface(input, workspace), full);
    for (const [key, array] of Object.entries(storage)) assert.equal(workspace[key], array);
    assert.deepEqual(input, before);
    assert.throws(() => evaluateCompositeJointLumenSurface({ ...input, order: 'gradient' }, workspace), /full or value/); assert.equal(workspace.operatorReady, false);
});

test('value mode preserves original material-label, geometry identity and branch gates and independent feed/spin changes', () => {
    const input = fixture(), workspace = createCompositeJointLumenSurfaceWorkspace();
    const changes = [
        x => { x.previous.input.lumenRadius += .01; },
        x => { x.current.contact.gap += .01; },
        x => { x.tools[0].materialPath.previousEdgeId = 'unknown-hinge'; },
        x => { x.tools[0].materialPath.previousMap.sStart += 100; refresh(x); },
    ];
    for (const change of changes) {
        const next = structuredClone(input); change(next);
        for (const order of ['full', 'value']) {
            assert.throws(() => evaluateCompositeJointLumenSurface({ ...next, order }, workspace));
            assert.equal(workspace.forceMapValid, false); assert.equal(workspace.slipJacobianValid, false); assert.ok(workspace.forceMap.every(Number.isNaN));
        }
    }
    for (const spin of [0, 2 * Math.PI, -4 * Math.PI]) {
        const next = structuredClone(input); next.tools[0].angle += spin; next.tools[1].angle -= spin;
        next.tools[0].materialPath.previousMap.sStart += .002; next.tools[1].materialPath.previousMap.sStart -= .003; refresh(next);
        const full = structuredClone(evaluateCompositeJointLumenSurface(next, workspace)), value = evaluateCompositeJointLumenSurface({ ...next, order: 'value' }, workspace);
        assert.deepEqual(value.increment, full.increment); assert.deepEqual(value.forceMap, full.forceMap);
    }
});
