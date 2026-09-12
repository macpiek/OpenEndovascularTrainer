import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { Quaternion, Vector3 } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffCoupledBoundaryStep } from '../src/physics/kirchhoffCoupledBoundaryRows.js';
import { beginKirchhoffSplitMotion } from '../src/physics/kirchhoffSplitMotion.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion, commitKirchhoffTwoChannelBiasMaterial } from '../src/physics/kirchhoffTwoChannelMotion.js';
import { beginKirchhoffTwoChannelRows, prepareKirchhoffTwoChannelRows, commitKirchhoffTwoChannelRows, measureKirchhoffTwoChannelRows } from '../src/physics/kirchhoffTwoChannelRows.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';

const systemModule = process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT
    ? pathToFileURL(resolve(process.env.OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT, 'src/physics/kirchhoffTwoChannelSystem.js'))
    : new URL('../src/physics/kirchhoffTwoChannelSystem.js', import.meta.url);
const { solveKirchhoffTwoChannelSystem } = await import(systemModule);
const dt = 1 / 120, Q = ['X', 'Y', 'Z', 'W'];
const near = (a, b, reason, tolerance = 1e-8) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance, `${reason}: ${a} vs ${b}`);
const put = (body, q) => Q.forEach((a, i) => { body['orientation' + a][0] = q.toArray()[i]; });
const rotation = vector => new Quaternion().setFromAxisAngle(vector.clone().normalize(), vector.length());
function fixture({ inverseMass = 1, angular = false } = {}) {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const a = world.createRod('bias-release-a', 3, 1, { mass: 1, adaptationCompliance: dt ** 2 * .1,
        kirchhoffBendCompliance: dt ** 2 * .1, kirchhoffTwistCompliance: dt ** 2 * .1 });
    const b = world.createRod('bias-release-b', 2, 1, { mass: 1, adaptationCompliance: dt ** 2 * .1 });
    a.inverseMass.fill(inverseMass); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const i of [1, 2, 3]) body['inverseInertia' + i].fill(0);
    if (angular) for (const [i, v] of [[1, 2], [2, .5], [3, 1.25]]) a['inverseInertia' + i][0] = v;
    a.copyCurrentToPrevious(); b.copyCurrentToPrevious();
    const joint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    beginKirchhoffCoupledBoundaryStep(joint); beginKirchhoffSplitMotion(joint, world); beginKirchhoffTwoChannelMotion(joint);
    const state = beginKirchhoffTwoChannelRows(joint, world);
    return { world, joint, a, b, state };
}
function normal(f, { node = 0, alpha = 0, gap = -.12, gradients, sign = 1 } = {}) {
    return { kind: 'wall', side: 0, node, owner: f.a, component: -1, alpha, strain: 0, lambda: 0, lower: 0, upper: Infinity,
        _splitActualStrain: gap, gradients: gradients ?? [0, 1, 2].map(n => ({ side: 0, dof: n * 6 + 1, value: sign / 3 })) };
}
function solve(f, rows, groups = []) {
    const p = prepareKirchhoffTwoChannelRows(f.joint, rows, groups);
    assert.equal(p.ready, true, JSON.stringify(p.issues));
    const result = solveKirchhoffTwoChannelSystem(f.joint, dt, { additionalRows: rows, groups, channels: p.channels,
        basis: 'individual', tolerance: 1e-9, maximumPosition: 100, maximumAngle: 100, includeSystem: true });
    assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics));
    return result;
}
function apply(f, result, scale = result.scale) {
    result.scale = scale;
    applyKirchhoffTwoChannelPhysicalMotion(f.joint, result); applyKirchhoffCoupledCorrection(f.joint, result);
    commitKirchhoffTwoChannelBiasMaterial(f.joint, result); commitKirchhoffTwoChannelRows(f.joint, result);
}
const bankBeta = (f, node = 0) => f.state.bank.bodies[0].wallLambda[node];
function ownedNormalWrench(f) {
    const total = [f.a, f.b].map(b => new Float64Array(b.count * 6));
    for (const e of [...f.state.entries, ...f.state.releases]) for (const g of e.frozenWorldGradients)
        total[g.side][g.dof] += e.bank[e.slot] * g.value;
    return total;
}
function sameWrench(actual, expected, reason) {
    actual.forEach((a, side) => a.forEach((v, i) => near(v, expected[side][i], reason, 1e-12)));
}
function foot(f, row, weights, t) {
    f.a.wallT[row.node] = t;
    row.gradients = weights.map((value, n) => ({ side: 0, dof: n * 6 + 1, value }));
    row._splitActualStrain = 0;
}

test('native release keeps fractional old beta and applies its whole old-J response at the same common scale', () => {
    const f = fixture(), wall = normal(f), first = solve(f, [wall]); apply(f, first);
    const beta = bankBeta(f); assert.ok(beta > 0);
    const before = f.a.y.slice(), velocity = f.joint._splitMotion.bodies[0].velocityY.slice();
    const rows = [], released = solve(f, rows), index = rows.findIndex(r => r.kind === 'two-channel-bias-release');
    assert.equal(index, 0); assert.equal(bankBeta(f), 0); assert.equal(f.state.releases[0].bank.lambda, beta);
    assert.equal(released.additionalIncrement[index], 0); assert.equal(released.biasAdditionalIncrement[index], -beta);
    const descriptor = released.system.descriptors[released.system.native.additionalOffset + index];
    assert.deepEqual(descriptor.bias, { channel: 'bias-motion', strain: 0, alpha: 0, lambda: beta, lower: 0, upper: 0 });
    for (let n = 0; n < f.a.count; n++) {
        near(released.bias[0].correction[n * 6 + 1], -beta / 3, 'old reaction releases as a rigid translation');
        near(released.physical[0].correction[n * 6 + 1], 0, 'release has no direct physical response');
    }
    apply(f, released, .25);
    assert.equal(f.state.releases[0].bank.lambda, beta * .75);
    const measured = measureKirchhoffTwoChannelRows(f.joint, []);
    assert.equal(measured.pendingReleases.length, 1); near(measured.releasePositionMm, beta * .75 / 3, 'remaining generalized response in mm');
    for (let n = 0; n < f.a.count; n++) {
        near(f.a.y[n], before[n] - beta / 12, 'actual fractional geometry', 1e-7);
        near(f.joint._splitMotion.bodies[0].velocityY[n], velocity[n], 'bias adds no physical impulse');
    }
    // Reusing the combined array replaces only old trailing release rows.
    const remainder = solve(f, rows); assert.equal(rows.length, 1);
    assert.equal(remainder.biasAdditionalIncrement[0], -beta * .75); apply(f, remainder);
    assert.equal(f.state.releases[0].bank.lambda, 0);
    assert.equal(measureKirchhoffTwoChannelRows(f.joint).pendingReleases.length, 0);
});

test('new witness starts with beta zero alongside the retained old force without shifting friction indices', () => {
    const f = fixture(), wall = normal(f); f.a.wallT[0] = .2;
    apply(f, solve(f, [wall])); const old = bankBeta(f);
    f.a.wallT[0] = .8;
    const current = { ...normal(f, { gap: -.01 }), feature: 'different-surface' }, friction = () => ({ strain: 0, alpha: 0, lambda: 0, gradients: [{ side: 0, dof: 0, value: 1 }] });
    const rows = [current, friction(), friction()], group = { type: 'coulomb-disk', rows: [1, 2], radius: 0 };
    const result = solve(f, rows, [group]);
    assert.equal(rows.length, 4); assert.equal(rows[3].kind, 'two-channel-bias-release');
    assert.deepEqual(group.rows, [1, 2]);
    const descriptors = result.system.descriptors.slice(result.system.native.additionalOffset);
    assert.equal(descriptors[0].bias.lambda, 0); assert.equal(descriptors[3].bias.lambda, old);
    assert.equal(descriptors[1].bias, null); assert.equal(descriptors[2].bias, null);
    apply(f, result, .5);
    assert.equal(f.state.releases[0].bank.lambda, old * .5);
    assert.equal(bankBeta(f), result.biasAdditionalIncrement[0] * .5);
    assert.notEqual(f.state.releases[0].bank, f.state.bank.bodies[0].wallLambda);
});

test('rotated angular release preserves old world torque, current anisotropic mobility and full-system force balance', () => {
    const f = fixture({ angular: true }), q0 = rotation(new Vector3(.25, -.32, .19));
    put(f.a, q0); put(f.joint._splitMotion.twoChannel.physicalPose[0], q0);
    const local = new Vector3(.2, -.4, .5), wall = normal(f, { alpha: .1, gap: -1,
        gradients: local.toArray().map((value, i) => ({ side: 0, dof: 3 + i, value })) });
    const originalFrame = new Quaternion(...Q.map(a => f.a['orientation' + a][0])).normalize();
    apply(f, solve(f, [wall])); const beta = bankBeta(f); assert.ok(beta > 0);
    const world = local.clone().applyQuaternion(originalFrame);
    const q = rotation(new Vector3(-.6, .21, .43)); put(f.a, q);
    const currentFrame = new Quaternion(...Q.map(a => f.a['orientation' + a][0])).normalize();
    const expected = world.clone().applyQuaternion(currentFrame.clone().invert());
    const rows = [], r = solve(f, rows), releaseRow = rows[0];
    for (let i = 0; i < 3; i++) near(releaseRow.gradients[i].value, expected.toArray()[i], 'world wrench transported to current local J', 1e-7);
    assert.ok(Math.hypot(...releaseRow.gradients.map((g, i) => g.value - local.toArray()[i])) > .1);
    const measured = measureKirchhoffTwoChannelRows(f.joint, []), expectedResponse = expected.toArray().map((v, i) => -beta * v * f.a['inverseInertia' + (i + 1)][0]);
    for (let i = 0; i < 3; i++) near(measured.releaseCorrection[0][3 + i], expectedResponse[i], 'anisotropic pending angular response', 1e-7);
    near(measured.releaseAngleRad, Math.hypot(...expectedResponse), 'angular gate is radians', 1e-7);
    // Independently sum every row impulse from the frozen native J. The
    // release participates in the same material/contact response, not a clip.
    const native = r.system.native;
    for (let side = 0; side < 2; side++) for (let dof = 0; dof < native.columns[side].length; dof++) {
        const entries = native.columns[side][dof];
        let impulse = 0;
        for (let i = 0; i < entries.length; i += 2) impulse += entries[i + 1] * r.biasIncrement[native.order[entries[i]]];
        near(r.bias[side].correction[dof], impulse * (native.material[side]?.weight[dof] ?? 0), 'full bias force balance', 1e-7);
    }
    const frozen = structuredClone(f.state.releases[0].frozenWorldGradients);
    apply(f, r, .375);
    assert.equal(f.state.releases[0].bank.lambda, beta * .625);
    assert.deepEqual(f.state.releases[0].frozenWorldGradients, frozen, 'commit does not rotate the retained world wrench');
});

test('pending gate uses net generalized response, including cancellation and tiny multipliers with large mobility', () => {
    const f = fixture(), positive = normal(f, { alpha: 1, gap: -1 }), negative = normal(f, { node: 1, alpha: 1, gap: -1, sign: -1 });
    apply(f, solve(f, [positive, negative]));
    const m = measureKirchhoffTwoChannelRows(f.joint, []);
    assert.equal(m.pendingReleases.length, 2); assert.ok(m.pendingReleases.every(r => r.positionMm > .1));
    near(m.releasePositionMm, 0, 'equal opposite pending responses cancel in the full mechanics');
    const g = fixture({ inverseMass: 1e12 }); apply(g, solve(g, [normal(g)]));
    assert.ok(bankBeta(g) > 0 && bankBeta(g) < 1e-10);
    const small = measureKirchhoffTwoChannelRows(g.joint, []);
    assert.equal(small.pendingReleases.length, 1); assert.ok(small.releasePositionMm > .1);
});

test('whole trial rollback restores retirement, old canonical beta, owned arrays and applied receipts exactly', () => {
    const f = fixture(), wall = normal(f); apply(f, solve(f, [wall]));
    const bank = f.state.bank.bodies[0].wallLambda, before = bank.slice(), pose = f.a.y.slice(), state = f.state;
    const whole = captureKirchhoffCoupledTrialState(f.joint, { world: f.world, reusePropertyLayout: true, frozenFrictionBatches: true });
    const rows = [], r = solve(f, rows), retired = f.state.releases[0], retained = retired.bank, old = retained.lambda;
    const trial = captureKirchhoffCoupledTrialState(f.joint, { world: f.world, reusePropertyLayout: true, frozenFrictionBatches: true });
    apply(f, r, .25); restoreKirchhoffCoupledTrialState(trial);
    assert.equal(f.state.releases[0], retired); assert.equal(retired.bank, retained); assert.equal(retained.lambda, old);
    r.scale = .5; apply(f, r, .5); assert.equal(retained.lambda, old * .5);
    restoreKirchhoffCoupledTrialState(whole);
    assert.equal(f.joint._splitMotion.twoChannel.rows, state); assert.equal(state.bank.bodies[0].wallLambda, bank);
    assert.deepEqual(bank, before); assert.deepEqual(f.a.y, pose); assert.equal(state.releases.length, 0);
});

test('continuous transports telescope exactly through one reusable difference carrier without moving pose or velocity', () => {
    const f = fixture(), wall = normal(f); apply(f, solve(f, [wall]));
    const beta = bankBeta(f), force = ownedNormalWrench(f), pose = f.a.y.slice(), v = f.joint._splitMotion.bodies[0].velocityY.slice();
    foot(f, wall, [.5, .5, 0], .25);
    const first = measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(first.pendingReleases.length, 1); assert.equal(first.pendingReleases[0].mode, 'difference');
    assert.equal(first.pendingReleases[0].carrier, 1); assert.equal(bankBeta(f), beta);
    const carrier = f.state.releases[0], id = carrier.id;
    sameWrench(ownedNormalWrench(f), force, 'canonical plus difference preserves the full old force');
    foot(f, wall, [0, .5, .5], .75); measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(f.state.releases.length, 1); assert.equal(f.state.releases[0].id, id);
    sameWrench(ownedNormalWrench(f), force, 'second transport retains original anchor');
    foot(f, wall, [1 / 3, 1 / 3, 1 / 3], 0);
    const closed = measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(closed.pendingReleases.length, 0); assert.equal(carrier.bank.lambda, 0);
    assert.deepEqual(carrier.frozenWorldGradients, [], 'closed transport cycle has exactly zero world difference');
    assert.equal(closed.releasePositionMm, 0); sameWrench(ownedNormalWrench(f), force, 'closed cycle preserves force');
    foot(f, wall, [.5, .5, 0], .25); measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(f.state.releases[0], carrier); assert.equal(f.state.releases.length, 1, 'reuse zero carrier instead of appending unbounded rows');
    assert.deepEqual(f.a.y, pose); assert.deepEqual(f.joint._splitMotion.bodies[0].velocityY, v);
});

test('native fractional difference release preserves force balance through a subsequent transport and snapshot retry', () => {
    const f = fixture(), wall = normal(f); apply(f, solve(f, [wall]));
    foot(f, wall, [.5, .5, 0], .25);
    const rows = [wall], result = solve(f, rows), difference = f.state.releases[0];
    assert.equal(difference.mode, 'difference'); assert.equal(result.biasAdditionalIncrement[1], -1);
    const oldForce = ownedNormalWrench(f), expected = oldForce.map(v => v.slice()), scale = .25;
    for (const target of f.state.pending.targets) {
        const gradients = target.release ? target.entry.frozenWorldGradients : target.worldGradients;
        const delta = scale * result.biasAdditionalIncrement[target.index];
        for (const g of gradients) expected[g.side][g.dof] += delta * g.value;
    }
    const snapshot = captureKirchhoffCoupledTrialState(f.joint, { world: f.world, reusePropertyLayout: true, frozenFrictionBatches: true });
    apply(f, result, scale);
    assert.equal(difference.bank.lambda, .75);
    sameWrench(ownedNormalWrench(f), expected, 'all accepted canonical/release force increments use one scale');
    const beforeTransport = ownedNormalWrench(f), beta = bankBeta(f);
    foot(f, wall, [0, .5, .5], .75); measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(f.state.releases.length, 1); assert.equal(f.state.releases[0], difference); assert.equal(difference.bank.lambda, 1);
    assert.equal(bankBeta(f), beta); sameWrench(ownedNormalWrench(f), beforeTransport, 'remaining fraction is retained during the next transport');
    restoreKirchhoffCoupledTrialState(snapshot);
    assert.equal(f.state.releases[0], difference); assert.equal(difference.bank.lambda, 1);
    sameWrench(ownedNormalWrench(f), oldForce, 'rollback restores old anchor and both force banks');
    apply(f, result, .5); assert.equal(difference.bank.lambda, .5, 'same native result retries at the new common scale');
});

test('continuous angular transport retains world reaction across frame rotations, then semantic replacement fully retires both terms', () => {
    const f = fixture({ angular: true }), local = new Vector3(.2, -.4, .5), q0 = rotation(new Vector3(.25, -.32, .19));
    put(f.a, q0); put(f.joint._splitMotion.twoChannel.physicalPose[0], q0);
    const wall = normal(f, { alpha: .1, gap: -1, gradients: local.toArray().map((value, i) => ({ side: 0, dof: i + 3, value })) });
    apply(f, solve(f, [wall])); const original = ownedNormalWrench(f), beta = bankBeta(f);
    put(f.a, rotation(new Vector3(-.4, .6, .15)));
    const measured = measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(bankBeta(f), beta); assert.equal(measured.pendingReleases[0].mode, 'difference');
    assert.ok(measured.releaseAngleRad > 0); sameWrench(ownedNormalWrench(f), original, 'frame transport preserves world torque');
    const difference = f.state.releases[0];
    put(f.a, rotation(new Vector3(.2, -.6, .8))); measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(f.state.releases.length, 1); assert.equal(f.state.releases[0], difference);
    sameWrench(ownedNormalWrench(f), original, 'successive rotations telescope in world coordinates');
    wall.feature = 'new-semantic-surface';
    const replacement = measureKirchhoffTwoChannelRows(f.joint, [wall]);
    assert.equal(bankBeta(f), 0); assert.equal(f.state.entries[0].difference, null);
    assert.equal(replacement.pendingReleases.length, 2); assert.ok(replacement.pendingReleases.some(r => r.mode === 'full'));
    sameWrench(ownedNormalWrench(f), original, 'semantic replacement retires canonical AND existing difference, without losing either');
    const rows = [wall], r = solve(f, rows);
    assert.equal(rows.filter(row => row.kind === 'two-channel-bias-release').length, 2);
    assert.equal(r.system.descriptors[r.system.native.additionalOffset].bias.lambda, 0);
});

test('missing force history, material/mobility changes and negative bias remain hard failures rather than releases', () => {
    for (const failure of ['history', 'material', 'mobility', 'negative']) {
        const f = fixture(); apply(f, solve(f, [normal(f)])); const entry = f.state.entries[0], beta = bankBeta(f);
        if (failure === 'history') delete entry.frozenWorldGradients;
        if (failure === 'material') f.a.materialCoordinate[0] += .25;
        if (failure === 'mobility') f.a.inverseMass[0] *= 2;
        if (failure === 'negative') entry.bank[entry.slot] = -beta;
        if (failure === 'negative') assert.throws(() => prepareKirchhoffTwoChannelRows(f.joint, []), /Negative normal/);
        else {
            const prepared = prepareKirchhoffTwoChannelRows(f.joint, []);
            assert.equal(prepared.ready, false); assert.ok(prepared.issues[0].reason.includes('history') || prepared.issues[0].reason.includes('mobility'));
            assert.equal(bankBeta(f), beta);
        }
        assert.equal(f.state.releases.length, 0);
    }
});

test('a force representation changed after frozen assembly cannot be committed using the old result', () => {
    const f = fixture(), wall = normal(f); apply(f, solve(f, [wall]));
    const r = solve(f, [wall]);
    foot(f, wall, [.5, .5, 0], .25); measureKirchhoffTwoChannelRows(f.joint, [wall]);
    applyKirchhoffTwoChannelPhysicalMotion(f.joint, r); applyKirchhoffCoupledCorrection(f.joint, r);
    commitKirchhoffTwoChannelBiasMaterial(f.joint, r);
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /representation changed/);
});
