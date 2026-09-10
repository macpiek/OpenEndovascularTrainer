import assert from 'node:assert/strict';
import test from 'node:test';
import { Quaternion, Vector3 } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { assembleKirchhoffCoupledSystem } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffCoupledBoundaryStep } from '../src/physics/kirchhoffCoupledBoundaryRows.js';
import { beginKirchhoffCoupledOrientationStep, buildKirchhoffCoupledOrientationRows } from '../src/physics/kirchhoffCoupledOrientationRows.js';
import { beginKirchhoffSplitMotion } from '../src/physics/kirchhoffSplitMotion.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion, commitKirchhoffTwoChannelBiasMaterial,
    measureKirchhoffTwoChannelMaterial } from '../src/physics/kirchhoffTwoChannelMotion.js';
import { beginKirchhoffTwoChannelRows, prepareKirchhoffTwoChannelRows, commitKirchhoffTwoChannelRows,
    measureKirchhoffTwoChannelRows } from '../src/physics/kirchhoffTwoChannelRows.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';

const dt = 1 / 120, XYZ = ['X', 'Y', 'Z'], Q = [...XYZ, 'W'];
const near = (a, b, reason, tol = 1e-10) => assert.ok(Number.isFinite(a) && Math.abs(a - b) < tol, `${reason}: ${a} vs ${b}`);
const rotation = (x, y, z) => {
    const v = new Vector3(x, y, z), angle = v.length();
    return new Quaternion().setFromAxisAngle(v.divideScalar(angle || 1), angle);
};
const put = (pose, segment, q) => Q.forEach((a, i) => { pose['orientation' + a][segment] = q.toArray()[i]; });
const target = (body, q) => Q.forEach((a, i) => { body['orientationControl' + a] = q.toArray()[i]; });
function fixture() {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const profile = { mass: 1, radius: .25, inverseAngularInertia: 1, adaptationCompliance: .01 * dt * dt,
        kirchhoffBendCompliance: .02, kirchhoffTwistCompliance: .03, linearDamping: 1, angularDamping: 1 };
    const a = world.createRod('two-channel-rows-a', 3, 1, profile), b = world.createRod('two-channel-rows-b', 3, 1, profile);
    for (const body of [a, b]) {
        for (let i = 0; i < body.count; i++) body.setNodePosition(i, i, body === a ? 0 : -4, 0);
        body.copyCurrentToPrevious();
    }
    const tool = { bodyA: a, bodyB: b, lambdas: new Float64Array([.7, .8]), _jointReactions: new Map() };
    const sheath = { startX: 0, startY: 0, startZ: 0, axisX: 1, axisY: 0, axisZ: 0,
        proximalExtension: 1, length: 4, innerRadius: 1, lambdas: new Map() };
    world.toolContacts.push(tool); world.sheaths.push(sheath);
    const contact = { id: 2, feature: 'interior', innerMaterialSegmentId: 3, outerMaterialSegmentId: 4, normalLambda: .3 };
    const record = { manifoldContact: contact, gap: .1, _splitActualGap: -.02, _normalAlpha: .2,
        _innerSegmentIndex: 0, _outerSegmentIndex: 0, innerWeights: [0, 1], outerWeights: [0, 1], normal: [0, 1, 0],
        normalGradients: [{ side: 0, dof: 7, value: -1 }, { side: 1, dof: 7, value: 1 }] };
    const joint = { innerBody: a, outerBody: b, kirchhoffContacts: [record] };
    beginKirchhoffCoupledBoundaryStep(joint); beginKirchhoffSplitMotion(joint, world); beginKirchhoffTwoChannelMotion(joint);
    const state = beginKirchhoffTwoChannelRows(joint, world);
    return { world, joint, a, b, tool, sheath, contact, record, state, bank: state.bank };
}
function row(f, kind, { side = 0, node = 0, component = -1, alpha = .2, gap = -.01, owner, ...rest } = {}) {
    return { kind, side, node, component, alpha, strain: .125, _splitActualStrain: gap, lambda: 0, lower: 0, upper: Infinity,
        owner: owner ?? [f.a, f.b][side], gradients: [{ side, dof: node * 6 + 1, value: 1 }], ...rest };
}
function control(f, component = 0) {
    f.a.controlEnabled[1] = 1; f.a.controlCompliance[1] = .2 * dt * dt;
    const key = XYZ[component];
    f.a['control' + key][1] = f.a[key.toLowerCase()][1] + .1;
    return row(f, 'control', { node: 1, component, alpha: f.a.controlCompliance[1] / dt ** 2,
        strain: f.a[key.toLowerCase()][1] - f.a['control' + key][1], lower: -Infinity,
        gradients: [{ side: 0, dof: 6 + component, value: 1 }] });
}
function sheathRow(f, node = 1) {
    return row(f, 'sheath', { node, alpha: 0, owner: {}, sheathWitness: { geometry: f.joint._splitMotion.sheathHistory.get(f.sheath) } });
}
function assemble(f, rows = [], groups = []) {
    const prepared = prepareKirchhoffTwoChannelRows(f.joint, rows, groups);
    const native = assembleKirchhoffCoupledSystem(f.joint, dt, { additionalRows: rows, groups });
    const descriptors = prepared.channels(native);
    return { prepared, native, descriptors };
}
function result(f, rows, scale = .25) {
    const response = () => [f.a, f.b].map(b => ({ correction: new Float64Array(b.count * 6), lambda: new Float64Array(b.segmentCount * 6) }));
    return { physical: response(), bias: response(), scale, diagnostics: { converged: true },
        additionalIncrement: new Float64Array(rows.length), biasAdditionalIncrement: new Float64Array(rows.length),
        biasContactIncrement: new Float64Array(f.joint.kirchhoffContacts.length) };
}
function motion(f, r) { applyKirchhoffTwoChannelPhysicalMotion(f.joint, r); commitKirchhoffTwoChannelBiasMaterial(f.joint, r); }
function apply(f, rows, increments, contactIncrement = 0, scale = .25) {
    const r = result(f, rows, scale); r.biasAdditionalIncrement.set(increments); r.biasContactIncrement.fill(contactIncrement);
    motion(f, r); commitKirchhoffTwoChannelRows(f.joint, r); return r;
}

test('begin creates real independently owned banks without resetting physical arrays or history', () => {
    const f = fixture(), { bank } = f;
    assert.equal(f.joint._splitMotion.bank.tools[0].tool, f.tool);
    assert.equal(f.joint._splitMotion.bank.tools[0].lambdas, f.tool.lambdas);
    assert.equal(f.joint._splitMotion.bank.joints._coupledBoundaries, f.joint._coupledBoundaries);
    assert.equal(bank.tools[0].tool, f.tool); assert.notEqual(bank.tools[0].lambdas, f.tool.lambdas);
    assert.deepEqual([...f.tool.lambdas], [.7, .8]); assert.equal(f.contact.normalLambda, .3);
    assert.notEqual(bank.bodies[0].wallLambda, f.a.wallLambda);
    assert.notEqual(bank.joints._coupledBoundaries.controls[0], f.joint._coupledBoundaries.controls[0]);
    assert.equal(bank.joints._coupledBoundaries.sheaths.has(f.sheath), true);
    assert.throws(() => beginKirchhoffTwoChannelRows(f.joint, f.world), /must not restart/);
});

test('every runtime row gets its correct channel; compliance and raw geometry remain independent of physical strain', () => {
    const f = fixture(), rows = [row(f, 'wall'), row(f, 'tool', { owner: f.tool }), sheathRow(f),
        row(f, 'split-sweep', { node: 2, point: [0, 0, 0], n: [0, 1, 0] }), row(f, 'split-point-wall', { node: 1 }), control(f),
        row(f, 'fold'), row(f, 'external-friction'), row(f, 'split-wall-friction'), row(f, 'tool-release')];
    f.a.y[1] += .004;
    const { native, descriptors } = assemble(f, rows), material = measureKirchhoffTwoChannelMaterial(f.joint);
    for (let i = 0; i < native.contactOffset; i++) {
        const n = native.rows[i], d = descriptors[i];
        assert.equal(d.physical, 'pose'); assert.equal(d.bias.channel, 'bias-motion');
        assert.equal(d.bias.strain, material.biasStrain[n.side][n.local]); assert.equal(d.bias.alpha, n.alpha);
    }
    const normal = descriptors[native.contactOffset];
    assert.equal(normal.physical, 'physical-motion'); assert.equal(normal.bias.channel, 'pose');
    assert.equal(normal.bias.strain, -.02); assert.equal(normal.bias.alpha, .2); assert.equal(normal.bias.lambda, 0);
    rows.forEach((r, i) => {
        const d = descriptors[native.additionalOffset + i];
        if (i < 5) { assert.equal(d.physical, 'physical-motion'); assert.equal(d.bias.channel, 'pose'); assert.equal(d.bias.strain, -.01); }
        else if (i === 5) { assert.equal(d.physical, 'pose'); assert.equal(d.bias.channel, 'bias-motion'); assert.equal(d.bias.strain, 0); }
        else { assert.equal(d.physical, i === 6 ? 'pose' : 'physical-motion'); assert.equal(d.bias, null); }
    });
    assert.equal(f.contact.normalLambda, .3); assert.deepEqual([...f.tool.lambdas], [.7, .8]);
});

test('untyped surface friction is classified only through its explicit group indices and never sees beta normal load', () => {
    const f = fixture(), rows = [row(f, 'wall'), row(f, undefined, { lower: -Infinity }), row(f, undefined, { node: 1, lower: -Infinity })];
    const groups = [{ type: 'coulomb-disk', rows: [1, 2], radius: 0 }];
    const { native, descriptors } = assemble(f, rows, groups);
    for (const i of [1, 2]) assert.deepEqual(descriptors[native.additionalOffset + i], { physical: 'physical-motion', bias: null });
    apply(f, rows, [.8, 0, 0], .4);
    const second = assemble(f, rows, groups);
    assert.equal(second.descriptors[second.native.additionalOffset].bias.lambda, .2);
    assert.equal(second.native.rows[second.native.additionalOffset].lambda, 0);
    // Native assembly reduces a zero-radius disk to fixed zero bounds.
    for (const i of [1, 2]) {
        const sorted = [...second.native.order].indexOf(second.native.additionalOffset + i);
        assert.ok(second.native.lower[sorted] === 0); assert.ok(second.native.upper[sorted] === 0);
    }
    assert.throws(() => prepareKirchhoffTwoChannelRows(f.joint, rows, [{ rows: [0] }]), /cannot become friction/);
    assert.throws(() => prepareKirchhoffTwoChannelRows(f.joint, [row(f, 'mystery')]), /Unsupported/);
});

test('commits snapshot targets at the exact shared scale despite collector pool reuse and row reordering', () => {
    const f = fixture(), rows = [row(f, 'wall', { node: 0 }), row(f, 'wall', { node: 1 }), sheathRow(f),
        row(f, 'tool', { owner: f.tool }), row(f, 'split-sweep', { node: 2 }), row(f, 'split-point-wall', { node: 1 }), control(f)];
    assemble(f, rows);
    const p = f.state.pending, expectedGradients = p.targets[1].gradients.map(g => ({ ...g }));
    rows[0].node = 1; rows[0].kind = 'fold'; rows[0].gradients[0].value = 999;
    const r = apply(f, rows, [1, 2, 3, 4, 5, 6, -7], 8, .125);
    assert.deepEqual([...f.bank.bodies[0].wallLambda], [.125, .25]);
    assert.equal(f.bank.joints._coupledBoundaries.sheaths.get(f.sheath)[0].lambda[1], .375);
    assert.equal(f.bank.tools[0].lambdas[0], .5); assert.equal(f.bank.sweeps[0].get(2).lambda, .625);
    assert.equal(f.bank.pointWalls[0].get(1).lambda, .75); assert.equal(f.bank.joints._coupledBoundaries.controls[0][3], -.875);
    assert.equal(f.bank.contacts[0].normalLambda, 1); assert.equal(f.contact.normalLambda, .3);
    assert.deepEqual(f.state.entries.find(e => e.kind === 'wall' && e.node === 0).frozenGradients, expectedGradients);
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /uncommitted/);
    rows[0] = row(f, 'wall', { node: 0 }); rows.reverse();
    const again = assemble(f, rows);
    assert.equal(again.descriptors[again.native.additionalOffset + rows.length - 1].bias.lambda, .125);
    assert.equal(f.joint._splitMotion.sheathHistory.get(f.sheath).reactionHistory.bias[0][1], .375);
});

test('missing loaded normals retain whole old reactions in release banks while missing controls remain unsupported', () => {
    const f = fixture(), rows = [row(f, 'wall'), sheathRow(f), row(f, 'tool', { owner: f.tool }),
        row(f, 'split-sweep', { node: 2 }), row(f, 'split-point-wall', { node: 1 }), control(f)];
    assemble(f, rows); apply(f, rows, [1, 2, 3, 4, 5, -6], 7);
    const before = f.state.entries.map(e => [e.bank[e.slot], structuredClone(e.frozenWorldGradients)]);
    f.joint.kirchhoffContacts = [];
    const measured = measureKirchhoffTwoChannelRows(f.joint, []);
    assert.equal(measured.supported, false); assert.equal(measured.missingLoadedRows.length, 1);
    assert.equal(measured.missingLoadedRows[0].kind, 'control'); assert.equal(measured.pendingReleases.length, 6);
    const prepared = prepareKirchhoffTwoChannelRows(f.joint, []);
    assert.equal(prepared.ready, false);
    assert.throws(() => prepared.channels(assembleKirchhoffCoupledSystem(f.joint, dt)), /missing-loaded-bias-row/);
    f.state.entries.forEach((e, i) => {
        if (e.kind === 'control') assert.equal(e.bank[e.slot], before[i][0]);
        else {
            assert.equal(e.bank[e.slot], 0);
            const retired = f.state.releases.find(r => r.owner === e.owner && r.key === e.key);
            assert.equal(retired.bank.lambda, before[i][0]); assert.deepEqual(retired.frozenWorldGradients, before[i][1]);
        }
    });
});

test('a manifold object rekey or changed material label cannot inherit a loaded beta', () => {
    const f = fixture(); assemble(f); apply(f, [], [], 1);
    f.contact.id++;
    let measured = measureKirchhoffTwoChannelRows(f.joint);
    assert.equal(measured.supported, true); assert.equal(measured.pendingReleases.length, 1);
    assert.equal(f.bank.contacts[0].normalLambda, 0); assert.equal(measured.pendingReleases[0].beta, .25);
    f.contact.id--; f.a.materialCoordinate[1] += .1;
    measured = measureKirchhoffTwoChannelRows(f.joint);
    assert.equal(measured.supported, false); assert.ok(measured.issues.some(i => i.reason === 'bias-release-material-or-mobility-changed'));
    assert.equal(f.state.releases[0].bank.lambda, .25);
});

test('continuous material-foot changes preserve beta with explicit world-wrench difference releases', () => {
    const f = fixture(), tool = row(f, 'tool', { owner: f.tool, tA: .4, tB: .3 });
    delete tool.side; // Actual runtime tool rows have an owner and no side.
    f.a.wallT[0] = .4;
    const rows = [row(f, 'wall'), tool];
    rows[0].gradients = [{ side: 0, dof: 1, value: .6 }, { side: 0, dof: 7, value: .4 }];
    assemble(f, rows); apply(f, rows, [1, 2], 3);
    f.a.wallT[0] = .6; tool.tA = .6; f.record.innerWeights = [.4, .6];
    rows[0].gradients = [{ side: 0, dof: 1, value: .4 }, { side: 0, dof: 7, value: .6 }];
    tool.gradients = [{ side: 0, dof: 1, value: .8 }, { side: 0, dof: 7, value: .2 }];
    f.record.normalGradients = [{ side: 0, dof: 1, value: -.4 }, { side: 0, dof: 7, value: -.6 }, { side: 1, dof: 7, value: 1 }];
    const m = measureKirchhoffTwoChannelRows(f.joint, rows);
    assert.equal(m.supported, true); assert.equal(m.pendingReleases.length, 3);
    assert.equal(f.bank.tools[0].lambdas[0], .5); assert.equal(f.bank.bodies[0].wallLambda[0], .25);
    assert.ok(m.pendingReleases.every(r => r.mode === 'difference' && r.carrier === 1));
});

test('missing angular normal retains the old world wrench even after material frames rotate', () => {
    const f = fixture(), q = rotation(.4, .2, -.3), local = new Vector3(.2, -.5, .7);
    put(f.a, 0, q);
    f.record.normalGradients = local.toArray().map((value, i) => ({ side: 0, dof: 3 + i, value }));
    assemble(f); apply(f, [], [], 2);
    const entry = f.state.entries.find(e => e.kind === 'normal'), frozen = structuredClone(entry.frozenWorldGradients);
    const expected = local.clone().applyQuaternion(new Quaternion(...Q.map(a => f.a['orientation' + a][0])).normalize());
    frozen.forEach((g, i) => near(g.value, expected.toArray()[i], 'stored world torque gradient', 1e-7));
    put(f.a, 0, rotation(-.8, .5, .1)); f.joint.kirchhoffContacts = [];
    assert.equal(measureKirchhoffTwoChannelRows(f.joint).pendingReleases.length, 1);
    assert.deepEqual(f.state.releases[0].frozenWorldGradients, frozen); assert.equal(f.state.releases[0].bank.lambda, .5);
    assert.equal(entry.bank[entry.slot], 0);
});

test('zero-beta rows can vanish; a released reaction can vanish only after its negative increment was actually committed', () => {
    const f = fixture(), rows = [row(f, 'wall')]; assemble(f, rows); apply(f, rows, [2]);
    assemble(f, rows); const released = apply(f, rows, [-2]);
    assert.equal(released.biasAdditionalIncrement[0], -2); assert.equal(f.bank.bodies[0].wallLambda[0], 0);
    assert.equal(measureKirchhoffTwoChannelRows(f.joint, []).supported, true);
    assert.equal(prepareKirchhoffTwoChannelRows(f.joint, []).ready, true);
});

test('fresh residuals use nonlinear raw geometry and vector control norms, including loaded separating normals', () => {
    const f = fixture(), rows = [row(f, 'wall', { alpha: .4 }), ...[0, 1, 2].map(i => control(f, i))];
    assemble(f, rows); apply(f, rows, [2, 3, 4, 0], 1);
    f.record._splitActualGap = -.05; // -.05 + .2*.25 = 0
    rows[0]._splitActualStrain = -.2; // -.2 + .4*.5 = 0
    const m = measureKirchhoffTwoChannelRows(f.joint, rows);
    near(m.normalResidualMm, 0, 'fresh compliant normals');
    near(m.controlResidualMm, .25, 'sqrt((.2*.75)^2+(.2*1)^2)', 1e-8);
    rows[0]._splitActualStrain = .03;
    near(measureKirchhoffTwoChannelRows(f.joint, rows).normalResidualMm, .23, 'loaded equality even at separating geometry');
    assert.equal(rows[0].strain, .125, 'physical history RHS was not substituted for geometry');
    f.joint._splitMotion.phase = 'complete';
    assert.equal(measureKirchhoffTwoChannelRows(f.joint, rows).supported, true);
    assert.throws(() => prepareKirchhoffTwoChannelRows(f.joint, rows), /physical phase/);
});

test('orientation uses the difference of native target-frame logs, measures freshly with null batch and leaves the solve batch untouched', () => {
    const f = fixture(); f.a.orientationControlSegment = 0; f.a.orientationControlCompliance = dt ** 2 * .3;
    const qt = rotation(.4, -.3, .2), qp = rotation(-.25, .17, .11), qg = rotation(.21, .33, -.14);
    target(f.a, qt); put(f.a, 0, qg); put(f.joint._splitMotion.twoChannel.physicalPose[0], 0, qp);
    beginKirchhoffCoupledOrientationStep(f.joint);
    const batch = buildKirchhoffCoupledOrientationRows(f.joint, dt), rows = [...batch.rows];
    const { native, descriptors } = assemble(f, rows);
    const log = q => {
        if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
        const r = Math.hypot(q.x, q.y, q.z), angle = 2 * Math.atan2(r, q.w);
        return [q.x, q.y, q.z].map(v => r ? v * angle / r : 0);
    };
    const g = log(qt.clone().invert().multiply(qg)), p = log(qt.clone().invert().multiply(qp));
    const difference = g.map((v, i) => v - p[i]), wrong = log(qp.clone().invert().multiply(qg));
    assert.ok(Math.hypot(...difference.map((v, i) => v - wrong[i])) > .05);
    rows.forEach((_, i) => near(descriptors[native.additionalOffset + i].bias.strain, difference[i], 'same-target log difference', 1e-7));
    apply(f, rows, [1, -2, 3]);
    const snapshot = { version: batch.version, rows: rows.map(r => ({ strain: r.strain, gradients: structuredClone(r.gradients) })) };
    let m = measureKirchhoffTwoChannelRows(f.joint, [], null);
    near(m.orientationResidualRad, Math.hypot(...difference.map((v, i) => v + .3 * [.25, -.5, .75][i])), 'orientation vector residual', 1e-7);
    put(f.a, 0, rotation(.6, -.1, .2));
    assert.notEqual(measureKirchhoffTwoChannelRows(f.joint, [], null).orientationResidualRad, m.orientationResidualRad);
    assert.deepEqual({ version: batch.version, rows: rows.map(r => ({ strain: r.strain, gradients: r.gradients })) }, snapshot);
    f.a.orientationControlSegment = 1;
    m = measureKirchhoffTwoChannelRows(f.joint, [], null);
    assert.equal(m.supported, false); assert.equal(m.issues.filter(i => i.reason === 'loaded-bias-row-identity-changed').length, 3);
});

test('commit prevalidation is atomic for scale mismatch, stale targets, invalid normals and foreign bias friction increments', () => {
    const f = fixture(), rows = [row(f, 'wall'), control(f), row(f, 'external-friction')];
    assemble(f, rows); const r = result(f, rows); r.biasAdditionalIncrement.set([1, 2, 0]);
    motion(f, r);
    assert.throws(() => prepareKirchhoffTwoChannelRows(f.joint, rows), /Commit the preceding/);
    r.scale = .5;
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /same scale/);
    r.scale = .25; f.a.controlX[1] += .1;
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /owner changed/);
    f.a.controlX[1] -= .1; // float32 roundtrip is not an exact target restore
    f.a.controlX[1] = f.state.pending.targets.find(t => t.kind === 'control').guards.find(g => g.key === 'controlX').value;
    r.biasAdditionalIncrement[0] = -.0000000000001;
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /Negative solved/);
    r.biasAdditionalIncrement[0] = 1; r.biasAdditionalIncrement[2] = .1;
    assert.throws(() => commitKirchhoffTwoChannelRows(f.joint, r), /physical-only/);
    assert.equal(f.bank.bodies[0].wallLambda[0], 0); assert.equal(f.bank.joints._coupledBoundaries.controls[0][3], 0);
    r.biasAdditionalIncrement[2] = 0; commitKirchhoffTwoChannelRows(f.joint, r);
    assert.equal(f.bank.bodies[0].wallLambda[0], .25);
});

test('native assembly detects a source equation or normal compliance changed after prepare', () => {
    const f = fixture(), rows = [row(f, 'wall')];
    let prepared = prepareKirchhoffTwoChannelRows(f.joint, rows); rows[0].strain += .1;
    assert.throws(() => prepared.channels(assembleKirchhoffCoupledSystem(f.joint, dt, { additionalRows: rows })), /equation changed/);
    prepared = prepareKirchhoffTwoChannelRows(f.joint, rows); f.record._normalAlpha = .3;
    assert.throws(() => prepared.channels(assembleKirchhoffCoupledSystem(f.joint, dt, { additionalRows: rows })), /equation changed/);
});

test('trial rollback restores all beta bytes, real bank references, target receipts and sheath reaction history', () => {
    const f = fixture(), rows = [sheathRow(f), row(f, 'tool', { owner: f.tool }), row(f, 'split-point-wall', { node: 1 }), control(f)];
    assemble(f, rows);
    const state = f.state, bank = f.bank, controls = bank.joints._coupledBoundaries.controls[0], contacts = bank.contacts,
        history = f.joint._splitMotion.sheathHistory.get(f.sheath).reactionHistory.bias[0], pending = state.pending;
    const snapshot = captureKirchhoffCoupledTrialState(f.joint, { world: f.world, reusePropertyLayout: true, frozenFrictionBatches: true });
    const r = apply(f, rows, [1, 2, 3, -4], 5);
    restoreKirchhoffCoupledTrialState(snapshot);
    assert.equal(f.joint._splitMotion.twoChannel.rows, state); assert.equal(state.bank, bank);
    assert.equal(f.joint._splitMotion.biasBank, bank); assert.equal(bank.joints._coupledBoundaries.controls[0], controls);
    assert.equal(bank.contacts, contacts); assert.equal(state.pending, pending); assert.equal(pending.committed, false);
    assert.ok(controls.every(v => v === 0)); assert.ok(history.every(v => v === 0));
    assert.equal(bank.contacts[0].normalLambda, 0); assert.equal(bank.pointWalls[0].get(1).lambda, 0);
    assert.equal(f.joint._splitMotion.twoChannel.applications.size, 0);
    r.scale = .125; motion(f, r); commitKirchhoffTwoChannelRows(f.joint, r);
    assert.equal(history[1], .125); assert.equal(bank.tools[0].lambdas[0], .25); assert.equal(controls[3], -.5);
    assert.deepEqual([...f.tool.lambdas], [.7, .8]); assert.equal(f.contact.normalLambda, .3);
});

test('sheath history journals the actual difference wrench rather than its unit carrier', () => {
    const f = fixture(), rows = [sheathRow(f)], history = f.joint._splitMotion.sheathHistory.get(f.sheath).reactionHistory.bias[0];
    assemble(f, rows); apply(f, rows, [.04]);
    const maximum = history[1]; assert.equal(maximum, .01);
    rows[0].gradients = [{ side: 0, dof: 7, value: Math.cos(.1) }, { side: 0, dof: 8, value: Math.sin(.1) }];
    const measured = measureKirchhoffTwoChannelRows(f.joint, rows);
    assert.equal(measured.pendingReleases[0].mode, 'difference'); assert.equal(measured.pendingReleases[0].carrier, 1);
    assemble(f, rows); assert.equal(rows.length, 2);
    apply(f, rows, [0, -1], 0, .5);
    assert.equal(f.state.releases[0].bank.lambda, .5);
    assert.equal(history[1], maximum, 'unit carrier must not fabricate a normal force of one');
    measureKirchhoffTwoChannelRows(f.joint, []);
    assert.equal(f.bank.joints._coupledBoundaries.sheaths.get(f.sheath)[0].lambda[1], 0);
    assert.equal(history[1], maximum, 'retirement cannot erase the earlier loaded-step witness');
});
