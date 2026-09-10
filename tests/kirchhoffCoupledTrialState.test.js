import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const runtimeRoot = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? fileURLToPath(new URL('../', import.meta.url)));
const load = path => import(pathToFileURL(resolve(runtimeRoot, path)));
const { EndovascularPhysicsWorld } = await load('src/physics/endovascularPhysicsWorld.js');
const { buildKirchhoffCoupledFrictionRows, commitKirchhoffCoupledFrictionMultipliers,
    measureKirchhoffCoupledFrictionResidual } = await load('src/physics/kirchhoffCoupledFrictionRows.js');
const { beginKirchhoffSplitMotion } = await load('src/physics/kirchhoffSplitMotion.js');
const { buildKirchhoffSplitWallFriction, commitKirchhoffSplitWallFriction,
    measureKirchhoffSplitWallFriction } = await load('src/physics/kirchhoffSplitWallFriction.js');
import { captureKirchhoffCoupledTrialState as capture, restoreKirchhoffCoupledTrialState as restore } from '../src/physics/kirchhoffCoupledTrialState.js';

function fixture() {
    const world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120 });
    const inner = world.createRod('wire', 2, 5, { radius: .4445 });
    const outer = world.createRod('catheter', 2, 5, { radius: .8 });
    for (let i = 0; i < 2; i++) inner.setNodePosition(i, 5 * i, .0405, 0);
    const c = world.addContainment(inner, outer, { innerRadius: .485, axialFriction: .2 });
    const contact = c.manifold.upsertContact({ innerMaterialSegmentId: 1, outerMaterialSegmentId: 2,
        innerSegmentIndex: 0, outerSegmentIndex: 0, normal: [0, 1, 0], tangentU: [1, 0, 0] });
    contact.normalLambda = 1; contact.tangentLambda.set([.1, .02]);
    const record = { id: 'record', kind: 'side', gap: 0, normal: [0, 1, 0],
        _innerSegmentIndex: 0, _outerSegmentIndex: 0, innerT: .5, outerT: .5,
        innerWeights: [.5, .5], outerWeights: [.5, .5], manifoldContact: contact };
    c.kirchhoffContacts.push(record);
    c._kirchhoffRuntimeRecords = [record];
    c._jointFrictionBatch = buildKirchhoffCoupledFrictionRows(c, 1 / 120);
    return { world, inner, outer, c, contact, record };
}

test('rollback restores exact body bytes and identities without traversing static geometry', () => {
    const f = fixture(), positions = f.inner.x;
    f.inner.contactField = new Proxy({}, { ownKeys() { throw new Error('Static collision geometry was traversed'); } });
    const initial = positions.slice(), snapshot = capture(f.c, { world: f.world });
    f.inner.x[0] += 1;
    f.inner.x = new Float32Array([7, 8]);
    f.inner.orientationW[0] = .9;
    f.inner.adaptationLambdaX[0] = 10;
    f.inner.wallActive[0] = 1;
    f.inner.wallProjectionX[0] = .2;
    f.inner.newTrialScalar = 1;
    restore(snapshot);
    assert.equal(f.inner.x, positions);
    assert.deepEqual(f.inner.x, initial);
    assert.equal(f.inner.adaptationLambdaX[0], 0);
    assert.equal(f.inner.wallActive[0], 0);
    assert.equal(f.inner.wallProjectionX[0], 0);
    assert.equal('newTrialScalar' in f.inner, false);
    assert.ok(snapshot.bytes > 0 && snapshot.objectCount > 0);
});

test('geometry basis remap, friction commit, material rekey and pooled contact replacement roll back by identity', () => {
    const f = fixture(), oldId = f.contact.id, basis = [...f.contact.normal], tangent = [...f.contact.tangentLambda];
    const records = f.c.kirchhoffContacts, pool = f.c._kirchhoffRuntimeRecords;
    const snapshot = capture(f.c);
    f.c.manifold.refreshKnownContact(f.contact, { normal: [0, 0, 1], tangentU: [1, 0, 0], projectFriction: false });
    f.record.normal = [0, 0, 1];
    f.record.surfaceFrictionDiagnosticPhase = 'trial';
    const batch = buildKirchhoffCoupledFrictionRows(f.c, 1 / 120, f.c._jointFrictionBatch);
    commitKirchhoffCoupledFrictionMultipliers(batch, [.01, -.005]);
    f.c.manifold.rekeyKnownContact(f.contact, { innerMaterialSegmentId: 3, outerMaterialSegmentId: 4,
        normal: [1, 0, 0], tangentU: [0, 1, 0] });
    const newContact = f.c.manifold.upsertContact({ innerMaterialSegmentId: 5, outerMaterialSegmentId: 6, normal: [0, 1, 0] });
    f.c.kirchhoffContacts.length = 0;
    f.c._kirchhoffRuntimeRecords[0] = { manifoldContact: newContact };
    restore(snapshot);
    assert.equal(f.c.kirchhoffContacts, records); assert.equal(f.c._kirchhoffRuntimeRecords, pool);
    assert.equal(records[0], f.record); assert.equal(pool[0], f.record);
    assert.equal(f.record.manifoldContact, f.contact);
    assert.equal(f.contact.id, oldId); assert.equal(f.c.manifold.getContact(oldId), f.contact);
    assert.equal(f.c.manifold.size, 1); assert.equal(f.contact._manifold, f.c.manifold);
    assert.equal(newContact._manifold, null);
    assert.deepEqual(f.contact.normal, basis); assert.deepEqual([...f.contact.tangentLambda], tangent);
    assert.equal(f.c._jointFrictionBatch.committed, false);
    assert.equal('surfaceFrictionDiagnosticPhase' in f.record, false);
    assert.equal('innerSurfaceMomentImpulse' in f.record, false);
    assert.equal(snapshot.needsReassembly, true);
});

test('boundary/fold/sheath/tool multipliers and branch hints restore while actual work counts remain', () => {
    const f = fixture(), sheath = f.world.addSheath({ start: { x: 0, y: 0, z: 0 }, end: { x: 1, y: 0, z: 0 } });
    const sheathState = { lambda: new Float64Array([.1, .2]), normal: new Float64Array([0, 1, 0]), active: new Uint8Array([1, 0]) };
    f.c._coupledBoundaries = { controls: [new Float64Array([.1, .2, .3])], wallHints: [new Uint8Array([1])], sheaths: new Map([[sheath, [sheathState]]]) };
    f.c._coupledFoldRows = { storage: [{ body: f.inner, lambda: new Float64Array([.5]), axisLocal: new Float64Array([.3, .4]), axisValid: new Uint8Array([1]) }], pending: true, buildVersion: 4 };
    const tool = { bodyA: f.inner, bodyB: f.outer, lambdas: new Float32Array([.2]), _jointBoundaryRows: new Map([[1, { activeHint: true, owner: f.inner }]]) };
    f.world.toolContacts.push(tool);
    f.world.maxPenetration = .1; f.world.lastJointFactorizations = 5;
    const snapshot = capture(f.c, { world: f.world });
    f.c._coupledBoundaries.controls[0].fill(20); f.c._coupledBoundaries.wallHints[0].fill(0);
    sheathState.lambda.fill(0); sheathState.normal.fill(0); sheathState.active.fill(0);
    f.c._coupledFoldRows.storage[0].lambda.fill(7); f.c._coupledFoldRows.storage[0].axisValid.fill(0);
    f.c._coupledFoldRows.pending = false; f.c._coupledFoldRows.buildVersion++;
    tool.lambdas.fill(1); tool._jointBoundaryRows.clear();
    f.world.maxPenetration = 10; f.world.lastJointFactorizations = 8;
    const workspace = f.c._coupledSystemQP = { kernel: {}, factor: new Float64Array([1, 2]) };
    const bundle = f.c._bundleRuntime = { _paired: true, _revision: 3 };
    restore(snapshot);
    assert.deepEqual([...f.c._coupledBoundaries.controls[0]], [.1, .2, .3]);
    assert.deepEqual([...sheathState.lambda], [.1, .2]); assert.equal(sheathState.active[0], 1);
    assert.equal(f.c._coupledFoldRows.storage[0].lambda[0], .5); assert.equal(f.c._coupledFoldRows.pending, true);
    assert.equal(f.c._coupledFoldRows.buildVersion, 4); assert.equal(tool._jointBoundaryRows.get(1).owner, f.inner);
    assert.equal(f.world.maxPenetration, .1); assert.equal(f.world.lastJointFactorizations, 8);
    assert.equal(f.c._coupledSystemQP, workspace, 'retain allocated numeric/WASM workspace');
    assert.equal(f.c._bundleRuntime, bundle); assert.equal(bundle._paired, false);
    sheathState.lambda[0] = 9; restore(snapshot); assert.equal(sheathState.lambda[0], .1);
    assert.equal(snapshot.restoreCount, 2);
});

test('active topology edits fail before any rollback writes', () => {
    const f = fixture(), snapshot = capture(f.c);
    f.inner.activeEnd = 0; f.inner.x[0] = 42;
    assert.throws(() => restore(snapshot), /topology/);
    assert.equal(f.inner.x[0], 42);
});

test('sealed record layouts still capture changing values, hidden data and child identities', () => {
    const f = fixture(), snapshot = {}, options = { reusePropertyLayout: true };
    const state = { value: 1, child: { load: new Float64Array([2, 3]) } };
    Object.defineProperty(state, 'hidden', { value: 4, writable: true, configurable: false });
    Object.defineProperty(state, 'accessor', { get() { throw new Error('Do not invoke an accessor'); } });
    f.c.sealedMetadata = Object.seal(state);
    capture(f.c, options, snapshot);
    const child = state.child = { load: new Float64Array([5, 6]) };
    state.value = 7; state.hidden = 8;
    capture(f.c, options, snapshot);
    state.child = { load: new Float64Array([99]) };
    child.load.fill(100); state.value = 10; state.hidden = 11;
    restore(snapshot);
    assert.equal(state.child, child);
    assert.deepEqual([...child.load], [5, 6]);
    assert.equal(state.value, 7); assert.equal(state.hidden, 8);
    assert.equal(Object.getOwnPropertyDescriptor(state, 'hidden').enumerable, false);
    assert.equal(Object.isSealed(state), true);
});

test('reused storage captures new values, changed array identities, keys and owner contacts', () => {
    const f = fixture(), snapshot = {}, owner = {}, owners = new Map([[owner, new Map([[0, f.contact]])]]);
    f.c._coupledExternalFriction = { owners, contacts: [f.contact] };
    capture(f.c, { reusePropertyLayout: true }, snapshot);
    const bytes = snapshot.records.find(r => r.object === f.inner.x).copy;
    f.inner.x[0] = 3; f.inner.persistedNewScalar = 7;
    const replacement = f.inner.y = new Float64Array([.1, .2]);
    capture(f.c, { reusePropertyLayout: true }, snapshot);
    assert.equal(snapshot.records.find(r => r.object === f.inner.x).copy, bytes);
    f.inner.x[0] = 5; f.inner.y = new Float64Array(2);
    delete f.inner.persistedNewScalar; f.inner.rejectedNewScalar = 8;
    owners.get(owner).set(1, {}); owners.set({}, new Map());
    restore(snapshot);
    assert.equal(f.inner.x[0], 3); assert.equal(f.inner.y, replacement);
    assert.equal(f.inner.persistedNewScalar, 7); assert.equal('rejectedNewScalar' in f.inner, false);
    assert.equal(owners.size, 1); assert.equal(owners.get(owner).size, 1);
});

test('frozen solve batches retry the same correction after fresh geometry and friction measurements', () => {
    const f = fixture(), reference = fixture(), batch = f.c._jointFrictionBatch;
    const options = { reusePropertyLayout: true, frozenFrictionBatches: true };
    const snapshot = capture(f.c, options);
    capture(f.c, options, snapshot);
    const force = [.02, -.01], start = f.inner.x.slice();
    commitKirchhoffCoupledFrictionMultipliers(batch, force);
    f.inner.x[0] += .03;
    f.c.manifold.refreshKnownContact(f.contact, { normal: [0, 0, 1], tangentU: [1, 0, 0], projectFriction: false });
    f.record.normal = [0, 0, 1];
    measureKirchhoffCoupledFrictionResidual(f.c, 1 / 120, f.c._jointFrictionResidual ??= {});
    restore(snapshot);
    assert.deepEqual(f.inner.x, start);
    commitKirchhoffCoupledFrictionMultipliers(batch, force, .5);
    commitKirchhoffCoupledFrictionMultipliers(reference.c._jointFrictionBatch, force, .5);
    assert.deepEqual(f.contact.tangentLambda, reference.contact.tangentLambda);
    assert.deepEqual(f.record.surfaceTangentialIncrement, reference.record.surfaceTangentialIncrement);
    assert.deepEqual(f.record.innerSurfaceMomentImpulse, reference.record.innerSurfaceMomentImpulse);
    assert.deepEqual(f.record.outerSurfaceMomentImpulse, reference.record.outerSurfaceMomentImpulse);
    assert.ok(snapshot.objectCount < capture(reference.c).objectCount);
});

test('rebuilding an immutable solve batch rejects rollback before touching physical state', () => {
    const f = fixture(), snapshot = capture(f.c, { frozenFrictionBatches: true });
    buildKirchhoffCoupledFrictionRows(f.c, 1 / 120, f.c._jointFrictionBatch);
    f.inner.x[0] = 99;
    assert.throws(() => restore(snapshot), /frozen friction solve batch/);
    assert.equal(f.inner.x[0], 99);
});

function splitWallFixture() {
    const f = fixture();
    f.inner.wallStaticFriction = f.inner.wallKineticFriction = .3;
    f.inner.wallLambda[0] = .4; f.inner.wallT[0] = .5; f.inner.wallNormalY[0] = -1;
    beginKirchhoffSplitMotion(f.c, f.world);
    const normal = { kind: 'wall', side: 0, node: 0, owner: f.inner, alpha: 0, strain: 0,
        gradients: [{ side: 0, dof: 1, value: -.5 }, { side: 0, dof: 7, value: -.5 }] };
    f.normals = [normal];
    f.batch = f.c._jointSplitWallFrictionBatch = buildKirchhoffSplitWallFriction(f.c, f.normals, 1 / 120);
    f.wallContact = f.batch.entries[0].contact;
    // Real World appends the borrowed friction rows to the same boundary
    // collector. That alias must not reintroduce their derivative graph.
    f.c._coupledBoundaries = { rows: [normal, ...f.batch.rows] };
    return f;
}

test('frozen split-wall rows preserve their mechanical owner and retry after a fresh wall measurement', () => {
    const f = splitWallFixture(), reference = splitWallFixture();
    const options = { world: f.world, reusePropertyLayout: true, frozenFrictionBatches: true };
    const complete = capture(f.c, { world: f.world }), snapshot = capture(f.c, options);
    const before = f.inner.x.slice(), tangent = f.wallContact.tangentLambda.slice();
    const increment = Float64Array.of(.02, -.01);
    assert.ok(snapshot.objectCount < complete.objectCount);
    assert.ok(snapshot.records.some(r => r.object === f.wallContact.tangentLambda), 'physical wall history is owned');
    assert.ok(!snapshot.records.some(r => r.object === f.batch.rows[0]), 'immutable solve rows are excluded through their alias');
    commitKirchhoffSplitWallFriction(f.batch, increment, 1);
    f.inner.x[0] += .03; f.inner.wallT[0] = .75;
    measureKirchhoffSplitWallFriction(f.c, f.normals, 1 / 120, f.c._jointSplitWallFrictionResidual ??= {});
    restore(snapshot);
    assert.deepEqual(f.inner.x, before); assert.equal(f.inner.wallT[0], .5);
    assert.deepEqual(f.wallContact.tangentLambda, tangent); assert.equal(f.batch.committed, false);
    commitKirchhoffSplitWallFriction(f.batch, increment, .5);
    commitKirchhoffSplitWallFriction(reference.batch, increment, .5);
    assert.deepEqual(f.wallContact.tangentLambda, reference.wallContact.tangentLambda);
    assert.deepEqual(f.wallContact.tangentU, reference.wallContact.tangentU);
    assert.deepEqual(f.wallContact.tangentV, reference.wallContact.tangentV);
});

test('rebuilding frozen split-wall rows rejects restore before physical writes', () => {
    const f = splitWallFixture(), snapshot = capture(f.c, { frozenFrictionBatches: true });
    buildKirchhoffSplitWallFriction(f.c, f.normals, 1 / 120, f.batch);
    f.inner.x[0] = 99;
    assert.throws(() => restore(snapshot), /frozen friction solve batch/);
    assert.equal(f.inner.x[0], 99);
});

test('reused property layout preserves accessors, hidden values and same-size key replacements', () => {
    const f = fixture(), snapshot = {}, state = f.c.trialMetadata = { a: 1, b: 2 };
    Object.defineProperty(state, 'hidden', { value: 3, writable: true, configurable: true });
    Object.defineProperty(state, 'accessor', { get() { throw new Error('Do not evaluate an accessor'); }, configurable: true });
    capture(f.c, { reusePropertyLayout: true }, snapshot);
    state.a = 4; state.hidden = 5;
    capture(f.c, { reusePropertyLayout: true }, snapshot);
    state.a = 8; state.hidden = 9;
    restore(snapshot);
    assert.equal(state.a, 4); assert.equal(state.hidden, 5);
    assert.equal(Object.getOwnPropertyDescriptor(state, 'hidden').enumerable, false);
    delete state.b; state.c = 10;
    capture(f.c, { reusePropertyLayout: true }, snapshot);
    state.b = 11; delete state.c;
    restore(snapshot);
    assert.equal(state.c, 10); assert.equal('b' in state, false);
    assert.equal(typeof Object.getOwnPropertyDescriptor(state, 'accessor').get, 'function');
});

test('reused traversal discovers same-size graph replacements, cycles, maps and sets', () => {
    const f = fixture(), snapshot = {}, a = { value: 1 }, b = { value: 2 };
    const state = f.c.graphProbe = { child: a, map: new Map([[0, a]]), set: new Set([a]) };
    a.owner = state; b.owner = state;
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot);
    capture(f.c, options, snapshot);
    state.child = b; state.map.set(0, b); state.set.clear(); state.set.add(b);
    capture(f.c, options, snapshot);
    assert.ok(snapshot.records.some(r => r.object === b));
    assert.ok(!snapshot.records.some(r => r.object === a), 'Unreachable former children must leave the plan');
    b.value = 9; state.child = a; state.map.clear(); state.set.clear(); a.value = 10;
    restore(snapshot);
    assert.equal(state.child, b); assert.equal(b.owner, state); assert.equal(b.value, 2);
    assert.equal(state.map.get(0), b); assert.ok(state.set.has(b));
    assert.equal(a.value, 10, 'An unreachable object is not part of this rollback');
});

test('root and external barrier changes update a reused traversal without inspecting static geometry', () => {
    const f = fixture(), snapshot = {}, extra = { lambda: new Float64Array([3]) };
    const geometry = new Proxy({}, { ownKeys() { throw Error('Static geometry was inspected'); } });
    f.c.extraState = extra; f.c.staticGeometry = geometry;
    capture(f.c, { reusePropertyLayout: true, external: [geometry, extra] }, snapshot);
    capture(f.c, { reusePropertyLayout: true, external: [geometry] }, snapshot);
    extra.lambda[0] = 4;
    restore(snapshot); assert.equal(extra.lambda[0], 3);
    const tool = { lambda: 7 };
    capture(f.c, { reusePropertyLayout: true, external: [geometry], toolContacts: [tool] }, snapshot);
    tool.lambda = 9; restore(snapshot); assert.equal(tool.lambda, 7);
    capture(f.c, { reusePropertyLayout: true, external: [geometry, extra], toolContacts: [] }, snapshot);
    extra.lambda[0] = 10; tool.lambda = 11;
    restore(snapshot); assert.equal(extra.lambda[0], 10); assert.equal(tool.lambda, 11);
});

test('filtered body state can switch between captured numeric data and external objects', () => {
    const f = fixture(), snapshot = {}, extra = { untouched: 1 }, options = { reusePropertyLayout: true };
    f.inner.dynamicState = 3;
    capture(f.c, options, snapshot);
    f.inner.dynamicState = extra;
    capture(f.c, options, snapshot);
    extra.untouched = 2;
    restore(snapshot); assert.equal(extra.untouched, 2);
    f.inner.dynamicState = new Float64Array([4]);
    capture(f.c, options, snapshot);
    f.inner.dynamicState[0] = 9;
    restore(snapshot); assert.equal(f.inner.dynamicState[0], 4);
});

test('shared views retain exact NaN payload, signed zero and subview bytes on repeated rollback', () => {
    const f = fixture(), buffer = new ArrayBuffer(48), words = new Uint32Array(buffer);
    words.set([0, 0x80000000, 0x12345678, 0x7ff81234, 3, 4, 5, 6, 7, 8, 9, 10]);
    f.c.aliasProbe = { doubles: new Float64Array(buffer), slice: new DataView(buffer, 5, 19) };
    const original = new Uint8Array(buffer).slice(), snapshot = {};
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot); capture(f.c, options, snapshot);
    new Uint8Array(buffer).fill(255); restore(snapshot);
    assert.deepEqual(new Uint8Array(buffer), original);
    assert.equal(Object.is(f.c.aliasProbe.doubles[0], -0), true);
    assert.equal(Number.isNaN(f.c.aliasProbe.doubles[1]), true);
    new Uint8Array(buffer).fill(0); restore(snapshot);
    assert.deepEqual(new Uint8Array(buffer), original);
});

test('array holes, additional hidden keys and changed identities are captured between trials', () => {
    const f = fixture(), snapshot = {}, rows = f.c.arrayProbe = [1, , 3];
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot);
    rows.length = 5; rows[4] = { value: 7 };
    Object.defineProperty(rows, 'hidden', { value: 9, writable: true, configurable: true });
    capture(f.c, options, snapshot);
    const child = rows[4];
    rows.fill(8); rows.length = 2; rows.extra = 10; rows.hidden = 11; child.value = 12;
    restore(snapshot);
    assert.equal(rows.length, 5); assert.equal(1 in rows, false); assert.equal(3 in rows, false);
    assert.equal(rows[4], child); assert.equal(child.value, 7); assert.equal(rows.hidden, 9);
    assert.equal('extra' in rows, false);
    assert.equal(Object.getOwnPropertyDescriptor(rows, 'hidden').enumerable, false);
});

test('dense trial arrays restore length, references and hidden trial additions on every retry', () => {
    const f = fixture(), snapshot = {}, child = { value: 7 };
    const rows = f.c.arrayProbe = [child, 2, -0, NaN];
    child.owner = rows;
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot);
    rows.push(child, 9);
    capture(f.c, options, snapshot);
    for (const length of [1, 12, 0]) {
        rows.length = length;
        rows[0] = 42; child.value = 99;
        Object.defineProperty(rows, 'hiddenTrialField', { value: 10, configurable: true });
        restore(snapshot);
        assert.deepEqual(rows, [child, 2, -0, NaN, child, 9]);
        assert.equal(child.value, 7); assert.equal(child.owner, rows);
        assert.equal(Object.hasOwn(rows, 'hiddenTrialField'), false);
    }
});

test('array rollback switches safely between dense buffers, sparse arrays and accessor metadata', () => {
    const f = fixture(), snapshot = {}, rows = f.c.arrayProbe = [1, 2, 3];
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot);
    delete rows[1]; rows.length = 5;
    Object.defineProperty(rows, 'hidden', { value: 8, writable: true, configurable: true });
    capture(f.c, options, snapshot);
    rows.fill(9); rows.hidden = 10;
    restore(snapshot);
    assert.equal(1 in rows, false); assert.equal(3 in rows, false);
    assert.equal(rows.hidden, 8);
    delete rows.hidden; rows.length = 3; rows[1] = 4;
    capture(f.c, options, snapshot);
    rows.splice(0, 3); restore(snapshot);
    assert.deepEqual(rows, [1, 4, 3]);
    const getter = () => { throw Error('Array accessor must not be invoked'); };
    Object.defineProperty(rows, '3', { get: getter, configurable: true, enumerable: true });
    capture(f.c, options, snapshot);
    delete rows[3]; rows[0] = 99;
    restore(snapshot);
    assert.equal(Object.getOwnPropertyDescriptor(rows, '3').get, getter);
    assert.equal(rows[0], 1); assert.equal(rows.length, 4);
});

test('dense array retry matches the complete snapshot including graph ownership and property descriptors', () => {
    const f = fixture(), optimized = {}, child = { value: 6 };
    f.c.arrayProbe = [child, f.inner.x, child, f.contact];
    child.owner = f.c.arrayProbe;
    capture(f.c, { reusePropertyLayout: true }, optimized);
    capture(f.c, { reusePropertyLayout: true }, optimized);
    const reference = capture(f.c);
    f.c.arrayProbe.reverse(); f.c.arrayProbe.push({ value: 10 });
    delete f.c.arrayProbe[1]; child.value = 9; f.inner.x.fill(20);
    f.c.manifold.clear();
    restore(optimized);
    for (const record of reference.records) {
        if (record.kind === 'bytes') assert.deepEqual(record.view, record.copy);
        else if (record.kind === 'object') {
            const actualKeys = Object.getOwnPropertyNames(record.object)
                .filter(key => !record.filter || record.filter(key, record.object[key]));
            assert.deepEqual(actualKeys.slice().sort(), record.keys.slice().sort());
            for (const key of record.keys)
                assert.deepEqual(Object.getOwnPropertyDescriptor(record.object, key), record.descriptors[key]);
        } else if (record.kind === 'map') assert.deepEqual([...record.object], record.entries);
        else if (record.kind === 'set') assert.deepEqual([...record.object], record.entries);
    }
});

test('packed contact values fall back for new accessors and hidden properties without retaining old children', () => {
    const f = fixture(), snapshot = {}, oldChild = { value: 2 };
    const state = f.c.metadata = { value: 1, child: oldChild };
    const options = { reusePropertyLayout: true };
    capture(f.c, options, snapshot);
    const child = state.child = { value: 3 };
    capture(f.c, options, snapshot);
    const getter = () => { throw Error('Contact accessor must not be invoked'); };
    Object.defineProperty(state, 'accessor', { get: getter, configurable: true });
    Object.defineProperty(state, 'hidden', { value: 7, writable: true, configurable: true });
    capture(f.c, options, snapshot);
    state.hidden = 9; state.value = 10; child.value = 11; oldChild.value = 12;
    delete state.accessor;
    restore(snapshot);
    assert.equal(state.value, 1); assert.equal(state.hidden, 7);
    assert.equal(state.child, child); assert.equal(child.value, 3); assert.equal(oldChild.value, 12);
    assert.equal(Object.getOwnPropertyDescriptor(state, 'accessor').get, getter);
    assert.equal(Object.getOwnPropertyDescriptor(state, 'hidden').enumerable, false);
    delete state.accessor; delete state.hidden;
    for (const reusePropertyLayout of [true, false, true]) {
        capture(f.c, { reusePropertyLayout }, snapshot);
        state.child = oldChild; state.value = 15;
        restore(snapshot);
        assert.equal(state.child, child); assert.equal(state.value, 1);
    }
});

test('a rejected real joint apply and contact refresh reproduce the original mechanical solve', async () => {
    const { solveKirchhoffCoupledSystem: solve, applyKirchhoffCoupledCorrection: apply } =
        await load('src/physics/kirchhoffCoupledSystem.js');
    const stopped = Symbol('finished'), snapshot = {};
    let world, original, calls = 0, changedBytes = 0;
    const coupledSystem = {
        solve(c, dt, options) {
            if (calls++) {
                for (const record of snapshot.records) if (record.kind === 'bytes') {
                    for (let i = 0; i < record.copy.length; i++) changedBytes += Number(record.view[i] !== record.copy[i]);
                }
                assert.ok(changedBytes > 0, 'The real trial must change saved mechanics');
                solve(c, dt, options); // Reassemble at the rejected nonlinear geometry.
                const workspace = c._coupledSystemAssembly;
                restore(snapshot);
                assert.equal(c._coupledSystemAssembly, workspace);
                for (const record of snapshot.records) if (record.kind === 'bytes') assert.deepEqual(record.view, record.copy);
                const replay = solve(c, dt, c._jointOptions);
                for (const [a, b] of [[original.inner.correction, replay.inner.correction],
                    [original.outer.correction, replay.outer.correction],
                    [original.contactIncrement, replay.contactIncrement],
                    [original.additionalIncrement, replay.additionalIncrement]]) {
                    assert.equal(a.length, b.length);
                    for (let i = 0; i < a.length; i++) assert.ok(Math.abs(a[i] - b[i]) < 1e-12);
                }
                throw stopped;
            }
            original = solve(c, dt, options);
            // Capture twice to exercise the reused plan before the real apply.
            capture(c, { world, reusePropertyLayout: true }, snapshot);
            capture(c, { world, reusePropertyLayout: true }, snapshot);
            return original;
        }, apply
    };
    world = new EndovascularPhysicsWorld({ fixedDt: 1 / 120, coupledSystem });
    const profile = { radius: .4445, innerRadius: .485, linearDamping: 1, angularDamping: 1,
        projectionVelocityRetention: 1, foldLimitStrength: 0, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 2, 10, { ...profile, mass: 1 });
    const outer = world.createRod('catheter', 2, 10, { ...profile, mass: 3 });
    for (let i = 0; i < 2; i++) { inner.setNodePosition(i, i * 10, .2, 0); inner.velocityX[i] = 6; }
    inner.angularVelocityX.fill(2);
    world.addContainment(inner, outer, { innerRadius: .485, axialFriction: .2, torsionalFriction: .2,
        portalFilletRadius: 0, coupledBendingRateDamping: 0, radialVelocityDamping: 0 });
    assert.throws(() => world.stepFixed(), error => error === stopped);
    assert.equal(calls, 2); assert.equal(snapshot.restoreCount, 1);
});

test('isolated component rollback restores material, walls and owned state without touching another tool',()=>{
 const {world,inner,outer}=fixture();
 const c={bodies:[inner],kirchhoffContacts:[],_jointBoundary:{lambda:new Float64Array([.2,.3])}};
 const x=inner.x.slice(),lambda=inner.adaptationLambdaX.slice();
 const snapshot=capture(c,{world,reusePropertyLayout:true});
 inner.x[0]+=2;inner.adaptationLambdaX[0]=9;inner.wallLambda[0]=4;
 c._jointBoundary.lambda[0]=7;outer.x[0]=23;
 restore(snapshot);
 assert.deepEqual(inner.x,x);assert.deepEqual(inner.adaptationLambdaX,lambda);
 assert.equal(inner.wallLambda[0],0);assert.deepEqual(c._jointBoundary.lambda,new Float64Array([.2,.3]));
 assert.equal(outer.x[0],23);
 const reused=capture(c,{world,reusePropertyLayout:true},snapshot);
 inner.y[0]=5;restore(reused);assert.notEqual(inner.y[0],5);
 c.bodies=[outer];assert.throws(()=>restore(snapshot),/topology/);
});

const {BufferGeometry,Float32BufferAttribute}=await import('three');
const {MeshBVH}=await import('three-mesh-bvh');
const {beginKirchhoffWallWitnessStep,collectKirchhoffWallWitnessRows}=await import('../src/physics/kirchhoffWallWitnessRows.js');
const {buildKirchhoffWallWitnessFriction,appendKirchhoffWallWitnessFriction,commitKirchhoffWallWitnessFriction,measureKirchhoffWallWitnessFriction}=await import('../src/physics/kirchhoffWallWitnessFriction.js');
function witnessFixture(count=25){
 const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-20,-20,0,20,-20,0,0,20,0],3));geometry.boundsTree=new MeshBVH(geometry);
 const field={fallbackGeometry:geometry},world=new EndovascularPhysicsWorld({contactField:field});
 const body=world.createRod('witness-catheter',count,.1,{radius:.5,wallFriction:.2});body.contactField=field;
 for(let n=0;n<count;n++)body.setNodePosition(n,n*.1,0,.4);
 for(let n=0;n<count-1;n++){body.wallActive[n]=1;body.wallFaceIndex[n]=0;body.wallT[n]=.5;body.wallGap[n]=-.1;}
 const c={bodies:[body],kirchhoffContacts:[]};beginKirchhoffWallWitnessStep(c);
 const rows=collectKirchhoffWallWitnessRows(c,field,[],1/120),batch=buildKirchhoffWallWitnessFriction(c,rows,1/120,c._wallWitnessFrictionSolve={});
 appendKirchhoffWallWitnessFriction(batch,rows,[]);c._coupledBoundaries={rows};
 const measured=buildKirchhoffWallWitnessFriction(c,rows,1/120,c._wallWitnessFrictionMeasure={});c._jointStateMeasurement={wallWitnessFriction:measureKirchhoffWallWitnessFriction(c,measured)};
 return {c,body,world,rows,batch};
}
test('frozen witness batches omit scratch while retaining exact tangent wrench history and retry scale',()=>{
 const f=witnessFixture(),reference=witnessFixture(),options={world:f.world,frozenFrictionBatches:true,reusePropertyLayout:true};
 const complete=capture(f.c,{world:f.world}),snapshot=capture(f.c,options);
 const witness=f.c._wallWitnessRows.witnesses[0],ledger=witness.ledger;
 assert.ok(snapshot.objectCount<complete.objectCount);
 for(const physical of [ledger,ledger.tangentLambda,ledger.wrenches,witness.friction.tangentU,witness.friction.tangentV])assert.ok(snapshot.records.some(r=>r.object===physical));
 for(const scratch of [f.batch,f.batch.rows[0],f.batch.entries[0].wall,f.c._wallWitnessFrictionMeasure])assert.ok(!snapshot.records.some(r=>r.object===scratch));
 const delta=new Float64Array(f.rows.length);delta[f.batch.rowOffset]=.01;delta[f.batch.rowOffset+1]=-.02;
 commitKirchhoffWallWitnessFriction(f.batch,delta,1);f.body.x[0]+=.03;
 measureKirchhoffWallWitnessFriction(f.c,f.c._wallWitnessFrictionMeasure);
 restore(snapshot);assert.equal(f.batch.committed,false);assert.deepEqual([...ledger.tangentLambda],[0,0]);assert.deepEqual(ledger.wrenches,[]);
 commitKirchhoffWallWitnessFriction(f.batch,delta,.25);commitKirchhoffWallWitnessFriction(reference.batch,delta,.25);
 const expected=reference.c._wallWitnessRows.witnesses[0].ledger;
 for(const key of ['lambda','tangentLambda','wrenches','normalWrenches','retiring'])assert.deepEqual(ledger[key],expected[key]);
});
test('witness solve rebuild is detected before rollback touches physical state',()=>{
 const f=witnessFixture(3),snapshot=capture(f.c,{world:f.world,frozenFrictionBatches:true});
 buildKirchhoffWallWitnessFriction(f.c,f.rows,1/120,f.batch);f.body.x[0]=99;
 assert.throws(()=>restore(snapshot),/frozen friction solve batch/);assert.equal(f.body.x[0],99);
});
test('whole candidate snapshots never capture prior trial snapshots or grow with disposable measurement banks',()=>{
 const f=witnessFixture(),trial=f.c._jointTrialState={};
 capture(f.c,{world:f.world,frozenFrictionBatches:true,reusePropertyLayout:true},trial);
 const baseline=capture(f.c,{world:f.world});
 assert.ok(!baseline.records.some(r=>r.object===trial||r.object===trial.records));
 assert.ok(!baseline.records.some(r=>r.object===f.c._wallWitnessFrictionMeasure.measurementBatch),'measurement aliases must not recapture disposable banks');
 for(let step=0;step<12;step++){
  measureKirchhoffWallWitnessFriction(f.c,f.c._wallWitnessFrictionMeasure);
  capture(f.c,{world:f.world,frozenFrictionBatches:true,reusePropertyLayout:true},trial);
  const base=capture(f.c,{world:f.world});
  assert.equal(base.objectCount,baseline.objectCount);assert.equal(base.bytes,baseline.bytes);
  restore(base);assert.equal(f.c._jointTrialState,trial);
 }
});
