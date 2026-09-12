import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';

const DT = 1 / 120;
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-14, `${a} != ${b}`);
function fixture({ mode = 'accept', maxSubsteps = 2, clock = null } = {}) {
    const control = { mode, calls: 0, resets: 0, commits: 0, rollbacks: 0, inputs: [] };
    const system = {
        id: 'controlled-whole-physical-step',
        step(world, dt) {
            assert.equal(this, system); assert.equal(dt, DT);
            control.calls++; control.inputs.push(world.bodies[0].forceX[0]);
            const body = world.bodies[0], next = body.x.slice();
            next[0] += 1; // A private candidate, published only on success.
            clock?.(5);
            if (control.mode !== 'accept') {
                control.rollbacks++; clock?.(2);
                if (control.mode === 'throw') throw new Error('provider rolled back');
                return { accepted: false, dt, status: 'controlled-rejection', diagnostics: { commits: control.commits } };
            }
            body.x.set(next); control.commits++;
            return { accepted: true, dt, status: 'accepted', diagnostics: { commits: control.commits } };
        },
        reset(world) { assert.equal(this, system); assert.equal(world.stepCount, 0); control.resets++; control.commits = 0; }
    };
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, maxSubsteps, wholeStepSystem: system,
        coupledSystem: { solve() { assert.fail('Legacy coupled solve'); } },
        contactField: { queryCapsule() { assert.fail('Legacy contact query'); } } });
    const body = world.createRod('wire', 3, 1);
    for (let i = 0; i < body.count; i++) body.setNodePosition(i, i, 0, 0);
    body.copyCurrentToPrevious();
    return { world, body, system, control };
}

test('whole-dt provider publishes once before legacy prediction, integration, damping or contact passes', () => {
    const f = fixture(), x = f.body.x.slice(), previous = f.body.previousX.slice();
    // These traps belong to the real Body that the original implementation
    // mutates before integration and reads during its configured damping.
    Object.defineProperty(f.body, '_splitPhysicalMotion', { set() { assert.fail('Legacy predictor entered'); } });
    Object.defineProperty(f.body, 'linearDamping', { get() { assert.fail('Legacy damping entered'); } });
    f.body.velocityX.fill(13); f.body.forceX.fill(7);
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true); assert.equal(result.dt, DT); assert.equal(result.systemId, f.system.id);
    assert.equal(result.consumedPendingDt, false); assert.equal(f.world.lastStepResult, result);
    assert.equal(f.world.stepCount, 1); assert.equal(f.world.accumulator, 0); assert.equal(f.control.commits, 1);
    assert.equal(f.body.x[0], x[0] + 1); assert.deepEqual(f.body.previousX, previous);
    assert.ok(f.body.velocityX.every(v => v === 13)); assert.ok(f.body.forceX.every(v => v === 7));
    for (const [name, timing] of Object.entries(f.world.timings)) assert.equal(timing.recordedCount, name === 'total' ? 1 : 0);
    assert.equal(f.world.getStats().wholeStep, result); assert.equal(f.world.getStats().jointMotion, null);
});

test('early rejection keeps prepared input and elapsed dt; direct retry commits and consumes them exactly once', () => {
    const f = fixture({ mode: 'reject' }), before = f.body.x.slice(); let preparations = 0;
    const prepare = () => f.body.forceX.fill(++preparations);
    assert.equal(f.world.advance(DT, prepare), 0);
    const failed = structuredClone(f.world.lastStepResult);
    assert.equal(f.world.advance(0, prepare), 0); assert.equal(preparations, 1);
    assert.deepEqual(f.body.x, before); assert.deepEqual(f.control.inputs, [1, 1]);
    assert.equal(f.world.accumulator, DT); assert.equal(f.world.stepCount, 0); assert.equal(f.world.lastSubsteps, 0);
    f.control.mode = 'accept'; const retried = f.world.stepFixed();
    assert.equal(retried.accepted, true); assert.equal(retried.consumedPendingDt, true);
    assert.equal(f.world.accumulator, 0); assert.equal(f.world.stepCount, 1); assert.equal(f.control.commits, 1);
    assert.equal(f.world.advance(0, prepare), 0); assert.equal(preparations, 1);
    assert.equal(failed.accepted, false); assert.equal(failed.diagnostics.commits, 0);
});

test('provider exception preserves pending preparation, body transaction, clocks and an explicit failed result', () => {
    const f = fixture({ mode: 'throw' }), before = f.body.x.slice(); let prepared = 0;
    const prepare = () => f.body.forceX.fill(++prepared);
    assert.throws(() => f.world.advance(DT, prepare), /provider rolled back/);
    assert.equal(f.world.lastStepResult.accepted, false); assert.equal(f.world.lastStepResult.status, 'whole-step-error');
    assert.equal(f.world.stepCount, 0); assert.equal(f.world.accumulator, DT); assert.deepEqual(f.body.x, before);
    assert.throws(() => f.world.advance(0, prepare), /provider rolled back/); assert.equal(prepared, 1);
    assert.equal(f.world.timings.total.recordedCount, 2); assert.equal(f.world.lastSubsteps, 0);
    f.control.mode = 'accept'; assert.equal(f.world.advance(0, prepare), 1);
    assert.equal(prepared, 1); assert.deepEqual(f.control.inputs, [1, 1, 1]); assert.equal(f.world.stepCount, 1);
});

test('an unfunded direct retry never subtracts time, while an advance retry funds the same prepared dt once', () => {
    const a = fixture({ mode: 'reject' }); assert.equal(a.world.stepFixed().accepted, false);
    assert.equal(a.world.advance(0, () => assert.fail('No queued dt')), 0);
    a.control.mode = 'accept'; assert.equal(a.world.stepFixed().consumedPendingDt, false);
    assert.equal(a.world.stepCount, 1); assert.equal(a.world.accumulator, 0);
    const b = fixture({ mode: 'reject', maxSubsteps: 1 }); assert.equal(b.world.stepFixed().accepted, false);
    b.control.mode = 'accept'; let prepared = 0;
    assert.equal(b.world.advance(2 * DT, () => prepared++), 1);
    assert.equal(prepared, 0); assert.equal(b.world.accumulator, DT); assert.equal(b.world.lastStepResult.consumedPendingDt, true);
    assert.equal(b.world.advance(0, () => prepared++), 1); assert.equal(prepared, 1); assert.equal(b.world.accumulator, 0);
});

test('maxSubsteps and an intervening rejection preserve the full backlog without dropping or duplicate preparation', () => {
    const f = fixture(); let prepared = 0;
    const prepare = () => { prepared++; f.control.mode = prepared === 2 ? 'reject' : 'accept'; };
    assert.equal(f.world.advance(4 * DT, prepare), 1);
    close(f.world.accumulator, 3 * DT); assert.equal(f.world.lastSubsteps, 1); assert.equal(prepared, 2);
    f.control.mode = 'accept'; assert.equal(f.world.advance(0, prepare), 2);
    close(f.world.accumulator, DT); assert.equal(prepared, 3);
    assert.equal(f.world.advance(0, prepare), 1); close(f.world.accumulator, 0);
    assert.equal(prepared, 4); assert.equal(f.control.calls, 5); assert.equal(f.world.stepCount, 4);
    assert.equal(f.world.droppedTime, 0); assert.equal(f.world.timings.total.recordedCount, 5);
});

test('pending dt, provider object, id and methods cannot change or fall back, even before the next accumulator update', () => {
    const f = fixture({ mode: 'reject' }); f.world.advance(DT);
    const alternatives = [
        [f.world, 'fixedDt', DT / 2], [f.world, 'wholeStepSystem', null],
        [f.world, 'wholeStepSystem', { ...f.system }], [f.system, 'id', 'different'],
        [f.system, 'step', () => assert.fail('Replacement step')], [f.system, 'reset', () => {}]
    ];
    for (const [object, key, replacement] of alternatives) {
        const original = object[key]; object[key] = replacement;
        assert.throws(() => f.world.advance(7 * DT), /pending whole timestep/);
        assert.throws(() => f.world.stepFixed(), /pending whole timestep/);
        assert.equal(f.world.accumulator, DT); assert.equal(f.world.stepCount, 0); assert.equal(f.control.calls, 1);
        object[key] = original;
    }
    f.control.mode = 'accept'; assert.equal(f.world.stepFixed().accepted, true);
    f.world.wholeStepSystem = null; assert.equal(f.world.advance(0), 0); assert.equal(f.world.stepCount, 1);
});

test('reset explicitly abandons a pending dt and resets the owned adapter without keeping pending inputs', () => {
    const f = fixture({ mode: 'reject' }); f.world.advance(DT); assert.equal(f.world.resetSimulationState(), f.world);
    assert.equal(f.control.resets, 1); assert.equal(f.world._pendingWholeSubstep, null);
    assert.equal(f.world.lastStepResult, null); assert.equal(f.world.accumulator, 0); assert.equal(f.world.stepCount, 0);
    assert.equal(f.world.timings.total.recordedCount, 0);
    f.control.mode = 'accept'; let prepared = 0;
    assert.equal(f.world.advance(DT, () => prepared++), 1); assert.equal(prepared, 1);
    f.control.mode = 'reject'; f.world.advance(DT);
    // An explicit reset can also retire the original pending adapter after
    // a rejected mode change. It must not strand that adapter's transaction.
    let replacementResets = 0;
    f.world.wholeStepSystem = { id: 'replacement', step() { assert.fail(); }, reset() { replacementResets++; } };
    f.world.resetSimulationState(); assert.equal(f.control.resets, 2); assert.equal(replacementResets, 1);
});

test('a preparation exception is never replayed or passed as a valid partial input; reset is required', () => {
    const f = fixture(), error = new Error('partial preparation'); let prepared = 0;
    const prepare = () => { f.body.forceX.fill(++prepared); throw error; };
    assert.throws(() => f.world.advance(DT, prepare), e => e === error);
    assert.equal(f.world.lastStepResult.status, 'whole-step-preparation-error'); assert.equal(f.control.calls, 0);
    assert.throws(() => f.world.advance(0, prepare), e => e === error);
    assert.throws(() => f.world.stepFixed(), e => e === error);
    assert.equal(prepared, 1); assert.equal(f.control.calls, 0); assert.equal(f.world.accumulator, DT);
    f.world.resetSimulationState(); assert.equal(f.world.advance(DT, () => f.body.forceX.fill(2)), 1);
    assert.deepEqual(f.control.inputs, [2]);
    // JavaScript permits non-Error/falsy throws; they must still revoke the
    // preparation instead of accidentally marking the partial input valid.
    assert.throws(() => f.world.advance(DT, () => { throw 0; }), e => e === 0);
    assert.throws(() => f.world.stepFixed(), e => e === 0); assert.equal(f.control.calls, 1);
});

test('whole-step timings include preparation and provider rollback exactly once without old phase samples', t => {
    let time = 0; t.mock.method(globalThis.performance, 'now', () => time);
    const f = fixture({ mode: 'reject', clock: dt => time += dt });
    f.world.advance(DT, () => time += 3);
    assert.equal(f.world.timings.total.last, 10); assert.equal(f.world.timings.total.recordedCount, 1);
    f.control.mode = 'accept'; f.world.stepFixed();
    assert.equal(f.world.timings.total.last, 5); assert.equal(f.world.timings.total.total, 15);
    assert.equal(f.world.timings.total.recordedCount, 2); assert.equal(f.world.timings.integrate.recordedCount, 0);
});

test('malformed and asynchronous provider results cannot commit World clocks and can retry without re-preparation', () => {
    for (const bad of [null, {}, { accepted: 1, dt: DT, status: 'wrong' }, { accepted: true, dt: 2 * DT, status: 'wrong' },
        { accepted: true, dt: DT, status: '' }, Promise.resolve({ accepted: true, dt: DT, status: 'async' })]) {
        let result = bad, calls = 0, prepared = 0;
        const world = new EndovascularPhysicsWorld({ fixedDt: DT,
            wholeStepSystem: { id: 'contract-control', step() { calls++; return result; }, reset() {} } });
        assert.throws(() => world.advance(DT, () => prepared++), /synchronous/);
        assert.equal(world.stepCount, 0); assert.equal(world.accumulator, DT); assert.equal(world.lastStepResult.accepted, false);
        result = { accepted: true, dt: DT, status: 'accepted' }; assert.equal(world.advance(0, () => prepared++), 1);
        assert.equal(prepared, 1); assert.equal(calls, 2); assert.equal(world.stepCount, 1); assert.equal(world.accumulator, 0);
    }
    for (const wholeStepSystem of [{}, { id: '', step() {}, reset() {} }, { id: 5, step() {}, reset() {} },
        { id: 'missing-reset', step() {} }, { id: 'missing-step', reset() {} }])
        assert.throws(() => new EndovascularPhysicsWorld({ wholeStepSystem }), /wholeStepSystem requires/);
});

test('provider clock mutation and reentrant execution are explicit errors, never extra committed dt', () => {
    for (const act of [world => world.stepCount++, world => world.accumulator++, world => world.stepFixed(), world => world.resetSimulationState()]) {
        const world = new EndovascularPhysicsWorld({ wholeStepSystem: { id: 'bad-clock-owner',
            step(w, dt) { act(w); return { accepted: true, dt, status: 'accepted' }; }, reset() {} } });
        assert.throws(() => world.advance(DT), /clocks|running/);
        assert.equal(world.stepCount, 0); assert.equal(world.accumulator, DT); assert.equal(world.lastStepResult.accepted, false);
    }
});

test('a callback may select the whole-dt law before a new step, but a pending split cannot switch laws', () => {
    const f = fixture({ mode: 'reject' }); f.world.wholeStepSystem = null; let prepared = 0;
    assert.equal(f.world.advance(DT, () => { prepared++; f.world.wholeStepSystem = f.system; }), 0);
    f.control.mode = 'accept'; assert.equal(f.world.stepFixed().consumedPendingDt, true);
    assert.equal(f.world.accumulator, 0); assert.equal(prepared, 1); assert.equal(f.world.stepCount, 1);
    const split = new EndovascularPhysicsWorld({ jointMotionMode: 'split-physical-bias' });
    split._pendingSplitSubstep = { dt: DT }; split.wholeStepSystem = f.system;
    assert.throws(() => split.advance(DT), /pending split timestep/);
    assert.throws(() => split.stepFixed(), /pending split timestep/);
    assert.equal(split.accumulator, 0); assert.equal(split.stepCount, 0);
});
