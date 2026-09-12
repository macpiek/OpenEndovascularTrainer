import assert from 'node:assert/strict';
import test from 'node:test';
import { Vector3, Quaternion } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem as solve, applyKirchhoffCoupledCorrection as apply } from '../src/physics/kirchhoffCoupledSystem.js';
import { locateKirchhoffDistalLumenBranch as locate, isKirchhoffDistalLumenWitness as owns } from '../src/physics/kirchhoffToolContactOwnership.js';

const xyz = ['x', 'y', 'z'];
function accumulate(target, gradients, scale) {
    for (const g of gradients) {
        const key = g.side + ':' + g.dof;
        target.set(key, (target.get(key) ?? 0) + g.value * scale);
    }
}
const spatialKeys = ['fx', 'fy', 'fz', 'mx', 'my', 'mz'];
// Independent Three.js pullback of the PRE-APPLY Jacobian. This oracle never
// calls the production capture/transport helpers or derives its expectation
// from the ledger being checked.
function spatialReaction(joint, values) {
    const out = new Map();
    for (const [side, body] of [joint.innerBody, joint.outerBody].entries()) for (let node = 0; node < body.count; node++) {
        const f = new Vector3(...[0, 1, 2].map(axis => values.get(side + ':' + (node * 6 + axis)) ?? 0));
        const m = new Vector3(body.x[node], body.y[node], body.z[node]).cross(f);
        if (node < body.activeEnd) m.add(new Vector3(...[3, 4, 5].map(axis => values.get(side + ':' + (node * 6 + axis)) ?? 0))
            .applyQuaternion(new Quaternion(body.orientationX[node], body.orientationY[node], body.orientationZ[node], body.orientationW[node]).normalize()));
        [...f.toArray(), ...m.toArray()].forEach((value, axis) => out.set(`${side}:${node}:${axis}`, value));
    }
    return out;
}
function sameSpatial(actual, expected) {
    const values = new Map();
    for (const w of actual) spatialKeys.forEach((key, axis) => values.set(`${w.side}:${w.node}:${axis}`, w[key]));
    for (const key of new Set([...values.keys(), ...expected.keys()]))
        assert.ok(Math.abs((values.get(key) ?? 0) - (expected.get(key) ?? 0)) < 1e-10, `${key}: ${values.get(key)} != ${expected.get(key)}`);
}
function sameBodyWrench(actual, expected) {
    for (const side of [0, 1]) for (let axis = 0; axis < 6; axis++) {
        const sum = map => [...map].reduce((value, [key, next]) => key.startsWith(side + ':') && key.endsWith(':' + axis) ? value + next : value, 0);
        assert.ok(Math.abs(sum(actual) - sum(expected)) < 1e-10, `side ${side}, world wrench ${axis}: ${sum(actual)} != ${sum(expected)}`);
    }
}
function rotated(body, node, vector) {
    const q = new Quaternion(body.orientationX[node], body.orientationY[node], body.orientationZ[node], body.orientationW[node]).normalize();
    const { x, y, z, w } = q;
    const [a, b, c] = vector, tx = 2 * (y * c - z * b), ty = 2 * (z * a - x * c), tz = 2 * (x * b - y * a);
    return [a + w * tx + y * tz - z * ty, b + w * ty + z * tx - x * tz, c + w * tz + x * ty - y * tx];
}
function wrench(joint, values) {
    const force = [0, 0, 0], moment = [0, 0, 0];
    for (const [side, body] of [joint.innerBody, joint.outerBody].entries()) for (let node = 0; node < body.count; node++) {
        const f = [0, 1, 2].map(axis => values.get(side + ':' + (node * 6 + axis)) ?? 0);
        const spin = node < body.segmentCount ? rotated(body, node,
            [3, 4, 5].map(axis => values.get(side + ':' + (node * 6 + axis)) ?? 0)) : [0, 0, 0];
        const p = xyz.map(axis => body[axis][node]);
        for (let axis = 0; axis < 3; axis++) {
            force[axis] += f[axis];
            moment[axis] += spin[axis] + p[(axis + 1) % 3] * f[(axis + 2) % 3] - p[(axis + 2) % 3] * f[(axis + 1) % 3];
        }
    }
    return { force, moment };
}

for (const [transition, forceRollback, loadedPasses = 1] of [['mouth', false], ['range', false], ['disabled', false], ['mouth', true], ['mouth', false, 2], ['translated-mouth', false], ['empty-range', false], ['twisted-translated', false], ['twisted-translated', true]]) test(`loaded external ${transition} transition after ${loadedPasses} loads ${forceRollback ? 'rolls back and retries its reaction' : 'converges after physical release'}`, () => {
    const stopped = Symbol('rollback retry verified');
    const world = new EndovascularPhysicsWorld();
    const profile = { radius: .4445, foldLimitStrength: 0, linearDamping: 1, angularDamping: 1,
        projectionVelocityRetention: 1, sleepFrames: 1e6 };
    const wire = world.createRod('wire', 4, 5, { ...profile, mass: 1 });
    const catheter = world.createRod('catheter', 3, 5, { ...profile, radius: .8333, mass: 3 });
    for (let i = 0; i < 4; i++) { wire.setNodePosition(i, i * 5 + (transition === 'translated-mouth' ? -.1 : .1), transition === 'translated-mouth' ? .8 : .04, 0); wire.velocityZ[i] = 1; }
    wire.angularVelocityX.fill(1);
    const joint = world.addContainment(wire, catheter, { innerRadius: .485, startNode: 0, endNode: 1,
        containedLength: 10, enforceDistalPortal: false, openDistal: true, axialFriction: .2,
        torsionalFriction: .2, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    const tool = world.addToolContact(wire, catheter, { startSegmentA: 2, endSegmentA: 2,
        startSegmentB: 1, endSegmentB: 1, openDistalB: false, friction: .2 });
    if (transition === 'translated-mouth') {
        joint.containedLength = 0; joint.endNode = 0; tool.openDistalB = true;
    }
    const index = 5, firstReaction = new Map();
    let expectedSpatial = new Map(), releaseExpected;
    let rows, calls = 0, releaseCalls = 0, releasedTrials = 0, retried = false, firstFn, firstFt;
    let releaseStart = null, releaseNormal, releaseTangent;
    world.coupledSystem = {
        solve(c, dt, options) {
            rows = options.additionalRows.map(r => ({ ...r, gradients: r.gradients.map(g => ({ ...g })) }));
            const release = rows.find(r => r.kind === 'tool-release');
            if (release) {
                releaseCalls++;
                assert.equal(release.lower, 0); assert.equal(release.upper, 0);
                assert.equal(rows.some(r => r.kind === 'tool' && r.node === index), false);
                assert.equal(rows.some(r => r.kind === 'external-friction'), false);
                releaseStart = new Map(); accumulate(releaseStart, release.gradients, 1);
                releaseExpected = new Map(expectedSpatial);
                sameBodyWrench(spatialReaction(c, releaseStart), releaseExpected);
                const { force, moment } = wrench(c, releaseStart);
                assert.ok(Math.hypot(...force) < 1e-11);
                assert.ok(Math.hypot(...moment) < 1e-10, 'Every release balances at its current frames and orbital arms');
                releaseNormal = tool.lambdas[index];
                releaseTangent = [...c._coupledExternalFriction.owners.get(tool).get(index).tangentLambda];
                if (releaseCalls === 1) {
                    assert.equal(releaseNormal, firstFn, 'Range/enable changes must not clear normal force');
                    assert.deepEqual(releaseTangent, firstFt);
                }
            } else releaseStart = null;
            const result = solve(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true, includeSystem: true });
            assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
            if (release) assert.equal(result.additionalIncrement[rows.indexOf(release)], -1);
            const system = result.system;
            const deltas = [...result.inner.lambda, ...result.outer.lambda, ...result.contactIncrement, ...result.additionalIncrement];
            // Check full W J^T dLambda, including the transported release and
            // the material reaction it excites, in the original individual
            // coordinates independently of the solver's transformed recovery.
            for (const [side, body] of [wire, catheter].entries()) {
                const correction = (side ? result.outer : result.inner).correction;
                for (let dof = body.activeStart * 6; dof < body.activeEnd * 6 + 3; dof++) {
                    const entries = system.columns[side][dof];
                    let action = 0;
                    for (let k = 0; k < entries.length; k += 2) action += entries[k + 1] * deltas[system.order[entries[k]]];
                    const node = Math.floor(dof / 6), axis = dof % 6;
                    const weight = axis < 3 ? body.inverseMass[node] : body['inverseInertia' + (axis - 2)][node];
                    assert.ok(Math.abs(correction[dof] - weight * action) < 1e-10, `full WJ at ${side}:${dof}`);
                }
            }
            calls++;
            return result;
        },
        apply(c, result) {
            if (releaseStart) {
                sameSpatial(tool._jointReactions.get(index).wrenches, releaseExpected);
                // Includes every backtracked retry at the restored pose.
                sameBodyWrench(spatialReaction(c, releaseStart), releaseExpected);
            }
            if (calls <= loadedPasses) {
                const increment = new Map();
                for (let i = 0; i < rows.length; i++) if (rows[i].kind === 'tool' || rows[i].kind === 'external-friction')
                    accumulate(increment, rows[i].gradients, result.scale * result.additionalIncrement[i]);
                for (const [key, value] of increment) firstReaction.set(key, (firstReaction.get(key) ?? 0) + value);
                for (const [key, value] of spatialReaction(c, increment)) expectedSpatial.set(key, (expectedSpatial.get(key) ?? 0) + value);
                const { force, moment } = wrench(c, increment);
                assert.ok(Math.hypot(...force) < 1e-11);
                assert.ok(Math.hypot(...moment) < 1e-10, 'Orbital plus material spin reaction must balance');
                assert.ok([...firstReaction.entries()].some(([key, value]) => Number(key.split(':')[1]) % 6 >= 3 && Math.abs(value) > 1e-6),
                    'The real load must include a nonzero surface moment');
            }
            const before = [wire, catheter].map(b => xyz.map(key => b[key].slice()));
            const frames = [wire, catheter].map(b => Array.from({ length: b.segmentCount }, (_, i) =>
                new Quaternion(b.orientationX[i], b.orientationY[i], b.orientationZ[i], b.orientationW[i]).normalize()));
            apply(c, result);
            for (const [side, body] of [wire, catheter].entries()) for (let i = body.activeStart; i <= body.activeEnd; i++)
                for (let axis = 0; axis < 3; axis++) assert.ok(Math.abs(body[xyz[axis]][i] - before[side][axis][i] -
                    result.scale * (side ? result.outer : result.inner).correction[i * 6 + axis]) < 1e-6,
                'Both tools receive the actual full-system correction');
            for (const [side, body] of [wire, catheter].entries()) for (let i = body.activeStart; i < body.activeEnd; i++) {
                const correction = (side ? result.outer : result.inner).correction;
                const rotation = new Vector3(...[3, 4, 5].map(axis => result.scale * correction[i * 6 + axis]));
                const angle = rotation.length();
                const expected = frames[side][i].multiply(angle ? new Quaternion().setFromAxisAngle(rotation.divideScalar(angle), angle) : new Quaternion());
                const actual = new Quaternion(body.orientationX[i], body.orientationY[i], body.orientationZ[i], body.orientationW[i]).normalize();
                assert.ok(1 - Math.abs(expected.dot(actual)) < 1e-12, 'Local RIGHT frame update uses the same common scale');
            }
        }
    };
    world.debugJointTrial = (c, state, pass, trial, scale) => {
        if (pass < loadedPasses) {
            sameSpatial(tool._jointReactions.get(index).wrenches, expectedSpatial);
            firstFn = tool.lambdas[index]; firstFt = [...c._coupledExternalFriction.owners.get(tool).get(index).tangentLambda];
            assert.ok(firstFn > 0 && Math.hypot(...firstFt) > 0);
            if (pass + 1 < loadedPasses) return;
            // An operator/configuration transition is introduced BETWEEN real
            // nonlinear iterations, after a genuine normal+friction apply.
            if (transition === 'mouth') tool.openDistalB = true;
            if (transition === 'translated-mouth') {
                // A prescribed transport update crosses the actual mouth while
                // keeping its open flag and the already applied forces intact.
                const dx = catheter.x[catheter.activeEnd] + .1 - wire.x[2];
                const dy = catheter.y[catheter.activeEnd] + .04 - wire.y[2];
                for (let i = 0; i < wire.count; i++) { wire.x[i] += dx; wire.y[i] += dy; }
                joint.containedLength = 10; joint.endNode = 1;
            }
            if (transition === 'range') { tool.startSegmentB = 0; tool.endSegmentB = 0; }
            if (transition === 'empty-range') { tool.startSegmentB = 1; tool.endSegmentB = 0; }
            if (transition === 'disabled') tool.enabled = false;
            if (transition === 'twisted-translated') {
                tool.enabled = false;
                // Independent material spin and nonrigid endpoint translations
                // force BOTH frame and orbital-arm transport, including the
                // terminal node whose solver has no rotational DOFs.
                for (const [side, body] of [wire, catheter].entries()) {
                    for (let node = 0; node < body.count; node++) {
                        body.x[node] += .03 * (side ? -1 : 1) * (node + 1);
                        body.y[node] += .02 * (side + 1) * node;
                        body.z[node] += .04 * (side ? 1 : -1) * (node + 1);
                    }
                    for (let node = 0; node < body.segmentCount; node++) {
                        const q = new Quaternion(body.orientationX[node], body.orientationY[node], body.orientationZ[node], body.orientationW[node]);
                        q.multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), (side ? -.7 : Math.PI / 2))).normalize();
                        ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { body['orientation' + axis][node] = q.toArray()[i]; });
                    }
                }
            }
            joint.enforceDistalPortal = true;
        } else if (releaseStart) {
            releasedTrials++;
            expectedSpatial = new Map([...releaseExpected].map(([key, value]) => [key, value * (1 - scale)]));
            sameSpatial(tool._jointReactions.get(index).wrenches, expectedSpatial);
            assert.ok(Math.abs(tool.lambdas[index] - releaseNormal * (1 - scale)) < 1e-6);
            const ft = c._coupledExternalFriction.owners.get(tool).get(index).tangentLambda;
            for (let axis = 0; axis < 2; axis++) assert.ok(Math.abs(ft[axis] - releaseTangent[axis] * (1 - scale)) < 1e-12);
            if (forceRollback && pass === 1 && trial === 0) {
                // Force one rejection to exercise production rollback of the
                // ledger, old J, Fn/Ft and the changing tool window together.
                state.merit = Number.MAX_VALUE; state.settled = false;
            }
            if (pass === 1 && trial > 0) { retried = true; if (forceRollback) throw stopped; }
        }
    };
    if (forceRollback) {
        assert.throws(() => world.stepFixed(), error => error === stopped);
        assert.ok(retried && releasedTrials > 1);
        return;
    }
    world.stepFixed();
    assert.ok(world.lastCoupledClosureConverged, JSON.stringify(joint._jointTrialFailure ?? joint._jointLinearFailure));
    assert.ok(releaseCalls > 0 && releasedTrials > 0);
    assert.equal(tool.lambdas[index], 0);
    assert.ok(joint._coupledExternalFriction.owners.get(tool).get(index).tangentLambda.every(v => v === 0));
    assert.ok(tool._jointReactions.get(index).wrenches.every(w => spatialKeys.every(key => w[key] === 0)));
    assert.equal(joint._jointToolReleaseResidual.pending, 0);
    assert.ok(joint.kirchhoffSolverResidual <= world.coupledContainmentTolerance);
    assert.ok(joint._jointMaterialResidual.adaptationMm <= world.coupledContainmentTolerance);
    assert.ok(joint._jointMaterialResidual.bendTwistRad <= world.coupledAngularToleranceRad);
});

for (const [branchHeight, loopZ] of [[.04, .02], [.6, .1]]) test(`a returning loop keeps a real external reaction, material branch height ${branchHeight}`, () => {
    const world = new EndovascularPhysicsWorld(), stopped = Symbol('one real trial inspected');
    const points = [[0, branchHeight, 0], [5, branchHeight, 0], [15, branchHeight, 0], [9.99, -1, loopZ], [10.01, 1, loopZ], [15, 1, loopZ]];
    const wire = world.createRod('wire', 6, 5, { radius: .4445, foldLimitStrength: 0 });
    const catheter = world.createRod('catheter', 3, 5, { radius: .8333, foldLimitStrength: 0 });
    points.forEach((p, i) => wire.setNodePosition(i, ...p));
    wire.captureRestConfiguration(); wire.captureKirchhoffRestConfiguration();
    const joint = world.addContainment(wire, catheter, { innerRadius: .485, startNode: 0, endNode: 1,
        containedLength: 10, enforceDistalPortal: true, openDistal: true, searchWindow: 2, axialFriction: 0, torsionalFriction: 0 });
    const tool = world.addToolContact(wire, catheter, { startSegmentA: 3, endSegmentA: 3,
        startSegmentB: 1, endSegmentB: 1, openDistalB: true, friction: 0 });
    world.coupledSystem = { solve(c, dt, options) {
        assert.equal(locate(c).segment, branchHeight < .485 ? 1 : -1);
        assert.equal(c._slidingPortalState.segment, branchHeight < .485 ? 1 : -1);
        assert.ok(c.kirchhoffContacts.every(r => r.kind !== 'portal' || r._innerSegmentIndex !== 3),
            'An uncertified return may not acquire a lumen portal row');
        assert.ok(options.additionalRows.some(r => r.kind === 'tool' && r.segmentA === 3));
        const result = solve(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true });
        assert.ok(result.diagnostics.converged); return result;
    }, apply };
    world.debugJointTrial = () => {
        assert.ok(tool.lambdas[7] > 0, 'The real joint solve must load the returning-loop contact');
        assert.ok(tool._jointReactions.get(7).wrenches.some(w => spatialKeys.some(key => w[key] !== 0)));
        throw stopped;
    };
    assert.throws(() => world.stepFixed(), error => error === stopped);
});

test('branch certification follows a refined offset arc beyond endNode+1 without a larger search window', () => {
    const h = .05, radius = 10.04, count = 635;
    const inner = { activeStart: 628, activeEnd: count - 1, segmentCount: count - 1,
        restLength: new Float64Array(count - 1).fill(h), nodeRadius: new Float64Array(count).fill(.4445),
        x: [], y: [], z: new Float64Array(count) };
    for (let i = 0; i < count; i++) { inner.x[i] = radius * Math.cos(i * h / radius); inner.y[i] = radius * Math.sin(i * h / radius); }
    const outer = { activeStart: 0, activeEnd: 1, segmentCount: 1, x: [-10, -10], y: [.1, 0], z: [0, 0] };
    const joint = { enabled: true, enforceDistalPortal: true, openDistal: true, innerBody: inner, outerBody: outer,
        innerRadius: .485, containedLength: 10 * Math.PI, innerArcOffset: 628 * h, startNode: 628, endNode: 628,
        _slidingPortalState: { segment: 630 } };
    const branch = locate(joint); assert.equal(branch.segment, 630);
    const point = xyz.map(key => (1 - branch.t) * inner[key][630] + branch.t * inner[key][631]);
    const tool = { openDistalB: true, bodyA: inner, bodyB: outer };
    assert.equal(owns(joint, tool, 630, 0, branch.t, ...point), true);
});
