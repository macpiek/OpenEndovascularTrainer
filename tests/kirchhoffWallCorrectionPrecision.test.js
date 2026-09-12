import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffCoupledBoundaryStep, collectKirchhoffCoupledBoundaryRows,
    applyKirchhoffCoupledBoundaryMultipliers, measureKirchhoffCoupledBoundaryResidual } from '../src/physics/kirchhoffCoupledBoundaryRows.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';

test('both production tools retain the same precise position history at initialization', () => {
    const world = new EndovascularPhysicsWorld();
    const p = [-234 + 2e-6, 512 - 2e-6, 11.5 + 2e-7];
    for (const id of ['guidewire', 'catheter']) {
        const body = world.createRod(id, 2, 4);
        body.setNodePosition(0, ...p);
        assert.deepEqual([body.x[0], body.y[0], body.z[0]], p);
        assert.deepEqual([body.previousX[0], body.previousY[0], body.previousZ[0]], p);
    }
    const experimental = new EndovascularPhysicsWorld({ jointMotionMode: 'split-physical-bias' });
    assert.ok(experimental.createRod('split', 2, 4).x instanceof Float32Array,
        'experimental split motion retains its separate history contract');
});

// At anatomical coordinates, a valid normal correction can be smaller than
// one Float32 ULP. The solver must apply it, not only its associated impulse.
for (const origin of [0, -234, 512]) for (const scale of [1, .125]) {
    test(`wall correction and its reaction survive application/rollback at y=${origin}, scale=${scale}`, () => {
        const world = new EndovascularPhysicsWorld();
        const body = world.createRod('catheter', 4, 4, { radius: .5, mass: 1, wallCompliance: 0 });
        for (let n = 0; n < body.count; n++) body.setNodePosition(n, n * 4, origin, 0);
        body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
        const c = { bodies: [body], kirchhoffContacts: [] }, dt = 1 / 120;
        beginKirchhoffCoupledBoundaryStep(c);
        const penetration = 2e-6;
        for (let n = 0; n < body.segmentCount; n++) {
            body.wallActive[n] = 1; body.wallT[n] = .37;
            body.wallNormalY[n] = 1;
            body.wallY[n] = origin - body.nodeRadius[n] + penetration;
        }
        const rows = collectKirchhoffCoupledBoundaryRows(c, [], dt);
        const base = rows.map(row => row.strain);
        for (const gap of base) assert.ok(Math.abs(gap + penetration) < 1e-12, `contact point quantization: ${gap}`);
        const result = solveKirchhoffCoupledSystem(c, dt, { additionalRows: rows,
            activeCondensation: true, simultaneousCoulomb: true, tolerance: 1e-10 });
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        const predicted = rows.map(row => row.gradients.reduce((sum, g) =>
            sum + g.value * result.responses[g.side].correction[g.dof], 0));
        const snapshot = captureKirchhoffCoupledTrialState(c);
        const history = [...body.previousY];
        applyKirchhoffCoupledCorrection(c, result, scale);
        applyKirchhoffCoupledBoundaryMultipliers(c, result.additionalIncrement, scale);
        const applied = collectKirchhoffCoupledBoundaryRows(c, [], dt);
        applied.forEach((row, i) => assert.ok(Math.abs(row.strain - base[i] - scale * predicted[i]) < 1e-12,
            `applied normal motion disagrees with the solved direction: ${row.strain - base[i]} vs ${scale * predicted[i]}`));
        assert.ok(measureKirchhoffCoupledBoundaryResidual(applied) < penetration * (1 - scale) + 1e-10);
        assert.ok(body.wallLambda.some(value => value > 0), 'the real solve must generate wall reactions');
        assert.deepEqual([...body.previousY], history, 'constraint application does not rewrite velocity history');
        restoreKirchhoffCoupledTrialState(snapshot);
        assert.deepEqual([...body.y], Array(body.count).fill(origin));
        assert.deepEqual([...body.previousY], history);
        assert.ok(body.wallLambda.every(value => value === 0));
    });
}

test('contact interpolation and unit normal retain a sub-ULP signed gap at anatomical coordinates', () => {
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('guidewire', 2, 4, { radius: .4 });
    const normal = [1 / Math.sqrt(14), 2 / Math.sqrt(14), 3 / Math.sqrt(14)];
    const a = [-4.9, -236.12, 11.5], b = [-3.99, -234.25, 9.07], t = .37123456789;
    body.setNodePosition(0, ...a); body.setNodePosition(1, ...b);
    body.wallActive[0] = 1; body.wallT[0] = t;
    const gap = -2e-7, radius = body.nodeRadius[0];
    for (let axis = 0; axis < 3; axis++) {
        const key = ['X', 'Y', 'Z'][axis];
        body[`wallNormal${key}`][0] = normal[axis];
        body[`wall${key}`][0] = (1 - t) * a[axis] + t * b[axis] - (radius + gap) * normal[axis];
    }
    const c = { bodies: [body] };
    beginKirchhoffCoupledBoundaryStep(c);
    const [row] = collectKirchhoffCoupledBoundaryRows(c, [], 1 / 120);
    assert.ok(Math.abs(row.strain - gap) < 1e-12, `${row.strain} vs ${gap}`);
});

test('captured coaxial wire at an oblique catheter mouth has a consistent unit fillet normal', () => {
    const stop = new Error('captured assembled contact');
    let checked = 0;
    const world = new EndovascularPhysicsWorld({ coupledSystem: {
        solve(c) {
            for (const record of c.kirchhoffContacts) if (record.kind === 'distal-fillet') {
                assert.ok(Math.abs(Math.hypot(...record.normal) - 1) < 1e-12);
                assert.ok(record.normalGradientDiagnostics.normalMismatch < 1e-7);
                checked++;
            }
            throw stop;
        }, apply: applyKirchhoffCoupledCorrection
    } });
    const inner = world.createRod('wire', 2, 5, { radius: .4445, sleepFrames: 1e6 });
    const outer = world.createRod('catheter', 2, 4, { radius: .8333333, sleepFrames: 1e6 });
    const positions = [
        [[-83.6763368654905, -425.705347461962, 23.341794757304182],
            [-82.49007721376931, -420.96030885507724, 22.303817562048145]],
        [[-84.59371099615507, -429.37484398462027, 24.14449712163568],
            [-83.64470327477812, -425.5788130991125, 23.31411536543085]]
    ];
    [inner, outer].forEach((body, side) => positions[side].forEach((p, n) => body.setNodePosition(n, ...p)));
    world.addContainment(inner, outer, { innerRadius: .485, portalFilletRadius: .15,
        enforceDistalPortal: true, openDistal: true, containedLength: 4 });
    assert.throws(() => world.stepFixed(), error => error === stop);
    assert.ok(checked > 0, 'must reach the same on-axis fillet branch as the captured runtime');
});
