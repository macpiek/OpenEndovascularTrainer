// Dedicated native repro: no physics timestep or anatomy is executed.
import assert from 'node:assert/strict';
import test from 'node:test';
import { pathToFileURL } from 'node:url';
const root = process.env.OET_TWO_CHANNEL_SOURCE_ROOT ? pathToFileURL(process.env.OET_TWO_CHANNEL_SOURCE_ROOT + '/') : new URL('../', import.meta.url);
const [{ EndovascularPhysicsWorld }, { solveKirchhoffTwoChannelSystem }] = await Promise.all([
    import(new URL('src/physics/endovascularPhysicsWorld.js', root)), import(new URL('src/physics/kirchhoffTwoChannelSystem.js', root))]);

test('failed equality factor is counted when the dense fallback is already settled', () => {
    const dt = 1 / 120, world = new EndovascularPhysicsWorld();
    const a = world.createRod('accounting-a', 2, 1, { mass: 1, adaptationCompliance: dt * dt });
    const b = world.createRod('accounting-b', 2, 1, { mass: 1, adaptationCompliance: dt * dt });
    a.setPinned(0, true); b.inverseMass.fill(0);
    for (const body of [a, b]) for (const axis of [1, 2, 3]) body['inverseInertia' + axis].fill(0);
    const control = () => ({ kind: 'control', strain: 0, lambda: 0, alpha: 0,
        lower: -Infinity, upper: Infinity, gradients: [{ side: 0, dof: 6, value: 1 }] });
    const constraint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    let denseFactors;
    const result = solveKirchhoffTwoChannelSystem(constraint, dt, {
        includeSystem: true, additionalRows: [control(), control()],
        channels: native => native.rows.map(row => ({ physical: 'pose', bias: {
            channel: 'bias-motion', strain: 0, lambda: 0, alpha: row.alpha, lower: -Infinity, upper: Infinity } })),
        debugCoulombResult: ({ result }) => { denseFactors = result.diagnostics.factorizations; }
    });
    assert.equal(result.diagnostics.converged, true);
    assert.equal(result.diagnostics.reconstructionResidual, 0);
    assert.equal(result.system.condensation.reason, 'unsafe-equality-factor');
    assert.equal(result.system.condensation.minimumScaledPivot, 1e-12);
    assert.equal(denseFactors, 0);
    // unsafe-equality-factor is returned only AFTER the attempted Wasm factor.
    assert.equal(result.diagnostics.factorizations, denseFactors + 1,
        'count the failed Ae factor as well as factors from the dense fallback');
});
