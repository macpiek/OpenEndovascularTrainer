import { performance } from 'node:perf_hooks';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../../src/physics/applyKirchhoffMaterialProfile.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
const world = new EndovascularPhysicsWorld();
const bodies = [201, 197].map((count, side) => {
    const body = world.createRod(`benchmark-${side}`, count, 5, { mass: side ? 0.07 : 0.03, sleepFrames: 1e6 });
    applyKirchhoffMaterialProfile(body, 'berenstein');
    body.restRotation1.fill(0); body.restRotation2.fill(0); body.setPinned(0, true);
    body.setProximalOrientationControl(body.orientationX[0], body.orientationY[0], body.orientationZ[0], body.orientationW[0], 0, 0);
    return body;
});
const constraint = { innerBody: bodies[0], outerBody: bodies[1], kirchhoffContacts: Array.from({ length: 479 }, (_, i) => {
    const u = 2 + i / 478 * 192, s = Math.floor(u), t = u - s;
    return { _innerSegmentIndex: s, _outerSegmentIndex: s, innerWeights: [1 - t, t], outerWeights: [1 - t, t],
        normal: [0, 1, 0], gap: -0.001 * (1 + Math.sin(u * 0.1)), _normalAlpha: 0, manifoldContact: { normalLambda: 0 } };
}) };
const solve = () => solveKirchhoffCoupledSystem(constraint, 1 / 120, { tolerance: 1e-7 });
const quantile = (a, q) => [...a].sort((x, y) => x - y)[Math.min(a.length - 1, Math.floor(q * a.length))];
function measure(label, passes) {
    const times = []; let result;
    for (let i = 0; i < passes; i++) {
        const start = performance.now(); result = solve(); times.push(performance.now() - start);
        if (!result.diagnostics.converged) throw new Error(JSON.stringify(result.diagnostics));
    }
    return { label, passes, meanMs: times.reduce((a, b) => a + b, 0) / times.length, medianMs: quantile(times, 0.5), p95Ms: quantile(times, 0.95), diagnostics: result.diagnostics, result };
}
const cold = measure('first solve, all hard rows penetrate', 1);
const repeated = measure('same unsolved state, JIT and workspace warm', 20);
applyKirchhoffCoupledCorrection(constraint, repeated.result);
constraint.kirchhoffContacts.forEach((r, i) => {
    r.manifoldContact.normalLambda += repeated.result.scale * repeated.result.contactIncrement[i];
    const s = r._innerSegmentIndex, t = r.innerWeights[1];
    r.gap += (1 - t) * (bodies[1].y[s] - bodies[0].y[s]) + t * (bodies[1].y[s + 1] - bodies[0].y[s + 1]);
});
const warm = measure('loaded state, frozen normal stencil', 20);
// Real world semantics reset XPBD multipliers at the next fixed step. Keep
// only identity-aware boolean hints, then solve nonzero material residuals.
for (const body of bodies) for (const key of ['adaptationLambdaX', 'adaptationLambdaY', 'adaptationLambdaZ',
    'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3']) body[key].fill(0);
for (const r of constraint.kirchhoffContacts) r.manifoldContact.normalLambda = 0;
const reset = measure('loaded geometry after all force multipliers reset', 20);
bodies[0].y[190] += 0.0005;
for (const r of constraint.kirchhoffContacts) {
    if (r._innerSegmentIndex === 190) r.gap -= r.innerWeights[0] * 0.0005;
    if (r._innerSegmentIndex === 189) r.gap -= r.innerWeights[1] * 0.0005;
}
const perturbed = measure('perturbed loaded geometry after multiplier reset', 20);
for (const item of [cold, repeated, warm, reset, perturbed]) delete item.result;
console.log(JSON.stringify({ fixture: 'Synthetic straight full rods, 479 redundant linear hard normal rows; all 2382 rod dofs retained',
    limitation: 'Microbenchmark only; excludes geometry collection, world iterations, vessel contact, integration, rendering; repeated state is not dynamic replay', cold, repeated, warm, reset, perturbed }, null, 2));
