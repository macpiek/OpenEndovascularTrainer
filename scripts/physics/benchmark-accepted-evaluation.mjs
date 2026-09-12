import fs from 'node:fs';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { measureKirchhoffCoupledFrictionResidual } from '../../src/physics/kirchhoffCoupledFrictionRows.js';

class World extends EndovascularPhysicsWorld {
    stepFixed() {
        for (const body of this.bodies) configureKirchhoffToolRuntime(body);
        return super.stepFixed();
    }
}
const dt = 1 / 120;
const fixture = createCoupledRuntimeFixture({ ...await loadCoupledRuntimeAnatomy(), World,
    coupledSystem: { independentComponents: true, physicalTrialState: true, earlyTrialRejection: true,
        solve(c, step, options) { return solveKirchhoffCoupledSystem(c, step,
            { ...options, activeCondensation: true, simultaneousCoulomb: true }); },
        apply: applyKirchhoffCoupledCorrection } });
for (const [tool, distance, rate] of [['guidewire', 309, 44], ['catheter', 100, 52]]) {
    for (let i = 0; i < Math.ceil(distance / (rate * dt)); i++)
        fixture.step({ [tool + 'Advance']: Math.min(1, (distance - i * rate * dt) / (rate * dt)) });
}
const c = fixture.containment, outputs = [{}, {}];
const options = [
    [{ cacheInputs: false }, { cacheInputs: false }],
    [{ cacheInputs: true }, { cacheInputs: true, reuseInputs: true }]
];
// One candidate evaluation plus its next-iteration evaluation. Alternate the
// two variants within a single process on the same loaded physical state.
function pairs(variant, count) {
    const out = outputs[variant];
    let reuseCount = 0;
    const t = performance.now();
    for (let i = 0; i < count; i++) {
        measureKirchhoffCoupledFrictionResidual(c, dt, out, options[variant][0]);
        measureKirchhoffCoupledFrictionResidual(c, dt, out, options[variant][1]);
        reuseCount += Number(out.reusedEvaluation);
    }
    return { msPerPair: (performance.now() - t) / count, reuseCount };
}
for (let i = 0; i < 20; i++) { pairs(i % 2, 10); }
const samples = [];
for (let i = 0; i < 80; i++) {
    const order = i % 2 ? [1, 0] : [0, 1], row = {};
    for (const variant of order) row[variant ? 'after' : 'before'] = pairs(variant, 16);
    samples.push(row);
}
const summary = key => {
    const xs = samples.map(x => x[key].msPerPair).sort((a, b) => a - b);
    return { mean: xs.reduce((a, b) => a + b, 0) / xs.length,
        median: xs[Math.floor(xs.length / 2)], p95: xs[Math.ceil(xs.length * .95) - 1] };
};
const report = { scope: 'Frozen real-anatomy contact state, residual candidate + repeated base evaluation only; not full-step timing or FPS',
    wireMm: fixture.transport.progress, catheterMm: fixture.catheter.progress,
    contacts: c.kirchhoffContacts.length, before: summary('before'), after: summary('after'), samples };
fs.writeFileSync(process.argv[2] ?? '/tmp/oet-accepted-micro.json', JSON.stringify(report, null, 2));
console.log(JSON.stringify({ ...report, samples: undefined }));
