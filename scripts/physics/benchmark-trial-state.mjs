// Compare against a saved source tree with the optional fourth argument.
// Timings exclude rendering and fingerprints; compare both under similar load.
import fs from 'node:fs';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy, poseFingerprint } from '../../tests/helpers/coupledRuntimeFixture.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { ConstraintStageProfile } from '../../src/physics/constraintStageProfile.js';
import { sampleCatheterBrowserBenchmarkCommands, createBrowserBenchmarkCommands } from '../../src/benchmark/browserBenchmarkScenario.js';

const steps = Number(process.argv[2] ?? 2386);
const runtime = process.argv[4] ? pathToFileURL(resolve(process.argv[4]) + '/') : new URL('../../', import.meta.url);
const { EndovascularPhysicsWorld } = await import(new URL('src/physics/endovascularPhysicsWorld.js', runtime));
const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(new URL('src/physics/kirchhoffCoupledSystem.js', runtime));
const profile = new ConstraintStageProfile();
class World extends EndovascularPhysicsWorld {
    stepFixed() {
        for (const body of this.bodies) configureKirchhoffToolRuntime(body);
        const result = super.stepFixed();
        profile.record(this);
        return result;
    }
}
const anatomy = await loadCoupledRuntimeAnatomy();
const fixture = createCoupledRuntimeFixture({ ...anatomy, World, coupledSystem: {
    independentComponents: true,
    physicalTrialState: true,
    earlyTrialRejection: process.env.OET_EARLY_TRIAL_REJECTION !== '0',
    solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt, { ...options, activeCondensation: true, simultaneousCoulomb: true }),
    apply: applyKirchhoffCoupledCorrection
} });
try {
    const commands = createBrowserBenchmarkCommands(), poses = [];
    let maxLengthError = 0, maxPenetration = 0, finite = true;
    for (let i = 0; i < steps; i++) {
        fixture.step(sampleCatheterBrowserBenchmarkCommands(i * 1000 / 120, commands));
        poses.push(fixture.world.bodies.map(poseFingerprint));
        maxPenetration = Math.max(maxPenetration, fixture.world.settledMaxPenetration);
        for (const body of fixture.world.bodies) for (let j = body.activeStart; j < body.activeEnd; j++) {
            const length = Math.hypot(body.x[j + 1] - body.x[j], body.y[j + 1] - body.y[j], body.z[j + 1] - body.z[j]);
            finite &&= Number.isFinite(length);
            maxLengthError = Math.max(maxLengthError, Math.abs(length - body.restLength[j]));
        }
    }
    const report = { steps, runtime: runtime.href, finite, maxLengthError, maxPenetration,
        profile: profile.report(), poses, final: fixture.world.getStats() };
    if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify({ steps, finite, failedSteps: report.profile.failedSteps,
        fields: report.profile.fields, costs: report.final.jointCosts }));
} finally {
    fixture.dispose();
    anatomy.dispose();
}
