// Paired measurement: alternate variant order each step to reduce background
// load/order bias. Both use the same anatomy and built-in mechanics, no renderer.
import fs from 'node:fs';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy, poseFingerprint } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { ConstraintStageProfile } from '../../src/physics/constraintStageProfile.js';
import { sampleCatheterBrowserBenchmarkCommands, createBrowserBenchmarkCommands } from '../../src/benchmark/browserBenchmarkScenario.js';

const steps = Number(process.argv[2] ?? 2386), anatomy = await loadCoupledRuntimeAnatomy();
const variants = [];
try {
    for (const earlyTrialRejection of [false, true]) {
        const profile = new ConstraintStageProfile();
        class World extends EndovascularPhysicsWorld {
            stepFixed() {
                for (const body of this.bodies) configureKirchhoffToolRuntime(body);
                const result = super.stepFixed(); profile.record(this); return result;
            }
        }
        const fixture = createCoupledRuntimeFixture({ ...anatomy, World, coupledSystem: {
            independentComponents: true, physicalTrialState: true, earlyTrialRejection,
            solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt, { ...options,
                activeCondensation: true, simultaneousCoulomb: true }), apply: applyKirchhoffCoupledCorrection
        } });
        variants.push({ fixture, profile, earlyTrialRejection });
    }
    const commands = createBrowserBenchmarkCommands(), differences = [];
    for (let i = 0; i < steps; i++) {
        sampleCatheterBrowserBenchmarkCommands(i * 1000 / 120, commands);
        variants[i % 2].fixture.step(commands);
        variants[1 - i % 2].fixture.step(commands);
        const [before, after] = variants.map(v => v.fixture.world);
        const posesEqual = before.bodies.every((body, side) => poseFingerprint(body) === poseFingerprint(after.bodies[side]));
        const decisionsEqual = ['accepted', 'rejected'].every(key => before.lastJointLineSearch[key]
            .every((count, j) => count === after.lastJointLineSearch[key][j]));
        if (!posesEqual || !decisionsEqual) differences.push({ step: i, posesEqual, decisionsEqual });
    }
    const report = { steps, method: 'Interleaved fixed steps, alternating variant order, shared read-only anatomy, no rendering',
        differences, variants: variants.map(v => ({ earlyTrialRejection: v.earlyTrialRejection,
            profile: v.profile.report(), final: v.fixture.world.getStats() })) };
    if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report));
} finally {
    for (const variant of variants) variant.fixture.dispose();
    anatomy.dispose();
}
