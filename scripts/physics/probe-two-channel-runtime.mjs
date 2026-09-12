import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture } from '../../tests/helpers/coupledRuntimeFixture.js';
import { createCoupledSolverSelection } from '../../src/physics/coupledSolverSelection.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { configureKirchhoffSplitBias } from '../../src/physics/kirchhoffSplitMotion.js';
import { solveKirchhoffTwoChannelSystem } from '../../src/physics/kirchhoffTwoChannelSystem.js';

const root = new URL('../../', import.meta.url), output = process.argv[2] ?? '/tmp/oet-two-channel-runtime.json';
const paths = fs.readdirSync(new URL('src/physics/', root)).filter(p => p.endsWith('.js')).map(p => 'src/physics/' + p)
    .concat(['src/pigtailCatheter.js', 'src/vesselGeometry.js', 'tests/helpers/coupledRuntimeFixture.js',
        'scripts/physics/probe-two-channel-runtime.mjs']).sort();
const hashes = () => Object.fromEntries(paths.map(p => [p, createHash('sha256').update(fs.readFileSync(new URL(p, root))).digest('hex')]));
const report = { scope: 'Actual joint two-channel World timesteps, short synthetic-vessel insertion using the application actuation adapter. No browser FPS or full-anatomy claim.',
    sourceBefore: hashes(), steps: [] };
const selection = createCoupledSolverSelection('joint-two-channel', {
    solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection, solveTwoChannel: solveKirchhoffTwoChannelSystem
});
const fixture = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: selection.jointMotionMode });
configureKirchhoffSplitBias(fixture.containment, { materialMode: selection.biasMaterialMode });
fixture.world.captureCoupledClosureTrace = true;
try {
    for (let i = 0; i < 33; i++) fixture.step({ guidewireAdvance: 1 });
    report.prepared = fixture.snapshot();
    for (let i = 0; i < 40; i++) {
        const step = fixture.step({ catheterAdvance: 1 });
        const d = fixture.world.getStats().jointMotion;
        report.steps.push({ ...step, twoChannel: d?.twoChannel ?? false, jointPasses: d?.jointPasses,
            historyCommits: d?.historyCommits, physicalConeViolation: d?.physicalConeViolation,
            coneFilterAcceptances: structuredClone(d?.coneFilterAcceptances ?? []),
            linearRefinements: structuredClone(d?.linearRefinements ?? []),
            physicalFailure: d?.physicalFailure, channels: d?.finalChannelResidual });
        if (step.accepted === false) break;
    }
} catch (error) {
    report.error = { message: error.message, stack: error.stack };
} finally {
    report.final = { state: fixture.snapshot(), diagnostics: fixture.world.getStats().jointMotion,
        trace: structuredClone(fixture.world.coupledClosureTrace) };
    report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
    fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
    fixture.dispose();
}
console.log(JSON.stringify({ output, sourceStable: report.sourceStable, state: report.final.state,
    acceptedCoupledSteps: report.steps.filter(s => s.twoChannel && s.accepted !== false).length,
    last: report.steps.at(-1), error: report.error }, null, 2));
