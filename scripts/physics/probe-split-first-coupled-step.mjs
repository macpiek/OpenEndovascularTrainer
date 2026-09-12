import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture } from '../../tests/helpers/coupledRuntimeFixture.js';
import { createCoupledSolverSelection } from '../../src/physics/coupledSolverSelection.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { configureKirchhoffSplitBias, kirchhoffSplitRowMotion } from '../../src/physics/kirchhoffSplitMotion.js';

const output = process.argv[2] ?? '/tmp/oet-split-first-coupled-step.json';
const coupledStepLimit = Number(process.argv[3] ?? 1);
if (!Number.isInteger(coupledStepLimit) || coupledStepLimit < 1 || coupledStepLimit > 120)
    throw new RangeError('Use 1..120 consecutive coupled steps');
const hash = path => createHash('sha256').update(fs.readFileSync(new URL('../../' + path, import.meta.url))).digest('hex');
const paths = fs.readdirSync(new URL('../../src/physics/', import.meta.url)).filter(p => p.endsWith('.js'))
    .map(p => 'src/physics/' + p).concat(['tests/helpers/coupledRuntimeFixture.js', 'scripts/physics/probe-split-first-coupled-step.mjs']).sort();
const hashes = () => Object.fromEntries(paths.map(p => [p, hash(p)]));
const selection = createCoupledSolverSelection('joint-active-coulomb', {
    solve(c, dt, options) {
        const result = solveKirchhoffCoupledSystem(c, dt, options);
        report.solves.push({ phase: c._splitMotion?.phase, diagnostics: { ...result.diagnostics },
            additional: options.additionalRows.filter(r => Math.abs(r.strain) > 1e-8 || Math.abs(r.lambda) > 1e-8)
                .map(r => ({ kind: r.kind, side: r.side, node: r.node, strain: r.strain, actual: r._splitActualStrain,
                    start: r._splitStartGap, alpha: r.alpha, lambda: r.lambda })) });
        return result;
    }, apply: applyKirchhoffCoupledCorrection
});
const fixture = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: 'split-physical-bias' });
configureKirchhoffSplitBias(fixture.containment, { materialMode: 'preserve-strain' });
const { world, containment } = fixture;
const report = { scope: 'First coupled step on a synthetic vessel without anatomy field; read-only phase observations, no injected geometry or force changes.',
    sourceBefore: hashes(), dt: world.fixedDt, coupledStepLimit, coupledAccepted: 0,
    trace: [], solves: [], steps: [], preparationAttempts: 0, catheterAttempts: 0 };
function contacts() {
    return containment.kirchhoffContacts.map(r => ({ id: r.id, contactId: r.manifoldContact?.id, kind: r.kind,
        innerMaterial: r.manifoldContact?.innerMaterial, outerMaterial: r.manifoldContact?.outerMaterial,
        innerSegment: r._innerSegmentIndex, outerSegment: r._outerSegmentIndex, innerT: r.innerT, outerT: r.outerT,
        actualGap: r._splitActualGap, physicalGap: r.gap, startGap: r._splitStartGap,
        physicalNormalMotion: r._splitMotionGradients ? kirchhoffSplitRowMotion(containment, r._splitMotionGradients) : null,
        normal: [...r.normal], lambda: r.manifoldContact?.normalLambda,
        tangent: r.manifoldContact ? [...r.manifoldContact.tangentLambda] : [] }));
}
world.debugJointTrial = (c, state, pass, trial, scale) => {
    report.trace.push({ phase: state.motionPhase, pass, trial, scale, settled: state.settled,
        merit: state.merit, contacts: contacts(), material: { ...state.materialResidual } });
};
try {
    // Prepare 12.1 mm of wire before advancing the catheter. Every command
    // travels through the regular fixture adapter before the physical step.
    for (let i = 0; i < 33; i++) { fixture.step({ guidewireAdvance: 1 }); report.preparationAttempts++; }
    report.prepared = fixture.snapshot();
    for (let i = 0; i < 20 + coupledStepLimit; i++) {
        const result = fixture.step({ catheterAdvance: 1 }); report.catheterAttempts++;
        const diagnostics = world.getStats().jointMotion;
        if (diagnostics) {
            if (result.accepted !== false && diagnostics.certified) report.coupledAccepted++;
            report.steps.push({ state: fixture.snapshot(), accepted: result.accepted !== false,
                stepCount: world.stepCount, physicalPasses: diagnostics.physicalPasses,
                biasPasses: diagnostics.biasPasses, biasInitialStateSettled: diagnostics.biasInitialStateSettled,
                physicalKKTResidualMm: diagnostics.physicalKKTResidualMm,
                finalMaterialResidual: diagnostics.finalMaterialResidual,
                unverifiedHistoryKinds: diagnostics.unverifiedHistoryKinds,
                factorizations: world.lastJointFactorizations, stepMs: world.timings.total.last });
        }
        if (result.accepted === false || report.coupledAccepted >= coupledStepLimit) break;
    }
    report.final = { state: fixture.snapshot(), jointMotion: world.getStats().jointMotion,
        stepCount: world.stepCount, contacts: contacts(), linearFailure: containment._jointLinearFailure,
        trialFailure: containment._jointTrialFailure };
    report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
    fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify({ output, prepared: report.prepared, catheterAttempts: report.catheterAttempts,
        final: report.final, sourceStable: report.sourceStable }, null, 2));
} finally { fixture.dispose(); }
