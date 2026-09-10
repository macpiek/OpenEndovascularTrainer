import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture } from '../../tests/helpers/coupledRuntimeFixture.js';
import { createCoupledSolverSelection } from '../../src/physics/coupledSolverSelection.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { solveKirchhoffTwoChannelSystem } from '../../src/physics/kirchhoffTwoChannelSystem.js';
import { configureKirchhoffSplitBias } from '../../src/physics/kirchhoffSplitMotion.js';
import { captureKirchhoffSplitStep, restoreKirchhoffSplitStep } from '../../src/physics/kirchhoffSplitStepTransaction.js';

const output = process.argv[2] ?? '/tmp/oet-two-channel-frozen-runtime.json';
const paths = fs.readdirSync(new URL('../../src/physics/', import.meta.url)).filter(p => p.endsWith('.js'))
    .map(p => 'src/physics/' + p).concat(['tests/helpers/coupledRuntimeFixture.js', 'scripts/physics/probe-two-channel-frozen-runtime.mjs']).sort();
const hashes = () => Object.fromEntries(paths.map(p => [p, createHash('sha256').update(fs.readFileSync(new URL('../../' + p, import.meta.url))).digest('hex')]));
const report = { scope: 'Dry simultaneous direction on the real short-insertion fixture; the candidate is never applied, and the original split runtime continues after rollback.',
    sourceBefore: hashes(), frozen: null };
let fixture;
const selection = createCoupledSolverSelection('joint-active-coulomb', {
    solve(c, dt, options) {
        const original = solveKirchhoffCoupledSystem(c, dt, options);
        if (report.frozen || fixture.catheter.progress < 7 || c._splitMotion?.phase !== 'physical' || c._splitMotion.diagnostics.physicalPasses !== 5)
            return original;
        const snapshot = captureKirchhoffSplitStep(fixture.world);
        try {
            const frictionRows = new Set(options.groups.flatMap(group => Array.from(group.rows ?? group.rowIndices)));
            const started = performance.now();
            const twoChannelOptions = { ...options, includeSystem: true,
                channels(native) {
                    return native.rows.slice(0, native.count).map((row, originalIndex) => {
                        const penalty = () => ({ physical: 'pose', bias: { channel: 'bias-motion', strain: 0,
                            alpha: row.alpha, lambda: 0, lower: -Infinity, upper: Infinity } });
                        const geometry = strain => ({ physical: 'physical-motion', bias: { channel: 'pose', strain,
                            alpha: row.alpha, lambda: 0, lower: 0, upper: Infinity } });
                        if (row.kind === 'material') return penalty();
                        if (row.kind === 'normal') return geometry(c.kirchhoffContacts[row.local]._splitActualGap);
                        const index = originalIndex - native.additionalOffset, source = options.additionalRows[index];
                        if (frictionRows.has(index)) return { physical: 'physical-motion', bias: null };
                        if (['control', 'orientation-control'].includes(source.kind)) return penalty();
                        if (source.kind === 'fold') return { physical: 'pose', bias: null };
                        if (source.kind === 'tool-release') return { physical: 'physical-motion', bias: null };
                        if (['wall', 'tool', 'sheath', 'split-sweep', 'split-point-wall'].includes(source.kind)) return geometry(source._splitActualStrain);
                        throw new Error('Unclassified runtime row: ' + source.kind);
                    });
                } };
            // Keep the complete reference matrix in the replay artifact.
            // The second solve below exercises the integrated condensed path.
            const candidate = solveKirchhoffTwoChannelSystem(c, dt, { ...twoChannelOptions, condensation: 'none' });
            report.frozen = { stepCount: fixture.world.stepCount, catheterMm: fixture.catheter.progress,
                physicalPass: c._splitMotion.diagnostics.physicalPasses, original: { ...original.diagnostics },
                candidate: { ...candidate.diagnostics }, solveMs: performance.now() - started, scale: candidate.scale,
                physicalMaximum: candidate.physical.map(r => Math.max(...r.correction.map(Math.abs))),
                biasMaximum: candidate.bias.map(r => Math.max(...r.correction.map(Math.abs))),
                matrixCount: candidate.system.count,
                physicalNormalIncrement: [...candidate.contactIncrement], biasNormalIncrement: [...candidate.biasContactIncrement] };
            const native = candidate.system.native;
            const frozenInput = { schema: 'kirchhoff-two-channel-frozen-v1', channels: candidate.system.descriptors,
                native: { count: native.count, band: native.band,
                    matrix: [...native.matrix], rhs: [...native.rhs], lower: [...native.lower], upper: [...native.upper],
                    order: [...native.order], inverseOrder: [...native.inverseOrder],
                    rows: native.rows.slice(0, native.count).map(r => ({ kind: r.kind, alpha: r.alpha, lambda: r.lambda })),
                    groups: native.groups },
                oracle: { matrix: [...candidate.system.matrix], rhs: [...candidate.system.rhs],
                    physicalIncrement: [...candidate.physicalIncrement], biasIncrement: [...candidate.biasIncrement] } };
            report.frozen.inputFile = output.replace(/\.json$/, '') + '.system.json';
            fs.writeFileSync(report.frozen.inputFile, JSON.stringify(frozenInput,
                (_, value) => typeof value === 'number' && !Number.isFinite(value) ? String(value) : value) + '\n');
            const condensedStarted = performance.now();
            const condensed = solveKirchhoffTwoChannelSystem(c, dt, twoChannelOptions);
            report.frozen.integratedCondensation = { ...condensed.diagnostics,
                solveMs: performance.now() - condensedStarted, scale: condensed.scale,
                expandedMatrixAllocated: condensed.system.matrix !== null,
                physicalMaximum: condensed.physical.map(r => Math.max(...r.correction.map(Math.abs))),
                biasMaximum: condensed.bias.map(r => Math.max(...r.correction.map(Math.abs))) };
        } finally { restoreKirchhoffSplitStep(fixture.world, snapshot); }
        return original;
    }, apply: applyKirchhoffCoupledCorrection
});
fixture = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: 'split-physical-bias' });
configureKirchhoffSplitBias(fixture.containment, { materialMode: 'preserve-strain' });
try {
    for (let i = 0; i < 33; i++) fixture.step({ guidewireAdvance: 1 });
    for (let i = 0; i < 40; i++) if (fixture.step({ catheterAdvance: 1 }).accepted === false) break;
    report.final = { state: fixture.snapshot(), stepCount: fixture.world.stepCount, jointMotion: fixture.world.getStats().jointMotion };
    report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
    fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
    if (!report.frozen) throw new Error('The requested frozen physical state was not reached');
    console.log(JSON.stringify({ output, sourceStable: report.sourceStable, frozen: report.frozen, final: report.final.state }, null, 2));
} finally { fixture.dispose(); }
