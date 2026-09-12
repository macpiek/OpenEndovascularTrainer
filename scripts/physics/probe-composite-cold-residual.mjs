import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { createCompositeColdMaterialFixture } from '../../tests/helpers/compositeColdMaterialFixture.js';
import { solveCompositeChainIncrement } from '../../src/physics/kirchhoffCompositeChain.js';

const output = process.argv[2] ?? '/tmp/oet-composite-cold-residual.json';
const sourcePaths = ['src/physics/kirchhoffCompositeChain.js', 'src/physics/kirchhoffCompositeMaterialCache.js',
    'src/physics/kirchhoffCompositeMesh.js', 'src/physics/kirchhoffCompositeElementFast.js',
    'src/physics/kirchhoffCompositeElementFastKernelBytes.js', 'tests/helpers/compositeColdMaterialFixture.js',
    'tests/fixtures/compositeColdMaterialReference.json', 'scripts/physics/probe-composite-cold-residual.py',
    'scripts/physics/probe-composite-cold-residual.mjs', 'tests/kirchhoffCompositeChainRefinement.test.js'];
const sourceHashes = () => Object.fromEntries(sourcePaths.map(p => [p, createHash('sha256')
    .update(fs.readFileSync(new URL('../../' + p, import.meta.url))).digest('hex')]));
const sourceBefore = sourceHashes(), cases = {}, hashes = {}, timings = {};
for (const elementBackend of ['javascript', 'wasm']) {
    const fixture = createCompositeColdMaterialFixture({ elementBackend });
    const w = fixture.workspace, baseline = solveCompositeChainIncrement(w, { diagonal: fixture.diagonal, tolerance: 1e-7, maxRefinementSteps: 0 });
    const unrefined = Array.from(baseline.increment), initialResult = { converged: baseline.converged,
        maximumResidual: baseline.maximumResidual, refinementSteps: baseline.refinementSteps };
    const result = solveCompositeChainIncrement(w, { diagonal: fixture.diagonal, tolerance: 1e-7 });
    cases[elementBackend] = { n: w.layout.dofCount, band: w.layout.band, matrix: Array.from(w.matrix), rhs: Array.from(w.rhs),
        increments: { unrefined, application: Array.from(result.increment) }, initialResult, result: { converged: result.converged,
            maximumResidual: result.maximumResidual, factorizations: result.factorizations, refinementSteps: result.refinementSteps,
            linearSolves: result.linearSolves, initialMaximumResidual: result.initialMaximumResidual } };
    hashes[elementBackend] = createHash('sha256').update(Buffer.from(w.matrix.buffer, w.matrix.byteOffset, w.matrix.byteLength))
        .update(Buffer.from(w.rhs.buffer, w.rhs.byteOffset, w.rhs.byteLength)).digest('hex');
    const rows = [];
    for (let pair = 0; pair < 10; pair++) {
        const row = {};
        for (const maxRefinementSteps of pair % 2 ? [2, 0] : [0, 2]) {
            const start = performance.now(), measured = solveCompositeChainIncrement(w, {
                diagonal: fixture.diagonal, tolerance: 1e-7, maxRefinementSteps });
            row[maxRefinementSteps ? 'refined' : 'unrefined'] = { elapsedMs: performance.now() - start,
                converged: measured.converged, refinementSteps: measured.refinementSteps,
                maximumResidual: measured.maximumResidual, factorizations: measured.factorizations, linearSolves: measured.linearSolves };
        }
        if (pair >= 4) rows.push(row);
    }
    const median = kind => { const v = rows.map(r => r[kind].elapsedMs).sort((a, b) => a - b); return (v[2] + v[3]) / 2; };
    timings[elementBackend] = { scope: 'Paired whole solve after assembly. Both use accurate residual. Only extra refinement differs; unrefined attempts remain failed.',
        warmupPairs: 4, measuredPairs: 6, medianUnrefinedMs: median('unrefined'), medianRefinedMs: median('refined'), rows };
}
const python = spawnSync('python3', [new URL('./probe-composite-cold-residual.py', import.meta.url).pathname], {
    input: JSON.stringify(cases), encoding: 'utf8', maxBuffer: 10 * 1024 * 1024 });
if (python.status !== 0) throw new Error(python.stderr || `Decimal oracle exited ${python.status}`);
const oracle = JSON.parse(python.stdout), report = { scope: 'Frozen full original cold310.017 matrices/RHS, independent 80-digit LDL solution; same unshifted operator and residual norm.',
    date: new Date().toISOString(), hashes, cases, oracle, timings, sourceBefore, sourceAfter: sourceHashes() };
report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, hashes, sourceStable: report.sourceStable,
    timings: Object.fromEntries(Object.entries(timings).map(([k, { rows, ...rest }]) => [k, rest])),
    cases: Object.fromEntries(Object.entries(cases).map(([name, c]) => [name, {
    result: c.result, highPrecisionMaximumOriginalResidual: oracle[name].highPrecisionMaximumOriginalResidual,
    roundedHighPrecisionMaximumOriginalResidual: oracle[name].roundedHighPrecisionMaximumOriginalResidual,
    candidate: Object.fromEntries(Object.entries(oracle[name].candidates.application).filter(([k]) => k !== 'residual')) }])) }, null, 2));
