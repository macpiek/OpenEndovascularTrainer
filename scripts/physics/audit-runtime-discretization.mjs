import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { resolve, relative } from 'node:path';
import { fileURLToPath } from 'node:url';

// Offline arithmetic on a saved state. Does not import a simulator, launch a
// replay, change discretization, or certify omitted mechanical modes.
const root = fileURLToPath(new URL('../../', import.meta.url));
const inputPath = resolve(process.argv[2] ?? resolve(root, 'reports/kirchhoff-runtime-discretization-input.json'));
const outputPath = resolve(process.argv[3] ?? resolve(root, 'reports/kirchhoff-runtime-discretization-audit.json'));
if (inputPath === outputPath) throw new Error('Input and output must be different files');
const inputBytes = readFileSync(inputPath), input = JSON.parse(inputBytes);
if (input.schemaVersion !== 1) throw new Error('Unsupported discretization input version');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const settings = input.analysisSettings;
const body = input.mechanicalState.bodies[settings.wireBodyIndex];
if (!body || body.activeEnd <= body.activeStart) throw new Error('An active wire centerline is required');
const finite = (v, name) => {
    if (!Number.isFinite(v)) throw new Error(`${name} must be finite`);
    return v;
};
const raw = [];
for (let node = body.activeStart + 1; node < body.activeEnd; node++) {
    const h0 = finite(body.restLength[node - 1], 'left material length');
    const h1 = finite(body.restLength[node], 'right material length');
    if (h0 <= 0 || h1 <= 0) throw new Error('Material edge lengths must be positive');
    const t = h0 / (h0 + h1);
    const error = ['x', 'y', 'z'].map(axis => finite(body[axis][node], 'fine position') -
        ((1 - t) * finite(body[axis][node - 1], 'left position') + t * finite(body[axis][node + 1], 'right position')));
    raw.push({ node, leftMaterialLengthMm: h0, rightMaterialLengthMm: h1, candidateLengthMm: h0 + h1,
        fineNodeParameter: t, positionErrorVectorMm: error, positionErrorMm: Math.hypot(...error) });
}
const sorted = raw.map(r => r.positionErrorMm).sort((a, b) => a - b);
const quantile = p => {
    const index = (sorted.length - 1) * p, lower = Math.floor(index), fraction = index - lower;
    return sorted[lower] * (1 - fraction) + sorted[Math.ceil(index)] * fraction;
};
const thresholds = settings.thresholdsMm.map(tolerance => {
    finite(tolerance, 'position tolerance');
    if (tolerance < 0) throw new Error('Tolerance must be nonnegative');
    const eligible = raw.filter(r => r.positionErrorMm <= tolerance).map(r => r.node);
    const spans = [];
    for (const node of eligible) {
        const last = spans.at(-1);
        if (last && node === last[1] + 1) last[1] = node;
        else spans.push([node, node]);
    }
    return { toleranceMm: tolerance, geometryOnlyEligibleMidpoints: eligible.length, eligibleNodeIndices: eligible,
        contiguousEligibleNodeRanges: spans,
        disjointEligiblePairsWithOddLocalMidpoint: eligible.filter(i => (i - body.activeStart) % 2 === 1).length,
        eligibleRemoteMidpoints: eligible.filter(i => i >= settings.remoteInteriorStartsAtNode).length,
        status: 'geometry-only upper bound; contact/support/stability/twist/force certificates still required' };
});
const references = ['src/physics/kirchhoffBundleModel.js', 'src/physics/kirchhoffBundleDiscretization.js'];
const output = {
    schemaVersion: 1,
    sourceHashes: { input: { path: relative(root, inputPath), sha256: sha(inputBytes) },
        generator: { path: relative(root, fileURLToPath(import.meta.url)), sha256: sha(readFileSync(fileURLToPath(import.meta.url))) },
        modelApiReferences: references.map(path => ({ path, sha256: sha(readFileSync(resolve(root, path))) })),
        originalCapture: input.sourceCapture },
    method: {
        operation: 'Replace two adjacent material edges by one linear position field; evaluate it at the original interior material node.',
        formula: 't=h_left/(h_left+h_right); e_i=norm(x_i-((1-t)*x_{i-1}+t*x_{i+1}))',
        units: 'millimetres', candidateRepresentation: 'linear position interpolation used by buildAdaptiveBundleMesh',
        sampleCount: raw.length, thresholds: settings.thresholdsMm,
        distributionQuantile: 'linear interpolation at p*(n-1) in sorted errors',
        boundStatus: 'Each observed error is a rejection witness, not an upper certificate between samples or on omitted dynamics.',
        missingEvidence: ['per-segment wall/support/control masks', 'continuous swept-gap bounds', 'SO(3) frame/twist interpolation bounds',
            'energy/force/moment error budgets and bounds', 'stability/coercivity of eliminated modes'],
        noRuntimeModification: true, noAdditionalReplay: true
    },
    capture: input.capture,
    measuredRuntime: {
        activeNodeCounts: input.mechanicalState.bodies.map(b => b.activeEnd - b.activeStart + 1),
        activeWallContactCounts: input.mechanicalState.bodyStats.map(b => ({ id: b.id, activeWallContacts: b.activeWallContacts })),
        emittedLumenRecords: input.mechanicalState.records.length,
        loadedLumenRecords: input.mechanicalState.records.filter(r => r.normalLambda > 0).length,
        finalLinear: input.mechanicalState.finalLinear,
        containment: input.mechanicalState.constraint,
        normalBalances: input.mechanicalState.records.map(r => ({ kind: r.kind, gapMm: r.gap, alpha: r.normalAlpha,
            normalLambda: r.normalLambda, alphaLambdaMm: r.alphaLambda, equationResidualMm: r.equilibriumResidual,
            containedSpanFraction: r.containedSpanFraction }))
    },
    geometricErrorStatisticsMm: { minimum: sorted[0], median: quantile(0.5), p95: quantile(0.95),
        maximum: sorted.at(-1), mean: sorted.reduce((sum, value) => sum + value, 0) / sorted.length },
    thresholds, rawMidpointResiduals: raw,
    decision: { retainFineRuntimeMesh: true, activeOrUncertainOverlapRepresentation: 'full',
        preferredNextOptimization: 'exact skyline or bilateral material Schur with full reconstruction and original residual checks',
        certifiedCoarseningAdmitted: false,
        scope: 'This snapshot and linear-field coarsening only; no claim that higher-order curved elements are impossible.' }
};
writeFileSync(outputPath, JSON.stringify(output, null, 2) + '\n');
console.log(JSON.stringify({ output: relative(root, outputPath), sampleCount: raw.length,
    statisticsMm: output.geometricErrorStatisticsMm, thresholds, decision: output.decision }));
