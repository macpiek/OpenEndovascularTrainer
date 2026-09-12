import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import { createHash } from 'node:crypto';
import { performance } from 'node:perf_hooks';
import { measureKirchhoffBiasReleases } from '../../src/physics/kirchhoffTwoChannelRelease.js';
import { measureKirchhoffBiasReleasesReference } from '../../tests/fixtures/twoChannelReleaseMeasureReference.js';
import { sparseReleaseMeasureFixture, measureFloat64Allocations, releaseMeasureBytes } from '../../tests/fixtures/sparseReleaseMeasureFixture.js';

const root = new URL('../../', import.meta.url), output = process.argv[2] ?? '/tmp/oet-release-measure-perf.json';
const paths = ['src/physics/kirchhoffTwoChannelRelease.js', 'tests/fixtures/twoChannelReleaseMeasureReference.js',
    'tests/fixtures/sparseReleaseMeasureFixture.js', 'tests/kirchhoffTwoChannelReleaseMeasure.test.js',
    'scripts/physics/measure-two-channel-release-perf.mjs'];
const hash = data => createHash('sha256').update(data).digest('hex');
const hashes = () => Object.fromEntries(paths.map(p => [p, hash(fs.readFileSync(new URL(p, root)))]));
const sourceBefore = hashes(), joint = sparseReleaseMeasureFixture(), stateBefore = releaseMeasureBytes(joint);
const baseline = measureKirchhoffBiasReleasesReference(joint), optimized = measureKirchhoffBiasReleases(joint);
assert.deepEqual(releaseMeasureBytes(optimized), releaseMeasureBytes(baseline)); assert.deepEqual(releaseMeasureBytes(joint), stateBefore);
const allocation = { baseline: measureFloat64Allocations(measureKirchhoffBiasReleasesReference, joint),
    optimized: measureFloat64Allocations(measureKirchhoffBiasReleases, joint) };
for (const a of Object.values(allocation)) delete a.lengths;
let sink = 0;
function timed(fn, iterations) {
    const start = performance.now();
    for (let i = 0; i < iterations; i++) {
        const r = fn(joint); sink += r.releasePositionMm + r.releaseAngleRad + r.pendingReleases.length;
    }
    return (performance.now() - start) / iterations;
}
for (let i = 0; i < 24; i++) { timed(measureKirchhoffBiasReleasesReference, 1); timed(measureKirchhoffBiasReleases, 1); }
const calibration = timed(measureKirchhoffBiasReleasesReference, 8), iterations = Math.max(8, Math.min(128, Math.ceil(300 / calibration)));
const rounds = [];
for (let i = 0; i < 9; i++) {
    const record = { round: i + 1, order: i % 2 ? ['optimized', 'baseline'] : ['baseline', 'optimized'] };
    for (const name of record.order) {
        globalThis.gc?.(); // Explicit collection is outside the timed interval.
        record[name + 'MsPerCall'] = timed(name === 'baseline' ? measureKirchhoffBiasReleasesReference : measureKirchhoffBiasReleases, iterations);
    }
    rounds.push(record);
}
function stats(key) {
    const values = rounds.map(r => r[key]).sort((a, b) => a - b);
    return { medianMsPerCall: values[Math.floor(values.length / 2)], minimumMsPerCall: values[0], maximumMsPerCall: values.at(-1) };
}
const referenceStats = stats('baselineMsPerCall'), optimizedStats = stats('optimizedMsPerCall');
assert.deepEqual(releaseMeasureBytes(measureKirchhoffBiasReleases(joint)), releaseMeasureBytes(baseline)); assert.deepEqual(releaseMeasureBytes(joint), stateBefore);
const sourceAfter = hashes(); assert.deepEqual(sourceAfter, sourceBefore);
const report = { scope: 'Local release-measurement microbenchmark only; not whole solver, browser or FPS performance.',
    timestamp: new Date().toISOString(), node: process.version, platform: process.platform, arch: process.arch,
    cpu: os.cpus()[0]?.model, logicalCpus: os.cpus().length, explicitGc: typeof globalThis.gc === 'function',
    fixture: { nodesPerBody: [200, 200], releases: 400, activeStart: [7, 11], rotatedFrames: true,
        anisotropicInertia: true, duplicateDofs: true, support: 'two active nodes per body, plus inactive/end-node samples' },
    methodology: { warmupPairs: 24, rounds: rounds.length, iterationsPerVariantPerRound: iterations,
        alternatingOrder: true, timedIncludesAllocationAndIncidentalGc: true, allocationInstrumentationOutsideTiming: true },
    parity: { entireOutputBytesEqual: true, entireInputStateBytesUnchanged: true, outputSha256: hash(releaseMeasureBytes(baseline)), stateSha256: hash(stateBefore) },
    allocation, baseline: referenceStats, optimized: optimizedStats,
    medianSpeedup: referenceStats.medianMsPerCall / optimizedStats.medianMsPerCall,
    medianTimeReductionPercent: 100 * (1 - optimizedStats.medianMsPerCall / referenceStats.medianMsPerCall),
    rounds, sourceBefore, sourceAfter, sourceStable: true, sink };
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, baseline: report.baseline, optimized: report.optimized,
    speedup: report.medianSpeedup, allocation, parity: report.parity, sourceStable: true }, null, 2));
