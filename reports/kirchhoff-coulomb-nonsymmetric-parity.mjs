import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL, fileURLToPath } from 'node:url';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const baselinePath = process.argv[2] ?? '/tmp/oet-nonsymmetric-newton-baseline/kirchhoffCoulombNewtonSolver.js';
const sourcePath = path.join(root, 'src/physics/kirchhoffCoulombNewtonSolver.js');
const hash = p => createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const baselineHash = hash(baselinePath);
assert.equal(baselineHash, '6b5ffe0d4c4ee2394878abeef4f49dbf8131e4882d6c3f092c9d614e5b1c0cd7');
const originalSource = fs.readFileSync(baselinePath, 'utf8').replace(/from '\.\/([^']+)'/g,
    (_, name) => `from '${pathToFileURL(path.join(root, 'src/physics', name)).href}'`);
const baseline = await import(`data:text/javascript;base64,${Buffer.from(originalSource).toString('base64')}`);
const current = await import(pathToFileURL(sourcePath));
function capture(entry, p, backend, matrixFormat) {
    const directions = [], iterations = [];
    const options = { ...p.options, tolerance: p.options.tolerance, retryWithoutHints: false, coulombLinearSolver: backend,
        initialIncrement: p.initialIncrement, ...(matrixFormat ? { matrixFormat } : {}),
        debugCoulombDirection({ x, F, J, direction }) { directions.push(structuredClone({ x, F, J, direction })); },
        debugCoulombIteration(value) { iterations.push(structuredClone(value)); } };
    const args = ['matrix', 'rhs', 'lower', 'upper'].map(k => Float64Array.from(p[k]));
    return { result: entry(...args, p.count, p.band, structuredClone(p.groups), options), directions, iterations };
}
let comparedBytes = 0;
function compareBytes(a, b) {
    if (ArrayBuffer.isView(a)) {
        const x = Buffer.from(a.buffer, a.byteOffset, a.byteLength), y = Buffer.from(b.buffer, b.byteOffset, b.byteLength);
        assert.ok(x.equals(y)); comparedBytes += x.length;
    } else if (a && typeof a === 'object') for (const key of Object.keys(a)) compareBytes(a[key], b[key]);
}
const cases = [];
for (const name of ['kirchhoff-condensed-load-cycle', 'kirchhoff-coulomb-switching-plane',
    'kirchhoff-coulomb-hinted-seed', 'kirchhoff-coulomb-144-bound-recovery']) {
    const fixture = path.join(root, `tests/fixtures/${name}.json.gz`);
    const p = JSON.parse(gunzipSync(fs.readFileSync(fixture)), (_, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    for (const entry of ['solveCoulombNewton', 'solveSeededCoulombNewton']) for (const backend of [undefined, 'band-lu']) {
        const before = capture(baseline[entry], p, backend);
        for (const format of [undefined, 'symmetric-band']) {
            const after = capture(current[entry], p, backend, format);
            assert.deepEqual(after, before); compareBytes(after, before);
            cases.push({ fixture: name, fixtureHash: hash(fixture), count: p.count, entry,
                backend: backend ?? 'dense-lu', matrixFormat: format ?? 'default', converged: after.result.diagnostics.converged,
                directions: after.directions.length, identical: true });
        }
    }
}
const output = { baselinePath, baselineHash, currentHash: hash(sourcePath), pairs: cases.length, comparedBytes, cases };
fs.writeFileSync(path.join(root, 'reports/kirchhoff-coulomb-nonsymmetric-parity.json'), JSON.stringify(output, null, 2) + '\n');
console.log(JSON.stringify({ baselineHash, currentHash: output.currentHash, pairs: cases.length, comparedBytes, passed: true }));
