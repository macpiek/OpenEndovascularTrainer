import fs from 'node:fs';
import { pathToFileURL } from 'node:url';
const snapshot = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'), (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
const moduleUrl = process.argv[3] ? pathToFileURL(process.argv[3]).href : new URL('../../src/physics/kirchhoffCoupledFrictionSolver.js', import.meta.url).href;
const { solveCoupledFrictionQP } = await import(moduleUrl);
const matrix = new Float64Array(snapshot.matrix), rhs = new Float64Array(snapshot.rhs);
const lower = new Float64Array(snapshot.lower), upper = new Float64Array(snapshot.upper), initialFree = new Uint8Array(snapshot.initialFree);
const repetitions = Number(process.argv[4] ?? 1), times = [], results = [];
const reuseWorkspace = process.argv[5] === 'reuse', sharedWorkspace = {};
let first;
for (let pass = 0; pass < repetitions; pass++) {
    // Every trial uses the original force state and identical initial hints.
    // Optional workspace reuse matches runtime allocation without force or
    // working-set warm starts changing the frozen mathematical problem.
    const start = performance.now();
    const result = solveCoupledFrictionQP(matrix, rhs, lower, upper, snapshot.count, snapshot.band, snapshot.groups,
        { ...snapshot.options, initialFree, frictionWorkspace: reuseWorkspace ? sharedWorkspace : {} });
    times.push(performance.now() - start); results.push(result.diagnostics);
    if (!first) first = Array.from(result.increment);
}
console.log(JSON.stringify({ source: process.argv[2], solver: moduleUrl, node: process.version, workspaceMode: reuseWorkspace ? 'reuse' : 'fresh', times, results, firstIncrement: first,
    coneProfile: globalThis.__coneProfile, linearProfile: globalThis.__linearProfile }, null, 2));
