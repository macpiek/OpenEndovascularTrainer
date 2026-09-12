// One fixed variant, one alternating-order pass. No world replay or rendering.
// Usage: node reports/kirchhoff-coulomb-prediction-reproduce.mjs sequence.jsonl[.gz] output.json
import fs from 'node:fs';
import readline from 'node:readline';
import { createGunzip } from 'node:zlib';
import { createHash } from 'node:crypto';
import { coulombPredictionContract, createAcceptedCoulombHistory, predictCoulombIncrement } from '../src/physics/kirchhoffCoulombPrediction.js';
import { solveCoulombWithPrediction } from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import { solveActiveCondensedCoupledQP } from '../src/physics/kirchhoffActiveCondensedSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

const [path, output] = process.argv.slice(2);
if (!path || !output) throw new Error('Provide the frozen sequence path and an output JSON path');
const revive = (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value;
async function* events() {
    const input = fs.createReadStream(path);
    for await (const line of readline.createInterface({ input: path.endsWith('.gz') ? input.pipe(createGunzip()) : input }))
        yield JSON.parse(line, revive);
}
// Outcome annotations identify which already-applied trial was accepted. Only
// that trial's recorded mechanics enters history after its own system is solved.
const trials = new Map(), outcomes = new Map();
for await (const event of events()) {
    if (event.type === 'trial') trials.set(`${event.solveId}:${event.trial}`, event);
    else if (event.type === 'outcome') outcomes.set(event.solveId, event);
}
function kkt(p, x, lower, upper, residual) {
    const groups = p.groups.map(group => ({ ...group, radii: group.normalRow == null ? group.radii :
        group.mu.map(mu => mu * Math.max(0, group.normalLambda + x[group.normalRow])) }));
    const result = measureCoupledLoadKKT(residual, x, lower, upper, groups);
    return { maximumResidual: result.maximumResidual, frictionResidual: result.groupResidual,
        coneViolation: result.coneViolation, bounds: x.every((value, i) => Number.isFinite(value) && value >= lower[i] && value <= upper[i]) };
}
function reconstruction(p, x, lower, upper) {
    const residual = Float64Array.from(p.rhs, (value, i) => value - p.rows[i].alpha * x[i]);
    const corrections = p.columns.map((columns, side) => {
        const correction = new Float64Array(columns.length);
        columns.forEach((column, dof) => {
            let response = 0;
            for (let j = 0; j < column.length; j += 2) response += column[j + 1] * x[column[j]];
            correction[dof] = p.weights[side][dof] * response;
            for (let j = 0; j < column.length; j += 2) residual[column[j]] -= column[j + 1] * correction[dof];
        });
        return correction;
    });
    return { kkt: kkt(p, x, lower, upper, residual), corrections };
}
function original(p, x, lower, upper) {
    const residual = Float64Array.from(p.rhs);
    for (let i = 0; i < p.count; i++) for (let j = Math.max(0, i - p.band + 1); j <= i; j++) {
        const value = p.matrix[i * p.band + i - j];
        residual[i] -= value * x[j];
        if (i !== j) residual[j] -= value * x[i];
    }
    return kkt(p, x, lower, upper, residual);
}
const passes = (audit, tolerance) => audit.bounds && audit.maximumResidual <= tolerance && audit.coneViolation <= 1e-9;
const hash = x => createHash('sha256').update(new Uint8Array(x.buffer, x.byteOffset, x.byteLength)).digest('hex');
const workspaces = [0, 1].map(() => ({ workspace: {}, loadWorkspace: {}, frictionWorkspace: {} }));
const report = { scope: 'One alternating-order A/B pass on 24 DIFFERENT successive frozen systems; original accepted trial histories, exact input hints/row bounds, old residual law only. Candidate results are NEVER inserted into mechanical history.', systems: [] };
let history;
for await (const p of events()) {
    if (p.type !== 'system') continue;
    const matrix = Float64Array.from(p.matrix), rhs = Float64Array.from(p.rhs);
    // Top-level bounds have already been replaced by this captured solve's
    // final cone bounds. Preserve actual original input bounds and hints.
    const lower = Float64Array.from(p.rows, row => row.lower), upper = Float64Array.from(p.rows, row => row.upper);
    const options = workspaces.map(workspace => ({ ...p.options, ...workspace, initialFree: Uint8Array.from(p.initialFree) }));
    const contract = coulombPredictionContract(p, 'oet-coupled-sequence-200');
    const predictionStart = performance.now();
    const prediction = predictCoulombIncrement(p, history, contract);
    const predictionMs = performance.now() - predictionStart, variants = [];
    for (const index of p.solveId % 2 ? [0, 1] : [1, 0]) {
        let certified, certificateMs = 0;
        const certify = result => {
            const start = performance.now();
            certified = reconstruction(p, result.increment, lower, upper);
            certificateMs += performance.now() - start;
            return passes(certified.kkt, p.options.tolerance);
        };
        const fallback = () => {
            const result = solveActiveCondensedCoupledQP(matrix, rhs, lower, upper, p.count, p.band, p.groups, options[index]);
            if (!certify(result)) throw new Error(`Fallback Jdx failed on solve ${p.solveId}`);
            return result;
        };
        const start = performance.now();
        const result = index ? solveCoulombWithPrediction(matrix, rhs, lower, upper, p.count, p.band, p.groups,
            { ...options[index], prediction, certifyPrediction: certify }, fallback) : fallback();
        const solveMs = performance.now() - start;
        const audit = original(p, result.increment, lower, upper);
        if (!result.diagnostics.converged || !passes(audit, p.options.tolerance)) throw new Error(`Full KKT failed on solve ${p.solveId}`);
        variants[index] = { ms: solveMs + (index ? predictionMs : 0), solveMs, predictionMs: index ? predictionMs : 0,
            certificateMs, diagnostics: result.diagnostics, audit, reconstructionKKT: certified.kkt,
            incrementHash: hash(result.increment), result, corrections: certified.corrections };
    }
    let maxForceDelta = 0, maximumPrimalCorrectionDifference = 0;
    for (let i = 0; i < p.count; i++) maxForceDelta = Math.max(maxForceDelta,
        Math.abs(variants[0].result.increment[i] - variants[1].result.increment[i]));
    variants[0].corrections.forEach((correction, side) => correction.forEach((value, i) => {
        maximumPrimalCorrectionDifference = Math.max(maximumPrimalCorrectionDifference, Math.abs(value - variants[1].corrections[side][i]));
    }));
    report.systems.push({ solveId: p.solveId, step: p.state.executedSteps, pass: p.outerPass, count: p.count,
        topLevelPostSolveBoundChanges: p.rows.filter((row, i) => row.lower !== p.lower[i] || row.upper !== p.upper[i]).length,
        mapping: prediction.diagnostics, maxForceDelta, maximumPrimalCorrectionDifference,
        variants: variants.map(({ result, corrections, ...variant }) => variant) });
    console.log(JSON.stringify({ solveId: p.solveId, factors: variants.map(variant => variant.diagnostics.factorizations),
        prediction: variants[1].diagnostics.prediction }));
    const outcome = outcomes.get(p.solveId);
    if (outcome?.accepted) history = createAcceptedCoulombHistory(p, trials.get(`${p.solveId}:${outcome.trial}`), outcome, contract);
}
if (report.systems.length !== 24) throw new Error(`Expected all 24 sequence systems, got ${report.systems.length}`);
report.total = report.systems.reduce((sum, system) => {
    for (let i = 0; i < 2; i++) { sum.ms[i] += system.variants[i].ms; sum.factors[i] += system.variants[i].diagnostics.factorizations; }
    const prediction = system.variants[1].diagnostics.prediction;
    sum.predicted += Number(prediction.accepted); sum.attempts += Number(prediction.attempted);
    sum.predictionFactors += prediction.factorizations; sum.rejectedFactors += prediction.accepted ? 0 : prediction.factorizations;
    return sum;
}, { ms: [0, 0], factors: [0, 0], predicted: 0, attempts: 0, predictionFactors: 0, rejectedFactors: 0 });
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report.total));
