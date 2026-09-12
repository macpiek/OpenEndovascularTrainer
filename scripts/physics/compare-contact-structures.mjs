import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { gzipSync, gunzipSync } from 'node:zlib';
import { Worker, isMainThread, parentPort, workerData } from 'node:worker_threads';
import { solveActiveCondensedCoupledQP } from '../../src/physics/kirchhoffActiveCondensedSolver.js';
import { measureCoupledLoadKKT } from '../../src/physics/kirchhoffCoupledLoadSolver.js';
import { measureCoupledFrictionKKT } from '../../src/physics/kirchhoffCoupledFrictionSolver.js';
import { auditFrozenCoupledSystem } from './audit-frozen-coupled-system.mjs';

const encode = (_key, value) => typeof value === 'number' && !Number.isFinite(value) ? String(value)
    : ArrayBuffer.isView(value) ? Array.from(value) : value;
const decode = (_key, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value === 'NaN' ? NaN : value;
const directory = path.resolve(process.argv[3] ?? 'reports/contact-structure-comparison-2026-09-11');

// Worker isolation supplies a hard wall limit without interrupting app physics.
if (!isMainThread) {
    const input = JSON.parse(gunzipSync(fs.readFileSync(workerData.file)), decode);
    const matrix = Float64Array.from(input.matrix), rhs = Float64Array.from(input.rhs);
    const lower = Float64Array.from(input.lower), upper = Float64Array.from(input.upper);
    const options = { ...input.options, initialFree: Uint8Array.from(input.initialFree),
        simultaneousCoulomb: true, workspace: {}, frictionWorkspace: {}, loadWorkspace: {},
        ...(workerData.variant === 'full-band' ? { coulombStructure: 'full-band' } : {}) };
    const times = [], checks = [];
    let result, previousIncrement, maximumRepeatIncrementDifference = 0;
    for (let i = 0; i < workerData.repetitions; i++) {
        const started = performance.now();
        result = solveActiveCondensedCoupledQP(matrix, rhs, lower, upper, input.count, input.band, input.groups, options);
        times.push(performance.now() - started);
        if (previousIncrement) result.increment.forEach((value, row) => {
            maximumRepeatIncrementDifference = Math.max(maximumRepeatIncrementDifference, Math.abs(value - previousIncrement[row]));
        });
        previousIncrement = result.increment.slice();
        checks.push({ converged: result.diagnostics.converged,
            maximumResidual: result.diagnostics.maximumResidual,
            ...auditFrozenCoupledSystem({ ...input, increment: result.increment,
                lower: result.lower, upper: result.upper, rows: input.alpha.map(alpha => ({ alpha })) }) });
    }
    // Independent reconstruction through the original physical Jacobian and
    // inverse masses, not through the reduced matrix or its residual array.
    const residual = rhs.map((value, i) => value - input.alpha[i] * result.increment[i]);
    const corrections = input.columns.map((columns, side) => columns.map((entries, dof) => {
        let force = 0;
        for (let k = 0; k < entries.length; k += 2) force += entries[k + 1] * result.increment[entries[k]];
        const correction = input.weights[side][dof] * force;
        for (let k = 0; k < entries.length; k += 2) residual[entries[k]] -= entries[k + 1] * correction;
        return correction;
    }));
    const kkt = input.groups.some(g => g.normalRow != null)
        ? measureCoupledLoadKKT(residual, result.increment, result.lower, result.upper, result.allGroups)
        : measureCoupledFrictionKKT(residual, result.increment, result.lower, result.upper, result.groups ?? []);
    const warmupRuns = workerData.warmupRuns ?? 1;
    const warm = times.slice(warmupRuns).sort((a, b) => a - b);
    const warmMedianMs = warm.length ? (warm[Math.floor((warm.length - 1) / 2)] + warm[Math.floor(warm.length / 2)]) / 2 : null;
    parentPort.postMessage({ variant: workerData.variant, times, coldMs: times[0], warmMedianMs,
        warmupRuns, maximumRepeatIncrementDifference, checks, diagnostics: result.diagnostics,
        physicalResidual: kkt.maximumResidual, finite: corrections.every(c => c.every(Number.isFinite)),
        certified: checks.every(check => check.converged && check.passed) &&
            kkt.maximumResidual <= (options.tolerance ?? 1e-8),
        corrections, reactions: Array.from(result.increment) });
} else if (process.argv[2] === 'capture') {
    fs.mkdirSync(directory, { recursive: true });
    const { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } = await import('../../tests/helpers/coupledRuntimeFixture.js');
    const { EndovascularPhysicsWorld } = await import('../../src/physics/endovascularPhysicsWorld.js');
    const { configureKirchhoffToolRuntime } = await import('../../src/physics/kirchhoffToolRuntime.js');
    const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import('../../src/physics/kirchhoffCoupledSystem.js');
    const anatomy = await loadCoupledRuntimeAnatomy();
    const targets = [100, 200, 400], captured = new Set();
    let fixture, steps = 0, failures = 0;
    class World extends EndovascularPhysicsWorld {
        stepFixed() { for (const body of this.bodies) configureKirchhoffToolRuntime(body); return super.stepFixed(); }
    }
    const began = performance.now();
    fixture = createCoupledRuntimeFixture({ ...anatomy, World, coupledSystem: {
        independentComponents: true, physicalTrialState: true, earlyTrialRejection: true,
        solve: (constraint, dt, options) => solveKirchhoffCoupledSystem(constraint, dt, { ...options,
            activeCondensation: true, simultaneousCoulomb: true,
            assemblySolver(system, linearOptions) {
                const target = targets.find(t => !captured.has(t) && fixture.catheter.progress >= t - 1e-7);
                if (target != null && system.bodies.length === 2 && system.groups.length) {
                    const numericOptions = Object.fromEntries(Object.entries(linearOptions)
                        .filter(([_key, value]) => ['number', 'boolean', 'string'].includes(typeof value)));
                    const input = { depth: target, wireMm: fixture.transport.progress, catheterMm: fixture.catheter.progress,
                        steps, precedingFailedSteps: failures, config: Object.fromEntries(Object.entries(fixture.config).filter(([_k, v]) => ['number', 'boolean', 'string'].includes(typeof v))), options: numericOptions,
                        count: system.count, band: system.band, matrix: system.matrix, rhs: system.rhs,
                        lower: system.lower, upper: system.upper, groups: system.groups, initialFree: system.initialFree,
                        columns: system.columns, weights: system.bodies.map((b, side) => system.material[side]?.weight ?? new Float64Array(b.count * 6)),
                        alpha: Array.from(system.order, original => system.rows[original].alpha) };
                    fs.writeFileSync(path.join(directory, `depth-${target}.json.gz`), gzipSync(JSON.stringify(input, encode), { level: 1 }));
                    captured.add(target);
                    console.log(JSON.stringify({ captured: target, count: system.count, band: system.band,
                        groups: system.groups.length, elapsedMs: performance.now() - began }));
                }
                return solveActiveCondensedCoupledQP(system.matrix, system.rhs, system.lower, system.upper,
                    system.count, system.band, system.groups, { ...linearOptions, simultaneousCoulomb: true });
            } }), apply: applyKirchhoffCoupledCorrection
    } });
    try {
        while (captured.size < targets.length) {
            const dt = fixture.config.fixedDt;
            const wire = Math.max(0, Math.min(1, (600 - fixture.transport.progress) / (44 * dt)));
            const next = targets.find(t => fixture.catheter.progress < t - 1e-7) ?? 400;
            const catheter = wire ? 0 : Math.max(0, Math.min(1, (next - fixture.catheter.progress) / (52 * dt)));
            fixture.step({ guidewireAdvance: wire, catheterAdvance: catheter });
            failures += Number(Boolean(fixture.world.lastJointNonlinearFailure)); steps++;
            if (steps % 120 === 0) console.log(JSON.stringify({ steps, wireMm: fixture.transport.progress,
                catheterMm: fixture.catheter.progress, failures, elapsedMs: performance.now() - began }));
        }
    } finally {
        fs.writeFileSync(path.join(directory, 'capture-summary.json'), JSON.stringify({ captured: [...captured],
            steps, failures, snapshot: fixture.snapshot(), elapsedMs: performance.now() - began }, null, 2));
        fixture.dispose(); anatomy.dispose();
    }
} else if (process.argv[2] === 'compare') {
    const timeoutMs = Number(process.env.OET_STRUCTURE_TIMEOUT_MS ?? 30000);
    const repetitions = Number(process.env.OET_STRUCTURE_REPETITIONS ?? 21);
    const warmupRuns = Number(process.env.OET_STRUCTURE_WARMUP_RUNS ?? 6);
    const variantOrder = process.env.OET_STRUCTURE_REVERSE_ORDER === '1' ? ['full-band', 'condensed'] : ['condensed', 'full-band'];
    if (!(timeoutMs > 0 && Number.isFinite(timeoutMs) && Number.isInteger(repetitions) && repetitions > 0 &&
        Number.isInteger(warmupRuns) && warmupRuns >= 0 && warmupRuns < repetitions))
        throw new RangeError('Use a positive timeout and repetition count, with fewer nonnegative warmup runs');
    const run = (file, variant) => new Promise(resolve => {
        const worker = new Worker(new URL(import.meta.url), { workerData: { file, variant, repetitions, warmupRuns } });
        const timer = setTimeout(() => { worker.terminate(); resolve({ variant, timedOut: true, timeoutMs }); }, timeoutMs);
        worker.once('message', result => { clearTimeout(timer); resolve(result); });
        worker.once('error', error => { clearTimeout(timer); resolve({ variant, error: error.message }); });
        worker.once('exit', code => { clearTimeout(timer); resolve({ variant, error: `Worker exited without a result (${code})` }); });
    });
    const results = [];
    for (const depth of [100, 200, 400]) if (!fs.existsSync(path.join(directory, `depth-${depth}.json.gz`)))
        throw new Error(`Missing capture at ${depth} mm; finish capture before comparing`);
    for (const depth of [100, 200, 400]) {
        const file = path.join(directory, `depth-${depth}.json.gz`);
        const bytes = gunzipSync(fs.readFileSync(file)), input = JSON.parse(bytes, decode);
        const variants = [];
        for (const variant of variantOrder) variants.push(await run(file, variant));
        let maximumTranslationDifference = null, maximumRotationDifference = null, maximumReactionDifference = null;
        if (variants.every(v => v.corrections)) {
            maximumTranslationDifference = maximumRotationDifference = maximumReactionDifference = 0;
            variants[0].corrections.forEach((values, side) => values.forEach((value, dof) => {
                const delta = Math.abs(value - variants[1].corrections[side][dof]);
                if (dof % 6 < 3) maximumTranslationDifference = Math.max(maximumTranslationDifference, delta);
                else maximumRotationDifference = Math.max(maximumRotationDifference, delta);
            }));
            variants[0].reactions.forEach((v, i) => { maximumReactionDifference = Math.max(maximumReactionDifference, Math.abs(v - variants[1].reactions[i])); });
        }
        const row = { depth, count: input.count, band: input.band, groups: input.groups.length,
            wireMm: input.wireMm, catheterMm: input.catheterMm, precedingFailedSteps: input.precedingFailedSteps,
            sha256: createHash('sha256').update(bytes).digest('hex'), repetitions, warmupRuns, timeoutMs,
            variants, bothCertified: variants.every(v => v.certified),
            maximumTranslationDifference, maximumRotationDifference, maximumReactionDifference };
        results.push(row);
        console.log(JSON.stringify({ depth, variants: variants.map(({ corrections, reactions, ...rest }) => rest),
            maximumTranslationDifference, maximumRotationDifference, maximumReactionDifference }, encode));
        fs.writeFileSync(path.join(directory, 'comparison.json'), JSON.stringify(results, encode, 2));
    }
} else throw new Error('Usage: node scripts/physics/compare-contact-structures.mjs capture|compare [directory]');
