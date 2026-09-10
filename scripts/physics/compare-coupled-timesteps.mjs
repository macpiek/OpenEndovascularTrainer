import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { DEEP_CATHETER_BENCHMARK_PHASES } from '../../src/benchmark/shortCatheterBenchmark.js';
import { measureKirchhoffFrictionMerit } from '../../src/physics/kirchhoffFrictionMerit.js';
import { createCoupledSolverSelection } from '../../src/physics/coupledSolverSelection.js';
import { createFrozenCoupledMetadata } from './frozen-coupled-metadata.mjs';
import { configureKirchhoffSplitBias } from '../../src/physics/kirchhoffSplitMotion.js';

// Diagnostic comparison, not a browser/FPS acceptance test. Distances and
// held/rotation durations are equal; feed durations differ by at most one
// fixed step because a fractional final command preserves the exact distance.
const argv = process.argv.slice(2), option = key => argv.includes(key) ? argv[argv.indexOf(key) + 1] : null;
const solverId = option('--solver') ?? 'joint-active-coulomb';
const jointMotionMode = option('--motion-mode') ?? 'position-history';
if (!['position-history', 'split-physical-bias'].includes(jointMotionMode))
    throw new RangeError('Use --motion-mode position-history|split-physical-bias');
const biasMaterialMode = option('--bias-material-mode') ?? 'physical-compliance';
if (!['physical-compliance', 'preserve-strain'].includes(biasMaterialMode) ||
    option('--bias-material-mode') && jointMotionMode !== 'split-physical-bias')
    throw new RangeError('Use --bias-material-mode physical-compliance|preserve-strain with split-physical-bias');
if (!['joint', 'joint-active-coulomb', 'joint-full-band'].includes(solverId))
    throw new RangeError('The timestep diagnostic requires a joint solver variant');
const rates = (option('--hz') ?? '60,120').split(',').map(Number), through = Number(option('--through-index') ?? 4);
const prepareHz = option('--prepare-hz') == null ? null : Number(option('--prepare-hz'));
if (rates.some(hz => ![60, 120, 240].includes(hz)) || !Number.isInteger(through) || through < 0 || through >= DEEP_CATHETER_BENCHMARK_PHASES.length)
    throw new RangeError('Use --hz 60,120,240 and a valid --through-index');
if (prepareHz !== null && ![60, 120, 240].includes(prepareHz)) throw new RangeError('Use --prepare-hz 60,120,240');
const output = option('--output') ?? '/tmp/oet-coupled-timestep-comparison.json';
const captureFailure = option('--capture-linear-failure');
const captureSystem = option('--capture-system'), captureSystemFrom = Number(option('--capture-system-from-mm') ?? 0);
const captureSystemMode = option('--capture-system-mode') ?? 'first';
const captureSystemLimit = Number(option('--capture-system-limit') ?? 16);
const stopAfterSystemCapture = argv.includes('--stop-after-system-capture');
if (!['first', 'max-factorizations', 'sequence'].includes(captureSystemMode))
    throw new RangeError('Use --capture-system-mode first|max-factorizations|sequence');
if (captureSystem && captureSystemMode === 'sequence' && jointMotionMode !== 'position-history')
    throw new RangeError('Split motion needs phase-aware accepted-trial history before sequence capture is supported');
if (!Number.isInteger(captureSystemLimit) || captureSystemLimit < 1 ||
    stopAfterSystemCapture && (!captureSystem || captureSystemMode !== 'sequence'))
    throw new RangeError('Use a positive capture limit and sequence mode for --stop-after-system-capture');
if (!Number.isFinite(captureSystemFrom) || captureSystemFrom < 0) throw new RangeError('Use a nonnegative --capture-system-from-mm');
let systemCaptured = false, capturedFactorizations = -1, capturedSystems = 0;
const traceTrials = option('--trace-trials'), traceFrom = Number(option('--trace-from-mm') ?? 0);
const sourcePaths = [...fs.readdirSync(new URL('../../src/physics/', import.meta.url))
    .filter(name => name.endsWith('.js')).map(name => 'src/physics/' + name),
    'tests/helpers/coupledRuntimeFixture.js', 'src/pigtailCatheter.js',
    'src/benchmark/shortCatheterBenchmark.js', 'scripts/physics/compare-coupled-timesteps.mjs',
    'scripts/physics/frozen-coupled-metadata.mjs'].sort();
const hashes = () => sourcePaths.map(path => [path, createHash('sha256').update(fs.readFileSync(new URL('../../' + path, import.meta.url))).digest('hex')]);
const encode = value => JSON.stringify(value, (_key, item) => ArrayBuffer.isView(item) ? Array.from(item) :
    typeof item === 'number' && !Number.isFinite(item) ? String(item) : typeof item === 'bigint' ? String(item) : item);
const report = { scope: 'Diagnostic Node replay, no rendering, FPS or scheduler/backlog measurement',
    measuredAt: new Date().toISOString(), node: process.version, sourceBefore: hashes(), prepareHz, solverId, jointMotionMode,
    biasMaterialMode: jointMotionMode === 'split-physical-bias' ? biasMaterialMode : null,
    notes: ['Same commanded distances, speeds and held times, with exact fractional final feed commands.',
        'Body linear/angular damping and component velocity damping are converted from per-120-Hz-step factors to preserve decay per second.',
        'Projection-velocity retention remains part of the original numerical model; this comparison does not establish timestep-independent transient dynamics.',
        'Preparation uses the existing independent guidewire path; its failures are recorded. A failed coupled step stops that frequency run.',
        'Optional --prepare-hz uses that frequency for phases 0-2, then changes every fixture clock together; prepared positions, frames and velocities are not reset.'], runs: [] };
const anatomy = await loadCoupledRuntimeAnatomy(new URL('../../', import.meta.url));
function summarize(values) {
    const sorted = values.slice().sort((a, b) => a - b);
    return { mean: values.reduce((s, v) => s + v, 0) / (values.length || 1), p95: sorted[Math.max(0, Math.ceil(sorted.length * .95) - 1)] ?? 0 };
}
try {
    for (const hz of rates) {
        let dt = 1 / (prepareHz ?? hz);
        const fixture = createCoupledRuntimeFixture({ ...anatomy, fixedDt: dt, jointMotionMode }), world = fixture.world;
        if (jointMotionMode === 'split-physical-bias') configureKirchhoffSplitBias(fixture.containment, { materialMode: biasMaterialMode });
        const run = { hz, dt: 1 / hz, preparationHz: prepareHz ?? hz, jointMotionMode,
            biasMaterialMode: jointMotionMode === 'split-physical-bias' ? biasMaterialMode : null,
            phases: [], complete: false, failure: null }; report.runs.push(run);
        const damping = world.bodies.map(body => ({ linear: body.linearDamping, angular: body.angularDamping, sleep: body.sleepFrames }));
        const componentDamping = world.containments.map(c => ({ radialVelocityDamping: c.radialVelocityDamping,
            coupledBendingRateDamping: c.coupledBendingRateDamping }));
        function setRate(rate) {
            dt = 1 / rate; fixture.setFixedDtForComparison(dt);
            world.bodies.forEach((body, i) => {
                body.linearDamping = damping[i].linear ** (dt * 120); body.angularDamping = damping[i].angular ** (dt * 120);
                body.sleepFrames = Math.max(1, Math.ceil(damping[i].sleep * rate / 120));
            });
            world.containments.forEach((c, i) => {
                for (const key of ['radialVelocityDamping', 'coupledBendingRateDamping'])
                    c[key] = 1 - (1 - componentDamping[i][key]) ** (dt * 120);
            });
        }
        setRate(prepareHz ?? hz);
        world.captureCoupledClosureTrace = true;
        const metadata = createFrozenCoupledMetadata(), stepCaptures = [];
        let capturedSolve = null;
        const writeSequenceEvent = event => fs.appendFileSync(captureSystem, encode(event) + '\n');
        const trials = [];
        function trace(c, phase, extra = {}, batch = c._jointFrictionBatch) {
            if (!traceTrials || fixture.catheter.progress < traceFrom) return;
            const friction = batch?.entries.map(entry => ({ id: entry.contact.id,
                normalLambda: entry.contact.normalLambda, normal: Array.from(entry.contact.normal),
                mu: entry.surface.group.mu, lambda: entry.surface.rows.map(row => row.lambda),
                strain: entry.surface.rows.map(row => row.strain),
                merit: measureKirchhoffFrictionMerit({ entries: [entry] }).maximumMm })) ?? [];
            trials.push({ phase, progress: fixture.snapshot(), ...extra, friction,
                records: c.kirchhoffContacts.map(record => Object.fromEntries(Object.entries(record).filter(([key, value]) =>
                    ['string', 'number', 'boolean'].includes(typeof value) || ArrayBuffer.isView(value))
                    .map(([key, value]) => [key, ArrayBuffer.isView(value) ? Array.from(value) : value]))),
                bodies: world.bodies.map(body => Object.fromEntries(Object.entries(body).filter(([key, value]) =>
                    ['string', 'number', 'boolean'].includes(typeof value) || ArrayBuffer.isView(value))
                    .map(([key, value]) => [key, ArrayBuffer.isView(value) ? Array.from(value) : value]))) });
        }
        world.debugJointTrial = (traceTrials || captureSystemMode === 'sequence') ? (c, state, pass, trial, scale) => {
            if (traceTrials) trace(c, 'trial',
            { pass, trial, scale, motionPhase: state.motionPhase, settled: state.settled, merit: state.merit,
                normal: c.kirchhoffSolverResidual, boundary: c._jointBoundaryResidual,
                material: { ...state.materialResidual }, fold: { ...state.foldResidual },
                externalFriction: state.externalFrictionResidual.maximumDisplacementResidualMm }, state.frictionResidual._batch);
            if (capturedSolve) writeSequenceEvent({ type: 'trial', solveId: capturedSolve.solveId,
                provisional: true, physicalStep: fixture.snapshot().executedSteps, outerPass: pass + 1,
                trial, scale, settled: state.settled, merit: state.merit, mechanics: metadata.trialState(c) });
        } : undefined;
        const selection = createCoupledSolverSelection(solverId, {
            solve(c, timestep, options) {
                let failed;
                capturedSolve = null;
                trace(c, 'before-solve');
                const freeze = captureSystem && (!systemCaptured || captureSystemMode === 'max-factorizations' ||
                    captureSystemMode === 'sequence' && capturedSystems < captureSystemLimit) &&
                    fixture.catheter.progress >= captureSystemFrom;
                const result = solveKirchhoffCoupledSystem(c, timestep,
                    { ...options, includeSystem: Boolean(freeze),
                        debugCoulombResult: captureFailure ? problem => {
                            if (!problem.result.diagnostics.converged) failed = problem;
                        } : undefined });
                const factors = result.diagnostics.factorizations ?? 0;
                if (freeze && (!systemCaptured || factors > capturedFactorizations || captureSystemMode === 'sequence')) {
                    const s = result.system, increment = new Float64Array(s.count);
                    [result.inner, result.outer].forEach((response, side) => {
                        for (let row = 0; row < (s.material[side]?.rowCount ?? 0); row++)
                            increment[s.inverseOrder[s.materialOffsets[side] + row]] = response.lambda[row];
                    });
                    for (let row = 0; row < result.contactIncrement.length; row++)
                        increment[s.inverseOrder[s.contactOffset + row]] = result.contactIncrement[row];
                    for (let row = 0; row < result.additionalIncrement.length; row++)
                        increment[s.inverseOrder[s.additionalOffset + row]] = result.additionalIncrement[row];
                    const solveId = capturedSystems + 1;
                    const encoded = encode({
                        type: 'system', solveId, provisional: true, proposedScale: result.scale,
                        scope: 'Frozen pre-application linearization; individual J/W and full original dual matrix. No geometry/timestep is skipped.',
                        source: hashes(), captureMode: captureSystemMode, outerPass: world.lastCoupledClosurePasses,
                        state: fixture.snapshot(), dt: timestep, count: s.count, band: s.band, basis: s.basis,
                        matrix: s.matrix, rhs: s.rhs, lower: s.lower, upper: s.upper, increment,
                        initialFree: s.initialFree.subarray(0, s.count),
                        groups: s.originalGroups, columns: s.columns,
                        frictionMetadata: metadata.friction(c),
                        weights: s.material.map(material => material?.weight ?? []),
                        rows: s.order.map(original => { const row = s.rows[original];
                            return { kind: row.kind, sourceKind: row.kind === 'additional' ? options.additionalRows[row.local]?.kind : row.kind,
                                side: row.side, local: row.local, coordinate: row.coordinate,
                                sourceSide: row.kind === 'additional' ? options.additionalRows[row.local].side : row.side,
                                alpha: row.alpha, lambda: row.lambda, lower: row.lower, upper: row.upper,
                                activeHint: row.activeHint,
                                contactId: row.kind === 'normal' ? c.kirchhoffContacts[row.local].manifoldContact.id : undefined,
                                recordId: row.kind === 'normal' ? c.kirchhoffContacts[row.local].id : undefined,
                                identity: row.kind === 'normal' ? metadata.contact(c.kirchhoffContacts[row.local].manifoldContact,
                                    c.kirchhoffContacts[row.local]) : row.kind === 'additional' ?
                                    metadata.sourceRow(options.additionalRows[row.local]) : undefined,
                                node: row.kind === 'additional' ? options.additionalRows[row.local].node : undefined,
                                component: row.kind === 'additional' ? options.additionalRows[row.local].component : undefined
                            }; }),
                        bodies: s.bodies.map((body, side) => ({ id: body.id, objectId: metadata.objectId(body),
                            activeStart: body.activeStart, activeEnd: body.activeEnd,
                            materialRowStart: s.material[side]?.start,
                            x: body.x, y: body.y, z: body.z, materialCoordinate: body.materialCoordinate,
                            orientationX: body.orientationX, orientationY: body.orientationY,
                            orientationZ: body.orientationZ, orientationW: body.orientationW })),
                        options: { tolerance: options.tolerance, numericalShift: options.numericalShift,
                            activeCondensation: options.activeCondensation, simultaneousCoulomb: options.simultaneousCoulomb,
                            coulombStructure: options.coulombStructure },
                        diagnostics: result.diagnostics
                    });
                    if (captureSystemMode === 'sequence' && capturedSystems) fs.appendFileSync(captureSystem, encoded + '\n');
                    else fs.writeFileSync(captureSystem, encoded + (captureSystemMode === 'sequence' ? '\n' : ''));
                    systemCaptured = true;
                    capturedFactorizations = factors;
                    capturedSystems++;
                    if (captureSystemMode === 'sequence') {
                        capturedSolve = { solveId, outerPass: world.lastCoupledClosurePasses };
                        stepCaptures.push(capturedSolve);
                    }
                }
                if (!result.diagnostics.converged && failed) {
                    const data = { ...failed, options: { tolerance: options.tolerance,
                        numericalShift: options.numericalShift, initialIncrement: failed.initialIncrement,
                        initialFree: failed.initialFree },
                        state: fixture.snapshot() };
                    fs.writeFileSync(captureFailure, JSON.stringify(data, (_key, value) =>
                        ArrayBuffer.isView(value) ? Array.from(value) :
                            typeof value === 'number' && !Number.isFinite(value) ? String(value) : value));
                }
                return result;
            },
            apply: applyKirchhoffCoupledCorrection
        });
        world.coupledSystem = selection.coupledSystem;
        try {
            for (let index = 0; index <= through; index++) {
                if (index === 3) {
                    run.preparedState = fixture.snapshot();
                    setRate(hz);
                }
                const phase = DEEP_CATHETER_BENCHMARK_PHASES[index];
                const phaseHz = 1 / dt;
                const feed = phase.wireMm ?? phase.catheterMm ?? 0, rate = phase.wireMm ? 44 : feed < 0 ? 32 : 52;
                const steps = feed ? Math.ceil(Math.abs(feed) / (rate * dt)) : Math.round(phase.steps / 120 * phaseHz);
                const times = [], fullTimes = [], row = { name: phase.name, hz: phaseHz, plannedSteps: steps, steps: 0, unconverged: 0,
                    certifiedSteps: 0, advancedPhysicsSteps: 0, physicalPassSum: 0, biasPassSum: 0,
                    maximumLengthError: 0, maximumWallPenetration: 0, maximumLinearResidual: 0, factorizationSum: 0, outerPassSum: 0 };
                run.phases.push(row);
                for (let step = 0; step < steps; step++) {
                    const command = { catheterType: 'berenstein', catheterRotation: phase.rotation ?? 0, guidewireRotation: phase.wireRotation ?? 0 };
                    if (feed) command[phase.wireMm ? 'guidewireAdvance' : 'catheterAdvance'] = Math.sign(feed) *
                        Math.max(0, Math.min(1, (Math.abs(feed) - step * rate * dt) / (rate * dt)));
                    const beforeStep = world.stepCount;
                    const start = performance.now(); fixture.step(command); fullTimes.push(performance.now() - start);
                    row.advancedPhysicsSteps += world.stepCount - beforeStep;
                    row.certifiedSteps += Number(world.lastCoupledClosureConverged);
                    const motion = world.getStats().jointMotion;
                    row.physicalPassSum += motion?.physicalPasses ?? 0;
                    row.biasPassSum += motion?.biasPasses ?? 0;
                    for (const captured of stepCaptures) {
                        const accepted = world.coupledClosureTrace.find(entry => entry.pass === captured.outerPass);
                        writeSequenceEvent({ type: 'outcome', ...captured, accepted: Boolean(accepted),
                            trial: accepted ? accepted.trials - 1 : null, scale: accepted?.scale ?? null,
                            settled: accepted?.settled ?? false, coneRepair: accepted?.coneRepair ?? null,
                            stepConverged: world.lastCoupledClosureConverged });
                    }
                    stepCaptures.length = 0; capturedSolve = null;
                    times.push(world.timings.total.last); row.steps++;
                    row.unconverged += Number(!world.lastCoupledClosureConverged);
                    row.maximumWallPenetration = Math.max(row.maximumWallPenetration, world.settledMaxPenetration);
                    row.maximumLinearResidual = Math.max(row.maximumLinearResidual, fixture.containment._jointDiagnostics?.maximumResidual ?? 0);
                    row.factorizationSum += world.lastJointFactorizations; row.outerPassSum += world.lastCoupledClosurePasses;
                    for (const b of world.bodies) for (let i = b.activeStart; i < b.activeEnd; i++) row.maximumLengthError = Math.max(row.maximumLengthError,
                        Math.abs(Math.hypot(b.x[i + 1] - b.x[i], b.y[i + 1] - b.y[i], b.z[i + 1] - b.z[i]) - b.restLength[i]) / b.restLength[i]);
                    if (index >= 3 && !world.lastCoupledClosureConverged) {
                        run.failure = { phase: phase.name, step, state: fixture.snapshot(), linear: fixture.containment._jointDiagnostics,
                            trial: fixture.containment._jointTrialFailure, trace: world.coupledClosureTrace,
                            jointMotion: motion };
                        break;
                    }
                    if (stopAfterSystemCapture && capturedSystems >= captureSystemLimit) {
                        run.stoppedAfterSystemCapture = true; break;
                    }
                }
                Object.assign(row, { physicsMs: summarize(times), fullMs: summarize(fullTimes),
                    simulatedSeconds: row.advancedPhysicsSteps * dt, attemptedSeconds: row.steps * dt,
                    meanFactors: row.factorizationSum / row.steps, meanOuterPasses: row.outerPassSum / row.steps, end: fixture.snapshot() });
                console.log(JSON.stringify({ hz, ...row }));
                if (run.failure || run.stoppedAfterSystemCapture) break;
            }
            run.complete = !run.failure && !run.stoppedAfterSystemCapture && run.phases.length === through + 1;
        } catch (error) { run.failure = { error: String(error), stack: error.stack, state: fixture.snapshot() }; }
        finally {
            run.coupledSolver = selection.getReport(world);
            run.finalJointMotion = world.getStats().jointMotion;
            if (traceTrials) fs.writeFileSync(traceTrials, JSON.stringify(trials, (_key, value) =>
                typeof value === 'number' && !Number.isFinite(value) ? String(value) : value));
            fixture.dispose(); fs.writeFileSync(output, JSON.stringify(report, null, 2));
        }
    }
} finally {
    anatomy.dispose(); report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
    if (captureSystem) report.systemCapture = { path: captureSystem, fromMm: captureSystemFrom, mode: captureSystemMode,
        captured: systemCaptured, factorizations: capturedFactorizations,
        count: capturedSystems, limit: captureSystemLimit,
        note: 'Synchronous diagnostic exports are included in their step costs; do not use this run as a clean timing comparison.' };
    fs.writeFileSync(output, JSON.stringify(report, null, 2));
}
