import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy, poseFingerprint } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';

// Reproduce UI insertion depths without disturbing the live browser trajectory.
const output = process.argv[2] ?? '/tmp/oet-overlap-309.json';
class World extends EndovascularPhysicsWorld {
    stepFixed() {
        for (const body of this.bodies) configureKirchhoffToolRuntime(body);
        return super.stepFixed();
    }
}
const report = { scope: 'Node physics replay; same depths and defaults, not the exact browser contact history or rendered FPS',
    interToolFriction: process.env.OET_INTER_TOOL_FRICTION === '1',
    compactContactTrial: process.env.OET_COMPACT_CONTACT_TRIAL !== '0',
    reuseCandidateEvaluation: process.env.OET_REUSE_CANDIDATE_EVALUATION !== '0',
    reuseAcceptedEvaluation: process.env.OET_REUSE_ACCEPTED_EVALUATION !== '0',
    fixedDt: 1 / 120, wireMm: 309, catheterMm: 100, startedAt: new Date().toISOString(), scenarios: [] };
const summary = xs => {
    const ys = xs.slice().sort((a,b) => a-b);
    return { mean: xs.reduce((a,b) => a+b,0)/xs.length, p50: ys[Math.floor(ys.length/2)],
        p95: ys[Math.ceil(ys.length*.95)-1], max: ys.at(-1) };
};
function physicalHash(fixture) {
    const hash = createHash('sha256');
    for (const body of fixture.world.bodies) for (const key of Object.keys(body).sort()) {
        const value = body[key];
        if (ArrayBuffer.isView(value)) {
            hash.update(key);
            hash.update(new Uint8Array(value.buffer, value.byteOffset, value.byteLength));
        }
    }
    for (const contact of fixture.containment.manifold.contacts()) for (const key of Object.keys(contact).sort()) {
        const value = contact[key];
        if (ArrayBuffer.isView(value)) {
            hash.update(key); hash.update(new Uint8Array(value.buffer, value.byteOffset, value.byteLength));
        } else if (value === null || ['number', 'boolean', 'string'].includes(typeof value)) hash.update(JSON.stringify([key, value]));
    }
    return hash.digest('hex');
}
for (const scenario of ['wire-alone', 'catheter-alone', 'overlap']) {
    const anatomy = await loadCoupledRuntimeAnatomy();
    let linear = [];
    const fixture = createCoupledRuntimeFixture({ ...anatomy, World, interToolFriction: report.interToolFriction, coupledSystem: {
        independentComponents: true, physicalTrialState: true, earlyTrialRejection: true,
        compactContactTrial: report.compactContactTrial,
        reuseCandidateEvaluation: report.reuseCandidateEvaluation,
        reuseAcceptedEvaluation: report.reuseAcceptedEvaluation,
        solve(constraint, dt, options) {
            const result = solveKirchhoffCoupledSystem(constraint, dt, {
                ...options, activeCondensation: true, simultaneousCoulomb: true
            });
            linear.push({ ...result.diagnostics, condensedCosts: { ...result.diagnostics.condensedCosts } });
            return result;
        }, apply: applyKirchhoffCoupledCorrection
    } });
    const record = { scenario, phases: [] };
    report.scenarios.push(record);
    report.config ??= Object.fromEntries(Object.entries(fixture.config).filter(([,v]) => ['number','boolean','string'].includes(typeof v)));
    async function phase(name, tool, target, hold = 0, extraCommands = {}) {
        const rows = [], start = tool === 'guidewire' ? fixture.transport.progress : fixture.catheter.progress;
        const direction = Math.sign(target - start);
        const rate = tool === 'guidewire' ? 44 : direction < 0 ? 32 : 52;
        const count = tool ? Math.ceil(Math.abs(target-start)/(rate/120)) : hold;
        for (let i=0;i<count;i++) {
            linear = [];
            const t = performance.now();
            const result = fixture.step({ ...extraCommands, ...(tool ? { [tool+'Advance']: direction * Math.min(1, Math.max(0,(Math.abs(target-start)-i*rate/120)/(rate/120))) } : {}) });
            const fullMs = performance.now()-t, w = fixture.world;
            rows.push({ fullMs, wireMm: fixture.transport.progress, catheterMm: fixture.catheter.progress,
                poseHashes: w.bodies.map(poseFingerprint),
                physicalHash: physicalHash(fixture),
                timings: Object.fromEntries(Object.entries(w.timings).map(([k,v]) => [k,v.last])),
                costs: {...w.lastJointCosts}, passes: w.lastCoupledClosurePasses, trials: w.lastJointTrialEvaluations,
                backtracks: w.lastJointBacktracks, rows: w.lastJointMaximumRows, factorizations: w.lastJointFactorizations,
                solver: w.lastCoupledSolver, failure: w.lastJointNonlinearFailure ?? null, rejected: result.accepted === false,
                penetrationMm: w.settledMaxPenetration,
                lumenPenetrationMm: fixture.containment.kirchhoffMaxViolation,
                lumenFrictionRows: fixture.containment._jointFrictionBatch?.rows.length ?? 0,
                lumenNormalContacts: fixture.containment.kirchhoffContacts.length,
                maximumLumenTangentialForce: Math.max(0, ...[...fixture.containment.manifold.contacts()].map(c => Math.hypot(...c.tangentLambda, c.twistLambda))),
                finitePose: w.bodies.every(b => ['x','y','z','orientationX','orientationY','orientationZ','orientationW'].every(k => b[k].every(Number.isFinite))),
                activeNodes: w.bodies.map(b => b.activeEnd-b.activeStart+1),
                linear: linear.map(d => Object.fromEntries(Object.entries(d).filter(([k,v]) =>
                    typeof v === 'number' || typeof v === 'boolean' || typeof v === 'string'))) });
        }
        const measured = tool ? rows.slice(-Math.min(rows.length, 47)) : rows;
        const stats = { name, steps: rows.length, measuredSteps: measured.length,
            fullMs: summary(measured.map(r => r.fullMs)),
            timings: Object.fromEntries(Object.keys(measured[0].timings).map(k => [k, summary(measured.map(r => r.timings[k]))])),
            meanCosts: Object.fromEntries(Object.keys(measured[0].costs).map(k => [k, summary(measured.map(r => r.costs[k])).mean])),
            counters: Object.fromEntries(['passes','trials','backtracks','rows','factorizations'].map(k => [k,summary(measured.map(r=>r[k]))])),
            failures: rows.filter(r=>r.failure || r.rejected).length, samples: rows };
        record.phases.push(stats);
        fs.writeFileSync(output, JSON.stringify(report,null,2));
        console.log(JSON.stringify({ scenario, phase:name, ms:stats.fullMs, costs:stats.meanCosts, counters:stats.counters, failures:stats.failures }));
    }
    try {
        if (scenario !== 'catheter-alone') await phase('wire-feed-309', 'guidewire', 309);
        if (scenario === 'wire-alone') await phase('wire-hold', null, 0, 60);
        else {
            for (const target of [20,50,100]) await phase('catheter-feed-'+target, 'catheter', target);
            await phase('catheter-hold-100', null, 0, 60);
            if (process.env.OET_VERIFY_WITHDRAWAL === '1') {
                await phase('catheter-withdraw-80', 'catheter', 80);
                await phase('catheter-refeed-100', 'catheter', 100);
            }
        }
        if (scenario === 'overlap' && process.env.OET_VERIFY_BOTH === '1') {
            await phase('both-feed-120', 'catheter', 120, 0, { guidewireAdvance: 1 });
            await phase('wire-withdraw-inside', 'guidewire', 309);
            await phase('wire-rotate-inside', null, 0, 30, { guidewireRotation: 1 });
            await phase('catheter-rotate-over-wire', null, 0, 30, { catheterRotation: 1 });
        }
    } finally { fixture.dispose(); anatomy.dispose(); }
}
report.complete = true;
fs.writeFileSync(output, JSON.stringify(report,null,2));
