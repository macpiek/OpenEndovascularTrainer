import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

// A contradiction audit, not a solver-acceptance test or expected-failure PASS.
const root = resolve(process.env.OET_SPLIT_MOTION_SOURCE_ROOT ?? '/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer');
const physics = resolve(root, 'src/physics');
const sources = fs.readdirSync(physics, { recursive: true }).filter(p => p.endsWith('.js')).sort();
const hashes = () => Object.fromEntries(sources.map(p => [p, crypto.createHash('sha256').update(fs.readFileSync(resolve(physics, p))).digest('hex')]));
const before = hashes();
assert.ok(before['endovascularPhysicsWorld.js'].startsWith('2d820149'), 'requested stable World');
assert.ok(before['kirchhoffSplitMotion.js'].startsWith('53128abc'), 'requested stable SplitMotion');
const { EndovascularPhysicsWorld } = await import(pathToFileURL(resolve(physics, 'endovascularPhysicsWorld.js')));
const { solveKirchhoffCoupledSystem: solve, applyKirchhoffCoupledCorrection: apply } = await import(pathToFileURL(resolve(physics, 'kirchhoffCoupledSystem.js')));
const dt = 1 / 120, radius = .5, incoming = 2;
const near = (a, b, tol, label) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tol, `${label}: ${a} vs ${b}`);

class Plane {
    voxelSize = .5;
    write(p, r, out) {
        const gap = -p.y - r, penetration = Math.max(0, -gap);
        Object.assign(out, { signedDistance: -p.y, signedGap: gap, penetration, inside: p.y <= 0,
            violation: gap < 0, branchId: 0, faceIndex: 0, source: 'first-impact-affine', timeOfImpact: gap < 0 ? 0 : 1 });
        Object.assign(out.point, p); Object.assign(out.closestPoint, { x: p.x, y: 0, z: p.z });
        Object.assign(out.inward, { x: 0, y: -1, z: 0 }); Object.assign(out.normal, out.inward);
        Object.assign(out.target, { x: p.x, y: p.y - penetration, z: p.z }); return out;
    }
    querySphere(p, r, out) { return this.write(p, r, out); }
    queryCapsule(a, b, r, out) { const p = a.y >= b.y ? a : b; this.write(p, r, out); out.segmentT = p === a ? 0 : 1; return out; }
    sweepSphere(a, b, r, out) { this.write(b, r, out); const ga = -a.y - r, gb = -b.y - r;
        out.timeOfImpact = gb >= 0 ? 1 : ga <= 0 ? 0 : ga / (ga - gb); return out; }
}

function audit(requestedGap) {
    let predictor; const linear = [], phases = [];
    const world = new EndovascularPhysicsWorld({ fixedDt: dt, jointMotionMode: 'split-physical-bias', contactField: new Plane(),
        coupledSystem: { solve(c, step, options) {
            const rows = options.additionalRows.filter(r => ['wall', 'split-point-wall'].includes(r.kind)).map(r => ({
                kind: r.kind, node: r.node, startGap: r._splitStartGap, physicalRow: r.strain, lambdaBefore: r.lambda }));
            const result = solve(c, step, { ...options, activeCondensation: true, simultaneousCoulomb: true });
            linear.push({ converged: result.diagnostics.converged, residual: result.diagnostics.maximumResidual, rows }); return result;
        }, apply } });
    const profile = { radius, mass: 1, inverseAngularInertia: 1, adaptationCompliance: 0,
        kirchhoffBendCompliance: 0, kirchhoffTwistCompliance: 0, foldLimitStrength: 0,
        linearDamping: 1, angularDamping: 1, projectionVelocityRetention: 0, wallProjectionVelocityRetention: 0,
        toolProjectionVelocityRetention: 0, wallStaticFriction: 0, wallKineticFriction: 0, wallCompliance: 0,
        sleepFrames: 1e6, sleepVelocity: 0, sleepAngularVelocity: 0 };
    const wire = world.createRod('impact-wire', 2, .5, profile), support = world.createRod('supported-pair', 3, 1, profile);
    for (let i = 0; i < 2; i++) wire.setNodePosition(i, 1.25 + .5 * i, -radius - requestedGap, 0);
    for (let i = 0; i < 3; i++) support.setNodePosition(i, i, -4, 0);
    support.inverseMass.fill(0); for (const a of [1, 2, 3]) support['inverseInertia' + a].fill(0);
    world.addContainment(wire, support, { innerRadius: 100, openDistal: false, openProximal: false,
        axialFriction: 0, torsionalFriction: 0, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    wire.copyCurrentToPrevious(); support.copyCurrentToPrevious(); wire.velocityY.fill(incoming);
    const startGap = -wire.y[0] - radius, initialY = [...wire.y];
    wire.debugConstraintPhase = phase => { if (phase === 'afterIntegrate') predictor = Array.from(wire.y, (v, i) => (v - initialY[i]) / dt); };
    world.debugJointTrial = (c, state, pass, trial) => phases.push({ phase: state.motionPhase, pass, trial, settled: state.settled, merit: state.merit });
    world.stepFixed(); const stats = world.getStats(), d = stats.jointMotion;
    const velocity = ['X', 'Y', 'Z'].map(a => [...wire['velocity' + a]]), omega = ['X', 'Y', 'Z'].map(a => [...wire['angularVelocity' + a]]);
    const gaps = Array.from(wire.y, y => -y - radius), normalLambda = d.contacts.reduce((s, c) => s + c.normalPhysical, 0);
    const physicalRow = velocity[1].map(v => Math.max(0, startGap) - dt * v);
    const nominalKinetic = incoming * incoming, predictorKinetic = predictor.reduce((s, v) => s + .5 * v * v, 0);
    const kinetic = [...velocity, ...omega].flat().reduce((s, v) => s + .5 * v * v, 0); // all active wire m=I=1
    const impulse = normalLambda / dt, momentumLoss = predictor.reduce((s, v, i) => s + v - velocity[1][i], 0);
    assert.equal(d.mode, 'split-physical-bias'); assert.equal(d.sweptWitnesses, 0);
    assert.ok(d.physicalAccepted && d.biasAccepted && d.finalPhysicalResidualSettled);
    assert.ok(linear.every(s => s.converged)); assert.ok(d.physicalKKTResidualMm <= world.coupledContainmentTolerance);
    assert.ok(gaps.every(g => Math.abs(g) <= 2e-6) && physicalRow.every(g => Math.abs(g) <= 2e-6));
    velocity[1].forEach(v => near(v, startGap / dt, 5e-4, 'active discrete contact velocity'));
    near(impulse, momentumLoss, .002, 'normal impulse and physical momentum');
    assert.ok(kinetic <= predictorKinetic + 2e-6); assert.equal(d.physicalConeViolation, 0);
    const gateResidualMm = d.maximumOutwardContactVelocity * dt;
    const gateRejected = gateResidualMm > world.coupledContainmentTolerance;
    assert.equal(d.certified, !gateRejected); assert.equal(d.historyCommits, Number(!gateRejected));
    return { requestedGap, storedStartGap: startGap, dt, nominalIncoming: incoming, predictorVelocity: predictor,
        analyticDiscreteClosingVelocity: startGap / dt, rawFinalGaps: gaps, velocity, omega,
        independentDiscreteContactResidualMm: physicalRow, normalLambda, impulse, momentumLoss,
        kineticEnergy: { nominalInitial: nominalKinetic, afterFloat32Prediction: predictorKinetic, final: kinetic, dissipated: predictorKinetic - kinetic },
        gateResidualMm, gateToleranceMm: world.coupledContainmentTolerance, gateRejected,
        additionalTerminalStoppingImpulse: velocity[1].reduce((s, v) => s + v, 0),
        nominalFirstImpactTime: startGap / incoming,
        worldClosureConverged: stats.coupledClosureConverged, diagnosis: d, linear, phases };
}

const cases = [audit(0), audit(.0005), audit(.01)];
const after = hashes(); assert.deepEqual(after, before, 'stable physics source during the complete audit');
const result = { sourceRoot: root, sourceHashesBefore: before, sourceHashesAfter: after, sourceUnchanged: true,
    auditKind: 'discrete-law versus terminal-velocity gate; no expected-failure acceptance', cases,
    conflictReproduced: cases[2].gateRejected && cases[2].diagnosis.finalPhysicalResidualSettled,
    interpretation: 'For an active positive-gap contact the discrete complementarity law fixes dt*v_close=g_start; the additional gate demands dt*v_close<=.001. The .01-mm first-impact case satisfies the former and fails the latter. An actual terminal stop needs an additional impulse or a separately defined CCD/integration law.' };
fs.writeFileSync(new URL('./split-first-impact-gate-proof.json', import.meta.url), JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify({ conflictReproduced: result.conflictReproduced, sourceUnchanged: true, cases: cases.map(c => ({
    gap: c.storedStartGap, closingVelocity: c.velocity[1], impulse: c.impulse, kinetic: c.kineticEnergy.final,
    rawGap: c.rawFinalGaps, KKT: c.diagnosis.physicalKKTResidualMm, gateResidual: c.gateResidualMm,
    certified: c.diagnosis.certified, historyCommits: c.diagnosis.historyCommits })) }, null, 2));
