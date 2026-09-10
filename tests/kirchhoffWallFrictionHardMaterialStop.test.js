import assert from 'node:assert/strict';
import test from 'node:test';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const root = resolve(process.env.OET_HARD_MATERIAL_STOP_SOURCE_ROOT ?? fileURLToPath(new URL('../', import.meta.url)));
const source = path => pathToFileURL(resolve(root, path));
const { fixture, DT } = await import(source('tests/fixtures/splitMotionAnalyticWorld.js'));
const { evaluateKirchhoffWallFrictionCandidate, captureKirchhoffWallFrictionIncoming,
    initializeKirchhoffWallFrictionModes, prepareKirchhoffWallFrictionEntry } = await import(source('src/physics/kirchhoffWallFrictionMode.js'));
const { assembleKirchhoffDirect } = await import(source('src/physics/kirchhoffDirectSolver.js'));

const TARGET = 'split-point-wall:0:2';
function captured() {
    const f = fixture({ wall: true, y: -.5 });
    f.wire.wallStaticFriction = .6; f.wire.wallKineticFriction = .2;
    for (const a of [1, 2, 3]) f.wire['inverseInertia' + a].fill(0);
    for (let i = 0; i < 2; i++) {
        f.wire.forceX.fill(.4 / DT); f.wire.forceY.fill(1 / DT);
        assert.equal(f.world.stepFixed().accepted, true, 'two real native World steps');
    }
    f.state = f.constraint._splitMotion;
    f.incoming = f.state.wallFrictionModes.incoming.bodies[0];
    f.batch = f.constraint._jointSplitWallFrictionResidual._batch;
    f.motion = f.state.bodies[0];
    f.target = f.batch.entries.find(e => e.contact.key === TARGET);
    return f;
}
function evidence(f) {
    return evaluateKirchhoffWallFrictionCandidate(f.constraint, f.batch, { converged: true }).contacts.find(c => c.key === TARGET);
}

test('native static dual preserves original primal, reactions and nonzero constitutive RHS', () => {
    const f = captured(), native = assembleKirchhoffDirect(f.wire, DT);
    const before = { velocity: [...f.motion.velocityX], x: [...f.wire.x], material: [...native.lambda],
        contacts: f.batch.entries.map(e => [e.contact.normalLambda, ...e.contact.tangentLambda]) };
    const decision = evaluateKirchhoffWallFrictionCandidate(f.constraint, f.batch, { converged: true });
    const c = decision.contacts.find(c => c.key === TARGET), proof = decision.staticDualCertificates[0];
    assert.equal(c.stopped, true); assert.equal(c.stopCertificate, 'native-hard-material-static-dual');
    assert.equal(c.normalLambda, 0); assert.deepEqual(c.lambda, [0, 0]);
    assert.equal(proof.nativeGradientVerification, true);
    assert.ok(proof.totalNormalLambda > 0 && Math.hypot(...proof.totalWorldTangent) < proof.staticCapacity);
    assert.ok(proof.materialDelta.every(row => row.alphaDeltaLambda === 0));
    assert.ok(proof.mobileBalance.every(row => Math.abs(row.residual) <= row.arithmeticBound));
    assert.ok(proof.maximumTangentResidualMm <= f.state.wallFrictionModes.displacementToleranceMm);
    assert.ok(before.velocity[0] !== before.velocity[1] && before.velocity[0] !== before.velocity[2]);
    assert.deepEqual([...f.motion.velocityX], before.velocity); assert.deepEqual([...f.wire.x], before.x);
    assert.deepEqual([...native.lambda], before.material);
    assert.deepEqual(f.batch.entries.map(e => [e.contact.normalLambda, ...e.contact.tangentLambda]), before.contacts);
    assert.notEqual(native.strain[3], 0); assert.notEqual(native.strain[9], 0);
    assert.equal(native.alpha[3], 0); assert.equal(native.alpha[9], 0);
    assert.equal(f.constraint._wallFrictionHistory.records.get(TARGET).materialSignature.last, 2);
    // Independent force/torque oracle for this straight x-axis rod. Adaptation
    // x rows carry axial forces only; the contact offset is +.5 y everywhere.
    const forces = [0, 0, 0], torques = [0, 0];
    for (const witness of proof.alternateContacts) {
        const entry = f.batch.entries.find(e => e.contact.key === witness.key);
        const deltaX = witness.lambda[0] - witness.originalLambda[0], segment = entry.record._innerSegmentIndex;
        entry.record.innerWeights.forEach((w, i) => { forces[segment + i] += w * deltaX; });
        torques[segment] += -.5 * deltaX;
        assert.equal(witness.normalLambda, entry.contact.normalLambda);
        if (witness.normalLambda > 0) assert.ok(Math.hypot(...witness.lambda) < .6 * witness.normalLambda);
    }
    for (const row of proof.materialDelta) {
        if (row.axis !== 0) { assert.equal(row.deltaLambda, 0); continue; }
        forces[row.segment] -= row.deltaLambda; forces[row.segment + 1] += row.deltaLambda;
    }
    assert.ok(forces.every(value => Math.abs(value) < 1e-17));
    for (const reaction of proof.fixedSupportTorqueDelta) {
        assert.equal(reaction.materialAxis, 1);
        assert.ok(Math.abs(reaction.deltaReaction + torques[reaction.segment]) < 1e-17);
    }
    assert.ok(Math.abs(torques[0] + torques[1]) < 1e-17);
    const owned = structuredClone(proof);
    native.gradients.fill(123); f.target.contact.tangentLambda.fill(456);
    assert.deepEqual(proof, owned, 'the certificate owns its witness after later native scratch/bank mutation');
});

const negatives = [
    ['finite native compliance', f => { f.wire.adaptationCompliance = 1e-12; }],
    ['tiny commanded motion with a nonzero prescribed RHS', f => {
        f.wire.setControlTarget(2, 1 + 1e-6, -.5, 0, 0); f.motion.velocityX.fill(3.6e-9);
    }],
    ['nonzero prescribed RHS captured before prediction', f => { f.incoming.controlEnabled[2] = 1; }],
    ['changed rest data after incoming capture', f => { f.wire.restLength[0] += .001; }],
    ['moving prescribed translation in the chain', f => { f.wire.inverseMass[1] = 0; f.motion.velocityX[1] = .1; }],
    ['moving angular support', f => { f.motion.angularVelocityX[0] = .1; }],
    ['nonzero incoming angular rate', f => { f.incoming.angularVelocityX[0] = .1; }],
    ['changed frame after incoming capture', f => { f.wire.orientationX[0] = .4; }],
    ['different collider feature', f => { f.target.modeRecord.identity.face = 1; }],
    ['different wall plane', f => { f.target.modeRecord.identity.plane = .1; }],
    ['missing immutable dt history', f => { delete f.state.start; }],
    ['material relabeling', f => { f.wire.materialCoordinate[1] = 10; }],
    ['free unloaded contact without any loaded source', f => {
        f.wire.wallLambda.fill(0);
        for (const e of f.batch.entries) { e.normalRow.lambda = 0; e.contact.tangentLambda.fill(0); }
    }],
    ['same tiny speeds without hard-row representation', f => {
        f.wire.adaptationCompliance = .02; f.motion.velocityX.fill(3.6e-9);
    }],
    ['unresolved static tangent equality', f => { f.motion.velocityX[2] += 1; }],
    ['incoming sliding mode despite identical tiny current velocities', f => {
        f.target.modeRecord.mode = 'slide'; f.motion.velocityX.fill(3.6e-9);
    }],
    ['native gradient failing independent generalized-force balance', f => {
        const native = f.wire.kirchhoffScratch.direct; native.gradients[0] *= 1.01;
    }]
];
for (const [name, mutate] of negatives) test('hard stop proof rejects ' + name, () => {
    const f = captured(); mutate(f); const c = evidence(f);
    assert.equal(c.stopped, false); assert.equal(c.stopCertificate, null); assert.equal(c.kinematicStopProof, undefined);
});

test('changed material history invalidates a committed inherited stop at the next incoming capture', () => {
    const f = captured();
    f.wire.adaptationCompliance = .02;
    const incoming = captureKirchhoffWallFrictionIncoming(f.constraint, f.world);
    f.state.phase = 'physical'; f.state.step = f.world.stepCount;
    initializeKirchhoffWallFrictionModes(f.constraint, incoming, { displacementToleranceMm: .0002 });
    prepareKirchhoffWallFrictionEntry(f.constraint, f.target);
    assert.equal(f.target.modeRecord.mode, 'slide');
    assert.equal(f.target.modeRecord.incomingCertificate, 'nonzero-incoming');
});

test('externally commanded tiny incoming motion cannot borrow the prior global static certificate', () => {
    const f = captured();
    f.wire.velocityX.fill(3.6e-9);
    const incoming = captureKirchhoffWallFrictionIncoming(f.constraint, f.world);
    f.state.phase = 'physical'; f.state.step = f.world.stepCount;
    initializeKirchhoffWallFrictionModes(f.constraint, incoming, { displacementToleranceMm: .0002 });
    for (const entry of f.batch.entries) prepareKirchhoffWallFrictionEntry(f.constraint, entry);
    assert.ok(f.batch.entries.every(entry => entry.modeRecord.mode === 'slide'));
    const before = [...f.wire.velocityX], decision = evaluateKirchhoffWallFrictionCandidate(f.constraint, f.batch, { converged: true });
    assert.equal(decision.staticDualCertificates, undefined);
    assert.equal(decision.contacts.find(c => c.key === TARGET).stopped, false);
    assert.deepEqual([...f.wire.velocityX], before);
});
