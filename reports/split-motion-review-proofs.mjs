// Bounded post-fix review: one 3-node/5-node World step and one contact-bank
// lifecycle proof. No scene replay, source mutation, or alternate solver.
import assert from 'node:assert/strict';
import { fixture, frame, source } from './split-motion-review-fixture.mjs';
const { assembleKirchhoffDirect } = await import(source('src/physics/kirchhoffDirectSolver.js'));
const { beginKirchhoffSplitMotion, beginKirchhoffSplitBias, finishKirchhoffSplitBias } =
    await import(source('src/physics/kirchhoffSplitMotion.js'));

function nativeMaterial(body, dt) {
    const system = assembleKirchhoffDirect(body, dt);
    let adaptationMm = 0, bendTwistRad = 0, elasticEnergy = 0;
    for (let row = 0; row < system.rowCount; row += 6) {
        const residual = axis => system.strain[row + axis] + system.alpha[row + axis] * system.lambda[row + axis];
        adaptationMm = Math.max(adaptationMm, Math.hypot(residual(3), residual(4), residual(5)));
        bendTwistRad = Math.max(bendTwistRad, Math.hypot(residual(0), residual(1), residual(2)));
        for (let axis = 0; axis < 6; axis++) if (system.alpha[row + axis] > 0)
            elasticEnergy += system.strain[row + axis] ** 2 / (2 * system.alpha[row + axis] * dt * dt);
    }
    return { adaptationMm, bendTwistRad, elasticEnergy };
}
function tiltedWall() {
    const f = fixture({ wall: true, y: -.5 });
    for (let i = 0; i < f.wire.count; i++) f.wire.setNodePosition(i, i - 1, -.5 + .2 * (i - 1), 0);
    f.wire.captureRestConfiguration(); f.wire.copyCurrentToPrevious();
    const initial = nativeMaterial(f.wire, f.world.fixedDt);
    assert.ok(initial.adaptationMm < 1e-6 && initial.bendTwistRad === 0 && initial.elasticEnergy === 0);
    f.world.stepFixed();
    const d = f.constraint._splitMotion.diagnostics, actual = nativeMaterial(f.wire, f.world.fixedDt);
    const angle = frame(f.wire, 0).angleTo(frame(f.wire, 1));
    assert.ok(angle > .07, 'independent frame angle confirms the nonuniform geometric bend');
    assert.ok(actual.bendTwistRad > 70 * f.world.coupledAngularToleranceRad);
    assert.equal(f.world.lastCoupledClosureConverged, false);
    assert.equal(d.certified, false); assert.equal(d.historyCommits, 0);
    assert.ok(Math.abs(d.finalMaterialResidual.bendTwistRad - actual.bendTwistRad) < 1e-12);
    assert.ok(d.physicalPhaseMaterialResidual.bendTwistRad < f.world.coupledAngularToleranceRad);
    return { initial, actual, independentFrameAngleRad: angle,
        worldConverged: f.world.lastCoupledClosureConverged, diagnostics: d };
}
function relabelledContact() {
    const f = fixture(), joint = f.constraint;
    joint.manifold.beginStep();
    const contact = joint.manifold.upsertContact({ id: 'stable-runtime-slot',
        innerMaterialSegmentId: 'inner-old', outerMaterialSegmentId: 'outer-old', feature: 'side',
        innerSegmentIndex: 0, outerSegmentIndex: 0, normal: [0, 1, 0], tangentU: [1, 0, 0] });
    contact.normalLambda = 2; contact.tangentLambda[0] = .3;
    beginKirchhoffSplitMotion(joint, f.world); beginKirchhoffSplitBias(joint, f.world, true);
    joint.manifold.rekeyKnownContact(contact, { id: 'stable-runtime-slot',
        innerMaterialSegmentId: 'inner-new', outerMaterialSegmentId: 'outer-new', feature: 'side',
        innerSegmentIndex: 1, outerSegmentIndex: 1, normal: [0, 1, 0], tangentU: [1, 0, 0] });
    finishKirchhoffSplitBias(joint, true);
    assert.equal(contact.id, 'stable-runtime-slot', 'identity cannot be checked using only the runtime id');
    assert.equal(contact.normalLambda, 0); assert.deepEqual(Array.from(contact.tangentLambda), [0, 0]);
    assert.ok(joint._splitMotion.diagnostics.unverifiedHistoryKinds.includes('contact-identity-changed-during-bias'));
    const saved = joint._splitMotion.bank.contacts[0];
    assert.equal(saved.normalLambda, 2); assert.equal(saved.tangentLambda[0], .3);
    assert.equal(saved.innerMaterialSegmentId, 'inner-old');
    assert.equal(joint._splitMotion.diagnostics.historyCommits, 0);
    return { id: contact.id, currentInner: contact.innerMaterialSegmentId, currentOuter: contact.outerMaterialSegmentId,
        currentFn: contact.normalLambda, currentFt: Array.from(contact.tangentLambda),
        savedInner: saved.innerMaterialSegmentId, savedFn: saved.normalLambda, savedFt: Array.from(saved.tangentLambda),
        limitations: joint._splitMotion.diagnostics.unverifiedHistoryKinds, historyCommits: 0 };
}
console.log(JSON.stringify({ scope: 'Two bounded post-guard review proofs; no full-scene or performance claim',
    tiltedWall: tiltedWall(), relabelledContact: relabelledContact(), checksPassed: 2 }, null, 2));
