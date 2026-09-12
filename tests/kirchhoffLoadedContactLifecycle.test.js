import assert from 'node:assert/strict';
import test from 'node:test';
import { existsSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { captureCoupledContactAudit, stringifyCoupledContactAudit } from './helpers/coupledContactAudit.js';

const root = resolve(process.env.OET_BUNDLE_PARENT_PATH ?? fileURLToPath(new URL('../', import.meta.url)));
const worldPath = join(root, 'src/physics/endovascularPhysicsWorld.js');
const kernelPath = join(root, 'src/physics/kirchhoffCoupledSystem.js');

const skip = !existsSync(kernelPath) && 'Set OET_BUNDLE_PARENT_PATH in frozen delegation baseline';
async function runtime() {
    const { EndovascularPhysicsWorld } = await import(pathToFileURL(worldPath));
    const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await import(pathToFileURL(kernelPath));
    return { EndovascularPhysicsWorld, solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection };
}

function runScenario(runtime, halo, shift = 0) {
    const { EndovascularPhysicsWorld, solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = runtime;
    let previous = [], retainedOutsideHalo = 0, unloadedByQP = 0, solves = 0, loadedSegmentMigrations = 0;
    const world = new EndovascularPhysicsWorld({ coupledSystem: {
        solve(constraint, dt, options) {
            solves++;
            const contacts = constraint.kirchhoffContacts;
            for (const old of previous) if (old.lambdaAfter > 1e-9) {
                const retained = contacts.find(r => r.manifoldContact === old.contact);
                assert.ok(retained, `Loaded ${old.id} was dropped before its unloading solve (Fn=${old.lambdaAfter})`);
                assert.ok(Math.abs(old.contact.normalLambda - old.lambdaAfter) < 1e-12,
                    'Geometry refresh must not erase a loaded normal multiplier');
                if (old.contact.outerSegmentIndex !== old.outerSegment) {
                    loadedSegmentMigrations++;
                    assert.notEqual(old.contact.outerMaterialSegmentId, old.outerMaterialId,
                        'The closest-foot material metadata must move with the new segment');
                    assert.equal(old.contact.innerMaterialSegmentId, old.innerMaterialId);
                    assert.equal(old.contact.id, old.id, 'The inner quadrature/feature still owns this contact');
                }
                if (retained.gap > halo) retainedOutsideHalo++;
            }
            const result = solveKirchhoffCoupledSystem(constraint, dt, options);
            assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
            previous = contacts.map((record, index) => {
                const contact = record.manifoldContact, before = contact.normalLambda;
                const increment = result.scale * result.contactIncrement[index];
                const after = Math.max(0, before + increment);
                if (record.gap > halo && before > 1e-9 && after <= 1e-12) {
                    assert.ok(Math.abs(increment + before) < 1e-10);
                    unloadedByQP++;
                }
                return { contact, id: record.id, lambdaAfter: after, outerSegment: contact.outerSegmentIndex,
                    outerMaterialId: contact.outerMaterialSegmentId, innerMaterialId: contact.innerMaterialSegmentId };
            });
            return result;
        },
        apply: applyKirchhoffCoupledCorrection
    } });
    const profile = { radius: 0.4, mass: 0.03, foldLimitStrength: 0, projectionVelocityRetention: 1,
        linearDamping: 1, angularDamping: 1, sleepFrames: 1e6 };
    const inner = world.createRod('wire', 4, 5, profile), outer = world.createRod('catheter', 4, 5, profile);
    inner.y.set([0, 3, -3, 0]);
    for (let node = 0; node < inner.count; node++) inner.x[node] += shift;
    inner.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    outer.inverseMass.fill(0); outer.inverseInertia1.fill(0); outer.inverseInertia2.fill(0); outer.inverseInertia3.fill(0);
    inner.setControlTarget(0, 0, 0, 0, 1e-6); inner.setControlTarget(3, 15, 0, 0, 1e-6);
    const constraint = world.addContainment(inner, outer, { innerRadius: 1.4, openDistal: false,
        axialFriction: 0, torsionalFriction: 0, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    constraint.kirchhoffContactActivation = halo;
    world.stepFixed();
    assert.equal(world.lastCoupledSolver, 'joint'); assert.equal(world.lastCoupledClosureConverged, true);
    assert.ok(constraint.kirchhoffSolverResidual <= world.coupledContainmentTolerance);
    assert.ok(constraint._jointBoundaryResidual <= world.coupledContainmentTolerance);
    assert.ok(constraint._jointMaterialResidual.adaptationMm <= world.coupledContainmentTolerance);
    assert.ok(constraint._jointMaterialResidual.bendTwistRad <= world.coupledAngularToleranceRad);
    const listed = new Set(constraint.kirchhoffContacts.map(r => r.manifoldContact));
    for (const contact of constraint.manifold.contacts()) if (!listed.has(contact)) {
        assert.equal(contact.normalLambda, 0, `Unlisted normal force remained on ${contact.id}`);
        assert.equal(contact.tangentLambda[0], 0); assert.equal(contact.tangentLambda[1], 0);
    }
    const firstX = inner.x[0], audit = captureCoupledContactAudit(world, constraint, { purpose: 'lifecycle-regression' });
    assert.equal(audit.solver, 'joint'); assert.equal(audit.converged, true);
    assert.equal(audit.retainedUnlisted.length, 0);
    assert.equal(JSON.parse(stringifyCoupledContactAudit(audit)).constraint.containedLength, 'Infinity');
    audit.bodies[0].x[0] += 1;
    assert.equal(inner.x[0], firstX, 'An exported audit must own its arrays');
    return { retainedOutsideHalo, unloadedByQP, loadedSegmentMigrations, solves,
        pose: ['x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW']
            .flatMap(key => Array.from(inner[key])) };
}

test('loaded lumen rows unload through the QP outside the halo with halo-independent equilibrium', { skip }, async () => {
    const r = await runtime(), small = runScenario(r, 0.01), large = runScenario(r, 0.2);
    assert.ok(small.retainedOutsideHalo >= 2, 'Fixture must exercise loaded rows outside the small halo');
    assert.ok(small.unloadedByQP >= 2, 'Those rows must release their forces through the common QP');
    assert.ok(small.solves > 1 && large.solves > 1);
    small.pose.forEach((value, i) => assert.ok(Math.abs(value - large.pose[i]) < 2e-7,
        `Numerical activation halo changed equilibrium coordinate ${i}: ${value} vs ${large.pose[i]}`));
});

test('loaded lumen force survives closest outer segment rekey within a fixed step', { skip }, async () => {
    // A 0.75 mm axial shift makes a loaded quadrature foot cross outer segment
    // 1 -> 0 inside the SAME fixed step. Before the fix, Fn=7.719142... was
    // silently cleared while the contact still penetrated by 0.775160... mm.
    const migrated = runScenario(await runtime(), 0.2, 0.75);
    assert.ok(migrated.loadedSegmentMigrations > 0,
        'Fixture must cross a closest outer segment while carrying a positive normal force');
});
