import assert from 'node:assert/strict';
import test from 'node:test';
import { Vector3, Quaternion } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffSplitMotion, prepareKirchhoffSplitBoundaryRows, applyKirchhoffSplitPhysicalIncrement } from '../src/physics/kirchhoffSplitMotion.js';
import { captureKirchhoffWallFrictionIncoming, initializeKirchhoffWallFrictionModes, evaluateKirchhoffWallFrictionCandidate } from '../src/physics/kirchhoffWallFrictionMode.js';
import { buildKirchhoffSplitWallFriction, appendKirchhoffSplitWallFriction, commitKirchhoffSplitWallFriction } from '../src/physics/kirchhoffSplitWallFriction.js';

const DT = 1 / 120, XYZ = ['X', 'Y', 'Z'], xyz = ['x', 'y', 'z'];
const near = (a, b, tolerance, message) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tolerance, `${message}: ${a} vs ${b}`);
const point = (body, i) => new Vector3(...xyz.map(a => body[a][i]));
const frame = (body, i) => new Quaternion(...[...XYZ, 'W'].map(a => body['orientation' + a][i])).normalize();

test('unequal kinetic wall rows keep the native full-rod surface impulse and radius moment with free angular DOFs', t => {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT, jointMotionMode: 'split-physical-bias' });
    const profile = { radius: .5, mass: 1, inverseAngularInertia: 1, adaptationCompliance: 0,
        kirchhoffBendCompliance: 0, kirchhoffTwistCompliance: 0, foldLimitStrength: 0,
        linearDamping: 1, angularDamping: 1, wallStaticFriction: .6, wallKineticFriction: .2 };
    const wire = world.createRod('free-frame-wall-witness', 2, .5, profile);
    const catheter = world.createRod('remote-supported-catheter', 3, 1, { ...profile, wallStaticFriction: 0, wallKineticFriction: 0 });
    for (let i = 0; i < wire.count; i++) wire.setNodePosition(i, 1.25 + .5 * i, -.5, 0);
    for (let i = 0; i < catheter.count; i++) catheter.setNodePosition(i, i - 1, -4, 0);
    catheter.inverseMass.fill(0); for (let a = 1; a <= 3; a++) catheter['inverseInertia' + a].fill(0);
    const joint = world.addContainment(wire, catheter, { innerRadius: 100, openDistal: false, openProximal: false,
        axialFriction: 0, torsionalFriction: 0, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    wire.copyCurrentToPrevious(); catheter.copyCurrentToPrevious();
    wire.velocityX.fill(4); wire.velocityY.fill(1); wire.velocityZ.fill(2);
    const incoming = captureKirchhoffWallFrictionIncoming(joint, world);
    for (const a of XYZ) for (let i = 0; i < wire.count; i++) wire[a.toLowerCase()][i] += DT * wire['velocity' + a][i];
    beginKirchhoffSplitMotion(joint, world);
    initializeKirchhoffWallFrictionModes(joint, incoming, { displacementToleranceMm: 1e-8 });
    joint.kirchhoffContacts.length = 0;
    // Two endpoint witnesses on the analytic y=0 plane. No production force
    // Jacobian supplies the expected surface points or resultant wrench.
    const normalRows = [0, 1].map(node => ({ kind: 'split-point-wall', side: 0, node, owner: wire,
        lambda: 0, alpha: 0, strain: -wire.y[node] - .5, lower: 0, upper: Infinity, normal: [0, -1, 0],
        gradients: [{ side: 0, dof: node * 6 + 1, value: -1 }],
        wallFrictionWitness: { branchId: 0, faceIndex: 0, planeOffset: 0 } }));
    prepareKirchhoffSplitBoundaryRows(joint, normalRows);
    const rows = [...normalRows], groups = [], batch = buildKirchhoffSplitWallFriction(joint, rows, DT);
    appendKirchhoffSplitWallFriction(batch, rows, groups);
    assert.ok(batch.entries.every(e => e.modeRecord.mode === 'slide'));
    assert.ok(batch.groups.every(g => [...g.mu].every(mu => mu === .2)));
    const result = solveKirchhoffCoupledSystem(joint, DT, { additionalRows: rows, groups, resolveNormalLoads: true,
        activeCondensation: true, simultaneousCoulomb: true, tolerance: 1e-8, numericalShift: 1e-8 });
    assert.equal(result.diagnostics.converged, true, JSON.stringify(result.diagnostics));
    const expectedForce = new Vector3(), expectedMoment = new Vector3(), radialMoment = new Vector3();
    const factor = result.scale / DT;
    for (let i = 0; i < normalRows.length; i++) {
        const p = point(wire, i).add(new Vector3(0, .5, 0));
        const force = new Vector3(0, -factor * result.additionalIncrement[i], 0);
        const e = batch.entries[i];
        for (let axis = 0; axis < 2; axis++) {
            const tangent = new Vector3(...e.surface.axes[axis]).multiplyScalar(factor * result.additionalIncrement[batch.rowOffset + e.rowStart + axis]);
            force.add(tangent); radialMoment.add(new Vector3(0, .5, 0).cross(tangent));
        }
        expectedForce.add(force); expectedMoment.add(p.clone().cross(force));
    }
    const actualForce = new Vector3(), actualMoment = new Vector3();
    for (const [body, response] of [[wire, result.inner], [catheter, result.outer]]) {
        for (let node = body.activeStart; node <= body.activeEnd; node++) {
            if (body.inverseMass[node] > 0) {
                const impulse = new Vector3(...[0, 1, 2].map(a => factor * response.correction[node * 6 + a] / body.inverseMass[node]));
                actualForce.add(impulse); actualMoment.add(point(body, node).cross(impulse));
            }
            if (node < body.activeEnd) {
                const local = new Vector3(...[0, 1, 2].map(a => body['inverseInertia' + (a + 1)][node] > 0 ?
                    factor * response.correction[node * 6 + 3 + a] / body['inverseInertia' + (a + 1)][node] : 0));
                actualMoment.add(local.applyQuaternion(frame(body, node)));
            }
        }
    }
    for (const a of xyz) {
        near(actualForce[a], expectedForce[a], 1e-9, 'total linear impulse ' + a);
        near(actualMoment[a], expectedMoment[a], 1e-9, 'total angular impulse ' + a);
    }
    assert.ok(radialMoment.length() > .01, 'nonzero radius torque must be exercised');
    applyKirchhoffSplitPhysicalIncrement(joint, result); applyKirchhoffCoupledCorrection(joint, result);
    for (let i = 0; i < normalRows.length; i++) normalRows[i].lambda += result.scale * result.additionalIncrement[i];
    commitKirchhoffSplitWallFriction(batch, result.additionalIncrement, result.scale);
    const fresh = buildKirchhoffSplitWallFriction(joint, normalRows, DT);
    const decision = evaluateKirchhoffWallFrictionCandidate(joint, fresh, { converged: true });
    assert.equal(decision.accepted, true, JSON.stringify(decision));
    assert.ok(joint._splitMotion.bodies[0].angularVelocityX.some(value => Math.abs(value) > .001), 'actual physical angular response');
    for (const c of decision.contacts) {
        near(Math.hypot(...c.lambda), .2 * c.normalLambda, 1e-9, 'kinetic boundary');
        assert.ok(c.lambda.reduce((sum, value, i) => sum + value * c.displacement[i], 0) <= 0, 'friction opposes the physical surface slip');
    }
    t.diagnostic(JSON.stringify({ expectedForce: expectedForce.toArray(), actualForce: actualForce.toArray(),
        expectedMoment: expectedMoment.toArray(), actualMoment: actualMoment.toArray(), radialMoment: radialMoment.toArray(),
        contacts: decision.contacts.map(c => ({ muApplied: c.muApplied, residualMm: c.residualMm, coneViolation: c.coneViolation })) }));
});
