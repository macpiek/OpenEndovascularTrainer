import assert from 'node:assert/strict';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { defineKirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { solveKirchhoffDirect } from '../src/physics/kirchhoffDirectSolver.js';
import {
    conjugateQuaternion, multiplyQuaternions, quaternionExp, quaternionLog
} from '../src/physics/discreteKirchhoffRod.js';

const beam = defineKirchhoffMaterialProfile({
    id: 'analytical-cantilever',
    sampleEI1: () => 1e6,
    sampleGJ: () => 1e6 / 1.3
});

function frame(body, segment) {
    return { x: body.orientationX[segment], y: body.orientationY[segment],
        z: body.orientationZ[segment], w: body.orientationW[segment] };
}

function clampBase(body, segment = body.activeStart) {
    body.setPinned(segment, true);
    const q = frame(body, segment);
    body.setProximalOrientationControl(q.x, q.y, q.z, q.w, 0, segment);
}

function lengthError(body) {
    let result = 0;
    for (let s = body.activeStart; s < body.activeEnd; s++) {
        result = Math.max(result, Math.abs(Math.hypot(
            body.x[s + 1] - body.x[s], body.y[s + 1] - body.y[s],
            body.z[s + 1] - body.z[s]) - body.restLength[s]));
    }
    return result;
}

// An external engineering oracle, not parity with another implementation:
// small-deflection cantilever y(L) = F L^3 / (3 EI). Low inertia exposes
// artificial numerical softness hidden by a heavy slowly moving fixture.
const deflections = [];
for (const [spacing, dt, iterations] of [[5, 1 / 120, 6], [2.5, 1 / 120, 6],
    [5, 1 / 60, 2], [5, 1 / 240, 6]]) {
    const length = 100;
    const force = 5;
    const world = new EndovascularPhysicsWorld({ fixedDt: dt, iterations, penetrationIterations: iterations });
    const body = world.createRod('cantilever', length / spacing + 1, spacing, {
        mass: 0.001 * spacing / 5,
        rodModel: 'kirchhoff', constitutiveSolver: 'direct',
        foldLimitStrength: 0, sleepFrames: 100000,
        linearDamping: Math.pow(0.9, dt * 120),
        angularDamping: Math.pow(0.9, dt * 120)
    });
    applyKirchhoffMaterialProfile(body, beam);
    clampBase(body);
    for (let step = 0; step < Math.round(10 / dt); step++) {
        body.forceY[body.activeEnd] = force;
        world.stepFixed();
    }
    const expected = force * length ** 3 / (3e6);
    const actual = body.y[body.activeEnd];
    assert.ok(Math.abs(actual - expected) / expected < 0.025,
        `cantilever: ${actual} vs ${expected}, spacing=${spacing}, dt=${dt}, iterations=${iterations}`);
    assert.ok(lengthError(body) < 0.001, 'bending must preserve wire length');
    deflections.push({ spacing, dt, iterations, actual, expected });
    // Unloading must recover the manufactured straight state, without a
    // release-specific reset, a shape target or a centerline attraction.
    for (let step = 0; step < Math.round(10 / dt); step++) world.stepFixed();
    assert.ok(Math.abs(body.y[body.activeEnd]) < 0.025,
        `unloaded cantilever retained ${body.y[body.activeEnd]} mm deflection`);
}

// A long, very stiff wire transmits a proximal rotation to its free end in
// the same solve. Exercise an offset active range and a non-world-axis frame.
{
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('twist', 104, 2, {
        rodModel: 'kirchhoff', constitutiveSolver: 'direct',
        kirchhoffBendCompliance: 1e-12, kirchhoffTwistCompliance: 1e-12
    });
    body.setActiveRange(3, 103);
    clampBase(body, 3);
    const original = frame(body, 3);
    const target = multiplyQuaternions(original, quaternionExp({ x: 0, y: 0, z: 0.15 }));
    body.setProximalOrientationControl(target.x, target.y, target.z, target.w, 0, 3);
    const inactive = Array.from(body.x.slice(0, 3));
    for (let pass = 0; pass < 6; pass++) solveKirchhoffDirect(body);
    const twist = quaternionLog(multiplyQuaternions(conjugateQuaternion(original), frame(body, 102)));
    assert.ok(Math.abs(twist.z - 0.15) < 0.0001, `distal torque transmission: ${twist.z}`);
    assert.ok(Math.hypot(twist.x, twist.y) < 1e-7, 'pure twist must not add bending');
    assert.deepEqual(Array.from(body.x.slice(0, 3)), inactive);
    assert.ok(lengthError(body) < 1e-6);
}

// Pinned sheath nodes create redundant axial equations. Their null space
// must not cause NaNs, change the prescribed boundary or corrupt free nodes.
{
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('sheath', 21, 5, {
        rodModel: 'kirchhoff', constitutiveSolver: 'direct'
    });
    applyKirchhoffMaterialProfile(body, beam);
    for (let node = 0; node < 8; node++) body.setPinned(node, true);
    clampBase(body);
    body.y[15] += 0.5;
    for (let step = 0; step < 120; step++) world.stepFixed();
    for (let node = 0; node < body.count; node++) {
        assert.ok(Number.isFinite(body.x[node]) && Number.isFinite(body.y[node]));
        if (node < 8) assert.equal(body.x[node], node * 5);
    }
    assert.ok(lengthError(body) < 0.001);
}

console.log('Direct Kirchhoff mechanics passed', JSON.stringify(deflections));

// A newly exposed rod may contain only one segment. It still needs the
// adaptation solve even though it has no internal bending joint yet.
{
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('single-edge', 2, 5, {
        rodModel: 'kirchhoff', constitutiveSolver: 'direct'
    });
    clampBase(body);
    body.y[1] = 0.25;
    world.stepFixed();
    assert.ok(Math.abs(body.y[1]) < 1e-5);
    assert.ok(lengthError(body) < 1e-5);
}
