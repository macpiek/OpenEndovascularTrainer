import assert from 'node:assert/strict';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffDirect, kirchhoffDirectContactResponse } from '../src/physics/kirchhoffDirectSolver.js';
import { applyKirchhoffMaterialProfile } from '../src/physics/applyKirchhoffMaterialProfile.js';
import { defineKirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import {
    evaluateBendTwistConstraint, evaluateBendTwistLocalConstraintNormalized,
    materialFrameDirectors, multiplyQuaternions, quaternionExp
} from '../src/physics/discreteKirchhoffRod.js';

const DT = 1 / 120;

// Differentiate the public SO(3) strain with right-sided material-frame
// perturbations. This checks the optimized gradients without sharing their
// local-to-local derivative construction with the oracle.
for (let sample = 0; sample < 80; sample++) {
    const q0 = quaternionExp({ x: 0.7*Math.sin(sample), y: 0.9*Math.cos(sample*0.7), z: 0.6 });
    const q1 = quaternionExp({ x: -0.5, y: 0.8*Math.sin(sample*0.3), z: Math.cos(sample) });
    const rest = { x: 0.4*Math.sin(sample), y: -0.3, z: 0.2*Math.cos(sample) };
    const state = evaluateBendTwistLocalConstraintNormalized(q0, q1, rest, {});
    const relative = materialFrameDirectors(state.relative);
    const epsilon = 1e-6;
    for (let frame = 0; frame < 2; frame++) for (let axis = 0; axis < 3; axis++) {
        const direction = ['x', 'y', 'z'][axis];
        const rotation = { x: 0, y: 0, z: 0, [direction]: epsilon };
        const positive = multiplyQuaternions(frame === 0 ? q0 : q1, quaternionExp(rotation));
        rotation[direction] = -epsilon;
        const negative = multiplyQuaternions(frame === 0 ? q0 : q1, quaternionExp(rotation));
        const plus = evaluateBendTwistConstraint(frame === 0 ? positive : q0,
            frame === 1 ? positive : q1, rest).strain;
        const minus = evaluateBendTwistConstraint(frame === 0 ? negative : q0,
            frame === 1 ? negative : q1, rest).strain;
        for (let component = 0; component < 3; component++) {
            const key = ['x', 'y', 'z'][component];
            const offset = component*3, g = state.localGradient;
            const director = relative[['d1', 'd2', 'd3'][axis]];
            const analytic = frame === 0 ? -g[offset+axis] :
                g[offset]*director.x + g[offset+1]*director.y + g[offset+2]*director.z;
            assert.ok(Math.abs(analytic - (plus[key]-minus[key])/(2*epsilon)) < 1e-7,
                'material-frame bend/twist derivative must match finite differences');
        }
    }
}

function denseSolve(matrix, rhs) {
    const n = rhs.length;
    // Partial-pivot Gaussian elimination is independent of the banded
    // Cholesky implementation, and also catches accidentally dropped fill.
    for (let col = 0; col < n; col++) {
        let pivot = col;
        for (let row = col + 1; row < n; row++) {
            if (Math.abs(matrix[row][col]) > Math.abs(matrix[pivot][col])) pivot = row;
        }
        [matrix[col], matrix[pivot]] = [matrix[pivot], matrix[col]];
        [rhs[col], rhs[pivot]] = [rhs[pivot], rhs[col]];
        assert.ok(Math.abs(matrix[col][col]) > 1e-12);
        for (let row = col + 1; row < n; row++) {
            const scale = matrix[row][col] / matrix[col][col];
            for (let k = col + 1; k < n; k++) matrix[row][k] -= scale * matrix[col][k];
            rhs[row] -= scale * rhs[col];
        }
    }
    for (let row = n - 1; row >= 0; row--) {
        for (let k = row + 1; k < n; k++) rhs[row] -= matrix[row][k] * rhs[k];
        rhs[row] /= matrix[row][row];
    }
    return rhs;
}

{
    const world = new EndovascularPhysicsWorld();
    const body = world.createRod('dense-oracle', 13, 3, {
          adaptationCompliance: 1e-6
    });
    body.setActiveRange(2, 11);
    for (let i = 0; i < body.count; i++) {
        body.y[i] = Math.sin(i * 0.4);
        body.z[i] = Math.cos(i * 0.7);
    }
    body.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    body.setPinned(2, true);
    applyKirchhoffMaterialProfile(body, 'berenstein');
    const s = solveKirchhoffDirect(body, DT, true);
    const matrix = Array.from({ length: s.rowCount }, () => new Float64Array(s.rowCount));
    for (let row = 0; row < s.rowCount; row++) {
        const segment = 2 + Math.floor(row / 6), component = row % 6;
        const compliance = component >= 3 ? body.adaptationCompliance : segment === 2 ? DT*DT :
            [body.kirchhoffBendCompliance1, body.kirchhoffBendCompliance2,
                body.kirchhoffTwistCompliance][component][segment];
        matrix[row][row] = compliance / (DT*DT);
    }
    const degreeCapacity = 9;
    for (let dof = 2*6; dof < 12*6; dof++) {
        const offset = dof * degreeCapacity;
        for (let a = 0; a < s.degree[dof]; a++) {
            const row = s.rows[offset+a];
            if (a) assert.ok(row > s.rows[offset+a-1], 'Jacobian rows must remain sorted');
            for (let b = 0; b < s.degree[dof]; b++) {
                const col = s.rows[offset+b];
                assert.ok(Math.abs(row-col) <= 8, 'the complete system must fit its declared band');
                matrix[row][col] += s.gradients[offset+a] * s.weight[dof] * s.gradients[offset+b];
            }
        }
    }
    const node = 9, normal = [0.2, -0.8, 0.3];
    const rhs = new Float64Array(s.rowCount);
    for (let axis = 0; axis < 3; axis++) {
        const dof = node*6+axis, offset = dof*degreeCapacity;
        for (let k = 0; k < s.degree[dof]; k++) {
            rhs[s.rows[offset+k]] -= s.gradients[offset+k] * s.weight[dof] * normal[axis];
        }
    }
    const expectedLambda = denseSolve(matrix, rhs);
    const actual = kirchhoffDirectContactResponse(body, [node], [1], 1, normal, 1);
    for (let row = 0; row < s.rowCount; row++) {
        assert.ok(Math.abs(actual.lambda[row] - expectedLambda[row]) < 1e-8,
            `banded reaction differs from dense oracle at row ${row}`);
    }
    const factor = s.matrix.slice();
    solveKirchhoffDirect(body, DT, true, true);
    assert.equal(s.factorReuseCount, 1);
    assert.deepEqual(s.matrix, factor, 'an unchanged frame must reuse the same factor');
    const q = multiplyQuaternions({ x: body.orientationX[5], y: body.orientationY[5],
        z: body.orientationZ[5], w: body.orientationW[5] }, quaternionExp({ x: 0.004, y: 0, z: 0 }));
    body.orientationX[5] = q.x; body.orientationY[5] = q.y;
    body.orientationZ[5] = q.z; body.orientationW[5] = q.w;
    solveKirchhoffDirect(body, DT, true, true);
    assert.equal(s.factorizationCount, 2, 'rotation beyond the bound must rebuild the factor');
    body.setPinned(4, true);
    solveKirchhoffDirect(body, DT, true, true);
    assert.equal(s.factorizationCount, 3, 'a changed positional boundary must rebuild the factor');
}

const material = defineKirchhoffMaterialProfile({ id: 'reuse-cantilever',
    sampleEI1: () => 1e6, sampleGJ: () => 7e5 });
const cases = [false, true].map(reuseDirectLinearization => {
    const world = new EndovascularPhysicsWorld({ reuseDirectLinearization });
    const body = world.createRod('loaded-beam', 21, 4, { 
         mass: 0.01, linearDamping: 0.9, angularDamping: 0.9,
        foldLimitStrength: 0, sleepFrames: 1e6 });
    applyKirchhoffMaterialProfile(body, material);
    body.setPinned(0, true);
    body.setProximalOrientationControl(body.orientationX[0], body.orientationY[0],
        body.orientationZ[0], body.orientationW[0], 0, 0);
    return { world, body };
});
let reused = 0;
for (let step = 0; step < 360; step++) {
    for (const { world, body } of cases) {
        if (step === 180) applyKirchhoffMaterialProfile(body, material, { stiffnessScale: 2 });
        body.forceY[20] = step < 240 ? 10 : 0;
        world.stepFixed();
        assert.ok(body.kirchhoffScratch.direct.factorizationCount > 0,
            'every physical step must assemble a fresh material system');
        reused += body.kirchhoffScratch.direct.factorReuseCount;
    }
    for (const axis of ['x', 'y', 'z']) for (let i = 0; i < 21; i++) {
        assert.ok(Math.abs(cases[0].body[axis][i] - cases[1].body[axis][i]) < 0.002,
            'bounded factor reuse must preserve the loaded/unloaded trajectory');
    }
}
assert.ok(reused > 100, 'the optimization must avoid repeated factorization');
console.log(`Kirchhoff direct optimization: dense oracle, invalidation and trajectory passed; ${reused} factors reused`);
