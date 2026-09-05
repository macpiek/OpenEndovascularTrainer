import {
    evaluateBendTwistConstraint,
    materialFrameDirectors,
    multiplyQuaternions,
    normalizeQuaternion,
    quaternionExp
} from './discreteKirchhoffRod.js';

// Interleave three adaptation and three bend/twist equations per edge.
// Adjacent equations share only neighboring nodes/frames, so J W J^T has
// half-bandwidth 11 regardless of rod length. Factorization is O(n).
const BAND = 12;
const DEGREE = 9;

function scratchFor(body) {
    if (body.kirchhoffScratch.direct) return body.kirchhoffScratch.direct;
    const dofs = body.count * 6;
    const rows = body.segmentCount * 6;
    return body.kirchhoffScratch.direct = {
        matrix: new Float64Array(rows * BAND),
        rhs: new Float64Array(rows),
        weight: new Float64Array(dofs),
        degree: new Uint8Array(dofs),
        rows: new Int32Array(dofs * DEGREE),
        gradients: new Float64Array(dofs * DEGREE),
        correction: new Float64Array(dofs),
        rotations: new Float64Array(body.segmentCount * 9),
        q0: {}, q1: {}, rest: {}, bend: {}, directors: {},
        rotation: {}, increment: {}, result: {}
    };
}

function readFrame(body, segment, out) {
    out.x = body.orientationX[segment];
    out.y = body.orientationY[segment];
    out.z = body.orientationZ[segment];
    out.w = body.orientationW[segment];
    return out;
}

function addGradient(scratch, dof, row, gradient) {
    if (gradient === 0 || scratch.weight[dof] === 0) return;
    const slot = dof * DEGREE + scratch.degree[dof]++;
    scratch.rows[slot] = row;
    scratch.gradients[slot] = gradient;
}

function equation(scratch, row, strain, compliance, lambda, inverseDtSquared) {
    const alpha = compliance * inverseDtSquared;
    scratch.matrix[row * BAND] = alpha;
    scratch.rhs[row] = -strain - alpha * lambda;
}

// SPD banded Cholesky. Rows with no free generalized coordinate can remain
// at a prescribed boundary. A relative 1e-12 pivot floor handles these zero
// rows and roundoff; the redundant axial equation is eliminated at assembly.
function factorAndSolve(matrix, rhs, count) {
    for (let row = 0; row < count; row++) {
        const base = row * BAND;
        const first = Math.max(0, row - BAND + 1);
        const diagonalScale = Math.max(1, matrix[base]);
        for (let col = first; col <= row; col++) {
            let value = matrix[base + row - col];
            const colBase = col * BAND;
            for (let k = first; k < col; k++) {
                value -= matrix[base + row - k] * matrix[colBase + col - k];
            }
            if (col === row) {
                matrix[base] = Math.sqrt(Math.max(value, diagonalScale * 1e-12));
            } else {
                matrix[base + row - col] = value / matrix[colBase];
            }
        }
        for (let col = first; col < row; col++) {
            rhs[row] -= matrix[base + row - col] * rhs[col];
        }
        rhs[row] /= matrix[base];
    }
    for (let row = count - 1; row >= 0; row--) {
        const end = Math.min(count, row + BAND);
        for (let col = row + 1; col < end; col++) {
            rhs[row] -= matrix[col * BAND + col - row] * rhs[col];
        }
        rhs[row] /= matrix[row * BAND];
    }
}

/**
 * One Newton/XPBD update of the complete Kirchhoff constitutive system:
 * (J W J^T + compliance / dt^2) deltaLambda = -C - alpha lambda.
 *
 * Solves inextensibility, no shear, bending and twist together. Positions and
 * material frames receive the SAME update and multipliers are shared with the
 * existing local/closure solvers. Contact stays in the world's outer loop.
 * This changes convergence, not the manufactured shape or EI/GJ.
 */
export function solveKirchhoffDirect(body, dt = 1 / 120) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('A positive finite timestep is required');
    const start = Math.max(0, body.activeStart);
    const end = Math.min(body.segmentCount, body.activeEnd);
    if (body.sleeping || end <= start) return;
    const scratch = scratchFor(body);
    const { matrix, rhs, weight, degree, rotations } = scratch;
    const count = (end - start) * 6;
    matrix.fill(0, 0, count * BAND);
    degree.fill(0, start * 6, (end + 1) * 6);
    scratch.correction.fill(0, start * 6, (end + 1) * 6);
    const inverseDtSquared = 1 / (dt * dt);
    const controlled = body.orientationControlCompliance === 0
        ? body.orientationControlSegment : -1;
    if (controlled >= start && controlled < end) {
        body.orientationX[controlled] = body.orientationControlX;
        body.orientationY[controlled] = body.orientationControlY;
        body.orientationZ[controlled] = body.orientationControlZ;
        body.orientationW[controlled] = body.orientationControlW;
    }

    for (let node = start; node <= end; node++) {
        const dof = node * 6;
        weight[dof] = weight[dof + 1] = weight[dof + 2] = body.inverseMass[node];
        if (node === end) continue;
        weight[dof + 3] = node === controlled ? 0 : body.inverseInertia1[node];
        weight[dof + 4] = node === controlled ? 0 : body.inverseInertia2[node];
        weight[dof + 5] = node === controlled ? 0 : body.inverseInertia3[node];
        const frame = readFrame(body, node, scratch.q1);
        const directors = materialFrameDirectors(frame, scratch.directors);
        const offset = node * 9;
        // Column-major material-to-world rotation matrix.
        for (let column = 0; column < 3; column++) {
            const director = column === 0 ? directors.d1 : column === 1 ? directors.d2 : directors.d3;
            rotations[offset + column * 3] = director.x;
            rotations[offset + column * 3 + 1] = director.y;
            rotations[offset + column * 3 + 2] = director.z;
        }
    }

    for (let segment = start; segment < end; segment++) {
        const row = (segment - start) * 6;
        const dof = segment * 6;
        const rotation = segment * 9;
        const length = body.restLength[segment];
        // With two prescribed endpoints the axial equation is redundant:
        // a unit director has only two independent tangent coordinates.
        // Remove its dominant component, retaining the two transverse
        // equations that align the frame. This also avoids trying to correct
        // float32 roundoff in a boundary length with an infinite axial load.
        let redundantAxis = -1;
        if (body.inverseMass[segment] === 0 && body.inverseMass[segment + 1] === 0) {
            redundantAxis = 0;
            for (let axis = 1; axis < 3; axis++) {
                if (Math.abs(rotations[rotation + 6 + axis]) >
                    Math.abs(rotations[rotation + 6 + redundantAxis])) redundantAxis = axis;
            }
        }
        for (let axis = 0; axis < 3; axis++) {
            const positions = axis === 0 ? body.x : axis === 1 ? body.y : body.z;
            const lambdas = axis === 0 ? body.adaptationLambdaX
                : axis === 1 ? body.adaptationLambdaY : body.adaptationLambdaZ;
            if (axis === redundantAxis) {
                lambdas[segment] = 0;
                equation(scratch, row + axis, 0, dt * dt, 0, inverseDtSquared);
                continue;
            }
            const strain = positions[segment + 1] - positions[segment] -
                length * rotations[rotation + 6 + axis];
            equation(scratch, row + axis, strain, body.adaptationCompliance,
                lambdas[segment], inverseDtSquared);
            addGradient(scratch, dof + axis, row + axis, -1);
            addGradient(scratch, dof + 6 + axis, row + axis, 1);
            // dC/dtheta_local = [ L*d2, -L*d1, 0 ].
            addGradient(scratch, dof + 3, row + axis, length * rotations[rotation + 3 + axis]);
            addGradient(scratch, dof + 4, row + axis, -length * rotations[rotation + axis]);
        }
        if (segment === start) {
            for (let axis = 0; axis < 3; axis++) {
                equation(scratch, row + 3 + axis, 0, dt * dt, 0, inverseDtSquared);
            }
            continue;
        }
        readFrame(body, segment - 1, scratch.q0);
        readFrame(body, segment, scratch.q1);
        scratch.rest.x = body.restRotation1[segment];
        scratch.rest.y = body.restRotation2[segment];
        scratch.rest.z = body.restRotation3[segment];
        const state = evaluateBendTwistConstraint(scratch.q0, scratch.q1, scratch.rest, scratch.bend);
        for (let axis = 0; axis < 3; axis++) {
            const compliance = axis === 0 ? body.kirchhoffBendCompliance1
                : axis === 1 ? body.kirchhoffBendCompliance2 : body.kirchhoffTwistCompliance;
            const lambdas = axis === 0 ? body.bendTwistLambda1
                : axis === 1 ? body.bendTwistLambda2 : body.bendTwistLambda3;
            const strain = axis === 0 ? state.strain.x : axis === 1 ? state.strain.y : state.strain.z;
            equation(scratch, row + 3 + axis, strain, compliance[segment], lambdas[segment], inverseDtSquared);
            for (let local = 0; local < 3; local++) {
                let previous = 0;
                let next = 0;
                for (let world = 0; world < 3; world++) {
                    previous += state.gradient0[axis * 3 + world] * rotations[rotation - 9 + local * 3 + world];
                    next += state.gradient1[axis * 3 + world] * rotations[rotation + local * 3 + world];
                }
                addGradient(scratch, dof - 3 + local, row + 3 + axis, previous);
                addGradient(scratch, dof + 3 + local, row + 3 + axis, next);
            }
        }
    }

    // Assemble J W J^T by generalized coordinate, using at most nine
    // nonzero Jacobian entries per coordinate. No dense rod-sized matrix.
    for (let dof = start * 6; dof < (end + 1) * 6; dof++) {
        const offset = dof * DEGREE;
        for (let a = 0; a < degree[dof]; a++) {
            const rowA = scratch.rows[offset + a];
            const gradient = scratch.gradients[offset + a] * weight[dof];
            for (let b = 0; b <= a; b++) {
                const rowB = scratch.rows[offset + b];
                const row = Math.max(rowA, rowB);
                const col = Math.min(rowA, rowB);
                matrix[row * BAND + row - col] += gradient * scratch.gradients[offset + b];
            }
        }
    }
    factorAndSolve(matrix, rhs, count);
    let scale = 1;
    for (let dof = start * 6; dof < (end + 1) * 6; dof++) {
        const offset = dof * DEGREE;
        let value = 0;
        for (let slot = 0; slot < degree[dof]; slot++) {
            value += scratch.gradients[offset + slot] * rhs[scratch.rows[offset + slot]];
        }
        scratch.correction[dof] = value * weight[dof];
    }
    // Damped Newton trust region limits a large nonlinear correction, not
    // velocity or force. Scale deltaLambda and all coordinates together so
    // a contact update cannot receive mismatched position/frame impulses.
    for (let node = start; node <= end; node++) {
        const offset = node * 6;
        const correction = scratch.correction;
        const angular = node === end ? 0 : Math.hypot(
            correction[offset + 3], correction[offset + 4], correction[offset + 5]);
        const spatial = Math.hypot(correction[offset], correction[offset + 1], correction[offset + 2]);
        scale = Math.min(scale, 0.25 / Math.max(0.25, angular),
            (body.segmentLength * 0.25) / Math.max(body.segmentLength * 0.25, spatial));
    }
    for (let node = start; node <= end; node++) {
        const offset = node * 6;
        body.x[node] += scale * scratch.correction[offset];
        body.y[node] += scale * scratch.correction[offset + 1];
        body.z[node] += scale * scratch.correction[offset + 2];
        if (node === end) continue;
        scratch.rotation.x = scale * scratch.correction[offset + 3];
        scratch.rotation.y = scale * scratch.correction[offset + 4];
        scratch.rotation.z = scale * scratch.correction[offset + 5];
        quaternionExp(scratch.rotation, scratch.increment);
        readFrame(body, node, scratch.q0);
        // Right multiplication applies the local angular correction.
        multiplyQuaternions(scratch.q0, scratch.increment, scratch.result);
        normalizeQuaternion(scratch.result, scratch.result);
        body.orientationX[node] = scratch.result.x;
        body.orientationY[node] = scratch.result.y;
        body.orientationZ[node] = scratch.result.z;
        body.orientationW[node] = scratch.result.w;
        const row = (node - start) * 6;
        body.adaptationLambdaX[node] += scale * rhs[row];
        body.adaptationLambdaY[node] += scale * rhs[row + 1];
        body.adaptationLambdaZ[node] += scale * rhs[row + 2];
        if (node > start) {
            body.bendTwistLambda1[node] += scale * rhs[row + 3];
            body.bendTwistLambda2[node] += scale * rhs[row + 4];
            body.bendTwistLambda3[node] += scale * rhs[row + 5];
        }
    }
}
