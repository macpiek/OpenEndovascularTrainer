import { createKirchhoffLinearKernel, kirchhoffLinearKernelFor } from './kirchhoffLinearKernel.js';
import {
    evaluateBendTwistLocalConstraintNormalized,
    multiplyQuaternions,
    normalizeQuaternion,
    quaternionExp
} from './discreteKirchhoffRod.js';

// Place bend/twist BEFORE adaptation on each edge. The bend rows connect
// the previous and current frames; adaptation connects the current frame and
// its two positions. This ordering reduces half-bandwidth from 11 to 8,
// without changing any equation or eliminating any physical degree of freedom.
const BAND = 9;
const DEGREE = 9;

function scratchFor(body) {
    if (body.kirchhoffScratch.direct) return body.kirchhoffScratch.direct;
    const dofs = body.count * 6;
    const rows = body.segmentCount * 6;
    const kernel = createKirchhoffLinearKernel(body.count * 2048 + 4096);
    return body.kirchhoffScratch.direct = {
        kernel,
        blockImpulse: kernel.alloc(Float64Array, dofs),
        blockCorrection: kernel.alloc(Float64Array, dofs),
        blockLambda: kernel.alloc(Float64Array, rows),
        matrix: kernel.alloc(Float64Array, rows * BAND),
        rhs: kernel.alloc(Float64Array, rows),
        alpha: kernel.alloc(Float64Array, rows),
        strain: kernel.alloc(Float64Array, rows),
        lambda: kernel.alloc(Float64Array, rows),
        degreeCapacity: DEGREE,
        weight: kernel.alloc(Float64Array, dofs),
        degree: kernel.alloc(Uint8Array, dofs),
        rows: kernel.alloc(Int32Array, dofs * DEGREE),
        gradients: kernel.alloc(Float64Array, dofs * DEGREE),
        correction: kernel.alloc(Float64Array, dofs),
        rotations: kernel.alloc(Float64Array, body.segmentCount * 9),
        linearizationFrames: kernel.alloc(Float64Array, body.segmentCount * 4),
        redundantAxes: kernel.alloc(Int8Array, body.segmentCount),
        factorAge: Infinity, factorizationCount: 0, factorReuseCount: 0,
        q0: {}, q1: {}, rest: {}, bend: {},
        relativeRotation: new Float64Array(9),
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

// Column-major material-to-world matrix. Compute the quaternion norm once,
// instead of normalizing and rotating each of the three basis vectors.
function writeRotation(q, out, offset = 0) {
    const scale = 2 / Math.max(1e-24, q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    const xx = q.x*q.x*scale, yy = q.y*q.y*scale, zz = q.z*q.z*scale;
    const xy = q.x*q.y*scale, xz = q.x*q.z*scale, yz = q.y*q.z*scale;
    const wx = q.w*q.x*scale, wy = q.w*q.y*scale, wz = q.w*q.z*scale;
    out[offset] = 1-yy-zz; out[offset+1] = xy+wz; out[offset+2] = xz-wy;
    out[offset+3] = xy-wz; out[offset+4] = 1-xx-zz; out[offset+5] = yz+wx;
    out[offset+6] = xz+wy; out[offset+7] = yz-wx; out[offset+8] = 1-xx-yy;
}

function addGradient(scratch, dof, row, gradient) {
    if (gradient === 0 || scratch.weight[dof] === 0) return;
    const slot = dof * DEGREE + scratch.degree[dof]++;
    scratch.rows[slot] = row;
    scratch.gradients[slot] = gradient;
}

function equation(scratch, row, strain, compliance, lambda, inverseDtSquared) {
    const alpha = compliance * inverseDtSquared;
    if (scratch.assembling) scratch.matrix[row * BAND] = alpha;
    scratch.rhs[row] = -strain - alpha * lambda;
    scratch.alpha[row] = alpha;
    scratch.strain[row] = strain;
    scratch.lambda[row] = lambda;
}

// SPD banded Cholesky. Rows with no free generalized coordinate can remain
// at a prescribed boundary. A relative 1e-12 pivot floor handles these zero
// rows and roundoff; the redundant axial equation is eliminated at assembly.
function factorAndSolve(matrix, rhs, count) {
    const kernel = kirchhoffLinearKernelFor(matrix);
    kernel.factorBand(matrix.byteOffset, count, BAND, -1, 1);
    kernel.solveBand(matrix.byteOffset, rhs.byteOffset, count, BAND);
}

function solveFactored(matrix, rhs, count) {
    const kernel = kirchhoffLinearKernelFor(matrix);
    if (rhs.buffer === matrix.buffer) kernel.solveBand(matrix.byteOffset, rhs.byteOffset, count, BAND);
    else {
        const temp = kernel.externalRhs ??= kernel.alloc(Float64Array, rhs.length);
        temp.set(rhs.subarray(0, count));
        kernel.solveBand(matrix.byteOffset, temp.byteOffset, count, BAND);
        rhs.set(temp.subarray(0, count));
    }
}

// Modified Newton within ONE fixed step: reuse a factor at most three times,
// only while every frame remains within 0.001 rad of its linearization.
// The world invalidates it before each step, so material/topology changes,
// integration and transport always receive a new Jacobian. Standalone calls
// keep exact relinearization by default. The current nonlinear strain and
// XPBD multipliers are evaluated on every constitutive update.
function canReuseFactor(body, scratch, start, end, controlled, dt) {
    if (scratch.factorAge >= 3 || scratch.start !== start || scratch.end !== end ||
        scratch.controlled !== controlled || scratch.dt !== dt) return false;
    const frames = scratch.linearizationFrames;
    for (let node = start; node <= end; node++) {
        const dof = node * 6;
        if (scratch.weight[dof] !== body.inverseMass[node]) return false;
        if (node === end) continue;
        if (node !== controlled && (scratch.weight[dof + 3] !== body.inverseInertia1[node] ||
            scratch.weight[dof + 4] !== body.inverseInertia2[node] ||
            scratch.weight[dof + 5] !== body.inverseInertia3[node])) return false;
        const offset = node * 4;
        const dot = frames[offset] * body.orientationX[node] +
            frames[offset + 1] * body.orientationY[node] +
            frames[offset + 2] * body.orientationZ[node] +
            frames[offset + 3] * body.orientationW[node];
        if (1 - Math.abs(dot) > 1.25e-7) return false;
    }
    return true;
}

/**
 * One Newton/XPBD update of the complete Kirchhoff constitutive system:
 * (J W J^T + compliance / dt^2) deltaLambda = -C - alpha lambda.
 *
 * Solves inextensibility, no shear, bending and twist together. Positions and
 * material frames receive the same update. Shared multipliers also serve the
 * length/adaptation preconditioners; contact stays in the world's outer loop.
 * This changes convergence, not the manufactured shape or EI/GJ.
 */
export function solveKirchhoffDirect(body, dt = 1 / 120, linearizeOnly = false, allowReuse = false) {
    return solveOrAssemble(body, dt, linearizeOnly, allowReuse, false);
}

/** Fresh constitutive assembly without factorization or a Newton update.
 * J is stored by generalized coordinate (degree/rows/gradients); translations
 * are world coordinates and rotations are right-sided material coordinates.
 * Arrays are borrowed until the next direct/coupled assembly of this body.
 * Prescribed frames and redundant boundary rows follow the solo solver.
 */
export function assembleKirchhoffDirect(body, dt = 1 / 120) {
    return solveOrAssemble(body, dt, true, false, true);
}

function solveOrAssemble(body, dt, linearizeOnly, allowReuse, assemblyOnly) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('A positive finite timestep is required');
    const start = Math.max(0, body.activeStart);
    const end = Math.min(body.segmentCount, body.activeEnd);
    if (body.sleeping || end <= start) return;
    const scratch = scratchFor(body);
    const { matrix, rhs, weight, degree, rotations } = scratch;
    const count = (end - start) * 6;
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

    const reuse = allowReuse && canReuseFactor(body, scratch, start, end, controlled, dt);
    scratch.assembling = !reuse;
    if (reuse) {
        scratch.factorAge++;
        scratch.factorReuseCount++;
        if (linearizeOnly) return scratch;
    } else {
        scratch.factorAge = 0;
        if (!assemblyOnly) scratch.factorizationCount++;
        scratch.controlled = controlled;
        scratch.dt = dt;
        matrix.fill(0, 0, count * BAND);
        degree.fill(0, start * 6, (end + 1) * 6);
    }

    for (let node = start; node <= end; node++) {
        const dof = node * 6;
        weight[dof] = weight[dof + 1] = weight[dof + 2] = body.inverseMass[node];
        if (node === end) continue;
        weight[dof + 3] = node === controlled ? 0 : body.inverseInertia1[node];
        weight[dof + 4] = node === controlled ? 0 : body.inverseInertia2[node];
        weight[dof + 5] = node === controlled ? 0 : body.inverseInertia3[node];
        const frame = readFrame(body, node, scratch.q1);
        if (!reuse) {
            const offset = node * 4;
            scratch.linearizationFrames[offset] = frame.x;
            scratch.linearizationFrames[offset + 1] = frame.y;
            scratch.linearizationFrames[offset + 2] = frame.z;
            scratch.linearizationFrames[offset + 3] = frame.w;
        }
        writeRotation(frame, rotations, node * 9);
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
        let redundantAxis = reuse ? scratch.redundantAxes[segment] : -1;
        if (!reuse && body.inverseMass[segment] === 0 && body.inverseMass[segment + 1] === 0) {
            redundantAxis = 0;
            for (let axis = 1; axis < 3; axis++) {
                if (Math.abs(rotations[rotation + 6 + axis]) >
                    Math.abs(rotations[rotation + 6 + redundantAxis])) redundantAxis = axis;
            }
        }
        if (!reuse) scratch.redundantAxes[segment] = redundantAxis;
        if (segment === start) {
            for (let axis = 0; axis < 3; axis++) {
                equation(scratch, row + axis, 0, dt * dt, 0, inverseDtSquared);
            }
        } else {
            readFrame(body, segment - 1, scratch.q0);
            readFrame(body, segment, scratch.q1);
            scratch.rest.x = body.restRotation1[segment];
            scratch.rest.y = body.restRotation2[segment];
            scratch.rest.z = body.restRotation3[segment];
            normalizeQuaternion(scratch.q0, scratch.q0);
            normalizeQuaternion(scratch.q1, scratch.q1);
            const state = evaluateBendTwistLocalConstraintNormalized(
                scratch.q0, scratch.q1, scratch.rest, scratch.bend, !reuse);
            if (!reuse) writeRotation(state.relative, scratch.relativeRotation);
            for (let axis = 0; axis < 3; axis++) {
                const compliance = axis === 0 ? body.kirchhoffBendCompliance1
                    : axis === 1 ? body.kirchhoffBendCompliance2 : body.kirchhoffTwistCompliance;
                const lambdas = axis === 0 ? body.bendTwistLambda1
                    : axis === 1 ? body.bendTwistLambda2 : body.bendTwistLambda3;
                const strain = axis === 0 ? state.strain.x : axis === 1 ? state.strain.y : state.strain.z;
                equation(scratch, row + axis, strain, compliance[segment], lambdas[segment], inverseDtSquared);
                if (reuse) continue;
                const gradient = state.localGradient;
                const relative = scratch.relativeRotation;
                const g0 = gradient[axis * 3], g1 = gradient[axis * 3 + 1];
                const g2 = gradient[axis * 3 + 2];
                for (let local = 0; local < 3; local++) {
                    // G0 = -J^-1 Rrest^T, G1 = -G0 R0^T R1.
                    // Stay in the local material frames throughout.
                    const previous = -gradient[axis * 3 + local];
                    const r = local * 3;
                    const next = g0 * relative[r] + g1 * relative[r + 1] + g2 * relative[r + 2];
                    addGradient(scratch, dof - 3 + local, row + axis, previous);
                    addGradient(scratch, dof + 3 + local, row + axis, next);
                }
            }
        }
        for (let axis = 0; axis < 3; axis++) {
            const positions = axis === 0 ? body.x : axis === 1 ? body.y : body.z;
            const lambdas = axis === 0 ? body.adaptationLambdaX
                : axis === 1 ? body.adaptationLambdaY : body.adaptationLambdaZ;
            if (axis === redundantAxis) {
                if (!assemblyOnly) lambdas[segment] = 0;
                equation(scratch, row + 3 + axis, 0, dt * dt, 0, inverseDtSquared);
                continue;
            }
            const strain = positions[segment + 1] - positions[segment] -
                length * rotations[rotation + 6 + axis];
            equation(scratch, row + 3 + axis, strain, body.adaptationCompliance,
                lambdas[segment], inverseDtSquared);
            if (reuse) continue;
            addGradient(scratch, dof + axis, row + 3 + axis, -1);
            addGradient(scratch, dof + 6 + axis, row + 3 + axis, 1);
            // dC/dtheta_local = [ L*d2, -L*d1, 0 ].
            addGradient(scratch, dof + 3, row + 3 + axis, length * rotations[rotation + 3 + axis]);
            addGradient(scratch, dof + 4, row + 3 + axis, -length * rotations[rotation + axis]);
        }

    }

    scratch.start = start;
    scratch.end = end;
    scratch.rowCount = count;
    if (assemblyOnly) {
        // An unfactored matrix must never be reused by the solo mobility path.
        scratch.factorAge = Infinity;
        return scratch;
    }

    if (reuse) {
        solveFactored(matrix, rhs, count);
    } else {
        // Assemble J W J^T by generalized coordinate, using at most nine
        // nonzero Jacobian entries per coordinate. No dense rod-sized matrix.
        for (let dof = start * 6; dof < (end + 1) * 6; dof++) {
            const offset = dof * DEGREE;
            for (let a = 0; a < degree[dof]; a++) {
                const rowA = scratch.rows[offset + a];
                const gradient = scratch.gradients[offset + a] * weight[dof];
                for (let b = 0; b <= a; b++) {
                    const rowB = scratch.rows[offset + b];
                    // Rows are appended in ascending order by edge and component.
                    matrix[rowA * BAND + rowA - rowB] += gradient * scratch.gradients[offset + b];
                }
            }
        }
        factorAndSolve(matrix, rhs, count);
    }
    scratch.start = start;
    scratch.end = end;
    scratch.rowCount = count;
    if (linearizeOnly) return scratch;
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
    applyKirchhoffDirectCorrection(body, scratch.correction, rhs, scale);
}

export function applyKirchhoffDirectCorrection(body, correction, lambda, scale, contact = false) {
    const scratch = scratchFor(body);
    const { start, end } = scratch;
    for (let node = start; node <= end; node++) {
        const offset = node * 6;
        body.x[node] += scale * correction[offset];
        body.y[node] += scale * correction[offset + 1];
        body.z[node] += scale * correction[offset + 2];
        if (contact) {
            body.toolProjectionX[node] += scale * correction[offset];
            body.toolProjectionY[node] += scale * correction[offset + 1];
            body.toolProjectionZ[node] += scale * correction[offset + 2];
        }
        if (node === end) continue;
        scratch.rotation.x = scale * correction[offset + 3];
        scratch.rotation.y = scale * correction[offset + 4];
        scratch.rotation.z = scale * correction[offset + 5];
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
        body.adaptationLambdaX[node] += scale * lambda[row + 3];
        body.adaptationLambdaY[node] += scale * lambda[row + 4];
        body.adaptationLambdaZ[node] += scale * lambda[row + 5];
        if (node > start) {
            body.bendTwistLambda1[node] += scale * lambda[row];
            body.bendTwistLambda2[node] += scale * lambda[row + 1];
            body.bendTwistLambda3[node] += scale * lambda[row + 2];
        }
    }
}

/** Apply the rod's constrained mobility to a sparse translational impulse.
 * Wc = W - W J^T (J W J^T + alpha)^-1 J W.
 * Its material multiplier increment is retained, so a contact reaction
 * changes positions AND bending/twist throughout the rod consistently.
 * Call solveKirchhoffDirect(body, dt, true) once before a contact sweep.
 */
export function kirchhoffDirectContactResponse(body, nodes, weights, count, normal, sign, out = {}) {
    const scratch = scratchFor(body);
    const { matrix, weight, degree, start, end, rowCount } = scratch;
    out.correction ??= new Float64Array(body.count * 6);
    out.lambda ??= new Float64Array(body.segmentCount * 6);
    const correction = out.correction;
    const rhs = out.lambda;
    correction.fill(0);
    rhs.fill(0, 0, rowCount);
    for (let i = 0; i < count; i++) {
        const node = nodes[i];
        if (node < start || node > end) continue;
        for (let axis = 0; axis < 3; axis++) {
            const dof = node * 6 + axis;
            const impulse = sign * weights[i] * normal[axis] * weight[dof];
            correction[dof] += impulse;
            for (let k = 0; k < degree[dof]; k++) {
                const entry = dof * DEGREE + k;
                rhs[scratch.rows[entry]] -= scratch.gradients[entry] * impulse;
            }
        }
    }
    solveFactored(matrix, rhs, rowCount);
    out.maximumPosition = 0;
    out.maximumAngle = 0;
    for (let node = start; node <= end; node++) {
        for (let axis = 0; axis < (node === end ? 3 : 6); axis++) {
            const dof = node * 6 + axis;
            let value = 0;
            for (let k = 0; k < degree[dof]; k++) {
                const entry = dof * DEGREE + k;
                value += scratch.gradients[entry] * rhs[scratch.rows[entry]];
            }
            correction[dof] += weight[dof] * value;
        }
        const d = node * 6;
        out.maximumPosition = Math.max(out.maximumPosition,
            Math.hypot(correction[d], correction[d + 1], correction[d + 2]));
        if (node < end) out.maximumAngle = Math.max(out.maximumAngle,
            Math.hypot(correction[d + 3], correction[d + 4], correction[d + 5]));
    }
    out.mobility = 0;
    for (let i = 0; i < count; i++) {
        const d = nodes[i] * 6;
        out.mobility += sign * weights[i] * (normal[0] * correction[d] +
            normal[1] * correction[d + 1] + normal[2] * correction[d + 2]);
    }
    out.mobility = Math.max(0, out.mobility);
    return out;
}

/** Constrained mobility for a complete, accumulated generalized impulse.
 * Same operator and material multiplier update as the sparse contact response,
 * with one pair of triangular solves for the entire contact block.
 */
export function kirchhoffDirectBlockResponse(body, impulse, out = {}) {
    const s = scratchFor(body);
    s.blockImpulse.set(impulse);
    s.blockCorrection.fill(0);
    s.kernel.response(s.matrix.byteOffset, s.weight.byteOffset, s.degree.byteOffset,
        s.rows.byteOffset, s.gradients.byteOffset, s.blockImpulse.byteOffset,
        s.blockCorrection.byteOffset, s.blockLambda.byteOffset, s.start, s.end, s.rowCount);
    out.correction = s.blockCorrection;
    out.lambda = s.blockLambda;
    return out;
}
