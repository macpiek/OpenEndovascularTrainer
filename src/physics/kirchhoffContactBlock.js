import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';
import { kirchhoffDirectBlockResponse, applyKirchhoffDirectCorrection } from './kirchhoffDirectSolver.js';

function stencil(record, side, visit) {
    const nodes = side === 0 ? record._innerNodeIndices : record._outerNodeIndices;
    const weights = side === 0 ? record._innerNodeWeights : record._outerNodeWeights;
    const count = nodes ? (side === 0 ? record._innerNodeCount : record._outerNodeCount) : 2;
    const fallback = side === 0 ? record.innerWeights : record.outerWeights;
    const segment = side === 0 ? record._innerSegmentIndex : record._outerSegmentIndex;
    for (let i = 0; i < count; i++) visit(nodes ? nodes[i] : segment + i, weights ? weights[i] : fallback[i]);
}

/** Solve all unilateral normal contacts on frozen geometry, with reciprocal
 * responses of both bodies. Local mobility is the inexpensive contact block in
 * the world's alternating full-rod/contact solve. Constrained mobility applies
 * each complete rod's material Schur complement without a dense inverse.
 * Regularization damps only this increment; it is not physical compliance.
 */
export function solveKirchhoffContactBlock(constraint, tolerance = 1e-5, regularization = 0) {
    const direct = constraint._contactBlockDirect !== false;
    const records = constraint.kirchhoffContacts;
    const count = records.length;
    const bodies = [constraint.innerBody, constraint.outerBody];
    const state = constraint._contactBlock ??= { capacity: 0,
        impulse: bodies.map(b => new Float64Array(b.count * 6)), response: [{}, {}] };
    if (state.capacity < count) {
        state.capacity = Math.max(count, state.capacity * 2, 16);
        for (const key of ['increment', 'residual', 'direction', 'product', 'preconditioned', 'diagonal', 'projected', 'projectedProduct']) {
            state[key] = new Float64Array(state.capacity);
        }
        state.free = new Uint8Array(state.capacity);
        state.stencilNodes = new Int32Array(state.capacity * 8);
        state.stencilWeights = new Float64Array(state.capacity * 8);
        state.stencilCounts = new Uint8Array(state.capacity * 2);
    }
    const { increment, residual, direction, product, preconditioned, diagonal, free } = state;
    increment?.fill(0, 0, count);
    state.iterations = 0;
    if (!count) return state;
    for (let i = 0; i < count; i++) {
        const r = records[i];
        for (let side = 0; side < 2; side++) {
            let n = 0;
            stencil(r, side, (node, weight) => {
                state.stencilNodes[i * 8 + side * 4 + n] = node;
                state.stencilWeights[i * 8 + side * 4 + n++] = (side === 0 ? -1 : 1) * weight;
            });
            state.stencilCounts[i * 2 + side] = n;
        }
        residual[i] = -r.gap - r._normalAlpha * r.manifoldContact.normalLambda;
        let mobility = r._normalAlpha;
        for (let side = 0; side < 2; side++) stencil(r, side, (node, weight) => {
            mobility += weight * weight * bodies[side].inverseMass[node];
        });
        diagonal[i] = Math.max(1e-12, mobility);
        // Previous activity is only a working-set hint, never a retained force.
        // Complementarity checks below can release it or activate omitted rows.
        free[i] = Number(r.manifoldContact.normalLambda > 1e-10 ||
            (r.manifoldContact._blockActive ?? (residual[i] > tolerance)));
    }
    const nodes = state.stencilNodes, weights = state.stencilWeights, sizes = state.stencilCounts;
    // The local contact Gram matrix is a banded preconditioner for the full
    // rod-constrained Schur operator. Shared cubic neighbours are coupled here;
    // diagonal scaling alone leaves almost redundant quadrature rows untreated.
    const byNode = state.byNode ??= bodies.map(b => Array.from({length: b.count}, () => []));
    let band = 1;
    for (let side = 0; side < 2; side++) {
        for (const entries of byNode[side]) entries.length = 0;
        for (let i = 0; i < count; i++) {
            for (let k = 0; k < sizes[i * 2 + side]; k++) {
                const offset = i * 8 + side * 4 + k;
                const entries = byNode[side][nodes[offset]];
                if (entries.length) band = Math.max(band, i - entries[0] + 1);
                entries.push(i, weights[offset]);
            }
        }
    }
    if (!state.preconditioner || state.preconditioner.length < count * band || state.kernelRhs.length < count) {
        const capacity = Math.max(16, count * 2), matrixSize = capacity * band;
        state.kernel = createKirchhoffLinearKernel(matrixSize * 16 + capacity * 40 + 128);
        state.preconditioner = state.kernel.alloc(Float64Array, matrixSize);
        state.kernelRhs = state.kernel.alloc(Float64Array, capacity);
        state.kernelFree = state.kernel.alloc(Uint8Array, capacity);
        state.preconditionerOriginal = state.kernel.alloc(Float64Array, matrixSize);
        state.kernelInput = state.kernel.alloc(Float64Array, capacity);
        state.kernelOutput = state.kernel.alloc(Float64Array, capacity);
        state.kernelDiagonal = state.kernel.alloc(Float64Array, capacity);
    }
    state.band = band;
    const matrix = state.preconditioner;
    matrix.fill(0, 0, count * band);
    for (let i = 0; i < count; i++) matrix[i * band] = records[i]._normalAlpha + diagonal[i] * 1e-3;
    for (let side = 0; side < 2; side++) {
        for (let node = 0; node < bodies[side].count; node++) {
            const entries = byNode[side][node], mass = bodies[side].inverseMass[node];
            for (let a = 0; a < entries.length; a += 2) {
                const row = entries[a], normal = records[row].normal;
                for (let b = 0; b <= a; b += 2) {
                    const col = entries[b], other = records[col].normal;
                    matrix[row * band + row - col] += mass * entries[a + 1] * entries[b + 1] *
                        (normal[0] * other[0] + normal[1] * other[1] + normal[2] * other[2]);
                }
            }
        }
    }
    const original = state.preconditionerOriginal.subarray(0, count * band);
    original.set(matrix.subarray(0, count * band));
    state.kernelDiagonal.set(diagonal.subarray(0, count));
    const factor = () => {
        matrix.set(original);
        state.kernelFree.set(free.subarray(0, count));
        state.kernel.factorBand(matrix.byteOffset, count, band, state.kernelFree.byteOffset, 0);
    };
    const precondition = () => {
        const temp = state.kernelRhs;
        for (let i = 0; i < count; i++) temp[i] = free[i] ? residual[i] : 0;
        state.kernel.solveBand(matrix.byteOffset, temp.byteOffset, count, band);
        for (let i = 0; i < count; i++) preconditioned[i] = free[i] ? temp[i] : 0;
    };
    const multiply = (input, output, buildResponse = false) => {
        if (!direct && !buildResponse) {
            state.kernelInput.set(input.subarray(0, count));
            state.kernel.multiplyBand(original.byteOffset, state.kernelInput.byteOffset,
                state.kernelOutput.byteOffset, count, band, state.kernelDiagonal.byteOffset,
                regularization - 1e-3);
            output.set(state.kernelOutput.subarray(0, count));
            return;
        }
        for (const impulse of state.impulse) impulse.fill(0);
        for (let i = 0; i < count; i++) {
            const normal = records[i].normal;
            for (let side = 0; side < 2; side++) {
                const impulse = state.impulse[side];
                for (let k = 0; k < sizes[i * 2 + side]; k++) {
                    const offset = i * 8 + side * 4 + k, dof = nodes[offset] * 6;
                    const scalar = input[i] * weights[offset];
                    impulse[dof] += scalar * normal[0];
                    impulse[dof + 1] += scalar * normal[1];
                    impulse[dof + 2] += scalar * normal[2];
                }
            }
        }
        for (let side = 0; side < 2; side++) {
            const body = bodies[side];
            if (direct) kirchhoffDirectBlockResponse(body, state.impulse[side], state.response[side]);
            else {
                const correction = state.response[side].localCorrection ??= new Float64Array(body.count * 6);
                state.response[side].correction = correction;
                correction.fill(0);
                for (let node = body.activeStart; node <= body.activeEnd; node++) {
                    for (let axis = 0; axis < 3; axis++) correction[node * 6 + axis] = body.inverseMass[node] * state.impulse[side][node * 6 + axis];
                }
            }
        }
        for (let i = 0; i < count; i++) {
            const r = records[i], normal = r.normal;
            let value = (r._normalAlpha + regularization * diagonal[i]) * input[i];
            for (let side = 0; side < 2; side++) {
                const correction = state.response[side].correction;
                for (let k = 0; k < sizes[i * 2 + side]; k++) {
                    const offset = i * 8 + side * 4 + k, dof = nodes[offset] * 6;
                    value += weights[offset] * (normal[0] * correction[dof] + normal[1] * correction[dof + 1] + normal[2] * correction[dof + 2]);
                }
            }
            output[i] = value;
        }
    };
    let previousDot = 0, restart = true, refactor = true;
    for (let iteration = 0; iteration < 96; iteration++) {
        let dot = 0, maximum = 0;
        for (let i = 0; i < count; i++) if (free[i]) maximum = Math.max(maximum, Math.abs(residual[i]));
        if (maximum < tolerance) {
            let entering = -1, violation = tolerance;
            for (let i = 0; i < count; i++) if (!free[i] && residual[i] > violation) {
                entering = i; violation = residual[i];
            }
            if (entering < 0) break;
            free[entering] = 1;
            refactor = restart = true;
        }
        if (refactor) { factor(); refactor = false; }
        precondition();
        for (let i = 0; i < count; i++) dot += residual[i] * preconditioned[i];
        if (dot < 1e-24) break;
        const beta = restart ? 0 : dot / previousDot;
        for (let i = 0; i < count; i++) direction[i] = free[i] ? preconditioned[i] + beta * direction[i] : 0;
        let changed = false;
        for (let i = 0; i < count; i++) {
            if (records[i].manifoldContact.normalLambda + increment[i] <= 1e-10 && direction[i] < -1e-12) {
                free[i] = 0; changed = true;
            }
        }
        if (changed) { restart = refactor = true; continue; }
        multiply(direction, product);
        let denominator = 0, bound = Infinity;
        for (let i = 0; i < count; i++) {
            denominator += direction[i] * product[i];
            if (direction[i] < 0) bound = Math.min(bound,
                -(records[i].manifoldContact.normalLambda + increment[i]) / direction[i]);
        }
        if (denominator <= 1e-24) break;
        // Release multiple bounds together only along a feasible descent step.
        // The inequality below gives a positive Armijo decrease (at least 5%
        // of the directional decrease) in the quadratic contact objective.
        if (!direct && bound < dot / denominator) {
            let step = dot / denominator, accepted = false;
            const delta = state.projected, action = state.projectedProduct;
            for (let search = 0; search < 16; search++, step *= 0.5) {
                for (let i = 0; i < count; i++) delta[i] = Math.max(-records[i].manifoldContact.normalLambda,
                    increment[i] + step * direction[i]) - increment[i];
                multiply(delta, action);
                let descent = 0, curvature = 0;
                for (let i = 0; i < count; i++) { descent += delta[i] * residual[i]; curvature += delta[i] * action[i]; }
                if (descent > 0 && curvature < 1.9 * descent) { accepted = true; break; }
            }
            if (accepted) {
                for (let i = 0; i < count; i++) {
                    increment[i] += delta[i]; residual[i] -= action[i];
                    if (free[i] && records[i].manifoldContact.normalLambda + increment[i] <= 1e-10) free[i] = 0;
                }
                restart = refactor = true;
                state.iterations++;
                continue;
            }
        }
        const step = Math.min(dot / denominator, bound);
        for (let i = 0; i < count; i++) {
            increment[i] += step * direction[i];
            residual[i] -= step * product[i];
            if (step >= bound && free[i] && direction[i] < 0 && records[i].manifoldContact.normalLambda + increment[i] <= 1e-10) {
                free[i] = 0; refactor = true;
            }
        }
        previousDot = dot;
        restart = step >= bound;
        state.iterations++;
    }
    multiply(increment, product, true);
    let scale = 1;
    for (let side = 0; side < 2; side++) {
        const b = bodies[side], c = state.response[side].correction;
        for (let node = b.activeStart; node <= b.activeEnd; node++) {
            const d = node * 6;
            scale = Math.min(scale,
                Math.min(0.25, constraint.lumenMaxCorrection) / Math.max(1e-12, Math.hypot(c[d], c[d + 1], c[d + 2])),
                0.1 / Math.max(1e-12, Math.hypot(c[d + 3], c[d + 4], c[d + 5])));
        }
    }
    for (let side = 0; side < 2; side++) {
        const response = state.response[side];
        const body = bodies[side];
        if (direct) applyKirchhoffDirectCorrection(body, response.correction, response.lambda, scale, true);
        else {
            for (let node = body.activeStart; node <= body.activeEnd; node++) {
                const d = node * 6, c = response.correction;
                body.x[node] += scale * c[d]; body.y[node] += scale * c[d + 1]; body.z[node] += scale * c[d + 2];
                body.toolProjectionX[node] += scale * c[d]; body.toolProjectionY[node] += scale * c[d + 1]; body.toolProjectionZ[node] += scale * c[d + 2];
            }
        }
    }
    for (let i = 0; i < count; i++) {
        increment[i] *= scale;
        records[i].manifoldContact._blockActive = records[i].manifoldContact.normalLambda + increment[i] > 1e-10;
    }
    return state;
}
