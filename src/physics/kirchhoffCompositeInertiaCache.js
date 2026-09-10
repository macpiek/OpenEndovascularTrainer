import { createCompositeInertiaWorkspace, assembleCompositeTranslationalInertia,
    scatterCompositeTranslationalInertia } from './kirchhoffCompositeKinematics.js';

/** Compile EXACT Kinematics inertia once for one dt and fixed material maps.
 * Origin is the previous accepted spatial position field. Both old physical
 * material velocities and convective map rates are frozen by owned packed
 * values; future input mutation cannot change the compiled transaction.
 *
 * append(positions,chain) belongs AFTER elastic assembly, BEFORE constraints.
 * It adds each original full consistent edge Hessian, including convection
 * and off-diagonal terms. No mass lumping, implicit angular inertia, map/feed
 * update, material history commit, or timestep acceptance occurs here.
 * Layout is borrowed read-only and must be the target Chain's layout object.
 * append outputs/sample arrays are borrowed until the next append call.
 * Explicit order:'gradient' adds only energy/gradient and marks the target
 * Hessian invalid; stale matrix bytes are never read, validated or modified.
 */
export function createCompositeInertiaCache({ layout, coordinates, previousPositions, dt, inertiaEdges }) {
    const n = layout?.nodeCount, dofs = layout?.dofCount, band = layout?.band;
    if (!Number.isInteger(n) || n < 2 || !Number.isInteger(dofs) || !Number.isInteger(band) ||
        coordinates?.length !== n || previousPositions?.length !== n || inertiaEdges?.length !== n - 1)
        throw new RangeError('Inertia compilation needs one fixed Chain layout, coordinate/position per node and input per edge');
    const origin = new Float64Array(3 * n), positionDofs = new Int32Array(3 * n);
    for (let node = 0; node < n; node++) {
        if (previousPositions[node]?.length !== 3 || !previousPositions[node].every(Number.isFinite))
            throw new TypeError('Finite previous spatial positions are required');
        for (let axis = 0; axis < 3; axis++) {
            origin[3 * node + axis] = previousPositions[node][axis];
            positionDofs[3 * node + axis] = layout.positions[node] + axis;
        }
    }
    const toolCount = inertiaEdges.reduce((sum, edge) => sum + (edge?.tools?.length ?? 0), 0);
    const sampleCount = 2 * toolCount;
    const edgeToolOffsets = new Int32Array(n), coefficients = new Float64Array(2 * sampleCount), weights = new Float64Array(sampleCount);
    const affineTerms = new Float64Array(3 * sampleCount);
    const velocityOrigin = new Float64Array(3 * sampleCount), incrementOrigin = new Float64Array(3 * sampleCount);
    const velocities = new Float64Array(3 * sampleCount), velocityIncrements = new Float64Array(3 * sampleCount);
    const toolIds = [], edgeMatrixOffsets = new Int32Array(n), matrixIndices = [], matrixValues = [];
    const locals = [null, createCompositeInertiaWorkspace(1), createCompositeInertiaWorkspace(2)];
    const validationChain = { layout, energy: 0, gradient: new Float64Array(dofs), hessian: new Float64Array(dofs * band) };
    let toolCursor = 0, sampleCursor = 0, mass = 0, oldKineticEnergy = 0;
    for (let edge = 0; edge + 1 < n; edge++) {
        const tools = inertiaEdges[edge]?.tools;
        if (!Array.isArray(tools) || tools.length < 1 || tools.length > 2) throw new RangeError('One or two prepared material inputs are required per edge');
        edgeToolOffsets[edge] = toolCursor; edgeMatrixOffsets[edge] = matrixIndices.length;
        const local = assembleCompositeTranslationalInertia({ coordinates: [coordinates[edge], coordinates[edge + 1]],
            positions: [previousPositions[edge], previousPositions[edge + 1]],
            previousPositions: [previousPositions[edge], previousPositions[edge + 1]], dt, tools }, locals[tools.length]);
        // The original scatter validates ownership, all position DOFs and the
        // complete consistent band. It runs only at transaction compilation.
        scatterCompositeTranslationalInertia(local, edge, validationChain);
        mass += local.mass; oldKineticEnergy += local.oldKineticEnergy;
        for (const tool of local.tools) {
            toolIds.push(tool.id); toolCursor++;
            for (const sample of tool.samples) {
                weights[sampleCursor] = sample.massWeight;
                coefficients[2 * sampleCursor] = sample.coefficients[0]; coefficients[2 * sampleCursor + 1] = sample.coefficients[1];
                affineTerms[3 * sampleCursor] = (1 - sample.fraction) / dt;
                affineTerms[3 * sampleCursor + 1] = sample.fraction / dt;
                affineTerms[3 * sampleCursor + 2] = sample.u / (coordinates[edge + 1] - coordinates[edge]);
                for (let axis = 0; axis < 3; axis++) {
                    velocityOrigin[3 * sampleCursor + axis] = sample.velocity[axis];
                    incrementOrigin[3 * sampleCursor + axis] = sample.velocityIncrement[axis];
                }
                sampleCursor++;
            }
        }
        for (let row = 0; row < 6; row++) for (let col = 0; col <= row; col++) {
            const value = local.hessian[6 * row + col];
            if (value === 0) continue;
            const a = positionDofs[3 * edge + row], b = positionDofs[3 * edge + col];
            matrixIndices.push(Math.max(a, b) * band + Math.abs(a - b)); matrixValues.push(value);
        }
    }
    if (!Number.isFinite(mass) || !Number.isFinite(oldKineticEnergy)) throw new RangeError('Nonfinite compiled mass or old kinetic energy');
    edgeToolOffsets[n - 1] = toolCursor; edgeMatrixOffsets[n - 1] = matrixIndices.length;
    const packedMatrixIndices = Int32Array.from(matrixIndices), packedMatrixValues = Float64Array.from(matrixValues);
    const displacement = new Float64Array(3 * n), localGradient = new Float64Array(6), localKineticGradient = new Float64Array(6);
    const gradient = new Float64Array(dofs), kineticGradient = new Float64Array(dofs);
    const toolEnergy = new Float64Array(toolCount), toolKineticEnergy = new Float64Array(toolCount);
    const result = { layout, dt, mass, oldKineticEnergy, energy: 0, kineticEnergy: 0,
        gradient, kineticGradient, velocities, velocityIncrements, toolEnergy, toolKineticEnergy,
        // These immutable descriptors identify the independent per-edge
        // materials in packed diagnostic outputs without exposing frozen data.
        edgeTools: Object.freeze(Array.from({ length: n - 1 }, (_, edge) => Object.freeze(toolIds.slice(edgeToolOffsets[edge], edgeToolOffsets[edge + 1])))),
        sampleCount, storedMatrixEntries: packedMatrixValues.length,
        energyKind: 'material-velocity-increment', includesAngularInertia: false, append };
    function append(positions, chain, { order = 'full' } = {}) {
        if (!['full', 'gradient'].includes(order)) throw new TypeError('Inertia append order must be full or gradient');
        const withHessian = order === 'full';
        if (!withHessian) chain.hessianValid = false;
        if (chain.layout !== layout || positions?.length !== n || chain.gradient?.length !== dofs || chain.hessian?.length !== dofs * band)
            throw new RangeError('Compiled inertia needs its original fixed Chain layout and position count');
        // Current geometry is the ONLY changing physical input. Validate it
        // once per node, never maps/masses/history again in the hot edge loop.
        for (let node = 0; node < n; node++) {
            const p = positions[node];
            if (p?.length !== 3) throw new TypeError('Current positions need three finite coordinates');
            for (let axis = 0; axis < 3; axis++) {
                if (!Number.isFinite(p[axis])) throw new RangeError('Nonfinite current inertia position');
                displacement[3 * node + axis] = p[axis] - origin[3 * node + axis];
            }
        }
        result.energy = result.kineticEnergy = 0; gradient.fill(0); kineticGradient.fill(0);
        for (let edge = 0; edge + 1 < n; edge++) {
            localGradient.fill(0); localKineticGradient.fill(0);
            let edgeEnergy = 0, edgeKineticEnergy = 0;
            for (let tool = edgeToolOffsets[edge]; tool < edgeToolOffsets[edge + 1]; tool++) {
                let energy = 0, kineticEnergy = 0;
                for (let sample = 2 * tool; sample < 2 * tool + 2; sample++) {
                    const a0 = coefficients[2 * sample], a1 = coefficients[2 * sample + 1], weight = weights[sample];
                    for (let axis = 0; axis < 3; axis++) {
                        const d0 = displacement[3 * edge + axis], d1 = displacement[3 * (edge + 1) + axis];
                        // Algebraically a0*d0+a1*d1. Keep the convective
                        // difference grouped so rigid displacement cannot
                        // cancel two very large opposite transport terms.
                        const shift = affineTerms[3 * sample] * d0 + affineTerms[3 * sample + 1] * d1 +
                            affineTerms[3 * sample + 2] * (d1 - d0);
                        const velocity = velocityOrigin[3 * sample + axis] + shift;
                        const increment = incrementOrigin[3 * sample + axis] + shift;
                        velocities[3 * sample + axis] = velocity; velocityIncrements[3 * sample + axis] = increment;
                        // Positive sums avoid cancellation of a large kinetic
                        // reference energy when a velocity increment is small.
                        energy += .5 * weight * increment * increment;
                        kineticEnergy += .5 * weight * velocity * velocity;
                        localGradient[axis] += weight * a0 * increment; localGradient[3 + axis] += weight * a1 * increment;
                        localKineticGradient[axis] += weight * a0 * velocity; localKineticGradient[3 + axis] += weight * a1 * velocity;
                    }
                }
                toolEnergy[tool] = energy; toolKineticEnergy[tool] = kineticEnergy;
                edgeEnergy += energy; edgeKineticEnergy += kineticEnergy;
            }
            result.energy += edgeEnergy; result.kineticEnergy += edgeKineticEnergy; chain.energy += edgeEnergy;
            for (let row = 0; row < 6; row++) {
                const dof = positionDofs[3 * edge + row];
                gradient[dof] += localGradient[row]; kineticGradient[dof] += localKineticGradient[row];
                chain.gradient[dof] += localGradient[row];
            }
            // Preserve the original edge-by-edge accumulation order exactly.
            if (withHessian) for (let at = edgeMatrixOffsets[edge]; at < edgeMatrixOffsets[edge + 1]; at++)
                chain.hessian[packedMatrixIndices[at]] += packedMatrixValues[at];
        }
        if (![chain.energy, result.energy, result.kineticEnergy].every(Number.isFinite) ||
            !chain.gradient.every(Number.isFinite) || (withHessian && !chain.hessian.every(Number.isFinite)) ||
            !kineticGradient.every(Number.isFinite) || !velocities.every(Number.isFinite) || !velocityIncrements.every(Number.isFinite))
            throw new RangeError('Nonfinite compiled inertia response');
        return result;
    }
    return result;
}
