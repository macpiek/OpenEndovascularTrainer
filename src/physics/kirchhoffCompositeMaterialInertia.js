import { createCompositeInertiaWorkspace, assembleCompositeTranslationalInertia } from './kirchhoffCompositeKinematics.js';
import { createCompositeContinuousMaterialInertiaEdge } from './kirchhoffCompositeContinuousInertia.js';

/** One material's exact prepared affine-edge inertia. Its current and previous
 * positions belong to THAT material, allowing the wire and catheter to have
 * different geometries in one joint solve. Material maps and old velocities
 * are explicit, owned and frozen for this prepared dt. No history is inferred
 * from the other material and no angular inertia is silently added.
 *
 * evaluate() returns borrowed local [x0,y0,z0,x1,y1,z1] E/g/H and physical
 * velocity/momentum diagnostics. The full consistent 6x6 tangent includes
 * convection and cross-endpoint terms. The caller pulls these same local
 * blocks into common/relative coordinates once. No scatter or solve happens
 * here, and this object cannot certify a physical timestep. An explicit
 * continuousGeometry opts into the shared C2 field and its larger local
 * position support; that path retains polynomial old material velocities.
 */
export function createCompositeMaterialInertiaEdge({ coordinates, previousPositions, dt, tool, continuousGeometry }) {
    if (continuousGeometry !== undefined) {
        if (coordinates?.length !== 2 || coordinates.some((v,i)=>v!==continuousGeometry.coordinates?.[i]))
            throw new RangeError('Continuous inertia coordinates must equal the compiled physical edge');
        return createCompositeContinuousMaterialInertiaEdge({geometry:continuousGeometry,previousPositions,dt,tool});
    }
    const output = createCompositeInertiaWorkspace(1);
    // The existing independent operator validates the physical inputs and
    // compiles the immutable two-sample coefficients at zero displacement.
    assembleCompositeTranslationalInertia({ coordinates, positions: previousPositions,
        previousPositions, dt, tools: [tool] }, output);
    const origin = Float64Array.from(previousPositions.flatMap(p => Array.from(p)));
    const hessian = output.hessian.slice(), length = coordinates[1] - coordinates[0];
    const mass = output.mass, oldKineticEnergy = output.oldKineticEnergy;
    const oldMomentum = output.tools[0].oldMomentum.slice(), id = output.tools[0].id;
    const prepared = output.tools[0].samples.map(sample => ({ fraction: sample.fraction, s: sample.s,
        dsDt: sample.dsDt, u: sample.u, massWeight: sample.massWeight,
        coefficients: sample.coefficients.slice(), velocity: sample.velocity.slice(),
        oldMaterialVelocity: sample.oldMaterialVelocity.slice(), velocityIncrement: sample.velocityIncrement.slice(),
        translation: [(1 - sample.fraction) / dt, sample.fraction / dt], transport: sample.u / length }));
    const displacement = new Float64Array(6);
    output.hessianValid = true; output.evaluationOrder = 'full';
    output.scope = 'prepared-one-material-affine-inertia'; output.certified = false;
    function evaluate(positions, { order = 'full' } = {}) {
        output.hessianValid = false;
        if (order !== 'full' && order !== 'gradient') throw new TypeError('Material inertia order must be full or gradient');
        if (positions?.length !== 2) throw new RangeError('Two current material endpoints are required');
        for (let node = 0; node < 2; node++) {
            if (positions[node]?.length !== 3) throw new RangeError('Each current material endpoint requires three coordinates');
            for (let axis = 0; axis < 3; axis++) {
                const value = positions[node][axis];
                if (!Number.isFinite(value)) throw new RangeError('Finite current material positions are required');
                displacement[3 * node + axis] = value - origin[3 * node + axis];
            }
        }
        const full = order === 'full', local = output.tools[0];
        output.hessianValid = false; output.evaluationOrder = order;
        output.energy = output.kineticEnergy = 0; output.mass = mass; output.oldKineticEnergy = oldKineticEnergy;
        output.gradient.fill(0); output.kineticGradient.fill(0); output.momentumIncrement.fill(0);
        local.id = id; local.energy = local.kineticEnergy = 0; local.mass = mass; local.oldKineticEnergy = oldKineticEnergy;
        local.momentum.fill(0); local.oldMomentum.set(oldMomentum);
        for (let index = 0; index < prepared.length; index++) {
            const p = prepared[index], sample = local.samples[index], weight = p.massWeight;
            sample.fraction = p.fraction; sample.s = p.s; sample.dsDt = p.dsDt; sample.u = p.u; sample.massWeight = weight;
            sample.coefficients.set(p.coefficients); sample.oldMaterialVelocity.set(p.oldMaterialVelocity);
            for (let axis = 0; axis < 3; axis++) {
                const d0 = displacement[axis], d1 = displacement[3 + axis];
                // Group the transport difference: a rigid displacement must
                // not subtract two large opposite convective contributions.
                const shift = p.translation[0] * d0 + p.translation[1] * d1 + p.transport * (d1 - d0);
                const velocity = p.velocity[axis] + shift, increment = p.velocityIncrement[axis] + shift;
                sample.velocity[axis] = velocity; sample.velocityIncrement[axis] = increment;
                local.energy += .5 * weight * increment * increment;
                local.kineticEnergy += .5 * weight * velocity * velocity;
                local.momentum[axis] += weight * velocity; output.momentumIncrement[axis] += weight * increment;
                output.gradient[axis] += weight * p.coefficients[0] * increment;
                output.gradient[3 + axis] += weight * p.coefficients[1] * increment;
                output.kineticGradient[axis] += weight * p.coefficients[0] * velocity;
                output.kineticGradient[3 + axis] += weight * p.coefficients[1] * velocity;
            }
        }
        output.energy = local.energy; output.kineticEnergy = local.kineticEnergy;
        if (full) output.hessian.set(hessian);
        if (![output.energy, output.kineticEnergy].every(Number.isFinite) ||
            !output.gradient.every(Number.isFinite) || !output.kineticGradient.every(Number.isFinite) ||
            !output.momentumIncrement.every(Number.isFinite) || !local.momentum.every(Number.isFinite) ||
            local.samples.some(sample => !sample.velocity.every(Number.isFinite) || !sample.velocityIncrement.every(Number.isFinite)))
            throw new RangeError('Nonfinite prepared material inertia response');
        output.hessianValid = full;
        return output;
    }
    return Object.freeze({ id, dt, mass, coordinateLength: length, evaluate,
        scope: 'prepared-one-material-affine-inertia', includesAngularInertia: false });
}
