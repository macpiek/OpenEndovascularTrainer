/** Bilateral edge-length constraints for ONE common axis.
 * coordinates are its physical rest-arc coordinates x, so the target edge
 * length is x[e+1]-x[e]. A tool's dsDx is a material-coordinate measure;
 * it is not physical stretch and does not enter this operator.
 */
function finiteArray(values, count, name) {
    if (!values || values.length !== count) throw new TypeError(`${name} needs ${count} finite values`);
    for (let i = 0; i < count; i++) if (!Number.isFinite(values[i]))
        throw new TypeError(`${name} needs ${count} finite values`);
}

export function createCompositeLengthConstraintWorkspace(layout) {
    if (!layout || !Number.isInteger(layout.nodeCount) || layout.nodeCount < 2 ||
        !Number.isInteger(layout.dofCount) || layout.dofCount < 6 ||
        !Number.isInteger(layout.band) || layout.band < 1)
        throw new RangeError('A common-chain layout with at least two nodes is required');
    finiteArray(layout.positions, layout.nodeCount, 'position DOF offsets');
    for (let i = 0; i < layout.nodeCount; i++) {
        const offset = layout.positions[i];
        if (!Number.isInteger(offset) || offset < 0 || offset + 2 >= layout.dofCount ||
            (i && offset < layout.positions[i - 1] + 3))
            throw new RangeError('Distinct spatially ordered position DOFs are required');
        if (i && offset + 2 - layout.positions[i - 1] >= layout.band)
            throw new RangeError('The existing band must contain every edge position block');
    }
    const edges = layout.nodeCount - 1, n = layout.dofCount;
    const evaluation = {
        lengths: new Float64Array(edges), restLengths: new Float64Array(edges),
        residuals: new Float64Array(edges), directions: new Float64Array(3 * edges),
        jacobian: new Float64Array(6 * edges), physicalMultipliers: new Float64Array(edges),
        constraintForces: new Float64Array(n), maximumLengthResidual: 0,
        withinLengthTolerance: false, tolerance: null, scope: 'original-length-constraints',
    };
    const operator = {
        energy: 0, gradient: new Float64Array(n), hessian: new Float64Array(n * layout.band),
        trialMultipliers: new Float64Array(edges), trialForces: new Float64Array(n),
        penalties: new Float64Array(edges), evaluation,
        hessianType: 'gauss-newton', scope: 'length-augmented-objective',
    };
    return { layout, evaluation, operator };
}

/** Re-evaluate ORIGINAL g_e=|q1-q0|-(x1-x0) and its exact first derivative.
 * multipliers are signed physical bilateral multipliers (force units).
 * constraintForces=-J^T multipliers. The length-only tolerance flag does not
 * certify force balance, nonlinear convergence, or acceptance of a timestep.
 * Outputs borrow workspace buffers and are overwritten by the next call.
 */
export function evaluateCompositeLengthConstraints({ positions, coordinates, multipliers, tolerance }, workspace) {
    const { layout, evaluation: out } = workspace, edges = layout.nodeCount - 1;
    out.withinLengthTolerance = false;
    if (!Number.isFinite(tolerance) || tolerance <= 0)
        throw new RangeError('An explicit positive physical length tolerance is required');
    if (!positions || positions.length !== layout.nodeCount)
        throw new RangeError('Positions must match the common chain');
    finiteArray(coordinates, layout.nodeCount, 'rest-arc coordinates');
    finiteArray(multipliers, edges, 'physical length multipliers');
    for (let i = 0; i < positions.length; i++) finiteArray(positions[i], 3, 'position');
    out.constraintForces.fill(0);
    out.maximumLengthResidual = 0;
    out.tolerance = tolerance;
    out.physicalMultipliers.set(multipliers);
    for (let e = 0; e < edges; e++) {
        const rest = coordinates[e + 1] - coordinates[e];
        if (!(rest > 0) || !Number.isFinite(rest))
            throw new RangeError('Common rest-arc coordinates must strictly increase with finite edge lengths');
        const dx = positions[e + 1][0] - positions[e][0];
        const dy = positions[e + 1][1] - positions[e][1];
        const dz = positions[e + 1][2] - positions[e][2];
        const length = Math.hypot(dx, dy, dz);
        if (!(length > 0) || !Number.isFinite(length))
            throw new RangeError('A physical edge cannot collapse or have nonfinite length');
        const residual = length - rest, lambda = out.physicalMultipliers[e];
        out.lengths[e] = length; out.restLengths[e] = rest; out.residuals[e] = residual;
        out.maximumLengthResidual = Math.max(out.maximumLengthResidual, Math.abs(residual));
        for (let axis = 0; axis < 3; axis++) {
            const t = (axis === 0 ? dx : axis === 1 ? dy : dz) / length;
            out.directions[3 * e + axis] = t;
            out.jacobian[6 * e + axis] = -t;
            out.jacobian[6 * e + 3 + axis] = t;
            out.constraintForces[layout.positions[e] + axis] += lambda * t;
            out.constraintForces[layout.positions[e + 1] + axis] -= lambda * t;
        }
    }
    if (!Number.isFinite(out.maximumLengthResidual) || !out.constraintForces.every(Number.isFinite))
        throw new RangeError('Nonfinite original length residual or physical reaction');
    out.withinLengthTolerance = out.maximumLengthResidual <= tolerance;
    return out;
}

/** Optional AL contribution sum(lambda*g + .5*mu*g^2), with explicit positive
 * penalty mu in force/length units (scalar or one value per edge). The trial
 * multiplier lambda+mu*g is NOT a committed physical reaction. The supplied
 * multipliers are never updated. GN is mu*J^T*J; it deliberately omits the
 * exact curvature term (lambda+mu*g)*Hessian(g).
 *
 * If target is supplied, ADD this operator once to the already assembled
 * Chain energy, gradient and lower band. It must share workspace.layout.
 * This does not solve constraints, update multipliers, or accept a timestep.
 */
export function assembleCompositeLengthConstraints(args, workspace, target) {
    const { layout, operator: out } = workspace, edges = layout.nodeCount - 1;
    const scalarPenalty = typeof args.penalty === 'number';
    if (scalarPenalty) {
        if (!Number.isFinite(args.penalty) || args.penalty <= 0)
            throw new RangeError('An explicit positive finite AL penalty is required');
    } else {
        finiteArray(args.penalty, edges, 'AL penalties');
        if (Array.from(args.penalty).some(v => v <= 0))
            throw new RangeError('Every AL penalty must be positive');
    }
    const measured = evaluateCompositeLengthConstraints(args, workspace);
    out.energy = 0; out.gradient.fill(0); out.hessian.fill(0);
    for (let e = 0; e < edges; e++) {
        const mu = scalarPenalty ? args.penalty : args.penalty[e], g = measured.residuals[e];
        const trial = measured.physicalMultipliers[e] + mu * g;
        out.penalties[e] = mu; out.trialMultipliers[e] = trial;
        out.energy += measured.physicalMultipliers[e] * g + .5 * (mu * g) * g;
        if (!Number.isFinite(trial)) throw new RangeError('Nonfinite AL trial multiplier');
        for (let row = 0; row < 6; row++) {
            const globalRow = layout.positions[e + (row >= 3 ? 1 : 0)] + row % 3;
            const jr = measured.jacobian[6 * e + row];
            out.gradient[globalRow] += trial * jr;
            for (let col = 0; col <= row; col++) {
                const globalCol = layout.positions[e + (col >= 3 ? 1 : 0)] + col % 3;
                out.hessian[globalRow * layout.band + globalRow - globalCol] +=
                    mu * jr * measured.jacobian[6 * e + col];
            }
        }
    }
    if (!Number.isFinite(out.energy) || !out.gradient.every(Number.isFinite) || !out.hessian.every(Number.isFinite))
        throw new RangeError('Nonfinite length augmented operator');
    for (let i = 0; i < layout.dofCount; i++) out.trialForces[i] = -out.gradient[i];
    if (target) {
        if (target.layout !== layout || target.gradient === out.gradient || target.hessian === out.hessian)
            throw new RangeError('Scatter requires a separate target sharing the common layout');
        finiteArray(target.gradient, layout.dofCount, 'target gradient');
        finiteArray(target.hessian, layout.dofCount * layout.band, 'target band');
        if (!Number.isFinite(target.energy) || !Number.isFinite(target.energy + out.energy))
            throw new RangeError('Nonfinite target energy');
        // Validate additions before mutating an existing material operator.
        for (const key of ['gradient', 'hessian']) for (let i = 0; i < out[key].length; i++)
            if (!Number.isFinite(target[key][i] + out[key][i])) throw new RangeError('Nonfinite scattered length operator');
        target.energy += out.energy;
        for (const key of ['gradient', 'hessian']) for (let i = 0; i < out[key].length; i++) target[key][i] += out[key][i];
    }
    return out;
}

/** Call on the final candidate geometry and physical multipliers. This reads
 * original constraints afresh; no cached AL trial multiplier or energy is a
 * substitute for physical residuals and signed constraint forces.
 */
export function measureCompositeLengthConstraints(args, workspace) {
    return evaluateCompositeLengthConstraints(args, workspace);
}
