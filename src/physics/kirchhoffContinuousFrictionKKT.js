/** Continuous fixed-normal maximum-dissipation residual in displacement units.
 * For ellipse E={lambda: |D^-1 lambda|<=1}, sliding gives
 * lambda=-D^2 d/|D d|. Thus d+|D d| D^-2 lambda=0, while d=0
 * permits all feasible sticking forces. Force feasibility remains independent.
 * No inverse mobility, mass-dependent tolerance, or force projection is used.
 */
export function evaluateKirchhoffContinuousFrictionKKT(lambda, displacement, normalLambda, mu, out = {}) {
    if (!Number.isFinite(normalLambda) || normalLambda < 0) throw new RangeError('Invalid normal load');
    out.axes ??= new Float64Array(2);
    out.displacementResidual ??= new Float64Array(2);
    out.coneViolation = out.residualMm = 0;
    for (let i = 0; i < 2; i++) {
        if (!Number.isFinite(lambda[i]) || !Number.isFinite(displacement[i]) || !Number.isFinite(mu[i]) || mu[i] < 0)
            throw new RangeError('Invalid friction data');
        out.axes[i] = normalLambda * mu[i];
        if (!Number.isFinite(out.axes[i])) throw new RangeError('Non-finite friction radius');
        out.displacementResidual[i] = 0;
    }
    const [a, b] = out.axes;
    if (a === 0 || b === 0) {
        for (let i = 0; i < 2; i++) {
            const radius = out.axes[i], value = lambda[i], d = displacement[i];
            if (radius === 0) {
                if (value !== 0) out.coneViolation = Infinity;
            } else {
                out.coneViolation = Math.max(out.coneViolation, Math.abs(value) / radius - 1);
                out.displacementResidual[i] = d + Math.abs(d) * value / radius;
            }
        }
    } else {
        const scale = Math.max(a, b), ax = a / scale, by = b / scale;
        const weightedSlip = Math.hypot(ax * displacement[0], by * displacement[1]);
        const x = lambda[0] / a, y = lambda[1] / b;
        out.coneViolation = Math.max(0, Math.hypot(x, y) - 1);
        out.displacementResidual[0] = displacement[0] + weightedSlip * x / ax;
        out.displacementResidual[1] = displacement[1] + weightedSlip * y / by;
    }
    out.residualMm = Math.max(Math.abs(out.displacementResidual[0]), Math.abs(out.displacementResidual[1]));
    return out;
}
