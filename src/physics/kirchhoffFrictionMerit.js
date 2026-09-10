import { evaluateKirchhoffSurfaceFriction } from './kirchhoffSurfaceFriction.js';

/** A continuous merit for line search, NOT the final friction acceptance test.
 * k=lambda_max(J W J^T) has units mm/multiplier. The natural map
 * k*(lambda-P_E(lambda-displacement/k)) is continuous when a contact loads or
 * unloads, including Fn=0. The displacement/cone KKT gates remain independent.
 * Local angular mobility and hard frame/active-range masks match system apply.
 */
export function measureKirchhoffFrictionMerit(batch, out = {}) {
    const coefficients = out._coefficients ??= new Map();
    const pool = out._pool ??= [];
    const options = out._options ??= {};
    out.maximumMm = 0;
    for (const entry of batch.entries) {
        const rows = entry.surface.rows, bodies = entry.surface._surfaceScratch.bodies;
        coefficients.clear();
        let used = 0;
        for (let axis = 0; axis < 2; axis++) for (const g of rows[axis].gradients) {
            const key = g.side * (bodies[0].count * 6) + g.dof;
            let pair = coefficients.get(key);
            if (!pair) {
                pair = pool[used++] ??= [0, 0, 0];
                pair[0] = pair[1] = 0;
                const body = bodies[g.side], node = Math.floor(g.dof / 6), component = g.dof % 6;
                const start = body.activeStart ?? 0, end = body.activeEnd ?? body.count - 1;
                pair[2] = node < start || node > end || component >= 3 &&
                    (node === end || body.orientationControlCompliance === 0 && node === body.orientationControlSegment)
                    ? 0 : component < 3 ? body.inverseMass[node] : body['inverseInertia' + (component - 2)][node];
                coefficients.set(key, pair);
            }
            pair[axis] += g.value;
        }
        let a = 0, b = 0, c = 0;
        for (const [u, v, w] of coefficients.values()) {
            a += u * w * u; b += u * w * v; c += v * w * v;
        }
        const mobility = .5 * (a + c + Math.hypot(a - c, 2 * b));
        // A wholly prescribed witness has no available mechanical correction;
        // use a finite reference scale only for its merit, never acceptance.
        const k = mobility > 0 ? mobility : 1;
        options.inverseMobility = 1 / k;
        const lambda = out._lambda ??= new Float64Array(2), slip = out._slip ??= new Float64Array(2);
        for (let axis = 0; axis < 2; axis++) { lambda[axis] = rows[axis].lambda; slip[axis] = rows[axis].strain; }
        const residual = evaluateKirchhoffSurfaceFriction(lambda, slip,
            entry.contact.normalLambda, entry.surface.group.mu, options, out._residual ??= {});
        out.maximumMm = Math.max(out.maximumMm, k * residual.residual);
    }
    return out;
}
