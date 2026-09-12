// Independent world-space momentum oracle. Node mass and segment inertia are
// separate contributions: rod angular velocities are WORLD vectors while the
// three inverse inertias are expressed in each segment's MATERIAL frame.
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const vector = (body, prefix, index) => ['X', 'Y', 'Z'].map(axis => body[prefix + axis][index]);
const quaternion = (body, prefix, index) => [...vector(body, prefix, index), body[prefix + 'W'][index]];

function rotate(q, v, inverse = false) {
    const norm = Math.hypot(...q);
    if (!Number.isFinite(norm) || Math.abs(norm - 1) > 1e-8) throw new RangeError('Momentum requires a unit material frame');
    const u = q.slice(0, 3).map(value => value * (inverse ? -1 : 1) / norm), w = q[3] / norm;
    const uv = cross(u, v), uuv = cross(u, uv);
    return v.map((value, axis) => value + 2 * (w * uv[axis] + uuv[axis]));
}

function reconstructedAngularVelocity(body, index, dt) {
    const q = quaternion(body, 'orientation', index), p = quaternion(body, 'previousOrientation', index);
    const pInverse = [-p[0], -p[1], -p[2]], qp = cross(q, pInverse);
    let relative = [...q.slice(0, 3).map((value, axis) => value * p[3] + pInverse[axis] * q[3] + qp[axis]),
        q[3] * p[3] + q[0] * p[0] + q[1] * p[1] + q[2] * p[2]];
    if (relative[3] < 0) relative = relative.map(value => -value);
    const sine = Math.hypot(...relative.slice(0, 3));
    const scale = sine > 1e-15 ? 2 * Math.atan2(sine, relative[3]) / (sine * dt) : 2 / dt;
    return relative.slice(0, 3).map(value => value * scale);
}

/** All active bodies must be free: pinned mass / locked inertia would require
 * external reaction impulses. reconstructDt measures raw position/quaternion
 * reconstruction before velocity filters; it does not mutate the bodies.
 * origin fixes the point about which orbital angular momentum is reported.
 */
export function measureRodSystemMomentum(bodies, { origin = [0, 0, 0], reconstructDt = null } = {}) {
    if (reconstructDt !== null && !(Number.isFinite(reconstructDt) && reconstructDt > 0)) throw new RangeError('Invalid reconstruction timestep');
    const result = { linear: [0, 0, 0], orbital: [0, 0, 0], spin: [0, 0, 0], total: [0, 0, 0], kineticEnergy: 0 };
    for (const body of bodies) {
        for (let i = body.activeStart; i <= body.activeEnd; i++) {
            const inverseMass = body.inverseMass[i];
            if (!(Number.isFinite(inverseMass) && inverseMass > 0)) throw new RangeError('Momentum oracle requires free active nodes');
            const position = ['x', 'y', 'z'].map(axis => body[axis][i]);
            const velocity = reconstructDt === null ? vector(body, 'velocity', i) : position.map((value, axis) =>
                (value - body['previous' + ['X', 'Y', 'Z'][axis]][i]) / reconstructDt);
            const momentum = velocity.map(value => value / inverseMass);
            const orbital = cross(position.map((value, axis) => value - origin[axis]), momentum);
            for (let axis = 0; axis < 3; axis++) { result.linear[axis] += momentum[axis]; result.orbital[axis] += orbital[axis]; }
            result.kineticEnergy += velocity.reduce((energy, value, axis) => energy + value * momentum[axis], 0) / 2;
        }
        for (let i = body.activeStart; i < Math.min(body.activeEnd, body.segmentCount); i++) {
            const q = quaternion(body, 'orientation', i);
            const omega = reconstructDt === null ? vector(body, 'angularVelocity', i) : reconstructedAngularVelocity(body, i, reconstructDt);
            const localOmega = rotate(q, omega, true);
            const localMomentum = localOmega.map((value, axis) => {
                const inverseInertia = body['inverseInertia' + (axis + 1)][i];
                if (!(Number.isFinite(inverseInertia) && inverseInertia > 0)) throw new RangeError('Momentum oracle requires free active segment rotations');
                return value / inverseInertia;
            });
            const spin = rotate(q, localMomentum);
            for (let axis = 0; axis < 3; axis++) result.spin[axis] += spin[axis];
            result.kineticEnergy += omega.reduce((energy, value, axis) => energy + value * spin[axis], 0) / 2;
        }
    }
    result.total = result.orbital.map((value, axis) => value + result.spin[axis]);
    if (!Object.values(result).flat().every(Number.isFinite)) throw new RangeError('Non-finite momentum');
    return result;
}

/** Instantaneous wrench of an unapplied Newton correction. This uses the
 * actual recovered mobility response, independently of contact diagnostics.
 * dx = M^-1 J^T dLambda; local dtheta = I_local^-1 J_theta^T dLambda.
 * Divide impulses by dt for velocity units. A separate orbital term describes
 * the finite position-reconstruction error: dL = p x m dx/dt + dx x m v.
 */
export function measureRodSystemCorrection(bodies, responses, scale, dt) {
    const result = { linearImpulse: [0, 0, 0], momentImpulse: [0, 0, 0], angularImpulse: [0, 0, 0], finiteOrbital: [0, 0, 0] };
    for (const [side, body] of bodies.entries()) {
        const correction = responses[side].correction;
        for (let i = body.activeStart; i <= body.activeEnd; i++) {
            const p = ['x', 'y', 'z'].map(axis => body[axis][i]);
            const delta = Array.from(correction.slice(i * 6, i * 6 + 3), value => value * scale);
            const impulse = delta.map(value => value / body.inverseMass[i]);
            const moment = cross(p, impulse);
            const previousMomentum = p.map((value, axis) =>
                (value - body['previous' + ['X', 'Y', 'Z'][axis]][i]) / (dt * body.inverseMass[i]));
            const finiteOrbital = cross(delta, previousMomentum);
            for (let axis = 0; axis < 3; axis++) {
                result.linearImpulse[axis] += impulse[axis];
                result.momentImpulse[axis] += moment[axis];
                result.finiteOrbital[axis] += finiteOrbital[axis];
            }
            if (i === body.activeEnd) continue;
            const localImpulse = [0, 1, 2].map(axis => scale * correction[i * 6 + 3 + axis] / body['inverseInertia' + (axis + 1)][i]);
            const angularImpulse = rotate(quaternion(body, 'orientation', i), localImpulse);
            for (let axis = 0; axis < 3; axis++) {
                result.momentImpulse[axis] += angularImpulse[axis];
                result.angularImpulse[axis] += angularImpulse[axis];
            }
        }
    }
    if (!Object.values(result).flat().every(Number.isFinite)) throw new RangeError('Non-finite correction wrench');
    return result;
}
