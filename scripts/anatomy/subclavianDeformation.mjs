/** Offline, smooth transport of the vessel wall and its outlet landmarks.
 * Cross-section landmarks constrain radial distortion while the main axis moves.
 * This is an atlas correction, not a patient-specific anatomical registration.
 */
import * as THREE from 'three';

const lerp = (a, b, t) => a.map((x, k) => x + (b[k] - x) * t);
const smooth = value => {
    const t = Math.max(0, Math.min(1, value));
    return t * t * t * (t * (6 * t - 15) + 10);
};
const vector = a => new THREE.Vector3(...a);

function solve(matrix, rhs) {
    const n = matrix.length;
    const a = matrix.map((row, i) => [...row, ...rhs[i]]);
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) {
            if (Math.abs(a[i][k]) > Math.abs(a[pivot][k])) pivot = i;
        }
        [a[k], a[pivot]] = [a[pivot], a[k]];
        if (Math.abs(a[k][k]) < 1e-12) throw Error('Singular deformation field');
        for (let i = k + 1; i < n; i++) {
            const ratio = a[i][k] / a[k][k];
            for (let j = k; j < n + 3; j++) a[i][j] -= ratio * a[k][j];
        }
    }
    const result = Array.from({ length: n }, () => [0, 0, 0]);
    for (let i = n - 1; i >= 0; i--) for (let c = 0; c < 3; c++) {
        let value = a[i][n + c];
        for (let j = i + 1; j < n; j++) value -= a[i][j] * result[j][c];
        result[i][c] = value / a[i][i];
    }
    return result;
}

export function inDeformationRegion(p) {
    return Math.abs(p[0]) >= 20 && Math.abs(p[0]) <= 260 &&
        p[1] >= -160 && p[1] <= 240 && p[2] >= -160 && p[2] <= 160;
}

export function createSubclavianDeformation(curves) {
    const rounds = 100;
    const denominator = 2 * 22 * 22;
    const frames = [];
    for (let step = 0; step < rounds; step++) for (const curve of curves) {
        const centers = [], velocity = [];
        for (let i = 0; i < curve.old.length; i++) {
            const lo = Math.max(0, i - 1), hi = Math.min(curve.old.length - 1, i + 1);
            const at = step / rounds, bt = (step + 1) / rounds;
            const tangent = vector(curve.old[hi]).sub(vector(curve.old[lo])).normalize();
            const currentTangent = vector(lerp(curve.old[hi], curve.next[hi], at))
                .sub(vector(lerp(curve.old[lo], curve.next[lo], at))).normalize();
            const nextTangent = vector(lerp(curve.old[hi], curve.next[hi], bt))
                .sub(vector(lerp(curve.old[lo], curve.next[lo], bt))).normalize();
            const currentRotation = new THREE.Quaternion().setFromUnitVectors(tangent, currentTangent);
            const nextRotation = new THREE.Quaternion().setFromUnitVectors(tangent, nextTangent);
            const u = new THREE.Vector3(0, 0, 1).cross(tangent).normalize().multiplyScalar(6);
            const v = tangent.clone().cross(u).normalize().multiplyScalar(6);
            const current = vector(lerp(curve.old[i], curve.next[i], at));
            const next = vector(lerp(curve.old[i], curve.next[i], bt));
            for (const radial of [new THREE.Vector3(), u, u.clone().negate(), v, v.clone().negate()]) {
                const a = current.clone().add(radial.clone().applyQuaternion(currentRotation));
                const b = next.clone().add(radial.clone().applyQuaternion(nextRotation));
                centers.push(a.toArray());
                velocity.push(b.sub(a).toArray());
            }
        }
        const matrix = centers.map((a, i) => centers.map((b, j) =>
            Math.exp(-(Math.hypot(...a.map((v, k) => v - b[k])) ** 2) / denominator) +
            (i === j ? 0.0001 : 0)));
        frames.push({ side: curve.side, centers, coefficients: solve(matrix, velocity) });
    }
    function move(points, onProgress) {
        for (let f = 0; f < frames.length; f++) {
            const { side, centers, coefficients } = frames[f];
            for (const p of points) {
                if (side === 'right' ? p[0] > 0 : p[0] < 0) continue;
                let vx = 0, vy = 0, vz = 0;
                for (let i = 0; i < centers.length; i++) {
                    const q = centers[i];
                    const dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
                    const distanceSquared = dx * dx + dy * dy + dz * dz;
                    if (distanceSquared > 10000) continue;
                    const weight = Math.exp(-distanceSquared / denominator);
                    vx += weight * coefficients[i][0];
                    vy += weight * coefficients[i][1];
                    vz += weight * coefficients[i][2];
                }
                const medialWeight = smooth((Math.abs(p[0]) - 30) / 15);
                p[0] += medialWeight * vx;
                p[1] += medialWeight * vy;
                p[2] += medialWeight * vz;
            }
            if (f % 40 === 0) onProgress?.(f / frames.length);
        }
        return points;
    }
    return { move };
}
